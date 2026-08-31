"""Every channel the board has, through the filter chain, over the link.

The point the numbers make: **more channels is a lower cutoff, and the
board is what says by how much.** A record grows with every channel added,
the link carries a fixed number of bytes a second, so the records a second
falls - and the passband falls with it. Nothing here chooses that; it is
read off the board's own `max_rate_hz` for the stride it actually has.

The digital pins ride the same chain and come out as a DUTY, because a
level sampled once and decimated by two thousand is aliased by
construction: KEEPALIVE toggles at about 100 kHz and would read as a coin
toss.

    cd host
    python tools/daq_allchannels.py                 # every channel
    python tools/daq_allchannels.py --sweep         # what each count costs
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100, bessel, scaling            # noqa: E402
from coaxial.errors import RigError                          # noqa: E402


def sweep_rate(daq, names, records=300, timeout=6.0):
    """What the poll loop actually manages, in sweeps a second.

    A FINITE RUN, and that is the whole trick. Asked for no rate with
    `records=0` the board substitutes what the LINK carries and gates the
    triggers to it - so a free-running measurement at accumulate 1 reads
    the link, not the loop: 279 sweeps a second measured, where a finite
    burst gives thousands. A run that ends is left alone, because it ends.

    Measured, not assumed: the converter could do megasamples and the main
    loop reads one channel a turn, so what a task really gets is this - and
    it falls with every channel added, a sweep being longer.
    """
    daq.shape()
    daq.configure(names, accumulate=1, digital=True, records=records,
                  interval_us=0)
    began = time.time()
    daq.start()
    while time.time() - began < timeout:
        if daq.state()['done']:
            break
        time.sleep(0.005)
    span = time.time() - began
    state = daq.state()
    daq.stop()
    made = state['produced'] + state['dropped']
    return made / max(span, 1e-6)


def plan_for(daq, names, order):
    """Configure for `names`, measure what the board really gives, design.

    Two configures, and the first is not waste: the stride and the sweep
    rate are both facts of the channel list, and the board is the only
    thing that knows either, so the chain cannot be designed until the task
    exists to be measured.
    """
    fs = sweep_rate(daq, names)
    daq.shape()
    layout = daq.configure(names, accumulate=1, digital=True)
    carries = daq.state()['max_rate_hz']
    chain = bessel.for_link(fs=fs, max_rate_hz=carries, order=order)
    chain['sweeps'] = fs
    return layout, carries, chain


def sweep(daq, every, order):
    """What each channel count costs: the loop's rate, the link's, and the
    cutoff and rejection that fall out of the two."""
    print('\n%-3s %6s %8s %7s %11s %7s %9s %9s'
          % ('ch', 'stride', 'sweeps/s', 'link/s', 'boxcar x d', 'out/s',
             'cutoff', 'alias'))
    for count in range(1, len(every) + 1):
        layout, carries, chain = plan_for(daq, every[:count], order)
        print('%-3d %6d %8.0f %7d %6d x %-4d %7.1f %6.1f Hz %6.1f dB'
              % (count, layout['stride'], chain['sweeps'], carries,
                 chain['boxcar'], chain['decimate'], chain['out_rate'],
                 chain['cutoff'], chain['worst_alias_db']))


def run(device, args):
    """Every channel at once, and what came back."""
    daq = device.daq
    every = daq.channels()
    names = every if args.channels == 'all' else args.channels.split(',')

    if args.sweep:
        sweep(daq, every, args.order)
        return 0

    layout, carries, chain = plan_for(daq, names, args.order)
    print('\n-- %d channels, %d pins --' % (len(layout['fields']),
                                            len(layout['pins'])))
    print('  record    %d bytes; the link carries %d of them a second'
          % (layout['stride'], carries))
    print('  converter %.0f sweeps a second, measured - one channel a turn '
          'of the main loop' % chain['sweeps'])
    print('  chain     boxcar %d x decimate %d, %d biquads'
          % (chain['boxcar'], chain['decimate'], len(chain['sections'])))
    print('  cutoff    %.1f Hz - a fifth of the %.1f records a second this '
          'many channels leave' % (chain['cutoff'], chain['out_rate']))
    print('  rejects   %.1f dB of what would fold' % chain['worst_alias_db'])

    # And now for real: the accumulate the chain wants, the sections, and a
    # rate under what the link drains so the ring is never the story.
    daq.shape()
    daq.configure(names, accumulate=chain['boxcar'], digital=True)
    daq.shape(chain['sections'], chain['decimate'])
    daq.start()

    got, began = [], time.time()
    while len(got) < args.records and time.time() - began < args.seconds:
        block = daq.acquire()
        if block:
            got.extend(block)
        else:
            time.sleep(0.002)
    span = time.time() - began
    state = daq.state()
    daq.stop()
    daq.shape()

    if not got:
        print('  nothing came back in %.1f s' % span)
        return 1

    print('\n  %d records in %.2f s (%.0f/s), %d dropped, peak %s of %s'
          % (len(got), span, len(got) / max(span, 1e-6), state['dropped'],
             state.get('worst'), state.get('capacity')))

    params = device.board.analog.scaling()
    print('\n%-10s %14s %12s' % ('channel', 'mean', 'in its unit'))
    for field in layout['fields']:
        name = field['signal']
        total = sum(r[name] for r in got)
        count = sum(max(1, r['samples']) for r in got)
        mean = total / float(count)
        convert = scaling.converter(field['unit'], field['differential'],
                                    signal=name, params=params)
        print('%-10s %14.1f %9.3f %-3s'
              % (name, mean, convert(mean),
                 scaling.symbol(field['unit'], name)))

    if layout['pins']:
        print('\n%-14s %10s' % ('pin', 'duty'))
        for pin in layout['pins']:
            name = pin['signal']
            duty = sum(r['digital'][name] for r in got) / len(got)
            print('%-14s %9.1f %%' % (name, duty * 100.0))
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--port', default='COM4')
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--channels', default='all')
    p.add_argument('--order', type=int, default=4)
    p.add_argument('--records', type=int, default=200)
    p.add_argument('--seconds', type=float, default=15.0)
    p.add_argument('--sweep', action='store_true',
                   help='what each channel count costs, and nothing else')
    args = p.parse_args(argv)

    device = Coaxial63100(port=args.port, power_afe=True,
                          simulated_device=bool(args.simulated))
    daq = device.daq
    try:
        daq.open()
    except RigError as exc:
        print('  could not open the board: %s' % exc)
        return 1
    print('link: %s' % device.origin.label)
    try:
        return run(device, args)
    except RigError as exc:
        print('  the board refused: %s' % exc)
        return 1
    finally:
        try:
            daq.shape()
        except RigError:
            pass
        device.close()


if __name__ == '__main__':
    sys.exit(main())
