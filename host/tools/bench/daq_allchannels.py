"""Every channel the board has, through the filter chain, over the link.

More channels is a longer record, fewer records a second on the same link,
and a lower cutoff; the board's `max_rate_hz` for the stride says by how
much. Digital pins ride the same chain and come out as a duty (KEEPALIVE
toggles at ~100 kHz; one sample decimated by 2000 is a coin toss).

    python tools/bench/daq_allchannels.py            # every channel
    python tools/bench/daq_allchannels.py --sweep    # what each count costs
"""
import argparse
import sys

from coaxial import Coaxial63100
from coaxial.acquire import bessel
from coaxial.devices import scaling
from coaxial.errors import RigError


def plan_for(daq, names, order):
    """The chain for `names`: the measured sweep rate against what the link carries."""
    fs = daq.sweep_rate(names)
    layout = daq.configure(names, accumulate=1, digital=True)
    carries = daq.state()['max_rate_hz']
    chain = dict(bessel.for_link(fs=fs, max_rate_hz=carries, order=order), sweeps=fs)
    return layout, carries, chain


def sweep(daq, order):
    print('\n%-3s %6s %8s %7s %11s %7s %9s %9s'
          % ('ch', 'stride', 'sweeps/s', 'link/s', 'boxcar x d', 'out/s', 'cutoff', 'alias'))
    every = daq.channels()
    for count in range(1, len(every) + 1):
        layout, carries, chain = plan_for(daq, every[:count], order)
        print('%-3d %6d %8.0f %7d %6d x %-4d %7.1f %6.1f Hz %6.1f dB'
              % (count, layout['stride'], chain['sweeps'], carries, chain['boxcar'],
                 chain['decimate'], chain['out_rate'], chain['cutoff'], chain['worst_alias_db']))


def describe(layout, carries, chain):
    print('\n-- %d channels, %d pins --' % (len(layout['fields']), len(layout['pins'])))
    print('  record    %d bytes; the link carries %d a second' % (layout['stride'], carries))
    print('  converter %.0f sweeps a second, measured' % chain['sweeps'])
    print('  chain     boxcar %d x decimate %d, %d biquads'
          % (chain['boxcar'], chain['decimate'], len(chain['sections'])))
    print('  cutoff    %.1f Hz, a fifth of %.1f records a second'
          % (chain['cutoff'], chain['out_rate']))
    print('  rejects   %.1f dB of what would fold' % chain['worst_alias_db'])


def means(rig, layout, run):
    """Each channel's mean over the run, in codes and in its unit."""
    params = rig.board.analog.scaling()
    print('\n%-10s %14s %12s' % ('channel', 'mean', 'in its unit'))
    for field in layout['fields']:
        name = field['signal']
        mean = sum(r[name] for r in run.records) / float(sum(max(1, r['samples'])
                                                             for r in run.records))
        convert = scaling.converter(field['unit'], field['differential'], signal=name,
                                    params=params)
        print('%-10s %14.1f %9.3f %-3s'
              % (name, mean, convert(mean), scaling.symbol(field['unit'], name)))


def duties(layout, run):
    if layout['pins']:
        print('\n%-14s %10s' % ('pin', 'duty'))
    for pin in layout['pins']:
        name = pin['signal']
        duty = sum(r['digital'][name] for r in run.records) / len(run.records)
        print('%-14s %9.1f %%' % (name, duty * 100.0))


def arguments(argv):
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('--port', default='COM4')
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--channels', default='all')
    p.add_argument('--order', type=int, default=4)
    p.add_argument('--records', type=int, default=200)
    p.add_argument('--seconds', type=float, default=15.0)
    p.add_argument('--sweep', action='store_true', help='what each channel count costs')
    return p.parse_args(argv)


def main(argv=None):
    args = arguments(argv)
    try:
        with Coaxial63100(port=args.port, power_afe=True,
                          simulated=args.simulated) as rig:
            print('link: %s' % rig.origin.label)
            daq = rig.daq
            if args.sweep:
                return sweep(daq, args.order)
            names = daq.channels() if args.channels == 'all' else args.channels.split(',')

            layout, carries, chain = plan_for(daq, names, args.order)
            describe(layout, carries, chain)
            layout = daq.configure(names, chain=chain, digital=True)
            run = daq.collect(args.records, timeout=args.seconds)
            daq.shape()

            if not run.records:
                print('  nothing came back in %.1f s' % run.seconds)
                return 1
            print('\n  %d records in %.2f s (%.0f/s), %d dropped, peak %s of %s'
                  % (len(run.records), run.seconds, len(run.records) / run.seconds,
                     run.state['dropped'], run.state.get('worst'), run.state.get('capacity')))
            means(rig, layout, run)
            duties(layout, run)
            return 0
    except RigError as exc:
        print('  the board refused: %s' % exc)
        return 1


if __name__ == '__main__':
    sys.exit(main())
