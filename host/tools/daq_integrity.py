"""Does the whole path carry every sample it was given - and stop the rest?

A known tone is generated ON THE TARGET, filtered by the chain the host
designed, decimated into the ring, and read back over the link. The host
knows the frequency, the sample rate and the decimation, so it knows what
every output sample should be. That is what makes this a transfer test
rather than a measurement: a record that fell out of the ring, a block read
twice, a byte lost in a frame all show up as a phase that jumped, and
nothing else does.

Two passes, because the chain has two jobs:

  in band   a tone the filter is meant to pass. It must arrive with its
            amplitude and with a phase that never steps - that is the
            transfer being honest.
  out of    a tone above the output's Nyquist. It must NOT arrive. An
  band      unfiltered decimation would fold it into the passband as
            something that was never there, which is the whole reason the
            filter is in the path.

    cd host
    python tools/daq_integrity.py
    python tools/daq_integrity.py --alias 100000 --rate 1000000

Nothing analog is involved: with a tone on, the meter is not read at all.
The answer says whether the LINK and the RING are honest, not the front end.
"""
import argparse
import cmath
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100, bessel                   # noqa: E402
from coaxial.errors import RigError                        # noqa: E402


class Report:

    """What passed and what did not, a line each."""

    def __init__(self):
        self.passed = self.failed = 0

    def check(self, what, ok, detail=''):
        self.passed += bool(ok)
        self.failed += (not ok)
        print('  %s  %-44s %s' % ('PASS' if ok else 'FAIL', what, detail))


def fold(f, rate):
    """Where a tone lands after sampling at `rate`."""
    left = math.fmod(abs(f), rate)
    return rate - left if left > rate / 2.0 else left


def project(values, hz, rate):
    """The amplitude of `hz` in `values`, by projection.

    Peak to peak would do for a fat tone and lies about a thin one: a sine
    sampled eight times a cycle never lands on its own peak, and one that
    decimated onto DC is an offset rather than a ripple.
    """
    n = len(values)
    if n < 8:
        return 0.0
    mean = sum(values) / n
    if hz < 1e-9:
        return abs(mean)
    acc = sum((v - mean) * cmath.exp(-2j * math.pi * hz * i / rate)
              for i, v in enumerate(values))
    return 2.0 * abs(acc) / n


def phase_steps(values, hz, rate, window):
    """How far the tone's phase moved between windows.

    A clean transfer holds one phase for the whole run, because every sample
    is where the arithmetic says it is. A record lost or repeated moves
    everything after it, which is a step here and invisible in the values.
    """
    mean = sum(values) / len(values)
    phases = []
    for start in range(0, len(values) - window + 1, window):
        acc = sum((values[i] - mean) * cmath.exp(-2j * math.pi * hz * i / rate)
                  for i in range(start, start + window))
        if abs(acc) > 1e-9:
            phases.append(cmath.phase(acc))

    steps = []
    for before, after in zip(phases, phases[1:]):
        step = after - before
        while step > math.pi:
            step -= 2.0 * math.pi
        while step < -math.pi:
            step += 2.0 * math.pi
        steps.append(step)
    return steps, len(phases)


def settle_records(chain):
    """Records to throw away before judging: the filter's own settling.

    A Bessel started from rest meets a step - the tone's DC offset - and its
    answer to that is not the tone. Three time constants of the cutoff, in
    records. MEASURED: judging from the first record put a 0.30 rad step in
    the phase track where every other window sat at 0.016, and read 40 dB of
    a stopped alias that was really the transient.
    """
    return max(16, int(3.0 * chain['out_rate'] / chain['cutoff']))


def capture(device, chain, hz, args):
    """One pass: load the chain, run the tone, drain the ring."""
    daq = device.daq
    daq.shape()                        # cleared first - configure refuses a
    layout = daq.configure(args.channels.split(','),   # clock-closed record
                           accumulate=chain['boxcar'], digital=False)
    daq.shape(chain['sections'], chain['decimate'])
    daq.tone(hz=hz, rate_hz=args.rate, amplitude=args.amplitude,
             offset=args.offset)
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
    daq.tone(0)
    daq.shape()
    return layout, got, state, span


def pass_in_band(device, chain, args, report):
    """A tone the chain passes: it must arrive whole and in step."""
    hz = args.tone
    print('\n-- in band: %d Hz, which the chain is meant to pass --' % hz)
    layout, got, state, span = capture(device, chain, hz, args)
    name = layout['fields'][0]['signal']
    settle = settle_records(chain)

    report.check('records arrived',
                 len(got) > 64 + settle,
                 '%d in %.2f s, %.0f/s against %.0f asked for'
                 % (len(got), span, len(got) / max(span, 1e-6),
                    chain['out_rate']))
    if len(got) <= 64 + settle:
        return

    report.check('the ring dropped nothing', state['dropped'] == 0,
                 '%d dropped, peak %s of %s records held'
                 % (state['dropped'], state.get('worst'), state.get('capacity')))
    counts = sorted(set(r['samples'] for r in got))
    report.check('every record holds the same sample count', len(counts) == 1,
                 'counts seen: %s' % counts)

    values = [r[name] / max(1, r['samples']) for r in got[settle:]]
    lands = fold(hz, chain['out_rate'])
    got_amp = project(values, lands, chain['out_rate'])
    want = args.amplitude * bessel.chain_gain(chain, hz)

    report.check('the tone arrived at the amplitude the design predicted',
                 abs(got_amp - want) < max(0.2 * want, 2.0),
                 '%.1f codes at %.1f Hz, design says %.1f' % (got_amp, lands, want))

    window = max(16, int(2.0 * chain['out_rate'] / max(lands, 1.0)))
    if len(values) < 4 * window:
        print('  ....  too few records for a phase track (%d)' % len(values))
        return
    steps, windows = phase_steps(values, lands, chain['out_rate'], window)
    worst = max((abs(s) for s in steps), default=0.0)
    # One lost record moves every sample after it by a whole sample's worth
    # of phase. Anything smaller than a third of that is jitter, not a gap.
    per_record = 2.0 * math.pi * lands / chain['out_rate']
    ranked = sorted((abs(s) for s in steps), reverse=True)
    report.check('the phase never jumped - nothing fell out of the stream',
                 worst < 0.35 * per_record,
                 'worst step %.4f rad, one lost record would be %.4f, over '
                 '%d windows (next three %s)'
                 % (worst, per_record, windows,
                    ', '.join('%.3f' % s for s in ranked[1:4])))


def pass_out_of_band(device, chain, args, report):
    """A tone above the output's Nyquist: it must not come through."""
    hz = args.alias
    lands = fold(hz, chain['out_rate'])
    print('\n-- out of band: %d Hz, which would fold onto %.1f Hz --'
          % (hz, lands))
    layout, got, state, span = capture(device, chain, hz, args)
    name = layout['fields'][0]['signal']
    settle = settle_records(chain)

    report.check('records arrived', len(got) > 64 + settle,
                 '%d in %.2f s, %d dropped as the filter settled'
                 % (len(got), span, settle))
    if len(got) <= 64 + settle:
        return

    values = [r[name] / max(1, r['samples']) for r in got[settle:]]
    leaked = project(values, lands, chain['out_rate'])
    unfiltered = float(args.amplitude)
    stopped = 20.0 * math.log10(max(leaked, 1e-9) / unfiltered)

    report.check('the fold was stopped, not passed on',
                 leaked < unfiltered * 0.02,
                 '%.2f codes of %d got through: %.1f dB'
                 % (leaked, args.amplitude, stopped))
    report.check('stopped hard, not merely reduced', stopped < -40.0,
                 'chain predicts %.1f dB here; measured %.1f'
                 % (20.0 * math.log10(max(bessel.chain_gain(chain, hz), 1e-12)),
                    stopped))
    report.check('nothing was dropped doing it', state['dropped'] == 0,
                 '%d dropped' % state['dropped'])


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--port', default='COM4')
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--tone', type=int, default=61,
                   help='the in-band tone, Hz - it must survive the chain')
    p.add_argument('--alias', type=int, default=0,
                   help='the out-of-band tone, Hz - it must not. Default 0 '
                        'places it so it folds onto the in-band tone exactly: '
                        'same output frequency, one real and one an alias, '
                        'and the chain has to tell them apart')
    p.add_argument('--rate', type=int, default=1000000,
                   help='what the generator samples at, Hz')
    p.add_argument('--out', type=float, default=500.0,
                   help='records a second the link is to carry')
    p.add_argument('--order', type=int, default=4)
    p.add_argument('--amplitude', type=int, default=12000)
    p.add_argument('--offset', type=int, default=32768)
    p.add_argument('--records', type=int, default=600)
    p.add_argument('--seconds', type=float, default=20.0)
    p.add_argument('--channels', default='Phase U,NTC')
    args = p.parse_args(argv)

    device = Coaxial63100(port=args.port, power_afe=True,
                          simulated_device=bool(args.simulated))
    daq = device.daq
    try:
        daq.open()
    except RigError as exc:
        print('  FAIL  could not open the board: %s' % exc)
        return 1

    chain = bessel.design(fs=float(args.rate), out_rate=float(args.out),
                          order=args.order)
    if not args.alias:
        # Folded onto the in-band tone's own frequency: the strongest form
        # of the question, because the two are then indistinguishable in the
        # record and only the filter can have stopped one.
        harmonic = int(chain['fs'] / 4.0 / chain['out_rate'])
        args.alias = int(harmonic * chain['out_rate']) + args.tone
    report = Report()
    print('\n-- the chain --')
    print('  link      %s' % device.origin.label)
    print('  design    boxcar %d x decimate %d, %d biquads, cutoff %.1f Hz'
          % (chain['boxcar'], chain['decimate'], len(chain['sections']),
             chain['cutoff']))
    print('  rejects   %.1f dB of what would fold, %.3f samples of group '
          'delay ripple' % (chain['worst_alias_db'],
                            chain['group_delay_samples']))
    print('  carries   %d input samples a record, %.1f records a second'
          % (chain['boxcar'] * chain['decimate'], chain['out_rate']))

    try:
        pass_in_band(device, chain, args, report)
        pass_out_of_band(device, chain, args, report)
    except RigError as exc:
        report.check('the board answered throughout', False, str(exc))
    finally:
        try:
            device.daq.tone(0)
            device.daq.shape()
        except RigError:
            pass
        device.close()

    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
