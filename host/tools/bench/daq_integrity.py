"""Does the path carry every sample it was given, and stop the rest?

A tone or a ramp generated on the target, filtered by the host's chain,
decimated into the ring and read back. The host knows every output sample,
so a lost record shows as a phase step and nothing else does. No analog
path is involved.

    exact transport   a ramp, every record the exact integer it should be
    exact filter      the same ramp through the biquads, against float64
    in band           a tone the chain passes: whole amplitude, no phase step
    out of band       a tone above the output Nyquist: stopped, not folded

    python tools/bench/daq_integrity.py [--alias 100000 --rate 1000000]
"""
import argparse
import cmath
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial import Coaxial63100  # noqa: E402
from coaxial.acquire import bessel  # noqa: E402
from coaxial.errors import RigError  # noqa: E402


class Report:

    """A line per check; the verdict is the exit code."""

    def __init__(self):
        self.passed = self.failed = 0

    def check(self, what, ok, detail=''):
        self.passed += bool(ok)
        self.failed += not ok
        print('  %s  %-44s %s' % ('PASS' if ok else 'FAIL', what, detail))

    def verdict(self):
        print('\n%d passed, %d failed' % (self.passed, self.failed))
        return 1 if self.failed else 0


# -- the run ------------------------------------------------------------------

def run(daq, args, chain, **tone):
    """Chain loaded, generator on, one run collected; the path left clear."""
    layout = daq.configure(args.channels.split(','), chain=chain, digital=False)
    daq.tone(**tone)
    try:
        return layout, daq.collect(args.records, timeout=args.seconds)
    finally:
        daq.tone(0)
        daq.shape()


def ramp_run(daq, args, sections):
    """The ramp through `sections`, at a rate the link drains."""
    rate = int(args.exact_out * args.accumulate * args.decimate)
    chain = {'boxcar': args.accumulate, 'sections': sections, 'decimate': args.decimate}
    return run(daq, args, chain, hz=args.step, rate_hz=rate, amplitude=args.modulus,
               offset=0, kind=1)


def tone_run(daq, args, chain, hz):
    return run(daq, args, chain, hz=hz, rate_hz=args.rate, amplitude=args.amplitude,
               offset=args.offset)


# -- the arithmetic -------------------------------------------------------------

def fold(f, rate):
    """Where a tone lands after sampling at `rate`."""
    left = math.fmod(abs(f), rate)
    return rate - left if left > rate / 2.0 else left


def project(values, hz, rate):
    """The amplitude of `hz` in `values`."""
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
    """The tone's phase step between windows, and how many windows."""
    mean = sum(values) / len(values)
    phases = []
    for start in range(0, len(values) - window + 1, window):
        acc = sum((values[i] - mean) * cmath.exp(-2j * math.pi * hz * i / rate)
                  for i in range(start, start + window))
        if abs(acc) > 1e-9:
            phases.append(cmath.phase(acc))
    steps = [(b - a + math.pi) % (2.0 * math.pi) - math.pi for a, b in zip(phases, phases[1:])]
    return steps, len(phases)


def settle_records(chain):
    """Records the filter takes to settle."""
    return max(16, int(3.0 * chain['out_rate'] / chain['cutoff']))


def ramp_record(first_sample, n, step, modulus, offset):
    """The integer a record holds for `n` ramp samples from `first_sample`."""
    return sum(offset + ((first_sample + i) * step) % modulus for i in range(n))


def biquad_run(sections, values):
    """The cascade in float64, transposed DF2 - filter/src/filter.c's equation."""
    state = [[0.0, 0.0] for _ in sections]
    out = []
    for x in values:
        for i, (b0, b1, b2, a1, a2) in enumerate(sections):
            s1, s2 = state[i]
            y = b0 * x + s1
            state[i] = [b1 * x - a1 * y + s2, b2 * x - a2 * y]
            x = y
        out.append(x)
    return out


def means(layout, got, chain):
    """Each record's mean, after the filter settled."""
    name = layout['fields'][0]['signal']
    return [r[name] / max(1, r['samples']) for r in got.records[settle_records(chain):]]


def first_boxcar(layout, got, args):
    """Every boxcar the first record can have come from, and those that explain all records."""
    name = layout['fields'][0]['signal']
    n, dec = args.accumulate, args.decimate
    starts = [b for b in range(args.search)
              if ramp_record(b * n, n, args.step, args.modulus, 0) == got.records[0][name]]
    fits, best = [], None
    for base in starts:
        wrong = next(((k, r[name], want) for k, r in enumerate(got.records)
                      for want in [ramp_record((base + k * dec) * n, n, args.step,
                                               args.modulus, 0)] if r[name] != want), None)
        if wrong is None:
            fits.append(base)
        elif best is None or wrong[0] > best[0]:
            best = wrong
    return starts, fits, best


# -- the passes -----------------------------------------------------------------

def arrived(report, got, least):
    report.check('records arrived', len(got.records) > least,
                 '%d in %.2f s' % (len(got.records), got.seconds))
    return len(got.records) > least


def dropped_nothing(report, got):
    report.check('the ring dropped nothing', got.state['dropped'] == 0,
                 '%d dropped, peak %s of %s' % (got.state['dropped'], got.state.get('worst'),
                                                got.state.get('capacity')))


def exact_transport(daq, chain, args, report):
    """Every record, exactly."""
    print('\n-- exact: a ramp, %d summed, every %dth kept --' % (args.accumulate, args.decimate))
    layout, got = ramp_run(daq, args, ())
    if not arrived(report, got, 32):
        return
    dropped_nothing(report, got)
    starts, fits, best = first_boxcar(layout, got, args)
    report.check('EVERY record is the exact integer it should be', len(fits) >= 1,
                 '%d records, %d candidate start(s), %d explain all%s'
                 % (len(got.records), len(starts), len(fits),
                    '' if fits or not best else '; best diverges at %d: got %d want %d' % best))
    counts = sorted(set(r['samples'] for r in got.records))
    report.check('and carries the sample count that made it', counts == [args.accumulate],
                 'counts seen: %s' % counts)
    fields = [f['signal'] for f in layout['fields']]
    if len(fields) > 1:
        report.check('both fields of a record hold the same sample',
                     all(r[fields[1]] == r[fields[0]] for r in got.records),
                     '%s against %s over %d records' % (fields[1], fields[0], len(got.records)))


def exact_filter(daq, chain, args, report):
    """The biquads per sample, against the same arithmetic in float64."""
    print('\n-- exact: the same ramp through %d biquads --' % len(chain['sections']))
    layout, got = ramp_run(daq, args, chain['sections'])
    if not arrived(report, got, 32):
        return
    dropped_nothing(report, got)
    name = layout['fields'][0]['signal']
    n, dec = args.accumulate, args.decimate
    boxcars = [ramp_record(b * n, n, args.step, args.modulus, 0) / float(n)
               for b in range((len(got.records) + 2) * dec)]
    kept = biquad_run(chain['sections'], boxcars)
    errors = [abs(float(r[name]) / n - kept[(k + 1) * dec - 1]) for k, r in enumerate(got.records)]
    worst = max(errors)
    report.check('EVERY sample matches the same filter in double precision', worst < 0.05,
                 'worst %.4f codes of a %d-code ramp, at record %d of %d'
                 % (worst, args.modulus, errors.index(worst), len(got.records)))


def in_band(daq, chain, args, report):
    """A tone the chain passes: whole, and in step."""
    print('\n-- in band: %d Hz, which the chain is meant to pass --' % args.tone)
    layout, got = tone_run(daq, args, chain, args.tone)
    if not arrived(report, got, 64 + settle_records(chain)):
        return
    dropped_nothing(report, got)
    counts = sorted(set(r['samples'] for r in got.records))
    report.check('every record holds the same sample count', len(counts) == 1,
                 'counts seen: %s' % counts)

    values = means(layout, got, chain)
    lands = fold(args.tone, chain['out_rate'])
    amplitude = project(values, lands, chain['out_rate'])
    want = args.amplitude * bessel.chain_gain(chain, args.tone)
    report.check('the tone arrived at the amplitude the design predicted',
                 abs(amplitude - want) < max(0.2 * want, 2.0),
                 '%.1f codes at %.1f Hz, design says %.1f' % (amplitude, lands, want))

    window = max(16, int(2.0 * chain['out_rate'] / max(lands, 1.0)))
    if len(values) < 4 * window:
        print('  ....  too few records for a phase track (%d)' % len(values))
        return
    steps, windows = phase_steps(values, lands, chain['out_rate'], window)
    ranked = sorted((abs(s) for s in steps), reverse=True) or [0.0]
    per_record = 2.0 * math.pi * lands / chain['out_rate']        # one lost record's step
    report.check('the phase never jumped - nothing fell out of the stream',
                 ranked[0] < 0.35 * per_record,
                 'worst step %.4f rad, one lost record %.4f, over %d windows (next %s)'
                 % (ranked[0], per_record, windows, ', '.join('%.3f' % s for s in ranked[1:4])))


def out_of_band(daq, chain, args, report):
    """A tone above the output's Nyquist: stopped."""
    lands = fold(args.alias, chain['out_rate'])
    print('\n-- out of band: %d Hz, which would fold onto %.1f Hz --' % (args.alias, lands))
    layout, got = tone_run(daq, args, chain, args.alias)
    if not arrived(report, got, 64 + settle_records(chain)):
        return
    dropped_nothing(report, got)
    leaked = project(means(layout, got, chain), lands, chain['out_rate'])
    stopped = 20.0 * math.log10(max(leaked, 1e-9) / args.amplitude)
    predicted = 20.0 * math.log10(max(bessel.chain_gain(chain, args.alias), 1e-12))
    report.check('the fold was stopped, not passed on', leaked < args.amplitude * 0.02,
                 '%.2f codes of %d got through: %.1f dB' % (leaked, args.amplitude, stopped))
    report.check('stopped hard, not merely reduced', stopped < -40.0,
                 'chain predicts %.1f dB here; measured %.1f' % (predicted, stopped))


PASSES = (exact_transport, exact_filter, in_band, out_of_band)


# -- the run of runs ------------------------------------------------------------

def describe(rig, chain):
    print('\n-- the chain --')
    print('  link      %s' % rig.origin.label)
    print('  design    boxcar %d x decimate %d, %d biquads, cutoff %.1f Hz'
          % (chain['boxcar'], chain['decimate'], len(chain['sections']), chain['cutoff']))
    print('  rejects   %.1f dB of what would fold, %.3f samples of group delay ripple'
          % (chain['worst_alias_db'], chain['group_delay_samples']))
    print('  carries   %d input samples a record, %.1f records a second'
          % (chain['boxcar'] * chain['decimate'], chain['out_rate']))


def folded_onto(tone, chain):
    """The alias that lands exactly on `tone`: only the filter can tell them apart."""
    harmonic = int(chain['fs'] / 4.0 / chain['out_rate'])
    return int(harmonic * chain['out_rate']) + tone


def arguments(argv):
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('--port', default='COM4')
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--tone', type=int, default=61, help='in-band tone, Hz')
    p.add_argument('--alias', type=int, default=0,
                   help='out-of-band tone, Hz; 0 folds it onto --tone')
    p.add_argument('--rate', type=int, default=1000000, help='generator rate, Hz')
    p.add_argument('--out', type=float, default=500.0, help='records a second on the link')
    p.add_argument('--order', type=int, default=4)
    p.add_argument('--amplitude', type=int, default=12000)
    p.add_argument('--offset', type=int, default=32768)
    p.add_argument('--records', type=int, default=600)
    p.add_argument('--seconds', type=float, default=20.0)
    p.add_argument('--channels', default='Phase U,NTC')
    p.add_argument('--accumulate', type=int, default=64, help='ramp samples a record')
    p.add_argument('--decimate', type=int, default=4, help='one boxcar kept in this many')
    p.add_argument('--step', type=int, default=1, help='what the ramp adds a sample')
    p.add_argument('--modulus', type=int, default=4093,
                   help='the ramp period; prime, so one alignment fits')
    p.add_argument('--exact-out', type=float, default=120.0, dest='exact_out',
                   help='records a second in the exact passes, under what the link drains')
    p.add_argument('--search', type=int, default=512,
                   help='boxcars searched for the first record; under the ramp period')
    return p.parse_args(argv)


def main(argv=None):
    args = arguments(argv)
    chain = bessel.design(fs=float(args.rate), out_rate=float(args.out), order=args.order)
    args.alias = args.alias or folded_onto(args.tone, chain)
    report = Report()
    try:
        with Coaxial63100(port=args.port, power_afe=True,
                          simulated_device=args.simulated) as rig:
            describe(rig, chain)
            for check in PASSES:
                check(rig.daq, chain, args, report)
    except RigError as exc:
        report.check('the board answered throughout', False, str(exc))
    return report.verdict()


if __name__ == '__main__':
    sys.exit(main())
