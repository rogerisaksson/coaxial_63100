"""The anti-alias chain, run for real and judged against the arithmetic.

`Filter/` is hardware-free like the Modbus core, and for the same reason: a
decimating filter is either right or it quietly folds a tone onto the answer,
and that can be settled on a desk. This builds it with the host gcc, drives
it through ctypes, and checks it against `coaxial.bessel` - which is the
design that WROTE the coefficients, so agreement is not circular in the part
that matters: the C is checked against the transfer function evaluated
independently, and the aliasing is checked by feeding a real tone through
and measuring what came out.

    cd host && python tests/test_filter_core.py
"""
import cmath
import ctypes
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

from test_modbus_core import Report, build, find_cc          # noqa: E402
from coaxial import bessel                                    # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
FILTER = os.path.join(REPO, 'Filter')
SOURCES = [os.path.join(FILTER, 'test', 'harness.c'),
           os.path.join(FILTER, 'Src', 'filter.c')]


class Chain:

    """One filter behind the harness, designed and driven from here."""

    def __init__(self, lib):
        lib.flt_new.restype = ctypes.c_void_p
        lib.flt_run.restype = ctypes.c_uint32
        lib.flt_ratio.restype = ctypes.c_uint32
        lib.flt_design.restype = ctypes.c_int
        self.lib = lib
        self.h = ctypes.c_void_p(lib.flt_new())

    def design(self, boxcar, decimate, sections):
        flat = bessel.flat(sections)
        buf = (ctypes.c_float * max(1, len(flat)))(*flat)
        return bool(self.lib.flt_design(self.h, ctypes.c_uint16(boxcar),
                                        ctypes.c_uint16(decimate),
                                        ctypes.c_uint8(len(sections)), buf))

    def load(self, chain):
        return self.design(chain['boxcar'], chain['decimate'],
                           chain['sections'])

    def reset(self):
        self.lib.flt_reset(self.h)

    def ratio(self):
        return self.lib.flt_ratio(self.h)

    def run(self, samples):
        n = len(samples)
        buf = (ctypes.c_int32 * n)(*[int(round(s)) for s in samples])
        out = (ctypes.c_float * (n + 1))()
        made = self.lib.flt_run(self.h, buf, ctypes.c_uint32(n), out)
        return [out[i] for i in range(made)]


def tone(hz, fs, n, amplitude=10000.0, offset=0.0):
    """`n` samples of a sine, as the converter would hand them over."""
    return [offset + amplitude * math.sin(2.0 * math.pi * hz * i / fs)
            for i in range(n)]


def amplitude_at(values, hz, fs, drop=0.5):
    """The amplitude of `hz` in the settled tail, by projection.

    NOT peak-to-peak, which was the first instrument here and was wrong
    twice over: a sine sampled five times a cycle never lands on its own
    peak, so it read 7 % low at 2 kHz of a 10 kHz stream - and a tone that
    decimates onto DC is a constant offset, which a peak-to-peak sees as
    nothing at all. Projecting onto the frequency being asked about has
    neither problem. Hann-windowed against leakage, and its coherent gain
    divided back out.
    """
    tail = values[int(len(values) * drop):]
    n = len(tail)
    if n < 8:
        return 0.0
    window = [0.5 - 0.5 * math.cos(2.0 * math.pi * i / n) for i in range(n)]
    norm = sum(window)

    if hz < 1e-9:                       # folded onto DC: a bare offset
        return abs(sum(w * v for w, v in zip(window, tail)) / norm)
    if abs(hz - fs / 2.0) < 1e-9:       # and onto Nyquist: an alternation
        return abs(sum(w * v * (-1) ** i
                       for i, (w, v) in enumerate(zip(window, tail))) / norm)

    acc = sum(w * v * cmath.exp(-2j * math.pi * hz * i / fs)
              for i, (w, v) in enumerate(zip(window, tail)))
    return 2.0 * abs(acc) / norm


# ---- the design, on its own -------------------------------------------------

def test_the_design(report, _lib):
    """What `coaxial.bessel` computes, against values that are known."""
    report.check('the reverse Bessel polynomial is exact',
                 bessel.reverse_bessel(2) == [3, 3, 1] and
                 bessel.reverse_bessel(3) == [15, 15, 6, 1],
                 'n=2 [3,3,1], n=3 [15,15,6,1]')

    poles = bessel.prototype(2)
    got = sorted(poles, key=lambda p: p.imag)[-1]
    report.check('the second-order poles are the tabulated ones',
                 abs(got.real + 1.1016) < 2e-3 and abs(got.imag - 0.6360) < 2e-3,
                 '%.4f%+.4fj against -1.1016+0.6360j' % (got.real, got.imag))

    for order in (1, 2, 3, 4, 6, 8):
        s = bessel.sections(10000.0, 400.0, order)
        dc = abs(bessel.response(s, 0.0, 10000.0))
        cut = 20.0 * math.log10(abs(bessel.response(s, 400.0, 10000.0)))
        report.check('order %d passes DC unchanged and is -3 dB at the cutoff'
                     % order,
                     abs(dc - 1.0) < 1e-9 and abs(cut + 3.01) < 0.05,
                     'DC %.9f, fc %.3f dB, %d sections' % (dc, cut, len(s)))

    # The pre-warp earns its place here: without it the digital cutoff lands
    # low by tan(x)/x, which at a tenth of the rate is already 3 %.
    s = bessel.sections(1000.0, 100.0, 4)
    cut = 20.0 * math.log10(abs(bessel.response(s, 100.0, 1000.0)))
    report.check('the pre-warp puts -3 dB where it was asked for, not near it',
                 abs(cut + 3.01) < 0.05, '%.3f dB at fc/fs = 0.1' % cut)


def test_bessel_buys_group_delay(report, _lib):
    """The reason it is a Bessel and not something sharper."""
    chain = bessel.design(fs=3.75e6, out_rate=1000.0, order=4)
    report.check('the group delay is flat across the passband',
                 chain['group_delay_samples'] < 0.05,
                 '%.4f output samples peak to peak'
                 % chain['group_delay_samples'])

    # And what it costs, stated rather than hidden: order steepens the far
    # stopband and leaves the knee alone, which is the whole reason the
    # default cutoff is a fifth of the output rate and not the half the
    # sampling theorem would allow.
    mid, fc = 10000.0, 400.0
    knee = [20.0 * math.log10(abs(bessel.response(
        bessel.sections(mid, fc, n), 1.5 * fc, mid))) for n in (2, 8)]
    far = [20.0 * math.log10(abs(bessel.response(
        bessel.sections(mid, fc, n), 4.0 * fc, mid))) for n in (2, 8)]
    report.check('order steepens the far stopband, not the knee',
                 abs(knee[1] - knee[0]) < 3.0 and (far[0] - far[1]) > 25.0,
                 '1.5fc %.1f -> %.1f dB, 4fc %.1f -> %.1f dB'
                 % (knee[0], knee[1], far[0], far[1]))


def test_the_default_is_honest(report, _lib):
    """The default cutoff is the one that actually stops the fold."""
    loose = bessel.design(fs=3.75e6, out_rate=1000.0, order=4, cutoff=400.0)
    tight = bessel.design(fs=3.75e6, out_rate=1000.0, order=4)
    report.check('the default rejects what a half-Nyquist passband does not',
                 tight['worst_alias_db'] < loose['worst_alias_db'] - 20.0,
                 'cutoff 400 Hz %.1f dB, default %.0f Hz %.1f dB'
                 % (loose['worst_alias_db'], tight['cutoff'],
                    tight['worst_alias_db']))
    report.check('the chain reports what it fails to stop, in dB',
                 -60.0 < tight['worst_alias_db'] < -20.0,
                 '%.2f dB worst fold into the passband'
                 % tight['worst_alias_db'])


# ---- the C, against the arithmetic -------------------------------------------

def test_pass_through(report, lib):
    """A design that changes nothing hands every sample straight back."""
    c = Chain(lib)
    report.check('a pass-through design is valid', c.design(1, 1, []))
    report.check('its ratio is one', c.ratio() == 1, 'ratio %d' % c.ratio())
    got = c.run([5, -7, 9, 0])
    report.check('and every sample comes out unchanged',
                 got == [5.0, -7.0, 9.0, 0.0], '%s' % (got,))


def test_the_boxcar_averages(report, lib):
    """Stage one: sum and dump, divided at the dump."""
    c = Chain(lib)
    c.design(4, 1, [])
    got = c.run([10, 20, 30, 40, 1, 1, 1, 1])
    report.check('a boxcar of four emits the mean of each four',
                 got == [25.0, 1.0], '%s' % (got,))
    report.check('and its ratio is the boxcar', c.ratio() == 4)

    c.reset()
    got = c.run([10, 20, 30])
    report.check('a part-filled boxcar emits nothing', got == [], '%s' % (got,))


def test_decimation_counts(report, lib):
    """Stage three throws away only what stage two has already shaped."""
    c = Chain(lib)
    c.design(2, 3, [])
    got = c.run(list(range(60)))
    report.check('boxcar 2 then decimate 3 is one output in six',
                 len(got) == 10, '%d outputs from 60 samples' % len(got))
    report.check('the ratio is the product', c.ratio() == 6,
                 'ratio %d' % c.ratio())
    # The first output is the mean of samples 4 and 5: two boxcars are
    # swallowed by the decimation before one is let through.
    report.check('and it keeps the LAST of each group, not the first',
                 abs(got[0] - 4.5) < 1e-6, 'first output %.3f' % got[0])


def test_the_c_matches_the_transfer_function(report, lib):
    """The cascade in C against the transfer function it was designed from.

    A tone in, the settled amplitude out, against |H| evaluated in Python.
    Not the same arithmetic twice: one is a difference equation run sample
    by sample, the other is the response of the filter it came from.
    """
    mid = 10000.0
    sects = bessel.sections(mid, 500.0, 4)
    c = Chain(lib)
    report.check('a four-section design loads', c.design(1, 1, sects))

    for hz in (50.0, 250.0, 500.0, 1000.0, 2000.0):
        c.reset()
        out = c.run(tone(hz, mid, 8000))
        got = amplitude_at(out, hz, mid) / 10000.0
        want = abs(bessel.response(sects, hz, mid))
        report.check('at %6.0f Hz the C matches |H| to 1 %%' % hz,
                     abs(got - want) < max(0.01 * want, 1e-4),
                     'ran %.5f, computed %.5f' % (got, want))


def test_dc_survives_the_chain(report, lib):
    """A steady reading must come out itself: an anti-alias filter that
    moved the mean would be a calibration error nobody looked for."""
    chain = bessel.design(fs=100000.0, out_rate=1000.0, order=4)
    c = Chain(lib)
    c.load(chain)
    out = c.run([32768] * 20000)
    settled = out[len(out) // 2:]
    worst = max(abs(v - 32768.0) for v in settled) if settled else 1e9
    report.check('DC comes through the whole chain unchanged',
                 worst < 0.5, 'worst %.4f codes off 32768 over %d outputs'
                 % (worst, len(settled)))


def test_a_tone_that_would_alias_is_stopped(report, lib):
    """THE POINT OF ALL OF IT.

    A tone above half the output rate has nowhere to go but on top of the
    answer. This feeds one through the real chain and measures what came out
    against what the design said would.
    """
    chain = bessel.design(fs=100000.0, out_rate=1000.0, order=4)
    c = Chain(lib)
    c.load(chain)

    for hz in (800.0, 1200.0, 2000.0, 5000.0):
        c.reset()
        out = c.run(tone(hz, chain['fs'], 200000))
        # WHERE IT LANDS is the whole point: decimation does not move the
        # amplitude, it moves the frequency, and 2 kHz off a 1 kHz output
        # rate arrives as a DC offset rather than a tone.
        lands = bessel._fold(hz, chain['out_rate'])
        got = amplitude_at(out, lands, chain['out_rate']) / 10000.0
        want = bessel.chain_gain(chain, hz)
        report.check('%5.0f Hz folds in at the level the design predicted'
                     % hz,
                     abs(got - want) < max(0.25 * want, 2e-4),
                     'ran %.6f (%.1f dB) at %.0f Hz, computed %.6f'
                     % (got, 20.0 * math.log10(max(got, 1e-12)), lands, want))

    # And the passband is not damaged by any of it.
    c.reset()
    out = c.run(tone(50.0, chain['fs'], 200000))
    got = amplitude_at(out, 50.0, chain['out_rate']) / 10000.0
    want = bessel.chain_gain(chain, 50.0)
    report.check('while 50 Hz passes through it',
                 abs(got - want) < 0.01, '%.4f of the input, computed %.4f'
                 % (got, want))


def test_reset_forgets(report, lib):
    """State carried across a reconfigure is a transient nothing explains."""
    mid = 10000.0
    c = Chain(lib)
    c.design(1, 1, bessel.sections(mid, 500.0, 4))
    c.run([20000] * 500)
    hot = c.run([0] * 20)

    c.reset()
    cold = c.run([0] * 20)
    report.check('a filter that ran carries its state into the next sample',
                 abs(hot[0]) > 1.0, 'first output after a step %.2f' % hot[0])
    report.check('and reset clears it', abs(cold[0]) < 1e-6,
                 'first output after reset %.6g' % cold[0])


def test_refusals(report, lib):
    """What the design will not pretend to do."""
    c = Chain(lib)
    report.check('a zero boxcar is refused', not c.design(0, 1, []))
    report.check('a zero decimation is refused', not c.design(1, 0, []))

    for bad, why in (((1000.0, 600.0, 4), 'a cutoff above the Nyquist'),
                     ((1000.0, 0.0, 4), 'a cutoff of nothing')):
        try:
            bessel.sections(*bad)
            report.check('%s is refused' % why, False, 'it was accepted')
        except ValueError as exc:
            report.check('%s is refused, and says so' % why, True,
                         str(exc)[:60])

    try:
        bessel.design(fs=1000.0, out_rate=1000.0, order=9)
        report.check('an order past four biquads is refused', False)
    except ValueError as exc:
        report.check('an order past four biquads is refused', True,
                     str(exc)[:60])


ROSTER = (test_the_design, test_bessel_buys_group_delay,
          test_the_default_is_honest, test_pass_through,
          test_the_boxcar_averages, test_decimation_counts,
          test_the_c_matches_the_transfer_function, test_dc_survives_the_chain,
          test_a_tone_that_would_alias_is_stopped, test_reset_forgets,
          test_refusals)


def main():
    cc = find_cc()
    if cc is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    lib_path, warnings = build(cc, SOURCES, [os.path.join(FILTER, 'Inc')],
                               name='filtercore')
    lib = ctypes.CDLL(lib_path)
    report = Report()
    report.check('Filter/ builds warning-free with the firmware flags',
                 not warnings, '; '.join(warnings[:3]))
    for test in ROSTER:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, lib)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
