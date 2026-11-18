#!/usr/bin/env python3
"""The robot's own elementary functions (machine.libm, wbc_libm.h): one set of bits, under one ulp.

What the robot's code calls (`machine.libm`: a number or an array through an extension of this interpreter, the
numpy twin where none is built) gives the twin's bits by every way in: floats one at a time over every draw, arrays
whole, of every length to 20, strided, of two dimensions, numpy's scalars, ints, an array with a number, two to
broadcast; and an array the robot's tracer follows takes each call as a statement of its own, wbc_libm.h's in the C
it writes, held bit for bit (`tools.cores.held`).

The same bits in C as in Python on every machine, under one unit in the last place, and IEEE 754's and C11 Annex
F's every special value. Each of sin, cos, tan, exp, expm1, tanh, log, arctan, arcsin, arccos, arctan2, hypot and power: its C
over arrays (wbc_libm.c),
its C a call at a time as another core inlines it, and its numpy twin, bit for bit over draws across its range and
where it is hard (near multiples of pi/2, past 2^22 up to the largest double, exp's edges, log near 1, subnormals);
the draws' digest pinned, so that a machine whose bits part fails here; its error against the function to 60
digits (tools.cores.libm.reference) under one unit in the last place; zeros, infinities and NaNs as Annex F's
tables have them; the coefficients and constants as their tool lays them; odd functions odd and even ones even, to the bit; arrays of every length through their tails and
chunks the same as the scalar calls. The pace beside numpy's: tools/cores/libm_bench.py."""
import hashlib
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report
from libm_laid import ANNEX, EXACT

SEED = 63100
#: The draws' digest: the twin's bits over every draw, NaNs as one NaN - laid on Windows (UCRT) and held by the
#: Linux runner's containers (glibc) alike; a change of a coefficient or of an order lays it anew.
DIGEST = 'f9e56defe8d1fa3c175a87d2a414cce4eeb8b03c3a143586a845a8e3cf96e9c6'
#: Points a function's error is read at against its 60 digits (the Decimal reference takes milliseconds a point).
READ = 120
INF, NAN = math.inf, math.nan
#: Kahan and McDonald's hardest double to reduce by pi/2 (its rest about 2^-60 of it).
HARDEST = 6381956970095103.0 * 2.0 ** 797


def _mags(rng, lo, hi, n):
    """n magnitudes 2^lo to 2^hi, each exact on every machine: a significand in [1, 2) scaled by a power of two (no
    platform's pow, whose last bit parts the hosts), subnormals as whole steps of the least."""
    sub = rng.integers(1, 1 << 52, n).astype(np.float64) * 5e-324
    if hi <= -1021:
        return sub
    out = np.ldexp(rng.uniform(1.0, 2.0, n), rng.integers(max(lo, -1021), hi, n))
    return np.where((np.arange(n) % 7 == 0) & (lo < -1021), sub, out)


def _draws(name, rng):
    """A function's draws: across its range, and where it is hard."""
    u, e = rng.uniform, lambda lo, hi, n: _mags(rng, lo, hi, n)
    big = np.concatenate([e(17, 1023, 600), -e(17, 1023, 200), [HARDEST, -HARDEST, 2.0 ** 22, 2.0 ** 22 - 0.5, 1.7976931348623157e308]])
    k = np.round(u(-2e5, 2e5, 800))
    near = np.concatenate([k * (math.pi / 2), np.nextafter(k * (math.pi / 2), INF), np.nextafter(k * (math.pi / 2), -INF)])
    pick = {
        'trig': np.concatenate([u(-7, 7, 6000), u(-1e5, 1e5, 2000), near, big, e(-1074, -17, 300), -e(-1074, -17, 300)]),
        'exp': np.concatenate([u(-746, 710, 6000), u(-1, 1, 2000), e(-1074, -3, 500), -e(-1074, -3, 500), u(-745.2, -708, 500), u(709, 709.8, 300)]),
        'expm1': np.concatenate([u(-40, 710, 4000), u(-2, 2, 4000), e(-1074, -3, 600), -e(-1074, -3, 600), u(-60, -30, 200)]),
        'log': np.concatenate([e(-1074, 1023, 5000), u(0.5, 2, 3000), 1.0 + u(-1e-2, 1e-2, 1000), 1.0 + e(-53, -26, 300),
                               1.0 - e(-53, -26, 300), e(-1074, -1022, 300)]),
        'tanh': np.concatenate([u(-22.5, 22.5, 5000), u(-1, 1, 3000), e(-1074, -3, 500), -e(-1074, -3, 500), u(18, 23, 300),
                                -u(18, 23, 300)]),
        'arctan': np.concatenate([u(-4, 4, 6000), e(-1074, 1023, 1500), -e(-1074, 1023, 1500), u(0.43, 0.45, 300), u(0.68, 0.69, 300)]),
        'arcsin': np.concatenate([u(-1, 1, 6000), 1.0 - e(-55, -3, 800), -1.0 + e(-55, -3, 800), e(-1074, -3, 500), u(0.97, 0.98, 300),
                                  u(0.49, 0.51, 300)]),
    }
    return pick['trig' if name in ('sin', 'cos', 'tan') else ('arcsin' if name == 'arccos' else name)]


SPECIAL = np.array([0.0, -0.0, INF, -INF, NAN, 1.0, -1.0, 5e-324, -5e-324, 2.2250738585072014e-308, 1.7976931348623157e308,
                    -1.7976931348623157e308, 0.5, -0.5, 2.0])


def _cases():
    """{function: its first argument (and its second)}: the draws and the specials."""
    from tools.cores import libm_c
    rng = np.random.default_rng(SEED)
    out: dict = {name: (np.concatenate([_draws(name, rng), SPECIAL]), None) for name in libm_c.ONE}
    y = np.concatenate([rng.uniform(-10, 10, 4000), _mags(rng, -1074, 1023, 1000) * rng.choice([-1.0, 1.0], 1000)])
    x = np.concatenate([rng.uniform(-10, 10, 4000), _mags(rng, -1074, 1023, 1000) * rng.choice([-1.0, 1.0], 1000)])
    y, x = np.concatenate([y, np.repeat(SPECIAL, SPECIAL.size)]), np.concatenate([x, np.tile(SPECIAL, SPECIAL.size)])
    for name in libm_c.TWO:
        out[name] = (y, x)
    m, u, n, sign = (lambda lo, hi, k: _mags(rng, lo, hi, k)), rng.uniform, 600, lambda k: rng.choice([-1.0, 1.0], k)
    base = [m(-30, 30, 5 * n), u(0.5, 2.0, 3 * n), 1.0 + m(-53, -20, n) * sign(n), np.full(2 * n, 10.0), u(0.0, 30.0, 2 * n),
            -u(0.0, 30.0, n), m(-1074, -960, n), y[-SPECIAL.size ** 2:]]
    to = [u(-30, 30, 5 * n), u(-1000, 1000, 3 * n), m(20, 60, n) * sign(n), u(-300, 300, 2 * n), np.round(u(-8, 8, 2 * n)),
          np.round(u(-8, 8, n)), u(0.9, 1.1, n), x[-SPECIAL.size ** 2:]]
    out['power'] = (np.concatenate(base), np.concatenate(to))
    return out


def _same(a, b):
    """Bitwise the same, every NaN one NaN."""
    a, b = np.asarray(a, dtype=np.float64), np.asarray(b, dtype=np.float64)
    return (a.view(np.uint64) == b.view(np.uint64)) | (np.isnan(a) & np.isnan(b))


def _twin(name, a, b):
    from machine import libm_twin
    return getattr(libm_twin, name)(a) if b is None else getattr(libm_twin, name)(a, b)


def test_c_arrays_scalars_and_twin_agree_to_the_bit(report):
    """Each function's C over arrays, its C a call at a time and its numpy twin: the same bits at every draw."""
    from tools.cores import libm_c
    for name, (a, b) in _cases().items():
        arr, one, twin = libm_c.call(name, a, b), libm_c.scalar(name, a, b), _twin(name, a, b)
        bad = np.flatnonzero(~(_same(arr, twin) & _same(arr, one)))
        report.check('%s: arrays, scalars and twin the same bits' % name, bad.size == 0,
                     '%d draws' % a.size if bad.size == 0 else 'at %r: %r %r %r' % (
                         float(a[bad[0]]), float(arr[bad[0]]), float(one[bad[0]]), float(twin[bad[0]])))
    a = _cases()['sin'][0]
    s, c = libm_c.sincos(a)
    report.check('sincos: its pair the same bits as sin and cos', bool(_same(s, libm_c.call('sin', a)).all()
                 and _same(c, libm_c.call('cos', a)).all() and _same(s, libm_c.scalar('sincos_sin', a)).all()
                 and _same(c, libm_c.scalar('sincos_cos', a)).all()), '%d draws' % a.size)


def test_the_draws_digest_is_pinned(report):
    """The twin's bits over every draw hash to the digest laid on Windows and held on Linux."""
    h = hashlib.sha256()
    for name, (a, b) in _cases().items():
        y = np.asarray(_twin(name, a, b), dtype=np.float64)
        h.update(name.encode() + np.where(np.isnan(y), NAN, y).astype('<f8').tobytes())
    report.check("the draws' digest is the one pinned", h.hexdigest() == DIGEST, h.hexdigest())


def test_errors_under_one_unit_in_the_last_place(report):
    """Each function's error against its value to 60 digits, at READ of its draws (finite, with a finite value): under
    one unit in the last place of the nearest double, its worst told."""
    from tools.cores import libm as reference
    from tools.cores import libm_c
    for name, (a, b) in _cases().items():
        y = libm_c.call(name, a, b)
        keep = np.flatnonzero(np.isfinite(a) & np.isfinite(y) & (np.isfinite(b) if b is not None else True))
        keep = keep[np.linspace(0, keep.size - 1, READ).astype(int)]
        worst, at = 0.0, None
        for i in keep:
            err = float(reference.ulps(float(y[i]), reference.reference(name, float(a[i]), None if b is None else float(b[i]))))
            if err > worst:
                worst, at = err, float(a[i])
        report.check('%s: under one unit in the last place' % name, worst < 1.0, 'worst %.3f at %r' % (worst, at))


def test_special_values_as_annex_f(report):
    """Zeros by their sign, infinities, NaNs, the domains' edges, exp's range and arctan2's quadrants: each value as
    Annex F has it, to the bit, in the C and in the twin."""
    from tools.cores import libm_c
    wrong = []
    for name, args, want in ANNEX:
        a = np.array([args[0]])
        b = None if len(args) == 1 else np.array([args[1]])
        for got in (libm_c.call(name, a, b)[0], libm_c.scalar(name, a, b)[0], _twin(name, a, b)[0]):
            if not _same(got, want):
                wrong.append('%s%r = %r, not %r' % (name, args, float(got), want))
    report.check('every special value as Annex F has it', not wrong, '; '.join(wrong[:4]) or '%d values' % len(ANNEX))


def test_powers_a_double_holds_are_exact(report):
    """A power whose value a double holds - 2^k across the doubles, a whole base's while under 2^53, a square's root,
    x^1 - is that double: in the C's arrays, a call at a time, and the twin."""
    from tools.cores import libm_c
    x, y, want = (np.array(v, dtype=np.float64) for v in zip(*EXACT))
    bad = ['%r^%r = %r, not %r' % (float(x[i]), float(y[i]), float(got[i]), float(want[i]))
           for got in (libm_c.call('power', x, y), libm_c.scalar('power', x, y), _twin('power', x, y))
           for i in np.flatnonzero(~_same(got, want))]
    report.check('every power a double holds is that double', not bad, '; '.join(bad[:4]) or '%d powers' % len(EXACT))


def test_odd_and_even_to_the_bit(report):
    """sin, tan, arctan, arcsin, tanh and arctan2's y odd, cos even, hypot even in each argument and symmetric: to the
    bit."""
    from tools.cores import libm_c
    cases = _cases()
    for name, kind in (('sin', -1.0), ('tan', -1.0), ('arctan', -1.0), ('arcsin', -1.0), ('tanh', -1.0), ('cos', 1.0)):
        a = cases[name][0]
        report.check('%s %s' % (name, 'odd' if kind < 0 else 'even'), bool(_same(libm_c.call(name, -a), kind * libm_c.call(name, a)).all()),
                     '%d draws' % a.size)
    y, x = cases['arctan2']
    report.check('arctan2 odd in y', bool(_same(libm_c.call('arctan2', -y, x), -libm_c.call('arctan2', y, x)).all()), '%d draws' % y.size)
    h = libm_c.call('hypot', y, x)
    report.check('hypot even in each argument and symmetric', bool(
        (_same(libm_c.call('hypot', -y, x), h) & _same(libm_c.call('hypot', y, -x), h) & _same(libm_c.call('hypot', x, y), h)).all()),
        '%d draws' % y.size)


def test_every_length_through_tails_and_chunks(report):
    """An array of every length to 20, and about the chunk's and its block's edges, through the C's runs and tails:
    each element the scalar call's bits."""
    from tools.cores import libm_c
    rng = np.random.default_rng(SEED + 1)
    bad = []
    for n in list(range(0, 21)) + [255, 256, 257, 263, 511, 512, 513, 1000]:
        a = np.concatenate([rng.uniform(-8, 8, n - n // 3), _mags(rng, 20, 1000, n // 3)])
        b = rng.uniform(-8, 8, n)
        for name in libm_c.ONE + libm_c.TWO:
            arg = np.abs(a) if name == 'log' else (a / (1.0 + np.abs(a)) if name in ('arcsin', 'arccos') else a)
            if not _same(libm_c.call(name, arg, b), libm_c.scalar(name, arg, b)).all():
                bad.append('%s n=%d' % (name, n))
    report.check('every length the scalar calls\' bits', not bad, ', '.join(bad[:6]) or '29 lengths, 11 functions')


def test_the_codes_calls_are_the_twins_bits(report):
    """`machine.libm` as the robot's code calls it, every way in, against the twin at every draw: a float a call, the
    whole array, arrays of every length to 20 (the few a call each, a block, its tail), strided and of two
    dimensions, numpy's float64 and a 0-d array, an int where the draw is whole, sin and cos together, an array with
    a number and two broadcast."""
    from machine import libm
    from tools.cores import libm_quick
    bad = []
    for name, (a, b) in _cases().items():
        twin, fn = _twin(name, a, b), getattr(libm, name)
        args = (a,) if b is None else (a, b)
        ways = {'whole': fn(*args), 'floats': [fn(*[float(v) for v in row]) for row in zip(*args)],
                'strided': fn(*[v[::3] for v in args]), 'numpy scalars': [fn(*row) for row in zip(*[v[:400] for v in args])],
                '0-d': [fn(*[np.array(v) for v in row]) for row in zip(*[v[:50] for v in args])]}
        wants = {'whole': twin, 'floats': twin, 'strided': twin[::3], 'numpy scalars': twin[:400], '0-d': twin[:50]}
        n2 = (a.size // 7) * 7
        ways['two dimensions'], wants['two dimensions'] = fn(*[v[:n2].reshape(7, -1) for v in args]), twin[:n2].reshape(7, -1)
        for n in range(0, 21):
            ways['n=%d' % n], wants['n=%d' % n] = fn(*[v[40:40 + n].copy() for v in args]), twin[40:40 + n]
        whole = np.flatnonzero((a == np.round(a)) & (np.abs(a) < 2.0 ** 53) & ((b is None) or (b == np.round(b)) & (np.abs(b) < 2.0 ** 53)))[:200]
        ways['ints'] = [fn(*[int(v[i]) for v in args]) for i in whole]
        wants['ints'] = _twin(name, *([np.array([float(int(v[i])) for i in whole]) for v in args] + [None] * (b is None)))
        if b is not None:
            ways['with a number'], wants['with a number'] = fn(a[:300], float(b[7])), _twin(name, a[:300], np.full(300, b[7]))
            ways['a number with'], wants['a number with'] = fn(float(a[7]), b[:300]), _twin(name, np.full(300, a[7]), b[:300])
            ways['broadcast'] = fn(a[:60].reshape(60, 1), b[:9])
            wants['broadcast'] = _twin(name, *[np.ascontiguousarray(v) for v in np.broadcast_arrays(a[:60].reshape(60, 1), b[:9])])
        bad += ['%s %s' % (name, way) for way, got in ways.items()
                if np.shape(got) != np.shape(wants[way]) or not _same(got, wants[way]).all()]
    a = _cases()['sin'][0]
    (s, c), (ts, tc) = libm.sincos(a), (_twin('sin', a, None), _twin('cos', a, None))
    pairs = [libm.sincos(float(v)) for v in a]
    if not (_same(s, ts).all() and _same(c, tc).all() and _same([p[0] for p in pairs], ts).all() and _same([p[1] for p in pairs], tc).all()):
        bad.append('sincos')
    report.check("the robot's code's calls (%s) are the twin's bits every way in" % (
        'the extension' if libm_quick.module() is not None else 'the twin itself: no extension built'), not bad,
        ', '.join(bad[:8]) or '%d functions, 30 ways each' % len(_cases()))
    report.check('a float in, a float out; an array in, a float64 array of its shape',
                 isinstance(libm.sin(0.5), float) and isinstance(libm.arctan2(1, 2.0), float)
                 and libm.exp(np.zeros((2, 3))).shape == (2, 3) and libm.exp(np.zeros((2, 3))).dtype == np.float64,
                 '%s %s' % (type(libm.sin(0.5)).__name__, libm.exp(np.zeros((2, 3))).dtype))


def _owns(x, y):
    """The robot's own libm on traced arrays (`machine.libm`): each function, sin and cos together, an array with a
    number, one function of another."""
    from machine import libm
    unit, pos = np.clip(y, -1.0, 1.0), np.abs(x) + 0.1
    return {'trig': (libm.sin(x), libm.cos(y), libm.tan(0.3 * x)), 'pair': libm.sincos(x + y),
            'exps': (libm.exp(y), libm.expm1(x), libm.log(pos), libm.tanh(y)),
            'arcs': (libm.arctan(x), libm.arcsin(unit), libm.arccos(unit), libm.arctan2(y, x), libm.hypot(x, y)),
            'mixed': libm.arctan2(x, 0.5) + libm.hypot(2.0, libm.sin(y)), 'row': libm.cos(x[1]) * libm.sin(y[:, 2:3]),
            'powers': (libm.power(pos, y), libm.power(pos, 2.5), libm.sq(x) + libm.cube(y) - libm.quart(unit))}


def test_traced_arrays_take_it_in_c(report):
    """`_owns` compiled (`held.compiled`): `machine.libm` on traced arrays a statement of its own, wbc_libm.h's calls
    in C, held bit for bit on its traced arrays and fresh draws - on every host, numpy's last bits no part of it."""
    from tools.cores import held, trace
    rng = np.random.default_rng(17)
    args = (rng.normal(size=(4, 9)) * 3.0, rng.normal(size=(4, 9)))
    try:
        run = held.compiled(_owns, args, (0, 1), 'owns', fresh=4)
        kinds = {node for st in run.prog.statements if st.kind == 'ew' for node in _kinds(st.args)}
        report.check("the robot's own libm on traced arrays in C bit for bit, each call a node of its own",
                     {'own_' + n for n in ('sin', 'cos', 'tan', 'exp', 'expm1', 'log', 'tanh', 'arctan', 'arcsin', 'arccos',
                                           'arctan2', 'hypot', 'power')} <= kinds, '%d statements' % len(run.prog.statements))
    except (trace.Untraced, AssertionError, RuntimeError) as e:
        report.check("the robot's own libm on traced arrays in C bit for bit, each call a node of its own", False, str(e)[:300])


def _kinds(tree):
    """The kinds of an elementwise tree's nodes."""
    if not isinstance(tree, tuple) or not tree or not isinstance(tree[0], str):
        return set()
    return {tree[0]} | {k for kid in tree[1:] for k in _kinds(kid)}


def test_values_stay_in_their_ranges(report):
    """|sin|, |cos|, |tanh| <= 1; arcsin and arctan within pi/2, arccos in [0, pi], arctan2 within pi; exp >= 0,
    hypot at least either magnitude: at every draw."""
    from tools.cores import libm_c
    c, ok = _cases(), []
    with np.errstate(invalid='ignore'):
        for name, lo, hi in (('sin', -1.0, 1.0), ('cos', -1.0, 1.0), ('arcsin', -math.pi / 2, math.pi / 2),
                             ('arctan', -math.pi / 2, math.pi / 2), ('arccos', 0.0, math.pi), ('exp', 0.0, INF),
                             ('tanh', -1.0, 1.0)):
            y = libm_c.call(name, c[name][0])
            ok.append((name, bool(((y >= lo) & (y <= hi) | np.isnan(y)).all())))
        y, x = c['arctan2']
        t, h = libm_c.call('arctan2', y, x), libm_c.call('hypot', y, x)
        ok += [('arctan2', bool(((np.abs(t) <= math.pi) | np.isnan(t)).all())),
               ('hypot', bool(((h >= np.fmax(np.abs(y), np.abs(x))) | np.isnan(h)).all()))]
    report.check('every value in its range', all(v for _n, v in ok), ', '.join(n for n, v in ok if not v) or '9 functions')


def test_the_tables_are_their_tools(report):
    """wbc_libm_tables.h and machine/libm_tables.py as tools/cores/libm.py lays them now (its fits, not their read)."""
    from tools.cores import libm
    c, py, _worst = libm.laid(read=False)
    texts = []
    for path in (libm.C_PATH, libm.PY_PATH):
        with open(path, encoding='utf-8', newline='') as f:
            texts.append(f.read())
    report.check("the tables are their tool's", texts == [c, py], 'lay them: python tools/cores/libm.py --write')


ROSTER = (test_the_tables_are_their_tools, test_c_arrays_scalars_and_twin_agree_to_the_bit, test_the_draws_digest_is_pinned,
          test_errors_under_one_unit_in_the_last_place, test_special_values_as_annex_f, test_odd_and_even_to_the_bit,
          test_powers_a_double_holds_are_exact, test_every_length_through_tails_and_chunks, test_the_codes_calls_are_the_twins_bits,
          test_traced_arrays_take_it_in_c, test_values_stay_in_their_ranges)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    from tools.cores.build import find_cc
    if find_cc() is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
