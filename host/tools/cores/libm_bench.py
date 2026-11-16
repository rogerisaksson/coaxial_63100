#!/usr/bin/env python3
"""The robot's libm beside numpy's (the platform's): its pace and accuracy, a function at a time.

    python tools/cores/libm_bench.py                        # ns an element at each size, and the accuracy table
    python tools/cores/libm_bench.py --sizes 3 64 1048576   # the sizes timed
    python tools/cores/libm_bench.py --points 2000          # the points the accuracy is read at, a function

Pace: the C's arrays (wbc_libm.c, a vector of lanes at a time), the C a call at a time as another core inlines it
(wbc_sin, ...), and numpy into its own `out=` (no allocation either side), timed by turns over the same draws, each
the best of REPEATS rounds of at least TIMED seconds. Accuracy: both against the function to 60 digits
(tools.cores.libm.reference) at the same draws: the worst and the mean error in units in the last place, and the
share past half a unit (not the nearest double).
"""
import argparse
import os
import sys
import time

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, HOST)

import numpy as np

REPEATS, TIMED, SEED = 7, 0.02, 7


def draws(name, n, rng):
    """n draws over a function's working range (the robot's angles, exp's whole range, log's)."""
    u = rng.uniform
    pick = {'sin': lambda: u(-10, 10, n), 'cos': lambda: u(-10, 10, n), 'tan': lambda: u(-1.5, 1.5, n),
            'exp': lambda: u(-700, 700, n), 'expm1': lambda: u(-2, 2, n), 'log': lambda: np.exp(u(-700, 700, n)),
            'arctan': lambda: u(-5, 5, n), 'arcsin': lambda: u(-1, 1, n), 'arccos': lambda: u(-1, 1, n),
            'arctan2': lambda: u(-10, 10, n), 'hypot': lambda: u(-10, 10, n)}
    return pick[name]()


def _best(call):
    """The best of REPEATS rounds of `call`, each repeated till TIMED seconds: seconds a call."""
    best = float('inf')
    for _ in range(REPEATS):
        k, t0 = 0, time.perf_counter()
        while True:
            call()
            k += 1
            dt = time.perf_counter() - t0
            if dt >= TIMED:
                break
        best = min(best, dt / k)
    return best


def pace(names, sizes):
    """[(function, size, ours' arrays, ours a call at a time, numpy's: ns an element)]."""
    from tools.cores import libm_c
    lib, rows, rng = libm_c.lib(), [], np.random.default_rng(SEED)
    for name in names:
        for n in sizes:
            a, b = draws(name, n, rng), draws(name, n, rng)
            y = np.empty_like(a)
            pa, pb, py = libm_c._p(a), libm_c._p(b), libm_c._p(y)
            arr = getattr(lib, 'wbh_libm_' + name)
            which = libm_c.SCALAR.index(name)
            ufunc = getattr(np, name)
            if name in libm_c.TWO:
                calls = (lambda: arr(pa, pb, py, n), lambda: lib.wbh_libm_scalar(which, pa, pb, py, n),
                         lambda: ufunc(a, b, out=y))
            else:
                calls = (lambda: arr(pa, py, n), lambda: lib.wbh_libm_scalar(which, pa, pa, py, n), lambda: ufunc(a, out=y))
            times = [float('inf')] * 3
            for _ in range(2):  # by turns, twice over
                times = [min(t, _best(c)) for t, c in zip(times, calls)]
            rows.append((name, n) + tuple(1e9 * t / n for t in times))
    return rows


def accuracy(names, points):
    """[(function, ours' worst, mean, share past half; numpy's the same)], over the same draws."""
    from tools.cores import libm as reference
    from tools.cores import libm_c
    rng, rows = np.random.default_rng(SEED + 1), []
    for name in names:
        a, b = draws(name, points, rng), draws(name, points, rng)
        ours = libm_c.call(name, a, b) if name in libm_c.TWO else libm_c.call(name, a)
        with np.errstate(all='ignore'):
            theirs = getattr(np, name)(a, b) if name in libm_c.TWO else getattr(np, name)(a)
        errs = []
        for i in range(points):
            q = reference.reference(name, float(a[i]), float(b[i]))
            errs.append((float(reference.ulps(float(ours[i]), q)), float(reference.ulps(float(theirs[i]), q))))
        e = np.array(errs)
        rows.append((name,) + tuple(v for k in (0, 1) for v in (e[:, k].max(), e[:, k].mean(), (e[:, k] > 0.5).mean())))
    return rows


def main(argv=None):
    from tools.cores import libm_c
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--sizes', type=int, nargs='+', default=[3, 8, 64, 4096, 1 << 20], help='array sizes timed')
    parser.add_argument('--points', type=int, default=1000, help='points the accuracy is read at, a function')
    parser.add_argument('--only', nargs='+', default=list(libm_c.ONE + libm_c.TWO), help='the functions')
    args = parser.parse_args(argv)
    print('pace, ns an element: ours over arrays | ours a call at a time | numpy (x: numpy over ours\' arrays)')
    for name, n, arr, one, theirs in pace(args.only, args.sizes):
        print('  %-8s n=%-8d %8.2f | %8.2f | %8.2f   x%.2f' % (name, n, arr, one, theirs, theirs / arr))
    print('\naccuracy at %d draws, units in the last place: ours worst, mean, past 1/2 | numpy the same' % args.points)
    for name, ow, om, op, nw, nm, nump in accuracy(args.only, args.points):
        print('  %-8s %6.3f %6.3f %6.1f%% | %6.3f %6.3f %6.1f%%' % (name, ow, om, 100 * op, nw, nm, 100 * nump))
    return 0


if __name__ == '__main__':
    sys.exit(main())
