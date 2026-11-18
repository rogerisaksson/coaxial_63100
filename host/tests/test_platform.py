#!/usr/bin/env python3
"""What each machine computes its own way and what every one computes alike: each layer under the robot's
arithmetic on seeded work alone, not a step of MuJoCo's. IEEE 754's basic operations and square root, and the
robot's C on whatever target it is built for (COAXIAL_ISA: its QP, wbc_qp.c, by OpenBLAS's kernels alone - its QR the
robot's own) - one set of bits on every machine; the platform's libm (math's: UCRT's on Windows, glibc's on Linux, each rounding its last bit its own way), numpy's
float64 functions over arrays (its own SIMD paths where it dispatches them - X86_V3, ASIMD, SVE -, the platform's
libm where it does not), OpenBLAS's products and solves (by its kernels), numpy's LAPACK (by its kernels and system:
its Fortran's auxiliaries take the platform's runtime) and MuJoCo's compiler turning a geom by
its fromto, zaxis or euler (by the platform's atan2, sin and cos: a right angle's half UCRT's 0.7071067811865476,
glibc's ...475 - one XML, two models), each held to the digests its class laid (LIBM, NUMPY, BLAS, LAPACKS, TURNS: by
system and processor, numpy's target for a function, OpenBLAS's kernels). A class none laid is told with its
digests, to be laid; an x86-64 under the relay is held to the desk's numpy paths (`tools.dev.arithmetic`).
Where two machines part - a walk that falls on one and stands on the other - this says which layer and which
function parts, its work a second's."""
import hashlib
import math
import os
import platform
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report
from platform_laid import BLAS, IEEE, LAPACKS, LIBM, NUMPY, QPS, QR_ALIKE, TURNS

SEED, N = 63100, 20000
LAPACK = ('qr', 'lstsq', 'svd', 'eigh', 'solve', 'cholesky', 'inv')
#: The functions read, numpy's names: math's of each, numpy's, and its draws' names (two where it takes two); power2
#: is MuJoCo's own: an impedance's pow at its solimp's power 2 in every soft contact's step.
FUNCS = (('sin', math.sin, np.sin, ('wide',)), ('cos', math.cos, np.cos, ('wide',)), ('tan', math.tan, np.tan, ('wide',)),
         ('exp', math.exp, np.exp, ('exp',)), ('expm1', math.expm1, np.expm1, ('unit',)), ('log', math.log, np.log, ('pos',)),
         ('log1p', math.log1p, np.log1p, ('pos',)), ('arcsin', math.asin, np.arcsin, ('unit',)),
         ('arccos', math.acos, np.arccos, ('unit',)), ('arctan', math.atan, np.arctan, ('wide',)),
         ('arctan2', math.atan2, np.arctan2, ('wide', 'other')), ('hypot', math.hypot, np.hypot, ('wide', 'other')),
         ('power', math.pow, np.power, ('base', 'power')), ('power2', lambda x: math.pow(x, 2.0), lambda x: np.power(x, 2.0),
                                                            ('base',)),
         ('tanh', math.tanh, np.tanh, ('wide',)))
#: The robot's own of the functions (machine.libm): what the platform's part from, told.
OWN = ('sin', 'cos', 'tan', 'exp', 'expm1', 'log', 'arcsin', 'arccos', 'arctan', 'arctan2', 'hypot', 'tanh')
#: A scene of the geoms MuJoCo's compiler turns: capsules along x, along y, along -z and slanting (the robot's thigh's),
#: a cylinder up y, boxes a right angle about x and 30 deg about y.
SCENE = ('<mujoco><worldbody><body>'
         '<geom type="capsule" size="0.05" fromto="0 0 0 0.3 0 0"/><geom type="capsule" size="0.05" fromto="0 0 0 0 0.3 0"/>'
         '<geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.3"/>'
         '<geom type="capsule" size="0.058" fromto="0.006 -0.22 0.012 0 -0.36 0"/>'
         '<geom type="cylinder" size="0.05 0.1" zaxis="0 1 0"/><geom type="box" size="0.1 0.1 0.1" euler="90 0 0"/>'
         '<geom type="box" size="0.1 0.1 0.1" euler="0 30 0"/></body></worldbody></mujoco>')


def _digest(values):
    """Values' bits, NaNs as one NaN, hashed short."""
    a = np.asarray(values, dtype=np.float64)
    return hashlib.blake2b(np.where(np.isnan(a), np.nan, a).astype('<f8').tobytes(), digest_size=8).hexdigest()


def _class():
    """This host's system and processor, as Python's platform names them: Windows-AMD64, Linux-x86_64, Linux-aarch64."""
    return '%s-%s' % (platform.system(), platform.machine())


def _draws():
    """The seeded draws, each exact on every machine (uniform's bits scaled by a power of two)."""
    rng = np.random.default_rng(SEED)
    u = rng.uniform
    return {'wide': u(-40.0, 40.0, N), 'other': u(-40.0, 40.0, N), 'unit': u(-1.0, 1.0, N), 'pos': u(1e-3, 50.0, N),
            'exp': np.concatenate([u(-700.0, 700.0, N // 2), u(-1.0, 1.0, N // 2)]), 'base': u(1e-3, 4.0, N),
            'power': u(-3.0, 3.0, N)}


def _targets():
    """numpy's target for each function here (its float64 loop's): its SIMD paths' name or its baseline's."""
    from numpy.lib import introspect
    info = introspect.opt_func_info(func_name='|'.join(f[0] for f in FUNCS if f[0] != 'power2'), signature='float64')
    out = {name: next(iter(sigs.values()))['current'] for name, sigs in info.items()}
    return {f[0]: out.get('power' if f[0] == 'power2' else f[0], '?') for f in FUNCS}


def _kernels():
    """OpenBLAS's kernels as the library numpy loaded names them (its DYNAMIC_ARCH's pick), None where unread."""
    import ctypes
    import glob
    here = os.path.dirname(np.__file__)
    for path in glob.glob(os.path.join(here, '..', 'numpy.libs', '*openblas*')) + glob.glob(os.path.join(here, '.libs', '*openblas*')):
        lib = ctypes.CDLL(path)
        for name in ('scipy_openblas_get_corename64_', 'openblas_get_corename64_', 'openblas_get_corename'):
            fn = getattr(lib, name, None)
            if fn is not None:
                fn.restype = ctypes.c_char_p
                return fn().decode()
    return None


def _held(report, label, table, key, got):
    """`got` ({name: digest}) held to `table`'s laid for `key`: each name its own check, or the class told."""
    laid = table.get(key)
    if laid is None:
        report.check('%s: %s not laid yet, told' % (label, key), True, repr(got))
        return
    for name, digest in got.items():
        report.check('%s: %s as %s laid it' % (label, name, key), laid.get(name) == digest, '%s, laid %s' % (digest, laid.get(name)))


def test_basic_operations_and_square_root_alike(report):
    """+, -, *, / and square root over the draws, numpy's arrays and Python's floats: one digest on every machine."""
    d = _draws()
    a, b = d['wide'], d['other']
    arrays = np.concatenate([a + b, a - b, a * b, a / b, np.sqrt(np.abs(a))])
    floats = [f(x, y) for x, y in zip(a.tolist()[:2000], b.tolist()[:2000])
              for f in (lambda x, y: x + y, lambda x, y: x * y, lambda x, y: x / y, lambda x, y: math.sqrt(abs(x)))]
    got = _digest(np.concatenate([arrays, floats]))
    report.check('IEEE 754: the basic operations and square root, the digest every machine has', got == IEEE or not IEEE,
                 '%s, laid %s' % (got, IEEE or 'none: told'))


def test_the_platforms_libm(report):
    """math's functions over the draws: digests by system and processor, and how far each parts from the robot's own."""
    from machine import libm
    d = _draws()
    got, apart = {}, []
    for name, fn, _np, keys in FUNCS:
        args = [d[k].tolist() for k in keys]
        y = np.array([fn(*xs) for xs in zip(*args)])
        got[name] = _digest(y)
        if name in OWN:
            own = np.asarray(getattr(libm, name)(*[d[k] for k in keys]), dtype=np.float64)
            apart.append('%s %d' % (name, int(np.sum(own != y))))
    _held(report, 'the platform\'s libm', LIBM, _class(), got)
    report.check('told: of %d draws a function, the platform\'s apart from the robot\'s own (machine.libm)' % N, True, ', '.join(apart))


def test_numpys_float64_functions(report):
    """numpy's functions over the draws as arrays: digests by system and processor, each with numpy's target for it."""
    d = _draws()
    targets = _targets()
    got = {name: '%s %s' % (targets[name], _digest(fn(*[d[k] for k in keys]))) for name, _m, fn, keys in FUNCS}
    _held(report, "numpy's functions", NUMPY, _class(), got)


def test_openblas_products_and_solves(report):
    """OpenBLAS's products and solves on exact inputs (uniform draws and elementwise operations: a normal draw's tail
    takes the platform's log): by its kernels."""
    rng = np.random.default_rng(SEED + 1)
    a, b, v, u = rng.uniform(-1.0, 1.0, (37, 53)), rng.uniform(-1.0, 1.0, (53, 41)), rng.uniform(-1.0, 1.0, 64), rng.uniform(-1.0, 1.0, 64)
    s = u[:, None] * u[None, :] + np.diag(rng.uniform(64.0, 65.0, 64))
    got = {'gemm': _digest(a @ b), 'gemv': _digest(s @ v), 'solve': _digest(np.linalg.solve(s, v)),
           'cholesky': _digest(np.linalg.cholesky(s)), 'square': _digest(s @ s)}
    _held(report, 'OpenBLAS', BLAS, '%s-%s' % (_class(), _kernels()), got)


def test_lapacks_factorings(report):
    """numpy's LAPACK on exact inputs, twenty a routine: by its kernels and system - its Fortran's auxiliaries take the
    platform's runtime (lstsq's, eigh's and some of QR's bits part between Windows and Linux on Haswell's kernels)."""
    rng = np.random.default_rng(SEED + 3)
    work = {name: [] for name in LAPACK}
    for _ in range(20):
        a, b = rng.uniform(-1.0, 1.0, (9, 6)), rng.uniform(-1.0, 1.0, 9)
        u = rng.uniform(-1.0, 1.0, 12)
        s = u[:, None] * u[None, :] + np.diag(rng.uniform(2.0, 3.0, 12))
        for name, out in (('qr', np.linalg.qr(a)), ('lstsq', (np.linalg.lstsq(a, b, rcond=None)[0],)),
                          ('svd', np.linalg.svd(a)), ('eigh', np.linalg.eigh(s)), ('solve', (np.linalg.solve(s, u),)),
                          ('cholesky', (np.linalg.cholesky(s),)), ('inv', (np.linalg.inv(s),))):
            work[name] += [np.ravel(x) for x in out]
    _held(report, 'LAPACK', LAPACKS, '%s-%s' % (_class(), _kernels()), {name: _digest(np.concatenate(v)) for name, v in work.items()})


def test_mujocos_compiler_turns(report):
    """SCENE compiled (no step): its geoms' turns by system and processor, and which part from the turn by square roots
    alone - the half angle's cosine and sine sqrt((1 +- c) / 2) of its cosine c, a right angle's sqrt(0.5) both."""
    import importlib
    m = importlib.import_module('mujoco').MjModel.from_xml_string(SCENE)
    q = np.asarray(m.geom_quat, dtype=np.float64)
    half = math.sqrt(0.5)
    rooted = {0: (half, 0.0, -half, 0.0), 1: (half, half, 0.0, -0.0), 4: (half, -half, 0.0, 0.0), 5: (half, half, 0.0, 0.0)}
    apart = [i for i, r in rooted.items() if tuple(float(c) for c in q[i]) != r]
    _held(report, "MuJoCo's compiler", TURNS, _class(), {'turns': _digest(q)})
    report.check("told: the right angles MuJoCo's compiler turns otherwise than square roots do", True,
                 'geoms %s of %s' % (apart, sorted(rooted)))


def test_the_robots_qp_in_c(report):
    """The robot's QP in C (wbc_qp.c, built for this host's target) over seeded problems built of elementwise
    operations alone: by OpenBLAS's kernels, Windows and Linux alike on them and every target alike (avx, avx2 and
    native one digest on the desk)."""
    from tools.cores import qp
    rng = np.random.default_rng(SEED + 2)
    out = []
    for _ in range(60):
        n, m = 8, 6
        u = rng.uniform(-1.0, 1.0, n)
        h = u[:, None] * u[None, :] + np.diag(rng.uniform(1.0, 4.0, n))
        x, _held_rows, mult = qp.solve(h, rng.uniform(-1.0, 1.0, n), rng.uniform(-1.0, 1.0, (m, n)), rng.uniform(-1.0, 1.0, m))
        out.append(np.zeros(n) if x is None else x)
        out.append(np.asarray(mult, dtype=np.float64).ravel() if mult is not None else np.zeros(0))
    _held(report, "the robot's QP in C (%s)" % os.environ.get('COAXIAL_ISA', 'native'), QPS, _kernels(),
          {'qp': _digest(np.concatenate(out))})


def test_the_robots_sums_and_qr_alike(report):
    """The robot's pairwise sum (npsum) beside numpy's at every length to 1100, a run and a row of a block; its QR
    (wbc_qr) beside its twin (machine.stack.qp.householder) on every shape to 24 by 24 and on a zero, a repeated and a
    scaled column: the same bits, Q orthogonal and QR its matrix within 8 m units, one digest on every machine."""
    from machine.stack import qp as twin
    from tools.cores import qp
    rng = np.random.default_rng(SEED + 4)
    runs = [rng.uniform(-1.0, 1.0, n) * rng.uniform(0.5, 2.0) for n in range(1101)]
    block = np.ascontiguousarray(rng.uniform(-1.0, 1.0, (9, 300)))
    apart = sum(qp.npsum(r) != float(np.sum(r)) for r in runs) + int(np.sum(np.array([qp.npsum(row) for row in block])
                                                                         != np.sum(block, axis=1)))
    report.check("the robot's pairwise sum is numpy's: every length to 1100 and a block's rows", apart == 0,
                 '%d apart' % apart)
    shapes = [rng.uniform(-1.0, 1.0, (m, q)) for m in range(1, 25) for q in range(1, m + 1)]
    edge = rng.uniform(-1.0, 1.0, (12, 6))
    edge[:, 1], edge[:, 3], edge[:, 4], edge[:, 5] = 0.0, edge[:, 2], edge[:, 4] * 1e150, edge[:, 5] * 1e-150
    out, unlike, worst = [], 0, 0.0
    for a in shapes + [edge]:
        got, (tq, tr) = qp.qr(a), twin.householder(a)
        if got is None:
            unlike += 1
            continue
        cq, cr = got
        unlike += int(not (np.array_equal(cq.view(np.uint64), tq.view(np.uint64)) and np.array_equal(cr.view(np.uint64), tr.view(np.uint64))))
        m, scale = a.shape[0], float(np.abs(a).max())
        worst = max(worst, float(np.abs(cq.T @ cq - np.eye(m)).max()) / (m * 2.0 ** -52),
                    float(np.abs((cq @ cr - a) / np.maximum(np.abs(a).max(axis=0), 1e-300)).max()) / (m * 2.0 ** -52) if scale else 0.0)
        out += [cq.ravel(), cr.ravel()]
    report.check("the robot's QR in C is its twin's bits on %d shapes, Q orthogonal and QR its matrix within 8 m units"
                 % (len(shapes) + 1), unlike == 0 and worst <= 8.0, '%d unlike, worst %.2f m units' % (unlike, worst))
    got = _digest(np.concatenate(out + [np.array([qp.npsum(r) for r in runs])]))
    report.check("the robot's QR and sums: the digest every machine has", got == QR_ALIKE or not QR_ALIKE,
                 '%s, laid %s' % (got, QR_ALIKE or 'none: told'))


def test_an_x86_is_held_to_the_desks_arithmetic(report):
    """Under the relay an x86-64 runs no numpy path past X86_V3 (`tools.dev.arithmetic`); its OpenBLAS kernels its own
    pick, told - its products and LAPACK held by them above."""
    from tools.dev import arithmetic
    if platform.machine().lower() not in arithmetic.X86:
        report.check('not an x86-64 (%s): its own arithmetic, held to its class' % _class(), True)
        return
    wider = sorted(set(_targets().values()) & set(arithmetic.WIDER))
    report.check("an x86-64's numpy runs no path past X86_V3", not wider, 'wider paths %s, OpenBLAS %s' % (wider, _kernels()))


ROSTER = (test_basic_operations_and_square_root_alike, test_the_platforms_libm, test_numpys_float64_functions,
          test_openblas_products_and_solves, test_lapacks_factorings, test_mujocos_compiler_turns, test_the_robots_qp_in_c,
          test_the_robots_sums_and_qr_alike,
          test_an_x86_is_held_to_the_desks_arithmetic)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    print('class %s, numpy %s, OpenBLAS %s' % (_class(), np.__version__, _kernels()))
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
