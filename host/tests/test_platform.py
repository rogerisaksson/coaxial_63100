#!/usr/bin/env python3
"""What each machine computes its own way and what every one computes alike: each layer under the robot's
arithmetic on seeded work alone, not a step of MuJoCo's. IEEE 754's basic operations and square root, and the
robot's C on whatever target it is built for (COAXIAL_ISA: its QP, wbc_qp.c, by system while its QR is LAPACK's)
- one set of bits on every machine; the platform's libm (math's: UCRT's on Windows, glibc's on Linux, each rounding its last bit its own way), numpy's
float64 functions over arrays (its own SIMD paths where it dispatches them - X86_V3, ASIMD, SVE -, the platform's
libm where it does not), OpenBLAS's products and solves (by its kernels), numpy's LAPACK (by its kernels and system:
its Fortran's auxiliaries take the platform's runtime) and MuJoCo's compiler turning a geom by
its fromto, zaxis or euler (by the platform's atan2, sin and cos: a right angle's half UCRT's 0.7071067811865476,
glibc's ...475 - one XML, two models), each held to the digests its class laid (LIBM, NUMPY, BLAS, LAPACKS, TURNS: by
system and processor, numpy's target for a function, OpenBLAS's kernels). A class none laid is told with its
digests, to be laid; an x86-64 under the relay is held to the desk's kernels and paths (`tools.dev.arithmetic`).
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

SEED, N = 63100, 20000
#: The basic operations' and square root's digest: one on every machine.
IEEE = '1f828e7967236254'
#: The robot's QP in C (wbc_qp.c) by system, processor and OpenBLAS's kernels: its products and QR are numpy's
#: OpenBLAS's and LAPACK's (wbc_blas.c's dgeqrf and dorgqr), whose bits part between systems and kernels.
QPS = {
    'Linux-aarch64-armv9sme': {'qp': 'c55295a6adee197d'},
    'Linux-x86_64-Haswell': {'qp': 'd279562f122b1ba0'},
    'Windows-AMD64-Haswell': {'qp': '67d868fd7078b09f'},
}
#: The platform's libm, a function at a time, by system and processor.
LIBM = {
    'Linux-aarch64': {'sin': '05499355fafc2a34', 'cos': '707818489ee348fd', 'tan': 'f8ae32e49e14a496',
                      'exp': 'ebb29a6ad87c85cc', 'expm1': '0e66947e8709b707', 'log': '5cfb03c4bac0fe96',
                      'log1p': 'ace9c59adb951715', 'arcsin': '1d6fc3b7ae3642cd', 'arccos': 'f15cd9d9a2e0f6a2',
                      'arctan': '034dda9cc376bc98', 'arctan2': '29790e22fa17a088', 'hypot': 'b7ae948bdc263fb8',
                      'power': '6dff36f49d9c7688', 'power2': '96eed274401f65c1', 'tanh': 'e9c74ef196da8d66'},
    'Linux-x86_64': {'sin': '05499355fafc2a34', 'cos': '707818489ee348fd', 'tan': 'f8ae32e49e14a496',
                     'exp': 'ebb29a6ad87c85cc', 'expm1': '0e66947e8709b707', 'log': '5cfb03c4bac0fe96',
                     'log1p': 'ace9c59adb951715', 'arcsin': '1d6fc3b7ae3642cd', 'arccos': 'f15cd9d9a2e0f6a2',
                     'arctan': '034dda9cc376bc98', 'arctan2': '29790e22fa17a088', 'hypot': 'b7ae948bdc263fb8',
                     'power': '6dff36f49d9c7688', 'power2': '96eed274401f65c1', 'tanh': 'e9c74ef196da8d66'},
    'Windows-AMD64': {'sin': 'be2539f885177d33', 'cos': '6b7bce834c8e7a76', 'tan': 'c8529ff8ebd78682',
                      'exp': 'e27ada39f6ac4dfa', 'expm1': '0be8573280b39685', 'log': '665ed5d0ed26fd0f',
                      'log1p': '90de35f95bf1c7cd', 'arcsin': '191f78865ba12646', 'arccos': '8f50e46b62c63046',
                      'arctan': 'c4e473af7db8ab87', 'arctan2': 'd2c1fb88e72f0958', 'hypot': 'b7ae948bdc263fb8',
                      'power': '880365b107a38193', 'power2': '19b55c9e45162165', 'tanh': '174b129f49e7adcd'},
}
#: numpy's float64 functions by system and processor: each function's target there and its digest.
NUMPY = {
    'Linux-aarch64': {'sin': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 05499355fafc2a34',
                      'cos': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 707818489ee348fd',
                      'tan': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) f8ae32e49e14a496',
                      'exp': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) ebb29a6ad87c85cc',
                      'expm1': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 0e66947e8709b707',
                      'log': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 5cfb03c4bac0fe96',
                      'log1p': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) ace9c59adb951715',
                      'arcsin': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 1d6fc3b7ae3642cd',
                      'arccos': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) f15cd9d9a2e0f6a2',
                      'arctan': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 034dda9cc376bc98',
                      'arctan2': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 29790e22fa17a088',
                      'hypot': '? aa298164a506653b',
                      'power': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 6dff36f49d9c7688',
                      'power2': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 0c7e5fe3ffa63729',
                      'tanh': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 359b7e80c2d37e86'},
    'Linux-x86_64': {'sin': 'X86_V3 05499355fafc2a34', 'cos': 'X86_V3 707818489ee348fd',
                     'tan': 'baseline(X86_V2) f8ae32e49e14a496', 'exp': 'X86_V3 ebb29a6ad87c85cc',
                     'expm1': 'baseline(X86_V2) 0e66947e8709b707', 'log': 'X86_V3 5cfb03c4bac0fe96',
                     'log1p': 'baseline(X86_V2) ace9c59adb951715', 'arcsin': 'baseline(X86_V2) 1d6fc3b7ae3642cd',
                     'arccos': 'baseline(X86_V2) f15cd9d9a2e0f6a2', 'arctan': 'baseline(X86_V2) 034dda9cc376bc98',
                     'arctan2': 'baseline(X86_V2) 29790e22fa17a088', 'hypot': '? aa298164a506653b',
                     'power': 'baseline(X86_V2) 6dff36f49d9c7688', 'power2': 'baseline(X86_V2) 0c7e5fe3ffa63729',
                     'tanh': 'X86_V3 359b7e80c2d37e86'},
    'Windows-AMD64': {'sin': 'X86_V3 be2539f885177d33', 'cos': 'X86_V3 6b7bce834c8e7a76',
                      'tan': 'baseline(X86_V2) c8529ff8ebd78682', 'exp': 'X86_V3 e27ada39f6ac4dfa',
                      'expm1': 'baseline(X86_V2) 0be8573280b39685', 'log': 'X86_V3 665ed5d0ed26fd0f',
                      'log1p': 'baseline(X86_V2) 90de35f95bf1c7cd', 'arcsin': 'baseline(X86_V2) 191f78865ba12646',
                      'arccos': 'baseline(X86_V2) 8f50e46b62c63046', 'arctan': 'baseline(X86_V2) c4e473af7db8ab87',
                      'arctan2': 'baseline(X86_V2) d2c1fb88e72f0958', 'hypot': '? 15b2477691bdc25b',
                      'power': 'baseline(X86_V2) 880365b107a38193', 'power2': 'baseline(X86_V2) 0c7e5fe3ffa63729',
                      'tanh': 'X86_V3 359b7e80c2d37e86'},
}
#: OpenBLAS's products and solves by system, processor and kernels; numpy's LAPACK routines (LAPACK) likewise.
BLAS = {
    'Linux-aarch64-armv9sme': {'gemm': '13e8b3d08e515631', 'gemv': '82780987130da300', 'solve': '3cc6a51c24ec2a6d',
                               'cholesky': '6ccc5edd162c4dca', 'square': '7b69d3c0cb59fa00'},
    'Linux-x86_64-Haswell': {'gemm': '715bea6357929cd0', 'gemv': 'ace4320175be5d93', 'solve': 'f85019fa04c5829c',
                             'cholesky': '8f831a674ca91271', 'square': '7b69d3c0cb59fa00'},
    'Windows-AMD64-Haswell': {'gemm': '715bea6357929cd0', 'gemv': 'ace4320175be5d93', 'solve': 'f85019fa04c5829c',
                              'cholesky': '8f831a674ca91271', 'square': '7b69d3c0cb59fa00'},
}
LAPACK = ('qr', 'lstsq', 'svd', 'eigh', 'solve', 'cholesky', 'inv')
LAPACKS = {
    'Linux-aarch64-armv9sme': {'qr': 'de6eac875f1eb729', 'lstsq': '76cd8a11a9070579', 'svd': '779c2931677126aa',
                               'eigh': '512d4348030df0f0', 'solve': 'eabac25f9ec7a5f5',
                               'cholesky': '5d17b5e2dd02f5d6', 'inv': '6d177302d3726333'},
    'Linux-x86_64-Haswell': {'qr': '8bed43d61498a2d3', 'lstsq': 'e514ea2de649877b', 'svd': '30148a81e9d99c48',
                             'eigh': '2f10ff77d6776a71', 'solve': '027bf1c3aa66bc84', 'cholesky': '11e6a989db9d8ce6',
                             'inv': '9951226ce0f5cade'},
    'Windows-AMD64-Haswell': {'qr': '1f90308bd2418e00', 'lstsq': '2958b643384928fd', 'svd': '570a694069432b8b',
                              'eigh': '82ab835c7519b03f', 'solve': '027bf1c3aa66bc84', 'cholesky': '11e6a989db9d8ce6',
                              'inv': '9951226ce0f5cade'},
}
#: MuJoCo's compiler's turns of SCENE's geoms by system and processor.
TURNS = {
    'Linux-aarch64': {'turns': 'fb5ad14be5cb1784'},
    'Linux-x86_64': {'turns': 'fb5ad14be5cb1784'},
    'Windows-AMD64': {'turns': 'ac8f2de3b6ae3690'},
}
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
OWN = ('sin', 'cos', 'tan', 'exp', 'expm1', 'log', 'arcsin', 'arccos', 'arctan', 'arctan2', 'hypot')
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
    operations alone: by system, processor and OpenBLAS's kernels, every target alike (avx, avx2 and native one digest on
    the desk)."""
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
    _held(report, "the robot's QP in C (%s)" % os.environ.get('COAXIAL_ISA', 'native'), QPS, '%s-%s' % (_class(), _kernels()),
          {'qp': _digest(np.concatenate(out))})


def test_an_x86_is_held_to_the_desks_arithmetic(report):
    """Under the relay an x86-64 runs OpenBLAS's Haswell kernels where it can and no numpy path past X86_V3
    (`tools.dev.arithmetic`): GitHub's AVX-512 runners' own kernels parted their walks from the desk's."""
    from tools.dev import arithmetic
    held = arithmetic.held()
    if not held:
        report.check('not an x86-64 (%s): its own arithmetic, held to its class' % _class(), True)
        return
    kernels, wider = _kernels(), sorted(set(_targets().values()) & set(arithmetic.WIDER))
    report.check("an x86-64's OpenBLAS runs the desk's kernels and numpy no path past X86_V3",
                 kernels == held.get('OPENBLAS_CORETYPE', kernels) and not wider, 'kernels %s, wider paths %s' % (kernels, wider))


ROSTER = (test_basic_operations_and_square_root_alike, test_the_platforms_libm, test_numpys_float64_functions,
          test_openblas_products_and_solves, test_lapacks_factorings, test_mujocos_compiler_turns, test_the_robots_qp_in_c,
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
