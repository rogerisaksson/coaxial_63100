"""The arithmetic every x86-64 host is held to: numpy's float64 functions on the desk's paths.

    from tools.dev import arithmetic
    env.update(arithmetic.held())     # {'NPY_DISABLE_CPU_FEATURES': 'X86_V4 AVX512_ICL AVX512_SPR'} where dispatched

numpy dispatches its float64 sin, cos, exp, log and tanh to its AVX-512 paths where its build has them; held to its
paths no wider than X86_V3, every x86-64 host computes them as the desk does (tests/test_platform.py holds it); an Arm
is its own, nothing held. The relay lays it on every job (`tools.dev.focus.environment`), the gate's runners and the
Linux containers with it. OpenBLAS's kernels stay its own pick, each class keyed by them in test_platform: held to
Haswell's (OPENBLAS_CORETYPE) on GitHub's AVX-512 and Zen runners, the robot's cores calling its products crashed
(SIGSEGV) in seven suites at f7be13ab.
"""
import platform

#: numpy's dispatch targets past X86_V3: its AVX-512 sets, disabled where its build dispatches them (a target the
#: build has not, named, is a warning on every process's start: Windows' numpy has none).
WIDER = ('X86_V4', 'AVX512_ICL', 'AVX512_SPR')
#: The x86-64 machine's names as Python's platform has them.
X86 = ('x86_64', 'amd64')


def held(machine=None, dispatch=None):
    """The environment holding this host's numpy to the desk's paths: {} off x86-64 or where its build dispatches
    nothing wider. `machine` and `dispatch` (numpy's targets) this host's where not given."""
    machine = (machine or platform.machine()).lower()
    if machine not in X86:
        return {}
    if dispatch is None:
        import numpy as np
        dispatch = getattr(getattr(getattr(np, '_core', None), '_multiarray_umath', None), '__cpu_dispatch__', ())
    wider = ' '.join(target for target in WIDER if target in dispatch)
    return {'NPY_DISABLE_CPU_FEATURES': wider} if wider else {}
