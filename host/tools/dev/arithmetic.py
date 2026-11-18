"""The arithmetic every x86-64 host is held to: one machine's products and functions on every one.

    from tools.dev import arithmetic
    env.update(arithmetic.held())     # {'OPENBLAS_CORETYPE': 'Haswell', 'NPY_DISABLE_CPU_FEATURES': ...}

OpenBLAS picks its kernels by the processor - Haswell's on the desk's AMD, SkylakeX's on GitHub's AVX-512 runners
(Xeon 8573C, EPYC 9V74) - and their products part in the last bit; numpy dispatches its float64 sin, cos, exp, log
and tanh to its AVX-512 paths where its build has them. Held to OpenBLAS's Haswell kernels where the processor runs
them (AVX2 and FMA3) and to numpy's paths no wider than X86_V3, every x86-64 host computes those as the desk does
(tests/test_platform.py holds it); an Arm is its own, nothing held. The relay lays it on every job
(`tools.dev.focus.environment`), the gate's runners and the Linux containers with it.
"""
import platform

#: numpy's dispatch targets past X86_V3: its AVX-512 sets, disabled where its build dispatches them (a target the
#: build has not, named, is a warning on every process's start: Windows' numpy has none).
WIDER = ('X86_V4', 'AVX512_ICL', 'AVX512_SPR')
#: The x86-64 machine's names as Python's platform has them; OpenBLAS's kernels held to, and what they need.
X86 = ('x86_64', 'amd64')
CORE, NEEDS = 'Haswell', ('AVX2', 'FMA3')


def held(machine=None, features=None, dispatch=None):
    """The environment holding this host's OpenBLAS and numpy to the desk's arithmetic: {} off x86-64.
    `machine`, `features` ({set: on}) and `dispatch` (numpy's targets) this host's where not given."""
    machine = (machine or platform.machine()).lower()
    if machine not in X86:
        return {}
    if features is None or dispatch is None:
        import numpy as np
        umath = getattr(getattr(np, '_core', None), '_multiarray_umath', None)
        features = getattr(umath, '__cpu_features__', {}) if features is None else features
        dispatch = getattr(umath, '__cpu_dispatch__', ()) if dispatch is None else dispatch
    out = {'OPENBLAS_CORETYPE': CORE} if all(features.get(name) for name in NEEDS) else {}
    wider = ' '.join(target for target in WIDER if target in dispatch)
    return {**out, 'NPY_DISABLE_CPU_FEATURES': wider} if wider else out
