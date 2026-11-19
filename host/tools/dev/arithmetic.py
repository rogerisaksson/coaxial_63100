"""The arithmetic every x86-64 host is held to: OpenBLAS's Nehalem kernels, numpy's paths to X86_V3.

    from tools.dev import arithmetic
    env.update(arithmetic.held())     # {'OPENBLAS_CORETYPE': 'Nehalem', 'NPY_DISABLE_CPU_FEATURES': ...}

OpenBLAS picks its kernels by the processor - Haswell's on the desk's AMD and on half of GitHub's runners, SkylakeX's
on the rest (EPYC 9V74 and 9V45 showing AVX-512, Xeon 8370C, 8573C, 6975P-C) - and their products, solves and
factorings part in the last bit: a knife edge of the robot's walk drawn one way on one runner, the other on the next.
Held to Nehalem's (SSE2, no FMA) every x86-64 runs the same kernels, probed on each of those runners alike; Haswell's
cannot be held on AMD's Zen 4 and 5 - OpenBLAS lays its Zen blocking (setparam-ref.c: P and Q 512) on whatever kernels
are asked there, and Haswell's jumped to address 0 in a large product (numpy 2.5.4 and 2.5.5 alike). numpy dispatches
its float64 sin, cos, exp, log and tanh to its AVX-512 paths where its build has them: held no wider than X86_V3.
tests/test_platform.py holds both; an Arm is its own, nothing held. The relay lays it on every job
(`tools.dev.focus.environment`), the gate's runners and the Linux containers with it.
"""
import platform

#: numpy's dispatch targets past X86_V3: its AVX-512 sets, disabled where its build dispatches them (a target the
#: build has not, named, is a warning on every process's start: Windows' numpy has none).
WIDER = ('X86_V4', 'AVX512_ICL', 'AVX512_SPR')
#: The x86-64 machine's names as Python's platform has them; the OpenBLAS kernels every one of them runs.
X86, CORE = ('x86_64', 'amd64'), 'Nehalem'


def held(machine=None, dispatch=None):
    """The environment holding this host's OpenBLAS and numpy to every x86-64's arithmetic: {} off x86-64.
    `machine` and `dispatch` (numpy's targets) this host's where not given."""
    machine = (machine or platform.machine()).lower()
    if machine not in X86:
        return {}
    if dispatch is None:
        import numpy as np
        dispatch = getattr(getattr(getattr(np, '_core', None), '_multiarray_umath', None), '__cpu_dispatch__', ())
    wider = ' '.join(target for target in WIDER if target in dispatch)
    return {'OPENBLAS_CORETYPE': CORE, **({'NPY_DISABLE_CPU_FEATURES': wider} if wider else {})}
