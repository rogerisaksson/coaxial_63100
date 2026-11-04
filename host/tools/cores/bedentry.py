"""Her way into a bed over its end to its middle in C through ctypes (wbc_bedentry.c), bit for bit.

`machine.moves.bedentry`'s `laid` and `step` on its tables, the move her loop plays (`machine.process.joints.MOVES`).

    state = bedentry.laid(q_deg, t)                                                    # from where she sits
    done = bedentry.step(state, cmd, q_deg, t, dt, soles, along, half, pitch, speed)   # cmd (deg) laid over
"""
import ctypes
import functools

import numpy as np

from machine.moves import bedentry as py
from tools.cores import wbc

#: What her loop gives a pass beyond her pose, and her text going on while it plays (`machine.moves.bedentry`).
SENSED, HOLDS = py.SENSED, py.HOLDS
P, Q, M = len(py.PHASES), len(py.PARTS), len(py.JOINT)


class Entry(ctypes.Structure):

    """wbc_bedentry_t: the move's tables (wbc_move.h)."""

    _fields_ = ([('joint', ctypes.c_int * M), ('part', ctypes.c_int * M), ('over', ctypes.c_int * P),
                 ('done', ctypes.c_int)]
                + [(name, ctypes.c_double * P) for name in ('next', 'again', 'counts')]
                + [(name, (ctypes.c_double * Q) * P) for name in ('goal', 'how', 'rate')]
                + [(name, ctypes.c_double) for name in ('plant', 'pull_in', 'still', 'lain', 'near', 'strokes', 'tol')])


@functools.lru_cache(maxsize=None)
def _laid():
    """The library typed and the move's tables filled from `machine.moves.bedentry`, wbc_bedentry_t's size the C's."""
    lib = wbc.library()
    lib.wbh_bedentry_size.restype = ctypes.c_int
    if lib.wbh_bedentry_size() != ctypes.sizeof(Entry):
        raise ValueError('wbc_bedentry_t is %d bytes, its mirror %d' % (lib.wbh_bedentry_size(), ctypes.sizeof(Entry)))
    lib.wbh_bedentry_laid.argtypes, lib.wbh_bedentry_laid.restype = [ctypes.c_void_p], None
    lib.wbh_bedentry_step.argtypes = [ctypes.POINTER(Entry)] + [ctypes.c_void_p] * 2 + [ctypes.c_double] * 6
    lib.wbh_bedentry_step.restype = ctypes.c_double
    cells, ints = np.ctypeslib.as_ctypes, (lambda v: (ctypes.c_int * len(v))(*v.tolist()))
    tables = Entry(joint=ints(py.JOINT), part=ints(py.PART), over=ints(py.OVER), done=py.DONE, next=cells(py.NEXT),
                   again=cells(py.AGAIN), counts=cells(py.COUNTS), goal=cells(py.GOAL), how=cells(py.HOW),
                   rate=cells(py.RATES), plant=py.PLANT, pull_in=py.PULL_IN, still=py.STILL, lain=py.LAIN,
                   near=py.NEAR, strokes=py.STROKES, tol=py.TOL)
    return lib, tables


def laid(q_deg, t):
    """`machine.moves.bedentry.laid` in C: the move's state as it begins."""
    lib, _tables = _laid()
    state = np.zeros(py.WAY + 1)
    lib.wbh_bedentry_laid(state.ctypes.data)
    return state


def step(state, cmd, q_deg, t, dt, soles, along, half, pitch, speed):
    """`machine.moves.bedentry.step` in C on `state` and `cmd` (contiguous float64, laid over in place) and her senses."""
    lib, tables = _laid()
    return lib.wbh_bedentry_step(ctypes.byref(tables), state.ctypes.data, cmd.ctypes.data, dt, soles, along, half,
                                 pitch, speed)
