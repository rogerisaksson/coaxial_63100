"""Her way out of a bed over its end in C through ctypes (wbc_bedexit.c), bit for bit.

`machine.moves.bedexit`'s `laid` and `step` on its tables, the move her loop plays (`machine.process.joints.MOVES`).

    state = bedexit.laid(q_deg, t)                                     # from where she lies, at t s
    done = bedexit.step(state, cmd, q_deg, t, dt, soles, shins, pitch)  # cmd (deg, figure.JOINTS) laid over
"""
import ctypes
import functools

import numpy as np

from machine.moves import bedexit as py
from tools.cores import wbc

P, Q, M = len(py.PHASES), len(py.PARTS), len(py.JOINT)


class Exit(ctypes.Structure):

    """wbc_bedexit_t: the move's tables (wbc_move.h)."""

    _fields_ = ([('joint', ctypes.c_int * M), ('part', ctypes.c_int * M)]
                + [(name, ctypes.c_int) for name in ('joints', 'pushed', 'stretched', 'seated', 'done', 'spare')]
                + [(name, ctypes.c_double * P) for name in ('length', 'next', 'trunk', 'holds')]
                + [(name, (ctypes.c_double * Q) * P) for name in ('goal', 'how', 'rate', 'over', 'how_over', 'rate_over')]
                + [(name, ctypes.c_double) for name in ('trunk_gain', 'hip_lo', 'hip_hi', 'shin_share', 'sole_share',
                                                        'knee_seat', 'strokes')])


def _rows(table):
    return ((ctypes.c_double * Q) * P)(*[(ctypes.c_double * Q)(*row) for row in table.tolist()])


def _cells(vector):
    return (ctypes.c_double * P)(*vector.tolist())


@functools.lru_cache(maxsize=None)
def _laid():
    """The library typed and the move's tables filled from `machine.moves.bedexit`, wbc_bedexit_t's size the C's."""
    lib = wbc.library()
    lib.wbh_bedexit_size.restype = ctypes.c_int
    if lib.wbh_bedexit_size() != ctypes.sizeof(Exit):
        raise ValueError('wbc_bedexit_t is %d bytes, its mirror %d' % (lib.wbh_bedexit_size(), ctypes.sizeof(Exit)))
    lib.wbh_bedexit_laid.argtypes = [ctypes.POINTER(Exit), ctypes.c_void_p, ctypes.c_double, ctypes.c_void_p]
    lib.wbh_bedexit_laid.restype = None
    lib.wbh_bedexit_step.argtypes = [ctypes.POINTER(Exit)] + [ctypes.c_void_p] * 3 + [ctypes.c_double] * 5
    lib.wbh_bedexit_step.restype = ctypes.c_double
    tables = Exit(joint=(ctypes.c_int * M)(*py.JOINT.tolist()), part=(ctypes.c_int * M)(*py.PART.tolist()),
                  joints=len(py.JOINTS), pushed=py.PUSHED, stretched=py.STRETCHED_AT, seated=py.SEATED, done=py.DONE,
                  length=_cells(py.LENGTH), next=_cells(py.NEXT), trunk=_cells(py.TRUNKS), holds=_cells(py.HOLDS),
                  goal=_rows(py.GOAL), how=_rows(py.HOW), rate=_rows(py.RATE), over=_rows(py.GOAL_OVER),
                  how_over=_rows(py.HOW_OVER), rate_over=_rows(py.RATE_OVER), trunk_gain=py.TRUNK_GAIN, hip_lo=py.HIP_LO,
                  hip_hi=py.HIP_HI, shin_share=py.SHIN_SHARE, sole_share=py.SOLE_SHARE, knee_seat=py.KNEE_SEAT,
                  strokes=py.STROKES)
    return lib, tables


def laid(q_deg, t):
    """`machine.moves.bedexit.laid` in C: the move's state from her pose `q_deg` (deg) at `t` s."""
    lib, tables = _laid()
    q = np.ascontiguousarray(q_deg, dtype=float)
    state = np.zeros(py.FROM + len(py.JOINTS))
    lib.wbh_bedexit_laid(ctypes.byref(tables), q.ctypes.data, t, state.ctypes.data)
    return state


def step(state, cmd, q_deg, t, dt, soles, shins, pitch):
    """`machine.moves.bedexit.step` in C on `state` and `cmd` (contiguous float64, laid over in place), her pose
    `q_deg` (deg) and her senses."""
    lib, tables = _laid()
    q = np.ascontiguousarray(q_deg, dtype=float)
    return lib.wbh_bedexit_step(ctypes.byref(tables), state.ctypes.data, cmd.ctypes.data, q.ctypes.data, t, dt, soles,
                                shins, pitch)
