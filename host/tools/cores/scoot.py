"""Her scoot on the floor in C through ctypes (wbc_scoot.c), bit for bit.

`machine.moves.scoot`'s `laid` and `step` on its tables, the move her loop plays (`machine.process.joints.MOVES`).

    state = scoot.laid(q_deg, t)                                                     # from where she sits
    scoot.step(state, cmd, q_deg, t, dt, pitch, hands, left, right, back, turn, heading)    # cmd laid over
"""
import ctypes
import functools

import numpy as np

from machine.moves import scoot as py
from tools.cores import wbc

#: What her loop gives a pass beyond her pose (`machine.moves.scoot.SENSED`).
SENSED = py.SENSED
P, Q, M = len(py.PHASES), len(py.PARTS), len(py.JOINT)


class Scoot(ctypes.Structure):

    """wbc_scoot_t: the move's tables (wbc_move.h)."""

    _fields_ = ([('joint', ctypes.c_int * M), ('part', ctypes.c_int * M), ('cycle', ctypes.c_int), ('stroking', ctypes.c_int),
                 ('over', ctypes.c_int * P), ('side', ctypes.c_double * M)]
                + [(name, (ctypes.c_double * Q) * P) for name in ('how', 'goal', 'rate')]
                + [(name, ctypes.c_double * P) for name in ('asked', 'idle')]
                + [(name, ctypes.c_double) for name in ('fold', 'stretch', 'settle', 'folded', 'trunk', 'gain', 'hip_lo',
                                                        'hip_hi', 'leant', 'stroke_rate', 'bear', 'skim', 'ask', 'tol',
                                                        'turn', 'stroke_turn', 'lead', 'upright')])


def _table(table):
    return ((ctypes.c_double * Q) * P)(*[(ctypes.c_double * Q)(*row) for row in table.tolist()])


def _cells(vector):
    return (ctypes.c_double * len(vector))(*vector.tolist())


def _ints(vector):
    return (ctypes.c_int * len(vector))(*vector.tolist())


@functools.lru_cache(maxsize=None)
def _laid():
    """The library typed and the move's tables filled from `machine.moves.scoot`, wbc_scoot_t's size the C's."""
    lib = wbc.library()
    lib.wbh_scoot_size.restype = ctypes.c_int
    if lib.wbh_scoot_size() != ctypes.sizeof(Scoot):
        raise ValueError('wbc_scoot_t is %d bytes, its mirror %d' % (lib.wbh_scoot_size(), ctypes.sizeof(Scoot)))
    lib.wbh_scoot_laid.argtypes, lib.wbh_scoot_laid.restype = [ctypes.c_void_p], None
    lib.wbh_scoot_step.argtypes = [ctypes.POINTER(Scoot)] + [ctypes.c_void_p] * 2 + [ctypes.c_double] * 8
    lib.wbh_scoot_step.restype = ctypes.c_double
    tables = Scoot(joint=_ints(py.JOINT), part=_ints(py.PART), cycle=py.CYCLE, stroking=py.STROKING,
                   over=_ints(py.OVER), side=_cells(py.SIDE), how=_table(py.HOW), goal=_table(py.GOAL),
                   rate=_table(py.RATES), asked=_cells(py.ASKED), idle=_cells(py.IDLE), fold=py.FOLD, stretch=py.STRETCH,
                   settle=py.SETTLE, folded=py.FOLDED, trunk=py.TRUNK, gain=py.GAIN, hip_lo=py.HIP_LO, hip_hi=py.HIP_HI,
                   leant=py.LEANT, stroke_rate=py.STROKE_RATE, bear=py.BEAR, skim=py.SKIM, ask=py.ASK, tol=py.TOL,
                   turn=py.TURN, stroke_turn=py.STROKE_TURN, lead=py.LEAD, upright=py.UPRIGHT)
    return lib, tables


def laid(q_deg, t):
    """`machine.moves.scoot.laid` in C: the move's state as it begins, bracing."""
    lib, _tables = _laid()
    state = np.zeros(py.AIMED + 1)
    lib.wbh_scoot_laid(state.ctypes.data)
    return state


def step(state, cmd, q_deg, t, dt, pitch, hands, left, right, back, turn, heading):
    """`machine.moves.scoot.step` in C on `state` and `cmd` (contiguous float64, laid over in place) and her senses."""
    lib, tables = _laid()
    return lib.wbh_scoot_step(ctypes.byref(tables), state.ctypes.data, cmd.ctypes.data, dt, pitch, hands, left, right,
                              back, turn, heading)
