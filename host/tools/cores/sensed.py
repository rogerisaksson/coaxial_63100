"""Her moves whose phases end by what she senses in C through ctypes (wbc_sensed.c), bit for bit.

`machine.moves.sensed`'s `laid` and `step` on a move's tables, the moves her loop plays (`machine.process.joints.MOVES`):
each move's tables laid in C's own once, its phases first and the rest of them, past the last, zero.

    move = sensed.move('bed-entry')                                                 # its tables in C
    state = move.laid(q_deg, t)                                                     # from where she sits
    done = move.step(state, cmd, q_deg, t, dt, soles, along, half, pitch, speed)    # cmd (deg) laid over
"""
import ctypes
import functools

import numpy as np

from machine.errors import MachineError
from machine.moves import sensed as py
from tools.cores import wbc

#: A move's phases at most (wbc_move.h's WBC_SENSED_PHASES), the parts it moves and their joints.
P, Q, M = 15, len(py.PARTS), len(py.JOINT)


class Tables(ctypes.Structure):

    """wbc_sensed_t: a move's tables (wbc_move.h)."""

    _fields_ = ([('joint', ctypes.c_int * M), ('part', ctypes.c_int * M), ('over', ctypes.c_int * P),
                 ('done', ctypes.c_int)]
                + [(name, ctypes.c_double * P) for name in ('next', 'again', 'counts')]
                + [(name, (ctypes.c_double * Q) * P) for name in ('goal', 'how', 'rate')]
                + [(name, ctypes.c_double) for name in ('plant', 'pull_in', 'still', 'lain', 'near', 'strokes', 'tol')])


@functools.lru_cache(maxsize=1)
def _typed():
    """The library, its moves' laid and step typed, wbc_sensed_t's size the C's."""
    lib = wbc.library()
    lib.wbh_sensed_size.restype = ctypes.c_int
    if lib.wbh_sensed_size() != ctypes.sizeof(Tables):
        raise ValueError('wbc_sensed_t is %d bytes, its mirror %d' % (lib.wbh_sensed_size(), ctypes.sizeof(Tables)))
    lib.wbh_sensed_laid.argtypes, lib.wbh_sensed_laid.restype = [ctypes.c_void_p], None
    lib.wbh_sensed_step.argtypes = [ctypes.POINTER(Tables)] + [ctypes.c_void_p] * 2 + [ctypes.c_double] * 6
    lib.wbh_sensed_step.restype = ctypes.c_double
    return lib


@functools.lru_cache(maxsize=None)
def _laid(name):
    """(the library, move `name`'s tables in C) from `machine.moves.sensed`'s, refused past P phases."""
    t = py.tables(name)
    n = len(t.names)
    if n > P:
        raise MachineError('%s: %d phases, her C moves take %d at most' % (name, n, P))

    def padded(a):
        return np.concatenate([a, np.zeros((P - n,) + a.shape[1:], a.dtype)])
    cells, ints = np.ctypeslib.as_ctypes, (lambda v: (ctypes.c_int * len(v))(*v.tolist()))
    tables = Tables(joint=ints(py.JOINT), part=ints(py.PART), over=ints(padded(t.over)), done=t.done,
                    next=cells(padded(t.next)), again=cells(padded(t.again)), counts=cells(padded(t.counts)),
                    goal=cells(padded(t.goal)), how=cells(padded(t.how)), rate=cells(padded(t.rates)),
                    **dict(zip(('plant', 'pull_in', 'still', 'lain', 'near', 'strokes', 'tol'), t.numbers)))
    return _typed(), tables


class Move(py.Move):

    """Move `name` in C (`_laid`): `machine.moves.sensed.Move`'s, its `laid` and `step` the C's."""

    def _laid(self):
        return _laid(self.name)

    def laid(self, q_deg, t):
        """`machine.moves.sensed.laid` in C: the move's state as it begins."""
        lib, _tables = self._laid()
        state = np.zeros(py.WAY + 1)
        lib.wbh_sensed_laid(state.ctypes.data)
        return state

    def step(self, state, cmd, q_deg, t, dt, soles, along, half, pitch, speed):
        """`machine.moves.sensed.step` in C on `state` and `cmd` (contiguous float64, laid over in place) and her
        senses."""
        lib, tables = self._laid()
        return lib.wbh_sensed_step(ctypes.byref(tables), state.ctypes.data, cmd.ctypes.data, dt, soles, along, half,
                                   pitch, speed)


@functools.lru_cache(maxsize=None)
def move(name):
    """Move `name` in C (`Move`)."""
    return Move(name)
