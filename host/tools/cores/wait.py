"""The rows of her table that wait, in C through ctypes (wbc_wait.c), her python's bit for bit.

    k, ask = motion.laid()
    waits = wait.laid(k, a, out, y, fr, st)                      # machine.motion.held_rows, risen_rows
    shift, first, held_s = waits.held(tab, t, dt, held_s)        # tab's times shifted in place
    shift, first, prop_until = waits.risen(tab, t, dt, prop_until)
"""
import ctypes
import functools

import numpy as np

from tools.cores import wbc


class Wait(ctypes.Structure):

    """wbc_wait_t: her table's layout and the waits' constants."""

    _fields_ = ([(name, ctypes.c_int) for name in ('table', 'at', 'vals_z', 'said_z', 'vals_gain', 'said_gain', 'st_to_z')]
                + [(name, ctypes.c_double) for name in ('edge_s', 'tiny', 'phi_w')])


@functools.lru_cache(maxsize=None)
def _typed():
    """The waits typed, wbc_wait_t's size the C's."""
    lib = wbc.library()
    lib.wbh_wait_size.restype = ctypes.c_int
    if lib.wbh_wait_size() != ctypes.sizeof(Wait):
        raise ValueError('wbc_wait_t is %d bytes, its mirror %d' % (lib.wbh_wait_size(), ctypes.sizeof(Wait)))
    lib.wbh_wait_rows.restype = ctypes.c_int
    lib.wbh_wait_held.argtypes = ([ctypes.c_void_p, ctypes.POINTER(Wait), ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p,
                                   ctypes.c_void_p] + [ctypes.c_double] * 3 + [ctypes.c_void_p])
    lib.wbh_wait_risen.argtypes = ([ctypes.c_void_p, ctypes.POINTER(Wait), ctypes.c_void_p, ctypes.c_int]
                                   + [ctypes.c_void_p] * 4 + [ctypes.c_double] * 3 + [ctypes.c_void_p])
    lib.wbh_wait_held.restype = lib.wbh_wait_risen.restype = None
    return lib.wbh_wait_held, lib.wbh_wait_risen, int(lib.wbh_wait_rows())


class Waits:

    """The waits in C on her law's arrays, pointed at once; her table given at each, rows at most
    `rows` (wbc_wait.c's)."""

    def __init__(self, k, a, out, y, fr, st):
        from machine import latent, stride
        self.w = Wait(table=latent.TABLE, at=latent.AT, vals_z=latent.VALS.start + latent.Z.start,
                      said_z=latent.SAID.start + latent.Z.start, vals_gain=latent.VALS.start + latent.GAIN,
                      said_gain=latent.SAID.start + latent.GAIN, st_to_z=latent.TO.start + latent.Z.start,
                      edge_s=latent.EDGE_S, tiny=stride.TINY, phi_w=stride.PHI_W)
        self.kept = (a, out, y, fr, st)
        self.at = [wbc.pointer(arr) for arr in self.kept]
        self.got = np.zeros(3)
        self.held_c, self.risen_c, self.rows = _typed()
        self.k, self.wp, self.gp = ctypes.addressof(k), ctypes.byref(self.w), self.got.ctypes.data

    def _table(self, tab):
        if len(tab) > self.rows:
            raise ValueError('wbc_wait.c lays %d rows at most, her table has %d' % (self.rows, len(tab)))
        return wbc.pointer(tab) if len(tab) else None

    def held(self, tab, t, dt, held_s):
        """(shift, first, held_s) of `machine.motion.held_rows`, `tab`'s times shifted in place."""
        self.held_c(self.k, self.wp, self._table(tab), len(tab), self.at[0], self.at[1], t, dt, held_s, self.gp)
        return tuple(self.got.tolist())

    def risen(self, tab, t, dt, prop_until):
        """(shift, first, prop_until) of `machine.motion.risen_rows`, `tab`'s times shifted in place."""
        self.risen_c(self.k, self.wp, self._table(tab), len(tab), self.at[2], self.at[3], self.at[0], self.at[4], t, dt,
                     prop_until, self.gp)
        return tuple(self.got.tolist())


def laid(k, a, out, y, fr, st):
    """The waits on modal's `a`, the latent `out`, modal's `y` and `fr`, her latent `st`; `k` the ask's
    (`tools.cores.motion.laid`)."""
    return Waits(k, a, out, y, fr, st)
