"""The rows of her table that wait or wake her, in C through ctypes (wbc_wait.c), bit for bit.

    k, ask = motion.laid()
    waits = wait.laid(k, a, out, y, fr, st)                      # machine.waits.held_rows, risen_rows, woken_rows
    shift, first, held_s = waits.held(tab, t, dt, held_s)        # tab's times shifted in place
    shift, first, prop_until = waits.risen(tab, t, dt, prop_until)
    shift, first = waits.borne(tab, t, dt)                       # machine.waits.borne_rows
    waits.latched(tab, t, dt)                                    # machine.waits.latched_rows: her vector in st
    waits.woken(tab, t, dt)                                      # her rhythm woken in st
"""
import ctypes
import functools

import numpy as np

from tools.cores import motion as c_motion, wbc


class Wait(ctypes.Structure):

    """wbc_wait_t: her table's layout, the waits' constants, numpy's BLAS."""

    _fields_ = ([(name, ctypes.c_int) for name in ('table', 'at', 'vals_z', 'said_z', 'vals_gain', 'said_gain', 'st_to_z',
                                                   'contact', 'vals_amp', 'said_phase', 'vals_phase', 'st_now_gain',
                                                   'st_now_phase', 'st_theta', 'latch', 'st_now_z', 'st_from_z',
                                                   'st_v0_z', 'st_v1_z', 'rise')]
                + [(name, ctypes.c_double) for name in ('edge_s', 'tiny', 'phi_w')] + [('blas', c_motion.Blas)])


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
    lib.wbh_wait_landed.argtypes = ([ctypes.c_void_p, ctypes.POINTER(Wait), ctypes.c_void_p, ctypes.c_int]
                                    + [ctypes.c_void_p] * 2 + [ctypes.c_double] * 2 + [ctypes.c_void_p])
    lib.wbh_wait_sunk.argtypes = ([ctypes.c_void_p, ctypes.POINTER(Wait), ctypes.c_void_p, ctypes.c_int]
                                  + [ctypes.c_void_p] * 2 + [ctypes.c_double] * 2 + [ctypes.c_void_p])
    lib.wbh_wait_sunk.restype = None
    lib.wbh_wait_borne.argtypes = ([ctypes.c_void_p, ctypes.POINTER(Wait), ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
                                   + [ctypes.c_double] * 2 + [ctypes.c_void_p])
    lib.wbh_wait_woken.argtypes = ([ctypes.c_void_p, ctypes.POINTER(Wait), ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
                                   + [ctypes.c_double] * 2)
    lib.wbh_wait_latched.argtypes = ([ctypes.c_void_p, ctypes.POINTER(Wait), ctypes.c_void_p, ctypes.c_int]
                                     + [ctypes.c_void_p] * 4 + [ctypes.c_double] * 2)
    lib.wbh_wait_held.restype = lib.wbh_wait_risen.restype = lib.wbh_wait_landed.restype = None
    lib.wbh_wait_borne.restype = lib.wbh_wait_latched.restype = None
    lib.wbh_wait_woken.restype = None
    return (lib.wbh_wait_held, lib.wbh_wait_risen, lib.wbh_wait_landed, lib.wbh_wait_borne, lib.wbh_wait_latched,
            lib.wbh_wait_woken, int(lib.wbh_wait_rows()), lib.wbh_wait_sunk)


class Waits:

    """The waits in C on her law's arrays, pointed at once; her table given at each, rows at most
    `rows` (wbc_wait.c's), an empty one a laid row read nowhere."""

    def __init__(self, k, a, out, y, fr, st):
        from machine import latent, stride
        self.w = Wait(table=latent.TABLE, at=latent.AT, vals_z=latent.VALS.start + latent.Z.start,
                      said_z=latent.SAID.start + latent.Z.start, vals_gain=latent.VALS.start + latent.GAIN,
                      contact=latent.CONTACT,
                      said_gain=latent.SAID.start + latent.GAIN, st_to_z=latent.TO.start + latent.Z.start,
                      vals_amp=latent.VALS.start + latent.AMP.start,
                      said_phase=latent.SAID.start + latent.PHASE.start, vals_phase=latent.VALS.start + latent.PHASE.start,
                      st_now_gain=latent.NOW.start + latent.GAIN, st_now_phase=latent.NOW.start + latent.PHASE.start,
                      st_theta=latent.THETA, latch=latent.LATCH, st_now_z=latent.NOW.start + latent.Z.start,
                      st_from_z=latent.FROM.start + latent.Z.start, st_v0_z=latent.V0.start + latent.Z.start,
                      st_v1_z=latent.V1.start + latent.Z.start, rise=latent.RISES, edge_s=latent.EDGE_S, tiny=stride.TINY,
                      phi_w=stride.PHI_W,
                      blas=c_motion.numpy_blas())
        self.kept = (a, out, y, fr, st, np.zeros((1, latent.TABLE)))
        self.at = [wbc.pointer(arr) for arr in self.kept]
        self.got = np.zeros(3)
        (self.held_c, self.risen_c, self.landed_c, self.borne_c, self.latched_c, self.woken_c, self.rows,
         self.sunk_c) = _typed()
        self.k, self.wp, self.gp = ctypes.addressof(k), ctypes.byref(self.w), self.got.ctypes.data

    def _table(self, tab):
        if len(tab) > self.rows:
            raise ValueError('wbc_wait.c lays %d rows at most, her table has %d' % (self.rows, len(tab)))
        return wbc.pointer(tab) if len(tab) else self.at[5]

    def held(self, tab, t, dt, held_s):
        """(shift, first, held_s) of `machine.waits.held_rows`, `tab`'s times shifted in place."""
        self.held_c(self.k, self.wp, self._table(tab), len(tab), self.at[0], self.at[1], t, dt, held_s, self.gp)
        return tuple(self.got.tolist())

    def risen(self, tab, t, dt, prop_until):
        """(shift, first, prop_until) of `machine.waits.risen_rows`, `tab`'s times shifted in place."""
        self.risen_c(self.k, self.wp, self._table(tab), len(tab), self.at[2], self.at[3], self.at[0], self.at[4], t, dt,
                     prop_until, self.gp)
        return tuple(self.got.tolist())

    def landed(self, tab, t, dt):
        """(shift, first) of `machine.waits.landed_rows`, `tab`'s times shifted in place."""
        self.landed_c(self.k, self.wp, self._table(tab), len(tab), self.at[2], self.at[4], t, dt, self.gp)
        return tuple(self.got[:2].tolist())

    def sunk(self, tab, t, dt):
        """(shift, first) of `machine.waits.sunk_rows`, `tab`'s times shifted in place."""
        self.sunk_c(self.k, self.wp, self._table(tab), len(tab), self.at[2], self.at[3], t, dt, self.gp)
        return tuple(self.got[:2].tolist())

    def borne(self, tab, t, dt):
        """(shift, first) of `machine.waits.borne_rows`, `tab`'s times shifted in place."""
        self.borne_c(self.k, self.wp, self._table(tab), len(tab), self.at[2], t, dt, self.gp)
        return tuple(self.got[:2].tolist())

    def latched(self, tab, t, dt):
        """`machine.waits.latched_rows`: her vector latched in `st` where she is by the rows of `tab` due."""
        self.latched_c(self.k, self.wp, self._table(tab), len(tab), self.at[2], self.at[3], self.at[0], self.at[4], t,
                       dt)

    def woken(self, tab, t, dt):
        """`machine.waits.woken_rows`: her rhythm woken in `st` by the rows of `tab` due."""
        self.woken_c(self.k, self.wp, self._table(tab), len(tab), self.at[4], t, dt)


def laid(k, a, out, y, fr, st):
    """The waits on modal's `a`, the latent `out`, modal's `y` and `fr`, her latent `st`; `k` the ask's
    (`tools.cores.motion.laid`)."""
    return Waits(k, a, out, y, fr, st)
