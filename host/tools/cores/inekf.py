"""inekf.stepped's arithmetic around its points and step in C (wbc_inekf.c), bit for bit.

    g['inekf_glue'] = inekf.glued()          # laid by machine.law.motion.laid; None its python (`inekf.stepped`)
    local, weights, sh = g['inekf_glue'].bearing(y, shares)     # machine.senses.inekf.bearing's
    g['inekf_glue'].rolled(f, soles, anchors, sh, dt)           # machine.senses.inekf.rolled's
"""
import ctypes
import functools

import numpy as np

from tools.cores import wbc

_i, _d, _vp = ctypes.c_int, ctypes.c_double, ctypes.c_void_p


class Inekf(ctypes.Structure):

    """wbc_inekf_t: each offset into the robot's edge's array, its core's segments and the touch its shares hold to."""

    _fields_ = ([(name, _i) for name in ('touch', 'cop', 'press', 'at')] + [('core', _i * 2)]
                + [(name, _i) for name in ('insoles', 'spare')] + [(name, _d) for name in ('touched_n', 'release')])


@functools.lru_cache(maxsize=None)
def _typed():
    """The arithmetic's C typed, wbc_inekf_t's size the C's."""
    lib = wbc.library()
    lib.wbh_inekf_size.restype = _i
    if lib.wbh_inekf_size() != ctypes.sizeof(Inekf):
        raise ValueError('wbc_inekf_t is %d bytes, its mirror %d' % (lib.wbh_inekf_size(), ctypes.sizeof(Inekf)))
    lib.wbh_inekf_laid.argtypes = [ctypes.POINTER(Inekf), _vp, _vp, _i, _vp, _vp, _vp]
    lib.wbh_inekf_laid.restype = None
    lib.wbh_inekf_rolled.argtypes = [ctypes.POINTER(Inekf), _vp, _vp, _vp, _vp, _d, _d, _vp]
    lib.wbh_inekf_rolled.restype = _d
    return lib.wbh_inekf_laid, lib.wbh_inekf_rolled


class Glued:

    """`inekf.stepped`'s arithmetic in C on its arrays: a law pass's points laid into one of two buffers in turn -
    the filter holds the last pass's (`f['last']`) while this one's are laid -, its drift rolled on; numpy's
    arrays around them were 15 us of each law pass."""

    def __init__(self):
        from machine.senses import edge, inekf, observer
        self.k = Inekf(touch=edge.TOUCH.start, cop=edge.COP.start, press=edge.PRESS.start, at=edge.AT.start,
                       insoles=len(edge.INSOLES), touched_n=edge.TOUCHED_N, release=observer.RELEASE)
        self.k.core[:] = [int(c) for c in inekf.CORE]
        rows = len(edge.INSOLES) + len(inekf.CORE)
        self.bufs = [(np.zeros((rows, 3)), np.zeros(rows), np.zeros(inekf.CONTACTS)) for _ in range(2)]
        self.turn, self.at, self.fns = 0, ctypes.byref(self.k), _typed()
        from machine import libm
        self.gyro2, self.roll, self.sq = inekf.GYRO * inekf.GYRO, inekf.ROLL_M_S, libm.sq

    def bearing(self, y, shares):
        """`inekf.bearing` off the robot's edge's `y` and each contact's `shares` (3 at most), into this pass's
        buffers."""
        self.turn ^= 1
        local, weights, sh = self.bufs[self.turn]
        shares = np.ascontiguousarray(shares, dtype=np.float64)
        self.fns[0](self.at, wbc.address(y), wbc.address(shares), len(shares), wbc.address(local), wbc.address(weights),
                    wbc.address(sh))
        return local, weights, sh

    def rolled(self, f, soles, anchors, sh, dt):
        """`inekf.rolled`: the filter `f`'s drift a law pass `dt` on, its gyro's and its contacts' roll."""
        soles, anchors = np.ascontiguousarray(soles, dtype=np.float64), np.ascontiguousarray(anchors, dtype=np.float64)
        on = np.ascontiguousarray(f['on'], dtype=np.float64)
        return self.fns[1](self.at, wbc.address(soles), wbc.address(anchors), wbc.address(sh), wbc.address(on),
                           self.sq(self.roll * dt), self.gyro2 * dt, wbc.address(f['drift']))


def glued():
    """`inekf.stepped`'s arithmetic in C (`Glued`), laid once a session."""
    return Glued()
