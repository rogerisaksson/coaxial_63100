"""Her surety's brace on the solve in C through ctypes (wbc_brace.c), her python's bit for bit.

    brace = brace.laid(body, brace_rows, cv, acv, legs, free_kp)      # machine.brace.scaled, braced
    Ms = brace.scaled(M, w_up)                                       # the solve's M, the four dofs' inertia scaled off
    jeff, leg_kp = brace.blend(M, Ms, q, v, h, posture, qacc, tau, over, sw, bw, d_ax, d_lat, w_hop, w_up, stance)
"""
import ctypes
import functools

import numpy as np

from tools.cores import motion as c_motion, wbc


class Brace(ctypes.Structure):

    """wbc_brace_t: her body's indices, the tuck's rows, the gains, numpy's BLAS, the scratch."""

    _fields_ = ([(name, ctypes.c_int) for name in ('n', 'nd', 'ncv', 'nleg')]
                + [(name, ctypes.c_void_p) for name in ('act', 'acv', 'cv', 'leg_joints', 'leg_len', 'brace', 'clamp',
                                                        'free_kp', 'work')]
                + [(name, ctypes.c_double) for name in ('kd0', 'kd1', 'standing_kp')]
                + [('blas', c_motion.Blas)])


@functools.lru_cache(maxsize=None)
def _typed():
    """The brace typed, wbc_brace_t's size the C's."""
    lib = wbc.library()
    lib.wbh_brace_size.restype = ctypes.c_int
    if lib.wbh_brace_size() != ctypes.sizeof(Brace):
        raise ValueError('wbc_brace_t is %d bytes, its mirror %d' % (lib.wbh_brace_size(), ctypes.sizeof(Brace)))
    lib.wbh_brace_scaled.argtypes = [ctypes.POINTER(Brace)] + [ctypes.c_void_p] * 3
    lib.wbh_brace_blend.argtypes = ([ctypes.POINTER(Brace)] + [ctypes.c_void_p] * 9 + [ctypes.c_double] * 5
                                    + [ctypes.c_void_p] * 4)
    lib.wbh_brace_scaled.restype = lib.wbh_brace_blend.restype = None
    return lib.wbh_brace_scaled, lib.wbh_brace_blend


class Braced:

    """The brace in C on her body `b`: its driven dofs `act`, the cervical and shoulder joints `cv`
    among the driven and their dofs `acv`, the tuck's `rows` (3, driven), her `legs`' joints and
    their free stiffness `free_kp`; the arrays of a pass given at each call."""

    def __init__(self, b, rows, cv, acv, legs, free_kp):
        from machine import se2, wbc as law
        n, nd, ncv = b.n, len(b.act), len(cv)
        self.kept = [np.ascontiguousarray(a, dtype=np.int32) for a in (b.act, acv, cv, np.concatenate(legs),
                                                                       [len(leg) for leg in legs])]
        self.kept += [np.ascontiguousarray(a, dtype=np.float64) for a in (rows, b.clamp, free_kp)]
        self.work = np.zeros(2 * n * n + 8 * nd + 8 * ncv + nd * nd + ncv * ncv + 12 * (nd + ncv))
        self.s = Brace(n=n, nd=nd, ncv=ncv, nleg=len(legs), kd0=law.POSTURE_KD[0], kd1=law.POSTURE_KD[1],
                       standing_kp=se2.STANDING_KP, blas=c_motion.numpy_blas())
        for name, arr in zip(('act', 'acv', 'cv', 'leg_joints', 'leg_len'), self.kept[:5]):
            setattr(self.s, name, wbc.pointer(arr, np.int32))
        for name, arr in zip(('brace', 'clamp', 'free_kp'), self.kept[5:]):
            setattr(self.s, name, wbc.pointer(arr))
        self.s.work = wbc.pointer(self.work)
        self.scaled_c, self.blend_c = _typed()
        self.sp, self.n, self.nd = ctypes.byref(self.s), n, nd
        self.Ms, self.jeff, self.leg_kp = np.zeros((n, n)), np.zeros(nd), np.zeros(len(np.concatenate(legs)))
        self.mp, self.jp, self.lp = wbc.pointer(self.Ms), wbc.pointer(self.jeff), wbc.pointer(self.leg_kp)

    def scaled(self, M, w_up):
        """The solve's M: `M` with the four dofs' inertia scaled off by `w_up` (`machine.brace.scaled`)."""
        self.scaled_c(self.sp, wbc.pointer(M), wbc.pointer(w_up), self.mp)
        return self.Ms

    def blend(self, M, Ms, q, v, h, posture, qacc, tau, over, sw, bw, d_ax, d_lat, w_hop, w_up, stance):
        """(jeff, leg_kp) of `machine.brace.braced`; `qacc`, `tau` and `over` blended in place."""
        p = [wbc.pointer(a) for a in (M, Ms, q, v, h, posture, qacc, tau)]
        self.blend_c(self.sp, *p, wbc.pointer(over, np.bool_), sw, bw, d_ax, d_lat, w_hop, wbc.pointer(w_up),
                     wbc.pointer(stance), self.jp, self.lp)
        return self.jeff, self.leg_kp


def laid(b, rows, cv, acv, legs, free_kp):
    """The brace on her body `b` (`Braced`)."""
    return Braced(b, rows, cv, acv, legs, free_kp)
