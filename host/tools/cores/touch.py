"""Her touch's filter in C through ctypes (wbc_touch.c), her python's bit for bit.

    k, ask = motion.laid()
    step = touch.laid(k, y_in, xmat, xpos, ids, tips, y, fr, a, st, out, rest, state)    # machine.senses.touch.stepped
    twist = step(asked, twist, tw_on, heading, dt, vel)       # a pass: out, a and state in place, her twist back
"""
import ctypes
import functools

import numpy as np

from tools.cores import wbc


class Touch(ctypes.Structure):

    """wbc_touch_t: the filter's reading of her law's arrays, laid once."""

    _fields_ = ([(name, ctypes.c_int) for name in ('touch_at', 'st_now_gain', 'st_to_gain', 'r_z', 'r_k', 'r_d',
                                                   's_felt', 's_hold', 's_brake', 's_stand', 's_drv', 's_clear')]
                + [('core', ctypes.c_int * 4), ('soles', (ctypes.c_int * 2) * 2)]
                + [(name, ctypes.c_void_p) for name in ('ids', 'tips')]
                + [(name, ctypes.c_double) for name in ('resume_s', 'along_cos', 'felt_s', 'edge_m', 'touched_n')]
                + [(name, ctypes.c_void_p) for name in ('y_in', 'xmat', 'xpos', 'y', 'fr', 'st', 'rest', 'a', 'out',
                                                        'state')])


@functools.lru_cache(maxsize=None)
def _typed():
    """The step typed, wbc_touch_t's size the C's."""
    lib = wbc.library()
    lib.wbh_touch_size.restype = ctypes.c_int
    if lib.wbh_touch_size() != ctypes.sizeof(Touch):
        raise ValueError('wbc_touch_t is %d bytes, its mirror %d' % (lib.wbh_touch_size(), ctypes.sizeof(Touch)))
    lib.wbh_touch_step.argtypes = ([ctypes.c_void_p, ctypes.POINTER(Touch), ctypes.c_void_p, ctypes.c_void_p]
                                   + [ctypes.c_double] * 3 + [ctypes.c_void_p])
    lib.wbh_touch_step.restype = None
    return lib.wbh_touch_step


def laid(k, y_in, xmat, xpos, ids, tips, y, fr, a, st, out, rest, state):
    """The filter in C on her law's arrays, pointed at once - `machine.senses.touch.stepped` of her edge's
    `y_in`, her model's `xmat` and `xpos` with her skin's bodies `ids` and the toes' `tips`, modal's
    `y`, `fr` and `a`, her latent `st`, its `out` and her `rest`, the filter's `state` -: a step
    (asked, twist, tw_on, heading, dt, vel) -> her twist (3,); `k` the ask's (`tools.cores.motion.laid`)."""
    from machine.senses import edge
    from machine.law import latent
    from machine.senses import touch
    t = Touch(touch_at=edge.TOUCH.start, st_now_gain=latent.NOW.start + latent.GAIN, st_to_gain=latent.TO.start + latent.GAIN,
              r_z=latent.Z.start, r_k=latent.K.start, r_d=latent.D.start, s_felt=touch.FELT.start, s_hold=touch.HOLD_S,
              s_brake=touch.U_BRAKE, s_stand=touch.U_STAND, s_drv=touch.U_DRV, s_clear=touch.CLEAR.start,
              resume_s=touch.RESUME_S, along_cos=touch.ALONG_COS, felt_s=touch.FELT_S, edge_m=touch.EDGE_M,
              touched_n=edge.TOUCHED_N)
    t.core[:] = list(touch.CORE)
    for s in range(2):
        t.soles[s][:] = list(touch.SOLES[s])
    t.kept = (np.ascontiguousarray(ids, dtype=np.int32), np.ascontiguousarray(tips, dtype=np.float64),
              y_in, xmat, xpos, y, fr, a, st, out, rest, state)
    t.ids, t.tips = wbc.pointer(t.kept[0], np.int32), wbc.pointer(t.kept[1])
    for name, arr in zip(('y_in', 'xmat', 'xpos', 'y', 'fr', 'a', 'st', 'out', 'rest', 'state'), t.kept[2:]):
        setattr(t, name, wbc.pointer(arr))
    asked, twist = np.zeros(3), np.zeros(3)
    step, at, tp, ap, wp = _typed(), ctypes.addressof(k), ctypes.byref(t), asked.ctypes.data, twist.ctypes.data

    def stepped(asked_now, twist_now, tw_on, heading, dt, vel):
        asked[:], twist[:] = asked_now, twist_now
        step(at, tp, ap, wp, tw_on, heading, dt, wbc.pointer(vel))
        return tuple(twist.tolist())
    return stepped
