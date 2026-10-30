"""Her twist's step in C through ctypes (wbc_twist.c), her python's bit for bit.

    k, ask = motion.laid()
    step = twist.laid(k, y_in, y, fr, a, st, out, posture, state, words, waist, head)    # machine.law.se2.twisted
    step(dt, vel, held, prior)       # a pass: fr, st, out, posture, state and words in place
"""
import ctypes
import functools

import numpy as np

from tools.cores import wbc


class Twist(ctypes.Structure):

    """wbc_twist_t: the step's reading of her law's arrays, laid once."""

    _fields_ = ([(name, ctypes.c_int) for name in ('gyro_at', 'quat_at', 'y_cells', 'f_heading', 'st_now_gain',
                                                   'st_now_phase', 'st_to_amp', 'st_to_hz', 'st_now_hz', 'st_theta',
                                                   'waist', 'head', 's_last_way', 's_look_yaw', 's_turn_dc',
                                                   'w_heading', 'w_target', 'w_has', 'w_twist', 'w_actual', 'w_active',
                                                   'w_stop', 'w_t', 'w_back', 'w_backing', 'w_clear')]
                + [(name, ctypes.c_double) for name in ('walk_m_s', 'back_m_s', 'strafe_m_s', 'yaw_rate', 'floor_hz',
                                                        'touch_n', 'stirs')]
                + [(name, ctypes.c_void_p) for name in ('y_in', 'y', 'a', 'fr', 'st', 'out', 'posture', 'state',
                                                        'words')])


@functools.lru_cache(maxsize=None)
def _typed():
    """The step typed, wbc_twist_t's size the C's."""
    lib = wbc.library()
    lib.wbh_twist_size.restype = ctypes.c_int
    if lib.wbh_twist_size() != ctypes.sizeof(Twist):
        raise ValueError('wbc_twist_t is %d bytes, its mirror %d' % (lib.wbh_twist_size(), ctypes.sizeof(Twist)))
    lib.wbh_twist_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(Twist), ctypes.c_double] + [ctypes.c_void_p] * 3
    lib.wbh_twist_step.restype = None
    return lib.wbh_twist_step


def laid(k, y_in, y, fr, a, st, out, posture, state, words, waist, head):
    """The step in C on her law's arrays, pointed at once - `machine.law.se2.twisted` of her edge's
    `y_in`, modal's `y`, `fr` and `a`, her latent `st` and its `out`, her `posture` with her
    `waist`'s and her `head`'s places in it, the step's `state` and its `words` -: a step (dt, vel,
    held, prior); `k` the ask's (`tools.cores.motion.laid`)."""
    from machine.senses import edge
    from machine.law import latent, modal, se2, stride
    from machine.moves import leap
    t = Twist(gyro_at=edge.GYRO.start, quat_at=edge.QUAT.start, y_cells=modal.CELLS.start, f_heading=modal.HEADING,
              st_now_gain=latent.NOW.start + latent.GAIN, st_now_phase=latent.NOW.start + latent.PHASE.start,
              st_to_amp=latent.TO.start + latent.AMP.start, st_to_hz=latent.TO.start + latent.HZ,
              st_now_hz=latent.NOW.start + latent.HZ, st_theta=latent.THETA, waist=int(waist), head=int(head),
              s_last_way=se2.LAST_WAY, s_look_yaw=se2.LOOK_YAW, s_turn_dc=se2.TURN_DC,
              w_heading=se2.W_HEADING, w_target=se2.W_TARGET, w_has=se2.W_HAS, w_twist=se2.W_TWIST.start,
              w_actual=se2.W_ACTUAL.start, w_active=se2.W_ACTIVE, w_stop=se2.W_STOP, w_t=se2.W_T, w_back=se2.W_BACK,
              w_backing=se2.W_BACKING, w_clear=se2.W_CLEAR,
              walk_m_s=se2.WALK_SPEED, back_m_s=se2.BACK_SPEED, strafe_m_s=se2.STRAFE_SPEED, yaw_rate=se2.YAW_RATE,
              floor_hz=se2.FLOOR_HZ, touch_n=leap.TOUCH_N, stirs=stride.STIRS)
    t.kept = (y_in, y, a, fr, st, out, posture, state, words)
    for name, arr in zip(('y_in', 'y', 'a', 'fr', 'st', 'out', 'posture', 'state', 'words'), t.kept):
        setattr(t, name, wbc.pointer(arr))
    step, at, tp = _typed(), ctypes.addressof(k), ctypes.byref(t)

    def stepped(dt, vel, held, prior):
        step(at, tp, dt, wbc.pointer(vel), wbc.pointer(np.ascontiguousarray(held, dtype=np.float64)),
             wbc.pointer(np.ascontiguousarray(prior, dtype=np.float64)))
    return stepped
