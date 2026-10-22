"""Her surety field in C through ctypes (wbc_surety.c), her python's bit for bit.

    k, ask = motion.laid()
    field = surety.laid(k, y_in, y, fr, a, st, out)    # motion.surety_field of them, laid once
    cur, vec = field(prev, dt, ct, t)                  # the field dt s on from prev; vec its (5,)
"""
import ctypes
import functools

import numpy as np

from tools.cores import wbc


class Surety(ctypes.Structure):

    """wbc_surety_t: her surety field's reading of her law's arrays, laid once."""

    _fields_ = ([(name, ctypes.c_int) for name in ('quat_at', 'gyro_at', 'accel_at', 'load_at', 'k_to', 'k_from',
                                                   'gain_now', 'zmp_out', 'held_out')]
                + [(name, ctypes.c_void_p) for name in ('y_in', 'y', 'fr', 'a', 'st', 'out')])


@functools.lru_cache(maxsize=None)
def _typed():
    """The field's step typed, wbc_surety_t's size the C's."""
    lib = wbc.library()
    lib.wbh_surety_size.restype = ctypes.c_int
    if lib.wbh_surety_size() != ctypes.sizeof(Surety):
        raise ValueError('wbc_surety_t is %d bytes, its mirror %d' % (lib.wbh_surety_size(), ctypes.sizeof(Surety)))
    lib.wbh_imp_surety.argtypes = [ctypes.c_void_p, ctypes.POINTER(Surety)] + [ctypes.c_double] * 4 + [ctypes.c_void_p]
    lib.wbh_imp_surety.restype = ctypes.c_double
    return lib.wbh_imp_surety


def laid(k, y_in, y, fr, a, st, out):
    """Her surety field in C on her law's arrays, pointed at once - `machine.motion.surety_field` of
    her edge's `y_in`, modal's `y`, `fr` and `a`, her latent `st` and its `out` -: a step (prev, dt,
    ct, t) -> (cur, vec), `vec` its (5,) laid once; `k` the ask's (`tools.cores.motion.laid`)."""
    from machine import edge, latent, modal
    s = Surety(quat_at=edge.QUAT.start, gyro_at=edge.GYRO.start, accel_at=edge.ACCEL.start, load_at=edge.LOAD.start,
               k_to=latent.TO.start + latent.K.start + modal.AXIAL, k_from=latent.FROM.start + latent.K.start + modal.AXIAL,
               gain_now=latent.NOW.start + latent.GAIN, zmp_out=latent.ZMP,
               held_out=latent.HELD)
    s.kept = (y_in, y, fr, a, st, out)
    for name, arr in zip(('y_in', 'y', 'fr', 'a', 'st', 'out'), s.kept):
        setattr(s, name, wbc.pointer(arr))
    vec, step, at, sp = np.zeros(5), _typed(), ctypes.addressof(k), ctypes.byref(s)
    vp = vec.ctypes.data

    def field(prev, dt, ct, t):
        return step(at, sp, prev, dt, ct, t, vp), vec
    return field
