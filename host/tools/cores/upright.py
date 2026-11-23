"""control's weighing between its parts in C (wbc_upright.c), `machine.law.upright` bit for bit.

    g['upright'] = upright.uprighted(g, g['borne'])   # laid by machine.law.motion.laid, `upright.weighs` its twin
    seat, hold, w_neck, w_base, w_hop_base, w_hop_base, pelvic = g['upright'](g, s, told, ask, sw, w_hop)
"""
import ctypes
import functools

import numpy as np

from tools.cores import wbc
from tools.cores.borne import Borne

_i, _d, _vp = ctypes.c_int, ctypes.c_double, ctypes.c_void_p


class Upright(ctypes.Structure):

    """wbc_upright_t: each flat index control weighs at, laid once, and its constants."""

    _fields_ = ([(name, _i) for name in ('st_to_pal', 'st_to_zmp', 'st_to_gain', 'st_now_gain', 'st_to_drive',
                                         'out_slid', 'out_held')]
                + [('out_ks', _i * 3), ('rest_k', _i * 3)]
                + [(name, _i) for name in ('a_strides', 'a_borne', 'told_isch', 'told_palmar', 'told_n', 'tab_cols',
                                           'tab_pal', 'pelvis', 'spare')]
                + [(name, _d) for name in ('touch_n', 'tiny', 'prop_on', 'stirs')])


@functools.lru_cache(maxsize=None)
def _typed():
    """The weighing's C typed, wbc_upright_t's size the C's."""
    lib = wbc.library()
    lib.wbh_upright_size.restype = _i
    if lib.wbh_upright_size() != ctypes.sizeof(Upright):
        raise ValueError('wbc_upright_t is %d bytes, its mirror %d' % (lib.wbh_upright_size(), ctypes.sizeof(Upright)))
    lib.wbh_upright.argtypes = ([ctypes.POINTER(Upright), ctypes.POINTER(Borne)] + [_vp] * 6 + [_i] + [_d] * 6
                                + [_vp])
    lib.wbh_upright.restype = None
    return lib.wbh_upright


def uprighted(g, read):
    """`machine.law.upright.weighs` in C (wbc_upright) on the robot's laid session `g` - its latent state, its
    latent's out, its rest and its ask, the same arrays a whole session - and borne's contacts `read` last read
    (`borne.laid`): its arithmetic between control's parts was 30 us of each law pass in Python's floats."""
    from machine.law import impedance, latent, modal, stride, upright as law
    from machine.moves import emancipation, leap
    from machine.senses.observer import TINY
    from machine.texts import ortho
    to, now, pal = latent.TO.start, latent.NOW.start, latent.MANIFOLD.start + ortho.PALMAR
    armed = [int(k) for k in law.ARMED]
    u = Upright(st_to_pal=to + pal, st_to_zmp=to + latent.ZMP, st_to_gain=to + latent.GAIN, st_now_gain=now + latent.GAIN,
                st_to_drive=to + latent.AMP.start + modal.DRIVE, out_slid=latent.SLID, out_held=latent.HELD,
                a_strides=modal.STRIDES, a_borne=modal.BORNE, told_isch=int(emancipation.SEAT_ROWS[0]),
                told_palmar=ortho.PALMAR, told_n=len(ortho.MANIFOLDS), tab_cols=latent.TABLE, tab_pal=pal,
                pelvis=g['body'].pelvis, touch_n=leap.TOUCH_N, tiny=TINY, prop_on=impedance.PROP_ON,
                stirs=stride.STIRS)
    u.out_ks[:] = [latent.KS.start + k for k in armed]
    u.rest_k[:] = [latent.K.start + k for k in armed]
    fn, t, got = _typed(), read.kept[0], np.zeros(7)
    st, out, a, rest = g['st'], g['out'], g['a'], g['rest']
    at, born, laid = ctypes.byref(u), ctypes.byref(t), tuple(wbc.address(x) for x in (st, out, a, rest, got))

    def upright(g_, s, told, ask, sw, w_hop):
        told_, tab = np.ascontiguousarray(told, dtype=np.float64), np.ascontiguousarray(g_['tab'], dtype=np.float64)
        loads = g_['loads']
        fn(at, born, laid[0], laid[1], laid[2], laid[3], wbc.pointer(told_), wbc.address(tab), len(tab),
           float(loads[0]), float(loads[1]), float(g_['fr'][modal.WEIGHT]), float(ask.get('reclined', 0.0)), sw, w_hop,
           laid[4])
        seat, hold, w_neck, w_base, w_hb, w_hb2, pelvic = got.tolist()
        return seat, hold, w_neck, w_base, w_hb, w_hb2, pelvic > 0.0
    upright.kept = (u, read, st, out, a, rest, got)
    return upright
