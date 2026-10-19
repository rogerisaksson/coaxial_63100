"""`machine.impedance.ask` in C (wbc_imp.c, wbc_stride.c) through ctypes, bit for bit.

The struct it is laid on, filled of the modules that define its layouts and constants.

    k, ask = imp.laid()
    imp.asked(k, ask, out, y, fr, a)        # `a` this pass, in place: impedance.ask(out, y, fr, a)
"""
import ctypes
import functools

import numpy as np

from tools.cores import wbc

#: wbc_imp_t's ints in its order, by the names of what fills them: her modes (`machine.modal`),
#: the offsets of latent.step's out, of modal's y, fr and a.
MODES = 'axial lateral drive pitch sagittal lift width list turn arms rock'.split()
OUT = 'ref rate acc ks ds psi sway turns held yawed'.split()
Y = ('com vcom sole vsole load quat spin moment lowest hip tilt tilt_rate arm prop prop_lo prop_hi grip vhead '
     'hspin').split()
FR = 'rot w0 floor weight knee pitch0 dt reach0 hips_by arm0 unit z0 mass ground floor_at'.split()
A = ('stance com_acc sole_acc want fold fold_rate turned bend kdot going rolled headed tipped swung folds settle '
     'edge heeled strides prefold swings raise need lead outside borne').split()


def _at(x):
    return x.start if isinstance(x, slice) else int(x)


def _doubles():
    """wbc_imp_t's doubles in its order: (name, value) - a tuple or an array lays that many."""
    from machine import figure, flight, impedance as imp, modal, poise, stride, wbc as law
    return (('on', imp.ON), ('going_s', imp.GOING_S), ('ahead', imp.AHEAD), ('yaw_k', imp.YAW_K),
            ('throw_m_s2', imp.THROW_M_S2), ('prop_on', imp.PROP_ON), ('low_m', imp.LOW_M), ('low_leg', imp.LOW_LEG),
            ('falls', imp.FALLS), ('upward', imp.UPWARD), ('between', imp.BETWEEN),
            ('g', modal.G), ('axial_m', modal.AXIAL_M), ('pitch_rad', modal.PITCH_RAD), ('clear_m', modal.CLEAR_M),
            ('list_rad', modal.LIST_RAD), ('turn_rad', modal.TURN_RAD), ('arm_rad', modal.ARM_RAD),
            ('leg_m', modal.LEG_M), ('pelvis', modal.PELVIS), ('momentum_k', law.MOMENTUM_K),
            ('posture_kd', law.POSTURE_KD), ('turn_kd', law.TURN_KD), ('contact_k', law.CONTACT_K),
            ('big', poise.BIG), ('tiny', poise.TINY), ('half_m', poise.HALF_M), ('margin_m', poise.MARGIN_M),
            ('capture_s', poise.CAPTURE_S), ('extent', poise.EXTENT), ('lift_eps', poise.LIFT_EPS),
            ('gate_m', poise.GATE_M), ('early', poise.EARLY), ('along_on', poise.ON),
            ('touch_n', flight.TOUCH_N), ('touch_m', flight.TOUCH_M), ('rest_m_s', flight.REST_M_S),
            ('middle', flight.MIDDLE), ('ankle_h', figure.ANKLE_H), ('ball', figure.BALL), ('toe_m', figure.TOE_M),
            ('limbs', stride.LIMBS), ('stride_m', stride.STRIDE_M), ('rock_rad', stride.ROCK_RAD),
            ('stirs', stride.STIRS), ('phi_w', stride.PHI_W), ('back', stride.BACK), ('ahead_m_s', stride.AHEAD_M_S),
            ('reach_m', stride.REACH_M), ('apart_m', stride.APART_M), ('soon_s', stride.SOON_S),
            ('rise_u', stride.RISE_U), ('fall_u', stride.FALL_U), ('rise_u2', stride.RISE_U ** 2),
            ('fall_u2', stride.FALL_U ** 2), ('arrive_u', stride.ARRIVE_U), ('leads', stride.LEADS),
            ('reach_m_s', stride.REACH_M_S), ('reach_s', stride.REACH_S), ('flat_k', stride.FLAT_K),
            ('heel_rad', stride.HEEL_RAD), ('heel_rad_s', stride.HEEL_RAD_S), ('dip_m', stride.DIP_M),
            ('edge_rad', stride.EDGE_RAD), ('fold_rad', stride.FOLD_RAD), ('heel_up', stride.HEEL_UP))


@functools.lru_cache(maxsize=None)
def _typed(fields):
    """wbc_imp_t's mirror of its `fields`, its size the C's, and the step typed on it."""
    Imp = type('Imp', (ctypes.Structure,), {'_fields_': list(fields)})
    lib = wbc.library()
    lib.wbh_imp_size.restype = ctypes.c_int
    lib.wbh_imp_ask.argtypes = [ctypes.POINTER(Imp)] + [ctypes.c_void_p] * 4
    lib.wbh_imp_ask.restype = None
    if lib.wbh_imp_size() != ctypes.sizeof(Imp):
        raise ValueError('wbc_imp_t is %d bytes, its mirror %d' % (lib.wbh_imp_size(), ctypes.sizeof(Imp)))
    return Imp, lib.wbh_imp_ask


def laid():
    """(k, ask): wbc_imp_t filled of the constants as they are now - `tools.sim.ballistic` sets
    them before her session is laid -, and the step on it."""
    from machine import impedance, latent, modal, stride
    if (stride.ALONG, stride.LEFT, stride.UP) != (0, 1, 2) or modal.HANDS.shape != (modal.N, 2, 3):
        raise ValueError('wbc_imp.c reads her way along, left, up and her hands (n, 2, 3)')
    if modal.N > 16:
        raise ValueError('wbc_imp.c lays %d modes at most, she has %d' % (16, modal.N))
    ints = ([('n', modal.N)] + [(m, getattr(modal, m.upper())) for m in MODES]
            + [('o_' + o, _at(getattr(latent, o.upper()))) for o in OUT]
            + [('y_' + v, _at(getattr(modal, v.upper()))) for v in Y]
            + [('f_' + f, _at(getattr(modal, f.upper()))) for f in FR]
            + [('a_' + x, _at(getattr(modal, x.upper()))) for x in A])
    doubles = [(name, np.ndim(v) > 0, np.atleast_1d(np.asarray(v, dtype=float))) for name, v in _doubles()]
    Imp, ask = _typed(tuple([(name, ctypes.c_int) for name, _v in ints]
                            + [('free_by', ctypes.c_int * 3), ('hands', ctypes.c_void_p)]
                            + [(name, ctypes.c_double * len(v) if many else ctypes.c_double)
                               for name, many, v in doubles]))
    k = Imp(**{name: int(v) for name, v in ints})
    k.free_by[:] = [int(i) for i in impedance.FREE_BY]
    k.kept = np.array(modal.HANDS, dtype=np.float64)
    k.hands = wbc.pointer(k.kept)
    for name, many, v in doubles:
        if many:
            getattr(k, name)[:] = v.tolist()
        else:
            setattr(k, name, float(v[0]))
    return k, ask


def asked(k, ask, out, y, fr, a):
    """`a`, the loop's ask this pass, in place - `impedance.ask(out, y, fr, a)` in C."""
    ask(ctypes.byref(k), wbc.pointer(out), wbc.pointer(y), wbc.pointer(fr), wbc.pointer(a))
    return a
