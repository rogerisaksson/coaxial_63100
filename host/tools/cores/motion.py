"""Her motion pass in C through ctypes, her python's bit for bit.

Her ask (wbc_imp.c, wbc_stride.c), her latent step and her upper body's coupling (wbc_law.c) on
numpy's own OpenBLAS (wbc_blas.c); the structs they are laid on, filled of the modules that define
their layouts and constants. Her surety field: `tools.cores.surety`; her step: `tools.cores.step`.

    k, ask = motion.laid()
    motion.asked(k, ask, out, y, fr, a)     # `a` this pass, in place: impedance.ask(out, y, fr, a)
    c = motion.latent_laid()
    motion.stepped(c, st, tab, now, dt, out)    # latent.step(st, tab, now, dt, out)
    dq = motion.coupled(c, q, qd, ref_q, ref_v, eso_z, out)  # support.couple(...)
    motion.exact_host()                     # bit for bit here, or only within the last bits
"""
import ctypes
import functools
import glob
import math
import os

import numpy as np

from tools.cores import wbc

#: wbc_imp_t's ints in its order, by the names of what fills them: her modes (`machine.modal`),
#: the offsets of latent.step's out, of modal's y, fr and a.
MODES = 'axial lateral drive pitch sagittal lift width list turn arms rock reach elbows'.split()
OUT = 'ref rate acc ks ds psi sway turns held yawed bend ducked manifolded'.split()
Y = ('com vcom sole vsole load quat spin moment lowest hip tilt tilt_rate arm prop prop_lo prop_hi grip vhead '
     'hspin elbow palms').split()
FR = ('rot w0 floor weight knee pitch0 dt reach0 hips_by arm0 unit z0 mass ground floor_at'
      ' map_at map_way map_along map_rise map_n elbow0').split()
A = ('stance com_acc sole_acc want fold fold_rate turned bend kdot going rolled headed tipped swung folds settle '
     'edge heeled strides prefold swings raise need lead outside borne bend_rate cell ducking rests propped '
     'propped_below propped_since propped_box reached elbowed let_go').split()


def _at(x):
    """An offset's start: a float is another constant of the same name (modal's SUPPORT, a ZMP's
    mode, once took its slot as 1 - her right sole's stance)."""
    if not isinstance(x, (slice, int, np.integer)):
        raise TypeError('an offset, not %r' % (x,))
    return x.start if isinstance(x, slice) else int(x)


def _doubles():
    """wbc_imp_t's doubles in its order: (name, value) - a tuple or an array lays that many."""
    from machine import borne, edge, figure, impedance as imp, leap, modal, poise, stride, wbc as law
    return (('on', imp.ON), ('going_s', imp.GOING_S), ('ahead', imp.AHEAD), ('yaw_k', imp.YAW_K),
            ('throw_m_s2', imp.THROW_M_S2), ('prop_on', imp.PROP_ON), ('prop_engage', borne.PROP_ENGAGE),
            ('prop_release', borne.PROP_RELEASE), ('prop_dwell_s', borne.PROP_DWELL_S), ('prop_blank_s', borne.PROP_BLANK_S),
            ('low_m', imp.LOW_M), ('low_leg', imp.LOW_LEG),
            ('falls', imp.FALLS), ('upward', imp.UPWARD), ('between', imp.BETWEEN),
            ('g', modal.G), ('axial_m', modal.AXIAL_M), ('pitch_rad', modal.PITCH_RAD), ('clear_m', modal.CLEAR_M),
            ('list_rad', modal.LIST_RAD), ('turn_rad', modal.TURN_RAD), ('arm_rad', modal.ARM_RAD),
            ('leg_m', modal.LEG_M), ('pelvis', modal.PELVIS), ('momentum_k', law.MOMENTUM_K),
            ('posture_kd', law.POSTURE_KD), ('turn_kd', law.TURN_KD), ('contact_k', law.CONTACT_K),
            ('big', poise.BIG), ('tiny', poise.TINY), ('half_m', poise.HALF_M), ('margin_m', poise.MARGIN_M),
            ('capture_s', poise.CAPTURE_S), ('extent', poise.EXTENT), ('lift_eps', poise.LIFT_EPS),
            ('gate_m', poise.GATE_M), ('early', poise.EARLY), ('along_on', poise.ON),
            ('touch_n', leap.TOUCH_N), ('touch_m', leap.TOUCH_M), ('rest_m_s', leap.REST_M_S),
            ('middle', leap.MIDDLE), ('ankle_h', figure.ANKLE_H), ('ball', figure.BALL), ('toe_m', figure.TOE_M),
            ('limbs', stride.LIMBS), ('stride_m', stride.STRIDE_M), ('rock_rad', stride.ROCK_RAD),
            ('stirs', stride.STIRS), ('phi_w', stride.PHI_W), ('back', stride.BACK), ('ahead_m_s', stride.AHEAD_M_S),
            ('reach_m', stride.REACH_M), ('apart_m', stride.APART_M), ('soon_s', stride.SOON_S),
            ('rise_u', stride.RISE_U), ('fall_u', stride.FALL_U), ('rise_u2', stride.RISE_U ** 2),
            ('fall_u2', stride.FALL_U ** 2), ('arrive_u', stride.ARRIVE_U), ('leads', stride.LEADS),
            ('reach_m_s', stride.REACH_M_S), ('reach_s', stride.REACH_S), ('flat_k', stride.FLAT_K),
            ('heel_rad', stride.HEEL_RAD), ('heel_rad_s', stride.HEEL_RAD_S), ('dip_m', stride.DIP_M),
            ('edge_rad', stride.EDGE_RAD), ('fold_rad', stride.FOLD_RAD), ('heel_up', stride.HEEL_UP),
            ('map_k', float(modal.MAP_K)))


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
@functools.lru_cache(maxsize=None)
def exact_host(draws=1000):
    """Whether this host's numpy computes her law's primitives as her law's C does, bit for bit -
    its small matmuls', dots' and einsum's orders (wbh_np_probe); its sin, cos, tan, arcsin,
    arccos, arctan2, exp, log and hypot on arrays and on scalars and a scalar's square its pow,
    libm called side by side as her C calls it, 10^5 draws across its paths (wbh_np_libm): where
    it does, her law in C is her python's bit for bit; a numpy dispatching elsewhere (SVML, other
    OpenBLAS kernels) parts the last bits."""
    lib = wbc.library()
    lib.wbh_np_probe.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
    lib.wbh_np_probe.restype = None
    lib.wbh_np_libm.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p]
    lib.wbh_np_libm.restype = None
    rng = np.random.default_rng(3)
    a = rng.normal(size=(draws, 9)) * rng.choice([1e-3, 1.0, 1e3], size=(draws, 1))
    b = rng.normal(size=(draws, 9))

    def probe(which):
        out = np.zeros((draws, 9))
        lib.wbh_np_probe(which, a.ctypes.data, b.ctypes.data, out.ctypes.data, draws)
        return out
    wants = {0: (a.reshape(draws, 3, 3) @ b[:, :3, None])[:, :, 0],
             1: (a[:, :4].reshape(draws, 2, 2) @ b[:, :2, None])[:, :, 0],
             2: (a[:, :6].reshape(draws, 2, 3) @ np.transpose(b.reshape(draws, 3, 3), (0, 2, 1))).reshape(draws, 6),
             3: (a[:, :6].reshape(draws, 2, 3) @ b.reshape(draws, 3, 3)).reshape(draws, 6),
             4: np.array([x[:3] @ y[:3] for x, y in zip(a, b)])[:, None],
             5: np.array([np.einsum('nia,ia->n', x[None, :6].reshape(1, 2, 3), y[:6].reshape(2, 3))[0]
                          for x, y in zip(a, b)])[:, None]}
    for which, want in wants.items():
        if not np.array_equal(probe(which)[:, :want.shape[1]], want):
            return False
    x = np.concatenate([a[:, 0] * 0.3, rng.uniform(-8.0, 8.0, 100 * draws)])
    ins = (x, rng.normal(size=len(x)), np.clip(x, -1.0, 1.0), np.clip(x, -50.0, 50.0), np.abs(x) + 1e-3)
    got, few, laid = np.zeros((len(x), 10)), 2 * draws, np.ascontiguousarray(np.stack(ins, axis=1))
    lib.wbh_np_libm(len(x), laid.ctypes.data, got.ctypes.data)
    libm: list = [(np.sin, (0,)), (np.cos, (0,)), (np.tan, (0,)), (np.arcsin, (2,)), (np.arccos, (2,)),
                  (np.arctan2, (0, 1)), (np.exp, (3,)), (np.log, (4,)), (np.hypot, (0, 1))]
    for j, (fn, at) in enumerate(libm):
        scalars = np.array([fn(*[np.float64(ins[i][k]) for i in at]) for k in range(few)])
        if not (np.array_equal(got[:, j], fn(*[ins[i] for i in at])) and np.array_equal(got[:few, j], scalars)):
            return False
    return np.array_equal(got[:few, 9], np.array([np.float64(v) ** 2 for v in x[:few]]))


class Blas(ctypes.Structure):

    """wbc_blas_t: numpy's own OpenBLAS - cblas_dgemm, cblas_dsyrk, cblas_dgemv, cblas_ddot, dgesv."""

    _fields_ = [(name, ctypes.c_void_p) for name in ('gemm', 'syrk', 'gemv', 'dot', 'gesv')]


@functools.lru_cache(maxsize=None)
def numpy_blas():
    """numpy's own OpenBLAS (scipy-openblas, ILP64) as wbc_blas_t: the library numpy's matmul and
    linalg call, so C calling it gets numpy's bits."""
    found = sorted(glob.glob(os.path.join(os.path.dirname(np.__file__), os.pardir, 'numpy.libs', '*scipy_openblas64*')))
    if not found:
        raise RuntimeError("numpy carries no scipy-openblas64 beside it: her law in C needs numpy's own BLAS")
    lib = ctypes.CDLL(os.path.abspath(found[0]))
    names = ('scipy_cblas_dgemm64_', 'scipy_cblas_dsyrk64_', 'scipy_cblas_dgemv64_', 'scipy_cblas_ddot64_',
             'scipy_dgesv_64_')
    return Blas(*[ctypes.cast(getattr(lib, n), ctypes.c_void_p).value for n in names])


#: wbc_law_t's ints and doubles in its order, by what fills them.
LAW_INTS = ('n p driven z k d amp phase gain hz zmp yaw now from to t0 blend theta v0 v1 contig shape vals said at '
            'lerp curve shaped table ref rate acc ks ds psi sway turns held yawed bend duck ducked manifold '
            'manifolded slide slid').split()
LAW_DOUBLES = 'edge_s tau tau_v eso_0 eso_s sigma bleed'.split()


class Law(ctypes.Structure):

    """wbc_law_t: latent.step's and support.couple's layouts, their constants, numpy's BLAS."""

    _fields_ = ([(name, ctypes.c_int) for name in LAW_INTS] + [(name, ctypes.c_double) for name in LAW_DOUBLES]
                + [('pi', ctypes.c_void_p), ('gamma', ctypes.c_void_p), ('blas', Blas)])


@functools.lru_cache(maxsize=None)
def _law_typed():
    """The core's latent step and coupling typed, wbc_law_t's size the C's."""
    lib = wbc.library()
    lib.wbh_law_size.restype = ctypes.c_int
    if lib.wbh_law_size() != ctypes.sizeof(Law):
        raise ValueError('wbc_law_t is %d bytes, its mirror %d' % (lib.wbh_law_size(), ctypes.sizeof(Law)))
    lib.wbh_law_latent.argtypes = [ctypes.POINTER(Law), ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int,
                                   ctypes.c_double, ctypes.c_double, ctypes.c_void_p, ctypes.c_void_p]
    lib.wbh_law_couple.argtypes = [ctypes.POINTER(Law)] + [ctypes.c_void_p] * 7
    lib.wbh_law_latent.restype = lib.wbh_law_couple.restype = None
    return lib.wbh_law_latent, lib.wbh_law_couple


def latent_laid():
    """wbc_law_t filled of machine.latent, machine.ortho and machine.support as they are now, and
    its steps: (struct, latent, couple)."""
    from machine import latent, ortho, support
    if ortho.N > 16 or ortho.P > 93 or len(ortho.MANIFOLDS) != 5 or support.DRIVEN > 32:
        raise ValueError('wbc_law.c lays 16 modes, 93 parameters, 5 manifolds and 32 drives at most')
    names = {'n': ortho.N, 'p': ortho.P, 'driven': support.DRIVEN, 'gain': latent.GAIN, 'hz': latent.HZ,
             'zmp': latent.ZMP, 'yaw': latent.YAW, 't0': latent.T0, 'blend': latent.BLEND, 'theta': latent.THETA,
             'contig': latent.CONTIG,
             'at': latent.AT, 'lerp': latent.LERP, 'curve': latent.CURVE, 'table': latent.TABLE, 'turns': latent.TURNS,
             'held': latent.HELD, 'yawed': latent.YAWED, 'slide': latent.SLIDE, 'slid': latent.SLID}
    k = Law(**{name: int(names[name]) if name in names else _at(getattr(latent, name.upper())) for name in LAW_INTS},
            edge_s=latent.EDGE_S, tau=math.tau, tau_v=support.TAU_V, eso_0=support.ESO_0, eso_s=support.ESO_S,
            sigma=support.SIGMA, bleed=support.BLEED)
    k.kept = (np.array(support.PI, dtype=np.float64), np.array(support.GAMMA, dtype=np.float64))
    k.pi, k.gamma = wbc.pointer(k.kept[0]), wbc.pointer(k.kept[1])
    k.blas = numpy_blas()
    k.hit = np.zeros(64)
    return (k,) + _law_typed()


def stepped(laid_, st, tab, now, dt, out):
    """`out` this pass, `st` blended - latent.step(st, tab, now, dt, out) in C."""
    k, step, _couple = laid_
    if len(k.hit) < len(tab):
        k.hit = np.zeros(2 * len(tab))
    step(ctypes.byref(k), wbc.pointer(st), wbc.pointer(tab), len(tab), float(now), float(dt), wbc.pointer(out),
         k.hit.ctypes.data)
    return out


def coupled(laid_, q, qd, ref_q, ref_v, eso_z, out):
    """`dq`, out's REF shifted and `eso_z` bled - support.couple(q, qd, ref_q, ref_v, eso_z, out) in C."""
    k, _step, couple = laid_
    dq = np.empty(len(q))
    couple(ctypes.byref(k), wbc.pointer(q), wbc.pointer(qd), wbc.pointer(ref_q), wbc.pointer(ref_v),
           wbc.pointer(eso_z), wbc.pointer(out), dq.ctypes.data)
    return dq
