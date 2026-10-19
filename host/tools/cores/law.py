"""Her law in C through ctypes, her python's bit for bit.

Her ask (wbc_imp.c, wbc_stride.c), her latent step and her upper body's coupling (wbc_law.c) on
numpy's own OpenBLAS (wbc_blas.c); the structs they are laid on, filled of the modules that define
their layouts and constants.

    k, ask = law.laid()
    law.asked(k, ask, out, y, fr, a)        # `a` this pass, in place: impedance.ask(out, y, fr, a)
    c = law.latent_laid()
    law.stepped(c, st, tab, now, dt, out)   # latent.step(st, tab, now, dt, out)
    dq = law.coupled(c, q, qd, ref_q, ref_v, eso_z, out)     # support.couple(...)
    solve = law.wrench_laid(body)
    out = solve(body, s, ask)               # wrench.step(body, s, ask)
    law.exact_host()                        # bit for bit here, or only within the last bits
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
LAW_INTS = ('n p driven z k d amp phase gain hz zmp yaw now from to t0 blend theta vals said at lerp table '
            'ref rate acc ks ds psi sway turns held yawed').split()
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
    if ortho.N > 16 or ortho.P > 64 or support.DRIVEN > 32:
        raise ValueError('wbc_law.c lays 16 modes, 64 parameters and 32 drives at most')
    names = {'n': ortho.N, 'p': ortho.P, 'driven': support.DRIVEN, 'gain': latent.GAIN, 'hz': latent.HZ,
             'zmp': latent.ZMP, 'yaw': latent.YAW, 't0': latent.T0, 'blend': latent.BLEND, 'theta': latent.THETA,
             'at': latent.AT, 'lerp': latent.LERP, 'table': latent.TABLE, 'turns': latent.TURNS,
             'held': latent.HELD, 'yawed': latent.YAWED}
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


@functools.lru_cache(maxsize=None)
def _step_typed():
    """wbc_step_t's mirror laid by the C's own sizes, its bytes the C's, and the step typed on it."""
    lib = wbc.library()
    sizes = (ctypes.c_int * 7)()
    lib.wbh_step_sizes(sizes)
    n, d, c, u, r, work, size = list(sizes)
    i, f, p = ctypes.c_int, ctypes.c_double, ctypes.c_void_p
    fields = [(name, i) for name in 'n nd nu0 nupper nheld npass nact npairs'.split()]
    fields += [('act', i * d), ('legs', (i * 6) * 2), ('knees', i * 2), ('upper', i * d), ('held', i * n),
               ('passive', i * n), ('active', i * n), ('pairs', (i * 2) * d), ('rounds', i), ('past', i),
               ('mass', f), ('moment', f * u), ('room', f * 2)]
    fields += [(name, f) for name in ('yaw_m damp2 knee_min w_com w_turn w_swing w_knee w_posture w_height w_moment '
                                      'w_fold w_stop w_part w_air w_pin w_keep w_still grow_n ridge contact_k '
                                      'height_band held_k momentum_k mu g').split()]
    fields += [('posture_kd', f * 2), ('swing_kd', f * 2), ('turn_kd', f * 2), ('blas', Blas)]
    fields += [(name, p) for name in 'M h v q com k'.split()]
    fields += [(name, p * 2) for name in 'R quat J6 dJv sole tquat Jr dJr tw'.split()]
    fields += [('nc', i)] + [(name, p * c) for name in 'cJ cp cdJv cf'.split()]
    fields += [('stance', i * 2), ('nturns', i), ('kdot_given', i), ('swing', i * 2), ('fold', i * 2), ('wep', i),
               ('com_acc', f * 3), ('turn', (f * 4) * 2), ('kdot', f * 3), ('settle', (f * 6) * 2), ('edge', f * 2),
               ('strides', f), ('borne', f), ('mus', f * 2), ('swing_at', (f * 3) * 2), ('swing_speed', (f * 3) * 2),
               ('swing_acc', (f * 3) * 2), ('swing_quat', (f * 4) * 2), ('fold_knee', f * 2), ('fold_rate', f * 2),
               ('fold_acc', f * 2)]
    fields += [(name, p) for name in 'posture derate clamp wep_nm'.split()]
    fields += [('tau', f * d), ('qacc', f * n), ('bears', f * 2), ('cop', (f * 2) * 2), ('forces', (f * 3) * c),
               ('scale', f), ('load', f * d), ('cop_n', i), ('cop_none', i * 2), ('over', i * d), ('work', f * work)]
    Step = type('Step', (ctypes.Structure,), {'_fields_': fields})
    if ctypes.sizeof(Step) != size:
        raise ValueError('wbc_step_t is %d bytes, its mirror %d' % (size, ctypes.sizeof(Step)))
    lib.wbh_step.argtypes = [ctypes.POINTER(Step)]
    lib.wbh_step.restype = ctypes.c_int
    return Step, lib.wbh_step, (n, d, c)


def wrench_laid(b, sensed=None):
    """`machine.wrench.step` in C on her law's body `b`: a solve (b, s, ask) -> its dict, `b.load`
    set; her body's indices and the step's constants laid once, her mass, clamps and WEP read at
    each - her model's masses move as her wear finds a load. `s` her compiled sensing's last
    (`sensed`, `bodied.on_body`), the step pointed at its block in one write (POINTED)."""
    from machine import wbc as law, wrench
    Step, step, (most_n, most_d, most_c) = _step_typed()
    passive, active = np.flatnonzero(b.passive), np.flatnonzero(b.active_dof)
    if b.n > most_n or len(b.act) > most_d or wrench.NU > most_d:
        raise ValueError('wbc_step.c lays %d dofs and %d drives at most' % (most_n, most_d))
    k = Step(n=b.n, nd=len(b.act), nu0=wrench.NU, nupper=len(wrench.UPPER), nheld=len(b.held), npass=len(passive),
             nact=len(active), npairs=len(b.pairs), rounds=wrench.ROUNDS, past=wrench.PAST, yaw_m=wrench.YAW_M,
             damp2=wrench.DAMP ** 2, knee_min=wrench.KNEE_MIN, grow_n=wrench.GROW_N, ridge=wrench.RIDGE,
             contact_k=law.CONTACT_K, height_band=law.HEIGHT_BAND, held_k=law.HELD_K, momentum_k=law.MOMENTUM_K,
             mu=law.MU, g=law.G)
    k.act[:len(b.act)] = [int(x) for x in b.act]
    for side in range(2):
        k.legs[side][:] = wrench.LEGS[side]
        k.knees[side] = wrench.KNEES[side]
    k.upper[:len(wrench.UPPER)] = wrench.UPPER
    k.held[:len(b.held)] = [int(x) for x in b.held]
    k.passive[:len(passive)] = passive.tolist()
    k.active[:len(active)] = active.tolist()
    for at, (p, r) in enumerate(b.pairs):
        k.pairs[at][0], k.pairs[at][1] = p, r
    k.moment[:wrench.NU] = wrench.MOMENT.tolist()
    k.room[:] = wrench.ROOM.tolist()
    for name in 'COM TURN SWING KNEE POSTURE HEIGHT MOMENT FOLD STOP PART AIR PIN KEEP STILL'.split():
        setattr(k, 'w_' + name.lower(), getattr(wrench, 'W_' + name))
    k.posture_kd[:], k.swing_kd[:], k.turn_kd[:] = law.POSTURE_KD, law.SWING_KD, law.TURN_KD
    k.blas = numpy_blas()
    views = {name: wbc.plain(np.ctypeslib.as_array(getattr(k, name))) for name in VIEWED}
    first = getattr(Step, POINTED[0][0]).offset
    k.sensed, k.placed, views['words'] = sensed, None, np.frombuffer(k, dtype=np.uint64)
    k.pointed = slice(first // 8, first // 8 + len(POINTED))
    if [getattr(Step, f).offset + 8 * i for f, i, _p in POINTED] != list(range(first, first + 8 * len(POINTED), 8)):
        k.sensed = None
    return functools.partial(_stepped, k, step, views)


#: The step's pointers at what her sensing returns, in the struct's order: (field, index, path).
POINTED = ([(name, 0, (name,)) for name in ('M', 'h', 'v', 'q', 'com', 'k')]
           + [(name, side, ('feet', side, name)) for name in ('R', 'quat', 'J6', 'dJv', 'sole') for side in (0, 1)]
           + [(field, t, ('turns', t, name)) for field, name in (('tquat', 'quat'), ('Jr', 'Jr'), ('dJr', 'dJr'),
                                                                ('tw', 'w')) for t in (0, 1)])


#: The step's fields read and written through numpy views of the struct's own memory.
VIEWED = ('stance', 'com_acc', 'turn', 'kdot', 'settle', 'edge', 'mus', 'swing', 'fold', 'swing_at', 'swing_speed',
          'swing_acc', 'swing_quat', 'fold_knee', 'fold_rate', 'fold_acc', 'tau', 'qacc', 'bears', 'cop', 'forces',
          'load', 'cop_none', 'over')


def _stepped(k, step, v, b, s, ask):
    """wrench.step(b, s, ask) on the laid `k`: what `s` senses pointed at, what `ask` asks copied
    through `v`, its fields' views."""
    from machine import wbc as law
    kept = []

    def at(a):
        a = np.ascontiguousarray(a, dtype=np.float64)
        kept.append(a)
        return a.ctypes.data
    last = getattr(k.sensed, 'last', None)
    if last is not None and s is last[0]:
        c = k.sensed.c
        if k.placed is None or k.placed[0] is not c:
            k.placed = (c, np.array([8 * c.place(path) for _f, _i, path in POINTED], dtype=np.uint64))
        v['words'][k.pointed] = last[1] + k.placed[1]
    else:
        k.M, k.h, k.v, k.q, k.com, k.k = (at(s[name]) for name in ('M', 'h', 'v', 'q', 'com', 'k'))
        for side, f in enumerate(s['feet']):
            k.R[side], k.quat[side], k.J6[side], k.dJv[side], k.sole[side] = (at(f[name]) for name in
                                                                               ('R', 'quat', 'J6', 'dJv', 'sole'))
        for t, turn in enumerate(s['turns']):
            k.tquat[t], k.Jr[t], k.dJr[t], k.tw[t] = (at(turn[name]) for name in ('quat', 'Jr', 'dJr', 'w'))
    contacts = s.get('contacts', ())
    if len(contacts) > len(k.cJ):
        raise ValueError('wbc_step.c lays %d contacts at most, %d bear her' % (len(k.cJ), len(contacts)))
    k.nc = len(contacts)
    for at_, c in enumerate(contacts):
        k.cJ[at_], k.cp[at_], k.cdJv[at_], k.cf[at_] = (at(c[name]) for name in ('J', 'p', 'dJv', 'f'))
    v['stance'][:] = [bool(x) for x in ask['stance']]
    v['com_acc'][:] = ask['com_acc']
    turns = ask.get('turns', ())
    k.nturns = len(turns)
    for t, quat in enumerate(turns):
        v['turn'][t] = quat
    kdot = ask.get('kdot')
    k.kdot_given = int(kdot is not None)
    if kdot is not None:
        v['kdot'][:] = kdot
    v['settle'][:] = ask.get('settle', 0.0)
    v['edge'][:] = ask.get('edge', (0.0, 0.0))
    v['mus'][:] = ask.get('mu', (law.MU, law.MU))
    k.strides, k.borne = float(ask.get('strides', 0.0)), float(ask.get('borne', 0.0))
    swing, fold = ask.get('swing', {}), ask.get('fold', {})
    for side in range(2):
        v['swing'][side] = side in swing
        if side in swing:
            v['swing_at'][side], v['swing_speed'][side], v['swing_acc'][side], v['swing_quat'][side] = swing[side]
        v['fold'][side] = side in fold
        if side in fold:
            v['fold_knee'][side], v['fold_rate'][side], v['fold_acc'][side] = fold[side]
    k.wep = int(bool(ask.get('wep')))
    k.posture = at(ask['posture'])
    derate = np.asarray(ask.get('derate', 1.0), float)
    k.derate = at(derate if derate.shape == (k.nd,) else np.broadcast_to(derate, (k.nd,)))
    k.clamp, k.wep_nm, k.mass = at(b.clamp), at(b.wep), b.mass
    if step(ctypes.byref(k)):
        raise np.linalg.LinAlgError('Singular matrix')
    nd, cop = k.nd, [None if v['cop_none'][side] else v['cop'][side].copy() for side in range(k.cop_n)]
    b.load = v['load'][:nd].copy()
    return {'tau': v['tau'][:nd].copy(), 'qacc': v['qacc'][:k.n].copy(), 'bears': v['bears'].tolist(), 'cop': cop,
            'over': v['over'][:nd].astype(bool), 'slack': [], 'held': (),
            'borne': v['forces'][:k.nc].reshape(-1, 3).copy(), 'scale': k.scale}
