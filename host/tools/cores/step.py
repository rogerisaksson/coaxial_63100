"""Her whole-body steps in C through ctypes (wbc_step.c, wbc_chain.c), their python twins' bit for bit.

`machine.stack.wrench.step`'s and `machine.stack.chain.step`'s:

    solve = step.wrench_laid(body)
    out = solve(body, s, ask)               # wrench.step(body, s, ask)
    chained = step.chain_laid(body, None, regime)
    out = chained(body, s, ask)             # chain.step(body, s, ask, regime)
"""
import ctypes
import functools
import operator

import numpy as np

from machine.senses import borne
from tools.cores import wbc
from tools.cores.motion import Blas, numpy_blas


@functools.lru_cache(maxsize=None)
def _step_typed():
    """wbc_step_t's mirror laid by the C's own sizes, its bytes the C's, and the step typed on it."""
    lib = wbc.library()
    sizes = (ctypes.c_int * 8)()
    lib.wbh_step_sizes(sizes)
    n, d, c, u, r, work, size, QP_ROWS = list(sizes)
    i, f, p = ctypes.c_int, ctypes.c_double, ctypes.c_void_p
    fields = [(name, i) for name in 'n nd nu0 nupper nheld npass nact npairs nbuffers'.split()]
    fields += [('act', i * d), ('legs', (i * 6) * 2), ('knees', i * 2), ('upper', i * d), ('held', i * n),
               ('passive', i * n), ('active', i * n), ('pairs', (i * 2) * d), ('rounds', i), ('past', i),
               ('buffer_at', i * (2 * d)), ('sides', i * d), ('carpal', i), ('buffer_stop', f * (2 * d)), ('buffer_side', f * (2 * d)), ('buffer', f * 4),
               ('mass', f), ('moment', f * u), ('room', f * 2), ('arms', f * d)]
    fields += [(name, f) for name in ('yaw_m damp2 w_com w_turn w_swing w_knee w_posture w_height w_moment '
                                      'w_fold w_stop w_part w_air w_pin w_keep w_still grow_n ridge contact_k '
                                      'height_band held_k momentum_k mu g slide_m_s').split()]
    fields += [('posture_kd', f * 2), ('swing_kd', f * 2), ('turn_kd', f * 2), ('blas', Blas)]
    fields += [(name, p) for name in 'M h v q com k'.split()]
    fields += [(name, p * 2) for name in 'R quat J6 dJv sole tquat Jr dJr tw'.split()]
    fields += [('nc', i)] + [(name, p * c) for name in 'cJ cp cdJv cf'.split()] + [('cpatch', i * c)]
    fields += [('stance', i * 2), ('nturns', i), ('kdot_given', i), ('swing', i * 2), ('fold', i * 2), ('wep', i), ('let_go', i * 2),
               ('com_acc', f * 3), ('turn', (f * 4) * 2), ('kdot', f * 3), ('settle', (f * 6) * 2), ('edge', f * 2),
               ('strides', f), ('borne', f), ('hold', f), ('sat', f), ('mus', f * 2), ('rests', f * 2), ('slide', f * 3), ('fz_cap', f),
               ('slid_cap', f), ('hand_cap', f), ('residual', f), ('let_cap', f * 2), ('let_hold', f * 2), ('freed', f * 2), ('open_kp', f), ('open_kd', f), ('swing_at', (f * 3) * 2),
               ('swing_speed', (f * 3) * 2),
               ('swing_acc', (f * 3) * 2), ('swing_quat', (f * 4) * 2), ('fold_knee', f * 2), ('fold_rate', f * 2),
               ('fold_acc', f * 2), ('knee_rate', f * 2)]
    fields += [(name, p) for name in 'posture derate clamp wep_nm lo hi'.split()]
    fields += [('tau', f * d), ('qacc', f * n), ('bears', f * 2), ('cop', (f * 2) * 2), ('forces', (f * 3) * c),
               ('scale', f), ('load', f * d), ('cop_n', i), ('cop_none', i * 2), ('over', i * d), ('work', f * work),
               ('chain_lam', f), ('chain_task_lam', f), ('chain_soft', f), ('chain_bleed', f), ('chain_mu', f), ('chain_hold_n', f),
               ('chain_anchor_mu', f), ('chain_w_stick', f), ('chain_reg', f), ('chain_nreg', f),
               ('cbody', i * c), ('chain_w_turn', f), ('chain_w_swing', f), ('chain_w_posture', f), ('chain_turn_kp', f),
               ('chain_turn_kd', f), ('chain_posture_kp', f), ('chain_posture_kd', f), ('chain_w_twist', f), ('chain_twist_kd', f),
               ('chain_vx', f), ('chain_free', i * 2), ('cgive', f * c), ('hang', f * 2),
               ('hang_rate', f), ('cn', p * c), ('chain_held', (i * QP_ROWS) * 2), ('chain_nheld', i * 2),
               ('chain_let', (i * 3) * c), ('chain_nlet', i), ('chain_let_pad', i),
               ('chain_slide_m_s', f), ('chain_w_pin', f), ('chain_tilt', f), ('soles_free', (f * 6) * 2), ('soles', (f * 6) * 2)]
    Step = type('Step', (ctypes.Structure,), {'_fields_': fields})
    if ctypes.sizeof(Step) != size:
        raise ValueError('wbc_step_t is %d bytes, its mirror %d' % (size, ctypes.sizeof(Step)))
    lib.wbh_step.argtypes = lib.wbh_chain.argtypes = [ctypes.POINTER(Step)]
    lib.wbh_step.restype = lib.wbh_chain.restype = ctypes.c_int
    return Step, (lib.wbh_step, lib.wbh_chain), (n, d, c)


def _laid(b, sensed):
    """(the step laid for her law's body `b`, its fields' views): her body's indices and the step's constants laid
    once, her mass, clamps and WEP read at each call - her model's masses move as her wear finds a load. `s` her
    compiled sensing's last (`sensed`, `bodied.on_body`), the step pointed at its block in one write (POINTED)."""
    from machine.stack import rounds, wbc as law, wrench
    Step, _steps, (most_n, most_d, most_c) = _step_typed()
    passive, active = np.flatnonzero(b.passive), np.flatnonzero(b.active_dof)
    if b.n > most_n or len(b.act) > most_d or rounds.NU > most_d:
        raise ValueError('wbc_step.c lays %d dofs and %d drives at most' % (most_n, most_d))
    k = Step(n=b.n, nd=len(b.act), nu0=rounds.NU, nupper=len(rounds.UPPER), nheld=len(b.held), npass=len(passive),
             nact=len(active), npairs=len(b.pairs), nbuffers=len(b.buffers[0]), rounds=rounds.ROUNDS, past=wrench.PAST,
             yaw_m=rounds.YAW_M, damp2=wrench.DAMP ** 2, grow_n=rounds.GROW_N, ridge=wrench.RIDGE,
             contact_k=law.CONTACT_K, height_band=law.HEIGHT_BAND, held_k=law.HELD_K, momentum_k=law.MOMENTUM_K,
             mu=law.MU, g=law.G, slide_m_s=law.SLIDE_M_S)
    k.act[:len(b.act)] = [int(x) for x in b.act]
    for side in range(2):
        k.legs[side][:] = rounds.LEGS[side]
        k.knees[side] = rounds.KNEES[side]
    k.upper[:len(rounds.UPPER)] = rounds.UPPER
    for at, (i, stop, side) in enumerate(b.buffers[0]):
        k.buffer_at[at], k.buffer_stop[at], k.buffer_side[at] = i, stop, side
    k.buffer[:] = b.buffers[1]
    k.arms[:len(rounds.UPPER)] = wrench.ARMED.tolist()
    k.sides[:len(rounds.UPPER)] = wrench.SIDES.tolist()
    k.carpal = borne.CARPAL
    k.held[:len(b.held)] = [int(x) for x in b.held]
    k.passive[:len(passive)] = passive.tolist()
    k.active[:len(active)] = active.tolist()
    for at, (p, r) in enumerate(b.pairs):
        k.pairs[at][0], k.pairs[at][1] = p, r
    k.moment[:rounds.NU] = rounds.MOMENT.tolist()
    k.room[:] = rounds.ROOM.tolist()
    for name in 'COM TURN SWING KNEE POSTURE HEIGHT MOMENT FOLD STOP PART AIR PIN KEEP STILL'.split():
        setattr(k, 'w_' + name.lower(), getattr(wrench, 'W_' + name))
    k.posture_kd[:], k.swing_kd[:], k.turn_kd[:] = law.POSTURE_KD, law.SWING_KD, law.TURN_KD
    k.blas = numpy_blas()
    views = {name: wbc.plain(np.ctypeslib.as_array(getattr(k, name))) for name in VIEWED}
    first = getattr(Step, POINTED[0][0]).offset
    k.sensed, k.placed, k.quick, views['words'] = sensed, None, None, np.frombuffer(k, dtype=np.uint64)
    k.pointed = slice(first // 8, first // 8 + len(POINTED))
    if [getattr(Step, f).offset + 8 * i for f, i, _p in POINTED] != list(range(first, first + 8 * len(POINTED), 8)):
        k.sensed = None
    return k, views


def wrench_laid(b, sensed=None):
    """`machine.stack.wrench.step` in C (wbc_step.c) on her law's body `b`: a solve (b, s, ask) -> its dict, `b.load`
    set (`_laid`)."""
    k, views = _laid(b, sensed)
    k.quick = _quick(k, _step_typed()[1][0])
    return functools.partial(_stepped, k, _step_typed()[1][0], views)


def _quick(k, step):
    """(quick's step, `k`'s stepper, MISS): `_stepped` in one C call (quick_step.c) where the extension is built and
    her sensing pointed at - its struct's FIELDS, `step`'s C, her law's friction -; else None."""
    from machine.stack import wbc as law
    from tools.cores import quick
    mod = quick.module()
    if mod is None or k.sensed is None:
        return None
    Step = type(k)
    laid = mod.stepper(ctypes.addressof(k), ctypes.cast(step, ctypes.c_void_p).value,
                       tuple(getattr(Step, name).offset for name in FIELDS), len(k.cJ), k.pointed.start, len(POINTED),
                       float(law.MU), SENSED, quick.MISS, np.linalg.LinAlgError)
    return mod.step, laid, quick.MISS


def regime():
    """The closed chains' regime as the kernel's text says it (`closed-chain-regime`, `machine.stack.chain.Regime`)."""
    from machine.stack import chain
    from machine.texts import kernel
    return chain.regime(kernel.read()['closed-chain-regime'])


def chain_laid(b, sensed=None, twin=False):
    """`machine.stack.chain.step` in C (wbc_chain.c) on her law's body `b`, laid as `wrench_laid` lays the step, the
    closed chains' `regime` laid once: a solve (b, s, ask) -> its dict; its python twin itself where `twin`."""
    from machine.stack import chain
    said = regime()
    if twin:
        return functools.partial(chain.step, regime=said)
    k, views = _laid(b, sensed)
    k.chain_lam, k.chain_task_lam, k.chain_soft, k.chain_bleed, k.chain_mu = said.lam, said.task, said.soft, said.bleed, said.mu
    k.chain_hold_n, k.chain_anchor_mu, k.chain_w_stick, k.chain_reg, k.chain_nreg = said.hold, said.anchor, said.stick, said.reg, said.nreg
    k.chain_w_turn, k.chain_w_swing, k.chain_w_posture = said.turn, said.swing, said.posture
    k.chain_turn_kp, k.chain_turn_kd, k.chain_posture_kp, k.chain_posture_kd = said.tkp, said.tkd, said.pkp, said.pkd
    k.chain_w_twist, k.chain_twist_kd = said.twist, said.vkd
    k.chain_slide_m_s, k.chain_w_pin, k.chain_tilt = said.slide, said.pin, said.tilt
    k.chain_nheld[0] = k.chain_nheld[1] = 0
    k.chain_nlet = 0
    k.quick = _quick(k, _step_typed()[1][1])
    return functools.partial(_stepped, k, _step_typed()[1][1], views)


#: The step's pointers at what her sensing returns, in the struct's order: (field, index, path).
POINTED = ([(name, 0, (name,)) for name in ('M', 'h', 'v', 'q', 'com', 'k')]
           + [(name, side, ('feet', side, name)) for name in ('R', 'quat', 'J6', 'dJv', 'sole') for side in (0, 1)]
           + [(field, t, ('turns', t, name)) for field, name in (('tquat', 'quat'), ('Jr', 'Jr'), ('dJr', 'dJr'),
                                                                ('tw', 'w')) for t in (0, 1)])


#: What of her sensing the pointed write takes as her compiled sensing's: the same objects, M apart.
SENSED = ('h', 'v', 'q', 'com', 'k', 'feet', 'turns')
#: The struct's fields `_stepped` writes and reads in C, their offsets laid in this order (quick_step.c's enum).
FIELDS = ('n nd nc M cJ cp cdJv cf cpatch cbody cgive stance let_go com_acc nturns turn kdot_given kdot settle edge mus '
          'rests slide fz_cap slid_cap hand_cap let_cap let_hold open_kp open_kd freed strides borne hold sat chain_vx '
          'chain_free swing swing_at swing_speed swing_acc swing_quat fold fold_knee fold_rate fold_acc knee_rate wep '
          'posture derate clamp wep_nm mass lo hi tau qacc bears cop cop_n cop_none over forces load scale '
          'residual hang hang_rate cn soles_free soles').split()
#: The step's fields read and written through numpy views of the struct's own memory.
VIEWED = ('stance', 'let_go', 'com_acc', 'turn', 'kdot', 'settle', 'edge', 'mus', 'rests', 'slide', 'let_cap', 'let_hold', 'freed', 'hang', 'swing', 'fold', 'swing_at', 'swing_speed',
          'swing_acc', 'swing_quat', 'fold_knee', 'fold_rate', 'fold_acc', 'knee_rate', 'tau', 'qacc',
          'bears', 'cop',
          'forces',
          'load', 'cop_none', 'over', 'soles_free', 'soles')


def _spread(a, n):
    """`a` as `n` doubles: itself where it is so, else broadcast (np.broadcast_to's setup alone
    took a third of a call's own time)."""
    a = np.asarray(a, float)
    return a if a.shape == (n,) else np.broadcast_to(a, (n,))


def _stepped(k, step, v, b, s, ask):
    """wrench.step(b, s, ask) on the laid `k`: what `s` senses pointed at - her compiled sensing's
    last in one write, its M as `s` has it -, what `ask` asks copied through `v`, its fields' views; in one C call
    where `k.quick` is laid and all of it is as laid (quick_step.c: in Python 27 us a law pass about its C)."""
    q = k.quick
    if q is not None:
        got = q[0](q[1], k, b, s, ask)
        if got is not q[2]:
            return got
    from machine.stack import wbc as law
    kept = []

    def at(a):
        a = np.ascontiguousarray(a, dtype=np.float64)
        kept.append(a)
        return wbc.address(a)
    last = getattr(k.sensed, 'last', None)
    if last is not None and all(map(operator.is_, map(s.get, SENSED), map(last[0].get, SENSED))):
        c = k.sensed.c
        if k.placed is None or k.placed[0] is not c:
            k.placed = (c, np.array([8 * c.place(path) for _f, _i, path in POINTED], dtype=np.uint64))
        np.add(k.placed[1], last[1], out=v['words'][k.pointed])
        k.M = at(s['M'])
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
        k.cJ[at_], k.cp[at_], k.cdJv[at_], k.cf[at_], k.cn[at_] = (at(c[name]) for name in ('J', 'p', 'dJv', 'f', 'n'))
        k.cpatch[at_], k.cbody[at_], k.cgive[at_] = int(c.get('patch', 0)), int(c.get('body', 0)), float(c.get('give', 0.0))
    v['stance'][:] = [bool(x) for x in ask['stance']]
    v['let_go'][:] = [bool(x) for x in ask.get('let_go', (False, False))]
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
    v['rests'][:] = ask.get('rests', (0.0, 0.0))
    v['slide'][:] = ask.get('slide', (0.0, 0.0, 0.0))
    k.fz_cap, k.slid_cap = float(ask.get('fz_cap', np.inf)), float(ask.get('slid_cap', np.inf))
    k.hand_cap = float(ask.get('hand_cap', 0.0))
    v['let_cap'][:] = ask.get('let_cap', (np.inf, np.inf))
    v['let_hold'][:] = ask.get('let_hold', (0.0, 0.0))
    k.open_kp, k.open_kd, v['freed'][:] = ask.get('open', (0.0, 0.0, (0.0, 0.0)))
    v['hang'][:], k.hang_rate = ask.get('hang', (0.0, 0.0)), float(ask.get('hang_rate', 0.0))
    k.strides, k.borne, k.hold = float(ask.get('strides', 0.0)), float(ask.get('borne', 0.0)), float(ask.get('hold', 1.0))
    k.sat, k.chain_vx = float(ask.get('sat', 0.0)), float(ask.get('twist', (0.0,))[0])
    k.chain_free[0], k.chain_free[1] = (int(bool(x)) for x in ask.get('free', (False, False)))
    swing, fold = ask.get('swing', {}), ask.get('fold', {})
    for side in range(2):
        v['swing'][side] = side in swing
        if side in swing:
            v['swing_at'][side], v['swing_speed'][side], v['swing_acc'][side], v['swing_quat'][side] = swing[side]
        v['fold'][side] = side in fold
        if side in fold:
            v['fold_knee'][side], v['fold_rate'][side], v['fold_acc'][side] = fold[side]
    v['knee_rate'][:] = ask.get('knee_rate', (0.0, 0.0))
    k.wep = int(bool(ask.get('wep')))
    k.posture = at(ask['posture'])
    k.derate = at(_spread(ask.get('derate', 1.0), k.nd))
    k.clamp, k.wep_nm, k.mass = at(b.clamp), at(b.wep), b.mass
    k.lo, k.hi = at(_spread(ask.get('lo', -np.inf), k.nd)), at(_spread(ask.get('hi', np.inf), k.nd))
    if step(ctypes.byref(k)):
        raise np.linalg.LinAlgError('Singular matrix')
    nd, cop = k.nd, [None if v['cop_none'][side] else v['cop'][side].copy() for side in range(k.cop_n)]
    b.load = v['load'][:nd].copy()
    return {'tau': v['tau'][:nd].copy(), 'qacc': v['qacc'][:k.n].copy(), 'bears': v['bears'].tolist(), 'cop': cop,
            'over': v['over'][:nd].astype(bool), 'slack': [], 'held': (),
            'borne': v['forces'][:k.nc].reshape(-1, 3).copy(), 'scale': k.scale, 'residual': k.residual,
            'soles_free': v['soles_free'].copy(), 'soles': v['soles'].copy()}
