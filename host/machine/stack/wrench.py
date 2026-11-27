"""The robot's whole body's step: one weighted least squares within its soles' and contacts' bounds.

    out = wrench.step(body, s, ask)         # `wbc.step`'s ask in, its {tau, qacc, bears, ..} out

Twenty-one unknowns: six a sole - its wrench where it stands, its acceleration where not - and the robot's
upper body's nine accelerations; three a contact (`machine.senses.borne`): the force on a segment that bears
it, its point held still, kinetic friction where it is felt sliding as its text allows (`ask['slide']`), under
`ask['fz_cap']` and a slider's `ask['slid_cap']`; one surface's segments a patch, split as felt. Every joint
follows: a leg through its sole, the pelvis by the six base rows. One weighted least squares - the robot's
centre of mass, the turns, a swing, the posture, its standing knees, its height, its angular momentum -
within each sole's push, pyramid and centre of pressure, each contact's push and friction and the robot's
height's band, by the stack's dual active set (`within`); striding, what the floor cannot give goes to the
robot's inertia (`rounds.taken`). A drive the ask takes past its clamp shrinks the ask (`shrunk`). wbc_step.c is
it in C bit for bit; the board's wbc_wrench.c still shrinks the ask as one (docs/findings/wrench.md).
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import libm
from machine.stack.pressed import W_STILL, patches
from machine.stack.rounds import DRIVEN, KNEES, LEGS, MOMENT, NU, ROOM, UPPER
from machine.stack.shrunk import box, overloaded, paired, paired_limit, paired_shrunk
from machine.stack.within import W_KEEP, within
from machine.stack.wbc import CONTACT_K, HEIGHT_BAND, HELD_K, MOMENTUM_K, MU, POSTURE_KD, SWING_KD, TURN_KD, G, turn_error


#: A leg's solve's damping: at 1e-3 a swung leg straight at the knee was asked 17 000 rad/s^2, at 0.05
#: the robot left the floor 8 mm where 20.
DAMP = 0.02
#: The rows' weights. The robot's centre of mass along the floor, m/s^2, all but exact - sat (`ask['sat']`) as the
#: robot's turns', its balance its trunk's and hips' (`xform-balance-biped-to-seated`): all but exact
#: there its trunk could not come to its pitch without moving it, and sat back 30-50 deg, the robot's spine
#: flexed 50 deg where its aim was -7 -; the turns and a swinging sole, rad/s^2 and m/s^2 - the sole at a third of the
#: turns the walk fell at its 7th step -; the robot's standing knees' posture, rad/s^2 - at 1, 60 N felled its
#: 2 times of 8 -; the posture, its height, its angular momentum (N m), a swinging knee's fold, a
#: buffer; a wrench's parts in shares of its weight, a sole's acceleration left alone, a pinned part.
W_COM, W_TURN, W_SWING, W_KNEE, W_POSTURE, W_HEIGHT, W_MOMENT = 1000.0, 30.0, 30.0, 3.0, 1.0, 1.0, 0.3
W_FOLD, W_STOP, W_PART, W_AIR, W_PIN = 3.0, 30.0, 0.1, 1e-3, 1e5
#: The robot's rows' groups in the order they are laid: what `got['rows_free']` tells each one's weighted
#: residual by.
GROUPS = ('com', 'height', 'turns', 'posture', 'swing', 'knee', 'fold', 'stop', 'momentum', 'spare')


_EYE3, _EYE6, _EYE_NU, _UP = np.eye(3), np.eye(6), np.eye(NU), np.array([0.0, 1.0, 0.0])
#: A hung leg's chain below its knee (`ask['hang']`), a leg's joints in `rounds.LEGS`' order: the kernel's hang's
#: tibiofemoral, talocrural and calcaneus (`modulate-unloaded-limb-jet`).
PASSIVE = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 1.0])
_UPPER_IDX = 12 + np.arange(len(UPPER))
_UPPER_ROWS = _EYE_NU[12:NU]
#: The robot's arms' rows `ask['hold']` times the posture's, else the step pitched its trunk by them:
#: shoulders 130 deg at 939 deg/s sitting, 255 at 1 633 lying back; hands' jerk 15-21 000 m/s^3 seated, 1
#: 400-1 800 walking. Not kept: all upper drives at W_TURN (4 800-6 700, 900; pelvis rolled 3 deg, a woman's
#: 5; walk 2.75 off, 2.55); the arms alone (a crouch's spine folded, fell).
ARMED = np.array([float(DRIVEN[i].endswith(('shoulder', 'elbow'))) for i in UPPER])
#: Each upper drive's side, the robot's left 0 and its right 1, -1 neither: an arm whose hand bears is held
#: by its contact, as a pressing contact's point is, not by its rows - held by both, the first hand down drove the
#: robot's trunk round 8 cm at the other's and lifted it 5 cm off the seat.
SIDES = np.array([0 if DRIVEN[i].startswith('left_') else 1 if DRIVEN[i].startswith('right_') else -1 for i in UPPER])


def _loose(contacts):
    """ARMED but where a hand bears (`contacts`' `carpal`, `side`): held by its contact."""
    return ARMED * (1.0 - np.isin(SIDES, [int(c['side']) for c in contacts if c.get('carpal')]))


def _inv(A, damp2):
    """A's damped inverse, `damp2` lambda^2."""
    return np.linalg.solve(A.T @ A + damp2 * _EYE6, A.T)


def _maps(b, s, ask, want):
    """(T, E, t0, a6, asked): qdd = T base + E u + t0; each swinging sole's asked acceleration and
    whether asked. A standing sole is still, or goes as `ask['settle']` has it (its turn and its
    middle's acceleration, world)."""
    n, M, h, v, act = b.n, s['M'], s['h'], s['v'], b.act
    T, E, t0 = np.zeros((n, 6)), np.zeros((n, NU)), np.zeros(n)
    T[:6] = _EYE6
    t0[b.held] = -HELD_K * v[b.held]
    E[act[UPPER], _UPPER_IDX] = 1.0
    a6, asked, swing, let_go = np.zeros(NU), np.zeros(2), ask.get('swing', {}), ask.get('let_go', (False, False))
    settle = ask.get('settle', np.zeros((2, 6)))
    for k, leg in enumerate(LEGS):
        f, cols = s['feet'][k], act[leg]
        twist = f['J6'] @ v
        if not ask['stance'][k] and k not in swing:
            # let go: an open chain (`machine.moves.emancipation`), each joint on its posture as the swing's
            # row asks it, what its foot rests on holding it (`borne.read`), its own weight in the robot's
            # dynamics; its hip's acceleration given, a squat's let-go hips pressed the robot's soles and
            # stood it up off its hands on the floor; its knee and ankle hanging, the robot's
            # heels held on the floor and its hips' extension lifted its pelvis 0.157 -> 0.25
            # m. Not let go, on its posture
            hung = float(bool(let_go[k]))
            kp, kd, freed = ask.get('open', (0.0, 0.0, np.zeros(2)))
            opened = freed[k] * (kp * (np.asarray(ask['posture'])[leg] - s['q'][leg]) - kd * v[cols]) / np.diag(M)[cols]
            opened = opened + (1.0 - freed[k]) * want[leg]
            # hung off a seat (`ask['hang']`, its share): its knee and ankle as gravity swings them, damped at the
            # kernel's pendulum (`ask['hang_rate']`) - asked so, the robot's drives hold none of it and what its
            # foot rests on bears its own weight; held on its chain instead its toes pressed a low chair's floor at
            # 133 N
            passive = -h[cols] / np.diag(M)[cols] - float(ask.get('hang_rate', 0.0)) * v[cols]
            opened = opened + (float(ask.get('hang', (0.0, 0.0))[k]) * PASSIVE) * (passive - opened)
            t0[cols] = want[leg] * (1.0 - hung)
            E[cols, 6 * k:6 * k + 6] = hung * _EYE6
            a6[6 * k:6 * k + 6] = hung * opened
            asked[k] = hung
            continue
        inv = _inv(f['J6'][:, cols], DAMP * DAMP)
        T[cols] = -inv @ f['J6'][:, :6]
        if ask['stance'][k]:
            t0[cols] = inv @ (settle[k] - f['dJv'] - CONTACT_K * twist)
            continue
        at, speed, acc, quat = swing[k]
        a6[6 * k:6 * k + 3] = TURN_KD[0] * turn_error(quat, f['quat']) - TURN_KD[1] * twist[:3]
        a6[6 * k + 3:6 * k + 6] = (np.asarray(acc) + SWING_KD[0] * (np.asarray(at) - f['sole'])
                                   + SWING_KD[1] * (np.asarray(speed) - twist[3:]))
        asked[k] = 1.0
        t0[cols] = -inv @ f['dJv']
        E[cols, 6 * k:6 * k + 6] = inv
    # the joints no drive holds by their own rows of the robot's dynamics
    passive = getattr(b, 'passive', None)
    if passive is None:
        passive = np.array([i >= 6 and i in b.free for i in range(n)])
    active = ~passive
    m_pp = M[np.ix_(passive, passive)]
    pull = -np.linalg.solve(m_pp, M[np.ix_(passive, active)])
    T[passive], E[passive] = pull @ T[active], pull @ E[active]
    t0[passive] = pull @ t0[active] - np.linalg.solve(m_pp, h[passive])
    return T, E, t0, a6, asked


def _residual(R0, rhs, first, grouped):
    """Each of GROUPS' summed squared weighted residual at the first, floor-unbounded answer `first`: a row's its own
    dot, summed in row order (wbc_step.c's, `wbc_dot`)."""
    res = np.array([row @ first for row in R0]) - np.concatenate(rhs)
    return np.bincount(np.asarray(grouped, int), res * res, minlength=len(GROUPS))


def step(b, s, ask, solve=None):
    """{tau, qacc, bears, cop, over, slack, held, scale, soles_free, soles, rows_free}: `ask` as `wbc.step`
    takes it; `strides` how far the robot strides (0 none), its balance then its landings',
    not its rows'; `scale` what its drives carry of it; each sole's six unknowns in the first
    answer, the floor unbounded, and in the last; each GROUPS' residual. `solve` the answer's (`within`; the board's
    rounds `clipped.solve`)."""
    solve = within if solve is None else solve
    contacts = s.get('contacts', ())
    stance = np.array([float(bool(k)) for k in ask['stance']])
    n, M, h, v, feet, act, mass = b.n, s['M'], s['h'], s['v'], s['feet'], b.act, b.mass
    k3, nu = 3 * len(contacts), NU + 3 * len(contacts)
    each, grounded = np.repeat(stance, 6), max(float(stance.max()), float(k3 > 0))
    turns, a = ask.get('turns', ()), np.asarray(ask['com_acc'], float)
    turned, kdot = float(len(turns) > 0), ask.get('kdot')
    want = POSTURE_KD[0] * (np.asarray(ask['posture']) - s['q']) - POSTURE_KD[1] * v[act]
    T, E, t0, a6, asked = _maps(b, s, ask, want)
    if k3:
        E, a6 = np.hstack([E, np.zeros((n, k3))]), np.concatenate([a6, np.zeros(k3)])
    # the base's rows of the robot's dynamics: A base = Gm u - M_b E u - d
    Gm = np.zeros((6, nu))
    Gm[:, :6] = feet[0]['J6'][:, :6].T * stance[0]
    Gm[:, 6:12] = feet[1]['J6'][:, :6].T * stance[1]
    for i, c in enumerate(contacts):
        Gm[:, NU + 3 * i:NU + 3 * i + 3] = c['J'][:, :6].T
    Ai = np.linalg.inv(M[:6] @ T + 1e-9 * _EYE6)
    P, p0 = Ai @ (Gm - M[:6] @ E), -Ai @ (M[:6] @ t0 + h[:6])
    full, full0 = T @ P + E, T @ p0 + t0
    rows, rhs, rhs0, grouped = [], [], [], []

    def lay(R, c, weight, c0=None, group=0):
        """A row = `c`, weighed; `c0` what it is with nothing asked, every acceleration 0."""
        c, w = np.atleast_1d(c), np.asarray(weight, float)
        rows.append((w[:, None] if w.ndim else w) * (R if R.ndim == 2 else R[None, :]))
        rhs.append(w * c)
        rhs0.append(w * np.broadcast_to(np.asarray(0.0 if c0 is None else c0, float), c.shape))
        grouped.extend([group] * len(rhs[-1]))

    sumf, about = np.zeros((3, nu)), np.zeros((3, nu))
    for k, f in enumerate(feet):
        r, sk = f['sole'] - s['com'], stance[k]
        sumf[:, 6 * k + 3:6 * k + 6] = sk * _EYE3
        about[:, 6 * k:6 * k + 3] = sk * _EYE3
        about[:, 6 * k + 3:6 * k + 6] = sk * np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]],
                                                       [-r[1], r[0], 0.0]])
    for i, c in enumerate(contacts):
        r, at = c['p'] - s['com'], slice(NU + 3 * i, NU + 3 * i + 3)
        sumf[:, at] = _EYE3
        about[:, at] = np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]])
    still, caps, slips = patches(contacts, nu, full, full0, v, ask, W_PIN * float(stance.max()) / (mass * G))
    lay(sumf[[0, 2]] / mass, a[[0, 2]], (W_COM + (W_TURN - W_COM) * float(ask.get('sat', 0.0))) * grounded, group=0)
    lay(sumf[1] / mass, a[1] + G, W_HEIGHT * grounded, G, group=1)
    for t, turn in zip(s['turns'], turns):
        left = -t['dJr'] - t['Jr'] @ full0
        lay(t['Jr'] @ full, TURN_KD[0] * turn_error(turn, t['quat']) - TURN_KD[1] * t['w'] + left, W_TURN * turned, left, group=2)
    lay(_UPPER_ROWS if not k3 else np.eye(nu)[12:NU], want[UPPER], W_POSTURE * (1.0 + (ask.get('hold', 1.0) - 1.0) * _loose(contacts)), group=3)
    edges, mu, strides = ask.get('edge', (0.0, 0.0)), ask.get('mu', (MU, MU)), float(ask.get('strides', 0.0))
    swung = np.concatenate([np.repeat(asked, 6), np.zeros(nu - 12)])
    resting = np.asarray(ask.get('rests', (0.0, 0.0)), float) * (1.0 - np.asarray(ask.get('let_go', (False, False)), float))
    lay(np.diag(swung), a6, W_SWING + (W_STILL - W_SWING) * np.concatenate([np.repeat(resting, 6), np.zeros(nu - 12)]), group=4)
    knees, left = act[KNEES], -float(stance @ full0[act[KNEES]])
    # the knees' rows damped against the posture's rate: against still, a push's knees were held
    # at 0.2 of their clamp and the robot's height at its band's floor
    knee_rate = np.asarray(ask.get('knee_rate', (0.0, 0.0)), float)
    lay(stance @ full[knees], float(stance @ (want[KNEES] + POSTURE_KD[1] * knee_rate)) + left, W_KNEE * grounded, left, group=5)
    for k, (knee, rate, acc) in sorted(ask.get('fold', {}).items()):
        j = knees[k]
        lay(full[j], acc + SWING_KD[0] * (knee - s['q'][KNEES[k]]) + SWING_KD[1] * (rate - v[j]) - full0[j], W_FOLD,
            -full0[j], group=6)
    stops, (width, stiff, damp, reach) = b.buffers
    for i, stop, side in stops:
        j, depth = act[i], side * (stop - s['q'][i]) + width
        lay(full[j], stiff * side * max(depth, 0.0) - damp * v[j] - full0[j],
            W_STOP * np.clip((depth + reach * max(-side * v[j], 0.0)) / width, 0.0, 1.0), -full0[j], group=7)
    lay(about, -MOMENTUM_K * s['k'] if kdot is None else np.asarray(kdot), W_MOMENT * grounded, group=8)
    part, inv_mg, inv_mgr = np.zeros(nu), 1.0 / (mass * G), 1.0 / (mass * G * ROOM[0])
    part[:12], part[NU:] = (inv_mgr, inv_mgr, inv_mgr, inv_mg, inv_mg, inv_mg) * 2, inv_mg
    spare = W_PART * np.concatenate([each, np.zeros(NU - 12), np.ones(k3)]) * part + W_AIR * np.concatenate([1.0 - each, np.zeros(nu - 12)])
    lay(np.diag(spare), np.zeros(nu), 1.0, group=9)
    R0, up = np.vstack(rows), sumf[1] / mass
    H0 = R0.T @ R0 + 1e-12 * (_EYE_NU if not k3 else np.eye(nu))
    frames, moment = [f['R'] for f in feet], (MOMENT if not k3 else np.concatenate([MOMENT, np.zeros(k3)]))
    keep = W_KEEP * np.concatenate([each, np.zeros(nu - 12)]) * part
    kinetic = (full.T @ M @ full + np.diag(libm.sq(W_SWING * swung + spare) + keep * keep)) if strides > 0.0 else None
    net = np.vstack([about[:, :12], sumf[:, :12]]) if float(stance.min()) > 0.0 else None
    geo, stride = (nu, k3, moment, part, up, sumf[1], mass, grounded, stance, frames, edges, mu, caps, slips, 1.0, net), (strides, kinetic)
    felt = [c['f'][1] for c in contacts]

    def torques(u, alpha=1.0):
        """(the drives' torques, every acceleration) at the unknowns `u`."""
        qdd = full @ u + (full0 if alpha == 1.0 else alpha * full0)
        tau = (M @ qdd + (h if alpha == 1.0 else alpha * h))[act]
        for i, c in enumerate(contacts):
            tau = tau - c['J'][:, act].T @ u[NU + 3 * i:NU + 3 * i + 3]
        for k, f in enumerate(feet):
            tau = tau - f['J6'][:, act].T @ (stance[k] * u[6 * k:6 * k + 6])
        return tau, qdd

    # the ask's answer, and nothing's: every acceleration 0, the robot's weight borne, by the same rows
    whole, first = solve(H0, R0.T @ np.concatenate(rhs), [(S, r) for S, r, _r0 in still], felt,
                         a[1] - HEIGHT_BAND, a[1] + HEIGHT_BAND, geo, stride)
    limit, lo, hi = box(b, ask)
    tau, qdd = torques(whole)
    plim, ptau = paired_limit(limit, b.pairs, b.kt), paired(tau, b.pairs, b.kt)
    if (np.abs(tau) > limit).any() or (np.abs(ptau) > plim).any():
        stood, _first = solve(H0, R0.T @ np.concatenate(rhs0), [(S, r0) for S, _r, r0 in still], felt,
                              -HEIGHT_BAND, HEIGHT_BAND, geo, stride)
        tau0 = torques(stood)[0]
        ptau0 = paired(tau0, b.pairs, b.kt)
        scale = paired_shrunk(tau, tau0, ptau, ptau0, limit, plim)
        u = whole + (scale - 1.0) * (whole - stood)
        zeros = np.zeros(len(limit))
        s0 = (paired_shrunk(tau + (scale - 1.0) * (tau - tau0), zeros, ptau + (scale - 1.0) * (ptau - ptau0), zeros,
                            limit, plim)
              if ((np.abs(tau0) > limit).any() or (np.abs(ptau0) > plim).any()) else 1.0)
        scale, u = s0 * scale, (s0 * u if s0 < 1.0 else u)
        carried, qdd = torques(u, s0)
    else:
        scale, u, carried = 1.0, whole, tau
    bears, cop = [0.0, 0.0], []
    for k, f in enumerate(feet):
        wk = stance[k] * u[6 * k:6 * k + 6]
        bears[k] = float(wk[4])
        if ask['stance'][k]:
            ml = f['R'].T @ wk[:3]
            cop.append(np.array([ml[2], -ml[0]]) / wk[4] if wk[4] > 1e-6 else None)
    # the drives asked past their clamps and their loads: the ask's, before it was shrunk
    over, b.load = overloaded(tau, ptau, b.clamp, b.pairs, b.kt)
    return {'tau': np.clip(carried, lo, hi), 'qacc': qdd, 'bears': bears, 'cop': cop, 'over': over,
            'slack': [], 'held': (), 'borne': u[NU:].reshape(-1, 3), 'scale': scale,
            'soles_free': first[:12].reshape(2, 6), 'soles': u[:12].reshape(2, 6), 'rows_free': _residual(R0, rhs, first, grouped)}

