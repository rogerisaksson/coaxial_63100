"""Her whole body's step without an active set: a fixed run of small solves, masks and clips.

    out = wrench.step(body, s, ask)         # `wbc.step`'s ask in, its {tau, qacc, bears, ..} out

Twenty-one unknowns: six a sole - its wrench where it stands, its acceleration where not - and her
upper body's nine accelerations; three a contact (`machine.borne`): the force on a segment that bears
her, its point held still, kinetic friction where it is felt sliding as its text allows (`ask['slide']`,
`rounds.pushed`), under `ask['fz_cap']` and a slider's `ask['slid_cap']`; one surface's segments a
patch, split as felt. Every joint follows: a leg through its sole, the pelvis by the six base rows.
One weighted least squares - her centre of mass, the turns, a swing, the posture, her standing knees,
her height, her angular momentum - ROUNDS times, each wrench clipped into its sole's bounds and the
clipped pinned for the next; striding, what the floor cannot give goes to her inertia (`_taken`). A
drive the ask takes past its clamp is pinned at it and the rest solved round it, ROUNDS times at most.
wbc_step.c is it in C bit for bit; the board's wbc_wrench.c still shrinks the ask as one (docs/findings/wrench.md).
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.pressed import W_STILL, patches
from machine.rounds import DRIVEN, KNEES, LEGS, MOMENT, NU, ROOM, ROUNDS, UPPER, bounded, pushed, taken
from machine.shrunk import box
from machine.wbc import CONTACT_K, HEIGHT_BAND, HELD_K, MOMENTUM_K, MU, POSTURE_KD, SWING_KD, TURN_KD, G, turn_error


#: A leg's solve's damping: at 1e-3 a swung leg straight at the knee was asked 17 000 rad/s^2, at 0.05
#: she left the floor 8 mm where 20.
DAMP = 0.02
#: Rounds more, at most, for an answer past her height's band after them.
PAST = 2
#: The rows' weights. Her centre of mass along the floor, m/s^2, all but exact; the turns and a
#: swinging sole, rad/s^2 and m/s^2 - the sole at a third of the turns the walk fell at its 7th
#: step -; her standing knees' posture, rad/s^2 - at 1, 60 N felled her 2 times of 8 -; the
#: posture, her height, her angular momentum (N m), a swinging knee's fold, a buffer; a
#: wrench's parts in shares of her weight, a sole's acceleration left alone, a pinned part.
W_COM, W_TURN, W_SWING, W_KNEE, W_POSTURE, W_HEIGHT, W_MOMENT = 1000.0, 30.0, 30.0, 3.0, 1.0, 1.0, 0.3
W_FOLD, W_STOP, W_PART, W_AIR, W_PIN = 3.0, 30.0, 0.1, 1e-3, 1e5
#: Striding, a standing sole's wrench kept in the projection's metric, of its share of her weight:
#: her soles pushed against each other change her motion by nothing; unkept the faces were met through
#: it - 926 N along one sole, -906 the other - till a sole pulled and both were let go, 11 times in 12 s
#: of her walk. At 10 none, her walk 418 J/m, 2.53 off a woman's (412, 2.61); at 30 2.84; at 100 let go
#: 187 times.
W_KEEP = 3.0
#: A solve with no answer is solved again with this share of its largest diagonal on its free unknowns:
#: falling on a ramp, two bearing points 16 mm apart left the rounds' matrix rank 21 of 27 (cond 2.9e19),
#: their split said by no row, and the solve raised.
RIDGE = 1e-12


_EYE3, _EYE6, _EYE_NU, _UP = np.eye(3), np.eye(6), np.eye(NU), np.array([0.0, 1.0, 0.0])
_UPPER_IDX = 12 + np.arange(len(UPPER))
_UPPER_ROWS = _EYE_NU[12:NU]
#: Her arms' rows `ask['hold']` times the posture's, else the step pitched her trunk by them: shoulders
#: 130 deg at 939 deg/s sitting, 255 at 1 633 lying back; hands' jerk 15-21 000 m/s^3 seated, 1 400-1 800
#: walking. Not kept: all upper drives at W_TURN (4 800-6 700, 900; pelvis rolled 3 deg, a woman's 5;
#: walk 2.75 off, 2.55); the arms alone (a crouch's spine folded, fell).
ARMED = np.array([float(DRIVEN[i].endswith(('shoulder', 'elbow'))) for i in UPPER])
#: Each upper drive's side, her left 0 and her right 1, -1 neither: an arm whose hand bears is held by its contact,
#: as a pressing contact's point is, not by its rows - held by both, the first hand down drove her trunk round 8 cm
#: at the other's and lifted it 5 cm off the seat.
SIDES = np.array([0 if DRIVEN[i].startswith('left_') else 1 if DRIVEN[i].startswith('right_') else -1 for i in UPPER])


def _loose(contacts):
    """ARMED but where a hand bears (`contacts`' `carpal`, `side`): held by its contact."""
    return ARMED * (1.0 - np.isin(SIDES, [int(c['side']) for c in contacts if c.get('carpal')]))


def _inv(A, damp2):
    """A's damped inverse, `damp2` lambda^2."""
    return np.linalg.solve(A.T @ A + damp2 * _EYE6, A.T)


def _solved(K, rhs, free):
    """K^-1 rhs, K ridged by RIDGE on its `free` unknowns where it has no inverse."""
    try:
        return np.linalg.solve(K, rhs)
    except np.linalg.LinAlgError:
        return np.linalg.solve(K + np.diag(free * RIDGE * float(np.diag(K).max())), rhs)


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
            # let go: its hip on its posture as the swing's row asks it, what its foot rests on holding it
            # (`borne.read`), its knee and ankle hanging (below); its hip's acceleration given, a squat's let-go
            # hips pressed her soles and stood her up off her hands on the floor. Not let go, on its posture
            hung = float(bool(let_go[k]))
            t0[cols] = want[leg]
            t0[cols[:3]] *= 1.0 - hung
            E[cols[:3], 6 * k:6 * k + 3] = hung * _EYE3
            a6[6 * k:6 * k + 3] = hung * want[leg[:3]]
            asked[k] = hung
            continue
        inv = _inv(f['J6'][:, cols], DAMP ** 2)
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
    # the joints no drive holds, and a let-go leg's knee and ankle, by their own rows of her dynamics
    passive = getattr(b, 'passive', None)
    if passive is None:
        passive = np.array([i >= 6 and i in b.free for i in range(n)])
    hung = np.zeros(n, bool)
    for k, leg in enumerate(LEGS):
        hung[act[leg][3:]] = bool(let_go[k])
    passive = passive | hung
    active = ~passive
    m_pp = M[np.ix_(passive, passive)]
    pull = -np.linalg.solve(m_pp, M[np.ix_(passive, active)])
    T[passive], E[passive] = pull @ T[active], pull @ E[active]
    t0[passive] = pull @ t0[active] - np.linalg.solve(m_pp, h[passive])
    return T, E, t0, a6, asked


def _rounds(H0, g0, held, bears, lo, hi, geo, stride):
    """The unknowns for the rows' normal matrix `H0` and right-hand side `g0` (R^T c), the contacts'
    `held` rows (`bears`, N up), her height's band `lo` to `hi`: the rounds, then the stride. `geo`: (nu, k3,
    moment, part, up, lift, mass, grounded, stance, frames, edges, mu, caps, slips); `stride`: (strides, her
    kinetic metric)."""
    nu, k3, moment, part, up, lift, mass, grounded, stance, frames, edges, mu, cap, slips = geo
    strides, kinetic = stride
    pinned, at, band, edge = np.zeros(nu), np.zeros(nu), 0.0, 0.0
    # a contact told to bear nothing is let go, as one the step would have pull
    pressed, done, rounds = (np.broadcast_to(cap, (len(bears),)) > 0.0).astype(float), 0, ROUNDS
    Hr = gr = None
    while done < rounds:
        if held:
            Hr = H0 + sum(p * S.T @ S for p, (S, _r) in zip(pressed, held))
            gr = g0 + sum(p * S.T @ r for p, (S, r) in zip(pressed, held))
        else:
            Hr, gr = H0, g0
        # a clipped force is held by a row, W_PIN its weight's share; a clipped moment is its
        # value and her height at its band's edge is met, the rest solved round them: as rows
        # of W_PIN they were 1e7 on the diagonal, and what her soles share came 0.6 N m apart
        # from one solve to another
        hard, soft = pinned * moment, (W_PIN * pinned * (1.0 - moment) * part) ** 2
        s_t, rhs_s = np.zeros((nu, nu)), soft * at
        if k3:
            c_idx, n_idx = NU + np.arange(k3), NU + 3 * (np.arange(k3) // 3) + 1
            cone = (c_idx != n_idx) & (pinned[n_idx] == 0.0) & (at[n_idx] > 1e-9)
            rc = cone * (at[c_idx] / np.maximum(at[n_idx], 1e-9))
            s_t[c_idx, n_idx] = -soft[c_idx] * rc
            s_t[n_idx, c_idx] = -soft[c_idx] * rc
            np.add.at(s_t, (n_idx, n_idx), soft[c_idx] * (rc * rc))
            rhs_s[c_idx] *= 1.0 - cone
        H, free = Hr + np.diag(soft) + s_t, 1.0 - hard
        K = H * np.outer(free, free) + np.diag(hard)
        u, way = _solved(K, np.array([free * (gr + rhs_s - H @ (hard * at)) + hard * at, up]).T, free).T
        u = u + band * (edge - up @ u) / (up @ way + 1.0 - band) * way
        # her height's band: past an edge, held to it
        ay = float(lift @ u) / mass - G
        past = min(max(ay, lo), hi) + G
        band, edge = max(band, grounded * float(abs(past - G - ay) > 1e-9)), edge + (1.0 - band) * (past - edge)
        at, moved = bounded(u, frames, stance, edges, mu)
        if k3:
            at, kept = pushed(at, bears, cap, slips)
            pressed = pressed * (u[NU + 1:NU + k3:3] >= 0.0)
            pinned = np.maximum(pinned, np.maximum(moved, kept))
        else:
            pinned = np.maximum(pinned, moved)
        done += 1
        # an answer past her height's band after the rounds - a contact let go in the last, its
        # solve never held to it - is held to its edge: off a mattress's edge the step planned 145
        # N up of her 358 asked and the edge took 309 along its normal, 23 deg ahead
        ay = float(lift @ at) / mass - G
        if (done == rounds < ROUNDS + PAST and not strides
                and grounded * abs(min(max(ay, lo), hi) - ay) > 1e-9):
            band, edge, rounds = 1.0, min(max(ay, lo), hi) + G, rounds + 1
        if (not pinned.any() and not band) or (strides >= 1.0 and k3 == 0):
            break
    if strides > 0.0 and (pinned.any() or band):
        return at + strides * (taken(_solved(Hr, gr, np.ones(nu)), kinetic, up, frames, stance, edges, (lo, hi),
                                      grounded, mu) - at)
    return at


def _pinned(solve, torques, A, c0, limit):
    """(the unknowns, the drives' torques the ask takes and those carried, every acceleration, the share of drives
    free): a drive the ask takes past its clamp held at it and the rest solved round it, ROUNDS times at most, as a
    clipped contact force is (`_rounds`) - `solve(dH, dg)` the rounds on the rows' normal matrix and right-hand side,
    those added, `A` u + `c0` the drives' torques, `torques(u)` them and every acceleration. Shrunk as one toward
    nothing asked, a palm's arm at its clamp under her took the whole ask to 0.08 of it and she fell limp onto her
    hands, an arm flung 0.99 m up."""
    u = solve()
    tau, qdd = torques(u)
    carried, pinned, at = tau, np.zeros(len(limit), bool), np.zeros(len(limit))
    for _ in range(ROUNDS):
        over = (np.abs(carried) > limit) & ~pinned
        if not over.any():
            break
        at, pinned = np.where(over, np.clip(carried, -limit, limit), at), pinned | over
        w = W_PIN / limit[pinned]
        Ap = w[:, None] * A[pinned]
        u = solve(Ap.T @ Ap, Ap.T @ (w * (at[pinned] - c0[pinned])))
        carried, qdd = torques(u)
    return u, tau, carried, qdd, 1.0 - float(pinned.sum()) / len(limit)


def step(b, s, ask):
    """{tau, qacc, bears, cop, over, slack, held, scale}: `ask` as `wbc.step` takes it; `strides` how far
    she strides (0 none), her balance then her landings', not her rows'; `scale` what her drives carry of it."""
    n, M, h, v, feet, act, mass = b.n, s['M'], s['h'], s['v'], s['feet'], b.act, b.mass
    contacts = s.get('contacts', ())
    k3, nu = 3 * len(contacts), NU + 3 * len(contacts)
    stance = np.array([float(bool(k)) for k in ask['stance']])
    each, grounded = np.repeat(stance, 6), max(float(stance.max()), float(k3 > 0))
    turns = ask.get('turns', ())
    turned = float(len(turns) > 0)
    want = POSTURE_KD[0] * (np.asarray(ask['posture']) - s['q']) - POSTURE_KD[1] * v[act]
    T, E, t0, a6, asked = _maps(b, s, ask, want)
    if k3:
        E, a6 = np.hstack([E, np.zeros((n, k3))]), np.concatenate([a6, np.zeros(k3)])
    # the base's rows of her dynamics: A base = Gm u - M_b E u - d
    Gm = np.zeros((6, nu))
    Gm[:, :6] = feet[0]['J6'][:, :6].T * stance[0]
    Gm[:, 6:12] = feet[1]['J6'][:, :6].T * stance[1]
    for i, c in enumerate(contacts):
        Gm[:, NU + 3 * i:NU + 3 * i + 3] = c['J'][:, :6].T
    Ai = np.linalg.inv(M[:6] @ T + 1e-9 * _EYE6)
    P, p0 = Ai @ (Gm - M[:6] @ E), -Ai @ (M[:6] @ t0 + h[:6])
    full, full0 = T @ P + E, T @ p0 + t0
    rows, rhs, rhs0 = [], [], []

    def lay(R, c, weight, c0=None):
        """A row = `c`, weighed; `c0` what it is with nothing asked, every acceleration 0."""
        c, w = np.atleast_1d(c), np.asarray(weight, float)
        rows.append((w[:, None] if w.ndim else w) * (R if R.ndim == 2 else R[None, :]))
        rhs.append(w * c)
        rhs0.append(w * np.broadcast_to(np.asarray(0.0 if c0 is None else c0, float), c.shape))

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
    a = np.asarray(ask['com_acc'], float)
    lay(sumf[[0, 2]] / mass, a[[0, 2]], W_COM * grounded)
    lay(sumf[1] / mass, a[1] + G, W_HEIGHT * grounded, G)
    for t, turn in zip(s['turns'], turns):
        left = -t['dJr'] - t['Jr'] @ full0
        lay(t['Jr'] @ full, TURN_KD[0] * turn_error(turn, t['quat']) - TURN_KD[1] * t['w'] + left, W_TURN * turned,
            left)
    # a palm told to bear reaches its place on her posture's spring, its row a swinging sole's as a parry's landing is
    # (`modal.reached`): posed alone, her fists hung 0.20 m over the floor and met it as her trunk lay back past 100 deg;
    # on a sole's swing's spring, 0.3 m off it asked 480 m/s^2 and her soles let go
    for k, to in ask.get('reach', {}).items():
        hand = s['hands'][k]
        left = -hand['dJv'] - hand['J'] @ full0
        lay(hand['J'] @ full, POSTURE_KD[0] * (to - hand['p']) - POSTURE_KD[1] * hand['v'] + left, W_SWING, left)
    lay(_UPPER_ROWS if not k3 else np.eye(nu)[12:NU], want[UPPER], W_POSTURE * (1.0 + (ask.get('hold', 1.0) - 1.0) * _loose(contacts)))
    edges, mu, strides = ask.get('edge', (0.0, 0.0)), ask.get('mu', (MU, MU)), float(ask.get('strides', 0.0))
    swung = np.concatenate([np.repeat(asked, 6), np.zeros(nu - 12)])
    resting = np.asarray(ask.get('rests', (0.0, 0.0)), float) * (1.0 - np.asarray(ask.get('let_go', (False, False)), float))
    lay(np.diag(swung), a6, W_SWING + (W_STILL - W_SWING) * np.concatenate([np.repeat(resting, 6), np.zeros(nu - 12)]))
    knees, left = act[KNEES], -float(stance @ full0[act[KNEES]])
    # the knees' rows damped against the posture's rate: against still, a push's knees were held
    # at 0.2 of their clamp and her height at its band's floor
    knee_rate = np.asarray(ask.get('knee_rate', (0.0, 0.0)), float)
    lay(stance @ full[knees], float(stance @ (want[KNEES] + POSTURE_KD[1] * knee_rate)) + left, W_KNEE * grounded, left)
    for k, (knee, rate, acc) in ask.get('fold', {}).items():
        j = knees[k]
        lay(full[j], acc + SWING_KD[0] * (knee - s['q'][KNEES[k]]) + SWING_KD[1] * (rate - v[j]) - full0[j], W_FOLD,
            -full0[j])
    stops, (width, stiff, damp, reach) = b.buffers
    for i, stop, side in stops:
        j, depth = act[i], side * (stop - s['q'][i]) + width
        lay(full[j], stiff * side * max(depth, 0.0) - damp * v[j] - full0[j],
            W_STOP * np.clip((depth + reach * max(-side * v[j], 0.0)) / width, 0.0, 1.0), -full0[j])
    kdot = ask.get('kdot')
    lay(about, -MOMENTUM_K * s['k'] if kdot is None else np.asarray(kdot), W_MOMENT * grounded)
    part, inv_mg, inv_mgr = np.zeros(nu), 1.0 / (mass * G), 1.0 / (mass * G * ROOM[0])
    part[:12] = (inv_mgr, inv_mgr, inv_mgr, inv_mg, inv_mg, inv_mg) * 2
    if k3:
        part[NU:] = inv_mg
    spare = W_PART * np.concatenate([each, np.zeros(NU - 12), np.ones(k3)]) * part + W_AIR * np.concatenate([1.0 - each, np.zeros(nu - 12)])
    lay(np.diag(spare), np.zeros(nu), 1.0)
    R0, up = np.vstack(rows), sumf[1] / mass
    H0 = R0.T @ R0 + 1e-12 * (_EYE_NU if not k3 else np.eye(nu))
    frames, moment = [f['R'] for f in feet], (MOMENT if not k3 else np.concatenate([MOMENT, np.zeros(k3)]))
    keep = W_KEEP * np.concatenate([each, np.zeros(nu - 12)]) * part
    kinetic = (full.T @ M @ full + np.diag((W_SWING * swung + spare) ** 2 + keep ** 2)) if strides > 0.0 else None
    geo, stride = (nu, k3, moment, part, up, sumf[1], mass, grounded, stance, frames, edges, mu, caps, slips), (strides, kinetic)
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

    limit, lo, hi = box(b, ask)
    A = (M @ full)[act]
    for i, c in enumerate(contacts):
        A[:, NU + 3 * i:NU + 3 * i + 3] -= c['J'][:, act].T
    for k, f in enumerate(feet):
        A[:, 6 * k:6 * k + 6] -= stance[k] * f['J6'][:, act].T
    g0, held = R0.T @ np.concatenate(rhs), [(S, r) for S, r, _r0 in still]
    u, tau, carried, qdd, scale = _pinned(
        lambda dH=None, dg=None: _rounds(H0 if dH is None else H0 + dH, g0 if dg is None else g0 + dg, held, felt,
                                         a[1] - HEIGHT_BAND, a[1] + HEIGHT_BAND, geo, stride),
        torques, A, (M @ full0 + h)[act], limit)
    bears, cop = [0.0, 0.0], []
    for k, f in enumerate(feet):
        wk = stance[k] * u[6 * k:6 * k + 6]
        bears[k] = float(wk[4])
        if ask['stance'][k]:
            ml = f['R'].T @ wk[:3]
            cop.append(np.array([ml[2], -ml[0]]) / wk[4] if wk[4] > 1e-6 else None)
    # the drives asked past their clamps and their loads: the ask's, before any was pinned
    past = np.abs(tau) > b.clamp * (1.0 + 1e-6)
    over = past.copy()
    for p, r in b.pairs:
        over[p] = over[r] = past[p] or past[r]
    b.load = np.abs(tau) / b.clamp
    return {'tau': np.clip(carried, lo, hi), 'qacc': qdd, 'bears': bears, 'cop': cop, 'over': over,
            'slack': [], 'held': (), 'borne': u[NU:].reshape(-1, 3), 'scale': scale}

