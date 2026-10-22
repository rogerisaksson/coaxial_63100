"""Her whole body's step without an active set: a fixed run of small solves, masks and clips.

    out = wrench.step(body, s, ask)         # `wbc.step`'s ask in, its {tau, qacc, bears, ..} out

Twenty-one unknowns: six a sole - the wrench it takes of the floor where it stands, its own
acceleration where it does not - and her upper body's nine accelerations; and three a contact
(`s['contacts']`, `machine.borne`): the force on a segment that bears her, its point still. Every joint follows
from them: a leg through its sole (still where it stands), the pelvis by the six base rows of her
dynamics. One weighted least squares finds them - her centre of mass's ask, the turns, a swinging
sole's ask, the posture, her standing knees, her height, her angular momentum - ROUNDS times,
each wrench clipped into its sole's bounds and what was clipped pinned for the next. Striding,
what the floor cannot give is not traded among the rows again: her inertia takes it (`_taken`).
An ask her drives cannot carry is shrunk as one toward asking nothing - the same rows solved
again with every acceleration 0 and her weight borne - as far as brings every torque within its
clamp (`shrunk`); nothing is clipped apart. `wbc.Body` and `wbc.sense` as the stack has them;
wbc/src/wbc_wrench.c is it in C, row for row (docs/findings/wrench.md).
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import drives
from machine.figure import BALL, HEEL, JOINTS, SOLE_HALF
from machine.wbc import (CONTACT_K, HEIGHT_BAND, HELD_K, MOMENTUM_K, MU, POSTURE_KD, SWING_KD,
                         TURN_KD, G, turn_error)

_DRIVEN = [j for j in JOINTS if drives.passive(j) is None]
#: Among the driven joints: each leg's six, hip to sole; each knee; the rest, her upper body's.
LEGS = [[_DRIVEN.index(side + k) for k in ('hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll')]
        for side in ('left_', 'right_')]
KNEES = [leg[3] for leg in LEGS]
UPPER = [i for i in range(len(_DRIVEN)) if i not in LEGS[0] + LEGS[1]]
NU = 12 + len(UPPER)
#: Which of the unknowns are a sole's moment's parts.
MOMENT = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0] * 2 + [0.0] * len(UPPER))

#: The centre of pressure's room in a sole, across and along, m: its edges less EDGE_M; a
#: sole's yaw moment's arm, m.
EDGE_M, YAW_M = 0.003, 0.04
ROOM = np.array([SOLE_HALF - EDGE_M, (BALL + HEEL) / 2.0 - EDGE_M])
#: Rounds of clipping; a leg's solve's damping - at 1e-3 a swung leg straight at the knee was
#: asked 17 000 rad/s^2, at 0.05 she left the floor 8 mm where 20; a knee nearer straight than
#: KNEE_MIN rad is sprung back.
ROUNDS, DAMP, KNEE_MIN = 4, 0.02, 0.07
#: Rounds more, at most, for an answer past her height's band after them.
PAST = 2
#: The rows' weights. Her centre of mass along the floor, m/s^2, all but exact; the turns and a
#: swinging sole, rad/s^2 and m/s^2 - the sole at a third of the turns the walk fell at its 7th
#: step -; her standing knees' posture, rad/s^2 - at 1, 60 N felled her 2 times of 8 -; the
#: posture, her height, her angular momentum (N m), a swinging knee's fold, a knee's stop; a
#: wrench's parts in shares of her weight, a sole's acceleration left alone, a pinned part.
W_COM, W_TURN, W_SWING, W_KNEE, W_POSTURE, W_HEIGHT, W_MOMENT = 1000.0, 30.0, 30.0, 3.0, 1.0, 1.0, 0.3
W_FOLD, W_STOP, W_PART, W_AIR, W_PIN = 3.0, 30.0, 0.1, 1e-3, 1e5
#: Striding, a standing sole's wrench kept in the projection's metric, of its share of her weight:
#: her soles pushed against each other change her motion by nothing, and unkept the faces were
#: met through it - 926 N along one sole, -906 along the other - till a sole pulled and both
#: were let go, 11 times in 12 s of her walk. At 10 none and her walk 418 J/m, 2.53 off a
#: woman's (412, 2.61); at 30 2.84; at 100 let go 187 times.
W_KEEP = 3.0
#: A contact's point held still, m/s^2, while it presses - one the step would have pull is let go,
#: its point free: held still at nothing, a seat she rose from asked 5134 N of a sole; its push no
#: more than GROW_N over what it bears now: let to plan 311 N where a mattress's soft edge bore
#: 107, the step's push behind her centre of mass never came and she rolled back onto the bed.
W_STILL, GROW_N = 1e4, 30.0
#: A solve with no answer of its own is solved again with this share of its largest diagonal on
#: its free unknowns: falling on a ramp, two bearing points 16 mm apart left the rounds' matrix
#: rank 21 of 27 (cond 2.9e19), their split said by no row, and the solve raised.
RIDGE = 1e-12


_EYE3, _EYE6, _EYE_NU = np.eye(3), np.eye(6), np.eye(NU)
_DAMP_EYE6 = DAMP ** 2 * _EYE6
_UPPER_IDX = 12 + np.arange(len(UPPER))
_UPPER_ROWS = _EYE_NU[12:NU]
_GIVES = 1e-9 * (np.arange(13) == 12)
#: Her arms' rows `ask['hold']` times the posture's, else the step pitched her trunk by them: shoulders
#: 130 deg at 939 deg/s sitting, 255 at 1 633 lying back; hands' jerk 15-21 000 m/s^3 seated, 1 400-1 800
#: walking. Not kept: all upper drives at W_TURN (4 800-6 700, 900; pelvis rolled 3 deg, a woman's 5;
#: walk 2.75 off, 2.55); the arms alone (a crouch's spine folded, fell).
ARMED = np.array([float(_DRIVEN[i].endswith(('shoulder', 'elbow'))) for i in UPPER])


def _inv(A):
    """A's damped inverse."""
    return np.linalg.solve(A.T @ A + (_DAMP_EYE6 if A.shape[1] == 6 else DAMP ** 2 * np.eye(A.shape[1])), A.T)


def _solved(K, rhs, free):
    """K^-1 rhs, K ridged by RIDGE on its `free` unknowns where it has no inverse."""
    try:
        return np.linalg.solve(K, rhs)
    except np.linalg.LinAlgError:
        return np.linalg.solve(K + np.diag(free * RIDGE * float(np.diag(K).max())), rhs)


def bounded(u, frames, stance, edge=(0.0, 0.0), mu=(MU, MU)):
    """`u` with each standing sole's wrench (moment, force; world) inside its bounds - pressing,
    in its friction's pyramid (`mu` each's), its centre of pressure in its room -, and which
    parts moved. A sole on an `edge` - 1 its ball, -1 its heel, a share between - has its centre
    of pressure that far to it."""
    out = u.copy()
    for k, R in enumerate(frames):
        uk = u[6 * k:6 * k + 6]
        m = R.T @ uk[:3]
        fx, fy, fz = float(uk[3]), max(float(uk[4]), 0.0), float(uk[5])
        ek, muk = float(edge[k]), float(mu[k])
        c0, c1, c2, mf = ROOM[1] * fy, muk * YAW_M * fy, ROOM[0] * fy, muk * fy
        lo0 = -c0 * (1.0 - 2.0 * max(-ek, 0.0))
        hi0 = c0 * (1.0 - 2.0 * max(ek, 0.0))
        mc = np.array([min(max(float(m[0]), lo0), hi0),
                       min(max(float(m[1]), -c1), c1),
                       min(max(float(m[2]), -c2), c2)])
        inside = np.empty(6)
        inside[:3] = R @ mc
        inside[3] = min(max(fx, -mf), mf)
        inside[4] = fy
        inside[5] = min(max(fz, -mf), mf)
        out[6 * k:6 * k + 6] += stance[k] * (inside - uk)
    return out, (np.abs(out - u) > 1e-9).astype(float)


def pushed(u, bears):
    """`u` with each contact's force (world, after the soles' and her upper body's unknowns)
    pressing, no more than GROW_N over what it `bears` now (N up, one a contact), and in its
    friction's pyramid, and which parts moved."""
    k = len(bears)
    if not k:
        return u.copy(), np.zeros(len(u))
    out = u.copy()
    f = out[NU:NU + 3 * k].reshape(k, 3)
    f[:, 1] = np.clip(f[:, 1], 0.0, np.asarray(bears, float) + GROW_N)
    f[:, [0, 2]] = np.clip(f[:, [0, 2]], -MU * f[:, 1:2], MU * f[:, 1:2])
    out[NU:NU + 3 * k] = f.ravel()
    return out, (np.abs(out - u) > 1e-9).astype(float)


def faced(u, frames, stance, edge=(0.0, 0.0), mu=(MU, MU)):
    """`u` with each standing sole's wrench in its cone, as `bounded`; the faces it was past -
    six a sole, its moment about its own axes (pitch, yaw, roll), then its force - and each
    face's share of the sole's push: no part has a bound of its own."""
    out, past, share = u.copy(), np.zeros(12), np.zeros(12)
    for k, R in enumerate(frames):
        was = np.empty(6)
        was[:3], was[3:] = R.T @ u[6 * k:6 * k + 3], u[6 * k + 3:6 * k + 6]
        fy = max(float(was[4]), 0.0)
        ek, muk = float(edge[k]), float(mu[k])
        low = -np.array([ROOM[1] * (1.0 - 2.0 * max(-ek, 0.0)), muk * YAW_M, ROOM[0], muk, 0.0, muk])
        high = np.array([ROOM[1] * (1.0 - 2.0 * max(ek, 0.0)), muk * YAW_M, ROOM[0], muk, 1.0, muk])
        lfy, hfy = low * fy, high * fy
        inside = np.clip(was, lfy, hfy)
        past[6 * k:6 * k + 6] = stance[k] * (np.abs(inside - was) > 1e-9)
        share[6 * k:6 * k + 6] = np.where(was < lfy, low, high)
        w = np.empty(6)
        w[:3], w[3:] = R @ inside[:3], inside[3:]
        out[6 * k:6 * k + 6] += stance[k] * (w - u[6 * k:6 * k + 6])
    return out, past, share


def _taken(u0, metric, up, frames, stance, edge, band, grounded, mu):
    """Striding, what the floor cannot give her inertia takes: `u0`, the rows' answer with the
    floor unbounded, brought to the faces her soles' wrenches are past by the least change in
    `metric` - her kinetic energy's, a swinging sole's ask held. Traded among the rows, a row
    a joint, her head was the cheapest moment about her and nodded 46 deg a stride; her
    centre of mass held as well, her trunk pitched 9 to 10 deg, or she was down at 12 s. A
    face is a part of a sole's wrench its share of the sole's push, about the sole's own
    axes; her height's `band` is decided before the faces - after them a throw's soles,
    asked to pull by her knees' rows, pressed nothing - and gives where a cone and it cannot
    both be met."""
    Gi = np.linalg.inv(metric)
    F, N = np.zeros((13, u0.size)), np.zeros((13, u0.size))
    for k, R in enumerate(frames):
        F[6 * k:6 * k + 3, 6 * k:6 * k + 3], F[6 * k + 3:6 * k + 6, 6 * k + 3:6 * k + 6] = R.T, _EYE3
        N[6 * k:6 * k + 6, 6 * k + 4] = 1.0
    F[12] = up
    met, share, level, gives = np.zeros(13), np.zeros(13), np.zeros(13), _GIVES
    u = at = u0
    for _ in range(ROUNDS):
        ay = float(up @ u) - G
        past = min(max(ay, band[0]), band[1]) + G
        level[12] += (1.0 - met[12]) * (past - level[12])
        met[12] = max(met[12], grounded * float(abs(past - G - ay) > 1e-9))
        C = met[:, None] * (F - share[:, None] * N)
        u = u0 - Gi @ C.T @ np.linalg.solve(C @ Gi @ C.T + np.diag(1.0 - met + gives), C @ u0 - met * level)
        at, out, to = faced(u, frames, stance, edge, mu)
        share[:12] += out * (to - share[:12])
        met[:12] = np.maximum(met[:12], out)
    return at


def shrunk(tau, tau0, limit):
    """How far from `tau0`, the drives' torques with nothing asked, toward `tau`, the ask's,
    every drive stays within its `limit`: the largest s in [0, 1] with every |tau0 + s (tau -
    tau0)| <= limit, each drive's in closed form, the least of them; 0 where none is."""
    t0 = np.where(np.abs(tau0) <= limit, tau0, 0.0)
    d = tau - t0
    with np.errstate(divide='ignore', invalid='ignore'):
        leaves = np.where(d > 0.0, limit - t0, -limit - t0) / d
        enters = np.where(d > 0.0, -limit - t0, limit - t0) / d
    s = np.where(d == 0.0, 1.0 * (np.abs(tau) <= limit), np.minimum(1.0, leaves))
    s = np.where(s >= np.maximum(0.0, np.where(d == 0.0, 0.0, enters)), s, 0.0)
    return float(np.clip(s.min(), 0.0, 1.0))


def _maps(b, s, ask, want):
    """(T, E, t0, a6, asked): every acceleration of the pelvis's six and the unknowns, qdd = T base
    + E u + t0; each swinging sole's asked acceleration and whether it is asked. A standing
    sole is still, or goes as `ask['settle']` has it (a sole's turn and its middle's
    acceleration, world: flat again from its heel, over its ball)."""
    n, M, h, v, act = b.n, s['M'], s['h'], s['v'], b.act
    T, E, t0 = np.zeros((n, 6)), np.zeros((n, NU)), np.zeros(n)
    T[:6] = _EYE6
    t0[b.held] = -HELD_K * v[b.held]
    E[act[UPPER], _UPPER_IDX] = 1.0
    a6, asked, swing = np.zeros(NU), np.zeros(2), ask.get('swing', {})
    settle = ask.get('settle', np.zeros((2, 6)))
    for k, leg in enumerate(LEGS):
        f, cols = s['feet'][k], act[leg]
        twist = f['J6'] @ v
        if not ask['stance'][k] and k not in swing:
            t0[cols] = want[leg]
            continue
        inv = _inv(f['J6'][:, cols])
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
    # the joints no drive holds, by their own rows of her dynamics under all the rest
    passive = getattr(b, 'passive', None)
    if passive is None:
        passive = np.array([i >= 6 and i in b.free for i in range(n)])
        active = ~passive
    else:
        active = b.active_dof
    m_pp = M[np.ix_(passive, passive)]
    pull = -np.linalg.solve(m_pp, M[np.ix_(passive, active)])
    T[passive], E[passive] = pull @ T[active], pull @ E[active]
    t0[passive] = pull @ t0[active] - np.linalg.solve(m_pp, h[passive])
    return T, E, t0, a6, asked


def _rounds(H0, g0, held, bears, lo, hi, geo, stride):
    """The unknowns for the rows' normal matrix `H0` and right-hand side `g0` (R^T c), the
    contacts' `held` rows pressing on what they bear (`bears`, N up), her height's band `lo` to
    `hi`: the rounds, then the stride. `geo`: (nu, k3, moment, part, up, lift, mass, grounded,
    stance, frames, edges, mu); `stride`: (strides, her kinetic energy's metric)."""
    nu, k3, moment, part, up, lift, mass, grounded, stance, frames, edges, mu = geo
    strides, kinetic = stride
    pinned, at, band, edge = np.zeros(nu), np.zeros(nu), 0.0, 0.0
    pressed, done, rounds = np.ones(len(bears)), 0, ROUNDS
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
        H, free = Hr + np.diag(soft), 1.0 - hard
        K = H * np.outer(free, free) + np.diag(hard)
        u, way = _solved(K, np.array([free * (gr + soft * at - H @ (hard * at)) + hard * at, up]).T, free).T
        u = u + band * (edge - up @ u) / (up @ way + 1.0 - band) * way
        # her height's band: past an edge, held to it
        ay = float(lift @ u) / mass - G
        past = min(max(ay, lo), hi) + G
        band, edge = max(band, grounded * float(abs(past - G - ay) > 1e-9)), edge + (1.0 - band) * (past - edge)
        at, moved = bounded(u, frames, stance, edges, mu)
        if k3:
            at, kept = pushed(at, bears)
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
        return at + strides * (_taken(_solved(Hr, gr, np.ones(nu)), kinetic, up, frames, stance, edges, (lo, hi),
                                      grounded, mu) - at)
    return at


def step(b, s, ask):
    """{tau, qacc, bears, cop, over, slack, held, scale}: `ask` as `wbc.step` takes it, and
    `strides`, how far she strides (0 with none): her balance her soles' landings' then, not her
    rows'; `scale` how much of the ask her drives carry, 1 whole."""
    n, M, h, v, feet, act, mass = b.n, s['M'], s['h'], s['v'], s['feet'], b.act, b.mass
    contacts = s.get('contacts', ())
    k3, nu = 3 * len(contacts), NU + 3 * len(contacts)
    stance = np.array([float(bool(k)) for k in ask['stance']])
    each, grounded = np.repeat(stance, 6), max(float(stance.max()), float(k3 > 0))
    turns = ask.get('turns', ())
    turned = max(grounded, float(ask.get('borne', 0.0))) * float(len(turns) > 0)
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

    sumf, about, still = np.zeros((3, nu)), np.zeros((3, nu)), []
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
        left = -c['dJv'] - c['J'] @ full0
        still.append((W_STILL * (c['J'] @ full), W_STILL * (left - CONTACT_K * (c['J'] @ v)), W_STILL * left))
    a = np.asarray(ask['com_acc'], float)
    lay(sumf[[0, 2]] / mass, a[[0, 2]], W_COM * grounded)
    lay(sumf[1] / mass, a[1] + G, W_HEIGHT * grounded, G)
    for t, turn in zip(s['turns'], turns):
        left = -t['dJr'] - t['Jr'] @ full0
        lay(t['Jr'] @ full, TURN_KD[0] * turn_error(turn, t['quat']) - TURN_KD[1] * t['w'] + left, W_TURN * turned,
            left)
    lay(_UPPER_ROWS if not k3 else np.eye(nu)[12:NU], want[UPPER], W_POSTURE * (1.0 + (ask.get('hold', 1.0) - 1.0) * ARMED))
    edges, mu, strides = ask.get('edge', (0.0, 0.0)), ask.get('mu', (MU, MU)), float(ask.get('strides', 0.0))
    swung = np.concatenate([np.repeat(asked, 6), np.zeros(nu - 12)])
    lay(np.diag(swung), a6, W_SWING + (W_STILL - W_SWING) * np.concatenate([np.repeat(ask.get('rests', (0.0, 0.0)), 6), np.zeros(nu - 12)]))
    knees = act[KNEES]
    left = -float(stance @ full0[knees])
    # the knees' rows damped against the posture's rate: against still, a push's knees were held
    # at 0.2 of their clamp and her height at its band's floor
    knee_rate = np.asarray(ask.get('knee_rate', (0.0, 0.0)), float)
    lay(stance @ full[knees], float(stance @ (want[KNEES] + POSTURE_KD[1] * knee_rate)) + left, W_KNEE * grounded, left)
    for k, (knee, rate, acc) in ask.get('fold', {}).items():
        j = knees[k]
        lay(full[j], acc + SWING_KD[0] * (knee - s['q'][KNEES[k]]) + SWING_KD[1] * (rate - v[j]) - full0[j], W_FOLD,
            -full0[j])
    for k, j in enumerate(knees):
        short = KNEE_MIN - s['q'][KNEES[k]]
        lay(full[j], POSTURE_KD[0] * short - POSTURE_KD[1] * v[j] - full0[j],
            W_STOP * min(max(float(short / KNEE_MIN), 0.0), 1.0), -full0[j])
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
    geo, stride = (nu, k3, moment, part, up, sumf[1], mass, grounded, stance, frames, edges, mu), (strides, kinetic)
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

    # the ask's answer, and nothing's: every acceleration 0, her weight borne, by the same rows
    whole = _rounds(H0, R0.T @ np.concatenate(rhs), [(S, r) for S, r, _r0 in still], felt,
                    a[1] - HEIGHT_BAND, a[1] + HEIGHT_BAND, geo, stride)
    limit = (b.wep if ask.get('wep') else b.clamp) * np.asarray(ask.get('derate', 1.0))
    tau, qdd = torques(whole)
    if (np.abs(tau) > limit).any():
        stood = _rounds(H0, R0.T @ np.concatenate(rhs0), [(S, r0) for S, _r, r0 in still], felt,
                        -HEIGHT_BAND, HEIGHT_BAND, geo, stride)
        tau0 = torques(stood)[0]
        scale = shrunk(tau, tau0, limit)
        u = whole + (scale - 1.0) * (whole - stood)
        s0 = (shrunk(tau + (scale - 1.0) * (tau - tau0), np.zeros(len(limit)), limit)
              if (np.abs(tau0) > limit).any() else 1.0)
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
    past = np.abs(tau) > b.clamp * (1.0 + 1e-6)
    over = past.copy()
    for p, r in b.pairs:
        over[p] = over[r] = past[p] or past[r]
    b.load = np.abs(tau) / b.clamp
    return {'tau': np.clip(carried, -limit, limit), 'qacc': qdd, 'bears': bears, 'cop': cop, 'over': over,
            'slack': [], 'held': (), 'borne': u[NU:].reshape(-1, 3), 'scale': scale}

