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
The drives' clamps clip the torques after it. `wbc.Body` and `wbc.sense` as the stack has them;
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


def _inv(A):
    """A's damped inverse."""
    return np.linalg.solve(A.T @ A + DAMP ** 2 * np.eye(A.shape[1]), A.T)


def bounded(u, frames, stance, edge=(0.0, 0.0)):
    """`u` with each standing sole's wrench (moment, force; world) inside its bounds - pressing,
    in its friction's pyramid, its centre of pressure in its room -, and which parts moved. A
    sole on an `edge` - 1 its ball, -1 its heel, a share between - has its centre of pressure
    that far to it."""
    out = u.copy()
    for k, R in enumerate(frames):
        m, f = R.T @ u[6 * k:6 * k + 3], u[6 * k + 3:6 * k + 6]
        fy = max(f[1], 0.0)
        cap = np.array([ROOM[1] * fy, MU * YAW_M * fy, ROOM[0] * fy])
        low = -cap * np.array([1.0 - 2.0 * max(-edge[k], 0.0), 1.0, 1.0])
        cap = cap * np.array([1.0 - 2.0 * max(edge[k], 0.0), 1.0, 1.0])
        inside = np.concatenate([R @ np.clip(m, low, cap),
                                 [np.clip(f[0], -MU * fy, MU * fy), fy, np.clip(f[2], -MU * fy, MU * fy)]])
        out[6 * k:6 * k + 6] += stance[k] * (inside - u[6 * k:6 * k + 6])
    return out, (np.abs(out - u) > 1e-9).astype(float)


def pushed(u, bears):
    """`u` with each contact's force (world, after the soles' and her upper body's unknowns)
    pressing, no more than GROW_N over what it `bears` now (N up, one a contact), and in its
    friction's pyramid, and which parts moved."""
    out, k = u.copy(), len(bears)
    f = out[NU:NU + 3 * k].reshape(k, 3)
    f[:, 1] = np.clip(f[:, 1], 0.0, np.asarray(bears, float) + GROW_N)
    f[:, [0, 2]] = np.clip(f[:, [0, 2]], -MU * f[:, 1:2], MU * f[:, 1:2])
    out[NU:NU + 3 * k] = f.ravel()
    return out, (np.abs(out - u) > 1e-9).astype(float)


def faced(u, frames, stance, edge=(0.0, 0.0)):
    """`u` with each standing sole's wrench in its cone, as `bounded`; the faces it was past -
    six a sole, its moment about its own axes (pitch, yaw, roll), then its force - and each
    face's share of the sole's push: no part has a bound of its own."""
    out, past, share = u.copy(), np.zeros(12), np.zeros(12)
    for k, R in enumerate(frames):
        was = np.concatenate([R.T @ u[6 * k:6 * k + 3], u[6 * k + 3:6 * k + 6]])
        fy = max(was[4], 0.0)
        low = -np.array([ROOM[1] * (1.0 - 2.0 * max(-edge[k], 0.0)), MU * YAW_M, ROOM[0], MU, 0.0, MU])
        high = np.array([ROOM[1] * (1.0 - 2.0 * max(edge[k], 0.0)), MU * YAW_M, ROOM[0], MU, 1.0, MU])
        inside = np.clip(was, low * fy, high * fy)
        past[6 * k:6 * k + 6] = stance[k] * (np.abs(inside - was) > 1e-9)
        share[6 * k:6 * k + 6] = np.where(was < low * fy, low, high)
        out[6 * k:6 * k + 6] += stance[k] * (np.concatenate([R @ inside[:3], inside[3:]]) - u[6 * k:6 * k + 6])
    return out, past, share


def _taken(u0, metric, up, frames, stance, edge, band, grounded):
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
        F[6 * k:6 * k + 3, 6 * k:6 * k + 3], F[6 * k + 3:6 * k + 6, 6 * k + 3:6 * k + 6] = R.T, np.eye(3)
        N[6 * k:6 * k + 6, 6 * k + 4] = 1.0
    F[12] = up
    met, share, level, gives = np.zeros(13), np.zeros(13), np.zeros(13), 1e-9 * (np.arange(13) == 12)
    u = at = u0
    for _ in range(ROUNDS):
        ay = float(up @ u) - G
        past = float(np.clip(ay, band[0], band[1])) + G
        level[12] += (1.0 - met[12]) * (past - level[12])
        met[12] = max(met[12], grounded * float(abs(past - G - ay) > 1e-9))
        C = met[:, None] * (F - share[:, None] * N)
        u = u0 - Gi @ C.T @ np.linalg.solve(C @ Gi @ C.T + np.diag(1.0 - met + gives), C @ u0 - met * level)
        at, out, to = faced(u, frames, stance, edge)
        share[:12] += out * (to - share[:12])
        met[:12] = np.maximum(met[:12], out)
    return at


def _maps(b, s, ask, want):
    """(T, E, t0, a6, asked): every acceleration of the pelvis's six and the unknowns, qdd = T base
    + E u + t0; each swinging sole's asked acceleration and whether it is asked. A standing
    sole is still, or goes as `ask['settle']` has it (a sole's turn and its middle's
    acceleration, world: flat again from its heel, over its ball)."""
    n, M, h, v, act = b.n, s['M'], s['h'], s['v'], b.act
    T, E, t0 = np.zeros((n, 6)), np.zeros((n, NU)), np.zeros(n)
    T[:6] = np.eye(6)
    t0[b.held] = -HELD_K * v[b.held]
    E[act[UPPER], 12 + np.arange(len(UPPER))] = 1.0
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
        a6[6 * k:6 * k + 6] = np.concatenate([
            TURN_KD[0] * turn_error(quat, f['quat']) - TURN_KD[1] * twist[:3],
            np.asarray(acc) + SWING_KD[0] * (np.asarray(at) - f['sole'])
            + SWING_KD[1] * (np.asarray(speed) - twist[3:])])
        asked[k] = 1.0
        t0[cols] = -inv @ f['dJv']
        E[cols, 6 * k:6 * k + 6] = inv
    # the joints no drive holds, by their own rows of her dynamics under all the rest
    passive = np.array([i >= 6 and i in b.free for i in range(n)])
    pull = -np.linalg.solve(M[np.ix_(passive, passive)], M[np.ix_(passive, ~passive)])
    T[passive], E[passive] = pull @ T[~passive], pull @ E[~passive]
    t0[passive] = pull @ t0[~passive] - np.linalg.solve(M[np.ix_(passive, passive)], h[passive])
    return T, E, t0, a6, asked


def step(b, s, ask):
    """{tau, qacc, bears, cop, over, slack, held}: `ask` as `wbc.step` takes it, and `strides`,
    how far she strides (0 with none): her balance her soles' landings' then, not her rows'."""
    n, M, h, v, feet, act, mass = b.n, s['M'], s['h'], s['v'], s['feet'], b.act, b.mass
    contacts = s.get('contacts', ())
    k3 = 3 * len(contacts)
    nu = NU + k3
    stance = np.array([float(bool(k)) for k in ask['stance']])
    each = np.repeat(stance, 6)
    grounded = max(float(stance.max()), float(k3 > 0))
    turns = ask.get('turns', ())
    turned = max(grounded, float(ask.get('borne', 0.0))) * float(len(turns) > 0)
    want = POSTURE_KD[0] * (np.asarray(ask['posture']) - s['q']) - POSTURE_KD[1] * v[act]
    T, E, t0, a6, asked = _maps(b, s, ask, want)
    E, a6 = np.hstack([E, np.zeros((n, k3))]), np.concatenate([a6, np.zeros(k3)])
    # the base's rows of her dynamics: A base = Gm u - M_b E u - d
    Gm = np.zeros((6, nu))
    Gm[:, :12] = np.hstack([f['J6'][:, :6].T for f in feet]) * each
    for i, c in enumerate(contacts):
        Gm[:, NU + 3 * i:NU + 3 * i + 3] = c['J'][:, :6].T
    Ai = np.linalg.inv(M[:6] @ T + 1e-9 * np.eye(6))
    P, p0 = Ai @ (Gm - M[:6] @ E), -Ai @ (M[:6] @ t0 + h[:6])
    full, full0 = T @ P + E, T @ p0 + t0
    rows, rhs = [], []

    def lay(R, c, weight):
        rows.append(weight * np.atleast_2d(R))
        rhs.append(weight * np.atleast_1d(c))

    sumf, about, still = np.zeros((3, nu)), np.zeros((3, nu)), []
    for k, f in enumerate(feet):
        r = f['sole'] - s['com']
        sumf[:, 6 * k + 3:6 * k + 6] = stance[k] * np.eye(3)
        about[:, 6 * k:6 * k + 3] = stance[k] * np.eye(3)
        about[:, 6 * k + 3:6 * k + 6] = stance[k] * np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]],
                                                              [-r[1], r[0], 0.0]])
    for i, c in enumerate(contacts):
        r, at = c['p'] - s['com'], slice(NU + 3 * i, NU + 3 * i + 3)
        sumf[:, at] = np.eye(3)
        about[:, at] = np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]])
        still.append((W_STILL * (c['J'] @ full), W_STILL * (-c['dJv'] - CONTACT_K * (c['J'] @ v) - c['J'] @ full0)))
    a = np.asarray(ask['com_acc'], float)
    lay(sumf[[0, 2]] / mass, a[[0, 2]], W_COM * grounded)
    lay(sumf[1] / mass, a[1] + G, W_HEIGHT * grounded)
    for t, turn in zip(s['turns'], turns):
        lay(t['Jr'] @ full, TURN_KD[0] * turn_error(turn, t['quat']) - TURN_KD[1] * t['w'] - t['dJr']
            - t['Jr'] @ full0, W_TURN * turned)
    lay(np.eye(nu)[12:NU], want[UPPER], W_POSTURE)
    swung = np.concatenate([np.repeat(asked, 6), np.zeros(nu - 12)])
    lay(np.diag(swung), a6, W_SWING)
    knees = act[KNEES]
    lay(stance @ full[knees], float(stance @ want[KNEES]) - float(stance @ full0[knees]), W_KNEE * grounded)
    for k, (knee, rate, acc) in ask.get('fold', {}).items():
        j = knees[k]
        lay(full[j], acc + SWING_KD[0] * (knee - s['q'][KNEES[k]]) + SWING_KD[1] * (rate - v[j]) - full0[j], W_FOLD)
    for k, j in enumerate(knees):
        short = KNEE_MIN - s['q'][KNEES[k]]
        lay(full[j], POSTURE_KD[0] * short - POSTURE_KD[1] * v[j] - full0[j],
            W_STOP * float(np.clip(short / KNEE_MIN, 0.0, 1.0)))
    kdot = ask.get('kdot')
    lay(about, -MOMENTUM_K * s['k'] if kdot is None else np.asarray(kdot), W_MOMENT * grounded)
    part = np.zeros(nu)
    part[:12] = np.tile(np.concatenate([np.full(3, 1.0 / (mass * G * ROOM[0])), np.full(3, 1.0 / (mass * G))]), 2)
    part[NU:] = 1.0 / (mass * G)
    spare = W_PART * np.concatenate([each, np.zeros(NU - 12), np.ones(k3)]) * part + W_AIR * np.concatenate(
        [1.0 - each, np.zeros(nu - 12)])
    lay(np.diag(spare), np.zeros(nu), 1.0)
    R0, c0 = np.vstack(rows), np.concatenate(rhs)
    H0, g0, up = R0.T @ R0 + 1e-12 * np.eye(nu), R0.T @ c0, sumf[1] / mass
    frames, moment = [f['R'] for f in feet], np.concatenate([MOMENT, np.zeros(k3)])
    pinned, at, band, edge = np.zeros(nu), np.zeros(nu), 0.0, 0.0
    lo, hi = a[1] - HEIGHT_BAND, a[1] + HEIGHT_BAND
    rows0, pressed = (H0, g0), np.ones(len(contacts))
    for _ in range(ROUNDS):
        H0 = rows0[0] + sum(p * S.T @ S for p, (S, _r) in zip(pressed, still))
        g0 = rows0[1] + sum(p * S.T @ r for p, (S, r) in zip(pressed, still))
        # a clipped force is held by a row, W_PIN its weight's share; a clipped moment is its
        # value and her height at its band's edge is met, the rest solved round them: as rows
        # of W_PIN they were 1e7 on the diagonal, and what her soles share came 0.6 N m apart
        # from one solve to another
        hard, soft = pinned * moment, (W_PIN * pinned * (1.0 - moment) * part) ** 2
        H, free = H0 + np.diag(soft), 1.0 - hard
        K = H * np.outer(free, free) + np.diag(hard)
        u, way = np.linalg.solve(K, np.array([free * (g0 + soft * at - H @ (hard * at)) + hard * at, up]).T).T
        u = u + band * (edge - up @ u) / (up @ way + 1.0 - band) * way
        # her height's band: past an edge, held to it
        ay = float(sumf[1] @ u) / mass - G
        past = float(np.clip(ay, lo, hi)) + G
        band, edge = max(band, grounded * float(abs(past - G - ay) > 1e-9)), edge + (1.0 - band) * (past - edge)
        at, moved = bounded(u, frames, stance, ask.get('edge', (0.0, 0.0)))
        at, held = pushed(at, [c['f'][1] for c in contacts])
        pressed = pressed * (u[NU + 1:NU + k3:3] >= 0.0)
        pinned = np.maximum(pinned, np.maximum(moved, held))
    u, strides = at, float(ask.get('strides', 0.0))
    if strides > 0.0:
        keep = W_KEEP * np.concatenate([each, np.zeros(nu - 12)]) * part
        kinetic = full.T @ M @ full + np.diag((W_SWING * swung + spare) ** 2 + keep ** 2)
        u = u + strides * (_taken(np.linalg.solve(H0, g0), kinetic, up, frames, stance,
                                  ask.get('edge', (0.0, 0.0)), (lo, hi), grounded) - u)
    qdd = full @ u + full0
    tau = (M @ qdd + h)[act]
    for i, c in enumerate(contacts):
        tau = tau - c['J'][:, act].T @ u[NU + 3 * i:NU + 3 * i + 3]
    bears, cop = [0.0, 0.0], []
    for k, f in enumerate(feet):
        wk = stance[k] * u[6 * k:6 * k + 6]
        tau = tau - f['J6'][:, act].T @ wk
        bears[k] = float(wk[4])
        if ask['stance'][k]:
            ml = f['R'].T @ wk[:3]
            cop.append(np.array([ml[2], -ml[0]]) / wk[4] if wk[4] > 1e-6 else None)
    limit = (b.wep if ask.get('wep') else b.clamp) * np.asarray(ask.get('derate', 1.0))
    past = np.abs(tau) > b.clamp * (1.0 + 1e-6)
    over = past.copy()
    for p, r in b.pairs:
        over[p] = over[r] = past[p] or past[r]
    b.load = np.abs(tau) / b.clamp
    return {'tau': np.clip(tau, -limit, limit), 'qacc': qdd, 'bears': bears, 'cop': cop, 'over': over,
            'slack': [], 'held': (), 'borne': u[NU:].reshape(-1, 3)}
