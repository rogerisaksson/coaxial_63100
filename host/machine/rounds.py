"""The step's unknowns and its rounds' clips (wbc_rounds.c). A sole's cone, a contact's push, the faces a wrench
is past, what the floor cannot give taken.

    at, moved = rounds.bounded(u, frames, stance, edge, mu)   # each standing sole's wrench in its cone
    at, kept = rounds.pushed(u, bears, cap, slide)            # each contact's force pressing, in its pyramid
    at, out, to = rounds.faced(u, frames, stance, edge, mu)   # the faces a sole's wrench is past
    at = rounds.taken(u0, metric, up, frames, stance, edge, band, grounded, mu)  # striding, her inertia's share

The unknowns (`machine.wrench`): each leg's six (hip to sole), her upper body's, then each contact's three.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import drives
from machine.figure import BALL, HEEL, JOINTS, SOLE_HALF
from machine.wbc import MU, G

#: Among the driven joints: each leg's six, hip to sole; each knee; the rest, her upper body's.
DRIVEN = [j for j in JOINTS if drives.passive(j) is None]
#: Among the driven joints: each leg's six, hip to sole; each knee; the rest, her upper body's.
LEGS = [[DRIVEN.index(side + k) for k in ('hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll')]
        for side in ('left_', 'right_')]
KNEES = [leg[3] for leg in LEGS]
UPPER = [i for i in range(len(DRIVEN)) if i not in LEGS[0] + LEGS[1]]
NU = 12 + len(UPPER)
#: Which of the unknowns are a sole's moment's parts.
MOMENT = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0] * 2 + [0.0] * len(UPPER))

#: The centre of pressure's room in a sole, across and along, m: its edges less EDGE_M; a
#: sole's yaw moment's arm, m.
EDGE_M, YAW_M = 0.003, 0.04
ROOM = np.array([SOLE_HALF - EDGE_M, (BALL + HEEL) / 2.0 - EDGE_M])
#: Rounds of clipping.
ROUNDS = 4
#: A contact's push no more than this over what it bears now, N a pass (`wrench.W_STILL`).
GROW_N = 30.0
_EYE3 = np.eye(3)
_GIVES = 1e-9 * (np.arange(13) == 12)


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


def pushed(u, bears, cap=np.inf, slide=(0.0, 0.0, 0.0)):
    """`u` with each contact's force (world, after the soles' and her upper body's unknowns)
    pressing, no more than GROW_N over what it `bears` now (N up, one a contact) nor `cap`, and in
    its friction's pyramid - sliding as `slide` has it, its friction kinetic against it -, and
    which parts moved."""
    k = len(bears)
    if not k:
        return u.copy(), np.zeros(len(u))
    out = u.copy()
    f = out[NU:NU + 3 * k].reshape(k, 3)
    held = np.asarray(bears, float)
    f[:, 1] = np.clip(f[:, 1], 0.0, np.minimum(held + GROW_N, cap))
    way = np.asarray(slide, float)[[0, 2]]
    sliding = float(np.hypot(*way) > 0.0)
    f[:, [0, 2]] = ((1.0 - sliding) * np.clip(f[:, [0, 2]], -MU * f[:, 1:2], MU * f[:, 1:2])
                    - sliding * MU * f[:, 1:2] * way / max(float(np.hypot(*way)), 1e-12))
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


def taken(u0, metric, up, frames, stance, edge, band, grounded, mu):
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
