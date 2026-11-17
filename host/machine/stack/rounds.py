"""The step's unknowns, its soles' faces and a stride's share taken (wbc_rounds.c).

    at, out, to = rounds.faced(u, frames, stance, edge, mu)   # the faces a sole's wrench is past
    at = rounds.taken(u0, metric, up, frames, stance, edge, band, grounded, mu)  # striding, the robot's inertia's share

The unknowns (`machine.stack.wrench`): each leg's six (hip to sole), the robot's upper body's, then each
contact's three.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body import drives
from machine.body.figure import BALL, HEEL, JOINTS, SOLE_HALF
from machine.stack.wbc import MU, G

#: Among the driven joints: each leg's six, hip to sole; each knee; the rest, the robot's upper body's.
DRIVEN = [j for j in JOINTS if drives.passive(j) is None]
#: Among the driven joints: each leg's six, hip to sole; each knee; the rest, the robot's upper body's.
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
#: Rounds of a stride's faces (`taken`).
ROUNDS = 4
#: A contact's push no more than this over what it bears now, N a pass (`wrench.W_STILL`).
GROW_N = 30.0
#: A contact's friction's sixteen-sided pyramid inscribed in its round cone (the world's: elliptic): its faces'
#: directions on the world's x and z, k pi / 8, and their reach, cos(pi / 16) of the cone's - square roots alone, so
#: the C's the same bits. The box pyramid planned a seat 1.41 of its friction across its corner, the robot slid off it;
#: the eight-sided one 0.92 along its faces, a chair's dismount fell short of the floor.
_RH = float(np.sqrt(0.5))
_C8, _S8 = float(np.sqrt(0.5 + 0.5 * _RH)), float(np.sqrt(0.5 - 0.5 * _RH))
CONE = ((1.0, 0.0), (_C8, _S8), (_RH, _RH), (_S8, _C8), (0.0, 1.0), (-_S8, _C8), (-_RH, _RH), (-_C8, _S8),
        (-1.0, 0.0), (-_C8, -_S8), (-_RH, -_RH), (-_S8, -_C8), (0.0, -1.0), (_S8, -_C8), (_RH, -_RH), (_C8, -_S8))
COS16 = float(np.sqrt(0.5 + 0.5 * _C8))
_EYE3 = np.eye(3)
_GIVES = 1e-9 * (np.arange(13) == 12)


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
    """Striding, what the floor cannot give the robot's inertia takes: `u0`, the rows' answer with the
    floor unbounded, brought to the faces its soles' wrenches are past by the least change in
    `metric` - its kinetic energy's, a swinging sole's ask held. Traded among the rows, a row
    a joint, the robot's head was the cheapest moment about it and nodded 46 deg a stride; its
    centre of mass held as well, its trunk pitched 9 to 10 deg, or it was down at 12 s. A
    face is a part of a sole's wrench its share of the sole's push, about the sole's own
    axes; the robot's height's `band` is decided before the faces - after them a throw's soles,
    asked to pull by its knees' rows, pressed nothing - and gives where a cone and it cannot
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
