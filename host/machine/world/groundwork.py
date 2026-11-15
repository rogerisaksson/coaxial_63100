"""The robot's body on the floor: what it rests on, the roll's ring and girdles, a substrate's trough.

    up = groundwork.pitched(phi)                        # (m, 3) the floor's up, the robot's frame, its pitched phi
    up = groundwork.rolled(theta, side)                 # (m, 3) on its back rolled theta onto `side`
    h = groundwork.rest(points, radii, centre, up)      # (m,) m: its centre over the floor it rests on
    gap = groundwork.clear(points, radii, up)           # (m, n) m: each point's skin over that floor
    at = groundwork.settled(h)                          # where it rests: h's local minima round a turn
    w = groundwork.ring(theta)                          # (m, 5) the circumferential ring's shares
    pelvis, thorax = groundwork.girdles(t, omega, total, lag, lead)   # rad: each girdle's roll
    sink = groundwork.sinkage(weight, k)                # m: a substrate's give under its
    tau = groundwork.escape(weight, r, sink, theta)     # N m: the text's torque out of its trough
    hard, soft = groundwork.substrate(k, above, below)  # 1 where a branch of the roll law is on

The kernel's groundwork texts (lisp/humanoid/kernel/floor_*.lisp, ground_rolling_and_transitions.lisp). The robot's
frame `machine.body.figure`'s: x its left, y up, z ahead; its skin as points with radii - a capsule
its two ends, a box its corners at 0. Flat arrays and arithmetic: no branch, no loop, no dict.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

#: The ring's quarters, rad of roll: the robot's back, the side it rolls onto, its front, the other side, its
#: back.
QUARTERS = 0.5 * np.pi * np.arange(5)


def pitched(phi):
    """(m, 3): the floor's up in the robot's frame, its pitched `phi` (m,) rad - 0 upright, pi/2 on its back,
    -pi/2 on its front."""
    phi = np.atleast_1d(phi)
    return np.stack([np.zeros_like(phi), np.cos(phi), np.sin(phi)], axis=1)


def rolled(theta, side):
    """(m, 3): the floor's up in the robot's frame, on its back rolled `theta` (m,) rad about its long axis onto
    `side` (1 its left, -1 its right): pi/2 on that side, pi on its front."""
    theta = np.atleast_1d(theta)
    return np.stack([-side * np.sin(theta), np.zeros_like(theta), np.cos(theta)], axis=1)


def low(points, radii, up):
    """(m, n) m: each point's skin along `up` (m, 3), the robot's frame: its place's height less its
    radius."""
    return up @ points.T - radii


def rest(points, radii, centre, up):
    """(m,) m: the robot's `centre` over the floor it rests on, its up `up` (m, 3): the robot's lowest
    skin on it."""
    return up @ centre - low(points, radii, up).min(axis=1)


def clear(points, radii, up):
    """(m, n) m: each point's skin over the floor the robot rests on (`rest`), 0 where it touches."""
    h = low(points, radii, up)
    return h - h.min(axis=1)[:, None]


def settled(h):
    """Indices where `h` (m,), sampled round a turn, is least of its neighbours: where the robot rests, a nudge
    either way raising it."""
    return np.flatnonzero((h < np.roll(h, 1)) & (h <= np.roll(h, -1)))


def ring(theta):
    """(m, 5): the circumferential ring's shares at roll `theta` (m,) rad (`drive-helical-axial-roll-wave`) - the
    robot's back, the side it rolls onto, its front, the other side, its back - each whole
    at its quarter, shared linearly between."""
    theta = np.atleast_1d(theta)[:, None] % (2.0 * np.pi)
    return np.clip(1.0 - np.abs(theta - QUARTERS) / (0.5 * np.pi), 0.0, 1.0)


def girdles(t, omega, total, lag, lead):
    """(2, m) rad: the robot's pelvis's and its thorax's roll at `t` (m,) s of a roll of `total` rad at `omega` rad/s,
    girdle `lead` (0 the pelvis, 1 the thorax) `lag` rad ahead: its waist turns by their difference, `lag` at most,
    both done at (total + lag) / omega."""
    behind = lag * (np.arange(2) != lead)
    return np.clip(omega * np.atleast_1d(t)[None, :] - behind[:, None], 0.0, total)


def sinkage(weight, k):
    """m: a substrate of stiffness `k` (N/m, d Fz / dz) sunk under `weight` N."""
    return weight / k


def escape(weight, r, sink, theta):
    """N m: the text's torque rolling the robot's `theta` rad out of a trough `sink` m deep, `r` its centre
    from the edge it rolls over: weight (r sin theta + sink / 2 cos theta). The text's m g / (2 k_sub
    L_contact) is a length only with k_sub L_contact the substrate's d Fz / dz, as its roll law reads it: half
    the sinkage."""
    return weight * (r * np.sin(theta) + 0.5 * sink * np.cos(theta))


def substrate(k, above, below):
    """(2, ...): 1 where the roll law's hard branch (`k` over `above`) and its soft branch (`k` under `below`) are on
    (`adapt-rolling-substrate-mechanics`), stiffnesses in one unit."""
    k = np.asarray(k, float)
    return np.stack([np.heaviside(k - above, 0.0), np.heaviside(below - k, 0.0)])
