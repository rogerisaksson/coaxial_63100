"""The board's step's rounds (wbc_wrench.c) in Python: its least squares clipped a round at a time.

The rows' least squares clipped into the soles' and contacts' bounds a round at a time, the clipped pinned for the
next: the board's own step is held to it (tests/test_wbc_core, test_shield); the robot's loop solves within its
bounds instead (`within.within`).

    whole, first = clipped.solve(H0, g0, held, bears, lo, hi, geo, stride)   # its rounds
    out = clipped.step(b, s, ask)            # `wrench.step` on them
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import libm
from machine.stack.rounds import GROW_N, NU, ROOM, ROUNDS, YAW_M, taken
from machine.stack.wbc import MU, G
from machine.stack import wrench
from machine.stack.within import solved
from machine.stack.wrench import W_PIN

#: Rounds more, at most, for an answer past the robot's height's band after them.
PAST = 2


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


def pushed(u, bears, cap=np.inf, slips=None, give=1.0):
    """`u` with each contact's force (world, after the soles' and the robot's upper body's unknowns)
    pressing, no more than GROW_N over what it `bears` now (N up, one a contact) nor `cap` (N, one a
    contact or one for all), of that `give` the robot's own to plan (none where what bears it carries
    the robot as felt, `wrench.step`), and in its friction's pyramid - a point sliding (`slips`: each one's
    (sliding, its slip along the world's x and z, m/s), `wrench._patch`) has its friction the kinetic one
    against its slip, given -, and which parts moved."""
    k = len(bears)
    if not k:
        return u.copy(), np.zeros(len(u))
    out = u.copy()
    f = out[NU:NU + 3 * k].reshape(k, 3)
    held = np.asarray(bears, float)
    f[:, 1] = np.clip(f[:, 1], 0.0, np.minimum(held + GROW_N, cap) * give)
    slid = np.zeros((k, 3)) if slips is None else np.asarray(slips, float).reshape(k, 3)
    sliding, way = slid[:, :1], slid[:, 1:]
    speed = libm.hypot(way[:, 0], way[:, 1])
    f[:, [0, 2]] = ((1.0 - sliding) * np.clip(f[:, [0, 2]], -MU * f[:, 1:2], MU * f[:, 1:2])
                    - sliding * MU * f[:, 1:2] * way / np.maximum(speed, 1e-12)[:, None])
    out[NU:NU + 3 * k] = f.ravel()
    return out, (np.abs(out - u) > 1e-9).astype(float)


def solve(H0, g0, held, bears, lo, hi, geo, stride):
    """(The unknowns, the first, floor-unbounded answer) for the rows' normal matrix `H0` and
    right-hand side `g0` (R^T c), the contacts'
    `held` rows (`bears`, N up), the robot's height's band `lo` to `hi`: the rounds, then the stride. `geo`: (nu,
    k3, moment, part, up, lift, mass, grounded, stance, frames, edges, mu, caps, slips, give - the share of a
    contact's push the robot's own to plan); `stride`: (strides, its kinetic metric)."""
    nu, k3, moment, part, up, lift, mass, grounded, stance, frames, edges, mu, cap, slips, give, _net = geo
    strides, kinetic = stride
    pinned, at, band, edge, first = np.zeros(nu), np.zeros(nu), 0.0, 0.0, np.zeros(nu)
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
        # value and the robot's height at its band's edge is met, the rest solved round them: as rows
        # of W_PIN they were 1e7 on the diagonal, and what the robot's soles share came 0.6 N m apart
        # from one solve to another
        hard, soft = pinned * moment, libm.sq(W_PIN * pinned * (1.0 - moment) * part)
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
        u, way = solved(K, np.array([free * (gr + rhs_s - H @ (hard * at)) + hard * at, up]).T, free).T
        u = u + band * (edge - up @ u) / (up @ way + 1.0 - band) * way
        first = u.copy() if done == 0 else first
        # the robot's height's band: past an edge, held to it
        ay = float(lift @ u) / mass - G
        past = min(max(ay, lo), hi) + G
        band, edge = max(band, grounded * float(abs(past - G - ay) > 1e-9)), edge + (1.0 - band) * (past - edge)
        at, moved = bounded(u, frames, stance, edges, mu)
        if k3:
            at, kept = pushed(at, bears, cap, slips, give)
            pressed = pressed * (u[NU + 1:NU + k3:3] >= 0.0)
            pinned = np.maximum(pinned, np.maximum(moved, kept))
        else:
            pinned = np.maximum(pinned, moved)
        done += 1
        # an answer past the robot's height's band after the rounds - a contact let go in the last, its
        # solve never held to it - is held to its edge: off a mattress's edge the step planned 145
        # N up of the robot's 358 asked and the edge took 309 along its normal, 23 deg ahead
        ay = float(lift @ at) / mass - G
        if (done == rounds < ROUNDS + PAST and not strides
                and grounded * abs(min(max(ay, lo), hi) - ay) > 1e-9):
            band, edge, rounds = 1.0, min(max(ay, lo), hi) + G, rounds + 1
        if (not pinned.any() and not band) or (strides >= 1.0 and k3 == 0):
            break
    if strides > 0.0 and (pinned.any() or band):
        return at + strides * (taken(solved(Hr, gr, np.ones(nu)), kinetic, up, frames, stance, edges, (lo, hi),
                                      grounded, mu) - at), first
    return at, first


def step(b, s, ask):
    """`wrench.step` on the board's rounds (`solve`): the answer wbc_wrench.c is held to."""
    return wrench.step(b, s, ask, solve)
