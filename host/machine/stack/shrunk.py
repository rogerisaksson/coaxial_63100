"""The robot's step's answer within its drives (wbc_shrunk.c): shrunk to clamps, clipped to bounds."""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np


def box(b, ask):
    """(limit, lo, hi) each drive's this step, N m: its clamp - its WEP's where granted - as derated,
    the robot's ask shrunk as one to it; its torque's bounds within its back-EMF's (`machine.body.bemf`:
    `ask['lo']`, `ask['hi']`, 0 within both), each drive clipped to them. Shrunk as one to them, a fast
    joint's falling motoring bound took the robot's whole ask down and felled its charged run leap's
    landing."""
    limit = (b.wep if ask.get('wep') else b.clamp) * np.asarray(ask.get('derate', 1.0))
    return limit, np.maximum(-limit, ask.get('lo', -np.inf)), np.minimum(limit, ask.get('hi', np.inf))


def paired(tau, pairs, kt):
    """Each drive's load as its boards carry it: a joint's own torque (N m), or a parallel pair's
    two motor currents (A), common plus or less differential (`wbc_pass.c`)."""
    out = tau.copy()
    for p, r in pairs:
        a, c = tau[p] / kt[p], tau[r] / kt[r]
        out[p], out[r] = a + c, a - c
    return out


def paired_limit(limit, pairs, kt):
    """Each drive's ceiling matching `paired`: its torque limit (N m), or a parallel pair's
    symmetric per-motor current ceiling (A) from its drives' limits."""
    out = limit.copy()
    for p, r in pairs:
        tp, tr = float(limit[p] / kt[p]), float(limit[r] / kt[r])
        out[p] = out[r] = max(tp, tr) if (tp > 0.0 and tr > 0.0) else 0.0
    return out


def paired_shrunk(tau, tau0, ptau, ptau0, limit, plim):
    """Smallest shrink factor keeping every joint within `limit` and every parallel pair's two
    motors within `plim`."""
    return min(shrunk(tau, tau0, -limit, limit), shrunk(ptau, ptau0, -plim, plim))


def overloaded(tau, ptau, clamp, pairs, kt):
    """(over, load) of drives' torques `tau` and paired motor loads `ptau` against `clamp`:
    each joint against its clamp and each parallel pair's two motors against its symmetric
    current clamp, either joint or motor past its clamp flagging both."""
    pclamp = paired_limit(clamp, pairs, kt)
    past = (np.abs(tau) > clamp * (1.0 + 1e-6)) | (np.abs(ptau) > pclamp * (1.0 + 1e-6))
    over = past.copy()
    for p, r in pairs:
        over[p] = over[r] = past[p] or past[r]
    return over, np.maximum(np.abs(tau) / clamp, np.abs(ptau) / pclamp)


def shrunk(tau, tau0, lo, hi):
    """How far from `tau0`, the drives' torques with nothing asked, toward `tau`, the ask's,
    every drive stays within its bounds [`lo`, `hi`] (0 within them): the largest s in [0, 1] with
    every tau0 + s (tau - tau0) within them, each drive's in closed form, the least of them; 0
    where none is."""
    t0 = np.where((tau0 >= lo) & (tau0 <= hi), tau0, 0.0)
    d = tau - t0
    with np.errstate(divide='ignore', invalid='ignore'):
        leaves = np.where(d > 0.0, hi - t0, lo - t0) / d
        enters = np.where(d > 0.0, lo - t0, hi - t0) / d
    s = np.where(d == 0.0, 1.0 * ((tau >= lo) & (tau <= hi)), np.minimum(1.0, leaves))
    s = np.where(s >= np.maximum(0.0, np.where(d == 0.0, 0.0, enters)), s, 0.0)
    return float(np.clip(s.min(), 0.0, 1.0))
