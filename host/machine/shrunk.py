"""Her step's answer within her drives (wbc_shrunk.c): shrunk to her clamps, clipped to her bounds."""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np


def box(b, ask):
    """(limit, lo, hi) each drive's this step, N m: its clamp - its WEP's where granted - as derated,
    her ask shrunk as one to it; its torque's bounds within its back-EMF's (`machine.bemf`: `ask['lo']`,
    `ask['hi']`, 0 within both), each drive clipped to them. Shrunk as one to them, a fast joint's
    falling motoring bound took her whole ask down and felled her charged run leap's landing."""
    limit = (b.wep if ask.get('wep') else b.clamp) * np.asarray(ask.get('derate', 1.0))
    return limit, np.maximum(-limit, ask.get('lo', -np.inf)), np.minimum(limit, ask.get('hi', np.inf))


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
