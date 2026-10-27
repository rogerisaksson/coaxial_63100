"""Her body at a seat's or a bed's edge: what meets it behind her, a rim's halt, a roll's drift.

    reach = bedside.behind(points, radii, lo, hi)  # (m, n) m: each point's skin behind her within heights lo..hi
    v = bedside.halted(gap, speed, gamma)          # m/s: a slide's speed its rim's barrier lets it keep
    f = bedside.pendulum(moment, inertia)          # Hz: a limb hung from its joint, swinging free
    y = bedside.drift(h, theta)                    # m: her travel rolling on a floor, h her centre over it
    v = bedside.escape(h, rate, off, gain)         # m/s: the shrimp's scoot across, her centre held on its line

The kernel's bedside texts (machine/ortho_kernel/bedside_*.lisp). Her frame `machine.figure`'s: x her left, y up,
z ahead; her skin as points with radii (`groundwork`). Flat arrays and arithmetic: no branch, no loop, no dict.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np


def behind(points, radii, lo, hi):
    """(m, n) m: how far behind her (-z) each point's skin (`points` (n, 3), `radii` (n,)) reaches within heights
    `lo`..`hi` (m,), -inf where it reaches none of them: a fixture's face there meets the most of a row first as she
    backs onto it (`index-popliteal-standoff-datum`)."""
    lo, hi = np.asarray(lo, float)[:, None], np.asarray(hi, float)[:, None]
    off = np.clip(lo - points[:, 1], 0.0, None) + np.clip(points[:, 1] - hi, 0.0, None)
    return np.where(off <= radii, np.sqrt(np.clip(radii * radii - off * off, 0.0, None)) - points[:, 2], -np.inf)


def halted(gap, speed, gamma):
    """m/s: a slide's speed toward its halt, `gap` m short of it - its asked `speed`, no more than `gamma` /s times the
    gap (the rim's h' >= -gamma h, `inject-elliptical-seat-rim-barrier`): it nears the halt as exp(-gamma t), never
    past."""
    return np.clip(gamma * np.asarray(gap, float), 0.0, speed)


def pendulum(moment, inertia):
    """Hz: a limb hung from its joint, swinging free - `moment` its mass times its centre's distance below the joint
    (kg m), `inertia` about the joint (kg m^2) (`specialize-posture-dangling-crural-hang`'s eigenfrequency)."""
    return np.sqrt(9.81 * np.asarray(moment, float) / inertia) / (2.0 * np.pi)


def drift(h, theta):
    """m: her travel along the floor rolling without slip through `theta` (m,) rad, `h` (m,) her centre's height over
    the floor there (`groundwork.rest`): the integral of h d theta from the first sample, R theta for a cylinder."""
    return np.concatenate([[0.0], np.cumsum(0.5 * (h[1:] + h[:-1]) * np.diff(theta))])


def escape(h, rate, off, gain):
    """m/s: the shrimp's scoot across the bed (`execute-zero-translation-bed-roll`), -h theta' - gain off: `h` her
    centre's height over the bed (the text's R_eff), `rate` her roll's rad/s, `off` her centre from its line, m."""
    return -h * rate - gain * off
