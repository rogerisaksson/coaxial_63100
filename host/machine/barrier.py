"""Her capture point's barrier: her centre of mass's ask held to keep it in her support.

    acc = barrier.barred(acc, com, vcom, om, corners, rest)   # the ask, each edge's h' + gamma h >= 0

The kernel's `enforce-dcm-barrier-zcbf`: h = an edge's distance in of her capture point
xi = c + c' / omega, less the text's margin, over the hull of her bearing soles' corners and what else bears her. Each
edge asks h' + GAMMA h >= 0, h' = -n'(c' + c'' / omega): n'c'' <= omega (GAMMA h - n'c'); past it
(h < 0) no further out, h' >= 0 - pulled back it threw her rise forward off its seat. The ask's
floor part is projected onto those half planes, SWEEPS times in turn, as far as `rest`: what her
rows lean on that her comparator does not feel (`observer.weighed`), the text's blind settle, while
she does not stride - a stride moves her support itself. Kept on in every stance it threw the
chair's rise (her pelvis's jerk 266 -> 1 000 m/s^3).
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import kernel

#: The text's barrier: its decay, /s; her capture point kept this far in of her support's edges, m.
GAMMA = kernel.said('enforce-dcm-barrier-zcbf', 'barrier-gamma')
MARGIN_M = 1e-3 * kernel.said('enforce-dcm-barrier-zcbf', 'min-capture-margin-mm')
#: The projections onto the edges' half planes in turn: a hull's corner meets two.
SWEEPS = 3


def hull(points):
    """The convex hull of floor `points` (n, 2), counterclockwise, Andrew's monotone chain."""
    pts = sorted(map(tuple, points))
    turn = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and turn(lower[-2], lower[-1], p) <= 0.0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and turn(upper[-2], upper[-1], p) <= 0.0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def barred(acc, com, vcom, om, corners, rest):
    """`acc` (world, y up) with its floor part (x, z) held by her capture point's barrier over the
    hull of `corners` (n, 3), as far as `rest`."""
    ring = hull(np.asarray(corners)[:, [0, 2]])
    edge = np.roll(ring, -1, axis=0) - ring
    n = np.stack([edge[:, 1], -edge[:, 0]], axis=1) / np.linalg.norm(edge, axis=1)[:, None]
    n *= np.sign(np.einsum('ij,ij->i', n, ring - ring.mean(axis=0)))[:, None]
    v, xi = vcom[[0, 2]], com[[0, 2]] + vcom[[0, 2]] / om
    h = np.einsum('ij,ij->i', n, ring) - n @ xi - MARGIN_M
    bound = om * (GAMMA * np.maximum(h, 0.0) - n @ v)
    a = acc[[0, 2]].copy()
    for _k in range(SWEEPS):
        for ni, bi in zip(n, bound):
            a -= ni * max(0.0, float(ni @ a) - bi)
    out = acc.copy()
    out[[0, 2]] += rest * (a - acc[[0, 2]])
    return out
