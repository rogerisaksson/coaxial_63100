"""The robot's capture point's barrier: its centre of mass's ask held to keep it in its support.

    acc = barrier.barred(acc, com, vcom, om, corners, rest)   # the ask, each edge's h' + gamma h >= 0

The kernel's `enforce-dcm-barrier-zcbf`: h = an edge's distance in of the robot's capture point
xi = c + c' / omega, less the text's margin, over the hull of its bearing soles' corners and what else bears it. Each
edge asks h' + GAMMA h >= 0, h' = -n'(c' + c'' / omega): n'c'' <= omega (GAMMA h - n'c'); past it
(h < 0) no further out, h' >= 0 - pulled back it threw the robot's rise forward off its seat. The ask's
floor part is projected onto those half planes, SWEEPS times in turn, as far as `rest`: what the robot's
rows lean on that its comparator does not feel (`observer.weighed`), the text's blind settle, while
it does not stride - a stride moves its support itself. Kept on in every stance it threw the
chair's rise (the robot's pelvis's jerk 266 -> 1 000 m/s^3).
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import libm
from machine.texts import kernel

#: The text's barrier: its decay, /s; the robot's capture point kept this far in of its support's
#: edges, m.
GAMMA = kernel.said('enforce-dcm-barrier-zcbf', 'barrier-gamma')
MARGIN_M = 1e-3 * kernel.said('enforce-dcm-barrier-zcbf', 'min-capture-margin-mm')
#: The projections onto the edges' half planes in turn: a hull's corner meets two.
SWEEPS = 3
#: The hulls the robot's soles and auxiliary supports make, by their corners: three to twelve. Each one's part of
#: `ringed` is traced or bound ahead so no pass pays an 8 ms trace when such a hull first bears the robot.
HULLS = tuple(range(3, 13))


def hull(points):
    """The convex hull of floor `points` (n, 2), counterclockwise, Andrew's monotone chain on Python's
    floats."""
    pts = sorted(map(tuple, np.asarray(points).tolist()))
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


def ringed(acc, com, vcom, om, ring, rest):
    """`acc` (world, y up) with its floor part (x, z) held by the robot's capture point's barrier over a
    hull's `ring` (k, 2), as far as `rest`; in C a ring's length a part (`motion.laid`), its
    np.maximum(0.0, d) Python's max(0.0, d) but at nan."""
    edge = np.roll(ring, -1, axis=0) - ring
    n = np.stack([edge[:, 1], -edge[:, 0]], axis=1) / np.linalg.norm(edge, axis=1)[:, None]
    n *= np.sign(np.einsum('ij,ij->i', n, ring - ring.mean(axis=0)))[:, None]
    v, xi = vcom[[0, 2]], com[[0, 2]] + vcom[[0, 2]] / om
    h = np.einsum('ij,ij->i', n, ring) - n @ xi - MARGIN_M
    bound = om * (GAMMA * np.maximum(h, 0.0) - n @ v)
    a = acc[[0, 2]]
    for _k in range(SWEEPS):
        for ni, bi in zip(n, bound):
            a -= ni * np.maximum(0.0, ni @ a - bi)
    out = acc.copy()
    out[[0, 2]] += rest * (a - acc[[0, 2]])
    return out


def ahead(law, left, warm_only=False, paced=True):
    """`law` (`ringed`, a part a ring's length) on a regular ring of the first length `left`, its result
    dropped, till that part is traced or stays numpy - then that length off `left`: already-kept hulls bind at once,
    untraced ones pace one at a time (`tools.cores.bodied`) unless `paced` is False; `warm_only` binds kept hulls
    without tracing cold ones; `left` empty, nothing."""
    from tools.cores import kept
    while left:
        k = left[0]
        key = ((3,), (3,), (3,), (), (k, 2), ())
        get = getattr(law, 'get', None)
        found = None if get is None else get(key)
        warm = found is not None and kept.ready(found)
        if warm_only and not warm:
            return
        angle = np.arange(k) * (2.0 * np.pi / k)
        z = np.zeros(3)
        ring = 0.1 * np.stack([libm.cos(angle), libm.sin(angle)], axis=1)
        if get is None:
            law(z, z, z, 3.0, ring, 0.0)
        else:
            law(z, z, z, 3.0, ring, 0.0, paced=paced and not warm)
        part = getattr(law, 'part', None)
        if part is None or part.made is not None or part.why is not None:
            left.pop(0)
        if not warm and paced:
            return


def barred(acc, com, vcom, om, corners, rest, law=ringed, hull_=hull):
    """`ringed` by `law` (its C, or itself) over the hull of floor `corners` (n, 3) by `hull_` (its C,
    `tools.cores.borne.hulled`, or `hull`)."""
    return law(acc, com, vcom, om, hull_(np.asarray(corners)[:, [0, 2]]), rest)
