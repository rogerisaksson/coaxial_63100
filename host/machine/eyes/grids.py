"""Her grids' cells grown and joined, her points' hulls: of numpy alone, nothing of hers read.

    wider = grids.grown(cells, r)             # each set cell and those within r of it
    groups = grids.joined(on, offsets)        # [(index arrays)] the set cells' connected groups, lowest first
    hull = grids.convex(pts)                  # [(x, y)] the hull of points, counter-clockwise
    pts = grids.rim(cols, rows)               # a picture's points a hull can hold: each column's lowest and highest

Her map's and her picture's (`machine.eyes.voxels`, `machine.eyes.sight`).
"""
import math


def grown(cells, r, join=None):
    """`cells` (nx, nz) grown by `r` of them round each, `join` of a cell and each neighbour's (numpy's or without)."""
    import numpy as np
    join, out = join or np.logical_or, cells.copy()
    nx, nz = cells.shape
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            if di * di + dj * dj > r * r or (di == 0 and dj == 0):
                continue
            src = cells[max(0, -di):nx - max(0, di), max(0, -dj):nz - max(0, dj)]
            at = out[max(0, di):nx - max(0, -di), max(0, dj):nz - max(0, -dj)]
            at[...] = join(at, src)
    return out


def joined(on, offsets):
    """The connected groups of the set voxels of `on` (any rank), neighbours by `offsets` (one of each
    pair): each a tuple of index arrays, union-find on the flat indices with pointer jumping."""
    import numpy as np
    at = np.flatnonzero(on)
    if not len(at):
        return []
    idx = np.array(np.unravel_index(at, on.shape))
    label = np.arange(len(at))
    strides = np.array([math.prod(on.shape[d + 1:]) for d in range(on.ndim)])
    pairs = []
    for off in offsets:
        other = idx + off[:, None]
        inside = ((other >= 0) & (other < np.array(on.shape)[:, None])).all(axis=0)
        flat = (strides @ other)[inside]
        where = np.searchsorted(at, flat)
        found = where < len(at)
        found[found] = at[where[found]] == flat[found]
        pairs.append((np.flatnonzero(inside)[found], where[found]))
    a, b = np.concatenate([p[0] for p in pairs]), np.concatenate([p[1] for p in pairs])
    while True:
        low = np.minimum(label[a], label[b])
        np.minimum.at(label, a, low)
        np.minimum.at(label, b, low)
        jumped = label[label]
        if np.array_equal(jumped, label) and np.array_equal(label[a], label[b]):
            break
        label = jumped
    order = np.argsort(label, kind='stable')
    starts = np.flatnonzero(np.r_[True, label[order][1:] != label[order][:-1]])
    return [tuple(idx[:, order[s:e]]) for s, e in zip(starts, np.r_[starts[1:], len(order)])]


def rim(cols, rows):
    """[(col, row)] of a picture's points (two (n,) of integers) that a hull can hold: each column's lowest and its
    highest alone - one between them lies on their line -, so `convex` of them is `convex` of them all."""
    import numpy as np
    order = np.lexsort((rows, cols))
    c, r = cols[order], rows[order]
    first = np.flatnonzero(np.r_[True, c[1:] != c[:-1]][:len(c)])
    ends = np.concatenate([first, np.r_[first[1:] - 1, len(c) - 1][:len(first)]])
    return list(zip(c[ends].tolist(), r[ends].tolist()))


def convex(pts):
    """The convex hull of (x, y) points, counter-clockwise (Andrew's monotone chain)."""
    pts = sorted(set(pts))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]
