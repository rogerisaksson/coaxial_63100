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
    idx, a, b = _paired(at, on.shape, offsets)
    label = _unioned(np.arange(len(at)), a, b)
    return _grouped(idx, label, np.argsort(label, kind='stable'))


def joined_by(codes, offsets):
    """[(value, index arrays)] the connected groups of each value's cells of `codes` (integers, -1 none), value by
    value and each value's lowest first - `joined(codes == value)` a value at a time, in one union-find: a full grid's
    a value was most of her picture's lumps."""
    import numpy as np
    at = np.flatnonzero(codes >= 0)
    if not len(at):
        return []
    idx, a, b = _paired(at, codes.shape, offsets)
    value = codes.reshape(-1)[at]
    same = value[a] == value[b]
    label = _unioned(np.arange(len(at)), a[same], b[same])
    order = np.lexsort((label, value))
    return list(zip(value[order][_starts(label, order)].tolist(), _grouped(idx, label, order)))


def coded(*columns):
    """Each row's place among the distinct rows of integer-valued `columns` ((n,) each), first column first - numpy's
    unique of the rows stacked (axis 0) - as one integer a row: its unique of rows sorted them field by field."""
    import numpy as np
    key = np.zeros(len(columns[0]), np.int64)
    for c in columns:
        c = np.asarray(c).astype(np.int64)
        lo = int(c.min()) if len(c) else 0
        key = key * (int(c.max()) - lo + 1 if len(c) else 1) + (c - lo)
    return np.unique(key, return_inverse=True)[1].reshape(-1)


def _paired(at, shape, offsets):
    """(indices (ndim, n) of the flat places `at` (ascending) of a grid `shape`, and the pairs of them neighbours by
    `offsets`: (a, b) places in `at`)."""
    import numpy as np
    idx = np.array(np.unravel_index(at, shape))
    strides = np.array([math.prod(shape[d + 1:]) for d in range(len(shape))])
    pairs = []
    for off in offsets:
        other = idx + off[:, None]
        inside = ((other >= 0) & (other < np.array(shape)[:, None])).all(axis=0)
        flat = (strides @ other)[inside]
        where = np.searchsorted(at, flat)
        found = where < len(at)
        found[found] = at[where[found]] == flat[found]
        pairs.append((np.flatnonzero(inside)[found], where[found]))
    return idx, np.concatenate([p[0] for p in pairs]), np.concatenate([p[1] for p in pairs])


def _unioned(label, a, b):
    """`label` joined over the pairs (a, b), each its group's least, by pointer jumping."""
    import numpy as np
    while True:
        low = np.minimum(label[a], label[b])
        np.minimum.at(label, a, low)
        np.minimum.at(label, b, low)
        jumped = label[label]
        if np.array_equal(jumped, label) and np.array_equal(label[a], label[b]):
            return label
        label = jumped


def _starts(label, order):
    """Where each group begins in `order`."""
    import numpy as np
    return np.flatnonzero(np.r_[True, label[order][1:] != label[order][:-1]])


def _grouped(idx, label, order):
    """[index arrays] each group's, in `order`."""
    import numpy as np
    starts = _starts(label, order)
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
