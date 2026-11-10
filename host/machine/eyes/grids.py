"""Her grids' cells grown and joined: a grid grown round its set cells, their connected groups.

    wider = grids.grown(cells, r)             # each set cell and those within r of it
    groups = grids.joined(on, offsets)        # [(index arrays)] the set cells' connected groups, lowest first

Her map's and her picture's (`machine.eyes.voxels`, `machine.eyes.sight`): numpy alone, nothing of hers read.
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
