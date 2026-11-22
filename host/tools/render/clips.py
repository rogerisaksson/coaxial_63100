"""Where drawn meshes cut through each other: the triangle pairs that cross; meshes split finer.

    clips.crossed(a, b)       # how many triangle pairs of `a` and `b` [(tri, colour)] cross: an edge of one through
                              # the other's face - cloth through a guard, a strut through a cage
    clips.finer(tris, edge)   # `tris` split until no edge is longer than `edge`, m

The pairs are found a grid cell at a time (CELL_M): each triangle's box marks its cells, and only triangles of the
two sharing a cell are tried - each edge of either against the other's face (Moller-Trumbore), PART of a pair's
parameters short of its ends, so a strut set on a face, or two faces touching, is no cut.
"""
import numpy as np

#: The grid's cell, m, and how far short of an edge's ends and a face's rim a cut must lie, of their lengths.
CELL_M, PART = 0.006, 1e-6
#: The pairs tried at once.
AT_ONCE = 200_000


def _cells(tris):
    """(triangle, cell) rows: every grid cell each triangle's box spans, a cell as one integer."""
    lo, hi = np.floor(tris.min(axis=1) / CELL_M).astype(np.int64), np.floor(tris.max(axis=1) / CELL_M).astype(np.int64)
    span = hi - lo + 1
    count = span.prod(axis=1)
    owner = np.repeat(np.arange(len(tris)), count)
    k = np.arange(count.sum()) - np.repeat(np.cumsum(count) - count, count)
    sx, sy = span[owner, 0], span[owner, 1]
    cell = lo[owner] + np.stack([k % sx, (k // sx) % sy, k // (sx * sy)], axis=1)
    return owner, (cell[:, 0] * 73856093) ^ (cell[:, 1] * 19349663) ^ (cell[:, 2] * 83492791)


def _pairs(a, b):
    """The (a, b) triangle pairs sharing a grid cell, each once."""
    ia, ka = _cells(a)
    ib, kb = _cells(b)
    order = np.argsort(kb)
    kb, ib = kb[order], ib[order]
    first, last = np.searchsorted(kb, ka, 'left'), np.searchsorted(kb, ka, 'right')
    count = last - first
    left = np.repeat(ia, count)
    right = ib[np.repeat(first, count) + np.arange(count.sum()) - np.repeat(np.cumsum(count) - count, count)]
    return np.unique(np.stack([left, right], axis=1), axis=0)


def _pierced(p, q, v0, v1, v2):
    """Whether each segment p-q passes through its triangle (v0, v1, v2), short of their rims by PART."""
    d, e1, e2 = q - p, v1 - v0, v2 - v0
    h = np.cross(d, e2)
    a = np.einsum('ij,ij->i', e1, h)
    ok = np.abs(a) > 1e-18
    f = np.where(ok, 1.0 / np.where(ok, a, 1.0), 0.0)
    s = p - v0
    u = f * np.einsum('ij,ij->i', s, h)
    w = np.cross(s, e1)
    v, t = f * np.einsum('ij,ij->i', d, w), f * np.einsum('ij,ij->i', e2, w)
    return ok & (u > PART) & (v > PART) & (u + v < 1.0 - PART) & (t > PART) & (t < 1.0 - PART)


def crossed(a, b):
    """How many triangle pairs of `a` and `b` ([(tri, colour)]) cross: an edge of either through the other's face."""
    if not a or not b:
        return 0
    ta, tb = np.array([t for t, _c in a], float), np.array([t for t, _c in b], float)
    lo, hi = np.maximum(ta.min(axis=(0, 1)), tb.min(axis=(0, 1))), np.minimum(ta.max(axis=(0, 1)), tb.max(axis=(0, 1)))
    if (lo > hi).any():
        return 0
    ta = ta[(ta.max(axis=1) >= lo).all(axis=1) & (ta.min(axis=1) <= hi).all(axis=1)]
    tb = tb[(tb.max(axis=1) >= lo).all(axis=1) & (tb.min(axis=1) <= hi).all(axis=1)]
    if not len(ta) or not len(tb):
        return 0
    pairs, out = _pairs(ta, tb), 0
    for k in range(0, len(pairs), AT_ONCE):
        x, y = ta[pairs[k:k + AT_ONCE, 0]], tb[pairs[k:k + AT_ONCE, 1]]
        hit = np.zeros(len(x), bool)
        for one, other in ((x, y), (y, x)):
            for i in range(3):
                hit |= _pierced(one[:, i], one[:, (i + 1) % 3], other[:, 0], other[:, 1], other[:, 2])
        out += int(hit.sum())
    return out


def finer(tris, edge):
    """`tris` [(tri, colour)] each split into k by k alike, k the least that leaves no edge longer than `edge`."""
    out = []
    for tri, colour in tris:
        k = max(1, int(np.ceil(max(np.linalg.norm(tri[i] - tri[(i + 1) % 3]) for i in range(3)) / edge)))
        if k == 1:
            out.append((tri, colour))
            continue
        at = {(i, j): tri[0] + (tri[1] - tri[0]) * i / k + (tri[2] - tri[0]) * j / k
              for i in range(k + 1) for j in range(k + 1 - i)}
        for i in range(k):
            for j in range(k - i):
                out.append((np.array([at[i, j], at[i + 1, j], at[i, j + 1]]), colour))
                if i + j < k - 1:
                    out.append((np.array([at[i + 1, j], at[i + 1, j + 1], at[i, j + 1]]), colour))
    return out
