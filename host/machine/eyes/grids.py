"""The robot's grid cells grown and joined, its points' hulls: of numpy alone, nothing of its own read.

    wider = grids.grown(cells, r)             # each set cell and those within r of it
    groups = grids.joined(on, offsets)        # [(index arrays)] the set cells' connected groups, lowest first
    hull = grids.convex(pts)                  # [(x, y)] the hull of points, counter-clockwise
    pts = grids.rim(cols, rows)               # a picture's points a hull can hold: each column's lowest and highest
    path = grids.astar(cost, start, goal)     # the cells of least cost from start to goal, eight ways
    corners = grids.pruned(cost, path)        # its corners, each line between no dearer than the path it cuts

The robot's map's and its picture's (`machine.eyes.voxels`, `machine.eyes.sight`).
"""
import heapq
import math

from machine import libm


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
    from tools.cores import vision
    got = vision.grid_joined(on, offsets)
    if got is not None:
        return got
    import numpy as np
    at = np.flatnonzero(on)
    if not len(at):
        return []
    idx, a, b = _paired(at, on.shape, offsets)
    label = _unioned(np.arange(len(at)), a, b)
    return _grouped(idx, label, np.argsort(label, kind='stable'))


def joined_by(codes, offsets, xyz=None, gap=0.0, flat=None):
    """[(value, index arrays)] the connected groups of each value's cells of `codes` (integers, -1 none), value by
    value and each value's lowest first - `joined(codes == value)` a value at a time, in one union-find: a full grid's
    a value was most of the robot's picture's lumps."""
    import numpy as np
    if xyz is None or gap <= 0.0:
        from tools.cores import vision
        got = vision.grid_cc_codes(codes, offsets)
        if got is not None:
            if not got:
                return []
            idx, label, value = got
            order = np.lexsort((label, value))
            return list(zip(value[order][_starts(label, order)].tolist(), _grouped(idx, label, order)))
    at = np.flatnonzero(codes >= 0)
    if not len(at):
        return []
    idx, a, b = _paired(at, codes.shape, offsets)
    value = codes.reshape(-1)[at]
    same = value[a] == value[b]
    if xyz is not None and gap > 0.0:
        pts = xyz.reshape(-1, 3)[at]
        ok = np.linalg.norm(pts[a] - pts[b], axis=1) <= gap
        if flat is not None:
            fl = flat.reshape(-1)[at]
            ok = fl[a] | fl[b] | ok
        same = same & ok
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


def convex_xy(at):
    """`(m, 2)` float64 convex hull of `(n, 2)` array `at`."""
    import numpy as np
    from tools.cores import vision
    got = vision.convex_f64(at)
    if got is not None:
        return got
    return np.array(convex(list(map(tuple, np.asarray(at).tolist()))), float).reshape(-1, 2)


def convex(pts):
    """The convex hull of (x, y) points, counter-clockwise (Andrew's monotone chain)."""
    if isinstance(pts, (list, tuple)) and len(pts) >= 8:
        all_int = all(len(p) == 2 and type(p[0]) is int and type(p[1]) is int
                      and abs(p[0]) < (1 << 52) and abs(p[1]) < (1 << 52) for p in pts)
        all_flt = not all_int and all(len(p) == 2 and type(p[0]) is float and type(p[1]) is float for p in pts)
        if all_int or all_flt:
            from tools.cores import vision
            got = vision.convex_f64(pts)
            if got is not None:
                return [(int(x), int(y)) for x, y in got.tolist()] if all_int else [tuple(xy) for xy in got.tolist()]
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


def nearest_free(blocked, cell):
    """The free cell of `blocked` (nx, nz) nearest `cell`, `cell` itself where none is."""
    from tools.cores import way
    got = way.nearest_free(blocked, cell)
    if got is not None:
        return got
    import numpy as np
    free = np.argwhere(~blocked)
    if len(free) == 0:
        return cell
    k = np.argmin((libm.sq(free - np.array(cell))).sum(axis=1))
    return (int(free[k][0]), int(free[k][1]))


def astar(cost, start, goal):
    """The cells from `start` to `goal` of least cost, `cost` (nx, nz) a metre's through each, inf blocked:
    eight ways, a step its length times its two cells' mean cost, no diagonal past a blocked cell; []
    where none. Each cell costs 1 at least: the straight distance is its heuristic."""
    from tools.cores import way
    got = way.astar(cost, start, goal)
    if got is not None:
        return got
    return astar_twin(cost, start, goal)


def astar_twin(cost, start, goal):
    """`astar` in pure Python `heapq`, bit-exact oracle for `way.astar`."""
    nx, nz = cost.shape
    c_ = cost.tolist()
    root2 = math.sqrt(2.0)
    steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, root2), (1, -1, root2), (-1, 1, root2), (-1, -1, root2)]
    came, spent = {start: None}, {start: 0.0}
    todo = [(math.dist(start, goal), start)]
    while todo:
        _f, here = heapq.heappop(todo)
        if here == goal:
            path = []
            while here is not None:
                path.append(here)
                here = came[here]
            return path[::-1]
        for di, dj, w in steps:
            nxt = (here[0] + di, here[1] + dj)
            if not (0 <= nxt[0] < nx and 0 <= nxt[1] < nz) or math.isinf(c_[nxt[0]][nxt[1]]):
                continue
            if w > 1.0 and (math.isinf(c_[here[0] + di][here[1]]) or math.isinf(c_[here[0]][here[1] + dj])):
                continue
            c = spent[here] + w * 0.5 * (c_[here[0]][here[1]] + c_[nxt[0]][nxt[1]])
            if c < spent.get(nxt, math.inf):
                spent[nxt], came[nxt] = c, here
                heapq.heappush(todo, (c + math.dist(nxt, goal), nxt))
    return []


def line(a, b):
    """The cells of the straight line from `a` to `b`, a cell a sample."""
    n = max(abs(b[0] - a[0]), abs(b[1] - a[1]), 1)
    return [(int(round(a[0] + (b[0] - a[0]) * k / n)), int(round(a[1] + (b[1] - a[1]) * k / n))) for k in range(n + 1)]


def cuts(cost, a, b, most):
    """Whether the straight line from cell `a` to `b` is free and its dearest cell costs `most` or less."""
    on = [float(cost[c]) for c in line(a, b)]
    return max(on) <= most + 1e-9


def pruned(cost, path):
    """The path's corners: each the furthest cell straight from the one before whose line's dearest
    cell is no dearer than the path's it cuts (`cuts`): no nearer things than the way it
    replaces; at a cost of 1 each, the furthest seen free."""
    out, i = [path[0]], 0
    on = [float(cost[c]) for c in path]
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not cuts(cost, path[i], path[j], max(on[i:j + 1])):
            j -= 1
        out.append(path[j])
        i = j
    return out
