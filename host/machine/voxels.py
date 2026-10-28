"""Her map of the room in voxels: its surfaces and structures told as boxes, each with its turn.

What her looks put there, in her estimate's frame.

    chart = voxels.Map(palette)                  # CELL_M voxels over the room's floor, FLOOR_M to CEIL_M
    cells = chart.add(xyz, rgb, eye)             # one look's points and their colours; the rays' voxels freed
    things, labels = chart.things()              # (kind, centre, half, R, id, clear) each, a voxel's thing in labels
    chart.blocked()                              # the floor's cells she may not stand in, her radius round

A voxel is a thing from TOLD looks with HITS points in it, hit more often than seen through; its kind
the `palette`'s whose body colour its chromaticity lies nearest, the light's white added (the
dichromatic model: a highlight lies on the line from a kind's chroma to grey, so a lit timber wall
stays timber and a grey patch its neighbours'). A top is an occupied voxel seen through above with
nothing standing in the layer above round it - its normal up, a hole in a wall's face none -; a
surface the 4-connected tops of one level, a structure the 26-connected rest - joined apart over
and under a surface within its footprint a voxel round: what stands on a seat is not what holds it
up -, each wall one, a chair's seat a surface, its back and its legs structures; a part of a
structure whose colour differs from
the structure's past the three-sigma test on its own pixels (`SIGMA`) a thing of its own - the door
in the glass gable, a chair's back against a wall; a structure resting on nothing, neither the floor's
layer nor under a surface, a stray left out. A thing's kind its pooled colour's; its box its
footprint's principal axes (its turn), its extents along them a voxel wide, its height its voxels', a
structure in pieces till no box claims floor further than a voxel from its cells (`_pieces`); its
id the one of the thing it shares most voxels with at the look before, else new (`machine.registry`);
`clear` 1.0 where the space over its top is seen through - a surface's -, 0.0 a structure's, its top
unknown (`machine.affordance`: a seat or a bed is a surface). Nothing here reads the world's geometry.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import figure, room

#: Her map: CELL_M voxels over the room's floor (X_M, Z_M), FLOOR_M to CEIL_M high - her eyes' kept points
#: fill none finer: at 0.05 m a look round told 553-791 things to 85 (HITS 1-3; 490-559 at 640 x 480), her
#: registry's 256 slots all live and the bed off her buses -; a voxel is a thing from TOLD looks with HITS
#: points in it; a point NEAR_M to FAR_M from the eye - further the depth is a tenth out, nearer is her own
#: raised hand -; her own radius, inflating what she plans round: her widest half (`figure.WIDE_HALF`) and
#: the map's half cell - 0.3 shut the 0.9 m doorway.
CELL_M, X_M, Z_M, FLOOR_M, CEIL_M = 0.1, (-3.5, 3.5), (-2.0, 5.0), 0.08, 2.3
HITS, TOLD, NEAR_M, FAR_M = 3, 2, 0.7, 3.5
RADIUS_M = figure.WIDE_HALF + 0.5 * CELL_M
#: What she may not stand in: a thing over her ankle and under the doorway's top - she steps over
#: lower, walks under higher, pushes the door aside (`route.blocked`).
STAND_M = (figure.ANKLE_H, 2.0 * room.DOOR[1])
#: The 26 neighbours' offsets in a voxel grid, one of each pair.
_AROUND = np.array([(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1) if (i, j, k) > (0, 0, 0)])
#: The four neighbours' offsets in a floor grid, one of each pair.
_BESIDE = np.array([(1, 0), (0, 1)])
#: A cell's own and its eight neighbours' offsets, (i, k).
_ROUND = np.array([(i, k) for i in (-1, 0, 1) for k in (-1, 0, 1)])
#: A part of a structure is its own thing where its pooled colour lies this many standard errors off the
#: structure's kind's lit line, the error the structure's own pixels' scatter: the three-sigma test.
SIGMA = 3.0


#: The light's chromaticity: white.
GREY = np.full(3, 1.0 / 3.0)


def chroma(rgb):
    """A colour's chromaticity, its share of each of r, g, b."""
    return rgb / np.maximum(rgb.sum(axis=-1, keepdims=True), 1e-6)


def lit(c, means):
    """(..., K) each chromaticity's distance from each kind's: the least to the line from the kind's body
    colour `means` (K, 3) to the light's GREY, a highlight anywhere on it."""
    body = means - GREY
    span = np.maximum((body * body).sum(axis=1), 1e-12)
    t = np.clip(((c[..., None, :] - GREY) * body).sum(axis=-1) / span, 0.0, 1.0)
    return np.linalg.norm(c[..., None, :] - (GREY + t[..., None] * body), axis=-1)


class Map:

    """Her voxels, `voxel` m a side: each's hits, looks that told it, looks through it, its colour."""

    def __init__(self, palette, voxel=CELL_M):
        self.means_of, self.names, self.voxel = dict(palette), list(palette), voxel
        self.means = chroma(np.array([palette[k] for k in self.names], float))
        self.nx = int(round((X_M[1] - X_M[0]) / self.voxel))
        self.ny = int(math.ceil((CEIL_M - FLOOR_M) / self.voxel))
        self.nz = int(round((Z_M[1] - Z_M[0]) / self.voxel))
        shape = (self.nx, self.ny, self.nz)
        self.hits, self.told, self.free = (np.zeros(shape, dtype=np.int32) for _ in range(3))
        self.colour = np.zeros(shape + (3,), dtype=np.float32)
        self.looks, self._made = 0, None
        self._labels, self._ids, self._next = None, [], 1

    def cell(self, x, z):
        return (int(math.floor((x - X_M[0]) / self.voxel)), int(math.floor((z - Z_M[0]) / self.voxel)))

    def centre(self, i, k):
        return (X_M[0] + (i + 0.5) * self.voxel, Z_M[0] + (k + 0.5) * self.voxel)

    def _index(self, xyz):
        """(i, j, k) of points `xyz` (n, 3), and whether each lies in the map."""
        i = np.floor((xyz[:, 0] - X_M[0]) / self.voxel).astype(int)
        j = np.floor((xyz[:, 1] - FLOOR_M) / self.voxel).astype(int)
        k = np.floor((xyz[:, 2] - Z_M[0]) / self.voxel).astype(int)
        inside = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny) & (k >= 0) & (k < self.nz)
        return i, j, k, inside

    def add(self, xyz, rgb, eye):
        """One look into the map: the points `xyz` (n, 3, her estimate's frame) with their colours `rgb`
        (n, 3) standing between FLOOR_M and CEIL_M, NEAR_M to FAR_M from `eye`; the voxels their rays
        crossed, free. (n,) each point's voxel in the map's flat order, -1 off it."""
        away = np.linalg.norm(xyz - eye, axis=1)
        ok = (xyz[:, 1] > FLOOR_M) & (xyz[:, 1] < CEIL_M) & (away > NEAR_M) & (away < FAR_M)
        self._seen_through(eye, xyz[ok])
        i, j, k, inside = self._index(xyz)
        on = ok & inside
        flat = np.where(on, (i * self.ny + j) * self.nz + k, -1)
        at = flat[on]
        np.add.at(self.hits.reshape(-1), at, 1)
        self.told.reshape(-1)[:] += np.bincount(at, minlength=self.hits.size) >= HITS
        np.add.at(self.colour.reshape(-1, 3), at, rgb[on])
        self.looks, self._made = self.looks + 1, None
        return flat

    def _seen_through(self, eye, xyz, step=0.8):
        """The voxels each ray from `eye` crossed short of its point by a voxel, `step` voxels a sample, up to
        the first her map holds occupied - a ray grazing along a wall she has seen stops at it -, counted
        free once a look."""
        step, d = step * self.voxel, xyz - eye
        far = np.linalg.norm(d, axis=1)
        n = int(np.ceil(far.max() / step)) if len(far) else 0
        if n == 0:
            return
        t = (np.arange(n) * step)[None, :]
        k = t / np.maximum(far[:, None], 1e-6)
        i, j, kk, inside = self._index((eye + d[:, None, :] * k[:, :, None]).reshape(-1, 3))
        flat = np.where(inside, (i * self.ny + j) * self.nz + kk, 0).reshape(len(far), n)
        solid = (inside & self.occupied().reshape(-1)[flat.reshape(-1)]).reshape(len(far), n)
        stop = np.where(solid.any(axis=1), solid.argmax(axis=1), n)
        on = (t < (far[:, None] - self.voxel)) & (np.arange(n)[None, :] < stop[:, None]) & inside.reshape(len(far), n)
        self.free.reshape(-1)[np.unique(flat[on])] += 1

    def occupied(self):
        """The voxels TOLD looks have shown a thing in, more than have seen through them."""
        return (self.told >= TOLD) & (self.told > self.free)

    def kinds(self):
        """Each occupied voxel's kind: the palette's nearest its colour's chromaticity, lit (`lit`); -1 the
        rest - over every voxel it took 0.78 s of a look's 1.30 at 0.05 m."""
        out, occ = np.full(self.hits.shape, -1), self.occupied()
        out[occ] = np.argmin(lit(chroma(self.colour[occ] / self.hits[occ][:, None]), self.means), axis=-1)
        return out

    def _kind(self, at):
        """(the kind of the voxels `at` (index arrays) by their pooled colour, its distance from each kind's
        lit line)."""
        d = lit(chroma(self.colour[at].sum(axis=0)), self.means)
        return int(np.argmin(d)), d

    def things(self):
        """([(kind, centre (3), half (3), R (3, 3), id, clear) each surface and each structure], labels
        (nx, ny, nz) each voxel's thing, -1 none); made once a look."""
        if self._made is not None:
            return self._made
        out, labels, surfaces = self._segmented()
        ids = self._tracked(labels, len(out))
        self._labels, self._ids = labels, ids
        self._made = ([t + (i, float(k < surfaces)) for k, (t, i) in enumerate(zip(out, ids))], labels)
        return self._made

    def _tracked(self, labels, n):
        """Each of the `n` things' id: the one of the thing at the look before sharing most of its voxels,
        each id given once, the rest new."""
        ids = [0] * n
        if self._labels is not None:
            both = (labels >= 0) & (self._labels >= 0)
            pair, count = np.unique(labels[both].astype(np.int64) * len(self._ids) + self._labels[both], return_counts=True)
            taken = set()
            for p in pair[np.argsort(-count, kind='stable')]:
                new, old = divmod(int(p), len(self._ids))
                if ids[new] == 0 and self._ids[old] not in taken:
                    ids[new], taken = self._ids[old], taken | {self._ids[old]}
        for k in range(n):
            if ids[k] == 0:
                ids[k], self._next = self._next, self._next + 1
        return ids

    def _segmented(self):
        """([(kind, centre, half, R) each surface, then each structure], labels, the surfaces' count)."""
        occ, kind = self.occupied(), self.kinds()
        stood = np.zeros_like(occ)
        for di in (-1, 0, 1):
            for dk in (-1, 0, 1):
                stood[max(0, -di):self.nx - max(0, di), :-1, max(0, -dk):self.nz - max(0, dk)] |= occ[
                    max(0, di):self.nx - max(0, -di), 1:, max(0, dk):self.nz - max(0, -dk)]
        above = np.zeros_like(occ)
        above[:, :-1] = self.free[:, 1:] > 0
        top = occ & ~stood & above
        labels, out = np.full(occ.shape, -1), []
        for j in range(self.ny):
            for i, k in _joined(top[:, j], _BESIDE):
                labels[i, j, k] = len(out)
                out.append(self._box(i, k, j, j, self._kind((i, np.full(len(i), j), k))[0]))
        surfaces = len(out)
        # each voxel's nearest surface above it in its column, a voxel round: what holds a seat up is under
        # it, a back on the seat under its own top, a wall under none
        over, run = np.full(occ.shape, -1), np.full((self.nx, self.nz), -1)
        for j in reversed(range(self.ny)):
            over[:, j, :] = run
            run = np.where(grown(top[:, j, :], 1), j, run)
        for level in np.unique(over[occ & ~top]):
            for group in _joined(occ & ~top & (over == level), _AROUND):
                parent, _d = self._kind(group)
                pixel, own = self._pixel(group, parent), np.ones(len(group[0]), bool)
                other = np.zeros_like(occ)
                other[group] = kind[group] != parent
                for part in _joined(other, _AROUND):
                    if self._apart(pixel, parent, part):
                        self._told(part, out, labels)
                        own &= labels[group] < 0
                rest = tuple(a[own] for a in group)
                # what rests on nothing - above the floor's layer under no surface - is a stray, not a thing
                if len(rest[0]) and (level >= 0 or rest[1].min() == 0):
                    self._told(rest, out, labels, parent)
        return out, labels, surfaces

    def _told(self, at, out, labels, kind=None):
        """A structure's voxels `at` (index arrays) onto `out` in its pieces (`_pieces`), each voxel's piece in
        `labels`; each piece's kind `kind`, or its own pooled colour's."""
        for piece in self._pieces(at):
            labels[piece] = len(out)
            out.append(self._box(piece[0], piece[2], int(piece[1].min()), int(piece[1].max()),
                                 self._kind(piece)[0] if kind is None else kind))

    def _pieces(self, at):
        """A structure's voxels `at` (index arrays) in pieces, each halved across its footprint's long axis at
        its mean till its box (`_box`) claims no floor further than a voxel from the cells its voxels stand
        in: whole, the lodge's walls joined at its corners were one box 1.8-3.1 m off each wall. A surface
        is told whole: its holes are what her eyes missed of a face seen from above - held to its cells, a
        bed her stereo saw in patches came in pieces."""
        out, todo = [], [np.arange(len(at[0]))]
        while todo:
            n = todo.pop()
            ik = np.column_stack([at[0][n], at[2][n]]).astype(float)
            ik -= ik.mean(axis=0)
            _w, v = np.linalg.eigh(ik.T @ ik)
            along, across = ik @ v[:, 1], ik @ v[:, 0]
            cells = np.unique((at[0][n] + 1) * (self.nz + 2) + at[2][n] + 1)
            near = np.unique((cells[:, None] + (_ROUND[:, 0] * (self.nz + 2) + _ROUND[:, 1])[None, :]).ravel())
            if len(cells) > 1 and (np.ptp(along) + 1.0) * (np.ptp(across) + 1.0) > len(near) + 1e-9:
                todo += [n[along >= 0.0], n[along < 0.0]]
            else:
                out.append(tuple(a[n] for a in at))
        return out

    def _pixel(self, group, parent):
        """A structure `group`'s (index arrays) of kind `parent` pixels' spread from the kind's lit line: its
        voxels' chroma distance's deviation over the root of a voxel's points."""
        voxel = lit(chroma(self.colour[group] / np.maximum(self.hits[group], 1)[:, None]), self.means[parent:parent + 1])[:, 0]
        return float(voxel.std() * math.sqrt(max(1.0, float(np.median(self.hits[group])))))

    def _apart(self, pixel, parent, part):
        """Whether `part` (index arrays) of a structure of kind `parent` is a thing of its own: its pooled
        chroma's distance from the parent kind's lit line, over the standard error the structure's pixels
        give it (`pixel`, `_pixel`'s), past SIGMA."""
        _k, d = self._kind(part)
        return d[parent] * math.sqrt(float(self.hits[part].sum())) > SIGMA * max(pixel, 1e-9)

    def _box(self, i, k, j_lo, j_hi, kind):
        """A thing's box of the cells (i, k) its voxels stand in, levels j_lo to j_hi: its footprint's
        principal axes its turn, its extents along them a voxel wide, its height its levels'."""
        at = np.column_stack([X_M[0] + (i + 0.5) * self.voxel, Z_M[0] + (k + 0.5) * self.voxel])
        mean = at.mean(axis=0)
        cov = (at - mean).T @ (at - mean) / len(at)
        w, v = np.linalg.eigh(cov)
        a, b = v[:, 1], v[:, 0]
        along, across = (at - mean) @ a, (at - mean) @ b
        c = mean + 0.5 * (along.max() + along.min()) * a + 0.5 * (across.max() + across.min()) * b
        half = (0.5 * (along.max() - along.min()) + 0.5 * self.voxel, 0.5 * (across.max() - across.min()) + 0.5 * self.voxel)
        lo, hi = FLOOR_M + j_lo * self.voxel, FLOOR_M + (j_hi + 1) * self.voxel
        R = np.array([[a[0], 0.0, b[0]], [0.0, 1.0, 0.0], [a[1], 0.0, b[1]]])
        return (self.names[kind], (float(c[0]), 0.5 * (lo + hi), float(c[1])), (half[0], 0.5 * (hi - lo), half[1]), R)

    def blocked(self):
        """The floor's cells she may not stand in: a voxel of a thing within STAND_M over the floor, a
        door's not, grown by her radius; made once a look."""
        things, labels = self.things()
        j = np.arange(self.ny)
        band = (FLOOR_M + (j + 1) * self.voxel > STAND_M[0]) & (FLOOR_M + j * self.voxel < STAND_M[1])
        door = np.array([t[0] == 'door' for t in things] + [False])
        hit = ((labels >= 0) & ~door[labels] & band[None, :, None]).any(axis=1)
        return grown(hit, max(1, int(math.ceil(RADIUS_M / self.voxel))))


def grown(cells, r, join=np.logical_or):
    """`cells` (nx, nz) grown by `r` of them round each, `join` of a cell and each neighbour's."""
    out = cells.copy()
    nx, nz = cells.shape
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            if di * di + dj * dj > r * r or (di == 0 and dj == 0):
                continue
            src = cells[max(0, -di):nx - max(0, di), max(0, -dj):nz - max(0, dj)]
            at = out[max(0, di):nx - max(0, -di), max(0, dj):nz - max(0, -dj)]
            at[...] = join(at, src)
    return out


def _joined(on, offsets):
    """The connected groups of the set voxels of `on` (any rank), neighbours by `offsets` (one of each
    pair): each a tuple of index arrays, union-find on the flat indices with pointer jumping."""
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
