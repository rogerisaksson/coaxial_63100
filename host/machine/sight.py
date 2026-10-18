"""Her eyes and what she makes of them. A stereo pair on her head rendered from the world, the
disparity by block matching, depth from it, the points into a map of the room she keeps, its
things told apart by shape and colour, and her way planned round them (docs/findings/sight.md).

    keep = Keep(world)                      # her eyes on a world and the map she keeps
    keep.look(world, now)                   # one look into the map, LOOK_S apart at most
    things = keep.map.things()              # [(kind, centre, half sizes, height, look)]
    way = keep.map.plan(at, to)             # [(x, z)] round what she knows
    keep.hulls()                            # what the last look showed, hulled by thing

Nothing here reads the world's geometry: the cameras' images and poses alone (`physics.SIGHT`).
"""
import heapq
import math
import os

#: Her eyes: BASELINE_M apart on the head, EYES (x, y, z) in the head's frame the left one's,
#: looking along the head's z, DOWN_DEG under it - level, their view's lower edge 30 deg
#: down, a table's top was out of it nearer than 1.4 m and a chair's seat nearer than 1.9;
#: FOVY_DEG high, W x H pixels.
BASELINE_M, EYES, FOVY_DEG, W, H, DOWN_DEG = 0.1, (0.05, 0.10, 0.09), 60.0, 320, 240, 15.0

#: Block matching: DISPARITIES tried, a WINDOW square summed; a pixel is kept with texture
#: (its window's spread over TEXTURE) and a match UNIQUE times better than the next best. The
#: table's top 1.6-2.3 m off, looked down on, spreads 2.2 at the median: at 4.0 a quarter of
#: it was kept, at 1.5 0.70, 99 % of those within 1.5 px of the renderer's.
DISPARITIES, WINDOW, TEXTURE, UNIQUE = 48, 9, 1.5, 1.15

#: Her map: CELL_M squares over the room's floor (X_M, Z_M); a cell is a thing from TOLD looks
#: with HITS points in it - one look's mismatches are not a thing -
#: between FLOOR_M and CEIL_M high, NEAR_M to FAR_M from the eye - further the depth is a
#: tenth out, nearer is her own raised hand -, hit more often than seen through under its
#: top; her own radius, inflating what she plans round. What she has not seen is open: her way is planned
#: through it and again as she sees (`barred`) - held shut, a target unseen had no way and
#: every first word a look round first, 14 s on 23 steps. A look LOOK_S apart.
CELL_M, X_M, Z_M = 0.1, (-3.5, 3.5), (-2.0, 5.0)
HITS, TOLD, FLOOR_M, CEIL_M, NEAR_M, FAR_M, RADIUS_M, LOOK_S = 3, 2, 0.08, 2.3, 0.7, 2.5, 0.3, 0.5

#: What the room's things are made to look like, for her eyes (`mjcf`): a kind's colour,
#: speckled with a darker shade, the speckles a cube's on every face and one scale in space,
#: the pattern SPECKLE_M long: a checker repeats, and so did speckles laid 12 a face - on the
#: door 7 cm apart, 4.5 px at 3.4 m - and block matching matched one to the next: a thing
#: 1.2 m high stood 1 m before her where nothing is. A lump's look is the kind nearest its
#: cells' chromaticity (`_chroma`), whatever the light on it.
SPECKLE_M = 4.0
LOOKS = {'glass': ((0.55, 0.75, 1.0), (0.35, 0.5, 0.7)),
         'wood': ((0.6, 0.38, 0.2), (0.35, 0.2, 0.1)),
         'bed': ((0.3, 0.42, 0.85), (0.15, 0.22, 0.5)),
         'door': ((0.55, 0.2, 0.12), (0.3, 0.1, 0.06)),
         'lamp': ((0.95, 0.85, 0.3), (0.6, 0.52, 0.15)),
         'switch': ((1.0, 1.0, 1.0), (0.6, 0.6, 0.6))}
ALPHA = {'glass': 0.7}

#: The page's picture of her view, pixels wide and high (`coaxial.graphics.inset`).
VIEW = (32, 24)
#: Each kind's ink for the page, (r, g, b).
INK = {'wall': (120, 190, 255), 'door': (255, 120, 90), 'lamp': (255, 230, 90),
       'bed': (110, 140, 255), 'table': (230, 170, 100), 'chair': (255, 190, 120),
       'thing': (200, 200, 200)}


def _np():
    os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
    import numpy
    return numpy


def assets():
    """The `<asset>` block, a speckled texture and a material a kind, and the model's extent
    set to the room: the cameras clip nearer than a hundredth of it, and the floor's parked
    patches made it 10 km."""
    out = ['<statistic extent="8" center="0 1 1"/>', '<asset>']
    for kind, (a, b) in LOOKS.items():
        out.append('<texture name="%s" type="cube" builtin="flat" rgb1="%g %g %g" mark="random" '
                   'markrgb="%g %g %g" random="0.35" width="128" height="128"/>' % ((kind,) + a + b))
        out.append('<material name="%s" texture="%s" texuniform="true" texrepeat="%g %g" '
                   'rgba="1 1 1 %g"/>' % (kind, kind, 1.0 / SPECKLE_M, 1.0 / SPECKLE_M,
                                         ALPHA.get(kind, 1.0)))
    return out + ['</asset>']


def lights():
    """The room lit from its ceiling, for her eyes: the model has no light of its own."""
    return ['<light pos="%g 2.2 %g" dir="0 -1 0" diffuse="0.9 0.9 0.9" specular="0.2 0.2 0.2"/>'
            % at for at in ((-1.5, 0.0), (1.5, 0.0), (-1.5, 2.5), (1.5, 2.5))]


def cameras():
    """Her eyes in the head's body: each looks along the head's z, DOWN_DEG under it, x to
    her right."""
    x, y, z = EYES
    up = (math.cos(math.radians(DOWN_DEG)), math.sin(math.radians(DOWN_DEG)))
    return ['<camera name="eye_%s" pos="%g %g %g" xyaxes="-1 0 0 0 %g %g" fovy="%g"/>'
            % ((side, sx, y, z) + up + (FOVY_DEG,))
            for side, sx in (('left', x), ('right', x - BASELINE_M))]


def focal():
    """The focal length, px."""
    return (H / 2.0) / math.tan(math.radians(FOVY_DEG) / 2.0)


class Seen:

    """One look: both eyes' images (H, W, 3) uint8 and the left eye's pose, world."""

    def __init__(self, left, right, at, turn):
        self.left, self.right, self.at, self.turn = left, right, at, turn


class Eyes:

    """The renderer on a world's model, both cameras."""

    def __init__(self, world):
        import mujoco
        self.renderer = mujoco.Renderer(world.model, H, W)
        self.ids = {side: world.model.camera('eye_%s' % side).id for side in ('left', 'right')}

    def see(self, world):
        d = world.data
        shots = {}
        for side in ('left', 'right'):
            self.renderer.update_scene(d, camera=self.ids[side])
            shots[side] = self.renderer.render().copy()
        i = self.ids['left']
        return Seen(shots['left'], shots['right'], d.cam_xpos[i].copy(),
                    d.cam_xmat[i].reshape(3, 3).copy())

    def truth(self, world):
        """The left eye's depth from the renderer's own buffer, m: the measure to hold the
        stereo to, never hers."""
        self.renderer.enable_depth_rendering()
        try:
            self.renderer.update_scene(world.data, camera=self.ids['left'])
            return self.renderer.render().copy()
        finally:
            self.renderer.disable_depth_rendering()


def _gray(rgb):
    np = _np()
    return rgb.astype(np.float32) @ np.array([0.299, 0.587, 0.114], dtype=np.float32)


def _boxed(a, r):
    """`a` summed over (2r+1)^2 windows, edges clipped."""
    np = _np()
    p = np.pad(a, r, mode='edge')
    c = np.cumsum(np.cumsum(p, axis=0), axis=1)
    c = np.pad(c, ((1, 0), (1, 0)))
    k = 2 * r + 1
    return c[k:, k:] - c[:-k, k:] - c[k:, :-k] + c[:-k, :-k]


def disparity(left, right):
    """(disparity px, kept) of the left image against the right: the best SAD a pixel over
    DISPARITIES, refined by the parabola through its neighbours; kept where textured and the
    best UNIQUE times under the next best."""
    np = _np()
    gl, gr = _gray(left), _gray(right)
    r = WINDOW // 2
    cost = np.full((DISPARITIES, H, W), np.inf, dtype=np.float32)
    for d in range(DISPARITIES):
        diff = np.abs(gl[:, d:] - gr[:, :W - d])
        cost[d, :, d:] = _boxed(diff, r)
    best = np.argmin(cost, axis=0)
    rows, cols = np.indices((H, W))
    c0 = cost[best, rows, cols]
    lo = cost[np.maximum(best - 1, 0), rows, cols]
    hi = cost[np.minimum(best + 1, DISPARITIES - 1), rows, cols]
    denom = lo - 2.0 * c0 + hi
    curved = np.isfinite(denom) & (denom > 1e-6)
    sub = np.zeros_like(c0)
    sub[curved] = 0.5 * (lo[curved] - hi[curved]) / denom[curved]
    disp = best + np.clip(sub, -0.5, 0.5)
    # the next best away from the best's own neighbours: next to it the cost is its own
    apart = np.abs(np.arange(DISPARITIES)[:, None, None] - best[None, :, :]) > 2
    second = np.where(apart, cost, np.inf).min(axis=0)
    mean = _boxed(gl, r) / WINDOW ** 2
    spread = np.sqrt(np.maximum(_boxed(gl * gl, r) / WINDOW ** 2 - mean * mean, 0.0))
    # the right eye's own best match from the same costs: a left pixel is kept where the
    # right pixel it matched picks it back, within a pixel - what one eye alone sees, and a
    # match to the wrong speckle, do not
    back = np.full_like(cost, np.inf)
    for d in range(DISPARITIES):
        back[d, :, :W - d] = cost[d, :, d:]
    agreed = np.abs(np.argmin(back, axis=0)[rows, np.clip(cols - best, 0, W - 1)] - best) <= 1
    kept = ((spread > TEXTURE) & (second > UNIQUE * np.maximum(c0, 1.0)) & (best > 0)
            & (best < DISPARITIES - 1) & np.isfinite(c0) & agreed)
    kept[:, :DISPARITIES] = False
    return disp, kept


def depth(disp):
    """Depth along the eye's axis, m, from a disparity."""
    return focal() * BASELINE_M / _np().maximum(disp, 1e-3)


def points(seen, disp, kept, every=2):
    """World (x, y, z), colours (r, g, b) and pixels (row, col) of the kept pixels, every
    `every`th row and column."""
    np = _np()
    f = focal()
    rows, cols = np.indices((H, W))
    sel = kept & (rows % every == 0) & (cols % every == 0)
    z = depth(disp[sel])
    x = (cols[sel] - W / 2.0) / f * z
    y = -(rows[sel] - H / 2.0) / f * z
    cam = np.stack([x, y, -z], axis=1)
    return cam @ seen.turn.T + seen.at, seen.left[sel].astype(np.float32) / 255.0, rows[sel], cols[sel]


class Look:

    """What one look put into the map: the images and, of the points kept, their pixels and
    cells."""

    def __init__(self, seen, rows, cols, cells):
        self.seen, self.rows, self.cols, self.cells = seen, rows, cols, cells


class Map:

    """What she knows of the room's floor: a cell's hits, looks through it, its highest point
    and its colour."""

    def __init__(self):
        np = _np()
        self.nx = int(round((X_M[1] - X_M[0]) / CELL_M))
        self.nz = int(round((Z_M[1] - Z_M[0]) / CELL_M))
        self.hits = np.zeros((self.nx, self.nz), dtype=np.int32)
        self.told = np.zeros((self.nx, self.nz), dtype=np.int32)
        self.free = np.zeros((self.nx, self.nz), dtype=np.int32)
        self.high = np.zeros((self.nx, self.nz), dtype=np.float32)
        self.colour = np.zeros((self.nx, self.nz, 3), dtype=np.float32)
        self.looks, self._at, self._grown = 0, -1, None

    def cell(self, x, z):
        return (int(math.floor((x - X_M[0]) / CELL_M)), int(math.floor((z - Z_M[0]) / CELL_M)))

    def centre(self, i, j):
        return (X_M[0] + (i + 0.5) * CELL_M, Z_M[0] + (j + 0.5) * CELL_M)

    def add(self, seen):
        """One look into the map: the points standing between FLOOR_M and CEIL_M, NEAR_M to
        FAR_M from the eye; the cells seen through, free. The Look."""
        np = _np()
        disp, kept = disparity(seen.left, seen.right)
        xyz, rgb, rows, cols = points(seen, disp, kept)
        away = np.linalg.norm(xyz - seen.at, axis=1)
        ok = (xyz[:, 1] > FLOOR_M) & (xyz[:, 1] < CEIL_M) & (away > NEAR_M) & (away < FAR_M)
        xyz, rgb, rows, cols = xyz[ok], rgb[ok], rows[ok], cols[ok]
        self._seen_through(seen.at, xyz)
        i = np.floor((xyz[:, 0] - X_M[0]) / CELL_M).astype(int)
        j = np.floor((xyz[:, 2] - Z_M[0]) / CELL_M).astype(int)
        inside = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.nz)
        i, j, y, rgb = i[inside], j[inside], xyz[inside, 1], rgb[inside]
        np.add.at(self.hits, (i, j), 1)
        self.told += np.bincount(i * self.nz + j, minlength=self.nx * self.nz).reshape(
            self.nx, self.nz) >= HITS
        np.maximum.at(self.high, (i, j), y.astype(np.float32))
        np.add.at(self.colour, (i, j), rgb)
        self.looks += 1
        return Look(seen, rows[inside], cols[inside], np.stack([i, j], axis=1))

    def _seen_through(self, eye, xyz, step=0.08):
        """The cells each ray crossed short of its point, a cell under what stood highest in
        it, counted free: a ray over a table to the wall behind says nothing of the table."""
        np = _np()
        d = xyz[:, [0, 2]] - eye[[0, 2]]
        far = np.linalg.norm(d, axis=1)
        n = int(np.ceil(far.max() / step)) if len(far) else 0
        if n == 0:
            return
        t = (np.arange(n) * step)[None, :]
        on = t < (far[:, None] - CELL_M)
        k = t / np.maximum(far[:, None], 1e-6)
        i = np.floor((eye[0] + d[:, 0:1] * k - X_M[0])[on] / CELL_M).astype(int)
        j = np.floor((eye[2] + d[:, 1:2] * k - Z_M[0])[on] / CELL_M).astype(int)
        y = (eye[1] + (xyz[:, 1:2] - eye[1]) * k)[on]
        inside = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.nz)
        i, j, y = i[inside], j[inside], y[inside]
        under = y < self.high[i, j] - CELL_M
        self.free.flat[np.unique(i[under] * self.nz + j[under])] += 1

    def occupied(self):
        """The cells TOLD looks have shown a thing in, more than have seen through them."""
        return (self.told >= TOLD) & (self.told > self.free)

    def lumps(self):
        """[cells] the occupied cells of one look joined four ways: a door's leaf is a lump
        apart from the wall it stands in."""
        np = _np()
        occ = self.occupied()
        means = np.array([_chroma(np.array(a)) for a, _b in LOOKS.values()])
        mean = _chroma(self.colour / np.maximum(self.hits, 1)[:, :, None])
        look = np.argmin(((mean[:, :, None, :] - means[None, None, :, :]) ** 2).sum(axis=3), axis=2)
        seen = np.zeros_like(occ)
        out = []
        for i, j in zip(*np.nonzero(occ)):
            if seen[i, j]:
                continue
            cells, todo = [], [(int(i), int(j))]
            seen[i, j] = True
            while todo:
                a, b = todo.pop()
                cells.append((a, b))
                for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    p, q = a + da, b + db
                    if (0 <= p < self.nx and 0 <= q < self.nz and occ[p, q] and not seen[p, q]
                            and look[p, q] == look[i, j]):
                        seen[p, q] = True
                        todo.append((p, q))
            out.append(cells)
        return out

    def things(self, lumps=None):
        """[(kind, (x, z), (half x, half z), height, look)]: each lump told by its size, its
        height and its look."""
        np = _np()
        names = list(LOOKS)
        means = np.array([_chroma(np.array(a)) for a, _b in LOOKS.values()])
        out = []
        for cells in self.lumps() if lumps is None else lumps:
            ii = np.array([c[0] for c in cells])
            jj = np.array([c[1] for c in cells])
            lo, hi = self.centre(ii.min(), jj.min()), self.centre(ii.max(), jj.max())
            half = ((hi[0] - lo[0] + CELL_M) / 2.0, (hi[1] - lo[1] + CELL_M) / 2.0)
            colour = _chroma(self.colour[ii, jj].sum(axis=0) / max(1, int(self.hits[ii, jj].sum())))
            look = names[int(np.argmin(((means - colour) ** 2).sum(axis=1)))]
            height = float(np.percentile(self.high[ii, jj], 95))
            kind = classify(look, half, height)
            at = ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0)
            if kind == 'bed':
                half = (max(half[0], 0.45), max(half[1], 1.0))
                at = (hi[0] + CELL_M / 2.0 - half[0] if at[0] < 0.0 else lo[0] - CELL_M / 2.0 + half[0],
                      lo[1] - CELL_M / 2.0 + half[1] if at[1] > 0.0 else hi[1] + CELL_M / 2.0 - half[1])
            out.append((kind, at, half, height, look))
        return out

    def blocked(self):
        """The cells she may not stand in: the occupied ones but a door's, grown by her
        radius; made once a look."""
        if self._grown is not None and self._at == self.looks:
            return self._grown
        r = int(math.ceil(RADIUS_M / CELL_M))
        occ = self.occupied().copy()
        lumps = self.lumps()
        for cells, thing in zip(lumps, self.things(lumps)):
            if thing[0] == 'door':
                for i, j in cells:
                    occ[i, j] = False
        grown = occ.copy()
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj > r * r or (di == 0 and dj == 0):
                    continue
                src = occ[max(0, -di):self.nx - max(0, di), max(0, -dj):self.nz - max(0, dj)]
                grown[max(0, di):self.nx - max(0, -di), max(0, dj):self.nz - max(0, -dj)] |= src
        self._at, self._grown = self.looks, grown
        return grown

    def barred(self, points):
        """Whether the line through `points` [(x, z), ..] crosses what she knows, her radius
        round it."""
        blocked = self.blocked()
        for a, b in zip(points, points[1:]):
            n = max(1, int(math.dist(a, b) / (CELL_M / 2.0)))
            for k in range(n + 1):
                i, j = self.cell(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                if 0 <= i < self.nx and 0 <= j < self.nz and blocked[i, j]:
                    return True
        return False

    def plan(self, at, to):
        """[(x, z)] from `at` to `to` round what she knows, the corners alone; `to` in a
        thing's reach, the nearest cell out of it; [] with no way or `to` off the map."""
        blocked = self.blocked()
        start, goal = self.cell(*at), self.cell(*to)
        if not all(0 <= c[0] < self.nx and 0 <= c[1] < self.nz for c in (start, goal)):
            return []
        if blocked[goal]:
            goal = _nearest_free(blocked, goal)
        if blocked[start]:
            start = _nearest_free(blocked, start)
        path = _astar(blocked, start, goal)
        if not path:
            return []
        return [self.centre(*c) for c in _pruned(blocked, path)[1:]]


class Keep:

    """Her eyes on a world and the map she keeps with them: `look` at most LOOK_S apart; the
    last look kept for the page."""

    def __init__(self, world):
        self.eyes, self.map, self.looked_at, self.last = Eyes(world), Map(), None, None

    def look(self, world, now):
        """One look into the map, if LOOK_S have passed since the last; the Look, or None."""
        if self.looked_at is not None and now - self.looked_at < LOOK_S:
            return None
        self.looked_at = now
        self.last = self.map.add(self.eyes.see(world))
        return self.last

    def view(self):
        """(thumbnail, hulls) of the last look for the page, VIEW pixels; None before one."""
        if self.last is None:
            return None
        return thumbnail(self.last.seen.left, VIEW[0], VIEW[1]), self.hulls()

    def hulls(self):
        """What the last look showed, by thing: [(kind, ink, [(col, row), ..] its hull in the
        left eye's image)], the things she has told from her map."""
        if self.last is None:
            return []
        lumps = self.map.lumps()
        of = {cell: k for k, cells in enumerate(lumps) for cell in cells}
        kinds = [t[0] for t in self.map.things(lumps)]
        by = {}
        for row, col, (i, j) in zip(self.last.rows, self.last.cols, self.last.cells):
            k = of.get((int(i), int(j)))
            if k is not None:
                by.setdefault(k, []).append((int(col), int(row)))
        out = []
        for k, pts in by.items():
            hull = _hull(pts)
            if len(hull) >= 3:
                out.append((kinds[k], INK.get(kinds[k], INK['thing']), hull))
        return out


def thumbnail(rgb, w, h):
    """`rgb` (H, W, 3) shrunk to (h, w, 3) by block means."""
    np = _np()
    hh, ww = rgb.shape[0] // h * h, rgb.shape[1] // w * w
    return rgb[:hh, :ww].reshape(h, hh // h, w, ww // w, 3).mean(axis=(1, 3)).astype(np.uint8)


def _hull(pts):
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


def _chroma(rgb):
    """A colour's chromaticity, its share of each of r, g, b."""
    return rgb / _np().maximum(rgb.sum(axis=-1, keepdims=True), 1e-6)


def classify(look, half, height):
    """A lump's kind from its footprint and its height first, its look where those do not
    tell: walls and doors stand over 1.8 m, a lamp is thin, a bed long and low, a table
    waist-high and long, a chair waist-high and short - a chair's lump 0.5 m, the table's top
    as seen from the room's middle 0.8."""
    long = max(half) * 2.0
    if height >= 1.8:
        return 'door' if look == 'door' and long <= 1.2 else 'wall'
    if long <= 0.3 and height >= 1.2:
        return 'lamp'
    if long >= 1.6 and height <= 0.7:
        return 'bed'
    if 0.6 <= height <= 1.1:
        return 'table' if long >= 0.65 else 'chair'
    return look if look in ('bed', 'lamp') else 'thing'


def _nearest_free(blocked, cell):
    np = _np()
    free = np.argwhere(~blocked)
    if len(free) == 0:
        return cell
    k = np.argmin(((free - np.array(cell)) ** 2).sum(axis=1))
    return (int(free[k][0]), int(free[k][1]))


def _astar(blocked, start, goal):
    nx, nz = blocked.shape
    root2 = math.sqrt(2.0)
    steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, root2), (1, -1, root2), (-1, 1, root2), (-1, -1, root2)]
    came, cost = {start: None}, {start: 0.0}
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
            if not (0 <= nxt[0] < nx and 0 <= nxt[1] < nz) or blocked[nxt]:
                continue
            if w > 1.0 and (blocked[here[0] + di, here[1]] or blocked[here[0], here[1] + dj]):
                continue
            c = cost[here] + w
            if c < cost.get(nxt, math.inf):
                cost[nxt], came[nxt] = c, here
                heapq.heappush(todo, (c + math.dist(nxt, goal), nxt))
    return []


def _clear(blocked, a, b):
    """Whether the straight line of cells from `a` to `b` is free."""
    n = max(abs(b[0] - a[0]), abs(b[1] - a[1]), 1)
    for k in range(n + 1):
        i = int(round(a[0] + (b[0] - a[0]) * k / n))
        j = int(round(a[1] + (b[1] - a[1]) * k / n))
        if blocked[i, j]:
            return False
    return True


def _pruned(blocked, path):
    """The path's corners: each the furthest cell seen straight from the one before."""
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _clear(blocked, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out
