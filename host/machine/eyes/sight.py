"""Her eyes and what she makes of them. A stereo pair on her head rendered from the world, the
disparity by block matching, depth from it, the points into the map she keeps of the room
(`machine.eyes.voxels`) in her estimate's frame, its things boxes with their turn (docs/findings/sight.md).

    eyes, keep = Eyes(world), Keep()        # her eyes on a world, the map she keeps
    keep.took(*eyes.stereo(world, head))    # a look matched by her stereo into the map, from her head's pose
    keep.took(*eyes.shot(world, head))      # a look's parallax from the renderer's depth (`machine.eyes.camera`)
    things = told(keep)                     # [(kind, centre, half sizes, turn)] her registry's looks
    way = plan(keep.map.blocked(), at, to)  # [(x, z)] round what she knows
    keep.hulls()                            # what the last look showed, hulled by what she tells of it

Her keep reads nothing of the world: a look's frame - an eye's colour and parallax at her own pose - alone,
rendered on the world's side (`physics.SIGHT`).
"""
import atexit
import heapq
import math
import weakref

from machine.eyes.stereo import BASELINE_M, FOVY_DEG, H, W, _np
from machine.eyes.voxels import CELL_M, X_M, Z_M
from machine.eyes import epipolar, gimbal, instances, squint, voxels

#: A look's points every EVERY'th pixel each way: at 640 x 480 her map's 19 200 points a look as at 320 x 240
#: every second - its cost theirs, 45 ms a look against 161 every second -, each point's depth twice as fine.
EVERY = 4

#: A look LOOK_S apart. What she has not seen is open: her way is planned through it and again as she
#: sees - held shut, a target unseen had no way and every first word a look round first, 14 s on 23
#: steps.
LOOK_S = 0.5
#: How far a look's pixels are kept (`Eyes.shot`), across her map's floor (X_M, Z_M), m: her map takes those within
#: FAR_M (`voxels.Map.add`), her picture tells the rest by their look (`Keep.hulls`).
REACH_M = math.hypot(X_M[1] - X_M[0], Z_M[1] - Z_M[0])

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
         'switch': ((1.0, 1.0, 1.0), (0.6, 0.6, 0.6)),
         'timber': ((0.27, 0.22, 0.20), (0.14, 0.11, 0.10)), 'roof': ((0.93, 0.95, 0.98), (0.75, 0.78, 0.82)),
         'stone': ((0.58, 0.56, 0.52), (0.36, 0.35, 0.33)), 'rail': ((0.72, 0.75, 0.80), (0.45, 0.47, 0.50))}
ALPHA = {'glass': 0.7}

#: The page's picture of her view, pixels wide and high (`coaxial.graphics.inset`); a lump of one look smaller than a
#: cell of it - her points every EVERY'th pixel each way - her picture leaves untold, points.
VIEW = (32, 24)
LUMP = (W // VIEW[0]) * (H // VIEW[1]) // EVERY ** 2
#: Each class's ink for the page, (r, g, b) (`classes`); the classes under and over all else, one region each.
INK = {'wall': (120, 190, 255), 'window': (160, 235, 255), 'post': (140, 160, 210), 'door': (255, 120, 90),
       'railing': (175, 185, 205), 'roof': (235, 240, 250), 'ceiling': (210, 215, 235), 'floor': (150, 120, 90),
       'stairs': (215, 155, 95), 'step': (215, 155, 95), 'platform': (195, 145, 95), 'lamp': (255, 230, 90),
       'switch': (240, 240, 240), 'bed': (110, 140, 255), 'sofa': (200, 120, 160), 'table': (230, 170, 100),
       'chair': (255, 190, 120), 'bench': (245, 175, 115), 'stool': (255, 175, 140), 'cabinet': (180, 140, 110),
       'shelf': (205, 165, 125), 'veranda': (205, 160, 110), 'terrace': (185, 175, 160), 'ground': (125, 140, 115),
       'cabin': (255, 215, 170), 'house': (255, 215, 170), 'thing': (200, 200, 200)}
SPREAD = ('floor', 'ceiling', 'ground')


def assets():
    """The `<asset>` block, a speckled texture and a material a kind, and the model's extent
    set to the room: the cameras clip nearer than a hundredth of it, and the floor's parked
    patches made it 10 km."""
    out = ['<statistic extent="8" center="0 1 1"/>', '<visual><global offwidth="%d" offheight="%d"/></visual>'
           % (epipolar.FINE * W, epipolar.FINE * H), '<asset>']
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


def focal():
    """The focal length, px."""
    return (H / 2.0) / math.tan(math.radians(FOVY_DEG) / 2.0)


def eye_pose(head, turn=None, at=gimbal.EYES):
    """(the left eye's place `at`, the frame `turn` (3, 3) - her eye's at rest by default -, both in her head's) in
    her `head`'s {pos, quat} frame."""
    np = _np()
    w, x, y, z = head['quat']
    R = np.array([[1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
                  [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
                  [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)]])
    return np.asarray(head['pos'], float) + R @ np.asarray(at, float), R @ (gimbal.mounted() if turn is None else turn)


class Seen:

    """One look: both eyes' images (H, W, 3) uint8 and the left eye's pose - the renderer's, till
    `Keep.look` lays hers -, its pictures' lay (`epipolar.Laid`'s beta, fv, baseline)."""

    def __init__(self, left, right, at, turn, lay=None):
        self.left, self.right, self.at, self.turn = left, right, at, turn
        self.lay = (0.0, focal(), BASELINE_M) if lay is None else lay


def _closed(eyes):
    """At the process's exit, eyes still alive closed (`Eyes.close`), before EGL's own end: registered after it, a
    weak reference - one closed as it went held nothing."""
    alive = eyes()
    if alive is not None:
        alive.close()


class Eyes:

    """The renderer on a world's model, both cameras; closed at its process's exit (`close`)."""

    def __init__(self, world):
        import mujoco
        self.renderer = mujoco.Renderer(world.model, epipolar.FINE * H, epipolar.FINE * W)
        self.ids = {side: world.model.camera('eye_%s' % side).id for side in ('left', 'right')}
        self.squint, self.laid, self.held = squint.Squint(), None, {'left': (0.0, 0.0), 'right': (0.0, 0.0)}
        atexit.register(_closed, weakref.ref(self))

    def close(self):
        """Its renderer's GL context let go while GL is there: left to its deallocator, it was freed after EGL had
        ended and raised an EGLError at the interpreter's end (CI's runner, MUJOCO_GL=egl, Python 3.14). Its own close
        frees what is there once, so its deallocator after it frees nothing."""
        self.renderer.close()

    def see(self, world):
        d = world.data
        shots = {}
        for side in ('left', 'right'):
            self.renderer.update_scene(d, camera=self.ids[side])
            shots[side] = self.renderer.render().copy()
        i = self.ids['left']
        return Seen(shots['left'], shots['right'], d.cam_xpos[i].copy(),
                    d.cam_xmat[i].reshape(3, 3).copy())

    def stereo(self, world, head, gz=(0.0, 0.0), held=None):
        """A look's frame (seen, disparity, kept) at her servos' gaze `gz`: both eyes' pictures laid by their turns and
        places as her calibration told her them - each gimbal where its pushrods' play leaves it by how she turned them
        (`held`: her own, each command's, `gaze.held`; else played from the last look's) -, her right eye's as her
        pictures told her past it (`squint.Squint`), and matched
        (`epipolar.matched`), told on by the look; at her `head` {pos, quat} as her estimate has it - on the CPU too: CI's
        runner (Mesa's frames) told no chair from the renderer's own depth (`shot`), 4.13 m off, and one from her match."""
        seen, start = self.see(world), None
        self.held = dict(held) if held is not None else {
            s: gimbal.played(self.held[s], gimbal.angles(s, gz, True), s, True) for s in ('left', 'right')}
        (lr, lc), (told, rc) = [(gimbal.eye(s, a, True), gimbal.place(s, a, True)) for s, a in self.held.items()]
        for _ in range(2):
            rr = told @ self.squint.turn()
            left, right, disp, kept, laid = epipolar.matched(seen.left, seen.right, lr, rr, rc - lc)
            start, again = self.squint.told(laid, left, right, disp, kept, lr, rr, rc - lc, gz, start)
            if not again:
                break
        self.laid = laid
        return Seen(left, right, *eye_pose(head, laid.E, lc), lay=(laid.beta, laid.fv, laid.B)), disp, kept

    def shot(self, world, head):
        """A look's frame (seen, disparity, kept) as her fake camera gives it (`machine.eyes.camera`): the left
        eye's colour and the parallax her stereo would match from the renderer's own depth (`truth`), every
        pixel within REACH_M kept, laid at her `head` {pos, quat} as her estimate has it."""
        np = _np()
        self.renderer.update_scene(world.data, camera=self.ids['left'])
        left, z = np.rint(_area(self.renderer.render())).astype(np.uint8), self.truth(world)
        kept = z < REACH_M
        at = gimbal.place('left', (0.0, 0.0), True)
        return (Seen(left, None, *eye_pose(head, gimbal.mounted('left', True), at)),
                np.where(kept, focal() * BASELINE_M / np.maximum(z, 1e-6), 0.0), kept)

    def truth(self, world):
        """The left eye's depth from the renderer's own buffer, m, each pixel its area's mean (`_area`): the
        measure to hold the stereo to, never hers."""
        self.renderer.enable_depth_rendering()
        try:
            self.renderer.update_scene(world.data, camera=self.ids['left'])
            return _area(self.renderer.render())
        finally:
            self.renderer.disable_depth_rendering()


def _area(a):
    """(H, W, ..) float: a finer picture's (`epipolar.FINE`) pixels' means, each of hers its area's."""
    f = epipolar.FINE
    return _np().asarray(a, float).reshape((H, f, W, f) + a.shape[2:]).mean(axis=(1, 3))


def depth(disp):
    """Depth along the eye's axis, m, from a disparity."""
    return focal() * BASELINE_M / _np().maximum(disp, 1e-3)


def points(seen, disp, kept, every=EVERY):
    """World (x, y, z), colours (r, g, b) and pixels (row, col) of the kept pixels, every
    `every`th row and column (`epipolar.points`)."""
    np = _np()
    rows, cols = np.indices((H, W))
    sel = kept & (rows % every == 0) & (cols % every == 0)
    cam = epipolar.points(rows[sel], cols[sel], disp[sel], *seen.lay)
    return cam @ seen.turn.T + seen.at, seen.left[sel].astype(np.float32) / 255.0, rows[sel], cols[sel]


class Look:

    """What one look put into the map: the images and, of the points kept, their pixels, voxels, places and colours."""

    def __init__(self, seen, rows, cols, at, xyz, rgb):
        self.seen, self.rows, self.cols, self.at, self.xyz, self.rgb = seen, rows, cols, at, xyz, rgb


class Keep:

    """The map she keeps (`voxels.Map` of LOOKS' colours) and her last look into it, kept for the page."""

    def __init__(self):
        self.map, self.last, self.objects = voxels.Map({k: a for k, (a, _b) in LOOKS.items()}), None, None

    def forget(self):
        """Her map laid anew, the last look let go: the room changed under her."""
        self.map, self.last, self.objects = voxels.Map(self.map.means_of), None, None

    def took(self, seen, disp, kept):
        """A look's frame (`Eyes.stereo`, `Eyes.shot`) into the map, its objects boxed (`instances`) where her
        segmenter is; the Look."""
        xyz, rgb, rows, cols = points(seen, disp, kept)
        self.last = Look(seen, rows, cols, self.map.add(xyz, rgb, seen.at), xyz, rgb)
        seg = instances.segmenter()
        self.objects = instances.boxes(seg(seen.left), self.last, self.map) if seg is not None else None
        return self.last

    def view(self):
        """(thumbnail, hulls) of the last look for the page, VIEW pixels; None before one."""
        if self.last is None:
            return None
        return thumbnail(self.last.seen.left, VIEW[0], VIEW[1]), self.hulls()

    def hulls(self):
        """What the last look showed by what she tells of it (`_told`): [(class, ink, [(col, row), ..] its hull in
        the left eye's image)], one round each thing's points - a window's panes one, SPREAD's each all one, a thing
        of under LUMP points none -, a building seen from outside it round all its own; what she cannot tell none."""
        if self.last is None:
            return []
        np, last = _np(), self.last
        told, one, housed, built = self._told(last)
        kinds, code = np.unique(told.astype(str), return_inverse=True)
        one = np.where(np.isin(kinds[code], SPREAD), -1, one)
        out = []
        for i, k in np.unique(np.column_stack([code, one]), axis=0).tolist():
            kind, at = str(kinds[i]), np.flatnonzero((code == i) & (one == k))
            hull = convex(list(zip(last.cols[at].tolist(), last.rows[at].tolist())))
            if kind != 'thing' and len(hull) >= 3 and len(at) >= LUMP:
                out.append((kind, INK.get(kind, INK['thing']), hull))
        for b, kind in enumerate(built):
            at = np.flatnonzero(housed == b)
            hull = convex(list(zip(last.cols[at].tolist(), last.rows[at].tolist())))
            if len(hull) >= 3:
                out.append((kind, INK.get(kind, INK['thing']), hull))
        return out

    def _told(self, last):
        """((n,) what she tells of each of the points of `last`, her last look - 'thing' what she cannot -, (n,) the one
        thing each is of - -1 none -, (n,) the building each is of seen from outside it - -1 none -, [each building's
        class]) of her map's things and the lumps of what it took none of past her own hands (`voxels.NEAR_M`), told at
        once (`classes.of`, `classes.grouped`): a
        point of her map's its thing's whole's class - a chair's back's and legs' its seat's -; a lump one look's points
        joined in the picture, those under her map's floor (`voxels.FLOOR_M`) in bands a cell apart and seen clear over,
        each told by its box - one at her floor running on to her picture's reach the ground -, one under LUMP points
        none; what rises past her map's ceiling within its reach is the top of what it holds, none."""
        from machine.eyes import classes
        np = _np()
        things, labels = self.map.things()
        kinds, centre, half, R, clear = classes.stacked(things)
        away = np.linalg.norm(last.xyz - last.seen.at, axis=1)
        off = np.flatnonzero((last.at < 0) & (away > voxels.NEAR_M)
                             & ((last.xyz[:, 1] < voxels.CEIL_M) | (away >= voxels.FAR_M)))
        xyz, low = last.xyz[off], last.xyz[off, 1] <= voxels.FLOOR_M
        look = np.argmin(voxels.lit(voxels.chroma(last.rgb[off]), self.map.means), axis=-1)
        band = np.where(low, np.round(xyz[:, 1] / voxels.CELL_M), 1.0)
        code = np.unique(np.column_stack([look, low, band]), axis=0, return_inverse=True)[1].reshape(-1)
        grid, index = np.full((H // EVERY + 1, W // EVERY + 1), -1), np.full((H // EVERY + 1, W // EVERY + 1), -1)
        grid[last.rows[off] // EVERY, last.cols[off] // EVERY] = code
        index[last.rows[off] // EVERY, last.cols[off] // EVERY] = np.arange(len(off))
        lumps = [i for j in np.unique(code) for g in voxels._joined(grid == j, voxels._BESIDE) for i in [index[g]]
                 if len(i) >= LUMP]
        level = np.array([bool(low[i[0]]) for i in lumps], bool)
        ground = level & np.array([away[off[i]].max() >= REACH_M - voxels.CELL_M for i in lumps], bool)
        lo = np.array([xyz[i].min(axis=0) for i in lumps]).reshape(-1, 3)
        hi = np.array([xyz[i].max(axis=0) for i in lumps]).reshape(-1, 3)
        n, m = len(kinds), len(lumps)
        boxes = (np.vstack([centre, 0.5 * (lo + hi)]), np.vstack([half, 0.5 * (hi - lo)]),
                 np.concatenate([R, np.tile(np.eye(3), (m, 1, 1))]))
        names, whole = classes.of(kinds + ['ground' if g else self.map.names[look[i[0]]] for g, i in zip(ground, lumps)],
                                  *boxes, np.concatenate([clear, level.astype(float)]))
        thing = np.full(len(last.at), n + m)
        thing[last.at >= 0] = labels.reshape(-1)[last.at[last.at >= 0]]
        thing = np.where(thing < 0, n + m, thing)
        for j, i in enumerate(lumps):
            thing[off[i]] = n + j
        (ones, built), housed = classes.grouped(names, whole, *boxes), np.full(len(last.at), -1)
        for b, (_name, c, h, members) in enumerate(built):
            seen_out = bool((np.abs(np.asarray(last.seen.at)[::2] - c[::2]) > h[::2]).any())
            housed = np.where(seen_out & np.isin(thing, members), b, housed)
        return (np.array(names + ['thing'], object)[thing], np.append(ones, -1)[thing], housed,
                [b[0] for b in built])


def thumbnail(rgb, w, h):
    """`rgb` (H, W, 3) shrunk to (h, w, 3) by block means."""
    np = _np()
    hh, ww = rgb.shape[0] // h * h, rgb.shape[1] // w * w
    return rgb[:hh, :ww].reshape(h, hh // h, w, ww // w, 3).mean(axis=(1, 3)).astype(np.uint8)


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


def _nearest_free(blocked, cell):
    np = _np()
    free = np.argwhere(~blocked)
    if len(free) == 0:
        return cell
    k = np.argmin(((free - np.array(cell)) ** 2).sum(axis=1))
    return (int(free[k][0]), int(free[k][1]))


def _astar(cost, start, goal):
    """The cells from `start` to `goal` of least cost, `cost` (nx, nz) a metre's through each, inf blocked:
    eight ways, a step its length times its two cells' mean cost, no diagonal past a blocked cell; []
    where none. Each cell costs 1 at least: the straight distance is its heuristic."""
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


def _line(a, b):
    """The cells of the straight line from `a` to `b`, a cell a sample."""
    n = max(abs(b[0] - a[0]), abs(b[1] - a[1]), 1)
    return [(int(round(a[0] + (b[0] - a[0]) * k / n)), int(round(a[1] + (b[1] - a[1]) * k / n))) for k in range(n + 1)]


def _cuts(cost, a, b, most):
    """Whether the straight line from cell `a` to `b` is free and its dearest cell costs `most` or less."""
    on = [float(cost[c]) for c in _line(a, b)]
    return max(on) <= most + 1e-9


def _pruned(cost, path):
    """The path's corners: each the furthest cell straight from the one before whose line's dearest
    cell is no dearer than the path's it cuts (`_cuts`): no nearer her things than the way it
    replaces; at a cost of 1 each, the furthest seen free."""
    out, i = [path[0]], 0
    on = [float(cost[c]) for c in path]
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _cuts(cost, path[i], path[j], max(on[i:j + 1])):
            j -= 1
        out.append(path[j])
        i = j
    return out


def told(keep):
    """[(kind, centre, half sizes, turn, id, clear)] of what her map tells (`voxels.Map.things`) as
    `world.props()` gives the room's boxes, each thing's id and whether the space over it is seen clear with
    it: her registry's looks (`machine.eyes.registry`) from her eyes - the last look's objects (`instances`) in place
    of those of her map's they are, where her segmenter is."""
    things = keep.map.things()[0]
    return instances.merged(things, keep.objects, keep.map) if keep.objects else things


def plan(blocked, at, to):
    """[(x, z)] from `at` to `to` round the cells `blocked` (`voxels.Map.blocked`), the corners alone;
    either end in a thing's reach, the nearest cell out of it; [] with no way or an end off the map."""
    np = _np()
    cells = [(int(math.floor((p[0] - X_M[0]) / CELL_M)), int(math.floor((p[1] - Z_M[0]) / CELL_M))) for p in (at, to)]
    if not all(0 <= c[0] < blocked.shape[0] and 0 <= c[1] < blocked.shape[1] for c in cells):
        return []
    start, goal = (c if not blocked[c] else _nearest_free(blocked, c) for c in cells)
    cost = np.where(blocked, np.inf, 1.0)
    path = _astar(cost, start, goal)
    return [(X_M[0] + (i + 0.5) * CELL_M, Z_M[0] + (j + 0.5) * CELL_M) for i, j in _pruned(cost, path)[1:]] if path else []
