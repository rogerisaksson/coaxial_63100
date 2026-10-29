"""Her eyes and what she makes of them. A stereo pair on her head rendered from the world, the
disparity by block matching, depth from it, the points into the map she keeps of the room
(`machine.voxels`) in her estimate's frame, its things boxes with their turn (docs/findings/sight.md).

    eyes, keep = Eyes(world), Keep()        # her eyes on a world, the map she keeps
    keep.took(*eyes.stereo(world, head))    # a look matched by her stereo into the map, from her head's pose
    keep.took(*eyes.shot(world, head))      # a look's parallax from the renderer's depth (`machine.camera`)
    things = told(keep)                     # [(kind, centre, half sizes, turn)] her registry's looks
    way = plan(keep.map.blocked(), at, to)  # [(x, z)] round what she knows
    keep.hulls()                            # what the last look showed, hulled by thing

Her keep reads nothing of the world: a look's frame - an eye's colour and parallax at her own pose - alone,
rendered on the world's side (`physics.SIGHT`).
"""
import heapq
import math

from machine.stereo import H, W, _np, disparity
from machine.voxels import CELL_M, X_M, Z_M
from machine import voxels

#: Her eyes: BASELINE_M apart on the head, EYES (x, y, z) in the head's frame the left one's,
#: looking along the head's z, DOWN_DEG under it - level, their view's lower edge 30 deg
#: down, a table's top was out of it nearer than 1.4 m and a chair's seat nearer than 1.9;
#: FOVY_DEG high, W x H pixels (`machine.stereo`).
BASELINE_M, EYES, FOVY_DEG, DOWN_DEG = 0.1, (0.05, 0.10, 0.09), 60.0, 15.0

#: A look LOOK_S apart. What she has not seen is open: her way is planned through it and again as she
#: sees - held shut, a target unseen had no way and every first word a look round first, 14 s on 23
#: steps.
LOOK_S = 0.5

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

#: The page's picture of her view, pixels wide and high (`coaxial.graphics.inset`).
VIEW = (32, 24)
#: Each kind's ink for the page, (r, g, b).
INK = {'wall': (120, 190, 255), 'door': (255, 120, 90), 'lamp': (255, 230, 90),
       'bed': (110, 140, 255), 'table': (230, 170, 100), 'chair': (255, 190, 120),
       'thing': (200, 200, 200)}


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


#: The left eye's frame in the head's (`cameras`' xyaxes): x her left, y up and DOWN_DEG forward, z back.
_EYE_R = ((-1.0, 0.0, 0.0), (0.0, math.cos(math.radians(DOWN_DEG)), math.sin(math.radians(DOWN_DEG))),
          (0.0, math.sin(math.radians(DOWN_DEG)), -math.cos(math.radians(DOWN_DEG))))


def eye_pose(head):
    """(the left eye's place, its rotation) in the frame her `head` {pos, quat} is given in."""
    np = _np()
    w, x, y, z = head['quat']
    R = np.array([[1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
                  [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
                  [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)]])
    return np.asarray(head['pos'], float) + R @ np.array(EYES), R @ np.array(_EYE_R).T


class Seen:

    """One look: both eyes' images (H, W, 3) uint8 and the left eye's pose - the renderer's, till
    `Keep.look` lays hers."""

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

    def stereo(self, world, head):
        """A look's frame (seen, disparity, kept): both eyes matched (`disparity`), laid at her `head`
        {pos, quat} as her estimate has it."""
        seen = self.see(world)
        seen.at, seen.turn = eye_pose(head)
        return (seen,) + tuple(disparity(seen.left, seen.right))

    def shot(self, world, head):
        """A look's frame (seen, disparity, kept) as her fake camera gives it (`machine.camera`): the left
        eye's colour and the parallax her stereo would match from the renderer's own depth (`truth`), every
        pixel within FAR_M kept, laid at her `head` {pos, quat} as her estimate has it."""
        np = _np()
        self.renderer.update_scene(world.data, camera=self.ids['left'])
        left, z = self.renderer.render().copy(), self.truth(world)
        kept = z < voxels.FAR_M
        return Seen(left, None, *eye_pose(head)), np.where(kept, focal() * BASELINE_M / np.maximum(z, 1e-6), 0.0), kept

    def truth(self, world):
        """The left eye's depth from the renderer's own buffer, m: the measure to hold the
        stereo to, never hers."""
        self.renderer.enable_depth_rendering()
        try:
            self.renderer.update_scene(world.data, camera=self.ids['left'])
            return self.renderer.render().copy()
        finally:
            self.renderer.disable_depth_rendering()


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

    """What one look put into the map: the images and, of the points kept, their pixels and voxels."""

    def __init__(self, seen, rows, cols, at):
        self.seen, self.rows, self.cols, self.at = seen, rows, cols, at


class Keep:

    """The map she keeps (`voxels.Map` of LOOKS' colours) and her last look into it, kept for the page."""

    def __init__(self):
        self.map, self.last = voxels.Map({k: a for k, (a, _b) in LOOKS.items()}), None

    def forget(self):
        """Her map laid anew, the last look let go: the room changed under her."""
        self.map, self.last = voxels.Map(self.map.means_of), None

    def took(self, seen, disp, kept):
        """A look's frame (`Eyes.stereo`, `Eyes.shot`) into the map; the Look."""
        xyz, rgb, rows, cols = points(seen, disp, kept)
        self.last = Look(seen, rows, cols, self.map.add(xyz, rgb, seen.at))
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
        things, labels = self.map.things()
        on = self.last.at >= 0
        which = labels.reshape(-1)[self.last.at[on]]
        by = {}
        for row, col, k in zip(self.last.rows[on], self.last.cols[on], which):
            if k >= 0:
                by.setdefault(int(k), []).append((int(col), int(row)))
        out = []
        for k, pts in by.items():
            hull = convex(pts)
            if len(hull) >= 3:
                out.append((things[k][0], INK.get(things[k][0], INK['thing']), hull))
        return out


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
    it: her registry's looks (`machine.registry`) from her eyes."""
    return keep.map.things()[0]


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
