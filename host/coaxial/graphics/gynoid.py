"""The gynoid: a slender body on `machine.figure`'s joints, posed, lit, drawn in braille.

    lines = render(angles, 96, 40, yaw=30, lit=gpu.LitRaster())   # the GPU's lit raster
    lines = render(angles, 96, 40, root=(where, turn))             # the pelvis placed, world
    lines = render(angles, 96, 40, labels={'left_knee': (boxes, number)})   # called out

A part is a closed loft or ellipsoid in its own frame, hung off its parent at the figure's offset
and turned by its joints - the figure's names and signs. Without `root`, the lowest point of the
feet stands on the floor. 1.69 m tall; the lattice, the glowing core and the plates are the
materials `gpu.LIT_WGSL` lights. The floor scrolls under her by `travel` metres.
"""
import math

from coaxial.graphics import engine
from coaxial.graphics.raster import BRAILLE, BRAILLE_BITS, DOTS_X, DOTS_Y, NOISE
from machine import ansi, figure
from machine.figure import TOE_RY
from machine.gait import ANKLE_H, BALL, SHANK, THIGH

#: A corner's material, as `gpu.LIT_WGSL` colours it.
MESH, SKIN, PLATE, CORE = 0, 1, 2, 3

#: Corners round a ring.
AROUND = 20

#: The camera: its distance in the engine's units, the point it turns about (her middle, metres
#: over the floor), and how far out she reaches from it.
DISTANCE, CENTRE, REACH = 3.2, (0.0, 0.84, 0.0), 0.92

#: The floor's grid pitch, metres, and its half extent.
FLOOR_PITCH, FLOOR_HALF = 0.3, 2.4


def _np():
    from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
    return np


def _loft(rings, material, poles=None, along='y'):
    """A closed body through elliptical rings (at, rx, rz[, dz]) up its axis, capped at `poles`
    (default the first and last ring): (corners, triangles, uv, materials) in its part's frame.
    `along` 'z' lays it forward, a ring's rz then its height and dz its drop."""
    np = _np()
    k = np.arange(AROUND) * (2.0 * math.pi / AROUND)
    rows = [np.stack([r[1] * np.cos(k), np.full(AROUND, float(r[0])),
                      r[2] * np.sin(k) + (r[3] if len(r) > 3 else 0.0)], 1) for r in rings]
    low, high = poles or (rings[0][0], rings[-1][0])
    ends = [[0.0, low, rings[0][3] if len(rings[0]) > 3 else 0.0],
            [0.0, high, rings[-1][3] if len(rings[-1]) > 3 else 0.0]]
    corners = np.vstack(rows + [np.array(ends)])
    n, j = len(rows), np.arange(AROUND)
    a = (np.arange(n - 1)[:, None] * AROUND + j).ravel()
    b = (np.arange(n - 1)[:, None] * AROUND + (j + 1) % AROUND).ravel()
    c, d = a + AROUND, b + AROUND
    bottom, top = n * AROUND, n * AROUND + 1
    last = (n - 1) * AROUND
    triangles = np.vstack([np.stack([a, c, b], 1), np.stack([c, d, b], 1),
                           np.stack([np.full(AROUND, bottom), j, (j + 1) % AROUND], 1),
                           np.stack([np.full(AROUND, top), last + (j + 1) % AROUND, last + j], 1)])
    uv = np.stack([np.concatenate([np.tile(j / AROUND, n), [0.0, 0.0]]), corners[:, 1]], 1)
    if along == 'z':                       # the axis forward: (x, y, z) -> (x, -z, y)
        corners = np.stack([corners[:, 0], -corners[:, 2], corners[:, 1]], 1)
    if callable(material):
        materials = material(corners)
    else:
        materials = np.full(len(corners), material)
    return corners, triangles, uv, materials


def _ellipsoid(centre, radii, material, rows=10):
    """An ellipsoid about `centre`, its poles on its y axis."""
    np = _np()
    cx, cy, cz = centre
    rx, ry, rz = radii
    phi = np.pi * np.arange(1, rows) / rows
    body = _loft([(cy - ry * math.cos(p), rx * math.sin(p), rz * math.sin(p), cz) for p in phi],
                 material, poles=(cy - ry, cy + ry))
    corners = body[0]
    corners[:, 0] += cx
    return body


def _limb(length, top, middle, bottom, material, flat=1.0, bulge_at=0.3):
    """A tapered limb hanging from its joint down -y: `top` at the joint, `middle` at `bulge_at` of
    the way, `bottom` at the end, rounded; `flat` its depth over its width."""
    rings = [(-length - 0.6 * bottom, 0.5 * bottom), (-length - 0.3 * bottom, 0.9 * bottom),
             (-length, bottom), (-length * (0.5 + 0.5 * bulge_at), 0.5 * (middle + bottom)),
             (-length * bulge_at, middle), (0.0, top), (0.3 * top, 0.88 * top),
             (0.55 * top, 0.5 * top)]
    return _loft([(y, r, r * flat) for y, r in rings], material)


#: The head's centre over the head joint, metres.
HEAD_Y = 0.095


def _face(corners):
    """The head's corners: skin on the face, the hood's mesh round the rest."""
    np = _np()
    y, z = corners[:, 1] - HEAD_Y, corners[:, 2] - 0.012
    return np.where((z > 0.028) & (y < 0.07) & (y > -0.12), SKIN, MESH)


def _core(corners):
    """The torso's corners: a band of the core's glow in front of the midriff."""
    np = _np()
    return np.where((corners[:, 1] > 0.03) & (corners[:, 1] < 0.10) & (corners[:, 2] > 0.035),
                    CORE, MESH)


def _meshes():
    """{segment: mesh} for the figure's segments, and the parts it has no joint for:
    [(name, parent, offset, mesh)]."""
    pelvis = _loft([(-0.10, 0.055, 0.048), (-0.07, 0.115, 0.08, -0.006),
                    (-0.03, 0.15, 0.095, -0.014), (0.02, 0.148, 0.09, -0.008), (0.07, 0.122, 0.078),
                    (0.11, 0.1, 0.07), (0.13, 0.094, 0.066)], PLATE, poles=(-0.115, 0.14))
    # From 4.5 cm inside the pelvis, so the waist does not open as the spine bends.
    torso = _loft([(-0.045, 0.09, 0.063), (0.0, 0.094, 0.066), (0.047, 0.096, 0.068),
                   (0.093, 0.105, 0.074),
                   (0.149, 0.118, 0.08), (0.205, 0.126, 0.083), (0.26, 0.13, 0.078),
                   (0.307, 0.138, 0.07), (0.344, 0.14, 0.062), (0.372, 0.1, 0.055),
                   (0.39, 0.05, 0.045)], _core, poles=(-0.055, 0.40))
    meshes = {'pelvis': pelvis, 'torso': torso,
              'neck': _loft([(0.0, 0.031, 0.029), (0.05, 0.027, 0.026), (0.095, 0.026, 0.025)],
                            MESH, poles=(-0.01, 0.10)),
              'head': _ellipsoid((0.0, HEAD_Y, 0.012), (0.066, 0.1, 0.084), _face, rows=12)}
    extra = [('jaw', 'head', (0.0, 0.0, 0.0),
              _ellipsoid((0.0, 0.042, 0.03), (0.045, 0.046, 0.054), SKIN))]
    for side, x in (('left', 1.0), ('right', -1.0)):
        extra += [('%s_bust' % side, 'torso', (0.052 * x, 0.21, 0.05),
                   _ellipsoid((0.0, 0.0, 0.0), (0.048, 0.044, 0.04), PLATE, rows=8)),
                  ('%s_cap' % side, 'torso', (0.135 * x, 0.335, -0.004),
                   _ellipsoid((0.0, 0.0, 0.0), (0.042, 0.036, 0.04), PLATE, rows=8))]
        meshes.update({
            side + '_upper_arm': _limb(0.27, 0.031, 0.03, 0.023, MESH, flat=0.95),
            side + '_forearm': _limb(0.24, 0.024, 0.023, 0.017, MESH, flat=0.85),
            side + '_hand': _ellipsoid((0.0, -0.043, 0.004), (0.014, 0.047, 0.032), PLATE, rows=8),
            side + '_fingers': _ellipsoid((0.0, -0.035, 0.0), (0.011, 0.042, 0.028), PLATE, rows=8),
            side + '_thigh': _limb(THIGH, 0.064, 0.055, 0.039, MESH, bulge_at=0.22),
            side + '_shank': _limb(SHANK, 0.04, 0.044, 0.024, PLATE, bulge_at=0.3),
            side + '_foot': _loft([(z, rx, rv, ANKLE_H - rv) for z, rx, rv in _SOLE], PLATE,
                                  poles=(-0.07, BALL + 0.01), along='z'),
            side + '_toes': _ellipsoid((0.0, 0.0, 0.03), (0.04, TOE_RY, 0.035), PLATE, rows=6)})
    return meshes, extra


def _parts():
    """(name, parent, joints ((joint, axis, sign), ..), offset, rest turn about z (deg), mesh):
    the figure's segments, then the parts it carries without a joint; parents first."""
    meshes, extra = _meshes()
    out = [(seg[0], seg[1], seg[2], seg[3], seg[4], meshes[seg[0]]) for seg in figure.SEGMENTS]
    return out + [(name, parent, (), offset, 0.0, mesh) for name, parent, offset, mesh in extra]


#: The foot's rings forward of the ankle, (z, half width, half height): each hung so its
#: bottom is the sole, flat ANKLE_H under the ankle, as the walk plants it.
_SOLE = ((-0.055, 0.028, 0.03), (-0.02, 0.034, 0.042), (0.04, 0.042, 0.034),
         (0.10, 0.044, 0.024), (BALL, 0.04, 0.018))


class Body:

    """The parts' meshes laid end to end once, with their smooth normals; `pose(angles)` turns them."""

    def __init__(self):
        np = _np()
        self.parts = _parts()
        corners, triangles, uv, materials, spans, faces = [], [], [], [], [], []
        base = 0
        for i, (*_head, mesh) in enumerate(self.parts):
            c, t, u, m = mesh
            corners.append(c)
            triangles.append(t + base)
            uv.append(u)
            materials.append(m)
            spans.append((base, base + len(c)))
            faces.append(np.full(len(t), i))
            base += len(c)
        self.corners = np.vstack(corners)
        self.index = np.vstack(triangles).astype(np.uint32)
        self.uv, self.materials = np.vstack(uv), np.concatenate(materials).astype(np.uint32)
        self.spans = spans
        self.normals = _normals(self.corners, self.index, spans)
        #: The parts whose lowest corner is the sole.
        self.soles = [i for i, part in enumerate(self.parts)
                      if part[0].endswith(('_foot', '_toes'))]
        #: Every triangle sampled DENSE_M apart, for the dots drawn without a card: (points,
        #: normals, uv, materials) in their parts' frames, and each part's span of them.
        self.dense, self.dense_spans = _sampled(self, np.concatenate(faces))

    def _frames(self, angles, root=None):
        """Each part's (turn, spot) in the world for {joint: degrees}; `root` the pelvis's (place,
        turn 3x3), else at the origin, upright."""
        np = _np()
        where, turn = root if root is not None else ((0.0, 0.0, 0.0), np.eye(3))
        placed, out = {}, []
        for name, parent, joints, offset, rest, _mesh in self.parts:
            if parent:
                above, at = placed[parent]
                spot = at + above @ np.asarray(offset, float)
                here = above @ _turn('z', rest) if rest else above
            else:
                spot, here = np.asarray(where, float), np.asarray(turn, float)
            for joint, axis, sign in joints:
                here = here @ _turn(axis, sign * float(angles.get(joint, 0.0)))
            placed[name] = (here, spot)
            out.append(placed[name])
        return out

    def pivots(self, angles, root=None):
        """{joint: its pivot, world}; `root` as `pose` takes it."""
        np = _np()
        frames = self._frames(angles, root)
        floor = np.zeros(3)
        if root is None:
            positions, _normals = _placed(self.corners, self.normals, self.spans, frames)
            floor[1] = min(positions[slice(*self.spans[i]), 1].min() for i in self.soles)
        return {joint: spot - floor for (_name, _parent, joints, *_rest), (_turn, spot)
                in zip(self.parts, frames) for joint, _axis, _sign in joints}

    def pose(self, angles, dense=False, root=None):
        """(positions, normals) in metres, the floor at y 0, for {joint: degrees}; `root` the
        pelvis's (place, turn), else the feet found and stood on the floor; `dense` the sampled
        points' instead of the corners'."""
        frames = self._frames(angles, root)
        positions, normals = _placed(self.corners, self.normals, self.spans, frames)
        floor = 0.0 if root is not None else min(
            positions[slice(*self.spans[i]), 1].min() for i in self.soles)
        if dense:
            positions, normals = _placed(self.dense[0], self.dense[1], self.dense_spans, frames)
        positions[:, 1] -= floor
        return positions, normals


def _placed(points, normals, spans, frames):
    """Points and normals each moved into the body's frame by its part's (turn, spot)."""
    np = _np()
    out_p, out_n = np.empty_like(points), np.empty_like(normals)
    for (lo, hi), (turn, spot) in zip(spans, frames):
        out_p[lo:hi] = points[lo:hi] @ turn.T + spot
        out_n[lo:hi] = normals[lo:hi] @ turn.T
    return out_p, out_n


#: How far apart the dots drawn without a card are sampled on the surface, metres: under two
#: fine dots at 110 x 50 cells, so a 2x2 splat closes it; 7 mm was 278 000 points, 290 ms.
DENSE_M = 0.012


def _sampled(body, part_of):
    """Every triangle sampled on a barycentric grid DENSE_M apart, grouped by part: ((points,
    normals, uv, materials), [(lo, hi)] a part)."""
    np = _np()
    index = body.index.astype(int)
    a, b, c = (body.corners[index[:, k]] for k in range(3))
    edge = np.max([np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1),
                   np.linalg.norm(a - c, axis=1)], axis=0)
    steps = np.maximum(1, np.ceil(edge / DENSE_M)).astype(int)
    fields = (body.corners, body.normals, body.uv)
    chunks = [[] for _ in range(len(fields) + 1)]
    parts = []
    for n in np.unique(steps):
        tris = np.flatnonzero(steps == n)
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
        keep = (i + j) <= n
        weights = np.stack([n - i[keep] - j[keep], i[keep], j[keep]], 1) / float(n)
        for out, field in zip(chunks, fields):
            corners = np.stack([field[index[tris, k]] for k in range(3)], 1)
            out.append(np.einsum('sk,tk...->ts...', weights, corners).reshape(
                -1, *field.shape[1:]))
        # A material is its nearest corner's: blended, a hood's edge came out skin.
        chunks[3].append(body.materials[index[tris][:, weights.argmax(axis=1)]].ravel())
        parts.append(np.repeat(part_of[tris], len(weights)))
    order = np.argsort(np.concatenate(parts), kind='stable')
    dense = [np.concatenate(out)[order] for out in chunks]
    counts = np.bincount(np.concatenate(parts), minlength=len(body.parts))
    ends = np.cumsum(counts)
    return tuple(dense), list(zip((ends - counts).tolist(), ends.tolist()))


def _normals(corners, index, spans):
    """Smooth normals: each corner's faces' normals, area-weighted, within its own part."""
    np = _np()
    a, b, c = (corners[index[:, k]] for k in range(3))
    faces = np.cross(b - a, c - a)
    out = np.zeros_like(corners)
    for k in range(3):
        np.add.at(out, index[:, k], faces)
    length = np.linalg.norm(out, axis=1, keepdims=True)
    return out / np.where(length > 0.0, length, 1.0)


def _turn(axis, degrees):
    np = _np()
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    if axis == 'x':
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    if axis == 'y':
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def view(yaw, pitch):
    """The view as the engine's 3x3, row-major: turned `yaw` about her, tipped `pitch` down."""
    return tuple((_turn('x', pitch) @ _turn('y', -yaw)).ravel().tolist())


#: The palette `gpu.LIT_WGSL` lights, for the dots drawn without a card.
PALETTE = ((0.58, 0.64, 0.72), (0.96, 0.79, 0.69), (0.80, 0.83, 0.88), (0.35, 0.85, 1.0))
KEY, FILL = (-0.45, 0.62, 0.64), (0.7, 0.1, 0.7)


def _lit_here(normals, materials, uv):
    """`gpu.LIT_WGSL`'s light on corners in view space: (n, 3) RGB 0..255."""
    np = _np()
    key = np.asarray(KEY) / np.linalg.norm(KEY)
    fill = np.asarray(FILL) / np.linalg.norm(FILL)
    n = normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-9)
    n = np.where(n[:, 2:3] < 0.0, -n, n)
    lam = np.clip(n @ key, 0.0, None)
    fil = 0.35 * np.clip(n @ fill, 0.0, None)
    rim = (1.0 - np.clip(n[:, 2], 0.0, 1.0)) ** 3
    half = (key + (0.0, 0.0, 1.0)) / np.linalg.norm(key + (0.0, 0.0, 1.0))
    spec = np.clip(n @ half, 0.0, None) ** 40
    base = np.asarray(PALETTE)[np.minimum(materials, 3)]
    g = np.abs(np.modf(uv * (22.0, 30.0))[0] - 0.5).max(axis=1)
    base = np.where((materials == MESH)[:, None] & (g > 0.4)[:, None], base * 0.5, base)
    shine = np.where(materials == SKIN, 0.12, 0.55)
    c = (base * (0.10 + 0.85 * lam + fil)[:, None] + (spec * shine)[:, None]
         + np.outer(rim * 0.8, (0.55, 0.75, 1.0)))
    c = np.where((materials == CORE)[:, None], base * (0.75 + 0.25 * lam)[:, None], c)
    return (np.clip(c, 0.0, 1.0) * 255.0).astype(np.uint8)


def _project(points, m, cam, centre):
    """(sx, sy, w) of world points: engine.project, every point at once."""
    np = _np()
    q = (np.asarray(points) - centre) @ np.asarray(m).reshape(3, 3).T
    w = 1.0 / (cam['distance'] - q[:, 2])
    return (cam['cx'] + cam['scale'] * w * q[:, 0],
            cam['cy'] - cam['scale'] * cam.get('aspect', 0.5) * w * q[:, 1], w)


def _splat(body_arrays, m, cam, centre):
    """(depth, colour) without a card: every corner a 2x2 dot splat, the nearest winning."""
    np = _np()
    positions, normals, materials, uv = body_arrays
    height, width = cam['height'], cam['width']
    sx, sy, w = _project(positions, m, cam, centre)
    colour = _lit_here(normals @ np.asarray(m).reshape(3, 3).T, materials, uv)
    depth = np.zeros((height, width), np.float32)
    rgb = np.zeros((height, width, 3), np.uint8)
    order = np.argsort(w)
    ix, iy = np.rint(sx[order]).astype(int), np.rint(sy[order]).astype(int)
    for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1)):
        x, y = ix + dx, iy + dy
        keep = (x >= 0) & (x < width) & (y >= 0) & (y < height)
        depth[y[keep], x[keep]] = w[order][keep]
        rgb[y[keep], x[keep]] = colour[order][keep]
    return depth, rgb


def _floor(m, cam, centre, travel):
    """The floor's grid points as a (height, width) brightness: 0 none, far dimmer."""
    np = _np()
    ticks = np.arange(-FLOOR_HALF, FLOOR_HALF + 1e-9, FLOOR_PITCH)
    gx, gz = np.meshgrid(ticks, ticks - (travel % FLOOR_PITCH))
    points = np.stack([gx.ravel(), np.zeros(gx.size), gz.ravel()], 1)
    sx, sy, w = _project(points, m, cam, centre)
    x, y = np.rint(sx).astype(int), np.rint(sy).astype(int)
    keep = (w > 0.0) & (x >= 0) & (x < cam['width']) & (y >= 0) & (y < cam['height'])
    out = np.zeros((cam['height'], cam['width']))
    edge = np.clip(1.0 - np.hypot(gx.ravel(), gz.ravel()) / (FLOOR_HALF * 1.1), 0.0, 1.0)
    out[y[keep], x[keep]] = 0.25 + 0.75 * edge[keep]
    return out


#: The floor's ink at full brightness: the house's teal.
FLOOR_INK = (40, 130, 140)


def _mask(height, width):
    np = _np()
    noise = np.asarray(NOISE, float) / 4096.0
    reps = (height // len(noise) + 1, width // len(noise[0]) + 1)
    return np.tile(noise, reps)[:height, :width]


#: A callout's leader ink, and how far either side of her middle the callouts stand, metres
#: across the view: clear of her widest stride and arm.
LEADER_INK, CALLOUT_M = (96, 110, 124), 0.42


def _line(dots, a, b):
    """The dots from `a` to `b`, (x, y) dot coordinates, set in `dots`."""
    np = _np()
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
    x = np.rint(np.linspace(a[0], b[0], n)).astype(int)
    y = np.rint(np.linspace(a[1], b[1], n)).astype(int)
    keep = (x >= 0) & (x < dots.shape[1]) & (y >= 0) & (y < dots.shape[0])
    dots[y[keep], x[keep]] = True


def _packed(fg, bg=None):
    """A cell's inks as one key: fg's 24 bits, bg's over them plus one, 0 for none; -1 no fg."""
    if fg is None:
        return -1
    key = (fg[0] << 16) | (fg[1] << 8) | fg[2]
    return key if bg is None else key | (((bg[0] << 16) | (bg[1] << 8) | bg[2]) + 1) << 24


def callouts(labels, anchors, places, columns, width, height):
    """({(row, col): (codepoint, key)}, leader dots) for `labels` {joint: (inner, outer)}, each
    [(char, fg, bg)], inks (r, g, b) or None: outside `columns` (left, right) dots, a side's
    joints on its side and the rest on the side they stand, each on the row its joint has at rest
    (`places` {joint: (x, y)}, dots) or the next free one down, the inner end toward her, a space
    between, and a leader from it to the joint's pivot as it is (`anchors`, dots). The callouts
    stand still; the leaders follow."""
    np = _np()
    dots = np.zeros((height * DOTS_Y, width * DOTS_X), bool)
    mid = (columns[0] + columns[1]) / 2.0
    xs = {side: [x for j, (x, _y) in places.items() if j.startswith(side)]
          for side in ('left_', 'right_')}
    flip = bool(xs['left_'] and xs['right_']) and np.mean(xs['left_']) > np.mean(xs['right_'])
    sides = {True: [], False: []}
    for joint, (inner, outer) in labels.items():
        if joint in anchors and joint in places:
            side = places[joint][0] < mid
            if joint.startswith(('left_', 'right_')):
                side = joint.startswith('left_') != flip
            sides[side].append((places[joint][1], joint, inner, outer))
    overlay = {}
    for left, items in sides.items():
        items.sort(key=lambda item: item[:2])
        rows, last = [], -1
        for y, *_rest in items:
            last = max(int(y // DOTS_Y), last + 1)
            rows.append(last)
        over = (rows[-1] - (height - 1)) if rows else 0
        rows = [max(0, r - max(0, over)) for r in rows]
        for row, (_y, joint, inner, outer) in zip(rows, items):
            gap = [(' ', None, None)]
            cells = outer + gap + inner if left else inner + gap + outer
            start = (int(columns[0] // DOTS_X) - len(cells) if left
                     else int(columns[1] // DOTS_X) + 1)
            for k, (char, fg, bg) in enumerate(cells):
                if 0 <= start + k < width:
                    overlay[(row, start + k)] = (ord(char), _packed(fg, bg))
            end = (start + len(cells)) * DOTS_X if left else start * DOTS_X - 1
            _line(dots, (end, row * DOTS_Y + DOTS_Y // 2), anchors[joint])
    return overlay, dots


def braille(depth, rgb, floor, width, height, colour=True, overlay=None, leaders=None):
    """Dot rasters down to cells: a dot where the light clears the blue noise, the silhouette and
    every depth step always; the floor's dots where she is not, and the `leaders`' dots; the
    `overlay`'s cells {(row, col): (codepoint, key)} over all (`callouts`). Lines, ANSI where
    `colour`."""
    np = _np()
    covered = depth > 0.0
    lum = rgb.astype(float) @ (0.2126, 0.7152, 0.0722) / 255.0
    pad = np.pad(depth, 1)
    steps = [pad[1:-1, :-2], pad[1:-1, 2:], pad[:-2, 1:-1], pad[2:, 1:-1]]
    edge = covered & np.any([(s == 0.0) | (np.abs(s - depth) > 0.04 * depth) for s in steps], 0)
    lit = (covered & (0.04 + 0.96 * np.clip(lum, 0.0, 1.0) ** 1.5 > _mask(*depth.shape))) | edge
    ground = (floor > 0.0) & ~covered
    lead = leaders & ~covered if leaders is not None else np.zeros_like(covered)
    bits = np.array([[BRAILLE_BITS[lane][y] for lane in range(DOTS_X)] for y in range(DOTS_Y)])
    cells = ((lit | ground | lead).reshape(height, DOTS_Y, width, DOTS_X)
             * bits[None, :, None, :]).sum(axis=(1, 3))
    hits = covered.reshape(height, DOTS_Y, width, DOTS_X).sum(axis=(1, 3))
    body = ((rgb * covered[..., None]).reshape(height, DOTS_Y, width, DOTS_X, 3).sum(axis=(1, 3))
            / np.maximum(hits, 1)[..., None])
    body = np.clip(body * 1.2 + 18.0, 0.0, 255.0)
    shine = floor.reshape(height, DOTS_Y, width, DOTS_X).max(axis=(1, 3))
    text = np.where(cells > 0, BRAILLE + cells, ord(' ')).tolist()
    for (row, col), (char, _ink) in (overlay or {}).items():
        text[row][col] = char
    if not colour:
        return [''.join(map(chr, row)) for row in text]
    # A cell's ink in steps of INK_STEP a channel, so runs of it share one escape; a blank cell
    # takes its left neighbour's, so a gap does not break a run. Cell by cell: 11.7 ms a frame at
    # 180 x 56, and rich parsed an escape a cell after it.
    ink = np.where((hits > 0)[..., None], body, np.asarray(FLOOR_INK) * shine[..., None])
    led = lead.reshape(height, DOTS_Y, width, DOTS_X).any(axis=(1, 3)) & (hits == 0)
    ink = np.where(led[..., None], np.asarray(LEADER_INK, float), ink)
    ink = (ink.astype(int) // INK_STEP) * INK_STEP
    key = np.where(cells > 0, (ink[..., 0] << 16) | (ink[..., 1] << 8) | ink[..., 2], -1)
    for (row, col), (_char, packed) in (overlay or {}).items():
        key[row, col] = packed
    # A blank cell takes its left neighbour's ink, not its ground.
    left = np.maximum.accumulate(np.where(key >= 0, np.arange(width), 0), axis=1)
    carried = np.take_along_axis(key, left, axis=1)
    key = np.where((key < 0) & (carried >= 0), carried & 0xFFFFFF, carried)
    lines = []
    for row, keys in zip(text, key.tolist()):
        out, at, ground = [], 0, False
        line = ''.join(map(chr, row))
        for end in [i for i in range(1, width) if keys[i] != keys[i - 1]] + [width]:
            if keys[at] >= 0:
                out.append(_escape(keys[at]))
                if ground and keys[at] < 1 << 24:
                    out.append(NO_GROUND)
                ground = keys[at] >= 1 << 24
            out.append(line[at:end])
            at = end
        lines.append(''.join(out) + (ansi.RESET if max(keys) >= 0 else ''))
    return lines


#: A channel's step in the drawn ink.
INK_STEP = 8

#: Escapes by packed ink.
_ESCAPES = {}


#: The terminal's own ground again.
NO_GROUND = '\033[49m'


def _escape(packed):
    got = _ESCAPES.get(packed)
    if got is None:
        fg = packed & 0xFFFFFF
        got = ansi.code((fg >> 16, (fg >> 8) & 255, fg & 255))
        if packed >= 1 << 24:
            bg = (packed >> 24) - 1
            got += ansi.back((bg >> 16, (bg >> 8) & 255, bg & 255))
        _ESCAPES[packed] = got
    return got


#: The body, built on first use: 0.1 s of lofts.
_BODY = {}


def body():
    got = _BODY.get('body')
    if got is None:
        got = _BODY['body'] = Body()
    return got


class Follow:

    """A camera's place along her walk: on at her mean speed, meaned over `seconds`, and toward
    her place four times slower - her surge shows, and a head carried evenly stands still. Tied to
    her pelvis, the pelvis stood still and an even head swung (2026-09-26)."""

    def __init__(self, seconds=1.0):
        self.seconds, self.at = seconds, None

    def __call__(self, place, speed, t):
        """The camera's place for her at `place` going `speed`, m and m/s, at `t`, s."""
        if self.at is None or abs(place - self.at[0]) > 1.0 or t <= self.at[2]:
            self.at = (place, speed, t)
            return place
        at, mean, then = self.at
        dt = t - then
        mean += (speed - mean) * min(1.0, dt / self.seconds)
        at += mean * dt + (place - at) * min(1.0, dt / (4.0 * self.seconds))
        self.at = (at, mean, t)
        return at


def render(angles, width, height, yaw=30.0, pitch=8.0, zoom=1.0, colour=True, travel=0.0,
           lit=None, root=None, labels=None):
    """Her, posed at {joint: degrees}, the pelvis at `root` (place, turn) if given, `width` x
    `height` cells: lines. `lit` a `gpu.LitRaster`, or None to splat her dots here; `labels`
    {joint: (inner, outer)} called out either side of her (`callouts`)."""
    np = _np()
    who = body()
    m = view(yaw, pitch)
    fine = engine.fine(engine.camera(width, height, REACH, distance=DISTANCE, zoom=zoom))
    centre = np.asarray(CENTRE)
    if lit is not None:
        positions, normals = who.pose(angles, root=root)
        depth, rgb = lit.raster(positions, normals, who.uv, who.materials, who.index, m, fine,
                                CENTRE, REACH * 1.4)
    else:
        positions, normals = who.pose(angles, dense=True, root=root)
        depth, rgb = _splat((positions, normals, who.dense[3], who.dense[2]), m, fine, centre)
    overlay = leaders = None
    if labels:
        pivots, rest = who.pivots(angles, root=root), who.pivots({})
        names = [j for j in labels if j in pivots]

        def dots_of(points):
            sx, sy, _w = _project(points, m, fine, centre)
            return list(zip(sx.tolist(), sy.tolist()))
        across = np.asarray(m).reshape(3, 3)[0] * CALLOUT_M
        (x0, _y0), (x1, _y1) = dots_of([centre - across, centre + across])
        overlay, leaders = callouts(labels, dict(zip(names, dots_of([pivots[j] for j in names]))),
                                    dict(zip(names, dots_of([rest[j] for j in names]))),
                                    (min(x0, x1), max(x0, x1)), width, height)
    return braille(depth, rgb, _floor(m, fine, centre, travel), width, height, colour, overlay,
                   leaders)
