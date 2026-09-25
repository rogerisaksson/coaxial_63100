"""The gynoid: a slender body on the humanoid's twenty joints, posed, lit, drawn in braille.

    lines = render(angles, 96, 40, yaw=30, lit=gpu.LitRaster())   # the GPU's lit raster
    lines = render(angles, 96, 40)                                  # no card: dots splatted here

A part is a closed loft or ellipsoid in its own frame, hung off its parent at an offset and turned
by its joint - the humanoid's names and signs; the legs are `machine.gait`'s. Given the pelvis's
place (`gait.sway`), what the type has no joint for is drawn: the spine bends the pelvis's drop
back out, so the torso stands upright over the line, and each leg keeps its track and heading
as a hip's ab- and adduction and rotation would. Without it, the lowest point of the feet stands
on the floor. 1.69 m tall; the lattice, the glowing core and the plates are the materials
`gpu.LIT_WGSL` lights. The floor scrolls under her by `travel` metres.
"""
import math

from coaxial.graphics import engine
from coaxial.graphics.raster import BRAILLE, BRAILLE_BITS, DOTS_X, DOTS_Y, NOISE
from machine import ansi
from machine.gait import ANKLE_H, BALL, HIP_DROP, HIP_HALF, SHANK, STAND_M, THIGH

#: How much of the pelvis's sideways shift, and of its rise and fall about its level, the torso's
#: base takes: the rest the spine bends out and gives, so the head sails - riding the pelvis, it
#: bobbed 23 mm a step.
FOLLOW, FOLLOW_Y = 0.15, 0.15

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


def _parts():
    """(name, parent, joint, axis, sign, offset, rest, mesh): parents first. `rest` is a fixed turn
    (axis, degrees) before the joint's own."""
    pelvis = _loft([(-0.10, 0.055, 0.048), (-0.07, 0.115, 0.08, -0.006),
                    (-0.03, 0.15, 0.095, -0.014), (0.02, 0.148, 0.09, -0.008), (0.07, 0.122, 0.078),
                    (0.11, 0.1, 0.07), (0.13, 0.094, 0.066)], PLATE, poles=(-0.115, 0.14))
    # From 4.5 cm inside the pelvis: the spine gives the pelvis's rise and fall, and the waist
    # must not open.
    torso = _loft([(-0.045, 0.09, 0.063), (0.0, 0.094, 0.066), (0.047, 0.096, 0.068),
                   (0.093, 0.105, 0.074),
                   (0.149, 0.118, 0.08), (0.205, 0.126, 0.083), (0.26, 0.13, 0.078),
                   (0.307, 0.138, 0.07), (0.344, 0.14, 0.062), (0.372, 0.1, 0.055),
                   (0.39, 0.05, 0.045)], _core, poles=(-0.055, 0.40))
    out = [('pelvis', None, 'pelvis', 'y', 1, (0.0, 0.0, 0.0), None, pelvis),
           ('torso', 'pelvis', 'waist', 'y', 1, (0.0, 0.12, 0.0), None, torso),
           ('neck', 'torso', 'neck', 'x', 1, (0.0, 0.385, 0.006), None,
            _loft([(0.0, 0.031, 0.029), (0.05, 0.027, 0.026), (0.095, 0.026, 0.025)], MESH,
                  poles=(-0.01, 0.10))),
           ('head', 'neck', 'head', 'y', 1, (0.0, 0.09, 0.012), None,
            _ellipsoid((0.0, HEAD_Y, 0.012), (0.066, 0.1, 0.084), _face, rows=12)),
           ('jaw', 'head', None, 'y', 1, (0.0, 0.0, 0.0), None,
            _ellipsoid((0.0, 0.042, 0.03), (0.045, 0.046, 0.054), SKIN))]
    for side, x in (('left', 1.0), ('right', -1.0)):
        out += [
            ('%s_bust' % side, 'torso', None, 'y', 1, (0.052 * x, 0.21, 0.05), None,
             _ellipsoid((0.0, 0.0, 0.0), (0.048, 0.044, 0.04), PLATE, rows=8)),
            ('%s_cap' % side, 'torso', None, 'y', 1, (0.135 * x, 0.335, -0.004), None,
             _ellipsoid((0.0, 0.0, 0.0), (0.042, 0.036, 0.04), PLATE, rows=8)),
            ('%s_upper_arm' % side, 'torso', side + '_shoulder', 'x', -1,
             (0.148 * x, 0.325, -0.005), ('z', 6.0 * x),
             _limb(0.27, 0.031, 0.03, 0.023, MESH, flat=0.95)),
            ('%s_forearm' % side, '%s_upper_arm' % side, side + '_elbow', 'x', -1,
             (0.0, -0.27, 0.0), None, _limb(0.24, 0.024, 0.023, 0.017, MESH, flat=0.85)),
            ('%s_hand' % side, '%s_forearm' % side, side + '_wrist', 'x', -1, (0.0, -0.24, 0.0),
             None, _ellipsoid((0.0, -0.043, 0.004), (0.014, 0.047, 0.032), PLATE, rows=8)),
            ('%s_fingers' % side, '%s_hand' % side, side + '_gripper', 'x', -1,
             (0.0, -0.086, 0.004), None,
             _ellipsoid((0.0, -0.035, 0.0), (0.011, 0.042, 0.028), PLATE, rows=8)),
            ('%s_thigh' % side, 'pelvis', side + '_hip', 'x', 1,
             (HIP_HALF * x, -HIP_DROP, 0.0), ('z', 0.5 * x),
             _limb(THIGH, 0.064, 0.055, 0.039, MESH, bulge_at=0.22)),
            ('%s_shank' % side, '%s_thigh' % side, side + '_knee', 'x', 1, (0.0, -THIGH, 0.0),
             None, _limb(SHANK, 0.04, 0.044, 0.024, PLATE, bulge_at=0.3)),
            ('%s_foot' % side, '%s_shank' % side, side + '_ankle', 'x', 1, (0.0, -SHANK, 0.0),
             None, _loft([(z, rx, rv, ANKLE_H - rv) for z, rx, rv in _SOLE], PLATE,
                         poles=(-0.07, BALL + 0.01), along='z')),
            ('%s_toes' % side, '%s_foot' % side, side + '_foot', 'x', 1,
             (0.0, -(ANKLE_H - TOE_RY), BALL), None,
             _ellipsoid((0.0, 0.0, 0.03), (0.04, TOE_RY, 0.035), PLATE, rows=6))]
    return out


#: The foot's rings forward of the ankle, (z, half width, half height): each hung so its
#: bottom is the sole, flat ANKLE_H under the ankle, as the walk plants it.
_SOLE = ((-0.055, 0.028, 0.03), (-0.02, 0.034, 0.042), (0.04, 0.042, 0.034),
         (0.10, 0.044, 0.024), (BALL, 0.04, 0.018))
#: The toes' half height: their bottom the sole too.
TOE_RY = 0.014


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

    def _frames(self, angles, root=None, tracks=None):
        """Each part's (turn, spot) in the body's frame for {joint: degrees}; with `root`, the
        pelvis's (lateral metres, roll degrees, height metres), the torso upright over the line
        and each leg down to its foot's `tracks` (left, right; `gait.tracks`), facing ahead."""
        np = _np()
        lateral, roll, height, level = root if root is not None else (0.0, 0.0, 0.0, 0.0)
        left, right, turn_left, turn_right = (tracks if tracks is not None
                                              else (STAND_M, -STAND_M, 0.0, 0.0))
        placed, out = {}, []
        for name, parent, joint, axis, sign, offset, rest, _mesh in self.parts:
            turn, at = (placed[parent] if parent
                        else (_turn('z', roll), np.array([lateral, height, 0.0])))
            spot = at + turn @ np.asarray(offset, float)
            base = turn @ (_turn(*rest) if rest else np.eye(3))
            if root is not None and name == 'torso':
                # The spine bends the pelvis's drop back out: the torso upright, its base
                # FOLLOW of the way with the pelvis.
                base = _turn('z', -roll) @ turn
                spot[0] = FOLLOW * lateral
                spot[1] -= (1.0 - FOLLOW_Y) * (height - level)
            elif root is not None and joint and joint.endswith('_hip'):
                # A hip's ab/adduction and rotation, the type's lack: the leg down its track,
                # facing ahead, whatever the pelvis does.
                track, twist = ((left, turn_left) if name.startswith('left')
                                else (right, turn_right))
                base = (_turn('z', math.degrees(math.atan2(track - spot[0], spot[1])))
                        @ _turn('y', twist))
            here = base @ _turn(axis, sign * float(angles.get(joint, 0.0))) if joint else base
            placed[name] = (here, spot)
            out.append(placed[name])
        return out

    def pose(self, angles, dense=False, root=None, tracks=None):
        """(positions, normals) in metres, the floor at y 0, for {joint: degrees}; `root` the
        pelvis's place (`gait.sway`) and `tracks` the feet's (`gait.tracks`), else the feet found
        and stood on the floor; `dense` the sampled points' instead of the corners'."""
        frames = self._frames(angles, root, tracks)
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


def braille(depth, rgb, floor, width, height, colour=True):
    """Dot rasters down to cells: a dot where the light clears the blue noise, the silhouette and
    every depth step always; the floor's dots where she is not. Lines, ANSI where `colour`."""
    np = _np()
    covered = depth > 0.0
    lum = rgb.astype(float) @ (0.2126, 0.7152, 0.0722) / 255.0
    pad = np.pad(depth, 1)
    steps = [pad[1:-1, :-2], pad[1:-1, 2:], pad[:-2, 1:-1], pad[2:, 1:-1]]
    edge = covered & np.any([(s == 0.0) | (np.abs(s - depth) > 0.04 * depth) for s in steps], 0)
    lit = (covered & (0.04 + 0.96 * np.clip(lum, 0.0, 1.0) ** 1.5 > _mask(*depth.shape))) | edge
    ground = (floor > 0.0) & ~covered
    bits = np.array([[BRAILLE_BITS[lane][y] for lane in range(DOTS_X)] for y in range(DOTS_Y)])
    cells = ((lit | ground).reshape(height, DOTS_Y, width, DOTS_X)
             * bits[None, :, None, :]).sum(axis=(1, 3))
    hits = covered.reshape(height, DOTS_Y, width, DOTS_X).sum(axis=(1, 3))
    body = ((rgb * covered[..., None]).reshape(height, DOTS_Y, width, DOTS_X, 3).sum(axis=(1, 3))
            / np.maximum(hits, 1)[..., None])
    body = np.clip(body * 1.2 + 18.0, 0.0, 255.0)
    shine = floor.reshape(height, DOTS_Y, width, DOTS_X).max(axis=(1, 3))
    text = np.where(cells > 0, BRAILLE + cells, ord(' ')).tolist()
    if not colour:
        return [''.join(map(chr, row)) for row in text]
    # A cell's ink in steps of INK_STEP a channel, so runs of it share one escape; a blank cell
    # takes its left neighbour's, so a gap does not break a run. Cell by cell: 11.7 ms a frame at
    # 180 x 56, and rich parsed an escape a cell after it.
    ink = np.where((hits > 0)[..., None], body, np.asarray(FLOOR_INK) * shine[..., None])
    ink = (ink.astype(int) // INK_STEP) * INK_STEP
    key = np.where(cells > 0, (ink[..., 0] << 16) | (ink[..., 1] << 8) | ink[..., 2], -1)
    left = np.maximum.accumulate(np.where(key >= 0, np.arange(width), 0), axis=1)
    key = np.take_along_axis(key, left, axis=1)
    lines = []
    for row, keys in zip(text, key.tolist()):
        out, at = [], 0
        line = ''.join(map(chr, row))
        for end in [i for i in range(1, width) if keys[i] != keys[i - 1]] + [width]:
            if keys[at] >= 0:
                out.append(_escape(keys[at]))
            out.append(line[at:end])
            at = end
        lines.append(''.join(out) + (ansi.RESET if max(keys) >= 0 else ''))
    return lines


#: A channel's step in the drawn ink.
INK_STEP = 8

#: Escapes by packed ink.
_ESCAPES = {}


def _escape(packed):
    got = _ESCAPES.get(packed)
    if got is None:
        got = _ESCAPES[packed] = ansi.code((packed >> 16, (packed >> 8) & 255, packed & 255))
    return got


#: The body, built on first use: 0.1 s of lofts.
_BODY = {}


def body():
    got = _BODY.get('body')
    if got is None:
        got = _BODY['body'] = Body()
    return got


def render(angles, width, height, yaw=30.0, pitch=8.0, zoom=1.0, colour=True, travel=0.0,
           lit=None, root=None, tracks=None):
    """Her, posed at {joint: degrees}, swayed by `root` (`machine.gait.sway`) on `tracks`, `width` x
    `height` cells: lines. `lit` a `gpu.LitRaster`, or None to splat her dots here."""
    np = _np()
    who = body()
    m = view(yaw, pitch)
    fine = engine.fine(engine.camera(width, height, REACH, distance=DISTANCE, zoom=zoom))
    centre = np.asarray(CENTRE)
    if lit is not None:
        positions, normals = who.pose(angles, root=root, tracks=tracks)
        depth, rgb = lit.raster(positions, normals, who.uv, who.materials, who.index, m, fine,
                                CENTRE, REACH * 1.4)
    else:
        positions, normals = who.pose(angles, dense=True, root=root, tracks=tracks)
        depth, rgb = _splat((positions, normals, who.dense[3], who.dense[2]), m, fine, centre)
    return braille(depth, rgb, _floor(m, fine, centre, travel), width, height, colour)
