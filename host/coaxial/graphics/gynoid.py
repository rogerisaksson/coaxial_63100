"""The gynoid: a slender body on `machine.figure`'s joints, posed, lit, drawn in braille.

1.69 m tall; each part is a closed loft or ellipsoid hung off its parent at `machine.figure`'s
offset. Without `root`, the lowest foot stands on y = 0.
"""
import math
from typing import Any

from coaxial.graphics import arrows as _arrows, drape, drums, engine, mechanism, panels, sneaker
from coaxial.graphics.callouts import callouts, line, packed
from coaxial.graphics.lit import (CORE, MESH, PAINTED, PLATE, SKIN, braille, grid, paint,
                                  project, splat)
from coaxial.graphics.props import lines as prop_lines
from coaxial.graphics.raster import DOTS_X, DOTS_Y
from coaxial.graphics.shapes import (ellipsoid, limb, loft, moved, sampled, smooth, turn_about,
                                     view)
from machine import ansi, figure
from machine.figure import ANKLE_H, FOREARM, HAIR_AT, HEM_AT, SHANK, THIGH, UPPER_ARM

DISTANCE, CENTRE, REACH = 3.2, (0.0, 0.84, 0.0), 0.92
_SIDES = (('left', 1.0), ('right', -1.0))
_NP = None


def _np():
    global _NP
    if _NP is None:
        from coaxial.model.blocks import numpy as np
        _NP = np
    return _NP


DENIM, TEE = (150, 178, 206), (178, 30, 40)
LOOSE_M, FIT_M, BAGGY_M, HEM_R = 0.02, 0.008, 0.02, (0.074, 0.08, 0.004)
HEM_UP, HEM_LEAN = 0.029, 0.26
JEANS_LEG = ((0.01, 0.07, 0.076, -0.008), (-0.1, 0.072, 0.088, -0.016), (-0.2, 0.074, 0.092, -0.018),
             (HEM_UP - SHANK - ANKLE_H + HEM_AT, HEM_R[0], HEM_R[1], -HEM_R[2], HEM_LEAN))

PAD_FULL_M = 0.002
THIGH_R, SCULPT_M, THIGH_AHEAD = (0.09 + PAD_FULL_M, 0.08, 0.058 + PAD_FULL_M), 0.018, 0.0
THIGH_BEHIND = 0.04
INNER_AT, INNER_R = (-0.036, -0.11, -0.008), (0.022, 0.055, 0.032)
CHEEK_AT, CHEEK_R = (0.056, -0.072, -0.078), (0.06, 0.056, 0.045)


def _wear():
    """Tee, sleeves, and jeans shells."""
    tee, denim, z0, b = paint(TEE), paint(DENIM), (0.0, 0.0, 0.0), BAGGY_M - 0.012
    out = [('cloth_tee', 'torso', z0, loft(_draped(), tee, poles=(HEM_Y - 0.01, 0.378))),
           ('cloth_seat', 'pelvis', z0, loft(
               [_hung(-0.12, 0.095, 0.05, 0.100), _hung(-0.10, 0.11, 0.06, 0.112)]
               + [_hung(*r) for r in _SEAT] + [(0.11, 0.118, 0.084)], denim, poles=(-0.135, 0.118)))]
    for side, x in _SIDES:
        out += [(f'cloth_{side}_bust', 'torso', (BUST_AT[0] * x, BUST_AT[1], BUST_AT[2] + b),
                 ellipsoid(z0, tuple(r + LOOSE_M / 2.0 for r in BUST_R), tee, rows=8)),
                (f'cloth_{side}_inner', f'{side}_thigh', (INNER_AT[0] * x, INNER_AT[1], INNER_AT[2]),
                 ellipsoid(z0, tuple(r + FIT_M for r in INNER_R), denim, rows=8))]
    for side, x in _SIDES:
        out += [(f'cloth_{side}_sleeve', f'{side}_upper_arm', z0, loft(
                    [(0.04, 0.056, 0.056), (0.0, 0.069, 0.065), (-0.07, 0.066, 0.062),
                     (-0.13, 0.06, 0.056)], tee, poles=(0.05, -0.133))),
                (f'cloth_{side}_thigh', f'{side}_thigh', z0,
                 limb(THIGH, THIGH_R[0] + FIT_M, THIGH_R[1] + FIT_M, THIGH_R[2] + LOOSE_M,
                      denim, bulge_at=0.22, out=x * SCULPT_M, ahead=THIGH_AHEAD, behind=THIGH_BEHIND)),
                (f'cloth_{side}_shin', f'{side}_shank', z0, loft(
                    [(0.03, 0.068 + PAD_FULL_M, 0.068 + PAD_FULL_M),
                     (0.0, 0.077 + PAD_FULL_M, 0.077 + PAD_FULL_M), (-0.06, 0.078, 0.076),
                     (-HEM_AT, 0.078, 0.076)], denim, poles=(0.045, -HEM_AT - 0.01))),
                (f'cloth_{side}_leg', f'{side}_shank', ((f'{side}_hem_x', 'x', 1), (f'{side}_hem_z', 'z', 1)),
                 (0.0, -HEM_AT, 0.0), loft(JEANS_LEG, denim, poles=(0.02, JEANS_LEG[-1][0] - 0.02)))]
    return out


_SEAT = ((-0.07, 0.148, 0.084, 0.126), (-0.03, 0.18, 0.096, 0.157), (0.02, 0.178, 0.096, 0.159),
         (0.07, 0.146, 0.088, 0.116))
#: The torso's rings, (y, rx, rz): the tee hangs on them.
_TORSO = ((-0.045, 0.094, 0.066), (0.0, 0.098, 0.069), (0.047, 0.102, 0.072), (0.093, 0.112, 0.079),
          (0.149, 0.122, 0.082), (0.205, 0.126, 0.083), (0.26, 0.13, 0.078), (0.307, 0.138, 0.07),
          (0.344, 0.14, 0.062), (0.372, 0.1, 0.055), (0.39, 0.05, 0.045))
#: The jeans' waistband under the tee's hem, in the torso's frame, (y, rx, front, back), the
#: hem at HEM_Y over it; the tee fitted, EASE_M off the body.
HEM_Y = -0.035
_HIPS = ((HEM_Y, 0.118, 0.084, 0.084), (-0.012, 0.118, 0.084, 0.084))
EASE_M = 0.008


def _hung(y, rx, front, back):
    return (y, rx, (front + back) / 2.0, (front - back) / 2.0)


def _draped():
    """The fitted tee's rings: the torso's each EASE_M off, over the bust where it is, the hem
    over the jeans' waistband; its collar the torso's own."""
    bust = BUST_AT[2] + BUST_R[2] + LOOSE_M / 2.0
    out = [_hung(y, rx + EASE_M, front + EASE_M, back + EASE_M) for y, rx, front, back in _HIPS]
    for y, rx, rz in _TORSO[1:9]:
        front = bust if abs(y - BUST_AT[1]) <= BUST_R[1] else rz
        out.append(_hung(y, rx + EASE_M, max(front, out[-1][2] + out[-1][3] - 0.03) + EASE_M, rz + EASE_M))
    return out + [(0.366, 0.112, 0.064)]


HEAD_Y = 0.095
HAIR, HAIR_ENDS, LIPS, EYES = (118, 80, 52), (222, 168, 100), (192, 112, 112), (46, 46, 58)
PART_M, BALAYAGE = 0.004, 1.2
#: Her ponytail: gathered at the occiput, hanging behind the neck.
HAIR_FALL = ((0.085, 0.030, 0.026, -0.082), (0.05, 0.030, 0.026, -0.094), (0.0, 0.028, 0.024, -0.098),
             (-0.06, 0.026, 0.022, -0.100), (-0.12, 0.020, 0.016, -0.102))
#: The skull's radii and its middle; the hair a cap HAIR_M over it from ear level, its front
#: edge the hairline: at the ears' line below, up the forehead to HAIRLINE of the skull's height.
SKULL_R, SKULL_Z, HAIR_M, HAIRLINE = (0.07, 0.104, 0.09), 0.012, 0.014, 0.65
BUST_AT, BUST_R = (0.056, 0.208, 0.062), (0.062, 0.056, 0.052)
HUNG = (('hair_x', 'x', 1), ('hair_z', 'z', 1))


def _features():
    z0 = (0.0, 0.0, 0.0)
    out = [('nose', 'head', z0, ellipsoid((0.0, HEAD_Y - 0.004, 0.1), (0.011, 0.022, 0.016), SKIN, rows=6)),
           ('lips', 'head', z0, ellipsoid((0.0, HEAD_Y - 0.048, 0.094), (0.02, 0.007, 0.01), paint(LIPS), rows=6)),
           ('hair', 'head', z0, _cap()),
           ('hair_fall', 'head', HUNG, HAIR_AT, _hair(HAIR_FALL, 0.0, 0.03))]
    for side, x in _SIDES:
        out += [(f'{side}_ear', 'head', z0,
                 ellipsoid((0.071 * x, HEAD_Y - 0.004, 0.006), (0.009, 0.028, 0.018), SKIN, rows=6)),
                (f'{side}_eye', 'head', z0,
                 ellipsoid((0.029 * x, HEAD_Y + 0.016, 0.092), (0.013, 0.006, 0.006), paint(EYES), rows=6))]
    return out


def _cap():
    """The hair over the skull: rings HAIR_M off it from ear level to the crown, each ring's
    front held back to the hairline, from the ears' line to the forehead."""
    rx, ry, rz = (r + HAIR_M for r in SKULL_R)
    rings = []
    for k in range(9):
        u = -0.2 + 1.15 * k / 8.0
        s = math.sqrt(max(0.0, 1.0 - u * u))
        s_line = math.sqrt(max(0.0, 1.0 - HAIRLINE * HAIRLINE))
        front = min(SKULL_Z + rz * s, (SKULL_Z + rz * s_line) * min(1.0, (u + 0.2) / (HAIRLINE + 0.2)))
        back = SKULL_Z - rz * s
        rings.append((HEAD_Y + u * ry, rx * s, (front - back) / 2.0, (front + back) / 2.0 - PART_M))
    return loft(rings, paint(HAIR), poles=(rings[0][0] - 0.02, HEAD_Y + ry + 0.004))


def _hair(rings, x, crown):
    ax, ay, az = HAIR_AT
    top, end = rings[0][0] - ay, rings[-1][0] - ay
    corners, triangles, uv, materials = loft(
        [(y - ay, rx, rz, z - az) for y, rx, rz, z in rings], lambda c: _balayage(c, top, end),
        poles=(top + crown, end - 0.01))
    corners[:, 0] += x - ax
    return corners, triangles, uv, materials


def _balayage(corners, top, end):
    np = _np()
    u = np.clip((top - corners[:, 1]) / (top - end), 0.0, 1.0)[:, None] ** BALAYAGE
    rgb = (np.asarray(HAIR) * (1.0 - u) + np.asarray(HAIR_ENDS) * u).astype(int)
    return PAINTED | rgb[:, 0] << 16 | rgb[:, 1] << 8 | rgb[:, 2]


def _face(corners):
    np = _np()
    y, z = corners[:, 1] - HEAD_Y, corners[:, 2] - 0.012
    return np.where((z > 0.028) & (y < 0.07) & (y > -0.12), SKIN, MESH)


def _core(corners):
    np = _np()
    return np.where((corners[:, 1] > 0.03) & (corners[:, 1] < 0.10) & (corners[:, 2] > 0.035),
                    CORE, MESH)


def _meshes():
    """({segment: mesh}, [(name, parent, offset, mesh), ...]) for the figure and extra parts."""
    z0 = (0.0, 0.0, 0.0)
    pelvis = loft([(-0.10, 0.06, 0.05), (-0.07, 0.135, 0.097, -0.0215),
                   (-0.03, 0.168, 0.115, -0.0275), (0.02, 0.166, 0.117, -0.0295), (0.07, 0.14, 0.092, -0.012),
                   (0.11, 0.114, 0.074), (0.13, 0.098, 0.068)], PLATE, poles=(-0.115, 0.14))
    torso = loft(list(_TORSO), _core, poles=(-0.055, 0.40))
    meshes = {'pelvis': pelvis, 'torso': torso,
              'neck': loft([(0.0, 0.055, 0.052), (0.035, 0.05, 0.048), (0.07, 0.05, 0.048)],
                           SKIN, poles=(-0.01, 0.075)),
              'head': ellipsoid((0.0, HEAD_Y, SKULL_Z), SKULL_R, _face, rows=12)}
    extra = [('jaw', 'head', z0, ellipsoid((0.0, 0.042, 0.03), (0.047, 0.048, 0.056), SKIN))] + _features()
    for side, x in _SIDES:
        for tag, par, (ax, ay, az), rad, mat in (
                ('bust', 'torso', BUST_AT, BUST_R, PLATE), ('cheek', 'pelvis', CHEEK_AT, CHEEK_R, PLATE),
                ('inner', f'{side}_thigh', INNER_AT, INNER_R, MESH),
                ('cap', 'torso', (0.135, 0.335, -0.004), (0.05, 0.042, 0.046), PLATE)):
            extra.append((f'{side}_{tag}', par, (ax * x, ay, az), ellipsoid(z0, rad, mat, rows=8)))
        meshes.update({
            f'{side}_upper_arm': limb(UPPER_ARM, 0.04, 0.038, 0.040 + PAD_FULL_M, MESH, flat=0.95),
            f'{side}_forearm': limb(FOREARM, 0.040 + PAD_FULL_M, 0.028, 0.02, MESH, flat=0.9),
            f'{side}_hand': ellipsoid((0.0, -0.043, 0.004), (0.014, 0.047, 0.032), PLATE, rows=8),
            f'{side}_fingers': ellipsoid((0.0, -0.035, 0.0), (0.011, 0.042, 0.028), PLATE, rows=8),
            f'{side}_thigh': limb(THIGH, THIGH_R[0], THIGH_R[1], THIGH_R[2], MESH, bulge_at=0.22,
                                  out=x * SCULPT_M, ahead=THIGH_AHEAD, behind=THIGH_BEHIND),
            f'{side}_shank': limb(SHANK, 0.06 + PAD_FULL_M, 0.062, 0.034, PLATE, bulge_at=0.3, ahead=0.008),
            **sneaker.feet(side)})
        extra += sneaker.soles(side)
    return meshes, extra + drums.stages() + drums.drums() + _wear()


def _worn(name):
    return name.startswith(('cloth_', 'hair')) or name.endswith('_sole')


def _parts(dressed=True):
    meshes, extra = _meshes()
    meshes = panels.over(meshes)
    if not dressed:
        extra = [part for part in extra if not _worn(part[0])] + mechanism.bands(meshes)
        for name, (c, t, u, _m) in list(meshes.items()):
            if name.endswith(('_foot', '_toes')):
                meshes[name] = (c, t, u, _np().full(len(c), PLATE))
    out = [(seg[0], seg[1], seg[2], seg[3], seg[4], meshes[seg[0]]) for seg in figure.SEGMENTS]
    return out + [(name, parent, joints, offset, 0.0, mesh)
                  for name, parent, *rest in extra for joints, offset, mesh in [_split(rest)]]


def _split(rest) -> tuple[Any, Any, Any]:
    return (rest[0], rest[1], rest[2]) if len(rest) == 3 else ((), rest[0], rest[1])


class Body:

    """The parts' meshes laid end to end once, normals smoothed; `pose(angles)` turns them."""

    def __init__(self, dressed=True, see=None):
        np = _np()
        self.parts: list[Any] = mechanism.parts(see == 'actuators') if see else _parts(dressed)
        self.see = see
        cs, ts, us, ms, spans, faces, base = [], [], [], [], [], [], 0
        for i, (*_head, (c, t, u, m)) in enumerate(self.parts):
            cs.append(c)
            ts.append(t + base)
            us.append(u)
            ms.append(m)
            spans.append((base, base + len(c)))
            faces.append(np.full(len(t), i))
            base += len(c)
        self.corners = np.vstack(cs)
        self.index = np.vstack(ts).astype(np.uint32)
        self.uv, self.materials = np.vstack(us), np.concatenate(ms).astype(np.uint32)
        self.spans = spans
        self.normals = smooth(self.corners, self.index)
        self.soles = [i for i, p in enumerate(self.parts) if p[0].endswith(('_foot', '_toes'))]
        self.drums = {p[0][len('drive_'):]: i for i, p in enumerate(self.parts) if p[0].startswith('drive_')}
        self.patches = drums.patches(self.parts)
        self.dense, self.dense_spans = sampled(self, np.concatenate(faces))
        self.dense_near = drums.near(self.patches, self.parts, self.drums, self.dense, self.dense_spans)
        self.bends = drape.table(self.parts, self.corners, self.spans, self.dense[0], self.dense_spans)
        self._rest_pivots = self.pivots({})

    def _frames(self, angles, root=None):
        np = _np()
        where, turn = root if root is not None else ((0.0, 0.0, 0.0), np.eye(3))
        spread = float(angles.get('arms_out', 0.0))
        placed, out = {}, []
        for name, parent, joints, offset, rest, _mesh in self.parts:
            if parent == '*':
                out.append(None)
                continue
            if parent:
                above, at = placed[parent]
                spot = at + above @ np.asarray(offset, float)
                if spread and name.endswith('_upper_arm'):
                    rest = rest + math.copysign(spread, rest)
                here = above @ turn_about('z', rest) if rest else above
            else:
                spot, here = np.asarray(where, float), np.asarray(turn, float)
            here: Any = here
            for joint, axis, sign in joints:
                deg = sign * float(angles.get(joint, 0.0))
                if deg:
                    here = here @ turn_about(axis, deg)
            placed[name] = (here, spot)
            out.append(placed[name])
        if self.see:
            mechanism.posed(self.parts, placed, angles, out)
        return out

    def _floor(self, frames):
        return min((float((self.corners[slice(*self.spans[i])] @ frames[i][0][1]).min() + frames[i][1][1])
                    for i in self.soles if self.spans[i][1] > self.spans[i][0]), default=0.0)

    def pivots(self, angles, root=None, _frames=None):
        frames = _frames if _frames is not None else self._frames(angles, root)
        floor = _np().array([0.0, self._floor(frames) if root is None else 0.0, 0.0])
        return {joint: spot - floor for (_n, _p, joints, *_r), (_t, spot)
                in zip(self.parts, frames) for joint, _a, _s in joints}

    def pose(self, angles, dense=False, root=None, _frames=None):
        frames = _frames if _frames is not None else self._frames(angles, root)
        floor = 0.0 if root is not None else self._floor(frames)
        src = (self.dense[0], self.dense[1], self.dense_spans) if dense else (
            self.corners, self.normals, self.spans)
        pos, norm = moved(*src, frames)
        drape.bent(pos, norm, src[0], src[2], frames, self.bends, dense)
        if floor:
            pos[:, 1] -= floor
        return pos, norm


def _band(depth, width):
    across = _np().nonzero((depth > 0.0).any(axis=0))[0]
    if not len(across):
        return None
    return max(0, int(across[0]) // DOTS_X - 1), min(width, int(across[-1]) // DOTS_X + 2)


_BODY = {}


def body(dressed=True, see=None):
    got = _BODY.get((dressed, see))
    if got is None:
        got = _BODY[(dressed, see)] = Body(dressed, see)
    return got


class Follow:

    """A camera's place over the floor, (x, z), tracking mean velocity and position."""

    def __init__(self, seconds=1.0):
        self.seconds, self.at = seconds, None

    def __call__(self, place, speed, t):
        if self.at is None or math.dist(place, self.at[0]) > 1.0 or t < self.at[2]:
            self.at = (tuple(place), tuple(speed), t)
            return self.at[0]
        at, mean, then = self.at
        if t == then:
            return at
        on, pull = min(1.0, (t - then) / self.seconds), min(1.0, (t - then) / (4.0 * self.seconds))
        mean = tuple(m + (s - m) * on for m, s in zip(mean, speed))
        at = tuple(a + m * (t - then) + (p - a) * pull for a, m, p in zip(at, mean, place))
        self.at = (at, mean, t)
        return at


MARK = (((0, 0), '\u253c'), ((-1, 0), '\u2500'), ((-2, 0), '\u2500'), ((1, 0), '\u2500'),
        ((2, 0), '\u2500'), ((0, -1), '\u2502'), ((0, 1), '\u2502'))


def _marked(overlay, mark, m, cam, centre, travel, width, height):
    np = _np()
    point, ink = mark
    at = np.asarray(point, float) - (travel[0], 0.0, travel[1])
    sx, sy, w = project(at[None, :], m, cam, centre)
    if w[0] <= 0.0:
        return overlay
    out = dict(overlay or {})
    col, row = int(sx[0] // DOTS_X), int(sy[0] // DOTS_Y)
    for (dc, dr), char in MARK:
        if 0 <= row + dr < height and 0 <= col + dc < width:
            out[(row + dr, col + dc)] = (ord(char), packed(ink, None))
    return out


def _heated(who, heat):
    if not heat:
        return who.materials, who.dense[3]
    mats, dense = who.materials.copy(), who.dense[3].copy()
    for joint, celsius in heat.items():
        if joint not in who.drums:
            continue
        worn, i = paint(ansi.thermal_rgb(celsius)), who.drums[joint]
        mats[slice(*who.spans[i])] = worn
        dense[slice(*who.dense_spans[i])] = worn
        for part, corners in who.patches.get(joint, ()):
            mats[who.spans[part][0] + corners] = worn
            lo, hi = who.dense_spans[part]
            dense[lo:hi][who.dense_near[joint][part]] = worn
    return mats, dense


def _wired(who, frames, root, m, fine, centre):
    np = _np()
    depth = np.zeros((int(fine['height']), int(fine['width'])))
    rgb, layers = np.zeros(depth.shape + (3,)), mechanism.wires(who.parts, frames)
    low = 0.0 if root is not None else min(min(a[:, 1].min(), b[:, 1].min()) for a, b, _ink in layers)
    wired = []
    for a, b, ink in layers:
        ax, ay, aw = project(a - (0.0, low, 0.0), m, fine, centre)
        bx, by, bw = project(b - (0.0, low, 0.0), m, fine, centre)
        dots = np.zeros(depth.shape, bool)
        for k in np.flatnonzero((aw > 0.0) & (bw > 0.0)):
            line(dots, (ax[k], ay[k]), (bx[k], by[k]))
        wired.append((dots, ink))
    return depth, rgb, wired


def render(angles, width, height, yaw=30.0, pitch=8.0, zoom=1.0, colour=True, travel=(0.0, 0.0),
           lit=None, root=None, labels=None, heat=None, props=None, legend=None, dressed=True, dents=None,
           mark=None, around=False, see=None, pan=(0.0, 0.0), inset=None, arrows=None, horizon=None):
    """Render the gynoid at `angles` (`{joint: deg}`) into `width` x `height` braille lines;
    `horizon` [(segments, ink)] on it (`coaxial.graphics.horizon`)."""
    np, who, m = _np(), body(dressed, see), view(yaw, pitch)
    fine = engine.fine(engine.camera(width, height, REACH, distance=DISTANCE, zoom=zoom))
    centre = np.asarray(CENTRE) + np.asarray(pan) @ np.reshape(m, (3, 3))[:2]
    mats, dense = _heated(who, heat)
    frames, wired = who._frames(angles, root), []
    if see == 'actuators':
        depth, rgb, wired = _wired(who, frames, root, m, fine, centre)
    elif lit is not None:
        pos, norm = who.pose(angles, root=root, _frames=frames)
        depth, rgb = lit.raster(pos, norm, who.uv, mats, who.index, m, fine, centre, REACH * 1.4)
    else:
        pos, norm = who.pose(angles, dense=True, root=root, _frames=frames)
        depth, rgb = splat((pos, norm, dense, who.dense[2]), m, fine, centre)
    overlay = leaders = None
    if labels:
        pivots, rest = who.pivots(angles, root=root, _frames=frames), who._rest_pivots
        names = [j for j in labels if j in pivots]

        def dots_of(points):
            sx, sy, _w = project(points, m, fine, centre)
            return list(zip(sx.tolist(), sy.tolist()))
        overlay, leaders = callouts(labels, dict(zip(names, dots_of([pivots[j] for j in names]))),
                                    dict(zip(names, dots_of([rest[j] for j in names]))),
                                    width, height, height - (1 if legend else 0),
                                    _band(depth, width) if around else None)
    if legend:
        overlay = dict(overlay or {})
        for col, (char, fg, bg) in enumerate(legend[:width]):
            overlay[(height - 1, col)] = (ord(char), packed(fg, bg))
    if mark is not None:
        overlay = _marked(overlay, mark, m, fine, centre, travel, width, height)
    if inset:
        overlay = {**(overlay or {}), **inset}
    return braille(depth, rgb, grid(m, fine, centre, travel), width, height, colour, overlay,
                   leaders, prop_lines(props or (), m, fine, centre, travel, dents)
                   + _arrows.layers(arrows, m, fine, centre, travel) + wired
                   + [(dots, ink) for segs, ink in (horizon or ())
                      for dots in (_arrows._rasterize(segs, m, fine, centre, travel),) if dots is not None])

