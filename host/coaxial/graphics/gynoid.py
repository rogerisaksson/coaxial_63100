"""The gynoid: a slender body on `machine.figure`'s joints, posed, lit, drawn in braille.

    lines = render(angles, 96, 40, yaw=30, lit=gpu.LitRaster())   # the GPU's lit raster
    lines = render(angles, 96, 40, root=(where, turn))             # the pelvis placed, world
    lines = render(angles, 96, 40, labels={'left_knee': cells})   # called out at the edge

A part is a closed loft or ellipsoid in its own frame, hung off its parent at the figure's offset
and turned by its joints - the figure's names and signs. Without `root`, the lowest point of the
feet stands on the floor. 1.69 m tall; the lattice, the glowing core and the plates are the
materials `gpu.LIT_WGSL` lights. The floor scrolls under her by `travel` metres. Each drive's
assembly (`machine.drives`) is a drum on its joint's axis, or where it is mounted (`drums`). She
wears a tee, jeans and sneakers (`_wear`); a drum under the cloth shows as a patch sewn on at its
ends.
"""
import math
from typing import Any

from coaxial.graphics import drums, engine
from coaxial.graphics.callouts import callouts, line, packed
from coaxial.graphics.lit import (CORE, MESH, PAINTED, PLATE, SKIN, braille, grid, paint,
                                  project, splat)
from coaxial.graphics.raster import DOTS_X
from coaxial.graphics.shapes import (ellipsoid, limb, loft, moved, sampled, smooth, turn_about,
                                     view)
from machine import ansi, figure
from machine.figure import FOREARM, HAIR_AT, HEM_AT, TOE_M, TOE_RY, UPPER_ARM
from machine.gait import ANKLE_H, BALL, HEEL, SHANK, THIGH

#: The camera: its distance in the engine's units, the point it turns about (her middle, metres
#: over the floor), and how far out she reaches from it.
DISTANCE, CENTRE, REACH = 3.2, (0.0, 0.84, 0.0), 0.92


def _np():
    from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
    return np


#: Her clothes' colours, and how far out of her they hang, m: high-waisted jeans,
#: a white tee, white sneakers; the jeans LOOSE_M out over the seat, FIT_M the thighs, the tee
#: BAGGY_M, patched over the drums (`drums.PATCH_M`). The jeans' legs (`JEANS_LEG`) hang on
#: their hems' hinges (`physics.HEMS`) straight to a hem HEM_R (half width, half depth, set back),
#: HEM_UP up at its sides, leaning HEM_LEAN onto the sneaker's vamp: 88 mm across and level, the
#: other foot passed 16 mm into it, the toe box 35 mm out of it (2026-09-28).
DENIM, TEE, SNEAKER = (118, 150, 182), (230, 230, 226), (236, 236, 232)
LOOSE_M, FIT_M, BAGGY_M, HEM_R = 0.02, 0.008, 0.02, (0.074, 0.08, 0.004)
HEM_UP, HEM_LEAN = 0.029, 0.26
JEANS_LEG = ((0.01, 0.07, 0.068), (-0.1, 0.072, 0.072), (-0.2, 0.074, 0.076, -0.002),
             (HEM_UP - SHANK - ANKLE_H + HEM_AT, HEM_R[0], HEM_R[1], -HEM_R[2], HEM_LEAN))


#: The thighs' radii, m: at the hip, at their fullest and at the knee; their middle SCULPT_M out,
#: the inside drawn in off the other's, the outside full.
THIGH_R, SCULPT_M = (0.068, 0.06, 0.054), 0.008

#: The inner thigh's fullness high under the seat, 6 mm proud of the thigh (INNER_AT the left's,
#: on its thigh; INNER_R), the jeans FIT_M over it; the seat's two cheeks low on it (CHEEK_AT the
#: left's; CHEEK_R), 6 mm proud of the pelvis and inside the jeans' seat. At 2.6 cm, the jeans
#: over them, they read too big (the user, 2026-09-30).
INNER_AT, INNER_R = (-0.036, -0.11, -0.008), (0.022, 0.055, 0.032)
CHEEK_AT, CHEEK_R = (0.042, -0.065, -0.055), (0.055, 0.055, 0.04)


def _wear():
    """[(name, parent, offset, mesh)] or with joints: the tee, full over the bust, its sleeves
    to mid upper arm; the jeans over the seat, the thighs, the shins to HEM_AT under the knee,
    each leg on its hem's hinges on down - shells LOOSE_M out of her."""
    tee, denim = paint(TEE), paint(DENIM)
    b = BAGGY_M - 0.012
    out = [('cloth_tee', 'torso', (0.0, 0.0, 0.0), loft(
        [_hung(y, rx + b, front, back + b) for y, rx, front, back in _TANK]
        + [(0.307, 0.150 + b, 0.082 + b), (0.344, 0.152 + b, 0.074 + b), (0.366, 0.112, 0.064)],
        tee, poles=(-0.036, 0.378))),
           ('cloth_seat', 'pelvis', (0.0, 0.0, 0.0), loft(
               [(-0.10, 0.072, 0.065)] + [_hung(*ring) for ring in _SEAT]
               + [(0.11, 0.112, 0.082)], denim, poles=(-0.12, 0.118)))]
    for side, x in (('left', 1.0), ('right', -1.0)):
        out += [('cloth_%s_bust' % side, 'torso', (BUST_AT[0] * x, BUST_AT[1], BUST_AT[2] + b),
                 ellipsoid((0.0, 0.0, 0.0), tuple(r + LOOSE_M / 2.0 for r in BUST_R), tee,
                            rows=8)),
                ('cloth_%s_inner' % side, side + '_thigh',
                 (INNER_AT[0] * x, INNER_AT[1], INNER_AT[2]),
                 ellipsoid((0.0, 0.0, 0.0), tuple(r + FIT_M for r in INNER_R), denim, rows=8))]
    for side, x in (('left', 1.0), ('right', -1.0)):
        out += [('cloth_%s_sleeve' % side, side + '_upper_arm', (0.0, 0.0, 0.0), loft(
            [(0.04, 0.042, 0.042), (0.0, 0.054, 0.05), (-0.07, 0.052, 0.048),
             (-0.13, 0.05, 0.046)], tee, poles=(0.05, -0.133))),
                ('cloth_%s_thigh' % side, side + '_thigh', (0.0, 0.0, 0.0),
                 limb(THIGH, THIGH_R[0] + FIT_M, THIGH_R[1] + FIT_M, THIGH_R[2] + LOOSE_M,
                      denim, bulge_at=0.22, out=x * SCULPT_M)),
                ('cloth_%s_shin' % side, side + '_shank', (0.0, 0.0, 0.0), loft(
                    [(0.03, 0.054, 0.054), (0.0, 0.066, 0.066), (-0.06, 0.07, 0.068),
                     (-HEM_AT, 0.072, 0.07)], denim, poles=(0.045, -HEM_AT - 0.01))),
                ('cloth_%s_leg' % side, side + '_shank', ((side + '_hem_x', 'x', 1),
                                                          (side + '_hem_z', 'z', 1)),
                 (0.0, -HEM_AT, 0.0), loft(JEANS_LEG, denim,
                                           poles=(0.02, JEANS_LEG[-1][0] - 0.02)))]
    return out


#: The tee's rings under its shoulders, (y, half width, front, back) m: its front hangs from
#: the bust's apex (`BUST_AT`, `BUST_R`) nearly plumb to the hem. The jeans' seat's: flat over
#: the belly, full over the seat.
_TANK = ((-0.03, 0.104, 0.104, 0.077), (0.047, 0.108, 0.108, 0.080), (0.093, 0.117, 0.11, 0.085),
         (0.149, 0.130, 0.112, 0.089), (0.205, 0.138, 0.116, 0.094), (0.26, 0.142, 0.104, 0.090))
_SEAT = ((-0.07, 0.132, 0.086, 0.103), (-0.03, 0.167, 0.088, 0.126), (0.02, 0.165, 0.088, 0.115),
         (0.07, 0.137, 0.086, 0.093))


def _hung(y, rx, front, back):
    """A loft's ring at `y`, `rx` wide (half), reaching `front` m forward and `back` m back."""
    return (y, rx, (front + back) / 2.0, (front - back) / 2.0)


#: The head's centre over the head joint, metres.
HEAD_Y = 0.095


#: Her hair, lips and eyes: a full wavy lob to just above her shoulders, parted aside PART_M,
#: dark at the roots (HAIR) and caramel toward the ends (HAIR_ENDS). Rings (y, half width, half
#: depth, its centre's z) in her head's frame, hung from `figure.HAIR_AT` on the hair's hinges:
#: the fall round the back of her head and neck, its front inside them, and a lock HAIR_LOCK_X
#: either side framing her face, clear of her jaw and her neck; each wider and narrower down its
#: length, a wave.
HAIR, HAIR_ENDS, LIPS, EYES = (118, 80, 52), (222, 168, 100), (192, 112, 112), (46, 46, 58)
PART_M, BALAYAGE = 0.004, 1.2
HAIR_FALL = ((0.10, 0.104, 0.08, -0.053), (0.06, 0.106, 0.079, -0.056),
             (0.02, 0.111, 0.076, -0.06), (-0.01, 0.108, 0.073, -0.063),
             (-0.045, 0.114, 0.068, -0.066))
HAIR_LOCK = ((0.12, 0.022, 0.043, 0.022), (0.09, 0.025, 0.048, 0.027), (0.06, 0.023, 0.049, 0.026),
             (0.03, 0.027, 0.049, 0.024), (0.0, 0.024, 0.048, 0.02), (-0.02, 0.028, 0.046, 0.016),
             (-0.045, 0.026, 0.042, 0.012))
HAIR_LOCK_X = 0.084

#: The bust's centre (the left's) on the torso and its radii, m: its apex 2 cm before the chest;
#: 3.2 cm out and near round it read as spheres (the user, 2026-09-30).
BUST_AT, BUST_R = (0.055, 0.208, 0.058), (0.055, 0.05, 0.045)


def _features():
    """[(name, parent, offset, mesh)] or with joints, on her head: the nose, the ears, the eyes
    and the lips, and her hair - a cap over the skull behind the face, parted aside, and a fall
    and two locks to her shoulders on the hair's hinges."""
    out = [('nose', 'head', (0.0, 0.0, 0.0),
            ellipsoid((0.0, HEAD_Y - 0.004, 0.1), (0.011, 0.022, 0.016), SKIN, rows=6)),
           ('lips', 'head', (0.0, 0.0, 0.0),
            ellipsoid((0.0, HEAD_Y - 0.048, 0.094), (0.02, 0.007, 0.01), paint(LIPS), rows=6)),
           ('hair', 'head', (0.0, 0.0, 0.0),
            ellipsoid((-PART_M, HEAD_Y + 0.036, -0.018), (0.094, 0.11, 0.11), paint(HAIR))),
           ('hair_fall', 'head', HUNG, HAIR_AT, _hair(HAIR_FALL, 0.0, 0.03))]
    for side, x in (('left', 1.0), ('right', -1.0)):
        out += [('%s_lock' % side, 'head', HUNG, HAIR_AT, _hair(HAIR_LOCK, HAIR_LOCK_X * x, 0.01))]
        out += [('%s_ear' % side, 'head', (0.0, 0.0, 0.0),
                 ellipsoid((0.071 * x, HEAD_Y - 0.004, 0.006), (0.009, 0.028, 0.018), SKIN,
                            rows=6)),
                ('%s_eye' % side, 'head', (0.0, 0.0, 0.0),
                 ellipsoid((0.029 * x, HEAD_Y + 0.016, 0.092), (0.013, 0.006, 0.006),
                            paint(EYES), rows=6))]
    return out


#: The hair's hinges (`physics.HAIRS`), as a part rides them.
HUNG = (('hair_x', 'x', 1), ('hair_z', 'z', 1))


def _hair(rings, x, crown):
    """A body of hair through `rings` (y, half width, half depth, centre's z; her head's frame)
    moved `x` aside, capped `crown` over its top ring, in the frame of its hinges at HAIR_AT."""
    ax, ay, az = HAIR_AT
    top, end = rings[0][0] - ay, rings[-1][0] - ay
    corners, triangles, uv, materials = loft(
        [(y - ay, rx, rz, z - az) for y, rx, rz, z in rings], lambda c: _balayage(c, top, end),
        poles=(top + crown, end - 0.01))
    corners[:, 0] += x - ax
    return corners, triangles, uv, materials


def _balayage(corners, top, end):
    """The hair's paint by corner: HAIR at `top`, HAIR_ENDS at `end` and below, lighter down
    its length as the way to BALAYAGE."""
    np = _np()
    u = np.clip((top - corners[:, 1]) / (top - end), 0.0, 1.0)[:, None] ** BALAYAGE
    rgb = (np.asarray(HAIR) * (1.0 - u) + np.asarray(HAIR_ENDS) * u).astype(int)
    return PAINTED | rgb[:, 0] << 16 | rgb[:, 1] << 8 | rgb[:, 2]


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
    pelvis = loft([(-0.10, 0.055, 0.048), (-0.07, 0.115, 0.08, -0.006),
                    (-0.03, 0.15, 0.095, -0.014), (0.02, 0.148, 0.09, -0.008), (0.07, 0.122, 0.078),
                    (0.11, 0.1, 0.07), (0.13, 0.094, 0.066)], PLATE, poles=(-0.115, 0.14))
    # From 4.5 cm inside the pelvis, so the waist does not open as the spine bends.
    torso = loft([(-0.045, 0.09, 0.063), (0.0, 0.094, 0.066), (0.047, 0.096, 0.068),
                   (0.093, 0.105, 0.074),
                   (0.149, 0.118, 0.08), (0.205, 0.126, 0.083), (0.26, 0.13, 0.078),
                   (0.307, 0.138, 0.07), (0.344, 0.14, 0.062), (0.372, 0.1, 0.055),
                   (0.39, 0.05, 0.045)], _core, poles=(-0.055, 0.40))
    meshes = {'pelvis': pelvis, 'torso': torso,
              'neck': loft([(0.0, 0.046, 0.043), (0.035, 0.040, 0.038), (0.07, 0.041, 0.039)],
                            SKIN, poles=(-0.01, 0.075)),
              'head': ellipsoid((0.0, HEAD_Y, 0.012), (0.07, 0.104, 0.09), _face, rows=12)}
    extra = [('jaw', 'head', (0.0, 0.0, 0.0),
              ellipsoid((0.0, 0.042, 0.03), (0.047, 0.048, 0.056), SKIN))] + _features()
    for side, x in (('left', 1.0), ('right', -1.0)):
        extra += [('%s_bust' % side, 'torso', (BUST_AT[0] * x, BUST_AT[1], BUST_AT[2]),
                   ellipsoid((0.0, 0.0, 0.0), BUST_R, PLATE, rows=8)),
                  ('%s_cheek' % side, 'pelvis', (CHEEK_AT[0] * x, CHEEK_AT[1], CHEEK_AT[2]),
                   ellipsoid((0.0, 0.0, 0.0), CHEEK_R, PLATE, rows=8)),
                  ('%s_inner' % side, side + '_thigh', (INNER_AT[0] * x, INNER_AT[1], INNER_AT[2]),
                   ellipsoid((0.0, 0.0, 0.0), INNER_R, MESH, rows=8)),
                  ('%s_cap' % side, 'torso', (0.135 * x, 0.335, -0.004),
                   ellipsoid((0.0, 0.0, 0.0), (0.042, 0.036, 0.04), PLATE, rows=8))]
        meshes.update({
            side + '_upper_arm': limb(UPPER_ARM, 0.031, 0.03, 0.024, MESH, flat=0.95),
            side + '_forearm': limb(FOREARM, 0.025, 0.025, 0.018, MESH, flat=0.9),
            side + '_hand': ellipsoid((0.0, -0.043, 0.004), (0.014, 0.047, 0.032), PLATE, rows=8),
            side + '_fingers': ellipsoid((0.0, -0.035, 0.0), (0.011, 0.042, 0.028), PLATE, rows=8),
            side + '_thigh': limb(THIGH, THIGH_R[0], THIGH_R[1], THIGH_R[2], MESH, bulge_at=0.22,
                                  out=x * SCULPT_M),
            side + '_shank': limb(SHANK, 0.052, 0.058, 0.032, PLATE, bulge_at=0.3),
            side + '_foot': loft([(z, rx, rv, ANKLE_H - rv) for z, rx, rv in _SHOE],
                                  paint(SNEAKER), poles=(-HEEL, BALL + 0.006), along='z'),
            side + '_toes': loft([(z, rx, rv, -(rv - TOE_RY + _spring(z)))
                                   for z, rx, rv in _TOE_CAP],
                                  paint(SNEAKER), poles=(-0.006, TOE_M + 0.002), along='z')})
        extra += _soles(side)
    return meshes, extra + drums.drums() + _wear()


def _worn(name):
    """Whether a part is worn: her clothes, her hair and her sneakers' soles."""
    return name.startswith(('cloth_', 'hair')) or name.endswith(('_lock', '_sole'))


def _parts(dressed=True):
    """(name, parent, joints ((joint, axis, sign), ..), offset, rest turn about z (deg), mesh):
    the figure's segments, then the parts it carries - on joints of their own, the jeans' legs on
    their hems' - parents first. Undressed, her shell: nothing worn, the feet plated."""
    meshes, extra = _meshes()
    if not dressed:
        extra = [part for part in extra if not _worn(part[0])]
        for name, (c, t, u, _m) in list(meshes.items()):
            if name.endswith(('_foot', '_toes')):
                meshes[name] = (c, t, u, _np().full(len(c), PLATE))
    out = [(seg[0], seg[1], seg[2], seg[3], seg[4], meshes[seg[0]]) for seg in figure.SEGMENTS]
    return out + [(name, parent, joints, offset, 0.0, mesh)
                  for name, parent, *rest in extra for joints, offset, mesh in [_split(rest)]]


def _split(rest) -> tuple[Any, Any, Any]:
    """(joints, offset, mesh) of an extra part's (offset, mesh) or (joints, offset, mesh)."""
    if len(rest) == 3:
        return rest[0], rest[1], rest[2]
    return (), rest[0], rest[1]


#: A sneaker, size 37-38: the shoe's rings forward of the ankle, (z, half width, half height),
#: each hung so its bottom is the sole, flat ANKLE_H under the ankle, as the walk plants it - the
#: collar round the ankle, the tongue over the instep, the laces down to the ball; the toe cap's
#: from the ball, its sole sprung SPRING_M up at its tip `figure.TOE_M` ahead. Its sole SOLE_M
#: deep, gum.
_SHOE = ((-0.05, 0.027, 0.031), (-0.028, 0.03, 0.03), (0.0, 0.033, 0.029),
         (0.033, 0.035, 0.031), (0.066, 0.038, 0.025), (0.095, 0.04, 0.02), (BALL, 0.04, 0.0175))
_TOE_CAP = ((0.0, 0.04, 0.0175), (0.019, 0.039, 0.0165), (0.035, 0.035, 0.0145),
            (0.049, 0.028, 0.012), (0.056, 0.018, 0.008))
SPRING_M, SOLE_M, SOLE_PROUD, GUM = 0.012, 0.02, 0.002, (196, 150, 100)


def _spring(z):
    """How far the toe cap's sole lifts off the floor `z` m ahead of the ball."""
    return SPRING_M * max(0.0, z / TOE_M) ** 2


def _soles(side):
    """[(name, parent, offset, mesh)]: the sneaker's gum sole under the shoe and under the toe
    cap, SOLE_M deep and SOLE_PROUD wider than the white above it."""
    gum, h = paint(GUM), SOLE_M / 2.0
    return [(side + '_sole', side + '_foot', (0.0, 0.0, 0.0), loft(
        [(z, rx + SOLE_PROUD, h, ANKLE_H - h) for z, rx, _rv in _SHOE], gum,
        poles=(-HEEL - SOLE_PROUD, BALL + 0.006), along='z')),
            (side + '_toe_sole', side + '_toes', (0.0, 0.0, 0.0), loft(
                [(z, rx + SOLE_PROUD, h, -(h - TOE_RY + _spring(z))) for z, rx, _rv in _TOE_CAP],
                gum, poles=(-0.006, TOE_M + SOLE_PROUD), along='z'))]


class Body:

    """The parts' meshes laid end to end once, with their smooth normals; `pose(angles)` turns them."""

    def __init__(self, dressed=True):
        np = _np()
        self.parts = _parts(dressed)
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
        self.normals = smooth(self.corners, self.index)
        #: The parts whose lowest corner is the sole; each drive's drum's, by its joint.
        self.soles = [i for i, part in enumerate(self.parts)
                      if part[0].endswith(('_foot', '_toes'))]
        self.drums = {part[0][len('drive_'):]: i for i, part in enumerate(self.parts)
                      if part[0].startswith('drive_')}
        #: Each drum's patches on the cloth over it: [(part, its corners')] by joint.
        self.patches = drums.patches(self.parts)
        #: Every triangle sampled DENSE_M apart, for the dots drawn without a card: (points,
        #: normals, uv, materials) in their parts' frames, and each part's span of them.
        self.dense, self.dense_spans = sampled(self, np.concatenate(faces))
        #: The same patches among the dots drawn without a card: {joint: {part: mask}}.
        self.dense_near = drums.near(self.patches, self.parts, self.drums, self.dense,
                                     self.dense_spans)

    def _frames(self, angles, root=None):
        """Each part's (turn, spot) in the world for {joint: degrees}; `root` the pelvis's (place,
        turn 3x3), else at the origin, upright. `angles['arms_out']`, degrees, spreads the upper
        arms out from their rest, drawn only - her shoulders have no such joint."""
        np = _np()
        where, turn = root if root is not None else ((0.0, 0.0, 0.0), np.eye(3))
        spread = float(angles.get('arms_out', 0.0))
        placed, out = {}, []
        for name, parent, joints, offset, rest, _mesh in self.parts:
            if parent:
                above, at = placed[parent]
                spot = at + above @ np.asarray(offset, float)
                if spread and name.endswith('_upper_arm'):
                    rest = rest + math.copysign(spread, rest)
                here = above @ turn_about('z', rest) if rest else above
            else:
                spot, here = np.asarray(where, float), np.asarray(turn, float)
            for joint, axis, sign in joints:
                here = here @ turn_about(axis, sign * float(angles.get(joint, 0.0)))
            placed[name] = (here, spot)
            out.append(placed[name])
        return out

    def pivots(self, angles, root=None):
        """{joint: its pivot, world}; `root` as `pose` takes it."""
        np = _np()
        frames = self._frames(angles, root)
        floor = np.zeros(3)
        if root is None:
            positions, _normals = moved(self.corners, self.normals, self.spans, frames)
            floor[1] = min(positions[slice(*self.spans[i]), 1].min() for i in self.soles)
        return {joint: spot - floor for (_name, _parent, joints, *_rest), (_turn, spot)
                in zip(self.parts, frames) for joint, _axis, _sign in joints}

    def pose(self, angles, dense=False, root=None):
        """(positions, normals) in metres, the floor at y 0, for {joint: degrees}; `root` the
        pelvis's (place, turn), else the feet found and stood on the floor; `dense` the sampled
        points' instead of the corners'."""
        frames = self._frames(angles, root)
        positions, normals = moved(self.corners, self.normals, self.spans, frames)
        floor = 0.0 if root is not None else min(
            positions[slice(*self.spans[i]), 1].min() for i in self.soles)
        if dense:
            positions, normals = moved(self.dense[0], self.dense[1], self.dense_spans, frames)
        positions[:, 1] -= floor
        return positions, normals


def _band(depth, width):
    """(left, right) cells: where her drawing begins and ends across, a cell off her."""
    np = _np()
    across = np.nonzero((depth > 0.0).any(axis=0))[0]
    if not len(across):
        return None
    return max(0, int(across[0]) // DOTS_X - 1), min(width, int(across[-1]) // DOTS_X + 2)


#: The body, built on first use: 0.1 s of lofts.
_BODY = {}


def body(dressed=True):
    got = _BODY.get(dressed)
    if got is None:
        got = _BODY[dressed] = Body(dressed)
    return got


class Follow:

    """A camera's place over the floor, (x, z): on at her mean velocity, meaned over `seconds`,
    and toward her place four times slower - her surge shows, and a head carried evenly stands
    still. Tied to her pelvis, the pelvis stood still and an even head swung (2026-09-26); along
    the world's z alone, up from a fall facing aside she walked out past the lens (2026-09-30)."""

    def __init__(self, seconds=1.0):
        self.seconds, self.at = seconds, None

    def __call__(self, place, speed, t):
        """The camera's place for her at `place` going `speed`, (x, z) m and m/s, at `t`, s."""
        if self.at is None or math.dist(place, self.at[0]) > 1.0 or t <= self.at[2]:
            self.at = (tuple(place), tuple(speed), t)
            return self.at[0]
        at, mean, then = self.at
        on, pull = min(1.0, (t - then) / self.seconds), min(1.0, (t - then) / (4.0 * self.seconds))
        mean = tuple(m + (s - m) * on for m, s in zip(mean, speed))
        at = tuple(a + m * (t - then) + (p - a) * pull for a, m, p in zip(at, mean, place))
        self.at = (at, mean, t)
        return at


#: What she trips on, its ink by kind (`World.props`).
PROP_INK = {'hole': (255, 96, 128), 'sill': (255, 184, 80), 'slip': (96, 214, 255),
            'rug': (200, 160, 110), 'lace': (230, 90, 230), 'stairs': (220, 225, 235)}


def _props(props, m, cam, centre, travel):
    """[(dots, ink)]: each prop's edges - a box's twelve, a lace's line - in the fine camera's
    dots, the floor `travel` (x, z) m on."""
    np = _np()
    out = []
    for kind, *shape in props:
        if kind == 'lace':
            corners = np.asarray(shape, float)
            edges = ((0, 1),)
        else:
            (cx, cy, cz), half, turn = shape
            signs = np.array([(i, j, k) for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float)
            corners = (signs * np.asarray(half)) @ np.asarray(turn, float).T + (cx, cy, cz)
            edges = [(a, b) for a in range(8) for b in range(a + 1, 8)
                     if bin(a ^ b).count('1') == 1]
        corners[:, 0] -= travel[0]
        corners[:, 2] -= travel[1]
        sx, sy, w = project(corners, m, cam, centre)
        dots = np.zeros((cam['height'], cam['width']), bool)
        for a, b in edges:
            if w[a] > 0.0 and w[b] > 0.0:
                line(dots, (sx[a], sy[a]), (sx[b], sy[b]))
        out.append((dots, PROP_INK.get(kind, (200, 200, 200))))
    return out


def render(angles, width, height, yaw=30.0, pitch=8.0, zoom=1.0, colour=True, travel=(0.0, 0.0),
           lit=None, root=None, labels=None, heat=None, props=None, legend=None, dressed=True,
           around=False):
    """Her, posed at {joint: degrees}, the pelvis at `root` (place, turn) if given, `width` x
    `height` cells: lines. `lit` a `gpu.LitRaster`, or None to splat her dots here; `labels`
    {joint: [row, ..]} called out at the edges, a leader to each joint (`callouts`); `heat`
    {joint: C} each drive's drum painted its temperature's colour (`ansi.thermal_rgb`); `props`
    what she trips on, world (`World.props`), drawn as edges `travel` (x, z) m back; `legend` a row
    [(char, fg, bg)] on the last line, the callouts kept above it; `dressed` False her shell;
    `around` the callouts docked around her, not at the drawing's edges."""
    np = _np()
    who = body(dressed)
    m = view(yaw, pitch)
    fine = engine.fine(engine.camera(width, height, REACH, distance=DISTANCE, zoom=zoom))
    centre = np.asarray(CENTRE)
    materials, dense = who.materials, who.dense[3]
    if heat:
        materials, dense = materials.copy(), dense.copy()
        for joint, celsius in heat.items():
            if joint in who.drums:
                worn = paint(ansi.thermal_rgb(celsius))
                i = who.drums[joint]
                materials[slice(*who.spans[i])] = worn
                dense[slice(*who.dense_spans[i])] = worn
                for part, corners in who.patches.get(joint, ()):
                    materials[who.spans[part][0] + corners] = worn
                    lo, hi = who.dense_spans[part]
                    dense[lo:hi][who.dense_near[joint][part]] = worn
    if lit is not None:
        positions, normals = who.pose(angles, root=root)
        depth, rgb = lit.raster(positions, normals, who.uv, materials, who.index, m, fine,
                                CENTRE, REACH * 1.4)
    else:
        positions, normals = who.pose(angles, dense=True, root=root)
        depth, rgb = splat((positions, normals, dense, who.dense[2]), m, fine, centre)
    overlay = leaders = None
    if labels:
        pivots, rest = who.pivots(angles, root=root), who.pivots({})
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
    return braille(depth, rgb, grid(m, fine, centre, travel), width, height, colour, overlay,
                   leaders, _props(props or (), m, fine, centre, travel))
