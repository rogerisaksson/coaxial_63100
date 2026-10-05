"""The quad drawn: its frame, the 63100 cans on their coaxial boards, the propellers, in braille.

    lines = render(pose, rotors, 96, 40, lit=gpu.LitRaster())   # the GPU's lit raster
    lines = render(pose, rotors, 96, 40)                        # its dots splatted here

`pose` is `machine.quad.Sky.state()`'s, its 'turn' and 'at'; `rotors` each rotor's (angle rad,
speed rad/s, can C, board C), the can and its board painted their heat (`ansi.thermal_rgb`). A
propeller slower than BLADES_RAD_S shows its blades; faster, the disc they sweep, its rim dotted
where the frame does not cover it. The ground's grid to the horizon's edge, a pole marked each
metre up and what stands about the course are its world (`coaxial.graphics.scenery`); the camera
frames `reach` m about `centre`, SIGHT reaches off.
"""
import math

from coaxial.graphics import engine, scenery, shapes
from coaxial.graphics.callouts import line
from coaxial.graphics.lit import CORE, PLATE, braille, paint, project, splat
from machine import ansi, quad

#: The camera: the eye's distance in reaches framed, and the reach the quad fills, m - the
#: propellers' tips 0.68 m out.
SIGHT, REACH = 3.75, 0.8

#: The frame's parts, m: the centre plate and the dome on it, an arm's tube, a can (the 63100,
#: 63 mm by 100 mm) and its board under it, a skid's leg, a propeller's blade, a hub.
PLATE_R, PLATE_H = 0.1, 0.03
ARM_R, CAN_R, CAN_L, BOARD_R, BOARD_H = 0.012, 0.0315, 0.1, 0.034, 0.005
LEG_R, FOOT_R, BLADE_R, BLADE_W, HUB_R = 0.008, 0.015, 0.254, 0.022, 0.014

#: Their colours: carbon, the boards' solder mask, the propellers.
CARBON, MASK, PROP = (70, 74, 84), (30, 110, 64), (205, 205, 200)

#: The speed below which a propeller shows its blades, rad/s; the swept disc's rim, a dot
#: every RIM_EVERY of its line, and the inks.
BLADES_RAD_S, RIM_EVERY = 20.0, 2
RIM_INK = (120, 132, 150)


def _at(mesh, turn=None, spot=(0.0, 0.0, 0.0)):
    """`mesh` turned by the 3x3 `turn` and moved to `spot`, its frame's."""
    from coaxial.model.blocks import numpy as np
    corners, triangles, uv, materials = mesh
    if turn is not None:
        corners = corners @ np.asarray(turn).T
    return corners + np.asarray(spot, float), triangles, uv, materials


def _arm(x, z, material):
    """An arm's tube from the plate's rim out to the can at (x, z)."""
    out = math.hypot(x, z)
    length = out - PLATE_R * 0.8 - CAN_R
    mid = PLATE_R * 0.8 + length / 2.0
    turn = shapes.turn_about('y', -math.degrees(math.atan2(z, x)))
    return _at(shapes.drum(ARM_R, length, 'x', material), turn,
               (mid * x / out, 0.0, mid * z / out))


def parts():
    """[(name, mesh)]: the frame in its own axes, x right, y up, z ahead - the rotors at
    `quad.ROTOR_AT`, the skids' feet where `quad.mjcf` stands them."""
    carbon = paint(CARBON)
    out = [('plate', shapes.drum(PLATE_R, PLATE_H, 'y', PLATE)),
           ('dome', shapes.ellipsoid((0.0, PLATE_H / 2.0, 0.0), (0.055, 0.03, 0.075), CORE))]
    for k, (x, z) in enumerate(quad.ROTOR_AT):
        top = quad.DISC_M
        out += [('arm%d' % k, _arm(x, z, carbon)),
                ('can%d' % k, _at(shapes.drum(CAN_R, CAN_L, 'y', PLATE), None,
                                  (x, top - CAN_L / 2.0, z))),
                ('board%d' % k, _at(shapes.drum(BOARD_R, BOARD_H, 'y', paint(MASK)), None,
                                    (x, top - CAN_L - BOARD_H / 2.0, z))),
                ('hub%d' % k, shapes.ellipsoid((x, top + 0.004, z), (HUB_R, 0.01, HUB_R),
                                               PLATE, rows=6)),
                ('leg%d' % k, _at(shapes.drum(LEG_R, quad.SKID_M, 'y', carbon), None,
                                  (0.7 * x, -quad.SKID_M / 2.0, 0.7 * z))),
                ('foot%d' % k, shapes.ellipsoid((0.7 * x, -quad.SKID_M, 0.7 * z),
                                                (FOOT_R, FOOT_R, FOOT_R), PLATE, rows=6))]
    return out


def blades():
    """A propeller's two blades about its hub, y its axis."""
    prop = paint(PROP)
    return [('blade', shapes.ellipsoid((s * BLADE_R / 2.0, 0.0, 0.0),
                                       (BLADE_R / 2.0, 0.004, BLADE_W), prop, rows=8))
            for s in (-1.0, 1.0)]


class Mesh:

    """Parts laid end to end once: corners, triangles, uv, materials and smooth normals, each
    part's span, and the dots the splat draws without a card."""

    def __init__(self, named):
        from coaxial.model.blocks import numpy as np
        self.parts = [name for name, _mesh in named]
        corners, triangles, uv, materials, faces = [], [], [], [], []
        self.spans, base = [], 0
        for i, (_name, (c, t, u, m)) in enumerate(named):
            corners.append(c)
            triangles.append(t + base)
            uv.append(u)
            materials.append(m)
            faces.append(np.full(len(t), i))
            self.spans.append((base, base + len(c)))
            base += len(c)
        self.corners = np.vstack(corners)
        self.index = np.vstack(triangles).astype(np.uint32)
        self.uv = np.vstack(uv)
        self.materials = np.concatenate(materials).astype(np.uint32)
        self.normals = shapes.smooth(self.corners, self.index)
        self.dense, self.dense_spans = shapes.sampled(self, np.concatenate(faces))

    def span(self, name, dense=False):
        return (self.dense_spans if dense else self.spans)[self.parts.index(name)]


#: The frame and a propeller, built on first use.
_MESH = {}


def mesh(which):
    got = _MESH.get(which)
    if got is None:
        got = _MESH[which] = Mesh(parts() if which == 'frame' else blades())
    return got


def _painted(frame, rotors, dense):
    """The frame's materials, or its dots', each can and board its heat's colour."""
    materials = (frame.dense[3] if dense else frame.materials).copy()
    for k, (_angle, _w, can_c, board_c) in enumerate(rotors):
        for name, celsius in (('can%d' % k, can_c), ('board%d' % k, board_c)):
            if celsius is not None:
                materials[slice(*frame.span(name, dense))] = paint(ansi.thermal_rgb(celsius))
    return materials


def _posed(pose, rotors, dense):
    """(positions, normals, uv, materials, index) in the world: the frame at its pose, each
    slow propeller's blades at its angle over its hub."""
    from coaxial.model.blocks import numpy as np
    turn, at = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    frame, prop = mesh('frame'), mesh('prop')
    pick = (lambda m: (m.dense[0], m.dense[1], m.dense[2], m.dense[3])) if dense else \
        (lambda m: (m.corners, m.normals, m.uv, m.materials))
    points, normals, uv, _materials = pick(frame)
    out = [[points @ turn.T + at], [normals @ turn.T], [uv], [_painted(frame, rotors, dense)]]
    index, base = [frame.index], len(points)
    for (x, z), spin, (angle, w, _can, _board) in zip(quad.ROTOR_AT, quad.SPIN, rotors):
        if abs(w) >= BLADES_RAD_S:
            continue
        here = turn @ shapes.turn_about('y', math.degrees(spin * angle))
        p, n, u, m = pick(prop)
        out[0].append(p @ here.T + at + turn @ np.array([x, quad.DISC_M + 0.006, z]))
        out[1].append(n @ here.T)
        out[2].append(u)
        out[3].append(m)
        index.append(prop.index + base)
        base += len(p)
    return tuple(np.concatenate(o) for o in out) + (np.vstack(index),)


def _discs(pose, rotors, m, cam, centre):
    """[(dots, ink)]: each fast propeller's swept disc, its rim dotted."""
    from coaxial.model.blocks import numpy as np
    turn, at = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    rim = np.zeros((cam['height'], cam['width']), bool)
    k = np.arange(96) * (2.0 * math.pi / 96)
    ring = np.stack([np.cos(k), np.zeros_like(k), np.sin(k)], 1) * BLADE_R
    for (x, z), (_angle, w, _can, _board) in zip(quad.ROTOR_AT, rotors):
        if abs(w) < BLADES_RAD_S:
            continue
        hub = at + turn @ np.array([x, quad.DISC_M + 0.006, z])
        sx, sy, _w = project(ring @ turn.T + hub, m, cam, centre)
        drawn = np.zeros_like(rim)
        for a in range(len(k)):
            line(drawn, (sx[a], sy[a]), (sx[a - 1], sy[a - 1]))
        ys, xs = np.nonzero(drawn)
        keep = ((xs + ys) % RIM_EVERY) == 0
        rim[ys[keep], xs[keep]] = True
    return [(rim, RIM_INK)]


def render(pose, rotors, width, height, yaw=30.0, pitch=18.0, reach=REACH, centre=None,
           colour=True, lit=None, world=True, gate=None):
    """The quad at `pose` on its `rotors`, `width` x `height` cells, `reach` m framed about
    `centre` (the quad's middle) from SIGHT reaches off: lines. `lit` a `gpu.LitRaster`, or
    None to splat its dots here; `world` False the quad alone, nothing under it or about it;
    `gate` the course's gates stood too, the one of that number lit."""
    from coaxial.model.blocks import numpy as np
    m = shapes.view(yaw, pitch)
    fine = engine.fine(engine.camera(width, height, reach, distance=SIGHT * reach))
    at = np.asarray(pose['at'], float)
    centre = at if centre is None else np.asarray(centre, float)
    if lit is not None:
        positions, normals, uv, materials, index = _posed(pose, rotors, False)
        depth, rgb = lit.raster(positions, normals, uv, materials, index, m, fine,
                                tuple(centre), float(np.linalg.norm(at - centre)) + REACH)
    else:
        positions, normals, uv, materials, _index = _posed(pose, rotors, True)
        depth, rgb = splat((positions, normals, materials, uv), m, fine, centre)
    if not world:
        return braille(depth, rgb, np.zeros(depth.shape), width, height, colour,
                       props=_discs(pose, rotors, m, fine, centre))
    return braille(depth, rgb, scenery.ground(m, fine, centre), width, height, colour,
                   props=_discs(pose, rotors, m, fine, centre)
                   + scenery.props(m, fine, centre, gate))
