"""The quad drawn: its frame, the 63100 cans on their coaxial boards, the propellers, in braille.

    lines = render(pose, rotors, 96, 40, lit=gpu.LitRaster())   # the GPU's lit raster
    lines = render(pose, rotors, 96, 40)                        # its dots splatted here

`pose` is `machine.quad.Sky.state()`'s, its 'turn' and 'at'; `rotors` each rotor's (angle rad,
speed rad/s, can C, board C[, sink C]), the can, its board and its heatsink painted their heat
(`ansi.thermal_rgb`). A propeller slower than BLADES_RAD_S shows its blades; faster, the disc they
sweep, its rim dotted where the frame does not cover it. The ground's grid to the horizon's edge,
a pole marked each metre up and what stands about the course are its world
(`coaxial.graphics.scenery`); the camera frames `reach` m about `centre`, SIGHT reaches off.
"""
import math

from coaxial.graphics import engine, scenery, shapes
from coaxial.graphics.callouts import line
from coaxial.graphics.lit import CORE, PLATE, braille, paint, project, splat
from machine import ansi, quad

#: The camera: the eye's distance in reaches framed, and the reach the quad fills, m - the
#: propellers' tips 0.68 m out.
SIGHT, REACH = 3.75, 0.8

#: The frame's parts, m: the centre plate, an arm's tube, a rotor's motor (42 x 27 mm: 39 W a
#: hover, aircraft.md), the Coaxial 63100 board behind its stator (68 mm) and the finned
#: heatsink under it, a shaft, a skid's leg, a propeller's blade, a hub.
PLATE_R, PLATE_H = 0.1, 0.03
ARM_R, MOTOR_R, MOTOR_L, BOARD_R, BOARD_H = 0.012, 0.021, 0.027, 0.034, 0.005
SINK_R, SINK_H, SHAFT_R = 0.032, 0.006, 0.004
LEG_R, BLADE_W, HUB_R = 0.008, 0.022, 0.014

#: What a page draws of a craft, S stepping on: the craft whole, its airframe, its pack in it,
#: and bare - the shafts, blades, surfaces and cranks that move.
LAYERS = ('shell', 'frame', 'pack', 'bare')

#: Their colours: carbon, the boards' solder mask, the propellers, a painted skin - carbon's
#: dark halftone left a body a cloud of dots.
CARBON, MASK, PROP, SKIN = (70, 74, 84), (30, 110, 64), (205, 205, 200), (188, 194, 204)
#: A tactical drone's slate and its black fittings; a racer's red; a pack's cells.
SLATE, DARK, ACCENT, CELLS = (86, 92, 100), (34, 36, 40), (214, 62, 50), (226, 196, 64)

#: The speed below which a propeller shows its blades, rad/s; the swept disc's rim, a dot
#: every RIM_EVERY of its line, and the inks.
BLADES_RAD_S, RIM_EVERY = 20.0, 2
RIM_INK = (120, 132, 150)

#: A thrust's arrow up its rotor's axis from the hub, m a newton - a rotor's top, 54 N, 1.35 m,
#: a hover's 4.9 N 0.12 m -, its head a fifth of it, and its ink.
ARROW_M_PER_N, ARROW_HEAD, ARROW_INK = 0.025, 0.2, (205, 110, 255)


def _at(mesh, turn=None, spot=(0.0, 0.0, 0.0)):
    """`mesh` turned by the 3x3 `turn` and moved to `spot`, its frame's."""
    from coaxial.model.blocks import numpy as np
    corners, triangles, uv, materials = mesh
    if turn is not None:
        corners = corners @ np.asarray(turn).T
    return corners + np.asarray(spot, float), triangles, uv, materials


def _arm(x, z, material):
    """An arm's round boom from the body's side out to the can at (x, z)."""
    out = math.hypot(x, z)
    length = out - PLATE_R * 0.7 - MOTOR_R
    mid = PLATE_R * 0.7 + length / 2.0
    turn = shapes.turn_about('y', -math.degrees(math.atan2(z, x)))
    return _at(shapes.drum(ARM_R, length, 'x', material), turn, (mid * x / out, 0.0, mid * z / out))


def _hump():
    """The body: a hump along z, flat under, its back rising to the middle."""
    rings = [(-0.20, 0.05, 0.014, 0.0), (-0.12, 0.08, 0.030, 0.012), (-0.04, 0.095, 0.046, 0.022),
             (0.04, 0.095, 0.046, 0.022), (0.12, 0.085, 0.036, 0.014), (0.20, 0.06, 0.022, 0.004),
             (0.25, 0.03, 0.012, -0.004)]
    return shapes.loft(rings, paint(SLATE), poles=(-0.21, 0.26), along='z')


def drive(x, y, z, length=MOTOR_L, sink=None):
    """A drive's stack up y from `y` at (x, z): the heatsink, the 63100 board, the motor
    `length` long, as [(name, mesh)] to suffix; `sink` its material."""
    stack = ((SINK_H, lambda: shapes.drum(SINK_R, SINK_H, 'y', sink or paint(DARK)), 'sink'),
             (BOARD_H, lambda: shapes.drum(BOARD_R, BOARD_H, 'y', paint(MASK)), 'board'),
             (length, lambda: shapes.drum(MOTOR_R, length, 'y', PLATE), 'can'))
    out, top = [], y
    for h, mesh, name in stack:
        out.append((name, _at(mesh(), None, (x, top + h / 2.0, z))))
        top += h
    return out[::-1]


def parts(see='shell'):
    """[(name, mesh)]: the frame in its own axes, x right, y up, z ahead - a tactical quad's:
    a slate hump on four round arms, a gimbal ball under its nose, an aerial aft, two skid
    frames; the rotors at `quad.ROTOR_AT`, the skids' rails at `quad.SKID_M`; `see` a layer of
    LAYERS: the airframe two carbon plates, the pack between them, bare the shafts on their arms."""
    slate, dark, shell = paint(SLATE), paint(DARK), see == 'shell'
    frame, top, arm = see != 'bare', quad.DISC_M, 0.011
    if shell:
        out = [('hump', _hump()),
               ('ball', shapes.ellipsoid((0.0, -0.045, 0.14), (0.028, 0.028, 0.028), dark, rows=7)),
               ('lens', shapes.ellipsoid((0.0, -0.045, 0.168), (0.011, 0.011, 0.005), CORE, rows=5)),
               ('aerial', _at(shapes.drum(0.004, 0.11, 'y', dark), None, (0.05, 0.09, -0.12)))]
    elif frame:
        out = [('plate', shapes.box((0.0, -0.02, 0.0), (0.075, 0.003, 0.14), paint(CARBON))),
               ('deck', shapes.box((0.0, 0.028, 0.0), (0.075, 0.003, 0.14), paint(CARBON)))]
        if see == 'pack':
            out.append(('pack', shapes.box((0.0, 0.004, -0.01), (0.0225, 0.02, 0.07), paint(CELLS))))
    else:
        out = []
    for s in (-1.0, 1.0) if frame else ():
        out.append(('rail%d' % (s > 0), _at(shapes.drum(0.006, 0.36, 'z', dark), None,
                                             (s * 0.1, -quad.SKID_M, 0.0))))
        out += [('leg%d_%d' % (s > 0, k), _at(shapes.drum(LEG_R, quad.SKID_M, 'y', dark), None,
                                               (s * 0.1, -quad.SKID_M / 2.0, z)))
                for k, z in enumerate((-0.14, 0.14))]
    for k, (x, z) in enumerate(quad.ROTOR_AT):
        if frame:
            out += [('arm%d' % k, _arm(x, z, slate if shell else paint(CARBON)))]
            out += [(name + str(k), mesh) for name, mesh in drive(x, arm, z)]
        else:
            out.append(('rod%d' % k, _at(shapes.drum(0.003, math.hypot(x, z), 'z', PLATE),
                                         shapes.turn_about('y', math.degrees(math.atan2(x, z))),
                                         (x / 2.0, arm, z / 2.0))))
        out += [('shaft%d' % k, _at(shapes.drum(SHAFT_R, top - arm, 'y', PLATE), None,
                                    (x, (top + arm) / 2.0, z))),
                ('hub%d' % k, shapes.ellipsoid((x, top + 0.004, z), (HUB_R, 0.01, HUB_R),
                                               PLATE, rows=6))]
    return out


def blades():
    """A propeller's two blades about its hub, y its axis."""
    prop = paint(PROP)
    return [('blade', shapes.ellipsoid((s * quad.DISC_R / 2.0, 0.0, 0.0),
                                       (quad.DISC_R / 2.0, 0.004, BLADE_W), prop, rows=8))
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


def mesh(which, see='shell'):
    got = _MESH.get((which, see))
    if got is None:
        got = _MESH[(which, see)] = Mesh(parts(see) if which == 'frame' else blades())
    return got


def _painted(frame, rotors, dense):
    """The frame's materials, or its dots', each can, board and heatsink its heat's colour."""
    materials = (frame.dense[3] if dense else frame.materials).copy()
    for k, rotor in enumerate(rotors):
        can_c, board_c = rotor[2], rotor[3]
        sink_c = rotor[4] if len(rotor) > 4 else None
        for name, celsius in (('can%d' % k, can_c), ('board%d' % k, board_c),
                              ('sink%d' % k, sink_c)):
            if celsius is not None and name in frame.parts:
                materials[slice(*frame.span(name, dense))] = paint(ansi.thermal_rgb(celsius))
    return materials


def pose_meshes(pose, rotors, dense, frame, moving):
    """(positions, normals, uv, materials, index) in the world: `frame` at `pose` and each
    `(part_mesh, rot_local, spot)` in `moving` turned and placed in the frame's axes."""
    from coaxial.model.blocks import numpy as np
    turn, at = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    pick = (lambda m: (m.dense[0], m.dense[1], m.dense[2], m.dense[3])) if dense else \
        (lambda m: (m.corners, m.normals, m.uv, m.materials))
    points, normals, uv, _materials = pick(frame)
    out = [[points @ turn.T + at], [normals @ turn.T], [uv], [_painted(frame, rotors, dense)]]
    index, base = [frame.index], len(points)
    for part, rot_local, spot in moving:
        here = turn @ rot_local
        p, n, u, m = pick(part)
        out[0].append(p @ here.T + at + turn @ np.asarray(spot, float))
        out[1].append(n @ here.T)
        out[2].append(u)
        out[3].append(m)
        index.append(part.index + base)
        base += len(p)
    return tuple(np.concatenate(o) for o in out) + (np.vstack(index),)


def _posed(pose, rotors, dense, see='shell'):
    """(positions, normals, uv, materials, index) in the world: the frame at its pose, each
    slow propeller's blades at its angle over its hub - every propeller's, bare."""
    frame, prop = mesh('frame', see), mesh('prop')
    moving = [(prop, shapes.turn_about('y', math.degrees(spin * angle)), (x, quad.DISC_M + 0.006, z))
              for (x, z), spin, (angle, w, *_heat) in zip(quad.ROTOR_AT, quad.SPIN, rotors)
              if abs(w) < BLADES_RAD_S or see == 'bare']
    return pose_meshes(pose, rotors, dense, frame, moving)


def swept_rings(pose, rings_and_rotors, m, cam, centre):
    """[(dots, ink)]: each fast rotor's swept ring `(ring, spot)` where `abs(w) >= BLADES_RAD_S`,
    its rim dotted."""
    from coaxial.model.blocks import numpy as np
    turn, at = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    rim = np.zeros((cam['height'], cam['width']), bool)
    for (ring, spot), (_angle, w, *_heat) in rings_and_rotors:
        if abs(w) < BLADES_RAD_S:
            continue
        hub = at + turn @ np.asarray(spot, float)
        sx, sy, _w = project(ring @ turn.T + hub, m, cam, centre)
        drawn = np.zeros_like(rim)
        for a in range(len(ring)):
            line(drawn, (sx[a], sy[a]), (sx[a - 1], sy[a - 1]))
        ys, xs = np.nonzero(drawn)
        keep = ((xs + ys) % RIM_EVERY) == 0
        rim[ys[keep], xs[keep]] = True
    return [(rim, RIM_INK)]


def _discs(pose, rotors, m, cam, centre):
    """[(dots, ink)]: each fast propeller's swept disc, its rim dotted."""
    from coaxial.model.blocks import numpy as np
    k = np.arange(96) * (2.0 * math.pi / 96)
    ring = np.stack([np.cos(k), np.zeros_like(k), np.sin(k)], 1) * quad.DISC_R
    specs = [((ring, (x, quad.DISC_M + 0.006, z)), r) for (x, z), r in zip(quad.ROTOR_AT, rotors)]
    return swept_rings(pose, specs, m, cam, centre)


def _arrows(pose, thrusts, m, cam, centre):
    """[(dots, ink, over)]: each rotor's thrust, `thrusts` N, an arrow up its axis from its
    hub, over whatever covers it."""
    from coaxial.model.blocks import numpy as np
    turn, at = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    up, side = turn[:, 1], turn[:, 0]
    drawn = np.zeros((cam['height'], cam['width']), bool)
    for (x, z), thrust in zip(quad.ROTOR_AT, thrusts):
        length = ARROW_M_PER_N * max(0.0, thrust)
        if length <= 0.0:
            continue
        hub = at + turn @ np.array([x, quad.DISC_M, z])
        tip = hub + length * up
        back = tip - ARROW_HEAD * length * up
        points = np.array([hub, tip, back + 0.5 * ARROW_HEAD * length * side,
                           back - 0.5 * ARROW_HEAD * length * side])
        sx, sy, _w = project(points, m, cam, centre)
        for a, b in ((0, 1), (1, 2), (1, 3)):
            line(drawn, (sx[a], sy[a]), (sx[b], sy[b]))
    return [(drawn, ARROW_INK, True)]


def render_craft(pose, rotors, width, height, yaw, pitch, reach, sight, base_reach, centre,
                 colour, lit, world, gate, thrusts, line_, inset, see, lens,
                 posed_fn, discs_fn, arrows_fn, course_mod=None):
    """Shared 3D braille craft renderer across quad, helicopter, ornithopter and aerobat."""
    from coaxial.model.blocks import numpy as np
    m = shapes.view(yaw, pitch)
    fine = engine.fine(engine.camera(width, height, reach, distance=sight * reach, zoom=lens))
    at = np.asarray(pose['at'], float)
    centre = at if centre is None else np.asarray(centre, float)
    if lit is not None:
        positions, normals, uv, materials, index = posed_fn(pose, rotors, False, see)
        depth, rgb = lit.raster(positions, normals, uv, materials, index, m, fine,
                                tuple(centre), float(np.linalg.norm(at - centre)) + base_reach)
    else:
        positions, normals, uv, materials, _index = posed_fn(pose, rotors, True, see)
        depth, rgb = splat((positions, normals, materials, uv), m, fine, centre)
    own = ([] if see == 'bare' else discs_fn(pose, rotors, m, fine, centre)) + (
        arrows_fn(pose, thrusts, m, fine, centre) if thrusts is not None else [])
    if not world:
        return braille(depth, rgb, np.zeros(depth.shape), width, height, colour, inset,
                       props=own)
    scenery_props = (scenery.props(m, fine, centre, gate, line_) if course_mod is None
                     else scenery.props(m, fine, centre, gate, line_, course_mod))
    return braille(depth, rgb, scenery.ground(m, fine, centre), width, height, colour, inset,
                   props=own + scenery_props)


def render(pose, rotors, width, height, yaw=30.0, pitch=18.0, reach=REACH, centre=None,
           colour=True, lit=None, world=True, gate=None, thrusts=None, line_=False, inset=None,
           see='shell', lens=1.0):
    """The quad at `pose` on its `rotors`, `width` x `height` cells, `reach` m framed about
    `centre` (the quad's middle) from SIGHT reaches off through a `lens` that much nearer: lines. `lit` a `gpu.LitRaster`, or
    None to splat its dots here; `world` False the quad alone, nothing under it or about it;
    `gate` the course's gates stood too, the one of that number lit, and `line_` the line it
    asks through them; `thrusts` each rotor's, N, its arrow; `inset` cells {(row, col):
    (codepoint, key)} laid over all; `see` a layer of LAYERS."""
    return render_craft(pose, rotors, width, height, yaw, pitch, reach, SIGHT, REACH, centre,
                        colour, lit, world, gate, thrusts, line_, inset, see, lens,
                        _posed, _discs, _arrows)

