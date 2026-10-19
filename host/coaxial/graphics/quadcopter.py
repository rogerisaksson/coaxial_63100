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

#: A rotor's motor (42 x 27 mm: 39 W a hover, aircraft.md), the Coaxial 63100 board behind its
#: stator (68 mm) and the finned heatsink under it, a shaft, a leg, a hub, m.
MOTOR_R, MOTOR_L, BOARD_R, BOARD_H = 0.021, 0.027, 0.034, 0.005
SINK_R, SINK_H, SHAFT_R = 0.032, 0.006, 0.004
LEG_R, HUB_R = 0.006, 0.014

#: A Hunter long-range quad, measured off a model, cm: its motors' reach from the middle; its
#: plates (nose, tail, half-width, half-width and half-length where the arms join, the arms'
#: plate's and the top's thickness, the top's height over the arms'); an arm's width; the pack
#: under the plates (nose, tail, half-width, depth, gap); the camera between them ahead (length,
#: half-width, bottom, top over the arms'); the GPS on its mast (back, height, radius, depth);
#: the aerial aft (its foot's back and height, its tip's); a blade's chord along its radius,
#: both over the radius.
REACH_CM = 19.63
PLATES = (7.2, -6.5, 1.75, 2.5, 3.5, 0.5, 0.2, 3.55)
ARM_W, PACK, CAMERA = 1.6, (4.5, -4.0, 2.1, 3.7, 0.45), (2.8, 1.5, 0.55, 3.05)
GPS, AERIAL = (4.0, 7.4, 1.7, 1.2), ((6.8, 2.55), (10.5, 6.35))
CHORD = ((0.06, 0.10), (0.25, 0.11), (0.4, 0.15), (0.65, 0.15), (0.8, 0.12), (0.92, 0.08),
         (0.98, 0.045), (1.0, 0.02))
#: The form's unit at the rotors' reach, m a cm; the arms' plate's middle, m up.
K, ARM_Y = math.hypot(*quad.ROTOR_AT[0]) / REACH_CM, 0.011

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


def _arm(root, tip, material):
    """A flat arm from `root` (x, z) on the plates out to its pad at `tip`."""
    (x0, z0), (x1, z1) = root, tip
    length = math.hypot(x1 - x0, z1 - z0)
    turn = shapes.turn_about('y', -math.degrees(math.atan2(z1 - z0, x1 - x0)))
    return _at(shapes.box((0.0, 0.0, 0.0), (0.5 * length, 0.5 * K * PLATES[5], 0.5 * K * ARM_W), material),
               turn, (0.5 * (x0 + x1), ARM_Y, 0.5 * (z0 + z1)))


def _box(name, nose, tail, half, bottom, top, material):
    """(name, a box from `tail` to `nose`, cm ahead, `half` cm across, `bottom` to `top` m up)."""
    return (name, shapes.box((0.0, 0.5 * (bottom + top), 0.5 * K * (nose + tail)),
                             (K * half, 0.5 * (top - bottom), 0.5 * K * (nose - tail)), material))


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
    """[(name, mesh)]: the frame in its own axes, x right, y up, z ahead - a long-range quad's: two
    plates on four flat arms, the camera between them ahead, the pack under them, the GPS on its
    mast, the aerial aft, a leg under each arm to its skid's foot (`quad.SKID_M`); the rotors at
    `quad.ROTOR_AT`; `see` a layer of LAYERS: the airframe its plates, arms and legs, the pack
    under them, bare the shafts on their arms."""
    shell, frame, top = see == 'shell', see != 'bare', quad.DISC_M
    nose, tail, half, joint_w, joint_l, arm_t, top_t, top_h = PLATES
    plate, carbon = paint(SLATE if shell else CARBON), paint(CARBON)
    a0, a1, deck = ARM_Y - 0.5 * K * arm_t, ARM_Y + 0.5 * K * arm_t, ARM_Y + K * top_h
    out = []
    if frame:
        out += [_box('plate', nose, tail, half, a0, a1, plate),
                _box('joint', joint_l, -joint_l, joint_w, a0, a1, plate),
                _box('deck', nose, tail, half, deck - 0.5 * K * top_t, deck + 0.5 * K * top_t, plate)]
        out += [('post%d' % k, _at(shapes.drum(0.004, deck - a1, 'y', PLATE), None,
                                   (sx * K * (half - 0.3), 0.5 * (a1 + deck), K * z)))
                for k, (sx, z) in enumerate((sx, z) for sx in (-1.0, 1.0) for z in (nose - 0.6, tail + 0.6))]
    if see in ('shell', 'pack'):
        p_nose, p_tail, p_half, depth, gap = PACK
        out.append(_box('pack', p_nose, p_tail, p_half, a0 - K * (gap + depth), a0 - K * gap, paint(CELLS)))
    if shell:
        length, c_half, c_bottom, c_top = CAMERA
        back, high, radius, thick = GPS
        (foot_b, foot_h), (tip_b, tip_h) = AERIAL
        gps, tip = (0.0, ARM_Y + K * high, -K * back), (0.0, ARM_Y + K * tip_h, -K * tip_b)
        out += [_box('camera', nose, nose - length, c_half, ARM_Y + K * c_bottom, ARM_Y + K * c_top, paint(DARK)),
                ('lens', _at(shapes.drum(K, 0.01, 'z', CORE), None,
                             (0.0, ARM_Y + 0.5 * K * (c_bottom + c_top), K * nose + 0.005))),
                ('mast', shapes.strut((0.0, deck, 0.0), gps, 0.005, paint(ACCENT))),
                ('gps', _at(shapes.drum(K * radius, K * thick, 'y', paint(SKIN)), None, gps)),
                ('aerial', shapes.strut((0.0, ARM_Y + K * foot_h, -K * foot_b), tip, 0.003, carbon)),
                ('cap', shapes.ellipsoid(tip, (0.012, 0.012, 0.012), paint(ACCENT), rows=6))]
    for k, (x, z) in enumerate(quad.ROTOR_AT):
        if frame:
            foot = (0.7 * x, -quad.SKID_M, 0.7 * z)
            root = (math.copysign(K * joint_w, x), math.copysign(K * joint_l, z))
            out += [('arm%d' % k, _arm(root, (x, z), plate)),
                    ('pad%d' % k, _at(shapes.drum(BOARD_R + 0.003, K * arm_t, 'y', plate), None, (x, ARM_Y, z))),
                    ('leg%d' % k, shapes.strut((foot[0], a0, foot[2]), foot, LEG_R, carbon)),
                    ('foot%d' % k, shapes.ellipsoid(foot, (quad.FOOT_M,) * 3, carbon, rows=6))]
            out += [(name + str(k), mesh) for name, mesh in drive(x, a1, z)]
        else:
            out.append(('rod%d' % k, _at(shapes.drum(0.003, math.hypot(x, z), 'z', PLATE),
                                         shapes.turn_about('y', math.degrees(math.atan2(x, z))),
                                         (x / 2.0, ARM_Y, z / 2.0))))
        out += [('shaft%d' % k, _at(shapes.drum(SHAFT_R, top - a1, 'y', PLATE), None,
                                    (x, (top + a1) / 2.0, z))),
                ('hub%d' % k, shapes.ellipsoid((x, top + 0.004, z), (HUB_R, 0.01, HUB_R),
                                               PLATE, rows=6))]
    return out


def prop_blade(radius, s):
    """A propeller's blade from its hub out to `radius` along `s` x, CHORD's planform, a thin
    foil."""
    def edges(r):
        half = 0.5 * radius * shapes.over(CHORD, r / radius, 1)
        return half, -half
    widest = 0.5 * radius * max(c for _r, c in CHORD)
    stations = [HUB_R + (radius - HUB_R) * j / 13.0 for j in range(14)]
    rows = [widest - 2.0 * widest * j / 5.0 for j in range(6)]
    return shapes.surface(stations, rows, edges, shapes.foil(edges, 0.08),
                          lambda r, c, t: (s * r, t, s * c), paint(PROP))


def blades():
    """A propeller's two blades about its hub, y its axis."""
    return [('blade', prop_blade(quad.DISC_R, s)) for s in (-1.0, 1.0)]


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
           see='shell', lens=1.0, field=None):
    """The quad at `pose` on its `rotors`, `width` x `height` cells, `reach` m framed about
    `centre` (the quad's middle) from SIGHT reaches off through a `lens` that much nearer:
    lines. `lit` a `gpu.LitRaster`, or None to splat its dots here; `world` False the quad
    alone, nothing under it or about it; `field` what stands about it (`grounds`, or
    `airfield`); `gate` the course's gates stood too, the one of that number lit, and `line_`
    the line it asks through them; `thrusts` each rotor's, N, its arrow; `inset` cells {(row,
    col): (codepoint, key)} laid over all; `see` a layer of LAYERS."""
    return render_craft(pose, rotors, width, height, yaw, pitch, reach, SIGHT, REACH, centre,
                        colour, lit, world, gate, thrusts, line_, inset, see, lens,
                        _posed, _discs, _arrows, field)

