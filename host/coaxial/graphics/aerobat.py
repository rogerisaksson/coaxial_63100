"""The 3D RC aerobatic monoplane drawn: fuselage, wings, 3D surfaces, 63100 drives and props.

    lines = render(pose, rotors, 96, 40, lit=gpu.LitRaster())   # the GPU's lit raster
    lines = render(pose, rotors, 96, 40)                        # its dots splatted here

`pose` is `machine.aerobat.Sky.state()`'s, its 'turn', 'at' and 'controls'; `rotors` the front
and rear coaxial 63100 tractor drives' (angle rad, speed rad/s, can C, board C[, sink C]), each
can, board and heatsink painted their heat (`ansi.thermal_rgb`). A propeller slower than
`BLADES_RAD_S` shows its blades; faster, the swept prop disc ring, while the oversized 3D
ailerons, elevator, and rudder deflect live with `controls`.
"""
import math

from coaxial.graphics import engine, scenery, shapes
from coaxial.graphics.callouts import line
from coaxial.graphics.helicopter import force_arrows
from coaxial.graphics.lit import CORE, PLATE, braille, paint, project, splat
from coaxial.graphics.quadcopter import (BLADE_W, BLADES_RAD_S, BOARD_H, BOARD_R, CARBON, CELLS,
                                         MASK, MOTOR_R, PROP, RIM_EVERY, RIM_INK, SHAFT_R,
                                         SINK_H, SINK_R, Mesh, _at, _painted)
from machine import aerobat as aero_phys, airfield

#: The camera: the eye's distance in reaches framed, and the reach the monoplane fills, m -
#: the wing tips 0.65 m out, the spinner at +0.57 m, the tail post at -0.52.
SIGHT, REACH = 3.75, 0.75

#: The airframe's geometry in its body axes (x right wing, y nose and thrust, -z dorsal, +z
#: ventral), m: the coaxial tractor hubs at the nose, the wing's half span and its station, the
#: tail's station and the stabiliser's half span, the taildragger's wheels.
PROP_Y, VECT_Y, PROP_R = aero_phys.PROP_Z, aero_phys.PROP_Z - 0.045, aero_phys.PROP_R
HALF_SPAN, WING_Y, CHORD = aero_phys.HALF_M, 0.03, aero_phys.WING_CHORD
TAIL_Y, HSTAB_HALF = -aero_phys.TAIL_ARM, 0.5 * aero_phys.STAB_SPAN
SKID_M, FOOT_M, SKID_X, STRUT_Z = (
    aero_phys.SKID_M, aero_phys.FOOT_M, aero_phys.SKID_X, aero_phys.STRUT_Z)
GEAR_MAIN_Z, GEAR_TAIL_Z = aero_phys.GEAR_MAIN_Z, aero_phys.GEAR_TAIL_Z
#: The two tractor motors, 42 x 40 mm each (292 W at the governed top, aircraft.md), stacked
#: in the cowl from MOTOR_Y aft, each with its 63100 board and sink behind its stator.
MOTOR_L, MOTOR_Y = 0.040, 0.495

#: The gear an Extra 300's: each main leg a carbon bow out from the belly and down to its wheel
#: in a spat, the wheels WHEEL_R drawn on the ground where the frame's rest; one tail wheel on a
#: leaf under the tail post. Drawn as straight struts with a tail wheel each side, it read as
#: a quad's skids.
WHEEL_R, TAIL_WHEEL_R, BOW_N, BOW_R = 0.04, 0.018, 8, 0.006

_AERO_MESH = {}


def _bow(s, carbon, spat):
    """A main leg, `s` its side: its bow in BOW_N pieces, its wheel, and its spat in `spat` (None:
    none)."""
    y, x = STRUT_Z[1], s * SKID_X
    z = GEAR_MAIN_Z + FOOT_M - WHEEL_R
    a, b, c = (s * 0.02, 0.07), (s * 0.13, 0.12), (x, z)
    pts = [tuple((1 - t) ** 2 * p + 2 * (1 - t) * t * q + t * t * r for p, q, r in zip(a, b, c))
           for t in (j / BOW_N for j in range(BOW_N + 1))]
    out = []
    for j, ((x0, z0), (x1, z1)) in enumerate(zip(pts, pts[1:])):
        dx, dz = x1 - x0, z1 - z0
        out.append(('bow%d_%d' % (s > 0, j),
                    _at(shapes.drum(BOW_R, math.hypot(dx, dz), 'z', carbon),
                        shapes.turn_about('y', -math.degrees(math.atan2(dx, dz))),
                        (0.5 * (x0 + x1), y, 0.5 * (z0 + z1)))))
    out.append(('wheel%d' % (s > 0), shapes.ellipsoid((x, y, z), (0.35 * WHEEL_R, WHEEL_R, WHEEL_R),
                                                       PLATE, rows=6)))
    if spat is not None:
        out.append(('spat%d' % (s > 0), shapes.ellipsoid((x, y - 0.012, z - 0.006), (0.022, 0.078, 0.046),
                                                          spat, rows=7)))
    return out


def _tail_wheel(carbon):
    """The tail wheel on its leaf under the tail post."""
    z = GEAR_TAIL_Z + FOOT_M - TAIL_WHEEL_R
    return [('tail_leaf', _at(shapes.drum(BOW_R, z - 0.02, 'z', carbon), None, (0.0, STRUT_Z[0], 0.5 * (z + 0.02)))),
            ('tail_wheel', shapes.ellipsoid((0.0, STRUT_Z[0], z), (0.4 * TAIL_WHEEL_R, TAIL_WHEEL_R, TAIL_WHEEL_R),
                                            PLATE, rows=6))]


SURFACE = (92, 98, 112)
#: The terminal's palette on an Extra 300: its shell the quad's skin, its cowl, tips, spats and
#: rudder the pages' amber, its lettering black - black keeps no dots (`lit.braille`).
WHITE, RED, INK = (188, 194, 204), (255, 176, 0), (0, 0, 0)
#: Its wing ahead of the ailerons' hinge: tapered ROOT_C to TIP_C, the leading edge swept, the
#: tips rounded over their last TIP_S of the span, THICK of the chord through - square boxes
#: read as a trainer's. Its stab and fin tapered alike.
ROOT_C, TIP_C, TIP_S, THICK = 0.31, 0.18, 0.08, 0.05
#: "COAXIAL" and "63100" in square wide capitals like Eurostile Bold Extended, PX_W a column
#: along the span and PX_H a row along the chord, from LETTER_X out, their top LETTER_Y ahead of
#: the hinge, on top, read from above nose up.
PX_W, PX_H, LETTER_X, LETTER_Y = 0.0125, 0.0214, 0.11, 0.165
GLYPHS = {
    'C': ('.#####', '##....', '##....', '##....', '##....', '##....', '.#####'),
    'O': ('.####.', '##..##', '##..##', '##..##', '##..##', '##..##', '.####.'),
    'A': ('.####.', '##..##', '##..##', '######', '##..##', '##..##', '##..##'),
    'X': ('##..##', '##..##', '.####.', '..##..', '.####.', '##..##', '##..##'),
    'I': ('##', '##', '##', '##', '##', '##', '##'),
    'L': ('##....', '##....', '##....', '##....', '##....', '##....', '######'),
    '6': ('.#####', '##....', '##....', '#####.', '##..##', '##..##', '.####.'),
    '3': ('#####.', '....##', '....##', '.####.', '....##', '....##', '#####.'),
    '1': ('.##', '###', '.##', '.##', '.##', '.##', '.##'),
    '0': ('.####.', '##..##', '##..##', '##..##', '##..##', '##..##', '.####.'),
}


def _letters(word):
    """`word` in GLYPHS' rows, top first, a column apart."""
    return ['.'.join(GLYPHS[ch][r] for ch in word) for r in range(7)]


_WORDS = {'COAXIAL': _letters('COAXIAL'), '63100': _letters('63100')}
def _inked(x, y, side, hinge):
    """Whether the lettering inks the wing at `x`, `y` on `side` (-1 its top): "COAXIAL" out to
    in on the left, "63100" in to out on the right."""
    rows = int((hinge + LETTER_Y - y) // PX_H)
    text = _WORDS['COAXIAL' if x < 0.0 else '63100']
    n = len(text[0])
    col = int((abs(x) - LETTER_X) // PX_W)
    col = n - 1 - col if x < 0.0 else col
    return side < 0 and 0 <= rows < 7 and 0 <= col < n and text[rows][col] == '#'


def _lifting(half, root, tip, hinge, place, colour, ink=None, step=None):
    """A tapered, round-tipped lifting surface `half` each way of its root (`half` < 0: one way
    only), `root` to `tip` of chord ahead of `hinge` on its straight hinge line, as
    (corners, triangles, uv, materials): `place(s, y, t)` its point at `s` along the span, `y`
    along the chord, `t` through it; `ink(s, y, side)` a quad's lettering, `step` its columns."""
    from coaxial.model.blocks import numpy as np
    span = abs(half)
    flat = span * (1.0 - TIP_S)

    def chord(s):
        a = abs(s)
        c = root + (tip - root) * min(a, flat) / flat
        return c * math.sqrt(max(0.0, 1.0 - ((a - flat) / (span - flat)) ** 2)) if a > flat else c
    out = [0.0, LETTER_X] if step else [0.0]
    while step and out[-1] + step < flat:
        out.append(out[-1] + step)
    out += [flat * k / 6.0 for k in range(1, 7)] if not step else [flat]
    out += [flat + (span - flat) * k / 4.0 for k in range(1, 5)]
    stations = sorted(set(out)) if half < 0 else sorted(set([-x for x in out] + out))
    band = [hinge + LETTER_Y - k * PX_H for k in range(7, -1, -1)] if ink else []
    corners, mats = [], []
    for side in (-1, 1):
        for s0, s1 in zip(stations, stations[1:]):
            ys = [[hinge] + [min(b, hinge + chord(s)) for b in band] + [hinge + chord(s)] for s in (s0, s1)]
            for j in range(len(ys[0]) - 1):
                quad = ((s0, ys[0][j]), (s1, ys[1][j]), (s1, ys[1][j + 1]), (s0, ys[0][j + 1]))
                if side < 0:
                    quad = quad[::-1]
                for s, y in quad:
                    c = max(chord(s), 1e-6)
                    u = min(1.0, (y - hinge) / c)
                    corners.append(place(s, y, side * THICK * c * math.sqrt(max(0.0, math.sin(math.pi * (0.15 + 0.85 * u))))))
                ys_mid, s_mid = 0.25 * sum(y for _s, y in quad), 0.5 * (s0 + s1)
                mats += [ink[1] if ink and ink[0](s_mid, ys_mid, side) else colour] * 4
        if side < 0:
            continue
        for s0, s1 in zip(stations, stations[1:]):
            t0, t1 = (THICK * max(chord(s), 1e-6) * math.sqrt(math.sin(0.15 * math.pi)) for s in (s0, s1))
            corners += [place(s0, hinge, -t0), place(s1, hinge, -t1), place(s1, hinge, t1), place(s0, hinge, t0)]
            mats += [colour] * 4
    corners = np.array(corners, float)
    quads = np.arange(len(corners) // 4)[:, None, None] * 4 + np.array([[0, 1, 2], [0, 2, 3]])[None]
    return corners, quads.reshape(-1, 3), np.zeros((len(corners), 2)), np.array(mats)


def _fuselage():
    """The fuselage along +y from the tail post to the cowl: deep under the canopy, flat-sided
    aft, a round cowl ahead."""
    rings = [
        (-0.52, 0.010, 0.016), (-0.42, 0.020, 0.032), (-0.30, 0.032, 0.050),
        (-0.16, 0.046, 0.066), (-0.02, 0.056, 0.078), (0.12, 0.060, 0.080),
        (0.26, 0.058, 0.072), (0.38, 0.052, 0.058), (0.47, 0.044, 0.046),
        (0.53, 0.034, 0.034),
    ]
    from coaxial.model.blocks import numpy as np
    white, red = paint(WHITE), paint(RED)
    return shapes.loft(rings, lambda c: np.where(c[:, 1] > 0.40, red, white), poles=(-0.53, 0.535),
                       along='y')


def _stack():
    """The coaxial pair's parts down the nose from MOTOR_Y: a motor, its board, its sink, the
    next motor, its board, its sink; and where the stack ends."""
    out, top = [], MOTOR_Y
    for k in (0, 1):
        for h, mesh, name in ((MOTOR_L, lambda: shapes.drum(MOTOR_R, MOTOR_L, 'y', PLATE), 'can'),
                              (BOARD_H, lambda: shapes.drum(BOARD_R, BOARD_H, 'y', paint(MASK)), 'board'),
                              (SINK_H, lambda: shapes.drum(SINK_R, SINK_H, 'y', paint(CARBON)), 'sink')):
            out.append((name + str(k), _at(mesh(), None, (0.0, top - h / 2.0, 0.0))))
            top -= h
    return out, top


def _aero_parts(see='shell'):
    """[(name, mesh)]: the aerobat airframe in its body axes; `see` a layer: the airframe its
    longerons, firewall, spars and ribs, the pack under the canopy, bare the shafts, the spars
    and the keel's line."""
    carbon, skin, accent = paint(CARBON), paint(WHITE), paint(RED)
    shell, frame = see == 'shell', see != 'bare'
    stack, aft = _stack()
    out = [('spinner', shapes.ellipsoid((0.0, PROP_Y + 0.024, 0.0), (0.028, 0.044, 0.028),
                                        paint(WHITE), rows=6)),
           ('shaft0', _at(shapes.drum(SHAFT_R, PROP_Y + 0.02 - aft, 'y', PLATE), None,
                          (0.0, 0.5 * (PROP_Y + 0.02 + aft), 0.0))),
           ('spar', _at(shapes.drum(0.007, 2.0 * HALF_SPAN, 'x', PLATE), None, (0.0, WING_Y, 0.0))),
           ('stabspar', _at(shapes.drum(0.005, 2.0 * HSTAB_HALF, 'x', PLATE), None,
                            (0.0, TAIL_Y + 0.02, 0.0))),
           ('finpost', _at(shapes.drum(0.005, 0.19, 'z', PLATE), None, (0.0, TAIL_Y + 0.04, -0.105)))]
    if shell:
        out += [('fuselage', _fuselage()),
                ('canopy', shapes.ellipsoid((0.0, 0.10, -0.078), (0.040, 0.165, 0.042), CORE, rows=7)),
                ('wing', _lifting(HALF_SPAN, ROOT_C, TIP_C, AIL_HINGE[1], lambda s, y, t: (s, y, t), skin,
                                  ((lambda s, y, side: _inked(s, y, side, AIL_HINGE[1])), paint(INK)), PX_W)),
                ('hstab', _lifting(HSTAB_HALF, 0.10, 0.06, TAIL_HINGE[1], lambda s, y, t: (s, y, t), skin)),
                ('vfin', _lifting(-0.19, 0.17, 0.06, TAIL_HINGE[1], lambda s, y, t: (t, y, -0.012 - s), accent))]
    elif frame:
        out += [('longeron%d' % k, _at(shapes.drum(0.005, aft + 0.52, 'y', PLATE), None,
                                       (sx * 0.03, 0.5 * (aft - 0.52), sz * 0.035)))
                for k, (sx, sz) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1)))]
        out += [('firewall', _at(shapes.drum(0.046, 0.006, 'y', PLATE), None, (0.0, aft - 0.003, 0.0)))]
        out += [('rib%d' % k, shapes.box((x, WING_Y, 0.0), (0.002, 0.46 * CHORD, 0.011), PLATE))
                for k, x in enumerate(HALF_SPAN * (j / 5.0) for j in range(-5, 6))]
        if see == 'pack':
            out.append(('pack', shapes.box((0.0, 0.20, 0.0), (0.022, 0.07, 0.02), paint(CELLS))))
    else:
        out.append(('keel', _at(shapes.drum(0.003, aft + 0.52, 'y', PLATE), None,
                                (0.0, 0.5 * (aft - 0.52), 0.0))))
    if not frame:
        return out
    out += stack + _tail_wheel(carbon)
    for s in (-1.0, 1.0):
        out += _bow(s, carbon, accent if shell else None)
    return out


def _prop_blades(radius):
    """Two symmetric tractor propeller blades in the x-z plane about the y axis, shown when slow."""
    prop = paint(PROP)
    return [('blade', shapes.ellipsoid((s * radius / 2.0, 0.0, 0.0),
                                       (radius / 2.0, 0.004, BLADE_W * 0.85), prop, rows=7))
            for s in (-1.0, 1.0)]


def _aileron_mesh(side):
    """One full-span 3D aileron behind its hinge on the wing's trailing edge, on `side`."""
    s = float(side)
    return [('ail', _lifting(-0.80 * HALF_SPAN, 0.075, 0.06, 0.0,
                             lambda d, y, t: (s * (0.12 * HALF_SPAN + d), -y, t), paint(WHITE)))]


def _elevator_mesh():
    """The oversized 3D elevator behind its hinge on the stabiliser's trailing edge."""
    return [('elev', _lifting(0.96 * HSTAB_HALF, 0.064, 0.05, 0.0, lambda d, y, t: (d, -y, t), paint(WHITE)))]


def _rudder_mesh():
    """The 3D rudder behind its hinge on the fin's trailing edge."""
    return [('rud', _lifting(-0.19, 0.075, 0.05, 0.0, lambda d, y, t: (t, -y, -0.012 - d), paint(RED)))]


#: The hinges, in the body: the ailerons' at the wing's trailing edge, the elevator's and the
#: rudder's at the tail's.
AIL_HINGE, TAIL_HINGE = (0.0, WING_Y - 0.42 * CHORD, 0.0), (0.0, TAIL_Y - 0.035, 0.0)


def aero_mesh(which, see='shell'):
    """The cached `Mesh` for `'frame'`, `'prop'`, `'vect'`, `'ail_l'`, `'ail_r'`, `'elev'`, `'rud'`."""
    got = _AERO_MESH.get((which, see))
    if got is None:
        builders = {
            'frame': lambda: _aero_parts(see),
            'prop': lambda: _prop_blades(PROP_R),
            'vect': lambda: _prop_blades(0.88 * PROP_R),
            'ail_l': lambda: _aileron_mesh(-1.0),
            'ail_r': lambda: _aileron_mesh(1.0),
            'elev': _elevator_mesh,
            'rud': _rudder_mesh,
        }
        got = _AERO_MESH[(which, see)] = Mesh(builders[which]())
    return got


def _posed(pose, rotors, dense, see='shell'):
    """(positions, normals, uv, materials, index) in the world: the aerobat airframe at its true
    physical `pose['turn']`, its slow coaxial tractor propellers, and its deflected 3D surfaces."""
    from coaxial.model.blocks import numpy as np
    turn, at = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    frame = aero_mesh('frame', see)
    pick = (lambda mesh: (mesh.dense[0], mesh.dense[1], mesh.dense[2], mesh.dense[3])) if dense \
        else (lambda mesh: (mesh.corners, mesh.normals, mesh.uv, mesh.materials))
    points, normals, uv, _materials = pick(frame)
    out = [[points @ turn.T + at], [normals @ turn.T], [uv], [_painted(frame, rotors, dense)]]
    index, base = [frame.index], len(points)
    _throt, elev, ail, rud = pose.get('controls', (0.0, 0.0, 0.0, 0.0))
    parts = [
        ('ail_l', shapes.turn_about('x', -32.0 * float(ail)), AIL_HINGE),
        ('ail_r', shapes.turn_about('x', 32.0 * float(ail)), AIL_HINGE),
        ('elev', shapes.turn_about('x', 36.0 * float(elev)), TAIL_HINGE),
        ('rud', shapes.turn_about('z', 36.0 * float(rud)), TAIL_HINGE),
    ]
    for (kind, y_hub, sign), rotor in zip((('prop', PROP_Y, 1.0), ('vect', VECT_Y, -1.0)), rotors):
        angle, w = float(rotor[0]), float(rotor[1])
        if abs(w) < BLADES_RAD_S or see == 'bare':
            parts.append((kind, shapes.turn_about('y', math.degrees(sign * angle)),
                          (0.0, y_hub, 0.0)))
    for kind, rot_local, spot in parts:
        mesh = aero_mesh(kind)
        here = turn @ rot_local
        p, n, u, mat = pick(mesh)
        out[0].append(p @ here.T + at + turn @ np.array(spot))
        out[1].append(n @ here.T)
        out[2].append(u)
        out[3].append(mat)
        index.append(mesh.index + base)
        base += len(p)
    return tuple(np.concatenate(o) for o in out) + (np.vstack(index),)


def _discs(pose, rotors, m, cam, centre):
    """[(dots, ink)]: when turning faster than `BLADES_RAD_S`, the coaxial tractor propeller
    swept disc rings in the x-z plane at the nose, their rims dotted."""
    from coaxial.model.blocks import numpy as np
    turn, at = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    rim = np.zeros((cam['height'], cam['width']), bool)
    k = np.arange(96) * (2.0 * math.pi / 96)
    unit = np.stack([np.cos(k), np.zeros_like(k), np.sin(k)], 1)
    for (y_hub, radius), (_angle, w, *_heat) in zip(((PROP_Y, PROP_R), (VECT_Y, 0.88 * PROP_R)),
                                                    rotors):
        if abs(w) < BLADES_RAD_S:
            continue
        hub = at + turn @ np.array((0.0, y_hub, 0.0))
        sx, sy, _w = project((unit * radius) @ turn.T + hub, m, cam, centre)
        drawn = np.zeros_like(rim)
        for a in range(len(k)):
            line(drawn, (sx[a], sy[a]), (sx[a - 1], sy[a - 1]))
        ys, xs = np.nonzero(drawn)
        keep = ((xs + ys) % RIM_EVERY) == 0
        rim[ys[keep], xs[keep]] = True
    return [(rim, RIM_INK)]


#: The propeller's thrust from the spinner along the nose, the rudder's from the tail across.
ARROWS = (((0.0, PROP_Y + 0.05, 0.0), 1, 0, 0.014), (TAIL_HINGE, 0, 2, 0.08))



def render(pose, rotors, width, height, yaw=30.0, pitch=18.0, reach=REACH, centre=None,
           colour=True, lit=None, world=True, gate=None, thrusts=None, line_=False, inset=None,
           see='shell'):
    """The 3D aerobatic monoplane at `pose` on its `rotors`, `width` x `height` cells, `reach` m
    framed about `centre` from SIGHT reaches off: lines."""
    from coaxial.model.blocks import numpy as np
    m = shapes.view(yaw, pitch)
    fine = engine.fine(engine.camera(width, height, reach, distance=SIGHT * reach))
    at = np.asarray(pose['at'], float)
    centre = at if centre is None else np.asarray(centre, float)
    if lit is not None:
        positions, normals, uv, materials, index = _posed(pose, rotors, False, see)
        depth, rgb = lit.raster(positions, normals, uv, materials, index, m, fine,
                                tuple(centre), float(np.linalg.norm(at - centre)) + REACH)
    else:
        positions, normals, uv, materials, _index = _posed(pose, rotors, True, see)
        depth, rgb = splat((positions, normals, materials, uv), m, fine, centre)
    own = ([] if see == 'bare' else _discs(pose, rotors, m, fine, centre)) + (
        force_arrows(pose, thrusts, ARROWS, m, fine, centre) if thrusts is not None else [])
    if not world:
        return braille(depth, rgb, np.zeros(depth.shape), width, height, colour, inset,
                       props=own)
    return braille(depth, rgb, scenery.ground(m, fine, centre), width, height, colour, inset,
                   props=own + scenery.props(m, fine, centre, gate, line_, airfield))
