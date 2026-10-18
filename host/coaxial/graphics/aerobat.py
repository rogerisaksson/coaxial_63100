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
                                         LEG_R, MASK, MOTOR_R, PROP, RIM_EVERY, RIM_INK, SHAFT_R,
                                         SINK_H, SINK_R, ACCENT, Mesh, _at, _painted)
from machine import aerobat as aero_phys

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

_AERO_MESH = {}


SURFACE = (92, 98, 112)
#: The Edge 540's navy, its yellow cowl and white spinner.
NAVY, YELLOW, WHITE = (28, 42, 98), (240, 190, 40), (236, 236, 232)


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
    navy, yellow = paint(NAVY), paint(YELLOW)
    return shapes.loft(rings, lambda c: np.where(c[:, 1] > 0.40, yellow, navy), poles=(-0.53, 0.535),
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
    carbon, skin, accent = paint(CARBON), paint(NAVY), paint(ACCENT)
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
                ('vfin', shapes.box((0.0, TAIL_Y + 0.04, -0.105), (0.005, 0.075, 0.095), accent)),
                ('finlead', shapes.box((0.0, TAIL_Y + 0.14, -0.06), (0.005, 0.03, 0.045), accent))]
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
    out += stack
    legs = ((STRUT_Z[1], GEAR_MAIN_Z, 1.0), (STRUT_Z[0], GEAR_TAIL_Z, 0.45))
    for s in (-1.0, 1.0):
        if shell:
            out.append(('wing%d' % (s > 0),
                        shapes.box((s * 0.50 * HALF_SPAN, WING_Y, 0.0),
                                   (0.50 * HALF_SPAN, 0.46 * CHORD, 0.011), skin)))
            out.append(('tip%d' % (s > 0),
                        shapes.box((s * 0.93 * HALF_SPAN, WING_Y, 0.0),
                                   (0.07 * HALF_SPAN, 0.46 * CHORD, 0.012), accent)))
            out.append(('hstab%d' % (s > 0),
                        shapes.box((s * 0.5 * HSTAB_HALF, TAIL_Y + 0.02, 0.0),
                                   (0.5 * HSTAB_HALF, 0.05, 0.005), skin)))
        for k, (y, z, x_sc) in enumerate(legs):
            x = s * x_sc * SKID_X
            dx, dz = x - s * 0.04, z - 0.03
            length = math.hypot(dx, dz)
            out.append(('gear_leg%d_%d' % (s > 0, k),
                        _at(shapes.drum(LEG_R, length, 'z', carbon),
                            shapes.turn_about('y', -math.degrees(math.atan2(dx, dz))),
                            (s * 0.04 + 0.5 * dx, y, 0.03 + 0.5 * dz))))
            out.append(('wheel%d_%d' % (s > 0, k),
                        shapes.ellipsoid((x, y, z), (FOOT_M, FOOT_M, FOOT_M), PLATE, rows=6)))
            if k == 0 and shell:
                out.append(('pant%d' % (s > 0),
                            shapes.ellipsoid((x, y + 0.005, z - 0.01), (0.014, 0.06, 0.042),
                                             accent, rows=6)))
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
    return [('ail', shapes.box((s * 0.56 * HALF_SPAN, -0.038, 0.0),
                               (0.44 * HALF_SPAN, 0.038, 0.006), paint(SURFACE)))]


def _elevator_mesh():
    """The oversized 3D elevator behind its hinge on the stabiliser's trailing edge."""
    return [('elev', shapes.box((s * 0.5 * HSTAB_HALF, -0.032, 0.0),
                                (0.5 * HSTAB_HALF, 0.032, 0.005), paint(SURFACE)))
            for s in (-1.0, 1.0)]


def _rudder_mesh():
    """The 3D rudder behind its hinge on the fin's trailing edge."""
    return [('rud', shapes.box((0.0, -0.036, -0.10), (0.005, 0.036, 0.090), paint(SURFACE)))]


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
                   props=own + scenery.props(m, fine, centre, gate, line_))
