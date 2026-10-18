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
from coaxial.graphics.quadcopter import (BLADE_W, BLADES_RAD_S, BOARD_H, BOARD_R, CAN_L, CAN_R,
                                         CARBON, LEG_R, MASK, PROP, RIM_EVERY, RIM_INK, SINK_H,
                                         SINK_R, SKIN, ACCENT, Mesh, _at, _painted)
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

_AERO_MESH = {}


SURFACE = (92, 98, 112)


def _fuselage():
    """The fuselage along +y from the tail post to the cowl: deep under the canopy, flat-sided
    aft, a round cowl ahead."""
    rings = [
        (-0.52, 0.010, 0.016), (-0.42, 0.020, 0.032), (-0.30, 0.032, 0.050),
        (-0.16, 0.046, 0.066), (-0.02, 0.056, 0.078), (0.12, 0.060, 0.080),
        (0.26, 0.058, 0.072), (0.38, 0.052, 0.058), (0.47, 0.044, 0.046),
        (0.53, 0.034, 0.034),
    ]
    return shapes.loft(rings, paint(SKIN), poles=(-0.53, 0.535), along='y')


def _aero_parts():
    """[(name, mesh)]: the aerobat airframe in its body axes."""
    carbon, skin, accent = paint(CARBON), paint(SKIN), paint(ACCENT)
    can_h = CAN_L * 0.58
    out = [
        ('fuselage', _fuselage()),
        ('canopy', shapes.ellipsoid((0.0, 0.10, -0.078), (0.040, 0.165, 0.042), CORE, rows=7)),
        ('spinner', shapes.ellipsoid((0.0, PROP_Y + 0.024, 0.0), (0.028, 0.044, 0.028),
                                     PLATE, rows=6)),
        ('can0', _at(shapes.drum(CAN_R, can_h, 'y', PLATE), None, (0.0, 0.47, 0.0))),
        ('board0', _at(shapes.drum(BOARD_R, BOARD_H, 'y', paint(MASK)), None,
                       (0.0, 0.47 - can_h / 2.0 - BOARD_H / 2.0, 0.0))),
        ('sink0', _at(shapes.drum(SINK_R, SINK_H, 'y', carbon), None,
                      (0.0, 0.47 - can_h / 2.0 - BOARD_H - SINK_H / 2.0, 0.0))),
        ('can1', _at(shapes.drum(CAN_R, can_h, 'y', PLATE), None, (0.0, 0.385, 0.0))),
        ('board1', _at(shapes.drum(BOARD_R, BOARD_H, 'y', paint(MASK)), None,
                       (0.0, 0.385 - can_h / 2.0 - BOARD_H / 2.0, 0.0))),
        ('sink1', _at(shapes.drum(SINK_R, SINK_H, 'y', carbon), None,
                      (0.0, 0.385 - can_h / 2.0 - BOARD_H - SINK_H / 2.0, 0.0))),
        ('vfin', shapes.box((0.0, TAIL_Y + 0.04, -0.105), (0.005, 0.075, 0.095), accent)),
        ('finlead', shapes.box((0.0, TAIL_Y + 0.14, -0.06), (0.005, 0.03, 0.045), accent)),
    ]
    legs = ((STRUT_Z[1], GEAR_MAIN_Z, 1.0), (STRUT_Z[0], GEAR_TAIL_Z, 0.45))
    for s in (-1.0, 1.0):
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
            if k == 0:
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


def aero_mesh(which):
    """The cached `Mesh` for `'frame'`, `'prop'`, `'vect'`, `'ail_l'`, `'ail_r'`, `'elev'`, `'rud'`."""
    got = _AERO_MESH.get(which)
    if got is None:
        builders = {
            'frame': _aero_parts,
            'prop': lambda: _prop_blades(PROP_R),
            'vect': lambda: _prop_blades(0.88 * PROP_R),
            'ail_l': lambda: _aileron_mesh(-1.0),
            'ail_r': lambda: _aileron_mesh(1.0),
            'elev': _elevator_mesh,
            'rud': _rudder_mesh,
        }
        got = _AERO_MESH[which] = Mesh(builders[which]())
    return got


def _posed(pose, rotors, dense):
    """(positions, normals, uv, materials, index) in the world: the aerobat airframe at its true
    physical `pose['turn']`, its slow coaxial tractor propellers, and its deflected 3D surfaces."""
    from coaxial.model.blocks import numpy as np
    turn, at = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    frame = aero_mesh('frame')
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
        if abs(w) < BLADES_RAD_S:
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
           colour=True, lit=None, world=True, gate=None, thrusts=None, line_=False, inset=None):
    """The 3D aerobatic monoplane at `pose` on its `rotors`, `width` x `height` cells, `reach` m
    framed about `centre` from SIGHT reaches off: lines."""
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
    own = _discs(pose, rotors, m, fine, centre) + (
        force_arrows(pose, thrusts, ARROWS, m, fine, centre) if thrusts is not None else [])
    if not world:
        return braille(depth, rgb, np.zeros(depth.shape), width, height, colour, inset,
                       props=own)
    return braille(depth, rgb, scenery.ground(m, fine, centre), width, height, colour, inset,
                   props=own + scenery.props(m, fine, centre, gate, line_))
