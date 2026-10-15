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
from coaxial.graphics.helicopter import NEG_ARROW_INK
from coaxial.graphics.lit import CORE, PLATE, braille, paint, project, splat
from coaxial.graphics.quadcopter import (ARROW_HEAD, ARROW_INK, BLADE_W, BLADES_RAD_S, BOARD_H,
                                         BOARD_R, CAN_L, CAN_R, CARBON, LEG_R, MASK, PROP,
                                         RIM_EVERY, RIM_INK, SINK_H, SINK_R, Mesh, _at, _painted)
from machine import aerobat as aero_phys

#: The camera: the eye's distance in reaches framed, and the reach the monoplane fills, m -
#: the main wing tips 0.52 m out, the coaxial nose spinner at +0.36 m.
SIGHT, REACH = 3.75, 0.68

#: The airframe's geometry in its true MuJoCo body axes (x right wing, y nose/propeller axis,
#: -z dorsal canopy/fin, +z ventral landing gear), m: coaxial tractor propeller hubs at the
#: nose, main wing half-span, and taildragger main/tail wheels.
PROP_Y, VECT_Y, PROP_R = aero_phys.PROP_Z, aero_phys.PROP_Z - 0.045, aero_phys.PROP_R
HALF_SPAN, WING_Y = aero_phys.HALF_M, 0.055
TAIL_Y, HSTAB_SPAN = -0.105, 0.175
SKID_M, FOOT_M, SKID_X, STRUT_Z = (
    aero_phys.SKID_M, aero_phys.FOOT_M, aero_phys.SKID_X, aero_phys.STRUT_Z)
GEAR_MAIN_Z, GEAR_TAIL_Z = aero_phys.GEAR_MAIN_Z, aero_phys.GEAR_TAIL_Z

_AERO_MESH = {}


SURFACE = (92, 98, 112)


def _fuselage():
    """The slender carbon fuselage spine along the +y tractor thrust axis from the tail post
    (-0.115 m) to the coaxial nose mount (+0.285 m)."""
    rings = [
        (-0.115, 0.009, 0.011), (-0.065, 0.012, 0.015),
        (-0.010, 0.015, 0.019), (0.055, 0.018, 0.022),
        (0.125, 0.016, 0.019), (0.205, 0.014, 0.015),
        (0.280, 0.010, 0.010),
    ]
    return shapes.loft(rings, paint(CARBON), poles=(-0.122, 0.288), along='y')


def _aero_parts():
    """[(name, mesh)]: the aerobat airframe in its true MuJoCo body axes (x right wing, y nose
    tractor axis, -z dorsal, +z ventral)."""
    carbon = paint(CARBON)
    can_h = CAN_L * 0.58
    out = [
        ('fuselage', _fuselage()),
        ('canopy', shapes.ellipsoid((0.0, 0.050, -0.028), (0.026, 0.068, 0.022), CORE, rows=7)),
        ('spinner', shapes.ellipsoid((0.0, PROP_Y + 0.018, 0.0), (0.016, 0.024, 0.016),
                                     PLATE, rows=6)),
        ('can0', _at(shapes.drum(CAN_R, can_h, 'y', PLATE), None,
                     (0.0, 0.246, 0.0))),
        ('board0', _at(shapes.drum(BOARD_R, BOARD_H, 'y', paint(MASK)), None,
                       (0.0, 0.246 - can_h / 2.0 - BOARD_H / 2.0, 0.0))),
        ('sink0', _at(shapes.drum(SINK_R, SINK_H, 'y', carbon), None,
                      (0.0, 0.246 - can_h / 2.0 - BOARD_H - SINK_H / 2.0, 0.0))),
        ('can1', _at(shapes.drum(CAN_R, can_h, 'y', PLATE), None,
                     (0.0, 0.162, 0.0))),
        ('board1', _at(shapes.drum(BOARD_R, BOARD_H, 'y', paint(MASK)), None,
                       (0.0, 0.162 - can_h / 2.0 - BOARD_H / 2.0, 0.0))),
        ('sink1', _at(shapes.drum(SINK_R, SINK_H, 'y', carbon), None,
                      (0.0, 0.162 - can_h / 2.0 - BOARD_H - SINK_H / 2.0, 0.0))),
        ('vfin', shapes.ellipsoid((0.0, TAIL_Y + 0.012, -0.052), (0.004, 0.026, 0.064),
                                  carbon, rows=6)),
    ]
    gear = ((-SKID_M, GEAR_TAIL_Z, 0.45, 0.085, 28.0),
            (STRUT_Z[1], GEAR_MAIN_Z, 1.0, 0.145, -32.0))
    for s in (-1.0, 1.0):
        out.append(('spar%d' % (s > 0),
                    _at(shapes.drum(0.007, HALF_SPAN - 0.04, 'x', carbon), None,
                        (s * 0.5 * (HALF_SPAN + 0.02), WING_Y + 0.016, 0.0))))
        out.append(('wing%d' % (s > 0),
                    shapes.ellipsoid((s * 0.52 * HALF_SPAN, WING_Y, 0.0),
                                     (0.48 * HALF_SPAN, 0.036, 0.004), carbon, rows=7)))
        out.append(('hstab%d' % (s > 0),
                    shapes.ellipsoid((s * 0.52 * HSTAB_SPAN, TAIL_Y + 0.010, 0.0),
                                     (0.48 * HSTAB_SPAN, 0.022, 0.004), carbon, rows=6)))
        for k, (by, bz, x_sc, leg_l, ang_x) in enumerate(gear):
            turn_leg = shapes.turn_about('y', -s * 28.0) @ shapes.turn_about('x', ang_x)
            out.append(('gear_leg%d_%d' % (s > 0, k),
                        _at(shapes.drum(LEG_R, leg_l, 'z', carbon), turn_leg,
                            (0.55 * s * x_sc * SKID_X, by, 0.52 * bz))))
            out.append(('wheel%d_%d' % (s > 0, k),
                        shapes.ellipsoid((s * x_sc * SKID_X, by, bz),
                                         (FOOT_M,) * 3, PLATE, rows=6)))
    return out


def _prop_blades(radius):
    """Two symmetric tractor propeller blades in the x-z plane about the y axis, shown when slow."""
    prop = paint(PROP)
    return [('blade', shapes.ellipsoid((s * radius / 2.0, 0.0, 0.0),
                                       (radius / 2.0, 0.004, BLADE_W * 0.85), prop, rows=7))
            for s in (-1.0, 1.0)]


def _aileron_mesh(side):
    """One full-span 3D aileron along the trailing edge (-y) of the wing on `side` (`-1` or `+1`)."""
    s = float(side)
    return [('ail', shapes.ellipsoid((s * 0.54 * HALF_SPAN, -0.016, 0.0),
                                     (0.42 * HALF_SPAN, 0.015, 0.004), paint(SURFACE), rows=6))]


def _elevator_mesh():
    """The oversized 3D elevator hinged along the horizontal stabilizer trailing edge (-y)."""
    return [('elev', shapes.ellipsoid((s * 0.54 * HSTAB_SPAN, -0.014, 0.0),
                                      (0.44 * HSTAB_SPAN, 0.014, 0.004), paint(SURFACE), rows=6))
            for s in (-1.0, 1.0)]


def _rudder_mesh():
    """The counterbalanced 3D rudder hinged along the dorsal vertical fin trailing edge (-y, -z)."""
    return [('rud', shapes.ellipsoid((0.0, -0.016, -0.044),
                                     (0.004, 0.016, 0.068), paint(SURFACE), rows=6))]


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
        ('ail_l', shapes.turn_about('x', -32.0 * float(ail)), (0.0, WING_Y - 0.036, 0.0)),
        ('ail_r', shapes.turn_about('x', 32.0 * float(ail)), (0.0, WING_Y - 0.036, 0.0)),
        ('elev', shapes.turn_about('x', 36.0 * float(elev)), (0.0, TAIL_Y - 0.012, 0.0)),
        ('rud', shapes.turn_about('z', 36.0 * float(rud)), (0.0, TAIL_Y - 0.012, 0.0)),
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


def _arrows(pose, thrusts, m, cam, centre):
    """[(dots, ink)]: 3D tractor thrust arrow along the true physical thrust axis `turn[:, 1]`
    from the nose spinner and yaw/rudder torque arrow from the tail."""
    from coaxial.model.blocks import numpy as np
    turn, at = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    up, side, fwd = turn[:, 1], turn[:, 0], turn[:, 2]
    pos_drawn = np.zeros((cam['height'], cam['width']), bool)
    neg_drawn = np.zeros((cam['height'], cam['width']), bool)
    specs = (((0.0, PROP_Y + 0.025, 0.0), up, side, 0.014, True),
             ((0.0, TAIL_Y - 0.020, 0.0), side, fwd, 0.08, False))
    for (spot, axis, cross_axis, scale, nose_out), thrust in zip(specs, thrusts):
        length = scale * abs(thrust)
        if length <= 1e-4:
            continue
        way = (-axis if (axis[1] > 0.0 and thrust < 0.0) else axis) if nose_out else (
            axis if thrust >= 0.0 else -axis)
        target = pos_drawn if thrust >= 0.0 else neg_drawn
        origin = (0.0, -SKID_M, 0.0) if (nose_out and axis[1] > 0.0 and thrust < 0.0) else spot
        hub = at + turn @ np.array(origin)
        tip = hub + length * way
        back = tip - ARROW_HEAD * length * way
        points = np.array([hub, tip, back + 0.5 * ARROW_HEAD * length * cross_axis,
                           back - 0.5 * ARROW_HEAD * length * cross_axis])
        sx, sy, _w = project(points, m, cam, centre)
        for a, b in ((0, 1), (1, 2), (1, 3)):
            line(target, (sx[a], sy[a]), (sx[b], sy[b]))
    out = [(pos_drawn, ARROW_INK)]
    if neg_drawn.any():
        out.append((neg_drawn, NEG_ARROW_INK))
    return out


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
        _arrows(pose, thrusts, m, fine, centre) if thrusts is not None else [])
    if not world:
        return braille(depth, rgb, np.zeros(depth.shape), width, height, colour, inset,
                       props=own)
    return braille(depth, rgb, scenery.ground(m, fine, centre), width, height, colour, inset,
                   props=own + scenery.props(m, fine, centre, gate, line_))
