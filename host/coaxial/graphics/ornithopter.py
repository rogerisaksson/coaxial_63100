"""The Dune-style 4-wing flapping ornithopter drawn: carapace, V-tail, 63100 drives and wings.

    lines = render(pose, rotors, 96, 40, lit=gpu.LitRaster())   # the GPU's lit raster
    lines = render(pose, rotors, 96, 40)                        # its dots splatted here

`pose` is `machine.ornithopter.Sky.state()`'s, its 'turn', 'at' and 'controls'; `rotors` the fore
and aft 63100 drives' (angle rad, speed rad/s, can C, board C[, sink C]), each can, board and
heatsink painted their heat (`ansi.thermal_rgb`). All four dragonfly wings articulate in diurnal
stroke, feathering pitch, and backward dive-fold sweep; above `BLADES_RAD_S`, their wingtip
stroke arcs shimmer in braille.
"""
import math

from coaxial.graphics import engine, scenery, shapes
from coaxial.graphics.callouts import line
from coaxial.graphics.helicopter import NEG_ARROW_INK
from coaxial.graphics.lit import CORE, PLATE, braille, paint, project, splat
from coaxial.graphics.quadcopter import (ARROW_HEAD, ARROW_INK, BLADES_RAD_S, BOARD_H, BOARD_R,
                                         CAN_L, CAN_R, CARBON, LEG_R, MASK, RIM_EVERY, RIM_INK,
                                         SINK_H, SINK_R, Mesh, _at, _painted)
from machine import ornithopter as orni_phys

#: The camera: the eye's distance in reaches framed, and the reach the ornithopter fills, m -
#: the 4 articulated wings spanning 0.56 m out, the split V-tail at -0.64 m.
SIGHT, REACH = 3.75, 0.68

#: The airframe's geometry, m: fore and aft wing shoulders on the dorsal carapace, wing span,
#: and landing skid height and span.
SHOULDER_X, SHOULDER_Y = orni_phys.SHOULDER_X, orni_phys.SHOULDER_Y
FORE_Z, AFT_Z = orni_phys.FORE_Z, orni_phys.AFT_Z
WING_SPAN, AFT_SPAN = orni_phys.WING_SPAN, 0.88 * orni_phys.WING_SPAN
SKID_M, FOOT_M, SKID_X, STRUT_Z = (
    orni_phys.SKID_M, orni_phys.FOOT_M, orni_phys.SKID_X, orni_phys.STRUT_Z)

_ORNI_MESH = {}


def _carapace():
    """The streamlined carbon thoracic carapace along +z above the twin 63100 drives."""
    stations = [
        (-0.12, 0.008, 0.044, 0.032), (-0.06, 0.004, 0.056, 0.052),
        (0.02, -0.006, 0.060, 0.058), (0.10, -0.010, 0.054, 0.054),
        (0.18, -0.012, 0.042, 0.042), (0.25, -0.010, 0.024, 0.026),
        (0.31, -0.008, 0.004, 0.008),
    ]
    rings = [(z, rx, 0.5 * (yt - yb), -0.5 * (yt + yb), 0.0) for z, yb, yt, rx in stations]
    return shapes.loft(rings, paint(CARBON), poles=(-0.13, 0.32), along='z')


def _boom():
    """The slender carbon dragonfly tail boom from the thoracic carapace (-0.12 m) to the V-tail
    (-0.62 m)."""
    rings = [(-0.62 + (i / 10.0) * 0.50,
              0.009 + (i / 10.0) * 0.007, 0.009 + (i / 10.0) * 0.007,
              -0.018, 0.0) for i in range(11)]
    return shapes.loft(rings, paint(CARBON), poles=(-0.63, -0.11), along='z')


def _orni_parts():
    """[(name, mesh)]: the ornithopter body in its own axes, x right, y up, z ahead."""
    carbon = paint(CARBON)
    can_h = CAN_L * 0.72
    out = [
        ('carapace', _carapace()),
        ('visor', shapes.ellipsoid((0.0, 0.036, 0.155), (0.040, 0.020, 0.072), CORE, rows=7)),
        ('boom', _boom()),
        ('can0', _at(shapes.drum(CAN_R, can_h, 'y', PLATE), None,
                     (0.0, 0.004 - can_h / 2.0, FORE_Z))),
        ('board0', _at(shapes.drum(BOARD_R, BOARD_H, 'y', paint(MASK)), None,
                       (0.0, 0.004 - can_h - BOARD_H / 2.0, FORE_Z))),
        ('sink0', _at(shapes.drum(SINK_R, SINK_H, 'y', carbon), None,
                      (0.0, 0.004 - can_h - BOARD_H - SINK_H / 2.0, FORE_Z))),
        ('can1', _at(shapes.drum(CAN_R, can_h, 'y', PLATE), None,
                     (0.0, 0.004 - can_h / 2.0, AFT_Z))),
        ('board1', _at(shapes.drum(BOARD_R, BOARD_H, 'y', paint(MASK)), None,
                       (0.0, 0.004 - can_h - BOARD_H / 2.0, AFT_Z))),
        ('sink1', _at(shapes.drum(SINK_R, SINK_H, 'y', carbon), None,
                      (0.0, 0.004 - can_h - BOARD_H - SINK_H / 2.0, AFT_Z))),
    ]
    for s in (-1.0, 1.0):
        out.append(('sh_f%d' % (s > 0),
                    shapes.ellipsoid((s * SHOULDER_X, SHOULDER_Y, FORE_Z),
                                     (0.016, 0.012, 0.016), PLATE, rows=6)))
        out.append(('sh_a%d' % (s > 0),
                    shapes.ellipsoid((s * SHOULDER_X, SHOULDER_Y + 0.004, AFT_Z),
                                     (0.015, 0.011, 0.015), PLATE, rows=6)))
        out.append(('vtail%d' % (s > 0),
                    _at(shapes.ellipsoid((0.0, 0.0, 0.0), (0.088, 0.005, 0.022), carbon, rows=6),
                        shapes.turn_about('z', s * 34.0) @ shapes.turn_about('y', -s * 16.0),
                        (s * 0.068, 0.052, -0.58))))
        out.append(('claw_rail%d' % (s > 0),
                    _at(shapes.drum(LEG_R, 0.30, 'z', carbon), None,
                        (s * SKID_X, -SKID_M, 0.025))))
        for k, z in enumerate(STRUT_Z):
            out.append(('leg%d_%d' % (s > 0, k),
                        _at(shapes.drum(LEG_R, SKID_M - 0.032, 'y', carbon),
                            shapes.turn_about('z', s * 20.0),
                            (s * 0.78 * SKID_X, -0.5 * (SKID_M + 0.032), z))))
            out.append(('claw%d_%d' % (s > 0, k),
                        shapes.ellipsoid((s * SKID_X, -SKID_M, z), (FOOT_M,) * 3, PLATE, rows=6)))
    return out


def _wing_blade(span, side):
    """One slender dragonfly wing extending along `side * +x` from its shoulder joint at the origin."""
    carbon = paint(CARBON)
    s = float(side)
    return [
        ('spar', _at(shapes.drum(0.007, 0.88 * span, 'x', carbon), None,
                     (s * 0.44 * span, 0.003, 0.012))),
        ('blade', shapes.ellipsoid((s * 0.50 * span, 0.0, -0.004),
                                   (0.50 * span, 0.004, 0.020), carbon, rows=7)),
    ]


def orni_mesh(which):
    """The cached `Mesh` for `'frame'`, `'fore_l'`, `'fore_r'`, `'aft_l'`, or `'aft_r'`."""
    got = _ORNI_MESH.get(which)
    if got is None:
        builders = {
            'frame': _orni_parts,
            'fore_l': lambda: _wing_blade(WING_SPAN, -1.0),
            'fore_r': lambda: _wing_blade(WING_SPAN, 1.0),
            'aft_l': lambda: _wing_blade(AFT_SPAN, -1.0),
            'aft_r': lambda: _wing_blade(AFT_SPAN, 1.0),
        }
        got = _ORNI_MESH[which] = Mesh(builders[which]())
    return got


def _wing_pose_angles(rotors, controls):
    """Compute `(dihedral_deg, pitch_deg, sweep_deg)` for each of the 4 wings (`fore_l`, `fore_r`,
    `aft_l`, `aft_r`) from the live drive azimuths `rotors` and normalized `controls`."""
    stroke, pitch_t, roll_d, yaw_t = controls if controls is not None else (0.45, 0.0, 0.0, 0.0)
    ang_f, w_f = float(rotors[0][0]), float(rotors[0][1])
    ang_a, w_a = float(rotors[1][0]), float(rotors[1][1])
    flapping = max(abs(w_f), abs(w_a)) >= BLADES_RAD_S
    fold = max(0.0, min(1.0, -stroke - 0.10)) if stroke < -0.10 else 0.0
    if not flapping:
        return ((20.0, 6.0, 14.0), (20.0, 6.0, 14.0),
                (-8.0, 4.0, -20.0), (-8.0, 4.0, -20.0))
    amp = (22.0 + 18.0 * abs(stroke)) * (1.0 - 0.65 * fold)
    phase_f = ang_f
    phase_a = ang_a + math.pi
    dih_fl = 12.0 + amp * (1.0 - 0.25 * roll_d) * math.sin(phase_f)
    dih_fr = 12.0 + amp * (1.0 + 0.25 * roll_d) * math.sin(phase_f)
    dih_al = -6.0 + 0.90 * amp * (1.0 - 0.25 * roll_d) * math.sin(phase_a)
    dih_ar = -6.0 + 0.90 * amp * (1.0 + 0.25 * roll_d) * math.sin(phase_a)
    feather_f = 10.0 * math.cos(phase_f) + 12.0 * pitch_t
    feather_a = 8.0 * math.cos(phase_a) + 14.0 * yaw_t
    sweep_f = 16.0 - 50.0 * fold + 8.0 * pitch_t
    sweep_a = -20.0 - 40.0 * fold - 8.0 * pitch_t
    return ((dih_fl, feather_f, sweep_f), (dih_fr, feather_f, sweep_f),
            (dih_al, feather_a - 10.0 * yaw_t, sweep_a), (dih_ar, feather_a + 10.0 * yaw_t, sweep_a))


def _posed(pose, rotors, dense):
    """(positions, normals, uv, materials, index) in the world: the ornithopter carapace at `pose`
    and all four dragonfly wings articulated at their shoulder joints."""
    from coaxial.model.blocks import numpy as np
    turn, at = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    frame = orni_mesh('frame')
    pick = (lambda mesh: (mesh.dense[0], mesh.dense[1], mesh.dense[2], mesh.dense[3])) if dense \
        else (lambda mesh: (mesh.corners, mesh.normals, mesh.uv, mesh.materials))
    points, normals, uv, _materials = pick(frame)
    out = [[points @ turn.T + at], [normals @ turn.T], [uv], [_painted(frame, rotors, dense)]]
    index, base = [frame.index], len(points)
    wings = (('fore_l', -1.0, (-SHOULDER_X, SHOULDER_Y, FORE_Z)),
              ('fore_r', 1.0, (SHOULDER_X, SHOULDER_Y, FORE_Z)),
              ('aft_l', -1.0, (-SHOULDER_X, SHOULDER_Y + 0.004, AFT_Z)),
              ('aft_r', 1.0, (SHOULDER_X, SHOULDER_Y + 0.004, AFT_Z)))
    angles = _wing_pose_angles(rotors, pose.get('controls'))
    for (kind, side, spot), (dih, feather, sweep) in zip(wings, angles):
        wing = orni_mesh(kind)
        rot_local = (shapes.turn_about('y', -side * sweep)
                     @ shapes.turn_about('z', side * dih)
                     @ shapes.turn_about('x', feather))
        here = turn @ rot_local
        p, n, u, mat = pick(wing)
        out[0].append(p @ here.T + at + turn @ np.array(spot))
        out[1].append(n @ here.T)
        out[2].append(u)
        out[3].append(mat)
        index.append(wing.index + base)
        base += len(p)
    return tuple(np.concatenate(o) for o in out) + (np.vstack(index),)


def _stroke_arcs(pose, rotors, m, cam, centre):
    """[(dots, ink)]: when flapping above `BLADES_RAD_S`, fore and aft wingtip stroke-envelope
    arcs sweeping through the diurnal dihedral arc on both sides of the carapace."""
    from coaxial.model.blocks import numpy as np
    turn, at = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    rim = np.zeros((cam['height'], cam['width']), bool)
    stroke = (pose.get('controls') or (0.45, 0.0, 0.0, 0.0))[0]
    fold = max(0.0, min(1.0, -stroke - 0.10)) if stroke < -0.10 else 0.0
    pairs = ((WING_SPAN, FORE_Z + 0.05 - 0.18 * fold, (-0.28, 0.56), rotors[0][1]),
             (AFT_SPAN, AFT_Z - 0.07 - 0.14 * fold, (-0.42, 0.34), rotors[1][1]))
    for span, z_tip, (phi0, phi1), w in pairs:
        if abs(float(w)) < BLADES_RAD_S:
            continue
        phi = np.linspace(phi0, phi1, 28)
        for side in (-1.0, 1.0):
            arc = np.stack([side * (SHOULDER_X + span * np.cos(phi)),
                            SHOULDER_Y + span * np.sin(phi),
                            np.full_like(phi, z_tip) + 0.025 * np.sin(2.0 * phi)], 1)
            sx, sy, _w = project(arc @ turn.T + at, m, cam, centre)
            drawn = np.zeros_like(rim)
            for a in range(1, len(phi)):
                line(drawn, (sx[a], sy[a]), (sx[a - 1], sy[a - 1]))
            ys, xs = np.nonzero(drawn)
            keep = ((xs + ys) % RIM_EVERY) == 0
            rim[ys[keep], xs[keep]] = True
    return [(rim, RIM_INK)] if rim.any() else []


def _arrows(pose, thrusts, m, cam, centre):
    """[(dots, ink)]: fore and aft wing-pair stroke thrust arrows from the dorsal shoulders."""
    from coaxial.model.blocks import numpy as np
    turn, at = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    up, side = turn[:, 1], turn[:, 0]
    pos_drawn = np.zeros((cam['height'], cam['width']), bool)
    neg_drawn = np.zeros((cam['height'], cam['width']), bool)
    spots = ((0.0, SHOULDER_Y + 0.035, FORE_Z), (0.0, SHOULDER_Y + 0.035, AFT_Z))
    for spot, thrust in zip(spots, thrusts):
        length = 0.022 * abs(thrust)
        if length <= 1e-4:
            continue
        way = up if thrust >= 0.0 else -up
        target = pos_drawn if thrust >= 0.0 else neg_drawn
        hub = at + turn @ np.array(spot)
        tip = hub + length * way
        back = tip - ARROW_HEAD * length * way
        points = np.array([hub, tip, back + 0.5 * ARROW_HEAD * length * side,
                           back - 0.5 * ARROW_HEAD * length * side])
        sx, sy, _w = project(points, m, cam, centre)
        for a, b in ((0, 1), (1, 2), (1, 3)):
            line(target, (sx[a], sy[a]), (sx[b], sy[b]))
    out = [(pos_drawn, ARROW_INK)]
    if neg_drawn.any():
        out.append((neg_drawn, NEG_ARROW_INK))
    return out


def render(pose, rotors, width, height, yaw=30.0, pitch=18.0, reach=REACH, centre=None,
           colour=True, lit=None, world=True, gate=None, thrusts=None, line_=False, inset=None):
    """The 4-wing Dune ornithopter at `pose` on its `rotors`, `width` x `height` cells, `reach` m
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
    own = _stroke_arcs(pose, rotors, m, fine, centre) + (
        _arrows(pose, thrusts, m, fine, centre) if thrusts is not None else [])
    if not world:
        return braille(depth, rgb, np.zeros(depth.shape), width, height, colour, inset,
                       props=own)
    return braille(depth, rgb, scenery.ground(m, fine, centre), width, height, colour, inset,
                   props=own + scenery.props(m, fine, centre, gate, line_))
