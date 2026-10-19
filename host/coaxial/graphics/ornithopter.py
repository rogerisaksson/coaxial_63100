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

from coaxial.graphics import shapes
from coaxial.graphics.callouts import line
from coaxial.graphics.helicopter import force_arrows
from coaxial.graphics.lit import CORE, PLATE, paint, project
from coaxial.graphics.quadcopter import (BLADES_RAD_S, BOARD_H, BOARD_R, CARBON, CELLS, LEG_R,
                                         MASK, MOTOR_R, RIM_EVERY, RIM_INK, SHAFT_R, SINK_H,
                                         SINK_R, Mesh, _at, pose_meshes, render_craft)
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
#: A wing pair's motor, 42 x 40 mm (304 W at the governed top, aircraft.md), lying along z in
#: the thorax at MOTOR_Y, its 63100 board behind it, its crank up to the shoulders.
MOTOR_L, MOTOR_Y = 0.040, 0.020

_ORNI_MESH = {}


#: The pod's dusty black, its wings' sand-coloured skin.
DUSK, WING = (58, 54, 48), (150, 142, 126)


def _carapace():
    """The pod along +z: a tapering tail boom, a deep thorax under the shoulders, the cockpit's
    nose ahead, dusty black."""
    stations = [
        (-0.68, 0.000, 0.018, 0.012), (-0.55, -0.006, 0.028, 0.020),
        (-0.40, -0.020, 0.050, 0.036), (-0.24, -0.042, 0.074, 0.060),
        (-0.08, -0.056, 0.088, 0.080), (0.06, -0.060, 0.094, 0.090),
        (0.18, -0.056, 0.092, 0.088), (0.28, -0.044, 0.084, 0.078),
        (0.36, -0.026, 0.066, 0.060), (0.42, -0.004, 0.040, 0.034),
    ]
    rings = [(z, rx, 0.5 * (yt - yb), -0.5 * (yt + yb), 0.0) for z, yb, yt, rx in stations]
    return shapes.loft(rings, paint(DUSK), poles=(-0.69, 0.43), along='z')


def _drive(k, z, frame):
    """The wing pair's drive at station `z`: its motor aft of it along z with the board and
    sink behind, or bare its shaft; the crank up to the shoulders' cross shaft."""
    out = [('crank%d' % k, _at(shapes.drum(SHAFT_R, SHOULDER_Y - MOTOR_Y, 'y', PLATE), None,
                               (0.0, 0.5 * (SHOULDER_Y + MOTOR_Y), z))),
           ('cross%d' % k, _at(shapes.drum(SHAFT_R, 2.0 * SHOULDER_X, 'x', PLATE), None,
                               (0.0, SHOULDER_Y, z)))]
    if not frame:
        return out
    stack = ((MOTOR_L, lambda: shapes.drum(MOTOR_R, MOTOR_L, 'z', PLATE), 'can'),
             (BOARD_H, lambda: shapes.drum(BOARD_R, BOARD_H, 'z', paint(MASK)), 'board'),
             (SINK_H, lambda: shapes.drum(SINK_R, SINK_H, 'z', paint(CARBON)), 'sink'))
    back = z
    for h, mesh, name in stack:
        out.append((name + str(k), _at(mesh(), None, (0.0, MOTOR_Y, back - h / 2.0))))
        back -= h
    return out


def _orni_parts(see='shell'):
    """[(name, mesh)]: the ornithopter body in its own axes, x right, y up, z ahead; `see` a
    layer: the airframe a spine with its drives and legs, the pack in the thorax, bare the
    spine's line, the cranks and the tail."""
    carbon, shell, frame = paint(CARBON), see == 'shell', see != 'bare'
    out = ([('carapace', _carapace()),
            ('canopy', shapes.ellipsoid((0.0, 0.07, 0.26), (0.075, 0.05, 0.14), CORE, rows=7))]
           if shell else
           [('spine', _at(shapes.drum(0.008 if frame else 0.003, 1.06, 'z', PLATE), None,
                          (0.0, 0.012, -0.13)))])
    if see == 'pack':
        out.append(('pack', shapes.box((0.0, MOTOR_Y, -0.005), (0.022, 0.018, 0.045), paint(CELLS))))
    out += _drive(0, FORE_Z, frame) + _drive(1, AFT_Z, frame)
    for s in (-1.0, 1.0):
        out.append(('sh_f%d' % (s > 0),
                    shapes.ellipsoid((s * SHOULDER_X, SHOULDER_Y, FORE_Z),
                                     (0.016, 0.012, 0.016), PLATE, rows=6)))
        out.append(('sh_a%d' % (s > 0),
                    shapes.ellipsoid((s * SHOULDER_X, SHOULDER_Y + 0.004, AFT_Z),
                                     (0.015, 0.011, 0.015), PLATE, rows=6)))
        out.append(('vtail%d' % (s > 0),
                    _at(shapes.ellipsoid((0.0, 0.0, 0.0), (0.10, 0.004, 0.035), paint(WING), rows=6),
                        shapes.turn_about('z', s * 34.0) @ shapes.turn_about('y', -s * 16.0),
                        (s * 0.055, 0.04, -0.62))))
        for k, z in enumerate(STRUT_Z):
            if not frame:
                continue
            out.append(('leg%d_%d' % (s > 0, k),
                        _at(shapes.drum(LEG_R, SKID_M - 0.032, 'y', carbon),
                            shapes.turn_about('z', s * 28.0) @ shapes.turn_about('x', (k - 0.5) * 30.0),
                            (s * 0.78 * SKID_X, -0.5 * (SKID_M + 0.032), z))))
            out.append(('claw%d_%d' % (s > 0, k),
                        shapes.ellipsoid((s * SKID_X, -SKID_M, z), (FOOT_M,) * 3, PLATE, rows=6)))
    return out


def _wing_blade(span, side):
    """One slender dragonfly wing extending along `side * +x` from its shoulder joint at the origin."""
    s = float(side)
    return [
        ('spar', _at(shapes.drum(0.007, 0.88 * span, 'x', paint(CARBON)), None,
                     (s * 0.44 * span, 0.003, 0.012))),
        ('blade', shapes.ellipsoid((s * 0.50 * span, 0.0, -0.004),
                                   (0.50 * span, 0.003, 0.5 * orni_phys.WING_CHORD), paint(WING), rows=7)),
    ]


def orni_mesh(which, see='shell'):
    """The cached `Mesh` for `'frame'`, `'fore_l'`, `'fore_r'`, `'aft_l'`, or `'aft_r'`."""
    got = _ORNI_MESH.get((which, see))
    if got is None:
        builders = {
            'frame': lambda: _orni_parts(see),
            'fore_l': lambda: _wing_blade(WING_SPAN, -1.0),
            'fore_r': lambda: _wing_blade(WING_SPAN, 1.0),
            'aft_l': lambda: _wing_blade(AFT_SPAN, -1.0),
            'aft_r': lambda: _wing_blade(AFT_SPAN, 1.0),
        }
        got = _ORNI_MESH[(which, see)] = Mesh(builders[which]())
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
    phase_f = ang_f / orni_phys.GEAR_RATIO
    phase_a = ang_a / orni_phys.GEAR_RATIO + math.pi
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


def _posed(pose, rotors, dense, see='shell'):
    """(positions, normals, uv, materials, index) in the world: the ornithopter carapace at `pose`
    and all four dragonfly wings articulated at their shoulder joints."""
    wings = (('fore_l', -1.0, (-SHOULDER_X, SHOULDER_Y, FORE_Z)),
             ('fore_r', 1.0, (SHOULDER_X, SHOULDER_Y, FORE_Z)),
             ('aft_l', -1.0, (-SHOULDER_X, SHOULDER_Y + 0.004, AFT_Z)),
             ('aft_r', 1.0, (SHOULDER_X, SHOULDER_Y + 0.004, AFT_Z)))
    angles = _wing_pose_angles(rotors, pose.get('controls'))
    moving = [(orni_mesh(kind),
               shapes.turn_about('y', -side * sweep)
               @ shapes.turn_about('z', side * dih)
               @ shapes.turn_about('x', feather),
               spot)
              for (kind, side, spot), (dih, feather, sweep) in zip(wings, angles)]
    return pose_meshes(pose, rotors, dense, orni_mesh('frame', see), moving)


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


#: Each wing pair's thrust from its shoulders, along the carapace's up.
ARROWS = (((0.0, SHOULDER_Y + 0.035, FORE_Z), 1, 0, 0.022),
          ((0.0, SHOULDER_Y + 0.035, AFT_Z), 1, 0, 0.022))


def render(pose, rotors, width, height, yaw=30.0, pitch=18.0, reach=REACH, centre=None,
           colour=True, lit=None, world=True, gate=None, thrusts=None, line_=False, inset=None,
           see='shell', lens=1.0):
    """The 4-wing Dune ornithopter at `pose` on its `rotors`, `width` x `height` cells, `reach` m
    framed about `centre` from SIGHT reaches off through a `lens` that much nearer: lines."""
    return render_craft(pose, rotors, width, height, yaw, pitch, reach, SIGHT, REACH, centre,
                        colour, lit, world, gate, thrusts, line_, inset, see, lens, _posed,
                        _stroke_arcs, lambda p, t, m, c, ctr: force_arrows(p, t, ARROWS, m, c, ctr))

