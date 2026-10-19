"""The 3D helicopter drawn: its canopy, boom, skids, 63100 drives and rotors, in braille.

    lines = render(pose, rotors, 96, 40, lit=gpu.LitRaster())   # the GPU's lit raster
    lines = render(pose, rotors, 96, 40)                        # its dots splatted here

`pose` is `machine.helicopter.Sky.state()`'s, its 'turn' and 'at'; `rotors` the main and tail
drives' (angle rad, speed rad/s, can C, board C[, sink C]), each can, board and heatsink painted
their heat (`ansi.thermal_rgb`). A rotor slower than BLADES_RAD_S shows its blades; faster,
the disc they sweep, its rim dotted where the frame does not cover it. The collective's arrow
points up the mast on positive pitch and down it when inverted on negative pitch.
"""
import math

from coaxial.graphics import engine, scenery, shapes
from coaxial.graphics.callouts import line
from coaxial.graphics.lit import PLATE, braille, paint, project, splat
from coaxial.graphics.quadcopter import (ARROW_HEAD, ARROW_INK, ARROW_M_PER_N, BLADE_W,
                                         BLADES_RAD_S, BOARD_H, BOARD_R, CARBON, CELLS, LEG_R,
                                         MASK, MOTOR_R, PROP, RIM_EVERY, RIM_INK, SHAFT_R,
                                         SINK_H, SINK_R, SKIN, Mesh, _at, _painted)
from machine import helicopter as heli_phys

#: The camera: the eye's distance in reaches framed, and the reach the helicopter fills, m -
#: the main rotor's tips 0.50 m out, the tail rotor at -0.68 m.
SIGHT, REACH = 3.75, 0.68

#: The airframe's geometry, m: the main rotor disc's height on the mast and its radius, the
#: swashplate on the mast, the skids below the frame, the tail boom's reach aft and the tail
#: rotor's hub and radius.
DISC_M, ROTOR_R, SWASH_M = heli_phys.DISC_M, heli_phys.ROTOR_R, 0.115
SKID_M, FOOT_M, SKID_X, STRUT_Z = (
    heli_phys.SKID_M, heli_phys.FOOT_M, heli_phys.SKID_X, heli_phys.STRUT_Z)
TAIL_M, TAIL_R = heli_phys.TAIL_M, heli_phys.TAIL_R
TAIL_AT = (0.032, 0.022, -TAIL_M)
#: The main motor, 42 x 48 mm (271 W at the governed top, aircraft.md), up from the frame's
#: floor; the tail's, 28 x 20 mm; the 63100 boards on the frames' faces, 64 mm apart.
MOTOR_L, MOTOR_Y, TAIL_MOTOR = 0.048, 0.055, (0.014, 0.020)
NEG_ARROW_INK = (96, 224, 255)
#: A T-REX 700's red canopy, fin and blade tips.
RED = (186, 28, 36)

_HELI_MESH = {}


def _boom():
    """The carbon tail boom from the frame's back (-0.08 m) to the tail gearbox (-TAIL_M)."""
    rings = [(-TAIL_M + (i / 10.0) * (-0.08 + TAIL_M), 0.013 + (i / 10.0) * 0.004,
              0.013 + (i / 10.0) * 0.004, 0.0, 0.0) for i in range(11)]
    return shapes.loft(rings, paint(CARBON), poles=(-TAIL_M - 0.008, -0.075), along='z')


def _side(name, x, mesh_r, h, material):
    """A disc `mesh_r` round, `h` thick, standing on the frame's face at `x`, its axis across."""
    return (name, _at(shapes.drum(mesh_r, h, 'x', material), None, (x, 0.04, 0.06)))


def _canopy():
    """The canopy along z over the frames, a T-REX's teardrop from the nose into the boom: red
    above the frames' floor, black under."""
    from coaxial.model.blocks import numpy as np
    rings = [(-0.26, 0.014, 0.018, -0.015), (-0.22, 0.026, 0.036, -0.022), (-0.16, 0.046, 0.062, -0.030),
             (-0.10, 0.062, 0.080, -0.035), (-0.02, 0.070, 0.090, -0.035), (0.08, 0.068, 0.088, -0.035),
             (0.18, 0.055, 0.072, -0.035), (0.25, 0.035, 0.045, -0.035), (0.30, 0.012, 0.014, -0.030)]
    red, carbon = paint(RED), paint(CARBON)
    return shapes.loft(rings, lambda c: np.where(c[:, 1] > 0.005, red, carbon),
                       poles=(-0.27, 0.31), along='z')


def _tipped(s):
    """A blade's paint on side `s`: red past 0.8 of its reach, white within."""
    from coaxial.model.blocks import numpy as np
    red, prop = paint(RED), paint(PROP)
    return lambda c: np.where(s * c[:, 0] > 0.3 * ROTOR_R, red, prop)


def heli_parts(see='shell'):
    """[(name, mesh)]: the helicopter in its own axes, x right, y up, z ahead; `see` a layer:
    the airframe its two frames, tray and boom, the pack the tray's cells, bare the mast, the
    swashplate, the tail shaft and the boom's line."""
    carbon, skin = paint(CARBON), paint(SKIN)
    shell, frame = see == 'shell', see != 'bare'
    out = [
        ('mast', _at(shapes.drum(0.010, DISC_M - MOTOR_Y - MOTOR_L, 'y', PLATE), None,
                     (0.0, 0.5 * (DISC_M + MOTOR_Y + MOTOR_L), -0.015))),
        ('swash', _at(shapes.drum(0.034, 0.010, 'y', PLATE), None, (0.0, SWASH_M, -0.015))),
        ('hub0', shapes.ellipsoid((0.0, DISC_M + 0.004, -0.015), (0.03, 0.012, 0.03), PLATE,
                                  rows=6)),
        ('shaft1', _at(shapes.drum(SHAFT_R, 0.07, 'x', PLATE), None,
                       (0.0, TAIL_AT[1], TAIL_AT[2]))),
        ('hub1', shapes.ellipsoid(TAIL_AT, (0.012, 0.012, 0.012), PLATE, rows=6)),
    ]
    if not frame:
        return out + [('boom', _at(shapes.drum(0.003, TAIL_M - 0.08, 'z', PLATE), None,
                                   (0.0, 0.012, -0.5 * (TAIL_M + 0.08))))]
    out += [
        ('frame0', shapes.box((-0.032, 0.025, 0.03), (0.004, 0.06, 0.14), skin)),
        ('frame1', shapes.box((0.032, 0.025, 0.03), (0.004, 0.06, 0.14), skin)),
        ('tray', shapes.box((0.0, -0.025, 0.11), (0.034, 0.018, 0.09),
                            paint(CELLS) if see == 'pack' else PLATE)),
        ('can0', _at(shapes.drum(MOTOR_R, MOTOR_L, 'y', PLATE), None,
                     (0.0, MOTOR_Y + MOTOR_L / 2.0, -0.015))),
        _side('board0', -0.036 - BOARD_H / 2.0, BOARD_R, BOARD_H, paint(MASK)),
        _side('sink0', -0.036 - BOARD_H - SINK_H / 2.0, SINK_R, SINK_H, carbon),
        ('boom', _boom()),
        ('can1', _at(shapes.drum(*TAIL_MOTOR, 'x', PLATE), None,
                     (0.012, TAIL_AT[1], TAIL_AT[2]))),
        _side('board1', 0.036 + BOARD_H / 2.0, BOARD_R, BOARD_H, paint(MASK)),
        _side('sink1', 0.036 + BOARD_H + SINK_H / 2.0, SINK_R, SINK_H, carbon),
    ]
    if shell:
        out += [('canopy', _canopy()),
                ('vfin', _at(shapes.box((0.0, 0.0, 0.0), (0.004, 0.065, 0.04), paint(RED)),
                             shapes.turn_about('x', 25.0), (0.0, 0.035, -TAIL_M + 0.03))),
                ('hstab', shapes.box((0.0, 0.03, -0.46), (0.10, 0.004, 0.03), carbon))]
    for s in (-1.0, 1.0):
        out.append(('skid%d' % (s > 0),
                    _at(shapes.drum(1.4 * LEG_R, 0.46, 'z', skin), None,
                        (s * SKID_X, -SKID_M, 0.03))))
        for k, z in enumerate(STRUT_Z):
            out.append(('strut%d_%d' % (s > 0, k),
                        _at(shapes.drum(LEG_R, SKID_M - 0.025, 'y', carbon),
                            shapes.turn_about('z', s * 18.0),
                            (s * 0.78 * SKID_X, -0.5 * (SKID_M + 0.025), z))))
    return out


def main_blades():
    """The main rotor's two blades about the mast (y axis), shown when slow (`abs(w) < BLADES_RAD_S`)."""
    return [('blade', shapes.ellipsoid((s * ROTOR_R / 2.0, 0.0, 0.0),
                                       (ROTOR_R / 2.0, 0.004, BLADE_W), _tipped(s), rows=8))
            for s in (-1.0, 1.0)]


def tail_blades():
    """The tail rotor's two blades in the y-z plane about its x axis, shown when slow."""
    prop = paint(PROP)
    return [('blade', shapes.ellipsoid((0.0, 0.0, s * TAIL_R / 2.0),
                                       (0.004, 0.014, TAIL_R / 2.0), prop, rows=6))
            for s in (-1.0, 1.0)]


def heli_mesh(which, see='shell'):
    """The cached `Mesh` for `'frame'`, `'main'` or `'tail'`."""
    got = _HELI_MESH.get((which, see))
    if got is None:
        builders = {'frame': lambda: heli_parts(see), 'main': main_blades, 'tail': tail_blades}
        got = _HELI_MESH[(which, see)] = Mesh(builders[which]())
    return got


def _posed(pose, rotors, dense, see='shell'):
    """(positions, normals, uv, materials, index) in the world: the helicopter frame at `pose`,
    and each slow rotor's blades at its angle over its hub."""
    from coaxial.model.blocks import numpy as np
    turn, at = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    frame = heli_mesh('frame', see)
    pick = (lambda mesh: (mesh.dense[0], mesh.dense[1], mesh.dense[2], mesh.dense[3])) if dense \
        else (lambda mesh: (mesh.corners, mesh.normals, mesh.uv, mesh.materials))
    points, normals, uv, _materials = pick(frame)
    out = [[points @ turn.T + at], [normals @ turn.T], [uv], [_painted(frame, rotors, dense)]]
    index, base = [frame.index], len(points)
    specs = (('main', 'y', (0.0, DISC_M + 0.006, -0.015)),
             ('tail', 'x', TAIL_AT))
    for (kind, axis, spot), rotor in zip(specs, rotors):
        angle, w = rotor[0], rotor[1]
        if abs(w) >= BLADES_RAD_S and see != 'bare':
            continue
        prop = heli_mesh(kind)
        here = turn @ shapes.turn_about(axis, math.degrees(angle))
        p, n, u, mat = pick(prop)
        out[0].append(p @ here.T + at + turn @ np.array(spot))
        out[1].append(n @ here.T)
        out[2].append(u)
        out[3].append(mat)
        index.append(prop.index + base)
        base += len(p)
    return tuple(np.concatenate(o) for o in out) + (np.vstack(index),)


def _discs(pose, rotors, m, cam, centre):
    """[(dots, ink)]: when turning faster than BLADES_RAD_S, the swept main rotor tip-path ring
    and vertical tail rotor ring, their rims dotted."""
    from coaxial.model.blocks import numpy as np
    turn, at = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    rim = np.zeros((cam['height'], cam['width']), bool)
    k = np.arange(96) * (2.0 * math.pi / 96)
    rings = ((np.stack([np.cos(k), np.zeros_like(k), np.sin(k)], 1) * ROTOR_R,
              (0.0, DISC_M + 0.006, -0.015)),
             (np.stack([np.zeros_like(k), np.cos(k), np.sin(k)], 1) * TAIL_R, TAIL_AT))
    for (ring, spot), (_angle, w, *_heat) in zip(rings, rotors):
        if abs(w) < BLADES_RAD_S:
            continue
        hub = at + turn @ np.array(spot)
        sx, sy, _w = project(ring @ turn.T + hub, m, cam, centre)
        drawn = np.zeros_like(rim)
        for a in range(len(k)):
            line(drawn, (sx[a], sy[a]), (sx[a - 1], sy[a - 1]))
        ys, xs = np.nonzero(drawn)
        keep = ((xs + ys) % RIM_EVERY) == 0
        rim[ys[keep], xs[keep]] = True
    return [(rim, RIM_INK)]


def force_arrows(pose, thrusts, specs, m, cam, centre):
    """[(dots, ink)]: each of `thrusts`, N, an arrow of the force from its hub - `specs` a
    (hub in the frame's axes, the column of `turn` it acts along, the column across it, m a
    newton) each - `NEG_ARROW_INK` where it pulls against its axis: an inverted hover's
    collective points at the sky; drawn over whatever covers it."""
    from coaxial.model.blocks import numpy as np
    turn, at = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    pos_drawn = np.zeros((cam['height'], cam['width']), bool)
    neg_drawn = np.zeros((cam['height'], cam['width']), bool)
    for (spot, along, across, scale), thrust in zip(specs, thrusts):
        length = scale * abs(thrust)
        if length <= 1e-4:
            continue
        way = turn[:, along] * (1.0 if thrust >= 0.0 else -1.0)
        target = pos_drawn if thrust >= 0.0 else neg_drawn
        hub = at + turn @ np.array(spot)
        tip = hub + length * way
        back = tip - ARROW_HEAD * length * way
        points = np.array([hub, tip, back + 0.5 * ARROW_HEAD * length * turn[:, across],
                           back - 0.5 * ARROW_HEAD * length * turn[:, across]])
        sx, sy, _w = project(points, m, cam, centre)
        for a, b in ((0, 1), (1, 2), (1, 3)):
            line(target, (sx[a], sy[a]), (sx[b], sy[b]))
    out = [(pos_drawn, ARROW_INK, True)]
    if neg_drawn.any():
        out.append((neg_drawn, NEG_ARROW_INK, True))
    return out


#: The collective's arrow from the mast's hub along it, the tail's from its hub across.
ARROWS = (((0.0, DISC_M + 0.006, -0.015), 1, 0, 0.014), (TAIL_AT, 0, 2, ARROW_M_PER_N))



def render(pose, rotors, width, height, yaw=30.0, pitch=18.0, reach=REACH, centre=None,
           colour=True, lit=None, world=True, gate=None, thrusts=None, line_=False, inset=None,
           see='shell', lens=1.0):
    """The 3D helicopter at `pose` on its `rotors`, `width` x `height` cells, `reach` m framed
    about `centre` from SIGHT reaches off through a `lens` that much nearer: lines."""
    from coaxial.model.blocks import numpy as np
    m = shapes.view(yaw, pitch)
    fine = engine.fine(engine.camera(width, height, reach, distance=SIGHT * reach, zoom=lens))
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
