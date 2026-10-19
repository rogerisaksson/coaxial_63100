"""The 3D helicopter drawn in braille, a T-REX 700's form (`machine.helicopter.form`).

Its canopy, frames, boom, skids, 63100 drives and rotors.

    lines = render(pose, rotors, 96, 40, lit=gpu.LitRaster())   # the GPU's lit raster
    lines = render(pose, rotors, 96, 40)                        # its dots splatted here

`pose` is `machine.helicopter.Sky.state()`'s, its 'turn' and 'at'; `rotors` the main and tail
drives' (angle rad, speed rad/s, can C, board C[, sink C]), each can, board and heatsink painted
their heat (`ansi.thermal_rgb`). A rotor slower than BLADES_RAD_S shows its blades; faster,
the disc they sweep, its rim dotted where the frame does not cover it. The collective's arrow
points up the mast on positive pitch and down it when inverted on negative pitch.
"""
import math

from coaxial.graphics import shapes
from coaxial.graphics.callouts import line
from coaxial.graphics.lit import CORE, PLATE, paint, project
from coaxial.graphics.quadcopter import (ARROW_HEAD, ARROW_INK, ARROW_M_PER_N, BLADES_RAD_S,
                                         BOARD_H, BOARD_R, CARBON, CELLS, MASK, PROP, SHAFT_R,
                                         SINK_H, SINK_R, SKIN, Mesh, _at, drive, pose_meshes,
                                         render_craft, swept_rings)
from machine import helicopter as heli_phys
from machine.helicopter import form

#: The camera: the eye's distance in reaches framed, and the reach the helicopter fills, m -
#: the main rotor's tips 0.625 m out, the tail rotor at -0.85 m.
SIGHT, REACH = 3.75, 0.85

#: What the sky flies and stands on, m: the main rotor's disc on the mast and its radius, the
#: tail rotor's arm and radius, the skids' feet under the frame.
DISC_M, ROTOR_R, TAIL_M, TAIL_R = heli_phys.DISC_M, heli_phys.ROTOR_R, heli_phys.TAIL_M, heli_phys.TAIL_R
SKID_M, FOOT_M, SKID_X, STRUT_Z = heli_phys.SKID_M, heli_phys.FOOT_M, heli_phys.SKID_X, heli_phys.STRUT_Z
#: The mast just aft of the centre of gravity; the form's unit, its tail rotor at the boom's arm
#: from the mast; the ground under the feet; the skids' tubes' radius, m.
MAST_Z = -0.015
K, GROUND, TUBE_R = (TAIL_M + MAST_Z) / form.TAIL[2], SKID_M + FOOT_M, 0.006
#: The main motor, 42 x 48 mm (271 W at the governed top, aircraft.md), under the mast from
#: MOTOR_Y, its 63100 board and sink under it; the tail's, 28 x 20 mm, out from the boom's side
#: on its board and sink.
MOTOR_L, MOTOR_Y, TAIL_MOTOR = 0.048, 0.055, (0.014, 0.020)
NEG_ARROW_INK = (96, 224, 255)
#: A T-REX 700's red canopy, fin and blade tips.
RED = (186, 28, 36)

_HELI_MESH = {}


def at(x, y, z):
    """A point of the form, its axes', in the helicopter's: x right, y up, z ahead, m."""
    return (-K * x, K * y - GROUND, MAST_Z - K * z)


TAIL_AT, HUB_AT = at(*form.TAIL), (0.0, DISC_M + 0.006, MAST_Z)


def _canopy():
    """The canopy lofted through the form's sections, back to nose: red, its window glass."""
    from coaxial.model.blocks import numpy as np
    rings = [(MAST_Z - K * z, K * w, 0.5 * K * (top - bottom), GROUND - 0.5 * K * (top + bottom))
             for z, bottom, top, w in form.CANOPY[-2:0:-1]]
    z0, y0, slope = form.WINDOW

    def coat(c):
        z, y = (MAST_Z - c[:, 2]) / K, (c[:, 1] + GROUND) / K
        return np.where((z < z0) & (y > y0 + slope * (z - z0)), CORE, paint(RED))
    return shapes.loft(rings, coat, poles=(at(0, 0, form.CANOPY[-1][0])[2], at(0, 0, form.CANOPY[0][0])[2]),
                       along='z')


def _plate(x):
    """A frame's side plate at `x` through the form's outline."""
    def edges(s):
        z = (MAST_Z - s) / K
        return K * shapes.over(form.FRAME, z, 2) - GROUND, K * shapes.over(form.FRAME, z, 1) - GROUND
    return shapes.surface([MAST_Z - K * r[0] for r in form.FRAME],
                          [K * y - GROUND for y in (0.36, 0.30, 0.25, 0.20, 0.15, 0.11, 0.075)],
                          edges, lambda s, y: 0.5 * K * form.FRAME_T, lambda s, y, t: (x + t, y, s),
                          paint(CARBON))


def _stab():
    """The stabiliser across the boom, swept and tapered to its tips."""
    (x0, le0, te0), (x1, le1, te1) = form.STAB

    def edges(s):
        f = max(0.0, abs(s) / K - x0) / (x1 - x0)
        return MAST_Z - K * (le0 + f * (le1 - le0)), MAST_Z - K * (te0 + f * (te1 - te0))
    side = [K * (x0 + (x1 - x0) * j / 4.0) for j in range(5)]
    rows = [MAST_Z - K * (le0 + d) for d in (0.0, 0.012, 0.025, 0.04, 0.06)]
    return shapes.surface([-s for s in side[::-1]] + side, rows, edges, shapes.foil(edges, 0.08),
                          lambda s, c, t: (s, K * form.STAB_Y - GROUND + t, c), paint(CARBON))


def _fin():
    """The fin under the boom, swept back down to its tip."""
    rows_up = form.FIN[::-1]

    def edges(s):
        y = (s + GROUND) / K
        return MAST_Z - K * shapes.over(rows_up, y, 1), MAST_Z - K * shapes.over(rows_up, y, 2)
    heights = [K * r[0] - GROUND for r in rows_up]
    rows = [MAST_Z - K * (0.93 + d) for d in (0.0, 0.015, 0.035, 0.06, 0.085, 0.109)]
    return shapes.surface(heights, rows, edges, shapes.foil(edges, form.FIN_T),
                          lambda s, c, t: (-K * form.FIN_X + t, s, c), paint(RED))


def _gear(skin):
    """The skids from the rear feet to their turned-up noses, and the two bows across under the
    frame, each from skid to skid."""
    y = TUBE_R - GROUND
    out = [('skid%d' % (s > 0), shapes.tube([(s * SKID_X, y, STRUT_Z[0] - FOOT_M),
                                             (s * SKID_X, y, at(0, 0, form.SKID[3])[2]),
                                             (s * SKID_X,) + at(0, *form.NOSE_UP)[1:]], TUBE_R, skin))
           for s in (-1.0, 1.0)]
    top_x, top_y = K * form.BOW_TOP[0], K * form.BOW_TOP[1] - GROUND
    for k, zb in enumerate(form.BOWS):
        z = at(0, 0, zb)[2]
        half = shapes.bowed((top_x, top_y, z), (0.8 * SKID_X + 0.2 * top_x, top_y, z), (SKID_X, y, z))
        out.append(('bow%d' % k, shapes.tube([(-p[0], p[1], p[2]) for p in half[::-1]] + half,
                                             0.8 * TUBE_R, skin)))
    return out


def _tail_drive():
    """The tail's stack out from the boom's side to its rotor: the sink, the 63100 board, the
    can."""
    x, out = -K * form.BOOM[3], []
    for h, r, name, material in ((SINK_H, SINK_R, 'sink1', paint(CARBON)),
                                 (BOARD_H, BOARD_R, 'board1', paint(MASK)),
                                 (TAIL_MOTOR[1], TAIL_MOTOR[0], 'can1', PLATE)):
        out.append((name, _at(shapes.drum(r, h, 'x', material), None, (x - h / 2.0, TAIL_AT[1], TAIL_AT[2]))))
        x -= h
    return out


def heli_parts(see='shell'):
    """[(name, mesh)]: the helicopter in its own axes, x right, y up, z ahead; `see` a layer:
    the airframe its frames, tray, boom and gear, the pack the tray's cells, bare the mast, the
    swashplate, the tail shaft and the boom's line."""
    carbon, skin = paint(CARBON), paint(SKIN)
    z0, _z1, boom_y, boom_r = form.BOOM
    out = [
        ('mast', _at(shapes.drum(0.006, DISC_M - MOTOR_Y - MOTOR_L, 'y', PLATE), None,
                     (0.0, 0.5 * (DISC_M + MOTOR_Y + MOTOR_L), MAST_Z))),
        ('swash', _at(shapes.drum(K * form.SWASH[1], 0.008, 'y', PLATE), None,
                      (0.0, K * form.SWASH[0] - GROUND, MAST_Z))),
        ('hub0', _at(shapes.drum(K * form.HUB[1], 0.014, 'y', PLATE), None, HUB_AT)),
        ('shaft1', _at(shapes.drum(SHAFT_R, -TAIL_AT[0], 'x', PLATE), None,
                       (0.5 * TAIL_AT[0], TAIL_AT[1], TAIL_AT[2]))),
        ('hub1', shapes.ellipsoid(TAIL_AT, (0.006, 0.009, 0.009), PLATE, rows=6)),
    ]
    a, b = at(0, boom_y, z0), at(0, boom_y, form.CASE[0])
    if see == 'bare':
        return out + [('boom', _at(shapes.drum(0.003, a[2] - b[2], 'z', PLATE), None,
                                   (0.0, a[1], 0.5 * (a[2] + b[2]))))]
    (fx, fy, fz), (bx, by, bz) = form.SUPPORT
    tz0, tz1, ty0, ty1, tw = form.TRAY
    cz0, cz1, cy0, cy1 = form.CASE
    out += [
        ('frame0', _plate(-K * form.HALF_FRAME)),
        ('frame1', _plate(K * form.HALF_FRAME)),
        ('tray', shapes.box(at(0, 0.5 * (ty0 + ty1), 0.5 * (tz0 + tz1)),
                            (K * tw, 0.5 * K * (ty1 - ty0), 0.5 * K * (tz1 - tz0)),
                            paint(CELLS) if see == 'pack' else PLATE)),
        ('boom', _at(shapes.drum(K * boom_r, a[2] - b[2], 'z', carbon), None,
                     (0.0, a[1], 0.5 * (a[2] + b[2])))),
        ('case', shapes.box(at(0, 0.5 * (cy0 + cy1), 0.5 * (cz0 + cz1)),
                            (K * boom_r, 0.5 * K * (cy1 - cy0), 0.5 * K * (cz1 - cz0)), carbon)),
        ('hstab', _stab()),
        ('vfin', _fin()),
    ] + [('support%d' % (s > 0), shapes.strut(at(s * fx, fy, fz), at(s * bx, by, bz), 0.003, carbon))
         for s in (-1.0, 1.0)]
    out += _gear(skin) + [(name + '0', mesh) for name, mesh in
                          drive(0.0, MOTOR_Y - BOARD_H - SINK_H, MAST_Z, MOTOR_L, carbon)] + _tail_drive()
    if see == 'shell':
        out.append(('canopy', _canopy()))
    return out


def _blade(r0, r1, chord, plane, ink=None):
    """A blade from `r0` out to `r1`, `chord` wide, NACA 0012, laid by `plane(r, c, t)`: its
    tip's last 4 % rounded back."""
    def edges(s):
        cut = max(0.0, (s - 0.96 * r1) / (0.04 * r1))
        return 0.5 * chord * (1.0 - 0.7 * cut * cut), -0.5 * chord
    stations = [r0 + (r1 - r0) * j / 12.0 for j in range(12)] + [0.98 * r1, r1]
    rows = [0.5 * chord - chord * j / 5.0 for j in range(6)]
    return shapes.surface(stations, rows, edges, shapes.foil(edges, 0.12), plane, paint(PROP), ink)


def main_blades():
    """The main rotor's two blades and their grips about the mast (y axis), shown when slow
    (`abs(w) < BLADES_RAD_S`): red past 0.85 of the radius."""
    g0, g1, gr = (K * g for g in form.GRIPS)
    root, chord, _thick = form.BLADE
    tip = (lambda r, c, side: r > 0.85 * ROTOR_R, paint(RED))
    out = []
    for s in (-1.0, 1.0):
        out += [('grip', _at(shapes.drum(gr, g1 - g0, 'x', PLATE), None, (s * 0.5 * (g0 + g1), 0.0, 0.0))),
                ('blade', _blade(K * root, ROTOR_R, K * chord, lambda r, c, t, s=s: (s * r, t, s * c), tip))]
    return out


def tail_blades():
    """The tail rotor's two blades in the y-z plane about its x axis, shown when slow."""
    return [('blade', _blade(0.01, TAIL_R, K * form.TAIL_CHORD, lambda r, c, t, s=s: (t, s * r, s * c)))
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
    specs = (('main', 'y', HUB_AT), ('tail', 'x', TAIL_AT))
    moving = [(heli_mesh(kind), shapes.turn_about(axis, math.degrees(rotor[0])), spot)
              for (kind, axis, spot), rotor in zip(specs, rotors)
              if abs(rotor[1]) < BLADES_RAD_S or see == 'bare']
    return pose_meshes(pose, rotors, dense, heli_mesh('frame', see), moving)


_RINGS = []


def _discs(pose, rotors, m, cam, centre):
    """[(dots, ink)]: when turning faster than BLADES_RAD_S, the swept main rotor tip-path ring
    and vertical tail rotor ring, their rims dotted; the rings laid once."""
    if not _RINGS:
        from coaxial.model.blocks import numpy as np
        k = np.arange(96) * (2.0 * math.pi / 96)
        _RINGS.extend(((np.stack([np.cos(k), np.zeros_like(k), np.sin(k)], 1) * ROTOR_R, HUB_AT),
                       (np.stack([np.zeros_like(k), np.cos(k), np.sin(k)], 1) * TAIL_R, TAIL_AT)))
    return swept_rings(pose, zip(_RINGS, rotors), m, cam, centre)


def force_arrows(pose, thrusts, specs, m, cam, centre):
    """[(dots, ink)]: each of `thrusts`, N, an arrow of the force from its hub - `specs` a
    (hub in the frame's axes, the column of `turn` it acts along, the column across it, m a
    newton) each - `NEG_ARROW_INK` where it pulls against its axis: an inverted hover's
    collective points at the sky; drawn over whatever covers it."""
    from coaxial.model.blocks import numpy as np
    turn, at_ = np.asarray(pose['turn'], float), np.asarray(pose['at'], float)
    pos_drawn = np.zeros((cam['height'], cam['width']), bool)
    neg_drawn = np.zeros((cam['height'], cam['width']), bool)
    for (spot, along, across, scale), thrust in zip(specs, thrusts):
        length = scale * abs(thrust)
        if length <= 1e-4:
            continue
        way = turn[:, along] * (1.0 if thrust >= 0.0 else -1.0)
        target = pos_drawn if thrust >= 0.0 else neg_drawn
        hub = at_ + turn @ np.array(spot)
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
ARROWS = ((HUB_AT, 1, 0, 0.014), (TAIL_AT, 0, 2, ARROW_M_PER_N))


def render(pose, rotors, width, height, yaw=30.0, pitch=18.0, reach=REACH, centre=None,
           colour=True, lit=None, world=True, gate=None, thrusts=None, line_=False, inset=None,
           see='shell', lens=1.0, field=None):
    """The 3D helicopter at `pose` on its `rotors`, `width` x `height` cells, `reach` m framed
    about `centre` from SIGHT reaches off through a `lens` that much nearer: lines."""
    return render_craft(pose, rotors, width, height, yaw, pitch, reach, SIGHT, REACH, centre,
                        colour, lit, world, gate, thrusts, line_, inset, see, lens, _posed, _discs,
                        lambda p, t, m, c, ctr: force_arrows(p, t, ARROWS, m, c, ctr), field)
