"""The 3D RC aerobatic monoplane drawn, an Extra 300 (`machine.aerobat.form`) on its hinges.

    lines = render(pose, rotors, 96, 40, lit=gpu.LitRaster())   # the GPU's lit raster
    lines = render(pose, rotors, 96, 40)                        # its dots splatted here

`pose` is `machine.aerobat.Sky.state()`'s, its 'turn', 'at' and 'controls'; `rotors` the front
and rear coaxial 63100 tractor drives' (angle rad, speed rad/s, can C, board C[, sink C]), each
can, board and heatsink painted their heat (`ansi.thermal_rgb`). A propeller slower than
`BLADES_RAD_S` shows its blades; faster, the swept prop disc ring, while the ailerons, elevator
and rudder deflect live with `controls` about their hinge lines.
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
from machine.aerobat import form

#: The camera: the eye's distance in reaches framed, and the reach the monoplane fills, m -
#: the wing tips 0.80 m out, the spinner's tip at +0.49 m, the rudder's foot at -1.00.
SIGHT, REACH = 3.75, 1.05

#: The airframe in its body axes (x the span, y the nose and thrust, z down), m: the form's unit,
#: the coaxial tractor hubs ahead of the cowl, the wheels where the sky stands it.
K = aero_phys.HALF_M / form.HALF
PROP_Y, VECT_Y, PROP_R = aero_phys.PROP_Z, aero_phys.PROP_Z - 0.045, aero_phys.PROP_R
HALF_SPAN, FOOT_M, SKID_X, STRUT_Z = aero_phys.HALF_M, aero_phys.FOOT_M, aero_phys.SKID_X, aero_phys.STRUT_Z
GEAR_MAIN_Z, GEAR_TAIL_Z = aero_phys.GEAR_MAIN_Z, aero_phys.GEAR_TAIL_Z
#: The two tractor motors, 42 x 40 mm each (292 W at the governed top, aircraft.md), stacked
#: in the cowl from MOTOR_Y aft, each with its 63100 board and sink behind its stator.
MOTOR_L, MOTOR_Y = 0.040, K * form.COWL_X - 0.004

#: The gear an Extra 300's: each main leg a carbon bow from the belly out and down to its wheel
#: in a spat, the tail wheel on a leaf from under the rudder; the main legs 6.7 cm longer than
#: the model's, the sky standing it 12 deg nose-up on them. Drawn as straight struts with a tail
#: wheel each side, it read as a quad's skids.
WHEEL_R, TAIL_WHEEL_R, BOW_N, BOW_R = K * form.WHEEL_R, K * form.TAIL_WHEEL_R, 8, 0.006

_AERO_MESH = {}


def _bow(s, carbon, spat):
    """A main leg, `s` its side: its bow in BOW_N pieces from the belly to its spat, its wheel,
    and its spat in `spat` (None: none)."""
    y, x = STRUT_Z[1], s * SKID_X
    z = GEAR_MAIN_Z + FOOT_M - WHEEL_R
    rx, ry, rz = (K * r for r in form.SPAT)
    along, up = (K * d for d in form.SPAT_AT)
    belly = K * (form.THRUST_Y - form.LEG[1])
    a, b, c = (s * K * form.LEG[2], belly), (x, belly + 0.01), (x, z - up - 0.5 * rz)
    pts = [tuple((1 - t) ** 2 * p + 2 * (1 - t) * t * q + t * t * r for p, q, r in zip(a, b, c))
           for t in (j / BOW_N for j in range(BOW_N + 1))]
    out = [('bow%d_%d' % (s > 0, j), shapes.strut((x0, y, z0), (x1, y, z1), BOW_R, carbon))
           for j, ((x0, z0), (x1, z1)) in enumerate(zip(pts, pts[1:]))]
    out.append(('wheel%d' % (s > 0), shapes.ellipsoid((x, y, z), (0.35 * WHEEL_R, WHEEL_R, WHEEL_R),
                                                       PLATE, rows=6)))
    if spat is not None:
        out.append(('spat%d' % (s > 0), shapes.ellipsoid((x, y + along, z - up), (rx, ry, rz), spat, rows=7)))
    return out


def _tail_wheel(carbon):
    """The tail wheel on its leaf from under the rudder's hinge."""
    y, z = STRUT_Z[0], GEAR_TAIL_Z + FOOT_M - TAIL_WHEEL_R
    root = form.body(form.RUDDER_X + 0.21, form.RUDDER_Y - 0.06, 0.0, K)
    return [('tail_leaf', shapes.strut(root, (0.0, y, z), BOW_R, carbon)),
            ('tail_wheel', shapes.ellipsoid((0.0, y, z), (0.4 * TAIL_WHEEL_R, TAIL_WHEEL_R, TAIL_WHEEL_R),
                                            PLATE, rows=6))]


#: The terminal's palette on an Extra 300: its shell the quad's skin, its cowl, fin, rudder and
#: spats the pages' amber, its lettering black - black keeps no dots (`lit.braille`).
WHITE, RED, INK = (188, 194, 204), (255, 176, 0), (0, 0, 0)
#: "COAXIAL" and "63100" in square wide capitals like Eurostile Bold Extended, PX_W a column
#: along the span and PX_H a row along the chord, from LETTER_X out, their top LETTER_Y a
#: centimetre behind the leading edge at their outer end, on top, read from above nose up.
PX_W, PX_H, LETTER_X = 0.0125, 0.0214, 0.11
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
_WIDE = max(len(w[0]) for w in _WORDS.values())
LETTER_Y = K * form.wing_edges((LETTER_X + _WIDE * PX_W) / K)[0] - 0.01


def _inked(x, y, side):
    """Whether the lettering inks the wing at `x`, `y` on `side` (-1 its top): "COAXIAL" out to
    in on +x, "63100" in to out on -x - "COAXIAL 63100" from above nose up; the other way
    round it read mirrored."""
    rows = int((LETTER_Y - y) // PX_H)
    text = _WORDS['COAXIAL' if x > 0.0 else '63100']
    n = len(text[0])
    col = int((abs(x) - LETTER_X) // PX_W)
    col = n - 1 - col if x > 0.0 else col
    return side < 0 and 0 <= rows < 7 and 0 <= col < n and text[rows][col] == '#'


def _wing_edges(s):
    """The wing's (leading, trailing) edge y at span `s`, m."""
    le, te = form.wing_edges(s / K)
    return K * le, K * te


def _hinge(s):
    """The ailerons' hinge y at span `s`, m."""
    le, te = _wing_edges(s)
    return te + form.AILERON * (le - te)


def _wing_half(s, y):
    le, te = _wing_edges(s)
    return (form.WING_T[0] - form.WING_T[1] * abs(s) / K) * (le - te) * shapes.naca((le - y) / max(le - te, 1e-6))


def _wing_place(s, y, t):
    return (s, y, K * (form.THRUST_Y - form.WING_MID[0]) - form.WING_MID[1] * abs(s) + t)


def _fixed_edges(s):
    le, te = _wing_edges(s)
    return (le, _hinge(s)) if AIL_IN <= abs(s) <= AIL_OUT else (le, te)


#: The ailerons' span, the rows the wing is cut along (its lettering's PX_H apart), its stations
#: (the lettering's PX_W apart where it is).
AIL_IN, AIL_OUT = K * form.AIL_Z, K * form.TIP_Z
_ROWS = [LETTER_Y + k * PX_H for k in (4, 2)] + [LETTER_Y - k * PX_H for k in range(8)] + [
    LETTER_Y - 7 * PX_H - d for d in (0.03, 0.07, 0.11, 0.15, 0.19)]


def _span(lo, hi, step):
    return [lo + (hi - lo) * k / max(1, round((hi - lo) / step)) for k in range(max(1, round((hi - lo) / step)) + 1)]


def _wing():
    """The wing ahead of the ailerons' hinge, through the fuselage tip to tip, lettered on top."""
    side = sorted(set(_span(0.0, LETTER_X, 0.05) + _span(LETTER_X, LETTER_X + _WIDE * PX_W, PX_W)
                      + _span(LETTER_X + _WIDE * PX_W, AIL_OUT, 0.05) + [AIL_IN - 1e-4, AIL_IN + 1e-4]
                      + [AIL_OUT - 1e-4, AIL_OUT + 1e-4] + _span(AIL_OUT + 1e-4, HALF_SPAN, 0.02)))
    stations = sorted(set([-s for s in side] + side))
    return shapes.surface(stations, _ROWS, _fixed_edges, _wing_half, _wing_place, paint(WHITE),
                    (_inked, paint(INK)))


def _aileron(s):
    """The aileron on side `s`, about its hinge's inner end."""
    from coaxial.model.blocks import numpy as np
    stations = [s * a for a in _span(AIL_IN, AIL_OUT, 0.06)]
    if s < 0:
        stations = stations[::-1]
    c, t, u, m = shapes.surface(stations, _ROWS, lambda x: (_hinge(x), _wing_edges(x)[1]), _wing_half, _wing_place,
                          paint(WHITE))
    return c - np.asarray(AIL_AT[s > 0]), t, u, m


#: Each aileron's hinge, inner end and axis (+x-ward), as `_posed` turns it.
AIL_AT = tuple(_wing_place(s * AIL_IN, _hinge(AIL_IN), 0.0) for s in (-1.0, 1.0))
_AIL_OUT_AT = tuple(_wing_place(s * AIL_OUT, _hinge(AIL_OUT), 0.0) for s in (-1.0, 1.0))
AIL_AXIS = tuple(tuple(s * (o - i) / math.dist(o_, i_) for o, i in zip(o_, i_))
                 for s, o_, i_ in zip((-1.0, 1.0), _AIL_OUT_AT, AIL_AT))


def _stab_half(s, y):
    le, te = (K * e for e in form.stab_edges(s / K))
    return form.STAB_T * (le - te) * shapes.naca((le - y) / (le - te))


def _stab_place(s, y, t):
    return (s, y, K * (form.THRUST_Y - form.STAB_Y) + t)


def _stab():
    """The stabiliser ahead of the elevator's hinge, tip to tip."""
    side = _span(0.0, K * form.STAB_Z, 0.04)
    rows = [K * (form.STAB_LE[0] + 0.1) - d for d in (0.0, 0.01, 0.025, 0.045, 0.07, 0.095, 0.12)]
    return shapes.surface(sorted(set([-s for s in side] + side)), rows,
                    lambda s: (K * form.stab_edges(s / K)[0], ELEV_AT[1]), _stab_half, _stab_place, paint(WHITE))


def _elevator_mesh():
    """The elevator's halves behind their hinge, about it."""
    from coaxial.model.blocks import numpy as np
    rows = [ELEV_AT[1] - d for d in (0.0, 0.02, 0.045, 0.07, 0.1, 0.13)]
    halves = [shapes.surface([s * a for a in _span(K * form.ELEV_Z, K * form.STAB_Z, 0.05)][::int(s)], rows,
                       lambda x: (ELEV_AT[1], K * form.stab_edges(x / K)[1]), _stab_half, _stab_place, paint(WHITE))
              for s in (-1.0, 1.0)]
    return [('elev%d' % k, (c - np.asarray(ELEV_AT), t, u, m)) for k, (c, t, u, m) in enumerate(halves)]


def _fin_half(h, y):
    up = form.THRUST_Y + h / K
    le, te = (K * e for e in form.fin_edges(up))
    thick = max(0.03, form.FIN_T[0] - form.FIN_T[1] * (up - form.FIN_Y))
    return thick * (le - te) * shapes.naca((le - y) / max(le - te, 1e-6))


def _fin_place(h, y, t):
    return (t, y, -h)


def _fin():
    """The fin ahead of the rudder's hinge, from inside the fuselage to its top."""
    heights = _span(K * (form.FIN_Y - form.THRUST_Y), K * (form.TOP_Y - form.THRUST_Y), 0.035)
    rows = [K * (form.FIN_LE[0] + 0.1) - d for d in (0.0, 0.015, 0.035, 0.06, 0.09, 0.12, 0.17)]
    return shapes.surface(heights, rows, lambda h: (K * form.fin_edges(form.THRUST_Y + h / K)[0], RUDDER_AT[1]),
                    _fin_half, _fin_place, paint(RED))


def _rudder_mesh():
    """The rudder behind its hinge from the fuselage's end to the top, about the hinge."""
    from coaxial.model.blocks import numpy as np
    heights = _span(K * (form.RUDDER_Y - form.THRUST_Y), K * (form.TOP_Y - form.THRUST_Y), 0.035)
    rows = [RUDDER_AT[1] - d for d in (0.0, 0.02, 0.05, 0.08, 0.11, 0.14)]
    c, t, u, m = shapes.surface(heights, rows, lambda h: (RUDDER_AT[1], K * form.fin_edges(form.THRUST_Y + h / K)[1]),
                          _fin_half, _fin_place, paint(RED))
    return [('rud', (c - np.asarray(RUDDER_AT), t, u, m))]


#: The elevator's and the rudder's hinges.
ELEV_AT = (0.0, K * form.ELEV_X, K * (form.THRUST_Y - form.STAB_Y))
RUDDER_AT = (0.0, K * form.RUDDER_X, 0.0)


def _fuselage():
    """The fuselage lofted through the form's sections, tail to cowl: its cowl amber, its canopy
    glass."""
    from coaxial.model.blocks import numpy as np
    rings = []
    for x, low, high, widths in form.SECTIONS[::-1]:
        hs = [low + (high - low) * j / (len(widths) - 1) for j in range(len(widths))]
        side = [(w, h) for w, h in zip(widths, hs)]
        ring = [(0.0, high)] + side[::-1] + [(0.0, low)] + [(-w, h) for w, h in side]
        rings.append([form.body(x, h, w, K) for w, h in ring])
    n = len(rings[0])
    ends = [form.body(form.RUDDER_X - 0.05, 0.5 * (form.SECTIONS[-1][1] + form.SECTIONS[-1][2]), 0.0, K),
            form.body(form.COWL_X, 0.5 * (form.SECTIONS[0][1] + form.SECTIONS[0][2]), 0.0, K)]
    corners = np.array([p for r in rings for p in r] + ends, float)
    m, j = len(rings), np.arange(n)
    a = (np.arange(m - 1)[:, None] * n + j).ravel()
    b = (np.arange(m - 1)[:, None] * n + (j + 1) % n).ravel()
    c, d = a + n, b + n
    tail, nose, last = m * n, m * n + 1, (m - 1) * n
    triangles = np.vstack([np.stack([a, c, b], 1), np.stack([c, d, b], 1),
                           np.stack([np.full(n, tail), j, (j + 1) % n], 1),
                           np.stack([np.full(n, nose), last + (j + 1) % n, last + j], 1)])
    x0, y0, slope, x1 = form.CANOPY
    x, up = corners[:, 1] / K, form.THRUST_Y - corners[:, 2] / K
    glass = (x <= x0) & (x >= x1) & (up > y0 + slope * (x - x0) + 0.01)
    mats = np.where(x > form.COWL_AFT, paint(RED), np.where(glass, CORE, paint(WHITE)))
    return corners, triangles, np.zeros((len(corners), 2)), mats


def _spinner():
    """The spinner from the cowl's face over both hubs to its tip."""
    r, y0, tip = K * 0.65, K * form.COWL_X, PROP_Y + 0.004 + 0.086
    rings = [(y0, 0.9 * r, 0.9 * r), (y0 + 0.004, r, r), (PROP_Y + 0.004, r, r)] + [
        (PROP_Y + 0.004 + 0.086 * f, r * math.sqrt(1.0 - f * f), r * math.sqrt(1.0 - f * f))
        for f in (0.3, 0.55, 0.75, 0.9)]
    return shapes.loft(rings, paint(WHITE), poles=(y0, tip))


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


#: The wing's spar at a quarter of its root chord, the stabiliser's, the rudder's post, the
#: tail's end.
SPAR_Y = _wing_edges(0.0)[0] - 0.25 * (_wing_edges(0.0)[0] - _wing_edges(0.0)[1])
WING_Z, TAIL_END = _wing_place(0.0, 0.0, 0.0)[2], K * form.RUDDER_X


def _aero_parts(see='shell'):
    """[(name, mesh)]: the aerobat airframe in its body axes; `see` a layer: the airframe its
    longerons, firewall, spars and ribs, the pack under the canopy, bare the shafts, the spars
    and the keel's line."""
    carbon, accent = paint(CARBON), paint(RED)
    shell, frame = see == 'shell', see != 'bare'
    stack, aft = _stack()
    stab_y = ELEV_AT[1] + 0.5 * (K * form.STAB_LE[0] - ELEV_AT[1])
    out = [('spinner', _spinner()),
           ('shaft0', _at(shapes.drum(SHAFT_R, PROP_Y + 0.02 - aft, 'y', PLATE), None,
                          (0.0, 0.5 * (PROP_Y + 0.02 + aft), 0.0))),
           ('spar', _at(shapes.drum(0.007, 2.0 * HALF_SPAN, 'x', PLATE), None, (0.0, SPAR_Y, WING_Z))),
           ('stabspar', _at(shapes.drum(0.005, 2.0 * K * form.STAB_Z, 'x', PLATE), None,
                            (0.0, stab_y, ELEV_AT[2]))),
           ('finpost', shapes.strut((0.0, TAIL_END, K * (form.THRUST_Y - form.TOP_Y)), (0.0, TAIL_END, 0.04), 0.005,
                              PLATE))]
    if shell:
        out += [('fuselage', _fuselage()), ('wing', _wing()), ('hstab', _stab()), ('vfin', _fin())]
    elif frame:
        out += [('longeron%d' % k, _at(shapes.drum(0.005, aft - TAIL_END, 'y', PLATE), None,
                                       (sx * 0.03, 0.5 * (aft + TAIL_END), sz * 0.035)))
                for k, (sx, sz) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1)))]
        out += [('firewall', _at(shapes.drum(0.046, 0.006, 'y', PLATE), None, (0.0, aft - 0.003, 0.0)))]
        for k, x in enumerate(HALF_SPAN * (j / 5.0) for j in range(-5, 6)):
            le, te = _wing_edges(x)
            out.append(('rib%d' % k, shapes.box((x, 0.5 * (le + te), _wing_place(x, 0.0, 0.0)[2]),
                                                (0.002, 0.5 * (le - te), 0.011), PLATE)))
        if see == 'pack':
            out.append(('pack', shapes.box((0.0, 0.0, 0.03), (0.022, 0.07, 0.02), paint(CELLS))))
    else:
        out.append(('keel', _at(shapes.drum(0.003, aft - TAIL_END, 'y', PLATE), None,
                                (0.0, 0.5 * (aft + TAIL_END), 0.0))))
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


def aero_mesh(which, see='shell'):
    """The cached `Mesh` for `'frame'`, `'prop'`, `'vect'`, `'ail_l'`, `'ail_r'`, `'elev'`, `'rud'`."""
    got = _AERO_MESH.get((which, see))
    if got is None:
        builders = {
            'frame': lambda: _aero_parts(see),
            'prop': lambda: _prop_blades(PROP_R),
            'vect': lambda: _prop_blades(0.88 * PROP_R),
            'ail_l': lambda: [('ail', _aileron(-1.0))],
            'ail_r': lambda: [('ail', _aileron(1.0))],
            'elev': _elevator_mesh,
            'rud': _rudder_mesh,
        }
        got = _AERO_MESH[(which, see)] = Mesh(builders[which]())
    return got


def _posed(pose, rotors, dense, see='shell'):
    """(positions, normals, uv, materials, index) in the world: the aerobat airframe at its true
    physical `pose['turn']`, its slow coaxial tractor propellers, and its surfaces turned about
    their hinges."""
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
        ('ail_l', shapes.turn_about(AIL_AXIS[0], -32.0 * float(ail)), AIL_AT[0]),
        ('ail_r', shapes.turn_about(AIL_AXIS[1], 32.0 * float(ail)), AIL_AT[1]),
        ('elev', shapes.turn_about('x', 36.0 * float(elev)), ELEV_AT),
        ('rud', shapes.turn_about('z', 36.0 * float(rud)), RUDDER_AT),
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


#: The propeller's thrust from the spinner along the nose, the rudder's from its middle across.
ARROWS = (((0.0, PROP_Y + 0.05, 0.0), 1, 0, 0.014), ((0.0, TAIL_END - 0.06, -0.10), 0, 2, 0.08))


def render(pose, rotors, width, height, yaw=30.0, pitch=18.0, reach=REACH, centre=None,
           colour=True, lit=None, world=True, gate=None, thrusts=None, line_=False, inset=None,
           see='shell', lens=1.0):
    """The 3D aerobatic monoplane at `pose` on its `rotors`, `width` x `height` cells, `reach` m
    framed about `centre` from SIGHT reaches off through a `lens` that much nearer: lines."""
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
                   props=own + scenery.props(m, fine, centre, gate, line_, airfield))
