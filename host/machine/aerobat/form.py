"""The aerobat's form, an Extra 300's, measured off a model 21.9 units across.

The model's axes x its nose, y up, z its span; `body` lays a point in the body's (x the span, y the
nose, z down) at the wing's span. What the page draws and what touches the runway (`airframe.mjcf`).
"""

#: The model's half span and its thrust line's height, units; the centre of gravity at x 0, 27 %
#: of the wing's mean chord.
HALF, THRUST_Y = 10.95, 4.07

#: The fuselage, nose to tail: (x, bottom, top, half-widths at nine heights bottom to top), the
#: wing's and the stabiliser's roots bridged; its cowl's face at COWL_X, the cowl back to COWL_AFT,
#: the canopy above CANOPY's line (x, y, slope) back to its end.
SECTIONS = (
    (4.72, 3.34, 4.48, (0.06, 0.39, 0.48, 0.53, 0.56, 0.53, 0.47, 0.38, 0.13)),
    (4.55, 3.15, 4.50, (0.56, 0.89, 1.02, 1.09, 1.13, 1.14, 1.09, 0.72, 0.12)),
    (4.30, 2.90, 4.54, (0.64, 1.01, 1.17, 1.26, 1.31, 1.33, 1.31, 1.12, 0.06)),
    (3.80, 2.61, 4.61, (0.68, 1.07, 1.23, 1.32, 1.38, 1.41, 1.40, 1.26, 0.39)),
    (3.00, 2.41, 4.73, (0.72, 1.11, 1.28, 1.38, 1.44, 1.47, 1.46, 1.31, 0.40)),
    (2.00, 2.30, 4.81, (0.25, 1.10, 1.28, 1.38, 1.44, 1.48, 1.47, 1.34, 0.34)),
    (1.00, 2.33, 4.86, (0.30, 1.08, 1.27, 1.40, 1.46, 1.48, 1.43, 1.30, 0.30)),
    (0.00, 2.36, 4.92, (0.24, 1.03, 1.25, 1.40, 1.46, 1.45, 1.38, 1.25, 0.21)),
    (-1.00, 2.42, 4.96, (0.50, 1.00, 1.22, 1.35, 1.40, 1.38, 1.33, 1.21, 0.23)),
    (-2.00, 2.46, 4.99, (0.50, 0.95, 1.18, 1.30, 1.33, 1.31, 1.27, 1.16, 0.22)),
    (-2.35, 2.49, 5.04, (0.50, 0.93, 1.15, 1.27, 1.28, 1.26, 1.23, 1.10, 0.06)),
    (-2.80, 2.52, 5.44, (0.49, 0.94, 1.12, 1.21, 1.21, 1.18, 1.04, 0.56, 0.08)),
    (-3.30, 2.57, 5.69, (0.47, 0.91, 1.10, 1.14, 1.14, 1.06, 0.76, 0.58, 0.07)),
    (-4.00, 2.62, 5.80, (0.43, 0.86, 1.05, 1.06, 1.05, 0.91, 0.68, 0.56, 0.06)),
    (-4.80, 2.68, 5.71, (0.37, 0.79, 0.96, 0.96, 0.95, 0.80, 0.61, 0.53, 0.09)),
    (-5.60, 2.75, 5.53, (0.32, 0.71, 0.87, 0.87, 0.86, 0.75, 0.60, 0.49, 0.17)),
    (-6.50, 2.82, 5.37, (0.26, 0.62, 0.76, 0.77, 0.75, 0.65, 0.53, 0.43, 0.14)),
    (-7.50, 2.90, 5.18, (0.20, 0.53, 0.64, 0.65, 0.63, 0.55, 0.45, 0.37, 0.11)),
    (-8.50, 2.98, 5.00, (0.13, 0.43, 0.52, 0.53, 0.51, 0.45, 0.37, 0.31, 0.07)),
    (-9.50, 3.05, 4.84, (0.08, 0.33, 0.41, 0.42, 0.40, 0.36, 0.32, 0.26, 0.06)),
    (-10.50, 3.21, 4.68, (0.10, 0.26, 0.31, 0.31, 0.29, 0.26, 0.22, 0.17, 0.06)),
    (-11.20, 3.32, 4.57, (0.10, 0.21, 0.24, 0.24, 0.23, 0.21, 0.19, 0.15, 0.06)),
    (-11.80, 3.51, 4.30, (0.10, 0.16, 0.18, 0.18, 0.18, 0.17, 0.15, 0.12, 0.06)),
)
COWL_X, COWL_AFT, CANOPY = 4.76, 1.9, (-2.3, 4.93, -0.035, -6.4)
#: The sections the fuselage's contact boxes run between: nose to the wing, the cockpit, aft, the
#: tail.
HULL = (0, 7, 13, 18, len(SECTIONS) - 1)

#: The wing: its leading edge x = WING_LE[0] + WING_LE[1] |z| out to HALF, its trailing edge
#: likewise to TIP_Z, then cut forward to TIP_TE at HALF; its chord's middle at y = WING_MID[0] +
#: WING_MID[1] |z|, WING_T[0] - WING_T[1] |z| of its chord thick. The ailerons AILERON of the
#: chord from AIL_Z out to TIP_Z, hinged on their front.
WING_LE, WING_TE, WING_MID, WING_T = (1.47, -0.080), (-3.74, 0.1586), (3.39, 0.0085), (0.17, 0.009)
TIP_Z, TIP_TE, AILERON, AIL_Z = 10.0, -1.26, 0.28, 1.6

#: The stabiliser out to STAB_Z: leading and trailing edges as the wing's, at STAB_Y, STAB_T of
#: its chord thick; the elevator's halves behind ELEV_X from ELEV_Z out, clear of the rudder.
STAB_LE, STAB_TE, STAB_Y, STAB_Z, STAB_T, ELEV_X, ELEV_Z = (
    (-9.66, -0.118), (-12.53, 0.109), 4.43, 3.9, 0.09, -11.1, 0.55)

#: The fin from FIN_Y in the fuselage up to TOP_Y, its leading edge x = FIN_LE[0] + FIN_LE[1]
#: (y - FIN_Y); the rudder behind RUDDER_X from RUDDER_Y, the fuselage's end, its foot at FOOT,
#: its trailing edge raked RAKE up from it; both FIN_T[0] - FIN_T[1] (y - FIN_Y) of the chord
#: thick.
FIN_LE, FIN_Y, TOP_Y, RUDDER_X, RUDDER_Y, FOOT, RAKE = (
    (-9.75, -0.384), 4.4, 7.48, -11.85, 3.35, (-13.65, 3.60), 0.2526)
FIN_T = (0.085, 0.018)

#: The gear: the main wheels WHEEL_R; each spat SPAT's radii (across, along, up) about a point
#: SPAT_AT (along, up) from its wheel; each leg from the belly at LEG (x, y, |z|).
WHEEL_R, TAIL_WHEEL_R, SPAT, SPAT_AT, LEG = 0.42, 0.31, (0.385, 1.37, 0.35), (-0.38, 0.40), (1.45, 2.36, 0.7)


def body(x, y, z, k):
    """The model's point (x, y, z) in the body's axes, m, `k` m a unit."""
    return (k * z, k * x, k * (THRUST_Y - y))


def wing_edges(z):
    """The wing's (leading, trailing) edge x at `z`."""
    a = abs(z)
    le = WING_LE[0] + WING_LE[1] * a
    if a <= TIP_Z:
        return le, WING_TE[0] + WING_TE[1] * a
    te = WING_TE[0] + WING_TE[1] * TIP_Z
    return le, te + (TIP_TE - te) * (a - TIP_Z) / (HALF - TIP_Z)


def stab_edges(z):
    """The stabiliser's (leading, trailing) edge x at `z`."""
    a = abs(z)
    return STAB_LE[0] + STAB_LE[1] * a, STAB_TE[0] + STAB_TE[1] * a


def fin_edges(y):
    """The fin's and the rudder's (leading, trailing) edge x at height `y`: its foot a slope up
    from the fuselage's end to FOOT."""
    le = FIN_LE[0] + FIN_LE[1] * (y - FIN_Y)
    if y >= FOOT[1]:
        return le, FOOT[0] + RAKE * (y - FOOT[1])
    return le, RUDDER_X + (FOOT[0] - RUDDER_X) * (y - RUDDER_Y) / (FOOT[1] - RUDDER_Y)


def extents(k):
    """What touches the runway, in the body's axes at `k` m a unit, m: 'tail' the stabiliser's
    trailing tips, the rudder's foot, its top aft; 'tips' the wing's tip corners; 'fuselage' four
    boxes (centre, half sizes) along its sections - one, its belly's depth to the tail, struck the
    runway rolling -; 'wing', 'stab' and 'fin' a box each."""
    tip_le, tip_te = wing_edges(HALF)
    s_le, s_te = stab_edges(STAB_Z)
    top_le, top_te = fin_edges(TOP_Y)
    tips = [body(x, WING_MID[0] + WING_MID[1] * HALF, s * HALF, k) for s in (-1.0, 1.0) for x in (tip_le, tip_te)]
    tail = [body(s_te, STAB_Y, s * STAB_Z, k) for s in (-1.0, 1.0)] + [body(FOOT[0], FOOT[1], 0.0, k),
                                                                      body(top_te, TOP_Y, 0.0, k)]
    hull = []
    for a, b in zip(HULL, HULL[1:]):
        part = SECTIONS[a:b + 1]
        low, high, nose, end = min(s[1] for s in part), max(s[2] for s in part), part[0][0], part[-1][0]
        hull.append((body(0.5 * (nose + end), 0.5 * (low + high), 0.0, k),
                     (k * max(max(s[3]) for s in part), 0.5 * k * (nose - end), 0.5 * k * (high - low))))
    root_le, root_te = wing_edges(0.0)
    root_s, root_t = stab_edges(0.0)
    return {
        'tail': tail, 'tips': tips, 'fuselage': hull,
        'wing': (body(0.25 * (root_le + root_te + tip_le + tip_te), WING_MID[0], 0.0, k),
                 (k * HALF, 0.25 * k * (root_le - root_te + tip_le - tip_te),
                  0.25 * k * (WING_T[0] * (root_le - root_te) + (WING_T[0] - WING_T[1] * HALF) * (tip_le - tip_te)))),
        'stab': (body(0.25 * (root_s + root_t + s_le + s_te), STAB_Y, 0.0, k),
                 (k * STAB_Z, 0.25 * k * (root_s - root_t + s_le - s_te), 0.5 * k * STAB_T * (root_s - root_t))),
        'fin': (body(0.25 * (fin_edges(FIN_Y)[0] + top_le) + 0.25 * (FOOT[0] + top_te), 0.5 * (RUDDER_Y + TOP_Y), 0.0, k),
                (0.5 * k * FIN_T[0] * (fin_edges(FIN_Y)[0] - FOOT[0]),
                 0.25 * k * (fin_edges(FIN_Y)[0] - FOOT[0] + top_le - top_te), 0.5 * k * (TOP_Y - RUDDER_Y))),
    }
