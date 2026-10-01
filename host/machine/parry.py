"""The parry's upper body, shoved or catching: the arms raised, eased in, held, eased out.

    k, way = parry.ease((k, way), active, toward, dt)   # toward: +1 to her left, -1 her right
    out = parry.upper(out, k, way)                       # the walker's setpoints, the parry on

The step that catches her is the walker's (`machine.landing`). The trunk rolled toward the fall
- the hip strategy - and the waist turning the arms toward it were measured and are off.
"""

#: The spine rolled TRUNK_DEG toward the fall at most, the waist turned FACE_DEG toward it, the
#: shoulders raised ARMS_DEG and the elbows bent ELBOWS_DEG; in over IN_S, out over OUT_S. Named
#: apart from gait's ROLL_DEG and falls' WAIST_DEG: a knob sets the first module holding it. Held
#: of 48 shoves of 60 N: still 24; the arms 15 and 20 22, 30 and 30 11; rolled 12 either way 0,
#: the waist 5 lost 3 of 16 (2026-10-01).
TRUNK_DEG, FACE_DEG, ARMS_DEG, ELBOWS_DEG, IN_S, OUT_S = 0.0, 0.0, 15.0, 20.0, 0.12, 0.5


def ease(state, active, toward, dt):
    """(k, way) a pass on: k toward 1 while `active`, else back to 0; the way latched as k
    leaves 0."""
    k, way = state
    if active and k == 0.0:
        way = toward
    return (min(1.0, k + dt / IN_S) if active else max(0.0, k - dt / OUT_S)), way


def upper(out, k, way):
    """`out` with the parry k of the way on toward `way`: the spine's roll (+ to her right), the
    waist's turn (+ to her left), the arms."""
    if k <= 0.0:
        return out
    k = k * k * (3.0 - 2.0 * k)
    out = dict(out)
    out['spine_roll'] = out.get('spine_roll', 0.0) - way * TRUNK_DEG * k
    out['waist'] = out.get('waist', 0.0) + way * FACE_DEG * k
    for side in ('left_', 'right_'):
        out[side + 'shoulder'] = out.get(side + 'shoulder', 0.0) + ARMS_DEG * k
        out[side + 'elbow'] = out.get(side + 'elbow', 0.0) + ELBOWS_DEG * k
    return out
