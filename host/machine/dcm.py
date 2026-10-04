"""Standing balance as one law in the capture point's complex plane, no cases.

The user's (2026-10-04). Real is her left, imaginary ahead, from q, the support's point nearest
the capture point. Inside the sole's hold - HOLD_M across and along, a box about q - the ankle
can put the centre of pressure beyond the capture point and it comes back; outside, it cannot:
a step is due. The foot bearing less steps; the other stands, and once it stands alone the
capture point runs from its point as e^(omega t). The stepping foot lands on that ray - the
capture point then lies between her feet, whichever way she was pushed - STANCE_M past where
the capture point will be as it comes down, and clear of the standing sole.

    if dcm.due(xi):                              # the capture point out of the hold
        at = dcm.landing(xi, stand, omega, t, sign)   # complex from q; None if only across
"""
import math

#: The sole's hold past the support's point, m: across, the ankle's (`capture.MARGIN`'s 0.04),
#: and along, half the sole; the landing STANCE_M beyond the capture point as it will be at
#: STEP_S, a step's time from its call to bearing. STANCE_M 0.08 past a round hold of 0.04
#: returned every step as the next, an overshoot b running back at least b; a step called
#: while the capture point was still inside the hold, by the time it would take to leave,
#: fired on 2 cm with the ankle already bringing it back; an ellipse called the next step with
#: the capture point 3 cm inside and 5.3 ahead of the landed foot's point, on its sole
#: (2026-10-04).
HOLD_M = {'across': 0.04, 'along': 0.08}
STANCE_M, STEP_S = 0.02, 0.22

#: The stepping foot's point clear of the standing one's: a sole's width and 2 cm across, or a
#: sole's length and 1 cm along. The ray run from the centre of pressure between her feet put a
#: foot on her midline on a push from behind, the stance 6.6 cm wide; the support carried past
#: the capture point along the way it left alone, the feet kept apart, left the capture point
#: outside the line between them on every push with a side to it (2026-10-04).
CLEAR_M = {'across': 0.11, 'along': 0.25}


def hold(xi):
    """The hold's reach along `xi`'s way, m: the box HOLD_M."""
    d = abs(xi)
    if d == 0.0:
        return HOLD_M['across']
    c, s = abs(xi.real) / d, abs(xi.imag) / d
    return min(HOLD_M['across'] / c if c else math.inf, HOLD_M['along'] / s if s else math.inf)


def due(xi):
    """Whether a step is due: the capture point `xi` (complex, from q) out of the hold."""
    return abs(xi) >= hold(xi)


def landing(xi, stand, omega, t, sign):
    """Where the stepping foot (`sign` 1 the left, -1 the right) lands coming down in `t` s,
    complex from q: on the ray from `stand`, the standing foot's point, through the capture
    point `xi` as it will be then; None when that is across the standing foot."""
    d = (xi - stand) * math.exp(omega * t)
    r = abs(d)
    if r == 0.0 or sign * d.real <= 0.0:
        return None
    u = d / r
    clear = min(CLEAR_M['across'] / abs(u.real), CLEAR_M['along'] / abs(u.imag) if u.imag else math.inf)
    return stand + max(r + STANCE_M, clear) * u
