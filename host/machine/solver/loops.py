"""A loop's stiffness and damping, solved between what its task needs and what its machine allows.

    w = loops.natural(dead_s, zeta, margin_rad, lags_s)       # rad/s the dead time and the lags allow
    w = loops.rate_natural(dead_s, margin_rad, lags_s)        # ... for a first-order rate loop d / (J s)
    k = loops.stiffness(inertia, w)                           # N m/rad that frequency is on that inertia
    loop = loops.design(inertia, zeta, needs, ceilings)       # Loop(kp, kd, natural, need, ceiling, feasible)
    loop = loops.stiffest(inertia, zeta, needs, ceilings)     # ... at its least ceiling
    loop = loops.rate(inertia, needs, ceilings)               # first-order rate loop (kp = 0) at its ceiling
    kr = loops.bessel_rate(damping)                           # Bessel PI rate integral stiffness d^2 / 3
    wo = loops.observer(w, factor, ceiling_rad_s)             # observer bandwidth serving w, under ceiling

A drive holds an inertia J - a joint's, a craft's about an axis - by a stiffness kp and a damping kd:
the loop's natural frequency is sqrt(kp / J) and its damping ratio kd / (2 sqrt(kp J)). What sets them
is no share of a rate. A task needs a stiffness at the least (the gravity a stance joint must overcome,
the frequency a motion is to be followed at) and a machine allows one at the most (the phase margin its
dead time and its actuator's lags leave, a structure's series stiffness, the torque it has for an
excursion): a loop is feasible where its stiffest need lies under its least ceiling, and stands at
either end of that - the softest that meets every need (`design`: a joint's) or the stiffest its
machine allows (`stiffest`: a craft's tilt).
Every function is one of numbers alone: the robot's joints and a craft's axes are solved by the same
ones, a law that takes its gains per inertia with an inertia of one.
"""
import math
from typing import NamedTuple

#: The bisections' halvings: a double's 53 bits and a guard.
HALVINGS = 64


class Loop(NamedTuple):
    """A loop as solved: its stiffness and damping, the natural frequency they are on the most inertia
    the drive sees, the need that set the stiffness and the least ceiling by their names, and whether
    the need lies under that ceiling."""
    kp: float
    kd: float
    natural_rad_s: float
    need: str
    ceiling: str
    feasible: bool


def crossover(zeta):
    """Where a loop damped `zeta` has gain one, over its natural frequency: x of x^4 = 1 + (2 zeta x)^2,
    the loop (kp + kd s) / (J s^2) with no dead time and no lag."""
    twice = 2.0 * zeta * zeta
    return math.sqrt(twice + math.sqrt(twice * twice + 1.0))


def margin(zeta):
    """The phase margin, rad, a loop damped `zeta` has of its own, with no dead time and no lag: the lead
    of its damping where its gain is one. What a dead time and lags may take of it is a design's target:
    `natural(dead_s, zeta, margin(zeta) - taken_rad)`."""
    return math.atan(2.0 * zeta * crossover(zeta))


def _margin(natural_rad_s, zeta, dead_s, lags_s):
    """The phase margin, rad, of a loop of `natural_rad_s` damped `zeta` behind a dead time and first-order
    lags: found at the frequency its gain is one, a bisection under the crossover it has without them."""
    low, high = 0.0, crossover(zeta) * natural_rad_s
    for _ in range(HALVINGS):
        mid = 0.5 * (low + high)
        lead = math.hypot(1.0, 2.0 * zeta * mid / natural_rad_s)
        lagged = math.prod(math.hypot(1.0, mid * lag) for lag in lags_s)
        gain = natural_rad_s * natural_rad_s * lead / (mid * mid * lagged)
        low, high = (mid, high) if gain > 1.0 else (low, mid)
    cross = 0.5 * (low + high)
    return (math.atan(2.0 * zeta * cross / natural_rad_s) - cross * dead_s
            - sum(math.atan(cross * lag) for lag in lags_s))


def natural(dead_s, zeta, margin_rad, lags_s=()):
    """The highest natural frequency, rad/s, at which a loop damped `zeta` keeps the phase margin
    `margin_rad` behind its dead time `dead_s` (sampling, transport) and its actuator's first-order lags
    `lags_s`: without either no frequency is too high (inf), and a margin of nothing, or one the loop
    does not have with neither, is refused."""
    free = margin(zeta)
    if not 0.0 < margin_rad < free:
        raise ValueError('a loop damped %g has a margin under %g rad, above none: %g asked' % (zeta, free, margin_rad))
    slow = dead_s + sum(lags_s)
    if not slow > 0.0:
        return math.inf
    low, high = 0.0, 1.0 / slow
    while _margin(high, zeta, dead_s, lags_s) > margin_rad:
        low, high = high, 2.0 * high
    for _ in range(HALVINGS):
        mid = 0.5 * (low + high)
        low, high = (mid, high) if _margin(mid, zeta, dead_s, lags_s) > margin_rad else (low, mid)
    return low


def stiffness(inertia, natural_rad_s):
    """The stiffness, N m/rad, a natural frequency is on an inertia: J w^2."""
    return inertia * natural_rad_s * natural_rad_s


def design(inertia, zeta, needs, ceilings):
    """The loop on `inertia` - kg m^2, one where the law takes its gains per inertia, (least, most) where
    the drive sees a range of it - damped `zeta`: the softest stiffness that meets every one of `needs`
    ({name: N m/rad at the least}), feasible where it lies under every one of `ceilings` ({name: N m/rad
    at the most}); its damping is `zeta`'s on the geometric mean of the least and the most inertia - as
    far over it on the least as under it on the most. A need on a frequency is `stiffness(most, w)`, a
    ceiling on one `stiffness(least, w)`."""
    need, ceiling = _bounds(needs, ceilings)
    return _laid(inertia, zeta, needs[need], need, ceiling, needs[need] <= ceilings[ceiling])


def stiffest(inertia, zeta, needs, ceilings):
    """The loop on `inertia` as stiff as its machine allows, where `design` lays the softest its task
    needs: its stiffness the least of `ceilings`, feasible where every one of `needs` lies under it. A
    loop asked to stand at what its drive gives - a craft's tilt, all its drive at the excursion that
    may ask it - with the motion it is to follow the need beneath. The same Loop: `need` the stiffest
    need, `ceiling` the one that set the stiffness; inertia and damping as `design` takes them."""
    need, ceiling = _bounds(needs, ceilings)
    return _laid(inertia, zeta, ceilings[ceiling], need, ceiling, needs[need] <= ceilings[ceiling])


def _bounds(needs, ceilings):
    """(the stiffest need's name, the least ceiling's)."""
    return max(needs, key=lambda name: needs[name]), min(ceilings, key=lambda name: ceilings[name])


def _laid(inertia, zeta, kp, need, ceiling, feasible):
    """The Loop of a stiffness on an inertia or a range of it: damped on the geometric mean of the least
    and the most, its natural frequency the one on the most."""
    least, most = (min(inertia), max(inertia)) if isinstance(inertia, tuple) else (inertia, inertia)
    return Loop(kp, 2.0 * zeta * math.sqrt(kp * math.sqrt(least * most)), math.sqrt(kp / most), need, ceiling, feasible)


def _rate_margin(rate_rad_s, dead_s, lags_s):
    """Phase margin, rad, of a first-order rate loop `rate_rad_s / s` behind `dead_s` and `lags_s`."""
    low, high = 0.0, rate_rad_s
    for _ in range(HALVINGS):
        mid = 0.5 * (low + high)
        gain = rate_rad_s / (mid * math.prod(math.hypot(1.0, mid * lag) for lag in lags_s)) if mid > 0.0 else math.inf
        low, high = (mid, high) if gain > 1.0 else (low, mid)
    cross = 0.5 * (low + high)
    return 0.5 * math.pi - cross * dead_s - sum(math.atan(cross * lag) for lag in lags_s)


def rate_natural(dead_s, margin_rad, lags_s=()):
    """The highest crossover frequency, rad/s, at which a first-order rate loop `d / (J s)` keeps the phase
    margin `margin_rad` (between none and pi / 2) behind its dead time `dead_s` and first-order lags `lags_s`."""
    if not 0.0 < margin_rad < 0.5 * math.pi:
        raise ValueError('a rate loop has a margin under pi/2 rad, above none: %g asked' % margin_rad)
    slow = dead_s + sum(lags_s)
    if not slow > 0.0:
        return math.inf
    if not lags_s:
        return (0.5 * math.pi - margin_rad) / dead_s
    low, high = 0.0, (0.5 * math.pi - margin_rad) / slow
    while _rate_margin(high, dead_s, lags_s) > margin_rad:
        low, high = high, 2.0 * high
    for _ in range(HALVINGS):
        mid = 0.5 * (low + high)
        low, high = (mid, high) if _rate_margin(mid, dead_s, lags_s) > margin_rad else (low, mid)
    return low


def rate(inertia, needs, ceilings):
    """A first-order rate loop on `inertia` (kp = 0, kd in N m s/rad or 1/s for unit inertia) at the least of
    `ceilings`, feasible where every one of `needs` lies under it."""
    need, ceiling = _bounds(needs, ceilings)
    most = max(inertia) if isinstance(inertia, tuple) else inertia
    kd = ceilings[ceiling]
    return Loop(0.0, kd, kd / most, need, ceiling, needs[need] <= kd)


def bessel_rate(damping):
    """The integral rate stiffness `d^2 / 3` of a second-order Bessel maximally-flat PI rate loop
    `(d s + kr) / (J s^2)` with rate damping `d`: damping ratio sqrt(3) / 2, zero step overshoot."""
    return damping * damping / 3.0


def observer(loop_rad_s, factor, ceiling_rad_s=math.inf):
    """An extended state observer's bandwidth, rad/s, serving a loop of `loop_rad_s` by `factor`, bounded
    above by `ceiling_rad_s`."""
    return min(factor * loop_rad_s, ceiling_rad_s)
