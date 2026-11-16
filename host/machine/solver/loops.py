"""A loop's stiffness and damping, solved between what its task needs and what its machine allows.

    w = loops.natural(dead_s, zeta, margin_rad, lags_s)       # rad/s the dead time and the lags allow
    k = loops.stiffness(inertia, w)                           # N m/rad that frequency is on that inertia
    loop = loops.design(inertia, zeta, needs, ceilings)       # Loop(kp, kd, natural, need, ceiling, feasible)

A drive holds an inertia J - a joint's, a craft's about an axis - by a stiffness kp and a damping kd:
the loop's natural frequency is sqrt(kp / J) and its damping ratio kd / (2 sqrt(kp J)). What sets them
is no share of a rate. A task needs a stiffness at the least (the gravity a stance joint must overcome,
the frequency a motion is to be followed at) and a machine allows one at the most (the phase margin its
dead time and its actuator's lags leave, a structure's series stiffness, the torque it has for an
excursion): the loop is the softest that meets every need, feasible where no ceiling lies under it.
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
    free = math.atan(2.0 * zeta * crossover(zeta))
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
    at the most}); its damping is `zeta`'s on the most inertia, more on less. A need on a frequency is
    `stiffness(most, w)`, a ceiling on one `stiffness(least, w)`."""
    most = max(inertia) if isinstance(inertia, tuple) else inertia
    need = max(needs, key=lambda name: needs[name])
    ceiling = min(ceilings, key=lambda name: ceilings[name])
    kp = needs[need]
    return Loop(kp, 2.0 * zeta * math.sqrt(kp * most), math.sqrt(kp / most), need, ceiling, kp <= ceilings[ceiling])
