"""A standing leg a strut: how high it carries the pelvis, how its foot lies on the floor.

    y = strut.reach(sign, at, turn, strut.ankle(flat, pitch, heading), knee)
    pitch = strut.rocker(leg, off, on, sunk, strut.need(hip, flat, heading, knee))

For her going on the one law (`machine.going`). The pelvis is never over what a leg's knee at
`knee` deg reaches, a row's `strut` (`machine.gaits`). Its foot gives the pitch it landed with
to the floor, and lets its heel rise as the hip goes ahead of it or as the strut's length asks.
"""
import math

from machine import gait
from machine.figure import SOLE_BALL, SOLE_HEEL, add, apply, mul, rx, ry, sub
from machine.runner import ABSORB, HEEL_FROM_M, HEEL_TO_M, eased
from machine.stance import length

#: A heel rises as the strut's length asks, HEEL_UP_DEG at most. On its heel a foot's toes
#: are down FLAT_S on.
HEEL_UP_DEG, FLAT_S = 50.0, 0.09


def reach(sign, at, turn, ankle, knee):
    """The pelvis's height, m, its hip a strut's length - the knee at `knee` deg - over
    `ankle`, the pelvis at `at` (x, z) along the floor."""
    hip = apply(turn, (sign * gait.HIP_HALF, -gait.HIP_DROP, 0.0))
    flat2 = (at[0] + hip[0] - ankle[0]) ** 2 + (at[1] + hip[2] - ankle[2]) ** 2
    return ankle[1] + math.sqrt(max(0.0, length(knee) ** 2 - flat2)) - hip[1]


def ankle(flat, pitch, heading):
    """A standing foot's ankle, pitched `pitch` rad about its heel or its ball, from where it
    is flat."""
    p = SOLE_HEEL if pitch < 0.0 else SOLE_BALL
    return add(flat, sub(apply(ry(heading), p), apply(mul(ry(heading), rx(pitch)), p)))


def heel(off, on):
    """The heel's rise, rad, her hip `on` m ahead of the ankle: `off` deg in full."""
    return math.radians(off) * eased((on - HEEL_FROM_M) / (HEEL_TO_M - HEEL_FROM_M))


def top(sign, at, turn, flat, heading, off, knee):
    """`reach` over the foot flat at `flat`, its heel risen as her hip's place has it."""
    hip = apply(turn, (sign * gait.HIP_HALF, -gait.HIP_DROP, 0.0))
    on = ((at[0] + hip[0] - flat[0]) * math.sin(heading)
          + (at[1] + hip[2] - flat[2]) * math.cos(heading))
    return reach(sign, at, turn, ankle(flat, heel(off, on), heading), knee)


def need(hip, flat, heading, knee):
    """The heel's rise, rad, that lets a strut - the knee at `knee` deg - reach `hip` from the
    foot flat at `flat`."""
    far = length(knee)
    low, high = 0.0, math.radians(HEEL_UP_DEG)
    if math.dist(hip, ankle(flat, low, heading)) <= far:
        return 0.0
    for _ in range(12):
        mid = 0.5 * (low + high)
        low, high = (mid, high) if math.dist(hip, ankle(flat, mid, heading)) > far else (low, mid)
    return high


def rocker(leg, off, on, sunk, need=0.0):
    """A standing foot's pitch, rad, the heel up: what it landed with given to the floor - on
    its ball as she sinks `sunk` m, on its heel in FLAT_S -, and the heel's rise as her hip
    goes `on` m ahead of the ankle (`heel`) or as its leg's length asks, `need`."""
    landed, up = leg['landed'], max(need, heel(off, on))
    if landed <= 0.0:
        return landed * (1.0 - eased(leg['t'] / FLAT_S)) + up
    high = gait.BALL * math.sin(landed)
    give = high * math.exp(-ABSORB * max(0.0, sunk) / high) if high > 1e-6 else 0.0
    return max(math.asin(min(1.0, give / gait.BALL)), up)
