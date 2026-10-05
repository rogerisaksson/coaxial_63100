"""Her stand on the one law: the foot that stays, when the other leaves, how her legs lean her.

    law.stays, law.parry = hold.staying(law, a, loads)     # both feet down
    hold.leaves(law, side, a, loads, share)                  # the other may go
    dx, dz = hold.lean(law, standing, a, com)                # the pelvis asked that far on

Her capture point - her centre of mass and its speed over the pendulum's rate - is asked where
she is to go: ahead of her by the speed asked over that rate and, the more she stands (`stood`),
over the middle of her feet. A step is wanted as she is asked on, or as it leaves her feet's hold
(`dcm.due`): then, both feet down, her legs bring it to where a step leaves from, `track` in
from the foot that stays, and the other leaves as it is there. `law` is `machine.going`'s
`Going`.
"""
import math

from machine import dcm
from machine.runner import eased

#: Her standing legs lean her LEAN_K m a m her capture point is off where it is asked - HOLD_K
#: standing, what she still moves taken back LEAN_D of it -, LEAN_M at most; asked under
#: GOES_M_S she stands the more.
LEAN_K, HOLD_K, LEAN_D, LEAN_M, GOES_M_S = 0.2, 0.4, 0.014, 0.025, 0.34

#: A step is wanted asked STIRS_M_S or over. The other foot leaves with her capture point no
#: more than `track` and SHIFT_M in from the one that stays, or bearing under LIGHT_N. Waited for
#: within SHIFT_M either way, a foot down 2.4 cm from it kept the other on the floor, and her
#: capture point ran 3.7 cm out over the first: down at 1.4 s (2026-10-05).
STIRS_M_S, SHIFT_M, LIGHT_N = 0.067, 0.02, 43.0


def stood(speed):
    """How much she stands at the speed asked: 1 at none, 0 from GOES_M_S."""
    return 1.0 - eased(speed / GOES_M_S)


def point(law, com):
    """(x, z) of her capture point, her centre of mass at `com`."""
    return com[0] + law.v[0] / law.omega, com[2] + law.v[2] / law.omega


def held(law, a, com):
    """(x, z) of her standing feet's line nearest her capture point - what holds her - and that
    point."""
    c, s = math.cos(law.heading), math.sin(law.heading)
    xi = point(law, com)
    feet = [(leg['flat'][0] + a['under'] * s, leg['flat'][2] + a['under'] * c)
            for leg in law.legs.values() if leg['stands']]
    (ax, az), (bx, bz) = feet[0], feet[-1]
    span = (bx - ax) ** 2 + (bz - az) ** 2
    k = max(0.0, min(1.0, ((xi[0] - ax) * (bx - ax) + (xi[1] - az) * (bz - az)) / span)
            ) if span > 0.0 else 0.0
    return (ax + k * (bx - ax), az + k * (bz - az)), xi


def staying(law, a, loads):
    """(the foot that stays as the other steps, whether that one leaves at once), both down:
    (None, False) with no step wanted. Her capture point out of her feet's hold and she hardly
    going, the foot bearing more stays and the other leaves at once; else the one landed last
    stays - as long down, the one bearing more -, kept till the other has left."""
    c, s = math.cos(law.heading), math.sin(law.heading)
    at, xi = held(law, a, law.com)
    off = (xi[0] - at[0], xi[1] - at[1])
    more = max(loads, key=loads.get)
    if a['speed'] < GOES_M_S and dcm.due(complex(off[0] * c - off[1] * s,
                                                 off[0] * s + off[1] * c)):
        return more, True
    if a['speed'] < STIRS_M_S:
        return None, False
    if law.stays is not None:
        return law.stays, False
    left, right = law.legs['left']['t'], law.legs['right']['t']
    return ('left' if left < right else 'right' if right < left else more), False


def leaves(law, side, a, loads, share):
    """Whether the foot `side` leaves, both down and one to stay (`staying`): at once, the hold
    lost; else as her capture point is where a step leaves from, or as it bears under LIGHT_N -
    the less a walk's (`share` of one), the further in it may be."""
    if law.stays in (None, side):
        return False
    if law.parry:
        return True
    c, s = math.cos(law.heading), math.sin(law.heading)
    stay, sign = law.legs[law.stays]['flat'], 1.0 if side == 'left' else -1.0
    xi = point(law, law.com)
    far = sign * ((xi[0] - stay[0]) * c - (xi[1] - stay[2]) * s)
    return far * share < a['track'] + SHIFT_M or loads[side] < LIGHT_N


def lean(law, standing, a, com):
    """(x, z) her standing legs lean the pelvis, m, toward where her capture point is asked.
    Along: ahead by the speed asked over the pendulum's rate; standing, over her feet's middle
    - asked onto their line where it is nearest, nothing held her along it and she was down at
    5.1 s. Across with both feet down alone, a step wanted `track` in from the foot that stays:
    one holds her no wider than its sole, and held over it her capture point stayed there -
    the free foot came down 11 cm from it, hers then 11 cm in from that one and off across at
    1.5 m/s (2026-10-05)."""
    c, s = math.cos(law.heading), math.sin(law.heading)
    still = stood(a['speed'])
    mid = [sum(law.legs[side]['flat'][i] for side in standing) / len(standing) for i in (0, 2)]
    if len(standing) == 2 and law.stays is not None:
        stay = law.legs[law.stays]['flat']
        shift = (stay[0] - mid[0]) * c - (stay[2] - mid[1]) * s
        shift -= math.copysign(min(abs(shift), a['track']), shift)
        mid = [mid[0] + shift * c, mid[1] - shift * s]
    on_v, x_v = law.v[0] * s + law.v[2] * c, law.v[0] * c - law.v[2] * s
    xi = point(law, com)
    e = (mid[0] - xi[0], mid[1] - xi[1])
    along = (still * (HOLD_K * (e[0] * s + e[1] * c + a['under']) - LEAN_D * on_v)
             + (1.0 - still) * LEAN_K * (a['speed'] - on_v) / law.omega)
    across = (HOLD_K * (e[0] * c - e[1] * s) - LEAN_D * x_v) * (
        (still if law.stays is None else 1.0) if len(standing) == 2 else 0.0)
    far = math.hypot(along, across)
    k = min(1.0, LEAN_M / far) if far > 0.0 else 0.0
    return k * (along * s + across * c), k * (along * c - across * s)
