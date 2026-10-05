"""What of her is free on the one law: a leg on its way to its landing, her arms.

    joints = free.lifted(law, leg, sign, pel, now, free.swung(law, leg, sign, pel, now, v))
    out.update(free.upper(law, swing, roll, dt))

`law` is `machine.going`'s `Going`, `leg` a row of its legs that does not stand. The leg goes
joint by joint from where it left the floor to its landing pose (`Going.landing`), its knee
folded on the way, clear of the floor and of the standing foot; an arm swings against its own
leg, its elbow bent as the row has it.
"""
import math

from machine import figure, gait
from machine.figure import SOLE_BALL, SOLE_HEEL, add, apply
from machine.runner import ARM, COAST_S, LEVEL, eased, pitched

#: Clearance. A free sole is LIFT_M over the floor from LIFTS_U of its swing to LANDS_U, let go
#: over DOWN_U: raised to it, the foot as it was - a knee's fold slewed 230 deg/s lost to the
#: knee's own 340 unfolding. A landing's ankle is CLEAR_M across from a standing one's still
#: down then, and the free foot as far within PASS_M of it along her way: 6-7 cm apart it
#: struck the standing foot, 180-475 N on both soles' sensors; held so in a run, her feet 7 cm
#: apart, she was down in 3 s (2026-10-05).
LIFT_M, LIFTS_U, LANDS_U, DOWN_U, CLEAR_M, PASS_M = 0.01, 0.2, 0.8, 0.1, 0.11, (0.15, 0.27)
SOLE_TOE = (0.0, -gait.ANKLE_H, gait.BALL + gait.TOE_M)

#: An elbow follows its shoulder's reach ahead ARM_S behind it.
ARM_S = 0.1


def swung(law, leg, sign, pel, now, v):
    """A free leg's six joints, rad, `leg['u']` of its way from where it left the floor to
    its landing pose: joint by joint, its knee folded on the way, its foot level; the height
    the pelvis has with that pose's sole down kept (`leg['meets']`)."""
    a = law.ask
    ankle, foot = law.landing(sign, figure.hip(sign, pel, now), v, leg['to_go'],
                              leg.get('reach', 0.0), leg.get('beside'))
    leg['meets'] = pel[1] - law.lowest(ankle, foot)
    joints = list(figure.leg(sign, pel, now, ankle, foot))
    if leg.get('from') is None:
        return joints
    u = leg['u']
    coast = COAST_S * (1.0 - math.exp(-leg['t'] / COAST_S))
    joints = [p + r * coast + (q - p - r * coast) * eased(u)
              for p, r, q in zip(leg['from'], leg['rate'], joints)]
    joints[3] += math.radians(a['fold']) * math.sin(math.pi * min(1.0, u / a['folded']))
    rise = (leg['rise'] * (1.0 - eased(u / LEVEL))
            + math.radians(a['land']) * eased((u - LEVEL) / (1.0 - LEVEL)))
    joints[4] = rise - (pitched(now) + joints[2] + joints[3])
    return joints


def lifted(law, leg, sign, pel, now, joints):
    """What is free keeps its clearance (LIFT_M, CLEAR_M ..): a sole under it is raised to
    it, a foot passing the standing one moved out from it, the foot as it was."""
    u = leg.get('u', 1.0)
    clear = LIFT_M * eased(u / LIFTS_U) * (1.0 - eased((u - LANDS_U) / DOWN_U))
    ankle, foot = figure.foot_of(sign, pel, now, joints)
    # coming down, its heel and its ball alone - the toes give: counted, a forefoot landing
    # was lifted 1.6 cm, the run's knee 28 deg where 22, and she was down at 0.8 s
    low = min(add(ankle, apply(foot, q))[1]
              for q in (SOLE_HEEL, SOLE_BALL) + (SOLE_TOE,) * (u <= LANDS_U))
    up = max(0.0, clear - low) if clear > 0.0 else 0.0
    c, s = math.cos(law.heading), math.sin(law.heading)
    out = 0.0
    if leg.get('beside') is not None:
        (bx, _by, bz), share = leg['beside']
        along = (ankle[0] - bx) * s + (ankle[2] - bz) * c
        near = 1.0 - eased((abs(along) - PASS_M[0]) / (PASS_M[1] - PASS_M[0]))
        out = max(0.0, share * near * CLEAR_M
                  - sign * ((ankle[0] - bx) * c - (ankle[2] - bz) * s))
    if up <= 0.0 and out <= 0.0:
        return joints
    return list(figure.leg(sign, pel, now, (ankle[0] + sign * out * c, ankle[1] + up,
                                            ankle[2] - sign * out * s), foot))


def upper(law, swing, roll=0.0, dt=0.0):
    """The upper body: the arms against the legs' swing {side: deg ahead}, an elbow bent the
    row's `elbow` and `play` deg more a deg its shoulder reaches ahead - on the runner's 80
    deg, still, her walk's arms stood out before her (the user, 2026-10-05) -; the spine
    against the pelvis's `roll`, rad."""
    a = law.ask
    out = {'spine_roll': math.degrees(roll), 'spine': 0.0, 'waist': 0.0,
           'neck': -0.5 * a['lean'], 'head': 0.0}
    for side in ('left', 'right'):
        ahead = out[side + '_shoulder'] = -ARM * swing[side]
        was = law.arms.get(side, ahead)
        law.arms[side] = was + min(1.0, dt / ARM_S) * (ahead - was)
        out[side + '_elbow'] = max(0.0, a['elbow'] + a['play'] * law.arms[side])
        out[side + '_wrist'], out[side + '_gripper'] = 0.0, 20.0
    return out
