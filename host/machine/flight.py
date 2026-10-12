"""Flight on the whole-body stack: which soles stand, told from this pass alone.

    ask = flight.step(ask, loads, s, hips)        # between a law's ask and `wbc.step`

The stack lays a force on every sole it is asked to stand on; a floor that is not there gives
none. A sole the ask means to stand stands as it bears TOUCH_N; one that bears nothing stands
lying within TOUCH_M of the floor beside one that bears, or with her centre of mass at rest. One
that does not stand is asked to its landing: under her capture point as she comes down, as far
off it as it is off the others, drawn in to what its leg reaches (`reach.land`), its hip
LAND_DROP_M under the leg's length over it, falling as she falls. With no sole standing no turn
is asked, the stack lays no force, and her centre of mass falls: `wbc.step` and the loop in C
need nothing more. A limb is an index, as `reach.LIMBS`; nothing is kept from one pass to the
next.
"""
import math

from machine import reach
from machine.figure import ANKLE_H, BALL, HEEL

G = 9.81

#: A sole bears at TOUCH_N, lies on the floor within TOUCH_M; she rests under REST_M_S.
#: Near alone, the stack pressed a sole 1 mm over the floor onto it as she left: 648 N on it,
#: 23 deg over. All or none while she rises flickered on one sole: 21 deg over three hops.
TOUCH_N, TOUCH_M, REST_M_S = 15.0, 0.002, 0.1

#: A sole in the air waits for the floor with its hip LAND_DROP_M under the leg's length over
#: it - the knee 25 deg -, no straighter than LAND_SHORT_M short of that length where drawn.
LAND_DROP_M, LAND_SHORT_M = 0.018, 0.004

#: The sole's middle ahead of its ankle, m.
MIDDLE = (BALL - HEEL) / 2.0


def standing(meant, loads, over, rising):
    """Which soles stand, a bool a sole: `meant` the ask's stance, `loads` the soles' N, `over`
    each sole's height over the floor, `rising` her centre of mass's speed up."""
    rests = (max(loads) >= TOUCH_N) | (abs(rising) <= REST_M_S)
    return tuple(bool(m) & bool((n >= TOUCH_N) | ((h <= TOUCH_M) & rests))
                 for m, n, h in zip(meant, loads, over))


def landing(limb, hip, heading, yaw, com, vcom, off):
    """(sole's middle, speed, acceleration, quaternion), the stack's swing ask for a sole in the
    air: under her capture point as she lands - from her hip the same whenever that is -, `off`
    it (x, z), drawn in to the leg's reach from `hip`."""
    drop = reach.THIGH + reach.SHANK - LAND_DROP_M
    omega = math.sqrt(G / max(0.3, com[1] - (hip[1] - drop - ANKLE_H)))
    c, s = math.cos(yaw), math.sin(yaw)
    want = ((com[0] + vcom[0] / omega + off[0] - MIDDLE * s, hip[1] - drop,
             com[2] + vcom[2] / omega + off[1] - MIDDLE * c), yaw)
    (ankle, yaw), _m = reach.land(limb, hip, heading, want, LAND_SHORT_M)
    c, s = math.cos(yaw), math.sin(yaw)
    return ((ankle[0] + MIDDLE * s, ankle[1] - ANKLE_H, ankle[2] + MIDDLE * c),
            (vcom[0], vcom[1], vcom[2]), (0.0, -G, 0.0),
            (math.cos(0.5 * yaw), 0.0, math.sin(0.5 * yaw), 0.0))


def step(ask, loads, s, hips, floor=0.0):
    """`ask` as the stack is to be asked it: the soles it means to stand that do not touch asked
    to their landing; with none standing, no turn. `s` is `wbc.sense`'s, `hips` each limb's hip
    joint, world, `floor` the floor's height under her."""
    meant = ask['stance']
    feet = s['feet']
    over = [float(f['pts'][:, 1].min()) - floor for f in feet]
    stance = standing(meant, loads, over, float(s['vcom'][1]))
    count = sum(bool(m) for m in meant)
    mid = [sum(float(f['sole'][i]) * bool(m) for f, m in zip(feet, meant)) / max(1, count) for i in (0, 2)]
    w, x, y, z = (float(v) for v in s['turns'][0]['quat'])
    heading = math.atan2(2.0 * (x * z + w * y), 1.0 - 2.0 * (x * x + y * y))
    swing = dict(ask.get('swing', {}))
    swing.update({k: landing(k, hips[k], heading, math.atan2(feet[k]['R'][0][2], feet[k]['R'][2][2]),
                             s['com'], s['vcom'], (float(feet[k]['sole'][0]) - mid[0],
                                                   float(feet[k]['sole'][2]) - mid[1]))
                  for k in (0, 1) if meant[k] and not stance[k]})
    # with no sole standing there is nothing to turn her against: asked, even as they were,
    # the stack spun her legs to hold her trunk still - the knees out at 3 rad/s with no
    # torque in them, on their stops as she landed
    return dict(ask, stance=stance, swing=swing, turns=ask['turns'][:2 * any(stance)])
