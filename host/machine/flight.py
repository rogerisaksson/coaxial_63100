"""Flight on the whole-body stack: soles that bear nothing are released, taken again as she lands.

    st = flight.state()
    ask = flight.step(st, ask, loads, bears, s, hips)    # between a law's ask and `wbc.step`

The stack lays a force on every sole it is asked to stand on; a floor that is not there gives
none. The soles the ask means to stand are released - asked as swinging - together, once every
one of them has borne under RELEASE_N for RELEASE_S: she has left the floor. One alone only as a
sole lost, the stack having laid weight on it LOST_S with nothing under it. Released, a sole is
asked to where it lands: under her capture point as she comes down, as far off it as it stood
off the others, drawn in to what its leg reaches (`reach.land`), its hip LAND_DROP_M under the
leg's length over it, falling as she falls. It stands again bearing LAND_N as she comes down.
With no sole standing the stack is asked no turn, lays no force, and her centre of mass falls:
`wbc.step` and the loop in C need nothing more. A limb is an index, as `reach.LIMBS`: no sole is
told from the other.
"""
import math

from machine import reach
from machine.gait import ANKLE_H, BALL, HEEL

G = 9.81

#: The standing soles are released having all borne under RELEASE_N for RELEASE_S; one alone
#: after LOST_S of it, the stack laying over twice that on it (`balance.LOST_S`). A sole stands
#: again bearing LAND_N while she rises no faster than RISES_M_S. Released one at a time as
#: each rose, or one taken again as it brushed the floor 2 ms into her rise, the other was left
#: the whole throw: 648 N on one sole for 50 ms, and over she went, 19 deg.
RELEASE_N, RELEASE_S, LOST_S, LAND_N, RISES_M_S = 15.0, 0.004, 0.05, 20.0, 0.1

#: A released sole waits for the floor with its hip LAND_DROP_M under the leg's length over it -
#: the knee 25 deg -, no straighter than LAND_SHORT_M short of that length wherever it is drawn.
LAND_DROP_M, LAND_SHORT_M = 0.018, 0.004

#: The sole's middle ahead of its ankle, m.
MIDDLE = (BALL - HEEL) / 2.0


def state():
    """Nothing released: each sole's time borne light, s; whether it is released; its yaw and its
    way off the others as it left."""
    return {'light': [0.0, 0.0], 'air': [False, False], 'yaw': [0.0, 0.0],
            'off': [(0.0, 0.0), (0.0, 0.0)]}


def released(st, meant, loads, bears, rising, dt):
    """Each sole's release moved on a step, a bool a sole: `meant` the ask's stance, `loads` the
    soles' N, `bears` what the stack laid on each, `rising` her centre of mass's speed up."""
    stood = []
    for k in (0, 1):
        if not meant[k]:
            st['light'][k], st['air'][k] = 0.0, False
        elif st['air'][k]:
            st['air'][k], st['light'][k] = loads[k] < LAND_N or rising > RISES_M_S, 0.0
        else:
            st['light'][k] = st['light'][k] + dt if loads[k] < RELEASE_N else 0.0
            stood.append(k)
    left = bool(stood) and all(st['light'][k] >= RELEASE_S for k in stood)
    for k in stood:
        st['air'][k] = left or (st['light'][k] >= LOST_S and bears[k] > 2.0 * RELEASE_N)
    return tuple(st['air'])


def landing(limb, hip, heading, yaw, com, vcom, off):
    """(sole's middle, speed, acceleration, quaternion), the stack's swing ask for a released
    sole: under her capture point as she lands - from her hip the same whenever that is -, `off`
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


def step(st, ask, loads, bears, s, hips, dt=0.001):
    """`ask` as the stack is to be asked it: the soles it means to stand that bear nothing
    released, each asked to its landing. `s` is `wbc.sense`'s; `hips` each limb's hip joint,
    world."""
    meant = ask['stance']
    was = tuple(st['air'])
    air = released(st, meant, loads, bears, float(s['vcom'][1]), dt)
    if not any(air):
        return ask
    soles = [(float(f['sole'][0]), float(f['sole'][2])) for f in s['feet']]
    stood = [k for k in (0, 1) if meant[k]]
    mid = [sum(soles[k][i] for k in stood) / len(stood) for i in (0, 1)]
    w, x, y, z = (float(v) for v in s['turns'][0]['quat'])
    heading = math.atan2(2.0 * (x * z + w * y), 1.0 - 2.0 * (x * x + y * y))
    swing = dict(ask.get('swing', {}))
    for k in (0, 1):
        if not air[k]:
            continue
        if not was[k]:
            R = s['feet'][k]['R']
            st['yaw'][k] = math.atan2(R[0][2], R[2][2])
            st['off'][k] = (soles[k][0] - mid[0], soles[k][1] - mid[1])
        swing[k] = landing(k, hips[k], heading, st['yaw'][k], s['com'], s['vcom'], st['off'][k])
    stance = tuple(bool(meant[k]) and not air[k] for k in (0, 1))
    if not any(stance):
        # nothing to turn her against. Her turns asked even as they were, their rates bled,
        # the stack spun her legs to hold her trunk still: the knees out at 3 rad/s with no
        # torque in them, on their stops as she landed
        return dict(ask, stance=stance, swing=swing, turns=())
    return dict(ask, stance=stance, swing=swing)
