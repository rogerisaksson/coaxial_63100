"""Falling past recovery: her legs' and trunk's drives shorted, her arms out toward the fall.

What `machine.director` sets her as she goes down.

    out = falls.reach(tip_deg, rate_deg_s)      # the arms, the neck, the waist's turn
    out = falls.yielded(out, bus)               # an arm at the floor: soft from there
    out = falls.tucked(out, bus, k, lay)        # down, drawn in by k of TUCK_S from `lay`
"""
import math

from machine import gait

#: Falling, the drives of SHORT_FALLING's kinds have their phases shorted through the low sides
#: (`physics.World.short`), each joint giving kt^2/R of its speed back against it - a damper,
#: not a pose held; the arms and the neck go to CATCH over CURL_S, and softly on into YIELD as an
#: arm lands.
#: Five falls (the hole, the slip, the rug, the stairs, a lace): 2 lay prone on their forearms,
#: 1 on her back and 2 on a side, the head at the floor once, at 0.11 m/s; every drive shorted
#: once down, 4 prone and 1 on her back, the head 0.05-1.14 m/s four times; curled and held as
#: before, 3 on her left side, 1 on her right and 1 prone, 2.29 and 0.25 m/s (2026-09-30). Over
#: 0.4 s the arms were 65 % into their catch as the hands met the floor 0.27 s into P's shove at
#: phase 0.14, the head at 1.74 m/s 0.2 s on; over 0.25, 3.22 (2026-10-04).
SHORT_FALLING = ('hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll', 'foot', 'spine',
                 'spine_roll')
CURL_S = 0.4

#: Falling, the arms and the neck by the way she tips and how hard: tipping past GUARD_DEG_S as
#: the fall is declared, the forearms before her face and the chin tucked (`guard`), no arm put
#: out to break; slower, ahead the hands out before her, the elbows soft, the head up HEAD_UP_DEG
#: - with the chin down 40 her head met the floor at 0.85 m/s, up 20, 40 or 60 the chest and the
#: hips took it (2026-09-28); tipping more than BEHIND_DEG from her forward, the arms down behind
#: her and the chin tucked. Declared at 18-341 deg/s: the hole, the lace, the stairs, the rug.
HEAD_UP_DEG, BEHIND_DEG, GUARD_DEG_S, AIM_DEG = 40.0, 120.0, 150.0, 12.0
#: Ahead or guarded, the waist turns her torso - the arms, a shoulder's one axis in its plane -
#: toward the way she tips, from none AHEAD_DEG off her front to all of it SIDE_DEG off, WAIST_DEG
#: at most, at once (AT_ONCE: its drive's peak the ease). Turned toward the lace's dive, 23
#: degrees off, her head met the floor at 1.5-2.5 kN in 6 laces of 6, unturned in none
#: (2026-10-01).
AHEAD_DEG, SIDE_DEG, WAIST_DEG = 20.0, 35.0, 45.0
AT_ONCE = ('waist', 'spine', 'spine_roll') + tuple(side + j for side in ('left_', 'right_')
                                                  for j in ('hip', 'knee', 'ankle', 'hip_roll'))
CATCH = {'ahead': {'neck': HEAD_UP_DEG, 'left_shoulder': 90.0, 'right_shoulder': 90.0,
                   'left_elbow': 30.0, 'right_elbow': 30.0, 'left_wrist': 0.0, 'right_wrist': 0.0,
                   'left_gripper': 0.0, 'right_gripper': 0.0},
         'behind': {'left_shoulder': -45.0, 'right_shoulder': -45.0, 'left_elbow': 20.0,
                    'right_elbow': 20.0, 'neck': 45.0},
         'guard': {'neck': 45.0, 'left_shoulder': 115.0, 'right_shoulder': 115.0,
                   'left_elbow': 125.0, 'right_elbow': 125.0, 'left_wrist': 0.0,
                   'right_wrist': 0.0}}

#: An arm at the floor (TOUCH_M), the arms go on into YIELD, the forearms by her face, soft: each
#: asked at most SOFT_DEG past where it is, 26 N m a shoulder and 14 an elbow - at 5, 13 and 7,
#: a shove's arms folded 30 -> 157 deg in 0.15 s and her head met the floor at 2.46 m/s, at 10
#: and 15 in none of 16 (2026-10-02). Held to the catch
#: they pinned their drives at their peaks in all four falls, the stairs' left straight out
#: (2026-09-30); held out straight she caught herself and toppled over them sideways; down on a
#: lace the head stayed 71 mm off the floor where straight arms left 132; bent to 70, the hands
#: over the head, it met the floor at 1.3 m/s (2026-09-28).
YIELD = {'left_elbow': 90.0, 'right_elbow': 90.0, 'left_shoulder': 110.0, 'right_shoulder': 110.0}
SOFT_DEG, TOUCH_M = 10.0, 0.01

#: Past saving she goes down into a crouch, not limp (CROUCH): the leg on the side she tips to
#: lunges, its hip rolled out by how far aside she tips, LUNGE's; the other kneels, KNEEL's; the
#: spine CROUCH_SPINE forward - driven at once, shorted only once she is down (`machine.director`).
#: Shoved past saving 16 times: limp, her peak on the floor 8.7 kN at the median, 11.0 the worst,
#: a thigh first 8 times; crouched 6.2 and 10.0, a shank first 14; the crouch held on once down,
#: soft, 6.7 and 12.0, her head down once (2026-10-01).
CROUCH, CROUCH_SPINE = True, 25.0
LUNGE = {'hip': -70.0, 'knee': 90.0, 'ankle': -20.0, 'hip_roll': 25.0}
KNEEL = {'hip': -20.0, 'knee': 120.0, 'ankle': 30.0}
ARM_PARTS = tuple(side + part for side in ('left_', 'right_')
                  for part in ('upper_arm', 'forearm', 'hand', 'fingers'))



#: Down, its blow past by TUCK_AFTER_S, she draws in over TUCK_S (TUCK): the forearms before her
#: face, the hips and knees folded, the spine curled, the head level - each joint asked at most
#: TUCK_SOFT_DEG past where it is, the floor never fought - so tumbling on, down a slope, nothing
#: is caught out to break; her shorted drives armed again for it. Lying, her arms stood 686 mm
#: out from her chest after the P shove, 505-522 walking (`look.py`'s hands out). Drawn in
#: 0.2 s after she was down her head met the floor 7 times of 16, at up to 3.6 kN; 0.5, 5 and
#: 0.8 kN; 1.0, 4 and 0.5 kN, as untucked; its chin in 45 degrees, lying on a side, rolled
#: her head to the floor at 1.01 m/s, in 20 at 0.42, level at none past the landing's 0.72
#: (2026-10-01). Drawn from the crouch's asked pose, a hip the floor held at -29 went to -75 in
#: 0.2 s and rolled her head to the floor at 1.39 m/s; from where she lay, 0.90 (2026-10-03).
TUCKED, TUCK_AFTER_S, TUCK_S, TUCK_SOFT_DEG = 1.0, 1.0, 0.6, 3.0
TUCK = dict({'neck': 0.0, 'spine': 30.0, 'spine_roll': 0.0, 'waist': 0.0},
            **{side + j: v for side in ('left_', 'right_') for j, v in (
                ('shoulder', 110.0), ('elbow', 125.0), ('wrist', 0.0), ('gripper', 20.0),
                ('hip_yaw', 0.0), ('hip_roll', 0.0), ('hip', -80.0), ('knee', 110.0),
                ('ankle', 20.0), ('ankle_roll', 0.0))})


def reach(tip, rate):
    """{joint: deg} the arms, the neck and the waist go to, tipping `tip` deg off her forward (+
    to her left) at `rate` deg/s: CATCH's by the way, the waist turned toward it ahead - the
    director turns it on as the tip moves, till an arm lands."""
    way = 'guard' if rate > GUARD_DEG_S else 'behind' if abs(tip) > BEHIND_DEG else 'ahead'
    return dict(CATCH[way]) if way == 'behind' else dict(CATCH[way], waist=turn(tip))


def aimed(to, start, bus, tilt, tip, rate):
    """The curl's `to` and `start` {joint: deg} aimed where she tips now, till an arm lands: the
    waist turned on with it; tipped `tilt` past AIM_DEG, the arms' way read anew (`reach`), and
    True when it changed - they begin again from where they are. Aimed once, as the fall was
    called while she sank upright 3 deg off plumb, the way read behind her, the arms went back
    and she pitched onto her face, her head at 4.31 m/s (P's shove at phase 0.14, 2026-10-04)."""
    if 'waist' in to:
        to['waist'] = turn(tip)
    new = reach(tip, rate) if tilt > AIM_DEG else to
    if new['left_shoulder'] == to.get('left_shoulder'):
        return False
    start.update({j: bus.get(j + '.deg', 0.0) for j in new})
    to.update(new)
    return True


def crouch(tip):
    """{joint: deg} her legs and trunk go to past saving, tipping `tip` deg off her forward (+ to
    her left): the side she tips to lunges, out by how far aside, the other kneels."""
    side, other = ('left_', 'right_') if tip > 0.0 else ('right_', 'left_')
    aside = max(0.0, min(1.0, (abs(tip) - AHEAD_DEG) / (SIDE_DEG - AHEAD_DEG)))
    out = {'spine': CROUCH_SPINE, 'spine_roll': 0.0}
    out.update({side + j: v * (aside if j == 'hip_roll' else 1.0) for j, v in LUNGE.items()})
    out.update({other + j: v for j, v in KNEEL.items()})
    return out


def turn(tip):
    """The waist's turn toward a tip `tip` deg off her forward, deg."""
    k = max(0.0, min(1.0, (abs(tip) - AHEAD_DEG) / (SIDE_DEG - AHEAD_DEG)))
    return math.copysign(min(WAIST_DEG, k * abs(tip)), tip)


def cause(tip, rate, stumbled):
    """What felled her, as the planner reads it."""
    return 'tipped %s at %.0f deg/s%s' % (
        'forward' if abs(tip) < 45.0 else 'back' if abs(tip) > 135.0 else
        'to her left' if tip > 0.0 else 'to her right', rate, ' out of a stumble' if stumbled else '')


def tucked(out, bus, k, start):
    """`out` drawn in toward TUCK by `k` of TUCK_S from `start` {joint: deg}, where she lay as it
    began, eased, each joint at most TUCK_SOFT_DEG past where it is."""
    e = gait.eased(max(0.0, min(1.0, k)))
    got = dict(out)
    for j, v in TUCK.items():
        now = bus.get(j + '.deg', 0.0)
        aim = start.get(j, now) + (v - start.get(j, now)) * e
        got[j] = now + max(-TUCK_SOFT_DEG, min(TUCK_SOFT_DEG, aim - now))
    return got


def yielded(out, bus):
    """`out` with the arms gone on into YIELD, each at most SOFT_DEG past where it is."""
    return dict(out, **{j: bus[j + '.deg'] + max(-SOFT_DEG, min(SOFT_DEG, v - bus[j + '.deg']))
                        for j, v in YIELD.items()})
