"""Falling past recovery: her legs' and trunk's drives shorted, her arms out toward the fall.

What `machine.director` sets her as she goes down.

    out = falls.reach(tip_deg, rate_deg_s)      # the arms, the neck, the waist's turn
    out = falls.yielded(out, bus)               # an arm at the floor: soft from there
"""
import math

#: Falling, the drives of SHORT_FALLING's kinds have their phases shorted through the low sides
#: (`physics.World.short`), each joint giving kt^2/R of its speed back against it - a damper,
#: not a pose held; the arms and the neck go to CATCH over CURL_S, and softly on into YIELD as an
#: arm lands.
#: Five falls (the hole, the slip, the rug, the stairs, a lace): 2 lay prone on their forearms,
#: 1 on her back and 2 on a side, the head at the floor once, at 0.11 m/s; every drive shorted
#: once down, 4 prone and 1 on her back, the head 0.05-1.14 m/s four times; curled and held as
#: before, 3 on her left side, 1 on her right and 1 prone, 2.29 and 0.25 m/s (2026-09-30).
SHORT_FALLING = ('hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll', 'foot', 'spine',
                 'spine_roll')
CURL_S = 0.4

#: Falling, the arms and the neck by the way she tips and how hard: tipping past GUARD_DEG_S as
#: the fall is declared, the forearms before her face and the chin tucked (`guard`), no arm put
#: out to break; slower, ahead the hands out before her, the elbows soft, the head up HEAD_UP_DEG
#: - with the chin down 40 her head met the floor at 0.85 m/s, up 20, 40 or 60 the chest and the
#: hips took it (2026-09-28); tipping more than BEHIND_DEG from her forward, the arms down behind
#: her and the chin tucked. Declared at 18-341 deg/s: the hole, the lace, the stairs, the rug.
HEAD_UP_DEG, BEHIND_DEG, GUARD_DEG_S = 40.0, 120.0, 150.0
#: Ahead or guarded, the waist turns her torso - the arms, a shoulder's one axis in its plane -
#: toward the way she tips, from none AHEAD_DEG off her front to all of it SIDE_DEG off, WAIST_DEG
#: at most, at once (AT_ONCE: its drive's peak the ease). Turned toward the lace's dive, 23
#: degrees off, her head met the floor at 1.5-2.5 kN in 6 laces of 6, unturned in none
#: (2026-10-01).
AHEAD_DEG, SIDE_DEG, WAIST_DEG, AT_ONCE = 20.0, 35.0, 45.0, ('waist',)
CATCH = {'ahead': {'neck': HEAD_UP_DEG, 'left_shoulder': 90.0, 'right_shoulder': 90.0,
                   'left_elbow': 30.0, 'right_elbow': 30.0, 'left_wrist': 0.0, 'right_wrist': 0.0,
                   'left_gripper': 0.0, 'right_gripper': 0.0},
         'behind': {'left_shoulder': -45.0, 'right_shoulder': -45.0, 'left_elbow': 20.0,
                    'right_elbow': 20.0, 'neck': 45.0},
         'guard': {'neck': 45.0, 'left_shoulder': 115.0, 'right_shoulder': 115.0,
                   'left_elbow': 125.0, 'right_elbow': 125.0, 'left_wrist': 0.0,
                   'right_wrist': 0.0}}

#: An arm at the floor (TOUCH_M), the arms go on into YIELD, the forearms by her face, soft: each
#: asked at most SOFT_DEG past where it is, 13 N m a shoulder and 7 an elbow. Held to the catch
#: they pinned their drives at their peaks in all four falls, the stairs' left straight out
#: (2026-09-30); held out straight she caught herself and toppled over them sideways; down on a
#: lace the head stayed 71 mm off the floor where straight arms left 132; bent to 70, the hands
#: over the head, it met the floor at 1.3 m/s (2026-09-28).
YIELD = {'left_elbow': 90.0, 'right_elbow': 90.0, 'left_shoulder': 110.0, 'right_shoulder': 110.0}
SOFT_DEG, TOUCH_M = 5.0, 0.01
ARM_PARTS = tuple(side + part for side in ('left_', 'right_')
                  for part in ('upper_arm', 'forearm', 'hand', 'fingers'))



def reach(tip, rate):
    """{joint: deg} the arms, the neck and the waist go to, tipping `tip` deg off her forward (+
    to her left) at `rate` deg/s: CATCH's by the way, the waist turned toward it ahead - the
    director turns it on as the tip moves, till an arm lands."""
    way = 'guard' if rate > GUARD_DEG_S else 'behind' if abs(tip) > BEHIND_DEG else 'ahead'
    return dict(CATCH[way]) if way == 'behind' else dict(CATCH[way], waist=turn(tip))


def turn(tip):
    """The waist's turn toward a tip `tip` deg off her forward, deg."""
    k = max(0.0, min(1.0, (abs(tip) - AHEAD_DEG) / (SIDE_DEG - AHEAD_DEG)))
    return math.copysign(min(WAIST_DEG, k * abs(tip)), tip)


def cause(tip, rate, stumbled):
    """What felled her, as the planner reads it."""
    return 'tipped %s at %.0f deg/s%s' % (
        'forward' if abs(tip) < 45.0 else 'back' if abs(tip) > 135.0 else
        'to her left' if tip > 0.0 else 'to her right', rate, ' out of a stumble' if stumbled else '')


def yielded(out, bus):
    """`out` with the arms gone on into YIELD, each at most SOFT_DEG past where it is."""
    return dict(out, **{j: bus[j + '.deg'] + max(-SOFT_DEG, min(SOFT_DEG, v - bus[j + '.deg']))
                        for j, v in YIELD.items()})
