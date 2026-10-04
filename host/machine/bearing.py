"""A keyframe's legs by what they bear: standing to stay, each foot bears its share of her.

    y = bearing.height(borne, pinned, frame, turn, bus, y, held)   # no higher than they reach
    bearing.legs(borne, pinned, frame, turn, now, bus, dt, held, out)   # `out`'s legs again

For `machine.arrival`'s player: `borne` {side: how far a stance leg, 0-1; side + ' let': how far
its foot is let down under its mark, m} and `pinned` {side: where a stepping foot landed}, both
till its next `play`, are its state; `held` whether she stands to stay (its `stand_s`).
"""
import math

from machine import figure, gait, stance, walkplan
from machine.figure import LEG, mul, rx, ry

#: A stepping foot in the air reaches PRESS_M under its mark: solved from the pelvis as it is, it
#: sat on the floor at 0 N (60 N shove, 2026-10-04).
PRESS_M = 0.01

#: Standing to stay, each foot bears its share (`shared`): let down at SHARE_M_S a share of her
#: weight it lacks and drawn up as fast a share too many, within SHARE_M of its mark, easing
#: home at SHARE_LEAK a second. Against the leg bearing the more a stance leg and a landed foot
#: seeking the floor: the stand suite 99.3 and 89.2 % against 237.9 and 77.5, the push polar 32
#: of 48 against 25 - 100 N 9 of 12 against 3 -, the bricks 96 and 63 % against 60 and 55, the
#: rockers 100 and 93 against 63 and 90. Its forms on the polar: the share what the capture
#: point asks, 24; a step run as before it, 18-20; the standing foot's let-down held through
#: the step, 30 (2026-10-04, docs/findings/standing.md).
SHARE_M_S, SHARE_LEAK, SHARE_M = 0.3, 0.5, (-0.04, 0.08)

#: How far a leg is a stance leg, by what it bears, moves no faster than BEAR_S from none to all:
#: at once, its load flickering about `stance.LANDED_N` as the foot lifted into the first step
#: switched the leg between target and pelvis, the knee 2-8 deg a pass (2026-10-01).
BEAR_S = 0.05


def foot(frame, pitch):
    """A keyframe's foot's turn: pitched toes-up `pitch` deg, facing its `yaw` - unturned, up
    from a fall facing back, the legs were solved half a turn twisted (2026-09-30)."""
    return mul(ry(math.radians(frame.get('yaw', 0.0))), rx(math.radians(pitch)))


def height(borne, pinned, frame, turn, bus, y, held):
    """`y`, the pelvis's, standing to stay no higher than both legs reach their marks - let
    down as they are - from where the hips are: over the front foot after a step, 0.90 m up,
    the rear leg hung straight 2.4 cm off the floor and she stood on one foot's toes (80 N
    from behind). The stepping foot's mark left out, or `y` held within 1 cm of the pelvis as
    it is, the polar stood 11-13 of 36 against 14: chance, neither kept (2026-10-04)."""
    if not held:
        return y
    for side, sign in walkplan.SIDES:
        mark = pinned.get(side, frame[side][0])
        h = figure.hip(sign, (bus['pelvis.pose.x'], 0.0, bus['pelvis.pose.z']), turn)
        d = math.hypot(h[0] - mark[0], h[2] - mark[2])
        y = min(y, mark[1] - borne.get(side + ' let', 0.0) - h[1]
                + math.sqrt(max(0.0, gait.REACH ** 2 - d * d)))
    return y


def shared(borne, pinned, frame, turn, now, bus, dt, out):
    """Standing to stay, `out`'s legs: each a stance leg, its foot let down or drawn up until
    it bears its share of her weight - what her centre of mass asks of it between the two
    marks, all of it while the other steps. A keyframe's `swing` foot (1 left, -1 right)
    reaches from the pelvis as it is, PRESS_M under its mark, until it lands - bearing
    `stance.LANDED_N` as it comes down (the keyframe's `land`; by load alone a foot hanging on
    a brick was landed at once, by its height one leaving on sprung toes that still pressed
    60-157 N, 2 cm up and never stepping, 2026-10-04) - and is `pinned` there, its mark."""
    pel = (bus['pelvis.pose.x'], bus['pelvis.pose.y'], bus['pelvis.pose.z'])
    loads = {side: bus['pelvis.pose.%s_load' % side] for side, _sign in walkplan.SIDES}
    total = max(sum(loads.values()), 100.0)
    air = {}
    for side, sign in walkplan.SIDES:
        swing = frame.get('swing', 0.0) * sign
        if (swing > 0.5 and side not in pinned and frame.get('land', 0.0) > 0.5
                and loads[side] >= stance.LANDED_N):
            pinned[side] = figure.foot_of(sign, pel, now, tuple(
                math.radians(bus.get(side + k + '.deg', 0.0)) for k in LEG))[0]
        air[side] = swing > 0.5 and side not in pinned
    marks = {side: pinned.get(side, frame[side][0]) for side, _sign in walkplan.SIDES}
    a, b = marks['left'], marks['right']
    dx, dz = b[0] - a[0], b[2] - a[2]
    span = dx * dx + dz * dz
    u = max(0.0, min(1.0, ((bus['pelvis.pose.com_x'] - a[0]) * dx
                           + (bus['pelvis.pose.com_z'] - a[2]) * dz) / span)) if span else 0.5
    for side, sign in walkplan.SIDES:
        ankle, pitch = frame[side]
        w = max(0.0, min(1.0, 1.0 - frame.get('swing', 0.0) * sign))
        if air[side]:
            borne[side + ' let'] = 0.0
            ankle = (ankle[0], ankle[1] - PRESS_M, ankle[2])
        else:
            share = 1.0 if air[walkplan._OTHER[side]] else u if side == 'right' else 1.0 - u
            e = max(-1.0, min(1.0, share - loads[side] / total))
            s = borne.get(side + ' let', 0.0)
            s = borne[side + ' let'] = max(SHARE_M[0], min(SHARE_M[1], s + (
                SHARE_M_S * e - SHARE_LEAK * s) * dt))
            ankle = (marks[side][0], marks[side][1] - s, marks[side][2])
        hip_from = tuple(w * p + (1.0 - w) * c for p, c in zip(frame['pelvis'], pel))
        reach = mul(walkplan.turned(tuple(w * c for c in walkplan.vee(
            mul(turn, figure.t(now))))), now)
        for k, v in zip(LEG, figure.leg(sign, hip_from, reach, ankle, foot(frame, pitch))):
            out[side + k] = math.degrees(v)


def legs(borne, pinned, frame, turn, now, bus, dt, held, out):
    """`out`'s legs solved again by what each bears: standing to stay (`held`), `shared`. Else
    one bearing under `stance.LANDED_N` reaches from the pelvis as it is, as the walker's
    swinging leg: from the pelvis's target, moved by the feedback, the stepping foot landed 8
    cm off (2026-09-26). A keyframe's `planted` feet are stance whatever they bear: crouched
    with her hands down, both read light and she was flung up."""
    if held:
        return shared(borne, pinned, frame, turn, now, bus, dt, out)
    pel = (bus['pelvis.pose.x'], bus['pelvis.pose.y'], bus['pelvis.pose.z'])
    for side, sign in walkplan.SIDES:
        b = min(1.0, max(frame.get('planted', 0.0),
                         bus['pelvis.pose.%s_load' % side] / stance.LANDED_N))
        was = borne.get(side, b)
        b = borne[side] = max(was - dt / BEAR_S, min(was + dt / BEAR_S, b))
        if b < 1.0:
            ankle, pitch = frame[side]
            hip_from = tuple(b * a + (1.0 - b) * c for a, c in zip(frame['pelvis'], pel))
            reach = mul(walkplan.turned(tuple(b * c for c in walkplan.vee(
                mul(turn, figure.t(now))))), now)
            for k, v in zip(LEG, figure.leg(sign, hip_from, reach, ankle, foot(frame, pitch))):
                out[side + k] = math.degrees(v)
