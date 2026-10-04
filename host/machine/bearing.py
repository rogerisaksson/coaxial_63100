"""A keyframe's legs by what they bear: one bearing little reaches from the pelvis as it is.

    y = bearing.height(pinned, frame, turn, bus, y, held)   # no higher than they reach
    bearing.legs(borne, pinned, frame, turn, now, bus, dt, held, out)   # `out`'s legs again

For `machine.arrival`'s player: `borne` {side: how far a stance leg, 0-1} and `pinned` {side:
where a stepping foot landed, lowered till it bears}, both till its next `play`, are its state; `held` whether she
stands to stay (its `stand_s`).
"""
import math

from machine import figure, gait, stance, walkplan
from machine.figure import LEG, mul, rx, ry

#: Standing held, a foot reaches PRESS_M under its mark for what it does not bear: solved from
#: the pelvis as it is it sat on the floor at 0 N (60 N shove, 2026-10-04). A `swing` foot is
#: pinned where it first bears coming down and from there seeks the floor at SEEK_M_S until it
#: bears, SEEK_M at most: pinned alone, touching toes first with the ankle 1.2 cm up, it levelled
#: 2 mm over the floor at 0 N and the capture point ran on (100 N from behind, 2026-10-04).
PRESS_M, SEEK_M_S, SEEK_M = 0.01, 0.3, 0.04

#: How far a leg is a stance leg, by what it bears, moves no faster than BEAR_S from none to all:
#: at once, its load flickering about `stance.LANDED_N` as the foot lifted into the first step
#: switched the leg between target and pelvis, the knee 2-8 deg a pass (2026-10-01).
BEAR_S = 0.05


def foot(frame, pitch):
    """A keyframe's foot's turn: pitched toes-up `pitch` deg, facing its `yaw` - unturned, up
    from a fall facing back, the legs were solved half a turn twisted (2026-09-30)."""
    return mul(ry(math.radians(frame.get('yaw', 0.0))), rx(math.radians(pitch)))


def height(pinned, frame, turn, bus, y, held):
    """`y`, the pelvis's, standing to stay no higher than both legs reach their marks from
    where the hips are: over the front foot after a step, 0.90 m up, the rear leg hung straight
    2.4 cm off the floor and she stood on one foot's toes (80 N from behind). The stepping
    foot's mark left out, or `y` held within 1 cm of the pelvis as it is - the standing knee
    had bent 33 deg under a 5 cm drop asked and lifted its foot off the floor -, the polar
    stood 11-13 of 36 against 14: chance, neither kept (2026-10-04)."""
    if not held:
        return y
    for side, sign in walkplan.SIDES:
        mark = pinned.get(side, frame[side][0])
        h = figure.hip(sign, (bus['pelvis.pose.x'], 0.0, bus['pelvis.pose.z']), turn)
        d = math.hypot(h[0] - mark[0], h[2] - mark[2])
        y = min(y, mark[1] - h[1] + math.sqrt(max(0.0, gait.REACH ** 2 - d * d)))
    return y


def legs(borne, pinned, frame, turn, now, bus, dt, held, out):
    """`out`'s legs solved again by what each bears. One bearing under `stance.LANDED_N` reaches
    from the pelvis as it is, as the walker's swinging leg: from the pelvis's target, moved by
    the feedback, the stepping foot landed 8 cm off (2026-09-26). A keyframe's `planted` feet
    are stance whatever they bear: crouched with her hands down, both read light and she was
    flung up. Its `swing` foot (1 left, -1 right) reaches whatever it bears - stance for 50 ms,
    a standing step pushed her off the other foot - and is `pinned` where it lands, PRESS_M
    under: 2 cm under hopped her 4 cm up (2026-10-04). Standing to stay (`held`) the leg bearing
    the more is a stance leg whatever it bears: on her heels after 80 N from the front both feet
    chattered 0-850 N, neither leg held the pelvis and it pitched 54 deg back in 0.15 s; both
    stance whatever they bore, the free rocker felled her before the nudge (2026-10-04)."""
    pel = (bus['pelvis.pose.x'], bus['pelvis.pose.y'], bus['pelvis.pose.z'])
    loads = {side: bus['pelvis.pose.%s_load' % side] for side, _sign in walkplan.SIDES}
    most = max(loads.values())
    for side, sign in walkplan.SIDES:
        load = loads[side]
        swing = frame.get('swing', 0.0) * sign
        bears = min(1.0, load / stance.LANDED_N)
        b = bears
        if held and most < stance.LANDED_N:
            b = borne.get(side, 1.0) if most < 1.0 else load / most
        b = max(frame.get('planted', 0.0), b) * max(0.0, min(1.0, 1.0 - swing))
        # Landed where it bears coming down (the keyframe's `land`): by load alone a foot hanging
        # on a brick was pinned at once; by its height, on sprung toes pressing 60-157 N as the
        # ankle rose, 2 cm up as it left, never stepping (2026-10-04).
        if (swing > 0.5 and side not in pinned and frame.get('land', 0.0) > 0.5
                and load >= stance.LANDED_N):
            pinned[side] = figure.foot_of(sign, pel, now, tuple(
                math.radians(bus.get(side + k + '.deg', 0.0)) for k in LEG))[0]
        was = borne.get(side, b)
        b = borne[side] = max(was - dt / BEAR_S, min(was + dt / BEAR_S, b))
        if b < 1.0 or side in pinned:
            ankle, pitch = frame[side]
            if side in pinned:
                x, y, z = pinned[side]
                ankle = pinned[side] = (x, max(ankle[1] - SEEK_M, y - SEEK_M_S * dt * (1.0 - bears)), z)
            elif held:
                ankle = (ankle[0], ankle[1] - PRESS_M * (1.0 - b), ankle[2])
            hip_from = tuple(b * a + (1.0 - b) * c for a, c in zip(frame['pelvis'], pel))
            reach = mul(walkplan.turned(tuple(b * c for c in walkplan.vee(
                mul(turn, figure.t(now))))), now)
            for k, v in zip(LEG, figure.leg(sign, hip_from, reach, ankle, foot(frame, pitch))):
                out[side + k] = math.degrees(v)
