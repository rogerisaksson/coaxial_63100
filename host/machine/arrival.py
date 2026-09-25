"""The gynoid's arrival: huddled in a deep squat as a T-800 lands, rising, stepping off into the walk.

    arrival = Arrival(machine)             # a DYNAMIC gynoid
    arrival.land()                         # the body placed in the squat, still
    machine.loop.write(**arrival.step(dt)) # every pass; `arrival.stage` names where she is

Keyframes in the task space - the pelvis and its tilt, each ankle and its foot's pitch, the
spine, head and arms - eased from one to the next, the legs by IK; her centre of mass held to
each keyframe's own, the pelvis's target moved against its error. She rises over her feet, shifts
onto the left and swings the right a short step ahead, falling forward over the left foot's ball
as it lands: `ready` for `machine.walker` to take her on at the landing. On a
knee, four points on the floor fought the feedback and threw her; from a squat her feet carry her
(2026-09-25).
"""
import math
from typing import Any

from machine import figure, gait, walker
from machine.figure import LEG, add, rx

#: Her feet in the squat and standing: the ankles FEET_X either side of the line, at z 0.
FEET_X = 0.08

#: The centre of mass fed back through the pelvis's target: its error, 1, and its speed's, s -
#: against the keyframes' own: against none, it braked her into the settling and she fell back
#: (2026-09-26).
COM_K, COM_D = 1.5, 0.15

#: Entered moving, the first keyframe is reached slowing from her speed, over as long as that
#: takes, within ENTER_S seconds.
ENTER_S = (0.4, 1.5)

#: Stopping, her weight is brought STOP_M ahead of the front ankle, m: over the sole's middle,
#: the rear leg still pushing, she came on over the ball and tipped (2026-09-26).
STOP_M = 0.05

#: The speed's filter, s.
SPEED_S = 0.01

#: Over the feet: the centre of their soles, m ahead of the ankles.
FEET_Z = (gait.BALL - gait.HEEL) / 2.0

#: Onto the left foot, her weight is brought SHIFT_IN inside its ankle, then LIFT_IN as the right
#: lifts: brought 1.5 cm inside at once, it came on at 0.15 m/s as the right foot left the floor,
#: on to the left sole's outer edge, and she fell off it at the first step (2026-09-26).
SHIFT_IN, LIFT_IN = 0.035, 0.015

#: Her first stride, of the walk's (`gait.pace`'s at the walker's cadence x this), and the walk's
#: phase her first step lands at - the right foot's landing - on her standing stance
#: (`walker.STAND_WIDE_M`): on the walk's line she stood on 7 cm across and tipped off it
#: (2026-09-25).
FIRST, READY_AT = 0.6, 0.5

#: Her first step lands with the pelvis FIRST_DOWN under the walk's, m: at the walk's, both legs
#: straight and the rear heel rising, she hopped off the landing (2026-09-26).
FIRST_DOWN = 0.025

#: Her weight FIRST_ONTO of the way across from the left foot to the right as it lands: left over
#: the left, she had no way across to the right and fell off it as the left lifted (2026-09-26).
FIRST_ONTO = 0.8


def _squat(height=0.44, tilt=25.0, spine=45.0, shoulder=60.0, elbow=30.0) -> dict[str, Any]:
    """The arrival as a keyframe: both feet flat, the pelvis `height` up and tipped `tilt`
    forward, the spine bent `spine` more, the head bowed, the right arm's knuckles toward the
    floor by `shoulder` and `elbow`, the left forearm on its knee; her centre of mass over the
    feet."""
    return over({'pelvis': (0.0, height, -0.12), 'tilt': tilt,
                 'left': ((FEET_X, gait.ANKLE_H, 0.0), 0.0),
                 'right': ((-FEET_X, gait.ANKLE_H, 0.0), 0.0),
                 'joints': {'spine': spine, 'neck': 35.0, 'right_shoulder': shoulder,
                            'right_elbow': elbow, 'right_wrist': 0.0, 'right_gripper': 80.0,
                            'left_shoulder': 35.0, 'left_elbow': 70.0, 'left_wrist': 10.0,
                            'left_gripper': 30.0}}, 0.0, FEET_Z)


def angles_of(frame, turn=None):
    """{joint: deg} for a keyframe: its joints, the legs by IK from its pelvis, tipped as it says
    or turned `turn`, to its ankles."""
    turn = turn or rx(math.radians(frame['tilt']))
    out = {j: 0.0 for j in figure.JOINTS}
    out.update(frame['joints'])
    for side, sign in (('left', 1.0), ('right', -1.0)):
        ankle, pitch = frame[side]
        for k, v in zip(LEG, figure.leg(sign, frame['pelvis'], turn, ankle,
                                          rx(math.radians(pitch)))):
            out[side + k] = math.degrees(v)
    return out


def com_of(frame):
    """A keyframe's centre of mass, world."""
    return figure.com(angles_of(frame), frame['pelvis'], rx(math.radians(frame['tilt'])))


def over(frame, x, z, rounds=6) -> dict[str, Any]:
    """`frame` with its pelvis moved until its centre of mass stands over (x, z)."""
    frame = dict(frame)
    for _ in range(rounds):
        c = com_of(frame)
        p = frame['pelvis']
        frame['pelvis'] = (p[0] + x - c[0], p[1], p[2] + z - c[2])
    return frame


def _mix(a, b, k) -> Any:
    if isinstance(a, dict):
        return {key: _mix(a.get(key, 0.0), b.get(key, 0.0), k) for key in set(a) | set(b)}
    if isinstance(a, tuple):
        return tuple(_mix(x, y, k) for x, y in zip(a, b))
    return a + (b - a) * k


def paused(stride, ball, wide=0.0, at=None, on='left') -> dict[str, Any]:
    """The walk's own pose at its phase `at` (READY_AT), at `stride`, its feet `wide` further out a
    side: the ball of the foot `on` on the floor at `ball`, the pelvis and the other foot where the
    walk puts them from there."""
    lateral, height, _yaw, legs, upper, _roll = walker.plan(READY_AT if at is None else at, stride)
    out, pelvis = {}, None
    feet = {side: ((a[0] + sign * wide, a[1], a[2]), pitch, toes)
            for (side, sign), (a, _twist, pitch, toes) in zip(walker.SIDES, legs)}
    ankle, pitch, _toes = feet[on]
    rel = add(ankle, figure.apply(rx(-pitch), figure.SOLE_BALL))
    pelvis = (ball[0] - rel[0], height, ball[2] - rel[2])
    for side, (ankle, pitch, toes) in feet.items():
        out[side] = ((pelvis[0] + ankle[0], ankle[1], pelvis[2] + ankle[2]), math.degrees(-pitch))
        out[side + '_toes'] = toes
    joints = dict(zip(walker.UPPER, upper), spine=0.0, spine_roll=0.0,
                  left_foot=out['left_toes'], right_foot=out['right_toes'])
    return {'pelvis': pelvis, 'tilt': 0.0, 'left': out['left'], 'right': out['right'],
            'joints': joints}


def keyframes(cadence=gait.CADENCE) -> list[tuple[str, float, dict[str, Any]]]:
    """[(stage, seconds to reach it, keyframe)]: the squat, the head up, the hand off the floor,
    rising, standing, onto the left foot, the right swung ahead and landing - `ready`."""
    squat = _squat()
    look = dict(squat, joints=dict(squat['joints'], neck=-30.0))
    push = over(dict(squat, tilt=15.0, joints=dict(
        squat['joints'], spine=30.0, neck=-15.0, right_shoulder=25.0, right_elbow=25.0,
        right_gripper=30.0)), 0.0, FEET_Z)
    rise = over(dict(push, tilt=0.0, pelvis=(0.0, gait.standing()[2] - 0.01, push['pelvis'][2]),
                     joints=dict(push['joints'], spine=0.0, neck=3.0, right_shoulder=0.0,
                                 right_elbow=10.0, right_gripper=18.0, left_shoulder=0.0,
                                 left_elbow=10.0, left_wrist=5.0, left_gripper=18.0)),
                0.0, FEET_Z)
    shift = over(rise, FEET_X - SHIFT_IN, FEET_Z)
    # The right foot swung to where the walk lands it, a step wider, while her weight falls
    # forward over the left foot's ball; it lands as the walk lands it, on its ball, the left heel
    # rising, and the walker takes her on there. Stopped flat-footed in the walk's pose, the front
    # leg could not reach and she fell back; landed flat, the walk tipped both feet at once and
    # she hopped (2026-09-25).
    stride = FIRST * gait.pace(cadence)
    placed = paused(stride, (FEET_X, 0.0, gait.BALL), walker.STAND_WIDE_M)
    front = placed['right'][0]
    placed = dict(placed, pelvis=add(placed['pelvis'], (0.0, -FIRST_DOWN, 0.0)))
    toward = placed['left'][0][0] + FIRST_ONTO * (front[0] - placed['left'][0][0])
    placed = over(placed, toward, com_of(placed)[2])
    lifted = over(dict(shift, right=((front[0], gait.ANKLE_H + 0.06, 0.5 * front[2]), 0.0)),
                  FEET_X - LIFT_IN, gait.BALL - 0.04)
    return [('squat', 0.0, squat), ('squat', 1.5, squat), ('look', 0.8, look),
            ('push', 1.0, push), ('rise', 2.0, rise), ('stand', 1.0, rise), ('shift', 1.2, shift),
            ('step', 0.35, lifted), ('step', 0.3, placed), ('ready', 1e9, placed)]


def moved(frame, dx, dz) -> dict[str, Any]:
    """`frame` moved `dx` sideways and `dz` on along the floor."""
    return dict(frame, pelvis=add(frame['pelvis'], (dx, 0.0, dz)),
                left=(add(frame['left'][0], (dx, 0.0, dz)), frame['left'][1]),
                right=(add(frame['right'][0], (dx, 0.0, dz)), frame['right'][1]))


def settling(now, front, cadence=gait.CADENCE) -> list[tuple[str, float, dict[str, Any]]]:
    """[(stage, seconds, keyframe)] from `now`, a keyframe of her in mid-step on both feet, to the
    squat beside the front foot: her weight onto the front foot, laid flat at `front` (its
    ankle), STOP_M ahead of it as the rear foot swings up, the rear set down beside it on the
    squat's stance, her weight back between them, stood up, the arms down, crouched, squatted -
    `rest`. The arrival's own step, backwards."""
    frames = keyframes(cadence)
    squat, push, rise = (frames[k][2] for k in (0, 3, 4))
    ahead = 'left' if now['left'][0][2] >= now['right'][0][2] else 'right'
    behind = 'right' if ahead == 'left' else 'left'
    sign = 1.0 if ahead == 'left' else -1.0
    x, z = front[0] - sign * FEET_X, front[2]
    beside = (x - sign * FEET_X, gait.ANKLE_H, z)
    lifted = over(dict(now, joints=rise['joints'], tilt=0.0,
                       **{ahead: (front, 0.0), behind: ((beside[0], gait.ANKLE_H + 0.05,
                                                         (now[behind][0][2] + z) / 2.0), 0.0)}),
                  front[0] - sign * 0.015, z + STOP_M)
    placed = over(dict(lifted, **{behind: (beside, 0.0)}), front[0] - sign * 0.015, z + STOP_M)

    def on_feet(frame):
        return over(moved(frame, x, z), x, z + FEET_Z)
    stood, crouched, squatted = on_feet(rise), on_feet(push), on_feet(squat)
    return [('settle', 0.0, now), ('settle', 0.6, lifted),
            ('settle', 0.3, placed), ('settle', 0.8, stood), ('lower', 1.5, crouched),
            ('lower', 1.2, squatted), ('rest', 1e9, squatted)]


#: The arrival's stages, in order - the walker has her after `ready` - and the settling's.
STAGES = ('squat', 'look', 'push', 'rise', 'stand', 'shift', 'step', 'ready', 'settle', 'lower',
          'rest')


class Arrival:

    """The squat, the rise, the first step: `step` sets every joint each pass, from the keyframes
    and her centre of mass as the loop read it."""

    def __init__(self, machine, cadence=gait.CADENCE):
        self.machine, self.cadence = machine, cadence
        self.world = machine.nodes['pelvis'].world
        self.play(keyframes(cadence))

    def play(self, frames, speed=0.0):
        """From the first of `frames` [(stage, seconds, keyframe)] on, from now; moving on at
        `speed`, m/s, the second is reached slowing from it (`ENTER_S`)."""
        self.frames = list(frames)
        self.coms = [com_of(frame) for _stage, _s, frame in frames]
        self.entering = speed > 0.0
        if self.entering:
            stage, _span, frame = self.frames[1]
            span = 2.0 * abs(self.coms[1][2] - self.coms[0][2]) / speed
            self.frames[1] = (stage, min(ENTER_S[1], max(ENTER_S[0], span)), frame)
        self.t, self.stage, self.com_was, self.v = 0.0, frames[0][0], None, (0.0, speed)
        self.want_was = None

    def land(self):
        """The body placed in the squat, still, a hair over the floor."""
        self.play(keyframes(self.cadence))
        frame = self.frames[0][2]
        h = math.radians(frame['tilt']) / 2.0
        self.world.reset(angles_of(frame), where=add(frame['pelvis'], (0.0, 0.002, 0.0)),
                         turn=(math.cos(h), math.sin(h), 0.0, 0.0))

    def rise(self, dx, dz):
        """Up again from the squat `dx` sideways and `dz` on of where she landed."""
        self.play([(stage, s, moved(frame, dx, dz))
                   for stage, s, frame in keyframes(self.cadence)])

    def settle(self, now, front, speed):
        """Down into the squat from `now`, a keyframe of her mid-step moving on at `speed`, m/s
        (`settling`)."""
        self.play(settling(now, front, self.cadence), speed)

    def _at(self):
        """(stage, keyframe, its centre of mass) now: between two keyframes, eased - entered
        moving, the first slowing from the start."""
        t = self.t
        for i, ((stage, span, frame), (_s, _sp, before), c, c0) in enumerate(zip(
                self.frames[1:], self.frames, self.coms[1:], self.coms)):
            if t <= span:
                u = t / span if span > 0.0 else 1.0
                k = 1.0 - (1.0 - u) ** 2 if i == 0 and self.entering else gait.eased(u)
                return stage, _mix(before, frame, k), _mix(c0, c, k)
            t -= span
        return self.frames[-1][0], self.frames[-1][2], self.coms[-1]

    def step(self, dt):
        """{joint: degrees}: where every drive should be now."""
        self.t += dt
        self.stage, frame, want = self._at()
        bus = self.machine.loop.bus
        com = (bus['pelvis.pose.com_x'], bus['pelvis.pose.com_z'])
        if self.com_was is not None:
            k = min(1.0, dt / SPEED_S)
            self.v = tuple(v + ((c - w) / dt - v) * k
                           for v, c, w in zip(self.v, com, self.com_was))
        self.com_was = com
        v_want = (0.0, 0.0) if self.want_was is None else (
            (want[0] - self.want_was[0]) / dt, (want[2] - self.want_was[2]) / dt)
        self.want_was = want
        p = frame['pelvis']
        frame = dict(frame, pelvis=(
            p[0] - COM_K * (com[0] - want[0]) - COM_D * (self.v[0] - v_want[0]), p[1],
            p[2] - COM_K * (com[1] - want[2]) - COM_D * (self.v[1] - v_want[1])))
        # The pelvis's attitude turned back past its error, as the walker turns it: held by the
        # legs' servos alone, it tipped back as she rolled onto the stepping foot.
        turn = rx(math.radians(frame['tilt']))
        now = figure.quat(bus['pelvis.pose.qw'], bus['pelvis.pose.qx'], bus['pelvis.pose.qy'],
                           bus['pelvis.pose.qz'])
        err = walker._vee(figure.mul(turn, figure.t(now)))
        turn = figure.mul(walker._turned(tuple(walker.TURN_K * c for c in err)), turn)
        out = angles_of(frame, turn)
        # A leg bearing under `walker.LANDED_N` reaches from where the pelvis is, as the walker's
        # swinging leg: reached from the pelvis's target, moved by the feedback, the stepping foot
        # landed 8 cm off its mark (2026-09-26).
        pel = (bus['pelvis.pose.x'], bus['pelvis.pose.y'], bus['pelvis.pose.z'])
        for side, sign in walker.SIDES:
            b = min(1.0, bus['pelvis.pose.%s_load' % side] / walker.LANDED_N)
            if b < 1.0:
                ankle, pitch = frame[side]
                hip_from = tuple(b * a + (1.0 - b) * c for a, c in zip(frame['pelvis'], pel))
                reach = figure.mul(walker._turned(tuple(b * c for c in walker._vee(
                    figure.mul(turn, figure.t(now))))), now)
                for k, v in zip(LEG, figure.leg(sign, hip_from, reach, ankle,
                                                  rx(math.radians(pitch)))):
                    out[side + k] = math.degrees(v)
        return out
