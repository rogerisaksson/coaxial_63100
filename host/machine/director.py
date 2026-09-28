"""The gynoid's director: which move sets her joints each pass, and when one hands to the next.

    director = Director(machine, cadence=0.85, walk_s=12.0)   # a DYNAMIC gynoid
    director.begin()                             # landed in the squat
    machine.loop.write(**director.step(dt))      # every pass; `director.stage` says what she does
    director.halt(); director.rise()             # down into the squat, up again

The arrival (`machine.arrival`) takes her from the squat to her first step and hands her to the
walker (`machine.walker`), which sets every step as it comes. Shoved, the walker lands the
swinging foot on the capture point, swapping feet when that would cross them (`catch`); a stance
foot that slides is held where it slid to. Halted, her stride
shortened to her first's, she settles at a landing: onto the front foot, the rear beside it, down
into the squat (`rest`); risen, she walks on. `walk_s` and `rest_s` run that round by
themselves. Falling past recovery, over CURL_S she goes onto all fours turned the way she
tips, or sits down tipping back, and lies as she landed: `begin` lands her again.

Her drives' boards report their heat on the bus (`machine.heat`): warming, her legs ease the
pace. A board whose gates dropped is armed again.
"""
import math

from machine import arrival, figure, gait, heat, walker

#: Falling, past the walker's recovery: the pelvis tipped past FALLING_DEG and tipping on faster
#: than FALLING_DEG_S (the head's gyro), or under FALLING_M, walking. She curls into the squat's
#: joints over CURL_S with the arms out toward the fall (CATCH) and lies as she landed. Fallen -
#: under FALLEN_M or tipped past FALLEN_DEG walking, under SQUAT_FALLEN_M in the arrival's moves
#: - is down. Curled at 35 degrees over 1.5 s, the legs walked on through the fall and she lay
#: with her torso through the floor (2026-09-27).
FALLING_DEG, FALLING_DEG_S, FALLING_M, CURL_S = 12.0, 60.0, 0.65, 0.4
FALLEN_M, FALLEN_DEG, SQUAT_FALLEN_M = 0.55, 35.0, 0.3

#: Falling, the joints she goes to by the way she tips: ahead onto her knees and hands, all
#: fours, the head up (`tools/sim/getup_lab.py`'s push-up); behind into the squat's, the arms
#: down behind her and the chin tucked.
#: Down on all fours her knees go apart, each hip KNEES_APART_DEG out: from the catwalk's in,
#: her feet struck each other as the legs folded, 550 N, and her head met the floor (2026-09-28).
KNEES_APART_DEG = 10.0

CATCH = {'ahead': {'left_hip': -90.0, 'right_hip': -90.0, 'left_knee': 90.0, 'right_knee': 90.0,
                   'left_hip_roll': KNEES_APART_DEG, 'right_hip_roll': KNEES_APART_DEG,
                   'left_ankle': 20.0, 'right_ankle': 20.0, 'spine': 0.0, 'neck': -40.0,
                   'left_shoulder': 90.0, 'right_shoulder': 90.0, 'left_elbow': 10.0,
                   'right_elbow': 10.0, 'left_wrist': 0.0, 'right_wrist': 0.0,
                   'left_gripper': 0.0, 'right_gripper': 0.0},
         'behind': {'left_shoulder': -45.0, 'right_shoulder': -45.0, 'left_elbow': 20.0,
                    'right_elbow': 20.0, 'neck': 45.0}}

#: Tipping more than BEHIND_DEG from her forward she sits down (`CATCH['behind']`); less, the
#: waist turns her toward the way she tips as it turns, TWIST_DEG at most, the arms reaching
#: that way. Five falls (a lace at 300, 400 and 600 N, the hole, the rug), the head's speed at
#: the floor: curled into the squat, the arms out, 0.35 0.10 3.05 1.71 0.32 m/s, rolled over a
#: hand; on all fours 1.02 0.13 0.69 0.58 4.78, the rug onto a hip; turned as she tips, the
#: head down twice, 0.69 and 0.58, the hands first four times (2026-09-28).
BEHIND_DEG, TWIST_DEG = 120.0, 60.0


#: A stance foot bearing `walker.BEARS_N` slid past SLIP_M of where it landed is held where it
#: is; one lifting is not sliding. At 2 cm, the feet's slides under the ankle's drive at 0.65
#: strides/s re-anchored the plan eight times in two seconds and she fell (2026-09-26).
SLIP_M = 0.04

#: From the walk into the settling, the setpoints ease over BLEND_S: the settling's first
#: keyframe is her pose with the pelvis unrolled and unturned.
BLEND_S = 0.3

#: She rises and starts at the walk's own cadence (`gait.CADENCE`) whatever is asked - the start
#: holds there only (2026-09-26) - and walking goes to the asked one at PACE_RATE strides/s a
#: second.
PACE_RATE = 0.1

#: The legs' drives' heat as their boards report it: `spent` of a drive's envelope, 1 at its
#: ceiling, the board derating past `heat.THROTTLE_AT`. Walking, the most spent past EASE_AT
#: eases the pace, to EASE_FLOOR strides/s at EASE_FULL. Walked 60 s at 0.85 strides/s the hips'
#: laminate spent 0.95 and derated to 0.5; at 0.75 and 0.65 it held 0.76 and 0.74; eased, 0.79
#: over 110 s. A knee warmed to 100 C (0.93, derated 0.73) walked on, derated 0.98 2.5 s later;
#: stopped to cool, she fell in the settle as a plain halt does (2026-09-28).
EASE_AT, EASE_FULL, EASE_FLOOR = 0.7, 0.88, 0.65

#: A drive heard with its gates dropped is armed again REARM_S after, the wait doubled up to
#: REARM_MAX_S when it drops within REARM_BACKOFF_S of its last arming; an arming is heard back
#: ARM_LAG_S later.
REARM_S, REARM_BACKOFF_S, REARM_MAX_S, ARM_LAG_S = 0.05, 1.0, 1.6, 0.005

#: Her moments numbered from 1, on the page and in tools/sim/look.py alike, so a seam is named by
#: its two numbers: the squat to the walk, then what the walk may turn to.
MOMENTS = ('squat', 'look', 'push', 'rise', 'stand', 'shift', 'lean', 'step', 'walk', 'catch',
           'halt', 'settle', 'lower', 'rest', 'falling', 'fallen')


def moment(stage):
    """A stage by its moment's number: '7 lean'; one not numbered as it is."""
    return '%d %s' % (MOMENTS.index(stage) + 1, stage) if stage in MOMENTS else stage


class Director:

    """The moves one after another, each pass's setpoints from whichever has her."""

    def __init__(self, machine, cadence=gait.CADENCE, walk_s=None, rest_s=None):
        self.machine, self.asked = machine, float(cadence)
        self.arrival = arrival.Arrival(machine, gait.CADENCE)
        self.walker = walker.Walker(machine, gait.CADENCE)
        self.walk_s, self.rest_s = walk_s, rest_s
        self.stage, self.fallen_at, self.slips = 'squat', None, 0
        self.since, self.blend, self.age = 0.0, None, 0.0
        #: Since when she curls and from and to what; the tilt last pass, (deg, s).
        self.falling_at, self.curl_from, self.curl_to, self.tilt_was = None, {}, {}, None
        self.curled = arrival.angles_of(arrival.keyframes(gait.CADENCE)[0][2])
        #: Each joint's drive by its node's channels, the legs'; a dropped drive's (heard at,
        #: wait) and when each was last armed.
        self.world = machine.nodes['pelvis'].world
        self.drives = {figure.JOINTS[n.index]: n.name + '.angle.' for n in machine.nodes
                       if hasattr(n, 'index')}
        self.legs = [side + k for side, _sign in walker.SIDES for k in figure.LEG + ('_foot',)
                     if side + k in self.drives]
        self.dropped, self.armed = {}, {}

    @property
    def pendulum(self):
        """The pendulum between her ears: how smoothly she goes (`machine.pendulum`); the
        walker's, read by it walking and here otherwise."""
        return self.walker.pendulum

    @property
    def cadence(self):
        """The cadence asked, strides/s; `walker.cadence` is the one she walks at."""
        return self.asked

    @cadence.setter
    def cadence(self, value):
        self.asked = float(value)

    def begin(self):
        """Landed in the squat, the arrival to take her up."""
        self.arrival.land()
        self.stage, self.fallen_at, self.since = self.arrival.stage, None, 0.0
        self.walker.last, self.blend, self.curl_from = None, None, {}
        self.falling_at, self.curl_to, self.tilt_was = None, {}, None
        self.walker.heading = 0.0
        self.walker.pendulum = type(self.walker.pendulum)()
        self.walker.cadence = gait.CADENCE
        self.dropped, self.armed = {}, {}

    def halt(self):
        """Walking, to a stop and down into the squat."""
        if self.stage in ('walk', 'catch'):
            self.walker.halt()
            self.stage = 'halt'

    def spent(self, joints=None):
        """The most spent of `joints`' drives (the legs'), as their boards last said."""
        bus = self.machine.loop.bus
        return max(bus.get(self.drives[j] + 'spent', 0.0) for j in (joints or self.legs))

    def rise(self):
        """Resting in the squat, up and walking again."""
        if self.stage == 'rest':
            bus = self.machine.loop.bus
            feet = [self._foot(bus, side, sign)[0] for side, sign in walker.SIDES]
            self.arrival.rise(sum(f[0] for f in feet) / 2.0, sum(f[2] for f in feet) / 2.0)
            self.stage, self.since = self.arrival.stage, 0.0

    def step(self, dt):
        """{joint: degrees}: what whichever move has her sets now."""
        bus = self.machine.loop.bus
        self._arm(bus)
        if self.stage in arrival.STAGES or self.stage in ('falling', 'fallen'):
            self.pendulum.read(bus, dt)
        if self.falling_at is None and (self._falling(bus) or self._fallen(bus)):
            self.falling_at, self.stage = bus['t'], 'falling'
            way = self._fall_way(bus)
            self.curl_to = dict(self.curled, **CATCH['behind' if abs(way) > BEHIND_DEG
                                                     else 'ahead'])
            self.curl_from = {j: bus.get(j + '.deg', 0.0) for j in self.curl_to}
        if self.fallen_at is None and self._fallen(bus):
            self.fallen_at, self.stage = bus['t'], 'fallen'
        if self.falling_at is not None:
            way = self._fall_way(bus)
            if abs(way) <= BEHIND_DEG and TWIST_DEG > 0.0:
                self.curl_to['waist'] = max(-TWIST_DEG, min(TWIST_DEG, way))
            k = gait.eased((bus['t'] - self.falling_at) / CURL_S)
            return {j: self.curl_from[j] + (v - self.curl_from[j]) * k
                    for j, v in self.curl_to.items()}
        self.since += dt
        if self.stage in arrival.STAGES:
            out = self.arrival.step(dt)
            if self.arrival.stage != self.stage and self.arrival.stage == 'rest':
                self.since = 0.0
            self.stage = self.arrival.stage
            if self.blend is not None:
                self.age += dt
                k = gait.eased(self.age / BLEND_S)
                out = {j: self.blend.get(j, v) + (v - self.blend.get(j, v)) * k
                       for j, v in out.items()}
                if self.age >= BLEND_S:
                    self.blend = None
            if self.stage == 'ready':
                self.walker.begin(out, scale=arrival.FIRST,
                                  ball_ahead=self.walker.ball_ahead('left'), lean=gait.LEAN_DEG)
                self.stage, self.since = 'walk', 0.0
            elif self.stage == 'rest' and self.rest_s is not None and self.since > self.rest_s:
                self.rise()
            self.walker.last = out
            return out
        self._watch(bus)
        hot = self.spent()
        if self.walker.held is None:
            step = PACE_RATE * dt
            pace = self.asked - max(0.0, self.asked - EASE_FLOOR) * min(1.0, max(
                0.0, (hot - EASE_AT) / (EASE_FULL - EASE_AT)))
            self.walker.cadence += max(-step, min(step, pace - self.walker.cadence))
        out = self.walker.step(dt)
        if self.stage == 'halt':
            if self.walker.halted and min(bus['pelvis.pose.left_load'],
                                          bus['pelvis.pose.right_load']) > walker.LANDED_N:
                now = self._now(bus, out)
                ahead = 'left' if now['left'][0][2] >= now['right'][0][2] else 'right'
                self.arrival.settle(now, self._flat(bus, ahead), bus['pelvis.pose.vz'])
                self.stage, self.blend, self.age = self.arrival.stage, out, 0.0
        elif self.walk_s is not None and self.since > self.walk_s:
            self.halt()
        self.walker.last = out
        return out

    def _arm(self, bus):
        """Each drive heard with its gates dropped armed again when its wait is out."""
        t = bus['t']
        for joint, name in self.drives.items():
            if int(bus.get(name + 'status', 1)) & heat.GATES_ON or (
                    t - self.armed.get(joint, (-1e9, 0.0))[0] < ARM_LAG_S):
                continue
            if joint not in self.dropped:
                at, wait = self.armed.get(joint, (-1e9, REARM_S / 2.0))
                wait = min(REARM_MAX_S, 2.0 * wait) if t - at < REARM_BACKOFF_S else REARM_S
                self.dropped[joint] = (t, wait)
            elif t - self.dropped[joint][0] >= self.dropped[joint][1]:
                self.world.arm(figure.JOINTS.index(joint))
                self.armed[joint] = (t, self.dropped.pop(joint)[1])

    def _tilt(self, bus):
        """The pelvis's tilt from upright, degrees."""
        up = self._pelvis(bus)[1][1][1]
        return math.degrees(math.acos(max(-1.0, min(1.0, up))))

    def _falling(self, bus):
        """Past recovery, walking: tipped past FALLING_DEG and tipping on faster than
        FALLING_DEG_S, or the pelvis under FALLING_M."""
        if self.stage in arrival.STAGES:
            self.tilt_was = None
            return False
        tilt, t = self._tilt(bus), bus['t']
        rate = 0.0 if self.tilt_was is None else (
            (tilt - self.tilt_was[0]) / max(1e-6, t - self.tilt_was[1]))
        self.tilt_was = (tilt, t)
        return bus['pelvis.pose.y'] < FALLING_M or (tilt > FALLING_DEG and rate > FALLING_DEG_S)

    def _fall_way(self, bus):
        """Which way the pelvis tips, deg about the vertical from its own forward, + to its
        left."""
        turn = self._pelvis(bus)[1]
        up, ahead, left = ((turn[0][k], turn[2][k]) for k in (1, 2, 0))
        return math.degrees(math.atan2(up[0] * left[0] + up[1] * left[1],
                                       up[0] * ahead[0] + up[1] * ahead[1]))

    def _fallen(self, bus):
        """Down: the pelvis under its floor for what she does, or tipped past FALLEN_DEG
        walking."""
        if self.stage in arrival.STAGES:
            return bus['pelvis.pose.y'] < SQUAT_FALLEN_M
        return bus['pelvis.pose.y'] < FALLEN_M or self._tilt(bus) > FALLEN_DEG

    def _foot(self, bus, side, sign):
        """(ankle, the foot's pitch toes-up deg) of a leg as the loop read it, world."""
        angles = tuple(math.radians(bus.get(side + k + '.deg', 0.0)) for k in figure.LEG)
        ankle, foot = figure.foot_of(sign, *self._pelvis(bus), angles)
        return ankle, math.degrees(math.atan2(foot[2][1], foot[1][1]))

    def _flat(self, bus, side):
        """A leg's ankle, world, its foot laid flat about the end of its sole that is down."""
        ankle, foot = figure.foot_of(1.0 if side == 'left' else -1.0, *self._pelvis(bus),
                                     tuple(math.radians(bus.get(side + k + '.deg', 0.0))
                                           for k in figure.LEG))
        heel, ball = (figure.add(ankle, figure.apply(foot, p))
                      for p in (figure.SOLE_HEEL, figure.SOLE_BALL))
        if heel[1] < ball[1]:
            return (heel[0], gait.ANKLE_H, heel[2] + gait.HEEL)
        return (ball[0], gait.ANKLE_H, ball[2] - gait.BALL)

    def _pelvis(self, bus):
        """(place, turn) of the pelvis as the loop read it."""
        return ((bus['pelvis.pose.x'], bus['pelvis.pose.y'], bus['pelvis.pose.z']),
                figure.quat(bus['pelvis.pose.qw'], bus['pelvis.pose.qx'],
                             bus['pelvis.pose.qy'], bus['pelvis.pose.qz']))

    def _now(self, bus, out):
        """Her pose as the loop read it, as a keyframe: the pelvis tipped but not rolled or
        turned, the feet where they stand, the upper body as last set."""
        pel, turn = self._pelvis(bus)
        frame = {'pelvis': pel, 'tilt': math.degrees(math.atan2(turn[2][1], turn[1][1])),
                 'joints': {j: out[j] for j in walker.UPPER if j in out}}
        for side, sign in walker.SIDES:
            frame[side] = self._foot(bus, side, sign)
        return frame

    def _watch(self, bus):
        """The stage as the walker has her; a slid foot held where it went."""
        if self.stage != 'halt':
            self.stage = 'catch' if self.walker.catching else 'walk'
        for side, ball in self.walker.balls.items():
            held = self.walker.anchor.get(side)
            if held is None or bus['pelvis.pose.%s_load' % side] < walker.BEARS_N:
                continue
            if (ball[0] - held[0]) ** 2 + (ball[2] - held[2]) ** 2 > SLIP_M ** 2:
                self.walker.anchor[side] = ball
                self.slips += 1
