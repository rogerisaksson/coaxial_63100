"""The gynoid's director: which move sets her joints each pass, and when one hands to the next.

    director = Director(machine, cadence=0.85, walk_s=12.0)   # a DYNAMIC gynoid
    director.begin()                             # landed in the squat
    machine.loop.write(**director.step(dt))      # every pass; `director.stage` says what she does
    director.halt(); director.rise()             # down into the squat, up again

The arrival (`machine.arrival`) takes her from the squat to her first step and hands her to the
walker (`machine.walker`), which sets every step as it comes. A shove or a stumble - her pelvis
thrown sideways or tipped past CATCH - makes the walker put the swinging foot where she is
falling, sooner (`catch`); a stance foot that slides is held where it slid to. Halted, her stride
shortened to her first's, she settles at a landing: onto the front foot, the rear beside it, down
into the squat (`rest`); risen, she walks on. `walk_s` and `rest_s` run that round by
themselves. Fallen, she curls up into the squat's joints over CURL_S and stays down: `begin`
lands her again.
"""
import math

from machine import arrival, figure, gait, walker

#: Fallen: the pelvis under FALLEN_M or tipped past FALLEN_DEG walking, under SQUAT_FALLEN_M in
#: the arrival's moves; curled up into the squat's joints over CURL_S.
FALLEN_M, FALLEN_DEG, SQUAT_FALLEN_M, CURL_S = 0.55, 35.0, 0.3, 1.5

#: A catch: the pelvis's sideways speed past CATCH_M_S or its attitude CATCH_DEG off the plan's
#: (`walker.Walker.tilt_deg`); for CATCH_S
#: the swinging foot goes CATCH_STEP further the way she moves, the phase CATCH_PACE faster.
CATCH_M_S, CATCH_DEG, CATCH_S, CATCH_STEP, CATCH_PACE = 0.25, 8.0, 0.6, 0.25, 0.4

#: A stance foot bearing `walker.BEARS_N` slid past SLIP_M of where it landed is held where it
#: is; one lifting is not sliding.
SLIP_M = 0.02

#: From the walk into the settling, the setpoints ease over BLEND_S: the settling's first
#: keyframe is her pose with the pelvis unrolled and unturned.
BLEND_S = 0.3

#: She rises and starts at the walk's own cadence (`gait.CADENCE`) whatever is asked - the start
#: holds there only (2026-09-26) - and walking goes to the asked one at PACE_RATE strides/s a
#: second.
PACE_RATE = 0.1


class Director:

    """The moves one after another, each pass's setpoints from whichever has her."""

    def __init__(self, machine, cadence=gait.CADENCE, walk_s=None, rest_s=None):
        self.machine, self.asked = machine, float(cadence)
        self.arrival = arrival.Arrival(machine, gait.CADENCE)
        self.walker = walker.Walker(machine, gait.CADENCE)
        self.walk_s, self.rest_s = walk_s, rest_s
        self.stage, self.fallen_at, self.caught, self.slips = 'squat', None, 0.0, 0
        self.since, self.blend, self.age = 0.0, None, 0.0
        self.curl_from = None
        self.curled = arrival.angles_of(arrival.keyframes(gait.CADENCE)[0][2])

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
        self.stage, self.fallen_at, self.caught, self.since = self.arrival.stage, None, 0.0, 0.0
        self.walker.last, self.blend, self.curl_from = None, None, None
        self.walker.pendulum = type(self.walker.pendulum)()
        self.walker.cadence = gait.CADENCE

    def halt(self):
        """Walking, to a stop and down into the squat."""
        if self.stage in ('walk', 'catch'):
            self.walker.halt()
            self.stage = 'halt'

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
        if self.stage in arrival.STAGES or self.stage == 'fallen':
            self.pendulum.read(bus, dt)
        if self.fallen_at is None and self._fallen(bus):
            self.fallen_at, self.stage = bus['t'], 'fallen'
            self.curl_from = {j: bus.get(j + '.deg', 0.0) for j in self.curled}
        if self.fallen_at is not None:
            k = gait.eased((bus['t'] - self.fallen_at) / CURL_S)
            was = self.curl_from or self.curled
            return {j: was[j] + (v - was[j]) * k for j, v in self.curled.items()}
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
                self.walker.begin(out, scale=arrival.FIRST, phase=arrival.READY_AT,
                                  wide=walker.STAND_WIDE_M)
                self.stage, self.since = 'walk', 0.0
            elif self.stage == 'rest' and self.rest_s is not None and self.since > self.rest_s:
                self.rise()
            self.walker.last = out
            return out
        self._watch(bus, dt)
        if self.walker.held is None:
            step = PACE_RATE * dt
            self.walker.cadence += max(-step, min(step, self.asked - self.walker.cadence))
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

    def _fallen(self, bus):
        """Down: the pelvis under its floor for what she does, or tipped past FALLEN_DEG
        walking."""
        if self.stage in arrival.STAGES:
            return bus['pelvis.pose.y'] < SQUAT_FALLEN_M
        up = self._pelvis(bus)[1][1][1]
        return (bus['pelvis.pose.y'] < FALLEN_M
                or math.degrees(math.acos(max(-1.0, min(1.0, up)))) > FALLEN_DEG)

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

    def _watch(self, bus, dt):
        """A catch when she is thrown; a slid foot held where it went."""
        tilt = self.walker.tilt_deg()
        # Not while the walk starts: the stand's pose is off the plan's by its turn, and every
        # start was a catch that threw her sideways (2026-09-26).
        if self.walker.held is None and (abs(self.walker.v_side) > CATCH_M_S
                                         or tilt > CATCH_DEG):
            self.caught = CATCH_S
        self.caught = max(0.0, self.caught - dt)
        self.walker.catch = CATCH_STEP if self.caught > 0.0 else 0.0
        self.walker.hurry = CATCH_PACE if self.caught > 0.0 else 0.0
        if self.stage != 'halt':
            self.stage = 'catch' if self.caught > 0.0 else 'walk'
        for side, ball in self.walker.balls.items():
            held = self.walker.anchor.get(side)
            if held is None or bus['pelvis.pose.%s_load' % side] < walker.BEARS_N:
                continue
            if (ball[0] - held[0]) ** 2 + (ball[2] - held[2]) ** 2 > SLIP_M ** 2:
                self.walker.anchor[side] = ball
                self.slips += 1
