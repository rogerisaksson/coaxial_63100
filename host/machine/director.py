"""The gynoid's director: which move sets her joints each pass, and when one hands to the next.

    director = Director(machine, cadence=0.85, walk_s=12.0)   # a DYNAMIC gynoid
    director.begin()                             # landed in the squat
    machine.loop.write(**director.step(dt))      # every pass; `director.stage` says what she does
    director.halt(); director.rise()             # down into the squat, up again

The arrival (`machine.arrival`) takes her from the squat to her first step and hands her to the
walker (`machine.walker`), which sets every step as it comes. Shoved, the walker catches
(`machine.landing`); a sliding stance foot is held where it slid. Halted, she settles at a
landing into the squat (`rest`); risen, she walks on. `walk_s` and `rest_s` run that round by
themselves; risen, she stands `stand_s` first, stepping to keep her feet (`machine.stand`).
Falling past recovery, her legs' and trunk's drives are shorted, dampers, while her arms reach
toward the fall; down and still, her drives cut and checked (`machine.down`), she gets up
(`machine.getup`) into a crouch the arrival takes her up from - given up, she lies cut.
"""
import math
from concurrent.futures import ThreadPoolExecutor

from machine import (arrival, down, drives, falls, figure, gait, getup, heat, observer, planner,
                     stance, stand, walker, walkplan)

#: Falling, past the walker's recovery: the trunk (`_trunk`) tipped past FALLING_DEG and tipping
#: on faster than FALLING_DEG_S, or the pelvis under FALLING_M, walking. Fallen - under FALLEN_M
#: or tipped past FALLEN_DEG walking, under SQUAT_FALLEN_M in the arrival's moves - is down. Read
#: on the pelvis, a parry's step pitched it 9 degrees at 141 deg/s and the fall called shorted
#: her legs as the foot landed on the capture point (2026-10-01); on the head, 0.77 kg whipped
#: 5.4 m/s under P's shove called her down 0.08 s in and it struck at 1.75 and 3.57 m/s; on the
#: trunk in none of 16 (2026-10-02).
FALLING_DEG, FALLING_DEG_S, FALLING_M = 12.0, 60.0, 0.65
FALLEN_M, FALLEN_DEG, SQUAT_FALLEN_M = 0.55, 35.0, 0.3

#: Down and checked, she gets up GETUP_TRIES times at most between two landings.
GETUP_TRIES = 3

#: Standing and stepping (`machine.stand`): watched for a fall as walking, a step's lunge
#: past TREAD_DEG (the brick's step down read as a fall at 12.3 degrees, 2026-10-04).
STANDING, TREAD_DEG = ('stand', 'tread'), 25.0


#: A stance foot bearing `stance.BEARS_N` slid past SLIP_M is held where it is: at 2 cm the
#: plan re-anchored eight times in two seconds and she fell (2026-09-26).
SLIP_M = 0.04

#: From the walk into the settling the setpoints ease over BLEND_S from her pose, the pelvis
#: unrolled and unturned.
BLEND_S = 0.3

#: She rises and starts at `gait.CADENCE` whatever is asked; walking goes to the asked
#: cadence at PACE_RATE strides/s a second.
PACE_RATE = 0.1

#: The legs' drives' heat as their boards report it: `spent` of a drive's envelope, 1 at its
#: ceiling, derating past `heat.THROTTLE_AT`. Walking, the most spent past EASE_AT eases the
#: pace, to EASE_FLOOR strides/s at EASE_FULL: 60 s at 0.85 strides/s the hips' laminate spent
#: 0.95 and derated to 0.5; at 0.75 and 0.65, 0.76 and 0.74; eased, 0.79 over 110 s (2026-09-28).
EASE_AT, EASE_FULL, EASE_FLOOR = 0.7, 0.88, 0.65

#: A drive heard with its gates dropped is armed again REARM_S after, the wait doubled up to
#: REARM_MAX_S when it drops within REARM_BACKOFF_S of its last arming, heard back ARM_LAG_S on.
REARM_S, REARM_BACKOFF_S, REARM_MAX_S, ARM_LAG_S = 0.05, 1.0, 1.6, 0.005

#: Her moments numbered from 1, on the page and in look.py alike.
MOMENTS = ('squat', 'look', 'push', 'rise', 'stand', 'shift', 'lean', 'step', 'walk', 'catch',
           'halt', 'settle', 'lower', 'rest', 'falling', 'fallen') + getup.STAGES + ('check', 'tread')


#: A thread for a model's get-up plan: the loop goes on.
_PLANNERS = ThreadPoolExecutor(1)


def moment(stage):
    """A stage by its moment's number: '7 lean'; unnumbered, as it is."""
    return '%d %s' % (MOMENTS.index(stage) + 1, stage) if stage in MOMENTS else stage


class Director:

    """The moves one after another, each pass's setpoints from whichever has her."""

    def __init__(self, machine, cadence=gait.CADENCE, walk_s=None, rest_s=None, local=None,
                 server=None, stand_s=0.0):
        self.machine, self.asked = machine, float(cadence)
        #: The models that plan her get-up (`machine.planner`): a local one, and a server's
        #: asked once it has failed; neither, the planner's own.
        self.local, self.server = local, server
        if local is not None:
            # Warmed now: loaded at the first fall's ask, gemma4:12b took 4-9 s more.
            _PLANNERS.submit(local.chat, [{'role': 'user', 'content': 'ready'}], think=False,
                             num_predict=1)
        self.arrival = arrival.Arrival(machine, gait.CADENCE, stand_s)
        self.walker = walker.Walker(machine, gait.CADENCE)
        self.walk_s, self.rest_s = walk_s, rest_s
        self.stage, self.fallen_at, self.slips = 'squat', None, 0
        self.since, self.blend, self.age = 0.0, None, 0.0
        #: Since when she falls and her arms and neck from and to what (`falls.reach`); the tilt last
        #: pass, (deg, s), and its rate, deg/s; when an arm met the floor; since when she tucks and
        #: from where, (s, {joint: deg}); her drives down.
        self.falling_at, self.curl_from, self.curl_to, self.tilt_was = None, {}, {}, None
        self.curl_at = 0.0
        self.fall_rate, self.touched_at, self.tucked, self.down = 0.0, None, None, None
        #: The get-up and the get-ups since she landed; what felled her, as the observer says it;
        #: the plans tried since, [(steps, why)], and one being made.
        self.getup, self.tries = getup.GetUp(machine), 0
        self.cause, self.tried, self.planning = '', [], None
        #: Standing: the steps since she landed, seconds since the last, each foot's hang.
        self.treads, self.hang, self.stepping = 0, {}, None
        #: Each joint's drive by its node's channels, the legs'; a dropped drive's (heard at,
        #: wait) and when each was last armed.
        self.world = machine.nodes['pelvis'].world
        self.drives = {figure.JOINTS[n.index]: n.name + '.angle.' for n in machine.nodes
                       if hasattr(n, 'index')}
        self.legs = [side + k for side, _sign in walkplan.SIDES for k in figure.LEG + ('_foot',)
                     if side + k in self.drives]
        self.dropped, self.armed = {}, {}

    @property
    def pendulum(self):
        """The pendulum between her ears (`machine.pendulum`): the walker's, read here when it
        does not walk."""
        return self.walker.pendulum

    @property
    def cadence(self):
        """The cadence asked; `walker.cadence` is the one she walks at."""
        return self.asked

    @cadence.setter
    def cadence(self, value):
        self.asked = float(value)

    @property
    def given_up(self):
        """Down with no get-up left: GETUP_TRIES of them failed."""
        return self.stage == 'fallen' and self.tries >= GETUP_TRIES

    def begin(self, drop=0.002, up=0.0, stagger=0.0, stage='squat'):
        """Landed in the squat - or standing, `stage` 'stand' - from `drop` m, `up` m over the
        floor, the left foot `stagger` m ahead; the arrival takes her on."""
        self.getup = getup.GetUp(self.machine)
        self.arrival.land(drop, up, stagger, stage)
        self.stage, self.fallen_at, self.since = self.arrival.stage, None, 0.0
        self.walker.cadence = gait.CADENCE
        self.walker.reset()
        self.blend, self.curl_from = None, {}
        self.falling_at, self.curl_to, self.tilt_was, self.touched_at = None, {}, None, None
        self.tucked, self.down = None, None
        self.dropped, self.armed, self.tries = {}, {}, 0
        self.cause, self.tried, self.planning = '', [], None
        self.treads, self.hang, self.stepping = 0, {}, None

    def halt(self):
        """Walking, to a stop and down into the squat."""
        if self.stage in ('walk', 'catch'):
            self.walker.halt()
            self.stage = 'halt'

    def _planned(self, now):
        """The house's plan for `now`, at once; a model, if any, asked meanwhile, its plan
        taken at a step's end (`_asked`): waited on, she lay still 4-9 s on the page."""
        if self.local is not None or self.server is not None:
            self.planning = _PLANNERS.submit(planner.plan, now, self.local, self.server,
                                             list(self.tried))
        return planner.plan(now, history=self.tried)[0]

    def _asked(self, nxt):
        """A model's plan once in, from `nxt` on, if that is not what runs; else None."""
        if self.planning is None or not self.planning.done():
            return None
        steps, self.planning = self.planning.result()[0], None
        rest = tuple(steps[steps.index(nxt):]) if nxt in steps else ()
        return rest if rest and rest != tuple(m[1] for m in self.getup.marks) else None

    def _begin(self, steps, now):
        """The get-up on `steps` from where she is, a try more."""
        self.plan, self.tries = tuple(steps), self.tries + 1
        self.getup.begin(*planner.stream(steps, now))

    def spent(self, joints=None):
        """The most spent of `joints`' drives (the legs'), as their boards said."""
        bus = self.machine.loop.bus
        return max(bus.get(self.drives[j] + 'spent', 0.0) for j in (joints or self.legs))

    def rise(self):
        """Resting in the squat, up and walking again."""
        if self.stage == 'rest':
            bus = self.machine.loop.bus
            feet = [self._foot(bus, side, sign)[0] for side, sign in walkplan.SIDES]
            self.arrival.rise(sum(f[0] for f in feet) / 2.0, sum(f[2] for f in feet) / 2.0,
                              math.degrees(self.walker.heading))
            self.stage, self.since = self.arrival.stage, 0.0

    def step(self, dt):
        """{joint: degrees}: what whichever move has her sets now."""
        bus = self.machine.loop.bus
        if self.down is None:
            self._arm(bus)
        if (self.stage in arrival.STAGES or self.stage in getup.STAGES
                or self.stage in ('falling', 'fallen')):
            self.pendulum.read(self.walker.view(bus), dt)
        if self.falling_at is None and (self._falling(bus) or self._fallen(bus)):
            tip = self._fall_way(bus)
            self.cause = falls.cause(tip, self.fall_rate, self.stage == 'catch')
            self.falling_at, self.curl_at, self.stage = bus['t'], bus['t'], 'falling'
            self.curl_to = dict(falls.reach(tip, self.fall_rate),
                                **(falls.crouch(tip) if falls.CROUCH else {}))
            self.curl_from = {j: bus.get(j + '.deg', 0.0) for j in self.curl_to}
            if not falls.CROUCH:
                self._short(falls.SHORT_FALLING)
        if self.fallen_at is None and self._fallen(bus):
            self.fallen_at, self.stage, self.down = bus['t'], 'fallen', down.Down(self.world)
            if falls.CROUCH:
                self._short(falls.SHORT_FALLING)
        if self.falling_at is not None:
            if self.touched_at is None:
                tilt, t = self._tilt(bus), bus['t']
                rate = (tilt - self.tilt_was[0]) / max(1e-6, t - self.tilt_was[1]) if self.tilt_was else 0.0
                self.tilt_was = (tilt, t)
                if falls.aimed(self.curl_to, self.curl_from, bus, tilt, self._fall_way(bus),
                               max(rate, self.fall_rate)):
                    self.curl_at = t
            k = gait.eased((bus['t'] - self.curl_at) / falls.CURL_S)
            out = {j: self.curl_from[j] + (v - self.curl_from[j]) * k
                   for j, v in self.curl_to.items()}
            out.update({j: self.curl_to[j] for j in falls.AT_ONCE if j in self.curl_to})
            if self.touched_at is None and self.world.lifted(falls.ARM_PARTS) < falls.TOUCH_M:
                self.touched_at = bus['t']
            if self.touched_at is not None:
                out = falls.yielded(out, bus)
            return out if self.down is None else self._lain(bus, dt, out, self.down)
        self.since += dt
        if self.stage in getup.STAGES:
            out = self.getup.step(dt)
            self.stage = self.getup.stage
            ended = self.getup.ended()
            if ended is None and self.getup.marks and self.getup.marks[0][1] in observer.EARLY:
                ended = self.getup.reached(observer.check(
                    self.getup.marks[0][1], observer.status(bus, self.world, self))[0], dt)
            if ended is not None:
                now = observer.status(bus, self.world, self)
                ok, why = observer.check(ended, now)
                nxt = self.getup.marks[0][1] if self.getup.marks else None
                if ok and nxt and not planner.fits(nxt, now['lying']):
                    ok, why = False, '%s left her %s, where %s cannot begin' % (
                        ended, now['lying'], nxt)
                if not ok:
                    self.tried.append((self.plan, why))
                    if self.tries >= GETUP_TRIES:
                        self.stage, self.fallen_at, self.falling_at = 'fallen', bus['t'], bus['t']
                        self.curl_at = bus['t']
                        self.curl_to, self.down = {}, down.Down(self.world, check=False)
                        return out
                    self._begin(self._planned(now), now)
                    return self.getup.step(0.0)
                asked = self._asked(nxt) if nxt else None
                if asked:
                    self.plan = asked
                    self.getup.begin(*planner.stream(asked, now))
                    return self.getup.step(0.0)
            if self.getup.done:
                frames = self.getup.handed()
                self.arrival.play(frames)
                # facing where she lay; the walk as before the first (`Walker.reset`)
                self.walker.reset()
                self.walker.face(math.radians(frames[0][2]['yaw']))
                self.stage, self.blend, self.age, self.since = 'squat', out, 0.0, 0.0
            self.walker.last = out
            return out
        if self.stage in arrival.STAGES:
            out = self.arrival.step(dt)
            if self.arrival.stage != self.stage and self.arrival.stage == 'rest':
                self.since = 0.0
            self.stage = self.arrival.stage
            if self.stage == 'tread':
                stand.retarget(self, bus)
            if self.stage == 'stand' and self.arrival.stand_s:
                frames = stand.needed(self, bus, dt, out)
                if frames is not None:
                    self.arrival.play(frames)
                    self.treads += 1
                    out, self.stage = self.arrival.step(0.0), self.arrival.stage
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
                                          bus['pelvis.pose.right_load']) > stance.LANDED_N:
                # settled as along the walk's line, turned back onto her heading
                seen, h = self.walker.view(bus), self.walker.heading
                now = self._now(seen, out)
                ahead = 'left' if now['left'][0][2] >= now['right'][0][2] else 'right'
                speed = bus['pelvis.pose.vx'] * math.sin(h) + bus['pelvis.pose.vz'] * math.cos(h)
                self.arrival.settle(now, self._flat(seen, ahead), speed, math.degrees(h))
                self.stage, self.blend, self.age = self.arrival.stage, out, 0.0
        elif self.walk_s is not None and self.since > self.walk_s:
            self.halt()
        self.walker.last = out
        return out

    def _arm(self, bus):
        """Each drive heard with its gates dropped, armed again after its wait."""
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

    def _lain(self, bus, dt, out, d):
        """Down (`d`): braked, tucked (`falls.TUCKED`), cut, checked and up; given up, cut.
        Cut, each setpoint where its joint is: armed again, none jumps."""
        if d.step(dt, bus):
            out = {j: bus.get(j + '.deg', 0.0) for j in figure.JOINTS}
        elif falls.TUCKED and bus['t'] - self.fallen_at >= falls.TUCK_AFTER_S:
            if self.tucked is None:
                self.tucked = (bus['t'], {j: bus.get(j + '.deg', 0.0) for j in falls.TUCK})
                for i in range(len(figure.JOINTS)):
                    self.world.arm(i)
            out = falls.tucked(out, bus, (bus['t'] - self.tucked[0]) / falls.TUCK_S,
                               self.tucked[1])
        self.stage = 'check' if d.mode == 'checking' else 'fallen'
        if d.mode == 'checked' and self.tries < GETUP_TRIES:
            for i in range(len(figure.JOINTS)):
                self.world.arm(i)
            now = observer.status(bus, self.world, self)
            self._begin(self._planned(now), now)
            self.stage, self.falling_at, self.fallen_at = self.getup.stage, None, None
            self.touched_at = self.tucked = self.down = None
        return out

    def _short(self, kinds):
        """The drives of `kinds` (None: every one) shorted through their low sides."""
        for i, joint in enumerate(figure.JOINTS):
            if kinds is None or drives.kind(joint) in kinds:
                self.world.short(i)

    def _tilt(self, bus):
        """The trunk's tilt from upright, degrees (`_trunk`)."""
        up = self._trunk(bus)[1][1]
        return math.degrees(math.acos(max(-1.0, min(1.0, up))))

    def _trunk(self, bus):
        """The torso's turn: the head's IMU's less the head's and the neck's joints as read."""
        head = figure.quat(*(bus['pelvis.pose.head_q' + a] for a in 'wxyz'))
        return figure.mul(figure.mul(head, figure.ry(-math.radians(bus['head.deg']))),
                          figure.rx(-math.radians(bus['neck.deg'])))

    def _falling(self, bus):
        """Past recovery, walking (FALLING_DEG ..)."""
        if (self.stage in arrival.STAGES and self.stage not in STANDING
                or self.stage in getup.STAGES):
            self.tilt_was = None
            return False
        tilt, t = self._tilt(bus), bus['t']
        self.fall_rate = 0.0 if self.tilt_was is None else (
            (tilt - self.tilt_was[0]) / max(1e-6, t - self.tilt_was[1]))
        self.tilt_was = (tilt, t)
        deg = TREAD_DEG if self.stage == 'tread' else FALLING_DEG
        return bus['pelvis.pose.y'] < FALLING_M or (tilt > deg and self.fall_rate > FALLING_DEG_S)

    def _fall_way(self, bus):
        """Which way the trunk tips, deg from the pelvis's forward, + to its left."""
        turn, head = self._pelvis(bus)[1], self._trunk(bus)
        up = (head[0][1], head[2][1])
        ahead, left = ((turn[0][k], turn[2][k]) for k in (2, 0))
        return math.degrees(math.atan2(up[0] * left[0] + up[1] * left[1],
                                       up[0] * ahead[0] + up[1] * ahead[1]))

    def _fallen(self, bus):
        """Down (FALLEN_M ..)."""
        if self.stage in arrival.STAGES and self.stage not in STANDING:
            return bus['pelvis.pose.y'] < SQUAT_FALLEN_M
        if self.stage in getup.STAGES:
            return False
        return bus['pelvis.pose.y'] < FALLEN_M or self._tilt(bus) > FALLEN_DEG

    def _foot(self, bus, side, sign):
        """(ankle, the foot's pitch toes-up deg) of a leg as the loop read it, world."""
        angles = tuple(math.radians(bus.get(side + k + '.deg', 0.0)) for k in figure.LEG)
        pel, turn = self._pelvis(bus)
        ankle, foot = figure.foot_of(sign, pel, turn, angles)
        foot = figure.mul(figure.ry(-math.atan2(turn[0][2], turn[2][2])), foot)
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
        """Her pose as read, a keyframe: the pelvis tipped, not rolled or turned; the feet
        where they stand; the upper body as last set."""
        pel, turn = self._pelvis(bus)
        yaw = math.degrees(math.atan2(turn[0][2], turn[2][2]))
        local = figure.mul(figure.ry(-math.radians(yaw)), turn)
        frame = {'pelvis': pel, 'tilt': math.degrees(math.atan2(local[2][1], local[1][1])),
                 'yaw': yaw, 'joints': {j: out[j] for j in walkplan.UPPER if j in out}}
        for side, sign in walkplan.SIDES:
            frame[side] = self._foot(bus, side, sign)
        return frame

    def _watch(self, bus):
        """The stage as the walker has her; a slid foot held where it went."""
        if self.stage != 'halt':
            self.stage = 'catch' if self.walker.catching else 'walk'
        for side, ball in self.walker.balls.items():
            held = self.walker.anchor.get(side)
            if held is None or bus['pelvis.pose.%s_load' % side] < stance.BEARS_N:
                continue
            if (ball[0] - held[0]) ** 2 + (ball[2] - held[2]) ** 2 > SLIP_M ** 2:
                self.walker.anchor[side] = (ball[0], held[1], ball[2])
                self.slips += 1
