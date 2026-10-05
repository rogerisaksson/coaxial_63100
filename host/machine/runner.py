"""The gynoid's run: a bounce a foot, flight between.

    runner = Runner(machine, speed=1.5)
    runner.start()                              # placed in flight at its speed (a tool's)
    machine.loop.write(**runner.step(dt))       # every pass

A foot comes down under where she will be at the middle of its stance (Raibert, 1986): half her
way over it ahead of the hip, further ahead the faster she is than asked and has been, across by
her sideways speed. It takes her as she comes - the pelvis's height over it a quintic in the
stance's time, from where and how fast she lands, in free fall, to the same height rising: the
flight asked. On the ball of the foot: the heel comes down, and rises again as she leaves; the
knee is what that leaves it. Her trunk is held by the standing hip; the other leg goes joint by
joint to its landing pose, its knee folded on the way, and waits where it will land on the
floor; the arms swing against the legs. On a flat floor, from a flight (`tools/sim/run.py`):
her walk into it, her turns and her parries are not in it.
"""
import math

from machine import curves, figure, gait, walkplan
from machine.figure import LEG, SOLE_BALL, SOLE_HEEL, add, apply, mul, rx, ry, sub, t
from machine.pendulum import G
from machine.stance import length

#: A stance's and a flight's seconds; she leaves the floor TAKE_M higher than she landed, her
#: leg longer there; the stance's height begins START_G of a free fall's acceleration - in free
#: fall, 1, her sole bore 360 N at 24 ms, 68 at 60 and 648 at 110. A stance of 0.22 s ran her
#: 1.94 m/s at 253 J/m on a rotor 0.4 of the U8's; on the U8's it holds 1.4 m/s at 403-451 J/m
#: and is down at 1.75 asked, where 0.30 s runs 1.66 at 266. At 0.32 she is down at 10.6 s; at
#: 0.26, asked 2.0 from 1.5, she ran 30 s at 1.91 (2026-10-05).
CONTACT_S, FLIGHT_S, TAKE_M, START_G = 0.30, 0.10, 0.0, 1.0

#: The knee as the foot lands, deg. The foot lands on its ball, the heel HEEL_DEG up; standing,
#: the heel gives ABSORB of what she sinks, never quite to the floor, and rises as her hip
#: goes on ahead of the ankle, from HEEL_FROM_M to HEEL_OFF_DEG at HEEL_TO_M. Brought down on a
#: clock, 10 deg over 44 ms, the ankle sank faster than she did and the leg stood stiff under
#: her: 532 N in 20 ms, the hip, the knee and the ankle at their clamps, and off the floor
#: again. Landed flat, the knee took all of her sink: 59 deg and 78 N m late in the stance, 59
#: N m rms. Given to the floor, the heel met it 68 ms in: the ankle 136 N m (2026-10-05).
KNEE_LAND, HEEL_DEG, ABSORB, HEEL_FROM_M, HEEL_TO_M, HEEL_OFF_DEG = 22.0, 14.0, 0.65, 0.0, 0.22, 50.0

#: The foot further ahead a m/s she runs over the speed asked, s; the feet TRACK_M either side
#: of her line and across by SIDE_S of her sideways speed; the foot waits where it will land on
#: the floor, MOST_M ahead of its hip at most - coming down it goes back under her at the
#: floor's speed.
SPEED_S, TRACK_M, SIDE_S, MOST_M = 0.06, 0.035, 0.14, 0.36

#: Her speed is her centre of mass's, filtered over SPEED_FILTER_S: the pelvis's swings 0.3 m/s
#: with her trunk's pitch.
SPEED_FILTER_S = 0.01

#: What her speed is off the one asked, a landing after another, moves where the feet land:
#: SPEED_I m a m/s a landing, BIAS_M at most. On SPEED_S alone she ran 1.57 m/s asked 2.0, 1.23
#: with none of it, and at 0.15 past 2.4 and down (2026-10-05).
SPEED_I, BIAS_M = 0.008, 0.15

#: Her trunk ahead of plumb, deg; the pelvis's attitude turned back past its error (the walker's).
LEAN_DEG, TURN_K = 6.0, 1.42

#: The swinging leg goes joint by joint from where it left to its landing pose, its knee
#: FOLD_DEG further bent at the middle of the way, a sine of it; through the flight it waits
#: ahead and comes back to its place. On the clock alone, a first flight 40 ms long had it
#: straight under the other leg's dip: 496 N on its sole, its hip and knee at their clamps, and
#: she was thrown up at 1.1 m/s. Folded along the line from hip to ankle, 20 cm shorter, the
#: thigh went 39 deg on in 80 ms and her trunk pitched on at 220 deg/s a flight. A fold of no
#: rate at its ends, the sine's square, had her down within 2 s (2026-10-05).
FOLD_DEG = 55.0

#: At its pose READY_S before it is due to land. At its pose as the other foot left the floor,
#: the leg was straight under her 5 cm before she was up - 300 N on its sole, and she was
#: thrown up at 1.1 m/s; four fifths there as the other left and the rest on a clock, her
#: flights went short and long by turns, 31 and 149 ms, and she fell at 5.1 s; ready 40-100 ms
#: before, swapped in under way, she was down 0.8-7.5 s on (2026-10-05).
READY_S = 0.02

#: The swinging foot is level LEVEL of its swing in, and its heel up for the landing from
#: there on: left to hang it pointed 73 deg down, its ankle taking back 0.7 of the knee's fold
#: 37, and its toes met the floor under the standing leg's dip - the hip and the knee at their
#: clamps; level by 0.3, the ankle turned 975 deg/s as the knee folded (2026-10-05).
LEVEL = 0.5

#: A leg changes hands over BLEND_S: what its setpoints jump by as it lands fades out, and the
#: rate they left the floor at dies over COAST_S. At once, a landing asked the hip, the knee and
#: the ankle their clamps for a pass or two and the knee 800 deg/s (2026-10-05).
BLEND_S, COAST_S = 0.03, 0.02

#: A test's start: this far over the take-off's height, m, the leg that left the floor
#: START_U of its swing on.
START_M, START_U = 0.03, 0.2

#: A foot is down: its sole within TOUCH_M of the floor or bearing LANDED_N.
TOUCH_M, LANDED_N = 0.003, 120.0

#: The arms: a shoulder's deg a deg of its own leg's swing, against it; the elbow, deg.
ARM, ELBOW_DEG = 0.6, 80.0


def eased(u):
    """0 to 1 over 0 to 1, no rate at either end (`curves.eased` has no acceleration there
    either)."""
    u = max(0.0, min(1.0, u))
    return u * u * (3.0 - 2.0 * u)


def heel(on, landed, sunk):
    """The standing heel's rise, rad: her hip `on` m ahead of the flat foot's ankle, the heel
    landed `landed` rad up and she `sunk` m under her landing's height."""
    up = gait.BALL * math.sin(landed)
    give = up * math.exp(-ABSORB * max(0.0, sunk) / up) if up > 1e-6 else 0.0
    return max(math.asin(min(1.0, give / gait.BALL)),
               math.radians(HEEL_OFF_DEG) * eased((on - HEEL_FROM_M) / (HEEL_TO_M - HEEL_FROM_M)))


def pitched(turn):
    """A turn's pitch on, rad."""
    return math.atan2(turn[2][1], turn[1][1])


class Runner:

    """The setpoints of a run for a DYNAMIC gynoid, every pass, from what the loop read."""

    def __init__(self, machine, speed=1.5):
        self.machine, self.speed = machine, float(speed)
        self.world = machine.nodes['pelvis'].world
        self.heading = 0.0
        #: A leg a row: whether it stands, how long it has, and its stance's or its swing's own.
        self.legs = {side: {'stands': False, 't': 0.0} for side, _sign in walkplan.SIDES}
        self.last, self.rates, self.steps, self.com, self.v = {}, {}, [], None, (0.0, 0.0, 0.0)
        self.bias = 0.0

    # -- the cycle as asked ------------------------------------------------------------------

    def rise(self):
        """The pelvis's rise rate at take-off, m/s: TAKE_M down, at its landing's height,
        FLIGHT_S on."""
        return 0.5 * G * FLIGHT_S - TAKE_M / FLIGHT_S

    def ahead(self, v):
        """How far ahead of its hip a foot's ball comes down, m, at her speed `v`."""
        return 0.5 * v * CONTACT_S + SPEED_S * (v - self.speed) + self.bias

    def landing(self, sign, hip, v, to_go=0.0):
        """(ankle, foot's turn) of the landing pose under `hip`, `to_go` s before it lands: the
        ball ahead and across as the law has it, the heel HEEL_DEG up, the knee KNEE_LAND."""
        # Brought down on the floor's own point instead - its ball 5 cm over it 80 ms before,
        # the floor's speed under it - her weight came on it at once, 377 N at 21 ms, and she
        # rose at 0.48 m/s of the 0.49 asked; her feet landed 0.10 m ahead of their place, the
        # hip's servo 5 Hz with the leg on it, and her speed swung 1.0-2.6 m/s (2026-10-05).
        c, s = math.cos(self.heading), math.sin(self.heading)
        on, across = v[0] * s + v[2] * c, v[0] * c - v[2] * s
        foot = mul(ry(self.heading), rx(math.radians(HEEL_DEG)))
        rel = apply(foot, SOLE_BALL)
        dz = min(MOST_M, self.ahead(on) + on * max(0.0, to_go))
        dx = sign * (TRACK_M - gait.HIP_HALF) + across * (0.5 * CONTACT_S + SIDE_S)
        # the ankle's offset from the hip, across and along her heading
        ax, az = dx - (rel[0] * c - rel[2] * s), dz - (rel[0] * s + rel[2] * c)
        down = math.sqrt(max(1e-6, length(KNEE_LAND) ** 2 - ax * ax - az * az))
        return add(hip, (ax * c + az * s, -down, -ax * s + az * c)), foot

    def fall(self, sign, hip, v):
        """The seconds till a foot lands: her fall till its landing pose's ball is on the floor."""
        ankle, foot = self.landing(sign, hip, v)
        low = max(0.0, add(ankle, apply(foot, SOLE_BALL))[1])
        return (v[1] + math.sqrt(v[1] * v[1] + 2.0 * G * low)) / G

    def land_y(self):
        """The pelvis's height as a foot comes down at the speed asked, m."""
        hip = figure.hip(1.0, (0.0, 1.0, 0.0), rx(math.radians(LEAN_DEG)))
        ankle, foot = self.landing(1.0, hip, (0.0, 0.0, self.speed))
        return 1.0 - add(ankle, apply(foot, SOLE_BALL))[1]

    # -- a test's start ----------------------------------------------------------------------

    def start(self):
        """The body placed in flight at the speed asked, rising as a take-off does: the left
        leg START_U of its swing from where it left the floor behind her, the right coming
        down."""
        lean = math.radians(LEAN_DEG)
        turn = rx(lean)
        v = (0.0, self.rise(), self.speed)
        # the left leg as it left the floor: on its ball behind her at the take-off's height
        off = (0.0, self.land_y() + TAKE_M, 0.0)
        up = math.radians(HEEL_OFF_DEG)
        ball = (TRACK_M, 0.0, -0.5 * self.speed * CONTACT_S)
        was = figure.leg(1.0, off, turn, sub(ball, apply(rx(up), SOLE_BALL)), rx(up))
        pelvis = (0.0, off[1] + START_M, 0.0)
        fall = self.fall(-1.0, figure.hip(-1.0, pelvis, turn), v)
        self.legs = {'left': {'stands': False, 't': START_U * (CONTACT_S + 2.0 * FLIGHT_S),
                              'rise': up, 'from': was, 'u': START_U, 'rate': (0.0,) * 6,
                              'to_go': fall + CONTACT_S + FLIGHT_S},
                     'right': {'stands': False, 't': 1e3, 'from': None, 'to_go': fall}}
        angles = self._upper({'left': -20.0, 'right': 20.0})
        for side, sign in walkplan.SIDES:
            for k, a in zip(LEG, self.swung(self.legs[side], sign, pelvis, turn, v)):
                angles[side + k] = math.degrees(a)
        angles['left_foot'] = angles['right_foot'] = 0.0
        self.world.reset(angles, where=pelvis, turn=(math.cos(lean / 2), math.sin(lean / 2),
                                                     0.0, 0.0), speed=v)
        self.last, self.rates, self.steps, self.com, self.v = angles, {}, [], None, v
        self.bias = 0.0
        return angles

    # -- every pass --------------------------------------------------------------------------

    def swung(self, leg, sign, pel, now, v):
        """A swinging leg's six joints, rad, `leg['u']` of its way from where it left the
        floor to its landing pose: joint by joint, its knee folded on the way, its foot level."""
        ankle, foot = self.landing(sign, figure.hip(sign, pel, now), v, leg['to_go'])
        joints = list(figure.leg(sign, pel, now, ankle, foot))
        if leg.get('from') is None:
            return joints
        u = leg['u']
        coast = COAST_S * (1.0 - math.exp(-leg['t'] / COAST_S))
        joints = [a + r * coast + (b - a - r * coast) * eased(u)
                  for a, r, b in zip(leg['from'], leg['rate'], joints)]
        joints[3] += math.radians(FOLD_DEG) * math.sin(math.pi * u)
        # the foot level through the swing: its heel's rise from what it left with to none by
        # LEVEL of the way, to the landing's from 1 - LEVEL
        rise = (leg['rise'] * (1.0 - eased(u / LEVEL))
                + math.radians(HEEL_DEG) * eased((u - LEVEL) / (1.0 - LEVEL)))
        joints[4] = rise - (pitched(now) + joints[2] + joints[3])
        return joints

    def _upper(self, swing):
        """The upper body: the arms against the legs' swing {side: deg ahead}."""
        out = {'spine_roll': 0.0, 'spine': 0.0, 'waist': 0.0, 'neck': -0.5 * LEAN_DEG, 'head': 0.0}
        for side in ('left', 'right'):
            out[side + '_shoulder'] = -ARM * swing[side]
            out[side + '_elbow'] = ELBOW_DEG
            out[side + '_wrist'], out[side + '_gripper'] = 0.0, 20.0
        return out

    def step(self, dt):
        """{joint: degrees}: where every drive should be now."""
        bus = self.machine.loop.bus
        pel = (bus['pelvis.pose.x'], bus['pelvis.pose.y'], bus['pelvis.pose.z'])
        now = figure.quat(bus['pelvis.pose.qw'], bus['pelvis.pose.qx'], bus['pelvis.pose.qy'],
                          bus['pelvis.pose.qz'])
        com = (bus['pelvis.pose.com_x'], bus['pelvis.pose.com_y'], bus['pelvis.pose.com_z'])
        if self.com is not None and dt > 0.0:
            k = min(1.0, dt / SPEED_FILTER_S)
            self.v = tuple(a + k * ((c - b) / dt - a) for a, b, c in zip(self.v, self.com, com))
        self.com = com
        # across and along her centre of mass's, up and down the pelvis's own
        v = (self.v[0], bus['pelvis.pose.vy'], self.v[2])
        want = mul(ry(self.heading), rx(math.radians(LEAN_DEG)))
        err = walkplan.vee(mul(want, t(now)))
        turn = mul(walkplan.turned(tuple(TURN_K * c for c in err)), want)
        out, swing = {}, {}
        for side, sign in walkplan.SIDES:
            leg, other = self.legs[side], self.legs[walkplan._OTHER[side]]
            leg['t'] += dt
            angles = tuple(math.radians(bus.get(side + k + '.deg', 0.0)) for k in LEG)
            ankle, foot = figure.foot_of(sign, pel, now, angles)
            ball, back = add(ankle, apply(foot, SOLE_BALL)), add(ankle, apply(foot, SOLE_HEEL))
            last = tuple(math.radians(self.last.get(side + k, 0.0)) for k in LEG)
            # a foot comes down; a foot has stood its stance
            if (not leg['stands'] and not other['stands'] and leg['t'] > 0.5 * CONTACT_S
                    and (min(ball[1], back[1]) <= TOUCH_M
                         or bus['pelvis.pose.%s_load' % side] > LANDED_N)):
                # the pelvis's height through the stance, its ends over the stance's span
                leg.update(stands=True, t=0.0, ball=(ball[0], 0.0, ball[2]), was=last, off=None,
                           landed=max(0.0, pitched(foot)),
                           y=((pel[1], v[1] * CONTACT_S, -START_G * G * CONTACT_S ** 2),
                              (pel[1] + TAKE_M, self.rise() * CONTACT_S, 0.0)))
                self.bias = max(-BIAS_M, min(BIAS_M, self.bias + SPEED_I * (
                    v[0] * math.sin(self.heading) + v[2] * math.cos(self.heading) - self.speed)))
                self.steps.append({'t': bus['t'], 'side': side, 'v': v[2], 'sink': v[1],
                                   'y': pel[1], 'ahead': ball[2] - figure.hip(sign, pel, now)[2]})
            elif leg['stands'] and leg['t'] >= CONTACT_S:
                leg.update(stands=False, t=0.0, rise=pitched(now) + last[2] + last[3] + last[4],
                           rate=self.rates.get(side, (0.0,) * 6), to_go=None, u=0.0,
                           **{'from': last})
                if self.steps:
                    self.steps[-1].update(off=bus['t'], rise=v[1], y_off=pel[1])
            if leg['stands']:
                landed, leaving = leg['y']
                target = (pel[0], curves.hermite(landed, leaving, leg['t'] / CONTACT_S), pel[2])
                flat = sub(leg['ball'], apply(ry(self.heading), SOLE_BALL))
                on = sub(figure.hip(sign, target, turn), flat)
                foot = mul(ry(self.heading), rx(heel(
                    on[0] * math.sin(self.heading) + on[2] * math.cos(self.heading),
                    leg['landed'], landed[0] - target[1])))
                joints = list(figure.leg(sign, target, turn, sub(leg['ball'], apply(foot, SOLE_BALL)),
                                         foot))
                if leg['off'] is None:
                    leg['off'] = tuple(a - b for a, b in zip(leg['was'], joints))
                fade = 1.0 - eased(leg['t'] / BLEND_S)
                joints = [a + b * fade for a, b in zip(joints, leg['off'])]
            else:
                # how long till it lands: her fall till its landing pose's sole is on the floor
                # - and, the other foot's turn first, its stance and the flight after it
                fall = self.fall(sign, figure.hip(sign, pel, now), v)
                to_go = (CONTACT_S - other['t'] + FLIGHT_S if other['stands'] else
                         fall if leg['t'] > other['t'] else fall + CONTACT_S + FLIGHT_S)
                if leg.get('to_go') is not None:
                    to_go = max(leg['to_go'] - 3.0 * dt, min(leg['to_go'] + dt, to_go))
                leg['to_go'] = to_go
                # its way's share goes on as fast as brings it to its pose READY_S before it is
                # due to land: what is left of the way over what is left of the time
                u = leg.get('u', 1.0)
                leg['u'] = min(1.0, u + dt * (1.0 - u) / max(to_go - READY_S, READY_S))
                joints = self.swung(leg, sign, pel, now, v)
            if dt > 0.0:
                self.rates[side] = tuple((a - b) / dt for a, b in zip(joints, last))
            for k, a in zip(LEG, joints):
                out[side + k] = math.degrees(a)
            out[side + '_foot'] = 0.0
            swing[side] = -out[side + '_hip']
        out.update(self._upper(swing))
        self.last = out
        return out
