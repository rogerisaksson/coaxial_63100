"""The gynoid's going as one law: her stand, her walk and her run rows of the same setpoints.

    going = Going(machine, gaits.STAND)         # placed by `tools/sim/go.py`
    going.ask = gaits.between(0.3)              # any pass: the setpoints, nothing else
    machine.loop.write(**going.step(dt))        # every pass

A limb bears or is free. What bears carries the pelvis: its height the bounce's - from where and
how fast she came down on the foot to `up` over it and rising `rise` - and never over what a
standing leg reaches on a strut's knee (`machine.strut`); her trunk held by the standing hips;
along the floor she goes where the pendulum takes her, leaned toward where her capture point is
asked (`machine.hold`). What is free goes to its next contact: under where she will be half a
support on, further ahead the faster she is than asked (Raibert, 1986), across at her capture
point and `track` out, clear of the floor and of the standing foot; there and nothing under it,
it reaches on down. A foot leaves `both` s - `stand` less `step` - after the other landed, and
by as much before that one is due with `both` under 0: both bear between, a walk, or she flies,
a run. The free one is due the other foot's `step` on and, the more both bear a step, as her hip
is half a step past the standing foot or her capture point has run a step's width from it. A
gait is a row of `machine.gaits`; asked no speed she stands, and steps only as her capture point
leaves her feet's hold. On a flat floor: her turns and a shove's parry are not in it, and
nothing of it is on the page (docs/findings/going.md).
"""
import math

from machine import curves, figure, gait, gaits, hold, strut, walkplan
from machine.figure import LEG, SOLE_BALL, SOLE_HEEL, add, apply, mul, rx, ry, sub, t
from machine.pendulum import G
from machine.runner import (ARM, BIAS_M, BLEND_S, COAST_S, ELBOW_DEG, LANDED_N, LEVEL, READY_S,
                            SPEED_FILTER_S, SPEED_S, START_G, TOUCH_M, eased, pitched)
from machine.stance import length

#: What her speed is off the one asked, a step's mean after another, moves where the feet land:
#: SPEED_I m a m/s a landing. The pelvis's attitude turned back TURN_K past its error. Where she
#: flies her speed's own gain (`runner.SPEED_S`) is 1 + FLY_K times a walk's: on the walk's the
#: run's row asked 1.0 m/s ran 0.7 for 3 s, then away past 1.47 and down at 7.5 s; on twice it,
#: 0.99, and the walk on twice it stood, walked and stood again on 7 timings of 10 (2026-10-05).
SPEED_I, TURN_K, FLY_K = 0.013, 1.6, 1.0

#: Past its bounce the height's rate dies over ON_S. A walk's landing is told from her speed,
#: SLOW_M_S at least. A walk's start placed at its speed: her feet WIDE tracks apart - the
#: pendulum's cycle at a step of 0.5 s -, the pelvis START_IN of half of it toward the foot she
#: stands on.
ON_S, SLOW_M_S, WIDE, START_IN = 0.14, 0.3, 6.0, 0.25

#: A foot is down touching or bearing (`runner.TOUCH_M`, `LANDED_N`) from LANDS_U of its swing
#: - or bearing BEARS_N wherever its swing is: what bears her is a contact, planned or not; one
#: at 250-500 N mid-swing was none and she fell on it -, SWUNG_S at least in the air. At its
#: pose and nothing under it, it reaches on down REACH_M_S.
LANDS_U, BEARS_N, SWUNG_S, REACH_M_S = 0.8, 250.0, 0.12, 0.3

#: Clearance. A free sole is LIFT_M over the floor from LIFTS_U of its swing to LANDS_U, let go
#: over DOWN_U: raised to it, the foot as it was - a knee's fold slewed 230 deg/s lost to the
#: knee's own 340 unfolding. A landing's ankle is CLEAR_M across from a standing one's still
#: down then, and the free foot as far within PASS_M of it along her way: 6-7 cm apart it
#: struck the standing foot, 180-475 N on both soles' sensors; held so in a run, her feet 7 cm
#: apart, she was down in 3 s (2026-10-05). A walk's ways - that, and a foot due by where she
#: is - are hers the more both feet bear a step, in full from BESIDE_S of it.
LIFT_M, LIFTS_U, DOWN_U, CLEAR_M, BESIDE_S, PASS_M = 0.01, 0.2, 0.1, 0.11, 0.05, (0.15, 0.27)
SOLE_TOE = (0.0, -gait.ANKLE_H, gait.BALL + gait.TOE_M)


class Going:

    """The setpoints of her going for a DYNAMIC gynoid, every pass, from what the loop read."""

    def __init__(self, machine, ask=None):
        self.machine, self.ask = machine, dict(gaits.RUN if ask is None else ask)
        self.world = machine.nodes['pelvis'].world
        self.heading = 0.0
        #: A leg a row: whether it stands, how long it has, and its stance's or its swing's own.
        self.legs = {side: {'stands': False, 't': 0.0} for side, _sign in walkplan.SIDES}
        #: The height's law since the last landing: its ends, its seconds, its rate past them.
        self.y = {}
        self.last, self.rates, self.steps, self.com, self.v = {}, {}, [], None, (0.0, 0.0, 0.0)
        self.bias, self.omega = 0.0, math.sqrt(G / 0.9)
        #: Her speed meaned over her last step, m/s, and where and when that step began.
        self.pace, self.mark = self.ask['speed'], None
        #: Both feet down, the foot that stays as the other steps and whether that one leaves
        #: at once (`hold.staying`).
        self.stays, self.parry = None, False

    # -- the setpoints' own ------------------------------------------------------------------

    def support(self):
        """The seconds a foot is her support: a step's, a stance's if shorter."""
        return min(self.ask['step'], self.ask['stand'])

    def share(self):
        """How much of a walk the row asked is, 0 to 1: the s both feet bear a step, of
        BESIDE_S."""
        return min(1.0, max(0.0, (self.ask['stand'] - self.ask['step']) / BESIDE_S))

    def ahead(self, v):
        """How far ahead of its hip a foot's sole comes down, m, at her speed `v`."""
        a = self.ask
        return (0.5 * v * self.support()
                + SPEED_S * (1.0 + FLY_K * (1.0 - self.share())) * (v - a['speed'])
                + self.bias * (1.0 - hold.stood(a['speed'])))

    def landing(self, sign, hip, v, to_go=0.0, reach=0.0, beside=None):
        """(ankle, foot's turn) of the landing pose under `hip`, `to_go` s before it lands: the
        sole ahead and across as the law has it - its share of CLEAR_M across from the ankle
        `beside` (ankle, share) -, the foot pitched and the knee bent as asked, `reach` m on."""
        a = self.ask
        c, s = math.cos(self.heading), math.sin(self.heading)
        on, across = v[0] * s + v[2] * c, v[0] * c - v[2] * s
        foot = mul(ry(self.heading), rx(math.radians(a['land'])))
        rel = apply(foot, (0.0, -gait.ANKLE_H, a['under']))
        dz = max(-a['reach'], min(a['reach'], self.ahead(on) + on * max(0.0, to_go)))
        # across, the pendulum's own gain: the runner's 0.5 Tc + 0.14 s was 0.29 of its 0.30
        # - from her centre of mass, not her hip: with it off the pelvis her feet fell short
        # of her capture point and walked off after it (2026-10-05)
        off = (-sign * gait.HIP_HALF if self.com is None else
               (self.com[0] - hip[0]) * c - (self.com[2] - hip[2]) * s)
        dx = off + sign * a['track'] + across / self.omega
        ax, az = dx - (rel[0] * c - rel[2] * s), dz - (rel[0] * s + rel[2] * c)
        if beside is not None:
            (bx, _by, bz), share = beside
            ax = sign * max(sign * ax, sign * ((bx - hip[0]) * c - (bz - hip[2]) * s)
                            + share * CLEAR_M)
        far = min(gait.THIGH + gait.SHANK - 1e-3, length(a['knee']) + reach)
        down = math.sqrt(max(1e-6, far ** 2 - ax * ax - az * az))
        return add(hip, (ax * c + az * s, -down, -ax * s + az * c)), foot

    def lowest(self, ankle, foot):
        """The sole's lower point's height, m."""
        return min(add(ankle, apply(foot, p))[1] for p in (SOLE_BALL, SOLE_HEEL))

    def fall(self, sign, hip, v):
        """The seconds till a foot lands in flight: her fall till its pose's sole is down."""
        low = max(0.0, self.lowest(*self.landing(sign, hip, v)))
        return (v[1] + math.sqrt(v[1] * v[1] + 2.0 * G * low)) / G

    def land_y(self):
        """The pelvis's height as a foot comes down at the speed asked, m."""
        hip = figure.hip(1.0, (0.0, 1.0, 0.0), rx(math.radians(self.ask['lean'])))
        return 1.0 - self.lowest(*self.landing(1.0, hip, (0.0, 0.0, self.ask['speed'])))

    # -- every pass --------------------------------------------------------------------------

    def swung(self, leg, sign, pel, now, v):
        """A free leg's six joints, rad, `leg['u']` of its way from where it left the floor to
        its landing pose: joint by joint, its knee folded on the way, its foot level."""
        a = self.ask
        ankle, foot = self.landing(sign, figure.hip(sign, pel, now), v, leg['to_go'],
                                   leg.get('reach', 0.0), leg.get('beside'))
        joints = list(figure.leg(sign, pel, now, ankle, foot))
        if leg.get('from') is None:
            return joints
        u = leg['u']
        coast = COAST_S * (1.0 - math.exp(-leg['t'] / COAST_S))
        joints = [p + r * coast + (q - p - r * coast) * eased(u)
                  for p, r, q in zip(leg['from'], leg['rate'], joints)]
        joints[3] += math.radians(a['fold']) * math.sin(math.pi * min(1.0, u / a['folded']))
        rise = (leg['rise'] * (1.0 - eased(u / LEVEL))
                + math.radians(a['land']) * eased((u - LEVEL) / (1.0 - LEVEL)))
        joints[4] = rise - (pitched(now) + joints[2] + joints[3])
        return joints

    def lifted(self, leg, sign, pel, now, joints):
        """What is free keeps its clearance (LIFT_M, CLEAR_M ..): a sole under it is raised to
        it, a foot passing the standing one moved out from it, the foot as it was."""
        u = leg.get('u', 1.0)
        clear = LIFT_M * eased(u / LIFTS_U) * (1.0 - eased((u - LANDS_U) / DOWN_U))
        ankle, foot = figure.foot_of(sign, pel, now, joints)
        # coming down, its heel and its ball alone - the toes give: counted, a forefoot landing
        # was lifted 1.6 cm, the run's knee 28 deg where 22, and she was down at 0.8 s
        low = min(add(ankle, apply(foot, q))[1]
                  for q in (SOLE_HEEL, SOLE_BALL) + (SOLE_TOE,) * (u <= LANDS_U))
        up = max(0.0, clear - low) if clear > 0.0 else 0.0
        c, s = math.cos(self.heading), math.sin(self.heading)
        out = 0.0
        if leg.get('beside') is not None:
            (bx, _by, bz), share = leg['beside']
            along = (ankle[0] - bx) * s + (ankle[2] - bz) * c
            near = 1.0 - eased((abs(along) - PASS_M[0]) / (PASS_M[1] - PASS_M[0]))
            out = max(0.0, share * near * CLEAR_M
                      - sign * ((ankle[0] - bx) * c - (ankle[2] - bz) * s))
        if up <= 0.0 and out <= 0.0:
            return joints
        return list(figure.leg(sign, pel, now, (ankle[0] + sign * out * c, ankle[1] + up,
                                                ankle[2] - sign * out * s), foot))

    def _upper(self, swing):
        """The upper body: the arms against the legs' swing {side: deg ahead}."""
        out = {'spine_roll': 0.0, 'spine': 0.0, 'waist': 0.0, 'neck': -0.5 * self.ask['lean'],
               'head': 0.0}
        for side in ('left', 'right'):
            out[side + '_shoulder'] = -ARM * swing[side]
            out[side + '_elbow'] = ELBOW_DEG
            out[side + '_wrist'], out[side + '_gripper'] = 0.0, 20.0
        return out

    def _landed(self, leg, other, sign, pel, now, v, at, on_heel, foot, last):
        """A foot down at `at` on the floor: it stands, and the height's law is laid anew."""
        a = self.ask
        flat = sub((at[0], 0.0, at[2]), apply(ry(self.heading), SOLE_HEEL if on_heel else SOLE_BALL))
        leg.update(stands=True, t=0.0, flat=flat, was=last, off=None,
                   landed=pitched(foot) if on_heel else max(0.0, pitched(foot)))
        # the bounce: to `up` over her landing and rising as asked - or, that over what the
        # strut will reach then, onto the strut as it will be going: ended 10 cm up it threw
        # her off the floor, the knee 37 to 6 deg in 0.1 s (2026-10-05)
        span = a['bounce']
        tops = [strut.top(sign, (pel[0] + v[0] * k, pel[2] + v[2] * k), now, flat, self.heading,
                          a['off']) for k in (span, span + 1e-3)]
        end, rate = pel[1] + a['up'], a['rise']
        if end >= tops[0]:
            end, rate = tops[0], (tops[1] - tops[0]) / 1e-3
        self.y = {'t': 0.0, 'span': span, 'landed': pel[1], 'rate': rate,
                  'from': (pel[1], v[1] * span,
                           -(0.0 if other['stands'] else START_G) * G * span ** 2),
                  'to': (end, rate * span, 0.0)}

    def _carried(self, standing, lead, pel, com, turn, dt):
        """({side: (ankle, foot's turn)}, the pelvis's target) with `standing` legs down, the
        one landed last `lead`: the bounce's height and on as she left it, never over what the
        legs that carry her reach; along the floor where she is, leaned as `hold.lean` has it."""
        a = self.ask
        self.y['t'] += dt
        # past the bounce she goes on as she left it, that rate dying over ON_S
        y = (curves.hermite(self.y['from'], self.y['to'], min(1.0, self.y['t'] / self.y['span']))
             + self.y['rate'] * ON_S * (1.0 - math.exp(-max(0.0, self.y['t'] - self.y['span'])
                                                       / ON_S)))
        sunk = self.y['landed'] - y
        c, s = math.cos(self.heading), math.sin(self.heading)
        on = {}
        for side in standing:
            ahead = sub(figure.hip(1.0 if side == 'left' else -1.0, (pel[0], y, pel[2]), turn),
                        self.legs[side]['flat'])
            on[side] = ahead[0] * s + ahead[2] * c
        # the leg landed last on its own heel's place; another, its heel as far up as it goes:
        # both down, the one that reaches her carries her - let fall 1.4 cm to the new leg's
        # reach she came onto a straight leg at 531 N, 0.92 to 0.5 m/s (2026-10-05)
        cap = max(strut.reach(1.0 if side == 'left' else -1.0, (pel[0], pel[2]), turn, strut.ankle(
            self.legs[side]['flat'], strut.rocker(self.legs[side], a['off'], on[side], sunk)
            if side == lead else math.radians(strut.HEEL_UP_DEG), self.heading))
            for side in standing)
        lean = hold.lean(self, standing, a, com)
        target, feet = (pel[0] + lean[0], min(y, cap), pel[2] + lean[1]), {}
        for side in standing:
            leg, sign = self.legs[side], 1.0 if side == 'left' else -1.0
            pitch = strut.rocker(leg, a['off'], on[side], sunk, strut.need(
                figure.hip(sign, target, turn), leg['flat'], self.heading))
            feet[side] = (strut.ankle(leg['flat'], pitch, self.heading),
                          mul(ry(self.heading), rx(pitch)))
        return feet, target

    def _due(self, leg, other, sign, pel, now, com, v, both, share):
        """The seconds till the free foot `leg` is due. In flight, her fall onto its pose. The
        other standing, that foot's `step` on - a run's - and, `share` of it, a walk's: as her
        hip is half a step past the standing foot's point - the strut's arc has brought her
        down to it -, hardly going its swing's time; or sooner, as her capture point has run as
        far across from that foot as a step's swing takes it from `track`, the pendulum's
        e^(omega t)."""
        a = self.ask
        if not other['stands']:
            fall = self.fall(sign, figure.hip(sign, pel, now), v)
            return fall if leg['t'] > other['t'] else fall + a['step']
        to_go = max(0.0, a['step'] - other['t'])
        if share <= 0.0:
            return to_go
        c, s = math.cos(self.heading), math.sin(self.heading)
        on = sub(figure.hip(-sign, pel, now), other['flat'])
        air = a['step'] - both
        due = max(max(0.0, (0.5 * a['speed'] * a['step'] + a['under'] - (on[0] * s + on[2] * c))
                      / max(SLOW_M_S, v[0] * s + v[2] * c)),
                  (air - leg['t']) * hold.stood(a['speed']))
        xi = hold.point(self, com)
        far = sign * ((xi[0] - other['flat'][0]) * c - (xi[1] - other['flat'][2]) * s)
        if far > 0.0:
            due = min(due, max(0.0, air + math.log(a['track'] / far) / self.omega))
        return to_go + share * (due - to_go)

    def step(self, dt):
        """{joint: degrees}: where every drive should be now."""
        bus, a = self.machine.loop.bus, self.ask
        both = a['stand'] - a['step']            # s both feet bear a step; under 0, her flight's
        share = self.share()
        pel = (bus['pelvis.pose.x'], bus['pelvis.pose.y'], bus['pelvis.pose.z'])
        now = figure.quat(bus['pelvis.pose.qw'], bus['pelvis.pose.qx'], bus['pelvis.pose.qy'],
                          bus['pelvis.pose.qz'])
        com = (bus['pelvis.pose.com_x'], bus['pelvis.pose.com_y'], bus['pelvis.pose.com_z'])
        if self.com is not None and dt > 0.0:
            k = min(1.0, dt / SPEED_FILTER_S)
            self.v = tuple(p + k * ((q - b) / dt - p) for p, b, q in zip(self.v, self.com, com))
        self.com, self.omega = com, math.sqrt(G / max(0.3, com[1]))
        v = (self.v[0], bus['pelvis.pose.vy'], self.v[2])
        c, s = math.cos(self.heading), math.sin(self.heading)
        # the events: a foot comes down; a foot has stood its stance
        seen = {}
        loads = {side: bus['pelvis.pose.%s_load' % side] for side, _sign in walkplan.SIDES}
        self.stays, self.parry = (hold.staying(self, a, loads) if both > 0.0 and all(
            leg['stands'] for leg in self.legs.values()) else (None, False))
        for side, sign in walkplan.SIDES:
            leg, other = self.legs[side], self.legs[walkplan._OTHER[side]]
            leg['t'] += dt
            angles = tuple(math.radians(bus.get(side + k + '.deg', 0.0)) for k in LEG)
            ankle, foot = figure.foot_of(sign, pel, now, angles)
            ball, back = add(ankle, apply(foot, SOLE_BALL)), add(ankle, apply(foot, SOLE_HEEL))
            last = seen[side] = tuple(math.radians(self.last.get(side + k, 0.0)) for k in LEG)
            down = loads[side] > BEARS_N or (leg.get('u', 1.0) >= LANDS_U and (
                min(ball[1], back[1]) <= TOUCH_M or loads[side] > LANDED_N))
            if (not leg['stands'] and down and leg['t'] > SWUNG_S
                    and (other['stands'] or v[1] <= 0.0)):
                on_heel = back[1] < ball[1]
                self._landed(leg, other, sign, pel, now, v, back if on_heel else ball, on_heel,
                             foot, last)
                on = pel[0] * s + pel[2] * c
                self.pace = ((on - self.mark[0]) / (bus['t'] - self.mark[1])
                             if self.mark is not None and bus['t'] > self.mark[1] else
                             v[0] * s + v[2] * c)
                self.mark = (on, bus['t'])
                self.bias = max(-BIAS_M, min(BIAS_M, self.bias + SPEED_I * (
                    self.pace - a['speed'])))
                self.steps.append({'t': bus['t'], 'side': side, 'v': v[0] * s + v[2] * c,
                                   'sink': v[1], 'y': pel[1], 'knee': math.degrees(angles[3]),
                                   'ahead': ball[2] - figure.hip(sign, pel, now)[2],
                                   'both': other['stands']})
            elif leg['stands'] and both <= (
                    other['t'] if other['stands'] else
                    -other['to_go'] if other.get('to_go') is not None else -math.inf) and (
                    both <= 0.0 or hold.leaves(self, side, a, loads, share)):
                # `both` s after the other landed - before it is due, `both` under 0: a run's
                # flight, its stance `stand` s as the other's step is on time
                self.stays = None
                leg.update(stands=False, t=0.0, rise=pitched(now) + last[2] + last[3] + last[4],
                           rate=self.rates.get(side, (0.0,) * 6), to_go=None, u=0.0, reach=0.0,
                           **{'from': last})
                row = next((r for r in reversed(self.steps) if r['side'] == side), None)
                if row is not None:
                    row.update(off=bus['t'], rise=v[1], y_off=pel[1])
        standing = [side for side, _sign in walkplan.SIDES if self.legs[side]['stands']]
        if len(standing) < 2:
            self.stays = None
        lead = min(standing, key=lambda side: self.legs[side]['t']) if standing else ''
        # her trunk: leaned as asked, the pelvis rolled over the leg that carries her
        roll = math.radians(a['list']) * {'left': -1.0, 'right': 1.0}.get(lead, 0.0)
        want = mul(mul(ry(self.heading), rx(math.radians(a['lean']))), figure.rz(-roll))
        turn = mul(walkplan.turned(tuple(TURN_K * e for e in walkplan.vee(mul(want, t(now))))),
                   want)
        feet, target = (self._carried(standing, lead, pel, com, turn, dt) if standing else
                        ({}, pel))
        out, swing = {}, {}
        for side, sign in walkplan.SIDES:
            leg, other = self.legs[side], self.legs[walkplan._OTHER[side]]
            if leg['stands']:
                joints = list(figure.leg(sign, target, turn, *feet[side]))
                if leg['off'] is None:
                    leg['off'] = tuple(p - q for p, q in zip(leg['was'], joints))
                fade = 1.0 - eased(leg['t'] / BLEND_S)
                joints = [q + b * fade for q, b in zip(joints, leg['off'])]
            else:
                to_go = self._due(leg, other, sign, pel, now, com, v, both, share)
                leg['beside'] = (other['flat'], share) if other['stands'] and share > 0.0 else None
                if leg.get('to_go') is not None:
                    to_go = max(leg['to_go'] - 3.0 * dt, min(leg['to_go'] + dt, to_go))
                leg['to_go'] = to_go
                u = leg.get('u', 1.0)
                leg['u'] = min(1.0, u + dt * (1.0 - u) / max(to_go - READY_S, READY_S))
                if leg['u'] >= 0.999 and other['stands']:
                    leg['reach'] = leg.get('reach', 0.0) + REACH_M_S * dt
                joints = self.lifted(leg, sign, pel, now, self.swung(leg, sign, pel, now, v))
            if dt > 0.0:
                self.rates[side] = tuple((q - b) / dt for q, b in zip(joints, seen[side]))
            for k, q in zip(LEG, joints):
                out[side + k] = math.degrees(q)
            out[side + '_foot'] = 0.0
            swing[side] = -out[side + '_hip']
        out.update(self._upper(swing))
        self.last = out
        return out
