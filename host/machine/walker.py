"""The gynoid's moves set on the fly: the walk planned from the feet, legs by IK, balance fed back.

    walker = Walker(machine)                 # a DYNAMIC gynoid (`machine.physics`)
    walker.start()                           # the body placed mid-stride, moving
    machine.loop.write(**walker.step(dt))    # every pass: the setpoints from what the loop read
    machine.loop.step(dt)

The plan is `machine.gait`'s walk, tabled by stride over the phase: the pelvis's place and turn,
each foot's. Each pass the phase is pulled to where the pelvis stands over the stance balls; a
stance foot is held where it landed and the pelvis set from it; a swing foot is reached for from
where the pelvis is, and put down further the way she sways; the pelvis's attitude and its sideways
error are turned back past the plan. Narrow on the line, the legs stand in a V and the body is an
inverted pendulum on them: without the feedback she fell in two seconds (2026-09-25).
"""
import math

from machine import figure, gait
from machine.figure import LEG, SOLE_BALL, SOLE_HEEL, add, apply, mul, rx, ry, rz, sub, t

#: The phase pulled to the body, strides a second per stride of error.
PULL = 5.0

#: The pelvis's sideways error and its speed fed back, 1 and s; the swing foot put down further
#: the way the pelvis moves, s; its attitude turned back past the plan, 1. With the gait's
#: shape, the finest of 576 walks at 0.65, 0.8 and 0.9 strides/s, judged by the stance knee's
#: bend, the landing's peak, the pelvis's shake, the head's travel and the copper's heat: bend
#: 11.7 -> 5.7 degrees past 5, peak 2.9 -> 2.2 body weights, head 8.5 -> 6.0 cm, heat 0.47 ->
#: 0.29 and work 0.60 -> 0.35 m g d (2026-09-26).
SIDE_K, SIDE_D, STEP_D, TURN_K = 0.235, 0.069, 0.057, 1.42

#: The pelvis's sideways speed is filtered over SPEED_S, s: raw, every landing's jolt went straight
#: into the feedback, and she fell in two seconds.
SPEED_S = 0.005

#: A foot takes the weight over ACCEPT of a stride from its landing, and gives it up over UNLOAD
#: before toe-off: taken at once, a landing off its mark jolted the pelvis.
ACCEPT, UNLOAD = 0.04, 0.06

#: How much of the pelvis's pitch the spine takes back out: riding the pelvis, the torso
#: pitched 8 degrees a step and the head bobbed 9 cm (2026-09-25).
PLUMB = 1.0

#: A catwalk: the feet planted TRACK_M off the line, swung WIDEN_M further out round the standing
#: one; the pelvis turned TURN_GAIN of the walk's turn, the torso turning it back.
TRACK_M, WIDEN_M, TURN_GAIN = 0.02, 0.023, 1.3

#: From standing on the left foot: the stride grows from BEGIN of a stride's to all of it over
#: RAMP_S, the setpoints ease from the stand's over BLEND_S; the phase starts at BEGIN_AT, the left
#: foot flat, the right lifted into its swing - begun at its toe-off, the plan tipped a flat foot
#: onto its ball and it threw her (2026-09-25).
BEGIN, RAMP_S, BLEND_S, BEGIN_AT = 0.1, 2.5, 0.3, 0.30

#: From standing, the phase runs at EASE_IN of the cadence at first, and is pulled to the body
#: from PULL_FROM of the walk's stride: pulled only past half, it ran ahead of her first step and
#: the front leg, straightened for a pelvis not yet over it, lifted her (2026-09-26).
EASE_IN, PULL_FROM = 0.8, 0.2

#: The pull slows the phase to LEAD of its pace at most: pulled to a body standing still, the
#: phase stood still with it, one foot in the air for a second, and she toppled off the other
#: (2026-09-26).
LEAD = 0.5

#: To a stop (`halt`): over HALT_S the stride shrinks to HALT of a stride's, the first stride's
#: (`arrival.FIRST`), and the pace to HALT_PACE of the cadence. Down to 0.1 in 2 s, the hips rose
#: with the short stride's reach and threw her off the floor, 0.3 s on no foot; at the pace kept,
#: she came onto the front foot at 0.55 m/s and on over its ball (2026-09-26).
HALT, HALT_S, HALT_PACE = 0.6, 1.5, 0.7

#: Standing, the feet are STAND_WIDE_M further out a side than the catwalk's: the steps glide in
#: from there as she starts (`begin`) and out to it as she stops (`halt`), the pelvis between
#: them. Only the swinging foot spread, the plan put her pelvis over the spread stance foot, and
#: she swayed off it (2026-09-26).
STAND_WIDE_M = 0.06

#: A sole bearing this much has landed, N; bearing BEARS_N it is all stance, up to BEARS_UNTIL
#: of a stride past its toe-off.
LANDED_N, BEARS_N, BEARS_UNTIL = 60.0, 250.0, 0.06

#: The table's phases a stride.
SAMPLES = 240

SIDES = (('left', 1.0), ('right', -1.0))


def carried(q):
    """How much a leg carries at its phase `q`: 0 in the air, 1 in stance, eased between."""
    q %= 1.0
    if q >= gait.TOE_OFF:
        return 0.0
    return gait.eased(min(q / ACCEPT, (gait.TOE_OFF - q) / UNLOAD))


def _sample(p, stride):
    """The walk at the left leg's phase `p`: (pelvis lateral, height, yaw rad, per leg (ankle
    from the pelvis x y z, twist rad, pitch rad, toes deg), the upper body's joints deg, the
    pelvis's roll rad - dropped on the swing side, the spine taking it back out)."""
    angles = gait.walk(0.0, stride=stride, glance=False, phase=p)
    yaw = math.radians(angles['pelvis'] * TURN_GAIN)
    angles['waist'] *= TURN_GAIN
    lateral, roll, height, _level = gait.sway(0.0, stride=stride, phase=p)
    left, right, twist_l, twist_r = gait.tracks(0.0, phase=p, track=TRACK_M, widen=WIDEN_M)
    legs = []
    for q, track, twist in ((p, left, twist_l), ((p + 0.5) % 1.0, right, twist_r)):
        x, y = gait.planted(q, stride)[:2] if q < gait.TOE_OFF else gait.swung(q, stride)
        pitch = gait.pitch_of(q)
        legs.append(((track - lateral, y, x), math.radians(twist), math.radians(pitch),
                     gait.toes_of(q, pitch)))
    angles['spine_roll'] = -roll
    upper = tuple(angles.get(j, 0.0) for j in UPPER)
    return lateral, height, yaw, tuple(legs), upper, math.radians(roll)


#: The joints the plan sets directly: the spine and head, the arms.
UPPER = tuple(j for j in figure.JOINTS
              if not j.endswith(tuple(LEG)) and not j.endswith('_foot'))


class _Tables(dict):
    def __missing__(self, stride):
        self[stride] = [_sample(k / SAMPLES, stride) for k in range(SAMPLES)]
        return self[stride]


#: The plan by stride, every `gait.PACE_STEP`, tabled on first ask.
_TABLES = _Tables()


def _mix(a, b, k):
    """`a` k of the way to `b`, element by element, through nested tuples."""
    if isinstance(a, tuple):
        return tuple(_mix(x, y, k) for x, y in zip(a, b))
    return a + (b - a) * k


def plan(p, stride) -> tuple:
    """The walk at phase `p` and `stride`, as `_sample` answers, from the tables."""
    low = math.floor(stride / gait.PACE_STEP) * gait.PACE_STEP
    w = (stride - low) / gait.PACE_STEP
    x = (p % 1.0) * SAMPLES
    i, u = int(x), x - int(x)

    def at(table):
        a, b = table[i % SAMPLES], table[(i + 1) % SAMPLES]
        if abs(b[2] - a[2]) > math.pi:
            b = (b[0], b[1], a[2]) + b[3:]
        return _mix(a, b, u)
    below = at(_TABLES[round(max(low, gait.PACE_STEP), 6)])
    if w < 1e-9:
        return below
    return _mix(below, at(_TABLES[round(low + gait.PACE_STEP, 6)]), w)


def _vee(r):
    """The small turn a rotation is, as a vector: axis times angle for small angles."""
    return ((r[2][1] - r[1][2]) / 2.0, (r[0][2] - r[2][0]) / 2.0, (r[1][0] - r[0][1]) / 2.0)


def _turned(v):
    """The rotation by the vector `v`: about its axis, its length in radians (Rodrigues)."""
    angle = math.sqrt(sum(c * c for c in v))
    if angle < 1e-12:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    k = tuple(c / angle for c in v)
    kx = ((0.0, -k[2], k[1]), (k[2], 0.0, -k[0]), (-k[1], k[0], 0.0))
    kk = mul(kx, kx)
    s, c = math.sin(angle), 1.0 - math.cos(angle)
    return tuple(tuple((1.0 if i == j else 0.0) + s * kx[i][j] + c * kk[i][j] for j in range(3))
                 for i in range(3))


class Walker:

    """The setpoints for a DYNAMIC gynoid, every pass, from what the loop read."""

    def __init__(self, machine, cadence=gait.CADENCE):
        self.machine, self.cadence = machine, float(cadence)
        self.world = machine.nodes['pelvis'].world
        self.phase = 0.0
        self.anchor, self.was_q = {}, {}
        self.side_was, self.x_was, self.v_side = None, None, 0.0
        self.scale, self.age, self.held, self.wide, self.first = 1.0, 0.0, None, 0.0, 1.0
        self.lift = None
        #: The last setpoints; each ball where it stands; the pelvis's attitude off the plan's,
        #: deg.
        self.last, self.balls, self.tilt = None, {}, 0.0
        #: A catch (`machine.director`): the swinging foot put down `catch` s further the way
        #: the pelvis moves, the phase `hurry` faster.
        self.catch, self.hurry = 0.0, 0.0
        #: Seconds since `halt`, None walking on; the stride's length last pass, m.
        self.halting, self.halt_from, self.length_was = None, 1.0, None

    def halt(self):
        """To a stop over HALT_S: the stride down to HALT of its own; `halted` from then."""
        self.halting, self.halt_from, self.held = 0.0, self.scale, None

    @property
    def halted(self):
        return self.halting is not None and self.halting >= HALT_S

    @property
    def stride(self):
        return max(gait.PACE_STEP, gait.pace(self.cadence) * self.scale)

    def start(self):
        """The body placed mid-stride at phase 0: the plan's pose, moving at its speed."""
        stride, dt = self.stride, 1e-3
        poses = []
        for p in (0.0, dt * self.cadence):
            lateral, height, yaw, legs, upper, roll = plan(p, stride)
            pelvis = (lateral, height, p * gait.STRIDE_M * stride)
            angles = dict(zip(UPPER, upper))
            for (side, sign), (ankle, twist, pitch, toes) in zip(SIDES, legs):
                foot = mul(ry(twist), rx(-pitch))
                at = add(pelvis, (ankle[0], ankle[1] - height, ankle[2]))
                for k, v in zip(LEG, figure.leg(sign, pelvis, mul(ry(yaw), rz(roll)), at, foot)):
                    angles[side + k] = math.degrees(v)
                angles[side + '_foot'] = toes
            poses.append((pelvis, yaw, roll, angles))
        (pelvis, yaw, roll, angles), (ahead, _yaw, _roll, later) = poses
        c, s, cr, sr = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(roll / 2), math.sin(roll / 2)
        self.world.reset(angles, where=pelvis, turn=(c * cr, s * sr, s * cr, c * sr),
                         rates={j: (later[j] - angles[j]) / dt for j in angles},
                         speed=tuple((b - a) / dt for a, b in zip(pelvis, ahead)))
        self.phase, self.anchor, self.was_q = 0.0, {}, {}
        self.side_was, self.x_was, self.v_side = None, None, 0.0
        self.scale, self.held, self.halting, self.length_was = 1.0, None, None, None
        return angles

    def begin(self, held, wide=0.0, scale=None, phase=None):
        """Walking from where she stands on her left foot, the right lifted: {joint: deg}
        `held` the stand's setpoints, eased out of over BLEND_S; the first steps `wide` m further
        out than the walk's, narrowing as the stride grows."""
        self.phase, self.anchor, self.was_q = BEGIN_AT if phase is None else phase, {}, {}
        self.side_was, self.x_was, self.v_side = None, None, 0.0
        self.scale, self.age, self.held, self.wide = BEGIN, 0.0, dict(held), float(wide)
        self.first = BEGIN if scale is None else float(scale)
        self.scale = self.first
        self.lift, self.halting, self.length_was = None, None, None

    def step(self, dt):
        """{joint: degrees}: where every drive should be now."""
        bus = self.machine.loop.bus
        pel = (bus['pelvis.pose.x'], bus['pelvis.pose.y'], bus['pelvis.pose.z'])
        turn_now = figure.quat(bus['pelvis.pose.qw'], bus['pelvis.pose.qx'], bus['pelvis.pose.qy'],
                         bus['pelvis.pose.qz'])
        if self.x_was is not None:
            self.v_side += ((pel[0] - self.x_was) / dt - self.v_side) * min(1.0, dt / SPEED_S)
        self.x_was = pel[0]
        v_side = self.v_side
        if self.halting is not None:
            self.halting += dt
            self.scale = self.halt_from + (HALT - self.halt_from) * gait.eased(self.halting / HALT_S)
        stride = self.stride
        length = gait.STRIDE_M * stride
        balls = {}
        for side, sign in SIDES:
            angles = tuple(math.radians(bus.get(side + k + '.deg', 0.0)) for k in LEG)
            at = figure.ball(sign, pel, turn_now, angles)
            balls[side] = (at[0], 0.0, at[2])
        self.balls = balls
        self._advance(dt, length, balls, pel)
        lateral, height, yaw, legs, upper, roll = plan(self.phase, stride)
        spread = self.wide * (1.0 - gait.eased(self.age / RAMP_S))
        if self.halting is not None:
            spread += STAND_WIDE_M * gait.eased(self.halting / HALT_S)
        legs = tuple(((a[0] + sign * spread, a[1], a[2]),) + tuple(rest)
                     for (_side, sign), (a, *rest) in zip(SIDES, legs))
        height, yaw, roll = float(height), float(yaw), float(roll)
        if self.held is not None:
            # From standing, the pelvis's height eases from where it stood to the walk's: the
            # walk's short first strides ride 7 cm higher, and taken at once she hopped.
            if self.lift is None:
                self.lift = pel[1] - height
            height += self.lift * (1.0 - gait.eased(self.age / RAMP_S))
        qs = (self.phase, (self.phase + 0.5) % 1.0)
        # A foot is held where it landed until it leaves: landed once its sole bears, or ACCEPT
        # into its stance whatever it bears. Held from the phase alone, a foot still in the air
        # was taken to bear her, and she fell when the other left (2026-09-25).
        for (side, _sign), q in zip(SIDES, qs):
            if q >= gait.TOE_OFF:
                self.anchor.pop(side, None)
            elif side not in self.anchor and (bus['pelvis.pose.%s_load' % side] > LANDED_N
                                              or q >= ACCEPT):
                self.anchor[side] = balls[side]
            self.was_q[side] = q
        feet = [mul(ry(twist), rx(-pitch)) for _ankle, twist, pitch, _toes in legs]
        # A stance ankle where its foot, rolling on its heel or its ball, keeps that on the floor
        # where the anchor says.
        held = {side: sub(add(self.anchor[side], apply(ry(twist), sub(p, SOLE_BALL))),
                          apply(foot, p))
                for (side, _sign), (_an, twist, pitch, _to), foot in zip(SIDES, legs, feet)
                if side in self.anchor
                for p in ((SOLE_HEEL if pitch > 0.0 else SOLE_BALL),)}
        # The pelvis where the stance feet say, weighed; sideways, its error turned back past.
        est, weight = [0.0, 0.0, 0.0], 0.0
        for (side, _sign), q, (ankle, _tw, _pi, _to) in zip(SIDES, qs, legs):
            b = carried(q)
            if b > 0.0 and side in held:
                est = [e + b * c for e, c in zip(est, sub(held[side], ankle))]
                weight += b
        planned = (est[0] / weight, height, est[2] / weight) if weight else (pel[0], height, pel[2])
        v_ref = 0.0 if self.side_was is None else (planned[0] - self.side_was) / dt
        self.side_was = planned[0]
        target = (planned[0] - SIDE_K * (pel[0] - planned[0]) - SIDE_D * (v_side - v_ref),
                  height, planned[2])
        turn = mul(ry(yaw), rz(roll))
        off = _vee(mul(turn, t(turn_now)))
        self.tilt = math.degrees(math.sqrt(sum(c * c for c in off)))
        turn = mul(_turned(tuple(TURN_K * c for c in off)), turn)
        target = self._reachable(target, turn, held, qs)
        out = dict(zip(UPPER, upper))
        # The plumb line: the spine takes the pelvis's pitch back out, the torso upright; the neck
        # what the torso still leans, the head level.
        pitch = math.degrees(math.atan2(turn_now[2][1], turn_now[1][1]))
        out['spine'] = -PLUMB * pitch
        out['neck'] = out['neck'] - (pitch + bus.get('spine.deg', 0.0))
        for (side, sign), q, (ankle, _tw, _pi, toes), foot in zip(SIDES, qs, legs, feet):
            b = carried(q)
            free = (pel[0] + ankle[0] + (STEP_D + self.catch) * (v_side - v_ref), ankle[1],
                    planned[2] + ankle[2])
            if b > 0.0 and side in held:
                at = tuple(b * h + (1.0 - b) * f for h, f in zip(held[side], free))
            else:
                at = free
            # A stance leg carries the pelvis to where it should be; a swing leg reaches from
            # where the pelvis is, as it is turned. A leg still bearing is a stance leg whatever
            # the phase says: reached from the pelvis as it stood while it bore, the rear heel's
            # rise lifted her and the front foot hung off the floor (2026-09-26).
            if q < gait.TOE_OFF + BEARS_UNTIL and bus['pelvis.pose.%s_load' % side] > LANDED_N:
                b = max(b, min(1.0, bus['pelvis.pose.%s_load' % side] / BEARS_N))
            hip_from = tuple(b * a + (1.0 - b) * c for a, c in zip(target, pel))
            reach = turn if b >= 1.0 else mul(_turned(tuple(b * c for c in _vee(mul(turn, t(turn_now))))), turn_now)
            for k, v in zip(LEG, figure.leg(sign, hip_from, reach, at, foot)):
                out[side + k] = math.degrees(v)
            out[side + '_foot'] = toes
        if self.held is not None:
            self.age += dt
            self.scale = self.first + (1.0 - self.first) * gait.eased(self.age / RAMP_S)
            k = gait.eased(self.age / BLEND_S)
            out = {j: self.held.get(j, v) + (v - self.held.get(j, v)) * k for j, v in out.items()}
            if self.age >= RAMP_S:
                self.held = None
        self.last = out
        return out

    def _advance(self, dt, length, balls, pel):
        """The phase on by a pass: carried across a change of stride, at its pace, pulled to the
        body."""
        # A change of stride carries the phase with it, each stance foot kept where it stands under
        # her (`gait.planted`: L (STANCE_AT - q) ahead of the hip), weighed. Left to the pull, a
        # stride shortened to a stop sped her up, bounced her off the floor and threw her
        # (2026-09-26).
        if self.length_was is not None and abs(length - self.length_was) > 1e-12:
            shift = weight = 0.0
            for q in (self.phase, (self.phase + 0.5) % 1.0):
                b = carried(q)
                shift += b * (1.0 - self.length_was / length) * (gait.STANCE_AT - q)
                weight += b
            if weight > 0.0:
                self.phase = (self.phase + shift / weight) % 1.0
        self.length_was = length
        # The phase pulled to where the pelvis stands over each stance ball.
        err = weight = 0.0
        for side, q in (('left', self.phase), ('right', (self.phase + 0.5) % 1.0)):
            if 0.05 < q < gait.TOE_OFF - 0.05:
                b = carried(q)
                err += b * ((length * gait.STANCE_AT + gait.BALL - (balls[side][2] - pel[2]))
                            / length - q)
                weight += b
        # A short stride makes a centimetre a large phase: no pull under PULL_FROM of the walk's.
        pull = PULL * max(0.0, min(1.0, (self.scale - PULL_FROM) / (1.0 - PULL_FROM)))
        # From standing the phase starts slow: at full cadence from rest, the rear heel's push threw
        # her up instead of on (2026-09-25).
        pace = self.cadence * (1.0 if self.held is None else
                               EASE_IN + (1.0 - EASE_IN) * gait.eased(self.age / RAMP_S))
        pace *= 1.0 + self.hurry
        if self.halting is not None:
            pace *= 1.0 + (HALT_PACE - 1.0) * gait.eased(self.halting / HALT_S)
        rate = max(LEAD * pace, pace + pull * (err / weight if weight else 0.0))
        self.phase = (self.phase + dt * rate) % 1.0

    def _reachable(self, target, turn, held, qs):
        """The pelvis's target, lowered to where each stance leg reaches it."""
        # Never higher than a stance leg reaches (`gait.REACH`): a foot landed on a longer stride
        # than the plan's now - shortening to a stop - held her up on a straight leg, and its
        # heel's rise threw her off the floor (2026-09-26).
        reach = gait.REACH
        for (side, sign), q in zip(SIDES, qs):
            if side in held and carried(q) > 0.0:
                hip = figure.hip(sign, target, turn)
                d2 = (hip[0] - held[side][0]) ** 2 + (hip[2] - held[side][2]) ** 2
                over = hip[1] - held[side][1] - math.sqrt(max(0.0, reach * reach - d2))
                if over > 0.0:
                    target = (target[0], target[1] - carried(q) * over, target[2])
        return target

    def tilt_deg(self):
        """The pelvis's attitude off the plan's as last read, degrees: the plan leans, rolls and
        turns it 8 degrees on its own, and taken from plumb every first step was a catch
        (2026-09-26)."""
        return self.tilt
