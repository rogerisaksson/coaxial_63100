"""The gynoid's moves set on the fly: the walk planned from the feet, legs by IK, balance fed back.

    walker = Walker(machine)                 # a DYNAMIC gynoid (`machine.physics`)
    walker.start()                           # the body placed mid-stride, moving
    machine.loop.write(**walker.step(dt))    # every pass: the setpoints from what the loop read
    machine.loop.step(dt)

The plan is `machine.gait`'s walk, tabled by stride over the phase: the pelvis's place and turn,
each foot's. Each pass the phase is pulled to where the pelvis stands over the stance balls; a
stance foot is held where it landed and the pelvis set from it; a swing foot is put down on the
capture point, a step's own offset out from it, and feet are swapped when that would cross them;
the pelvis's attitude and its sideways error are turned back past the plan. Narrow on the line,
the legs stand in a V and the body is an inverted pendulum on them: without the feedback she fell
in two seconds (2026-09-25).
"""
import math

from machine import capture, figure, gait
from machine.pendulum import SPINE_TO_EARS_M, Pendulum
from machine.figure import LEG, SOLE_BALL, SOLE_HEEL, add, apply, mul, rx, ry, rz, sub, t

#: The phase pulled to the body, strides a second per stride of error.
PULL = 5.0

#: The pelvis's sideways error and its speed fed back, 1 and s; its attitude turned back past
#: the plan, 1. With the gait's shape, the finest of 576 walks at 0.65, 0.8 and 0.9 strides/s,
#: judged by the stance knee's bend, the landing's peak, the pelvis's shake, the head's travel
#: and the copper's heat: bend 11.7 -> 5.7 degrees past 5, peak 2.9 -> 2.2 body weights, head
#: 8.5 -> 6.0 cm, heat 0.47 -> 0.29 and work 0.60 -> 0.35 m g d (2026-09-26). Driven by the
#: capture point's error from the plan's place and speed instead, 1.2 of it, the standing foot
#: slid 20 cm after a shove; by what it is off the walk's course alone, she fell in 1.4 s
#: (2026-09-26).
SIDE_K, SIDE_D, TURN_K = 0.235, 0.069, 1.42

#: Where a swinging foot lands across is `machine.capture`'s: on the capture point as it will be
#: at the landing. Put down from the pelvis instead, a shove toward the standing foot took the
#: swinging one across it and she fell in 0.3 s; the capture point past the standing foot ran
#: 109 -> 425 mm in 0.42 s once the other lifted (2026-09-26).

#: A swap is a side step outside the plan's phase: the swinging foot is put down where it is,
#: flat, lowered over DOWN_S, and once it bears BEARS_N for DWELL_S the standing one steps out to
#: where the capture point will be when it lands, foreseen with no ankle to help, SIDE_AHEAD_M
#: ahead of the pelvis - put 5 cm behind, the walk began again on it in its push-off and the
#: heel's rise threw her 3 cm up; 5 cm ahead, the leg reaching for it sank her onto the bent
#: rear leg and she pitched over it (2026-09-26) - lifted SIDE_LIFT_M at the middle of SIDE_S
#: seconds and set down as softly
#: as lifted - an arc of sine struck at 0.9 m/s, 2100 N, and the leg threw her 18 cm up
#: (2026-09-26); the phase SIDE_HURRY faster meanwhile. Left to the plan, the swapped foot landed
#: 0.24 s after the shove and the other lifted 0.12 s later still, the capture point 20 cm out by
#: then; dropped at once it struck 1200 N and bounced; the other lifted on 60 N of one pass and
#: she hung in the air; the step out sized once, with the ankle's help, was 11 cm of the 20
#: needed (2026-09-26).
DOWN_S, DWELL_S, SIDE_AHEAD_M, SIDE_LIFT_M, SIDE_S, SIDE_HURRY = 0.1, 0.02, 0.1, 0.06, 0.15, 0.3

#: The swapped foot is put down DOWN_AHEAD_M ahead of the pelvis, m: where it was, the body walked
#: on past it, the leg could not reach back, and the foot left the floor (2026-09-26).
DOWN_AHEAD_M = 0.15

#: The side step is over once the stepped-out foot bears BEARS_N and more than the other; if the
#: other still bears her SIDE_GIVE_S on, she has come back to it, and the walk begins again on
#: that one. Ended as the stepped-out foot touched, the plan lifted the one that bore her
#: (2026-09-26).
SIDE_GIVE_S = 0.4

#: The pelvis's forward target within LURCH_M of the pelvis, m, and moved from it no faster
#: than LURCH_M_S: a foot put down short of the plan's spot had the phase race to catch up and
#: the target run off at 2 m/s; landed 5 cm short after a shove, the target moved 5 cm back in
#: 30 ms, the landed knee snapped straight and threw her 5 cm up (2026-09-26).
LURCH_M, LURCH_M_S = 0.15, 0.3

#: The capture point is the centre of mass plus the pelvis's sideways speed over omega, not the
#: centre of mass's own: the swinging leg's speed is in that, and a wide step's put the capture
#: point 6 cm out past where it went - the foot chased itself (2026-09-26).

#: No foot waits at toe-off for the capture point to come over the other: held there 0.3 s
#: while the legs drove the pelvis toward the standing foot, the pelvis moved 3 mm; walking on,
#: the plan froze while the body went 24 cm past the standing foot, the leg could not reach and
#: she sank 17 cm (2026-09-26).

#: A swinging foot is reached for no further than SWING_REACH of a leg's: asked further, its step
#: is shortened first and the pelvis lowered the rest, LOWER_M at most, LOWER_M_S a second down
#: and RAISE_M_S up - lowered at once, the standing knee bent before the body had sunk and the
#: foot left the floor; raised as fast, the leg threw her 13 cm into the air (2026-09-26).
SWING_REACH, LOWER_M, LOWER_M_S, RAISE_M_S = 0.985, 0.12, 0.4, 0.1

#: In a side step the attitude is turned back past its error by no more than SIDE_TURN_RAD: the
#: legs are solved for the turned pelvis, and 8 degrees of it dragged the planted foot 12 cm
#: (2026-09-26).
SIDE_TURN_RAD = 0.035

#: A standing foot does not lift while the other bears under LANDED_N, LAND_WAIT_S at most: its
#: phase pinned PIN short of toe-off, a stride growing carried it 2e-5 a pass past a 1e-6 pin
#: (2026-09-26).
LAND_WAIT_S, PIN = 0.3, 1e-3

#: A side step ends in the walk begun again (`begin`) at the stride her speed says, RESUME of the
#: walk's at least, her speed on filtered over ON_S, s.
RESUME, ON_S = 0.3, 0.05


#: The pelvis's sideways speed is filtered over SPEED_S, s: raw, every landing's jolt went straight
#: into the feedback, and she fell in two seconds.
SPEED_S = 0.005

#: The pelvis is driven across no further than SOLE_M from where it is, m - what the ankle can do
#: on the sole: asked 10 cm across after a side step, the ankle saturated, the foot tipped on its
#: edge and threw her up (2026-09-26).
SOLE_M = 0.035

#: The ankle's sideways drive is let go as the capture point leaves the standing feet's soles by
#: HOLD_M, m, the damping kept: driven over a wide foot the capture point was 13 cm inside of,
#: the leg leaned her on over it (2026-09-26).
HOLD_M = 0.07

#: A foot takes the weight over ACCEPT of a stride from its landing, and gives it up over UNLOAD
#: before toe-off: taken at once, a landing off its mark jolted the pelvis.
ACCEPT, UNLOAD = 0.04, 0.06

#: How much of the pelvis's pitch the spine takes back out: riding the pelvis, the torso
#: pitched 8 degrees a step and the head bobbed 9 cm (2026-09-25).
PLUMB = 1.0

#: The torso counters her surge, twice a stride: the spine SURGE_DEG back at SURGE_AT of the left
#: leg's stride and each half stride on, as far forward between, less on a shorter stride - her
#: head carried on at an even speed, the pendulum between her ears still (`machine.pendulum`).
#: At 2 degrees, SWAY_K 1: the pendulum's stir over 30 s at 0.85 and 0.9 strides/s 4.3, 4.9 ->
#: 1.7, 1.9 mm, but rising and gliding to another pace she fell three times in four; off until a
#: search over disturbances finds a pair that holds (2026-09-26).
SURGE_DEG, SURGE_AT = 0.0, 0.125

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

#: The pull slows the phase to LEAD of its pace at most, and hurries it to 1/LEAD: pulled to a
#: body standing still, the phase stood still with it, one foot in the air for a second, and she
#: toppled off the other; thrown forward off a short landing, it raced to 3.5 strides/s
#: (2026-09-26).
LEAD = 0.5

#: To a stop (`halt`): over HALT_S the stride shrinks to HALT of a stride's, the first stride's
#: (`arrival.FIRST`), and the pace to HALT_PACE of the cadence. Down to 0.1 in 2 s, the hips rose
#: with the short stride's reach and threw her off the floor, 0.3 s on no foot; at the pace kept,
#: she came onto the front foot at 0.55 m/s and on over its ball (2026-09-26).
HALT, HALT_S, HALT_PACE = 0.6, 1.5, 0.7

#: Standing, the feet are STAND_WIDE_M further out a side than the catwalk's: the steps glide in
#: from there over WIDE_S as she starts (`begin`) and out to it as she stops (`halt`), the pelvis
#: between them. Only the swinging foot spread, the plan put her pelvis over the spread stance
#: foot, and she swayed off it; narrowed with the stride over 2.5 s, the third step lifted with
#: the capture point 8 cm inside the standing foot, the legs driven 3.5 cm across had moved the
#: pelvis 3 mm in 0.2 s, and she fell off it (2026-09-26).
STAND_WIDE_M, WIDE_S = 0.06, 0.6

#: The pendulum between her ears damped as a crane damps its load: her head moved toward the bob
#: by SWAY_K of its offset, on and across, through the spine's pitch and roll
#: (`machine.pendulum`). Moved by its drift too, the landings' jolts shook her down in 2 s
#: (2026-09-26).
SWAY_K = 0.0

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


FLAT = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _reach(hip, at, reach):
    """`at`, an ankle's target, within `reach` of the hip: its step shortened first, then the
    ankle held up; (the target, how far the pelvis must come down for it to reach as asked)."""
    dx, dy, dz = (a - h for a, h in zip(at, hip))
    if dx * dx + dy * dy + dz * dz <= reach * reach:
        return at, 0.0
    room = reach * reach - dx * dx - dy * dy
    if room >= 0.0:
        return (at[0], at[1], hip[2] + math.copysign(math.sqrt(room), dz)), 0.0
    dx = max(-0.9 * reach, min(0.9 * reach, dx))
    low = -math.sqrt(reach * reach - dx * dx)
    return (hip[0] + dx, hip[1] + low, hip[2]), low - dy


class Walker:

    """The setpoints for a DYNAMIC gynoid, every pass, from what the loop read."""

    def __init__(self, machine, cadence=gait.CADENCE):
        self.machine, self.cadence = machine, float(cadence)
        self.world = machine.nodes['pelvis'].world
        self.phase = 0.0
        self.anchor, self.was_q = {}, {}
        self.x_was, self.v_side, self.z_was, self.v_on = None, 0.0, None, 0.0
        self.scale, self.age, self.held, self.wide, self.first = 1.0, 0.0, None, 0.0, 1.0
        self.lift = None
        #: The last setpoints; each ball where it stands; where each foot last stood, x; the
        #: pelvis as planned and as targeted.
        self.last, self.balls, self.stood, self.planned, self.target = None, {}, {}, None, None
        #: A catch under way (`machine.director`); the phase run `hurry` faster; the landings'
        #: rows (`machine.capture`); the side step under way (`_sidestep`), None walking; the
        #: foot the walk begins again on after one.
        self.catching, self.hurry, self.capture = False, 0.0, capture.state()
        self.side, self.resume = None, None
        #: The phase's rate, strides/s; how long a standing foot has waited for the other to bear;
        #: how far the pelvis's target is lowered, m: for a swinging foot to reach, from a
        #: landing.
        self.rate, self.waited, self.lowered = float(cadence), 0.0, 0.0
        #: The forward target's offset from the pelvis last pass, m; None to begin with.
        self.lurch = None
        #: Seconds since `halt`, None walking on; the stride's length last pass, m.
        self.halting, self.halt_from, self.length_was = None, 1.0, None
        #: The pendulum between her ears, read each pass (`machine.pendulum`).
        self.pendulum = Pendulum()

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
        self.phase, self.anchor, self.was_q, self.stood = 0.0, {}, {}, {}
        self.x_was, self.v_side, self.z_was, self.v_on = None, 0.0, None, 0.0
        self.scale, self.held, self.halting, self.length_was = 1.0, None, None, None
        self.capture, self.side, self.resume, self.hurry = capture.state(), None, None, 0.0
        self.rate, self.waited, self.lurch = self.cadence, 0.0, None
        return angles

    def begin(self, held, wide=0.0, scale=None, phase=None):
        """Walking from where she stands on her left foot, the right lifted: {joint: deg}
        `held` the stand's setpoints, eased out of over BLEND_S; the first steps `wide` m further
        out than the walk's, narrowing over WIDE_S."""
        self.phase, self.anchor, self.was_q = BEGIN_AT if phase is None else phase, {}, {}
        self.stood, self.x_was, self.v_side = {}, None, 0.0
        self.scale, self.age, self.held, self.wide = BEGIN, 0.0, dict(held), float(wide)
        self.first = BEGIN if scale is None else float(scale)
        self.scale = self.first
        self.lift, self.halting, self.length_was = None, None, None
        self.capture, self.side, self.resume, self.hurry = capture.state(), None, None, 0.0
        self.rate, self.waited, self.lurch = self.cadence, 0.0, None

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
        if self.z_was is not None:
            self.v_on += ((pel[2] - self.z_was) / dt - self.v_on) * min(1.0, dt / ON_S)
        self.z_was = pel[2]
        omega = math.sqrt(9.81 / max(0.3, bus['pelvis.pose.com_y']))
        xi = bus['pelvis.pose.com_x'] + v_side / omega
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
        if self.resume is not None:
            self._restart(balls, pel)
            stride, length = self.stride, gait.STRIDE_M * self.stride
        self._advance(dt, length, balls, pel, bus)
        lateral, height, yaw, legs, upper, roll = plan(self.phase, stride)
        spread = self.wide * (1.0 - gait.eased(self.age / WIDE_S))
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
        self._anchor(bus, qs, balls, height, pel)
        feet = [mul(ry(twist), rx(-pitch)) for _ankle, twist, pitch, _toes in legs]
        # A stance ankle where its foot, rolling on its heel or its ball, keeps that on the floor
        # where the anchor says.
        held = {side: sub(add(self.anchor[side], apply(ry(twist), sub(p, SOLE_BALL))),
                          apply(foot, p))
                for (side, _sign), (_an, twist, pitch, _to), foot in zip(SIDES, legs, feet)
                if side in self.anchor
                for p in ((SOLE_HEEL if pitch > 0.0 else SOLE_BALL),)}
        # The pelvis on: where the stance feet say, weighed.
        on, weight = 0.0, 0.0
        for (side, _sign), q, (ankle, _tw, _pi, _to) in zip(SIDES, qs, legs):
            b = carried(q)
            if b > 0.0 and side in held:
                on, weight = on + b * (held[side][2] - ankle[2]), weight + b
        lurch = max(-LURCH_M, min(LURCH_M, (on / weight if weight else pel[2]) - pel[2]))
        if self.lurch is not None:
            lurch = max(self.lurch - LURCH_M_S * dt, min(self.lurch + LURCH_M_S * dt, lurch))
        self.lurch = lurch
        planned_z = pel[2] + lurch
        swings, lower, feet_x, _off = self._landings(dt, bus, qs, legs, balls, pel, turn_now,
                                                     planned_z, spread, length, xi, omega)
        # The pelvis across: the walk's sway about the feet's line - each foot's, where it stands
        # or last stood, TRACK_M in, weighed as the plan has it carry: between them on both, over
        # the standing one as the other lifts. About the midline alone, the wide first steps left
        # the capture point 3 cm inside the standing foot at toe-off and it ran off; about the
        # swinging foot's landing, a wide landing drew the pelvis after it and itself wider
        # (2026-09-26). Its error and its speed's, against the plan's own speed, turned back past.
        line = weight = 0.0
        for (side, sign), q in zip(SIDES, qs):
            b = carried(q)
            line, weight = line + b * (feet_x[side] - sign * TRACK_M), weight + b
        planned_x = line / weight + lateral
        v_ref = (plan(self.phase + 1e-3, stride)[0] - lateral) * 1e3 * self.rate
        planned = (planned_x, height, planned_z)
        self.lowered += max(-RAISE_M_S * dt, min(LOWER_M_S * dt, min(LOWER_M, lower) - self.lowered))
        if self.side is not None and self.side['stage'] == 'out' and self.side['since'] == 0.0:
            # The foot put down takes the pelvis from where it is, as a landing does: from the
            # plan's height, 9 mm up, its leg hopped her off the floor (2026-09-26).
            self.lowered = max(self.lowered, min(LOWER_M, height - pel[1]))
        stands = [self.anchor[s][0] for s, _sign in SIDES if s in self.anchor] or [pel[0]]
        out_by = max(0.0, min(stands) - HOLD_M - xi, xi - max(stands) - HOLD_M)
        hold = max(0.0, 1.0 - out_by / SOLE_M)
        across = (hold * (planned_x - SIDE_K * (pel[0] - planned_x) - pel[0])
                  - SIDE_D * (v_side - v_ref))
        target = (pel[0] + max(-SOLE_M, min(SOLE_M, across)), height - self.lowered, planned_z)
        turn = mul(ry(yaw), rz(roll))
        off = [TURN_K * c for c in _vee(mul(turn, t(turn_now)))]
        if self.side is not None:
            off = [max(-SIDE_TURN_RAD, min(SIDE_TURN_RAD, c)) for c in off]
        turn = mul(_turned(tuple(off)), turn)
        target = self._reachable(target, turn, held, qs)
        self.planned, self.target = planned, target
        out = dict(zip(UPPER, upper))
        # The plumb line: the spine takes the pelvis's pitch back out, the torso upright; the neck
        # what the torso still leans, the head level.
        pitch = math.degrees(math.atan2(turn_now[2][1], turn_now[1][1]))
        self.pendulum.read(bus, dt)
        toward = [math.degrees(SWAY_K * o / SPINE_TO_EARS_M) for o in self.pendulum.off]
        out['spine'] = (-PLUMB * pitch - SURGE_DEG * min(1.0, self.scale)
                        * math.cos(4.0 * math.pi * (self.phase - SURGE_AT)) + toward[0])
        out['spine_roll'] = out['spine_roll'] - toward[1]
        out['neck'] = out['neck'] - (pitch + bus.get('spine.deg', 0.0))
        self._legs(out, bus, qs, legs, feet, held, swings, target, turn, turn_now, pel)
        if self.held is not None:
            self.age += dt
            self.scale = self.first + (1.0 - self.first) * gait.eased(self.age / RAMP_S)
            k = gait.eased(self.age / BLEND_S)
            out = {j: self.held.get(j, v) + (v - self.held.get(j, v)) * k for j, v in out.items()}
            if self.age >= RAMP_S:
                self.held = None
        self.last = out
        return out

    def _restart(self, balls, pel):
        """The walk begun again after a side step, as from standing (`begin`): on the foot that
        bears her, at the phase its place says, the other to step in beside it, the midline the
        walk's own from the standing foot, the stride her speed's. Resumed in one pass, the
        plan's height 3 cm above a body sunk over the leaning leg, both legs threw her 6 cm into
        the air, and the other foot was yanked into mid-swing; begun wide, the midline between
        the feet drew her off the standing one (2026-09-26)."""
        on, ball = self.resume, balls[self.resume]
        sign = 1.0 if on == 'left' else -1.0
        full = self.cadence * gait.STRIDE_M * gait.pace(self.cadence)
        scale = max(RESUME, min(1.0, self.v_on / full))
        length = gait.STRIDE_M * max(gait.PACE_STEP, gait.pace(self.cadence) * scale)
        q = min(gait.HEEL_OFF, gait.STANCE_AT + (gait.BALL - (ball[2] - pel[2])) / length)
        self.begin(self.last, phase=q if on == 'left' else (q + 0.5) % 1.0, scale=scale)
        self.stood['right' if on == 'left' else 'left'] = ball[0] - sign * 2.0 * TRACK_M

    def _anchor(self, bus, qs, balls, height, pel):
        """A foot is held where it landed until it leaves: landed once its sole bears, or ACCEPT
        into its stance whatever it bears. Held from the phase alone, a foot still in the air
        was taken to bear her, and she fell when the other left (2026-09-25)."""
        for (side, _sign), q in zip(SIDES, qs):
            if q >= gait.TOE_OFF or (self.side is not None and side == self.side['out']
                                     and self.side['stage'] == 'out'):
                self.anchor.pop(side, None)
            elif side not in self.anchor and (bus['pelvis.pose.%s_load' % side] > LANDED_N
                                              or q >= ACCEPT):
                self.anchor[side] = balls[side]
                # The height's target starts from where the body is, up again at RAISE_M_S:
                # landed with the body 3 cm low over the leaning leg, both legs pushed to the
                # plan's height and threw her 5 cm into the air (2026-09-26).
                self.lowered = max(self.lowered, min(LOWER_M, height - pel[1]))
            self.was_q[side] = q

    def _legs(self, out, bus, qs, legs, feet, held, swings, target, turn, turn_now, pel):
        """Each leg's joints into `out`, deg. A stance leg carries the pelvis to where it should
        be; a swing leg reaches from where the pelvis is, as it is turned. A leg still bearing is
        a stance leg whatever the phase says: reached from the pelvis as it stood while it bore,
        the rear heel's rise lifted her and the front foot hung off the floor (2026-09-26). In a
        side step the foot that bears carries the pelvis on and up but not across - the body a
        free pendulum on it, as the step out was foreseen - and holds the attitude; the other
        reaches from where the pelvis is; both feet flat. Carried across to the plan's target
        20 cm off, the leg chattered on the floor, then threw her up; its attitude let go, the
        torso fell over the hip in 60 ms; its height let go, she sank; the stepped-out foot
        pitched as the plan's swing had it struck the floor on its edge, 2200 N, 5 cm before its
        ankle was down (2026-09-26)."""
        for (side, sign), q, (_ankle, _tw, _pi, toes), foot in zip(SIDES, qs, legs, feet):
            b = carried(q)
            at = held[side] if side in held else swings[side]
            if self.side is not None and (side == self.side['out'] or q >= gait.TOE_OFF):
                foot, toes = FLAT, 0.0
            if q < gait.TOE_OFF + BEARS_UNTIL and bus['pelvis.pose.%s_load' % side] > LANDED_N:
                b = max(b, min(1.0, bus['pelvis.pose.%s_load' % side] / BEARS_N))
            if self.side is not None:
                bears = self.side['out'] if self.side['stage'] == 'down' else self.side['down']
                b = 1.0 if side == bears else 0.0
                hip_from = (pel[0], target[1], target[2]) if b else pel
            else:
                hip_from = tuple(b * a + (1.0 - b) * c for a, c in zip(target, pel))
            reach = turn if b >= 1.0 else mul(_turned(tuple(b * c for c in _vee(mul(turn, t(turn_now))))), turn_now)
            for k, v in zip(LEG, figure.leg(sign, hip_from, reach, at, foot)):
                out[side + k] = math.degrees(v)
            out[side + '_foot'] = toes

    def _landings(self, dt, bus, qs, legs, balls, pel, turn_now, planned_z, spread, length, xi,
                  omega):
        """({side: its ankle's target} for each foot not held, how far the pelvis must come down
        for them to reach, {side: x, where it stands or last stood}, what the capture point is
        off the walk's course a row): across on the capture point (`machine.capture`), round the
        standing foot; on, the plan's, shortened to reach; in a side step (`_sidestep`), its
        own."""
        standing = [(self.anchor[o] if o in self.anchor else balls[o])[0] for o in ('right', 'left')]
        sep = 2.0 * (TRACK_M + spread)
        x, swapping, catch, off = capture.landing(self.capture, (
            qs, (1.0, -1.0), (xi, xi), standing, (sep, sep), (self.rate, self.rate),
            (omega, omega)))
        loads = {side: bus['pelvis.pose.%s_load' % side] for side, _sign in SIDES}
        self._sidestep(dt, swapping, loads, balls, legs, pel, length)
        if self.side is not None:
            self.capture['swapping'][:] = 0.0
        self.hurry = SIDE_HURRY if self.side is not None else 0.0
        # No catch in the first strides: their landings are off the walk's own by design.
        self.catching = self.side is not None or (self.held is None and bool(catch.any()))
        out, lower, feet_x = {}, 0.0, {}
        for i, ((side, sign), q, (ankle, _tw, _pi, _to)) in enumerate(zip(SIDES, qs, legs)):
            step = self.side
            if step is not None and side == step['down'] and q >= gait.TOE_OFF:
                step['down_s'] = min(DOWN_S, step['down_s'] + dt)
                x0, y0, z0 = step['at']
                at = (x0, y0 + (gait.ANKLE_H - y0) * gait.eased(step['down_s'] / DOWN_S), z0)
                at, short = _reach(figure.hip(sign, pel, turn_now), at, SWING_REACH * gait.REACH)
                out[side], lower = at, max(lower, short)
                self.stood[side] = feet_x[side] = x0
                continue
            if step is not None and side == step['out'] and step['stage'] == 'out':
                # Foreseen once as it begins, with no ankle to help and no gain - on the capture
                # point, the body to come to rest over the foot. Followed as the capture point
                # ran, the foot chased the swing of its own leg's mass, 16 cm past (2026-09-26).
                if step['x_out'] is None:
                    xs, _swap, _catch, _off = capture.landing(capture.state(1, margin=0.0, gain=1.0), (
                        (1.0 - SIDE_S * self.rate,), (sign,), (xi,), (balls[step['down']][0],),
                        (sep,), (self.rate,), (omega,)))
                    step['x_out'] = float(xs[0])
                u = min(1.0, step['since'] / SIDE_S)
                # Eased on from where it stood: sent SIDE_AHEAD_M ahead at once while it bore
                # 1400 N, it dragged and threw her 1.5 cm up, the other foot off the floor
                # (2026-09-26).
                at = (step['x_from'] + (step['x_out'] - step['x_from']) * gait.eased(u),
                      gait.ANKLE_H + SIDE_LIFT_M * math.sin(math.pi * u) ** 2,
                      step['z_from'] + (pel[2] + SIDE_AHEAD_M - step['z_from']) * gait.eased(u))
                at, short = _reach(figure.hip(sign, pel, turn_now), at, SWING_REACH * gait.REACH)
                out[side], lower = at, max(lower, short)
                feet_x[side] = self.stood.get(side, balls[side][0])
                continue
            if side in self.anchor:
                self.stood[side] = feet_x[side] = self.anchor[side][0]
                continue
            feet_x[side] = self.stood.get(side, balls[side][0])
            u = (q - gait.TOE_OFF) / (1.0 - gait.TOE_OFF) if q >= gait.TOE_OFF else 0.0
            at = (float(x[i]) + sign * WIDEN_M * math.sin(math.pi * u) ** 2, ankle[1],
                  planned_z + ankle[2])
            at, short = _reach(figure.hip(sign, pel, turn_now), at, SWING_REACH * gait.REACH)
            out[side], lower = at, max(lower, short)
        return out, lower, feet_x, off

    def _sidestep(self, dt, swapping, loads, balls, legs, pel, length):
        """The side step's stages: begun on a swap the law asks while the other foot bears -
        `down`, the swinging foot put down flat where it is, no nearer across than the law's
        `cross`; `out` once it bears BEARS_N for DWELL_S, the other foot stepping; over when that
        one bears her, or the first still does SIDE_GIVE_S on - the walk begun again on
        whichever (`step`), and none begun while that blends in. Resumed at the phase left, the
        plan lifted the foot she had just caught herself on; asked as the blend began, a second
        side step put the foot down in the air (2026-09-26)."""
        if self.side is None and (self.held is None or self.age >= BLEND_S):
            for i, ((side, sign), (ankle, _tw, _pi, _to)) in enumerate(zip(SIDES, legs)):
                other = 'right' if side == 'left' else 'left'
                if swapping[i] and loads[other] > LANDED_N and side not in self.anchor:
                    ball, there = balls[side], balls[other][0]
                    across = max(sign * (ball[0] - there), float(self.capture['cross'][i]))
                    self.side = {'down': side, 'out': other, 'stage': 'down', 'since': 0.0,
                                 'at': (there + sign * across, ankle[1],
                                        max(ball[2] - gait.BALL, pel[2] + DOWN_AHEAD_M)),
                                 'x_out': None, 'borne': 0.0, 'down_s': 0.0}
                    break
        if self.side is None:
            return
        if self.side['stage'] == 'down':
            self.side['borne'] = (self.side['borne'] + dt
                                  if loads[self.side['down']] > BEARS_N else 0.0)
            if self.side['borne'] >= DWELL_S:
                self.side['stage'], self.side['since'], self.side['borne'] = 'out', 0.0, 0.0
                self.side['x_from'] = balls[self.side['out']][0]
                self.side['z_from'] = balls[self.side['out']][2] - gait.BALL
        else:
            self.side['since'] += dt
            out, down = self.side['out'], self.side['down']
            took = loads[out] > max(BEARS_N, loads[down])
            self.side['borne'] = self.side['borne'] + dt if took else 0.0
            # The step stays this pass, the walk begun again at the next (`step`): let go at
            # once, the plan's stale phase swung the foot she stood on 0.8 m out for a pass, and
            # the blend began from that (2026-09-26).
            if self.side['since'] >= 0.5 * SIDE_S and self.side['borne'] >= DWELL_S:
                self.resume = out
            elif self.side['since'] >= SIDE_GIVE_S and loads[down] > max(BEARS_N, loads[out]):
                self.resume = down

    def _advance(self, dt, length, balls, pel, bus):
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
        rate = max(LEAD * pace, min(pace / LEAD, pace + pull * (err / weight if weight else 0.0)))
        step = dt * rate
        # A standing foot lifts only once the other bears, LAND_WAIT_S at most: lifted on the
        # clock while the other still reached for the floor, she stood on neither (2026-09-26).
        for side, offset in (('left', 0.0), ('right', 0.5)):
            q = (self.phase + offset) % 1.0
            other = 'right' if side == 'left' else 'left'
            if (q < gait.TOE_OFF <= q + step + PIN and self.waited < LAND_WAIT_S
                    and bus['pelvis.pose.%s_load' % other] < LANDED_N):
                step, self.waited = gait.TOE_OFF - PIN - q, self.waited + dt
                break
        else:
            self.waited = 0.0
        self.rate = rate
        self.phase = (self.phase + step) % 1.0

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
