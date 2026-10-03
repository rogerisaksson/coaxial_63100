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

from machine import capture, figure, gait, landing, parry, stance, walkplan
from machine.pendulum import SPINE_TO_EARS_M, Pendulum
from machine.figure import LEG, add, mul, rx, ry, rz, t


#: The pelvis's sideways error and its speed fed back, 1 and s; its attitude turned back past
#: the plan, 1. With the gait's shape, the finest of 576 walks at 0.65, 0.8 and 0.9 strides/s,
#: judged by the stance knee's bend, the landing's peak, the pelvis's shake, the head's travel
#: and the copper's heat: bend 11.7 -> 5.7 degrees past 5, peak 2.9 -> 2.2 body weights, head
#: 8.5 -> 6.0 cm, heat 0.47 -> 0.29 and work 0.60 -> 0.35 m g d (2026-09-26). Driven by the
#: capture point's error from the plan's place and speed instead, 1.2 of it, the standing foot
#: slid 20 cm after a shove; by what it is off the walk's course alone, she fell in 1.4 s
#: (2026-09-26). SIDE_K and SIDE_D retuned (`physics.STAGED`).
SIDE_K, SIDE_D, TURN_K = 0.218864, 0.0809803, 1.42


#: The pelvis's forward target within LURCH_M of the pelvis, m, and moved from it no faster
#: than LURCH_M_S: a foot put down short of the plan's spot had the phase race to catch up and
#: the target run off at 2 m/s; landed 5 cm short after a shove, the target moved 5 cm back in
#: 30 ms, the landed knee snapped straight and threw her 5 cm up (2026-09-26).
LURCH_M, LURCH_M_S = 0.15, 0.3


#: In single support the pelvis goes CG_OVER of the way out over the standing foot: on the line
#: it hung 5-43 mm to the swinging side (2026-09-28).
CG_OVER = 0.0


#: In a side step the attitude is turned back past its error by no more than SIDE_TURN_RAD: the
#: legs are solved for the turned pelvis, and 8 degrees of it dragged the planted foot 12 cm
#: (2026-09-26).
SIDE_TURN_RAD = 0.035


#: Her speed on, filtered over ON_S, s (`landing.restart` begins the walk again at its stride).
ON_S = 0.05


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


#: How much of the pelvis's pitch the spine takes back out: riding the pelvis, the torso
#: pitched 8 degrees a step and the head bobbed 9 cm (2026-09-25).
PLUMB = 1.0


#: The joints that law sets, not eased in from what she was held at: the arrival holds her torso
#: by it too, and eased in, the spine froze at the hand-off while the pelvis tipped 3 degrees on
#: and the torso swung 9.4 -> 12.1 -> 8.3 (2026-09-28).
PLUMBED = ('spine', 'neck')


#: The torso counters her surge, twice a stride: the spine SURGE_DEG back at SURGE_AT of the left
#: leg's stride and each half stride on, as far forward between, less on a shorter stride - her
#: head carried on at an even speed, the pendulum between her ears still (`machine.pendulum`).
#: At 2 degrees, SWAY_K 1: the pendulum's stir over 30 s at 0.85 and 0.9 strides/s 4.3, 4.9 ->
#: 1.7, 1.9 mm, but rising and gliding to another pace she fell three times in four
#: (2026-09-26). 3.9 from a search over the scoreboard's twelve trials: the walks' stir 4.4 ->
#: 2.4 mm, the rises and the shoves held as before, 3.5-4.5 alike. Off: at 3.9 the torso
#: nodded 7.6 degrees a stride to hold the head's fore-aft to 24 mm where the pelvis surges 65
#: (the collision at every touchdown, -0.25 m/s), a head out of step with the body to the eye;
#: the pendulum, which cannot tell a nod from a smooth ride, 1.2 -> 3.5 mm (2026-09-27).
SURGE_DEG, SURGE_AT = 0.0, 0.125


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
#: (2026-09-26). With the torso's counter, 0.556 took the stir to 1.8 mm and 0.55 felled the walk
#: at 0.9 in 4 s: off (2026-09-27). At 0.1 the head 53 -> 48.5 mm fore and aft a stride, the
#: strike 1613 -> 1115 N, held 81.3 -> 87.9 %; 0.15 57 mm (2026-09-28).
SWAY_K = 0.1


#: Her shoulders kept over the walk's line while her hips sway: the spine rolls against the
#: pelvis's offset from the feet's midline, SHOULDERS_BACK of it taken back at the shoulders,
#: SHOULDERS_M up the torso. At 0.5 the shoulders 31.8 -> 26.7 mm across to the hips' 44.9, but
#: held 79.5 %, with the pendulum 73.2 (the walk at 0.65 fell); 0.25 with it caught 13 times
#: (2026-09-28). On the skimming swing 0.3 holds 100 %, the stir 2.11 -> 1.87 mm (2026-09-28).
#: With the turn at 4 and dropped 6, 0.25 held 78.5 % where 0.3 66.0; 0.4 fell (2026-09-30).
SHOULDERS_BACK, SHOULDERS_M = 0.25, 0.335


class Walker:

    """The setpoints for a DYNAMIC gynoid, every pass, from what the loop read."""

    def __init__(self, machine, cadence=gait.CADENCE):
        self.machine, self.cadence = machine, float(cadence)
        self.world = machine.nodes['pelvis'].world
        self.reset()

    def reset(self):
        """Every state of a walk as before the first, landed anew (`Director.begin`): kept from
        a fall, the pelvis lowered 12 cm and the speed read from 3.7 m back, the walk after a
        restart ran 8-10 cm crouched and a sole bore 16 kN (2026-09-28)."""
        #: The walk's line, radians from the world's z about the vertical (`view`).
        self.heading = 0.0
        self.phase = 0.0
        self.anchor, self.was_q = {}, {}
        self.x_was, self.v_side, self.z_was, self.v_on = None, 0.0, None, 0.0
        self.scale, self.age, self.held, self.wide, self.first = 1.0, 0.0, None, 0.0, 1.0
        #: The lean she began from, deg ahead of the plumb line, let out over gait.LEAN_OUT_S.
        self.lean = 0.0
        self.lift = None
        #: The last setpoints; each ball where it stands; where each foot last stood, x; the
        #: pelvis as planned and as targeted.
        self.last, self.balls, self.stood, self.planned, self.target = None, {}, {}, None, None
        #: A catch under way (`machine.director`); the phase run `hurry` faster; the landings'
        #: rows (`machine.capture`); the side step under way (`_sidestep`), None walking; the
        #: foot the walk begins again on after one.
        self.catching, self.hurry, self.capture = False, 0.0, capture.state()
        #: Shoved: the capture point past the standing feet's outer edge (`landing.shoved`); the
        #: parry's upper body, how far on and which way (`machine.parry`); how long neither sole
        #: has borne her, s (`stance.FLIGHT_S`).
        self.shoved, self.parry, self.flight, self.again = False, (0.0, 1.0), 0.0, False
        self.side, self.resume = None, None
        #: The phase's rate, strides/s; how long a standing foot has waited for the other to bear;
        #: how far the pelvis's target is lowered, m: for a swinging foot to reach, from a
        #: landing.
        self.rate, self.waited, self.lowered = self.cadence, 0.0, 0.0
        #: The forward target's offset from the pelvis last pass, m; None to begin with; the
        #: seconds a begun walk blends in over.
        self.lurch, self.blend_s = None, stance.BLEND_S
        #: Seconds since `halt`, None walking on; the stride's length last pass, m.
        self.halting, self.halt_from, self.length_was = None, 1.0, None
        #: The pendulum between her ears, read each pass (`machine.pendulum`).
        self.pendulum = Pendulum()
        #: {side: [seconds since its swinging foot met something, where in its swing, how far on
        #: it lands]}; where a heel clears what was met, along the walk, or None (`landing.over`).
        self.trip, self.over = {}, None
        #: {side: the floor's height under that foot where it last bore}; the step up from the
        #: other's at the last landing; the feet borne since they landed (`stance.anchor`).
        self.floor, self.rise, self.borne = {}, 0.0, set()
        #: The floor under her, m, as the pelvis follows it (`stance.UNDER_M_S`).
        self.under = 0.0

    def halt(self):
        """To a stop over HALT_S: the stride down to HALT of its own; `halted` from then."""
        self.halting, self.halt_from, self.held = 0.0, self.scale, None

    @property
    def halted(self):
        return self.halting is not None and self.halting >= stance.HALT_S

    @property
    def stride(self):
        return max(gait.PACE_STEP, gait.pace(self.cadence) * self.scale)

    def start(self):
        """The body placed mid-stride at phase 0: the plan's pose, moving at its speed."""
        stride, dt = self.stride, 1e-3
        poses = []
        for p in (0.0, dt * self.cadence):
            lateral, height, yaw, legs, upper, roll = walkplan.plan(p, stride)
            pelvis = (lateral, height, p * gait.STRIDE_M * stride)
            angles = dict(zip(walkplan.UPPER, upper))
            for (side, sign), (ankle, twist, pitch, toes) in zip(walkplan.SIDES, legs):
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
        self.reset()
        return angles

    def begin(self, held, wide=0.0, scale=None, phase=None, blend_s=stance.BLEND_S, ball_ahead=None,
              on='left', lean=0.0, again=False):
        """Walking from where she stands on her left foot, the right lifted: {joint: deg}
        `held` the stand's setpoints, eased out of over `blend_s`; the first steps `wide` m
        further out than the walk's, narrowing over WIDE_S; at `phase`, or where the plan has
        the pelvis `ball_ahead` m behind the ball of the foot `on`, HEEL_OFF at most; the pelvis
        and the torso `lean` deg ahead of the plumb line, let out over gait.LEAN_OUT_S; `again`
        after a catch, which may catch again (`landing.SIDE_AGAIN_S`)."""
        self.blend_s, self.lean = blend_s, float(lean)
        self.anchor, self.was_q = {}, {}
        self.stood, self.x_was, self.v_side = {}, None, 0.0
        self.age, self.held, self.wide = 0.0, dict(held), float(wide)
        self.first = stance.BEGIN if scale is None else float(scale)
        self.scale = self.first
        if ball_ahead is not None:
            # No sooner than its heel strike: a catch landed further ahead came to -0.21, the
            # phase wrapped into its swing and lifted the foot she was to stand on (2026-10-01).
            phase = max(0.0, min(gait.HEEL_OFF, gait.STANCE_AT
                                 + (gait.BALL - ball_ahead) / (gait.STRIDE_M * self.stride)))
            phase = phase if on == 'left' else (phase + 0.5) % 1.0
        self.phase = stance.BEGIN_AT if phase is None else phase
        self.lift, self.halting, self.length_was = None, None, None
        self.capture, self.side, self.resume, self.hurry = capture.state(), None, None, 0.0
        # Read a pass stale, a catch had the other foot, bearing where the new phase swung it,
        # begin the walk again on itself (2026-10-01).
        self.catching, self.again = False, again
        self.rate, self.waited, self.lurch = self.cadence, 0.0, None

    def face(self, heading):
        """The walk's line turned to `heading`, radians, the pendulum read anew along it: read
        on across the turn, the ears' track jumped by it and the first steps fell (2026-09-30)."""
        self.heading, self.pendulum = float(heading), Pendulum()

    def view(self, bus):
        """The bus as the walk sees it: the pelvis's place, turn and centre of mass turned about
        the vertical so the walk's line (`heading`) lies along z."""
        h = self.heading
        if h == 0.0:
            return bus
        c, s = math.cos(h), math.sin(h)
        out = dict(bus)
        for x, z in (('pelvis.pose.x', 'pelvis.pose.z'), ('pelvis.pose.com_x', 'pelvis.pose.com_z')):
            out[x], out[z] = bus[x] * c - bus[z] * s, bus[x] * s + bus[z] * c
        w, x, y, z = (bus['pelvis.pose.q' + k] for k in 'wxyz')
        hc, hs = math.cos(h / 2.0), -math.sin(h / 2.0)
        out['pelvis.pose.qw'], out['pelvis.pose.qx'] = hc * w - hs * y, hc * x + hs * z
        out['pelvis.pose.qy'], out['pelvis.pose.qz'] = hc * y + hs * w, hc * z - hs * x
        return out

    def ball_ahead(self, side):
        """How far the ball of this foot stands ahead of the pelvis, m, as the loop read it."""
        bus = self.view(self.machine.loop.bus)
        pel = (bus['pelvis.pose.x'], bus['pelvis.pose.y'], bus['pelvis.pose.z'])
        turn = figure.quat(bus['pelvis.pose.qw'], bus['pelvis.pose.qx'], bus['pelvis.pose.qy'],
                           bus['pelvis.pose.qz'])
        angles = tuple(math.radians(bus.get(side + k + '.deg', 0.0)) for k in LEG)
        return figure.ball(1.0 if side == 'left' else -1.0, pel, turn, angles)[2] - pel[2]

    def step(self, dt):
        """{joint: degrees}: where every drive should be now."""
        walkplan.ease(dt)
        bus = self.view(self.machine.loop.bus)
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
            self.scale = self.halt_from + (stance.HALT - self.halt_from) * gait.eased(self.halting / stance.HALT_S)
        stride = self.stride
        length = gait.STRIDE_M * stride
        balls, ankles, soles = {}, {}, {}
        for side, sign in walkplan.SIDES:
            angles = tuple(math.radians(bus.get(side + k + '.deg', 0.0)) for k in LEG)
            at = figure.ball(sign, pel, turn_now, angles)
            balls[side], soles[side] = (at[0], 0.0, at[2]), at[1]
            ankles[side] = figure.foot_of(sign, pel, turn_now, angles)[0]
        self.balls = balls
        if self.resume is not None:
            landing.restart(self, balls, pel)
            stride, length = self.stride, gait.STRIDE_M * self.stride
        stance.advance(self, dt, length, balls, pel, bus)
        lateral, height, yaw, legs, upper, roll = walkplan.plan(self.phase, stride)
        if landing.SWING_LEAD_S:
            led = walkplan.plan(self.phase + landing.SWING_LEAD_S * self.rate, stride)[3]
            legs = tuple(b if landing.led(q) else a
                         for q, a, b in zip((self.phase, self.phase + 0.5), legs, led))
        spread = self.wide * (1.0 - gait.eased(self.age / WIDE_S))
        if self.halting is not None:
            spread += STAND_WIDE_M * gait.eased(self.halting / stance.HALT_S)
        legs = tuple(((a[0] + sign * spread, a[1], a[2]),) + tuple(rest)
                     for (_side, sign), (a, *rest) in zip(walkplan.SIDES, legs))
        height, yaw, roll = float(height), float(yaw), float(roll)
        if self.held is not None:
            # From standing, the pelvis's height eases from where it stood to the walk's: the
            # walk's short first strides ride 7 cm higher, and taken at once she hopped.
            if self.lift is None:
                self.lift = pel[1] - height
            height += self.lift * (1.0 - gait.eased(self.age / stance.RAMP_S))
        qs = (self.phase, (self.phase + 0.5) % 1.0)
        stance.anchor(self, dt, bus, qs, balls, soles, height, pel)
        under = stance.under(self, dt, qs)
        feet, held = stance.held(self, qs, legs)
        # The pelvis on: where the stance feet say, weighed.
        on, weight = 0.0, 0.0
        for (side, _sign), q, (ankle, _tw, _pi, _to) in zip(walkplan.SIDES, qs, legs):
            b = walkplan.carried(q)
            if b > 0.0 and side in held:
                on, weight = on + b * (held[side][2] - ankle[2]), weight + b
        lurch = max(-LURCH_M, min(LURCH_M, (on / weight if weight else pel[2]) - pel[2]))
        if self.lurch is not None:
            lurch = max(self.lurch - LURCH_M_S * dt, min(self.lurch + LURCH_M_S * dt, lurch))
        self.lurch = lurch
        planned_z = pel[2] + lurch
        swings, lower, feet_x, _off = landing.landings(self, dt, bus, qs, legs, balls, pel, turn_now,
                                                     planned_z, spread, length, xi, omega, ankles)
        # The pelvis across: the walk's sway about the feet's line - each foot's, where it stands
        # or last stood, TRACK_M in, weighed as the plan has it carry: between them on both, over
        # the standing one as the other lifts. About the midline alone, the wide first steps left
        # the capture point 3 cm inside the standing foot at toe-off and it ran off; about the
        # swinging foot's landing, a wide landing drew the pelvis after it and itself wider
        # (2026-09-26). Its error and its speed's, against the plan's own speed, turned back past.
        line = weight = 0.0
        for (side, sign), q in zip(walkplan.SIDES, qs):
            b = walkplan.carried(q)
            line, weight = line + b * (feet_x[side] - sign * walkplan.TRACK_M * (1.0 - CG_OVER)), weight + b
        planned_x = line / weight + lateral
        v_ref = (walkplan.plan(self.phase + 1e-3, stride)[0] - lateral) * 1e3 * self.rate
        planned = (planned_x, height + under, planned_z)
        self.lowered += max(-(stance.RAISE_M_S + self.lowered / stance.RAISE_S) * dt,
                            min(stance.LOWER_M_S * dt, min(stance.LOWER_M, lower) - self.lowered))
        if self.side is not None and self.side['stage'] == 'out' and self.side['since'] == 0.0:
            # The foot put down takes the pelvis from where it is, as a landing does: from the
            # plan's height, 9 mm up, its leg hopped her off the floor; from a height still
            # lowered for a catch, 3 cm down, its leg lifted it and she dropped onto it, 2100 N
            # (2026-09-26).
            self.lowered = min(stance.LOWER_M, height - pel[1])
        stands = [self.anchor[s][0] for s, _sign in walkplan.SIDES if s in self.anchor] or [pel[0]]
        out_by = max(0.0, min(stands) - HOLD_M - xi, xi - max(stands) - HOLD_M)
        hold = max(0.0, 1.0 - out_by / SOLE_M)
        across = (hold * (planned_x - SIDE_K * (pel[0] - planned_x) - pel[0])
                  - SIDE_D * (v_side - v_ref))
        target = (pel[0] + max(-SOLE_M, min(SOLE_M, across)), height + under - self.lowered,
                  planned_z)
        lean = self.lean * (1.0 - gait.eased(self.age / gait.LEAN_OUT_S))
        turn = mul(mul(ry(yaw), rx(math.radians(lean))), rz(roll))
        off = [TURN_K * c for c in walkplan.vee(mul(turn, t(turn_now)))]
        if self.side is not None:
            off = [max(-SIDE_TURN_RAD, min(SIDE_TURN_RAD, c)) for c in off]
        turn = mul(walkplan.turned(tuple(off)), turn)
        target = stance.reachable(self, target, turn, held, qs)
        self.planned, self.target = planned, target
        out = dict(zip(walkplan.UPPER, upper))
        # The plumb line: the spine takes the pelvis's pitch back out, the torso upright; the neck
        # what the torso still leans, the head level.
        pitch = math.degrees(math.atan2(turn_now[2][1], turn_now[1][1]))
        self.pendulum.read(bus, dt)
        toward = [math.degrees(SWAY_K * o / SPINE_TO_EARS_M) for o in self.pendulum.off]
        out['spine'] = (-PLUMB * pitch + lean - SURGE_DEG * min(1.0, self.scale)
                        * math.cos(4.0 * math.pi * (self.phase - SURGE_AT)) + toward[0])
        out['spine_roll'] = (out['spine_roll'] - toward[1] + math.degrees(math.atan2(
            SHOULDERS_BACK * (pel[0] - line / weight), SHOULDERS_M)))
        out['neck'] = out['neck'] - (pitch + bus.get('spine.deg', 0.0))
        stance.legs(self, out, bus, qs, legs, feet, held, swings, target, turn, turn_now, pel)
        self.parry = parry.ease(self.parry, self.catching or self.shoved,
                                1.0 if v_side >= 0.0 else -1.0, dt)
        out = parry.upper(out, *self.parry)
        if self.held is not None:
            self.age += dt
            self.scale = self.first + (1.0 - self.first) * gait.eased(self.age / stance.RAMP_S)
            k = gait.eased(self.age / self.blend_s)
            out = {j: v if j in PLUMBED else self.held.get(j, v) + (v - self.held.get(j, v)) * k
                   for j, v in out.items()}
            if self.age >= stance.RAMP_S:
                self.held = None
        self.last = out
        return out
