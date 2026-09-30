"""A walk worked out from the feet: each on the floor through its stance, the twenty joints by IK.

    angles = walk(t, cadence=0.85)              # {joint: degrees} for machine.loop.write
    lateral, roll, height = sway(t)             # the pelvis's place: drawn, not a joint
    angles = blend(stand(), walk(t), 0.4)       # part of the way from one pose to another

A leg's phase 0 is its foot landing - on the ball, the toes pointed down to take the step. The
heel settles but never lands, rises again to push off, and the foot swings to the next landing
on an arc, pointed; the hip and knee follow by two-link IK from where the ankle is, the ankle
and toes from how the foot is pitched. The pelvis glides at one speed, as high as the stance
legs reach with a soft knee, smoothed; it alone sways: shifting over the stance leg, dropping
on the swing side, turning with the swinging leg. The torso turns the turn back and the head
holds its heading. Faster, the steps lengthen and the knees bend more (`pace`). Every piece is
eased (smootherstep): position, speed and acceleration are continuous. Signs are the type's
(`machine.routines`): a hip negative forward, a knee positive flexed, an ankle or a toe
negative toes-up, a shoulder positive forward.
"""
import math

from machine.curves import eased, ends, hermite, pivot, septic, through

#: The humanoid's joints, as its body names them (`machine.routines.TYPES['humanoid']`).
JOINTS = ('pelvis', 'waist', 'neck', 'head',
          'left_shoulder', 'left_elbow', 'left_wrist', 'left_gripper',
          'left_hip', 'left_knee', 'left_ankle', 'left_foot',
          'right_shoulder', 'right_elbow', 'right_wrist', 'right_gripper',
          'right_hip', 'right_knee', 'right_ankle', 'right_foot')

#: The legs, metres (`coaxial.graphics.gynoid` is built on them): the hips apart (half) and
#: below the pelvis's centre, thigh and shank, the ankle over the sole, the ball ahead of it
#: (the toes' joint) and the heel behind; the tracks the feet walk on, either side of the line.
HIP_HALF, HIP_DROP, THIGH, SHANK = 0.082, 0.055, 0.39, 0.38
#: A 23 cm sole with the toes (`figure.CONTACTS`), a sneaker in size 36-37: 27 cm read as boats,
#: 24 still big (2026-09-28).
ANKLE_H, BALL, HEEL = 0.075, 0.117, 0.055
#: The toes' joint over the sole, and the toes' length from it.
TOE_RY, TOE_M = 0.014, 0.058

#: The feet walk a beam: each planted TRACK_M off the line, swung WIDEN_M further out round the
#: standing foot at mid-swing, the stand's STAND_M apart (half).
TRACK_M, WIDEN_M, STAND_M = 0.03, 0.01, HIP_HALF

#: The walk: metres a stride (two steps) at `stride` 1 - 1.15 was more than her 0.77 m legs
#: reach behind at toe-off, and the hips sank to let them. At 1.0 (the toe-off at 0.66) the
#: pelvis dipped 49 mm under the stand and the head went 56 mm fore and aft; at 0.85, 34 and 35,
#: the scoreboard's cost 7.74 -> 7.18, held 83.7 %, the stir 3.4 -> 2.3 mm (2026-09-28).
STRIDE_M = 0.85

#: The cadence a stride of 1 goes with, strides a second; faster, the steps lengthen as the
#: cadence to PACE_POWER (`pace`); the hips are fitted every PACE_STEP of stride and weighed
#: between.
CADENCE, PACE_POWER, PACE_STEP = 0.85, 0.6, 0.05

#: The stance knee's bend at its straightest, degrees, at a stride of 1: soft, the hip and the
#: ball of the foot carry the glide; a longer stride bends it as the stride to KNEE_POWER. At 4.7
#: the leg landed bent 23 degrees and straightened to 7 within 0.16 of a stride, the pelvis up
#: 26 mm at 0.29 m/s and stopped dead at the top, 6 m/s2 either way at her ears: the pendulum's
#: stir 2.4 mm; at 8, 12, 16, 20, 25: 2.2, 2.0, 2.0, 1.8, 1.8 mm, at 30 she fell. Back to 8
#: with the heel strike at 15 degrees and the height's recovery at 0.03 m/s (`stance.RAISE_M_S`):
#: the knee 9-29 degrees through stance for 16-29, the stir 1.4 -> 1.2 mm, the shoves as before;
#: at 10 alone the walk begun from the lean fell 2.7 s in. With the heel rising to 50 degrees at
#: toe-off, 6: the walks and the floor's events as at 8; at 4 the walk at 0.9 strides/s fell in
#: 0.6 s, its front foot short of its landing (2026-09-27).
KNEE_SOFT_DEG, KNEE_POWER = 6.0, 1.5

#: Never straighter than KNEE_MIN_DEG, whatever the stride: at a short first stride the soft
#: knee came to 1 degree, the IK out of reach, and the front foot hung over the floor (2026-09-26).
KNEE_MIN_DEG = 4.0

#: The foot lands on its heel, toes up LAND_DEG (`pitch_of`), and rolls about it flat by SETTLE;
#: its heel rises from HEEL_OFF about the ball and leaves the floor at toe-off, TOE_OFF. Landed on
#: the ball, heel up, the landing knee stood at 42 degrees and the step struck 3.3 body weights
#: (2026-09-26). Toes up 15 for 9.3, the heel strike's peak 1.55 -> 1.1 kN, 20 alike; the
#: front leg's reach at the strike grows with it but the plan's height is the trailing leg's,
#: flat and 22 cm behind, so the landing knee bends the more (2026-09-27). Off at 0.62 the thigh
#: still stood 1 degree ahead of upright as the toes left the floor, 196 mm behind the hip -
#: her feet in front (`STANCE_AT`); at 0.66, 8.5 behind, 235 mm; 0.68, 11.7 and the head's bob
#: 31 mm; 0.7, 22.9 and her head 140 mm fore and aft (2026-09-28).
SETTLE, HEEL_OFF, TOE_OFF, LAND_DEG, LAND_RATE = 0.13, 0.36, 0.66, 15.0, -75.0

#: The foot's pitch at toe-off, degrees toes-up, and its rate and acceleration there, a stride
#: and a stride squared: the heel rises fastest at toe-off and on into the air. Eased to a stop
#: there, the whole foot stood still, the knee straightened -180 deg/s and then bent +409
#: (2026-09-25).
TOE_DEG, TOE_RATE, TOE_ACC = -50.0, -300.0, 4000.0

#: The heel up RISE_DEG toes-down as the other foot lands (RISE_AT), turning RISE_RATE a stride:
#: up 4 there, the trailing leg held the hips at 822 mm where the landing leg needed 841 to land
#: straight, its knee at 31 degrees; at 10, 13, the hips' rise and fall 15 -> 7 mm, held 100 %;
#: at 15 she fell at 0.9 strides/s (2026-09-28).
RISE_AT, RISE_DEG, RISE_RATE = 0.5, -10.0, -180.0

#: The foot levels in the swing: SWING_DEG toes up at SWING_AT of the way from toe-off to the
#: landing, turning SWING_RATE a stride, hanging toes-down from the ankle until it passes under
#: the hip. Level at 0.4 over the skim (LIFT_M), the foot trailed flat and the knee bent twice,
#: 45, 31, 46 degrees: she pedalled; at 0.6 once, to 55 (2026-09-28).
SWING_AT, SWING_DEG, SWING_RATE = 0.6, 0.0, 60.0

#: The body's lean ahead of the plumb line before the first step, deg - the pelvis tipped in
#: the lean's frame, the torso with it - and how long the walk takes to let it out, s: pushed
#: on with the torso plumb she read as leaning back before she stepped; a lean kept through
#: the walk, 2 to 4 deg, had her fall to the slip and the hot knee, held 76 % against 90
#: (2026-09-27). At 4 the walker's plan stood the pelvis up as it took her, 7 degrees back in
#: 0.2 s, the stance heel rising: on her toes and leaning back; the plan tips it as the lean
#: now. At 8 from the weight's shift on she leant on unnaturally (2026-09-28).
LEAN_DEG, LEAN_OUT_S = 4.0, 2.0


def _knots(stride):
    """The foot's pitch toes-up at its knots: (phase, degrees, a stride, a stride squared), a
    quintic between; the landing's the toes already coming down. The heel rises as the stride:
    to the full 50 degrees on the walk's first short strides, the push-off hopped her off the
    front foot and she zigzagged over (2026-09-27)."""
    swing = (((TOE_OFF + SWING_AT * (1.0 - TOE_OFF), SWING_DEG, SWING_RATE, 0.0),)
             if 0.0 < SWING_AT < 1.0 else ())
    rise = (((RISE_AT, RISE_DEG * stride, RISE_RATE * stride, 0.0),)
            if HEEL_OFF < RISE_AT < TOE_OFF else ())
    return ((SETTLE, 0.0, 0.0, 0.0), (HEEL_OFF, 0.0, 0.0, 0.0)) + rise + (
            (TOE_OFF, TOE_DEG * stride, TOE_RATE * stride, TOE_ACC * stride),) + swing + (
            (1.0, LAND_DEG, LAND_RATE, 0.0), (1.0 + SETTLE, 0.0, 0.0, 0.0))

#: The middle of a leg's single support: from the other's toe-off to its own landing.
MID_STANCE = 0.5 * TOE_OFF

#: Where the ball is planted: the ankle over it at STANCE_AT, early, so the leg's reach is
#: behind her; centred, the thigh never passed upright and her feet were always in front. At
#: 0.24 with the toe-off at 0.66 the thigh leaves 12.2 degrees behind upright, the toes 297 mm
#: behind the hip, the strike 1550 -> 1108 N, the head's bob 11 -> 28 mm and its surge 45 -> 54;
#: held 83.8 -> 85.6 %, the stir 2.6 -> 3.6 mm; at 0.2 alone the surge doubled, 0.14 fell
#: (2026-09-28).
STANCE_AT = 0.24

#: The hips ride as high as a stance leg reaches (REACH), smoothed: held level, the knees stood
#: at 33-40 degrees, a crouch. Swept (2026-09-25): stance knee at most 16 degrees, the thigh 20
#: behind upright at toe-off, the hips rising and falling 20 mm.

#: The pelvis alone moves: over the stance leg (metres; at 3 cm both legs leaned together, a
#: parallelogram), dropping on the swing side (degrees,
#: its obliquity) and turning about the spine (degrees, its rotation); the torso turns all of it
#: back (COUNTER 1), so the shoulders and the head go straight. Turned 12, the pelvis swung 39
#: degrees, the hips wagging past the catwalk's sway; at 6, 24, and from in front and behind her
#: hips no longer swayed under her shoulders (60 and 41 mm across): at 9 and dropped 6, 34 and
#: 12.5, the hips 72 mm across under shoulders at 25 (`walker.SHOULDERS_BACK`) (2026-09-28).
#: The bench saw her bottom swing, not her hips sway: at 3 and dropped 7 the pelvis turns 16.7
#: and rolls 14.5, the hips 72 mm across under shoulders at 23, held 74 % of the Monte Carlo
#: where 77; dropped 8, from mid-stride at 0.918 she fell in her first step. A woman's catwalk
#: take turns 27 and rolls 27 (`tools/sim/mocap.py`). Turned 4, dropped 6, the shoulders back
#: 0.25: swing 19.2 where 16.0, hips wag 65.3 mm where 71.4, held 78.5 % where 74.0 (2026-09-30).
SHIFT_M, ROLL_DEG, TURN_DEG, COUNTER = 0.010, 6.0, 4.0, 1.0

#: How far each arm joint trails the one above it, radians of the stride.
TRAIL = 0.55

#: An arm's swing, a joint a row from the shoulder down: (joint, mean deg, swing deg), the
#: shoulder's swing a stride's; each joint TRAIL behind the one above. Hers: the upper arm near
#: the body, the forearm carrying the swing, the hand soft.
ARM = (('_shoulder', 0.0, 12.0), ('_elbow', 24.0, 11.0), ('_wrist', 10.0, 7.0),
       ('_gripper', 22.0, 5.0))

#: The longest a leg reaches, hip to ankle: its knee at 2 degrees - the hips' limit and the IK
#: both. Past it the reach eases to it over SOFT_REACH_M: clamped, the knee stopped dead and
#: started again, and it showed. Near straight a knee is all angle: 1 mm short is 6 degrees,
#: 15 mm is 30.
REACH = math.sqrt(THIGH ** 2 + SHANK ** 2 + 2.0 * THIGH * SHANK * math.cos(math.radians(2.0)))
SOFT_REACH_M = 0.002


def _within(d):
    """`d` metres, eased under REACH: d well short of it, REACH well past it, smooth between."""
    over = (REACH - d) / SOFT_REACH_M
    return REACH - SOFT_REACH_M * (math.log1p(math.exp(over)) if over < 30.0 else over)


def pitch_of(q, stride=1.0):
    """The foot's pitch toes-up, degrees, at this leg's phase `q` and `stride`, through `_knots`:
    flat from SETTLE, the heel rising through toe-off, round through the swing and the landing
    down to flat again - eased to a stop at the landing, the foot moved in steps."""
    q = q % 1.0 + (1.0 if q % 1.0 < SETTLE else 0.0)
    knots = _knots(stride)
    for (q0, *start), (q1, *end) in zip(knots, knots[1:]):
        if q < q1:
            span = q1 - q0
            scale = (1.0, span, span * span)
            return hermite([v * s for v, s in zip(start, scale)],
                            [v * s for v, s in zip(end, scale)], (q - q0) / span)
    return 0.0


def planted(q, stride=1.0):
    """(ankle ahead of the hip, ankle over the floor, the foot's pitch toes-up) of a planted foot
    at its leg's phase `q`, metres and degrees: toes up, pivoting on its heel, heel up on its
    ball, the hip moving on at a stride a cycle. The stance's; the swing is `swung`."""
    length = STRIDE_M * stride
    q %= 1.0
    pitch = pitch_of(q, stride)
    if pitch > 0.0:
        x, y = pivot(length * STANCE_AT - HEEL, 0.0, HEEL, ANKLE_H, pitch)
    else:
        x, y = pivot(length * STANCE_AT + BALL, 0.0, -BALL, ANKLE_H, pitch)
    return x - length * q, y, pitch


def _roll(p):
    return ROLL_DEG * math.cos(2.0 * math.pi * (p - MID_STANCE))


def _yaw(p, stride):
    """The pelvis's turn, degrees: furthest at each strike, the striking leg's hip forward (a
    positive turn takes her left hip back)."""
    return -TURN_DEG * stride * math.cos(2.0 * math.pi * p)


def _limit(p, stride):
    """How high the hips' mid-point can ride at `p`: every foot on the floor within REACH; a foot
    in the air holds nothing up."""
    reach = REACH
    roll, yaw = math.radians(_roll(p) * stride), math.radians(_yaw(p, stride))
    low = math.inf
    for sign, q in ((1.0, p), (-1.0, (p + 0.5) % 1.0)):
        if q % 1.0 >= TOE_OFF:
            continue
        x, y, _pitch = planted(q, stride)
        dx = x + sign * HIP_HALF * math.sin(yaw)
        low = min(low, y + math.sqrt(max(0.0, reach * reach - dx * dx))
                  - sign * HIP_HALF * math.sin(roll))
    return low


#: The hips' height: the limit at SAMPLES phases, eroded - a soft minimum over ERODE of a stride,
#: SOFT_MIN_M soft - to round each double support's dip, blurred over BLUR of a stride, and run
#: through by a Catmull-Rom curve. Fitted by three harmonics it overshot the dips and came down
#: 3 cm to clear them, the knees bent 30 degrees; eroded alone, the rise as the other foot left
#: the floor was a step, a jerk 156 times the median. Eroded over 0.027 the plan dipped 17 mm
#: where the body dipped 27, and the stance leg lifted her back to it in a rush: over 0.05 the
#: stir 2.0 -> 1.7 mm, over 0.08 1.5, the held share sliding (2026-09-27).
SAMPLES, ERODE, SOFT_MIN_M, BLUR = 240, 0.05, 0.002, 0.030


def _eroded(limits):
    """Each sample's soft minimum over its neighbours, weighted over ERODE of a stride."""
    n = len(limits)
    out = []
    for i in range(n):
        low = min(limits)
        total = weight = 0.0
        for j, v in enumerate(limits):
            d = ((j - i + n // 2) % n - n // 2) / float(n)
            w = math.exp(-0.5 * (d / ERODE) ** 2)
            total += w * math.exp(-(v - low) / SOFT_MIN_M)
            weight += w
        out.append(low - SOFT_MIN_M * math.log(total / weight))
    return out


def _blurred(values):
    """Each sample the Gaussian-weighted mean of its neighbours over BLUR of a stride."""
    n = len(values)
    weights = [math.exp(-0.5 * ((((d + n // 2) % n - n // 2) / float(n)) / BLUR) ** 2)
               for d in range(n)]
    total = sum(weights)
    return [sum(weights[(j - i) % n] * v for j, v in enumerate(values)) / total
            for i in range(n)]


def _fit(stride):
    """The hips' height at SAMPLES phases: the eroded limit, lowered until the curve through it
    is under the limit everywhere, and then as far again as a soft knee takes a leg's length."""
    limits = [_limit(k / SAMPLES, stride) for k in range(SAMPLES)]
    smooth = _blurred(_eroded(limits))
    over = max(through(smooth, k / (4.0 * SAMPLES)) - _limit(k / (4.0 * SAMPLES), stride)
               for k in range(4 * SAMPLES))
    knee = math.radians(max(KNEE_MIN_DEG, KNEE_SOFT_DEG * stride ** KNEE_POWER))
    soft = REACH - math.sqrt(THIGH ** 2 + SHANK ** 2 + 2.0 * THIGH * SHANK * math.cos(knee))
    return [v - max(0.0, over) - soft for v in smooth]


def pace(cadence):
    """The stride a cadence walks at: 1 at CADENCE, longer faster."""
    return max(PACE_STEP, (cadence / CADENCE) ** PACE_POWER)


class _Fits(dict):
    def __missing__(self, stride):
        self[stride] = _fit(stride)
        return self[stride]


#: The hips' height by stride, fitted on first ask, every PACE_STEP.
_FITS = _Fits()


def _hips(p, stride):
    """The hips' mid-point over the floor at the left leg's phase `p`: smooth, under what the
    stance legs reach; between two fitted strides, both weighed, so a pace changes smoothly."""
    low = math.floor(stride / PACE_STEP) * PACE_STEP
    w = (stride - low) / PACE_STEP
    below = through(_FITS[round(low, 6)], p)
    return below if w < 1e-9 else (1.0 - w) * below + w * through(
        _FITS[round(low + PACE_STEP, 6)], p)


def _hip_of(sign, stride):
    """where(q) for a leg - the left `sign` 1, half a stride ahead of the right: its hip's height
    over the floor, dropping and rising with the pelvis, and how far the pelvis's turn carries it
    forward of the centre."""
    def where(q):
        p = (q if sign > 0.0 else q + 0.5) % 1.0
        roll, yaw = math.radians(_roll(p) * stride), math.radians(_yaw(p, stride))
        return (_hips(p, stride) + sign * HIP_HALF * math.sin(roll),
                -sign * HIP_HALF * math.sin(yaw))
    return where


def _ik(x, y, hip, ahead):
    """(thigh, knee) degrees - the thigh's lean forward of plumb, the knee's bend - for the ankle
    at (x ahead, y over the floor) from a hip `hip` over the floor, `ahead` forward."""
    dx, dy = x - ahead, y - hip
    reach = _within(math.hypot(dx, dy))
    bend = math.acos(max(-1.0, min(1.0, (THIGH ** 2 + reach ** 2 - SHANK ** 2)
                                   / (2.0 * THIGH * reach))))
    knee = math.pi - math.acos(max(-1.0, min(1.0, (THIGH ** 2 + SHANK ** 2 - reach ** 2)
                                             / (2.0 * THIGH * SHANK))))
    return math.degrees(math.atan2(dx, -dy) + bend), math.degrees(knee)


#: The swinging foot skims the floor: its heel's, ball's and toes' least height over it
#: (`_least`, soft within LIFT_SOFT_M) LIFT_M, taken from the planted foot's pivot over
#: LIFT_RISE of the swing and handed to the landing's over LIFT_FALL. Lifted by a bump over a
#: path, the foot went 135 mm up and the knee to 75 degrees: she trod the air (2026-09-28).
LIFT_M, LIFT_RISE, LIFT_FALL, LIFT_SOFT_M = 0.025, 0.25, 0.4, 0.003

#: The step the swing's end conditions are differenced over, of a stride.
DIFF = 1e-4


def _least(q, stride):
    """The foot's least height under its ankle at the leg's phase `q`, metres: the heel's, the
    ball's and the toes' tip's, soft within LIFT_SOFT_M, pitched and bent as planned."""
    pitch = pitch_of(q, stride)
    a, b = math.radians(pitch), math.radians(pitch - toes_of(q, pitch))
    ball = BALL * math.sin(a) - ANKLE_H * math.cos(a)
    heights = (-HEEL * math.sin(a) - ANKLE_H * math.cos(a), ball,
               ball + TOE_RY * (math.cos(a) - math.cos(b)) + TOE_M * math.sin(b))
    low = min(heights)
    return low - LIFT_SOFT_M * math.log(sum(math.exp((low - h) / LIFT_SOFT_M) for h in heights))


def swung(q, stride=1.0):
    """(ankle ahead of the hip, ankle over the floor) of a swinging foot at its leg's phase `q`:
    ahead, in the floor's frame a septic Hermite from toe-off to the next landing, meeting the
    planted foot's place, speed, acceleration and jerk at both ends; over the floor, the planted
    foot's pivot handed to the skim (LIFT_M) and back. Swung in the joints instead, the path knew
    no floor: the foot went under it and the knee, bent to lift it, snapped straight to land;
    quintic, the jerk jumped 3 rms at toe-off (2026-09-25)."""
    length, span = STRIDE_M * stride, 1.0 - TOE_OFF

    def floor(p):
        x, y, _pitch = planted(p, stride)
        return x + length * p, y

    u = (q % 1.0 - TOE_OFF) / span
    (x0, _y0), (x1, _y1) = ends(floor, TOE_OFF, -DIFF, span), ends(floor, 1.0, DIFF, span)
    pivot = planted(q, stride)[1]
    skim = septic(u / LIFT_RISE) * septic((1.0 - u) / LIFT_FALL)
    return (hermite(x0, x1, u) - length * (q % 1.0),
            pivot + skim * (LIFT_M - _least(q, stride) - pivot))


def leg(q, where, stride=1.0):
    """(hip, knee, ankle, toes) in the joints' signs for this leg at its phase `q`; `where(q)`
    its hip's (height over the floor, metres forward of the pelvis's centre): two-link IK to
    the ankle, planted (`planted`) or swinging (`swung`); the ankle and the toes from how the
    foot is pitched (`pitch_of`)."""
    q %= 1.0
    pitch = pitch_of(q, stride)
    x, y = planted(q, stride)[:2] if q < TOE_OFF else swung(q, stride)
    thigh, knee = _ik(x, y, *where(q))
    return -thigh, knee, thigh - knee - pitch, toes_of(q, pitch)


def toes_of(q, pitch):
    """The toes' bend, degrees, at the leg's phase `q` and the foot's `pitch`: flat on the floor
    while the heel is up over them; in the air they point with the foot, and flatten again to take
    the floor; lined up with it toes up. Curled up 20 degrees as the foot left the floor they
    touched it still (the drive at 40 N m/rad follows nothing in 0.05 s) and a slip fell
    (2026-09-27)."""
    u = max(0.0, (q % 1.0 - TOE_OFF) / (1.0 - TOE_OFF))
    return min(0.0, pitch) * (1.0 - 64.0 * u ** 3 * (1.0 - u) ** 3)


def walk(t, cadence=CADENCE, stride=None, glance=True, phase=None):
    """{joint: degrees} at `t` s of a walk at `cadence` strides a second - at `phase` of a
    stride where given, for a cadence that changes; `stride` scales the step, the sway and the
    knees' bend, `pace(cadence)` unless given; `glance` turns the head now and then."""
    stride = pace(cadence) if stride is None else stride
    p = (t * cadence if phase is None else phase) % 1.0
    out = stand()
    yaw = math.radians(_yaw(p, stride))
    for side, sign, phase in (('left', 1.0, p), ('right', -1.0, (p + 0.5) % 1.0)):
        hip, knee, ankle, toes = leg(phase, _hip_of(sign, stride), stride)
        out[side + '_hip'], out[side + '_knee'] = hip, knee
        out[side + '_ankle'], out[side + '_foot'] = ankle, toes
        # An arm swings with the other leg, forward as that leg reaches; the elbow, the wrist
        # and the fingers each a little later than the joint above.
        arm = 2.0 * math.pi * ((phase + 0.5) % 1.0)
        for k, (joint, mean, swing) in enumerate(ARM):
            out[side + joint] = mean + swing * (stride if k == 0 else 1.0) * math.cos(
                arm - k * TRAIL)
    out['pelvis'] = math.degrees(yaw)
    out['waist'] = -COUNTER * math.degrees(yaw)
    # The head holds its heading: it turns back what the pelvis and the torso turned.
    held = -(out['pelvis'] + out['waist'])
    out['head'] = held + ((16.0 * math.sin(2.0 * math.pi * t / 9.0) ** 3) if glance else 0.0)
    out['neck'] = 3.0
    return out


def sway(t, cadence=CADENCE, stride=None, phase=None):
    """(lateral metres, roll degrees, height metres, level metres): the pelvis's centre over the
    stance leg - left of the line while the left foot is down - dropping on the swing side (roll
    positive raises her left), its height over the floor, and that height's mean over the
    stride: the level the torso and head sail at while the pelvis rises and falls about it.
    Not a joint: what the floor does."""
    stride = pace(cadence) if stride is None else stride
    p = (t * cadence if phase is None else phase) % 1.0
    mid = math.cos(2.0 * math.pi * (p - MID_STANCE))
    roll = _roll(p) * stride
    level = sum(_hips(k / 48.0, stride) for k in range(48)) / 48.0 + HIP_DROP
    return (SHIFT_M * stride * mid, roll,
            _hips(p, stride) + HIP_DROP * math.cos(math.radians(roll)), level)


#: A swinging leg turns in at the hip, TWIST_DEG at mid-swing: its knee comes in toward the line,
#: left and right mirrored - wax on, wax off. The turn is nothing at toe-off and at the landing,
#: to its second derivative. Each foot's toes out TOE_OUT_DEG throughout: at 11 in and none out,
#: pigeon-toed; at 6 and 8, 7.4 out in stance and 2.5 swinging (2026-09-28).
TWIST_DEG, TOE_OUT_DEG = 6.0, 8.0


def tracks(t, cadence=CADENCE, phase=None, track=TRACK_M, widen=WIDEN_M):
    """(left, right, left twist, right twist): each foot's place off the line, metres, positive
    her left - `track` while planted, `widen` further out at mid-swing - and each leg's turn in
    at the hip, degrees about the vertical, positive turning a forward point to her left. Not
    joints the type has: what the hips' ab/adduction and rotation do, drawn."""
    p = (t * cadence if phase is None else phase) % 1.0
    places, turns = [], []
    for sign, q in ((1.0, p), (-1.0, (p + 0.5) % 1.0)):
        u = (q - TOE_OFF) / (1.0 - TOE_OFF) if q >= TOE_OFF else 0.0
        places.append(sign * (track + widen * math.sin(math.pi * u) ** 2))
        turns.append(sign * (TOE_OUT_DEG - TWIST_DEG * 64.0 * u ** 3 * (1.0 - u) ** 3))
    return tuple(places + turns)


def standing():
    """The pelvis's (lateral, roll, height, level) at a stand: legs straight but for a soft knee."""
    return 0.0, 0.0, _still_hip() + HIP_DROP, _still_hip() + HIP_DROP


def _still_hip():
    """The hips' height with both feet flat under them and the knees as `stand` bends them."""
    knee = math.radians(STAND_KNEE)
    return ANKLE_H + math.sqrt(THIGH ** 2 + SHANK ** 2 + 2.0 * THIGH * SHANK * math.cos(knee))


#: A stand's soft knee, degrees; the hip and the ankle take half of it each way, the shin upright.
STAND_KNEE = 6.0


def stand() -> dict[str, float]:
    """Every joint at rest: the knees soft, arms down and soft at the elbow, the fingers curled."""
    out: dict[str, float] = {name: 0.0 for name in JOINTS}
    out.update({'left_elbow': 10.0, 'right_elbow': 10.0, 'left_wrist': 5.0,
                'right_wrist': 5.0, 'left_gripper': 18.0, 'right_gripper': 18.0})
    for side in ('left', 'right'):
        out.update({side + '_hip': -STAND_KNEE / 2.0, side + '_knee': STAND_KNEE,
                    side + '_ankle': -STAND_KNEE / 2.0})
    return out


def blend(a, b, k):
    """{joint: degrees} `k` of the way from pose `a` to pose `b`, eased."""
    e = eased(k)
    return {joint: a.get(joint, 0.0) + (b.get(joint, 0.0) - a.get(joint, 0.0)) * e
            for joint in set(a) | set(b)}
