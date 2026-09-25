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
ANKLE_H, BALL, HEEL = 0.075, 0.135, 0.07

#: The feet walk a beam: each planted TRACK_M off the line, swung WIDEN_M further out round the
#: standing foot at mid-swing, the stand's STAND_M apart (half).
TRACK_M, WIDEN_M, STAND_M = 0.03, 0.01, HIP_HALF

#: The walk: metres a stride (two steps) at `stride` 1 - 1.15 was more than her 0.77 m legs
#: reach behind at toe-off, and the hips sank to let them.
STRIDE_M = 1.0

#: The cadence a stride of 1 goes with, strides a second; faster, the steps lengthen as the
#: cadence to PACE_POWER (`pace`); the hips are fitted every PACE_STEP of stride and weighed
#: between.
CADENCE, PACE_POWER, PACE_STEP = 0.85, 0.6, 0.05

#: The stance knee's bend at its straightest, degrees, at a stride of 1: soft, the hip and the
#: ball of the foot carry the glide; a longer stride bends it as the stride to KNEE_POWER.
KNEE_SOFT_DEG, KNEE_POWER = 6.0, 1.5

#: The foot lands on its ball, heel up (`pitch_of`), and pivots there all stance: the heel comes
#: down to the floor by SETTLE, mid-step, rises from HEEL_OFF and leaves it at toe-off, TOE_OFF.
#: Landed flat on a heel, the step came down as a slap; held up, she tripped along on her toes
#: (2026-09-25).
SETTLE, HEEL_OFF, TOE_OFF = 0.25, 0.40, 0.62

#: The foot's pitch toes-up at its knots: (phase, degrees, a stride, a stride squared), a quintic
#: between. The heel rises fastest at toe-off and on to 73 degrees in the air: eased to a stop
#: there, the whole foot stood still, the knee straightened -180 deg/s and then bent +409
#: (2026-09-25). The landing's the heel still coming down.
PITCH_KNOTS = ((SETTLE, 0.0, 0.0, 0.0), (HEEL_OFF, 0.0, 0.0, 0.0),
               (TOE_OFF, -45.0, -420.0, 1000.0), (1.0, -15.6, 136.0, -373.0),
               (1.0 + SETTLE, 0.0, 0.0, 0.0))

#: The middle of a leg's single support: from the other's toe-off to its own landing.
MID_STANCE = 0.5 * TOE_OFF

#: Where the ball is planted: the ankle over it at STANCE_AT, early, so the leg's reach is
#: behind her; centred, the thigh never passed upright and her feet were always in front.
STANCE_AT = 0.22

#: The hips ride as high as a stance leg reaches (REACH), smoothed: held level, the knees stood
#: at 33-40 degrees, a crouch. Swept (2026-09-25): stance knee at most 16 degrees, the thigh 20
#: behind upright at toe-off, the hips rising and falling 20 mm.

#: The pelvis alone moves: over the stance leg (metres; at 3 cm both legs leaned together, a
#: parallelogram), dropping on the swing side (degrees,
#: its obliquity) and turning about the spine (degrees, its rotation); the torso turns all of it
#: back (COUNTER 1), so the shoulders and the head go straight.
SHIFT_M, ROLL_DEG, TURN_DEG, COUNTER = 0.012, 7.0, 8.0, 1.0

#: How far each arm joint trails the one above it, radians of the stride.
TRAIL = 0.55

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


def eased(x):
    """0 to 1 over 0 to 1 with no jerk at either end (smootherstep)."""
    x = min(1.0, max(0.0, x))
    return x * x * x * (x * (6.0 * x - 15.0) + 10.0)


def _pivot(x, y, dx, dy, pitch):
    """The ankle, from a point (x, y) on the foot and the ankle's offset (dx, dy) from it on a flat
    foot, the foot pitched `pitch` degrees toes-up about that point."""
    c, s = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    return x + dx * c - dy * s, y + dx * s + dy * c


def pitch_of(q):
    """The foot's pitch toes-up, degrees, at this leg's phase `q`, through PITCH_KNOTS: flat
    from SETTLE, the heel rising through toe-off, round through the swing and the landing down
    to flat again - eased to a stop at the landing, the foot moved in steps."""
    q = q % 1.0 + (1.0 if q % 1.0 < SETTLE else 0.0)
    for (q0, *start), (q1, *end) in zip(PITCH_KNOTS, PITCH_KNOTS[1:]):
        if q < q1:
            span = q1 - q0
            scale = (1.0, span, span * span)
            return _hermite([v * s for v, s in zip(start, scale)],
                            [v * s for v, s in zip(end, scale)], (q - q0) / span)
    return 0.0


def planted(q, stride=1.0):
    """(ankle ahead of the hip, ankle over the floor, the foot's pitch toes-up) of a planted foot
    at its leg's phase `q`, metres and degrees: pivoting on its ball, the hip moving on at a
    stride a cycle. The stance's; the swing is the joints' own (`leg`)."""
    length = STRIDE_M * stride
    q %= 1.0
    pitch = pitch_of(q)
    x, y = _pivot(length * STANCE_AT + BALL, 0.0, -BALL, ANKLE_H, pitch)
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
#: the floor was a step, a jerk 156 times the median.
SAMPLES, ERODE, SOFT_MIN_M, BLUR = 240, 0.035, 0.002, 0.035


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


def _through(samples, p):
    """The periodic Catmull-Rom curve through `samples` (a stride's) at phase `p`."""
    n = len(samples)
    x = (p % 1.0) * n
    i, u = int(x), x - int(x)
    a, b, c, d = (samples[(i + k) % n] for k in (-1, 0, 1, 2))
    return 0.5 * (2.0 * b + (c - a) * u + (2.0 * a - 5.0 * b + 4.0 * c - d) * u * u
                  + (3.0 * (b - c) + d - a) * u * u * u)


def _fit(stride):
    """The hips' height at SAMPLES phases: the eroded limit, lowered until the curve through it
    is under the limit everywhere, and then as far again as a soft knee takes a leg's length."""
    limits = [_limit(k / SAMPLES, stride) for k in range(SAMPLES)]
    smooth = _blurred(_eroded(limits))
    over = max(_through(smooth, k / (4.0 * SAMPLES)) - _limit(k / (4.0 * SAMPLES), stride)
               for k in range(4 * SAMPLES))
    knee = math.radians(KNEE_SOFT_DEG * stride ** KNEE_POWER)
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
    below = _through(_FITS[round(low, 6)], p)
    return below if w < 1e-9 else (1.0 - w) * below + w * _through(
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


#: The swing ankle's lift over the path between toe-off and the landing, metres at mid-swing:
#: at 0.07, the foot pitched on past toe-off, the toes dragged 10 mm into the floor.
LIFT_M = 0.09

#: The step the swing's end conditions are differenced over, of a stride.
DIFF = 1e-4


#: Hermite bases, u^0..u^7 coefficients per end condition in order (value, rate, acceleration,
#: jerk) at u 0 then at u 1: the quintic from three, the septic from four.
_BASES = {3: ((1, 0, 0, -10, 15, -6), (0, 1, 0, -6, 8, -3), (0, 0, 0.5, -1.5, 1.5, -0.5),
              (0, 0, 0, 10, -15, 6), (0, 0, 0, -4, 7, -3), (0, 0, 0, 0.5, -1, 0.5)),
          4: ((1, 0, 0, 0, -35, 84, -70, 20), (0, 1, 0, 0, -20, 45, -36, 10),
              (0, 0, 0.5, 0, -5, 10, -7.5, 2), (0, 0, 0, 1 / 6, -2 / 3, 1, -2 / 3, 1 / 6),
              (0, 0, 0, 0, 35, -84, 70, -20), (0, 0, 0, 0, -15, 39, -34, 10),
              (0, 0, 0, 0, 2.5, -7, 6.5, -2), (0, 0, 0, 0, -1 / 6, 0.5, -0.5, 1 / 6))}


def _hermite(start, end, u):
    """The polynomial from `start` at u 0 to `end` at u 1, each (value, rate, acceleration[,
    jerk]) over u."""
    powers = [u ** k for k in range(2 * len(start))]
    return sum(w * sum(c * p for c, p in zip(basis, powers))
               for w, basis in zip(list(start) + list(end), _BASES[len(start)]))


def _ends(f, q, h, span):
    """Per coordinate of f at q: (value, rate, acceleration, jerk) over `span`, differenced one
    side, steps of `h` (negative, behind)."""
    return [(a, (4.0 * b - 3.0 * a - c) / (2.0 * h) * span, (a - 2.0 * b + c) / h ** 2 * span ** 2,
             (3.0 * (b - c) + d - a) / h ** 3 * span ** 3)
            for a, b, c, d in zip(*(f(q + k * h) for k in range(4)))]


def swung(q, stride=1.0):
    """(ankle ahead of the hip, ankle over the floor) of a swinging foot at its leg's phase `q`:
    in the floor's frame a septic Hermite from toe-off to the next landing, meeting the planted
    foot's place, speed, acceleration and jerk at both ends, lifted LIFT_M at mid-swing by a bump
    nothing at either end to its second derivative. Swung in the joints instead, the path knew
    no floor: the foot went under it and the knee, bent to lift it, snapped straight to land;
    quintic, the jerk jumped 3 rms at toe-off (2026-09-25)."""
    length, span = STRIDE_M * stride, 1.0 - TOE_OFF

    def floor(p):
        x, y, _pitch = planted(p, stride)
        return x + length * p, y

    u = (q % 1.0 - TOE_OFF) / span
    x, y = (_hermite(a, b, u) for a, b in zip(_ends(floor, TOE_OFF, -DIFF, span),
                                               _ends(floor, 1.0, DIFF, span)))
    return x - length * (q % 1.0), y + LIFT_M * 64.0 * u ** 3 * (1.0 - u) ** 3


def leg(q, where, stride=1.0):
    """(hip, knee, ankle, toes) in the joints' signs for this leg at its phase `q`; `where(q)`
    its hip's (height over the floor, metres forward of the pelvis's centre): two-link IK to
    the ankle, planted (`planted`) or swinging (`swung`); the ankle and the toes from how the
    foot is pitched (`pitch_of`)."""
    q %= 1.0
    pitch = pitch_of(q)
    x, y = planted(q, stride)[:2] if q < TOE_OFF else swung(q, stride)
    thigh, knee = _ik(x, y, *where(q))
    # The toes lie flat on the floor the whole stance, the heel over them; in the air they point
    # with the foot, and flatten again to take the floor.
    u = max(0.0, (q - TOE_OFF) / (1.0 - TOE_OFF))
    hold = 1.0 - 64.0 * u ** 3 * (1.0 - u) ** 3
    return -thigh, knee, thigh - knee - pitch, pitch * hold


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
        out[side + '_shoulder'] = 16.0 * stride * math.cos(arm)
        out[side + '_elbow'] = 20.0 + 9.0 * math.cos(arm - TRAIL)
        out[side + '_wrist'] = 8.0 + 6.0 * math.cos(arm - 2.0 * TRAIL)
        out[side + '_gripper'] = 20.0 + 5.0 * math.cos(arm - 3.0 * TRAIL)
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


#: A swinging leg turns in at the hip, TWIST_DEG at mid-swing: its knee comes in toward the line
#: and its foot goes round the standing one, left and right mirrored - wax on, wax off. The
#: turn is nothing at toe-off and at the landing, to its second derivative.
TWIST_DEG = 12.0


def tracks(t, cadence=CADENCE, phase=None):
    """(left, right, left twist, right twist): each foot's place off the line, metres, positive
    her left - TRACK_M while planted, WIDEN_M further out at mid-swing - and each leg's turn in
    at the hip, degrees about the vertical, positive turning a forward point to her left. Not
    joints the type has: what the hips' ab/adduction and rotation do, drawn."""
    p = (t * cadence if phase is None else phase) % 1.0
    places, turns = [], []
    for sign, q in ((1.0, p), (-1.0, (p + 0.5) % 1.0)):
        u = (q - TOE_OFF) / (1.0 - TOE_OFF) if q >= TOE_OFF else 0.0
        places.append(sign * (TRACK_M + WIDEN_M * math.sin(math.pi * u) ** 2))
        turns.append(-sign * TWIST_DEG * 64.0 * u ** 3 * (1.0 - u) ** 3)
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
