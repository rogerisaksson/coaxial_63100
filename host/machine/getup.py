"""The gynoid's get-up: the steps a plan is made of, run into the squat the arrival rises from.

Straightened out, rolled onto her front, the knees drawn under her, sat back on her heels and
onto her feet on her hands into the arrival's squat (`machine.arrival`) - each step a fragment
(`fragment`) of the stream `machine.planner` writes from what `machine.observer` says.

    getup = GetUp(machine)
    getup.begin(stream, marks)                 # down and still, her drives armed
    machine.loop.write(**getup.step(dt))       # every pass; `getup.stage`; `getup.ended()`
    arrival.play(getup.handed())               # done: the arrival's frames from her pose

A stream is data, a step at a time: (verb, stage, seconds, target) - `ease` the joints from where
they are to {joint: deg}; `plan` to a keyframe of the floor-up chain (`chain`), the joints
sampled between consecutive `plan` steps.

The plan's frame: x her left, y up, z her way, the balls of her feet at z 0, `arrival.FEET_X`
either side. A keyframe: each foot pitched `phi` toes-down about its ball, the shank and the thigh
at their angles (deg about x: a segment's -y at (y, z) = (-cos a, -sin a)), the pelvis tipped
`tilt`, the spine and neck; the hands on the floor at z `hand`, or the arms at `shoulder` and
`elbow`; `seat` sat, the thigh solved to set the seat on the floor.
"""
import math

from machine import arrival, figure, gait, walkplan
from machine.curves import eased

T, S = gait.THIGH, gait.SHANK
#: The ankle from the ball of its foot, the toes flat.
ANKLE_FROM_BALL = (0.0, gait.ANKLE_H - gait.TOE_RY, -gait.BALL)
#: The hand's contact and the seat's (the buttocks' axes' ends), their segments' frames.
HAND_AT, HAND_R = figure.FIST_AT, figure.FIST_R
SEAT_R = figure.SEAT_R
SEAT_ENDS = tuple((0.0, figure.SEAT_Y, z) for z in figure.SEAT_Z)
ARM = -next(s for s in figure.SEGMENTS if s[0] == 'left_forearm')[3][1]
FOREARM = -next(s for s in figure.SEGMENTS if s[0] == 'left_hand')[3][1]
HANDS = {'left_wrist': 0.0, 'right_wrist': 0.0, 'left_gripper': 20.0, 'right_gripper': 20.0}

#: The plan's joints sampled every SAMPLE_S.
SAMPLE_S = 0.02

#: Past the heels - her centre of mass PAST_M ahead of them - the arms hold where they are and
#: the shins feed it back to the plan's, SHIN_K deg a metre and SHIN_D a m/s. Unfed she ran on
#: over her toes onto her head, 0.25 m/s through her feet (2026-09-30).
PAST_M, SHIN_K, SHIN_D = 0.02, 150.0, 17.0

#: Handed over, the arms lifted clear of the floor over CLEAR_S before the arrival's squat, the
#: legs its stance's whatever they bear: lifted at once, the hands behind pushed her up 60 mm
#: in 0.2 s and over; swung by the arrival's light legs, her feet were flung 0.5 m (2026-09-30).
CLEAR_S = 0.7
CLEAR = {'left_shoulder': 0.0, 'left_elbow': 110.0, 'right_shoulder': 0.0, 'right_elbow': 110.0}


def _u(a):
    r = math.radians(a)
    return -math.cos(r), -math.sin(r)


def chain(k):
    """((ankle), (knee), (hip), (pelvis)), each (y, z), up from the floor for keyframe `k`."""
    a = figure.apply(figure.rx(math.radians(k['phi'])), ANKLE_FROM_BALL)
    ay, az = gait.TOE_RY + a[1], a[2]
    sy, sz = _u(k['shank'])
    ky, kz = ay - S * sy, az - S * sz
    ty, tz = _u(k['thigh'])
    hy, hz = ky - T * ty, kz - T * tz
    t = math.radians(k['tilt'])
    return (ay, az), (ky, kz), (hy, hz), (hy + gait.HIP_DROP * math.cos(t),
                                          hz + gait.HIP_DROP * math.sin(t))


def seat_low(k):
    """The lowest of the seat's contacts over the floor, m."""
    py = chain(k)[3][0]
    return py + min(figure.apply(figure.rx(math.radians(k['tilt'])), e)[1]
                    for e in SEAT_ENDS) - SEAT_R


def seated(k):
    """`k` with its thigh turned (about the knee) to set the seat on the floor."""
    lo, hi = -179.0, -90.0
    if seat_low(dict(k, thigh=lo)) > 0.0:
        return dict(k, thigh=lo)
    for _ in range(50):
        mid = (lo + hi) / 2.0
        if seat_low(dict(k, thigh=mid)) > 0.0:
            hi = mid
        else:
            lo = mid
    return dict(k, thigh=(lo + hi) / 2.0)


def solved(k):
    """`k` sat as it asks, and its seat never under the floor."""
    k = seated(k) if k.get('seat') else k
    return seated(dict(k, seat=True)) if seat_low(k) < 0.0 else k


def place(k):
    """(pelvis, turn) of keyframe `k`, the plan's frame."""
    py, pz = chain(k)[3]
    return (0.0, py, pz), figure.rx(math.radians(k['tilt']))


def _hand(d, k):
    at, turn = figure.frames(d, *place(k))['left_hand']
    c = figure.add(at, figure.apply(turn, HAND_AT))
    return c[1] - HAND_R, c[2]


def arms(k, d):
    """(shoulder, elbow) deg setting the hands' contacts on the floor at z `k['hand']`: the
    sagittal two links, then Newton on the figure's own frames."""
    at = figure.frames(d, *place(k))['left_upper_arm'][0]
    p = math.radians(k['tilt'] + k['spine'])
    fore = math.hypot(FOREARM - HAND_AT[1], HAND_AT[2])
    delta = math.atan2(HAND_AT[2], FOREARM - HAND_AT[1])
    dy, dz = HAND_R - at[1], k['hand'] - at[2]
    far = min(math.hypot(dy, dz), ARM + fore - 1e-6)

    def angle(a, b, c):
        return math.acos(max(-1.0, min(1.0, (a * a + b * b - c * c) / (2.0 * a * b))))
    bend = math.pi - angle(ARM, fore, far)
    s = math.degrees(p - math.atan2(-dz, -dy) - angle(ARM, far, fore))
    e = math.degrees(bend - delta)
    for _ in range(8):
        y, z = _hand(dict(d, left_shoulder=s, left_elbow=e), k)
        ry, rz = y, z - k['hand']
        if abs(ry) + abs(rz) < 2e-5:
            break
        ys, zs = _hand(dict(d, left_shoulder=s + 0.01, left_elbow=e), k)
        ye, ze = _hand(dict(d, left_shoulder=s, left_elbow=e + 0.01), k)
        a, b, c, g = (ys - y) / 0.01, (ye - y) / 0.01, (zs - z) / 0.01, (ze - z) / 0.01
        det = a * g - b * c
        if abs(det) < 1e-12:
            break
        ds, de = -(g * ry - b * rz) / det, -(-c * ry + a * rz) / det
        n = max(1.0, math.hypot(ds, de) / 5.0)
        s, e = s + ds / n, e + de / n
    return s, e


def joints(k):
    """{joint: deg} for keyframe `k`, both sides alike."""
    d = dict({j: 0.0 for j in figure.JOINTS}, **HANDS)
    for side in ('left', 'right'):
        d[side + '_hip'] = k['thigh'] - k['tilt']
        d[side + '_knee'] = k['shank'] - k['thigh']
        d[side + '_ankle'] = k['phi'] - k['shank']
        d[side + '_foot'] = -k['phi']
    d['spine'], d['neck'] = k['spine'], k['neck']
    s, e = arms(k, d) if k.get('hand') is not None else (k['shoulder'], k['elbow'])
    for side in ('left', 'right'):
        d[side + '_shoulder'], d[side + '_elbow'] = s, e
    return d


def com(k, d):
    """Her centre of mass for keyframe `k` at joints `d`, the plan's frame."""
    return figure.com(d, *place(k))


def _mix(a, b, w):
    return {key: b.get(key) if a.get(key) is None or b.get(key) is None
            or isinstance(a.get(key), bool) else a[key] + (b[key] - a[key]) * w
            for key in set(a) | set(b)}


def sample(frames):
    """[(t, stage, keyframe, joints, com z)] every SAMPLE_S over `frames` [(stage, seconds, k)],
    each keyframe eased from the one before; a hand's arm as its keyframe's IK has it, eased."""
    frames = [(n, s, solved(k)) for n, s, k in frames]
    frames = [(n, s, dict(k, shoulder=d['left_shoulder'], elbow=d['left_elbow'])
               if k.get('hand') is not None else k) for (n, s, k), d in
              zip(frames, [joints(k) for _n, _s, k in frames])]
    out, t = [], 0.0
    for (_n, _s, a), (name, span, b) in zip(frames, frames[1:]):
        n = max(1, int(round(span / SAMPLE_S)))
        for i in range(1, n + 1):
            k = _mix(a, b, eased(i / n))
            k['seat'] = a.get('seat') and b.get('seat')
            if b.get('hand') is None:
                k['hand'] = None
            k = solved(k)
            d = joints(k)
            t += span / n
            out.append((t, name, k, d, com(k, d)[2]))
    return out


def _pose(hip, knee, ankle, spine, neck, shoulder, elbow):
    """{joint: deg}, both sides alike, the rest straight."""
    d = dict({j: 0.0 for j in figure.JOINTS}, **HANDS)
    for side in ('left_', 'right_'):
        d.update({side + 'hip': hip, side + 'knee': knee, side + 'ankle': ankle,
                  side + 'shoulder': shoulder, side + 'elbow': elbow})
    return dict(d, spine=spine, neck=neck)


#: Sat, the heels in, the pelvis rolled back on the buttocks and the spine on: her centre of mass
#: 26 mm ahead of the seat, the hands behind on the floor.
SIT = dict(phi=0.0, seat=True, shank=-12.0, thigh=-150.0, tilt=-25.0, spine=45.0, neck=10.0,
           hand=-0.45)
#: Straightened out; from her back, sat up on the spine and the hips, the heels drawn in.
UNFOLD = (('ease', 'unfold', 1.0, _pose(0.0, 0.0, 0.0, 0.0, 20.0, 0.0, 90.0)),
          ('ease', 'unfold', 0.6, {}))
SIT_UP = (('ease', 'prop', 0.8, _pose(0.0, 30.0, 0.0, 60.0, 40.0, 60.0, 10.0)),
          ('ease', 'prop', 1.2, _pose(-80.0, 30.0, 0.0, 40.0, 20.0, 60.0, 10.0)),
          ('ease', 'sit', 1.0, _pose(-80.0, 30.0, 0.0, 20.0, 10.0, -30.0, 0.0)),
          ('ease', 'sit', 1.0, joints(solved(SIT))),
          ('ease', 'sit', 1.0, {}))
BACK = UNFOLD + SIT_UP

#: Face down, the kneel: the knees drawn under her hips, the chest down; sat back on her heels,
#: the pelvis 0.40 m up; onto her feet on her hands - leant onto them, the knees up on the
#: tucked toes, into the arrival's own squat, it rising her from there. From CMA-ESs over the
#: states real falls left her in, scored by the observer through the arrival: 8 of 16 kneels
#: stood (2026-10-01); lifted by the floor-up plan instead she stood in none of 224, the knees
#: never off the floor, and squatted on her own she fell as the arrival took her.
KNEES_UNDER = (('ease', 'prop', 1.19, _pose(3.0, 52.5, 24.6, -7.9, -20.0, 131.1, 84.9)),
               ('ease', 'prop', 1.46, _pose(-87.8, 96.8, 14.8, 22.9, -20.0, 163.1, 114.4)))
SIT_BACK = (('ease', 'sit', 0.78, _pose(-156.5, 129.2, 12.6, -10.0, -20.0, 170.0, 19.7)),
            ('ease', 'sit', 2.27, dict(_pose(-96.5, 128.6, -4.5, 40.4, -20.0, 51.7, 6.8),
                                       left_foot=18.1, right_foot=18.1)))
ONTO_FEET = (('ease', 'lift', 0.42, dict(_pose(-143.9, 160.0, -60.0, 51.5, -20.0, 86.1, 18.2),
                                         left_foot=35.0, right_foot=35.0)),
             ('ease', 'lift', 1.57, dict(_pose(-117.7, 133.4, -43.7, 64.1, -20.0, 134.0, 0.0),
                                         left_foot=47.5, right_foot=47.5)),
             ('ease', 'crouch', 1.48, arrival.angles_of(arrival._squat())),
             ('ease', 'crouch', 1.0, {}))

#: Face down, onto her back first: laid flat, the left knee drawn up and the left arm reaching
#: over her, the spine rolled and the head turned, then over, and flat. From a CMA-ES over the
#: roll laid face down: 157 of its 224 ended on her back (2026-09-30).
ROLL = (('ease', 'roll', 0.8, _pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0)),
        ('ease', 'roll', 0.8, dict(_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0), left_hip=-80.0,
                                   left_hip_roll=6.0, left_hip_yaw=33.0, left_knee=80.0,
                                   left_shoulder=117.0, left_elbow=58.0, right_shoulder=120.0,
                                   waist=7.0, spine_roll=37.0, head=60.0)),
        ('ease', 'roll', 1.2, dict(_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0), left_hip=-38.0,
                                   left_knee=2.0, left_shoulder=50.0, left_elbow=102.0)),
        ('ease', 'roll', 1.0, _pose(0.0, 0.0, 0.0, 0.0, 20.0, 0.0, 90.0)))


def _mirrored(pose):
    """`pose` for the other side: left and right swapped, the trunk's and the head's turns back."""
    out = {}
    for j, v in pose.items():
        side, _, rest = j.partition('_')
        if side in ('left', 'right'):
            out[('right_' if side == 'left' else 'left_') + rest] = v
        else:
            out[j] = -v if j in ('spine_roll', 'waist', 'head') else v
    return out


#: ROLL over her right side, lifting her left, and over her left: the one her head's IMU finds
#: lifted (`observer.status`'s left_up). One way only, lain on her left side she rolled onto her
#: face three times and was sat up from there (the lace, 2026-09-30).
ROLLS_BY = {1: ROLL, -1: tuple((v, s, t, _mirrored(p)) for v, s, t, p in ROLL)}

#: From her back onto her front: the left knee drawn up and across, both arms up over her head,
#: the waist turned back, then the knee down and the left arm reaching on. From a CMA-ES from her
#: back scored by the observer: her face 0.75 down at its end (2026-09-30); ROLL from her back
#: left her there, face up 0.98, 15 falls in 20.
TO_FRONT = (('ease', 'roll', 0.8, _pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0)),
            ('ease', 'roll', 0.5, dict(_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0), left_hip=-117.4,
                                       left_hip_roll=-1.0, left_hip_yaw=42.7, left_knee=87.5,
                                       left_shoulder=120.1, left_elbow=101.2,
                                       right_shoulder=116.5, waist=-30.9, spine_roll=-3.9,
                                       head=-31.8)),
            ('ease', 'roll', 0.5, dict(_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0), left_hip=-4.6,
                                       left_knee=39.6, left_shoulder=170.0, left_elbow=51.9)),
            ('ease', 'roll', 0.9, _pose(0.0, 0.0, 0.0, 0.0, 20.0, 0.0, 90.0)))
FRONTS_BY = {1: TO_FRONT, -1: tuple((v, s, t, _mirrored(p)) for v, s, t, p in TO_FRONT)}

#: The get-up's stages, in order.
STAGES = ('unfold', 'roll', 'prop', 'sit', 'fold', 'lift', 'crouch')


def fragment(step, now):
    """A plan's step (`planner.STEPS`) as the stream's steps, for her as `now` says."""
    if step in ('roll onto back', 'roll onto front'):
        return (ROLLS_BY if step == 'roll onto back' else FRONTS_BY)[
            1 if now['left_up'] >= 0.0 else -1]
    return {'straighten out': UNFOLD, 'sit up': SIT_UP, 'knees under': KNEES_UNDER,
            'sit back on heels': SIT_BACK,
            'onto feet': ONTO_FEET}[step]


class GetUp:

    """A stream's steps one after another, each pass's setpoints from the one that has her."""

    def __init__(self, machine):
        self.machine = machine
        self.world = machine.nodes['pelvis'].world
        self.stream, self.i, self.t, self.done, self.marks = (), 0, 0.0, False, []
        self.stage, self.begun, self.samples, self.last = STAGES[0], {}, [], {}

    def begin(self, stream=BACK, marks=()):
        """From her pose as the loop reads it, the stream's first step on; `marks` [(index,
        step)] where a plan's steps end, each read once by `ended`."""
        self.stream, self.i, self.t, self.done = tuple(stream), 0, 0.0, False
        self.marks = list(marks)
        self.stage = stream[0][1]
        self.begun = self._now()
        self.last, self.samples, self.plan_t = dict(self.begun), [], 0.0
        self.com_was, self.v, self.past, self.held = None, 0.0, False, None

    def _now(self):
        bus = self.machine.loop.bus
        return {j: bus.get(j + '.deg', 0.0) for j in figure.JOINTS}

    def ended(self):
        """The plan's step that has just ended, once, or None."""
        if self.marks and (self.i >= self.marks[0][0] or self.done):
            return self.marks.pop(0)[1]
        return None

    def step(self, dt):
        """{joint: degrees}: where every drive should be now."""
        self.t += dt
        verb, stage, span, target = self.stream[self.i]
        while self.t >= span and not self.done:
            self.t -= span
            if verb == 'plan':
                self.plan_t += span
            if self.i + 1 >= len(self.stream):
                self.done = True
                break
            self.i += 1
            if verb == 'ease':
                self.begun = dict(self.last)
            verb, stage, span, target = self.stream[self.i]
            if verb == 'plan' and not self.samples:
                self._plan()
        self.stage = stage
        if verb == 'ease':
            k = eased(self.t / span) if span > 0.0 else 1.0
            self.last = {j: v + (target.get(j, v) - v) * k for j, v in self.begun.items()}
            return dict(self.last)
        return self._planned(self.plan_t + min(self.t, span))

    def _plan(self):
        """The stream's `plan` steps from here sampled, the plan's frame set on the world's:
        her way the pelvis's heading, the balls of her feet where it has them."""
        plans = []
        for verb, stage, span, k in self.stream[self.i:]:
            if verb != 'plan':
                break
            plans.append((stage, span, k))
        self.samples, self.plan_t = sample(plans), 0.0
        bus = self.machine.loop.bus
        turn = figure.quat(*(bus['pelvis.pose.q' + a] for a in 'wxyz'))
        yaw = math.atan2(turn[0][2], turn[2][2])
        self.way = (math.sin(yaw), math.cos(yaw))
        self.dz = self._along(self.world.data.xpos[self.world.model.body('left_toes').id])

    def _along(self, p):
        """How far along her way a world point is, m. Measured in the world's z, fallen facing
        back she was fed the wrong way and sank to the floor three tries in three (2026-09-30)."""
        return float(p[0]) * self.way[0] + float(p[2]) * self.way[1]

    def _planned(self, t):
        """The plan's joints at `t` into it; past the heels, the arms held and the shins
        feeding her centre of mass back to the plan's."""
        s = self.samples
        j = next((n for n, row in enumerate(s) if row[0] >= t), len(s) - 1)
        ta, da, ca = (s[j - 1][0], s[j - 1][3], s[j - 1][4]) if j else (0.0, self.last, None)
        tb, _stage, _k, db, cb = s[j]
        w = 0.0 if tb <= ta else max(0.0, min(1.0, (t - ta) / (tb - ta)))
        out = {n: da.get(n, v) + (v - da.get(n, v)) * w for n, v in db.items()}
        want = cb if ca is None else ca + (cb - ca) * w
        if self.stage in ('lift', 'crouch'):
            out = self._fed(out, want + self.dz)
        self.last = out
        return dict(out)

    def _fed(self, out, want):
        bus = self.machine.loop.bus
        cz = self._along((bus['pelvis.pose.com_x'], 0.0, bus['pelvis.pose.com_z']))
        if self.com_was is not None:
            self.v = (cz - self.com_was) / 0.001
        self.com_was = cz
        d, m = self.world.data, self.world.model
        heel = min(self._along(d.xpos[m.body(side + '_foot').id])
                   for side in ('left', 'right')) - gait.HEEL
        self.past = self.past or cz > heel + PAST_M
        if not self.past:
            return out
        if self.held is None:
            self.held = {j: bus[j + '.deg'] for side in ('left_', 'right_')
                         for j in (side + 'shoulder', side + 'elbow')}
        shin = SHIN_K * (want - cz) - SHIN_D * self.v
        out = dict(out, **self.held)
        for side in ('left_', 'right_'):
            out[side + 'ankle'] -= shin
            out[side + 'knee'] += shin
        return out

    def handed(self):
        """The arrival's keyframes from her pose as the loop reads it - the pelvis tipped and
        facing as it is, the feet where they stand, the upper body as measured - the hands
        lifted clear over CLEAR_S, then its own from the squat on, moved to her feet."""
        bus = self.machine.loop.bus
        pel = (bus['pelvis.pose.x'], bus['pelvis.pose.y'], bus['pelvis.pose.z'])
        turn = figure.quat(*(bus['pelvis.pose.q' + a] for a in 'wxyz'))
        yaw = math.degrees(math.atan2(turn[0][2], turn[2][2]))
        # the tilt in her own frame: read about the world's x, fallen facing back it came out
        # backwards and the legs were solved straight, flinging her up to 1.1 m (2026-09-30)
        local = figure.mul(figure.ry(-math.radians(yaw)), turn)
        first = {'pelvis': pel, 'tilt': math.degrees(math.atan2(local[2][1], local[1][1])),
                 'yaw': yaw, 'planted': True,
                 'joints': {j: bus[j + '.deg'] for j in walkplan.UPPER}}
        for side, sign in (('left', 1.0), ('right', -1.0)):
            angles = tuple(math.radians(bus[side + k + '.deg']) for k in figure.LEG)
            ankle, foot = figure.foot_of(sign, pel, turn, angles)
            foot = figure.mul(figure.ry(-math.radians(yaw)), foot)
            first[side] = (ankle, math.degrees(math.atan2(foot[2][1], foot[1][1])))
        dx = (first['left'][0][0] + first['right'][0][0]) / 2.0
        dz = (first['left'][0][2] + first['right'][0][2]) / 2.0
        clear = dict(first, joints=dict(first['joints'], **CLEAR))
        return ([('squat', 0.0, first), ('squat', CLEAR_S, clear)]
                + [(s, t, dict(arrival.moved(f, dx, dz, yaw), planted=True) if s == 'squat'
                    else arrival.moved(f, dx, dz, yaw))
                   for s, t, f in arrival.keyframes(gait.CADENCE)[1:]])
