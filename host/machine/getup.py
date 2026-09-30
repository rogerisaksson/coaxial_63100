"""The gynoid's get-up: from her back or a side into the crouch the arrival rises from.

Onto her back, sat up on her hands, the heels drawn in, folded over her knees and lifted on her
hands over her feet into a crouch, the arrival's squat (`machine.arrival`) taking her up.

    getup = GetUp(machine)
    getup.begin(BACK)                          # down and still, her drives armed
    machine.loop.write(**getup.step(dt))       # every pass; `getup.stage`; `getup.done`
    arrival.play(getup.handed())               # done: the arrival's frames from her pose

A stream is data, a step at a time: (verb, stage, seconds, target) - `ease` the joints from where
they are to {joint: deg}; `plan` to a keyframe of the floor-up chain (`chain`), the joints
sampled between consecutive `plan` steps. A planner - a fast local model - writes one for how
she lies (docs/TODO.md).

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
_HAND = next(c for c in figure.CONTACTS if c[0] == 'hand')
_SEAT = next(c for c in figure.CONTACTS if c[0] == 'pelvis')
#: The hand's contact and the seat's (the buttocks' axes' ends), their segments' frames.
HAND_AT, HAND_R = _HAND[3], list(_HAND[2])[0]
SEAT_R, _HALF = list(_SEAT[2])
SEAT_ENDS = tuple((0.0, _SEAT[3][1], _SEAT[3][2] + d) for d in (-_HALF, _HALF))
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
#: Folded over the knees, the hands on the floor ahead: her centre of mass over the heels, sat -
#: sat with the trunk up it stays 0.13 m behind them, the hip past 130 degrees to get there.
FOLD = dict(phi=0.0, seat=True, shank=0.0, thigh=-150.0, tilt=-31.5, spine=92.0, neck=30.0,
            hand=0.30)
LIFT = dict(phi=0.0, shank=14.0, thigh=-140.0, tilt=-10.0, spine=84.0, neck=30.0, hand=0.35)
CROUCH = dict(phi=0.0, shank=40.0, thigh=-95.0, tilt=25.0, spine=53.0, neck=10.0, hand=0.40)

#: From her back or a side: straightened out she rolls onto her back (three falls of three,
#: 2026-09-30); propped on her elbows, up onto her hands, sat; folded over and lifted on her
#: hands, the hands behind her pushing - the planned arms reach on, the shoulders pinned at their
#: 40 N m behind her - into the crouch. The folds, the lifts, the crouches and their times from a
#: CMA-ES over four seats: its tenth generation's best and median stood from all four, the hips
#: and knees past a woman's 150 N m 0.0-1.0 % of the time (2026-09-30).
BACK = (('ease', 'unfold', 1.0, _pose(0.0, 0.0, 0.0, 0.0, 20.0, 0.0, 90.0)),
        ('ease', 'unfold', 0.6, {}),
        ('ease', 'prop', 0.8, _pose(0.0, 10.0, 0.0, 50.0, 40.0, -40.0, 90.0)),
        ('ease', 'prop', 1.0, _pose(-40.0, 20.0, 0.0, 40.0, 20.0, -30.0, 0.0)),
        ('ease', 'sit', 1.0, _pose(-80.0, 30.0, 0.0, 20.0, 10.0, -30.0, 0.0)),
        ('ease', 'sit', 1.0, joints(solved(SIT))),
        ('ease', 'sit', 1.0, {}),
        ('plan', 'sit', 0.0, SIT),
        ('plan', 'fold', 0.6, dict(SIT, hand=None, shoulder=90.0, elbow=20.0, spine=70.0,
                                   neck=20.0)),
        ('plan', 'fold', 1.3, FOLD),
        ('plan', 'lift', 2.1, LIFT),
        ('plan', 'crouch', 2.8, CROUCH),
        ('plan', 'crouch', 0.8, CROUCH))

#: The get-up's stages, in order.
STAGES = ('unfold', 'prop', 'sit', 'fold', 'lift', 'crouch')

#: Unfolded face down - the pelvis's forward FACE_DOWN of the way down - `BACK` gives up: it sits
#: her up from her back only.
FACE_DOWN = 0.5


class GetUp:

    """A stream's steps one after another, each pass's setpoints from the one that has her."""

    def __init__(self, machine):
        self.machine = machine
        self.world = machine.nodes['pelvis'].world
        self.stream, self.i, self.t, self.done, self.gave_up = (), 0, 0.0, False, False
        self.stage, self.begun, self.samples, self.last = STAGES[0], {}, [], {}

    def begin(self, stream=BACK):
        """From her pose as the loop reads it, the stream's first step on."""
        self.stream, self.i, self.t, self.done = tuple(stream), 0, 0.0, False
        self.gave_up = False
        self.stage = stream[0][1]
        self.begun = self._now()
        self.last, self.samples, self.plan_t = dict(self.begun), [], 0.0
        self.com_was, self.v, self.past, self.held = None, 0.0, False, None

    def _now(self):
        bus = self.machine.loop.bus
        return {j: bus.get(j + '.deg', 0.0) for j in figure.JOINTS}

    def _facing(self):
        """The pelvis's forward's world-up part: +1 on her back, -1 face down."""
        bus = self.machine.loop.bus
        return figure.quat(*(bus['pelvis.pose.q' + a] for a in 'wxyz'))[1][2]

    def step(self, dt):
        """{joint: degrees}: where every drive should be now; given up, where they are."""
        if self.gave_up:
            return dict(self.last)
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
        if self.stage == 'unfold' and stage != 'unfold' and self._facing() < -FACE_DOWN:
            self.gave_up = True
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
