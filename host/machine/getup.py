"""The gynoid's get-up: the steps a plan is made of, run into the squat the arrival rises from.

Straightened out, rolled onto her front, the knees drawn under her, sat back on her heels and
onto her feet on her hands into the arrival's squat (`machine.arrival`) - each step a fragment
(`fragment`) of the stream `machine.planner` writes from what `machine.observer` says.

    getup = GetUp(machine)
    getup.begin(stream, marks)                 # down and still, her drives armed
    machine.loop.write(**getup.step(dt))       # every pass; `getup.stage`; `getup.ended()`
    arrival.play(getup.handed())               # done: the arrival's frames from her pose

A stream is data, a step at a time: (verb, stage, seconds, target) - `ease` the joints from where
they are to {joint: deg}.
"""
import math

from machine import arrival, figure, gait, walkplan

HANDS = {'left_wrist': 0.0, 'right_wrist': 0.0, 'left_gripper': 20.0, 'right_gripper': 20.0}

#: Handed over, the arms lifted clear of the floor over CLEAR_S before the arrival's squat, the
#: legs its stance's whatever they bear: lifted at once, the hands behind pushed her up 60 mm
#: in 0.2 s and over; swung by the arrival's light legs, her feet were flung 0.5 m (2026-09-30).
CLEAR_S = 0.7
CLEAR = {'left_shoulder': 0.0, 'left_elbow': 110.0, 'right_shoulder': 0.0, 'right_elbow': 110.0}

#: A step the observer ends on its outcome (`observer.EARLY`) once it has held REACHED_S: her
#: thigh lifted off the floor a pass read as crouched, ended at its start (2026-10-01).
REACHED_S = 0.2


def _pose(hip, knee, ankle, spine, neck, shoulder, elbow):
    """{joint: deg}, both sides alike, the rest straight."""
    d = dict({j: 0.0 for j in figure.JOINTS}, **HANDS)
    for side in ('left_', 'right_'):
        d.update({side + 'hip': hip, side + 'knee': knee, side + 'ankle': ankle,
                  side + 'shoulder': shoulder, side + 'elbow': elbow})
    return dict(d, spine=spine, neck=neck)


#: Straightened out.
UNFOLD = (('ease', 'unfold', 1.0, _pose(0.0, 0.0, 0.0, 0.0, 20.0, 0.0, 90.0)),
          ('ease', 'unfold', 0.2, {}))
#: Face down, the kneel: the knees drawn under her hips, the chest down; sat back on her heels,
#: upright over her toes; onto her feet - the arms thrown on and the trunk folded over the knees,
#: up on the tucked toes, the heels down into a squat on straight arms, then the arrival's own -
#: ended by the observer as she crouches still and flat-footed over her feet (`observer.EARLY`).
#: From CMA-ESs over the states real falls left her in, scored by the observer through the
#: arrival: back on her heels from 18 of 22 kneels, walking off from 12 of 12 on her pads, 11 of
#: 12 bare (2026-10-01); lifted by the floor-up plan instead she stood in none of 224, the knees
#: never off the floor.
#: Her hands put down under her shoulders, 100 deg, not reached out ahead at the search's 131 and
#: 163 (the user): 16 falls of 16 up as before (2026-10-01).
#: The crouch's hip -126.5: her thighs as modules (r 58 -> 72 mm) meet her torso at -126, -140
#: before; asked -138.7 the hip drove 100 N m into it, on her hands 0.17 m ahead of her feet, and
#: none of 5 falls walked again; re-searched over the onto feet's three, 5 of 5 (2026-10-03).
KNEES_UNDER = (('ease', 'prop', 1.19, _pose(3.0, 52.5, 24.6, -7.9, -20.0, 100.0, 84.9)),
               ('ease', 'prop', 1.46, _pose(-87.8, 96.8, 14.8, 22.9, -20.0, 100.0, 114.4)))
#: Sat back over 0.6 s, not the search's 0.3: her neck whipped 866 -> 520 deg/s, a foot 3.9 -> 2.8
#: m/s, 16 falls of 16 up as before (2026-10-01). The hip -130, where her torso stops her thigh,
#: not the search's -163.2 - past -142 her femur goes through her hip's yaw drum: 5 falls of 5
#: walking at 20.4-23.1 s (2026-10-03).
SIT_BACK = (('ease', 'sit', 0.6, _pose(-130.0, 158.2, 12.2, 60.0, -20.0, 54.3, 12.0)),
            ('ease', 'sit', 3.08, dict(_pose(-21.9, 160.0, -46.8, 30.9, -20.0, 72.7, 65.2),
                                       left_foot=54.5, right_foot=54.5)))
ONTO_FEET = (('ease', 'lift', 1.11, dict(_pose(-14.9, 139.2, -33.3, 37.7, -20.0, 68.2, -8.9),
                                         left_foot=28.6, right_foot=28.6)),
             ('ease', 'lift', 0.48, dict(_pose(-134.5, 122.0, -40.0, 38.9, -20.0, 141.9, -16.7),
                                         left_foot=36.9, right_foot=36.9)),
             ('ease', 'crouch', 1.94, dict(_pose(-126.5, 113.3, -33.8, 35.2, -20.0, 75.4, 6.0),
                                           left_foot=3.0, right_foot=3.0)),
             ('ease', 'crouch', 2.23, arrival.angles_of(arrival._squat())),
             ('ease', 'crouch', 0.3, {}))


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


#: From her back onto her front, the arms pushing, the legs near straight (the user, 2026-10-02):
#: the bottom arm overhead, the top arm bent by her side; the top arm pressed down and straightened,
#: its knee 60 -> 13 deg, the trunk turning; the recovery position, the top knee down in front;
#: into what the knees under begins from - ended prone flat, she rolled back. CMA-ES over its 30
#: knobs from flat and two logged falls: from all ten face down at -0.98 to -1.00 and knelt, a
#: foot at 1.3 m/s, the hip and knee 133 deg/s. The leg-pushed roll before it, searched on the
#: 35 kg build, lay on her back on the modules', its planted foot never down; thrown, a foot flew
#: 4.9 m/s, the hip and knee 803 deg/s (2026-10-01, 2026-10-02). Its hip roll 27, not the
#: search's 30: the gimbal turns -35..+28 (`skeleton.FORK_R`) (2026-10-03).
_BASE = _pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0)
TO_FRONT = (('ease', 'roll', 0.79, dict(_BASE, left_hip=-47.0, left_knee=60.0, left_hip_roll=27.0,
                                       left_hip_yaw=17.8, left_shoulder=65.5, left_elbow=69.9,
                                       right_shoulder=154.7, waist=43.9, spine_roll=-35.0,
                                       head=-13.2)),
            ('ease', 'roll', 0.54, dict(_BASE, left_hip=-50.8, left_knee=13.4, left_hip_roll=27.0,
                                       left_hip_yaw=17.8, left_shoulder=-6.3, left_elbow=2.5,
                                       right_shoulder=205.3, waist=-31.6, spine_roll=12.4,
                                       head=-13.2)),
            ('ease', 'roll', 1.16, dict(_BASE, left_hip=-75.1, left_knee=80.5, right_knee=56.0,
                                       left_shoulder=116.6, left_elbow=139.1,
                                       right_shoulder=223.5, waist=-1.2, head=-13.2)),
            ('ease', 'roll', 1.12, dict(_BASE, left_knee=56.4, right_hip=-9.6, right_knee=38.3,
                                       left_shoulder=156.9, left_elbow=18.3, right_shoulder=27.4,
                                       right_elbow=140, waist=53.1)),
            ('ease', 'roll', 0.3, {}))
FRONTS_BY = {1: TO_FRONT, -1: tuple((v, s, t, _mirrored(p)) for v, s, t, p in TO_FRONT)}

#: The get-up's stages, in order.
STAGES = ('unfold', 'roll', 'prop', 'sit', 'lift', 'crouch')


def fragment(step, now):
    """A plan's step (`planner.STEPS`) as the stream's steps, for her as `now` says."""
    if step == 'roll onto front':
        return FRONTS_BY[1 if now['left_up'] >= 0.0 else -1]
    return {'straighten out': UNFOLD, 'knees under': KNEES_UNDER,
            'sit back on heels': SIT_BACK,
            'onto feet': ONTO_FEET}[step]


def _curve(begun, stream):
    """([pose a keyframe, `begun` first], [deg/s a keyframe]): a curve through the keyframes,
    each one's rate its neighbours' (Fritsch-Carlson: none where a joint turns back, never
    more than 3 of either side's) - at rest only where the stream begins and ends. Eased to a
    stop at each, her setpoints stood 1.67 s from the unfold to the crouch, through it 0.59
    (2026-10-01)."""
    knots = [dict(begun)]
    for _verb, _stage, _span, target in stream:
        knots.append({j: target.get(j, v) for j, v in knots[-1].items()})
    spans = [max(1e-6, step[2]) for step in stream]
    slopes = [{j: 0.0 for j in begun}]
    for k in range(1, len(knots) - 1):
        rate = {}
        for j in begun:
            d0 = (knots[k][j] - knots[k - 1][j]) / spans[k - 1]
            d1 = (knots[k + 1][j] - knots[k][j]) / spans[k]
            rate[j] = (0.0 if d0 * d1 <= 0.0 else
                       math.copysign(min(0.5 * abs(d0 + d1), 3.0 * min(abs(d0), abs(d1))), d0))
        slopes.append(rate)
    slopes.append({j: 0.0 for j in begun})
    return knots, slopes


class GetUp:

    """A stream's steps one after another, each pass's setpoints from the one that has her."""

    def __init__(self, machine):
        self.machine = machine
        self.world = machine.nodes['pelvis'].world
        self.stream, self.i, self.t, self.done, self.marks = (), 0, 0.0, False, []
        self.stage, self.begun, self.last, self.holding = STAGES[0], {}, {}, 0.0

    def begin(self, stream, marks=()):
        """From her pose as the loop reads it, the stream's first step on; `marks` [(index,
        step)] where a plan's steps end, each read once by `ended`."""
        self.stream, self.i, self.t, self.done = tuple(stream), 0, 0.0, False
        self.marks = list(marks)
        self.stage = stream[0][1]
        self.begun = self._now()
        self.last, self.holding = dict(self.begun), 0.0
        self.knots, self.slopes = _curve(self.begun, self.stream)

    def _now(self):
        bus = self.machine.loop.bus
        return {j: bus.get(j + '.deg', 0.0) for j in figure.JOINTS}

    def ended(self):
        """The plan's step that has just ended, once, or None."""
        if self.marks and (self.i >= self.marks[0][0] or self.done):
            return self.marks.pop(0)[1]
        return None

    def reached(self, ok, dt):
        """The next mark's step ended now once `ok` has held REACHED_S - the stream on from its
        end, done if it was the last - else None."""
        self.holding = self.holding + dt if ok else 0.0
        if self.holding < REACHED_S:
            return None
        self.holding = 0.0
        end, step = self.marks.pop(0)
        self.i, self.t = min(end, len(self.stream) - 1), 0.0
        self.done = end >= len(self.stream)
        self.begun = dict(self.last)
        if not self.done:
            # On from where the setpoints are, half the way's rate on: from the curve's own
            # keyframe a hip jumped 15-21 deg a pass into the unfold and the sit (2026-10-01).
            span = max(1e-6, self.stream[self.i][2])
            self.knots[self.i] = dict(self.last)
            self.slopes[self.i] = {j: 0.5 * (self.knots[self.i + 1][j] - v) / span
                                   for j, v in self.last.items()}
        return step

    def step(self, dt):
        """{joint: degrees}: where every drive should be now."""
        self.t += dt
        _verb, stage, span, target = self.stream[self.i]
        while self.t >= span and not self.done:
            self.t -= span
            if self.i + 1 >= len(self.stream):
                self.done = True
                break
            self.i += 1
            _verb, stage, span, target = self.stream[self.i]
        self.stage = stage
        if self.done:
            self.last = dict(self.knots[-1])
            return dict(self.last)
        u = min(1.0, self.t / span) if span > 0.0 else 1.0
        a, b, ma, mb = (self.knots[self.i], self.knots[self.i + 1], self.slopes[self.i],
                        self.slopes[self.i + 1])
        h00, h10, h01, h11 = (2 * u ** 3 - 3 * u ** 2 + 1, u ** 3 - 2 * u ** 2 + u,
                              3 * u ** 2 - 2 * u ** 3, u ** 3 - u ** 2)
        self.last = {j: h00 * a[j] + h10 * span * ma[j] + h01 * b[j] + h11 * span * mb[j]
                     for j in a}
        return dict(self.last)

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
