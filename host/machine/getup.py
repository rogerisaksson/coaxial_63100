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
from machine.curves import eased

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
          ('ease', 'unfold', 0.6, {}))
#: Face down, the kneel: the knees drawn under her hips, the chest down; sat back on her heels,
#: upright over her toes; onto her feet - the arms thrown on and the trunk folded over the knees,
#: up on the tucked toes, the heels down into a squat on straight arms, then the arrival's own -
#: ended by the observer as she crouches still and flat-footed over her feet (`observer.EARLY`).
#: From CMA-ESs over the states real falls left her in, scored by the observer through the
#: arrival: back on her heels from 18 of 22 kneels, walking off from 11 of 12 (2026-10-01); lifted
#: by the floor-up plan instead she stood in none of 224, the knees never off the floor.
KNEES_UNDER = (('ease', 'prop', 1.19, _pose(3.0, 52.5, 24.6, -7.9, -20.0, 131.1, 84.9)),
               ('ease', 'prop', 1.46, _pose(-87.8, 96.8, 14.8, 22.9, -20.0, 163.1, 114.4)))
SIT_BACK = (('ease', 'sit', 0.3, _pose(-163.2, 158.2, 12.2, 60.0, -20.0, 54.3, 12.0)),
            ('ease', 'sit', 3.08, dict(_pose(-21.9, 160.0, -46.8, 30.9, -20.0, 72.7, 65.2),
                                       left_foot=54.5, right_foot=54.5)))
ONTO_FEET = (('ease', 'lift', 0.47, dict(_pose(-24.2, 145.7, -58.3, 33.4, -20.0, 44.5, -5.0),
                                         left_foot=60.6, right_foot=60.6)),
             ('ease', 'lift', 0.49, dict(_pose(-144.3, 123.0, -47.9, 35.9, -20.0, 120.7, -5.0),
                                         left_foot=51.0, right_foot=51.0)),
             ('ease', 'crouch', 1.46, dict(_pose(-150.0, 133.6, -26.3, 41.2, -20.0, 76.6, -5.0),
                                           left_foot=3.9, right_foot=3.9)),
             ('ease', 'crouch', 2.05, arrival.angles_of(arrival._squat())),
             ('ease', 'crouch', 1.0, {}))


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


#: From her back onto her front: the left knee drawn up and across, both arms up over her head,
#: the waist turned back, then the knee down and the left arm reaching on. From a CMA-ES from her
#: back scored by the observer: her face 0.75 down at its end (2026-09-30); the roll that had
#: put her on her back, run from it, left her there, face up 0.98, 15 falls in 20.
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
    if step == 'roll onto front':
        return FRONTS_BY[1 if now['left_up'] >= 0.0 else -1]
    return {'straighten out': UNFOLD, 'knees under': KNEES_UNDER,
            'sit back on heels': SIT_BACK,
            'onto feet': ONTO_FEET}[step]


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
            self.begun = dict(self.last)
            _verb, stage, span, target = self.stream[self.i]
        self.stage = stage
        k = eased(self.t / span) if span > 0.0 else 1.0
        self.last = {j: v + (target.get(j, v) - v) * k for j, v in self.begun.items()}
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
