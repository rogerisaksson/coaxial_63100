"""The robot's way out of a bed over its end: sat up on its arms, scooted to the edge, seated there.

A move the robot's joints play as a program does (`machine.process.joints`), its phases told by what the robot
senses, not by the clock alone: lain along the bed, its elbows are set under its shoulders and the
robot's hips fold as its arms lift it (held straight, the arms lifted it as a plank); the
robot's hands set on the bed by its hips; then strokes like an earthworm's - its straight arms push
back as its knees pull in, its legs stretch, its arms come forward on the bed - the robot's
buttocks dragging, its hips holding its trunk TRUNK deg back of upright (held by its palms'
share of its weight instead, a push that lifted its hands opened its hips and the robot
rolled onto its back with its legs in the air). Once the robot's shanks bear nothing as the robot's
legs stretch its knees are past the edge (on the foam they bear 40-100 N): its knees fold to TUCK and
the robot's ankles to 0 at the push's own rates as its arms push, its buttocks after its feet
(yielding, drawn from where they were, its drives' damping undone, its shanks whipped down, 35 of the
robot's weight on its shins; told to tuck to 110 deg against its planted feet its knees
stalled at 85-94, its soles and the bed squeezing 1.5 and 1.7 of it; drawn freely its feet
slid under the mattress onto the bed's frame, which bore 300 N on a foot as it stood); seated once the robot's
knees fold past KNEE_SEAT on its soles, the push ending there (told at its end alone, a push dragged the
robot's 0.21 m on, its hips 0.06 m in off the lodge bed's end and 0.15 behind its ankles, and the
robot's thrust threw it forward off its feet; its soles' share alone told nothing: 0.59 at a
push's end lifted 0.26 m back, 0.48 sliding 0.24 back), its heels on the frame's face. There they go on
SETTLE_S s, the robot's heels sliding down its face to the floor (left 4 cm up it, the robot's ankles 0.13 m up, not
0.10, its scoot stood it on its toes, its hips 0.00-0.08 m behind its ankles,
and its seat's upright trunk threw it back up the bed). Then the robot scoots: its trunk SCOOT_TRUNK
deg forward off its buttocks, its arms planted where the push left them, its knees and ankles blended to its seat
(SEATED_KNEE, SEATED_ANKLE), its feet flattening about its heels drag its hips toward them (set out ahead of the
frame at 95 and -20 its hips sat 0.32-0.34 m behind its ankles, where no lean of its trunk, 70 deg, put it over its
soles and its thrust waited; scooted from its heels on the floor, 0.21-0.23). Then seated, the robot's trunk upright,
its arms forward: its law's seat, whose handover stands it up. The move's phases are a table (PHASES), every branch a
select; its arithmetic is wbc_bedexit.c's, bit for bit (`tools.cores.bedexit`, what the robot's loop plays).

    state = bedexit.laid(q_deg, t)                                     # from where the robot lies, at t s
    done = bedexit.step(state, cmd, q_deg, t, dt, soles, shins, pitch)  # cmd (deg, figure.JOINTS) laid over
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body.figure import JOINTS
from machine.moves import anatomy

J = {j: i for i, j in enumerate(JOINTS)}
#: The parts it moves; each joint of theirs (both sides, the spine alone) and its part.
PARTS = ('hip', 'knee', 'ankle', 'shoulder', 'elbow', 'spine')
JOINT = np.array([J[side + part] for part in PARTS[:-1] for side in ('left_', 'right_')] + [J['spine']])
PART = np.array([k for k in range(len(PARTS) - 1) for _side in range(2)] + [len(PARTS) - 1])
KNEES = JOINT[PART == PARTS.index('knee')]
#: How a goal is reached: blended in over its phase from where the phase began, the robot's sensed angle and the
#: goal past it, or drawn toward from the robot's sensed angle by PULL deg at most, yielding past it; else a rate,
#: deg/s.
BLEND, FOLLOW, TOWARD = 0.0, -1.0, -2.0
#: The move's numbers and phases as the suite's lisp says them, their reasons there (`defmove bed-exit`,
#: moves.lisp): the robot's hips' gain on its trunk (deg/s a deg) and range (deg); past the edge the
#: robot's shanks' share under which they hang, a pull's most past where they are (TOWARD, deg); seated, the
#: robot's soles' share and its knees' fold (deg); the strokes.
_MOVE = anatomy.move('bed-exit')
TRUNK_GAIN, HIP_LO, HIP_HI, SHIN_SHARE, PULL, SOLE_SHARE, KNEE_SEAT, STROKES = (_MOVE['constants'][k] for k in (
    'trunk-gain', 'hip-lo', 'hip-hi', 'shin-share', 'pull', 'sole-share', 'knee-seat', 'strokes'))
#: How the lisp says a goal is reached: BLEND .. TOWARD, a number its rate.
_BY = {':blend': BLEND, ':follow': FOLLOW, ':toward': TOWARD}
#: The phases: (name, s, the next, {part: (deg, how)}, past the edge {part: (deg, how)}, the trunk the robot's hips
#: hold or None); no seconds, never by the clock.
PHASES = tuple((p['name'].lstrip(':'), p.get('seconds', math.inf), p['next'].lstrip(':'),
                {part: (deg, _BY[by] if isinstance(by, str) else by) for part, (deg, by) in p.get('goals', {}).items()},
                {part: (deg, _BY[by] if isinstance(by, str) else by) for part, (deg, by) in p.get('over', {}).items()},
                p.get('trunk'))
               for p in _MOVE['phases'])
NAMES = tuple(phase[0] for phase in PHASES)
#: The push (it counts the strokes and tells the robot seated), the stretch (it tells the robot past the edge), the
#: settle (where it goes seated), the end.
PUSHED, STRETCHED_AT, SEATED, DONE = (NAMES.index(name) for name in ('push', 'stretch', 'settle', 'done'))
#: PHASES as flat tables: each phase's length, the next, the trunk the robot's hips hold and whether they do;
#: each part's goal, how it is reached (`anatomy`'s HELD .. TOWARD) and its rate or pull, on the bed and past its
#: edge.
LENGTH = np.array([phase[1] for phase in PHASES])
NEXT = np.array([NAMES.index(phase[2]) for phase in PHASES], float)
TRUNKS = np.array([0.0 if phase[5] is None else phase[5] for phase in PHASES])
HOLDS = np.array([float(phase[5] is not None) for phase in PHASES])
GOAL, HOW, RATE, GOAL_OVER, HOW_OVER, RATE_OVER = (np.zeros((len(PHASES), len(PARTS))) for _table in range(6))
for _k, _phase in enumerate(PHASES):
    for _goals, _goal, _how, _rate in ((_phase[3], GOAL, HOW, RATE), ({**_phase[3], **_phase[4]}, GOAL_OVER, HOW_OVER,
                                                                         RATE_OVER)):
        for _part, (_deg, _by) in _goals.items():
            _i = PARTS.index(_part)
            _goal[_k, _i] = _deg
            _how[_k, _i], _rate[_k, _i] = {BLEND: (anatomy.BLEND, 0.0), FOLLOW: (anatomy.FOLLOW, 0.0),
                                           TOWARD: (anatomy.TOWARD, PULL)}.get(_by, (anatomy.RATE, _by))
#: The state's slots: phase, its time, the strokes, past the edge (0/1), and the pose the phase began from.
PHASE, AT, COUNT, OVER, FROM = 0, 1, 2, 3, 4


def laid(q_deg, t):
    """The move's state from the robot's pose `q_deg` (deg, figure.JOINTS) at `t` s: its first phase, nothing
    done."""
    state = np.zeros(FROM + len(JOINTS))
    state[AT] = t
    state[FROM:] = q_deg
    return state


def step(state, cmd, q_deg, t, dt, soles, shins, pitch):
    """A pass of the move at `t`, `dt` s on: `cmd` (deg, figure.JOINTS) laid over from `state`, the robot's pose
    `q_deg` (deg), its soles' and shanks' share of its weight and its trunk's pitch (deg) sensed; 1.0 once it sits on
    the edge."""
    k, el, over = int(state[PHASE]), t - state[AT], state[OVER] > 0.5
    a, x, start = min(el / LENGTH[k], 1.0), cmd[JOINT], state[FROM + JOINT]
    how = np.where(over, HOW_OVER[k, PART], HOW[k, PART])
    goal = np.where(over, GOAL_OVER[k, PART], GOAL[k, PART])
    rate, q = np.where(over, RATE_OVER[k, PART], RATE[k, PART]), q_deg[JOINT]
    x = anatomy.driven(how, goal, rate, x, q, start, a, dt)
    held = anatomy.held(x, TRUNK_GAIN, pitch, TRUNKS[k], dt, HIP_LO, HIP_HI)
    cmd[JOINT] = np.where((PART == 0) & (HOLDS[k] > 0.5), held, x)
    sat = state[OVER] * float(soles >= SOLE_SHARE) * float(np.mean(q_deg[KNEES]) >= KNEE_SEAT)
    due = el >= LENGTH[k] or (k == PUSHED and sat > 0.0)
    pushed, stretched = due and k == PUSHED, due and k == STRETCHED_AT
    state[COUNT] += float(pushed)
    state[OVER] = max(state[OVER], float(stretched and shins < SHIN_SHARE))
    seated = sat + float(state[COUNT] >= STROKES)
    state[PHASE], state[AT] = (SEATED if pushed and seated > 0.0 else NEXT[k], t) if due else (k, state[AT])
    state[FROM:] = cmd if due else state[FROM:]
    return float(state[PHASE] >= DONE)
