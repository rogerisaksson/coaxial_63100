"""Her way out of a bed over its end: sat up on her arms, scooted to the edge, seated there.

A move her joints play as a program does (`machine.process.joints`), its phases told by what she senses, not by
the clock alone: lain along the bed, her elbows are set under her shoulders and her hips fold as her arms lift her
(held straight, the arms lifted her as a plank); her hands set on the bed by her hips; then strokes like an
earthworm's - her straight arms push back as her knees pull in, her legs stretch, her arms come forward on the bed -
her buttocks dragging, her hips holding her trunk TRUNK deg back of upright (held by her palms' share of her weight
instead, a push that lifted her hands opened her hips and she rolled onto her back with her legs in the air). Once
her shanks bear nothing as her legs stretch her knees are past the edge (on the foam they bear 40-100 N): her knees
fold to TUCK and her ankles to 0 at the push's own rates as her arms push, her buttocks after her feet (yielding,
drawn from where they were, her drives' damping undone, her shanks whipped down, 35 of her weight on her shins;
told to tuck to 110 deg against her planted feet her knees stalled at 85-94, her soles and the bed squeezing 1.5 and
1.7 of her; drawn freely her feet slid under the mattress onto the bed's frame, which bore 300 N on a foot as she
stood); seated once her knees fold past KNEE_SEAT on her soles, the push ending there (told at its end alone, a push
dragged her 0.21 m on, her hips 0.06 m in off the lodge bed's end and 0.15 behind her ankles, and her thrust threw
her forward off her feet; her soles' share alone told nothing: 0.59 at a push's end lifted 0.26 m back, 0.48 sliding
0.24 back), her heels on the frame's face. There they go on SETTLE_S s, her heels sliding down its face to the floor
(left 4 cm up it, her ankles 0.13 m up, not 0.10, her scoot stood her on her toes, her hips 0.00-0.08 m behind her
ankles, and her seat's upright trunk threw her back up the bed). Then she scoots: her trunk SCOOT_TRUNK deg forward
off her buttocks, her arms planted where the push left them, her knees and ankles blended to her seat (SEATED_KNEE,
SEATED_ANKLE), her feet flattening about her heels drag her hips toward them (set out ahead of the frame at 95 and
-20 her hips sat 0.32-0.34 m behind her ankles, where no lean of her trunk, 70 deg, put her over her soles and her
thrust waited; scooted from her heels on the floor, 0.21-0.23). Then seated, her trunk upright, her arms forward: her
law's seat, whose handover stands her up. Its phases are a table (PHASES), every branch a select; its arithmetic is
wbc_bedexit.c's, bit for bit (`tools.cores.bedexit`, what her loop plays).

    state = bedexit.laid(q_deg, t)                                     # from where she lies, at t s
    done = bedexit.step(state, cmd, q_deg, t, dt, soles, shins, pitch)  # cmd (deg, figure.JOINTS) laid over
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body.figure import JOINTS

J = {j: i for i, j in enumerate(JOINTS)}
#: The parts it moves; each joint of theirs (both sides, the spine alone) and its part.
PARTS = ('hip', 'knee', 'ankle', 'shoulder', 'elbow', 'spine')
JOINT = np.array([J[side + part] for part in PARTS[:-1] for side in ('left_', 'right_')] + [J['spine']])
PART = np.array([k for k in range(len(PARTS) - 1) for _side in range(2)] + [len(PARTS) - 1])
KNEES = JOINT[PART == PARTS.index('knee')]
#: How a goal is reached: blended in over its phase from where the phase began, her sensed angle and the goal past
#: it, or drawn toward from her sensed angle by PULL deg at most, yielding past it; else a rate, deg/s.
BLEND, FOLLOW, TOWARD = 0.0, -1.0, -2.0
#: A stroke: the shoulders from SET to PUSH (deg) as the knees pull from STRETCHED to PULLED; its phases' lengths, s.
SET, PUSH, STRETCHED, PULLED, PUSH_S, STRETCH_S, SLIDE_S = -10.0, -40.0, 10.0, 60.0, 0.8, 0.4, 0.6
#: Her trunk's pitch her hips hold stroking, scooting and seated, deg (forward +, her law's `modal.pitched`); their
#: gain, deg/s a deg, and range, deg (scooting they fold past 100: held there she rose and fell back).
TRUNK, SCOOT_TRUNK, SEATED_TRUNK, TRUNK_GAIN, HIP_LO, HIP_HI = -20.0, 15.0, -5.0, 4.0, -130.0, -40.0
#: Past the edge: her shanks' share of her weight under which they hang; her knees drawing her buttocks to her feet
#: (TUCK, deg); a pull's most past where they are (TOWARD), deg; seated: her soles' share and her knees' fold, deg;
#: the strokes at most.
SHIN_SHARE, TUCK, PULL, SOLE_SHARE, KNEE_SEAT, STROKES = 0.02, 105.0, 8.0, 0.25, 100.0, 30.0
#: Seated, her knees and ankles, deg, scooted to on her heels at the frame's face (the low chair's seat, 104 and -20,
#: sits her hips 0.26 m behind her ankles); her legs settling and the scoot, s. Her trunk 20 deg forward stood her on
#: her toes and threw her back onto the bed, 25 slid her off its end; her arms brought forward as she scooted, 3 of 7
#: starts fell.
SEATED_KNEE, SEATED_ANKLE, SETTLE_S, SCOOT_S = 108.0, -28.0, 0.5, 1.5
#: The phases: (name, s, the next, {part: (deg, how)}, past the edge {part: (deg, how)}, the trunk her hips hold or
#: None). The sit-up: the elbows under the shoulders, legs straight; the arms lifting her as the hips fold. Her hands
#: set by her hips. A stroke: the push (to the settle once she is seated, `step`), the legs stretched, the arms
#: forward again. Her legs settling, her heels down the frame's face; the scoot. Her law's seat. Past the edge her
#: knees fold to TUCK and her ankles to 0 at the push's rates.
_TUCKED = {'knee': (TUCK, 120.0), 'ankle': (0.0, 60.0)}
PHASES = (('under', 1.5, 'up', {'hip': (0.0, BLEND), 'knee': (5.0, BLEND), 'shoulder': (-90.0, BLEND),
                                'elbow': (90.0, BLEND)}, {}, None),
          ('up', 1.5, 'place', {'hip': (-75.0, BLEND), 'shoulder': (-120.0, BLEND), 'elbow': (30.0, BLEND)}, {}, None),
          ('place', 1.0, 'push', {'shoulder': (SET, BLEND), 'elbow': (0.0, BLEND)}, {}, TRUNK),
          ('push', PUSH_S, 'stretch', {'shoulder': (PUSH, abs(PUSH - SET) / (PUSH_S - 0.1)), 'knee': (PULLED, 120.0),
                                       'elbow': (0.0, 200.0), 'ankle': (0.0, 60.0)},
           _TUCKED, TRUNK),
          ('stretch', STRETCH_S, 'slide', {'knee': (STRETCHED, 150.0), 'ankle': (0.0, 80.0)}, _TUCKED, TRUNK),
          ('slide', SLIDE_S, 'push', {'shoulder': (SET, abs(PUSH - SET) / (SLIDE_S - 0.1))}, _TUCKED, TRUNK),
          ('settle', SETTLE_S, 'scoot', _TUCKED, {}, TRUNK),
          ('scoot', SCOOT_S, 'seat', {'knee': (SEATED_KNEE, BLEND), 'ankle': (SEATED_ANKLE, BLEND)}, {}, SCOOT_TRUNK),
          ('seat', 1.0, 'done', {'knee': (SEATED_KNEE, BLEND), 'ankle': (SEATED_ANKLE, BLEND), 'shoulder': (25.0, BLEND),
                                 'elbow': (19.0, BLEND), 'spine': (-30.0, BLEND)}, {}, SEATED_TRUNK),
          ('done', math.inf, 'done', {}, {}, None))
NAMES = tuple(phase[0] for phase in PHASES)
#: The push (it counts the strokes and tells her seated), the stretch (it tells her past the edge), the settle (where
#: she goes seated), the end.
PUSHED, STRETCHED_AT, SEATED, DONE = (NAMES.index(name) for name in ('push', 'stretch', 'settle', 'done'))
#: PHASES as flat tables: each phase's length, the next, the trunk her hips hold and whether they do; each part's
#: goal, how it is reached (0 held, 1 blended, 2 at its rate, 3 followed, 4 drawn toward) and its rate or pull, on
#: the bed and past its edge.
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
            _how[_k, _i], _rate[_k, _i] = {BLEND: (1.0, 0.0), FOLLOW: (3.0, 0.0), TOWARD: (4.0, PULL)}.get(_by, (2.0, _by))
#: The state's slots: phase, its time, the strokes, past the edge (0/1), and the pose the phase began from.
PHASE, AT, COUNT, OVER, FROM = 0, 1, 2, 3, 4


def laid(q_deg, t):
    """The move's state from her pose `q_deg` (deg, figure.JOINTS) at `t` s: its first phase, nothing done."""
    state = np.zeros(FROM + len(JOINTS))
    state[AT] = t
    state[FROM:] = q_deg
    return state


def step(state, cmd, q_deg, t, dt, soles, shins, pitch):
    """A pass of the move at `t`, `dt` s on: `cmd` (deg, figure.JOINTS) laid over from `state`, her pose `q_deg` (deg),
    her soles' and shanks' share of her weight and her trunk's pitch (deg) sensed; 1.0 once she sits on the edge."""
    k, el, over = int(state[PHASE]), t - state[AT], state[OVER] > 0.5
    a, x, start = min(el / LENGTH[k], 1.0), cmd[JOINT], state[FROM + JOINT]
    how = np.where(over, HOW_OVER[k, PART], HOW[k, PART])
    goal = np.where(over, GOAL_OVER[k, PART], GOAL[k, PART])
    rate, q = np.where(over, RATE_OVER[k, PART], RATE[k, PART]), q_deg[JOINT]
    reach = rate * dt
    x = np.where(how == 1.0, start + a * (goal - start),
                 np.where(how == 2.0, x + np.clip(goal - x, -reach, reach),
                          np.where(how == 3.0, q + goal, np.where(how == 4.0, q + np.clip(goal - q, 0.0, rate), x))))
    held = np.clip(x + TRUNK_GAIN * (pitch - TRUNKS[k]) * dt, HIP_LO, HIP_HI)
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
