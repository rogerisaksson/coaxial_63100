"""Her way into a bed over its end to its middle, told by where she lies on it.

A move her joints play as a program does (`machine.process.joints`), bound by her text to her scoot onto the bed's
end (`bed_lean.lisp`); every phase told by what she senses and where a look puts her on the bed her eyes saw
(`furniture.lain`), none by the clock (her text counted the puppet's program's strokes from the depth she saw, 0.16 m
each, and she lay wherever they took her). Her joints go to each phase's pose at the puppet's pace (its beat's change
over its seconds, `joints.PROGRAMS`). Her feet set till they bear PLANT of her weight; her shoulders flexed against
her fists as her hips lift her thighs till her pelvis is PULL_IN m in from the bed's end, or the pull is over and she
is still (her speed under STILL); her trunk down till it lies (her pitch past LAIN); her legs raised,
folded, lowered, shoved and stretched along the bed, her trunk straight, till she is still; then strokes up it - her
heels drawn in, her pelvis bridged, her heels pushed till she is at the middle or the push is over, her pelvis set
down, her knees no straighter, till she is still - while she is more than NEAR short of its middle, STROKES at most;
then straight, her legs square. Her text is not held back while it plays (HOLDS): its flat stage runs under it so her
rest limps her on the foam as it ends (held back, her rise after it was never said). Its phases are a table (PHASES),
every branch a select; its arithmetic is wbc_bedentry.c's, bit for bit (`tools.cores.bedentry`, what her loop plays).

    state = bedentry.laid(q_deg, t)                                                    # from where she sits
    done = bedentry.step(state, cmd, q_deg, t, dt, soles, along, half, pitch, speed)   # cmd (deg) laid over
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body.figure import JOINTS
from machine.moves import anatomy

#: What `machine.process.joints` gives a pass beyond her pose: her soles' share of her weight, her pelvis from the
#: middle of the bed she lies on along its length (m) and that half length (m), her trunk's pitch (deg, forward +),
#: her pelvis's speed (m/s) - her place and speed as a look takes her.
SENSED = ('soles', 'along', 'half', 'pitch', 'speed')
#: Her text's rows go on while it plays (`joints.played`).
HOLDS = False
#: The parts it moves, the first SIDED a side each - shoulders, elbows, hips, knees, ankles, her legs' rolls and
#: yaws -, then her spine, waist and neck; each joint of theirs and its part.
PARTS, SIDED = ('shoulder', 'elbow', 'hip', 'knee', 'ankle', 'hip_roll', 'hip_yaw', 'spine', 'waist', 'neck'), 7
JOINT = np.array([JOINTS.index(side + p) for p in PARTS[:SIDED] for side in ('left_', 'right_')]
                 + [JOINTS.index(p) for p in PARTS[SIDED:]])
PART = np.repeat(np.arange(len(PARTS)), [2] * SIDED + [1] * (len(PARTS) - SIDED))
#: Her feet's share of her weight planted; her pelvis pulled in from the bed's end (m); still under her speed (m/s:
#: lying still 0.000-0.003, her pull 0.3-0.6); lying past her pitch (deg: flat -97 to -102); a stroke
#: again while this short of the middle (m), at most STROKES; a goal reached within TOL deg.
PLANT, PULL_IN, STILL, LAIN, NEAR, STROKES, TOL = 0.05, 0.30, 0.01, -90.0, 0.03, 12.0, 1e-6
#: How a phase ends: its pose reached; her feet planted or its pose reached; her pelvis pulled in, or its pose reached
#: and she still; its pose reached and she lying; its pose reached and she still; at the middle or its pose reached.
#: How a part goes (`anatomy`'s): held, to its goal at its rate, to its goal at its rate but no straighter than it is.
REACHED, FEET, PULLED, LYING, STILLED, MIDDLE = range(6)
HELD, RATE, LEAST = anatomy.HELD, anatomy.RATE, anatomy.LEAST
_TRUNK = {'spine': (0.0, 20.0), 'waist': (0.0, 20.0), 'neck': (0.0, 20.0)}
_SQUARE = {'hip_roll': (0.0, 15.0), 'hip_yaw': (0.0, 15.0)}
#: The phases: (name, how it ends, the next, the next while a stroke is due, {part: (deg, deg/s[, how])}) - the
#: puppet's program's poses (`joints.PROGRAMS`), its heels drawn in from her stretch at its first stroke's pace.
PHASES = (('feet', FEET, 'pull', 'pull', {'knee': (95.0, 9.0), 'ankle': (0.0, 30.0)}),
          ('pull', PULLED, 'lie', 'lie', {'shoulder': (30.0, 83.0), 'elbow': (0.0, 30.0), 'hip': (-75.0, 22.0),
                                          'knee': (30.0, 114.0), 'ankle': (0.0, 46.0)}),
          ('lie', LYING, 'up', 'up', {'shoulder': (0.0, 23.0), 'elbow': (5.0, 4.0), 'hip': (-10.0, 50.0),
                                      'knee': (15.0, 12.0)}),
          ('up', REACHED, 'fold', 'fold', {'hip': (-75.0, 108.0), 'knee': (6.0, 15.0)}),
          ('fold', REACHED, 'lower', 'lower', {'knee': (150.0, 288.0)}),
          ('lower', REACHED, 'shove', 'shove', {'hip': (-60.0, 50.0)}),
          ('shove', REACHED, 'stretch', 'stretch', {'hip': (-35.0, 42.0), 'knee': (100.0, 83.0)}),
          ('stretch', STILLED, 'straight', 'draw', {**_TRUNK, 'knee': (6.0, 313.0), 'hip': (-3.0, 107.0)}),
          ('draw', REACHED, 'bridge', 'bridge', {**_SQUARE, 'hip': (-56.0, 133.0), 'knee': (146.0, 350.0)}),
          ('plant', REACHED, 'bridge', 'bridge', {**_SQUARE, 'hip': (-56.0, 25.0), 'knee': (146.0, 95.0)}),
          ('bridge', REACHED, 'push', 'push', {'hip': (-34.0, 88.0), 'knee': (134.0, 48.0)}),
          ('push', MIDDLE, 'set', 'set', {'hip': (-32.0, 7.0), 'knee': (104.0, 100.0)}),
          ('set', STILLED, 'straight', 'plant', {'hip': (-46.0, 70.0), 'knee': (108.0, 20.0, LEAST)}),
          ('straight', REACHED, 'done', 'done', {**_TRUNK, **_SQUARE, 'knee': (6.0, 255.0), 'hip': (-3.0, 108.0)}),
          ('done', REACHED, 'done', 'done', {}))
#: PHASES as flat tables: each phase's name, how it ends, the next and the next while a stroke is due, whether it
#: counts a stroke; each part's goal, rate and how it goes; the end.
NAMES, _ENDS, _NEXTS, _AGAINS, _GOALS = zip(*PHASES)
OVER = np.array(_ENDS)
NEXT, AGAIN = (np.array([float(NAMES.index(name)) for name in names]) for names in (_NEXTS, _AGAINS))
COUNTS = np.array([float(name == 'set') for name in NAMES])
GOAL, RATES, HOW = (np.zeros((len(PHASES), len(PARTS))) for _table in range(3))
for _k, _goals in enumerate(_GOALS):
    for _part, (_deg, _rate, *_how) in _goals.items():
        _i = PARTS.index(_part)
        GOAL[_k, _i], RATES[_k, _i], HOW[_k, _i] = _deg, _rate, (_how or [RATE])[0]
DONE = NAMES.index('done')
#: The state's slots: the phase, the strokes, her side of the bed's middle (0 till her first pass).
PHASE, COUNT, WAY = 0, 1, 2


def laid(q_deg, t):
    """The move's state as it begins, from her pose `q_deg` (deg) at `t` s: her feet first, no stroke, no side yet."""
    return np.zeros(WAY + 1)


def step(state, cmd, q_deg, t, dt, soles, along, half, pitch, speed):
    """A pass of the move `dt` s on: `cmd` (deg, figure.JOINTS) laid over from `state`, her soles' share of her weight,
    her pelvis along the bed from its middle and its half length (m), her trunk's pitch (deg) and her speed (m/s)
    sensed (`SENSED`); 1.0 once she lies straight."""
    k = int(state[PHASE])
    way = state[WAY] if state[WAY] != 0.0 else (-1.0 if along < 0.0 else 1.0)
    remaining = way * along
    x, how = cmd[JOINT], HOW[k, PART]
    goal = anatomy.aimed(how, GOAL[k, PART], x)
    cmd[JOINT] = anatomy.driven(how, goal, RATES[k, PART], x, x, x, 0.0, dt)
    reached = bool(np.all((np.abs(goal - cmd[JOINT]) <= TOL) | (how == HELD)))
    still = speed <= STILL
    due = (reached, reached or soles >= PLANT, half - remaining >= PULL_IN or (reached and still),
           reached and pitch <= LAIN, reached and still, reached or remaining <= 0.0)[OVER[k]]
    state[WAY] = way
    state[COUNT] = state[COUNT] + COUNTS[k] * float(due)
    again = remaining > NEAR and state[COUNT] < STROKES
    state[PHASE] = (AGAIN[k] if again else NEXT[k]) if due else float(k)
    return float(state[PHASE] >= DONE)
