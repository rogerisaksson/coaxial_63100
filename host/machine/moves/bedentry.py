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
#: How a phase ends: its pose reached; her feet planted or its pose reached; her pelvis pulled in, or its pose reached
#: and she still; its pose reached and she lying; its pose reached and she still; at the middle or its pose reached.
#: How a part goes (`anatomy`'s): held, to its goal at its rate, to its goal at its rate but no straighter than it is.
REACHED, FEET, PULLED, LYING, STILLED, MIDDLE = range(6)
HELD, RATE, LEAST = anatomy.HELD, anatomy.RATE, anatomy.LEAST
#: Its numbers and phases as the suite's lisp says them, their reasons there (`defmove bed-entry`, moves.lisp): her
#: feet's share planted; her pelvis pulled in (m); still under her speed (m/s); lying past her pitch (deg); a stroke
#: again while this short of the middle (m), at most STROKES; a goal reached within TOL deg.
_MOVE = anatomy.move('bed-entry')
PLANT, PULL_IN, STILL, LAIN, NEAR, STROKES, TOL = (_MOVE['constants'][k] for k in (
    'plant', 'pull-in', 'still', 'lain', 'near', 'strokes', 'tol'))
#: How the lisp says a phase ends, a part goes.
_ENDED = {':reached': REACHED, ':feet': FEET, ':pulled': PULLED, ':lying': LYING, ':stilled': STILLED,
          ':middle': MIDDLE}


def _goals_of(p):
    """A phase's {part: (deg, deg/s[, how])}: its shared goals (`:with`) then its own, the lisp's hyphens underscores."""
    goals = {part: goal for shared in p.get('with', ()) for part, goal in shared.items()} | p.get('goals', {})
    return {part.replace('-', '_'): tuple(LEAST if v == ':least' else v for v in goal) for part, goal in goals.items()}


#: The phases: (name, how it ends, the next, the next while a stroke is due, {part: (deg, deg/s[, how])}).
PHASES = tuple((p['name'].lstrip(':'), _ENDED[p['ends']], p['next'].lstrip(':'), p['again'].lstrip(':'), _goals_of(p))
               for p in _MOVE['phases'])
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
