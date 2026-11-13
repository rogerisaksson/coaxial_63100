"""Her moves whose phases end by what she senses, each a table of the suite's lisp (`defmove`).

A move her joints play as a program does (`machine.process.joints`), bound by her text to a stage of hers by its
`:joint-program`, the `defmove` of that name: her way into a bed over its end to its middle (`bed-entry`,
bed_lean.lisp), up onto a seat too high for her feet and down off it (`chair-mount`, `chair-dismount`,
terminal_sequencer.lisp). Each phase drives her
parts to its goals as her anatomy goes (`anatomy`: held, at a rate, at a rate but no straighter than she is) and ends
by what she senses and where a look puts her on the furniture her eyes saw (`furniture.lain`), none by the clock: its
goals reached; her feet planted (her soles' share PLANT) or that; her pelvis pulled in (PULL_IN m from its end), or
that and she still (her speed under STILL); that and she lying (her pitch past LAIN); that and she still; at the
middle or that. The phase after it, or while a stroke is due - more than NEAR short of the middle, STROKES at most, a
phase's `:counts` one each - its `:again`. Her text's rows wait while it plays as its table says (`:holds`). A number
no end of its table reads it need not say; one an end reads it must (`tables` refuses it). Every branch a select; its
arithmetic is wbc_sensed.c's, bit for bit (`tools.cores.sensed`, what her loop plays).

    move = sensed.move('bed-entry')                                                 # its tables, laid and step
    state = move.laid(q_deg, t)                                                     # from where she sits
    done = move.step(state, cmd, q_deg, t, dt, soles, along, half, pitch, speed)    # cmd (deg) laid over
"""
import collections
import functools
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body.figure import JOINTS
from machine.errors import MachineError
from machine.moves import anatomy

#: What `machine.process.joints` gives a pass beyond her pose: her soles' share of her weight, her pelvis from the
#: middle of the furniture she is on along its length (m) and that half length (m), her trunk's pitch (deg, forward
#: +), her pelvis's speed (m/s) - her place and speed as a look takes her.
SENSED = ('soles', 'along', 'half', 'pitch', 'speed')
#: The parts it moves, the first SIDED a side each - shoulders, elbows, hips, knees, ankles and their rolls, her
#: legs' rolls and yaws -, then her spine, waist and neck; each joint of theirs and its part.
PARTS, SIDED = (
    'shoulder', 'elbow', 'hip', 'knee', 'ankle', 'ankle_roll', 'hip_roll', 'hip_yaw', 'spine', 'waist', 'neck'), 8
JOINT = np.array([JOINTS.index(side + p) for p in PARTS[:SIDED] for side in ('left_', 'right_')]
                 + [JOINTS.index(p) for p in PARTS[SIDED:]])
PART = np.repeat(np.arange(len(PARTS)), [2] * SIDED + [1] * (len(PARTS) - SIDED))
#: How a phase ends: its goals reached; her feet planted or its goals reached; her pelvis pulled in, or its goals
#: reached and she still; its goals reached and she lying; its goals reached and she still; at the middle or its goals
#: reached - each the numbers it reads besides a goal's reach (READS). How a part goes (`anatomy`'s): held, to its goal
#: at its rate, to its goal at its rate but no straighter than it is.
REACHED, FEET, PULLED, LYING, STILLED, MIDDLE = range(6)
READS = ((), ('plant',), ('pull-in', 'still'), ('lain',), ('still',), ())
HELD, RATE, LEAST = anatomy.HELD, anatomy.RATE, anatomy.LEAST
#: How the lisp says a phase ends.
_ENDED = {':reached': REACHED, ':feet': FEET, ':pulled': PULLED, ':lying': LYING, ':stilled': STILLED,
          ':middle': MIDDLE}
#: A move's numbers as its lisp names them: her feet's share planted; her pelvis pulled in (m); still under her speed
#: (m/s); lying past her pitch (deg); a stroke again while this short of the middle (m), at most this many; a goal
#: reached within this many deg - one no end of its table reads laid 0.
NUMBERS = ('plant', 'pull-in', 'still', 'lain', 'near', 'strokes', 'tol')
#: A move's tables (`tables`): each phase's name, how it ends, the next and the next while a stroke is due, whether it
#: counts a stroke; each part's goal, rate and how it goes; the end; its NUMBERS; whether her text's rows wait.
Tables = collections.namedtuple('Tables', 'names over next again counts goal rates how done numbers holds')
#: The state's slots: the phase, the strokes, her side of the furniture's middle (0 till her first pass).
PHASE, COUNT, WAY = 0, 1, 2


def _goals_of(p):
    """A phase's {part: (deg, deg/s[, how])}: its shared goals (`:with`) then its own, the lisp's hyphens underscores."""
    goals = {part: goal for shared in p.get('with', ()) for part, goal in shared.items()} | p.get('goals', {})
    return {part.replace('-', '_'): tuple(LEAST if v == ':least' else v for v in goal) for part, goal in goals.items()}


@functools.lru_cache(maxsize=None)
def tables(name):
    """Move `name`'s Tables as the suite's lisp says them (`defmove`, moves.lisp): a phase's next while a stroke is due
    its next where it says none; refused where an end of it reads a number it does not say."""
    told = anatomy.move(name)
    phases = [(p['name'].lstrip(':'), _ENDED[p['ends']], p['next'].lstrip(':'), p.get('again', p['next']).lstrip(':'),
               float(bool(p.get('counts', False))), _goals_of(p)) for p in told['phases']]
    names, ends, nexts, agains, counts, goals = zip(*phases)
    said = told.get('constants', {})
    stroked = agains != nexts or any(counts)
    unsaid = sorted(({'tol'} | {n for e in ends for n in READS[e]} | ({'near', 'strokes'} if stroked else set()))
                    - set(said))
    if unsaid:
        raise MachineError('%s: its ends read %s, which it does not say' % (name, ', '.join(unsaid)))
    goal, rates, how = (np.zeros((len(phases), len(PARTS))) for _table in range(3))
    for k, part_goals in enumerate(goals):
        for part, (deg, rate, *how_) in part_goals.items():
            i = PARTS.index(part)
            goal[k, i], rates[k, i], how[k, i] = deg, rate, (how_ or [RATE])[0]
    after, again = (np.array([float(names.index(n)) for n in ns]) for ns in (nexts, agains))
    return Tables(names, np.array(ends), after, again, np.array(counts), goal, rates, how, names.index('done'),
                  tuple(said.get(n, 0.0) for n in NUMBERS), bool(told.get('holds', True)))


def laid(tb, q_deg, t):
    """Move `tb`'s state (its Tables) as it begins, from her pose `q_deg` (deg) at `t` s: its first phase, no stroke,
    no side yet."""
    return np.zeros(WAY + 1)


def step(tb, state, cmd, q_deg, t, dt, soles, along, half, pitch, speed):
    """A pass of move `tb` (its Tables) `dt` s on: `cmd` (deg, figure.JOINTS) laid over from `state`, her soles' share
    of her weight, her pelvis along the furniture from its middle and its half length (m), her trunk's pitch (deg) and
    her speed (m/s) sensed (`SENSED`); 1.0 once at its last phase."""
    plant, pull_in, still_at, lain, near, strokes, tol = tb.numbers
    k = int(state[PHASE])
    way = state[WAY] if state[WAY] != 0.0 else (-1.0 if along < 0.0 else 1.0)
    remaining = way * along
    x, how = cmd[JOINT], tb.how[k, PART]
    goal = anatomy.aimed(how, tb.goal[k, PART], x)
    cmd[JOINT] = anatomy.driven(how, goal, tb.rates[k, PART], x, x, x, 0.0, dt)
    reached = bool(np.all((np.abs(goal - cmd[JOINT]) <= tol) | (how == HELD)))
    still = speed <= still_at
    due = (reached, reached or soles >= plant, half - remaining >= pull_in or (reached and still),
           reached and pitch <= lain, reached and still, reached or remaining <= 0.0)[tb.over[k]]
    state[WAY] = way
    state[COUNT] = state[COUNT] + tb.counts[k] * float(due)
    again = remaining > near and state[COUNT] < strokes
    state[PHASE] = (tb.again[k] if again else tb.next[k]) if due else float(k)
    return float(state[PHASE] >= tb.done)


class Move:

    """Move `name` as her loop plays one (`joints.played`): its senses, whether her text's rows wait while it plays,
    its tables (`tables`), its `laid` and `step`."""

    def __init__(self, name):
        self.name, self.tables = name, tables(name)
        self.SENSED, self.HOLDS = SENSED, self.tables.holds

    def laid(self, q_deg, t):
        return laid(self.tables, q_deg, t)

    def step(self, state, cmd, q_deg, t, dt, soles, along, half, pitch, speed):
        return step(self.tables, state, cmd, q_deg, t, dt, soles, along, half, pitch, speed)


@functools.lru_cache(maxsize=None)
def move(name):
    """Move `name` (`Move`): the `defmove` of that name in the suite's lisp."""
    return Move(name)
