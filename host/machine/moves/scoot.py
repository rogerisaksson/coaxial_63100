"""Her scoot on the floor as her twist tells it: a crab on her seat, heels in turn, then a stroke.

A move her joints play as a program does (`machine.process.joints`), bound by her text to her floor rest once she sits
(`clocked`) and begun again by a twist while she sits on the floor (`joints.scooted`, `g['sat']`). Its gait is her
text's, the kernel's seated butt-scoot (`degraded_scoot_and_crawl_*.lisp`, the segment `clocked` sits her by): its
clock's phases order her limbs (her heels in turn, then her fists together), its stroke's length is her knees' sweep
from the take's fold (`clocked.chord`), its beat and limb duty the stroke's rate. No clock times a phase: each is told
by what she senses and by her twist (her text's 32 clocked strokes waited on fists that never bore, and she sat still
from the third). A cycle: both feet lift to their strokes' starts and she sits on her seat and fists; each heel sets in
its turn, her seat bearing her (stood on her fists and feet, a lifted limb left her three points and she tipped: S, A,
D 5 of 5 from one sit); her arms reach back to her seat's brace; her hips open her off her seat onto her fists and feet
where both heels push her back (sat, a push slid her feet out: S +0.01 m in 8 s), or she pulls herself along on her
seat where they pull (vaulted, W went 0.55-0.72 m in 8 s, sat 1.05-1.60); her stroke; her hips lean her trunk back to
its lean. Her legs' rolls and yaws and her trunk's roll and twist stay square (splayed, she spun 37-99 deg a back
stroke run and fell). Her twist let go, she sits where her last heel set. Its phases are a table (PHASES), every
branch a select; its arithmetic is wbc_scoot.c's, bit for bit (`tools.cores.scoot`, what her loop plays).

    state = scoot.laid(q_deg, t)                                                           # from where she sits
    scoot.step(state, cmd, q_deg, t, dt, pitch, hands, left, right, back, turn, heading)  # cmd laid over; 0.0
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body import figure
from machine.body.figure import JOINTS
from machine.errors import MachineError
from machine.moves import anatomy, clocked, seated

#: What `machine.process.joints` gives a pass beyond her pose: her trunk's pitch (deg, forward +), her fists' and each
#: foot's share of her weight, her twist back and turning left as shares of her keys' (1 a key held, -1 its opposite),
#: her pelvis's heading (deg, left +).
SENSED = ('pitch', 'hands', 'left_foot', 'right_foot', 'back', 'turn', 'heading')
#: The parts it moves, a side each (left, right) - her hips, knees, shoulders, elbows, her legs' rolls and yaws and her
#: ankles -, then her trunk's roll and twist.
PAIRED, SINGLE = ('hip', 'knee', 'shoulder', 'elbow', 'hip_roll', 'hip_yaw', 'ankle_roll', 'ankle'), ('spine_roll', 'waist')
PARTS = PAIRED + SINGLE
JOINT = np.array([JOINTS.index(side + p) for p in PAIRED for side in ('left_', 'right_')] + [JOINTS.index(p) for p in SINGLE])
PART = np.array([k for k in range(len(PAIRED)) for _side in range(2)] + [len(PAIRED) + k for k in range(len(SINGLE))])
SIDE = np.array([1.0, -1.0] * len(PAIRED) + [0.0] * len(SINGLE))
#: The side each joint is on: 0 her left (and her trunk's), 1 her right; each one's other side (her trunk's its own).
SIDED = (SIDE < 0.0).astype(int)
PARTNER = np.array([i ^ 1 for i in range(2 * len(PAIRED))] + [2 * len(PAIRED) + k for k in range(len(SINGLE))])
HIP, KNEE = PART == 0, PART == 1
#: Her gait as her text lays it (`clocked`): its clock's phases [LH LF RH RF axial] (hind her heels, fore her fists),
#: of a turn, and a limb's stance on it, s, and its stroke, m (`clocked.turns`). Her heels step in their phases' order;
#: her fists reach together after them (her two-joint arms shove her round as one steps alone: a fist set turned her
#: 32 deg, and A and W+A tipped her at 22-44 deg).
PHASED = clocked.phases('scoot_floor')
STANCE_S, STROKE_M = clocked.turns('scoot_floor')[2:]
if PHASED[1] != PHASED[3] or PHASED[1] <= max(PHASED[0], PHASED[2]):
    raise MachineError('refused: her fists reach together after her heels, not at %s' % (PHASED[:4],))
#: Each step's side (her heel's, in its phase's order) and the joints it moves (its hip and knee: 1 a joint it moves).
ORDER = np.argsort(np.array([PHASED[0], PHASED[2]]), kind='stable')
STEPS = np.array([[float(SIDED[i] == side and PART[i] < 2) for i in range(len(JOINT))] for side in ORDER])
#: The parts a step moves: a hip or knee its heel's is held while it steps.
LIMBED = PART < 2


def _bent(chord):
    """The knee, deg, that sets her hip joints `chord` m behind her ankle sat on the floor (`clocked.chord`'s)."""
    up = clocked.SAT - figure.ANKLE_H
    c = (chord * chord + up * up - figure.THIGH ** 2 - figure.SHANK ** 2) / (2.0 * figure.THIGH * figure.SHANK)
    return math.degrees(math.acos(min(max(c, -1.0), 1.0)))


#: Its numbers and phases as the suite's lisp says them, their reasons there (`defmove scoot`, moves.lisp): a stroke's
#: knees folded (deg), a hip's lift and set, her trunk within LEANT of its lean, her braced elbows, the trunk her hips
#: hold with their gain and range, her vaulted shoulders, the rates (deg/s), what bears and touches, a twist's ask and
#: a turn's shares.
_MOVE = anatomy.move('scoot')
(FOLD, FOLDED, DOWN, LEANT, LOCKED, TRUNK, GAIN, HIP_LO, HIP_HI, VAULTED, AIR_RATE, BRACE_RATE, LIFT_RATE, SET_RATE,
 SQUARE_RATE, EVEN_RATE, BEAR, SKIM, ASK, TOL, STROKE_TURN, TURN, HALF) = (_MOVE['constants'][k] for k in (
    'fold', 'folded', 'down', 'leant', 'locked', 'trunk', 'gain', 'hip-lo', 'hip-hi', 'vaulted', 'air-rate',
    'brace-rate', 'lift-rate', 'set-rate', 'square-rate', 'even-rate', 'bear', 'skim', 'ask', 'tol', 'stroke-turn',
    'turn', 'half'))
#: What her body sets: a stroke's knees stretched where her hips have gone the stroke's length; her shoulders sat, her
#: fists behind her braced at her lean (`seated.braced`); a stroke's knees' rate at a key held, its sweep in her text's
#: stance; her aim a turn's LEAD ahead as she turns.
STRETCH = _bent(clocked.chord(math.radians(FOLD)) + STROKE_M)
SEAT = math.degrees(seated.braced(math.radians(TRUNK)))
STROKE_RATE = (FOLD - STRETCH) / STANCE_S
LEAD = STROKE_TURN * TURN
#: How a part reaches its goal: 0 held, 1 its phase's goal at its rate, 2 the stroke's end at her way's rate, 3 the
#: stroke's start (a hip by its knee's way), 4 its phase's goal while its foot touches none, 5 its pair's mean at its
#: rate, a hip holding her trunk as well (her stance square as she sits), 6 its phase's goal while her trunk is short
#: of LEANT back of its lean.
HELD, RATE, END, START, SETTING, EVEN, UNSEAT = range(7)
#: When a phase is over: its goals reached; its goals reached, her fists touching and her trunk at its lean; its heel
#: touching or its goals reached; her knees at their ends or her twist let go; her twist asking; her trunk LEANT back
#: of its lean; her trunk within LEANT of it.
REACHED, BRACED, SET, STROKED, ASKING, OFF, LEANED = range(7)
#: How the lisp says a part goes, a phase ends, and the goal her body sets.
_HOWS = {':rate': RATE, ':end': END, ':start': START, ':setting': SETTING, ':even': EVEN, ':unseat': UNSEAT}
_ENDED = {':reached': REACHED, ':braced': BRACED, ':set': SET, ':stroked': STROKED, ':asking': ASKING, ':off': OFF,
          ':leaned': LEANED}
_BODY = {':seat': SEAT}


def _goals_of(p):
    """A phase's {part: (how, goal, rate)}: its shared goals (`:with`) then its own, the lisp's hyphens underscores."""
    goals = {part: goal for shared in p.get('with', ()) for part, goal in shared.items()} | p.get('goals', {})
    return {part.replace('-', '_'): (_HOWS[how], _BODY[goal] if isinstance(goal, str) else goal, rate)
            for part, (how, goal, rate) in goals.items()}


#: The phases: (name, {part: (how, goal, rate)}, over when, the next asked, idle).
PHASES = tuple((p['name'].lstrip(':'), _goals_of(p), _ENDED[p['ends']], p['asked'].lstrip(':'), p['idle'].lstrip(':'))
               for p in _MOVE['phases'])
NAMED = tuple(phase[0] for phase in PHASES)
SITTING, SETTING_DOWN, REACHING, PUSHING, HOLDING = (NAMED.index(n) for n in ('sit', 'set', 'reach', 'push', 'hold'))
#: PHASES as flat tables: each part's how, goal and rate a phase; when it is over; the next asked and idle; whether a
#: step's heel alone moves; whether her twist's ways are taken as it ends (a cycle begun); whether her hips hold her
#: trunk (not as she sits, steps or reaches: her fists and her seat hold it there; nor pulling on her seat).
HOW, GOAL, RATES = (np.zeros((len(PHASES), len(PARTS))) for _table in range(3))
for _k, _phase in enumerate(PHASES):
    for _part, (_how, _goal, _rate) in _phase[1].items():
        HOW[_k, PARTS.index(_part)], GOAL[_k, PARTS.index(_part)], RATES[_k, PARTS.index(_part)] = _how, _goal, _rate
OVER = np.array([phase[2] for phase in PHASES])
ASKED, IDLE = (np.array([float(NAMED.index(phase[c])) for phase in PHASES]) for c in (3, 4))
MASKED = np.array([float(n in ('lift', 'set')) for n in NAMED])
CYCLES = np.array([float(n in ('brace', 'hold', 'lean')) for n in NAMED])
TRUNKED = np.array([float(n not in ('sit', 'lift', 'set', 'reach')) for n in NAMED])
#: The state's slots: the phase, the step (of ORDER), each side's way this cycle (+ its knee stretches, - it folds, 0
#: still), the heading she aims at (deg) and whether she has one.
PHASE, STEP, WAY, AIM, AIMED = 0, 1, 2, 4, 5
#: Her sides, left then right, as her turn shares them (left +).
SIDES = np.array([1.0, -1.0])


def laid(q_deg, t):
    """The move's state from her pose `q_deg` (deg, figure.JOINTS) at `t` s: bracing, her first step next, no way, no
    aim yet."""
    return np.zeros(AIMED + 1)


def step(state, cmd, q_deg, t, dt, pitch, hands, left, right, back, turn, heading):
    """A pass of the move `dt` s on: `cmd` (deg, figure.JOINTS) laid over from `state`, her trunk's pitch (deg), her
    fists' and feet's shares of her weight, her twist and her heading (`SENSED`) sensed; 0.0, the move never over by
    itself (a word of hers ends it)."""
    k, n = int(state[PHASE]), int(state[STEP])
    x = cmd[JOINT]
    way = state[WAY + SIDED]
    pushed = state[WAY] > 0.0 and state[WAY + 1] > 0.0
    up = pitch <= TRUNK - LEANT
    how, rate = HOW[k, PART], RATES[k, PART]
    how = np.where((MASKED[k] > 0.5) & LIMBED & (STEPS[n] < 0.5), HELD, how)
    stretched, folding = way > 0.0, way < 0.0
    start = np.where(stretched, FOLD, np.where(folding, STRETCH, x))
    end = start + (np.where(stretched, STRETCH, np.where(folding, FOLD, x)) - start) * np.abs(way)
    begin = np.where(KNEE, start, np.where(way != 0.0, FOLDED, x))
    goal = np.where(how == EVEN, 0.5 * (x + x[PARTNER]), np.where(
        (how == RATE) | (how == SETTING) | (how == UNSEAT), GOAL[k, PART], np.where(how == END, end, np.where(
            how == START, begin, x))))
    foot = np.where(SIDED == 0, left, right)
    rate = np.where(how == END, STROKE_RATE * np.abs(way), np.where(HIP & (how == START), rate * (
        stretched | (foot >= SKIM)), np.where(how == SETTING, rate * (foot < SKIM), np.where(
            how == UNSEAT, rate * float(not up), rate))))
    moved = anatomy.driven(np.where(how == HELD, anatomy.HELD, anatomy.RATE), goal, rate, x, x, x, 0.0, dt)
    held = anatomy.held(moved, GAIN, pitch, TRUNK, dt, HIP_LO, HIP_HI)
    trunked = TRUNKED[k] > 0.5 and (pushed or k != PUSHING) and (left >= BEAR or right >= BEAR)
    cmd[JOINT] = np.where(HIP & ((how == HELD) | (how == EVEN) | (rate == 0.0)) & trunked, held, moved)
    near = np.abs(goal - cmd[JOINT]) <= TOL
    ok = np.where(how == EVEN, np.abs(x - x[PARTNER]) <= TOL, near | (how == HELD) | (HIP & (rate == 0.0)))
    reached = bool(np.all(ok))
    stroked = bool(np.all(near | (rate == 0.0) | ~KNEE))
    asked = abs(back) + abs(turn) > ASK
    turning = abs(turn) > ASK
    aim = heading + LEAD * (float(turn > 0.0) - float(turn < 0.0)) if turning else (
        state[AIM] if asked and state[AIMED] > 0.5 else heading)
    state[AIM], state[AIMED] = aim, 1.0
    off = aim - heading
    off = off - 360.0 * np.floor((off + 180.0) / 360.0)
    share = np.clip(off / STROKE_TURN, -TURN, TURN)
    want = np.clip(back + SIDES * share, -1.0, 1.0)
    want = np.where(back > ASK, np.maximum(want, HALF * back), np.minimum(want, 0.0))
    braced = reached and hands >= SKIM and abs(pitch - TRUNK) <= LEANT
    over = (reached, braced, (left, right)[ORDER[n]] >= SKIM or reached, stroked or not asked, asked, up,
            abs(pitch - TRUNK) <= LEANT)[OVER[k]]
    last = n == len(ORDER) - 1
    nxt = ASKED[k] if asked else IDLE[k]
    nxt = (float(REACHING) if asked else float(HOLDING)) if k == SETTING_DOWN and last else nxt
    nxt = float(PUSHING) if k == REACHING and asked and not pushed else nxt
    state[WAY:WAY + 2] = want if over and CYCLES[k] > 0.5 and nxt == SITTING and asked else state[WAY:WAY + 2]
    state[STEP] = (n + 1.0) * float(not last) if over and k == SETTING_DOWN else float(n)
    state[PHASE] = nxt if over else k
    return 0.0
