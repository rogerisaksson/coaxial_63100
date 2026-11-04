"""Her scoot on the floor as her twist tells it: sat on her fists, her heels stroking her along.

A move her joints play as a program does (`machine.process.joints`), bound by her text to her floor rest once she sits
(`clocked`) and begun again by a twist while she sits on the floor (`joints.scooted`, `g['sat']`); its phases told by
what she senses and by her twist, never by the clock (her text's 32 clocked strokes waited on fists that never bore,
and she sat still from the third). Her arms straighten onto her fists behind her as her hips hold her trunk at the
crawl take's lean (TRUNK; her law's own rest, its elbows bent 45 deg under her fists, gave 25 N m and laid her down in
4.6 s); while her twist asks, strokes - her feet lifted to the stroke's start and set down till both bear, her knees
stroked to its end at her twist's rate, each side by its own share, back on the side her heading turns (a differential
drive: her heading held where her twist goes straight, a stroke's turn ahead of it where it turns) -, her fists sliding
with her, her legs' rolls and yaws and her trunk's roll and twist square (splayed, she spun 37-99 deg a back stroke
run and fell). Her twist let go, she holds where the stroke ends, sat on her fists, till her next twist or word.
Its phases are a table (PHASES), every branch a select; its arithmetic is wbc_scoot.c's, bit for bit
(`tools.cores.scoot`, what her loop plays).

    state = scoot.laid(q_deg, t)                                                           # from where she sits
    scoot.step(state, cmd, q_deg, t, dt, pitch, hands, left, right, back, turn, heading)  # cmd laid over; 0.0
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body.figure import JOINTS

#: What `machine.process.joints` gives a pass beyond her pose: her trunk's pitch (deg, forward +), her fists' and each
#: foot's share of her weight, her twist back and turning left as shares of her keys' (1 a key held, -1 its opposite),
#: her pelvis's heading (deg, left +).
SENSED = ('pitch', 'hands', 'left_foot', 'right_foot', 'back', 'turn', 'heading')
#: The parts it moves, a side each (left, right) - her hips, knees, shoulders, elbows and her legs' rolls and yaws -,
#: then her trunk's roll and twist.
PAIRED, SINGLE = ('hip', 'knee', 'shoulder', 'elbow', 'hip_roll', 'hip_yaw', 'ankle_roll'), ('spine_roll', 'waist')
PARTS = PAIRED + SINGLE
JOINT = np.array([JOINTS.index(side + p) for p in PAIRED for side in ('left_', 'right_')] + [JOINTS.index(p) for p in SINGLE])
PART = np.array([k for k in range(len(PAIRED)) for _side in range(2)] + [len(PAIRED) + k for k in range(len(SINGLE))])
SIDE = np.array([1.0, -1.0] * len(PAIRED) + [0.0] * len(SINGLE))
HIP, KNEE = PART == 0, PART == 1
#: A stroke's knees, deg: folded as the crawl take's most (126), stretched to STRETCH; a stretched knee bent back to
#: STRETCH + SETTLE as the stroke ends. Her shoulders braced at HANDS, her fists 0.3-0.45 m behind her hips, her arms
#: straight (bent, her elbows' 25 N m gave 15 deg under her and she lay down; her fists swept with her or lifted and
#: set again, she fell back). A hip lifts its foot to FOLDED as its knee folds to a stroke back (folded on the floor,
#: her heels dragged her 0.12-0.21 m forward); stretching to a stroke forward it lifts only while its foot touches,
#: holding her trunk once it skims (lifted to -100, it threw her back; held, her heels pushed her 0.23-0.27 m back). A
#: hip sets its foot toward DOWN till it bears, then holds her trunk; her trunk within LEANT of its lean as she braces.
FOLD, STRETCH, SETTLE, HANDS, FOLDED, DOWN, LEANT = 126.0, 40.0, 15.0, -45.0, -112.0, -40.0, 6.0
#: Her elbows braced at their stop, deg (their range's -5): locked past straight, a load along her arm presses them into
#: it (straight at 0, they gave 17-73 deg as she went back over her fists, one fist ended by her hip and she spun on the
#: other, 92 deg, and fell after W then S).
LOCKED = -5.0
#: Her trunk's pitch her hips hold, deg (forward +): the crawl take's lean, 47 deg off the floor (-40, 35 off it, laid
#: her down as her legs lifted); their gain, deg/s a deg, and range, deg (her bed exit's, -130: at -115 her hips could
#: flex no further and she fell back).
TRUNK, GAIN, HIP_LO, HIP_HI = -28.0, 30.0, -130.0, -35.0
#: Her trunk's pitch over a stroke back, deg: up off her fists so they slide with her (at her lean her locked arms braked
#: her, S at 8 s went 0.00 m; of six starts each, back 0.78 m a key's 8 s for 0.32, no fall in either).
UPRIGHT = -20.0
#: Rates, deg/s: a stroke's knees at a key held, her limbs in the air, her bracing shoulders, her hips lifting and
#: setting her feet, a stretched knee settling, her squared joints.
STROKE_RATE, AIR_RATE, BRACE_RATE, LIFT_RATE, SET_RATE, SETTLE_RATE, SQUARE_RATE = 100.0, 300.0, 90.0, 120.0, 60.0, 150.0, 120.0
#: A limb bears once it takes BEAR of her weight (her fists both, a foot alone) and touches at SKIM; a twist asks past
#: ASK; a goal is reached within TOL deg.
BEAR, SKIM, ASK, TOL = 0.1, 0.02, 0.01, 1e-6
#: A turn, deg, a whole stroke's differential turned her (STROKE_TURN, in 0.5 s, and threw her off her fists); a stroke
#: takes at most TURN of it (a half threw her in a turn's third stroke), her aim a stroke's turn ahead as she turns.
STROKE_TURN, TURN = 37.0, 0.25
LEAD = STROKE_TURN * TURN
#: How a part reaches its goal: 0 held, 1 its phase's goal at its rate, 2 the stroke's end at her twist's rate, 3 the
#: stroke's start (a hip by its knee's way), 4 a stretched knee settled, 5 its phase's goal while its foot bears none.
HELD, RATE, END, START, SETTLED, SETTING = range(6)
#: When a phase is over: its goals reached; and her fists and both feet bearing with her trunk at its lean, her tripod
#: set (braced before her feet bore, S told as she sat spun her 158 deg); both feet bearing or its goals reached; its
#: knees at their ends or her twist let go; her twist asking.
REACHED, BRACED, SET, STROKED, ASKING = range(5)
#: Her legs' rolls and yaws and her trunk's roll and twist, squared in every phase.
_SQUARE = {part: (RATE, 0.0, SQUARE_RATE) for part in ('hip_roll', 'hip_yaw', 'ankle_roll', 'spine_roll', 'waist')}
#: The phases: (name, {part: (how, goal, rate)}, over when, the next asked, idle); a hip not moving holds her trunk.
PHASES = (('brace', {'shoulder': (RATE, HANDS, BRACE_RATE), 'elbow': (RATE, LOCKED, AIR_RATE), **_SQUARE}, BRACED, 'lift',
           'hold'),
          ('lift', {'hip': (START, 0.0, LIFT_RATE), 'knee': (START, 0.0, AIR_RATE), **_SQUARE}, REACHED, 'set', 'set'),
          ('set', {'hip': (SETTING, DOWN, SET_RATE), **_SQUARE}, SET, 'stroke', 'stroke'),
          ('stroke', {'knee': (END, 0.0, 0.0), **_SQUARE}, STROKED, 'settle', 'settle'),
          ('settle', {'knee': (SETTLED, 0.0, SETTLE_RATE), **_SQUARE}, REACHED, 'lift', 'hold'),
          ('hold', dict(_SQUARE), ASKING, 'lift', 'hold'))
NAMED = tuple(phase[0] for phase in PHASES)
CYCLE, STROKING = NAMED.index('lift'), NAMED.index('stroke')
#: PHASES as flat tables: each part's how, goal and rate a phase; when it is over; the next asked and idle.
HOW, GOAL, RATES = (np.zeros((len(PHASES), len(PARTS))) for _table in range(3))
for _k, _phase in enumerate(PHASES):
    for _part, (_how, _goal, _rate) in _phase[1].items():
        HOW[_k, PARTS.index(_part)], GOAL[_k, PARTS.index(_part)], RATES[_k, PARTS.index(_part)] = _how, _goal, _rate
OVER = np.array([phase[2] for phase in PHASES])
ASKED, IDLE = (np.array([float(NAMED.index(phase[c])) for phase in PHASES]) for c in (3, 4))
#: The state's slots: the phase, each side's share of this stroke (+ its knee stretches, - it folds, 0 still), the
#: heading she aims at (deg) and whether she has one.
PHASE, WAY, AIM, AIMED = 0, 1, 3, 4


def laid(q_deg, t):
    """The move's state from her pose `q_deg` (deg, figure.JOINTS) at `t` s: bracing first, no stroke, no aim yet."""
    return np.zeros(AIMED + 1)


def step(state, cmd, q_deg, t, dt, pitch, hands, left, right, back, turn, heading):
    """A pass of the move `dt` s on: `cmd` (deg, figure.JOINTS) laid over from `state`, her trunk's pitch (deg), her
    fists' and feet's shares of her weight, her twist and her heading (`SENSED`) sensed; 0.0, the move never over by
    itself (a word of hers ends it)."""
    k = int(state[PHASE])
    x = cmd[JOINT]
    way = state[WAY + (SIDE < 0.0)]
    how, rate = HOW[k, PART], RATES[k, PART]
    stretched, folding = way > 0.0, way < 0.0
    start = np.where(stretched, FOLD, np.where(folding, STRETCH, x))
    end = start + (np.where(stretched, STRETCH, np.where(folding, FOLD, x)) - start) * np.abs(way)
    begin = np.where(KNEE, start, np.where(way != 0.0, FOLDED, x))
    goal = np.where((how == RATE) | (how == SETTING), GOAL[k, PART], np.where(how == END, end, np.where(
        how == START, begin, np.where(how == SETTLED, np.where(stretched, np.maximum(x, STRETCH + SETTLE), x), x))))
    foot = np.where(SIDE > 0.0, left, right)
    rate = np.where(how == END, STROKE_RATE * np.abs(way), np.where(HIP & (how == START), rate * (
        stretched | (foot >= SKIM)), np.where(how == SETTING, rate * (foot < BEAR), rate)))
    reach = rate * dt
    moved = np.where(how == HELD, x, x + np.clip(goal - x, -reach, reach))
    lean = TRUNK + (UPRIGHT - TRUNK) * float(k == STROKING and state[WAY] > 0.0 and state[WAY + 1] > 0.0)
    held = np.clip(moved + GAIN * (pitch - lean) * dt, HIP_LO, HIP_HI)
    cmd[JOINT] = np.where(HIP & ((how == HELD) | (rate == 0.0)), held, moved)
    near = np.abs(goal - cmd[JOINT]) <= TOL
    reached = bool(np.all(near | (how == HELD) | (HIP & (rate == 0.0))))
    asked = abs(back) + abs(turn) > ASK
    turning = abs(turn) > ASK
    aim = heading + LEAD * (float(turn > 0.0) - float(turn < 0.0)) if turning else (
        state[AIM] if asked and state[AIMED] > 0.5 else heading)
    state[AIM], state[AIMED] = aim, 1.0
    off = aim - heading
    off = off - 360.0 * np.floor((off + 180.0) / 360.0)
    share = np.clip(off / STROKE_TURN, -TURN, TURN)
    over = (reached, reached and hands >= BEAR and abs(pitch - TRUNK) <= LEANT and left >= BEAR and right >= BEAR,
            (left >= BEAR and right >= BEAR) or reached, bool(np.all(near | (rate == 0.0) | ~KNEE)) or not asked,
            asked)[OVER[k]]
    nxt = ASKED[k] if asked else IDLE[k]
    shares = np.clip(back + np.array([1.0, -1.0]) * share, -1.0, 1.0)
    state[WAY:WAY + 2] = shares if over and nxt == CYCLE else state[WAY:WAY + 2]
    state[PHASE] = nxt if over else k
    return 0.0
