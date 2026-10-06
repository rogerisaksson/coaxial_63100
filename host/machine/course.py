"""The quad's course: gates in the air among trees, houses and cars, a lap a line flown as rows.

    route = aerobatics.routine(course.CARD)                  # on the floor, its laps ahead
    route['seen'] = flying.seen(sky.state(), dt)             # what the line's rows go by
    name, flying.ask = aerobatics.fly(route, now, holds)     # the card's row, or the line's for now

The line is a closed curve through every gate along its heading, a Hermite span a gate. The frame
is flown along it as fast as its lean lets it: each bend at the speed a share of the pull along
the floor turns it - LEAN of the frame's whole, the boards' envelopes' at most (`flying.most`) - braked
for ahead of it, a crest no faster than it may fall. So a lap is as fast as the FETs' envelopes
are cool: asked more lean than they leave it for long, it flies on them. `TREES`, `HOUSES`,
`CARS` and `MASTS` stand about the line, for whoever draws it (`coaxial.graphics.scenery`).
"""
import functools
import math

from machine.aerobatics import DOWN, HOVER, OVER, PRESSED, resized
from machine import quad

#: A gate: its middle (x, y, z), m from where the frame rises, y up, and its heading, degrees
#: from z toward x - flown through along it; its opening GATE_M square, the floor under it at
#: the lowest. The first stands over the spot, the lap's start and its finish, the floor its
#: lower edge: a bar there was in the frame's way up, struck 8 cm off the floor; then the alley
#: between two trees, up and over the house's ridge, three quarters round the mast and down
#: it, the dive between two cars, the low bend, the slalom's two trees and home.
GATE_M = 3.4
GATES = ((0.0, 1.7, 0.0, 0.0), (0.0, 2.5, 14.0, 0.0), (7.0, 6.0, 23.0, 90.0),
         (19.0, 7.5, 23.0, 90.0), (31.0, 7.0, 21.5, 90.0), (36.5, 6.0, 27.0, 0.0),
         (31.0, 5.0, 32.5, 270.0), (25.5, 4.0, 27.0, 180.0), (27.0, 3.2, 15.0, 180.0),
         (27.0, 1.1, 3.0, 180.0), (20.0, 2.2, -6.0, 270.0), (13.0, 2.6, -3.5, 270.0),
         (6.0, 3.0, -8.5, 270.0), (2.5, 2.6, -5.0, 315.0))

#: What stands about the line. A tree: (x, z, its height, its crown's radius), m. A house: (x, z,
#: its width along x, its depth along z, its walls' height, its ridge's), the ridge along x. A
#: car: (x, z, its heading, degrees), CAR_M wide, high and long. A mast: (x, z, its side, its
#: height).
TREES = ((-3.5, 14.0, 6.0, 1.4), (3.5, 14.0, 7.0, 1.6), (8.0, 15.0, 7.5, 1.8),
         (21.0, 17.5, 6.5, 1.6), (13.0, -7.0, 6.0, 1.5), (6.0, -4.5, 6.5, 1.5),
         (-8.0, 6.0, 7.0, 1.7), (-9.0, 21.0, 9.0, 2.0), (35.0, 2.0, 7.0, 1.6),
         (13.0, 34.0, 8.0, 1.9), (31.0, -11.0, 6.5, 1.6), (-6.0, -12.0, 7.5, 1.8),
         (41.0, 34.0, 8.5, 2.0), (-12.0, -3.0, 6.0, 1.5))
HOUSES = ((13.0, 23.0, 8.0, 6.0, 3.0, 4.6), (36.0, 13.0, 7.0, 8.0, 3.0, 4.8),
          (-13.0, 11.0, 6.0, 8.0, 2.8, 4.4))
CARS = ((24.0, 3.0, 0.0), (30.0, 3.0, 180.0), (21.0, -11.0, 95.0), (15.0, -12.0, 85.0),
        (2.0, 21.0, 20.0), (40.0, 22.0, 5.0))
CAR_M = (1.8, 1.4, 4.4)
MASTS = ((31.0, 27.0, 1.6, 14.0),)

#: Their shapes: a tree's crown a six-sided cone from CROWN of its height up, on a trunk
#: TRUNK_M thick; a car's body from CAR_LOW m up to CAR_BODY of its height, its cabin on it
#: CAR_CABIN of its width and of its length; a gate's bars and posts BAR_M thick, m.
CROWN, TRUNK_M, CAR_LOW, CAR_BODY, CAR_CABIN, BAR_M = 0.35, 0.12, 0.25, 0.55, (0.9, 0.5), 0.05

#: The line: a span's tangents this much of its chord, sampled every DS m; its laps. And how
#: it crosses each gate, a row a gate: turned off the gate's heading, degrees, and its tangent
#: there, of TENSION's - the tuner's (`tools/sim/quad_race.py`); the first as it stands. It
#: rises through a gate SLOPE of the way from the gate before it to the one after, the first
#: level: level through each, the climb from the second gate to the third - 3.5 m in 12.5 -
#: was an S of 10 m/s^2 up and down at 10 m/s, the frame 1.9 m under it; rising, 0.26 m, a
#: lap 17.8 s where 19.3 on the same plan, and half its size flies where it struck
#: (2026-10-06).
TENSION, DS, LAPS, SLOPE = 1.167, 0.25, 2, 1.075
WAYS = tuple((turn, 1.0) for turn in (0.0, 6.2, -17.3, 10.7, -10.9, 7.0, 5.4, 11.3, -5.6, 10.0,
                                      -10.2, -10.2, 2.7, 16.9))

#: The lean a lap asks, of the pull along the floor all of the frame's rotors give - 26 m/s^2,
#: 69 degrees, more than the envelopes leave it for long - and its speed at most, m/s. The
#: pull it is planned on is the envelopes' share of that lean, EASY of the frame's whole at
#: the least, come down to in EASE_S of the whole and back up in twice that: planned on what
#: the law's reach left, a lap never eased and the boards stood at 0.94 of their envelopes,
#: two throttling (2026-10-06). Of that pull the plan has GRIP, a circle: a bend takes what
#: its speed asks of it, the brake before it BRAKE and the way out of it GO of what the bend
#: there leaves; the brake to its finish STOP of the pull - at a bend's it stood 0.36 m past
#: the gate, through it at 2.6 m/s; bends are braked for AHEAD_M ahead; a crest is flown no
#: faster than lets it fall DROP of gravity; a bend's pull swings from one side to the other
#: in SWING_S at the fastest.
#: The tuner's (`tools/sim/quad_race.py`, 2026-10-06), flown - these, LEAD_S, LOOK_S, SOFT_S,
#: TENSION, SLOPE and the line's crossings: on the level line 19.2 and 18.1 s where 20.8 and
#: 19.5 as the day began; on the line rising through its gates 16.1 and 15.3; the brake and
#: the way out on the circle, where shares of the pull whatever the bend, 15.5 and 15.1; the
#: tilt's loop stiffer (`flying.TILT_KP`), 14.9 and 14.4 - on four boards 15.3 and 15.1
#: where 21.9 and 20.7, a gate 0.42 m off at the most in six winds it never flew, stuffy
#: 17.7 and 17.1.
LEAN, EASY, EASE_S, TOP_M_S = 0.753, 0.116, 2.0, 13.0
GRIP, BRAKE, GO, AHEAD_M, DROP, SWING_S = 0.77, 0.744, 0.847, 40.0, 0.705, 0.511
STOP = 0.25

#: The frame's place on the line is looked for REACH_M on from the last; the law's spot is kept
#: on the line, SLACK_M from that place along it at most; a bend's pull is asked LEAD_S ahead
#: of it - what the discs' turn and the rotors' spool trail a pull by; the frame's miss of the
#: line is taken back over LOOK_S of its speed, LOOK_M at least. Its heading comes round to
#: its way at AIM_K, 1/s, no faster than TURN_RAD_S, and its lean turns with it; its nose is
#: left free of it: a turn of the frame is the discs' drag's, 10 N of thrust apart a N m -
#: held on its heading, the rotors were at their clamp 12 % of a lap, 5 % with the heading
#: still - and then the lean's own turn trailed its bend and a gate was passed 0.65 m wide
#: (2026-10-06). It stands STAND_M from its finish, this slow.
#: How it came to this, each measured on a lap of jittered passes (2026-10-05): flown by the
#: clock it fell 10 m behind its place and cut its bends 4 m inside their gates; its spot homed
#: to the frame's own place, the law kept the frame's speed, 2.5 m/s under the plan's; its
#: speed's way led with the bend, it flew 0.9 m inside the mast's gates; the row for the
#: pass's start, a pass of 30 ms after one of 14 asked twice the bend's pull.
#: The speed it asks comes to the plan's over SOFT_S, the plan looked up as much further on:
#: it steps from its way out of a bend to its brake for the next, 14 m/s^2 in a pass, and
#: the rotors spooled for the pull that stepped with it - on the four boards in seven winds
#: laps 20.2-20.7 and 19.6-20.7 s, their envelopes at 0.72-0.77, a gate 0.67 m off at the
#: most; softened, 19.5-19.7 and 18.6-19.1, 0.66-0.69, 0.50 m (2026-10-06).
REACH_M, SLACK_M, LEAD_S, LOOK_S, LOOK_M, SOFT_S = 3.0, 2.0, 0.159, 0.283, 3.0, 0.13
AIM_K, TURN_RAD_S, STAND_M, STAND_M_S = 5.0, math.tau, 0.6, 0.5

#: The lap's rows: the hover's, at a lap's pace up and down, its nose free; the lean a row
#: may take this much of the pull it is planned on.
RACE, LEAN_OVER = dict(HOVER, pace=12.0, nose=0.0), 1.5
#: On the grid: over its spot in the first gate.
GRID = dict(HOVER, height=GATES[0][1])


#: The course's constants that have a unit, each by the powers of its metres and its seconds,
#: and what is placed on it: for a frame of another size the course is as much larger and its
#: plan goes by that frame's clock (`sized`); its shares and its angles are any frame's.
_UNITS = {'GATE_M': (1, 0), 'TRUNK_M': (1, 0), 'BAR_M': (1, 0), 'DS': (1, 0), 'AHEAD_M': (1, 0),
          'REACH_M': (1, 0), 'SLACK_M': (1, 0), 'LOOK_M': (1, 0), 'STAND_M': (1, 0),
          'EASE_S': (0, 1), 'TOP_M_S': (1, -1), 'SWING_S': (0, 1), 'LEAD_S': (0, 1), 'SOFT_S': (0, 1),
          'LOOK_S': (0, 1), 'AIM_K': (0, -1), 'TURN_RAD_S': (0, -1), 'STAND_M_S': (1, -1)}
_PLACED = {'GATES': (3,), 'TREES': (), 'HOUSES': (), 'CARS': (2,), 'CAR_M': None, 'MASTS': ()}
_BUILT, _SET, _ROWS = {}, {}, []


def sized():
    """The course for the frame as `quad.sized` has it: its gates and what stands, each where
    and as large - a heading the same -, its plan's lengths and times, its rows and its
    card's seconds; the line laid again."""
    global CARD
    scope, size = globals(), quad.scales()[0]
    if not _SET:
        _SET.update({name: scope[name] for name in _PLACED}, CARD=CARD)
        _ROWS.extend((row, dict(row)) for row in (RACE, GRID))
    quad.rescaled(scope, _UNITS, _BUILT)
    for name, angles in _PLACED.items():
        scope[name] = tuple(size * v for v in _SET[name]) if angles is None else tuple(
            tuple(v if k in angles else size * v for k, v in enumerate(row)) for row in _SET[name])
    CARD = resized(_SET['CARD'], _ROWS)
    track.cache_clear()


def gate(x, y, z, heading, size=None):
    """A gate's edges, [(an end, the other)]: its opening's frame, `size` square to its
    heading, the floor under it at the lowest, on two posts."""
    c, s = math.cos(math.radians(heading)), math.sin(math.radians(heading))
    half = (GATE_M if size is None else size) / 2
    frame = [(x + a * c, max(0.0, y + b), z - a * s)
             for a, b in ((-half, -half), (half, -half), (half, half), (-half, half))]
    return list(zip(frame, frame[1:] + frame[:1])) + [(p, (p[0], 0.0, p[2])) for p in frame[:2]]


def solids():
    """What stands, as the frame's world has it to fly into (`quad.mjcf`): a tree its trunk and
    its cone, a house its walls and its roof, a car its body and its cabin, a mast; a gate
    (`gateN`) its bars and its posts - none along the floor, where the floor is its lower
    edge."""
    out = []
    for x, z, high, crown in TREES:
        foot = CROWN * high
        out += [('rod', 'tree', (x, 0.0, z), (x, foot, z), TRUNK_M),
                ('hull', 'tree', [(x + crown * math.cos(k * math.tau / 6), foot,
                                   z + crown * math.sin(k * math.tau / 6)) for k in range(6)]
                 + [(x, high, z)])]
    for x, z, wide, deep, wall, ridge in HOUSES:
        out += [('box', 'house', (x, wall / 2, z), (wide / 2, wall / 2, deep / 2), 0.0),
                ('hull', 'house', [(x + a * wide / 2, wall, z + b * deep / 2)
                                   for a in (-1, 1) for b in (-1, 1)]
                 + [(x + a * wide / 2, ridge, z) for a in (-1, 1)])]
    wide, high, long_ = CAR_M
    for x, z, heading in CARS:
        out += [('box', 'car', (x, (CAR_LOW + CAR_BODY * high) / 2, z),
                 (wide / 2, (CAR_BODY * high - CAR_LOW) / 2, long_ / 2), heading),
                ('box', 'car', (x, (1.0 + CAR_BODY) * high / 2, z),
                 (CAR_CABIN[0] * wide / 2, (1.0 - CAR_BODY) * high / 2,
                  CAR_CABIN[1] * long_ / 2), heading)]
    for x, z, side, high in MASTS:
        out.append(('box', 'mast', (x, high / 2, z), (side / 2, high / 2, side / 2), 0.0))
    for k, place in enumerate(GATES):
        out += [('rod', 'gate%d' % k, a, b, BAR_M) for a, b in gate(*place)
                if max(a[1], b[1]) > 0.0]
    return out


def _span(a, b, u):
    """The point `u` of the way along the Hermite span from crossing `a` to crossing `b` - a
    gate's middle, the way through it, degrees, its tangent's tension and its rise, m a m
    along the floor: through each along its way."""
    chord = TENSION * math.dist(a[:3], b[:3])
    out = []
    for k in range(3):
        m0, m1 = ((g[4] * chord * math.sin(math.radians(g[3])), g[4] * chord * g[5],
                   g[4] * chord * math.cos(math.radians(g[3])))[k] for g in (a, b))
        out.append((2 * u ** 3 - 3 * u ** 2 + 1) * a[k] + (u ** 3 - 2 * u ** 2 + u) * m0
                   + (-2 * u ** 3 + 3 * u ** 2) * b[k] + (u ** 3 - u ** 2) * m1)
    return tuple(out)


@functools.lru_cache(maxsize=None)
def track():
    """The line, a sample every DS m round the lap: {'at': its points, 'way': its unit
    tangents, 'bends': its (curvature, 1/m, its slope's own, 1/m, its bearing's turn, rad/m),
    'swings': what that turn changes by a metre, 1/m^2, 'gates': where each gate is along it,
    m, 'length', 'step'}."""
    fine, marks = [], []
    count = len(GATES)
    rises = [SLOPE * (GATES[(i + 1) % count][1] - GATES[i - 1][1]) / (
        math.dist(GATES[i - 1][:3:2], GATES[i][:3:2])
        + math.dist(GATES[i][:3:2], GATES[(i + 1) % count][:3:2])) if i else 0.0
        for i in range(count)]
    cross = [(x, y, z, heading + turn, tension, rise)
             for (x, y, z, heading), (turn, tension), rise in zip(GATES, WAYS, rises)]
    for i, gate in enumerate(cross):
        marks.append(len(fine))
        fine += [_span(gate, cross[(i + 1) % len(cross)], u / 64.0) for u in range(64)]
    run = [0.0]
    for p, q in zip(fine, fine[1:] + fine[:1]):
        run.append(run[-1] + math.dist(p, q))
    length, n = run[-1], int(run[-1] / DS)
    at, k = [], 0
    for j in range(n):
        s = j * length / n
        while run[k + 1] < s:
            k += 1
        share = (s - run[k]) / max(1e-9, run[k + 1] - run[k])
        p, q = fine[k], fine[(k + 1) % len(fine)]
        at.append(tuple(a + (b - a) * share for a, b in zip(p, q)))
    way = []
    for j in range(n):
        d = [b - a for a, b in zip(at[j - 1], at[(j + 1) % n])]
        size = math.sqrt(sum(x * x for x in d))
        way.append(tuple(x / size for x in d))
    step = length / n
    bends = [(math.dist(way[j - 2], way[(j + 2) % n]) / (4.0 * step),
              (way[(j + 2) % n][1] - way[j - 2][1]) / (4.0 * step),
              ((math.atan2(way[(j + 2) % n][0], way[(j + 2) % n][2])
                - math.atan2(way[j - 2][0], way[j - 2][2]) + math.pi) % math.tau - math.pi)
              / (4.0 * step)) for j in range(n)]
    swings = [abs(bends[(j + 2) % n][2] - bends[j - 2][2]) / (4.0 * step) for j in range(n)]
    return {'at': at, 'way': way, 'bends': bends, 'swings': swings, 'length': length,
            'step': step, 'gates': [run[m] for m in marks]}


def _on(row, s):
    """A row of the line - its points, its tangents, its bends - `s` samples along it, between
    two."""
    n = len(row)
    k, share = int(math.floor(s)) % n, s - math.floor(s)
    return tuple(a + (b - a) * share for a, b in zip(row[k], row[(k + 1) % n]))


def nearest(line_, point, low, high):
    """How many samples along the line - between two, whole laps counted - its point nearest
    the (x, y, z) `point` over the floor is, of those from `low` to `high`: from the nearest
    sample onto the line as it lies between two, twice. By that sample's own tangent, half a
    step either way, a frame off its line in a bend jumped at every sample, and its asked
    speed's change with it: 4.3 m/s^2 rms rough a pass to the next where 3.0, a rotor's
    spool on the boards 8.5 A rms where 7.6 (2026-10-06)."""
    n, step = len(line_['at']), line_['step']
    k = min(range(low, high + 1), key=lambda j: (
        (line_['at'][j % n][0] - point[0]) ** 2 + (line_['at'][j % n][2] - point[2]) ** 2))
    s = float(k)
    for _ in range(2):
        here, way = _on(line_['at'], s), _on(line_['way'], s)
        past = (point[0] - here[0]) * way[0] + (point[2] - here[2]) * way[2]
        s = max(k - 1.0, min(k + 1.0, s + past / (way[0] * way[0] + way[2] * way[2]) / step))
    return s


def bend_speed(bend, grip, wind):
    """The speed a bend of `bend`, 1/m, is flown at on `grip`, m/s^2 across it, in a wind of
    `wind`, m/s: the wind's drag across takes its share first - 0.5 rho CdA (v + w) w a kg,
    all of the wind across and with the frame's own speed in it - a fifth of the grip left at
    the least."""
    air = 0.5 * quad.RHO * quad.BODY_CDA / quad.MASS_KG * wind
    left = max(0.2 * grip, grip - air * wind)
    return 2.0 * left / (air + math.sqrt(air * air + 4.0 * bend * left))


def line(route, now):
    """The lap's row for now (`aerobatics.fly`'s giver), by what `route['seen']` says
    (`Flying.seen`) - where the frame is and goes, the law's spot and heading, the share of
    their pull the envelopes leave, the pass's seconds, whether it is `spent`: its speed the
    one the bends ahead allow on that share of its lean and what the wind as learnt leaves of
    it (`bend_speed`), along the line's way and back onto it, its bend's pull asked LEAD_S
    ahead, its spot kept on the line. Its laps flown - spent, the one it is on - it stands in
    the first gate, `lapped`. Where the line's turn swings, no faster than the bend's pull
    goes from one side to the other in SWING_S: v^3 a metre of swing is the pull's change a
    second."""
    lap = route.get('lap')
    if lap is None or lap['began'] != route['at']:
        lap = route['lap'] = {'began': route['at'], 'k': 0, 'v': 0.0, 'asks': 0.0, 'laps': 0,
                              'gate': 1, 'of': LAPS, 'pull': 0.0}
    seen, line_ = route.get('seen') or {}, track()
    n, step = len(line_['at']), line_['step']
    dt, heading = seen.get('dt', 0.0), seen.get('heading', 0.0)
    at, vel = seen.get('at', line_['at'][0]), seen.get('vel', (0.0, 0.0, 0.0))
    wind = math.hypot(*seen.get('wind', (0.0, 0.0)))
    # the pull it is planned on: the envelopes' share of its lean, come to slowly
    full = seen.get('full', quad.GRAVITY)
    lean = LEAN * full
    ease = lean * dt / EASE_S
    pull = lap['pull'] = max(EASY * full, lap['pull'] - ease, min(
        lean * seen.get('share', 1.0), seen.get('most', lean), lap['pull'] + 0.5 * ease))
    # where the frame is on the line, from where it was on
    s = nearest(line_, at, lap['k'], lap['k'] + int(REACH_M / step))
    if seen.get('spent'):
        lap['of'] = min(lap['of'], int(s) // n + 1)
    end = float(lap['of'] * n)
    s = min(end, s)
    k = lap['k'] = int(s)
    here, way = _on(line_['at'], s), _on(line_['way'], s)
    along = max(0.0, sum(float(v) * w for v, w in zip(vel, way)))
    # what the law takes the change of spans the pass to come: asked for where that ends
    on = min(end, s + along * dt / step)
    lead = min(end, on + LEAD_S * lap['v'] / step)
    # as fast as the bends ahead allow on a circle of its grip - a bend at the speed all of the
    # grip turns it, a crest as its fall, braked for and come out of on what the bend there
    # leaves of the grip - run back from the far end of what is looked at to where it is as
    # much further on as its asked speed trails; to a stand at its finish
    soon = min(end, lead + SOFT_S * lap['v'] / step)
    grip, base, far = GRIP * pull, int(soon), min(int(AHEAD_M / step), int(end - soon))
    crest, swung = grip / (DROP * quad.GRAVITY), 2.0 * grip / SWING_S
    square, bend = TOP_M_S * TOP_M_S, 0.0
    for j in range(far, -1, -1):
        bend, rise, _veer = line_['bends'][(base + j) % n]
        bend += max(0.0, -rise) * crest
        if bend > 0.0:
            square = min(square, bend_speed(bend, grip, wind) ** 2)
        swing = line_['swings'][(base + j) % n]
        if swing > 0.0:
            square = min(square, (swung / swing) ** (2.0 / 3.0))
        if j:
            left = math.sqrt(max(0.0, grip * grip - (square * bend) ** 2))
            square += 2.0 * BRAKE * left * step * (1.0 if j > 1 else 1.0 - soon % 1.0)
    allowed = min(TOP_M_S, math.sqrt(min(square, 2.0 * STOP * pull * (end - soon) * step)))
    left = math.sqrt(max(0.0, grip * grip - (lap['v'] ** 2 * bend) ** 2))
    speed = lap['v'] = min(allowed, lap['v'] + GO * left * dt)
    lap.update(laps=min(lap['of'] - 1, k // n),
               gate=next((g for g, mark in enumerate(line_['gates']) if mark > (k % n) * step), 0))
    if (end - s) * step <= STAND_M and math.sqrt(sum(float(v) ** 2 for v in vel)) <= STAND_M_S:
        route['holds'] = ('lapped',)
    # its way: the line's, the frame's miss of the line taken back over its look
    to, veer = _on(line_['way'], on), _on(line_['bends'], on)[2]
    look = max(LOOK_M, LOOK_S * speed)
    aim = math.atan2(look * to[0] - (at[0] - here[0]), look * to[2] - (at[2] - here[2]))
    # its heading round to it as the line turns, and its speed along that way
    turn = speed * veer + AIM_K * ((aim - heading + math.pi) % math.tau - math.pi)
    turn = max(-TURN_RAD_S, min(TURN_RAD_S, turn))
    nose = heading + turn * dt
    lap['asks'] += (speed - lap['asks']) * min(1.0, dt / SOFT_S)
    flat = lap['asks'] * math.hypot(to[0], to[2])
    # the pull its bend takes a lag ahead, over the one where it is: the row's own
    ahead, bent = _on(line_['way'], lead), _on(line_['bends'], lead)[2]
    more = [flat * flat * (bent * a / math.hypot(ahead[0], ahead[2])
                           - veer * b / math.hypot(to[0], to[2]))
            for a, b in ((ahead[2], to[2]), (-ahead[0], -to[0]))]
    # the law's spot onto the line, no further along it from the frame than its slack
    slack = SLACK_M / step
    spot = seen.get('spot', (here[0], here[2]))
    place = _on(line_['at'], max(s - slack, min(s + slack, end, nearest(
        line_, (spot[0], 0.0, spot[1]), k - int(slack), k + int(slack) + 1))))
    c, sn = math.cos(nose), math.sin(nose)
    return dict(RACE, height=here[1], climb=along * way[1], lean=LEAN_OVER * pull,
                push=along * along * _on(line_['bends'], s)[1], x=place[0], z=place[2],
                speed=flat * math.cos(aim - nose), slide=flat * math.sin(aim - nose),
                surge=more[0] * sn + more[1] * c, sway=more[0] * c - more[1] * sn,
                turn=math.degrees(turn))


#: The course's flight: from the floor onto its grid, its laps, and down.
CARD = (
    ('idle', DOWN, 2.0, 0.0, 'fit'),
    ('spool', PRESSED, 2.5, 2.5, ''),
    ('lift', GRID, 3.0, 3.0, ''),
    ('grid', GRID, 1.0, 0.0, 'held'),
    ('lap', line, 0.0, 0.0, 'lapped'),
    ('level', GRID, 1.5, 1.0, 'held'),
    ('descend', OVER, 1.5, 1.5, 'held'),
    ('land', PRESSED, 3.0, 3.0, ''),
)
