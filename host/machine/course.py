"""The quad's course: gates in the air among trees, houses and cars, a lap a line flown as rows.

    route = aerobatics.routine(course.CARD)                  # on the floor, its laps ahead
    route['seen'] = flying.seen(sky.state(), dt)             # what the line's rows go by
    name, flying.ask = aerobatics.fly(route, now, holds)     # the card's row, or the line's for now

The line is a closed curve through every gate along its heading, a Hermite span a gate. The frame
is flown along it as fast as its lean lets it: each bend at the speed a share of the pull along
the floor turns it - LEAN of the frame's whole, the boards' envelopes' at most (`flying.most`) - braked
for ahead of it, a crest no faster than it may fall. So a lap is as fast as the FETs' envelopes
are cool: asked more lean than they leave it for long, it flies on them. Its gates and what
stands about it are `machine.grounds`'.
"""
import functools
import math

from machine.aerobatics import DOWN, HOVER, OVER, PRESSED, resized
from machine import grounds, quad

#: The line: a span's tangents this much of its chord, sampled every DS m; its laps. And how
#: it crosses each gate, a row a gate - the tuner's (`tools/sim/quad_race.py`), the first as
#: it stands: turned off the gate's heading, degrees; its tangent there, of TENSION's; where
#: across and up its opening, m off its middle - its apex. It rises through a gate SLOPE of
#: the way from the gate before it to the one after, the first level: level through each, a
#: climb of 3.5 m in 12.5 was an S of 10 m/s^2 up and down at 10 m/s (2026-10-06).
TENSION, DS, LAPS, SLOPE = 1.098, 0.25, 2, 1.067
WAYS = ((3.8, 0.988, -0.20, -0.50), (-20.5, 1.163, 0.14, -0.05), (-8.8, 1.036, -0.09, -0.23),
        (1.6, 1.042, 0.20, 0.12), (-10.3, 0.722, 0.11, -0.04), (-6.4, 0.709, -0.44, -0.09),
        (8.3, 1.348, 0.06, 0.13), (2.0, 0.741, -0.52, -0.09), (3.1, 0.767, -0.09, -0.33),
        (-2.9, 0.780, -0.14, 0.05), (-6.3, 0.735, -0.21, -0.06), (-13.6, 0.936, 0.04, 0.13),
        (-4.3, 1.209, -0.36, -0.11), (1.3, 0.818, -0.36, 0.09), (12.3, 1.091, 0.20, 0.11),
        (3.1, 1.041, 0.29, -0.28))

#: The lean a lap asks, of the pull along the floor all of the frame's rotors give - 26 m/s^2,
#: 69 degrees, more than the envelopes leave it for long - and its speed at most, m/s, below
#: the one its pull holds against the frame's drag: capped at 13 its rotors ran at 0.54-0.71
#: of their top between the gates, at the cap 58-100 % of each straight (2026-10-09). The
#: pull it is planned on is the envelopes' share of that lean, EASY of the frame's whole at
#: the least, come down to in EASE_S of the whole and back up in twice that: planned on what
#: the law's reach left, a lap never eased and the boards stood at 0.94 of their envelopes,
#: two throttling (2026-10-06). Of that pull the plan has GRIP, a circle: a bend takes what
#: its speed asks of it, the brake before it BRAKE and the way out of it GO of what the bend
#: there leaves; the brake to its finish STOP of the pull - at a bend's it stood 0.36 m past
#: the gate, through it at 2.6 m/s; bends are braked for AHEAD_M ahead; a crest is flown no
#: faster than lets it fall DROP of gravity; a bend's pull swings from one side to the other
#: in SWING_S at the fastest.
#: These, LEAD_S, LOOK_S, SOFT_S, TENSION, SLOPE and the line's crossings are the tuner's: on
#: the larger course with the 16x14s, laps of 17.9 and 17.0 s with a small frame's tip 0.26 m
#: past its margin; a gate's margin weighed five times, 19.6 and 19.2, on four boards 19.5 and
#: 19.0, in two winds it never flew 19.5-19.8 and 19.0-19.9, every tip inside (2026-10-09).
LEAN, EASY, EASE_S, TOP_M_S = 0.249, 0.05, 2.0, 22.2
#: Out of a bend and into the next it pulls along its way on THROTTLE of the pull its frame
#: has, as LEAN is of it - its grip an ellipse, the bend's across and this along -, the bend's
#: grip at the least: on the bend's, a 16x14 ran at 0.19 of its top between the gates.
THROTTLE = 0.30
GRIP, BRAKE, GO, AHEAD_M, DROP, SWING_S = 0.751, 0.682, 0.941, 40.0, 0.511, 0.484
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
#: Flown by the clock it fell 10 m behind its place and cut its bends 4 m inside their gates;
#: its spot homed to the frame's own place, it kept the frame's speed, 2.5 m/s under the plan's
#: (2026-10-05). The speed it asks comes to the plan's over SOFT_S, the plan looked up as much
#: further on: it stepped from its way out of a bend to its brake for the next, 14 m/s^2 in a
#: pass (2026-10-06).
REACH_M, SLACK_M, LEAD_S, LOOK_S, LOOK_M, SOFT_S = 3.0, 2.0, 0.135, 0.309, 3.0, 0.223
AIM_K, TURN_RAD_S, STAND_M, STAND_M_S = 5.0, math.tau, 0.6, 0.5

#: The lap's rows: the hover's, at a lap's pace up and down, its nose free; the lean a row
#: may take this much of the pull it is planned on.
RACE, LEAN_OVER = dict(HOVER, pace=12.0, nose=0.0), 1.5
#: On the grid: over its spot in the first gate.
GRID = dict(HOVER, height=grounds.GATES[0][1])


#: The course's constants that have a unit, each by the powers of its metres and its seconds:
#: for a frame of another size the course is as much larger (`grounds.sized`) and its plan goes
#: by that frame's clock (`sized`); its shares and its angles are any frame's.
_UNITS = {'DS': (1, 0), 'AHEAD_M': (1, 0),
          'REACH_M': (1, 0), 'SLACK_M': (1, 0), 'LOOK_M': (1, 0), 'STAND_M': (1, 0),
          'EASE_S': (0, 1), 'TOP_M_S': (1, -1), 'SWING_S': (0, 1), 'LEAD_S': (0, 1), 'SOFT_S': (0, 1),
          'LOOK_S': (0, 1), 'AIM_K': (0, -1), 'TURN_RAD_S': (0, -1), 'STAND_M_S': (1, -1)}
_BUILT, _SET, _ROWS = {}, {}, []


def sized():
    """The course for the frame as `quad.sized` has it, its grounds `sized` first: its plan's
    lengths and times, its rows and its card's seconds; the line laid again."""
    global CARD
    if not _SET:
        _SET.update(CARD=CARD)
        _ROWS.extend((row, dict(row)) for row in (RACE, GRID))
    quad.rescaled(globals(), _UNITS, _BUILT)
    CARD = resized(_SET['CARD'], _ROWS)
    track.cache_clear()



def _span(a, b, u):
    """The point `u` of the way along the Hermite span from crossing `a` to crossing `b` - where
    it crosses a gate, the way through it, degrees, its tangent's tension and its rise, m a m
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
    fine, marks, gates = [], [], grounds.GATES
    count = len(gates)
    rises = [SLOPE * (gates[(i + 1) % count][1] - gates[i - 1][1]) / (
        math.dist(gates[i - 1][:3:2], gates[i][:3:2])
        + math.dist(gates[i][:3:2], gates[(i + 1) % count][:3:2])) if i else 0.0
        for i in range(count)]
    cross = [(x + across * math.cos(math.radians(heading)), y + up,
              z - across * math.sin(math.radians(heading)), heading + turn, tension, rise)
             for (x, y, z, heading, _size), (turn, tension, across, up), rise
             in zip(gates, WAYS, rises)]
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
    push = max(grip, THROTTLE / LEAN * pull)
    crest, swung = grip / (DROP * quad.GRAVITY), 2.0 * grip / SWING_S
    top = min(TOP_M_S, math.sqrt(max(pull, push) * quad.MASS_KG
                                 / (0.5 * quad.RHO * quad.BODY_CDA)))
    square, bend = top * top, 0.0
    for j in range(far, -1, -1):
        bend, rise, _veer = line_['bends'][(base + j) % n]
        bend += max(0.0, -rise) * crest
        if bend > 0.0:
            square = min(square, bend_speed(bend, grip, wind) ** 2)
        swing = line_['swings'][(base + j) % n]
        if swing > 0.0:
            square = min(square, (swung / swing) ** (2.0 / 3.0))
        if j:
            left = push * math.sqrt(max(0.0, 1.0 - (square * bend / grip) ** 2))
            square += 2.0 * BRAKE * left * step * (1.0 if j > 1 else 1.0 - soon % 1.0)
    allowed = min(top, math.sqrt(min(square, 2.0 * STOP * pull * (end - soon) * step)))
    left = push * math.sqrt(max(0.0, 1.0 - (lap['v'] ** 2 * bend / grip) ** 2))
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
    return dict(RACE, height=here[1], climb=along * way[1], lean=LEAN_OVER * max(pull, push),
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
