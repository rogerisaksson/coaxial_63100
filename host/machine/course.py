"""The quad's course: gates in the air among trees, houses and cars, a lap a line flown as rows.

    route = aerobatics.routine(course.CARD)                  # on the floor, its laps ahead
    route['seen'] = flying.seen(sky.state(), dt)             # what the line's rows go by
    name, flying.ask = aerobatics.fly(route, now, holds)     # the card's row, or the line's for now

The line is a closed curve through every gate along its heading, a Hermite span a gate. The frame
is flown along it as fast as its thrust lets it, pointed wherever it must - down, where a dive is
faster than a fall: each bend at the speed its sphere turns it, braked for ahead of it. So a lap
is as fast as the FETs' envelopes are cool: asked more than they leave it for long, it flies on
them. Its gates and what stands about it are `machine.grounds`'.
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
        (2.9, 1.348, 0.14, 0.02), (2.0, 0.741, -0.52, -0.09), (3.1, 0.767, -0.09, -0.22),
        (-2.9, 0.780, -0.14, 0.05), (-6.3, 0.735, -0.21, -0.06), (-13.6, 0.936, 0.15, 0.03),
        (-4.3, 1.209, -0.36, -0.11), (1.3, 0.818, -0.36, 0.09), (12.3, 1.091, 0.20, 0.11),
        (3.1, 1.041, 0.29, -0.28))

#: The plan (the user, 2026-10-10: raw - a dive pushed inverted, no level flight kept): a
#: sphere about gravity, PULL of what the boards' envelopes leave of the rotors' thrust, the
#: thrust v^2 k + a t + g up wherever that points. A bend takes what its speed asks across; it
#: is braked for BRAKE and sped out of GO of what is left along its way, the air through the
#: discs taking their thrust (`quad.inflow`), the slope and the drag theirs; braked to its
#: finish on STOP of it; the pull comes down to the envelopes' in EASE_S and back in twice
#: that; bends braked for AHEAD_M ahead; a bend's pull swings from one side to the other in
#: SWING_S at the fastest; the sphere FLOOR of gravity at the least. On a circle of 0.249 of its
#: pull along the floor, a crest no faster than 0.511 g let it fall and 0.30 of the pull along
#: its way, the 16x14s lapped in 19.6 and 19.2 s, 15.2 m/s at the most, never sped down past
#: -8.8 m/s^2 (2026-10-09/10). Searched raw over boards, winds, sizes and heat: 13.8 and 14.0,
#: on four boards 14.1 and 14.7; in two winds it never flew every flight whole, the boards'
#: tips 0.06-0.09 m past their margin, SOA 0.84-1.00, WEP 0.6-2.4 s (2026-10-10). These, the
#: law's gains, LEAD_S, LOOK_S, SOFT_S, SLACK, TENSION, SLOPE and the crossings are the tuner's.
PULL, BRAKE, GO, EASE_S, STOP, FLOOR = 0.399, 0.727, 0.669, 2.0, 0.25, 1.1
AHEAD_M, SWING_S = 40.0, 0.360

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
REACH_M, SLACK_M, LEAD_S, LOOK_S, LOOK_M, SOFT_S = 3.0, 2.0, 0.089, 0.191, 3.0, 0.217
AIM_K, TURN_RAD_S, STAND_M, STAND_M_S = 5.0, math.tau, 0.6, 0.5

#: The lap's rows: the hover's, at any pace up and down, its nose free; the lean a row may take
#: this much of the pull it is planned on. Its discs point down as far as its line asks - the
#: drag's part in it - and SLACK of gravity past that: free, 0.4 m over a rise in a bend the
#: frame turned over to 125 degrees, and fell 1.5 m into the floor righting itself (2026-10-10).
RACE, LEAN_OVER, SLACK = dict(HOVER, pace=100.0, nose=0.0), 1.5, 0.264
#: On the grid: over its spot in the first gate.
GRID = dict(HOVER, height=grounds.GATES[0][1])


#: The course's constants that have a unit, each by the powers of its metres and its seconds:
#: for a frame of another size the course is as much larger (`grounds.sized`) and its plan goes
#: by that frame's clock (`sized`); its shares and its angles are any frame's.
_UNITS = {'DS': (1, 0), 'AHEAD_M': (1, 0),
          'REACH_M': (1, 0), 'SLACK_M': (1, 0), 'LOOK_M': (1, 0), 'STAND_M': (1, 0),
          'EASE_S': (0, 1), 'SWING_S': (0, 1), 'LEAD_S': (0, 1), 'SOFT_S': (0, 1),
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
    'curve': its way's turn, a vector, 1/m, 'swings': what that changes by a metre, 1/m^2,
    'gates': where each gate is along it, m, 'length', 'step'}."""
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
    curve = [tuple((b - a) / (4.0 * step) for a, b in zip(way[j - 2], way[(j + 2) % n]))
             for j in range(n)]
    swings = [math.dist(curve[(j + 2) % n], curve[j - 2]) / (4.0 * step) for j in range(n)]
    return {'at': at, 'way': way, 'bends': bends, 'curve': curve, 'swings': swings,
            'length': length, 'step': step, 'gates': [run[m] for m in marks]}


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


def _across(square, curve, rise):
    """The pull across its way a bend `curve` asks at v^2 = `square`, m/s^2, the way rising
    `rise` a metre: |v^2 k + g (up - rise way)|."""
    turn = sum(c * c for c in curve)
    return math.sqrt(max(0.0, square * square * turn + 2.0 * square * quad.GRAVITY * curve[1]
                         + quad.GRAVITY ** 2 * (1.0 - rise * rise)))


def _fastest(curve, rise, sphere):
    """v^2 at which a bend `curve`'s pull across fills the `sphere`, m/s^2 (`_across`): over a
    crest gravity bends it the first g, under it the frame's thrust down the rest."""
    turn = sum(c * c for c in curve)
    if turn < 1e-12:
        return math.inf
    lift = quad.GRAVITY * curve[1]
    free = quad.GRAVITY ** 2 * (1.0 - rise * rise) - sphere * sphere
    return (math.sqrt(max(0.0, lift * lift - turn * free)) - lift) / turn


def _along(speed, across, sphere, pitch):
    """The pull along its way the `sphere` leaves beside `across`, m/s^2, at `speed`, m/s: the
    air through the discs takes their thrust - none at `pitch`, m/s, along their axis - as much
    as the thrust points along the way, and the pull is most where it points half way there."""
    if across >= sphere:
        return 0.0
    low, high = 0.0, 1.0
    for _ in range(16):
        share = 0.5 * (low + high)
        if sphere * (1.0 - speed * share / pitch) * math.sqrt(1.0 - share * share) >= across:
            low = share
        else:
            high = share
    share = min(low, 0.5 * pitch / speed if speed > 0.0 else 1.0)
    return sphere * (1.0 - speed * share / pitch) * share


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
    # the sphere it is planned on: PULL of what the envelopes leave, FLOOR of gravity at the
    # least - spent, PULL of the law's least was under gravity -, come to slowly
    whole = max(FLOOR * quad.GRAVITY, PULL * seen.get('pull', 2.0 * quad.GRAVITY))
    # at a share of all of it a second: of what is left, a stuffy room's spent boards were
    # asked twice what they had for 2 s and it flew into a house (2026-10-10)
    ease = PULL * seen.get('full', 2.0 * quad.GRAVITY) * dt / EASE_S
    pull = lap['pull'] = whole if not lap['pull'] else max(lap['pull'] - ease, min(
        whole, lap['pull'] + 0.5 * ease))
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
    # as fast as the bends ahead allow in its sphere - the wind's drag across taking its share
    # first - a bend at the speed all of it turns it, braked for on what the bend there leaves
    # along its way, the slope's and the drag's in it, run back from the far end of what is
    # looked at to where it is as much further on as its asked speed trails; to a stand at its
    # finish; sped up on what is left where it is, through the air along its discs
    soon = min(end, lead + SOFT_S * lap['v'] / step)
    base, far = int(soon), min(int(AHEAD_M / step), int(end - soon))
    drag = 0.5 * quad.RHO * quad.BODY_CDA / quad.MASS_KG
    sphere = max(0.5 * pull, pull - drag * wind * (wind + lap['v']))
    swung, square = 2.0 * sphere / SWING_S, math.inf
    for j in range(far, -1, -1):
        rise, curve = line_['way'][(base + j) % n][1], line_['curve'][(base + j) % n]
        square = min(square, _fastest(curve, rise, sphere))
        swing = line_['swings'][(base + j) % n]
        if swing > 0.0:
            square = min(square, (swung / swing) ** (2.0 / 3.0))
        if j:
            across = _across(square, curve, rise)
            brake = (math.sqrt(max(0.0, sphere * sphere - across * across))
                     + quad.GRAVITY * rise + drag * square)
            square += 2.0 * BRAKE * max(0.0, brake) * step * (1.0 if j > 1 else 1.0 - soon % 1.0)
    allowed = math.sqrt(max(0.0, min(square, 2.0 * STOP * sphere * (end - soon) * step)))
    was, curve = lap['v'], _on(line_['curve'], s)
    sped = (GO * _along(was, _across(was * was, curve, way[1]), sphere,
                        seen.get('pitch', math.inf)) - quad.GRAVITY * way[1] - drag * was * was)
    speed = lap['v'] = max(0.0, min(allowed, was + sped * dt))
    gained = max(-sphere, min(sphere, (speed - was) / dt)) if dt > 0.0 else 0.0
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
    push = along * along * _on(line_['bends'], s)[1] + gained * way[1]
    held = push + drag * along * along * way[1]
    return dict(RACE, height=here[1], climb=along * way[1], lean=LEAN_OVER * sphere, push=push,
                light=min(RACE['light'], 1.0 + held / quad.GRAVITY - SLACK), x=place[0],
                z=place[2],
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
