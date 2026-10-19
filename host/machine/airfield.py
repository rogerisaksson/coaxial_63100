"""The aerobat's airfield: its runway, a course laid for a plane, what stands about them.

    route = aerobatics.routine(airfield.CARD)            # off the runway, its laps, back onto it
    name, row = aerobatics.fly(route, now, holds)        # `line` gives the lap's rows
    sky = aerobat.Sky(airfield.solids())                 # the field as the plane's world has it

The runway ends at the middle of the world on RUNWAY_RAD. The lap (LEGS) is a plane's, flown
at SPEED_M_S on turns of TURN_M from the near turn's exit: up the runway's line through an air
gate and under a limbo bar at LOW_M, round the far turn up through a hoop at TOP_M, a slalom of
three pylons down the back straight, a knife gate taken on its side, round the near turn onto
the runway's line. Its line (`track`) is what it flies and what is drawn:
straights and arcs a metre apart, each with its height and its knife. The gates stand only
while it is flown (`quad.Sky.stand`); what stands always is clear of every line the routine and
the course fly.
"""
import math

from machine import grounds
from machine.aerobatics import DOWN, HOVER, PRESSED

RUNWAY_RAD = math.radians(50.0)
#: The runway's strip, m along its line from its end: where a landing stops.
RUNWAY_ALONG = (-20.5, 16.0)
SPEED_M_S, TURN_M, HIGH_M, LOW_M, TOP_M, LAPS = 14.0, 27.0, 3.0, 1.2, 10.0, 2
#: The point on the line it turns for, s ahead over the floor (L1, ArduPlane's), its rate at the
#: most; the height and the knife read so far ahead, s. The line's tangent crabbed onto it over
#: a second (`wingborne.lined`) put it 3.9 m off the hoop in gusts.
L1_S, TURN_RAD_S, LEAD_S = 1.0, math.radians(45.0), (0.8, 0.5)
#: Faster on its side, m/s: at 14 m/s in gusts its wings swung 34 deg off upright in the gate.
KNIFE_M_S = 4.0
#: A pylon's side and the knife gate's width, m: an air race's, twice the wings, taken on its
#: side by the rule - at 1.5 m, narrower than them, gusts put it 1.6 m off in knife-edge.
PYLON_M, SLOT_M = 0.4, 3.2
#: Where the lap starts, (along, across) the runway from its end: the near turn's exit.
START = (-76.0, 0.0)
#: The slalom, (deg, m) turned, + toward the far side: its peaks 4 m either side of its pylons.
WEAVE = ((-30.0, 30.0), (60.0, 30.0), (-60.0, 30.0), (60.0, 30.0), (-30.0, 30.0))
#: The lap: metres straight, or (deg, m) turned.
LEGS = (185.0, (180.0, TURN_M), 8.0) + WEAVE + (57.0, (180.0, TURN_M))


def placed(along, across):
    """World (x, z) of a point `along` the runway from its end and `across` it, m."""
    s, c = math.sin(RUNWAY_RAD), math.cos(RUNWAY_RAD)
    return along * s + across * c, along * c - across * s


def _laid():
    """The lap a metre apart, [(along, across, heading rad off the runway's, 1/m turned)], and
    where along it each leg begins."""
    a, c, psi, out, starts = START[0], START[1], 0.0, [], []
    for leg in LEGS:
        starts.append(len(out))
        if isinstance(leg, tuple):
            turned, r = math.radians(leg[0]), leg[1]
            length = abs(turned) * r
        else:
            turned, r, length = 0.0, math.inf, leg
        bend, n = math.copysign(1.0 / r, turned), max(1, round(length))
        for _ in range(n):
            out.append((a, c, psi, bend))
            step = length / n
            if bend:
                a += (math.sin(psi + bend * step) - math.sin(psi)) / bend
                c += (math.cos(psi) - math.cos(psi + bend * step)) / bend
                psi += bend * step
            else:
                a, c = a + step * math.cos(psi), c + step * math.sin(psi)
    return out, starts


_LAP, _STARTS = _laid()
#: Where along the lap, m: the air gate, the limbo, the hoop at the far turn's top, the pylons
#: at the slalom's middle arcs, the slot.
S_GATE, S_LIMBO = 120.0, 157.0
S_HOOP = _STARTS[1] + 0.5 * math.pi * TURN_M
S_PYLONS = tuple(_STARTS[k] + 0.5 * (_STARTS[k + 1] - _STARTS[k]) for k in (4, 5, 6))
S_SLOT = _STARTS[8] + 30.0


def _bump(d, w):
    return 0.5 * (1.0 + math.cos(math.pi * d / w)) if abs(d) < w else 0.0


#: The lap's line: [(along, across, heading, 1/m turned, m high, knife)] a metre apart.
TRACK = tuple(p + (HIGH_M - (HIGH_M - LOW_M) * _bump(s - S_LIMBO, 30.0) + (TOP_M - HIGH_M) * _bump(s - S_HOOP, 55.0),
                   1.0 if abs(s - S_SLOT) <= 8.0 else _bump(abs(s - S_SLOT) - 8.0, 22.0))
              for s, p in enumerate(_LAP))


def _gate(s, wide, high, middle, across=None):
    a, c, psi = _LAP[int(s)][:3]
    x, z = placed(a, c if across is None else across)
    return x, middle, z, math.degrees(RUNWAY_RAD + psi), wide, high


#: The gates in the order flown, (x, y, z, heading, wide, high): the air gate, the limbo bar,
#: the hoop, the pylons on the slalom's middle line, the slot.
GATES = (_gate(S_GATE, 8.0, 8.0, 4.0), _gate(S_LIMBO, 8.0, 2.8, 1.4),
         _gate(S_HOOP, 6.0, 6.0, TOP_M)) + tuple(
    _gate(s, PYLON_M, 8.0, 4.0, _LAP[_STARTS[4]][1]) for s in S_PYLONS) + (_gate(S_SLOT, SLOT_M, 7.0, 3.5),)

#: What stands always: a tree line down the far side and across the far end, (x, z, m high,
#: crown m); the hangar by the runway's start, (x, z, wide, deep, wall, ridge); two cars by it.
TREES = tuple(placed(a, c) + (h, 1.4) for a, c, h in (
    [(a, -85.0, 7.0 + (a % 3)) for a in range(-100, 101, 25)]
    + [(165.0, c, 8.0) for c in (-45.0, -15.0, 15.0, 45.0, 75.0)] + [(a, 100.0, 6.5) for a in (-60.0, 0.0, 60.0)]))
HOUSES = (placed(-12.0, -22.0) + (12.0, 10.0, 4.0, 5.5),)
CARS = tuple(placed(a, c) + (math.degrees(RUNWAY_RAD),) for a, c in ((-3.0, -19.0), (0.5, -19.5)))
MASTS, HALLS = (), ()

#: Its lean the wing's, m/s^2: 12 at 14 m/s is 10.5 deg; the hover's 8 left 0.7 in a turn.
CRUISE = dict(HOVER, height=HIGH_M, pace=5.0, speed=SPEED_M_S, home=0.0, lean=12.0)
FINAL = dict(CRUISE, height=0.22, speed=11.5, pace=1.8)


def solids():
    """The field as the plane's world has it (`grounds.solids`)."""
    import sys
    return grounds.solids(sys.modules[__name__])


def track():
    """{'at': [(x, y, z)]}: the lap's line a metre apart at its height, as `course.track` gives its line."""
    return {'at': [(x, h, z) for a, c, _psi, _bend, h, _knife in TRACK for x, z in [placed(a, c)]]}


def line(route, now):
    """The lap's row for now (`aerobatics.fly`'s giver): on its wing at SPEED_M_S at the height
    and the knife the line asks LEAD_S ahead, turned for its point L1_S ahead as `2 sin(eta) /
    L1_S`, eta its way over the floor off it, TURN_RAD_S at the most; a lap at its end, LAPS
    laps, `lapped` there."""
    fresh = {'began': route['at'], 'k': None, 'laps': 0, 'gate': 0, 'of': LAPS}
    lap = route.get('lap', fresh)
    lap = route['lap'] = (lap, fresh)[int(lap['began'] != route['at'])]
    seen = route.get('seen', {})
    x, y, z = seen.get('at', (0.0, 0.0, 0.0))
    vel, heading = seen.get('vel', (0.0, 0.0, 0.0)), seen.get('heading', RUNWAY_RAD)
    s, c = math.sin(RUNWAY_RAD), math.cos(RUNWAY_RAD)
    a, b, n = x * s + z * c, x * c - z * s, len(TRACK)
    near = range(n) if lap['k'] is None else range(lap['k'], lap['k'] + 40)
    k = min(near, key=lambda i: (TRACK[i % n][0] - a) ** 2 + (TRACK[i % n][1] - b) ** 2) % n
    lap.update(laps=lap['laps'] + int(lap['k'] is not None and k < lap['k']), k=k)
    lap['gate'] = next((g for g, at in enumerate(S_GATES) if at > k), 0)
    route['holds'] = (route.get('holds', ()), ('lapped',))[int(lap['laps'] >= LAPS)]
    fast = math.hypot(vel[0], vel[2])
    px, pz = placed(*TRACK[(k + max(4, round(L1_S * fast))) % n][:2])
    eta = (math.atan2(px - x, pz - z) - (math.atan2(vel[0], vel[2]) if fast > 1.0 else heading)
           + math.pi) % math.tau - math.pi
    rate = max(-TURN_RAD_S, min(TURN_RAD_S, 2.0 * math.sin(eta) / L1_S))
    knife = TRACK[(k + round(LEAD_S[1] * SPEED_M_S)) % n][5]
    return dict(CRUISE, turn=math.degrees(rate), height=TRACK[(k + round(LEAD_S[0] * SPEED_M_S)) % n][4],
                knife=knife, speed=SPEED_M_S + KNIFE_M_S * knife)


#: Where along the lap each gate stands, m.
S_GATES = (S_GATE, S_LIMBO, S_HOOP) + S_PYLONS + (S_SLOT,)

#: The course's flight: off the runway into its laps and back onto it.
CARD = (
    ('idle', DOWN, 2.0, 0.0, 'fit'),
    ('spool', PRESSED, 2.5, 2.5, ''),
    ('lap', line, 0.0, 0.0, 'lapped'),
    ('final', FINAL, 2.4, 0.5, ''),
    ('land', PRESSED, 10.0, 0.0, ''),
)
