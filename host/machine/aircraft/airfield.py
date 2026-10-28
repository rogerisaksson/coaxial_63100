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
import os

from machine.aircraft import aerolisp, grounds
from machine.aircraft.aerobatics import DOWN, HOVER, PRESSED

with open(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'lisp', 'aircraft', 'aerobat.lisp'), encoding='utf-8') as _f:
    _STUNTS = aerolisp.compile_manifest(_f.read())['stunts']
KNIFE_PASS, ROLLING_HARRIER = _STUNTS['knife_pass'], _STUNTS['rolling_harrier']

RUNWAY_RAD = math.radians(50.0)
#: The runway's strip, m along its line from its end: where a landing stops. To 16 m, its
#: take-off's length, a landing in the gusty kind's 1 m/s tail wind on the floor rolled past it in
#: 10 of 13: a 9-13 m float and 32 m on its wheels from 12.2 m/s.
RUNWAY_ALONG = (-20.5, 30.0)
#: Its lap at SPEED_M_S: at 20 m/s its 17x8 on a pack sagged to 48 V, its top 600 rad/s and its pitch's
#: speed 20 m/s, could not hold it and hung nose-high at 4-9 m/s into the slalom's pylons.
SPEED_M_S, TURN_M, HIGH_M, LOW_M, TOP_M, LAPS = 16.0, 12.0, 3.0, 1.2, 10.0, 2
#: The point on the line it turns for, s ahead over the floor (L1, ArduPlane's), its rate at the
#: most; the height and the knife read so far ahead, s. The line's tangent crabbed onto it over
#: a second (`wingborne.lined`) put it 3.9 m off the hoop in gusts.
L1_S, TURN_RAD_S, LEAD_S = 0.7, math.radians(120.0), (0.8, 0.5)
#: Faster on its side, m/s: at 14 m/s in gusts its wings swung 34 deg off upright in the gate.
KNIFE_M_S = 4.0
#: A pylon's side and the knife gate's width, m: an air race's, twice the wings, taken on its
#: side by the rule - at 1.5 m, narrower than them, gusts put it 1.6 m off in knife-edge.
PYLON_M, SLOT_M = 0.4, 3.2
#: Where the lap starts, (along, across) the runway from its end: the near turn's exit.
START = (-110.0, 0.0)
#: The slalom, (deg, m) turned, + toward the far side: its peaks 4 m either side of its pylons.
WEAVE = ((-30.0, 30.0), (60.0, 30.0), (-60.0, 30.0), (60.0, 30.0), (-30.0, 30.0))
#: The lap: metres straight, or (deg, m) turned.
LEGS = (219.0, (180.0, TURN_M), 8.0) + WEAVE + (91.0, (180.0, TURN_M))


def placed(along, across):
    """World (x, z) of a point `along` the runway from its end and `across` it, m."""
    s, c = math.sin(RUNWAY_RAD), math.cos(RUNWAY_RAD)
    return along * s + across * c, along * c - across * s


def _laid(start=START, legs=LEGS):
    """A line from `start` up the runway's way a metre apart, [(along, across, heading rad off
    the runway's, 1/m turned)], `legs` metres straight or (deg, m) turned; where each begins."""
    a, c, psi, out, starts = start[0], start[1], 0.0, [], []
    for leg in legs:
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
S_GATE, S_LIMBO = 154.0, 179.0
S_HOOP = _STARTS[1] + 0.5 * math.pi * TURN_M
S_PYLONS = tuple(_STARTS[k] + 0.5 * (_STARTS[k + 1] - _STARTS[k]) for k in (4, 5, 6))
S_SLOT = _STARTS[8] + 30.0


def _bump(d, w):
    return 0.5 * (1.0 + math.cos(math.pi * d / w)) if abs(d) < w else 0.0


#: The lap's line: [(along, across, heading, 1/m turned, m high, knife)] a metre apart.
TRACK = tuple(p + (HIGH_M - (HIGH_M - LOW_M) * _bump(s - S_LIMBO, 30.0) + (TOP_M - HIGH_M) * _bump(s - S_HOOP, 55.0),
                   1.0 if abs(s - S_SLOT) <= 8.0 else _bump(abs(s - S_SLOT) - 8.0, 22.0))
              for s, p in enumerate(_LAP))


def _ramp(x):
    return 0.5 * (1.0 - math.cos(math.pi * min(1.0, max(0.0, x))))


#: The routine's opening off the runway, a line as the lap is: from LIFT_M up its line RUN_M
#: to OPEN_M_S, sharp right a quarter turn straight into a sharp left three quarters round on
#: OPENING_M turns - onto the runway's line again 2 OPENING_M on -, then PASS_M down it, down
#: to KNIFE_M and rolled onto its side over DESCENT_M at PASS_M_S, its top to the spectator
#: (the other way it showed them its belly), rolled back and up over its last PULL_M; a
#: teardrop to the far side on OPENING_M turns, its leg twice them so it closes on the line,
#: and RUN_IN_M up it slowing to RUN_IN_M_S, before the spectator. Flown as timed turns the
#: same figure ended 22 m beside the runway at 6 m/s, the teardrop 54 m out and 19 beside it;
#: turned off the lift at 13 m/s on 8 m it bled to 3.6; the 1.4 m wing ran wide of 9 m turns.
#: Its tip 4 cm over the runway on its side: at 0.45 m it struck.
LIFT_M, RUN_M, OPENING_M, OPEN_M_S = 20.0, 12.0, 11.0, 18.0
PASS_M, PASS_M_S, KNIFE_M, DESCENT_M, PULL_M = 74.0, KNIFE_PASS['speed'], KNIFE_PASS['height'], 12.0, 15.0
HARRIER_M, HARRIER_H, HARRIER_M_S, HARRIER_ROLL = (
    38.0, ROLLING_HARRIER['height'], ROLLING_HARRIER['speed'], ROLLING_HARRIER['roll'])
RUN_IN_M, RUN_IN_M_S = 54.0, 12.0
#: Gusty, the opening's pass higher by the law's gusts times GUST_S, m (`aerobat.law.GUST_S`).
GUST_S = 0.6
_OPEN, _OPEN_AT = _laid((LIFT_M, 0.0), (RUN_M, (-90.0, OPENING_M), (270.0, OPENING_M), PASS_M,
                                        (-45.0, OPENING_M), 2.0 * OPENING_M, (225.0, OPENING_M),
                                        RUN_IN_M))
_SIDE = tuple(_ramp((s - _OPEN_AT[3]) / DESCENT_M) - _ramp((s - _OPEN_AT[4] + PULL_M) / PULL_M)
              for s in range(len(_OPEN)))
_HARR_S0 = _OPEN_AT[7] + 10.0
_HARR = tuple(_ramp((s - _OPEN_AT[7]) / 12.0) for s in range(len(_OPEN)))
OPENING = tuple(p + (HIGH_M - (HIGH_M - KNIFE_M) * k - (HIGH_M - HARRIER_H) * h, -k)
                for p, k, h in zip(_OPEN, _SIDE, _HARR))


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
MASTS, HALLS, PADS = (), (), ()

#: Its lean the wing's, m/s^2: 12 at 14 m/s is 10.5 deg; the hover's 8 left 0.7 in a turn.
CRUISE = dict(HOVER, height=HIGH_M, pace=5.0, speed=SPEED_M_S, home=0.0, lean=50.0)
FINAL = dict(CRUISE, height=0.22, speed=11.5, pace=1.8)
#: The opening's lean its carbon airframe's 15 g; what a turn takes short of it is its wing's,
#: short of the stall (`aerobat.law`). On 15 g the course's laps left it into the floor on its
#: final, 36 m on.
G15_M_S2 = 15.0 * 9.81


def solids():
    """The field as the plane's world has it (`grounds.solids`)."""
    import sys
    return grounds.solids(sys.modules[__name__])


def track():
    """{'at': [(x, y, z)]}: the lap's line a metre apart at its height, as `course.track` gives its line."""
    return {'at': [(x, h, z) for a, c, _psi, _bend, h, _knife in TRACK for x, z in [placed(a, c)]]}


def _at(track, s, round_=False):
    """Linearly interpolated point along `track` at continuous arc-length coordinate `s`."""
    n = len(track)
    s = s % n if round_ else max(0.0, min(n - 1.0, s))
    i = min(int(s), n - 1)
    f = s - i
    p, q = track[i], track[(i + 1) % n if round_ else min(n - 1, i + 1)]
    return tuple(a + f * (b - a) for a, b in zip(p, q))


def line(route, now):
    """The lap's row for now (`aerobatics.fly`'s giver): on its wing at SPEED_M_S at the height
    and the knife the line asks LEAD_S ahead, turned for its point L1_S ahead as `2 sin(eta) /
    L1_S`, eta its way over the floor off it, TURN_RAD_S at the most; a lap at its end, LAPS
    laps, `lapped` there."""
    fresh = {'began': route['at'], 'k': None, 'laps': 0, 'gate': 0, 'of': LAPS}
    lap = route.get('lap', fresh)
    lap = route['lap'] = (lap, fresh)[int(lap['began'] != route['at'])]
    k, s, rate = _followed(TRACK, lap['k'], route.get('seen', {}), True)
    lap.update(laps=lap['laps'] + int(lap['k'] is not None and k < lap['k']), k=k)
    lap['gate'] = next((g for g, at in enumerate(S_GATES) if at > k), 0)
    route['holds'] = (route.get('holds', ()), ('lapped',))[int(lap['laps'] >= LAPS)]
    knife = _at(TRACK, s + LEAD_S[1] * SPEED_M_S, True)[5]
    return dict(CRUISE, turn=math.degrees(rate), height=_at(TRACK, s + LEAD_S[0] * SPEED_M_S, True)[4],
                knife=knife, speed=SPEED_M_S + KNIFE_M_S * knife, lined=0.0)


def _followed(track, k, seen, round_):
    """(the point of `track` nearest the frame from `k` on - none, anywhere -, its continuous
    arc-length coordinate `s`, and the turn for its point L1_S ahead as `2 sin(eta) / L1_S`):
    round its end where it is `round_`, else held."""
    x, _y, z = seen.get('at', (0.0, 0.0, 0.0))
    vel, heading = seen.get('vel', (0.0, 0.0, 0.0)), seen.get('heading', RUNWAY_RAD)
    s, c = math.sin(RUNWAY_RAD), math.cos(RUNWAY_RAD)
    a, b, n = x * s + z * c, x * c - z * s, len(track)
    near = range(n) if k is None else range(k, k + 40) if round_ else range(k, min(n, k + 40))
    k = min(near, key=lambda i: (track[i % n][0] - a) ** 2 + (track[i % n][1] - b) ** 2) % n
    ds = max(-0.5, min(0.5, (a - track[k][0]) * math.cos(track[k][2]) + (b - track[k][1]) * math.sin(track[k][2])))
    fast = math.hypot(vel[0], vel[2])
    px, pz = placed(*_at(track, k + ds + max(4.0, L1_S * fast), round_)[:2])
    eta = (math.atan2(px - x, pz - z) - (math.atan2(vel[0], vel[2]) if fast > 1.0 else heading)
           + math.pi) % math.tau - math.pi
    return k, k + ds, max(-TURN_RAD_S, min(TURN_RAD_S, 2.0 * math.sin(eta) / L1_S))


def _open_step(route, start, end, hold):
    fresh = {'began': route['at'], 'k': start}
    run = route.get('open', fresh)
    run = route['open'] = (run, fresh)[int(run['began'] != route['at'])]
    k, s, rate = _followed(OPENING, run['k'], route.get('seen', {}), False)
    run['k'] = k
    route['holds'] = (route.get('holds', ()), (hold,))[int(k >= end)]
    knife = _at(OPENING, s + LEAD_S[1] * PASS_M_S, False)[5]
    roll = HARRIER_ROLL * float(_HARR_S0 <= s <= _HARR_S0 + HARRIER_M)
    slow = (OPEN_M_S - HARRIER_M_S) * _ramp((s - _HARR_S0) / 18.0)
    lead_h = LEAD_S[0] * (OPEN_M_S - slow)
    # Gusty, its pass higher by its law's gusts (`aerokin.seen`): at KNIFE_M on the 17x8 the
    # page's tours sank it onto its tip 15 times in 47 crashes.
    lift = GUST_S * route.get('seen', {}).get('gust', 0.0)
    return dict(CRUISE, turn=math.degrees(rate),
                height=_at(OPENING, s + lead_h, False)[4] + lift,
                knife=knife, roll=roll, speed=OPEN_M_S + (PASS_M_S - OPEN_M_S) * abs(knife) - slow,
                light=0.1, lined=0.0, lean=G15_M_S2)


def opening(route, now):
    """The routine's knife-edge pass (`aerobatics.fly`'s giver): OPENING's line from the runway
    through the teardrop and the knife-edge pass (`knife_pass`) down the runway and round the
    far turnaround to `_OPEN_AT[7]`; `knifed` there."""
    return _open_step(route, 0, _OPEN_AT[7], 'knifed')


def rolling_harrier(route, now):
    """The routine's low rolling harrier pass (`aerobatics.fly`'s giver): OPENING's line from
    `_OPEN_AT[7]` up the runway in a slow high-alpha rolling harrier (`rolling_harrier`);
    `passed` at its end."""
    return _open_step(route, _OPEN_AT[7], len(OPENING) - 2, 'passed')


#: The circuit, a line as the lap is: from the lap's START up the runway's line to TURN_AT_M,
#: round to its near side on CIRCUIT_M, down it and round onto its line at BASE_M, CIRCUIT_H up
#: at CIRCUIT_M_S, slowed SLOW_M_S2 to it before its first turn; the routine joins it off its hold.
#: As timed legs it came down 25-37 m beside the runway in gusts, or into the house. On 12 m turns
#: at 12 m/s - 12.3 deg of the stall's 15 - it stalled in gusts, the course in its last near turn
#: in all of eight; at 14 m/s a tip struck lining up low. The far side's gates stood 1.3 m off.
#: At 1.8 m its final came in on a 2 deg glide.
TURN_AT_M, CIRCUIT_M, BASE_M, CIRCUIT_H, CIRCUIT_M_S = 20.0, 20.0, -70.0, 3.0, 12.0
CIRCUIT = tuple(p + (CIRCUIT_H, 0.0) for p in _laid(
    START, (TURN_AT_M - START[0], (-180.0, CIRCUIT_M), TURN_AT_M - BASE_M, (-180.0, CIRCUIT_M)))[0])
_TURN_K = round(TURN_AT_M - START[0])
#: The final down the runway's line from where the circuit leaves it: on a glide from CIRCUIT_H
#: there to TOUCH_M along, flared where the glide is FLARE_H up. Flown on its heading for its
#: seconds it touched 15-62 m short of the runway, 0.3-4.8 m off its line.
TOUCH_M, FLARE_H = -20.0, 0.22
GLIDE = CIRCUIT_H / (TOUCH_M - BASE_M)
#: Its speed down to FINAL's where it flares, slowed SLOW_M_S2 a second - its idle drag's 1.7-2.9
#: at 11.5-15 m/s -, the one it came on with at the most. Asked FINAL's at once off the lap's 18
#: m/s it pitched up to slow, ballooned to 8 m and came down at 1.7 m/s into its flare; slowed as
#: its drag does, 0.01 of its speed a metre, it held 20 m/s first and touched 15-35 m on.
SLOW_M_S2 = 1.5
_FLARE_AT = TOUCH_M - FLARE_H / GLIDE
APPROACH = tuple(p + (min(HIGH_M, max(0.0, GLIDE * (TOUCH_M - p[0]))), 0.0)
                 for p in _laid((-130.0, 0.0), (160.0,))[0])


def circuit(route, now):
    """The circuit's row for now (`aerobatics.fly`'s giver): CIRCUIT flown as the lap is
    (`_followed`) from where the frame joins it, slowed from the speed it came on with to
    CIRCUIT_M_S at its first turn; `circled` at its end, on the runway's line."""
    seen = route.get('seen', {})
    fresh = {'began': route['at'], 'k': None, 'v': min(SPEED_M_S, math.hypot(*seen.get('vel', (0.0, 0.0, 0.0))[::2]))}
    run = route.get('circle', fresh)
    run = route['circle'] = (run, fresh)[int(run['began'] != route['at'])]
    k, s, rate = _followed(CIRCUIT, run['k'], seen, False)
    run['k'], n = k, len(CIRCUIT)
    route['holds'] = (route.get('holds', ()), ('circled',))[int(k >= n - 2)]
    speed = max(CIRCUIT_M_S, min(run['v'], math.sqrt(CIRCUIT_M_S ** 2 + 2.0 * SLOW_M_S2 * max(0.0, _TURN_K - s))))
    return dict(CRUISE, turn=math.degrees(rate), height=CIRCUIT_H, speed=speed, lined=0.0)


def approach(route, now):
    """The final's row for now (`aerobatics.fly`'s giver): APPROACH flown as the lap is from where
    the frame joins it, on its glide where it is - its slope the climb -; `flared` where the
    glide is FLARE_H. Its height LEAD_S ahead brought it down at 1.3 m/s into the flare."""
    seen = route.get('seen', {})
    v_g = math.hypot(*seen.get('vel', (0.0, 0.0, 0.0))[::2])
    h = seen.get('at', (0.0, 1.0, 0.0))[1] - 0.28
    fresh = {'began': route['at'], 'k': None, 'v': min(CIRCUIT_M_S, v_g)}
    run = route.get('glide', fresh)
    run = route['glide'] = (run, fresh)[int(run['began'] != route['at'])]
    k, s, rate = _followed(APPROACH, run['k'], seen, False)
    pt = _at(APPROACH, s, False)
    run['k'], here = k, pt[4]
    route['holds'] = (route.get('holds', ()), ('flared',))[int(here <= FLARE_H or (here < 1.0 and h <= FLARE_H))]
    left = max(0.0, _FLARE_AT - pt[0])
    speed = max(FINAL['speed'], min(run['v'], math.sqrt(FINAL['speed'] ** 2 + 2.0 * SLOW_M_S2 * left)))
    sloped = float(FLARE_H < here < HIGH_M) * min(1.0, max(0.0, (h - FLARE_H) / 0.15))
    return dict(FINAL, turn=math.degrees(rate), height=max(FLARE_H, here), speed=speed,
                climb=-sloped * GLIDE * max(speed, v_g))


#: Where along the lap each gate stands, m.
S_GATES = (S_GATE, S_LIMBO, S_HOOP) + S_PYLONS + (S_SLOT,)

#: The course's flight: off the runway into its laps and round its circuit back onto it.
CARD = (
    ('idle', DOWN, 2.0, 0.0, 'fit'),
    ('spool', PRESSED, 2.5, 2.5, ''),
    ('lap', line, 0.0, 0.0, 'lapped'),
    ('circuit', circuit, 0.0, 0.0, 'circled'),
    ('final', approach, 0.0, 0.0, 'flared'),
    ('land', PRESSED, 10.0, 0.0, ''),
)
