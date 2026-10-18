"""The aerobat's airfield: its runway, a gate course laid for a plane, what stands about them.

    route = aerobatics.routine(airfield.CARD)            # off the runway, its laps, back onto it
    name, row = aerobatics.fly(route, now, holds)        # `line` gives the lap's rows
    sky = aerobat.Sky(airfield.solids())                 # the field as the plane's world has it

The runway ends at the middle of the world on RUNWAY_RAD. The course is a plane's: arches
GATE_M square on its straights, a pylon in each turn of TURN_M at SPEED_M_S, a run-up of 20 m
past the runway's end into its straightest leg, a slalom of three on the back straight, the
lap closed down the runway's line. Its gates stand only while it is flown (`quad.Sky.stand`);
what stands always is clear of every line the routine and the course fly.
"""
import math

from machine import grounds
from machine.aerobatics import DOWN, HOVER, PRESSED
from machine.wingborne import lined

RUNWAY_RAD = math.radians(50.0)
GATE_M, SPEED_M_S, TURN_M, HIGH_M, LAPS = 6.0, 12.0, 22.0, 3.0, 2
#: The heading's loop, 1/s, and its rate at the most: 35 deg/s holds 12 m/s within lean 8.
AIM_K, TURN_RAD_S = 3.0, math.radians(35.0)


def placed(along, across):
    """World (x, z) of a point `along` the runway from its end and `across` it, m."""
    s, c = math.sin(RUNWAY_RAD), math.cos(RUNWAY_RAD)
    return along * s + across * c, along * c - across * s


def _bend(middle, way):
    """A turn's point at the heading `way`, deg off the runway's, about its `middle`."""
    w = math.radians(way)
    return middle[0] + TURN_M * math.sin(w), middle[1] - TURN_M * math.cos(w), way


#: The lap's lines: (along, across, deg off the runway's heading, a gate's number or None) -
#: the gate short of the runway, two down the run-up's line, a turn about BENDS[0], the
#: slalom, a turn about BENDS[1]; the last its exit onto the runway's line, where the landing
#: takes it. A gate at a turn's apex took a tip in gusty wind: the downwind half swings wide.
BENDS = ((80.0, 22.0), (-50.0, 22.0))
WAYS = ((-35.0, 0.0, 0.0, 0), (20.0, 0.0, 0.0, 1), (55.0, 0.0, 0.0, 2)) + tuple(
    _bend(BENDS[0], w) + (None,) for w in (45.0, 90.0, 135.0, 180.0)) + (
    (50.0, 47.0, 180.0, 3), (10.0, 41.0, 180.0, 4), (-30.0, 47.0, 180.0, 5)) + tuple(
    _bend(BENDS[1], w) + (None,) for w in (180.0, 225.0, 270.0, 315.0, 360.0))
GATES = tuple(placed(a, c)[:1] + (GATE_M / 2,) + placed(a, c)[1:] + (math.degrees(RUNWAY_RAD) + w, GATE_M)
              for a, c, w, g in WAYS if g is not None)

#: What stands always: a tree line down the far side and across the far end, (x, z, m high,
#: crown m); the hangar by the runway's start, (x, z, wide, deep, wall, ridge); two cars by it;
#: a pylon in each turn of the course, (x, z, side, m high).
TREES = tuple(placed(a, c) + (h, 1.4) for a, c, h in (
    [(a, -60.0, 7.0 + (a % 3)) for a in range(-100, 101, 25)]
    + [(140.0, c, 8.0) for c in (-30.0, 0.0, 30.0, 60.0)] + [(a, 80.0, 6.5) for a in (-60.0, 0.0, 60.0)]))
HOUSES = (placed(-12.0, -22.0) + (12.0, 10.0, 4.0, 5.5),)
CARS = tuple(placed(a, c) + (math.degrees(RUNWAY_RAD),) for a, c in ((-3.0, -19.0), (0.5, -19.5)))
MASTS = tuple(placed(*bend) + (0.4, 8.0) for bend in BENDS)
HALLS = ()

CRUISE = dict(HOVER, height=HIGH_M, pace=3.0, speed=SPEED_M_S, home=0.0)
FINAL = dict(CRUISE, height=0.22, speed=11.5, pace=1.8)


def solids():
    """The field as the plane's world has it (`grounds.solids`)."""
    import sys
    return grounds.solids(sys.modules[__name__])


def track():
    """{'at': [(x, y, z)]}: the lap's ways joined a metre apart, as `course.track` gives its line."""
    ends, out = [placed(a, c) for a, c, _w, _g in WAYS], []
    for (x0, z0), (x1, z1) in zip(ends, ends[1:] + ends[:1]):
        n = max(1, int(math.hypot(x1 - x0, z1 - z0)))
        out += [(x0 + (x1 - x0) * k / n, HIGH_M, z0 + (z1 - z0) * k / n) for k in range(n)]
    return {'at': out}


def line(route, now):
    """The lap's row for now (`aerobatics.fly`'s giver): on its wing at SPEED_M_S onto the line
    of the way it flies to next (`wingborne.lined`), turned at AIM_K to TURN_RAD_S at the most;
    a way passed once its plane is behind it, a lap at the last; LAPS laps, `lapped` there."""
    fresh = {'began': route['at'], 'k': 0, 'laps': 0, 'gate': 0, 'of': LAPS}
    lap = route.get('lap', fresh)
    lap = route['lap'] = (lap, fresh)[int(lap['began'] != route['at'])]
    seen = route.get('seen', {})
    x, y, z = seen.get('at', (0.0, 0.0, 0.0))
    vel, heading = seen.get('vel', (0.0, 0.0, 0.0)), seen.get('heading', RUNWAY_RAD)
    a, c, w, _g = WAYS[lap['k']]
    wx, wz = placed(a, c)
    way = RUNWAY_RAD + math.radians(w)
    passed = int((x - wx) * math.sin(way) + (z - wz) * math.cos(way) > 0.0)
    lapped = passed * int(lap['k'] == len(WAYS) - 1)
    lap.update(k=(lap['k'] + passed) % len(WAYS), laps=lap['laps'] + lapped)
    lap['gate'] = next(g for k in range(len(WAYS)) for g in [WAYS[(lap['k'] + k) % len(WAYS)][3]]
                       if g is not None)
    route['holds'] = (route.get('holds', ()), ('lapped',))[int(lap['laps'] >= LAPS)]
    a, c, w, _g = WAYS[lap['k']]
    wx, wz = placed(a, c)
    aim = lined(RUNWAY_RAD + math.radians(w), None, vel, (x - wx, y, z - wz), y, SPEED_M_S)
    miss = (aim - heading + math.pi) % math.tau - math.pi
    return dict(CRUISE, turn=math.degrees(max(-TURN_RAD_S, min(TURN_RAD_S, AIM_K * miss))))


#: The course's flight: off the runway into its laps and back onto it.
CARD = (
    ('idle', DOWN, 2.0, 0.0, 'fit'),
    ('spool', PRESSED, 2.5, 2.5, ''),
    ('lap', line, 0.0, 0.0, 'lapped'),
    ('final', FINAL, 2.4, 0.5, ''),
    ('land', PRESSED, 8.0, 0.0, ''),
)
