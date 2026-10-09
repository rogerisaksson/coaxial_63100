"""The quad's grounds: its gates in the air and what stands about them.

Solid as the frame's world has them (`solids`), and how near a frame's middle comes to each
(`clearances`).

    sky = quad.Sky(grounds.solids())                 # the world the frame flies into
    near = grounds.clearances(points)                # [(what, m)], the nearest first
"""
import math

#: A gate: its middle (x, y, z), m from where the frame rises, y up; its heading, degrees from
#: z toward x - flown through along it; its opening, m square, the floor under it at the
#: lowest. The first stands over the spot, the lap's start and its finish, the floor its lower
#: edge. Then the slalom's four, narrow, weaving between trees; up round the bend to the high
#: gate over the hedge, 9.4 m higher 15 m on; along the top beside the mast and the dive, 10 m
#: down 13 m on; into the hall by its north window and out by its south, a metre across; the
#: alley between the two sheds, 3 m wide; the low bend west; the second slalom between the
#: cars; up round the last bend and down home. A gate in a hall's wall is one of its windows:
#: the wall its frame.
GATE_M = 3.4
GATES = ((0.0, 1.7, 0.0, 0.0, 3.4), (1.8, 2.0, 10.0, 0.0, 2.8), (-1.8, 2.2, 18.0, 0.0, 2.8),
         (1.8, 2.4, 26.0, 0.0, 2.8), (-1.8, 2.6, 34.0, 0.0, 2.8),
         (10.0, 12.0, 44.0, 90.0, 3.4), (24.0, 13.0, 44.0, 90.0, 3.4),
         (35.5, 2.8, 38.0, 180.0, 3.0), (34.5, 2.6, 30.0, 180.0, 2.6),
         (33.5, 2.4, 18.0, 180.0, 2.6), (33.5, 2.2, 9.0, 180.0, 2.6),
         (30.0, 1.4, -5.0, 225.0, 3.0), (22.0, 2.0, -9.0, 270.0, 2.8),
         (15.0, 2.4, -13.0, 270.0, 2.8), (8.0, 2.0, -9.0, 270.0, 2.8),
         (2.0, 4.2, -5.5, 315.0, 3.4))

#: What stands about the line. A tree: (x, z, its height, its crown's radius), m. A house: (x, z,
#: its width along x, its depth along z, its walls' height, its ridge's), the ridge along x; a
#: hall the same, hollow, its walls WALL_M thick and open where a gate stands in one. A car:
#: (x, z, its heading, degrees), CAR_M wide, high and long. A mast: (x, z, its side, its height).
TREES = ((-3.4, 10.0, 6.5, 1.2), (3.4, 18.0, 7.0, 1.3), (-3.4, 26.0, 6.5, 1.2),
         (3.6, 34.0, 7.5, 1.3), (6.5, 38.0, 10.0, 1.8), (13.0, 39.0, 9.5, 1.7),
         (-8.0, 40.0, 9.0, 2.0), (22.0, -4.5, 6.5, 1.5), (15.0, -17.5, 7.0, 1.5),
         (8.0, -4.5, 6.5, 1.5), (6.0, -1.5, 7.5, 1.5), (-12.0, 18.0, 7.5, 1.7),
         (48.0, 30.0, 8.5, 2.0), (18.0, 24.0, 8.0, 1.9))
HOUSES = ((29.5, 9.0, 5.0, 8.0, 3.0, 4.4), (37.5, 9.0, 5.0, 8.0, 3.0, 4.4),
          (-14.0, 4.0, 6.0, 8.0, 2.8, 4.4))
HALLS = ((34.0, 24.0, 10.0, 12.0, 5.0, 7.5),)
CARS = ((26.0, -14.5, 10.0), (18.5, -5.0, 85.0), (11.5, -16.5, 95.0), (40.0, 0.0, 0.0),
        (-6.0, 22.0, 20.0), (44.0, 16.0, 5.0))
CAR_M = (1.8, 1.4, 4.4)
MASTS = ((42.0, 44.0, 1.6, 16.0),)

#: Their shapes: a tree's crown a six-sided cone from CROWN of its height up, on a trunk
#: TRUNK_M thick; a car's body from CAR_LOW m up to CAR_BODY of its height, its cabin on it
#: CAR_CABIN of its width and of its length; a gate's bars and posts BAR_M thick; a hall's
#: walls WALL_M, m.
CROWN, TRUNK_M, CAR_LOW, CAR_BODY, CAR_CABIN, BAR_M, WALL_M = (0.35, 0.12, 0.25, 0.55,
                                                               (0.9, 0.5), 0.05, 0.2)

#: The grounds' lengths, and what is placed: for a frame of another size as much larger (`sized`).
_UNITS = {'GATE_M': (1, 0), 'TRUNK_M': (1, 0), 'BAR_M': (1, 0), 'WALL_M': (1, 0)}
_PLACED = {'GATES': (3,), 'TREES': (), 'HOUSES': (), 'HALLS': (), 'CARS': (2,), 'CAR_M': None,
           'MASTS': ()}
_BUILT, _SET = {}, {}


def sized():
    """The grounds for the frame as `quad.sized` has it: each gate and thing where and as
    large - a heading the same."""
    from machine import quad
    scope, size = globals(), quad.scales()[0]
    if not _SET:
        _SET.update({name: scope[name] for name in _PLACED})
    quad.rescaled(scope, _UNITS, _BUILT)
    for name, angles in _PLACED.items():
        scope[name] = tuple(size * v for v in _SET[name]) if angles is None else tuple(
            tuple(v if k in angles else size * v for k, v in enumerate(row)) for row in _SET[name])


def gate(x, y, z, heading, size=None):
    """A gate's edges, [(an end, the other)]: its opening's frame, `size` square to its
    heading, the floor under it at the lowest, on two posts."""
    c, s = math.cos(math.radians(heading)), math.sin(math.radians(heading))
    half = (GATE_M if size is None else size) / 2
    frame = [(x + a * c, max(0.0, y + b), z - a * s)
             for a, b in ((-half, -half), (half, -half), (half, half), (-half, half))]
    return list(zip(frame, frame[1:] + frame[:1])) + [(p, (p[0], 0.0, p[2])) for p in frame[:2]]


def walls(hall):
    """A hall's walls as they stand, less its windows: [((x, y, z) middle, (x, y, z) half
    sizes)] - the north and south along x, the east and west along z - and its windows, the
    gates standing in them, by number."""
    x, z, wide, deep, wall, _ridge = hall
    out, windows = [], []
    for along, at, lo, hi in ((0, z + deep / 2, x - wide / 2, x + wide / 2),
                              (0, z - deep / 2, x - wide / 2, x + wide / 2),
                              (2, x + wide / 2, z - deep / 2, z + deep / 2),
                              (2, x - wide / 2, z - deep / 2, z + deep / 2)):
        across = 2 - along
        holes = [(g, place) for g, place in enumerate(GATES)
                 if abs(place[across] - at) <= WALL_M and lo < place[along] < hi]
        windows += [g for g, _place in holes]
        cuts = sorted({lo, hi} | {place[along] + k * place[4] / 2 for _g, place in holes
                                  for k in (-1, 1)})
        for a, b in zip(cuts, cuts[1:]):
            hole = next((p for _g, p in holes if abs((a + b) / 2 - p[along]) < p[4] / 2), None)
            spans = [(0.0, wall)] if hole is None else [(0.0, hole[1] - hole[4] / 2),
                                                        (hole[1] + hole[4] / 2, wall)]
            for low, high in spans:
                if high > low and b > a:
                    middle = [0.0, (low + high) / 2, 0.0]
                    half = [WALL_M / 2, (high - low) / 2, WALL_M / 2]
                    middle[along], middle[across] = (a + b) / 2, at
                    half[along] = (b - a) / 2
                    out.append((tuple(middle), tuple(half)))
    return out, windows


def windows():
    """Every hall's windows, by gate number."""
    return {g for hall in HALLS for g in walls(hall)[1]}


def _roof(x, z, wide, deep, wall, ridge):
    return [(x + a * wide / 2, wall, z + b * deep / 2) for a in (-1, 1) for b in (-1, 1)] + [
        (x + a * wide / 2, ridge, z) for a in (-1, 1)]


def solids():
    """What stands, as the frame's world has it to fly into (`quad.mjcf`): a tree its trunk and
    its cone, a house its walls and its roof, a hall its walls about its windows and its roof,
    a car its body and its cabin, a mast; a gate (`gateN`) its bars and its posts - none along
    the floor, where the floor is its lower edge, none at all where it is a window."""
    out = []
    for x, z, high, crown in TREES:
        foot = CROWN * high
        out += [('rod', 'tree', (x, 0.0, z), (x, foot, z), TRUNK_M),
                ('hull', 'tree', [(x + crown * math.cos(k * math.tau / 6), foot,
                                   z + crown * math.sin(k * math.tau / 6)) for k in range(6)]
                 + [(x, high, z)])]
    for x, z, wide, deep, wall, ridge in HOUSES:
        out += [('box', 'house', (x, wall / 2, z), (wide / 2, wall / 2, deep / 2), 0.0),
                ('hull', 'house', _roof(x, z, wide, deep, wall, ridge))]
    for hall in HALLS:
        out += [('box', 'house', middle, half, 0.0) for middle, half in walls(hall)[0]]
        out.append(('hull', 'house', _roof(*hall)))
    wide, high, long_ = CAR_M
    for x, z, heading in CARS:
        out += [('box', 'car', (x, (CAR_LOW + CAR_BODY * high) / 2, z),
                 (wide / 2, (CAR_BODY * high - CAR_LOW) / 2, long_ / 2), heading),
                ('box', 'car', (x, (1.0 + CAR_BODY) * high / 2, z),
                 (CAR_CABIN[0] * wide / 2, (1.0 - CAR_BODY) * high / 2,
                  CAR_CABIN[1] * long_ / 2), heading)]
    for x, z, side, high in MASTS:
        out.append(('box', 'mast', (x, high / 2, z), (side / 2, high / 2, side / 2), 0.0))
    framed = windows()
    for k, place in enumerate(GATES):
        if k not in framed:
            out += [('rod', 'gate%d' % k, a, b, BAR_M) for a, b in gate(*place)
                    if max(a[1], b[1]) > 0.0]
    return out


def clearances(points):
    """[(what, m)], the nearest first: how near any of the (x, y, z) `points` - a frame's
    middle - came to each tree, house, hall's wall or roof, car and mast: beside it or over
    it, whichever is more."""
    out = []
    for x, z, high, crown in TREES:
        out.append(('the tree at (%g, %g)' % (x, z), min(
            (math.hypot(p[0] - x, p[2] - z) - crown for p in points if p[1] <= high),
            default=math.inf)))
    for x, z, wide, deep, _wall, ridge in HOUSES:
        out.append(('the house at (%g, %g)' % (x, z), min(
            max(abs(p[0] - x) - wide / 2, abs(p[2] - z) - deep / 2, p[1] - ridge)
            for p in points)))
    for hall in HALLS:
        x, z, wide, deep, wall, ridge = hall
        boxes = walls(hall)[0]
        out.append(('the hall at (%g, %g)' % (x, z), min(min(
            [max(abs(p[k] - m[k]) - h[k] for k in range(3)) for m, h in boxes]
            + [max(abs(p[0] - x) - wide / 2, abs(p[2] - z) - deep / 2, wall - p[1],
                   p[1] - ridge)]) for p in points)))
    wide, high, long_ = CAR_M
    for x, z, heading in CARS:
        c, s = math.cos(math.radians(heading)), math.sin(math.radians(heading))
        out.append(('the car at (%g, %g)' % (x, z), min(
            max(abs((p[0] - x) * c - (p[2] - z) * s) - wide / 2,
                abs((p[0] - x) * s + (p[2] - z) * c) - long_ / 2, p[1] - high)
            for p in points)))
    for x, z, side, high in MASTS:
        out.append(('the mast at (%g, %g)' % (x, z), min(
            max(math.hypot(p[0] - x, p[2] - z) - side, p[1] - high) for p in points)))
    return sorted(out, key=lambda pair: pair[1])
