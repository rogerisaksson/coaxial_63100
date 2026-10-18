"""The quad's aerobatics: a figure a row of setpoints (`machine.aerokin`'s), a routine a card of them.

    route = aerobatics.routine(card)                 # a card's table, at its first row
    name, row = aerobatics.fly(route, now, holds)    # the card's row for now, eased in
    payload = aerokin.step(law, row, frame, dt, pull)    # that row flown: aero/'s payload

Spent - its pack, its boards' envelopes - the routine comes down from wherever it is and waits
on the floor until it is fit.
"""
import math
from typing import Any

from machine import quad
from machine.quad import FLOOR_M, HOVER_M

#: A figure a row of setpoints: height, m over the floor, and the pace it may go there at, m/s,
#: come to and stopped from in a second; climb and push, that height's own rate and pull, m/s
#: and m/s^2 - a row's none, its easing's on the way to it; speed along its heading and slide
#: across it, m/s, surge and sway a pull along and across it over what those speeds' change
#: takes, m/s^2 - a line's bend, asked ahead of it; turn, its heading's, deg/s, and nose, how
#: firmly its nose is held on that heading, 0 to 1 - free, its lean alone turns with it; x and
#: z, its place, m from where it rose, and home, how much its spot goes there, 0 to 1; lean,
#: the pull along the floor it may take, m/s^2 - 39 degrees in a hover; roll and flip,
#: turns/s about its nose and about its wing; light, the least share of gravity its discs lean
#: against, under none pointed down - half in a figure: leant against all of it while asked to
#: fall, the thrust cut for the fall took the pull along the floor with it, over a crest at 0.3
#: of its weight a bend had 0.6 of its pull and the frame ran 1.2 m wide.
_ST, _KIN, _ENV_M = quad.STUNTS, quad.KINEMATICS, (FLOOR_M, HOVER_M)
HOVER = _ST['hover']
#: Full tilt: a height out of reach, at any pace.
SKY = _ST['sky']
#: The lift off the floor: the hover come to at any pace, on all the rotors have.
LAUNCH = _ST['launch']
#: The stop over the floor, from wherever it is: let fall, burned, held. It falls free, half
#: its weight on the rotors - 13 m/s, blown 10-13 m off in a gusty wind -, and its burn
#: begins the rotors' spool before the stop on all they have: at the mark in four winds of
#: four. And on the floor: let fall half a metre under it, the skids down and the rotors at
#: their idle.
STOP = _ST['stop']
DOWN = _ST['down']
#: A turn on its spot, as the discs' drag turns it.
PIROUETTE = _ST['pirouette']
#: An orbit: slid ORBIT_M_S round the point ORBIT_M ahead of its nose, its nose on it, banked 25
#: degrees, a lap LAP_S. Mixed with a hover it keeps its circle - both its slide and its turn
#: are as much of theirs - and eased in and out alike it is as many laps as its seconds.
ORBIT_M, ORBIT_M_S = _KIN['horizontal']['orbit_radius'], _KIN['horizontal']['orbit_speed']
ORBIT = _ST['orbit']
LAP_S = math.tau * ORBIT_M / ORBIT_M_S
#: The orbit climbed: a corkscrew to TOP_M; and its way back, down again.
TOP_M = _KIN['envelope']['top_height']
CORKSCREW = _ST['corkscrew']
UNWIND = _ST['unwind']
#: Over its spot at the top. A toss: the sky's row at a pace; at that pace it turns over, about
#: its nose or about its wing, and is caught where it was tossed from.
HIGH = _ST['high']
TOSS = _ST['toss']
ROLL = _ST['roll']
FLIP = _ST['flip']
#: Pressed on the floor, its height asked this far under it, m: the skids down, the rotors
#: bearing all but a twentieth of it - where a landing ends and a lift begins.
PRESSED = _ST['pressed']
#: Down to the stop's mark over its spot from wherever it is, at a pace.
OVER = _ST['over']

#: The routine: (name, row, seconds on it at least, seconds eased into it, what it waits for
#: before it leaves). `held`: the frame at its height and its spot, still, its turns whole;
#: `fit`: a pack with more than its reserve in it, the boards' envelopes with their room. A
#: row may be what gives one - `row(route, now)` - where its setpoints are not a figure's but
#: a line's (`machine.course`); what it holds itself it leaves in the route's `holds`.
CARD = quad.MANIFEST['routines']['quadcopter']
#: `spent` among what holds takes the routine to its flight's row of this name, the next before
#: one that waits to be fit - but from one that waits to be fit, on the floor already, and from
#: one that gives its own row: a line ends itself (`machine.course`: where it is on it, things
#: stand under it).
SPENT_TO = 'descend'


#: A row's setpoints that have a unit, each by the powers of its metres and its seconds, and
#: the routine's own: at a frame of another size they go by its size and its clock (`sized`).
KEYS = {'height': (1, 0), 'pace': (1, -1), 'climb': (1, -1), 'push': (1, -2), 'speed': (1, -1),
        'slide': (1, -1), 'surge': (1, -2), 'sway': (1, -2), 'turn': (0, -1), 'x': (1, 0),
        'z': (1, 0), 'lean': (1, -2), 'roll': (0, -1), 'flip': (0, -1)}
_UNITS = {'ORBIT_M': (1, 0), 'ORBIT_M_S': (1, -1), 'LAP_S': (0, 1), 'TOP_M': (1, 0),
          'FLOOR_M': (1, 0), 'HOVER_M': (1, 0)}
_BUILT, _ROWS, _CARD = {}, [], []


def mix(a, b, k):
    """The row `k` of the way from `a` to `b`."""
    return {name: a[name] + (b[name] - a[name]) * k for name in a}


def resized(card, rows):
    """`card`'s seconds and the setpoints of `rows` - [(a row, as built)] - for the frame as
    it is sized: the card."""
    size, clock = quad.scales()
    for row, built in rows:
        row.update({key: built[key] * size ** metres * clock ** seconds
                    for key, (metres, seconds) in KEYS.items()})
    return tuple((name, row, seconds * clock, over * clock, waits)
                 for name, row, seconds, over, waits in card)


def sized():
    """The routine for the frame as `quad.sized` has it: every row's setpoints and its card's
    seconds."""
    global CARD
    if not _ROWS:
        _ROWS.extend((row, dict(row)) for row in globals().values()
                     if isinstance(row, dict) and 'height' in row and row is not KEYS)
        _CARD.append(CARD)
    quad.rescaled(globals(), _UNITS, _BUILT)
    CARD = resized(_CARD[0], _ROWS)


def ease(x):
    """The share of a move made `x` of the way through its time, at rest and unpulled at both
    ends: the least jerk a move has."""
    x = max(0.0, min(1.0, x))
    return x * x * x * (10.0 - 15.0 * x + 6.0 * x * x)


def eased(was, row, into, seconds):
    """The row `into` s of the way from `was` to `row`, eased over `seconds` - none, it is
    there -, its height's own rate and pull on it."""
    over = max(seconds, 1e-12)
    x, span = max(float(seconds <= 0.0), min(1.0, into / over)), row['height'] - was['height']
    there = dict(mix(was, row, ease(x)), climb=span * 30.0 * x * x * (1.0 - x) ** 2 / over,
                 push=span * 60.0 * x * (1.0 - x) * (1.0 - 2.0 * x) / (over * over))
    return (there, dict(row))[int(x >= 1.0)]


def _way_down(card, row):
    """The SPENT_TO's row of the flight `row` is in and ahead of it, or None: a flight is the
    rows up to the next that waits to be fit. Sought to the card's end, a pack spent on the
    first flight's way down went on to the second's, and its flight was never flown."""
    for k in range(row + 1, len(card)):
        if 'fit' in card[k][4].split():
            return None
        if card[k][0] == SPENT_TO:
            return k
    return None


def laid(card):
    """A card as its table, a column a thing a pass asks of a row: its name; what gives its
    setpoints for now - a figure eased in from the row left, a line's own giver -; its
    seconds; what it waits for; the row a spent flight goes to from it, or none (-1): from a
    figure that does not wait to be fit."""
    figures = [isinstance(row, dict) for _name, row, *_rest in card]
    ways = [_way_down(card, k) for k in range(len(card))]
    return {'name': [name for name, *_rest in card],
            'give': [(row, lambda route, now, row=row, over=over: eased(
                route['was'], row, now - route['at'], over))[int(figure)]
                for (_name, row, _seconds, over, _waits), figure in zip(card, figures)],
            'seconds': [seconds for _name, _row, seconds, *_rest in card],
            'waits': [frozenset(waits.split()) for *_rest, waits in card],
            'down': [(-1, way)[int(figure and way is not None and 'fit' not in waits.split())]
                     for way, figure, (*_rest, waits) in zip(ways, figures, card)]}


def routine(card=CARD):
    """A routine at its first row: the row it is on, when that began, the row it left and the
    row flown last; its card and that card's table (`laid`)."""
    return {'row': 0, 'at': 0.0, 'was': dict(card[0][1]), 'flown': dict(card[0][1]),
            'card': card, 'plan': laid(card)}


def fly(route, now, holds=('held', 'fit')) -> tuple[str, dict[str, Any]]:
    """(the figure's name, its row for `now`): `route` moved on where its row's seconds are up
    and all it waits for is among `holds` or what its row held last pass - or to its flight's
    SPENT_TO's row, `spent` among `holds` -, then its row's setpoints for now, a figure's
    eased in from the row flown last over its own seconds. A move is a number of 0 or 1: no
    clause a kind of row."""
    plan, row = route['plan'], route['row']
    down = int('spent' in holds) * int(plan['down'][row] >= 0)
    on = (1 - down) * int(now - route['at'] >= plan['seconds'][row]) * int(
        plan['waits'][row] <= set(holds) | set(route.get('holds', ())))
    moved = down + on
    row = down * plan['down'][row] + on * ((row + 1) % len(plan['name'])) + (1 - moved) * row
    route.update(row=row, at=(route['at'], now)[moved], was=(route['was'], route['flown'])[moved],
                 holds=(route.get('holds', ()), ())[moved])
    route['flown'] = here = plan['give'][row](route, now)
    return plan['name'][row], here
