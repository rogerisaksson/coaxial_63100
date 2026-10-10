"""The quad's aerobatics: a figure a row of `machine.flying`'s setpoints, a routine a card of them.

    flying.ask = aerobatics.HOVER                    # any pass: the setpoints, nothing else
    name, row = aerobatics.fly(route, now, holds)    # the card's row for now, eased in

Spent - its pack, its boards' envelopes - the routine comes down from wherever it is and waits
on the floor until it is fit.
"""
import math
from typing import Any

from machine import quad
from machine.gaits import mix
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
#: of its weight a bend had 0.6 of its pull and the frame ran 1.2 m wide (2026-10-05).
HOVER = {'height': HOVER_M, 'pace': 1.5, 'climb': 0.0, 'push': 0.0, 'speed': 0.0, 'slide': 0.0,
         'surge': 0.0, 'sway': 0.0, 'turn': 0.0, 'nose': 1.0, 'x': 0.0, 'z': 0.0, 'home': 1.0,
         'lean': 8.0, 'roll': 0.0, 'flip': 0.0, 'light': 0.5}
#: Full tilt: a height out of reach, at any pace.
SKY = dict(HOVER, height=1000.0, pace=100.0)
#: The stop over the floor, from wherever it is: let fall, burned, held. And on the floor: let
#: fall half a metre under it, the skids down and the rotors at their idle.
STOP = dict(HOVER, height=FLOOR_M, pace=100.0)
DOWN = dict(STOP, height=-0.5)
#: A turn on its spot, as the discs' drag turns it.
PIROUETTE = dict(HOVER, turn=120.0)
#: An orbit: slid ORBIT_M_S round the point ORBIT_M ahead of its nose, its nose on it, banked 25
#: degrees, a lap LAP_S. Mixed with a hover it keeps its circle - both its slide and its turn
#: are as much of theirs - and eased in and out alike it is as many laps as its seconds.
ORBIT_M, ORBIT_M_S = 2.0, 3.0
ORBIT = dict(HOVER, height=2.0, slide=ORBIT_M_S, turn=-math.degrees(ORBIT_M_S / ORBIT_M),
             home=0.0)
LAP_S = math.tau * ORBIT_M / ORBIT_M_S
#: The orbit climbed: a corkscrew to TOP_M; and its way back, down again.
TOP_M = 8.0
CORKSCREW = dict(ORBIT, height=TOP_M, pace=2.0)
UNWIND = dict(ORBIT, height=HOVER_M, pace=2.5, slide=-ORBIT_M_S, turn=-ORBIT['turn'])
#: Over its spot at the top. A toss: the sky's row at a pace; at that pace it turns over, about
#: its nose or about its wing, and is caught where it was tossed from.
HIGH = dict(HOVER, height=TOP_M, pace=4.0)
TOSS = dict(SKY, pace=5.0)
ROLL = dict(HIGH, roll=1.25)
FLIP = dict(HIGH, flip=-1.25)
#: Pressed on the floor, its height asked this far under it, m: the skids down, the rotors
#: bearing all but a twentieth of it - where a landing ends and a lift begins.
PRESSED = dict(HOVER, height=-0.02)
#: Down to the stop's mark over its spot from wherever it is, at a pace.
OVER = dict(HOVER, height=FLOOR_M, pace=2.0)

#: The routine: (name, row, seconds on it at least, seconds eased into it, what it waits for
#: before it leaves). `held`: the frame at its height and its spot, still, its turns whole;
#: `fit`: a pack with more than its reserve in it, the boards' envelopes with their room. A
#: row may be what gives one - `row(route, now)` - where its setpoints are not a figure's but
#: a line's (`machine.course`); what it holds itself it leaves in the route's `holds`.
CARD = (
    ('idle', DOWN, 2.0, 0.0, 'fit'),
    ('spool', PRESSED, 2.5, 2.5, ''),
    ('lift', HOVER, 3.0, 3.0, ''),
    ('hover', HOVER, 3.0, 0.0, ''),
    ('pirouette', PIROUETTE, 3.0, 2.0, ''),
    ('poise', HOVER, 2.0, 2.0, ''),
    ('orbit', ORBIT, LAP_S, 1.5, ''),
    ('corkscrew', CORKSCREW, LAP_S, 1.5, ''),
    ('home', HIGH, 2.0, 1.5, 'held'),
    ('toss', TOSS, 1.0, 0.0, ''),
    ('roll', ROLL, 0.8, 0.0, ''),
    ('catch', HIGH, 3.0, 0.0, ''),
    ('toss', TOSS, 1.0, 0.0, ''),
    ('flip', FLIP, 0.8, 0.0, ''),
    ('catch', HIGH, 3.0, 0.0, ''),
    ('unwind', UNWIND, LAP_S, 1.5, ''),
    ('level', HOVER, 2.0, 1.5, 'held'),
    ('full tilt', SKY, 2.0, 0.0, ''),
    ('burn', STOP, 0.0, 0.0, 'held'),
    ('hold', STOP, 1.5, 0.0, ''),
    ('descend', OVER, 1.5, 1.5, 'held'),
    ('land', PRESSED, 3.0, 3.0, ''),
)
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
    if not 0.0 <= into < seconds:
        return dict(row)
    x, span = into / seconds, row['height'] - was['height']
    return dict(mix(was, row, ease(x)), climb=span * 30.0 * x * x * (1.0 - x) ** 2 / seconds,
                push=span * 60.0 * x * (1.0 - x) * (1.0 - 2.0 * x) / (seconds * seconds))


def routine(card=CARD):
    """A routine at its first row: the row it is on, when that began and the row it left."""
    return {'row': 0, 'at': 0.0, 'was': dict(card[0][1]), 'card': card}


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


def fly(route, now, holds=('held', 'fit')) -> tuple[str, dict[str, Any]]:
    """(the figure's name, its row for `now`): `route` moved on where its row's seconds are up
    and all it waits for is among `holds` - or to its flight's SPENT_TO's row, `spent` among
    them - the row eased in from the one left over its own seconds."""
    card = route['card']
    name, row, seconds, over, waits = card[route['row']]
    into = now - route['at']
    here = eased(route['was'], row, into, over) if isinstance(row, dict) else row(route, now)
    down = _way_down(card, route['row'])
    if ('spent' in holds and down is not None and isinstance(row, dict)
            and 'fit' not in waits.split()):
        route.update(row=down, at=now, was=here, holds=())
    elif into >= seconds and set(waits.split()) <= set(holds) | set(route.get('holds', ())):
        route.update(row=(route['row'] + 1) % len(card), at=now, was=here, holds=())
    else:
        return name, here
    name, row, _seconds, over, _waits = card[route['row']]
    return name, eased(route['was'], row, 0.0, over) if isinstance(row, dict) else row(route, now)
