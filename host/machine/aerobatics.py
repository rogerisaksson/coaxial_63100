"""The quad's aerobatics: a figure a row of `machine.flying`'s setpoints, a routine a card of them.

    flying.ask = aerobatics.HOVER                    # any pass: the setpoints, nothing else
    name, row = aerobatics.fly(route, now, holds)    # the card's row for now, eased in
"""
import math

from machine.gaits import mix
from machine.quad import FLOOR_M, HOVER_M

#: A figure a row of setpoints: height, m over the floor, and the pace it may go there at, m/s,
#: come to and stopped from in a second;
#: speed along its heading and slide across it, m/s; turn, its heading's, deg/s; home, how much
#: its spot goes back where it rose, 0 to 1; roll and flip, turns/s about its nose and about its
#: wing.
HOVER = {'height': HOVER_M, 'pace': 1.5, 'speed': 0.0, 'slide': 0.0, 'turn': 0.0, 'home': 1.0,
         'roll': 0.0, 'flip': 0.0}
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

#: The routine: (name, row, seconds on it at least, seconds eased into it, what it waits for
#: before it leaves). `ready`: the boards' thermal observers and their envelopes' room; `held`:
#: the frame at its height and its spot, still, its turns whole.
CARD = (
    ('idle', DOWN, 2.0, 0.0, ''),
    ('spool', PRESSED, 2.5, 2.5, ''),
    ('lift', HOVER, 3.0, 3.0, ''),
    ('hover', HOVER, 3.0, 0.0, 'ready'),
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
    ('level', HOVER, 2.0, 1.5, 'held ready'),
    ('full tilt', SKY, 2.0, 0.0, ''),
    ('burn', STOP, 0.0, 0.0, 'held'),
    ('hold', STOP, 3.0, 0.0, ''),
    ('land', PRESSED, 3.0, 3.0, ''),
)


def ease(x):
    """The share of a move made `x` of the way through its time, at rest and unpulled at both
    ends: the least jerk a move has."""
    x = max(0.0, min(1.0, x))
    return x * x * x * (10.0 - 15.0 * x + 6.0 * x * x)


def routine(card=CARD):
    """A routine at its first row: the row it is on, when that began and the row it left."""
    return {'row': 0, 'at': 0.0, 'was': dict(card[0][1]), 'card': card}


def fly(route, now, holds=('ready', 'held')):
    """(the figure's name, its row for `now`): `route` moved on where its row's seconds are up
    and all it waits for is among `holds`, the row eased in from the one left over its own
    seconds."""
    card = route['card']
    name, row, seconds, eased, waits = card[route['row']]
    into = now - route['at']
    if into >= seconds and set(waits.split()) <= set(holds):
        route.update(row=(route['row'] + 1) % len(card), at=now,
                     was=mix(route['was'], row, ease(into / eased) if eased > 0.0 else 1.0))
        name, row, seconds, eased, waits = card[route['row']]
        into = 0.0
    return name, mix(route['was'], row, ease(into / eased) if eased > 0.0 else 1.0)
