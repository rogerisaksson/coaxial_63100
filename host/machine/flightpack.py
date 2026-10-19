"""A craft's flight pack: its cells' volts by the charge left, its bus under what its drives take.

    cells = flightpack.pack()                     # a fresh pack
    volts = flightpack.drawn(cells, watts, dt)    # its bus under what they take
"""
import math

#: The pack: PACK_CELLS in series, a cell's open volts by the share of its charge left - a LiPo's
#: curve, 63 V full -, its charge, A h - a racer's, a few flights -, and its and its leads'
#: resistance, ohm, the bus drooping that much a link amp: assumptions with a name each. Spent
#: at RESERVE of its charge. At 0.12 ohm the four's 8 kW at their clamps took the bus to half.
PACK_CELLS, PACK_AH, PACK_OHM, RESERVE = 15, 1.0, 0.04, 0.2
CELL_V = ((0.0, 3.30), (0.05, 3.50), (0.2, 3.70), (0.5, 3.85), (0.9, 4.05), (1.0, 4.20))


def open_volts(left):
    """The pack's volts unloaded with `left` of its charge in it."""
    left = max(0.0, min(1.0, left))
    for (a, low), (b, high) in zip(CELL_V, CELL_V[1:]):
        if left <= b:
            return PACK_CELLS * (low + (high - low) * (left - a) / (b - a))
    return PACK_CELLS * CELL_V[-1][1]


def pack():
    """A fresh pack: the share of its charge left, its bus's volts, the amps and watts it gave
    last."""
    return {'left': 1.0, 'volts': open_volts(1.0), 'amps': 0.0, 'watts': 0.0}


def sagged(volts, watts):
    """The pack's bus with `watts` taken at it, V: its open `volts` less PACK_OHM a link amp."""
    return 0.5 * (volts + math.sqrt(max(0.0, volts * volts - 4.0 * PACK_OHM * watts)))


def drawn(cells, watts, dt):
    """The pack `cells` `dt` s on with `watts` taken at its bus - given, braked into: the bus's
    volts (`sagged`), its charge less what its link's amps took."""
    volts = open_volts(cells['left'])
    bus = sagged(volts, watts)
    amps = (volts - bus) / PACK_OHM
    cells.update(left=max(0.0, min(1.0, cells['left'] - amps * dt / (3600.0 * PACK_AH))),
                 volts=bus, amps=amps, watts=watts)
    return bus
