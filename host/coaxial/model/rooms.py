"""The rooms a simulated or emulated board is laid in, and the tour through them.

A room scales the board's air path and laminate capacity and sets its temperature; the tour
moves on by the board's own identification. The observer is not told the room and reads it from
its own losses, as a board does.
"""

#: The situations: the bench, a box, a fan, a heat sink laid over it; the bench's robot's rooms
#: named for their temperature (2026-09-06).
SITUATIONS = {'bench': {'air': 1.0, 'capacity': 1.0, 'ambient': 25.0},
              'box': {'air': 2.0, 'capacity': 1.0, 'ambient': 25.0},
              'fan': {'air': 0.5, 'capacity': 1.0, 'ambient': 25.0},
              'heatsink': {'air': 0.35, 'capacity': 1.6, 'ambient': 25.0},
              'stuffy': {'air': 1.5, 'capacity': 1.0, 'ambient': 25.0},
              'outdoors': {'air': 0.8, 'capacity': 1.0, 'ambient': -20.0},
              'temperate': {'air': 1.0, 'capacity': 1.0, 'ambient': 20.0},
              'cold': {'air': 0.9, 'capacity': 1.0, 'ambient': -25.0},
              'toasty': {'air': 1.2, 'capacity': 1.0, 'ambient': 45.0}}

#: The tour: temperate 20 C, cold -25, toasty 45, round again. It moves on when the room is
#: earned - STABLE held TOUR_STABLE_S (10 wall s at HASTE), no sooner than TOUR_MIN_S - or at
#: TOUR_MAX_S, model s. Measured under the stand-in page's cycle: STABLE at minute 10
#: temperate, 23-25 cold, 38-48 toasty, so the cap is 50 min (CI, 2026-09-06).
TOUR = ('temperate', 'cold', 'toasty')
TOUR_STABLE_S = 100.0
TOUR_MIN_S = 300.0
TOUR_MAX_S = 3000.0


def next_stop(situation):
    """The tour's room after `situation`, its first from anywhere off it."""
    at = TOUR.index(situation) if situation in TOUR else -1
    return TOUR[(at + 1) % len(TOUR)]


def step(earned_s, stable, dt, stood_s, situation):
    """The tour `dt` model s on: (the STABLE seconds earned in the room, the room to move to or
    None) - on once STABLE held TOUR_STABLE_S and the room stood TOUR_MIN_S, or at TOUR_MAX_S
    whatever the state did."""
    earned = earned_s + dt if stable else 0.0
    if (earned >= TOUR_STABLE_S and stood_s >= TOUR_MIN_S) or stood_s >= TOUR_MAX_S:
        return 0.0, next_stop(situation)
    return earned, None
