"""The AEROBAT page's flight: PROP and VECT 63100 boards armed, 3D monoplane stunts and course.

    rotor = arm(Coaxial63100(execution_mode=SIMULATED).open())     # PROP and VECT boards
    flight, law = fresh(), aerokin.fresh()                         # the flights' own, on the floor
    for clock, frame in passed(rotors, sky, route, law, flight, clock, dt):
        ...

`CARD` is what the page flies: the unlimited 3D aerobatic monoplane routine (`machine.aerobat`),
its two 63100 drives (`PROP` and `VECT` driving the coaxial contra-rotating tractor stages) and
their thermal observers warmed on it, then the 16-gate course (`machine.course`).
"""
from machine import aerobat, course
from terminal.views.helicopter import flight as hflight
from terminal.views.quad.flight import arm, asked, observers, reset, spun, struck, warmth

#: The aerobat's two 63100 drives: front tractor propeller and rear contra/vector stage.
ROTORS = ('PROP', 'VECT')

#: The page's flights: the 3D aerobat routine, then two laps of the 16-gate course.
CARD = aerobat.CARD + course.CARD


def fresh():
    """The aerobat flights' own before the first."""
    return hflight.fresh(aerobat)


def shell(flight, rotors):
    """`(have, spent, fit)` as the two boards' thermal envelopes leave the aerobat now."""
    return hflight.shell(flight, rotors, aerobat)


def passed(rotors, sky, route, law, flight, clock, dt):
    """Advance the aerobat flight by `dt` s from `clock` in steps of `STEP_S`."""
    return hflight.passed(rotors, sky, route, law, flight, clock, dt, aerobat)


def step(rotors, sky, route, law, flight, clock, dt):
    """One `dt` s step of the aerobat flight."""
    return hflight.step(rotors, sky, route, law, flight, clock, dt, aerobat)


def begun(law, flight):
    """Reset `law` and the aerobat's 3D `Core` back to the runway."""
    hflight.begun(law, flight, aerobat)


def flown(rotors, sky, core, law, row, flight, dt):
    """Step the `PROP` and `VECT` 63100 drives, `aerobat.Core`, and `aerobat.Sky`."""
    return hflight.flown(rotors, sky, core, law, row, flight, dt, aerobat)


AERO_REEXPORTS = (arm, asked, observers, reset, spun, struck, warmth)
