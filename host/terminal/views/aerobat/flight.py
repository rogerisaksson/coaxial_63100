"""The AEROBAT page's flight: PROP and VECT 63100 boards armed, 3D monoplane stunts and course.

    rotor = arm(Coaxial63100(execution_mode=SIMULATED).open())     # PROP and VECT boards
    flight, law = fresh(), aerokin.fresh()                         # the flights' own, on the floor
    for clock, frame in passed(rotors, sky, route, law, flight, clock, dt):
        ...

`CARD` is what the page flies: the unlimited 3D aerobatic monoplane routine (`machine.aerobat`),
its two 63100 drives (`PROP` driving the single tractor propeller and `VECT` the tail/vector drive)
and their thermal observers warmed on it, then its airfield's gate course (`machine.airfield`).
"""
from machine import aerobat, airfield
from terminal.views.helicopter import flight as hflight
from terminal.views.quad import flight as qflight
from terminal.views.quad.flight import asked, observers, reset, spun, struck, warmth

#: The aerobat's two 63100 drives: single tractor propeller and tail/vector drive.
ROTORS = ('PROP', 'VECT')

#: The page's flights: the 3D aerobat routine, then two laps of its airfield's course.
CARD, FIELD = aerobat.CARD + airfield.CARD, airfield


def arm(rig, **kw):
    """A stand-in board armed (`quad.flight.arm`) under the tractor propeller's load: its inertia
    and its drag on the drive's model and its speed's loop."""
    return qflight.arm(rig, rotor_m=aerobat.ROTOR, **kw)


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
