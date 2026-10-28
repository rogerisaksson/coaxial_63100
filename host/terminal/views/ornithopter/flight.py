"""The ORNITHOPTER page's flight: FORE and AFT 63100 boards armed, Dune stunts and the course.

    rotor = arm(Coaxial63100(execution_mode=SIMULATED).open())     # FORE and AFT wing boards
    flight, law = fresh(), aerokin.fresh()                         # the flights' own, on the floor
    for clock, frame in passed(rotors, sky, route, law, flight, clock, dt):
        ...

`CARD` is what the page flies: the Dune 4-wing ornithopter routine (`machine.aircraft.ornithopter`), its
two 63100 drives (`FORE` and `AFT` flapping the fore and aft wing pairs) and their thermal
observers warmed on it, then the 16-gate course (`machine.aircraft.course`).
"""
from machine.aircraft import course, ornithopter, sweep
from terminal.views.helicopter import flight as hflight
from terminal.views.quad.flight import arm, asked, observers, reset, spun, struck, warmth

#: The ornithopter's two 63100 wing drives: fore wing pair and aft wing pair on the carapace.
ROTORS = ('FORE', 'AFT')

#: The page's flights: the Dune ornithopter routine first, then the 16-gate race course.
CARD = sweep.opened(ornithopter.CARD) + course.TWIN_CARD


def fresh():
    """The ornithopter flights' own before the first."""
    return hflight.fresh(ornithopter)


def shell(flight, rotors):
    """`(have, spent, fit)` as the two wing-drive boards' thermal envelopes leave the ornithopter."""
    return hflight.shell(flight, rotors, ornithopter)


def passed(rotors, sky, route, law, flight, clock, dt):
    """Advance the ornithopter flight by `dt` s from `clock` in steps of `STEP_S`."""
    return hflight.passed(rotors, sky, route, law, flight, clock, dt, ornithopter)


def step(rotors, sky, route, law, flight, clock, dt):
    """One `dt` s step of the ornithopter flight."""
    return hflight.step(rotors, sky, route, law, flight, clock, dt, ornithopter)


def begun(law, flight):
    """Reset `law` and the ornithopter's 3D `Core` back to the desert floor."""
    hflight.begun(law, flight, ornithopter)


def flown(rotors, sky, core, law, row, flight, dt):
    """Step the `FORE` and `AFT` 63100 drives, `ornithopter.Core`, and `ornithopter.Sky`."""
    return hflight.flown(rotors, sky, core, law, row, flight, dt, ornithopter)


ORNI_REEXPORTS = (arm, asked, observers, reset, spun, struck, warmth)
