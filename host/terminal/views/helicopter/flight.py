"""The HELICOPTER page's flight: two stand-in boards armed, a pass of 3D stunts and the course.

    rotor = arm(Coaxial63100(execution_mode=SIMULATED).open())     # MAIN and TAIL boards
    flight, law = fresh(), aerokin.fresh()                         # the flights' own, on the floor
    for clock, frame in passed(rotors, sky, route, law, flight, clock, dt):
        ...

`CARD` is what the page flies: the 3D aerobatics routine (`machine.aircraft.helicopter`), its two 63100
drives (`MAIN` on the mast and `TAIL` on the boom) and their thermal observers warmed on it, then
the 16-gate course (`machine.aircraft.course`).
"""
from machine.aircraft import aerobatics, aerokin, course, flightpack, helicopter, pull, sweep, weather
from terminal.views.quad import flight as qflight
from terminal.views.quad.flight import (CRASHED, CRASH_S, FAULTS, THERMAL_S, TICK_S, _LIB,
                                        _step_bus_and_heat, _step_drives, arm, asked, fallen, kept,
                                        lined, observers, reset, spun, steps, struck, warmth)

#: The helicopter's two 63100 drives: main rotor on the mast and tail rotor on the boom.
ROTORS = ('MAIN', 'TAIL')

#: The page's flights: the 3D helicopter routine first, then the 16-gate race course.
CARD = sweep.opened(helicopter.CARD) + course.TWIN_CARD


def fresh(craft=helicopter):
    """The craft's flights' own before the first: stage, apex, controls, the pack's cells and
    peaks, the wind (`weather.air`), and the 3D `craft.Core`."""
    cells = flightpack.pack()
    top = craft.K_THRUST * pull.held(pull.I_MAX, getattr(craft, 'ROTOR', pull.ROTOR)) ** 2 / craft.MASS_KG
    card = craft.CARD + course.CARD
    return {'stage': card[0][0], 'apex': 0.0, 'share': 1.0, 'over': 0.0, 'lap': None,
            'owed': 0.0, 'read': 0.0, 'core': craft.Core(), 'top': top,
            'pull': aerokin.HEADROOM * min(craft.PULL_MAX, top), 'row': dict(aerobatics.DOWN),
            'controls': (0.0, 0.0, 0.0, 0.0), 'stood': 0.0, 'cells': cells,
            'peak': {'watts': 0.0, 'low': cells['volts']},
            'air': weather.air(), 'wreck': None, 'crashes': 0}


def shell(flight, rotors, craft=helicopter):
    """`(have, spent, fit)` as the two boards' thermal envelopes and pack charge leave the
    craft now."""
    left = flight['cells']['left']
    hold = min([pull.I_WEP] + [(r['budget'] or {}).get('hold_amps', pull.I_WEP) for r in rotors])
    w_top = pull.top(left, min(pull.I_MAX, hold))
    have = aerokin.HEADROOM * min(craft.PULL_MAX, craft.K_THRUST * w_top * w_top / craft.MASS_KG)
    flat = left <= flightpack.RESERVE
    return have, flat or have < aerokin.LEAST * craft.GRAVITY, not flat and hold >= pull.I_MAX


def passed(rotors, sky, route, law, flight, clock, dt, craft=helicopter):
    """Advance the flight by `dt` s from `clock` in steps of `STEP_S`, yielding `(clock, frame)`
    after each step."""
    return qflight.passed(rotors, sky, route, law, flight, clock, dt,
                          lambda r, s, rt, l, f, cl, d: step(r, s, rt, l, f, cl, d, craft))


def step(rotors, sky, route, law, flight, clock, dt, craft=helicopter):
    """One `dt` s step of the flight: advance the card's figure, run the 3D controller and the
    two 63100 drives (`flown`), or handle crash and recovery."""
    cells, core, frame = flight['cells'], flight['core'], sky.state(False)
    have, spent, fit = shell(flight, rotors, craft)
    sky.stand(lined(route))
    wreck = flight.get('wreck')
    if wreck is None and frame['hit']:
        wreck = flight['wreck'] = {'what': frame['hit'], 'for': 0.0}
        flight['crashes'] += 1
    if wreck is None:
        # The laps planned on the craft's sphere, the figures on all it has; its law on all.
        sphere = min(have, craft.COURSE_SPHERE) if lined(route) else have
        route['seen'] = dict(aerokin.seen(law, frame, dt, sphere, flight['top']), spent=spent)
        name, row = aerobatics.fly(route, clock, [word for word, holds in (
            ('held', law['held']), ('spent', spent), ('fit', fit)) if holds])
        flight.update(pull=have, share=have / (aerokin.HEADROOM * flight['top']), row=row)
        flown(rotors, sky, core, law, row, flight, dt, craft)
    else:
        name = CRASHED
        wreck['for'] += dt
        again = wreck['for'] >= CRASH_S
        if again:
            row = max((k for k in range(route['row'] + 1)
                       if 'fit' in route['card'][k][4].split()), default=0)
            route.update(row=row, at=clock, was=dict(route['card'][row][1]), holds=())
            route.pop('lap', None)
            sky.reset()
            begun(law, flight, craft)
            reset(rotors)
            flight['wreck'] = None
        fallen(rotors, sky, flight, dt, not again)
    if name == CRASHED:
        return name
    if 'fit' in route['card'][route['row']][4].split() and not fit:
        return 'swap' if cells['left'] <= flightpack.RESERVE else 'cool'
    return aerokin.FALL if name == 'burn' and law['doing'] == aerokin.FALL else name


def begun(law, flight, craft=helicopter):
    """Reset `law` and the craft's 3D `Core` back to the spot."""
    law.clear()
    law.update(aerokin.fresh())
    flight.update(core=craft.Core(), row=dict(aerobatics.DOWN),
                  controls=(0.0, 0.0, 0.0, 0.0))


def throttled(rotor, speed):
    """A rotor its law asks `speed`, rad/s: cut where asked none - its loop off, its bridge off
    (`_step_drives`), run down on its propeller's drag -, spun from rest (`spun`) where a cut one
    is asked to turn."""
    if speed <= 0.0 and not rotor['cut']:
        rotor['rig'].board.ctrl.off()
        rotor['rig'].board.drive.write(iq_ref=0.0)
        rotor['cut'] = True
    elif speed > 0.0 and rotor['cut']:
        spun(rotor)


def flown(rotors, sky, core, law, row, flight, dt, craft=helicopter):
    """Step the two 63100 drives, the 3D controller (`core.step`), and the MuJoCo airframe
    (`sky.step`) across `dt` s in substeps of `TICK_S`."""
    cells, air = flight['cells'], flight.get('air')
    volts, clamp = cells['volts'], pull.I_WEP
    holds = [0.0 if r['fault'] else min(clamp, (r['budget'] or {}).get('hold_amps', clamp))
             for r in rotors]
    turned = getattr(craft, 'ROTOR', pull.ROTOR)
    top, count, out = pull.ceiling(volts, clamp, turned), max(1, int(round(dt / TICK_S))), None
    along = core.plan(law, row, sky.state(False), dt, flight['pull'])
    scales = craft.drive_loads(sky.controls, getattr(sky, 'share', 1.0))
    for rotor, sc in zip(rotors, scales):
        w = rotor['w']
        rotor['rig'].board.drive.model.configure(
            b=turned.b + 2.0 * turned.drag * abs(w),
            load=(sc - 2.0) * turned.drag * w * abs(w), vdc=volts)
    periods = int(round(TICK_S * 50000))
    for _ in range(count):
        _step_drives(rotors, periods)
        limits = [max(craft.IDLE_RAD_S,
                      min(top, r['ask'] + min(pull.slew(clamp, r['w_hat'], turned),
                                              pull.reach(r['w_hat'], volts, hold, turned)) * TICK_S))
                  for r, hold in zip(rotors, holds)]
        out = core.step(law, row, sky.state(False), TICK_S, flight['pull'], limits, along)
        for rotor, speed in zip(rotors, out.speeds):
            throttled(rotor, speed)
            rotor['ask'] = speed
        if air is not None:
            weather.blown(air, TICK_S)
        sky.step((rotors[0]['w'], rotors[1]['w']) + tuple(out.controls), TICK_S, air)
    assert out is not None
    flight['controls'] = out.controls
    _step_bus_and_heat(rotors, sky, flight, dt, craft.COOLING)
    return out


EXPORTED = (arm, asked, observers, spun, struck, FAULTS, THERMAL_S, _LIB, kept, steps, warmth)


