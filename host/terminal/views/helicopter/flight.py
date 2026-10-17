"""The HELICOPTER page's flight: two stand-in boards armed, a pass of 3D stunts and the course.

    rotor = arm(Coaxial63100(execution_mode=SIMULATED).open())     # MAIN and TAIL boards
    flight, law = fresh(), aerokin.fresh()                         # the flights' own, on the floor
    for clock, frame in passed(rotors, sky, route, law, flight, clock, dt):
        ...

`CARD` is what the page flies: the 3D aerobatics routine (`machine.helicopter`), its two 63100
drives (`MAIN` on the mast and `TAIL` on the boom) and their thermal observers warmed on it, then
the 16-gate course (`machine.course`).
"""
import ctypes
import math

from machine import aerobatics, aerokin, course, heat as mheat, helicopter, pull, quad
from terminal.views.quad.flight import (CRASHED, CRASH_S, FAULTS, STEP_S, THERMAL_S, TICK_S, _LIB,
                                        airspeeds, arm, asked, fallen, kept, lined, observers,
                                        reset, spun, steps, struck, warmth)

#: The helicopter's two 63100 drives: main rotor on the mast and tail rotor on the boom.
ROTORS = ('MAIN', 'TAIL')

#: The page's flights: the 3D helicopter routine first, then the 16-gate race course.
CARD = helicopter.CARD + course.CARD


def fresh(craft=helicopter):
    """The craft's flights' own before the first: stage, apex, controls, the pack's cells and
    peaks, the wind (`quad.air`), and the 3D `craft.Core`."""
    cells = quad.pack()
    top = craft.K_THRUST * pull.held(pull.I_MAX) ** 2 / craft.MASS_KG
    card = craft.CARD + course.CARD
    return {'stage': card[0][0], 'apex': 0.0, 'share': 1.0, 'over': 0.0, 'lap': None,
            'owed': 0.0, 'read': 0.0, 'core': craft.Core(), 'top': top,
            'pull': aerokin.HEADROOM * min(craft.PULL_MAX, top), 'row': dict(aerobatics.DOWN),
            'controls': (0.0, 0.0, 0.0, 0.0), 'stood': 0.0, 'cells': cells,
            'peak': {'watts': 0.0, 'low': cells['volts']},
            'air': quad.air(), 'wreck': None, 'crashes': 0}


def shell(flight, rotors, craft=helicopter):
    """`(have, spent, fit)` as the two boards' thermal envelopes and pack charge leave the
    craft now."""
    left = flight['cells']['left']
    hold = min([pull.I_WEP] + [(r['budget'] or {}).get('hold_amps', pull.I_WEP) for r in rotors])
    w_top = pull.top(left, min(pull.I_MAX, hold))
    have = aerokin.HEADROOM * min(craft.PULL_MAX, craft.K_THRUST * w_top * w_top / craft.MASS_KG)
    flat = left <= quad.RESERVE
    return have, flat or have < aerokin.LEAST * craft.GRAVITY, not flat and hold >= pull.I_MAX


def passed(rotors, sky, route, law, flight, clock, dt, craft=helicopter):
    """Advance the flight by `dt` s from `clock` in steps of `STEP_S`, yielding `(clock, frame)`
    after each step."""
    count, flight['owed'] = steps(flight['owed'] + dt)
    for part in [STEP_S] * count:
        clock += part
        row = route['row']
        for rotor in rotors:
            rotor['budget'] = rotor['rig'].board.thermal.budget()
        name = flight['stage'] = step(rotors, sky, route, law, flight, clock, part, craft)
        flight['controls'] = sky.controls
        flight['lap'] = route.get('lap') if callable(route['card'][route['row']][1]) else None
        frame = sky.state()
        flight['stood'] = kept(flight, flight['stood'], name, route['row'] != row
                               and 'fit' in route['card'][row][4].split(), frame, part)
        if clock - flight['read'] >= THERMAL_S - 1e-9:
            flight['read'] = clock
            for rotor in rotors:
                rotor['budget'], rotor['ident'], rotor['board_c'] = warmth(rotor['rig'])
        yield clock, frame


def step(rotors, sky, route, law, flight, clock, dt, craft=helicopter):
    """One `dt` s step of the flight: advance the card's figure, run the 3D controller and the
    two 63100 drives (`flown`), or handle crash and recovery."""
    cells, core, frame = flight['cells'], flight['core'], sky.state()
    have, spent, fit = shell(flight, rotors, craft)
    sky.stand(lined(route))
    wreck = flight.get('wreck')
    if wreck is None and frame['hit']:
        wreck = flight['wreck'] = {'what': frame['hit'], 'for': 0.0}
        flight['crashes'] += 1
    if wreck is None:
        # The laps planned on the craft's sphere, the figures on all it has.
        sphere = min(have, craft.COURSE_SPHERE) if lined(route) else have
        route['seen'] = dict(aerokin.seen(law, frame, dt, sphere, flight['top']), spent=spent)
        name, row = aerobatics.fly(route, clock, [word for word, holds in (
            ('held', law['held']), ('spent', spent), ('fit', fit)) if holds])
        flight.update(pull=sphere, share=have / (aerokin.HEADROOM * flight['top']), row=row)
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
        return 'swap' if cells['left'] <= quad.RESERVE else 'cool'
    return aerokin.FALL if name == 'burn' and law['doing'] == aerokin.FALL else name


def begun(law, flight, craft=helicopter):
    """Reset `law` and the craft's 3D `Core` back to the spot."""
    law.clear()
    law.update(aerokin.fresh())
    flight.update(core=craft.Core(), row=dict(aerobatics.DOWN),
                  controls=(0.0, 0.0, 0.0, 0.0))


def flown(rotors, sky, core, law, row, flight, dt, craft=helicopter):
    """Step the two 63100 drives, the 3D controller (`core.step`), and the MuJoCo airframe
    (`sky.step`) across `dt` s in substeps of `TICK_S`."""
    cells, air = flight['cells'], flight.get('air')
    volts, clamp = cells['volts'], pull.I_WEP
    holds = [0.0 if r['fault'] else min(clamp, (r['budget'] or {}).get('hold_amps', clamp))
             for r in rotors]
    top, count, out = pull.ceiling(volts, clamp), max(1, int(round(dt / TICK_S))), None
    along = core.plan(law, row, sky.state(), dt, flight['pull'])
    scales = craft.drive_loads(sky.controls)
    for rotor, sc in zip(rotors, scales):
        w = rotor['w']
        rotor['rig'].board.drive.model.configure(
            b=pull.ROTOR.b + 2.0 * quad.K_DRAG * abs(w),
            load=(sc - 2.0) * quad.K_DRAG * w * abs(w), vdc=volts)
    lib = _LIB
    periods = int(round(TICK_S * 50000))
    tau = math.tau
    for _ in range(count):
        for rotor in rotors:
            buf = rotor['buf']
            tripped = lib.world_drive_pass(rotor['slot'], periods, True, True,
                                           ctypes.c_float(rotor['ask']), buf)
            if tripped:
                rotor['rig'].gates.off()
            w = buf[0] / rotor['pairs']
            rotor['w'] = w
            rotor['w_hat'] = buf[1] / rotor['pairs']
            rotor['angle'] = (rotor['angle'] + w * TICK_S) % tau
            id_val, iq_val = buf[2], buf[3]
            rotor['iq'] = iq_val
            rotor['amps'] = math.hypot(id_val, iq_val)
            rotor['fault'] = FAULTS.get(int(buf[4]))
        limits = [max(craft.IDLE_RAD_S,
                      min(top, r['ask'] + min(pull.slew(clamp, r['w_hat']),
                                              pull.reach(r['w_hat'], volts, hold)) * TICK_S))
                  for r, hold in zip(rotors, holds)]
        out = core.step(law, row, sky.state(), TICK_S, flight['pull'], limits, along)
        for rotor, speed in zip(rotors, out.speeds):
            rotor['ask'] = speed
        if air is not None:
            quad.blown(air, TICK_S)
        sky.step((rotors[0]['w'], rotors[1]['w']) + tuple(out.controls), TICK_S, air)
    watts = sum(pull.watts(r['w'], r['iq'], r['amps']) for r in rotors)
    quad.drawn(cells, watts, dt)
    flight['over'] += dt * float(max(rotor['amps'] for rotor in rotors) > pull.I_MAX)
    told, through = airspeeds(sky.state(), air)
    for rotor in rotors:
        heat = rotor['rig'].board.thermal
        heat.fast_forward(dt * heat.HASTE, live=True)
        heat.airspeed(told, truth=through)
        if 'cooling' in rotor:
            mheat.step_board_cooling(
                rotor['cooling'], rotor.get('amps') or 0.0, volts, dt * heat.HASTE, through,
                craft.COOLING)
    return out


EXPORTED = (arm, asked, observers, spun, struck)

