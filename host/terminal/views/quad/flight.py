"""The QUAD page's flight: four stand-in boards armed, a pass of it, what they hold of its pull.

    rotor = arm(Coaxial63100(execution_mode=SIMULATED).open())     # a board for flight, four times
    flight, law = fresh(), aerokin.fresh()                         # the flights' own, on the floor
    for clock, frame in passed(rotors, sky, route, law, flight, clock, dt):   # a pass, its steps
        ...

`CARD` is what the page flies, flight after flight: the routine (machine.aerobatics), its boards
and their observers warmed on it, then the course (machine.course). A step is its card's row
made a payload (machine.aerokin) and the core's passes on it (aero/), each rotor's loop after
the thrust it is given.
"""
import ctypes
import math
import time

from coaxial.control.commission import Commissioning
from coaxial.control.motion import started
from coaxial.model.thermal import MOTOR
from coaxial.simulated.sto import PILOT_VOLTS
from machine import aerobatics, aerokin, course, pull, quad
from machine.controller import Feedback
from machine.parts import Gain, Slew, SpeedPI
from tools.cores import aero, stand

_LIB = stand.library()
FAULTS = {0: None, 1: 'overcurrent', 2: 'stage', 3: 'supply'}

#: The page's flights, one after the other and again: the routine first - the boards warm and
#: their observers out of UNCERTAIN before the course asks its lean of them.
CARD = aerobatics.CARD + course.CARD

#: Each rotor's machine and its trip for a flight, A, the board's rating. Its clamps, its
#: tops and its reach are the action space's (`machine.pull`): the drive's clamp and its
#: speed loop's WEP's, each board holding its own to what its envelope leaves; the plan's
#: the flight's.
PROFILE, I_TRIP = 'outrunner_63100_14p', 100.0

#: The rotors' speed loop, Hz: each board's own (device 12), ticked LOOP_HZ a second in its
#: drive's sample: stepped here a pass of 20 ms at a time, 38 A of braking held a pass took a
#: rotor from 197 rad/s through its crossover to 37, the estimate lost it on the torque's turn
#: and two boards tripped in a burn.
SPEED_HZ, LOOP_HZ = 3.0, 1000

#: A rotor's speed is come to no faster than this, rad/s^2 - its loop's own slew, the
#: flight's clamp's. Stepped, 14 rad/s more in a pass was the whole clamp, and the gate
#: stage's envelope 0.2 the higher for a read. On WEP's slew, 4 347, a rotor braked to its
#: idle in a lap went to 15 rad/s and was lost; flown, its loop's slew scales by speed
#: (`pull.slew`).
SPOOL_RAD_S2 = pull.slew(pull.I_WEP)

#: A rotor is asked no slower than this while it flies, rad/s: its idle. Slower, the law's
#: estimate has no back-EMF to stand on - asked none in a fall, a rotor went to -5 rad/s and its
#: board tripped.
LEAST_RAD_S = quad.IDLE_RAD_S

#: A rotor is started on this current, A, its vector turned up to the idle over this long, s of
#: the rotor's clock: from rest through its loop the injection's estimate ran 2.8 rad ahead in
#: 20 ms on 30 A and the rotor spun back to -44 rad/s.
START_A, START_S = 15.0, 0.4

#: The STO chain's charge on the master's pilot before the interlock passes, s, on the wall's
#: clock the stand-in's chain runs on: opened, a board's charge pump read 2.16 V of the 3.0
#: wanted and its level detector 0.31 of 2.0.
STO_SETTLE_S = 0.05

#: The observers' states out of UNCERTAIN, the margin off its floor: STABLE from 49-70 s of a
#: flight, never in 900 s held at 16 A.
GO = ('CONVERGING', 'STABLE')

#: A spent pack is changed on the floor in this long, s.
SWAP_S = 3.0

#: The flight is stepped this long, s - the race's (tools/sim/quad_race.py) -, a pass's
#: seconds short of a step owed to the next: stepped as the wall's passes came, a loaded page
#: flew its air's eddies drawn on other steps and crossed gate 0 0.97 m low, one flight in nine.
#: Stepped 50 ms the frame rang on its rotors: 30 A through the corkscrew where
#: 24 at 45 ms.
STEP_S = 0.02

#: The boards' observers are read this often by the flight's clock, s.
THERMAL_S = 0.1

#: The core's pass (aero/), s: the world's step, STEP_S / TICK_S of them a step of the flight.
TICK_S = quad.STEP_S

#: An ask goes to its board as this many rows, a tick of its loop each, along the line from
#: the last (`asked`): written whole a pass of the core, the loop's feed-forward took each
#: for a spool and the tick after for its end - 6.3 A apart at 500 Hz on 300 rad/s^2, 12 %
#: of the copper's heat round the course.
ROWS = int(round(TICK_S * LOOP_HZ))

#: Struck, a flight is over: its stage this, its rotors stopped and the wreck left where it
#: falls for so long, s; then the frame is on its spot again and the flight begun over.
CRASHED, CRASH_S = 'crashed', 4.0

#: What it struck, in words (`quad.Sky.hit`); a gate by its number.
STRUCK = {'tree': 'into a tree', 'house': 'into a house', 'car': 'into a car',
          'mast': 'into the mast', 'floor': 'on the floor'}

#: What a board is mounted in: its rotor's wash, 2.5 K/W off the laminate in a hover against
#: the bench's 8.33 (thermal_app.c). On 8.33 the pack's 63 V had the gate stage at 0.65-0.72
#: of its envelope in a hover and every board throttling by the corkscrew.
APPLICATION = 'airstream'


def arm(rig, clock=1, warm_s=100.0, room='bench'):
    """A stand-in board's stage and drive for flight: its gates on the master's pilot, the
    63100's model under the propeller's inertia, a flight's clamp, its record's gains
    commissioning's arithmetic on that motor - on the record's placeholders the board's law
    trips at the first torque -, its own loop on the rotor's speed laid - a slew, the pole
    pairs, the speed's PI -, the rotor at its idle (`spun`).
    The thermal observer runs on `clock` thermal seconds a second (1 for the flight's own
    seconds), warmed `warm_s` s at idle on the floor so the observers are out of UNCERTAIN
    before a first flight."""
    board = rig.board
    # Its heat on the flight's clock from here: on the wall's until the first pass, four boards
    # armed took a run's own seconds of it, and a flight run again was another.
    board.thermal.fast_forward(0.0)
    # In its rotor's wash, and the stand-in's truth laid on that: the observer and the board it
    # watches in the same air from the first pass.
    board.thermal.configure(application=APPLICATION, clock=clock)
    board.thermal.situation(room or 'bench')
    board.afe.on()
    rig.pilot(PILOT_VOLTS)
    time.sleep(STO_SETTLE_S)
    board.gate_drivers.clear()
    rig.gates.on()
    drive = board.drive
    drive.configure(profile=PROFILE)
    drive.configure(drv_i_max=pull.I_WEP, drv_i_trip=I_TRIP)
    drive.model.configure(j=pull.ROTOR.j, b=pull.ROTOR.b, load=0.0, vdc=quad.open_volts(1.0))
    drive.configure(source='model')
    Commissioning(rig).gains()
    params = drive.params()
    pairs = max(1.0, params.get('motor_pole_pairs') or 1.0)
    kt = 1.5 * pairs * (params.get('motor_lambda') or 0.002)
    board.ctrl.load(Feedback(SpeedPI(SPEED_HZ, pull.I_WEP, kt, pull.ROTOR.j, pull.ROTOR.b,
                                     quad.K_DRAG),
                             prefilter=Slew(SPOOL_RAD_S2), measure=Gain(1.0 / pairs)))
    board.ctrl.wire('omega_hat', 'iq_ref', LOOP_HZ)
    rotor = {'rig': rig, 'pairs': pairs, 'w': 0.0, 'w_hat': 0.0, 'iq': 0.0,
             'amps': 0.0, 'angle': 0.0, 'ask': 0.0, 'cut': False, 'fault': None,
             'budget': {}, 'ident': {}, 'board_c': None,
             'slot': drive._slot, 'buf': (ctypes.c_double * 5)()}
    spun(rotor)
    if warm_s:
        board.thermal.fast_forward(warm_s, live=True)
    return rotor


def spun(rotor):
    """A rotor from rest to its idle, its board's loop on it there: started as a stepper is -
    its vector turned up to the idle -, the law's estimate on the back-EMF by the time the
    loop has it."""
    board, idle = rotor['rig'].board, quad.IDLE_RAD_S * rotor['pairs']
    drive, loop = board.drive, board.ctrl
    loop.off()
    loop.drop()
    started(drive, lambda seconds: (drive.paced(seconds), drive.state()), START_A, idle,
            START_S)
    loop.rows([(0.0, quad.IDLE_RAD_S)])
    loop.on()
    rotor.update(ask=quad.IDLE_RAD_S, cut=False)


def asked(rotor, speed):
    """A rotor's loop asked `speed`, rad/s, by the core's next pass: ROWS rows along the line
    from its last ask, none for the same again."""
    was, rotor['ask'] = rotor['ask'], speed
    if speed != was:
        rotor['rig'].board.ctrl.rows([(1.0 / LOOP_HZ, was + (speed - was) * (k + 1.0) / ROWS)
                                      for k in range(ROWS)])


def fresh():
    """The flights' own before the first: the stage, the apex, the share of the clamp's pull
    its rows have, the lap, how long a spent pack has stood, the pack's cells and a flight's
    peaks of them, the air they are flown in (`quad.air`), the wreck of one struck - what it
    struck, how long ago - and how many were; the seconds a rotor was over the flight's
    clamp, on WEP's; the seconds a pass owes its next step, the clock its observers were
    read at; the frame's core, what the flight's clamp pulls a kg and the pull its row has
    (`shell`), m/s^2, the row flown."""
    cells = quad.pack()
    top = pull.thrust(pull.held(pull.I_MAX)) / quad.MASS_KG
    return {'stage': CARD[0][0], 'apex': 0.0, 'share': 1.0, 'over': 0.0, 'lap': None,
            'owed': 0.0, 'read': 0.0, 'core': aero.Core(), 'top': top,
            'pull': aerokin.HEADROOM * top, 'row': dict(aerobatics.DOWN),
            'stood': 0.0, 'cells': cells, 'peak': {'watts': 0.0, 'low': cells['volts']},
            'air': quad.air(), 'wreck': None, 'crashes': 0}


def shell(flight, rotors):
    """(the pull a row has, m/s^2 a kg; whether the flight is spent; whether it is fit) as
    the action space has it now: the rotors' top on the clamp the board with the least in it
    holds (`thermal.budget`'s `hold_amps`) - the flight's at the most - and on the pack's
    charge, HEADROOM of it; spent with its pack at its reserve or that pull under what the
    frame is sped up on at the least, fit with a pack over its reserve and every board
    holding the flight's clamp. Each board holds its own clamp to what it may carry, and no
    node of it comes to its throttle: asked by a rule on the envelopes' room, read ten times
    a second, full tilt had the four at 57 A for 0.1 s, then each throttled on its own - 33,
    8, 32 and 18 A - and the frame went over to 83 degrees."""
    left = flight['cells']['left']
    hold = min([pull.I_WEP] + [(r['budget'] or {}).get('hold_amps', pull.I_WEP) for r in rotors])
    have = aerokin.HEADROOM * pull.thrust(pull.top(left, min(pull.I_MAX, hold))) / quad.MASS_KG
    flat = left <= quad.RESERVE
    return have, flat or have < aerokin.LEAST * quad.GRAVITY, not flat and hold >= pull.I_MAX


def struck(flight):
    """What the flight's wreck struck, in words, or ''."""
    what = (flight.get('wreck') or {}).get('what') or ''
    return STRUCK.get(what, what.replace('gate', 'into gate '))


def steps(owed):
    """(the steps of STEP_S `owed` s holds, the seconds still owed)."""
    count = int(owed / STEP_S + 1e-9)
    return count, max(0.0, owed - count * STEP_S)


def passed(rotors, sky, route, law, flight, clock, dt):
    """The flight a pass of `dt` s on from `clock` - what it owed before -, (its clock, the
    frame) after each of its steps: what its boards hold, the card's row flown, its lap where
    the row is a line's, a spent pack changed where it has stood to be; the boards' observers
    read every THERMAL_S."""
    count, flight['owed'] = steps(flight['owed'] + dt)
    for part in [STEP_S] * count:
        clock += part
        row = route['row']
        for rotor in rotors:
            rotor['budget'] = rotor['rig'].board.thermal.budget()
        name = flight['stage'] = step(rotors, sky, route, law, flight, clock, part)
        flight['lap'] = route.get('lap') if callable(route['card'][route['row']][1]) else None
        frame = sky.state()
        flight['stood'] = kept(flight, flight['stood'], name, route['row'] != row
                               and 'fit' in route['card'][row][4].split(), frame, part)
        if clock - flight['read'] >= THERMAL_S - 1e-9:
            flight['read'] = clock
            for rotor in rotors:
                rotor['budget'], rotor['ident'], rotor['board_c'] = warmth(rotor['rig'])
        yield clock, frame


def step(rotors, sky, route, law, flight, clock, dt):
    """The flight `dt` s on, its stage: the card's row for now - taken down, spent - made a
    payload of the frame as it flies (`aerokin.step`) on the pull the action space has for
    it (`shell`), and flown (`flown`): the core's passes on it, each rotor's loop after the
    thrust it is given within what its board holds, the frame in MuJoCo in the flight's air
    among what stands - the course's gates for its flights alone. The burn's row falls
    first; where the card waits to be fit a spent pack is changed and the boards cool.
    Struck (`quad.Sky.hit`), it is CRASHED: its rotors stopped (`fallen`), and CRASH_S on the
    frame on its spot and its flight begun again."""
    cells, core, frame = flight['cells'], flight['core'], sky.state()
    have, spent, fit = shell(flight, rotors)
    sky.stand(lined(route))
    wreck = flight.get('wreck')
    if wreck is None and frame['hit']:
        wreck = flight['wreck'] = {'what': frame['hit'], 'for': 0.0}
        flight['crashes'] += 1
    if wreck is None:
        route['seen'] = dict(aerokin.seen(law, frame, dt, flight['pull'], flight['top']),
                             spent=spent)
        name, row = aerobatics.fly(route, clock, [word for word, holds in (
            ('held', law['held']), ('spent', spent), ('fit', fit)) if holds])
        flight.update(pull=have, share=have / (aerokin.HEADROOM * flight['top']), row=row)
        flown(rotors, sky, core, aero.payload(aerokin.step(
            law, row, frame, dt, have, tuple(core.out.head), route['seen']['pitch'])),
            flight, dt)
    else:
        name = CRASHED
        wreck['for'] += dt
        again = wreck['for'] >= CRASH_S
        if again:
            row = max(k for k in range(route['row'] + 1) if 'fit' in route['card'][k][4].split())
            route.update(row=row, at=clock, was=dict(route['card'][row][1]), holds=())
            route.pop('lap', None)
            sky.reset()
            begun(law, flight)
            reset(rotors)
            flight['wreck'] = None
        fallen(rotors, sky, flight, dt, not again)
    if name == CRASHED:
        return name
    if 'fit' in route['card'][route['row']][4].split() and not fit:
        return 'swap' if cells['left'] <= quad.RESERVE else 'cool'
    return aerokin.FALL if name == 'burn' and law['doing'] == aerokin.FALL else name


def begun(law, flight):
    """The law and the frame's core as before a first pass: the frame put back on its spot."""
    law.clear()
    law.update(aerokin.fresh())
    flight.update(core=aero.Core(), row=dict(aerobatics.DOWN))


def fallen(rotors, sky, flight, dt, cut=True):
    """A wreck `dt` s on: its rotors `cut` - run down on their propellers' drag: asked to a
    stand through its loop, a sensorless rotor hunted about it at 17 A -, the pack's bus, the
    boards' heat and their airspeed, the frame where it falls."""
    cells, air, watts = flight['cells'], flight.get('air'), 0.0
    for rotor in rotors:
        drive = rotor['rig'].board.drive
        turned(rotor, dt)
        watts += stated(rotor)
        if cut and not rotor['cut']:
            rotor['rig'].board.ctrl.off()
            drive.write(iq_ref=0.0)
            rotor.update(ask=0.0, cut=True)
        drive.model.configure(b=pull.ROTOR.b, vdc=cells['volts'],
                              load=quad.K_DRAG * rotor['w'] * abs(rotor['w']))
    quad.drawn(cells, watts, dt)
    told, through = airspeeds(sky.state(), air)
    for rotor in rotors:
        heat = rotor['rig'].board.thermal
        heat.fast_forward(dt * heat.HASTE, live=True)
        heat.airspeed(told, truth=through)
    if air is not None:
        quad.blown(air, dt)
    sky.step([r['w'] for r in rotors], dt, air)


def turned(rotor, dt):
    """A rotor `dt` s on by the flight's clock - not the wall's: the pass's seconds, however
    late -: its speed as it turns and as its board estimates it, one read of its model."""
    drive = rotor['rig'].board.drive
    drive.paced(dt)
    motor = drive.model.read()
    rotor['w'] = motor['omega'] / rotor['pairs']
    rotor['w_hat'] = (motor.get('omega_hat') or 0.0) / rotor['pairs']
    rotor['angle'] = (rotor['angle'] + rotor['w'] * dt) % math.tau


def stated(rotor):
    """What a rotor's board says of itself, a step's: its fault, its currents; the watts its
    stage takes (`pull.watts`)."""
    now = rotor['rig'].board.drive.state()
    rotor['fault'] = now.get('fault')
    rotor['iq'] = now.get('iq') or 0.0
    rotor['amps'] = math.hypot(now.get('id') or 0.0, rotor['iq'])
    return pull.watts(rotor['w'], rotor['iq'], rotor['amps'])


def flown(rotors, sky, core, payload, flight, dt):
    """The flight `dt` s on on `payload` (aero/), the core's passes of TICK_S: each board's
    rotor read; its reach from the speed its loop is asked - the loop's slew either way, up no
    faster than the current its board holds - WEP's clamp at the most - and its link's volts
    leave it gives (`pull.reach`: none of it on a tripped drive), under its link's ceiling -;
    the core's thrusts asked of the loops; the frame on the rotors' speeds in the flight's
    air. Then the step's own: the pack's bus, the seconds a rotor was over the flight's
    clamp, the boards' heat and their airspeed. The core's last answer.
    A propeller's torque is laid a step, K w |w| as its slope about the speed it turns at:
    laid a pass, a tenth of a flight's cost. A rotor's cap counted from its speed and its
    loop's slew, one behind what it was asked had its next ask cut to that, and the laps were
    0.3 s the longer."""
    cells, air = flight['cells'], flight.get('air')
    volts, clamp = cells['volts'], pull.I_WEP
    holds = [0.0 if r['fault'] else min(clamp, (r['budget'] or {}).get('hold_amps', clamp))
             for r in rotors]
    top, count, out = pull.ceiling(volts, clamp), max(1, int(round(dt / TICK_S))), None
    for rotor in rotors:
        rotor['rig'].board.drive.model.configure(
            b=pull.ROTOR.b + 2.0 * quad.K_DRAG * abs(rotor['w']),
            load=-quad.K_DRAG * rotor['w'] * abs(rotor['w']), vdc=volts)
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
            id_val = buf[2]
            iq_val = buf[3]
            rotor['iq'] = iq_val
            rotor['amps'] = math.hypot(id_val, iq_val)
            rotor['fault'] = FAULTS.get(int(buf[4]))
        most = [pull.reach(r['w_hat'], volts, hold) for r, hold in zip(rotors, holds)]
        slews = [pull.slew(clamp, r['w_hat']) for r in rotors]
        out = core.step(payload, aero.reading(sky.state(), *aero.reach(
            [r['ask'] for r in rotors], [r['w_hat'] for r in rotors], TICK_S, slews,
            LEAST_RAD_S, top, most), TICK_S))
        for rotor, speed in zip(rotors, aero.speeds(out.thrust)):
            rotor['ask'] = speed
        if air is not None:
            quad.blown(air, TICK_S)
        sky.step([r['w'] for r in rotors], TICK_S, air)
    watts = sum(pull.watts(r['w'], r['iq'], r['amps']) for r in rotors)
    quad.drawn(cells, watts, dt)
    flight['over'] += dt * float(max(rotor['amps'] for rotor in rotors) > pull.I_MAX)
    told, through = airspeeds(sky.state(), air)
    for rotor in rotors:
        heat = rotor['rig'].board.thermal
        heat.fast_forward(dt * heat.HASTE, live=True)
        heat.airspeed(told, truth=through)
    return out


def airspeeds(frame, air):
    """(the frame's speed as its flight controller has it - over the ground -, its speed
    through `air`), m/s: what its boards' observers are told, what their truths fly in."""
    wind = quad.wind_at(air['wind'], frame['h']) if air else (0.0, 0.0, 0.0)
    return (math.sqrt(sum(float(v) ** 2 for v in frame['vel'])),
            math.sqrt(sum((float(v) - u) ** 2 for v, u in zip(frame['vel'], wind))))


def kept(flight, stood, stage, off, frame, dt):
    """The flight's own after a pass at `stage`, seen `frame`: a spent pack changed where it
    has `stood` SWAP_S to be - those seconds back -, a flight's peaks and its seconds over
    its clamp from where it is `off` that floor, its apex."""
    cells, peak = flight['cells'], flight['peak']
    stood = stood + dt if stage == 'swap' else 0.0
    if stood >= SWAP_S:
        cells.update(quad.pack())
    if off:
        peak.update(watts=0.0, low=cells['volts'])
        flight['over'] = 0.0
    peak.update(watts=max(peak['watts'], cells['watts']), low=min(peak['low'], cells['volts']))
    flight['apex'] = frame['h'] if stage == 'full tilt' else max(flight['apex'], frame['h'])
    return stood


def lined(route):
    """Whether the flight `route` is on flies a line: a row that gives its own among those
    between two that wait to be fit."""
    card, row = route['card'], route['row']
    floors = [k for k, figure in enumerate(card) if 'fit' in figure[4].split()] + [len(card)]
    first = max((k for k in floors if k <= row), default=0)
    return any(callable(figure[1]) for figure in card[first:min(k for k in floors if k > row)])


def reset(rotors):
    """Every rotor from rest again (`spun`): the frame back on the floor, at their idle."""
    for rotor in rotors:
        rotor['rig'].board.drive.model.reset()
        spun(rotor)


def warmth(rig):
    """(budget, identification, the board's hottest node C) of a rotor's thermal observer."""
    thermal = rig.board.thermal
    nodes = thermal.state().get('nodes') or {}
    board = [c for n, c in nodes.items() if n not in MOTOR and c is not None]
    return thermal.budget(), thermal.identification(), max(board) if board else None


def observers(rotors):
    """The four observers in a row's words: who is stable, who converges, of how many where
    one is UNCERTAIN yet. `4 of 4 converging` it said whatever they were."""
    states = [(r['ident'] or {}).get('state') for r in rotors]
    known, stable = sum(s in GO for s in states), states.count('STABLE')
    if stable == len(states):
        return '%d of %d stable' % (stable, len(states))
    return '%d stable, %d converging%s' % (
        stable, known - stable, ' of %d' % len(states) if known < len(states) else '')
