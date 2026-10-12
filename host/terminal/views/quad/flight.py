"""The QUAD page's flight: four stand-in boards armed, a pass of it, their envelopes' share.

    rotor = arm(Coaxial63100(execution_mode=SIMULATED).open())     # a board for flight, four times
    flight, law = fresh(), aerokin.fresh()                         # the flights' own, on the floor
    for clock, frame in passed(rotors, sky, route, law, flight, clock, dt):   # a pass, its steps
        ...

`CARD` is what the page flies, flight after flight: the routine (machine.aerobatics), its boards
and their observers warmed on it, then the course (machine.course). A step is its card's row
made a payload (machine.aerokin) and the core's passes on it (aero/), each rotor's loop after
the thrust it is given.
"""
import math
import time

from coaxial.control.commission import Commissioning
from coaxial.control.motion import started
from coaxial.model.thermal import MOTOR
from coaxial.simulated.sto import PILOT_VOLTS
from machine import aerobatics, aerokin, course, quad
from machine.controller import Feedback
from machine.parts import Gain, Slew, SpeedPI
from tools.cores import aero

#: The page's flights, one after the other and again: the routine first - the boards warm and
#: their observers out of UNCERTAIN before the course asks its lean of them.
CARD = aerobatics.CARD + course.CARD

#: Each rotor's machine, its clamp and trip for a flight, A - the clamp the boards' envelopes
#: bind under, WEP's past them, the trip the board's rating -, and the can and a 16x14 turning,
#: kg m^2 and N m s. The drive's own clamp is WEP's, I_WEP: the speed loop holds the flight to
#: I_MAX - on I_WEP the boards stood at 1.00 of their envelopes in full tilt.
PROFILE, I_MAX, I_TRIP, I_WEP = 'outrunner_63100_14p', 60.0, 100.0, 90.0
ROTOR_J, ROTOR_B = 1.2e-4 + 3.3e-4, 8e-4

#: The rotors' speed loop, Hz, and the top the flight is flown on, mechanical rad/s: the 16x14
#: on the pack's 63 V at the clamp, measured on a stand-in - 703 at 60 A, 691 on a sagged 55 V;
#: on the 24 V link the 20x10 stood at 310. The four's thrust there, N. The loop is each
#: board's own (device 12), ticked LOOP_HZ a second in its drive's sample: stepped here a pass
#: of 20 ms at a time, 38 A of braking held a pass took a rotor from 197 rad/s through its
#: crossover to 37, the estimate lost it on the torque's turn and two boards tripped in a burn.
SPEED_HZ, TOP_RAD_S, LOOP_HZ = 3.0, 700.0, 1000
TOP_N = 4.0 * quad.K_THRUST * TOP_RAD_S * TOP_RAD_S
#: WEP's: 739 at its 90 A, 66 A of it, the link's ceiling; and the four's thrust there.
TOP_WEP_RAD_S = 735.0
TOP_WEP_N = 4.0 * quad.K_THRUST * TOP_WEP_RAD_S * TOP_WEP_RAD_S

#: A rotor's speed is come to no faster than this, rad/s^2 - its loop's own slew: half the
#: clamp to turn the can and its propeller, 0.5 x 0.0435 N m/A x 60 A over ROTOR_J. Stepped, 14
#: rad/s more in a pass was the whole clamp, and the gate stage's envelope 0.2 the higher for a
#: read.
SPOOL_RAD_S2 = 2900.0

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

#: The boards' envelopes in the flight's: its row is planned on the share of the rotors' pull
#: that the board with the least in it may carry (`thermal.budget`'s `hold_amps`, of I_MAX) -
#: each board holds its own clamp to that, and no node of it comes to its throttle. Asked by a
#: rule on the envelopes' room, read ten times a second, full tilt had the four at 57 A for
#: 0.1 s, then each board throttled on its own - 33, 8, 32 and 18 A - and the frame went
#: over to 83 degrees. The share none for GONE_S, it comes down: at once, a burn's own spend
#: took it off its burn.
GONE_S = 2.0

#: A flight is begun on this share of the rotors' pull at the least. All of it is a board at
#: 0.52 of its envelope or under, and an idle board is not cool: on the gate stage's dump at
#: 63 V, no wash over it, its laminate comes to 79 C and it stands at 0.52-0.56 - waited for
#: all of its pull, the page stood 120 s on the floor and would yet.
FIT = 0.7

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

#: War emergency power, on an observer's word: the law asking more than it has, the frame's
#: ghost flown RISK_S on as it goes strikes a thing (`quad.Sky.ahead`) - on the ghost's word
#: alone, every dive at a gate and every way down to land took it, 2.3 s a flight.
#: For WEP_HOLD_S the law has all of it - no
#: envelope's share, the boards' thermal derate held off (`thermal.wep`, the trip standing),
#: I_WEP and TOP_WEP_RAD_S, its lean whatever its row's or, the floor ahead, what is left of
#: the pull after what it asks up - WEP_S of it a flight, given back on the floor.
RISK_S, WEP_HOLD_S, WEP_S = 0.5, 0.6, 5.0

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


def arm(rig):
    """A stand-in board's stage and drive for flight: its gates on the master's pilot, the
    63100's model under the propeller's inertia, a flight's clamp, its record's gains
    commissioning's arithmetic on that motor - on the record's placeholders the board's law
    trips at the first torque -, its own loop on the rotor's speed laid - a slew, the pole
    pairs, the speed's PI -, the rotor at its idle (`spun`)."""
    board = rig.board
    # Its heat on the flight's clock from here: on the wall's until the first pass, four boards
    # armed took a run's own seconds of it, and a flight run again was another.
    board.thermal.fast_forward(0.0)
    # In its rotor's wash, and the stand-in's truth laid on that: the observer and the board it
    # watches in the same air from the first pass.
    board.thermal.configure(application=APPLICATION)
    board.thermal.situation('bench')
    board.afe.on()
    rig.pilot(PILOT_VOLTS)
    time.sleep(STO_SETTLE_S)
    board.gate_drivers.clear()
    rig.gates.on()
    drive = board.drive
    drive.configure(profile=PROFILE)
    drive.configure(drv_i_max=I_WEP, drv_i_trip=I_TRIP)
    drive.model.configure(j=ROTOR_J, b=ROTOR_B, load=0.0, vdc=quad.open_volts(1.0))
    drive.configure(source='model')
    Commissioning(rig).gains()
    params = drive.params()
    pairs = max(1.0, params.get('motor_pole_pairs') or 1.0)
    kt = 1.5 * pairs * (params.get('motor_lambda') or 0.002)
    board.ctrl.load(Feedback(SpeedPI(SPEED_HZ, I_MAX, kt, ROTOR_J, ROTOR_B, quad.K_DRAG),
                             prefilter=Slew(SPOOL_RAD_S2), measure=Gain(1.0 / pairs)))
    board.ctrl.wire('omega_hat', 'iq_ref', LOOP_HZ)
    rotor = {'rig': rig, 'pairs': pairs, 'kt': kt, 'w': 0.0, 'w_hat': 0.0, 'iq': 0.0,
             'amps': 0.0, 'angle': 0.0, 'ask': 0.0, 'cut': False, 'fault': None,
             'budget': {}, 'ident': {}, 'board_c': None}
    spun(rotor)
    return rotor


def spun(rotor):
    """A rotor from rest to its idle, its board's loop on it there: started as a stepper is -
    its vector turned up to the idle -, the law's estimate on the back-EMF by the time the
    loop has it."""
    board, idle = rotor['rig'].board, quad.IDLE_RAD_S * rotor['pairs']
    drive, loop = board.drive, board.ctrl
    loop.off()
    started(drive, lambda seconds: (drive.paced(seconds), drive.state()), START_A, idle,
            START_S)
    loop.write(quad.IDLE_RAD_S)
    loop.on()
    rotor.update(ask=quad.IDLE_RAD_S, cut=False)


def fresh():
    """The flights' own before the first: the stage, the apex, the share of their pull the
    envelopes leave and how long it has been none, the lap, how long a spent pack has stood,
    the pack's cells and a flight's peaks of them, the air they are flown in (`quad.air`),
    the wreck of one struck - what it struck, how long ago - and how many were; its emergency
    power: the seconds left of it, how long it is on yet, what it was taken from, how often;
    the seconds a pass owes its next step, the clock its observers were read at; the frame's
    core, the rotors' top a kg and the pull its row is planned on, m/s^2, the row flown."""
    cells = quad.pack()
    top = TOP_N / quad.MASS_KG
    return {'stage': CARD[0][0], 'apex': 0.0, 'share': 1.0, 'gone': 0.0, 'lap': None,
            'owed': 0.0, 'read': 0.0, 'core': aero.Core(), 'top': top,
            'pull': aerokin.HEADROOM * top, 'row': dict(aerobatics.DOWN),
            'stood': 0.0, 'cells': cells, 'peak': {'watts': 0.0, 'low': cells['volts']},
            'air': quad.air(), 'wreck': None, 'crashes': 0,
            'wep': {'left': WEP_S, 'on': 0.0, 'from': None, 'taken': 0}}


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
    frame) after each of its steps: the envelopes' share and how long it has been none, the
    card's row flown, its lap where the row is a line's, a spent pack changed where it has stood
    to be; the boards' observers read every THERMAL_S."""
    count, flight['owed'] = steps(flight['owed'] + dt)
    for part in [STEP_S] * count:
        clock += part
        row = route['row']
        for rotor in rotors:
            rotor['budget'] = rotor['rig'].board.thermal.budget()
        flight['share'] = envelope(rotors)
        flight['gone'] = flight['gone'] + part if flight['share'] <= 0.0 else 0.0
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
    """The flight `dt` s on, its stage: the card's row for now - taken down, its pack or the
    boards' envelopes spent - made a payload of the frame as it flies (`aerokin.step`),
    planned on the share of their pull the envelopes leave, and flown (`flown`): the core's
    passes on it, each rotor's loop after the thrust it is given within what its board
    holds, the frame in MuJoCo in the flight's air among what stands - the course's gates
    for its flights alone. The burn's row falls first; where the card waits to be fit a spent
    pack is changed and the boards cool. Struck (`quad.Sky.hit`), it is CRASHED: its rotors
    stopped (`fallen`), and CRASH_S on the frame on its spot and its flight begun again."""
    cells, share, core = flight['cells'], flight['share'], flight['core']
    flat = cells['left'] <= quad.RESERVE
    spent, frame = flat or flight['gone'] >= GONE_S, sky.state()
    sky.stand(lined(route))
    wreck = flight.get('wreck')
    if wreck is None and frame['hit']:
        wreck = flight['wreck'] = {'what': frame['hit'], 'for': 0.0}
        flight['crashes'] += 1
    if wreck is None:
        route['seen'] = dict(aerokin.seen(law, frame, dt, flight['pull'], flight['top']),
                             spent=spent)
        name, row = aerobatics.fly(route, clock, [word for word, holds in (
            ('held', law['held']), ('spent', spent), ('fit', not flat and share >= FIT))
            if holds])
        share, wep = emergency(flight, sky, law, route, share, dt)
        top = flight['top'] = (TOP_WEP_N if wep else TOP_N) / quad.MASS_KG
        flight.update(pull=share * aerokin.HEADROOM * top, row=row)
        if flight['wep'].pop('fresh', False):
            for rotor in rotors:
                rotor['rig'].board.thermal.wep(WEP_HOLD_S)
        flown(rotors, sky, core, aero.payload(aerokin.step(
            law, row, frame, dt, flight['pull'], tuple(core.out.head), wep,
            route['seen']['pitch'])), flight, dt, wep is not None)
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
    if 'fit' in route['card'][route['row']][4].split() and (flat or share < FIT):
        return 'swap' if flat else 'cool'
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
        drive.model.configure(b=ROTOR_B, load=quad.K_DRAG * rotor['w'] * abs(rotor['w']),
                              vdc=cells['volts'])
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
    stage takes."""
    now = rotor['rig'].board.drive.state()
    rotor['fault'] = now.get('fault')
    rotor['iq'] = now.get('iq') or 0.0
    rotor['amps'] = math.hypot(now.get('id') or 0.0, rotor['iq'])
    return 1.5 * ((now.get('vd') or 0.0) * (now.get('id') or 0.0)
                  + (now.get('vq') or 0.0) * (now.get('iq') or 0.0))


def flown(rotors, sky, core, payload, flight, dt, wep=False):
    """The flight `dt` s on on `payload` (aero/), the core's passes of TICK_S: each board's
    rotor read; its reach from the speed its loop is asked - the loop's slew either way, up no
    faster than its board's current gives (`reached`) -; the core's thrusts asked of the
    loops; the frame on the rotors' speeds in the flight's air. Then the step's own: the
    pack's bus, the boards' heat and their airspeed. The core's last answer. A propeller's
    torque is laid a step, K w |w| as its slope about the speed it turns at: laid a pass, a
    tenth of a flight's cost."""
    cells, air = flight['cells'], flight.get('air')
    clamp, top = (I_WEP, TOP_WEP_RAD_S) if wep else (I_MAX, TOP_RAD_S)
    count, out = max(1, int(round(dt / TICK_S))), None
    for rotor in rotors:
        rotor['rig'].board.drive.model.configure(
            b=ROTOR_B + 2.0 * quad.K_DRAG * abs(rotor['w']),
            load=-quad.K_DRAG * rotor['w'] * abs(rotor['w']), vdc=cells['volts'])
    for _ in range(count):
        for rotor in rotors:
            turned(rotor, TICK_S)
        most = [(reached(r, clamp, TICK_S) - r['w_hat']) / TICK_S for r in rotors]
        out = core.step(payload, aero.reading(sky.state(), *aero.reach(
            [r['ask'] for r in rotors], [r['w_hat'] for r in rotors], TICK_S, SPOOL_RAD_S2,
            LEAST_RAD_S, top, most), TICK_S))
        for rotor, speed in zip(rotors, aero.speeds(out.thrust)):
            rotor['ask'] = speed
            rotor['rig'].board.ctrl.write(speed)
        if air is not None:
            quad.blown(air, TICK_S)
        sky.step([r['w'] for r in rotors], TICK_S, air)
    quad.drawn(cells, sum(stated(rotor) for rotor in rotors), dt)
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


def emergency(flight, sky, law, route, share, dt):
    """(the share of the rotors' pull the law has this step, what war emergency power is
    taken from or None): the envelopes' - or, on it, all of the pull. Taken where the law
    asks more than it has (its `short`) and the frame's ghost strikes a thing within RISK_S
    (the observer); kept WEP_HOLD_S past that,
    the boards told afresh (`fresh`); WEP_S of it a flight, whole again where the card waits
    to be fit."""
    wep = flight.get('wep')
    if wep is None:
        return share, None
    if 'fit' in route['card'][route['row']][4].split():
        wep.update(left=WEP_S, on=0.0)
        return share, None
    risk = sky.ahead(RISK_S) if law['short'] and wep['left'] > 0.0 else None
    if risk:
        wep['taken'] += wep['on'] < 1e-9
        wep.update(on=WEP_HOLD_S, fresh=True, **{'from': risk[0]})
    if wep['on'] < 1e-9 or wep['left'] < 1e-9:
        return share, None
    wep.update(on=wep['on'] - dt, left=wep['left'] - dt)
    return 1.0, wep['from']


def envelope(rotors):
    """What the boards' envelopes leave of the rotors' pull, 0..1: the least any of them may
    carry (`hold_amps`), of the flight's clamp."""
    return max(0.0, min([1.0] + [(r['budget'] or {}).get('hold_amps', I_MAX) / I_MAX
                                 for r in rotors]))


def reached(rotor, clamp, dt):
    """The speed a rotor reaches `dt` s on at the most, rad/s: from the one its board says it
    has, on what the current that board may carry - `clamp` A at the most - leaves over its
    propeller's drag and its own; none of it on a tripped drive. Its cap in the thrusts'
    share: counted from its speed and its loop's slew, a rotor behind what it was asked had
    its next ask cut to that, and the laps were 0.3 s the longer."""
    w = rotor['w_hat']
    amps = 0.0 if rotor['fault'] else min(clamp, (rotor['budget'] or {}).get('hold_amps', clamp))
    return w + (rotor['kt'] * amps - quad.K_DRAG * w * abs(w) - ROTOR_B * w) / ROTOR_J * dt


def kept(flight, stood, stage, off, frame, dt):
    """The flight's own after a pass at `stage`, seen `frame`: a spent pack changed where it
    has `stood` SWAP_S to be - those seconds back -, a flight's peaks from where it is `off`
    that floor, its apex."""
    cells, peak = flight['cells'], flight['peak']
    stood = stood + dt if stage == 'swap' else 0.0
    if stood >= SWAP_S:
        cells.update(quad.pack())
    if off:
        peak.update(watts=0.0, low=cells['volts'])
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
