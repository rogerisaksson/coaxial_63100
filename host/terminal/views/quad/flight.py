"""The QUAD page's flight: four stand-in boards armed, a pass of it, their envelopes' share.

    rotor = arm(Coaxial63100(execution_mode=SIMULATED).open())     # a board for flight, four times
    flight = fresh()                                               # the flights' own, on the floor
    for clock, frame in passed(rotors, sky, route, flying, flight, clock, dt):   # a pass, its steps
        ...

`CARD` is what the page flies, flight after flight: the routine (machine.aerobatics), its boards
and their observers warmed on it, then the course (machine.course).
"""
import math
import time

from coaxial.devices.thermal import THROTTLE_AT
from coaxial.model.thermal import BOARD_CAPACITY, MOTOR
from coaxial.simulated.sto import PILOT_VOLTS
from machine import aerobatics, course, quad
from machine.flying import FALL, Flying
from machine.parts import SpeedPI

#: The page's flights, one after the other and again: the routine first - the boards warm and
#: their observers out of UNCERTAIN before the course asks its lean of them (the user,
#: 2026-10-05).
CARD = aerobatics.CARD + course.CARD

#: Each rotor's machine, its clamp and trip for a flight, A, and the can and propeller turning,
#: kg m^2 and N m s (quad.json's propeller, the 63100's can). The hover takes 5.7 A; on a 50 A
#: clamp the spools into full tilt and the burn put the switches at 0.95 of the envelope a
#: second flight, throttled, and it burned into the floor at 7.8 m/s (2026-09-28). The drive's
#: own clamp is WEP's, I_WEP: the speed loop holds the flight to I_MAX.
PROFILE, I_MAX, I_TRIP, I_WEP = 'outrunner_63100_14p', 30.0, 45.0, 40.0
ROTOR_J, ROTOR_B = 1.2e-4 + 8e-4, 8e-4

#: The rotors' speed loop, Hz, and the top the flight is flown on, mechanical rad/s: a 24 V
#: link's ceiling on the 63100 at a 50 A clamp, measured - 2 960 rpm, the q inductance's drop
#: beside the back-EMF; the flux's alone, 454, planned the burn on twice the thrust there was.
#: On the pack's 63 V the clamp's 30 A turn the propeller 432: 6.5 A are left to spool on. The
#: four's thrust there, N.
SPEED_HZ, TOP_RAD_S = 3.0, 310.0
TOP_N = 4.0 * quad.K_THRUST * TOP_RAD_S * TOP_RAD_S
#: WEP's: the pack's 63 V at the clamp, and the four's thrust there.
TOP_WEP_RAD_S = 432.0
TOP_WEP_N = 4.0 * quad.K_THRUST * TOP_WEP_RAD_S * TOP_WEP_RAD_S

#: A rotor's speed is asked no faster than this, rad/s^2: half the clamp to turn the can and
#: its propeller. Stepped, 14 rad/s more in a pass was the whole clamp, and the gate stage's
#: envelope 0.2 the higher for a read - 0.91 once in a flight (2026-10-05).
SPOOL_RAD_S2 = 700.0

#: The STO chain's charge on the master's pilot before the interlock passes, s, on the wall's
#: clock the stand-in's chain runs on: opened, a board's charge pump read 2.16 V of the 3.0
#: wanted and its level detector 0.31 of 2.0.
STO_SETTLE_S = 0.05

#: The observers' states out of UNCERTAIN, the margin off its floor: STABLE from 49-70 s of a
#: flight, never in 900 s held at 16 A (2026-10-05, 2026-09-28).
GO = ('CONVERGING', 'STABLE')

#: The boards' envelopes in the flight's: the rotors are asked the share of their pull that
#: the least room UNDER a throttle's point leaves, of SPEND - what full tilt and a burn spend,
#: ten flights on end peaking at 0.73 from hovers at 0.46 - times a throttling board's own
#: derate: taken over TAKEN_S, given back over RECOVER_S - a pass at the clamp is 0.2 of an
#: envelope for one read, the thermal clock ten times the flight's. An observer UNCERTAIN
#: trims its ceilings to 0.8 and its share's room with them; waited for at a room of 0.4, the
#: hover's own share 0.42, the third flight never left its hover; its room counted to the
#: throttle's own point, full tilt on a pack half spent stood at 0.90. Spent GONE_S, it comes
#: down; at once, a burn's own spend took it off its burn (2026-10-05). At 0.08 under it the
#: laps had 0.38 of the rotors' pull at the least, at 0.04 0.52 and a lap 1.5 s the shorter,
#: the boards at 0.70-0.74 of their envelopes (2026-10-06). That spend was 27 K of a 100 K
#: span; a leg's is its FETs' junction's 150 since (2026-10-09): 0.2.
SPEND, UNDER, TAKEN_S, RECOVER_S, GONE_S = 0.2, 0.04, 0.5, 2.0, 2.0

#: A flight is begun on this share of the rotors' pull at the least. All of it is a board at
#: 0.52 of its envelope or under, and an idle board is not cool: on the gate stage's dump at
#: 63 V, no wash over it, its laminate comes to 79 C and it stands at 0.52-0.56 - waited for
#: all of its pull, the page stood 120 s on the floor and would yet (2026-10-06).
FIT = 0.7

#: A spent pack is changed on the floor in this long, s.
SWAP_S = 3.0

#: The flight is stepped this long at the most, s: a late pass is its steps. Stepped 50 ms - a
#: starved page's every pass - the frame rang on its rotors: 30 A through the corkscrew where
#: 24 at 45 ms, the envelopes' share at 0.22, `home` never held and the pack spent on it 38 s
#: later (2026-10-06).
STEP_S = 0.025

#: War emergency power, on an observer's word: the law asking more than it has, the frame's
#: ghost flown RISK_S on as it goes strikes a thing (`quad.Sky.ahead`) - on the ghost's word
#: alone, every dive at a gate and every way down to land took it, 2.3 s a flight
#: (2026-10-09). For WEP_HOLD_S the law has all of it - no
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

#: A board's laminate to the air under its propeller, K/W, in its record: a third of the
#: bench's still air, an assumption - the wash over both faces. On the bench's 8.33 the
#: pack's 63 V had the gate stage at 0.65-0.72 of its envelope in a hover and every board
#: throttling by the corkscrew (2026-10-05).
WASH_K_PER_W = 2.5


def arm(rig):
    """A stand-in board's stage and drive for flight: its gates on the master's pilot, the
    63100's model under the propeller's inertia, a flight's clamp, sensorless."""
    board = rig.board
    # Its record's air under the propeller, and the stand-in's truth laid on that record: the
    # observer and the board it watches in the same air from the first pass.
    board.thermal.configure(board_to_ambient=WASH_K_PER_W, board_capacity=BOARD_CAPACITY)
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
    drive.on('sensorless')
    params = drive.params()
    pairs = max(1.0, params.get('motor_pole_pairs') or 1.0)
    kt = 1.5 * pairs * (params.get('motor_lambda') or 0.002)
    return {'rig': rig, 'pairs': pairs, 'kt': kt, 'w': 0.0, 'w_hat': 0.0, 'iq': 0.0,
            'amps': 0.0, 'angle': 0.0, 'ask': 0.0,
            'pi': SpeedPI(SPEED_HZ, I_MAX, kt, ROTOR_J, ROTOR_B, quad.K_DRAG),
            'budget': {}, 'ident': {}, 'board_c': None}


def fresh():
    """The flights' own before the first: the stage, the apex, the share of their pull the
    envelopes leave and how long it has been none, the lap, how long a spent pack has stood,
    the pack's cells and a flight's peaks of them, the air they are flown in (`quad.air`),
    the wreck of one struck - what it struck, how long ago - and how many were; its emergency
    power: the seconds left of it, how long it is on yet, what it was taken from, how often."""
    cells = quad.pack()
    return {'stage': CARD[0][0], 'apex': 0.0, 'share': 1.0, 'gone': 0.0, 'lap': None,
            'stood': 0.0, 'cells': cells, 'peak': {'watts': 0.0, 'low': cells['volts']},
            'air': quad.air(), 'wreck': None, 'crashes': 0,
            'wep': {'left': WEP_S, 'on': 0.0, 'from': None, 'taken': 0}}


def struck(flight):
    """What the flight's wreck struck, in words, or ''."""
    what = (flight.get('wreck') or {}).get('what') or ''
    return STRUCK.get(what, what.replace('gate', 'into gate '))


def steps(dt):
    """A pass of `dt` s as the flight's steps, s each: STEP_S at the most."""
    count = max(1, math.ceil(dt / STEP_S - 1e-9))
    return [dt / count] * count


def passed(rotors, sky, route, flying, flight, clock, dt):
    """The flight a pass of `dt` s on from `clock`, (its clock, the frame) after each of its
    steps: the envelopes' share and how long it has been none, the card's row flown, its lap
    where the row is a line's, a spent pack changed where it has stood to be."""
    for part in steps(dt):
        clock += part
        row = route['row']
        flight['share'] = envelope(rotors, flight['share'], part)
        flight['gone'] = flight['gone'] + part if flight['share'] <= 0.0 else 0.0
        name = flight['stage'] = step(rotors, sky, route, flying, flight, clock, part)
        flight['lap'] = route.get('lap') if callable(route['card'][route['row']][1]) else None
        frame = sky.state()
        flight['stood'] = kept(flight, flight['stood'], name, route['row'] != row
                               and 'fit' in route['card'][row][4].split(), frame, part)
        yield clock, frame


def step(rotors, sky, route, flying, flight, clock, dt):
    """The flight `dt` s on, its stage: the card's row asked of the law - taken down, its
    pack or the boards' envelopes spent - each rotor's loop after the law's thrust for it,
    within the share of their pull the envelopes leave; the propeller on each shaft, the
    pack's bus under what the four take, and the frame in MuJoCo on the rotors' thrust and
    drag in the flight's air, blown those `dt` s on, among what stands - the course's gates
    for its flights alone; the stand-ins' rotors and heat stepped those seconds with it,
    whatever the wall's clock did. The burn's row falls first; where the card waits to be fit
    a spent pack is changed and the boards cool. Struck (`quad.Sky.hit`), it is CRASHED: its
    rotors stopped, and CRASH_S on the frame on its spot and its flight begun again."""
    cells, share = flight['cells'], flight['share']
    flat = cells['left'] <= quad.RESERVE
    spent, frame = flat or flight['gone'] >= GONE_S, sky.state()
    sky.stand(lined(route))
    wreck = flight.get('wreck')
    if wreck is None and frame['hit']:
        wreck = flight['wreck'] = {'what': frame['hit'], 'for': 0.0}
        flight['crashes'] += 1
    if wreck is None:
        route['seen'] = dict(flying.seen(frame, dt), spent=spent)
        name, flying.ask = aerobatics.fly(route, clock, [word for word, holds in (
            ('held', flying.held), ('spent', spent), ('fit', not flat and share >= FIT))
            if holds])
        share, wep = emergency(flight, sky, flying, route, share, dt)
        flying.top = TOP_WEP_N if wep else TOP_N
        thrusts = flying.step(frame, dt, share, wep)
        if flight['wep'].pop('fresh', False):
            for rotor in rotors:
                rotor['rig'].board.thermal.wep(WEP_HOLD_S)
    else:
        name, thrusts = CRASHED, [0.0] * len(rotors)
        wreck['for'] += dt
        if wreck['for'] >= CRASH_S:
            row = max(k for k in range(route['row'] + 1) if 'fit' in route['card'][k][4].split())
            route.update(row=row, at=clock, was=dict(route['card'][row][1]), holds=())
            route.pop('lap', None)
            sky.reset()
            Flying.__init__(flying, TOP_N, aerobatics.DOWN)
            reset(rotors)
            flight['wreck'] = None
    watts = 0.0
    for rotor, thrust in zip(rotors, thrusts):
        drive = rotor['rig'].board.drive
        # The rotor on the flight's clock, not the wall's: the pass's seconds, however late.
        drive.paced(dt)
        now = drive.state()
        rotor['w_hat'] = (now.get('omega_hat') or 0.0) / rotor['pairs']
        rotor['amps'] = math.hypot(now.get('id') or 0.0, now.get('iq') or 0.0)
        watts += 1.5 * ((now.get('vd') or 0.0) * (now.get('id') or 0.0)
                        + (now.get('vq') or 0.0) * (now.get('iq') or 0.0))
        rotor['w'] = drive.model.read()['omega'] / rotor['pairs']
        rotor['angle'] = (rotor['angle'] + rotor['w'] * dt) % math.tau
        if wreck is None:
            given = I_WEP if flying.top > TOP_N else I_MAX
            more = min(TOP_WEP_RAD_S if given > I_MAX else TOP_RAD_S,
                       quad.speed_for(thrust)) - rotor['ask']
            spool = SPOOL_RAD_S2 * given / I_MAX
            rotor['ask'] += max(-spool * dt, min(spool * dt, more))
            rotor['pi'].limit = given
            rotor['iq'] = rotor['pi'].step(dt, setpoint=rotor['ask'],
                                           measured=rotor['w_hat'])['command']
        else:
            # Cut: run down on its propeller's drag. Asked to a stand through its loop, a
            # sensorless rotor hunted about it at 17 A (2026-10-06).
            rotor['ask'] = rotor['iq'] = 0.0
        drive.write(iq_ref=rotor['iq'])
        drive.model.configure(load=quad.K_DRAG * rotor['w'] * abs(rotor['w']), vdc=cells['volts'])
        # Its heat on the flight's clock as well, the pass's seconds at the stand-in's haste: on
        # the wall's a starved pass's currents stood for all it waited, and a loaded host's
        # boards read 0.94 of their envelopes where 0.82 (2026-10-06).
        heat = rotor['rig'].board.thermal
        heat.fast_forward(dt * heat.HASTE, live=True)
    quad.drawn(cells, watts, dt)
    air = flight.get('air')
    if air is not None:
        quad.blown(air, dt)
    sky.step([r['w'] for r in rotors], dt, air)
    if name == CRASHED:
        return name
    if 'fit' in route['card'][route['row']][4].split() and (flat or share < FIT):
        return 'swap' if flat else 'cool'
    return FALL if name == 'burn' and flying.doing == FALL else name


def emergency(flight, sky, flying, route, share, dt):
    """(the share of the rotors' pull the law has this step, what war emergency power is
    taken from or None): the envelopes' - or, on it, all of the pull. Taken where the law
    asks more than it has (`Flying.short`) and the frame's ghost strikes a thing within RISK_S
    (the observer); kept WEP_HOLD_S past that,
    the boards told afresh (`fresh`); WEP_S of it a flight, whole again where the card waits
    to be fit."""
    wep = flight.get('wep')
    if wep is None:
        return share, None
    if 'fit' in route['card'][route['row']][4].split():
        wep.update(left=WEP_S, on=0.0)
        return share, None
    risk = sky.ahead(RISK_S) if flying.short and wep['left'] > 0.0 else None
    if risk:
        wep['taken'] += wep['on'] < 1e-9
        wep.update(on=WEP_HOLD_S, fresh=True, **{'from': risk[0]})
    if wep['on'] < 1e-9 or wep['left'] < 1e-9:
        return share, None
    wep.update(on=wep['on'] - dt, left=wep['left'] - dt)
    return 1.0, wep['from']


def envelope(rotors, was, dt):
    """What the boards' envelopes leave of the rotors' pull, 0..1, `dt` s after it was `was`."""
    budgets = [r['budget'] or {} for r in rotors]
    share = min(min(1.0, b.get('derate') or 1.0)
                * max(0.0, min(1.0, (THROTTLE_AT - UNDER - (b.get('worst') or 0.0)) / SPEND))
                for b in budgets)
    return max(was - dt / TAKEN_S, min(share, was + dt / RECOVER_S))


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
    """Every rotor's speed loop from rest: the frame back on the floor spools them from idle."""
    for rotor in rotors:
        rotor['pi'] = SpeedPI(SPEED_HZ, I_MAX, rotor['kt'], ROTOR_J, ROTOR_B, quad.K_DRAG)
        rotor['ask'] = 0.0


def warmth(rig):
    """(budget, identification, the board's hottest node C) of a rotor's thermal observer."""
    thermal = rig.board.thermal
    nodes = thermal.state().get('nodes') or {}
    board = [c for n, c in nodes.items() if n not in MOTOR and c is not None]
    return thermal.budget(), thermal.identification(), max(board) if board else None


def observers(rotors):
    """The four observers in a row's words: who is stable, who converges, of how many where
    one is UNCERTAIN yet. `4 of 4 converging` it said whatever they were (2026-10-05)."""
    states = [(r['ident'] or {}).get('state') for r in rotors]
    known, stable = sum(s in GO for s in states), states.count('STABLE')
    if stable == len(states):
        return '%d of %d stable' % (stable, len(states))
    return '%d stable, %d converging%s' % (
        stable, known - stable, ' of %d' % len(states) if known < len(states) else '')
