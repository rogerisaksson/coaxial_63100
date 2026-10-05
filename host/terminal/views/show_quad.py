"""QUAD: four coaxial boards flying a frame in MuJoCo, its routine, full tilt, a stop 10 cm up.

A 2 kg frame (machine.quad) spooled on the floor, lifted to a hover and flown through its
routine (machine.aerobatics' card on machine.flying's law): a pirouette, an orbit and a
corkscrew up, a roll and a flip over a toss, the corkscrew back down, full tilt into the sky,
the fall burned to a stop 10 cm over the floor, held and landed. Each rotor a stand-in board
armed on the master's pilot, its drive sensorless on the 63100 outrunner's model under an APC
20x10E propeller, its speed loop on the drive's own estimate; the four on one pack, 63 V full,
its bus drooping under what they take; the frame in MuJoCo on the rotors' thrust and drag,
gravity and the air. The rotors are asked what the boards' thermal envelopes leave of their
pull, their observers converged or not; a pack or the envelopes spent, it comes down for a
charged one, or to cool. The
camera sits close on the floor before a flight, pulls back as it lifts and follows it, looking
down on it (coaxial.graphics.quadcopter); the height and the power, the bus and the hottest
board over the last half minute are boxes at the side (terminal.views.quad.traces).

    python terminal/views/show_quad.py
    python terminal/views/show_quad.py --frames 60
"""
import argparse
import math
import sys
import time
from contextlib import suppress

from rich.text import Text

from coaxial import Coaxial63100
from coaxial.devices.thermal import THROTTLE_AT
from coaxial.errors import RigError
from coaxial.graphics import gpu, quadcopter
from coaxial.model.thermal import BOARD_CAPACITY, MOTOR
from coaxial.simulated.sto import PILOT_VOLTS
from machine import aerobatics, quad
from machine.flying import FALL, Flying
from machine.modes import SIMULATED
from machine.parts import SpeedPI
from terminal.loader import TO_MENU
from terminal.ui.screen import FPS_CAP, Feed, closing, run_view, say
from terminal.ui.scroll import HUD_WIDTH
from terminal.ui.stage import boot, frame_of, hud, stage
from terminal.views.quad import traces

TITLE = 'QUAD'

#: The rotors, round the frame: front left, front right, rear right, rear left.
ROTORS = ('FL', 'FR', 'RR', 'RL')

#: Each rotor's machine, its clamp and trip for a flight, A, and the can and propeller turning,
#: kg m^2 and N m s (quad.json's propeller, the 63100's can). The hover takes 5.7 A; on a 50 A
#: clamp the spools into full tilt and the burn put the switches at 0.95 of the envelope a
#: second flight, throttled, and it burned into the floor at 7.8 m/s (2026-09-28).
PROFILE, I_MAX, I_TRIP = 'outrunner_63100_14p', 30.0, 45.0
ROTOR_J, ROTOR_B = 1.2e-4 + 8e-4, 8e-4

#: The rotors' speed loop, Hz, and the top the flight is flown on, mechanical rad/s: a 24 V
#: link's ceiling on the 63100 at a 50 A clamp, measured - 2 960 rpm, the q inductance's drop
#: beside the back-EMF; the flux's alone, 454, planned the burn on twice the thrust there was.
#: On the pack's 63 V the clamp's 30 A turn the propeller 432: 6.5 A are left to spool on. The
#: four's thrust there, N.
SPEED_HZ, TOP_RAD_S = 3.0, 310.0
TOP_N = 4.0 * quad.K_THRUST * TOP_RAD_S * TOP_RAD_S

#: A rotor's speed is asked no faster than this, rad/s^2: half the clamp to turn the can and
#: its propeller. Stepped, 14 rad/s more in a pass was the whole clamp, and the gate stage's
#: envelope 0.2 the higher for a read - 0.91 once in a flight (2026-10-05).
SPOOL_RAD_S2 = 700.0

#: The flight's step, s - the stand-in's boards answer out of memory -, the longest a pass
#: steps it, and the thermal reads' period, s. The flight's clock is its passes' sum: a pass in
#: twenty ran 50 ms, the routine's rows asked by the wall's clock moved further than their
#: pass and the rotors' loops went to their clamp on what that looked like (2026-10-05).
PHYSICS_S, PASS_S, THERMAL_S = 0.01, 0.05, 0.1

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
#: down; at once, a burn's own spend took it off its burn (2026-10-05).
SPEND, UNDER, TAKEN_S, RECOVER_S, GONE_S = 0.3, 0.08, 0.5, 2.0, 2.0

#: A spent pack is changed on the floor in this long, s.
SWAP_S = 3.0

#: A board's laminate to the air under its propeller, K/W, in its record: a third of the
#: bench's still air, an assumption - the wash over both faces. On the bench's 8.33 the
#: pack's 63 V had the gate stage at 0.65-0.72 of its envelope in a hover and every board
#: throttling by the corkscrew (2026-10-05).
WASH_K_PER_W = 2.5

#: The camera: the reach framed about the quad on the floor before a flight and in the air, m,
#: the pull from one to the other, s, its look down, degrees, its turn about the quad, degrees a
#: second, and how far it trails the climb, s of it, at most a share of the reach.
NEAR_M, FAR_M, PULL_S, PITCH, ORBIT_DEG_S = 0.8, 2.4, 1.5, 18.0, 3.0
YAW, TRAIL_S, TRAIL_SHARE = 30.0, 0.12, 0.4

#: A press of + or -, the reach framed over or times it, and the zoom's bounds.
ZOOM_STEP, ZOOM = 1.15, (0.25, 4.0)

#: A word for each observer's state, as the rotor page's TH OBS shows it.
WORD = {'UNCERTAIN': 'UNCR', 'CONVERGING': 'CONV', 'STABLE': 'STBL'}


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
    drive.configure(drv_i_max=I_MAX, drv_i_trip=I_TRIP)
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


def step(rotors, sky, route, flying, flight, clock, dt):
    """The flight `dt` s on, its stage: the routine's row asked of the law - taken down, its
    pack or the boards' envelopes spent - each rotor's loop after the law's thrust for it,
    within the share of their pull the envelopes leave; the propeller on each shaft, the
    pack's bus under what the four take, and the frame in MuJoCo on the rotors' thrust and
    drag. The burn's row falls first; where the routine waits to be fit a spent pack is
    changed and the boards cool."""
    cells, share = flight['cells'], flight['share']
    flat = cells['left'] <= quad.RESERVE
    name, flying.ask = aerobatics.fly(route, clock, [word for word, holds in (
        ('held', flying.held), ('spent', flat or flight['gone'] >= GONE_S),
        ('fit', not flat and share >= 1.0)) if holds])
    watts = 0.0
    for rotor, thrust in zip(rotors, flying.step(sky.state(), dt, share)):
        drive = rotor['rig'].board.drive
        now = drive.state()
        rotor['w_hat'] = (now.get('omega_hat') or 0.0) / rotor['pairs']
        rotor['amps'] = math.hypot(now.get('id') or 0.0, now.get('iq') or 0.0)
        watts += 1.5 * ((now.get('vd') or 0.0) * (now.get('id') or 0.0)
                        + (now.get('vq') or 0.0) * (now.get('iq') or 0.0))
        rotor['w'] = drive.model.read()['omega'] / rotor['pairs']
        rotor['angle'] = (rotor['angle'] + rotor['w'] * dt) % math.tau
        more = min(TOP_RAD_S, quad.speed_for(thrust)) - rotor['ask']
        rotor['ask'] += max(-SPOOL_RAD_S2 * dt, min(SPOOL_RAD_S2 * dt, more))
        rotor['iq'] = rotor['pi'].step(dt, setpoint=rotor['ask'],
                                       measured=rotor['w_hat'])['command']
        drive.write(iq_ref=rotor['iq'])
        drive.model.configure(load=quad.K_DRAG * rotor['w'] * abs(rotor['w']), vdc=cells['volts'])
    quad.drawn(cells, watts, dt)
    sky.step([r['w'] for r in rotors], dt)
    if 'fit' in route['card'][route['row']][4].split() and (flat or share < 1.0):
        return 'swap' if flat else 'cool'
    return FALL if name == 'burn' and flying.doing == FALL else name


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


def observer_word(ident):
    """An observer's state and the margin it acts on, `CONV 88` - or a dash before it answers."""
    state = (ident or {}).get('state')
    if state not in WORD:
        return '-'
    margin = int(round(100.0 * (ident.get('margin') or 0.0)))
    return WORD[state] if margin >= 100 else '%s %d' % (WORD[state], margin)


def observers(rotors):
    """The four observers in a row's words: who is stable, who converges, of how many where
    one is UNCERTAIN yet. `4 of 4 converging` it said whatever they were (2026-10-05)."""
    states = [(r['ident'] or {}).get('state') for r in rotors]
    known, stable = sum(s in GO for s in states), states.count('STABLE')
    if stable == len(states):
        return '%d of %d stable' % (stable, len(states))
    return '%d stable, %d converging%s' % (
        stable, known - stable, ' of %d' % len(states) if known < len(states) else '')


def compose(console, origin, rotors, frame, flight, trace, now, art):
    """One frame: the quad in the viewport; at the side the `flight` - its stage, its apex, its
    pack `cells`, their `peak`, the `share` of their pull its rotors are asked and how long it
    has been `gone` - the rotors, and `trace` drawn: the height and the power, the bus and the
    hottest board."""
    name, cells, peak, share = flight['stage'], flight['cells'], flight['peak'], flight['share']
    weight = quad.MASS_KG * quad.GRAVITY
    lift = sum(quad.K_THRUST * r['w'] * r['w'] for r in rotors)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(frame['turn'][1][1])))))
    worst = max(((r['budget'] or {}).get('worst') or 0.0) for r in rotors)
    flying = hud('FLIGHT', [
        ('stage', Text(name.upper(), style='alarm' if name in ('full tilt', 'burn', 'swap', 'cool')
                       else 'value')),
        ('height', '%8.2f m' % frame['h']),
        ('climb', '%+8.1f m/s' % frame['v']),
        ('pull', '%+8.1f g' % (frame['a'] / quad.GRAVITY)),
        ('thrust', '%8.1f of %.1f N' % (lift, weight)),
        ('tilt', '%8.1f deg, %.2f m off' % (tilt, math.hypot(frame['at'][0], frame['at'][2]))),
        ('apex', '%8.1f m' % flight['apex']),
        ('TH OBS', observers(rotors)),
        ('SOA', Text('%8.0f %% worst, pull %.0f %%' % (100.0 * worst, 100.0 * share),
                     style='alarm' if share < 1.0 else 'value')),
        ('bus', '%8.1f V, low %.1f' % (cells['volts'], peak['low'])),
        ('pack', Text('%8.0f %% left, %.1f A' % (100.0 * cells['left'], cells['amps']),
                      style='alarm' if cells['left'] <= quad.RESERVE else 'value')),
        ('power', '%8.0f W, peak %.0f' % (cells['watts'], peak['watts']))])
    lines = []
    for label, rotor in zip(ROTORS, rotors):
        budget = rotor['budget'] or {}
        hot = rotor['amps'] >= 0.9 * I_MAX or budget.get('throttling') or budget.get('tripped')
        lines.append((label, Text('%4.0f rpm %4.1f A SOA %3.0f%% %s' % (
            rotor['w'] * 60.0 / math.tau, rotor['amps'], 100.0 * (budget.get('worst') or 0.0),
            'THR' if budget.get('throttling') else observer_word(rotor['ident'])),
            style='alarm' if hot else 'value')))
    return frame_of(console, origin, TITLE, art,
                    [flying, hud('ROTORS  rpm, current, SOA, TH OBS', lines),
                     hud('HEIGHT m, POWER  last %.0f s' % traces.TRACE_S,
                         traces.heights(trace, now, HUD_WIDTH - 4)),
                     hud('BUS V, PEAK TEMP C', traces.buses(trace, now, HUD_WIDTH - 4))],
                    (('+ -', 'ZOOM'), ('R', 'RESET'), ('Q', 'EXIT'), ('ESC', 'MENU')))


def size_of(console):
    """Cells for the drawing: what the viewport leaves beside the side column."""
    size = console.size if console.is_terminal else None
    return (max(24, (size.width if size else 110) - HUD_WIDTH - 6),
            max(12, (size.height if size else 44) - 6))


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    # Every page takes them (terminal.loader.common); the rotors are the stand-in's for now,
    # simulated before emulated.
    parser.add_argument('--port', help='taken, not used: four stand-in boards fly')
    parser.add_argument('--simulated', action='store_true', help='taken: the rotors are')
    parser.add_argument('--hz', type=float, default=FPS_CAP,
                        help='screen refreshes per second, at most %.0f' % FPS_CAP)
    parser.add_argument('--frames', type=int, default=0,
                        help='stop after this many, instead of running until closed')
    args = parser.parse_args(argv)

    rotors = []
    with boot('LINKING FOUR ROTORS') as ready:
        try:
            for _ in ROTORS:
                rotors.append(arm(Coaxial63100(execution_mode=SIMULATED).open()))
        except RigError as exc:
            say('fail', 'rotors', str(exc))
            for rotor in rotors:
                rotor['rig'].close()
            return 1
        ready()
    origin = rotors[0]['rig'].origin
    say('warn', 'rotors', 'four stand-in boards on the pilot, sensorless on the 63100 under '
        'APC 20x10E')
    card = gpu.adapter()
    lit = gpu.LitRaster(found=card) if card is not None else None
    say('ok', 'drawing', lit.name if lit is not None else 'this process, dots')
    sky, route, trace = quad.Sky(), aerobatics.routine(), []
    began = time.monotonic()
    full = quad.open_volts(1.0)
    held = {'at': 0.0, 'clock': 0.0, 'thermal_at': 0.0, 'frame': sky.state(), 'reset': False,
            'flying': Flying(TOP_N, aerobatics.DOWN), 'swap': 0.0,
            'flight': {'stage': aerobatics.CARD[0][0], 'apex': 0.0, 'share': 1.0, 'gone': 0.0,
                       'cells': quad.pack(), 'peak': {'watts': 0.0, 'low': full}}}
    camera = {'reach': NEAR_M, 'yaw': YAW, 't': None, 'zoom': 1.0}

    def zoomed(k):
        camera['zoom'] = max(ZOOM[0], min(ZOOM[1], camera['zoom'] * k))
    keys = dict([(k, lambda: zoomed(ZOOM_STEP)) for k in '+=']
                + [(k, lambda: zoomed(1.0 / ZOOM_STEP)) for k in '-_']
                + [(k, lambda: held.update(reset=True)) for k in 'rR'])

    def sample():
        """The flight's side of a frame, on the feed's thread: steps of PHYSICS_S."""
        with suppress(RigError):
            now = time.monotonic() - began
            dt = min(PASS_S, max(0.0, now - held['at']))
            held['at'] = now
            if dt <= 0.0:
                return
            clock = held['clock'] = held['clock'] + dt
            flight = held['flight']
            cells = flight['cells']
            if held['reset']:
                # R: the frame back on its skids at its spot, the routine from its spool on a
                # fresh pack, the rotors' loops from rest - on this thread, the one that steps
                # the world.
                held['reset'] = False
                sky.reset()
                route.update(aerobatics.routine(), at=clock)
                held['flying'] = Flying(TOP_N, aerobatics.DOWN)
                reset(rotors)
                trace.clear()
                cells.update(quad.pack())
                flight.update(apex=0.0, share=1.0, gone=0.0)
                camera['reach'] = NEAR_M
            row = route['row']
            flight['share'] = envelope(rotors, flight['share'], dt)
            flight['gone'] = flight['gone'] + dt if flight['share'] <= 0.0 else 0.0
            stage = flight['stage'] = step(rotors, sky, route, held['flying'], flight, clock, dt)
            frame = held['frame'] = sky.state()
            held['swap'] = kept(flight, held['swap'], stage, route['row'] != row
                                and 'fit' in route['card'][row][4].split(), frame, dt)
            trace.append((clock, frame['h'], cells['volts'], cells['watts'],
                          max((r['board_c'] for r in rotors if r['board_c'] is not None),
                              default=None)))
            while trace and clock - trace[0][0] > traces.TRACE_S:
                trace.pop(0)
            if now - held['thermal_at'] >= THERMAL_S:
                held['thermal_at'] = now
                for rotor in rotors:
                    rotor['budget'], rotor['ident'], rotor['board_c'] = warmth(rotor['rig'])

    feed = Feed(sample, period=PHYSICS_S).start()
    board_view = stage()
    terminal = board_view.is_terminal
    leaving = None

    def draw():
        now = time.monotonic() - began
        frame = held['frame']
        dt = 0.0 if camera['t'] is None else now - camera['t']
        camera['t'] = now
        want = NEAR_M if held['flying'].ask['height'] <= 0.0 else FAR_M
        camera['reach'] += (want - camera['reach']) * min(1.0, dt / PULL_S)
        camera['yaw'] += ORBIT_DEG_S * dt
        reach, (x, y, z) = camera['reach'] / camera['zoom'], frame['at']
        trail = max(-TRAIL_SHARE * reach, min(TRAIL_SHARE * reach, TRAIL_S * frame['v']))
        width, height = size_of(board_view)
        art = '\n'.join(quadcopter.render(
            frame, [(r['angle'], r['w'], (r['budget'] or {}).get('winding_c'), r['board_c'])
                    for r in rotors], width, height, yaw=camera['yaw'], pitch=PITCH,
            reach=reach, centre=(x, y - trail, z), colour=terminal, lit=lit))
        return compose(board_view, origin, rotors, frame, held['flight'], list(trace),
                       held['clock'], art)

    try:
        leaving = run_view(board_view, terminal, 1.0 / max(args.hz, 0.5), args.frames, draw,
                           on_input=lambda typed, _moved: [keys[k]() for k in typed if k in keys])
    finally:
        feed.stop()
        done = []
        for rotor in rotors:
            with suppress(RigError):
                rotor['rig'].board.drive.off()
                rotor['rig'].gates.off()
            rotor['rig'].close()
        done.append(('rotors', 'four drives off, gates off, four stand-ins closed'))
        sys.stdout.write('\n')
        closing(done, terminal, 0)
    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
