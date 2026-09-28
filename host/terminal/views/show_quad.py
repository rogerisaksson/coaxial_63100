"""QUAD: four coaxial boards flying a frame in MuJoCo, full tilt into the sky, a stop 10 cm up.

A 2 kg frame (machine.quad) spooled on the floor, lifted to a hover and held there until the four
boards' thermal observers have left UNCERTAIN, then sent full tilt into the sky, fallen and burned
to a stop 10 cm over the floor, held and landed. Each rotor a stand-in board armed on the
master's pilot, its drive sensorless on the 63100 outrunner's model under an APC 20x10E
propeller, its speed loop on the drive's own estimate; the frame in MuJoCo on the four rotors'
thrust and drag, gravity and the air, held level over its spot by the mixer. The camera sits close
on the floor before a flight, pulls back as it lifts and follows it up, looking down on it
(coaxial.graphics.quadcopter); the height over the last half minute is a box at the side.

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
from coaxial.draw import braille
from coaxial.errors import RigError
from coaxial.graphics import gpu, quadcopter
from coaxial.model.thermal import MOTOR
from coaxial.simulated.sto import PILOT_VOLTS
from machine import quad
from machine.modes import SIMULATED
from machine.parts import SpeedPI
from terminal.loader import TO_MENU
from terminal.ui.screen import FPS_CAP, Feed, closing, run_view, say
from terminal.ui.scroll import HUD_WIDTH
from terminal.ui.stage import boot, frame_of, hud, stage

TITLE = 'QUAD'

#: The rotors, round the frame: front left, front right, rear right, rear left.
ROTORS = ('FL', 'FR', 'RR', 'RL')

#: Each rotor's machine, its clamp and trip for a flight, A, and the can and propeller turning,
#: kg m^2 and N m s (quad.json's propeller, the 63100's can). The hover takes 5.7 A; on a 50 A
#: clamp the spools into full tilt and the burn put the switches at 0.95 of the envelope a
#: second flight, throttled, and it burned into the floor at 7.8 m/s (2026-09-28).
PROFILE, I_MAX, I_TRIP = 'outrunner_63100_14p', 30.0, 45.0
ROTOR_J, ROTOR_B = 1.2e-4 + 8e-4, 8e-4

#: The rotors' speed loop, Hz, and their top, mechanical rad/s: the 24 V link's ceiling on
#: the 63100 at the clamp, measured - 2 960 rpm, the q inductance's drop at 50 A beside the
#: back-EMF; the flux's alone, 454, planned the burn on twice the thrust there was.
SPEED_HZ, TOP_RAD_S = 3.0, 310.0

#: The flight's step, s - the stand-in's boards answer out of memory - and the thermal reads'
#: period, s.
PHYSICS_S, THERMAL_S = 0.01, 0.25

#: The STO chain's charge on the master's pilot before the interlock passes, s, on the wall's
#: clock the stand-in's chain runs on: opened, a board's charge pump read 2.16 V of the 3.0
#: wanted and its level detector 0.31 of 2.0.
STO_SETTLE_S = 0.05

#: The observers' states full tilt may go on: out of UNCERTAIN, the margin off its floor. After
#: a 3 s hover, UNCERTAIN at 0.80, full tilt peaked at 31 A and 36 % of the envelope, none
#: throttled or tripped; STABLE came after 199 s of 1.5-16 A swings a minute and never in 900 s
#: held at 16 A, which a hover cannot give (2026-09-28).
GO = ('CONVERGING', 'STABLE')

#: The envelope's room full tilt waits for, its worst share on every board: a third flight
#: 12 s after the second's burn went at 0.46, throttled at 0.94 and climbed on 27 A.
ROOM = 0.4

#: The least of the clamp a derated board's spool is allowed for, of the whole.
DERATE_FLOOR = 0.25

#: The camera: the reach framed about the quad on the floor before a flight and in the air, m,
#: the pull from one to the other, s, its look down, degrees, its turn about the quad, degrees a
#: second, and how far it trails the climb, s of it, at most a share of the reach.
NEAR_M, FAR_M, PULL_S, PITCH, ORBIT_DEG_S = 0.8, 2.4, 1.5, 18.0, 3.0
YAW, TRAIL_S, TRAIL_SHARE = 30.0, 0.12, 0.4

#: A press of + or -, the reach framed over or times it, and the zoom's bounds.
ZOOM_STEP, ZOOM = 1.15, (0.25, 4.0)

#: The trace: its span, s, its top, m, the band drawn linear from the floor, m, and the share of
#: the height it takes; its rows in the side column.
TRACE_S, TRACE_TOP_M, LINEAR_M, LINEAR_SHARE, TRACE_ROWS = 30.0, 200.0, 2.0, 0.3, 8

#: A word for each observer's state, as the rotor page's TH OBS shows it.
WORD = {'UNCERTAIN': 'UNCR', 'CONVERGING': 'CONV', 'STABLE': 'STBL'}


def arm(rig):
    """A stand-in board's stage and drive for flight: its gates on the master's pilot, the
    63100's model under the propeller's inertia, a flight's clamp, sensorless."""
    board = rig.board
    board.afe.on()
    rig.pilot(PILOT_VOLTS)
    time.sleep(STO_SETTLE_S)
    board.gate_drivers.clear()
    rig.gates.on()
    drive = board.drive
    drive.configure(profile=PROFILE)
    drive.configure(drv_i_max=I_MAX, drv_i_trip=I_TRIP)
    drive.model.configure(j=ROTOR_J, b=ROTOR_B, load=0.0)
    drive.configure(source='model')
    drive.on('sensorless')
    params = drive.params()
    pairs = max(1.0, params.get('motor_pole_pairs') or 1.0)
    kt = 1.5 * pairs * (params.get('motor_lambda') or 0.002)
    return {'rig': rig, 'pairs': pairs, 'kt': kt, 'w': 0.0, 'w_hat': 0.0, 'iq': 0.0,
            'amps': 0.0, 'angle': 0.0,
            'pi': SpeedPI(SPEED_HZ, I_MAX, kt, ROTOR_J, ROTOR_B, quad.K_DRAG),
            'budget': {}, 'ident': {}, 'board_c': None}


def step(rotors, sky, route, clock, dt, ready):
    """The flight `dt` s on: the plan's thrust shared over the rotors to hold the frame level
    over its spot - full tilt once `ready` - each rotor's loop after its share, the propeller on
    each shaft, and the frame in MuJoCo on the four rotors' thrust and drag. The burn allows for
    the most derated board's spool."""
    top = 4.0 * quad.K_THRUST * TOP_RAD_S * TOP_RAD_S
    derate = min(((r['budget'] or {}).get('derate') or 1.0) for r in rotors)
    state = sky.state()
    name, thrust = quad.plan(route, state, top, clock, ready,
                             quad.SPOOL_S / max(DERATE_FLOOR, min(1.0, derate)))
    for rotor, share in zip(rotors, quad.mix(state, thrust)):
        drive = rotor['rig'].board.drive
        now = drive.state()
        rotor['w_hat'] = (now.get('omega_hat') or 0.0) / rotor['pairs']
        rotor['amps'] = math.hypot(now.get('id') or 0.0, now.get('iq') or 0.0)
        rotor['w'] = drive.model.read()['omega'] / rotor['pairs']
        rotor['angle'] = (rotor['angle'] + rotor['w'] * dt) % math.tau
        w_ref = min(TOP_RAD_S, quad.speed_for(share))
        rotor['iq'] = rotor['pi'].step(dt, setpoint=w_ref, measured=rotor['w_hat'])['command']
        drive.write(iq_ref=rotor['iq'])
        drive.model.configure(load=quad.K_DRAG * rotor['w'] * abs(rotor['w']))
    sky.step([r['w'] for r in rotors], dt)
    return name


def room(rotors):
    """Whether full tilt may go: every observer out of UNCERTAIN and every envelope with ROOM,
    none throttling."""
    return all((r['ident'] or {}).get('state') in GO
               and ((r['budget'] or {}).get('worst') or 0.0) <= ROOM
               and not (r['budget'] or {}).get('throttling') for r in rotors)


def reset(rotors):
    """Every rotor's speed loop from rest: the frame back on the floor spools them from idle."""
    for rotor in rotors:
        rotor['pi'] = SpeedPI(SPEED_HZ, I_MAX, rotor['kt'], ROTOR_J, ROTOR_B, quad.K_DRAG)


def warmth(rig):
    """(budget, identification, the board's hottest node C) of a rotor's thermal observer."""
    thermal = rig.board.thermal
    nodes = thermal.state().get('nodes') or {}
    board = [c for n, c in nodes.items() if n not in MOTOR and c is not None]
    return thermal.budget(), thermal.identification(), max(board) if board else None


def height_y(h):
    """A height on the trace's scale, 0 at the floor to 1 at its top: linear to LINEAR_M, then
    logarithmic."""
    if h <= LINEAR_M:
        return LINEAR_SHARE * max(0.0, h) / LINEAR_M
    return min(1.0, LINEAR_SHARE + (1.0 - LINEAR_SHARE) * math.log10(h / LINEAR_M)
               / math.log10(TRACE_TOP_M / LINEAR_M))


#: The trace's marks: heights labelled up its left edge, m.
MARKS = (0.0, 0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0)


def trace_art(trace, now, width, rows):
    """The height over the last TRACE_S as braille, the marks up its left, the stop's line."""
    plot = max(4, width - 7)
    dots_x, dots_y = plot * 2, rows * 4
    cells = [[0] * plot for _ in range(rows)]

    def dot(x, y):
        if 0 <= x < dots_x and 0 <= y < dots_y:
            cells[y // 4][x // 2] |= braille.BIT[x % 2][y % 4]
    floor_y = dots_y - 1 - int(round(height_y(quad.FLOOR_M) * (dots_y - 1)))
    for x in range(0, dots_x, 3):
        dot(x, floor_y)
    was = None
    for t, h in trace:
        x = int(round((1.0 - (now - t) / TRACE_S) * (dots_x - 1)))
        y = dots_y - 1 - int(round(height_y(h) * (dots_y - 1)))
        if was is not None and 0 <= x < dots_x:
            for k in range(1, abs(y - was[1]) + 1):
                dot(was[0] + (x - was[0]) * k // max(1, abs(y - was[1])),
                    was[1] + (1 if y > was[1] else -1) * k)
        dot(x, y)
        was = (x, y)
    labels = {rows - 1 - min(rows - 1, int(height_y(m) * (rows - 1) + 0.5)): m for m in MARKS}
    lines = []
    for r in range(rows):
        mark = labels.get(r)
        axis = ('%5s ┤' % ('%g' % mark)) if mark is not None else '      │'
        lines.append(LABEL_INK + axis + TRACE_INK
                     + ''.join(braille.ALL[bits] for bits in cells[r]) + RESET)
    return lines


#: The trace's inks, as the side column takes them: SGR over a string, not rich's styles.
LABEL_INK, TRACE_INK, RESET = '\x1b[38;5;244m', '\x1b[38;5;51m', '\x1b[0m'


def observer_word(ident):
    """An observer's state and the margin it acts on, `CONV 88` - or a dash before it answers."""
    state = (ident or {}).get('state')
    if state not in WORD:
        return '-'
    margin = int(round(100.0 * (ident.get('margin') or 0.0)))
    return WORD[state] if margin >= 100 else '%s %d' % (WORD[state], margin)


def compose(console, origin, rotors, frame, route, trace, now, apex, ready, art):
    """One frame: the quad in the viewport, the flight, its rotors and the height's trace at the
    side."""
    name = quad.STAGES[route['stage']][0]
    weight = quad.MASS_KG * quad.GRAVITY
    lift = sum(quad.K_THRUST * r['w'] * r['w'] for r in rotors)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(frame['turn'][1][1])))))
    known = sum((r['ident'] or {}).get('state') in GO for r in rotors)
    worst = max(((r['budget'] or {}).get('worst') or 0.0) for r in rotors)
    waiting = name == 'hover' and not ready
    why = ('' if not waiting else ', full tilt waits' if known < len(rotors)
           else ', cooling to %.0f %%' % (100.0 * ROOM))
    flying = hud('FLIGHT', [
        ('stage', Text(name.upper(), style='alarm' if name in ('full tilt', 'burn') else 'value')),
        ('height', '%8.2f m' % frame['h']),
        ('climb', '%+8.1f m/s' % frame['v']),
        ('pull', '%+8.1f g' % (frame['a'] / quad.GRAVITY)),
        ('thrust', '%8.1f of %.1f N' % (lift, weight)),
        ('tilt', '%8.1f deg, %.2f m off' % (tilt, math.hypot(frame['at'][0], frame['at'][2]))),
        ('apex', '%8.1f m' % apex),
        ('TH OBS', Text('%d of 4 converging%s' % (known, why),
                        style='alarm' if waiting else 'value')),
        ('SOA', '%8.0f %% worst' % (100.0 * worst))])
    lines = []
    for label, rotor in zip(ROTORS, rotors):
        budget = rotor['budget'] or {}
        hot = rotor['amps'] >= 0.9 * I_MAX or budget.get('throttling') or budget.get('tripped')
        lines.append((label, Text('%4.0f rpm %4.1f A SOA %3.0f%% %s' % (
            rotor['w'] * 60.0 / math.tau, rotor['amps'], 100.0 * (budget.get('worst') or 0.0),
            'THR' if budget.get('throttling') else observer_word(rotor['ident'])),
            style='alarm' if hot else 'value')))
    height = hud('HEIGHT  last %.0f s' % TRACE_S, trace_art(trace, now, HUD_WIDTH - 4, TRACE_ROWS))
    return frame_of(console, origin, TITLE, art,
                    [flying, hud('ROTORS  rpm, current, SOA, TH OBS', lines), height],
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
    sky, route, trace = quad.Sky(), quad.flight(), []
    began = time.monotonic()
    held = {'at': 0.0, 'thermal_at': 0.0, 'apex': 0.0, 'frame': sky.state(), 'ready': False,
            'reset': False}
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
            dt = min(0.05, max(0.0, now - held['at']))
            held['at'] = now
            if dt <= 0.0:
                return
            if held['reset']:
                # R: the frame back on its skids at its spot, the flight from its spool, the
                # rotors' loops from rest - on this thread, the one that steps the world.
                held['reset'] = False
                sky.reset()
                route.update(quad.flight(), at=now)
                reset(rotors)
                trace.clear()
                held['apex'] = 0.0
                camera['reach'] = NEAR_M
            step(rotors, sky, route, now, dt, held['ready'])
            frame = held['frame'] = sky.state()
            held['apex'] = frame['h'] if quad.STAGES[route['stage']][0] == 'full tilt' \
                else max(held['apex'], frame['h'])
            trace.append((now, frame['h']))
            while trace and now - trace[0][0] > TRACE_S:
                trace.pop(0)
            if now - held['thermal_at'] >= THERMAL_S:
                held['thermal_at'] = now
                for rotor in rotors:
                    rotor['budget'], rotor['ident'], rotor['board_c'] = warmth(rotor['rig'])
                held['ready'] = room(rotors)

    feed = Feed(sample, period=PHYSICS_S).start()
    board_view = stage()
    terminal = board_view.is_terminal
    leaving = None

    def draw():
        now = time.monotonic() - began
        frame = held['frame']
        dt = 0.0 if camera['t'] is None else now - camera['t']
        camera['t'] = now
        want = NEAR_M if quad.STAGES[route['stage']][0] == 'spool' else FAR_M
        camera['reach'] += (want - camera['reach']) * min(1.0, dt / PULL_S)
        camera['yaw'] += ORBIT_DEG_S * dt
        reach, (x, y, z) = camera['reach'] / camera['zoom'], frame['at']
        trail = max(-TRAIL_SHARE * reach, min(TRAIL_SHARE * reach, TRAIL_S * frame['v']))
        width, height = size_of(board_view)
        art = '\n'.join(quadcopter.render(
            frame, [(r['angle'], r['w'], (r['budget'] or {}).get('winding_c'), r['board_c'])
                    for r in rotors], width, height, yaw=camera['yaw'], pitch=PITCH,
            reach=reach, centre=(x, y - trail, z), colour=terminal, lit=lit))
        return compose(board_view, origin, rotors, frame, route, list(trace), now,
                       held['apex'], held['ready'], art)

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
