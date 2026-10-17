"""QUAD: four coaxial boards flying a frame in MuJoCo, its routine, then gates among trees.

A 2 kg frame (machine.quad) on four stand-in boards, each armed on the master's pilot, its
drive sensorless on the 63100 outrunner's model under a 16x14 propeller, its speed loop
on the drive's own estimate; the four on one pack, 63 V full, its bus drooping under what they
take; the frame in MuJoCo on the rotors' thrust and drag, gravity and the air
(terminal.views.quad.flight). It flies flight after flight: first its routine
(machine.aerobatics' rows, machine.aerokin's payloads on aero/) - a pirouette, an orbit and a corkscrew up, a roll
and a flip over a toss, full tilt into the sky, the fall burned to a stop 10 cm over the floor -
its boards and their thermal observers warmed on it; then the course (machine.course): two laps
through sixteen gates - two slaloms, a climb, a dive, a hall's windows, an alley - raw, as fast
as its plan's sphere takes it, a dive pushed down inverted. What stands is solid: struck, the
flight is over, the wreck lies where it falls and the flight is begun again. Its rows have the
pull the action space has for them (machine.pull): the rotors' on what each board holds of
its clamp - WEP's at the most - and on the pack's charge; spent, it comes down for a charged
pack, or to cool. It is flown in the air's weather (machine.quad): a wind, gusts over it and eddies in it,
a kind at a time or one kept (W), none of it told to the law - its way and size a pointer at
the side (terminal.views.quad.wind). The camera sits close on the floor before a flight, pulls
back into a side RC-pilot perspective as it lifts - zooming a little to follow it while keeping
contact with the horizon, and on the course coming round behind it as it turns
(coaxial.graphics.quadcopter, scenery); the height and the power, the bus and the hottest
board over the last half minute are boxes at the side (terminal.views.quad.traces). In the
viewport's corner the picture of the camera it carries on a stabilised gimbal (machine.gimbal,
coaxial.graphics.onboard), there or not (C).

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
from coaxial.errors import RigError
from coaxial.draw.gauges import policy_word
from coaxial.graphics import gpu, inset, onboard, quadcopter
from machine import aerobatics, aerokin, gimbal, grounds, pull, quad
from machine.modes import SIMULATED
from terminal.loader import TO_MENU
from terminal.ui import recorder
from terminal.ui.screen import FPS_CAP, Feed, closing, run_view, say
from terminal.ui.scroll import HUD_WIDTH
from terminal.ui.stage import boot, frame_of, hud, stage
from terminal.views.quad import flight as flown
from terminal.views.quad import traces, wind

TITLE = 'QUAD'

#: The rotors, round the frame: front left, front right, rear right, rear left.
ROTORS = ('FL', 'FR', 'RR', 'RL')

#: The feed's period, s - the stand-in's boards answer out of memory -, and the longest a pass
#: takes the flight on, in its steps (`flight.steps`). The flight's clock is its steps' sum: a
#: pass in twenty ran 50 ms, the routine's rows asked by the wall's clock moved further than
#: their pass and the rotors' loops went to their clamp on what that looked like.
PHYSICS_S, PASS_S = 0.01, 0.05

#: The interpreter's threads change hands this often while the page flies, s: the flight's
#: thread asks its boards and its core 40 000 times a second and gives way at each. At
#: Python's own 5 ms it waited on the drawing's, a flight's second took 1.02 of the wall's
#: and its clock ran 21 % behind; here 0.52, none behind, 15.6 frames a second where 16.3.
SWITCH_S = 2e-4

#: The camera: sit close on the floor before a flight, pull back in the air and zoom a little
#: to follow altitude and box excursion while keeping contact with the horizon from a side
#: RC-pilot perspective (`YAW`, `PILOT_M` off the line).
NEAR_M, FAR_M, ZOOM_M, ZOOM_K, PULL_S = 0.8, 1.5, 0.7, 0.04, 1.5
YAW, SIDE_PITCH, PITCH, PILOT_M = 30.0, 18.0, 18.0, 14.0
HORIZON_M, HORIZON_DIP, HORIZON_LIFT = 150.0, 9.0, 0.25
TRAIL_S, TRAIL_SHARE = 0.12, 0.4

#: On the course: the reach framed, m; the camera comes round behind the frame's heading at
#: this gain, 1/s, and frames a point this far ahead of it, s of its speed.
CHASE_M, CHASE_K, CHASE_S = 4.0, 2.0, 0.3

#: A press of + or -, the reach framed over or times it, and the zoom's bounds.
ZOOM_STEP, ZOOM = 1.15, (0.25, 4.0)


def thrusts(frame, rotors):
    """Each rotor's thrust, N, as the frame flies: its speed's, less the air through its disc."""
    turn = frame['turn']
    along = sum(float(v) * float(turn[k][1]) for k, v in enumerate(frame['vel']))
    return [quad.K_THRUST * r['w'] * r['w'] * quad.inflow(along, r['w']) for r in rotors]


def compose(console, origin, rotors, frame, flight, trace, now, art, yaw=0.0):
    """One frame: the quad in the viewport; at the side the `flight` - its stage, its apex or
    its `lap`, its pack `cells`, their `peak`, the `share` of the clamp's pull its rows have
    or, a rotor past the flight's clamp, the seconds `over` it - its `air`'s wind as the
    view, `yaw` degrees round, has it, the rotors, and `trace` drawn: the height and the
    power, the bus and the hottest board."""
    name, cells, peak, share = flight['stage'], flight['cells'], flight['peak'], flight['share']
    weight = quad.MASS_KG * quad.GRAVITY
    lift = sum(quad.K_THRUST * r['w'] * r['w'] for r in rotors)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(frame['turn'][1][1])))))
    worst = max(((r['budget'] or {}).get('worst') or 0.0) for r in rotors)
    lap, over = flight.get('lap'), max(r['amps'] for r in rotors) > pull.I_MAX
    tight = min((r['budget'] or {}).get('hold_amps', pull.I_MAX) for r in rotors) < pull.I_MAX
    flying = hud('FLIGHT', [
        ('stage', Text(('%s %s' % (name, flown.struck(flight))).upper().strip(), style='alarm'
                       if name in ('full tilt', 'burn', 'swap', 'cool', flown.CRASHED)
                       else 'value')),
        ('height', '%5.2f m, %+.1f m/s, %+.1f g' % (frame['h'], frame['v'],
                                                    frame['a'] / quad.GRAVITY)),
        ('thrust', '%8.1f of %.1f N' % (lift, weight)),
        ('tilt', '%8.1f deg, %.2f m off' % (tilt, math.hypot(frame['at'][0], frame['at'][2]))),
        ('lap', '%8s gate %d, %.1f m/s' % (
            '%d/%d' % (lap['laps'] + 1, lap['of']), lap['gate'],
            math.hypot(float(frame['vel'][0]), float(frame['vel'][2]))))
        if lap else ('apex', '%8.1f m' % flight['apex']),
        ('TH OBS', flown.observers(rotors)),
        ('SOA', Text('%8.0f %% worst, %s' % (100.0 * worst, 'WEP %.1f s' % flight['over']
                                             if over else 'pull %.0f %%' % (100.0 * share)),
                     style='alarm' if tight or over else 'value')),
        ('bus', '%8.1f V, low %.1f' % (cells['volts'], peak['low'])),
        ('pack', Text('%8.0f %% left, %.1f A' % (100.0 * cells['left'], cells['amps']),
                      style='alarm' if cells['left'] <= quad.RESERVE else 'value')),
        ('power x4', '%8.0f W, peak %.0f' % (cells['watts'], peak['watts']))])
    lines = []
    for label, rotor in zip(ROTORS, rotors):
        budget = rotor['budget'] or {}
        held = budget.get('hold_amps', pull.I_MAX) < pull.I_MAX
        hot = rotor['amps'] >= 0.9 * pull.I_MAX or held or budget.get('tripped')
        lines.append((label, Text('%4.0f rpm %4.1f A %3.0f%% %s' % (
            rotor['w'] * 60.0 / math.tau, rotor['amps'], 100.0 * (budget.get('worst') or 0.0),
            'THR' if held else policy_word(rotor['ident'])[0]),
            style='alarm' if hot else 'value')))
    sinks = [r.get('cooling') or {} for r in rotors]
    sink_c = max((float(s.get('sink_c', 25.0)) for s in sinks), default=25.0)
    fan_pct = 100.0 * max((float(s.get('fan_duty', 0.0)) for s in sinks), default=0.0)
    fan_rpm = max((float(s.get('fan_rpm', 0.0)) for s in sinks), default=0.0)
    lines.append(('SINK', Text('%4.1f C  fan %3.0f%%  %4.0f rpm' % (sink_c, fan_pct, fan_rpm),
                               style='alarm' if fan_pct >= 80.0 else 'value')))
    return frame_of(console, origin, TITLE, art,
                    [flying, hud('WIND  the way it blows, in the view',
                                 wind.pointer(flight['air'], frame['h'], yaw)),
                     hud('ROTORS  rpm, current, SOA, TH OBS', lines),
                     hud('HEIGHT m, POWER x4  last %.0f s' % traces.TRACE_S,
                         traces.heights(trace, now, HUD_WIDTH - 4)),
                     hud('BUS V, PEAK TEMP C', traces.buses(trace, now, HUD_WIDTH - 4))],
                    (('+ -', 'ZOOM'), ('W', 'WIND'), ('T', 'THRUST'), ('L', 'LINE'),
                     ('C', 'CAMERA'), ('R', 'RECORD'), ('A', 'RESET'), ('Q', 'EXIT'), ('ESC', 'MENU')))


#: R's recording on a flying page: a row a pass of the flight, the keys typed since the row
#: before, then each rotor's rad/s.
FLIGHT = (('t', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'height', 'speed')
          + tuple('turn_%d%d' % (i, j) for i in range(3) for j in range(3)) + ('volts', 'watts', 'keys'))


def flown_row(clock, frame, cells, rotors, keys=''):
    """A recording's row (FLIGHT, then each rotor's rad/s) of a pass at `clock`."""
    return ([round(clock, 4)] + [float(v) for v in frame['at']] + [float(v) for v in frame['vel']]
            + [frame['h'], frame['v']] + [float(v) for v in frame['turn'].ravel()]
            + [cells.get('volts'), cells.get('watts'), keys] + [float(r['w']) for r in rotors])


def recorded(held, name, rotors):
    """R: recording the flight from now; again: written to the recordings."""
    if held.get('recording') is None:
        held['recording'] = recorder.started(name, FLIGHT + tuple('w_%d' % k for k in range(len(rotors))))
        return
    rec, held['recording'] = held['recording'], None
    held['recorded'] = recorder.saved(rec)


def typed(held, keys, keyed):
    """`keyed` on the page's `keys`, each kept for the recording's next row."""
    held['keyed'] = held.get('keyed', '') + ''.join(k for k in keyed if isinstance(k, str))
    for k in keyed:
        if k in keys:
            keys[k]()


def carried(frame, aim, gate, line_, width, height):
    """The carried camera's picture for `quadcopter.render`'s `inset`: the world from its lens
    on the `frame`, its sight `aim` (`gimbal.step`), with the course's gates where `gate` is the
    one flown to next - how far off it is the picture's note - and its line with `line_`."""
    eye = gimbal.lens(frame['at'], frame['turn'])
    note = '' if gate is None else 'GATE %d  %.0f m' % (
        gate, math.dist(eye, grounds.GATES[gate % len(grounds.GATES)][:3]))
    return inset.docked(onboard.picture(eye, aim, gate, line_), width, height, 'GIMBAL', note)


def hands(held, zoomed, record=None):
    """{key: what it does}: the page's keys on its `held` state, `zoomed` the view's zoom times
    its argument, `record` R's."""
    return dict([(k, lambda: zoomed(ZOOM_STEP)) for k in '+=']
                + [(k, lambda: zoomed(1.0 / ZOOM_STEP)) for k in '-_']
                + [(k, lambda: held.update(reset=True)) for k in 'aA']
                + [(k, record or (lambda: None)) for k in 'rR']
                + [(k, lambda: held.update(wind=True)) for k in 'wW']
                + [(k, lambda: held.update(arrows=not held['arrows'])) for k in 'tT']
                + [(k, lambda: held.update(line=not held['line'])) for k in 'lL']
                + [(k, lambda: held.update(carried=not held['carried'])) for k in 'cC'])


def look(camera, law, row, frame, chased, dt, near_m=NEAR_M, far_m=FAR_M, zoom_m=ZOOM_M,
         zoom_k=ZOOM_K, chase_m=CHASE_M, base_yaw=YAW, chase_pitch=PITCH):
    """The camera `dt` s on, `(reach, centre)`: during aerobatics (`not chased`), a side RC-pilot
    perspective (`base_yaw`) that pans gently across the flight box, zooms a little to follow
    altitude and horizontal excursion, and keeps visual contact with the horizon; on the course
    (`chased`), wider (`chase_m`), come round behind the law's heading and framing ahead of it."""
    aloft = row['height'] > 0.0
    x, y, z = (float(c) for c in frame['at'])
    dist = math.hypot(max(0.0, y), 0.5 * math.hypot(x, z))
    want = near_m if not aloft else chase_m if chased else far_m + min(zoom_m, zoom_k * dist)
    camera['reach'] += (want - camera['reach']) * min(1.0, dt / PULL_S)
    reach = camera['reach'] / camera['zoom']
    trail = max(-TRAIL_SHARE * reach, min(TRAIL_SHARE * reach, TRAIL_S * frame['v']))
    pitch = camera.get('pitch', SIDE_PITCH)
    if not chased:
        lift = min(0.5 * max(0.0, y - 0.16), HORIZON_LIFT * min(reach, camera['reach']))
        shift = max(-TRAIL_SHARE * reach, min(TRAIL_SHARE * reach, lift + TRAIL_S * frame['v']))
        cy_w = max(0.16, y - shift)
        rad = math.radians(base_yaw)
        across = z * math.sin(rad) - x * math.cos(rad)
        along = x * math.sin(rad) + z * math.cos(rad)
        want_yaw = base_yaw + math.degrees(math.atan2(across, max(4.0, PILOT_M - along)))
        turn = (want_yaw - camera['yaw'] + 180.0) % 360.0 - 180.0
        camera['yaw'] += turn * min(1.0, CHASE_K * dt)
        horizon_deg = math.degrees(math.atan2(cy_w, HORIZON_M))
        want_pitch = horizon_deg + SIDE_PITCH - HORIZON_DIP * min(1.0, max(0.0, cy_w - 2.0) / 16.0)
        camera['pitch'] = pitch + (want_pitch - pitch) * (1.0 if dt <= 0.0 else min(1.0, 8.0 * dt))
        hx = max(-0.2 * reach, min(0.2 * reach, TRAIL_S * float(frame['vel'][0])))
        hz = max(-0.2 * reach, min(0.2 * reach, TRAIL_S * float(frame['vel'][2])))
        return reach, (x - hx, cy_w, z - hz)
    heading = math.degrees(math.atan2(law['heading'][1], law['heading'][0]))
    behind = (heading + 180.0 - camera['yaw'] + 180.0) % 360.0 - 180.0
    camera['yaw'] += behind * min(1.0, CHASE_K * dt)
    camera['pitch'] = pitch + (chase_pitch - pitch) * (1.0 if dt <= 0.0 else min(1.0, CHASE_K * dt))
    ahead = [CHASE_S * float(frame['vel'][axis]) for axis in (0, 2)]
    far = max(1.0, math.hypot(ahead[0], ahead[1]) / (TRAIL_SHARE * reach))
    return reach, (x + ahead[0] / far, max(0.16, y - trail), z + ahead[1] / far)


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
                rotors.append(flown.arm(Coaxial63100(execution_mode=SIMULATED).open()))
        except RigError as exc:
            say('fail', 'rotors', str(exc))
            for rotor in rotors:
                rotor['rig'].close()
            return 1
        ready()
    origin = rotors[0]['rig'].origin
    say('warn', 'rotors', 'four stand-in boards on the pilot, sensorless on the 63100 under '
        '16x14s')
    card = gpu.adapter()
    lit = gpu.LitRaster(found=card) if card is not None else None
    say('ok', 'drawing', lit.name if lit is not None else 'this process, dots')
    sky, route, trace = quad.Sky(grounds.solids()), aerobatics.routine(flown.CARD), []
    began = time.monotonic()
    held = {'at': 0.0, 'clock': 0.0, 'frame': sky.state(), 'reset': False,
            'wind': False, 'law': aerokin.fresh(),
            'flight': flown.fresh(), 'arrows': True, 'line': True, 'carried': True,
            'aim': gimbal.REST}
    camera = {'reach': NEAR_M, 'yaw': YAW, 'pitch': SIDE_PITCH, 't': None, 'zoom': 1.0}

    def zoomed(k):
        camera['zoom'] = max(ZOOM[0], min(ZOOM[1], camera['zoom'] * k))
    keys = hands(held, zoomed, lambda: recorded(held, 'quad', rotors))

    def sample():
        """The flight's side of a frame, on the feed's thread: a pass of it."""
        with suppress(RigError):
            now = time.monotonic() - began
            dt = min(PASS_S, max(0.0, now - held['at']))
            held['at'] = now
            if dt <= 0.0:
                return
            flight = held['flight']
            cells = flight['cells']
            if held['wind']:
                # W: the air's next kind, kept - on the thread that blows it.
                held['wind'] = False
                quad.kept(flight['air'])
            if held['reset']:
                # A: the frame back on its skids at its spot, its flights from the routine's
                # spool on a fresh pack, the rotors' loops from rest - on this thread, the one
                # that steps the world.
                held['reset'] = False
                sky.reset()
                route.pop('lap', None)
                route.update(aerobatics.routine(flown.CARD), at=held['clock'])
                flown.begun(held['law'], flight)
                flown.reset(rotors)
                trace.clear()
                cells.update(quad.pack())
                flight.update(apex=0.0, share=1.0, gone=0.0, stood=0.0, wreck=None)
                camera.update(reach=NEAR_M, yaw=YAW, pitch=SIDE_PITCH)
            for clock, frame in flown.passed(rotors, sky, route, held['law'], flight,
                                             held['clock'], dt):
                held['aim'] = gimbal.step(held['aim'], frame['turn'], frame['vel'],
                                          clock - held['clock'])
                held['clock'], held['frame'] = clock, frame
                trace.append((clock, frame['h'], cells['volts'], cells['watts'],
                              max((r['board_c'] for r in rotors if r['board_c'] is not None),
                                  default=None)))
                if held.get('recording') is not None:
                    recorder.added(held['recording'], [flown_row(clock, frame, cells, rotors, held.pop('keyed', ''))])
            while trace and held['clock'] - trace[0][0] > traces.TRACE_S:
                trace.pop(0)

    switched = sys.getswitchinterval()
    sys.setswitchinterval(SWITCH_S)
    feed = Feed(sample, period=PHYSICS_S).start()
    board_view = stage()
    terminal = board_view.is_terminal
    leaving = None

    def draw():
        now = time.monotonic() - began
        frame = held['frame']
        dt = 0.0 if camera['t'] is None else now - camera['t']
        camera['t'] = now
        chased = flown.lined(route)
        reach, centre = look(camera, held['law'], held['flight']['row'], frame, chased, dt)
        width, height = size_of(board_view)
        gate = (held['flight']['lap'] or {}).get('gate', 1) if chased else None
        art = '\n'.join(quadcopter.render(
            frame, [(r['angle'], r['w'], (r['budget'] or {}).get('winding_c'), r['board_c'],
                     (r.get('cooling') or {}).get('sink_c'))
                    for r in rotors], width, height, yaw=camera['yaw'], pitch=camera['pitch'],
            reach=reach, centre=centre, colour=terminal, lit=lit, gate=gate,
            thrusts=thrusts(frame, rotors) if held['arrows'] else None,
            line_=chased and held['line'],
            inset={**(carried(frame, held['aim'], gate, chased and held['line'], width, height)
                      if held['carried'] else {}), **recorder.corner(held.get('recording'), width)}))
        return compose(board_view, origin, rotors, frame, held['flight'], list(trace),
                       held['clock'], art, yaw=camera['yaw'])

    try:
        leaving = run_view(board_view, terminal, 1.0 / max(args.hz, 0.5), args.frames, draw,
                           on_input=lambda keyed, _moved: typed(held, keys, keyed))
    finally:
        feed.stop()
        sys.setswitchinterval(switched)
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
