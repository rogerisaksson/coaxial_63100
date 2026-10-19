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
the side (terminal.views.quad.wind). Its routine is flown on the airfield, watched from beside its
runway, the course among its grounds, chased (machine.grounds.world, coaxial.graphics.quadcopter,
scenery); the height and the power, the bus and the hottest
board over the last half minute are boxes at the side (terminal.views.quad.traces). In the
viewport's corner the picture of the camera it carries on a stabilised gimbal (machine.gimbal,
coaxial.graphics.onboard), there or not (C).

    python terminal/views/show_quad.py
    python terminal/views/show_quad.py --frames 60
"""
import argparse
import functools
import math
import sys

from rich.text import Text

from coaxial.draw.gauges import policy_word
from coaxial.graphics import engine, inset, onboard, quadcopter
from machine import airfield, flightpack, gimbal, grounds, pull, quad
from terminal.loader import TO_MENU
from terminal.ui import recorder
from terminal.ui.console import WHEEL_STEP
from terminal.ui.ahead import Ahead
from terminal.ui.screen import FPS_CAP, closing, drawn_next, run_view, say
from terminal.ui.scroll import HUD_WIDTH
from terminal.ui.stage import boot, frame_of, hud, stage
from terminal.views.playback import Playback
from terminal.views.quad import flight as flown
from terminal.views.quad import flying, traces, wind

TITLE = 'QUAD ACRO FLYING'

#: The rotors, round the frame: front left, front right, rear right, rear left.
ROTORS = ('FL', 'FR', 'RR', 'RL')

#: The spectator of every page's figures: beside the runway's middle (`airfield.placed`),
#: SPOT_ACROSS off its line by the hangar, SPOT_Y up; gaze LEAD_S ahead, lens over LENS_S.
SPOT_ALONG, SPOT_ACROSS, SPOT_Y = -2.0, -10.0, 1.7
SPOT = airfield.placed(SPOT_ALONG, SPOT_ACROSS)
LEAD_S, LENS_S, FILL, FILL_MIN, HORIZON_RAD = 0.15, 0.6, 0.4, 0.3, 0.035
UP_FADE = (0.35, 0.79)
#: Before a flight and on the course: the reach framed on the floor, m, its pull over PULL_S;
#: the side the spectator stands on and the chase's pitch, deg.
NEAR_M, PULL_S, YAW, SIDE_PITCH, PITCH = 0.8, 1.5, 30.0, 18.0, 18.0
TRAIL_S, TRAIL_SHARE = 0.12, 0.4

#: On the course: the reach framed, m; the camera comes round behind the frame's heading.
CHASE_M, CHASE_K, CHASE_S = 4.0, 2.0, 0.3

#: A press of + or -, the reach framed over or times it, and the zoom's bounds.
ZOOM_STEP, ZOOM = 1.15, (0.25, 4.0)


def thrusts(frame, rotors):
    """Each rotor's thrust, N, as the frame flies: its speed's, less the air through its disc."""
    turn = frame['turn']
    along = sum(float(v) * float(turn[k][1]) for k, v in enumerate(frame['vel']))
    return [quad.K_THRUST * r['w'] * r['w'] * quad.inflow(along, r['w']) for r in rotors]


def compose_craft(console, origin, rotors, frame, flight, trace, now, art, yaw, title, craft,
                  flown_mod, extra_row=None):
    """One frame for any coaxial aircraft: the 3D viewport and the 5-box side HUD (`FLIGHT`,
    `WIND`, `ROTORS`, `HEIGHT/POWER`, `BUS/PEAK TEMP`)."""
    name, cells, peak, share = flight['stage'], flight['cells'], flight['peak'], flight['share']
    n_rotors = len(rotors)
    weight = craft.MASS_KG * craft.GRAVITY
    if extra_row is None:
        lift = sum(craft.K_THRUST * r['w'] * r['w'] for r in rotors)
        height_rows = [('height', '%5.2f m, %+.1f m/s, %+.1f g' % (
            frame['h'], frame['v'], frame['a'] / craft.GRAVITY))]
        thrust_fmt, power_hdr = '%8.1f of %.1f N', 'HEIGHT m, POWER x4  last %.0f s'
    else:
        ctrl0 = frame.get('controls', flight.get('controls', (0.0, 0.0, 0.0, 0.0)))[0]
        lift = float(frame.get('lift', craft.K_THRUST * rotors[0]['w'] ** 2 * ctrl0))
        height_rows = [('height', '%8.2f m' % frame['h']),
                       ('climb', '%+8.1f m/s, %+.1f g' % (frame['v'], frame['a'] / craft.GRAVITY))]
        thrust_fmt, power_hdr = '%+8.1f of %.1f N', 'HEIGHT m, POWER OF BOTH  last %.0f s'
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(frame['turn'][1][1])))))
    worst = max(((r['budget'] or {}).get('worst') or 0.0) for r in rotors)
    lap, over = flight.get('lap'), max(r['amps'] for r in rotors) > pull.I_MAX
    tight = min((r['budget'] or {}).get('hold_amps', pull.I_MAX) for r in rotors) < pull.I_MAX
    flying = hud('FLIGHT', [
        ('stage', Text(('%s %s' % (name, flown_mod.struck(flight))).upper().strip(), style='alarm'
                       if name in ('full tilt', 'burn', 'swap', 'cool', flown.CRASHED)
                       else 'value')),
        *height_rows,
        ('thrust', thrust_fmt % (lift, weight)),
        ('tilt', '%8.1f deg, %.2f m off' % (tilt, math.hypot(frame['at'][0], frame['at'][2]))),
        ('lap', '%8s gate %d, %.1f m/s' % (
            '%d/%d' % (lap['laps'] + 1, lap['of']), lap['gate'],
            math.hypot(float(frame['vel'][0]), float(frame['vel'][2]))))
        if lap else ('apex', '%8.1f m' % flight['apex']),
        ('TH OBS', flown_mod.observers(rotors)),
        ('SOA', Text('%8.0f %% worst, %s' % (100.0 * worst, 'WEP %.1f s' % flight['over']
                                             if over else 'pull %.0f %%' % (100.0 * share)),
                     style='alarm' if tight or over else 'value')),
        ('bus', '%8.1f V, low %.1f' % (cells['volts'], peak['low'])),
        ('pack', Text('%8.0f %% left, %.1f A' % (100.0 * cells['left'], cells['amps']),
                      style='alarm' if cells['left'] <= flightpack.RESERVE else 'value')),
        ('power x%d' % n_rotors, '%8.0f W, peak %.0f' % (cells['watts'], peak['watts']))])
    lines = []
    for label, rotor in zip(getattr(flown_mod, 'ROTORS', ROTORS), rotors):
        budget = rotor['budget'] or {}
        held = budget.get('hold_amps', pull.I_MAX) < pull.I_MAX
        hot = rotor['amps'] >= 0.9 * pull.I_MAX or held or budget.get('tripped')
        lines.append((label, Text('%4.0f rpm %4.1f A %3.0f%% %s' % (
            rotor['w'] * 60.0 / math.tau, rotor['amps'], 100.0 * (budget.get('worst') or 0.0),
            'THR' if held else policy_word(rotor['ident'])[0]),
            style='alarm' if hot else 'value')))
    if extra_row is not None:
        lines.append(extra_row)
    sinks = [r.get('cooling') or {} for r in rotors]
    sink_c = max((float(s.get('sink_c', 25.0)) for s in sinks), default=25.0)
    fan_pct = 100.0 * max((float(s.get('fan_duty', 0.0)) for s in sinks), default=0.0)
    fan_rpm = max((float(s.get('fan_rpm', 0.0)) for s in sinks), default=0.0)
    lines.append(('SINK', Text('%4.1f C  fan %3.0f%%  %4.0f rpm' % (sink_c, fan_pct, fan_rpm),
                               style='alarm' if fan_pct >= 80.0 else 'value')))
    return frame_of(console, origin, title, art,
                    [flying, hud('WIND  the way it blows, in the view',
                                 wind.pointer(flight['air'], frame['h'], yaw)),
                     hud('ROTORS  rpm, current, SOA, TH OBS', lines),
                     hud(power_hdr % traces.TRACE_S, traces.heights(trace, now, HUD_WIDTH - 4)),
                     hud('BUS V, PEAK TEMP C', traces.buses(trace, now, HUD_WIDTH - 4))],
                    (('+ -', 'ZOOM'), ('W', 'WIND'), ('T', 'THRUST'), ('S', 'LAYER'), ('L', 'LINE'),
                     ('C', 'CAMERA'), ('R', 'RECORD'), ('A', 'RESET'), ('Q', 'EXIT'), ('ESC', 'MENU')))


def compose(console, origin, rotors, frame, flight, trace, now, art, yaw=0.0):
    """One frame: the quad in the viewport; at the side the `flight` - its stage, its apex or
    its `lap`, its pack `cells`, their `peak`, the `share` of the clamp's pull its rows have
    or, a rotor past the flight's clamp, the seconds `over` it - its `air`'s wind as the
    view, `yaw` degrees round, has it, the rotors, and `trace` drawn: the height and the
    power, the bus and the hottest board."""
    return compose_craft(console, origin, rotors, frame, flight, trace, now, art, yaw,
                         TITLE, quad, flown)


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


def carried(frame, aim, gate, line_, width, height, field=grounds):
    """The carried camera's picture for `quadcopter.render`'s `inset`: the world on `field` from
    its lens on the `frame`, its sight `aim` (`gimbal.step`), the course's gates where `gate` is
    the one flown to next - its note how far - and its line with `line_`."""
    eye = gimbal.lens(frame['at'], frame['turn'])
    note = '' if gate is None else 'GATE %d  %.0f m' % (
        gate, math.dist(eye, field.GATES[gate % len(field.GATES)][:3]))
    return inset.docked(onboard.picture(eye, aim, gate, line_, field), width, height, 'GIMBAL', note)


def wheeled(zoomed, moved):
    """The wheel's `moved` (WHEEL_STEP a notch) on the view's `zoomed`: a notch a ZOOM_STEP."""
    if moved:
        zoomed(ZOOM_STEP ** (moved / WHEEL_STEP))


def hands(held, zoomed, record=None, send=None):
    """{key: what it does}: the page's keys on its `held` state, `zoomed` the view's zoom times
    its argument, `record` R's, `send` the flight's commands (`flying.Flight.apply`)."""
    sent = send or (lambda **command: None)
    return dict([(k, lambda: zoomed(ZOOM_STEP)) for k in '+=']
                + [(k, lambda: zoomed(1.0 / ZOOM_STEP)) for k in '-_']
                + [(k, lambda: sent(reset=True)) for k in 'aA']
                + [(k, record or (lambda: None)) for k in 'rR']
                + [(k, lambda: sent(wind=True)) for k in 'wW']
                + [(k, lambda: held.update(arrows=not held['arrows'])) for k in 'tT']
                + [(k, lambda: held.update(line=not held['line'])) for k in 'lL']
                + [(k, lambda: held.update(carried=not held['carried'])) for k in 'cC']
                + [(k, lambda: held.update(see=(held['see'] + 1) % len(quadcopter.LAYERS)))
                   for k in 'sS'])


@functools.lru_cache(maxsize=None)
def drawn_radius(build):
    """How far the craft `build('frame')` draws reaches from its middle, m: framed by its view's
    reach, the quad (0.46 m against 0.8) drew 51 braille cells and the others twice its size."""
    from coaxial.model.blocks import numpy as np
    return float(np.linalg.norm(build('frame').corners, axis=1).max())


def watch(camera, frame, eye, build, sight, dt):
    """The spectator at `eye` `dt` s on after the craft at `frame` (`build`'s, `drawn_radius`),
    `(reach, centre)` for a render from SIGHT `sight`: its gaze filtered, its lens
    (`camera['lens']`, the wheel's zoom on it) framing the craft with the horizon below it, the
    eye where it stands."""
    radius = drawn_radius(build)
    x, y, z = (float(c) for c in frame['at'])
    vel = [float(c) for c in frame['vel']]
    aim = (x + LEAD_S * vel[0], y + LEAD_S * vel[1], z + LEAD_S * vel[2])
    k = min(1.0, dt / LEAD_S) if dt > 0.0 and 'gaze' in camera else 1.0
    gaze = camera['gaze'] = tuple(g + (a - g) * k for g, a in zip(camera.get('gaze', aim), aim))
    dx, dy, dz = (g - e for g, e in zip(gaze, eye))
    d = math.hypot(math.hypot(dx, dz), dy)
    up, rho = math.atan2(dy, max(0.5, math.hypot(dx, dz))), math.atan2(radius, d)
    cam = engine.camera(400, 100, 1.0, distance=sight)
    half_tan = cam['height'] / cam['scale']
    framed = [half_tan / math.tan(min(1.5, rho / share)) for share in (FILL, FILL_MIN)]
    horizon = half_tan / math.tan(min(1.5, 0.5 * abs(up) + rho + HORIZON_RAD))
    kept = max(0.0, min(1.0, (UP_FADE[1] - abs(up)) / (UP_FADE[1] - UP_FADE[0])))
    want = framed[0] ** (1.0 - kept) * max(framed[1], min(framed[0], horizon)) ** kept * camera['zoom']
    lens = camera.get('lens', want)
    lens = camera['lens'] = lens * (want / lens) ** (min(1.0, dt / LENS_S) if dt > 0.0 else 1.0)
    half = math.atan(half_tan / lens)
    room = max(0.0, 0.9 * half - rho)
    pitch = max(up - room, min(up + room, 0.5 * up))
    yaw = math.atan2(-dx, -dz)
    camera['yaw'], camera['pitch'] = math.degrees(yaw), -math.degrees(pitch)
    return d / sight, (eye[0] - d * math.cos(pitch) * math.sin(yaw), eye[1] + d * math.sin(pitch),
                       eye[2] - d * math.cos(pitch) * math.cos(yaw))


def look(camera, law, row, frame, chased, dt, near_m=NEAR_M, chase_m=CHASE_M, base_yaw=YAW,
         chase_pitch=PITCH, build=quadcopter.mesh, sight=quadcopter.SIGHT):
    """The camera `dt` s on, `(reach, centre)`: during a craft's figures (`not chased`) its
    spectator (`watch`) at SPOT; on the course (`chased`) the
    frame `chase_m` wide, come round behind the law's heading and framing ahead of it."""
    if not chased:
        return watch(camera, frame, (SPOT[0], SPOT_Y, SPOT[1]), build, sight, dt)
    camera['lens'] = 1.0
    camera.pop('gaze', None)
    x, y, z = (float(c) for c in frame['at'])
    want = chase_m if row['height'] > 0.0 else near_m
    camera['reach'] += (want - camera['reach']) * min(1.0, dt / PULL_S)
    reach = camera['reach'] / camera['zoom']
    trail = max(-TRAIL_SHARE * reach, min(TRAIL_SHARE * reach, TRAIL_S * frame['v']))
    pitch = camera.get('pitch', SIDE_PITCH)
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


def run_craft_view(argv, doc, craft, gfx, flown_mod, compose_fn, thrusts_fn, look_fn,
                   base_yaw=YAW, near_m=NEAR_M, banner=None, close_note=None):
    """Run the interactive terminal view for any coaxial aircraft (`quad`, `helicopter`,
    `ornithopter`, or `aerobat`): its flight apart (`flying`), played back on the page's clock -
    steady (`Playback.steady`): in real time or slower, never run ahead."""
    labels = getattr(flown_mod, 'ROTORS', ROTORS)
    word = 'four' if len(labels) == 4 else 'two'
    parser = argparse.ArgumentParser(description=(doc or '').splitlines()[0])
    parser.add_argument('--port', help='taken, not used: %s stand-in boards fly' % word)
    parser.add_argument('--simulated', action='store_true', help='taken: the rotors are')
    parser.add_argument('--hz', type=float, default=FPS_CAP,
                        help='screen refreshes per second, at most %.0f' % FPS_CAP)
    parser.add_argument('--frames', type=int, default=0,
                        help='stop after this many, instead of running until closed')
    args = parser.parse_args(argv)
    with boot('LINKING %s ROTORS' % word.upper()) as ready:
        apart = flying.Flying(craft, flown_mod, len(labels))
        try:
            said = apart.first()
        except RuntimeError as exc:
            say('fail', 'rotors', str(exc))
            apart.close()
            return 1
        ready()
    origin = said[0]['origin']
    say('warn', 'rotors', banner or ('two stand-in boards on the pilot, %s and %s on the 63100'
                                     % labels))
    drawing = Ahead(gfx.__name__, 'render')
    say('ok', 'drawing', drawing.name or 'a process of its own, dots')
    playback, trace = Playback.steady(flying.blend), []
    held = {'epoch': None, 'shown': None, 'arrows': True, 'line': True, 'carried': True, 'see': 0}
    camera = {'reach': near_m, 'yaw': base_yaw, 'pitch': SIDE_PITCH, 't': None, 'zoom': 1.0}

    def zoomed(k):
        camera['zoom'] = max(ZOOM[0], min(ZOOM[1], camera['zoom'] * k))
    keys = hands(held, zoomed, lambda: recorded(held, craft.__name__.rpartition('.')[2], labels),
                 apart.send)

    def took(states):
        """The states the flight said since the last frame: the trace's rows and a recording's."""
        for state in states:
            if state['epoch'] != held['epoch']:
                held['epoch'] = state['epoch']
                trace.clear()
            frame, cells = state['frame'], state['flight']['cells']
            trace.append((state['t'], frame['h'], cells['volts'], cells['watts'],
                          max((r['board_c'] for r in state['rotors'] if r['board_c'] is not None),
                              default=None)))
            if held.get('recording') is not None:
                recorder.added(held['recording'], [flown_row(state['t'], frame, cells, state['rotors'],
                                                             held.pop('keyed', ''))])
        while trace and trace[-1][0] - trace[0][0] > traces.TRACE_S:
            trace.pop(0)
        playback.push(states)

    took(said)
    board_view = stage()
    terminal = board_view.is_terminal
    leaving = None

    def draw():
        took(apart.said())
        now = playback.at(drawn_next()) or said[-1]
        apart.shown(now['t'])
        if now['epoch'] != held['shown']:
            held['shown'] = now['epoch']
            camera.update(reach=near_m, yaw=base_yaw, pitch=SIDE_PITCH, t=None)
        frame, rotors, flight = now['frame'], now['rotors'], now['flight']
        dt = 0.0 if camera['t'] is None else now['t'] - camera['t']
        camera['t'] = now['t']
        chased = now['lined']
        reach, centre = look_fn(camera, {'heading': now['heading']}, flight['row'], frame, chased, dt)
        width, height = size_of(board_view)
        gate = (flight['lap'] or {}).get('gate', 1) if chased else None
        field = getattr(flown_mod, 'FIELD', grounds if chased else airfield)
        art = '\n'.join(drawing.drawn(
            frame, [(r['angle'], r['w'], (r['budget'] or {}).get('winding_c'), r['board_c'],
                     (r.get('cooling') or {}).get('sink_c'))
                    for r in rotors], width, height, yaw=camera['yaw'], pitch=camera['pitch'],
            reach=reach, centre=centre, colour=terminal, lit=True, gate=gate, lens=camera.get('lens', 1.0),
            field=field, thrusts=thrusts_fn(frame, rotors) if held['arrows'] else None,
            line_=chased and held['line'], see=quadcopter.LAYERS[held['see']],
            inset={**(carried(frame, now['aim'], gate, chased and held['line'], width, height, field)
                      if held['carried'] else {}), **recorder.corner(held.get('recording'), width)}))
        # Drawn a frame ahead: this frame shows the state asked the frame before, its HUD too.
        (shown, yaw), held['drawn'] = held.get('drawn') or (now, camera['yaw']), (now, camera['yaw'])
        return compose_fn(board_view, origin, shown['rotors'], shown['frame'], shown['flight'],
                          [row for row in trace if row[0] <= shown['t']], shown['t'], art, yaw=yaw)

    try:
        leaving = run_view(board_view, terminal, 1.0 / max(args.hz, 0.5), args.frames, draw,
                           on_input=lambda keyed, moved: (typed(held, keys, keyed), wheeled(zoomed, moved)),
                           mouse=True, select=frozenset(), scroll_keys=False)
    except RuntimeError as exc:
        say('fail', 'flight', str(exc))
    finally:
        apart.close(wait=3.0)
        drawing.close()
        sys.stdout.write('\n')
        closing([('rotors', close_note or ('%s drives off, gates off, %s stand-ins closed' % (word, word)))],
                terminal, 0)
    return TO_MENU if leaving == 'menu' else 0


def main(argv=None):
    return run_craft_view(
        argv, __doc__, quad, sys.modules[__name__].quadcopter, flown,
        lambda c, o, r, f, fl, tr, cl, ar, yaw=0.0: sys.modules[__name__].compose(
            c, o, r, f, fl, tr, cl, ar, yaw=yaw),
        thrusts, look, YAW, NEAR_M,
        banner='four stand-in boards on the pilot, sensorless on the 63100 under 16x14s')


if __name__ == '__main__':
    sys.exit(main())

