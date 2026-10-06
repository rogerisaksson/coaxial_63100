"""QUAD: four coaxial boards flying a frame in MuJoCo, its routine, then gates among trees.

A 2 kg frame (machine.quad) on four stand-in boards, each armed on the master's pilot, its
drive sensorless on the 63100 outrunner's model under an APC 20x10E propeller, its speed loop
on the drive's own estimate; the four on one pack, 63 V full, its bus drooping under what they
take; the frame in MuJoCo on the rotors' thrust and drag, gravity and the air
(terminal.views.quad.flight). It flies flight after flight: first its routine
(machine.aerobatics on machine.flying's law) - a pirouette, an orbit and a corkscrew up, a roll
and a flip over a toss, full tilt into the sky, the fall burned to a stop 10 cm over the floor -
its boards and their thermal observers warmed on it; then the course (machine.course): two laps
through fourteen gates, between trees, over a house, round a mast and between two cars, as
fast as the boards' envelopes leave it the lean for. The rotors are asked what those envelopes
leave of their pull; a pack or the envelopes spent, it comes down for a charged one, or to
cool. It is flown in the air's weather (machine.quad): a wind, gusts over it and eddies in it,
a kind at a time or one kept (W), none of it told to the law - its way and size a pointer at
the side (terminal.views.quad.wind). The camera sits close on the floor before a flight, pulls
back as it lifts and follows it, looking down on it - on the course from behind it, round as it
turns (coaxial.graphics.quadcopter, scenery); the height and the power, the bus and the hottest
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
from coaxial.errors import RigError
from coaxial.draw.gauges import policy_word
from coaxial.graphics import gpu, quadcopter
from machine import aerobatics, quad
from machine.flying import Flying
from machine.modes import SIMULATED
from terminal.loader import TO_MENU
from terminal.ui.screen import FPS_CAP, Feed, closing, run_view, say
from terminal.ui.scroll import HUD_WIDTH
from terminal.ui.stage import boot, frame_of, hud, stage
from terminal.views.quad import flight as flown
from terminal.views.quad import traces, wind

TITLE = 'QUAD'

#: The rotors, round the frame: front left, front right, rear right, rear left.
ROTORS = ('FL', 'FR', 'RR', 'RL')

#: The feed's period, s - the stand-in's boards answer out of memory -, the longest a pass
#: takes the flight on, in its steps (`flight.steps`), and the thermal reads' period, s. The
#: flight's clock is its passes' sum: a pass in twenty ran 50 ms, the routine's rows asked by
#: the wall's clock moved further than their pass and the rotors' loops went to their clamp on
#: what that looked like (2026-10-05).
PHYSICS_S, PASS_S, THERMAL_S = 0.01, 0.05, 0.1

#: The camera: the reach framed about the quad on the floor before a flight and in the air, m,
#: the pull from one to the other, s, its look down, degrees, its turn about the quad, degrees a
#: second, and how far it trails the climb, s of it, at most a share of the reach.
NEAR_M, FAR_M, PULL_S, PITCH, ORBIT_DEG_S = 0.8, 2.4, 1.5, 18.0, 3.0
YAW, TRAIL_S, TRAIL_SHARE = 30.0, 0.12, 0.4

#: On the course: the reach framed, m; the camera comes round behind the frame's heading at
#: this gain, 1/s, and frames a point this far ahead of it, s of its speed.
CHASE_M, CHASE_K, CHASE_S = 4.0, 2.0, 0.3

#: A press of + or -, the reach framed over or times it, and the zoom's bounds.
ZOOM_STEP, ZOOM = 1.15, (0.25, 4.0)


def compose(console, origin, rotors, frame, flight, trace, now, art, yaw=0.0):
    """One frame: the quad in the viewport; at the side the `flight` - its stage, its apex or
    its `lap`, its pack `cells`, their `peak`, the `share` of their pull its rotors are asked
    and how long it has been `gone` - its `air`'s wind as the view, `yaw` degrees round, has
    it, the rotors, and `trace` drawn: the height and the power, the bus and the hottest
    board."""
    name, cells, peak, share = flight['stage'], flight['cells'], flight['peak'], flight['share']
    weight = quad.MASS_KG * quad.GRAVITY
    lift = sum(quad.K_THRUST * r['w'] * r['w'] for r in rotors)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(frame['turn'][1][1])))))
    worst = max(((r['budget'] or {}).get('worst') or 0.0) for r in rotors)
    lap = flight.get('lap')
    flying = hud('FLIGHT', [
        ('stage', Text(name.upper(), style='alarm' if name in ('full tilt', 'burn', 'swap', 'cool')
                       else 'value')),
        ('height', '%8.2f m' % frame['h']),
        ('climb', '%+8.1f m/s, %+.1f g' % (frame['v'], frame['a'] / quad.GRAVITY)),
        ('thrust', '%8.1f of %.1f N' % (lift, weight)),
        ('tilt', '%8.1f deg, %.2f m off' % (tilt, math.hypot(frame['at'][0], frame['at'][2]))),
        ('lap', '%8s gate %d, %.1f m/s' % (
            '%d/%d' % (lap['laps'] + 1, lap['of']), lap['gate'],
            math.hypot(float(frame['vel'][0]), float(frame['vel'][2]))))
        if lap else ('apex', '%8.1f m' % flight['apex']),
        ('TH OBS', flown.observers(rotors)),
        ('SOA', Text('%8.0f %% worst, pull %.0f %%' % (100.0 * worst, 100.0 * share),
                     style='alarm' if share < 1.0 else 'value')),
        ('bus', '%8.1f V, low %.1f' % (cells['volts'], peak['low'])),
        ('pack', Text('%8.0f %% left, %.1f A' % (100.0 * cells['left'], cells['amps']),
                      style='alarm' if cells['left'] <= quad.RESERVE else 'value')),
        ('power', '%8.0f W, peak %.0f' % (cells['watts'], peak['watts']))])
    lines = []
    for label, rotor in zip(ROTORS, rotors):
        budget = rotor['budget'] or {}
        hot = (rotor['amps'] >= 0.9 * flown.I_MAX or budget.get('throttling')
               or budget.get('tripped'))
        lines.append((label, Text('%4.0f rpm %4.1f A SOA %3.0f%% %s' % (
            rotor['w'] * 60.0 / math.tau, rotor['amps'], 100.0 * (budget.get('worst') or 0.0),
            'THR' if budget.get('throttling') else policy_word(rotor['ident'])[0]),
            style='alarm' if hot else 'value')))
    return frame_of(console, origin, TITLE, art,
                    [flying, hud('WIND  the way it blows, in the view',
                                 wind.pointer(flight['air'], frame['h'], yaw)),
                     hud('ROTORS  rpm, current, SOA, TH OBS', lines),
                     hud('HEIGHT m, POWER  last %.0f s' % traces.TRACE_S,
                         traces.heights(trace, now, HUD_WIDTH - 4)),
                     hud('BUS V, PEAK TEMP C', traces.buses(trace, now, HUD_WIDTH - 4))],
                    (('+ -', 'ZOOM'), ('W', 'WIND'), ('R', 'RESET'), ('Q', 'EXIT'),
                     ('ESC', 'MENU')))


def look(camera, flying, frame, chased, dt):
    """The camera `dt` s on, (reach, centre): close on the floor, pulled back in the air,
    turning slowly about the quad and trailing its climb; `chased` - on the course - wider,
    come round behind its heading and framing ahead of it."""
    aloft = flying.ask['height'] > 0.0
    want = NEAR_M if not aloft else CHASE_M if chased else FAR_M
    camera['reach'] += (want - camera['reach']) * min(1.0, dt / PULL_S)
    reach, (x, y, z) = camera['reach'] / camera['zoom'], (float(c) for c in frame['at'])
    trail = max(-TRAIL_SHARE * reach, min(TRAIL_SHARE * reach, TRAIL_S * frame['v']))
    if not chased:
        camera['yaw'] += ORBIT_DEG_S * dt
        return reach, (x, y - trail, z)
    behind = (math.degrees(flying.heading) + 180.0 - camera['yaw'] + 180.0) % 360.0 - 180.0
    camera['yaw'] += behind * min(1.0, CHASE_K * dt)
    ahead = [CHASE_S * float(frame['vel'][axis]) for axis in (0, 2)]
    far = max(1.0, math.hypot(ahead[0], ahead[1]) / (TRAIL_SHARE * reach))
    return reach, (x + ahead[0] / far, y - trail, z + ahead[1] / far)


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
        'APC 20x10E')
    card = gpu.adapter()
    lit = gpu.LitRaster(found=card) if card is not None else None
    say('ok', 'drawing', lit.name if lit is not None else 'this process, dots')
    sky, route, trace = quad.Sky(), aerobatics.routine(flown.CARD), []
    began = time.monotonic()
    held = {'at': 0.0, 'clock': 0.0, 'thermal_at': 0.0, 'frame': sky.state(), 'reset': False,
            'wind': False, 'flying': Flying(flown.TOP_N, aerobatics.DOWN),
            'flight': flown.fresh()}
    camera = {'reach': NEAR_M, 'yaw': YAW, 't': None, 'zoom': 1.0}

    def zoomed(k):
        camera['zoom'] = max(ZOOM[0], min(ZOOM[1], camera['zoom'] * k))
    keys = dict([(k, lambda: zoomed(ZOOM_STEP)) for k in '+=']
                + [(k, lambda: zoomed(1.0 / ZOOM_STEP)) for k in '-_']
                + [(k, lambda: held.update(reset=True)) for k in 'rR']
                + [(k, lambda: held.update(wind=True)) for k in 'wW'])

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
                # R: the frame back on its skids at its spot, its flights from the routine's
                # spool on a fresh pack, the rotors' loops from rest - on this thread, the one
                # that steps the world.
                held['reset'] = False
                sky.reset()
                route.pop('lap', None)
                route.update(aerobatics.routine(flown.CARD), at=held['clock'])
                held['flying'] = Flying(flown.TOP_N, aerobatics.DOWN)
                flown.reset(rotors)
                trace.clear()
                cells.update(quad.pack())
                flight.update(apex=0.0, share=1.0, gone=0.0, stood=0.0)
                camera['reach'] = NEAR_M
            for clock, frame in flown.passed(rotors, sky, route, held['flying'], flight,
                                             held['clock'], dt):
                held['clock'], held['frame'] = clock, frame
                trace.append((clock, frame['h'], cells['volts'], cells['watts'],
                              max((r['board_c'] for r in rotors if r['board_c'] is not None),
                                  default=None)))
            while trace and held['clock'] - trace[0][0] > traces.TRACE_S:
                trace.pop(0)
            if now - held['thermal_at'] >= THERMAL_S:
                held['thermal_at'] = now
                for rotor in rotors:
                    rotor['budget'], rotor['ident'], rotor['board_c'] = flown.warmth(rotor['rig'])

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
        reach, centre = look(camera, held['flying'], frame, chased, dt)
        width, height = size_of(board_view)
        art = '\n'.join(quadcopter.render(
            frame, [(r['angle'], r['w'], (r['budget'] or {}).get('winding_c'), r['board_c'])
                    for r in rotors], width, height, yaw=camera['yaw'], pitch=PITCH,
            reach=reach, centre=centre, colour=terminal, lit=lit,
            gate=(held['flight']['lap'] or {}).get('gate', 1) if chased else None))
        return compose(board_view, origin, rotors, frame, held['flight'], list(trace),
                       held['clock'], art, yaw=camera['yaw'])

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
