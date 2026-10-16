"""HELICOPTER: two coaxial boards flying a 3D helicopter in MuJoCo, its stunts and gates.

A 2 kg single-main-rotor 3D helicopter (`machine.helicopter`) on two stand-in 63100 boards (`MAIN`
on the mast and `TAIL` on the boom), each armed on the master's pilot and running sensorless on the
63100 outrunner's model; the two on one 63 V pack; the airframe in MuJoCo under signed collective
pitch, cyclic swashplate hub moments, main-rotor reaction torque and tail-rotor anti-torque
(`terminal.views.helicopter.flight`). It flies flight after flight: first its 3D aerobatics routine
- hover, fast pirouette, banked funnel, climbing spiral, half-flip to sustained negative-collective
inverted hover and inverted pirouette, elevator tic-tocs, 3D piro-flip, axial roll, full-tilt
skyrocket punch-out, and collective burn stop 10 cm over the floor -, viewed from a side RC-pilot
perspective that zooms slightly to follow while keeping contact with the horizon, and then two laps
of the shared 16-gate course (`machine.course`) chased from behind.

    python terminal/views/show_helicopter.py
    python terminal/views/show_helicopter.py --frames 60
"""
import argparse
import math
import sys
import time
from contextlib import suppress

from rich.text import Text

from coaxial import Coaxial63100
from coaxial.draw.gauges import policy_word
from coaxial.errors import RigError
from coaxial.graphics import gpu
from coaxial.graphics import helicopter as heli_gfx
from machine import aerobatics, aerokin, gimbal, grounds, helicopter, pull, quad
from machine.modes import SIMULATED
from terminal.loader import TO_MENU
from terminal.ui import recorder
from terminal.ui.screen import FPS_CAP, Feed, closing, run_view, say
from terminal.ui.scroll import HUD_WIDTH
from terminal.ui.stage import boot, frame_of, hud, stage
from terminal.views.helicopter import flight as flown
from terminal.views.quad import traces, wind
from terminal.views.show_quad import (PASS_S, PHYSICS_S, SIDE_PITCH, SWITCH_S, ZOOM, carried,
                                       flown_row, hands, look as quad_look, recorded, size_of, typed)

TITLE = 'HELICOPTER'
ROTORS = flown.ROTORS
NEAR_M, FAR_M, ZOOM_M, ZOOM_K, CHASE_M = 0.76, 1.00, 0.48, 0.022, 2.2
YAW, PITCH = 52.0, 18.0


def look(camera, law, row, frame, chased, dt):
    """The helicopter camera `dt` s on, `(reach, centre)`: RC side perspective with slight
    zoom-follow and horizon contact during the 3D aerobatics warm-up, and behind the heading on
    the gate course."""
    return quad_look(camera, law, row, frame, chased, dt, near_m=NEAR_M, far_m=FAR_M,
                     zoom_m=ZOOM_M, zoom_k=ZOOM_K, chase_m=CHASE_M, base_yaw=YAW,
                     chase_pitch=PITCH)


def thrusts(frame, rotors):
    """`(main_thrust_n, tail_thrust_n)` in N as the helicopter flies, signed with collective and
    tail pitch."""
    turn = frame['turn']
    along = sum(float(v) * float(turn[k][1]) for k, v in enumerate(frame['vel']))
    coll, _cyc_p, _cyc_r, tail_p = frame.get('controls', (0.0, 0.0, 0.0, 0.0))
    w_main, w_tail = rotors[0]['w'], rotors[1]['w']
    signed_along = along if coll >= 0.0 else -along
    main_n = helicopter.K_THRUST * w_main * w_main * coll * helicopter.inflow(signed_along, w_main)
    tail_n = helicopter.K_TAIL * w_tail * w_tail * tail_p
    return main_n, tail_n


def compose_craft(console, origin, rotors, frame, flight, trace, now, art, yaw, title, craft,
                  flown_mod, extra_row):
    """One frame for a two-drive craft: the 3D viewport and the 5-box side HUD (`FLIGHT`, `WIND`,
    `ROTORS`, `HEIGHT/POWER`, `BUS/PEAK TEMP`)."""
    name, cells, peak, share = flight['stage'], flight['cells'], flight['peak'], flight['share']
    weight = craft.MASS_KG * craft.GRAVITY
    ctrl0 = frame.get('controls', flight.get('controls', (0.0, 0.0, 0.0, 0.0)))[0]
    lift = float(frame.get('lift', craft.K_THRUST * rotors[0]['w'] ** 2 * ctrl0))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(frame['turn'][1][1])))))
    worst = max(((r['budget'] or {}).get('worst') or 0.0) for r in rotors)
    lap, over = flight.get('lap'), max(r['amps'] for r in rotors) > pull.I_MAX
    tight = min((r['budget'] or {}).get('hold_amps', pull.I_MAX) for r in rotors) < pull.I_MAX
    flying = hud('FLIGHT', [
        ('stage', Text(('%s %s' % (name, flown_mod.struck(flight))).upper().strip(), style='alarm'
                       if name in ('full tilt', 'burn', 'swap', 'cool', flown.CRASHED)
                       else 'value')),
        ('height', '%8.2f m' % frame['h']),
        ('climb', '%+8.1f m/s, %+.1f g' % (frame['v'], frame['a'] / craft.GRAVITY)),
        ('thrust', '%+8.1f of %.1f N' % (lift, weight)),
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
                      style='alarm' if cells['left'] <= quad.RESERVE else 'value')),
        ('power x2', '%8.0f W, peak %.0f' % (cells['watts'], peak['watts']))])
    lines = []
    for label, rotor in zip(flown_mod.ROTORS, rotors):
        budget = rotor['budget'] or {}
        held = budget.get('hold_amps', pull.I_MAX) < pull.I_MAX
        hot = rotor['amps'] >= 0.9 * pull.I_MAX or held or budget.get('tripped')
        lines.append((label, Text('%4.0f rpm %4.1f A SOA %3.0f%% %s' % (
            rotor['w'] * 60.0 / math.tau, rotor['amps'], 100.0 * (budget.get('worst') or 0.0),
            'THR' if held else policy_word(rotor['ident'])[0]),
            style='alarm' if hot else 'value')))
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
                     hud('HEIGHT m, POWER OF BOTH  last %.0f s' % traces.TRACE_S,
                         traces.heights(trace, now, HUD_WIDTH - 4)),
                     hud('BUS V, PEAK TEMP C', traces.buses(trace, now, HUD_WIDTH - 4))],
                    (('+ -', 'ZOOM'), ('W', 'WIND'), ('T', 'THRUST'), ('L', 'LINE'),
                     ('C', 'CAMERA'), ('R', 'RECORD'), ('A', 'RESET'), ('Q', 'EXIT'), ('ESC', 'MENU')))


def compose(console, origin, rotors, frame, flight, trace, now, art, yaw=0.0):
    """One frame: the 3D helicopter in the viewport and the side HUD."""
    coll, cyc_p, cyc_r, tail_p = frame.get('controls', flight.get('controls', (0.0, 0.0, 0.0, 0.0)))
    cyc_deg = math.hypot(cyc_p, cyc_r) * helicopter.CYC_DEG
    swash = ('SWASH', Text('%+4.1f coll %4.1f cyc %+4.1f tail' % (
        coll * helicopter.COLL_DEG, cyc_deg, tail_p * helicopter.TAIL_DEG),
        style='alarm' if coll < -0.15 else 'value'))
    return compose_craft(console, origin, rotors, frame, flight, trace, now, art, yaw,
                         TITLE, helicopter, flown, swash)


def run_craft_view(argv, doc, craft, gfx, flown_mod, compose_fn, thrusts_fn, look_fn, base_yaw=YAW):
    """Run the interactive terminal view for a two-drive craft (`helicopter`, `ornithopter`, or
    `aerobat`)."""
    parser = argparse.ArgumentParser(description=(doc or '').splitlines()[0])
    parser.add_argument('--port', help='taken, not used: two stand-in boards fly')
    parser.add_argument('--simulated', action='store_true', help='taken: the rotors are')
    parser.add_argument('--hz', type=float, default=FPS_CAP,
                        help='screen refreshes per second, at most %.0f' % FPS_CAP)
    parser.add_argument('--frames', type=int, default=0,
                        help='stop after this many, instead of running until closed')
    args = parser.parse_args(argv)

    rotors = []
    with boot('LINKING TWO ROTORS') as ready:
        try:
            for _ in flown_mod.ROTORS:
                rotors.append(flown_mod.arm(Coaxial63100(execution_mode=SIMULATED).open()))
        except RigError as exc:
            say('fail', 'rotors', str(exc))
            for rotor in rotors:
                rotor['rig'].close()
            return 1
        ready()
    origin = rotors[0]['rig'].origin
    say('warn', 'rotors', 'two stand-in boards on the pilot, %s and %s on the 63100'
        % flown_mod.ROTORS)
    card = gpu.adapter()
    lit = gpu.LitRaster(found=card) if card is not None else None
    say('ok', 'drawing', lit.name if lit is not None else 'this process, dots')
    sky, route, trace = craft.Sky(grounds.solids()), aerobatics.routine(flown_mod.CARD), []
    began = time.monotonic()
    held = {'at': 0.0, 'clock': 0.0, 'frame': sky.state(), 'reset': False,
            'wind': False, 'law': aerokin.fresh(), 'flight': flown_mod.fresh(),
            'arrows': True, 'line': True, 'carried': True, 'aim': gimbal.REST}
    camera = {'reach': NEAR_M, 'yaw': base_yaw, 'pitch': SIDE_PITCH, 't': None, 'zoom': 1.0}

    def zoomed(k):
        camera['zoom'] = max(ZOOM[0], min(ZOOM[1], camera['zoom'] * k))
    keys = hands(held, zoomed, lambda: recorded(held, craft.__name__.rpartition('.')[2], rotors))

    def sample():
        """Advance the flight on the background feed thread."""
        with suppress(RigError):
            now = time.monotonic() - began
            dt = min(PASS_S, max(0.0, now - held['at']))
            held['at'] = now
            if dt <= 0.0:
                return
            flight = held['flight']
            cells = flight['cells']
            if held['wind']:
                held['wind'] = False
                quad.kept(flight['air'])
            if held['reset']:
                held['reset'] = False
                sky.reset()
                route.pop('lap', None)
                route.update(aerobatics.routine(flown_mod.CARD), at=held['clock'])
                flown_mod.begun(held['law'], flight)
                flown_mod.reset(rotors)
                trace.clear()
                cells.update(quad.pack())
                flight.update(apex=0.0, share=1.0, over=0.0, stood=0.0, wreck=None)
                camera.update(reach=NEAR_M, yaw=base_yaw, pitch=SIDE_PITCH)
            for clock, frame in flown_mod.passed(rotors, sky, route, held['law'], flight,
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
        reach, centre = look_fn(camera, held['law'], held['flight']['row'], frame, chased, dt)
        width, height = size_of(board_view)
        gate = (held['flight']['lap'] or {}).get('gate', 1) if chased else None
        art = '\n'.join(gfx.render(
            frame, [(r['angle'], r['w'], (r['budget'] or {}).get('winding_c'), r['board_c'],
                     (r.get('cooling') or {}).get('sink_c'))
                    for r in rotors], width, height, yaw=camera['yaw'], pitch=camera['pitch'],
            reach=reach, centre=centre, colour=terminal, lit=lit, gate=gate,
            thrusts=thrusts_fn(frame, rotors) if held['arrows'] else None,
            line_=chased and held['line'],
            inset={**(carried(frame, held['aim'], gate, chased and held['line'], width, height)
                      if held['carried'] else {}), **recorder.corner(held.get('recording'), width)}))
        return compose_fn(board_view, origin, rotors, frame, held['flight'], list(trace),
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
        done.append(('rotors', 'two drives off, gates off, two stand-ins closed'))
        sys.stdout.write('\n')
        closing(done, terminal, 0)
    return TO_MENU if leaving == 'menu' else 0


def main(argv=None):
    return run_craft_view(argv, __doc__, helicopter, heli_gfx, flown, compose, thrusts, look)


if __name__ == '__main__':
    sys.exit(main())

