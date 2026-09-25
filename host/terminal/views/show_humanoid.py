#!/usr/bin/env python3
"""A slender gynoid walking, her twenty joints driven through the machine's actuator API.

    python terminal/views/show_humanoid.py
    python terminal/views/show_humanoid.py --frames 30

Nothing is simulated or emulated: `Machine.discover('humanoid', execution_mode=VIRTUAL)`, each
joint where its command puts it, slewed (`machine.virtual`). Every frame the walk
(`machine.gait`) writes the twenty setpoints through `machine.loop`; what is drawn is what the
joints read back (`<joint>.deg`), lit on the GPU where a card answers (`coaxial.graphics.gynoid`).
"""
import argparse
import math
import sys
import time

from coaxial.comm.session import Origin
from coaxial.graphics import gpu, gynoid
from machine import Machine, gait
from machine.modes import VIRTUAL
from machine.routines import TYPES
from terminal.loader import TO_MENU
from terminal.ui import screen as _screen
from terminal.ui.screen import closing, run_view, say
from terminal.ui.scroll import HUD_WIDTH
from terminal.ui.stage import frame_of, hud, stage

_screen.CHATTER = False     # the boot bar replaced the scroll

TITLE = 'HUMANOID'

#: Where she runs: no port, no board; the band says it and the chip is VIRTUAL.
ORIGIN = Origin(False, 'virtual', 0, 'virtual', 'VIRTUAL ACTUATORS', 'virtual', 0)

#: The cadence's bounds and a key's step, strides a second, and how fast it follows the key: past
#: 1.0 her legs reach full length early in the swing, and the knee snaps (2026-09-25).
CADENCE, CADENCE_STEP, CADENCE_S = (0.5, 1.0), 0.05, 0.4

#: The camera: where it starts, three quarters round so a stride shows; degrees a key turns
#: it, the orbit's degrees a second, the zoom's bounds.
YAW, TURN_DEG, ORBIT_DEG_S, ZOOM = 60.0, 10.0, 12.0, (0.6, 2.5)

#: A wave: the right arm raised, the forearm swinging, for this long, easing in and out over
#: WAVE_EASE_S. Walking and standing ease into each other over BLEND_S: a pose that jumped, the
#: slew drove at its full rate, a robot's move.
WAVE_S, WAVE_EASE_S, BLEND_S = 3.0, 0.6, 0.8


def wave(t):
    """The right arm's wave `t` s in: up, the forearm swinging twice a second."""
    return {'right_shoulder': 150.0, 'right_elbow': 35.0 + 25.0 * math.sin(4.0 * math.pi * t),
            'right_wrist': 10.0, 'right_gripper': 5.0}


def size_of(console, args):
    """Cells for the drawing: what the viewport leaves, or --width/--height."""
    size = console.size if console.is_terminal else None
    width = args.width or max(24, (size.width if size else 110) - HUD_WIDTH - 6)
    height = args.height or max(12, (size.height if size else 44) - 6)
    return width, height


def boxes(state, angles, targets, name):
    """The side column: the walk, then each limb's joints, set and read back."""
    out = [hud('GAIT', [('state', 'waving' if state['wave'] is not None else
                         'walking' if state['walking'] else 'standing'),
                        ('cadence', '%.2f strides/s' % state['cadence']),
                        ('stride', '%.2f m' % (gait.STRIDE_M * gait.pace(state['cadence']))),
                        ('speed', '%.2f m/s' % (state['cadence'] * gait.eased(state['weight'])
                                                * gait.STRIDE_M * gait.pace(state['cadence']))),
                        ('phase', '%.2f of a stride' % state['phase']),
                        ('drawn by', name)])]
    for subsystem in TYPES['humanoid'].body:
        out.append(hud(subsystem.name.replace('_', ' ').upper(), [
            (joint.split('_', 1)[-1], '%7.1f %7.1f deg' % (targets.get(joint, 0.0),
                                                           angles.get(joint, 0.0)))
            for joint in subsystem.actuators]))
    return out


def _turned(step):
    return lambda state: state.update(yaw=state['yaw'] + step)


def _zoomed(k):
    return lambda state: state.update(zoom=max(ZOOM[0], min(ZOOM[1], state['zoom'] * k)))


def _paced(step):
    return lambda state: state.update(goal=max(CADENCE[0], min(CADENCE[1], state['goal'] + step)))


#: What each key does to the view's state.
KEYS = dict(
    [(' ', lambda state: state.update(walking=not state['walking'])),
     ('left', _turned(-TURN_DEG)), ('right', _turned(TURN_DEG)),
     ('[', _paced(-CADENCE_STEP)), (']', _paced(CADENCE_STEP))]
    + [(k, lambda state: state.update(wave=0.0)) for k in 'wW']
    + [(k, lambda state: state.update(orbit=not state['orbit'])) for k in 'oO']
    + [(k, lambda state: state.update(yaw=YAW, zoom=1.0)) for k in 'rR']
    + [(k, _zoomed(1.1)) for k in '+='] + [(k, _zoomed(1.0 / 1.1)) for k in '-_'])


def act_on(typed, state):
    for key in typed:
        if key in KEYS:
            KEYS[key](state)


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--port', default='COM4', help='accepted with every page; no board here')
    parser.add_argument('--hz', type=float, default=30.0, help='frames per second, at most')
    parser.add_argument('--simulated', action='store_true',
                        help='accepted for the view suite; nothing here is simulated either way')
    parser.add_argument('--frames', type=int, default=0,
                        help='stop after this many, instead of running until closed')
    parser.add_argument('--width', type=int, default=0, help='drawing width in cells, 0 fills')
    parser.add_argument('--height', type=int, default=0, help='drawing height in rows, 0 fills')
    parser.add_argument('--cadence', type=float, default=0.85, help='strides a second')
    args = parser.parse_args(argv)

    machine = Machine.discover('humanoid', execution_mode=VIRTUAL)
    machine.arm()
    say('ok', 'actuators', '%d virtual joints, %s' % (len(machine.actuators),
                                                     ', '.join(s.name for s in
                                                               TYPES['humanoid'].body)))
    card = gpu.adapter()
    lit = gpu.LitRaster(found=card) if card is not None else None
    name = lit.name if lit is not None else 'this process, dots'
    say('ok', 'drawing', name)
    gynoid.body()

    board_view = stage()
    terminal = board_view.is_terminal
    state = {'t': 0.0, 'phase': 0.0, 'walking': True, 'weight': 1.0, 'wave': None,
             'orbit': False, 'yaw': YAW, 'zoom': 1.0, 'travel': 0.0, 'cadence': args.cadence,
             'goal': args.cadence, 'last': time.monotonic()}

    def draw():
        now = time.monotonic()
        dt = min(0.1, max(1e-3, now - state['last']))
        state['last'] = now
        state['t'] += dt
        # The cadence follows the key at CADENCE_S a second, and the stride's phase is carried
        # on at it: t times a new cadence would jump her to another point of the stride.
        state['cadence'] += max(-dt * CADENCE_S, min(dt * CADENCE_S,
                                                     state['goal'] - state['cadence']))
        # Walking eases in and out of standing; the stride's own clock slows with it, so she
        # comes to a stand rather than her legs being set down.
        goal = 1.0 if state['walking'] else 0.0
        state['weight'] += max(-dt / BLEND_S, min(dt / BLEND_S, goal - state['weight']))
        eased = gait.eased(state['weight'])
        cadence = state['cadence']
        state['phase'] = (state['phase'] + dt * eased * cadence) % 1.0
        state['travel'] += dt * eased * cadence * gait.STRIDE_M * gait.pace(cadence)
        targets = gait.blend(gait.stand(), gait.walk(state['t'], cadence, phase=state['phase']),
                             state['weight'])
        if state['wave'] is not None:
            age = state['wave']
            ease = (gait.eased(age / WAVE_EASE_S)
                    * gait.eased((WAVE_S - age) / WAVE_EASE_S))
            targets = gait.blend(targets, dict(targets, **wave(age)), ease)
            state['wave'] = age + dt if age + dt < WAVE_S else None
        if state['orbit']:
            state['yaw'] += ORBIT_DEG_S * dt
        machine.loop.write(**targets)
        channels = machine.loop.step(dt)
        angles = {joint: channels[joint + '.deg'] for joint in machine.actuators}
        root = tuple(a + (b - a) * eased for a, b in
                     zip(gait.standing(), gait.sway(state['t'], cadence, phase=state['phase'])))
        tracks = tuple(a + (b - a) * eased for a, b in
                       zip((gait.STAND_M, -gait.STAND_M, 0.0, 0.0),
                           gait.tracks(state['t'], cadence, phase=state['phase'])))
        width, height = size_of(board_view, args)
        art = '\n'.join(gynoid.render(angles, width, height, yaw=state['yaw'],
                                      zoom=state['zoom'], colour=terminal,
                                      travel=state['travel'], lit=lit, root=root,
                                      tracks=tracks))
        return frame_of(board_view, ORIGIN, TITLE, art, boxes(state, angles, targets, name),
                        (('SPACE', 'WALK'), ('[ ]', 'PACE'), ('W', 'WAVE'), ('<- ->', 'TURN'),
                         ('+ -', 'ZOOM'), ('O', 'ORBIT'), ('R', 'RESET'), ('Q', 'EXIT'),
                         ('ESC', 'MENU')))

    leaving = None
    try:
        leaving = run_view(board_view, terminal, 1.0 / max(1.0, min(args.hz, 60.0)),
                           args.frames, draw, on_input=lambda typed, _moved: act_on(typed, state))
    finally:
        machine.disarm()
        sys.stdout.write('\n')
        closing([('joints', 'disarmed, where they stood'),
                 ('boards', 'none - every actuator was virtual')], terminal, 0)
    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
