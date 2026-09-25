#!/usr/bin/env python3
"""A slender gynoid walking under gravity: a body with mass, each joint a drive, balanced on the fly.

    python terminal/views/show_humanoid.py
    python terminal/views/show_humanoid.py --frames 30

`Machine.discover('gynoid', execution_mode=DYNAMIC)`: her figure in MuJoCo, 55 kg, each of her 27
joints a drive holding its setpoint (`machine.physics`); every millisecond her director reads where
she is and sets them all (`machine.director`): landed in a squat, she rises, steps off and walks,
catching herself when shoved. She runs in a process of her own paced to the wall clock
(`machine.running`); what is drawn is her joints' read-back and her pelvis as it stands, lit on the
GPU where a card answers (`coaxial.graphics.gynoid`), each drive called out beside her with its
torque, a bar and a number, and its power, a bar.
"""
import argparse
import math
import sys
import time

from coaxial.comm.session import Origin
from coaxial.graphics import gpu, gynoid
from coaxial.graphics.raster import BRAILLE, BRAILLE_BITS
from machine.figure import JOINTS, quat
from machine.routines import TYPES
from machine.running import Running
from terminal.loader import TO_MENU
from terminal.ui import screen as _screen
from terminal.ui.screen import closing, run_view, say
from terminal.ui.scroll import HUD_WIDTH
from terminal.ui.stage import frame_of, hud, stage

_screen.CHATTER = False     # the boot bar replaced the scroll

TITLE = 'HUMANOID'

#: Where she runs: no port, no board; the band says it and the chip is DYNAMIC.
ORIGIN = Origin(False, 'dynamic', 0, 'dynamic', 'GRAVITY 9.81 - MUJOCO', 'dynamic', 0)

#: The cadence's bounds and a key's step, strides a second: under 0.6 she fell from her first
#: stride, at 0.95 within two seconds (2026-09-25).
CADENCE, CADENCE_STEP = (0.6, 0.9), 0.05

#: The camera: where it starts, three quarters round so a stride shows; degrees a key turns
#: it, the orbit's degrees a second, the zoom's bounds.
YAW, TURN_DEG, ORBIT_DEG_S, ZOOM = 60.0, 10.0, 12.0, (0.6, 2.5)

#: A shove from her side, newtons for seconds.
PUSH_N, PUSH_S = 120.0, 0.12

#: A callout's inks: its boxes' ground, the torque's bar, the power's driving and braking, the
#: number.
BOX_GROUND, TORQUE_INK, DRIVE_INK, BRAKE_INK, NUMBER_INK = (
    (38, 46, 58), (255, 184, 80), (96, 214, 255), (255, 96, 128), (214, 220, 228))

#: The bars' full scale: the drive's peak torque, and its power at that torque and RAD_S; both
#: drawn through a square root, so a light load shows.
RAD_S = 4.0


def bar(fraction):
    """A braille cell filled from its foot in 8 steps, `fraction` 0 to 1 through a square root."""
    level = int(round(8.0 * math.sqrt(max(0.0, min(1.0, fraction)))))
    bits = 0
    for k in range(level):
        bits |= BRAILLE_BITS[k % 2][3 - k // 2]
    return chr(BRAILLE + bits)


#: The drives called out: the strong ones - the legs' and the spine's - or all, or none; L
#: steps through them.
CALLED = {'strong': tuple(j for j in JOINTS if j == 'spine' or j.endswith(
    ('_hip', '_hip_roll', '_knee', '_ankle'))), 'all': JOINTS, 'none': ()}
CALLING = ('strong', 'all', 'none')


def labels(now, called):
    """{joint: (inner, outer)} for `gynoid.render`, each of `called`: the torque's bar and the
    power's, each in a box of its own, and the torque, N m."""
    out = {}
    for joint in CALLED[called]:
        torque, power, peak = now['torque'][joint], now['power'][joint], now['peak'][joint]
        inner = [(bar(abs(torque) / peak), TORQUE_INK, BOX_GROUND), (' ', None, None),
                 (bar(abs(power) / (peak * RAD_S)), DRIVE_INK if power >= 0.0 else BRAKE_INK,
                  BOX_GROUND)]
        out[joint] = (inner, [(c, NUMBER_INK, None) for c in '%d' % round(abs(torque))])
    return out


def size_of(console, args):
    """Cells for the drawing: what the viewport leaves, or --width/--height."""
    size = console.size if console.is_terminal else None
    width = args.width or max(24, (size.width if size else 110) - HUD_WIDTH - 6)
    height = args.height or max(12, (size.height if size else 44) - 6)
    return width, height


def boxes(state, now, name):
    """The side column: the body, then each limb's joints as they read back."""
    angles = now['angles'] if now else {}
    out = [hud('BODY', [
        ('state', 'starting' if now is None else 'fallen - up again in 2 s' if now['fallen']
         else now['stage']),
        ('cadence', '%.2f strides/s' % state['cadence']),
        ('speed', '%.2f m/s' % (now['speed'] if now else 0.0)),
        ('phase', '%.2f of a stride' % (now['phase'] if now else 0.0)),
        ('soles', '%3.0f %3.0f N' % (now['loads'] if now else (0.0, 0.0))),
        ('slips', '%d' % (now['slips'] if now else 0)),
        ('physics', 'x%.1f real time' % now['ratio'] if now else '-'),
        ('drawn by', name)])]
    for subsystem in TYPES['gynoid'].body:
        out.append(hud(subsystem.name.replace('_', ' ').upper(), [
            (joint.split('_', 1)[-1] if subsystem.name != 'axis' else joint,
             '%7.1f deg' % angles.get(joint, 0.0)) for joint in subsystem.actuators]))
    return out


def _turned(step):
    return lambda state: state.update(yaw=state['yaw'] + step)


def _zoomed(k):
    return lambda state: state.update(zoom=max(ZOOM[0], min(ZOOM[1], state['zoom'] * k)))


def _paced(step):
    def pace(state):
        state['cadence'] = max(CADENCE[0], min(CADENCE[1], state['cadence'] + step))
        state['body'].send(cadence=state['cadence'])
    return pace


def _pushed(state):
    state['side'] = -state['side']
    state['body'].send(push=(state['side'] * PUSH_N, 0.0, 0.0), seconds=PUSH_S)


#: What each key does to the view's state.
KEYS = dict(
    [('left', _turned(-TURN_DEG)), ('right', _turned(TURN_DEG)),
     ('[', _paced(-CADENCE_STEP)), (']', _paced(CADENCE_STEP))]
    + [(k, _pushed) for k in 'pP']
    + [(k, lambda state: state['body'].send(restart=True)) for k in 'aA']
    + [(k, lambda state: state.update(orbit=not state['orbit'])) for k in 'oO']
    + [(k, lambda state: state.update(called=CALLING[(CALLING.index(state['called']) + 1)
                                                     % len(CALLING)])) for k in 'lL']
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
                        help='accepted for the view suite; the physics runs either way')
    parser.add_argument('--frames', type=int, default=0,
                        help='stop after this many, instead of running until closed')
    parser.add_argument('--width', type=int, default=0, help='drawing width in cells, 0 fills')
    parser.add_argument('--height', type=int, default=0, help='drawing height in rows, 0 fills')
    parser.add_argument('--cadence', type=float, default=0.85, help='strides a second')
    args = parser.parse_args(argv)

    cadence = max(CADENCE[0], min(CADENCE[1], args.cadence))
    body = Running(cadence)
    say('ok', 'body', '55 kg, %d drives, MuJoCo at 1 kHz in its own process'
        % sum(len(s.actuators) for s in TYPES['gynoid'].body))
    card = gpu.adapter()
    lit = gpu.LitRaster(found=card) if card is not None else None
    name = lit.name if lit is not None else 'this process, dots'
    say('ok', 'drawing', name)
    gynoid.body()

    board_view = stage()
    terminal = board_view.is_terminal
    state = {'body': body, 'cadence': cadence, 'orbit': False, 'yaw': YAW, 'zoom': 1.0,
             'side': 1.0, 'last_t': None, 'called': 'strong'}

    def draw():
        now = body.latest()
        if state['orbit'] and now is not None:
            if state['last_t'] is not None:
                state['yaw'] += ORBIT_DEG_S * max(0.0, now['t'] - state['last_t'])
            state['last_t'] = now['t']
        width, height = size_of(board_view, args)
        if now is None:
            art = '\n'.join(' ' * width for _ in range(height))
        else:
            x, y, z = now['where']
            art = '\n'.join(gynoid.render(now['angles'], width, height, yaw=state['yaw'],
                                          zoom=state['zoom'], colour=terminal, travel=z,
                                          lit=lit, root=((x, y, 0.0), quat(*now['turn'])),
                                          labels=labels(now, state['called'])))
        return frame_of(board_view, ORIGIN, TITLE, art, boxes(state, now, name),
                        (('[ ]', 'PACE'), ('P', 'PUSH'), ('A', 'AGAIN'), ('L', 'LABELS'),
                         ('<- ->', 'TURN'), ('+ -', 'ZOOM'), ('O', 'ORBIT'), ('R', 'RESET'),
                         ('Q', 'EXIT'), ('ESC', 'MENU')))

    leaving = None
    try:
        if args.frames:
            # The view suite asks for a few frames: wait for her first state, a second at most.
            for _ in range(100):
                if body.latest() is not None:
                    break
                time.sleep(0.01)
        leaving = run_view(board_view, terminal, 1.0 / max(1.0, min(args.hz, 60.0)),
                           args.frames, draw, on_input=lambda typed, _moved: act_on(typed, state))
    finally:
        body.close()
        sys.stdout.write('\n')
        closing([('body', 'stopped, her process ended'),
                 ('boards', 'none - every drive was simulated')], terminal, 0)
    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
