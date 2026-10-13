#!/usr/bin/env python3
"""A slender gynoid on her latent loop under gravity: a body with mass, each joint a drive.

    python terminal/views/show_humanoid.py
    python terminal/views/show_humanoid.py --frames 30

Her figure in MuJoCo, each joint a drive (`machine.physics`), her latent loop setting their
torques every millisecond in a process paced to the wall clock (`machine.sprung`); drawn from
her joints' read-back, lit on the GPU where a card answers (`coaxial.graphics.gynoid`), each
drive called out at the viewport's edge. She walks from rest; her moves are said by digit
(`terminal.views.gym`), the keys by group on TAB (GROUPS).
"""
import argparse
import math
import os
import sys
import time

from rich.text import Text

from coaxial.comm.session import Origin
from coaxial.graphics import gpu, gynoid
from coaxial.graphics.shapes import view
from machine import figure, sprung
from machine.figure import JOINTS, quat
from machine.routines import TYPES
from terminal.loader import TO_MENU
from terminal.views import gym, viewpoint
from terminal.views.humanoid_keys import CHARGE_MAX_N, act_on, charging, released, row, table
from terminal.views.playback import Playback
from terminal.views.scanner import VisionScanner, docked_vision
from terminal.ui import screen as _screen
from terminal.ui.screen import PORT, FPS_CAP, closing, run_view, say
from terminal.ui.scroll import HUD_WIDTH
from terminal.ui.stage import footer, frame_of, help_rows, helped, hud, stage

_screen.CHATTER = False     # the boot bar replaced the scroll

TITLE = 'HUMANOID'

#: Where she runs: no port, no board; the band says it and the chip is DYNAMIC.
ORIGIN = Origin(False, 'dynamic', 0, 'dynamic', 'GRAVITY 9.81 - MUJOCO', 'dynamic', 0)

#: A callout's inks: its ground, the torque's, the power's driving and braking, the legend's
#: words, a number on a dark patch and on a light; its width inside its frame, cells.
BOX_GROUND, TORQUE_INK, DRIVE_INK, BRAKE_INK, NUMBER_INK, DARK_INK, LIGHT_INK = (
    (38, 46, 58), (255, 184, 80), (96, 214, 255), (255, 96, 128), (214, 220, 228),
    (16, 18, 22), (236, 240, 244))
CALLOUT_W = 4

#: The band's meter, METER cells.
METER = 21


def meter(value, marks):
    """A scale: the first mark's word, a line with the middle's word where it falls, lit under
    the dot at `value`, the last's word."""
    lo, hi = marks[0][0], marks[-1][0]

    def cell(v):
        return max(0, min(METER - 1, round((v - lo) / (hi - lo) * (METER - 1))))
    word = marks[1][1]
    start = max(0, min(METER - len(word), cell(marks[1][0]) - len(word) // 2))
    dot = cell(value)
    out = Text(marks[0][1] + ' ', style='bar.dim')
    lit = start <= dot < start + len(word)
    for i in range(METER):
        if start <= i < start + len(word):
            out.append(word[i - start], style='bar' if lit else 'bar.dim')
        else:
            out.append('●' if i == dot else '─', style='bar' if i == dot else 'bar.dim')
    out.append(' ' + marks[-1][1], style='bar.dim')
    return out


#: The charge meter's marks, N; a landed push's mark shown LANDED_S; the mark's inks, charging
#: (dim to full) and landed.
CHARGES = ((0.0, '0'), (CHARGE_MAX_N / 2.0, 'N'), (CHARGE_MAX_N, '%.0f' % CHARGE_MAX_N))
LANDED_S = 0.5
CHARGE_INK, LANDED_INK = (255, 96, 128), (236, 240, 244)


def along(state):
    """The view's line of sight, a unit vector from the viewer onto her, world."""
    m = view(state['yaw'], state['pitch'])
    return (-m[6], -m[7], -m[8])


def torso(now):
    """Where a push lands: her torso's centre, world."""
    at, turn = figure.frames(now['angles'], now['where'], quat(*now['turn']))['torso']
    return figure.add(at, figure.apply(turn, figure.shares()['torso'][1]))


def marked(state, now, clock):
    """The push's mark on her torso, (world point, ink): charging, in CHARGE_INK as bright as
    the charge; LANDED_S after it lands, in LANDED_INK; else None."""
    charge, landed = charging(state, clock), state.get('landed')
    if charge:
        k = 0.3 + 0.7 * charge / CHARGE_MAX_N
        return torso(now), tuple(int(c * k) for c in CHARGE_INK)
    if landed and clock - landed[0] < LANDED_S:
        return torso(now), LANDED_INK
    return None


def gauges(state):
    """The band's meter: a push charging, its newtons; else none."""
    charge = charging(state, time.monotonic())
    if not charge:
        return None
    out = Text('PUSH ', style='bar.dim')
    out.append_text(meter(charge, CHARGES))
    return out


#: What the callouts show, T stepping through: each drive's torque, its power; the legend's
#: words.
SHOWN = ('torque', 'power')
SAYS = {'torque': 'TORQUE N m, of its peak', 'power': 'POWER W, driving | braking'}

#: The bars' full scale: the drive's peak torque, its power at that torque and RAD_S; through a
#: square root, so a light load shows.
RAD_S = 4.0


def _mixed(a, b, k):
    return tuple(int(x + (y - x) * max(0.0, min(1.0, k))) for x, y in zip(a, b))


def _patch(now, joint, shown):
    """(text, ground) of a joint's callout: the number `shown` and the colour it stands on."""
    torque, power, peak = now['torque'][joint], now['power'][joint], now['peak'][joint]
    if shown == 'power':
        ink = DRIVE_INK if power >= 0.0 else BRAKE_INK
        return '%.0f' % abs(power), _mixed(BOX_GROUND, ink, math.sqrt(abs(power) / (peak * RAD_S)))
    return '%.0f' % abs(torque), _mixed(BOX_GROUND, TORQUE_INK, math.sqrt(abs(torque) / peak))


def _ink_on(ground):
    return DARK_INK if 0.3 * ground[0] + 0.59 * ground[1] + 0.11 * ground[2] > 128 else LIGHT_INK


def legend(shown, width):
    """The viewport's last row: what the callouts show, its key, and its colours."""
    words = ' T: %s ' % SAYS[shown]
    cells = [(c, NUMBER_INK, None) for c in words]
    if shown == 'power':
        cells += [(' ', NUMBER_INK, _mixed(BOX_GROUND, DRIVE_INK, k / 3.0)) for k in range(4)]
        cells += [(' ', None, None)]
        cells += [(' ', NUMBER_INK, _mixed(BOX_GROUND, BRAKE_INK, k / 3.0)) for k in range(4)]
    else:
        cells += [(' ', NUMBER_INK, _mixed(BOX_GROUND, TORQUE_INK, k / 7.0)) for k in range(8)]
    return cells[:width]


#: The drives called out, L stepping through: the legs' and the spine's, all, none.
CALLED = {'strong': tuple(j for j in JOINTS if j == 'spine' or j.endswith(
    ('_hip', '_hip_roll', '_knee', '_ankle'))), 'all': JOINTS, 'none': ()}
CALLING = ('strong', 'all', 'none')


def labels(now, called, shown='torque'):
    """{joint: rows} for `gynoid.render`: each of `called`, the number `shown` (SHOWN) on a
    patch its size colours."""
    out = {}
    for joint in CALLED[called]:
        text, ground = _patch(now, joint, shown)
        out[joint] = [[(c, _ink_on(ground), ground) for c in text.rjust(CALLOUT_W)[-CALLOUT_W:]]]
    return out


#: The bar: the keys typed most; TAB slides the rest up in groups (`stage.helped`): her loop,
#: the view, the page.
BAR = (('TAB', 'KEYS'), ('~ / §', 'WASD'), ('ARROWS', 'WALK'), ('< ENT', 'SCENARIO'),
       ('STUNTS', 's e x m..'), ('SPC', 'PUSH'), ('A', 'RESET'), ('Q', 'EXIT'))


def bar_for(state):
    """The bottom bar: adapts when Quake WASD mode is active."""
    if state.get('wasd_mode'):
        return (('TAB', 'KEYS'), ('~ / §', 'NORMAL'), ('WASD', 'MOVE'), ('WA/SD', 'CURVES'),
                ('MOUSE', 'LOOK'), ('< ENT', 'SCENARIO'), ('SPC', 'PUSH'), ('Q', 'EXIT'))
    return BAR


GROUPS = (
    ('QUAKE (WASD)', (('~ / §', 'toggle QUAKE / NORMAL mode'),
                      ('W', 'walk forward (#204 gentle cruise)'),
                      ('S', 'step backward / brake (#205)'),
                      ('A', 'strafe left'),
                      ('D', 'strafe right'),
                      ('WA', 'vänstersväng (forward-left turn)'),
                      ('WD', 'högersväng (forward-right turn)'),
                      ('SA', 'bakåt-vänstersväng'),
                      ('SD', 'bakåt-högersväng (SD-högersväng)'),
                      ('MOUSE L-DRAG', 'look around in 3D (mouselook)'))),
    ('MOVE (ARROWS)', (('UP', 'walk forward (#204 gentle cruise)'),
                       ('DOWN', 'step backward / brake (#205)'),
                       ('LEFT RIGHT', 'sway / strafe left / right'),
                       ('1-5', '#201 stand, #202 sway, #203 step, #204 walk, #205 back'),
                       ('A', 'again: reset & stand anew (when not in WASD)'))),
    ('SCENARIO', (('< ,', 'cycle scenario + auto-move'),
                  ('ENTER', 'run / replay active scenario'),
                  ('scenarios', 'flat, chair, bed, slope, stairs, ice, snow, obstacle, sill, hole, rug'))),
    ('STUNTS', (('s / u', 'sit chair / rise chair (in normal mode)'),
                ('j / d', 'hop up chair / hop down chair'),
                ('e / g', 'lie bed / rise bed'),
                ('f / k', 'lie floor / rise floor'),
                ('x / n', 'stairs walk up / stairs down'),
                ('m / i', 'slope walk / ice walk (slick)'),
                ('y / z', 'parry shove / controlled breakfall'),
                ('b', 'ballistic walk'))),
    ('PUSH', (('SPACE', 'hold to charge push along view vector'),
              ('P N', 'lateral push / nudge'),
              ('shift', 'longitudinal along walk'))),
    ('VIEW (MOUSE)', (('L-DRAG', 'rotate / orbit camera (mouselook)'),
                      ('R-DRAG', 'pan viewport'),
                      ('WHEEL', 'zoom in / out'),
                      ('V', 'home camera view'),
                      ('O', 'toggle auto-orbit'),
                      ('C', 'cycle skin: dressed, shell, mechanism, actuators'),
                      ('L / T', 'joint callouts / torque & power'))),
    ('PAGE', (('R', 'record / save CSV'), ('TAB ?', 'these keys'), ('Q / ESC', 'exit'))))


def size_of(console, args, up=0.0):
    """Cells for the drawing: the viewport's less the key help `up`, or --width/--height."""
    size = console.size if console.is_terminal else None
    width = args.width or max(24, (size.width if size else 110) - HUD_WIDTH - 6)
    bar = footer(BAR).row_count + help_rows(GROUPS, up)
    height = args.height or max(12, (size.height if size else 44) - 5 - bar)
    return width, height


def boxes(state, now, name):
    """The side column: her modes and her moves, the body, then each limb's joints as they
    read back."""
    angles = now['angles'] if now else {}
    wasd = state.get('wasd_mode', False)
    out = gym.boxes(now, wasd=wasd) + [hud('BODY', [
        ('control', 'QUAKE (WASD)' if wasd else 'NORMAL'),
        ('status', gym.status(now)),
        ('rhythm', '%.2f strides/s' % (now['cadence'] if now else 0.0)),
        ('speed', '%.2f m/s' % (now['speed'] if now else 0.0)),
        ('phase', '%.2f of a stride' % (now['phase'] if now else 0.0)),
        ('soles', '%3.0f %3.0f N' % (now['loads'] if now else (0.0, 0.0))),
        ('physics', 'x%.1f real time' % now['ratio'] if now else '-'),
        ('drawing', '%.0f W' % now['watts'] if now else '-'),
        ('record', 'R starts' if state['recording'] is None and not state['recorded']
         else 'on, %.1f s - R saves' % (len(state['recording']) / 60.0)
         if state['recording'] is not None else os.path.basename(state['recorded'])),
        ('drawn by', name)])]
    for subsystem in TYPES['gynoid'].body:
        out.append(hud(subsystem.name.replace('_', ' ').upper(), [
            (joint.split('_', 1)[-1] if subsystem.name != 'axis' else joint,
             '%7.1f deg' % angles.get(joint, 0.0)) for joint in subsystem.actuators]))
    scanner = state.get('scanner')
    if scanner is not None and now is not None:
        detected = scanner.scan(now, state.get('yaw', 0.0))
        target_str = detected[0]['label'] if detected else 'none'
        action_str = detected[0]['affordances'][0][1] if (detected and detected[0]['affordances']) else '-'
        out.append(hud('VISION (EYES)', [
            ('model', scanner.model_name()),
            ('detected', target_str),
            ('action', action_str)]))
    return out


def _seen(state):
    """`gynoid.render`'s `see` as C has her: None her skin."""
    return state['skin'] if state['skin'] in ('mechanism', 'actuators') else None


#: What each key does to the view's state.
KEYS = gym.table(table(viewpoint.KEYS, CALLING, SHOWN))


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--port', default=PORT, help='accepted with every page; no board here')
    parser.add_argument('--hz', type=float, default=FPS_CAP, help='frames per second, at most')
    parser.add_argument('--simulated', action='store_true',
                        help='accepted for the view suite; the physics runs either way')
    parser.add_argument('--frames', type=int, default=0,
                        help='stop after this many, instead of running until closed')
    parser.add_argument('--width', type=int, default=0, help='drawing width in cells, 0 fills')
    parser.add_argument('--height', type=int, default=0, help='drawing height in rows, 0 fills')
    args = parser.parse_args(argv)

    card = gpu.adapter()
    lit = gpu.LitRaster(found=card) if card is not None else None
    name = lit.name if lit is not None else 'this process, dots'
    say('ok', 'drawing', name)
    gynoid.body(), gynoid.body(dressed=False)
    gynoid.body(see='mechanism'), gynoid.body(see='actuators')
    body = sprung.Running()
    say('ok', 'body', '%.1f kg, %d drives, MuJoCo at 1 kHz in its own process'
        % (figure.mass(), sum(len(s.actuators) for s in TYPES['gynoid'].body)))

    board_view = stage()
    terminal = board_view.is_terminal
    state = {'body': body, 'orbit': False, **viewpoint.HOME, 'side': 1.0, 'last_t': None,
             'called': 'strong', 'follow': gynoid.Follow(), 'recording': None, 'recorded': None,
             'playback': Playback(), 'shown': 'torque', 'skin': 'dressed',
             'scanner': VisionScanner(), 'now': None}

    def draw():
        said = []
        body.latest(into=said)
        if state['recording'] is not None:
            state['recording'] += [row(s, state['yaw']) for s in said]
        state['playback'].push(said)
        now = state['playback'].at(time.perf_counter())
        state['now'] = now
        pushed = released(state, time.monotonic())
        if pushed and now is not None:
            body.send(push=tuple(pushed * a for a in along(state)), seconds=sprung.SHOVE_S)
            state['landed'] = (time.monotonic(), pushed)
        if now is not None:
            viewpoint.orbited(state, now['t'])
        up = helped(state, state.pop('typed', ()), time.monotonic())
        width, height = size_of(board_view, args, up)
        if now is None:
            art = '\n'.join(' ' * width for _ in range(height))
        else:
            x, y, z = now['where']
            camera = state['follow']((x, z), now['velocity'], now['t'])
            inset_cells = docked_vision(state['scanner'], now, width, height,
                                        wasd_mode=state.get('wasd_mode', False))
            art = '\n'.join(gynoid.render(now['angles'], width, height, yaw=state['yaw'],
                                          pitch=state['pitch'], pan=state['pan'],
                                          zoom=state['zoom'], colour=terminal, travel=camera,
                                          lit=lit, root=((x - camera[0], y, z - camera[1]),
                                                         quat(*now['turn'])),
                                          labels=labels(now, state['called'], state['shown']),
                                          props=now.get('props'),
                                          mark=marked(state, now, time.monotonic()),
                                          legend=(legend(state['shown'], width)
                                                  if state['called'] != 'none' else None),
                                          dressed=state['skin'] == 'dressed',
                                          see=_seen(state),
                                          inset=inset_cells))
        return frame_of(board_view, ORIGIN, TITLE, art, boxes(state, now, name), bar_for(state),
                        gauges=gauges(state), help=(GROUPS, up))

    leaving = None
    try:
        # Wait for her first state so the first frame drawn has her standing on the floor and HUD boxes populated.
        for _ in range(100):
            first = body.latest()
            if first is not None:
                state['playback'].push([first])
                break
            time.sleep(0.01)
        leaving = run_view(board_view, terminal, 1.0 / max(1.0, args.hz),
                           args.frames, draw, on_input=lambda typed, wheel: act_on(KEYS, typed, state, wheel),
                           **viewpoint.mouse(state, lambda: size_of(board_view, args)))
    finally:
        body.close()
        sys.stdout.write('\n')
        closing([('body', 'stopped, her process ended'),
                 ('boards', 'none - every drive was simulated')], terminal, 0)
    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
