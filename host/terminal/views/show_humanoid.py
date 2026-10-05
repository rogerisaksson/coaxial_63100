#!/usr/bin/env python3
"""A slender gynoid walking under gravity: a body with mass, each joint a drive, balanced on the fly.

    python terminal/views/show_humanoid.py
    python terminal/views/show_humanoid.py --frames 30

Her figure in MuJoCo, each joint a drive (`machine.physics`), her director setting them every
millisecond (`machine.director`) in a process paced to the wall clock (`machine.running`); drawn
from her joints' read-back, lit on the GPU where a card answers (`coaxial.graphics.gynoid`), each
drive called out at the viewport's edge. The keys by group on TAB (GROUPS): the floor's events
ahead, the gym's rigs she lands on, a board glitched (`terminal.views.humanoid_keys`).
"""
import argparse
import math
import os
import sys
import time

from rich.text import Text

from coaxial.comm.session import Origin
from coaxial.graphics import gpu, gynoid
from coaxial_ollama.client import Chosen
from machine import ansi, figure, gait, gaits, style
from machine.director import moment
from machine.figure import JOINTS, quat
from machine.routines import TYPES
from machine.running import Running
from terminal.loader import TO_MENU
from terminal.views.overlay import (BOX_GROUND, BRAKE_INK, CALLOUT_W, DARK_INK, DRIVE_INK, LIGHT_INK,
                                    NUMBER_INK, SMALL, STAND, STANDING, TORQUE_INK, data_labels,
                                    data_legend, traffic)
from terminal.views import viewpoint
from terminal.views.humanoid_keys import CADENCE, act_on, row, table
from terminal.views.playback import Playback
from terminal.ui import screen as _screen
from terminal.ui.screen import PORT, FPS_CAP, closing, run_view, say
from terminal.ui.scroll import HUD_WIDTH
from terminal.ui.stage import footer, frame_of, help_rows, helped, hud, stage

_screen.CHATTER = False     # the boot bar replaced the scroll

TITLE = 'HUMANOID'

#: Where she runs: no port, no board; the band says it and the chip is DYNAMIC.
ORIGIN = Origin(False, 'dynamic', 0, 'dynamic', 'GRAVITY 9.81 - MUJOCO', 'dynamic', 0)


#: The band's meters, METER cells each: her pace (strides/s at each mark) and her style on
#: `style.SWAY`'s axis.
PACES = ((0.0, 'still'), (gait.CADENCE, 'walk'), (1.6, 'run'))
STYLES = ((-1.0, 'catwalk'), (0.0, 'normal'), (1.0, 'swagger'))
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


def gauges(state):
    """The band's PACE and STYLE meters from what the page asked of her."""
    out = Text('PACE ', style='bar.dim')
    k = state['pace']
    out.append_text(meter(state['cadence'] if not state['law'] else gait.CADENCE * (1.0 + k)
                          if k <= 0.0 else gait.CADENCE + (PACES[-1][0] - gait.CADENCE) * k,
                          PACES))
    out.append('   STYLE ', style='bar.dim')
    out.append_text(meter(state['sway'], STYLES))
    return out


#: What the callouts show, T stepping through: each drive's torque, its heat (its worst node,
#: the thermal observer's scale), its power; the legend's words and the heat's span, C.
SHOWN = ('torque', 'heat', 'power')
SAYS = {'torque': 'TORQUE N m, of its peak', 'heat': 'HEAT C', 'power': 'POWER W, driving | braking'}
HEAT_SPAN = (25.0, 100.0)

#: The bars' full scale: the drive's peak torque, its power at that torque and RAD_S; through a
#: square root, so a light load shows.
RAD_S = 4.0

def _mixed(a, b, k):
    return tuple(int(x + (y - x) * max(0.0, min(1.0, k))) for x, y in zip(a, b))


def _patch(now, joint, shown):
    """(text, ground) of a joint's callout: the number `shown` and the colour it stands on."""
    torque, power, peak = now['torque'][joint], now['power'][joint], now['peak'][joint]
    if shown == 'heat':
        celsius = now['heat'][joint][0] if 'heat' in now else HEAT_SPAN[0]
        return '%.0f' % celsius, ansi.thermal_rgb(celsius)
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
    if shown == 'heat':
        lo, hi = HEAT_SPAN
        steps = [lo + (hi - lo) * k / 7.0 for k in range(8)]
        cells += [(c, NUMBER_INK, None) for c in '%.0f ' % lo]
        cells += [(' ', NUMBER_INK, ansi.thermal_rgb(c)) for c in steps]
        cells += [(c, NUMBER_INK, None) for c in ' %.0f' % hi]
    elif shown == 'power':
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


#: The bar: the keys typed most; TAB slides the rest up in groups (`stage.helped`): her body,
#: the floor ahead, the gym - the rigs she lands on (`events.STANDING`, RIGS) and what befalls
#: her there -, the view, the page.
BAR = (('TAB', 'KEYS'), ('S F', 'PACE'), ('P', 'PUSH'), ('1-6', 'FLOOR'), ('7-0', 'GYM'),
       ('A', 'AGAIN'), ('R', 'RECORD'), ('Q', 'EXIT'), ('ESC', 'MENU'))
GROUPS = (
    ('BODY', (('S F', 'pace'), ('A', 'again: lands anew'),
              ('J', 'one law: S F stand .. run'), ('Z X', 'catwalk .. swagger'),
              ('K , .', 'style knob, trim'))),
    ('FLOOR', (('1', 'hole'), ('2', 'rug'), ('3', 'sill'), ('4', 'slip'), ('5', 'lace'),
               ('6', 'stairs'))),
    ('GYM', (('7', 'two bricks: she stands'), ('8', 'bricks staggered'), ('9', 'board, stiff'),
             ('0', 'rocker, free'), ('P N', 'push, nudge, in turn'),
             ('shift', 'along her way'), ('B', 'a brick gone'), ('G H', 'a knee in its SOA, hot'))),
    ('VIEW', (('<- ->', 'turn, or drag'), ('UP DOWN', 'move, or right drag'),
              ('WHEEL + -', 'zoom'), ('O', 'orbit'), ('V', 'home'),
              ('C', 'clothes .. stick'), ('L', 'labels'),
              ('T', 'torque, heat, power'), ('D', 'data'))),
    ('PAGE', (('R', 'record, then save'), ('TAB ?', 'these keys'), ('Q', 'exit'),
              ('ESC', 'menu'))))


def size_of(console, args, up=0.0):
    """Cells for the drawing: the viewport's less the key help `up`, or --width/--height."""
    size = console.size if console.is_terminal else None
    width = args.width or max(24, (size.width if size else 110) - HUD_WIDTH - 6)
    bar = footer(BAR).row_count + help_rows(GROUPS, up)
    height = args.height or max(12, (size.height if size else 44) - 5 - bar)
    return width, height


def boxes(state, now, name):
    """The side column: the body, then each limb's joints as they read back."""
    angles = now['angles'] if now else {}
    out = [hud('BODY', [
        ('status', _status(now)),
        ('pace', '%.2f m/s asked' % gaits.between(state['pace'])['speed']) if state['law'] else
        ('cadence', '%.2f strides/s' % state['cadence']),
        ('speed', '%.2f m/s' % (now['speed'] if now else 0.0)),
        ('phase', '%.2f of a stride' % (now['phase'] if now else 0.0)),
        ('soles', '%3.0f %3.0f N' % (now['loads'] if now else (0.0, 0.0))),
        ('slips', '%d' % (now['slips'] if now else 0)),
        ('pendulum', '%.2f mm' % now['stir'] if now else '-'),
        ('its parts', 'on %.2f  x %.2f  up %.2f' % now['stirs'] if now else '-'),
        ('physics', 'x%.1f real time' % now['ratio'] if now else '-'),
        ('drawing', '%.0f W' % now['watts'] if now and 'watts' in now else '-'),
        ('hottest', _hottest(now) if now else '-'),
        ('gym', state['rig'] or 'the floor'),
        ('glitched', '%s %s' % state['glitched'] if state['glitched'] else 'G soa, H hot'),
        ('ahead', _ahead(now) if now else '1-6'),
        ('record', 'R starts' if state['recording'] is None and not state['recorded']
         else 'on, %.1f s - R saves' % (len(state['recording']) / 60.0)
         if state['recording'] is not None else os.path.basename(state['recorded'])),
        ('drawn by', name)])]
    knobs = now.get('style', {}) if now else {}
    out.append(hud('STYLE', [
        (('> ' if name == state['knob'] else '  ') + name,
         _knob(knobs[name], style.unit(name)) if name in knobs else '-') for name in style.NAMES]))
    for subsystem in TYPES['gynoid'].body:
        out.append(hud(subsystem.name.replace('_', ' ').upper(), [
            (joint.split('_', 1)[-1] if subsystem.name != 'axis' else joint,
             '%7.1f deg' % angles.get(joint, 0.0)) for joint in subsystem.actuators]))
    return out


#: Her status in a word by the director's stage - for the eye, nothing to the director.
STATUS = {'squat': 'CROUCH', 'look': 'CROUCH', 'push': 'RISE', 'rise': 'RISE', 'stand': 'STAND',
          'shift': 'STAND', 'lean': 'STAND', 'step': 'WALK', 'walk': 'WALK', 'catch': 'CATCH',
          'halt': 'STOP', 'settle': 'STOP', 'lower': 'CROUCH', 'rest': 'REST', 'falling': 'FALL',
          'fallen': 'DOWN', 'unfold': 'GET UP', 'roll': 'GET UP', 'prop': 'GET UP',
          'sit': 'GET UP', 'lift': 'GET UP', 'crouch': 'GET UP'}


def _status(now):
    """Her stage in a word and by its moment; TRIP on a lace; given up, DOWN counting down to
    her landing again."""
    if now is None:
        return 'STARTING'
    word = STATUS.get(now['stage'], now['stage'].upper())
    if any(p[0] == 'lace' for p in now.get('props', ())):
        word = 'TRIP'
    if now['stage'] == 'fallen' and now.get('recover') is not None:
        return '%s in %.0f s' % (word, max(0.0, now['recover']))
    return '%s  %s' % (word, moment(now['stage']))


def _hottest(now):
    """The most spent board as it reports: its joint, worst node, envelope spent, derate."""
    joint = max(now['heat'], key=lambda j: now['heat'][j][1])
    celsius, spent, derate, gates = now['heat'][joint]
    return '%s %.0f C %.2f%s' % (joint, celsius, spent, ' x%.2f' % derate if gates else ' off')


#: Within NEAR_M of her pelvis a prop is under her; past AWAY_M behind, left behind.
NEAR_M, AWAY_M = 0.3, 1.0


def _ahead(now):
    """What she is about to trip on: a lace caught, the nearest prop and how far ahead, or one
    asked and the strides before it."""
    props = now.get('props', ())
    if any(p[0] == 'lace' for p in props):
        return 'lace caught'
    w, x, y, z = now['turn']
    h = math.atan2(2.0 * (x * z + w * y), 1.0 - 2.0 * (x * x + y * y))
    near = [(d, p[0]) for p in props if p[0] != 'lace' for d in [
        (p[1][0] - now['where'][0]) * math.sin(h) + (p[1][2] - now['where'][2]) * math.cos(h)]
        if d > -AWAY_M]
    if near:
        metres, kind = min(near)
        return '%s under her' % kind if abs(metres) <= NEAR_M else (
            '%s %.1f m ahead' % (kind, metres) if metres > 0.0 else '%s behind' % kind)
    if now.get('armed'):
        kind, left = now['armed']
        return '%s in %d strides' % (kind, left + 1)
    return '1-6'


def _knob(value, unit):
    """A style knob's value, metres in mm."""
    return '%.0f mm' % (value * 1e3) if unit == 'm' else ('%.2f %s' % (value, unit)).rstrip()


def _seen(state):
    """`gynoid.render`'s `see` as C has her: None her skin."""
    return state['skin'] if state['skin'] in ('mechanism', 'actuators') else None


#: What each key does to the view's state.
KEYS = table(viewpoint.KEYS, CALLING, SHOWN)


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
    parser.add_argument('--cadence', type=float, default=0.85, help='strides a second')
    args = parser.parse_args(argv)

    cadence = max(CADENCE[0], min(CADENCE[1], args.cadence))
    body = Running(cadence, local=Chosen())
    say('ok', 'body', '%.1f kg, %d drives, MuJoCo at 1 kHz in its own process'
        % (figure.mass(), sum(len(s.actuators) for s in TYPES['gynoid'].body)))
    card = gpu.adapter()
    lit = gpu.LitRaster(found=card) if card is not None else None
    name = lit.name if lit is not None else 'this process, dots'
    say('ok', 'drawing', name)
    gynoid.body(), gynoid.body(dressed=False)
    gynoid.body(see='mechanism'), gynoid.body(see='actuators')

    board_view = stage()
    terminal = board_view.is_terminal
    state = {'body': body, 'cadence': cadence, 'orbit': False, **viewpoint.HOME,
             'side': 1.0, 'last_t': None, 'called': 'strong', 'follow': gynoid.Follow(),
             'recording': None, 'recorded': None, 'glitches': 0, 'glitched': None,
             'tripped': None, 'playback': Playback(), 'shown': 'torque', 'skin': 'dressed',
             'data': False, 'traffic': None, 'knob': style.NAMES[0], 'sway': 0.0, 'rig': None,
             'law': False, 'pace': 0.0}

    def draw():
        said = []
        body.latest(into=said)
        if state['recording'] is not None:
            state['recording'] += [row(s, state['yaw']) for s in said]
        state['playback'].push(said)
        now = state['playback'].at(time.perf_counter())
        if now is not None:
            viewpoint.orbited(state, now['t'])
        up = helped(state, state.pop('typed', ()), time.monotonic())
        width, height = size_of(board_view, args, up)
        if now is None:
            art = '\n'.join(' ' * width for _ in range(height))
        elif state['data']:
            _lateral, _roll, rise, _level = STANDING
            art = '\n'.join(gynoid.render(STAND, width, height, yaw=0.0, zoom=SMALL * state['zoom'],
                                          around=True,
                                          colour=terminal, lit=lit,
                                          root=((0.0, rise, 0.0), quat(1.0, 0.0, 0.0, 0.0)),
                                          labels=data_labels(now),
                                          heat={j: h[0] for j, h in now['heat'].items()},
                                          legend=data_legend(width),
                                          dressed=state['skin'] == 'dressed',
                                          see=_seen(state)))
        else:
            x, y, z = now['where']
            camera = state['follow']((x, z), now['velocity'], now['t'])
            art = '\n'.join(gynoid.render(now['angles'], width, height, yaw=state['yaw'],
                                          pitch=state['pitch'], pan=state['pan'],
                                          zoom=state['zoom'], colour=terminal, travel=camera,
                                          lit=lit, root=((x - camera[0], y, z - camera[1]),
                                                         quat(*now['turn'])),
                                          labels=labels(now, state['called'], state['shown']),
                                          heat={j: h[0] for j, h in now['heat'].items()},
                                          props=now.get('props'),
                                          legend=(legend(state['shown'], width)
                                                  if state['called'] != 'none' else None),
                                          dressed=state['skin'] == 'dressed',
                                          see=_seen(state)))
        side = boxes(state, now, name) + ([traffic(state, now)] if state['data'] and now else [])
        return frame_of(board_view, ORIGIN, TITLE, art, side, BAR, gauges=gauges(state),
                        help=(GROUPS, up))

    leaving = None
    try:
        if args.frames:
            # The view suite asks for a few frames: wait for her first state, a second at most.
            for _ in range(100):
                if body.latest() is not None:
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
