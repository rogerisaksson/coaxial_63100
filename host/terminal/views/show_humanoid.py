#!/usr/bin/env python3
"""A slender gynoid walking under gravity: a body with mass, each joint a drive, balanced on the fly.

    python terminal/views/show_humanoid.py
    python terminal/views/show_humanoid.py --frames 30

`Machine.discover('gynoid', execution_mode=DYNAMIC)`: her figure in MuJoCo, 55 kg, each of her 27
joints a drive holding its setpoint (`machine.physics`); every millisecond her director reads where
she is and sets them all (`machine.director`): landed in a squat, she rises, steps off and walks,
catching herself when shoved. She runs in a process of her own paced to the wall clock
(`machine.running`); what is drawn is her joints' read-back and her pelvis as it stands, lit on the
GPU where a card answers (`coaxial.graphics.gynoid`), each drive called out at the viewport's
edge with a leader to its joint: its angle, its torque as a bar and a number, its power as a bar.
G runs a leg's board into its SOA, H warms one, in turn (`GLITCHED`); the hottest board says
its heat as it reports it on the bus (`machine.heat`). Ctrl and a letter lays what she trips on
where her walk meets it (`TRIPS`, `machine.events`).
"""
import argparse
import csv
import math
import os
import sys
import time

from coaxial.comm.session import Origin
from coaxial.graphics import gpu, gynoid
from machine import ansi, style
from machine.director import moment
from machine.figure import JOINTS, SEGMENTS, frames, quat
from machine.routines import TYPES
from machine.running import Running
from terminal.loader import TO_MENU
from terminal.views.overlay import (BOX_GROUND, BRAKE_INK, CALLOUT_W, DARK_INK, DRIVE_INK, LIGHT_INK,
                                    NUMBER_INK, SMALL, STAND, STANDING, TORQUE_INK, data_labels,
                                    data_legend, traffic)
from terminal.ui import screen as _screen
from terminal.ui.screen import PORT, FPS_CAP, closing, run_view, say
from terminal.ui.scroll import HUD_WIDTH
from terminal.ui.stage import frame_of, hud, stage
from tools import REPO

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

#: The boards G and H glitch, in turn, and how long G's SOA lasts, s.
GLITCHED, SOA_S = ('left_knee', 'right_knee', 'left_hip', 'right_hip'), 0.5

#: What she trips on, by the digit that lays it: a hole, a rug, a threshold (tröskel), a
#: slippery patch, a lace, a stair. On Ctrl letters, Ctrl+S paused the terminal (XOFF) and Ctrl+H
#: came as a backspace (2026-09-28).
TRIPS = dict(zip('123456', ('hole', 'rug', 'sill', 'slip', 'lace', 'stairs')))

#: Where R's recordings go, a CSV from a press to the next: every state the page hears from her
#: (a slice's, 10-20 a second of her time) - the page's yaw, her state, each joint, each
#: segment's place in the world.
RECORDINGS = os.path.join(REPO, 'build', 'recordings')
HEADER = (['t', 'stage', 'yaw', 'speed', 'phase', 'left_load', 'right_load',
           'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'] + list(JOINTS)
          + ['%s_%s' % (seg[0], axis) for seg in SEGMENTS for axis in 'xyz']
          + ['set_' + j for j in JOINTS])

#: What the callouts show, T stepping through: each drive's torque, its heat (its worst node as
#: its board says it, the thermal observer's scale), its power; the legend's words and the heat's
#: span, C.
SHOWN = ('torque', 'heat', 'power')
SAYS = {'torque': 'TORQUE N m, of its peak', 'heat': 'HEAT C', 'power': 'POWER W, driving | braking'}
HEAT_SPAN = (25.0, 100.0)

#: The bars' full scale: the drive's peak torque, and its power at that torque and RAD_S; both
#: drawn through a square root, so a light load shows.
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


#: The drives called out: the strong ones - the legs' and the spine's - or all, or none; L
#: steps through them.
CALLED = {'strong': tuple(j for j in JOINTS if j == 'spine' or j.endswith(
    ('_hip', '_hip_roll', '_knee', '_ankle'))), 'all': JOINTS, 'none': ()}
CALLING = ('strong', 'all', 'none')


def labels(now, called, shown='torque'):
    """{joint: rows} for `gynoid.render`, each of `called` one row CALLOUT_W wide: the number
    `shown` (SHOWN) on a patch its size colours - the legend says what it is."""
    out = {}
    for joint in CALLED[called]:
        text, ground = _patch(now, joint, shown)
        out[joint] = [[(c, _ink_on(ground), ground) for c in text.rjust(CALLOUT_W)[-CALLOUT_W:]]]
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
        ('status', _status(now)),
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
          'fallen': 'DOWN', 'unfold': 'GET UP', 'prop': 'GET UP', 'sit': 'GET UP',
          'fold': 'GET UP', 'lift': 'GET UP', 'crouch': 'GET UP'}


def _status(now):
    """BODY's status: her stage in a word and by its moment; tripped on a lace, TRIP; down with
    no get-up left, DOWN counting down to her landing again."""
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
    """What she is about to trip on: a lace holding her foot, the nearest prop on the floor and
    how far ahead of her pelvis, or one asked and the strides before it befalls her."""
    props = now.get('props', ())
    if any(p[0] == 'lace' for p in props):
        return 'lace caught'
    near = [(p[1][2] - now['where'][2], p[0]) for p in props
            if p[0] != 'lace' and p[1][2] - now['where'][2] > -AWAY_M]
    if near:
        metres, kind = min(near)
        return '%s under her' % kind if abs(metres) <= NEAR_M else (
            '%s %.1f m ahead' % (kind, metres) if metres > 0.0 else '%s behind' % kind)
    if now.get('armed'):
        kind, left = now['armed']
        return '%s in %d strides' % (kind, left + 1)
    return '1-6'


def _tripped(event):
    def trip(state):
        state['tripped'] = event
        state['body'].send(event=event)
    return trip


def _glitched(kind):
    def glitch(state):
        joint = GLITCHED[state['glitches'] % len(GLITCHED)]
        state['glitches'] += 1
        state['glitched'] = (joint, kind)
        state['body'].send(glitch=(joint, kind, SOA_S))
    return glitch


def _turned(step):
    return lambda state: state.update(yaw=state['yaw'] + step)


def _zoomed(k):
    return lambda state: state.update(zoom=max(ZOOM[0], min(ZOOM[1], state['zoom'] * k)))


def _paced(step):
    def pace(state):
        state['cadence'] = max(CADENCE[0], min(CADENCE[1], state['cadence'] + step))
        state['body'].send(cadence=state['cadence'])
    return pace


def _knob(value, unit):
    """A style knob's value as the STYLE box says it: metres in mm."""
    return '%.0f mm' % (value * 1e3) if unit == 'm' else ('%.2f %s' % (value, unit)).rstrip()


def _next_knob(state):
    """K: the next style knob to trim."""
    state['knob'] = style.NAMES[(style.NAMES.index(state['knob']) + 1) % len(style.NAMES)]


def _trimmed(steps):
    """, and .: the knob picked a step down or up, the walk eased over to it (`machine.style`)."""
    return lambda state: state['body'].send(style=(state['knob'], steps))


#: The page plays her back LAG_S of her time behind the newest state said, its clock's pace her
#: process's ratio to real time and CATCH_UP of the lag's error a second, eased over PACE_S; a
#: buffer of KEEP_S. Drawn as each state
#: came, a frame at 15 a second showed the same state again 65 times in 235 and the rest 0.036 s
#: of her time apart with 0.022 of spread: her process runs 0.7 of real time in slices of 0.05 s
#: (2026-09-28).
LAG_S, CATCH_UP, PACE_S, KEEP_S = 0.2, 1.0, 0.5, 1.0


class Playback:

    """Her states said, played back on a clock of their own at an even pace, the frame's state
    blended between the two it falls between: `push(states)`, `at(wall)`."""

    def __init__(self):
        self.states, self.shown, self.wall, self.pace = [], None, None, 1.0

    def push(self, states):
        self.states += states
        if self.states:
            newest = self.states[-1]['t']
            self.states = [s for s in self.states if s['t'] >= newest - KEEP_S]

    def at(self, wall):
        """The state to draw at `wall` seconds, or None before the first."""
        if not self.states:
            return None
        newest = self.states[-1]['t']
        if self.shown is None or self.shown > newest or self.shown < self.states[0]['t']:
            self.shown, self.wall = newest - LAG_S, wall
        dt = max(0.0, wall - self.wall)
        self.wall = wall
        ratio = min(1.0, self.states[-1].get('ratio', 1.0))
        want = ratio + CATCH_UP * ((newest - LAG_S) - self.shown)
        self.pace += (want - self.pace) * min(1.0, dt / PACE_S)
        self.shown = min(newest, self.shown + max(0.0, self.pace) * dt)
        after = next((i for i, s in enumerate(self.states) if s['t'] >= self.shown),
                     len(self.states) - 1)
        b = self.states[after]
        a = self.states[max(0, after - 1)]
        span = b['t'] - a['t']
        k = 0.0 if span <= 1e-9 else max(0.0, min(1.0, (self.shown - a['t']) / span))
        return dict(b, t=self.shown, **_blended(a, b, k))


def _blended(a, b, k):
    """The joints, the pelvis's place, turn and speed k of the way from state a to b."""
    turn = [x + (y - x) * k for x, y in zip(a['turn'], b['turn'])]
    norm = math.sqrt(sum(c * c for c in turn)) or 1.0
    return {'angles': {j: v + (b['angles'].get(j, v) - v) * k for j, v in a['angles'].items()},
            'where': tuple(x + (y - x) * k for x, y in zip(a['where'], b['where'])),
            'turn': tuple(c / norm for c in turn), 'speed': a['speed'] + (b['speed'] - a['speed']) * k}


def row(now, yaw):
    """A recording's row (HEADER): her state at the page's `yaw`, each joint, each segment's
    place, and each joint as the director asked it."""
    placed = frames(now['angles'], now['where'], quat(*now['turn']))
    asked = now.get('set', {})
    return ([round(now['t'], 4), now['stage'], yaw, now['speed'], now['phase']]
            + list(now['loads']) + list(now['where']) + list(now['turn'])
            + [now['angles'].get(j, 0.0) for j in JOINTS]
            + [v for seg in SEGMENTS for v in placed[seg[0]][0]]
            + [asked.get(j, float('nan')) for j in JOINTS])


def _recorded(state):
    """R: recording from now; R again: written to RECORDINGS, its name on the page."""
    if state['recording'] is None:
        state['recording'], state['recorded'] = [], None
        return
    rows, state['recording'] = state['recording'], None
    os.makedirs(RECORDINGS, exist_ok=True)
    path = os.path.join(RECORDINGS, time.strftime('humanoid_%Y%m%d_%H%M%S.csv'))
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(HEADER)
        writer.writerows(rows)
    state['recorded'] = path


def _pushed(state):
    state['side'] = -state['side']
    state['body'].send(push=(state['side'] * PUSH_N, 0.0, 0.0), seconds=PUSH_S)


#: What each key does to the view's state.
KEYS = dict(
    [('left', _turned(-TURN_DEG)), ('right', _turned(TURN_DEG)),
     ('[', _paced(-CADENCE_STEP)), (']', _paced(CADENCE_STEP))]
    + [(k, _pushed) for k in 'pP']
    + [(k, _glitched('soa')) for k in 'gG'] + [(k, _glitched('hot')) for k in 'hH']
    + [(k, _tripped(event)) for k, event in TRIPS.items()]
    + [(k, lambda state: state['body'].send(restart=True)) for k in 'aA']
    + [(k, lambda state: state.update(orbit=not state['orbit'])) for k in 'oO']
    + [(k, lambda state: state.update(called=CALLING[(CALLING.index(state['called']) + 1)
                                                     % len(CALLING)])) for k in 'lL']
    + [(k, _recorded) for k in 'rR']
    + [(k, _next_knob) for k in 'kK'] + [(',', _trimmed(-1)), ('.', _trimmed(1))]
    + [(k, lambda state: state.update(shown=SHOWN[(SHOWN.index(state['shown']) + 1)
                                                  % len(SHOWN)])) for k in 'tT']
    + [(k, lambda state: state.update(yaw=YAW, zoom=1.0)) for k in 'vV']
    + [(k, lambda state: state.update(dressed=not state['dressed'])) for k in 'cC']
    + [(k, lambda state: state.update(data=not state['data'])) for k in 'dD']
    + [(k, _zoomed(1.1)) for k in '+='] + [(k, _zoomed(1.0 / 1.1)) for k in '-_'])


def act_on(typed, state):
    for key in typed:
        if key in KEYS:
            KEYS[key](state)


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
    body = Running(cadence)
    say('ok', 'body', '55 kg, %d drives, MuJoCo at 1 kHz in its own process'
        % sum(len(s.actuators) for s in TYPES['gynoid'].body))
    card = gpu.adapter()
    lit = gpu.LitRaster(found=card) if card is not None else None
    name = lit.name if lit is not None else 'this process, dots'
    say('ok', 'drawing', name)
    gynoid.body(), gynoid.body(dressed=False)

    board_view = stage()
    terminal = board_view.is_terminal
    state = {'body': body, 'cadence': cadence, 'orbit': False, 'yaw': YAW, 'zoom': 1.0,
             'side': 1.0, 'last_t': None, 'called': 'strong', 'follow': gynoid.Follow(),
             'recording': None, 'recorded': None, 'glitches': 0, 'glitched': None,
             'tripped': None, 'playback': Playback(), 'shown': 'torque', 'dressed': True,
             'data': False, 'traffic': None, 'knob': style.NAMES[0]}

    def draw():
        said = []
        body.latest(into=said)
        if state['recording'] is not None:
            state['recording'] += [row(s, state['yaw']) for s in said]
        state['playback'].push(said)
        now = state['playback'].at(time.perf_counter())
        if state['orbit'] and now is not None:
            if state['last_t'] is not None:
                state['yaw'] += ORBIT_DEG_S * max(0.0, now['t'] - state['last_t'])
            state['last_t'] = now['t']
        width, height = size_of(board_view, args)
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
                                          legend=data_legend(width), dressed=state['dressed']))
        else:
            x, y, z = now['where']
            camera = state['follow'](z, now['speed'], now['t'])
            art = '\n'.join(gynoid.render(now['angles'], width, height, yaw=state['yaw'],
                                          zoom=state['zoom'], colour=terminal, travel=camera,
                                          lit=lit, root=((x, y, z - camera),
                                                         quat(*now['turn'])),
                                          labels=labels(now, state['called'], state['shown']),
                                          heat={j: h[0] for j, h in now['heat'].items()},
                                          props=now.get('props'),
                                          legend=(legend(state['shown'], width)
                                                  if state['called'] != 'none' else None),
                                          dressed=state['dressed']))
        side = boxes(state, now, name) + ([traffic(state, now)] if state['data'] and now else [])
        return frame_of(board_view, ORIGIN, TITLE, art, side,
                        (('[ ]', 'PACE'), ('P', 'PUSH'), ('G', 'SOA'), ('H', 'HOT'),
                         ('1-6', 'HOLE RUG SILL SLIP LACE STAIRS'),
                         ('K , .', 'STYLE'), ('A', 'AGAIN'), ('L', 'LABELS'), ('T', 'SHOWN'),
                         ('<- ->', 'TURN'), ('+ -', 'ZOOM'), ('O', 'ORBIT'), ('R', 'RECORD'),
                         ('V', 'VIEW'), ('C', 'CLOTHES'), ('D', 'DATA'),
                         ('Q', 'EXIT'), ('ESC', 'MENU')))

    leaving = None
    try:
        if args.frames:
            # The view suite asks for a few frames: wait for her first state, a second at most.
            for _ in range(100):
                if body.latest() is not None:
                    break
                time.sleep(0.01)
        leaving = run_view(board_view, terminal, 1.0 / max(1.0, args.hz),
                           args.frames, draw, on_input=lambda typed, _moved: act_on(typed, state))
    finally:
        body.close()
        sys.stdout.write('\n')
        closing([('body', 'stopped, her process ended'),
                 ('boards', 'none - every drive was simulated')], terminal, 0)
    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
