#!/usr/bin/env python3
"""A slender gynoid on its latent loop under gravity: a body with mass, each joint a drive."""
import argparse
import collections
import math
import os
import sys
import time

from rich.text import Text

from coaxial.comm.session import Origin
from coaxial.draw import cross_section, gauges as _gauges
from coaxial.graphics import arrows, gynoid, horizon, likeness
from coaxial.graphics.callouts import packed
from machine.body import figure
from machine.law import se2
from machine.process import sprung
from machine.body.figure import JOINTS, quat
from machine.rig.routines import TYPES
from terminal import boards
from terminal.loader import answer
from terminal.views import gym, viewpoint
from terminal.views.humanoid_column import RAD_S, actual, boxes, telemetry_core
from terminal.views.humanoid_keys import CHARGE_MAX_N, PAD, charging, clicked, frame, heard, pad, rows_of
from terminal.views.playback import Playback
from terminal.views.scanner import VisionScanner, docked_vision
from terminal.ui import bindings, recorder, screen as _screen
from terminal.ui.ahead import Ahead
from terminal.ui.screen import PORT, FPS_CAP, closing, run_view, say
from terminal.ui.stage import boot, frame_of, hud, stage
from terminal.ui.taskbar import footer, help_rows, helped, room

_screen.CHATTER = False

TITLE = 'HUMANOID'
ORIGIN = Origin(False, 'dynamic', 0, 'dynamic', 'GRAVITY 9.81 - MUJOCO', 'dynamic', 0)

BOX_GROUND, TORQUE_INK, DRIVE_INK, BRAKE_INK, TEMP_INK, NUMBER_INK, DARK_INK, LIGHT_INK = (
    (38, 46, 58), (255, 184, 80), (96, 214, 255), (255, 96, 128), (255, 116, 64), (214, 220, 228),
    (16, 18, 22), (236, 240, 244))
CALLOUT_W, METER = 4, 21
#: The states the robot's drawings were last asked at, the newest last: the one a drawing `behind` them shows.
ASKED = 32


def meter(value, marks):
    lo, hi, word = marks[0][0], marks[-1][0], marks[1][1]
    cell = lambda v: max(0, min(METER - 1, round((v - lo) / (hi - lo) * (METER - 1))))
    start, dot = max(0, min(METER - len(word), cell(marks[1][0]) - len(word) // 2)), cell(value)
    out, lit = Text(marks[0][1] + ' ', style='bar.dim'), start <= dot < start + len(word)
    for i in range(METER):
        in_w = start <= i < start + len(word)
        out.append(word[i - start] if in_w else ('\u25cf' if i == dot else '\u2500'),
                   style='bar' if (lit if in_w else i == dot) else 'bar.dim')
    out.append(' ' + marks[-1][1], style='bar.dim')
    return out


CHARGES = ((0.0, '0'), (CHARGE_MAX_N / 2.0, 'N'), (CHARGE_MAX_N, '%.0f' % CHARGE_MAX_N))
LANDED_S = 0.5
CHARGE_INK, LANDED_INK = (255, 96, 128), (236, 240, 244)


def torso(now):
    at, turn = figure.frames(now['angles'], now['where'], quat(*now['turn']))['torso']
    return figure.add(at, figure.apply(turn, figure.shares()['torso'][1]))


def marked(state, now, clock):
    if now is None:
        return None
    charge, landed = charging(state, clock), state.get('landed')
    if charge:
        k = 0.3 + 0.7 * charge / CHARGE_MAX_N
        return torso(now), tuple(int(c * k) for c in CHARGE_INK)
    if landed and clock - landed[0] < LANDED_S:
        return torso(now), LANDED_INK
    return None


def _bar4(val):
    n = max(0, min(4, round(max(0.0, min(1.0, float(val))) * 4)))
    return '\u25b0' * n + '\u25b1' * (4 - n)


PROMPT_INK = (255, 196, 80)


def badge(width, height, state):
    """The viewport's corners: the tour's step and LLM command feed at its top, the robot's balance
    strip under it, the interactive LLM prompt bar and what Space and F offer at its foot
    (`gym.prompt`)."""
    if width < 30 or height < 6:
        return {}
    out, ink = {}, packed((120, 220, 255), (10, 22, 35))
    if gym.touring(state):
        tr = state['tour']
        th, cm = getattr(tr, 'last_thought', ''), getattr(tr, 'last_cmd', '')
        line0 = (' [TOUR] %s  LLM \u25b8 %s (%s) ' % (tr.current_label(), th, cm)) if th else (' [TOUR] %s ' % tr.current_label())
        out.update({(0, 1 + i): (ord(ch), ink) for i, ch in enumerate(line0[:width - 2])})
    # a said prompt's thought and command live in the chat pane alone: echoed here too the page
    # wore two prompt lines
    offer = gym.prompt(state)
    if offer:
        p_ink = packed(PROMPT_INK, (10, 22, 35))
        out.update({(height - 2, max(0, (width - len(offer) - 2) // 2) + i): (ord(ch), p_ink)
                    for i, ch in enumerate(' %s ' % offer[:width - 2])})
    now = state.get('now')
    if now is not None and width >= 54 and height >= 10:
        bal, thm, stb, tag = telemetry_core(now)
        strip = f' BAL {_bar4(bal)}  THM {_bar4(thm)}  STB {_bar4(stb)}  [{tag}] '
        fg = (255, 96, 96) if tag == 'CRITICAL' else ((255, 196, 80) if tag == 'ALERT' else (96, 214, 200))
        s_ink = packed(fg, (10, 22, 35))
        out.update({(1, 1 + i): (ord(ch), s_ink) for i, ch in enumerate(strip[:width - 2])})
    return out


def gauges(state):
    out = Text()
    charge = charging(state, time.monotonic())
    if charge:
        out.append(' PUSH ', style='bar.dim')
        out.append_text(meter(charge, CHARGES))
        return out
    bal, thm, stb, tag = telemetry_core(state.get('now'))
    st_v = 'alarm' if tag == 'CRITICAL' else 'bar'
    for lbl, val, st_b in ((' BAL ', bal, st_v), ('  THM ', thm, 'bar'), ('  STB ', stb, st_v)):
        out.append(lbl, style='bar.dim')
        out.append(_gauges.gauge(val, 6, colour=False), style=st_b)
    return out


SAYS = {'torque': 'TORQUE N m, of its peak', 'power': 'POWER W, driving | braking', 'temp': 'TEMP C, winding'}


def _mixed(a, b, k):
    return tuple(int(x + (y - x) * max(0.0, min(1.0, k))) for x, y in zip(a, b))


def _patch(now, joint, shown):
    torque, power, peak = now['torque'][joint], now['power'][joint], max(1.0, float(now['peak'][joint]))
    if shown == 'power':
        ink = DRIVE_INK if power >= 0.0 else BRAKE_INK
        return '%.0f' % abs(power), _mixed(BOX_GROUND, ink, math.sqrt(abs(power) / (peak * RAD_S)))
    if shown == 'temp':
        deg_c = float((now.get('temp') or {}).get(joint, 25.0))
        return '%.0f' % deg_c, _mixed(BOX_GROUND, TEMP_INK, max(0.0, min(1.0, (deg_c - 25.0) / 85.0)))
    return '%.0f' % abs(torque), _mixed(BOX_GROUND, TORQUE_INK, math.sqrt(abs(torque) / peak))


def _ink_on(ground):
    return DARK_INK if 0.3 * ground[0] + 0.59 * ground[1] + 0.11 * ground[2] > 128 else LIGHT_INK


def legend(shown, width):
    cells = [(c, NUMBER_INK, None) for c in f' T: {SAYS[shown]} ']
    if shown == 'power':
        cells += ([(' ', NUMBER_INK, _mixed(BOX_GROUND, DRIVE_INK, k / 3.0)) for k in range(4)] + [(' ', None, None)]
                  + [(' ', NUMBER_INK, _mixed(BOX_GROUND, BRAKE_INK, k / 3.0)) for k in range(4)])
    else:
        ink = TEMP_INK if shown == 'temp' else TORQUE_INK
        cells += [(' ', NUMBER_INK, _mixed(BOX_GROUND, ink, k / 7.0)) for k in range(8)]
    return cells[:width]


CALLED = {'strong': tuple(j for j in JOINTS if j == 'spine' or j.endswith(
    ('_hip', '_hip_roll', '_knee', '_ankle'))), 'all': JOINTS, 'none': ()}


def labels(now, called, shown='torque'):
    return {j: [[(c, _ink_on(g), g) for c in t.rjust(CALLOUT_W)[-CALLOUT_W:]]]
            for j in CALLED[called] for t, g in (_patch(now, j, shown),)}


chat_box = gym.chat_box


def size_of(console, args, up=0.0, below=0):
    """(width, height) of the robot's drawing: the room beside the column, the rows the band, the bar, `up`
    of the help and the `below` its chat's drawer takes leave."""
    size = console.size if console.is_terminal else None
    width = args.width or max(24, room(size.width if size else 110))
    bar = footer(PAD['bar']).row_count + help_rows(PAD['groups'], up)
    return width, args.height or max(12, (size.height if size else 44) - 5 - bar - below)


def _seen(state):
    return state['skin'] if state['skin'] in ('mechanism', 'actuators') else None


#: The robot's speed over the floor past which it goes and the chase camera follows, m/s.
GOING_M_S = 0.1


def scripted(state, keys):
    """`--keys`: the frame's key typed as the console would, a key a frame from the first, '.'
    none - a page rendered under a key (tools/render/page.py)."""
    k = state.get('frame', 0)
    state['frame'] = k + 1
    if k < len(keys) and keys[k] != '.':
        bindings.act_on(PAD, [keys[k]], state)


def _step_frame(state, dt):
    """A frame's commands: the tour's due, what the robot's keys hold (`humanoid_keys.frame`); the chase
    camera eased behind it."""
    if gym.touring(state):
        state['tour'].step(state, dt)
    t_mono, now = time.monotonic(), state.get('now')
    heading = float(now.get('heading') or 0.0) if now else 0.0
    frame(state, t_mono, heading)
    going = bool(now) and (math.hypot(*now['velocity']) > GOING_M_S or now.get('stage') == 'walk')
    viewpoint.chased(state, heading, going, t_mono)
    return t_mono


def _blank(width, height):
    return '\n'.join(' ' * width for _ in range(height))


def _draw_viewport(state, now, width, height, terminal):
    """(This frame's drawing of the robot, the state it was asked at): the frame before's where each drawing
    is waited for, the newest drawn's on a live page (`Ahead.waits`, `Ahead.behind`), blank before the
    first; its own at `now` asked for the next (`state['ahead']`, in its own process)."""
    if now is None:
        return _blank(width, height), state.get('drawn')
    x, y, z = now['where']
    camera, offer = state['follow']((x, z), now['velocity'], now['t']), gym.offered(state)
    docked = docked_vision(state['scanner'], now, width, height, offer)
    inset = {**gym.ballooned(state, offer, width, height, camera, docked), **docked, **badge(width, height, state),
             **recorder.corner(state.get('recording'), width)}
    ahead = state['ahead']
    lines = ahead.drawn(
        now['angles'], width, height, yaw=state['yaw'], pitch=state['pitch'], pan=viewpoint.panned(state),
        zoom=state['zoom'], colour=terminal, travel=camera, lit=True,
        root=((x - camera[0], y, z - camera[1]), quat(*now['turn'])),
        labels=labels(now, state['called'], state['shown']), props=now.get('props'), dents=now.get('dents'),
        rounds=now.get('rounds'), mark=marked(state, now, time.monotonic()),
        legend=legend(state['shown'], width) if state['called'] != 'none' else None,
        dressed=state['skin'] == 'dressed', like=likeness.FOUND, see=_seen(state), inset=inset, arrows=(now, state.get('pad')),
        hosed=None if state['skin'] == 'dressed' else now['t'],
        horizon=horizon.segments(now, now.get('props') or (), state.get('horizon') or 'off') or None)
    asked = state.setdefault('asked', collections.deque(maxlen=ASKED))
    asked.append(now)
    state['drawn'] = asked[max(0, len(asked) - 1 - ahead.behind)]
    return (_blank(width, height), now) if lines is None else ('\n'.join(lines), state['drawn'])


def _boot_humanoid(_board_view, state):
    return sprung.boot_humanoid(state, boot, say, Ahead, boards.EmulatedLimb)


def _compiled():
    """The robot's control system compiled for its sources before it is laid (`tools.sim.warmed`), on the
    boot's strip where it is behind: the robot's first pass on its C, not numpy's own a part at a time."""
    from tools.sim import warmed
    if warmed.current():
        return
    with boot('COMPILING CONTROL SYSTEM') as step:
        ok, told = warmed.ensure(step)
    say('ok' if ok else 'fail', 'control system', told)


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--hz', type=float, default=FPS_CAP)
    parser.add_argument('--simulated', action='store_true')
    parser.add_argument('--max', action='store_true', help='run flat out without real-time pacing')
    for flag in ('--frames', '--width', '--height'):
        parser.add_argument(flag, type=int, default=0)
    parser.add_argument('--keys', default='', help="a key a frame from the first, '.' none")
    parser.add_argument('--tour', action='store_true', help='start the autonomous room tour on launch')
    parser.add_argument('--prompt', default='', help='send an initial natural-language LLM prompt to the robot')
    parser.add_argument('--telemetry', type=int, default=0,
                        help='serve the robot\'s latent, impedance and wrench live on 127.0.0.1:PORT (machine.process.introspect)')
    args, board_view = parser.parse_args(argv), stage()
    if args.telemetry:
        from machine.process import worker as _worker
        os.environ[_worker.TELEMETRY] = str(args.telemetry)
    terminal, body, leaving, name = board_view.is_terminal, None, None, 'a process of its own, dots'
    emu_port = args.port if (not args.simulated and str(args.port).startswith('emulator://')) else None
    state = {'body': None, 'orbit': False, **viewpoint.HOME, 'side': 1.0, 'last_t': None,
             'called': 'strong', 'follow': gynoid.Follow(), 'recording': None, 'recorded': None,
             'playback': Playback.steady(), 'shown': 'torque', 'skin': 'dressed', 'scanner': VisionScanner(),
             'now': None, 'tour': None, 'pad': pad(), 'max': args.max,
             'emu_port': emu_port, 'emu': None, 'origin': ORIGIN,
             'chat_rows': [gym.CHAT_GREETING]}

    def draw():
        scripted(state, args.keys)
        t_mono = _step_frame(state, 1.0 / max(1.0, args.hz))
        said = []
        state['body'].latest(into=said)
        heard(state, state['body'].heard())
        if state['recording'] is not None:
            recorder.added(state['recording'], rows_of(state, said))
        state['playback'].push(said)
        now = state['now'] = state['playback'].at(_screen.drawn_next())
        if now is not None:
            state['body'].shown(now['t'])
            if state.get('emu') is not None:
                state['emu'].sync(now)
        actual(now, state['pad'])
        if now is not None:
            viewpoint.orbited(state, now['t'])
            gym.refused_told(state, now)
        gym.drain_plan(state)
        up, below = helped(t_mono), gym.drawer(state, t_mono)
        w_vp, h_vp = size_of(board_view, args, up, below)
        # Drawn ahead: this frame shows the state its drawing was asked at, its boxes too.
        art, shown = _draw_viewport(state, now, w_vp, h_vp, terminal)
        return frame_of(board_view, state['origin'], TITLE, art, boxes(state, shown, name), PAD,
                        under=chat_box(state, w_vp) if below else None, under_rows=below,
                        gauges=gauges(state))

    try:
        if not args.frames:
            gym.warm(state)
            _compiled()
        body, name = _boot_humanoid(board_view, state)
        # A live page never waits for its drawing; a smoke's frames each its own state's.
        state['ahead'].waits = bool(args.frames)
        if args.tour:
            gym.toggle_tour(state)
        if args.prompt:
            gym.submit_prompt(state, args.prompt)
        wheeled = lambda _keys, wheel: viewpoint.zoomed(state, max(0.5, 1.0 + wheel)) if wheel else None
        leaving = run_view(board_view, terminal, 1.0 / max(1.0, args.hz), args.frames, draw, on_input=wheeled,
                           pad=PAD, state=state, **viewpoint.mouse(state, lambda: size_of(
                               board_view, args, below=state['drawer']['rows']), clicked))
    finally:
        gym.release(state)
        if state.get('emu') is not None:
            state['emu'].close()
        if body is not None:
            body.close()
        if state.get('ahead') is not None:
            state['ahead'].close()
        sys.stdout.write('\n')
        emu = state.get('emu')
        closing([('body', 'stopped, the robot\'s process ended'),
                 ('boards', emu.closed_note() if emu else 'none - every drive was simulated')], terminal, 0)
    return answer(leaving)


if __name__ == '__main__':
    sys.exit(main())

