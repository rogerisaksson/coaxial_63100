#!/usr/bin/env python3
"""A slender gynoid on her latent loop under gravity: a body with mass, each joint a drive.

    python terminal/views/show_humanoid.py
    python terminal/views/show_humanoid.py --frames 30
"""
import argparse
import math
import os
import sys
import time

from rich.text import Text

from coaxial.comm.session import Origin
from coaxial.draw import cross_section, gauges as _gauges
from coaxial.graphics import gpu, gynoid
from coaxial.graphics.shapes import view
from machine import figure, se2, sprung
from machine.figure import JOINTS, quat
from machine.routines import TYPES
from terminal import keybindings
from terminal.loader import TO_MENU
from terminal.views import gym, viewpoint
from terminal.views.humanoid_keys import CHARGE_MAX_N, act_on, charging, released, row, step_se2, table
from terminal.views.playback import Playback
from terminal.views.scanner import VisionScanner, docked_vision
from terminal.ui import console as _console, screen as _screen
from terminal.ui.screen import PORT, FPS_CAP, closing, run_view, say
from terminal.ui.scroll import HUD_WIDTH
from terminal.ui.stage import boot, footer, frame_of, help_rows, helped, hud, stage

_screen.CHATTER = False     # the boot bar replaced the scroll

TITLE = 'HUMANOID'

#: Where she runs: no port, no board; the band says it and the chip is DYNAMIC.
ORIGIN = Origin(False, 'dynamic', 0, 'dynamic', 'GRAVITY 9.81 - MUJOCO', 'dynamic', 0)

#: A callout's inks: its ground, the torque's, the power's driving and braking, the legend's
#: words, a number on a dark patch and on a light; its width inside its frame, cells.
BOX_GROUND, TORQUE_INK, DRIVE_INK, BRAKE_INK, TEMP_INK, NUMBER_INK, DARK_INK, LIGHT_INK = (
    (38, 46, 58), (255, 184, 80), (96, 214, 255), (255, 96, 128), (255, 116, 64), (214, 220, 228),
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
    """The mark: a push's, charging or landed; or TPW crosshair reticle in walker mode."""
    charge, landed = charging(state, clock), state.get('landed')
    if charge:
        k = 0.3 + 0.7 * charge / CHARGE_MAX_N
        return torso(now), tuple(int(c * k) for c in CHARGE_INK)
    if landed and clock - landed[0] < LANDED_S:
        return torso(now), LANDED_INK
    if state.get('wasd_mode') and now is not None:
        vx, vy, vz = along(state)
        wx, wy, wz = torso(now)
        aim_pt = (wx + 1.8 * vx, wy + 1.8 * vy, wz + 1.8 * vz)
        return aim_pt, (96, 220, 255)
    return None


def _bar4(val):
    """4-cell block gauge for compact cybernetic telemetry strips."""
    n = max(0, min(4, round(max(0.0, min(1.0, float(val))) * 4)))
    return '▰' * n + '▱' * (4 - n)


_CONTROLLED_STUNTS = frozenset(sprung.STUNT_PHASES) | {
    'sit_chair', 'lie_bed', 'lie_floor', 'lean', 'parry_shove', 'catch', 'restore'}


def telemetry_core(now):
    """Return (bal, thm, stb, status_tag) in [0..1] for Balance, Thermal/Power headroom, and Stability."""
    if not now:
        return 1.0, 1.0, 1.0, 'NOMINAL'
    watts = float(now.get('watts', 45.0))
    thm = max(0.0, min(1.0, 1.0 - max(0.0, watts - 40.0) / 1160.0))
    if now.get('temp'):
        max_c = max(float(v) for v in now['temp'].values())
        thm = min(thm, max(0.0, min(1.0, (125.0 - max_c) / 100.0)))
    if now.get('fallen'):
        return 0.0, thm, 0.0, 'CRITICAL'
    doing = str(now.get('doing', 'rest'))
    if doing in _CONTROLLED_STUNTS:
        bal, stb = 0.95, 0.95
    elif doing == 'controlled_fall':
        bal, stb = 0.55, 0.55
    else:
        turn = now.get('turn', (1.0, 0.0, 0.0, 0.0))
        qx, qz = float(turn[1]), float(turn[3])
        up_y = 1.0 - 2.0 * (qx * qx + qz * qz)
        bal = max(0.0, min(1.0, (up_y - 0.5) / 0.5))
        loads = now.get('loads', (160.0, 160.0))
        load_norm = min(1.0, (float(loads[0]) + float(loads[1])) / 240.0)
        where_y = float((now.get('where') or (0.0, 0.90, 0.0))[1])
        stb = max(0.0, min(1.0, 0.5 * load_norm + 0.5 * min(1.0, where_y / 0.85)))
    worst = min(bal, thm, stb)
    tag = 'CRITICAL' if worst < 0.35 else ('ALERT' if worst < 0.65 else 'NOMINAL')
    return bal, thm, stb, tag


def tpw_badge(wasd_mode, width, height, state=None):
    """Cells for the Third Person Control & cybernetic telemetry strip in the top-left of the 3D viewport."""
    if width < 30 or height < 6:
        return {}
    out = {}
    from coaxial.graphics.callouts import packed
    text = " [Third Person Control ON] " if wasd_mode else " [Third Person Control OFF] "
    ink = packed((120, 220, 255), (10, 22, 35)) if wasd_mode else packed((110, 130, 150), (10, 22, 35))
    r, c = 0, 1
    for i, ch in enumerate(text):
        out[(r, c + i)] = (ord(ch), ink)
    now = state.get('now') if isinstance(state, dict) else None
    if now is not None and width >= 54 and height >= 10:
        bal, thm, stb, tag = telemetry_core(now)
        strip = " BAL %s  THM %s  STB %s  [%s] " % (_bar4(bal), _bar4(thm), _bar4(stb), tag)
        fg = (255, 96, 96) if tag == 'CRITICAL' else ((255, 196, 80) if tag == 'ALERT' else (96, 214, 200))
        strip_ink = packed(fg, (10, 22, 35))
        for i, ch in enumerate(strip[:width - 2]):
            out[(1, c + i)] = (ord(ch), strip_ink)
    return out


def gauges(state):
    """The band's meter: control mode status, cybernetic core gauges (BAL/THM/STB), or push meter."""
    wasd = state.get('wasd_mode', False)
    out = Text.assemble(
        (' [Third Person Control ON] ' if wasd else ' [Third Person Control OFF] ',
         'bold cyan' if wasd else 'bar.dim'),
        (' §/~ ', 'bar.dim')
    )
    charge = charging(state, time.monotonic())
    if charge:
        out.append('  PUSH ', style='bar.dim')
        out.append_text(meter(charge, CHARGES))
        return out
    bal, thm, stb, tag = telemetry_core(state.get('now'))
    style_val = 'alarm' if tag == 'CRITICAL' else 'bar'
    out.append(' BAL ', style='bar.dim')
    out.append(_gauges.gauge(bal, 6, colour=False), style=style_val)
    out.append('  THM ', style='bar.dim')
    out.append(_gauges.gauge(thm, 6, colour=False), style='bar')
    out.append('  STB ', style='bar.dim')
    out.append(_gauges.gauge(stb, 6, colour=False), style=style_val)
    return out


#: What the callouts and joint instruments show, T stepping through: torque, power, winding temp.
SHOWN = ('torque', 'power', 'temp')
SAYS = {
    'torque': 'TORQUE N m, of its peak',
    'power': 'POWER W, driving | braking',
    'temp': 'TEMP C, winding',
}

#: The bars' full scale: the drive's peak torque, its power at that torque and RAD_S; through a
#: square root, so a light load shows.
RAD_S = 4.0


def _mixed(a, b, k):
    return tuple(int(x + (y - x) * max(0.0, min(1.0, k))) for x, y in zip(a, b))


def _patch(now, joint, shown):
    """(text, ground) of a joint's callout: the number `shown` and the colour it stands on."""
    torque = now['torque'][joint]
    power = now['power'][joint]
    peak = max(1.0, float(now['peak'][joint]))
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
    """The viewport's last row: what the callouts show, its key, and its colours."""
    words = ' T: %s ' % SAYS[shown]
    cells = [(c, NUMBER_INK, None) for c in words]
    if shown == 'power':
        cells += [(' ', NUMBER_INK, _mixed(BOX_GROUND, DRIVE_INK, k / 3.0)) for k in range(4)]
        cells += [(' ', None, None)]
        cells += [(' ', NUMBER_INK, _mixed(BOX_GROUND, BRAKE_INK, k / 3.0)) for k in range(4)]
    elif shown == 'temp':
        cells += [(' ', NUMBER_INK, _mixed(BOX_GROUND, TEMP_INK, k / 7.0)) for k in range(8)]
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
BAR = keybindings.BAR_NORMAL


def bar_for(state):
    """The bottom bar: adapts when Third Person Control (TPW) is active."""
    return keybindings.bar_for(state.get('wasd_mode', False))


GROUPS = keybindings.groups()


def size_of(console, args, up=0.0):
    """Cells for the drawing: the viewport's less the key help `up`, or --width/--height."""
    size = console.size if console.is_terminal else None
    width = args.width or max(24, (size.width if size else 110) - HUD_WIDTH - 6)
    bar = footer(BAR).row_count + help_rows(GROUPS, up)
    height = args.height or max(12, (size.height if size else 44) - 5 - bar)
    return width, height


def _sync_actual(now, se2_st):
    if not now:
        return (0.0, 0.0, 0.0)
    hd, vel, twa = float(now.get('heading', 0.0)), now.get('velocity', (0.0, 0.0)), now.get('twist_actual')
    sh, ch = math.sin(hd), math.cos(hd)
    vx_v, vy_v = float(vel[0]) * sh + float(vel[1]) * ch, float(vel[0]) * ch - float(vel[1]) * sh
    vx_a = float(twa[0]) if (twa and abs(twa[0]) > 1e-4 and abs(vx_v) < 1e-4 and 'speed' not in now) else float(now.get('speed', vx_v))
    wz_a = float(twa[2]) if (twa and abs(twa[2]) > 1e-4) else float((se2_st or {}).get('actual', (0.0, 0.0, 0.0))[2])
    act = (vx_a, float(twa[1]) if (twa and abs(twa[1]) > 1e-4) else vy_v, wz_a)
    if se2_st is not None:
        se2_st['actual'] = act
    return act


_ANSI = {}


def _ansi(raw):
    got = _ANSI.get(raw)
    if got is None:
        if len(_ANSI) > 512:
            _ANSI.clear()
        _ANSI[raw] = got = Text.from_ansi(raw)
    return got


def _joint_row(now, joint, label, shown):
    deg = float((now.get('angles') or {}).get(joint, 0.0)) if now else 0.0
    tau = abs(float((now.get('torque') or {}).get(joint, 0.0))) if now else 0.0
    pwr = float((now.get('power') or {}).get(joint, 0.0)) if now else 0.0
    deg_c = float((now.get('temp') or {}).get(joint, 25.0)) if now else 25.0
    pk = max(1.0, float((now.get('peak') or {}).get(joint, 100.0))) if now else 100.0
    p_cl = max(-999.0, min(999.0, pwr))
    if shown == 'power':
        cls = cross_section.WATTS if pwr >= 0.0 else cross_section.SOA_WARN
        bar = _gauges.gauge(0.5 + 0.5 * max(-1.0, min(1.0, pwr / (pk * RAD_S))), 6, cls=cls, centre=0.5)
        tail = '%+4.0fW %3.0f°C' % (p_cl, deg_c)
    elif shown == 'temp':
        bar = _gauges.gauge(_gauges.temp_share(deg_c), 6, cls=_gauges.thermometer_class(deg_c))
        tail = '%3.0f°C %+4.0fW' % (deg_c, p_cl)
    else:
        bar, tail = _screen.gauge(min(1.0, tau / pk), 6), '%3.0fNm %3.0f°C' % (min(999.0, tau), deg_c)
    return (label.rjust(10), _ansi('%+5.0f° %s %s' % (deg, bar, tail)))


def boxes(state, now, name):
    """The side column: her modes and her moves, the body, then each limb's joints."""
    wasd, se2_st, shown = state.get('wasd_mode', False), state.get('se2'), state.get('shown', 'torque')
    tw_act = _sync_actual(now, se2_st)
    raw_set = se2_st.get('setpoint', (0.0, 0.0, 0.0)) if se2_st is not None else (0.0, 0.0, 0.0)
    tw_now = now.get('twist', (0.0, 0.0, 0.0)) if now else (0.0, 0.0, 0.0)
    tw_nom = se2.NOMINAL_TWIST.get(now.get('doing', ''), (0.0, 0.0, 0.0)) if now else (0.0, 0.0, 0.0)
    tw_set = next((t for t in (raw_set, tw_now, tw_nom) if any(abs(v) > 1e-4 for v in t)), (0.0, 0.0, 0.0))
    bal, thm, stb, tag = telemetry_core(now)
    soa = lambda v: cross_section.SOA_TRIP if v < 0.35 else (cross_section.SOA_WARN if v < 0.65 else cross_section.SOA_OK)
    core_row = _ansi('%s %3.0f%% stb %s %3.0f%%' % (
        _gauges.gauge(bal, 4, cls=soa(bal)), bal * 100.0, _gauges.gauge(stb, 4, cls=soa(stb)), stb * 100.0))
    spd_act, spd_set = tw_act[0], float(tw_set[0])
    s_act = 0.5 + 0.5 * max(-1.0, min(1.0, spd_act / se2.RUN_SPEED))
    s_set = 0.5 + 0.5 * max(-1.0, min(1.0, spd_set / se2.RUN_SPEED))
    spd_bar = _gauges.gauge(s_act, 8, centre=0.5, marks=((s_set, _gauges.MARK),))
    loads = now['loads'] if now else (0.0, 0.0)
    soles_row = _ansi('%3.0fN %s %s %3.0fN' % (
        loads[0], _gauges.gauge(min(1.0, loads[0] / 350.0), 5),
        _gauges.gauge(min(1.0, loads[1] / 350.0), 5), loads[1]))
    watts = float(now['watts']) if now else 0.0
    max_c = max((float(v) for v in now['temp'].values()), default=25.0) if (now and now.get('temp')) else 25.0
    pwr_row = _ansi('%4.0f W  %s  %4.0f °C' % (watts, _screen.gauge(watts / 800.0, 8), max_c))
    rhy_row = _ansi('%4.2f Hz  phase   %s' % (
        float(now['cadence']) if now else 0.0, _gauges.gauge(float(now['phase']) if now else 0.0, 8)))
    out = gym.boxes(now, wasd=wasd, tour=state.get('tour')) + [hud('BODY', [
        ('   control', '%s  [%s]' % ('TPW (WASD)' if wasd else 'normal', tag)),
        ('    status', gym.status(now)[:25]),
        ('   balance', core_row),
        ('     speed', _ansi('%+5.2f m/s %s %+5.2f' % (spd_act, spd_bar, spd_set))),
        ('     twist', 'vx%+5.2f vy%+5.2f wz%+5.2f' % tw_act),
        ('    rhythm', rhy_row),
        ('     soles', soles_row),
        ('     power', pwr_row),
        ('   physics', 'x%.1f real time  T:%s' % (now['ratio'], shown) if now else '-'),
        (' autopilot', state['tour'].current_label()[:25]
         if state.get('tour') and not state['tour'].done else 'manual'),
        ('    record', 'R starts' if state['recording'] is None and not state['recorded']
         else 'on, %.1f s - R saves' % (len(state['recording']) / 60.0)
         if state['recording'] is not None else os.path.basename(state['recorded'])[:25]),
        ('  drawn by', name[:25])])]
    for sub in TYPES['gynoid'].body:
        out.append(hud(sub.name.replace('_', ' ').upper(), [
            _joint_row(now, j, j.split('_', 1)[-1] if sub.name != 'axis' else j, shown)
            for j in sub.actuators]))
    scanner = state.get('scanner')
    if scanner is not None and now is not None:
        det = scanner.scan(now, state.get('yaw', 0.0))
        out.append(hud('VISION (EYES)', [
            ('     model', scanner.model_name()[:25]),
            ('  detected', (det[0]['label'] if det else 'none')[:25]),
            ('    action', (det[0]['affordances'][0][1] if (det and det[0]['affordances']) else '-')[:25])]))
    return out


def _seen(state):
    """`gynoid.render`'s `see` as C has her: None her skin."""
    return state['skin'] if state['skin'] in ('mechanism', 'actuators') else None


KEYS = gym.table(table(viewpoint.KEYS, CALLING, SHOWN))


def _step_frame(state, body, dt):
    tour = state.get('tour')
    if tour is not None and not tour.done:
        state['_in_tour_step'] = True
        try:
            tour.step(state, dt)
        finally:
            state['_in_tour_step'] = False
    t_mono = time.monotonic()
    se2_st = state.get('se2')
    if se2_st is not None and state.get('wasd_mode') and not (tour is not None and not tour.done):
        dt_real = min(0.1, max(0.01, t_mono - state.get('_se2_last_t', t_mono - dt)))
        if t_mono - state.get('_se2_last_t', 0.0) > 0.015:
            state['_se2_last_t'] = t_mono
            step_se2(se2_st, (), dt_real, now=t_mono, held=_console.held())
        any_wasd = any(se2_st['keys'].get(k) for k in ('w', 's', 'a', 'd', 'q', 'e'))
        if state.get('_wasd_active') and not any_wasd:
            se2_st['v_x'] = se2_st['v_y'] = se2_st['omega_z'] = 0.0
            se2_st['setpoint'] = se2_st['actual'] = state['_last_sent_twist'] = (0.0, 0.0, 0.0)
            se2_st['gait'], state['_wasd_active'] = None, False
            body.send(twist=(0.0, 0.0, 0.0))
        else:
            if any_wasd:
                state['_wasd_active'] = True
            tw = (round(se2_st['v_x'], 2), round(se2_st['v_y'], 2), round(se2_st['omega_z'], 2))
            if state.get('_last_sent_twist') != tw:
                state['_last_sent_twist'] = tw
                body.send(twist=tw)
    return t_mono, se2_st


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--port', default=PORT, help='accepted with every page; no board here')
    parser.add_argument('--hz', type=float, default=FPS_CAP, help='frames per second, at most')
    parser.add_argument('--simulated', action='store_true', help='accepted for the view suite')
    parser.add_argument('--frames', type=int, default=0, help='stop after this many frames')
    parser.add_argument('--width', type=int, default=0, help='drawing width in cells, 0 fills')
    parser.add_argument('--height', type=int, default=0, help='drawing height in rows, 0 fills')
    args = parser.parse_args(argv)

    board_view = stage()
    terminal = board_view.is_terminal
    body = leaving = lit = None
    name = 'this process, dots'
    state = {'body': None, 'orbit': False, **viewpoint.HOME, 'side': 1.0, 'last_t': None,
             'called': 'strong', 'follow': gynoid.Follow(), 'recording': None, 'recorded': None,
             'playback': Playback(), 'shown': 'torque', 'skin': 'dressed',
             'scanner': VisionScanner(), 'now': None, 'wasd_mode': True, 'tour': None,
             'se2': se2.state(), 'keys': KEYS}

    def draw():
        t_mono, se2_st = _step_frame(state, body, 1.0 / max(1.0, args.hz))
        said = []
        body.latest(into=said)
        if state['recording'] is not None:
            state['recording'] += [row(s, state['yaw']) for s in said]
        state['playback'].push(said)
        now = state['now'] = state['playback'].at(time.perf_counter())
        _sync_actual(now, se2_st)
        pushed = released(state, t_mono)
        if pushed and now is not None:
            gym._clear_se2(state)
            body.send(push=tuple(pushed * a for a in along(state)), seconds=sprung.SHOVE_S)
            state['landed'] = (t_mono, pushed)
        if now is not None:
            viewpoint.orbited(state, now['t'])
        up = helped(state, state.pop('typed', ()), t_mono)
        width, height = size_of(board_view, args, up)
        if now is None:
            art = '\n'.join(' ' * width for _ in range(height))
        else:
            x, y, z = now['where']
            camera = state['follow']((x, z), now['velocity'], now['t'])
            inset_cells = docked_vision(state['scanner'], now, width, height, wasd_mode=state.get('wasd_mode', False))
            inset_cells.update(tpw_badge(state.get('wasd_mode', False), width, height, state=state))
            art = '\n'.join(gynoid.render(
                now['angles'], width, height, yaw=state['yaw'], pitch=state['pitch'], pan=state['pan'],
                zoom=state['zoom'], colour=terminal, travel=camera, lit=lit,
                root=((x - camera[0], y, z - camera[1]), quat(*now['turn'])),
                labels=labels(now, state['called'], state['shown']), props=now.get('props'),
                mark=marked(state, now, time.monotonic()),
                legend=legend(state['shown'], width) if state['called'] != 'none' else None,
                dressed=state['skin'] == 'dressed', see=_seen(state), inset=inset_cells,
                arrows=(now, state.get('se2'))))
        return frame_of(board_view, ORIGIN, TITLE, art, boxes(state, now, name), bar_for(state),
                        gauges=gauges(state), help=(GROUPS, up))

    def on_input(typed, wheel):
        for k in typed:
            if k in _screen.SCROLL_STEP:
                _screen.scroll_by(board_view, _screen.SCROLL_STEP[k])
        act_on(KEYS, typed, state, wheel)

    try:
        with boot('LINKING HUMANOID', board_view) as step:
            step(0.15, 'SHADER & RASTER')
            card = gpu.adapter()
            lit = gpu.LitRaster(found=card) if card is not None else None
            name = lit.name if lit is not None else 'this process, dots'
            say('ok', 'drawing', name)
            step(0.40, 'GYNOID MESHES')
            gynoid.body(), gynoid.body(dressed=False), gynoid.body(see='mechanism'), gynoid.body(see='actuators')
            step(0.70, 'MUJOCO & LISP')
            body = state['body'] = sprung.Running()
            say('ok', 'body', '%.1f kg, %d drives, MuJoCo at 1 kHz in its own process'
                % (figure.mass(), sum(len(s.actuators) for s in TYPES['gynoid'].body)))
            step(0.90, 'SETTLING STAND')
            for _ in range(100):
                first = body.latest()
                if first is not None:
                    state['playback'].push([first])
                    break
                time.sleep(0.01)
            step()
        leaving = run_view(board_view, terminal, 1.0 / max(1.0, args.hz),
                           args.frames, draw, on_input=on_input, quits=frozenset(),
                           **viewpoint.mouse(state, lambda: size_of(board_view, args)))
    finally:
        if body is not None:
            body.close()
        sys.stdout.write('\n')
        closing([('body', 'stopped, her process ended'),
                 ('boards', 'none - every drive was simulated')], terminal, 0)
    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
