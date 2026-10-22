#!/usr/bin/env python3
"""A slender gynoid on her latent loop under gravity: a body with mass, each joint a drive."""
import argparse
import math
import os
import sys
import time

from rich.text import Text

from coaxial.comm.session import Origin
from coaxial.draw import cross_section, gauges as _gauges
from coaxial.graphics import arrows, gynoid, horizon
from coaxial.graphics.callouts import packed
from machine import figure, se2, sprung
from machine.figure import JOINTS, quat
from machine.routines import TYPES
from terminal import boards
from terminal.loader import answer
from terminal.views import gym, viewpoint
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


def meter(value, marks):
    lo, hi, word = marks[0][0], marks[-1][0], marks[1][1]
    cell = lambda v: max(0, min(METER - 1, round((v - lo) / (hi - lo) * (METER - 1))))
    start, dot = max(0, min(METER - len(word), cell(marks[1][0]) - len(word) // 2)), cell(value)
    out, lit = Text(marks[0][1] + ' ', style='bar.dim'), start <= dot < start + len(word)
    for i in range(METER):
        in_w = start <= i < start + len(word)
        out.append(word[i - start] if in_w else ('●' if i == dot else '─'),
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
    return '▰' * n + '▱' * (4 - n)


_STUNT_BAL = dict.fromkeys(
    frozenset(sprung.STUNT_PHASES) | {'sit_chair', 'lie_bed', 'lie_floor', 'lean', 'parry_shove', 'catch', 'restore'},
    (0.95, 0.95))
_STUNT_BAL['controlled_fall'] = (0.55, 0.55)


def _live_bal_stb(now):
    turn = now.get('turn', (1.0, 0.0, 0.0, 0.0))
    qx, qz = float(turn[1]), float(turn[3])
    bal = max(0.0, min(1.0, (1.0 - 2.0 * (qx * qx + qz * qz) - 0.5) / 0.5))
    loads = now.get('loads', (160.0, 160.0))
    load_norm = min(1.0, (float(loads[0]) + float(loads[1])) / 240.0)
    where_y = float((now.get('where') or (0.0, 0.90, 0.0))[1])
    return bal, max(0.0, min(1.0, 0.5 * load_norm + 0.5 * min(1.0, where_y / 0.85)))


def telemetry_core(now):
    if not now:
        return 1.0, 1.0, 1.0, 'NOMINAL'
    watts = float(now.get('watts', 45.0))
    thm = max(0.0, min(1.0, 1.0 - max(0.0, watts - 40.0) / 1160.0))
    temps = now.get('temp')
    thm = min(thm, max(0.0, min(1.0, (125.0 - max(float(v) for v in temps.values())) / 100.0))) if temps else thm
    if now.get('fallen'):
        return 0.0, thm, 0.0, 'CRITICAL'
    doing = str(now.get('doing', 'rest'))
    bal, stb = _STUNT_BAL[doing] if doing in _STUNT_BAL else _live_bal_stb(now)
    worst = min(bal, thm, stb)
    return bal, thm, stb, ('CRITICAL' if worst < 0.35 else ('ALERT' if worst < 0.65 else 'NOMINAL'))


PROMPT_INK = (255, 196, 80)


def badge(width, height, state):
    """The viewport's corners: the tour's step and LLM command feed at its top, her balance
    strip under it, the interactive LLM prompt bar and what Space and F offer at its foot
    (`gym.prompt`)."""
    if width < 30 or height < 6:
        return {}
    out, ink = {}, packed((120, 220, 255), (10, 22, 35))
    if gym.touring(state):
        tr = state['tour']
        th, cm = getattr(tr, 'last_thought', ''), getattr(tr, 'last_cmd', '')
        line0 = (' [TOUR] %s  LLM ▸ %s (%s) ' % (tr.current_label(), th, cm)) if th else (' [TOUR] %s ' % tr.current_label())
        out.update({(0, 1 + i): (ord(ch), ink) for i, ch in enumerate(line0[:width - 2])})
    elif state.get('llm_last'):
        th, cm = state['llm_last']
        out.update({(0, 1 + i): (ord(ch), ink) for i, ch in enumerate((' [LLM] %s ▸ %s ' % (th, cm))[:width - 2])})
    lp = state.get('llm_prompt')
    if lp and lp.get('open'):
        l_ink = packed((120, 255, 180), (10, 22, 35))
        p_txt = (' LLM > %s_ ' % lp.get('text', ''))[:width - 2]
        out.update({(height - 3, 1 + i): (ord(ch), l_ink) for i, ch in enumerate(p_txt)})
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
RAD_S = 4.0


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
    """(width, height) of her drawing: the room beside the column, the rows the band, the bar, `up`
    of the help and the `below` her chat's drawer takes leave."""
    size = console.size if console.is_terminal else None
    width = args.width or max(24, room(size.width if size else 110))
    bar = footer(PAD['bar']).row_count + help_rows(PAD['groups'], up)
    return width, args.height or max(12, (size.height if size else 44) - 5 - bar - below)


def _sync_actual(now, keys):
    if not now:
        return (0.0, 0.0, 0.0)
    hd = arrows.pelvis_yaw(now)
    return arrows._actual_twist(now, keys, math.sin(hd), math.cos(hd), str(now.get('doing', 'rest')), True)


_ANSI = {}


def _ansi(raw):
    got = _ANSI.get(raw)
    if got is None:
        if len(_ANSI) > 512:
            _ANSI.clear()
        _ANSI[raw] = got = Text.from_ansi(raw)
    return got


def _joint_row(now, joint, label, shown):
    get = lambda k, d=0.0: float((now.get(k) or {}).get(joint, d)) if now else d
    deg, tau, pwr, deg_c, pk = get('angles'), abs(get('torque')), get('power'), get('temp', 25.0), max(1.0, get('peak', 100.0))
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


def _bus_row(now, name, joints, alarms):
    from machine import drives
    get = lambda k, j, d=0.0: float((now.get(k) or {}).get(j, d)) if now else d
    nodes, pk_share, max_c = ['\x1b[38;5;66m├'], 0.0, 25.0
    for u in range(7):
        if u >= len(joints):
            nodes.append('\x1b[38;5;66m─')
            continue
        j = joints[u]
        pas, deg_c = drives.passive(j), get('temp', j, 25.0)
        max_c = max(max_c, deg_c)
        if pas:
            nodes.append('\x1b[38;5;66m▱')
        elif j in alarms:
            nodes.append('\x1b[38;5;196m▰')
        else:
            sh = min(1.0, abs(get('torque', j)) / max(1.0, get('peak', j, float(drives.peak(j)))))
            pk_share = max(pk_share, sh)
            c = 208 if (deg_c > 85.0 or sh > 0.85) else (214 if sh > 0.35 else 44)
            nodes.append(f'\x1b[38;5;{c}m▰')
    strip = ''.join(nodes) + '\x1b[0m'
    cls = cross_section.SOA_WARN if max_c > 85.0 else (cross_section.WATTS if pk_share > 0.5 else cross_section.SOA_OK)
    bar = _gauges.gauge(pk_share, 6, cls=cls)
    return (name.replace('_', ' ').rjust(10), _ansi('%s %d/%d %s %3.0f°C' % (strip, len(joints), len(joints), bar, max_c)))


def _tier_row(state, now, emu):
    """The BUSES box's tier row: where her boards run - her own process's SIL, or a limb on Renode
    -, her process paced to the wall (rt) or flat out (max), and how many times real time it
    could run (`headroom`)."""
    where = 'renode %s' % emu.limb if emu else 'in-process sil'
    return '%s  %s %.1fx' % (where, 'max' if state.get('max') else 'rt', float((now or {}).get('headroom', 1.0)))


def boxes(state, now, name):
    from machine import edge
    z0, keys, shown = (0.0, 0.0, 0.0), state.get('pad'), state.get('shown', 'torque')
    tw_act = _sync_actual(now, keys)
    raw_set = (keys.get('setpoint') or z0) if keys is not None else z0
    tw_now, tw_nom = (now.get('twist', z0), se2.NOMINAL_TWIST.get(now.get('doing', ''), z0)) if now else (z0, z0)
    tw_set = next((t for t in (raw_set, tw_now, tw_nom) if any(abs(v) > 1e-4 for v in t)), z0)
    bal, _thm, stb, tag = telemetry_core(now)
    soa = lambda v: cross_section.SOA_TRIP if v < 0.35 else (cross_section.SOA_WARN if v < 0.65 else cross_section.SOA_OK)
    core_row = _ansi('%s %3.0f%% stb %s %3.0f%%' % (
        _gauges.gauge(bal, 4, cls=soa(bal)), bal * 100.0, _gauges.gauge(stb, 4, cls=soa(stb)), stb * 100.0))
    spd_act, spd_set = tw_act[0], float(tw_set[0])
    s_norm = lambda v: 0.5 + 0.5 * max(-1.0, min(1.0, v / se2.RUN_SPEED))
    spd_bar = _gauges.gauge(s_norm(spd_act), 8, centre=0.5, marks=((s_norm(spd_set), _gauges.MARK),))
    loads, watts = (now['loads'], float(now['watts'])) if now else ((0.0, 0.0), 0.0)
    soles_row = _ansi('%3.0fN %s %s %3.0fN' % (
        loads[0], _gauges.gauge(min(1.0, loads[0] / 350.0), 5), _gauges.gauge(min(1.0, loads[1] / 350.0), 5), loads[1]))
    max_c = max((float(v) for v in now['temp'].values()), default=25.0) if (now and now.get('temp')) else 25.0
    rec = (recorder.badge(state['recording']) + ' - R saves' if state['recording'] is not None
           else os.path.basename(state['recorded'])[:25] if state['recorded'] else 'R starts')
    out = gym.boxes(now, tour=state.get('tour')) + [hud('BODY', [
        ('   control', '%s  [%s]' % ('tour' if gym.touring(state) else 'keys', tag)),
        ('    status', gym.status(now)[:25]), ('   balance', core_row),
        ('     speed', _ansi('%+5.2f m/s %s %+5.2f' % (spd_act, spd_bar, spd_set))),
        ('     twist', 'vx%+5.2f vy%+5.2f wz%+5.2f' % tw_act),
        ('    rhythm', _ansi('%4.2f Hz  phase   %s' % (float(now['cadence']) if now else 0.0, _gauges.gauge(float(now['phase']) if now else 0.0, 8)))),
        ('     soles', soles_row), ('     power', _ansi('%4.0f W  %s  %4.0f °C' % (watts, _screen.gauge(watts / 800.0, 8), max_c))),
        ('   physics', 'x%.1f real time  T:%s' % (now['ratio'], shown) if now else '-'),
        (' autopilot', state['tour'].current_label()[:25] if state.get('tour') and not state['tour'].done else 'manual'),
        ('    record', rec), ('   horizon', '%s  Z cycles' % (state.get('horizon') or 'off')),
        ('  drawn by', name[:25])])]
    w_now = (now.get('wear') if now else None) or {}
    al = w_now.get('alarms') or ()
    alarms = {str(a[0]).split('.')[0] for a in al}
    b_now = (now.get('buses') if now else None) or {}
    crc_err = sum(int(v[1]) for v in b_now.values() if len(v) > 1)
    emu = state.get('emu')
    out.append(hud('BUSES  10M RS485', [_bus_row(now, b, jnts, alarms) for b, jnts in edge.LIMBS] + [
        ('  modbus', emu.badge(crc_err) if emu else ('5x10M 27/27 sil %4d crc' % crc_err)),
        ('    tier', _tier_row(state, now, emu))]))
    for sub in TYPES['gynoid'].body:
        out.append(hud(sub.name.replace('_', ' ').upper(), [
            _joint_row(now, j, j.split('_', 1)[-1] if sub.name != 'axis' else j, shown) for j in sub.actuators]))
    ld = w_now.get('load') or {}
    lh = float((ld.get('left_hand') or (0.0,))[0])
    rh = float((ld.get('right_hand') or (0.0,))[0])
    tr = float((ld.get('torso') or (0.0,))[0])
    out.append(hud('SERVICE', [
        ('     hands', ('%+.2f kg L  %+.2f kg R' % (lh, rh))[:25]),
        ('     torso', ('%+.2f kg  due %.0fh' % (tr, float(w_now.get('left_h', 500.0))))[:25]),
        ('    alarms', ('%d: %s' % (len(al), al[0][0]) if al else 'none')[:25])]))
    scanner = state.get('scanner')
    if scanner is not None and now is not None:
        det = scanner.scan(now, viewpoint.looks(state))
        out.append(hud('VISION (EYES)', [
            ('     model', scanner.model_name()[:25]), ('  detected', (det[0]['label'] if det else 'none')[:25]),
            ('    action', (det[0]['affordances'][0][1] if (det and det[0]['affordances']) else '-')[:25])]))
    llm_last = state.get('llm_last')
    th_str = llm_last[0] if llm_last else (state['tour'].current_label() if gym.touring(state) else 'idle')
    cm_str = llm_last[1] if llm_last else ('tour' if gym.touring(state) else '-')
    out.append(hud('AGENT / LLM', [
        ('    prompt', (state.get('llm_prompt_last') or '/ ask LLM  H tour')[:25]),
        ('   thought', th_str[:25]),
        ('   command', cm_str[:25])] + [
        ('      feed', cm[:25]) for _th, cm in (state.get('llm_feed') or ())[-2:]]))
    return out


def _seen(state):
    return state['skin'] if state['skin'] in ('mechanism', 'actuators') else None


#: Her speed over the floor past which she goes and the chase camera follows, m/s.
GOING_M_S = 0.1


def scripted(state, keys):
    """`--keys`: the frame's key typed as the console would, a key a frame from the first, '.'
    none - a page rendered under a key (tools/render/page.py)."""
    k = state.get('frame', 0)
    state['frame'] = k + 1
    if k < len(keys) and keys[k] != '.':
        bindings.act_on(PAD, [keys[k]], state)


def _step_frame(state, dt):
    """A frame's commands: the tour's due, what her keys hold (`humanoid_keys.frame`); the chase
    camera eased behind her."""
    if gym.touring(state):
        state['tour'].step(state, dt)
    t_mono, now = time.monotonic(), state.get('now')
    heading = float(now.get('heading') or 0.0) if now else 0.0
    frame(state, t_mono, heading)
    going = bool(now) and (math.hypot(*now['velocity']) > GOING_M_S or now.get('stage') == 'walk')
    viewpoint.chased(state, heading, going, t_mono)
    return t_mono


def _draw_viewport(state, now, width, height, terminal):
    """This frame's drawing of her - asked the frame before - and hers at `now` asked for the next
    (`state['ahead']`, a frame ahead in its own process)."""
    if now is None:
        return '\n'.join(' ' * width for _ in range(height))
    x, y, z = now['where']
    camera, offer = state['follow']((x, z), now['velocity'], now['t']), gym.offered(state)
    docked = docked_vision(state['scanner'], now, width, height, offer)
    inset = {**gym.ballooned(state, offer, width, height, camera, docked), **docked, **badge(width, height, state),
             **recorder.corner(state.get('recording'), width)}
    return '\n'.join(state['ahead'].drawn(
        now['angles'], width, height, yaw=state['yaw'], pitch=state['pitch'], pan=viewpoint.panned(state),
        zoom=state['zoom'], colour=terminal, travel=camera, lit=True,
        root=((x - camera[0], y, z - camera[1]), quat(*now['turn'])),
        labels=labels(now, state['called'], state['shown']), props=now.get('props'), dents=now.get('dents'),
        mark=marked(state, now, time.monotonic()),
        legend=legend(state['shown'], width) if state['called'] != 'none' else None,
        dressed=state['skin'] == 'dressed', see=_seen(state), inset=inset, arrows=(now, state.get('pad')),
        horizon=horizon.segments(now, now.get('props') or (), state.get('horizon') or 'off') or None))


def _boot_humanoid(_board_view, state):
    return sprung.boot_humanoid(state, boot, say, Ahead, boards.EmulatedLimb)


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
    parser.add_argument('--prompt', default='', help='send an initial natural-language LLM prompt to the gynoid')
    args, board_view = parser.parse_args(argv), stage()
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
        _sync_actual(now, state['pad'])
        if now is not None:
            viewpoint.orbited(state, now['t'])
        up, below = helped(t_mono), gym.drawer(state, t_mono)
        # Drawn a frame ahead: this frame shows the state asked the frame before, its boxes too.
        shown, state['drawn'] = state.get('drawn') or now, now
        w_vp, h_vp = size_of(board_view, args, up, below)
        art = _draw_viewport(state, now, w_vp, h_vp, terminal)
        return frame_of(board_view, state['origin'], TITLE, art, boxes(state, shown, name), PAD,
                        under=chat_box(state, w_vp) if below else None, under_rows=below,
                        gauges=gauges(state))

    try:
        body, name = _boot_humanoid(board_view, state)
        if args.tour:
            gym.toggle_tour(state)
        if args.prompt:
            gym.submit_prompt(state, args.prompt)
        wheeled = lambda _keys, wheel: viewpoint.zoomed(state, max(0.5, 1.0 + wheel)) if wheel else None
        leaving = run_view(board_view, terminal, 1.0 / max(1.0, args.hz), args.frames, draw, on_input=wheeled,
                           pad=PAD, state=state, **viewpoint.mouse(state, lambda: size_of(
                               board_view, args, below=state['drawer']['rows']), clicked))
    finally:
        if state.get('emu') is not None:
            state['emu'].close()
        if body is not None:
            body.close()
        if state.get('ahead') is not None:
            state['ahead'].close()
        sys.stdout.write('\n')
        emu = state.get('emu')
        closing([('body', 'stopped, her process ended'),
                 ('boards', emu.closed_note() if emu else 'none - every drive was simulated')], terminal, 0)
    return answer(leaving)


if __name__ == '__main__':
    sys.exit(main())

