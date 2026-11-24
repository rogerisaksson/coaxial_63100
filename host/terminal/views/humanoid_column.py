"""The HUMANOID page's column: the robot's body, buses, drives, service, eyes and agent.

Each a box of the robot's state as its drawing shows it, and the balance it reads (`telemetry_core`).
"""
import math
import os

from rich.text import Text

from coaxial.draw import cross_section, gauges as _gauges
from coaxial.graphics import arrows
from machine.law import se2
from machine.process import sprung
from machine.rig.routines import TYPES
from terminal.views import gym
from terminal.ui import recorder, screen as _screen
from terminal.ui.stage import hud

#: A drive's power drawn to its peak torque at this speed, rad/s.
RAD_S = 4.0

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


def actual(now, keys):
    """The twist the robot makes now, (vx, vy, wz) in its own frame, as its keys `keys` read it."""
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
        tail = '%+4.0fW %3.0f\u00b0C' % (p_cl, deg_c)
    elif shown == 'temp':
        bar = _gauges.gauge(_gauges.temp_share(deg_c), 6, cls=_gauges.thermometer_class(deg_c))
        tail = '%3.0f\u00b0C %+4.0fW' % (deg_c, p_cl)
    else:
        bar, tail = _screen.gauge(min(1.0, tau / pk), 6), '%3.0fNm %3.0f\u00b0C' % (min(999.0, tau), deg_c)
    return (label.rjust(10), _ansi('%+5.0f\u00b0 %s %s' % (deg, bar, tail)))


def _bus_row(now, name, joints):
    """A bus's boards, each by its drive's load and heat: the robot's wear's alarms are SERVICE's - red
    here, a seat's and a fall's misread drift was taken for its buses' errors."""
    from machine.body import drives
    get = lambda k, j, d=0.0: float((now.get(k) or {}).get(j, d)) if now else d
    nodes, pk_share, max_c = ['\x1b[38;5;66m\u251c'], 0.0, 25.0
    for u in range(7):
        if u >= len(joints):
            nodes.append('\x1b[38;5;66m\u2500')
            continue
        j = joints[u]
        pas, deg_c = drives.passive(j), get('temp', j, 25.0)
        max_c = max(max_c, deg_c)
        if pas:
            nodes.append('\x1b[38;5;66m\u25b1')
        else:
            sh = min(1.0, abs(get('torque', j)) / max(1.0, get('peak', j, float(drives.peak(j)))))
            pk_share = max(pk_share, sh)
            c = 208 if (deg_c > 85.0 or sh > 0.85) else (214 if sh > 0.35 else 44)
            nodes.append(f'\x1b[38;5;{c}m\u25b0')
    strip = ''.join(nodes) + '\x1b[0m'
    cls = cross_section.SOA_WARN if max_c > 85.0 else (cross_section.WATTS if pk_share > 0.5 else cross_section.SOA_OK)
    bar = _gauges.gauge(pk_share, 6, cls=cls)
    return (name.replace('_', ' ').rjust(10), _ansi('%s %d/%d %s %3.0f\u00b0C' % (strip, len(joints), len(joints), bar, max_c)))


def _tier_row(state, now, emu):
    """The BUSES box's tier row: where the robot's boards run - its own process's SIL, or a limb on
    Renode -, its process paced to the wall (rt) or flat out (max), and how many times real time it
    could run (`headroom`)."""
    where = 'renode %s' % emu.limb if emu else 'in-process sil'
    return '%s  %s %.1fx' % (where, 'max' if state.get('max') else 'rt', float((now or {}).get('headroom', 1.0)))


def boxes(state, now, name):
    from machine.senses import edge
    z0, keys, shown = (0.0, 0.0, 0.0), state.get('pad'), state.get('shown', 'torque')
    tw_act = actual(now, keys)
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
        ('    status', ((now or {}).get('shutdown') or gym.status(now))[:25]), ('   balance', core_row),
        ('     speed', _ansi('%+5.2f m/s %s %+5.2f' % (spd_act, spd_bar, spd_set))),
        ('     twist', 'vx%+5.2f vy%+5.2f wz%+5.2f' % tw_act),
        ('    rhythm', _ansi('%4.2f Hz  phase   %s' % (float(now['cadence']) if now else 0.0, _gauges.gauge(float(now['phase']) if now else 0.0, 8)))),
        ('     soles', soles_row), ('     power', _ansi('%4.0f W  %s  %4.0f \u00b0C' % (watts, _screen.gauge(watts / 800.0, 8), max_c))),
        ('   physics', 'x%.1f real time  T:%s' % (now['ratio'], shown) if now else '-'),
        (' autopilot', state['tour'].current_label()[:25] if state.get('tour') and not state['tour'].done else 'manual'),
        ('    record', rec), ('   horizon', '%s  Z cycles' % (state.get('horizon') or 'off')),
        ('  drawn by', name[:25])])]
    w_now = (now.get('wear') if now else None) or {}
    al = w_now.get('alarms') or ()
    b_now = (now.get('buses') if now else None) or {}
    crc_err = sum(int(v[1]) for v in b_now.values() if len(v) > 1)
    emu = state.get('emu')
    out.append(hud('BUSES  10M RS485', [_bus_row(now, b, jnts) for b, jnts in edge.LIMBS] + [
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
        ('      wear', ('%d drift: %s' % (len(al), al[0][0]) if al else 'none')[:25])]))
    scanner = state.get('scanner')
    if scanner is not None and now is not None:
        det = scanner.seen(now)
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
