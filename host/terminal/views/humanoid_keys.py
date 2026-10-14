"""The HUMANOID page's keys: what each does to the view's state and asks of her body.

    KEYS = humanoid_keys.table(viewpoint.KEYS, CALLING, SHOWN)
    humanoid_keys.act_on(KEYS, typed, state, wheel)

The view (`terminal.views.show_humanoid`) holds the state and draws it; a key's handler takes
that state, moves it and sends her body (`machine.sprung`) what it asks. R's recordings and
their rows (HEADER, `row`) are here with the key that writes them.
"""
import csv
import os
import time

from machine import se2
from machine.figure import JOINTS, SEGMENTS, frames, quat
from terminal import keybindings
from terminal.views import gym, viewpoint
from tools import REPO

#: Space held charges a push from the view onto her: CHARGE_N_S a second held, CHARGE_MAX_N at
#: most, landing once the key's repeats stop - RELEASE_S after the last, FIRST_S after a lone
#: press (a terminal's first repeat comes 250-500 ms on) - along the view's line of sight.
CHARGE_N_S, CHARGE_MAX_N, RELEASE_S, FIRST_S = 120.0, 250.0, 0.25, 0.8

#: Where R's recordings go, a CSV from a press to the next: every state the page hears from her,
#: 10-20 a second of her time (HEADER).
RECORDINGS = os.path.join(REPO, 'build', 'recordings')
HEADER = (['t', 'stage', 'yaw', 'speed', 'phase', 'left_load', 'right_load',
           'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'] + list(JOINTS)
          + ['%s_%s' % (seg[0], axis) for seg in SEGMENTS for axis in 'xyz']
          + ['set_' + j for j in JOINTS] + ['watts'])

#: What C steps her through: dressed, her shell, her mechanism (`coaxial.graphics.mechanism`),
#: her drives alone - the stick figure.
SKINS = ('dressed', 'shell', 'mechanism', 'actuators')


def row(now, yaw):
    """A recording's row (HEADER) at the page's `yaw`."""
    placed = frames(now['angles'], now['where'], quat(*now['turn']))
    asked = now.get('set', {})
    return ([round(now['t'], 4), now['stage'], yaw, now['speed'], now['phase']]
            + list(now['loads']) + list(now['where']) + list(now['turn'])
            + [now['angles'].get(j, 0.0) for j in JOINTS]
            + [v for seg in SEGMENTS for v in placed[seg[0]][0]]
            + [asked.get(j, float('nan')) for j in JOINTS] + [now.get('watts', float('nan'))])


def _recorded(state):
    """R: recording from now; again: written to RECORDINGS, its name on the page."""
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


def _charged(state):
    """Space, each press and repeat: jump/recover in TPW mode; otherwise the push charging."""
    gym._cancel_tour(state)
    if state.get('wasd_mode'):
        state['charge'] = None
        now_st = state.get('now') or {}
        doing = now_st.get('doing', '')
        scen = state.get('scenario') or now_st.get('scenario') or 'open'
        where = now_st.get('where') or (0.0, 0.90, 0.0)
        if where[1] > 1.12 or doing in ('seat_land', 'hop_up_chair'):
            gym._clear_se2(state)
            state['body'].send(say='hop_down_chair')
        elif doing in ('seated', 'sit_chair') or (
                doing in ('approach', 'lower', 'cushion', 'land') and scen == 'chair'):
            gym._clear_se2(state)
            state['body'].send(say='rise_chair')
        elif doing in ('bed_rest', 'lie_bed', 'sit_edge', 'recline') or (
                doing in ('approach', 'lower') and scen == 'bed'):
            gym._clear_se2(state)
            state['body'].send(say='rise_bed')
        elif doing in ('floor_rest', 'lie_floor', 'kneel') or (
                doing in ('approach', 'lower', 'recline') and scen in ('mattress', 'floor')):
            gym._clear_se2(state)
            state['body'].send(say='rise_floor')
        elif doing == 'down' or now_st.get('fallen'):
            gym._clear_se2(state)
            state['body'].send(say='#201')
        else:
            state['body'].send(say='hop')
        return
    now = time.monotonic()
    charge = state.get('charge')
    if not charge:
        charge = state['charge'] = {'began': now, 'last': now, 'repeats': 0}
    charge['last'] = now
    charge['repeats'] += 1


def charging(state, now):
    """The push charged so far, N: CHARGE_N_S a second since space was first pressed, to
    CHARGE_MAX_N; 0 with none charging or in TPW mode."""
    if state.get('wasd_mode'):
        return 0.0
    charge = state.get('charge')
    return min(CHARGE_MAX_N, CHARGE_N_S * (now - charge['began'])) if charge else 0.0


def released(state, now):
    """The push let go, N, once space's repeats have stopped (RELEASE_S, FIRST_S); 0 while it
    charges or none is."""
    if state.get('wasd_mode'):
        state['charge'] = None
        return 0.0
    charge = state.get('charge')
    if not charge:
        return 0.0
    if now - charge['last'] < (RELEASE_S if charge['repeats'] > 1 else FIRST_S):
        return 0.0
    n = min(CHARGE_MAX_N, CHARGE_N_S * (charge['last'] - charge['began'] + RELEASE_S / 2.0))
    state['charge'] = None
    return n


def _skinned(state):
    """The next of SKINS."""
    state['skin'] = SKINS[(SKINS.index(state['skin']) + 1) % len(SKINS)]


def _restarted(state):
    """A: restart her onto the floor and clear any active autopilot or SE(2) latch."""
    gym._cancel_tour(state)
    gym._clear_se2(state)
    state['body'].send(restart=True)


def table(view, calling, shown):
    """{key: what it does to the state}: `view`'s keys and the page's own; L and T step through
    `calling` and `shown`."""
    return dict(
        list(view.items())
        + [(k, _restarted) for k in 'aA']
        + [(k, lambda state: state.update(orbit=not state['orbit'])) for k in 'oO']
        + [(k, lambda state: state.update(called=calling[(calling.index(state['called']) + 1)
                                                         % len(calling)])) for k in 'lL']
        + [(k, _recorded) for k in 'rR']
        + [(k, lambda state: state.update(shown=shown[(shown.index(state['shown']) + 1)
                                                      % len(shown)])) for k in 'tT']
        + [(k, _skinned) for k in 'cC'] + [(' ', _charged)])


KEYLOG = os.path.join(REPO, 'build', 'keylog.txt')


def log_key(key, action=''):
    """Append a key typed on the tty to build/keylog.txt."""
    try:
        os.makedirs(os.path.dirname(KEYLOG), exist_ok=True)
        now = time.time()
        ts = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now)) + ('.%03d' % int((now % 1) * 1000))
        msg = '%s | TTY: %r' % (ts, key)
        if action:
            msg += ' -> %s' % action
        with open(KEYLOG, 'a', encoding='utf-8') as f:
            f.write(msg + '\n')
    except Exception:
        pass


def _action_of(key, wasd_mode=False):
    return keybindings.describe_action(key, wasd_mode=wasd_mode)


CHORDS = {
    frozenset(('w', 'a')): ('wa', 'turn_left'),
    frozenset(('w', 'd')): ('wd', 'turn_right'),
    frozenset(('s', 'd')): ('sd', 'turn_back_right'),
    frozenset(('s', 'a')): ('sa', 'turn_back_left'),
    frozenset(('w', 'q')): ('wq', 'turn_left'),
    frozenset(('w', 'e')): ('we', 'turn_right'),
    frozenset(('s', 'e')): ('se', 'turn_back_right'),
    frozenset(('s', 'q')): ('sq', 'turn_back_left'),
    frozenset(('a', 'e')): ('ae', 'strafe_left'),
    frozenset(('a', 'q')): ('aq', 'strafe_left'),
    frozenset(('d', 'q')): ('dq', 'strafe_right'),
    frozenset(('d', 'e')): ('de', 'strafe_right'),
}


_QE_CHORDS = {
    'wq': ('w', 'q'), 'qw': ('q', 'w'), 'we': ('w', 'e'), 'ew': ('e', 'w'),
    'sq': ('s', 'q'), 'qs': ('q', 's'), 'se': ('s', 'e'), 'es': ('e', 's'),
    'ae': ('a', 'e'), 'ea': ('e', 'a'), 'aq': ('a', 'q'), 'qa': ('q', 'a'),
    'dq': ('d', 'q'), 'qd': ('q', 'd'), 'de': ('d', 'e'), 'ed': ('e', 'd'),
}


def _match_chord(keys_set, prefer=()):
    for pair, target in CHORDS.items():
        if pair.issubset(keys_set) and (not prefer or (pair & set(prefer))):
            return target[0], target[1], tuple(pair)
    for pair, target in CHORDS.items():
        if pair.issubset(keys_set):
            return target[0], target[1], tuple(pair)
    return None


def step_se2(se2_st, raw=(), dt=0.05, now=None, held=None):
    """Step SE(2) controller with full Q/E pure yaw and circle-strafe support."""
    items = [raw] if isinstance(raw, str) else ([k for k, v in raw.items() if v] if isinstance(raw, dict) else list(raw or ()))
    toks = set()
    for it in items:
        if isinstance(it, str):
            s = it.strip()
            low = s.lower()
            if s in ('W', 'A', 'S', 'D', 'Q', 'E') or (len(low) == 2 and s.isupper()) or low in ('shift', 'sprint'):
                toks.add('shift')
            if low in _QE_CHORDS:
                toks.update(_QE_CHORDS[low])
            elif low in se2._CHORD_EXPAND:
                toks.update(se2._CHORD_EXPAND[low])
            elif low in ('w', 's', 'a', 'd', 'q', 'e'):
                toks.add(low)
    prev_ls = dict(se2_st.get('last_seen') or {})
    prev_act = se2_st.get('actual', (0.0, 0.0, 0.0))
    prev_yaw = float(se2_st.get('yaw', 0.0))
    if held is None and now is not None and (toks & {'q', 'e'}):
        for k in ('w', 's', 'a', 'd'):
            if k in prev_ls and (now - prev_ls[k]) <= se2.HOLD_WINDOW_S:
                se2_st.setdefault('last_seen', {})[k] = float(now)
    se2.step_controller(se2_st, list(toks) if toks else raw, dt, now=now, hold_window=se2.HOLD_WINDOW_S, held=held)
    if 'q' not in se2_st['keys'] or 'e' not in se2_st['keys']:
        ls = se2_st.setdefault('last_seen', {})
        if held is not None and (held or not toks):
            act = {k.lower() for k in held if isinstance(k, str)} | toks
            for k in ('q', 'e', 'shift'):
                if k in act and now is not None:
                    ls[k] = float(now)
                else:
                    ls.pop(k, None)
        elif now is not None:
            for k in ('q', 'e', 'shift'):
                if k in prev_ls and k not in ls:
                    ls[k] = prev_ls[k]
            for k1, k2 in (('q', 'e'), ('e', 'q')):
                if k1 in toks and k2 not in toks:
                    ls.pop(k2, None)
            if toks and 'shift' not in toks and (toks & {'w', 'a', 's', 'd', 'q', 'e'}):
                ls.pop('shift', None)
            if toks & {'w', 's', 'a', 'd'}:
                for qk in ('q', 'e'):
                    if qk in ls and (now - ls[qk]) <= se2.HOLD_WINDOW_S:
                        ls[qk] = float(now)
            for k in toks & {'q', 'e', 'shift'}:
                ls[k] = float(now)
            act = {k for k, ts in list(ls.items()) if (now - ts) <= se2.HOLD_WINDOW_S}
        else:
            act = toks
        q, e = 'q' in act and 'e' not in act, 'e' in act and 'q' not in act
        w, s, a, d = (bool(se2_st['keys'].get(k)) for k in ('w', 's', 'a', 'd'))
        shift = bool(se2_st['keys'].get('shift')) or ('shift' in act and (w or s or a or d or q or e))
        se2_st['keys'].update(q=q, e=e, shift=shift)
        if q or e:
            rz = se2.YAW_SPRINT_RATE if shift else se2.YAW_RATE
            wz_set = rz if q else -rz
            vx_set, vy_set, _ = se2_st['setpoint']
            se2_st['setpoint'] = (vx_set, vy_set, wz_set)
            se2_st['omega_z'] = wz_set
            dt_f = max(0.0, float(dt))
            wz_0 = 0.0 if wz_set * prev_act[2] < 0.0 else float(prev_act[2])
            wz_act = se2._approach(wz_0, wz_set, 0.75 if abs(wz_set) >= abs(wz_0) else se2.STRAFE_DECAY, dt_f)
            se2_st['actual'] = (se2_st['actual'][0], se2_st['actual'][1], wz_act)
            se2_st['yaw'] = se2.wrap_angle(prev_yaw + wz_set * dt_f)
            se2_st['v_wx'], se2_st['v_wy'] = se2.body_to_world(se2_st['v_x'], se2_st['v_y'], se2_st['yaw'])
            se2_st['gait'] = (('turn_left' if (a or q) else 'turn_right') if w else
                              (('turn_back_left' if (a or q) else 'turn_back_right') if s else
                               (('strafe_left' if a else 'strafe_right') if (a or d) else
                                ('turn_left' if q else 'turn_right'))))
    return se2_st


def _check_wasd_chords(typed, state):
    if not (state.get('wasd_mode') and typed):
        return typed
    from terminal.ui import console as _console
    lowered = [k.lower() if isinstance(k, str) else k for k in typed]
    t_now = time.monotonic()
    se2_st = state.setdefault('se2', se2.state())
    move_in = {k for k in lowered if k in ('w', 's', 'a', 'd', 'q', 'e')}
    if len(move_in) >= 2:
        for k in list(se2_st.get('last_seen', ())):
            if k in ('w', 's', 'a', 'd', 'q', 'e') and k not in move_in:
                se2_st['last_seen'].pop(k, None)
    last_dt = min(0.1, max(0.01, t_now - state.get('_se2_last_t', t_now - 0.05)))
    state['_se2_last_t'] = t_now
    step_se2(se2_st, typed, last_dt, now=t_now, held=_console.held())

    active = {k for k in ('w', 's', 'a', 'd', 'q', 'e') if se2_st['keys'].get(k)}
    if active:
        gym._cancel_tour(state)
        state['_wasd_active'] = True
    found = _match_chord(active, move_in)
    if found:
        tag, mv, _ = found
        state['body'].send(say=mv)
        log_key(tag, _action_of(tag, wasd_mode=True))
        state['_last_wasd'] = (tag, t_now)
        return [k for k in typed if (k.lower() if isinstance(k, str) else k) not in active]

    for k in lowered:
        if k in active:
            state['_last_wasd'] = (k, t_now)
    return [k for k in typed if (k.lower() if isinstance(k, str) else k) not in ('w', 's', 'a', 'd', 'q', 'e')
            or (k.lower() if isinstance(k, str) else k) in active]


def act_on(keys, typed, state, wheel=0.0):
    """`typed` and the wheel on the view's `state`, by `keys` (`table`)."""
    if wheel:
        viewpoint.zoomed(state, max(0.5, 1.0 + wheel))
    state.setdefault('typed', []).extend(typed)
    typed = _check_wasd_chords(typed, state)
    t_now = time.monotonic()
    wasd_on = bool(state.get('wasd_mode'))
    filtered = []
    for key in typed:
        last_k, last_t = state.get('_last_action_key', (None, 0.0))
        if wasd_on and key == ' ' and last_k == ' ' and (t_now - last_t) < 0.35:
            continue
        if key == last_k and key != ' ' and (t_now - last_t) < 0.12:
            if wasd_on and key in ('w', 'W') and (
                    key == 'W' or state.get('se2', {}).get('keys', {}).get('shift')):
                state['body'].send(say='jog')
            continue
        state['_last_action_key'] = (key, t_now)
        filtered.append(key)
    for key in filtered:
        log_key(key, _action_of(key, wasd_mode=wasd_on))
        if key in keys:
            keys[key](state)


