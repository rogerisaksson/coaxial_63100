"""The HUMANOID page's keys: what each does to the view's state and asks of her body.

    KEYS = humanoid_keys.table(viewpoint.KEYS, CALLING, SHOWN)
    humanoid_keys.act_on(KEYS, typed, state, wheel)

The view (`terminal.views.show_humanoid`) holds the state and draws it; a key's handler takes
that state, moves it and sends her body (`machine.sprung`) what it asks. R's recordings and
their rows (HEADER, `row`) are here with the key that writes them.
"""
import json
import time

from machine import se2
from machine.figure import JOINTS, SEGMENTS, frames, quat
from terminal import keybindings
from terminal.ui import console, recorder
from terminal.views import gym, viewpoint

#: Space held charges a push from the view onto her: CHARGE_N_S a second held, CHARGE_MAX_N at
#: most, landing once the key's repeats stop - RELEASE_S after the last, FIRST_S after a lone
#: press (a terminal's first repeat comes 250-500 ms on) - along the view's line of sight.
CHARGE_N_S, CHARGE_MAX_N, RELEASE_S, FIRST_S = 120.0, 250.0, 0.25, 0.8

#: Where R's recordings go, a CSV from a press to the next: every state the page hears from her,
#: 10-20 a second of her time (HEADER); beside each, the keys typed and held since the row before
#: and the commands she heard since, (epoch, pass, t, command) each - from her epoch's lay on in
#: the first row, so `tools/sim/replay.py` runs it again pass for pass.
RECORDINGS = recorder.RECORDINGS
HEADER = (['t', 'stage', 'yaw', 'speed', 'phase', 'left_load', 'right_load',
           'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'] + list(JOINTS)
          + ['%s_%s' % (seg[0], axis) for seg in SEGMENTS for axis in 'xyz']
          + ['set_' + j for j in JOINTS] + ['watts', 'epoch', 'keys', 'commands'])

#: What C steps her through: dressed, her shell, her mechanism (`coaxial.graphics.mechanism`),
#: her drives alone - the stick figure.
SKINS = ('dressed', 'shell', 'mechanism', 'actuators')


def row(now, yaw, keys='', commands=()):
    """A recording's row (HEADER) at the page's `yaw`: `keys` typed and held, `commands` heard."""
    placed = frames(now['angles'], now['where'], quat(*now['turn']))
    asked = now.get('set', {})
    return ([round(now['t'], 4), now['stage'], yaw, now['speed'], now['phase']]
            + list(now['loads']) + list(now['where']) + list(now['turn'])
            + [now['angles'].get(j, 0.0) for j in JOINTS]
            + [v for seg in SEGMENTS for v in placed[seg[0]][0]]
            + [asked.get(j, float('nan')) for j in JOINTS] + [now.get('watts', float('nan')),
                                                              now.get('epoch', 0), keys,
                                                              json.dumps(plain(list(commands)))])


def plain(value):
    """`value` as JSON has it: tuples as lists, numbers as floats, bytes as hex."""
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, (bytes, bytearray)):
        return value.hex()
    if isinstance(value, (bool, str, int)) or value is None:
        return value
    return float(value)


def heard(state, entries):
    """Her heard commands into the page's `log` - her epoch's, from its lay - and, recording,
    into those the next row carries."""
    for entry in entries:
        if 'laid' in entry[3]:
            state['log'] = []
        state.setdefault('log', []).append(entry)
        if state.get('recording') is not None:
            state['pending'].append(entry)


def rows_of(state, said):
    """A row of each state `said`; the keys typed and held, and the commands heard, since the
    rows before on the first."""
    typed, state['keyed'] = state.get('keyed', []), []
    held = sorted(k for k in (console.held() or ()) if isinstance(k, str))
    keys = ' '.join(typed) + (' [%s]' % '+'.join(held) if held else '')
    out = []
    for k, now in enumerate(said):
        out.append(row(now, state['yaw'], keys if k == 0 else '', state['pending'] if k == 0 else ()))
        if k == 0:
            state['pending'] = []
    return out


def _recorded(state):
    """R: recording from now, her epoch's commands so far its first row's; again: written to
    RECORDINGS, its name on the page."""
    if state['recording'] is None:
        state['recording'], state['recorded'] = recorder.started('humanoid', HEADER), None
        state['pending'], state['keyed'] = list(state.get('log', ())), []
        return
    rec, state['recording'] = state['recording'], None
    state['recorded'] = recorder.saved(rec, RECORDINGS)


def _charged(state):
    """Space, each press and repeat: jump/recover in TPW mode; otherwise the push charging."""
    gym._cancel_tour(state)
    if state.get('wasd_mode'):
        state['charge'] = None
        rise = gym.contextual_rise(state.get('now'), state.get('scenario'))
        if rise:
            gym._clear_se2(state)
            state['body'].send(say=rise)
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
    if not charge or now - charge['last'] < (RELEASE_S if charge['repeats'] > 1 else FIRST_S):
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


CHORDS = {frozenset(tag): (tag, mv) for tag, mv in keybindings.CHORD_PAIRS}
_QE_CHORDS = {k: tuple(k) for tag, _ in keybindings.CHORD_PAIRS if set(tag) & {'q', 'e'} for k in (tag, tag[::-1])}


def _match_chord(keys_set, prefer=()):
    for pair, target in CHORDS.items():
        if pair.issubset(keys_set) and (not prefer or (pair & set(prefer))):
            return target[0], target[1], tuple(pair)
    for pair, target in CHORDS.items():
        if pair.issubset(keys_set):
            return target[0], target[1], tuple(pair)
    return None


def _se2_tokens(raw):
    items = [raw] if isinstance(raw, str) else ([k for k, v in raw.items() if v] if isinstance(raw, dict) else list(raw or ()))
    toks = set()
    for s in (it.strip() for it in items if isinstance(it, str)):
        low = s.lower()
        if s in ('W', 'A', 'S', 'D', 'Q', 'E') or (len(low) == 2 and s.isupper()) or low in ('shift', 'sprint'):
            toks.add('shift')
        toks.update(_QE_CHORDS.get(low) or se2._CHORD_EXPAND.get(low) or ((low,) if low in 'wsadqe' else ()))
    return toks


def _sync_qe_seen(se2_st, toks, prev_ls, now, held):
    ls = se2_st.setdefault('last_seen', {})
    if held is not None and (held or not toks):
        act = {k.lower() for k in held if isinstance(k, str)} | toks
        for k in ('q', 'e', 'shift'):
            if k in act and now is not None:
                ls[k] = float(now)
            else:
                ls.pop(k, None)
        return act
    if now is None:
        return toks
    for k in ('q', 'e', 'shift'):
        if k in prev_ls and k not in ls:
            ls[k] = prev_ls[k]
    for k1, k2 in (('q', 'e'), ('e', 'q')):
        if k1 in toks and k2 not in toks:
            ls.pop(k2, None)
    if toks and 'shift' not in toks and (toks & set('wasdqe')):
        ls.pop('shift', None)
    if toks & set('wsad'):
        for qk in ('q', 'e'):
            if qk in ls and (now - ls[qk]) <= se2.HOLD_WINDOW_S:
                ls[qk] = float(now)
    for k in toks & {'q', 'e', 'shift'}:
        ls[k] = float(now)
    return {k for k, ts in list(ls.items()) if (now - ts) <= se2.HOLD_WINDOW_S}


def step_se2(se2_st, raw=(), dt=0.05, now=None, held=None):
    """Step SE(2) controller with full Q/E pure yaw and circle-strafe support."""
    toks = _se2_tokens(raw)
    prev_ls = dict(se2_st.get('last_seen') or {})
    prev_act, prev_yaw = se2_st.get('actual', (0.0, 0.0, 0.0)), float(se2_st.get('yaw', 0.0))
    if held is None and now is not None and (toks & {'q', 'e'}):
        for k in ('w', 's', 'a', 'd'):
            if k in prev_ls and (now - prev_ls[k]) <= se2.HOLD_WINDOW_S:
                se2_st.setdefault('last_seen', {})[k] = float(now)
    se2.step_controller(se2_st, list(toks) if toks else raw, dt, now=now, hold_window=se2.HOLD_WINDOW_S, held=held)
    if 'q' in se2_st['keys'] and 'e' in se2_st['keys']:
        return se2_st
    act = _sync_qe_seen(se2_st, toks, prev_ls, now, held)
    q, e = 'q' in act and 'e' not in act, 'e' in act and 'q' not in act
    w, s, a, d = (bool(se2_st['keys'].get(k)) for k in ('w', 's', 'a', 'd'))
    shift = bool(se2_st['keys'].get('shift')) or ('shift' in act and (w or s or a or d or q or e))
    se2_st['keys'].update(q=q, e=e, shift=shift)
    if q or e:
        wz_set = (se2.YAW_SPRINT_RATE if shift else se2.YAW_RATE) * (1.0 if q else -1.0)
        se2_st['setpoint'] = (se2_st['setpoint'][0], se2_st['setpoint'][1], wz_set)
        se2_st['omega_z'] = wz_set
        dt_f = max(0.0, float(dt))
        wz_0 = 0.0 if wz_set * prev_act[2] < 0.0 else float(prev_act[2])
        wz_act = se2._approach(wz_0, wz_set, 0.75 if abs(wz_set) >= abs(wz_0) else se2.STRAFE_DECAY, dt_f)
        se2_st['actual'] = (se2_st['actual'][0], se2_st['actual'][1], wz_act)
        se2_st['yaw'] = se2.wrap_angle(prev_yaw + wz_set * dt_f)
        se2_st['v_wx'], se2_st['v_wy'] = se2.body_to_world(se2_st['v_x'], se2_st['v_y'], se2_st['yaw'])
        qk = 'q' if q else 'e'
        se2_st['gait'] = next((keybindings.CHORDS[(qk, k)] for k in ('w', 's', 'a', 'd') if se2_st['keys'].get(k)),
                              'turn_left' if q else 'turn_right')
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
    state.setdefault('keyed', []).extend(k for k in typed if isinstance(k, str))
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
        if key in keys:
            keys[key](state)
