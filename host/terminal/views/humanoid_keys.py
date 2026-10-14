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
from terminal.views import viewpoint
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
    """Space, each press and repeat: jump in TPW mode; otherwise the push charging."""
    if state.get('wasd_mode'):
        state['charge'] = None
        now_st = state.get('now') or {}
        doing = now_st.get('doing', '')
        where = now_st.get('where') or (0.0, 0.90, 0.0)
        if where[1] > 1.12:
            state['body'].send(say='hop_down_chair')
        elif doing in ('seated', 'sit_chair'):
            state['body'].send(say='rise_chair')
        elif doing in ('bed_rest', 'lie_bed'):
            state['body'].send(say='rise_bed')
        elif doing in ('floor_rest', 'lie_floor'):
            state['body'].send(say='rise_floor')
        elif doing == 'down':
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


def table(view, calling, shown):
    """{key: what it does to the state}: `view`'s keys and the page's own; L and T step through
    `calling` and `shown`."""
    return dict(
        list(view.items())
        + [(k, lambda state: state['body'].send(restart=True)) for k in 'aA']
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
}


def _match_chord(keys_set):
    for pair, target in CHORDS.items():
        if pair.issubset(keys_set):
            return target[0], target[1], tuple(pair)
    return None


def _check_wasd_chords(typed, state):
    if not (state.get('wasd_mode') and typed):
        return typed
    from terminal.ui import console as _console
    lowered = [k.lower() if isinstance(k, str) else k for k in typed]
    t_now = time.monotonic()
    se2_st = state.setdefault('se2', se2.state())
    last_dt = min(0.1, max(0.01, t_now - state.get('_se2_last_t', t_now - 0.05)))
    state['_se2_last_t'] = t_now
    held_now = _console.held()
    se2.step_controller(se2_st, typed, last_dt, now=t_now, hold_window=se2.HOLD_WINDOW_S, held=held_now)

    active = {k for k in ('w', 's', 'a', 'd') if se2_st['keys'].get(k)}
    if active:
        state['_wasd_active'] = True
    found = _match_chord(active)
    if found:
        tag, mv, consumed = found
        state['body'].send(say=mv)
        log_key(tag, _action_of(tag, wasd_mode=True))
        state['_last_wasd'] = (tag, t_now)
        return [k for k in typed if (k.lower() if isinstance(k, str) else k) not in consumed]

    for k in lowered:
        if k in ('w', 'a', 's', 'd'):
            state['_last_wasd'] = (k, t_now)
    return typed


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


