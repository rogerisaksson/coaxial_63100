"""The HUMANOID page's keys: what each does to the view's state and asks of her body.

    KEYS = humanoid_keys.table(viewpoint.KEYS, CALLING, SHOWN)
    humanoid_keys.act_on(KEYS, typed, state, wheel)
    humanoid_keys.steered(state, now, heading)     # each frame: the keys held as her twist

The view (`terminal.views.show_humanoid`) holds the state and draws it; a key's handler takes
that state, moves it and sends her body (`machine.sprung`) what it asks: a twist for the keys that
move her, a verb for the rest - what an agent sends. R's recordings and their rows (HEADER,
`row`) are here with the key that writes them.
"""
import json
import math
import time

from machine import se2
from machine.figure import JOINTS, SEGMENTS, frames, quat
from terminal.ui import console, recorder
from terminal.views import gym, viewpoint

#: P held charges a push from the view onto her: CHARGE_N_S a second held, CHARGE_MAX_N at most,
#: landing once the key's repeats stop - RELEASE_S after the last, FIRST_S after a lone press (a
#: terminal's first repeat comes 250-500 ms on) - along the view's line of sight.
CHARGE_N_S, CHARGE_MAX_N, RELEASE_S, FIRST_S = 120.0, 250.0, 0.25, 0.8

#: W A S D a way from the view - W where it looks -: her heading turned to it, TURN_K rad/s a
#: radian off, to se2.YAW_RATE in steps of TWIST_STEP; walked at se2.WALK_SPEED within a quarter
#: turn of it, se2.RUN_SPEED on Shift within SPRINT_RAD. Q E a sidestep at se2.STRAFE_SPEED, her
#: facing kept. Each twist one the bench's plays hold (`tools/sim/scenarios.py`, KEYS).
TURN_K, SPRINT_RAD, TWIST_STEP = 1.5, 0.2, 0.05
MOVES = frozenset('wasdqeWASDQE')
#: A key typed counts held this long, s, where the console cannot tell (a terminal's first
#: repeat comes 250-500 ms on).
HOLD_WINDOW_S = 0.65

#: Where R's recordings go, a CSV from a press to the next: every state the page hears from her,
#: 10-20 a second of her time (HEADER); beside each, the keys typed and held since the row before
#: and the commands she heard since, (epoch, pass, t, command) each - from her epoch's lay on in
#: the first row, so `tools/sim/replay.py` runs it again pass for pass.
RECORDINGS = recorder.RECORDINGS
HEADER = (['t', 'stage', 'yaw', 'speed', 'phase', 'left_load', 'right_load',
           'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'] + list(JOINTS)
          + ['%s_%s' % (seg[0], axis) for seg in SEGMENTS for axis in 'xyz']
          + ['set_' + j for j in JOINTS] + ['watts', 'epoch', 'keys', 'commands'])

#: What M steps her through: dressed, her shell, her mechanism (`coaxial.graphics.mechanism`),
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


def pad():
    """The keys that move her as last seen (`polled`'s) and the twist they asked."""
    return {'keys': dict.fromkeys(('w', 's', 'a', 'd', 'q', 'e', 'shift'), False), 'last_seen': {},
            'setpoint': (0.0, 0.0, 0.0), 'actual': (0.0, 0.0, 0.0)}


def _moving(raw):
    """The keys that move her among `raw` - characters typed, or the console's held names -,
    Shift with an upper-case one."""
    out = set()
    for k in raw:
        if isinstance(k, str) and (k == 'shift' or (len(k) == 1 and k in 'WASDQE')):
            out.add('shift')
        if isinstance(k, str) and k.lower() in ('w', 'a', 's', 'd', 'q', 'e'):
            out.add(k.lower())
    return out


def polled(raw, st, now, held=None):
    """{key: held} of `st` (`pad`) with `raw` typed now: the console's `held` where it tells, else
    each key seen within HOLD_WINDOW_S - one key typed renews those held beside it, its opposite
    dropped -; of two opposite keys, neither."""
    tokens, seen = _moving(raw), st['last_seen']
    if held is not None and (held or not tokens):
        active = _moving(held) | tokens
        st['last_seen'] = dict.fromkeys(active, float(now))
    else:
        for k1, k2 in (('w', 's'), ('s', 'w'), ('a', 'd'), ('d', 'a'), ('q', 'e'), ('e', 'q')):
            if k1 in tokens and k2 not in tokens:
                seen.pop(k2, None)
        if tokens and 'shift' not in tokens:
            seen.pop('shift', None)
        if tokens:
            for k in list(seen):
                if now - seen[k] <= HOLD_WINDOW_S:
                    seen[k] = float(now)
        seen.update(dict.fromkeys(tokens, float(now)))
        active = {k for k, ts in seen.items() if now - ts <= HOLD_WINDOW_S}
    w, s, a, d = ('w' in active) > ('s' in active), ('s' in active) > ('w' in active), \
        ('a' in active) > ('d' in active), ('d' in active) > ('a' in active)
    q, e = ('q' in active) > ('e' in active), ('e' in active) > ('q' in active)
    st['keys'] = {'w': w, 's': s, 'a': a, 'd': d, 'q': q, 'e': e, 'shift': 'shift' in active and (w or s or a or d or q or e)}
    return st['keys']


def twist(keys, look, heading):
    """(vx, vy, wz) the `keys` held (`polled`) ask of her: `look` the heading the view looks along,
    `heading` hers, rad."""
    fwd, left = int(keys['w']) - int(keys['s']), int(keys['a']) - int(keys['d'])
    if fwd or left:
        off = se2.wrap_angle(look + math.atan2(left, fwd) - heading)
        wz = round(TWIST_STEP * round(max(-se2.YAW_RATE, min(se2.YAW_RATE, TURN_K * off)) / TWIST_STEP), 2)
        if abs(off) >= 0.5 * math.pi:
            return (0.0, 0.0, wz)
        return (se2.RUN_SPEED if keys['shift'] and abs(off) < SPRINT_RAD else se2.WALK_SPEED, 0.0, wz)
    return (0.0, (int(keys['q']) - int(keys['e'])) * se2.STRAFE_SPEED, 0.0)


def steered(state, now, heading):
    """Each frame: the keys held - the console's, else those typed within HOLD_WINDOW_S - as
    her twist, sent when it changes; a key that moves her ends the tour. `now` the wall clock."""
    held = state.setdefault('pad', pad())
    keys = polled((), held, now, console.held())
    if any(keys[k] for k in 'wsadqe'):
        gym._cancel_tour(state)
    elif gym.touring(state):
        return
    tw = twist(keys, viewpoint.looks(state), heading)
    if tw != held['setpoint']:
        held['setpoint'] = tw
        state['body'].send(twist=tw)


def _charged(state):
    """P, each press and repeat: the push charging."""
    now = time.monotonic()
    charge = state.get('charge')
    if not charge:
        charge = state['charge'] = {'began': now, 'last': now, 'repeats': 0}
    charge['last'] = now
    charge['repeats'] += 1


def charging(state, now):
    """The push charged so far, N: CHARGE_N_S a second since P was first pressed, to
    CHARGE_MAX_N; 0 with none charging."""
    charge = state.get('charge')
    return min(CHARGE_MAX_N, CHARGE_N_S * (now - charge['began'])) if charge else 0.0


def released(state, now):
    """The push let go, N, once P's repeats have stopped (RELEASE_S, FIRST_S); 0 while it
    charges or none is."""
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
    """Backspace: her laid anew on the floor, the tour and the keys' twist ended."""
    gym._cancel_tour(state)
    state['pad'] = pad()
    state['body'].send(restart=True)


def table(view, calling, shown):
    """{key: what it does to the state}: `view`'s keys and the page's own; L and T step through
    `calling` and `shown`."""
    return dict(
        list(view.items())
        + [(k, _restarted) for k in ('\x08', '\x7f')]
        + [(k, lambda state: state.update(orbit=not state['orbit'])) for k in 'oO']
        + [(k, lambda state: state.update(called=calling[(calling.index(state['called']) + 1)
                                                         % len(calling)])) for k in 'lL']
        + [(k, _recorded) for k in 'rR']
        + [(k, lambda state: state.update(shown=shown[(shown.index(state['shown']) + 1)
                                                      % len(shown)])) for k in 'tT']
        + [(k, _skinned) for k in 'mM'] + [('p', _charged)]
        + [(k, lambda state: state.update(horizon=not state.get('horizon'))) for k in 'zZ'])


def act_on(keys, typed, state, wheel=0.0):
    """`typed` and the wheel on the view's `state`, by `keys` (`table`): the keys that move her
    into her pad (`steered` sends them), the rest each its handler, a key repeated within 0.12 s
    once - P's repeats all, they charge it."""
    if wheel:
        viewpoint.zoomed(state, max(0.5, 1.0 + wheel))
    state.setdefault('typed', []).extend(typed)
    state.setdefault('keyed', []).extend(k for k in typed if isinstance(k, str))
    now = time.monotonic()
    moves = [k for k in typed if k in MOVES]
    if moves:
        gym._cancel_tour(state)
        polled(moves, state.setdefault('pad', pad()), now, console.held())
    for key in typed:
        if key in MOVES or key not in keys:
            continue
        last, at = state.get('_last_action_key', (None, 0.0))
        if key == last and key != 'p' and now - at < 0.12:
            continue
        state['_last_action_key'] = (key, now)
        keys[key](state)
