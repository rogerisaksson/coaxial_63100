"""The HUMANOID page's keys: its rows of the terminal's one table (`terminal.ui.bindings`), its modes.

    run_view(.., pad=humanoid_keys.PAD, state=state)       # her page's keys, the terminal's after them
    humanoid_keys.frame(state, now, heading)               # each frame: what is held, F's choice

Her modes come before her rows: the LLM prompt open takes every key - Enter says it, Esc closes
it, Backspace takes a character back -; F's choice open takes a digit, F or a function key, a key
that acts on her closing it first, the view's leaving it, Esc shutting it. W A S D Q E - Shift
sprints - are held into her pad, its twist told her each frame (`steered`); Space and P charge
while held; the rest act once a press. An act takes the view's state, moves it and sends her body
(`machine.sprung`) what it asks - what an agent sends, its verbs `terminal.views.gym`'s. R's
recordings and their rows (HEADER, `row`) are here with the key that writes them.
"""
import json
import math

from machine import se2, sprung
from machine.figure import JOINTS, SEGMENTS, frames, quat
from machine.leap import hop_charge
from terminal.ui import bindings, console, recorder
from terminal.views import gym, viewpoint

#: P held charges a push from the view onto her: CHARGE_N_S a second held, CHARGE_MAX_N at most,
#: landing once the key's repeats stop - RELEASE_S after the last, FIRST_S after a lone press (a
#: terminal's first repeat comes 250-500 ms on) - along the view's line of sight.
CHARGE_N_S, CHARGE_MAX_N, RELEASE_S, FIRST_S = 120.0, 250.0, 0.25, 0.8
#: Space's repeats feed her hop's charge HOP_HOLD_S after it; her table's rows of a hop under way, where Space
#: says no other (`leap.standing_leap`'s names, a stride hop's push).
HOP_HOLD_S = 0.55
HOPPING = ('hop', 'load', 'push', 'throw', 'flight', 'land')

#: Where R's recordings go, a CSV from a press to the next: every state the page hears from her,
#: 10-20 a second of her time (HEADER); beside each, the keys typed and held since the row before
#: and the commands she heard since, (epoch, pass, t, command) each - from her epoch's lay on in
#: the first row, so `tools/sim/replay.py` runs it again pass for pass.
RECORDINGS = recorder.RECORDINGS
HEADER = (['t', 'stage', 'yaw', 'speed', 'phase', 'left_load', 'right_load',
           'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'] + list(JOINTS)
          + ['%s_%s' % (seg[0], axis) for seg in SEGMENTS for axis in 'xyz']
          + ['set_' + j for j in JOINTS] + ['watts', 'epoch', 'ratio', 'headroom', 'crc', 'link', 'keys', 'commands'])

#: What L, T and M step through: the joints called out (`show_humanoid.CALLED`), what each
#: callout shows, her skin - dressed, her shell, her mechanism (`coaxial.graphics.mechanism`), her
#: drives alone.
CALLING, SHOWN, SKINS = ('strong', 'all', 'none'), ('torque', 'power', 'temp'), ('dressed', 'shell', 'mechanism', 'actuators')
_CYCLES = {'l': ('called', CALLING), 't': ('shown', SHOWN), 'm': ('skin', SKINS)}
#: The view's steps: an arrow's turn, deg, round her; a key's zoom.
_LOOKS, _ZOOMS = {'left': -viewpoint.TURN_DEG, 'right': viewpoint.TURN_DEG}, {'+': 1.1, '=': 1.1, '-': 1 / 1.1, '_': 1 / 1.1}
#: The scenes' keys, each the step through `floor.SCENARIOS` it takes - Enter the one she is in.
_SCENES = {'[': -1, ']': 1, ',': 1, '<': 1, '\r': 0, '\n': 0}
#: The function keys, F1 the first balloon's.
FUNCTION_KEYS = tuple('f%d' % (k + 1) for k in range(len(gym.DIGITS)))


def row(now, yaw, keys='', commands=(), crc=0, link=0):
    """A recording's row (HEADER) at the page's `yaw`: `keys` typed and held, `commands` heard, her
    pace against the wall and what it could be, the frames her buses refused (`crc`) and her
    Renode limb's writes timed out or refused (`link`) so far."""
    placed = frames(now['angles'], now['where'], quat(*now['turn']))
    asked = now.get('set', {})
    return ([round(now['t'], 4), now['stage'], yaw, now['speed'], now['phase']]
            + list(now['loads']) + list(now['where']) + list(now['turn'])
            + [now['angles'].get(j, 0.0) for j in JOINTS]
            + [v for seg in SEGMENTS for v in placed[seg[0]][0]]
            + [asked.get(j, float('nan')) for j in JOINTS] + [now.get('watts', float('nan')),
                                                              now.get('epoch', 0),
                                                              now.get('ratio', float('nan')),
                                                              now.get('headroom', float('nan')), crc, link, keys,
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
    held = sorted(k for k in (console.held() or ()) if isinstance(k, str) and k != ' ')
    keys = ' '.join(typed) + (' [%s]' % '+'.join(held) if held else '')
    out, emu = [], state.get('emu')
    for k, now in enumerate(said):
        crc = sum(int(v[1]) for v in (now.get('buses') or {}).values() if len(v) > 1)
        out.append(row(now, state['yaw'], keys if k == 0 else '', state['pending'] if k == 0 else (), crc,
                       emu.faults if emu else 0))
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
    each key seen within `console.HOLD_WINDOW_S` - one key typed renews those held beside it, its
    opposite dropped -; of two opposite keys, neither."""
    tokens, seen = _moving(raw), st['last_seen']
    if held is not None:
        active = _moving(held)
        st['last_seen'] = dict.fromkeys(active, float(now))
    else:
        for k1, k2 in (('w', 's'), ('s', 'w'), ('a', 'd'), ('d', 'a'), ('q', 'e'), ('e', 'q')):
            if k1 in tokens and k2 not in tokens:
                seen.pop(k2, None)
        if tokens and 'shift' not in tokens:
            seen.pop('shift', None)
        if tokens:
            for k in list(seen):
                if now - seen[k] <= console.HOLD_WINDOW_S:
                    seen[k] = float(now)
        seen.update(dict.fromkeys(tokens, float(now)))
        active = {k for k, ts in seen.items() if now - ts <= console.HOLD_WINDOW_S}
    w, s, a, d = ('w' in active) > ('s' in active), ('s' in active) > ('w' in active), \
        ('a' in active) > ('d' in active), ('d' in active) > ('a' in active)
    q, e = ('q' in active) > ('e' in active), ('e' in active) > ('q' in active)
    st['keys'] = {'w': w, 's': s, 'a': a, 'd': d, 'q': q, 'e': e, 'shift': 'shift' in active and (w or s or a or d or q or e)}
    return st['keys']


def twist(keys, look=0.0, heading=0.0):
    """(vx, vy, wz) the `keys` held (`polled`) ask of her in her own frame, independent of `look`."""
    _ = look, heading
    fwd = int(keys['w']) - int(keys['s'])
    turn = int(keys['a']) - int(keys['d'])
    strafe = int(keys['q']) - int(keys['e'])
    wz = round(turn * se2.YAW_RATE, 2)
    if fwd < 0:
        return (-se2.BACK_SPEED, 0.0, wz)
    if fwd > 0:
        return (se2.RUN_SPEED if keys['shift'] and not turn else se2.WALK_SPEED, 0.0, wz)
    if turn:
        return (0.0, 0.0, wz)
    return (0.0, strafe * se2.STRAFE_SPEED, 0.0)


def _hop_held(state, now, held):
    """Space held during the leap's preload window (5 ms to 250 ms): sends progressive `charge` in [0, 1]."""
    hh = state.get('hop_hold')
    if not hh or now - hh['began'] > HOP_HOLD_S:
        state['hop_hold'] = None
        return
    if ' ' in (held or ()) or (hh.get('repeats', 0) > 0 and now - hh['last'] <= bindings.REPEAT_S):
        ch = hop_charge(now - hh['began'])
        if ch > hh['sent'] + 0.02:
            hh['sent'] = ch
            state['body'].send(charge=round(ch, 3))


def steered(state, now, heading):
    """Each frame: the keys held - the console's, else those typed within
    `console.HOLD_WINDOW_S` - as her twist, sent when it changes and again as she reports a scene
    laid anew, her twist zeroed with it; a key that moves her ends the tour. `now` the wall clock."""
    if (state.get('llm_prompt') or {}).get('open'):
        return
    raw_held = console.held()
    _hop_held(state, now, raw_held)
    held = state.setdefault('pad', pad())
    keys = polled((), held, now, raw_held)
    if any(keys[k] for k in 'wsadqe'):
        gym.cancel_tour(state)
    elif gym.touring(state):
        return
    tw = twist(keys, viewpoint.looks(state), heading)
    scene = (state.get('now') or {}).get('scenario')
    if scene != state.get('_scene'):
        state['_scene'], held['setpoint'] = scene, None
    if tw != held['setpoint']:
        held['setpoint'] = tw
        state['body'].send(twist=tw)


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


def _chosen(state, k):
    """F's `k`th offer said (`gym.chose`), its choice closed."""
    state['choosing'] = None
    gym.chose(state, k)


def _interact(state, _key, now):
    """F: with one offer, or F again while its choice is open, the first's at once; with more, the
    choice open - a digit within `console.HOLD_WINDOW_S` says its offer (`_digit`), else the
    first's once that passes (`frame`)."""
    state['f_at'] = now
    if state.get('choosing') is None and len(gym.offered(state)) > 1:
        state['choosing'] = now
        return
    _chosen(state, 0)


def _digit(state, key, now):
    """A digit: within `console.HOLD_WINDOW_S` of F the offer it keys while F's choice is open -
    nothing once F said its one -, else the gym's entry of its place."""
    k = gym.DIGITS.index(key)
    if now - state.get('f_at', -math.inf) > console.HOLD_WINDOW_S:
        return gym.said(state, k)
    state['f_at'] = -math.inf
    if state.get('choosing') is not None:
        _chosen(state, k)


def _picked(state, key, _now):
    """A function key: the balloon it keys, at once."""
    _chosen(state, FUNCTION_KEYS.index(key))


def _hopped(state, _key, now):
    """A hop said (`gym.hopped`), its charge from now."""
    if gym.hopped(state) == 'hop':
        state['hop_hold'] = {'began': now, 'last': now, 'sent': 0.0}


_HOPPED = bindings.once(_hopped)


def _hop(state, key, now):
    """Space: a hop once a press, its repeats while she loads it charging it (`_hop_held`), none said while one is under
    way (HOPPING): held on, it said one in her flight and her landing and felled her."""
    hh, doing = state.get('hop_hold'), (state.get('now') or {}).get('doing')
    if hh and now - hh['began'] <= HOP_HOLD_S and doing in ('load', 'push', 'hop'):
        hh['last'], hh['repeats'] = now, hh.get('repeats', 0) + 1
        return _hop_held(state, now, (' ',))
    if doing in HOPPING:
        return None
    return _HOPPED(state, key, now)


def _charged(state, _key, now):
    """P, each press and repeat: the push charging."""
    charge = state.get('charge')
    if not charge:
        charge = state['charge'] = {'began': now, 'last': now, 'repeats': 0}
    charge['last'] = now
    charge['repeats'] += 1


def _prompted(state, _key, _now):
    """/ or : the LLM prompt open (`gym.open_prompt`), what was held or chosen let go."""
    gym.open_prompt(state)
    state['hop_hold'] = state['charge'] = state['choosing'] = None


def _restarted(state):
    """Backspace: her laid anew on the floor, the tour and the keys' twist ended."""
    gym.cancel_tour(state)
    state['pad'] = pad()
    state['body'].send(restart=True)


def _cycled(state, key, _now):
    """L, T, M: the next of what the key steps through (_CYCLES)."""
    name, ring = _CYCLES[key.lower()]
    state[name] = ring[(ring.index(state[name]) + 1) % len(ring)]


def _horizon(state, key, _now):
    """Z on through her horizon's modes (`horizon.MODES`), Shift+Z back."""
    from coaxial.graphics.horizon import MODES
    state['horizon'] = MODES[(MODES.index(state.get('horizon') or 'off') + (1 if key == 'z' else -1)) % len(MODES)]


_did, _cased, _HELD = bindings.did, bindings.cased, bindings.HELD
#: Her page's keys, a row a line of TAB's help (`bindings.page`), the terminal's after them.
_ROWS = (
    ('HER', 'W A S D', 'walk ahead, turn left, walk back, turn right (in place, or on an arc with W/S)',
     (('WASD', 'MOVE'),), dict.fromkeys('wasdWASD', _HELD)),
    ('HER', 'SHIFT', 'sprint', (('SHIFT', 'SPRINT'),), {}),
    ('HER', 'Q E', 'sidestep left, right', (('Q E', 'SIDESTEP'),), dict.fromkeys('qeQE', _HELD)),
    ('HER', 'SPACE', 'hop - onto a chair ahead, down from one', (('SPC', 'HOP'),), {' ': _hop}),
    ('HER', 'F / F 1-0', "a balloon's verb: the one she faces / the one so keyed (or F1-F10)",
     (('F', 'INTERACT'),), {**_cased('f', _interact), **dict.fromkeys(FUNCTION_KEYS, _picked)}),
    ('HER', 'C', 'crouch, and up', (('C', 'CROUCH'),), _cased('c', _did(gym.crouched))),
    ('HER', '1-0 / >', "the gym's entries, but after F / its other text", (),
     {**dict.fromkeys(gym.DIGITS, _digit), '>': _did(gym.next_text)}),
    ('VIEW', 'L-DRAG / ← →', 'look round her; behind her again as she goes', (),
     {k: lambda state, key, _now: viewpoint.turned(state, _LOOKS[key] / viewpoint.DRAG_DEG[0], 0.0) for k in _LOOKS}),
    ('VIEW', 'X', 'the other shoulder', (('X', 'SHOULDER'),), _cased('x', _did(viewpoint.shouldered))),
    ('VIEW', 'Z', "her horizon: each sole's landing room, her capture point", (), _cased('z', _horizon)),
    ('VIEW', 'R-DRAG / WHEEL + -', 'pan / zoom', (),
     {k: lambda state, key, _now: viewpoint.zoomed(state, _ZOOMS[key]) for k in _ZOOMS}),
    ('VIEW', 'V / O', 'behind her / orbit', (), {**_cased('v', _did(lambda state: state.update(viewpoint.HOME))),
                                                 **_cased('o', _did(lambda state: state.update(orbit=not state['orbit'])))}),
    ('VIEW', 'M', 'dressed, shell, mechanism, drives', (), _cased('m', _cycled)),
    ('VIEW', 'L / T', 'callouts / torque, power, temp', (), {**_cased('l', _cycled), **_cased('t', _cycled)}),
    ('LAB', '[ / ] , < / ENTER', 'scene before / after / again', (('[ ]', 'SCENE'),),
     {k: lambda state, key, _now: gym.step_scenario(state, _SCENES[key]) for k in _SCENES}),
    ('LAB', 'P held', 'charge a push along the view', (), {'p': _charged}),
    ('LAB', 'SHIFT+P / SHIFT+N', '%.0f N / %.0f N along the view' % tuple(gym.SHOVES_N.values()), (),
     {k: lambda state, key, _now: gym.shoved(state, gym.SHOVES_N[key]) for k in gym.SHOVES_N}),
    ('LAB', 'H / /', 'the tour of the scenes / ask LLM prompt', (('H', 'TOUR'), ('/', 'CHAT')),
     {**_cased('h', _did(gym.toggle_tour)), '/': _prompted, ':': _prompted}),
    ('LAB', 'BACKSPACE', 'laid anew', (), dict.fromkeys('\x08\x7f', _did(_restarted))),
    ('LAB', 'R', 'record / save', (), _cased('r', _did(_recorded))),
)
#: Keys whose repeats are their own - P's charge it, Space's its hop's -; every other key's act
#: once a press (`bindings.once`).
REPEATING = frozenset('p ')
ROWS = tuple((group, label, what, bar, {key: act if act is _HELD or key in REPEATING else bindings.once(act)
                                        for key, act in binds.items()})
             for group, label, what, bar, binds in _ROWS)
#: The keys F's choice takes; the view's leave it open, the rest shut it before they act.
CHOOSING = frozenset(gym.DIGITS) | frozenset('fF') | frozenset(FUNCTION_KEYS)
VIEWING = frozenset(key for group, *_mid, binds in ROWS if group == 'VIEW' for key in binds)

_PROMPT_ACTIONS = {
    '\r': gym.submit_prompt,
    '\n': gym.submit_prompt,
    '\x1b': lambda s: s['llm_prompt'].update(open=False),
    '\x08': lambda s: s['llm_prompt'].update(text=s['llm_prompt']['text'][:-1]),
    '\x7f': lambda s: s['llm_prompt'].update(text=s['llm_prompt']['text'][:-1]),
}


def _prompt_key(state, key):
    """`key` into the open LLM prompt: its own (_PROMPT_ACTIONS), else a character of its text."""
    act = _PROMPT_ACTIONS.get(key)
    if act is not None:
        act(state)
    elif isinstance(key, str) and len(key) == 1 and key.isprintable():
        state['llm_prompt']['text'] += key


def _take(state, key, _now):
    """Her page's modes, before its rows: the LLM prompt open takes every key (`_prompt_key`); F's
    choice open takes Esc - shut -, leaves its own keys and the view's to their rows, and is shut by
    any other before it acts; a key past the prompt kept for R's recording (`rows_of`)."""
    if (state.get('llm_prompt') or {}).get('open'):
        return _prompt_key(state, key)
    state.setdefault('keyed', []).append(key)
    if state.get('choosing') is not None and key == '\x1b':
        state['choosing'] = None
        return None
    if key not in CHOOSING | VIEWING:
        state['choosing'] = None
    return bindings.PASS


def _held(state, keys, now):
    """`keys` that move her, typed together, into her pad; the tour ended."""
    gym.cancel_tour(state)
    polled(keys, state.setdefault('pad', pad()), now, console.held())


#: Her page's keys for `run_view`: its rows, the terminal's after them, its modes and its pad.
PAD = bindings.page(ROWS, take=_take, held=_held)


def clicked(state):
    """A left click on the viewport while F's choice is open: its first offer said, the choice closed. With none open
    a click says nothing - said F's first offer, a click to turn the view walked her off to it."""
    if state.get('choosing') is not None:
        _chosen(state, 0)


def lapsed(state, now):
    """F's choice open past `console.HOLD_WINDOW_S`: the first offer said, its choice closed."""
    opened = state.get('choosing')
    if opened is not None and now - opened > console.HOLD_WINDOW_S:
        _chosen(state, 0)


def frame(state, now, heading):
    """Each frame: F's choice lapsed (`lapsed`); the keys held - the console's, else those typed
    within `console.HOLD_WINDOW_S` - her twist and Space's charge (`steered`); P let go shoves her
    along the view. `now` the wall clock."""
    lapsed(state, now)
    steered(state, now, heading)
    pushed = released(state, now)
    if pushed and state.get('now') is not None:
        state['body'].send(push=tuple(pushed * a for a in viewpoint.along(state)), seconds=sprung.SHOVE_S)
        state['landed'] = (now, pushed)
