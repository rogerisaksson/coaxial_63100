"""The HUMANOID page's gym: her verbs, her latent loop's moves said by digit, her modes shown.

    KEYS = gym.table(KEYS)       # the page's keys with the gym's: verbs, digits, scenes, shoves
    gym.offered(state)           # what F offers now, keyed: the balloons, the plate, the prompt
    gym.boxes(now)               # the side column's LATENT and SAY boxes
    gym.status(now)              # what she is doing, in a line

She stands on the loop (`machine.sprung`); the keys that move her are a twist (`humanoid_keys`).
Space, C and F are verbs - a hop, a crouch, a balloon's offer -, each one an agent would send.
F offers rising from what bears her, else each thing she has seen and its verb, a balloon on it:
F alone says the first's - the one she faces -, F then a digit - or the function key - the one so
keyed. A digit says
an entry of the gym's text (`sprung.entries`) in place of what was to come, > takes the gym's
other text (`sprung.GYMS`), read anew each time - an edit to machine/gynoid_mesomorph_medium.lisp
is two presses away -; [ ] step through the scenes, Enter lays the scene again, P N shove her
along the view, H tours the scenes. LATENT shows each mode as it is asked and as she is, and its
spring.
"""
import math
import time

from rich.text import Text

from coaxial.draw import gauges
from coaxial.graphics import balloons, gynoid
from machine import sight, sprung, tour
from terminal import keybindings
from terminal.ui import console
from terminal.ui.stage import hud
from terminal.views import scanner, viewpoint

#: The gym's next text.
NEXT = '>'
#: A digit says the gym's entry of its place: 1 the first, 0 the tenth.
DIGITS = '1234567890'
#: P and N shove her along the view, the bench's 80 N she parries and 200 N she falls to
#: (`tools/sim/scenarios.py`, SHOVES), sprung.SHOVE_S long.
SHOVES_N = {'P': 80.0, 'N': 200.0}
#: A thing near enough for a verb she does where she stands, m from her pelvis over the floor; a
#: chair she hops onto, m ahead and within a quarter turn of her heading; her pelvis over this, m,
#: on a chair's seat.
REACH_M, ONTO_M, ON_CHAIR_M = 1.2, 0.7, 1.12
#: The crouch's phase (`postures_base.lisp`).
CROUCHED = 'down'
#: Each mode's name in the box; what each stream payload is called on the page.
SHORT = ('axial', 'lateral', 'drive', 'pitch', 'sagittal', 'lift', 'width', 'list', 'turn', 'arms', 'rock')
NAMED = {
    'stand': 'stand', 'sway': 'sway', 'step': 'step', 'walk': 'walk', 'walk_back': 'walk back',
    'weight_shift': 'weight shift', 'bow': 'bow', 'ballistic_walk': 'ballistic walk',
    'sit_chair': 'sit chair', 'rise_chair': 'rise chair', 'lie_bed': 'lie bed', 'rise_bed': 'rise bed',
    'lie_floor': 'lie floor', 'rise_floor': 'rise floor', 'hop_down_chair': 'hop down chair',
    'hop_up_chair': 'hop up chair', 'strafe_left': 'sidestep left', 'strafe_right': 'sidestep right',
    'turn_left': 'turn left', 'turn_right': 'turn right', 'spin_left': 'spin left', 'spin_right': 'spin right',
    'turn_back_left': 'turn back left', 'turn_back_right': 'turn back right', 'exit_door': 'exit door',
}

_RISE_TABLE = (
    (('seat_land', 'hop_up_chair'), None, 'hop_down_chair'),
    (('bed_rest', 'lie_bed', 'sit_edge', 'recline'), None, 'rise_bed'),
    (('approach', 'lower'), ('bed',), 'rise_bed'),
    (('floor_rest', 'lie_floor', 'kneel'), None, 'rise_floor'),
    (('approach', 'lower', 'recline', 'cushion', 'land'), ('mattress', 'floor'), 'rise_floor'),
    (('seated', 'sit_chair'), None, 'rise_chair'),
    (('cushion', 'land'), ('chair',), 'rise_chair'),
    (('approach', 'lower'), None, 'rise_chair'),
    (('cushion',), None, 'rise_floor'),
)


def contextual_rise(now, scenario=None):
    """The verb that takes her up from what bears her - seated, lying, kneeling, crouched, on a
    chair's seat -; None standing."""
    if not now:
        return None
    doing, scen = now.get('doing', ''), scenario or now.get('scenario') or 'open'
    if (now.get('where') or (0.0, 0.90, 0.0))[1] > ON_CHAIR_M:
        return 'hop_down_chair'
    for phases, scens, move in _RISE_TABLE:
        if doing in phases and (scens is None or scen in scens):
            return move
    if now.get('stage') in ('down', 'fallen', 'floor'):
        return 'rise_floor'
    return 'stand' if (doing == CROUCHED or now.get('fallen')) else None


def hop(now, seen):
    """What Space says now: down from a chair's seat, up onto a chair close ahead (`seen`,
    `scanner.scan`'s), else a hop; None before her first state."""
    if not now:
        return None
    x, _y, z = now.get('where') or (0.0, 0.9, 0.0)
    heading = float(now.get('heading') or 0.0)
    ahead = [d for d in seen if d['kind'] == 'chair' and d['dist'] < ONTO_M
             and (d['pos'][0] - x) * math.sin(heading) + (d['pos'][2] - z) * math.cos(heading) > 0.5 * d['dist']]
    return 'hop_down_chair' if contextual_rise(now) == 'hop_down_chair' else ('hop_up_chair' if ahead else 'hop')


def _things(seen):
    """`seen` (`scanner.scan`'s) joined into things: a kind's boxes within her map's cell of each
    other one (`sight.lumps`) - a chair's seat and its back -; each (kind, its nearest box's dist,
    lo, hi), its bounds."""
    things = []
    for d in seen:
        lo = [p - e for p, e in zip(d['pos'], d['extent'])]
        hi = [p + e for p, e in zip(d['pos'], d['extent'])]
        met = [t for t in things if t[0] == d['kind'] and all(
            a <= y + sight.CELL_M and x <= b + sight.CELL_M for a, b, x, y in zip(lo, hi, t[2], t[3]))]
        for _kind, _dist, a, b in met:
            lo, hi = [min(u, v) for u, v in zip(lo, a)], [max(u, v) for u, v in zip(hi, b)]
        things = [t for t in things if t not in met] + [(d['kind'], min([d['dist']] + [t[1] for t in met]), lo, hi)]
    return things


def _turn(now, at):
    """How far she turns to face `at` (x, y, z), rad."""
    x, _y, z = now.get('where') or (0.0, 0.9, 0.0)
    off = math.atan2(at[0] - x, at[2] - z) - float(now.get('heading') or 0.0)
    return abs((off + math.pi) % math.tau - math.pi)


def offers(now, seen):
    """[(verb, word, at (x, y, z), thing)] F offers now, the one she faces most nearly first - the
    nearer of two in line -: rising from what bears her alone, at her; else each thing she has
    identified (`seen`, `scanner.identified`'s) with a verb - one she walks to (`tour.errands`)
    wherever it stands, one she does where she stands within REACH_M -, at its middle."""
    if not now:
        return []
    rise, walked = contextual_rise(now), tour.errands()
    if rise:
        return [(rise, NAMED.get(rise, rise), tuple(now.get('where') or (0.0, 0.9, 0.0)), 'her')]
    got = [(verb, word, tuple((a + b) / 2.0 for a, b in zip(lo, hi)), '%s %.1fm' % (kind, dist), dist)
           for kind, dist, lo, hi in _things(seen) if kind in scanner.AFFORDANCES
           for verb, word in (scanner.AFFORDANCES[kind],) if verb in walked or dist < REACH_M]
    return [offer[:4] for offer in sorted(got, key=lambda o: (_turn(now, o[2]), o[4]))][:len(DIGITS)]


def keyed(n):
    """The keys of `n` offers: F and each's digit (`DIGITS`), one alone F1 too."""
    return ['F' + d for d in DIGITS[:n]]


def offered(state):
    """[(key, verb, word, at, thing)] F offers as the page shows them: what she has identified
    (`offers`, `keyed`)."""
    now, sc = state.get('now'), state.get('scanner')
    got = offers(now, sc.identified(now) if sc is not None and now else [])
    return [(key,) + offer for key, offer in zip(keyed(len(got)), got)]


def prompt(state):
    """What F and Space would say, as the page offers it: '[F1] sit chair  [F2] exit door  [SPC]
    hop up', a plain hop unsaid; '' with nothing to offer."""
    now, sc = state.get('now'), state.get('scanner')
    words = ([(key, verb) for key, verb, *_rest in offered(state)]
             + [('SPC', hop(now, sc.scan(now) if sc is not None and now else []) or '')])
    return '  '.join('[%s] %s' % (k, NAMED.get(v, v).replace('_', ' ')) for k, v in words if v and v != 'hop')


def ballooned(state, offer, width, height, travel, docked=()):
    """The viewport's balloons: each of `offer` (`offered`'s) its key and word on its thing, on a
    drawing `width` by `height` of the page's view over `travel`, out of the `docked` picture's
    cells; framed lit while F's choice is open (`coaxial.graphics.balloons`)."""
    places = gynoid.cells([at for _k, _v, _w, at, _t in offer], width, height, yaw=state['yaw'],
                          pitch=state['pitch'], zoom=state['zoom'], travel=travel, pan=viewpoint.panned(state))
    return balloons.laid([(p, key, word) for p, (key, _v, word, _a, _t) in zip(places, offer) if p],
                         width, height, lit=state.get('choosing') is not None, shut=frozenset(docked))


def touring(state):
    """Whether the tour (`machine.tour`) is commanding her."""
    tour = state.get('tour')
    return tour is not None and not tour.done


def _cancel_tour(state):
    """The tour ended by a key that acts on her; the keys' twist sent again on the next frame."""
    if touring(state):
        state['tour'] = None
        state.setdefault('pad', {})['setpoint'] = None


def _said(state, verb, **where):
    """`verb` said, the tour and any held pad keys ended: a gym entry's name or place; `where`
    its thing's place (`at`, x z)."""
    _cancel_tour(state)
    pad = state.get('pad')
    if isinstance(pad, dict) and verb != 'hop':
        pad['keys'] = dict.fromkeys(pad.get('keys') or ('w', 's', 'a', 'd', 'q', 'e', 'shift'), False)
        pad['last_seen'], pad['setpoint'] = {}, (0.0, 0.0, 0.0)
    if verb is not None:
        if verb == 'hop':
            now_t = time.monotonic()
            state['hop_hold'] = {'began': now_t, 'last': now_t, 'sent': 0.0}
        state['body'].send(say=verb, **where)


def _hopped(state):
    """Space: what it says now (`hop`)."""
    now, sc = state.get('now'), state.get('scanner')
    _said(state, hop(now, sc.scan(now) if sc is not None and now else []))


def chose(state, k):
    """F's `k`th offer said - its verb at its thing (`at`, x z) -, F's choice closed."""
    state['choosing'] = None
    for _key, verb, _word, at, _thing in offered(state)[k:k + 1]:
        _said(state, verb, at=(at[0], at[2]))


def _interact(state):
    """F: with one offer, or F again while the choice is open, the first's said at once; with
    more, the choice open - a digit within `console.HOLD_WINDOW_S` (F counted held) says its
    offer (`_digit`), else the first's once that passes (`lapsed`)."""
    t = state['f_at'] = time.monotonic()
    if state.get('choosing') is None and len(offered(state)) > 1:
        state['choosing'] = t
        return
    chose(state, 0)


def _digit(state, i):
    """A digit: within `console.HOLD_WINDOW_S` of F, F's `i`th offer while its choice is open -
    nothing once F said its one -; else the gym's `i`th entry."""
    if time.monotonic() - state.get('f_at', -math.inf) > console.HOLD_WINDOW_S:
        return _said(state, i)
    state['f_at'] = -math.inf
    if state.get('choosing') is not None:
        chose(state, i)


def lapsed(state, now):
    """Each frame: F's choice open past `console.HOLD_WINDOW_S` of the wall clock `now` says the
    first's."""
    if now - (state.get('choosing') or math.inf) > console.HOLD_WINDOW_S:
        chose(state, 0)


def _crouched(state):
    """C: a crouch, or up from one."""
    now = state.get('now') or {}
    _said(state, 'stand' if now.get('doing') == CROUCHED else 'crouch')


def _run_scenario(state, scen=None):
    """Scene `scen` laid round her, the one she is in again with none."""
    _cancel_tour(state)
    scen = scen or state.get('scenario', 'flat')
    state['scenario'] = scen
    state['body'].send(scenario=scen)
    state.setdefault('pad', {})['setpoint'] = None


def _next(state):
    state['gym'] = sprung.GYMS[(sprung.GYMS.index(state.get('gym', sprung.GYMS[0])) + 1) % len(sprung.GYMS)]
    state['body'].send(gym=state['gym'])


def _step_scenario(state, delta=1):
    from machine import floor
    scens = floor.SCENARIOS
    cur = state.get('scenario', 'flat')
    _run_scenario(state, scens[(scens.index(cur) + delta) % len(scens)] if cur in scens else scens[0])


def _shoved(n):
    def shove(state):
        _cancel_tour(state)
        state['body'].send(push=tuple(n * a for a in viewpoint.along(state)), seconds=sprung.SHOVE_S)
    return shove


def _toggle_tour(state):
    """H / h: the tour from its start, or ended."""
    from machine.tour import HouseTour
    if touring(state):
        _cancel_tour(state)
        return
    tr = state['tour'] = HouseTour(loop=True)
    state['llm_last'] = (tr.last_thought, tr.last_cmd)
    state['llm_feed'] = (state.get('llm_feed', []) + [(tr.last_thought, tr.last_cmd)])[-8:]
    rows: list = list(state.get('chat_rows') or ())
    rows.extend([('value', '> start tour'), ('label', '  cmd: ' + tr.last_cmd), (None, tr.last_thought)])
    state['chat_rows'] = rows[-24:]


def open_prompt(state):
    """/ or : opens the interactive LLM prompt bar for the gynoid and releases held motion keys."""
    state['llm_prompt'] = {'open': True, 'text': ''}
    state['hop_hold'] = state['charge'] = None
    pad = state.get('pad')
    if isinstance(pad, dict):
        pad['keys'] = dict.fromkeys(pad.get('keys') or ('w', 's', 'a', 'd', 'q', 'e', 'shift'), False)
        pad['last_seen'] = {}
        if pad.get('setpoint') not in (None, (0.0, 0.0, 0.0)) and state.get('body') is not None:
            pad['setpoint'] = (0.0, 0.0, 0.0)
            state['body'].send(twist=(0.0, 0.0, 0.0))


def submit_prompt(state, text=None):
    """Compile and dispatch a natural-language LLM prompt for the gynoid."""
    from machine import tour
    q = (text if text is not None else (state.get('llm_prompt') or {}).get('text', '')).strip()
    state['llm_prompt'] = {'open': False, 'text': ''}
    if not q:
        return None
    state['llm_prompt_last'] = q
    rows: list = list(state.get('chat_rows') or ())
    rows.append(('value', '> ' + q))
    plan = tour.compile_prompt(q, state.get('now'))
    if plan.get('tour'):
        tr = state['tour'] = tour.HouseTour(loop=True)
        entry = (plan['thought'], tr.last_cmd)
        state['llm_last'] = entry
        state['llm_feed'] = (state.get('llm_feed', []) + [entry])[-8:]
        rows.extend([('label', '  cmd: ' + tr.last_cmd), (None, plan['thought'])])
        state['chat_rows'] = rows[-24:]
        return plan
    _cancel_tour(state)
    pad = state.get('pad')
    if isinstance(pad, dict):
        pad['keys'] = dict.fromkeys(pad.get('keys') or ('w', 's', 'a', 'd', 'q', 'e', 'shift'), False)
        pad['last_seen'], pad['setpoint'] = {}, (0.0, 0.0, 0.0)
    for cmd in plan.get('cmds', ()):
        cm = tour.format_cmd(cmd, None)
        entry = (plan['thought'], cm)
        state['llm_last'] = entry
        state['llm_feed'] = (state.get('llm_feed', []) + [entry])[-8:]
        rows.append(('label', '  cmd: ' + cm))
        if state.get('body') is not None:
            state['body'].send(**cmd)
    rows.append((None, plan['thought']))
    state['chat_rows'] = rows[-24:]
    return plan


def table(keys):
    """`keys` (the page's own) with the gym's laid over them: verbs, digits, scenes, shoves."""
    out = dict(keys)
    out[NEXT] = _next
    for k in ('<', ',', ']'):
        out[k] = lambda state: _step_scenario(state, 1)
    out['['] = lambda state: _step_scenario(state, -1)
    for k in ('\r', '\n'):
        out[k] = lambda state: _run_scenario(state)
    out.update((k, lambda state, i=i: _digit(state, i)) for i, k in enumerate(DIGITS))
    out.update(('f%d' % (i + 1), lambda state, i=i: chose(state, i)) for i in range(len(DIGITS)))
    out.update((k, _shoved(n)) for k, n in SHOVES_N.items())
    out[' '] = _hopped
    for k in 'fF':
        out[k] = _interact
    for k in 'cC':
        out[k] = _crouched
    for k in 'hH':
        out[k] = _toggle_tour
    for k in '/:':
        out[k] = open_prompt
    return out


def status(now):
    """Her status: what she is doing; down, counting to her landing again; a text or a command
    refused, why."""
    if now is None:
        return 'STARTING'
    if now['fallen']:
        return 'DOWN in %.0f s' % max(0.0, now['recover'])
    if now.get('refused'):
        return 'REFUSED  %s' % now['refused']
    return 'LATENT  %s' % NAMED.get(now['doing'], now['doing'])


def _mode_row(name, row):
    ref, z, k, _d = float(row[1]), float(row[2]), float(row[3]), float(row[4])
    s_ref = 0.5 + 0.5 * max(-1.0, min(1.0, ref))
    s_act = 0.5 + 0.5 * max(-1.0, min(1.0, z))
    bar = gauges.gauge(s_act, 8, centre=0.5, marks=((s_ref, gauges.MARK),))
    return (name.rjust(10), Text.from_ansi('%+5.2f %s %+5.2f %4.0f' % (ref, bar, z, k)))


def boxes(now, tour=None):
    """The side column's boxes: each mode asked and as she is, its spring's N/m; the scene, the
    verbs' keys, what each digit says."""
    if not now:
        return []
    says = now.get('says') or ('gynoid_mesomorph_medium',)
    gym_name, entries = says[0], says[1:]
    scen = str(now.get('scenario') or 'flat')
    say_rows = [
        ('control'.rjust(10), '[TOUR] ' + tour.current_label()[:18] if tour is not None and not tour.done
         else 'keys or an agent'),
        ('scenario'.rjust(10), scen[:25]),
        ('situation'.rjust(10), str(now.get('situation') or 'open')[:25]),
        ('objective'.rjust(10), keybindings.SCENARIO_DESC.get(scen, scen).split('//')[-1].strip()[:25]),
        ('wasd / shf'.rjust(10), 'move & sidestep / sprint'),
        ('q e / spc'.rjust(10), 'turn in place / hop'),
        ('f / c'.rjust(10), 'interact / crouch'),
    ]
    items = [(DIGITS[i], NAMED.get(s, str(s)).replace('_', ' '))
             for i, s in enumerate(entries[:len(DIGITS)])]
    for idx in range(0, len(items), 2):
        (k1, n1), rest = items[idx], items[idx + 1:idx + 2]
        say_rows.append((('%s / %s' % (k1, rest[0][0]) if rest else k1).rjust(10),
                         ('%-12s %-12s' % (n1[:12], rest[0][1][:12]) if rest else n1[:25]).rstrip()))
    return [hud('LATENT', [_mode_row(name, row) for name, row in zip(SHORT, now.get('modes') or ())]),
            hud('SAY  ' + gym_name.upper(), say_rows)]


CHAT_ROWS = 4
CHAT_GREETING = ('name', 'GYNOID LLM AGENT - press / to prompt, H for autonomous house tour')


def chat_box(state, width=80):
    """The persistent BOARD CHAT-style transcript and prompt input panel under the viewport."""
    import time
    from terminal.ui.recorder import SPIN
    rows: list = list(state.get('chat_rows') or (CHAT_GREETING,))
    tail = rows[-(CHAT_ROWS - 1):]
    while len(tail) < CHAT_ROWS - 1:
        tail.insert(0, ('label', ''))
    body = Text(no_wrap=True, overflow='ellipsis')
    room = max(20, width - 4)
    for i, (style, line) in enumerate(tail):
        if i:
            body.append('\n')
        body.append(str(line)[:room], style=style or 'value')
    lp = state.get('llm_prompt') or {}
    open_p, entry = bool(lp.get('open')), str(lp.get('text', ''))
    blink = int(time.monotonic() * 2.0) % 2 == 0
    lead = SPIN[int(time.monotonic() * 8.0) % len(SPIN)] if state.get('llm_busy') else '>'
    body.append('\n')
    body.append('  %s ' % lead, style='value')
    if open_p or entry:
        body.append(entry[:max(1, room - 6)], style='value')
        body.append('_' if (open_p and blink) else ' ', style='value')
    else:
        body.append('press / to talk to the gynoid (e.g. "lie on the bed", "sit in the chair", "tour")'[:max(1, room - 5)],
                    style='label')
    return hud('GYNOID CHAT  LLM / AGENT', [body])
