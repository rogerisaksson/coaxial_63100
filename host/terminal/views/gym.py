"""The HUMANOID page's gym: her verbs, her latent loop's moves said by digit, her modes shown.

    gym.said(state, verb)        # a verb, what an agent sends; the page's keys bind them (`humanoid_keys`)
    gym.offered(state)           # what F offers now, keyed: the balloons, the plate, the prompt
    gym.boxes(now)               # the side column's LATENT and SAY boxes
    gym.status(now)              # what she is doing, in a line

She stands on the loop (`machine.process.sprung`); the keys that move her are a twist (`humanoid_keys`).
A hop, a crouch, a balloon's offer are verbs, each one an agent would send. F offers only what she
has identified, a balloon on it: rising from the thing that bears her, else each thing and its
verb; from the floor alone her rise, no thing and no balloon. A gym entry is said
by its place in the gym's text (`sprung.entries`) in place of what was to come, the gym's other
text (`sprung.GYMS`) read anew each time - an edit to lisp/humanoid/gynoid_mesomorph_medium.lisp is two
presses away -; the scenes stepped through and laid again, shoves along the view, the tour of the
scenes. LATENT shows each mode as it is asked and as she is, and its spring.
"""
import math
import time

from rich.text import Text

from coaxial.draw import gauges
from coaxial.graphics import balloons, gynoid
from machine.way import errand, tour
from machine.eyes import sight
from machine.process import sprung
from terminal.ui.stage import hud
from terminal.ui.taskbar import HELP_S, slid
from terminal.views import scanner, viewpoint

#: A digit says the gym's entry of its place: 1 the first, 0 the tenth.
DIGITS = '1234567890'
#: P and N shove her along the view, the bench's 80 N she parries and 200 N she falls to
#: (`tools/sim/scenarios.py`, SHOVES), sprung.SHOVE_S long.
SHOVES_N = {'P': 80.0, 'N': 200.0}
#: Each scene's objective (`machine.world.floor.SCENARIOS`).
SCENARIO_DESC = {
    'flat': 'level ground // stand and go',
    'lodge': 'the lodge // its deck, stair and steps',
    'chair': 'chair // F sit, Space hop',
    'chair_low': 'low chair // F sit, knee high',
    'bed': 'soft bed // F lies and rises',
    'mattress': 'floor mattress // F lies and rises',
    'slope': 'ramp // walk up 10 deg',
    'slope_down': 'ramp // walk down 10 deg',
    'cross_slope': 'camber // walk across 10 deg',
    'stairs': 'stairs // walk up',
    'stairs_down': 'stairs // walk down',
    'stairs_take': "the takes' stairs // walk up",
    'ice': 'ice // walk on, slick',
    'snow': 'snow // walk on, yielding',
    'asphalt': 'asphalt // walk on, rough',
    'slip': 'slick patch // walk over',
    'obstacle': 'wall and tree // walk into',
    'wall': 'wall // walk into, stand',
    'tree': 'tree // walk into, stand',
    'sill': 'door sill // trip, walk on',
    'hole': '10 cm hole // step in, walk on',
    'rug': 'loose rug // walk over',
    'edge_behind': "step's edge behind // back off it",
}
#: A thing near enough for a verb she does where she stands, m from her pelvis over the floor; a
#: chair she hops onto, m ahead and within a quarter turn of her heading; her pelvis over this, m,
#: on a chair's seat.
REACH_M, ONTO_M, ON_CHAIR_M = 1.2, 0.7, 1.12
#: The crouch's phase (`postures_base.lisp`).
CROUCHED = 'down'
#: Each mode's name in the box; what each stream payload is called on the page.
SHORT = ('axial', 'lateral', 'drive', 'pitch', 'sagittal', 'lift', 'width', 'list', 'turn', 'arms', 'rock', 'reach', 'fold')
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
#: The thing a rise takes her up from, by the verb that put her on it (`scanner.AFFORDANCES`).
_BEARS = {'hop_down_chair': 'sit_chair', 'rise_chair': 'sit_chair', 'rise_bed': 'lie_bed', 'rise_floor': 'lie_floor'}


def _propped(now, scenario=None):
    """The verb that takes her up from a thing bearing her - a chair's seat, a bed, a mattress
    (`_RISE_TABLE`) -; None on her feet or on the floor alone."""
    doing, scen = now.get('doing', ''), scenario or now.get('scenario') or 'open'
    if (now.get('where') or (0.0, 0.90, 0.0))[1] > ON_CHAIR_M:
        return 'hop_down_chair'
    return next((move for phases, scens, move in _RISE_TABLE
                 if doing in phases and (scens is None or scen in scens)), None)


def _floored(now):
    """The verb that takes her up from the floor alone - down, fallen, crouched -; None else."""
    if now.get('stage') in ('down', 'fallen', 'floor'):
        return 'rise_floor'
    return 'stand' if (now.get('doing') == CROUCHED or now.get('fallen')) else None


def contextual_rise(now, scenario=None):
    """The verb that takes her up from what bears her - a thing (`_propped`), else the floor
    (`_floored`) -; None standing."""
    return (_propped(now, scenario) or _floored(now)) if now else None


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


def _middle(lo, hi):
    """A box's middle, (x, y, z)."""
    return tuple((a + b) / 2.0 for a, b in zip(lo, hi))


def offers(now, seen):
    """[(verb, word, at (x, y, z) or None, thing)] F offers now, each a thing she has identified
    (`seen`, `scanner.identified`'s), never one she has not: on a thing, its rise alone, at that
    thing (`_BEARS`) within REACH_M; on the floor alone, its rise at none - no thing, no balloon -;
    on her feet, each thing with a verb - one she walks to (`errand.errands`) wherever it stands, one
    she does where she stands within REACH_M -, at its middle, the one she faces most nearly first,
    the nearer of two in line."""
    if not now:
        return []
    things = [(kind, dist, lo, hi) for kind, dist, lo, hi in _things(seen) if kind in scanner.AFFORDANCES]
    propped, floored = _propped(now), _floored(now)
    if propped:
        bearing = sorted((dist, kind, lo, hi) for kind, dist, lo, hi in things
                         if scanner.AFFORDANCES[kind][0] == _BEARS[propped] and dist < REACH_M)
        return [(propped, NAMED.get(propped, propped), _middle(lo, hi), '%s %.1fm' % (kind, dist))
                for dist, kind, lo, hi in bearing[:1]]
    if floored:
        return [(floored, NAMED.get(floored, floored), None, 'her')]
    walked = errand.errands()
    got = [(verb, word, _middle(lo, hi), '%s %.1fm' % (kind, dist), dist) for kind, dist, lo, hi in things
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
    """The viewport's balloons: each of `offer` (`offered`'s) at a thing its key and word on it, on
    a drawing `width` by `height` of the page's view over `travel`, out of the `docked` picture's
    cells; framed lit while F's choice is open (`coaxial.graphics.balloons`)."""
    on = [o for o in offer if o[3] is not None]
    places = gynoid.cells([at for _k, _v, _w, at, _t in on], width, height, yaw=state['yaw'],
                          pitch=state['pitch'], zoom=state['zoom'], travel=travel, pan=viewpoint.panned(state))
    return balloons.laid([(p, key, word) for p, (key, _v, word, _a, _t) in zip(places, on) if p],
                         width, height, lit=state.get('choosing') is not None, shut=frozenset(docked))


def touring(state):
    """Whether the tour (`machine.way.tour`) is commanding her."""
    tour = state.get('tour')
    return tour is not None and not tour.done


def cancel_tour(state):
    """The tour ended by a key that acts on her; the keys' twist sent again on the next frame."""
    if touring(state):
        state['tour'] = None
        state.setdefault('pad', {})['setpoint'] = None


def said(state, verb, **where):
    """`verb` said, the tour and any held pad keys ended: a gym entry's name or place; `where`
    its thing's place (`at`, x z)."""
    cancel_tour(state)
    pad = state.get('pad')
    if isinstance(pad, dict) and verb != 'hop':
        pad['keys'] = dict.fromkeys(pad.get('keys') or ('w', 's', 'a', 'd', 'q', 'e', 'shift'), False)
        pad['last_seen'], pad['setpoint'] = {}, (0.0, 0.0, 0.0)
    if verb is not None:
        state['body'].send(say=verb, **where)


def hopped(state):
    """A hop, what it is now (`hop`): the verb said."""
    now, sc = state.get('now'), state.get('scanner')
    verb = hop(now, sc.scan(now) if sc is not None and now else [])
    said(state, verb)
    return verb


def chose(state, k):
    """F's `k`th offer said: its verb at its thing (`at`, x z), the floor's at none."""
    for _key, verb, _word, at, _thing in offered(state)[k:k + 1]:
        said(state, verb, **({} if at is None else {'at': (at[0], at[2])}))


def crouched(state):
    """A crouch, or up from one."""
    now = state.get('now') or {}
    said(state, 'stand' if now.get('doing') == CROUCHED else 'crouch')


def run_scenario(state, scen=None):
    """Scene `scen` laid round her, the one she is in again with none."""
    cancel_tour(state)
    scen = scen or state.get('scenario', 'flat')
    state['scenario'] = scen
    state['body'].send(scenario=scen)
    state.setdefault('pad', {})['setpoint'] = None


def next_text(state):
    """The gym's other text (`sprung.GYMS`)."""
    state['gym'] = sprung.GYMS[(sprung.GYMS.index(state.get('gym', sprung.GYMS[0])) + 1) % len(sprung.GYMS)]
    state['body'].send(gym=state['gym'])


def step_scenario(state, delta):
    """The scene `delta` on through `floor.SCENARIOS` laid, the one she is in with 0."""
    from machine.world import floor
    scens = floor.SCENARIOS
    cur = state.get('scenario', 'flat')
    run_scenario(state, scens[(scens.index(cur) + delta) % len(scens)] if cur in scens else scens[0])


def shoved(state, n):
    """A shove of `n` N along the view, sprung.SHOVE_S long."""
    cancel_tour(state)
    state['body'].send(push=tuple(n * a for a in viewpoint.along(state)), seconds=sprung.SHOVE_S)


def toggle_tour(state):
    """The tour from its start, or ended."""
    from machine.way.tour import HouseTour
    if touring(state):
        cancel_tour(state)
        return
    tr = state['tour'] = HouseTour(loop=True)
    state['llm_last'] = (tr.last_thought, tr.last_cmd)
    state['llm_feed'] = (state.get('llm_feed', []) + [(tr.last_thought, tr.last_cmd)])[-8:]
    rows: list = list(state.get('chat_rows') or ())
    rows.extend([('value', '> start tour'), ('label', '  cmd: ' + tr.last_cmd), (None, tr.last_thought)])
    state['chat_rows'] = rows[-24:]


def warm(state):
    """The page's model looked up and loaded off the page's thread (`gynoid.warm`), its name in
    `state['model']`; none here, nothing."""
    import threading
    threading.Thread(target=_warmed, args=(state,), daemon=True).start()


def _warmed(state):
    from coaxial_ollama import gynoid
    from coaxial_ollama.client import OllamaError
    name = state['model'] = state.get('model') or gynoid.model()
    if name:
        try:
            gynoid.warm(name)
        except (OllamaError, OSError):
            pass


def release(state):
    """The page's model's memory handed back as the page closes (`gynoid.release`); none asked, nothing."""
    from coaxial_ollama import gynoid
    from coaxial_ollama.client import OllamaError
    if state.get('model'):
        try:
            gynoid.release(state['model'])
        except (OllamaError, OSError):
            pass


def open_prompt(state):
    """The interactive LLM prompt bar for the gynoid opened, held motion keys released, the page's
    model warmed while the prompt is typed."""
    if state.get('model'):
        warm(state)
    state['llm_prompt'] = {'open': True, 'text': ''}
    pad = state.get('pad')
    if isinstance(pad, dict):
        pad['keys'] = dict.fromkeys(pad.get('keys') or ('w', 's', 'a', 'd', 'q', 'e', 'shift'), False)
        pad['last_seen'] = {}
        if pad.get('setpoint') not in (None, (0.0, 0.0, 0.0)) and state.get('body') is not None:
            pad['setpoint'] = (0.0, 0.0, 0.0)
            state['body'].send(twist=(0.0, 0.0, 0.0))


def submit_prompt(state, text=None):
    """A prompt said: the local model asked on a thread of its own when one is here
    (`coaxial_ollama.gynoid`, any language), its plan dispatched on the page's thread as it
    lands (`drain_plan`); none here, the keyword rules (`tour.compile_prompt`)."""
    import threading
    from coaxial_ollama import gynoid
    q = (text if text is not None else (state.get('llm_prompt') or {}).get('text', '')).strip()
    state['llm_prompt'] = {'open': False, 'text': ''}
    if not q:
        return None
    state['llm_prompt_last'] = q
    rows: list = list(state.get('chat_rows') or ())
    rows.append(('value', '> ' + q))
    name = state['model'] = gynoid.model()
    if name:
        state['chat_rows'] = (rows + [('label', '  thinking (%s) ...' % name)])[-24:]
        state['asking'] = threading.Thread(target=_asked, args=(state, q, name), daemon=True)
        state['asking'].start()
        return None
    state['chat_rows'] = rows[-24:]
    return _dispatch(state, tour.compile_prompt(q, state.get('now')))


def _asked(state, q, name):
    """The model's plan of `q` into `state['plan_ready']`; the model failing, the keyword rules'."""
    from coaxial_ollama import gynoid
    from coaxial_ollama.client import OllamaError
    now = state.get('now') or {}
    try:
        plan = gynoid.plan(q, now.get('doing', 'rest'), name)
    except (OllamaError, OSError, ValueError, RuntimeError):
        plan = tour.compile_prompt(q, now)
    state['plan_ready'] = plan


def drain_plan(state):
    """A plan the model has landed, dispatched on the page's own thread; None with none."""
    plan = state.pop('plan_ready', None)
    return None if plan is None else _dispatch(state, plan)


def _dispatch(state, plan):
    """A plan into her body and the chat: the tour begun, else each command sent."""
    rows: list = list(state.get('chat_rows') or ())
    if plan.get('tour'):
        tr = state['tour'] = tour.HouseTour(loop=True)
        entry = (plan['thought'], tr.last_cmd)
        state['llm_last'] = entry
        state['llm_feed'] = (state.get('llm_feed', []) + [entry])[-8:]
        rows.extend([('label', '  cmd: ' + tr.last_cmd), (None, plan['thought'])])
        state['chat_rows'] = rows[-24:]
        return plan
    cancel_tour(state)
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


def refused_told(state, now):
    """A refusal into the chat pane, once per reason: the say's answer lives where the prompt was
    typed - on the status row alone, 25 characters, 'lay on floor' looked like nothing happening."""
    why = (now or {}).get('refused')
    if why and why != state.get('refused_shown'):
        state['refused_shown'] = why
        state['chat_rows'] = (list(state.get('chat_rows') or ()) + [('label', 'REFUSED: %s' % why)])[-24:]


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
    """The side column's boxes: each mode asked and as she is, its spring's N/m; the scene, what
    each digit says (the keys' bar and help say the rest, `humanoid_keys`)."""
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
        ('objective'.rjust(10), SCENARIO_DESC.get(scen, scen).split('//')[-1].strip()[:25]),
    ]
    items = [(DIGITS[i], NAMED.get(s, str(s)).replace('_', ' '))
             for i, s in enumerate(entries[:len(DIGITS)])]
    for idx in range(0, len(items), 2):
        (k1, n1), rest = items[idx], items[idx + 1:idx + 2]
        say_rows.append((('%s / %s' % (k1, rest[0][0]) if rest else k1).rjust(10),
                         ('%-12s %-12s' % (n1[:12], rest[0][1][:12]) if rest else n1[:25]).rstrip()))
    return [hud('LATENT', [_mode_row(name, row) for name, row in zip(SHORT, now.get('modes') or ())]),
            hud('SAY  ' + gym_name.upper(), say_rows)]


#: Her chat's lines, its prompt's among them; its braille's steps a second.
CHAT_ROWS, SPIN_HZ = 4, 8.0
CHAT_GREETING = ('name', 'GYNOID LLM AGENT - press / to prompt, H for autonomous house tour')


def drawer(state, now):
    """The rows her chat takes under the viewport at the clock `now`: CHAT_ROWS and its frame as
    much as is up, slid as ?'s help (`taskbar.slid`) - up while her prompt is open and HELP_S
    after its last line, the greeting's as the page opens."""
    held = state.setdefault('drawer', {'up': 0.0, 'framed': now, 'last': None, 'said': now})
    last = (state.get('chat_rows') or [None])[-1]
    if last is not held['last']:
        held['last'], held['said'] = last, now
    wanted = bool((state.get('llm_prompt') or {}).get('open')) or now - held['said'] < HELP_S
    held['rows'] = round(slid(held, wanted, now) * (CHAT_ROWS + 2))
    return held['rows']


def chat_box(state, width=80):
    """Her chat's transcript and prompt, its lead a braille turning (`recorder.SPIN`)."""
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
    lead = SPIN[int(time.monotonic() * SPIN_HZ) % len(SPIN)]
    body.append('\n')
    body.append('  %s ' % lead, style='value')
    if open_p or entry:
        body.append(entry[:max(1, room - 6)], style='value')
        body.append('_' if (open_p and blink) else ' ', style='value')
    else:
        body.append('press / to talk to the gynoid (e.g. "lie on the bed", "sit in the chair", "tour")'[:max(1, room - 5)],
                    style='label')
    return hud('GYNOID CHAT  LLM / AGENT', [body])
