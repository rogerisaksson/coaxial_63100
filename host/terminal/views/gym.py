"""The HUMANOID page's gym: her verbs, her latent loop's moves said by digit, her modes shown.

    KEYS = gym.table(KEYS)       # the page's keys with the gym's: verbs, digits, scenes, shoves
    gym.offered(now, seen)       # what Space and F say now, the page's prompt
    gym.boxes(now)               # the side column's LATENT and SAY boxes
    gym.status(now)              # what she is doing, in a line

She stands on the loop (`machine.sprung`); the keys that move her are a twist (`humanoid_keys`).
Space, C and F are verbs - a hop, a crouch, settling on or rising from what is near -, each one
an agent would send. A digit says an entry of the gym's text (`sprung.entries`) in place of what
was to come, > takes the gym's other text (`sprung.GYMS`), read anew each time - an edit to
machine/gynoid_mesomorph_medium.lisp is two presses away -; [ ] step through the scenes, Enter
lays the scene again, P N shove her along the view, H tours the scenes. LATENT shows each mode
as it is asked and as she is, and its spring.
"""
import math

from rich.text import Text

from coaxial.draw import gauges
from machine import sprung
from terminal import keybindings
from terminal.ui.stage import hud
from terminal.views import scanner, viewpoint

#: The gym's next text.
NEXT = '>'
#: A digit says the gym's entry of its place: 1 the first, 0 the tenth.
DIGITS = '1234567890'
#: P and N shove her along the view, the bench's 80 N she parries and 200 N she falls to
#: (`tools/sim/scenarios.py`, SHOVES), sprung.SHOVE_S long.
SHOVES_N = {'P': 80.0, 'N': 200.0}
#: A support near enough for F, m from her pelvis over the floor; a chair she hops onto, m
#: ahead and within a quarter turn of her heading; her pelvis over this, m, on a chair's seat.
REACH_M, ONTO_M, ON_CHAIR_M = 1.2, 0.7, 1.12
#: The crouch's phase (`postures_base.lisp`).
CROUCHED = 'down'
#: Each mode's name in the box; what each stream payload is called on the page.
SHORT = ('axial', 'lateral', 'drive', 'pitch', 'sagittal', 'lift', 'width', 'list', 'turn', 'arms', 'rock')
NAMED = {
    '#201': '#201 stand', '#202': '#202 sway', '#203': '#203 step', '#204': '#204 walk', '#205': '#205 back',
    'weight_shift': 'weight shift', 'bow': 'bow', 'ballistic_walk': 'ballistic walk',
    'sit_chair': 'sit chair', 'rise_chair': 'rise chair', 'lie_bed': 'lie bed', 'rise_bed': 'rise bed',
    'lie_floor': 'lie floor', 'rise_floor': 'rise floor', 'hop_down_chair': 'hop down chair',
    'hop_up_chair': 'hop up chair', 'strafe_left': 'sidestep left', 'strafe_right': 'sidestep right',
    'turn_left': 'turn left', 'turn_right': 'turn right',
    'turn_back_left': 'turn back left', 'turn_back_right': 'turn back right',
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
    return '#201' if (doing == CROUCHED or now.get('fallen')) else None


def offered(now, seen):
    """{key: verb} Space and F say now: `now` her state, `seen` the scanner's detections round
    her, nearest first (`scanner.scan`'s, `reach`). Space a hop - down from a chair's seat, up
    onto a chair close ahead -; F up from what bears her, else onto the support nearest."""
    if not now:
        return {}
    x, _y, z = now.get('where') or (0.0, 0.9, 0.0)
    heading = float(now.get('heading') or 0.0)
    ahead = [d for d in seen if d['kind'] == 'chair' and d['dist'] < ONTO_M
             and (d['pos'][0] - x) * math.sin(heading) + (d['pos'][2] - z) * math.cos(heading) > 0.5 * d['dist']]
    rise = contextual_rise(now)
    near = next((scanner.PRIMARY_ACTION[d['kind']] for d in seen
                 if d['dist'] < REACH_M and d['kind'] in scanner.PRIMARY_ACTION), None)
    hop = 'hop_down_chair' if rise == 'hop_down_chair' else ('hop_up_chair' if ahead else 'hop')
    return {' ': hop, 'f': rise or near} if (rise or near) else {' ': hop}


def touring(state):
    """Whether the tour (`machine.tour`) is commanding her."""
    tour = state.get('tour')
    return tour is not None and not tour.done


def _cancel_tour(state):
    """The tour ended by a key that acts on her; the keys' twist sent again on the next frame."""
    if touring(state):
        state['tour'] = None
        state.setdefault('pad', {})['setpoint'] = None


def _said(state, verb):
    """`verb` said, the tour and any held pad keys ended: a gym entry's name or place."""
    _cancel_tour(state)
    pad = state.get('pad')
    if isinstance(pad, dict):
        pad['keys'] = dict.fromkeys(pad.get('keys') or ('w', 's', 'a', 'd', 'q', 'e', 'shift'), False)
        pad['last_seen'], pad['setpoint'] = {}, (0.0, 0.0, 0.0)
    if verb is not None:
        state['body'].send(say=verb)


def _offer(key):
    def say(state):
        seen = state['scanner'].scan(state.get('now'), reach=REACH_M) if state.get('scanner') else []
        _said(state, offered(state.get('now'), seen).get(key))
    return say


def _crouched(state):
    """C: a crouch, or up from one."""
    now = state.get('now') or {}
    _said(state, '#201' if now.get('doing') == CROUCHED else 'crouch')


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
    """H: the tour from its start, or ended."""
    from machine.tour import HouseTour
    if touring(state):
        _cancel_tour(state)
        return
    state['tour'] = HouseTour(loop=True)


def table(keys):
    """`keys` (the page's own) with the gym's laid over them: verbs, digits, scenes, shoves."""
    out = dict(keys)
    out[NEXT] = _next
    for k in ('<', ',', ']'):
        out[k] = lambda state: _step_scenario(state, 1)
    out['['] = lambda state: _step_scenario(state, -1)
    for k in ('\r', '\n'):
        out[k] = lambda state: _run_scenario(state)
    out.update((k, lambda state, i=i: _said(state, i)) for i, k in enumerate(DIGITS))
    out.update((k, _shoved(n)) for k, n in SHOVES_N.items())
    out[' '] = _offer(' ')
    for k in 'fF':
        out[k] = _offer('f')
    for k in 'cC':
        out[k] = _crouched
    out['H'] = _toggle_tour
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
        ('wasd / shf'.rjust(10), 'go from the view / sprint'),
        ('q e / spc'.rjust(10), 'sidestep / hop'),
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
