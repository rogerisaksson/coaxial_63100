"""The HUMANOID page's gym: her latent loop's moves said by digit, her modes shown.

    KEYS = gym.table(KEYS)       # the page's keys with the gym's: the digits, >, W, P and N
    gym.boxes(now)               # the side column's LATENT and SAY boxes
    gym.status(now)              # what she is doing, in a line

She stands on the loop (`machine.sprung`) and walks from rest (`sprung.WALK`). A digit says an
entry of the gym's text (`sprung.entries`) in place of what was to come - a payload injected, a
behaviour begun -, > takes the gym's other text (`sprung.GYMS`), read anew each time - an edit
to machine/moves.lisp is two presses away -, W walks her from rest again, P and N shove her, A
lands her anew. LATENT shows each mode as it is asked and as she is, and its spring.
"""
from machine import sprung
from terminal.ui.stage import hud

#: The gym's next text; her walk from rest.
NEXT, WALKS = '>', 'wW'
#: A digit says the gym's entry of its place: 1 the first, 0 the tenth.
DIGITS = '1234567890'
#: P and N shove her (`sprung.SHOVES`), from each side in turn; shifted, from behind and ahead
#: in turn.
PUSHES = {'p': ('push', False), 'n': ('nudge', False), 'P': ('push', True), 'N': ('nudge', True)}
#: Each mode's name in the box; what each stream payload is called on the page.
SHORT = ('axial', 'lateral', 'drive', 'pitch', 'sagittal', 'lift', 'width', 'list', 'turn', 'arms', 'rock')
NAMED = {
    '#201': '#201 stand', '#202': '#202 sway', '#203': '#203 step', '#204': '#204 walk', '#205': '#205 back',
    'weight_shift': 'weight shift', 'bow': 'bow', 'ballistic_walk': 'ballistic walk',
    'sit_chair': 'sit chair', 'rise_chair': 'rise chair', 'lie_bed': 'lie bed', 'rise_bed': 'rise bed',
    'lie_floor': 'lie floor', 'rise_floor': 'rise floor', 'hop_down_chair': 'hop down chair',
    'hop_up_chair': 'hop up chair', 'stairs_walk': 'stairs walk', 'ice_walk': 'ice walk',
    'slope_walk': 'slope walk', 'parry_shove': 'parry shove', 'controlled_fall': 'controlled fall',
    'strafe_left': 'strafe left (A)', 'strafe_right': 'strafe right (D)',
    'turn_left': 'turn left (WA)', 'turn_right': 'turn right (WD)',
    'turn_back_left': 'turn back left (SA)', 'turn_back_right': 'turn back right (SD)',
}
#: Named keys to trigger stunts directly from the terminal.
STUNTS = {
    'weight_shift': 'h',
    'bow': ';',
    'ballistic_walk': 'b',
    'sit_chair': 's',
    'rise_chair': 'u',
    'lie_bed': 'e',
    'rise_bed': 'g',
    'lie_floor': 'f',
    'rise_floor': 'k',
    'hop_down_chair': 'd',
    'hop_up_chair': 'j',
    'stairs_walk': 'x',
    'ice_walk': 'i',
    'slope_walk': 'm',
    'parry_shove': 'y',
    'controlled_fall': 'z',
}


SCENARIO_MOVES = {
    'flat': '#201', 'chair': 'sit_chair', 'bed': 'lie_bed', 'slope': 'slope_walk',
    'stairs': 'stairs_walk', 'ice': 'ice_walk', 'snow': '#204', 'obstacle': '#204',
    'sill': '#204', 'hole': '#204', 'rug': '#204',
}


def _run_scenario(state, scen=None):
    if scen is None:
        scen = state.get('scenario', 'flat')
    state['scenario'] = scen
    state['body'].send(scenario=scen)
    mv = SCENARIO_MOVES.get(scen)
    if mv:
        state['body'].send(say=mv)


def _next(state):
    state['gym'] = sprung.GYMS[(sprung.GYMS.index(state.get('gym', sprung.GYMS[0])) + 1) % len(sprung.GYMS)]
    state['body'].send(gym=state['gym'])


def _next_scenario(state):
    from machine import floor
    scens = floor.SCENARIOS
    cur = state.get('scenario', 'flat')
    nxt = scens[(scens.index(cur) + 1) % len(scens)] if cur in scens else scens[0]
    _run_scenario(state, nxt)


def _shoved(event, along):
    def shove(state):
        state['side'] = -state['side']
        n = sprung.SHOVES[event] * state['side']
        state['body'].send(push=(0.0, 0.0, n) if along else (n, 0.0, 0.0), seconds=sprung.SHOVE_S)
    return shove


def table(keys):
    """`keys` (the page's own) with the gym's laid over them: arrows and WASD walk, back and strafe, digits say moves."""
    out = dict(keys)
    out[NEXT] = _next
    out['<'] = _next_scenario
    out[','] = _next_scenario
    out['\r'] = lambda state: _run_scenario(state)
    out['\n'] = lambda state: _run_scenario(state)
    out['up'] = lambda state: state['body'].send(say=3)      # #204 gentle walk forward
    out['down'] = lambda state: state['body'].send(say=4)    # #205 step backward / brake
    out['left'] = lambda state: state['body'].send(say='strafe_left' if state.get('wasd_mode') else 1)
    out['right'] = lambda state: state['body'].send(say='strafe_right' if state.get('wasd_mode') else 1)
    out.update((k, lambda state, i=i: state['body'].send(say=i)) for i, k in enumerate(DIGITS))
    out.update((k, _shoved(*how)) for k, how in PUSHES.items())
    for name, key in STUNTS.items():
        handler = (lambda state, n=name: state['body'].send(say=n))
        out[key] = handler
        if key.isalpha():
            out[key.upper()] = handler

    # Quake WASD & toggle handlers
    def _toggle_wasd(state):
        state['wasd_mode'] = not state.get('wasd_mode', False)

    def _key_w(state):
        if state.get('wasd_mode'):
            state['body'].send(say=3)
        else:
            state['body'].send(walk=True)

    def _key_s(state):
        if state.get('wasd_mode'):
            state['body'].send(say=4)
        else:
            state['body'].send(say='sit_chair')

    def _key_a(state):
        if state.get('wasd_mode'):
            state['body'].send(say='strafe_left')
        else:
            state['body'].send(restart=True)

    def _key_d(state):
        if state.get('wasd_mode'):
            state['body'].send(say='strafe_right')
        else:
            state['body'].send(say='hop_down_chair')

    for k in ('~', '`', '§', '½'):
        out[k] = _toggle_wasd
    for k in 'wW':
        out[k] = _key_w
    for k in 'sS':
        out[k] = _key_s
    for k in 'aA':
        out[k] = _key_a
    for k in 'dD':
        out[k] = _key_d
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


def boxes(now, wasd=False):
    """The side column's boxes: each mode asked and as she is, its spring's N/m and N s/m; what
    each digit and stunt key says."""
    if not now:
        return []
    says = now['says']
    gym_name = says[0]
    entries = says[1:]
    scen = now.get('scenario', 'flat')
    scen_mv = SCENARIO_MOVES.get(scen, '')
    scen_label = 'scen: %s [%s]' % (scen, NAMED.get(scen_mv, scen_mv)) if scen_mv else 'scen: ' + scen
    wasd_on = wasd or now.get('wasd_mode', False)
    say_rows = [('~ / §', 'WASD: %s' % ('ON' if wasd_on else 'OFF')), ('<', scen_label)]
    if wasd_on:
        say_rows.append(('WASD', 'fwd/back/strafe'))
        say_rows.append(('WA/SD', 'curves/turns'))
    else:
        say_rows.append(('^/v', 'fwd / back'))
    seen = set()
    for i, s in enumerate(entries):
        if s.startswith('#'):
            key = DIGITS[i] if i < len(DIGITS) else str(i)
        elif s in STUNTS:
            key = STUNTS[s]
        elif i < len(DIGITS) and s != 'init_walk':
            key = DIGITS[i]
        else:
            continue
        if s not in seen:
            seen.add(s)
            say_rows.append((key, NAMED.get(s, s)))
    return [hud('LATENT', [(name, '%+.2f is %+.2f %6.0f %5.0f' % row[1:])
                           for name, row in zip(SHORT, now['modes'])]),
            hud('SAY  ' + gym_name.upper(), say_rows)]
