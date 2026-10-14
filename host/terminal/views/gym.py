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
from machine import se2, sprung
from terminal import keybindings
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
    'hop_up_chair': 'hop up chair', 'stairs_walk': 'stairs walk', 'stairs_down': 'stairs down (N)',
    'ice_walk': 'ice walk',
    'slope_walk': 'slope walk', 'parry_shove': 'parry shove', 'controlled_fall': 'controlled fall',
    'strafe_left': 'strafe left (A)', 'strafe_right': 'strafe right (D)',
    'turn_left': 'turn left (WA)', 'turn_right': 'turn right (WD)',
    'circle_walk': 'circle walk (Q)',
    'turn_back_left': 'turn back left (SA)', 'turn_back_right': 'turn back right (SD)',
}
#: Named keys to trigger stunts directly from the terminal.
STUNTS = keybindings.STUNT_KEYS


SCENARIO_MOVES = {
    'flat': '#201', 'chair': 'sit_chair', 'bed': 'lie_bed', 'slope': 'slope_walk',
    'stairs': 'stairs_walk', 'ice': 'ice_walk', 'snow': '#204', 'obstacle': '#204',
    'sill': '#204', 'hole': '#204', 'rug': '#204', 'glass_room': 'circle_walk',
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


def _say_non_wasd(state, target):
    state['_wasd_active'] = False
    se2_st = state.get('se2')
    if se2_st is not None:
        se2_st['v_x'] = se2_st['v_y'] = se2_st['omega_z'] = 0.0
        se2_st['setpoint'] = (0.0, 0.0, 0.0)
        se2_st['actual'] = (0.0, 0.0, 0.0)
        se2_st['gait'] = None
        se2_st['keys'] = {'w': False, 's': False, 'a': False, 'd': False, 'shift': False}
        se2_st['last_seen'].clear()
    state['_last_sent_twist'] = (0.0, 0.0, 0.0)
    state['body'].send(say=target)


def _toggle_wasd(state, key='~'):
    import time
    now = time.monotonic()
    last_key, last_t = state.get('_last_wasd_toggle', ('', 0.0))
    if key == last_key and (now - last_t) < 0.15:
        return
    state['_last_wasd_toggle'] = (key, now)
    state['_last_wasd'] = (None, 0.0)
    was_on = state.get('wasd_mode', False)
    state['wasd_mode'] = not was_on
    if was_on:
        from terminal.views import scanner
        sc = state.get('scanner') or scanner.VisionScanner()
        detected = sc.scan(state.get('now'), state.get('yaw', 0.0))
        action = scanner.auto_action(detected)
        if action:
            state['body'].send(say=action)


def _key_w(state, shifted=False):
    if state.get('wasd_mode'):
        se2_st = state.setdefault('se2', se2.state())
        keys_st = se2_st.get('keys', {})
        if keys_st.get('a'):
            state['body'].send(say='turn_left')
        elif keys_st.get('d'):
            state['body'].send(say='turn_right')
        elif shifted or keys_st.get('shift') or se2_st.get('v_x', 0.0) > se2.WALK_SPEED + 0.01:
            state['body'].send(say='jog')
        else:
            state['body'].send(say=3)
    else:
        state['body'].send(walk=True)


def _key_s(state):
    now = state.get('now')
    is_moving = now and (now.get('stage') == 'walk' or abs(now.get('speed', 0.0)) > 0.05)
    if state.get('wasd_mode') or is_moving:
        keys_st = (state.get('se2') or {}).get('keys', {}) if state.get('wasd_mode') else {}
        if keys_st.get('a'):
            state['body'].send(say='turn_back_left')
        elif keys_st.get('d'):
            state['body'].send(say='turn_back_right')
        else:
            state['body'].send(say=4)
    else:
        state['body'].send(say='sit_chair')


def _key_a(state):
    if state.get('wasd_mode'):
        keys_st = (state.get('se2') or {}).get('keys', {})
        if keys_st.get('w'):
            state['body'].send(say='turn_left')
        elif keys_st.get('s'):
            state['body'].send(say='turn_back_left')
        else:
            state['body'].send(say='strafe_left')
    elif 'wasd_mode' not in state:
        state['body'].send(restart=True)


def _key_d(state):
    if state.get('wasd_mode'):
        keys_st = (state.get('se2') or {}).get('keys', {})
        if keys_st.get('w'):
            state['body'].send(say='turn_right')
        elif keys_st.get('s'):
            state['body'].send(say='turn_back_right')
        else:
            state['body'].send(say='strafe_right')
    else:
        state['body'].send(say='hop_down_chair')


def _key_f(state):
    if state.get('wasd_mode'):
        from terminal.views import scanner
        sc = state.get('scanner')
        now = state.get('now')
        doing = (now or {}).get('doing', '')
        if doing in ('seated', 'sit_chair'):
            state['body'].send(say='rise_chair')
            return
        if doing in ('bed_rest', 'lie_bed'):
            state['body'].send(say='rise_bed')
            return
        if doing in ('floor_rest', 'lie_floor') or (now and now.get('stage') in ('down', 'fallen', 'floor')):
            state['body'].send(say='rise_floor')
            return
        if doing == 'down':
            state['body'].send(say='#201')
            return
        if sc and now:
            detected = sc.scan(now, state.get('yaw', 0.0))
            act = scanner.auto_action(detected)
            if act:
                state['body'].send(say=act)
                return
    state['body'].send(say='lie_floor')


def _key_shift(state):
    if state.get('wasd_mode'):
        se2_st = state.get('se2') or {}
        gait = se2_st.get('gait')
        state['body'].send(say=gait if gait in se2.WASD_GAITS else 'jog')
    else:
        state['body'].send(say='ballistic_walk')


def _toggle_tour(state):
    from machine.tour import HouseTour
    if state.get('tour') is not None and not state['tour'].done:
        state['tour'] = None
    else:
        state['scenario'] = 'glass_room'
        state['body'].send(scenario='glass_room')
        state['tour'] = HouseTour(loop=True)


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
    out.update((k, lambda state, i=i: _say_non_wasd(state, i)) for i, k in enumerate(DIGITS))
    out.update((k, _shoved(*how)) for k, how in PUSHES.items())
    for name, key in STUNTS.items():
        handler = (lambda state, n=name: _say_non_wasd(state, n))
        out[key] = handler
        if key.isalpha():
            out[key.upper()] = handler
    for k in keybindings.TOGGLE_TPW_KEYS:
        out[k] = (lambda state, key=k: _toggle_wasd(state, key))
    out['w'] = lambda state: _key_w(state, False)
    out['W'] = lambda state: _key_w(state, True)
    for k in 'sS':
        out[k] = _key_s
    for k in 'aA':
        out[k] = _key_a
    for k in 'dD':
        out[k] = _key_d
    for k in 'fF':
        out[k] = _key_f
    for k in ('shift', 'Shift', 'sprint'):
        out[k] = _key_shift
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
    mode_badge = '>>> TPW ACTIVE <<<' if wasd_on else '[OFF: VISION / STUNTS]'
    say_rows = [
        ('CONTROL', mode_badge),
        ('TOGGLE', '§ / ~ (eller F2)'),
        ('<', scen_label),
    ]
    if wasd_on:
        say_rows.append(('W / S', 'body fwd ramp / back'))
        say_rows.append(('A / D', 'pure strafe L / R'))
        say_rows.append(('WA / WD', 'arc turn L / R'))
        say_rows.append(('SHIFT', 'sprint / jog'))
        say_rows.append(('F KEY', 'interact (sit/lie/rise)'))
        say_rows.append(('SPACE', 'jump in stride / hop'))
        say_rows.append(('MOUSE', 'free-orbit camera'))
    else:
        say_rows.append(('^ / v', 'fwd / back'))
        say_rows.append(('A key', 'off (switch with § / ~)'))
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
