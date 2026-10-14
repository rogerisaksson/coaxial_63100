"""The HUMANOID page's gym: her latent loop's moves said by digit, her modes shown.

    KEYS = gym.table(KEYS)       # the page's keys with the gym's: the digits, >, W, P and N
    gym.boxes(now)               # the side column's LATENT and SAY boxes
    gym.status(now)              # what she is doing, in a line

She stands on the loop (`machine.sprung`) and walks from rest (`sprung.WALK`). A digit says an
entry of the gym's text (`sprung.entries`) in place of what was to come - a payload injected, a
behaviour begun -, > takes the gym's other text (`sprung.GYMS`), read anew each time - an edit
to machine/gynoid_mesomorph_medium.lisp is two presses away -, W walks her from rest again, P and N
shove her, A lands her anew. LATENT shows each mode as it is asked and as she is, and its spring.
"""
import sys

from rich.text import Text

from coaxial.draw import gauges
from machine import se2, sprung
from terminal import keybindings
from terminal.ui.stage import hud

#: The gym's next text; her walk from rest.
NEXT, WALKS = '>', 'wW'
#: A digit says the gym's entry of its place: 1 the first, 0 the tenth.
DIGITS = '1234567890'
#: P and Shift+P apply moderate shoves (80 N lateral / fore-aft, parried); Shift+N applies a
#: severe shove (140 N, triggering parry attempt and controlled breakfall).
PUSHES = {'p': ('push', False), 'P': ('push', True), 'N': ('severe', False)}
#: Each mode's name in the box; what each stream payload is called on the page.
SHORT = ('axial', 'lateral', 'drive', 'pitch', 'sagittal', 'lift', 'width', 'list', 'turn', 'arms', 'rock')
NAMED = {
    '#201': '#201 stand', '#202': '#202 sway', '#203': '#203 step', '#204': '#204 walk', '#205': '#205 back',
    'weight_shift': 'weight shift', 'bow': 'bow', 'ballistic_walk': 'ballistic walk',
    'sit_chair': 'sit chair', 'rise_chair': 'rise chair', 'lie_bed': 'lie bed', 'rise_bed': 'rise bed',
    'lie_floor': 'lie floor', 'rise_floor': 'rise floor', 'hop_down_chair': 'hop down chair',
    'hop_up_chair': 'hop up chair', 'stairs_walk': 'stairs walk', 'stairs_down': 'stairs down',
    'ice_walk': 'ice walk',
    'slope_walk': 'slope walk', 'parry_shove': 'parry shove', 'controlled_fall': 'controlled fall',
    'strafe_left': 'strafe left (A)', 'strafe_right': 'strafe right (D)',
    'turn_left': 'turn left (WA)', 'turn_right': 'turn right (WD)',
    'circle_walk': 'circle walk (Q)',
    'turn_back_left': 'turn back left (SA)', 'turn_back_right': 'turn back right (SD)',
}
#: Named keys to trigger stunts directly from the terminal.
STUNTS = keybindings.STUNT_KEYS

SCENARIO_MOVES = keybindings.SCENARIO_MOVES
SCENARIO_DESC = keybindings.SCENARIO_DESC

_SCENARIO_RISE = {
    'chair': (('approach', 'lower', 'seated', 'sit_chair'), 'rise_chair'),
    'bed': (('sit_edge', 'recline', 'bed_rest', 'lie_bed'), 'rise_bed'),
    'mattress': (('kneel', 'lower', 'floor_rest', 'lie_floor'), 'rise_floor'),
}


def _cancel_tour(state):
    """Disengage automated house tour on manual operator intervention (ignoring tour's own dispatches)."""
    if state.get('tour') is None or state.get('_in_tour_step'):
        return
    frame = sys._getframe(1)
    for _ in range(4):
        if frame is None:
            break
        if frame.f_code.co_name == '_dispatch_key':
            return
        frame = frame.f_back
    state['tour'] = None


def _clear_se2(state):
    """Reset active SE(2) WASD velocity state so non-WASD actions are not overwritten on the next draw."""
    state['_wasd_active'] = False
    se2_st = state.get('se2')
    if se2_st is not None:
        se2_st['v_x'] = se2_st['v_y'] = se2_st['omega_z'] = 0.0
        se2_st['setpoint'] = (0.0, 0.0, 0.0)
        se2_st['actual'] = (0.0, 0.0, 0.0)
        se2_st['gait'] = None
        se2_st['keys'] = {'w': False, 's': False, 'a': False, 'd': False, 'q': False, 'e': False, 'shift': False}
        se2_st['last_seen'].clear()
    state['_last_sent_twist'] = (0.0, 0.0, 0.0)


def _run_scenario(state, scen=None):
    _cancel_tour(state)
    _clear_se2(state)
    is_replay = scen is None
    if scen is None:
        scen = state.get('scenario', 'flat')
    now = state.get('now')
    if is_replay and now and state.get('scenario') == scen:
        doing = now.get('doing', '')
        where = now.get('where') or (0.0, 0.90, 0.0)
        if scen == 'chair' and (where[1] > 1.12 or doing == 'seat_land'):
            state['body'].send(say='hop_down_chair')
            return
        if scen in _SCENARIO_RISE and doing in _SCENARIO_RISE[scen][0]:
            state['body'].send(say=_SCENARIO_RISE[scen][1])
            return
    state['scenario'] = scen
    state['body'].send(scenario=scen)
    mv = SCENARIO_MOVES.get(scen)
    if mv:
        state['body'].send(say=mv)


def _next(state):
    state['gym'] = sprung.GYMS[(sprung.GYMS.index(state.get('gym', sprung.GYMS[0])) + 1) % len(sprung.GYMS)]
    state['body'].send(gym=state['gym'])


def _step_scenario(state, delta=1):
    from machine import floor
    scens = floor.SCENARIOS
    cur = state.get('scenario', 'flat')
    nxt = scens[(scens.index(cur) + delta) % len(scens)] if cur in scens else scens[0]
    _run_scenario(state, nxt)


def _next_scenario(state):
    _step_scenario(state, 1)


def _prev_scenario(state):
    _step_scenario(state, -1)


def _shoved(event, along):
    def shove(state):
        _cancel_tour(state)
        _clear_se2(state)
        state['side'] = -state.get('side', 1.0)
        mag = 140.0 if event == 'severe' else sprung.SHOVES[event]
        n = mag * state['side']
        state['body'].send(push=(0.0, 0.0, n) if along else (n, 0.0, 0.0), seconds=sprung.SHOVE_S)
    return shove


def _say_non_wasd(state, target):
    _cancel_tour(state)
    _clear_se2(state)
    state['body'].send(say=target)


def _toggle_wasd(state, key='~'):
    import time
    now = time.monotonic()
    last_key, last_t = state.get('_last_wasd_toggle', ('', 0.0))
    if key == last_key and (now - last_t) < 0.15:
        return
    state['_last_wasd_toggle'] = (key, now)
    state['_last_wasd'] = (None, 0.0)
    _clear_se2(state)
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
    _cancel_tour(state)
    if state.get('wasd_mode'):
        se2_st = state.setdefault('se2', se2.state())
        keys_st = se2_st.get('keys', {})
        if keys_st.get('a') or keys_st.get('q'):
            state['body'].send(say='turn_left')
        elif keys_st.get('d') or keys_st.get('e'):
            state['body'].send(say='turn_right')
        elif shifted or keys_st.get('shift') or se2_st.get('v_x', 0.0) > se2.WALK_SPEED + 0.01:
            state['body'].send(say='jog')
        else:
            state['body'].send(say=3)
    else:
        state['body'].send(walk=True)


def _key_s(state):
    _cancel_tour(state)
    now = state.get('now')
    is_moving = now and (now.get('stage') == 'walk' or abs(now.get('speed', 0.0)) > 0.05)
    if state.get('wasd_mode') or is_moving:
        keys_st = (state.get('se2') or {}).get('keys', {}) if state.get('wasd_mode') else {}
        if keys_st.get('a') or keys_st.get('q'):
            state['body'].send(say='turn_back_left')
        elif keys_st.get('d') or keys_st.get('e'):
            state['body'].send(say='turn_back_right')
        else:
            state['body'].send(say=4)
    else:
        state['body'].send(say='sit_chair')


def _key_a(state):
    _cancel_tour(state)
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
    _cancel_tour(state)
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


def _key_q(state):
    _cancel_tour(state)
    if state.get('wasd_mode'):
        keys_st = (state.get('se2') or {}).get('keys', {})
        if keys_st.get('w'):
            state['body'].send(say='turn_left')
        elif keys_st.get('s'):
            state['body'].send(say='turn_back_left')
        elif keys_st.get('a'):
            state['body'].send(say='strafe_left')
        elif keys_st.get('d'):
            state['body'].send(say='strafe_right')
        else:
            state['body'].send(say='turn_left')
    else:
        _say_non_wasd(state, 'circle_walk')


def _key_e(state):
    _cancel_tour(state)
    if state.get('wasd_mode'):
        keys_st = (state.get('se2') or {}).get('keys', {})
        if keys_st.get('w'):
            state['body'].send(say='turn_right')
        elif keys_st.get('s'):
            state['body'].send(say='turn_back_right')
        elif keys_st.get('a'):
            state['body'].send(say='strafe_left')
        elif keys_st.get('d'):
            state['body'].send(say='strafe_right')
        else:
            state['body'].send(say='turn_right')
    else:
        _say_non_wasd(state, 'lie_bed')


def _key_f(state):
    _cancel_tour(state)
    _clear_se2(state)
    if state.get('wasd_mode'):
        from terminal.views import scanner
        sc = state.get('scanner')
        now = state.get('now')
        doing = (now or {}).get('doing', '')
        where = (now or {}).get('where') or (0.0, 0.90, 0.0)
        if where[1] > 1.12 or doing == 'seat_land':
            state['body'].send(say='hop_down_chair')
            return
        if doing in ('approach', 'lower', 'seated', 'sit_chair') and state.get('scenario') != 'mattress':
            state['body'].send(say='rise_chair')
            return
        if doing in ('sit_edge', 'recline', 'bed_rest', 'lie_bed'):
            state['body'].send(say='rise_bed')
            return
        if (doing in ('kneel', 'lower', 'floor_rest', 'lie_floor', 'cushion', 'land')
                or (now and now.get('stage') in ('down', 'fallen', 'floor'))):
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
    _cancel_tour(state)
    if state.get('wasd_mode'):
        se2_st = state.get('se2') or {}
        gait = se2_st.get('gait')
        state['body'].send(say=gait if gait in se2.WASD_GAITS else 'jog')
    else:
        state['body'].send(say='ballistic_walk')


def _toggle_tour(state):
    from machine.tour import HouseTour
    _clear_se2(state)
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
    out[']'] = _next_scenario
    out['['] = _prev_scenario
    out['\r'] = lambda state: _run_scenario(state)
    out['\n'] = lambda state: _run_scenario(state)
    out['up'] = lambda state: _say_non_wasd(state, 3)
    out['down'] = lambda state: _say_non_wasd(state, 4)
    out['left'] = lambda state: _say_non_wasd(state, 'strafe_left' if state.get('wasd_mode') else 1)
    out['right'] = lambda state: _say_non_wasd(state, 'strafe_right' if state.get('wasd_mode') else 1)
    out.update((k, lambda state, i=i: _say_non_wasd(state, i)) for i, k in enumerate(DIGITS))
    out.update((k, _shoved(*how)) for k, how in PUSHES.items())
    for name, key in STUNTS.items():
        handler = (lambda state, n=name: _say_non_wasd(state, n))
        out[key] = handler
        if key.isalpha() and key.upper() not in PUSHES:
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
    for k in 'qQ':
        out[k] = _key_q
    for k in 'eE':
        out[k] = _key_e
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


def _mode_row(name, row):
    ref, z, k, _d = float(row[1]), float(row[2]), float(row[3]), float(row[4])
    s_ref = 0.5 + 0.5 * max(-1.0, min(1.0, ref))
    s_act = 0.5 + 0.5 * max(-1.0, min(1.0, z))
    bar = gauges.gauge(s_act, 8, centre=0.5, marks=((s_ref, gauges.MARK),))
    return (name.rjust(10), Text.from_ansi('%+5.2f %s %+5.2f %4.0f' % (ref, bar, z, k)))


_SHORT_SAY = {'ballistic walk': 'ballistic wk', 'hop down chair': 'hop dn chair', 'controlled fall': 'breakfall'}


def boxes(now, wasd=False, tour=None):
    """The side column's boxes: each mode asked and as she is, its spring's N/m; what each
    digit and stunt key says in a unified two-column instrument layout."""
    if not now:
        return []
    says = now.get('says') or ('gynoid_mesomorph_medium',)
    gym_name, entries = says[0], says[1:]
    scen = now.get('scenario', 'flat')
    scen_mv = SCENARIO_MOVES.get(scen, '')
    scen_label = '%s (%s)' % (scen, NAMED.get(scen_mv, scen_mv)) if scen_mv else scen
    scen_obj = SCENARIO_DESC.get(scen, scen).split('//')[-1].strip()
    wasd_on = wasd or now.get('wasd_mode', False)
    tour_on = tour is not None and not getattr(tour, 'done', False)
    mode_badge = '[AUTO: HOUSE TOUR]' if tour_on else ('TPW active (§/~)' if wasd_on else 'normal (§/~ for TPW)')
    say_rows = [
        ('control'.rjust(10), mode_badge),
        ('scenario'.rjust(10), scen_label[:25]),
        ('objective'.rjust(10), scen_obj[:25]),
        ('[ ] / ent'.rjust(10), 'cycle / run scenario'),
    ]
    if wasd_on:
        say_rows.append(('wasd / shf'.rjust(10), 'move, arc / sprint'))
        say_rows.append(('spc / f'.rjust(10), 'jump / interact prop'))
    else:
        say_rows.append(('arrows / w'.rjust(10), 'walk, back, sway'))
        say_rows.append(('spc / p, N'.rjust(10), 'charge push / shove'))
    seen, items = set(), []
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
            raw_lbl = NAMED.get(s, s).split(' (')[0].replace('_', ' ')
            items.append((key, _SHORT_SAY.get(raw_lbl, raw_lbl)))
    for idx in range(0, len(items), 2):
        k1, n1 = items[idx]
        if idx + 1 < len(items):
            k2, n2 = items[idx + 1]
            say_rows.append((('%s / %s' % (k1, k2)).rjust(10), ('%-12s %-12s' % (n1[:12], n2[:12])).rstrip()))
        else:
            say_rows.append((k1.rjust(10), n1[:25]))
    return [hud('LATENT', [_mode_row(name, row) for name, row in zip(SHORT, now.get('modes') or ())]),
            hud('SAY  ' + gym_name.upper(), say_rows)]


