"""The HUMANOID page's hands: its mouse on the view, its keys on her loop and on the view."""
import sys
from types import SimpleNamespace as SN

from tools.dev.focus import chosen
from views_kit import Report


def test_the_humanoid_pages_mouse(report):
    """The HUMANOID page holds the mouse, a key typed kept (`viewpoint`)."""
    from terminal.ui import console
    from terminal.views import viewpoint
    keys = console.Keys(console=True, mouse=True, select=frozenset(), pan=True)
    esc = chr(27)
    keys._buffer = esc + '[<2;5;20M' + esc + '[<34;9;23M' + esc + '[<34;12;26M' + 'f'
    leave, zoom = keys.poll()
    report.check('a right drag moves by its cells, zooms nothing, F typed',
                 leave is None and zoom == 0.0 and keys.panned() == (7, 6) and keys.taken() == ['f'])
    keys._buffer = esc + '[<2;5;20M' + esc + '[200~qa'
    first = keys.poll()
    keys._buffer += 'A' + esc + '[201~' + 's'
    report.check('a paste dropped whole, split over two reads, the key after it kept',
                 first[0] is None and keys.poll()[0] is None and keys.taken() == ['s'])
    state: dict = dict(viewpoint.HOME, orbit=False, last_t=None)
    viewpoint.moved(state, 10, 0, 120, 50)
    viewpoint.turned(state, 5, 2)
    viewpoint.KEYS['up'](state)
    report.check('the view moved against the drag, turned and tipped, lifted',
                 state['pan'][0] < 0.0 and state['pan'][1] > 0.0
                 and state['yaw'] > viewpoint.YAW and state['pitch'] > viewpoint.PITCH)
    viewpoint.KEYS['v'](state)
    report.check('V home starts straight behind at yaw 180',
                 viewpoint.YAW == 180.0 and all(state[k] == v for k, v in viewpoint.HOME.items()))
    viewpoint.mouse(state, lambda: (120, 50))['on_drag'](5, 0)
    report.check('a left drag to the right turns her the way the left arrow does', state['yaw'] < viewpoint.YAW)
    sgr = [('\x1b', 27, 1), ('[', 219, 1), ('<', 226, 1), ('0', 48, 1), (';', 188, 1),
           ('5', 53, 2), (';', 188, 1), ('2', 50, 2), ('M', 77, 1),
           ('\x1b', 27, 1), ('[', 219, 1), ('<', 226, 1), ('0', 48, 1), (';', 188, 1),
           ('5', 53, 2), (';', 188, 1), ('2', 50, 2), ('m', 77, 1)]
    keys._buffer = ''.join(keys._key_of(SN(ch=c, vk=v, repeat=r, down=1)) for c, v, r in sgr)
    lv_c, _ = keys.poll()
    keys._buffer = ''.join(sum((keys._mouse_of(SN(pos=SN(X=x, Y=10), buttons=b, flags=console.MOUSE_MOVED))
                                for x, b in ((10, 1), (15, 1), (18, 0))), []))
    keys.poll()
    d_ok = (keys.clicked() == [(55, 22), (11, 11)] and keys.dragged() == (5, 0)
            and keys._buttons == 0 and not keys._holding)
    keys._grabbed, keys._holding, keys._buttons, keys._held_at = True, True, 1, 0.0
    keys._hold = lambda: setattr(keys, '_held_at', 999.0)
    keys.poll()
    report.check('left click via VT KEY_EVENT_RECORDs, MOUSE_MOVED drag/release, and HOLD_S stuck-drag reset work cleanly',
                 lv_c is None and d_ok and keys._buttons == 0 and not keys._holding and keys._held_at == 999.0)


def test_the_humanoid_pages_keys(report):
    """The page's keys are her loop's and the view's: A lands her, digits say moves, W walks her."""
    from machine import se2
    from terminal.views import gym, humanoid_keys, show_humanoid
    sent = []
    st: dict = {'body': SN(send=lambda **cmd: sent.append(cmd)), 'side': 1.0,
                'orbit': False, 'called': 'strong', 'shown': 'torque', 'skin': 'dressed'}
    for k in 'a4w':
        show_humanoid.KEYS[k](st)
    report.check('A lands her anew, 4 says the fourth entry, W her walk from rest',
                 sent == [{'restart': True}, {'say': 3}, {'walk': True}])
    for k in 'ltco':
        show_humanoid.KEYS[k](st)
    s_pwr = st['shown']
    show_humanoid.KEYS['t'](st)
    s_tmp = st['shown']
    show_humanoid.KEYS['t'](st)
    report.check('L the next callouts, T cycles power -> temp -> torque, C her shell, O the orbit',
                 st['called'] == 'all' and (s_pwr, s_tmp, st['shown']) == ('power', 'temp', 'torque')
                 and st['skin'] == 'shell' and st['orbit'] is True)
    letters = sorted(k for k in show_humanoid.KEYS if len(k) == 1 and k.isalpha() and k.islower())
    expected = sorted(set(list('aclnoprtvw') + [v for v in gym.STUNTS.values() if v.isalpha()]))
    report.check('its letters are the stunts, view and loop controls', letters == expected)
    sent.clear()
    for k in ('up', 'down', '<', '\r'):
        show_humanoid.KEYS[k](st)
    report.check('arrows drive forward and back, < cycles scenario and move, enter runs scenario',
                 sent[0] == {'say': 3} and sent[1] == {'say': 4} and 'scenario' in sent[2] and 'say' in sent[3])
    sent.clear()
    show_humanoid.KEYS['~'](st)
    report.check('tilde / § toggles WASD mode on', st.get('wasd_mode') is True)
    for k in 'wsad':
        show_humanoid.KEYS[k](st)
    report.check('WASD in TPW mode drives forward, back, strafe left and right',
                 sent == [{'say': 3}, {'say': 4}, {'say': 'strafe_left'}, {'say': 'strafe_right'}])
    sent.clear()
    humanoid_keys.act_on(show_humanoid.KEYS, ['w', 'a'], st)
    humanoid_keys.act_on(show_humanoid.KEYS, ['w', 'q'], st)
    report.check('WA and WQ chords execute left turn (forward left arc)',
                 sent == [{'say': 'turn_left'}, {'say': 'turn_left'}])
    sent.clear()
    for ch in (['s', 'd'], ['s', 'e'], ['a', 'e'], ['d', 'q'], ['q', 'e']):
        humanoid_keys.act_on(show_humanoid.KEYS, ch, st)
    humanoid_keys.act_on(show_humanoid.KEYS, ['a', 'e'], st)
    st['_se2_last_t'] -= 0.05
    show_humanoid._step_frame(st, st['body'], 0.05)
    tw_ae = (0.0, round(se2.STRAFE_SPEED, 2), -round(se2.YAW_RATE, 2))
    report.check('SD/SE turn back right, AE/DQ circle-strafe, Q+E cancels, and _step_frame streams twist',
                 sent == [{'say': 'turn_back_right'}, {'say': 'turn_back_right'}, {'say': 'strafe_left'},
                          {'say': 'strafe_right'}, {'say': 'strafe_left'}, {'twist': tw_ae}])
    show_humanoid.KEYS['§'](st)
    sent.clear()
    show_humanoid.KEYS['q'](st)
    show_humanoid.KEYS['e'](st)
    report.check('§ toggles WASD mode off again and normal mode Q/E trigger circle_walk/lie_bed',
                 st.get('wasd_mode') is False and sent == [{'say': 'circle_walk'}, {'say': 'lie_bed'}])


def test_a_held_space_charges_a_push(report):
    """Space held charges a push CHARGE_N_S a second, landing RELEASE_S after its repeats stop."""
    import math, time
    from terminal.views import humanoid_keys, show_humanoid, viewpoint
    keys = humanoid_keys.table(viewpoint.KEYS, ('strong',), ('torque',))
    st: dict = {'charge': None, **viewpoint.HOME}
    t0 = time.monotonic()
    humanoid_keys.act_on(keys, [' '], st)
    lone = (humanoid_keys.released(st, t0 + 0.5), humanoid_keys.released(st, t0 + 1.0))
    report.check('a lone press lands FIRST_S on, a tap of a push', lone[0] == 0.0 and 0.0 < lone[1] < 20.0)
    humanoid_keys.act_on(keys, [' '], st)
    st['charge']['began'] -= 1.0
    for _ in range(10):
        humanoid_keys.act_on(keys, [' '], st)
    held = humanoid_keys.charging(st, time.monotonic())
    soon = humanoid_keys.released(st, time.monotonic() + 0.1)
    landed = humanoid_keys.released(st, time.monotonic() + 0.3)
    report.check('held a second it charges CHARGE_N_S, lands RELEASE_S after the last repeat',
                 abs(held - humanoid_keys.CHARGE_N_S) < 5.0 and soon == 0.0
                 and abs(landed - humanoid_keys.CHARGE_N_S) < 20.0 and st['charge'] is None)
    a = show_humanoid.along(dict(viewpoint.HOME, yaw=0.0, pitch=0.0))
    b = show_humanoid.along(dict(viewpoint.HOME, yaw=90.0, pitch=0.0))
    report.check('the push comes along the line of sight: from +z at yaw 0, from +x at 90',
                 max(abs(x - y) for x, y in zip(a, (0.0, 0.0, -1.0))) < 1e-9
                 and max(abs(x - y) for x, y in zip(b, (-1.0, 0.0, 0.0))) < 1e-9
                 and abs(math.sqrt(sum(x * x for x in a)) - 1.0) < 1e-9)


def test_vision_scanner_docked_pip_and_affordances(report):
    """The stereo vision scanner detects props, renders PiP with affordances, and leaving WASD triggers auto-action."""
    from terminal.views import scanner, show_humanoid
    sc = scanner.VisionScanner()
    report.check('model identification resolves or reports local stereo',
                 'clef' in sc.model_name() or 'stereo' in sc.model_name())
    now = {'where': (0.0, 0.0, 0.0), 'turn': (1, 0, 0, 0), 'angles': {},
           'props': [('chair', (0.0, 0.45, 1.2), (0.22, 0.02, 0.22), ((1, 0, 0), (0, 1, 0), (0, 0, 1)))]}
    det = sc.scan(now, yaw=0.0)
    report.check('chair detected in forward sight cone', len(det) == 1 and det[0]['kind'] == 'chair')
    report.check('chair has sit and hop affordances', any(a[0] == 's' for a in det[0]['affordances']))
    report.check('auto_action selects sit_chair for detected chair', scanner.auto_action(det) == 'sit_chair')
    cells = scanner.docked_vision(sc, now, 100, 40, wasd_mode=True)
    report.check('docked vision returns cells overlay', len(cells) > 0)
    chars = ''.join(chr(c) for c, _k in cells.values())
    report.check('docked vision contains chair label and actions', 'chair' in chars and 'sit' in chars)
    sent = []
    st = {'body': SN(send=lambda **kw: sent.append(kw)), 'wasd_mode': True, 'scanner': sc, 'now': now, 'yaw': 0.0}
    show_humanoid.KEYS['~'](st)
    report.check('leaving WASD mode commands auto-action sit_chair', sent == [{'say': 'sit_chair'}])


def test_tpw_controls(report):
    """TPW mode toggle, Shift sprint, F interact, viewport badge, and A disabled in normal mode."""
    from terminal import keybindings
    from terminal.views import humanoid_keys, show_humanoid
    sent = []
    st = {'body': SN(send=lambda **cmd: sent.append(cmd)), 'wasd_mode': False,
          'yaw': 180.0, 'pitch': 8.0, 'now': {'angles': {}, 'where': (0.0, 0.9, 0.0), 'turn': (1, 0, 0, 0), 'stage': 'stand'}}
    show_humanoid.KEYS['a'](st)
    report.check('a in normal mode is disabled and does not reset robot', not sent)
    report.check('a description explains mode switch / restart disabled',
                 'restart disabled' in keybindings.describe_action('a', wasd_mode=False))
    show_humanoid.KEYS['§'](st)
    report.check('§ toggles TPW mode on', st.get('wasd_mode') is True)
    bar = dict(show_humanoid.bar_for(st))
    report.check('bar_for displays TPW controls', 'SHIFT' in bar and 'F' in bar)
    bg = show_humanoid.tpw_badge(True, 80, 24)
    report.check('TPW mode badge rendered in viewport', len(bg) > 0 and (0, 1) in bg)
    report.check('TPW badge text displays Third Person Control ON',
                 'Third Person Control ON' in ''.join(chr(c) for c, _ in bg.values()))
    report.check('TPW badge text displays Third Person Control OFF',
                 'Third Person Control OFF' in ''.join(chr(c) for c, _ in show_humanoid.tpw_badge(False, 80, 24).values()))
    sent.clear()
    show_humanoid.KEYS['shift'](st)
    report.check('shift sends jog (sprint)', sent == [{'say': 'jog'}])
    sent.clear()
    show_humanoid.KEYS['f'](st)
    report.check('f key executes interact action', len(sent) == 1 and 'say' in sent[0])
    report.check('shove crosshair is hidden in TPW mode', show_humanoid.marked(st, st['now'], 1.0) is None)
    sent.clear()
    humanoid_keys.act_on(show_humanoid.KEYS, [' '], st)
    report.check('Space in TPW mode triggers jump (hop) and does not charge push',
                 sent == [{'say': 'hop'}] and st.get('charge') is None and humanoid_keys.charging(st, 2.0) == 0.0)
    show_humanoid.KEYS['F1'](st)
    report.check('F1 toggles mode back to normal', st.get('wasd_mode') is False)
    report.check('shove crosshair marks torso on normal mode push and is hidden when idle',
                 show_humanoid.marked({**st, 'landed': (0.8, 120.0)}, st['now'], 1.0) is not None
                 and show_humanoid.marked(st, st['now'], 1.0) is None)


def test_surface_arrows_setpoint_actual_and_landing_zones(report):
    """Surface-conforming 3D setpoint arrow frames, progressive actual fill, jump, stop, and fall arrows."""
    import numpy as np
    from coaxial.graphics import arrows, gynoid
    from machine import physics, se2, sprung
    from terminal.views import humanoid_keys
    z3 = np.zeros(27)
    b0, w = sprung.state(sprung.laid(every=3), z3, z3, 0.0, 1.0, None), physics.World()
    w.scenario('slope')
    y0, _ = arrows.surface_at(0.0, 1.2, w.props())
    y1, n1 = arrows.surface_at(0.0, 2.8, w.props())
    report.check('arrow surface_at conforms to inclined slope ramp', y1 > y0 + 0.03 and n1[2] < -0.01)
    w.scenario('hole')
    report.check('arrow surface_at dips into 10cm hole', arrows.surface_at(0.0, 0.55, w.props())[0] < -0.08)

    st = se2.state()
    se2.step_controller(st, 'w', 0.02, now=1.0)
    nw0 = dict(b0, twist=(st['v_x'], 0.0, 0.0), velocity=(0.0, 0.02), speed=0.02, doing='#204')
    sw0 = arrows.describe(nw0, st)
    for k in range(7):
        se2.step_controller(st, 'w', 0.05, now=1.02 + (k + 1) * 0.05)
    sw_mid = arrows.describe(dict(nw0, velocity=(0.0, 0.35), speed=0.35), st)
    sw_blk = arrows.describe(dict(nw0, velocity=(0.0, 0.0), speed=0.0), st)
    sw_rev = arrows.describe(dict(nw0, velocity=(0.0, -0.25), speed=-0.25), st)
    for k in range(40):
        se2.step_controller(st, 'W', 0.05, now=1.40 + (k + 1) * 0.05)
    nw_fast = dict(b0, twist=(st['v_x'], 0.0, 0.0), velocity=(0.0, st['v_x']), speed=st['v_x'], doing='jog')
    sw1 = arrows.describe(nw_fast, st)
    report.check('W / Shift+W arrow starts hollow, stays empty if blocked, and fills from velocity',
                 sw0['kind'] == 'locomotion' and abs(sw0['origin'][0]) < 0.05 and sw_blk['fill'] == 0.0 and sw_rev['fill'] == 0.0
                 and 0.0 < sw0['fill'] < 0.20 < sw_mid['fill'] < 0.85 < sw1['fill'] <= 1.0 and sw1['setpoint'][0] > sw0['setpoint'][0]
                 and len(sw0['frame_segments']) > 10 and len(sw1['fill_segments']) > len(sw0['fill_segments']))

    st_wa, t_wa = se2.state(), 2.0
    for tok in ('wa', 'a', 'a', 'w', 'a', 'a'):
        t_wa += 0.05
        se2.step_controller(st_wa, tok, 0.05, now=t_wa)
    swa = arrows.describe(dict(b0, doing='turn_left', heading=0.35, velocity=(0.0, 0.69), twist_actual=(0.69, 0.0, 0.70)), st_wa)
    sae = arrows.describe(dict(b0, doing='strafe_left', twist_actual=(0.0, 0.45, -0.60)), humanoid_keys.step_se2(se2.state(), 'ae', 0.8, 3.0))
    sq = arrows.describe(dict(b0, doing='turn_left'), humanoid_keys.step_se2(se2.state(), 'q', 0.8, 3.0))
    mx = lambda sp, i: max(p[i] for s in sp['frame_segments'] for p in s)
    report.check('WA curved arrow, AE circle-strafe arc, and Q spin-in-place floor arc render cleanly',
                 swa['kind'] == 'locomotion' and mx(swa, 0) > 0.15 and swa['setpoint'][1] == 0.0
                 and abs(swa['pelvis_yaw']) < 1e-6 and 0.10 < swa['heading'] < 0.15
                 and swa['fill_segments'][-1][0] == swa['frame_segments'][-1][1]
                 and arrows.pelvis_yaw({'turn': (1, 0, 0, 0), 'heading': 0.8}) == 0.0
                 and sae['kind'] == 'locomotion' and mx(sae, 0) > 0.25 and mx(sae, 2) > 0.08
                 and sq['kind'] == 'locomotion' and len(sq['frame_segments']) > 10 and sq['fill'] > 0.5)

    shop = arrows.describe(dict(b0, doing='throw', where=(0.0, 1.02, 0.0)))
    report.check('jump (hop) draws upward 3D arrow frame and fill',
                 shop['kind'] == 'jump' and mx(shop, 1) > 0.75 and len(shop['fill_segments']) > 5)

    sstop = arrows.describe(dict(b0, doing='brake', velocity=(0.0, 0.45), speed=0.45))
    report.check('commanded stop (brake) draws stop bar target and braking arrow',
                 sstop['kind'] == 'stop' and len(sstop['target_segments']) >= 8 and sstop['landing'][2] > 0.25)

    sfall = arrows.describe(dict(b0, doing='controlled_fall', velocity=(0.35, 0.50), where=(0.0, 0.82, 0.0)))
    report.check('commanded fall estimates floor touch-down position and draws target ring',
                 sfall['kind'] == 'fall' and sfall['landing'] is not None and sfall['landing'][2] > 0.20 and len(sfall['target_segments']) >= 16)

    report.check('gynoid.render draws movement arrows into braille viewport',
                 gynoid.render(b0['angles'], 64, 28, colour=False) != gynoid.render(b0['angles'], 64, 28, colour=False, arrows=(nw_fast, st)))


def test_scenario_deck_tour_handoff_and_cybernetic_hud(report):
    """Floor scenarios, [ / ] cycle, Enter/F toggle, tour handoff, BAL/THM/STB gauges, and 40-cell HUD boxes."""
    import io
    from rich.console import Console
    from machine import floor, se2
    from terminal import keybindings
    from terminal.ui import stage
    from terminal.views import gym, humanoid_keys, scanner, show_humanoid

    miss = [s for s in floor.SCENARIOS if s not in keybindings.SCENARIO_MOVES or s not in keybindings.SCENARIO_DESC]
    sent, se2_st = [], se2.state()
    st = {'body': SN(send=lambda **cmd: sent.append(cmd)), 'wasd_mode': True,
          'scenario': 'chair', 'se2': se2_st, 'keys': show_humanoid.KEYS, 'now': {'doing': 'rest', 'where': (0.0, 0.9, 0.0)}}
    for k in ('n', 'N'):
        show_humanoid.KEYS[k](st)
    report.check('all 20 floor scenarios have moves/descriptions, n triggers stairs_down, Shift+N shoves',
                 not miss and len(floor.SCENARIOS) == 20 and sent[0] == {'say': 'stairs_down'} and sent[1]['push'][0] == -140.0)

    humanoid_keys.act_on(show_humanoid.KEYS, ['w'], st)
    sent.clear()
    st['now'] = {'doing': 'lower', 'where': (0.0, 0.72, 0.0)}
    show_humanoid.KEYS['\r'](st)
    st['now'] = {'doing': 'seat_land', 'where': (0.0, 1.25, 0.0)}
    show_humanoid.KEYS['f'](st)
    report.check('Enter/F on chair phases trigger rise_chair / hop_down_chair and clear SE(2) twist',
                 sent == [{'say': 'rise_chair'}, {'say': 'hop_down_chair'}] and se2_st['v_x'] == 0.0 and not st.get('_wasd_active'))
    sent.clear()
    st['now'], st['scenario'] = {'doing': 'sit_edge'}, 'bed'
    show_humanoid.KEYS['\r'](st)
    st['now'], st['scenario'] = {'doing': 'kneel'}, 'mattress'
    show_humanoid.KEYS['f'](st)
    report.check('Enter/F while on bed or mattress triggers rise_bed and rise_floor', sent == [{'say': 'rise_bed'}, {'say': 'rise_floor'}])
    sent.clear()
    st['scenario'] = 'bed'
    show_humanoid.KEYS['['](st)
    report.check('[ cycles scenario backward from bed to chair', st['scenario'] == 'chair' and {'scenario': 'chair'} in sent)
    sent.clear()
    show_humanoid.KEYS[']'](st)
    sc = scanner.VisionScanner()
    det = sc.scan({'where': (0.0, 0.9, 0.0), 'heading': 0.0, 'scenario': 'mattress',
                   'props': [('bed', (0.0, 0.08, 0.9), (0.45, 0.08, 0.95), ((1, 0, 0), (0, 1, 0), (0, 0, 1)))]}, yaw=180.0)
    report.check('] cycles forward to bed and VisionScanner detects mattress in heading cone',
                 st['scenario'] == 'bed' and {'scenario': 'bed'} in sent
                 and len(det) == 1 and det[0]['kind'] == 'mattress' and scanner.auto_action(det) == 'lie_floor')

    show_humanoid.KEYS['H'](st)
    tr, vis = st.get('tour'), set()
    for _ in range(100):
        if tr is not None and st.get('tour') is not None:
            vis.add(tr.step_idx)
            tr.step(st, 0.5)
    report.check('H engages HouseTour on glass_room and stepping all 14 steps does not self-cancel',
                 tr is not None and st.get('tour') is tr and len(vis) == 14)
    show_humanoid.KEYS['w'](st)
    report.check('manual W key disengages active HouseTour autopilot cleanly', st.get('tour') is None)

    nom = {'fallen': False, 'doing': '#204', 'watts': 120.0, 'turn': (1, 0, 0, 0), 'loads': (160.0, 160.0),
           'where': (0.0, 0.9, 0.0), 'cadence': 0.9, 'phase': 0.2, 'ratio': 1.2, 'speed': 0.65,
           'velocity': (0.0, 0.65), 'twist': (0.0, 0.0, 0.0), 'angles': {}, 'torque': {}, 'power': {}, 'temp': {}, 'peak': {}}
    bal, thm, stb, tag = show_humanoid.telemetry_core(nom)
    eb, _, es, et = show_humanoid.telemetry_core(dict(nom, doing='sit_edge', loads=(0.0, 0.0), where=(0.0, 0.55, 0.0)))
    report.check('nominal walking and controlled stunt phases report high BAL/THM/STB and NOMINAL tag',
                 bal > 0.9 and thm > 0.85 and stb > 0.9 and tag == 'NOMINAL' and (eb, es, et) == (0.95, 0.95, 'NOMINAL'))
    cb, _, cs, ct = show_humanoid.telemetry_core(dict(nom, fallen=True))
    _, _, _, ft = show_humanoid.telemetry_core(dict(nom, doing='controlled_fall'))
    report.check('fallen state reports CRITICAL tag while controlled_fall reports ALERT tag', (cb, cs, ct, ft) == (0.0, 0.0, 'CRITICAL', 'ALERT'))
    st.update(now=nom, recording=None, recorded=None, shown='torque', scanner=sc, tour=None)
    report.check('header gauges include BAL, THM, and STB bars alongside mode badge',
                 all(w in show_humanoid.gauges(st).plain for w in ('Third Person Control', 'BAL', 'THM', 'STB')))
    vpb = show_humanoid.tpw_badge(True, 80, 24, state=st)
    r1 = ''.join(chr(c) for (r, _), (c, _) in sorted(vpb.items()) if r == 1)
    jr = show_humanoid._joint_row({'temp': {'spine': 42.0}}, 'spine', 'spine', 'temp')[1].plain
    buf = io.StringIO()
    c40 = Console(file=buf, width=40, theme=stage.THEME, legacy_windows=False)
    for b in show_humanoid.boxes(st, nom, 'dots'):
        c40.print(b)
    b_txt = buf.getvalue()
    report.check('viewport badge renders telemetry strip, gym.boxes shows tour, and HUD boxes fit',
                 all(w in r1 for w in ('BAL', 'THM', 'STB', 'NOMINAL')) and callable(show_humanoid.boot)
                 and len(gym.boxes(nom, wasd=True, tour=SN(done=False))) == 2 and '42°C' in jr and len(jr) == 25
                 and '\u2026' not in b_txt and ('%+5.2f' % se2.WALK_SPEED) in b_txt and len(show_humanoid.legend('temp', 60)) > 20)


ROSTER = (test_the_humanoid_pages_mouse, test_the_humanoid_pages_keys, test_a_held_space_charges_a_push,
          test_vision_scanner_docked_pip_and_affordances, test_tpw_controls,
          test_surface_arrows_setpoint_actual_and_landing_zones, test_scenario_deck_tour_handoff_and_cybernetic_hud)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
