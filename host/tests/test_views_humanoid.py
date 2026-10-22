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
    report.check('a click by key records, a drag and its release, a stuck drag reset',
                 lv_c is None and d_ok and keys._buttons == 0 and not keys._holding and keys._held_at == 999.0)


def test_the_humanoid_pages_keys(report):
    """The page's keys are her verbs, the gym's and the view's: Backspace lands her anew, digits
    say entries, the arrows turn the view, the scene keys say nothing."""
    from terminal.views import show_humanoid, viewpoint
    sent = []
    st: dict = {'body': SN(send=lambda **cmd: sent.append(cmd)), 'orbit': False, 'called': 'strong',
                'shown': 'torque', 'skin': 'dressed', **viewpoint.HOME}
    for k in ('\x08', '4'):
        show_humanoid.KEYS[k](st)
    report.check('Backspace lands her anew, 4 says the fourth entry', sent == [{'restart': True}, {'say': 3}])
    for k in 'ltmo':
        show_humanoid.KEYS[k](st)
    s_pwr = st['shown']
    show_humanoid.KEYS['t'](st)
    s_tmp = st['shown']
    show_humanoid.KEYS['t'](st)
    report.check('L the next callouts, T cycles power -> temp -> torque, M her shell, O the orbit',
                 st['called'] == 'all' and (s_pwr, s_tmp, st['shown']) == ('power', 'temp', 'torque')
                 and st['skin'] == 'shell' and st['orbit'] is True)
    letters = ''.join(sorted(k for k in show_humanoid.KEYS if len(k) == 1 and k.isalpha() and k.islower()))
    report.check('its letters: verbs, the view, the lab - none moves her', letters == 'cfhlmoprtvxz', letters)
    sent.clear()
    yaw = st['yaw']
    for k in ('left', 'up', '<', '\r'):
        show_humanoid.KEYS[k](st)
    report.check('the arrows turn and lift the view, < lays the next scene, Enter lays it again, nothing said',
                 st['yaw'] < yaw and st['pan'][1] > 0.0 and sent == [{'scenario': 'lodge'}] * 2, sent)
    st.update(keys=show_humanoid.KEYS, orbit=False)
    for _ in range(4):
        show_humanoid.scripted(st, 'o.x')
    report.check("--keys types a key a frame, '.' none: O the first frame, X the third, nothing after",
                 st['orbit'] is True and st['shoulder'] == -1.0 and st['frame'] == 4)


def test_the_keys_that_move_her_are_one_twist(report):
    """W A S D move and turn her relative to her own body (independent of the viewport); Q E
    sidestep; Shift sprints: one twist, sent when it changes, each one the bench's plays hold."""
    import time
    from terminal.views import humanoid_keys, viewpoint
    from tools.sim import scenarios

    def tw(word, yaw=180.0, heading=0.0):
        keys = dict({k: k in word.lower() for k in 'wsadqe'}, shift=word != word.lower())
        return humanoid_keys.twist(keys, viewpoint.looks({'yaw': yaw}), heading)

    report.check('in her frame: W walks her on, S walks her back, A sidesteps left, Q turns her left in place',
                 (tw('w'), tw('s'), tw('a'), tw('q')) == ((0.69, 0.0, 0.0), (-0.35, 0.0, 0.0), (0.0, 0.5, 0.0), (0.0, 0.0, 0.7)))
    report.check('independent of view yaw: W walks straight, WA arcs left, Shift+W sprints, D sidesteps right',
                 (tw('w', 240.0), tw('wa', 240.0), tw('W', 90.0), tw('d', 45.0))
                 == ((0.69, 0.0, 0.0), (0.69, 0.0, 0.7), (1.4, 0.0, 0.0), (0.0, -0.5, 0.0)))
    every = {tw(w, yaw, hd) for w in ('w', 's', 'a', 'd', 'wa', 'wd', 'sa', 'sd', 'q', 'e', 'W', 'WA', 'S')
             for yaw in range(0, 360, 15) for hd in (0.0, 1.0, -2.0)}
    bound = [max(abs(t[i]) for t in scenarios.KEYS.values()) for i in range(3)]
    report.check("every twist the keys send within the bench plays' bounds, a sidestep alone, a sprint all but straight",
                 all(abs(t[i]) <= bound[i] + 1e-9 for t in every for i in range(3))
                 and all(not (t[1] and (t[0] or t[2])) for t in every)
                 and all(abs(t[2]) <= 0.3 for t in every if t[0] > 0.69), sorted(every)[:4])
    sent = []
    st: dict = {'body': SN(send=lambda **cmd: sent.append(cmd)), 'yaw': 180.0, 'tour': None}
    humanoid_keys.act_on({}, ['w'], st)
    t = time.monotonic()
    for dt in (0.0, 0.05, 2.0):
        humanoid_keys.steered(st, t + dt, 0.0)
    report.check('a W typed walks her, sent once while held, the stop once when let go',
                 sent == [{'twist': (0.69, 0.0, 0.0)}, {'twist': (0.0, 0.0, 0.0)}]
                 and not humanoid_keys.polled(['s'], st['pad'], t, held=set())['s'], sent)


def test_a_held_p_charges_a_push(report):
    """P held charges a push CHARGE_N_S a second, landing RELEASE_S after its repeats stop."""
    import math
    import time
    from terminal.views import humanoid_keys, viewpoint
    keys = humanoid_keys.table(viewpoint.KEYS, ('strong',), ('torque',))
    st: dict = {'charge': None, **viewpoint.HOME}
    t0 = time.monotonic()
    humanoid_keys.act_on(keys, ['p'], st)
    lone = (humanoid_keys.released(st, t0 + 0.5), humanoid_keys.released(st, t0 + 1.0))
    report.check('a lone press lands FIRST_S on, a tap of a push', lone[0] == 0.0 and 0.0 < lone[1] < 20.0)
    humanoid_keys.act_on(keys, ['p'], st)
    st['charge']['began'] -= 1.0
    for _ in range(10):
        humanoid_keys.act_on(keys, ['p'], st)
    held = humanoid_keys.charging(st, time.monotonic())
    soon = humanoid_keys.released(st, time.monotonic() + 0.1)
    landed = humanoid_keys.released(st, time.monotonic() + 0.3)
    report.check('held a second it charges CHARGE_N_S, lands RELEASE_S after the last repeat',
                 abs(held - humanoid_keys.CHARGE_N_S) < 5.0 and soon == 0.0
                 and abs(landed - humanoid_keys.CHARGE_N_S) < 20.0 and st['charge'] is None)
    a = viewpoint.along(dict(viewpoint.HOME, yaw=0.0))
    b = viewpoint.along(dict(viewpoint.HOME, yaw=90.0))
    report.check('the push comes along the view over the floor: from +z at yaw 0, from +x at 90',
                 max(abs(x - y) for x, y in zip(a, (0.0, 0.0, -1.0))) < 1e-9
                 and max(abs(x - y) for x, y in zip(b, (-1.0, 0.0, 0.0))) < 1e-9
                 and abs(math.sqrt(sum(x * x for x in a)) - 1.0) < 1e-9)


def test_vision_scanner_docked_pip_and_affordances(report):
    """The stereo scanner finds a support near her, F says its verb at it and the page prompts
    it; the balloons' keys on the docked plate (test_views_floor: the balloons)."""
    from terminal.views import gym, scanner, show_humanoid
    sc = scanner.VisionScanner()
    report.check('model identification resolves or reports local stereo',
                 'clef' in sc.model_name() or 'stereo' in sc.model_name())
    now = {'where': (0.0, 0.0, 0.0), 'turn': (1, 0, 0, 0), 'angles': {}, 'heading': 0.0,
           'props': [('chair', (0.0, 0.45, 0.9), (0.22, 0.02, 0.22), ((1, 0, 0), (0, 1, 0), (0, 0, 1)))]}
    det = sc.scan(now, 0.0)
    report.check('a chair in her sight, its verb sit_chair',
                 len(det) == 1 and det[0]['kind'] == 'chair' and scanner.auto_action(det) == 'sit_chair'
                 and det[0]['affordances'] == [('sit_chair', 'sit')])
    sent = []
    st = {'body': SN(send=lambda **kw: sent.append(kw)), 'scanner': sc, 'now': now, 'yaw': 180.0, 'tour': None}
    cells = scanner.docked_vision(sc, now, 100, 40, gym.offered(st))
    chars = ''.join(chr(c) for c, _k in cells.values())
    report.check('the docked eyes name the chair and F1 on it', 'chair' in chars and '[F1] sit' in chars)
    show_humanoid.KEYS['f'](st)
    report.check('F by the chair says sit_chair at it, the page prompts it, a plain hop unprompted',
                 [(c['say'], tuple(round(v, 9) for v in c['at'])) for c in sent] == [('sit_chair', (0.0, 0.9))]
                 and gym.prompt(st) == '[F1] sit chair', (sent, gym.prompt(st)))


def test_her_verbs_and_the_chase(report):
    """Space hops - up onto a chair close ahead, down from its seat -, C crouches and stands her, X
    swaps the shoulder; the view eases behind her as she goes, not after a look nor standing."""
    import math
    from terminal.views import scanner, show_humanoid, viewpoint
    sent = []
    chair = [('chair', (0.0, 0.45, 0.5), (0.22, 0.02, 0.22), ((1, 0, 0), (0, 1, 0), (0, 0, 1)))]
    st: dict = {'body': SN(send=lambda **cmd: sent.append(cmd)), 'scanner': scanner.VisionScanner(), 'tour': None,
                **viewpoint.HOME, 'now': {'where': (0.0, 0.9, 0.0), 'heading': 0.0, 'doing': 'rest', 'props': chair}}
    show_humanoid.KEYS[' '](st)
    st['now'] = dict(st['now'], where=(0.0, 1.35, 0.0))
    show_humanoid.KEYS[' '](st)
    st['now'] = dict(st['now'], where=(0.0, 0.9, 0.0), props=[])
    show_humanoid.KEYS[' '](st)
    report.check('Space hops her up onto a chair close ahead, down from its seat, in place with none',
                 sent == [{'say': 'hop_up_chair'}, {'say': 'hop_down_chair'}, {'say': 'hop'}], sent)
    sent.clear()
    show_humanoid.KEYS['c'](st)
    st['now'] = dict(st['now'], doing='down')
    show_humanoid.KEYS['c'](st)
    show_humanoid.KEYS['x'](st)
    report.check('C crouches her and stands her from it; X looks over her other shoulder',
                 sent == [{'say': 'crouch'}, {'say': 'stand'}] and st['shoulder'] == -1.0
                 and viewpoint.panned(st)[0] == -viewpoint.SHOULDER_M)
    t, behind = 100.0, 180.0 + math.degrees(1.0)
    went, looked, stood = ({**viewpoint.HOME, 'orbit': False} for _ in range(3))
    looked['looked'] = t + 0.5
    for view, going in ((went, True), (looked, True), (stood, False)):
        for dt in (0.0, viewpoint.LAG_S):
            viewpoint.chased(view, 1.0, going, t + 0.5 + dt)
    from coaxial.graphics import arrows
    errands = [arrows.describe(dict(st['now'], doing=d, entry=e, velocity=(0.0, 0.35)))['kind']
               for e in ('sit_chair', 'lie_bed', 'stairs_walk') for d in ('init_walk', 'brake', 'stand', e)]
    orb = {**viewpoint.HOME, 'orbit': False, 'left_held': True}
    viewpoint.mouse(orb, lambda: (120, 50))['on_drag'](30, 5)
    yaw_held, pitch_held = orb['yaw'], orb['pitch']
    viewpoint.chased(orb, 0.0, True, t)
    viewpoint.chased(orb, 0.0, True, t + 5.0)
    held_ok = orb['yaw'] == yaw_held and orb['pitch'] == pitch_held
    orb['left_held'] = False
    viewpoint.chased(orb, 0.0, True, t + 5.0 + viewpoint.LAG_S)
    ret_ok = abs((orb['yaw'] - yaw_held) / (180.0 - yaw_held) - (1.0 - math.exp(-1.0))) < 1e-6 and orb['pitch'] < pitch_held
    report.check('the view eases behind her as she goes, 63 % in LAG_S; left-drag holds while pressed and returns on release',
                 abs((went['yaw'] - 180.0) / (behind - 180.0) - (1.0 - math.exp(-1.0))) < 1e-6
                 and looked['yaw'] == stood['yaw'] == 180.0 and held_ok and ret_ok
                 and errands == ['fall'] * 8 + ['locomotion'] * 4)


def test_scenario_deck_tour_handoff_and_cybernetic_hud(report):
    """Floor scenarios, [ / ] cycle, F up from what bears her, the tour as an agent's commands,
    BAL/THM/STB gauges, and 40-cell HUD boxes."""
    import io
    from rich.console import Console
    from machine import floor, se2, tour
    from terminal import keybindings
    from terminal.ui import stage
    from terminal.views import gym, humanoid_keys, scanner, show_humanoid, viewpoint

    miss = [s for s in floor.SCENARIOS if s not in keybindings.SCENARIO_DESC]
    sent = []
    st = {'body': SN(send=lambda **cmd: sent.append(cmd)), 'scenario': 'chair', 'pad': humanoid_keys.pad(),
          'keys': show_humanoid.KEYS, 'tour': None, **viewpoint.HOME, 'now': {'doing': 'rest', 'where': (0.0, 0.9, 0.0)}}
    show_humanoid.KEYS['N'](st)
    report.check('all 21 floor scenarios described, Shift+N shoves 200 N along the view',
                 not miss and len(floor.SCENARIOS) == 21 and abs(sent[0]['push'][2] - 200.0) < 1e-9)
    sent.clear()
    st['now'] = {'doing': 'lower', 'where': (0.0, 0.72, 0.0), 'scenario': 'chair'}
    show_humanoid.KEYS['f'](st)
    st['now'] = {'doing': 'seat_land', 'where': (0.0, 1.25, 0.0), 'scenario': 'chair'}
    show_humanoid.KEYS['f'](st)
    st['now'] = {'doing': 'sit_edge', 'scenario': 'bed'}
    show_humanoid.KEYS['f'](st)
    st['now'] = {'doing': 'kneel', 'scenario': 'mattress'}
    show_humanoid.KEYS['f'](st)
    report.check('F rises her from a chair, hops her down from its seat, rises her from a bed and a mattress',
                 [c['say'] for c in sent] == ['rise_chair', 'hop_down_chair', 'rise_bed', 'rise_floor'], sent)
    sent.clear()
    st['scenario'] = 'bed'
    show_humanoid.KEYS['['](st)
    report.check('[ from bed back to chair', st['scenario'] == 'chair' and {'scenario': 'chair'} in sent)
    sent.clear()
    show_humanoid.KEYS[']'](st)
    sc = scanner.VisionScanner()
    det = sc.scan({'where': (0.0, 0.9, 0.0), 'heading': 0.0, 'scenario': 'mattress',
                   'props': [('bed', (0.0, 0.08, 0.9), (0.45, 0.08, 0.95), ((1, 0, 0), (0, 1, 0), (0, 0, 1)))]}, 0.0)
    report.check('] the bed; a mattress ahead seen',
                 st['scenario'] == 'bed' and {'scenario': 'bed'} in sent
                 and len(det) == 1 and det[0]['kind'] == 'mattress' and scanner.auto_action(det) == 'lie_floor')

    sent.clear()
    show_humanoid.KEYS['H'](st)
    tr, vis = st['tour'], set()
    for _ in range(2 * int(sum(step[1] for step in tour.STEPS)) + 2):
        if gym.touring(st):
            vis.add(tr.step_idx)
            tr.step(st, 0.5)
    verbs = {c['say'] for c in sent if 'say' in c}
    report.check('H tours every step, sending what an agent sends: twists, verbs, scenes, pushes',
                 tr is not None and st.get('tour') is tr and len(vis) == len(tour.STEPS)
                 and all(set(c) <= {'twist', 'say', 'scenario', 'push', 'seconds'} for c in sent)
                 and verbs == set(scanner.PRIMARY_ACTION.values()) | {'rise_chair', 'rise_bed', 'rise_floor'}, verbs)
    humanoid_keys.act_on(show_humanoid.KEYS, ['w'], st)
    report.check('a key that moves her ends the tour', st.get('tour') is None)

    nom = {'fallen': False, 'doing': 'walk', 'watts': 120.0, 'turn': (1, 0, 0, 0), 'loads': (160.0, 160.0),
           'where': (0.0, 0.9, 0.0), 'cadence': 0.9, 'phase': 0.2, 'ratio': 1.2, 'speed': 0.65,
           'velocity': (0.0, 0.65), 'twist': (0.0, 0.0, 0.0), 'angles': {}, 'torque': {}, 'power': {}, 'temp': {}, 'peak': {}}
    bal, thm, stb, tag = show_humanoid.telemetry_core(nom)
    eb, _, es, et = show_humanoid.telemetry_core(dict(nom, doing='sit_edge', loads=(0.0, 0.0), where=(0.0, 0.55, 0.0)))
    report.check('walking and a stunt read high BAL/THM/STB, NOMINAL',
                 bal > 0.9 and thm > 0.85 and stb > 0.9 and tag == 'NOMINAL' and (eb, es, et) == (0.95, 0.95, 'NOMINAL'))
    cb, _, cs, ct = show_humanoid.telemetry_core(dict(nom, fallen=True))
    _, _, _, ft = show_humanoid.telemetry_core(dict(nom, doing='controlled_fall'))
    report.check('fallen CRITICAL, a controlled fall ALERT', (cb, cs, ct, ft) == (0.0, 0.0, 'CRITICAL', 'ALERT'))
    st.update(now=nom, recording=None, recorded=None, shown='torque', scanner=sc, tour=None)
    report.check('the header: BAL, THM, STB', all(w in show_humanoid.gauges(st).plain for w in ('BAL', 'THM', 'STB')))
    vpb = show_humanoid.badge(80, 24, st)
    r1 = ''.join(chr(c) for (r, _), (c, _) in sorted(vpb.items()) if r == 1)
    jr = show_humanoid._joint_row({'temp': {'spine': 42.0}}, 'spine', 'spine', 'temp')[1].plain
    buf = io.StringIO()
    c40 = Console(file=buf, width=40, theme=stage.THEME, legacy_windows=False)
    for b in show_humanoid.boxes(st, nom, 'dots'):
        c40.print(b)
    b_txt = buf.getvalue()
    report.check('the badge strip, the tour in the boxes, 40 cells fit',
                 all(w in r1 for w in ('BAL', 'THM', 'STB', 'NOMINAL')) and callable(show_humanoid.boot)
                 and len(gym.boxes(nom, tour=SN(done=False, current_label=lambda: 'x'))) == 2 and '42°C' in jr and len(jr) == 25
                 and '…' not in b_txt and ('%+5.2f' % se2.WALK_SPEED) in b_txt and len(show_humanoid.legend('temp', 60)) > 20)
    sent.clear()
    act = lambda s: humanoid_keys.act_on(show_humanoid.KEYS, list(s), st)
    act('/sit in the chair\r')
    r0_llm = ''.join(chr(c) for (r, _), (c, _) in sorted(show_humanoid.badge(100, 24, st).items()) if r == 0)
    act('/go to bed')
    r_bar = ''.join(chr(c) for (r, _), (c, _) in sorted(show_humanoid.badge(100, 24, st).items()) if r == 21)
    cb_open = str(show_humanoid.chat_box(st, 80).renderable)
    for q in ('\r', '/exit out the door\r', '/tour the room\r'):
        act(q)
    cb_done = str(show_humanoid.chat_box(st, 80).renderable)
    report.check('/ opens interactive LLM prompt bar, compiles chair/bed/door/tour prompts and renders GYNOID CHAT dock',
                 sent == [{'say': 'sit_chair'}, {'say': 'lie_bed'}, {'say': 'exit_door'}]
                 and 'LLM > go to bed_' in r_bar and 'sit_chair' in r0_llm and gym.touring(st)
                 and '> sit in the chair' in cb_open and 'go to bed' in cb_open
                 and 'cmd: tour=start' in cb_done and ('/', 'CHAT') in keybindings.BAR, (sent, r_bar))


def test_her_clothes_bend_at_the_joints(report):
    """A stride: the seat's dots on the swung thigh's side follow it, the tee's hem the pelvis
    under a bent spine, the sleeves the torso; the rest of her as unbent."""
    from coaxial.graphics import gynoid
    body = gynoid.body()
    stride = {'left_hip': -35.0, 'right_hip': 25.0, 'left_knee': 15.0, 'right_knee': 40.0, 'spine': 15.0,
              'left_shoulder': 30.0, 'right_shoulder': -30.0}
    bent, _n = body.pose(stride, dense=True)
    kept, body.bends = body.bends, []
    try:
        stiff, _n = body.pose(stride, dense=True)
    finally:
        body.bends = kept
    moved = {body.parts[i][0]: float(abs(bent[lo:hi] - stiff[lo:hi]).max())
             for i, (lo, hi) in enumerate(body.dense_spans) if hi > lo}
    cloth = {k: v for k, v in moved.items() if v > 1e-9}
    report.check('the seat, the tee, the shins and the sleeves bend by 1-8 cm; nothing else moves',
                 set(cloth) == {'cloth_seat', 'cloth_tee', 'cloth_left_shin', 'cloth_right_shin',
                                'cloth_left_sleeve', 'cloth_right_sleeve'}
                 and all(0.01 < v < 0.08 for v in cloth.values()),
                 {k: round(v, 3) for k, v in cloth.items()})


def test_a_scene_laid_anew_is_told_the_keys_again(report):
    """W held through a scene change: her loop lays her anew with no twist, and the page tells
    her the keys' twist again as it sees the new scene, not only when the keys change."""
    from terminal.views import humanoid_keys, viewpoint
    sent = []
    state = {'body': SN(send=lambda **cmd: sent.append(cmd)), 'now': {'scenario': 'chair'}, 'tour': None,
             **viewpoint.HOME, 'pad': humanoid_keys.pad()}
    humanoid_keys.polled(['w'], state['pad'], 0.0)
    humanoid_keys.steered(state, 0.0, 0.0)
    humanoid_keys.polled(['w'], state['pad'], 0.05)
    humanoid_keys.steered(state, 0.05, 0.0)
    state['now'] = {'scenario': 'bed'}
    humanoid_keys.polled(['w'], state['pad'], 0.1)
    humanoid_keys.steered(state, 0.1, 0.0)
    twists = [c['twist'] for c in sent if 'twist' in c]
    report.check('the walk told once on W, not again the next frame, again on the new scene',
                 len(twists) == 2 and twists[0] == twists[1] and twists[0][0] > 0.5, twists)


ROSTER = (test_the_humanoid_pages_mouse, test_the_humanoid_pages_keys, test_the_keys_that_move_her_are_one_twist,
          test_a_held_p_charges_a_push, test_vision_scanner_docked_pip_and_affordances, test_her_verbs_and_the_chase,
          test_scenario_deck_tour_handoff_and_cybernetic_hud, test_her_clothes_bend_at_the_joints,
          test_a_scene_laid_anew_is_told_the_keys_again)


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
