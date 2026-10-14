"""The HUMANOID page's hands: its mouse on the view, its keys on her loop and on the view."""
import sys

from tools.dev.focus import chosen
from views_kit import Report


def test_the_humanoid_pages_mouse(report):
    """The HUMANOID page holds the mouse, a key typed kept: a right drag moves the view by its cells
    and zooms nothing, a left drag turns and tips it, the arrows lift it (`viewpoint`)."""
    from terminal.ui import console
    from terminal.views import viewpoint
    keys = console.Keys(console=True, mouse=True, select=frozenset(), pan=True)
    esc = chr(27)
    keys._buffer = esc + '[<2;5;20M' + esc + '[<34;9;23M' + esc + '[<34;12;26M' + 'f'
    leave, zoom = keys.poll()
    report.check('a right drag moves by its cells, zooms nothing, F typed',
                 leave is None and zoom == 0.0 and keys.panned() == (7, 6)
                 and keys.taken() == ['f'])
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
                 and state['yaw'] > viewpoint.YAW and state['pitch'] > viewpoint.PITCH,
                 '%s' % state)
    viewpoint.KEYS['v'](state)
    report.check('V home', all(state[k] == v for k, v in viewpoint.HOME.items()))
    viewpoint.mouse(state, lambda: (120, 50))['on_drag'](5, 0)
    report.check('a left drag to the right turns her the way the left arrow does',
                 state['yaw'] < viewpoint.YAW, '%s' % state)
    import types
    sgr_records = [
        ('\x1b', 27, 1), ('[', 219, 1), ('<', 226, 1), ('0', 48, 1), (';', 188, 1),
        ('5', 53, 2), (';', 188, 1), ('2', 50, 2), ('M', 77, 1),
        ('\x1b', 27, 1), ('[', 219, 1), ('<', 226, 1), ('0', 48, 1), (';', 188, 1),
        ('5', 53, 2), (';', 188, 1), ('2', 50, 2), ('m', 77, 1),
    ]
    emitted = [keys._key_of(types.SimpleNamespace(ch=ch, vk=vk, repeat=rep, down=1))
               for ch, vk, rep in sgr_records]
    keys._buffer = ''.join(emitted)
    leave_click, _ = keys.poll()
    report.check('left click via Win32 VT KEY_EVENT_RECORDs (col=55, row=22) registers click without exiting',
                 leave_click is None and keys.clicked() == [(55, 22)] and keys.taken() == [],
                 'leave=%s clicked=%s emitted=%r' % (leave_click, keys._clicked, ''.join(emitted)))


def test_the_humanoid_pages_keys(report):
    """The page's keys are her loop's and the view's: A lands her anew, the digits say her moves,
    W walks her, L, T, C and O step the view; no other letter is a key - nothing else runs her."""
    import types
    from terminal.views import show_humanoid
    sent = []
    body = types.SimpleNamespace(send=lambda **command: sent.append(command))
    state: dict = {'body': body, 'side': 1.0, 'orbit': False, 'called': 'strong', 'shown': 'torque',
                   'skin': 'dressed'}
    for key in 'a4w':
        show_humanoid.KEYS[key](state)
    report.check('A lands her anew, 4 says the fourth entry, W her walk from rest',
                 sent == [{'restart': True}, {'say': 3}, {'walk': True}], '%s' % sent)
    for key in 'ltco':
        show_humanoid.KEYS[key](state)
    report.check('L the next callouts, T the power, C her shell, O the orbit',
                 state['called'] == 'all' and state['shown'] == 'power' and state['skin'] == 'shell'
                 and state['orbit'] is True, '%s' % {k: state[k] for k in ('called', 'shown', 'skin', 'orbit')})
    from terminal.views import gym
    letters = sorted(k for k in show_humanoid.KEYS if len(k) == 1 and k.isalpha() and k.islower())
    expected = sorted(set(list('aclnoprtvw') + [v for v in gym.STUNTS.values() if v.isalpha()]))
    report.check('its letters are the stunts, view and loop controls', letters == expected,
                 ' '.join(letters))
    sent.clear()
    for key in ('up', 'down', '<', '\r'):
        show_humanoid.KEYS[key](state)
    report.check('arrows drive forward and back, < cycles scenario and move, enter runs scenario',
                 sent[0] == {'say': 3} and sent[1] == {'say': 4}
                 and 'scenario' in sent[2] and 'say' in sent[3], '%s' % sent)
    from terminal.views import humanoid_keys
    sent.clear()
    show_humanoid.KEYS['~'](state)
    report.check('tilde / § toggles WASD mode on', state.get('wasd_mode') is True)
    for k in 'wsad':
        show_humanoid.KEYS[k](state)
    report.check('WASD in TPW mode drives forward, back, strafe left and right',
                 sent == [{'say': 3}, {'say': 4}, {'say': 'strafe_left'}, {'say': 'strafe_right'}],
                 '%s' % sent)
    sent.clear()
    humanoid_keys.act_on(show_humanoid.KEYS, ['w', 'a'], state)
    report.check('WA chord executes left turn (vänstersväng)',
                 sent == [{'say': 'turn_left'}], '%s' % sent)
    sent.clear()
    humanoid_keys.act_on(show_humanoid.KEYS, ['s', 'd'], state)
    report.check('SD chord executes right backward turn (SD-högersväng)',
                 sent == [{'say': 'turn_back_right'}], '%s' % sent)
    show_humanoid.KEYS['§'](state)
    report.check('§ toggles WASD mode off again', state.get('wasd_mode') is False)


def test_a_held_space_charges_a_push(report):
    """Space held charges a push CHARGE_N_S a second, landing RELEASE_S after its repeats stop
    along the view's line of sight onto her torso, a mark there; a lone press FIRST_S on."""
    import math
    import time
    from terminal.views import humanoid_keys, show_humanoid, viewpoint
    keys = humanoid_keys.table(viewpoint.KEYS, ('strong',), ('torque',))
    state: dict = {'charge': None, **viewpoint.HOME}
    t0 = time.monotonic()
    humanoid_keys.act_on(keys, [' '], state)
    lone = (humanoid_keys.released(state, t0 + 0.5), humanoid_keys.released(state, t0 + 1.0))
    report.check('a lone press lands FIRST_S on, a tap of a push', lone[0] == 0.0 and 0.0 < lone[1] < 20.0,
                 '%s' % (lone,))
    humanoid_keys.act_on(keys, [' '], state)
    state['charge']['began'] -= 1.0
    for _ in range(10):
        humanoid_keys.act_on(keys, [' '], state)
    held = humanoid_keys.charging(state, time.monotonic())
    soon = humanoid_keys.released(state, time.monotonic() + 0.1)
    landed = humanoid_keys.released(state, time.monotonic() + 0.3)
    report.check('held a second it charges CHARGE_N_S, lands RELEASE_S after the last repeat',
                 abs(held - humanoid_keys.CHARGE_N_S) < 5.0 and soon == 0.0
                 and abs(landed - humanoid_keys.CHARGE_N_S) < 20.0 and state['charge'] is None,
                 'charging %.0f N, at 0.1 s %.0f, at 0.3 s %.0f' % (held, soon, landed))
    a = show_humanoid.along(dict(viewpoint.HOME, yaw=0.0, pitch=0.0))
    b = show_humanoid.along(dict(viewpoint.HOME, yaw=90.0, pitch=0.0))
    report.check('the push comes along the line of sight: from +z at yaw 0, from +x at 90',
                 max(abs(x - y) for x, y in zip(a, (0.0, 0.0, -1.0))) < 1e-9
                 and max(abs(x - y) for x, y in zip(b, (-1.0, 0.0, 0.0))) < 1e-9
                 and abs(math.sqrt(sum(x * x for x in a)) - 1.0) < 1e-9, '%s %s' % (a, b))


def test_r_records_and_saves(report):
    """R starts a recording, every state the page hears becomes a row (HEADER), R again writes
    the CSV under RECORDINGS and names it on the page - headless, as the page takes R."""
    import csv
    import os
    import tempfile
    from machine.figure import JOINTS
    from terminal.views import humanoid_keys
    now = {'t': 1.25, 'stage': 'walk', 'speed': 0.5, 'phase': 0.3, 'loads': (200.0, 300.0),
           'where': (0.0, 0.9, 1.0), 'turn': (1.0, 0.0, 0.0, 0.0), 'set': {'left_knee': 20.0},
           'watts': 150.0, 'angles': {j: 0.0 for j in JOINTS}}
    was, humanoid_keys.RECORDINGS = humanoid_keys.RECORDINGS, tempfile.mkdtemp()
    try:
        state: dict = {'recording': None, 'recorded': None, 'yaw': 60.0}
        humanoid_keys._recorded(state)
        state['recording'] += [humanoid_keys.row(now, state['yaw']) for _ in range(3)]
        humanoid_keys._recorded(state)
        path = state['recorded']
        with open(path, newline='', encoding='utf-8') as f:
            rows = list(csv.reader(f))
        report.check('R, three states, R: a CSV of the header and three rows, every field filled',
                     rows[0] == humanoid_keys.HEADER and len(rows) == 4
                     and all(len(r) == len(humanoid_keys.HEADER) for r in rows[1:])
                     and state['recording'] is None and os.path.basename(path).startswith('humanoid_'),
                     '%s: %d rows of %d fields' % (os.path.basename(path), len(rows) - 1, len(rows[-1])))
    finally:
        humanoid_keys.RECORDINGS = was


def test_vision_scanner_docked_pip_and_affordances(report):
    """The stereo vision scanner detects props, renders PiP with affordances, and leaving WASD triggers auto-action."""
    from terminal.views import scanner, show_humanoid
    sc = scanner.VisionScanner()
    report.check('model identification resolves or reports local stereo',
                 'clef' in sc.model_name() or 'stereo' in sc.model_name(), sc.model_name())

    # Simulated gynoid state facing a chair
    now = {
        'where': (0.0, 0.0, 0.0),
        'turn': (1.0, 0.0, 0.0, 0.0),
        'angles': {},
        'props': [('chair', (0.0, 0.45, 1.2), (0.22, 0.02, 0.22), ((1, 0, 0), (0, 1, 0), (0, 0, 1)))],
    }
    detected = sc.scan(now, yaw=0.0)
    report.check('chair detected in forward sight cone', len(detected) == 1 and detected[0]['kind'] == 'chair',
                 str(detected))
    report.check('chair has sit and hop affordances', any(a[0] == 's' for a in detected[0]['affordances']))
    report.check('auto_action selects sit_chair for detected chair', scanner.auto_action(detected) == 'sit_chair')

    # Docked vision PiP rendering
    cells = scanner.docked_vision(sc, now, 100, 40, wasd_mode=True)
    report.check('docked vision returns cells overlay', len(cells) > 0)
    chars = ''.join(chr(c) for c, _k in cells.values())
    report.check('docked vision contains chair label and actions', 'chair' in chars and 'sit' in chars)

    # Leaving WASD mode triggers auto-action
    sent = []
    class DummyBody:
        def send(self, **kw):
            sent.append(kw)
    state = {'body': DummyBody(), 'wasd_mode': True, 'scanner': sc, 'now': now, 'yaw': 0.0}
    show_humanoid.KEYS['~'](state)
    report.check('leaving WASD mode commands auto-action sit_chair', sent == [{'say': 'sit_chair'}], str(sent))


def test_tpw_controls(report):
    """TPW mode toggle (§ / ~ / F1 / F2), Shift sprints, F interacts, viewport badge renders, and A never resets robot."""
    import types
    from terminal.views import gym, humanoid_keys, show_humanoid
    sent = []
    body = types.SimpleNamespace(send=lambda **command: sent.append(command))
    state = {'body': body, 'wasd_mode': False, 'yaw': 180.0, 'pitch': 8.0, 'now': {
        'angles': {}, 'where': (0.0, 0.9, 0.0), 'turn': (1.0, 0.0, 0.0, 0.0), 'stage': 'stand'
    }}

    # 1. 'a' in normal mode does not reset robot
    show_humanoid.KEYS['a'](state)
    report.check('a in normal mode is disabled and does not reset robot', len(sent) == 0, str(sent))
    report.check('a description explains mode switch / restart disabled',
                 'restart disabled' in humanoid_keys._action_of('a', wasd_mode=False))

    # 2. § / ~ / F1 toggles TPW mode on
    show_humanoid.KEYS['§'](state)
    report.check('§ toggles TPW mode on', state.get('wasd_mode') is True)
    bar = show_humanoid.bar_for(state)
    report.check('bar_for displays TPW controls', any(k == 'SHIFT' for k, _ in bar) and any(k == 'F' for k, _ in bar))

    # 3. Discreet Third Person Control badge rendered in 3D viewport
    badge = show_humanoid.tpw_badge(True, 80, 24)
    report.check('TPW mode badge rendered in viewport', len(badge) > 0 and (0, 1) in badge)
    chars = ''.join(chr(c) for c, _k in badge.values())
    report.check('TPW badge text displays Third Person Control ON', 'Third Person Control ON' in chars)
    badge_off = show_humanoid.tpw_badge(False, 80, 24)
    chars_off = ''.join(chr(c) for c, _k in badge_off.values())
    report.check('TPW badge text displays Third Person Control OFF', 'Third Person Control OFF' in chars_off)

    # 4. Shift key sends sprint / jog
    sent.clear()
    show_humanoid.KEYS['shift'](state)
    report.check('shift sends jog (sprint)', sent == [{'say': 'jog'}], str(sent))

    # 5. F key interacts
    sent.clear()
    show_humanoid.KEYS['f'](state)
    report.check('f key executes interact action', len(sent) == 1 and 'say' in sent[0], str(sent))

    # 6. Crosshair reticle rendered along aim vector and Space triggers Jump (not push) in TPW mode
    mark = show_humanoid.marked(state, state['now'], 1.0)
    report.check('crosshair reticle is rendered along line of sight', mark is not None and mark[1] == (96, 220, 255), str(mark))
    sent.clear()
    humanoid_keys.act_on(show_humanoid.KEYS, [' '], state)
    report.check('Space in TPW mode triggers jump (hop) and does not charge push',
                 sent == [{'say': 'hop'}] and state.get('charge') is None
                 and humanoid_keys.charging(state, 2.0) == 0.0, str(sent))

    # 7. F1 toggles mode off cleanly
    show_humanoid.KEYS['F1'](state)
    report.check('F1 toggles mode back to normal', state.get('wasd_mode') is False)
    mark_off = show_humanoid.marked(state, state['now'], 1.0)
    report.check('crosshair hidden in normal mode', mark_off is None)


def test_surface_arrows_setpoint_actual_and_landing_zones(report):
    """Surface-conforming 3D setpoint arrow frames, progressive actual fill, jump arrows, stop bars, and fall landing zones."""
    import numpy as np
    from coaxial.graphics import arrows, gynoid
    from machine import physics, se2, sprung

    g = sprung.laid(every=3)
    z3 = np.zeros(27)
    base_now = sprung.state(g, z3, z3, 0.0, 1.0, None)

    # 1. Surface conformance across slope, snow, ice, stairs, and hole
    w = physics.World()
    w.scenario('slope')
    slope_props = w.props()
    y0, n0 = arrows.surface_at(0.0, 1.2, slope_props)
    y1, n1 = arrows.surface_at(0.0, 2.8, slope_props)
    report.check('arrow surface_at conforms to inclined slope ramp (y rises with z, normal tilted)',
                 y1 > y0 + 0.03 and n1[2] < -0.01, 'y0=%.3f, y1=%.3f, n1=%s' % (y0, y1, n1))
    w.scenario('hole')
    yh, _ = arrows.surface_at(0.0, 1.2, w.props())
    report.check('arrow surface_at dips into 10cm hole', yh < -0.08, 'yh=%.3f' % yh)

    # 2. WASD locomotion: setpoint frame immediately shows target, interior fill ramps progressively from 0 -> 1
    st = se2.state()
    se2.step_controller(st, 'w', 0.02, now=1.0)
    now_w0 = dict(base_now, twist=(st['v_x'], 0.0, 0.0), velocity=(0.0, 0.02), speed=0.02, doing='#204')
    spec_w0 = arrows.describe(now_w0, st)
    for k in range(7):
        se2.step_controller(st, 'w', 0.05, now=1.02 + (k + 1) * 0.05)
    spec_w_mid = arrows.describe(now_w0, st)
    for k in range(40):
        se2.step_controller(st, 'W', 0.05, now=1.40 + (k + 1) * 0.05)
    now_w_fast = dict(base_now, twist=(st['v_x'], 0.0, 0.0), velocity=(0.0, st['v_x']), speed=st['v_x'], doing='jog')
    spec_w1 = arrows.describe(now_w_fast, st)
    report.check('W / Shift+W arrow starts hollow and fills progressively as actual velocity reaches setpoint',
                 spec_w0['kind'] == 'locomotion' and abs(spec_w0['origin'][0]) < 0.05
                 and 0.0 < spec_w0['fill'] < 0.20 < spec_w_mid['fill'] < 0.85 < spec_w1['fill'] <= 1.0
                 and spec_w1['setpoint'][0] > spec_w0['setpoint'][0]
                 and len(spec_w0['frame_segments']) > 10 and len(spec_w1['fill_segments']) > len(spec_w0['fill_segments']),
                 'fill0=%.2f, fill_mid=%.2f, fill1=%.2f' % (spec_w0['fill'], spec_w_mid['fill'], spec_w1['fill']))

    # 3. Composite WA turn curves arrow tip toward +X (left) and never flickers to pure side arrow on single-key repeats
    st_wa = se2.state()
    t_wa = 2.0
    for tok in ('wa', 'a', 'a', 'w', 'a', 'a'):
        t_wa += 0.05
        se2.step_controller(st_wa, tok, 0.05, now=t_wa)
        spec_wa = arrows.describe(dict(base_now, doing='turn_left'), st_wa)
        assert spec_wa['setpoint'][0] > 0.5 and spec_wa['setpoint'][1] == 0.0 and spec_wa['setpoint'][2] > 0.4
    tip_x = max(p[0] for seg in spec_wa['frame_segments'] for p in seg)
    report.check('WA curved arrow frame bends toward left (+X > 0.15m) without flickering to pure side arrow',
                 spec_wa['kind'] == 'locomotion' and tip_x > 0.15 and spec_wa['setpoint'][1] == 0.0,
                 'tip_x=%.3f, setpoint=%s' % (tip_x, spec_wa['setpoint']))

    # 4. Jump draws upward 3D arrow with actual fill
    spec_hop = arrows.describe(dict(base_now, doing='throw', where=(0.0, 1.02, 0.0)))
    max_y = max(p[1] for seg in spec_hop['frame_segments'] for p in seg)
    report.check('jump (hop) draws upward 3D arrow frame and fill',
                 spec_hop['kind'] == 'jump' and max_y > 0.75 and len(spec_hop['fill_segments']) > 5,
                 'max_y=%.2f, fill=%.2f' % (max_y, spec_hop['fill']))

    # 5. Commanded stop draws braking arrow and transverse stop bar
    spec_stop = arrows.describe(dict(base_now, doing='brake', velocity=(0.0, 0.45), speed=0.45))
    report.check('commanded stop (brake) draws stop bar target and braking arrow',
                 spec_stop['kind'] == 'stop' and len(spec_stop['target_segments']) >= 8
                 and spec_stop['landing'][2] > 0.25)

    # 6. Commanded fall / shove draws estimated landing position on floor
    spec_fall = arrows.describe(dict(base_now, doing='controlled_fall', velocity=(0.35, 0.50), where=(0.0, 0.82, 0.0)))
    report.check('commanded fall estimates floor touch-down position and draws target ring + trajectory',
                 spec_fall['kind'] == 'fall' and spec_fall['landing'] is not None
                 and spec_fall['landing'][2] > 0.20 and len(spec_fall['target_segments']) >= 16)

    # 7. Braille render includes arrow layers cleanly
    art_plain = gynoid.render(base_now['angles'], 64, 28, colour=False)
    art_arrow = gynoid.render(base_now['angles'], 64, 28, colour=False, arrows=(now_w_fast, st))
    report.check('gynoid.render draws movement arrows into braille viewport', art_plain != art_arrow)


ROSTER = (test_the_humanoid_pages_mouse, test_the_humanoid_pages_keys,
          test_a_held_space_charges_a_push, test_r_records_and_saves,
          test_vision_scanner_docked_pip_and_affordances,
          test_tpw_controls,
          test_surface_arrows_setpoint_actual_and_landing_zones)


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
