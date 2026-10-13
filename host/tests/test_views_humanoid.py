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
    report.check('WASD in Quake mode drives forward, back, strafe left and right',
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


ROSTER = (test_the_humanoid_pages_mouse, test_the_humanoid_pages_keys,
          test_a_held_space_charges_a_push, test_r_records_and_saves)


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
