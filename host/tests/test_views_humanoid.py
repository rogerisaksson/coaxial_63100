"""The HUMANOID page's hands: its mouse on the view, its keys on her pace, the one law, her manners."""
import sys

from tools.dev.focus import chosen
from views_kit import Report


def test_the_humanoid_pages_mouse(report):
    """The HUMANOID page holds the mouse, F its pace: a right drag moves the view by its cells
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
    report.check('a left drag to the right turns her the way the left arrow does (the user, '
                 '2026-10-03)', state['yaw'] < viewpoint.YAW, '%s' % state)


def test_the_humanoid_pages_pace(report):
    """S and F step the row asked of her way on the one law (`machine.pace`), from her stand to
    the run and no further; J hands her to the walk as built, S and F then its cadence; a floor
    event hands her to it too, a rig stands her on the law; there M picks one of its manners
    (`gaits.MANNERS`) and Z and X move that one's amount."""
    import types
    from machine import gaits
    from terminal.views import humanoid_keys, viewpoint
    sent = []
    body = types.SimpleNamespace(send=lambda **command: sent.append(command))
    keys = humanoid_keys.table(viewpoint.KEYS, ('strong',), ('torque',))
    state: dict = {'body': body, 'cadence': 0.85, 'law': True, 'pace': 0.0, 'rig': None}
    humanoid_keys.act_on(keys, list('fffffsssssssss'), state)
    asked = [command.get('pace') for command in sent]
    levels = humanoid_keys.LEVELS
    report.check('F her rows up to the run and no further, S down to her stand and no further',
                 asked == list(levels[4:]) + [levels[-1]] + list(levels[-2::-1]) + [levels[0]] * 2,
                 '%s' % asked)
    del sent[:]
    humanoid_keys.act_on(keys, list('jf'), state)
    report.check('J the walk as built, F then a step of its cadence',
                 sent == [{'pace': None}, {'cadence': 0.9}] and not state['law'], '%s' % sent)
    del sent[:]
    humanoid_keys.act_on(keys, list('j27'), state)
    report.check('J back on the law at its walk; a floor event hands her to the walk as built',
                 sent[:3] == [{'pace': 0.0}, {'pace': None}, {'event': 'rug'}], '%s' % sent[:3])
    del sent[:]
    humanoid_keys.act_on(keys, list('j8'), state)
    report.check('on the law a rig stands her till F',
                 sent[-1] == {'pace': -1.0, 'rig': 'brick_on'} and state['pace'] == -1.0,
                 '%s' % sent)
    del sent[:]
    humanoid_keys.act_on(keys, list('xxmxz'), state)
    first, second = list(gaits.MANNERS)[:2]
    report.check('on the law X is a step more of the manner picked, M picks the next, each '
                 'keeps its own, and Z takes one out',
                 [c['manner'] for c in sent] == [((first, 0.25),), ((first, 0.5),),
                                                 ((first, 0.5), (second, 0.25)), ((first, 0.5),)],
                 '%s' % sent)
    del sent[:]
    humanoid_keys.act_on(keys, list('jm'), state)
    report.check('J lets her manners go before the walk as built has her; M there hands her '
                 'to the law, its pick kept',
                 sent == [{'manner': ()}, {'pace': None}, {'pace': 0.0}] and state['law']
                 and state['manner'] == second and not state['manners'], '%s' % sent)


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


ROSTER = (test_the_humanoid_pages_mouse, test_the_humanoid_pages_pace,
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
