"""Every live view runs two frames against the stand-in, as a subprocess.

The suite that was missing while the views were restyled: four separate
breaks in one afternoon - a NameError in a compose, a view inheriting
power_afe=False and refusing at open, the toon path silently falling back
to the photographic renderer, dead --watch plumbing - and every one was
found by running the view by hand, because nothing else runs them at all.

Subprocesses, not imports: a view's crash on a real console involves its
own argument parsing, its preflight and its teardown, and importing main()
skips the first of those. --simulated so no board and no ollama; --frames 2
so the loop, the painter and the teardown all execute once.
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HOST = os.path.dirname(HERE)

#: Every view, with the flags its two-frame run needs. Read off tools/
#: rather than hardcoded where possible - a new show_*.py joins by existing.
EXTRA = {
    'show_session.py': [],
    'menu.py': [],
    'show_capture.py': [],
    'show_desk.py': [],
    'show_gate_drivers.py': [],
    'show_orientation.py': ['--width', '72', '--height', '14'],
    'show_angle.py': [],
    'show_thermal_observer.py': [],
    'show_rotor_observer.py': [],
}


class Report:
    def __init__(self):
        self.passed = self.failed = 0

    def check(self, name, ok, detail=''):
        self.passed += bool(ok)
        self.failed += (not ok)
        print('  %s  %-58s %s' % ('PASS' if ok else 'FAIL', name, detail))


def views():
    """The scripts under tools/ that are views, plus the session."""
    got = sorted(name for name in os.listdir(os.path.join(HOST, 'tools'))
                 if name.startswith('show_') and name.endswith('.py'))
    return ['show_session.py', 'menu.py'] + got


def run_view(name):
    env = dict(os.environ, PYTHONIOENCODING='utf-8')
    return subprocess.run(
        [sys.executable, '-X', 'utf8', os.path.join('tools', name),
         '--simulated', '--frames', '2'] + EXTRA.get(name, []),
        cwd=HOST, env=env, capture_output=True, text=True,
        encoding='utf-8', errors='replace', timeout=120)


def test_each_view_draws_two_frames(report):
    for name in views():
        done = run_view(name)
        tail = (done.stdout + done.stderr).strip().splitlines()
        last = tail[-1][:70] if tail else 'no output at all'
        report.check('%s exits 0 simulated' % name,
                     done.returncode == 0,
                     'exit %d: %s' % (done.returncode, last))
        report.check('%s raised nothing' % name,
                     'Traceback' not in done.stdout + done.stderr, last)


def rows_of(owner, width, height, kind):
    """Which rows of a rendered drawing carry `kind`."""
    return [row for row in range(height)
            if any(owner[row][col] == kind for col in range(width))]


def test_the_instruments_stand_clear_of_the_machine(report):
    """The gutters equidistant, and the foot gauges off the can.

    BOTH FOUND BY EYE, which is the whole reason for the test. The left
    group is six thermometers and the right four, and a machine centred
    in the box sat one column off the left group and two off the right -
    it reads as a drawing that is not quite straight. Then the winding
    gauge was drawn along row 16 of nineteen with the can's own bottom
    edge on the same row, and the level ran through the machine.

    Rendered at the view's own size, because both faults are the size's:
    at another width the machine misses the gutters by luck.
    """
    sys.path.insert(0, HOST)
    from coaxial import machine
    from tools import show_rotor_observer as view

    width, height = view.ART_WIDTH, view.ART_ROWS
    n_left, n_right = len(view.SOA_NODES), len(view.BOARD_NODES)
    _, owner = machine._raster(
        6.0, 24, 28, width, height, None, None, None,
        [(0.4, machine.SOA_OK)] * n_left, [(0.3, machine.SOA_OK)] * n_right,
        (0.5, machine.SOA_OK),
        [(0.4, machine.SOA_WARN), (0.3, machine.WATTS)], 2.0)
    can = [col for row in range(height) for col in range(width)
           if owner[row][col] == machine.CAN]
    left, right = machine.gutters(width, height, n_left, n_right)
    gaps = (min(can) - max(left) - 1, min(right) - max(can) - 1)
    report.check('the gutters stand the same distance off the machine',
                 gaps[0] == gaps[1], 'left %d, right %d columns' % gaps)
    report.check('both groups fit inside the frame',
                 len(left) == n_left and len(right) == n_right,
                 '%d of %d left, %d of %d right'
                 % (len(left), n_left, len(right), n_right))

    can_rows = rows_of(owner, width, height, machine.CAN)
    for name, kind in (('winding', machine.SOA_WARN),
                       ('power', machine.WATTS)):
        on = rows_of(owner, width, height, kind)
        report.check('the %s gauge clears the can' % name,
                     bool(on) and not set(on) & set(can_rows),
                     'gauge rows %s, can rows %d..%d'
                     % (on, min(can_rows), max(can_rows)))


def test_each_gutter_says_its_hottest_node(report):
    """The caption's third row, in degrees, per group.

    A share of a ceiling is what the board acts on and it is not a
    temperature: the tubes cannot say 118 C and a bench asks for exactly
    that. The check is that the figure is the HOTTEST of its group and
    not the first or the mean - fed one hot leg among five cool ones,
    that is the one that has to appear.
    """
    sys.path.insert(0, HOST)
    from tools import show_rotor_observer as view

    nodes = dict.fromkeys(view.SOA_NODES + view.BOARD_NODES, 30.0)
    nodes['phase_v'], nodes['regulators'] = 118.4, 71.2
    said = {'thermal': {'nodes': nodes},
            'budget': {'used': {'phase_v': 0.95}, 'tripped': False}}
    peak, cls = view.hottest(said, view.SOA_NODES)
    report.check('the switch caption takes the hottest leg',
                 peak == 118.4, 'said %s' % (peak,))
    report.check('and its colour comes from that same node margin',
                 cls == view.machine.SOA_WARN, 'class %s' % (cls,))
    peak, _ = view.hottest(said, view.BOARD_NODES)
    report.check('the board caption takes the hottest board node',
                 peak == 71.2, 'said %s' % (peak,))
    report.check('a group with nothing measured says nothing',
                 view.hottest({}, view.SOA_NODES)[0] is None)


def test_the_soa_gauge_pulses_only_when_the_board_acts(report):
    """The alarm is the envelope acting, not a level this page picked.

    A red bar cannot get redder, so a stage being HELD BACK by its own
    envelope looked exactly like one sitting near a limit. The pulse is
    the difference. What it keys on is `throttling` and `tripped`, which
    the board reports out of the ceilings its record gave it - the page
    inventing a threshold to flash at would be the page judging a
    reading (invariant 10).
    """
    sys.path.insert(0, HOST)
    from tools import show_rotor_observer as view

    report.check('an idle board does not pulse', not view.flashing({}))
    report.check('nor does one merely close to a limit - near is not an '
                 'event',
                 not view.flashing({'budget': {'worst': 0.99,
                                               'throttling': False}}))

    for flag in ('throttling', 'tripped'):
        seen = set()
        until = time.monotonic() + 1.0
        while time.monotonic() < until:
            seen.add(view.flashing({'budget': {flag: True}}))
            time.sleep(0.01)
        report.check('while %s it pulses - both halves inside a second at '
                     '%.0f Hz' % (flag, view.FLASH_HZ),
                     seen == {True, False}, str(sorted(seen)))


def main():
    report = Report()
    print('\n-- every view, two frames, no board --')
    test_each_view_draws_two_frames(report)
    print('\n-- the rotor observer\'s geometry --')
    test_the_instruments_stand_clear_of_the_machine(report)
    test_each_gutter_says_its_hottest_node(report)
    test_the_soa_gauge_pulses_only_when_the_board_acts(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
