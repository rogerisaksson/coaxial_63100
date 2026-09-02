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


def main():
    report = Report()
    print('\n-- every view, two frames, no board --')
    test_each_view_draws_two_frames(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
