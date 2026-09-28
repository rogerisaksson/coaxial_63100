"""The views suites' kit: the report, the pages under terminal/views/, a page run for two frames."""
import os
import subprocess
import sys


HERE = os.path.dirname(os.path.abspath(__file__))


HOST = os.path.dirname(HERE)


#: Every view, with the flags its two-frame run needs. Read off terminal/views/
#: rather than hardcoded where possible - a new show_*.py joins by existing.
EXTRA = {
    'show_session.py': [],
    'menu.py': [],
    'show_capture.py': [],
    'show_desk.py': [],
    'show_gate_drivers.py': [],
    'show_orientation.py': ['--width', '72', '--height', '14'],
    'show_angle.py': ['--scales'],
    'show_thermal_observer.py': [],
    'show_rotor_observer.py': [],
}


class Report:
    def __init__(self):
        self.passed = self.failed = self.skipped = 0

    def check(self, name, ok, detail=''):
        self.passed += bool(ok)
        self.failed += (not ok)
        print('  %s  %-58s %s' % ('PASS' if ok else 'FAIL', name, detail))

    def skip(self, name, why):
        self.skipped += 1
        print('  SKIP  %-58s %s' % (name, why))


def views():
    """The views under terminal/views/, plus the session and the front page."""
    got = sorted(name for name in os.listdir(os.path.join(HOST, 'terminal', 'views'))
                 if name.startswith('show_') and name.endswith('.py'))
    return ['show_session.py', 'menu.py'] + got


#: A view gets this long to draw its two frames and exit.
VIEW_TIMEOUT = 120


def run_view(name):
    """One view run against the stand-in for two frames; its whole process
    tree killed at the timeout - a view's crew workers held the captured
    pipe otherwise, and the suite sat on it (2026-09-13)."""
    from tools.dev.runner import run_captured
    where = 'terminal' if name == 'menu.py' else os.path.join('terminal', 'views')
    done = run_captured(
        [sys.executable, '-X', 'utf8', os.path.join(where, name),
         '--simulated', '--frames', '2'] + EXTRA.get(name, []),
        VIEW_TIMEOUT, cwd=HOST)
    if done is None:
        raise subprocess.TimeoutExpired(name, VIEW_TIMEOUT)
    return done


def rows_of(owner, width, height, kind):
    """Which rows of a rendered drawing carry `kind`."""
    return [row for row in range(height)
            if any(owner[row][col] == kind for col in range(width))]


def ansi_plain(text):
    """`text` without its colour escapes."""
    import re
    return re.sub(r'\x1b\[[0-9;]*m', '', text)
