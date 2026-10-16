#!/usr/bin/env python3
"""Her scenarios that hold (`tools/sim/scenarios.py`): each placed, commanded as a player would,
judged by the world; a scenario joins HELD once it meets its goal, docs/TODO.md the rest."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The scenarios she meets: into a wall and a tree, over a sill, a slick patch, snow, asphalt.
HELD = ('wall', 'tree', 'sill', 'slip', 'snow', 'asphalt')


def _scenario(name):
    """A test of one scenario: run as the page runs her, its goal met."""
    def test(report):
        from tools.sim import scenarios
        got = scenarios.ran(name)
        report.check('%s: %s %s' % (name, got['goal'], got['aim']), got['met'],
                     'fell at %s, went %.2f m, rose %.2f m, ended %.2f m/s, bore %s' % (
                         got['fell_at'], got['went'], got['rose'], got['speed'], got['bore']))
    test.__name__ = 'test_' + name
    return test


ROSTER = [_scenario(name) for name in HELD]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
