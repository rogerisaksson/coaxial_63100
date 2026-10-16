#!/usr/bin/env python3
"""Her scenarios that hold (`tools/sim/scenarios.py`): each placed, commanded as a player would,
judged by the world; a scenario joins HELD once it meets its goal, docs/TODO.md the rest."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The scenarios she meets laid turned by each of WOBBLES, rad - one met at 0 and not at
#: 2e-4 is in the margin and out: into a tree, over a sill, a slick patch, ice, snow, asphalt; a
#: player's starts and stops, turns, strafes, reversals, sprint and weave.
HELD = ('tree', 'sill', 'slip', 'ice', 'snow', 'asphalt', 'play_start_stop', 'play_turns', 'play_strafes',
        'play_reverse', 'play_sprint', 'play_weave')
WOBBLES = (0.0, 2e-4, -2e-4)


def _scenario(name, wobble):
    """A test of one scenario laid turned `wobble`: run as the page runs her, its goal met."""
    def test(report):
        from tools.sim import scenarios
        got = scenarios.ran(name, wobble)
        report.check('%s laid %+.4f rad: %s %s' % (name, wobble, got['goal'], got['aim']), got['met'],
                     'fell at %s, went %.2f m, rose %.2f m, ended %.2f m/s, bore %s' % (
                         got['fell_at'], got['went'], got['rose'], got['speed'], got['bore']))
    test.__name__ = 'test_%s_%d' % (name, WOBBLES.index(wobble))
    return test


ROSTER = [_scenario(name, wobble) for name in HELD for wobble in WOBBLES]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
