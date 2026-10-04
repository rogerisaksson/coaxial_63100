"""The gynoid standing, as the page runs her with its stand held (`Director(stand_s=inf)`): the
floor's rigs under her (`machine.events.STANDING`), a nudge from each side, and she keeps her
feet (`machine.stand`). Her walk is test_gynoid.py's, her falls test_gynoid_falls.py's."""
import math
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report

#: Nudged at NUDGE_S, she stands to TO_S: her trunk never past HELD_DEG from upright.
NUDGE_S, TO_S, HELD_DEG = 5.0, 12.0, 25.0


def _stood(event, k):
    """(fell, her trunk's worst tilt deg, steps taken) standing to TO_S, `event` befalling her
    at NUDGE_S (`gait_montecarlo`'s stand trial)."""
    from machine import Machine, events
    from machine.director import Director
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    world = body.nodes['pelvis'].world
    director = Director(body, 0.85, stand_s=math.inf)
    up, stagger = events.rigged(event)
    director.begin(up=up, stagger=stagger)
    events.rig(event, director, world)
    body.loop.step(0.0)
    bus, laid, tilt, fell = body.loop.bus, False, 0.0, False
    while bus['t'] < TO_S and not fell:
        if not laid and bus['t'] >= NUDGE_S:
            events.befall(event, director, world, k)
            laid = True
        body.loop.write(**director.step(0.001))
        body.loop.step(0.001)
        fell = director.stage in ('falling', 'fallen')
        if laid:
            tilt = max(tilt, director._tilt(bus))
    body.close()
    return fell, tilt, director.treads


def test_nudged_standing_she_keeps_her_feet(report):
    """Standing, nudged from her left, her right, behind and ahead (`events.SHOVES` 'nudge'),
    she stands on to TO_S, her trunk within HELD_DEG of upright."""
    for event, k, way in (('nudge', 0, 'left'), ('nudge', 1, 'right'),
                          ('nudge_on', 0, 'behind'), ('nudge_on', 1, 'ahead')):
        fell, tilt, treads = _stood(event, k)
        report.check('nudged from %s she stands, her trunk within %.0f deg' % (way, HELD_DEG),
                     not fell and tilt <= HELD_DEG,
                     '%s, tilt %.1f deg, %d steps' % ('fell' if fell else 'stood', tilt, treads))


def test_on_the_board_nudged_she_keeps_her_feet(report):
    """On the balance board, stiff (`floor.BOARD_K`), nudged across its rocking and along it,
    she stands on."""
    for event, k, way in (('board', 0, 'rocking left'), ('board_on', 0, 'rocking ahead')):
        fell, tilt, treads = _stood(event, k)
        report.check('on the stiff board, nudged %s, she stands' % way,
                     not fell and tilt <= HELD_DEG,
                     '%s, tilt %.1f deg, %d steps' % ('fell' if fell else 'stood', tilt, treads))


ROSTER = (test_nudged_standing_she_keeps_her_feet, test_on_the_board_nudged_she_keeps_her_feet)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
