#!/usr/bin/env python3
"""An emulated board's world in rooms on native://: the thermal tour.

The world laid in the tour's rooms (coaxial.model.rooms) at a raised haste, the thermal page's
load on it: the identification STABLE in the first room, the tour on to the next, the board
cooling or warming faster than the observer expects and the state lost, then STABLE again - the
bench's words, 2026-09-28. Every deadline is on the observer's own clock: a slow host stretches
the wall's time, not the world's.

    python -X utf8 tests/test_native_rooms.py
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100  # noqa: E402
from coaxial.model import rooms  # noqa: E402
from terminal.ui.demo import cycle_motor, stop_motor  # noqa: E402
from tools.cores.build import find_cc  # noqa: E402
from tools.dev.focus import chosen, watchdog  # noqa: E402
from views_kit import Report  # noqa: E402

#: The world's and the observer's haste, the pages' ten times: measured STABLE in the first room
#: at 2 726 observer s, the room moved 54 s on, UNCERTAIN 54 s after it and STABLE again 850
#: later (2026-09-28).
HASTE = 100

#: The thermal page's load, observer s and A: 120 on, 240 off.
ON_S, OFF_S, AMPS = 120.0, 240.0, 30.0 * math.sqrt(2.0)

#: Each stage's deadline, observer s: three times the measured, at the least.
FIRST_S, MOVE_S, LOSE_S, FIND_S = 9000, 600, 600, 4000

#: The wall's watchdog, s, and how often the identification is read, wall s.
WALL_S, READ_S = 300, 0.5


def test_the_tour_loses_and_finds_the_room(report):
    """STABLE in the first room, the tour on once it is earned, the state lost to the next room
    and STABLE again there, the room identified nearer the new than the old."""
    rig = Coaxial63100(port='native://').open()
    marks = {}
    try:
        board = rig.board
        board.thermal.configure(clock=HASTE)
        board.transport.serial.heat_clock(HASTE)
        board.afe.on()
        board.thermal.configure(sample_every_s=2.0)
        first = rig.thermal.situation('tour')['situation']
        motor = cycle_motor(rig, rig.origin, ON_S / HASTE, OFF_S / HASTE, AMPS)
        began = read_at = time.monotonic()
        while 'found' not in marks and time.monotonic() - began < WALL_S:
            if motor is not None:
                motor()
            time.sleep(0.05)
            if time.monotonic() - read_at < READ_S:
                continue
            read_at = time.monotonic()
            ident = rig.thermal.identification()
            now, room = board.thermal.state()['seconds'], ident['truth']['situation']
            stable = ident['state'] == 'STABLE'
            if 'stable' not in marks:
                if stable and room == first:
                    marks['stable'] = now
                elif now > FIRST_S:
                    break
            elif 'moved' not in marks:
                if room != first:
                    marks['moved'], marks['room'] = now, room
                elif now > marks['stable'] + MOVE_S:
                    break
            elif 'lost' not in marks:
                if not stable:
                    marks['lost'] = now
                elif now > marks['moved'] + LOSE_S:
                    break
            elif stable:
                marks['found'], marks['ambient'] = now, ident['ambient']
            elif now > marks['lost'] + FIND_S:
                break
    finally:
        try:
            stop_motor(rig)
        finally:
            rig.close()
    said = ', '.join('%s %s' % (k, ('%.1f' % v) if isinstance(v, float) else v)
                     for k, v in marks.items())
    report.check('STABLE in the %s room within %d observer s' % (first, FIRST_S),
                 'stable' in marks, said)
    report.check('the tour on once the room is earned: STABLE held %.0f s, within %d'
                 % (rooms.TOUR_STABLE_S, MOVE_S),
                 'moved' in marks
                 and rooms.TOUR_STABLE_S <= marks['moved'] - marks['stable'] <= MOVE_S, said)
    report.check('the state lost to the next room within %d observer s' % LOSE_S,
                 'lost' in marks, said)
    old = rooms.SITUATIONS[first]['ambient']
    new = rooms.SITUATIONS[marks['room']]['ambient'] if 'room' in marks else old
    report.check('and STABLE again within %d, the room identified nearer %.0f than %.0f C'
                 % (FIND_S, new, old),
                 'found' in marks and abs(marks['ambient'] - new) < abs(marks['ambient'] - old),
                 said)


ROSTER = (test_the_tour_loses_and_finds_the_room,)


def main(argv=None):
    """Every test, or those the command line's words name (tools.dev.focus)."""
    report = Report()
    if find_cc() is None or os.environ.get('COAXIAL_GCOV'):
        # Built for gcov the C runs unoptimised: the world's pace is not the board's.
        print('  SKIP  no C compiler for the board layer, or built for gcov')
        print('\n0 passed, 0 failed, 1 skipped')
        return 0
    watchdog(WALL_S + 60)
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
