#!/usr/bin/env python3
"""The world's heat under an emulated board on native://: the tour's rooms, the demo's SOA.

The world laid in the tour's rooms (coaxial.model.rooms) at a raised haste, the thermal page's
load on it: the identification STABLE in the first room, the tour on to the next, the board
cooling or warming faster than the observer expects and the state lost, then STABLE again. And
the rotor page's demo, its loads on the world: the switches into their SOA and the envelope
throttling them there, no node past its ceiling - the bench's words, 2026-09-28. The tour's
deadlines are on the observer's own clock: a slow host stretches the wall's time, not the
world's.

    python -X utf8 tests/test_native_heat.py
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

#: The world's and the observer's haste, three times the pages': at 100 STABLE in the first
#: room at 2 726 observer s, the room moved 54 s on, UNCERTAIN 54 s after it and STABLE again
#: 850 later, but the observer's steps starve the board's loop there (2026-09-28).
HASTE = 30

#: The thermal page's load, observer s and A: 120 on, 240 off.
ON_S, OFF_S, AMPS = 120.0, 240.0, 30.0 * math.sqrt(2.0)

#: Each stage's deadline, observer s: three times the measured, at the least.
FIRST_S, MOVE_S, LOSE_S, FIND_S = 9000, 600, 600, 4000

#: The wall's budget, s, and how often the identification is read, wall s. Where the host runs
#: the world's clock under half its haste the tour is not judged: CI's 3.12 runner lost the room
#: at 2 929 observer s and its 300 s ran out before STABLE came back (3fe01e9).
WALL_S, READ_S = 400, 0.5

#: The rotor page's haste and its wall s, from DYNO, the first loaded segment - SPIN's rotor is
#: unloaded: at 30 the envelope throttled 6.0 s into the first spin-up then, the worst node
#: 0.93 of its span and 0.945 at most; at 100 the page drove nothing, the board holding real
#: time at 1 002 observer steps a second (2026-09-28). Its own board on the page's dyno - 63 V,
#: a fan's sink: the tour's room and haste stay on theirs.
SOA_HASTE, SOA_S, SOA_PORT, SOA_FROM = 30, 25.0, 'native://?world=dyno', 'DYNO'


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
        began_s = board.thermal.state()['seconds']
        now = began_s
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
    rate = (now - began_s) / max(1e-9, time.monotonic() - began)
    if 'found' not in marks and rate < 0.5 * HASTE:
        report.skip('the tour', 'the host ran the world at %.0f observer s a wall s of %d, %s'
                    % (rate, HASTE, said))
        return
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


def test_the_demo_takes_the_switches_into_their_soa(report):
    """The rotor page's demo on native, its loads on the world - the propeller its drag, the
    stages' torque: the envelope throttles the switches past 0.9 of their span in the first
    spin-up and holds every node under its ceiling. On the flywheel's current alone native's
    demo stood at 0.85 and never throttled (2026-09-28)."""
    from coaxial.model import thermal
    from terminal.ui.screen import FPS_CAP
    from terminal.views import show_rotor_observer as view
    from terminal.views.rotor import motions
    from tools.render import page

    rows, real, was, began = [], view.compose, thermal.HASTE, motions.START_AT

    def compose(rig, origin, console, v):
        out = real(rig, origin, console, v)
        budget = v.get('budget') or {}
        if budget:
            rows.append((v.get('stage'), budget.get('worst') or 0.0,
                         bool(budget.get('throttling')), bool(budget.get('tripped'))))
        return out
    view.compose, thermal.HASTE, motions.START_AT = compose, SOA_HASTE, SOA_FROM
    try:
        page.frame('rotor_observer', 150, 44, frames=int(SOA_S * FPS_CAP), port=SOA_PORT)
    finally:
        view.compose, thermal.HASTE, motions.START_AT = real, was, began
    first = next((r for r in rows if r[2]), None)
    worst = max((r[1] for r in rows), default=0.0)
    report.check('the envelope throttles the switches in the dyno\'s run, past 0.9 of their '
                 'span',
                 first is not None and first[1] >= 0.9,
                 'first at %s, %.2f of the span' % (first[0], first[1]) if first
                 else 'never, %.2f at most in %d reads' % (worst, len(rows)))
    report.check('and holds every node under its ceiling, none tripped',
                 rows and worst <= 1.0 and not any(r[3] for r in rows),
                 '%.3f at most' % worst)


def test_a_world_mounts_its_board(report):
    """A world names its link and what its board is mounted in (board/emu/worlds/dyno.json):
    the rig tells the board, its link reads the world's volts; told another, the board's
    world is laid so as well and the identification starts over on it."""
    from tools.emu import world as worlds
    dyno = worlds.load('dyno')
    rig = Coaxial63100(port='native://?world=dyno').open()
    try:
        board = rig.board
        board.afe.on()
        board.transport.sleep(0.3)
        told = board.thermal.identification()['application']
        volts = board.analog.dcbus_voltage()['volts']
        laid = rig.board.transport.serial.mount
        board.thermal.configure(application='immersion_oil')
        moved = board.thermal.identification()
        board.thermal.configure(application=worlds.mounted(dyno))
    finally:
        rig.close()
    report.check('the dyno\'s board told its sink, its link at the world\'s %.0f V' % dyno['link'],
                 told == worlds.mounted(dyno) and abs(volts - dyno['link']) <= 0.02 * dyno['link'],
                 '%s, %.1f V' % (told, volts))
    report.check('told another, it and its world are in it, the identification begun again',
                 moved['application'] == 'immersion_oil' and moved['state'] == 'UNCERTAIN'
                 and callable(laid),
                 '%s, %s' % (moved['application'], moved['state']))


ROSTER = (test_the_tour_loses_and_finds_the_room, test_the_demo_takes_the_switches_into_their_soa,
          test_a_world_mounts_its_board)


def main(argv=None):
    """Every test, or those the command line's words name (tools.dev.focus)."""
    report = Report()
    if find_cc() is None or os.environ.get('COAXIAL_GCOV'):
        # Built for gcov the C runs unoptimised: the world's pace is not the board's.
        print('  SKIP  no C compiler for the board layer, or built for gcov')
        print('\n0 passed, 0 failed, 1 skipped')
        return 0
    watchdog(WALL_S + SOA_S + 120)
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
