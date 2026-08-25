#!/usr/bin/env python3
"""The stand-in must answer in the same shape as the board.

Everything that falls back to `coaxial.simulated` when no board answers rests
on one assumption: a caller cannot tell the difference except in the numbers.
This suite is that assumption, checked - the same calls against the debug
probe and against the stand-in, compared with every number masked out.

The numbers are expected to differ and are never compared. What must match is
the shape: the same channels, the same directions, the same columns, the same
rows in the same order. A divergence here is a stand-in that would let a host
change pass on the desk and fail at the bench - which is exactly what it
already did once: the map came back with no units for NTC and DC bus, where
the board reports centi-degC and mV.

Needs a board. With none, both sides would be the stand-in and the comparison
would be trivially true, so it says so and skips rather than passing.

Run from the host directory:  python tests/test_parity.py
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial.simulated import SimulatedSession                # noqa: E402
from coaxial_mcp import tools as toolmod                      # noqa: E402
from coaxial_mcp.session import open_session                  # noqa: E402

# Any run of digits, with or without a sign or a decimal point. Masking rather
# than rounding: 24.614V and 39.075V are both a bus voltage and neither is
# wrong, so the value is not what this suite is about.
NUMBER = re.compile(r'[-+]?\d+(?:\.\d+)?')


class Report:
    def __init__(self):
        self.passed = 0
        self.failed = 0

    def check(self, name, condition, detail=''):
        if condition:
            self.passed += 1
            print('  PASS  %-46s %s' % (name, detail))
        else:
            self.failed += 1
            print('  FAIL  %-46s %s' % (name, detail))


def shape(text):
    """`text` as fields, with every number replaced by `#`.

    Whitespace is collapsed after masking, not before: the columns are
    right-aligned, so `-8689.9` and `-40.8` pad differently and a straight
    mask left two lines differing by one space - a difference in the width
    of a number this suite has already decided not to compare. What is left
    is the field order, which is the thing.
    """
    out = []
    for line in str(text).splitlines():
        out.append(' '.join(NUMBER.sub('#', line).split()))
    return out


def first_difference(left, right):
    """The first line the two disagree on, for the detail column."""
    for i, (a, b) in enumerate(zip(left, right)):
        if a != b:
            return 'line %d: %r vs %r' % (i + 1, a[:30], b[:30])
    if len(left) != len(right):
        return '%d lines vs %d' % (len(left), len(right))
    return 'identical'


def compare(report, what, left, right, skip=0):
    """Both renders, numbers masked, line for line past `skip` lines."""
    a, b = shape(left)[skip:], shape(right)[skip:]
    report.check('%s: same number of rows' % what, len(a) == len(b),
                 '%d vs %d' % (len(a), len(b)))
    report.check('%s: same shape once the numbers are masked' % what, a == b,
                 first_difference(a, b))


def main():
    board, origin = open_session()
    report = Report()

    if not origin.real:
        print('no board answered - both sides would be the stand-in, so there '
              'is nothing to compare')
        print('\n0 passed, 0 failed')
        return 0

    stand_in = SimulatedSession()
    print('-- %s against a simulated board --' % origin.label)
    try:
        # The map first: it is the one thing that must match exactly, numbers
        # included. A channel the stand-in does not have, or has facing the
        # other way, is not a difference in measurement.
        live_map = board.board.system.channel_map()
        fake_map = stand_in.board.system.channel_map()
        for section in ('analog', 'digital', 'reserved'):
            report.check('channel map, %s: identical, numbers and all'
                         % section, live_map[section] == fake_map[section],
                         '%d vs %d rows' % (len(live_map[section]),
                                            len(fake_map[section])))

        # board_info: the first three lines are identity, clock and the
        # device's own description, all of which are meant to differ - the
        # stand-in says "simulated" where a firmware version goes and again
        # in its description, on purpose. Everything below them is the map.
        compare(report, 'board_info',
                toolmod.HANDLERS['board_info'](board),
                toolmod.HANDLERS['board_info'](stand_in), skip=3)

        #...and the description is the one line that must NOT match: a
        # stand-in a caller could mistake for the board is the whole thing
        # this suite exists to prevent.
        live_head = shape(toolmod.HANDLERS['board_info'](board))[:3]
        fake_head = shape(toolmod.HANDLERS['board_info'](stand_in))[:3]
        report.check('the stand-in says it is one, in its own description',
                     'simulated' in fake_head[2].lower()
                     and 'simulated' not in live_head[2].lower(),
                     fake_head[2][:46])

        # A reading, with the front end in the same state on both sides, so
        # the AFE-off banner is either present on both or on neither.
        for state in ('on', 'off'):
            toolmod.HANDLERS['afe_power'](board, action=state)
            toolmod.HANDLERS['afe_power'](stand_in, action=state)
            compare(report, 'analog_read, AFE %s' % state,
                    toolmod.HANDLERS['analog_read'](board, samples=16),
                    toolmod.HANDLERS['analog_read'](stand_in, samples=16))

            # The digital levels move with the AFE on both sides, so this
            # runs inside the same loop and under the same state.
            compare(report, 'digital_read, AFE %s' % state,
                    toolmod.HANDLERS['digital_read'](board),
                    toolmod.HANDLERS['digital_read'](stand_in))

        # One named channel, the path a question about the NTC takes.
        compare(report, "analog_read ch=['NTC']",
                toolmod.HANDLERS['analog_read'](board, ch=['NTC'], samples=16),
                toolmod.HANDLERS['analog_read'](stand_in, ch=['NTC'],
                                                samples=16))

        # And a refusal: a bad channel name must fail the same way on both,
        # or a host change that looks fine on the desk breaks at the bench.
        for bad in ('nosuch', 'ch9'):
            live = fake = ''
            try:
                toolmod.HANDLERS['analog_read'](board, ch=[bad])
            except Exception as exc:                          # noqa: BLE001
                live = '%s: %s' % (type(exc).__name__, exc)
            try:
                toolmod.HANDLERS['analog_read'](stand_in, ch=[bad])
            except Exception as exc:                          # noqa: BLE001
                fake = '%s: %s' % (type(exc).__name__, exc)
            report.check('ch=[%r] is refused the same way' % bad,
                         shape(live) == shape(fake),
                         first_difference(shape(live), shape(fake)))
    finally:
        toolmod.HANDLERS['afe_power'](board, action='off')
        try:
            board.close()
        except Exception:                                     # noqa: BLE001
            pass

    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
