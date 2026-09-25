#!/usr/bin/env python3
"""The board layer's drive path on this host, in real time: `native://`.

A rig on `native://` (tools.cores.native), the real-time engine for SIL and HIL: comms/ and
board/src's stage, triple, meter and drive built for this host over board/native's TIM1, ADC1-3
and front end, the world's plant under them, a thread holding the board's clock to the wall's.
It stands as an emulated board, keeps real time and turns the demo motor on every period of
the timer. The firmware against its sensors and timers is test_emulator's, on Renode.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100  # noqa: E402
from coaxial.comm.session import standing  # noqa: E402
from terminal.ui.demo import stop_motor, turn_motor  # noqa: E402
from tools.cores import native  # noqa: E402
from tools.cores.build import find_cc  # noqa: E402

#: The wall seconds each window runs.
WINDOW_S = 1.0


class Report:
    def __init__(self):
        self.passed = self.failed = 0

    def check(self, name, ok, detail=''):
        if ok:
            self.passed += 1
            print('  PASS  %-60s %s' % (name, detail))
        else:
            self.failed += 1
            print('  FAIL  %-60s %s' % (name, detail))


def test_it_stands_as_an_emulated_board(report, rig):
    info = rig.board.version_info
    report.check('native:// stands as an emulated board, the firmware answering',
                 standing(rig.origin) == 'emulated' and info['device'] == 'coaxial_63100',
                 '%s, %s' % (rig.origin.label, info['firmware']))


def test_the_clock_keeps_the_wall(report, rig):
    serial = rig.board.transport.serial
    v0, w0 = serial.virtual_seconds(), time.monotonic()
    time.sleep(WINDOW_S)
    v1, w1 = serial.virtual_seconds(), time.monotonic()
    report.check('the board\'s clock keeps the wall\'s, within TRAIL_S',
                 abs((v1 - v0) - (w1 - w0)) <= native.TRAIL_S,
                 '%.4f virtual s in %.4f wall s' % (v1 - v0, w1 - w0))


def test_the_demo_motor_turns_in_real_time(report, rig):
    b = rig.board
    step = turn_motor(rig, rig.origin)
    if step is None:
        report.check('the demo drives native://', False, 'it took the board for a real one')
        return
    try:
        b.transport.sleep(WINDOW_S / 2)
        step()
        g0, w0 = b.gate_drivers.state(), time.monotonic()
        b.transport.sleep(WINDOW_S)
        step()
        g1, w1 = b.gate_drivers.state(), time.monotonic()
        d = b.drive.state()
    finally:
        stop_motor(rig)
    report.check('the drive holds its vector on the injected triple',
                 d['mode'] == 'hold' and g1['sync_armed'] and d['owns_compares'],
                 'mode %s, sync %s' % (d['mode'], g1['sync_armed']))
    updates, wall = g1['updates'] - g0['updates'], w1 - w0
    report.check('a triple every period of the timer, a wall second\'s worth a wall second',
                 abs(updates * d['ts'] - wall) <= native.TRAIL_S + d['ts'],
                 '%d updates in %.4f wall s, a period %.0f us' % (updates, wall, d['ts'] * 1e6))
    report.check('and none overran', g1['overruns'] == 0, '%d overruns' % g1['overruns'])


def main():
    report = Report()
    if find_cc() is None:
        print('no C compiler: the board layer cannot be built for this host')
        print('\n0 passed, 0 failed')
        return 0
    rig = Coaxial63100(port='native://').open()
    try:
        for test in (test_it_stands_as_an_emulated_board, test_the_clock_keeps_the_wall,
                     test_the_demo_motor_turns_in_real_time):
            print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
            test(report, rig)
    finally:
        rig.close()
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
