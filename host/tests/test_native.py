#!/usr/bin/env python3
"""The board layer's drive path on this host, in real time: `native://`.

A rig on `native://` (tools.cores.native), the real-time engine for SIL and HIL: comms/ and
board/src's board layer built for this host over board/native's chip, the world's plant under
it, a thread holding the board's clock to the wall's. It stands as an emulated board, keeps
real time, turns the demo motor on every period of the timer, its A1335 on the plant's shaft
and its BNO085 reading what a SIL pipes; the humanoid's twenty boards on five buses keep the
wall with every drive running. The firmware against its sensors and timers is
test_emulator's, on Renode.
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import EMULATED, Coaxial63100  # noqa: E402
from coaxial.comm.session import standing  # noqa: E402
from coaxial.devices.imu import ACCELEROMETER  # noqa: E402
from coaxial.node import discover  # noqa: E402
from terminal.ui.demo import stop_motor, turn_motor  # noqa: E402
from tools.cores import native  # noqa: E402
from tools.cores.build import find_cc  # noqa: E402

#: The wall seconds each window runs.
WINDOW_S = 1.0

#: One count of the A1335's twelve bits, degrees; one of the accelerometer's Q8, m/s^2.
ANGLE_COUNT = 360.0 / 4096
ACCEL_COUNT = 1.0 / 256

#: The humanoid's fleet on native://, a limb a bus.
BODY = 'native://?body=humanoid'


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


def test_the_parts_answer(report, rig):
    """The A1335 on the shaft the plant turned, then where a script puts it, and the BNO085's
    product id and a reading a SIL pipes, each to a count."""
    b, limb = rig.board, native.limb_for(rig.port)
    b.afe.on()
    try:
        b.transport.sleep(0.05)
        # Read between two looks at the shaft: the bench's flywheel coasts on after a drive.
        with limb.lock:
            before = limb.boards[0].lib.native_shaft_degrees() % 360.0
        read = b.angle.state().get('degrees')
        with limb.lock:
            shaft = limb.boards[0].lib.native_shaft_degrees() % 360.0
        limb.angle(1, 250.0)
        b.transport.sleep(0.05)
        degrees = b.angle.state().get('degrees')
        with b.imu.configuring():
            ident = b.imu.product_id()
        limb.imu(1, [0.0, 0.0, 3.5] + [0.0] * 9 + [1.0])
        b.imu.configure({ACCELEROMETER: 20000})
        b.transport.sleep(0.2)
        accel = (b.imu.state().get('accelerometer') or {}).get('value') or {}
    finally:
        b.afe.off()
    span = (shaft - before + 180.0) % 360.0 - 180.0
    into = (read - before + 180.0) % 360.0 - 180.0 if read is not None else math.inf
    report.check('the A1335 reads the shaft the plant turned, to a count',
                 min(0.0, span) - ANGLE_COUNT <= into <= max(0.0, span) + ANGLE_COUNT,
                 '%s read, the shaft %.3f to %.3f' % (read, before, shaft))
    report.check('and the angle put, to a count',
                 degrees is not None and abs(degrees - 250.0) <= ANGLE_COUNT, str(degrees))
    report.check('the BNO085 answers its product id', bool(ident.get('sw_version')), str(ident))
    report.check('and reads what is piped, to a count',
                 abs(accel.get('z', 0.0) - 3.5) <= ACCEL_COUNT, str(accel))


def test_the_body_keeps_the_wall(report):
    """The humanoid's twenty boards on five buses, every drive holding: each limb's clock on
    the wall's, each board a triple every period."""
    nodes = discover(port=BODY, execution_mode=EMULATED, units=range(1, 5))
    try:
        report.check('the humanoid\'s twenty boards answer on five buses', len(nodes) == 20,
                     '%d nodes' % len(nodes))
        steps = [turn_motor(n.rig, n.rig.origin) for n in nodes]
        limbs = native.limb_for(BODY) and native._LIMBS[BODY]
        try:
            u0 = [n.rig.board.gate_drivers.state()['updates'] for n in nodes]
            v0, w0 = {k: limb.seconds() for k, limb in limbs.items()}, time.monotonic()
            time.sleep(WINDOW_S)
            v1, w1 = {k: limb.seconds() for k, limb in limbs.items()}, time.monotonic()
            g1 = [n.rig.board.gate_drivers.state() for n in nodes]
            ts = nodes[0].rig.board.drive.state()['ts']
            for step in steps:
                if step is not None:
                    step()
        finally:
            for n in nodes:
                stop_motor(n.rig)
        off = {k: (v1[k] - v0[k]) - (w1 - w0) for k in v1}
        report.check('each limb\'s clock keeps the wall\'s, within TRAIL_S',
                     all(abs(e) <= native.TRAIL_S for e in off.values()),
                     ' '.join('%s %+.4f' % kv for kv in sorted(off.items())))
        short = [(n.name, g['updates'] - u) for n, g, u in zip(nodes, g1, u0)
                 if (g['updates'] - u) * ts < WINDOW_S - native.TRAIL_S or g['overruns']]
        report.check('every board a triple every period, none overrun', not short, str(short[:4]))
    finally:
        for n in nodes:
            n.close()


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
        print('\n-- the parts answer --')
        test_the_parts_answer(report, rig)
    finally:
        rig.close()
    print('\n-- the body keeps the wall --')
    test_the_body_keeps_the_wall(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
