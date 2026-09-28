#!/usr/bin/env python3
"""The board layer's drive path on this host, in real time: `native://`.

A rig on `native://` (tools.cores.native), the real-time engine for SIL and HIL: comms/ and
board/src's board layer built for this host over board/native's chip, the world's plant under
it, a thread holding the board's clock to the wall's. It stands as an emulated board, keeps
real time, turns the demo motor on every period of the timer, its A1335 on the plant's shaft
and its BNO085 reading what a SIL pipes. Named, the humanoid's twenty boards on five buses keep
the wall with every drive running - the humanoid is the stand-in's for now (2026-09-27). The
firmware against its sensors and timers is test_emulator's, on Renode.

    python -X utf8 tests/test_native.py                 # the rig's tests
    python -X utf8 tests/test_native.py sto current     # those
    python -X utf8 tests/test_native.py body            # the humanoid's fleet
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
from coaxial.model.inverter import GATE_UVLO_V  # noqa: E402
from coaxial.simulated.sto import PILOT_VOLTS  # noqa: E402
from terminal.ui.demo import stop_motor, turn_motor  # noqa: E402
from tools.cores import native  # noqa: E402
from tools.cores.build import find_cc  # noqa: E402
from tools.dev.focus import pick, watchdog  # noqa: E402

#: The wall seconds each window runs.
WINDOW_S = 1.0

#: One count of the A1335's twelve bits, degrees; one of the accelerometer's Q8, m/s^2.
ANGLE_COUNT = 360.0 / 4096
ACCEL_COUNT = 1.0 / 256

#: The humanoid's fleet on native://, a limb a bus.
BODY = 'native://?body=humanoid'

#: What each thermometer resolves, K: the NTC's 30 mK and the converter's noise, the MCU's die
#: through its factory points, the A1335's eighths.
THERMOMETER_K = {'ntc': 0.1, 'mcu': 0.2, 'afe': 0.2}

#: The demo's held vector for the current's check, A, and how near the world's mean square
#: over a second comes to the regulated current's: the ripple, ten reads of the regulated.
HELD_A = 10.0
SQUARE_SHARE = 0.03

#: How near the observer's legs keep to the world's, K: 0.3 alone, 1.2 under the offline
#: gate's load; a garbage sample or a plant hasted before its observer put them 5-6 K apart
#: (FINDINGS 2026-09-26).
LEG_K = 2.0

#: The STO chain: its time to release or trip from any input and settle, board s (1.5 ms at
#: the circuit's, world_sto.c).
STO_SETTLE_S = 0.05

#: thermal.h's node order, as the world gives them (Board.nodes).
NODES = ('driver_u', 'driver_v', 'driver_w', 'phase_u', 'phase_v', 'phase_w', 'mcu',
         'regulators', 'afe', 'board', 'hotswap', 'patch_u', 'patch_v', 'patch_w',
         'patch_left', 'patch_bottom', 'patch_right', 'winding', 'stator', 'rotor')


#: Built for gcov (tools/dev/cover.py) the C runs unoptimised and counted: its pace is not the
#: board's - 0.77 virtual s a wall s (2026-09-28) - so the checks on it stand aside.
GCOV = bool(os.environ.get('COAXIAL_GCOV'))


class Report:
    def __init__(self):
        self.passed = self.failed = self.skipped = 0

    def skip(self, name, why):
        self.skipped += 1
        print('  SKIP  %-60s %s' % (name, why))

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
    if GCOV:
        return report.skip('the board\'s clock keeps the wall\'s, within TRAIL_S',
                           'built for gcov, unoptimised')
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


def test_the_thermometers_read_the_world(report, rig):
    """The NTC, the MCU's die and the A1335's, a sample a thermal second with the rail held,
    each between two looks at the world's and within what it resolves."""
    b, limb = rig.board, native.limb_for(rig.port)
    b.afe.on()
    b.thermal.configure(sample_every_s=1.0)
    try:
        b.transport.sleep(3.0)
        with limb.lock:
            before = limb.boards[0].temperatures()
        got = b.thermal.state()
        b.transport.sleep(0.2)
        with limb.lock:
            after = limb.boards[0].temperatures()
    finally:
        b.thermal.configure(sample_every_s=30.0)
        b.afe.off()
    for index, name in enumerate(('ntc', 'mcu', 'afe')):
        low = min(before[index], after[index]) - THERMOMETER_K[name]
        high = max(before[index], after[index]) + THERMOMETER_K[name]
        report.check('the board\'s %s reads the world\'s, within what it resolves' % name,
                     got.get(name) is not None and low <= got[name] <= high,
                     '%s read, the world %.3f to %.3f' % (got.get(name), before[index],
                                                          after[index]))


def test_the_current_is_the_worlds(report, rig):
    """A held vector over a second: the world's three mean squares are 1.5 |i|^2 of what the
    drive regulates - the phases spanned and zeroed on the world's front end
    (Coaxial63100._in_its_world) - and the observer's legs the world's."""
    b, limb = rig.board, native.limb_for(rig.port)
    step = turn_motor(rig, rig.origin, HELD_A)
    if step is None:
        report.check('the demo drives native://', False, 'it took the board for a real one')
        return
    regulated = []
    try:
        step()
        b.transport.sleep(0.5)
        with limb.lock:
            limb.boards[0].squares()                  # the window opens
        for _ in range(10):
            d = b.drive.state()
            regulated.append(1.5 * (d['id'] ** 2 + d['iq'] ** 2))
            b.transport.sleep(0.1)
        with limb.lock:
            world = sum(limb.boards[0].squares())
            truth = dict(zip(NODES, limb.boards[0].nodes()))
        seen = b.thermal.state()['nodes']
    finally:
        stop_motor(rig)
    mean = sum(regulated) / len(regulated)
    report.check('the world carries the current the drive regulates, its mean squares 1.5 '
                 '|i|^2 within SQUARE_SHARE',
                 abs(world - mean) <= SQUARE_SHARE * mean,
                 '%.1f A^2 in the world, %.1f regulated' % (world, mean))
    apart = max(abs(seen[leg] - truth[leg]) for leg in NODES[:3])
    report.check('and the observer\'s legs are the world\'s, within LEG_K', apart <= LEG_K,
                 'observer %s, world %s' % (
                     ['%.1f' % seen[leg] for leg in NODES[:3]],
                     ['%.1f' % truth[leg] for leg in NODES[:3]]))


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
        if GCOV:
            report.skip('each limb\'s clock keeps the wall\'s, within TRAIL_S',
                        'built for gcov, unoptimised')
        else:
            report.check('each limb\'s clock keeps the wall\'s, within TRAIL_S',
                         all(abs(e) <= native.TRAIL_S for e in off.values()),
                         ' '.join('%s %+.4f' % kv for kv in sorted(off.items())))
        short = [(n.name, g['updates'] - u) for n, g, u in zip(nodes, g1, u0)
                 if (g['updates'] - u) * ts < WINDOW_S - native.TRAIL_S or g['overruns']]
        report.check('every board a triple every period, none overrun', not short, str(short[:4]))
    finally:
        for n in nodes:
            n.close()


def test_the_sto_chain_follows_the_pilot(report, rig):
    """The master's pilot on the bus and the AFE up: the chain releases - PE15 high, Clevel
    and Cinj over the interlock's, +15V7 over the drivers' UVLO - and the break latch clears;
    the pilot gone, PE15 falls and BIF latches; back, the stage arms on the interlock with
    neither bypass, and the pilot gone again drops MOE through the break."""
    b = rig.board
    b.afe.on()
    try:
        b.transport.sleep(STO_SETTLE_S)
        b.gate_drivers.clear()
        up = sto_seen(b)
        rig.pilot(0.0)
        b.transport.sleep(STO_SETTLE_S)
        down = sto_seen(b)
        rig.pilot(PILOT_VOLTS)
        b.transport.sleep(STO_SETTLE_S)
        rig.gates.on()
        armed = b.gate_drivers.state()
        rig.pilot(0.0)
        b.transport.sleep(STO_SETTLE_S)
        broken = b.gate_drivers.state()
    finally:
        rig.pilot(PILOT_VOLTS)
        rig.gates.off()
        b.afe.off()
    report.check('released on the pilot: PE15 high, the latch cleared, the pins over the interlock',
                 up['pe15'] and not up['fault'] and up['Clevel'] >= 2.0 and up['Cinj'] >= 3.0
                 and up['vgate'] >= GATE_UVLO_V,
                 'Cinj %.2f, Clevel %.2f V, +15V7 %.1f V' % (up['Cinj'], up['Clevel'], up['vgate']))
    report.check('the pilot gone: PE15 low, BIF latched, +15V7 under UVLO',
                 not down['pe15'] and down['fault'] and down['vgate'] < GATE_UVLO_V,
                 'Cinj %.2f, Clevel %.2f V, +15V7 %.1f V' % (down['Cinj'], down['Clevel'],
                                                           down['vgate']))
    report.check('armed on the interlock with neither bypass, the break drops MOE',
                 armed['pwm_enabled'] and not armed['break_bypassed']
                 and not broken['pwm_enabled'] and broken['fault'],
                 'MOE %s then %s' % (armed['pwm_enabled'], broken['pwm_enabled']))


def sto_seen(board):
    """PE15, BIF and the chain's three pins: Cinj and Clevel at the pin, +15V7 as the gate
    drivers' state reads it through the record's divider."""
    pins = {c['signal']: c['volts_at_pin'] for c in board.analog.read(samples=4)['channels']}
    state = board.gate_drivers.state()
    return {'pe15': board.analog.scan()['pe15'], 'fault': state['fault'],
            'Cinj': pins['Cinj'], 'Clevel': pins['Clevel'], 'vgate': state['vgate_mv'] / 1e3}


#: The rig's tests in their order; the thermometers last: their sample a thermal second winds
#: the observer's leg patches off the world's (the NTC anchor inverts a standing miss through
#: its lag at every sample, docs/TODO.md).
RIG = (test_it_stands_as_an_emulated_board, test_the_clock_keeps_the_wall,
       test_the_demo_motor_turns_in_real_time, test_the_parts_answer,
       test_the_current_is_the_worlds, test_the_sto_chain_follows_the_pilot,
       test_the_thermometers_read_the_world)

#: The suite's time, s: the rig's tests ran 20 s (2026-09-27); the humanoid's fleet its own.
RIG_S, BODY_S = 120, 300


def main(argv=None):
    names = list(sys.argv[1:] if argv is None else argv)
    body = 'body' in names
    names = [name for name in names if name != 'body']
    report = Report()
    if find_cc() is None:
        print('no C compiler: the board layer cannot be built for this host')
        print('\n0 passed, 0 failed')
        return 0
    tests = pick(RIG, names) if names or not body else []
    watchdog((RIG_S if tests else 0) + (BODY_S if body else 0))
    if tests:
        rig = Coaxial63100(port='native://').open()
        try:
            for test in tests:
                print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
                test(report, rig)
        finally:
            rig.close()
    if body:
        print('\n-- the body keeps the wall --')
        test_the_body_keeps_the_wall(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
