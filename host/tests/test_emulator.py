#!/usr/bin/env python3
"""The firmware's own image on an emulated MCU, its front end fed from the electronics.

The whole of it - CubeMX's code, the HAL, the board layer, comms/, the cores - on Renode's
STM32H753 (tools/emu, board/emu), the front end from the LTspice fit
(board/emu/coaxial_63100_afe.repl).

The bench's conformance suite runs first, alone on the console Renode serves one client at a
time; then a rig on the same port through test_wire's sweeps, and the front end's inputs
through the monitor. Skips without Renode or a built image, unless COAXIAL_EMULATOR is
`required` (CI).
"""
import os
import subprocess
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_wire as wire  # noqa: E402
from coaxial import EMULATED, Coaxial63100  # noqa: E402
from coaxial.devices import boot  # noqa: E402
from coaxial.errors import RigError  # noqa: E402
from tools.emu import protocol_emulator  # noqa: E402
from tools.emu.emulator import (BOOT_ELF, ELF, FAITHFUL_MIPS, Emulator, Limb,  # noqa: E402
                                find_renode)

AFE = 'sysbus.gpioPortB.afe'
ANGLE = 'sysbus.spi4.angle'
IMU = 'sysbus.spi2.imu'


def test_the_bench_conformance_holds(report, emu):
    done = subprocess.run([sys.executable, '-X', 'utf8', wire.CONFORMANCE, '--port', emu.url],
                          capture_output=True, text=True, encoding='utf-8',
                          timeout=wire.CONFORMANCE_S)
    lines = done.stdout.strip().splitlines() or ['no output: ' + done.stderr.strip()[-200:]]
    failed = [line.strip() for line in lines if line.strip().startswith('FAIL')]
    report.check('the bench conformance suite holds on the emulated MCU',
                 done.returncode == 0, '; '.join([lines[-1]] + failed[:3]))


def test_the_front_end_feeds_the_image(report, rig, emu):
    """The DC link fed through the front end, read back through the image's own scan: the
    reading follows what is fed. Recorded, not judged against a number."""
    b = rig.board
    b.afe.on()
    got = []
    for volts in (12.0, 24.0, 48.0):
        emu.command('%s DcBusVolts %s' % (AFE, volts))
        got.append((volts, b.analog.scan()['dcbus_mv']))
    emu.command('%s DcBusVolts 0' % AFE)
    b.afe.off()
    readings = [mv for _, mv in got]
    report.check('the DC link the image reads follows the one fed',
                 readings == sorted(readings) and len(set(readings)) == len(readings),
                 ', '.join('%g V -> %d mV' % pair for pair in got))


#: Where no emulator runs: Renode named at nothing, found nowhere else.
NOWHERE = {'RENODE': os.path.join(os.sep, 'no', 'renode'), 'LOCALAPPDATA': os.path.join(os.sep, 'no'),
           'PATH': ''}

FALLS_BACK = '''from coaxial import EMULATED, Coaxial63100
rig = Coaxial63100(execution_mode=EMULATED).open()
print(rig.simulated, rig.origin.label)
rig.close()'''


def test_the_injected_triple_runs(report, rig, emu):
    """The sync armed: the injected triple counts once a PWM period on TIM1's TRGO2, and rank 2
    on ADC3 follows the DC link as the front end is fed it."""
    b = rig.board
    b.afe.on()
    try:
        b.gate_drivers.configure(sync=True)
        seen = []
        for volts in (12.0, 36.0):
            emu.command('%s DcBusVolts %g' % (AFE, volts))
            b.transport.sleep(0.05)                 # of the board's: 2 500 periods
            seen.append(b.gate_drivers.state())
    finally:
        b.gate_drivers.configure(sync=False)
        emu.command('%s DcBusVolts 0' % AFE)
        b.afe.off()
    first, last = seen
    report.check('the injected triple counts on TRGO2 while the sync is armed',
                 last['sync_armed'] and last['updates'] > first['updates'] > 0,
                 '%d then %d triples' % (first['updates'], last['updates']))
    report.check('its rank 2 follows the DC link fed in',
                 (last['dcbus_raw'] or 0) > (first['dcbus_raw'] or 0),
                 '12 V -> %s, 36 V -> %s' % (first['dcbus_raw'], last['dcbus_raw']))


def test_the_angle_sensor_reads(report, rig, emu):
    """The A1335 on SPI4: the shaft's angle, set, read back through the firmware's poll to its
    twelve bits."""
    b = rig.board
    b.afe.on()
    try:
        got = []
        for degrees in (30.0, 250.0):
            emu.command('%s Degrees %g' % (ANGLE, degrees))
            b.transport.sleep(0.05)
            got.append((degrees, b.angle.state().get('degrees')))
    finally:
        b.afe.off()
    report.check('the angle sensor reads the shaft, to a count',
                 all(read is not None and abs(read - want) <= 360.0 / 4096 for want, read in got),
                 ', '.join('%g -> %s' % pair for pair in got))


def test_the_imu_answers(report, rig, emu):
    """The BNO085 on SPI2: its product id, and the accelerometer reading what it is given."""
    from coaxial.devices.imu import ACCELEROMETER

    imu = rig.board.imu
    rig.board.afe.on()
    try:
        emu.command('%s AccelZ 3.5' % IMU)
        rig.board.transport.sleep(0.2)
        with imu.configuring():
            ident = imu.product_id()
        imu.configure({ACCELEROMETER: 20000})
        rig.board.transport.sleep(0.2)               # ten reports at 20 ms
        accel = (imu.state().get('accelerometer') or {}).get('value') or {}
    finally:
        rig.board.afe.off()
    report.check('the IMU answers its product id', bool(ident.get('sw_version')), str(ident))
    report.check('its accelerometer reads what it is given, to a count',
                 abs(accel.get('z', 0.0) - 3.5) <= 1.0 / 256, str(accel))


#: Echoes a blast sends, of the most a frame carries less the envelope.
BLAST, BLAST_BYTES = 50, 240


def test_ten_megabit_on_the_bus(report):
    """A limb's bus at the bootloader's 10 Mbit/s, the core at the part's own speed: every echo
    comes back byte for byte, and the node counts no framing error past the host's handover
    byte and drops nothing from its ring."""
    with Limb(1, mips=FAITHFUL_MIPS, idle_mips=None, baud=10_000_000, mpu=True) as limb:
        rig = Coaxial63100(port=limb.url, unit=1, own_image=False).open()
        # Its waits on the limb's pace, as emulator:// gives them: the node asleep keeps real
        # time, awake it replied after a scale-1 wait had given up (2026-09-25).
        rig.board.transport.time_scale_source = limb.load
        try:
            link = rig.board.link
            wrong = 0
            for k in range(BLAST):
                data = bytes((k * 31 + n * 7) % 256 for n in range(BLAST_BYTES))
                try:
                    link.echo(data)
                except Exception:          # a lost echo is the finding, counted
                    wrong += 1
            port = link.state(port=1)
        finally:
            rig.close()
    report.check('10 Mbit/s on the bus: every echo back, nothing dropped, framing clean',
                 wrong == 0 and port['ring_dropped'] == 0 and port['bus_comm_error'] <= 1,
                 '%d of %d echoes wrong, %d framing errors, %d dropped'
                 % (wrong, BLAST, port['bus_comm_error'], port['ring_dropped']))


def test_a_blank_node_takes_the_host_build(report):
    """A node blank in its bootloader on a 10 Mbit limb, the core at the part's own speed:
    open() finds nothing at the unit, loads this host's build through the bootloader at 247
    over Modbus, and the application answers naming it - host and target on one build."""
    url = 'emulator://?nodes=1&boot=1&mpu=1&mips=%d' % FAITHFUL_MIPS
    path, image = boot.host_image() or (None, b'')
    want = (len(image), zlib.crc32(image))
    rig = Coaxial63100(port=url, unit=1, execution_mode=EMULATED, fallback=False)
    try:
        rig.open()
        named = rig.board.boot.state()['image']
        runs = rig.board.version_info is not None
        said = 'the application names %s against %s' % (named, want)
    except RigError as exc:
        named, runs, said = None, False, '%s: %s' % (type(exc).__name__, exc)
    finally:
        rig.close()
        protocol_emulator.release(url)
    report.check('a blank node takes this host\'s build over Modbus, and runs it',
                 rig.image_loaded == (path, True) and named == want and runs, said)


def test_emulated_falls_back_where_none_runs(report):
    """EMULATED on a machine with no Renode - CI's host job, a bare checkout - opens the
    stand-in and says why, as HARDWARE does where no board answers."""
    done = subprocess.run([sys.executable, '-X', 'utf8', '-c', FALLS_BACK],
                          env=dict(os.environ, **NOWHERE), capture_output=True, text=True,
                          encoding='utf-8', timeout=120)
    said = done.stdout.strip()
    report.check('EMULATED with no emulator falls back to the stand-in, and says why',
                 said.startswith('True Simulated - no emulator here'),
                 said[:100] or done.stderr.strip()[-200:])


def main():
    report = wire.Report()
    print('\n-- emulated falls back where none runs --')
    test_emulated_falls_back_where_none_runs(report)
    if find_renode() is None or not (os.path.exists(ELF) and os.path.exists(BOOT_ELF)):
        print('no Renode or no images (%s, %s): the emulator needs all three' % (ELF, BOOT_ELF))
        required = os.environ.get('COAXIAL_EMULATOR') == 'required'
        print('\n%d passed, %d failed' % (report.passed, report.failed + required))
        return int(required or report.failed)
    # Renode's own 100 MIPS: nothing here runs to the part's cycle budget.
    with Emulator(monitor=True, mips=None, mpu=True) as emu:
        print('\n-- the bench conformance holds --')
        test_the_bench_conformance_holds(report, emu)
        rig = Coaxial63100(port=emu.url, own_image=False).open()
        rig.board.transport.time_scale_source = emu.load
        try:
            for test in (wire.test_every_read_decodes, wire.test_settings_are_taken,
                         wire.test_the_wire_refuses, wire.test_every_verb_answers_or_refuses,
                         wire.test_the_acquisition_records_decode,
                         wire.test_the_record_survives_a_save):
                print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
                test(report, rig)
            print('\n-- the front end feeds the image --')
            test_the_front_end_feeds_the_image(report, rig, emu)
            for test in (test_the_injected_triple_runs, test_the_angle_sensor_reads,
                         test_the_imu_answers):
                print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
                test(report, rig, emu)
            # Last: its boot.stay resets the board 50 ms on.
            print('\n-- acquisition answers or refuses --')
            wire.test_acquisition_answers_or_refuses(report, rig)
        finally:
            rig.close()
    print('\n-- ten megabit on the bus --')
    test_ten_megabit_on_the_bus(report)
    print('\n-- a blank node takes the host build --')
    test_a_blank_node_takes_the_host_build(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
