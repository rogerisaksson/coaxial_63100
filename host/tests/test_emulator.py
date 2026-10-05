#!/usr/bin/env python3
"""The firmware's own image on an emulated MCU, its front end fed from the electronics.

The whole of it - CubeMX's code, the HAL, the board layer, comms/, the cores - on Renode's
STM32H753 (tools/emu, board/emu), the front end from the LTspice fit
(board/emu/coaxial_63100_afe.repl).

In groups on the relay, a process and a Renode each (tools.dev.focus): the bench's conformance
suite on a console of its own; a rig through test_wire's sweeps and the front end's inputs
through the monitor; a limb at its record's rate; a blank node loaded at 10 Mbit. Skips without Renode or a built
image, unless COAXIAL_EMULATOR is `required` (CI).

    python -X utf8 tests/test_emulator.py                   # every group
    python -X utf8 tests/test_emulator.py bus blank         # those
    python -X utf8 tests/test_emulator.py board sto imu     # the rig's tests with those words
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
from coaxial.model.inverter import GATE_UVLO_V  # noqa: E402
from coaxial.simulated.sto import PILOT_VOLTS  # noqa: E402
from tools.dev.focus import pick, run_groups, watchdog  # noqa: E402
from tools.dev.suites import EMULATOR_GROUPS  # noqa: E402
from tools.emu import protocol_emulator  # noqa: E402
from tools.emu import world as worlds  # noqa: E402
from tools.emu.emulator import (BOOT_ELF, ELF, FAITHFUL_MIPS, Emulator, Limb,  # noqa: E402
                                find_renode)

AFE = worlds.AFE
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


def test_the_image_proves_itself(report, rig, emu):
    """The board's self test where the image runs: none of it failed, its code's CRC the
    build's own."""
    from machine.rtu import crc16

    checks = {c['name']: c for c in rig.board.system.self_test()}
    failed = sorted(name for name, c in checks.items() if c['status'] == 'fail')
    report.check('the self test answers, none of it failed', bool(checks) and not failed,
                 ', '.join(failed) or '%d checks' % len(checks))
    # A gap between two segments is the store's 0xFF through the bootloader and the loader's 0
    # here: one linker lays the vectors and the header in one segment, CI's in two (2026-10-05).
    length, got = checks['image_len']['value'], checks['image_crc']['value']
    want = [crc16(boot.image_of(ELF, gap)[:length]) for gap in (0xFF, 0x00)]
    report.check("its code's CRC is the build's", got in want,
                 '%d bytes, %04X of %s' % (length, got, ' or '.join('%04X' % c for c in want)))


def test_the_front_end_feeds_the_image(report, rig, emu):
    """The DC link fed through the front end, read back through the image's own scan: the
    reading follows what is fed. Recorded, not judged against a number."""
    b = rig.board
    b.afe.on()
    got = []
    for volts in (12.0, 24.0, 48.0):
        emu.command('%s DcBusVolts %s' % (AFE, volts))
        got.append((volts, b.analog.scan()['dcbus_mv']))
    emu.command('%s DcBusVolts %s' % (AFE, worlds._decimal(worlds.LINK_VOLTS)))
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
        emu.command('%s DcBusVolts %s' % (AFE, worlds._decimal(worlds.LINK_VOLTS)))
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


def test_echoes_on_the_bus(report):
    """A limb's bus at the rate the app's record gives its port, the core at the part's own
    speed: every echo comes back byte for byte, and the node counts no framing error past the
    host's handover byte and drops nothing from its ring. The app refuses a `link_baud` past
    921 600; at 10 Mbit it had answered only because Renode's UARTs ignored the rates, which the
    transceivers now decode by (2026-09-28)."""
    with Limb(1, mips=FAITHFUL_MIPS, idle_mips=None, mpu=True) as limb:
        rig = Coaxial63100(port=limb.url, unit=1, own_image=False).open()
        # Its waits on the limb's pace, as emulator:// gives them: the node asleep keeps real
        # time, awake it replied after a scale-1 wait had given up (2026-09-25).
        rig.board.transport.time_scale_source = limb.load
        try:
            link = rig.board.link
            wrong, lost = 0, []
            opened = link.state(port=1)['bus_comm_error']
            for k in range(BLAST):
                data = bytes((k * 31 + n * 7) % 256 for n in range(BLAST_BYTES))
                try:
                    link.echo(data)
                except Exception as exc:   # a lost echo is the finding, counted and named
                    wrong += 1
                    lost.append('echo %d %s: %s' % (k, type(exc).__name__, str(exc)[:90]))
            port = link.state(port=1)
            scale = rig.board.transport.time_scale
        finally:
            rig.close()
    # Which echo, what the host raised, and the framing errors the open counted beside the
    # echoes': a request split on its way into Renode lost one of 50 with two framing errors on
    # CI's runner, never on this host (2026-09-28).
    report.check('the bus at the record\'s rate: every echo back, nothing dropped, framing clean',
                 wrong == 0 and port['ring_dropped'] == 0 and port['bus_comm_error'] <= 1,
                 '%d of %d echoes wrong, %d framing errors (%d at the open), %d dropped, pace %.0f%s'
                 % (wrong, BLAST, port['bus_comm_error'], opened, port['ring_dropped'], scale,
                    ''.join('; ' + row for row in lost[:2])))


def test_a_bus_off_the_nodes_rate_carries_nothing(report):
    """The host's adapter at 10 Mbit, the app's port at its record's 115 200: nothing crosses, as
    on the part; Renode's UARTs had passed every byte whatever the rates (2026-09-28)."""
    with Limb(1, mips=FAITHFUL_MIPS, idle_mips=None, baud=10_000_000, mpu=True) as limb:
        try:
            Coaxial63100(port=limb.url, unit=1, own_image=False).open().close()
            said = 'answered'
        except RigError as exc:
            said = type(exc).__name__
        with open(limb.log or os.devnull, encoding='utf-8', errors='replace') as f:
            garbles = [line.strip() for line in f if 'garbles' in line]
    report.check('a bus off the node\'s rate carries nothing: the open fails, the node garbles',
                 said != 'answered' and bool(garbles),
                 '%s; %s' % (said, garbles[0][-60:] if garbles else 'no garble logged'))


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


#: The STO chain: its time to release or trip from any input and settle, board s (1.5 ms at
#: the circuit's, world_sto.c).
STO_SETTLE_S = 0.05


def test_the_sto_chain_follows_the_pilot(report, rig, emu):
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
        emu.pilot(0.0)
        b.transport.sleep(STO_SETTLE_S)
        down = sto_seen(b)
        emu.pilot(PILOT_VOLTS)
        b.transport.sleep(STO_SETTLE_S)
        rig.gates.on()
        armed = b.gate_drivers.state()
        emu.pilot(0.0)
        b.transport.sleep(STO_SETTLE_S)
        broken = b.gate_drivers.state()
    finally:
        emu.pilot(PILOT_VOLTS)
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


#: The rig's tests in their order: test_wire's sweeps take the rig, this file's the emulator
#: too; the acquisition's last - its boot.stay resets the board 50 ms on.
BOARD = (wire.test_every_read_decodes, wire.test_settings_are_taken, wire.test_the_wire_refuses,
         wire.test_every_verb_answers_or_refuses, wire.test_the_acquisition_records_decode,
         wire.test_the_record_survives_a_save, test_the_image_proves_itself,
         test_the_front_end_feeds_the_image,
         test_the_injected_triple_runs, test_the_angle_sensor_reads, test_the_imu_answers,
         test_the_sto_chain_follows_the_pilot, wire.test_acquisition_answers_or_refuses)


def run(report, tests):
    """`tests` in order, their names first."""
    for test, *given in tests:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, *given)


def board(report, names=()):
    """A rig on an emulator of its own through BOARD, or those of it `names` picks. Renode's own
    100 MIPS: nothing here runs to the part's cycle budget."""
    with Emulator(monitor=True, mips=None, mpu=True) as emu:
        rig = Coaxial63100(port=emu.url, own_image=False).open()
        rig.board.transport.time_scale_source = emu.load
        try:
            run(report, [(test, rig) + ((emu,) if test.__module__ == __name__ else ())
                         for test in pick(BOARD, names)])
        finally:
            rig.close()


def conformance(report, _names=()):
    with Emulator(monitor=True, mips=None, mpu=True) as emu:
        run(report, [(test_the_bench_conformance_holds, emu)])


def fallback(report, _names=()):
    run(report, [(test_emulated_falls_back_where_none_runs,)])


def bus(report, _names=()):
    run(report, [(test_echoes_on_the_bus,), (test_a_bus_off_the_nodes_rate_carries_nothing,)])


def blank(report, _names=()):
    run(report, [(test_a_blank_node_takes_the_host_build,)])


#: Each a process and a Renode of its own on the relay, the longest first (2026-09-27: the rig
#: 84 s, the blank node 71, conformance 44, the bus 43).
GROUPS = {'board': board, 'blank': blank, 'conformance': conformance, 'bus': bus,
          'fallback': fallback}



def main(argv=None):
    names = list(sys.argv[1:] if argv is None else argv)
    groups, tests = (['board'], names[1:]) if names[:1] == ['board'] else (names or list(GROUPS), [])
    unknown = [g for g in groups if g not in GROUPS]
    if unknown:
        sys.exit('no group %s: %s' % (', '.join(unknown), ', '.join(GROUPS)))
    if find_renode() is None or not (os.path.exists(ELF) and os.path.exists(BOOT_ELF)):
        report = wire.Report()
        GROUPS['fallback'](report)
        print('no Renode or no images (%s, %s): the emulator needs all three' % (ELF, BOOT_ELF))
        required = os.environ.get('COAXIAL_EMULATOR') == 'required'
        print('\n%d passed, %d failed' % (report.passed, report.failed + required))
        return int(required or report.failed)
    if len(groups) > 1:
        return run_groups(__file__, groups, EMULATOR_GROUPS)
    watchdog(EMULATOR_GROUPS[groups[0]])
    report = wire.Report()
    GROUPS[groups[0]](report, tests)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
