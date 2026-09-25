#!/usr/bin/env python3
"""The device clients through the firmware's own wire, on this host.

A rig on `fakeboard://` (tools.cores.fakeboard): what answers is comms/ - the Modbus RTU
slave, the command tables, every handler - built with the host's compiler over a board
that answers neutrally. Every client's reads decode what the firmware encodes and its
settings are taken: a disagreement between the two sides of the wire fails here, not on
the bench. The record is the board's own (board_cal.c over a RAM sector), and the bench's
conformance suite runs over the same fake.
"""
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100  # noqa: E402
from tools.cores import fakeboard  # noqa: E402
from tools.cores.build import find_cc  # noqa: E402

CONFORMANCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_conformance.py')
#: 18 s measured on the laptop (2026-09-25), the harness's settle sleeps.
CONFORMANCE_S = 120


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


def ran(calls):
    """{name: the exception each call raised}, for those that raised."""
    raised = {}
    for name, call in calls:
        try:
            call()
        except Exception as exc:          # every one of them is a finding here
            raised[name] = '%s: %s' % (type(exc).__name__, exc)
    return raised


def test_the_rig_opens_on_the_firmware(report, rig):
    info = rig.board.version_info
    report.check('a rig opens on fakeboard://, the firmware answering its version',
                 rig.origin.label == 'fakeboard://' and info['device'] == 'coaxial_63100'
                 and info['proto_major'] >= 2, '%(firmware)s, protocol %(proto_major)d.'
                 '%(proto_minor)d' % info)


def test_every_read_decodes(report, rig):
    b = rig.board
    calls = [('%s.state' % name, getattr(b, name).state)
             for name in ('afe', 'angle', 'boot', 'calibration', 'ctrl', 'drive', 'gate_drivers',
                          'gpio', 'imu', 'link', 'power')]
    calls += [('system.clock', b.system.clock), ('system.version', b.system.version),
              ('system.channel_map', b.system.channel_map),
              ('analog.channels', b.analog.channels),
              ('gpio.read', lambda: b.gpio.read('B', 7)),
              ('gpio.port_read', lambda: b.gpio.port_read('B')),
              ('drive.params', b.drive.params), ('drive.setpoints', b.drive.setpoints),
              ('calibration.read', b.calibration.read),
              ('gate_drivers.dead_time', b.gate_drivers.dead_time),
              ('thermal.state', b.thermal.state), ('thermal.budget', b.thermal.budget),
              ('thermal.identification', b.thermal.identification)]
    raised = ran(calls)
    report.check('every device\'s reads decode what the firmware sends',
                 not raised, '%d reads%s' % (len(calls), '; ' + str(raised) if raised else ''))


def test_settings_are_taken(report, rig):
    b = rig.board
    calls = [('afe.on', b.afe.on), ('afe.off', b.afe.off), ('power.off', b.power.off),
             ('gate_drivers.off', b.gate_drivers.off),
             ('gate_drivers.reset_worst_gap', b.gate_drivers.reset_worst_gap),
             ('ctrl.wire', lambda: b.ctrl.wire('angle', 'theta', 100)),
             ('ctrl.rows', lambda: b.ctrl.rows([(0.5, 10.0), (0.2, 0.0)])),
             ('ctrl.drop', b.ctrl.drop), ('drive.set_theta', lambda: b.drive.set_theta(0.5)),
             ('calibration.defaults', b.calibration.defaults)]
    raised = ran(calls)
    report.check('settings are taken, the firmware acknowledging each',
                 not raised, '%d settings%s' % (len(calls), '; ' + str(raised) if raised else ''))


def test_the_wire_refuses(report, rig):
    """What the firmware refuses, refused through the wire as the client says it."""
    from coaxial.errors import DeviceStateError, ModbusException
    b = rig.board
    wrong = {}
    for name, call, kind in (
            ('analog.scan with the AFE off (invariant 9)', b.analog.scan, DeviceStateError),
            ('gpio.write with the test rig shut', lambda: b.gpio.write('B', 7, 1),
             ModbusException),
            ('gpio.port_write with the test rig shut', lambda: b.gpio.port_write('B', 1, 1),
             ModbusException)):
        try:
            call()
            wrong[name] = 'taken'
        except kind:
            pass
        except Exception as exc:          # the wrong refusal is as much a finding
            wrong[name] = '%s: %s' % (type(exc).__name__, exc)
    report.check('the AFE off refuses a cooked scan, a shut test rig its writes',
                 not wrong, str(wrong) if wrong else '')


def test_every_verb_answers_or_refuses(report, rig):
    """Invariant 8 across the wire: each verb a result or a raise from coaxial.errors, never
    a decode's KeyError or struct.error."""
    from coaxial.errors import RigError
    b = rig.board
    a, d, t, i, g, p = b.analog, b.drive, b.thermal, b.imu, b.gate_drivers, b.gpio
    calls = [('afe.on', b.afe.on), ('analog.scaling', a.scaling), ('analog.names', a.names),
             ('analog.index_of', lambda: a.index_of('Phase U')),
             ('analog.burst', lambda: a.burst(a.mask_all(), 8)),
             ('analog.read', lambda: a.read(samples=8)),
             ('analog.noise', lambda: a.noise(0, samples=16)),
             ('analog.ntc_temperature', a.ntc_temperature),
             ('analog.dcbus_voltage', a.dcbus_voltage),
             ('analog.phase_current', a.phase_current), ('analog.scan', a.scan),
             ('afe.off', b.afe.off),
             ('drive.model.state', d.model.state),
             ('drive.model.configure', lambda: d.model.configure(r=0.05)),
             ('drive.model.reset', d.model.reset), ('drive.observers.read', d.observers.read),
             ('drive.moments.read', lambda: d.moments.read(count=4, timeout=0.3, poll=0.05)),
             ('drive.configure source', lambda: d.configure(source='model')),
             ('drive.configure profile', lambda: d.configure(profile='outrunner_63100_14p')),
             ('drive.write', lambda: d.write(id_ref=0.0, theta=0.25)), ('drive.read', d.read),
             ('drive.reload', d.reload), ('drive.reset_cycles', d.reset_cycles),
             ('drive.on', lambda: d.on('volt')), ('drive.hold', d.hold), ('drive.off', d.off),
             ('thermal.network', t.network), ('thermal.reset', t.reset),
             ('thermal.configure', lambda: t.configure(sample_every_s=5.0)),
             ('thermal.read', t.read), ('thermal.situation', lambda: t.situation('box')),
             ('thermal.load_cycle', t.load_cycle),
             ('thermal.fast_forward', lambda: t.fast_forward(1.0)), ('thermal.truth', t.truth),
             ('imu.product_id', lambda: i.product_id), ('imu.peek', i.peek),
             ('imu.hold', i.hold), ('imu.resume', i.resume), ('imu.reset', i.reset),
             ('imu.wake_test', lambda: i.wake_test(10)), ('imu.pins', i.pins),
             ('imu.probe', i.probe), ('imu.poke', lambda: i.poke(0, b'\x00')),
             ('gate_drivers.on', g.on), ('gate_drivers.clear', g.clear),
             ('gate_drivers.off', g.off),
             ('gpio.on', p.on), ('gpio.configure', lambda: p.configure('B', 7, 'output')),
             ('gpio.write', lambda: p.write('B', 7, 1)), ('gpio.read', lambda: p.read('B', 7)),
             ('gpio.port_write', lambda: p.port_write('B', 1, 1)), ('gpio.off', p.off),
             ('board.probe', b.probe)]
    refused, wrong = {}, {}
    for name, call in calls:
        try:
            call()
        except RigError as exc:
            refused[name] = type(exc).__name__
        except Exception as exc:          # anything else is the finding
            wrong[name] = '%s: %s' % (type(exc).__name__, exc)
    for name, kind in sorted(refused.items()):
        print('        refused  %-28s %s' % (name, kind))
    report.check('every verb answers or refuses as the library\'s own',
                 not wrong, '%d answered, %d refused%s' % (
                     len(calls) - len(refused) - len(wrong), len(refused),
                     '; ' + str(wrong) if wrong else ''))


def test_the_record_survives_a_save(report, rig):
    """An edit is volatile until saved; a load reads back what was saved."""
    cal = rig.board.calibration
    cal.set_channel(0, 12, 345)
    cal.save()
    cal.set_channel(0, 0, 0)
    cal.load()
    got = cal.read()
    report.check('the record saved, edited and loaded reads back the save',
                 got['stored'] and got['channels'][0] == {'index': 0, 'offset_raw': 12,
                                                          'gain_ppm': 345},
                 str(got['channels'][0]))


def test_the_bench_conformance_holds(report, rig):
    """Every check the bench's conformance suite makes of a board, in a process of its own:
    one fake board a process."""
    done = subprocess.run([sys.executable, '-X', 'utf8', CONFORMANCE, '--port', 'fakeboard://'],
                          capture_output=True, text=True, encoding='utf-8',
                          timeout=CONFORMANCE_S)
    lines = done.stdout.strip().splitlines() or ['no output: ' + done.stderr.strip()[-200:]]
    failed = [line.strip() for line in lines if line.strip().startswith('FAIL')]
    report.check('the bench conformance suite holds over fakeboard://',
                 done.returncode == 0, '; '.join([lines[-1]] + failed[:3]))


def main():
    report = Report()
    if find_cc() is None:
        print('no C compiler: the firmware cannot be built for this host')
        print('\n0 passed, 0 failed')
        return 0
    rig = Coaxial63100(port='fakeboard://', own_image=False).open()
    try:
        for test in (test_the_rig_opens_on_the_firmware, test_every_read_decodes,
                     test_settings_are_taken, test_the_wire_refuses,
                     test_every_verb_answers_or_refuses, test_the_record_survives_a_save,
                     test_the_bench_conformance_holds):
            print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
            test(report, rig)
    finally:
        rig.close()
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    assert fakeboard.SOURCES
    sys.exit(main())
