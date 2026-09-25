#!/usr/bin/env python3
"""The device clients through the firmware's own wire, on this host.

A rig on `fakeboard://` (tools.cores.fakeboard): what answers is comms/ - the Modbus RTU
slave, the command tables, every handler - built with the host's compiler over a board
that answers neutrally. Every client's reads decode what the firmware encodes and its
settings are taken: a disagreement between the two sides of the wire fails here, not on
the bench.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100  # noqa: E402
from tools.cores import fakeboard  # noqa: E402
from tools.cores.build import find_cc  # noqa: E402


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


def main():
    report = Report()
    if find_cc() is None:
        print('no C compiler: the firmware cannot be built for this host')
        print('\n0 passed, 0 failed')
        return 0
    rig = Coaxial63100(port='fakeboard://', own_image=False).open()
    try:
        for test in (test_the_rig_opens_on_the_firmware, test_every_read_decodes,
                     test_settings_are_taken, test_the_wire_refuses):
            print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
            test(report, rig)
    finally:
        rig.close()
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    assert fakeboard.SOURCES
    sys.exit(main())
