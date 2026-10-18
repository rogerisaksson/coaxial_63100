#!/usr/bin/env python3
"""A drive identified on its native board (coaxial.control.identify): the leg's hip held and swung
by the board's own loop, its DAQ's currents and A1335 the rows; J, b and the gravity on its shaft
against the world's own."""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen, watchdog
from gynoid_kit import Report

#: The limb's world and the board on it; the campaign, s; each parameter within this of the
#: world's: over seven campaigns the hip read J 1.87 to 2.39e-4 for 2.13e-4, gravity 0.339 to
#: 0.355 for 0.327 N m, b 1.35 to 1.42e-3 for 1.19e-3 (identification.md).
PORT, UNIT, SECONDS = 'native://?nodes=1&world=humanoid_leg', 1, 16.0
WITHIN = {'J': 0.15, 'gravity': 0.15, 'b': 0.30}
#: The suite's time, s: the board's library built, 11 s to open, the campaign.
SUITE_S = 180


def test_the_hips_drive_is_read_off_its_board(report):
    """The hip's board commissioned as its motor (the profile's drive part), its campaign on the
    world's leg: J, b and the gravity within WITHIN of the world's, the torque left over under a
    third of it."""
    from coaxial import EMULATED, Coaxial63100
    from coaxial.control import identify
    from coaxial.devices.drive import PROFILES
    from tools.cores import native
    rig = Coaxial63100(port=PORT, unit=UNIT, execution_mode=EMULATED).open()
    try:
        node = native.limb_for(PORT).world.spec['nodes'][UNIT - 1]
        with open(os.path.join(PROFILES, node['motor'] + '.json'), encoding='utf-8') as f:
            profile = json.load(f)
        rig.drive.configure(**profile['drive'])
        load, model = node['load'], profile['model']
        got = identify.fitted(identify.campaign(rig, SECONDS), load['gear'])
    finally:
        rig.close()
    world = {'J': model['j'] + load['inertia'] / load['gear'] ** 2, 'b': model['b'] + load['damping'] / load['gear'] ** 2,
             'gravity': load['mass'] * 9.81 * load['arm'] / load['gear']}
    for name, tol in WITHIN.items():
        report.check('%s %.3e for the world\'s %.3e, within %.0f %%' % (name, got[name], world[name], 100 * tol),
                     abs(got[name] - world[name]) <= tol * abs(world[name]))
    report.check('the torque left over %.4f of %.4f N m rms, under a third' % (got['residual'], got['rms']),
                 got['residual'] < got['rms'] / 3.0, 'f %.2e offset %.2e, %d rows by gaps' % (got['f'], got['offset'], got['gaps']))


ROSTER = (test_the_hips_drive_is_read_off_its_board,)


def main(argv=None):
    from tools.cores.build import find_cc
    report = Report()
    if find_cc() is None:
        print('no C compiler: the board layer cannot be built for this host')
        print('\n0 passed, 0 failed')
        return 0
    watchdog(SUITE_S)
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
