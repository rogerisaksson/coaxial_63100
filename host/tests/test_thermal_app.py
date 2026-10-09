"""What a board is mounted in: thermal_app.c's network and air as the C lays them, the
mirror's the same, each application shedding as its reasons say, the stand-in mounted."""
import ctypes
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100  # noqa: E402
from coaxial.errors import RigError  # noqa: E402
from coaxial.model import thermal, thermal_app  # noqa: E402
from machine.modes import SIMULATED  # noqa: E402
from tools.cores.build import build, find_cc  # noqa: E402
from tools.cores.thermal import NODES, SOURCES, THERMAL, Model  # noqa: E402

from test_modbus_core import Report  # noqa: E402

#: A leg's 3 W and its shunts' 1.5, the housekeeping's own and the winding's 20: a load that
#: heats every path an application moves.
LOAD = dict([(n, 3.0) for n in thermal.DRIVERS] + [(n, 1.5) for n in thermal.PHASES]
            + [('mcu', 0.7), ('regulators', 0.75), ('winding', 20.0)])

#: A hover's rotor, rpm, and airspeeds across the board, m/s.
HOVER_RPM, AIRSPEEDS = 1470.0, (0.0, 6.0, 25.0)


def _close(a, b):
    return abs(a - b) <= 1e-5 * max(1.0, abs(a), abs(b))


def _as_cfg(net):
    """The C's network as the mirror's `cfg`."""
    nodes = net['nodes']
    return {'capacity': {n: v[0] for n, v in nodes.items()},
            'to_ambient': {n: v[1] for n, v in nodes.items()},
            'area_share': {n: v[2] for n, v in nodes.items()},
            'rth_die': {n: v[3] for n, v in nodes.items()},
            'forced': {n: v[4] for n, v in nodes.items()},
            'edges': list(net['edges']), 'board_to_ambient': net['board_to_ambient']}


def test_the_mirror_lays_what_the_c_lays(report, lib):
    """Each application over the C's still air, laid by the C and by the mirror."""
    off = []
    for k, app in enumerate(thermal_app.APPLICATIONS):
        m = Model(lib)
        still = _as_cfg(m.network())
        m.application(k)
        c, py = _as_cfg(m.network()), thermal_app.applied(still, app)
        for key in ('capacity', 'to_ambient', 'forced'):
            off += ['%s %s %s: C %.4g, mirror %.4g' % (app, key, n, c[key][n], py[key][n])
                    for n in NODES if not _close(c[key][n], py[key][n])]
        off += ['%s edge %d: C %.4g, mirror %.4g' % (app, e, a, b)
                for e, (a, b) in enumerate(zip(c['edges'], py['edges'])) if not _close(a, b)]
        if not _close(c['board_to_ambient'], py['board_to_ambient']):
            off.append('%s bulk: C %.4g, mirror %.4g' % (app, c['board_to_ambient'],
                                                        py['board_to_ambient']))
    report.check('every application\'s network is the C\'s in the mirror, node, edge and bulk',
                 not off, '; '.join(off[:4]))


def test_the_air_is_the_cs(report, lib):
    """The forced terms' rotor speed: the C's and the mirror's; the airstream's airspeed a
    rotor's wash, the rest deaf to it."""
    off, c_air = [], Model(lib).lib.thm_air_rpm
    for k, app in enumerate(thermal_app.APPLICATIONS):
        for rpm in (0.0, HOVER_RPM, 4000.0):
            for m_s in AIRSPEEDS:
                c, py = c_air(k, rpm, m_s), thermal_app.air_rpm(app, rpm, m_s)
                if not _close(c, py):
                    off.append('%s %.0f rpm %.0f m/s: C %.1f, mirror %.1f' % (app, rpm, m_s, c, py))
    report.check('the air\'s rotor speed is the C\'s in the mirror', not off, '; '.join(off[:4]))
    wash = thermal_app.WASH_M_S_PER_KRPM[1] * HOVER_RPM / 1000.0
    report.check('a hover\'s wash met at rest is a hover\'s rotor; still air hears no airspeed',
                 _close(thermal_app.air_rpm('airstream', 0.0, wash), HOVER_RPM)
                 and thermal_app.air_rpm('still', HOVER_RPM, 25.0) == HOVER_RPM,
                 '%.1f rpm' % thermal_app.air_rpm('airstream', 0.0, wash))


def _steady(app, rpm=0.0):
    return thermal.steady(LOAD, thermal_app.applied(thermal.CFG, app), ambient=25.0,
                          speed_rpm=rpm)


def test_each_application_sheds_as_its_reasons_say(report, lib):
    """Relations the tables' reasons state, at the steady state of one load."""
    t = {app: _steady(app) for app in thermal_app.APPLICATIONS}
    moved = [n for n in thermal.ALL_NODES if abs(t['airstream'][n] - t['still'][n]) > 1e-6]
    report.check('a drone at rest is a bench: no rotor, no wash', not moved, str(moved[:3]))
    hover = (_steady('still', HOVER_RPM), _steady('airstream', HOVER_RPM))
    report.check('in a hover\'s wash the legs and the laminate run cooler than behind a stator',
                 all(hover[1][n] < hover[0][n] for n in ('driver_v', 'patch_v', 'board')),
                 'leg %.1f C against %.1f' % (hover[1]['driver_v'], hover[0]['driver_v']))
    legs = [t[app]['driver_v'] for app in ('still', 'enclosure', 'fan_sink', 'cold_plate')]
    report.check('a housing, a fan\'s sink, a cold plate: each takes the legs lower',
                 all(a > b for a, b in zip(legs, legs[1:])),
                 ', '.join('%.1f' % x for x in legs))
    report.check('immersed the laminate sheds best, PAO better than transformer oil',
                 t['immersion_pao']['board'] < t['immersion_oil']['board']
                 < min(t[a]['board'] for a in ('still', 'enclosure', 'fan_sink')),
                 'PAO %.1f C, oil %.1f' % (t['immersion_pao']['board'],
                                          t['immersion_oil']['board']))


def test_the_stand_in_is_mounted(report, lib):
    """configure(application=) lays the stand-in's network and its truth's, the identification
    starting over; the airspeed held a second, a board's truth its own."""
    rig = Coaxial63100(execution_mode=SIMULATED).open()
    try:
        th = rig.board.thermal
        was = th._ident
        taken = th.configure(application='enclosure')
        got = th.identification()
        edge = th.network()['edges'][thermal_app.EDGE_MOUNT_FIRST]
        report.check('the stand-in takes an application, its network and identification anew',
                     taken == {'application': True} and got['application'] == 'enclosure'
                     and th._ident is not was and _close(edge[2], thermal_app.LEG_MOUNT[2]),
                     '%s, mount %.2f K/W' % (got['application'], edge[2]))
        try:
            th.configure(application='water')
            refused = False
        except RigError:
            refused = True
        report.check('an application it has no table for is refused', refused)
        th.configure(application='airstream')
        th.airspeed(20.0, truth=24.0)
        told, held_s, flown = th._airspeed
        th.fast_forward(thermal_app.AIRSPEED_HOLD_S * th.HASTE + 1.0)
        report.check('the told airspeed holds a second of its wall, the truth\'s own beside it',
                     told == 20.0 and flown == 24.0 and th._model_s > held_s,
                     'told %.0f m/s to %.1f model s, truth %.0f' % (told, held_s, flown))
    finally:
        rig.close()


ROSTER = (test_the_mirror_lays_what_the_c_lays, test_the_air_is_the_cs,
          test_each_application_sheds_as_its_reasons_say, test_the_stand_in_is_mounted)


def main():
    cc = find_cc()
    if cc is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    lib_path, warnings = build(cc, SOURCES, [os.path.join(THERMAL, 'inc')], name='thermalcore')
    lib = ctypes.CDLL(lib_path)
    report = Report()
    report.check('thermal/ builds warning-free with the firmware flags', not warnings,
                 '; '.join(warnings[:3]))
    for test in ROSTER:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, lib)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
