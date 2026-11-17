"""What a board is mounted in: thermal_app.c's network and air as the C lays them, the
mirror's the same, each application shedding as its reasons say, the stand-in mounted."""
import ctypes
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100
from coaxial.errors import RigError
from coaxial.model import thermal, thermal_app
from machine.rig.modes import SIMULATED
from tools.cores.build import build, find_cc
from tools.cores.thermal import NODES, SOURCES, THERMAL, Model

from test_modbus_core import Report

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
        taken = th.configure(application='enclosure')
        got = th.identification()
        edge = th.network()['edges'][thermal_app.EDGE_MOUNT_FIRST]
        report.check('the stand-in takes an application, its network and identification anew',
                     taken == {'application': True} and got['application'] == 'enclosure'
                     and got['state'] == 'UNCERTAIN' and got['updates'] == 0
                     and _close(edge[2], thermal_app.LEG_MOUNT[2]),
                     '%s, mount %.2f K/W' % (got['application'], edge[2]))
        try:
            th.configure(application='water')
            refused = False
        except RigError:
            refused = True
        report.check('an application it has no table for is refused', refused)
        th.configure(application='airstream')
        th.airspeed(20.0, truth=24.0)
        told, flown = th._air()
        th.fast_forward(thermal_app.AIRSPEED_HOLD_S * th.HASTE + 1.0)
        gone, still = th._air()
        report.check('the told airspeed holds a second of its wall, the truth\'s own beside it',
                     told == 20.0 and flown == 24.0 and gone == 0.0 and still == 24.0,
                     'told %.0f m/s then %.0f, truth %.0f' % (told, gone, flown))
    finally:
        rig.close()


def test_the_heat_dry_loss_scales_with_bus_voltage(report, _lib):
    """Capacitive switching loss Q_sw * f_pwm * V_bus scales with voltage, charge and PWM rate."""
    from machine.body import heat, mechatronics
    inv50 = mechatronics.library()['inverters']['coaxial_63050']
    report.check('dry loss is 2.4 W at 24 V', abs(heat.dry_loss(24.0) - 2.4) < 1e-6)
    report.check('dry loss is 4.4 W at 44 V', abs(heat.dry_loss(44.0) - 4.4) < 1e-6)
    report.check('dry loss at nominal 48 V is 4.8 W', abs(heat.dry_loss(48.0) - 4.8) < 1e-6)
    report.check('SWITCHING_W matches nominal 48 V dry loss', abs(heat.SWITCHING_W - 4.8) < 1e-6)
    report.check('50 A board has half the switching charge at 48 V (2.4 W)',
                 abs(heat.dry_loss(48.0, inv50) - 2.4) < 1e-6)
    h24 = heat.Heat([], volts=24.0)
    report.check('Heat instance with 24 V sets switching_w to 2.4 W', abs(h24.switching_w - 2.4) < 1e-6)


def test_heterogeneous_inverter_and_gearbox_derivations(report, _lib):
    """Per-inverter mounting resistance, laminate heat capacity, housekeeping and per-gearbox mesh."""
    from machine.body import dclink, drives, heat, mechatronics
    m100 = drives.MOUNT_BY_INV['coaxial_63100']
    m50 = drives.MOUNT_BY_INV['coaxial_63050']
    report.check('70 mm 50 A board has higher TIM and shell mounting K/W than 100 mm 100 A board',
                 m100 == (0.6, 2.19) and m50 == (1.22, 4.47), f'{m100} vs {m50}')
    inv50 = mechatronics.library()['inverters']['coaxial_63050']
    lam50 = mechatronics.laminate_j_k(inv50, mechatronics.materials())
    spec50 = drives.heat('left_knee') + drives.stator('left_knee') + (drives.ratio('left_knee'), lam50, 1.0, 0.25)
    h50 = heat.Heat([spec50], volts=48.0)
    report.check('Heat honours per-drive switch, laminate and housekeeping parameters',
                 abs(h50.capacity[0][0] - 0.25) < 1e-9 and abs(h50.capacity[0][1] - lam50) < 1e-9
                 and abs(float(h50._house_w[0]) - 1.0) < 1e-9)
    report.check('DC-link housekeeping + switching sum matches driven joints',
                 abs(dclink.laid()['house_w'] - len(dclink.DRIVEN) * (heat.HOUSEKEEPING_W + heat.SWITCHING_W)) < 1e-9
                 and drives.zeta('left_knee') == drives.BOX_ZETA and drives.zeta('left_foot') == 0.0)
    from machine.aircraft import flightpack
    from machine.body import compile as body_compile, gearbox, mjcf, peripherals, structural
    raw_inv = mechatronics._toml(os.path.join(mechatronics.LIBRARY, 'inverters.toml'))
    raw_w = mechatronics._toml(os.path.join(mechatronics.LIBRARY, 'world.toml'))
    report.check('passive armatures, backlash stops and mesh smoothing generalize per stack',
                 mjcf._passive_j('left_wrist') > 0.0
                 and mjcf._held_deg('left_gripper') == mjcf.HELD_DEG
                 and structural.FLEX_DEG == 0.5 * drives.BACKLASH_DEG
                 and gearbox.MESH_RAD_S > 0.0 and body_compile.ENC_RAD > 0.0
                 and peripherals.identify()['wear']['unit']['kt'] > 0.0)
    report.check('MOSFET die charge/capacitance and flight pack resistance derive from construction',
                 'switch_q_c' not in raw_inv['coaxial_63050']
                 and abs(inv50['switch_q_c'] - 1.0e-6) < 1e-12 and abs(inv50['switch_j_k'] - 0.175) < 1e-9
                 and mechatronics.primary_inverter() == 'coaxial_63100'
                 and not ({'ah', 'ohm'} & set(raw_w['flight_pack']))
                 and abs(flightpack.PACK_AH - 1.0) < 1e-9 and abs(flightpack.PACK_OHM - 0.04) < 1e-9)


ROSTER = (test_the_mirror_lays_what_the_c_lays, test_the_air_is_the_cs,
          test_each_application_sheds_as_its_reasons_say, test_the_stand_in_is_mounted,
          test_the_heat_dry_loss_scales_with_bus_voltage,
          test_heterogeneous_inverter_and_gearbox_derivations)


def main():
    cc = find_cc()
    if cc is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    lib_path, warnings = build(cc, SOURCES, [os.path.join(THERMAL, 'inc')], name='thermalapp')
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
