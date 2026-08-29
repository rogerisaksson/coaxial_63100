"""Check every parameter in the thermal model against what was measured.

Not a re-run of the calibration - a check that each number traces to a
measurement, and that the model with those numbers lands where the camera
did. A network can have the right resistances and still not converge, and a
parameter can be set without anyone remembering where it came from.

Every line says SOURCE (which measurement it comes from) or ASSUMED. That
distinction is what matters when somebody has to trust an estimate.
"""
import sys

sys.path.insert(0, __file__.rsplit('tools', 1)[0])

from coaxial.thermal import (AMBIENT, CFG, DRIVER_RISE_SWITCHING, NODES,
                             NTC_OFFSET, NTC_SEES_DRIVERS, board_from_ntc,
                             expected_ntc, settled_fraction, tau_minutes)
from coaxial import thermal
from coaxial.thermalmap import LAYOUT, render

#: The camera, 2026-08-28. Dead surface is the reference; ntc is the board's.
CAMERA = {
    'passive': {'dead': 30.0, 'mcu': 45.0, 'regulators': 38.0, 'bridge': 31.0,
                'afe': 31.0, 'hotswap': 31.0, 'ntc': 36.0},
    'afe on': {'dead': 31.1, 'mcu': 45.3, 'regulators': 39.2, 'afe': 37.0,
               'hotswap': 31.0},
    'traffic': {'dead': 31.4, 'mcu': 45.0, 'regulators': 39.0, 'afe': 37.3,
                'hotswap': 31.0},
    'switching': {'dead': 40.0, 'mcu': 57.3, 'regulators': 60.0,
                  'bridge': 50.1, 'afe': 40.0, 'hotswap': 46.0, 'ntc': 55.6},
}

#: Measured, but through a supply whose shunt the owner does not trust.
SUPPLY_IDLE_A = 0.050
LINK_V = 24.0
DISC_AREA_M2 = 2 * 3.14159 * 0.05 ** 2


def rule(title):
    print('\n%s\n%s' % (title, '-' * len(title)))


def check(name, got, want, tol, source):
    ok = abs(got - want) <= tol
    print('  %-26s %8.2f  vs %8.2f  %s  %s'
          % (name, got, want, 'ok ' if ok else 'FAIL', source))
    return ok


def main():
    bad = 0

    rule('1. The board\'s own two numbers')
    theta = (CAMERA['passive']['dead'] - AMBIENT) / (LINK_V * SUPPLY_IDLE_A)
    bad += not check('board_to_ambient K/W', CFG['board_to_ambient'], theta,
                     0.1, 'SOURCE: passive state vs supply 50 mA')
    bad += not check('tau min', tau_minutes(), 6.8, 0.5,
                     'SOURCE: the cooling curve, 11 samples')
    h = 1.0 / (CFG['board_to_ambient'] * DISC_AREA_M2)
    print('    %-24s %8.1f W/m2K  free convection + radiation is 8-12'
          % ('h it implies', h))

    rule('2. Spreading resistance per zone')
    fits = [
        ('mcu', 15.0, 0.666, 'SOURCE: passive +15.0 K / 0.666 W'),
        ('regulators', 8.0, 0.534, 'SOURCE: passive +8.0 K / 0.534 W'),
        ('afe', 5.4, 0.13, 'SOURCE: 2-1, +5.4 K / 0.13 W'),
    ]
    for node, delta, watt, source in fits:
        bad += not check('%s K/W' % node, CFG['to_board'][node], delta / watt,
                         1.5, source)

    # THE CAMERA SAW ONE BRIDGE ZONE, so it constrains the three legs
    # together and not one of them. Three in parallel is what it measured;
    # per leg is three times that, and no measurement says otherwise yet.
    for group, delta, watt, source in (
            (thermal.DRIVERS, 9.1, 0.60, 'SOURCE: 4-1, half the switching'),
            (thermal.PHASES, 9.1, 0.60, 'ASSUMED: same zone as the drivers')):
        parallel = 1.0 / sum(1.0 / CFG['to_board'][n] for n in group)
        bad += not check('%s K/W, three in parallel' % group[0].split('_')[0],
                         parallel, delta / watt, 1.5, source)

    rule('3. The NTC coupling')
    print('  %-26s %8.2f K  SOURCE: passive, no driver was warming'
          % ('offset', NTC_OFFSET))
    print('  %-26s %8.3f    SOURCE: switching, against the node\'s %.1f K'
          % ('sees_drivers', NTC_SEES_DRIVERS, DRIVER_RISE_SWITCHING))
    for tag, rise in (('passive', 0.0), ('switching', DRIVER_RISE_SWITCHING)):
        bad += not check('ntc %s' % tag,
                         expected_ntc(CAMERA[tag]['dead'], rise),
                         CAMERA[tag]['ntc'], 0.2,
                         'the model\'s ntc against the camera\'s')

    rule('4. Compensation back to bulk')
    for tag, rise in (('passive', 0.0), ('switching', DRIVER_RISE_SWITCHING)):
        bad += not check('bulk from ntc, %s' % tag,
                         board_from_ntc(CAMERA[tag]['ntc'], rise),
                         CAMERA[tag]['dead'], 0.2,
                         'the inverse of the line above')

    rule('5. How far a run gets')
    for mins in (5, 10, 25):
        print('  %-26s %7.0f %%   tau = %.1f min'
              % ('%d min' % mins, 100 * settled_fraction(mins), tau_minutes()))

    rule('6. What has no measurement behind it')
    fitted = [f[0] for f in fits if 'SOURCE' in f[3]]
    for node in NODES:
        if node not in fitted:
            print('  %-26s %8s    ASSUMED - no measurement' % (node, '-'))
    for name, why in (
            ('every capacity but board', 'parts settle in seconds, below what'
                                         ' this rig resolves'),
            ('mcu position in LAYOUT', 'nobody has looked at where it sits'),
            ('hotswap K/W', 'it ran unloaded through the whole campaign'),
            ('all LAYOUT coordinates', 'tape measure, not CAD')):
        print('  %-26s %8s    ASSUMED - %s' % (name, '-', why))

    rule('7. The board as a picture, switching state')
    s = CAMERA['switching']
    seen = dict((n, s['bridge']) for n in thermal.DRIVERS + thermal.PHASES)
    seen.update({'regulators': s['regulators'], 'afe': s['afe'],
                 'mcu': s['mcu']})
    print(render(seen, board_c=s['dead']))
    print('  zones drawn: %s' % ', '.join(sorted(LAYOUT)))

    print('\n%s' % ('every checked parameter traces to a measurement'
                    if not bad else '%d parameter(s) do not agree' % bad))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
