#!/usr/bin/env python3
"""The quad's thrusts shared over its rotors (`machine.thrusts`): by hand and as her law."""
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report


def test_the_thrusts_are_shared_as_her_torques(report):
    """`thrusts.stacked`: the thrusts as her whole-body law's levels - the hand's to the
    mN where nothing binds; a rotor at its cap, the tilt kept and the collective giving way."""
    from machine import quad, thrusts
    top, lift = 4.0 * quad.K_THRUST * 700.0 ** 2, quad.MASS_KG * quad.GRAVITY
    torque = (0.2, 0.05, -0.1)
    hand, stack = thrusts.shared(top, lift, torque), thrusts.stacked(top, lift, torque)
    report.check('nothing binding, the stack shares as the hand does',
                 max(abs(a - b) for a, b in zip(hand, stack)) < 1e-3, '%s' % stack)
    lift, torque = 0.95 * top, (0.5 * top * quad.ARM_M, 0.0, 0.0)
    stack = thrusts.stacked(top, lift, torque)
    got = (-sum(f * z for f, (_x, z) in zip(stack, quad.ROTOR_AT)), sum(stack))
    report.check('a rotor at its cap: the tilt kept, the collective the less',
                 abs(got[0] - torque[0]) < 1e-3 * top * quad.ARM_M and got[1] < lift
                 and max(stack) <= top / 4.0 + 1e-9,
                 'tilt %.2f of %.2f N m, collective %.0f of %.0f N' % (got + (torque[0], lift)))


def test_a_rotor_capped_takes_the_four_down(report):
    """A rotor with less in it than its share of the top (`caps`): by hand the tilt's torque
    kept and the collective what that rotor leaves; as the stack the tilt and the collective
    both, the heading's torque what they cost; none capped, as before."""
    from machine import quad, thrusts
    top, lift = 4.0 * quad.K_THRUST * 700.0 ** 2, quad.MASS_KG * quad.GRAVITY
    torque, caps = (0.3, 0.0, -0.2), [top / 4.0, 0.5 * lift / 4.0, top / 4.0, top / 4.0]
    asked, got = 2.0 * lift, {}
    for name, share in (('hand', thrusts.shared), ('stack', thrusts.stacked)):
        f = got[name] = share(top, asked, torque, 0.0, caps)
        tilt = (-sum(x * z for x, (_x, z) in zip(f, quad.ROTOR_AT)),
                sum(x * a for x, (a, _z) in zip(f, quad.ROTOR_AT)))
        report.check('by the %s: a rotor at half a hover\'s share, the tilt\'s torque kept'
                     % name, max(abs(tilt[0] - torque[0]), abs(tilt[1] - torque[2])) < 0.01
                     and f[1] <= caps[1] + 1e-6,
                     'tilt %+.3f %+.3f N m, %s N' % (tilt + (' '.join('%.1f' % x for x in f),)))
    report.check('the hand\'s collective gives way to it, the stack\'s is kept',
                 sum(got['hand']) < lift and abs(sum(got['stack']) - asked) < 0.01 * asked,
                 '%.1f and %.1f N of %.1f asked' % (sum(got['hand']), sum(got['stack']), asked))
    free = [top / 4.0] * 4
    report.check('no rotor capped under its share: as with none',
                 thrusts.shared(top, lift, torque, 0.0, free) == thrusts.shared(top, lift, torque))


ROSTER = (test_the_thrusts_are_shared_as_her_torques, test_a_rotor_capped_takes_the_four_down)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
