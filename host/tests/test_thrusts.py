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


ROSTER = (test_the_thrusts_are_shared_as_her_torques,)


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
