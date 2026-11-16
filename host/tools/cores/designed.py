#!/usr/bin/env python3
"""Every loop as the static solver lays it, beside the table its machine runs on today.

    python tools/cores/designed.py           # a row a loop: the robot's joints, each craft's axes
    python tools/cores/designed.py --json    # ... as JSON

The robot's servo table is laid by formulas a joint (`machine.body.compile.servos`) and the crafts'
attitude gains by shares (their impedance blocks); the solver lays each loop of what its drive turns
and the control design's targets (`machine.body.looped`, `machine.aircraft.attitude`). A row: the
machine, the loop, today's stiffness and damping, the solved ones, the need that set the stiffness,
the least ceiling and whether the need lies under it - the robot's in N m/rad and N m s/rad, a craft's
an inertia's (1/s^2, 1/s). Nothing runs on the solved column: the comparison a switch waits on.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: A row's columns, in order.
COLUMNS = ('machine', 'loop', 'kp', 'kd', 'solved_kp', 'solved_kd', 'need', 'ceiling', 'feasible')


def rows():
    """[(machine, loop, today's kp, today's kd, solved kp, solved kd, need, ceiling, feasible)]: the
    robot's driven joints by their kind, then every craft's axes."""
    from machine.aircraft import attitude
    from machine.body import compile, looped, mechatronics
    today, gains = compile.servos(), looped.gains()
    out = [(mechatronics.ROBOT, kind, today[kind][1], today[kind][2], gains[kind][0], gains[kind][1],
            loop.need, loop.ceiling, loop.feasible)
           for kind, (_rotor, _limb, _most, loop, _needs, _ceilings) in looped.table().items()]
    out += [(machine, name, kp, kd, loop.kp, loop.kd, loop.need, loop.ceiling, loop.feasible)
            for machine, name, kp, kd, loop, _needs, _ceilings in attitude.rows()]
    return out


def main(argv=None):
    """The rows said, a line a loop, or as JSON."""
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--json', action='store_true', help='the rows as JSON, a list of objects')
    args = parser.parse_args(argv)
    laid = rows()
    if args.json:
        print(json.dumps([dict(zip(COLUMNS, row)) for row in laid], indent=1))
        return 0
    print('%-36s %-11s %8s %7s  %8s %7s  %-8s %-10s %s' % (
        'machine', 'loop', 'kp', 'kd', 'solved', 'kd', 'need', 'ceiling', 'feasible'))
    for machine, loop, kp, kd, solved_kp, solved_kd, need, ceiling, feasible in laid:
        print('%-36s %-11s %8.1f %7.1f  %8.1f %7.1f  %-8s %-10s %s' % (
            machine, loop, kp, kd, solved_kp, solved_kd, need, ceiling, 'yes' if feasible else 'NO'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
