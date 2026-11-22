"""The robot's leg's reach over its foot as static arrays: where its stops leave the hip, by height.

    python tools/cores/reach.py --write     # machine/body/reach_rows.py
    laid = reach.rows()                     # what the file holds, laid anew from the robot's figure

A hip's place in its foot's frame - across to the limb's own side, up, along the toes - is the
ankle's roll, the ankle's pitch and the knee, whatever the pelvis's turn: the hip's own three
joints take that, its yaw alone stopped. The roll tilts the leg's plane about the foot's length;
in that plane the hip stands `u` m up and `z` m along. Laid: for each `u` the run of `z` the
ankle's pitch's travel and the knee's (`drives.TRAVEL`) leave, BACK to FORE - free, the leg's
length alone bounding it, at -+(THIGH + SHANK); the roll's travel as tangents; the hip's yaw's
stops; the sole. `machine.body.reach` reads them,
tests/test_reach.py holds them to MuJoCo.

A run is the longest along its `u`: ahead of an island the ankle's stroke forbids (the hip
0.50-0.63 m up) the strip left under the leg's length is not in it. A row is the least of itself
and its neighbours: what is read between rows is reached.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

if __name__ == '__main__' and '--write' in sys.argv:
    os.environ.pop('COAXIAL_ROBOT', None)

from machine.body.reach import rows
from tools import REPO
from tools.cores import written

PATH = os.path.join(REPO, 'host', 'machine', 'body', 'reach_rows.py')
NOTICE = "A leg's reach over its foot as static arrays: written by host/tools/cores/reach.py, not by hand."


def render(laid):
    """The module's text."""
    def flat(xs):
        return '(' + ', '.join('%g' % x for x in xs) + ')'

    def pairs(ps):
        return '(' + ', '.join('(%g, %g)' % p for p in ps) + ')'

    def wrapped(name, xs):
        out, line = [], '%s = (' % name
        for x in xs:
            if len(line) + len('%g, ' % x) > 99:
                out.append(line.rstrip())
                line = '    '
            line += '%g, ' % x
        return out + [line.rstrip(', ') + ')']

    return '\n'.join([
        '"""%s"""' % NOTICE, '',
        "#: Each limb's mirror, the robot's left's and its right's: across is to a limb's own side.",
        'LIMBS = (%s)' % ', '.join('%.1f' % v for v in laid['LIMBS']),
        '#: Thigh and shank, m.',
        'THIGH, SHANK = %g, %g' % (laid['THIGH'], laid['SHANK']),
        "#: The hip's yaw's stops, rad: the foot turned in and out of the pelvis's heading.",
        'YAW = %s' % flat(laid['YAW']),
        "#: The ankle's roll's stroke as tangents: the hip across over its height, to the other",
        '#: side and to its own.',
        'ACROSS = %s' % flat(laid['ACROSS']),
        "#: In the leg's plane, the hip U0 + k DU m up: BACK[k] to FORE[k] m along the toes its",
        "#: ankle's pitch's stroke and its knee's stop leave; -+(THIGH + SHANK), its length alone.",
        'U0, DU = %g, %g' % (laid['U0'], laid['DU']),
    ] + wrapped('BACK', laid['BACK']) + wrapped('FORE', laid['FORE']) + [
        "#: A sole heel to ball, a foot heel to its toes' tips: corners (across, along) m about",
        '#: the ankle, counter-clockwise.',
        'SOLE = %s' % pairs(laid['SOLE']),
        'FOOT = %s' % pairs(laid['FOOT']),
        '',
    ])


if __name__ == '__main__':
    if '--write' not in sys.argv[1:]:
        sys.exit(__doc__)
    done = rows()
    written(PATH, render(done))
    print('%d rows written, U0=%g..%g m' % (
        len(done['FORE']), done['U0'], done['U0'] + done['DU'] * (len(done['FORE']) - 1)))
