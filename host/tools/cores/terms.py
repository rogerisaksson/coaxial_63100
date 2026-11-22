"""The robot's movements' orthopaedic terms as static arrays: which joint makes each, and which way.

    python tools/cores/terms.py --write     # machine/body/term_rows.py
    laid = terms.rows()                     # what the file holds, laid anew from the robot's figure

Anatomy names a movement by what it does: flexion of the hip brings the knee forward, abduction
takes it from the midline, dorsiflexion raises the toes toward the shin. ANATOMY says that for
each part - a point or an axis of its segment and the way it goes. Which of the robot's joints makes it,
and with which sign, is found: each joint of the segment is turned in the robot's figure
(`figure.frames`) and the one whose motion is the term's is kept. A movement no driven joint of
the robot's own makes is absent. A limb's terms are found on both limbs and must agree - a limb is an
index, its joints' names data. Ranges, deg: the compiled figure's stops (`drives.TRAVEL`); nothing
stopping it, None. `machine.body.terms` reads them, tests/test_terms.py holds them to MuJoCo.
"""
import importlib
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

if __name__ == '__main__' and '--write' in sys.argv:
    os.environ.pop('COAXIAL_ROBOT', None)

from machine.body import figure
from machine.body.terms import rows as _rows
from tools import REPO
from tools.cores import written

PATH = os.path.join(REPO, 'host', 'machine', 'body', 'term_rows.py')
NOTICE = "The robot's movements' terms as static arrays: written by host/tools/cores/terms.py, not by hand."


def stops():
    """{joint: (low, high) deg} of every joint something stops: the compiled figure's stops."""
    mujoco = importlib.import_module('mujoco')
    from machine.body import mjcf
    m = mujoco.MjModel.from_xml_string(mjcf.mjcf())
    return {j: tuple(round(math.degrees(float(v)), 6) for v in m.joint(j).range)
            for j in figure.JOINTS if m.jnt_limited[m.joint(j).id]}


def rows():
    """What machine/body/term_rows.py holds, {name: value}."""
    return _rows(stops())


def render(laid):
    """The module's text."""
    def number(v):
        return 'None' if v is None else '%g' % v

    lines = ['"""%s"""' % NOTICE, '',
             "#: (part, movement): (joint kind, sign, low, high) - the movement its own way is the",
             "#: joint's angle times sign, deg; low and high its amount's stops, None with none. A",
             "#: movement toward a side (TOWARD) is told for limb 0's: times a limb's mirror.",
             'TERMS = {']
    lines += ['    (%r, %r): (%r, %s, %s, %s),' % (part, name, kind, number(sign), number(low), number(high))
              for (part, name), (kind, sign, low, high) in laid['TERMS'].items()]
    lines += ['}', 'TOWARD = (%s)' % ''.join('(%r, %r), ' % t for t in laid['TOWARD']).rstrip(' '),
              '#: Movements anatomy names that no driven joint of the robot\'s own makes.',
              'ABSENT = (']
    lines += ['    ' + ' '.join('(%r, %r),' % t for t in laid['ABSENT'][k:k + 3])
              for k in range(0, len(laid['ABSENT']), 3)]
    lines += [')', "#: Each limb's joints by kind: limb 0 the one on +x, as `reach_rows.LIMBS`.",
              'LIMBS = (']
    for limb in laid['LIMBS']:
        items = ['%r: %r' % item for item in limb.items()]
        rows_txt = [', '.join(items[k:k + 4]) for k in range(0, len(items), 4)]
        lines += ['    {' + (',\n     '.join(rows_txt)) + '},']
    return '\n'.join(lines + [')']) + '\n'


if __name__ == '__main__':
    if '--write' not in sys.argv[1:]:
        sys.exit(__doc__)
    done = rows()
    written(PATH, render(done))
    print('%d movements written, %d toward a side, %d absent' % (
        len(done['TERMS']), len(done['TOWARD']), len(done['ABSENT'])))
