#!/usr/bin/env python3
"""Her orthokinematic manifest: what a text's numbers move, as Lisp, of her build and her figure.

    python tools/cores/manifest.py --write     # machine/<robot>.manifest.lisp
    page = manifest.text()                     # what the file holds, laid anew

A text asks a mode's target in its unit, its stiffness and damping in N/m and N s/m of the mass
the mode moves (`machine.modal`). Her manifest gives each mode's unit, mass and resting spring,
her body's statics, and each drive's parts, ratio, peak, speed and range - of her build's file
(machine/robots, `machine.mechatronics`) and her figure laid on her loop (`sprung.laid`).
`ortho` compiles it (`defrobot`, `ortho.robot`); test_mirrors holds the file to this.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO
from tools.cores import written

NOTICE = 'Her orthokinematic manifest: written by host/tools/cores/manifest.py, not by hand.'


def path(robot=None):
    """Where her manifest is written."""
    from machine import mechatronics
    return os.path.join(REPO, 'host', 'machine', '%s.manifest.lisp' % (robot or mechatronics.ROBOT))


def laid():
    """{robot, body, modes, drives}: her manifest's numbers, her figure laid on her loop."""
    from machine import drives, latent, mechatronics, modal, ortho, sprung
    from machine.figure import JOINTS
    g = sprung.laid()
    fr, rest = g['fr'], g['rest']
    body = (('mass', g['body'].mass), ('weight', fr[modal.WEIGHT]), ('com_height', fr[modal.H0]),
            ('omega', fr[modal.OMEGA]), ('leg', modal.LEG_M), ('stance_width', 2.0 * fr[modal.W0]),
            ('knee', math.degrees(fr[modal.KNEE])))
    modes = [('mode_' + name, fr[modal.UNIT][i], fr[modal.MASS][i], rest[latent.K][i], rest[latent.D][i])
             for i, name in enumerate(ortho.MODES)]
    build = mechatronics.robot()
    rows = []
    for joint in JOINTS:
        if drives.passive(joint):
            continue
        kind = drives.kind(joint)
        motor, box, inverter = build['stacks'][kind]
        low, high = (math.degrees(v) for v in sprung._Q_LIM.get(kind, (-2.0, 2.0)))
        rows.append((joint, kind, motor, box, inverter, drives.ratio(joint), drives.peak(joint),
                     drives.speed(joint), low, high))
    return {'robot': mechatronics.ROBOT, 'body': body, 'modes': modes, 'drives': rows}


def text(done=None):
    """The manifest's text."""
    done = done or laid()
    lines = [';; ' + NOTICE,
             ';; Units: m, kg, s, N; a mode\'s unit m or rad, its mass kg, its stiffness N/m and damping',
             ';; N s/m of that mass; a drive\'s peak N m at its joint, speed deg/s, the range her law asks deg.',
             '(defrobot %s' % done['robot'],
             '  (body %s)' % ' '.join('(%s %.4g)' % kv for kv in done['body'])]
    lines += ['  (mode %s (unit %.4g) (mass %.4g) (stiffness %.4g) (damping %.4g))' % row for row in done['modes']]
    lines += ['  (drive %s (kind %s) (motor %s) (gearbox %s) (inverter %s) (ratio %.4g) (peak %.4g) '
              '(speed %.4g) (range %.4g %.4g))' % row for row in done['drives']]
    return '\n'.join(lines) + ')\n'


if __name__ == '__main__':
    if '--write' not in sys.argv[1:]:
        sys.exit(__doc__)
    page = text()
    written(path(), page)
    print('%s: %d lines' % (path(), page.count('\n')))
