#!/usr/bin/env python3
"""The gynoid as the eye sees her, measured a stage a row: a HUMANOID recording.

A recording (R on the page, build/recordings/*.csv) - the rows the page hears from her loop
(`humanoid_keys.row`); her walk simulated is `tools/sim/ortho.py --look`'s, the same rows.

    python tools/sim/look.py --last                 # the newest recording
    python tools/sim/look.py --csv build/recordings/humanoid_20260928_070724.csv
    python tools/sim/look.py --last --brief         # the stages in a line, the walk's measures
    python tools/sim/look.py --last --fbx build/walk.fbx   # her steady walk written a take (mocap.py)

A row a stage (the walk's first second apart): the pelvis and the head under the stand (the
dip), the torso ahead of plumb, the torso against the left shin (under 0 it leans back over bent
knees), the hips-to-shoulders line, the left knee (under 0 bent back), the head's pitch (its
range the nod), the left hip's roll, the thighs' gap as drawn, bare and in the jeans (under 0
they meet), how far the pelvis's and the head's forward point above level (90 on her back), the
seat's drives past her skin, what her drives draw - least and most over the stage.
"""
import argparse
import csv
import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO
from tools.sim.strides import _faces, _gap, _lean, _mid, _out, _p, _roll, walked

#: The head's rest pitch in the neck's offset (`figure.SEGMENTS`: 0.09 up, 0.012 on), deg.
HEAD_REST = math.degrees(math.atan2(0.012, 0.09))

#: The rows' rate as the page hears her, Hz; the walk's first seconds apart.
RATE_HZ, FIRST_S = 60.0, 1.0

#: The drives of a leg, by kind.
LEG_KINDS = ('hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll')


def _held(world):
    """What her spine's drive and her hips' pitch drives give, N m."""
    from machine.figure import JOINTS
    tau = world.data.ctrl
    return {'spine_nm': float(tau[JOINTS.index('spine')]),
            'hips_nm': float(tau[JOINTS.index('left_hip')] + tau[JOINTS.index('right_hip')])}


def _ahead(world):
    """Her upper body's centre of mass ahead of her hips' axis, m, along her way (z)."""
    m, d = world.model, world.data
    torso, pelvis = m.body('torso').id, m.body('pelvis').id
    hips = (d.xpos[m.body('left_thigh').id][2] + d.xpos[m.body('right_thigh').id][2]) / 2.0
    up, kg = m.body_subtreemass[torso], m.body_mass[pelvis]
    z = (up * d.subtree_com[torso][2] + kg * d.xipos[pelvis][2]) / (up + kg)
    return float(z - hips)


def recorded(path):
    """The rows of a recording."""
    with open(path, encoding='utf-8') as f:
        return list(csv.DictReader(f))


#: (name, unit, of a row and the stand's heights {pelvis, head, hip L, hip R}): each column.
MEASURES = (
    ('pelvis dy', 'mm', lambda r, ref: (float(r['y']) - ref['pelvis']) * 1e3),
    ('head dy', 'mm', lambda r, ref: (_p(r, 'head')[1] - ref['head']) * 1e3),
    ('hip L dy', 'mm', lambda r, ref: (_p(r, 'left_thigh')[1] - ref['hip L']) * 1e3),
    ('hip R dy', 'mm', lambda r, ref: (_p(r, 'right_thigh')[1] - ref['hip R']) * 1e3),
    ('pelvis roll', 'deg', lambda r, ref: _roll(r)),
    ('torso', 'deg', lambda r, ref: _lean(_p(r, 'torso'), _p(r, 'neck'))),
    ('torso-shin', 'deg', lambda r, ref: _lean(_p(r, 'torso'), _p(r, 'neck'))
     - _lean(_p(r, 'left_foot'), _p(r, 'left_shank'))),
    ('hips>shoulders', 'deg', lambda r, ref: _lean(_mid(_p(r, 'left_thigh'), _p(r, 'right_thigh')),
                                                   _mid(_p(r, 'left_upper_arm'),
                                                        _p(r, 'right_upper_arm')))),
    ('bow', 'deg', lambda r, ref: _lean(_p(r, 'torso'), _p(r, 'neck'))
     - _lean(_p(r, 'left_foot'), _p(r, 'left_thigh'))),
    ('knee L', 'deg', lambda r, ref: float(r['left_knee'])),
    ('head pitch', 'deg', lambda r, ref: _lean(_p(r, 'neck'), _p(r, 'head')) - HEAD_REST),
    ('hip roll L', 'deg', lambda r, ref: float(r['left_hip_roll'])),
    ('thigh gap', 'mm', lambda r, ref: _gap(r, False)),
    ('jeans gap', 'mm', lambda r, ref: _gap(r, True)),
    ('pelvis faces', 'deg', lambda r, ref: _faces(r, 'pelvis')),
    ('head faces', 'deg', lambda r, ref: _faces(r, 'head')),
    ('hands out', 'mm', lambda r, ref: 1e3 * max(
        math.dist(_p(r, side + '_fingers'), _mid(_p(r, 'torso'), _p(r, 'neck')))
        for side in ('left', 'right'))),
    ('seat out', 'mm', lambda r, ref: _out(r, SEAT)),
    ('drawing', 'W', lambda r, ref: float(r.get('watts') or 'nan')),
)

#: The seat's drives, `seat out` their worst reach past her skin.
SEAT = tuple(side + k for side in ('left_', 'right_') for k in ('hip', 'hip_roll', 'hip_yaw'))

#: A seam's measures, by name, the knee as set beside them; the times read about it, s.
SEAM = ('pelvis dy', 'head dy', 'pelvis roll', 'torso', 'bow', 'knee L')
SEAM_AT = (-0.3, -0.1, 0.0, 0.1, 0.3)


def staged(rows):
    """[(stage, rows)] in order, the walk's first FIRST_S apart; the stand's heights - her
    first rows', where none is called a stand."""
    stands = [r for r in rows if r['stage'] in ('stand', 'rest')] or rows[:1]
    mid = stands[len(stands) // 2]
    ref = {'pelvis': float(mid['y']), 'head': _p(mid, 'head')[1],
           'hip L': _p(mid, 'left_thigh')[1], 'hip R': _p(mid, 'right_thigh')[1]}
    walk_at, groups = None, []
    for r in rows:
        stage = r['stage']
        if r['stage'] == 'walk':
            walk_at = float(r['t']) if walk_at is None else walk_at
            stage += ', 1st s' if float(r['t']) - walk_at < FIRST_S else ''
        if not groups or groups[-1][0] != stage:
            groups.append((stage, []))
        groups[-1][1].append(r)
    return groups, ref


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--csv', help='a HUMANOID recording (R) to measure')
    parser.add_argument('--last', action='store_true', help='the newest recording')
    parser.add_argument('--brief', action='store_true',
                        help="the stages in a line and the walk's measures: no tables")
    parser.add_argument('--fbx', help='her steady walk written a take here (`fbx.wrote`)')
    args = parser.parse_args(argv)
    found = glob.glob(os.path.join(REPO, 'build', 'recordings', '*.csv')) if args.last else []
    path = args.csv or (max(found, key=os.path.getmtime) if found else None)
    if path is None:
        print('no recording: R on the HUMANOID page records one, --last or --csv measures it; '
              'her walk simulated is tools/sim/ortho.py --look')
        return 1
    rows = recorded(path)
    print(path)
    groups, ref = staged(rows)
    if args.brief:
        print(' '.join('%s@%.2f' % (stage, float(mine[0]['t'])) for stage, mine in groups))
        walked(rows, args.fbx)
        return 0
    print('%-14s %6s | %s' % ('moment', 'from s', ' | '.join(
        '%-15s' % ('%s %s' % (name, unit)) for name, unit, _f in MEASURES)))
    for stage, mine in groups:
        cells = []
        for _name, _unit, f in MEASURES:
            v = [f(r, ref) for r in mine]
            cells.append('%+6.1f..%+6.1f' % (min(v), max(v)))
        print('%-14s %6.2f | %s' % (stage, float(mine[0]['t']), ' | '.join(cells)))
    seams(rows, groups, ref)
    walked(rows, args.fbx)
    return 0


#: A foot and its toes, each side's: how near the feet come (`World.gap`).
LEFT_FOOT, RIGHT_FOOT = ('left_foot', 'left_toes'), ('right_foot', 'right_toes')

def seams(rows, groups, ref):
    """Each hand-off between stages: the SEAM measures SEAM_AT seconds about it, the left knee
    as it was set beside the knee as it is."""
    f = {name: g for name, _u, g in MEASURES}
    ts = [float(r['t']) for r in rows]
    for (before, _mine), (after, theirs) in zip(groups, groups[1:]):
        at = float(theirs[0]['t'])
        print()
        print('%s -> %s at %.2f s' % (before, after, at))
        print('   dt s | %s | knee L asked' % ' | '.join('%14s' % n for n in SEAM))
        for dt in SEAM_AT:
            r = rows[min(range(len(ts)), key=lambda i: abs(ts[i] - at - dt))]
            asked = r.get('set_left_knee', '')
            print('  %+5.2f | %s | %s' % (dt, ' | '.join('%+14.1f' % f[n](r, ref) for n in SEAM),
                                          '%12.1f' % float(asked) if asked else '           -'))


if __name__ == '__main__':
    sys.exit(main())
