#!/usr/bin/env python3
"""A human's sit as the reference: a sit's measures from joints' places, a take's or hers.

A take, a recording and her rows read alike, as `normal` reads a walk.

    python tools/sim/seats.py TAKE.fbx [..]                   # the takes' measures, SEAT their band
    python tools/sim/seats.py --recording REC.csv --from 22 --to 34   # hers on the page, the window s
    python tools/sim/seats.py --sim chair_behind sit_chair    # hers here: a scene and the verb
    measures = seats.measured(times, joints)                  # {name: value}

The takes (stunts/sitting, stunts/lying): sitting down from standing takes a human 0.6 s of
descent for 0.40-0.46 m, the hips ending 0.28-0.42 m behind the ankles with the knees at
94-98 deg; the trunk leans 45 deg forward on the way down onto a chair and 14 deg back onto a
bed's edge, and sits 3-7 deg off upright; both feet stay on the floor, rising nowhere; the
upper arms stay within 40 deg of where they hang through the sit and within 10 deg seated.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np  # behind the OpenBLAS cap
from tools.sim.stunts import angle

#: The reference, two takes sitting down (Stand To Sit, Sitting on Bed): (measure, unit, least,
#: most). A length in legs (hip to knee to ankle) reads a walker's size out of it.
SEAT = (
    ('drop', 'legs', 0.45, 0.52),
    ('descent', 's', 0.5, 0.8),
    ('hips back', 'legs', 0.31, 0.47),
    ('trunk peak', 'deg', -14.0, 45.0),
    ('trunk seated', 'deg', -7.0, 7.0),
    ('knee seated', 'deg', 94.0, 98.0),
    ('shank seated', 'deg', -10.5, 1.0),
    ('feet rise', 'legs', 0.0, 0.02),
    ('arm range', 'deg', 0.0, 40.0),
    ('arm seated', 'deg', 0.0, 10.0),
)
#: Of a drop, where the descent is read to begin and end.
EDGE = 0.1
#: Seated: the last SEATED_S of a sit.
SEATED_S = 0.5


def measured(times, joints):
    """{measure: value} of a sit from its joints' places (`fbx.take`'s, `fbx.joints`'s): the
    hips' drop and the seconds it takes, how far behind the ankles they end, the trunk's pitch
    at its peak and seated, the knees seated, the shanks off plumb seated (the knee ahead +), the feet's rise, the upper arms' swing."""
    hips, neck = joints['Hips'], joints['Neck']
    feet = np.stack([joints['LeftFoot'], joints['RightFoot']])
    toes = 0.5 * (joints['LeftToeBase'] + joints['RightToeBase'])
    leg = float(np.mean([np.linalg.norm(joints[s + 'UpLeg'] - joints[s + 'Leg'], axis=1).mean()
                         + np.linalg.norm(joints[s + 'Leg'] - joints[s + 'Foot'], axis=1).mean()
                         for s in ('Left', 'Right')]))
    seated = times >= times[-1] - SEATED_S
    fwd = (toes[0] - hips[0]) * np.array([1.0, 0.0, 1.0])
    fwd = fwd / max(float(np.linalg.norm(fwd)), 1e-9)
    h0, h1 = float(hips[0, 1]), float(hips[seated, 1].mean())
    drop = h0 - h1
    down = hips[:, 1] < h0 - EDGE * drop
    there = hips[:, 1] < h1 + EDGE * drop
    t_down = float(times[down][0]) if down.any() else float(times[-1])
    t_there = float(times[there][0]) if there.any() else float(times[-1])
    trunk = neck - hips
    pitch = np.degrees(np.arctan2(trunk @ fwd, trunk[:, 1]))
    knees = np.mean([180.0 - angle(joints[s + 'UpLeg'] - joints[s + 'Leg'], joints[s + 'Foot'] - joints[s + 'Leg'])
                     for s in ('Left', 'Right')], axis=0)
    shins = [joints[s + 'Leg'] - joints[s + 'Foot'] for s in ('Left', 'Right')]
    shanks = np.mean([np.degrees(np.arctan2(sh @ fwd, sh[:, 1])) for sh in shins], axis=0)
    arms = np.stack([angle(joints[s + 'ForeArm'] - joints[s + 'Arm'], hips - neck) for s in ('Left', 'Right')])
    return {
        'drop': drop / leg,
        'descent': t_there - t_down,
        'hips back': -float((hips[seated] - feet[:, seated].mean(axis=0)).mean(axis=0) @ fwd) / leg,
        'trunk peak': float(pitch[np.argmax(np.abs(pitch))]),
        'trunk seated': float(pitch[seated].mean()),
        'knee seated': float(knees[seated].mean()),
        'shank seated': float(shanks[seated].mean()),
        'feet rise': float((feet[:, :, 1] - feet[:, :1, 1]).max()) / leg,
        'arm range': float((arms.max(axis=1) - arms.min(axis=1)).max()),
        'arm seated': float((arms[:, seated].max(axis=1) - arms[:, seated].min(axis=1)).max()),
    }


def out(measures):
    """[(measure, value, least, most)] of `measures` off SEAT."""
    return [(name, measures[name], lo, hi) for name, _unit, lo, hi in SEAT if not lo <= measures[name] <= hi]


def table(columns):
    """Lines: a measure a row, SEAT's band, a column each of `columns` [(label, measures)], a value
    out of the band marked."""
    width = max(len(label) for label, _m in columns)
    lines = ['%-14s %-5s %-12s ' % ('', '', 'band') + ' '.join('%*s' % (width, label) for label, _m in columns)]
    for name, unit, lo, hi in SEAT:
        cells = ' '.join('%*s' % (width, '%.2f%s' % (m[name], '' if lo <= m[name] <= hi else '*')) for _label, m in columns)
        lines.append('%-14s %-5s %5.2f..%-5.2f ' % (name, unit, lo, hi) + cells)
    return lines


def recorded(path, start, end):
    """(times, joints) of a recording's rows between `start` and `end` s."""
    from tools.sim import fbx, look
    rows = [r for r in look.recorded(path) if start <= float(r['t']) <= end]
    return fbx.joints(rows)


def simulated(scene, verb, seconds):
    """(times, joints, flickers) of her sit here: `scene` laid, `verb` said, `seconds` followed;
    `flickers` how often a sole's load came and went past her soles' touch."""
    from machine import figure, leap, physics, worker
    from machine.figure import JOINTS
    from terminal.views.humanoid_keys import HEADER, row
    from tools.sim import fbx, look
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    s = worker.session()
    worker.apply(s, {'scenario': scene})
    g = s['g']
    world, d = g['world'], g['world'].data
    for _ in range(int(round(0.5 / worker.DT))):
        worker.stepped(s)
    worker.apply(s, {'say': verb})
    rows, told, borne, flickers = [], -1.0, np.zeros(2, bool), 0
    while g['t'] < seconds + 0.5:
        worker.stepped(s)
        loaded = np.array(g['loads']) > leap.TOUCH_N
        flickers += int((loaded != borne).sum())
        borne = loaded
        if g['t'] - told >= 1.0 / look.RATE_HZ:
            told = g['t']
            angles = {j: math.degrees(float(d.qpos[world.qadr[i]])) for i, j in enumerate(JOINTS)}
            now = {'t': g['t'], 'stage': 'sit', 'speed': float(d.qvel[2]), 'phase': 0.0,
                   'loads': tuple(float(v) for v in g['loads']), 'where': tuple(d.qpos[0:3].tolist()),
                   'turn': tuple(d.qpos[3:7].tolist()), 'angles': angles, 'set': angles, 'watts': world.drawn()}
            rows.append(dict(zip(HEADER, row(now, 60.0))))
    times, joints = fbx.joints(rows)
    return times, joints, flickers


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    parser.add_argument('takes', nargs='*', help='FBX takes')
    parser.add_argument('--recording', help='a HUMANOID recording')
    parser.add_argument('--from', dest='start', type=float, default=0.0, help='the recording from, s')
    parser.add_argument('--to', dest='end', type=float, default=math.inf, help='the recording to, s')
    parser.add_argument('--sim', nargs=2, metavar=('SCENE', 'VERB'), help='her sit here')
    parser.add_argument('--seconds', type=float, default=8.0, help='--sim followed, s')
    args = parser.parse_args(argv)
    from tools.sim import fbx
    columns = [(os.path.basename(p)[:12], measured(*fbx.take(p))) for p in args.takes]
    if args.recording:
        columns.append(('page', measured(*recorded(args.recording, args.start, args.end))))
    if args.sim:
        times, joints, flickers = simulated(args.sim[0], args.sim[1], args.seconds)
        columns.append(('%s' % args.sim[1][:12], measured(times, joints)))
        print('her soles came and went %d times in %.1f s' % (flickers, args.seconds))
    print('\n'.join(table(columns)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
