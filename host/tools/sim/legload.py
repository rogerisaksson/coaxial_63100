#!/usr/bin/env python3
"""What a knee, an elbow or an ankle carries through a scenario: the force or the torques across it.

    python -X utf8 tools/sim/legload.py [names ...] [--joint knee|elbow|ankle] [--json]
    python -X utf8 tools/sim/legload.py [names ...] --all [--json]

Each scenario (`scenarios.SCENARIOS`) runs as falls.py runs it. Each pass, MuJoCo's interaction force of each outer
segment (OUTER: the shank, the forearm) with the one before it (`cfrc_int`, after `mj_rnePostConstraint`) is moved
from the subtree's centre to the joint's anchor and turned into the outer segment's frame: along its length (axial,
N), about the joint's axis (the swing, N m), about the third axis (across the swing, the lateral, N m) and about its
length (the torsion, N m). The most of each, either side, is told, the axial also as the robot's weights - what the
joint's mount is sized against (tools/render/knee_loads.py). `--all` tells every mounted joint's at once (`borne`):
the force and the moment at its anchor along and about its outer segment's own x, y and z, up to where the robot
fell if it did - a fall's loads are the fall's.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: The scenarios a knee bears most in: a hop down from a chair and up onto one, a chair, a bed, stairs down, a
#: slope across the way and a slip.
HARDEST = ('hop_down', 'hop_up', 'chair', 'bed', 'stairs_down', 'cross_slope', 'slip')


#: The cells a hip's pose is told in, deg.
CELL_DEG = 5
#: The segment past each joint, whose force from the segment before it is read (a side's where it has sides).
OUTER = {'knee': 'shank', 'elbow': 'forearm', 'hip_yaw': 'hip_yaw', 'hip_roll': 'hip_roll', 'hip': 'thigh',
         'spine_roll': 'spine_roll', 'spine': 'torso', 'neck': 'neck', 'head': 'head', 'shoulder': 'upper_arm'}


def carried(name, joint='knee'):
    """{name, axial, swing, lateral, torsion: [peak, s]} of one scenario, N and N m, either leg."""
    import mujoco
    import numpy as np
    from tools.sim import scenarios
    peak = {k: [0.0, 0.0] for k in ('axial', 'swing', 'lateral', 'torsion')}
    series = {k: [] for k in peak}

    def laid(s):
        world = s.get('g', {}).get('world') if isinstance(s, dict) else None
        if world is None:
            return
        m, d = world.model, world.data
        mujoco.mj_rnePostConstraint(m, d)  # pyright: ignore[reportAttributeAccessIssue]
        for side in ('left', 'right'):
            b, j = m.body(side + '_' + OUTER[joint]).id, m.joint(side + '_' + joint).id
            torque, force = d.cfrc_int[b, :3], d.cfrc_int[b, 3:]
            centre = d.subtree_com[m.body_rootid[b]]
            moment = torque + np.cross(centre - d.xanchor[j], force)
            frame = d.xmat[b].reshape(3, 3)
            for key, axis, value in (('axial', 1, force), ('swing', 0, moment), ('lateral', 2, moment),
                                     ('torsion', 1, moment)):
                v = abs(float(value @ frame[:, axis]))
                series[key].append((float(d.time), v))
                if v > peak[key][0]:
                    peak[key] = [v, float(d.time)]

    scenarios.ran(name, laid=laid)
    return {'name': name, **{k: [round(v[0], 1), round(v[1], 3), round(_lasted(series[k], v[0]), 4)]
                             for k, v in peak.items()}}


def borne(name):
    """{name, stood, fell_at, joint: {force, moment: [x, y, z] the most of each, N and N m, at: [six] s when,
    lasted: [six] s above half that most, turned: [least, most] deg its angle went}}: every mounted joint's
    (OUTER) through one scenario, either side, in its outer segment's frame at its anchor - up to the robot's
    fall where it fell; hips: [[pitch, roll, yaw]] the cells of CELL_DEG a hip was posed in, the right's mirrored
    to the left's; feet: [[pitch, roll, knee]] the cells an ankle was posed in with its knee, likewise."""
    import mujoco
    import numpy as np
    from tools.sim import scenarios
    rows, times, series, angles = [], [], {joint: [] for joint in OUTER}, {joint: [] for joint in OUTER}
    posed, footed = [], []

    def laid(s):
        world = s.get('g', {}).get('world') if isinstance(s, dict) else None
        if world is None:
            return
        m, d = world.model, world.data
        if not rows:
            names = {m.body(b).name for b in range(m.nbody)}
            for joint, segment in OUTER.items():
                sides = [side for side in ('left_', 'right_') if side + segment in names] or ['']
                rows.append((joint, [(m.body(side + segment).id, m.joint(side + joint).id) for side in sides]))
        mujoco.mj_rnePostConstraint(m, d)  # pyright: ignore[reportAttributeAccessIssue]
        centre = d.subtree_com
        times.append(float(d.time))
        posed.append([[way * float(np.degrees(d.qpos[m.jnt_qposadr[m.joint(side + j).id]]))
                       for j, way in (('hip', 1.0), ('hip_roll', mirror), ('hip_yaw', mirror))]
                      for side, mirror in (('left_', 1.0), ('right_', -1.0))])
        footed.append([[way * float(np.degrees(d.qpos[m.jnt_qposadr[m.joint(side + j).id]]))
                        for j, way in (('ankle', 1.0), ('ankle_roll', mirror), ('knee', 1.0))]
                       for side, mirror in (('left_', 1.0), ('right_', -1.0))])
        for joint, pairs in rows:
            six = np.zeros(6)
            for b, j in pairs:
                torque, force = d.cfrc_int[b, :3], d.cfrc_int[b, 3:]
                moment = torque + np.cross(centre[m.body_rootid[b]] - d.xanchor[j], force)
                frame = d.xmat[b].reshape(3, 3)
                six = np.maximum(six, np.abs(np.concatenate([force @ frame, moment @ frame])))
            series[joint].append(six)
            turns = [float(d.qpos[m.jnt_qposadr[j]]) for _b, j in pairs]
            angles[joint].append((min(turns), max(turns)))

    told = scenarios.ran(name, laid=laid)
    fell = told.get('fell_at')
    t = np.array(times)
    upto = int(np.searchsorted(t, fell)) if fell is not None else len(t)
    out = {'name': name, 'stood': bool(told.get('stood')), 'fell_at': fell}
    for key, told_ in (('hips', posed), ('feet', footed)):
        cells = np.unique(np.floor(np.array(told_)[:upto].reshape(-1, 3) / CELL_DEG).astype(int), axis=0)
        out[key] = (cells * CELL_DEG).tolist()
    for joint, got in series.items():
        v = np.array(got)[:upto].reshape(-1, 6)
        top = v.max(axis=0, initial=0.0)
        when = t[:upto][v.argmax(axis=0)] if len(v) else np.zeros(6)
        out[joint] = {'force': np.round(top[:3], 1).tolist(), 'moment': np.round(top[3:], 1).tolist(),
                      'at': np.round(when, 3).tolist(),
                      'turned': np.round(np.degrees([np.min(np.array(angles[joint])[:upto, 0], initial=0.0),
                                                     np.max(np.array(angles[joint])[:upto, 1], initial=0.0)]),
                                         1).tolist(),
                      'lasted': [round(_lasted(list(zip(t[:upto], v[:, k])), float(top[k])), 4) for k in range(6)]}
    return out


def ankle(name):
    """{name, pitch, roll: [peak N m, s, above half s], drive: [peak N m, s, above half s], speed: peak rad/s}: the
    ankle's two torques through a scenario and what each of its two drives makes of them - the cranks' torques by
    virtual work through the rods (`linkage.crank`), either leg, either drive."""
    import mujoco
    import numpy as np
    from machine.body import linkage
    from tools.sim import scenarios
    rows = []

    def laid(s):
        world = s.get('g', {}).get('world') if isinstance(s, dict) else None
        if world is None:
            return
        m, d = world.model, world.data
        for side in ('left_', 'right_'):
            q = [float(np.degrees(d.qpos[m.jnt_qposadr[m.joint(side + j).id]])) for j in ('ankle', 'ankle_roll')]
            tau = [float(d.qfrc_actuator[m.jnt_dofadr[m.joint(side + j).id]]) for j in ('ankle', 'ankle_roll')]
            rows.append((float(d.time), side, q, tau))

    scenarios.ran(name, laid=laid)
    h = 0.25
    out = {k: [0.0, 0.0] for k in ('pitch', 'roll', 'drive')}
    series = {k: [] for k in out}
    speed, last = 0.0, {}
    for t, side, (qp, qr), (tp, tr) in rows:
        cr = [[linkage.crank('left_' + k, qp + dp, qr + dr) for dp, dr in ((h, 0.0), (-h, 0.0), (0.0, h), (0.0, -h))]
              for k in ('ankle', 'ankle_roll')]
        jac = np.array([[(c[0] - c[1]) / np.radians(2 * h), (c[2] - c[3]) / np.radians(2 * h)] for c in cr])
        drive = np.linalg.solve(jac.T, np.array([tp, tr]))
        theta = np.array([linkage.crank('left_' + k, qp, qr) for k in ('ankle', 'ankle_roll')])
        if side in last and t > last[side][0]:
            speed = max(speed, float(np.max(np.abs(theta - last[side][1]))) / (t - last[side][0]))
        last[side] = (t, theta)
        for key, v in (('pitch', abs(tp)), ('roll', abs(tr)), ('drive', float(np.max(np.abs(drive))))):
            series[key].append((t, v))
            if v > out[key][0]:
                out[key] = [v, t]
    return {'name': name, 'speed': round(speed, 1),
            **{k: [round(v[0], 1), round(v[1], 3), round(_lasted(series[k], v[0]), 4)] for k, v in out.items()}}


def _lasted(series, top):
    """How long, s, the value stayed above half its peak about the peak."""
    import numpy as np
    if not series or top <= 0.0:
        return 0.0
    t, v = np.array(series).T
    i = int(np.argmax(v))
    above = v >= 0.5 * top
    lo, hi = i, i
    while lo > 0 and above[lo - 1]:
        lo -= 1
    while hi < len(v) - 1 and above[hi + 1]:
        hi += 1
    return float(t[hi] - t[lo])


def main(argv=None):
    from machine.body import compile
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('names', nargs='*')
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--joint', default='knee', choices=('knee', 'elbow', 'ankle'))
    parser.add_argument('--all', action='store_true')
    args = parser.parse_args(argv)
    weight = compile.G_M_S2 * float(compile.gait()['mass_kg'])
    for name in args.names or HARDEST:
        if args.all:
            got = borne(name)
            print(json.dumps(got) if args.json else '\n'.join(
                ['%s: stood %s, fell at %s' % (name, got['stood'], got['fell_at'])] + [
                    '  %-10s force %s N | moment %s N m' % (joint, got[joint]['force'], got[joint]['moment'])
                    for joint in OUTER]))
            continue
        if args.joint == 'ankle':
            got = ankle(name)
            print(json.dumps(got) if args.json else '%-12s pitch %5.1f | roll %5.1f | a drive %5.1f N m (above half %s ms)'
                  ' | a crank %.1f rad/s' % (name, got['pitch'][0], got['roll'][0], got['drive'][0],
                                             '/'.join('%.0f' % (1e3 * got[k][2]) for k in ('pitch', 'roll', 'drive')),
                                             got['speed']))
            continue
        got = carried(name, args.joint)
        if args.json:
            print(json.dumps(got))
            continue
        print('%-12s axial %6.0f N (%.2f weights) | swing %5.0f | lateral %5.0f | torsion %5.0f N m'
              ' | above half the peak %s ms' % (
                  name, got['axial'][0], got['axial'][0] / weight, got['swing'][0], got['lateral'][0],
                  got['torsion'][0], '/'.join('%.0f' % (1e3 * got[k][2]) for k in ('axial', 'swing', 'lateral',
                                                                                    'torsion'))))
    return 0


if __name__ == '__main__':
    sys.exit(main())
