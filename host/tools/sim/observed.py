#!/usr/bin/env python3
"""The robot's momentum observer against its skin: each manifold's normal force through its scenes.

    python tools/sim/observed.py                     # walk, sit and rise, lie, rise from the bed: a table
    python tools/sim/observed.py --one chair         # one scene, its line as JSON (the relay's job)

Each law pass, standing or not yet fallen: the observer's normal force on each manifold
(`machine.senses.observer`, blind) against the robot's skin's (`edge.TOUCH`, MuJoCo's contacts) - each
segment's touch along each of its manifolds' normals, pushing, a sole's along its up. Per manifold: the
error's rms and 95th percentile, N; the most it reads where the robot's skin feels under FELT_N.
"""
import argparse
import json
import os
import sys
import typing

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: The scenes: the robot's walk, the chair's sit and rise, the bed's lie and rise, the floor mattress's.
SCENES = ('play_walk', 'chair', 'bed', 'bed_rise', 'mattress')
#: A manifold the robot's skin feels none of, N: MuJoCo's soft contact holds a resting hand at a few.
FELT_N = 5.0


def one(name):
    """{scene, fell_at, passes, per manifold: rms, p95, ghost N}."""
    import numpy as np
    from machine.senses import observer
    from tools.sim import scenarios
    rows, last = [], [None]

    def laid(s):
        g = s['g']
        if s['fallen_at'] is None and g.get('t_law') != last[0] and g['t'] > 1.0:
            last[0] = g['t_law']
            rows.append((g['observer']['normal'].copy(), observer.skin(g['world'], g['sense'])))

    out = scenarios.ran(name, laid=laid)
    got, truth = (np.array([r[k] for r in rows]) for k in (0, 1))
    err = got - truth
    line = {'scene': name, 'fell_at': out['fell_at'], 'passes': len(rows)}
    for k, manifold in enumerate(observer.MANIFOLDS):
        none = truth[:, k] < FELT_N
        line[manifold] = {'rms': round(float(np.sqrt((err[:, k] ** 2).mean())), 1),
                          'p95': round(float(np.percentile(np.abs(err[:, k]), 95)), 1),
                          'felt': round(float(truth[:, k].max()), 1),
                          'ghost': round(float(got[none, k].max()), 1) if none.any() else None}
    return line


def _truth(world, names, at):
    """MuJoCo's own push on the robot's dofs this pass, in its observer's (`observer.step`: the pelvis's turn and its
    push in its own frame, then each joint by name): every contact's and joint stop's constraint force through its
    Jacobian, the robot's skin and stops - what the observer's residual reads, its model aside."""
    import numpy as np
    mj, m, d = world._mj, world.model, world.data
    kinds = (mj.mjtConstraint.mjCNSTR_CONTACT_FRICTIONLESS, mj.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL,
             mj.mjtConstraint.mjCNSTR_CONTACT_ELLIPTIC, mj.mjtConstraint.mjCNSTR_LIMIT_JOINT)
    force = np.where(np.isin(d.efc_type[:d.nefc], kinds), d.efc_force[:d.nefc], 0.0)
    qfrc = np.zeros(m.nv)
    mj.mj_mulJacTVec(m, d, qfrc, force)
    root = next(j for j in range(m.njnt) if m.jnt_type[j] == mj.mjtJoint.mjJNT_FREE and m.body(m.jnt_bodyid[j]).name == 'pelvis')
    dof, out = int(m.jnt_dofadr[root]), np.zeros(5 + len(names))
    rot = d.xmat[m.jnt_bodyid[root]].reshape(3, 3)
    out[:3], out[3:6] = qfrc[dof + 3:dof + 6], rot.T @ qfrc[dof:dof + 3]
    for i, name in enumerate(names[1:], start=6):
        j = at.get(name)
        out[i] = qfrc[int(m.jnt_dofadr[j])] if j is not None else np.nan
    return out


def rows(name, since, until, top=4, ghost=60.0):
    """Each law pass of `name` from `since` to `until` s where its observer reads `ghost` N or more on a manifold its
    skin feels nothing of: the observer's residual a pass (its low-pass undone) against MuJoCo's own push (`_truth`),
    the `top` rows most apart - (dof, observer's, MuJoCo's), N m or N."""
    import math
    import numpy as np
    from machine.senses import observer
    from tools.cores import wbc as cores
    from tools.sim import scenarios
    names = cores.Core().names
    dof = lambda i: ('turn', 'turn', 'turn', 'push', 'push', 'push')[i] + 'xyz'[i % 3] if i < 6 else names[i - 5]
    out, last = [], dict.fromkeys(('law', 't', 'r'), typing.cast(typing.Any, None))

    def laid(s):
        g = s['g']
        if s['fallen_at'] is not None or g.get('t_law') == last['law']:
            return
        last['law'], obs, t = g.get('t_law'), g['observer'], float(g['t'])
        r, before, then = np.array(obs['r'], float), last['r'], last['t']
        last['t'], last['r'] = t, r
        if before is None or then is None or not since <= t <= until:
            return
        e = before + (r - before) / (1.0 - math.exp(-float(obs['c'].alpha) * (t - then)))
        felt = observer.skin(g['world'], g['sense'])
        worst = max([float(obs['normal'][k]) for k in range(len(felt)) if felt[k] < FELT_N] + [0.0])
        if worst < ghost:
            return
        m = g['world'].model
        at = {m.joint(j).name: j for j in range(m.njnt)}
        truth = _truth(g['world'], names, at)
        apart = np.nan_to_num(e - truth)
        out.append({'t': round(t, 3), 'ghost': round(worst, 1),
                    'rows': [(dof(int(i)), round(float(e[i]), 1), round(float(truth[i]), 1)) for i in np.argsort(-np.abs(apart))[:top]]})

    scenarios.ran(name, laid=laid)
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--one', help='one scene, its line as JSON')
    parser.add_argument('--rows', nargs=3, metavar=('SCENE', 'FROM', 'TO'),
                        help="its ghosts' passes: the observer's residual against MuJoCo's push, its rows most apart")
    parser.add_argument('scenes', nargs='*', default=list(SCENES))
    args = parser.parse_args(argv)
    if args.one:
        print(json.dumps(one(args.one)))
        return 0
    if args.rows:
        for line in rows(args.rows[0], float(args.rows[1]), float(args.rows[2])):
            print(json.dumps(line))
        return 0
    from machine.senses import observer
    from tools.dev import focus
    jobs = [focus.Job(n, [sys.executable, '-X', 'utf8', os.path.abspath(__file__), '--one', n], timeout=1800.0)
            for n in args.scenes]
    print('%-10s %-8s %s' % ('scene', 'fell', '  '.join('%-26s' % m for m in observer.MANIFOLDS)))
    for _job, text, _code, _seconds in focus.relay(jobs):
        line = json.loads(text.strip().splitlines()[-1])
        print('%-10s %-8s %s' % (line['scene'], line['fell_at'], '  '.join(
            '%5.1f %5.1f %6.1f %6s' % (line[m]['rms'], line[m]['p95'], line[m]['felt'], line[m]['ghost'])
            for m in observer.MANIFOLDS)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
