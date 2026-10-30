#!/usr/bin/env python3
"""Her momentum observer against her skin: each manifold's normal force through her scenes.

    python tools/sim/observed.py                     # walk, sit and rise, lie, rise from the bed: a table
    python tools/sim/observed.py --one chair         # one scene, its line as JSON (the relay's job)

Each law pass, standing or not yet fallen: the observer's normal force on each manifold
(`machine.senses.observer`, blind) against her skin's (`edge.TOUCH`, MuJoCo's contacts) - each segment's
touch along each of its manifolds' normals, pushing, a sole's along its up. Per manifold: the
error's rms and 95th percentile, N; the most it reads where her skin feels under FELT_N.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: The scenes: her walk, the chair's sit and rise, the bed's lie and rise, the floor mattress's.
SCENES = ('play_walk', 'chair', 'bed', 'bed_rise', 'mattress')
#: A manifold her skin feels none of, N: MuJoCo's soft contact holds a resting hand at a few.
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--one', help='one scene, its line as JSON')
    parser.add_argument('scenes', nargs='*', default=list(SCENES))
    args = parser.parse_args(argv)
    if args.one:
        print(json.dumps(one(args.one)))
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
