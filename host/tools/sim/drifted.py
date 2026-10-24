#!/usr/bin/env python3
"""Her two estimates of her pelvis against the world's: her estimate's and the right-invariant EKF's.

    python tools/sim/drifted.py                     # her walk, her turns, her moves: a table
    python tools/sim/drifted.py --one play_walk     # one scene, its line as JSON (the relay's job)

Each law pass from her first stride (`machine.estimate`, `machine.inekf`): how far each has drifted
along the floor and in height by the end, m, against how far she went; its velocity's error at the
90th percentile, m/s.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

SCENES = ('play_walk', 'play_turns', 'play_moves', 'play_strafes')
#: From her first stride, s.
SINCE_S = 1.0


def one(name):
    """{scene, went m, per estimate: drift m along the floor, in height, velocity's error p90 m/s}."""
    import numpy as np
    from tools.sim import scenarios
    rows, last = [], [None]

    def laid(s):
        g = s['g']
        if s['fallen_at'] is None and g.get('t_law') != last[0] and g['t'] >= SINCE_S:
            last[0] = g['t_law']
            d = g['world'].data
            rows.append(np.concatenate([d.qpos[0:3], d.qvel[0:3], g['est']['pos'], g['est']['vel'],
                                        g['inekf']['p'], g['inekf']['v']]))

    out = scenarios.ran(name, laid=laid)
    r = np.array(rows)
    line = {'scene': name, 'fell_at': out['fell_at'],
            'went': round(float(np.linalg.norm(r[-1, [0, 2]] - r[0, [0, 2]])), 2)}
    for k, who in ((6, 'estimate'), (12, 'inekf')):
        e = r[:, k:k + 3] - r[:, 0:3]
        e -= e[0]
        line[who] = {'floor': round(float(np.linalg.norm(e[-1, [0, 2]])), 3), 'height': round(float(e[-1, 1]), 3),
                     'speed': round(float(np.percentile(np.linalg.norm(r[:, k + 3:k + 6] - r[:, 3:6], axis=1), 90)), 3)}
    return line


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--one', help='one scene, its line as JSON')
    parser.add_argument('scenes', nargs='*', default=list(SCENES))
    args = parser.parse_args(argv)
    if args.one:
        print(json.dumps(one(args.one)))
        return 0
    from tools.dev import focus
    jobs = [focus.Job(n, [sys.executable, '-X', 'utf8', os.path.abspath(__file__), '--one', n], timeout=1800.0)
            for n in args.scenes]
    print('%-14s %-7s %6s   %-24s %-24s' % ('scene', 'fell', 'went', 'estimate floor/height/v', 'inekf floor/height/v'))
    for _job, text, _code, _seconds in focus.relay(jobs):
        line = json.loads(text.strip().splitlines()[-1])
        print('%-14s %-7s %6.2f   %-24s %-24s' % (line['scene'], line['fell_at'], line['went'], *(
            '%.3f %+.3f %.3f' % (line[w]['floor'], line[w]['height'], line[w]['speed']) for w in ('estimate', 'inekf'))))
    return 0


if __name__ == '__main__':
    sys.exit(main())
