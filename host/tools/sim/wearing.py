#!/usr/bin/env python3
"""Her wear filter's step response: test_wear's worn knee and hip, window by window, an ensemble.

    python tools/sim/wearing.py                          # walks x the change some windows later, a line each
    python tools/sim/wearing.py --course --after 48      # one run, the knee's f a window at a time, to 48 s on
    python tools/sim/wearing.py --against C:/sources/coaxial_ab     # each run on another tree too, beside

test_wear's own case (`tests/test_wear.py`: its session, its phases and changes, of the tree run): walked as built
NOMINAL_S, a load in her left hand LOADED_S, then her left knee's friction FRICTION_X, her left hip's torque an
amp TORQUE_X and her right elbow's encoder zero ZERO_RAD, read WORN_S on. A run's line: the knee's f and its
margin - f less SIGMAS of its doubt less its band, named where positive -, the hip's kt's, the nearest other
drift's (a false alarm where positive), what is named. A run a job on the relay (`tools.dev.focus.relay`).
"""
import argparse
import json
import os
import sys

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, HOST)

#: The ensemble: her walk's speeds, m/s, and the change a whole number of wear windows later.
SPEEDS, LATER = (0.66, 0.69, 0.72), (0, 1, 2, 3)


def read(t, band):
    """{knee f, its margin, hip kt, its margin, the nearest other drift and its margin, named} of `told`'s wear."""
    from machine.identify import wear
    margin = {}
    for name, b in band.items():
        site, kind = name.rsplit('.', 1)
        if kind in ('f', 'b', 'kt'):
            margin[name] = abs(t['drift'][site][kind]) - wear.SIGMAS * t['doubt'][name] - b
    other = max((v, n) for n, v in margin.items() if n not in ('left_knee.f', 'left_hip.kt'))
    return {'f': round(t['drift']['left_knee']['f'], 3), 'f_margin': round(margin['left_knee.f'], 3),
            'kt': round(t['drift']['left_hip']['kt'], 3), 'kt_margin': round(margin['left_hip.kt'], 3),
            'other': other[1], 'other_margin': round(other[0], 3), 'named': sorted(a[0] for a in t['alarms'])}


def judged(speed, later, after, course):
    """{speed, later, t0, rows [{at, ..`read`}]} of test_wear's worn phase, the change `later` windows on and read
    `after` s past it, or each window to it where `course`."""
    import numpy as np
    import test_wear as case
    from machine.identify import wear
    from machine.process import worker
    from machine.body.figure import JOINTS
    s, w = case._laid()
    g = s['g']
    world, m = g['world'], g['world'].model

    def walked(until):
        told = g['t'] >= 1.0
        while g['t'] < until and s['fallen_at'] is None:
            if g['t'] >= 1.0 and not told:
                worker.apply(s, {'twist': (speed, 0.0, 0.0)})
                told = True
            worker.stepped(s)
    hand = m.body('left_hand').id
    walked(case.NOMINAL_S)
    case._changed(world, lambda m: m.body_mass.__setitem__(hand, m.body_mass[hand] + case.LOAD_KG))
    t0 = case.NOMINAL_S + case.LOADED_S + later * wear.WINDOW_S
    walked(t0)
    knee, hip, elbow = world.vadr[JOINTS.index('left_knee')], m.joint('left_hip').id, JOINTS.index('right_elbow')
    act = int(np.flatnonzero(m.actuator_trnid[:, 0] == hip)[0])

    def worn(m):
        m.dof_frictionloss[knee] *= case.FRICTION_X
        m.actuator_gear[act, 0] *= case.TORQUE_X
    case._changed(world, worn)
    world.boxes.zero[elbow] = case.ZERO_RAD
    band = dict(zip(w['names'], w['band']))
    rows = []
    for at in (np.arange(wear.WINDOW_S, after + 1e-9, wear.WINDOW_S) if course else (after,)):
        walked(t0 + at)
        rows.append(dict(at=round(float(at), 1), **read(worker.told(s)['wear'], band)))
    return {'speed': speed, 'later': later, 't0': t0, 'fallen': s['fallen_at'], 'rows': rows}


def line(name, r):
    """A run's lines: a row each of what it read."""
    return '\n'.join('%-26s +%5.1f s  f %+.3f m %+.3f | hip kt %+.3f m %+.3f | %-15s %+.3f | %s%s' % (
        name, row['at'], row['f'], row['f_margin'], row['kt'], row['kt_margin'], row['other'], row['other_margin'],
        ','.join(row['named']), ' FELL %.1f' % r['fallen'] if r['fallen'] else '') for row in r['rows'])


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('--course', action='store_true', help="one run (the test's walk, no window later), each window")
    p.add_argument('--after', type=float, help='read this many s past the change (the test\'s WORN_S where none)')
    p.add_argument('--against', help='another tree (its root), each run there too')
    p.add_argument('--jobs', type=int, default=6, help='runs side by side (`tools.dev.focus.relay`)')
    p.add_argument('--one', nargs=5, help=argparse.SUPPRESS)
    args = p.parse_args(argv)
    if args.one:
        root, speed, later, after, course = args.one
        sys.path[:0] = [os.path.join(root, 'host'), os.path.join(root, 'host', 'tests')]
        os.environ['PYTHONPATH'] = os.path.join(root, 'host')
        import test_wear as case
        got = judged(float(speed), int(later), float(after) if after != '-' else case.WORN_S, course == '1')
        print('WEARING ' + json.dumps(got))
        return 0
    from tools.dev import focus
    trees = [os.path.dirname(HOST)] + ([os.path.abspath(args.against)] if args.against else [])
    runs = [(0.69, 0)] if args.course else [(v, k) for v in SPEEDS for k in LATER]
    after = '-' if args.after is None else str(args.after)
    work = [focus.Job('%s v%.2f +%d' % (os.path.basename(tree), v, k),
                      [sys.executable, '-X', 'utf8', os.path.abspath(__file__), '--one', tree, str(v), str(k), after,
                       '1' if args.course else '0'], timeout=900.0)
            for v, k in runs for tree in trees]
    for job, text, _code, _s in focus.relay(work, batons=args.jobs):
        got = next((ln[8:] for ln in (text or '').splitlines() if ln.startswith('WEARING ')), None)
        print(line(job.name, json.loads(got)) if got else '%-26s no run: %s' % (
            job.name, (text or 'out of time')[-200:].replace('\n', ' | ')), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
