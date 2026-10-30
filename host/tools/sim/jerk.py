#!/usr/bin/env python3
"""Her jerk through transitions against her steady walk's, each segment of her.

The DOD: no transition jerks her more than her walk does.

    python tools/sim/jerk.py                       # every transition, a table of ratios
    python tools/sim/jerk.py --one play_weave 0 99 # one scenario's peaks in a window, as JSON

Each segment's jerk is the referee's (`scenarios.JERKED`, `JERK_S`): a 20 ms window's third
difference; a peak is its largest. Her walk's peaks are `play_walk`'s from STEADY on, past its
start; a transition's are over its whole run.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: Her steady walk: `play_walk` between these, s.
STEADY = (4.0, 10.0)
#: The transitions judged against it: the plays of a player's keys, a seat, a hop.
TRANSITIONS = ('play_moves', 'play_start_stop', 'play_turns', 'play_strafes', 'play_reverse', 'play_sprint',
               'play_spin', 'play_weave', 'chair', 'hop_down')


def peaks(name, t0, t1):
    """{segment: peak jerk, m/s^3} of scenario `name` between `t0` and `t1` s, and whether it met
    its goal."""
    import numpy as np
    from machine.process import worker
    from tools.sim import scenarios
    real, rows = worker.stepped, []

    def stepped(s):
        real(s)
        g, w = s['g'], s['g']['world']
        ids = g.setdefault('_jerked', [w.model.body(n).id for n in scenarios.JERKED])
        rows.append([g['t']] + w.data.xpos[ids].ravel().tolist())
    worker.stepped = stepped
    try:
        met = scenarios.ran(name, 0.0)['met']
    finally:
        worker.stepped = real
    t, w = np.array(rows), int(scenarios.JERK_S / 0.001)
    out = {}
    for k, seg in enumerate(scenarios.JERKED):
        p = np.cumsum(t[:, 1 + 3 * k:4 + 3 * k], axis=0)
        j = np.linalg.norm(np.diff((p[w:] - p[:-w]) / w, n=3, axis=0), axis=1) / 0.001 ** 3
        at = t[w + 3:, 0][:len(j)] - 0.5 * scenarios.JERK_S
        out[seg] = round(float(j[(at >= t0) & (at <= t1)].max()), 1)
    return out, met


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--one', nargs=3, metavar=('NAME', 'FROM', 'TO'), help="one run's peaks, as JSON")
    args = parser.parse_args(argv)
    if args.one:
        got, met = peaks(args.one[0], float(args.one[1]), float(args.one[2]))
        print(json.dumps({'peaks': got, 'met': met}))
        return 0
    from tools.dev import focus
    from tools.sim import scenarios
    me = [sys.executable, '-X', 'utf8', os.path.abspath(__file__), '--one']
    jobs = [focus.Job('play_walk', me + ['play_walk', str(STEADY[0]), str(STEADY[1])], timeout=900.0)]
    jobs += [focus.Job(n, me + [n, '0', '99'], timeout=900.0) for n in TRANSITIONS]
    got = {}
    for job, text, _code, _s in focus.relay(jobs):
        line = next((ln for ln in reversed((text or '').splitlines()) if ln.startswith('{')), None)
        got[job.name] = json.loads(line) if line else {'peaks': {}, 'met': False}
    walk = got['play_walk']['peaks']
    print('%-16s %s' % ('steady walk', ' '.join('%10s' % s[:10] for s in scenarios.JERKED)))
    print('%-16s %s' % ('m/s^3', ' '.join('%10.0f' % walk.get(s, 0.0) for s in scenarios.JERKED)))
    for name in TRANSITIONS:
        p = got[name]['peaks']
        print('%-16s %s%s' % (name, ' '.join('%10.1f' % (p.get(s, 0.0) / max(walk.get(s, 1.0), 1.0))
                                             for s in scenarios.JERKED), '' if got[name]['met'] else '  (goal missed)'))
    worst = max(got[n]['peaks'].get(s, 0.0) / max(walk.get(s, 1.0), 1.0) for n in TRANSITIONS for s in scenarios.JERKED)
    print('the worst, of her walk: %.1f' % worst)
    return 0


if __name__ == '__main__':
    sys.exit(main())
