#!/usr/bin/env python3
"""Her falls by the world's truth: what struck first, how hard, how fast; her spasms about it.

    python tools/sim/falls.py                               # her falls, a line each: the slip, the 200 N shoves
    python tools/sim/falls.py shove_200_090_walk_0 slip     # those
    python tools/sim/falls.py --jobs 6 --json               # side by side on the relay, each run whole as JSON

A run a job on the relay (`tools.dev.focus.relay`), read by her analyzers off her loop's snapshots
(`watch.Watch`): landing's touches (`machine.world.struck`), motion's spasms past her own walk's fastest joint,
support's stumble and fall; at its end the driven joints she lies within STOP_DEG of a stop (`sysid.STOP_DEG`).
Times are from her fall, or from her stumble where she stood. `slip` is her walk held into the slip patch
(`introspected`'s), the rest `scenarios.SCENARIOS`'.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: Her falls asked when none is named: her walk into the slip patch, and every 200 N shove's first moment.
FALLS = ('slip',) + tuple('shove_200_%03d_%s_0' % (a, kind) for kind in ('stand', 'walk') for a in range(0, 360, 45))
#: The slip's play: her walk held 4 s into the patch.
SLIP = ('slip', 'w4')


def judged(name):
    """{name, fell, stumble, spasms [[joint, deg/s, s]], touches [[part, m/s, N, s]], stops [joint]} of one fall."""
    import numpy as np
    from machine.identify import sysid
    from machine.process import introspect, watch
    from machine.body.figure import JOINTS
    from tools.sim import scenarios
    if name == 'slip':
        from tools.sim.player import played
        plan, secs = played(SLIP[1])
        scenarios.SCENARIOS['slip_walked'] = (SLIP[0], plan, secs, 'stands', None)
        name_run = 'slip_walked'
    else:
        name_run = name
    tap, judge, told, last = introspect.Tap(), watch.Watch(), [], {}

    def laid(s):
        snap = tap.passed(s)
        if snap is None:
            return
        told.extend(judge.step(snap))
        last['s'], last['angles'] = s, snap['body']['angles']

    got = scenarios.ran(name_run, laid=laid)
    fell = got['fell_at']
    stumbled = next((e['t'] for e in told if e['event'] == 'stumble'), 0.0)
    zero = fell if fell is not None else stumbled
    at = lambda e: round(e['t'] - zero, 3)
    m, driven = last['s']['g']['world'].model, watch.meta()['rated']
    stops = []
    for j in JOINTS:
        k = m.joint(j).id
        if j in driven and m.jnt_limited[k]:
            lo, hi = np.degrees(m.jnt_range[k])
            if min(last['angles'][j] - lo, hi - last['angles'][j]) <= sysid.STOP_DEG:
                stops.append(j)
    return {'name': name, 'fell': fell,
            'stumble': next((at(e) for e in told if e['event'] == 'stumble'), None),
            'spasms': [[e['joint'], e['deg_s'], at(e)] for e in told if e['event'] == 'spasm'],
            'touches': [[e['part'], e['speed_m_s'], e['force_n'], at(e)] for e in told if e['event'] == 'touch'],
            'stops': stops}


def line(r):
    """One fall's line: when, its spasms, the first parts that struck, its head, its hardest, its stops."""
    first = ' '.join('%s %.1f' % (p, v) for p, v, _f, _t in r['touches'][:4])
    head = next(('%.2f m/s %.0f N' % (v, f) for p, v, f, _t in r['touches'] if p == 'head'), '-')
    hard = max(r['touches'], key=lambda x: x[2], default=None)
    worst = max((s[1] for s in r['spasms']), default=0.0)
    return '%-22s fell %-6s spasms %2d (%4.0f deg/s, %s)  first: %-48s head %-16s hardest %-22s stops %s' % (
        r['name'], '%.2f' % r['fell'] if r['fell'] is not None else '-', len(r['spasms']), worst,
        ' '.join('%+.2f' % s[2] for s in r['spasms'][:4]), first, head,
        '%s %.0f N' % (hard[0], hard[2]) if hard else '-', ','.join(r['stops']) or '-')


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('names', nargs='*', help='falls: slip or scenarios (`scenarios.SCENARIOS`), else FALLS')
    p.add_argument('--jobs', type=int, default=6, help='falls side by side (`tools.dev.focus.relay`)')
    p.add_argument('--json', action='store_true', help='each fall whole as JSON')
    p.add_argument('--one', help=argparse.SUPPRESS)
    args = p.parse_args(argv)
    if args.one:
        print('FALL ' + json.dumps(judged(args.one)))
        return 0
    from tools.dev import focus
    here = os.path.abspath(__file__)
    work = [focus.Job(n, [sys.executable, '-X', 'utf8', here, '--one', n], timeout=900.0)
            for n in (args.names or FALLS)]
    for job, text, _code, _s in focus.relay(work, batons=args.jobs):
        got = next((ln[5:] for ln in (text or '').splitlines() if ln.startswith('FALL ')), None)
        if got is None:
            print('%-22s crashed: %s' % (job.name, (text or '')[-300:].replace('\n', ' | ')), flush=True)
            continue
        r = json.loads(got)
        print(json.dumps(r) if args.json else line(r), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
