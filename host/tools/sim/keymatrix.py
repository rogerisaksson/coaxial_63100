#!/usr/bin/env python3
"""Every handover between her keys, judged as her analyzers judge her: one key held, the next on.

    python tools/sim/keymatrix.py                       # every ordered pair at each phase on the relay: sums, bad rows
    python tools/sim/keymatrix.py --one FROM TO HELD    # one run here, its row as JSON

A run on the flat floor: her key FROM (`player.KEYS`) held HELD s from 1 s, TO held TO_S, then none STILL_S -
HELDS press TO at other phases of her stride. Judged from TO's press: her fall, her stumbles (`support.stumble`),
her surety's least, her soles' heaviest load of her weight, her drift across her heading past what TO asks (m/s,
its most) and her yaw rate's wobble about what TO asks (rad/s, its spread over TO's last second).
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: Her keys as a player holds them (`player.KEYS`), none the last.
KEYS = ('w', 'wa', 'wd', 'a', 's', 'q', 'e', 'W', '-')
#: FROM held this long, s: TO pressed at three phases of her stride; TO held, s; none after, s.
HELDS, TO_S, STILL_S = (2.5, 2.7, 2.9), 2.5, 3.0


def one(frm, to, held):
    """A row of FROM held `held` s, then TO."""
    import numpy as np
    from machine.law import modal
    from machine.process import introspect, watch
    from tools.sim import player, scenarios
    press, end = 1.0 + held, 1.0 + held + TO_S
    plan = [(1.0, {'twist': player.KEYS[frm]}), (press, {'twist': player.KEYS[to]}), (end, {'twist': player.STOP})]
    scenarios.SCENARIOS['keymatrix'] = ('flat', plan, end + STILL_S, 'stands', None)
    vx, vy, wz = player.KEYS[to]
    tap, judge, told = introspect.Tap(), watch.Watch(), []
    seen = {'load': 0.0, 'surety': 1.0, 'drift': 0.0, 'yaw': [], 'last': None}

    def laid(s):
        g = s['g']
        snap = tap.passed(s)
        if snap is not None:
            told.extend(judge.step(snap))
        t, d = float(g['t']), g['world'].data
        if not press <= t <= end or g['n'] % g['every'] != 1:
            return
        w, x, y, z = (float(q) for q in d.qpos[3:7])
        heading = math.atan2(2.0 * (x * z + w * y), 1.0 - 2.0 * (x * x + y * y))
        across = float(d.qvel[0]) * math.cos(heading) - float(d.qvel[2]) * math.sin(heading)
        seen['load'] = max(seen['load'], max(float(f) for f in g['loads']) / float(g['fr'][modal.WEIGHT]))
        seen['surety'] = min(seen['surety'], float(g['surety']))
        seen['drift'] = max(seen['drift'], abs(abs(across) - abs(vy)))
        if seen['last'] is not None and t >= end - 1.0:
            seen['yaw'].append(math.remainder(heading - seen['last'][1], math.tau) / (t - seen['last'][0]) - wz)
        seen['last'] = (t, heading)

    out = scenarios.ran('keymatrix', laid=laid)
    fell = out['fell_at']
    stumbles = [round(e['t'] - press, 2) for e in told if e['event'] == 'stumble' and e['t'] >= press]
    return {'from': frm, 'to': to, 'held': held, 'fell': None if fell is None else round(fell - press, 2),
            'stumbles': stumbles[:4], 'surety': round(seen['surety'], 3), 'load': round(seen['load'], 2),
            'drift': round(seen['drift'], 3), 'wobble': round(float(np.std(seen['yaw'])) if seen['yaw'] else 0.0, 3),
            'result': 'bad' if fell is not None or stumbles else 'ok'}


def _relayed(argvs):
    """Each run its own process on the relay's batons (`focus.relay`): their rows."""
    from tools.dev import focus
    jobs = [focus.Job('keymatrix_%d' % k, [sys.executable, '-X', 'utf8', os.path.abspath(__file__)] + a, 0.3, 900.0, None)
            for k, a in enumerate(argvs)]
    lines = [[ln for ln in (text or '').splitlines() if ln.startswith('{')] for _job, text, _code, _s in focus.relay(jobs)]
    return [dict(json.loads(ln[-1])) if ln else dict(result='crashed') for ln in lines]


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('--one', nargs=3, metavar=('FROM', 'TO', 'HELD'), help='one run, here')
    args = p.parse_args(argv)
    if args.one:
        print(json.dumps(one(args.one[0], args.one[1], float(args.one[2]))), flush=True)
        return 0
    rows = _relayed([['--one', a, b, str(h)] for a in KEYS for b in KEYS if a != b for h in HELDS])
    bad = [r for r in rows if r.get('result') != 'ok']

    def med(k):
        vals = sorted(r[k] for r in rows if k in r)
        return round(float(vals[len(vals) // 2]), 3) if vals else None

    print(json.dumps({'runs': len(rows), 'bad': len(bad), 'fell': sum(r.get('fell') is not None for r in rows),
                      'load_med': med('load'), 'load_max': max((r.get('load', 0.0) for r in rows), default=None),
                      'surety_min': min((r.get('surety', 1.0) for r in rows), default=None),
                      'drift_med': med('drift'), 'wobble_med': med('wobble')}), flush=True)
    for r in bad:
        print(json.dumps(r), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
