#!/usr/bin/env python3
"""Her keys taking her errand over and handing it back as she moves, judged as her analyzers do.

    python tools/sim/takeover.py                          # every case from every spot on the relay: a sum, the bad rows
    python tools/sim/takeover.py --one X Z YAW CASE AT    # one run here, its row as JSON

A case from a cell of the lodge's free floor at a heading (`house.spots`, `house.HEADINGS`), her chair errand as F
offers it there (`gym.offered`, `gym.chose`): `taken` - AT s into the errand's walk her keys' WA held TAKE_S, let
go, STAND_S on; `handed` - her keys' W held WALK_S, let go, and AT s on her chair errand chosen as F does while
she still moves, run to its end or LIMIT_S. A row: her fall, her stumbles from the first key on (`support.stumble`),
her spasms counted (`motion.spasm`), her errand's last phase and whether it docked; no chair offered, 'not offered'.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: The keys held, s: WA taking her over, W before she hands back; the stand after a takeover, s; an errand at most, s.
TAKE_S, WALK_S, STAND_S, LIMIT_S = 1.0, 1.5, 4.0, 40.0
#: Each case's moments, s: into the errand's walk (`taken`), after her keys are let go (`handed`).
ATS = {'taken': (0.5, 1.5), 'handed': (0.0, 0.4)}


def one(x, z, yaw, case, at):
    """A row of `case` from (x, z, `yaw`) at its moment `at`."""
    from types import SimpleNamespace as SN
    from machine.process import introspect, watch, worker
    from terminal.views import gym, scanner, viewpoint
    from tools.sim import house, player
    w = house.Watch(x, z, yaw)
    g, tap, judge, told, keyed = w.g, introspect.Tap(), watch.Watch(), [], []
    st = {'body': SN(send=lambda **cmd: worker.apply(w.s, cmd)), 'scanner': scanner.VisionScanner(),
          'now': worker.told(w.s), 'yaw': 180.0, 'tour': None, **viewpoint.HOME}
    out = {'x': x, 'z': z, 'yaw': yaw, 'case': case, 'at': at}

    def chose():
        offers = gym.offered(st)
        k = [i for i, o in enumerate(offers) if o[1] == 'sit_chair']
        if k:
            gym.chose(st, gym.DIGITS.index(offers[k[0]][0][1:]))
        return bool(k)

    def go(seconds, until=lambda _w: False):
        def judged(_w):
            snap = tap.passed(w.s)
            if snap is not None:
                told.extend(judge.step(snap))
            return until(_w)
        return w.go(seconds, judged)

    def key(name):
        keyed.append(g['t'])
        worker.apply(w.s, {'twist': player.KEYS[name]})

    if case == 'taken':
        if not chose():
            return dict(out, result='not offered')
        if not go(LIMIT_S, lambda _w: (g.get('_errand') or {}).get('phase') == 'walk'):
            return dict(out, result='no walk', phase=(g.get('_errand') or {}).get('phase'))
        go(at)
        key('wa')
        go(TAKE_S)
        key('-')
        go(STAND_S)
    else:
        key('w')
        go(WALK_S)
        key('-')
        go(at)
        if not chose():
            go(STAND_S)
            fell = w.s['fallen_at']
            return dict(out, result='not offered' if fell is None else 'bad', fell=None if fell is None else round(fell - keyed[0], 2))
        go(LIMIT_S, lambda _w: not g.get('_errand'))
    t0 = keyed[0]
    fell = w.s['fallen_at']
    stumbles = [round(e['t'] - t0, 2) for e in told if e['event'] == 'stumble' and e['t'] >= t0]
    docked = any(tag.startswith('done/') for _t, tag, _x, _z in w.phases)
    bad = fell is not None or bool(stumbles)
    return dict(out, fell=None if fell is None else round(fell - t0, 2), stumbles=stumbles[:4],
                spasms=sum(e['event'] == 'spasm' for e in told if e['t'] >= t0), docked=docked,
                phase=(g.get('_errand') or {}).get('phase'), result='bad' if bad else 'ok')


def _relayed(argvs):
    """Each run its own process on the relay's batons (`focus.relay`): their rows."""
    from tools.dev import focus
    jobs = [focus.Job('takeover_%d' % k, [sys.executable, '-X', 'utf8', os.path.abspath(__file__)] + a, 0.4, 900.0, None)
            for k, a in enumerate(argvs)]
    lines = [[ln for ln in (text or '').splitlines() if ln.startswith('{')] for _job, text, _code, _s in focus.relay(jobs)]
    return [dict(json.loads(ln[-1])) if ln else dict(result='crashed') for ln in lines]


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('--one', nargs=5, metavar=('X', 'Z', 'YAW', 'CASE', 'AT'), help='one run, here')
    args = p.parse_args(argv)
    if args.one:
        x, z, yaw, case, at = args.one
        print(json.dumps(one(float(x), float(z), float(yaw), case, float(at))), flush=True)
        return 0
    from tools.sim import house
    rows = _relayed([['--one', str(x), str(z), str(h), case, str(at)]
                     for x, z in house.spots() for h in house.HEADINGS for case, ats in ATS.items() for at in ats])
    sums = {}
    for r in rows:
        key = '%s %s' % (r.get('case', '?'), r.get('result'))
        sums[key] = sums.get(key, 0) + 1
    print('  '.join('%s %d' % kv for kv in sorted(sums.items())), flush=True)
    for r in rows:
        if r.get('result') not in ('ok', 'not offered'):
            print(json.dumps(r), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
