#!/usr/bin/env python3
"""Where her falls land and how hard, over a spread of falls, a fall a baton on the relay.

    python tools/sim/landings.py where [--bare]      # each segment's landings clustered
    python tools/sim/landings.py gel 0.5:0.02:1.5 0.9:0.02:1.5 [--bare]   # PAD_SOFT:PAD_S:PAD_DAMP

FALLS walked into at 0.8 strides/s, each fall from its start to LANDED_S after she is down. A
segment's landing is its first LANDING_S on the floor: its contacts' points in its frame (the
right side mirrored onto the left) and their impulse, clustered within REACH_M - where the pads
go (`figure.PADS`). A gel (`mjcf.PAD_*`) is judged on the padded segments' worst peak a fall:
its median, its 90th percentile, its spread (90th less 10th) and the worst, the pads' deepest
give, the head's peak. `--bare` takes the pads off.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

FALLS = tuple((event, k) for event in ('lace', 'rug', 'hole', 'stairs') for k in (0, 1, 2))
LANDING_S, LANDED_S, REACH_M = 0.2, 1.5, 0.05
PADDED = ('upper_arm', 'forearm', 'pelvis', 'thigh', 'shank')


def one(event, k, values, bare):
    """A fall's landings: {'fell', 'deep', 'parts': {part: {'impulse', 'peak', 'spots'}}}."""
    import numpy as np
    from machine import Machine, events, figure
    from machine.director import Director
    from machine.modes import DYNAMIC
    from tools.sim import knobs
    knobs.set_(values)
    if bare:
        figure.PADS = ()
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    director = Director(body, 0.8)
    director.cadence = director.walker.cadence = 0.8
    director.walker.start()
    director.stage = 'walk'
    body.loop.step(0.0)
    bus, world = body.loop.bus, body.nodes['pelvis'].world
    m, d = world.model, world.data
    ours = {m.body(s[0]).id: s[0] for s in figure.SEGMENTS}
    parts, touched, deep, f = {}, {}, 0.0, np.zeros(6)
    laid, was, since, down = False, 0.0, None, None
    while bus['t'] < 30.0 and not (down is not None and bus['t'] > down + LANDED_S):
        body.loop.write(**director.step(0.001))
        body.loop.step(0.001)
        if not laid and bus['t'] >= 5.0 and was < events.at(event, k) <= director.walker.phase:
            events.lay(event, director, world, k)
            laid = True
        was = director.walker.phase
        since = since if since is not None or director.stage not in ('catch', 'falling') else 1
        down = bus['t'] if down is None and director.stage == 'fallen' else down
        if since is None:
            continue
        per = {}
        for i in range(d.ncon):
            c = d.contact[i]
            mine = [b for b in (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]) if b in ours]
            part = ours[mine[0]].split('_', 1)[-1] if len(mine) == 1 else 'foot'
            if part in ('foot', 'toes'):
                continue
            name = ours[mine[0]]
            world._mj.mj_contactForce(m, d, i, f)
            per[part] = per.get(part, 0.0) + abs(f[0])
            g = c.geom1 if m.geom_bodyid[c.geom1] == mine[0] else c.geom2
            deep = max(deep, -c.dist) if '_pad' in (m.geom(g).name or '') else deep
            if bus['t'] - touched.setdefault(name, bus['t']) > LANDING_S:
                continue
            at = d.xmat[mine[0]].reshape(3, 3).T @ (c.pos - d.xpos[mine[0]])
            if name.startswith('right_') or (part in ('pelvis', 'torso', 'head') and at[0] < 0):
                at[0] = -at[0]
            seg = parts.setdefault(part, {'impulse': 0.0, 'peak': 0.0, 'spots': {}})
            key = ','.join('%d' % round(v / 0.01) for v in at)
            seg['spots'][key] = seg['spots'].get(key, 0.0) + abs(f[0]) * 0.001
            seg['impulse'] += abs(f[0]) * 0.001
        for part, force in per.items():
            seg = parts.setdefault(part, {'impulse': 0.0, 'peak': 0.0, 'spots': {}})
            seg['peak'] = max(seg['peak'], force)
    body.close()
    return {'fell': down is not None, 'deep': deep, 'parts': parts}


def run(values, bare):
    """[one's result] over FALLS on the relay."""
    from tools.dev import focus
    jobs = [focus.Job('%s%d' % fall, [sys.executable, '-X', 'utf8', os.path.abspath(__file__),
                                       '--one', fall[0], str(fall[1]), json.dumps(values)]
                      + (['--bare'] if bare else []), 1.5, 600.0) for fall in FALLS]
    out = []
    for _job, text, _code, _s in focus.relay(jobs):
        line = next((ln for ln in reversed(text.splitlines()) if ln.startswith('{')), None)
        if line:
            out.append(json.loads(line))
    return [r for r in out if r['fell']]


def clusters(spots):
    """[(centre, share, r80)] of a segment's spots {key: N s}: the heaviest, all within REACH_M of
    it, again, three at most."""
    left = sorted(((tuple(int(v) / 100.0 for v in k.split(',')), j) for k, j in spots.items()
                   if j > 0.0), key=lambda s: -s[1])
    total, out = sum(j for _p, j in left) or 1.0, []
    while left and len(out) < 3:
        near = [s for s in left if math.dist(s[0], left[0][0]) <= REACH_M]
        j = sum(w for _p, w in near)
        c = tuple(sum(p[i] * w for p, w in near) / j for i in range(3))
        acc, r80 = 0.0, 0.0
        for r, w in sorted((math.dist(p, c), w) for p, w in near):
            acc, r80 = acc + w, r
            if acc >= 0.8 * j:
                break
        out.append((c, j / total, r80))
        left = [s for s in left if s not in near]
    return out


def where(results):
    """Each segment's landings, the heaviest first."""
    print('%d falls' % len(results))
    names = {p for r in results for p in r['parts']}
    for part in sorted(names, key=lambda p: -sum(r['parts'].get(p, {}).get('impulse', 0.0)
                                                 for r in results)):
        hit = [r['parts'][part] for r in results if part in r['parts']]
        spots = {}
        for h in hit:
            for key, j in h['spots'].items():
                spots[key] = spots.get(key, 0.0) + j
        print('%-10s in %2d falls  %6.1f N s  peak %4.1f kN' % (
            part, len(hit), sum(h['impulse'] for h in hit), max(h['peak'] for h in hit) / 1e3))
        for c, share, r80 in clusters(spots):
            print('    at (%+.2f %+.2f %+.2f) %3.0f %%, 80 %% within %.3f m' % (c + (100 * share, r80)))


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * (len(v) - 1) + 0.5))]


def gel(results, name):
    """A gel's line: the padded segments' worst peak a fall, kN; the deepest give, mm; the head."""
    peaks = [max((r['parts'][p]['peak'] for p in PADDED if p in r['parts']), default=0.0) / 1e3
             for r in results]
    heads = [r['parts'].get('head', {}).get('peak', 0.0) / 1e3 for r in results]
    print('%-18s %2d falls  median %5.2f  p90 %5.2f  spread %5.2f  worst %5.2f kN  give %4.1f mm  '
          'head %4.2f kN' % (name, len(results), q(peaks, 0.5), q(peaks, 0.9),
                              q(peaks, 0.9) - q(peaks, 0.1), max(peaks),
                              max(r['deep'] for r in results) * 1e3, max(heads)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('what', choices=('where', 'gel'), nargs='?')
    parser.add_argument('gels', nargs='*', metavar='SOFT:S:DAMP')
    parser.add_argument('--bare', action='store_true', help='the pads taken off')
    parser.add_argument('--one', nargs=3, metavar=('EVENT', 'K', 'VALUES'), help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.one:
        print(json.dumps(one(args.one[0], int(args.one[1]), json.loads(args.one[2]), args.bare)))
        return 0
    if args.what == 'where':
        where(run({}, args.bare))
        return 0
    for spec in args.gels or [None]:
        values = {} if spec is None else dict(zip(('PAD_SOFT', 'PAD_S', 'PAD_DAMP'),
                                                  map(float, spec.split(':'))))
        gel(run(values, args.bare), 'bare' if args.bare else spec or 'as built')
    return 0


if __name__ == '__main__':
    sys.exit(main())
