#!/usr/bin/env python3
"""Her get-up's knees under searched on her build as it is, scored on her walking again.

`getup.KNEES_UNDER`'s two keyframes, from the states real falls leave her in.

    python tools/sim/getup_search.py --starts        # the falls run, her lying states kept
    python tools/sim/getup_search.py                 # the get-up as it is, from each
    python tools/sim/getup_search.py --search --generations 16

A start is one of the scoreboard's falls (`gait_montecarlo`'s events, P's shove at SPREAD's
steps) run until her get-up begins: every joint, the pelvis's place and turn (STARTS). A run lays
her there still and lets the director check her and get her up (`machine.down`); its cost:
KEPT_K if she is not walking again by RUN_S, the seconds it took, TRY_K a try past the first.
"""
import argparse
import json
import math
import multiprocessing
import sys
import time

from tools.dev import background
from tools.sim import cmaes

#: The falls a start is taken from: (event, spread step).
FALLS = (('shove', -1), ('shove', 0), ('shove', 1), ('hole', 0), ('lace', 0))

#: A run's seconds; not walking again by then KEPT_K, and TRY_K a try past the first.
RUN_S, KEPT_K, TRY_K = 40.0, 60.0, 10.0

#: The knobs: each of the knees under's two keyframes, its pose's angles, deg, both sides alike
#: (`getup._pose`), and its seconds; searched SPAN either side of where they are.
FIELDS = ('hip', 'knee', 'ankle', 'spine', 'shoulder', 'elbow')
SPAN = {'hip': 35.0, 'knee': 35.0, 'ankle': 25.0, 'spine': 25.0, 'shoulder': 35.0, 'elbow': 35.0}
SECONDS = (0.6, 2.2)


def _now():
    """{knob: value} of the knees under as it is."""
    from machine import getup
    out = {}
    for k, (_verb, _stage, span, pose) in enumerate(getup.KNEES_UNDER):
        out.update({'k%d.%s' % (k, f): pose[('left_' + f) if f not in ('spine',) else f]
                    for f in FIELDS})
        out['k%d.s' % k] = span
    return out


def _apply(values):
    """The knees under set from {knob: value}, the neck as it was."""
    from machine import getup
    was = getup.KNEES_UNDER
    out = []
    for k, (verb, stage, span, pose) in enumerate(was):
        a = [values.get('k%d.%s' % (k, f), pose[('left_' + f) if f != 'spine' else f])
             for f in FIELDS]
        out.append((verb, stage, values.get('k%d.s' % k, span),
                    getup._pose(a[0], a[1], a[2], a[3], pose['neck'], a[4], a[5])))
    getup.KNEES_UNDER = tuple(out)


def _body():
    from machine import Machine
    from machine.director import Director
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    return body, Director(body, 0.85)


def start(fall):
    """(event, step) -> her lying state as her get-up begins: {joint: deg}, place, turn; None if
    she never fell."""
    from machine import events, getup
    from machine.figure import JOINTS
    event, k = fall
    body, d = _body()
    d.walker.start()
    d.stage = 'walk'
    body.loop.step(0.0)
    bus, world = body.loop.bus, body.nodes['pelvis'].world
    laid, was, out = False, 0.0, None
    while bus['t'] < 30.0 and out is None:
        body.loop.write(**d.step(0.001))
        body.loop.step(0.001)
        if not laid and bus['t'] >= 5.0 and was < events.at(event, k) <= d.walker.phase:
            events.lay(event, d, world, k)
            laid = True
        was = d.walker.phase
        if d.stage in getup.STAGES:
            q = world.data.qpos
            out = {'fall': list(fall), 'deg': {j: math.degrees(q[world.qadr[i]])
                                               for i, j in enumerate(JOINTS)},
                   'at': [float(v) for v in q[0:3]], 'turn': [float(v) for v in q[3:7]]}
    body.close()
    return out


def trial(job):
    """(values, start) -> (cost, walked s or None, tries)."""
    from machine import down
    values, lying = job
    _apply(values)
    body, d = _body()
    bus, world = body.loop.bus, body.nodes['pelvis'].world
    world.reset(lying['deg'], where=lying['at'], turn=lying['turn'])
    body.loop.step(0.0)
    d.stage, d.falling_at, d.fallen_at, d.down = 'fallen', bus['t'], bus['t'], down.Down(world)
    walked = None
    while bus['t'] < RUN_S and walked is None and not d.given_up:
        body.loop.write(**d.step(0.001))
        body.loop.step(0.001)
        if d.stage == 'walk':
            walked = bus['t']
    body.close()
    cost = (KEPT_K if walked is None else 0.0) + (walked or RUN_S) + TRY_K * max(0, d.tries - 1)
    return cost, walked, d.tries


def runs(pool, candidates, starts):
    """[(cost, share walking, nan, results)] a candidate, every start of each in one pool."""
    jobs = [(c, s) for c in candidates for s in starts]
    got = pool.map(trial, jobs, chunksize=1)
    out = []
    for k in range(len(candidates)):
        mine = got[k * len(starts):(k + 1) * len(starts)]
        out.append((sum(r[0] for r in mine) / len(mine),
                    sum(r[1] is not None for r in mine) / len(mine), math.nan, mine))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--starts', action='store_true', help='run the falls, keep the states')
    parser.add_argument('--file', default='getup_starts.json', help='the lying states')
    parser.add_argument('--search', action='store_true')
    parser.add_argument('--generations', type=int, default=16)
    parser.add_argument('--population', type=int, default=12)
    parser.add_argument('--sigma', type=float, default=0.15)
    parser.add_argument('--log', default='getup_search.jsonl')
    parser.add_argument('--workers', type=int, default=16)
    args = parser.parse_args(argv)
    background.lower()
    began = time.time()
    with multiprocessing.get_context('spawn').Pool(args.workers) as pool:
        if args.starts:
            got = [s for s in pool.map(start, FALLS, chunksize=1) if s is not None]
            json.dump(got, open(args.file, 'w'))
            print('%d starts of %d falls in %s, %.0f s' % (len(got), len(FALLS), args.file,
                                                           time.time() - began))
            return 0
        starts = json.load(open(args.file))
        now = _now()
        if args.search:
            spans = {n: ((v - SPAN[n.split('.')[1]], v + SPAN[n.split('.')[1]])
                         if not n.endswith('.s') else SECONDS) for n, v in now.items()}
            with open(args.log, 'a') as log:
                cost, values = cmaes.search(pool, spans, args.generations, args.population, log,
                                            lambda p, c: runs(p, c, starts), now.get, args.sigma)
            now = values or now
            print('BEST %.2f %s' % (cost, json.dumps({k: round(v, 2) for k, v in now.items()})))
        cost, held, _nan, results = runs(pool, [now], starts)[0]
        print('cost %.2f, walking again from %d of %d' % (cost, round(held * len(starts)),
                                                          len(starts)))
        for s, (c, walked, tries) in zip(starts, results):
            print('  %-6s %+d  %s, %d tries' % (s['fall'][0], s['fall'][1],
                                                'walking at %.1f s' % walked if walked else 'down',
                                                tries))
    print('%.0f s' % (time.time() - began))
    return 0


if __name__ == '__main__':
    sys.exit(main())
