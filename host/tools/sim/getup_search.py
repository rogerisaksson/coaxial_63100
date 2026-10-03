#!/usr/bin/env python3
"""Her get-up's keyframes searched on her build as it is, scored on her walking again.

`getup.KNEES_UNDER`'s two keyframes, or `getup.ONTO_FEET`'s first three (TABLES), from the states
real falls leave her in.

    python tools/sim/getup_search.py --starts        # the falls run, her lying states kept
    python tools/sim/getup_search.py                 # the get-up as it is, from each
    python tools/sim/getup_search.py --search --generations 16
    python tools/sim/getup_search.py --search --table f   # onto her feet's

A start is one of the scoreboard's falls (`gait_montecarlo`'s events, P's shove at SPREAD's
steps) run until her get-up begins: every joint, the pelvis's place and turn (STARTS). A run lays
her there still and lets the director check her and get her up (`machine.down`); its cost:
KEPT_K if she is not walking again by RUN_S, the seconds it took, TRY_K a try past the first.
"""
import argparse
import json
import math
import sys
import time

from tools.dev import background
from tools.sim import cmaes

#: The falls a start is taken from: (event, spread step).
FALLS = (('shove', -1), ('shove', 0), ('shove', 1), ('hole', 0), ('lace', 0))

#: A run's seconds; not walking again by then KEPT_K, and TRY_K a try past the first.
RUN_S, KEPT_K, TRY_K = 40.0, 60.0, 10.0

#: The knobs, a table a letter: its keyframes from the first, how many, the fields of each pose,
#: deg, both sides alike (`getup._pose`, the toes' `foot`), and its seconds' span; each field
#: searched SPAN either side of where it is.
TABLES = {'k': ('KNEES_UNDER', 2, ('hip', 'knee', 'ankle', 'spine', 'shoulder', 'elbow'),
                (0.6, 2.2)),
          'f': ('ONTO_FEET', 3, ('hip', 'knee', 'ankle', 'spine', 'shoulder', 'elbow', 'foot'),
                (0.3, 2.5))}
SPAN = {'hip': 35.0, 'knee': 35.0, 'ankle': 25.0, 'spine': 25.0, 'shoulder': 35.0, 'elbow': 35.0,
        'foot': 30.0}


def _key(field):
    return field if field == 'spine' else 'left_' + field


def _now(tables='k'):
    """{knob: value} of `tables`' keyframes as they are."""
    from machine import getup
    out = {}
    for t in tables:
        name, n, fields, _s = TABLES[t]
        for k, (_verb, _stage, span, pose) in enumerate(getattr(getup, name)[:n]):
            out.update({'%s%d.%s' % (t, k, f): pose[_key(f)] for f in fields})
            out['%s%d.s' % (t, k)] = span
    return out


def _apply(values):
    """The tables' keyframes set from {knob: value}, the rest of each pose as it was."""
    from machine import getup
    for t, (name, n, fields, _s) in TABLES.items():
        out = []
        for k, (verb, stage, span, pose) in enumerate(getattr(getup, name)):
            pose = dict(pose)
            for f in fields if k < n else ():
                v = values.get('%s%d.%s' % (t, k, f))
                if v is not None:
                    pose.update({f: v} if f == 'spine' else {'left_' + f: v, 'right_' + f: v})
            out.append((verb, stage, values.get('%s%d.s' % (t, k), span), pose))
        setattr(getup, name, tuple(out))


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


def relayed(args):
    """[the JSON each printed] of `args` lists run as jobs on the relay, this file's `--one`
    modes (`focus.relay`: a baton a physical core, never a pool of its own - the user,
    2026-10-03), in their order; a run lost None."""
    import os
    from tools.dev import focus
    jobs = [focus.Job(str(k), [sys.executable, '-X', 'utf8', os.path.abspath(__file__)] + a,
                      RUN_GB, 900.0) for k, a in enumerate(args)]
    got = {}
    for job, text, _code, _s in focus.relay(jobs):
        line = next((ln for ln in reversed(text.splitlines()) if ln.startswith('{')), None)
        got[job.name] = json.loads(line)['result'] if line else None
    return [got[str(k)] for k in range(len(args))]


def runs(_pool, candidates, starts, file='getup_starts.json'):
    """[(cost, share walking, nan, results)] a candidate, every start of each a job on the
    relay (`--one`, the starts from `file`)."""
    got = relayed([['--file', file, '--one', json.dumps(c), str(s)]
                   for c in candidates for s in range(len(starts))])
    out = []
    for k in range(len(candidates)):
        mine = [r or (KEPT_K + RUN_S, None, 1) for r in got[k * len(starts):(k + 1) * len(starts)]]
        out.append((sum(r[0] for r in mine) / len(mine),
                    sum(r[1] is not None for r in mine) / len(mine), math.nan, mine))
    return out


#: A run's commit on the relay, GB (a world and its five buses).
RUN_GB = 1.2


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--starts', action='store_true', help='run the falls, keep the states')
    parser.add_argument('--file', default='getup_starts.json', help='the lying states')
    parser.add_argument('--search', action='store_true')
    parser.add_argument('--generations', type=int, default=16)
    parser.add_argument('--population', type=int, default=12)
    parser.add_argument('--sigma', type=float, default=0.15)
    parser.add_argument('--log', default='getup_search.jsonl')
    parser.add_argument('--table', default='k', help='the tables searched: k, f or kf')
    parser.add_argument('--one', nargs=2, metavar=('VALUES', 'K'),
                        help="one run on the relay: a candidate's JSON and its start's index")
    parser.add_argument('--one-start', nargs=2, metavar=('EVENT', 'K'),
                        help='one fall on the relay: its event and spread step')
    args = parser.parse_args(argv)
    background.lower()
    if args.one:
        lying = json.load(open(args.file))[int(args.one[1])]
        print(json.dumps({'result': trial((json.loads(args.one[0]), lying))}))
        return 0
    if args.one_start:
        print(json.dumps({'result': start((args.one_start[0], int(args.one_start[1])))}))
        return 0
    began = time.time()
    if args.starts:
        got = [s for s in relayed([['--one-start', e, str(k)] for e, k in FALLS]) if s]
        json.dump(got, open(args.file, 'w'))
        print('%d starts of %d falls in %s, %.0f s' % (len(got), len(FALLS), args.file,
                                                       time.time() - began))
        return 0
    starts = json.load(open(args.file))
    now = _now(args.table)
    if args.search:
        spans = {n: ((v - SPAN[n.split('.')[1]], v + SPAN[n.split('.')[1]])
                     if not n.endswith('.s') else TABLES[n[0]][3]) for n, v in now.items()}
        with open(args.log, 'a') as log:
            cost, values = cmaes.search(None, spans, args.generations, args.population, log,
                                        lambda p, c: runs(p, c, starts, args.file), now.get,
                                        args.sigma)
        now = values or now
        print('BEST %.2f %s' % (cost, json.dumps({k: round(v, 2) for k, v in now.items()})))
    cost, held, _nan, results = runs(None, [now], starts, args.file)[0]
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
