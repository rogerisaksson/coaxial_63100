#!/usr/bin/env python3
"""Monte Carlo over the gynoid's walk: each candidate through the same trials, one process a core.

A candidate is a few module constants of the walk (`machine.walker`, `machine.gait`,
`machine.arrival`). Its trials, the same for every candidate, each run by the director as the
terminal page runs her:

- rise: landed in the squat, up and walking to a pace asked of her,
- walk: from mid-stride at a pace, the pendulum between her ears read (`machine.pendulum`),
- event: from mid-stride, the floor's event under her next left step (`physics.World.terrain`):
  a hole, a sill, a slip patch, a loose rug; or the left knee's drive glitched in its stance
  (`physics.World.glitch`): its gate dropped for a moment, or derated hot for seconds; each
  laid at a spread of places (SPREAD), the trial held their mean.

Two numbers: `held`, the share of the trials' time she stood, and `stir`, the pendulum's mean
over the walks, mm. Its cost is one: `stir + LOST (1 - held)`. A single run a candidate scores
chance - the rise flips on 0.5 % of any knob (docs/FINDINGS.md, 2026-09-26) - a spread of them
scores the walk.

    python tools/sim/gait_montecarlo.py                                  # the walk as it is
    python tools/sim/gait_montecarlo.py --grid SURGE_DEG=0,1,2 SWAY_K=0,0.5,1
    python tools/sim/gait_montecarlo.py --search SURGE_DEG=0:4 SWAY_K=0:2 --generations 12
"""
import argparse
import itertools
import json
import math
import multiprocessing
import sys
import time

#: (kind, pace, event): the trials. The floor's events took the shoves' place (a shove hardly
#: ever happens to a walker; a hole, a sill, a rug, a slippery patch, a lace and a drive's
#: glitch do), each laid as `machine.events` lays it the first time the left leg's phase
#: crosses its `events.at` after EVENT_AT_S. Laid on the clock the event met whatever phase a
#: candidate's pace had brought her to, and a 0.1 % change of any knob flipped a shove. The hip
#: held to a quarter for a second changed nothing: a stance hip asks under 60 N m (2026-09-27).
TRIALS = (('rise', 0.6, None), ('rise', 0.75, None), ('rise', 0.9, None),
          ('walk', 0.65, None), ('walk', 0.85, None), ('walk', 0.9, None),
          ('event', 0.85, 'hole'), ('event', 0.85, 'sill'), ('event', 0.85, 'slip'),
          ('event', 0.85, 'rug'), ('event', 0.65, 'sill'), ('event', 0.9, 'slip'),
          ('event', 0.85, 'soa'), ('event', 0.85, 'hot'), ('event', 0.85, 'lace'))

#: Each event laid at SPREAD steps (`events.STEP_M`, `events.GLITCH_STEP`). Laid at one place,
#: 2 % of an arm's swing flipped a slip or the hot knee and the held share ran 75-90 %
#: (2026-09-28).
SPREAD = (-1, 0, 1)

#: (trial, spread step): every run a candidate makes.
JOBS = [(t, k) for t in TRIALS for k in (SPREAD if t[0] == 'event' else (0,))]

#: A trial's seconds, by kind; a walk's stir is meaned from SETTLE_S; events laid from
#: EVENT_AT_S.
SECONDS = {'rise': 20.0, 'walk': 14.0, 'event': 24.0}
SETTLE_S, EVENT_AT_S = 4.0, 5.0

#: The cost of the trials' time lost, mm of stir for all of it; a walk fallen counts this stir.
LOST, FALLEN_STIR = 30.0, 10.0

MODULES = ('walker', 'gait', 'arrival', 'director', 'capture', 'physics', 'buses', 'events')


def _set(values):
    """The constants set where they live, the plan's tables cleared."""
    import importlib
    mods = [importlib.import_module('machine.' + m) for m in MODULES]
    for name, value in values.items():
        owner = next((m for m in mods if hasattr(m, name)), None)
        if owner is None:
            raise KeyError('no %s in machine.%s' % (name, ', machine.'.join(MODULES)))
        setattr(owner, name, value)
    gait, walker = mods[1], mods[0]
    gait._FITS.clear()
    walker._TABLES.clear()


def trial(job):
    """(held, stir or None, what happened) for one candidate's one run (`JOBS`)."""
    values, ((kind, pace, event), k) = job
    _set(values)
    from machine import Machine, events
    from machine.director import Director
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    director = Director(body, pace)
    if kind == 'rise':
        director.begin()
    else:
        director.cadence = pace
        director.walker.cadence = pace
        director.walker.start()
        director.stage = 'walk'
    body.loop.step(0.0)
    bus, world = body.loop.bus, body.nodes['pelvis'].world
    seconds = SECONDS[kind]
    stirred, passes, laid, was, tilt = 0.0, 0, False, 0.0, 0.0
    fell, up, down = None, None, 0.0
    while bus['t'] < seconds:
        body.loop.write(**director.step(0.001))
        body.loop.step(0.001)
        if director.stage in ('falling', 'fallen') and fell is None:
            fell, up = bus['t'], None
        if fell is not None and up is None:
            down += 0.001
            if director.stage == 'walk':
                up = bus['t']
        if (kind == 'event' and not laid and bus['t'] >= EVENT_AT_S
                and was < events.at(event, k) <= director.walker.phase):
            events.lay(event, director, world, k)
            laid = True
        was = director.walker.phase
        if laid:
            tilt = max(tilt, math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (
                bus['pelvis.pose.qx'] ** 2 + bus['pelvis.pose.qz'] ** 2))))))
        if kind == 'walk' and bus['t'] >= SETTLE_S:
            stirred += director.pendulum.energy
            passes += 1
    what = '%.1f m' % bus['pelvis.pose.z'] + (', tipped %.0f deg' % tilt if laid else '')
    if fell is not None:
        what = 'fell at %.1f s' % fell + (', up at %.1f s' % up if up else ', down') + ', ' + what
    return 1.0 - down / seconds, (stirred / passes if passes else None), what


def by_trial(results):
    """[(held, stir or None, [what])] a trial each, in TRIALS' order, from its runs' results
    in JOBS' order: the held share its spread's mean."""
    out = []
    for t in TRIALS:
        mine = [r for (u, _k), r in zip(JOBS, results) if u == t]
        out.append((sum(h for h, _s, _w in mine) / len(mine), mine[0][1], [w for _h, _s, w in mine]))
    return out


def score(results):
    """(cost, held, stir) of one candidate's run results, in JOBS' order."""
    trials = by_trial(results)
    held = sum(h for h, _s, _w in trials) / len(trials)
    walks = [(s if s is not None else FALLEN_STIR) for (kind, _p, _e), (_h, s, _w)
             in zip(TRIALS, trials) if kind == 'walk']
    stir = sum(walks) / len(walks)
    return stir + LOST * (1.0 - held), held, stir


def run(pool, candidates):
    """[(cost, held, stir, results)] for `candidates`, every run of each in one pool."""
    jobs = [(values, j) for values in candidates for j in JOBS]
    got = pool.map(trial, jobs, chunksize=1)
    out = []
    for k in range(len(candidates)):
        results = got[k * len(JOBS):(k + 1) * len(JOBS)]
        out.append(score(results) + (results,))
    return out


def _show(values, cost, held, stir, results: list | tuple = ()):
    print('%-40s cost %6.2f  held %5.1f %%  stir %5.2f mm' % (
        ' '.join('%s=%g' % kv for kv in values.items()) or 'as it is', cost, 100 * held, stir))
    trials = by_trial(results) if results else []
    for (kind, pace, event), (h, s, whats) in zip(TRIALS, trials):
        print('    %-5s %.2f %-5s %5.1f %%  %s%s' % (
            kind, pace, event or '', 100 * h, ' | '.join(whats),
            '' if s is None else '  stir %.2f mm' % s))


def search(pool, spans, generations, lam, log):
    """CMA-ES over `spans` {name: (low, high)}, each scaled to its span, from its middle."""
    import numpy as np
    names = list(spans)
    dim = len(names)
    mean, sigma = np.full(dim, 0.5), 0.25
    mu = lam // 2
    weights = math.log(mu + 0.5) - np.log(np.arange(1, mu + 1))
    weights /= weights.sum()
    mueff = 1.0 / (weights ** 2).sum()
    cc, cs = (4 + mueff / dim) / (dim + 4 + 2 * mueff / dim), (mueff + 2) / (dim + mueff + 5)
    c1 = 2 / ((dim + 1.3) ** 2 + mueff)
    cmu = min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((dim + 2) ** 2 + mueff))
    damps = 1 + 2 * max(0.0, math.sqrt((mueff - 1) / (dim + 1)) - 1) + cs
    chi = math.sqrt(dim) * (1 - 1 / (4 * dim) + 1 / (21 * dim * dim))
    pc, ps, cov = np.zeros(dim), np.zeros(dim), np.eye(dim)
    rng = np.random.default_rng(7)
    best = (math.inf, None)
    for gen in range(generations):
        began = time.time()
        vals, vecs = np.linalg.eigh(cov)
        root = vecs @ np.diag(np.sqrt(np.maximum(vals, 1e-20)))
        xs = np.clip(mean + sigma * rng.standard_normal((lam, dim)) @ root.T, 0.0, 1.0)
        cands = [{n: float(spans[n][0] + x[i] * (spans[n][1] - spans[n][0]))
                  for i, n in enumerate(names)} for x in xs]
        got = run(pool, cands)
        costs = np.array([g[0] for g in got])
        for values, (cost, held, stir, _r) in zip(cands, got):
            log.write(json.dumps({'gen': gen, 'values': values, 'cost': cost, 'held': held,
                                  'stir': stir}) + '\n')
            if cost < best[0]:
                best = (cost, values)
        log.flush()
        order = np.argsort(costs)
        old = mean
        mean = weights @ xs[order[:mu]]
        step = (mean - old) / sigma
        inv = vecs @ np.diag(1 / np.sqrt(np.maximum(vals, 1e-20))) @ vecs.T
        ps = (1 - cs) * ps + math.sqrt(cs * (2 - cs) * mueff) * inv @ step
        hsig = (np.linalg.norm(ps) / math.sqrt(1 - (1 - cs) ** (2 * (gen + 1))) / chi
                < 1.4 + 2 / (dim + 1))
        pc = (1 - cc) * pc + hsig * math.sqrt(cc * (2 - cc) * mueff) * step
        art = (xs[order[:mu]] - old) / sigma
        cov = ((1 - c1 - cmu) * cov + c1 * (np.outer(pc, pc) + (1 - hsig) * cc * (2 - cc) * cov)
               + cmu * art.T @ np.diag(weights) @ art)
        sigma *= math.exp((cs / damps) * (np.linalg.norm(ps) / chi - 1))
        print('gen %2d  best %.2f  median %.2f  sigma %.3f  %.0f s  | best so far %.2f %s' % (
            gen, costs.min(), float(np.median(costs)), sigma, time.time() - began, best[0],
            {k: round(v, 3) for k, v in (best[1] or {}).items()}), flush=True)
    return best


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--set', nargs='*', default=[], metavar='NAME=V',
                        help='constants for every candidate')
    parser.add_argument('--grid', nargs='*', default=[], metavar='NAME=V,V,..',
                        help='each combination a candidate')
    parser.add_argument('--search', nargs='*', default=[], metavar='NAME=LOW:HIGH',
                        help='CMA-ES over these spans')
    parser.add_argument('--generations', type=int, default=12)
    parser.add_argument('--population', type=int, default=12)
    parser.add_argument('--log', default='gait_montecarlo.jsonl', help='every candidate, a line')
    parser.add_argument('--workers', type=int, default=16)
    args = parser.parse_args(argv)
    fixed = {k: float(v) for k, v in (a.split('=') for a in args.set)}
    began = time.time()
    with multiprocessing.get_context('spawn').Pool(args.workers) as pool, \
            open(args.log, 'a') as log:
        if args.search:
            spans = {k: tuple(float(x) for x in v.split(':'))
                     for k, v in (a.split('=') for a in args.search)}
            cost, values = search(pool, spans, args.generations, args.population, log)
            print('BEST %.2f %s' % (cost, json.dumps(values)))
            cands = [dict(fixed, **(values or {}))]
        elif args.grid:
            axes = [(k, [float(x) for x in v.split(',')])
                    for k, v in (a.split('=') for a in args.grid)]
            cands = [dict(fixed, **dict(zip([k for k, _v in axes], combo)))
                     for combo in itertools.product(*[v for _k, v in axes])]
        else:
            cands = [fixed]
        got = run(pool, cands)
        ranked = sorted(zip(cands, got), key=lambda cg: cg[1][0])
        for values, (cost, held, stir, results) in ranked:
            _show(values, cost, held, stir, results if len(ranked) == 1 else ())
    print('%d candidates, %d runs, %.0f s' % (len(cands), len(cands) * len(JOBS),
                                              time.time() - began))
    return 0


if __name__ == '__main__':
    sys.exit(main())
