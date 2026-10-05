#!/usr/bin/env python3
"""Monte Carlo over the gynoid's walk: each candidate through the same trials, one process a core.

A candidate is a few module constants of the walk (`machine.walker`, `machine.gait`,
`machine.arrival`). Its trials, the same for every candidate, each run by the director as the
terminal page runs her:

- rise: landed in the squat from a spread of drops (RISE_DROP_M), up and walking to a pace
  asked of her,
- walk: from mid-stride at a pace, the pendulum between her ears read (`machine.pendulum`),
- event: from mid-stride, the floor's event under her next left step (`physics.World.terrain`):
  a hole, a sill, a slip patch, a loose rug; the left knee's drive glitched in its stance
  (`physics.World.glitch`): its gate dropped for a moment, or derated hot for seconds; a nudge
  from her side (`machine.events`); each laid at SPREAD places, the trial held their mean,
- fall: a shove past saving (`machine.events`).

SUITES (`--suite`): look, the rises and the walks on fantasy boards, SOA never binding
(`physics.ENVELOPE` 0); walk; faults, the events and the falls on the boards as built; stand; all.
`held`, the share of the trials' time she stood; `stir`, the pendulum's mean over the
walks, mm. The cost (2026-10-05, the user: a fall punished is met by a crouch): a walk off its
form (`looks.FORM`, its spread's) is rejected; on it, its look and power (`looks.cost`, `stir`
with it); an event, a rise and a stand each a reward, PARRY_K the share of its runs she ended
on her feet - a fall earns nothing and costs nothing; a fall past saving on its landing's
peak, body and head. A single run scores chance - the rise flips on 0.5 % of any knob
(docs/findings/walk.md, 2026-09-26) - a spread of them scores it.

    python tools/sim/gait_montecarlo.py                      # as built
    python tools/sim/gait_montecarlo.py --search SURGE_DEG=0:4 SWAY_K=0:2 --generations 12
"""
import argparse
import itertools
import json
import math
import os
import sys
import time

from machine import events
from tools.dev import background
from tools.sim import cmaes, knobs
from tools.sim import looks as form
from tools.sim.looks import LOOKS, cost as look_of, shown

#: (kind, pace, event): the trials. The floor's events took the shoves' place (a shove hardly
#: ever happens to a walker; a hole, a sill, a rug, a slippery patch, a lace and a drive's
#: glitch do), each laid as `machine.events` lays it the first time the left leg's phase
#: crosses its `events.at` after EVENT_AT_S. Laid on the clock the event met whatever phase a
#: candidate's pace had brought her to, and a 0.1 % change of any knob flipped a shove. The hip
#: held to a quarter for a second changed nothing: a stance hip asks under 60 N m (2026-09-27).
TRIALS = (('rise', 0.6, None), ('rise', 0.75, None), ('rise', 0.9, None),
          ('walk', 0.65, None), ('walk', 0.85, None), ('walk', 0.9, None), ('walk', 1.0, None),
          ('event', 0.85, 'hole'), ('event', 0.85, 'sill'), ('event', 0.85, 'slip'),
          ('event', 0.85, 'rug'), ('event', 0.65, 'sill'), ('event', 0.9, 'slip'),
          ('event', 0.85, 'soa'), ('event', 0.85, 'hot'), ('event', 0.85, 'lace'),
          ('event', 0.85, 'nudge'), ('fall', 0.85, 'shove')) + tuple(
              ('stand', 0.85, e) for e in events.STANDING)

#: Each event laid at SPREAD steps (`events.STEP_M`, `events.GLITCH_STEP`). Laid at one place,
#: 2 % of an arm's swing flipped a slip or the hot knee and the held share ran 75-90 %
#: (2026-09-28).
SPREAD = (-1, 0, 1)

#: Each walk run at SPREAD steps of WALK_SPREAD of its pace, its costs meaned: one run a candidate
#: scored chance - 15.85 in a search, 38.97 run again rounded to four figures (2026-09-28).
WALK_SPREAD = 0.02

#: The suites: which kinds of trial each runs.
SUITES = {'all': ('rise', 'walk', 'event', 'fall', 'stand'), 'look': ('rise', 'walk'),
          'walk': ('walk',), 'rise': ('rise',), 'faults': ('event', 'fall'), 'stand': ('stand',)}

#: (trial, spread step): every run a candidate makes (`suite` narrows them).
JOBS = [(t, k) for t in TRIALS for k in SPREAD]

#: Each rise landed in the squat from over its 2 mm, RISE_DROP_M more a SPREAD step and a third of
#: it a rise: she starts at the walk's own cadence whatever is asked (`director.PACE_RATE`), and
#: her three rises were one start - all three fell at 6.4 s (2026-10-02).
RISE_DROP_M = 0.002

#: A walk is looked at LOOK_HZ. A touch: the sole bearing TOUCH_N after QUIET_S of none, read
#: every millisecond, its peak taken over IMPACT_S (the cost of each: `tools.sim.looks`).
LOOK_HZ, IMPACT_S, TOUCH_N, QUIET_S = 50.0, 0.03, 30.0, 0.1

#: An event parried - she on her feet to its end -, PARRY_K the share of its runs: one more of
#: 30 is 10; until 2026-10-05 a fall's price, FALL_K, the same 300 - every cost since is 300 a
#: kind of trial run under those before. A
#: fall past saving, its landing over LAND_S from the fall: LAND_K a kN of the peak her body bears
#: on the floor, feet aside, HEAD_K a kN of her head's; GEAR_K a share past its rating of the
#: worst gearbox's peak torque (`World.geared`, `drives.shock`) - broken.
PARRY_K, LAND_S, LAND_K, HEAD_K, GEAR_K = 300.0, 1.5, 2.0, 10.0, 100.0

#: A stand's cost: its stir, mm, from its event on, TREAD_K a step taken; stood, PARRY_K.
TREAD_K = 1.0

#: Every trial and job, `suite()` narrowing from them, not itself (2026-10-04).
ALL_TRIALS, ALL_JOBS = TRIALS, list(JOBS)


def suite(name):
    """TRIALS and JOBS: suite `name`'s kinds."""
    global TRIALS, JOBS, SUITE
    SUITE = name
    TRIALS = tuple(t for t in ALL_TRIALS if t[0] in SUITES[name])
    JOBS = [(t, k) for t, k in ALL_JOBS if t[0] in SUITES[name]]


#: The suite the runs are on; a run's commit on the relay, GB (a world and its five buses).
SUITE, RUN_GB = 'all', 1.2


#: A trial's seconds, by kind; a walk's stir is meaned from SETTLE_S; events laid from
#: EVENT_AT_S.
SECONDS = {'rise': 20.0, 'walk': 14.0, 'event': 24.0, 'fall': 24.0, 'stand': 12.0}
SETTLE_S, EVENT_AT_S = 4.0, 5.0

#: A walk's run is judged on the strides it walked: its reach behind and its landing measured. A
#: walk trial none of whose runs walked costs LOST_K, its share of the walks: unjudged, falling at
#: 1.0 strides/s ranked better than walking it (2026-09-30).
JUDGED = ('thigh behind at lift', 'impact')
LOST_K = 30.0



def trial(job):
    """(held, stir or None, what happened, {look: value}) for one candidate's one run
    (`JOBS`): a rise or a walk on fantasy boards, an event on the boards as built."""
    values, ((kind, pace, event), k) = job
    faulted = kind in ('event', 'fall', 'stand')
    knobs.set_(dict(values, ENVELOPE=1.0 if faulted else 0.0))
    from tools.sim import look, strides
    from machine import Machine, drives, events, figure, heat, skeleton
    from machine.director import Director
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    director = Director(body, pace, stand_s=math.inf if kind == 'stand' else 0.0)
    if kind == 'rise':
        nth = [t for t in TRIALS if t[0] == 'rise'].index((kind, pace, event))
        director.begin(drop=0.002 + RISE_DROP_M * (k + 1 + nth / 3.0))
    elif kind == 'stand':
        up, stagger = events.rigged(event)
        director.begin(drop=0.002 + RISE_DROP_M * (k + 1), up=up, stagger=stagger)
        events.rig(event, director, body.nodes['pelvis'].world)
    else:
        # Standing, through the arrival's lean, as the page starts her: started dead through
        # the walker, the first stride fell at 0.85 on sprung toes (2026-10-04).
        director.begin(drop=0.002 + RISE_DROP_M * (k + 1), stage='stand')
        director.cadence = director.walker.cadence = pace * (1.0 + WALK_SPREAD * k)
    body.loop.step(0.0)
    bus, world = body.loop.bus, body.nodes['pelvis'].world
    seconds = SECONDS[kind]
    stirred, passes, laid, was, tilt = 0.0, 0, False, 0.0, 0.0
    fell, up, down, rows, looked = None, None, 0.0, [], -1.0
    quiet, window, impacts, touches, rates, was_load = 0.0, None, [], [], [], 0.0
    foot, moving = world.model.body('left_foot').id, world._np.zeros(6)
    pinned, drive_ms, peak = 0, 0, world.peak[world.driven] * 0.999
    np, m, d = world._np, world.model, world.data
    ours = {m.body(seg[0]).id: seg[0] for seg in figure.SEGMENTS}
    # Her skeleton's parts, welded bodies of their own where `physics.SKELETON` collides them:
    # a drum, a board or a tube the floor meets is what a fall breaks (the armour, the user,
    # 2026-10-04).
    bare_parts = {i for i in skeleton.owners(m) if i not in ours}
    force, drawn, landing, head, bare = np.zeros(6), 0.0, 0.0, 0.0, 0.0
    while bus['t'] < seconds:
        body.loop.write(**director.step(0.001))
        body.loop.step(0.001)
        if director.stage in ('falling', 'fallen') and fell is None:
            fell, up = bus['t'], None
        if fell is not None and up is None:
            down += 0.001
            if director.stage == 'walk':
                up = bus['t']
        if kind == 'stand' and not laid and bus['t'] >= EVENT_AT_S:
            events.befall(event, director, world, k)
            laid = True
        elif (faulted and not laid and bus['t'] >= EVENT_AT_S
              and was < events.at(event, k) <= director.walker.phase):
            events.lay(event, director, world, k)
            laid = True
        was = director.walker.phase
        if laid:
            tilt = max(tilt, math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (
                bus['pelvis.pose.qx'] ** 2 + bus['pelvis.pose.qz'] ** 2))))))
        if kind == 'stand' and laid:
            stirred += director.pendulum.energy
            passes += 1
        if kind == 'walk' and bus['t'] >= SETTLE_S:
            stirred += director.pendulum.energy
            passes += 1
            pinned += int((abs(world.data.ctrl[world.driven]) >= peak).sum())
            drive_ms += len(peak)
            drawn += world.drawn()
            if bus['t'] - looked >= 1.0 / LOOK_HZ:
                looked = bus['t']
                rows.append(look.sample(bus, director, world))
            load = bus['pelvis.pose.left_load']
            if window is not None:
                impacts[-1] = max(impacts[-1], load)
                rates[-1] = max(rates[-1], load - was_load)
                window = window - 0.001 if window > 0.001 else None
            elif load >= TOUCH_N and quiet >= QUIET_S:
                impacts.append(load)
                rates.append(load - was_load)
                world._mj.mj_objectVelocity(world.model, world.data, world._mj.mjtObj.mjOBJ_BODY,
                                            foot, moving, 0)
                touches.append(max(0.0, -float(moving[4])))
                window = IMPACT_S
            quiet, was_load = (quiet + 0.001 if load < TOUCH_N else 0.0), load
        if kind == 'fall' and fell is not None and bus['t'] <= fell + LAND_S:
            body_n = head_n = bare_n = 0.0
            for i in range(d.ncon):
                pair = (m.geom_bodyid[d.contact[i].geom1], m.geom_bodyid[d.contact[i].geom2])
                mine = [b for b in pair if b in ours]
                if len(mine) == 1 and not ours[mine[0]].endswith(('foot', 'toes')):
                    world._mj.mj_contactForce(m, d, i, force)
                    body_n += abs(force[0])
                    head_n += abs(force[0]) if ours[mine[0]] == 'head' else 0.0
                elif not mine and sum(b in bare_parts for b in pair) == 1:
                    world._mj.mj_contactForce(m, d, i, force)
                    bare_n += abs(force[0])
            landing, head = max(landing, body_n / 1e3), max(head, head_n / 1e3)
            bare = max(bare, bare_n / 1e3)
    # A pool's worker lives on: its world's buses and block closed here, not at its exit - left,
    # five bus processes a run piled up to 865 and the host ran out of memory (2026-09-28).
    body.close()
    what = '%.1f m' % bus['pelvis.pose.z'] + (', tipped %.0f deg' % tilt if laid else '')
    if fell is not None:
        what = 'fell at %.1f s' % fell + (', up at %.1f s' % up if up else ', down') + ', ' + what
    measured = strides.measured(rows) if len(rows) > 2 else {}
    walked = {name: v for name, v in measured.items()
              if name in LOOKS or name == 'energy' or any(name == f[0] for f in form.FORM)}
    if impacts:
        walked.update(impact=sum(impacts) / len(impacts), touch=sum(touches) / len(touches),
                      rate=sum(rates) / len(rates))
    if drive_ms:
        walked['load'] = 100.0 * pinned / drive_ms
        walked['power'] = drawn / passes
    walked['fell'] = float(fell is not None)
    if kind == 'stand':
        walked.update(treads=director.treads, stir=stirred / passes if passes else math.nan)
        what += ', %d treads' % director.treads
    if faulted:
        walked.update(landing=landing, head=head, bare=bare,
                      gear=float(np.max(world.geared / np.array([drives.shock(j)
                                                                  for j in figure.JOINTS]))))
    return 1.0 - down / seconds, (stirred / passes if passes else None), what, walked


def by_trial(results):
    """[(held, stir, [what], {look: value}, look's cost, off its form)] a trial each, in TRIALS'
    order, from its runs' results in JOBS' order: the held share its spread's mean, the stir and
    the look's cost its judged runs' (`JUDGED`), nan with none; the measures shown the first
    run's; a walk's form its spread's (`looks.broken`), [(measure, value, bound)]."""
    out = []
    for t in TRIALS:
        mine = [r for (u, _k), r in zip(JOBS, results) if u == t]
        judged = [r for r in mine if r[1] is not None and (t[0] != 'walk' or all(
            r[3].get(n, math.nan) == r[3].get(n, math.nan) for n in JUDGED))]
        mean = (lambda xs: sum(xs) / len(xs)) if judged else (lambda xs: math.nan)
        off = (form.broken(form.spread([r[3] for r in mine]), form.PARRIES)
               if t[0] == 'walk' else [])
        out.append((sum(r[0] for r in mine) / len(mine), mean([r[1] for r in judged]),
                    [r[2] for r in mine], mine[0][3], mean([look_of(r[3]) for r in judged]),
                    off))
    return out


def score(results):
    """(cost, held, stir) of one candidate's run results, in JOBS' order: the walks' look and
    power - a walk off its form, rejected -, the events, the rises and the stands parried, the
    stands' stir and steps, the landings past saving."""
    trials = by_trial(results)
    held = sum(t[0] for t in trials) / len(trials)
    walks = [t for (kind, _p, _e), t in zip(TRIALS, trials) if kind == 'walk']
    walked = [t for t in walks if t[1] == t[1]]
    if walks and (not walked or any(t[5] for t in walks)):
        return math.inf, held, math.nan
    stir = sum(t[1] for t in walked) / len(walked) if walked else math.nan
    cost = (stir + sum(t[4] for t in walked) / len(walked)
            + LOST_K * (len(walks) - len(walked)) / len(walks)) if walked else 0.0
    runs = [(t[0], r[3]) for (t, _k), r in zip(JOBS, results)]
    # A run lost on the relay (`run`) scores as a fall with nothing else.
    land = [LAND_K * w.get('landing', 0.0) + HEAD_K * w.get('head', 0.0)
            + GEAR_K * max(0.0, w.get('gear', 1.0) - 1.0)
            for kind, w in runs if kind == 'fall']
    for falls in ([w.get('fell', 1.0) for kind, w in runs if kind == which]
                  for which in ('event', 'rise', 'stand')):
        cost -= PARRY_K * (1.0 - sum(falls) / len(falls)) if falls else 0.0
    stands = [w['stir'] + TREAD_K * w['treads'] for kind, w in runs
              if kind == 'stand' and w.get('stir', math.nan) == w.get('stir', math.nan)]
    cost += sum(stands) / len(stands) if stands else 0.0
    return cost + (sum(land) / len(land) if land else 0.0), held, stir


def run(_pool, candidates):
    """[(cost, held, stir, results)] for `candidates`, every run of each a job on the relay
    (`focus.relay`: a baton a physical core, never a pool of its own - the user, 2026-10-03), its
    result the JSON line it prints (`--one`); a run lost counts as fallen."""
    from tools.dev import focus
    jobs = [focus.Job('%d.%d' % (c, k), [sys.executable, '-X', 'utf8', os.path.abspath(__file__),
                                         '--suite', SUITE, '--one', json.dumps(values), str(k)],
                      RUN_GB, 900.0)
            for c, values in enumerate(candidates) for k in range(len(JOBS))]
    got = {}
    for job, text, _code, _s in focus.relay(jobs):
        line = next((ln for ln in reversed(text.splitlines()) if ln.startswith('{')), None)
        got[job.name] = json.loads(line)['result'] if line else [0.0, None, 'lost', {}]
    out = []
    for c in range(len(candidates)):
        results = [got['%d.%d' % (c, k)] for k in range(len(JOBS))]
        out.append(score(results) + (results,))
    return out


def _show(values, cost, held, stir, results: list | tuple = ()):
    print('%-40s cost %6.2f  held %5.1f %%  stir %5.2f mm' % (
        ' '.join('%s=%g' % kv for kv in values.items()) or 'as it is', cost, 100 * held, stir))
    trials = by_trial(results) if results else []
    for (kind, pace, event), (h, s, whats, looks, _c, off) in zip(TRIALS, trials):
        print('    %-5s %.2f %-9s %5.1f %%  %s%s%s%s' % (
            kind, pace, event or '', 100 * h, ' | '.join(whats),
            '  OFF ITS FORM: ' + ', '.join('%s %.1f (%g)' % b for b in off) if off else '',
            '  stir %.2f mm' % s if kind == 'walk' else
            '  stir %.2f mm, %d treads' % (s, looks.get('treads', 0)) if kind == 'stand' else '',
            '  ' + shown(looks) if kind == 'walk' and looks
            else '  landing %.1f kN, head %.2f, bare %.2f' % (looks['landing'], looks['head'],
                                                             looks.get('bare', math.nan))
            if kind == 'fall' else ''))



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
    parser.add_argument('--sigma', type=float, default=0.08, help="the first step, of a span")
    parser.add_argument('--log', default='gait_montecarlo.jsonl', help='every candidate, a line')
    parser.add_argument('--suite', choices=sorted(SUITES), default='all',
                        help='the look on fantasy boards, the faults on the boards as built')
    parser.add_argument('--one', nargs=2, metavar=('VALUES', 'K'),
                        help="one run on the relay: a candidate's JSON and its job's index")
    args = parser.parse_args(argv)
    suite(args.suite)
    background.lower()
    if args.one:
        print(json.dumps({'result': trial((json.loads(args.one[0]), JOBS[int(args.one[1])]))}))
        return 0
    fixed = {k: float(v) for k, v in (a.split('=') for a in args.set)}
    began = time.time()
    with open(args.log, 'a') as log:
        if args.search:
            spans = {k: tuple(float(x) for x in v.split(':'))
                     for k, v in (a.split('=') for a in args.search)}
            cost, values = cmaes.search(
                None, spans, args.generations, args.population, log,
                lambda pool, cands: run(pool, [dict(fixed, **c) for c in cands]), knobs.now,
                args.sigma)
            print('BEST %.2f %s' % (cost, json.dumps(values)))
            cands = [dict(fixed, **(values or {}))]
        elif args.grid:
            axes = [(k, [float(x) for x in v.split(',')])
                    for k, v in (a.split('=') for a in args.grid)]
            cands = [dict(fixed, **dict(zip([k for k, _v in axes], combo)))
                     for combo in itertools.product(*[v for _k, v in axes])]
        else:
            cands = [fixed]
        got = run(None, cands)
        ranked = sorted(zip(cands, got), key=lambda cg: cg[1][0])
        for values, (cost, held, stir, results) in ranked:
            _show(values, cost, held, stir, results)
    print('%d candidates, %d runs, %.0f s' % (len(cands), len(cands) * len(JOBS),
                                              time.time() - began))
    return 0


if __name__ == '__main__':
    sys.exit(main())
