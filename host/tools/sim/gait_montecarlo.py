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
  from her side (`machine.events`); each laid at a spread of places (SPREAD), the trial held
  their mean,
- fall: a shove past saving, the page's P (`machine.events`).

Two suites (`--suite`): the look - the rises and the walks on fantasy boards, whose SOA never
binds (`physics.ENVELOPE` 0) - and the faults - the events and the falls on the boards as built.
`held`, the share of the trials' time she stood, shown; `stir`, the pendulum's mean over the
walks, mm. The cost in four, the bench's (2026-10-01): a walk on its look and its power
(`look_of`, `stir` with it), a fall in it ignored; an event on its parry and a rise on its start,
a fall the heaviest, FALL_K the share fallen; a fall past saving on its landing's peak, her
body's and her head's.
A candidate with no walking to judge ranks last. A single run a candidate scores chance - the
rise flips on 0.5 % of any knob (docs/findings/walk.md, 2026-09-26) - a spread of them scores
it.

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

from tools.dev import background
from tools.sim import cmaes

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
          ('event', 0.85, 'nudge'), ('fall', 0.85, 'shove'))

#: Each event laid at SPREAD steps (`events.STEP_M`, `events.GLITCH_STEP`). Laid at one place,
#: 2 % of an arm's swing flipped a slip or the hot knee and the held share ran 75-90 %
#: (2026-09-28).
SPREAD = (-1, 0, 1)

#: Each walk run at SPREAD steps of WALK_SPREAD of its pace, its costs meaned: one run a candidate
#: scored chance - 15.85 in a search, 38.97 run again rounded to four figures (2026-09-28).
WALK_SPREAD = 0.02

#: The suites: which kinds of trial each runs.
SUITES = {'all': ('rise', 'walk', 'event', 'fall'), 'look': ('rise', 'walk'), 'walk': ('walk',),
          'rise': ('rise',), 'faults': ('event', 'fall')}

#: (trial, spread step): every run a candidate makes (`suite` narrows them).
JOBS = [(t, k) for t in TRIALS for k in SPREAD]

#: Each rise landed in the squat from over its 2 mm, RISE_DROP_M more a SPREAD step and a third of
#: it a rise: she starts at the walk's own cadence whatever is asked (`director.PACE_RATE`), and
#: her three rises were one start - all three fell at 6.4 s (2026-10-02).
RISE_DROP_M = 0.002

#: The look's cost, a walk's: the thigh's reach ahead of upright at the landing past its reach
#: behind at the lift by more than BALANCE_DEG, BALANCE_K a degree; the head fore and aft past
#: SURGE_MM, SURGE_K a mm; the feet nearer than CLEAR_MM as they pass, CLEAR_K a mm. A walk is
#: looked at LOOK_HZ. Her legs a little further back, straight, graceful - not the trudge, the
#: feet far out in front and none behind (2026-09-28): the knee landing bent past KNEE_DEG, the
#: thigh swung out past where it lands by more than OVER_DEG, KNEE_K and OVER_K a degree.
BALANCE_DEG, BALANCE_K = 0.0, 0.2
KNEE_DEG, KNEE_K, OVER_DEG, OVER_K = 10.0, 0.5, 3.0, 0.5

#: The step taken out: the thigh short of REACH_DEG behind upright at the lift, REACH_K a degree,
#: the heaviest of the look - weighed light, the searches came to tiptoeing, the legs always in
#: front, the easiest balance (2026-09-28).
REACH_DEG, REACH_K = 10.0, 1.0
SURGE_MM, SURGE_K = 30.0, 0.15

#: The upper body's bob: the torso pitching past TORSO_DEG, TORSO_K a degree. The feet: the
#: stance's toes out of TOE_OUT degrees or the swinging foot's turned in, TOE_K a degree; the
#: ankle rolled past PRONATE_DEG under the shin, PRONATE_K a degree - pigeon-toed, the swinging
#: foot in 6.5, and overpronated at 4.9, to the eye (2026-09-28).
TORSO_DEG, TORSO_K = 2.0, 1.0
TOE_OUT, TOE_K, PRONATE_DEG, PRONATE_K = (5.0, 15.0), 0.5, 3.0, 0.5
CLEAR_MM, CLEAR_K = 5.0, 0.2
LOOK_HZ = 50.0

#: The landing's cost, a walk's, heavy - a soft walk keeps the drives whole, copper lost before
#: anything broken (2026-09-28): the sole's peak over IMPACT_S from its touch past IMPACT_N,
#: IMPACT_K a newton; the ankle falling past TOUCH_MS as it touches, TOUCH_K a m/s. A touch: the
#: sole bearing TOUCH_N after QUIET_S of none, read every millisecond.
IMPACT_S, IMPACT_N, IMPACT_K = 0.03, 700.0, 0.01
TOUCH_MS, TOUCH_K = 0.15, 20.0

#: The landing's loading rate, the clonk heard: the sole's steepest rise over a millisecond in
#: the impact's window past RATE_KN_S kN/s, RATE_K a kN/s - she should be as quiet as a person,
#: only her clothes heard against her (2026-09-28).
RATE_KN_S, RATE_K = 20.0, 0.02
TOUCH_N, QUIET_S = 30.0, 0.1

#: The drives' load: the share of drive-ms at a drive's peak past LOAD_PCT %, LOAD_K a percent -
#: at 0.85 strides/s 0.47 % of all, the ankles 1.78, the knees 1.40, the hips 0.95: at each
#: landing and as each leg is snapped into its swing (2026-09-30).
LOAD_PCT, LOAD_K = 0.0, 4.0

#: Her power walking, W - the page's sum (`machine.running`): the work done, the copper's heat,
#: the boards' own - ENERGY_K a watt past ENERGY_W; 482 W at 0.85 strides/s (2026-10-01).
ENERGY_W, ENERGY_K = 300.0, 0.05

#: An event's fall, FALL_K the share of its runs that fell: the heaviest - one more of 30 is 10. A
#: fall past saving, its landing over LAND_S from the fall: LAND_K a kN of the peak her body bears
#: on the floor, feet aside, HEAD_K a kN of her head's; GEAR_K a share past its rating of the
#: worst gearbox's peak torque (`World.geared`, `drives.shock`) - broken.
FALL_K, LAND_S, LAND_K, HEAD_K, GEAR_K = 300.0, 1.5, 2.0, 10.0, 100.0

#: The look's measures, by `strides.WALK`'s names, and the landing's and the power's.
LOOKS = ('thigh ahead at landing', 'thigh behind at lift', 'head fore-aft', 'feet clear',
         'torso pitch', 'toe out', 'toe out swinging', 'ankle roll', 'knee at landing',
         'thigh most ahead')
LANDS = ('impact', 'touch', 'rate', 'load', 'power')


def suite(name):
    """TRIALS and JOBS narrowed to suite `name`'s kinds."""
    global TRIALS, JOBS
    TRIALS = tuple(t for t in TRIALS if t[0] in SUITES[name])
    JOBS = [(t, k) for t, k in JOBS if t[0] in SUITES[name]]


def look_of(looks):
    """The look's cost of a walk's measures {name: value} (`LOOKS`)."""
    (ahead, behind, surge, clear, torso, out, swinging, roll, knee, most, impact, touch,
     rate, load, power) = (looks.get(n, math.nan) for n in LOOKS + LANDS)
    terms = (BALANCE_K * max(0.0, ahead - behind - BALANCE_DEG),
             REACH_K * max(0.0, REACH_DEG - behind),
             SURGE_K * max(0.0, surge - SURGE_MM), CLEAR_K * max(0.0, CLEAR_MM - clear),
             TORSO_K * max(0.0, torso - TORSO_DEG),
             TOE_K * (max(0.0, TOE_OUT[0] - out) + max(0.0, out - TOE_OUT[1])
                      + max(0.0, -swinging)),
             PRONATE_K * max(0.0, roll - PRONATE_DEG), KNEE_K * max(0.0, knee - KNEE_DEG),
             OVER_K * max(0.0, most - ahead - OVER_DEG),
             IMPACT_K * max(0.0, impact - IMPACT_N), TOUCH_K * max(0.0, touch - TOUCH_MS),
             RATE_K * max(0.0, rate - RATE_KN_S), LOAD_K * max(0.0, load - LOAD_PCT),
             ENERGY_K * max(0.0, power - ENERGY_W))
    return sum(t for t in terms if t == t)

#: A trial's seconds, by kind; a walk's stir is meaned from SETTLE_S; events laid from
#: EVENT_AT_S.
SECONDS = {'rise': 20.0, 'walk': 14.0, 'event': 24.0, 'fall': 24.0}
SETTLE_S, EVENT_AT_S = 4.0, 5.0

#: A walk's run is judged on the strides it walked: its reach behind and its landing measured. A
#: walk trial none of whose runs walked costs LOST_K, its share of the walks: unjudged, falling at
#: 1.0 strides/s ranked better than walking it (2026-09-30).
JUDGED = ('thigh behind at lift', 'impact')
LOST_K = 30.0

MODULES = ('walker', 'gait', 'walkplan', 'landing', 'stance', 'arrival', 'director', 'falls', 'parry',
           'getup', 'observer',
           'capture', 'physics', 'mjcf', 'build', 'buses', 'events', 'drives')


def _set(values):
    """The constants set where they live - the first of MODULES holding the name, or the one
    named, walkplan.TRACK_M (gait has its own) - the plan's tables cleared. Unqualified, a name
    two modules hold sets the first: TURN_DEG meant for the fall turned the walk's pelvis
    (2026-10-01)."""
    import importlib
    mods = [importlib.import_module('machine.' + m) for m in MODULES]
    for name, value in values.items():
        if '.' in name:
            module, name = name.split('.', 1)
            mods_named = [m for m in mods if m.__name__ == 'machine.' + module]
            owner = next((m for m in mods_named if hasattr(m, name)), None)
        else:
            owner = next((m for m in mods if hasattr(m, name)), None)
        if owner is None:
            raise KeyError('no %s in machine.%s' % (name, ', machine.'.join(MODULES)))
        setattr(owner, name, value)
    gait, walkplan = mods[1], mods[2]
    gait._FITS.clear()
    walkplan._TABLES.clear()


def trial(job):
    """(held, stir or None, what happened, {look: value}) for one candidate's one run
    (`JOBS`): a rise or a walk on fantasy boards, an event on the boards as built."""
    values, ((kind, pace, event), k) = job
    faulted = kind in ('event', 'fall')
    _set(dict(values, ENVELOPE=1.0 if faulted else 0.0))
    from tools.sim import look, strides
    from machine import Machine, drives, events, figure, heat
    from machine.director import Director
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    director = Director(body, pace)
    if kind == 'rise':
        nth = [t for t in TRIALS if t[0] == 'rise'].index((kind, pace, event))
        director.begin(drop=0.002 + RISE_DROP_M * (k + 1 + nth / 3.0))
    else:
        director.cadence = director.walker.cadence = pace * (1.0 + WALK_SPREAD * k)
        director.walker.start()
        director.stage = 'walk'
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
    force, drawn, landing, head = np.zeros(6), 0.0, 0.0, 0.0
    while bus['t'] < seconds:
        body.loop.write(**director.step(0.001))
        body.loop.step(0.001)
        if director.stage in ('falling', 'fallen') and fell is None:
            fell, up = bus['t'], None
        if fell is not None and up is None:
            down += 0.001
            if director.stage == 'walk':
                up = bus['t']
        if (faulted and not laid and bus['t'] >= EVENT_AT_S
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
            body_n = head_n = 0.0
            for i in range(d.ncon):
                mine = [b for b in (m.geom_bodyid[d.contact[i].geom1],
                                    m.geom_bodyid[d.contact[i].geom2]) if b in ours]
                if len(mine) == 1 and not ours[mine[0]].endswith(('foot', 'toes')):
                    world._mj.mj_contactForce(m, d, i, force)
                    body_n += abs(force[0])
                    head_n += abs(force[0]) if ours[mine[0]] == 'head' else 0.0
            landing, head = max(landing, body_n / 1e3), max(head, head_n / 1e3)
    # A pool's worker lives on: its world's buses and block closed here, not at its exit - left,
    # five bus processes a run piled up to 865 and the host ran out of memory (2026-09-28).
    body.close()
    what = '%.1f m' % bus['pelvis.pose.z'] + (', tipped %.0f deg' % tilt if laid else '')
    if fell is not None:
        what = 'fell at %.1f s' % fell + (', up at %.1f s' % up if up else ', down') + ', ' + what
    walked = {name: f(rows) for name, _unit, f in strides.WALK if name in LOOKS} if rows else {}
    if impacts:
        walked.update(impact=sum(impacts) / len(impacts), touch=sum(touches) / len(touches),
                      rate=sum(rates) / len(rates))
    if drive_ms:
        walked['load'] = 100.0 * pinned / drive_ms
        walked['power'] = drawn / passes
    walked['fell'] = float(fell is not None)
    if faulted:
        walked.update(landing=landing, head=head,
                      gear=float(np.max(world.geared / np.array([drives.shock(j)
                                                                  for j in figure.JOINTS]))))
    return 1.0 - down / seconds, (stirred / passes if passes else None), what, walked


def by_trial(results):
    """[(held, stir, [what], {look: value}, look's cost)] a trial each, in TRIALS' order, from
    its runs' results in JOBS' order: the held share its spread's mean, the stir and the look's
    cost its judged runs' (`JUDGED`), nan with none; the measures shown the first run's."""
    out = []
    for t in TRIALS:
        mine = [r for (u, _k), r in zip(JOBS, results) if u == t]
        judged = [r for r in mine if r[1] is not None
                  and all(r[3].get(n, math.nan) == r[3].get(n, math.nan) for n in JUDGED)]
        mean = (lambda xs: sum(xs) / len(xs)) if judged else (lambda xs: math.nan)
        out.append((sum(r[0] for r in mine) / len(mine), mean([r[1] for r in judged]),
                    [r[2] for r in mine], mine[0][3], mean([look_of(r[3]) for r in judged])))
    return out


def score(results):
    """(cost, held, stir) of one candidate's run results, in JOBS' order: the walks' look and
    power, the events' and the rises' falls, the landings past saving."""
    trials = by_trial(results)
    held = sum(t[0] for t in trials) / len(trials)
    walks = [t for (kind, _p, _e), t in zip(TRIALS, trials) if kind == 'walk']
    walked = [t for t in walks if t[1] == t[1]]
    if walks and not walked:
        return math.inf, held, math.nan
    stir = sum(t[1] for t in walked) / len(walked) if walked else math.nan
    cost = (stir + sum(t[4] for t in walked) / len(walked)
            + LOST_K * (len(walks) - len(walked)) / len(walks)) if walked else 0.0
    runs = [(t[0], r[3]) for (t, _k), r in zip(JOBS, results)]
    land = [LAND_K * w['landing'] + HEAD_K * w['head'] + GEAR_K * max(0.0, w['gear'] - 1.0)
            for kind, w in runs if kind == 'fall']
    for falls in ([w['fell'] for kind, w in runs if kind == which] for which in ('event', 'rise')):
        cost += FALL_K * sum(falls) / len(falls) if falls else 0.0
    return cost + (sum(land) / len(land) if land else 0.0), held, stir


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
    for (kind, pace, event), (h, s, whats, looks, _c) in zip(TRIALS, trials):
        print('    %-5s %.2f %-5s %5.1f %%  %s%s%s' % (
            kind, pace, event or '', 100 * h, ' | '.join(whats),
            '' if kind != 'walk' else '  stir %.2f mm' % s,
            '  ahead %.1f behind %.1f deg, surge %.1f, clear %.1f mm, torso %.1f, toes %.1f'
            ' swinging %.1f, roll %.1f, knee %.1f, most %.1f deg, impact %.0f N, touch %.2f m/s,'
            ' rate %.0f kN/s, load %.2f %%, power %.0f W'
            % tuple(looks.get(n, math.nan) for n in LOOKS + LANDS) if kind == 'walk' and looks
            else '  landing %.1f kN, head %.2f' % (looks['landing'], looks['head'])
            if kind == 'fall' else ''))


def _now(name):
    """A constant's value where it lives (`_set`'s modules), its module named or not."""
    import importlib
    mods = [importlib.import_module('machine.' + m) for m in MODULES]
    module, _dot, bare = name.rpartition('.')
    owner = next(m for m in mods if hasattr(m, bare)
                 and (not module or m.__name__ == 'machine.' + module))
    return float(getattr(owner, bare))



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
    parser.add_argument('--workers', type=int, default=16)
    parser.add_argument('--suite', choices=sorted(SUITES), default='all',
                        help='the look on fantasy boards, the faults on the boards as built')
    args = parser.parse_args(argv)
    suite(args.suite)
    fixed = {k: float(v) for k, v in (a.split('=') for a in args.set)}
    background.lower()
    began = time.time()
    with multiprocessing.get_context('spawn').Pool(args.workers) as pool, \
            open(args.log, 'a') as log:
        if args.search:
            spans = {k: tuple(float(x) for x in v.split(':'))
                     for k, v in (a.split('=') for a in args.search)}
            cost, values = cmaes.search(
                pool, spans, args.generations, args.population, log,
                lambda pool, cands: run(pool, [dict(fixed, **c) for c in cands]), _now, args.sigma)
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
            _show(values, cost, held, stir, results)
    print('%d candidates, %d runs, %.0f s' % (len(cands), len(cands) * len(JOBS),
                                              time.time() - began))
    return 0


if __name__ == '__main__':
    sys.exit(main())
