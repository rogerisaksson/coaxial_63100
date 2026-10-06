#!/usr/bin/env python3
"""Monte Carlo over the quad's laps: each candidate through the same flights, a process a core.

A candidate is a few constants of its lap - `machine.course`'s, `machine.flying`'s, the page's
`flight`'s, each by `module.NAME` - and of its line through the gates: `turnN` and `tenseN`,
gate N's row of `course.WAYS`. Its trials, the same for every candidate, each the course's
card flown once through among what stands, solid:

- ideal: rotors lagging to their speed, on all of their pull, a page's passes;
- boards: four stand-in boards (the page's flight, no page), their envelopes binding;

each in still air and in the air's tour from SEEDS (`--seeds`); ideal, as a frame of each of
SIZES on a course as much larger (`quad.sized`); the boards, laid in each of ROOMS and begun on
a pack with each of PACKS of its charge. A trial's cost is its laps' seconds, by its frame's
clock; MISS_K a metre a gate's middle is passed further off than MISS_M, ROOM_K a metre the
frame has less than ROOM_M about it, both of the frame as built; struck, STRUCK_S and no laps
counted. A candidate's is its trials' mean: a line found on its planned seconds alone was
flown 0.9 m off its gates (2026-10-06).

    python tools/sim/quad_race.py                                    # as built
    python tools/sim/quad_race.py --suite boards --grid course.GRIP=0.6,0.7,0.8
    python tools/sim/quad_race.py --search course.GRIP=0.5:0.9 turn11=-25:25 --verify 6 7 8

A search overfits what it flew: `--verify` flies its find beside its start in the air of seeds
it never searched in, every trial, and says which holds.
"""
import argparse
import itertools
import json
import math
import os
import random
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from machine import aerobatics, course, flying, quad  # noqa: E402
from tools import REPO  # noqa: E402
from tools.dev import background  # noqa: E402
from tools.sim import cmaes  # noqa: E402

#: The air's seeds a candidate is flown in, beside still air (`--seeds`: a run's own); the
#: frames' sizes beside the one built; the boards' rooms beside the bench (`coaxial.model.
#: rooms`: stuffy, half again the air's path - 21.5 and 20.5 s at 0.76 of their envelopes where
#: the bench's 19.6 and 18.6 at 0.68; toasty, 45 C, spends them, 0.92, a lap of 34 s and down,
#: 2026-10-06) and what their pack begins on beside all of its charge.
SEEDS, SIZES, ROOMS, PACKS = (3, 4, 5), (0.75, 1.5), ('stuffy',), (0.6,)
SUITES = {'all': ('ideal', 'boards'), 'ideal': ('ideal',), 'boards': ('boards',)}
BENCH = 'bench'


def trials(seeds=SEEDS, suite='all'):
    """The trials of `suite`: [(rotors, the air's seed or None for still, the frame's size,
    the boards' room, the pack's charge)]."""
    rows = [(rotors, seed, 1.0, BENCH, 1.0) for rotors in ('ideal', 'boards')
            for seed in (None,) + tuple(seeds)]
    rows += [('ideal', None, size, BENCH, 1.0) for size in SIZES]
    rows += [('boards', None, 1.0, room, 1.0) for room in ROOMS]
    rows += [('boards', None, 1.0, BENCH, left) for left in PACKS]
    return [row for row in rows if row[0] in SUITES[suite]]

#: The ideal rotors: their top, rad/s, their lag to a speed, s, the fastest they are spun up or
#: down, rad/s^2 (tests/test_quad_course.py's).
TOP_RAD_S, LAG_S, SPOOL_RAD_S2 = 310.0, 0.08, 900.0

#: The cost beside the laps' seconds: a gate's middle passed further off than MISS_M, s a
#: metre; less than ROOM_M about the frame's reach, s a metre; a flight struck.
MISS_M, MISS_K, ROOM_M, ROOM_K, STRUCK_S = 0.45, 30.0, 0.35, 30.0, 90.0

#: A run's commit on the relay, GB, and its seconds at the most.
RUN_GB, RUN_S = 0.5, 240.0

#: Where a constant lives, by its name's module.
MODULES = {'course': course, 'flying': flying, 'quad': quad}

LOG = os.path.join(REPO, 'build', 'quad_race.jsonl')


def owner(name):
    """(the module a `module.NAME` lives in, NAME); the page's flight loaded where it is asked."""
    module, _dot, key = name.partition('.')
    if module == 'flight' and module not in MODULES:
        from terminal.views.quad import flight
        MODULES[module] = flight
    return MODULES[module], key


def put(values):
    """{name: value} set where each lives - `module.NAME`, or `turnN`, `tenseN` in gate N's
    row of `course.WAYS` - and the line laid again."""
    ways = [list(way) for way in course.WAYS]
    for name, value in values.items():
        way = re.fullmatch(r'(turn|tense)(\d+)', name)
        if way:
            ways[int(way.group(2))][way.group(1) == 'tense'] = float(value)
        else:
            setattr(*owner(name), float(value))
    course.WAYS = tuple(tuple(way) for way in ways)
    course.track.cache_clear()


def sized(size):
    """The frame, its law, its routine and its course `size` times as built."""
    quad.sized(size)
    flying.sized()
    aerobatics.sized()
    course.sized()


def now(name):
    """A constant's value where it lives."""
    way = re.fullmatch(r'(turn|tense)(\d+)', name)
    if way:
        return course.WAYS[int(way.group(2))][way.group(1) == 'tense']
    return float(getattr(*owner(name)))


def missed(rows):
    """(the furthest a gate's middle was passed off, m, across or up; the passes) of the laps'
    rows (x, y, z, m/s)."""
    worst, count = 0.0, 0
    for gx, gy, gz, heading in course.GATES:
        nx, nz = math.sin(math.radians(heading)), math.cos(math.radians(heading))
        for a, b in zip(rows, rows[1:]):
            before = (a[0] - gx) * nx + (a[2] - gz) * nz
            after = (b[0] - gx) * nx + (b[2] - gz) * nz
            if before < 0.0 <= after and math.hypot(b[0] - gx, b[2] - gz) < 6.0 and b[3] > 1.0:
                worst = max(worst, abs((b[0] - gx) * nz - (b[2] - gz) * nx), abs(b[1] - gy))
                count += 1
    return worst, count


def room(rows):
    """The least room about the frame's reach, m: its middle to each tree, house, car and mast
    - beside it or over it, whichever is more."""
    least = math.inf
    for x, z, high, crown in course.TREES:
        least = min([least] + [math.hypot(r[0] - x, r[2] - z) - crown
                               for r in rows if r[1] <= high])
    for x, z, wide, deep, _wall, ridge in course.HOUSES:
        least = min([least] + [max(abs(r[0] - x) - wide / 2, abs(r[2] - z) - deep / 2,
                                   r[1] - ridge) for r in rows])
    wide, high, long_ = course.CAR_M
    for x, z, heading in course.CARS:
        c, s = math.cos(math.radians(heading)), math.sin(math.radians(heading))
        least = min([least] + [max(abs((r[0] - x) * c - (r[2] - z) * s) - wide / 2,
                                   abs((r[0] - x) * s + (r[2] - z) * c) - long_ / 2,
                                   r[1] - high) for r in rows])
    for x, z, side, high in course.MASTS:
        least = min([least] + [max(math.hypot(r[0] - x, r[2] - z) - side, r[1] - high)
                               for r in rows])
    return least - quad.reach()


def ideal(air, _room=BENCH, _left=1.0):
    """The card on ideal rotors in `air`, the frame as it is sized - its rotors as fast at
    their tips, its passes by its clock: (the laps' rows by lap, what it struck, 0, 1, 0)."""
    dice = random.Random(1)
    size, clock = quad.scales()
    top, lag, spool = TOP_RAD_S / size, LAG_S * clock, SPOOL_RAD_S2 / (size * clock)
    sky, route = quad.Sky(course.solids()), aerobatics.routine(course.CARD)
    law = flying.Flying(4.0 * quad.K_THRUST * top ** 2, aerobatics.DOWN)
    w, t, laps = [0.0] * 4, 0.0, {}
    while t < 150.0 * clock:
        dt = clock * (0.05 if dice.random() < 0.05 else dice.choice((0.01, 0.014, 0.02, 0.03)))
        state = sky.state()
        route['seen'] = dict(law.seen(state, dt), spent=False)
        name, law.ask = aerobatics.fly(route, t, (['held'] if law.held else []) + ['fit'])
        if name == 'land' or state['hit']:
            break
        for k, thrust in enumerate(law.step(state, dt, 1.0)):
            more = (min(top, quad.speed_for(thrust)) - w[k]) * min(1.0, dt / lag)
            w[k] += max(-spool * dt, min(spool * dt, more))
        if air:
            quad.blown(air, dt)
        sky.step(w, dt, air)
        t += dt
        if name == 'lap':
            frame = sky.state()
            laps.setdefault((route.get('lap') or {}).get('laps', 0), []).append(
                (t, float(frame['at'][0]), frame['h'], float(frame['at'][2]),
                 math.sqrt(sum(float(c) ** 2 for c in frame['vel']))))
    return laps, sky.hit, 0.0, 1.0, 0.0


def boards(air, room=BENCH, left=1.0):
    """The card on four stand-in boards in `air` (the page's flight), laid in `room` - told
    to no observer - on a pack with `left` of its charge: (the laps' rows by lap, what it
    struck, the boards' worst envelope, the least share of their pull, its seconds on emergency
    power)."""
    from coaxial import Coaxial63100
    from machine.modes import SIMULATED
    view = owner('flight.CARD')[0]
    rotors = [view.arm(Coaxial63100(execution_mode=SIMULATED).open()) for _ in range(4)]
    for rotor in rotors:
        rotor['rig'].board.thermal.situation(room)
    sky, route = quad.Sky(course.solids()), aerobatics.routine(course.CARD)
    law, flight = flying.Flying(view.TOP_N, aerobatics.DOWN), view.fresh()
    flight['cells'].update(left=left, volts=quad.open_volts(left))
    flight['air'], t, read, laps, soa, least = air, 0.0, 0.0, {}, 0.0, 1.0
    try:
        while t < 150.0 and flight['stage'] != 'land' and not sky.hit:
            for t, frame in view.passed(rotors, sky, route, law, flight, t, 0.02):
                if flight['stage'] == 'lap':
                    laps.setdefault((route.get('lap') or {}).get('laps', 0), []).append(
                        (t, float(frame['at'][0]), frame['h'], float(frame['at'][2]),
                         math.sqrt(sum(float(c) ** 2 for c in frame['vel']))))
                    soa = max([soa] + [(r['budget'] or {}).get('worst') or 0.0 for r in rotors])
                    least = min(least, flight['share'])
            if t - read >= 0.1:
                read = t
                for rotor in rotors:
                    rotor['budget'], rotor['ident'], rotor['board_c'] = view.warmth(rotor['rig'])
    finally:
        for rotor in rotors:
            rotor['rig'].board.drive.off()
            rotor['rig'].gates.off()
            rotor['rig'].close()
    return laps, sky.hit, soa, least, view.WEP_S - flight['wep']['left']


def trial(values, job):
    """One flight of a candidate: its laps' seconds by the frame's clock, the gates' worst
    miss and the room about the frame, m of the frame as built, what it struck, the boards'
    worst envelope and least share, its WEP's seconds."""
    put(values)
    rotors, seed, size, room_, left = job
    sized(size)
    clock = quad.scales()[1]
    laps, struck, soa, least, wep = (ideal if rotors == 'ideal' else boards)(
        None if seed is None else quad.air(seed), room_, left)
    rows = [row[1:] for lap in sorted(laps) for row in laps[lap]]
    miss, passes = missed(rows) if rows else (math.nan, 0)
    return {'laps': [(laps[lap][-1][0] - laps[lap][0][0]) / clock for lap in sorted(laps)],
            'miss': miss / size, 'passes': passes,
            'room': room(rows) / size if rows else math.nan, 'struck': struck,
            'soa': soa, 'share': least, 'wep': wep}


def cost_of(result):
    """A trial's cost, s: its laps' seconds and what its misses and its room cost - struck, or
    a lap short, STRUCK_S."""
    if not result or result['struck'] or len(result['laps']) < course.LAPS:
        return STRUCK_S
    return (sum(result['laps']) + MISS_K * max(0.0, result['miss'] - MISS_M)
            + ROOM_K * max(0.0, ROOM_M - result['room']))


def score(results):
    """(cost, the share of the flights flown whole, the gates' worst miss, m) of a candidate's
    trials."""
    whole = [r for r in results if r and not r['struck'] and len(r['laps']) >= course.LAPS]
    return (sum(cost_of(r) for r in results) / len(results), len(whole) / len(results),
            max((r['miss'] for r in whole), default=math.nan))


def run(_pool, candidates):
    """[(cost, whole, miss, results)] for `candidates`, every flight of each a job on the relay
    (`focus.relay`), its result the JSON line it prints (`--one`); one lost counts as struck."""
    from tools.dev import focus
    seeds = [str(job[1]) for job in JOBS if job[0] == JOBS[0][0] and job[1] is not None]
    jobs = [focus.Job('%d.%d' % (c, k), [sys.executable, '-X', 'utf8', os.path.abspath(__file__),
                                         '--suite', SUITE, '--seeds', *seeds,
                                         '--one', json.dumps(values), str(k)], RUN_GB, RUN_S)
            for c, values in enumerate(candidates) for k in range(len(JOBS))]
    got = {}
    for job, text, _code, _s in focus.relay(jobs):
        line = next((ln for ln in reversed(text.splitlines()) if ln.startswith('{')), None)
        got[job.name] = json.loads(line)['result'] if line else None
    out = []
    for c in range(len(candidates)):
        results = [got['%d.%d' % (c, k)] for k in range(len(JOBS))]
        out.append(score(results) + (results,))
    return out


def _show(values, cost, whole, miss, results):
    print('%-44s cost %6.2f  whole %3.0f %%  miss %.2f m' % (
        ' '.join('%s=%g' % kv for kv in values.items()) or 'as it is', cost, 100 * whole, miss))
    for (rotors, seed, size, room_, left), r in zip(JOBS, results):
        print('    %-6s %-6s %-6s %s' % (rotors, 'still' if seed is None else 'air %d' % seed, (
            'x%g' % size if size != 1.0 else room_ if room_ != BENCH
            else '%.0f %%' % (100 * left) if left != 1.0 else ''), (
            'laps %s s, a gate %.2f m off, %.2f m about it%s%s' % (
                ' '.join('%.1f' % x for x in r['laps']) or '-', r['miss'], r['room'],
                ', SOA %.2f, pull %.0f %% at the least, WEP %.1f s' % (
                    r['soa'], 100 * r['share'], r['wep']) if rotors == 'boards' else '',
                '  STRUCK %s' % r['struck'] if r['struck'] else '')) if r else 'lost'))


SUITE, JOBS = 'all', trials()


def main(argv=None):
    global SUITE, JOBS
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--set', nargs='*', default=[], metavar='NAME=V',
                        help='constants for every candidate')
    parser.add_argument('--grid', nargs='*', default=[], metavar='NAME=V,V,..',
                        help='each combination a candidate')
    parser.add_argument('--search', nargs='*', default=[], metavar='NAME=LOW:HIGH',
                        help='CMA-ES over these spans')
    parser.add_argument('--generations', type=int, default=12)
    parser.add_argument('--population', type=int, default=12)
    parser.add_argument('--sigma', type=float, default=0.15, help='the first step, of a span')
    parser.add_argument('--log', default=LOG, help='every candidate, a line')
    parser.add_argument('--suite', choices=sorted(SUITES), default='all')
    parser.add_argument('--seeds', type=int, nargs='*', default=list(SEEDS),
                        help="the air's seeds flown, beside still air")
    parser.add_argument('--verify', type=int, nargs='*', default=[], metavar='SEED',
                        help="a search's find and its start flown in these seeds' air, every trial")
    parser.add_argument('--one', nargs=2, metavar=('VALUES', 'K'),
                        help="one flight on the relay: a candidate's JSON and its job's index")
    args = parser.parse_args(argv)
    SUITE, JOBS = args.suite, trials(args.seeds, args.suite)
    background.lower()
    if args.one:
        print(json.dumps({'result': trial(json.loads(args.one[0]), JOBS[int(args.one[1])])}))
        return 0
    fixed = {k: float(v) for k, v in (a.split('=') for a in args.set)}
    began = time.time()
    os.makedirs(os.path.dirname(args.log), exist_ok=True)
    with open(args.log, 'a') as log:
        if args.search:
            spans = {k: tuple(float(x) for x in v.split(':'))
                     for k, v in (a.split('=') for a in args.search)}
            cost, values = cmaes.search(
                None, spans, args.generations, args.population, log,
                lambda pool, cands: run(pool, [dict(fixed, **c) for c in cands]), now,
                args.sigma, start=fixed)
            print('BEST %.2f %s' % (cost, json.dumps(values)))
            cands = [dict(fixed, **(values or {}))]
            if args.verify:
                SUITE, JOBS = 'all', trials(args.verify)
                cands.append(fixed)
        elif args.grid:
            axes = [(k, [float(x) for x in v.split(',')])
                    for k, v in (a.split('=') for a in args.grid)]
            cands = [dict(fixed, **dict(zip([k for k, _v in axes], combo)))
                     for combo in itertools.product(*[v for _k, v in axes])]
        else:
            cands = [fixed]
        ranked = sorted(zip(cands, run(None, cands)), key=lambda cg: cg[1][0])
        for values, (cost, whole, miss, results) in ranked:
            _show(values, cost, whole, miss, results)
        if args.search and args.verify:
            print('the find %s where it never searched: %.2f, its start %.2f' % (
                'holds' if ranked[0][0] is cands[0] else 'does not hold',
                *[got[0] for cand in cands for values, got in ranked if values is cand]))
    print('%d candidates, %d flights, %.0f s' % (len(cands), len(cands) * len(JOBS),
                                                time.time() - began))
    return 0


if __name__ == '__main__':
    sys.exit(main())
