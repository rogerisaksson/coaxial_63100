#!/usr/bin/env python3
"""Monte Carlo over the quad's laps: each candidate through the same flights, a process a core.

A candidate is a few constants of its lap - `machine.course`'s, `machine.flying`'s, the page's
`flight`'s, each by `module.NAME` - and of its line through the gates: `turnN`, `tenseN`,
`acrossN` and `upN`, gate N's row of `course.WAYS`. Its trials, the same for every candidate, each the course's
card flown once through among what stands, solid:

- ideal: rotors lagging to their speed, on all of their pull, a page's passes;
- boards: four stand-in boards (the page's flight, no page), their envelopes binding;

each in still air and in the air's tour from SEEDS (`--seeds`); ideal, as a frame of each of
SIZES on a course as much larger (`quad.sized`); the boards, laid in each of ROOMS and begun on
a pack with each of PACKS of its charge. A trial's cost is its laps' seconds, by its frame's
clock; MISS_K a metre a propeller's tip passes nearer a gate's frame than MISS_M, ROOM_K a
metre the frame has less than ROOM_M about it, both of the frame as built; struck, STRUCK_S and as
much again for the share of its gates it never passed, no laps counted. A candidate's is its trials' mean: a line found on its planned seconds alone was
flown 0.9 m off its gates (2026-10-06). Told beside it, not counted: the rotors' thrust of
their top, the flight's jerk, m/s^3 rms, and how often a lap its pull along its way turned.

    python tools/sim/quad_race.py                                    # as built
    python tools/sim/quad_race.py --suite boards --grid course.PULL=0.6,0.7,0.8
    python tools/sim/quad_race.py --search course.PULL=0.5:0.9 turn11=-25:25 --verify 6 7 8

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

from machine import aerobatics, course, flying, grounds, quad  # noqa: E402
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

#: The ideal rotors' lag to a speed past their spool, s; their top and spool are the page's
#: flight's: a stand-in rotor under the page's speed loop came to its ask a pass after its
#: spool, 40 ms for 50 rad/s and 60 for 100 at 20 ms passes (2026-10-10). At 0.08 the raw
#: plan struck its gates where the boards flew it whole.
LAG_S = 0.02

#: The cost beside the laps' seconds: a propeller's tip nearer a gate's frame than MISS_M, s a
#: metre - a 3.4 m gate's middle passed 0.67 m off, a 2.6 m window's 0.27 -; less than ROOM_M
#: about the frame's reach, s a metre; a flight struck. At 30 s a metre a search bought 2 s a
#: lap with a small frame's tip 0.26 m past its margin (2026-10-09).
MISS_M, MISS_K, ROOM_M, ROOM_K, STRUCK_S = 0.35, 150.0, 0.35, 30.0, 90.0

#: How a flight flowed (`flowed`): its pull smoothed over this long, s, and a turn of its pull
#: along its way counted from this much either way, m/s^2.
FLOW_S, FLOW_M_S2 = 0.1, 1.0

#: A run's commit on the relay, GB, and its seconds at the most; the relay's batons, None its
#: own (`--batons`: fewer leave the page's terminal a core).
RUN_GB, RUN_S, BATONS = 0.5, 240.0, None

#: A crossing's tuned names, in its row's order (`course.WAYS`).
WAY_ROW = ('turn', 'tense', 'across', 'up')
WAY = r'(%s)(\d+)' % '|'.join(WAY_ROW)

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
    """{name: value} set where each lives - `module.NAME`, or `turnN`, `tenseN`, `acrossN`,
    `upN` in gate N's row of `course.WAYS` - and the line laid again."""
    ways = [list(way) for way in course.WAYS]
    for name, value in values.items():
        way = re.fullmatch(WAY, name)
        if way:
            ways[int(way.group(2))][WAY_ROW.index(way.group(1))] = float(value)
        else:
            setattr(*owner(name), float(value))
    course.WAYS = tuple(tuple(way) for way in ways)
    course.track.cache_clear()


def sized(size):
    """The frame, its law, its routine, its grounds and its course `size` times as built."""
    quad.sized(size)
    flying.sized()
    aerobatics.sized()
    grounds.sized()
    course.sized()


def now(name):
    """A constant's value where it lives."""
    way = re.fullmatch(WAY, name)
    if way:
        return course.WAYS[int(way.group(2))][WAY_ROW.index(way.group(1))]
    return float(getattr(*owner(name)))


def missed(rows):
    """(the most a propeller's tip passed nearer a gate's frame than MISS_M, m - none past it,
    less than nothing; the passes) of the laps' rows (x, y, z, m/s), MISS_M of the frame as
    built: unscaled, a frame of 0.75 its size was held 0.47 of its own from a gate's frame
    (2026-10-10)."""
    worst, count = -math.inf, 0
    size_, clock = quad.scales()
    for gx, gy, gz, heading, size in grounds.GATES:
        leaves = size / 2.0 - quad.reach() - MISS_M * size_
        nx, nz = math.sin(math.radians(heading)), math.cos(math.radians(heading))
        for a, b in zip(rows, rows[1:]):
            before = (a[0] - gx) * nx + (a[2] - gz) * nz
            after = (b[0] - gx) * nx + (b[2] - gz) * nz
            if (before < 0.0 <= after and math.hypot(b[0] - gx, b[2] - gz) < 6.0 * size_
                    and b[3] > size_ / clock):
                worst = max(worst, abs((b[0] - gx) * nz - (b[2] - gz) * nx) - leaves,
                            abs(b[1] - gy) - leaves)
                count += 1
    return worst, count


def room(rows):
    """The least room about the frame's reach, m: its middle to each thing standing
    (`grounds.clearances`) and to the floor: a frame banked 84 degrees in the slalom's last
    bend fell from 1.9 m into it (2026-10-09)."""
    least = min(r[1] + quad.SKID_M + quad.FOOT_M for r in rows)
    return min(least, grounds.clearances([r[:3] for r in rows])[0][1]) - quad.reach()


def flowed(laps):
    """How a flight's laps flowed, from their rows (t, x, y, z, m/s, the pull flown x, y, z,
    the rotors' thrust of their top): (that thrust's mean; its jerk, m/s^3 rms, the pull
    smoothed over FLOW_S; how often a lap its pull along its way turned from speeding to
    slowing or back, FLOW_M_S2 either way)."""
    rows = [row for lap in sorted(laps) for row in laps[lap]]
    if len(rows) < 3:
        return math.nan, math.nan, math.nan
    pulls, k = [], 0
    for i, row in enumerate(rows):
        while rows[k][0] < row[0] - FLOW_S:
            k += 1
        pulls.append([sum(r[n] for r in rows[k:i + 1]) / (i + 1 - k) for n in (5, 6, 7)])
    square, turns, was = 0.0, 0, 0.0
    for i in range(1, len(rows)):
        dt = rows[i][0] - rows[i - 1][0]
        way = [rows[i][n] - rows[i - 1][n] for n in (1, 2, 3)]
        far = math.sqrt(sum(x * x for x in way)) or 1.0
        square += sum((a - b) ** 2 for a, b in zip(pulls[i], pulls[i - 1])) / dt
        along = sum(a * x for a, x in zip(pulls[i], way)) / far
        if abs(along) >= FLOW_M_S2 and along * was <= 0.0:
            turns, was = turns + (was != 0.0), along
    return (sum(r[8] for r in rows) / len(rows), math.sqrt(square / (rows[-1][0] - rows[0][0])),
            turns / len(laps))


def ideal(air, _room=BENCH, _left=1.0):
    """The card on ideal rotors in `air`, the frame as it is sized - its rotors as fast at
    their tips, its passes by its clock, war emergency power as the page's flight takes it:
    (the laps' rows by lap, what it struck, 0, 1, its seconds on WEP)."""
    view = owner('flight.CARD')[0]
    dice = random.Random(1)
    size, clock = quad.scales()
    top, lag, spool = view.TOP_RAD_S / size, LAG_S * clock, view.SPOOL_RAD_S2 / (size * clock)
    wep_top, wep_spool = view.TOP_WEP_RAD_S / size, spool * view.I_WEP / view.I_MAX
    sky, route = quad.Sky(grounds.solids()), aerobatics.routine(course.CARD)
    law = flying.Flying(4.0 * quad.K_THRUST * top ** 2, aerobatics.DOWN)
    normal, given = law.top, 4.0 * quad.K_THRUST * wep_top ** 2
    flight = {'wep': {'left': view.WEP_S * clock, 'on': 0.0, 'from': None, 'taken': 0}}
    w, t, laps = [0.0] * 4, 0.0, {}
    while t < 150.0 * clock:
        dt = clock * (0.05 if dice.random() < 0.05 else dice.choice((0.01, 0.014, 0.02, 0.03)))
        state = sky.state()
        route['seen'] = dict(law.seen(state, dt), spent=False)
        name, law.ask = aerobatics.fly(route, t, (['held'] if law.held else []) + ['fit'])
        if name == 'land' or state['hit']:
            break
        _share, wep = view.emergency(flight, sky, law, route, 1.0, dt / clock)
        law.top = given if wep else normal
        fast, spun = (wep_top, wep_spool) if wep else (top, spool)
        for k, thrust in enumerate(law.step(state, dt, 1.0, wep)):
            more = (min(fast, quad.speed_for(thrust, law.along)) - w[k]) * min(1.0, dt / lag)
            w[k] += max(-spun * dt, min(spun * dt, more))
        if air:
            quad.blown(air, dt)
        sky.step(w, dt, air)
        t += dt
        if name == 'lap':
            frame = sky.state()
            laps.setdefault((route.get('lap') or {}).get('laps', 0), []).append(
                (t, float(frame['at'][0]), frame['h'], float(frame['at'][2]),
                 math.sqrt(sum(float(c) ** 2 for c in frame['vel'])),
                 *(float(c) for c in frame['acc']), sum(x * x for x in w) / (4.0 * top * top)))
    return laps, sky.hit, 0.0, 1.0, (view.WEP_S - flight['wep']['left']) / clock


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
    sky, route = quad.Sky(grounds.solids()), aerobatics.routine(course.CARD)
    law, flight = flying.Flying(view.TOP_N, aerobatics.DOWN), view.fresh()
    flight['cells'].update(left=left, volts=quad.open_volts(left))
    flight['air'], t, laps, soa, least = air, 0.0, {}, 0.0, 1.0
    try:
        while t < 150.0 and flight['stage'] != 'land' and not sky.hit:
            for t, frame in view.passed(rotors, sky, route, law, flight, t, 0.02):
                if flight['stage'] == 'lap':
                    laps.setdefault((route.get('lap') or {}).get('laps', 0), []).append(
                        (t, float(frame['at'][0]), frame['h'], float(frame['at'][2]),
                         math.sqrt(sum(float(c) ** 2 for c in frame['vel'])),
                         *(float(c) for c in frame['acc']),
                         sum(quad.K_THRUST * r['w'] ** 2 for r in rotors) / view.TOP_N))
                    soa = max([soa] + [(r['budget'] or {}).get('worst') or 0.0 for r in rotors])
                    least = min(least, flight['share'])
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
    thrust, jerk, turns = flowed(laps)
    return {'laps': [(laps[lap][-1][0] - laps[lap][0][0]) / clock for lap in sorted(laps)],
            'miss': miss / size, 'passes': passes,
            'room': room(rows) / size if rows else math.nan, 'struck': struck,
            'soa': soa, 'share': least, 'wep': wep, 'thrust': thrust,
            'jerk': jerk * clock ** 3 / size, 'turns': turns}


def cost_of(result):
    """A trial's cost, s: its laps' seconds and what its misses and its room cost - struck, or
    a lap short, STRUCK_S and as much again for the share of its gates it never passed: at
    STRUCK_S whatever it passed, a search's every candidate struck alike and it had no way
    out (2026-10-10)."""
    if not result or result['struck'] or len(result['laps']) < course.LAPS:
        gates = course.LAPS * len(grounds.GATES) - 1
        passed = (result or {}).get('passes', gates if result else 0)
        return STRUCK_S * (2.0 - min(1.0, passed / gates))
    return (sum(result['laps']) + MISS_K * max(0.0, result['miss'])
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
    for job, text, _code, _s in focus.relay(jobs, BATONS):
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
            'laps %s s, a gate\'s frame %+.2f m past its margin, %.2f m about it, thrust %.0f %%, '
            'jerk %.0f, %.0f '
            'turns%s%s%s' % (
                ' '.join('%.1f' % x for x in r['laps']) or '-', r['miss'], r['room'],
                100 * r['thrust'], r['jerk'], r['turns'],
                ', SOA %.2f, pull %.0f %% at the least' % (r['soa'], 100 * r['share'])
                if rotors == 'boards' else '', ', WEP %.1f s' % r['wep'] if r['wep'] else '',
                '  STRUCK %s' % r['struck'] if r['struck'] else '')) if r else 'lost'))


SUITE, JOBS = 'all', trials()


def main(argv=None):
    global SUITE, JOBS, BATONS
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
    parser.add_argument('--batons', type=int, help="the relay's, its own where none")
    parser.add_argument('--one', nargs=2, metavar=('VALUES', 'K'),
                        help="one flight on the relay: a candidate's JSON and its job's index")
    args = parser.parse_args(argv)
    SUITE, JOBS, BATONS = args.suite, trials(args.seeds, args.suite), args.batons
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
