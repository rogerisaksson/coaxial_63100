"""The quad's course (machine.course, machine.grounds) on ideal rotors: its gates, what stands,
drawn, its envelopes."""
import functools
import math
import random
import sys

from tools.dev.focus import chosen
from views_kit import Report

#: What the frame keeps clear between a propeller's tip and a gate's frame or a thing standing, m.
CLEAR_M = 0.25

#: The rotors the course is flown on here: the page's flight's top, rad/s, and fastest spool,
#: rad/s^2; the tuner's lag to a speed, s.
from terminal.views.quad.flight import SPOOL_RAD_S2, TOP_RAD_S  # noqa: E402
from tools.sim.quad_race import LAG_S  # noqa: E402


def reach():
    """The frame from its middle to a propeller's tip, m."""
    from machine import quad
    return quad.reach()


@functools.lru_cache(maxsize=None)
def lapped(share=1.0, spent_at=None, air=False):
    """The course's card once through, a row a pass: machine.quad's frame in MuJoCo on rotors
    lagging to their speed, on `share` of their pull, its passes a page's - 10 to 30 ms, one
    in twenty 50; `spent` from `spent_at` s on; in still air, or - `air` - the air's tour;
    among what stands, solid (`grounds.solids`)."""
    from machine import aerobatics, course, grounds, quad
    from machine.flying import Flying
    dice = random.Random(1)
    sky, route = quad.Sky(grounds.solids()), aerobatics.routine(course.CARD)
    flying = Flying(4.0 * quad.K_THRUST * TOP_RAD_S ** 2, aerobatics.DOWN)
    blown = quad.air() if air else None
    w, t, rows = [0.0] * 4, 0.0, []
    while t < 240.0:
        dt = 0.05 if dice.random() < 0.05 else dice.choice((0.01, 0.014, 0.02, 0.03))
        state, spent = sky.state(), spent_at is not None and t >= spent_at
        route['seen'] = dict(flying.seen(state, dt), spent=spent)
        name, flying.ask = aerobatics.fly(route, t, (['held'] if flying.held else [])
                                          + ['spent' if spent else 'fit'])
        if rows and rows[-1]['name'] == course.CARD[-1][0] and name == course.CARD[0][0]:
            break
        for k, thrust in enumerate(flying.step(state, dt, share)):
            more = (min(TOP_RAD_S, quad.speed_for(thrust)) - w[k]) * min(1.0, dt / LAG_S)
            w[k] += max(-SPOOL_RAD_S2 * dt, min(SPOOL_RAD_S2 * dt, more))
        if blown:
            quad.blown(blown, dt)
        sky.step(w, dt, blown)
        state, lap = sky.state(), route.get('lap') or {}
        rows.append({'t': t, 'name': name, 'x': float(state['at'][0]), 'y': state['h'],
                     'z': float(state['at'][2]),
                     'v': math.sqrt(sum(float(c) ** 2 for c in state['vel'])),
                     'tilt': math.degrees(math.acos(max(-1.0, min(1.0, float(state['turn'][1][1]))))),
                     'laps': lap.get('laps', 0), 'of': lap.get('of', 0), 'hit': state['hit']})
        t += dt
    return tuple(rows)


def passes(rows):
    """{gate: [(across, up, m/s)]}: where the frame passed each gate's plane on its laps, m off
    the gate's middle."""
    from machine import grounds
    laps = [r for r in rows if r['name'] == 'lap']
    out = {}
    for g, (gx, gy, gz, heading, _size) in enumerate(grounds.GATES):
        nx, nz = math.sin(math.radians(heading)), math.cos(math.radians(heading))
        out[g] = []
        for a, b in zip(laps, laps[1:]):
            before = (a['x'] - gx) * nx + (a['z'] - gz) * nz
            after = (b['x'] - gx) * nx + (b['z'] - gz) * nz
            if before < 0.0 <= after and math.hypot(b['x'] - gx, b['z'] - gz) < 6.0 and b['v'] > 1.0:
                out[g].append(((b['x'] - gx) * nz - (b['z'] - gz) * nx, b['y'] - gy, b['v']))
    return out


def clear(rows):
    """[(what, m)], the nearest first: how near the frame's middle came to each thing standing
    (`grounds.clearances`)."""
    from machine import grounds
    return grounds.clearances([(r['x'], r['y'], r['z']) for r in rows])


def worst(gates):
    """The most any pass came nearer its gate's frame than CLEAR_M from a propeller's tip, m,
    across or up - less than nothing where every one kept it."""
    from machine import grounds
    return max((max(abs(across), abs(up)) - (grounds.GATES[g][4] / 2.0 - reach() - CLEAR_M)
                for g, hits in gates.items() for across, up, _v in hits), default=math.nan)


def lap_seconds(rows, lap):
    mine = [r['t'] for r in rows if r['name'] == 'lap' and r['laps'] == lap]
    return mine[-1] - mine[0] if mine else math.nan


def test_its_gates_are_flown(report):
    """The course from the floor and back: both laps through every gate inside its opening, a
    propeller's tip clear of its frame; clear of every tree, house, car and mast; leant and
    fast as a lap is; landed where it rose."""
    from machine import course, grounds, quad
    rows = lapped()
    gates = passes(rows)
    # The first gate is the grid: left from a stand and come to a stand in, passed between.
    counted = [len(gates[g]) for g in range(len(grounds.GATES))]
    report.check('every gate passed on each of its %d laps inside its opening, a propeller\'s '
                 'tip %.2f m clear of its frame' % (course.LAPS, CLEAR_M),
                 counted == [course.LAPS - 1] + [course.LAPS] * (len(grounds.GATES) - 1)
                 and worst(gates) <= 0.0,
                 'passes %s; %+.2f m past a gate\'s room at the most' % (counted, worst(gates)))
    near = clear(rows)
    report.check('a propeller\'s tip %.2f m clear of every tree, house, car and mast' % CLEAR_M,
                 near[0][1] >= reach() + CLEAR_M,
                 '; '.join('%s %.2f m' % pair for pair in near[:3]))
    laps = [r for r in rows if r['name'] == 'lap']
    times = [lap_seconds(rows, lap) for lap in range(course.LAPS)]
    # Past `held` no thrust holds it up; past 95 it goes over.
    held = math.degrees(math.acos(quad.MASS_KG * quad.GRAVITY / (4.0 * quad.K_THRUST
                                                                   * TOP_RAD_S ** 2)))
    report.check('a lap of %.0f m in 10-30 s, 9 m/s and 50 degrees of lean in it, knife-edge '
                 'at most - %.1f held' % (course.track()['length'], held),
                 all(10.0 <= s <= 30.0 for s in times) and max(r['v'] for r in laps) >= 9.0
                 and 50.0 <= max(r['tilt'] for r in laps) <= 95.0,
                 '%s s, %.1f m/s and %.0f degrees at the most' % (
                     ' and '.join('%.1f' % s for s in times), max(r['v'] for r in laps),
                     max(r['tilt'] for r in laps)))
    names = [r['name'] for i, r in enumerate(rows) if i == 0 or rows[i - 1]['name'] != r['name']]
    last = rows[-1]
    report.check('from the floor onto its grid, its laps, and landed where it rose',
                 names == [name for name, *_row in course.CARD] and abs(last['y']) <= 0.01
                 and math.hypot(last['x'], last['z']) <= 0.3,
                 '%s; ends %.3f m up, %.2f m off' % (' '.join(names), last['y'],
                                                    math.hypot(last['x'], last['z'])))


def test_it_flies_on_its_envelopes(report):
    """On half of the rotors' pull - the boards' envelopes half spent - the same line slower and
    leant less, every gate still inside its opening; spent in its first lap, that lap is its
    last and it lands where it rose."""
    from machine import course
    whole, half = lapped(), lapped(share=0.5)
    gates = passes(half)
    lean = [max(r['tilt'] for r in rows if r['name'] == 'lap') for rows in (whole, half)]
    times = [lap_seconds(rows, 1) for rows in (whole, half)]
    report.check('on half their pull a lap a tenth longer and less leant, its gates '
                 'as before',
                 times[1] >= 1.1 * times[0] and lean[1] < lean[0]
                 and all(len(hits) >= course.LAPS - 1 for hits in gates.values())
                 and worst(gates) <= 0.0,
                 '%.1f s for %.1f, %.0f degrees for %.0f; %+.2f m past a gate\'s room' % (
                     times[1], times[0], lean[1], lean[0], worst(gates)))
    at = 0.5 * lap_seconds(whole, 0) + next(r['t'] for r in whole if r['name'] == 'lap')
    spent = lapped(spent_at=at)
    gates, last = passes(spent), spent[-1]
    laps = [r for r in spent if r['name'] == 'lap']
    report.check('spent half a lap in, that lap its last: its gates passed once, then the '
                 'floor where it rose',
                 laps[-1]['of'] == 1 and [len(gates[g]) for g in gates] == [0] + [1] * (len(gates) - 1)
                 and worst(gates) <= 0.0 and last['name'] == course.CARD[-1][0]
                 and abs(last['y']) <= 0.01 and math.hypot(last['x'], last['z']) <= 0.3,
                 '%d of %d laps, passes %s, ends in its %s %.3f m up, %.2f m off' % (
                     laps[-1]['laps'] + 1, laps[-1]['of'], [len(gates[g]) for g in gates],
                     last['name'], last['y'], math.hypot(last['x'], last['z'])))


def test_its_gates_in_wind(report):
    """The course in the air's tour - a constant wind, gusts, a changing one and eddies in its
    two laps, none of it told to the law: every gate passed inside its opening, the frame
    clear of its frame; its laps a tenth slower at the most."""
    from machine import course, grounds
    still, rows = lapped(), lapped(air=True)
    gates = passes(rows)
    counted = [len(gates[g]) for g in range(1, len(grounds.GATES))]
    report.check('every gate passed on both laps in the wind inside its opening, nothing '
                 'struck',
                 counted == [course.LAPS] * len(counted) and worst(gates) <= 0.0
                 and not any(r['hit'] for r in rows),
                 'passes %s, %+.2f m past a gate\'s room at the most, %+.2f in still air' % (
                     counted, worst(gates), worst(passes(still))))
    times = [[lap_seconds(flown, lap) for lap in range(course.LAPS)] for flown in (still, rows)]
    report.check('its laps in the wind no more than a tenth slower',
                 all(0.9 * a <= b <= 1.1 * a for a, b in zip(*times)),
                 '%s s in the wind, %s still' % (' '.join('%.1f' % x for x in times[1]),
                                                ' '.join('%.1f' % x for x in times[0])))


def test_what_stands_is_solid(report):
    """machine.quad's world with the grounds' things in it (grounds.solids): every tree, house,
    hall, car, mast and gate, the first gate with no bar along the floor, a hall's windows none;
    the frame put in a tree's crown, in a house, on a car, against the mast and across a gate's
    bar has struck each - the gate only while the gates stand -, in a hall nothing and in its
    wall the wall; on its skids on the floor nothing, a disc on the floor the floor."""
    from machine import grounds, quad
    things = grounds.solids()
    kinds = [name.rstrip('0123456789') for _shape, name, *_size in things]
    window = min(grounds.windows())
    bars = [sum(name == 'gate%d' % k for _shape, name, *_size in things) for k in (0, 3, window)]
    hall = grounds.HALLS[0]
    report.check('a trunk and a crown a tree, walls and a roof a house and a hall, a body and a '
                 'cabin a car, the mast; a gate six bars, the first three: the floor its lower '
                 'edge, a window none: the wall its frame',
                 [kinds.count(kind) for kind in ('tree', 'house', 'car', 'mast')]
                 == [2 * len(grounds.TREES), 2 * len(grounds.HOUSES) + sum(
                     len(grounds.walls(h)[0]) + 1 for h in grounds.HALLS), 2 * len(grounds.CARS),
                     len(grounds.MASTS)] and bars == [3, 6, 0],
                 '%d things; gate 0 %d bars, gate 3 %d, window %d %d' % (
                     len(things), bars[0], bars[1], window, bars[2]))
    sky = quad.Sky(things)

    def put(at, over=0.0):
        """What the frame has struck a step after it is put at `at`, `over` rad about its
        nose."""
        sky.reset()
        sky.data.qpos[0:3] = at
        sky.data.qpos[3:7] = (math.cos(over / 2.0), 0.0, 0.0, math.sin(over / 2.0))
        sky.step([0.0] * 4, quad.STEP_S)
        return sky.state()['hit']
    tree, house, car, mast = grounds.TREES[0], grounds.HOUSES[0], grounds.CARS[0], grounds.MASTS[0]
    gate = grounds.GATES[1]
    bar = (gate[0], gate[1] + gate[4] / 2.0, gate[2])
    struck = [put((tree[0], 0.6 * tree[2], tree[1])), put((house[0], 2.0, house[1])),
              put((car[0], 0.6, car[1])), put((mast[0], 5.0, mast[1])), put(bar),
              put((hall[0], 2.5, hall[1])), put((hall[0] + hall[2] / 2.0, 2.0, hall[1]))]
    sky.stand(False)
    free = put(bar)
    sky.stand(True)
    rest = [put((0.0, quad.SKID_M + quad.FOOT_M, 0.0)), put((8.0, 0.3, -15.0), math.pi / 2.0)]
    report.check('put in a tree\'s crown, a house, a car, the mast and across a gate\'s bar it '
                 'has struck each - the gate only while the gates stand; in the hall nothing, '
                 'in its wall the wall; on its skids nothing, a disc on the floor the floor',
                 struck == ['tree', 'house', 'car', 'mast', 'gate1', None, 'house'] and free is None
                 and rest == [None, 'floor'],
                 '%s; the gates down %s; on the floor %s' % (struck, free, rest))


def test_what_is_ahead_is_seen(report):
    """quad.Sky.ahead, the frame's ghost flown half a second on as it goes: at the mast at
    8 m/s from 5 m it is in its way, away from it or slowly at it nothing is; through a gate
    on its middle nothing, 1.2 m off its middle the gate's own bar."""
    from machine import grounds, quad
    sky = quad.Sky(grounds.solids())
    mast, gate = grounds.MASTS[0], grounds.GATES[1]

    def ahead(at, vel):
        sky.reset()
        sky.data.qpos[0:3], sky.data.qvel[0:3] = at, vel
        sky._mj.mj_forward(sky.model, sky.data)
        return (sky.ahead(0.5) or (None,))[0]
    off = (mast[0] + 3.5, 9.0, mast[1] + 3.5)
    seen = [ahead(off, (-5.66, 0.0, -5.66)), ahead(off, (5.66, 0.0, 5.66)),
            ahead(off, (-1.41, 0.0, -1.41))]
    through = [ahead((gate[0] + x, gate[1] + quad.SKID_M + quad.FOOT_M, gate[2] - 3.0), (0.0, 0.0, 8.0))
               for x in (0.0, 1.2)]
    report.check('the mast in its way at 8 m/s from 5 m, not going from it nor at 2 m/s; a '
                 'gate\'s bar 1.2 m off its middle, nothing on it',
                 seen == ['mast', None, None] and through == [None, 'gate1'],
                 '%s; %s' % (seen, through))


def test_its_tuner_scores(report):
    """tools.sim.quad_race: a candidate's constants set where they live and its line laid
    again; a flight's cost its laps' seconds, a gate passed past its margin and a thing near
    counted, one struck the dearest."""
    from machine import course
    from tools.sim import quad_race as race
    was = (course.GRIP, course.WAYS, course.track()['length'])
    try:
        race.put({'course.GRIP': 0.5, 'turn3': 10.0, 'tense3': 1.2, 'across3': 0.3, 'up3': -0.2})
        put = (course.GRIP, course.WAYS[3], race.now('turn3'), race.now('course.GRIP'),
               race.now('across3'), course.track()['length'])
    finally:
        course.GRIP, course.WAYS = was[:2]
        course.track.cache_clear()
    report.check('a constant set where it lives, gate 3 crossed turned, tensed and off its '
                 'middle, the line laid again',
                 put[:5] == (0.5, (10.0, 1.2, 0.3, -0.2), 10.0, 0.5, 0.3)
                 and abs(put[5] - was[2]) > 0.01 and course.track()['length'] == was[2],
                 '%s; the line %.2f m where %.2f' % (put[:2], put[5], was[2]))
    flown = {'laps': [20.0, 19.0], 'miss': -0.1, 'room': 0.5, 'struck': None}
    costs = [race.cost_of(dict(flown, **more)) for more in (
        {}, {'miss': 0.1}, {'room': race.ROOM_M - 0.1}, {'struck': 'tree'}, {'laps': [20.0]})]
    cost, whole, miss = race.score([flown, dict(flown, struck='tree')])
    report.check('laps of 39 s cost them; a propeller\'s tip 0.1 m past its margin of a gate\'s '
                 'frame %.0f s more, 0.1 m less about the frame %.0f; struck or a lap short %.0f; '
                 'a candidate its flights\' mean' % (
                     0.1 * race.MISS_K, 0.1 * race.ROOM_K, race.STRUCK_S),
                 all(abs(got - want) < 1e-9 for got, want in zip(costs, (
                     39.0, 39.0 + 0.1 * race.MISS_K, 39.0 + 0.1 * race.ROOM_K, race.STRUCK_S,
                     race.STRUCK_S))) and abs(cost - (39.0 + race.STRUCK_S) / 2.0) < 1e-9
                 and whole == 0.5 and miss == -0.1,
                 '%s; %.1f, %.0f %% whole' % (['%.1f' % c for c in costs], cost, 100 * whole))


def test_a_frame_of_another_size(report):
    """quad.sized, the law, the routine and the course after it: frames smaller and larger,
    each on a course as much larger and rotors as fast at their tips, fly their laps as
    long by their own clocks, a gate as near in their own reaches. And built again."""
    from machine import course, grounds, quad
    from tools.sim import quad_race as race
    was = (quad.MASS_KG, grounds.GATES, course.SWING_S)
    try:
        flown = {size: race.trial({}, ('ideal', None, size, race.BENCH, 1.0))
                 for size in (1.0,) + race.SIZES}
    finally:
        race.sized(1.0)
    room = race.MISS_M - CLEAR_M
    built = flown[1.0]['laps']
    report.check('frames of %s times its size: their laps flown, nothing struck, a propeller\'s '
                 'tip %.2f m clear of a gate\'s frame and a lap within a fifth, in the built '
                 'frame\'s measure' % (' and '.join('%g' % s for s in race.SIZES), CLEAR_M),
                 all(len(f['laps']) == course.LAPS and not f['struck'] and f['miss'] <= room
                     and all(0.8 * a <= b <= 1.2 * a for a, b in zip(built, f['laps']))
                     for f in flown.values()),
                 '; '.join('x%g laps %s s, %+.2f m past its margin%s' % (
                     size, ' '.join('%.1f' % x for x in f['laps']), f['miss'],
                     ', struck %s' % f['struck'] if f['struck'] else '')
                     for size, f in flown.items()))
    report.check('and built again: its mass, its gates, its plan\'s times',
                 (quad.MASS_KG, grounds.GATES, course.SWING_S) == was,
                 '%.1f kg' % quad.MASS_KG)


def test_its_tilt_is_its_discs_own(report):
    """machine.flying's tilt loop, the frame's nose a quarter turn off its heading and a lean
    asked: the rotors' thrusts turn its discs the way of the lean, none across it; its nose is
    turned back to its heading where the row holds it there, left where it is free."""
    from machine import aerobatics, quad
    from machine.figure import ry
    from machine.flying import Flying

    def moments(yawed, **row):
        """(the thrusts' moment along the lean, across it, about the upright) for a level
        frame turned `yawed` degrees about its upright, a slide along +x asked of it."""
        flying = Flying(4.0 * quad.K_THRUST * TOP_RAD_S ** 2, dict(aerobatics.HOVER, slide=3.0, **row))
        turn = ry(math.radians(yawed))
        thrusts = flying.step({'turn': turn, 'spin': (0.0, 0.0, 0.0), 'vel': (0.0, 0.0, 0.0),
                               'at': (0.0, quad.HOVER_M, 0.0), 'h': quad.HOVER_M, 'v': 0.0,
                               'acc': (0.0, 0.0, 0.0), 'lift': quad.MASS_KG * quad.GRAVITY}, 0.01)
        arms = [(turn[0][0] * x + turn[0][2] * z, turn[2][0] * x + turn[2][2] * z)
                for x, z in quad.ROTOR_AT]
        return (sum(n * x for n, (x, _z) in zip(thrusts, arms)),
                sum(n * z for n, (_x, z) in zip(thrusts, arms)),
                sum(n * s for n, s in zip(thrusts, quad.SPIN)))
    straight, turned, free = moments(0.0), moments(90.0), moments(90.0, nose=0.0)
    report.check('a quarter turn off its heading the discs are still turned the lean\'s way, as '
                 'hard, none of it across',
                 straight[0] < -0.3 and abs(turned[0] - straight[0]) <= 0.05 * abs(straight[0])
                 and abs(turned[1]) <= 0.05 * abs(straight[0]),
                 '%.2f N m along and %.2f across, %.2f and %.2f on its heading' % (
                     turned[0], turned[1], straight[0], straight[1]))
    report.check('its nose turned back where the row holds it on its heading, left where free',
                 abs(straight[2]) < 1e-6 and abs(turned[2]) > 0.5 and abs(free[2]) < 1e-6,
                 '%.2f N apart held, %.2f free' % (turned[2], free[2]))


def test_its_world_is_drawn(report):
    """coaxial.graphics.scenery from behind the grid, the slalom's first gate ahead: the gates
    stood only where one is named, that one in its own ink; an eye flown through the first two
    gates past the trees draws no line across the view; far off nothing; the ground a grid."""
    from coaxial.graphics import engine, quadcopter, scenery, shapes
    from coaxial.model.blocks import numpy as np
    m, reach_m = shapes.view(180.0, 18.0), 4.0
    cam = engine.fine(engine.camera(100, 36, reach_m, distance=quadcopter.SIGHT * reach_m))

    def lit(dots, ink, name):
        """The cells of `ink`'s hue among those with dots."""
        want = np.asarray(scenery.INKS[name], float)
        cells = dots.reshape(36, 4, 100, 2).any(axis=(1, 3))
        size = np.linalg.norm(ink, axis=2)
        like = (ink @ want) / np.maximum(1e-9, size * np.linalg.norm(want))
        return int((cells & (size > 0.0) & (like > 0.9995)).sum())
    (bare, bare_ink), = scenery.props(m, cam, (0.0, 2.0, 5.0))
    (stood, ink), = scenery.props(m, cam, (0.0, 2.0, 5.0), 1)
    report.check('the gates stood only where one is named, the one flown to next in its ink',
                 stood.sum() > bare.sum() + 200 and lit(stood, ink, 'next') >= 20
                 and lit(bare, bare_ink, 'next') == 0 and lit(bare, bare_ink, 'gate') == 0
                 and lit(stood, ink, 'gate') >= 20 and lit(bare, bare_ink, 'tree') >= 20,
                 '%d dots for %d bare; %d cells of the next gate\'s ink, %d of the others\', %d '
                 'of the trees\'' % (stood.sum(), bare.sum(), lit(stood, ink, 'next'),
                                    lit(stood, ink, 'gate'), lit(bare, bare_ink, 'tree')))
    most = 0.0
    for z in np.arange(-4.0, 16.0, 0.5):
        (dots, _ink), = scenery.props(m, cam, (0.18 * float(z), 1.7 + 0.03 * float(z), float(z)), 1)
        most = max(most, float(dots.mean()))
    (far, _ink), = scenery.props(m, cam, (0.0, 2.0, 400.0), 1)
    report.check('an eye flown through the gates\' frames and past the trees draws under a '
                 'tenth of the view; 400 m off, nothing',
                 0.0 < most <= 0.1 and far.sum() == 0,
                 '%.1f %% of the dots at the most, %d far off' % (100.0 * most, far.sum()))
    floor = scenery.ground(m, cam, (0.0, 2.0, 5.0))
    report.check('the ground its grid, dimmer the further off',
                 floor.shape == (cam['height'], cam['width']) and 0.0 < floor.max() <= 1.0
                 and 200 <= (floor > 0.0).sum() and floor[floor > 0.0].min() >= 0.2,
                 '%d dots, %.2f-%.2f bright' % ((floor > 0.0).sum(), floor[floor > 0.0].min(),
                                               floor.max()))


ROSTER = (test_its_gates_are_flown, test_it_flies_on_its_envelopes, test_its_gates_in_wind,
          test_what_stands_is_solid, test_what_is_ahead_is_seen, test_its_tuner_scores,
          test_a_frame_of_another_size, test_its_tilt_is_its_discs_own, test_its_world_is_drawn)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
