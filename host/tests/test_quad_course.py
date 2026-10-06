"""The quad's course (machine.course) on ideal rotors: its gates, what stands, drawn, its envelopes."""
import functools
import math
import random
import sys

from tools.dev.focus import chosen
from views_kit import Report

#: What the frame keeps clear between a propeller's tip and a gate's frame or a thing standing, m.
CLEAR_M = 0.25

#: The rotors the course is flown on here: their top, rad/s, their lag to a speed, s, and the
#: fastest they are spun up or down, rad/s^2.
TOP_RAD_S, LAG_S, SPOOL_RAD_S2 = 310.0, 0.08, 900.0


def reach():
    """The frame from its middle to a propeller's tip, m."""
    from coaxial.graphics import quadcopter
    from machine import quad
    return math.hypot(*quad.ROTOR_AT[0]) + quadcopter.BLADE_R


@functools.lru_cache(maxsize=None)
def lapped(share=1.0, spent_at=None, air=False):
    """The course's card once through, a row a pass: machine.quad's frame in MuJoCo on rotors
    lagging to their speed, on `share` of their pull, its passes a page's - 10 to 30 ms, one
    in twenty 50; `spent` from `spent_at` s on; in still air, or - `air` - the air's tour."""
    from machine import aerobatics, course, quad
    from machine.flying import Flying
    dice = random.Random(1)
    sky, route = quad.Sky(), aerobatics.routine(course.CARD)
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
                     'laps': lap.get('laps', 0), 'of': lap.get('of', 0)})
        t += dt
    return tuple(rows)


def passes(rows):
    """{gate: [(across, up, m/s)]}: where the frame passed each gate's plane on its laps, m off
    the gate's middle."""
    from machine import course
    laps = [r for r in rows if r['name'] == 'lap']
    out = {}
    for g, (gx, gy, gz, heading) in enumerate(course.GATES):
        nx, nz = math.sin(math.radians(heading)), math.cos(math.radians(heading))
        out[g] = []
        for a, b in zip(laps, laps[1:]):
            before = (a['x'] - gx) * nx + (a['z'] - gz) * nz
            after = (b['x'] - gx) * nx + (b['z'] - gz) * nz
            if before < 0.0 <= after and math.hypot(b['x'] - gx, b['z'] - gz) < 6.0 and b['v'] > 1.0:
                out[g].append(((b['x'] - gx) * nz - (b['z'] - gz) * nx, b['y'] - gy, b['v']))
    return out


def clear(rows):
    """[(what, m)], the nearest first: how near the frame's middle came to each thing standing -
    beside it or over it, whichever is more."""
    from machine import course
    out = []
    for x, z, high, crown in course.TREES:
        out.append(('the tree at (%g, %g)' % (x, z), min(
            (math.hypot(r['x'] - x, r['z'] - z) - crown for r in rows if r['y'] <= high),
            default=math.inf)))
    for x, z, wide, deep, _wall, ridge in course.HOUSES:
        out.append(('the house at (%g, %g)' % (x, z), min(
            max(abs(r['x'] - x) - wide / 2, abs(r['z'] - z) - deep / 2, r['y'] - ridge)
            for r in rows)))
    wide, high, long_ = course.CAR_M
    for x, z, heading in course.CARS:
        c, s = math.cos(math.radians(heading)), math.sin(math.radians(heading))
        out.append(('the car at (%g, %g)' % (x, z), min(
            max(abs((r['x'] - x) * c - (r['z'] - z) * s) - wide / 2,
                abs((r['x'] - x) * s + (r['z'] - z) * c) - long_ / 2, r['y'] - high)
            for r in rows)))
    for x, z, side, high in course.MASTS:
        out.append(('the mast at (%g, %g)' % (x, z), min(
            max(math.hypot(r['x'] - x, r['z'] - z) - side, r['y'] - high) for r in rows)))
    return sorted(out, key=lambda pair: pair[1])


def worst(gates):
    """The furthest any pass was from its gate's middle, m, across or up."""
    return max((max(abs(across), abs(up)) for hits in gates.values() for across, up, _v in hits),
               default=math.nan)


def lap_seconds(rows, lap):
    mine = [r['t'] for r in rows if r['name'] == 'lap' and r['laps'] == lap]
    return mine[-1] - mine[0] if mine else math.nan


def test_its_gates_are_flown(report):
    """The course from the floor and back: both laps through every gate inside its opening, a
    propeller's tip clear of its frame; clear of every tree, house, car and mast; leant and
    fast as a lap is; landed where it rose."""
    from machine import course
    rows = lapped()
    gates, room = passes(rows), course.GATE_M / 2.0 - reach() - CLEAR_M
    # The first gate is the grid: left from a stand and come to a stand in, passed between.
    counted = [len(gates[g]) for g in range(len(course.GATES))]
    report.check('every gate passed on each of its %d laps, its middle within %.2f m of the '
                 'frame\'s - an opening of %.1f m, %.2f m to a propeller\'s tip, %.2f m clear'
                 % (course.LAPS, room, course.GATE_M, reach(), CLEAR_M),
                 counted == [course.LAPS - 1] + [course.LAPS] * (len(course.GATES) - 1)
                 and worst(gates) <= room,
                 'passes %s; %.2f m off at the most' % (counted, worst(gates)))
    near = clear(rows)
    report.check('a propeller\'s tip %.2f m clear of every tree, house, car and mast' % CLEAR_M,
                 near[0][1] >= reach() + CLEAR_M,
                 '; '.join('%s %.2f m' % pair for pair in near[:3]))
    laps = [r for r in rows if r['name'] == 'lap']
    times = [lap_seconds(rows, lap) for lap in range(course.LAPS)]
    report.check('a lap of %.0f m in 15-30 s, 9 m/s and 50 degrees of lean in it'
                 % course.track()['length'],
                 all(15.0 <= s <= 30.0 for s in times) and max(r['v'] for r in laps) >= 9.0
                 and 50.0 <= max(r['tilt'] for r in laps) <= 80.0,
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
    gates, room = passes(half), course.GATE_M / 2.0 - reach() - CLEAR_M
    lean = [max(r['tilt'] for r in rows if r['name'] == 'lap') for rows in (whole, half)]
    times = [lap_seconds(rows, 1) for rows in (whole, half)]
    report.check('on half their pull a lap a fifth longer and 15 degrees less leant, its gates '
                 'as before',
                 times[1] >= 1.2 * times[0] and lean[1] <= lean[0] - 15.0
                 and all(len(hits) >= course.LAPS - 1 for hits in gates.values())
                 and worst(gates) <= room,
                 '%.1f s for %.1f, %.0f degrees for %.0f; %.2f m off at the most' % (
                     times[1], times[0], lean[1], lean[0], worst(gates)))
    at = 0.5 * lap_seconds(whole, 0) + next(r['t'] for r in whole if r['name'] == 'lap')
    spent = lapped(spent_at=at)
    gates, last = passes(spent), spent[-1]
    laps = [r for r in spent if r['name'] == 'lap']
    report.check('spent half a lap in, that lap its last: its gates passed once, then the '
                 'floor where it rose',
                 laps[-1]['of'] == 1 and [len(gates[g]) for g in gates] == [0] + [1] * (len(gates) - 1)
                 and worst(gates) <= room and last['name'] == course.CARD[-1][0]
                 and abs(last['y']) <= 0.01 and math.hypot(last['x'], last['z']) <= 0.3,
                 '%d of %d laps, passes %s, ends in its %s %.3f m up, %.2f m off' % (
                     laps[-1]['laps'] + 1, laps[-1]['of'], [len(gates[g]) for g in gates],
                     last['name'], last['y'], math.hypot(last['x'], last['z'])))


def test_its_gates_in_wind(report):
    """The course in the air's tour - a constant wind, gusts, a changing one and eddies in its
    two laps, none of it told to the law: every gate passed inside its opening, the frame
    clear of its frame; its laps a tenth slower at the most."""
    from machine import course
    still, rows = lapped(), lapped(air=True)
    gates, room = passes(rows), course.GATE_M / 2.0 - reach() - CLEAR_M
    counted = [len(gates[g]) for g in range(1, len(course.GATES))]
    report.check('every gate passed on both laps in the wind, its middle within %.2f m' % room,
                 counted == [course.LAPS] * len(counted) and worst(gates) <= room,
                 'passes %s, %.2f m off at the most, %.2f in still air' % (
                     counted, worst(gates), worst(passes(still))))
    times = [[lap_seconds(flown, lap) for lap in range(course.LAPS)] for flown in (still, rows)]
    report.check('its laps in the wind no more than a tenth slower',
                 all(0.9 * a <= b <= 1.1 * a for a, b in zip(*times)),
                 '%s s in the wind, %s still' % (' '.join('%.1f' % x for x in times[1]),
                                                ' '.join('%.1f' % x for x in times[0])))


def test_a_line_ends_itself(report):
    """A routine's card with a row that gives its own (aerobatics.fly): spent, a figure's row
    leaves for its flight's way down - not the card's first, nor on it the next flight's -; the
    line's row stays its own until it holds what it waits for."""
    from machine import aerobatics

    def line(route, _now):
        if route.get('done'):
            route['holds'] = ('lapped',)
        return dict(aerobatics.HOVER, speed=2.0)
    card = (('idle', aerobatics.DOWN, 0.0, 0.0, 'fit'), ('hover', aerobatics.HOVER, 9.0, 0.0, ''),
            ('descend', aerobatics.OVER, 0.0, 0.0, 'held'),
            ('idle', aerobatics.DOWN, 0.0, 0.0, 'fit'), ('lift', aerobatics.HOVER, 9.0, 0.0, ''),
            ('lap', line, 0.0, 0.0, 'lapped'), ('descend', aerobatics.OVER, 0.0, 0.0, 'held'))
    went = []
    for row in (1, 2, 4, 5):
        route = dict(aerobatics.routine(card), row=row)
        went.append(aerobatics.fly(route, 1.0, ('spent',))[0])
        went.append(route['row'])
    report.check('spent: the first flight\'s hover to its own way down and no further from '
                 'there, the second\'s lift to the second\'s, the line\'s row its own',
                 went == ['descend', 2, 'descend', 2, 'descend', 6, 'lap', 5], str(went))
    route = dict(aerobatics.routine(card), row=5)
    first = aerobatics.fly(route, 1.0, ('fit',))
    route['done'] = True
    aerobatics.fly(route, 2.0, ('fit',))
    after = aerobatics.fly(route, 3.0, ('fit',))
    report.check('the line gives its row, and holding what it waits for it is left',
                 first[0] == 'lap' and first[1]['speed'] == 2.0 and after[0] == 'descend'
                 and route['row'] == 6,
                 '%s at %.1f m/s, then %s' % (first[0], first[1]['speed'], after[0]))


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
    """coaxial.graphics.scenery from behind the grid, the alley's gate ahead: the course's gates
    stood only where one is named, that one in its own ink; an eye flown through a gate's frame
    and past the trees draws no line across the view; far off, nothing; the ground its grid."""
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
    for z in np.arange(-22.0, 32.0, 0.5):
        (dots, _ink), = scenery.props(m, cam, (0.0, 2.2, float(z)), 1)
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
          test_a_line_ends_itself, test_its_tilt_is_its_discs_own, test_its_world_is_drawn)


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
