"""The rotorcraft's sweep past the spectator (`machine.aircraft.sweep`): the quad on its core and ideal
rotors, the helicopter and the ornithopter in MuJoCo, each through its sweep and home; their
routines on the airfield (`grounds.world`)."""
import math
import random
import sys

from tools.dev.focus import chosen
from views_kit import Report


def _swept(name, rows, report):
    """`rows` (stage, along, across, m up, m/s, struck) checked: past the runway's middle - the
    spectator beside it - fast and low on the sweep's line, home for its figures."""
    from machine.aircraft import sweep
    low = sweep.ACROSS_M - 2.0 * sweep.TURN_M
    past = [r for r in rows if r[0] == 'sweep' and abs(r[1] + 2.0) < 1.0 and abs(r[2] - low) < 1.0]
    home = [r for r in rows if r[0] == 'hover']
    report.check('%s sweeps past the runway\'s middle fast and low on its line, then home' % name,
                 bool(past) and min(r[4] for r in past) > 8.0 and max(r[3] for r in past) < 2.0
                 and bool(home) and math.hypot(home[-1][1], home[-1][2]) < 0.5
                 and not any(r[5] for r in rows),
                 'past %s, home %s, struck %s' % (['%.1f m/s %.1f m' % (r[4], r[3]) for r in past[:3]],
                                                  home[-1][1:3] if home else None,
                                                  next((r[5] for r in rows if r[5]), None)))


def _row(stage, state, hit):
    from machine.aircraft import airfield
    s, c = math.sin(airfield.RUNWAY_RAD), math.cos(airfield.RUNWAY_RAD)
    x, z = float(state['at'][0]), float(state['at'][2])
    return (stage, x * s + z * c, x * c - z * s, state['h'],
            math.hypot(float(state['vel'][0]), float(state['vel'][2])), hit)


def test_the_quad_sweeps(report):
    """The quad's routine opened by the sweep, on its core and ideal rotors among what stands."""
    from machine.aircraft import aerobatics, aerokin, grounds, sweep
    from tools.sim import streams
    dice, flight = random.Random(1), streams.Flight(None, things=grounds.solids())
    sky, route, law = flight.sky, aerobatics.routine(sweep.opened(aerobatics.CARD)), aerokin.fresh()
    sky.stand(False)
    t, rows = 0.0, []
    while t < 60.0 and (not rows or rows[-1][0] != 'pirouette'):
        dt = dice.choice((0.01, 0.014, 0.02, 0.03))
        name, row = streams.asked(flight, route, law, t, dt, ('fit',), 1.0)
        streams.flew(flight, route, law, row, dt, 1.0)
        rows.append(_row(name, sky.state(), sky.state()['hit']))
        t += dt
    _swept('the quad', rows, report)


def test_the_helicopter_and_the_ornithopter_sweep(report):
    """Their routines opened by the quad's sweep, on their course's sphere, in MuJoCo."""
    from machine.aircraft import aerobatics, aerokin, grounds, helicopter, ornithopter, pull, sweep
    dt, tick = 0.02, 0.002
    for craft in (helicopter, ornithopter):
        sky, core, law = craft.Sky(grounds.solids()), craft.Core(), aerokin.fresh()
        sky.stand(False)
        route, rows = aerobatics.routine(sweep.opened(craft.CARD)), []
        top, have = craft.K_THRUST * pull.held(pull.I_MAX) ** 2 / craft.MASS_KG, craft.COURSE_SPHERE
        whole = aerokin.HEADROOM * craft.PULL_MAX
        for k in range(int(60.0 / dt)):
            st = sky.state()
            route['seen'] = dict(aerokin.seen(law, st, dt, have, top), spent=False)
            stage, row = aerobatics.fly(route, k * dt, ('held', 'fit') if law['held'] else ('fit',))
            along = core.plan(law, row, st, dt, whole)
            for _ in range(10):
                ans = core.step(law, row, sky.state(), tick, whole, None, along)
                sky.step(ans.speeds + ans.controls, tick)
            rows.append(_row(stage, sky.state(), sky.hit))
            if sky.hit is not None or stage == 'pirouette':
                break
        _swept('the ' + craft.__name__.rpartition('.')[2], rows, report)


def test_the_routines_fly_the_airfield(report):
    """The rotorcraft's world (`grounds.world`): the course's grounds standing for its flights
    alone, the airfield's runway and what stands by it for the routines, as the aerobat's."""
    from machine.aircraft import airfield, grounds, quad
    sky = quad.Sky(grounds.world())

    def struck(x, y, z):
        sky.reset()
        sky.data.qpos[0:3] = (x, y, z)
        sky._mj.mj_forward(sky.model, sky.data)
        return sky._struck(sky.data)
    tree, hangar = grounds.TREES[9], airfield.HOUSES[0]
    seen = []
    for course in (False, True):
        sky.stand(course)
        seen.append((struck(tree[0], 1.0, tree[1]), struck(hangar[0], 1.5, hangar[1])))
    report.check("a routine flies among the airfield's things, the course among its own",
                 seen == [(None, 'house'), ('tree', None)], seen)


ROSTER = (test_the_quad_sweeps, test_the_helicopter_and_the_ornithopter_sweep,
          test_the_routines_fly_the_airfield)


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
