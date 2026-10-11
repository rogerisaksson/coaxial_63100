"""Flight on the stack (`machine.flight`): her soles released together as she leaves the floor, one
alone only as lost, taken again as she comes down; a landing under her within her leg's reach;
the loop in C meeting a flying sole's ask as the python stack does; a hop stood on both."""
import math
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The stack laying this on each sole, N, and a sole pressing or not.
LAID, PRESSED, LIGHT = 150.0, 300.0, 0.0


def _run(passes):
    """`flight.released` after `passes` ((meant, loads, bears, rising), ..), a millisecond each."""
    from machine import flight
    st, out = flight.state(), (False, False)
    for meant, loads, bears, rising in passes:
        out = flight.released(st, meant, loads, bears, rising, 0.001)
    return out


def test_her_soles_leave_the_floor_together(report):
    """`flight.released`, a millisecond a pass: one sole light beside one that bears is kept 49
    ms and released alone at 50, the stack laying weight on it - never where it lays none; both
    light, both go in the pass the fourth millisecond ends; one in the air is taken again as it
    bears while she comes down, not while she rises; a sole the ask swings is never hers to
    release."""
    both, lay = (True, True), (LAID, LAID)
    lone = [(both, (LIGHT, PRESSED), lay, 0.0)]
    kept, lost = _run(lone * 49), _run(lone * 50)
    shifted = _run([(both, (LIGHT, PRESSED), (0.0, 2.0 * LAID), 0.0)] * 200)
    report.check('a sole light beside one that bears goes alone only as lost, the stack counting on it',
                 kept == (False, False) and lost == (True, False) and shifted == (False, False),
                 '49 ms %s, 50 ms %s, her weight shifted off it 200 ms %s' % (kept, lost, shifted))
    left = [(both, (LIGHT, LIGHT), lay, 0.5)]
    early, off = _run(left * 3), _run(left * 4)
    report.check('both light, both leave in the one pass', early == (False, False) and off == (True, True),
                 '3 ms %s, 4 ms %s' % (early, off))
    brushed = _run(left * 4 + [(both, (30.0, LIGHT), lay, 0.7)])
    landed = _run(left * 4 + [(both, (30.0, LIGHT), lay, -0.5)])
    report.check('a sole is taken again as it bears while she comes down, not as she rises',
                 brushed == (True, True) and landed == (False, True), 'rising %s, falling %s' % (brushed, landed))
    swung = _run([((True, False), (LIGHT, LIGHT), (LAID, 0.0), 0.0)] * 60)
    report.check('a sole the ask swings is not released', swung == (True, False), '%s' % (swung,))


def test_a_landing_is_under_her_within_reach(report):
    """`flight.landing` from a hip over a floor, her centre of mass still or moving: the sole
    asked flat, falling as she falls, its ankle within her leg's reach (`reach.margins`); still,
    under her centre of mass by its own way off it."""
    from machine import figure, flight, reach
    from machine.gait import ANKLE_H
    pelvis, worst, off_by, asks = (0.0, 0.92, 0.0), math.inf, 0.0, 0
    for limb, mirror in enumerate(reach.LIMBS):
        hip = figure.hip(mirror, pelvis, figure.IDENT)
        way = (mirror * 0.09, 0.0)
        for vx, vz in ((0.0, 0.0), (0.0, 0.4), (0.3, -0.2), (-0.5, 1.5), (2.0, 0.0)):
            com, vcom = (0.0, 1.0, 0.01), (vx, -0.6, vz)
            at, speed, acc, quat = flight.landing(limb, hip, 0.0, 0.0, com, vcom, way)
            ankle = (at[0], at[1] + ANKLE_H, at[2] - flight.MIDDLE)
            r = reach.over(limb, hip, ankle, figure.IDENT)
            worst, asks = min(worst, *reach.margins(r, flight.LAND_SHORT_M)), asks + 1
            if vx == vz == 0.0:
                off_by = max(off_by, abs(at[0] - com[0] - way[0]), abs(at[2] - com[2] - way[1]))
            if speed != vcom or acc != (0.0, -flight.G, 0.0) or quat != (1.0, 0.0, 0.0, 0.0):
                worst = -1.0
    report.check('a landing is flat, falls as she falls, and is within her reach', worst > -1e-9,
                 '%d asks, %.1f mm inside at the least' % (asks, worst * 1e3))
    report.check('still, a sole lands under her centre of mass by its own way off it', off_by < 1e-9,
                 '%.2g m off' % off_by)


def _flying():
    """(body, s, ask, names): her crouch lifted off the floor, rising 0.7 m/s, her knees going
    out 4.7 rad/s as at a take-off, no sole standing, each asked to its landing, no turn."""
    import numpy as np
    from machine import balance, flight, gait, physics, wbc
    from machine.figure import JOINTS
    from tools.sim.hop import crouched
    world, body = physics.World(), wbc.Body()
    names = [j for j, d in zip(JOINTS, body.driven) if d]
    world.reset(gait.stand())
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    st = balance.state(s, body.knees)
    qpos, qvel = world.data.qpos.copy(), np.zeros(len(world.data.qvel))
    qpos[body.qact] = crouched(st['posture'], names, 20.0)
    qpos[1] += 0.05
    qvel[1] = 0.7
    for k in body.knees:
        qvel[body.act[k]] = -4.7
    s = wbc.sense(body, qpos, qvel)
    hips = [body.d.xpos[body.m.body(seg).id].copy() for seg in ('left_thigh', 'right_thigh')]
    mid = (s['feet'][0]['sole'][0] + s['feet'][1]['sole'][0]) / 2.0
    swing = {k: flight.landing(k, hips[k], 0.0, 0.0, s['com'], s['vcom'],
                               (float(s['feet'][k]['sole'][0] - mid), 0.0)) for k in (0, 1)}
    return body, s, {'stance': (False, False), 'com_acc': np.zeros(3), 'posture': st['posture'],
                     'turns': (), 'swing': swing, 'wep': False}, names


def test_the_loop_meets_a_flying_soles_ask(report):
    """No sole standing, the loop in C plans each sole the acceleration the python stack plans,
    and no force: the margins' rows laid for a sole in the air too, it met 33 of 115 m/s^2."""
    import numpy as np
    from machine import wbc
    from tools.cores.wbc import shared, stack_step
    body, s, ask, _names = _flying()
    ours = wbc.step(body, s, ask)
    shared().reset()
    theirs = stack_step(body, s, ask)
    apart = max(float(np.abs(f['J6'] @ (ours['qacc'] - theirs['qacc'])).max()) for f in s['feet'])
    planned = [float((f['J6'] @ ours['qacc'] + f['dJv'])[5]) for f in s['feet']]
    report.check('no sole standing, the loop plans the soles as the python stack does',
                 apart < 0.5 and max(theirs['bears']) < 1e-6 and min(planned) < -50.0,
                 '%.2g m/s^2 apart, the soles asked back %.0f and %.0f m/s^2, bearing %.1g N'
                 % (apart, planned[0], planned[1], max(theirs['bears'])))


def test_she_hops_and_stands(report):
    """A hop on the python stack (`tools.sim.hop`): she leaves the floor with both soles at once,
    flies a tenth of a second or more with 15 mm under them, lands on both and stands."""
    from tools.sim import hop
    out = hop.hop()
    report.check('thrown, she flies, lands and stands',
                 out['stood'] and out['flights'] == 1 and out['flight_s'] >= 0.1 and out['clear_mm'] >= 15.0,
                 '%.3f s, %.1f mm under her soles, landed %d N, tilt %.1f deg'
                 % (out['flight_s'], out['clear_mm'], out['landed_n'], out['tilt']))
    report.check('her soles leave together and stand again', out['stances'] == 'SS -- SS', out['stances'])


def test_she_hops_on_the_loop(report):
    """The same on the loop in C, her throw asked 10 m/s^2: flown and stood."""
    from tools.sim import hop
    out = hop.hop(push=10.0, core=True)
    seen = out['stances'].split()
    report.check('on the loop in C she flies, lands and stands',
                 out['stood'] and out['flights'] == 1 and out['flight_s'] >= 0.1 and out['clear_mm'] >= 15.0
                 and seen[:2] == ['SS', '--'] and seen[-1] == 'SS',
                 '%.3f s, %.1f mm under her soles, landed %d N, tilt %.1f deg, %s'
                 % (out['flight_s'], out['clear_mm'], out['landed_n'], out['tilt'], out['stances']))


ROSTER = [test_her_soles_leave_the_floor_together, test_a_landing_is_under_her_within_reach,
          test_the_loop_meets_a_flying_soles_ask, test_she_hops_and_stands, test_she_hops_on_the_loop]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
