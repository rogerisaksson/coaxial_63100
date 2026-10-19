"""Leap on the stack (`machine.leap`): which soles stand told from one pass - each as it bears,
an unloaded one beside one that bears or with her at rest; a landing under her within her leg's
reach; the loop in C meeting an airborne sole's ask as the python stack does; a hop stood on both;
forward leaps from standing and in-stride scaled by hold charge."""
import math
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report

#: A sole pressing or not, N; one on the floor, a millimetre over it, in the air, m.
PRESSED, LIGHT = 300.0, 0.0
ON, NEAR, UP = 0.0, 0.001, 0.02


def test_her_soles_stand_as_they_bear(report):
    """`leap.standing` of one pass: at rest on the floor both stand before either bears, and an
    unloaded one beside one that bears whatever her speed; bearing nothing as she rises both
    leave in the one pass, a millimetre over the floor; as she falls none stands until one
    bears; one bearing alone stands at every speed of hers; a sole the ask swings never."""
    from machine.leap import standing
    both = (True, True)
    rest = standing(both, (LIGHT, LIGHT), (ON, ON), 0.0)
    beside = [standing(both, (LIGHT, PRESSED), (ON, ON), v) for v in (-0.5, 0.0, 0.3, 0.8)]
    report.check('at rest on the floor both stand, and an unloaded sole beside one that bears',
                 rest == both and beside == [both] * 4, 'at rest %s, beside %s' % (rest, beside))
    left = standing(both, (LIGHT, LIGHT), (NEAR, NEAR), 0.6)
    fell = standing(both, (LIGHT, LIGHT), (NEAR, UP), -0.8)
    report.check('bearing nothing as she rises both leave in the one pass; as she falls none stands',
                 left == fell == (False, False), 'rising %s, falling %s' % (left, fell))
    first = standing(both, (30.0, LIGHT), (ON, NEAR), -0.8)
    alone = [standing(both, (PRESSED, LIGHT), (ON, UP), v) for v in (-0.8, 0.05, 0.1, 0.11, 0.3, 0.8)]
    report.check('one that bears stands at every speed of hers, the other with it once on the floor',
                 first == both and alone == [(True, False)] * 6, 'the second beside it %s, alone %s' % (first, alone))
    swung = standing((True, False), (PRESSED, PRESSED), (ON, ON), 0.0)
    report.check('a sole the ask swings is never stood on', swung == (True, False), '%s' % (swung,))


def test_a_landing_is_under_her_within_reach(report):
    """`leap.landing` from a hip over a floor, her centre of mass still or moving: the sole
    asked flat, falling as she falls, its ankle within her leg's reach (`reach.margins`); still,
    under her centre of mass by its own way off it."""
    from machine import figure, leap, reach
    from machine.figure import ANKLE_H
    pelvis, worst, off_by, asks = (0.0, 0.92, 0.0), math.inf, 0.0, 0
    for limb, mirror in enumerate(reach.LIMBS):
        hip = figure.hip(mirror, pelvis, figure.IDENT)
        way = (mirror * 0.09, 0.0)
        for vx, vz in ((0.0, 0.0), (0.0, 0.4), (0.3, -0.2), (-0.5, 1.5), (2.0, 0.0)):
            com, vcom = (0.0, 1.0, 0.01), (vx, -0.6, vz)
            at, speed, acc, quat = leap.landing(limb, hip, 0.0, 0.0, com, vcom, way)
            ankle = (at[0], at[1] + ANKLE_H, at[2] - leap.MIDDLE)
            r = reach.over(limb, hip, ankle, figure.IDENT)
            worst, asks = min(worst, *reach.margins(r, leap.LAND_SHORT_M)), asks + 1
            if vx == vz == 0.0:
                off_by = max(off_by, abs(at[0] - com[0] - way[0]), abs(at[2] - com[2] - way[1]))
            if speed != vcom or acc != (0.0, -leap.G, 0.0) or quat != (1.0, 0.0, 0.0, 0.0):
                worst = -1.0
    report.check('a landing is flat, falls as she falls, and is within her reach', worst > -1e-9,
                 '%d asks, %.1f mm inside at the least' % (asks, worst * 1e3))
    report.check('still, a sole lands under her centre of mass by its own way off it', off_by < 1e-9,
                 '%.2g m off' % off_by)


def _airborne():
    """(body, s, ask, names): her crouch lifted off the floor, rising 0.7 m/s, her knees going
    out 4.7 rad/s as at a take-off, no sole standing, each asked to its landing, no turn."""
    import numpy as np
    from machine import figure, leap, physics, wbc
    from machine.figure import JOINTS
    from tools.sim.hop import crouched
    world, body = physics.World(), wbc.Body()
    names = [j for j, d in zip(JOINTS, body.driven) if d]
    world.reset(figure.stand())
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    st = {'posture': s['q'].copy()}
    qpos, qvel = world.data.qpos.copy(), np.zeros(len(world.data.qvel))
    qpos[body.qact] = crouched(st['posture'], names, 20.0)
    qpos[1] += 0.05
    qvel[1] = 0.7
    for k in body.knees:
        qvel[body.act[k]] = -4.7
    s = wbc.sense(body, qpos, qvel)
    hips = [body.d.xpos[body.m.body(seg).id].copy() for seg in ('left_thigh', 'right_thigh')]
    mid = (s['feet'][0]['sole'][0] + s['feet'][1]['sole'][0]) / 2.0
    swing = {k: leap.landing(k, hips[k], 0.0, 0.0, s['com'], s['vcom'],
                             (float(s['feet'][k]['sole'][0] - mid), 0.0)) for k in (0, 1)}
    return body, s, {'stance': (False, False), 'com_acc': np.zeros(3), 'posture': st['posture'],
                     'turns': (), 'swing': swing, 'wep': False}, names


def test_the_loop_meets_an_airborne_soles_ask(report):
    """No sole standing, the loop in C plans each sole the acceleration the python stack plans,
    and no force: the margins' rows laid for a sole in the air too, it met 33 of 115 m/s^2."""
    import numpy as np
    from machine import wbc
    from tools.cores.wbc import shared, stack_step
    body, s, ask, _names = _airborne()
    ours = wbc.step(body, s, ask)
    shared().reset()
    theirs = stack_step(body, s, ask)
    apart = max(float(np.abs(f['J6'] @ (ours['qacc'] - theirs['qacc'])).max()) for f in s['feet'])
    planned = [float((f['J6'] @ ours['qacc'] + f['dJv'])[5]) for f in s['feet']]
    report.check('no sole standing, the loop plans the soles as the python stack does',
                 apart < 0.5 and max(theirs['bears']) < 1e-6 and min(planned) < -50.0,
                 '%.2g m/s^2 apart, the soles asked back %.0f and %.0f m/s^2, bearing %.1g N'
                 % (apart, planned[0], planned[1], max(theirs['bears'])))


def test_forward_leap_scales_with_charge(report):
    """`leap.standing_leap`, `leap.stride_leap` and `leap.update_leap_charge`: a leap springs
    forward without braking first, and holding charge longer lengthens the forward throw and
    preload duration while keeping the active trajectory table continuous."""
    import numpy as np
    from machine import latent, leap, modal, sprung
    by = {e[0]: e for e in sprung.entries()}
    base_wire, walk_wire = by['hop'][1], by['#204'][1]
    tap_w, _names, tap_dur, tap_load = leap.standing_leap(base_wire, 0.0)
    full_w, _names, full_dur, full_load = leap.standing_leap(base_wire, 1.0)
    rtap = np.frombuffer(tap_w, latent.WIRE)
    rfull = np.frombuffer(full_w, latent.WIRE)
    s_tap = float(rtap['p'][1, latent.Z.start + modal.SAGITTAL])
    s_full = float(rfull['p'][1, latent.Z.start + modal.SAGITTAL])
    report.check('standing leap springs forward and scales with hold charge',
                 0.30 < s_tap < s_full and tap_load < full_load and tap_dur < full_dur,
                 'sagittal throw %.3f -> %.3f, load %d -> %d ms' % (s_tap, s_full, tap_load, full_load))
    st_tap, _n, st_tap_dur, _l = leap.stride_leap(base_wire, walk_wire, 0.0)
    st_full, _n, st_full_dur, _l = leap.stride_leap(base_wire, walk_wire, 1.0)
    rst_tap = np.frombuffer(st_tap, latent.WIRE)
    rst_full = np.frombuffer(st_full, latent.WIRE)
    d_tap = float(rst_tap['p'][1, latent.AMP.start + modal.DRIVE])
    d_full = float(rst_full['p'][1, latent.AMP.start + modal.DRIVE])
    report.check('in-stride leap scales forward drive and lift with hold charge without braking',
                 0.60 < d_tap < d_full and st_tap_dur < st_full_dur,
                 'drive amp %.3f -> %.3f, dur %.3f -> %.3f s' % (d_tap, d_full, st_tap_dur, st_full_dur))


def test_she_hops_and_stands(report):
    """A hop on the python stack (`tools.sim.hop`): she leaves the floor with both soles at once,
    is airborne a tenth of a second or more with 15 mm under them, lands on both and stands."""
    from tools.sim import hop
    out = hop.hop()
    report.check('thrown, she leaps, lands and stands',
                 out['stood'] and out['leaps'] == 1 and out['air_s'] >= 0.1 and out['clear_mm'] >= 15.0,
                 '%.3f s, %.1f mm under her soles, landed %d N, tilt %.1f deg'
                 % (out['air_s'], out['clear_mm'], out['landed_n'], out['tilt']))
    report.check('her soles leave together and stand again', out['stances'] == 'SS -- SS', out['stances'])


def test_she_hops_on_the_loop(report):
    """The same on the loop in C, and three in a row: each leapt, each landed, stood."""
    from tools.sim import hop
    for hops in (1, 3):
        out = hop.hop(hops=hops, core=True)
        seen = out['stances'].split()
        report.check('on the loop in C she leaps %d, lands and stands' % hops,
                     out['stood'] and out['leaps'] == hops and out['air_s'] >= 0.1 * hops
                     and out['clear_mm'] >= 15.0 and seen[:2] == ['SS', '--'] and seen[-1] == 'SS',
                     '%.3f s, %.1f mm under her soles, landed %d N, tilt %.1f deg, %s'
                     % (out['air_s'], out['clear_mm'], out['landed_n'], out['tilt'], out['stances']))


def test_she_leaps_in_stride_and_walks_on(report):
    """Walking at 0.69 m/s, told `hop` at mid-stride (`t = 2.0 s`), she gathers onto both soles,
    leaps with both soles off the floor (`air >= 0.15 s`, `>= 45 mm` clearance), lands and walks on."""
    from machine import worker
    for ch in (0.0, 1.0):
        s = worker.session()
        worker.apply(s, {'scenario': 'flat'})
        for _ in range(500):
            worker.stepped(s)
        worker.apply(s, {'twist': (0.69, 0.0, 0.0)})
        for _ in range(2000):
            worker.stepped(s)
        z0 = float(s['g']['est']['pos'][2])
        worker.apply(s, {'charge': ch, 'say': 'hop'} if ch > 0.0 else {'say': 'hop'})
        air_s, clear_mm = 0.0, 0.0
        for _ in range(3500):
            worker.stepped(s)
            l0, l1 = s['g']['loads']
            if l0 < 5.0 and l1 < 5.0:
                air_s += worker.DT
            clear_mm = max(clear_mm, 1e3 * min(float(f['pts'][:, 1].min()) for f in s['g']['s']['feet']))
        dz = float(s['g']['est']['pos'][2]) - z0
        report.check('in-stride leap (charge %.1f) flies on both soles, lands and walks on' % ch,
                     s['fallen_at'] is None and air_s >= 0.15 and clear_mm >= 45.0 and dz >= 0.45,
                     'air %.3f s, clear %.1f mm, dz %.2f m, fallen_at=%s'
                     % (air_s, clear_mm, dz, s['fallen_at']))


ROSTER = [test_her_soles_stand_as_they_bear, test_a_landing_is_under_her_within_reach,
          test_the_loop_meets_an_airborne_soles_ask, test_forward_leap_scales_with_charge,
          test_she_hops_and_stands, test_she_hops_on_the_loop, test_she_leaps_in_stride_and_walks_on]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
