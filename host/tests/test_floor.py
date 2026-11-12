#!/usr/bin/env python3
"""Her floor: the plane under her soles, its grip as she feels it, the situations that cap her rhythm,
what she does on it."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: Her trunk's pitch this far off her scoot's lean, deg, has tipped her over (her fall goes unflagged on the floor).
TIPPED = 50.0


def test_the_floor_runs_on_under_her_soles(report):
    """Level, the floor under each sole is the floor's height to the bit; pitched as her standing
    soles stand, a sole 0.3 m on along her way stands on it 0.3 tan(pitch) higher."""
    import numpy as np
    from machine.law import modal
    fr, y = np.zeros(modal.FRAME), np.zeros(modal.SENSED)
    fr[modal.ROT] = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float).ravel()
    fr[modal.FLOOR], fr[modal.FLOOR_AT] = 0.0123, (0.0, 0.0)
    y[modal.SOLE] = (0.1, 0.0, 0.0, -0.1, 0.0, 0.3)
    fr[modal.GROUND] = (0.004, -0.006)
    level, rise = modal.under(y, fr, np.array([1.0, 0.0]))
    report.check('a level floor (soft soles read it 0.004 rad) is its height under both soles',
                 (level == fr[modal.FLOOR]).all() and rise == 0.0, str(level))
    fr[modal.GROUND] = (-0.17, -0.17)
    up, rise = modal.under(y, fr, np.array([1.0, 0.0]))
    report.check('up a 10 deg ramp a sole 0.3 m ahead stands 0.3 tan(0.17) higher',
                 abs(up[1] - up[0] - 0.3 * math.tan(0.17)) < 1e-9 and abs(rise - math.tan(0.17)) < 1e-12,
                 '%.4f m' % (up[1] - up[0]))


def test_a_grown_prop_bears_where_it_has_grown(report):
    """A prop grown past its compiled size has its bounds grown with it: a 6 m top bears 2 m from its
    middle, where MuJoCo's cull by its compiled ones left it nothing."""
    import numpy as np
    from machine.world import floor, physics
    world = physics.World()
    floor.place(world, 'slope', z=0.25, top=6.0)
    m, d = world.model, world.data
    b = m.body('slope_top').id
    g = m.body_geomadr[b]
    world._mj.mj_forward(m, d)
    half = m.geom_size[g][:3]
    report.check('its geom\'s box, its sphere and its body\'s box are its size',
                 np.allclose(m.geom_aabb[g][3:], half) and np.allclose(m.bvh_aabb[m.body_bvhadr[b]][3:], half)
                 and abs(m.geom_rbound[g] - np.linalg.norm(half)) < 1e-9, '%s, %.3f' % (half, m.geom_rbound[g]))


def test_her_grip_bounds_her_rhythm(report):
    """Her text's slippery situation caps her rhythm under its grip, its targets kept; at her law's
    own grip nothing moves; a sliding sole's grip is what it used, a holding one's at least that, a
    landing one's once it has settled."""
    import numpy as np
    from machine.senses import grip, situation
    from machine.law import latent, modal, motion
    from machine.texts import ortho
    from machine.stack.wbc import MU
    table = situation.text()
    k = table['names'].index('slippery')
    report.check('her text has the slippery floor under a quarter of a sole\'s load',
                 table['hi'][k, situation.READINGS.index('grip')] == 0.25, str(table['hi'][k]))
    rng = np.random.default_rng(1)
    out = np.zeros(latent.OUT)
    out[latent.REF], out[latent.RATE], out[latent.ACC] = rng.normal(size=(3, ortho.N))
    out[latent.SWAY], out[latent.PSI], out[latent.TURNS] = 0.6, rng.uniform(0, 6, ortho.N), 5.9
    stance, level = np.ones(2), np.array([MU, 0.0, 0.0, 1.0, 0.0, 0.0])
    held = out.copy()
    motion.gripped(held, np.full(2, MU), stance, table['caps'], situation.read(table, level))
    report.check('at her law\'s own grip her rhythm is the same to the bit', (held == out).all())
    iced, on_ice = out.copy(), level.copy()
    on_ice[0] = 0.1
    motion.gripped(iced, np.full(2, 0.1), stance, table['caps'], situation.read(table, on_ice))
    target = lambda o: o[latent.REF] - o[latent.SWAY] * np.sin(o[latent.PSI])
    report.check('on ice (0.1) her drive sways 0.20 at most, her soles rock none, her targets kept',
                 iced[latent.SWAY][modal.DRIVE] <= 0.2 + 1e-12 and iced[latent.SWAY][modal.ROCK] == 0.0
                 and np.allclose(target(iced), target(out)),
                 'drive %.3f, rock %.3f' % (iced[latent.SWAY][modal.DRIVE], iced[latent.SWAY][modal.ROCK]))
    frames = np.broadcast_to(np.eye(3), (2, 2, 3, 3))
    touch = np.zeros((len(grip.edge.SKIN), 3))
    touch[grip.SOLES[:, 0]] = (8.0, 300.0, 0.0)
    touch[grip.SOLES[:, 0]] = (-8.0, 300.0, 0.0)
    seen, slid = np.zeros((2, grip.SEEN)), np.full(2, MU)
    for _ in range(grip.SEEN):
        slid = grip.felt(slid, seen, np.zeros(2), frames, touch, np.array([0.5, 0.0, 0.0]), 550.0, 1.0)
    held_on = grip.felt(np.full(2, 0.01), np.zeros((2, grip.SEEN)), np.zeros(2), frames, touch, np.zeros(3), 550.0,
                        grip.TRUST_S)
    report.check('a sole sliding at one share its last passes has that grip, a holding one at least it',
                 np.allclose(slid, 8.0 / 300.0) and (held_on >= 8.0 / 300.0).all(), '%s, %s' % (slid, held_on))
    landing, seen, settled, early = np.full(2, MU), np.full((2, grip.SEEN), -1.0), np.zeros(2), []
    for k in range(10):
        landing = grip.felt(landing, seen, settled, frames, touch, np.array([0.5, 0.0, 0.0]), 550.0, grip.SETTLE_S / 8.0)
        early += [float(landing.min())] if k < 7 else []
    report.check('a sole sliding since it landed reads that share once it has settled, its settle not before',
                 min(early) == MU and np.allclose(landing, 8.0 / 300.0), '%s, %s' % (min(early), landing))


def test_told_to_lie_on_the_floor_she_lies_where_she_stands(report):
    """Said lie_floor on the flat floor: she crouches, tips back soft and lies - pelvis under 0.25 m,
    doing floor_rest, her drives limp (se2.LIMP, clamps zero over the bus) - quiet (joints under 30
    deg/s, no contact past 300 N after the settle), no fall reset for 14 s (se2.GROUNDED)."""
    import numpy as np
    from machine.world import physics
    from machine.process import sprung, worker
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    s = worker.session()
    g, d = s['g'], s['g']['world'].data
    worker.apply(s, {'scenario': 'flat'})
    for _ in range(int(round(0.5 / worker.DT))):
        worker.stepped(s)
    worker.apply(s, {'say': 'lie_floor'})
    t0, worst_v, worst_f = g['t'], 0.0, 0.0
    import mujoco
    contact_force = getattr(mujoco, 'mj_contactForce')
    m, f6 = g['world'].model, np.zeros(6)
    while g['t'] < t0 + 14.0 and s.get('epoch', 0) == 0:
        worker.stepped(s)
        if g['t'] - t0 > 8.0:
            worst_v = max(worst_v, float(np.abs(d.qvel[6:]).max()))
            for i in range(d.ncon):
                contact_force(m, d, i, f6)
                worst_f = max(worst_f, abs(float(f6[0])))
    report.check('she lies where she stood: pelvis under 0.25, floor_rest, quiet, no reset',
                 s.get('epoch', 0) == 0 and float(d.qpos[1]) < 0.25 and sprung.doing(g) == 'floor_rest'
                 and math.degrees(worst_v) < 30.0 and worst_f < 300.0,
                 'pelvis %.3f doing %s worst %d deg/s %d N epoch %s' % (
                     d.qpos[1], sprung.doing(g), math.degrees(worst_v), worst_f, s.get('epoch', 0)))


def test_told_to_scoot_on_the_floor_she_sits_on_her_fists_and_her_keys_scoot_her(report):
    """Said scoot_floor on the flat floor: she sits (pelvis under 0.25 m) and her scoot (`machine.moves.scoot`, bound to
    her floor rest) holds her on her fists, her trunk within its band of the crawl take's lean and her pelvis within
    2 cm over the last 3 s of 12 without a fall; then S held 8 s scoots her back and W held 8 s forward, 0.2 m each
    at least, and A held 8 s turns her left and D right, 2 deg each at least, her trunk never TIPPED off its lean (her
    law's own rest crept on bent elbows and laid her down 4.6 s on; on the floor her fall goes unflagged)."""
    import numpy as np
    from machine.law import modal
    from machine.moves import scoot
    from machine.world import physics
    from machine.process import worker
    from terminal.views import humanoid_keys
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    s = worker.session()
    g, d = s['g'], s['g']['world'].data
    worker.apply(s, {'scenario': 'flat'})
    for _ in range(int(round(0.5 / worker.DT))):
        worker.stepped(s)
    worker.apply(s, {'say': 'scoot_floor'})
    t0, sat, still, lean = g['t'], False, None, 0.0
    while g['t'] < t0 + 12.0 and s['fallen_at'] is None:
        worker.stepped(s)
        sat = sat or float(d.qpos[1]) < 0.25
        if g['t'] >= t0 + 9.0:
            still = d.qpos[[0, 2]].copy() if still is None else still
            pitch = float(np.degrees(modal.pitched(g['y'], g['fr'][modal.ROT].reshape(3, 3))[1]))
            lean = max(lean, abs(pitch - scoot.TRUNK))
    played = (g.get('played') or {}).get('name')
    drift = float(np.hypot(*(d.qpos[[0, 2]] - still))) if still is not None else math.nan
    report.check('she sits on the floor, held on her fists by her scoot, still and at its lean, 12 s without a fall',
                 sat and s['fallen_at'] is None and played == 'scoot' and drift < 0.02 and lean <= scoot.LEANT,
                 'sat %s fallen %s playing %s drift %.3f m lean off %.1f deg' % (sat, s['fallen_at'], played, drift, lean))
    yaw = 2.0 * math.atan2(float(d.qpos[5]), float(d.qpos[3]))
    way, went, turned, off = np.array([math.sin(yaw), 0.0, math.cos(yaw)]), {}, {}, 0.0
    for key in ('s', 'w', 'a', 'd'):
        keys = dict.fromkeys(('w', 's', 'a', 'd', 'q', 'e', 'shift'), False)
        keys[key] = True
        at, was, t1 = d.qpos[0:3].copy(), 2.0 * math.atan2(float(d.qpos[5]), float(d.qpos[3])), g['t']
        worker.apply(s, {'twist': humanoid_keys.twist(keys)})
        while g['t'] < t1 + 8.0 and s['fallen_at'] is None:
            worker.stepped(s)
            pitch = float(np.degrees(modal.pitched(g['y'], g['fr'][modal.ROT].reshape(3, 3))[1]))
            off = max(off, abs(pitch - scoot.TRUNK))
        went[key] = float((d.qpos[0:3] - at) @ way)
        now = 2.0 * math.atan2(float(d.qpos[5]), float(d.qpos[3]))
        turned[key] = math.degrees((now - was + math.pi) % (2.0 * math.pi) - math.pi)
    report.check('S held scoots her back and W forward, 0.2 m each at least, her trunk never tipped',
                 went['s'] <= -0.2 and went['w'] >= 0.2 and off < TIPPED and s['fallen_at'] is None,
                 'S %+.2f m W %+.2f m trunk off its lean %.1f deg at most' % (went['s'], went['w'], off))
    report.check('A held turns her left and D right, her trunk never tipped',
                 turned['a'] >= 0.5 and turned['d'] <= -0.1 and off < TIPPED and s['fallen_at'] is None,
                 'A %+.1f deg D %+.1f deg trunk off its lean %.1f deg at most' % (turned['a'], turned['d'], off))


def test_her_legs_are_released_only_where_the_rest_of_what_bears_her_holds_her(report):
    """Her legs' release (intentional_limb_emancipation.lisp): asked where her soles are told nothing and something
    else is; allowed where her capture point lies inside what bears her besides them by the text's margin - 0.2 m
    inside a seat's and hands' square, open; outside it, or on her soles alone, shut; at her walk's 0.7 m/s her Froude
    number holds it to exp(-(Fr/0.045)^2)."""
    import math
    import numpy as np
    from machine.moves import cascade, emancipation as em
    from machine.law import latent
    from machine.texts import ortho
    weight, told, out = 305.5, np.array([0.0, 0.64, 0.0, 0.0, 0.36]), np.zeros(latent.OUT)
    report.check('her text asks her legs released where it tells her soles nothing and her seat and hands bear',
                 (cascade.intent(told, out) == 1.0).all() and (cascade.intent(np.array([1.0, 0, 0, 0, 0]), out) == 0.0).all()
                 and (cascade.intent(np.zeros(len(ortho.MANIFOLDS)), out) == 0.0).all(), str(cascade.intent(told, out)))
    square = np.array([(x, z) for x in (-0.2, 0.2) for z in (-0.2, 0.2)])
    soles = np.array([(0.1, 0.4), (-0.1, 0.4)])
    pts, room = np.concatenate([square, soles]), np.zeros(6)
    own = np.array([[0, 0, 0, 0, 1, 1], [0, 0, 0, 0, 1, 1]], float)
    loads, still = np.array([10.0, 10.0]), np.zeros(3)
    inside, parts = em.gate(np.zeros(2), pts, room, own, still, told, loads, weight)
    outside, far = em.gate(np.array([0.3, 0.0]), pts, room, own, still, told, loads, weight)
    soled, _ = em.gate(np.zeros(2), pts, room, np.ones((2, 6)), still, told, loads, weight)
    walking, going = em.gate(np.zeros(2), pts, room, own, np.array([0.0, 0.0, 0.7]), told, loads, weight)
    froude = 0.7 * 0.7 / (9.81 * cascade.LEG_M)
    report.check('0.2 m inside her seat and hands she may let her legs go; 0.1 m outside, or on her soles alone, not',
                 (inside == 1.0).all() and np.allclose(parts[3], 0.2 * weight, atol=1e-3 * weight)
                 and (outside == 0.0).all() and np.allclose(far[3], -0.1 * weight, atol=1e-3 * weight)
                 and (soled == 0.0).all(), '%s N m in, %s out' % (parts[3], far[3]))
    report.check("at her walk's 0.7 m/s her Froude number holds her gate to exp(-(Fr/0.045)^2)",
                 abs(going[0] - math.exp(-(froude / cascade.FROUDE) ** 2)) < 2e-3 and (walking < 0.2).all(), '%.3f' % going[0])


def test_her_release_lets_her_estimator_go_first_then_her_step_then_her_leg(report):
    """Released over the text's 160 ms (its cascade's windows): her estimator lets the sole go as an anchor by 0.25 of
    it, her step lets the leg go then sheds its foot's push to nothing by 0.75, the leg moves after - and back within
    the relock's 25 ms once the gate shuts."""
    import numpy as np
    from machine.moves import cascade as em
    from machine.law import modal
    gammas = np.linspace(0.0, 1.0, 101)
    unanchored = 1.0 - em.window(gammas, em.UNANCHOR)
    shed = em.window(gammas, em.SHED)
    let = em.window(gammas, (0.0, em.SHED[0]))
    free = em.window(gammas, em.FREE)
    first = lambda x: float(gammas[np.argmax(x)])
    report.check('anchor let go by 0.25, the leg let go by 0.15, its push shed from 0.15 to 0.75, free past it',
                 first(unanchored == 0.0) == 0.25 and first(let >= 1.0) == 0.15 and first(shed > 0.0) > 0.15
                 and first(shed >= 1.0) == 0.75 and first(free > 0.0) > 0.75,
                 '%.2f %.2f %.2f %.2f' % (first(unanchored == 0.0), first(let >= 1.0), first(shed >= 1.0), first(free > 0.0)))
    report.check('released within the slew, relocked within the relock', em.SLEW_S == 0.16 and em.RELOCK_S == 0.025
                 and modal.LET_GO.stop - modal.LET_GO.start == 2, '%g %g' % (em.SLEW_S, em.RELOCK_S))


ROSTER = [test_the_floor_runs_on_under_her_soles, test_a_grown_prop_bears_where_it_has_grown,
          test_her_grip_bounds_her_rhythm, test_told_to_lie_on_the_floor_she_lies_where_she_stands,
          test_told_to_scoot_on_the_floor_she_sits_on_her_fists_and_her_keys_scoot_her,
          test_her_legs_are_released_only_where_the_rest_of_what_bears_her_holds_her,
          test_her_release_lets_her_estimator_go_first_then_her_step_then_her_leg]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
