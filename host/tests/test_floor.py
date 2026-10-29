#!/usr/bin/env python3
"""Her floor: the plane under her soles, its grip as she feels it, the situations that cap her rhythm,
what she does on it."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report


def test_the_floor_runs_on_under_her_soles(report):
    """Level, the floor under each sole is the floor's height to the bit; pitched as her standing
    soles stand, a sole 0.3 m on along her way stands on it 0.3 tan(pitch) higher."""
    import numpy as np
    from machine import modal
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
    from machine import floor, physics
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
    own grip nothing moves; a sliding sole's grip is what it used, a holding one's at least that."""
    import numpy as np
    from machine import grip, latent, modal, motion, ortho, situation
    from machine.wbc import MU
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
        slid = grip.felt(slid, seen, frames, touch, np.array([0.5, 0.0, 0.0]), 550.0, 1.0)
    held_on = grip.felt(np.full(2, 0.01), np.zeros((2, grip.SEEN)), frames, touch, np.zeros(3), 550.0, grip.TRUST_S)
    report.check('a sole sliding at one share its last passes has that grip, a holding one at least it',
                 np.allclose(slid, 8.0 / 300.0) and (held_on >= 8.0 / 300.0).all(), '%s, %s' % (slid, held_on))


def test_told_to_lie_on_the_floor_she_lies_where_she_stands(report):
    """Said lie_floor on the flat floor: she crouches, tips back soft and lies - pelvis under 0.25 m,
    doing floor_rest, her drives limp (se2.LIMP, clamps zero over the bus) - quiet (joints under 30
    deg/s, no contact past 300 N after the settle), no fall reset for 14 s (se2.GROUNDED)."""
    import numpy as np
    from machine import physics, sprung, worker
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


def test_told_to_scoot_on_the_floor_she_drags_her_butt_forward(report):
    """Said scoot_floor on the flat floor: she sits on her butt and hands (pelvis under 0.25 m) and her strokes drag
    her 0.1 m along her way without a fall."""
    import numpy as np
    from machine import physics, sprung, worker
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    s = worker.session()
    g, d = s['g'], s['g']['world'].data
    worker.apply(s, {'scenario': 'flat'})
    for _ in range(int(round(0.5 / worker.DT))):
        worker.stepped(s)
    worker.apply(s, {'say': 'scoot_floor'})
    yaw = 2.0 * math.atan2(float(d.qpos[5]), float(d.qpos[3]))
    way = np.array([math.sin(yaw), 0.0, math.cos(yaw)])
    t0, sat, start = g['t'], False, None
    while g['t'] < t0 + 12.0 and s['fallen_at'] is None:
        worker.stepped(s)
        sat = sat or float(d.qpos[1]) < 0.25
        start = float(d.qpos[0:3] @ way) if start is None and str(sprung.doing(g)).startswith('stroke') else start
    went = math.nan if start is None else float(d.qpos[0:3] @ way) - start
    report.check('she sits on the floor and her strokes drag her 0.1 m forward, no fall',
                 sat and s['fallen_at'] is None and went >= 0.1,
                 'sat %s fallen %s went %.3f m' % (sat, s['fallen_at'], went))


def test_her_legs_are_released_only_where_her_other_supports_hold_her(report):
    """Her legs' release (intentional_limb_emancipation.lisp): asked where her soles are told nothing and something
    else bears her; allowed where her capture point lies inside the contacts that bear her besides her soles by the
    text's least margin - 0.2 m inside a seat's and hands' square, open; outside it, or with nothing but her soles,
    shut; at her walk's 0.7 m/s her Froude number shuts it to its exp(-(Fr/0.045)^2)."""
    import math
    import numpy as np
    from machine import emancipation as em
    weight, w = 305.5, np.array([0.0, 0.64, 0.0, 0.0, 0.36])
    report.check('her text asks her legs released where it tells her soles nothing and her seat and hands bear',
                 em.intent(w) == 1.0 and em.intent(np.array([1.0, 0, 0, 0, 0])) == 0.0
                 and em.intent(np.array([0.1, 0.9, 0, 0, 0])) == 0.0, str(em.intent(w)))
    square = [{'p': np.array([x, 0.0, z]), 'patch': 0} for x in (-0.2, 0.2) for z in (-0.2, 0.2)]
    still = {'vcom': np.zeros(3), 'com': np.array([0.0, 0.3, 0.0]), 'contacts': square}
    inside, _, inside_nm = em.gate(still, w, weight, 0.0)
    outside, _, outside_nm = em.gate(dict(still, com=np.array([0.3, 0.3, 0.0])), w, weight, 0.0)
    soled, _, _ = em.gate(dict(still, contacts=()), w, weight, 0.0)
    walking, parts, _ = em.gate(dict(still, vcom=np.array([0.0, 0.0, 0.7])), w, weight, 0.0)
    froude = 0.7 * 0.7 / (9.81 * em.LEG_M)
    report.check('0.2 m inside her seat and hands she may let her legs go; outside them or on her soles alone not',
                 inside == 1.0 and abs(inside_nm - 0.2 * weight) < 1e-9 and outside == 0.0 and outside_nm < 0.0
                 and soled == 0.0, '%.1f N m in, %.1f out' % (inside_nm, outside_nm))
    report.check('at her walk\'s 0.7 m/s her Froude number holds her gate to exp(-(Fr/0.045)^2)',
                 abs(parts[0] - math.exp(-(froude / em.FROUDE) ** 2)) < 1e-3 and walking < 0.2, '%.3f' % parts[0])


ROSTER = [test_the_floor_runs_on_under_her_soles, test_a_grown_prop_bears_where_it_has_grown,
          test_her_grip_bounds_her_rhythm, test_told_to_lie_on_the_floor_she_lies_where_she_stands,
          test_told_to_scoot_on_the_floor_she_drags_her_butt_forward,
          test_her_legs_are_released_only_where_her_other_supports_hold_her]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
