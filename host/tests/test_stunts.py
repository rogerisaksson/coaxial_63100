"""Integration tests for gynoid stunts, obstacle compliance, hole traversal, stumble reflex, and shoves."""
import math
import os
import sys

os.environ['OPENBLAS_NUM_THREADS'] = '1'

from tools.dev.focus import chosen
from gynoid_kit import Report

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _compiled():
    from machine import ortho
    with open(os.path.join(HERE, 'machine', 'gynoid_mesomorph_medium.lisp'), encoding='utf-8') as f:
        return ortho.compiled(f.read())


def _hit(g, bid):
    m, d = g['world'].model, g['world'].data
    return bool(d.ncon and (m.geom_bodyid[d.contact.geom[:d.ncon]] == bid).any())


def test_stunts_compile_and_execute(report):
    from machine import sprung
    tables, phases = _compiled()
    for name in ('sit_chair', 'rise_chair', 'lie_bed', 'rise_bed', 'lie_floor', 'rise_floor', 'hop_down_chair',
                 'hop_up_chair', 'stairs_walk', 'stairs_down', 'ice_walk', 'slope_walk', 'parry_shove', 'controlled_fall'):
        report.check('%s in compiled moves' % name, name in phases)
        g = sprung.laid()
        sprung.said(g, name, tables[name], phases[name] + ('rest',), at=g['t'] + 0.05, reset=True)
        for _ in range(100):
            sprung.passed(g, 0.001)
        report.check('%s steps 100ms without error' % name, sprung.tilt(g) < 25.0)


def test_wall_obstacle_collision_and_compliance(report):
    from machine import floor, latent, modal, sprung
    t, p = _compiled()
    g = sprung.laid()
    sprung.said(g, 'ballistic_walk', t['ballistic_walk'], p['ballistic_walk'] + ('rest',), at=0.01, reset=True)
    floor.place(g['world'], 'wall', z=0.5)
    wid, hit, damped = g['world'].model.body('wall').id, False, False
    for _ in range(2500):
        _, _, _, out = sprung.passed(g, 0.001)
        if _hit(g, wid):
            hit, damped = True, damped or abs(float(out[latent.SWAY][modal.DRIVE])) < 0.5
        if hit and damped:
            break
    report.check('wall contact detected', hit)
    report.check('drive damped upon wall collision', damped)
    report.check('stands upright against wall', sprung.tilt(g) < 25.0)


def test_tree_obstacle_collision_and_compliance(report):
    from machine import floor, sprung
    t, p = _compiled()
    g = sprung.laid()
    sprung.said(g, 'ballistic_walk', t['ballistic_walk'], p['ballistic_walk'] + ('rest',), at=0.01, reset=True)
    floor.place(g['world'], 'tree', z=0.5)
    tid, hit = g['world'].model.body('tree').id, False
    for _ in range(2500):
        sprung.passed(g, 0.001)
        if _hit(g, tid):
            hit = True
            break
    report.check('tree contact detected', hit)
    report.check('tree collision absorbed upright', sprung.tilt(g) < 25.0)


def test_stumble_reflex_boosts_limb_lift(report):
    from machine import floor, latent, modal, sprung
    t, p = _compiled()
    for kind, z_at in (('sill', 0.35), ('rug', 0.20)):
        g = sprung.laid()
        sprung.said(g, 'ballistic_walk', t['ballistic_walk'], p['ballistic_walk'] + ('rest',), at=0.01, reset=True)
        floor.place(g['world'], kind, z=z_at)
        bid, stumbled, boosted, hits, max_tilt = g['world'].model.body(kind).id, False, False, 0, 0.0
        for _ in range(2500):
            _, _, _, out = sprung.passed(g, 0.001)
            max_tilt, hits = max(max_tilt, sprung.tilt(g)), hits + _hit(g, bid)
            if g['stumble_until'] > g['t']:
                stumbled, boosted = True, boosted or float(out[latent.SWAY][modal.LIFT]) >= 2.5
        report.check('stumble detected on %s' % kind, stumbled and hits > 0)
        report.check('stumble on %s boosts lift (>= 2.5)' % kind, boosted)
        report.check('upright traversal over %s' % kind, max_tilt < 25.0 and sprung.tilt(g) < 25.0 and float(g['world'].data.qpos[2]) > 0.60)


def test_step_across_10cm_hole(report):
    from machine import floor, modal, sprung
    gym, g = sprung.entries(), sprung.laid(every=3)
    sprung._heard(g, gym, {'scenario': 'hole', 'say': '#204'})
    report.check('hole depth is 10cm', abs(floor.HOLE_M - 0.10) < 1e-6)
    gid, max_tilt, min_y, hits_b = g['world'].model.geom('slab_b').id, 0.0, 1.0, 0
    for _ in range(3200):
        sprung.passed(g, 0.001)
        max_tilt, min_y = max(max_tilt, sprung.tilt(g)), min(min_y, float(g['y'][modal.LOWEST][0]), float(g['y'][modal.LOWEST][1]))
        d = g['world'].data
        if d.ncon and (d.contact.geom[:d.ncon] == gid).any():
            hits_b += 1
    report.check('walks across 10cm hole onto far slab', max_tilt < 25.0 and sprung.tilt(g) < 25.0 and float(g['world'].data.qpos[2]) > 1.0 and min_y < -0.02 and hits_b > 0)


def test_moderate_and_severe_shove_reflexes(report):
    from machine import sprung
    gym, g = sprung.entries(), sprung.laid()
    sprung._heard(g, gym, {'push': (0.0, 0.0, 80.0), 'seconds': 0.12})
    for _ in range(400):
        sprung.passed(g, 0.001)
    report.check('80 N shove commands parry_shove', sprung.doing(g) in ('catch', 'restore', 'parry_shove'))
    report.check('80 N shove absorbed upright', sprung.tilt(g) < 15.0)
    g2 = sprung.laid()
    sprung._heard(g2, gym, {'push': (0.0, 0.0, 140.0), 'seconds': 0.12})
    for _ in range(400):
        sprung.passed(g2, 0.001)
    report.check('140 N shove commands controlled_fall', sprung.doing(g2) in ('cushion', 'land', 'controlled_fall'))
    report.check('140 N shove absorbed compliantly', sprung.tilt(g2) < 25.0)
    m, d = g2['world'].model, g2['world'].data
    q_ls, q_rs = (float(d.qpos[m.jnt_qposadr[m.joint(s).id]]) for s in ('left_shoulder', 'right_shoulder'))
    report.check('arms remain symmetric', abs(q_ls - q_rs) < 0.20)


def test_ballistic_start_trailing_leg_flexion_and_smooth_transitions(report):
    from machine import sprung
    t, p = _compiled()
    g = sprung.laid()
    sprung.said(g, 'ballistic_walk', t['ballistic_walk'], p['ballistic_walk'] + ('rest',), at=0.01, reset=True)
    knees, min_l, min_r = list(g['body'].knees), 999.0, 999.0
    for _ in range(1200):
        sprung.passed(g, 0.001)
        ql, qr = (float(g['world'].data.qpos[g['body'].qact][k]) for k in knees)
        min_l, min_r = min(min_l, ql), min(min_r, qr)
    report.check('left knee flexion (>= 4.0 deg)', math.degrees(min_l) >= 4.0)
    report.check('right knee flexion (>= 4.0 deg)', math.degrees(min_r) >= 4.0)
    report.check('ballistic start upright', sprung.tilt(g) < 15.0)


def test_random_mode_transitions_without_crashes(report):
    import random
    from machine import sprung
    gym = sprung.entries()
    for seed in (42, 7):
        rng, g, fell = random.Random(seed), sprung.laid(), None
        for m in [rng.randint(0, 9) for _ in range(6)]:
            sprung._heard(g, gym, {'say': m})
            for _ in range(800):
                sprung.passed(g, 0.001)
                if sprung.tilt(g) > sprung.DOWN_DEG:
                    fell = True
                    break
            if fell:
                break
        report.check('seed %d mode sequence completed upright' % seed, not fell and sprung.tilt(g) < 25.0)


def test_all_scenarios_instantiation_and_props(report):
    from coaxial.graphics import gynoid
    from machine import floor, physics
    world = physics.World()
    report.check('all 20 scenarios present', len(floor.SCENARIOS) == 20)
    for name in floor.SCENARIOS:
        world.scenario(name)
        pr = world.props()
        if name in ('flat', 'clear', 'none'):
            report.check('%s has 0 props' % name, len(pr) == 0)
        else:
            report.check('%s has props' % name, len(pr) > 0)
            for kind, pos, half, rot in pr:
                report.check('%s %s has ink' % (name, kind), kind in gynoid.PROP_INK)
                report.check('%s prop dims valid' % name, len(pos) == 3 and len(half) == 3 and len(rot) == 3 and len(rot[0]) == 3)


def test_stunts_executed_on_specialized_scenarios(report):
    from machine import sprung
    gym = sprung.entries()
    for scen, say, n, max_t, min_z, bname in (
        ('chair', 'sit_chair', 2500, 20.0, 0.50, 'chair'),
        ('ice', 'ice_walk', 1200, 25.0, None, 'ice'),
        ('slope', 'slope_walk', 1800, 25.0, None, 'slope_ramp'),
        ('bed', 'lie_bed', 2200, 30.0, 0.50, 'prop_bed'),
        ('stairs', 'stairs_walk', 1800, 25.0, None, 'step0'),
        ('stairs_down', 'stairs_down', 1800, 25.0, None, 'step0'),
    ):
        g = sprung.laid()
        sprung._heard(g, gym, {'scenario': scen})
        report.check('scenario set to %s' % scen, g.get('scenario') == scen)
        sprung._heard(g, gym, {'say': say})
        bid, hits, max_tilt = g['world'].model.body(bname).id, 0, 0.0
        for _ in range(n):
            sprung.passed(g, 0.001)
            max_tilt, hits = max(max_tilt, sprung.tilt(g)), hits + _hit(g, bid)
        if min_z is not None:
            report.check('walks forward to %s' % scen, float(g['world'].data.qpos[2]) >= min_z)
        report.check('%s contacts %s' % (say, scen), hits > 0)
        report.check('%s on %s completes upright' % (say, scen), sprung.tilt(g) < max_t and max_tilt < max_t)
    gf = sprung.laid()
    sprung._heard(gf, gym, {'scenario': 'flat', 'say': 'lie_floor'})
    for _ in range(2200):
        sprung.passed(gf, 0.001)
    report.check('lie_floor lowers compliantly', sprung.doing(gf) in ('kneel', 'lower', 'floor_rest', 'lie_floor'))


def test_gentle_walk_mode_4_ballistic_initiation_and_keylogger(report):
    from machine import sprung
    from terminal.views import humanoid_keys
    humanoid_keys.log_key('4', humanoid_keys._action_of('4'))
    with open(humanoid_keys.KEYLOG, 'r', encoding='utf-8') as f:
        report.check('keylog records key 4', "TTY: '4' -> say: 3 (#204 walk) [gentle walk]" in f.read())
    gym, g = sprung.entries(), sprung.laid()
    sprung._heard(g, gym, {'say': 3})
    knees, min_l, min_r, max_tilt = list(g['body'].knees), 999.0, 999.0, 0.0
    for _ in range(2500):
        sprung.passed(g, 0.001)
        ql, qr = (float(g['world'].data.qpos[g['body'].qact][k]) for k in knees)
        min_l, min_r, max_tilt = min(min_l, ql), min(min_r, qr), max(max_tilt, sprung.tilt(g))
    report.check('gentle walk left knee flexion', math.degrees(min_l) >= 4.0)
    report.check('gentle walk right knee flexion', math.degrees(min_r) >= 4.0)
    report.check('gentle walk upright (< 10 deg)', max_tilt < 10.0)
    report.check('gentle walk cruises (>= 0.5 m/s)', float(g['world'].data.qvel[2]) >= 0.5)


def test_quake_wasd_locomotion_and_curves(report):
    from machine import sprung
    by = {e[0]: e for e in sprung.entries(name='gynoid_mesomorph_medium')}
    for name in ('strafe_left', 'turn_left', 'turn_back_right'):
        report.check('%s exists' % name, name in by)
        g = sprung.laid()
        sprung.said(g, *by[name], at=g['t'])
        for _ in range(1500):
            sprung.passed(g, 0.001)
        report.check('%s executes (< 10 deg)' % name, sprung.tilt(g) < 10.0)
        if name == 'strafe_left':
            report.check('strafe left shifts +x', g['world'].data.qpos[0] > 0.1)


def test_drivetrain_capacity_and_stunt_torque_envelope(report):
    from machine import drives
    report.check('2 motor frames', set(drives.FRAMES.keys()) == {'U8', 'MN6007'})
    report.check('gearbox family B 1:30', drives.RATIO == 30.0 and 'B' in drives.BOXES)
    report.check('63100 100A inverter', 100 in drives.INVERTERS and drives.INVERTERS[100][1] == 100.0)
    for j in ('left_hip', 'right_hip', 'left_knee', 'right_knee'):
        report.check('%s peak >= 100 Nm' % j, drives.peak(j) >= 100.0)
    for j in ('left_knee', 'right_knee', 'left_ankle', 'right_ankle'):
        report.check('%s shock >= 120 Nm' % j, drives.shock(j) >= 120.0)
    for j in ('left_knee', 'left_hip', 'left_ankle'):
        report.check('%s flex bounded' % j, 0.0 < drives.flex(j) < 1e-3)


def test_shooter_circle_walk_and_continuous_turning(report):
    from machine import sprung
    by = {e[0]: e for e in sprung.entries(name='gynoid_mesomorph_medium')}
    report.check('circle_walk exists', 'circle_walk' in by)
    g = sprung.laid()
    sprung.said(g, *by['ballistic_walk'], at=0.01, reset=True)
    for _ in range(1000):
        sprung.passed(g, 0.001)
    sprung.said(g, *by['circle_walk'], at=g['t'] + 0.01, reset=False)
    yaw0, max_tilt = float(g['heading']), 0.0
    for _ in range(6000):
        sprung.passed(g, 0.001)
        max_tilt = max(max_tilt, sprung.tilt(g))
    report.check('circle walk swept >= 30 deg', abs(math.degrees(g['heading'] - yaw0)) >= 30.0)
    report.check('circle walk tilt < 8.0 deg', max_tilt < 8.0)
    report.check('circle walk zero falls', not (g['world'].data.qpos[2] < 0.3))


def test_glass_room_environment_and_door_push_through(report):
    import mujoco
    from machine import floor, physics, sprung
    physics.ROOM = 1.0
    by = {e[0]: e for e in sprung.entries(name='gynoid_mesomorph_medium')}
    report.check('glass_room in SCENARIOS', 'glass_room' in floor.SCENARIOS)
    g = sprung.laid()
    floor.scenario(g['world'], 'glass_room')
    m = g['world'].model
    bnames = [m.body(i).name for i in range(m.nbody)]
    report.check('glass walls in model', 'wall_left' in bnames and 'wall_right' in bnames)
    report.check('swinging door in model', 'door' in bnames)
    g['world'].data.qpos[0], g['world'].data.qpos[2] = 0.75, 2.0
    mujoco.mj_forward(m, g['world'].data)
    sprung.said(g, *by['ballistic_walk'], at=0.01, reset=True)
    max_ang, djnt = 0.0, m.joint('door').qposadr[0]
    for _ in range(4000):
        sprung.passed(g, 0.001)
        max_ang = max(max_ang, abs(float(g['world'].data.qpos[djnt])))
        if float(g['world'].data.qpos[2]) >= 4.05:
            break
    report.check('door pushed open (>= 20 deg)', math.degrees(max_ang) >= 20.0)
    report.check('exited glass room (z >= 4.0m)', float(g['world'].data.qpos[2]) >= 4.0)
    report.check('traversed doorway upright', sprung.tilt(g) < 10.0)


def test_full_duration_stunt_and_shove_trajectories(report):
    from machine import sprung
    gym = sprung.entries()
    pbody = {'chair': 'chair', 'bed': 'prop_bed', 'stairs': 'step0', 'stairs_down': 'step0'}
    for name, scen, actions, total_s in (
        ('chair_sit_rise', 'chair', ((0.01, {'say': 'sit_chair'}), (4.0, {'say': 'rise_chair'})), 7.0),
        ('bed_lie_rise', 'bed', ((0.01, {'say': 'lie_bed'}), (4.5, {'say': 'rise_bed'})), 8.0),
        ('floor_lie_rise', 'flat', ((0.01, {'say': 'lie_floor'}), (4.0, {'say': 'rise_floor'})), 7.0),
        ('chair_hop_down', 'chair', ((0.01, {'say': 'hop_down_chair'}),), 4.0),
        ('chair_hop_up', 'chair', ((0.01, {'say': 'hop_up_chair'}),), 4.0),
        ('stairs_up', 'stairs', ((0.01, {'say': 'stairs_walk'}),), 6.0),
        ('stairs_down', 'stairs_down', ((0.01, {'say': 'stairs_down'}),), 6.0),
        ('shove_walk_lat', 'flat', ((0.01, {'say': '#204'}), (2.0, {'push': (80.0, 0.0, 0.0), 'seconds': 0.12})), 5.0),
        ('shove_hard', 'flat', ((0.01, {'say': '#204'}), (2.0, {'push': (140.0, 0.0, 0.0), 'seconds': 0.12})), 5.0),
    ):
        g = sprung.laid(every=3)
        sprung._heard(g, gym, {'scenario': scen})
        bid = g['world'].model.body(pbody[scen]).id if scen in pbody else None
        idx, hits, max_tilt = 0, 0, 0.0
        for step in range(int(total_s * 1000)):
            while idx < len(actions) and step * 0.001 >= actions[idx][0]:
                sprung._heard(g, gym, actions[idx][1])
                idx += 1
            sprung.passed(g, 0.001)
            max_tilt, hits = max(max_tilt, sprung.tilt(g)), hits + (_hit(g, bid) if bid is not None else 0)
        report.check('%s full %.1fs upright' % (name, total_s),
                     sprung.tilt(g) < 15.0 and float(g['world'].data.qpos[1]) > 0.70 and max_tilt < 35.0
                     and (bid is None or hits > 0))


def test_extended_terrains_multidirectional_shoves_and_spasm_free_compliance(report):
    import numpy as np
    from machine import sprung
    gym = sprung.entries(name='gynoid_mesomorph_medium')
    for scen, actions, n_steps, bname in (
        ('slope_down', ((0, {'say': 'slope_walk'}),), 2000, 'slope_ramp'),
        ('cross_slope', ((0, {'say': 'slope_walk'}),), 2000, 'slope_ramp'),
        ('asphalt', ((0, {'say': '#204'}),), 1600, 'slope_top'),
        ('snow', ((0, {'say': '#204'}),), 1600, 'snow'),
        ('slip', ((0, {'say': 'ice_walk'}),), 1500, 'slip'),
        ('rug', ((0, {'say': '#204'}),), 1600, 'rug'),
        ('mattress', ((0, {'say': 'lie_floor'}), (2800, {'say': 'rise_floor'})), 5200, 'prop_bed'),
    ):
        g = sprung.laid(every=3)
        sprung._heard(g, gym, {'scenario': scen})
        bid, idx, max_tilt, max_lead, hits = g['world'].model.body(bname).id, 0, 0.0, 0.0, 0
        for step in range(n_steps):
            while idx < len(actions) and step >= actions[idx][0]:
                sprung._heard(g, gym, actions[idx][1])
                idx += 1
            sprung.passed(g, 0.001)
            max_tilt, hits = max(max_tilt, sprung.tilt(g)), hits + _hit(g, bid)
            max_lead = max(max_lead, float(np.max(np.abs(g['ref_q'] - g['world'].data.qpos[g['body'].qact]))))
        report.check('%s contacts %s' % (scen, bname), hits > 0)
        report.check('%s completes upright without spasm' % scen, sprung.tilt(g) < 20.0 and max_tilt < 30.0 and max_lead < 0.60)

    for label, init_cmd, force in (
        ('fwd_shove', {'say': '#204'}, (0.0, 0.0, 75.0)),
        ('back_shove', {'say': '#204'}, (0.0, 0.0, -75.0)),
        ('left_shove', {'say': 'turn_left'}, (75.0, 0.0, 0.0)),
        ('right_shove', {'say': '#205'}, (-75.0, 0.0, 0.0)),
        ('diag_shove', {'twist': (0.69, 0.0, 0.0)}, (55.0, 0.0, -55.0)),
    ):
        g = sprung.laid(every=3)
        sprung._heard(g, gym, init_cmd)
        for _ in range(900):
            sprung.passed(g, 0.001)
        sprung._heard(g, gym, {'push': force, 'seconds': 0.12})
        report.check('%s commands parry_shove' % label, g.get('active_entry') == 'parry_shove' and not g.get('_twist_active'))
        for _ in range(1200):
            sprung.passed(g, 0.001)
        report.check('%s parried upright' % label, sprung.tilt(g) < 20.0 and float(g['world'].data.qpos[1]) > 0.72)

    g_ag, g_tp = sprung.laid(every=3), sprung.laid(every=3)
    sprung._heard(g_ag, gym, {'say': '#204'})
    sprung._heard(g_tp, gym, {'twist': (0.69, 0.0, 0.0)})
    for _ in range(800):
        sprung.passed(g_ag, 0.001)
        sprung.passed(g_tp, 0.001)
    report.check('say and twist share twist_actual', abs(g_ag['twist_actual'][0] - g_tp['twist_actual'][0]) < 1e-4 and g_ag['twist_actual'][0] > 0.25)


ROSTER = [
    test_stunts_compile_and_execute, test_all_scenarios_instantiation_and_props,
    test_stunts_executed_on_specialized_scenarios, test_gentle_walk_mode_4_ballistic_initiation_and_keylogger,
    test_wall_obstacle_collision_and_compliance, test_tree_obstacle_collision_and_compliance,
    test_stumble_reflex_boosts_limb_lift, test_step_across_10cm_hole, test_moderate_and_severe_shove_reflexes,
    test_ballistic_start_trailing_leg_flexion_and_smooth_transitions, test_random_mode_transitions_without_crashes,
    test_quake_wasd_locomotion_and_curves, test_shooter_circle_walk_and_continuous_turning,
    test_glass_room_environment_and_door_push_through, test_drivetrain_capacity_and_stunt_torque_envelope,
    test_full_duration_stunt_and_shove_trajectories,
    test_extended_terrains_multidirectional_shoves_and_spasm_free_compliance,
]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())

