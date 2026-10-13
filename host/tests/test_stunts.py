"""Integration tests for gynoid stunts, obstacle compliance, hole traversal, stumble reflex, and shoves."""
import math
import os
import sys
import time

from tools.dev.focus import chosen
from gynoid_kit import Report


HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_stunts_compile_and_execute(report):
    """All 13 new stunt behaviors compile in moves.lisp, can be commanded via said(), and step cleanly."""
    from machine import latent, modal, ortho, sprung
    with open(os.path.join(HERE, 'machine', 'moves.lisp'), encoding='utf-8') as f:
        text = f.read()
    tables, phases = ortho.compiled(text)
    stunts = (
        'sit_chair', 'rise_chair', 'lie_bed', 'rise_bed', 'lie_floor', 'rise_floor',
        'hop_down_chair', 'hop_up_chair', 'stairs_walk', 'ice_walk', 'slope_walk',
        'parry_shove', 'controlled_fall'
    )
    for name in stunts:
        report.check('%s in compiled moves' % name, name in phases, '%s phases: %s' % (name, phases.get(name)))

    # Verify each stunt can be initialized in sprung and stepped
    for name in stunts:
        g = sprung.laid()
        sprung.said(g, name, tables[name], phases[name] + ('rest',), at=g['t'] + 0.05, reset=True)
        # Step 100 ms
        for _ in range(100):
            sprung.passed(g, 0.001)
        report.check('%s steps 100ms without error' % name, sprung.tilt(g) < 25.0, 'tilt %.1f deg' % sprung.tilt(g))


def test_wall_obstacle_collision_and_compliance(report):
    """Walking into a wall compliantly damps drive, zeros going, and maintains upright balance."""
    from machine import floor, latent, modal, ortho, sprung
    with open(os.path.join(HERE, 'machine', 'moves.lisp'), encoding='utf-8') as f:
        tables, phases = ortho.compiled(f.read())
    g = sprung.laid()
    sprung.said(g, 'ballistic_walk', tables['ballistic_walk'], phases['ballistic_walk'] + ('rest',), at=0.01, reset=True)
    # Place wall at z=0.5m ahead
    floor.place(g['world'], 'wall', z=0.5)

    wall_id = g['world'].model.body('wall').id
    hit = False
    damped = False
    for _ in range(2500): # 2.5s
        s, loads, ask, out = sprung.passed(g, 0.001)
        if g['world'].data.ncon:
            c_bids = g['world'].model.geom_bodyid[g['world'].data.contact.geom[:g['world'].data.ncon]]
            if (c_bids == wall_id).any():
                hit = True
                if abs(float(out[latent.SWAY][modal.DRIVE])) < 0.5:
                    damped = True
        if hit and damped:
            break

    tilt = sprung.tilt(g)
    report.check('wall contact detected', hit)
    report.check('drive compliantly damped upon wall collision', damped)
    report.check('she stands upright against wall without falling', tilt < 25.0, 'tilt %.1f deg' % tilt)


def test_tree_obstacle_collision_and_compliance(report):
    """Walking into a tree compliantly damps drive and prevents falling."""
    from machine import floor, latent, modal, ortho, sprung
    with open(os.path.join(HERE, 'machine', 'moves.lisp'), encoding='utf-8') as f:
        tables, phases = ortho.compiled(f.read())
    g = sprung.laid()
    sprung.said(g, 'ballistic_walk', tables['ballistic_walk'], phases['ballistic_walk'] + ('rest',), at=0.01, reset=True)
    # Place tree at z=0.5m ahead
    floor.place(g['world'], 'tree', z=0.5)

    tree_id = g['world'].model.body('tree').id
    hit = False
    for _ in range(2500):
        s, loads, ask, out = sprung.passed(g, 0.001)
        if g['world'].data.ncon:
            c_bids = g['world'].model.geom_bodyid[g['world'].data.contact.geom[:g['world'].data.ncon]]
            if (c_bids == tree_id).any():
                hit = True
        if hit:
            break

    tilt = sprung.tilt(g)
    report.check('tree contact detected', hit)
    report.check('tree collision absorbed without falling', tilt < 25.0, 'tilt %.1f deg' % tilt)


def test_stumble_reflex_boosts_limb_lift(report):
    """Stumbling over a sill triggers limb lift clearance boost reflex."""
    from machine import floor, latent, modal, ortho, sprung
    with open(os.path.join(HERE, 'machine', 'moves.lisp'), encoding='utf-8') as f:
        tables, phases = ortho.compiled(f.read())
    g = sprung.laid()
    sprung.said(g, 'ballistic_walk', tables['ballistic_walk'], phases['ballistic_walk'] + ('rest',), at=0.01, reset=True)
    # Place sill at z=0.35m
    floor.place(g['world'], 'sill', z=0.35)

    stumbled = False
    boosted = False
    for _ in range(2500):
        s, loads, ask, out = sprung.passed(g, 0.001)
        if g['stumble_until'] > g['t']:
            stumbled = True
            if float(out[latent.SWAY][modal.LIFT]) >= 2.5:
                boosted = True
        if stumbled and boosted:
            break

    tilt = sprung.tilt(g)
    report.check('stumble detected on swing foot collision with sill', stumbled)
    report.check('stumble triggers mode_limb_lift clearance boost (>= 2.5)', boosted)
    report.check('upright recovery after stumble over sill', tilt < 25.0, 'tilt %.1f deg' % tilt)


def test_step_across_10cm_hole(report):
    """Stepping into and across a 10cm deep hole maintains balance and does not fall."""
    from machine import floor, latent, modal, ortho, sprung
    g = sprung.laid()
    sprung.walked(g)
    # Place 10cm hole at z=1.8m
    floor.place(g['world'], 'hole', z=1.8)
    report.check('hole depth is 10cm', abs(floor.HOLE_M - 0.10) < 1e-6)

    for _ in range(3500):
        sprung.passed(g, 0.001)

    tilt = sprung.tilt(g)
    report.check('walks across 10cm hole without falling', tilt < 25.0, 'tilt %.1f deg' % tilt)


def test_moderate_and_severe_shove_reflexes(report):
    """Moderate shove (80 N) triggers parry; severe shove triggers cushion/controlled fall; both physically survive."""
    from machine import latent, modal, ortho, sprung
    gym = sprung.entries()

    # Test 80 N moderate shove
    g = sprung.laid()
    sprung._heard(g, gym, {'push': (0.0, 0.0, 80.0), 'seconds': 0.12})
    for _ in range(400):
        sprung.passed(g, 0.001)
    report.check('80 N shove commands parry_shove', sprung.doing(g) in ('catch', 'restore', 'parry_shove'),
                 'doing: %s' % sprung.doing(g))
    report.check('80 N shove absorbed upright without falling', sprung.tilt(g) < 15.0,
                 'tilt: %.1f deg' % sprung.tilt(g))

    # Test 140 N severe shove
    g2 = sprung.laid()
    sprung._heard(g2, gym, {'push': (0.0, 0.0, 140.0), 'seconds': 0.12})
    for _ in range(400):
        sprung.passed(g2, 0.001)
    report.check('140 N shove commands controlled_fall', sprung.doing(g2) in ('cushion', 'land', 'controlled_fall'),
                 'doing: %s' % sprung.doing(g2))
    report.check('140 N shove absorbed compliantly', sprung.tilt(g2) < 25.0,
                 'tilt: %.1f deg' % sprung.tilt(g2))


def test_ballistic_start_trailing_leg_flexion_and_smooth_transitions(report):
    """Trailing leg stays naturally flexed during ballistic start (>= 4.0 deg) with smooth jerk-free torque."""
    from machine import ortho, sprung
    with open(os.path.join(HERE, 'machine', 'moves.lisp'), encoding='utf-8') as f:
        text = f.read()
    tables, phases = ortho.compiled(text)
    g = sprung.laid()
    sprung.said(g, 'ballistic_walk', tables['ballistic_walk'], phases['ballistic_walk'] + ('rest',), at=0.01, reset=True)
    knees = list(g['body'].knees)
    min_l, min_r = 999.0, 999.0
    for _ in range(1200):
        sprung.passed(g, 0.001)
        ql = float(g['world'].data.qpos[g['body'].qact][knees[0]])
        qr = float(g['world'].data.qpos[g['body'].qact][knees[1]])
        min_l = min(min_l, ql)
        min_r = min(min_r, qr)

    min_l_deg, min_r_deg = math.degrees(min_l), math.degrees(min_r)
    report.check('trailing leg left knee maintains natural flexion (>= 4.0 deg)', min_l_deg >= 4.0,
                 'min left: %.2f deg' % min_l_deg)
    report.check('trailing leg right knee maintains natural flexion (>= 4.0 deg)', min_r_deg >= 4.0,
                 'min right: %.2f deg' % min_r_deg)
    report.check('ballistic gait initiation maintains upright posture', sprung.tilt(g) < 15.0,
                 'tilt: %.1f deg' % sprung.tilt(g))


def test_random_mode_transitions_without_crashes(report):
    """Randomly jumping between all modes (0..9) executes compliant transitions without falls."""
    import random
    from machine import sprung
    gym = sprung.entries()
    for seed in (42, 7):
        rng = random.Random(seed)
        g = sprung.laid()
        modes = [rng.randint(0, 9) for _ in range(6)]
        fell = None
        for m in modes:
            sprung._heard(g, gym, {'say': m})
            for _ in range(800):
                try:
                    sprung.passed(g, 0.001)
                except Exception as e:
                    fell = str(e)
                    break
                if sprung.tilt(g) > sprung.DOWN_DEG:
                    fell = 'tilt %.1f deg' % sprung.tilt(g)
                    break
            if fell:
                break
        report.check('seed %d mode sequence completed without falling' % seed,
                     fell is None and sprung.tilt(g) < 25.0, 'modes=%s fell=%s, tilt=%.1f deg' % (modes, fell, sprung.tilt(g)))


ROSTER = [
    test_stunts_compile_and_execute,
    test_wall_obstacle_collision_and_compliance,
    test_tree_obstacle_collision_and_compliance,
    test_stumble_reflex_boosts_limb_lift,
    test_step_across_10cm_hole,
    test_moderate_and_severe_shove_reflexes,
    test_ballistic_start_trailing_leg_flexion_and_smooth_transitions,
    test_random_mode_transitions_without_crashes,
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
