#!/usr/bin/env python3
"""Gynoid body on fantasy boards: garments, leg IK, rigs, skeleton and static compiler."""
import sys

from structure_kit import Report
from tools.dev.focus import chosen
from machine.world import physics

physics.ENVELOPE = 0.0


def test_dressed_or_bare(report):
    """Bare body: worn garments/hair/soles gone, feet plated, four quick-release bands."""
    from coaxial.graphics import gynoid, lit, sneaker
    dressed, bare = gynoid.body(), gynoid.body(dressed=False)
    worn = [p[0] for p in dressed.parts if gynoid._worn(p[0])]
    bands = [p[0] for p in bare.parts if p[0].startswith('release_')]
    report.check('bare, every worn part gone, a band a quick-release, nothing else',
                 [p[0] for p in bare.parts if not p[0].startswith('release_')]
                 == [p[0] for p in dressed.parts if not gynoid._worn(p[0])]
                 and len(bands) == 4
                 and {'cloth_tee', 'cloth_left_sleeve', 'hair', 'hair_fall',
                      'left_sole'} <= set(worn),
                 '%d of %d worn, %d bands' % (len(worn), len(dressed.parts), len(bands)))
    feet = [i for i, p in enumerate(bare.parts) if p[0].split('_', 1)[-1] in ('foot', 'toes')]
    report.check('feet plated bare, sneakers dressed',
                 all((bare.materials[slice(*bare.spans[i])] == lit.PLATE).all() for i in feet)
                 and all((dressed.materials[slice(*dressed.spans[i])]
                          == lit.paint(sneaker.SNEAKER)).all() for i in feet))


def test_the_robots_views_draw(report):
    """Mechanism and actuator views draw, no spur or bevel part on a held wrist."""
    from coaxial.graphics import gynoid
    for see in ('mechanism', 'actuators'):
        seen = gynoid.body(see=see)
        report.check('%s drawn, nothing on a held wrist' % see,
                     len(seen.parts) > 20 and not any(
                         p[0].startswith(('bevel', 'pinion', 'spur')) and p[0].endswith('_wrist')
                         for p in seen.parts), '%d parts' % len(seen.parts))


def _weyl(index, dim):
    primes = (2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61)
    return ((index + 1) * (primes[dim % len(primes)] ** 0.5)) % 1.0


def _span(lo, hi, u):
    return lo + (hi - lo) * u


def test_a_virtual_body_is_told(report):
    """VIRTUAL: joints slew to poses told, read back, run the type's walk routine."""
    from machine import Machine
    from machine.rig.modes import VIRTUAL
    from machine.rig.routines import TYPES
    from machine.rig.virtual import VirtualJoint
    expected = sum(len(s.actuators) for s in TYPES['humanoid'].body)
    body = Machine.discover('humanoid', execution_mode=VIRTUAL)
    report.check('virtual joints fitted bus by bus as the type names them',
                 len(body.actuators) == expected
                 and all(isinstance(a, VirtualJoint) for a in body.actuators.values())
                 and body.actuators['pelvis'].node.name == 'V1_1'
                 and body.actuators['right_foot'].node.name == 'V5_4',
                 {n: a.node.name for n, a in list(body.actuators.items())[:5]})
    body.arm()
    told = {j: 4.0 * (k % 5 - 2) for k, j in enumerate(body.actuators)}
    body.loop.write(**told)
    for _ in range(12):
        got = body.loop.step(0.04)
    worst = max(abs(got[j + '.deg'] - told[j]) for j in body.actuators)
    report.check('a pose written, each joint reads back where it was told', worst < 1e-9,
                 '%.3g deg' % worst)
    dt = 0.05
    step_deg = body.actuators['left_knee'].deg_s * dt
    body.loop.write(left_knee=told['left_knee'] + 1.6 * step_deg)
    got = body.loop.step(dt)
    got = body.loop.step(dt)
    moved = got['left_knee.deg'] - told['left_knee']
    report.check('a joint slews at its rate over a control step',
                 abs(moved - step_deg) < 1e-6, '%.3f deg' % moved)
    out = body.run('0 run=walk times=1')
    report.check('the type\'s walk routine runs on them', out.status == 'done', out.status)
    body.disarm()


def test_a_leg_by_its_foot(report):
    """figure.leg answers the six joints that put a foot where `foot_of` finds it across the robot's travel."""
    import math
    from machine.body import drives, figure
    hy = math.radians(0.5 * drives.TRAVEL['hip_yaw'][1])
    hr = math.radians(0.4 * drives.TRAVEL['spine_roll'][1])
    kn_hi = math.radians(0.75 * drives.TRAVEL['knee'][1])
    an_lo, an_hi = (math.radians(0.5 * v) for v in drives.TRAVEL['ankle'])
    ar = math.radians(0.8 * drives.TRAVEL['ankle_roll'][1])
    h_stand = figure.standing()
    reach = 0.25 * (figure.THIGH + figure.SHANK)
    worst = 0.0
    for i in range(200):
        angles = (_span(-hy, hy, _weyl(i, 0)), _span(-hr, hr, _weyl(i, 1)),
                  _span(-2.0 * hy, hy, _weyl(i, 2)), _span(0.05, kn_hi, _weyl(i, 3)),
                  _span(an_lo, an_hi, _weyl(i, 4)), _span(-ar, ar, _weyl(i, 5)))
        sign = 1.0 if i % 2 == 0 else -1.0
        pelvis = (_span(-reach, reach, _weyl(i, 6)),
                  _span(0.85 * h_stand, 1.15 * h_stand, _weyl(i, 7)),
                  _span(-reach, reach, _weyl(i, 8)))
        turn = figure.mul(figure.mul(figure.ry(_span(-hy, hy, _weyl(i, 9))),
                                     figure.rx(_span(-hr, hr, _weyl(i, 10)))),
                          figure.rz(_span(-hr, hr, _weyl(i, 11))))
        ankle, foot = figure.foot_of(sign, pelvis, turn, angles)
        got = figure.leg(sign, pelvis, turn, ankle, foot)
        worst = max(worst, max(abs(a - b) for a, b in zip(got, angles)))
    report.check('a leg\'s six joints from its foot, 200 poses round trip', worst < 1e-9,
                 '%.2g rad' % worst)
    report.check('the figure\'s segment shares sum to 1 at %.1f kg' % figure.MASS_KG, abs(
        sum(seg[5] for seg in figure.SEGMENTS) - 1.0) < 1e-3,
        sum(seg[5] for seg in figure.SEGMENTS))


def test_a_body_with_mass_is_fitted(report):
    """DYNAMIC: each of the robot's joints a drive on a body with mass, its pose read beside them."""
    from machine import Machine
    from machine.body import figure
    from machine.rig.modes import DYNAMIC
    from machine.rig.dynamic import DriveJoint
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    report.check('all %d drives fitted bus by bus, the robot\'s pose read beside them' % len(figure.JOINTS),
                 len(body.actuators) == len(figure.JOINTS)
                 and all(isinstance(a, DriveJoint) for a in body.actuators.values())
                 and body.actuators['spine'].node.name == 'D1_1'
                 and [n.name for n in body.others] == ['pelvis'],
                 {n: a.node.name for n, a in list(body.actuators.items())[:4]})
    body.close()


def test_the_robots_skeleton_collides(report):
    """Skeleton: bone geoms at unchanged mass, clear at stand/squat, meeting femur past stop."""
    import importlib
    import math
    from machine.body import drives, figure, mjcf, skeleton
    from machine.world import physics
    mujoco = importlib.import_module('mujoco')
    was = physics.SKELETON
    try:
        physics.SKELETON = 0.0
        bare = mujoco.MjModel.from_xml_string(mjcf.mjcf())
        physics.SKELETON = 1.0
        model = mujoco.MjModel.from_xml_string(mjcf.mjcf())
    finally:
        physics.SKELETON = was
    report.check('the robot\'s skeleton is %d geoms more, its weight as it was' % (model.ngeom - bare.ngeom),
                 model.ngeom > bare.ngeom
                 and abs(sum(model.body_mass) - sum(bare.body_mass)) < 1e-3,
                 '%.4f kg apart' % abs(sum(model.body_mass) - sum(bare.body_mass)))
    report.check('standing and in the squat it touches nothing',
                 not skeleton.overlapping(model, figure.stand())
                 and not skeleton.overlapping(model, figure.SQUAT))
    d = mujoco.MjData(model)
    d.qpos[1] = 2.0 * figure.standing()
    d.qpos[model.joint('left_knee').qposadr[0]] = math.radians(drives.TRAVEL['knee'][1] + 5.0)
    mujoco.mj_forward(model, d)
    met = {tuple(sorted((model.body(model.geom_bodyid[d.contact[i].geom1]).name,
                         model.body(model.geom_bodyid[d.contact[i].geom2]).name)))
           for i in range(d.ncon)}
    drums = (('left_ankle_drum', 'left_ankle_roll_drum') if len(skeleton.HUNG['thigh'][0]) > 1
             else ('left_ankle_roll_drum',))
    report.check("folded past its stop, the ankle's drives meet the femur, the skins not each other",
                 all((d_name, 'left_thigh_bone') in met for d_name in drums)
                 and ('left_shank', 'left_thigh') not in met, '%s' % sorted(met)[:4])


def test_each_drive_turns_from_its_gearbox(report):
    """Each drive turns what it drives about its drum axis from its gearbox end."""
    from tools.sim import fit
    wrong = [(j, w) for j, _what, w in fit.outputs() if w]
    report.check('every drive turns what it drives from its gearbox, on its axis', not wrong, '%s' % wrong)


def test_each_drive_is_held(report):
    """Each drive structurally mounted to a bone, gimbal, boom or trunk frame."""
    from tools.sim import held
    loose = [j for j, by in held.held_by().items() if not by]
    report.check('every drive held', not loose, '%s held by nothing' % loose)


def test_a_gearbox_has_play(report):
    """BOXED: rotor crosses backlash free, winds up `tau / wind`, crosses play on reversal."""
    from machine.body import drives, figure
    from machine.world import physics
    from machine.body.figure import JOINTS
    i = next(JOINTS.index(j) for j in ('waist', 'neck', 'left_elbow') if not drives.passive(j))
    physics.BOXED = 1.0
    try:
        _gearbox_has_play(report, physics, figure, i)
    finally:
        physics.BOXED = 0.0


def _gearbox_has_play(report, physics, figure, i):
    world = physics.World()
    world.reset(figure.stand())
    world.gains[i] = 0.0
    gap, wind, tau = float(world.boxes.gap[i]), float(world.boxes.k[i]), 3.0
    q0, now = float(world.data.qpos[world.qadr[i]]), [0.0]
    world.clock = lambda: now[0]

    def run(seconds, feed):
        world.feed[i] = feed
        rows = []
        for _ in range(int(round(seconds / physics.STEP_S))):
            now[0] += physics.STEP_S
            world.advance()
            rows.append((now[0], float(world.boxes.delta[i]),
                         float(world.data.qpos[world.qadr[i]]) - q0))
        return rows
    on = run(0.3, tau)
    across = next((t for t, d, _q in on if d >= gap), None)
    still = max(abs(q) for t, _d, q in on if across is None or t <= across)
    report.check('the link stands while the rotor crosses the play, 10-60 ms',
                 across is not None and 0.01 <= across <= 0.06 and still < 1e-4, '%s s, %.2e rad' % (across, still))
    wound = gap + tau / wind
    report.check('past the play the box winds up tau / wind and the link turns',
                 abs(on[-1][1] - wound) < 0.3 * wound and on[-1][2] > 0.005,
                 '%.1f vs %.1f mrad, link %.1f mrad' % (1e3 * on[-1][1], 1e3 * wound, 1e3 * on[-1][2]))
    back = run(0.3, -tau)
    cross = next((t for t, d, _q in back if d <= -gap), None)
    at = next((q for t, _d, q in back if cross is not None and t >= cross), None)
    report.check('reversed, the rotor crosses the play again before the link turns back',
                 cross is not None and 0.01 <= cross - on[-1][0] <= 0.1 and at is not None
                 and at >= on[-1][2] - 1e-6, '%s s' % (None if cross is None else round(cross - on[-1][0], 3)))


def test_every_build_runs_the_robot(report):
    """Each non-default build (`mechatronics.robots`) runs `smoke.py` under COAXIAL_ROBOT."""
    import os
    import subprocess
    from machine.body import mechatronics
    host = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    others = [n for n in mechatronics.robots() if n != mechatronics.ROBOT]
    report.check('a second build beside its own', bool(others), ', '.join(mechatronics.robots()))
    for other in others:
        run = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(host, 'tools', 'dev', 'smoke.py')],
                             cwd=host, env=dict(os.environ, COAXIAL_ROBOT=other, PYTHONPATH=host),
                             capture_output=True, text=True, timeout=600)
        report.check('%s runs: every module imported, its first seconds on the loop' % other,
                     run.returncode == 0 and 'smoke: clean' in run.stdout, (run.stdout + run.stderr)[-300:])


def _s(words):
    return set(words.split())


def test_the_robots_gait_is_stated_on_it(report):
    """Declarative `.toml` specs hold physical construction only, cache/recompile on edit."""
    import math, os, tempfile, tomllib
    from machine.body import build, drives, figure, gait, mechatronics, servos
    with open(os.path.join(mechatronics.ROBOTS, mechatronics.BASE + '.toml'), 'rb') as f:
        raw = tomllib.load(f)
    banned = _s('servo wind series gait service pack_v cooling clamped rigid_play_deg '
                'ahead_wind ahead_play mount_k_w off_mount_k_w passive_limits')
    lib, cf, wf, mf = (mechatronics.library(), mechatronics.crafts(),
                       mechatronics.environment(), mechatronics.materials())
    bus_ban = _s('bus_hz law_div thermal_s spin_s stall_s threads_a_limb eso_bw_share '
                 'observe_x rate_s play_nm wear_window_s wear_trend_windows')
    ck = {k for c in cf.values() if isinstance(c, dict) for k in tuple(c) + tuple(c.get('gimbal', ()))}
    craft_ban = ck & _s('step_s arena drive_load k_thrust k_drag inertia_xx stiffness damping '
                        'turn_bandwidth alpha_run_deg climb_deg harrier_deg tictoc_hz tictoc_deg '
                        'most_deg nose_m_s head_tip_m_s drive_rad_s profile_drag_share '
                        'induced_drag_share cyclic_drag_share cd_induced')
    lib_ban = ({k for m in lib['motors'].values() for k in m} & _s('rotor_kgm2 stator_j_k')
               | {k for b in lib['gearboxes'].values() for k in b}
               & _s('stiffness_nm_rad stiffness_nm_rad_m2 drag_nm_m3 shock_pa fill_share length_share '
                    'input_j_share zeta friction_share mesh_rad_s')
               | {k for v in lib['inverters'].values() for k in v}
               & _s('laminate_k_w laminate_j_k into_k_w loop_hz v_frac envelope dry_loss_24v_w dry_loss_w_per_v')
               | (set(lib['inverters']['coaxial_63100']['heatsink']) & _s('fan_temp_on fan_temp_full fan_idle_duty')))
    dyn_mods = ('machine/world/physics.py', 'machine/world/stiction.py', 'machine/body/gearbox.py',
                'machine/body/mjcf.py')
    dyn_direct = [m for m in dyn_mods
                  if 'mechatronics' in open(os.path.join(os.path.dirname(os.path.dirname(__file__)), *m.split('/')),
                                            encoding='utf-8').read()]
    report.check('library, humanoid and craft .toml files hold physical construction only',
                 _s('bus stops boards mounts linkage skeleton garments elastomers') <= set(raw)
                 and not (banned & set(raw)) and not (_s('squat stand stand_knee') & set(raw['figure']))
                 and not (bus_ban & set(raw['bus']))
                 and not (_s('census_px window_px quantum p1 p2 texture unique') & set(raw['eyes']))
                 and not (_s('flex_at flex_deg margin') & set(raw['shell']))
                 and not (_s('settle_s damp soft width_m') & set(raw['elastomers']['sole']))
                 and not (_s('stop_s hold_n') & set(raw['garments']['lace']))
                 and not (_s('stop_s d_nms_rad') & (set(raw['garments']['hem']) | set(raw['garments']['hair'])))
                 and not (_s('stiffness damping') & set(raw['passive']['gripper']))
                 and not (_s('step_s substeps cone impratio stand_m stand_solref_s touched_n '
                             'simulation harness load_s joint_damping') & set(wf))
                 and 'discs' not in wf['weather'] and not craft_ban and not lib_ban and not dyn_direct
                 and 'g_m_s2' in wf and 'solids' in mf and not (_s('stiffness rad_s') & set(raw['toes']))
                 and lib['gearboxes']['A']['ratio'] == lib['gearboxes']['B']['ratio']
                 and mechatronics.laminate_j_k(lib['inverters']['coaxial_63050'], mf) == 7.8
                 and 'stack_on' in raw['boards']['ankle'] and 'stack_on' in raw['boards']['head'])
    with tempfile.NamedTemporaryFile('w', suffix='.toml', delete=False, encoding='utf-8') as tf:
        tf.write('mass = 2.0\n')
        probe = tf.name
    calls = []
    chk = lambda: (calls.append(1), mechatronics._toml(probe)['mass'])[1]
    try:
        v1 = mechatronics.compiled('_probe_test', (probe,), chk)
        v2 = mechatronics.compiled('_probe_test', (probe,), chk)
        with open(probe, 'w', encoding='utf-8') as tf:
            tf.write('mass = 3.5\n')
        v3 = mechatronics.compiled('_probe_test', (probe,), chk)
    finally:
        for p in (probe, os.path.join(mechatronics.CACHE, '_probe_test.json')):
            if os.path.exists(p):
                os.unlink(p)
        mechatronics._COMPILED.pop('_probe_test', None)
    pk, sq = raw['pack'], figure.SQUAT
    sq_turn = figure.rx(math.radians(-(sq['left_hip'] + sq['left_knee'] + sq['left_ankle'])))
    ank_rel, _ = figure.foot_of(1.0, (0.0, 0.0, 0.0), sq_turn, [math.radians(sq['left' + k]) for k in figure.LEG])
    sq_com = figure.com(sq, (0.0, figure.ANKLE_H - ank_rel[1], -ank_rel[2]), sq_turn)
    report.check('meta-compiler caches first run and recompiles on parameter change',
                 (v1, v2, v3, len(calls)) == (2.0, 2.0, 3.5, 2)
                 and drives.PACK_V == float(pk['series'] * pk['cell_v_min'])
                 and 0.4 * figure.MASS_KG < build.mass() < figure.MASS_KG
                 and drives.SERIES['ankle'] > 0.0 and drives.TRACK == 0.0
                 and len(servos.SERVO) == len(drives.STACKS) and len(drives.WIND) == len(drives.STACKS)
                 and abs(sq_com[2] - 0.5 * (figure.BALL - figure.HEEL)) < 1e-3)
    report.check('the robot\'s leg and mass are those its gait is stated at', gait.SIMILAR == 1.0 and gait.MASSY == 1.0,
                 'leg %r, mass %r of its own' % (gait.SIMILAR, gait.MASSY))


def test_the_robots_tiers_by_the_machine(report):
    """Execution tiers fit core counts (`modes.afforded`/`chosen`); `worker.told` reports headroom."""
    from machine.rig import modes
    from machine.process import worker
    fits = {name: ok for name, _c, ok in modes.afforded(16, 20)}
    report.check('on 16 cores 20 boards fit in-process and native, not Renode; in-process chosen, the one wired',
                 fits['in-process'] and fits['native'] and not fits['emulated'] and modes.chosen(16, 20) == 'in-process',
                 modes.told(16, 20))
    few = {name: ok for name, _c, ok in modes.afforded(2, 20)}
    report.check('on 2 cores in-process alone fits', few['in-process'] and not few['native'] and not few['emulated']
                 and modes.chosen(2, 20) == 'in-process', modes.told(2, 20))
    report.check('on 1 core none fits and the robot runs in its own process', modes.chosen(1, 20) == 'in-process',
                 modes.told(1, 20))
    s = worker.session()
    worker.Robot(s).said(0.5, 1.7)
    told = worker.told(s)
    report.check("the robot's state tells its ratio and headroom", told['ratio'] == 0.5 and told['headroom'] == 1.7,
                 'ratio %s headroom %s' % (told['ratio'], told['headroom']))
    s['g']['world'].close()


def test_the_referee_and_the_robots_averages_in_c(report):
    """C referee (`wbc_pass_paired`) and drive averages (`wbc_pass_averaged`) match numpy bit-for-bit."""
    from machine.body import figure
    from machine.law import modal
    from machine.world import referee
    from machine.process import worker
    from tools.cores.orders import same
    s = worker.session(walks=True)
    g, apart, falls = s['g'], 0, 0
    while g['t'] < 9.0 and s['fallen_at'] is None:
        if abs(g['t'] - 5.0) < 0.5 * physics.STEP_S:
            g['world'].push((0.0, 0.0, 3.0 * figure.mass() * modal.G), 0.2)
        torque, power = s['torque'].copy(), s['power'].copy()
        worker.stepped(s)
        world, d = g['world'], g['world'].data
        torque += worker.K * (d.ctrl - torque)
        power += worker.K * (d.ctrl * d.qvel[world.vadr] - power)
        _m, (down, ground), _at = g['_falls_by']
        a, b = d.contact.geom[:d.ncon, 0], d.contact.geom[:d.ncon, 1]
        under = d.contact.pos[:d.ncon, 1] < float(g['fr'][modal.FLOOR]) + figure.ANKLE_H + figure.SHANK
        fallen = bool((((down[a] & ground[b]) | (down[b] & ground[a])) & under).any())
        falls += fallen
        apart += (fallen != referee.fallen(g, world)) + (not (same(torque, s['torque']) and same(power, s['power'])))
    report.check('the referee and the robot\'s averages in C are their numpy at every pass, a shove its fall',
                 not apart and falls > 0 and s['fallen_at'] is not None, '%d apart, fallen at %s' % (apart, s['fallen_at']))
    g['world'].close()


ROSTER = (test_the_robots_tiers_by_the_machine, test_the_referee_and_the_robots_averages_in_c, test_every_build_runs_the_robot, test_the_robots_gait_is_stated_on_it, test_a_virtual_body_is_told, test_a_leg_by_its_foot, test_a_body_with_mass_is_fitted,
          test_dressed_or_bare, test_the_robots_views_draw, test_the_robots_skeleton_collides,
          test_each_drive_turns_from_its_gearbox, test_each_drive_is_held,
          test_a_gearbox_has_play)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
