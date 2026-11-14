#!/usr/bin/env python3
"""The gynoid's body on fantasy boards (`physics.ENVELOPE` 0, their SOA never binding): her
clothes and her look, her leg by its foot, a virtual and a dynamic machine of her, her skeleton,
her drives' mounts and gearboxes, her kinematics as motors."""
import sys

from structure_kit import Report
from tools.dev.focus import chosen
from machine.world import physics

physics.ENVELOPE = 0.0


def test_dressed_or_bare(report):
    """Bare body strips worn garments/hair/soles, plates feet and shows four quick-release bands."""
    from coaxial.graphics import gynoid, lit, sneaker
    dressed, bare = gynoid.body(), gynoid.body(dressed=False)
    worn = [p[0] for p in dressed.parts if gynoid._worn(p[0])]
    bands = [p[0] for p in bare.parts if p[0].startswith('release_')]
    report.check('bare, every worn part gone, a band a quick-release and nothing else',
                 [p[0] for p in bare.parts if not p[0].startswith('release_')]
                 == [p[0] for p in dressed.parts if not gynoid._worn(p[0])]
                 and len(bands) == 4
                 and {'cloth_tee', 'cloth_left_sleeve', 'hair', 'hair_fall',
                      'left_sole'} <= set(worn),
                 '%d of %d worn, %d bands' % (len(worn), len(dressed.parts), len(bands)))
    feet = [i for i, p in enumerate(bare.parts) if p[0].split('_', 1)[-1] in ('foot', 'toes')]
    report.check('her feet plated bare, sneakers dressed',
                 all((bare.materials[slice(*bare.spans[i])] == lit.PLATE).all() for i in feet)
                 and all((dressed.materials[slice(*dressed.spans[i])]
                          == lit.paint(sneaker.SNEAKER)).all() for i in feet))


def test_her_views_draw(report):
    """The page's mechanism and actuators views draw: a held drive (`drives.WAYS`) has no drum,
    so no spur or bevel pair on it - the wrists' bevels crashed the page."""
    from coaxial.graphics import gynoid
    for see in ('mechanism', 'actuators'):
        seen = gynoid.body(see=see)
        report.check('her %s drawn, nothing on a held wrist' % see,
                     len(seen.parts) > 20 and not any(
                         p[0].startswith(('bevel', 'pinion', 'spur')) and p[0].endswith('_wrist')
                         for p in seen.parts), '%d parts' % len(seen.parts))


def _weyl(index, dim):
    primes = (2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61)
    return ((index + 1) * (primes[dim % len(primes)] ** 0.5)) % 1.0


def _span(lo, hi, u):
    return lo + (hi - lo) * u


def test_a_virtual_body_is_told(report):
    """VIRTUAL: no board, the humanoid's joints each where it is told, slewed; a pose
    written, each reads its command back, the type's routines run."""
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
    report.check('a pose written, every joint reads back where it was told', worst < 1e-9,
                 '%.3g deg' % worst)
    dt = 0.05
    step_deg = body.actuators['left_knee'].deg_s * dt
    body.loop.write(left_knee=told['left_knee'] + 1.6 * step_deg)
    got = body.loop.step(dt)
    got = body.loop.step(dt)
    moved = got['left_knee.deg'] - told['left_knee']
    report.check('a joint slews at its rate over one control step',
                 abs(moved - step_deg) < 1e-6, '%.3f deg' % moved)
    out = body.run('0 run=walk times=1')
    report.check('the type\'s walk routine runs on them', out.status == 'done', out.status)
    body.disarm()


def test_a_leg_by_its_foot(report):
    """figure.leg answers the six joints that put a foot where `foot_of` finds it across her travel."""
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
    report.check('the figure\'s segment shares sum to unity at %.1f kg' % figure.MASS_KG, abs(
        sum(seg[5] for seg in figure.SEGMENTS) - 1.0) < 1e-3,
        sum(seg[5] for seg in figure.SEGMENTS))


def test_a_body_with_mass_is_fitted(report):
    """DYNAMIC: each of the gynoid's joints is a drive on a body with mass, her pose read beside them."""
    from machine import Machine
    from machine.body import figure
    from machine.rig.modes import DYNAMIC
    from machine.rig.dynamic import DriveJoint
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    report.check('all %d drives fitted bus by bus, and her pose read beside them' % len(figure.JOINTS),
                 len(body.actuators) == len(figure.JOINTS)
                 and all(isinstance(a, DriveJoint) for a in body.actuators.values())
                 and body.actuators['spine'].node.name == 'D1_1'
                 and [n.name for n in body.others] == ['pelvis'],
                 {n: a.node.name for n, a in list(body.actuators.items())[:4]})
    body.close()


def test_her_skeleton_collides(report):
    """Skeleton adds bone geoms at unchanged mass, clear at stand/squat and meeting femur past stop."""
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
    report.check('her skeleton is %d geoms more, her weight as it was' % (model.ngeom - bare.ngeom),
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
    report.check("folded past its stop, the ankle's drives meet the femur, the skins not each other",
                 ('left_ankle_drum', 'left_thigh_bone') in met
                 and ('left_ankle_roll_drum', 'left_thigh_bone') in met
                 and ('left_shank', 'left_thigh') not in met, '%s' % sorted(met)[:4])


def test_each_drive_turns_from_its_gearbox(report):
    """Every drive turns what it drives about its drum axis from its gearbox end."""
    from tools.sim import fit
    wrong = [(j, w) for j, _what, w in fit.outputs() if w]
    report.check('every drive turns what it drives from its gearbox, on its axis', not wrong, '%s' % wrong)


def test_each_drive_is_held(report):
    """Every drive is structurally mounted to a bone, gimbal, boom or trunk frame."""
    from tools.sim import held
    loose = [j for j, by in held.held_by().items() if not by]
    report.check('every drive held', not loose, '%s held by nothing' % loose)


def test_her_kinematics_as_motors(report):
    """Motor chains match `figure.frames`, damped solve matches `figure.leg`, half-screw squares."""
    import math
    from machine.body import drives, figure, motors
    pelvis = (0.1 * figure.SCALE, figure.standing(), -0.2 * figure.SCALE)
    turn = figure.mul(figure.ry(0.3), figure.rx(0.08))
    base, off, leg, half = motors.of(turn, pelvis), 0.0, 0.0, 0.0

    def apart(m, at, here):
        return max(math.dist(motors.place(m), at), max(
            abs(a - b) for p, q in zip(motors.turn(m), here) for a, b in zip(p, q)))
    for i in range(50):
        deg = {j: _span(*(drives.travel(j) or (-40.0, 40.0)), _weyl(i, k))
               for k, j in enumerate(figure.JOINTS)}
        placed = figure.frames(deg, pelvis, turn)
        for end in ('left_foot', 'right_toes', 'left_hand', 'right_fingers', 'head'):
            rows = motors.chain(end)
            m = motors.forward(rows, base, [math.radians(deg[j]) for j, _pre, _axis in rows])
            off = max(off, apart(m, *placed[end]))
    report.check('a chain of motors puts a foot, a hand and her head where the figure has them',
                 off < 1e-12, '%.1e m, or of a unit axis' % off)
    rows = motors.chain('left_foot')
    hy = math.radians(0.5 * drives.TRAVEL['hip_yaw'][1])
    hr = math.radians(0.4 * drives.TRAVEL['spine_roll'][1])
    kn_hi = math.radians(0.70 * drives.TRAVEL['knee'][1])
    an_lo, an_hi = (math.radians(0.5 * v) for v in drives.TRAVEL['ankle'])
    ar = math.radians(0.8 * drives.TRAVEL['ankle_roll'][1])
    for i in range(50):
        angles = (_span(-hy, hy, _weyl(i, 0)), _span(-hr, hr, _weyl(i, 1)),
                  _span(-2.0 * hy, hy, _weyl(i, 2)), _span(0.05, kn_hi, _weyl(i, 3)),
                  _span(an_lo, an_hi, _weyl(i, 4)), _span(-ar, ar, _weyl(i, 5)))
        ankle, foot = figure.foot_of(1.0, pelvis, turn, angles)
        there = motors.of(foot, ankle)
        seed = [a + _span(-0.05, 0.05, _weyl(i, 12 + k)) for k, a in enumerate(angles)]
        got, _left = motors.solve(rows, base, there, seed, damp=1e-6, passes=8)
        leg = max(leg, max(abs(a - b) for a, b in zip(
            got, figure.leg(1.0, pelvis, turn, ankle, foot))))
        step = motors.mul(motors.inv(base), motors.between(base, there, 0.5))
        half = max(half, apart(motors.mul(base, motors.mul(step, step)), ankle, foot))
    report.check('its solve answers the recipe\'s six joints from 0.05 rad off', leg < 1e-8,
                 '%.1e rad' % leg)
    report.check('half the screw between two poses, taken twice, is the other', half < 1e-9,
                 '%.1e m, or of a unit axis' % half)


def test_a_gearbox_has_play(report):
    """BOXED: rotor crosses backlash free, winds up `tau / wind`, and crosses play on reversal."""
    from machine.body import figure
    from machine.world import physics
    from machine.body.figure import JOINTS
    i = JOINTS.index('waist')
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
    still = max(abs(q) for t, d, q in on if across is None or t <= across)
    report.check('the link stands while the rotor crosses the play, 10-60 ms',
                 across is not None and 0.01 <= across <= 0.06 and still < 1e-4,
                 'across at %s s, the link moved %.2e rad meanwhile' % (across, still))
    wound = gap + tau / wind
    report.check('past the play the box winds up tau / wind and the link turns',
                 abs(on[-1][1] - wound) < 0.3 * wound and on[-1][2] > 0.005,
                 'rotor %.1f mrad past the link of %.1f, the link turned %.1f mrad in 0.3 s' % (
                     1e3 * on[-1][1], 1e3 * wound, 1e3 * on[-1][2]))
    back = run(0.3, -tau)
    cross = next((t for t, d, _q in back if d <= -gap), None)
    at = next((q for t, d, q in back if cross is not None and t >= cross), None)
    report.check('reversed, the rotor crosses the play again before the link turns back',
                 cross is not None and 0.01 <= cross - on[-1][0] <= 0.1 and at is not None
                 and at >= on[-1][2] - 1e-6,
                 'the other flank met %s s after the reversal, the link at %.1f mrad from %.1f' % (
                     None if cross is None else round(cross - on[-1][0], 3),
                     1e3 * (at if at is not None else 0.0), 1e3 * on[-1][2]))


def test_every_build_runs_her(report):
    """Every build file but hers (`mechatronics.robots`) runs her on her loop under COAXIAL_ROBOT:
    every module imported, her first seconds and a walk told (`tools/dev/smoke.py`)."""
    import os
    import subprocess
    from machine.body import mechatronics
    host = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    others = [n for n in mechatronics.robots() if n != mechatronics.ROBOT]
    report.check('a second build beside hers', bool(others), ', '.join(mechatronics.robots()))
    for other in others:
        run = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(host, 'tools', 'dev', 'smoke.py')],
                             cwd=host, env=dict(os.environ, COAXIAL_ROBOT=other, PYTHONPATH=host),
                             capture_output=True, text=True, timeout=600)
        report.check('%s runs her: every module imported, her first seconds on the loop' % other,
                     run.returncode == 0 and 'smoke: clean' in run.stdout, (run.stdout + run.stderr)[-300:])


def test_her_gait_is_stated_on_her(report):
    """Declarative `.toml` construction compiles into `.cache/compiled/` on first run and recompiles on edit."""
    import math
    import os
    import tempfile
    import tomllib
    from machine.body import build, drives, figure, gait, mechatronics, servos
    with open(os.path.join(mechatronics.ROBOTS, mechatronics.HERS + '.toml'), 'rb') as f:
        raw = tomllib.load(f)
    banned = {'servo', 'wind', 'series', 'gait', 'service', 'pack_v', 'cooling', 'clamped',
              'rigid_play_deg', 'ahead_wind', 'ahead_play', 'mount_k_w', 'off_mount_k_w'}
    required = {'bus', 'passive_limits', 'stops', 'boards', 'mounts', 'linkage', 'skeleton', 'garments', 'elastomers'}
    lib, c_cfg = mechatronics.library(), mechatronics.crafts()
    w_cfg, m_cfg = mechatronics.environment(), mechatronics.materials()
    bus_banned = {'eso_bw_share', 'observe_x', 'rate_s', 'play_nm', 'wear_window_s', 'wear_trend_windows'}
    craft_banned = ({k for c in c_cfg.values() if isinstance(c, dict) for k in c}
                    & {'k_thrust', 'k_drag', 'inertia_xx', 'stiffness', 'damping', 'turn_bandwidth'})
    lib_banned = ({k for m in lib['motors'].values() for k in m} & {'rotor_kgm2', 'stator_j_k'}
                  | {k for b in lib['gearboxes'].values() for k in b} & {'stiffness_nm_rad'}
                  | {k for v in lib['inverters'].values() for k in v} & {'laminate_k_w', 'loop_hz'})
    report.check('library, humanoid and craft .toml files hold pure physical construction only',
                 required <= set(raw) and not (banned & set(raw))
                 and not ({'squat', 'stand', 'stand_knee'} & set(raw['figure']))
                 and not (bus_banned & set(raw['bus'])) and not craft_banned
                 and 'g_m_s2' in w_cfg and 'solids' in m_cfg and 'rad_s' not in raw['toes'] and not lib_banned
                 and 'stack_on' in raw['boards']['ankle'] and 'stack_on' in raw['boards']['head'])
    with tempfile.NamedTemporaryFile('w', suffix='.toml', delete=False, encoding='utf-8') as tf:
        tf.write('mass = 2.0\n')
        probe = tf.name
    calls = []
    try:
        v1 = mechatronics.compiled('_probe_test', (probe,), lambda: (calls.append(1), mechatronics._toml(probe)['mass'])[1])
        v2 = mechatronics.compiled('_probe_test', (probe,), lambda: (calls.append(1), mechatronics._toml(probe)['mass'])[1])
        with open(probe, 'w', encoding='utf-8') as tf:
            tf.write('mass = 3.5\n')
        v3 = mechatronics.compiled('_probe_test', (probe,), lambda: (calls.append(1), mechatronics._toml(probe)['mass'])[1])
    finally:
        for p in (probe, os.path.join(mechatronics.CACHE, '_probe_test.json')):
            if os.path.exists(p):
                os.unlink(p)
        mechatronics._COMPILED.pop('_probe_test', None)
    pk, sq = raw['pack'], figure.SQUAT
    sq_turn = figure.rx(math.radians(-(sq['left_hip'] + sq['left_knee'] + sq['left_ankle'])))
    ank_rel, _ = figure.foot_of(1.0, (0.0, 0.0, 0.0), sq_turn, [math.radians(sq['left' + k]) for k in figure.LEG])
    sq_com = figure.com(sq, (0.0, figure.ANKLE_H - ank_rel[1], -ank_rel[2]), sq_turn)
    report.check('meta-compiler caches first run and recompiles automatically on parameter change',
                 (v1, v2, v3, len(calls)) == (2.0, 2.0, 3.5, 2)
                 and drives.PACK_V == float(pk['series'] * pk['cell_v_min'])
                 and 0.4 * figure.MASS_KG < build.mass() < figure.MASS_KG
                 and drives.SERIES['ankle'] > 0.0 and drives.TRACK == 0.0
                 and len(servos.SERVO) == len(drives.STACKS) and len(drives.WIND) == len(drives.STACKS)
                 and abs(sq_com[2] - 0.5 * (figure.BALL - figure.HEEL)) < 1e-3)
    report.check('her leg and her mass are those her gait is stated at', gait.SIMILAR == 1.0 and gait.MASSY == 1.0,
                 'leg %r, mass %r of hers' % (gait.SIMILAR, gait.MASSY))


def test_her_tiers_by_the_machine(report):
    """`modes.afforded`, `chosen` and `told`: on 16 cores her 20 boards fit in-process and native,
    not on Renode; her body wired to in-process alone, that is chosen; on 2 cores in-process alone
    fits; on 1 none does and she runs in her own process. `sprung.state` tells her headroom beside
    her ratio."""
    from machine.rig import modes
    from machine.process import sprung, worker
    fits = {name: ok for name, _c, ok in modes.afforded(16, 20)}
    report.check('on 16 cores 20 boards fit in-process and native, not on Renode; in-process chosen, the one wired',
                 fits['in-process'] and fits['native'] and not fits['emulated'] and modes.chosen(16, 20) == 'in-process',
                 modes.told(16, 20))
    few = {name: ok for name, _c, ok in modes.afforded(2, 20)}
    report.check('on 2 cores in-process alone fits', few['in-process'] and not few['native'] and not few['emulated']
                 and modes.chosen(2, 20) == 'in-process', modes.told(2, 20))
    report.check('on 1 core none fits and she runs in her own process', modes.chosen(1, 20) == 'in-process',
                 modes.told(1, 20))
    s = worker.session()
    worker.Her(s).said(0.5, 1.7)
    told = worker.told(s)
    report.check("her state tells her ratio and her headroom", told['ratio'] == 0.5 and told['headroom'] == 1.7,
                 'ratio %s headroom %s' % (told['ratio'], told['headroom']))
    s['g']['world'].close()


def test_the_referee_and_her_averages_in_c(report):
    """Her worker's referee (`referee.fallen`, wbc_pass_paired) and her drives' averages
    (wbc_pass_averaged) beside their numpy at every pass of her walk and a shove that throws her:
    the same fall at each pass, a fall seen, the averages bit for bit."""
    import numpy as np
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
    report.check('the referee and her averages in C are their numpy at every pass, a shove her fall',
                 not apart and falls > 0 and s['fallen_at'] is not None, '%d apart, fallen at %s' % (apart, s['fallen_at']))
    g['world'].close()


ROSTER = (test_her_tiers_by_the_machine, test_the_referee_and_her_averages_in_c, test_every_build_runs_her, test_her_gait_is_stated_on_her, test_a_virtual_body_is_told, test_a_leg_by_its_foot, test_a_body_with_mass_is_fitted,
          test_dressed_or_bare, test_her_views_draw, test_her_skeleton_collides,
          test_each_drive_turns_from_its_gearbox, test_each_drive_is_held,
          test_a_gearbox_has_play, test_her_kinematics_as_motors)


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
