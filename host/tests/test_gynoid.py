#!/usr/bin/env python3
"""The gynoid on fantasy boards (`physics.ENVELOPE` 0, their SOA never binding): her walk, her
clothes and her look. Her faults on the boards as built are test_gynoid_faults.py's."""
import sys

from tools.dev.focus import chosen

from machine import physics

physics.ENVELOPE = 0.0

class Report:
    def __init__(self):
        self.passed = self.failed = 0

    def check(self, name, ok, detail=''):
        self.passed += bool(ok)
        self.failed += (not ok)
        print('  %s  %-58s %s' % ('PASS' if ok else 'FAIL', name, detail))


def test_dressed_or_bare(report):
    """Her clothes come off at a key (`gynoid.render`, dressed): the bare body is the dressed one
    without what she wears - her clothes, her hair, her soles - her feet plated, her limbs' quick-
    releases banded on her shell."""
    from coaxial.graphics import gynoid, lit, sneaker
    dressed, bare = gynoid.body(), gynoid.body(dressed=False)
    worn = [p[0] for p in dressed.parts if gynoid._worn(p[0])]
    bands = [p[0] for p in bare.parts if p[0].startswith('release_')]
    report.check('bare, every worn part gone, a band a quick-release and nothing else',
                 [p[0] for p in bare.parts if not p[0].startswith('release_')]
                 == [p[0] for p in dressed.parts if not gynoid._worn(p[0])]
                 and len(bands) == 4
                 and {'cloth_tee', 'cloth_left_sleeve', 'hair_fall', 'left_lock',
                      'left_sole'} <= set(worn),
                 '%d of %d worn, %d bands' % (len(worn), len(dressed.parts), len(bands)))
    feet = [i for i, p in enumerate(bare.parts) if p[0].split('_', 1)[-1] in ('foot', 'toes')]
    report.check('her feet plated bare, sneakers dressed',
                 all((bare.materials[slice(*bare.spans[i])] == lit.PLATE).all() for i in feet)
                 and all((dressed.materials[slice(*dressed.spans[i])]
                          == lit.paint(sneaker.SNEAKER)).all() for i in feet))


def test_her_views_draw(report):
    """The page's mechanism and actuators views draw: a held drive (`drives.WAYS`) has no drum,
    so no spur or bevel pair on it - the wrists' bevels crashed the page (2026-10-04)."""
    from coaxial.graphics import gynoid
    for see in ('mechanism', 'actuators'):
        seen = gynoid.body(see=see)
        report.check('her %s drawn, nothing on a held wrist' % see,
                     len(seen.parts) > 20 and not any(
                         p[0].startswith(('bevel', 'pinion', 'spur')) and p[0].endswith('_wrist')
                         for p in seen.parts), '%d parts' % len(seen.parts))


def test_the_floor_outlasts_a_walk(report):
    """The floor's slab ends past an hour's walk at her fastest pace: at 80 m she stepped off it
    100 s in (2026-09-28)."""
    from machine import floor, gait
    from terminal.views.humanoid_keys import CADENCE
    fastest = CADENCE[1] * gait.STRIDE_M * gait.pace(CADENCE[1])
    report.check('the slab past an hour at %.2f m/s' % fastest,
                 floor.SLAB_TO_M > 3600.0 * fastest, "%.0f m" % floor.SLAB_TO_M)


def test_a_virtual_body_walks(report):
    """VIRTUAL: no board, the humanoid's twenty joints each where it is told, slewed; the walk
    (`machine.gait`) writes them all, each reads its command back, the type's routines run."""
    from machine import Machine, gait
    from machine.modes import VIRTUAL
    from machine.virtual import VirtualJoint
    body = Machine.discover('humanoid', execution_mode=VIRTUAL)
    report.check('twenty virtual joints, fitted bus by bus as the type names them',
                 len(body.actuators) == 20
                 and all(isinstance(a, VirtualJoint) for a in body.actuators.values())
                 and body.actuators['pelvis'].node.name == 'V1_1'
                 and body.actuators['right_foot'].node.name == 'V5_4',
                 {n: a.node.name for n, a in list(body.actuators.items())[:5]})
    body.arm()
    told = gait.walk(0.4)
    body.loop.write(**told)
    for _ in range(12):
        got = body.loop.step(0.04)
    worst = max(abs(got[j + '.deg'] - told[j]) for j in body.actuators)
    report.check('the walk written, every joint reads back where it was told', worst < 1e-9,
                 '%.3g deg' % worst)
    body.loop.write(left_knee=told['left_knee'] + 120.0)
    got = body.loop.step(0.05)
    got = body.loop.step(0.05)
    moved = got['left_knee.deg'] - told['left_knee']
    report.check('a joint slews at its rate: 120 deg asked, 75 there 0.05 s on (1500 deg/s)',
                 abs(moved - 75.0) < 1e-6, '%.3f deg' % moved)
    out = body.run('0 run=walk times=1')
    report.check('the type\'s walk routine runs on them', out.status == 'done', out.status)
    phases = [gait.walk(k / 50.0) for k in range(50)]
    report.check('the gait names every joint the body has, each within its span',
                 all(set(p) == set(body.actuators) for p in phases)
                 and all(abs(v) <= body.ranges[j][1] for p in phases for j, v in p.items()),
                 max(abs(v) for p in phases for v in p.values()))
    body.disarm()


def test_a_leg_by_its_foot(report):
    """figure.leg answers the six joints that put a foot where `foot_of` finds it, any pose."""
    import math
    import random
    from machine import figure
    rng, worst = random.Random(1), 0.0
    for _ in range(200):
        angles = (rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), rng.uniform(-0.8, 0.4),
                  rng.uniform(0.05, 1.5), rng.uniform(-0.5, 0.5), rng.uniform(-0.3, 0.3))
        sign = rng.choice((1.0, -1.0))
        pelvis = (rng.uniform(-0.2, 0.2), rng.uniform(0.8, 1.2), rng.uniform(-0.2, 0.2))
        turn = figure.mul(figure.mul(figure.ry(rng.uniform(-0.5, 0.5)), figure.rx(rng.uniform(-0.2, 0.2))),
                          figure.rz(rng.uniform(-0.2, 0.2)))
        ankle, foot = figure.foot_of(sign, pelvis, turn, angles)
        got = figure.leg(sign, pelvis, turn, ankle, foot)
        worst = max(worst, max(abs(a - b) for a, b in zip(got, angles)))
    report.check('a leg\'s six joints from its foot, 200 poses round trip', worst < 1e-9,
                 '%.2g rad' % worst)
    report.check('the figure weighs her 55 kg, every share of it', abs(
        sum(seg[5] for seg in figure.SEGMENTS) - 1.0) < 1e-3,
        sum(seg[5] for seg in figure.SEGMENTS))


def test_a_body_with_mass_walks(report):
    """DYNAMIC: the gynoid's 27 joints each a drive on a body with mass; the walker sets them
    every millisecond from what the loop reads, and she walks on the line without falling."""
    from machine import Machine, gait
    from machine.modes import DYNAMIC
    from machine.dynamic import DriveJoint
    from machine.walker import Walker
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    report.check('27 drives fitted bus by bus, and her pose read beside them',
                 len(body.actuators) == 27
                 and all(isinstance(a, DriveJoint) for a in body.actuators.values())
                 and body.actuators['spine'].node.name == 'D1_1'
                 and [n.name for n in body.others] == ['pelvis'],
                 {n: a.node.name for n, a in list(body.actuators.items())[:4]})
    body.arm()
    walker = Walker(body, 0.85)
    walker.start()
    body.loop.step(0.0)
    lowest, borne = 9.0, []
    while body.loop.bus['t'] < 3.0:
        body.loop.write(**walker.step(0.001))
        body.loop.step(0.001)
        lowest = min(lowest, body.loop.bus['pelvis.pose.y'])
        if body.loop.bus['t'] >= 2.0:
            borne.append(sum(body.loop.bus['pelvis.pose.%s_load' % s] for s in ('left', 'right')))
    bus = body.loop.bus
    # 0.9 of what her strides make of 3 s: 2 m of the 0.85 m stride's 2.17 was past the 0.75 m
    # stride's 1.91 (2026-10-04).
    far = 0.9 * 3.0 * 0.85 * gait.STRIDE_M
    report.check('3 s at 0.85 strides/s: on her feet, 0.9 of her strides on, within 0.2 m of the line',
                 lowest > 0.7 and bus['pelvis.pose.z'] > far and abs(bus['pelvis.pose.x']) < 0.2,
                 'lowest %.2f m, %.2f m on of %.2f, %+.2f m off' % (
                     lowest, bus['pelvis.pose.z'], far, bus['pelvis.pose.x']))
    mean = sum(borne) / len(borne)
    report.check('her weight is on her soles: 539 N between them, meaned over her last second',
                 abs(mean - 539.0) < 270.0, '%.0f N' % mean)
    body.disarm()


def test_the_pendulum_between_her_ears(report):
    """The virtual pendulum hears the head alike every way: still at an even speed, stirred only
    along the way the head is shaken."""
    import math
    from machine.pendulum import Pendulum

    def shaken(axis, amount):
        pendulum = Pendulum()
        for k in range(6000):
            t = k / 1000.0
            at = [0.0, 1.5, 0.9 * t]
            if t > 3.0:
                at[axis] += amount * math.sin(2.0 * math.pi * 1.7 * t)
            pendulum.step(tuple(at), 0.001)
        return pendulum
    even = shaken(0, 0.0)
    report.check('at an even 0.9 m/s it hangs still, pulling her weight',
                 even.stir < 1e-9 and abs(even.felt - 1.0) < 1e-9,
                 '%.2g mm, felt %.9f' % (even.stir, even.felt))
    for name, axis, part in (('surged', 2, 0), ('swayed', 0, 1), ('bobbed', 1, 2)):
        stirs = shaken(axis, 0.01).stirs
        report.check('%s 1 cm at 1.7 Hz, it stirs that way alone' % name,
                     stirs[part] > 0.0 and all(stirs[k] < 1e-9 * stirs[part]
                                              for k in range(3) if k != part),
                     'on %.3g across %.3g up %.3g mm' % stirs)


def test_she_rises_and_walks(report):
    """The director: landed in the squat, she rises, steps off on her standing stance and walks
    on into the catwalk, the moves handing one to the next."""
    from machine import Machine
    from machine.director import Director
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    director = Director(body, 0.85)
    director.begin()
    body.loop.step(0.0)
    stages = []
    while body.loop.bus['t'] < 13.0:
        body.loop.write(**director.step(0.001))
        body.loop.step(0.001)
        if director.stage != 'catch' and (not stages or stages[-1] != director.stage):
            stages.append(director.stage)   # a catch is the walker's own, a step within the walk
    bus = body.loop.bus
    report.check('the squat to the walk, move by move, and walking at 13 s',
                 stages == ['squat', 'look', 'push', 'rise', 'stand', 'lean', 'shift', 'step',
                            'walk'],
                 ' '.join(stages))
    report.check('walked on over 2.5 m', bus['pelvis.pose.z'] > 2.5, '%.2f m' % bus['pelvis.pose.z'])
    body.disarm()


def test_a_style_eases_in(report):
    """A style knob trimmed (`machine.style`): the walk's plan eases from the old tables to the
    new over `walkplan.RETABLE_S`, never a jump; the knob within its bounds."""
    from machine import style, walkplan
    was = style.value('turn')
    try:
        old = walkplan.plan(0.3, 1.0)
        style.trim('turn', 4)
        at_once = walkplan.plan(0.3, 1.0)
        walkplan.ease(walkplan.RETABLE_S / 2.0)
        half = walkplan.plan(0.3, 1.0)
        walkplan.ease(walkplan.RETABLE_S)
        new = walkplan.plan(0.3, 1.0)
        report.check('trimmed, the plan the old one at first, between halfway, the new at the end',
                     at_once[2] == old[2] and min(old[2], new[2]) < half[2] < max(old[2], new[2])
                     and new[2] != old[2], 'yaw %.4f %.4f %.4f rad' % (old[2], half[2], new[2]))
        report.check('a knob kept within its bounds', style.set('turn', 99.0) == 12.0,
                     '%.1f deg' % style.value('turn'))
    finally:
        style.set('turn', was)
        walkplan.ease(walkplan.RETABLE_S)


def test_her_skeleton_collides(report):
    """Her skeleton switched on (`physics.SKELETON`, `machine.skeleton`): her model with it, the
    same weight; standing and in the squat it touches nothing; the knee folded to 170 degrees the
    calf's two ankle drives meet the femur, her skins next to each other still not."""
    import importlib
    import math
    from machine import mjcf, physics, skeleton
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
    from machine import arrival, gait
    report.check('standing and in the squat it touches nothing',
                 not skeleton.overlapping(model, gait.stand())
                 and not skeleton.overlapping(model, arrival.angles_of(arrival._squat())))
    d = mujoco.MjData(model)
    d.qpos[1] = 2.0
    d.qpos[model.joint('left_knee').qposadr[0]] = math.radians(170.0)
    mujoco.mj_forward(model, d)
    met = {tuple(sorted((model.body(model.geom_bodyid[d.contact[i].geom1]).name,
                         model.body(model.geom_bodyid[d.contact[i].geom2]).name)))
           for i in range(d.ncon)}
    # On the 68 mm stacks the ankle roll's drum met the knee's board first; on the one 60 mm
    # stack, the femur, the board at 175 deg (2026-10-05).
    report.check("folded, the ankle's drives meet the femur, the skins not each other",
                 ('left_ankle_drum', 'left_thigh_bone') in met
                 and ('left_ankle_roll_drum', 'left_thigh_bone') in met
                 and ('left_shank', 'left_thigh') not in met, '%s' % sorted(met)[:4])


def test_each_drive_turns_from_its_gearbox(report):
    """Every drive's output turns what it drives about its drum's own axis from its gearbox's
    end (`tools.sim.fit.outputs`) - an axial drive with a radial output caught by eye (the user,
    2026-10-03)."""
    from tools.sim import fit
    wrong = [(j, w) for j, _what, w in fit.outputs() if w]
    report.check('every drive turns what it drives from its gearbox, on its axis', not wrong,
                 '%s' % wrong)


def test_each_drive_is_held(report):
    """Every drive held by a bone, a gimbal, the boom or the trunk's frame, touching it or
    through its own collars and struts (`tools.sim.held`) - the hip's roll drum, the
    ankles' and the toes' held by nothing, caught by eye (the user, 2026-10-03)."""
    from tools.sim import held
    loose = [j for j, by in held.held_by().items() if not by]
    report.check('every drive held', not loose, '%s held by nothing' % loose)


def test_her_kinematics_as_motors(report):
    """`machine.motors`: a chain of `figure.SEGMENTS` multiplied out puts a segment where
    `figure.frames` has it; its damped solve answers `figure.leg`'s six joints; half the screw
    between two poses taken twice is the other."""
    import math
    import random
    from machine import figure, motors
    rng = random.Random(2)
    pelvis, turn = (0.1, 0.89, -0.2), figure.mul(figure.ry(0.3), figure.rx(0.08))
    base, off, leg, half = motors.of(turn, pelvis), 0.0, 0.0, 0.0

    def apart(m, at, here):
        return max(math.dist(motors.place(m), at), max(
            abs(a - b) for p, q in zip(motors.turn(m), here) for a, b in zip(p, q)))
    for _ in range(50):
        deg = {j: rng.uniform(-40.0, 40.0) for j in figure.JOINTS}
        placed = figure.frames(deg, pelvis, turn)
        for end in ('left_foot', 'right_toes', 'left_hand', 'right_fingers', 'head'):
            rows = motors.chain(end)
            m = motors.forward(rows, base, [math.radians(deg[j]) for j, _pre, _axis in rows])
            off = max(off, apart(m, *placed[end]))
    report.check('a chain of motors puts a foot, a hand and her head where the figure has them',
                 off < 1e-12, '%.1e m, or of a unit axis' % off)
    rows = motors.chain('left_foot')
    for _ in range(50):
        angles = (rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2), rng.uniform(-0.8, 0.5),
                  rng.uniform(0.05, 1.2), rng.uniform(-0.5, 0.5), rng.uniform(-0.2, 0.2))
        ankle, foot = figure.foot_of(1.0, pelvis, turn, angles)
        there = motors.of(foot, ankle)
        got, _left = motors.solve(rows, base, there, [a + rng.uniform(-0.05, 0.05) for a in angles],
                                  damp=1e-6, passes=8)
        leg = max(leg, max(abs(a - b) for a, b in zip(
            got, figure.leg(1.0, pelvis, turn, ankle, foot))))
        step = motors.mul(motors.inv(base), motors.between(base, there, 0.5))
        half = max(half, apart(motors.mul(base, motors.mul(step, step)), ankle, foot))
    report.check('its solve answers the recipe\'s six joints from 0.05 rad off', leg < 1e-8,
                 '%.1e rad' % leg)
    report.check('half the screw between two poses, taken twice, is the other', half < 1e-9,
                 '%.1e m, or of a unit axis' % half)


def test_a_gearbox_has_play(report):
    """BOXED: fed a torque, a drive's rotor runs free across its gearbox's play before any of it
    reaches the link, winds the box up tau / wind past it as the link turns, and reversed crosses
    the play again before the link turns back (`machine.gearbox`):
    the waist, its servo off, standing."""
    from machine import gait, physics
    from machine.figure import JOINTS
    i = JOINTS.index('waist')
    physics.BOXED = 1.0
    try:
        _gearbox_has_play(report, physics, gait, i)
    finally:
        physics.BOXED = 0.0


def _gearbox_has_play(report, physics, gait, i):
    world = physics.World()
    world.reset(gait.stand())
    world.gains[i] = 0.0
    gap, wind, tau = float(world.boxes.gap[i]), float(world.boxes.k[i]), 3.0
    q0, now = float(world.data.qpos[world.qadr[i]]), [0.0]
    world.clock = lambda: now[0]

    def run(seconds, feed):
        world.feed[i] = feed
        rows = []
        for _ in range(int(round(seconds / 0.001))):
            now[0] += 0.001
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


ROSTER = (test_a_virtual_body_walks, test_a_leg_by_its_foot, test_a_body_with_mass_walks,
          test_the_pendulum_between_her_ears, test_she_rises_and_walks, test_dressed_or_bare,
          test_her_views_draw, test_the_floor_outlasts_a_walk, test_a_style_eases_in,
          test_her_skeleton_collides,
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
