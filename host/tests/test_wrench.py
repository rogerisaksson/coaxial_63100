"""Her step without an active set (`machine.wrench`) in the stack's place (`tools/sim/wbc.py
--wrench`): one step standing beside the stack's, a contact bearing her, her stand, a nudge
held, a hop."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: A step standing: its torques within APART_NM of the stack's, each sole's load within APART_N.
APART_NM, APART_N = 0.5, 2.0


def _standing():
    """(her model, its sense, the capture point's ask) at her stand's first pass."""
    from machine import figure, physics, wbc
    from tools.sim import wbc as sim
    world, body = physics.World(), wbc.Body()
    world.reset(figure.stand())
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    world.reset(figure.stand(), where=(0.0, 1.0 - min(float(f['pts'][:, 1].min()) for f in s['feet']), 0.0))
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    mid = (s['feet'][0]['sole'][[0, 2]] + s['feet'][1]['sole'][[0, 2]]) / 2.0
    return body, s, sim.law(s, sim.state(s), mid)


def test_a_step_standing_is_the_stacks(report):
    """Her stand's first pass, the capture point's law: the torques and the soles' loads of
    `wrench.step` beside `wbc.step`'s."""
    import numpy as np
    from machine import wbc, wrench
    body, s, ask = _standing()
    ours, theirs = wrench.step(body, s, ask), wbc.step(body, s, ask)
    torque = float(np.abs(ours['tau'] - theirs['tau']).max())
    load = max(abs(a - b) for a, b in zip(ours['bears'], theirs['bears']))
    report.check('standing, a step asks the drives and the floor what the stack does',
                 torque < APART_NM and load < APART_N, '%.2f N m and %.1f N apart at most' % (torque, load))


def test_a_contact_bears(report):
    """A segment that bears her is a contact of her step (`machine.borne`): under her pelvis,
    bearing half her weight now, it takes what her soles give up, no more than GROW_N over what
    it bore, in its friction's pyramid."""
    import numpy as np
    from machine import rounds, wbc, wrench
    body, s, ask = _standing()
    pelvis = body.m.body('pelvis').id
    p, jp = body.d.xpos[pelvis].copy(), np.zeros((3, body.m.nv))
    body.mj.mj_jac(body.m, body.d, jp, None, p, pelvis)
    half = 0.5 * body.mass * wbc.G
    alone = wrench.step(body, s, ask)
    held = wrench.step(body, dict(s, contacts=[{'p': p, 'f': np.array([0.0, half, 0.0]), 'J': jp[:, :body.n].copy(),
                                                 'dJv': np.zeros(3)}]), ask)
    f = held['borne'][0]
    given = sum(alone['bears']) - sum(held['bears'])
    report.check('a contact under her pelvis takes what her soles give up, in its bounds',
                 0.0 < f[1] <= half + rounds.GROW_N and abs(given - f[1]) < 5.0
                 and np.hypot(f[0], f[2]) <= wbc.MU * f[1] * (1.0 + 1e-9),
                 '%.0f N on it, her soles %.0f N less, %.0f N across' % (f[1], given, np.hypot(f[0], f[2])))


def test_a_contact_slides(report):
    """The kernel's Coulomb slide (`drive-tangential-coulomb-slide`) on a pressing contact: asked to slide,
    its normal force stays under the cap and its friction is the kinetic one against the slide, given
    - not the step's to spend -, its point held along its normal alone."""
    import numpy as np
    from machine import wbc, wrench
    body, s, ask = _standing()
    pelvis = body.m.body('pelvis').id
    p, jp = body.d.xpos[pelvis].copy(), np.zeros((3, body.m.nv))
    body.mj.mj_jac(body.m, body.d, jp, None, p, pelvis)
    half, cap = 0.5 * body.mass * wbc.G, 0.3 * body.mass * wbc.G
    contact = [{'p': p, 'f': np.array([0.0, half, 0.0]), 'J': jp[:, :body.n].copy(), 'dJv': np.zeros(3)}]
    slid = wrench.step(body, dict(s, contacts=contact), dict(ask, slide=(0.0, 0.0, 0.16), fz_cap=cap))
    f = slid['borne'][0]
    report.check('sliding, the contact bears under the cap and its friction is MU of that against the slide',
                 0.0 < f[1] <= cap * (1.0 + 1e-9) and abs(f[2] + wbc.MU * f[1]) < 1e-6 * max(1.0, f[1])
                 and abs(f[0]) < 1e-6 * max(1.0, f[1]) and np.all(np.isfinite(slid['tau'])),
                 '%.0f N on it (cap %.0f), %.1f N along the slide, %.1f across' % (f[1], cap, f[2], f[0]))


def test_a_slide_said_rides_her_row(report):
    """A phase's slide (`ortho.SLIDE`, m/s along her way) is laid in its row and carried by her latent's
    output (`latent.SLID`) while the phase holds, her rest's none."""
    import numpy as np
    from machine import latent, ortho
    rest = np.zeros(ortho.P)
    rest[latent.K], rest[latent.HZ] = 100.0, 1.0
    tables, _ = ortho.compiled('(defbehavior b (phase p (duration 1s) (blend_duration 100ms)'
                               ' (task_constraints (slide -0.16))))')
    row = ortho.ROW.unpack_from(tables['b'], 0)
    st, out, dt = latent.state(rest), np.zeros(latent.OUT), 0.001
    tab = latent.table(tables['b'], 0.5, rest)
    seen = {}
    for tick in range(2000):
        latent.step(st, tab, tick * dt, dt, out)
        seen[tick] = float(out[latent.SLID])
    report.check('the row says -0.16 m/s; her latent carries it blended in, 0 again at rest',
                 abs(row[4 + ortho.SLIDE] + 0.16) < 1e-6 and abs(seen[700] + 0.16) < 1e-6
                 and abs(seen[549] + 0.08) < 0.01 and seen[1999] == 0.0,
                 'row %.3f, out %.3f at 700, %.3f at 549, %.3f at rest' % (row[4 + ortho.SLIDE], seen[700], seen[549], seen[1999]))


def test_her_reach_rolls_her_onto_the_ball(report):
    """The kernel's reachability on her loop (`machine.somatic`): a leg's hip to its sole over her leg's length;
    seated, from 0.86 of it her sole's centre of pressure goes onto its ball by the furniture's told share,
    whole at 0.98; standing, nothing borne, her soles stay as her stride has them."""
    import numpy as np
    from machine import latent, modal, somatic
    g = {'fr': np.zeros(modal.FRAME)}
    g['fr'][modal.ROT] = np.eye(3).ravel()
    g['fr'][modal.WEIGHT] = 300.0
    out = np.zeros(latent.OUT)

    def asked(rho, told, edge=(0.0, 0.0)):
        s = {'hips': [np.array([0.0, rho[0] * somatic.LEG_M, 0.0]), np.array([0.2, rho[1] * somatic.LEG_M, 0.0])],
             'feet': [{'sole': np.zeros(3)}, {'sole': np.array([0.2, 0.0, 0.0])}]}
        return somatic.asked(g, s, out, np.array([1.0 - told, told, 0.0, 0.0]), {'edge': edge})
    seated, deep, stood = asked((0.75, 0.70), 0.85), asked((0.92, 0.98), 0.85), asked((0.998, 0.998), 0.0)
    report.check('seated at 0.70-0.75 of her leg her soles are flat; at 0.92 and 0.98 onto the ball by the seat\'s 0.85;'
                 ' standing at 0.998, nothing borne, flat',
                 seated['edge'] == (0.0, 0.0) and abs(deep['edge'][0] - 0.85 * 0.5) < 1e-9 and abs(deep['edge'][1] - 0.85) < 1e-9
                 and stood['edge'] == (0.0, 0.0) and seated['fz_cap'] == np.inf and tuple(seated['slide']) == (0.0, 0.0, 0.0),
                 '%s %s %s' % (seated['edge'], deep['edge'], stood['edge']))
    out[latent.SLID] = -0.15
    slid = asked((0.75, 0.70), 0.85)
    report.check('a slide said of -0.15 m/s rides along her way, her contacts\' load capped at 0.55 of her weight',
                 abs(slid['slide'][0] + 0.15) < 1e-9 and abs(slid['fz_cap'] - 0.55 * 300.0) < 1e-9,
                 '%s cap %.1f' % (slid['slide'], slid['fz_cap']))


def test_she_stands_still(report):
    """Three seconds standing: up, every drive inside its clamp."""
    from tools.sim import wbc
    out = wbc.stand(seconds=3.0, wrench=True)
    report.check('standing 3 s she stays up, every drive within its clamp',
                 out['stood'] and out['top'] <= 1.0 and out['tilt'] < 2.0,
                 'tilt %.1f deg, drift %.1f mm, the most a clamp %.2f, %d us a step'
                 % (out['tilt'], out['drift_mm'], out['top'], out['us']))


def test_nudged_she_holds(report):
    """60 N for 0.12 s from her right and from her left, 38 N from behind: up."""
    from tools.sim import wbc
    for push, way, name in ((60.0, 0.0, 'toward her left'), (60.0, 180.0, 'toward her right'),
                            (38.0, 90.0, 'from behind')):
        out = wbc.stand(push, way, wrench=True)
        report.check('nudged %.0f N %s she holds' % (push, name), out['stood'], 'tilt %.1f deg' % out['tilt'])


def test_she_hops(report):
    """A hop (`tools.sim.hop`): thrown, airborne a tenth of a second, landed, stood."""
    from tools.sim import hop
    out = hop.hop(wrench=True)
    report.check('thrown, she leaps, lands and stands',
                 out['stood'] and out['leaps'] == 1 and out['air_s'] >= 0.1 and out['clear_mm'] >= 12.0,
                 '%.3f s, %.1f mm under her soles, landed %d N, tilt %.1f deg, %s'
                 % (out['air_s'], out['clear_mm'], out['landed_n'], out['tilt'], out['stances']))


def test_singular_fall_is_bounded(report):
    """When tumbling airborne near a straight-knee singularity (`|tau0| > limit`), `shrunk` and
    `step` scale the baseline into the drives' clamps without collapsing `scale` to 0 or flinging
    her driven joints at thousands of rad/s^2."""
    import numpy as np
    from machine import figure, physics, wbc, wrench
    world, body = physics.World(), wbc.Body()
    qpos = figure.stand()
    for j in ('left_knee', 'right_knee'):
        qpos[world.model.jnt_qposadr[world.model.joint(j).id]] = 0.005
    world.reset(qpos)
    qvel = np.zeros(len(world.data.qvel))
    qvel[:body.n] = 25.0
    s = wbc.sense(body, world.data.qpos, qvel)
    feet = s['feet']
    swung = {k: (feet[k]['sole'] + (0.0, 0.05, 0.05), np.zeros(3), np.zeros(3), feet[k]['quat']) for k in (0, 1)}
    ask = {'stance': (False, False), 'com_acc': np.array([0.0, -9.81, 0.0]), 'posture': s['q'].copy(),
           'turns': (), 'swing': swung, 'fold': {0: (0.6, 0.0, 0.0), 1: (0.6, 0.0, 0.0)}, 'wep': False}
    out = wrench.step(body, s, ask)
    qacc_max = float(np.abs(out['qacc'][body.act]).max())
    report.check('tumbling near a straight-knee singularity keeps qacc bounded and scale > 0',
                 0.0 < out['scale'] <= 1.0 and qacc_max < 1000.0 and bool((np.abs(out['tau']) <= body.clamp + 1e-9).all()),
                 'scale %.4f, max |qacc| %.1f rad/s^2' % (out['scale'], qacc_max))


def test_surety_contracts_authority_and_damps_fall(report):
    """Her directional surety field (`motion.surety`, `g['surety']`, `g['surety_vec']`) ranks a
    wide double-support stance above narrow standing and single-sole walking, contracts into her
    2D tuck-and-brace regime outside her upright envelope, absorbs at 0 on collapse so a bounce
    never re-fires torques, and damps her joints passively."""
    import numpy as np
    from machine import edge, modal, motion, sprung
    g = sprung.laid()
    for _ in range(5):
        sprung.passed(g)
    s_up = motion.surety(g, 0.2, ct=2.0)
    soles0 = g['y'][modal.SOLE].copy()
    g['y'][modal.SOLE] = np.array([-0.22, 0.0, 0.20, 0.22, 0.0, -0.20])
    s_wide = motion.surety(g, 1.0, ct=2.0)
    g['y'][modal.SOLE] = soles0
    g['y'][modal.LOAD][:] = g['sense'][edge.LOAD][:] = 0.0
    s_seat = motion.surety(g, 1.0, ct=4.0)
    g['y'][modal.LOAD][0] = g['sense'][edge.LOAD][0] = 150.0
    g['y'][modal.VCOM][2] = 0.55
    s_walk = motion.surety(g, 1.0, ct=4.0)
    g['sense'][edge.QUAT][:] = (0.891, 0.454, 0.0, 0.0)
    s_mid = motion.surety(g, 0.05)
    motion.control(g, 0.001)
    knee_tuck = float(g['ask']['posture'][g['body'].knees[0]])
    sh_reach = float(g['ask']['posture'][g['limbs']['shoulders'][0]])
    s_col = motion.surety(g, 0.05, ct=70.0)
    s_bnc = motion.surety(g, 0.05, ct=42.0)
    g['sense'][edge.QD][g['joints']] = 2.0
    motion.drives(g, 0.001)
    feed = g['cmd']['feed'].copy()
    qd = g['sense'][edge.QD][g['joints']]
    power = float((feed * qd).max())
    report.check('surety orders wide > stand > walk > topple -> 0 on collapse, braces arms/knees, and damps passively',
                 s_wide > s_up > s_walk > 0.65 and s_seat > 0.80 and 0.20 < s_mid < 0.60 and knee_tuck > 0.6
                 and sh_reach > 1.0 and s_col == 0.0 and s_bnc == 0.0 and power < -0.1
                 and float(np.abs(g['eso'][2]).max()) == 0.0,
                 'wide %.3f > stand %.3f > walk %.3f (seat %.3f) -> mid %.2f (knee %.2f, sh %.2f rad) -> col %.2f (bounce %.2f), %.2f W'
                 % (s_wide, s_up, s_walk, s_seat, s_mid, knee_tuck, sh_reach, s_col, s_bnc, power))


def test_her_master_bus_and_buffers(report):
    """Her master bus laid of the anti-spasm insert (`machine.supervisor`): each drive's slew its peak
    through its board's current loop, its jerk that over its rotor's inertia; a buffer before each
    stop of the insert's boundary nodes' joints, critically damped."""
    import math
    from coaxial.model import inverter, sensorless
    from machine import drives, supervisor, wbc
    from machine.figure import JOINTS
    names = [j for j in JOINTS if drives.passive(j) is None]
    _k, _s, laid = supervisor.bus(names, 1000.0)
    loop = sensorless.current_loop(0.0, 0.0, inverter.FSW, 0.0, drives.PACK_V)['bw_hz']
    report.check("each slew its peak x 2 pi x its board's loop, %.0f Hz; its jerk that over its rotor's inertia" % loop,
                 set(drives.LOOP_HZ.values()) == {loop}
                 and all(math.isclose(v, drives.peak(n) * 2.0 * math.pi * loop) for n, v in zip(names, laid['slew']))
                 and all(math.isclose(j, v / drives.armature(n)) for n, v, j in zip(names, laid['slew'], laid['jerk'])),
                 str([(n, round(v / 1e3)) for n, v in zip(names, laid['slew'])]))
    body = wbc.Body()
    rows, (width, stiff, damp, reach) = body.buffers
    ends = sorted((names[i], side) for i, _stop, side in rows)
    report.check('a buffer %.2f rad before each stop of %d joints, critically damped, reached 1/(e w) ahead'
                 % (width, len(ends) // 2),
                 ends == sorted((names[i], side) for i in supervisor.joints_of(
                     supervisor.insert()['joint-boundary']['nodes'], names) if i in body.stops for side in (1.0, -1.0))
                 and math.isclose(damp * damp, 4.0 * stiff) and math.isclose(reach * math.e * math.sqrt(stiff), 1.0),
                 str(ends))


ROSTER = (test_a_step_standing_is_the_stacks, test_a_contact_bears, test_a_contact_slides, test_a_slide_said_rides_her_row,
          test_her_reach_rolls_her_onto_the_ball, test_she_stands_still, test_nudged_she_holds,
          test_she_hops, test_singular_fall_is_bounded, test_surety_contracts_authority_and_damps_fall,
          test_her_master_bus_and_buffers)


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
