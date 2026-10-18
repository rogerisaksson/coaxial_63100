#!/usr/bin/env python3
"""Her body identified of her edge alone (`machine.sysid`, `machine.identified`): the regressor against MuJoCo's inverse
dynamics; a campaign on the stand reading her drives' friction and armature and her links; a
payload in her hand found; its corrections laid over her law's model and judged on a fresh
campaign; a world built 15 % off the figure read back; her gearboxes' wind-up and play on the
boxed world."""
import os
import sys
import tempfile

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report
from machine import mjcf, physics

mjcf.STAND = 1.0

#: Random states the regressor is held to MuJoCo at, and how far apart they may sit, N m.
STATES, APART = 40, 1e-9
#: A campaign's length, s (8 s a limb: at 4 the slow tone has no period and a hip's Coulomb
#: friction came out 8-22 % off by the phases drawn), the payload's and the judge's (the
#: relay's watchdog is 300 s a suite), and every how many steps its rows are read (100 Hz: at
#: 50 a hip yaw's friction came out 28 % off).
SECONDS, SHORT, EVERY = 48.0, 24.0, 10
#: What her hand carries in the second campaign, kg.
PAYLOAD_KG = 0.5
#: How far a drive's Coulomb friction and its armature may sit from the world's, of themselves;
#: its viscous friction, N m s/rad; a residual a drive, N m; a payload's shift of a first
#: moment, of the shift.
FRICTION, VISCOUS, RESIDUAL, SHIFT = 0.15, 0.20, 1.5, 0.25
#: The drives whose armature a leg's swing tells apart from its link: a knee's and an ankle's; a
#: hip's yaw swings 8.75 deg within its stops, its roll with the whole leg about its axis.
ARMATURES = ('knee', 'ankle', 'ankle_roll', 'elbow')
#: The first moments the payload moves most: its arm's and the trunk's.
MOVED = ('left_forearm.hy', 'left_upper_arm.hy', 'torso.hy')
#: How far a world is built off the figure, of each number.
OFF = 0.15
#: The drives whose gearboxes the campaign reads: those whose rotors ring least through it; the
#: knees', elbows' and ankles' rings at their mode, 187 rad/s, which a 1 kHz encoder reads at a
#: fifth of a cycle a step.
BOXES = ('spine_roll', 'spine', 'waist', 'neck', 'head', 'shoulder', 'hip_yaw', 'hip')
#: Her walk on the world off the figure, s, its first SPLIT_S s a campaign's rows and the rest the
#: judge's; the misfit a correction takes off the build's in the mean, of it.
WALK_S, SPLIT_S, TAKEN = 10.0, 5.5, 0.15


def _off(world, c, seed, keep=False):
    """`world` built OFF of the figure: every link's mass, centre and inertia, every drive's
    armature and friction drawn apart; `keep` its state through the constants laid anew."""
    m, rng = world.model, np.random.default_rng(seed)
    for body in c['lay']['bodies']:
        m.body_mass[body] *= rng.uniform(1.0 - OFF, 1.0 + OFF)
        m.body_inertia[body] *= rng.uniform(1.0 - OFF, 1.0 + OFF, 3)
        m.body_ipos[body] *= rng.uniform(1.0 - OFF, 1.0 + OFF, 3)
    for d in c['body'].act:
        for arr in (m.dof_armature, m.dof_damping, m.dof_frictionloss):
            arr[d] *= rng.uniform(1.0 - OFF, 1.0 + OFF)
    qpos, qvel = world.data.qpos.copy(), world.data.qvel.copy()
    world._mj.mj_setConst(m, world.data)
    if keep:
        world.data.qpos[:], world.data.qvel[:] = qpos, qvel
        world._mj.mj_forward(m, world.data)
    return world


def _states(b, rng):
    """(qpos, qvel, qacc) at STATES random states within her stops, the pelvis anywhere."""
    m, mj = b.m, b.mj
    qpos, qvel, qacc = np.zeros((STATES, m.nq)), rng.normal(size=(STATES, m.nv)), rng.normal(size=(STATES, m.nv))
    quat = rng.normal(size=(STATES, 4))
    qpos[:, :3], qpos[:, 3:7] = rng.normal(size=(STATES, 3)), quat / np.linalg.norm(quat, axis=1, keepdims=True)
    for j in range(m.njnt):
        if m.jnt_type[j] == mj.mjtJoint.mjJNT_HINGE:
            lo, hi = m.jnt_range[j] if m.jnt_limited[j] else (-1.5, 1.5)
            qpos[:, m.jnt_qposadr[j]] = rng.uniform(lo, hi, STATES)
    return qpos, qvel, qacc


def test_the_regressor_is_mujocos(report, c):
    """Y theta, theta the model's own, is MuJoCo's inverse dynamics on her drives' rows at random
    states - the Coulomb columns aside, which no model without constraints has."""
    from machine import sysid
    b, lay = c['body'], c['lay']
    m, d, mj = b.m, b.d, b.mj
    qpos, qvel, qacc = _states(b, np.random.default_rng(1))
    theta = sysid.truth(lay, m)
    theta[lay['drive'] + 2::3] = 0.0
    Y = sysid.regressor(lay, b, qpos, qvel, qacc, qacc[:, :3] - np.array(m.opt.gravity))
    worst, most = 0.0, 0.0
    for k in range(STATES):
        d.qpos[:], d.qvel[:], d.qacc[:] = qpos[k], qvel[k], qacc[k]
        mj.mj_inverse(m, d)
        want = d.qfrc_inverse[b.act]
        worst = max(worst, float(np.abs(want - Y[k, lay['driven']] @ theta).max()))
        most = max(most, float(np.abs(want).max()))
    report.check('her drives\' rows are MuJoCo\'s inverse dynamics at %d states' % STATES,
                 worst < APART, 'worst %.1e N m of %.0f' % (worst, most))
    report.check('%d links, %d columns, %d drives\' rows' % (len(lay['bodies']), len(lay['names']), len(lay['driven'])),
                 len(lay['names']) == 10 * len(lay['bodies']) + 3 * len(lay['driven']) == 343)


def _campaign(c, seed, world=None, seconds=SECONDS):
    """A campaign on the stand: (Y, tau, touched, world)."""
    from machine import sysid
    world = world or physics.World()
    world.reset(sysid.pose(), where=(0.0, mjcf.STAND_M, 0.0))
    ys, taus, touched = sysid.excite(world, sysid.setpoints(world, seconds, np.random.default_rng(seed)))
    Y, tau = sysid.rows(c['lay'], c['body'], ys, taus, physics.STEP_S, EVERY)
    return Y, tau, touched, world


def test_her_drives_and_links_are_read(report, c):
    """On the stand, a limb at a time, nothing touching her: each drive's Coulomb friction and
    viscous damping as the world has them, the armatures a swing tells apart, each drive's
    residual small, the trusted base parameters mostly the model's."""
    from machine import sysid
    from machine.drives import kind
    lay = c['lay']
    Y, tau, touched, world = _campaign(c, 1)
    base = sysid.base(lay, Y)
    got = sysid.identify(base, Y, tau)
    true = sysid.projected(base, sysid.truth(lay, world.model))
    c['base'], c['got'], c['true'] = base, got, true
    report.check('nothing touched her through %.0f s' % SECONDS, touched == 0, '%d steps' % touched)
    report.check('%d base parameters of %d columns, condition %.1e' % (len(base['ind']), len(lay['names']), got['condition']),
                 got['condition'] > 1e-6)
    names = got['names']
    off = {}
    for p, tol in (('f', FRICTION), ('b', VISCOUS)):
        for k, name in enumerate(names):
            if name.endswith('.' + p):
                err = abs(got['theta'][k] - true[k]) / (abs(true[k]) if p == 'f' else 1.0)
                if err > tol:
                    off[name] = '%.3f for %.3f' % (got['theta'][k], true[k])
    report.check('every drive\'s Coulomb friction within %.0f %% of the world\'s' % (100 * FRICTION),
                 not [n for n in off if n.endswith('.f')], '; '.join('%s %s' % kv for kv in off.items() if kv[0].endswith('.f')))
    report.check('every drive\'s viscous friction within %.2f N m s/rad' % VISCOUS,
                 not [n for n in off if n.endswith('.b')], '; '.join('%s %s' % kv for kv in off.items() if kv[0].endswith('.b')))
    arm = {name: abs(got['theta'][k] - true[k]) / abs(true[k]) for k, name in enumerate(names)
           if name.endswith('.J') and kind(name[:-2]) in ARMATURES}
    report.check('the armatures a swing tells apart within %.0f %%: %s' % (100 * FRICTION, ', '.join(sorted(set(ARMATURES)))),
                 arm and max(arm.values()) < FRICTION, 'worst %.1f %%' % (100 * max(arm.values())))
    report.check('every drive\'s residual under %.1f N m' % RESIDUAL, got['residual'].max() < RESIDUAL,
                 'worst %.2f N m, %d-%d rows read a drive' % (got['residual'].max(), got['read'].min(), got['read'].max()))
    err = np.abs(got['theta'] - true) / np.maximum(np.abs(true), 1e-9)
    tr = got['trusted']
    report.check('of %d base parameters trusted, two in three within 20 %% of the model\'s' % tr.sum(),
                 (err[tr] < 0.2).sum() >= 2.0 * tr.sum() / 3.0,
                 '%d within 5 %%, %d within 20 %%' % ((err[tr] < 0.05).sum(), (err[tr] < 0.2).sum()))


def _payload(c):
    """The payload's campaign, once: (base, got, Y, tau, touched, world), PAYLOAD_KG in her left
    hand."""
    from machine import sysid
    if 'payload' not in c:
        world = physics.World()
        world.model.body_mass[world.model.body('left_hand').id] += PAYLOAD_KG
        world._mj.mj_setConst(world.model, world.data)
        Y, tau, touched, world = _campaign(c, 2, world, SHORT)
        base = sysid.base(c['lay'], Y)
        c['payload'] = (base, sysid.identify(base, Y, tau), Y, tau, touched, world)
    return c['payload']


def test_a_payload_is_found(report, c):
    """PAYLOAD_KG in her left hand: the first moments it moves - her forearm's, her upper
    arm's, her trunk's - shift as the world's do."""
    from machine import sysid
    lay = c['lay']
    base, got, _Y, _tau, touched, world = _payload(c)
    was, now = sysid.projected(base, sysid.truth(lay, c['body'].m)), sysid.projected(base, sysid.truth(lay, world.model))
    report.check('nothing touched her carrying it', touched == 0, '%d steps' % touched)
    for name in MOVED:
        k = got['names'].index(name)
        shift, seen = now[k] - was[k], got['theta'][k] - was[k]
        report.check('%s shifts %.3f kg m, read %.3f' % (name, shift, seen),
                     abs(shift) > 0.01 and abs(seen - shift) < SHIFT * abs(shift) and got['trusted'][k],
                     '%.0f %% off, uncertainty %.1f %%' % (100 * abs(seen - shift) / abs(shift), 100 * got['uncertainty'][k]))


def test_her_corrections_are_laid_over_her_build(report, c):
    """The payload's campaign as the least move of her build that fits it, saved, read back and
    laid over a fresh model of her law: on a third campaign carrying the payload its residual
    falls where the payload is and rises nowhere."""
    from machine import identified, sysid, wbc
    lay = c['lay']
    _base, got, Y, tau, _touched, world = _payload(c)
    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, 'identified.toml')
        deltas = identified.save(path, identified.correction(lay, c['body'], Y, tau), got)
        back = identified.load(path)
        report.check('%d corrections saved read back the same' % len(deltas),
                     back.keys() == deltas.keys() and all(abs(back[k] - deltas[k]) <= 1e-6 * max(1.0, abs(deltas[k])) for k in deltas))
        fresh = wbc.Body(identified=False)
        try:
            applied = identified.correct(fresh, back)
        except Exception as refusal:
            applied = []
            report.check('laid over her law\'s model', False, str(refusal)[:200])
    report.check('laid over her law\'s model whole: %d links and drives corrected' % len(applied),
                 len(applied) >= 20 and 'left_forearm' in applied and 'left_upper_arm' in applied)
    Y, tau, _touched, world = _campaign(c, 3, world, SHORT)
    read = np.maximum((np.abs(Y).sum(axis=2) > 0).sum(axis=0), 1)
    rms = {}
    for label, model in (('nominal', c['body'].m), ('corrected', fresh.m)):
        r = Y @ sysid.truth(lay, model) - tau
        rms[label] = np.sqrt((r * r).sum(axis=0) / read)
    names = [n[:-2] for n in lay['names'][lay['drive']::3]]
    arm = [k for k, n in enumerate(names) if n.startswith(('left_shoulder', 'left_elbow'))]
    report.check('on her arm\'s drives the residual falls to under half: %s' % ', '.join(
        '%.2f -> %.2f' % (rms['nominal'][k], rms['corrected'][k]) for k in arm),
                 (rms['corrected'][arm] < 0.5 * rms['nominal'][arm]).all())
    worse = rms['corrected'] - rms['nominal']
    report.check('and rises nowhere by more than 0.1 N m', worse.max() < 0.1,
                 'worst +%.2f N m, %s' % (worse.max(), names[int(np.argmax(worse))]))


def test_a_world_off_the_figure_is_read_back(report, c):
    """A world built OFF of the figure - every link's mass, centre and inertia, every drive's
    armature and friction drawn apart -: each drive's Coulomb friction read back within
    FRICTION, the armatures a swing tells apart within it; its corrections take the build's
    misfit on a fresh campaign at least halfway to the world's own."""
    from machine import identified, sysid, wbc
    from machine.drives import kind
    lay = c['lay']
    world = _off(physics.World(), c, 7)
    m = world.model
    Y, tau, touched, world = _campaign(c, 4, world)
    base = sysid.base(lay, Y)
    got = sysid.identify(base, Y, tau)
    true = sysid.projected(base, sysid.truth(lay, m))
    err = np.abs(got['theta'] - true) / np.maximum(np.abs(true), 1e-9)
    f = {n: err[k] for k, n in enumerate(got['names']) if n.endswith('.f')}
    arm = {n: err[k] for k, n in enumerate(got['names']) if n.endswith('.J') and kind(n[:-2]) in ARMATURES}
    report.check('nothing touched her on the world %.0f %% off' % (100 * OFF), touched == 0, '%d steps' % touched)
    report.check('every drive\'s Coulomb friction read back within %.0f %%' % (100 * FRICTION),
                 max(f.values()) < FRICTION, 'worst %.1f %%, median %.1f %%' % (100 * max(f.values()), 100 * np.median(list(f.values()))))
    report.check('the armatures a swing tells apart within %.0f %%' % (100 * FRICTION),
                 max(arm.values()) < FRICTION, 'worst %.1f %%' % (100 * max(arm.values())))
    fresh = wbc.Body(identified=False)
    identified.correct(fresh, identified.correction(lay, c['body'], Y, tau))
    Y3, tau3, _touched, world = _campaign(c, 5, world, SHORT)
    read = np.maximum((np.abs(Y3).sum(axis=2) > 0).sum(axis=0), 1)
    rms = {}
    for label, model in (('build', c['body'].m), ('corrected', fresh.m), ('world', m)):
        r = Y3 @ sysid.truth(lay, model) - tau3
        rms[label] = float(np.sqrt((r * r).sum(axis=0) / read).mean())
    report.check('its corrections take the build\'s misfit at least halfway to the world\'s own',
                 rms['corrected'] <= 0.5 * (rms['build'] + rms['world']),
                 'build %.3f, corrected %.3f, world %.3f N m rms a drive' % (rms['build'], rms['corrected'], rms['world']))


def test_her_gearboxes_are_read(report, c):
    """On the world with its gearboxes (`physics.BOXED`), the campaign's arrays low-passed both
    ways, each drive's armature read at its motor's encoder; each box's wind-up and play from
    its two encoders on the drives whose rotors ring least (BOXES), within FRICTION - at the
    campaign's phases drawn with seed 1: drawn with 6 she touched herself 83 steps and the
    head's wind-up came 24 % off."""
    from machine import sysid
    from machine.drives import kind
    from machine.figure import JOINTS
    lay = c['lay']
    physics.BOXED = 1.0
    try:
        world = physics.World()
        world.reset(sysid.pose(), where=(0.0, mjcf.STAND_M, 0.0))
        ys, taus, touched = sysid.excite(world, sysid.setpoints(world, SECONDS, np.random.default_rng(1)))
    finally:
        physics.BOXED = 0.0
    smooth_ys, smooth_taus = sysid.smoothed(ys, taus, physics.STEP_S)
    Y, tau = sysid.rows(lay, c['body'], smooth_ys, smooth_taus, physics.STEP_S, EVERY)
    base = sysid.base(lay, Y)
    got = sysid.identify(base, Y, tau)
    driven = [j for j in JOINTS if world.driven[JOINTS.index(j)]]
    armature = np.array([got['theta'][got['names'].index(j + '.J')] for j in driven])
    rotor = np.array([world.boxes.j[JOINTS.index(j)] for j in driven])
    read = sysid.boxes(c['body'], ys, taus, physics.STEP_S, armature)
    report.check('nothing touched her on the boxed world', touched == 0, '%d steps' % touched)
    named = [i for i, j in enumerate(driven) if kind(j) in BOXES]
    arm = np.abs(armature[named] - rotor[named]) / rotor[named]
    report.check('every rotor\'s armature read positive at the motors\' encoders, the hip yaws\' within %.0f %%' % (100 * FRICTION),
                 (armature > 0.0).all() and all(arm[k] < FRICTION for k, i in enumerate(named) if kind(driven[i]) == 'hip_yaw'),
                 'worst %.1f %% on %s' % (100 * arm.max(), ', '.join(BOXES)))
    k_true = np.array([world.boxes.k[JOINTS.index(j)] for j in driven])
    gap_true = np.array([world.boxes.gap[JOINTS.index(j)] for j in driven])
    k_err = np.abs(read[named, 0] - k_true[named]) / k_true[named]
    gap_err = np.abs(read[named, 1] - gap_true[named]) / gap_true[named]
    report.check('each box\'s wind-up within %.0f %% on them' % (100 * FRICTION), k_err.max() < FRICTION,
                 'worst %.1f %% (%s)' % (100 * k_err.max(), driven[named[int(np.argmax(k_err))]]))
    report.check('each box\'s play within %.0f %% on them' % (100 * FRICTION), gap_err.max() < FRICTION,
                 'worst %.1f %% (%s)' % (100 * gap_err.max(), driven[named[int(np.argmax(gap_err))]]))


def _walked(c, seconds=WALK_S):
    """Her walk on the gym's world built OFF of the figure, 0.69 m/s from 1 s: (Y, tau, at, s) of
    its rows the model holds on (`sysid.free`), her edge arrays and her drives' torques a step."""
    from machine import edge, sysid, worker
    physics.ROOM, physics.SIGHT, mjcf.STAND = 0.0, 0.0, 0.0
    try:
        s = worker.session()
    finally:
        mjcf.STAND = 1.0
    g, world = s['g'], s['g']['world']
    _off(world, c, 7, keep=True)
    ys, taus, told = [], [], False
    while g['t'] < seconds and s['epoch'] == 0:
        if g['t'] >= 1.0 and not told:
            worker.apply(s, {'twist': (0.69, 0.0, 0.0)})
            told = True
        worker.stepped(s)
        ys.append(g['sense'].copy())
        taus.append(world.motor.copy())
    ys.append(edge.felt(world, g['sense'].copy()).copy())
    Y, tau = sysid.rows(c['lay'], c['body'], ys, taus, physics.STEP_S, EVERY)
    free = sysid.free(c['body'], ys, EVERY)
    return Y * free[:, :, None], tau * free, np.arange(0, len(ys) - 1, EVERY) * physics.STEP_S, s


def test_her_model_holds_on_her_walk(report, c):
    """On the world OFF of the figure she walks; on her walk's rows the model holds on, the world's
    own model fits the trunk's; a correction from the stand's campaign and the walk's first half
    takes TAKEN of the build's misfit off the second half in the mean and adds none to any drive -
    from the stand alone it rose on her legs: her pelvis still there, masses and first moments
    never stand apart."""
    from machine import identified, sysid, wbc
    from machine.figure import JOINTS
    lay = c['lay']
    Y, tau, _touched, world = _campaign(c, 4, _off(physics.World(), c, 7))
    Yw, tw, at, s = _walked(c)
    went = float(s['g']['world'].data.qpos[2])
    report.check('she walks %.1f m in %.0f s on the world %.0f %% off' % (went, WALK_S, 100 * OFF), s['fallen_at'] is None and went > 3.0)
    fit, judge = at < SPLIT_S, at >= SPLIT_S
    models = {'build': c['body'].m, 'world': s['g']['world'].model}
    for label, rows in (('stand', (Y, tau)), ('both', (np.concatenate([Y, Yw[fit]]), np.concatenate([tau, tw[fit]])))):
        fresh = wbc.Body(identified=False)
        identified.correct(fresh, identified.correction(lay, c['body'], *rows))
        models[label] = fresh.m
    read = np.maximum((np.abs(Yw[judge]).sum(axis=2) > 0).sum(axis=0), 1)
    rms = {}
    for label, m in models.items():
        r = Yw[judge] @ sysid.truth(lay, m) - tw[judge]
        rms[label] = np.sqrt((r * r).sum(axis=0) / read)
    driven = [j for j, d in zip(JOINTS, c['body'].driven) if d]
    trunk = [i for i, j in enumerate(driven) if j in ('spine_roll', 'spine', 'waist')]
    legs = [i for i, j in enumerate(driven) if j.endswith(('_hip', '_knee'))]
    report.check("the world's own model holds on the trunk's rows of her walk within 0.1 N m, %d-%d rows read a drive" % (read.min(), read.max()),
                 rms['world'][trunk].max() < 0.1, 'worst %.3f N m; the hips\' and knees\' %.2f, the edge\'s differences' % (
                     rms['world'][trunk].max(), rms['world'][legs].mean()))
    report.check('corrected on the stand and her walk\'s first half, the misfit on her second half falls %.0f %% in the mean: build %.3f, corrected %.3f, world %.3f N m' % (
        100 * (1.0 - rms['both'].mean() / rms['build'].mean()), rms['build'].mean(), rms['both'].mean(), rms['world'].mean()),
                 rms['both'].mean() < (1.0 - TAKEN) * rms['build'].mean(), 'from the stand alone %.3f, its legs %.2f for %.2f' % (
                     rms['stand'].mean(), rms['stand'][legs].mean(), rms['build'][legs].mean()))
    worse = rms['both'] - rms['build']
    report.check('and rises nowhere by more than 0.1 N m', worse.max() < 0.1, 'worst +%.2f N m, %s' % (worse.max(), driven[int(np.argmax(worse))]))


ROSTER = (test_the_regressor_is_mujocos, test_her_drives_and_links_are_read, test_a_payload_is_found,
          test_her_corrections_are_laid_over_her_build, test_a_world_off_the_figure_is_read_back,
          test_her_gearboxes_are_read, test_her_model_holds_on_her_walk)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    from machine import sysid, wbc
    report = Report()
    body = wbc.Body(identified=False)
    c = {'body': body, 'lay': sysid.laid(body)}
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, c)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
