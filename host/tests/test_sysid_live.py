#!/usr/bin/env python3
"""Her identification off the stand, on her law: her model held to her walk on a world off the figure;
standing on her insoles, her weight and its moments."""
import sys

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report
from machine.body import mjcf
from machine.world import physics
from test_sysid import EVERY, OFF, SHORT, _campaign, _off

#: Her walk on the world off the figure, s, its first SPLIT_S s a campaign's rows and the rest the
#: judge's; of what the world's own model takes off the build's misfit in the mean, what a
#: correction takes - 0.57 here, 0.62 on CI's Linux, where of the build's whole it took 17 and 10
#: %: the platform's arithmetic moves the floor no model takes; her stand swaying, s.
WALK_S, SPLIT_S, TAKEN, STAND_S = 10.0, 5.5, 0.5, 8.0
#: Her insoles' rows against the world's own model: her weight within this of itself, the moments
#: within this of their swing or ABOUT_NM, N m - a sway's moment across 1.9 N m, along 18.
WEIGHT, SWUNG, ABOUT_NM = 0.01, 0.05, 0.2


def _run(c, command, seconds, off):
    """Her run on the gym's world, `command` heard at 1 s, built `off` the figure or not: (ys, taus,
    s) her edge arrays and her drives' torques a step, the session."""
    from machine.senses import edge
    from machine.process import worker
    physics.ROOM, physics.SIGHT, mjcf.STAND = 0.0, 0.0, 0.0
    try:
        s = worker.session()
    finally:
        mjcf.STAND = 1.0
    g, world = s['g'], s['g']['world']
    if off:
        _off(world, c, 7, keep=True)
    ys, taus, told = [], [], False
    while g['t'] < seconds and s['epoch'] == 0:
        if g['t'] >= 1.0 and not told:
            worker.apply(s, command)
            told = True
        worker.stepped(s)
        ys.append(g['sense'].copy())
        taus.append(world.motor.copy())
    ys.append(edge.felt(world, g['sense'].copy()).copy())
    return ys, taus, s


def test_her_model_holds_on_her_walk(report, c):
    """On the world OFF of the figure she walks; on her walk's rows the model holds on, the world's
    own model fits the trunk's; a correction from the stand's campaign and the walk's first half
    takes TAKEN of what the world's own model takes off the build's misfit on the second half, in
    the mean, and adds none to any drive - from the stand alone it rose on her legs: her pelvis
    still there, masses and first moments never stand apart."""
    from machine.identify import identified, sysid
    from machine.stack import wbc
    lay = c['lay']
    Y, tau, _touched, world = _campaign(c, 4, _off(physics.World(), c, 7))
    ys, taus, s = _run(c, {'twist': (0.69, 0.0, 0.0)}, WALK_S, True)
    Yw, tw = sysid.rows(lay, c['body'], ys, taus, physics.STEP_S, EVERY)
    free = sysid.free(lay, ys, EVERY)
    Yw, tw, at = Yw * free[:, :, None], tw * free, np.arange(0, len(ys) - 1, EVERY) * physics.STEP_S
    went = float(s['g']['world'].data.qpos[2])
    report.check('she walks %.1f m in %.0f s on the world %.0f %% off' % (went, WALK_S, 100 * OFF), s['fallen_at'] is None and went > 3.0)
    fit, judge = at < SPLIT_S, at >= SPLIT_S
    models = {'build': c['body'].m, 'world': s['g']['world'].model}
    for label, rows in (('stand', (Y, tau)), ('both', (np.concatenate([Y, Yw[fit]]), np.concatenate([tau, tw[fit]])))):
        fresh = wbc.Body(identified=False)
        identified.correct(fresh, identified.correction(lay, c['body'], *rows))
        models[label] = fresh.m
    read = sysid.read(Yw[judge])
    rms = {label: sysid.misfit(Yw[judge], tw[judge], sysid.truth(lay, m)) for label, m in models.items()}
    driven = lay['drives']
    trunk = [i for i, j in enumerate(driven) if j in ('spine_roll', 'spine', 'waist')]
    legs = [i for i, j in enumerate(driven) if j.endswith(('_hip', '_knee'))]
    report.check("the world's own model holds on the trunk's rows of her walk within 0.1 N m, %d-%d rows read a drive" % (read.min(), read.max()),
                 rms['world'][trunk].max() < 0.1, 'worst %.3f N m; the hips\' and knees\' %.2f, the edge\'s differences' % (
                     rms['world'][trunk].max(), rms['world'][legs].mean()))
    build, both, own = rms['build'].mean(), rms['both'].mean(), rms['world'].mean()
    report.check('corrected on the stand and her walk\'s first half, the misfit on her second half falls %.2f of the way to the world\'s own: build %.3f, corrected %.3f, world %.3f N m' % (
        (build - both) / (build - own), build, both, own),
                 build - both >= TAKEN * (build - own), 'from the stand alone %.3f, its legs %.2f for %.2f' % (
                     rms['stand'].mean(), rms['stand'][legs].mean(), rms['build'][legs].mean()))
    worse = rms['both'] - rms['build']
    report.check('and rises nowhere by more than 0.1 N m', worse.max() < 0.1, 'worst +%.2f N m, %s' % (worse.max(), driven[int(np.argmax(worse))]))


def test_her_weight_and_its_moments_are_read(report, c):
    """Standing on her law and swaying, her insoles' three rows (`sysid.borne`) are the world's
    own model's - her weight within WEIGHT of itself, the moments within SWUNG of their swing -,
    and beside the stand's campaign they add her pelvis's own to the base parameters: her mass,
    and a first moment of her pelvis within TRUST of itself."""
    from machine.identify import sysid
    lay = c['lay']
    ys, _taus, s = _run(c, {'say': 1}, STAND_S, False)
    Yb, fb = sysid.borne(lay, c['body'], ys, physics.STEP_S, EVERY)
    pred = Yb @ sysid.truth(lay, s['g']['world'].model)
    apart = np.sqrt(np.mean((pred - fb) ** 2, axis=0))
    weight = c['body'].mass * 9.81 * sysid.LEVER_M
    report.check('swaying %.0f s her insoles read her weight %.1f for %.1f N, %.3f N m rms apart at %.1f m' % (
        STAND_S, fb[:, 0].mean() / sysid.LEVER_M, weight / sysid.LEVER_M, apart[0], sysid.LEVER_M),
                 s['fallen_at'] is None and apart[0] < WEIGHT * weight)
    report.check('and its moments about the point under her pelvis within %.0f %% of their swing or %.1f N m: %.2f and %.2f N m rms on %.1f and %.1f' % (
        100 * SWUNG, ABOUT_NM, apart[1], apart[2], fb[:, 1].std(), fb[:, 2].std()),
                 (apart[1:] < np.maximum(SWUNG * fb[:, 1:].std(axis=0), ABOUT_NM)).all())
    Y, tau, _touched, _world = _campaign(c, 2, seconds=SHORT)
    rows_ = np.concatenate([Y.reshape(-1, Y.shape[2]), Yb.reshape(-1, Yb.shape[2])])[:, None, :]
    told = np.concatenate([tau.reshape(-1), fb.reshape(-1)])[:, None]
    alone, both = sysid.base(lay, Y), sysid.base(lay, rows_)
    got = sysid.identify(both, rows_, told)
    true = sysid.projected(both, sysid.truth(lay, c['body'].m))
    k = got['names'].index('pelvis.m') if 'pelvis.m' in got['names'] else None
    moments = [i for i, n in enumerate(got['names']) if n.startswith('pelvis.h') and got['trusted'][i]]
    report.check('beside the stand\'s campaign they add her pelvis\'s own to the base parameters, %d -> %d: her mass %.2f for %.2f kg, %d first moment trusted' % (
        len(alone['ind']), len(both['ind']), got['theta'][k] if k is not None else 0.0, c['body'].mass, len(moments)),
                 k is not None and abs(got['theta'][k] - true[k]) < 0.01 * true[k] and len(moments) >= 1
                 and all(abs(got['theta'][i] - true[i]) < sysid.TRUST * abs(true[i]) for i in moments),
                 ', '.join('%s %.3f for %.3f' % (got['names'][i], got['theta'][i], true[i]) for i in moments))


ROSTER = (test_her_model_holds_on_her_walk, test_her_weight_and_its_moments_are_read)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    from machine.identify import sysid
    from machine.stack import wbc
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
