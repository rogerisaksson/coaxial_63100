"""Her step in closed chains (`machine.stack.chain`) in C beside its python, the forces in their friction's box.

No sole standing, her pelvis, shanks and hands bearing: wbc_chain.c's torques, accelerations and contact
forces are the python step's bit for bit; the forces press alone within their friction's box with the
complementarity met; her posture's ask moves the contacts' points next to nothing; a tick's time."""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: States compared, and how far the C may sit from the python (N m, rad/s^2, N): the same products in the same order.
STEPS, APART = 12, 1e-9


def test_the_chain_step_in_c_is_the_steps(report):
    """wbc_chain.c beside `machine.stack.chain.step` on her stand's pose under random velocities, five contacts felt."""
    import numpy as np
    from machine.body import figure
    from machine.stack import chain, wbc
    from machine.world import physics
    from tools.cores import step as glue
    world, b = physics.World(), wbc.Body()
    world.reset(figure.stand())
    regime, solve = glue.regime(), glue.chain_laid(b)
    # the text's regime gives a mattress's foam its pass of lag (its softness): the complementarity itself is the
    # rigid regime's to meet, swept to convergence
    rigid = regime._replace(soft=0.0, sweeps=800)
    rng, worst, boxed, moved, residuals = np.random.default_rng(7), np.zeros(3), 0.0, 0.0, []
    jp, jr = np.zeros((3, b.m.nv)), np.zeros((3, b.m.nv))
    bodies = [b.m.body(name).id for name in ('pelvis', 'left_shank', 'right_shank', 'left_hand', 'right_hand')]
    for _ in range(STEPS):
        qvel = np.zeros(len(world.data.qvel))
        qvel[:b.n] = 0.3 * rng.normal(size=b.n)
        s = wbc.sense(b, world.data.qpos, qvel)
        contacts = []
        for patch, body in enumerate(bodies):
            p = b.d.xpos[body] + 0.03 * rng.normal(size=3)
            b.mj.mj_jac(b.m, b.d, jp, jr, p, body)
            J = jp[:, :b.n].copy()
            b.mj.mj_jacDot(b.m, b.d, jp, jr, p, body)
            f = np.array([0.3, 1.0, 0.2]) * (60.0 + 80.0 * rng.random()) + 10.0 * rng.normal(size=3)
            contacts.append({'p': p, 'f': f, 'J': J, 'dJv': jp[:, :b.n] @ s['v'], 'patch': 2 * patch, 'side': 0})
        ask = {'stance': (False, False), 'com_acc': np.zeros(3), 'turns': (), 'wep': False,
               'posture': s['q'] + 0.05 * rng.normal(size=len(s['q']))}
        sc = dict(s, contacts=contacts)
        ours, theirs = chain.step(b, sc, ask, regime), solve(b, sc, ask)
        worst = np.maximum(worst, [np.abs(ours['tau'] - theirs['tau']).max(), np.abs(ours['qacc'] - theirs['qacc']).max(),
                                   np.abs(ours['borne'] - theirs['borne']).max()])
        hard = chain.step(b, sc, ask, rigid)
        residuals.append(hard['residual'])
        for Q, z in zip(chain.frames(np.concatenate([cs['f'] for cs in contacts])), hard['borne']):
            zn, zt1, zt2 = Q @ z
            boxed = max(boxed, -zn, abs(zt1) - regime.mu * zn, abs(zt2) - regime.mu * zn)
        Ja = np.vstack([cs['J'] for cs in contacts])[:, b.act]
        want = wbc.POSTURE_KD[0] * (ask['posture'] - s['q']) - wbc.POSTURE_KD[1] * s['v'][b.act]
        moved = max(moved, float(np.abs(Ja @ ours['qacc'][b.act]).max() / (ours['scale'] * np.abs(Ja @ want).max())))
    report.check("the closed chains' step in C is machine.stack.chain's over %d states" % STEPS, bool((worst < APART).all()),
                 '%.1e N m, %.1e rad/s^2, %.1e N apart at most' % tuple(worst))
    report.check("rigid, its forces press alone within their friction's box, the complementarity met",
                 boxed < 1e-9 and max(residuals) < 1e-9, 'box %.1e N, residual %.1e' % (boxed, max(residuals)))
    report.check("her posture's ask moves the contacts' points next to nothing", moved < 0.2, 'ratio %.3f' % moved)
    t = time.perf_counter()
    for _ in range(200):
        solve(b, sc, ask)
    us = 1e6 * (time.perf_counter() - t) / 200
    report.check('a tick of it on this host', us < 1000.0, '%.1f us' % us)


ROSTER = (test_the_chain_step_in_c_is_the_steps,)


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
