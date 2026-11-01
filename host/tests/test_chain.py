"""Her step in closed chains (`machine.stack.chain`) in C beside its python, the forces in their friction's box.

No sole standing, her pelvis (two points, one patch), shanks and hands bearing, her pelvis and trunk asked a
turn and a sole a swing: wbc_chain.c's torques, accelerations and contact forces are the python step's bit
for bit; rigid, the forces press alone within their friction's box with the complementarity met; the
plan turns her toward the ask through her drives; a tick's time."""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: States compared, and how far the C may sit from the python (N m, rad/s^2, N): the same products in the same order.
STEPS, APART = 12, 1e-9


def _product(a, b):
    """The quaternion a b (w, x, y, z): b turned by a."""
    import numpy as np
    return np.array([a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                     a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                     a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                     a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]])


def test_the_chain_step_in_c_is_the_steps(report):
    """wbc_chain.c beside `machine.stack.chain.step` on her stand's pose under random velocities, six contacts felt."""
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
    rigid = regime._replace(soft=0.0, sweeps=40000)
    rng, worst, boxed, turned, residuals = np.random.default_rng(7), np.zeros(3), 0.0, 0.0, []
    jp, jr = np.zeros((3, b.m.nv)), np.zeros((3, b.m.nv))
    bodies = [b.m.body(name).id for name in ('pelvis', 'pelvis', 'left_shank', 'left_hand', 'right_hand', 'right_shank',
                                             'left_forearm', 'right_forearm')]
    for _ in range(STEPS):
        qvel = np.zeros(len(world.data.qvel))
        qvel[:b.n] = 0.05 * rng.normal(size=b.n)
        s = wbc.sense(b, world.data.qpos, qvel)
        contacts = []
        for patch, body in enumerate(bodies):
            p = b.d.xpos[body] + 0.03 * rng.normal(size=3)
            b.mj.mj_jac(b.m, b.d, jp, jr, p, body)
            J = jp[:, :b.n].copy()
            b.mj.mj_jacDot(b.m, b.d, jp, jr, p, body)
            f = np.array([0.3, 1.0, 0.2]) * (60.0 + 80.0 * rng.random()) + 10.0 * rng.normal(size=3)
            contacts.append({'p': p, 'f': f, 'J': J, 'dJv': jp[:, :b.n] @ s['v'], 'patch': 2 * patch, 'side': 0, 'body': body})
        # a turn of 0.2 rad about her pelvis's patch normal, the torsion its points leave free
        axis = contacts[0]['f'] + contacts[1]['f']
        axis = axis / np.linalg.norm(axis)
        spin = np.array([np.cos(0.1), *(np.sin(0.1) * axis)])
        turns = tuple(_product(spin, t['quat']) for t in s['turns'])
        f1 = s['feet'][1]
        ask = {'stance': (False, False), 'com_acc': np.zeros(3), 'turns': turns, 'wep': False,
               'swing': {1: (f1['sole'] + (0.0, 0.05, 0.0), np.zeros(3), np.zeros(3), f1['quat'])},
               'posture': s['q'] + 0.02 * rng.normal(size=len(s['q']))}
        sc = dict(s, contacts=contacts)
        ours, theirs = chain.step(b, sc, ask, regime), solve(b, sc, ask)
        worst = np.maximum(worst, [np.abs(ours['tau'] - theirs['tau']).max(), np.abs(ours['qacc'] - theirs['qacc']).max(),
                                   np.abs(ours['borne'] - theirs['borne']).max()])
        hard = chain.step(b, sc, ask, rigid)
        residuals.append(hard['residual'])
        for Q, z in zip(chain.frames(np.concatenate([cs['f'] for cs in contacts])), hard['borne']):
            zn, zt1, zt2 = Q @ z
            boxed = max(boxed, -zn, abs(zt1) - regime.mu * zn, abs(zt2) - regime.mu * zn)
        # the plan answers a turn through her drives and the contacts it holds: still, nothing else asked, a turn of
        # 0.5 rad about her pelvis's normal, what it foresees of her base goes the asked way (on her pelvis's two points
        # alone it foresees none: pinned, they slide only past their box, and no lever asks that of them - her shank
        # and hands are the levers, and how far they take her is the state's)
        still = wbc.sense(b, world.data.qpos, np.zeros(len(world.data.qvel)))
        wide = np.array([np.cos(0.25), *(np.sin(0.25) * axis)])
        want = tuple(_product(wide, t['quat']) for t in still['turns'])
        keen = chain.step(b, dict(still, contacts=contacts), dict(ask, turns=want, swing={}, posture=still['q'].copy()),
                          regime._replace(task=0.05))
        fore = keen['qacc'].copy()
        fore[np.flatnonzero((np.arange(b.n) < 6) | b.passive)] = keen['foreseen']
        t0 = still['turns'][0]
        asked = regime.tkp * wbc.turn_error(np.asarray(want[0]), t0['quat']) - t0['dJr']
        turned = max(turned, 1.0 - float((t0['Jr'] @ fore + t0['dJr']) @ asked) / float(asked @ asked))
    report.check("the closed chains' step in C is machine.stack.chain's over %d states" % STEPS, bool((worst < APART).all()),
                 '%.1e N m, %.1e rad/s^2, %.1e N apart at most' % tuple(worst))
    report.check("rigid, its forces press alone within their friction's box, the complementarity met",
                 boxed < 1e-9 and max(residuals) < 1e-3, 'box %.1e N, residual %.1e' % (boxed, max(residuals)))
    report.check("the plan turns her pelvis the asked way through her drives and the contacts it holds",
                 turned < 1.0, 'the turn short of the ask by %.2f at worst' % turned)
    t = time.perf_counter()
    for _ in range(200):
        solve(b, sc, ask)
    us = 1e6 * (time.perf_counter() - t) / 200
    report.check('a tick of it on this host', us < 1000.0, '%.1f us' % us)


def test_the_step_is_hers_by_her_release(report):
    """`chain.chained`: a leg let go or no sole standing, and no sole asked to land; a sit or a rise on both soles and a
    bounce asked to land stay the upright step's. `chain.bearing` hands the chain a standing sole as a patch."""
    import numpy as np
    from machine.stack import chain
    picks = [chain.chained({'stance': st, 'let_go': lg, 'swing': sw}) for st, lg, sw in (
        ((True, False), (False, True), {}), ((False, False), (False, False), {}), ((False, False), (True, True), {}),
        ((True, True), (False, False), {}), ((False, False), (False, False), {0: None, 1: None}), ((True, False), (False, False), {1: None}))]
    report.check("hers with a leg let go, none standing or both let go; the upright step's on both soles or a landing asked",
                 picks == [1, 1, 1, 0, 0, 0], str(picks))
    n = 20
    feet = [{'sole': np.zeros(3), 'J6': np.arange(6.0 * n).reshape(6, n), 'dJv': np.arange(6.0)} for _k in range(2)]
    got = chain.bearing({'feet': feet, 'contacts': [{'body': 7}]}, (120.0, 0.0), (True, False), 1)
    report.check("a standing sole a patch of its own: its load up, its Jacobian's translation, a body below 0",
                 len(got) == 2 and got[1]['body'] == -1 and got[1]['f'][1] == 120.0 and got[1]['J'].shape == (3, n)
                 and np.array_equal(got[1]['J'], feet[0]['J6'][3:]) and len(chain.bearing({'feet': feet, 'contacts': []}, (120.0, 0.0), (True, False), 0)) == 0,
                 str([(c.get('body'), c['f'][1] if 'f' in c else None) for c in got]))


ROSTER = (test_the_chain_step_in_c_is_the_steps, test_the_step_is_hers_by_her_release)


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
