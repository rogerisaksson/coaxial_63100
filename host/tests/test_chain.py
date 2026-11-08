"""Her step in closed chains (`machine.stack.chain`) in C beside its python, one QP within friction.

No sole standing, her pelvis (two points, one patch), shanks and hands bearing, her pelvis and trunk asked a
turn and a sole a swing: wbc_chain.c's torques, accelerations and contact forces are the python step's bit
for bit; wherever the QP met its laws the forces press alone within their friction's cones and her torques
within their clamps; the QP turns her toward the ask through her drives; told a twist her seat slides at
its friction's edge and still bears her; a tick's time."""
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


def _lawful(contacts, got, regime, limit):
    """How far the step's forces and torques break its laws, N and N m: a pull, a tangent past its cone (her sled's
    `regime.mu`, an anchor's `regime.anchor`), a torque past its clamp."""
    import numpy as np
    from machine.senses.borne import CARPAL
    from machine.stack import chain
    worst = 0.0
    for c, z, Q in zip(contacts, got['borne'], chain.frames(np.concatenate([cs['n'] for cs in contacts]))):
        zn, zt1, zt2 = Q @ z
        mu = regime.mu if int(c['body']) >= 0 and int(c['patch']) < CARPAL else regime.anchor
        worst = max(worst, -zn, float(np.hypot(zt1, zt2)) - mu * zn * (1.0 + 1e-9))
    return max(worst, float((np.abs(got['tau']) - limit).max()))


def test_the_chain_step_in_c_is_the_steps(report):
    """wbc_chain.c beside `machine.stack.chain.step` on her stand's pose under random velocities, six contacts felt."""
    import numpy as np
    from machine.body import figure
    from machine.stack import chain, wbc
    from machine.stack.shrunk import box
    from machine.world import physics
    from tools.cores import step as glue
    world, b = physics.World(), wbc.Body()
    world.reset(figure.stand())
    regime, solve = glue.regime(), glue.chain_laid(b)
    rng, worst, lawless, turned, met = np.random.default_rng(7), np.zeros(3), 0.0, 0.0, 0

    def paired(sc, ask):
        """(the python's, the C's) from cold: the rows each held last warm the next, so a pair compared starts alike."""
        b.chain_warm, solve.args[0].chain_nheld[0], solve.args[0].chain_nheld[1] = ((), ()), 0, 0
        return chain.step(b, sc, ask, regime), solve(b, sc, ask)
    # her pelvis and right shank on a foam as her skin feels it (borne's give: 21 m/s^2 a N), her hands and forearms on
    # what is firm (none): the C the same
    gives, apart_g = (21.0, 21.0, 0.0, 0.0, 0.0, 21.0, 0.0, 0.0), 0.0
    jp, jr = np.zeros((3, b.m.nv)), np.zeros((3, b.m.nv))
    bodies = [b.m.body(name).id for name in ('pelvis', 'pelvis', 'left_shank', 'left_hand', 'right_hand', 'right_shank',
                                             'left_forearm', 'right_forearm')]
    for nth in range(STEPS):
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
            # her left shank's patch a let-go left foot's (borne.LET's), told free every other state
            contacts.append({'p': p, 'f': f, 'n': f / np.linalg.norm(f), 'J': J, 'dJv': jp[:, :b.n] @ s['v'], 'patch': 2 * patch if patch != 2 else 20, 'side': 0,
                             'body': body})
        # a turn of 0.2 rad about her pelvis's patch normal, the torsion its points leave free
        axis = contacts[0]['f'] + contacts[1]['f']
        axis = axis / np.linalg.norm(axis)
        spin = np.array([np.cos(0.1), *(np.sin(0.1) * axis)])
        turns = tuple(_product(spin, t['quat']) for t in s['turns'])
        f1 = s['feet'][1]
        ask = {'stance': (False, False), 'com_acc': np.zeros(3), 'turns': turns, 'wep': False,
               'swing': {1: (f1['sole'] + (0.0, 0.05, 0.0), np.zeros(3), np.zeros(3), f1['quat'])},
               'posture': s['q'] + 0.02 * rng.normal(size=len(s['q'])), 'twist': (0.3 * (nth % 3 - 1), 0.0, 0.0),
               'free': (nth % 2 == 1, False)}
        sc = dict(s, contacts=contacts)
        ours, theirs = paired(sc, ask)
        soft = dict(s, contacts=[dict(c, give=g) for c, g in zip(contacts, gives)])
        ours_g, theirs_g = paired(soft, ask)
        apart_g = max(apart_g, float(np.abs(ours_g['tau'] - theirs_g['tau']).max()),
                      float(np.abs(ours_g['borne'] - theirs_g['borne']).max()))
        worst = np.maximum(worst, [np.abs(ours['tau'] - theirs['tau']).max(), np.abs(ours['qacc'] - theirs['qacc']).max(),
                                   np.abs(ours['borne'] - theirs['borne']).max()])
        if ours['met']:
            met += 1
            lawless = max(lawless, _lawful(contacts, ours, regime, box(b, ask)[0]))
        # the QP answers a turn through her drives and the contacts it holds: still, nothing else asked, a turn of
        # 0.5 rad about her pelvis's normal, what it foresees of her base goes the asked way (on her pelvis's two points
        # alone it foresees none: pinned, they slide only past their cone, and no lever asks that of them - her shank
        # and hands are the levers, and how far they take her is the state's)
        still = wbc.sense(b, world.data.qpos, np.zeros(len(world.data.qvel)))
        wide = np.array([np.cos(0.25), *(np.sin(0.25) * axis)])
        want = tuple(_product(wide, t['quat']) for t in still['turns'])
        keen = chain.step(b, dict(still, contacts=contacts), dict(ask, turns=want, swing={}, posture=still['q'].copy(), twist=(0.0, 0.0, 0.0),
                                                                    free=(False, False)),
                          regime._replace(task=0.05))
        fore = keen['qacc'].copy()
        fore[np.flatnonzero((np.arange(b.n) < 6) | b.passive)] = keen['foreseen']
        t0 = still['turns'][0]
        asked = regime.tkp * wbc.turn_error(np.asarray(want[0]), t0['quat']) - t0['dJr']
        turned = max(turned, 1.0 - float((t0['Jr'] @ fore + t0['dJr']) @ asked) / float(asked @ asked))
    report.check("the closed chains' step in C is machine.stack.chain's over %d states" % STEPS, bool((worst < APART).all()),
                 '%.1e N m, %.1e rad/s^2, %.1e N apart at most' % tuple(worst))
    report.check("what bears her giving as her skin feels it, the C the same", apart_g < APART,
                 '%.1e apart at most' % apart_g)
    report.check("where the QP met its laws, its forces press alone within their cones and her torques within their clamps",
                 met > 0 and lawless < 1e-6, '%d of %d states met, the laws broken by %.1e at most' % (met, STEPS, lawless))
    report.check("the QP turns her pelvis the asked way through her drives and the contacts it holds",
                 turned < 1.0, 'the turn short of the ask by %.2f at worst' % turned)
    # told a twist, her seat slides at its friction's edge against the way told and still bears her, her hands beside it
    left, up, way = chain.ways(still['turns'][0]['quat'])
    weight, sled = float(b.m.body_subtreemass[b.m.body('pelvis').id]) * 9.81, []
    for name, off, share, patch in ([('pelvis', s * 0.07 * left - 0.08 * up - 0.03 * way, 0.40, 0) for s in (1.0, -1.0)]
                                    + [(n, np.zeros(3), 0.10, 6 + 3 * k) for k, n in enumerate(('left_hand', 'right_hand'))]):
        body = b.m.body(name).id
        p = b.d.xpos[body] + off
        b.mj.mj_jac(b.m, b.d, jp, jr, p, body)
        J = jp[:, :b.n].copy()
        b.mj.mj_jacDot(b.m, b.d, jp, jr, p, body)
        sled.append({'p': p, 'f': share * weight * up, 'n': up, 'J': J, 'dJv': jp[:, :b.n] @ still['v'], 'patch': patch, 'side': 0, 'body': body})
    seats, apart = [], 0.0
    for vx in (0.0, 0.3):
        sc, sat = dict(still, contacts=sled), dict(ask, turns=tuple(t['quat'] for t in still['turns']), swing={}, posture=still['q'].copy(),
                                                    twist=(vx, 0.0, 0.0), free=(False, False))
        ours, theirs = paired(sc, sat)
        apart = max(apart, float(np.abs(ours['tau'] - theirs['tau']).max()), float(np.abs(ours['borne'] - theirs['borne']).max()))
        seats.append(sum(float(up @ z) for z in ours['borne'][:2]))
    report.check("told a twist, her seat slides at its friction's edge, bearing her still, the C the same",
                 seats[1] >= 0.5 * seats[0] and apart < APART, 'held %.0f N, sliding %.0f N; C %.1e apart' % (seats[0], seats[1], apart))
    t = time.perf_counter()
    for _ in range(200):
        solve(b, sc, ask)
    us = 1e6 * (time.perf_counter() - t) / 200
    report.check('a tick of it on this host', us < 5000.0, '%.1f us' % us)


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
