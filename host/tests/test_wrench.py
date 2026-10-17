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
    from machine import wbc, wrench
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
                 0.0 < f[1] <= half + wrench.GROW_N and abs(given - f[1]) < 5.0
                 and np.hypot(f[0], f[2]) <= wbc.MU * f[1] * (1.0 + 1e-9),
                 '%.0f N on it, her soles %.0f N less, %.0f N across' % (f[1], given, np.hypot(f[0], f[2])))


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
    """A hop (`tools.sim.hop`): thrown, flown a tenth of a second, landed, stood."""
    from tools.sim import hop
    out = hop.hop(wrench=True)
    report.check('thrown, she flies, lands and stands',
                 out['stood'] and out['flights'] == 1 and out['flight_s'] >= 0.1 and out['clear_mm'] >= 12.0,
                 '%.3f s, %.1f mm under her soles, landed %d N, tilt %.1f deg, %s'
                 % (out['flight_s'], out['clear_mm'], out['landed_n'], out['tilt'], out['stances']))


ROSTER = (test_a_step_standing_is_the_stacks, test_a_contact_bears, test_she_stands_still, test_nudged_she_holds,
          test_she_hops)


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
