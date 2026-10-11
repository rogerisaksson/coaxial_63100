"""Her step without an active set (`machine.wrench`) in the stack's place (`tools/sim/wbc.py
--wrench`): one step standing beside the stack's, her stand, a nudge held, her walk, a hop."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen  # noqa: E402
from gynoid_kit import Report  # noqa: E402

#: A step standing: its torques within APART_NM of the stack's, each sole's load within APART_N.
APART_NM, APART_N = 0.5, 2.0


def test_a_step_standing_is_the_stacks(report):
    """Her stand's first pass, the capture point's law: the torques and the soles' loads of
    `wrench.step` beside `wbc.step`'s."""
    import numpy as np
    from machine import balance, gait, physics, wbc, wrench
    from tools.sim import wbc as sim
    world, body = physics.World(), wbc.Body()
    world.reset(gait.stand())
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    world.reset(gait.stand(), where=(0.0, 1.0 - min(float(f['pts'][:, 1].min()) for f in s['feet']), 0.0))
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    mid = (s['feet'][0]['sole'][[0, 2]] + s['feet'][1]['sole'][[0, 2]]) / 2.0
    ask = sim.law(s, balance.state(s, body.knees), mid)
    ours, theirs = wrench.step(body, s, ask), wbc.step(body, s, ask)
    torque = float(np.abs(ours['tau'] - theirs['tau']).max())
    load = max(abs(a - b) for a, b in zip(ours['bears'], theirs['bears']))
    report.check('standing, a step asks the drives and the floor what the stack does',
                 torque < APART_NM and load < APART_N, '%.2f N m and %.1f N apart at most' % (torque, load))


def test_she_stands_still(report):
    """Three seconds standing: up, every drive inside its clamp, no step."""
    from tools.sim import wbc
    out = wbc.stand(seconds=3.0, wrench=True)
    report.check('standing 3 s she stays up, every drive within its clamp',
                 out['stood'] and out['top'] <= 1.0 and not out['steps'] and out['tilt'] < 2.0,
                 'tilt %.1f deg, drift %.1f mm, the most a clamp %.2f, %d us a step'
                 % (out['tilt'], out['drift_mm'], out['top'], out['us']))


def test_nudged_she_holds_without_a_step(report):
    """60 N for 0.12 s from her right and from her left, 38 N from behind: up, no step taken."""
    from tools.sim import wbc
    for push, way, name in ((60.0, 0.0, 'toward her left'), (60.0, 180.0, 'toward her right'),
                            (38.0, 90.0, 'from behind')):
        out = wbc.stand(push, way, wrench=True)
        report.check('nudged %.0f N %s she holds, no step' % (push, name),
                     out['stood'] and not out['steps'], 'tilt %.1f deg, %d steps' % (out['tilt'], out['steps']))


def test_she_walks(report):
    """The MPC's walk, 8 s at 0.5 m/s asked: up at its end, every step taken, over 1.5 m."""
    from tools.sim import wbc
    out = wbc.stand(seconds=wbc.AT_S + 8.0, walk=(0.5, 0.5), wrench=True)
    report.check('she walks 8 s and stands', out['stood'] and out['steps'] >= 14 and out['walked_m'] >= 1.5,
                 '%d steps, %.2f m, %.2f m/s, %d J/m, tilt %.1f deg'
                 % (out['steps'], out['walked_m'], out['m_s'], out['j_m'], out['tilt']))


def test_she_hops(report):
    """A hop (`tools.sim.hop`): thrown, flown a tenth of a second, landed, stood."""
    from tools.sim import hop
    out = hop.hop(wrench=True)
    report.check('thrown, she flies, lands and stands',
                 out['stood'] and out['flights'] == 1 and out['flight_s'] >= 0.1 and out['clear_mm'] >= 12.0,
                 '%.3f s, %.1f mm under her soles, landed %d N, tilt %.1f deg, %s'
                 % (out['flight_s'], out['clear_mm'], out['landed_n'], out['tilt'], out['stances']))


ROSTER = (test_a_step_standing_is_the_stacks, test_she_stands_still, test_nudged_she_holds_without_a_step,
          test_she_walks, test_she_hops)


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
