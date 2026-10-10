"""Her balance as one convex QP a step (`machine.wbc`, `machine.mpc`, `machine.balance`): the
solver against its optimality, her stand still on the stack, a nudge held, a shove stepped out of
- in a world without buses (`tools/sim/wbc.py`)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen  # noqa: E402
from gynoid_kit import Report  # noqa: E402

#: Random problems the solver meets KKT on, and its worst residual allowed.
PROBLEMS, KKT = 200, 1e-8


def test_the_solver_meets_its_optimality(report):
    """Random strictly convex QPs, cold and warm from the set they ended on: stationary, feasible,
    multipliers pressing, slack where a row is loose; the warm solve the cold one's."""
    import numpy as np
    from machine import qp
    rng = np.random.default_rng(7)
    worst, apart, fails = 0.0, 0.0, 0
    for _ in range(PROBLEMS):
        n, m = int(rng.integers(2, 40)), int(rng.integers(0, 90))
        R = rng.normal(size=(n, n))
        H, g = R.T @ R + 0.1 * np.eye(n), 10.0 * rng.normal(size=n)
        A = rng.normal(size=(m, n))
        b = A @ rng.normal(size=n) + rng.uniform(0.0, 1.0, size=m)
        x, held, _u = qp.solve(H, g, A, b)
        if x is None:
            fails += 1
            continue
        Ah = A[list(held)]
        lam = (np.linalg.lstsq(Ah.T, -(H @ x + g), rcond=None)[0] if len(held)
               else np.zeros(0))
        scale = 1.0 + float(np.abs(g).max())
        worst = max(worst, float(np.abs(H @ x + g + Ah.T @ lam).max()) / scale,
                    max(0.0, float((A @ x - b).max())) if m else 0.0,
                    max(0.0, -float(lam.min())) if len(lam) else 0.0)
        g2 = g + 0.1 * rng.normal(size=n)
        warm, _h, _u = qp.solve(H, g2, A, b, held)
        cold, _h, _u = qp.solve(H, g2, A, b)
        if warm is not None and cold is not None:
            apart = max(apart, float(np.abs(warm - cold).max()) / (1.0 + float(np.abs(cold).max())))
    report.check('%d random QPs solved, KKT under %g, warm as cold' % (PROBLEMS, KKT),
                 not fails and worst < KKT and apart < 1e-6,
                 '%d unsolved, KKT %.1e, warm off cold %.1e' % (fails, worst, apart))


def test_a_level_keeps_what_those_above_it_won(report):
    """A stack of two tasks that cannot both be met under a bound: the first as if alone, the
    second only in what the first left."""
    import numpy as np
    from machine import qp
    E, e = np.zeros((0, 3)), np.zeros(0)
    G, h = np.array([[0.0, 0.0, 1.0]]), np.array([0.5])          # x2 <= 0.5
    first = (np.array([[1.0, 1.0, 0.0]]), np.array([2.0]), np.zeros((0, 3)), np.zeros(0), 1e3)
    second = (np.array([[1.0, -1.0, 0.0], [0.0, 0.0, 1.0]]), np.array([0.0, 3.0]),
              np.zeros((0, 3)), np.zeros(0), 1e3)
    x, _held, _s = qp.stack(E, e, G, h, [first, second])
    report.check('the first level met, the second within it and the bound',
                 x is not None and abs(x[0] + x[1] - 2.0) < 1e-6 and abs(x[0] - x[1]) < 1e-5
                 and abs(x[2] - 0.5) < 1e-6,
                 'x %s' % (None if x is None else np.round(x, 6)))


def test_she_stands_still_on_the_stack(report):
    """Two seconds standing: up, her centre of mass still once settled, every drive inside its
    clamp."""
    from tools.sim import wbc
    out = wbc.stand(seconds=2.0)
    report.check('standing 2 s on the stack she stays up, every drive within its clamp',
                 out['stood'] and out['top'] <= 1.0 / 0.95 + 1e-6 and not out['steps'],
                 'tilt %.1f deg, drift %.1f mm, the most a clamp %.2f, %d us a step'
                 % (out['tilt'], out['drift_mm'], out['top'], out['us']))


def test_nudged_aside_she_holds_without_a_step(report):
    """60 N for 0.12 s from her right and from her left: up, no step taken."""
    from tools.sim import wbc
    for way in (0.0, 180.0):
        out = wbc.stand(60.0, way)
        report.check('nudged 60 N toward %s she holds, no step' % ('her left' if way == 0 else
                                                                  'her right'),
                     out['stood'] and not out['steps'],
                     'tilt %.1f deg, %d steps' % (out['tilt'], out['steps']))


def test_shoved_from_behind_she_steps_out_of_it(report):
    """80 N for 0.12 s from behind - her capture point past her toes: she steps and stands."""
    from tools.sim import wbc
    out = wbc.stand(80.0, 90.0)
    report.check('shoved 80 N from behind she steps and stands',
                 out['stood'] and out['steps'] >= 1,
                 'tilt %.1f deg, %d steps, WEP %.2f s' % (out['tilt'], out['steps'], out['wep_s']))


ROSTER = (test_the_solver_meets_its_optimality, test_a_level_keeps_what_those_above_it_won,
          test_she_stands_still_on_the_stack, test_nudged_aside_she_holds_without_a_step,
          test_shoved_from_behind_she_steps_out_of_it)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
