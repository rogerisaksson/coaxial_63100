"""The stack's dual active set in C beside `machine.stack.qp.solve` on random QPs (tools.cores.qp).

Unknowns 2 to 60 under 0 to 200 rows met at a point drawn: rows too short to keep, problems no x meets, warm starts
from the last solve's held rows, the dual given one step so the interior point answers - the C's x, the rows it
holds and their multipliers are the python's bit for bit on an exact host, within their last bits elsewhere; a
solve's time."""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: Problems drawn, and how far the C may sit from the python where numpy dispatches elsewhere.
DRAWS, APART = 120, 1e-9


def _drawn(rng, n, m, meets=True):
    """(H, g, A, b) of n unknowns under m rows: H positive definite, a tenth of the rows too short to keep, every
    row met at a point drawn (some by a hair) - or two rows no x meets, the first against the second."""
    import numpy as np
    B = rng.normal(size=(n, n))
    H = B.T @ B + 0.1 * np.eye(n)
    A = rng.normal(size=(m, n))
    A[rng.random(m) < 0.1] = 0.0
    b = A @ rng.normal(size=n) + rng.uniform(0.0, 1.0, size=m) ** 2
    if not meets and m > 1:
        A[0], b[0], b[1] = -A[1], -1.0, -1.0
    return H, rng.normal(size=n), A, b


def _apart(ours, theirs):
    """How far two solves sit apart: nan where one meets its rows and the other does not or the rows held differ, else
    the most any x or multiplier differs."""
    import numpy as np
    if (ours[0] is None) != (theirs[0] is None) or tuple(ours[1]) != tuple(theirs[1]):
        return float('nan')
    if ours[0] is None:
        return 0.0
    return max(float(np.abs(ours[0] - theirs[0]).max()), float(np.abs(ours[2] - theirs[2]).max()) if len(ours[2]) else 0.0)


def test_the_dual_active_set_in_c_is_the_stacks(report):
    """`wbc_qp_solve` beside `qp.solve` over DRAWS random problems, cold, warm from the rows held and with the dual
    given one step (`most`) so the interior point answers."""
    import numpy as np
    from machine.stack import qp
    from tools.cores import motion as c_motion, qp as c_qp
    rng = np.random.default_rng(3)
    worst, unlike, none, warmed, interior = 0.0, 0, 0, 0, 0
    for k in range(DRAWS):
        n, m = int(rng.integers(2, 61)), int(rng.integers(0, 201)) if k % 7 else 0
        H, g, A, b = _drawn(rng, n, m, k % 7 != 3)
        ours = qp.solve(H, g, A, b)
        apart = _apart(ours, c_qp.solve(H, g, A, b))
        none += ours[0] is None
        if ours[0] is not None and m:
            g2 = g + 0.01 * rng.normal(size=n)
            apart = max(apart, _apart(qp.solve(H, g2, A, b, ours[1]), c_qp.solve(H, g2, A, b, ours[1])))
            warmed += 1
        if k % 5 == 0 and m:
            apart = max(apart, _apart(qp.solve(H, g, A, b, most=1), c_qp.solve(H, g, A, b, most=1)))
            interior += 1
        unlike += int(apart != apart or apart > APART)
        worst = max(worst, apart) if apart == apart else worst
    line = '%d problems (%d none meets, %d warm, %d by the interior point): %d unlike, %.1e apart at most' % (
        DRAWS, none, warmed, interior, unlike, worst)
    if c_motion.exact_host():
        report.check("the dual active set in C is the stack's bit for bit, cold, warm and where the interior point answers",
                     unlike == 0 and worst == 0.0 and none > 0 and interior > 0, line)
    else:
        report.check("the dual active set in C is the stack's within its last bits where numpy dispatches elsewhere",
                     unlike == 0 and none > 0 and interior > 0, line)
    H, g, A, b = _drawn(rng, 40, 120)
    t = time.perf_counter()
    for _ in range(20):
        c_qp.solve(H, g, A, b)
    us = 1e6 * (time.perf_counter() - t) / 20
    report.check('a solve of 40 unknowns under 120 rows on this host', us < 20000.0, '%.0f us' % us)


ROSTER = (test_the_dual_active_set_in_c_is_the_stacks,)


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
