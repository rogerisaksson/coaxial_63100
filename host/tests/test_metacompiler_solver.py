#!/usr/bin/env python3
"""The static solver held to its physics: a loop's margin, its bounds and the inertia a drive sees.

`machine.solver.loops`: the frequency solved for a dead time and lags leaves the margin asked, read off
the loop's own response by a search of the suite's and seen in a loop run in time; half the dead time
allows twice the frequency, a lag takes of it; the loop designed is the softest that meets its needs,
damped as asked on the most inertia, feasible under its ceilings alone. `machine.solver.inertia`: a
drive sees what the mass matrix says, more with a contact held. The package imports no machine.
The compilers that call the solver: test_metacompiler, test_metacompiler_crafts."""
import ast
import cmath
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from structure_kit import HOST, Report

#: Loops to solve, (dead time s, damping ratio, margin rad, lags s): a joint's on its bus, a craft's axis on
#: its step, a loop behind a lagging actuator, one with lags and no dead time.
CASES = ((0.0015, math.sqrt(0.5), 0.8, ()), (0.002, 0.68, 0.6, ()), (0.001, 1.0, 1.0, (0.004,)),
         (0.0, 0.5, 0.5, (0.02, 0.001)))
#: What the solver's package may import: the language's own and numpy.
PLAIN = {'math', 'typing', 'numpy'}


def _response(w, natural, zeta, dead, lags):
    """A loop's open response at `w` rad/s, per inertia: (w_n^2 + 2 zeta w_n s) / s^2 behind its dead time
    and its lags."""
    s = 1j * w
    out = (natural * natural + 2.0 * zeta * natural * s) / (s * s) * cmath.exp(-s * dead)
    for lag in lags:
        out /= 1.0 + s * lag
    return out


def _measured(natural, zeta, dead, lags):
    """The loop's phase margin, rad, read off its response where its gain is one: three decades either
    way of its natural frequency halved 200 times."""
    low, high = natural * 1e-3, natural * 1e3
    for _ in range(200):
        mid = math.sqrt(low * high)
        low, high = (mid, high) if abs(_response(mid, natural, zeta, dead, lags)) > 1.0 else (low, mid)
    return math.pi + cmath.phase(_response(math.sqrt(low * high), natural, zeta, dead, lags))


def _swung(natural, zeta, dead, periods=30, per_dead=40):
    """A loop let go a radian off, run in time: its widest swing in the last quarter of `periods` of its
    natural frequency over its widest in the first - the loop acting on what it saw a dead time ago."""
    dt = dead / per_dead
    steps = int(periods * math.tau / natural / dt)
    theta, rate, seen = 1.0, 0.0, [(1.0, 0.0)] * per_dead
    first = last = 0.0
    for i in range(steps):
        was, rate_was = seen[i % per_dead]
        seen[i % per_dead] = (theta, rate)
        rate -= (natural * natural * was + 2.0 * zeta * natural * rate_was) * dt
        theta += rate * dt
        first = max(first, abs(theta)) if i < steps // 4 else first
        last = max(last, abs(theta)) if i >= 3 * steps // 4 else last
    return last / first


def test_a_loop_keeps_the_margin_asked(r):
    """The frequency `loops.natural` solves leaves the phase margin asked, read off the loop's response
    by the suite's own search; a twentieth faster leaves less, a twentieth slower more."""
    from machine.solver import loops
    for dead, zeta, margin, lags in CASES:
        natural = loops.natural(dead, zeta, margin, lags)
        got = _measured(natural, zeta, dead, lags)
        r.check('dead %g s, lags %s, damped %.2f: %.1f rad/s leaves the %.1f rad asked' % (
            dead, lags, zeta, natural, margin),
            abs(got - margin) < 1e-9
            and _measured(1.05 * natural, zeta, dead, lags) < margin < _measured(0.95 * natural, zeta, dead, lags), got)
    free = math.atan(2.0 * 0.5 * loops.crossover(0.5))
    r.check("a loop's crossover is where its gain is one, its margin with no dead time the lead's there",
            math.isclose(abs(_response(loops.crossover(0.5), 1.0, 0.5, 0.0, ())), 1.0, rel_tol=1e-12)
            and math.isclose(_measured(1.0, 0.5, 0.0, ()), free, rel_tol=1e-9), free)


def test_a_dead_time_sets_the_frequency(r):
    """Half the dead time, the lags halved with it, allows twice the frequency; a lag takes of what the
    dead time alone allows; with neither no frequency is too high; a margin of nothing, or one the loop
    has not with neither, is refused."""
    from machine.solver import loops
    twice = (loops.natural(0.00075, 0.7, 0.8) / loops.natural(0.0015, 0.7, 0.8),
             loops.natural(0.001, 0.7, 0.8, (0.002,)) / loops.natural(0.002, 0.7, 0.8, (0.004,)))
    r.check('half the dead time allows twice the frequency, a lag halved with it',
            all(math.isclose(ratio, 2.0, rel_tol=1e-12) for ratio in twice), twice)
    lagged, alone = loops.natural(0.0015, 0.7, 0.8, (0.001,)), loops.natural(0.0015, 0.7, 0.8)
    r.check('a lag takes of what the dead time alone allows; with neither none is too high',
            0.0 < lagged < alone and loops.natural(0.0, 0.7, 0.8) == math.inf, (lagged, alone))
    refused = []
    for margin in (0.0, math.atan(2.0 * 0.5 * loops.crossover(0.5)), 2.0):
        try:
            loops.natural(0.001, 0.5, margin)
        except ValueError:
            refused.append(margin)
    r.check('a margin of nothing, the loop\'s own without dead time and one past it are refused',
            len(refused) == 3, refused)


def test_a_solved_loop_stands_in_time(r):
    """A loop run in time on what it saw a dead time ago: at four fifths of the frequency where its
    margin ends its swing dies, a quarter past it the swing grows, and the frequency solved for a margin
    asked lies under that end."""
    from machine.solver import loops
    for dead, zeta, margin, _lags in CASES[:2]:
        end = loops.natural(dead, zeta, 1e-6)
        dies, grows = _swung(0.8 * end, zeta, dead), _swung(1.25 * end, zeta, dead)
        r.check('dead %g s, damped %.2f: under %.0f rad/s the swing dies, past it it grows' % (dead, zeta, end),
                dies < 1e-3 and grows > 1e3 and loops.natural(dead, zeta, margin) < end, (dies, grows))


def test_a_loop_is_the_softest_that_meets_its_needs(r):
    """`loops.design` takes the stiffest need for the loop's stiffness and names it, names the least
    ceiling and says whether the need lies under it; its damping is the ratio asked on the most inertia
    the drive sees and more on less; twice the inertia at the same frequencies is twice the gains."""
    from machine.solver import loops
    zeta, least, most = math.sqrt(0.5), 0.05, 1.04
    needs = {'gravity': 600.0, 'task': loops.stiffness(most, 10.0)}
    ceilings = {'dead time': loops.stiffness(least, 245.0), 'series': 2518.0}
    loop = loops.design((least, most), zeta, needs, ceilings)
    r.check('the stiffest need is the stiffness, the least ceiling named, the loop feasible under it',
            loop.kp == 600.0 and (loop.need, loop.ceiling, loop.feasible) == ('gravity', 'series', True), loop)
    r.check('its damping is the ratio asked on the most inertia, more on the least',
            math.isclose(loop.kd / (2.0 * math.sqrt(loop.kp * most)), zeta, rel_tol=1e-12)
            and loop.kd / (2.0 * math.sqrt(loop.kp * least)) > zeta
            and math.isclose(loop.natural_rad_s, math.sqrt(loop.kp / most), rel_tol=1e-12), loop.kd)
    over = loops.design((least, most), zeta, {**needs, 'throw': 3000.0}, ceilings)
    r.check('a need past the least ceiling stands as asked and the loop is not feasible',
            over.kp == 3000.0 and over.need == 'throw' and not over.feasible, over)
    per = loops.design(1.0, 0.68, {'motion': loops.stiffness(1.0, 17.0)}, {'authority': 500.0})
    wide = loops.design(2.0, 0.68, {'motion': loops.stiffness(2.0, 17.0)}, {'authority': 1000.0})
    r.check('per inertia the gains are the frequency squared and twice the ratio times it, twice the inertia twice both',
            math.isclose(per.kp, 289.0, rel_tol=1e-12) and math.isclose(per.kd, 2.0 * 0.68 * 17.0, rel_tol=1e-12)
            and math.isclose(wide.kp, 2.0 * per.kp, rel_tol=1e-12) and math.isclose(wide.kd, 2.0 * per.kd, rel_tol=1e-12)
            and wide.natural_rad_s == per.natural_rad_s, (per, wide))


def test_a_drive_sees_what_the_mass_matrix_says(r):
    """`inertia.seen` on two coupled coordinates: free, the first sees its own less what the second gives;
    with the second held, its own whole; a held coordinate none of its own motion; a hold stated twice
    is one hold; uncoupled coordinates see their own; and an inertia falls at the root of gravity's
    stiffness over it."""
    from machine.solver import inertia
    own, tie, other = 2.0, 0.5, 1.0
    mass = [[own, tie], [tie, other]]
    r.check('free, a coordinate sees its own inertia less what the other gives; the other held, its own whole',
            math.isclose(inertia.seen(mass, 0), own - tie * tie / other, rel_tol=1e-12)
            and math.isclose(inertia.seen(mass, 0, [[0.0, 1.0]]), own, rel_tol=1e-12), inertia.seen(mass, 0))
    r.check('a held coordinate has no motion of its own, and a hold stated twice is one hold',
            inertia.seen(mass, 1, [[0.0, 1.0]]) == math.inf
            and math.isclose(inertia.seen(mass, 0, [[0.0, 1.0], [0.0, 2.0]]), own, rel_tol=1e-12),
            inertia.seen(mass, 1, [[0.0, 1.0]]))
    lone = [inertia.seen([[3.0, 0.0, 0.0], [0.0, 5.0, 0.0], [0.0, 0.0, 7.0]], k) for k in range(3)]
    r.check("uncoupled coordinates see their own; an inertia falls at the root of gravity's stiffness over it",
            all(math.isclose(a, b, rel_tol=1e-12) for a, b in zip(lone, (3.0, 5.0, 7.0)))
            and math.isclose(inertia.fall(281.0, 26.5), math.sqrt(281.0 / 26.5), rel_tol=1e-12), lone)


def test_the_solver_imports_no_machine(r):
    """The solver's package is functions of numbers: every module of it imports the language's own and
    numpy alone - no spec, no compiler, nothing of the robot or of a craft - so both compilers may call
    it and it calls neither."""
    folder = os.path.join(HOST, 'machine', 'solver')
    reached = {}
    for name in sorted(os.listdir(folder)):
        if not name.endswith('.py'):
            continue
        with open(os.path.join(folder, name), encoding='utf-8') as f:
            tree = ast.parse(f.read())
        reached[name] = sorted(
            {alias.name.split('.')[0] for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
            | {(node.module or '').split('.')[0] for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)})
    far = {name: got for name, got in reached.items() if not set(got) <= PLAIN}
    r.check("%d modules of the solver, each importing the language's own and numpy alone" % len(reached),
            len(reached) >= 3 and not far, far)


ROSTER = (test_a_loop_keeps_the_margin_asked, test_a_dead_time_sets_the_frequency,
          test_a_solved_loop_stands_in_time, test_a_loop_is_the_softest_that_meets_its_needs,
          test_a_drive_sees_what_the_mass_matrix_says, test_the_solver_imports_no_machine)


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
