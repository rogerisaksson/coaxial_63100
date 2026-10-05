"""The gynoid's run on a flat, smooth floor, from a flight at its speed (`machine.runner`): she
runs on at the speed asked, a flight every step, her trunk up, her soles' load a jog's.
`tools.sim.run.FORM` is the form, `tools.sim.run.ran` her running, on her boards as built. Her
walk is test_gynoid_gait.py's."""
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The speeds asked, m/s: a jog and the fastest she holds from a flight on her motors' rotors
#: (asked 2.0 she is down in 8-25 s; 1.94 m/s on a rotor 0.4 of theirs, 2026-10-05).
SPEEDS = (1.25, 1.75)

#: A speed is run RUN_S and judged from FROM_S on: she is at her speed 4 s in, her feet's bias
#: a landing at a time (`runner.SPEED_I`).
RUN_S, FROM_S = 10.0, 5.0


def test_she_runs_on_in_her_form(report):
    """At each speed she is up at RUN_S and every condition of `run.FORM` is met from FROM_S."""
    from tools.sim import run
    for speed in SPEEDS:
        m, _steps, _demand = run.ran(speed, RUN_S, {}, swap={}, swap_s=FROM_S - run.SETTLE_S)
        report.check('at %.2f m/s she runs on' % speed, not m['fell'],
                     '%.2f m/s, %.0f J/m' % (m['speed'], m['energy']))
        if m['fell']:
            continue
        has = dict(m, flight=m['flights'][0])
        for name, least, most in run.FORM:
            report.check('at %.2f: %s %s' % (speed, name, 'at least %g' % least
                                              if least is not None else 'at most %g' % most),
                         (least is None or has[name] >= least)
                         and (most is None or has[name] <= most), '%.3g' % has[name])


ROSTER = [test_she_runs_on_in_her_form]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
