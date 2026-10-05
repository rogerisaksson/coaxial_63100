"""The gynoid's going on the one law (`machine.going`) on a flat, smooth floor, from a start
placed at its speed: her walk's row walks on, a foot always down, her run's runs on, a flight a
step. `tools.sim.go.FORM` is what the rows hold, `tools.sim.go.went` her going, on her boards as
built. Her walk as built is test_gynoid_gait.py's, the runner test_gynoid_run.py's."""
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report

#: A row is gone on GO_S, s.
GO_S = 10.0


def test_a_gait_is_a_row_of_the_same_names(report):
    """The walk's, the run's and the row between hold the same setpoints, and her way between
    them begins and ends on them."""
    from machine import gaits
    report.check('the rows share their names',
                 set(gaits.WALK) == set(gaits.RUN) == set(gaits.MID), '%d' % len(gaits.WALK))
    off = max(abs(gaits.between(k)[name] - row[name]) for k, row in (
        (0.0, gaits.WALK), (0.5, gaits.MID), (1.0, gaits.RUN)) for name in row)
    report.check('her way begins on the walk, passes the row between and ends on the run',
                 off < 1e-9, '%.1e off' % off)


def test_her_rows_go_on(report):
    """On the walk's row and on the run's she is up at GO_S, every condition of `go.FORM` met."""
    from tools.sim import go
    for name, k in (('walk', 0.0), ('run', 1.0)):
        result = go.went([(0.0, k)], GO_S)
        went = result['segments'][-1]
        report.check('on the %s row she goes on' % name, not result['fell'],
                     '%.2f m/s, %.0f J/m' % (went['speed'], went['drawn']))
        if result['fell']:
            continue
        for measure, least, most in go.FORM[name]:
            report.check('on the %s row: %s %s' % (name, measure, 'at least %g' % least
                                                    if least is not None else 'at most %g' % most),
                         (least is None or went[measure] >= least)
                         and (most is None or went[measure] <= most), '%.3g' % went[measure])


ROSTER = [test_a_gait_is_a_row_of_the_same_names, test_her_rows_go_on]


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
