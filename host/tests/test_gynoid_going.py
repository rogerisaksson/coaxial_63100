"""The gynoid's going on the one law (`machine.going`) on a flat, smooth floor: her walk's row
walks on, a foot always down, her run's runs on, a flight a step; from a stand she is asked on,
walks, is asked to a stand and stands; from her walk she passes to her jog. `tools.sim.go.FORM`
is what her rows hold, `tools.sim.go.went` her going, on her boards as built. Her walk as built
is test_gynoid_gait.py's, the runner test_gynoid_run.py's."""
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report

#: A row is gone on GO_S, s, from a start placed at its speed.
GO_S = 10.0

#: Her ways on setpoints alone: (what, track "t:k ..", its seconds, {segment: `go.FORM`'s row}).
WAYS = (('stood, asked on and to a stand again', '0:-1 1.5:-1 2.5:0 8:0 9:-1 14:-1', 14.0,
         {0: 'stand', 2: 'walk', 4: 'stand again'}),
        ('from her walk to her jog', '0:-1 1:-1 2:0 5:0 6:0.5 12:0.5', 12.0,
         {0: 'stand', 2: 'walk', 4: 'jog'}))


def held(report, what, went, form):
    """`went`, a segment, checked against `form`, a row of `go.FORM`."""
    for measure, least, most in form:
        report.check('%s: %s %s' % (what, measure, 'at least %g' % least if least is not None
                                    else 'at most %g' % most),
                     (least is None or went[measure] >= least)
                     and (most is None or went[measure] <= most), '%.3g' % went[measure])


def test_a_gait_is_a_row_of_the_same_names(report):
    """The walk's, the run's and her jog's rows hold the same setpoints, and her way between
    them begins on her stand and passes each."""
    from machine import gaits
    report.check('the rows share their names',
                 set(gaits.WALK) == set(gaits.RUN) == set(gaits.JOG) == set(gaits.STAND),
                 '%d' % len(gaits.WALK))
    off = max(abs(gaits.between(k)[name] - row[name]) for k, row in (
        (-1.0, gaits.STAND), (0.0, gaits.WALK), (0.5, gaits.JOG), (1.0, gaits.RUN))
        for name in row)
    report.check('her way begins on her stand, passes the walk and her jog and ends on the run',
                 off < 1e-9, '%.1e off' % off)


def test_her_rows_go_on(report):
    """On the walk's row and on the run's she is up at GO_S, every condition of `go.FORM` met."""
    from tools.sim import go
    for name, k in (('walk', 0.0), ('run', 1.0)):
        result = go.went([(0.0, k)], GO_S)
        went = result['segments'][-1]
        report.check('on the %s row she goes on' % name, not result['fell'],
                     '%.2f m/s, %.0f J/m' % (went['speed'], went['drawn']))
        if not result['fell']:
            held(report, 'on the %s row' % name, went, go.FORM[name])


def test_her_ways_on_setpoints_alone(report):
    """Each of WAYS: she is up at its end, and every segment named holds its row of `go.FORM`."""
    from tools.sim import go
    for what, track, to_s, rows in WAYS:
        result = go.went([tuple(float(x) for x in p.split(':')) for p in track.split()], to_s)
        report.check('%s she is up' % what, not result['fell'],
                     'down at %.1f s' % result['fell'] if result['fell'] else '%g s' % to_s)
        if result['fell']:
            continue
        for i, name in rows.items():
            held(report, '%s, %s' % (what, name), result['segments'][i], go.FORM[name])


ROSTER = [test_a_gait_is_a_row_of_the_same_names, test_her_rows_go_on,
          test_her_ways_on_setpoints_alone]


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
