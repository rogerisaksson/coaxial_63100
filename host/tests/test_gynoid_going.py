"""The gynoid's going on the one law (`machine.going`) on a flat, smooth floor: her walk's row
walks on, a foot always down, her run's runs on, a flight a step; asked on from a stand she
walks, asked the run she passes her walk and her jog to it, and asked to a stand she stands
again; the director hands her to the law from its stand. `tools.sim.go.FORM` is what her rows
hold, `tools.sim.go.went` her going, on her boards as built; `tools/sim/ways.py` is the spread.
Her walk as built is test_gynoid_gait.py's, the runner test_gynoid_run.py's."""
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report

#: A row is gone on GO_S, s.
GO_S = 10.0

#: Her ways on setpoints alone: (what, rows asked "t:k ..", its seconds, {segment: `go.FORM`'s
#: row}) - a row asked again marks a segment's end.
WAYS = (('stood, asked on and to a stand again', '0:-1 1.5:0 4.5:0 9:-1 12:-1', 15.0,
         {0: 'stand', 2: 'walk', 4: 'stand again'}),
        ('asked the run and to a stand again', '0:-1 1:1 16:1 20:-1 36:-1', 39.0,
         {0: 'stand', 2: 'run', 4: 'stand again'}))


def held(report, what, went, form):
    """`went`, a segment, checked against `form`, a row of `go.FORM`."""
    for measure, least, most in form:
        report.check('%s: %s %s' % (what, measure, 'at least %g' % least if least is not None
                                    else 'at most %g' % most),
                     (least is None or went[measure] >= least)
                     and (most is None or went[measure] <= most), '%.3g' % went[measure])


def test_a_gait_is_a_row_of_the_same_names(report):
    """Her rows hold the same setpoints; her way between them passes each at its knot, and the
    row she goes on stays DWELL_S on a gait between before it leaves it."""
    from machine import gaits
    rows = (gaits.STAND, gaits.WALK, gaits.EASE, gaits.JOG, gaits.RUN)
    report.check('the rows share their names', len({frozenset(row) for row in rows}) == 1,
                 '%d' % len(gaits.WALK))
    off = max(abs(gaits.between(k)[name] - row[name])
              for k, row in zip(gaits.KNOTS, rows) for name in row)
    report.check('her way passes each row at its knot', off < 1e-9, '%.1e off' % off)
    k, on, at = -1.0, 0.0, {}
    for i in range(20000):
        k, on = gaits.toward(k, on, 1.0, 0.001)
        if k in gaits.KNOTS and k not in at:
            at[k] = 0.001 * i
    stays = [round(at[0.25] - at[0.0], 2), round(at[0.5] - at[0.25], 2)]
    report.check('asked the run from her stand she stays on her walk and on the row between',
                 at.get(1.0, 99.0) < 15.0 and min(stays) >= gaits.DWELL_S,
                 'at the run in %.1f s, %s s a gait' % (at.get(1.0, 99.0), stays))


def test_her_rows_go_on(report):
    """On the walk's row, asked on from a stand, and on the run's, placed at its speed, she
    is up GO_S on, every condition of `go.FORM` met."""
    from tools.sim import go
    for name, track in (('walk', [(0.0, -1.0), (1.0, 0.0), (6.0, 0.0)]), ('run', [(0.0, 1.0)])):
        result = go.went(track, track[-1][0] + GO_S, asked=len(track) > 1)
        went = result['segments'][-1]
        report.check('on the %s row she goes on' % name, not result['fell'],
                     '%.2f m/s, %.0f J/m' % (went['speed'], went['drawn']))
        if not result['fell']:
            held(report, 'on the %s row' % name, went, go.FORM[name])


def test_her_ways_on_setpoints_alone(report):
    """Each of WAYS: she is up at its end, and every segment named holds its row of `go.FORM`."""
    from tools.sim import go
    for what, track, to_s, rows in WAYS:
        result = go.went([tuple(float(x) for x in p.split(':')) for p in track.split()], to_s,
                         asked=True)
        report.check('%s she is up' % what, not result['fell'],
                     'down at %.1f s' % result['fell'] if result['fell'] else '%g s' % to_s)
        if result['fell']:
            continue
        for i, name in rows.items():
            held(report, '%s, %s' % (what, name), result['segments'][i], go.FORM[name])


def test_the_director_hands_her_to_the_law(report):
    """Landed standing under the director, a pace asked: the law takes her from the arrival's
    stand and she walks; asked to a stand she stands, the law's still."""
    from machine import Machine
    from machine.director import Director
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    try:
        body.arm()
        director = Director(body, stand_s=1e9)
        director.pace = -1.0
        director.begin(drop=0.004, stage='stand')
        body.loop.step(0.0)
        bus, stages, z = body.loop.bus, [], {}
        for until, pace in ((1.5, -1.0), (5.0, 0.0), (8.0, 0.0), (10.0, -1.0), (13.0, -1.0)):
            director.pace = pace
            while bus['t'] < until and director.stage != 'fallen':
                body.loop.write(**director.step(0.001))
                body.loop.step(0.001)
                if director.stage not in stages[-1:]:
                    stages.append(director.stage)
            z[until] = bus['pelvis.pose.z']
        report.check('the law takes her from the stand, and nothing takes her from the law',
                     stages == ['stand', 'go'], ' '.join(stages))
        walked = (z[8.0] - z[5.0]) / 3.0
        report.check('asked on she walks', 0.6 < walked < 0.9, '%.2f m/s' % walked)
        after = (z[13.0] - z[10.0]) / 3.0
        report.check('asked to a stand she stands', abs(after) < 0.05, '%.2f m/s' % after)
    finally:
        body.close()


ROSTER = [test_a_gait_is_a_row_of_the_same_names, test_her_rows_go_on,
          test_her_ways_on_setpoints_alone, test_the_director_hands_her_to_the_law]


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
