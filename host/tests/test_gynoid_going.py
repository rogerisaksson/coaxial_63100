"""The gynoid's going on the one law (`machine.going`) on a flat, smooth floor: her walk's row
walks on, a foot always down, her run's runs on, a flight a step; asked on from a stand she
walks, asked the run she passes her walk and her jog to it, and asked to a stand she stands
again; the director hands her to the law from its stand; her manners (`gaits.MANNERS`) come on
and go as she walks, each read back in its words. `tools.sim.go.FORM` is what her rows
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
        ('asked the run and to a stand again', '0:-1 1:1 18:1 22:-1 40:-1', 43.0,
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
    rows = (gaits.STAND, gaits.WALK, gaits.QUICK, gaits.EASE, gaits.JOG, gaits.RUN)
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
    stays = [round(at[b] - at[a], 2) for a, b in ((0.0, 0.125), (0.125, 0.25), (0.25, 0.5))]
    report.check('asked the run from her stand she stays on her walk and on each row between',
                 at.get(1.0, 99.0) < 17.0 and min(stays) >= gaits.DWELL_S,
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


#: A way is asked each of SHIFTS s later and she is up through LEAST of them: a host's floats
#: decide a passage's steps - CI's runner down at 30.4 s on her way back from the run, this
#: host up through it on 12 timings of 12 (2026-10-06; docs/TODO.md item 28).
SHIFTS, LEAST = (0.0, 0.13, 0.26), 2


def test_her_ways_on_setpoints_alone(report):
    """Each of WAYS, over SHIFTS: she is up at its end on LEAST of them, and every segment
    named holds its row of `go.FORM` on the first she is up through."""
    from tools.sim import go
    for what, track, to_s, rows in WAYS:
        points = [tuple(float(x) for x in p.split(':')) for p in track.split()]
        went = []
        for shift in SHIFTS:                     # till LEAST are up, or cannot be
            went.append(go.went([(t + shift * (t > 0.0), k) for t, k in points],
                                to_s + shift, asked=True))
            ups = sum(1 for r in went if not r['fell'])
            if ups >= LEAST or ups + len(SHIFTS) - len(went) < LEAST:
                break
        up = [result for result in went if not result['fell']]
        report.check('%s she is up, %d timings of %d' % (what, LEAST, len(SHIFTS)),
                     len(up) >= LEAST, ', '.join(
                         'down at %.1f s' % r['fell'] if r['fell'] else 'up' for r in went))
        for i, name in rows.items() if up else ():
            held(report, '%s, %s' % (what, name), up[0]['segments'][i], go.FORM[name])


#: The manner asked of her as she walks under the director.
CROUCHED = (('crouched', 1.0),)


def test_the_director_hands_her_to_the_law(report):
    """Landed standing under the director, a pace asked: the law takes her from the arrival's
    stand and she walks, crouched as asked; asked to a stand she stands, the law's still."""
    from machine import Machine, gaits
    from machine.director import Director
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    try:
        body.arm()
        director = Director(body, stand_s=1e9)
        director.pace = -1.0
        director.begin(drop=0.004, stage='stand')
        body.loop.step(0.0)
        bus, stages, z, strut = body.loop.bus, [], {}, {}
        for until, pace, manner in ((1.5, -1.0, ()), (5.0, 0.0, ()), (8.0, 0.0, CROUCHED),
                                    (10.0, -1.0, ()), (13.0, -1.0, ())):
            director.pace, director.manner = pace, manner
            while bus['t'] < until and director.stage != 'fallen':
                body.loop.write(**director.step(0.001))
                body.loop.step(0.001)
                if director.stage not in stages[-1:]:
                    stages.append(director.stage)
            z[until] = bus['pelvis.pose.z']
            strut[until] = director.going and director.going.ask['strut']
        report.check('the law takes her from the stand, and nothing takes her from the law',
                     stages == ['stand', 'go'], ' '.join(stages))
        walked = (z[8.0] - z[5.0]) / 3.0
        report.check('asked on she walks', 0.6 < walked < 1.0, '%.2f m/s' % walked)
        bent = gaits.MANNERS['crouched']['strut']
        report.check('asked crouched as she walks, her strut is that manner\'s in %g s and '
                     'the row\'s again as long after it is let go' % gaits.MANNER_S,
                     abs(strut[8.0] - strut[5.0] - bent) < 1e-6 and strut[13.0] == strut[5.0],
                     '%.1f, %.1f, %.1f deg' % (strut[5.0], strut[8.0], strut[13.0]))
        after = (z[13.0] - z[10.0]) / 3.0
        report.check('asked to a stand she stands', abs(after) < 0.05, '%.2f m/s' % after)
    finally:
        body.close()


#: The law's walk beside a woman's normal one (`tools.sim.normal.BAND`): what of it is on her
#: band, how far off it the walk may be in all, the words it may be said to be. 0.20 off it,
#: stiff 0.17 - the heel 22 deg up as its toes leave where 28 - since 2026-10-06; on
#: stilts (the user) before: 4.1 off, the swinging knee 26 deg, no heel's rise, the pelvis
#: level, her feet 0.29 legs apart, her arms bent 80 deg and still.
ON_BAND = ('elbow bent', 'elbow', 'arm', 'hand out', 'knee swinging', 'knee at landing',
           'knee straightest', 'pelvis roll', 'feet apart', 'walk ratio', 'vault')
OFF_BAND, WORDS, WALK_S = 0.6, ('stiff', 'shuffling', 'still-hipped'), 14.0


def test_its_walk_beside_a_womans(report):
    """The walk's row, as the page walks her on the law from the squat: ON_BAND's measures on a
    woman's band, the walk no further off it than OFF_BAND."""
    from tools.sim import fbx, look, normal, strides
    now = normal.measured(*fbx.joints(strides.steady(look.simulated(WALK_S, {}, pace=0.0))))
    far, out = normal.off(now)
    bands = {name: (least, most) for name, _unit, least, most in normal.BAND}
    for name in ON_BAND:
        report.check('%s %g to %g' % ((name,) + bands[name]),
                     bands[name][0] <= now.get(name, float('nan')) <= bands[name][1],
                     '%.3g' % now.get(name, float('nan')))
    report.check('no further off a woman\'s walk than %g' % OFF_BAND, far <= OFF_BAND,
                 '%.2f: %s' % (far, ', '.join('%s %.3g (%g)' % o for o in out)))
    said = normal.said(now)
    report.check('in words, none but %s, each under 0.5: no stilts' % ', '.join(WORDS),
                 all(word in WORDS and far < 0.5 for word, far in said)
                 and 'on stilts' not in normal.named(now),
                 '%s - %s' % (', '.join('%s %.2f' % w for w in said) or 'none',
                              ', '.join(normal.named(now)) or 'no gait named'))


#: Her manners on the law, each asked alone at an amount of 1 as she walks: (manner, the words
#: it is to be read back in) - MANNER_FOR s each, read over its last READ_S, from FROM_S on,
#: then plain again. Up through it on 12 timings of 12: leaning 0.50 at 559 J/m, crouched 0.58
#: at 409, wide 0.68 at 495, tripping 0.36 at 666, into the wind leaning 0.48 and crouched 0.31
#: at 539, and plain again 0.26 off a woman's band (2026-10-06).
MANNERS = (('leaning', ('leaning',)), ('crouched', ('crouched',)), ('wide', ('wide',)),
           ('tripping', ('tripping',)), ('into the wind', ('leaning', 'crouched')))
FROM_S, MANNER_FOR, READ_S = 8.0, 8.0, 5.0


def test_her_manners_in_their_words(report):
    """One walk on the law through each of MANNERS and to her plain walk again, on the first
    of SHIFTS she is up through: each manner said in its words (`normal.said`), and her walk
    then no further off a woman's band than OFF_BAND - a manner comes on and goes as she
    walks."""
    from tools.sim import go
    spans = [(FROM_S + i * MANNER_FOR, name) for i, (name, _words) in enumerate(MANNERS)]
    end = FROM_S + len(MANNERS) * MANNER_FOR
    windows = [(t + MANNER_FOR - READ_S, t + MANNER_FOR) for t, _name in spans] + [
        (end + MANNER_FOR - READ_S, end + MANNER_FOR)]
    went, result = [], {}
    for shift in SHIFTS:
        result = go.went([(0.0, -1.0), (1.3 + shift, -1.0), (2.3 + shift, 0.0)],
                         end + MANNER_FOR, form=windows,
                         manner=[(t, ((name, 1.0),)) for t, name in spans] + [(end, ())])
        went.append('down at %.1f s' % result['fell'] if result['fell'] else 'up')
        if not result['fell']:
            break
    report.check('she is up through her manners, each coming on and going as she walks',
                 not result['fell'], ', '.join(went))
    if result['fell']:
        return
    for (name, words), form in zip(MANNERS, result['forms']):
        said = dict(form['said'])
        report.check('%s: said %s' % (name, ' and '.join(words)),
                     all(word in said for word in words),
                     ', '.join('%s %.2f' % w for w in form['said']) + ' at %.0f J/m' % form['energy'])
    plain = result['forms'][-1]
    report.check('plain again, no further off a woman\'s walk than %g' % OFF_BAND,
                 plain['off'] <= OFF_BAND, '%.2f: %s' % (plain['off'], ', '.join(
                     '%s %.2f' % tuple(w) for w in plain['said']) or 'no word'))


#: The two long ones first: a shard takes every n-th (`focus.chosen`), and CI's three held
#: them in one, 220 and 111 s here, cut at 300 s (f8c6b1b, 2026-10-06).
ROSTER = [test_her_ways_on_setpoints_alone, test_her_manners_in_their_words,
          test_her_rows_go_on, test_a_gait_is_a_row_of_the_same_names,
          test_the_director_hands_her_to_the_law, test_its_walk_beside_a_womans]


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
