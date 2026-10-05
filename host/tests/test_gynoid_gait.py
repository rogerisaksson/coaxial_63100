"""The gynoid's walk on a flat, smooth floor, steady, as the page walks her: a human's (the user,
2026-10-05) - the pelvis rolling over a stance leg whose knee is all but straight, the leg on
behind the plumb line, her head still, her feet quiet, no parry asked. `tools.sim.looks.FORM` is
the form, `tools.sim.armada.Robot` her walking: marked steady at each pace and gone back to
(`tools.sim.replay`), a pace's measure 4 s of her time. Her rises, falls and parries are the
other gynoid suites'."""
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The paces she is walked at, strides/s: the page's, a slow one and a fast one.
PACES = (0.85, 0.65, 1.0)

#: From the squat, as the page starts her, she is watched SQUAT_S and asked no parry: her start
#: ending, a pre-swing of 22 deg parried twice at every pace where the steady walk did not
#: (2026-10-05).
SQUAT_S = 16.0

#: A pace is judged on a spread of walks, the sideways gain a thousandth either way, their
#: measures meaned and their parries summed, PARRIES at most: one walk scores chance - at 1.0
#: strides/s the gain's fourth digit made one parry in three strides or none, 18 walks about it
#: none (2026-10-05).
SPREAD, PARRIES = (1.0, 1.001, 0.999), 1

_ROBOT = []


def _robot():
    """Her, walking, marked at PACES: armed once for the suite."""
    if not _ROBOT:
        from tools.sim import armada
        _ROBOT.append(armada.Robot(PACES))
    return _ROBOT[0]


def test_her_walk_holds_its_form(report):
    """At each pace every condition of `looks.FORM` is met over the spread: the knee while her
    foot bears her alone behind the plumb line, the knee at its landing, the leg behind the plumb
    line, the standing hip up, the toes not back at their lift, her head's bob, surge and sway,
    the strike - and of the spread's nine strides at most PARRIES parried."""
    from tools.sim import knobs, looks
    robot, gain = _robot(), knobs.now('walker.SIDE_K')
    for pace in PACES:
        walks = [robot.tried({'walker.SIDE_K': gain * k} if k != 1.0 else {}, pace)
                 for k in SPREAD]
        fell = sum(1 for m in walks if m.get('fell'))
        report.check('at %.2f strides/s she walks on' % pace, not fell,
                     '%d of %d fell' % (fell, len(walks)) if fell else
                     '%.0f J/m at %.2f m/s' % (walks[0]['energy'], walks[0]['speed']))
        if fell:
            continue
        parries = sum(m.get('catches', 0) for m in walks)
        report.check('at %.2f: at most %d parry in the spread' % (pace, PARRIES),
                     parries <= PARRIES, '%d in %d walks' % (parries, len(walks)))
        for name, least, most in looks.FORM:
            if name == 'catches':
                continue
            got = [m[name] for m in walks if name in m]
            v = sum(got) / len(got) if len(got) == len(walks) else float('nan')
            report.check('at %.2f: %s %s' % (pace, name, 'at least %g' % least
                                              if least is not None else 'at most %g' % most),
                         v == v and (least is None or v >= least) and (most is None or v <= most),
                         '%.1f' % v)


def test_gone_back_she_walks_the_same(report):
    """Gone back to her mark she walks the same strides again, to the last digit, and a law
    swapped in and out leaves her as she was: two walks differ by their constants alone."""
    robot = _robot()
    first = robot.tried({}, PACES[0])
    other = robot.tried({'stance.PRE_SWING_DEG': 32.0}, PACES[0])
    again = robot.tried({}, PACES[0])
    report.check('gone back, the same walk to the last digit', first == again,
                 '%.6f and %.6f J/m' % (first.get('energy', 0.0), again.get('energy', 0.0)))
    report.check('a law swapped in is another walk', other != first,
                 '%.1f J/m against %.1f' % (other.get('energy', 0.0), first.get('energy', 0.0)))


def test_from_the_squat_she_walks_off_unparried(report):
    """As the page starts her - the squat, the rise, the lean, the first step - she walks on at
    each pace to SQUAT_S with no parry and her knee straight behind the plumb line: what is seen
    in the page's first seconds."""
    from tools.sim import look, looks, strides
    most = dict((name, top) for name, _least, top in looks.FORM)['knee behind plumb']
    for pace in PACES:
        rows = look.simulated(SQUAT_S, {}, cadence=pace)
        walk = [r for r in rows if r['stage'] in ('walk', 'catch')]
        parries = sum(1 for a, b in zip(rows, rows[1:]) if b['stage'] == 'catch' != a['stage'])
        report.check('from the squat at %.2f she walks on, unparried' % pace,
                     rows[-1]['stage'] == 'walk' and not parries,
                     '%s at %.1f s, %d parries' % (rows[-1]['stage'], float(rows[-1]['t']), parries))
        steady = [r for r in walk if float(r['t']) >= float(walk[0]['t']) + strides.WALK_FROM_S]
        knee = strides.measured(steady)['knee behind plumb'] if len(steady) > 2 else float('nan')
        report.check('from the squat at %.2f: knee behind plumb at most %g' % (pace, most),
                     knee <= most, '%.1f' % knee)


ROSTER = [test_her_walk_holds_its_form, test_gone_back_she_walks_the_same,
          test_from_the_squat_she_walks_off_unparried]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    if _ROBOT:
        _ROBOT.pop().close()
    print('\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
