"""The gynoid's walk on a flat, smooth floor, steady, as the page walks her: a human's (the user,
2026-10-05) - the pelvis rolling over a stance leg whose knee is all but straight, the leg on
behind the plumb line, her head still, her feet quiet, no parry asked. `tools.sim.looks.FORM` is
the form, `tools.sim.armada.Robot` her walking: marked steady at each pace and gone back to
(`tools.sim.replay`), a pace's measure 4 s of her time. Her walk as it was approved is a take,
tests/takes/walk.fbx, and a woman's normal walk a band (`tools.sim.normal`): she is held to
both (the user, 2026-10-05: a metric, an .fbx its reference, a test of what broke). Her rises,
falls and parries are the other gynoid suites'."""
import os
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The paces she is walked at, strides/s: the page's, a slow one and a fast one.
PACES = (0.85, 0.65, 1.0)

#: From the squat, as the page starts her, she is watched SQUAT_S and asked no parry: her start
#: ending, a pre-swing of 22 deg parried twice at every pace where the steady walk did not
#: (2026-10-05).
SQUAT_S = 16.0

#: A pace is judged on a spread of walks (`looks.spread`, `looks.PARRIES`): the sideways gain a
#: thousandth either way.
SPREAD = (1.0, 1.001, 0.999)

#: Her walk as approved: `python tools/sim/look.py --built --to 12 --fbx tests/takes/walk.fbx`
#: (2026-10-05), the same TAKE_S from the squat here. Written anew only with a walk approved.
TAKE_S, TAKE = 12.0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'takes', 'walk.fbx')

#: Her walk is its take's with each measure within WITHIN of its band's width: two windows of
#: one walk differ by 0.03 at most - the knee at its landing 0.3 deg, at its lift 0.5.
WITHIN = 0.1

#: What of her walk as built is out of a woman's band (`normal.BAND`), each no further than
#: this (2026-10-05: 1.04, 75.2 %, 29.5 and 32.1 deg, 0.015 legs): her step short for its
#: time, both feet down a quarter of a stride a step, her knee landing bent (docs/TODO.md item
#: 1), her pelvis rising and falling 12 mm.
KNOWN = {'walk ratio': 0.98, 'stance': 77.0, 'knee at landing': 31.0,
         'thigh ahead at landing': 33.5, 'pelvis bob': 0.012}

#: The words her walk as built may be said to be (`normal.said`): its knee lands bent, its
#: feet are long on the floor, its pelvis hardly rises and falls - still-hipped 0.10 on CI's
#: runner, a hair under saying here. Stiff, crouched, tripping, wide: not hers.
WORDS = ('landing bent', 'shuffling', 'still-hipped')

_ROBOT, _STEADY = [], []


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
    the strike - and of the spread's nine strides at most `looks.PARRIES` parried."""
    from tools.sim import knobs, looks
    robot, gain = _robot(), knobs.now('walker.SIDE_K')
    for pace in PACES:
        walks = [robot.tried({'walker.SIDE_K': gain * k} if k != 1.0 else {}, pace)
                 for k in SPREAD]
        m = looks.spread(walks)
        report.check('at %.2f strides/s she walks on' % pace, not m['fell'],
                     '%.0f J/m at %.2f m/s' % (m.get('energy', 0.0), m.get('speed', 0.0)))
        if m['fell']:
            continue
        off = dict((b[0], b) for b in looks.broken(m, looks.PARRIES))
        for name, least, most in looks.FORM:
            most = looks.PARRIES if name == 'catches' else most
            report.check('at %.2f: %s %s' % (pace, name, 'at least %g' % least
                                              if least is not None else 'at most %g' % most),
                         name not in off, '%.1f' % m.get(name, float('nan')))


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
    most = dict((name, top) for name, _least, top in looks.FORM
                if top is not None)['knee behind plumb']
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


def _steady():
    """Her steady walk's rows as the page walks her from the squat, TAKE_S of it: once."""
    if not _STEADY:
        from tools.sim import look, strides
        _STEADY.append(strides.steady(look.simulated(TAKE_S, {})))
    return _STEADY[0]


def test_her_walk_is_its_take(report):
    """Her walk written a take reads back to the mm, and it is the walk approved: each measure
    of `normal.BAND` its reference take's, within WITHIN of the band's width."""
    from coaxial.model.blocks import numpy as np
    from tools import REPO
    from tools.sim import fbx, normal
    rows = _steady()
    path = os.path.join(REPO, 'build', 'takes', 'test_gynoid_gait.fbx')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fbx.wrote(path, rows)
    times, read = fbx.take(path)
    _t, hers = fbx.joints(rows)
    worst = max(float(np.abs(read[name] - hers[name]).max()) for name in hers)
    report.check('written a take, every joint reads back within a mm',
                 len(times) == len(rows) and worst < 1e-3,
                 '%.3f mm over %d frames, %d joints' % (1e3 * worst, len(times), len(hers)))
    now, was = normal.measured(*fbx.joints(rows)), normal.measured(*fbx.take(TAKE))
    report.check('her strides and the take\'s are measured', now['strides'] >= 4 <= was['strides'],
                 '%d and %d' % (now['strides'], was['strides']))
    for widths, name, mine, its in sorted(normal.apart(now, was)[1], key=lambda e: e[1]):
        report.check('%s as her take has it' % name, widths <= WITHIN,
                     '%.3g, the take %.3g' % (mine, its))


def test_her_walk_beside_a_womans(report):
    """Each measure of a woman's normal walk (`normal.BAND`) is met by hers, or is one of KNOWN
    and no further out than there."""
    from tools.sim import fbx, normal
    now = normal.measured(*fbx.joints(_steady()))
    for name, unit, least, most in normal.BAND:
        v, known = float(now.get(name, float('nan'))), KNOWN.get(name)
        low, high = float(least), float(most)
        if known is not None:
            low, high = min(low, known), max(high, known)
        report.check('%s %g to %g %s%s' % (name, least, most, unit, '' if known is None
                                           else ', known out to %g' % known),
                     low <= v <= high, '%.3g' % v)
    report.check('nothing of KNOWN is back in her band unnoticed',
                 all(not least <= now[name] <= most for name, _u, least, most in normal.BAND
                     if name in KNOWN), 'off it %.2f' % normal.off(now)[0])
    said, named = normal.said(now), normal.named(now)
    report.check('in words, none but %s' % ', '.join(WORDS),
                 all(word in WORDS for word, _far in said),
                 ', '.join('%s %.2f' % w for w in said) or 'none')
    report.check("neither on stilts nor Groucho's", not set(named) & {'on stilts', "Groucho's"},
                 ', '.join(named) or 'no gait named')


#: Her manners asked (`style.MANNERS`) and what her walk is then, measured: (asked, the gait
#: `normal.named` has it).
MANNERS = (((('crouched', 1.0),), "Groucho's"), ((('catwalk', 2.0),), 'a catwalk'))


def test_her_manners_answer_their_words(report):
    """A concept asked of her walk is read back off it: crouched, Groucho's; a catwalk, one -
    and she walks on. Her walk as tuned is neither."""
    from machine import style
    from tools.sim import fbx, look, normal, strides
    try:
        for asked, gait in MANNERS:
            rows = look.simulated(TAKE_S, {}, manner=asked)
            walk = strides.steady(rows)
            named = normal.named(normal.measured(*fbx.joints(walk))) if len(walk) > 60 else []
            report.check('asked %s she walks on, %s' % (
                ', '.join('%s %g' % m for m in asked), gait),
                rows[-1]['stage'] == 'walk' and gait in named,
                '%s; %s' % (rows[-1]['stage'], ', '.join(named) or 'no gait named'))
    finally:
        style.manner(())
    named = normal.named(normal.measured(*fbx.joints(_steady())))
    report.check("as tuned neither Groucho's nor a catwalk",
                 not set(named) & {"Groucho's", 'a catwalk'}, ', '.join(named) or 'no gait named')


ROSTER = [test_her_walk_holds_its_form, test_gone_back_she_walks_the_same,
          test_from_the_squat_she_walks_off_unparried, test_her_walk_is_its_take,
          test_her_walk_beside_a_womans, test_her_manners_answer_their_words]


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
