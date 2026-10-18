"""Her errands on her loop (`worker.session`): a walk and a stop that settles, F to sit and to
rise, a scene laid anew and a walk at once."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from views_kit import Report

WALK, STOP = (0.69, 0.0, 0.0), (0.0, 0.0, 0.0)


def _run(s, seconds, until=None, after=0.0):
    """`seconds` of passes: (the flips of her doing from `after` s on, the seconds to `until`'s
    doing, or None)."""
    from machine import sprung, worker
    g, last, flips, got, t0 = s['g'], None, 0, None, s['g']['t']
    for _ in range(int(round(seconds / worker.DT))):
        worker.stepped(s)
        doing = sprung.doing(g)
        if doing != last:
            flips, last = flips + (last is not None and g['t'] - t0 >= after), doing
            if got is None and doing == until:
                got = g['t'] - t0
    return flips, got


def _speed(s):
    v = s['g']['est']['vel']
    return math.hypot(float(v[0]), float(v[2]))


def test_a_stop_settles_and_f_is_heard(report):
    """Walked 2 s and told to stop, her doing comes to the stand within the brake's own 1.2 s
    and, two of her rhythm's swings on, holds; told to sit she is seated within 5 s, told to rise she stands within 6 s; a scene
    laid anew with a walk asked at once, she walks at once."""
    from machine import floor, worker
    s = worker.session()
    worker.apply(s, {'scenario': 'flat'})
    _run(s, 0.5)
    worker.apply(s, {'twist': WALK})
    _run(s, 2.0)
    worker.apply(s, {'twist': STOP})
    flips, stood = _run(s, 4.0, until='#201', after=2.0)
    report.check('the stop: the stand within 1.3 s, under 10 flips of her doing from 2 s on',
                 stood is not None and stood < 1.3 and flips < 10 and _speed(s) < 0.2,
                 'stand after %s, %d flips, %.2f m/s at the end' % (stood, flips, _speed(s)))
    x, _, z = s['g']['est']['pos']
    floor.mocap(s['g']['world'], 'chair', (float(x), 0.0, float(z) - 0.30), math.pi)
    worker.apply(s, {'say': 'sit_chair'})
    _flips, seated = _run(s, 5.0, until='seated')
    worker.apply(s, {'say': 'rise_chair'})
    _flips, stand = _run(s, 6.0, until='stand')
    report.check('F sits her within 5 s and rises her within 6 s',
                 seated is not None and stand is not None, (seated, stand))
    worker.apply(s, {'scenario': 'bed'})
    worker.apply(s, {'twist': WALK})
    _flips, walking = _run(s, 1.0, until='init_walk')
    report.check('a scene laid anew, the walk asked at once begins within 0.1 s',
                 walking is not None and walking < 0.1, walking)


ROSTER = (test_a_stop_settles_and_f_is_heard,)


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
