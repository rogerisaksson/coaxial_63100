"""The landscape behind the board (coaxial.graphics.ground): the floor laid at once."""
import sys

from coaxial.graphics import approach, engine, ground, wireframe
from structure_kit import Report
from tools.dev.focus import chosen


def test_the_floor_is_laid_at_once(report):
    """`_lay` lays every segment at once: cell for cell, bit for bit, what `_segment` and
    `_ground_dot` lay one dot at a time, and `_shaded` greys them as `_greys` does - flat and
    rolled by a bank, at two sizes."""

    def dot_by_dot(runs, width, height, roll):
        cells = {}
        for line, points, fade in runs:
            def dot(fx, fy, depth, _k, line=line, fade=fade):
                ground._ground_dot(cells, width, height, fx, fy, depth, line, fade)
            prev = None
            for fx, fy, depth in points:
                if roll is not None:
                    fx, fy = roll(fx, fy)
                if prev is not None:
                    ground._segment(dot, prev, fx, fy, depth, line)
                prev = (fx, fy, depth)
        return cells

    def same(a, b):
        return set(a) == set(b) and all(a[at][:4] == b[at][:4] and a[at][5] == b[at][5]
                                        and len(a[at][4]) == len(b[at][4]) for at in a)

    for width, height in ((60, 20), (108, 44)):
        cam = engine.camera(width, height, 1.5, distance=3.2, zoom=1.0, tip=wireframe.CAMERA_TIP)
        static = ground._ground_static(width, height, 3.2, cam['view'])
        for step, bank, bob in ((0, None, None), (25, 12.0, 0.7), (71, -31.0, -1.2)):
            runs = ground._runs(static, width, height, step)
            roll = (approach.roller({'bank': bank, 'bob': bob}, width, height, static)
                    if bank is not None else None)
            once, slow = ground._lay(runs, width, height, roll), dot_by_dot(runs, width, height, roll)
            laid = ground._laid(runs, width, height, roll)
            shaded = {} if laid is None else {at: (mk, (g, g, g)) for at, mk, g in zip(
                laid[0].tolist(), laid[1].tolist(), ground._shaded(laid, static).tolist())}
            report.check('the floor laid at once is the floor laid dot by dot, %dx%d step %d%s'
                         % (width, height, step, ' rolled' if roll else ''),
                         same(once, slow) and shaded == ground._greys(slow, static),
                         '%d cells against %d' % (len(once), len(slow)))


ROSTER = (test_the_floor_is_laid_at_once,)


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
