"""The tube switched on over the shell and off from the last frame (`terminal.ui.tube`)."""
import io
import sys

from tools.dev.focus import chosen
from views_kit import Report

#: The screen the frames are drawn at, cells.
WIDTH, HEIGHT = 80, 24


def _white(row):
    """The cells of the beam's white in `row`."""
    return len(row.split('255;255;255m', 1)[-1].split('\x1b', 1)[0]) if '255;255;255m' in row else 0


def test_the_tube_switches_on_over_the_shell(report):
    """On: the shell's text kept, a white dot at its centre, drawn out to a line, the first frame
    unfolding from it over the shell, whole at the end."""
    from terminal.ui import tube
    shell = ['PS C:\\> .\\coaxial_tty.ps1'] + [''] * (HEIGHT - 1)
    lines = ['row %02d' % r for r in range(HEIGHT)]
    frame = tube._on_frame(shell, lines)
    dot, line, unfolding, whole = (frame(u, WIDTH, HEIGHT) for u in (tube.DOT / 2, (tube.DOT + tube.LINE) / 2,
                                                                     (tube.LINE + 1.0) / 2, 1.0))
    report.check('a dot, a line, the frame unfolding over the shell, whole at the end',
                 'coaxial_tty' in dot[0] and _white(dot[HEIGHT // 2]) < _white(line[HEIGHT // 2])
                 and 'coaxial_tty' in unfolding[0] and 0 < sum(r.startswith('row') for r in unfolding) < HEIGHT
                 and whole == lines, '%d rows unfolded' % sum(r.startswith('row') for r in unfolding))


def test_the_tube_switches_off_from_the_last_frame(report):
    """Off: the last frame whole at first, folded to a line, a dot, nothing; the shell back on the
    main screen, its cursor shown."""
    from terminal.ui import tube
    lines = ['row %02d' % r for r in range(HEIGHT)]
    frame = tube._off_frame(lines)
    first, folded, gone = (frame(u, WIDTH, HEIGHT) for u in (0.0, tube.FOLDED / 2, 1.0))
    real, sys.stdout = sys.stdout, io.StringIO()
    try:
        tube.off(lines)
        said = sys.stdout.getvalue()
    finally:
        sys.stdout = real
    report.check('the frame whole, then folded about its centre, then dark; the main screen back',
                 first == lines and sum(row.startswith('row') for row in folded) < HEIGHT
                 and not any(row.startswith('row') for row in gone) and said.endswith(tube.MAIN + tube.SHOW),
                 '%d rows folded' % sum(row.startswith('row') for row in folded))


ROSTER = (test_the_tube_switches_on_over_the_shell, test_the_tube_switches_off_from_the_last_frame)


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
