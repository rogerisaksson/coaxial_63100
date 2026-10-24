"""The tube switched on over the shell and off from the last frame (`terminal.ui.tube`)."""
import io
import sys

from tools.dev.focus import chosen
from views_kit import Report

#: The screen the frames are drawn at, cells.
WIDTH, HEIGHT = 80, 24


def test_the_tube_switches_on_over_the_shell(report):
    """On: the shell's text kept outside the band as it opens, a white beam at the centre first,
    black at the end - the front page next."""
    from terminal.ui import tube
    shell = ['PS C:\\> .\\coaxial_tty.ps1'] + [''] * (HEIGHT - 1)
    frame = tube._on_frame(shell)
    beam, opening, end = (frame(u, WIDTH, HEIGHT) for u in (tube.BEAM / 2, (tube.BEAM + tube.OPEN) / 2, 1.0))
    report.check('the shell kept while the beam is drawn and the band opens; black at the end',
                 'coaxial_tty' in beam[0] and 'coaxial_tty' in opening[0] and '255;255;255' in beam[HEIGHT // 2]
                 and all('48;2;0;0;0m' in row for row in end), repr(beam[HEIGHT // 2][:40]))


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
