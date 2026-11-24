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
    shell = ['PS C:\\> .\\system_terminal.ps1'] + [''] * (HEIGHT - 1)
    lines = ['row %02d' % r for r in range(HEIGHT)]
    frame = tube._on_frame(shell, lines)
    dot, line, unfolding, whole = (frame(u, WIDTH, HEIGHT) for u in (tube.DOT / 2, (tube.DOT + tube.LINE) / 2,
                                                                     (tube.LINE + 1.0) / 2, 1.0))
    report.check('a dot, a line, the frame unfolding over the shell, whole at the end',
                 'system_terminal' in dot[0] and _white(dot[HEIGHT // 2]) < _white(line[HEIGHT // 2])
                 and 'system_terminal' in unfolding[0] and 0 < sum(r.startswith('row') for r in unfolding) < HEIGHT
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


def test_a_frame_is_written_as_the_crt_lays_it(report):
    """A curtain's frame (`stage._Tube`): its rows as the CRT lays them, written from the screen's top, a row a
    line, the width each - the cells rich's Live drew through its screen, two splits of the frame fewer."""
    import re
    from rich.console import Console
    from rich.live import Live
    from rich.text import Text
    from terminal.ui import chrome, stage
    frame = Text('\n'.join('row %02d' % r for r in range(HEIGHT)))

    def console(file):
        return Console(file=file, force_terminal=True, width=WIDTH, height=HEIGHT, color_system='truecolor',
                       legacy_windows=False)

    def rows(said):
        # escapes out, the CRT's snow and sweep (braille, by the clock) as the blanks they fell in
        cells = re.sub('[⠀-⣿]', ' ', re.sub(r'\x1b\[[0-9;?]*[A-Za-z]', '', said))
        return [row for row in cells.split('\n') if row.startswith('row')]
    ours, theirs = io.StringIO(), io.StringIO()
    with stage.curtain(console(ours)) as page:
        mark = len(ours.getvalue())
        page.update(frame, refresh=True)
        written = ours.getvalue()[mark:]
    with Live(console=console(theirs), screen=True, auto_refresh=False, transient=True) as live:
        mark = len(theirs.getvalue())
        live.update(chrome.Crt(frame), refresh=True)
        drawn = theirs.getvalue()[mark:]
    got, want = rows(written), rows(drawn)
    report.check("from the screen's top, a row a line, as rich's Live drew it",
                 written.startswith('\x1b[H') and len(got) == HEIGHT and all(len(r) == WIDTH for r in got)
                 and got == want, '%d rows, %d as wide as the screen, %d as rich drew them'
                 % (len(got), sum(len(r) == WIDTH for r in got), sum(a == b for a, b in zip(got, want))))


ROSTER = (test_the_tube_switches_on_over_the_shell, test_the_tube_switches_off_from_the_last_frame,
          test_a_frame_is_written_as_the_crt_lays_it)


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
