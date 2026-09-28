"""The terminal the pages draw on: its rate, its chrome, the CRT, the console, a frame as it rasterises."""
import sys

from tools.dev.focus import chosen
from views_kit import Report


def test_the_screen_keeps_its_own_rate(report):
    """A page drawing at 2.5 Hz - its data's rate - is shown again at UI_HZ between
    draws, so what moves with the clock never waits on the data."""
    import io
    from rich.console import Console
    from rich.live import Live
    from rich.text import Text
    from terminal.ui import screen

    shown, drawn = [], []
    real = Live.update

    def counted(self, renderable, *, refresh=False):
        shown.append(renderable)
        return real(self, renderable, refresh=refresh)

    def draw():
        drawn.append(Text('frame %d' % len(drawn)))
        return drawn[-1]
    Live.update = counted
    try:
        screen.run_view(Console(file=io.StringIO()), False, 0.4, 2, draw,
                        scroll_keys=False)
    finally:
        Live.update = real
    again = sum(1 for r in shown if r is drawn[0])
    report.check('between two draws the frame is shown again at UI_HZ',
                 len(drawn) == 2 and again >= 0.4 * screen.UI_HZ - 2,
                 '%d draws, the first shown %d times' % (len(drawn), again))


def test_the_chrome_at_its_edges(report):
    """The chrome off its usual ground: no region to dress, control codes and wide
    characters in what it dresses, a piece of it with no room."""
    import io
    from rich.console import Console
    from rich.segment import ControlType, Segment
    from rich.style import Style
    from rich.text import Text
    from terminal.ui import chrome

    court = Console(file=io.StringIO(), record=True, force_terminal=True, width=40)
    court.print(chrome.Chrome(Text('plain'), 'METER BRIDGE'))
    report.check('with no height there is no region to dress: the drawing as it is',
                 court.export_text().strip() == 'plain')
    cells = chrome._cells([Segment('a'), Segment('', None, [(ControlType.HOME,)]), Segment('\u6f22b')])
    report.check('a control code takes no cell, a wide character two',
                 [ch for ch, _style in cells] == ['a', '\u6f22', '', 'b'], str(cells))
    rows = [[[' ', None] for _ in range(6)] for _ in range(2)]
    rows[0][2] = ['x', Style()]
    report.check('a piece goes in whole where it fits on blanks, or not at all',
                 not chrome._put(rows, 5, 0, 'ab', None)
                 and not chrome._put(rows, 0, 5, 'ab', None)
                 and not chrome._put(rows, 0, 1, 'ab', None)
                 and chrome._put(rows, 1, 1, 'ab', None) and rows[1][1][0] == 'a')


def test_each_page_draws_on_a_terminal(report):
    """Every page, simulated, on a terminal of the bench's size through the page tool: the
    full-screen layout, the CRT and the chrome - the path a pipe never takes."""
    import contextlib
    import io
    import re
    from tools.render import page

    drawn = {}
    for name in sorted(page.PAGES):
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            art = page.frame(name, 150, 44, frames=2)
        drawn[name] = [re.sub('\x1b\\[[0-9;]*m', '', line) for line in art.split('\n')]
    short = {name: len(lines) for name, lines in drawn.items() if len(lines) < 44}
    report.check('every page fills a 150 x 44 terminal', not short, str(short))
    bare = [name for name, lines in drawn.items()
            if not any(0x2801 <= ord(ch) <= 0x28FF for line in lines for ch in line)]
    report.check('and every one is on the CRT - its snow in braille', not bare, str(bare))


def test_the_console_it_draws_on(report):
    """stage(): VT processing asked of a Windows console, nothing where there is none;
    truecolor on a terminal rich guessed short of it."""
    from terminal.ui import stage

    asked = []

    class Kernel:
        def GetStdHandle(self, which):
            asked.append(('handle', which))
            return 7

        def GetConsoleMode(self, handle, mode):
            asked.append(('get', handle))
            return 1

        def SetConsoleMode(self, handle, mode):
            asked.append(('set', handle, mode & 0x0004))

    class Windows:
        kernel32 = Kernel()

    had = getattr(stage.ctypes, 'windll', None)
    try:
        stage.ctypes.windll = Windows()
        stage._vt_on()
        if hasattr(stage.ctypes, 'windll'):
            del stage.ctypes.windll
        stage._vt_on()
    finally:
        if had is not None:
            stage.ctypes.windll = had
    report.check('VT processing is switched on through kernel32, and skipped without it',
                 asked == [('handle', -11), ('get', 7), ('set', 7, 4)], str(asked))

    made = []
    real = stage.Console

    class Guessed(real):
        def __init__(self, *args, **kwargs):
            made.append(kwargs.get('color_system'))
            super().__init__(*args, **dict(kwargs, force_terminal=True))

        @property
        def color_system(self):
            return 'truecolor' if made[-1] == 'truecolor' else '256'
    stage.Console = Guessed
    try:
        console = stage.stage()
    finally:
        stage.Console = real
    report.check('a terminal guessed at 256 colours is drawn in truecolor',
                 made == [None, 'truecolor'] and console.color_system == 'truecolor',
                 str(made))


def test_the_crt_draws_on_the_terminal(report):
    """Through a screen-mode Live - the terminal's path, which gives its renderable no
    height - the CRT draws: snow over the screen, the braille band at the beam."""
    import io
    from rich.console import Console
    from rich.live import Live
    from rich.text import Text
    from terminal.ui import chrome

    def dots(t):
        out = io.StringIO()
        court = Console(file=out, force_terminal=True, width=100, height=30,
                        color_system='truecolor', legacy_windows=False)
        was = chrome.time.monotonic
        chrome.time.monotonic = lambda: t
        try:
            with Live(console=court, screen=True, auto_refresh=False, transient=True) as live:
                live.update(chrome.Crt(Text('hello')), refresh=True)
        finally:
            chrome.time.monotonic = was
        return sum(1 for ch in out.getvalue() if 0x2801 <= ord(ch) <= 0x28FF)
    quiet, beam = dots(0.0), dots(chrome.SWEEP_S * 0.5)
    report.check('the CRT draws through a screen-mode Live: snow, and more at the beam',
                 quiet > 0 and beam > quiet, '%d dots, %d with the beam mid-screen'
                 % (quiet, beam))


def test_the_terminal_is_asked_how_tall_a_cell_is(report):
    """The cell's shape is measured, not assumed."""
    from terminal.ui import aspect

    report.check('a terminal 1200 by 800 pixels over 100 by 40 cells has a '
                 'cell 1.67 times as tall as it is wide',
                 abs((aspect.cell_aspect_of((800, 1200), (40, 100)) or 0)
                     - 5.0 / 3.0) < 1e-9)
    report.check('nothing divisible by zero comes back as a number',
                 aspect.cell_aspect_of((0, 0), (1, 1)) is None
                 and aspect.cell_aspect_of((800, 1200), (0, 100)) is None)
    report.check('and a reply that cannot be a cell is refused',
                 aspect.cell_aspect_of((8000, 100), (40, 100)) is None
                 and aspect.cell_aspect_of((10, 1200), (40, 100)) is None,
                 str(aspect.ASPECT_RANGE))
    report.check('a pipe is never asked, so the query cannot land in a '
                 'render', aspect.probe_aspect(console=False) is None)


def test_every_page_scrolls_its_boxes(report):
    """The instrument column pages on every view: the arrows move it a box,
    a click on its markers and a drag over it too, and the key bar says
    SCROLL only while there is somewhere to go.
    """
    from terminal.ui import stage
    from terminal.ui import scroll

    class Size:
        width, height = 100, 14

    class Console:
        is_terminal = True
        size = Size()

    console = Console()
    boxes = [stage.hud('BOX %d' % i, [('a', 1), ('b', 2), ('c', 3)])
             for i in range(6)]
    shown = scroll.paged(console, boxes)
    at, seen, total = scroll.scroll_state(console)['pages']
    report.check('a short terminal shows what fits and says the rest are '
                 'below', at == 0 and 0 < seen < total == 6
                 and scroll.DOWN in shown[-1].plain,
                 '%d of %d shown' % (seen, total))
    scroll.scroll_by(console, 1)
    scroll.scroll_by(console, 1)
    shown = scroll.paged(console, boxes)
    at, seen, total = scroll.scroll_state(console)['pages']
    report.check('two arrows down start two boxes in, with a row saying so',
                 at == 2 and scroll.UP in shown[0].plain, '%d above' % at)
    for _ in range(10):
        scroll.scroll_by(console, 1)
    scroll.paged(console, boxes)
    at, seen, total = scroll.scroll_state(console)['pages']
    report.check('and the bottom is a full column, not one box and air',
                 seen == total and at < total, '%d..%d of %d' % (at, seen, total))
    scroll.scroll_click(console, Size.width - 2, 2)
    scroll.paged(console, boxes)
    report.check('a click on the top marker goes up one',
                 scroll.scroll_state(console)['pages'][0] == at - 1)
    scroll.scroll_click(console, 10, 2)
    report.check('a click over the drawing takes no grip, so a drag there '
                 'does not scroll', not scroll.scroll_state(console)['grip'])
    before = scroll.scroll_state(console)['pages'][0]
    scroll.scroll_drag(console, 20.0)
    report.check('...and moves nothing', scroll.scroll_state(console)['at'] == before)

    class Pipe:
        is_terminal = False

    report.check('piped, nothing is windowed',
                 len(scroll.paged(Pipe(), boxes)) == 6)
    try:
        scroll.paged(True, boxes)
        refused = False
    except TypeError:
        refused = True
    report.check('and a view handing over its is_terminal flag instead of '
                 'the console is refused - where a piped test run sees it',
                 refused)


def test_a_frame_rasterises_as_the_terminal_draws_it(report):
    """`ansi.image` draws a coloured frame cell by cell, the way the bench's
    terminal shows it: the notebooks' pictures, and `tools/render/ansi2png.py`.
    """
    from machine import ansi

    frame = (ansi.paint('ab', ansi.RED) + 'c\n'
             + ansi.paint('\u28ff', ansi.GREEN) + ' \u2801')
    rows = ansi.parse(frame)
    report.check('parse keeps every cell and its colour',
                 [len(r) for r in rows] == [3, 3]
                 and rows[0][0][1] == ansi.rgb(ansi.RED)
                 and rows[0][2][1] == ansi.PLAIN
                 and rows[1][0][1] == ansi.rgb(ansi.GREEN),
                 [[(c, fg) for c, fg, _bg in r] for r in rows])
    cell = (8, 16)
    img = ansi.image(frame, cell=cell)
    report.check('and the image is one cell per character',
                 img.size == (3 * cell[0], 2 * cell[1]), img.size)
    px = img.load()
    if px is None:
        raise AssertionError('no pixels')

    def ink(x0, y0, wants):
        seen = [px[x, y] for x in range(x0, x0 + cell[0])
                for y in range(y0, y0 + cell[1])]
        return any(wants(p) for p in seen), seen

    red, _ = ink(0, 0, lambda p: p[0] > 150 and p[1] < 60 and p[2] < 60)
    green, _ = ink(0, cell[1], lambda p: p[1] > 150 and p[0] < 60 and p[2] < 60)
    lit, blank = ink(cell[0], cell[1], lambda p: p != (0, 0, 0))
    report.check('the red letters, the green braille and the blank cell '
                 'are drawn in their own inks',
                 red and green and not lit, (red, green, not lit))


def test_the_attitude_caps_its_frame_rate(report):
    """Every view draws at most FPS_CAP frames a second whatever `--hz` asks - the bench's
    word, so the laptop's fans stay down - BOARD ATTITUDE never slower than one every two
    seconds."""
    from terminal.ui.screen import FPS_CAP
    from terminal.views import show_orientation as view

    report.check('the cap is twenty', FPS_CAP == 20.0, str(FPS_CAP))
    report.check('--hz 60 draws at twenty',
                 abs(view.period_of(60.0) - 1.0 / 20.0) < 1e-9,
                 '%.4f s' % view.period_of(60.0))
    report.check('--hz 20, the default, is honoured',
                 abs(view.period_of(20.0) - 0.05) < 1e-9
                 and view.parse_args(['--simulated']).hz == 20.0,
                 '%.4f s' % view.period_of(20.0))
    report.check('and nothing slower than one frame every two seconds',
                 view.period_of(0.0) == 2.0, '%.4f s' % view.period_of(0.0))


ROSTER = (test_the_screen_keeps_its_own_rate, test_the_chrome_at_its_edges,
          test_each_page_draws_on_a_terminal, test_the_console_it_draws_on,
          test_the_crt_draws_on_the_terminal, test_the_terminal_is_asked_how_tall_a_cell_is,
          test_every_page_scrolls_its_boxes, test_a_frame_rasterises_as_the_terminal_draws_it,
          test_the_attitude_caps_its_frame_rate)

def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
