"""The console style, defined once, and the renderer built around it.

The look is the two reference screens: Alien's Nostromo readouts and the
green phosphor terminal. What that means concretely, as rules:

  * ONE SOLID TITLE BAR across the top - dark teal, the view's name on it,
    the LIVE/SIMULATED chip at its right edge in its meaning colour.
  * The DRAWING fills a heavy-framed viewport; instruments sit in a fixed
    column of rounded boxes beside it. Nothing floats in a corner.
  * VALUES GLOW AMBER. Labels recede in ash. Names are cyan. That single
    assignment is most of the look: a dark screen where the numbers are
    the light sources.
  * A KEY BAR closes the bottom, reversed like the terminal reference.

Every style is NAMED here in the Theme and nowhere else - a view says
`value` or `label`, never a colour number, so the palette is one edit.
"""
import ctypes
import re
import threading
import time
from contextlib import contextmanager

from rich import box
from rich.align import Align
from rich.columns import Columns
from rich.console import Console, Group
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.progress import BarColumn, Progress, TextColumn
from rich.table import Table
from rich.text import Text
from rich.theme import Theme

from coaxial.comm import broker
from terminal.ui.marquee import Marquee
from terminal.ui.rate import Corner, rate_of
from terminal.ui.scroll import DOWN, HUD_WIDTH, UP, _fills, _rows_of, paged, scroll_state


#: The palette, named. Blade Runner's teal and sodium over Alien's phosphor
#: green chip. Meaning colours (LIVE green, SIMULATED yellow, alarm red)
#: keep their meanings and are never reused for decoration.
THEME = Theme({
    'bar':        'bold color(51) on color(23)',   # the title band
    'bar.dim':    'color(44) on color(23)',
    'name':       'bold color(44)',                # what things are called
    'label':      'color(66)',                     # the street
    'value':      'color(214)',                    # the light source
    'frame':      'color(23)',                     # the viewport's edge
    'frame.hud':  'color(66)',                     # an instrument's edge
    'keys':       'color(242) on grey15',
    'keys.key':   'bold color(44) on grey15',
    'chip.live':  'black on green3',
    'chip.sim':   'black on yellow3',
    'alarm':      'bold black on red3',
})


def _vt_on():
    """Enable VT processing on the Windows stdout console BEFORE rich looks
    at it.
    """
    try:
        kernel = ctypes.windll.kernel32
    except (ImportError, AttributeError):
        return
    handle = kernel.GetStdHandle(-11)
    mode = ctypes.c_uint()
    if kernel.GetConsoleMode(handle, ctypes.byref(mode)):
        kernel.SetConsoleMode(handle, mode.value | 0x0004)


def stage():
    """The console every view draws on. Plain and sequential when piped."""
    _vt_on()
    console = Console(highlight=False, theme=THEME)
    if console.is_terminal and console.color_system != 'truecolor':
        # rich guesses the colour depth from the environment, and on a Windows
        # console with no COLORTERM it guesses 256 or 16 - the glow ramp's
        # 24-bit gradient was requantised to the palette's five cyans, a hard
        # iso-line across the board where 99 luma met 106.
        console = Console(highlight=False, theme=THEME,
                          color_system='truecolor')
    return console


@contextmanager
def curtain(console):
    """The Live a view runs inside: the alternate screen on a terminal, so
    the shell underneath is untouched and comes back on exit.
    """
    live = Live(console=console, screen=console.is_terminal,
                auto_refresh=False, transient=console.is_terminal)
    with live:
        if console.is_terminal:
            console.clear()
        yield live


@contextmanager
def boot(label, console=None):
    """A brisk amber progress strip over whatever the block actually does."""


    court = console or stage()
    if not court.is_terminal:
        yield lambda *args: None
        return

    bar = Progress(
        BarColumn(bar_width=28, complete_style='value',
                  finished_style='value', style='frame.hud'),
        # The text AFTER the bar, bracketed.
        TextColumn('[{task.description}]', style='label', markup=False),
        console=court, transient=True)
    task = bar.add_task(label, total=100)
    stop = threading.Event()
    closed = []
    ceiling = [90.0]

    def creep():
        while not stop.is_set():
            if bar.tasks[0].completed < ceiling[0]:
                bar.update(task, advance=1.5)
            time.sleep(0.03)

    def finish():
        """The bar to the end and gone - once."""
        if closed:
            return
        closed.append(True)
        stop.set()
        walker.join(timeout=0.5)
        bar.update(task, completed=100)
        time.sleep(0.06)
        bar.stop()

    def step(share=None, text=None):
        if share is None:
            finish()
            return
        done = max(bar.tasks[0].completed, 100.0 * share)
        ceiling[0] = min(95.0, 100.0 * share + 8.0)
        bar.update(task, completed=done,
                   description=text if text else bar.tasks[0].description)

    bar.start()
    walker = threading.Thread(target=creep, daemon=True)
    walker.start()
    try:
        yield step
    finally:
        step()


def live(count):
    """The green LIVE chip, with the sessions on the port when known."""
    label = (' LIVE %d SESSION%s ' % (count, '' if count == 1 else 'S')
             if count else ' LIVE ')
    return Text(label, style='chip.live')


def chip(origin):
    """The meaning tag. Green LIVE / yellow SIMULATED, never restyled."""
    if origin.real:
        return live(broker.clients() or 0)
    return Text(' SIMULATED ', style='chip.sim')


def band_of(name, extra='', tag=None):
    """The band every page wears: `name` hard left, `extra` dim after it,
    `tag` right with one cell of air before the band's end.
    """
    left = Text.assemble((name, 'bar'),
                         ('   ' + extra if extra else '', 'bar.dim'))
    right = Text.assemble(tag, (' ', 'bar.dim')) if tag else Text('')
    return band(left, right)


def header(title, origin):
    """A view's band: its name, the port, the meaning chip right."""
    where = ("PORT: %s" % origin.port if origin.real
             else "" if origin.label == "Simulated" else origin.label)
    return band_of(title, where, chip(origin))


#: Cells the title band is set in from the left edge. Painted from column
#: 0 it stood out left of every box under it; two cells in - the frames'
#: title column - read as too far right. One.
BAND_INSET = 1


def band(*cells):
    """The title band: `cells` on the band's colour, an inset unpainted at
    both ends.
    """
    bar = Table.grid(expand=True, padding=0)
    bar.add_column(width=BAND_INSET)
    bar.add_column(style='bar.dim', justify='left', ratio=1)
    if len(cells) > 1:
        # Sized to its content: right-justified, rich strips the cell's
        # trailing spaces and the air before the band's end went with them.
        bar.add_column(style='bar.dim', width=cells[-1].cell_len)
    bar.add_column(width=BAND_INSET)
    bar.add_row(Text(''), *cells, Text(''))
    return bar


def footer(pairs):
    """The key bar: KEY: WHAT pairs on a reversed strip, terminal style."""
    line = Text('  ', style='keys')
    for i, (key, what) in enumerate(pairs):
        if i:
            line.append('  |  ', style='keys')
        if key:
            line.append(key, style='keys.key')
            line.append(': ', style='keys')
        if isinstance(what, Text):
            line.append_text(what)
        else:
            line.append(what, style='keys')
    bar = Table.grid(expand=True)
    bar.add_column(justify='left')
    bar.add_row(line)
    bar.style = 'keys'
    return bar


def hud(title, rows):
    """One instrument: labels recede, values glow, rounded frame."""
    grid = Table.grid(padding=(0, 1))
    grid.add_column(style='label', justify='right', no_wrap=True)
    grid.add_column(style='value', no_wrap=True)
    for row in rows:
        if isinstance(row, tuple):
            value = row[1]
            grid.add_row(str(row[0]),
                         value if isinstance(value, Text) else str(value))
        elif isinstance(row, Text):
            grid.add_row('', row)
        else:
            grid.add_row('', Text.from_ansi(str(row)))
    return Panel(grid, title=Text(title, style='name'), title_align='left',
                 box=box.ROUNDED, border_style='frame.hud',
                 padding=(0, 1), expand=True)


_ESCAPE = re.compile(r'\x1b')


def viewport(title, art, corner=''):
    """The drawing, centred in a heavy frame that owns its region; `corner`
    (a `Rate.label()`) over its top-left cells."""
    return Panel(Corner(Align(Marquee(art), align='center',
                              vertical='middle'), corner),
                 title=Text(' %s ' % title, style='name'),
                 title_align='left', box=box.HEAVY, border_style='frame',
                 padding=(0, 1), expand=True)


def frame_of(console, origin, title, art, boxes, keys, art_title=None,
             under=None):
    """THE template: title band, viewport left, instruments right, key bar."""
    if not _fills(console):
        return Group(header(title, origin),
                     viewport(art_title or title, art),
                     *([under] if under is not None else []),
                     *boxes, footer(keys))

    # THE COLUMN IS PAGED HERE, for every view at once, and the key bar says so
    # only while there is something to scroll to.
    boxes = paged(console, boxes)
    rate = rate_of(console).label()
    at, seen, total = scroll_state(console)['pages']
    if at or seen < total:
        keys = list(keys) + [(UP + ' ' + DOWN, 'SCROLL')]
    body = Layout()
    art_region = Layout(name='art')
    if under is None:
        art_region.update(viewport(art_title or title, art, rate))
    else:
        # `under` is a fixed height because the viewport takes the rest: a box
        # that grew with its content would push the bars off the bottom of a
        # short terminal instead of the other way round.
        art_region.split_column(
            Layout(viewport(art_title or title, art, rate), name='top'),
            Layout(under, name='under', size=_rows_of(under) + 2))
    body.split_row(art_region,
                   Layout(Group(*boxes) if boxes else Text(''),
                          name='hud', size=HUD_WIDTH))

    whole = Layout()
    whole.split_column(Layout(header(title, origin), size=1),
                       Layout(body, name='body'),
                       Layout(footer(keys), size=1))
    return whole


def panels_of(console, origin, title, groups, keys):
    """The template for table views: a FIXED grid of instruments, no art."""
    if not _fills(console):
        flat = [Columns(row, padding=(0, 1), expand=False)
                for row in groups]
        return Group(header(title, origin), *flat, footer(keys))

    body = Layout(name='body')
    row_layouts = []
    for r, row in enumerate(groups):
        strip = Layout(name='row%d' % r)
        if len(row) > 1:
            strip.split_row(*[Layout(cell) for cell in row])
        else:
            strip.update(row[0])
        # A single-cell row is a strip - the dash - and stays one line.
        if len(row) == 1 and isinstance(row[0], Text):
            strip.size = 1
        row_layouts.append(strip)
    body.split_column(*row_layouts)
    # The grid sits in the same heavy frame the drawing views give their
    # viewport, so a table page owns its region the way they do - the session
    # read as loose boxes on the bare screen.
    framed = Panel(body, title=Text(' %s ' % title, style='name'),
                   title_align='left', box=box.HEAVY, border_style='frame',
                   padding=0, expand=True)

    whole = Layout()
    whole.split_column(Layout(header(title, origin), size=1),
                       Layout(framed, name='grid'),
                       Layout(footer(keys), size=1))
    return whole
