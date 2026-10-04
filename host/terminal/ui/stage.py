"""The console style, defined once, and the renderer built around it.

The look is the two reference screens: Alien's Nostromo readouts and the
green phosphor terminal. What that means concretely, as rules:

  * One solid title bar across the top - dark teal, the view's name on it,
    the LIVE/SIMULATED chip at its right edge in its meaning colour.
  * The drawing fills a heavy-framed viewport; instruments sit in a fixed
    column of rounded boxes beside it. Nothing floats in a corner.
  * Values glow amber. Labels recede in ash. Names are cyan. That single
    assignment is most of the look: a dark screen where the numbers are
    the light sources.
  * A key bar closes the bottom, reversed like the terminal reference.
  * Over the drawing, the attitude view's HUD (`terminal.ui.chrome`): a graticule, lock
    brackets, the clock, a status tag in kana; the band carries the kana name too.

Every style is named here in the Theme and nowhere else - a view says
`value` or `label`, never a colour number, so the palette is one edit.
"""
import ctypes
import re
import threading
import time
import weakref
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
from terminal.ui.chrome import KANA, Chrome, Crt, clock
from terminal.ui.marquee import Marquee
from terminal.ui.sto import chip as sto_chip
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
    'rate':       '#2c5258',                       # the frame rate, in the background
    'keys':       'color(242) on grey15',
    'keys.key':   'bold color(44) on grey15',
    'chip.live':  'black on green3',
    'chip.sim':   'black on yellow3',
    'chip.emu':   'black on #3cafb9',
    'chip.nat':   'black on #6ea0ff',
    'chip.virtual': 'black on #a58cff',
    'chip.dynamic': 'black on #ff9f5a',
    'chip.sto':   'black on color(214)',         # the drivers supplied: the light source
    'chip.sto.off': 'color(66) on color(23)',   # the safe state, the band's own
    'alarm':      'bold black on red3',
})


def _vt_on():
    """Enable VT processing on the Windows stdout console before rich looks
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
    the shell underneath is untouched and comes back on exit; on one, every
    frame on the CRT (`chrome.Crt`).
    """
    live = Live(console=console, screen=console.is_terminal,
                auto_refresh=False, transient=console.is_terminal)
    with live:
        if console.is_terminal:
            console.clear()
            yield _Tube(live)
        else:
            yield live


class _Tube:
    """A Live whose every frame is shown on the CRT."""

    def __init__(self, live):
        self._live = live

    def update(self, renderable, *, refresh=False):
        self._live.update(Crt(renderable), refresh=refresh)

    def __getattr__(self, name):
        return getattr(self._live, name)


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
        # The text after the bar, bracketed.
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
    """The meaning tag: green LIVE on a board; EMU SIM NAT with the page's own lit - teal the
    image on Renode, yellow the stand-in, blue the board layer native; violet VIRTUAL and
    amber DYNAMIC for machines. Never restyled."""
    if origin.kind == 'emulator':
        return modes(mode_of(origin.port))
    if origin.kind == 'virtual':
        return VIRTUAL_CHIP
    if origin.kind == 'dynamic':
        return DYNAMIC_CHIP
    if origin.real:
        return live(broker.clients() or 0)
    return modes('SIM')


#: The execution modes the band names, the page's own in its chip's colours (the bench's
#: word, 2026-09-28): the firmware's image on Renode, the stand-in, the board layer built for
#: this host.
MODES = (('EMU', 'chip.emu'), ('SIM', 'chip.sim'), ('NAT', 'chip.nat'))


def modes(lit):
    """EMU SIM NAT, `lit` in its chip's colours and the others the band's dim."""
    return Text.assemble(*[(' %s ' % name, style if name == lit else 'bar.dim')
                           for name, style in MODES])


def mode_of(port):
    """The mode an emulated board's port runs it in: NAT on native://, else EMU."""
    return 'NAT' if str(port).startswith('native://') else 'EMU'

#: Virtual actuators' chip (machine.virtual): no board, nothing simulated.
VIRTUAL_CHIP = Text(' VIRTUAL ', style='chip.virtual')

#: A body with mass's chip (machine.physics): no board, the physics simulated.
DYNAMIC_CHIP = Text(' DYNAMIC ', style='chip.dynamic')


def band_of(name, extra='', tag=None, gauges=None):
    """The band every page wears: `name` hard left, its kana and `extra` dim after it, a
    page's `gauges` (Text) after them, the clock and `tag` right with one cell of air before
    the band's end.
    """
    kana = KANA[name][0] if name in KANA else ''
    left = Text.assemble((name, 'bar'), ('  ' + kana if kana else '', 'bar.dim'),
                         ('   ' + extra if extra else '', 'bar.dim'))
    if gauges is not None:
        left.append('   ')
        left.append_text(gauges)
    return band(left, _Ticking(tag))


class _Ticking(Text):
    """The band's right end - the clock, then `tag` - read when it is drawn, so a
    frame shown again between draws keeps time."""

    def __init__(self, tag):
        super().__init__()
        self._tag = tag
        self.append_text(self._now())

    def _now(self):
        return Text.assemble((clock() + '  ', 'bar.dim'), self._tag or '', (' ', 'bar.dim'))

    def __rich_console__(self, console, options):
        yield from self._now().__rich_console__(console, options)


def header(title, origin, gauges=None):
    """A view's band: its name, the port, the view's `gauges`, the STO chain's chip and the
    meaning chip right."""
    where = ("PORT: %s" % origin.port if origin.real
             else "" if origin.label == "Simulated" else origin.label)
    tag = chip(origin)
    sto = sto_chip()
    return band_of(title, where, Text.assemble(sto, ' ', tag) if sto is not None else tag,
                   gauges)


#: Cells the title band is set in from the left edge: at 0 it stood out
#: left of every box under it, at 2 (the frames' title column) too far
#: right.
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


def footer(pairs, width=0):
    """The key bar: KEY: WHAT pairs on a reversed strip, terminal style - broken between pairs
    onto a second row where `width` cells hold them not on one."""
    rows = [Text('  ', style='keys')]
    for i, (key, what) in enumerate(pairs):
        pair = Text(style='keys')
        if key:
            pair.append(key, style='keys.key')
            pair.append(': ', style='keys')
        if isinstance(what, Text):
            pair.append_text(what)
        else:
            pair.append(what, style='keys')
        if i and width and len(rows) < 2 and rows[-1].cell_len + 5 + pair.cell_len > width:
            rows.append(Text('  ', style='keys'))
        elif i:
            rows[-1].append('  |  ', style='keys')
        rows[-1].append_text(pair)
    bar = Table.grid(expand=True)
    bar.add_column(justify='left')
    for row in rows:
        bar.add_row(row)
    bar.style = 'keys'
    return bar


#: The key help: a page's keys by group on a panel that slides up from the bar on TAB or ? over
#: SLIDE_S and down again HELP_S after the last key, by itself - the old taskbar (the user,
#: 2026-10-04).
HELP_KEYS, SLIDE_S, HELP_S = ('\t', '?'), 0.25, 8.0


def helped(state, typed, now):
    """How much of the help is up, 0 to 1: `state['help']` [to, up, last key at, last frame at]
    moved by `typed` this frame and the clock `now` (HELP_KEYS, SLIDE_S, HELP_S)."""
    to, up, keyed, framed = state.setdefault('help', [0.0, 0.0, now, now])
    if typed:
        keyed = now
        if any(key in HELP_KEYS for key in typed):
            to = 0.0 if to else 1.0
    if to and now - keyed > HELP_S:
        to = 0.0
    step = (now - framed) / SLIDE_S
    up = max(0.0, min(1.0, up + max(-step, min(step, to - up))))
    state['help'] = [to, up, keyed, now]
    return up


def help_rows(groups, up):
    """How many rows of the help are up: `up` of a title row and its longest group's."""
    return round(up * (1 + max(len(pairs) for _title, pairs in groups)))


def help_panel(groups, up, width=150):
    """(the help's top `up` of its rows on the bar's strip, how many rows): `groups` ((title,
    ((key, what), ..)), ..) in equal columns across `width`, a title over each, nothing
    wrapped - None and 0 with nothing up."""
    rows = help_rows(groups, up)
    if rows <= 0:
        return None, 0
    column = max(12, (width - 2) // len(groups))
    columns = [column] * (len(groups) - 1) + [max(column, width - 2 - column * (len(groups) - 1))]
    grid = Table.grid(padding=0)
    grid.add_column(width=2)
    for wide in columns:
        grid.add_column(width=wide)
    for r in range(rows):
        cells = [Text('  ', style='keys')]
        for (title, pairs), wide in zip(groups, columns):
            keys = max(len(key) for key, _what in pairs) + 1
            cell = Text(style='keys', no_wrap=True, overflow='ellipsis')
            if r == 0:
                cell.append(title, style='keys.key')
            elif r - 1 < len(pairs):
                key, what = pairs[r - 1]
                cell.append(key.ljust(keys), style='keys.key')
                cell.append(what, style='keys')
            cell.pad_right(wide - cell.cell_len)
            cells.append(cell)
        grid.add_row(*cells)
    grid.style = 'keys'
    return grid, rows


def hud(title, rows):
    """One instrument: labels recede, values glow, rounded frame. Lines of one Text, not a
    grid: seven grids measured their columns every frame, a third of the rotor page's
    (2026-09-27)."""
    width = max((len(str(row[0])) for row in rows if isinstance(row, tuple)), default=0)
    body = Text(no_wrap=True, overflow='ellipsis')
    for i, row in enumerate(rows):
        if i:
            body.append('\n')
        if isinstance(row, tuple):
            body.append('%*s ' % (width, row[0]), style='label')
            value = row[1]
        else:
            body.append(' ' * (width + 1))
            value = row
        if not isinstance(value, Text):
            value = Text.from_ansi(str(value)) if isinstance(row, str) else Text(str(value))
        value = value.copy()
        value.style = value.style or 'value'
        body.append_text(value)
    return Panel(body, title=Text(title, style='name'), title_align='left',
                 box=box.ROUNDED, border_style='frame.hud',
                 padding=(0, 1), expand=True)


_ESCAPE = re.compile(r'\x1b')


def viewport(title, art, corner='', page=None):
    """The drawing, centred in a heavy frame that owns its region; `corner`
    (a `Rate.label()`) over its top-left cells; `page` its title, the house HUD
    over it (Chrome), None for a drawing that carries its own."""
    drawing = Align(Marquee(art), align='center', vertical='middle')
    return Panel(Corner(drawing if page is None else Chrome(drawing, page), corner),
                 title=Text(' %s ' % title, style='name'),
                 title_align='left', box=box.HEAVY, border_style='frame',
                 padding=(0, 1), expand=True)


def frame_of(console, origin, title, art, boxes, keys, art_title=None,
             under=None, dressed=True, gauges=None, wrap=False, help=None):
    """The template: title band (its `gauges` a Text after the name), viewport left,
    instruments right, key bar - on two rows `wrap`ped, the key `help` (groups, up) over it
    (`help_panel`); `dressed` False for a drawing with a HUD of its own."""
    panel, rows = help_panel(*help, width=console.width) if help else (None, 0)
    if not _fills(console):
        return Group(header(title, origin, gauges),
                     viewport(art_title or title, art),
                     *([under] if under is not None else []),
                     *boxes, *([panel] if panel is not None else []), footer(keys))

    # The column is paged here, for every view at once; the key bar says so
    # only while there is something to scroll to.
    boxes = paged(console, boxes)
    rate = rate_of(console).label()
    at, seen, total = scroll_state(console)['pages']
    if at or seen < total:
        keys = list(keys) + [(UP + ' ' + DOWN, 'SCROLL')]
    whole = _laid(console, under is not None)
    page = title if dressed else None
    drawn = viewport(art_title or title, art, rate, page)
    if under is None:
        whole['art'].update(drawn)
    else:
        # `under` is a fixed height because the viewport takes the rest: a box
        # that grew with its content would push the bars off the bottom of a
        # short terminal instead of the other way round.
        whole['top'].update(drawn)
        whole['under'].update(under)
        whole['under'].size = _rows_of(under) + 2
    whole['hud'].update(Group(*boxes) if boxes else Text(''))
    whole['header'].update(header(title, origin, gauges))
    bar = footer(keys, console.width if wrap else 0)
    whole['footer'].update(Group(panel, bar) if panel is not None else bar)
    whole['footer'].size = bar.row_count + rows
    return whole


#: Each console's layout, laid once and filled every frame: a rich Layout keys its render map by
#: itself, so a tree laid a frame left every frame's drawing in a cycle - 117 066 objects over
#: 40 frames of the thermal page, its peak up 34 kB a frame on 3.14's collector (2026-09-30).
_LAID: weakref.WeakKeyDictionary = weakref.WeakKeyDictionary()


def _laid(console, under):
    """The console's layout, `under` the drawing or not, laid on first ask."""
    have = _LAID.get(console)
    if have is None or have[0] != under:
        art = Layout(name='art')
        if under:
            art.split_column(Layout(name='top'), Layout(name='under'))
        body = Layout(name='body')
        body.split_row(art, Layout(name='hud', size=HUD_WIDTH))
        whole = Layout()
        whole.split_column(Layout(name='header', size=1), body, Layout(name='footer', size=1))
        have = _LAID[console] = (under, whole)
    return have[1]


def panels_of(console, origin, title, groups, keys):
    """The template for table views: a fixed grid of instruments, no art."""
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
    # viewport, so a table page owns its region the way they do; without it
    # the session read as loose boxes on the bare screen.
    framed = Panel(Corner(Chrome(body, title, lock=False), rate_of(console).label()),
                   title=Text(' %s ' % title, style='name'),
                   title_align='left', box=box.HEAVY, border_style='frame',
                   padding=0, expand=True)

    whole = Layout()
    whole.split_column(Layout(header(title, origin), size=1),
                       Layout(framed, name='grid'),
                       Layout(footer(keys), size=1))
    return whole
