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
from contextlib import contextmanager

import ctypes
import re
import threading
import time
import weakref

from rich import box
from rich.align import Align
from rich.cells import cell_len
from rich.color import Color
from rich.columns import Columns
from rich.console import Console, Group
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.segment import Segment
from rich.style import Style
from rich.table import Table
from rich.text import Text
from rich.theme import Theme
from rich.progress import BarColumn, Progress, TextColumn
from rich.measure import Measurement
from coaxial.comm import broker

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


#: The newest frame's weight in the drawing's rate: about five frames
#: smoothed, so the figure reads rather than flickers.
RATE_WEIGHT = 0.2
_RATES = weakref.WeakKeyDictionary()


def _eased(was, now):
    return now if was is None else was + RATE_WEIGHT * (now - was)


class Rate:
    """Frames a second, start to start, and one frame's cost from the
    draw's start to the terminal's write: the drawing's top-left corner."""

    def __init__(self):
        self.began = self.fps = self.ms = None

    def tick(self, started):
        self.ms = _eased(self.ms, 1000.0 * (time.monotonic() - started))
        if self.began is not None and started > self.began:
            self.fps = _eased(self.fps, 1.0 / (started - self.began))
        self.began = started

    def label(self):
        if self.ms is None:
            return ''
        fps = '%.1f fps  ' % self.fps if self.fps is not None else ''
        return '%s%.0f ms' % (fps, self.ms)


def rate_of(console):
    """The frame rate drawn on this console, made on first ask."""
    if console not in _RATES:
        _RATES[console] = Rate()
    return _RATES[console]


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


#: The marquee's pace: cells per second of travel, and the hold at each
#: end so the reader gets a full look before it turns back.
SLIDE_CPS = 5.0
SLIDE_HOLD = 1.2


def _slide(extra):
    """Where the window sits over art `extra` cells too wide: a ping-pong on
    the wall clock, held at each end.
    """

    travel = extra / SLIDE_CPS
    cycle = 2.0 * (travel + SLIDE_HOLD)
    t = time.monotonic() % cycle
    if t < SLIDE_HOLD:
        return 0
    t -= SLIDE_HOLD
    if t < travel:
        return int(extra * t / travel)
    t -= travel
    if t < SLIDE_HOLD:
        return extra
    return int(extra * (1.0 - (t - SLIDE_HOLD) / travel))


#: One SGR sequence, its parameters captured; and any escape at all, for
#: the line that carries something else.
_SGR = re.compile(r'\x1b\[([0-9;]*)m')
_ESCAPE = re.compile(r'\x1b')
#: rich Styles by (foreground, background) Color, made once: a moving
#: attitude frame carries ~700 runs of ~230 colours, and building a
#: Style per run was a third of what decoding the art cost.
_STYLES = {}


def _sgr_colour(codes, j):
    """The colour at `codes[j]` as (rich Color or None for the default,
    background?, parameters taken); None when the parameter is not a
    colour or a reset - which is the line's cue to take the slow road."""
    c = codes[j]
    if c in (38, 48) and j + 4 < len(codes) and codes[j + 1] == 2:
        return Color.from_rgb(*codes[j + 2:j + 5]), c == 48, 5
    if c in (38, 48) and j + 2 < len(codes) and codes[j + 1] == 5:
        return Color.from_ansi(codes[j + 2]), c == 48, 3
    if 30 <= c <= 37 or 40 <= c <= 47:
        return Color.from_ansi(c % 10), c >= 40, 1
    if 90 <= c <= 97 or 100 <= c <= 107:
        return Color.from_ansi(c % 10 + 8), c >= 100, 1
    if c in (39, 49):
        return Color.default(), c == 49, 1
    return None


def _style_of(fg, bg):
    if fg is None and bg is None:
        return None
    got = _STYLES.get((fg, bg))
    if got is None:
        got = _STYLES[(fg, bg)] = Style(color=fg, bgcolor=bg)
    return got


#: What an SGR parameter string does to (foreground, background), decoded
#: once per distinct string: the art repeats a few hundred of them
#: thousands of times a frame, and parsing each into ints and a Color per
#: run was three times the cost of the split itself (measured, the first
#: version of this decoder against the prototype: 7.3 ms against 2.5).
#: KEEP leaves a channel as it was; None sets it to the default.
KEEP = object()
_SGR_DONE = {}
#: A list of these that would stop being a cache is a session's worth of
#: art gone wrong; the cap only bounds it.
SGR_KEPT = 4096


def _sgr_effect(params):
    """(foreground, background) after `params` - each a Color, None for the
    default, or KEEP - or False when the string holds a parameter this
    does not read."""
    fg = bg = KEEP
    codes = [int(c) for c in params.split(';') if c] or [0]
    j = 0
    while j < len(codes):
        if codes[j] == 0:
            fg, bg, took = None, None, 1
        else:
            got = _sgr_colour(codes, j)
            if got is None:
                return False
            colour, background, took = got
            if background:
                bg = colour
            else:
                fg = colour
        j += took
    return fg, bg


def _runs_of(line):
    """The line as [(text, Style or None)] runs, decoded here; None for a
    line carrying an escape this does not read (a bold, a cursor move),
    which Text.from_ansi then decodes whole."""
    pieces = _SGR.split(line)
    runs, fg, bg = [], None, None
    style = None
    for i in range(0, len(pieces), 2):
        text = pieces[i]
        if text:
            if '\x1b' in text:
                return None
            runs.append((text, style))
        if i + 1 == len(pieces):
            break
        params = pieces[i + 1]
        effect = _SGR_DONE.get(params)
        if effect is None:
            if len(_SGR_DONE) >= SGR_KEPT:
                _SGR_DONE.clear()
            effect = _SGR_DONE[params] = _sgr_effect(params)
        if effect is False:
            return None
        new_fg, new_bg = effect
        if new_fg is not KEEP:
            fg = new_fg
        if new_bg is not KEEP:
            bg = new_bg
        style = _style_of(fg, bg)
    return runs


def _cropped(segments, at, width):
    """`segments` sliced to cells [at, at + width), runs re-joined: the
    crop the Text slice made, on Segments."""
    cells = [(ch, s.style) for s in segments for ch in s.text][at:at + width]
    out, style, buf = [], None, []
    for ch, st in cells:
        if buf and st is not style:
            out.append(Segment(''.join(buf), style))
            buf = []
        style = st
        buf.append(ch)
    if buf:
        out.append(Segment(''.join(buf), style))
    return out


class Marquee:

    """Art cropped to its frame instead of wrapped, slid back and forth when
    it is wider: the bars shorten, and the hidden end comes past on the
    slide.
    """

    def __init__(self, art):
        self.lines, self.widths = [], []
        for raw in art.split('\n'):
            runs = _runs_of(raw)
            if runs is None:
                line = Text.from_ansi(raw)
                line.no_wrap, line.overflow = True, 'crop'
                self.lines.append(line)
                self.widths.append(line.cell_len)
            else:
                self.lines.append([Segment(text, style) for text, style in runs])
                self.widths.append(cell_len(''.join(t for t, _ in runs)))
        self.wide = max(self.widths, default=0)

    def __rich_measure__(self, console, options):
        return Measurement(min(self.wide, options.max_width), self.wide)

    def __rich_console__(self, console, options):
        width = options.max_width
        extra = self.wide - width
        at = _slide(extra) if extra > 0 else 0
        newline = Segment.line()
        for line, wide in zip(self.lines, self.widths):
            if isinstance(line, Text):
                if at or wide > width:
                    line = line[at:at + width]
                    line.no_wrap, line.overflow = True, 'crop'
                yield line
                continue
            if at or wide > width:
                line = _cropped(line, at, width)
            yield from line
            yield newline


class Corner:
    """`inner` with `label` painted over its top-left cells: the frame rate
    in the drawing's own corner, no box of its own."""

    def __init__(self, inner, label):
        self.inner, self.label = inner, label

    def __rich_measure__(self, console, options):
        return Measurement.get(console, options, self.inner)

    def __rich_console__(self, console, options):
        lines = console.render_lines(self.inner, options, pad=True)
        wide = cell_len(self.label)
        if lines and 0 < wide < options.max_width:
            cut = list(Segment.divide(lines[0], [wide, options.max_width]))
            lines[0] = (list(Text(self.label, style='label').render(console))
                        + (cut[1] if len(cut) > 1 else []))
        newline = Segment.line()
        for line in lines:
            yield from line
            yield newline


def viewport(title, art, corner=''):
    """The drawing, centred in a heavy frame that owns its region; `corner`
    (a `Rate.label()`) over its top-left cells."""
    return Panel(Corner(Align(Marquee(art), align='center',
                              vertical='middle'), corner),
                 title=Text(' %s ' % title, style='name'),
                 title_align='left', box=box.HEAVY, border_style='frame',
                 padding=(0, 1), expand=True)


def _rows_of(panel):
    """Content lines in a hud, for sizing its Layout."""
    inner = panel.renderable
    return len(inner.rows) if isinstance(inner, Table) and inner.rows else 1


def _fills(console):
    """Whether to build the full-screen layout: only on a live terminal."""
    if isinstance(console, bool):
        raise TypeError('the stage draws on the console, not on its '
                        'is_terminal flag')
    return console.is_terminal


#: The instrument column's width. 40 since 2026-08-30: the thermal LEVELS
#: rows wanted air, and the column is the template's, so every view moves
#: together.
HUD_WIDTH = 40

#: Rows a hud's frame adds round its content.
BOX_BORDER = 2

#: The scroll affordances. Triangles rather than dots: they are not part
#: of the picture, they are something to click.
UP, DOWN = chr(0x25B4), chr(0x25BE)

#: Cells of drag per box scrolled. About a box's own height, so the
#: column moves at the hand's speed rather than flying.
DRAG_ROWS = 6.0


#: The instrument column's scroll PER CONSOLE, so every view that draws
#: through `frame_of` has one without holding it - and gone with the
#: console, which is what the weak keys are for.
_SCROLLS = weakref.WeakKeyDictionary()


def _fresh_scroll():
    """`at` is the first box shown, `pages` is `(at, seen, total)` after
    the last frame, `haul` the drag's remainder and `grip` whether a
    drag began over the column."""
    return {'at': 0, 'pages': (0, 0, 0), 'haul': 0.0, 'grip': False}


def scroll_state(console):
    """The instrument column's scroll on this console, made on first ask."""
    if console not in _SCROLLS:
        _SCROLLS[console] = _fresh_scroll()
    return _SCROLLS[console]


def _height_of(box):
    """Rows a box takes in the column: a hud's grid plus its frame, a
    bare line one."""
    if isinstance(box, Panel):
        return _rows_of(box) + BOX_BORDER
    return 1


def paged(console, boxes):
    """The instrument column, windowed, with an arrow where it continues."""
    state = scroll_state(console)
    boxes = list(boxes or ())
    try:
        room = console.size.height - 2 if _fills(console) else 0
    except (AttributeError, OSError):
        room = 0
    if room <= 0 or not boxes:
        state['pages'] = (0, len(boxes), len(boxes))
        return boxes

    heights = [_height_of(box) for box in boxes]
    # The last page is packed from the END, so scrolling to the bottom shows a
    # full column rather than one box and a lot of air.
    last, used = len(boxes), 0
    while last > 0 and used + heights[last - 1] + 1 <= room:
        used += heights[last - 1]
        last -= 1
    state['at'] = max(0, min(state['at'], last))

    at = state['at']
    out, taken = [], 1 if at else 0              # a row for the up arrow
    while at < len(boxes) and taken + heights[at] <= room - 1:
        out.append(boxes[at])
        taken += heights[at]
        at += 1
    state['pages'] = (state['at'], at, len(boxes))
    if state['at']:
        out.insert(0, Text(' %s  %s above' % (UP, state['at']),
                           style='keys'))
    if at < len(boxes):
        out.append(Text(' %s  %d more' % (DOWN, len(boxes) - at),
                        style='keys'))
    return out


def scroll_by(console, step):
    """One box up (negative) or down, within what the last frame said
    there was."""
    state = scroll_state(console)
    # FROM WHERE IT IS, not from where the last frame drew: two arrows between
    # frames used to land one box down, the second reading the first frame's
    # position again.
    at, seen, total = state['at'], state['pages'][1], state['pages'][2]
    if step > 0 and seen < total:
        state['at'] = at + 1
    elif step < 0 and at:
        state['at'] = at - 1


def scroll_click(console, column, row):
    """One click: the arrows at the top and bottom of the box column."""
    state = scroll_state(console)
    try:
        width, height = console.size.width, console.size.height
    except (AttributeError, OSError):
        return
    if not _fills(console) or not width or not height:
        return
    state['grip'] = column > width - HUD_WIDTH
    if not state['grip']:
        return
    at, seen, total = state['at'], state['pages'][1], state['pages'][2]
    if row == 2 and at:
        state['at'] = at - 1
    elif row == height - 1 and seen < total:
        state['at'] = at + 1


def scroll_drag(console, dy):
    """A left-drag on the instrument column, dragged like a page."""
    state = scroll_state(console)
    if not state['grip']:
        return
    state['haul'] += dy
    while abs(state['haul']) >= DRAG_ROWS:
        step = 1 if state['haul'] < 0 else -1
        state['haul'] -= step * -DRAG_ROWS
        at, seen, total = state['at'], state['pages'][1], state['pages'][2]
        if step > 0 and seen < total:
            state['at'] = at + 1
        elif step < 0 and at:
            state['at'] = at - 1
        else:
            state['haul'] = 0.0
            break


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
