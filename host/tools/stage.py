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

from rich import box
from rich.align import Align
from rich.columns import Columns
from rich.console import Console, Group
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich.theme import Theme

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
    """Enable VT processing on the Windows stdout console BEFORE rich
    looks at it. Without the flag rich sees a legacy console, routes
    every frame through the 16-colour Win32 API and no escape we write
    reaches the screen - measured: a bare conhost detects `windows`,
    legacy True. Nothing to do on a pipe or another platform."""
    try:
        import ctypes
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
        # rich guesses the colour depth from the environment, and on a
        # Windows console with no COLORTERM it guesses 256 or 16 - the
        # glow ramp's 24-bit gradient was requantised to the palette's
        # five cyans, a hard iso-line across the board where 99 luma
        # met 106. Every console this runs in renders 24-bit.
        console = Console(highlight=False, theme=THEME,
                          color_system='truecolor')
    return console


@contextmanager
def curtain(console):
    """The Live a view runs inside: the alternate screen on a terminal, so
    the shell underneath is untouched and comes back on exit. Piped, each
    update prints once, plainly, in order.

    The buffer is CLEARED on entry: some terminals keep the alternate
    screen's previous content, and the last session's page flashed for a
    frame before the first update painted over it."""
    live = Live(console=console, screen=console.is_terminal,
                auto_refresh=False, transient=console.is_terminal)
    with live:
        if console.is_terminal:
            console.clear()
        yield live


@contextmanager
def boot(label, console=None):
    """A brisk amber progress strip over whatever the block actually does.

    Deliberately a little gratuitous - the reference terminals never just
    open a page, they SPIN SOMETHING UP. The bar rides the real work: the
    block reports its milestones through the yielded `step(share,
    label)`, the bar creeps a little past each on its own clock while
    the next one runs, and it snaps full when the block returns. With
    no milestones reported it creeps to 90 % as before.

    `step()` doubles as `ready()`: called with no arguments it snaps
    full and takes the bar down NOW, for a view whose body keeps
    running long after the link is up.
    """
    import threading
    import time as _time

    from rich.progress import BarColumn, Progress, TextColumn

    court = console or stage()
    if not court.is_terminal:
        yield lambda *args: None
        return

    bar = Progress(
        BarColumn(bar_width=28, complete_style='value',
                  finished_style='value', style='frame.hud'),
        # The text AFTER the bar, bracketed. Text first, each milestone
        # shifted the bar by the difference in length and it flickered.
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
            _time.sleep(0.03)

    def step(share=None, text=None):
        if share is None:
            if closed:
                return
            closed.append(True)
            stop.set()
            walker.join(timeout=0.5)
            bar.update(task, completed=100)
            _time.sleep(0.06)
            bar.stop()
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
        from coaxial import broker
        return live(broker.clients() or 0)
    return Text(' SIMULATED ', style='chip.sim')


def band_of(name, extra='', tag=None):
    """The band every page wears: `name` hard left, `extra` dim after it,
    `tag` right with one cell of air before the band's end.

    The tag sits in a column styled as the band, so its padding is the
    band's: right-justified in a column of its own style it once painted
    a green field across half a wide row.
    """
    left = Text.assemble((name, 'bar'),
                         ('   ' + extra if extra else '', 'bar.dim'))
    right = Text.assemble(tag, (' ', 'bar.dim')) if tag else Text('')
    return band(left, right)


def header(title, origin):
    """A view's band: its name, the port, the meaning chip right. A
    stand-in that was fallen back to says so here - the port that failed
    to answer is the diagnosis."""
    where = ("PORT: %s" % origin.port if origin.real
             else "" if origin.label == "Simulated" else origin.label)
    return band_of(title, where, chip(origin))


#: Cells the title band is set in from the left edge. Painted from column
#: 0 it stood out left of every box under it; two cells in - the frames'
#: title column - read as too far right. One.
BAND_INSET = 1


def band(*cells):
    """The title band: `cells` on the band's colour, an inset unpainted
    at both ends. One cell paints the row; two - a left and a right -
    split it, the right one as wide as its text."""
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
    """The key bar: KEY: WHAT pairs on a reversed strip, terminal style.

    `what` may be a ready Text - a state that should POP (an inverted
    axis, an armed stage) rides the bar in its own style.
    """
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
    """One instrument: labels recede, values glow, rounded frame.

    `rows` are (label, value) pairs, ready Text objects, or ANSI strings.
    """
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
    """Where the window sits over art `extra` cells too wide: a ping-pong
    on the wall clock, held at each end. Wall time rather than a frame
    count, so every view slides at one pace whatever it redraws at."""
    import time

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


class Marquee:

    """Art cropped to its frame instead of wrapped, slid back and forth
    when it is wider: the bars shorten, and the hidden end comes past on
    the slide. Content is single-cell glyphs, so character index and
    display cell agree."""

    def __init__(self, art):
        self.lines = [Text.from_ansi(line) for line in art.split('\n')]
        for line in self.lines:
            line.no_wrap, line.overflow = True, 'crop'
        self.wide = max((l.cell_len for l in self.lines), default=0)

    def __rich_measure__(self, _console, options):
        from rich.measure import Measurement
        return Measurement(min(self.wide, options.max_width), self.wide)

    def __rich_console__(self, _console, options):
        width = options.max_width
        extra = self.wide - width
        at = _slide(extra) if extra > 0 else 0
        for line in self.lines:
            if at or line.cell_len > width:
                line = line[at:at + width]
                line.no_wrap, line.overflow = True, 'crop'
            yield line


def viewport(title, art):
    """The drawing, centred in a heavy frame that owns its region. Too
    wide for the frame, it crops and slides rather than wraps.

    `title` is the REGION's name, which is not always the page's: a page
    showing two kinds of thing names the region for what is in it.
    """
    return Panel(Align(Marquee(art), align='center',
                       vertical='middle'),
                 title=Text(' %s ' % title, style='name'),
                 title_align='left', box=box.HEAVY, border_style='frame',
                 padding=(0, 1), expand=True)


def _rows_of(panel):
    """Content lines in a hud, for sizing its Layout. The renderable is a
    grid, so its row count is what the frame has to make room for."""
    inner = getattr(panel, 'renderable', None)
    return len(getattr(inner, 'rows', []) or [1])


def _fills(console):
    """Whether to build the full-screen layout: only on a live terminal.
    Piped - the tests, a log - gets the same parts stacked plainly.
    Accepts the Console or a view's plain bool for it."""
    return bool(getattr(console, 'is_terminal', console))


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


def scroll_state(console):
    """The instrument column's scroll, kept ON THE CONSOLE so every view
    that draws through `frame_of` has one without holding it: `at` is
    the first box shown, `pages` is `(at, seen, total)` after the last
    frame, `haul` the drag's remainder and `grip` whether a drag began
    over the column."""
    state = getattr(console, 'coaxial_scroll', None)
    if state is None:
        state = {'at': 0, 'pages': (0, 0, 0), 'haul': 0.0, 'grip': False}
        try:
            setattr(console, 'coaxial_scroll', state)
        except AttributeError:
            pass
    return state


def _height_of(box):
    """Rows a box takes in the column: a hud's grid plus its frame, a
    bare line one."""
    if isinstance(box, Panel):
        return _rows_of(box) + BOX_BORDER
    return 1


def paged(console, boxes):
    """The instrument column, windowed, with an arrow where it continues.

    SEVEN BOXES DO NOT FIT. The column is the page's right-hand forty
    cells and the boxes fill it from the top; past the bottom of the
    terminal they are simply not drawn, and a reader has no way to know
    a THERMAL box exists at all. This shows as many as the terminal has
    room for and says which way the rest are, on a row that can be
    clicked to get there - and it was the rotor observer's alone until
    the bench asked for the arrows on every page, which is why it lives
    in the template now.

    Piped, nothing is windowed: a captured page is read in order and has
    no bottom to fall off.
    """
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
    # The last page is packed from the END, so scrolling to the bottom
    # shows a full column rather than one box and a lot of air.
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
    # FROM WHERE IT IS, not from where the last frame drew: two arrows
    # between frames used to land one box down, the second reading the
    # first frame's position again. The bounds are the last frame's -
    # `paged` clamps whatever this asks for.
    at, seen, total = state['at'], state['pages'][1], state['pages'][2]
    if step > 0 and seen < total:
        state['at'] = at + 1
    elif step < 0 and at:
        state['at'] = at - 1


def scroll_click(console, column, row):
    """One click: the arrows at the top and bottom of the box column.

    The hit test is the template's own geometry rather than anything
    measured off the frame - the header is row one, the key bar the last
    row, and the boxes are the right-hand HUD_WIDTH cells of everything
    between. The arrows are the first and last rows of that, which is
    where `paged` draws them. The press is also where a drag begins:
    `grip` is remembered so `scroll_drag` can tell a page-drag from a
    drag across the drawing.
    """
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
    """A left-drag on the instrument column, dragged like a page.

    ONLY A DRAG THAT STARTED THERE - the same rule a scrollbar has. A
    whole box per DRAG_ROWS of travel, and the remainder is kept:
    rounding each frame's few cells to zero made a slow drag do nothing
    at all. Dragging DOWN brings the boxes above into view, which is
    which way paper moves under a hand.
    """
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
    """THE template: title band, viewport left, instruments right, key bar.

    One function, so the views cannot drift apart. On a terminal it fills
    the screen the way the reference panels do; piped it degrades to the
    same content in reading order.
    """
    if not _fills(console):
        return Group(header(title, origin),
                     viewport(art_title or title, art),
                     *([under] if under is not None else []),
                     *boxes, footer(keys))

    # THE COLUMN IS PAGED HERE, for every view at once, and the key bar
    # says so only while there is something to scroll to.
    boxes = paged(console, boxes)
    at, seen, total = scroll_state(console)['pages']
    if at or seen < total:
        keys = list(keys) + [(UP + ' ' + DOWN, 'SCROLL')]
    body = Layout()
    art_region = Layout(name='art')
    if under is None:
        art_region.update(viewport(art_title or title, art))
    else:
        # `under` is a fixed height because the viewport takes the rest:
        # a box that grew with its content would push the bars off the
        # bottom of a short terminal instead of the other way round.
        art_region.split_column(
            Layout(viewport(art_title or title, art), name='top'),
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
    """The template for table views: a FIXED grid of instruments, no art.

    `groups` is a list of rows, each row a list of huds. On a terminal
    every row is a Layout split into equal columns, so a box keeps its
    place and its width whatever its text does this frame - flowing
    Columns resized with the values, and the screen read as a salad that
    flickered. Piped, the same content stacks in reading order.
    """
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
    # viewport, so a table page owns its region the way they do - the
    # session read as loose boxes on the bare screen.
    framed = Panel(body, title=Text(' %s ' % title, style='name'),
                   title_align='left', box=box.HEAVY, border_style='frame',
                   padding=0, expand=True)

    whole = Layout()
    whole.split_column(Layout(header(title, origin), size=1),
                       Layout(framed, name='grid'),
                       Layout(footer(keys), size=1))
    return whole
