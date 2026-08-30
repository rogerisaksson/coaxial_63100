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
        TextColumn('[label]{task.description}'),
        BarColumn(bar_width=28, complete_style='value',
                  finished_style='value', style='frame.hud'),
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


def chip(origin):
    """The meaning tag. Green LIVE / yellow SIMULATED, never restyled."""
    if origin.real:
        from coaxial import broker
        count = broker.clients() or 0
        label = (' LIVE  %d session%s ' % (count, '' if count == 1 else 's')
                 if count else ' LIVE ')
        return Text(label, style='chip.live')
    return Text(' SIMULATED ', style='chip.sim')


def header(title, origin, extra=''):
    """The title band: the meaning chip HARD LEFT, then the name.

    The chip was right-justified once, and its cell's padding picked up
    the band's background - on a wide terminal that read as a green field
    across half the row. Leftmost, it is a chip and nothing else.
    """
    bar = Table.grid(expand=True, padding=0)
    bar.add_column(justify='left')
    line = Text.assemble(chip(origin), ('  ' + title, 'bar'),
                         ('   ' + extra if extra else '', 'bar.dim'))
    bar.add_row(line)
    bar.style = 'bar.dim'
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


def viewport(title, art):
    """The drawing, centred in a heavy frame that owns its region."""
    return Panel(Align(Text.from_ansi(art), align='center',
                       vertical='middle'),
                 title=Text(' %s ' % title, style='name'),
                 title_align='left', box=box.HEAVY, border_style='frame',
                 padding=(0, 1), expand=True)


def _fills(console):
    """Whether to build the full-screen layout: only on a live terminal.
    Piped - the tests, a log - gets the same parts stacked plainly.
    Accepts the Console or a view's plain bool for it."""
    return bool(getattr(console, 'is_terminal', console))


def frame_of(console, origin, title, art, boxes, keys, extra=''):
    """THE template: title band, viewport left, instruments right, key bar.

    One function, so the views cannot drift apart. On a terminal it fills
    the screen the way the reference panels do; piped it degrades to the
    same content in reading order.
    """
    if not _fills(console):
        return Group(header(title, origin, extra),
                     viewport(title, art), *boxes, footer(keys))

    hud_width = 36
    body = Layout()
    body.split_row(Layout(viewport(title, art), name='art'),
                   Layout(Group(*boxes) if boxes else Text(''),
                          name='hud', size=hud_width))

    whole = Layout()
    whole.split_column(Layout(header(title, origin, extra), size=1),
                       Layout(body, name='body'),
                       Layout(footer(keys), size=1))
    return whole


def panels_of(console, origin, title, groups, keys, extra=''):
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
        return Group(header(title, origin, extra), *flat, footer(keys))

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

    whole = Layout()
    whole.split_column(Layout(header(title, origin, extra), size=1),
                       Layout(body, name='grid'),
                       Layout(footer(keys), size=1))
    return whole
