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


def stage():
    """The console every view draws on. Plain and sequential when piped."""
    return Console(highlight=False, theme=THEME)


def curtain(console):
    """The Live a view runs inside: the alternate screen on a terminal, so
    the shell underneath is untouched and comes back on exit. Piped, each
    update prints once, plainly, in order."""
    return Live(console=console, screen=console.is_terminal,
                auto_refresh=False, transient=console.is_terminal)


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
    """The solid title band: name left, origin chip right."""
    bar = Table.grid(expand=True, padding=(0, 1))
    bar.add_column(justify='left')
    bar.add_column(justify='right')
    left = Text.assemble(('  ' + title, 'bar'),
                         ('   ' + extra if extra else '', 'bar.dim'))
    bar.add_row(left, chip(origin))
    bar.style = 'bar.dim'
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
    """The template for table views: rows of instrument columns, no art.

    `groups` is a list of rows, each row a list of huds shown side by side.
    """
    rows = [Columns(row, padding=(0, 1), expand=False) for row in groups]
    if not _fills(console):
        return Group(header(title, origin, extra), *rows, footer(keys))

    whole = Layout()
    whole.split_column(Layout(header(title, origin, extra), size=1),
                       Layout(Group(*rows), name='body'),
                       Layout(footer(keys), size=1))
    return whole
