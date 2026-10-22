"""The terminal's own round every page: its key bar and help, the column's rail.

    footer(pairs)          # the page's pairs from the left, the terminal's at the bar's right end
    bar_of(pad, now)       # (its bar's pairs, its help)
    rail()                 # in the column's place, folded (`column`): its edge, a tab in its middle
    room(width)            # the drawing's columns, `beside()` - the column as it stands, the frame - off

A page's keys are its pad's (`terminal.ui.bindings`); `terminal.ui.stage` lays these round its
drawing.
"""
import math
import time

from rich.color import Color
from rich.style import Style
from rich.table import Table
from rich.text import Text

from terminal.ui import bindings
from terminal.ui.chrome import SWEEP_S
from terminal.ui.scroll import HUD_WIDTH


#: The terminal's own pairs on every bar (`bindings.SHARED`): at its right end, never cut.
TERMINAL_PAIRS = frozenset(pair for row in bindings.SHARED for pair in row[3])


def _paired(key, what):
    """KEY: WHAT on the bar's strip."""
    pair = Text(style='keys')
    if key:
        pair.append(key, style='keys.key')
        pair.append(': ', style='keys')
    if isinstance(what, Text):
        pair.append_text(what)
    else:
        pair.append(what, style='keys')
    return pair


def footer(pairs, width=0):
    """The key bar: KEY: WHAT pairs on a reversed strip, terminal style - the page's from the left,
    broken between pairs onto a second row where `width` cells hold them not on one, cut where
    they meet the terminal's own (TERMINAL_PAIRS) at its right end."""
    theirs = [p for p in pairs if isinstance(p[1], str) and p in TERMINAL_PAIRS]
    right = Text('   ', style='keys') + Text('  |  ', style='keys').join(_paired(*p) for p in theirs)
    right.append('  ', style='keys')
    room = max(1, width - right.cell_len) if width else 0
    rows = [Text('  ', style='keys')]
    for i, pair in enumerate(_paired(*p) for p in pairs if p not in theirs):
        if i and room and len(rows) < 2 and rows[-1].cell_len + 5 + pair.cell_len > room:
            rows.append(Text('  ', style='keys'))
        elif i:
            rows[-1].append('  |  ', style='keys')
        rows[-1].append_text(pair)
    bar = Table.grid(expand=True)
    bar.add_column(justify='left', no_wrap=True, overflow='ellipsis', ratio=1)
    if theirs:
        bar.add_column(justify='right', no_wrap=True, width=right.cell_len)
    for k, row in enumerate(rows):
        bar.add_row(row, *([right if k == 0 else Text('')] if theirs else []))
    bar.style = 'keys'
    return bar


#: The key help: a page's keys by group on a panel that slides up from the bar on ? over SLIDE_S
#: and down again HELP_S after the last key, by itself - the old taskbar.
SLIDE_S, HELP_S = 0.25, 8.0


def helped(now):
    """How much of the help is up, 0 to 1, at the clock `now`: slid toward ? (`bindings.UX`)
    over SLIDE_S, let down HELP_S after the last key."""
    ux = bindings.UX
    if ux['help'] and now - ux['keyed'] > HELP_S:
        ux['help'] = False
    step = (now - ux['framed']) / SLIDE_S
    ux['up'] = max(0.0, min(1.0, ux['up'] + max(-step, min(step, float(ux['help']) - ux['up']))))
    ux['framed'] = now
    return ux['up']


def bar_of(keys, now):
    """(the bar's pairs, the help (groups, up) or None) of a page's `keys` - its pad
    (`bindings.page`), or a bar's pairs alone."""
    if isinstance(keys, dict):
        return keys['bar'], (keys['groups'], helped(now))
    return keys, None


#: The instruments' column folded (Tab): a rail RAIL cells wide on the right edge.
RAIL = 3
#: The rail's arrow breathes with the screen's sweep (`chrome.SWEEP_S`): its ink from a key's
#: down BREATH of the way to an instrument edge's and back - faint, slow.
BREATH = 0.5


def column():
    """The instruments' column's width, cells: its rail's folded (Tab, `bindings.UX`)."""
    return HUD_WIDTH if bindings.UX['column'] else RAIL


#: `stage.viewport`'s frame and padding round the drawing, columns.
VIEWPORT_COLUMNS = 4


def beside():
    """The columns beside the drawing: the instruments', folded or out (`column`), and the
    viewport's frame."""
    return column() + VIEWPORT_COLUMNS


def room(width):
    """The drawing's columns in a window `width` across: what `beside` leaves it."""
    return width - beside()


def _rgb(style):
    """`style`'s ink as (r, g, b)."""
    return tuple((style.color or Color.default()).get_truecolor())


def breath(console, now):
    """The rail's arrow's style at `now`: a key's ink down BREATH of the way to an instrument
    edge's and back over the sweep's SWEEP_S."""
    key = console.get_style('keys.key')
    bright, dim = _rgb(key), _rgb(console.get_style('frame.hud'))
    k = BREATH * (1.0 - math.cos(2.0 * math.pi * now / SWEEP_S)) / 2.0
    return key + Style(color=Color.from_rgb(*(round(b + k * (d - b)) for b, d in zip(bright, dim))))


#: The folded column's tab, a row each: its top, its arrow, the key that brings the column out
#: (Tab, its keycap's ↹), its bottom - on its edge's line.
TAB = ('╭─╮', '│◂│', '│↹│', '╰─╯')


class _Rail:
    """The instruments' column folded: its edge a line down the page, a tab in the middle of it,
    its arrow breathing (`breath`) over its key - read when it is drawn, so a frame shown again
    between draws breathes on."""

    def __rich_console__(self, console, options):
        rows = options.height or len(TAB)
        top = (rows - len(TAB)) // 2
        inks = {'◂': breath(console, time.monotonic()), '↹': 'keys.key'}
        rail = Text(no_wrap=True)
        for row in range(rows):
            at = row - top
            line = TAB[at] if 0 <= at < len(TAB) else ' │ '
            rail.append('\n' if row else '')
            for cell in line:
                rail.append(cell, style=inks.get(cell, 'frame.hud'))
        yield rail


_RAIL = _Rail()


def rail():
    """The instruments' column folded: its edge, a tab in its middle, its arrow breathing."""
    return _RAIL


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
