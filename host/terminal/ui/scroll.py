"""A panel taller than the screen: its scroll state, the page it shows, wheel and drag."""
import weakref

from rich import box
from rich.panel import Panel
from rich.table import Table
from rich.text import Text


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
