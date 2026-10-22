"""Balloons on a drawing: a thing's key and word framed over it, a tail down to it.

    cells = balloons.laid([((row, col), 'F1', 'sit')], width, height, lit=False, shut=docked)

A balloon is three rows - the instruments' frame's top edge, ' key word ', its bottom edge -
centred over its thing's cell; nearest first, one meeting a balloon laid before it rises over it,
its tail longer and behind the balloons it passes. A thing off the drawing or behind the cells
`shut` (a picture docked on it) has its balloon at the cell nearest it on its row, its tip
pointing on to it.
"""
from coaxial.graphics.callouts import FRAME_CORNERS, FRAME_INK, packed
from coaxial.graphics.inset import NOTE_INK, PLATE

#: A balloon's word's ink (the instruments' label); its tail, the notch it leaves the frame by,
#: its tip on the thing, and by the side the thing is on where it is out of sight.
WORD_INK, TAIL, NOTCH, TIP, TIPS = (214, 220, 228), '│', '┬', '▾', {-1: '◂', 1: '▸'}


def _seen_at(row, col, width, height, shut):
    """(row, col, tip): the cell on the drawing and out of `shut` nearest (`row`, `col`) along its
    row - a docked picture stands at the drawing's right -, its tip pointing on to the thing."""
    row, at = min(row, height - 1), max(0, min(width - 1, col))
    while (row, at) in shut and at > 0:
        at -= 1
    return row, at, TIPS.get((col > at) - (col < at), TIP)


def _risen(bottom, left, right, taken):
    """The lowest bottom edge at `bottom` or over it whose balloon meets none of `taken`
    [(bottom, left, right, ..)]."""
    met = [low - 2 for low, a, b, *_rest in taken if low - 2 <= bottom and bottom - 2 <= low and a <= right and left <= b]
    return _risen(min(met) - 1, left, right, taken) if met else bottom


def _placed(tags, width, height, shut):
    """[(bottom, left, right, (row, col), text, key, tip)]: where each of `tags` stands - risen
    over those before it, left of `shut` -, those with room."""
    taken = []
    for (row, col), key, word in tags:
        (row, col, tip), text = _seen_at(row, col, width, height, shut), ' %s %s ' % (key, word)
        wide = len(text) + 2
        left = max(0, min(width - wide, col - wide // 2))
        bottom = _risen(row - 1, left, left + wide - 1, taken)
        edge = min([c for r in range(bottom - 2, bottom + 1) for c in range(left, left + wide) if (r, c) in shut],
                   default=width)
        left = min(left, edge - wide)
        bottom = _risen(row - 1, left, left + wide - 1, taken)
        if bottom >= 2 and left >= 0 and row < height:
            taken.append((bottom, left, left + wide - 1, (row, col), text, key, tip))
    return taken


def laid(tags, width, height, lit=False, shut=frozenset()):
    """{(row, col): (codepoint, key)}: each of `tags` [((row, col), key, word)], nearest first, a
    balloon over its cell (`_placed`), its tail down to it; dropped where the drawing has no room.
    `lit`, framed in the key's ink: a choice is open."""
    frame, inks = packed(NOTE_INK if lit else FRAME_INK), (packed(NOTE_INK, PLATE), packed(WORD_INK, PLATE))
    placed, out = _placed(tags, width, height, shut), {}
    for bottom, _left, _right, (row, col), _text, _key, tip in placed:
        out.update({(r, col): (ord(TAIL), frame) for r in range(bottom + 1, row)})
        out[(row, col)] = (ord(tip), frame)
    for bottom, left, right, (_row, col), text, key, _tip in placed:
        edges = ((bottom - 2, FRAME_CORNERS[:2], '─' * len(text)),
                 (bottom, FRAME_CORNERS[2:], ''.join(NOTCH if left + 1 + c == col else '─' for c in range(len(text)))))
        for r, corners, edge in edges:
            out.update({(r, left + c): (ord(ch), frame) for c, ch in enumerate(corners[0] + edge + corners[1])})
        out[(bottom - 1, left)] = out[(bottom - 1, right)] = (ord('│'), frame)
        out.update({(bottom - 1, left + 1 + c): (ord(ch), inks[not 0 < c <= len(key)]) for c, ch in enumerate(text)})
    return out
