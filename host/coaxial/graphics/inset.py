"""A picture in a viewport corner, framed as the tty's instruments: the robot's eyes' view, a camera's.

    cells = inset.eyes(seen, width, height)                  # the robot's look, its things hulled
    cells = inset.docked(inset.dotted(dots, ink, ground), width, height, 'GIMBAL', note)

A body is rows of cells [(codepoint, key)]: `pixels` an image, two pixel rows a cell; `dotted`
braille dots, an ink a cell on its ground. `docked` frames one at the viewport's top right - the
instruments' rounded frame, its title in the top edge and a note in the bottom one - its
`labels` plates on the picture, none over another; nothing where the viewport has no room.
"""
from coaxial.graphics.callouts import FRAME_CORNERS, FRAME_INK, packed
from coaxial.graphics.raster import BRAILLE, BRAILLE_BITS, DOTS_Y
from machine.eyes import sight

#: The inks of the frame's title and its note (the theme's `name` and `value`), and a label's
#: plate.
TITLE_INK, NOTE_INK, PLATE = (0, 215, 215), (255, 175, 0), (8, 14, 16)

#: The robot's eyes' picture: its title, and the share of a hull's ink laid over what it covers and
#: along its edge.
EYES, TINT, EDGE = 'VISION', 0.15, 0.7


def pixels(rgb):
    """A body of the image `rgb` ((2 * rows, wide, 3) uint8): two pixel rows a cell."""
    rows = rgb.tolist()
    return [[(ord('\u2580'), packed(tuple(top), tuple(low))) for top, low in zip(rows[r], rows[r + 1])]
            for r in range(0, len(rows) - 1, 2)]


def dotted(dots, ink, ground):
    """A body of braille: `dots` ((4 * rows, 2 * wide) bool), a cell's `ink` and its `ground`
    ((rows, wide, 3))."""
    from coaxial.model.blocks import numpy as np
    rows = dots.shape[0] // DOTS_Y
    glyphs = ((dots[0::4, 0::2] * BRAILLE_BITS[0][0]) | (dots[1::4, 0::2] * BRAILLE_BITS[0][1])
              | (dots[2::4, 0::2] * BRAILLE_BITS[0][2]) | (dots[3::4, 0::2] * BRAILLE_BITS[0][3])
              | (dots[0::4, 1::2] * BRAILLE_BITS[1][0]) | (dots[1::4, 1::2] * BRAILLE_BITS[1][1])
              | (dots[2::4, 1::2] * BRAILLE_BITS[1][2]) | (dots[3::4, 1::2] * BRAILLE_BITS[1][3]))
    fi, bi = ink.astype(np.int64), ground.astype(np.int64)
    fg = (fi[..., 0] << 16) | (fi[..., 1] << 8) | fi[..., 2]
    bg = (((bi[..., 0] << 16) | (bi[..., 1] << 8) | bi[..., 2]) + 1) << 24
    chars = np.where(glyphs, BRAILLE + glyphs.astype(int), ord(' ')).tolist()
    keys = (fg | bg).tolist()
    return [list(zip(chars[r], keys[r])) for r in range(rows)]


def docked(body, width, height, title, note='', labels=()):
    """{(row, col): (codepoint, key)}: `body` in the instruments' frame at the top right of a
    viewport `width` by `height` cells, `title` in its top edge and `note` in its bottom one;
    `labels` [(row, col, text, ink)], the body's cells, each a plate on it (`_laid`). Nothing
    where the viewport has no room for it."""
    rows, wide = len(body), len(body[0])
    top, left = 0, width - wide - 2
    if left < 0 or rows + 2 > height:
        return {}
    frame, out = packed(FRAME_INK), {}
    for row, corners, text, ink in ((top, FRAME_CORNERS[:2], title, TITLE_INK),
                                    (top + rows + 1, FRAME_CORNERS[2:], note, NOTE_INK)):
        text = ' %s ' % text[:wide - 4] if text else ''
        at = 1 if row == top else wide - 1 - len(text)
        edge = [(ord('\u2500'), frame)] * wide
        edge[at:at + len(text)] = [(ord(ch), packed(ink)) for ch in text]
        for c, cell in enumerate([(ord(corners[0]), frame)] + edge[:wide]
                                 + [(ord(corners[1]), frame)]):
            out[(row, left + c)] = cell
    for r, cells in enumerate(body):
        out[(top + 1 + r, left)] = out[(top + 1 + r, left + wide + 1)] = (ord('\u2502'), frame)
        for c, cell in enumerate(cells):
            out[(top + 1 + r, left + 1 + c)] = cell
    for r, c, text, ink in _laid(labels, rows, wide):
        for k, ch in enumerate(text):
            out[(top + 1 + r, left + 1 + c + k)] = (ord(ch), packed(ink, PLATE))
    return out


def _laid(labels, rows, wide):
    """`labels` [(row, col, text, ink)] where each is drawn: kept inside `rows` by `wide`
    cells, a row down or up - two at most - off the ones laid before it, a cell between two on
    a row; dropped where none is free."""
    out = []
    for row, col, text, ink in labels:
        text = text[:wide]
        col = max(0, min(wide - len(text), int(col)))
        for r in (int(row) + step for step in (0, 1, -1, 2, -2)):
            if 0 <= r < rows and not any(r == r0 and col <= c0 + len(t0) and c0 <= col + len(text)
                                         for r0, c0, t0, _ink in out):
                out.append((r, col, text, ink))
                break
    return out


def eyes(seen, width, height):
    """The robot's eyes' picture docked (`docked`): `seen` (thumbnail (2 * rows, wide, 3) uint8, hulls
    [(kind, ink, [(col, row), ..] in the eye's image)]) - each hull's ink along its edge, where
    the picture's own edge does not cut it, and lightly over what it covers; its kind a label
    at its top left, the smallest hull's first."""
    from coaxial.model.blocks import numpy as np
    thumb, hulls = seen
    rows, wide = thumb.shape[0], thumb.shape[1]
    picture = thumb.astype(float)
    sx, sy = wide / float(sight.W), rows / float(sight.H)
    labels = []
    for kind, ink, hull in hulls:
        poly = [(u * sx, v * sy) for u, v in hull]
        pts = _filled(poly, rows, wide)
        if pts:
            mask = np.zeros((rows, wide), bool)
            rr, cc = zip(*pts)
            mask[rr, cc] = True
            pad = np.pad(mask, 1, constant_values=True)
            rim = mask & ~(pad[:-2, 1:-1] & pad[2:, 1:-1] & pad[1:-1, :-2] & pad[1:-1, 2:])
            inner = mask & ~rim
            ink_f = np.array([float(i) for i in ink])
            picture[inner] = picture[inner] * (1.0 - TINT) + (TINT * ink_f)
            picture[rim] = picture[rim] * (1.0 - EDGE) + (EDGE * ink_f)
            count = int(mask.sum())
        else:
            count = 0
        labels.append((count, min(v for _u, v in poly) // 2, min(u for u, _v in poly),
                       kind, ink))
    return docked(pixels(picture.astype(thumb.dtype)), width, height, EYES,
                  '%d told' % len(hulls) if hulls else '',
                  [label[1:] for label in sorted(labels, key=lambda label: label[0])])


def _filled(poly, rows, cols):
    """The pixels (row, col) inside the polygon `poly` [(x, y), ..], by scanline."""
    out = []
    n = len(poly)
    if n < 3:
        return out
    for r in range(rows):
        y = r + 0.5
        crossings = []
        for k in range(n):
            (x0, y0), (x1, y1) = poly[k], poly[(k + 1) % n]
            if (y0 <= y < y1) or (y1 <= y < y0):
                crossings.append(x0 + (y - y0) * (x1 - x0) / (y1 - y0))
        crossings.sort()
        for a, b in zip(crossings[::2], crossings[1::2]):
            for c in range(max(0, int(a)), min(cols, int(b) + 1)):
                out.append((r, c))
    return out
