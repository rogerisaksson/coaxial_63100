"""The picture in the viewport's corner. Her eyes' view, each thing she has told hulled and
labelled in its kind's ink (`machine.sight.Keep.view`, docs/TODO.md 28)."""
from coaxial.graphics.callouts import packed
from machine import sight

#: The picture's place from the viewport's top-right corner (rows down, columns in), the
#: tint a hull lays over it, and its frame's ink.
MARGIN, TINT, FRAME = (1, 2), 0.45, (95, 135, 135)


def cells(seen, width):
    """{(row, col): (codepoint, key)} of the picture docked at the viewport's top right: `seen`
    (thumbnail (2 * high, wide, 3) uint8, hulls [(kind, ink, [(col, row), ..] in the eye's
    image)]), two pixel rows a cell."""
    thumb, hulls = seen
    rows, wide = thumb.shape[0], thumb.shape[1]
    high = rows // 2
    picture = [[tuple(int(c) for c in thumb[r, col]) for col in range(wide)] for r in range(rows)]
    sx, sy = wide / float(sight.W), rows / float(sight.H)
    labels = []
    for kind, ink, hull in hulls:
        poly = [(u * sx, v * sy) for u, v in hull]
        for r, c in _filled(poly, rows, wide):
            picture[r][c] = tuple(int(p * (1.0 - TINT) + i * TINT) for p, i in zip(picture[r][c], ink))
        labels.append((min(v for _u, v in poly), min(u for u, _v in poly), kind, ink))
    row0, col0 = MARGIN[0], width - wide - MARGIN[1]
    out = {}
    for r in range(high):
        for c in range(wide):
            out[(row0 + r, col0 + c)] = (ord('▀'), packed(picture[2 * r][c], picture[2 * r + 1][c]))
    for top, left, kind, ink in labels:
        r = row0 + min(high - 1, int(top // 2))
        for k, ch in enumerate(kind[:wide]):
            c = col0 + min(wide - 1, int(left)) + k
            if c < col0 + wide:
                out[(r, c)] = (ord(ch), packed(ink, picture[2 * (r - row0)][c - col0]))
    for c in range(-1, wide + 1):
        out[(row0 - 1, col0 + c)] = (ord('─'), packed(FRAME, None))
        out[(row0 + high, col0 + c)] = (ord('─'), packed(FRAME, None))
    for r in range(high):
        out[(row0 + r, col0 - 1)] = (ord('│'), packed(FRAME, None))
        out[(row0 + r, col0 + wide)] = (ord('│'), packed(FRAME, None))
    return out


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
