"""Callouts on a drawing: a framed column of rows a joint, a leader to it.

`callouts` places them at the drawing's edges, or around her (`band`), each at its joint's
height at rest, and draws the leaders; `line` a line of dots, `lines` many; `packed` a cell's inks
as a key.
"""
from coaxial.graphics.raster import DOTS_X, DOTS_Y


#: A callout's leader ink.
LEADER_INK = (96, 110, 124)


def line(dots, a, b):
    """The dots from `a` to `b`, (x, y) dot coordinates, set in `dots`: clipped to them
    (Liang-Barsky, the boundaries in turn), a dot a step of its longer side - numpy's linspace -,
    rounded to its nearest."""
    import numpy as np
    h, w = dots.shape
    if h <= 0 or w <= 0:
        return
    ax, ay, bx, by = float(a[0]), float(a[1]), float(b[0]), float(b[1])
    dx, dy = bx - ax, by - ay
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, ax), (dx, (w - 1) - ax), (-dy, ay), (dy, (h - 1) - ay)):
        if p == 0.0:
            if q < 0.0:
                return
        else:
            r = q / p
            if p < 0.0:
                if r > t1:
                    return
                if r > t0:
                    t0 = r
            else:
                if r < t0:
                    return
                if r < t1:
                    t1 = r
    x0, y0 = ax + t0 * dx, ay + t0 * dy
    x1, y1 = ax + t1 * dx, ay + t1 * dy
    n = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
    x = np.clip(np.rint(np.linspace(x0, x1, n)).astype(int), 0, w - 1)
    y = np.clip(np.rint(np.linspace(y0, y1, n)).astype(int), 0, h - 1)
    dots[y, x] = True


def lines(dots, ax, ay, bx, by):
    """`line` from (`ax`, `ay`) to (`bx`, `by`), arrays of dot coordinates, every one at once:
    the same dots, its clip and its linspace in numpy over all of them - a ring's 96 chords one
    call, not 96 (test_views_drawing holds them equal)."""
    import numpy as np
    h, w = dots.shape
    if h <= 0 or w <= 0:
        return
    ax, ay, bx, by = (np.asarray(v, float).ravel() for v in (ax, ay, bx, by))
    dx, dy = bx - ax, by - ay
    t0, t1, alive = np.zeros(ax.shape), np.ones(ax.shape), np.ones(ax.shape, bool)
    with np.errstate(divide='ignore', invalid='ignore'):
        for p, q in ((-dx, ax), (dx, (w - 1) - ax), (-dy, ay), (dy, (h - 1) - ay)):
            alive &= ~((p == 0.0) & (q < 0.0))
            r = q / p
            neg, pos = p < 0.0, p > 0.0
            alive &= ~(neg & (r > t1))
            t0 = np.where(neg & (r > t0), r, t0)
            alive &= ~(pos & (r < t0))
            t1 = np.where(pos & (r < t1), r, t1)
    if not alive.any():
        return
    ax, ay, dx, dy, t0, t1 = ax[alive], ay[alive], dx[alive], dy[alive], t0[alive], t1[alive]
    x0, y0, x1, y1 = ax + t0 * dx, ay + t0 * dy, ax + t1 * dx, ay + t1 * dy
    n = np.maximum(np.abs(x1 - x0), np.abs(y1 - y0)).astype(int) + 1
    seg = np.repeat(np.arange(len(n)), n)
    k = np.arange(len(seg)) - np.repeat(np.cumsum(n) - n, n)
    div = np.maximum(n - 1, 1)[seg]
    x = k * ((x1 - x0)[seg] / div) + x0[seg]
    y = k * ((y1 - y0)[seg] / div) + y0[seg]
    end = (k == n[seg] - 1) & (n[seg] > 1)
    x[end], y[end] = x1[seg][end], y1[seg][end]
    dots[np.clip(np.rint(y).astype(int), 0, h - 1), np.clip(np.rint(x).astype(int), 0, w - 1)] = True


def packed(fg, bg=None):
    """A cell's inks as one key: fg's 24 bits, bg's over them plus one, 0 for none; -1 no fg."""
    if fg is None:
        return -1
    key = (fg[0] << 16) | (fg[1] << 8) | fg[2]
    return key if bg is None else key | (((bg[0] << 16) | (bg[1] << 8) | bg[2]) + 1) << 24


#: A callout's frame, as the tty's instruments have theirs (`frame.hud`, rounded): its ink and
#: corners.
FRAME_INK, FRAME_CORNERS = (95, 135, 135), '╭╮╰╯'


def framed(overlay, top, left, rows, ink, title=''):
    """`rows` [[(char, fg, bg)], ..] in a rounded frame, its top-left corner at (`top`, `left`),
    `title` in its top edge: into `overlay`."""
    wide = max(max(len(r) for r in rows), len(title)) + 2
    for col in range(left, left + wide):
        edge = col in (left, left + wide - 1)
        for row, corner in ((top, 0), (top + len(rows) + 1, 2)):
            overlay[(row, col)] = (ord(FRAME_CORNERS[corner + (col != left)] if edge else '─'),
                                   ink)
    for c, char in enumerate(title):
        overlay[(top, left + 1 + c)] = (ord(char), ink)
    for k, cells in enumerate(rows):
        overlay[(top + 1 + k, left)] = overlay[(top + 1 + k, left + wide - 1)] = (ord('│'), ink)
        for c, (char, fg, bg) in enumerate(cells):
            overlay[(top + 1 + k, left + 1 + c)] = (ord(char), packed(fg, bg))


def callouts(labels, anchors, places, width, height, room=None, band=None):
    """({(row, col): (codepoint, key)}, leader dots) for `labels` {joint: [row, ..]}, a row
    [(char, fg, bg)], inks (r, g, b) or None: each a narrow framed column docked at the drawing's
    edge - a side's joints on its side and the rest on the side they stand - at the height its
    joint has at rest (`places` {joint: (x, y)}, dots), stacked down the edge as they meet and a
    column further in when the edge is full, within the first `room` rows (all of them); a leader
    from its inner edge to the joint's pivot as it is (`anchors`, dots). The callouts stand
    still; the leaders follow. `band` (left, right) cells: docked there, around her, not at the
    drawing's edges."""
    room = height if room is None else room
    import numpy as np
    dots = np.zeros((height * DOTS_Y, width * DOTS_X), bool)
    mid = width * DOTS_X / 2.0
    xs = {side: [x for j, (x, _y) in places.items() if j.startswith(side)]
          for side in ('left_', 'right_')}
    flip = bool(xs['left_'] and xs['right_']) and np.mean(xs['left_']) > np.mean(xs['right_'])
    sides = {True: [], False: []}
    for joint, rows in labels.items():
        # A callout's first row a string: its title, in its frame's top edge.
        if joint in anchors and joint in places:
            side = places[joint][0] < mid
            if joint.startswith(('left_', 'right_')):
                side = joint.startswith('left_') != flip
            sides[side].append((places[joint][1], joint, rows))
    overlay = {}
    ink = packed(FRAME_INK, None)
    for left, items in sides.items():
        items.sort(key=lambda item: item[:2])
        column, below = 0, 0
        for y, joint, rows in items:
            title, rows = (rows[0], rows[1:]) if isinstance(rows[0], str) else ('', rows)
            tall, wide = len(rows) + 2, max(max(len(r) for r in rows), len(title)) + 2
            # Zoomed in, a joint at rest can stand past the drawing's edge: its column is kept
            # within the room (off it, a zoom of 1.1^3 wrote past the last row and threw).
            want = min(max(int(y // DOTS_Y) - 1, 0), max(0, room - tall))
            top = max(want, below)
            if top + tall > room:
                column, top = column + 1, want
            below = top + tall
            if band is None:
                at = column * wide if left else width - (column + 1) * wide
            else:
                at = band[0] - (column + 1) * wide if left else band[1] + column * wide
            if top + tall > room or at < 0 or at + wide > width:
                continue
            framed(overlay, top, at, rows, ink, title)
            end = (at + wide) * DOTS_X if left else at * DOTS_X - 1
            line(dots, (end, (top + 1) * DOTS_Y + DOTS_Y // 2), anchors[joint])
    return overlay, dots
