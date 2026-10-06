"""The QUAD page's traces: height and power, bus and peak temperature, over the last half minute.

    lines = heights(trace, now, width)      # the height, m up its left; the power, W down its right
    lines = buses(trace, now, width)        # the bus, V up its left; the hottest board, C down its right

`trace` is [(s, m, V, W, C)], the flight's clock first; a curve's scale stands on its own side in
its own ink. A column of dots is its samples' mean, the power's most a spike over it: what the
rotors took for a pass before the boards' envelopes had it back.
"""
import math

from coaxial.draw import braille
from machine import quad

#: The traces' span, s; the height's top, m, the band drawn linear from the floor, m, and the
#: share of the plot it takes; the heights labelled up its left edge, m.
TRACE_S, TOP_M, LINEAR_M, LINEAR_SHARE = 30.0, 200.0, 2.0, 0.3
MARKS = (0.0, 0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0)

#: The power's scale, W to at least this in steps of POWER_STEP_W; the bus's and the
#: temperature's, over at least these about what they were, V and K; each plot's rows.
POWER_W, POWER_STEP_W, BUS_V, TEMP_K, HEIGHT_ROWS, BUS_ROWS = 1000.0, 500.0, 6.0, 10.0, 7, 4

#: The inks, as the side column takes them: SGR over a string, not rich's styles. A plot's
#: left curve and its scale, its right one's: the height's and the bus's, the power's and the
#: temperature's (the bench, 2026-10-05; both plots alike, 2026-10-06).
RESET = '\x1b[0m'
LEFT, RIGHT = '\x1b[38;5;51m', '\x1b[38;5;207m'


def height_y(h):
    """A height on the trace's scale, 0 at the floor to 1 at its top: linear to LINEAR_M, then
    logarithmic."""
    if h <= LINEAR_M:
        return LINEAR_SHARE * max(0.0, h) / LINEAR_M
    return min(1.0, LINEAR_SHARE + (1.0 - LINEAR_SHARE) * math.log10(h / LINEAR_M)
               / math.log10(TOP_M / LINEAR_M))


def _dots(points, now, wide, rows, rule=None, spikes=False):
    """`points` [(s, share)] - 0 the foot, 1 the top - over the last TRACE_S as a grid of
    braille bits, `rows` by `wide` cells, a dotted `rule` across at that share; `spikes` each
    column's most drawn up from its mean."""
    dots_x, dots_y = wide * 2, rows * 4
    cells = [[0] * wide for _ in range(rows)]

    def dot(x, y):
        if 0 <= x < dots_x and 0 <= y < dots_y:
            cells[y // 4][x // 2] |= braille.BIT[x % 2][y % 4]

    def row_of(share):
        return dots_y - 1 - int(round(max(0.0, min(1.0, share)) * (dots_y - 1)))
    if rule is not None:
        for x in range(0, dots_x, 3):
            dot(x, row_of(rule))
    # A column of dots its samples' mean: every pass joined to the next, the power was a comb
    # of the rotors' loops at their clamp.
    columns = {}
    for t, share in points:
        column = columns.setdefault(int(round((1.0 - (now - t) / TRACE_S) * (dots_x - 1))), [])
        column.append(share)
    was = None
    for x in sorted(columns):
        y = row_of(sum(columns[x]) / len(columns[x]))
        if was is not None:
            for k in range(1, abs(y - was[1]) + 1):
                dot(was[0] + (x - was[0]) * k // max(1, abs(y - was[1])),
                    was[1] + (1 if y > was[1] else -1) * k)
        dot(x, y)
        for peak in range(row_of(max(columns[x])), y if spikes else 0):
            dot(x, peak)
        was = (x, y)
    return cells


def plot(now, width, rows, left, right, rule=None, spikes=False):
    """Two curves over the last TRACE_S as `rows` lines `width` cells wide: `left` and `right`
    each ([(s, share)], {row: label}, ink), its labels on its own side in its ink, `rule` the
    left's dotted line, `spikes` the right's most a column; a cell both cross is the right's."""
    wide = max(4, width - 15)
    under = _dots(left[0], now, wide, rows, rule)
    over = _dots(right[0], now, wide, rows, spikes=spikes)
    lines = []
    for r in range(rows):
        row, ink = [], None
        for a, b in zip(under[r], over[r]):
            want = right[2] if b else left[2]
            if want != ink:
                row.append(want)
                ink = want
            row.append(braille.ALL[a | b])
        lines.append(left[2] + ('%5s ┤' % left[1][r] if r in left[1] else '      │') + RESET
                     + ''.join(row) + right[2]
                     + ('├ %s' % right[1][r] if r in right[1] else '│') + RESET)
    return lines


def _range(values, least, steps=1):
    """(low, high) whole about `values`, `least` apart at least and a whole number a step of
    `steps`."""
    high = math.ceil(max(values))
    low = min(high - least, math.floor(min(values)))
    return high - steps * math.ceil((high - low) / steps), high


def heights(trace, now, width):
    """The height, its marks up its left and the stop's line dotted; the four's power down
    its right, to the half kilowatt over its most, a pass's most a spike."""
    marks = {HEIGHT_ROWS - 1 - min(HEIGHT_ROWS - 1, int(height_y(m) * (HEIGHT_ROWS - 1) + 0.5)):
             '%g' % m for m in MARKS}
    top = max([POWER_W] + [POWER_STEP_W * math.ceil(w / POWER_STEP_W) for _t, _h, _v, w, _c in trace])
    return plot(now, width, HEIGHT_ROWS,
                ([(t, height_y(h)) for t, h, _v, _w, _c in trace], marks, LEFT),
                ([(t, w / top) for t, _h, _v, w, _c in trace],
                 {0: '%gkW' % (top / 1000.0), HEIGHT_ROWS - 1: '0'}, RIGHT),
                rule=height_y(quad.FLOOR_M), spikes=True)


def buses(trace, now, width):
    """The bus, V up its left - a mark a row - and the boards' hottest node, C down its
    right, each on the range it had; inked and marked as the height and the power are."""
    low, high = _range([v for _t, _h, v, _w, _c in trace] or [quad.open_volts(1.0)], BUS_V,
                       BUS_ROWS - 1)
    warm = [(t, c) for t, _h, _v, _w, c in trace if c is not None]
    cool, hot = _range([c for _t, c in warm] or [25.0], TEMP_K)
    return plot(now, width, BUS_ROWS,
                ([(t, (v - low) / (high - low)) for t, _h, v, _w, _c in trace],
                 {r: '%g' % (high - r * (high - low) / (BUS_ROWS - 1)) for r in range(BUS_ROWS)},
                 LEFT),
                ([(t, (c - cool) / (hot - cool)) for t, c in warm],
                 {0: '%gC' % hot, BUS_ROWS - 1: '%g' % cool}, RIGHT))
