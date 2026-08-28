"""The board as a thermal picture: stylised zones, in colour.

A reading in, text out. No serial port, no terminal, no clock - so it tests
without a board, the same way `orientation.py` and `dial.py` do.

**Stylised, not a CAD render.** The board's own ASCII from `ascii3d.py` shows
every component and is unreadable as a temperature field; this shows where the
heat is and nothing else. Solid cells painted by temperature, the way an IR
camera draws one - no letters on the field, because a glyph on a thermal
picture reads as a cold spot.

**The field is diffuse on purpose.** Heat in a laminate spreads; sharp zone
edges would be a lie about the physics, and would invite reading a cell as if
it were a measurement. Every source is a Gaussian blob and they sum.

GEOMETRY
An annulus, 100 mm across with a 10 mm bore, mounted coaxially behind an
outrunner's stator. Millimetres from centre, +y up, quadrants as in
mathematics - Q1 upper right.

    switches      across the top
    regulators    left
    mcu           left, below centre
    afe           along the bottom
    hot swap      right
    DC link       Q1 and just into Q2, out at the rim

**Placed by tape measure, not from CAD.** Good enough to see where the heat
sits and too coarse to compute with. `render` takes the layout as an argument
so a better set can be passed in without touching this file.
"""
import math
import shutil
import sys

from . import ansi

#: Board dimensions, millimetres. `electronics/` is the authority on the rest.
OUTER_MM = 50.0
BORE_MM = 5.0

#: Where the heat sources sit: (x, y, sigma) in millimetres per zone.
#:
#: `sigma` is how wide the blob is laid down, not the part's size - spreading
#: in copper is wider than the device feeding it. A row of points is a row of
#: parts, and they sum.
LAYOUT = {
    #: The six FETs sit just inside the three phase terminals along the top.
    'phases': [(-26, 30, 8), (-14, 34, 8), (2, 36, 8), (20, 32, 8)],
    #: The drivers are a band below them, and the middle one is the NTC's
    #: neighbour - which is why the NTC reads that hot spot and not the board.
    'drivers': [(-20, 19, 9), (0, 22, 9), (17, 19, 9)],
    #: The supply corner: bucks, LDOs and the LED droppers, out on the left.
    'regulators': [(-39, 8, 10), (-40, -4, 10), (-34, 1, 9)],
    #: The LQFP, left of centre and below it.
    'mcu': [(-16, -13, 12)],
    #: The amplifiers along the bottom.
    'afe': [(-16, -33, 9), (2, -37, 9), (19, -33, 9)],
    #: The hot swap and its terminal, on the right.
    'hotswap': [(38, -6, 10), (35, 4, 9)],
}

#: One cell is two characters in the PLAIN ramp, so the pixels come out
#: square - a terminal cell is about twice as tall as it is wide. In colour
#: the half block buys that back and a cell is one character.
CELL = '  '

#: Plain-text ramp, for when there is no colour. Coolest to hottest, and **no
#: leading space** - a space means off the board, and a ramp starting with one
#: makes the board's coolest parts indistinguishable from empty.
#:
#: Twenty steps, not nine: the scale is fixed at 120 K wide, and nine
#: characters over that is 13 K a step, which drew the board as one flat
#: symbol. Six kelvin a step shows the zones.
RAMP = '.,:;~-=+ic*xX#$%8W@'

#: The bar and its tick labels. Counted by the caller, so it is named.
SCALE_LINES = 2


def field(x_mm, y_mm, board_c, nodes, layout=None):
    """Temperature at one point: the board plus every source's contribution."""
    layout = LAYOUT if layout is None else layout
    got = board_c
    for name, spots in layout.items():
        value = nodes.get(name)
        if value is None:
            continue
        over = value - board_c
        if abs(over) < 1e-6:
            continue
        # STRONGEST point in the zone, not the sum of them. Dividing the rise
        # across a row made a row of four FETs peak at a quarter of its own
        # temperature, so the zone never reached the number the model gave it
        # and the picture said something different from the figures.
        # Zones still add to each other - that part is real.
        near = 0.0
        for sx, sy, sigma in spots:
            d2 = (x_mm - sx) ** 2 + (y_mm - sy) ** 2
            near = max(near, math.exp(-d2 / (2.0 * sigma * sigma)))
        got += over * near
    return got


def _grid(nodes, board_c, cells, layout):
    """(rows of temperature-or-None, lo, hi). None is off the board."""
    rows, lo, hi = [], None, None
    for row in range(cells):
        line = []
        for col in range(cells):
            x = (col - (cells - 1) / 2.0) * (2.0 * OUTER_MM / cells)
            y = ((cells - 1) / 2.0 - row) * (2.0 * OUTER_MM / cells)
            r = math.hypot(x, y)
            if r > OUTER_MM or r < BORE_MM:
                line.append(None)
                continue
            t = field(x, y, board_c, nodes, layout)
            line.append(t)
            lo = t if lo is None else min(lo, t)
            hi = t if hi is None else max(hi, t)
        rows.append(line)
    return rows, lo, hi


def _fit(colour, reserve):
    """Cells across the board, from the terminal.

    `reserve` is every line in the finished frame that is NOT picture: the
    banner, the readings, the blank lines, the scale and its labels. The
    caller counts its own, because guessing here is what clipped the bottom
    of the board off - a guess of 16 against a frame that spent 18.
    """
    size = shutil.get_terminal_size((80, 30))
    rows = max(size.lines - reserve, 8)
    if colour:
        wide, high = size.columns - 2, rows * 2
    else:
        wide, high = (size.columns - 2) // 2, rows
    return max(10, min(44, min(wide, high)))


def _half_rows(grid):
    """Colour rows, two picture rows per line, through `ansi.HALF`.

    The glyph is a LOWER half block: the background paints the row above it
    and the foreground the row below. One character row therefore carries two
    rows of the field, so the picture stays square at one character per cell
    instead of two - half the width and half the height of the plain ramp.
    """
    out = []
    for top in range(0, len(grid), 2):
        upper = grid[top]
        lower = grid[top + 1] if top + 1 < len(grid) else [None] * len(upper)
        parts, current = [], None

        for col, over in enumerate(upper):
            under = lower[col]
            # Each `want` fully determines the cell, so runs of equal colour
            # collapse to one escape - a smooth region costs a handful a line
            # rather than one per cell.
            if over is None and under is None:
                want, text = ansi.RESET, ' '
            elif under is None:
                want, text = ansi.back(ansi.thermal(over)), ' '
            elif over is None:
                want = ansi.RESET + ansi.code(ansi.thermal(under))
                text = ansi.HALF
            else:
                want = (ansi.back(ansi.thermal(over))
                        + ansi.code(ansi.thermal(under)))
                text = ansi.HALF

            if want != current:
                parts.append(want)
                current = want
            parts.append(text)

        out.append(''.join(parts) + ansi.RESET)
    return out


def _ramp_rows(grid):
    """Plain rows, one picture row per line, two characters per cell."""
    span = ansi.THERMAL_MAX - ansi.THERMAL_MIN
    out = []
    for line in grid:
        text = []
        for t in line:
            if t is None:
                text.append(CELL)
                continue
            fraction = (t - ansi.THERMAL_MIN) / span
            step = int(max(0.0, min(1.0, fraction)) * (len(RAMP) - 1))
            text.append(RAMP[step] * len(CELL))
        out.append(''.join(text))
    return out


def render(nodes, board_c, cells=None, colour=None, layout=None, title=None,
           reserve=None, trailing=2):
    """The board as a thermal picture.

    `nodes` is {zone: degrees} and `board_c` the bulk the field falls back to
    where no source reaches.

    The scale is FIXED, from `ansi.THERMAL_STOPS` - a colour is a temperature
    and means the same in every picture. Auto-ranging made a cool board look
    exactly like a hot one, so two pictures said nothing side by side.

    `cells` None fits the terminal, and `reserve` is how many lines the
    caller spends on everything that is not the picture - it cannot be known
    from here, and guessing it clipped the board's bottom edge.

    `trailing` blank lines follow the scale, so the last row of the picture
    is not the last row of the terminal.

    `colour` None asks the terminal: escapes into a pipe or a log are noise,
    and the character ramp reads fine there.
    """
    if colour is None:
        colour = bool(getattr(sys.stdout, 'isatty', lambda: False)())
    if cells is None:
        # SCALE_LINES plus the blank above them is what render itself adds;
        # anything else in the frame is the caller's to count.
        cells = _fit(colour, (SCALE_LINES + 1 + trailing)
                     if reserve is None else reserve)
    layout = LAYOUT if layout is None else layout

    grid, lo, _hi = _grid(nodes, board_c, cells, layout)
    if lo is None:
        return 'nothing to draw'

    out = []
    if title:
        out.extend(['  ' + title, ''])
    out.extend(_half_rows(grid) if colour else _ramp_rows(grid))
    out.append('')
    out.extend(_scale(cells if colour else cells * len(CELL), colour))
    out.extend([''] * trailing)
    return '\n'.join(out)


def _scale(width, colour, ticks=(-20, 0, 20, 40, 60, 80, 100)):
    """The temperature scale under the picture, same width as the board."""
    lo, hi = ansi.THERMAL_MIN, ansi.THERMAL_MAX

    if colour:
        bar = ''.join(
            ansi.back(ansi.thermal(lo + (hi - lo) * i / (width - 1.0))) + ' '
            for i in range(width)) + ansi.RESET
    else:
        bar = ''.join(
            RAMP[int(i / (width - 1.0) * (len(RAMP) - 1))]
            for i in range(width))

    labels = [' '] * width
    for t in ticks:
        text = '%d' % t
        at = int((t - lo) / (hi - lo) * (width - 1)) - len(text) // 2
        at = max(0, min(width - len(text), at))
        for k, ch in enumerate(text):
            labels[at + k] = ch

    return [bar, ''.join(labels).rstrip() + ' C']
