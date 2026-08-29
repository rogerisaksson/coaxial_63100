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

#: Board dimensions, RADII in millimetres: 100 mm across with a 10 mm bore,
#: confirmed 2026-08-29. `electronics/` is the authority on the rest.
OUTER_MM = 50.0
BORE_MM = 5.0

#: The finest grid worth drawing. Cost is O(cells^2) field evaluations, so
#: this is what stops a tall window spending a second a frame - measured, 88
#: cells is 7744 points and draws in 40 ms, which at 2 Hz is nothing.
#:
#: It was 44, which is what made the circle read as a staircase: the raster
#: is the only antialiasing there is, and half the cells is twice the step.
CELLS_MAX = 88

#: How tall a drawn cell is against its width, on the screen.
#:
#: BOTH RENDERERS ARE SQUARE IN CELLS - the ramp spends two characters a cell
#: and one row, the half-block one character and half a row - so a circle of
#: equal cells each way is round only if a cell is square on the glass. It is
#: not: a terminal character is about 9 x 20 pixels, so the cell comes out a
#: tenth taller than it is wide and the board stands up as an oval.
#:
#: Applied to the FIELD rather than to the grid: the row spacing in
#: millimetres is stretched by this, so fewer rows fit inside the radius and
#: the drawn shape comes back round. Tunable because it belongs to the font,
#: not to the board - `--aspect` on the views.
CELL_ASPECT = 1.10

#: The bore is drawn at least this many CELLS across, whatever the board's
#: millimetres work out to at the resolution in hand.
#:
#: A drawing concession and not a dimension: 5 mm of 50 is a tenth of the
#: radius, which lands on one cell at any terminal size worth using, and one
#: cell is a dent rather than a hole. Widened here rather than in BORE_MM,
#: because that one is what the board IS and belongs to `electronics/`.
#:
#: It only bites on the COARSE grid. The colour renderer spends one character
#: a cell and half a row, so it affords two or three times the cells and the
#: physical 5 mm resolves on its own - this is the plain ramp's floor.
#:
#: SIZE IS THE ONLY KNOB. A superellipse was tried at three exponents and
#: changed nothing - the raster is too coarse to care about the shape of the
#: curve, only about how many FIELD ROWS fall inside it:
#:
#:   up to 2.0 cells   2-4-4-2         the discrete circle, and the smallest
#:   2.4               4-4-4-4         a square
#:   3.2               4-6-6-6-6-4     an octagon
#:
#: 2.0, because the colour renderer carries TWO field rows per character
#: row: 2-4-4-2 is four characters across and two rows down, which is round
#: on the glass. The three larger shapes were each tried on the bench and
#: read as a square or a punched-out middle.
#:
#: The plain ramp spends a whole row per field row, so the same shape is
#: twice as tall there and coarser for it. That is the fallback - a pipe, a
#: log - and the colour path is what anybody looks at.
BORE_MIN_CELLS = 2.0

#: Where the heat sources sit: (x, y, sigma) in millimetres per zone.
#:
#: `sigma` is how wide the blob is laid down, not the part's size - spreading
#: in copper is wider than the device feeding it. A row of points is a row of
#: parts, and they sum.
LAYOUT = {
    #: The six FETs sit just inside the three phase terminals along the top,
    #: two per leg, and each pair sits above its own driver. One blob each so
    #: the same watts draw the same size whichever leg made them - the row
    #: used to be four undivided points and a hot U spread over all of it.
    'phase_u': [(-24, 31, 8)],
    'phase_v': [(-2, 36, 8)],
    'phase_w': [(19, 32, 8)],
    #: The drivers are a band below them, and the middle one is the NTC's
    #: neighbour - which is why the NTC reads that hot spot and not the board.
    'driver_u': [(-20, 19, 9)],
    'driver_v': [(0, 22, 9)],
    'driver_w': [(17, 19, 9)],
    #: The supply corner: bucks, LDOs and the LED droppers, out on the left.
    'regulators': [(-39, 8, 10), (-40, -4, 10), (-34, 1, 9)],
    #: The LQFP, left of centre and below it.
    'mcu': [(-16, -13, 12)],
    #: The amplifiers along the bottom.
    'afe': [(-16, -33, 9), (2, -37, 9), (19, -33, 9)],
    #: The hot swap and its terminal, on the right.
    'hotswap': [(38, -6, 10), (35, 4, 9)],
}

#: Two characters a cell in the plain ramp, so pixels come out square. In
#: colour the half block buys that back and a cell is one character.
CELL = '  '

#: Plain-text ramp, coolest first. No leading space - a space means off the
#: board. Twenty steps, not nine: 120 K over nine characters is 13 K a step
#: and drew the board flat.
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


def _grid(nodes, board_c, cells, layout, aspect=CELL_ASPECT):
    """(rows of temperature-or-None, lo, hi). None is off the board."""
    per_cell = 2.0 * OUTER_MM / cells
    bore = max(BORE_MM, BORE_MIN_CELLS * per_cell)

    # THE GRID IS NOT SQUARE, and that is the point. A cell is taller than it
    # is wide on the glass, so a round board needs fewer rows than columns -
    # and the rows have to SPAN the board rather than be a square grid with
    # blank margins trimmed off, because the trim lands on whole rows and
    # leaves one more at the top than the bottom. Measured: the outline read
    # 16, 24, 30 down the top and 26, 20, 10 up from the bottom.
    #
    # Even, because the half-block renderer pairs rows two to a line.
    down = max(4, int(round(cells / aspect)) // 2 * 2)
    per_row = 2.0 * OUTER_MM / down

    rows, lo, hi = [], None, None
    for row in range(down):
        line = []
        for col in range(cells):
            x = (col - (cells - 1) / 2.0) * per_cell
            y = ((down - 1) / 2.0 - row) * per_row
            r = math.hypot(x, y)
            if r > OUTER_MM or r < bore:
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

    # EVEN. The half-block renderer draws two field rows per character row,
    # so an odd count leaves the last one unpaired - it comes out as a solid
    # background block and the bottom of the board reads blockier than the
    # top. Rounded DOWN, so it still fits what was measured to be free.
    return max(10, min(CELLS_MAX, min(wide, high))) // 2 * 2


def _half_rows(grid):
    """Colour rows, two picture rows per line, through `ansi.HALF`.

    The glyph is a LOWER half block: the background paints the row above it
    and the foreground the row below. One character row therefore carries two
    rows of the field, so the picture stays square at one character per cell
    instead of two - half the width and half the height of the plain ramp.

    An edge cell has only one of the two, and BOTH cases get a half block -
    the upper one its own glyph. Painting the upper-only case as a
    background-coloured space filled the whole cell, which made the bottom
    of a round board a cell coarser than the top.
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
                # The UPPER half block, not a background-coloured space: a
                # space paints the whole cell, so the bottom edge of the
                # board came out a cell coarser than the top and read as
                # blocky against it.
                want = ansi.RESET + ansi.code(ansi.thermal(over))
                text = ansi.HALF_UP
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
           reserve=None, trailing=2, aspect=CELL_ASPECT):
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

    grid, lo, _hi = _grid(nodes, board_c, cells, layout, aspect)
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
