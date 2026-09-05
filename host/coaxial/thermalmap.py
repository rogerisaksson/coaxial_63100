"""The board as a thermal picture: a braille halftone in the thermal ramp,
with the rim and the parts that make the heat drawn on it.

A reading in, text out. No serial port, no terminal, no clock - so it tests
without a board, the same way `orientation.py` and `dial.py` do.

**Stylised, not a CAD render.** The board's own ASCII from `ascii3d.py` shows
every component and is unreadable as a temperature field; this shows where
the heat is and what makes it. THE FIELD IS A HALFTONE: each of a braille
cell's eight dots is lit where the temperature under it clears its
threshold in the blue-noise mask the attitude page's face uses
(`raster.NOISE`), so a hot zone is dense and a cool one sparse, and the
cell wears the thermal ramp's colour blended to 24 bits between its stops.
The rim, the bore's edge and the marked parts are drawn over it in solid
dots.

IT WAS HALF BLOCKS: one palette stop a cell, the nearest, and a circle
stepped at the cell. The bench's word was "pixelly", and "the thermal
observer's style to braille too, more anti-aliased" - the face BOARD
ATTITUDE had already settled on. Dots are twice the columns and four
times the rows, the rim is one dot wide, and the colour no longer bands.

**The field is diffuse on purpose.** Heat in a laminate spreads; sharp zone
edges would be a lie about the physics, and would invite reading a cell as
if it were a measurement. Every source is a Gaussian blob and they sum.

GEOMETRY
An annulus, 100 mm across with a 10 mm bore, mounted coaxially behind an
outrunner's stator. Millimetres from centre, +y up, quadrants as in
mathematics - Q1 upper right.

    switches      across the top, two a leg, each pair over its driver
    regulators    left
    mcu           left of centre, below it
    afe           along the bottom
    hot swap      right
    DC link       Q1 and just into Q2, out at the rim

**Placed from the pick and place, 2026-09-05.** `electronics/Coaxial 63100
Pick-Place.csv` is the authority on where a part sits, the way the parts
list is on what is fitted: `PLACED` carries its coordinates for the parts
the model heats and the picture marks, and `PNP_CENTRE` is the board's
centre in the exporter's frame - the midpoint of the parts' extents, which
the three phase pairs' symmetry agrees with to half a millimetre.
`test_sensorless` holds both to the file. It was a tape measure: the
switches were drawn 12 mm too high, the hot swap 15 mm too far out, and
the drivers a leg's width from their legs.

MARKED, on the bench's word: the MCU, the regulators, each phase's
switches WITH ITS SHUNTS, the front-end amplifiers, the hot-swap
controller and the thermistor - each group as a FRAME one dot wide
round the parts, the field's own halftone untouched inside it, and a
label. It was every package as a block with a white edge, and on the
bench those were "grey areas": a cell is one colour, so an edge cell
went white whole, and the small packages were all edge. A frame's cells
light only the frame's dots, so the line is a dot wide with a dot of
dark beside it - the same treatment the rim gets, "ideally one pixel".
The labels are the one place a glyph sits on the field, and they wear
the frame's ink so they read as marks and not as cold spots.
"""
import math
import shutil
import sys

from . import ansi
from .raster import BRAILLE, BRAILLE_BITS, NOISE, NOISE_N

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
#: The halftone puts four dots on every field row and two on every column,
#: so the field is still evaluated here and only the DOTS are finer.
CELLS_MAX = 88

#: How tall a drawn FIELD ROW is against a cell's width, on the screen.
#:
#: BOTH RENDERERS ARE SQUARE IN CELLS - the ramp spends two characters a
#: cell and one row, the halftone one character and half a row - so a circle
#: of equal cells each way is round only if a cell is square on the glass.
#: It is not: a terminal character is about 9 x 20 pixels, so the cell comes
#: out a tenth taller than it is wide and the board stands up as an oval.
#:
#: Applied to the FIELD rather than to the grid: the row spacing in
#: millimetres is stretched by this, so fewer rows fit inside the radius and
#: the drawn shape comes back round. Tunable because it belongs to the font,
#: not to the board - half the character aspect `screen.aspect_of` measures.
CELL_ASPECT = 1.10

#: The bore is drawn at least this many CELLS across on the plain ramp,
#: whatever the board's millimetres work out to at the resolution in hand.
#:
#: A drawing concession and not a dimension: 5 mm of 50 is a tenth of the
#: radius, which lands on one cell at any terminal size worth using, and one
#: cell is a dent rather than a hole. Widened here rather than in BORE_MM,
#: because that one is what the board IS and belongs to `electronics/`.
#:
#: It only bites on the COARSE grid. The halftone spends a dot every half
#: cell and the physical 5 mm resolves on its own - this is the plain
#: ramp's floor, for a pipe or a log.
#:
#: SIZE IS THE ONLY KNOB. A superellipse was tried at three exponents and
#: changed nothing - the raster is too coarse to care about the shape of the
#: curve, only about how many FIELD ROWS fall inside it:
#:
#:   up to 2.0 cells   2-4-4-2         the discrete circle, and the smallest
#:   2.4               4-4-4-4         a square
#:   3.2               4-6-6-6-6-4     an octagon
#:
#: The three larger shapes were each tried on the bench and read as a
#: square or a punched-out middle.
BORE_MIN_CELLS = 2.0

#: The board's centre in the pick and place's frame, millimetres. The
#: exporter's origin is a corner of its own; this is the midpoint of the
#: 527 top-layer placements' extents (x 60.5 to 152.0, y 27.9 to 120.5),
#: and the U and W switch pairs sit 28.7 mm either side of it.
PNP_CENTRE = (106.25, 74.2)


def from_pnp(x_mm, y_mm):
    """A pick-and-place coordinate as millimetres from the board's centre."""
    return (x_mm - PNP_CENTRE[0], y_mm - PNP_CENTRE[1])


#: Where the pick and place puts the parts this module needs, by
#: designator: the exporter's x and y, and the package's body wide by tall
#: as mounted, millimetres. Bodies from the datasheets - LQFP100,
#: PG-TDSON-8, PG-TSDSO-14, VSSOP - and the size is the drawing's, not a
#: dimension anything computes with.
PLACED = {
    'U3':   (89.4245, 65.2677, 14.0, 14.0),   # STM32H753VIT6, LQFP100
    'Q1U':  (80.9244, 93.2180, 5.2, 6.2),     # IAUCN10S7N021, PG-TDSON-8
    'Q2U':  (74.1172, 93.2180, 5.2, 6.2),
    'Q1V':  (109.3724, 94.6150, 5.2, 6.2),
    'Q2V':  (102.5906, 94.6150, 5.2, 6.2),
    'Q1W':  (137.5410, 93.2434, 5.2, 6.2),
    'Q2W':  (130.7592, 93.2434, 5.2, 6.2),
    'RU1':  (74.9260, 107.6333, 4.6, 7.1),    # WSHM2818, 7 mOhm: the two
    'RU2':  (82.8260, 111.7833, 4.6, 7.1),    # shunts a phase, in parallel,
    'RV1':  (101.9260, 109.4333, 4.6, 7.1),   # up by the terminals at the
    'RV2':  (110.9260, 109.4333, 4.6, 7.1),   # rim - 35 W between them at
    'RW1':  (130.0260, 111.7833, 4.6, 7.1),   # 100 A (HARDWARE.md)
    'RW2':  (137.9260, 107.6333, 4.6, 7.1),
    'U1U':  (70.1040, 86.7664, 4.9, 3.0),     # 2EDL8034, PG-TSDSO-14
    'U1V':  (98.9076, 88.0364, 4.9, 3.0),
    'U1W':  (126.7460, 86.8684, 4.9, 3.0),
    'OP1U': (90.9070, 40.0100, 3.0, 3.0),     # THS4551, VSSOP-8: the two
    'OP2U': (90.8816, 47.7012, 3.0, 3.0),     # amplifiers of a chain
    'OP1V': (104.1654, 36.0934, 3.0, 3.0),
    'OP2V': (104.1400, 43.7846, 3.0, 3.0),
    'OP1W': (117.4750, 36.9062, 3.0, 3.0),
    'OP2W': (117.4554, 44.5988, 3.0, 3.0),
    'U2':   (81.6102, 41.0464, 2.9, 1.6),     # REF2033, SOT-23-6
    'U8':   (68.8576, 64.6084, 4.0, 4.0),     # MP4541, QFN: the two bucks
    'U9':   (67.4879, 78.1307, 4.0, 4.0),     # that bring 63 V down
    'U1':   (78.2754, 78.2771, 3.0, 3.0),     # LDI8119-3.3, the 3.3 V LDO
    'U7':   (78.3844, 56.7919, 3.0, 3.0),     # LDI92-05, the 5 V LDO
    'U12':  (129.2097, 73.9139, 3.0, 3.0),    # LM5069, VSSOP-10
    'NTC1': (99.6188, 79.8322, 1.6, 0.8),     # 0603
}


def placed(ref):
    """A part's centre, millimetres from the board's."""
    x, y, _w, _h = PLACED[ref]
    return from_pnp(x, y)


def _blob(sigma, *refs):
    """A heat source at the mean of some parts' centres."""
    xs, ys = zip(*(placed(ref) for ref in refs))
    return (sum(xs) / len(xs), sum(ys) / len(ys), sigma)


#: Where the heat sources sit: (x, y, sigma) in millimetres per zone.
#:
#: `sigma` is how wide the blob is laid down, not the part's size - spreading
#: in copper is wider than the device feeding it. A row of points is a row of
#: parts, and they sum. Every point is a placed part or the middle of a
#: pair: a blob for a leg's switches and one for its shunts - the phase
#: node's watts are the FET's conduction AND the shunt's, and `field`
#: takes the strongest point so both reach the node's temperature - the
#: middle driver the NTC's neighbour, which is why the NTC reads that hot
#: spot and not the board, and the front end as its three chains and the
#: reference.
LAYOUT = {
    'phase_u': [_blob(8, 'Q1U', 'Q2U'), _blob(7, 'RU1', 'RU2')],
    'phase_v': [_blob(8, 'Q1V', 'Q2V'), _blob(7, 'RV1', 'RV2')],
    'phase_w': [_blob(8, 'Q1W', 'Q2W'), _blob(7, 'RW1', 'RW2')],
    'driver_u': [_blob(9, 'U1U')],
    'driver_v': [_blob(9, 'U1V')],
    'driver_w': [_blob(9, 'U1W')],
    'regulators': [_blob(10, 'U8'), _blob(10, 'U9'), _blob(8, 'U1'),
                   _blob(8, 'U7')],
    'mcu': [_blob(12, 'U3')],
    'afe': [_blob(9, 'OP1U', 'OP2U'), _blob(9, 'OP1V', 'OP2V'),
            _blob(9, 'OP1W', 'OP2W'), _blob(8, 'U2')],
    'hotswap': [_blob(10, 'U12')],
}

#: What the picture marks: a label, the parts the frame goes round, and
#: where the label sits - a side of the frame, `inside` where the frame
#: has room (the LQFP, and the gap between a leg's switches and its
#: shunts), or a point in millimetres. REG is the two bucks and the two
#: LDOs left of the MCU - they warm a little bringing 63 V down to what
#: the board runs on - and NTC is the thermistor beside the bore, its
#: label above the hole; each phase's frame takes its shunts, which sit
#: at the rim by the terminals; all on the bench's word.
MARKS = (
    ('MCU', ('U3',), 'inside'),
    ('REG', ('U8', 'U9', 'U1', 'U7'), 'inside'),
    ('U', ('Q1U', 'Q2U', 'RU1', 'RU2'), 'inside'),
    ('V', ('Q1V', 'Q2V', 'RV1', 'RV2'), 'inside'),
    ('W', ('Q1W', 'Q2W', 'RW1', 'RW2'), 'inside'),
    ('AFE', ('OP1U', 'OP2U', 'OP1V', 'OP2V', 'OP1W', 'OP2W'), 'below'),
    ('HS', ('U12',), 'right'),
    ('NTC', ('NTC1',), (0.0, 9.6)),
)

#: How far a frame stands off the parts inside it, millimetres.
FRAME_MM = 1.0

#: A label's cell is about this wide in millimetres on an 88-cell board;
#: what a side placement steps a label clear of its frame by.
LABEL_STEP_MM = 2.5


def frame(refs):
    """`(cx, cy, hw, hh)`: the box round some parts' bodies, FRAME_MM
    out, millimetres from the board's centre and half-sizes."""
    left = right = top = bottom = None
    for ref in refs:
        x, y, w, h = PLACED[ref]
        cx, cy = from_pnp(x, y)
        left = cx - w / 2.0 if left is None else min(left, cx - w / 2.0)
        right = cx + w / 2.0 if right is None else max(right, cx + w / 2.0)
        bottom = cy - h / 2.0 if bottom is None else min(bottom, cy - h / 2.0)
        top = cy + h / 2.0 if top is None else max(top, cy + h / 2.0)
    left, right = left - FRAME_MM, right + FRAME_MM
    bottom, top = bottom - FRAME_MM, top + FRAME_MM
    return ((left + right) / 2.0, (bottom + top) / 2.0,
            (right - left) / 2.0, (top - bottom) / 2.0)


def label_at(box, where, label):
    """Where a label's centre goes, millimetres, for a frame `box` and a
    placement: `inside`, `above`, `below`, `left`, `right`, or a point."""
    if isinstance(where, tuple):
        return where
    cx, cy, hw, hh = box
    half = LABEL_STEP_MM * len(label) / 2.0
    return {'inside': (cx, cy),
            'above': (cx, cy + hh + LABEL_STEP_MM),
            'below': (cx, cy - hh - LABEL_STEP_MM),
            'right': (cx + hw + half + 1.0, cy),
            'left': (cx - hw - half - 1.0, cy)}[where]

#: Two characters a cell in the plain ramp, so pixels come out square. In
#: the halftone a cell is one character.
CELL = '  '

#: Plain-text ramp, coolest first. No leading space - a space means off the
#: board. Twenty steps, not nine: 120 K over nine characters is 13 K a step
#: and drew the board flat.
RAMP = '.,:;~-=+ic*xX#$%8W@'

#: The bar and its tick labels. Counted by the caller, so it is named.
#: Lines the scale spends BELOW the picture: none since 2026-08-30 -
#: it rides beside the board as a vertical rail, hottest at the top.
SCALE_LINES = 0

#: The halftone's range: the share of a cell's dots lit at the ramp's cold
#: end and at its hot end. THE COLOUR CARRIES THE TEMPERATURE and the dots
#: carry it again, so a hot zone reads hot with the palette off. Not down
#: to nothing: under four dots in ten a cool board dissolved into grain in
#: the raster, and the rim was the only shape left. NOT UP EITHER: five
#: was tried when the scale's cold end read black on the bench, and it
#: put an idle board at seven dots in ten - inside the range the
#: attitude page measured as a brick wall, every cell a block with dark
#: mortar round it (`wireframe.DENSITY_FLOOR`). The cold end's fix was
#: its colour, in `ansi.THERMAL_STOPS`.
DENSITY_COLD, DENSITY_HOT = 0.40, 1.0

#: How many ranks the mask has: a share of one clears every one of them.
NOISE_LEVELS = NOISE_N * NOISE_N

#: The rim, the outlines and the labels: white, which every stop of the
#: ramp is darker than. The amber the gauges mark with is a stop of the
#: ramp itself - 85 C - and an outline in it would vanish over a hot leg.
MARK_INK = ansi.WHITE

#: What a dot is, by geometry alone: off the board, the field's
#: halftone, or a MARK - the rim, the bore's edge, a frame - which its
#: cell draws alone, in MARK_INK. Two shapes before this, both on the
#: bench: a dot-wide outline over the stipple that filled each package,
#: "a faint frame"; then every package a solid block with a white edge,
#: "grey areas". A frame round the GROUP, its cells lit only where the
#: line runs, is what reads as a frame.
OFF, FIELD, MARK = 0, 1, 2


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
    # Even, because the halftone pairs field rows two to a character row.
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


def _fit(colour, reserve, margin=0):
    """Cells across the board, from the terminal.

    `reserve` is every line in the finished frame that is NOT picture: the
    banner, the readings, the blank lines, the scale and its labels. The
    caller counts its own, because guessing here is what clipped the bottom
    of the board off - a guess of 16 against a frame that spent 18.
    """
    size = shutil.get_terminal_size((80, 30))
    rows = max(size.lines - reserve, 8)
    columns = max(size.columns - 2 - margin, 20)
    if colour:
        wide, high = columns, rows * 2
    else:
        wide, high = columns // 2, rows

    # EVEN. The halftone draws two field rows per character row, so an odd
    # count leaves the last one unpaired. Rounded DOWN, so it still fits
    # what was measured to be free.
    return max(10, min(CELLS_MAX, min(wide, high))) // 2 * 2


def _density(celsius):
    """The share of a cell's dots a temperature lights, before the mask."""
    f = (celsius - ansi.THERMAL_MIN) / (ansi.THERMAL_MAX - ansi.THERMAL_MIN)
    f = 0.0 if f < 0.0 else (1.0 if f > 1.0 else f)
    return DENSITY_COLD + (DENSITY_HOT - DENSITY_COLD) * f


_MASKS = {}

#: A frame's lines in a cell's dots: the top across dot row 1, the
#: bottom across dot row 2, the sides down a lane - so the corners are
#: right angles, `⡖⠒⠒⢲` over `⠧⠤⠤⠼`, the bench's own glyphs ("braille
#: with just a border and right angles"). A frame sampled from its
#: millimetres landed its lines on whatever dot row the edge fell, and
#: the corners came out ragged.
FRAME_TOP, FRAME_BOTTOM = 1, 2


def _draw_frame(rows, box, cells, down, dx, dy):
    """A frame SNAPPED TO THE CELL GRID: the box's edges land in the
    cells that hold them and are drawn as lines through those cells'
    dots - FRAME_TOP and FRAME_BOTTOM across, a lane down - so every
    corner is a right angle and every side a straight run. Never less
    than two cells each way, so a small part still gets a box; a dot
    past the rim is left as it is, so a frame that reaches the rim
    stops there."""
    cx, cy, hw, hh = box
    wide, high = 2 * cells, 2 * down
    c0 = int(((cx - hw) / dx + (wide - 1) / 2.0) // 2)
    c1 = int(((cx + hw) / dx + (wide - 1) / 2.0) // 2)
    r0 = int(((high - 1) / 2.0 - (cy + hh) / dy) // 4)
    r1 = int(((high - 1) / 2.0 - (cy - hh) / dy) // 4)
    c0, c1 = max(0, c0), min(cells - 1, max(c1, c0 + 1))
    r0, r1 = max(0, r0), min(down // 2 - 1, max(r1, r0 + 1))

    def mark(r, c, lane, y):
        j, i = 4 * r + y, 2 * c + lane
        if 0 <= j < high and 0 <= i < wide and rows[j][i] != OFF:
            rows[j][i] = MARK

    for c in range(c0, c1 + 1):
        for lane in (0, 1):
            mark(r0, c, lane, FRAME_TOP)
            mark(r1, c, lane, FRAME_BOTTOM)
    for r in range(r0, r1 + 1):
        for y in range(4):
            # The corner starts AT the line, not above or below it.
            if (r == r0 and y < FRAME_TOP) or (r == r1 and y > FRAME_BOTTOM):
                continue
            mark(r, c0, 0, y)
            mark(r, c1, 1, y)


def _mask(cells, down, marks):
    """What every dot is by geometry alone - off the board, field, or a
    mark: the rim, the bore's edge, a frame's line - and which cells a
    label covers. Cached by size, because none of it moves between frames
    and the line test is eight frames a dot.

    The rim is ONE DOT wide - `edge` is half a dot pitch either side of
    the circle - which is what draws it as a line the dots follow rather
    than a band the cells step. A frame is drawn by `_draw_frame` on the
    cell grid, and one that reaches past the rim - the shunts sit at the
    terminals - stops at the rim.
    """
    key = (cells, down, marks)
    got = _MASKS.get(key)
    if got is not None:
        return got
    per_cell = 2.0 * OUTER_MM / cells
    per_row = 2.0 * OUTER_MM / down
    dx, dy = per_cell / 2.0, per_row / 2.0
    edge = 0.5 * max(dx, dy)
    bore = max(BORE_MM, per_cell)
    boxes = [frame(refs) for _label, refs, _where in marks]

    wide, high = 2 * cells, 2 * down
    rows = []
    for j in range(high):
        y = ((high - 1) / 2.0 - j) * dy
        line = []
        for i in range(wide):
            x = (i - (wide - 1) / 2.0) * dx
            r = math.hypot(x, y)
            if r > OUTER_MM or r < bore:
                line.append(OFF)
                continue
            line.append(MARK if (r > OUTER_MM - 2.0 * edge
                                 or r < bore + 2.0 * edge) else FIELD)
        rows.append(line)
    for box in boxes:
        _draw_frame(rows, box, cells, down, dx, dy)

    labels = {}
    for (label, refs, where), box in zip(marks, boxes):
        lx, ly = label_at(box, where, label)
        col = int((lx / dx + (wide - 1) / 2.0) // 2) - len(label) // 2
        row = int(((high - 1) / 2.0 - ly / dy) // 4)
        for k, ch in enumerate(label):
            labels[(row, col + k)] = ch
    got = (rows, labels)
    _MASKS[key] = got
    return got


def _braille_rows(grid, marks=MARKS):
    """Colour rows: one character a cell, its four dot rows over the two
    field rows of the grid, in the ramp's colour blended per cell.

    A DOT IS LIT BY THE MASK, not by its neighbours: the temperature under
    it sets a share and the blue-noise rank at its own screen position
    decides, so a flat region is an even stipple and a gradient a smooth
    one, with no structure at any density (`wireframe` has the history).
    The field is sampled per cell - a blob is eight millimetres wide and a
    cell one, so the dots within a cell share a temperature and only the
    mask varies between them.

    A CELL WITH A MARK IN IT DRAWS THE MARK ALONE, in MARK_INK: one
    colour a cell is what a terminal gives, and lighting the field's dots
    in that cell too made every line a cell wide and white - the "grey
    areas" and the thick rim the bench saw. Only the line's dots, and the
    line is a dot wide with a dot of dark beside it. A cell whose field
    centre is off the board can still hold rim dots, and draws them.
    """
    down, cells = len(grid), len(grid[0])
    mask, labels = _mask(cells, down, tuple(marks or ()))
    bit = BRAILLE_BITS
    levels = NOISE_LEVELS
    n = NOISE_N
    out = []
    for r in range(down // 2):
        top = grid[2 * r]
        low = grid[2 * r + 1] if 2 * r + 1 < down else [None] * cells
        mrows = mask[4 * r:4 * r + 4]
        nrows = [NOISE[(4 * r + y) % n] for y in range(4)]
        line = []
        for c in range(cells):
            label = labels.get((r, c))
            if label is not None:
                line.append((label, MARK_INK))
                continue
            t0, t1 = top[c], low[c]
            if t0 is None:
                t0 = t1
            if t1 is None:
                t1 = t0
            if t0 is None:
                share0 = share1 = -1.0      # off the field: marks only
            else:
                share0 = _density(t0) * levels
                share1 = _density(t1) * levels
            marks, dots = 0, 0
            for lane in (0, 1):
                i = 2 * c + lane
                ni = i % n
                for y in range(4):
                    m = mrows[y][i]
                    if m == OFF:
                        continue
                    if m == MARK:
                        marks |= bit[lane][y]
                    elif (share0 if y < 2 else share1) > nrows[y][ni] + 0.5:
                        dots |= bit[lane][y]
            if marks:
                line.append((chr(BRAILLE + marks), MARK_INK))
            elif dots:
                line.append((chr(BRAILLE + dots),
                             ansi.thermal_rgb(int((t0 + t1) / 2.0 + 0.5))))
            else:
                line.append((' ', None))
        out.append(ansi.run(line))
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
           reserve=None, trailing=2, aspect=CELL_ASPECT, margin=0,
           marks=MARKS):
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
    and the character ramp reads fine there. `marks` is what the halftone
    outlines and labels; None draws the field alone.
    """
    if colour is None:
        colour = bool(getattr(sys.stdout, 'isatty', lambda: False)())
    if cells is None:
        # SCALE_LINES plus the blank above them is what render itself adds;
        # anything else in the frame is the caller's to count.
        cells = _fit(colour, (SCALE_LINES + trailing)
                     if reserve is None else reserve,
                     margin + RAIL_W)
    layout = LAYOUT if layout is None else layout

    grid, lo, _hi = _grid(nodes, board_c, cells, layout, aspect)
    if lo is None:
        return 'nothing to draw'

    out = []
    if title:
        out.extend(['  ' + title, ''])
    art = _braille_rows(grid, marks) if colour else _ramp_rows(grid)
    rail = _rail(len(art), colour)
    out.extend(row + '  ' + tag for row, tag in zip(art, rail))
    out.extend([''] * trailing)
    return '\n'.join(out)


#: Columns the vertical scale spends right of the board: two of block,
#: a space, and the widest label (' -20 C').
RAIL_W = 9


def _swatch(celsius, row, col):
    """One cell of the rail: the field's own halftone at this temperature,
    through the mask at this screen position - so the rail is the legend
    for the dots as well as for the colour."""
    share = _density(celsius) * NOISE_LEVELS
    n = NOISE_N
    bits = 0
    for lane in (0, 1):
        for y in range(4):
            if share > NOISE[(4 * row + y) % n][(2 * col + lane) % n] + 0.5:
                bits |= BRAILLE_BITS[lane][y]
    return chr(BRAILLE + bits)


def _rail(rows, colour):
    """The temperature scale as a column beside the board, hottest at the
    top. It spends width, which a round board has spare, instead of the
    rows it does not - the horizontal bar below cost the picture two
    lines at every terminal height. Blended like the field, so the rail
    is the ramp and not a stack of its stops, and IN BRAILLE like the
    field - two cells of the same halftone the board wears at that
    temperature, on the bench's word ("the temperature scale in braille
    too"). It was two background-painted spaces, a solid bar beside a
    dotted board."""
    lo, hi = ansi.THERMAL_MIN, ansi.THERMAL_MAX
    marks = {}
    for t in (100, 80, 60, 40, 20, 0, -20):
        at = int(round((hi - t) / float(hi - lo) * (rows - 1)))
        marks.setdefault(at, ' %d C' % t)
    out = []
    for r in range(rows):
        t = hi - (hi - lo) * (r / float(rows - 1) if rows > 1 else 0.0)
        if colour:
            block = ansi.run([(_swatch(t, r, 0) + _swatch(t, r, 1),
                               ansi.thermal_rgb(t))])
        else:
            block = RAMP[int((t - lo) / float(hi - lo)
                             * (len(RAMP) - 1))] * 2
        out.append(block + marks.get(r, ''))
    return out
