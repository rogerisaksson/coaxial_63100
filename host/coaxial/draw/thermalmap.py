"""The board as a thermal picture: a braille halftone in the thermal ramp.

With the rim and the parts that make the heat drawn on it.
"""
import math
import shutil
import sys

from coaxial.graphics.raster import BRAILLE, BRAILLE_BITS, NOISE, NOISE_N
from machine import ansi

#: Board dimensions, RADII in millimetres: 100 mm across with a 10 mm bore,
#: confirmed 2026-08-29. `electronics/` is the authority on the rest.
OUTER_MM = 50.0
BORE_MM = 5.0

#: The finest field grid: cost is O(cells^2) evaluations - 88 cells, 7744
#: points, 40 ms. At 44 the circle stepped; the halftone's dots are finer
#: than the field either way.
CELLS_MAX = 88

#: A field row's height against a cell's width on the screen: a terminal
#: character is ~9 x 20 px, so square cells drew the board an oval. Applied
#: to the field's row spacing; the font's, not the board's (half the aspect
#: `screen.aspect_of` measures).
CELL_ASPECT = 1.10

#: The bore's least width on the plain ramp, cells: 5 mm lands on one cell,
#: a dent. A drawing concession (BORE_MM is the board's); the halftone
#: resolves 5 mm on its own. Size is the only knob - superellipses at three
#: exponents changed nothing:
#:
#:   up to 2.0 cells   2-4-4-2         the discrete circle, and the smallest
#:   2.4               4-4-4-4         a square
#:   3.2               4-6-6-6-6-4     an octagon
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
    'U12':  (129.2097, 73.9139, 3.0, 3.0),    # LM5069, VSSOP-10: the hot
    'Q3':   (120.4214, 69.2150, 6.2, 5.2),    # swap, its back-to-back
    'Q4':   (121.6914, 75.9460, 6.2, 5.2),    # FETs - the bridge's part -
    'RTS1': (137.0330, 76.9366, 3.0, 10.1),   # and the fuse, Schurter UMT,
    'V1':   (136.8806, 67.6656, 3.2, 2.5),    # and the varistor, 1210
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


#: The heat sources, (x, y, sigma) mm per zone: sigma is how wide the blob is
#: laid (copper spreads wider than the part). A leg is a blob for its
#: switches and one for its shunts, and `field` takes the strongest point;
#: the middle driver is the NTC's neighbour.
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

#: How far a frame stands off the parts inside it, millimetres.
FRAME_MM = 1.0

#: The picture's marks: a label, the parts the frame goes round, where the
#: label sits (`bottom` writes it into the frame's bottom line, `⠧⠤MCU⠤⠼`),
#: and the frame's stand-off, mm. REG is the two bucks and two LDOs left of
#: the MCU, whose frame stands 3 mm off the package; each phase takes its
#: shunts at the rim; HS runs from the bore's edge to the rim (FETs 14-15 mm,
#: controller 23, fuse and varistor 31, terminals 40). All on the bench's
#: word.
MARKS = (
    ('MCU', ('U3',), 'bottom', 3.0),
    ('REG', ('U8', 'U9', 'U1', 'U7'), 'bottom', FRAME_MM),
    ('U', ('Q1U', 'Q2U', 'RU1', 'RU2'), 'bottom', FRAME_MM),
    ('V', ('Q1V', 'Q2V', 'RV1', 'RV2'), 'bottom', FRAME_MM),
    ('W', ('Q1W', 'Q2W', 'RW1', 'RW2'), 'bottom', FRAME_MM),
    ('AFE', ('OP1U', 'OP2U', 'OP1V', 'OP2V', 'OP1W', 'OP2W'), 'bottom',
     FRAME_MM),
    ('HS', ('U12', 'Q3', 'Q4', 'RTS1', 'V1'), 'bottom', FRAME_MM),
    ('NTC', ('NTC1',), (0.0, 9.6), FRAME_MM),
)

#: A label's cell is about this wide in millimetres on an 88-cell board;
#: what a side placement steps a label clear of its frame by.
LABEL_STEP_MM = 2.5


def frame(refs, margin=FRAME_MM):
    """`(cx, cy, hw, hh)`: the box round some parts' bodies, `margin`
    out, millimetres from the board's centre and half-sizes."""
    left = right = top = bottom = None
    for ref in refs:
        x, y, w, h = PLACED[ref]
        cx, cy = from_pnp(x, y)
        left = cx - w / 2.0 if left is None else min(left, cx - w / 2.0)
        right = cx + w / 2.0 if right is None else max(right, cx + w / 2.0)
        bottom = cy - h / 2.0 if bottom is None else min(bottom, cy - h / 2.0)
        top = cy + h / 2.0 if top is None else max(top, cy + h / 2.0)
    if left is None or right is None or top is None or bottom is None:
        raise ValueError('a frame round no parts')
    left, right = left - margin, right + margin
    bottom, top = bottom - margin, top + margin
    return ((left + right) / 2.0, (bottom + top) / 2.0,
            (right - left) / 2.0, (top - bottom) / 2.0)


def label_at(box, where, label):
    """Where a label's centre goes, millimetres, for a frame `box` and a
    placement: `bottom` or `top` ON the frame's line, `inside`, `above`,
    `below`, `left`, `right` beside it, or a point."""
    if isinstance(where, tuple):
        return where
    cx, cy, hw, hh = box
    half = LABEL_STEP_MM * len(label) / 2.0
    return {'inside': (cx, cy),
            'bottom': (cx, cy - hh),
            'top': (cx, cy + hh),
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

#: The halftone's range, the share of a cell's dots lit at the ramp's cold and
#: hot ends; the dots carry the temperature with the palette off. Under 0.4 a
#: cool board dissolved into grain; 0.5 put an idle board in the brick-wall
#: range (`shading.DENSITY_FLOOR`). The cold end's fix was its colour
#: (`ansi.THERMAL_STOPS`).
DENSITY_COLD, DENSITY_HOT = 0.40, 1.0

#: How many ranks the mask has: a share of one clears every one of them.
NOISE_LEVELS = NOISE_N * NOISE_N

#: Rim, outlines and labels in white: every ramp stop is darker (amber is the
#: ramp's 85 C and vanishes over a hot leg).
MARK_INK = ansi.WHITE

#: A dot by geometry alone: off the board, the field's halftone, or a mark
#: (rim, bore edge, frame) drawn alone in MARK_INK. A frame round the group,
#: lit only where its line runs, reads as a frame; a dot-wide outline over
#: each package read "faint", solid packages "grey areas".
OFF, FIELD, MARK = 0, 1, 2


#: Each laminate patch's centre, mm from the board's, off the model's area
#: raster, and the blend's width: the laminate under a point is the patches'
#: temperatures weighted by a Gaussian of distance - one patch per point drew
#: a step where the copper has a gradient.
PATCH_CENTRE = {'board': (0.0, -6.8), 'patch_u': (-27.6, 26.1),
                'patch_v': (0.0, 30.7), 'patch_w': (27.6, 26.1),
                'patch_left': (-35.3, -5.9), 'patch_bottom': (0.0, -35.3),
                'patch_right': (35.3, -5.9)}
PATCH_BLEND_MM = 14.0


def laminate_at(x_mm, y_mm, nodes, board_c):
    """The laminate under a point: the patches' temperatures blended by
    distance where the observer reports patches, the bulk `board_c`
    where an older firmware reports none."""
    weight, total = 0.0, 0.0
    two_sigma_sq = 2.0 * PATCH_BLEND_MM * PATCH_BLEND_MM
    for name, (cx, cy) in PATCH_CENTRE.items():
        value = nodes.get(name)
        if value is None:
            continue
        d2 = (x_mm - cx) ** 2 + (y_mm - cy) ** 2
        w = math.exp(-d2 / two_sigma_sq)
        weight += w
        total += w * value
    return total / weight if weight > 1e-9 else board_c


def field(x_mm, y_mm, board_c, nodes, layout=None):
    """Temperature at one point: the laminate under it plus every source's
    contribution.
    """
    layout = LAYOUT if layout is None else layout
    got = laminate_at(x_mm, y_mm, nodes, board_c)
    for name, spots in layout.items():
        value = nodes.get(name)
        if value is None:
            continue
        # A source's rise is over the laminate it sits on - its patch where
        # there is one - not over the bulk.
        over = value - got
        if abs(over) < 1e-6:
            continue
        # STRONGEST point in the zone, not the sum of them.
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

    # THE GRID IS NOT SQUARE, and that is the point.
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
    """Cells across the board, from the terminal."""
    size = shutil.get_terminal_size((80, 30))
    rows = max(size.lines - reserve, 8)
    columns = max(size.columns - 2 - margin, 20)
    if colour:
        wide, high = columns, rows * 2
    else:
        wide, high = columns // 2, rows

    # EVEN.
    return max(10, min(CELLS_MAX, min(wide, high))) // 2 * 2


def _density(celsius):
    """The share of a cell's dots a temperature lights, before the mask."""
    f = (celsius - ansi.THERMAL_MIN) / (ansi.THERMAL_MAX - ansi.THERMAL_MIN)
    f = 0.0 if f < 0.0 else (1.0 if f > 1.0 else f)
    return DENSITY_COLD + (DENSITY_HOT - DENSITY_COLD) * f


_MASKS = {}

#: A frame's dot rows: the top across row 1, the bottom across row 2, the
#: sides down a lane - right-angled corners, `⡖⠒⠒⢲` over `⠧⠤⠤⠼` (bench).
#: Sampled from millimetres the corners came out ragged.
FRAME_TOP, FRAME_BOTTOM = 1, 2


def _cell_rect(box, cells, down, dx, dy):
    """`[c0, c1, r0, r1]`: the cells a frame's edges land in."""
    cx, cy, hw, hh = box
    wide, high = 2 * cells, 2 * down
    half = (wide - 1) / 2.0

    def dot(offset_dots):
        # Half away from the centre, so a mirrored offset lands on the mirrored
        # dot: the edge's column, 0 ..
        rounded = math.floor(abs(offset_dots) + 0.5)
        at = half + (rounded if offset_dots >= 0.0 else -rounded)
        return int(max(0, min(wide - 1, at)))

    left, right = dot((cx - hw) / dx), dot((cx + hw) / dx)
    top = (high - 1) / 2.0 - (cy + hh) / dy
    bottom = (high - 1) / 2.0 - (cy - hh) / dy
    c0, c1 = left // 2, right // 2
    r0 = int(top // 4)
    r1 = (down // 2 - 1) - int(((high - 1) - bottom) // 4)
    rect = [max(0, c0), min(cells - 1, max(c1, c0 + 1)),
            max(0, r0), min(down // 2 - 1, max(r1, r0 + 1))]
    lanes = [left % 2, right % 2 if c1 > c0 else 1]
    return rect, lanes


def _share_edges(rects, lanes):
    """Two frames side by side SHARE THE LINE between them: where one's
    right edge and the other's left land within a cell column of each
    other, the second is drawn on the first's column, in the first's lane
    - one line, not two a dot apart into a solid column, nor two a cell
    apart.
    """
    for a, (ac0, ac1, ar0, ar1) in enumerate(rects):
        for b, (bc0, bc1, br0, br1) in enumerate(rects):
            if a == b or not (ar0 <= br1 and br0 <= ar1):
                continue
            if abs(bc0 - ac1) <= 1 and bc1 > ac1:
                rects[b][0] = ac1
                lanes[b][0] = lanes[a][1]


def _draw_frame(rows, rect, lane, cells, down):
    """A frame SNAPPED TO THE CELL GRID: drawn as lines through its cells'
    dots - FRAME_TOP and FRAME_BOTTOM across, a lane down each side - so
    every corner is a right angle and every side a straight run.
    """
    c0, c1, r0, r1 = rect
    wide, high = 2 * cells, 2 * down

    def mark(r, c, which, y):
        j, i = 4 * r + y, 2 * c + which
        if 0 <= j < high and 0 <= i < wide and rows[j][i] != OFF:
            rows[j][i] = MARK

    for c in range(c0, c1 + 1):
        for which in (0, 1):
            # THE LINE STARTS AT THE SIDE'S LANE, not at the corner cell's
            # edge.
            if (c == c0 and which < lane[0]) or (c == c1 and which > lane[1]):
                continue
            mark(r0, c, which, FRAME_TOP)
            mark(r1, c, which, FRAME_BOTTOM)
    for r in range(r0, r1 + 1):
        for y in range(4):
            # The corner starts AT the line, not above or below it.
            if (r == r0 and y < FRAME_TOP) or (r == r1 and y > FRAME_BOTTOM):
                continue
            mark(r, c0, lane[0], y)
            mark(r, c1, lane[1], y)


def _mask(cells, down, marks):
    """What every dot is by geometry alone - off the board, field, or a
    mark: the rim, the bore's edge, a frame's line - and which cells a
    label covers.
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
    boxes = [frame(refs, margin) for _label, refs, _where, margin in marks]

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
    placed = [_cell_rect(box, cells, down, dx, dy) for box in boxes]
    rects = [rect for rect, _lanes in placed]
    lanes = [lanes for _rect, lanes in placed]
    _share_edges(rects, lanes)
    for rect, lane in zip(rects, lanes):
        _draw_frame(rows, rect, lane, cells, down)

    labels = {}
    for (label, _refs, where, _margin), box in zip(marks, boxes):
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
            if t0 is None or t1 is None:
                share0 = share1 = -1.0      # off the field: marks only
                mid = 0
            else:
                share0 = _density(t0) * levels
                share1 = _density(t1) * levels
                mid = int((t0 + t1) / 2.0 + 0.5)
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
                line.append((chr(BRAILLE + dots), ansi.thermal_rgb(mid)))
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
    """The board as a thermal picture."""
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
    top.
    """
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
