"""What every character renderer here needs: the dot matrix, and which
cell a coordinate lands in.

Its own file because these have callers and no owner - `orientation`
draws an attitude, `dial` a shaft, `wireframe` the board and `machine`
the rotor, and neither rounding nor the braille alphabet belongs to any
one of them.
"""
import math

#: The pixels: braille, a 2x4 dot matrix per cell, so a drawing rasters
#: at twice the column and four times the row resolution and reads as a
#: fine dotted line - the same dots the chat page's spinner spins, asked
#: for by the bench as "pixels". Dots only: an ASCII stroke fallback
#: (`- | / \`) was built and taken out on the bench's word - the slashes
#: read as jank next to the dots, and the console that runs this already
#: shows the spinner.
BRAILLE = 0x2800
#: Bit for (column, row) of the 2x4 cell, the braille standard's order.
BRAILLE_BITS = ((0x01, 0x02, 0x04, 0x40), (0x08, 0x10, 0x20, 0x80))

#: How many dots a cell is, across and down. A character cell is about
#: one wide by two tall and a braille cell is two dots by four, so A DOT
#: IS SQUARE and a circle drawn in them is round without correction.
DOTS_X, DOTS_Y = 2, 4

#: The four corners a dot is sampled at, in dot units. SUPERSAMPLED
#: BECAUSE A DOT IS NOT A POINT: one sample at a dot's centre lights the
#: dot where a ring passes through that point and not where it passes
#: either side, and both round pictures in this tree came out as dashed
#: circles until they sampled the corners instead.
SUBDOT = ((-0.25, -0.25), (0.25, -0.25), (-0.25, 0.25), (0.25, 0.25))


#: The order a cell's eight dots light up in, dispersed rather than
#: filled: a 2x2 Bayer matrix tiled down the cell, so two dots are one in
#: each half and not two side by side. Filled in reading order a ladder
#: reads as a bar growing out of a corner, which is a bar chart and not a
#: tone.
DOT_RANK = ((0, 6, 1, 7), (4, 2, 5, 3))


def _spread(bits):
    """How scattered one pattern's dots are: adjacent lit pairs, then the
    dispersed order as a tie-break. Lower is more even."""
    lit = [(x, y) for x in range(DOTS_X) for y in range(DOTS_Y)
           if bits & BRAILLE_BITS[x][y]]
    touching = sum(1 for i, a in enumerate(lit) for b in lit[i + 1:]
                   if abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1)
    return (touching, sum(DOT_RANK[x][y] for x, y in lit))


#: The tone ladder, `SHADE[rung][phase]`: nine rungs by how many of the
#: cell's eight dots are lit, and under each rung EVERY pattern with that
#: many - 1, 8, 28, 56, 70, 56, 28, 8, 1, which is all 256 of U+2800.
#: Ordered within a rung by how evenly the dots are spread, so phase 0 is
#: the smoothest arrangement and the rest are there for grain.
#:
#: WHY THE WHOLE BLOCK AND NOT THREE GLYPHS. An ASCII render carries its
#: 3D in the characters, and ' .:' has two above blank - so a surface
#: leaning away had one step to fall through and a board came out as a
#: flat carpet with a rim. Eight dots in the same cell is a nine-step
#: ladder, and the PHASE spends the rest of the block: two cells at one
#: level can wear different dots, which breaks the banding a single
#: pattern per level draws across a shallow gradient. The count is the
#: tone; which dots carry it is not.
SHADE = tuple(
    tuple(chr(BRAILLE + bits) for bits in
          sorted((b for b in range(256) if bin(b).count('1') == rung),
                 key=_spread))
    for rung in range(9))

#: How many rungs the ladder has above blank.
RUNGS = len(SHADE) - 1


#: An ordered threshold per dot, a 4x4 Bayer over the DOT grid. What it
#: is for: a shape covering a quarter of a dot used to light it whole,
#: because a dot is one bit and the four corners `SUBDOT` samples were
#: read as "any". Every arc in a drawing came out a dot fatter than it
#: is and every edge stepped. Against this, a quarter-covered dot lights
#: at a quarter of the positions - which is the fringe drawn in the
#: patterns between solid and blank, and the reason the block has 256 of
#: them.
BAYER4 = ((0, 8, 2, 10), (12, 4, 14, 6), (3, 11, 1, 9), (15, 7, 13, 5))


def dithered(hits, of, x, y):
    """Whether a dot `hits` of `of` samples deep lights, at (x, y).

    HALF A DOT OR MORE ALWAYS LIGHTS. Only the outermost fringe is
    dithered: a rim, a coil wire or a tooth edge one dot wide is a line
    the drawing means, and thresholding all of it would break it into
    dashes. What this softens is the quarter-covered dot beyond that.
    """
    if hits * 2 >= of:
        return hits > 0
    return (hits > 0
            and (BAYER4[y % 4][x % 4] + 0.5) / 16.0 < float(hits) / of)


def shade(level, phase=0):
    """One cell of tone: `level` 0 to 1 up the ladder, blank at zero.

    `phase` picks among the patterns of that density - any stable
    per-cell number will do, and the renderers hand it the grain hash
    they already keep.
    """
    rung = int(level * RUNGS + 0.5)
    rung = 0 if rung < 0 else (RUNGS if rung > RUNGS else rung)
    row = SHADE[rung]
    return row[phase % len(row)]


def cell(value):
    """`value` to the nearest cell, halves always upward.

    Not round(): Python rounds halves to even, so 15.5 and 16.5 both land on
    16 and two adjacent positions collide. Measured, that wrote a label as
    "o x a  3 0 0".
    """
    return int(math.floor(value + 0.5))
