"""What every character renderer here needs: the dot matrix, and which cell
a coordinate lands in.
"""
import math
import os
import struct

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

#: Where a dot is sampled, in dot units. SUPERSAMPLED BECAUSE A DOT IS
#: NOT A POINT: one sample at a dot's centre lights the dot where a ring
#: passes through that point and not where it passes either side, and
#: both round pictures in this tree came out as dashed circles until they
#: sampled more than the middle.
#:
#: A ROTATED GRID, NOT THE FOUR CORNERS. Four samples on a square give a
#: near-horizontal or near-vertical edge only TWO distinct coverages -
#: both samples of a row cross it at once - which is exactly where a
#: circle looks worst, at its top, bottom and sides. The four-rooks
#: pattern puts one sample in each row and each column of a 4x4 grid, so
#: those edges get five, and the diagonals of the can come out `⢠⢾⠋`
#: where the corners drew `⢠⠞⠋`. Same four samples, same cost.
SUBDOT = ((-0.375, -0.125), (0.125, -0.375),
          (-0.125, 0.375), (0.375, 0.125))


#: The order a cell's eight dots light up in, dispersed rather than
#: filled: a 2x2 Bayer matrix tiled down the cell, so two dots are one in
#: each half and not two side by side. Filled in reading order a ladder
#: reads as a bar growing out of a corner, which is a bar chart and not a
#: tone.
DOT_RANK = ((0, 6, 1, 7), (4, 2, 5, 3))


def _spread(bits):
    """How scattered one pattern's dots are: adjacent lit pairs, then the
    dispersed order as a tie-break.
    """
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


def _bluenoise():
    """The threshold mask beside this module - `tools/render/bluenoise.py`'s
    output, 64 x 64 ranks 0..4095 by void-and-cluster - as rows.
    """
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'bluenoise64.bin')
    with open(path, 'rb') as f:
        data = f.read()
    n = int(round(math.sqrt(len(data) // 2)))
    flat = struct.unpack('<%dH' % (n * n), data)
    return n, [flat[y * n:(y + 1) * n] for y in range(n)]


#: THE HALFTONE MASK, shared: a dot is lit where the value at its own
#: position clears the rank at its own screen position, so a surface is a
#: stipple with no structure at any density. Laid over the DOTS - 64 by
#: 64, thirty-two cells wide and sixteen tall - and fixed in screen space,
#: so a board turning under it moves the density and not the dots. The
#: attitude's face and the thermal picture both draw through it;
#: `wireframe` has the history of what it replaced.
NOISE_N, NOISE = _bluenoise()


def covered(hits, of, x=0, y=0):
    """Whether a dot the shape covers `hits` of `of` samples deep lights."""
    return hits * 2 >= of


def shade(level, phase=0):
    """One cell of tone: `level` 0 to 1 up the ladder, blank at zero."""
    rung = int(level * RUNGS + 0.5)
    rung = 0 if rung < 0 else (RUNGS if rung > RUNGS else rung)
    row = SHADE[rung]
    return row[phase % len(row)]


def cell(value):
    """`value` to the nearest cell, halves always upward."""
    return int(math.floor(value + 0.5))


#: Sample tables a page's sizes are worth keeping: a resize builds
#: another, a runaway set clears them.
TABLES_KEPT = 8


def table(kept, key, build):
    """The table for `key` in `kept`, built by `build()` on first ask."""
    got = kept.get(key)
    if got is None:
        if len(kept) > TABLES_KEPT:
            kept.clear()
        got = kept[key] = build()
    return got
