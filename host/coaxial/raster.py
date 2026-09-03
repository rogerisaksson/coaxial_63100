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


def cell(value):
    """`value` to the nearest cell, halves always upward.

    Not round(): Python rounds halves to even, so 15.5 and 16.5 both land on
    16 and two adjacent positions collide. Measured, that wrote a label as
    "o x a  3 0 0".
    """
    return int(math.floor(value + 0.5))
