"""Every pattern in U+2800, and the vocabulary to ask for one."""
from .graphics.raster import BRAILLE, BRAILLE_BITS, DOTS_X, DOTS_Y

#: Every cell, indexed by its bit mask. All 256 of them, so a caller that
#: has computed a mask never has to know the offset.
ALL = tuple(chr(BRAILLE + bits) for bits in range(256))

#: The mask of one dot, `[lane][y]` - `raster`'s table under the name
#: this module uses for a column of a cell.
BIT = BRAILLE_BITS

#: The dot numbers, `[lane][y]`, for a caller who thinks in the standard's
#: numbering rather than in coordinates.
NUMBER = ((1, 2, 3, 7), (4, 5, 6, 8))

#: Where each dot number sits, `{number: (lane, y)}`.
AT = {NUMBER[lane][y]: (lane, y)
      for lane in range(DOTS_X) for y in range(DOTS_Y)}


def mask(cells):
    """The bit mask for an iterable of `(lane, y)` dots."""
    bits = 0
    for lane, y in cells:
        if 0 <= lane < DOTS_X and 0 <= y < DOTS_Y:
            bits |= BIT[lane][y]
    return bits


def glyph(cells):
    """The character for an iterable of `(lane, y)` dots."""
    return ALL[mask(cells)]


def numbered(*dots):
    """The character for dot NUMBERS, the way a braille chart names them."""
    return glyph(AT[n] for n in dots if n in AT)


def lit(char):
    """The `(lane, y)` dots a character carries, for reading one back."""
    bits = ord(char) - BRAILLE
    return tuple((lane, y) for lane in range(DOTS_X) for y in range(DOTS_Y)
                 if bits & BIT[lane][y])


def row(y, lanes=(0, 1)):
    """A horizontal stroke along dot row `y`, across `lanes`."""
    return glyph((lane, y) for lane in lanes)


def column(lane, ys=range(DOTS_Y)):
    """A vertical stroke down `lane`, over dot rows `ys`."""
    return glyph((lane, y) for y in ys)


def corner(y, lane, up=False, through=False):
    """Where a horizontal on dot row `y` turns vertical in `lane`."""
    reach = (0 if through else y - 1) if up else (DOTS_Y - 1 if through
                                                 else y + 1)
    lo, hi = (reach, y) if up else (y, reach)
    return glyph([(other, y) for other in range(DOTS_X)]
                 + [(lane, step) for step in range(max(0, lo),
                                                   min(DOTS_Y - 1, hi) + 1)])


def tee(y, lane):
    """A horizontal on dot row `y` met by a full-height stroke in `lane`."""
    return glyph([(other, y) for other in range(DOTS_X)]
                 + [(lane, step) for step in range(DOTS_Y)])


#: The dotted horizontal a leader runs along, one per dot row.
RUN = tuple(row(y) for y in range(DOTS_Y))

#: The line a leader falls down, one per lane.
FALL = tuple(column(lane) for lane in range(DOTS_X))
