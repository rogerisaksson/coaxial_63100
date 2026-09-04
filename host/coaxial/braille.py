"""Every pattern in U+2800, and the vocabulary to ask for one.

Its own file because the ALPHABET has callers and no owner. `machine`
draws a rotor, `dial` a protractor, `wireframe` a board and the views
draw leaders between them, and each was picking glyphs by hand -
`chr(0x2824)` for a run, `chr(0x2847)` for a drop, `chr(0x28A4)` for the
corner where they meet. Hand-picked, the set stays small and the corners
come out wrong: a run of `⠤` ending against a `⡇` is two marks that
happen to touch, and nobody notices until the drawing is read closely.

The block has 256 patterns and a drawing should be able to reach all of
them by what they MEAN. So: a cell is eight dots in two lanes of four,
`glyph()` turns a set of them into a character, and the line-drawing
names on top of it - `row`, `column`, `corner`, `tee` - are the ones a
leader actually needs.

Dot numbering is the braille standard's, and `raster.BRAILLE_BITS` is
the same table indexed `[lane][y]`::

    lane 0  lane 1        y
      1       4           0
      2       5           1
      3       6           2
      7       8           3
"""
from .raster import BRAILLE, BRAILLE_BITS, DOTS_X, DOTS_Y

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
    """The character for dot NUMBERS, the way a braille chart names them.

    `numbered(2, 5, 6)` is `⠲`, which is how the bench asks for a corner.
    """
    return glyph(AT[n] for n in dots if n in AT)


def lit(char):
    """The `(lane, y)` dots a character carries, for reading one back."""
    bits = ord(char) - BRAILLE
    return tuple((lane, y) for lane in range(DOTS_X) for y in range(DOTS_Y)
                 if bits & BIT[lane][y])


def row(y, lanes=(0, 1)):
    """A horizontal stroke along dot row `y`, across `lanes`.

    `row(2)` is `⠤`, the dotted run a leader is drawn with; `row(0)` is
    `⠉` and `row(3)` is `⣀`, which is where the same line lands when it
    has to sit above or below what it names.
    """
    return glyph((lane, y) for lane in lanes)


def column(lane, ys=range(DOTS_Y)):
    """A vertical stroke down `lane`, over dot rows `ys`.

    `column(0)` is `⡇` and `column(1)` is `⢸` - the same line falling
    down the near or the far half of its cell, which is what keeps a
    leader from doubling back over the corner it just turned.
    """
    return glyph((lane, y) for y in ys)


def corner(y, lane, up=False, through=False):
    """Where a horizontal on dot row `y` turns vertical in `lane`.

    `up` turns toward the top of the cell instead of the bottom.
    `through` carries the vertical to the cell's edge, for a line that
    CONTINUES into the next row; without it the stroke stops two dots
    along and reads as a hook that ends here.

    Which one a drawing wants depends on what is under it, and getting it
    wrong is visible: a hook where the line goes on breaks against the
    row below, and a stroke to the edge where it stops reads as a post
    the line happens to end at.

        corner(1, 1)                 U+2832  the run arrives and hooks down
        corner(1, 1, through=True)   U+28B2  and this one carries on
        corner(2, 0, through=True)   U+2856  the same turn, the other lane
    """
    reach = (0 if through else y - 1) if up else (DOTS_Y - 1 if through
                                                 else y + 1)
    lo, hi = (reach, y) if up else (y, reach)
    return glyph([(other, y) for other in range(DOTS_X)]
                 + [(lane, step) for step in range(max(0, lo),
                                                   min(DOTS_Y - 1, hi) + 1)])


def tee(y, lane):
    """A horizontal on dot row `y` met by a full-height stroke in `lane`.

    The junction where a line does not turn but arrives - `⢹` and `⡏`.
    """
    return glyph([(other, y) for other in range(DOTS_X)]
                 + [(lane, step) for step in range(DOTS_Y)])


#: The dotted horizontal a leader runs along, one per dot row.
RUN = tuple(row(y) for y in range(DOTS_Y))

#: The line a leader falls down, one per lane.
FALL = tuple(column(lane) for lane in range(DOTS_X))
