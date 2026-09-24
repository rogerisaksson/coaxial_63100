"""The rotor observer's geometry: the box, the fit, the columns and the glyphs the parts share."""
import math

from coaxial.draw import machine
from terminal.ui.scroll import HUD_WIDTH


#: The drawing's height before `fit` sizes it to the terminal. The can is
#: sized against its own band, not the height, so rows come off without it
#: running into the foot gauges: 28 dots of radius hold down to 24 rows.
NOMINAL_HEIGHT = 24

#: Caption rows above the drawing: the two margins first, then the gutter
#: groups' names and hottest readings beside the NTC. One foot row: WINDING,
#: the thermal observer's policy, POWER (bench 2026-09-05).
CAPTION_ROWS, FOOT_ROWS = 5, 1


#: What is left for the machine. No inset at the foot: at 19 rows with one
#: the winding gauge was drawn through the can (measured). Can 2..16,
#: winding 17, watts 18, labels below the box.
class Box:
    """The machine's drawing this frame: its columns, the braille rows the
    art gets, and the rows the page spends on it with the captions and
    the foot.
    """
    def __init__(self, width, height):
        self.width, self.height = width, height
        self.rows = height - CAPTION_ROWS - FOOT_ROWS


#: Rows above the machine (none: the leaders' corner glyph turns them down)
#: and below it (the two floor gauges); `fit` adds them to the can's rows.
HOP_ROWS, FLOOR_GAUGES = 0, 2

#: `stage.frame_of`'s furniture round the drawing: the viewport's frame and
#: padding, 4 columns; the title band, key bar and frame edges, 4 rows.
VIEWPORT_COLUMNS = 4

PAGE_ROWS = 4

#: The narrowest drawing and lowest band on any terminal: below them the
#: gutters' names run into the legend. 52 wide (from 46, the bench's "a
#: shade bigger") is also what a piped run draws.
NOMINAL_WIDTH = 52

MIN_WIDTH = 40

MIN_BAND = 8

BOX = Box(NOMINAL_WIDTH, NOMINAL_HEIGHT)


def _width_for(can):
    """The columns a can of `can` dots needs with the gutters beside it:
    `machine.layout` inverted, so a machine bound by the rows is not left
    in the middle of a wide box with its thermometers at the far edges."""
    room = 2.0 * (can / machine.F_FIT + 1.0)
    lead = LEFT_COLUMNS + machine.BAR_GAP
    trail = RIGHT_COLUMNS + machine.BAR_GAP
    return int(math.ceil(room / machine.DOTS_X)) + lead + trail


def fit(aspect, size=None):
    """Size the box to the can on THIS terminal - and, given the console's
    `size`, to the terminal as it is NOW: the width the page leaves
    beside the instrument column, the band it leaves under the captions,
    and the can the smaller of the two allows, the gutters drawn in
    against it.
    """
    stretch = aspect / machine.DOTS_Y * machine.DOTS_X
    width = (NOMINAL_WIDTH if size is None else
             max(MIN_WIDTH, size.width - HUD_WIDTH - VIEWPORT_COLUMNS))
    band = (10 ** 6 if size is None else
            max(MIN_BAND, size.height - PAGE_ROWS - CAPTION_ROWS - FOOT_ROWS
                - HOP_ROWS - FLOOR_GAUGES))
    can = machine.layout(width, band, LEFT_COLUMNS, RIGHT_COLUMNS,
                         stretch=stretch)[1].can
    width = min(width, max(MIN_WIDTH, _width_for(can)))
    rows = int(math.ceil((2.0 * can + 2.0) / (machine.DOTS_Y * stretch)))
    BOX.width = width
    BOX.rows = HOP_ROWS + rows + FLOOR_GAUGES
    BOX.height = BOX.rows + CAPTION_ROWS + FOOT_ROWS
    return BOX.rows


#: The legend's bar, cells, in the full braille cell the picture is made of.
BAR_CELLS = 12

#: The two margins as one-column tubes outboard of the board's four: S, the
#: switches' worst node against the record's ceilings, which the board acts
#: on; M, the winding (3 i^2 R into a placeholder pair on TEMP_SCALE_C),
#: which only the operator can act on.
HEADROOM_TITLES = ('S', 'M')

#: Air between the board's four tubes and the two margins, which are not
#: node temperatures. Two, so both gutters are eight columns (at one, the
#: right leader ran a column short, measured).
HEADROOM_GAP = 2

BAR_GLYPH = chr(0x28FF)

#: The thermal nodes a duty cycle can drive into the SOA: the shunt a
#: phase current crosses and the half-bridge above it, per leg. Named in
#: `coaxial.model.thermal`, ordered here the way a leg is read.
SOA_NODES = ('driver_u', 'phase_u', 'driver_v', 'phase_v',
             'driver_w', 'phase_w')

#: The nodes no duty cycle drives, in the right gutter. Both gutter figures
#: are their group's hottest tube on one temperature scale: idle, the MCU
#: (0.666 W through a linear LDO, 15 K over the copper) is hotter than a FET
#: carrying nothing, and the caption names the tube beside it.
BOARD_NODES = ('mcu', 'regulators', 'afe', 'board')

#: The NTC outermost in the left gutter, a column from the six estimates:
#: the one measurement stands apart.
NTC_GAP = 1

NTC_AT = len(SOA_NODES) + NTC_GAP

#: How many columns the left gutter needs.
LEFT_COLUMNS = NTC_AT + 1

#: Where the pair starts inside the right gutter.
HEADROOM_AT = len(BOARD_NODES) + HEADROOM_GAP

#: How many columns the right gutter needs for all of it.
RIGHT_COLUMNS = HEADROOM_AT + len(HEADROOM_TITLES)
