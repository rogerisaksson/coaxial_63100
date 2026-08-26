"""The shaft angle, drawn in characters.

The A1335 reads the field of a round magnet on the end of the rotating
shaft. It does not sit on the axis: it sits a little below it on the PCB,
looking up at the magnet's face. This draws that - the magnet from the front
with its north mark at the reported angle, and the sensor below it - because
a number between 0 and 360 tells nobody which way the shaft is pointing.

Pure: a reading in, a block of text out. No serial port, no terminal, no
clock, so it is testable without a board and `tools/show_angle.py` is the
only thing that needs one.

Angles run the way the sensor reports them: zero to the right, increasing
counter-clockwise. That is a drawing convention, not a claim about which way
the shaft turns - invariant 10 applies to a picture as much as to a voltage.
"""
import math

from . import ansi
from .raster import cell

#: The magnet, in character cells. Radius in columns; rows are worth two
#: columns, so the vertical scale is halved to keep it round.
RADIUS = 11.0
ROW_ASPECT = 0.5

#: How far below the shaft axis the sensor sits, in the same columns. Far
#: enough to clear the magnet's rim and show the air gap between the two,
#: which is the thing that has to be right on a real assembly.
SENSOR_DROP = 15.0

#: The rim, the pointer and the marks. One glyph each rather than a shading
#: ramp: this is a face-on diagram, not a lit surface, and a ramp here would
#: imply a light that is not in the picture.
RIM = '.'
POINTER = '#'
HUB = '+'
ZERO = '0'

#: Below this there is no magnet in front of the sensor and the angle is
#: noise. Measured on this board with nothing mounted: 2 gauss, and a heading
#: that wandered 27 degrees while the board sat still. Named because both the
#: pointer and the caption have to agree about it - they did not, once.
WEAK_GAUSS = 30

#: One colour per element, because the elements are already one glyph each.
#: The pointer is the reading and gets the only warm colour on the face; the
#: rim and the sensor are furniture and stay out of its way. A face where
#: everything is the same weight makes the reader hunt for the one thing that
#: moved.
INK = {RIM: ansi.DIM, POINTER: ansi.AMBER, HUB: ansi.WHITE,
       ZERO: ansi.GREEN}


def colourise(text):
    """The drawing again, with each element in its own colour."""
    return '\n'.join(
        ansi.run([(ch, INK.get(ch, ansi.DIM)) for ch in line])
        for line in text.split('\n'))


def _plot(grid, width, height, col, row, glyph):
    c, r = cell(col), cell(row)
    if 0 <= r < height and 0 <= c < width:
        grid[r][c] = glyph


def render(degrees, width=60, height=19, field=None):
    """The magnet at `degrees`, with the sensor below it.

    `field` is the gauss the part reports. Below a few tens of gauss there is
    no magnet in front of the sensor and the angle is noise; the picture says
    so rather than drawing a confident pointer at a number that means
    nothing.
    """
    grid = [[' '] * width for _ in range(height)]
    cx = (width - 1) / 2.0
    cy = RADIUS * ROW_ASPECT + 1.0

    steps = max(48, int(RADIUS * 8))
    for i in range(steps):
        phi = 2.0 * math.pi * i / steps
        _plot(grid, width, height, cx + RADIUS * math.cos(phi),
              cy - RADIUS * math.sin(phi) * ROW_ASPECT, RIM)

    # Zero, so the pointer has something to be an angle from. Outside the
    # rim rather than on it: a mark on the rim reads as part of the magnet.
    _plot(grid, width, height, cx + RADIUS + 2.0, cy, ZERO)

    weak = field is not None and field < WEAK_GAUSS
    phi = math.radians(degrees)
    if not weak:
        for i in range(1, int(RADIUS * 4) + 1):
            r = RADIUS * i / (RADIUS * 4)
            _plot(grid, width, height, cx + r * RADIUS * math.cos(phi) / RADIUS,
                  cy - r * RADIUS * math.sin(phi) * ROW_ASPECT / RADIUS,
                  POINTER)
    _plot(grid, width, height, cx, cy, HUB)

    # The sensor: a part on the board, below the axis and looking up at the
    # magnet's face. Drawn as what it is rather than as another circle.
    top = cell(cy + SENSOR_DROP * ROW_ASPECT)
    label = ' A1335 '
    left = cell(cx - len(label) / 2.0)
    for r, row in enumerate((('+' + '-' * len(label) + '+'),
                             ('|' + label + '|'),
                             ('+' + '-' * len(label) + '+'))):
        for c, glyph in enumerate(row):
            _plot(grid, width, height, left - 1 + c, top + r, glyph)

    # The gap between them, which is the working air gap.
    for r in range(cell(cy + RADIUS * ROW_ASPECT) + 1, top):
        _plot(grid, width, height, cx, r, ':')

    return '\n'.join(''.join(row).rstrip() for row in grid)


def picture(state, width=60, height=19):
    """The drawing with the numbers it is a reading of, above it.

    `state` is what `coaxial.angle.Angle.state()` returns. A reading the
    board never took prints as one, not as zero degrees.
    """
    if state.get('value') is None:
        return ('angle: no reading - loop %s, %s'
                % (state.get('loop', '?'), state.get('error', '?')))

    counts = state['value'] & 0x0FFF
    degrees = state.get('degrees', counts * 360.0 / 4096.0)
    field = state.get('field')
    weak = field is not None and field < WEAK_GAUSS

    # With no magnet the counts are still what the part said - real data,
    # and worth showing. The degrees are a claim about a shaft, and there is
    # no shaft angle in a number that wanders 27 degrees while the board sits
    # still. Suppressing the pointer and then printing the angle above it
    # anyway was the picture disagreeing with its own caption: measured on
    # this board at 2 gauss, the heading read 351.65 one frame and 12.74 the
    # next, and looked for all the world like a shaft spinning.
    if weak:
        heading = 'angle      -- deg   %4d of 4096 counts   flags %X' % (
            counts, state['value'] >> 12)
    else:
        heading = 'angle %7.2f deg   %4d of 4096 counts   flags %X' % (
            degrees, counts, state['value'] >> 12)

    lines = [
        heading,
        'loop %-8s %d readings, %d errors'
        % (state.get('loop', '?'), state.get('updates', 0),
           state.get('errors', 0)),
    ]

    if field is not None:
        lines.append('field %4d gauss%s'
                     % (field, '   - no magnet in front of the sensor, so the '
                        'angle is noise' if weak else ''))

    lines += ['', render(degrees, width, height, field), '',
              'coaxial_63100 - A1335 on SPI4, below the shaft']

    return '\n'.join(lines)
