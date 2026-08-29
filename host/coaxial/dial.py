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

from . import angle
from . import ansi
from .raster import cell

#: The magnet, in character cells. Radius in columns; rows are worth two
#: columns, so the vertical scale is halved to keep it round.
RADIUS = 13.0
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
TICK = "'"
ARC = ':'

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
#: One light and the street. The face - rim, graduations, zero - is one
#: quiet deep teal; the READING is the only amber: a dark sweep and a
#: bright pointer. It was rim-ash, ticks-teal, arc-teal, zero-green and
#: pointer-amber at once, and five voices on a face with one thing to say
#: read as a party, not an instrument.
INK = dict([(RIM, 23), (POINTER, ansi.AMBER), (HUB, 250),
            (ZERO, 23), (TICK, 23), (ARC, 130),
            ('(', 250), (')', 250)]
           + [(digit, ansi.ASH) for digit in '0123456789'])


def colourise(text):
    """The drawing again, with each element in its own colour."""
    return '\n'.join(
        # Only the face's own glyphs are inked; captions and the sensor
        # label keep the terminal's colour. Defaulting the rest to DIM
        # painted every caption word nearly black.
        ansi.run([(ch, INK.get(ch)) for ch in line])
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
    cy = (RADIUS + 2.0) * ROW_ASPECT + 1.5

    # A PROTRACTOR, not a plain circle: minor graduations every 6 degrees
    # around the whole rim, a heavier mark every 30, and the degree numbers
    # standing outside - the reference face, at terminal resolution.
    for mark in range(0, 360, 6):
        phi = math.radians(mark)
        glyph = TICK if mark % 30 == 0 else RIM
        _plot(grid, width, height, cx + RADIUS * math.cos(phi),
              cy - RADIUS * math.sin(phi) * ROW_ASPECT, glyph)
        if mark % 30 == 0:
            _plot(grid, width, height, cx + (RADIUS - 1.0) * math.cos(phi),
                  cy - (RADIUS - 1.0) * math.sin(phi) * ROW_ASPECT, TICK)

    for mark in range(0, 360, 30):
        phi = math.radians(mark)
        label = str(mark)
        lx = cx + (RADIUS + 2.6) * math.cos(phi) - len(label) / 2.0 + 0.5
        ly = cy - (RADIUS + 2.2) * math.sin(phi) * ROW_ASPECT
        for i, digit in enumerate(label):
            _plot(grid, width, height, lx + i, ly, digit)

    weak = field is not None and field < WEAK_GAUSS
    phi = math.radians(degrees)
    if not weak:
        for i in range(1, int(RADIUS * 4) + 1):
            r = RADIUS * i / (RADIUS * 4)
            _plot(grid, width, height, cx + r * RADIUS * math.cos(phi) / RADIUS,
                  cy - r * RADIUS * math.sin(phi) * ROW_ASPECT / RADIUS,
                  POINTER)
    # The centre - the reference's circled cross - drawn LAST so the
    # pointer passes under it, not through it.
    _plot(grid, width, height, cx - 1, cy, '(')
    _plot(grid, width, height, cx, cy, HUB)
    _plot(grid, width, height, cx + 1, cy, ')')

    return '\n'.join(''.join(row).rstrip() for row in grid)


def picture(state, width=60, height=19):
    """The drawing with the numbers it is a reading of, above it.

    `state` is what `coaxial.angle.Angle.state()` returns. A reading the
    board never took prints as one, not as zero degrees.
    """
    if state.get('value') is None:
        return ('angle: no reading - loop %s, %s'
                % (state.get('loop', '?'), state.get('error', '?')))

    counts = angle.counts(state['value'])
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
