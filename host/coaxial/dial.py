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

    weak = field is not None and field < 30
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

    lines = [
        'angle %7.2f deg   %4d of 4096 counts   flags %X'
        % (degrees, counts, state['value'] >> 12),
        'loop %-8s %d readings, %d errors'
        % (state.get('loop', '?'), state.get('updates', 0),
           state.get('errors', 0)),
    ]

    if field is not None:
        lines.append('field %4d gauss%s'
                     % (field, '   - no magnet in front of the sensor, so the '
                        'angle is noise' if field < 30 else ''))

    lines += ['', render(degrees, width, height, field), '',
              'coaxial_63100 - A1335 on SPI4, below the shaft']

    return '\n'.join(lines)
