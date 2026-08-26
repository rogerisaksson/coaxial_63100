"""The board's attitude, drawn in characters.

The IMU reports a rotation vector - a unit quaternion saying how the part is
turned relative to its reference frame. This turns that into a picture of the
PCB, because a quaternion read aloud tells nobody which way up anything is.

Pure: a quaternion in, a block of text out. No serial port, no terminal, no
clock - so it is testable without a board, and `tools/show_orientation.py` is
the only thing that needs one.

The board is drawn as its own outline with the connector edge marked, so a
rotation is read off the shape rather than off four decimals.
"""
import math

#: Half-extents of the PCB in its own frame: X along the connector edge, Y
#: across it, Z out of the component side. Proportions, not millimetres - the
#: drawing is a shape, not a measurement, and invariant 10 applies here too.
BOARD = (1.0, 0.8, 0.06)

EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 0),          # component side
    (4, 5), (5, 6), (6, 7), (7, 4),          # solder side
    (0, 4), (1, 5), (2, 6), (3, 7),          # the four verticals
)

#: Drawn brightest so the reader can tell which way round the board is.
CONNECTOR_EDGE = (0, 1)

#: What is silkscreened on the board, and where. Offset from the centre along
#: +Y rather than centred on it, so the label itself says which way up the
#: board is even when the outline is symmetric about both axes.
LABEL = 'Coaxial 63100'
LABEL_Y = 0.34
#: Narrow enough that the letters land on adjacent columns at the default
#: width - a baseline any wider spaces them out and the word stops reading.
LABEL_HALF_WIDTH = 0.40


def _cell(value):
    """`value` to the nearest cell, halves always upward.

    Not round(): Python rounds halves to even, so 15.5 and 16.5 both land on
    16 and consecutive positions collide. Measured - it wrote the label as
    "o x a  3 0 0", every second character on top of its neighbour.
    """
    return int(math.floor(value + 0.5))


def corners(half=BOARD):
    """The eight corners of the board in its own frame."""
    x, y, z = half
    return [(sx * x, sy * y, sz * z)
            for sz in (1, -1) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]


def rotate(q, v):
    """`v` turned by the unit quaternion `q`, given as (i, j, k, real).

    The sandwich product q*v*conj(q), written out rather than composed from a
    quaternion class: there is one rotation in this repository and a class for
    it would be an abstraction with one implementation.
    """
    i, j, k, w = q
    x, y, z = v

    # t = 2 * (qv x v), then v' = v + w*t + qv x t
    tx = 2.0 * (j * z - k * y)
    ty = 2.0 * (k * x - i * z)
    tz = 2.0 * (i * y - j * x)

    return (x + w * tx + (j * tz - k * ty),
            y + w * ty + (k * tx - i * tz),
            z + w * tz + (i * ty - j * tx))


def normalise(q):
    """`q` as a unit quaternion, or the identity if it has no length.

    A rotation vector from a part that is still settling can read all zeros,
    and dividing by that would put a NaN on the screen where an orientation
    belongs.
    """
    i, j, k, w = q
    n = math.sqrt(i * i + j * j + k * k + w * w)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (i / n, j / n, k / n, w / n)


def euler_degrees(q):
    """(roll, pitch, yaw) in degrees, for the caption under the picture.

    Aerospace order - yaw about Z, then pitch about Y, then roll about X -
    because that is what anybody reading "the board is tilted" expects.
    """
    i, j, k, w = normalise(q)

    roll = math.atan2(2.0 * (w * i + j * k), 1.0 - 2.0 * (i * i + j * j))
    sin_pitch = 2.0 * (w * j - k * i)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * k + i * j), 1.0 - 2.0 * (j * j + k * k))

    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def _label_ends(half=BOARD):
    """The two ends of the silkscreen baseline, on the component side.

    A baseline rather than a point per character: projecting each character
    separately and rounding put gaps in the word, because two characters can
    round to the same column while the next skips one. Interpolating along
    the projected segment spaces them evenly however the board is turned.
    """
    _x, y, z = half
    at = LABEL_Y * (y / abs(y) if y else 1.0)
    return ((-LABEL_HALF_WIDTH, at, z), (LABEL_HALF_WIDTH, at, z))


def facing(q):
    """How much of the component side is turned towards the reader, -1..1.

    The board's +Z normal after rotation. Positive means the labelled face is
    visible; at or below zero the reader is looking at the solder side and
    the silkscreen must not show through it.
    """
    return rotate(normalise(q), (0.0, 0.0, 1.0))[2]


def _line(grid, a, b, ch):
    """Bresenham between two (col, row) points, clipped to the grid."""
    (x0, y0), (x1, y1) = a, b
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy

    while True:
        if 0 <= y0 < len(grid) and 0 <= x0 < len(grid[0]):
            # A marked edge overwrites a plain one, never the other way, so
            # the connector stays visible where edges cross.
            if grid[y0][x0] == ' ' or ch != '.':
                grid[y0][x0] = ch
        if x0 == x1 and y0 == y1:
            return
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def _stamp(grid, text, start, end):
    """`text` written along the screen segment start..end, one cell apart.

    Laid out from the segment's midpoint along its direction, a cell per
    character, rather than by projecting each character and rounding. That
    rounding put gaps inside the word - two characters landing on one cell
    while the next skipped one - and a silkscreen with a hole in it reads as
    a rendering fault rather than as a label.

    A segment shorter than the text is a board turned edge-on: the label is
    dropped whole rather than written on top of itself.
    """
    (x0, y0), (x1, y1) = start, end
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)

    if length < len(text) - 1:
        return

    ux, uy = dx / length, dy / length
    mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    first = -(len(text) - 1) / 2.0

    for i, ch in enumerate(text):
        col = _cell(mx + (first + i) * ux)
        row = _cell(my + (first + i) * uy)
        if 0 <= row < len(grid) and 0 <= col < len(grid[0]):
            grid[row][col] = ch


def render(q, width=44, height=19):
    """The board under rotation `q`, as `height` lines of `width` characters.

    Orthographic along Z: what you see is the board from where the reader is
    sitting. Characters are twice as tall as they are wide, so the horizontal
    scale is doubled to keep the board square when it is square.

    The silkscreen needs about 42 columns to fit; narrower than that and the
    outline is still right but the label is left off, the same way it is when
    the board turns edge-on.
    """
    unit = normalise(q)
    grid = [[' '] * width for _ in range(height)]

    span = max(BOARD) * 1.35
    cx, cy = (width - 1) / 2.0, (height - 1) / 2.0

    def to_screen(point):
        x, y, _z = rotate(unit, point)
        return (_cell(cx + (x / span) * cx), _cell(cy - (y / span) * cy))

    pts = [to_screen(p) for p in corners()]

    for a, b in EDGES:
        marked = (a, b) == CONNECTOR_EDGE or (b, a) == CONNECTOR_EDGE
        _line(grid, pts[a], pts[b], '#' if marked else '.')

    # Last, and only when that face is towards the reader: the silkscreen is
    # printed on the component side and does not read through the board.
    if facing(unit) > 0.15:
        _stamp(grid, LABEL, *[to_screen(p) for p in _label_ends()])

    return '\n'.join(''.join(row).rstrip() for row in grid)


def picture(q, width=44, height=19, frame=None, age=None):
    """The drawing with its caption: the angles, and what the marked edge is.

    `frame` and `age` are for a live view: a counter that moves says the
    picture is being redrawn, and the number of samples since the last new
    quaternion says whether the part is still sending. A still picture of a
    still board and a frozen one look identical without them.
    """
    roll, pitch, yaw = euler_degrees(q)
    i, j, k, w = normalise(q)
    side = 'component side' if facing(q) > 0.15 else 'solder side'

    lines = [
        render(q, width, height),
        '',
        'roll %+7.1f   pitch %+7.1f   yaw %+7.1f  (degrees)'
        % (roll, pitch, yaw),
        'q  i %+.4f  j %+.4f  k %+.4f  real %+.4f' % (i, j, k, w),
        '#  the connector edge   -  facing you: %s' % side,
    ]

    if frame is not None:
        spin = '|/-\\'
        note = 'frame %d %s' % (frame, spin[frame % 4])
        if age:
            note += '   no new sample for %d frame%s' % (age,
                                                         '' if age == 1 else 's')
        else:
            note += '   live'
        lines.append(note)

    return '\n'.join(lines)
