"""The shaft angle, drawn as a protractor in the dot matrix.

The A1335 reads the field of a round magnet on the end of the rotating
shaft. It does not sit on the axis: it sits a little below it on the PCB,
looking up at the magnet's face. This draws the face of that instrument -
graduations all the way round, the reading swept from zero and a needle
standing at it - because a number between 0 and 360 tells nobody which
way the shaft is pointing.

DOTS, like every other picture in this tree. It was a character face
first - a rim of full stops, a needle of hashes - and the bench's word on
that ASCII stroke set was the same word it gave `raster`: a picture that
is dots in one view and hashes in the next is two drawing conventions in
one terminal. The braille cell buys four times the rows and twice the
columns, which on a round face is the difference between a circle and a
staircase, and it buys a needle that can taper.

WHAT STAYS TEXT IS THE GRADUATION NUMBERS. A braille cell cannot carry a
letter, and a protractor without its numbers is a circle. They sit
outside the rim where no dot goes, in the ash the rest of the tree gives
a caption, and the face itself is dots throughout.

Pure: a reading in, a block of text out. No serial port, no terminal, no
clock, so it is testable without a board and `tools/show_angle.py` is the
only thing that needs one.

Angles run the way the sensor reports them: zero to the right, increasing
counter-clockwise. That is a drawing convention, not a claim about which
way the shaft turns - invariant 10 applies to a picture as much as to a
voltage.
"""
import math

from . import angle
from . import ansi
from .ascii3d import CELL_ASPECT
from .raster import (BRAILLE, BRAILLE_BITS, cell, DOTS_X, DOTS_Y,
                     SUBDOT, covered)

#: Dots between the rim and the ring the numbers stand on, and the room
#: their own row needs beyond that. The face is sized to whatever is left
#: - a protractor whose numbers fall off the frame is not a reference.
LABEL_GAP = 4

#: The graduations, in dots inward from the rim. Every 6 degrees like the
#: reference face, heavier every 30. The minor tick is short enough that
#: sixty of them read as a scale rather than as a second ring.
MINOR_TICK, MAJOR_TICK = 2.5, 6.0

#: How wide a major tick is at the rim, in dots across. A tick one dot
#: wide is a tick one dot wide wherever it points; two make the difference
#: between the thirties and the sixes visible without a second colour.
MAJOR_WIDE = 1.2

#: The reading, swept from zero along the inside of the scale: which
#: radii the band occupies, in dots inward from the rim.
#: TWO DOTS DEEP, not four: at four the band was the heaviest thing on
#: the face and the needle - which is the reading - had to compete with
#: the trail behind it.
SWEEP_OUT, SWEEP_IN = 9.0, 11.0

#: A TAIL, NOT A BAND. The sweep still runs from zero - that is what says
#: how far round the shaft has gone - but it FADES behind the needle, and
#: fast: bright where the needle has just been, down to a trace a few
#: tens of degrees back. Drawn at one weight the whole way it was the
#: loudest thing on the face at large angles, and the needle was reading
#: against its own trail.
#:
#: How far back the fade takes, and how many steps it takes to get there.
SWEEP_FADE = math.radians(70.0)
SWEEP_STEPS = 6

#: The needle: how far short of the graduations it stops, and its half
#: width at the hub and at the tip. TAPERED, because a needle of constant
#: width is a bar, and the end that reads against the scale is the end
#: that has to be fine.
NEEDLE_CLEAR = 3.0
NEEDLE_ROOT, NEEDLE_TIP = 1.3, 0.4

#: The bead at the needle's tip and the hub it turns on, in dots. The
#: bead is a disc for the reason `machine`'s rotor mark is one: a shape
#: that changes with the angle it sits at is a mark you cannot follow
#: round.
BEAD_R, HUB_R = 1.9, 2.6

#: Below this there is no magnet in front of the sensor and the angle is
#: noise. Measured on this board with nothing mounted: 2 gauss, and a
#: heading that wandered 27 degrees while the board sat still. Named
#: because both the needle and the caption have to agree about it - they
#: did not, once.
WEAK_GAUSS = 30

#: What owns a cell, and so what colour it takes: a cell holds dots from
#: whatever passes through it and the highest class present wins. THE HUB
#: OUTRANKS THE NEEDLE so the needle passes under it rather than through
#: it, and the BEAD outranks everything because it is the reading.
#: THE TAIL IS SEVERAL CLASSES, one per step of the fade, because a cell
#: takes exactly one colour and the fade is a colour that varies along
#: the band. `SWEEP[0]` is the far end and `SWEEP[-1]` the end the needle
#: is standing on, so a cell holding two steps takes the brighter - the
#: one nearer the needle, which is the newer.
(FACE, MINOR, MAJOR) = range(3)
SWEEP = tuple(range(MAJOR + 1, MAJOR + 1 + SWEEP_STEPS))
(NEEDLE, HUB, BEAD) = range(SWEEP[-1] + 1, SWEEP[-1] + 4)

#: ONE LIGHT AND THE STREET, which is what the character face arrived at
#: and the matrix does not change: the instrument - rim, graduations, hub -
#: is one quiet deep teal, and the READING is the only warm thing on it,
#: a dark sweep under a bright needle. It was rim-ash, ticks-teal,
#: arc-teal, zero-green and needle-amber at once, and five voices on a
#: face with one thing to say read as a party.
#:
#: The teals are `machine`'s own - its can and yoke are 23 - so the two
#: round pictures in this tree are lit the same way.
#: The tail's own ramp, cold to warm, ending a step short of the
#: needle's amber: a tail that reached the needle's colour would read as
#: part of it, and what it is is where the needle has been.
SWEEP_RAMP = (236, 238, 58, 94, 136, 172)

INK = dict([(FACE, 23), (MINOR, 23), (MAJOR, 30),
            (NEEDLE, ansi.AMBER), (HUB, 250), (BEAD, 231)]
           + list(zip(SWEEP, SWEEP_RAMP)))

#: The graduation numbers. Ash, like every other caption here: they name
#: the scale and the reading is what the eye is meant to find.
LABEL_INK = ansi.ASH


class _Geometry:

    """Where everything on one face goes, in dots.

    ONE PLACE FOR IT because the raster, the labels and the tests all
    have to agree about where the rim is: a scale drawn to one radius
    with numbers placed against another is a protractor that lies.
    """

    def __init__(self, width, height):
        self.cx = width * DOTS_X / 2.0 - 0.5
        self.cy = height * DOTS_Y / 2.0 - 0.5
        # Bounded by whichever way round the box is tighter, less the room
        # the numbers need outside the rim. A dot is square, so this is the
        # same radius in both directions and the face is a circle.
        self.rim = (min(width * DOTS_X, height * DOTS_Y) / 2.0
                    - LABEL_GAP - DOTS_Y - 1.0)
        self.line = max(0.8, self.rim * 0.020)
        self.label = self.rim + LABEL_GAP + DOTS_Y / 2.0
        self.needle = self.rim - MAJOR_TICK - NEEDLE_CLEAR


def _sweep_span(degrees):
    """The reading as a span from zero, counter-clockwise, in radians.

    A reading of 350 is very nearly a whole turn of sweep and a reading
    of 10 is a sliver: the band says how far round from zero the shaft
    is, which is the question a protractor answers.
    """
    return math.radians(degrees % 360.0)


def _classify(dx, dy, geom, span, needle_at):
    """What is at `(dx, dy)` dots from the centre, or None for air."""
    radius = math.hypot(dx, dy)
    if radius > geom.rim + geom.line + 1.0:
        return None

    if needle_at is not None:
        # The bead first: it is the reading, and it sits on top of
        # whatever graduation it happens to be standing against.
        tip_x = geom.needle * math.cos(needle_at)
        tip_y = geom.needle * math.sin(needle_at)
        if math.hypot(dx - tip_x, dy - tip_y) <= BEAD_R:
            return BEAD

    if radius <= HUB_R:
        return HUB

    if needle_at is not None:
        # A TAPERED SHAFT, measured along the needle and across it: `along`
        # is how far out the point is and `across` how far off the line, so
        # the half width can be a function of the first.
        along = dx * math.cos(needle_at) + dy * math.sin(needle_at)
        across = abs(-dx * math.sin(needle_at) + dy * math.cos(needle_at))
        if 0.0 <= along <= geom.needle:
            share = along / max(1e-6, geom.needle)
            if across <= NEEDLE_ROOT + (NEEDLE_TIP - NEEDLE_ROOT) * share:
                return NEEDLE

    phi = math.atan2(dy, dx) % math.tau

    # The graduations, outermost first. A tick is a radial band: its ends
    # in radius, its width in dots across at the radius it is drawn.
    for step, depth, wide in ((30, MAJOR_TICK, MAJOR_WIDE),
                              (6, MINOR_TICK, 0.0)):
        pitch = math.radians(step)
        off = abs(((phi + pitch / 2.0) % pitch) - pitch / 2.0)
        if off * radius <= max(wide, geom.line) / 2.0:
            if geom.rim - depth <= radius <= geom.rim:
                return MAJOR if step == 30 else MINOR

    if abs(radius - geom.rim) <= geom.line:
        return FACE

    if span is not None and geom.rim - SWEEP_IN <= radius <= geom.rim - SWEEP_OUT:
        # ZERO TO THE READING, the way the angles run. A span of exactly
        # zero draws nothing: an instrument reading zero has swept none of
        # its scale, and a single stripe at the top of the band would read
        # as a mark rather than as an empty sweep.
        #
        # AND FADED BY HOW FAR BEHIND THE NEEDLE IT IS, not by where it is
        # on the face: `behind` is measured back from the reading, so the
        # bright end travels with the needle and the far end keeps the
        # trace back to zero.
        if 0.0 < phi <= span:
            behind = min(1.0, (span - phi) / SWEEP_FADE)
            return SWEEP[int((1.0 - behind) * (SWEEP_STEPS - 1) + 0.5)]

    return None


def _raster(degrees, width, height, weak, aspect):
    """Dots, their owners and the label overlay, one entry per cell."""
    dots = [[0] * width for _ in range(height)]
    owner = [[-1] * width for _ in range(height)]
    text = [[None] * width for _ in range(height)]
    geom = _Geometry(width, height)
    span = None if weak else _sweep_span(degrees)
    needle_at = None if weak else math.radians(degrees)

    stretch = aspect / DOTS_Y * DOTS_X
    for y in range(height * DOTS_Y):
        for x in range(width * DOTS_X):
            cls, hits = None, 0
            for ox, oy in SUBDOT:
                at = _classify(x + ox - geom.cx, (geom.cy - y - oy) * stretch,
                               geom, span, needle_at)
                if at is not None:
                    hits += 1
                    if cls is None or at > cls:
                        cls = at
            # THE CORNERS ARE COVERAGE. One of four lit the dot whole, so
            # the rim and the sweep both came out a dot fat and stepped
            # against each other; half a dot or more still lights outright
            # - a one-dot tick is a mark the face means - and the fringe
            # beyond it is dithered. `machine._raster` reads them the
            # same way, and they are the same drawing problem.
            if cls is None or not covered(hits, len(SUBDOT)):
                continue
            col, row = int(x) // DOTS_X, int(y) // DOTS_Y
            if 0 <= row < height and 0 <= col < width:
                dots[row][col] |= BRAILLE_BITS[int(x) % DOTS_X][int(y) % DOTS_Y]
                if cls > owner[row][col]:
                    owner[row][col] = cls

    # THE NUMBERS LAST, and only onto cells no dot reached. They stand
    # outside the rim, so a collision means the face has outgrown its box
    # rather than that a number is in the way - and a number written over
    # the scale would be worse than a missing one.
    for mark in range(0, 360, 30):
        phi = math.radians(mark)
        label = str(mark)
        lx = geom.cx + geom.label * math.cos(phi) - (len(label) - 1) * DOTS_X / 2.0
        ly = geom.cy - geom.label * math.sin(phi) / stretch
        row = cell(ly / DOTS_Y - 0.5)
        for index, digit in enumerate(label):
            col = cell((lx + index * DOTS_X) / DOTS_X - 0.5)
            if 0 <= row < height and 0 <= col < width and not dots[row][col]:
                text[row][col] = digit
    return dots, owner, text, geom


def render(degrees, width=64, height=23, field=None, aspect=CELL_ASPECT,
           colour=False):
    """The face at `degrees`, with the reading swept from zero.

    `field` is the gauss the part reports. Below a few tens of gauss there
    is no magnet in front of the sensor and the angle is noise; the
    picture says so by drawing the instrument and no reading, rather than
    a confident needle at a number that means nothing.

    `aspect` is how tall the terminal's cell is against its width, the
    same knob `machine.render` takes and for the same reason: the geometry
    is round at 2.0, and a face that reads as an ellipse is the FONT
    saying its cell is not.

    Colour is asked for here rather than applied afterwards: a braille
    cell carries dots from up to eight places and its glyph does not say
    which, so there is nothing for a `colourise(text)` to key on. That is
    what took the old one out.
    """
    weak = field is not None and field < WEAK_GAUSS
    dots, owner, text, _ = _raster(degrees, width, height, weak, aspect)
    lines = []
    for row in range(height):
        cells = [(text[row][col] or chr(BRAILLE + dots[row][col]),
                  LABEL_INK if text[row][col] else INK.get(owner[row][col]))
                 for col in range(width)]
        lines.append(ansi.run(cells) if colour
                     else ''.join(char for char, _ in cells))
    return '\n'.join(lines)


def caption(degrees, field=None):
    """One line naming the reading, for the row under the face.

    `machine.caption` is the same idea for the other round picture: the
    drawing says where, and one line says what - a needle standing at a
    graduation is read to about a degree and the part reports hundredths.

    A weak field prints no angle at all. The face has already left its
    needle off; a caption underneath saying 351.65 would be the picture
    disagreeing with itself, which is what it did once.
    """
    if field is not None and field < WEAK_GAUSS:
        return '--.-- deg   no magnet, %d gauss' % field
    text = '%.2f deg' % (degrees % 360.0)
    return text if field is None else '%s   %d gauss' % (text, field)


def picture(state, width=64, height=23):
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
    # still. Suppressing the needle and then printing the angle above it
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
