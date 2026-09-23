"""The shaft angle, drawn as a protractor in the dot matrix."""
import math

from . import angle
from . import ansi
from .ascii3d import CELL_ASPECT
from .scaling import KELVIN_AT_ZERO_C
from .graphics.raster import (BRAILLE, BRAILLE_BITS, cell, DOTS_X, DOTS_Y, table,
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

#: The reading's band, swept from zero inside the scale: radii in dots from
#: the rim. Two deep: at four it outweighed the needle.
SWEEP_OUT, SWEEP_IN = 9.0, 11.0

#: The sweep fades behind the needle - bright where it has just been, a trace
#: a few tens of degrees back; at one weight it outshouted the needle. How
#: far back the fade takes, and in how many steps.
SWEEP_FADE = math.radians(70.0)
SWEEP_STEPS = 6

#: The needle: its stop short of the graduations and its half width at hub
#: and tip - tapered, so the reading end is fine.
NEEDLE_CLEAR = 3.0
NEEDLE_ROOT, NEEDLE_TIP = 1.3, 0.4

#: The tip bead and the hub, dots; the bead a disc, which reads the same at
#: every angle.
BEAD_R, HUB_R = 1.9, 2.6

#: No magnet below this: the angle is noise (2 G with nothing mounted, the
#: heading wandering 27 degrees). One constant for the needle and caption.
WEAK_GAUSS = 30

#: Cell classes; the highest present wins. The hub outranks the needle (it
#: passes under), the bead everything. The tail is one class per fade step,
#: SWEEP[-1] nearest the needle, so a shared cell takes the newer.
(FACE, MINOR, MAJOR) = range(3)
SWEEP = tuple(range(MAJOR + 1, MAJOR + 1 + SWEEP_STEPS))
(NEEDLE, HUB, BEAD) = range(SWEEP[-1] + 1, SWEEP[-1] + 4)

#: One light: the instrument (rim, graduations, hub) in `machine`'s deep teal
#: 23, the reading the only warm thing - five colours read as a party. The
#: tail's ramp ends a step short of the needle's amber, so it reads as where
#: the needle has been.
SWEEP_RAMP = (236, 238, 58, 94, 136, 172)

INK = dict([(FACE, 23), (MINOR, 23), (MAJOR, 30),
            (NEEDLE, ansi.AMBER), (HUB, 250), (BEAD, 231)]
           + list(zip(SWEEP, SWEEP_RAMP)))

#: The graduation numbers. Ash, like every other caption here: they name
#: the scale and the reading is what the eye is meant to find.
LABEL_INK = ansi.ASH


class _Geometry:

    """Where everything on one face goes, in dots."""

    def __init__(self, width, height):
        self.cx = width * DOTS_X / 2.0 - 0.5
        self.cy = height * DOTS_Y / 2.0 - 0.5
        # Bounded by whichever way round the box is tighter, less the room the
        # numbers need outside the rim.
        self.rim = (min(width * DOTS_X, height * DOTS_Y) / 2.0
                    - LABEL_GAP - DOTS_Y - 1.0)
        self.line = max(0.8, self.rim * 0.020)
        self.label = self.rim + LABEL_GAP + DOTS_Y / 2.0
        self.needle = self.rim - MAJOR_TICK - NEEDLE_CLEAR


def _sweep_span(degrees):
    """The reading as a span from zero, counter-clockwise, in radians."""
    return math.radians(degrees % 360.0)


#: A sample in the band the sweep may light, decided per reading.
_SWEEP_BAND = object()

#: The face's sample tables by (width, height, aspect) - one per size
#: a page draws at.
_FACES = {}


def _fixed(radius, phi, geom):
    """What the face alone puts at a sample, in the order the drawing layers
    them: the hub, a graduation, the rim, the band the sweep may light,
    or nothing.
    """
    if radius <= HUB_R:
        return HUB
    for step, depth, wide in ((30, MAJOR_TICK, MAJOR_WIDE),
                              (6, MINOR_TICK, 0.0)):
        pitch = math.radians(step)
        off = abs(((phi + pitch / 2.0) % pitch) - pitch / 2.0)
        if (off * radius <= max(wide, geom.line) / 2.0
                and geom.rim - depth <= radius <= geom.rim):
            return MAJOR if step == 30 else MINOR
    if abs(radius - geom.rim) <= geom.line:
        return FACE
    if geom.rim - SWEEP_IN <= radius <= geom.rim - SWEEP_OUT:
        return _SWEEP_BAND
    return None


def _samples(width, height, aspect):
    """Every dot within the face, with its samples in `SUBDOT` order - `(dx,
    dy, radius, angle, what the face alone puts there)`.
    """
    return table(_FACES, (width, height, aspect),
                 lambda: _sampled(width, height, aspect))


def _sampled(width, height, aspect):
    """The table `_samples` keeps, built by walking every dot's samples
    over the face."""
    geom = _Geometry(width, height)
    stretch = aspect / DOTS_Y * DOTS_X
    made = []
    for y in range(height * DOTS_Y):
        for x in range(width * DOTS_X):
            samples = []
            for ox, oy in SUBDOT:
                dx, dy = x + ox - geom.cx, (geom.cy - y - oy) * stretch
                radius = math.hypot(dx, dy)
                if radius > geom.rim + geom.line + 1.0:
                    continue
                phi = math.atan2(dy, dx) % math.tau
                samples.append((dx, dy, radius, phi,
                                _fixed(radius, phi, geom)))
            if samples:
                made.append((x, y, tuple(samples)))
    return made


def _needle(geom, at):
    """The needle at `at` radians, laid out once a frame: its direction
    as a cosine and a sine, and its tip, where the bead sits."""
    c, s = math.cos(at), math.sin(at)
    return c, s, geom.needle * c, geom.needle * s


def _classify(sample, geom, span, needle):
    """What is at a sample this reading, or None for air."""
    dx, dy, _radius, phi, fixed = sample
    if needle is not None and _on_bead(dx, dy, needle):
        return BEAD
    if fixed == HUB:
        return HUB
    if needle is not None and _on_needle(dx, dy, geom, needle):
        return NEEDLE
    if fixed is not _SWEEP_BAND:
        return fixed

    # ZERO TO THE READING, the way the angles run.
    if span is not None and 0.0 < phi <= span:
        behind = min(1.0, (span - phi) / SWEEP_FADE)
        return SWEEP[int((1.0 - behind) * (SWEEP_STEPS - 1) + 0.5)]
    return None


def _on_bead(dx, dy, needle):
    """Within the bead at the needle's tip."""
    _c, _s, tip_x, tip_y = needle
    return math.hypot(dx - tip_x, dy - tip_y) <= BEAD_R


def _on_needle(dx, dy, geom, needle):
    """On the needle's TAPERED SHAFT, measured along the needle and across
    it: `along` is how far out the point is and `across` how far off the
    line, so the half width can be a function of the first."""
    c, s, _tip_x, _tip_y = needle
    along = dx * c + dy * s
    across = abs(-dx * s + dy * c)
    if not 0.0 <= along <= geom.needle:
        return False
    share = along / max(1e-6, geom.needle)
    return across <= NEEDLE_ROOT + (NEEDLE_TIP - NEEDLE_ROOT) * share


def _raster(degrees, width, height, weak, aspect):
    """Dots, their owners and the label overlay, one entry per cell."""
    dots = [[0] * width for _ in range(height)]
    owner = [[-1] * width for _ in range(height)]
    text = [[None] * width for _ in range(height)]
    geom = _Geometry(width, height)
    span = None if weak else _sweep_span(degrees)
    needle = None if weak else _needle(geom, math.radians(degrees))

    stretch = aspect / DOTS_Y * DOTS_X
    for x, y, samples in _samples(width, height, aspect):
        seen = [at for at in (_classify(sample, geom, span, needle)
                              for sample in samples) if at is not None]
        # THE CORNERS ARE COVERAGE.
        if not seen or not covered(len(seen), len(SUBDOT)):
            continue
        col, row = x // DOTS_X, y // DOTS_Y
        if not (0 <= row < height and 0 <= col < width):
            continue
        dots[row][col] |= BRAILLE_BITS[x % DOTS_X][y % DOTS_Y]
        owner[row][col] = max(owner[row][col], max(seen))

    # THE NUMBERS LAST, and only onto cells no dot reached.
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
    """The face at `degrees`, with the reading swept from zero."""
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


def caption(degrees, field=None, gauss=True):
    """One line naming the reading, for the row under the face."""
    if field is not None and field < WEAK_GAUSS:
        return '--.-- deg   no magnet, %d gauss' % field
    text = '%.2f deg' % (degrees % 360.0)
    return (text if field is None or not gauss
            else '%s   %d gauss' % (text, field))


#: A side scale's width in cells: four for a graduation's number, one for
#: its mark, two for the tube, one of air against the face.
SCALE_W = 8
#: The tube: two cells wide, solid - the reading's ink to the reading, ash
#: glass above (one dotted column read as a stray line; bench 2026-09-07).
TUBE_W = 2
#: The scales: the die -40 to 150 C (A1335 datasheet), the field 0-1200 G.
#: Each tube in three bands, blue under normal, green, red past (bench
#: 2026-09-07): the die's normal is where this board works, 15-65 C (the
#: thermal ramp read a room-temperature die as cold); the field's is the
#: datasheet's recommended 300-1000 G - weak or absent under, too close past.
DIE_RANGE = (-40.0, 150.0)
DIE_TICKS = (-40, 0, 50, 100, 150)
DIE_BAND = (15.0, 65.0)
FIELD_RANGE = (0.0, 1200.0)
FIELD_TICKS = (0, 300, 600, 900, 1200)
FIELD_BAND = (300.0, 1000.0)


def _band_ink(value, band):
    """Blue under `band`, green inside it, red past it."""
    if value < band[0]:
        return ansi.BLUE
    if value <= band[1]:
        return ansi.GREEN
    return ansi.RED


def field_ink(gauss):
    """The field tube's ink: blue under the recommended band - a weak
    magnet, or none - green inside it, red past it."""
    return _band_ink(gauss, FIELD_BAND)


def die_ink(celsius):
    """The die tube's ink: blue under the board's working range, green
    through it, red past it."""
    return _band_ink(celsius, DIE_BAND)


def scale(value, span, height, ticks, title, reading, ink_of, side='left',
          colour=False):
    """A vertical scale beside the face, `height + 1` lines of SCALE_W."""
    low, high = span
    rows = height - 2                        # the tube, rows 1..height-2
    dots_tall = rows * DOTS_Y
    frac = 0.0 if high == low else (float(value) - low) / (high - low)
    lit = int(round(max(0.0, min(1.0, frac)) * dots_tall))
    ink = ink_of(value)

    def at(v):
        """The dot row (from the tube's bottom) of `v`, clamped."""
        f = 0.0 if high == low else (float(v) - low) / (high - low)
        return int(round(max(0.0, min(1.0, f)) * (dots_tall - 1)))

    marks = {}
    for t in ticks:
        marks.setdefault(at(t), str(t))
    lines = []
    for row in range(height):
        if row == 0:
            text = title.center(SCALE_W)
            lines.append(ansi.paint(text, LABEL_INK) if colour else text)
            continue
        if row == height - 1:
            lines.append(' ' * SCALE_W)
            continue
        r = row - 1                          # the tube's own row, from the top
        tube = []
        label, mark_bits = '', 0
        for dy in range(DOTS_Y):
            from_bottom = dots_tall - 1 - (r * DOTS_Y + dy)
            if from_bottom in marks:
                label = marks[from_bottom]
                mark_bits |= BRAILLE_BITS[0][dy] | BRAILLE_BITS[1][dy]
        # Four dots wide the whole way: the fill in the reading's ink, the
        # glass above it in ash.
        full, glass = 0, 0
        for dy in range(DOTS_Y):
            from_bottom = dots_tall - 1 - (r * DOTS_Y + dy)
            both = BRAILLE_BITS[0][dy] | BRAILLE_BITS[1][dy]
            if from_bottom < lit:
                full |= both
            else:
                glass |= both
        cells = [(chr(BRAILLE + (full | glass)), ink if full else LABEL_INK)
                 for _ in range(TUBE_W)]
        mark = chr(BRAILLE + mark_bits) if mark_bits else ' '
        number = label.rjust(4) if side == 'left' else label.ljust(4)
        if side == 'left':
            parts = [(number, LABEL_INK), (mark, LABEL_INK)] + cells + [(' ', None)]
        else:
            parts = [(' ', None)] + cells + [(mark, LABEL_INK), (number, LABEL_INK)]
        lines.append(ansi.run(parts) if colour
                     else ''.join(t for t, _ in parts))
    foot = reading.center(SCALE_W)
    lines.append(ansi.paint(foot, ink) if colour else foot)
    return lines


def beside(face, left, right):
    """The face between its two scales, line for line, one space of air
    each side; `face` is the dial's text with its caption already under
    it, the scales what `scale` returned."""
    rows = face.split('\n')
    if not len(rows) == len(left) == len(right):
        raise ValueError('the face and its scales differ in rows: %d, %d, %d'
                         % (len(rows), len(left), len(right)))
    return '\n'.join(l + ' ' + f + ' ' + r for l, f, r in zip(left, rows, right))


def instrument(degrees, field, kelvin, width=58, height=21,
               aspect=CELL_ASPECT, colour=False):
    """The face with its caption, between the die's temperature and the
    field: what SHAFT ANGLE draws with its scales, and what a notebook
    shows.
    """
    text = caption(degrees, field, gauss=False)
    foot = (' ' * max(0, (width - len(text)) // 2) + text).ljust(width)
    face = '\n'.join([render(degrees, width, height, field, aspect=aspect,
                             colour=colour),
                      ansi.paint(foot, INK[NEEDLE]) if colour else foot])
    celsius = (kelvin or KELVIN_AT_ZERO_C) - KELVIN_AT_ZERO_C
    left = scale(celsius, DIE_RANGE, height, DIE_TICKS, 'DIE',
                 '%.1f C' % celsius, die_ink, 'left', colour=colour)
    right = scale(field or 0, FIELD_RANGE, height, FIELD_TICKS, 'FIELD',
                  '%d G' % (field or 0), field_ink, 'right', colour=colour)
    return beside(face, left, right)


def picture(state, width=64, height=23):
    """The drawing with the numbers it is a reading of, above it."""
    if state.get('value') is None:
        return ('angle: no reading - loop %s, %s'
                % (state.get('loop', '?'), state.get('error', '?')))

    counts = angle.counts(state['value'])
    degrees = state.get('degrees', counts * 360.0 / 4096.0)
    field = state.get('field')
    weak = field is not None and field < WEAK_GAUSS

    # With no magnet the counts are still what the part said - real data, and
    # worth showing.
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
