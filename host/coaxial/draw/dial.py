"""The shaft angle, drawn as a protractor in the dot matrix."""
import math

from coaxial.devices import angle
from coaxial.devices.scaling import KELVIN_AT_ZERO_C
from coaxial.draw.ascii3d import CELL_ASPECT
from coaxial.graphics.raster import (BRAILLE, BRAILLE_BITS, cell, DOTS_X, DOTS_Y, table,
                     SUBDOT, covered)
from machine import ansi

#: Dots between the rim and the ring the numbers stand on, and the room
#: their own row needs beyond that. The face is sized to whatever is left
#: - a protractor whose numbers fall off the frame is not a reference.
LABEL_GAP = 7

#: The graduations, in dots inward from the rim. Every 6 degrees like the
#: reference face, heavier every 30. The minor tick is short enough that
#: sixty of them read as a scale rather than as a second ring. On a rim of
#: MICRO_RIM dots or more, a micro tick each 3 degrees.
MINOR_TICK, MAJOR_TICK = 2.5, 6.0
MICRO_TICK, MICRO_RIM = 1.2, 60.0

#: The bezel: a ring BEZEL_GAP dots outside the rim.
BEZEL_GAP = 3.0

#: The crosshair: dotted along the axes, a dot each CROSS_PITCH, from the hub
#: to the sweep band; its horizontal arm alone where the sub-dials stand.
CROSS_PITCH = 3

#: Dials in the dial: on a rim of SUB_RIM dots or more, the field in the face's upper
#: half and the die in its lower - each SUB_R of the rim across, its centre SUB_OFF of
#: the rim off the hub. SEGMENTS LED segments round SUB_ARC degrees, SEG_DEPTH dots
#: deep, SEG_FILL of their pitch lit and dark teal off; lit to the reading in the
#: phosphor of the band it is in (BAND_INK: cool under, cyan in, red past), the leading one
#: white-hot; a notch NOTCH long outside the arc at each band edge; the reading and its
#: kana name (SUB_NAMES) at the centre (2026-09-24: hands and arcs read as a watch's).
SUB_RIM = 30.0
SUB_R, SUB_OFF = 0.27, 0.44
SUB_ARC = 270.0
SEGMENTS = 24
SEG_DEPTH, SEG_FILL = 3.0, 0.68
NOTCH = 1.5
SUB_NAMES = ('ｼﾞｶｲ', 'ｵﾝﾄﾞ')

#: How wide a major tick is at the rim, in dots across. A tick one dot
#: wide is a tick one dot wide wherever it points; two make the difference
#: between the thirties and the sixes visible without a second colour.
MAJOR_WIDE = 1.2

#: The reading's band, swept from zero inside the scale: radii in dots from
#: the rim. Two deep: at four it outweighed the needle.
SWEEP_OUT, SWEEP_IN = 9.0, 11.0

#: The sweep fades behind the needle - bright where it has just been, to black
#: SWEEP_FADE back (2026-09-24: "tonas till svärta", not grey); at one weight
#: it outshouted the needle. In SWEEP_STEPS steps.
SWEEP_FADE = math.radians(70.0)
SWEEP_STEPS = 8

#: The needle: its stop short of the graduations and its half width at hub
#: and tip - a crisp line, tapering to the reading end.
NEEDLE_CLEAR = 3.0
NEEDLE_ROOT, NEEDLE_TIP = 0.75, 0.35

#: The hub, a ring HUB_R across and RING thick: hollow - a filled disc read as a
#: watch's. No ball at the needle's tip (2026-09-24).
HUB_R, RING = 2.6, 0.7

#: No magnet below this: the angle is noise (2 G with nothing mounted, the
#: heading wandering 27 degrees). One constant for the needle and caption.
WEAK_GAUSS = 30

#: Cell classes; the highest present wins. The hub outranks the needle (it
#: passes under), and the needle passes under the sub-dials: none of it inside
#: their discs. The tail is one class per fade step, SWEEP[-1] nearest the
#: needle, so a shared cell takes the newer.
(CROSS, BEZEL, FACE, MICRO, MINOR, MAJOR, SEG_OFF, NOTCH_MARK) = range(8)
SWEEP = tuple(range(NOTCH_MARK + 1, NOTCH_MARK + 1 + SWEEP_STEPS))
(NEEDLE, SEG_LOW, SEG_OK, SEG_HOT, SEG_LEAD, HUB) = range(SWEEP[-1] + 1, SWEEP[-1] + 7)

#: The bands' phosphor: under, in, past - the side scales' and the sub-dials'.
BAND_INK = ((70, 130, 235), (40, 225, 200), (255, 64, 48))

#: One light: the instrument (rim, graduations, hub) in `cross_section`'s deep teal
#: 23, the thirties brighter, the reading the only warm thing - five colours
#: read as a party. The tail's ramp is the needle's amber from 0.8 of it down
#: to near black, so it reads as where the needle has been, fading out.
SWEEP_RAMP = tuple(tuple(int(ch * (0.1 + 0.7 * i / (SWEEP_STEPS - 1)))
                         for ch in (255, 176, 0)) for i in range(SWEEP_STEPS))

INK = dict([(CROSS, (20, 52, 58)), (BEZEL, 23), (FACE, 30), (MICRO, 23), (MINOR, 30),
            (MAJOR, 44), (SEG_OFF, (16, 46, 50)), (NOTCH_MARK, 44),
            (SEG_LOW, BAND_INK[0]), (SEG_OK, BAND_INK[1]), (SEG_HOT, BAND_INK[2]),
            (SEG_LEAD, (220, 255, 250)), (NEEDLE, ansi.AMBER), (HUB, 44)]
           + list(zip(SWEEP, SWEEP_RAMP)))
#: A sub-dial's kana name.
NAME_INK = (40, 120, 130)

#: The graduation numbers. Ash, like every other caption here: they name
#: the scale and the reading is what the eye is meant to find.
LABEL_INK = ansi.ASH


#: The scales: the die -40 to 150 C (A1335 datasheet), the field 0-1200 G.
#: Each in three bands, blue under normal, green, red past (bench
#: 2026-09-07): the die's normal is where this board works, 15-65 C (the
#: thermal ramp read a room-temperature die as cold); the field's is the
#: datasheet's recommended 300-1000 G - weak or absent under, too close past.
DIE_RANGE = (-40.0, 150.0)
DIE_TICKS = (-40, 0, 50, 100, 150)
DIE_BAND = (15.0, 65.0)
FIELD_RANGE = (0.0, 1200.0)
FIELD_TICKS = (0, 300, 600, 900, 1200)
FIELD_BAND = (300.0, 1000.0)
#: (range, band, ticks) of each, as a sub-dial takes them.
FIELD = (FIELD_RANGE, FIELD_BAND, FIELD_TICKS)
DIE = (DIE_RANGE, DIE_BAND, DIE_TICKS)


class _Geometry:

    """Where everything on one face goes, in dots."""

    def __init__(self, width, height):
        self.cx = width * DOTS_X / 2.0 - 0.5
        self.cy = height * DOTS_Y / 2.0 - 0.5
        # Bounded by whichever way round the box is tighter, less the room the
        # numbers need outside the rim.
        self.rim = (min(width * DOTS_X, height * DOTS_Y) / 2.0
                    - LABEL_GAP - DOTS_Y - 1.0)
        # A dot at most: grown with the rim it made the rim a fuzzy band (2026-09-24).
        self.line = max(0.8, min(1.0, self.rim * 0.020))
        self.label = self.rim + LABEL_GAP + DOTS_Y / 2.0
        self.needle = self.rim - MAJOR_TICK - NEEDLE_CLEAR
        # The sub-dials, (centre's height, radius, range, band, ticks), or none.
        r, off = self.rim * SUB_R, self.rim * SUB_OFF
        self.subs: tuple = (((off, r) + FIELD, (-off, r) + DIE) if self.rim >= SUB_RIM else ())


def _sweep_span(degrees):
    """The reading as a span from zero, counter-clockwise, in radians."""
    return math.radians(degrees % 360.0)


#: A sample in the band the sweep may light, decided per reading.
_SWEEP_BAND = object()
#: A sample in a sub-dial's disc that nothing of its own lights: air the needle
#: passes under.
_UNDER = object()

#: The face's sample tables by (width, height, aspect) - one per size
#: a page draws at.
_FACES = {}


def _fixed(radius, phi, geom):
    """What the face alone puts at a sample, in the order the drawing layers
    them: the hub, a graduation, the rim, the band the sweep may light,
    or nothing.
    """
    if radius <= HUB_R:
        return HUB if radius >= HUB_R - RING else None
    ticks = ((30, MAJOR_TICK, MAJOR_WIDE, MAJOR), (6, MINOR_TICK, 0.0, MINOR))
    if geom.rim >= MICRO_RIM:
        ticks += ((3, MICRO_TICK, 0.0, MICRO),)
    for step, depth, wide, cls in ticks:
        pitch = math.radians(step)
        off = abs(((phi + pitch / 2.0) % pitch) - pitch / 2.0)
        if (off * radius <= max(wide, geom.line) / 2.0
                and geom.rim - depth <= radius <= geom.rim):
            return cls
    if abs(radius - geom.rim) <= geom.line:
        return FACE
    if abs(radius - geom.rim - BEZEL_GAP) <= geom.line * 0.6:
        return BEZEL
    if geom.rim - SWEEP_IN <= radius <= geom.rim - SWEEP_OUT:
        return _SWEEP_BAND
    dx, dy = radius * math.cos(phi), radius * math.sin(phi)
    for which, (oy, r, span, band, _ticks) in enumerate(geom.subs):
        cls = _sub_fixed(dx, dy - oy, r, span, band, which)
        if cls is not None:
            return cls
    if HUB_R + 2.0 < radius < geom.rim - SWEEP_IN - 2.0 and int(radius) % CROSS_PITCH == 0:
        if abs(dy) <= 0.5 or (not geom.subs and abs(dx) <= 0.5):
            return CROSS
    return None


def _sub_share(value, span):
    return max(0.0, min(1.0, (value - span[0]) / (span[1] - span[0])))


def _sub_fixed(sx, sy, r, span, band, which):
    """What sub-dial `which` puts at (sx, sy) off its centre: ('seg', which, k)
    for its k-th segment - lit or not is the reading's - a notch at a band edge,
    _UNDER for the rest of its disc, which the needle passes under, or None off
    it. Its scale runs SUB_ARC clockwise from its lower left."""
    sr = math.hypot(sx, sy)
    if sr > r + 1.0 + NOTCH:
        return None
    turned = (90.0 + SUB_ARC / 2.0 - math.degrees(math.atan2(sy, sx))) % 360.0
    if sr < r - SEG_DEPTH or turned > SUB_ARC:
        return _UNDER
    if sr > r:
        for value in band:
            off = math.radians(abs(turned - SUB_ARC * _sub_share(value, span)))
            if off * sr <= 0.6 and sr >= r + 1.0:
                return NOTCH_MARK
        return _UNDER
    pitch = SUB_ARC / SEGMENTS
    k = min(SEGMENTS - 1, int(turned / pitch))
    return ('seg', which, k) if turned - k * pitch <= SEG_FILL * pitch else _UNDER


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
                if radius > geom.rim + BEZEL_GAP + geom.line + 1.0:
                    continue
                phi = math.atan2(dy, dx) % math.tau
                samples.append((dx, dy, radius, phi,
                                _fixed(radius, phi, geom)))
            if samples:
                made.append((x, y, tuple(samples)))
    return made


#: `_samples` as numpy arrays by the same key: what a frame classifies at once.
_ARRAYS = {}


def _arrays(width, height, aspect):
    """The face's samples as arrays, one entry a sample - `dot` its dot's index,
    `dx dy phi`, `base` what it is past the needle and the sweep (-1 air),
    `needle` whether the needle may take it, `sweep` in the band, `seg` its
    sub-dial segment (which * SEGMENTS + k, else -1) - and one a dot: `cell`
    (-1 off the frame) and `bit`."""
    return table(_ARRAYS, (width, height, aspect), lambda: _arrayed(width, height, aspect))


def _arrayed(width, height, aspect):
    from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
    cols = {k: [] for k in ('dot', 'dx', 'dy', 'phi', 'base', 'needle', 'sweep', 'seg', 'cell', 'bit')}
    for i, (x, y, samples) in enumerate(_samples(width, height, aspect)):
        col, row = x // DOTS_X, y // DOTS_Y
        cols['cell'].append(row * width + col if 0 <= row < height and 0 <= col < width else -1)
        cols['bit'].append(BRAILLE_BITS[x % DOTS_X][y % DOTS_Y])
        for dx, dy, _radius, phi, fixed in samples:
            seg = fixed[1] * SEGMENTS + fixed[2] if isinstance(fixed, tuple) else -1
            plain = not (seg >= 0 or fixed is _UNDER or fixed is _SWEEP_BAND or fixed is None)
            for key, value in (('dot', i), ('dx', dx), ('dy', dy), ('phi', phi), ('seg', seg),
                               ('base', fixed if plain else -1),
                               ('needle', seg < 0 and fixed is not _UNDER
                                and fixed not in (HUB, NOTCH_MARK)),
                               ('sweep', fixed is _SWEEP_BAND)):
                cols[key].append(value)
    return {k: np.array(v) for k, v in cols.items()}


def _classes(a, geom, span, needle, lit):
    """What is at every sample this reading, a class each, -1 for air; `lit` each
    sub-dial's segments lit and their classes, or None where it has no reading.
    The hub, a notch and a sub-dial's disc outrank the needle; the needle the
    sweep."""
    from coaxial.model.blocks import numpy as np
    cls = a['base'].copy()
    hit = np.zeros(len(cls), bool)
    if needle is not None:
        # On the needle's tapered shaft: `along` how far out, `across` how far
        # off the line, the half width a function of the first.
        c, s, _tip_x, _tip_y = needle
        along = a['dx'] * c + a['dy'] * s
        across = abs(-a['dx'] * s + a['dy'] * c)
        share = along / max(1e-6, geom.needle)
        hit = (a['needle'] & (0.0 <= along) & (along <= geom.needle)
               & (across <= NEEDLE_ROOT + (NEEDLE_TIP - NEEDLE_ROOT) * share))
        cls[hit] = NEEDLE
    if span is not None:
        # Zero to the reading, the way the angles run: SWEEP_FADE of it, to black.
        phi = a['phi']
        tail = a['sweep'] & ~hit & (0.0 < phi) & (phi <= span) & (span - phi < SWEEP_FADE)
        behind = (span - phi[tail]) / SWEEP_FADE
        cls[tail] = np.array(SWEEP)[((1.0 - behind) * (SWEEP_STEPS - 1) + 0.5).astype(int)]
    segs = a['seg'] >= 0
    if segs.any():
        flat = []
        for which in range(len(geom.subs)):
            on = lit[which] if which < len(lit) else None
            flat += [on[k] if on and k < len(on) else SEG_OFF for k in range(SEGMENTS)]
        cls[segs] = np.array(flat)[a['seg'][segs]]
    return cls


def _needle(geom, at):
    """The needle at `at` radians, laid out once a frame: its direction
    as a cosine and a sine, and its tip, where the bead sits."""
    c, s = math.cos(at), math.sin(at)
    return c, s, geom.needle * c, geom.needle * s


def _raster(degrees, width, height, weak, aspect, field=None, kelvin=None):
    """Dots, their owners, the label overlay and its inks, one entry per cell."""
    text = [[None] * width for _ in range(height)]
    inks = {}
    geom = _Geometry(width, height)
    span = None if weak else _sweep_span(degrees)
    needle = None if weak else _needle(geom, math.radians(degrees))
    celsius = None if kelvin is None else kelvin - KELVIN_AT_ZERO_C
    readings = list(zip(geom.subs, (field, celsius)))
    lit = [_segments(value, scale_span, band) if value is not None else None
           for (_oy, _r, scale_span, band, _ticks), value in readings]

    stretch = aspect / DOTS_Y * DOTS_X
    # Every sample at once: a sample at a time was 370 ms a frame at 200x60
    # (2026-09-25).
    from coaxial.model.blocks import numpy as np
    a = _arrays(width, height, aspect)
    cls = _classes(a, geom, span, needle, lit)
    count = np.bincount(a['dot'], weights=cls >= 0, minlength=len(a['cell']))
    top = np.full(len(a['cell']), -1)
    np.maximum.at(top, a['dot'], cls)
    # A dot lights when half its samples or more hit (`covered`).
    light = covered(count, len(SUBDOT)) & (count > 0) & (a['cell'] >= 0)
    bits, owners = np.zeros(width * height, int), np.full(width * height, -1)
    np.bitwise_or.at(bits, a['cell'][light], a['bit'][light])
    np.maximum.at(owners, a['cell'][light], top[light])
    dots = bits.reshape(height, width).tolist()
    owner = owners.reshape(height, width).tolist()

    # The numbers last, and only onto cells no dot reached.
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
    # Each sub-dial's reading at its centre in its band's ink, its kana name over it.
    for which, ((oy, _r, _scale_span, band, _ticks), value) in enumerate(readings):
        if value is None:
            said, ink = '--', NAME_INK
        else:
            said = ('%d G' % value) if band == FIELD_BAND else ('%.1f C' % value)
            ink = _band_ink(value, band)
        centre = cell((geom.cy - oy / stretch) / DOTS_Y - 0.5)
        for row, words, shade in ((centre - 1, SUB_NAMES[which], NAME_INK),
                                  (centre, said, ink)):
            lx = geom.cx - (len(words) - 1) * DOTS_X / 2.0
            for index, ch in enumerate(words):
                col = cell((lx + index * DOTS_X) / DOTS_X - 0.5)
                if 0 <= row < height and 0 <= col < width and not dots[row][col]:
                    text[row][col] = ch
                    inks[(row, col)] = shade
    return dots, owner, text, geom, inks


def _segments(value, span, band):
    """A sub-dial's lit segments' classes, in order, for `value`: all in the band
    the reading is in - the bar says the state, the notches where it turns - the
    leading one white-hot."""
    count = max(1, int(_sub_share(value, span) * SEGMENTS + 0.5))
    lit = SEG_LOW if value < band[0] else SEG_OK if value <= band[1] else SEG_HOT
    return [lit] * (count - 1) + [SEG_LEAD]


def render(degrees, width=64, height=23, field=None, aspect=CELL_ASPECT,
           colour=False, kelvin=None):
    """The face at `degrees`, with the reading swept from zero; the field and
    the die (`kelvin`) on their sub-dials when there is room."""
    weak = field is not None and field < WEAK_GAUSS
    dots, owner, text, _, inks = _raster(degrees, width, height, weak, aspect, field,
                                         kelvin)
    lines = []
    for row in range(height):
        cells = [(text[row][col] or chr(BRAILLE + dots[row][col]),
                  inks.get((row, col), LABEL_INK) if text[row][col]
                  else INK.get(owner[row][col]))
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


def _band_ink(value, band):
    """BAND_INK's: cool under `band`, cyan inside it, red past it."""
    return BAND_INK[0 if value < band[0] else 1 if value <= band[1] else 2]


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
                             colour=colour, kelvin=kelvin),
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
