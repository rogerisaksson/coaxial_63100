"""The machine in cross-section: stator teeth inside, magnets outside."""
import math

from . import ansi
from .ascii3d import CELL_ASPECT
from . import braille
from .graphics.raster import (BRAILLE, BRAILLE_BITS, DOTS_X, DOTS_Y, SUBDOT, table,
                     covered)

#: HOW TALL A CELL ACTUALLY IS, in units of its width - `ascii3d`'s, not
#: a second copy: it is a property of the terminal's font and every
#: renderer in this tree has to agree about it or two pictures on one
#: page are drawn at different aspects. A font that runs taller draws
#: this circle as an ellipse, tall by exactly the ratio.

#: How much of the box the machine fills. Under one: a drawing that ends
#: exactly on the frame reads as something cropped rather than something
#: drawn, and the can needs somewhere to be round in - and outside it,
#: room for the pointer's dot to be round in too, which is what set this
#: number. At 0.84 the dot's outer edge landed within a third of a dot of
#: the frame and came out flattened on one side at three o'clock. It came
#: down from 0.78 to leave GUTTERS: six bar columns to the left of the
#: machine and four to the right, which is where the thermal margins are
#: drawn - beside the thing that gets hot rather than in a box somewhere
#: else on the page.
#:
#: AND BACK TO ONE, because `layout` now hands the machine exactly the
#: columns the gutters leave it: the air is `BAR_GAP`, reserved before
#: the radius is worked out, so a fraction here would be a second helping
#: of the same margin. Every earlier value of this - 0.84, 0.78, 0.70,
#: 0.62 - was fighting a centre that sat in the middle of the box while
#: the gutters were six columns one side and four the other.
F_FIT = 1.0

#: Every radius as a fraction of the outermost, which is sized to the box
#: it is drawn in. NOT fixed dot counts: the drawing was tuned at 40x14,
#: where the can is 25 dots across and each of 28 magnets is four dots of
#: arc - few enough that the curves came out as staircases. The same
#: fractions in a box of 40x24 put the can at 40 dots and the jaggies go,
#: because the only cure for them is more dots.
F_MAGNET_OUT = 0.78
F_MAGNET_IN = 0.64
F_TOOTH_OUT = 0.56
F_TOOTH_IN = 0.30
F_BORE = 0.20
#: THE CAN IS A WALL, NOT A HOOP. One line at the outer radius read as a
#: wire circle drawn round a motor; a rotor can is a steel shell with a
#: thickness, and the two edges of it are what says so. The outer edge is
#: drawn heavier than everything else - it is the silhouette - and a
#: second, smaller circle sits just inside it - with AIR BETWEEN THEM.
#: At 1.9 line widths and 0.94 the two overlapped and came out as one
#: heavy band, which is a thicker hoop and not a wall: what says wall is
#: the gap.
F_CAN_INNER = 0.88
#: How much heavier the can's outer edge is than every other line. ONE:
#: the wall is two thin edges and the air between them, not a heavy band.
#: At 1.5 the silhouette outweighed everything inside it and the pointer
#: - the one thing on the outside that has to be found at a glance - read
#: as part of the can rather than as a mark on it.
CAN_WEIGHT = 1.0
#: How thick a drawn line is, as a fraction of the outer radius. Anything
#: that is not a magnet or a tooth is a RING rather than a filled area:
#: solid areas at this resolution read as texture, and a cross-section
#: that reads as texture says nothing. The first drawing filled the bore
#: and the whole tooth annulus and its middle could not be told from noise.
F_LINE = 0.032

#: The tooth annulus is deep for a reason: the teeth carry the phase
#: currents in their LENGTH, and at a sixth of the radius the difference
#: between a phase at half and a phase at full was two dots - a picture
#: of the magnetisation nobody could read. A third of the radius makes it
#: five, which can be seen from across a bench.
#:
#: The shortest a tooth is drawn, as a share of its full length. Not
#: zero: a phase carrying nothing is still a tooth, and teeth that
#: vanished left a stator with holes in it rather than one at rest.
TOOTH_STUB = 0.22

#: How far in from the top edge the horizontal gauge sits, in rows.
#:
#: ZERO NOW, for the reason `FLOOR_INSET` is: the caller writes the
#: scale's name on the row above the box, and a blank row between a gauge
#: and the words naming it read as the gauge belonging to the drawing.
#: The old note said NOT ZERO because hard against the frame a level
#: reads as part of it - true while there was nothing but frame up there,
#: and the title is what changed it.
GAUGE_INSET = 0

#: And none at the foot. The caller writes its labels on the row under
#: the box, and a blank row between a gauge and the words naming it read
#: as the gauge belonging to the drawing rather than to the label. The
#: top gauge keeps its row of air because what is above it is a frame,
#: not a caption.
FLOOR_INSET = 0


def _drive(amps, full=None):
    """The three phase currents as shares of full scale, or None."""
    if not amps:
        return None
    scale = full or max(abs(a) for a in amps) or 1.0
    return tuple(max(-1.0, min(1.0, a / scale)) for a in amps)


#: Where the bead rides, as a fraction of the can's radius: IN THE CAN'S
#: WALL, midway between its two edges, and its wake with it - the air
#: between the rings is a race for it to run in. It rode on the outer
#: rim, half in and half out, until the bench asked for it between the
#: two outer circles; a position indicator not attached to the thing
#: whose position it indicates indicates nothing, and the wall is the
#: rotor.
POINTER_SEAT = (1.0 + F_CAN_INNER) / 2.0

#: The bead: a ring with a dot in it - `_bead` has why a glyph, what it
#: costs, and the four dot answers that were built and not kept.
#:
#: U+0298 AND NOT U+29BF, FOR THE FONT'S SAKE. The circled bullet is
#: what the bench asked for and it is unambiguously narrow - no terminal
#: setting widens it - and it still came out squeezed to half its width
#: on the bench, three times reported. Measured: Consolas, the terminal's
#: default, has neither U+29BF nor a single braille cell, so the whole
#: drawing is rendered by the fallback font, and the fallback draws the
#: bullet into a cell whose proportions are not its own. The bilabial
#: click is the same mark - a ring round a dot - and Consolas carries it,
#: so the terminal draws it with its own metrics. Round marks it has and
#: that are narrow, for the record: `◦` `◌` `∙` `ʘ`.
#:
#: What actually shears these pages is the arrowheads and the degree
#: sign, which were East Asian ambiguous; the view has the note.
POINTER_GLYPH = chr(0x0298)

#: WHICH CELLS THE BEAD MAY COLOUR: the ones whose CENTRE it covers, plus
#: the one it sits in. A braille cell is eight dots and one colour, two
#: dots wide by four tall, against a rim two dots thick - so a cell merely
#: touched by the bead gets coloured over its whole four-dot height and
#: the mark bleeds above and below the ring it rides. Counting dots did
#: not fix that: three of the disc's nine still land in a cell it only
#: clips. Testing the cell's CENTRE bounds the coloured area by the bead's
#: own shape, which is the only thing that can - measured, ten inked cells
#: at three o'clock became two.

#: How much of a slot pitch is tooth. The rest is the slot, and it stays
#: open or the teeth merge into a ring and the count cannot be read off
#: the picture, which is the only reason to draw the stator at all.
TOOTH_FILL = 0.5

#: What owns a cell, and so what colour it takes. A cell holds dots from
#: whatever passes through it and the highest class present wins.
#:
#: THE CAN OUTRANKS THE MAGNETS, which is not what it looks like it
#: should be. A cell is two dots by four, the air gap between the magnet
#: band and the can is a couple of dots, and a cell on the silhouette
#: therefore holds some of each. With the magnets ranked above, that cell
#: took the magnet's amber and the outer ring came out yellow wherever a
#: north pole passed behind it - a rotor leaking into the stationary
#: part, which is the one thing this picture must not say.
(TRACK, BORE, YOKE, TOOTH_U, TOOTH_V, TOOTH_W, SOUTH, NORTH, CAN,
 TRUTH, POINTER, SOA_OK, SOA_WARN, SOA_TRIP, WATTS, SOA_FLASH) = range(16)

#: A THERMOMETER'S OWN COLOURS, cold to hot. Every other level here takes
#: its colour from a MARGIN - how close a node is to a ceiling it was
#: given - and the thermistor has no ceiling: it reads a temperature and
#: nothing on this board was given a limit for it. So it is coloured like
#: a thermometer instead, blue through to red, which says the one thing
#: about it that can be said.
NTC_RAMP = tuple(range(SOA_FLASH + 1, SOA_FLASH + 6))
PHASE_CLASS = (TOOTH_U, TOOTH_V, TOOTH_W)

#: The classes drawn as LINES - the rings - as against the bands and
#: teeth, which are areas. `Frame.put` has why a line takes a cell it
#: shares with an area: a line that loses its cell is a broken line, and
#: an area that loses one is a dot short at its edge.
#:
#: NOT THE SOUTH ARC. It is drawn thin, but it is a magnet: counted as a
#: line its fringe took 46 of 240 cells it shared with tooth tips over 48
#: poses, which is the rotor's colour on the stator again by another
#: door. A south pole a dot short at its edge is the fault nobody sees.
LINES = frozenset((BORE, YOKE, CAN))

#: The teeth, which the truth stroke yields to. `Frame.put` has why.
TEETH = frozenset(PHASE_CLASS)
#: What they are called, in the order the teeth take them. Here
#: rather than in the view because the drawing and the legend beside
#: it have to name the same phase the same colour.
PHASE_NAMES = ('U', 'V', 'W')

#: Two voices for the rotor, three for the stator, and the mark loudest.
#: The magnets are what moves and take the warm pair; the phases are
#: fixed furniture that still has to be told apart, which in braille it
#: can only be by colour - a cell carries one, and the two polarities sit
#: within a cell of each other.
INK = {TRACK: 237, BORE: 240, CAN: 23, YOKE: 23,
       TOOTH_U: 38, TOOTH_V: 71, TOOTH_W: 103,
       SOUTH: 94, NORTH: ansi.AMBER, TRUTH: 252, POINTER: ansi.AMBER,
       SOA_OK: 41, SOA_WARN: 178, SOA_TRIP: 196,
       #: Not a margin against a ceiling like the rest of them, so
       #: not one of their colours: this one is a quantity.
       WATTS: 45,
       #: The other half of the alarm pulse. A LEVEL THAT IS ALREADY RED
       #: cannot get redder, so a stage being held back by its own
       #: envelope looked exactly like one sitting near a limit: the
       #: pulse is the difference. Which frames take it is the caller's -
       #: this is a colour, not a clock.
       #:
       #: WITHIN THE RED FAMILY, not white against it. 231 was the
       #: loudest pair on the page and read as an emergency where the
       #: board is doing exactly what it was built to do - hold the
       #: stage back. A lighter red still says "this is moving" and
       #: leaves the shouting for something that deserves it.
       SOA_FLASH: 210}

#: The legend's own furniture - arrowheads, runs, the lines that fall to
#: a tube. A NOTCH LIGHTER THAN `TRACK`, which is the empty half of a
#: thermometer: both are things to be read past rather than read, but a
#: leader is the thread between a name and its level and disappeared
#: entirely at the track's grey.
LEADER_GREY = 243

#: Which dot rows of its cell a gauge's level fills, and which one a
#: horizontal leader runs along.
#:
#: THREE TALL SO A LEADER CAN ARRIVE IN THE MIDDLE OF IT. Two dots tall
#: there is no middle: a run either sat on the level's top row or floated
#: a dot above it, and both read as a line beside the bar rather than one
#: that lands on it. Three has a centre, and `RULE_Y` is that centre.
#:
#: THE COLUMN IT MEETS STILL RUNS THE WHOLE CELL. Clipped to start at the
#: run, the corner came out as a vertical that fell short; full height it
#: makes a T, which is the junction the bench drew.
GAUGE_Y = (0, 1, 2)
RULE_Y = 1

#: Blue at the cold end, red at the hot. The steps are wide because a
#: tube one column across cannot carry a gradient - what it can carry is
#: which band it is in.
INK.update(dict(zip(NTC_RAMP, (33, 45, 41, 178, 196))))

#: A MARK on a gauge - a burst's extreme, a held peak - above every
#: level class, because a tick that yields to the level it marks is
#: not seen. The palette's orange, the ink every page gives the thing
#: to be found: the bead on the rim, a value in a box, a mark on a
#: meter.
MARK = NTC_RAMP[-1] + 1
INK[MARK] = ansi.AMBER

#: THE BEAD'S WAKE, nearest first: the arc behind the bead is how fast
#: the can turns and which way, and it fades from the bead's own
#: orange into the south pole's brown so it reads as motion and not as
#: three more marks. Drawn over the rim it rides - a smear across the
#: can is what a moving thing looks like - and `Frame.put` lets it, as
#: it lets the truth stroke.
TRAIL = tuple(range(MARK + 1, MARK + 4))
INK.update(dict(zip(TRAIL, (208, 172, 130))))
MARKS = frozenset((TRUTH,) + TRAIL)

#: The bar classes in the order a fraction picks one: below the
#: board's throttle point, past it, at the ceiling. Which fraction
#: means which is the CALLER's - the ceilings live in the
#: calibration record and the board is what acts on them.
SOA_CLASS = (SOA_OK, SOA_WARN, SOA_TRIP)

#: Columns of air between the machine and the nearest bar. THE SAME
#: ON BOTH SIDES, which is why the placement is measured off the
#: can's own edge rather than off the frame: counted from the frame,
#: six bars and four put the two groups at different distances and
#: the page looked lopsided.
#: ONE COLUMN. Closed up entirely the tubes read as part of the drawing
#: rather than as instruments beside it - a bar chart's bars touch each
#: other, not the subject. Two was too much; this is the air that says
#: "these are separate things" and nothing more.
BAR_GAP = 1

#: A PHASE BRIGHTENS WITH ITS CURRENT. One hue each so a tooth says which
#: phase it belongs to, four steps of it so the same tooth says how hard
#: that phase is being driven - and over an electrical turn the three
#: ramps walk a third of a turn apart, which is the machine pulsing in
#: the terminal rather than a diagram of one. The dimmest step is still
#: lit: a phase carrying nothing is a phase, not a hole in the stator.
PHASE_RAMP = {TOOTH_U: (23, 30, 38, 51),
              TOOTH_V: (22, 29, 71, 84),
              TOOTH_W: (53, 90, 133, 177)}


def phase_ink(drive):
    """`INK`, with each phase at the brightness its current earns."""
    ink = dict(INK)
    if drive:
        for phase, cls in enumerate(PHASE_CLASS):
            ramp = PHASE_RAMP[cls]
            step = int(abs(drive[phase]) * (len(ramp) - 1) + 0.5)
            ink[cls] = ramp[min(len(ramp) - 1, max(0, step))]
    return ink


class _Radii:

    """The radii for one drawing, in dots, from the box it fits in."""

    def __init__(self, width, height, stretch=1.0):
        # THE HEIGHT IS MEASURED IN THE SAME UNITS AS THE WIDTH.
        self.can = (min(width * DOTS_X, height * DOTS_Y * stretch) / 2.0
                    - 1.0) * F_FIT
        self.magnet_out = self.can * F_MAGNET_OUT
        self.magnet_in = self.can * F_MAGNET_IN
        # THE TEETH REACH THEIR FULL FRACTION.
        self.tooth_out = self.can * F_TOOTH_OUT
        self.tooth_in = self.can * F_TOOTH_IN
        self.bore = self.can * F_BORE
        self.can_inner = self.can * F_CAN_INNER
        self.line = max(0.8, self.can * F_LINE)

    def ring(self, radius, at, weight=1.0):
        """How much of a dot at `radius` the ring at `at` covers, 0 to 1."""
        edge = self.line * weight + 0.5 - abs(radius - at)
        return 0.0 if edge <= 0.0 else (1.0 if edge >= 1.0 else edge)


def _magnet_class(radius, phi, rotor, poles, r):
    """North solid, south a thin arc, or None between them."""
    place = ((phi - rotor) % math.tau) / (math.tau / poles)
    index, into = int(place), place - int(place)
    if into < 0.1 or into > 0.9:            # the break between magnets
        return None, 0.0
    if index % 2 == 0:
        return NORTH, 1.0
    cover = r.ring(radius, (r.magnet_in + r.magnet_out) / 2.0)
    return (SOUTH, cover) if cover else (None, 0.0)


def _stubbed(radius, r, share):
    """Whether a sample lies past the length a tooth is drawn to under
    `share` of the drive: from the inside out for a positive share, from
    the outside in for a negative one."""
    span = (r.tooth_out - r.tooth_in) * (TOOTH_STUB + (1.0 - TOOTH_STUB)
                                         * abs(share))
    if share >= 0.0:
        return radius > r.tooth_in + span
    return radius < r.tooth_out - span


def _tooth_class(radius, phi, slots, r, drive):
    """The phase of the tooth at `phi`, or None for the slot beside it."""
    if not r.tooth_in <= radius <= r.tooth_out:
        return None, 0.0
    place = (phi % math.tau) / (math.tau / slots)
    if place - int(place) > TOOTH_FILL:
        return None, 0.0
    phase = int(place) % 3
    if drive is not None and _stubbed(radius, r, drive[phase]):
        return None, 0.0
    # A TOOTH IS A FILLED AREA, so what bounds it is its angle and its length,
    # not a stroke - a sample is inside it or it is not, and the supersampling
    # in `_body` is what softens those edges.
    return PHASE_CLASS[phase], 1.0


#: What a sample in a seat's table is: a vote a ring casts with its
#: coverage, a tooth-band sample the drive decides, a magnet-band sample
#: the rotor decides.
_FIXED, _TOOTH, _MAGNET = 0, 1, 2

#: Each seat's sample table, by the numbers that place the machine in
#: its box - one per size and cell aspect a page draws at.
_SEATS = {}


def _samples(frame, seat):
    """Every dot the machine reaches, with its four samples in `SUBDOT`
    order, each `(kind, a, b)`: a fixed vote `(class, coverage)`, or the
    `(radius, angle)` of a sample in the tooth band or the magnet band.
    """
    r = seat.radii
    key = (frame.width, frame.height, seat.cx, seat.cy, seat.stretch,
           r.can, r.line)
    return table(_SEATS, key, lambda: _sampled(frame, seat))


def _sampled(frame, seat):
    """The table `_samples` keeps, built by walking every dot's samples
    through the seat's rings and bands."""
    r = seat.radii
    made = []
    for y in range(frame.height * DOTS_Y):
        for x in range(frame.width * DOTS_X):
            samples = []
            for ox, oy in SUBDOT:
                dx = x + ox - seat.cx
                dy = (seat.cy - y - oy) * seat.stretch
                radius, phi = math.hypot(dx, dy), math.atan2(dy, dx)
                cover = r.ring(radius, r.bore)
                if cover:
                    samples.append((_FIXED, BORE, cover))
                    continue
                cover = r.ring(radius, r.tooth_in)
                if cover:
                    samples.append((_FIXED, YOKE, cover))
                    continue
                if radius <= r.tooth_out:
                    if r.tooth_in <= radius:
                        samples.append((_TOOTH, radius, phi))
                    continue
                if r.magnet_in <= radius <= r.magnet_out:
                    samples.append((_MAGNET, radius, phi))
                    continue
                cover = max(r.ring(radius, r.can, CAN_WEIGHT),
                            r.ring(radius, r.can_inner))
                if cover:
                    samples.append((_FIXED, CAN, cover))
            if samples:
                made.append((x, y, tuple(samples)))
    return made


def layout(width, height, n_left=0, n_right=0, rows=None, stretch=1.0):
    """Where everything goes: `(centre_x, radii, left_cols, right_cols)`."""
    lead = (n_left + BAR_GAP) if n_left else 0
    trail = (n_right + BAR_GAP) if n_right else 0
    room = max(1, width - lead - trail)
    # THE CAN IS SIZED AGAINST ITS OWN BAND, not the whole box.
    r = _Radii(room, rows if rows else height, stretch)
    cx = (lead + room / 2.0) * DOTS_X - 0.5
    left = [c for c in (lead - BAR_GAP - 1 - i for i in range(n_left))
            if 0 <= c < width]
    right = [c for c in (width - trail + BAR_GAP + i for i in range(n_right))
             if 0 <= c < width]
    return cx, r, left, right


def gutters(width, height, n_left, n_right):
    """Which columns the margin bars land in, `(left, right)`."""
    return layout(width, height, n_left, n_right)[2:]


def span(width, height, n_left=0, n_right=0, rows=None):
    """The columns the machine itself occupies, first and last."""
    cx, r, _, _ = layout(width, height, n_left, n_right, rows)
    return (int(math.floor((cx - r.can) / DOTS_X)),
            int(math.floor((cx + r.can) / DOTS_X)))


def _gauge(dots, owner, width, height, row, share, cls,
           n_left=0, n_right=0, part=None):
    """One horizontal level across the MACHINE'S width, from the left."""
    if row < 0 or row >= height:
        return
    first, last = span(width, height, n_left, n_right)
    if part is not None:
        # ONE OF SEVERAL ACROSS THE SAME WIDTH.
        index, count = part
        step = (last - first + 1) / float(max(1, count))
        first, last = (int(first + index * step),
                       int(first + (index + 1) * step) - 1)
    lo, hi = max(0, first) * DOTS_X, min(width - 1, last) * DOTS_X + DOTS_X
    filled = int(max(0.0, min(1.0, share)) * (hi - lo) + 0.5)
    _level(dots, owner, row, lo, hi, lo, lo + filled, cls)


def _level(dots, owner, row, lo, hi, start, end, cls):
    """A horizontal level on `row`, in DOT columns: the scale runs `lo` to
    `hi` (exclusive) and the level fills `start` to `end` in `cls`; the
    rest of the scale is track.
    """
    ends = ({start // DOTS_X, (end - 1) // DOTS_X} if end > start
            else set())
    for x in range(lo, hi):
        col = x // DOTS_X
        if start <= x < end:
            for y in GAUGE_Y:
                dots[row][col] |= BRAILLE_BITS[x % DOTS_X][y]
            owner[row][col] = max(owner[row][col], cls)
        elif x % DOTS_X == 0 and col not in ends:
            for y in GAUGE_Y:
                dots[row][col] |= BRAILLE_BITS[x % DOTS_X][y]
            owner[row][col] = max(owner[row][col], TRACK)


def _mark(dots, owner, row, x, cls, ys=GAUGE_Y):
    """A tick at dot column `x`, on dot rows `ys` - a level's height for a
    burst's extreme, the top dot alone for a held peak.
    """
    col = x // DOTS_X
    if 0 <= row < len(dots) and 0 <= col < len(dots[row]):
        for y in ys:
            dots[row][col] |= BRAILLE_BITS[x % DOTS_X][y]
        owner[row][col] = cls


def _bars(dots, owner, width, height, left, right, r, floors=1, reserve=0,
          has_top=True):
    """Vertical margin bars, filled from the bottom, one cell wide."""
    # THE FIRST ROW AND THE LAST FEW BELONG TO THE GAUGES.
    top_row = GAUGE_INSET + (1 if has_top else 0) + reserve
    tall = max(1, height - GAUGE_INSET - FLOOR_INSET - reserve
               - (1 if has_top else 0) - max(1, floors)) * DOTS_Y
    # Floor one side and ceil the other inside `gutters`: the centre sits
    # between two columns, so flooring both put the machine's right edge half a
    # column further out than its left and the gaps came out 1 and 0.
    _, _, at_left, at_right = layout(width, height,
                                     len(left or ()), len(right or ()))
    for bars, columns in ((left, at_left), (right, at_right)):
        for index, entry in enumerate(bars or []):
            if index >= len(columns) or entry is None:
                # A None is a SPACER: it takes a column and draws nothing,
                # which is how a caller puts air between two groups of bars
                # that measure different things.
                continue
            share, cls = entry
            _tube(dots, owner, columns[index], top_row * DOTS_Y, tall,
                  share, cls)


def _tube(dots, owner, col, top, tall, share, cls):
    """One thermometer in cell column `col`: a tube `tall` dots high from
    dot row `top`, the mercury `share` of it from the bottom in `cls`,
    the rest track.
    """
    filled = int(max(0.0, min(1.0, share)) * tall + 0.5)
    edge = (top + tall - filled) // DOTS_Y if filled else -1
    for step in range(tall):
        y = top + tall - 1 - step
        row = y // DOTS_Y
        if not (0 <= row < len(dots)):
            continue
        if step < filled:
            for lane in range(DOTS_X):
                dots[row][col] |= BRAILLE_BITS[lane][y % DOTS_Y]
            owner[row][col] = max(owner[row][col], cls)
        elif y % DOTS_Y in (1, 3) and row != edge:
            for lane in range(DOTS_X):
                dots[row][col] |= BRAILLE_BITS[lane][y % DOTS_Y]
            owner[row][col] = max(owner[row][col], TRACK)


def _overlay(dots, text, width, height, labels, leaders, rules):
    """Leaders in dots and names in text, over cells no drawing reached."""
    # THE LEADERS, IN DOTS, AND THEY FALL.
    lit = []
    met = {}
    for row, from_col, to_col, _shade in list(rules or []):
        lo, hi = min(from_col, to_col), max(from_col, to_col)
        met.setdefault(row, []).append((lo, hi))
    for entry in list(leaders or []):
        from_row, col, to_row, shade = entry[:4]
        # WHICH HALF OF THE CELL IT FALLS DOWN.
        lane = entry[4] if len(entry) > 4 else 0
        for row in range(from_row, to_row):
            if not (0 <= row < height and 0 <= col < width):
                continue
            # A HOOK WHERE IT MEETS A RULE, not a bar through it.
            turn = any(lo <= col <= hi for lo, hi in met.get(row, ()))
            # A CORNER WHERE IT MEETS A RULE, a stroke where it does not - and
            # how far down the corner reaches depends on whether the line
            # carries on.
            if turn:
                dots[row][col] |= braille.mask(braille.lit(
                    braille.corner(RULE_Y, lane,
                                   through=row + 1 < to_row)))
            else:
                dots[row][col] |= braille.mask(
                    (lane, y) for y in range(DOTS_Y))
                lit.append((row, col, shade))

    # AND THE HORIZONTAL HALF OF THE SAME FURNITURE.
    for row, from_col, to_col, shade in list(rules or []):
        for col in range(min(from_col, to_col), max(from_col, to_col) + 1):
            if 0 <= row < height and 0 <= col < width:
                for x in range(DOTS_X):
                    dots[row][col] |= BRAILLE_BITS[x][RULE_Y]
                lit.append((row, col, shade))

    # THE OVERLAY LAST, and only where no dot went.
    for row, col, said, _ink in list(labels or []):
        for step, ch in enumerate(said):
            here = col + step
            if 0 <= row < height and 0 <= here < width and not dots[row][here]:
                text[row][here] = ch
    return lit


class Frame:

    """The grid a drawing is built in: dots, who owns each cell, and the
    text laid over them.
    """

    def __init__(self, width, height):
        self.width, self.height = width, height
        self.dots = [[0] * width for _ in range(height)]
        self.owner = [[-1] * width for _ in range(height)]
        self.text = [[None] * width for _ in range(height)]
        #: How many dots each class has lit in each cell, so a cell can
        #: belong to what is mostly in it.
        self.tally: list = [[None] * width for _ in range(height)]

    def put(self, x, y, cls):
        """Light one dot, in DOT coordinates."""
        col, row = int(x) // DOTS_X, int(y) // DOTS_Y
        if not (0 <= row < self.height and 0 <= col < self.width):
            return
        self.dots[row][col] |= BRAILLE_BITS[int(x) % DOTS_X][int(y) % DOTS_Y]
        if cls is None:
            return
        tally = self.tally[row][col]
        if tally is None:
            tally = self.tally[row][col] = {}
        tally[cls] = tally.get(cls, 0) + 1
        # THE TRUTH STROKE FIRST, the one thing drawn to be FOUND: yielding to
        # the rings it owned no cell at all in some poses, so at its own angle
        # a ring cell goes white and it reads as reaching the rim.
        running = ([c for c in tally if c != TRUTH]
                   if set(tally) & TEETH else tally)
        self.owner[row][col] = max(
            running, key=lambda c: (c in MARKS, c in LINES,
                                    tally[c], c))

    def claim(self, row, col, cls, said=None):
        """Give a CELL to `cls`, and a character with it where the mark
        cannot be made of dots."""
        if not (0 <= row < self.height and 0 <= col < self.width):
            return
        self.owner[row][col] = cls
        if said is not None:
            self.text[row][col] = said

    def lines(self, ink, colour=False, tint=None):
        """THE ONE PLACE THIS BECOMES TERMINAL OUTPUT."""
        at = dict(tint or {})
        out = []
        for row in range(self.height):
            cells = [(self.text[row][col]
                      or chr(BRAILLE + self.dots[row][col]),
                      at[(row, col)] if (row, col) in at
                      else ink.get(self.owner[row][col]))
                     for col in range(self.width)]
            out.append(ansi.run(cells) if colour
                       else ''.join(char for char, _ in cells))
        return out


class Seat:

    """Where the machine sits in its box, and what is left around it."""

    def __init__(self, width, height, left, right, top, bottom,
                 labels, leaders, aspect):
        self.floors = max(1, len(list(bottom or ())))
        # A LABEL'S ROW IS WRITTEN ON, A LEADER'S `to_row` IS PAST ITS LAST:
        # the two are one column of arithmetic with different ends, and the can
        # has to start under both.
        written = [row + 1 for row, _col, _said, _ink in list(labels or [])]
        written += [entry[2] for entry in list(leaders or [])
                    if entry[0] == 0]
        self.reserve = max(written) if written else 0
        self.band = max(1, height - self.floors - self.reserve)
        # A DOT IS SQUARE ONLY WHEN A CELL IS TWO BY ONE.
        self.stretch = aspect / DOTS_Y * DOTS_X
        self.cx, self.radii, _, _ = layout(
            width, height, len(left or ()), len(right or ()), rows=self.band,
            stretch=self.stretch)
        # AND SEATED AT THE TOP OF THAT BAND, its first dot in the first row
        # under the reserve.
        self.cy = (self.reserve * DOTS_Y + 0.5
                   + self.radii.can / self.stretch)


def _body(frame, seat, rotor_deg, slots, poles, drive):
    """The machine itself, dot by dot."""
    rotor = math.radians(rotor_deg)
    r = seat.radii
    for x, y, samples in _samples(frame, seat):
        # EACH SAMPLE VOTES WITH ITS COVERAGE, and the dot goes to the class
        # that covers most of it.
        votes = {}
        for kind, a, b in samples:
            if kind == _FIXED:
                at, share = a, b
            elif kind == _TOOTH:
                at, share = _tooth_class(a, b, slots, r, drive)
            else:
                at, share = _magnet_class(a, b, rotor, poles, r)
            if at is not None:
                votes[at] = votes.get(at, 0.0) + share
        if votes and covered(sum(votes.values()), len(SUBDOT)):
            frame.put(x, y, max(votes, key=lambda c: (votes[c], c)))


def _bead(frame, seat, pointer_deg, glyph=None, rate=None):
    """The bench's own zero, riding the can's rim."""
    phi = math.radians(pointer_deg)
    # IN THE WALL, between the can's two edges, so the rings stay whole and the
    # bead runs in the race between them.
    seat_r = seat.radii.can * POINTER_SEAT
    at_x = seat.cx + seat_r * math.cos(phi)
    at_y = seat.cy - seat_r * math.sin(phi) / seat.stretch
    # THE NEAREST CELL, MEASURED FROM ITS CENTRE.
    col = int(math.floor((at_x - (DOTS_X - 1) / 2.0) / DOTS_X + 0.5))
    row = int(math.floor((at_y - (DOTS_Y - 1) / 2.0) / DOTS_Y + 0.5))
    frame.claim(row, col, POINTER, glyph or POINTER_GLYPH)
    if rate:
        _wake(frame, seat, phi, seat_r, rate, (row, col))


#: The wake's shutter, seconds of travel the trail shows. At 60 rpm
#: seven hundredths of a second is 25 degrees of rim, seen from across
#: a bench; at a crawl it is a few dots; and the cap keeps a fast can
#: from wearing a ring, which would say nothing about which way it
#: turns. A TENTH AND 120 FIRST, then "a shade narrower and shorter"
#: from the bench: shorter is the shutter and the cap, narrower is the
#: dots a dot apart rather than half a dot, a lighter line.
TRAIL_S = 0.07
TRAIL_MAX_DEG = 90.0
TRAIL_PITCH = 1.0


def _wake(frame, seat, phi, seat_r, rate, bead_cell):
    """The trail behind the bead: an arc on the rim, TRAIL_S of travel long
    at `rate` degrees a second, on the side the bead came from, fading in
    thirds through `TRAIL`.
    """
    length = min(TRAIL_MAX_DEG, abs(rate) * TRAIL_S)
    if length <= 0.0:
        return
    back = -1.0 if rate > 0.0 else 1.0
    steps = max(2, int(seat_r * math.radians(length) / TRAIL_PITCH) + 1)
    for i in range(1, steps + 1):
        t = i / float(steps)
        a = phi + back * math.radians(length) * t
        x = seat.cx + seat_r * math.cos(a)
        y = seat.cy - seat_r * math.sin(a) / seat.stretch
        if (int(y) // DOTS_Y, int(x) // DOTS_X) == bead_cell:
            continue
        frame.put(x, y, TRAIL[min(len(TRAIL) - 1, int(t * len(TRAIL)))])


def _truth(frame, seat, truth_deg):
    """The angle a shaft sensor says, as a tick in the AIR GAP."""
    phi = math.radians(truth_deg)
    r = seat.radii
    # THROUGH THE MAGNET BAND, which is the thing it is read against.
    inner, outer = r.magnet_in + 1.5, r.magnet_out - 0.5
    for step in range(int((outer - inner) * 4) + 1):
        radius = inner + step * 0.25
        frame.put(seat.cx + radius * math.cos(phi),
                  seat.cy - radius * math.sin(phi) / seat.stretch, TRUTH)


def _machine(frame, seat, rotor_deg, slots, poles, drive,
             truth_deg=None, pointer_deg=None, bead=None, pointer_rate=None):
    """THE MACHINE AND NOTHING ELSE: the cross-section, the bench's mark on
    the rim, and the tick a shaft sensor claims.
    """
    _body(frame, seat, rotor_deg, slots, poles, drive)
    if truth_deg is not None:
        _truth(frame, seat, truth_deg)
    if pointer_deg is not None:
        _bead(frame, seat, pointer_deg, bead, pointer_rate)


def _instruments(frame, seat, left, right, top, bottom):
    """The gutters and the gauges: everything measured AGAINST the machine
    rather than part of it.
    """
    floor = list(bottom or ())
    _bars(frame.dots, frame.owner, frame.width, frame.height, left, right,
          seat.radii, len(floor), reserve=seat.reserve, has_top=bool(top))
    for index, gauge in enumerate(list(top or ())):
        _gauge(frame.dots, frame.owner, frame.width, frame.height,
               GAUGE_INSET, gauge[0], gauge[1],
               len(left or ()), len(right or ()), part=(index, len(top)))
    for index, gauge in enumerate(floor):
        _gauge(frame.dots, frame.owner, frame.width, frame.height,
               frame.height - FLOOR_INSET - len(floor) + index,
               gauge[0], gauge[1], len(left or ()), len(right or ()))


def motor(rotor_deg, slots=24, poles=28, width=40, height=22, drive=None,
          truth_deg=None, pointer_deg=None, aspect=CELL_ASPECT,
          colour=False, bead=None, pointer_rate=None):
    """The machine alone, as text rows - no gutters, no gauges, no legend."""
    frame = Frame(width, height)
    seat = Seat(width, height, None, None, None, None, None, None, aspect)
    _machine(frame, seat, rotor_deg, slots, poles, drive,
             truth_deg=truth_deg, pointer_deg=pointer_deg, bead=bead,
             pointer_rate=pointer_rate)
    return frame.lines(phase_ink(drive), colour=colour)


def _raster(rotor_deg, slots, poles, width, height, truth_deg, drive,
            pointer_deg, left, right, top, bottom, aspect, labels=None,
            leaders=None, rules=None, bead=None, pointer_rate=None):
    """The whole page: the machine, its instruments, and the legend over
    both.
    """
    frame = Frame(width, height)
    seat = Seat(width, height, left, right, top, bottom, labels, leaders,
                aspect)
    _machine(frame, seat, rotor_deg, slots, poles, drive,
             truth_deg=truth_deg, pointer_deg=pointer_deg, bead=bead,
             pointer_rate=pointer_rate)
    _instruments(frame, seat, left, right, top, bottom)
    lit = _overlay(frame.dots, frame.text, width, height, labels, leaders,
                   rules)
    return frame, lit



def render(rotor_deg, slots=24, poles=28, width=40, height=22,
           truth_deg=None, amps=None, full=None, pointer_deg=None,
           left=None, right=None, top=None, bottom=None,
           aspect=CELL_ASPECT, colour=False, labels=None, leaders=None,
           rules=None, bead=None, pointer_rate=None):
    """The cross-section, `rotor_deg` being how far the can has turned."""
    poles = max(2, int(poles) - int(poles) % 2)
    slots = max(3, int(slots))
    drive = _drive(amps, full)
    frame, lit = _raster(rotor_deg, slots, poles, width, height,
                         truth_deg, drive, pointer_deg, left, right,
                         top, bottom, aspect, labels, leaders, rules,
                         bead, pointer_rate)
    # THE ONLY THING LEFT HERE IS WHO GETS WHICH COLOUR.
    at = {}
    for row, col, said, said_ink in list(labels or []):
        for step in range(len(said)):
            at[(row, col + step)] = said_ink
    for row, col, shade in lit:
        at.setdefault((row, col), shade)
    return '\n'.join(frame.lines(phase_ink(drive), colour=colour, tint=at))


def caption(slots, poles, rotor_deg, slipped=None):
    """One line naming the machine drawn and where the can is."""
    text = '%dN%dP  can %5.1f deg' % (slots, poles, rotor_deg % 360.0)
    if slipped:
        text += '  %+d magnets' % slipped
    return text
