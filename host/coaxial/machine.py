"""The machine in cross-section: stator teeth inside, magnets outside.

An outrunner is drawn the way it is built - the rotor is the *outside*,
a steel can carrying the magnets, turning around a stator whose teeth
face outwards. That is why the PCB sits coaxially behind the stator: the
middle of this picture is where the electronics are.

What the picture is for is the thing a number cannot say: WHICH TOOTH IS
UNDER WHICH MAGNET. An angle of 296.8 degrees electrical is one
fourteenth of a turn on a 28-pole machine, and a reader watching an
estimate cross zero has no way to tell a real commutation from an
observer that has slipped a pole. Here a slip is visible: the magnet band
steps round and the teeth stay where they are.

SLOTS AND POLES ARE DIFFERENT KINDS OF FACT. The pole count comes off
the wire - `motor_pole_pairs` in the calibration record, doubled - so
this draws whatever machine the record describes. The SLOT count is not
in the record, is not on the wire, and cannot be inferred from the pole
count: 24N28P is a common combination and so is 21N28P. It is therefore
a parameter of the drawing with a stated default, not a measurement, and
`--slots` is how a bench says what is actually on the shaft.

DOTS, the same braille matrix `wireframe` rasters the board into, for the
reason recorded beside it in `raster`: an ASCII stroke set was built
alongside that one and taken out on the bench's word. A picture that is
dots in one view and hashes and slashes in the next is two drawing
conventions in one terminal.

Pure: numbers in, a block of text out. No serial port, no terminal, no
clock. Angles run the way the rest of the tree reports them - zero to the
right, increasing counter-clockwise - which is a drawing convention and
not a claim about which way the shaft turns (invariant 10).
"""
import math

from . import ansi
from .ascii3d import CELL_ASPECT
from . import braille
from .raster import (BRAILLE, BRAILLE_BITS, DOTS_X, DOTS_Y, SUBDOT,
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
    """The three phase currents as shares of full scale, or None.

    Normalised HERE and not by the caller so every drawing means the same
    thing: a share of one is a tooth at full length. `full` defaults to
    the largest of the three, which shows the SHAPE of the excitation when
    nobody has said what full scale is - the three-phase sequence is
    visible either way, and only the absolute size is not.
    """
    if not amps:
        return None
    scale = full or max(abs(a) for a in amps) or 1.0
    return tuple(max(-1.0, min(1.0, a / scale)) for a in amps)


#: How far the bead stands proud of the can's outer edge, in dots. It
#: rides ON the rim rather than clear of it: a position indicator not
#: attached to the thing whose position it indicates indicates nothing.
POINTER_SEAT = 0.6

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
        # THE HEIGHT IS MEASURED IN THE SAME UNITS AS THE WIDTH. Radii
        # are x-dots and the drawing scales y by `stretch`, so a box
        # `height` rows tall holds `height * DOTS_Y * stretch` x-dots of
        # can - not `height * DOTS_Y`. Without it a cell taller than two
        # by one left the can sized as if its rows were shorter than
        # they are, and a band fitted to the can then shrank it.
        self.can = (min(width * DOTS_X, height * DOTS_Y * stretch) / 2.0
                    - 1.0) * F_FIT
        self.magnet_out = self.can * F_MAGNET_OUT
        self.magnet_in = self.can * F_MAGNET_IN
        # THE TEETH REACH THEIR FULL FRACTION. They were clamped two dots
        # short so that no cell could hold both a magnet and a tooth tip
        # - the gap is 0.08 of the radius, 2.6 dots against a cell four
        # tall, and at twelve and six o'clock the shared cell took the
        # magnet's amber. Clean, and the bench read it as the slots
        # drawn too small, which they were. At this resolution it is one
        # or the other; the picture is of the stator's current, so the
        # teeth keep their length and the shared cell goes to the tooth
        # (`Frame.put`). A magnet a dot short at its inner edge at two
        # angles is the cheaper fault.
        self.tooth_out = self.can * F_TOOTH_OUT
        self.tooth_in = self.can * F_TOOTH_IN
        self.bore = self.can * F_BORE
        self.can_inner = self.can * F_CAN_INNER
        self.line = max(0.8, self.can * F_LINE)

    def ring(self, radius, at, weight=1.0):
        """How much of a dot at `radius` the ring at `at` covers, 0 to 1.

        A COVERAGE AND NOT A YES. The band was a hard test - inside the
        half-width or out - and four corner samples then quantised a
        stroke to fifths, which on a thin ring is solid or nothing per
        dot. That is the staircase: an arc crossing a cell had no way to
        say it was only a third of the way into it.

        A dot is one unit across, so the stroke covers all of one whose
        centre is at least half a dot inside the edge, none of one half a
        dot outside, and a straight ramp between. That ramp is what draws
        `⣀` where an arc grazes the bottom of a cell, `⣤` where it is
        halfway in and `⣶` where it nearly fills it - the grading a
        braille cell can show, which a boolean cannot reach.
        """
        edge = self.line * weight + 0.5 - abs(radius - at)
        return 0.0 if edge <= 0.0 else (1.0 if edge >= 1.0 else edge)


def _magnet_class(radius, phi, rotor, poles, r):
    """North solid, south a thin arc, or None between them.

    POLARITY IS THICKNESS. A north magnet is the full depth of the band
    and a south is a line down the middle of it, which counts out as 28
    magnets and reads as 14 pole pairs at once.
    """
    place = ((phi - rotor) % math.tau) / (math.tau / poles)
    index, into = int(place), place - int(place)
    if into < 0.1 or into > 0.9:            # the break between magnets
        return None, 0.0
    if index % 2 == 0:
        return NORTH, 1.0
    cover = r.ring(radius, (r.magnet_in + r.magnet_out) / 2.0)
    return (SOUTH, cover) if cover else (None, 0.0)


def _tooth_class(radius, phi, slots, r, drive):
    """The phase of the tooth at `phi`, or None for the slot beside it.

    Teeth are numbered from zero to the right and take their phase in
    turn, which is what a three-phase machine with a slot count divisible
    by three is. A count that is not divisible by three still draws - the
    phases simply do not close on themselves, and that is the winding
    such a machine has.

    THE TOOTH IS AS LONG AS ITS PHASE IS DRIVEN, and grows from the end
    the current's sign says: outward from the yoke where the phase pushes,
    inward from the tip where it pulls. That is the magnetisation, and it
    is the reason to draw teeth rather than an angle - over one electrical
    turn the three lengths breathe in sequence a third of a turn apart,
    which is what a three-phase machine does and what no dial shows. With
    no currents given they are drawn full length, which is a machine
    nobody is asking anything of.
    """
    if not r.tooth_in <= radius <= r.tooth_out:
        return None, 0.0
    place = (phi % math.tau) / (math.tau / slots)
    if place - int(place) > TOOTH_FILL:
        return None, 0.0
    phase = int(place) % 3
    if drive is not None:
        share = drive[phase]
        span = (r.tooth_out - r.tooth_in) * (TOOTH_STUB + (1.0 - TOOTH_STUB)
                                             * abs(share))
        if share >= 0.0:
            if radius > r.tooth_in + span:
                return None, 0.0
        elif radius < r.tooth_out - span:
            return None, 0.0
    # A TOOTH IS A FILLED AREA, so what bounds it is its angle and its
    # length, not a stroke - a sample is inside it or it is not, and the
    # supersampling in `_body` is what softens those edges.
    return PHASE_CLASS[phase], 1.0


def _classify(radius, phi, rotor, slots, poles, r, drive):
    """What owns the point at (radius, phi), and how much of a dot there
    it covers - `(class, 0..1)`, or `(None, 0.0)` for air.

    THE COVERAGE TRAVELS WITH THE CLASS because only the shape knows it.
    A ring answers a ramp across its own edge; a tooth or a magnet is a
    filled area and answers one, its edges being angles that `_body`
    supersamples. Returned as a pair rather than worked out afterwards:
    the caller would have to ask which of six shapes it had hit to know
    which rule applied.
    """
    cover = r.ring(radius, r.bore)
    if cover:
        return BORE, cover
    cover = r.ring(radius, r.tooth_in)
    if cover:
        return YOKE, cover
    if radius <= r.tooth_out:
        return _tooth_class(radius, phi, slots, r, drive)
    if r.magnet_in <= radius <= r.magnet_out:
        return _magnet_class(radius, phi, rotor, poles, r)
    cover = max(r.ring(radius, r.can, CAN_WEIGHT),
                r.ring(radius, r.can_inner))
    if cover:
        return CAN, cover
    return None, 0.0


def layout(width, height, n_left=0, n_right=0, rows=None, stretch=1.0):
    """Where everything goes: `(centre_x, radii, left_cols, right_cols)`.

    ONE SOURCE FOR ALL OF IT. The span, the gutters, the raster and the
    caption each worked the geometry out again, and any two of them
    disagreeing put a scale through a thermometer or a title over a
    gutter. They all come here now.

    THE MACHINE IS CENTRED BETWEEN THE GUTTERS, not in the box. Six
    thermometers stand to its left and four to its right, so centred in
    the frame it sat nearer the left group than the right - measured, one
    column of air against two - and could not grow either, because the
    wider gutter reached the frame first. Centred on what is actually
    left over, it is symmetric AND bigger: one rule gives both.
    """
    lead = (n_left + BAR_GAP) if n_left else 0
    trail = (n_right + BAR_GAP) if n_right else 0
    room = max(1, width - lead - trail)
    # THE CAN IS SIZED AGAINST ITS OWN BAND, not the whole box. Given the
    # box it grew to fill it, so every row added for the foot gauges made
    # the can a row taller and it ran into them again - a loop that
    # cannot be escaped by changing the height, because the height was
    # the thing feeding it. `rows` is what is left after the gauges and
    # anything written above them, so the can's size follows the WIDTH
    # and the rows only change how much air is around it.
    r = _Radii(room, rows if rows else height, stretch)
    cx = (lead + room / 2.0) * DOTS_X - 0.5
    left = [c for c in (lead - BAR_GAP - 1 - i for i in range(n_left))
            if 0 <= c < width]
    right = [c for c in (width - trail + BAR_GAP + i for i in range(n_right))
             if 0 <= c < width]
    return cx, r, left, right


def gutters(width, height, n_left, n_right):
    """Which columns the margin bars land in, `(left, right)`.

    Exported because a caption has to sit over its own group and the only
    thing that knows where a group went is what put it there.
    """
    return layout(width, height, n_left, n_right)[2:]


def span(width, height, n_left=0, n_right=0, rows=None):
    """The columns the machine itself occupies, first and last.

    FLOORED AT BOTH ENDS. Ceiling the right one put its edge a column
    further out than the circle actually reached, so the gutter measured
    off it stood two columns clear where the left stood one.
    """
    cx, r, _, _ = layout(width, height, n_left, n_right, rows)
    return (int(math.floor((cx - r.can) / DOTS_X)),
            int(math.floor((cx + r.can) / DOTS_X)))


def _gauge(dots, owner, width, height, row, share, cls,
           n_left=0, n_right=0, part=None):
    """One horizontal level across the MACHINE'S width, from the left.

    Not the whole row: run edge to edge it passed above and below the
    gutter thermometers and the four corners of the box read as one
    instrument crossing another. Between the gutters it is over the
    thing it describes and the corners are empty.

    The same instrument as the gutters turned on its side, and drawn the
    same way: the level solid, the rest of the scale every other dot, so
    a reader sees how far along the scale the level is and not only that
    there is one.
    """
    if row < 0 or row >= height:
        return
    first, last = span(width, height, n_left, n_right)
    if part is not None:
        # ONE OF SEVERAL ACROSS THE SAME WIDTH. `part` is `(index, count)`
        # and the gauges share the machine's span end to end, so each
        # sits under its own name instead of a stack of bars that have to
        # be told apart by their order.
        index, count = part
        step = (last - first + 1) / float(max(1, count))
        first, last = (int(first + index * step),
                       int(first + (index + 1) * step) - 1)
    lo, hi = max(0, first) * DOTS_X, min(width - 1, last) * DOTS_X + DOTS_X
    filled = int(max(0.0, min(1.0, share)) * (hi - lo) + 0.5)
    _level(dots, owner, row, lo, hi, lo, lo + filled, cls)


def _level(dots, owner, row, lo, hi, start, end, cls):
    """A horizontal level on `row`, in DOT columns: the scale runs `lo`
    to `hi` (exclusive) and the level fills `start` to `end` in `cls`;
    the rest of the scale is track. The same instrument wherever a page
    draws a level - `gauges` calls it for every other view.

    THE CELLS THE LEVEL ENDS IN HOLD LEVEL AND NOTHING ELSE, for the
    reason `_tube` gives: a track dot inside one took the level's colour
    and the bar read a whole cell long whatever the level was. Kept
    clear, the end of the bar is drawn at the dot - one lane or two -
    and a level that moves one dot is seen to move. Both ends, because a
    bipolar gauge's level starts at its centre and not at the scale's.

    ONE COLUMN A CELL, AT THE GAUGE'S OWN HEIGHT. Every fourth dot put
    one in every other cell, so the empty half of a gauge came out as a
    dashed line with gaps a cell wide; one dot a cell on the middle row
    was a scale, but a scale a single dot tall beside a level three tall
    - the tubes' track runs the tube's whole width, and the bench asked
    for the same here: the empty half of the gauge is the gauge's own
    height, in the track's grey.
    """
    ends = ({start // DOTS_X, (end - 1) // DOTS_X} if end > start
            else set())
    for x in range(lo, hi):
        col = x // DOTS_X
        if start <= x < end:
            for y in GAUGE_Y:
                dots[row][col] |= BRAILLE_BITS[x % DOTS_X][y]
            if cls > owner[row][col]:
                owner[row][col] = cls
        elif x % DOTS_X == 0 and col not in ends:
            for y in GAUGE_Y:
                dots[row][col] |= BRAILLE_BITS[x % DOTS_X][y]
            if TRACK > owner[row][col]:
                owner[row][col] = TRACK


def _mark(dots, owner, row, x, cls, ys=GAUGE_Y):
    """A tick at dot column `x`, on dot rows `ys` - a level's height for
    a burst's extreme, the top dot alone for a held peak. Drawn over
    whatever is there and claiming the cell, because a mark that yields
    to the level it marks is not seen."""
    col = x // DOTS_X
    if 0 <= row < len(dots) and 0 <= col < len(dots[row]):
        for y in ys:
            dots[row][col] |= BRAILLE_BITS[x % DOTS_X][y]
        owner[row][col] = cls


def _bars(dots, owner, width, height, left, right, r, floors=1, reserve=0,
          has_top=True):
    """Vertical margin bars, filled from the bottom, one cell wide.

    LEFT AND RIGHT ARE THE CALLER'S SUBJECTS, not this module's: it draws
    fractions in gutters and knows nothing about what is hot. Each entry
    is `(fraction, class)`, and the class is one of `SOA_CLASS`.

    THERMOMETERS, not bars: each column is a tube the full height of the
    box with the level rising inside it. The tube is what makes the level
    mean anything - a column half the height of nothing is a number, a
    column half the height of its own ceiling is a margin.

    In the gutters and never over the machine: one drawn through the
    drawing would be a thermometer through a motor. Placed `BAR_GAP`
    columns from the CAN'S EDGE on each side, so both groups stand the
    same distance off whatever size the machine came out. Filled from the
    bottom because that is which way a level goes. The first entry of
    each side is the one nearest the machine, so both groups read
    outwards from it.
    """
    # THE FIRST ROW AND THE LAST FEW BELONG TO THE GAUGES. Run full
    # height, the tubes shared row 0 with the headroom scale drawn across
    # it and the scale appeared to run through the thermometers. `floors`
    # is how many rows are taken at the bottom - one gauge or several.
    # `reserve` is how many rows the caller wrote text into. THE TUBES
    # KEEP OUT OF IT: a label inside the drawing can only take cells no
    # dot reached, so a tube running through those rows ate the words -
    # measured, `MOTOR SOA 41 %` came out as `MOTOR SOA` with the value
    # chewed off by the board's own thermometers.
    # THE ROW AFTER THE TOP GAUGE, or the first row when there is none.
    # The `+1` was unconditional and left an empty row under the captions
    # once the headroom scales moved out of the drawing - which broke
    # every leader falling into a tube exactly where it should have
    # landed.
    top_row = GAUGE_INSET + (1 if has_top else 0) + reserve
    tall = max(1, height - GAUGE_INSET - FLOOR_INSET - reserve
               - (1 if has_top else 0) - max(1, floors)) * DOTS_Y
    # Floor one side and ceil the other inside `gutters`: the centre sits
    # between two columns, so flooring both put the machine's right edge
    # half a column further out than its left and the gaps came out 1
    # and 0. Adjacent, not spaced: six bars and four have to fit what the
    # machine leaves, and a bar chart's bars touch - what separates them
    # is their heights and their colours.
    _, _, at_left, at_right = layout(width, height,
                                     len(left or ()), len(right or ()))
    for bars, columns in ((left, at_left), (right, at_right)):
        for index, entry in enumerate(bars or ()):
            if index >= len(columns) or entry is None:
                # A None is a SPACER: it takes a column and draws
                # nothing, which is how a caller puts air between two
                # groups of bars that measure different things.
                continue
            share, cls = entry
            _tube(dots, owner, columns[index], top_row * DOTS_Y, tall,
                  share, cls)


def _tube(dots, owner, col, top, tall, share, cls):
    """One thermometer in cell column `col`: a tube `tall` dots high from
    dot row `top`, the mercury `share` of it from the bottom in `cls`,
    the rest track. The same instrument wherever a page draws a
    thermometer - `gauges` calls it for every other view.

    THE CELL THE MERCURY ENDS IN HOLDS MERCURY AND NOTHING ELSE. A cell
    is one colour, and the mercury's class wins it - so a track dot
    drawn above the level inside that cell took the mercury's colour
    and the level read a row higher than it was. Worse, it read a WHOLE
    row: the top of every bar came out `⣿` whatever the level, and a
    tube that fills in cell steps barely moves. Kept clear, the top of
    the mercury is drawn at the dot - `⣀`, `⣤`, `⣶`, `⣿` - and a level
    that moves one dot is seen to move.

    THE TUBE ABOVE IT, on the cell's own second and fourth rows so every
    track cell is the same `⣒`. A bar with nothing over it says how hot
    a node is; a bar in a tube says how hot it is OF WHAT IT MAY BE,
    which is the only version of the question a ceiling makes sense of.
    THE TUBE'S OWN WIDTH, both lanes: one lane made the empty half of a
    thermometer narrower than the mercury under it, so a tall tube read
    as a scale and a short one as a stray dot beside a bar - which is
    why the bench saw the dimmed pixels on some thermometers and not
    others. Every other dot ROW still, so it stays a scale and not more
    level.
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
            if cls > owner[row][col]:
                owner[row][col] = cls
        elif y % DOTS_Y in (1, 3) and row != edge:
            for lane in range(DOTS_X):
                dots[row][col] |= BRAILLE_BITS[lane][y % DOTS_Y]
            if TRACK > owner[row][col]:
                owner[row][col] = TRACK


def _overlay(dots, text, width, height, labels, leaders, rules):
    """Leaders in dots and names in text, over cells no drawing
    reached.

    OUT OF `_raster` BECAUSE IT IS A DIFFERENT JOB. That one turns
    a machine into dots; this writes a legend on the air beside it,
    and the two together ran past what a reader can hold.

    Answers the cells the leaders lit, `(row, col, ink)`, so the
    caller can colour them without giving them an owner class - a
    leader belongs to its label, not to the machine.
    """
    # THE LEADERS, IN DOTS, AND THEY FALL. `(from_row, col, to_row, ink)`
    # is a dotted column dropping from under a name in the caption rows
    # to the top of the bar it points at - the shape a bench drew on the
    # back of the page: a label, an arrowhead, and a line falling to the
    # thing it names.
    #
    # It was a horizontal run with a corner. That works and it reads as a
    # bracket rather than a pointer, and two of them at different lengths
    # read as two brackets rather than a staircase.
    # WHERE A RULE RUNS INSIDE ITS CELL. The gauges draw their level on
    # the middle two dot rows, so a rule on the top row floated a dot
    # clear of the bar it was pointing at and read as a separate line
    # over it. On the bar's own upper row it arrives ON the level.
    # WHERE A RULE RUNS, so a column meeting one can turn instead of
    # crossing it.
    lit = []
    met = {}
    for row, from_col, to_col, _shade in list(rules or ()):
        lo, hi = min(from_col, to_col), max(from_col, to_col)
        met.setdefault(row, []).append((lo, hi))
    for entry in list(leaders or ()):
        from_row, col, to_row, shade = entry[:4]
        # WHICH HALF OF THE CELL IT FALLS DOWN. Left by default, which is
        # where a line falling from a caption belongs; a fifth element
        # puts it in the right half instead, so a run arriving from the
        # left turns UP at the cell's right edge and the corner mirrors.
        # Drawn in the left lane both sides, the right-hand corner came
        # out as the left one and the L read as pointing back the way it
        # came.
        lane = entry[4] if len(entry) > 4 else 0
        for row in range(from_row, to_row):
            if 0 <= row < height and 0 <= col < width:
                # A HOOK WHERE IT MEETS A RULE, not a bar through it. The
                # column ran the full cell height in the corner, so the
                # junction came out as `\u28ba` - four dots of solid stroke
                # standing off a two-dot rule, which reads as a post the
                # line happens to end at. Turning at the rule and going
                # two dots down makes `\u2832`: a line that arrives, turns,
                # and carries on at the weight it came in at.
                turn = any(lo <= col <= hi for lo, hi in met.get(row, ()))
                # A CORNER WHERE IT MEETS A RULE, a stroke where it
                # does not - and how far down the corner reaches
                # depends on whether the line carries on.
                # `braille.corner` has both, and why getting it
                # wrong is visible.
                if turn:
                    dots[row][col] |= braille.mask(braille.lit(
                        braille.corner(RULE_Y, lane,
                                       through=row + 1 < to_row)))
                else:
                    dots[row][col] |= braille.mask(
                        (lane, y) for y in range(DOTS_Y))
                lit.append((row, col, shade))

    # AND THE HORIZONTAL HALF OF THE SAME FURNITURE. `(row, from_col,
    # to_col, ink)` runs along the TOP of its cells, so a run meeting the
    # top of a falling column makes the corner of an upside-down L: a
    # line that leaves an arrowhead, climbs, and turns in toward the
    # thing it names. A leader alone can only point at something above
    # it, and the two levels along the foot lie inboard of the arrows
    # that name them, not over them.
    for row, from_col, to_col, shade in list(rules or ()):
        for col in range(min(from_col, to_col), max(from_col, to_col) + 1):
            if 0 <= row < height and 0 <= col < width:
                for x in range(DOTS_X):
                    dots[row][col] |= BRAILLE_BITS[x][RULE_Y]
                lit.append((row, col, shade))

    # THE OVERLAY LAST, and only where no dot went. A braille cell cannot
    # carry a letter, so a name inside the drawing has to replace a cell
    # outright - which is fine over air and never over the machine. Each
    # entry is `(row, col, text, ink)` and the CALLER owns the placement:
    # this module draws a rotor, not a legend.
    for row, col, said, _ink in list(labels or ()):
        for step, ch in enumerate(said):
            here = col + step
            if 0 <= row < height and 0 <= here < width and not dots[row][here]:
                text[row][here] = ch
    return lit


class Frame:

    """The grid a drawing is built in: dots, who owns each cell, and the
    text laid over them.

    ONE OBJECT BECAUSE THREE PASSES WRITE THE SAME THREE ARRAYS. They
    were three locals in `_raster` with a closure over them, so every
    pass took six arguments or none depending on when it was written and
    the function grew past what a reader can hold. A pass now takes the
    frame and says what it draws.
    """

    def __init__(self, width, height):
        self.width, self.height = width, height
        self.dots = [[0] * width for _ in range(height)]
        self.owner = [[-1] * width for _ in range(height)]
        self.text = [[None] * width for _ in range(height)]
        #: How many dots each class has lit in each cell, so a cell can
        #: belong to what is mostly in it.
        self.tally = [[None] * width for _ in range(height)]

    def put(self, x, y, cls):
        """Light one dot, in DOT coordinates. `cls` None lights it and
        claims nothing.

        A LINE BEATS A FILL IN A SHARED CELL, then the most dots win. A
        cell is eight dots and one colour, and two rules were tried
        before this one, each wrong in one place:

        * the highest RANK that had lit any dot - so at the yoke a tooth
          outranked the ring and the yoke came out chopped into
          phase-coloured segments that changed with the drive;
        * the MOST DOTS - which mended the yoke and broke the can: the
          magnet band's outer edge and the can's inner ring are 0.10 of
          the radius apart, 3.3 dots against a cell four tall, and at
          twelve o'clock the cell they share is mostly magnet. The ring
          went amber in three places, ringed in red on the bench.

        A ring is a LINE the drawing means and a band is an area; a line
        that loses its cell is a broken line, an area that loses one is
        a dot short at its edge, which nobody sees. So a line class
        present in the cell takes it - the one with most dots if several
        - and only cells with no line in them go to the fill with most.
        """
        col, row = int(x) // DOTS_X, int(y) // DOTS_Y
        if 0 <= row < self.height and 0 <= col < self.width:
            self.dots[row][col] |= BRAILLE_BITS[int(x) % DOTS_X][
                int(y) % DOTS_Y]
            if cls is not None:
                tally = self.tally[row][col]
                if tally is None:
                    tally = self.tally[row][col] = {}
                tally[cls] = tally.get(cls, 0) + 1
                # THE TRUTH STROKE FIRST, the one thing drawn to be
                # FOUND: yielding to the rings it owned no cell at all in
                # some poses, so at its own angle a ring cell goes white
                # and it reads as reaching the rim. It never meets a
                # tooth - `_truth` keeps it a dot and a half inside the
                # band. THEN A RING over anything: a broken ring is seen.
                # THEN THE MOST DOTS, with no favour between a magnet and
                # a tooth. The air gap is less than a cell tall, so at
                # twelve and six o'clock a cell holds both; given to the
                # magnet it put amber on the teeth, given to the tooth it
                # put green on the band, and the bench saw each in turn.
                # Whichever has more of the cell is the colour least
                # wrong, and rank only breaks a tie. Teeth over rings was
                # tried too and put the yoke back in pieces.
                #
                # EXCEPT THAT THE STROKE YIELDS TO A TOOTH. The band is a
                # dot and a half from its inner end and a cell's diagonal
                # still bridges that at some angles - two cells in 48
                # poses held a tooth's tip and the stroke both. A white
                # cell on a tooth is a mark on the stator, which is where
                # this mark has been chased out of three times; in that
                # cell the stroke is simply not a candidate.
                running = ([c for c in tally if c != TRUTH]
                           if set(tally) & TEETH else tally)
                self.owner[row][col] = max(
                    running, key=lambda c: (c in MARKS, c in LINES,
                                            tally[c], c))

    def claim(self, row, col, cls, said=None):
        """Give a CELL to `cls`, and a character with it where the mark
        cannot be made of dots."""
        if 0 <= row < self.height and 0 <= col < self.width:
            self.owner[row][col] = cls
            if said is not None:
                self.text[row][col] = said

    def lines(self, ink, colour=False, tint=None):
        """THE ONE PLACE THIS BECOMES TERMINAL OUTPUT.

        Everything above writes into the buffer and nothing above knows
        what an escape sequence is; here the cells become characters and,
        if asked, colour. `ink` maps an owner class to its colour and
        `tint` overrides particular cells - a legend's words and the
        leader that belongs to them keep their own ink without owning the
        cells they cross.
        """
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

    """Where the machine sits in its box, and what is left around it.

    OUT OF `_raster` BECAUSE IT IS THE ANSWER EVERY PASS NEEDS AND NONE
    OF THEM SHOULD WORK OUT. The centre, the radii, how many rows the
    gauges and the legend took, and the dot aspect - six numbers that
    were computed inline and then passed around one at a time.
    """

    def __init__(self, width, height, left, right, top, bottom,
                 labels, leaders, aspect):
        self.floors = max(1, len(list(bottom or ())))
        # A LABEL'S ROW IS WRITTEN ON, A LEADER'S `to_row` IS PAST ITS
        # LAST: the two are one column of arithmetic with different ends,
        # and the can has to start under both. A LEADER THAT STARTS AT
        # THE TOP pushes the can down; one starting lower is drawn in
        # rows the floor gauges already own, and counting it here would
        # reserve the whole box.
        written = [row + 1 for row, _col, _said, _ink in list(labels or ())]
        written += [entry[2] for entry in list(leaders or ())
                    if entry[0] == 0]
        self.reserve = max(written) if written else 0
        self.band = max(1, height - self.floors - self.reserve)
        # A DOT IS SQUARE ONLY WHEN A CELL IS TWO BY ONE. Everything here
        # measures in x-dots and scales y by this, so a drawing stays
        # round on a terminal whose font says otherwise - and the band is
        # measured in the same units, or a band fitted to the can would
        # shrink it.
        self.stretch = aspect / DOTS_Y * DOTS_X
        self.cx, self.radii, _, _ = layout(
            width, height, len(left or ()), len(right or ()), rows=self.band,
            stretch=self.stretch)
        # AND SEATED AT THE TOP OF THAT BAND, its first dot in the first
        # row under the reserve. It was centred, so whatever the band had
        # over the can's height was split above and below it - and on a
        # terminal whose cell the view could not measure, drawn at an
        # assumed 2.0, that was a row of air between the legend and the
        # motor that nothing explained. Seated, there is exactly the
        # leaders' hop between them whatever the terminal says, and the
        # spare - if any - lies over the foot gauges, which name levels
        # and not the machine.
        self.cy = (self.reserve * DOTS_Y + 0.5
                   + self.radii.can / self.stretch)


def _body(frame, seat, rotor_deg, slots, poles, drive):
    """The machine itself, dot by dot.

    THE CORNERS ARE COVERAGE, not a vote on whether anything is there.
    One corner of four lit the dot whole, so every arc came out a dot
    fatter than it is and the can's rim stepped against the magnets
    inside it. Half a dot or more still lights outright - a one-dot rim
    is a line the drawing means - and the fringe beyond that is
    dithered, which is what puts the patterns between solid and blank on
    the page.
    """
    rotor = math.radians(rotor_deg)
    for y in range(frame.height * DOTS_Y):
        for x in range(frame.width * DOTS_X):
            # EACH SAMPLE VOTES WITH ITS COVERAGE, and the dot goes to the
            # class that covers most of it. It went to the highest RANK
            # among the samples, which is a rule about which shape is
            # more important and not about what is there: at the yoke a
            # tooth outranks the ring, so every cell the ring passed
            # through where a tooth roots took the tooth's colour and
            # the yoke came out chopped into phase-coloured segments that
            # changed with the drive. That is the colour fault a bench
            # sees in the stator. Rank only breaks a tie now.
            votes = {}
            for ox, oy in SUBDOT:
                dx = x + ox - seat.cx
                dy = (seat.cy - y - oy) * seat.stretch
                at, share = _classify(math.hypot(dx, dy), math.atan2(dy, dx),
                                      rotor, slots, poles, seat.radii, drive)
                if at is not None:
                    votes[at] = votes.get(at, 0.0) + share
            if votes and covered(sum(votes.values()), len(SUBDOT)):
                frame.put(x, y, max(votes, key=lambda c: (votes[c], c)))


def _bead(frame, seat, pointer_deg, glyph=None, rate=None):
    """The bench's own zero, riding the can's rim.

    `POINTER_GLYPH` IS THE BEAD AND THAT IS SETTLED. It is the bench's
    choice, made twice, and this docstring exists so it is not made a
    third time. What is left to get right is WHERE it goes.

    WHAT A GLYPH BUYS: it is the same mark at every angle by
    construction - no shape for a dot grid to approximate, nothing to
    straddle, no weight that changes on the way round. A braille cell
    cannot carry a letter, so it replaces the cell it lands in: the rim
    opens for the bead and closes behind it.

    WHAT IT COSTS, honestly: a character cell is about one wide by two
    tall, so the glyph's own proportions are the FONT'S and not this
    drawing's - against a picture made of square dots it reads a little
    narrow. Nothing here can change that; only dots are square.

    THE FOUR DOT ANSWERS, all built, none kept: a radial spur, which at
    the top of the can lay across four rows of one cell and read as a
    tick and at its sides across two columns and read as a dash; a
    square of dots centred ON the rim and clipped to the rotor's
    silhouette, which left a crescent cut differently at every angle; a
    sampled disc seated inside the rim, round and spread over three
    cells, which is a smear on the band; and a 2x2 block, or that disc
    at a dot's radius thresholded at half coverage so the weight graded
    with the sub-position - `⣀`, `⣤`, `⣶` climbing a cell. The best of
    them, and still a mark whose size changes as it travels.
    """
    phi = math.radians(pointer_deg)
    # ON THE RIM, half in and half out, so it breaks the silhouette
    # where it is and closes behind it.
    #
    # `stretch` IS NOT OPTIONAL, and leaving it out was a real bug. The
    # radii are in x-dots and `_body` scales y by it, so a bead placed
    # with plain trigonometry rode the rim only where a dot happened to
    # be square. On a terminal whose cell is not two-by-one it sat
    # outside the periphery, which is where the bench found it.
    seat_r = seat.radii.can + POINTER_SEAT
    at_x = seat.cx + seat_r * math.cos(phi)
    at_y = seat.cy - seat_r * math.sin(phi) / seat.stretch
    # THE NEAREST CELL, MEASURED FROM ITS CENTRE. Truncating puts the
    # mark in whichever cell the exact point falls inside, and a cell is
    # two dots across by four down - so the path comes out quantised
    # twice as coarsely down as across, an egg rather than a circle.
    # Measured over 360 degrees, the worst departure from the true
    # circle falls from 2.76 dots to 2.15.
    col = int(math.floor((at_x - (DOTS_X - 1) / 2.0) / DOTS_X + 0.5))
    row = int(math.floor((at_y - (DOTS_Y - 1) / 2.0) / DOTS_Y + 0.5))
    frame.claim(row, col, POINTER, glyph or POINTER_GLYPH)
    if rate:
        _wake(frame, seat, phi, seat_r, rate, (row, col))


#: The wake's shutter, seconds of travel the trail shows. At 60 rpm a
#: tenth of a second is 36 degrees of rim - a tenth of the way round,
#: seen from across a bench; at a crawl it is a few dots; and the cap
#: keeps a fast can from wearing a ring, which would say nothing about
#: which way it turns.
TRAIL_S = 0.1
TRAIL_MAX_DEG = 120.0


def _wake(frame, seat, phi, seat_r, rate, bead_cell):
    """The trail behind the bead: an arc on the rim, TRAIL_S of travel
    long at `rate` degrees a second, on the side the bead came from,
    fading in thirds through `TRAIL`. Its length is the speed and its
    side is the direction, which is what a smear behind a moving thing
    says. It stays out of the bead's own cell, or it would take the
    bead's colour with it."""
    length = min(TRAIL_MAX_DEG, abs(rate) * TRAIL_S)
    if length <= 0.0:
        return
    back = -1.0 if rate > 0.0 else 1.0
    steps = max(2, int(seat_r * math.radians(length) / 0.5) + 1)
    for i in range(1, steps + 1):
        t = i / float(steps)
        a = phi + back * math.radians(length) * t
        x = seat.cx + seat_r * math.cos(a)
        y = seat.cy - seat_r * math.sin(a) / seat.stretch
        if (int(y) // DOTS_Y, int(x) // DOTS_X) == bead_cell:
            continue
        frame.put(x, y, TRAIL[min(len(TRAIL) - 1, int(t * len(TRAIL)))])


def _truth(frame, seat, truth_deg):
    """The angle a shaft sensor says, as a tick in the AIR GAP.

    It was a notch cut outward through the can: the can is one dot thick
    and the mark four, so it took whole cells of the outer ring with it
    and read as a white gash in the stator - a drawing artefact rather
    than a rotor angle. In the gap it touches nothing, and the magnet
    band it should line up with is immediately outside it.
    """
    phi = math.radians(truth_deg)
    r = seat.radii
    # THROUGH THE MAGNET BAND, which is the thing it is read against. It
    # was a tick in the air gap between the tooth tips and the band - and
    # the air gap is the stator's side of the picture: whatever it did
    # not touch it stood over, a white mark at the slot mouths where the
    # teeth show their current, and it read as a second indicator drawn
    # across the magnetisation. Twice trimmed, twice still there. Then
    # outside the rim, where the bench's own mark rides: one column of
    # air at three and nine o'clock, so it reached into the gutter, and
    # the rim's own fringe left it no empty cell at some angles.
    #
    # The band has room, is the rotor, and is what the sensor's angle is
    # compared with: the estimate turns the band, the sensor draws the
    # stroke, and a slipped pole is the stroke standing off a magnet's
    # edge. A dot clear of the band's own edges, so it never shares a
    # cell with the can's rings - and the air gap keeps every tooth a
    # cell's diagonal away.
    # A DOT AND A HALF INSIDE THE BAND'S INNER EDGE, not half: at half it
    # shared five cells in 48 poses with a tooth's tip across the air
    # gap, and a stroke that wins those is a white mark on the stator.
    # The ordering in `Frame.put` cannot settle that - rings must beat
    # teeth, teeth must beat magnets, the stroke must beat rings to be
    # seen - so the stroke simply never meets a tooth.
    inner, outer = r.magnet_in + 1.5, r.magnet_out - 0.5
    for step in range(int((outer - inner) * 4) + 1):
        radius = inner + step * 0.25
        frame.put(seat.cx + radius * math.cos(phi),
                  seat.cy - radius * math.sin(phi) / seat.stretch, TRUTH)


def _machine(frame, seat, rotor_deg, slots, poles, drive,
             truth_deg=None, pointer_deg=None, bead=None, pointer_rate=None):
    """THE MACHINE AND NOTHING ELSE: the cross-section, the bench's mark
    on the rim, and the tick a shaft sensor claims.

    One pass, so a caller that wants the motor without the instruments
    hanging off it - `motor()` - asks for exactly this.
    """
    _body(frame, seat, rotor_deg, slots, poles, drive)
    if truth_deg is not None:
        _truth(frame, seat, truth_deg)
    if pointer_deg is not None:
        _bead(frame, seat, pointer_deg, bead, pointer_rate)


def _instruments(frame, seat, left, right, top, bottom):
    """The gutters and the gauges: everything measured AGAINST the
    machine rather than part of it.

    The gauges run in a ROW across the machine's width. It was one; a
    page that has to say how much is left of the board and of the
    windings at once cannot say it in one bar, and stacking them would
    need two rows and leave a reader matching bars to names by order.
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
    """The machine alone, as text rows - no gutters, no gauges, no
    legend.

    What `render` draws before it hangs instruments off it, and the only
    thing a caller who wants a picture of a motor actually wants.
    """
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
    both. Three passes and the seat they share - each has its own
    function, and this one only says the order."""
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
    """The cross-section, `rotor_deg` being how far the can has turned.

    `rotor_deg` is mechanical: the electrical angle over the pole pairs.
    Which pole pair it lands in is not recoverable from an electrical
    angle and does not need to be - the picture repeats every pole pair,
    so the one drawn is right whichever it is.

    `amps` is the three phase currents and `full` what to call full
    scale - the trip or the clamp, whatever the record says the stage may
    do. The teeth are drawn to that scale, so the picture is of THIS
    machine being driven THIS hard and not a diagram of a motor.

    `pointer_deg` puts a spur outside the can, which is where a mark on
    a real rotor goes: a bench's own zero, wherever it decided that is.

    `truth_deg` cuts a notch through the can at the rotor's real angle,
    when something knows it. The gap between the notch and the magnet
    band under it IS the observer's error, in the units a magnet works in.

    `left` and `right` are margin bars - `(fraction, class)` each - drawn
    in the gutters either side. `top` is a SEQUENCE of such pairs sharing
    the first row end to end, and `bottom` a sequence on the last rows,
    one each. What they measure is the
    caller's; this draws levels.

    `labels` are `(row, col, text, ink)` written over cells no dot
    reached - a name inside the drawing, which a braille cell cannot
    carry any other way. Where they go is the caller's: this draws a
    rotor, not a legend.

    `aspect` is how tall the terminal's cell is against its width. The
    geometry is exactly round at 2.0 - measured, 25.16 cell-widths each
    way - so a can that reads as an ellipse is the FONT saying it is not
    2.0, and this is where a bench tells the drawing what its font does.
    Ten per cent tall is a 2.2 cell, which several fonts are.

    Colour is asked for here rather than applied afterwards: a braille
    cell carries dots from up to eight places and its glyph does not say
    which, so there is nothing for a `colourise(text)` to key on.
    """
    poles = max(2, int(poles) - int(poles) % 2)
    slots = max(3, int(slots))
    drive = _drive(amps, full)
    frame, lit = _raster(rotor_deg, slots, poles, width, height,
                         truth_deg, drive, pointer_deg, left, right,
                         top, bottom, aspect, labels, leaders, rules,
                         bead, pointer_rate)
    # THE ONLY THING LEFT HERE IS WHO GETS WHICH COLOUR. The buffer holds
    # the picture; a legend's words and the leader that belongs to them
    # keep their own ink without owning the cells they cross, which is
    # why this is a map and not another owner class.
    at = {}
    for row, col, said, said_ink in list(labels or ()):
        for step in range(len(said)):
            at[(row, col + step)] = said_ink
    for row, col, shade in lit:
        at.setdefault((row, col), shade)
    return '\n'.join(frame.lines(phase_ink(drive), colour=colour, tint=at))


def caption(slots, poles, rotor_deg, slipped=None):
    """One line naming the machine drawn and where the can is.

    `slipped` is how many whole magnets the can has stepped since the
    caller last asked - a slipped pole reads here as a step nobody
    commanded, which is what the picture exists to make visible.
    """
    text = '%dN%dP  can %5.1f deg' % (slots, poles, rotor_deg % 360.0)
    if slipped:
        text += '  %+d magnets' % slipped
    return text
