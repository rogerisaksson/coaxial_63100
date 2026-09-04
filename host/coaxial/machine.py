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
from .raster import BRAILLE, BRAILLE_BITS, DOTS_X, DOTS_Y, SUBDOT

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


#: THE BEAD IS THREE DOTS BY THREE. A square block, not a disc: at this
#: size a disc is a plus sign or a blob depending on where its centre
#: falls between the dots, and a mark whose shape changes as it travels
#: reads as flicker. Nine dots is the smallest thing that is
#: unambiguously a mark and not a stray dot of the rim.
#:
#: It is centred ON the can's outer edge, not outside it: held clear it
#: read as something floating beside the drawing rather than a mark on
#: the rotor, and a position indicator that is not attached to the thing
#: whose position it indicates indicates nothing.
POINTER_SIDE = 3
#: How far its centre sits proud of the can's outer edge, and how far
#: past that edge it is allowed to reach. Seated ON the rim and CLIPPED
#: to it: standing proud it read as a blob orbiting the drawing rather
#: than a mark on the rotor, and a position indicator outside the thing
#: whose position it indicates is a smudge.
POINTER_SEAT = 0.6
#: How far past the can's outer edge the bead may reach. ONE DOT. Fully
#: inside, the bead only added dots to a rim that was already solid and
#: nothing but colour said it was there; four dots proud, it was a blob
#: orbiting the drawing. One dot is a nub on the silhouette.
POINTER_PROUD = 1.0
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
PHASE_CLASS = (TOOTH_U, TOOTH_V, TOOTH_W)
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
       SOUTH: 94, NORTH: ansi.AMBER, TRUTH: 252, POINTER: 231,
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
#: One column, not two. Two columns of air each side cost the machine
#: eight per cent of its diameter at this width, and the gutters read as
#: separate from it at one.
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

    def __init__(self, width, height):
        self.can = (min(width * DOTS_X, height * DOTS_Y) / 2.0 - 1.0) * F_FIT
        self.magnet_out = self.can * F_MAGNET_OUT
        self.magnet_in = self.can * F_MAGNET_IN
        self.tooth_out = self.can * F_TOOTH_OUT
        self.tooth_in = self.can * F_TOOTH_IN
        self.bore = self.can * F_BORE
        self.can_inner = self.can * F_CAN_INNER
        self.line = max(0.8, self.can * F_LINE)

    def ring(self, radius, at, weight=1.0):
        return abs(radius - at) <= self.line * weight


def _magnet_class(radius, phi, rotor, poles, r):
    """North solid, south a thin arc, or None between them.

    POLARITY IS THICKNESS. A north magnet is the full depth of the band
    and a south is a line down the middle of it, which counts out as 28
    magnets and reads as 14 pole pairs at once.
    """
    place = ((phi - rotor) % math.tau) / (math.tau / poles)
    index, into = int(place), place - int(place)
    if into < 0.1 or into > 0.9:            # the break between magnets
        return None
    if index % 2 == 0:
        return NORTH
    return SOUTH if r.ring(radius, (r.magnet_in + r.magnet_out) / 2.0) else None


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
        return None
    place = (phi % math.tau) / (math.tau / slots)
    if place - int(place) > TOOTH_FILL:
        return None
    phase = int(place) % 3
    if drive is not None:
        share = drive[phase]
        span = (r.tooth_out - r.tooth_in) * (TOOTH_STUB + (1.0 - TOOTH_STUB)
                                             * abs(share))
        if share >= 0.0:
            if radius > r.tooth_in + span:
                return None
        elif radius < r.tooth_out - span:
            return None
    return PHASE_CLASS[phase]


def _classify(radius, phi, rotor, slots, poles, r, drive):
    """Which element, if any, owns the dot at (radius, phi)."""
    if r.ring(radius, r.bore):
        return BORE
    if r.ring(radius, r.tooth_in):
        return YOKE
    if radius <= r.tooth_out:
        return _tooth_class(radius, phi, slots, r, drive)
    if r.magnet_in <= radius <= r.magnet_out:
        return _magnet_class(radius, phi, rotor, poles, r)
    if r.ring(radius, r.can, CAN_WEIGHT) or r.ring(radius, r.can_inner):
        return CAN
    return None


def layout(width, height, n_left=0, n_right=0):
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
    r = _Radii(room, height)
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


def span(width, height, n_left=0, n_right=0):
    """The columns the machine itself occupies, first and last.

    FLOORED AT BOTH ENDS. Ceiling the right one put its edge a column
    further out than the circle actually reached, so the gutter measured
    off it stood two columns clear where the left stood one.
    """
    cx, r, _, _ = layout(width, height, n_left, n_right)
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
    wide = hi - lo
    filled = int(max(0.0, min(1.0, share)) * wide + 0.5)
    for step in range(wide):
        x = lo + step
        col = x // DOTS_X
        if step < filled:
            for y in (row * DOTS_Y + 1, row * DOTS_Y + 2):
                dots[row][col] |= BRAILLE_BITS[x % DOTS_X][y % DOTS_Y]
            if cls > owner[row][col]:
                owner[row][col] = cls
        elif step % 4 == 0:
            dots[row][col] |= BRAILLE_BITS[x % DOTS_X][2]
            if TRACK > owner[row][col]:
                owner[row][col] = TRACK


def _bars(dots, owner, width, height, left, right, r, floors=1, reserve=0):
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
    top_row = GAUGE_INSET + 1 + reserve
    tall = max(1, height - GAUGE_INSET - FLOOR_INSET - 1 - reserve
               - max(1, floors)) * DOTS_Y
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
            col = columns[index]
            filled = int(max(0.0, min(1.0, share)) * tall + 0.5)
            for step in range(tall):
                y = top_row * DOTS_Y + tall - 1 - step
                row = y // DOTS_Y
                if step < filled:
                    # The column of mercury: both dots, solid, in the
                    # band's own colour.
                    for x in (col * DOTS_X, col * DOTS_X + 1):
                        dots[row][x // DOTS_X] |= \
                            BRAILLE_BITS[x % DOTS_X][y % DOTS_Y]
                    if cls > owner[row][col]:
                        owner[row][col] = cls
                elif step % 2 == 0:
                    # THE TUBE ABOVE IT. A bar with nothing over it says
                    # how hot a node is; a bar in a tube says how hot it
                    # is OF WHAT IT MAY BE, which is the only version of
                    # the question a ceiling makes sense of. Every other
                    # dot, one side, so the empty part reads as a scale
                    # and not as more level.
                    dots[row][col] |= BRAILLE_BITS[0][y % DOTS_Y]
                    if TRACK > owner[row][col]:
                        owner[row][col] = TRACK


def _overlay(dots, text, width, height, labels, leaders):
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
    lit = []
    for from_row, col, to_row, shade in list(leaders or ()):
        for row in range(from_row, to_row):
            if 0 <= row < height and 0 <= col < width:
                for y in range(DOTS_Y):
                    dots[row][col] |= BRAILLE_BITS[0][y]
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


def _raster(rotor_deg, slots, poles, width, height, truth_deg, drive,
            pointer_deg, left, right, top, bottom, aspect, labels=None,
            leaders=None):
    """Dots, their owners and a text overlay, one entry per cell."""
    dots = [[0] * width for _ in range(height)]
    owner = [[-1] * width for _ in range(height)]
    text = [[None] * width for _ in range(height)]
    cx, r, _, _ = layout(width, height, len(left or ()), len(right or ()))
    cy = height * DOTS_Y / 2.0 - 0.5
    rotor = math.radians(rotor_deg)

    def put(x, y, cls):
        """Light one dot. `cls` None lights it and claims nothing."""
        col, row = int(x) // DOTS_X, int(y) // DOTS_Y
        if 0 <= row < height and 0 <= col < width:
            dots[row][col] |= BRAILLE_BITS[int(x) % DOTS_X][int(y) % DOTS_Y]
            if cls is not None and cls > owner[row][col]:
                owner[row][col] = cls

    stretch = aspect / DOTS_Y * DOTS_X
    for y in range(height * DOTS_Y):
        for x in range(width * DOTS_X):
            cls = None
            for ox, oy in SUBDOT:
                dx, dy = x + ox - cx, (cy - y - oy) * stretch
                at = _classify(math.hypot(dx, dy), math.atan2(dy, dx),
                               rotor, slots, poles, r, drive)
                if at is not None and (cls is None or at > cls):
                    cls = at
            if cls is not None:
                put(x, y, cls)
    if pointer_deg is not None:
        # OUTSIDE THE CAN, where a mark on a real rotor would be. It is
        # the only thing in the picture that is allowed to be arbitrary:
        # `tare` puts it wherever a bench decides zero is, and the whole
        # point of a zero is that the machine has no opinion about where
        # it should be. It follows the rotor, so what it reads off is
        # travel from that zero rather than an electrical angle.
        #
        # A BEAD ON THE RIM. It rides the can's own outer edge, so it
        # travels with the drawing instead of orbiting it, and because
        # POINTER outranks CAN the cells it passes through take its
        # colour - the rim opens for it and closes behind it.
        #
        # A ROUND DOT, not a radial spur. The spur was a line of dots
        # along the radius: at the top and bottom of the can it lay
        # across four rows of one cell and read as a tick mark, at the
        # sides it lay along two columns and read as a dash, and a mark
        # whose shape depends on where it is is a mark you cannot follow
        # round. A disc is the same shape at every angle, which is the
        # whole job.
        phi = math.radians(pointer_deg)
        seat = r.can + POINTER_SEAT
        at_x = cx + seat * math.cos(phi)
        at_y = cy - seat * math.sin(phi)
        reach = POINTER_SIDE // 2
        share = {}
        for dy in range(-reach, reach + 1):
            for dx in range(-reach, reach + 1):
                x, y = round(at_x) + dx, round(at_y) + dy
                # CLIPPED TO THE ROTOR. Nothing of the bead is drawn past
                # the can's outer edge: the rotor's silhouette is the
                # drawing's own boundary and a mark that crosses it stops
                # being part of the machine.
                if math.hypot(x - cx, cy - y) > r.can + r.line + POINTER_PROUD:
                    continue
                put(x, y, None)
                share[(int(y) // DOTS_Y, int(x) // DOTS_X)] = True
        home = (int(at_y) // DOTS_Y, int(at_x) // DOTS_X)
        for row, col in share:
            middle = math.hypot(col * DOTS_X + (DOTS_X - 1) / 2.0 - at_x,
                                row * DOTS_Y + (DOTS_Y - 1) / 2.0 - at_y)
            if (middle <= POINTER_SIDE / 2.0 or (row, col) == home) \
                    and 0 <= row < height and 0 <= col < width:
                owner[row][col] = POINTER
    if truth_deg is not None:
        # A TICK IN THE AIR GAP, inside the magnet band it is read
        # against. It was a notch cut outward through the can first: the
        # can is one dot thick and the mark four, so it took whole cells
        # of the outer ring with it and read as a white gash in the
        # stator - a drawing artefact rather than a rotor angle. In the
        # gap it touches nothing, and the magnet band it should line up
        # with is immediately outside it.
        phi = math.radians(truth_deg)
        span = r.magnet_in - r.tooth_out
        for step in range(int(span * 4)):
            radius = r.tooth_out + 0.5 + step * 0.25
            put(cx + radius * math.cos(phi), cy - radius * math.sin(phi),
                TRUTH)
    floor = list(bottom or ())
    written = [row for row, _col, _said, _ink in list(labels or ())]
    _bars(dots, owner, width, height, left, right, r, len(floor),
          reserve=(max(written) + 1) if written else 0)
    # A ROW OF THEM, side by side across the machine's width. It was one
    # gauge; a page that has to say how much is left of the BOARD and of
    # the WINDINGS at once cannot say it in one bar, and stacking them
    # would need two rows and leave a reader matching bars to names by
    # their order.
    for index, gauge in enumerate(list(top or ())):
        _gauge(dots, owner, width, height, GAUGE_INSET, gauge[0], gauge[1],
               len(left or ()), len(right or ()),
               part=(index, len(top)))
    for index, gauge in enumerate(floor):
        _gauge(dots, owner, width, height,
               height - FLOOR_INSET - len(floor) + index, gauge[0], gauge[1],
               len(left or ()), len(right or ()))

    lit = _overlay(dots, text, width, height, labels, leaders)
    return dots, owner, text, lit


def render(rotor_deg, slots=24, poles=28, width=40, height=22,
           truth_deg=None, amps=None, full=None, pointer_deg=None,
           left=None, right=None, top=None, bottom=None,
           aspect=CELL_ASPECT, colour=False, labels=None, leaders=None):
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
    dots, owner, text, lit = _raster(rotor_deg, slots, poles, width, height,
                                     truth_deg, drive, pointer_deg, left,
                                     right, top, bottom, aspect, labels,
                                     leaders)
    ink = phase_ink(drive)
    at = {}
    for row, col, said, said_ink in list(labels or ()):
        for step in range(len(said)):
            at[(row, col + step)] = said_ink
    # A LEADER TAKES ITS LABEL'S COLOUR, so the rule and the words that
    # own it read as one thing. The cells keep their braille glyph - only
    # the ink is overridden - which is why this is a map and not a class.
    for row, col, shade in lit:
        at.setdefault((row, col), shade)
    lines = []
    for row in range(height):
        cells = [(text[row][col] or chr(BRAILLE + dots[row][col]),
                  at[(row, col)] if (row, col) in at
                  else ink.get(owner[row][col]))
                 for col in range(width)]
        lines.append(ansi.run(cells) if colour
                     else ''.join(char for char, _ in cells))
    return '\n'.join(lines)


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
