"""Levels in braille: the one instrument every page draws a level with.

MOTOR CONTROLLER drew its thermometers in dots - a tube the height of the
box with the mercury rising inside it at dot resolution, the empty part
a grey track, the colour a margin against a ceiling - and every other
page drew its levels as `====----` in ASCII. The bench asked for the
same instrument on the meter bridge, on the thermal observer and in
every box that shows a level. This is that instrument on its own: a
horizontal gauge one row tall, and a row of vertical tubes, drawn by
THE SAME CODE as the machine's gutters and floor - `machine._level` and
`machine._tube` on a `machine.Frame` - so a level means the same thing
and looks the same wherever it is.

Pure: fractions in, text out. What a fraction is OF, and which class
colours it, is the caller's - this draws levels (invariant 10).
"""
from . import machine
from .machine import Frame, INK, SOA_OK, TRACK
from .raster import DOTS_X, DOTS_Y

#: The tick class a caller puts on a gauge - a burst's extreme, a held
#: peak - here so a view need not reach into `machine` for it.
MARK = machine.MARK


def gauge(share, cells, cls=SOA_OK, centre=None, marks=(), colour=True):
    """One row, `cells` wide: a level `share` of the way along the scale
    in `cls`, the rest of the scale in the track's grey.

    `centre`, a share, makes it BIPOLAR: the level runs from the centre
    to `share` either way, the way a meter bridge's phase channel swings
    about zero, and the centre itself is marked.

    `marks` are `(share, cls)` ticks a level's height, drawn over
    whatever is there - a burst's extreme; `(share, cls, PEAK)` is the
    top dot alone, a held peak, so the two read apart on one row. `MARK`
    is the white for them; a caller with a reason may pass another
    class.
    """
    frame = Frame(cells, 1)
    wide = cells * DOTS_X

    def dot(at):
        return int(round(max(0.0, min(1.0, at)) * (wide - 1)))

    if centre is None:
        start, end = 0, int(max(0.0, min(1.0, share)) * wide + 0.5)
    else:
        here, zero = dot(share), dot(centre)
        start, end = (zero, here + 1) if here >= zero else (here, zero + 1)
    machine._level(frame.dots, frame.owner, 0, 0, wide, start, end, cls)
    if centre is not None and not start <= dot(centre) < end:
        # THE CENTRE IS MARKED when the level does not cover it: a
        # bipolar gauge at rest still says where zero is.
        machine._mark(frame.dots, frame.owner, 0, dot(centre), TRACK)
    for mark in marks:
        at, mark_cls = mark[0], mark[1]
        machine._mark(frame.dots, frame.owner, 0, dot(at), mark_cls,
                      mark[2] if len(mark) > 2 else machine.GAUGE_Y)
    return frame.lines(INK, colour=colour)[0]


#: The dot rows a held peak takes on a gauge: the top one alone, so it
#: reads apart from a burst extreme's full-height tick beside it.
PEAK = (0,)


#: ONE SCALE FOR EVERY THERMOMETER ON EVERY PAGE, degrees C: the motor
#: page's switch tubes, its NTC and its winding, the thermal observer's
#: tubes, the session's levels. A DRAWING SCALE, not a limit - the board
#: judges nothing by it (invariant 10) and the colours carry the real
#: ceilings. From -35 so a winter bench starts on the tube rather than
#: under it, to 130 so a node at the record's highest ceiling (125) is
#: seen short of the top - the bench's numbers, 2026-09-05. Before them
#: the motor page ran 125 from the reported ambient for its gutters and
#: 150 from a literal 20 for the winding, and the thermal observer's
#: map its own -20 to 100: three rulers, one of them moving with the
#: room.
TEMP_FLOOR_C, TEMP_SCALE_C = -35.0, 130.0


def temp_share(celsius):
    """Where a temperature sits on the scale, 0 at the floor and 1 at the
    top, clamped: the height of every thermometer."""
    span = TEMP_SCALE_C - TEMP_FLOOR_C
    return max(0.0, min(1.0, (celsius - TEMP_FLOOR_C) / span))


def margin_class(share, tripped=False):
    """Which band a node's margin is in - `machine.SOA_CLASS`'s order.

    THE BANDS ARE THE BOARD'S. `share` is the fraction of a node's
    ceiling and the ceiling came from the calibration record; amber is
    `THROTTLE_AT`, the same number `set_limit` writes and the board
    backs off at; red is the ceiling. The margin is reported - the
    action is the board's, and it takes it by dropping MOE (invariant
    10).
    """
    from .thermal_device import THROTTLE_AT
    if tripped or share >= 1.0:
        return machine.SOA_TRIP
    return machine.SOA_WARN if share >= THROTTLE_AT else machine.SOA_OK


def thermometer_class(celsius):
    """Which band of the thermometer ramp a reading is in.

    COLD TO HOT, blue at the scale's floor and red at its top, because
    the thermistor has no ceiling to be a margin against. Every other
    level is coloured by how close it is to a limit it was given; this
    one is coloured by what it says.
    """
    ramp = machine.NTC_RAMP
    step = int(temp_share(celsius) * (len(ramp) - 1) + 0.5)
    return ramp[max(0, min(len(ramp) - 1, step))]


def tubes(entries, rows, labels=(), pitch=None, colour=True):
    """A row of thermometers `rows` tall, one per entry, as text lines.

    Each entry is `(share, cls)` or None for a spacer column. `labels`
    are written under the tubes, one each, centred; `pitch` is the
    columns per tube and defaults to what the widest label needs. The
    lines come back coloured when `colour` is set, the label row in the
    track's grey so the tubes read first.
    """
    labels = [str(label) for label in labels]
    if pitch is None:
        pitch = max([2] + [len(label) + 1 for label in labels])
    width = pitch * max(1, len(entries))
    frame = Frame(width, rows)
    for index, entry in enumerate(entries):
        if entry is None:
            continue
        share, cls = entry
        col = index * pitch + (pitch - 1) // 2
        machine._tube(frame.dots, frame.owner, col, 0, rows * DOTS_Y,
                      share, cls)
    lines = frame.lines(INK, colour=colour)
    if labels:
        row = ''.join(label[:pitch].center(pitch)
                      for label in labels + [''] * (len(entries)
                                                    - len(labels)))
        if colour:
            from . import ansi
            row = ansi.paint(row, INK[TRACK] + 6)
        lines.append(row.rstrip() if not colour else row)
    return lines
