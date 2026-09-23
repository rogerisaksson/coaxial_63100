"""Levels in braille: the one instrument every page draws a level with."""
from . import machine
from .machine import Frame, INK, SOA_OK, TRACK
from .graphics.raster import DOTS_X, DOTS_Y
from .thermal_device import THROTTLE_AT
from . import ansi

#: The tick class a caller puts on a gauge - a burst's extreme, a held
#: peak - here so a view need not reach into `machine` for it.
MARK = machine.MARK


def gauge(share, cells, cls=SOA_OK, centre=None, marks=(), colour=True):
    """One row, `cells` wide: a level `share` of the way along the scale in
    `cls`, the rest of the scale in the track's grey.
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
        # THE CENTRE IS MARKED when the level does not cover it: a bipolar
        # gauge at rest still says where zero is.
        machine._mark(frame.dots, frame.owner, 0, dot(centre), TRACK)
    for mark in marks:
        at, mark_cls = mark[0], mark[1]
        machine._mark(frame.dots, frame.owner, 0, dot(at), mark_cls,
                      mark[2] if len(mark) > 2 else machine.GAUGE_Y)
    return frame.lines(INK, colour=colour)[0]


#: The dot rows a held peak takes on a gauge: the top one alone, so it
#: reads apart from a burst extreme's full-height tick beside it.
PEAK = (0,)

def bar(share, cells, cls=SOA_OK, tip=MARK, colour=True):
    """One row, `cells` wide: a SOLID level - every dot of every cell to the
    level, `⣿⣿⣿` in `cls` - ending in a column of `tip`'s ink, `⡇` or `⢸`
    whichever lane the level ends in, and the rest of the scale a track
    of grey columns the cell's full height, `⡇` a cell.
    """
    frame = Frame(cells, 1)
    wide = cells * DOTS_X
    end = int(max(0.0, min(1.0, share)) * wide + 0.5)
    at = min(end, wide - 1)             # the tip: the first dot past
    for x in range(at - at % DOTS_X):   # the whole cells before its cell
        col = x // DOTS_X
        for y in range(DOTS_Y):
            frame.dots[0][col] |= machine.BRAILLE_BITS[x % DOTS_X][y]
        if cls > frame.owner[0][col]:
            frame.owner[0][col] = cls
    machine._mark(frame.dots, frame.owner, 0, at, tip, range(DOTS_Y))
    for col in range(at // DOTS_X + 1, cells):
        for y in range(DOTS_Y):
            frame.dots[0][col] |= machine.BRAILLE_BITS[0][y]
        if TRACK > frame.owner[0][col]:
            frame.owner[0][col] = TRACK
    return frame.lines(INK, colour=colour)[0]


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
    """Which band a node's margin is in - `machine.SOA_CLASS`'s order."""
    if tripped or share >= 1.0:
        return machine.SOA_TRIP
    return machine.SOA_WARN if share >= THROTTLE_AT else machine.SOA_OK


def thermometer_class(celsius):
    """Which band of the thermometer ramp a reading is in."""
    ramp = machine.NTC_RAMP
    step = int(temp_share(celsius) * (len(ramp) - 1) + 0.5)
    return ramp[max(0, min(len(ramp) - 1, step))]


def tubes(entries, rows, labels=(), pitch=None, colour=True):
    """A row of thermometers `rows` tall, one per entry, as text lines."""
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
        lines.append(_label_row(labels, len(entries), pitch, colour))
    return lines


def _label_row(labels, count, pitch, colour):
    """The labels under the tubes, one pitch each, padded to `count`."""
    row = ''.join(label[:pitch].center(pitch)
                  for label in labels + [''] * (count - len(labels)))
    if not colour:
        return row.rstrip()
    return ansi.paint(row, INK[TRACK] + 6)
