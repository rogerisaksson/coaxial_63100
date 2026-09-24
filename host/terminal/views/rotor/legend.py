"""The rotor observer's gutter: the legend, its leaders to the drawing, the foot line."""
from coaxial.draw import braille, machine
from coaxial.draw.gauges import margin_class as soa_class, thermometer_class as ntc_class
from terminal.ui.screen import tint
from terminal.ui.scroll import UP
from terminal.views.rotor.layout import (BOARD_NODES, BOX, CAPTION_ROWS, HEADROOM_AT,
                                         HEADROOM_TITLES, HOP_ROWS, LEFT_COLUMNS, NTC_AT,
                                         RIGHT_COLUMNS, SOA_NODES)
from terminal.views.rotor.thermal import _policy, headrooms, watts, watts_bar, winding


#: Their names on the leaders. SWITCH, not BOARD: the margin is the six leg
#: nodes and the laminate they heat; the MCU and regulators trip but are not
#: in it (`soa_undriven_mask`).
HEADROOM_NAMES = ('SWITCH SOA', 'MOTOR SOA')

#: The degree sign as U+1D52: U+00B0 is East Asian Ambiguous width, and a
#: terminal set for it shears every caption row carrying a temperature.
DEGREE = chr(0x1D52)


def _place(row, name, columns, right_edge=False, until=None):
    """Write `name` over `columns`, centred, clamped to the frame."""
    if not columns or not name:
        return None
    middle = (min(columns) + max(columns)) / 2.0
    at = (BOX.width - len(name) if right_edge
          else int(round(middle - (len(name) - 1) / 2.0)))
    # A name wider than its group leans inward.
    if not right_edge:
        at = min(at, max(columns) + 1 - len(name))
    if until is not None:
        at = min(at, until + 1 - len(name))
    at = max(0, min(BOX.width - len(name), at))
    row[at:at + len(name)] = name
    return at


def reference(view):
    """The NTC, as text: the one measured temperature on this page."""
    seen = (view.get('thermal') or {}).get('ntc')
    if seen is None:
        return 'NTC  unread'
    return 'NTC %.1f %sC' % (seen, DEGREE)


def hottest(view, names):
    """The hottest node of a group: `(celsius, class)`."""
    nodes = (view.get('thermal') or {}).get('nodes') or {}
    budget = view.get('budget') or {}
    seen = [name for name in names if nodes.get(name) is not None]
    if not seen:
        return None, machine.TRACK
    at = max(seen, key=lambda name: nodes[name])
    return nodes[at], soa_class((budget.get('used') or {}).get(at, 0.0),
                                bool(budget.get('tripped')))


def _tinted(row, marks):
    """`row`, a list of characters, as a string with `marks` coloured."""
    out, cut = [], 0
    for at, length, ink in sorted(marks):
        out.append(''.join(row[cut:at]))
        out.append(tint(''.join(row[at:at + length]), ink))
        cut = at + length
    out.append(''.join(row[cut:]))
    return ''.join(out)


def _lane(column, first):
    """Which half of its cell a leader falls down: 0 left, 1 right."""
    return 0 if column < first else 1


def _legend(row, text, ink, column, centred):
    """One legend: a name with its value, an arrowhead over its own column,
    and the row it was written on.
    """
    return (row, text, ink, column, centred)


def _legend_targets(view, left, right):
    """Every legend as `(row, text, ink, column, centred)`."""
    bars = headrooms(view)
    said = []
    first, last = machine.span(BOX.width, BOX.rows,
                               LEFT_COLUMNS, RIGHT_COLUMNS)
    # The measurement first: everything under it is an estimate.
    seen = (view.get('thermal') or {}).get('ntc')
    if len(left) > NTC_AT and seen is not None:
        # Its own tube's colour: the thermometer ramp, not a margin's.
        said.append(_legend(0, reference(view),
                            machine.INK[ntc_class(seen)], left[NTC_AT], True))
    # SWITCH second and BOARD last, with the motor's margin between them.
    for group, columns, name, centred in (
            (SOA_NODES, left, 'SWITCH TEMPS', True),):
        peak, cls = hottest(view, group)
        if peak is None or not columns:
            continue
        # The middle of its own group, not the edge nearest the machine.
        seat = columns[len(columns) // 2 - 2]
        said.append(_legend(len(said), '%s %.1f %sC' % (name, peak, DEGREE),
                            machine.INK[cls], seat, centred))

    # The margins next, under the NTC and nearest the tubes they name.
    for index in reversed(range(len(HEADROOM_TITLES))):
        if len(right) > HEADROOM_AT:
            share, cls = bars[index]
            said.append(_legend(
                len(said),
                # A decimal: the tube cannot show one.
                '%s %.1f %%' % (HEADROOM_NAMES[index], 100.0 * share),
                machine.INK[cls], right[HEADROOM_AT + index], True))
    peak, cls = hottest(view, BOARD_NODES)
    if peak is not None and right:
        # One tube further in than the middle of its four.
        said.append(_legend(
            len(said), 'BOARD TEMPS %.1f %sC' % (peak, DEGREE),
            machine.INK[cls], right[len(BOARD_NODES) // 2 - 1], True))

    return said


def foot_furniture():
    """The two upside-down L's under the drawing, `(leaders, rules)`."""
    first, last = machine.span(BOX.width, BOX.rows,
                               LEFT_COLUMNS, RIGHT_COLUMNS)
    grey = machine.LEADER_GREY
    # Both run past the last art row: both carry on into the foot line where
    # the arrowhead is.
    return ([(BOX.rows - 2, 0, BOX.rows + 1, grey, 0),
             (BOX.rows - 1, BOX.width - 1, BOX.rows + 1, grey, 1)],
            [(BOX.rows - 2, 0, max(0, first - 1), grey),
             (BOX.rows - 1, min(BOX.width - 1, last + 1), BOX.width - 1,
              grey)])


def legend_drops(view, left, right):
    """One dotted hop into the drawing, under every legend."""
    # None while HOP_ROWS is 0: the corner glyph turns each run down.
    return [(0, column, HOP_ROWS, machine.LEADER_GREY,
             _lane(column, machine.span(BOX.width, BOX.rows,
                                        LEFT_COLUMNS, RIGHT_COLUMNS)[0]))
            for _row, _text, _ink, column, _centred
            in _legend_targets(view, left, right)] if HOP_ROWS else []


def _legend_rows(view, left, right):
    """The caption rows: four legends and the NTC, in dots and text."""
    said = _legend_targets(view, left, right)
    first, last = machine.span(BOX.width, BOX.rows,
                               LEFT_COLUMNS, RIGHT_COLUMNS)
    rows = []
    for index in range(CAPTION_ROWS):
        line = [' '] * BOX.width
        marks = []
        # Leaders already falling pass through first; the words go clear of
        # them, or a leader breaks at the captions.
        for row, _text, _ink, column, _in in said:
            if row < index:
                line[column] = DROP[_lane(column, first)]
                marks.append((column, 1, machine.LEADER_GREY))
        for row, text, ink, column, centred in said:
            if row != index:
                continue
            # Centred over the machine when the head is out in a gutter's
            # middle, hard against the head when it is the outermost tube.
            if column < first:
                at = first + 2
            else:
                at = last - 2 - len(text) + 1
            at = max(0, min(BOX.width - len(text), at))
            line[at:at + len(text)] = text
            marks.append((at, len(text), ink))
            # The head against the words, the run in dots.
            if column < at:
                head, span = at - 2, list(range(column, at - 3))
                line[head] = AIM_LEFT
            else:
                head = at + len(text) + 1
                span = list(range(head + 2, column + 1))
                line[head] = AIM_RIGHT
            # A column of air before anything already falling.
            span = [step for step in span
                    if line[step] not in DROP
                    and not (step + 1 < BOX.width
                             and line[step + 1] in DROP)]
            for step in span:
                line[step] = LEADER
            # The far end turns, whatever the air rule did to the run.
            turned = 0 <= column < BOX.width and line[column] not in DROP
            if turned:
                line[column] = TURN[_lane(column, first)]
            # One mark a cell.
            if turned and column not in span:
                marks.append((column, 1, machine.LEADER_GREY))
            marks.append((head, 1, machine.LEADER_GREY))
            for step in span:
                marks.append((step, 1, machine.LEADER_GREY))
        rows.append((line, marks))

    return [_tinted(line, marks) for line, marks in rows]


def _foot_line(view):
    """The row under the box: the winding and the link power."""
    # The two along the foot, named under them and carrying their own numbers.
    head = '%s WINDING %5.1f %sC' % (UP, winding(view), DEGREE)
    tail = 'POWER %5.2f kW %s' % (watts(view) / 1000.0, UP)
    # A stroke each, leaving the head and rising toward the level above it.
    label, word, ink = _policy(view)
    middle = len(label) + 1 + len(word)
    room = BOX.width - len(head) - len(tail)
    left = max(0, (room - middle) // 2)
    right = max(0, room - middle - left)
    # The figure wears the bar's ink: red past 2 kW, like the bar.
    foot = (tint(head, machine.INK[machine.SOA_WARN])
            + ' ' * left
            + tint(label, machine.LEADER_GREY) + ' ' + tint(word, ink)
            + ' ' * right
            + tint(tail, machine.INK[watts_bar(view)[1]]))
    return foot


def gutter_caption(view):
    """The caption rows above the drawing, and the one under its foot."""
    left, right = machine.gutters(BOX.width, BOX.rows,
                                  LEFT_COLUMNS, RIGHT_COLUMNS)
    return _legend_rows(view, left, right) + [_foot_line(view)]


#: The legend's arrowheads, U+25C2 and U+25B8: the large triangles are East
#: Asian Ambiguous width and shear the rows on a terminal set for it.
AIM_LEFT, AIM_RIGHT = chr(0x25C2), chr(0x25B8)

#: The leader's run, corner and fall, off `coaxial.draw.braille`. The corner
#: carries the run into the lane it falls in, so run and fall join.
LEADER = braille.RUN[2]

TURN = tuple(braille.corner(2, lane, through=True) for lane in (0, 1))

DROP = braille.FALL
