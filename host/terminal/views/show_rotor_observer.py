#!/usr/bin/env python3
"""The rotor observer: the drive watched live, on the model or the converters.

    python terminal/views/show_rotor_observer.py --simulated
    python terminal/views/show_rotor_observer.py --port COM4 --source model --iq 0.5
    python terminal/views/show_rotor_observer.py --port COM4 --source model --switch

The drive (0x6E device 10) runs on the board at the PWM rate. This page sets
its setpoints and draws the estimated rotor on the dial, the model's rotor
beside it when the model is the source, the dq currents, the innovation and
the interrupt's cost.

    S       start / stop                 M   mode: sensorless, hold, volt
    V       source: model / adc          I   injection on / off
    + -     iq_ref                       [ ] step size
    O / L   I/f speed target up / down   R   reset the model's rotor
    E       demo cycle: hold, rock to 200 rpm, send at the clamp, brake
    W       load loop: d current up and down; the watts and thermometers follow
    B       heavy start: 1 s at the clamp, then 3 s at half the no-load speed
            against a load
    T       tare the rotor mark
    A       arm / disarm the stage - only with --switch
    Q / ESC close / menu

An ellipse is the terminal's font: `--cell-aspect` (round at 2.0, measured
25.16 cell-widths each way; 2.4 rounds it at this size).

The speed loop (iq, the shaft) and the load loop (id) are independent. The
stand-in starts the speed loop on its own; a board starts neither unasked.

Every drive parameter is a switch, checked before it is written (`sane`):
trip <= the FETs' rating, clamp <= trip, injection <= a fifth of the link's
linear range, the stage armed only with --switch.
"""
import argparse
import math
import os
import sys
import time
import types
from contextlib import suppress

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from rich.text import Text                                  # noqa: E402

from coaxial.draw import braille, machine  # noqa: E402
from coaxial.model import motor, thermal as _thermal  # noqa: E402
from coaxial.simulated.power import SimulatedThermal   # noqa: E402
from coaxial.errors import RigError                         # noqa: E402
from coaxial.devices.thermal_device import THROTTLE_AT             # noqa: E402
from coaxial.draw.gauges import (TEMP_FLOOR_C, TEMP_SCALE_C,  # noqa: E402
                                 margin_class as soa_class, temp_share,
                                 thermometer_class as ntc_class)
from terminal.screen import (ASH, SODIUM, TO_MENU, closing, frame_of, hud, open_rig,  # noqa: E402
                             run_view, say, stage, tint)

from terminal import screen as _screen                                   # noqa: E402
from terminal.stage import HUD_WIDTH, UP                            # noqa: E402
_screen.CHATTER = False     # the boot bar replaced the scroll

#: The stage's own rating, which is the name: no trip past it.
RATING_A = 100.0

#: What a sane run stays inside, in SI. Bounds, not judgements about a
#: motor: they keep a typo from becoming a shoot-through or a runaway.
LIMITS = {
    'iq': (-RATING_A, RATING_A), 'id': (-RATING_A, RATING_A),
    'omega': (0.0, 3000.0), 'accel': (1.0, 20000.0),
    'v_inj': (0.0, 6.0), 'inj_periods': (1, 8),
    'kp': (0.0, 10.0), 'ki': (0.0, 1e5), 'l1': (0.0, 1.0), 'l2': (0.0, 1e6),
    'i_max': (0.0, RATING_A), 'i_trip': (0.0, RATING_A),
    'w_lo': (0.0, 5000.0), 'w_hi': (0.0, 5000.0),
    'vdc': (1.0, 80.0), 'load': (0.0, 5.0), 'noise': (0.0, 5.0),
    'theta0': (-math.pi, 2.0 * math.pi), 'sat': (0.0, 0.9),
}

MODES = ('sensorless', 'hold', 'volt')
STEPS = (0.05, 0.1, 0.25, 0.5, 1.0)

#: The drawing's height before `fit` sizes it to the terminal. The can is
#: sized against its own band, not the height, so rows come off without it
#: running into the foot gauges: 28 dots of radius hold down to 24 rows.
NOMINAL_HEIGHT = 24

#: Caption rows above the drawing: the two margins first, then the gutter
#: groups' names and hottest readings beside the NTC. One foot row: WINDING,
#: the thermal observer's policy, POWER (bench 2026-09-05).
CAPTION_ROWS, FOOT_ROWS = 5, 1

#: What is left for the machine. No inset at the foot: at 19 rows with one
#: the winding gauge was drawn through the can (measured). Can 2..16,
#: winding 17, watts 18, labels below the box.
class Box:
    """The machine's drawing this frame: its columns, the braille rows the
    art gets, and the rows the page spends on it with the captions and
    the foot.
    """
    def __init__(self, width, height):
        self.width, self.height = width, height
        self.rows = height - CAPTION_ROWS - FOOT_ROWS

#: Rows above the machine (none: the leaders' corner glyph turns them down)
#: and below it (the two floor gauges); `fit` adds them to the can's rows.
HOP_ROWS, FLOOR_GAUGES = 0, 2


#: `stage.frame_of`'s furniture round the drawing: the viewport's frame and
#: padding, 4 columns; the title band, key bar and frame edges, 4 rows.
VIEWPORT_COLUMNS = 4
PAGE_ROWS = 4

#: The narrowest drawing and lowest band on any terminal: below them the
#: gutters' names run into the legend. 52 wide (from 46, the bench's "a
#: shade bigger") is also what a piped run draws.
NOMINAL_WIDTH = 52
MIN_WIDTH = 40
MIN_BAND = 8

BOX = Box(NOMINAL_WIDTH, NOMINAL_HEIGHT)


def _width_for(can):
    """The columns a can of `can` dots needs with the gutters beside it:
    `machine.layout` inverted, so a machine bound by the rows is not left
    in the middle of a wide box with its thermometers at the far edges."""
    room = 2.0 * (can / machine.F_FIT + 1.0)
    lead = LEFT_COLUMNS + machine.BAR_GAP
    trail = RIGHT_COLUMNS + machine.BAR_GAP
    return int(math.ceil(room / machine.DOTS_X)) + lead + trail


def fit(aspect, size=None):
    """Size the box to the can on THIS terminal - and, given the console's
    `size`, to the terminal as it is NOW: the width the page leaves
    beside the instrument column, the band it leaves under the captions,
    and the can the smaller of the two allows, the gutters drawn in
    against it.
    """
    stretch = aspect / machine.DOTS_Y * machine.DOTS_X
    width = (NOMINAL_WIDTH if size is None else
             max(MIN_WIDTH, size.width - HUD_WIDTH - VIEWPORT_COLUMNS))
    band = (10 ** 6 if size is None else
            max(MIN_BAND, size.height - PAGE_ROWS - CAPTION_ROWS - FOOT_ROWS
                - HOP_ROWS - FLOOR_GAUGES))
    can = machine.layout(width, band, LEFT_COLUMNS, RIGHT_COLUMNS,
                         stretch=stretch)[1].can
    width = min(width, max(MIN_WIDTH, _width_for(can)))
    rows = int(math.ceil((2.0 * can + 2.0) / (machine.DOTS_Y * stretch)))
    BOX.width = width
    BOX.rows = HOP_ROWS + rows + FLOOR_GAUGES
    BOX.height = BOX.rows + CAPTION_ROWS + FOOT_ROWS
    return BOX.rows

#: Amber within this many of the chain's corner `wc`; below one corner the
#: chain calls itself invalid (red): `v - R i` has no back-EMF there.
FLOOR_MARGIN = 3.0

#: The demo cycle's period, seconds.
SWEEP_S = 16.0
#: The demo's speed-loop gain, A per rpm of error per SECOND: stepped per
#: frame it wound up to 475 rpm at 20 Hz.
ROCK_GAIN = 0.01

#: The heavy start (B, and every BURST_EVERY_S on the stand-in): 43 A for
#: 1 s, bounded by heat, not the clamp. Measured on the stand-in 2026-09-06:
#: 0.57 of the span on a cold board at the 80 % floor, 0.82 warm and STABLE;
#: 1.4-1.8 s reached no higher (0.68, 0.71). 38 A into a 40 A clamp reached
#: 0.70, so the clamp went to 50.
BURST_A = 43.0
BURST_S = 1.0
BURST_ACCEL = 12000.0
#: A burst every 45 s on the stand-in, long against the loops' 20 and 40 s;
#: then the burn: 3 s at 20 A against BURST_LOAD_NM at half the no-load
#: speed, where back-EMF times current reaches the kW bar.
BURST_EVERY_S = 45.0
BURST_HOLD_S = 3.0
BURST_HOLD_A = 20.0
#: Sized so the burn settles at half the no-load speed (torque x fade =
#: b w + load): 0.8 N.m sat at 778 of 3902 rpm, 0.42 sits near 1950.
BURST_LOAD_NM = 0.42

#: The load loop: 30 A peak, 40 s period (the legs' constant is seconds,
#: the board's minutes), rewritten when it moves 0.2 A.
LOAD_PEAK_A = 30.0
LOAD_PERIOD_S = 40.0
LOAD_GRAIN = 0.2
#: The legend's bar, cells, in the full braille cell the picture is made of.
BAR_CELLS = 12
#: The power face: (W / WATTS_SCALE) ^ p, p set so WATTS_MID is half the bar:
#: 20 W a tenth, 100 W 22 %, 500 W half, 2 kW full, red past it. Linear hid
#: the page's 97 W peak in two cells; logarithmic put it at 60 %. The figure
#: beside the bar is the watts, undistorted.
WATTS_SCALE = 2000.0
WATTS_MID = 500.0
#: Where the headroom gauge stops being green. The scale's own, not the
#: board's - see `headroom_class`.
HEADROOM_AMBER = 0.5
#: The two margins as one-column tubes outboard of the board's four: S, the
#: switches' worst node against the record's ceilings, which the board acts
#: on; M, the winding (3 i^2 R into a placeholder pair on TEMP_SCALE_C),
#: which only the operator can act on.
HEADROOM_TITLES = ('S', 'M')

#: Their names on the leaders. SWITCH, not BOARD: the margin is the six leg
#: nodes and the laminate they heat; the MCU and regulators trip but are not
#: in it (`soa_undriven_mask`).
HEADROOM_NAMES = ('SWITCH SOA', 'MOTOR SOA')

#: Air between the board's four tubes and the two margins, which are not
#: node temperatures. Two, so both gutters are eight columns (at one, the
#: right leader ran a column short, measured).
HEADROOM_GAP = 2

BAR_GLYPH = chr(0x28FF)
TRACK_GLYPH = chr(0x2812)
#: The degree sign as U+1D52: U+00B0 is East Asian Ambiguous width, and a
#: terminal set for it shears every caption row carrying a temperature.
DEGREE = chr(0x1D52)

#: The thermal nodes a duty cycle can drive into the SOA: the shunt a
#: phase current crosses and the half-bridge above it, per leg. Named in
#: `coaxial.model.thermal`, ordered here the way a leg is read.
SOA_NODES = ('driver_u', 'phase_u', 'driver_v', 'phase_v',
             'driver_w', 'phase_w')
#: The nodes no duty cycle drives, in the right gutter. Both gutter figures
#: are their group's hottest tube on one temperature scale: idle, the MCU
#: (0.666 W through a linear LDO, 15 K over the copper) is hotter than a FET
#: carrying nothing, and the caption names the tube beside it.

BOARD_NODES = ('mcu', 'regulators', 'afe', 'board')

#: The thermistor's colour ramp, C: the page's scale (`coaxial.draw.gauges`, one
#: for every thermometer and the winding), not a limit (invariant 10).
NTC_COLD_C, NTC_HOT_C = TEMP_FLOOR_C, TEMP_SCALE_C

#: The NTC outermost in the left gutter, a column from the six estimates:
#: the one measurement stands apart.
NTC_GAP = 1
NTC_AT = len(SOA_NODES) + NTC_GAP

#: How many columns the left gutter needs.
LEFT_COLUMNS = NTC_AT + 1

#: Where the pair starts inside the right gutter.
HEADROOM_AT = len(BOARD_NODES) + HEADROOM_GAP

#: How many columns the right gutter needs for all of it.
RIGHT_COLUMNS = HEADROOM_AT + len(HEADROOM_TITLES)


def sane(args):
    """Refuse a run outside LIMITS, and the two relations between them."""
    for name, (lo, hi) in LIMITS.items():
        value = getattr(args, name)
        if value is None:
            continue
        if not lo <= value <= hi:
            raise SystemExit('--%s %s is outside %s..%s' % (
                name.replace('_', '-'), value, lo, hi))
    if args.i_max is not None and args.i_trip is not None \
            and args.i_max > args.i_trip:
        raise SystemExit('--i-max %s is above --i-trip %s: the clamp cannot '
                         'exceed the trip' % (args.i_max, args.i_trip))
    if args.v_inj is not None and args.v_inj > 0.2 * args.vdc / math.sqrt(3.0):
        raise SystemExit('--v-inj %s is more than a fifth of the link\'s '
                         'linear range (%.2f V at %.0f V)'
                         % (args.v_inj, 0.2 * args.vdc / math.sqrt(3.0),
                            args.vdc))


def parameters(args):
    """The drive and model parameters the switches asked for, SI."""
    drive = {}
    for name, key in (('kp', 'drv_kp_mv_per_a'), ('ki', 'drv_ki_v_per_as'),
                      ('l1', 'drv_l1_milli'), ('l2', 'drv_l2_milli'),
                      ('i_max', 'drv_i_max_ma'), ('i_trip', 'drv_i_trip_ma'),
                      ('w_lo', 'drv_w_lo_mrad_s'), ('w_hi', 'drv_w_hi_mrad_s'),
                      ('inj_periods', 'drv_inj_periods')):
        if getattr(args, name) is not None:
            drive[key] = getattr(args, name)
    model = {}
    for name in ('vdc', 'load', 'noise', 'theta0', 'sat', 'j', 'b'):
        if getattr(args, name) is not None:
            model[name] = getattr(args, name)
    return drive, model


def eps_gain(params, v_inj, ts):
    """Demodulated amps per radian, from the record's Ld and Lq."""
    ld, lq = params['motor_ld_nh'], params['motor_lq_nh']
    return v_inj * ts * (lq - ld) / (ld * lq) if ld > 0.0 and lq > 0.0 else 0.0


#: The instrument column is 40 cells: a label of 9, a value of 24.
def drive_rows(view):
    s = view['state']
    return [
        ('mode', Text.from_ansi(mode_text(view))),
        ('source', view['source'].upper()),
        ('stage', 'ARMED' if s['stage_enabled'] else 'idle'),
        ('AFE', 'on' if s['afe_on'] else 'off'),
        ('fault', s['fault'] or 'none'),
        ('inject', '%7.2f V   fs/%d' % (view['v_inj'], 2 * view['inj_periods'])
         if view['inject'] else '%7s' % 'off'),
        ('iq ref', '%+7.2f A   step %.2f' % (view['iq'], view['step'])),
        ('I/f', '%7.0f rad/s' % view['omega']),
    ]


def observer_rows(view):
    """Seven rows always, the numbers on one right edge: a row that came
    and went with the model's reply moved every box under it, and units
    that started where the previous number ended read as a scatter."""
    s = view['state']
    pp = max(1.0, view['params'].get('motor_pole_pairs', 1.0))
    deg = math.degrees(s['theta_hat'])
    m = view.get('model') if view['source'] == 'model' else None
    # Seven-wide numbers, five-wide units, then the second pair: 26 cells,
    # which is what the column leaves beside a nine-letter label.
    return [('theta_hat', '%7.1f %-5s %6.1f mech' % (deg, 'deg', deg / pp)),
            ('omega_hat', '%7.0f %-5s %6.0f rpm'
             % (s['omega_hat'], 'rad/s',
                s['omega_hat'] / pp * 60.0 / math.tau)),
            ('model', '%7.1f %-5s %6.0f rad/s'
             % (math.degrees(m['theta']), 'deg', m['omega']) if m
             else '%7s %-5s %6s rad/s' % ('--', 'deg', '--')),
            ('error', '%+7.2f deg' % math.degrees(m.get('error', 0.0)) if m
             else '%7s deg' % '--'),
            ('innov', '%+7.4f rad' % s['eps']),
            ('i_h', '%7.3f A' % s['ih']),
            ('bemf', '%+7.3f rad' % s['e_bemf'])]


def travel(view):
    """How far the rotor has actually turned, in mechanical degrees."""

    now = time.monotonic()
    was = view.get('travel_at')
    view['travel_at'] = now
    if was is None:
        return
    view['travel'] += pointer_rate(view) * min(0.5, now - was)


def pointer_rate(view):
    """How fast the bead travels, degrees a second, signed: the loop's
    speed over the pole pairs - the one number `travel` integrates and
    the drawing trails the bead by, so the wake and the travel agree."""
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    speed = (view.get('chain') or {}).get('omega') or 0.0
    return math.degrees(speed / pairs)


def _place(row, name, columns, right_edge=False, until=None):
    """Write `name` over `columns`, centred, clamped to the frame."""
    if not columns or not name:
        return None
    middle = (min(columns) + max(columns)) / 2.0
    at = (BOX.width - len(name) if right_edge
          else int(round(middle - (len(name) - 1) / 2.0)))
    # A NAME WIDER THAN ITS GROUP LEANS INWARD.
    if not right_edge:
        at = min(at, max(columns) + 1 - len(name))
    if until is not None:
        at = min(at, until + 1 - len(name))
    at = max(0, min(BOX.width - len(name), at))
    row[at:at + len(name)] = name
    return at


def reference(view):
    """The NTC, as text: the one MEASURED temperature on this page."""
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
    # THE MEASUREMENT FIRST: everything under it is an estimate.
    seen = (view.get('thermal') or {}).get('ntc')
    if len(left) > NTC_AT and seen is not None:
        # Its own tube's colour: the thermometer ramp, not a margin's.
        said.append(_legend(0, reference(view),
                            machine.INK[ntc_class(seen)], left[NTC_AT], True))
    # SWITCH SECOND AND BOARD LAST, with the motor's margin between them.
    for group, columns, name, centred in (
            (SOA_NODES, left, 'SWITCH TEMPS', True),):
        peak, cls = hottest(view, group)
        if peak is None or not columns:
            continue
        # THE MIDDLE OF ITS OWN GROUP, not the edge nearest the machine.
        seat = columns[len(columns) // 2 - 2]
        said.append(_legend(len(said), '%s %.1f %sC' % (name, peak, DEGREE),
                            machine.INK[cls], seat, centred))

    # THE MARGINS LAST, under the NTC and nearest the tubes they name.
    for index in reversed(range(len(HEADROOM_TITLES))):
        if len(right) > HEADROOM_AT:
            share, cls = bars[index]
            said.append(_legend(
                len(said),
                # A DECIMAL, BECAUSE THE TUBE CANNOT SHOW THIS.
                '%s %.1f %%' % (HEADROOM_NAMES[index], 100.0 * share),
                machine.INK[cls], right[HEADROOM_AT + index], True))
    peak, cls = hottest(view, BOARD_NODES)
    if peak is not None and right:
        # ONE TUBE FURTHER IN than the middle of its four.
        said.append(_legend(
            len(said), 'BOARD TEMPS %.1f %sC' % (peak, DEGREE),
            machine.INK[cls], right[len(BOARD_NODES) // 2 - 1], True))

    return said


def foot_furniture():
    """The two upside-down L's under the drawing, `(leaders, rules)`."""
    first, last = machine.span(BOX.width, BOX.rows,
                               LEFT_COLUMNS, RIGHT_COLUMNS)
    grey = machine.LEADER_GREY
    # BOTH RUN PAST THE LAST ART ROW, because both carry on into the foot line
    # where the arrowhead is.
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
            # CENTRED OVER THE MACHINE when the head is out in a gutter's
            # middle, hard against the head when it is the outermost tube.
            if column < first:
                at = first + 2
            else:
                at = last - 2 - len(text) + 1
            at = max(0, min(BOX.width - len(text), at))
            line[at:at + len(text)] = text
            marks.append((at, len(text), ink))
            # THE HEAD AGAINST THE WORDS, the run in dots.
            if column < at:
                head, span = at - 2, list(range(column, at - 3))
                line[head] = AIM_LEFT
            else:
                head = at + len(text) + 1
                span = list(range(head + 2, column + 1))
                line[head] = AIM_RIGHT
            # A COLUMN OF AIR BEFORE ANYTHING ALREADY FALLING.
            span = [step for step in span
                    if line[step] not in DROP
                    and not (step + 1 < BOX.width
                             and line[step + 1] in DROP)]
            for step in span:
                line[step] = LEADER
            # AND THE FAR END TURNS, whatever the air rule did to the run.
            turned = 0 <= column < BOX.width and line[column] not in DROP
            if turned:
                line[column] = TURN[_lane(column, first)]
            # ONE MARK A CELL.
            if turned and column not in span:
                marks.append((column, 1, machine.LEADER_GREY))
            marks.append((head, 1, machine.LEADER_GREY))
            for step in span:
                marks.append((step, 1, machine.LEADER_GREY))
        rows.append((line, marks))

    return [_tinted(line, marks) for line, marks in rows]


def _foot_line(view):
    """The row under the box: the winding and the link power."""
    # THE TWO ALONG THE FOOT, named under them and CARRYING THEIR OWN NUMBERS.
    head = '%s WINDING %5.1f %sC' % (UP, winding(view), DEGREE)
    tail = 'POWER %5.2f kW %s' % (watts(view) / 1000.0, UP)
    # A STROKE EACH, LEAVING THE HEAD AND RISING toward the level above it.
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


#: The identification's states as the foot says them (the bench's
#: abbreviations), in the margin's inks.
POLICY_WORD = {'STABLE': 'STABLE', 'CONVERGING': 'CONV',
               'UNCERTAIN': 'UNCR'}
POLICY_SHORT = {'STABLE': 'STBL'}
POLICY_INK = {'STABLE': machine.SOA_OK, 'CONVERGING': machine.SOA_WARN,
              'UNCERTAIN': machine.SOA_TRIP}


def _policy(view):
    """`(label, word, ink)` for the foot: TH OBS and the state with the
    margin the board acts on now - `UNCR 80%`, `CONV 93%`, `STBL 97%`,
    `STABLE` once whole (continuous since 2026-09-06) - or a grey dash
    before op 10 answers.
    """
    ident = view.get('ident')
    state = ident['state'] if ident else None
    if state in POLICY_INK:
        word, ink = _policy_word(ident, state)
        return 'TH OBS', word, ink
    return 'TH OBS', '-', machine.LEADER_GREY


def _policy_word(ident, state):
    """The state's word with the margin's percent, or the trip's."""
    margin = ident.get('margin', 1.0)
    percent = int(round(100.0 * margin))
    cap = ident.get('trip_cap', 1.0)
    floor = ident.get('margin_floor', _thermal.IDENT_MARGIN_FLOOR)
    if cap < 1.0 and abs(margin - cap) < 1e-6 and margin < floor - 1e-6:
        return 'TRIP %d%%' % percent, machine.INK[machine.SOA_TRIP]
    word = POLICY_WORD[state]
    if percent < 100:
        word = '%s %d%%' % (POLICY_SHORT.get(state, word), percent)
    return word, machine.INK[POLICY_INK[state]]


def gutter_caption(view):
    """The caption rows above the drawing, and the one under its foot."""
    left, right = machine.gutters(BOX.width, BOX.rows,
                                  LEFT_COLUMNS, RIGHT_COLUMNS)
    return _legend_rows(view, left, right) + [_foot_line(view)]


def phase_amps(view):
    """The three phase currents, and what to call full scale."""
    s = view['state']
    theta = s['theta_hat']
    cos, sin = math.cos(theta), math.sin(theta)
    alpha = s['id'] * cos - s['iq'] * sin
    beta = s['id'] * sin + s['iq'] * cos
    root3 = math.sqrt(3.0) / 2.0
    amps = (alpha, -0.5 * alpha + root3 * beta, -0.5 * alpha - root3 * beta)
    # FULL SCALE IS THE VECTOR, NOT THE TRIP.
    return amps, math.hypot(s['id'], s['iq'])


def phase_rows(view):
    """The legend: which colour is which phase, and how hard it is driven."""

    amps, full = phase_amps(view)
    scale = full or max((abs(a) for a in amps), default=0.0) or 1.0
    rows = []
    for name, cls, value in zip(machine.PHASE_NAMES, machine.PHASE_CLASS, amps):
        ramp = machine.PHASE_RAMP[cls]
        share = min(1.0, abs(value) / scale)
        step = min(len(ramp) - 1, int(share * (len(ramp) - 1) + 0.5))
        bar = Text()
        bar.append(BAR_GLYPH * max(1, int(share * BAR_CELLS + 0.5)),
                   style='color(%d)' % ramp[step])
        bar.append(' ' * (BAR_CELLS - len(bar.plain)))
        bar.append('%+7.3f A  %s' % (value, 'push' if value >= 0 else 'pull'))
        rows.append((name, bar))
    return rows


def identity(view):
    """What machine the record says is on the shaft, or that it cannot say."""
    params = view['params']
    pairs = int(params.get('motor_pole_pairs') or 0)
    lam = params.get('motor_lambda_uvs') or 0.0
    if not pairs or not lam:
        return Text(' IDENTIFYING ', style='chip.sim')
    kv = motor.Parameters('', params.get('motor_r_uohm') or 0.0,
                          params.get('motor_ld_nh') or 0.0,
                          params.get('motor_lq_nh') or 0.0,
                          lam, pairs).kv
    return '%dN%dP %2d pp  KV %4.0f' % (view['slots'], 2 * pairs, pairs, kv)


def torque(view):
    """Shaft torque from the loop's own dq, newton-metres."""
    s, params = view['state'], view['params']
    pairs = params.get('motor_pole_pairs') or 0.0
    lam = params.get('motor_lambda_uvs') or 0.0
    ld = params.get('motor_ld_nh') or 0.0
    lq = params.get('motor_lq_nh') or 0.0
    return 1.5 * pairs * (lam * s['iq'] + (ld - lq) * s['id'] * s['iq'])


def status_rows(view):
    """Two rows, and neither of them is anywhere else on the page."""
    o = view.get('chain') or {}
    gone = view['travel'] - view['tare']
    loops = ' + '.join([n for n, on in (('speed', view['spin']),
                                        ('load', view['load'])) if on])
    if view['spin'] and view['stage']:
        loops = '%s: %s' % (view['stage'].upper(), loops)
    if time.time() < view['burst_until']:
        loops = 'BURST' + (' + ' + loops if loops else '')
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    speed = (view.get('chain') or {}).get('omega') or 0.0
    return [('machine', identity(view)),
            ('shaft', '%8.0f rpm %8.3f N.m'
             % (speed / pairs * 60.0 / math.tau, torque(view))),
            ('back-EMF', regime(view)),
            ('loops', loops or 'none - the drive is on its own'),
            ('travel', '%9.1f deg %7.2f turns %s'
             % (gone, gone / 360.0,
                'cw' if (o.get('omega') or 0.0) >= 0.0 else 'ccw')),
            # THE CELL'S SHAPE, AND WHERE THE NUMBER CAME FROM.
            ('cell', '%.2f tall %s' % (view.get('aspect', machine.CELL_ASPECT),
                                       view.get('aspect_how', 'assumed')))]


def regime(view):
    """The status chip: which commutation is running, and whether it can."""
    o = view.get('chain') or {}
    speed = abs(o.get('omega') or 0.0)
    corner = o.get('wc') or 0.0
    if not o.get('valid'):
        return Text(' NONE - STEPPING ', style='alarm')
    if corner and speed < FLOOR_MARGIN * corner:
        return Text(' NEAR THE FLOOR ', style='chip.sim')
    return Text(' HOLDING ', style='chip.live')


def no_load_rpm(view):
    """What the link will spin this machine to with nothing on the shaft."""
    params = view['params']
    lam = params.get('motor_lambda_uvs') or 0.0
    pairs = max(1.0, params.get('motor_pole_pairs') or 1.0)
    vdc = (view['state'] or {}).get('vdc') or 0.0
    if lam <= 0.0 or vdc <= 0.0:
        return 0.0
    return vdc / (math.sqrt(3.0) * lam) / pairs * 60.0 / math.tau


def heavy_start(rig, view):
    """A second at the clamp, then the burn."""
    drive = rig.board.drive
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    left = view['burst_until'] - time.time()
    if left > BURST_HOLD_S:
        # Breaking away: everything the clamp allows, at the top of the speed
        # range, and no load in the way of it.
        drive.model_param(load=0.0)
        drive.setpoint(id_ref=0.0, iq_ref=BURST_A, accel=BURST_ACCEL,
                       omega_target=no_load_rpm(view) / 60.0 * math.tau * pairs)
        view['iq'] = BURST_A
        return
    # Burning: half the no-load speed, and a load to make the volts and the
    # amps happen at the same time.
    drive.model_param(load=BURST_LOAD_NM)
    drive.setpoint(id_ref=0.0, iq_ref=BURST_HOLD_A, accel=BURST_ACCEL,
                   omega_target=no_load_rpm(view) / 120.0 * math.tau * pairs)
    view['iq'] = BURST_HOLD_A


def turn_the_handle(rig, view):
    """Whichever of the three is driving this frame, and only one of them."""
    if view['state']['mode'] == 'off':
        return
    now = time.time()
    # THE BURST IS PART OF THE SEQUENCE, not only a key.
    if (view['simulated'] and view['spin']
            and now - view['burst_at'] > BURST_EVERY_S):
        view['burst_at'] = now
        view['burst_until'] = now + BURST_S + BURST_HOLD_S
    if now < view['burst_until']:
        heavy_start(rig, view)
        view['bursting'] = True
        return
    if view['bursting']:
        view['bursting'] = False
        view['leaning'] = False
        rig.board.drive.model_param(load=0.0)
        rig.board.drive.setpoint(id_ref=0.0, iq_ref=view['iq'])
    if view['load']:
        load_loop(rig, view)
    if view['spin']:
        sweep(rig, view)


def load_loop(rig, view):
    """D current up and back down, continuously, the shape the speed loop
    has.
    """
    now = time.time()
    phase = ((now - view['load_at']) % LOAD_PERIOD_S) / LOAD_PERIOD_S
    ramp = 2.0 * phase if phase < 0.5 else 2.0 * (1.0 - phase)
    view['load_amps'] = LOAD_PEAK_A * ramp
    view['load_rising'] = phase < 0.5
    # Only when it has moved enough to matter: a setpoint is a round trip, and
    # one a frame against a board is the link's whole budget.
    if abs(view['load_amps'] - view['load_written']) >= LOAD_GRAIN:
        view['load_written'] = view['load_amps']
        rig.board.drive.setpoint(id_ref=view['load_amps'])


#: The demo cycle as fractions of SWEEP_S: hold, rock, send at the clamp,
#: brake. The send heats the legs so the margins move; the brake gets as
#: long as the send (at 0.82 the rotor still turned 1100 rpm at the hold).
CYCLE_HOLD, CYCLE_ROCK, CYCLE_SEND = 0.14, 0.46, 0.73
#: What the rock peaks at, and what the hold holds with.
ROCK_RPM = 200.0
HOLD_A = 12.0
#: The brake pulls the whole clamp above this and proportionally less below,
#: so the rotor lands on zero: a quarter of the electrical no-load speed.
BRAKE_FULL_RAD_S = 700.0



def rearm_after_trip(rig, origin, view):
    """THE STAND-IN'S OPERATOR."""
    if origin.real or not view.get('spin'):
        return
    budget = view.get('budget') or {}
    if not budget.get('trips') or budget.get('tripped'):
        return
    if view['state'].get('stage_enabled') or rig.gates.armed():
        return
    rig.gates.arm(bypass_sto=True, ignore_interlock=True)
    view['rearms'] = view.get('rearms', 0) + 1
    view['said'] = ('re-armed after thermal trip %d - the stand-in\'s '
                    'operator; the envelope is %d %% of the span'
                    % (budget['trips'],
                       int(round(100.0 * policy_margin(view)))))


def cycle_phase(view):
    """Where in the demo cycle we are, and how far into that phase."""
    turn = ((time.time() - view['spin_at']) % SWEEP_S) / SWEEP_S
    if turn < CYCLE_HOLD:
        return 'hold', turn / CYCLE_HOLD
    if turn < CYCLE_ROCK:
        return 'rock', (turn - CYCLE_HOLD) / (CYCLE_ROCK - CYCLE_HOLD)
    if turn < CYCLE_SEND:
        return 'send', (turn - CYCLE_ROCK) / (CYCLE_SEND - CYCLE_ROCK)
    return 'brake', (turn - CYCLE_SEND) / (1.0 - CYCLE_SEND)


def sweep(rig, view):
    """The demo cycle: hold, rock, send, brake, and round again."""
    drive = rig.board.drive
    stage, into = cycle_phase(view)
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    clamp = view['params'].get('drv_i_max_ma') or 5.0
    if stage != view['stage']:
        view['stage'] = stage
        view['leaning'] = False
        drive.model_param(load=0.0)
        drive.mode('hold' if stage == 'hold' else 'sensorless')
    if stage == 'hold':
        drive.setpoint(id_ref=HOLD_A, iq_ref=0.0, omega_target=0.0,
                       theta=0.0)
        view['iq'] = 0.0
        return
    if stage == 'rock':
        # ONE swing each way: two in five seconds gave the integrator 2.8 s a
        # side and it never left 25 rpm.
        target = ROCK_RPM * math.sin(math.tau * into)
        view['iq'] = _toward(view, target, clamp)
        drive.setpoint(id_ref=0.0, iq_ref=view['iq'],
                       omega_target=abs(target) / 60.0 * math.tau * pairs)
        return
    if stage == 'send':
        drive.setpoint(id_ref=0.0, iq_ref=clamp, accel=BURST_ACCEL,
                       omega_target=no_load_rpm(view) / 60.0 * math.tau * pairs)
        view['iq'] = clamp
        return
    # BRAKE: the same current the other way until it is stopped, then let it
    # be.
    turning = (view.get('chain') or {}).get('omega') or 0.0
    share = min(1.0, abs(turning) / BRAKE_FULL_RAD_S)
    view['iq'] = -math.copysign(clamp * share, turning) if share > 0.03 else 0.0
    drive.setpoint(id_ref=0.0, iq_ref=view['iq'], omega_target=0.0)


def _toward(view, rpm, clamp):
    """The speed loop's integrator, one frame."""
    now = time.time()
    dt = min(0.5, max(0.0, now - view['sweep_at']))
    view['sweep_at'] = now
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    turning = ((view.get('chain') or {}).get('omega') or 0.0) \
        / pairs * 60.0 / math.tau
    return max(-clamp, min(clamp,
                           view['iq'] + ROCK_GAIN * (rpm - turning) * dt))


def chain_rows(view):
    """The back-EMF chain that runs beside the loop, and what it costs."""
    o = view.get('chain')
    if not o:
        return [('chain', '%7s' % '--')] * 7
    pp = max(1.0, view['params'].get('motor_pole_pairs', 1.0))
    lam = view['params'].get('motor_lambda_uvs') or 0.0
    share = 'dual' if o['blend'] < 0.5 else 'flux'
    return [('theta', '%7.1f %-5s %6.1f mech'
             % (math.degrees(o['theta']), 'deg',
                math.degrees(o['theta']) / pp)),
            ('omega', '%7.0f %-5s %6.0f rpm'
             % (o['omega'], 'rad/s', o['omega'] / pp * 60.0 / math.tau)),
            ('vs loop', '%+7.2f deg  %s'
             % (math.degrees(o['error']),
                'holding' if o['valid'] else 'no back-EMF')),
            ('torque', '%7.2f %% of the command'
             % (100.0 * math.cos(o['error']))),
            ('carried by', '%7s %-5s %6.0f %%'
             % (share, '', 100.0 * o['blend'])),
            ('hand-over', '%7.0f %-5s %6.0f rad/s'
             % (o['blend_lo'], 'to', o['blend_hi'])),
            ('lambda', '%7.4f %-5s %6.1f %%'
             % (o['lambda_hat'], 'V.s',
                100.0 * o['lambda_hat'] / lam if lam else float('nan')))]


def loop_rows(view):
    s = view['state']
    ts = s['ts'] or 20e-6
    period = 2.0 * (view['gate']['period'] - 1) if view.get('gate') else 4750.0
    exit_ticks = s.get('exit_ticks_max') or 0
    return [('id / iq', '%+7.3f / %+.3f A' % (s['id'], s['iq'])),
            ('vd / vq', '%+7.3f / %+.3f V' % (s['vd'], s['vq'])),
            ('DC link', '%7.2f V' % s['vdc']),
            ('isr', '%7d cyc   %6.1f us'
             % (s['isr_cycles_max'], s['isr_cycles_max'] / 475.0)),
            ('exit', '%7.1f us  %5.0f %% of %.0f'
             % (exit_ticks / 237.5, 100.0 * exit_ticks / max(1.0, period),
                ts * 1e6)),
            ('periods', '%7d' % s['periods'])]


def winding(view):
    """The winding's temperature, estimated, degrees C."""

    budget = view.get('budget') or {}
    if 'winding_c' in budget:
        view['winding'] = budget['winding_c']
        view['winding_at'] = time.monotonic()
        return view['winding']
    now = time.monotonic()
    was, view['winding_at'] = view.get('winding_at'), now
    params = view['params']
    r_phase = params.get('motor_r_uohm') or 0.0
    k = params.get('winding_k_per_w') or motor.WINDING_K_PER_W
    heat = params.get('winding_j_per_k') or motor.WINDING_J_PER_K
    s = view['state']
    amps_rms = math.hypot(s['id'], s['iq']) / math.sqrt(2.0)
    target = _thermal.AMBIENT + 3.0 * amps_rms * amps_rms * r_phase * k
    if was is None:
        view['winding'] = _thermal.AMBIENT
        return view['winding']
    # The stand-in's haste, and only there: the real constant is ~7 min.
    tau = max(1e-3, k * heat) / (SimulatedThermal.HASTE
                                if view['simulated'] else 1.0)
    view['winding'] += (target - view['winding']) * min(1.0, (now - was) / tau)
    return view['winding']


def watts(view):
    """What the stage is putting into the machine, electrical, watts."""
    s = view['state']
    return 1.5 * (s['vd'] * s['id'] + s['vq'] * s['iq'])


def watts_bar(view):
    """The power as a fifth bar past the board's four, `(share, class)`."""
    return watts_share(watts(view))


def watts_share(w):
    """Watts to `(share, class)` on the power face - `WATTS_SCALE` has the
    shape and why.
    """
    power = math.log(0.5) / math.log(WATTS_MID / WATTS_SCALE)
    share = (abs(w) / WATTS_SCALE) ** power
    if share > 1.0:
        return 1.0, machine.SOA_TRIP
    return share, machine.WATTS


def headroom(view):
    """What is left of the whole board's thermal budget, 0 to 1."""
    worst = (view.get('budget') or {}).get('worst')
    return 1.0 - min(1.0, max(0.0, worst)) if worst is not None else 1.0


#: The envelope's word beside the mode, in a red darker than the trip's 196
#: and the pulse's 210: held, not hurt (the bench's words).
THROTTLE_RED = 124


def envelope_acting(view):
    """Whether the board is holding the stage back - throttling, or tripped."""
    budget = view.get('budget') or {}
    return bool(budget.get('throttling') or budget.get('tripped'))


def mode_text(view):
    """`HOLD (NORM)`, `SENSORLESS (THR)`: the mode, and whether the envelope
    is holding it back.
    """
    s = view['state']
    if s['mode'] == 'off':
        return tint('STOPPED', ASH)
    state = (tint('(THR)', THROTTLE_RED) if envelope_acting(view)
             else tint('(NORM)', SODIUM))
    return tint('%s ' % s['mode'].upper(), SODIUM) + state


#: The margin tube's pulse while the envelope acts, Hz; only the colour
#: pulses, the level stays readable. 3 read as an emergency.
FLASH_HZ = 1.5


def flashing(view):
    """Whether this frame takes the bright half of the alarm pulse."""
    if not envelope_acting(view):
        return False
    return (time.monotonic() * FLASH_HZ * 2.0) % 2.0 < 1.0



#: The legend's arrowheads, U+25C2 and U+25B8: the large triangles are East
#: Asian Ambiguous width and shear the rows on a terminal set for it.
AIM_LEFT, AIM_RIGHT = chr(0x25C2), chr(0x25B8)
#: The leader's run, corner and fall, off `coaxial.draw.braille`. The corner
#: carries the run into the lane it falls in, so run and fall join.
LEADER = braille.RUN[2]
TURN = tuple(braille.corner(2, lane, through=True) for lane in (0, 1))
DROP = braille.FALL



def ntc_bar(view):
    """The thermistor as a tube, on the same scale as every other."""
    seen = (view.get('thermal') or {}).get('ntc')
    if seen is None:
        return []
    return [(temp_share(seen), ntc_class(seen))]


def switch_headroom(view):
    """What is left of the SWITCHES' budget, 0 to 1: the worst of the six
    nodes a duty cycle drives (SOA_NODES), each against its own ceiling.
    """
    used = (view.get('budget') or {}).get('used') or {}
    shares = [used[n] for n in SOA_NODES if n in used]
    if not shares:
        return headroom(view)
    return 1.0 - min(1.0, max(0.0, max(shares)))


def headrooms(view):
    """The two margins as gutter tubes: the switches', then the motor's."""
    switch = switch_headroom(view)
    motor = motor_headroom(view)
    budget = view.get('budget') or {}
    # THE LEVEL IS WHAT IS SPENT, not what is left.
    margin = policy_margin(view)
    motor_spent = 1.0 - motor
    if 'winding_used' in budget:
        motor_spent *= margin       # the board's winding, under its policy
    return [((1.0 - switch) * margin, machine.SOA_FLASH if flashing(view)
             else headroom_class(switch)),
            (motor_spent, machine.SOA_FLASH if motor_flashing(view)
             else headroom_class(motor))]


def policy_margin(view):
    """What the identification's state leaves of every ceiling's span -
    the board's number off op 10, one when it has not answered."""
    return float((view.get('ident') or {}).get('margin', 1.0))


def motor_flashing(view):
    """The motor's pulse: the board holding the stage back FOR THE WINDING -
    its own factor under one, or its ceiling reached - since MINOR 12
    made it a node the envelope acts on.
    """
    budget = view.get('budget') or {}
    acting = (budget.get('winding_derate', 1.0) < 1.0
              or budget.get('winding_used', 0.0) >= 1.0)
    if not acting:
        return False
    return (time.monotonic() * FLASH_HZ * 2.0) % 2.0 < 1.0


def motor_headroom(view):
    """What is left of the winding's scale, 0 to 1."""
    budget = view.get('budget') or {}
    if 'winding_used' in budget:
        return max(0.0, 1.0 - budget['winding_used'])
    return motor_headroom_of(winding(view))


def motor_headroom_of(celsius):
    """The same margin from a temperature alone."""
    return 1.0 - temp_share(celsius)


def headroom_class(left):
    """The headroom gauge's colour: green, then amber, then red."""
    if left <= 1.0 - THROTTLE_AT:
        return machine.SOA_TRIP
    return machine.SOA_WARN if left <= HEADROOM_AMBER else machine.SOA_OK


def soa_bars(view, names):
    """`(fraction, class)` per node: HEIGHT IS HEAT, COLOUR IS MARGIN."""
    budget = view.get('budget') or {}
    used = budget.get('used') or {}
    seen = view.get('thermal') or {}
    nodes = seen.get('nodes') or {}
    tripped = bool(budget.get('tripped'))
    out = []
    for name in names:
        if name not in used or nodes.get(name) is None:
            continue
        share = temp_share(nodes[name])
        out.append((max(0.0, min(1.0, share)),
                    soa_class(used[name], tripped)))
    return out


def soa_bar(share, tripped=False):
    """One node's margin as a bar, in the same ink the gutters use."""
    share = max(0.0, min(1.0, share))
    ink = machine.INK[soa_class(share, tripped)]
    bar = Text()
    bar.append(BAR_GLYPH * max(1, int(share * BAR_CELLS + 0.5)),
               style='color(%d)' % ink)
    # The rest of the tube.
    bar.append(TRACK_GLYPH * (BAR_CELLS - len(bar.plain)),
               style='color(%d)' % machine.INK[machine.TRACK])
    return bar


def thermal_rows(view):
    """The six nodes that carry the current, as bars against their ceilings."""

    th, budget = view.get('thermal'), view.get('budget')
    if not th:
        return ['  (not read yet)']
    used = (budget or {}).get('used') or {}
    degrees = th.get('nodes') or {}
    tripped = bool((budget or {}).get('tripped'))
    rows = []
    for node in SOA_NODES:
        if node not in used:
            continue
        bar = soa_bar(used[node], tripped)
        bar.append('%3.0f%% %5.1fC' % (100.0 * used[node],
                                       degrees.get(node, float('nan'))))
        rows.append((_thermal.pretty(node), bar))
    for node in BOARD_NODES:
        if node in used:
            rows.append((node, soa_bar(used[node], tripped).append(
                '%3.0f%% %5.1fC' % (100.0 * used[node],
                                    degrees.get(node, float('nan'))))
                or None))
    rows.append(('headroom', '%9.0f %% left, worst %s'
                 % (100.0 * headroom(view),
                    (budget or {}).get('worst_node', '?'))))
    # THE THROTTLE AND THE BUDGET.
    factor = (budget or {}).get('derate')
    if factor is not None:
        rows.append(('throttle', Text(' %3.0f %% of the clamp ' % (100 * factor),
                                      style='chip.live' if factor > 0.99
                                      else 'chip.sim' if factor > 0.0
                                      else 'alarm')))
    soak = (budget or {}).get('soak_j') or {}
    worst_node = (budget or {}).get('worst_node')
    if worst_node in soak:
        rows.append(('soak', '%9.1f J left in %s' % (soak[worst_node],
                                                     worst_node)))
    # The two gauges along the foot, named in the order they lie there.
    rows.append(('winding', '%9.1f C %s, upper foot bar'
                 % (winding(view),
                    'board' if 'winding_c' in (view.get('budget') or {})
                    else 'est')))
    rows.append(('power', '%9.1f W of %.0f log, lower' % (watts(view),
                                                          WATTS_SCALE)))
    if view['load']:
        rows.append(('load loop', '%9.1f A of %.0f, %s'
                     % (view['load_amps'], LOAD_PEAK_A,
                        'rising' if view['load_rising'] else 'falling')))
    rows.append(('NTC', '%7.1f C' % th['ntc'] if th.get('ntc') is not None
                 else '%7s' % 'unread'))
    # THE ROOM, as identified - the board has no sensor for it - and on the
    # stand-in the one the truth stands in, so the tour can be read off this
    # page too.
    ident = view.get('ident') or {}
    if ident.get('ambient') is not None:
        truth = ident.get('truth') or {}
        rows.append(('room', '%7.1f C identified%s'
                     % (ident['ambient'],
                        '  sim %s %.0f' % (truth['situation'], truth['ambient'])
                        if truth else '')))
    if budget:
        left = budget.get('seconds_to_limit')
        rows.append(('worst', '%-11s %3.0f%%%s'
                     % (budget.get('worst_node', '?'),
                        100.0 * budget['worst'],
                        '  %.0f s' % left if left is not None else '')))
    return rows


def compose(rig, origin, console, view):

    s = view['state']
    # THE MACHINE, NOT A PROTRACTOR.
    pole_pairs = max(1, int(view['params'].get('motor_pole_pairs') or 1))
    # The true rotor is a notch on the can: its gap to the magnet band IS the
    # observer's error, in mechanical units.
    amps, full = phase_amps(view)
    # THE THERMOMETERS ARE NAMED, on a row of their own above them.
    heads = gutter_caption(view)
    # The names in ash, the readings in their own inks already.
    caption = list(heads[:CAPTION_ROWS])
    foot = list(heads[CAPTION_ROWS:])          # FOOT_ROWS of them
    turned = math.degrees(s['theta_hat']) / pole_pairs
    # THE CAN AND THE POINTER ARE DIFFERENT QUANTITIES.
    art = machine.render(turned, view['slots'], 2 * pole_pairs,
                         BOX.width, BOX.rows,
                         # THE SENSOR'S OWN STROKE IS NOT DRAWN.
                         truth_deg=None,
                         amps=amps, full=full, aspect=view['aspect'],
                         pointer_deg=view['travel'] - view['tare'],
                         pointer_rate=pointer_rate(view),
                         left=(soa_bars(view, SOA_NODES)
                               + [None] * NTC_GAP + ntc_bar(view)),
                         right=(soa_bars(view, BOARD_NODES)
                                + [None] * HEADROOM_GAP + headrooms(view)),
                         leaders=legend_drops(view, *machine.gutters(
                             BOX.width, BOX.rows,
                             LEFT_COLUMNS, RIGHT_COLUMNS))
                         + foot_furniture()[0],
                         rules=foot_furniture()[1],
                         top=None,
                         bottom=[(temp_share(winding(view)),
                                  machine.SOA_WARN),
                                 watts_bar(view)],
                         colour=True)
    art = '\n'.join(caption + [art] + foot)
    panels = [('STATUS', status_rows(view)),
              ('DRIVE', drive_rows(view)),
              ('PHASES', phase_rows(view)),
              ('OBSERVER', observer_rows(view)),
              ('CHAIN', chain_rows(view)),
              ('LOOP', loop_rows(view)),
              ('THERMAL', thermal_rows(view))]
    # Paged by `frame_of`, which is every view's; this only says what the boxes
    # are.
    boxes = [hud(*panel) for panel in panels]
    # FIXED-WIDTH LABELS.
    keys = [('S', '%-5s' % ('STOP' if s['mode'] != 'off' else 'START')),
            ('M', '%-10s' % (s['mode'].upper() if s['mode'] != 'off'
                             else view['mode'].upper())),
            ('V', view['source'].upper()), ('I', 'INJ'),
            ('+ -', 'IQ'), ('[ ]', 'STEP'), ('O L', 'I/F'), ('R', 'RESET'),
            ('T', 'TARE'),
            # The word stays and the colour changes: a chip that appeared and
            # vanished moved every key after it.
            ('B', Text('START', style='alarm')
             if time.time() < view['burst_until'] else 'START'),
            ('E', Text('SPEED', style='chip.live') if view['spin']
             else 'SPEED'),
            ('W', Text('LOAD', style='chip.live') if view['load']
             else 'LOAD'),
            # WHO HAS THE MOUSE.
            ('F', Text('MOUSE', style='chip.live') if _screen.holding()
             else 'MOUSE')]
    if view['switch']:
        keys.append(('A', Text('ARMED', style='chip.live')
                     if s['stage_enabled'] else 'ARM  '))
    keys += [('Q', 'EXIT'), ('ESC', 'MENU')]
    if view.get('said'):
        keys.append(('', view['said']))
    return frame_of(console, origin, 'ROTOR OBSERVER', art, boxes, keys)


def start(rig, view):
    """Enter the chosen mode with the setpoints the view holds."""
    d = rig.board.drive
    if view['mode'] == 'sensorless' and view['source'] == 'model':
        d.set_theta(d.model()['theta'] + 0.3)
    d.setpoint(iq_ref=view['iq'], id_ref=view['id'], theta=0.0,
               omega_target=view['omega'] if view['mode'] == 'hold' else 0.0,
               accel=view['accel'], vd=view['vd'], vq=0.0)
    d.mode(view['mode'])
    return 'running %s' % view['mode']


def _key_start_stop(rig, d, key, view):
    if view['state']['mode'] != 'off':
        d.off()
        return 'stopped'
    return start(rig, view)


def _key_mode(rig, d, key, view):
    view['mode'] = MODES[(MODES.index(view['mode']) + 1) % len(MODES)]
    if view['state']['mode'] != 'off':
        return start(rig, view)
    return 'mode %s' % view['mode']


def _key_source(rig, d, key, view):
    view['source'] = 'adc' if view['source'] == 'model' else 'model'
    d.source(view['source'])
    return 'source %s' % view['source']


def _key_inject(rig, d, key, view):
    view['inject'] = not view['inject']
    d.set_params(drv_inj_mv=view['v_inj'] if view['inject'] else 0.0)
    return 'injection %s' % ('on' if view['inject'] else 'off')


def _key_iq(rig, d, key, view):
    view['iq'] += view['step'] if key in '+=' else -view['step']
    view['iq'] = max(-view['i_max'], min(view['i_max'], view['iq']))
    d.setpoint(iq_ref=view['iq'])
    return 'iq_ref %+.2f A' % view['iq']


def _key_step(rig, d, key, view):
    i = STEPS.index(view['step']) + (1 if key == ']' else -1)
    view['step'] = STEPS[max(0, min(len(STEPS) - 1, i))]
    return 'step %.2f A' % view['step']


def _key_omega(rig, d, key, view):
    view['omega'] = max(0.0, min(LIMITS['omega'][1],
                                 view['omega'] + (50.0 if key == 'o' else -50.0)))
    d.setpoint(omega_target=view['omega'] if view['mode'] == 'hold' else 0.0)
    return 'I/f target %.0f rad/s' % view['omega']


def _key_burst(rig, d, key, view):
    view['burst_at'] = time.time()
    view['burst_until'] = view['burst_at'] + BURST_S + BURST_HOLD_S
    return ('heavy start - %.0f A for %.1f s, then %.0f s at half '
            'speed' % (BURST_A, BURST_S, BURST_HOLD_S))


def _key_spin(rig, d, key, view):
    view['spin'] = not view['spin']
    view['spin_at'] = time.time()
    if not view['spin']:
        d.setpoint(omega_target=0.0)
    return ('speed loop running - down through the floor and back'
            if view['spin'] else 'speed loop off')


def _key_load(rig, d, key, view):
    view['load'] = not view['load']
    view['load_at'] = time.time()
    view['load_amps'] = view['load_written'] = 0.0
    if not view['load']:
        d.setpoint(id_ref=0.0)
    return ('load loop running - d current in steps'
            if view['load'] else 'load loop off')


def _key_tare(rig, d, key, view):
    """TARE: the pointer's zero, not the board's."""
    view['tare'] = view['travel']
    return 'tared - the pointer reads travel from here'


def _key_reset(rig, d, key, view):
    d.model_reset()
    d.set_theta(view['theta0'] + 0.3)
    return 'model rotor reset, theta_hat 0.3 rad off it'


def _key_arm(rig, d, key, view):
    if not view['switch']:
        return ''
    if view['state']['stage_enabled']:
        rig.gates.disarm()
        return 'stage disarmed'
    rig.gates.arm(bypass_sto=True, ignore_interlock=not view['interlock'])
    return 'STAGE ARMED - the gates switch'


#: The keys, each `(rig, drive, key, view) -> what to say`. Scrolling the
#: box column is `run_view`'s, on every page.
KEYS = {
    's': _key_start_stop, 'm': _key_mode, 'v': _key_source, 'i': _key_inject,
    '+': _key_iq, '=': _key_iq, '-': _key_iq, '_': _key_iq,
    '[': _key_step, ']': _key_step, 'o': _key_omega, 'l': _key_omega,
    'b': _key_burst, 'e': _key_spin, 'w': _key_load, 't': _key_tare,
    'r': _key_reset, 'a': _key_arm,
}


def act(rig, key, view):
    """One keystroke against the board; returns what to say."""
    handler = KEYS.get(key)
    if handler is None:
        return ''
    try:
        return handler(rig, rig.board.drive, key, view)
    except RigError as exc:
        return str(exc)


def aspect_of(args):
    """What makes the can round on THIS terminal."""
    return _screen.aspect_of(args.cell_aspect)


def parse_args(argv):
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('--port', default='COM4')
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--frames', type=int, default=0)
    p.add_argument('--hz', type=float, default=DEFAULT_HZ)
    # A terminal size to fit to: a piped run draws as that terminal would
    # (`tools/render/ansi2png.py` rasters it).
    p.add_argument('--width', type=int, default=None)
    p.add_argument('--height', type=int, default=None)
    p.add_argument('--source', choices=('model', 'adc'), default='model')
    p.add_argument('--motor', help='a profile under motors/, written first')
    p.add_argument('--cell-aspect', type=float, default=None,
                   help='what makes the can round on THIS terminal. The '
                        'geometry is exactly round at 2.0 - measured, 25.16 '
                        'cell-widths each way - so an ellipse is the font '
                        'being taller than one by two. The steps are coarse: '
                        'a row is four dots, about 8 %% of the diameter, so '
                        'the value that rounds it is not always the font\'s '
                        'true ratio. Try 2.4 if it looks stretched in Y.')
    p.add_argument('--slots', type=int, default=24,
                   help='stator teeth to draw. NOT a measurement: the slot '
                        'count is not in the calibration record and cannot '
                        'be inferred from the pole count, which is. The '
                        'poles are drawn from the record either way.')
    p.add_argument('--mode', choices=MODES, default='sensorless')
    p.add_argument('--switch', action='store_true',
                   help='let A arm the stage: gates.arm(bypass_sto=True)')
    p.add_argument('--interlock', action='store_true',
                   help='honour the arming interlock when A arms')
    p.add_argument('--afe', action='store_true', help='switch AFE_ON on')
    p.add_argument('--start', action='store_true', help='start at once')
    for name, default in (('iq', 0.0), ('id', 0.0), ('omega', 300.0),
                          ('accel', 1500.0), ('v_inj', 1.0), ('vd', 0.5),
                          ('vdc', 24.0)):
        p.add_argument('--' + name.replace('_', '-'), type=float, default=default)
    p.add_argument('--inj-periods', type=int, default=None)
    for name in ('kp', 'ki', 'l1', 'l2', 'i_max', 'i_trip', 'w_lo', 'w_hi',
                 'load', 'noise', 'theta0', 'sat', 'j', 'b'):
        p.add_argument('--' + name.replace('_', '-'), type=float, default=None)
    return p.parse_args(argv)


def preflight(rig, args):
    """Profile, parameters, source - every one checked, then written."""
    d = rig.board.drive
    d.off()
    if args.motor:
        path = args.motor if os.path.exists(args.motor) else os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            'motors', args.motor)
        got = d.profile(path)
        say('ok', 'motor', got['name'])
    drive_params, model_params = parameters(args)
    if drive_params:
        d.set_params(**drive_params)
    if model_params:
        d.model_param(**model_params)
    params = d.params()
    ts = d.state()['ts'] or 20e-6
    # Injection on from the start: at standstill it is the only innovation
    # (without it the estimate ran 71 degrees from the model's rotor).
    d.set_params(drv_inj_mv=args.v_inj,
                 drv_eps_gain_ua_per_rad=eps_gain(params, args.v_inj, ts))
    d.source(args.source)
    say('ok', 'source', '%s%s' % (args.source, ' - the board integrates its own '
                                  'rotor' if args.source == 'model' else ''))
    say('ok', 'trip', '%.1f A clamp, %.1f A trip, rating %.0f'
        % (params['drv_i_max_ma'], params['drv_i_trip_ma'], RATING_A))
    return params


def demo_stage(rig, origin):
    """Give the stand-in a bridge to switch."""
    if origin.real:
        return
    rig.board.gate_drivers.bypass_break(True)
    rig.board.gate_drivers.enable()


#: The page's frame rate unless asked for: a page of numbers.
DEFAULT_HZ = 8.0
#: On the stand-in's model: a moving rotor wants more frames.
DEMO_HZ = 12.0
#: The demo's damping: 5e-4 puts 0.1 A at 100 rpm, the range that decides
#: this drive (the chain stops at 27 rpm, 20 rad/s electrical). The
#: placeholder 1e-5 put 0.08 A at 3900 rpm.
DEMO_B = 5e-4
#: Rotor and load, kg m^2: a 63100 can alone is ~5.4e-4 (0.64 kg at 29 mm);
#: the placeholder 2e-5 reached 3000 rpm inside one frame. A stand-in's
#: number: a bench writes its own (--j, a motor profile).
DEMO_J = 8e-3
#: The torque current it comes up with, and the step `+` and `-` walk it
#: by on the model.
DEMO_IQ = 0.06
DEMO_STEP = 0.01
#: A clamp the load can reach: at the record's 5 A the legs took 0.4 W
#: against 1.8 W of housekeeping; 50 A puts ~40 W in them.
DEMO_I_MAX = 50.0
DEMO_I_TRIP = 70.0
#: The iq step on a board, and on the stand-in's ADC source.
BOARD_STEP = 0.1


def _model_defaults(args):
    """The stand-in's machine on the model: the damping, the inertia, the
    torque current and the clamps above, where the caller left them."""
    if args.b is None:
        args.b = DEMO_B
    if args.j is None:
        args.j = DEMO_J
    if not args.iq:
        args.iq = DEMO_IQ
    if args.i_max is None:
        args.i_max = DEMO_I_MAX
    if args.i_trip is None:
        args.i_trip = DEMO_I_TRIP


def demo_defaults(args, origin):
    """What the stand-in comes up doing, and the iq step to walk it."""
    if origin.real:
        return BOARD_STEP
    args.start = True
    if args.hz == DEFAULT_HZ:
        args.hz = DEMO_HZ
    if args.source != 'model':
        return BOARD_STEP
    _model_defaults(args)
    return DEMO_STEP


def _link(args):
    """Open the board and put the front end where the source needs it."""
    rig = open_rig('LINKING ROTOR OBSERVER', port=args.port,
                   power_afe=False,
                   simulated_device=bool(args.simulated))
    if rig is None:
        return None, None, None, None
    origin, board = rig.origin, rig.board
    if not origin.real:
        # THE TOUR: rooms change as the identification earns them, so TH OBS
        # walks UNCR, CONV, STABLE on the foot (bench 2026-09-06).
        rig.thermal.situation('tour')
    was_on = board.afe.is_on()
    want_afe = args.afe or args.source == 'adc'
    if want_afe != was_on:
        board.afe.set(want_afe)
        time.sleep(0.3)
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))
    # The demo's defaults first: `preflight` hands `args` to the model.
    view_step = demo_defaults(args, origin)
    demo_stage(rig, origin)
    try:
        return rig, preflight(rig, args), was_on, view_step
    except RigError as exc:
        say('fail', 'drive', str(exc))
        rig.close()
        return None, None, None, None


def _sized(args, board_view):
    """The size the page is fitted to: the one asked for, the terminal's,
    or None piped - the nominal drawing."""
    if args.width and args.height:
        return types.SimpleNamespace(width=args.width, height=args.height)
    return board_view.size if board_view.is_terminal else None


def _console_for(args):
    """The console the page draws on - and, piped at a size, laid out for
    that terminal rather than for the eighty columns a pipe is assumed to
    be: the frame cropped the foot's WINDING to DING otherwise, measured.
    """
    board_view = stage()
    if args.width and args.height and not board_view.is_terminal:
        board_view.width, board_view.height = args.width, args.height
    return board_view


def main(argv=None):
    args = parse_args(argv)
    sane(args)

    rig, params, was_on, view_step = _link(args)
    if rig is None:
        return 1
    origin, board = rig.origin, rig.board

    # MEASURED ONCE, at start-up: the cell's shape is the terminal's and cannot
    # change under a running view.
    aspect, aspect_how = aspect_of(args)
    fit(aspect)
    view = {'source': args.source, 'mode': args.mode, 'iq': args.iq,
            'id': args.id, 'omega': args.omega, 'accel': args.accel,
            'vd': args.vd, 'v_inj': args.v_inj, 'inject': True,
            'inj_periods': int(params.get('drv_inj_periods') or 1),
            'step': view_step, 'slots': args.slots, 'switch': args.switch,
            'aspect': aspect, 'aspect_how': aspect_how,
            'spin': not origin.real, 'spin_at': time.time(),
            'simulated': not origin.real,
            'tare': 0.0, 'sweep_at': time.time(),
            'travel': 0.0, 'travel_at': None, 'leaning': False,
            'winding': _thermal.AMBIENT, 'winding_at': None,
            'burst_until': 0.0, 'bursting': False, 'stage': None,
            'burst_at': time.time(),
            'load': False, 'load_at': 0.0, 'load_rising': True,
            'load_amps': 0.0, 'load_written': 0.0,
            'interlock': args.interlock,
            'i_max': params['drv_i_max_ma'], 'theta0': args.theta0 or 0.0,
            'params': params, 'said': '', 'state': board.drive.state(),
            'chain': board.drive.observers(),
            'gate': board.gate_drivers.state(), 'model': None,
            'thermal': None, 'budget': None, 'ident': None}
    if args.start:
        view['said'] = act(rig, 's', view)

    board_view = _console_for(args)
    console = board_view.is_terminal
    # `console` is the flag the keys and the closing want; `compose` gets the
    # console itself.
    leaving = None
    thermal_at = [0.0]
    # HOW OFTEN THE THERMAL OBSERVER IS READ.
    thermal_every = 2.0 if origin.real else 0.25

    def draw():
        with suppress(RigError):
            view['state'] = board.drive.state()
            view['gate'] = board.gate_drivers.state()
            view['model'] = (board.drive.model()
                             if view['source'] == 'model' else None)
            # ONE REPLY FOR THE DIAL AND THE MARK.
            if view['model']:
                view['state']['theta_hat'] = view['model']['theta_hat']
                view['state']['omega_hat'] = view['model']['omega_hat']
            # The chain: a second answer to the angle, no shaft sensor behind it.
            view['chain'] = board.drive.observers()
            travel(view)
            turn_the_handle(rig, view)
            if time.time() - thermal_at[0] > thermal_every:
                view['thermal'] = board.thermal.state()
                view['budget'] = board.thermal.budget()
                view['ident'] = board.thermal.identification()
                thermal_at[0] = time.time()
                rearm_after_trip(rig, origin, view)
        # The console itself: `frame_of` pages on its scroll state and size.
        fit(view['aspect'], _sized(args, board_view))
        return compose(rig, origin, board_view, view)

    def on_input(typed, _moved):
        for key in typed:
            view['said'] = act(rig, key, view) or view['said']

    try:
        leaving = run_view(board_view, console, 1.0 / max(args.hz, 0.5),
                           args.frames, draw, on_input, mouse=True)
    finally:
        done = []
        try:
            board.drive.off()
            done.append(('drive', 'off, the compares released'))
            board.drive.source('adc')
            if rig.gates.armed():
                rig.gates.disarm()
                done.append(('gate stage', 'disarmed, MOE clear'))
            board.gate_drivers.disarm()
            if board.afe.is_on() != was_on:
                board.afe.set(was_on)
            done.append(('AFE_ON', 'back the way it was found'))
        except RigError as exc:
            done.append(('putting it back', 'FAILED: %s' % exc))
        rig.close()
        sys.stdout.write('\n')
        closing(done, console, 0)
    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
