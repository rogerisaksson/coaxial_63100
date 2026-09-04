#!/usr/bin/env python3
"""The rotor observer: the drive watched live, on the model or the converters.

    python tools/show_rotor_observer.py --simulated
    python tools/show_rotor_observer.py --port COM4 --source model --iq 0.5
    python tools/show_rotor_observer.py --port COM4 --source model --switch

The drive (0x6E device 10) runs on the board at the PWM rate; this sets what
it is asked to do and draws what it did: the estimated rotor angle on the
dial, the model's own rotor marked on the rim beside it when the model is the
source, the dq currents, the innovation, the interrupt's cost.

    S       start / stop                 M   mode: sensorless, hold, volt
    V       source: model / adc          I   injection on / off
    + -     iq_ref                       [ ] step size
    O / L   I/f speed target up / down   R   reset the model's rotor
    E       the demo cycle: hold, rock either way to 200 rpm, send it at
            the clamp, brake back to rest - and round again
    W       load loop: d current up and back down, so the watts ramp and
            the thermometers follow them
    B       heavy start: a second at the clamp accelerating hard, which
            takes the phase nodes to the top of their thermal budget,
            then seconds at half the machine's no-load speed against a
            load, which is where the watts are - then back to the loops
            and the cooling
    T       tare the rotor mark
    A       arm / disarm the stage - only with --switch
    Q / ESC close / menu

If the machine reads as an ellipse rather than a circle, that is the
terminal's font and not the drawing: `--cell-aspect` corrects it. The
geometry is round at 2.0 - measured, 25.16 cell-widths each way - and a
taller cell stretches it in Y by the same ratio. A row is four dots, so
the steps are coarse; 2.4 is what rounds it at this size.

The two loops are independent and drive different axes - the speed loop
`iq` and the shaft, the load loop `id` - so either runs alone or both
together. On the stand-in the speed loop starts on its own, because a
page opened onto a rotor that is not turning shows nothing; against a
board neither starts until it is asked for.

Every drive parameter is a switch, and every one is checked against the
stage before it is written: the trip cannot exceed the FETs' rating, the
clamp cannot exceed the trip, the injection cannot exceed a fifth of the
link's linear range, and the stage is never armed unless --switch says it
may be. The board judges nothing; this page does, before it asks.
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rich.text import Text                                  # noqa: E402

from coaxial import machine   # noqa: E402
from coaxial import motor     # noqa: E402
from coaxial.simulated.power import SimulatedThermal   # noqa: E402
from coaxial.errors import RigError                         # noqa: E402
from coaxial.thermal_device import THROTTLE_AT             # noqa: E402
from screen import (ASH, SODIUM, TO_MENU,  # noqa: E402
                    closing, say, tint)

import screen as _screen                                   # noqa: E402
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

#: The dial, drawn smaller than the shaft view's: four instrument boxes
#: sit beside it and the face is a pointer, not a protractor to read.
#: THE TIGHTEST BOX THAT KEEPS THE MACHINE'S SIZE. The can is sized
#: against its own band now - what is left after the foot gauges - so it
#: no longer grows to fill whatever height it is given, and rows can come
#: off without it running into anything. Swept: it holds 28 dots of
#: radius down to twenty-four and only starts shrinking below that, which
#: is where the band becomes the binding dimension rather than the width.
#:
#: It cost an afternoon of adding a row, watching the can grow into the
#: gauges, and adding another. The height was feeding the thing it was
#: meant to fix.
ART_WIDTH, ART_HEIGHT = 46, 24

#: Rows of the box that are captions rather than drawing: five above and
#: one below.
#:
#: THE TOP TWO ARE THE MARGINS, and they are first because they are what
#: a bench looks at first - how much is left of the board and of the
#: windings. Under them the two gutter groups name themselves over two
#: rows and put their hottest reading on a third, beside the NTC.
#:
#: The margins were drawn INSIDE the machine's air for a day, on leaders
#: reaching out to their tubes. It worked and it read as an afterthought:
#: the two things you must not cook were tucked into the drawing while
#: the furniture had the top of the page. Rows are cheap - the can is
#: bound by the width here, so two more cost the machine nothing.
CAPTION_ROWS, FOOT_ROWS = 5, 1

#: What is left for the machine. NINETEEN ROWS AND NO INSET AT THE FOOT:
#: at nineteen with an inset the can came out rows 2..16 and the winding
#: gauge was drawn on row 16, through the bottom of the can - measured.
#: A twentieth row cleared it and left the labels a blank row from the
#: gauges they name; `machine.FLOOR_INSET` clears it the other way, by
#: spending the row of air that was under the bottom gauge. Can 2..16,
#: winding 17, watts 18, labels on the row below the box.
ART_ROWS = ART_HEIGHT - CAPTION_ROWS - FOOT_ROWS

#: HOW CLOSE TO THE FLOOR IS TOO CLOSE. The chain reports `wc`, the leak's
#: corner, and calls itself invalid below it - both observers live on
#: `v - R i` and a rotor that slow makes no back-EMF worth the name. That
#: is the red line and the board's own number, not one invented here.
#: Amber is the approach to it: within this many corners the estimate is
#: still holding but the margin is going, and a drive that waits for red
#: to act has already lost the rotor.
FLOOR_MARGIN = 3.0

#: The demo sweep, simulated only. Down through amber into red, where the
#: back-EMF observers have nothing and the microstepper takes the rotor
#: instead, then up through the envelope. Two hundred rpm and forty
#: seconds because the point is to watch a hand-over happen: a sweep that
#: crosses in a second reads as flicker, and what changes at the crossing
#: is a whole commutation strategy. Twenty seconds a pass and the
#: direction reversed each pass: forty was one reversal a minute and
#: whoever was watching had stopped by then.
#:
#: NINETY RPM AND NOT TWO HUNDRED. At two hundred the rotor turns three
#: times a second and this page redraws twelve, so the rotor mark moved a
#: hundred degrees a frame and read as something flickering rather than
#: something turning - no refresh rate fixes that, only a slower rotor.
#: Ninety just reaches the speed at which the chain is comfortably clear
#: of its floor, which is the top of the range worth showing here.
SWEEP_LO_RPM, SWEEP_HI_RPM, SWEEP_S = 8.0, 200.0, 16.0
#: How hard the sweep pulls the speed toward its target: amps per rpm of
#: error PER SECOND. Closed on the speed rather than open on a current -
#: the current that holds a given speed depends on the damping, and this
#: view is meant to work whatever machine the record describes.
#:
#: Per second and not per frame, which it was: an integrator stepped once
#: a frame is an integrator whose gain is the frame rate, and at twenty
#: hertz this one wound up to 475 rpm chasing a target of 200 and never
#: came back. Against this machine it settles in about two seconds, which
#: is short against a forty-second sweep and long against the rotor.
SWEEP_GAIN = 0.0005
#: The rock's own gain, amps per rpm of error per second. Twenty times
#: `SWEEP_GAIN` because the rock has a few seconds to reach 200 rpm and
#: reverse, where the old triangle had twenty to walk 90.
ROCK_GAIN = 0.01
#: How much of each pass the demo puts current through the legs, and how
#: much. Twenty amps is half the clamp and a fifth of the stage's rating
#: - enough that the thermometers move within seconds, well short of
#: anything the stage would refuse.
SWEEP_LOAD_FRACTION, SWEEP_LOAD_A, SWEEP_LOAD_NM = 0.34, 20.0, 0.015

#: The load loop's peak and its period. Thirty amps is inside the clamp
#: the view writes and well inside the stage's hundred; forty seconds
#: because the legs' own constant is seconds and the board's is minutes,
#: and a cycle shorter than the slow one never shows the lag between
#: them. The smallest change worth a round trip is a fifth of an amp -
#: under what any thermometer here can show.
#: The heavy start: how hard, how long, and how fast it is asked to get
#: there. Forty-three amps is under the clamp this page writes and under
#: half the stage's rating; a second of it takes the phase nodes to
#: about nine tenths of their thermal budget - measured, not aimed at -
#: which is where a start belongs: bounded by HEAT rather than by the
#: current limit, close enough to the ceiling to see the bar go amber
#: and short enough that the envelope does not have to act.
#:
#: At 38 A into a 40 A clamp it only reached 0.70 of the budget, which
#: is not near anything; the clamp went to 50 to make room for this.
BURST_A = 43.0
BURST_S = 1.0
BURST_ACCEL = 12000.0
#: And the burn after it: how long, how much torque current, and what it
#: is pushing against. The load is what makes the power - at half the
#: no-load speed the back-EMF is real volts, and volts times amps is the
#: only thing on this page that reaches the kW bar.
#: How often the sequence takes the machine out to its envelope on its
#: own. Long against the two loops - they have their own periods of 20
#: and 40 s - so a burst reads as an event rather than as another cycle.
BURST_EVERY_S = 45.0
BURST_HOLD_S = 3.0
BURST_HOLD_A = 20.0
#: Sized so the burn SETTLES at half the no-load speed rather than
#: being aimed at it: `torque x fade = b w + load`, and at half speed the
#: link has taken half the back-EMF so `fade` is a half too. At 0.8 N.m
#: it sat at 778 rpm of a 3902 no-load; at this it sits near 1950.
BURST_LOAD_NM = 0.42

LOAD_PEAK_A = 30.0
LOAD_PERIOD_S = 40.0
LOAD_GRAIN = 0.2
#: How wide the legend's bar is, in cells, and what it is drawn
#: with - the full braille cell, so the legend is made of the same
#: ink as the picture it is a key to.
BAR_CELLS = 12
#: Cells of drag per box scrolled. About a box's own height, so the
#: column moves at the hand's speed rather than flying.
DRAG_ROWS = 6.0
#: The face of the winding thermometer at the foot of the drawing. A
#: SCALE, not a limit: nothing on this board says what the magnet wire
#: may take, and this page does not either (invariant 10).
WINDING_SCALE_C = 150.0
#: The face of the power bar beside the board's thermometers: watts, and
#: LOGARITHMIC over its decades.
#:
#: Linear to 2 kW it did not move. Measured over a full cycle of both
#: loops the peak electrical input is 97 W - 4.9 % of the face, under
#: two cells of the bar - because this page runs the machine at ninety
#: rpm so the rotor mark reads as motion, and ninety rpm with thirty
#: amps of d current is an I^2 R number rather than a kilowatt one.
#: Kilowatts want the machine's envelope, 11 371 rpm at the measured
#: link, and that is a different page.
#:
#: So the face keeps its 2 kW and counts decades instead: a watt at the
#: bottom, two kilowatts at the top, and each ten-fold a fixed share of
#: the bar. 97 W lands at 60 % of it. A log face is a choice about
#: legibility and says so - the number beside it in the box is the watts
#: themselves, undistorted.
WATTS_SCALE = 2000.0
WATTS_FLOOR = 1.0
#: Where the headroom gauge stops being green. The scale's own, not the
#: board's - see `headroom_class`.
HEADROOM_AMBER = 0.5
#: What the top gauge is called, over the machine it spans.
#: The two headroom scales, in the order they stand in the gutter.
#:
#: TWO, BECAUSE THERE ARE TWO WAYS TO COOK THIS BENCH. The board's is the
#: worst of ten nodes against ceilings the calibration record gave it -
#: silicon and copper. The motor's is the winding, which the board has no
#: sensor for and no authority over: it is `3 i^2 R` relaxed into a
#: placeholder pair, drawn against this page's own `WINDING_SCALE_C`. One
#: is a margin the board acts on; the other is a margin only the operator
#: can act on, and saying so is why they are named apart.
#: OUTBOARD OF THE BOARD TEMPS, as two more tubes rather than a level
#: across the drawing. A margin is a level against a ceiling and every
#: other level on this page stands up in a gutter; the headrooms were the
#: only ones lying down, which made them read as a scale over the machine
#: rather than as two more things with room left in them.
#:
#: One column each, so the names are a letter each under a shared SOA -
#: `BOARD SOA` is nine characters and a tube is one wide.
HEADROOM_TITLES = ('B', 'M')

#: What the leaders inside the drawing call them. Spelled out there
#: because there is room in the air where there is none over a
#: one-column tube.
HEADROOM_NAMES = ('BOARD SOA', 'MOTOR SOA')
HEADROOM_GROUP = 'SOA'

#: Columns of air between the board's thermometers and the two headroom
#: tubes. THEY ARE NOT THE SAME KIND OF THING: four of them are node
#: temperatures against their own ceilings and two are margins, one of
#: which the board acts on and one it has no authority over. Adjacent,
#: six tubes read as one stack and a reader counts them as six nodes.
#: `machine._bars` skips a None entry, so the gap costs a column and no
#: special case.
HEADROOM_GAP = 1

BAR_GLYPH = chr(0x28FF)
TRACK_GLYPH = chr(0x2812)
#: The scroll affordances. Triangles rather than dots: they are
#: not part of the picture, they are something to click.
UP, DOWN = chr(0x25B2), chr(0x25BC)
#: The degree sign. A bare C beside a number is a coulomb.
DEGREE = chr(0xB0)

#: The thermal nodes a duty cycle can drive into the SOA: the shunt a
#: phase current crosses and the half-bridge above it, per leg. Named in
#: `coaxial.thermal`, ordered here the way a leg is read.
SOA_NODES = ('driver_u', 'phase_u', 'driver_v', 'phase_v',
             'driver_w', 'phase_w')
#: And the rest of the network, which no duty cycle drives: the die, the
#: rails that feed it, the front end, and the laminate everything sits
#: on. Drawn on the other side of the machine because they fail for
#: different reasons and are read for different ones.
#: WHY BOTH FIGURES ARE THEIR GROUP'S HOTTEST, after two goes at it.
#:
#: The bench read SWITCH TEMPS below BOARD TEMPS and took it for a broken
#: model. It is not: idle, every driver and phase node settles at
#: 31.08 C, which is the board node exactly, and a node below the copper
#: cannot happen - `thermal_step` sheds `(t - board) / to_board`, so it
#: takes a negative shed and is pulled back up. The right gutter's
#: hottest is simply the MCU, 0.666 W through a linear LDO, 15 K over the
#: copper, and on an idle stage that IS hotter than a FET carrying
#: nothing.
#:
#: FIRST FIX, WITHDRAWN: report the copper on the right, so the ordering
#: a reader expects held by construction. It bought the ordering by
#: breaking something worse - the caption then disagreed with its own
#: gutter, saying 20.9 C under a stack whose tallest tube was the
#: regulators at 33.7 C. A figure that does not name the tube beside it
#: is worse than a figure that surprises.
#:
#: WHAT ACTUALLY FIXED IT was the tubes, not the caption: they were each
#: a share of their OWN ceiling, so two at the same height were two
#: different temperatures and the two gutters could not be compared at
#: all. `soa_bars` puts them on one temperature scale now. With one
#: ruler the surprise stops being one - the MCU tube is visibly the
#: tallest, the caption names it, and both figures are the tallest tube
#: in their own gutter.

BOARD_NODES = ('mcu', 'regulators', 'afe', 'board')

#: The ends of the thermistor's own colour ramp, degrees C. A BENCH
#: SCALE, not a limit: nothing on this board was given a ceiling for the
#: NTC, and these are the two ends of what a reader would call cold and
#: hot on it.
NTC_COLD_C, NTC_HOT_C = -20.0, 100.0

#: Columns of air between the switch thermometers and the NTC's own,
#: and where the NTC sits in the left gutter. OUTERMOST, so the six
#: estimates stand together and the one measurement stands apart from
#: them - the same reason the margins are outboard of the board's four.
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


#: The instrument column is 40 cells: a label of nine and a value of
#: twenty-four fit beside the frame. Longer rows were cropped mid-word.
def drive_rows(view):
    s = view['state']
    running = s['mode'] != 'off'
    return [
        ('mode', Text.from_ansi(
            tint('RUNNING %s' % s['mode'].upper(), SODIUM)
            if running else tint('STOPPED', ASH))),
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
    """How far the rotor has actually turned, in mechanical degrees.

    INTEGRATED, not read off an angle. A shaft sensor would be better and
    this board has one, but it reads a machine that may not be on the
    shaft yet; the observers report a speed whatever is mounted. At two
    hundred rpm the rotor is turning three times a second and this page
    redraws eight, so no sampled angle could be unwrapped anyway - the
    integral is what stays right across that.

    It drifts exactly as the speed estimate does, which is the honest
    property: a pointer that has wandered off the mark it was tared to
    is an observer that has been wrong, and that is worth seeing.
    """
    import time as _time

    now = _time.monotonic()
    was = view.get('travel_at')
    view['travel_at'] = now
    if was is None:
        return
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    speed = (view.get('chain') or {}).get('omega') or 0.0
    view['travel'] += math.degrees(speed / pairs) * min(0.5, now - was)


def _place(row, name, columns, right_edge=False, until=None):
    """Write `name` over `columns`, centred, clamped to the frame.

    Answers the column it started at, or None if it wrote nothing: a
    row with two readings on it in two different inks has to be cut where
    they actually landed, and the clamps above mean that is not where the
    caller asked for them.

    `until` is the last column it may occupy, for a name that has a
    neighbour outboard of it: `BOARD` is five characters over four
    columns of thermometers and `kW` sits two columns further out, so
    without one they came out as `BOARDkW` with nothing between them.
    """
    if not columns or not name:
        return None
    middle = (min(columns) + max(columns)) / 2.0
    at = (ART_WIDTH - len(name) if right_edge
          else int(round(middle - (len(name) - 1) / 2.0)))
    # A NAME WIDER THAN ITS GROUP LEANS INWARD. `BOARD` is five over four
    # columns of thermometers, and centred it ran one past them and took
    # the `D` off itself when `kW` landed on the same cell. Toward the
    # machine there is always air; outward there is the frame.
    if not right_edge:
        at = min(at, max(columns) + 1 - len(name))
    if until is not None:
        at = min(at, until + 1 - len(name))
    at = max(0, min(ART_WIDTH - len(name), at))
    row[at:at + len(name)] = name
    return at


def reference(view):
    """The NTC, as text: the one MEASURED temperature on this page.

    EVERY OTHER FIGURE HERE IS AN ESTIMATE. The ten node temperatures are
    a lumped network's opinion, the winding is `3 i^2 R` relaxed into a
    placeholder pair, and none of them has a sensor in it. The NTC has
    one - a thermistor beside the middle gate driver - and it is the only
    thing on the drawing a thermocouple could argue with. It sits above
    the headroom scale as the reference the rest is judged against, in
    the ink this page gives what is known rather than modelled.

    `None` is what the board answers with AFE_ON low, because the AFE
    powers the ADC reference and there is no reading at all then
    (invariant 9). It says so rather than drawing a number: a dash cannot
    be mistaken for a cold board.

    AGAINST THE STAND-IN IT IS NOT A MEASUREMENT. `SimulatedThermal`
    computes it with `thermal.expected_ntc` off its own nodes, so
    simulated this line is the model agreeing with itself and proves
    nothing about the model. The page's own SIMULATED banner is what says
    which one you are looking at; there is no second caveat here because
    a field that cried wolf on every simulated run would be ignored on
    the one that mattered.
    """
    seen = (view.get('thermal') or {}).get('ntc')
    if seen is None:
        return 'NTC  unread'
    return 'NTC %.1f %sC' % (seen, DEGREE)


def hottest(view, names):
    """The hottest node of a group: `(celsius, class)`.

    THE HOTTEST, not the mean. A group is as hot as its worst part, and
    an average over six legs hides the one that is cooking behind five
    that are not - the same reason `headroom` takes the worst node.

    Degrees off `state`, colour off `budget`: the tubes below are
    fractions of each node's own ceiling and this is a temperature, so
    the tallest tube and the hottest node can be different nodes. They
    are two questions - how close, and how hot - and the board answers
    both because neither can be worked out from the other without the
    limit, which lives in the calibration record.
    """
    nodes = (view.get('thermal') or {}).get('nodes') or {}
    budget = view.get('budget') or {}
    seen = [name for name in names if nodes.get(name) is not None]
    if not seen:
        return None, machine.TRACK
    at = max(seen, key=lambda name: nodes[name])
    return nodes[at], soa_class((budget.get('used') or {}).get(at, 0.0),
                                bool(budget.get('tripped')))


def _tinted(row, marks):
    """`row`, a list of characters, as a string with `marks` coloured.

    `marks` are `(at, length, ink)`. Two readings on one row in two inks
    cannot be tinted as a line - one colour would have to lie about one
    of them - so the row is cut where each actually landed.
    """
    out, cut = [], 0
    for at, length, ink in sorted(marks):
        out.append(''.join(row[cut:at]))
        out.append(tint(''.join(row[at:at + length]), ink))
        cut = at + length
    out.append(''.join(row[cut:]))
    return ''.join(out)


def _legend(row, text, ink, column, centred):
    """One legend: a name with its value, an arrowhead over its own
    column, and the row it was written on.

    `centred` puts the words over the machine rather than against the
    head. A head out in the middle of a gutter has room for them there,
    and two labels leaning toward their own sides sat at different depths
    and read as ragged.
    """
    return (row, text, ink, column, centred)


def _legend_rows(view, left, right):
    """The caption rows: four legends and the NTC, in dots and text.

    EVERY GROUP IS NAMED THE SAME WAY NOW. It was two rows of bare names
    over the gutters with the readings on a third, which put `SWITCH` and
    `BOARD` hard against the frame and made a reader carry the name down
    to the tubes themselves. A name, its value, an arrowhead and a line
    falling to what it names says the whole thing in one row and leans
    the words inboard where there is room for them.

    The lines run unbroken: a glyph in every row below their own, down to
    the tubes they land on.
    """
    bars = headrooms(view)
    said = []
    first, last = machine.span(ART_WIDTH, ART_ROWS,
                               LEFT_COLUMNS, RIGHT_COLUMNS)
    # THE MEASUREMENT FIRST, at the top, because everything under it is
    # an estimate and a page that opens with a model teaches a bench to
    # trust one. It had a row of its own and no tube to point at; it
    # names the tube that carries it now, like everything else here.
    seen = (view.get('thermal') or {}).get('ntc')
    if len(left) > NTC_AT and seen is not None:
        # ITS OWN TUBE'S COLOUR, like every other legend here: the name
        # and the level it belongs to share an ink, and for this one that
        # is the thermometer ramp rather than a margin's green to red.
        said.append(_legend(0, reference(view),
                            machine.INK[ntc_class(seen)], left[NTC_AT], True))
    # SWITCH SECOND AND BOARD LAST, with the motor's margin between them.
    # NOTHING CROSSES that way: each leader reaches further out than the
    # one above it on its own side, so no line has to pass under another
    # name to get where it is going.
    for group, columns, name, centred in (
            (SOA_NODES, left, 'SWITCH TEMPS', True),):
        peak, cls = hottest(view, group)
        if peak is None or not columns:
            continue
        # THE MIDDLE OF ITS OWN GROUP, not the edge nearest the machine.
        # A line falling on the inner tube said "this one" about a stack
        # of six; falling on the middle of them it says "these".
        seat = columns[len(columns) // 2]
        said.append(_legend(len(said), '%s %.1f %sC' % (name, peak, DEGREE),
                            machine.INK[cls], seat, centred))

    # THE MARGINS LAST, under the NTC and nearest the tubes they name.
    # They were first, at the top of the page, on the argument that a
    # bench looks at them first - and their lines then had to fall the
    # whole depth of the stack past three other legends. Read top down
    # the page now says what the parts are, what the one sensor reads,
    # and then how much room is left, and every leader is short.
    for index in reversed(range(len(HEADROOM_TITLES))):
        if len(right) > HEADROOM_AT:
            share, cls = bars[index]
            said.append(_legend(
                len(said),
                '%s %.0f %%' % (HEADROOM_NAMES[index], 100.0 * share),
                machine.INK[cls], right[HEADROOM_AT + index], True))
    peak, cls = hottest(view, BOARD_NODES)
    if peak is not None and right:
        # ONE TUBE FURTHER IN than the middle of its four. Its run passes
        # under two lines already falling to the margins outboard of it,
        # and reaching the middle it ended a hair from them; a notch
        # shorter and the row has air where the others have ink.
        said.append(_legend(
            len(said), 'BOARD TEMPS %.1f %sC' % (peak, DEGREE),
            machine.INK[cls], right[len(BOARD_NODES) // 2 - 1], True))

    rows = []
    for index in range(CAPTION_ROWS):
        line = [' '] * ART_WIDTH
        marks = []
        # THE LINES ALREADY FALLING pass through before anything is
        # written, and the words are placed clear of them: without that a
        # leader broke at the captions and picked up again inside the
        # drawing, which read as two marks and not one line.
        for row, _text, _ink, column, _in in said:
            if row < index:
                line[column] = DROP
                marks.append((column, 1, machine.LEADER_GREY))
        for row, text, ink, column, centred in said:
            if row != index:
                continue
            # CENTRED OVER THE MACHINE when the head is out in a gutter's
            # middle, hard against the head when it is the outermost
            # tube. The two gutter groups read as one stack that way -
            # leaning each toward its own side put them at different
            # depths and the rows looked ragged.
            # JUSTIFIED TO ITS OWN SIDE, not centred. Every arrowhead
            # on the left sits in one column and every one on the right
            # in another, so the two stacks read as two columns of
            # pointers rather than five names at five depths. Centred,
            # each head landed wherever its own name happened to end.
            if column < first:
                at = first + 2
            else:
                at = last - 2 - len(text) + 1
            at = max(0, min(ART_WIDTH - len(text), at))
            line[at:at + len(text)] = text
            marks.append((at, len(text), ink))
            # THE HEAD AGAINST THE WORDS, the run in dots. An arrowhead
            # parked out over its own column left the name floating in
            # the middle with no thread between them; beside the text it
            # says which way to look, and the braille carries the eye the
            # rest of the way to the tube.
            if column < at:
                head, span = at - 2, list(range(column, at - 2))
                line[head] = AIM_LEFT
            else:
                head = at + len(text) + 1
                span = list(range(head + 1, column + 1))
                line[head] = AIM_RIGHT
            # A COLUMN OF AIR BEFORE ANYTHING ALREADY FALLING. Two runs
            # ending on adjacent tubes met their neighbour's drop and the
            # pair read as one bracket; each stops short of the other's
            # line now, which is what makes them separate pointers.
            span = [step for step in span
                    if line[step] != DROP
                    and not (step + 1 < ART_WIDTH and line[step + 1] == DROP)]
            for step in span:
                line[step] = LEADER
            marks.append((head, 1, machine.LEADER_GREY))
            for step in span:
                marks.append((step, 1, machine.LEADER_GREY))
        rows.append((line, marks))

    return [_tinted(line, marks) for line, marks in rows]


def _foot_line(view):
    """The row under the box: the winding and the link power.

    OUT OF `gutter_caption` BECAUSE THAT ONE GREW PAST WHAT A
    READER CAN HOLD. Three caption rows and a foot are four
    different pieces of furniture; they were one function.
    """
    # THE TWO ALONG THE FOOT, named under them and CARRYING THEIR OWN
    # NUMBERS. The top gauge is titled by the row above it; these have
    # nothing above them but the drawing, so the arrows say which is
    # which - the winding is the upper of the two and the power the
    # lower. The value goes in the label because a level on a scale says
    # how far along it is and never what it is worth, and the box that
    # held the figures is four boxes down the column.
    #
    # EACH LABEL IN ITS OWN BAR'S INK. One grey line named two gauges
    # drawn in two colours, so which word went with which bar was left to
    # the arrows alone. The colour is the faster half of that answer.
    # BOTH POINT UP, because both bars are above this row. It was an up
    # and a down arrow meaning "the upper one" and "the lower one",
    # which is an ordering a reader has to be told; the COLOUR already
    # pairs each name with its own level, and the same head on both says
    # the same thing the four legends above say.
    # THE HEAD INBOARD, so the stroke can leave it and climb. It sat at
    # the outer end with the words between it and the machine, which put
    # the arrow as far from the level as the row allowed.
    head = 'WINDING %.1f %sC %s' % (winding(view), DEGREE, UP)
    tail = '%s %.2f kW' % (UP, watts(view) / 1000.0)
    # A STROKE EACH, LEAVING THE HEAD AND RISING toward the level above
    # it. The dots climb the cell - low pair, middle pair, top pair - so
    # the line reads as one that goes out from the arrow, up, and then
    # levels off along the bar it names. Flat, it pointed along the row
    # and the bar it meant was the one nobody was looking at.
    rise = ''.join(LEADER_RISE)
    fall = ''.join(reversed(LEADER_RISE))
    pad = ART_WIDTH - len(head) - len(tail) - len(rise) - len(fall)
    foot = (tint(head, machine.INK[machine.SOA_WARN])
            + tint(rise, machine.LEADER_GREY)
            + ' ' * max(0, pad)
            + tint(fall, machine.LEADER_GREY)
            + tint(tail, machine.INK[machine.WATTS]))
    return foot


def gutter_caption(view):
    """The caption rows above the drawing, and the one under its foot.

    Every gutter group is a legend now - `_legend_rows` has the shape -
    and the foot keeps its own two, which name levels that lie along the
    bottom of the drawing rather than stand in a gutter.
    """
    left, right = machine.gutters(ART_WIDTH, ART_ROWS,
                                  LEFT_COLUMNS, RIGHT_COLUMNS)
    return _legend_rows(view, left, right) + [_foot_line(view)]


def phase_amps(view):
    """The three phase currents, and what to call full scale.

    ONE DEFINITION, because the drawing and the legend beside it have to
    agree: a tooth at full length and a bar at full width are the same
    current or the picture is lying about itself. The board reports the
    loop's dq means and the angle it holds them at; the phases are that
    rotated back out, which is the same inverse Park and Clarke the
    firmware does on its way to the compares.
    """
    s = view['state']
    theta = s['theta_hat']
    cos, sin = math.cos(theta), math.sin(theta)
    alpha = s['id'] * cos - s['iq'] * sin
    beta = s['id'] * sin + s['iq'] * cos
    root3 = math.sqrt(3.0) / 2.0
    amps = (alpha, -0.5 * alpha + root3 * beta, -0.5 * alpha - root3 * beta)
    # FULL SCALE IS THE VECTOR, NOT THE TRIP. Scaled against `drv_i_max_ma`
    # the bars were one cell of twelve and the teeth all stubs: the clamp
    # is 5 A and this machine turns on a tenth of one, so the picture said
    # 'nothing is happening' about a rotor that was running. The phases
    # are projections of the commanded space vector, so that vector is
    # what a full tooth means - and the three then pulse a third of a turn
    # apart, which is the thing worth seeing.
    return amps, math.hypot(s['id'], s['iq'])


def phase_rows(view):
    """The legend: which colour is which phase, and how hard it is driven.

    The bar is the same ramp the teeth take, so this reads as a key to
    the picture rather than a second instrument - and it pulses with it,
    a third of a turn apart, which is what makes a still frame of a
    three-phase machine make sense.
    """
    from coaxial import machine

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
    """What machine the record says is on the shaft, or that it cannot say.

    THE RECORD IS THE ONLY SOURCE. `motor_pole_pairs` and
    `motor_lambda_uvs` are what `observer.autodetect` and
    `tools/commission.py` write there, and a record without them is a
    board that has not been told what it is driving - a state worth
    naming rather than a row of zeros. Until it has been, every rpm on
    this page is an electrical speed divided by a pole count nobody
    measured, so the chip is amber and says IDENTIFYING.

    The slot count is not in the record and cannot be: it is `--slots`,
    a parameter of the drawing, and the name reads `24N28P` only because
    a bench said the 24.
    """
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
    """Shaft torque from the loop's own dq, newton-metres.

    `1.5 p (lambda iq + (Ld - Lq) id iq)` - the magnet term and the
    reluctance one - out of the calibration record's own constants, so it
    is the same expression the firmware's model and every notebook use
    rather than a fourth copy of it.

    It is what the CURRENT implies, not what a shaft is measuring: this
    board has no torque sensor, and an angle error the observers have not
    noticed shows up here as torque that is not there.
    """
    s, params = view['state'], view['params']
    pairs = params.get('motor_pole_pairs') or 0.0
    lam = params.get('motor_lambda_uvs') or 0.0
    ld = params.get('motor_ld_nh') or 0.0
    lq = params.get('motor_lq_nh') or 0.0
    return 1.5 * pairs * (lam * s['iq'] + (ld - lq) * s['id'] * s['iq'])


def status_rows(view):
    """Two rows, and neither of them is anywhere else on the page.

    It said RUNNING SENSORLESS and the rpm, and DRIVE says the mode two
    boxes down while CHAIN says the speed two boxes up - a status box
    that repeats its neighbours is three places to check for one fact.
    What is left is what nothing else carries: whether the back-EMF
    chain can see the rotor at all, and how far the rotor has gone since
    it was tared. The direction rides the travel row rather than taking
    one of its own - a sign is easy to have backwards and hard to see in
    a number, and `cw` is neither.
    """
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
                'cw' if (o.get('omega') or 0.0) >= 0.0 else 'ccw'))]


def regime(view):
    """The status chip: which commutation is running, and whether it can.

    NOT THE MODE: DRIVE already prints RUNNING SENSORLESS, and this
    carried the same words and the rpm besides. What is left is the one
    thing neither box says - whether the back-EMF chain has a rotor to
    work with, or whether the microstepper is carrying it because
    nothing can see one.

    Three states, on the chain's own `wc` and the loop's own speed - no
    limit of this view's invention, and none of the board's either
    (invariant 10): it says where the estimate is, and what a drive would
    have to do about it is written beside it rather than judged.
    """
    o = view.get('chain') or {}
    speed = abs(o.get('omega') or 0.0)
    corner = o.get('wc') or 0.0
    if not o.get('valid'):
        return Text(' NONE - STEPPING ', style='alarm')
    if corner and speed < FLOOR_MARGIN * corner:
        return Text(' NEAR THE FLOOR ', style='chip.sim')
    return Text(' HOLDING ', style='chip.live')


def no_load_rpm(view):
    """What the link will spin this machine to with nothing on the shaft.

    `vdc / (sqrt(3) lambda)` is the electrical speed at which the
    back-EMF has taken the whole link, and the mechanical one is that
    over the pole pairs. Out of the record and the measured link, so a
    page that says HALF MAX SPEED means half of THIS machine's.
    """
    params = view['params']
    lam = params.get('motor_lambda_uvs') or 0.0
    pairs = max(1.0, params.get('motor_pole_pairs') or 1.0)
    vdc = (view['state'] or {}).get('vdc') or 0.0
    if lam <= 0.0 or vdc <= 0.0:
        return 0.0
    return vdc / (math.sqrt(3.0) * lam) / pairs * 60.0 / math.tau


def heavy_start(rig, view):
    """A start, a burn, and then back to the dutter. Three stages.

    WHAT A START ACTUALLY IS. Every other cycle on this page is gentle
    enough to watch; a machine breaking away from rest is not.

    FIRST, `BURST_S` of everything the clamp allows: the torque current
    goes to `BURST_A` and the rotor is accelerated with all of it. The
    phase nodes climb to about nine tenths of their budget in that
    second - measured - which is the point of it: a burst is bounded by
    HEAT and not by the current limit, and the only thing that says how
    long one may last is the thermal observer.

    THEN `BURST_HOLD_S` at half the machine's no-load speed against a
    load, which is where the WATTS are. A start is amps and barely any
    power; power is amps times volts and the volts are the back-EMF, so
    nothing on this page burns a real number until the rotor is turning.
    Half of `no_load_rpm` because the whole of it is no-load by
    definition - there is no headroom left at the top to push against.

    THEN it lets go, and the loops take the drive back and everything
    cools. What a node does on the way down is half of what the observer
    is for.

    It is deliberately close to the ceiling and deliberately not over it.
    If it does go over, the envelope drops the stage and the page shows
    that instead - the board's decision, not this function's.
    """
    drive = rig.board.drive
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    left = view['burst_until'] - time.time()
    if left > BURST_HOLD_S:
        # Breaking away: everything the clamp allows, at the top of the
        # speed range, and no load in the way of it.
        drive.model_param(load=0.0)
        drive.setpoint(id_ref=0.0, iq_ref=BURST_A, accel=BURST_ACCEL,
                       omega_target=no_load_rpm(view) / 60.0 * math.tau * pairs)
        view['iq'] = BURST_A
        return
    # Burning: half the no-load speed, and a load to make the volts and
    # the amps happen at the same time.
    drive.model_param(load=BURST_LOAD_NM)
    drive.setpoint(id_ref=0.0, iq_ref=BURST_HOLD_A, accel=BURST_ACCEL,
                   omega_target=no_load_rpm(view) / 120.0 * math.tau * pairs)
    view['iq'] = BURST_HOLD_A


def turn_the_handle(rig, view):
    """Whichever of the three is driving this frame, and only one of them.

    The burst outranks the loops while it runs and hands the drive back
    the way it found it - two things writing `iq_ref` a frame apart is
    one of them winning at random, and the load it leaned on would
    otherwise stay on the shaft after it let go.
    """
    if view['state']['mode'] == 'off':
        return
    now = time.time()
    # THE BURST IS PART OF THE SEQUENCE, not only a key. The speed loop
    # dutters between 8 and 90 rpm because that is the range where the
    # sensorless hand-over happens and where a rotor mark reads as
    # motion - and nothing in it ever approaches the machine's no-load
    # speed or puts a real number on the kW bar. Left alone, the page
    # showed the slow half of the machine and none of the fast one.
    #
    # Simulated only, and only while the speed loop is running: it is a
    # demonstration, and on a board a burst is something somebody asks
    # for.
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
    """D current up and back down, continuously, the shape the speed loop has.

    A TRIANGLE AND NOT A STAIRCASE. Treads with ramps between them were
    still a set of edges, and every edge put a corner in the power and a
    kink in each thermometer - a page of steps reads as something
    switching rather than as a machine being worked. Rising and falling
    without a corner anywhere, the watts ramp and the temperatures lag
    them, and the lag is the whole thing a thermal observer has to show.

    Down as well as up, for the same reason the speed loop goes both
    ways: what a node does while it COOLS is half of what the model is
    for, and a cycle that only climbs never shows it.

    D CURRENT, not torque: on a machine this round it makes none, so the
    rotor keeps whatever the speed loop is doing and the only thing that
    changes is what the legs carry. It is also what a bench uses to heat
    a stage on purpose, for the same reason.

    The stage judges nothing about it - `drv_i_max_ma` clamps it like any
    other current, and the peak here is inside the clamp the view wrote
    at preflight.
    """
    now = time.time()
    phase = ((now - view['load_at']) % LOAD_PERIOD_S) / LOAD_PERIOD_S
    ramp = 2.0 * phase if phase < 0.5 else 2.0 * (1.0 - phase)
    view['load_amps'] = LOAD_PEAK_A * ramp
    view['load_rising'] = phase < 0.5
    # Only when it has moved enough to matter: a setpoint is a round trip,
    # and one a frame against a board is the link's whole budget. The
    # grain is well under what a thermometer can show, so nothing of the
    # ramp is lost to it.
    if abs(view['load_amps'] - view['load_written']) >= LOAD_GRAIN:
        view['load_written'] = view['load_amps']
        rig.board.drive.setpoint(id_ref=view['load_amps'])


#: The demo cycle, as fractions of `SWEEP_S`. Four things a drive does,
#: in the order it does them: hold the rotor still against a current,
#: rock it either way, send it at everything the clamp allows, and brake
#: it back to rest. Then again.
#:
#: IT IS THE SEND THAT MAKES THE PAGE MOVE. With only the gentle half of
#: the machine running, the worst thermal node is `regulators` - a fixed
#: housekeeping watt that changes only as the board does - so SOA
#: HEADROOM sat still and the switch thermometers with it. The legs have
#: to actually get hot for either to mean anything, and a send at the
#: clamp is what does that.
#: The brake gets as much of the cycle as the send does: it is fighting
#: the same inertia with the same clamp, and the thermal derate is cutting
#: ITS current too - at 0.82 the rotor was still turning at 1100 rpm when
#: the hold came round, and a position lock cannot catch that.
CYCLE_HOLD, CYCLE_ROCK, CYCLE_SEND = 0.14, 0.46, 0.73
#: What the rock peaks at, and what the hold holds with.
ROCK_RPM = 200.0
HOLD_A = 12.0
#: Above this the brake pulls the whole clamp and below it proportionally
#: less, so the rotor arrives at zero rather than through it. A quarter of
#: the electrical no-load speed: enough of the stop is at full current to
#: be quick, and the last of it is gentle enough to land.
BRAKE_FULL_RAD_S = 700.0



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
    """The demo cycle: hold, rock, send, brake, and round again.

    FOUR THINGS A DRIVE DOES, in the order it does them, rather than one
    triangle in speed. Each of them exercises something different on the
    page and none of them is the same picture:

    HOLD commutates on the commanded angle with the rotor stationary -
    a microstepper holding position against a current. The back-EMF
    chain has nothing to see and says so, which is the honest state at
    zero speed and the one the whole sensorless floor is about.

    ROCK runs the speed loop either way to `ROCK_RPM` and back. Both
    directions, because a sign is the easiest thing in a drive to have
    backwards, and through the floor each way so the hand-over between
    the stepper and the back-EMF observers happens four times a cycle.

    SEND gives it the clamp and lets it run at the envelope. This is the
    only part that makes real heat and real watts, and therefore the
    only part that moves the thermal gauges - which is why it is here
    rather than on a key.

    BRAKE takes it back to rest against the current, so the cycle starts
    from the same place every time and the cooling is visible.
    """
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
        # ONE swing each way, not two: the speed integrator needs about a
        # second to reach 200 rpm and two swings in five gave it 2.8 s a
        # side, so it spent the whole phase chasing a target that had
        # already reversed and never left 25 rpm.
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
    # BRAKE: the same current the other way until it is stopped, then let
    # it be. Coasting would take the rotor's own damping constant, which
    # on this machine is longer than the whole cycle.
    # BRAKE IS CURRENT THE OTHER WAY, eased off as it slows.
    #
    # IT ONLY WORKS BECAUSE THE INERTIA IS REAL. Against the stand-in's
    # placeholder 2e-5 kg m^2 this drove the rotor from +3621 rpm through
    # zero to -3395 in one phase - 2.6 N.m stops that in milliseconds and
    # the page redraws every seventy, so any current that had to reverse
    # inside a frame overshot by whatever it was still applying, and a
    # position lock was the only thing that could stop it. With a rotor
    # that weighs something the deceleration is 328 rad/s^2 and the stop
    # takes about a second: slower than a frame, so a proportional brake
    # lands on zero instead of passing through it. A position lock cannot
    # do this job at all now - the spring is 0.63 N.m against an inertia
    # that needs four seconds of it.
    turning = (view.get('chain') or {}).get('omega') or 0.0
    share = min(1.0, abs(turning) / BRAKE_FULL_RAD_S)
    view['iq'] = -math.copysign(clamp * share, turning) if share > 0.03 else 0.0
    drive.setpoint(id_ref=0.0, iq_ref=view['iq'], omega_target=0.0)


def _toward(view, rpm, clamp):
    """The speed loop's integrator, one frame. Amps toward `rpm`.

    Closed on the speed rather than open on a current: what holds a given
    speed depends on the damping, and this page is meant to work whatever
    machine the record describes.
    """
    now = time.time()
    dt = min(0.5, max(0.0, now - view['sweep_at']))
    view['sweep_at'] = now
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    turning = ((view.get('chain') or {}).get('omega') or 0.0) \
        / pairs * 60.0 / math.tau
    return max(-clamp, min(clamp,
                           view['iq'] + ROCK_GAIN * (rpm - turning) * dt))


def chain_rows(view):
    """The back-EMF chain that runs beside the loop, and what it costs.

    Seven rows, the same shape as OBSERVER: the chain's own answer, how
    far it is from the loop's, which of its two models is carrying it,
    and the flux magnitude - the one quantity on this board that sees
    the magnets, an air gap away from every thermometer.
    """
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
    """The winding's temperature, estimated, degrees C.

    AN ESTIMATE AND SAID TO BE ONE. The board has no sensor in the
    machine - the NTC is on the PCB and the rotor is across an air gap -
    so this is the only temperature on the page that is not observed. It
    is `3 i_rms^2 R` out of the record's own phase resistance, relaxed
    into the winding's thermal pair, and that pair is a PLACEHOLDER the
    motor profile carries (`coaxial.motor`): the shape is right, the
    number is not measured, and a bench with a thermocouple writes over
    it.

    First order like the board's own observer, and integrated here
    because nothing on the wire carries it.
    """
    from coaxial import thermal as _thermal

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
    # THE SAME HASTE THE STAND-IN'S BOARD MODEL TAKES, and only there:
    # this winding's constant is nearly seven minutes, which is right and
    # unwatchable. Against a board the clock is the clock.
    tau = max(1e-3, k * heat) / (SimulatedThermal.HASTE
                                if view['simulated'] else 1.0)
    view['winding'] += (target - view['winding']) * min(1.0, (now - was) / tau)
    return view['winding']


def watts(view):
    """What the stage is putting into the machine, electrical, watts.

    `1.5 (vd id + vq iq)` out of the loop's own dq means - the same
    quantity the board reports and no reconstruction of it. It is the
    INPUT: the shaft gets it less the copper, and this board cannot
    separate the two without a torque sensor it does not have.
    """
    s = view['state']
    return 1.5 * (s['vd'] * s['id'] + s['vq'] * s['iq'])


def watts_bar(view):
    """The power as a fifth bar past the board's four, `(share, class)`.

    Full scale is `WATTS_SCALE`, the stage's own arithmetic - 63 V and
    100 A is 6.3 kW of link, and two is what a machine this size takes
    before anything else on this page runs out first. A SCALE, and the
    board judges nothing by it.
    """
    decades = math.log10(WATTS_SCALE / WATTS_FLOOR)
    share = math.log10(max(WATTS_FLOOR, abs(watts(view))) / WATTS_FLOOR)
    return (min(1.0, share / decades), machine.WATTS)


def headroom(view):
    """What is left of the whole board's thermal budget, 0 to 1.

    THE WORST NODE'S, because a stage is as close to its ceiling as its
    closest part - an average would hide the one leg that is cooking
    behind five that are not.
    """
    worst = (view.get('budget') or {}).get('worst')
    return 1.0 - min(1.0, max(0.0, worst)) if worst is not None else 1.0


#: How fast the SOA gauge pulses while the envelope is acting, hertz.
#: FAST ENOUGH TO CATCH THE CORNER OF AN EYE, slow enough to read the
#: level under it. The level itself never blinks - only its colour - so
#: what the bar says stays readable through the pulse.
#:
#: DOWN FROM 3. At three it read as an emergency; the board throttling
#: is the envelope working, not a fault, and a page that shouts about
#: routine work teaches a bench to stop looking. A slow pulse in a
#: lighter red says the same thing without the alarm.
FLASH_HZ = 1.5


def flashing(view):
    """Whether this frame takes the bright half of the alarm pulse.

    ON THE BOARD'S OWN VERDICT, never a threshold this page invented: it
    pulses while the envelope is ACTING - throttling, or tripped - and
    both are facts the board reports out of limits it was given
    (invariant 10). Being near a limit is not an event; being held back
    because of one is, and that is the thing worth a flash.

    On wall time rather than a frame count, so it pulses at the same rate
    whatever the view's frame rate is doing.
    """
    budget = view.get('budget') or {}
    if not (budget.get('throttling') or budget.get('tripped')):
        return False
    return (time.monotonic() * FLASH_HZ * 2.0) % 2.0 < 1.0


#: The rows inside the drawing the two headroom names sit on, and the
#: line that reaches from each to its own tube.
#:
#: A STAIRCASE, one row apart. Two tubes one column wide cannot carry a
#: name between them - `BOARD SOA` is nine characters - and stacking the
#: names above them left a reader matching two bars to two words by their
#: order. On its own row beside its own bar, with a rule reaching across
#: to touch it, each name says which one it means and carries its value
#: the way the foot gauges carry theirs.
HEADROOM_ROWS = (0, 1)
#: How far a leader's stub falls: the first row the tubes may use.
#: `machine` keeps the bars below whatever was written above them, so
#: this is only how far the stub has to reach to touch one.
LEADER_DROP = len(HEADROOM_ROWS)

#: The arrowhead a margin's name ends on, and the line it starts. The
#: drop continues into the machine's own dots below the captions; this is
#: the part of it that crosses the caption rows, where a row is a string
#: and not a raster.
#: The head sits against the words and says which way to look; the run
#: that reaches out to the column is braille, like everything else the
#: page draws. `DROP` is the same line continuing down a later row.
AIM_LEFT, AIM_RIGHT = chr(0x25C0), chr(0x25B6)
LEADER, DROP = chr(0x2824), chr(0x2847)

#: The foot's stroke, climbing out of its arrowhead: the low pair of
#: dots, then the middle, then the top. Short, because the levels it
#: names lie right above it and run the machine's whole width - it says
#: which way to look and nothing more.
LEADER_RISE = (chr(0x2824), chr(0x2812), chr(0x2809), chr(0x2809))


def ntc_bar(view):
    """The thermistor as a tube, on the same scale as every other.

    THE ONE MEASURED LEVEL, in TRUTH's ink - the colour this page gives
    what is known rather than modelled, the same the rotor's real angle
    takes. It has no ceiling and so no margin colour: a thermistor reads
    a temperature and nothing on this board was given a limit for it.

    Empty when the AFE is off and there is no reading at all, which
    draws an empty tube rather than a cold one.
    """
    seen = (view.get('thermal') or {}).get('ntc')
    if seen is None:
        return []
    ambient = (view.get('thermal') or {}).get('ambient')
    if ambient is None:
        ambient = 20.0
    span = max(1.0, TEMP_SCALE_C - ambient)
    return [(max(0.0, min(1.0, (seen - ambient) / span)), ntc_class(seen))]


def ntc_class(celsius):
    """Which band of the thermometer ramp a reading is in.

    COLD TO HOT, blue at `NTC_COLD_C` and red at `NTC_HOT_C`, because the
    thermistor has no ceiling to be a margin against. Every other level
    on this page is coloured by how close it is to a limit it was given;
    this one is coloured by what it says.
    """
    span = max(1.0, NTC_HOT_C - NTC_COLD_C)
    share = (celsius - NTC_COLD_C) / span
    step = int(share * (len(machine.NTC_RAMP) - 1) + 0.5)
    return machine.NTC_RAMP[max(0, min(len(machine.NTC_RAMP) - 1, step))]


def headrooms(view):
    """The two margins as gutter tubes: the board's, then the motor's.

    THEY STAND UP LIKE EVERYTHING ELSE. A margin is a level against a
    ceiling and every other level on this page is a tube in a gutter;
    these were the only ones lying across the drawing, which read as a
    scale over the machine rather than as two more things with room left.

    The board's still pulses while the envelope is acting - that is the
    board doing something, and the only alarm on the page. The motor's
    does not: nothing acts on it, and a flashing bar nobody can obey is
    noise.
    """
    board = headroom(view)
    motor = motor_headroom(view)
    # THE LEVEL IS WHAT IS SPENT, not what is left. Drawn as the margin
    # the tube emptied as things got hot, which is backwards for a
    # thermometer standing beside five that fill: everything else on this
    # page rises toward its limit and these fell away from theirs. The
    # COLOUR still comes from the margin, so a full tube is a red one.
    return [(1.0 - board, machine.SOA_FLASH if flashing(view)
             else headroom_class(board)),
            (1.0 - motor, headroom_class(motor))]


def motor_headroom(view):
    """What is left of the winding's scale, 0 to 1.

    THE OTHER WAY TO COOK A BENCH. The board's headroom is the worst of
    ten nodes against ceilings its calibration record gave it, and the
    board acts on that itself. The winding has no sensor and no ceiling
    the board was given: it is `3 i^2 R` relaxed into a placeholder pair
    (`coaxial.motor`), drawn against `WINDING_SCALE_C`, which is this
    PAGE's scale and not a rating off a motor datasheet - there is no
    motor datasheet in this tree.

    So it is a margin only the operator can act on, and it is named apart
    from the board's for that reason. The board still judges nothing
    here; the page is doing the arithmetic and saying whose it is.
    """
    return motor_headroom_of(winding(view))


def motor_headroom_of(celsius):
    """The same margin from a temperature alone.

    SPLIT OUT SO IT CAN BE CHECKED. `winding` integrates against the
    wall clock off the drive's own state, and a test that had to build
    that just to ask what 85 C is worth would be testing the integrator.
    """
    span = max(1.0, WINDING_SCALE_C - 20.0)
    return max(0.0, min(1.0, 1.0 - (celsius - 20.0) / span))


def headroom_class(left):
    """The headroom gauge's colour: green, then amber, then red.

    A LONG GREEN BAR THAT SHORTENS AND SOURS. Coloured by `soa_class` it
    was green until the board was already throttling and then red - true,
    and useless to watch, because the two thresholds the board acts on
    are the throttle point and the ceiling and there is nothing between
    them.

    So the red boundary is the board's - what is left when the worst node
    reaches `THROTTLE_AT` - and the amber one is THIS SCALE'S, the way a
    fuel gauge has a yellow band the tank knows nothing about. Half the
    budget is not a limit and nothing acts on it; it is where a bar
    starts being worth looking at.
    """
    if left <= 1.0 - THROTTLE_AT:
        return machine.SOA_TRIP
    return machine.SOA_WARN if left <= HEADROOM_AMBER else machine.SOA_OK


def soa_class(share, tripped=False):
    """Which band a node's margin is in - `machine.SOA_CLASS`'s order.

    THE BANDS ARE THE BOARD'S. `used` is the fraction of a node's ceiling
    and the ceiling came from the calibration record; amber is
    `THROTTLE_AT`, the same number `set_limit` writes and the board backs
    off at; red is the ceiling. The margin is reported - the action is
    the board's, and it takes it by dropping MOE (invariant 10).
    """
    if tripped or share >= 1.0:
        return machine.SOA_TRIP
    return machine.SOA_WARN if share >= THROTTLE_AT else machine.SOA_OK


#: The gutters' common temperature scale, degrees C.
#:
#: A DRAWING SCALE, not a limit: the board judges nothing by it
#: (invariant 10) and the colours below carry the real ceilings. The
#: record's own highest is 125, so a full tube is a node at the hottest
#: thing the record allows anything to be.
TEMP_SCALE_C = 125.0


def soa_bars(view, names):
    """`(fraction, class)` per node: HEIGHT IS HEAT, COLOUR IS MARGIN.

    THE TWO GUTTERS HAD DIFFERENT SCALES and it read as one drawing with
    two rulers. Every tube used to be its node's share of its OWN
    ceiling, and the ceilings differ - the copper's is 105 where the
    silicon's is 125 - so two tubes at the same height were two different
    temperatures, under captions in degrees that disagreed with them.

    Split, both questions get answered and neither is asked twice. The
    height is degrees on one scale, so the ten tubes are comparable with
    each other and with the figures above them. The colour is still
    `soa_class` on the node's own margin, so a copper at 100 C goes red
    where a FET at 100 C has not - which is exactly the fact the shared
    scale would otherwise have flattened.

    How close anything is to acting stays the SOA HEADROOM gauge's
    question; it takes the worst of all ten and pulses when the board
    does something about it.
    """
    budget = view.get('budget') or {}
    used = budget.get('used') or {}
    seen = view.get('thermal') or {}
    nodes = seen.get('nodes') or {}
    ambient = seen.get('ambient')
    if ambient is None:
        ambient = 20.0
    span = max(1.0, TEMP_SCALE_C - ambient)
    tripped = bool(budget.get('tripped'))
    out = []
    for name in names:
        if name not in used or nodes.get(name) is None:
            continue
        share = (nodes[name] - ambient) / span
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
    # The rest of the tube. A THINNER GLYPH, not the same one dimmed:
    # dimmed, a captured page shows every bar full, and the colour was
    # doing all the work of saying which part was level and which was
    # room left.
    bar.append(TRACK_GLYPH * (BAR_CELLS - len(bar.plain)),
               style='color(%d)' % machine.INK[machine.TRACK])
    return bar


def thermal_rows(view):
    """The six nodes that carry the current, as bars against their ceilings.

    THE SHUNTS AND THE BRIDGES, because they are what a drive can cook:
    `phase_*` is the sense resistor a hundred amps goes through and
    `driver_*` is the half-bridge above it. A number per node said how hot
    each was and nothing about how close - a temperature cannot say that
    without its limit beside it, which is why the board sends the fraction
    and keeps the degrees on `state()`.

    The rest of the network (mcu, regulators, afe, board) is not drawn
    per node: it cannot be driven into the SOA by a duty cycle, and
    whichever of it is worst arrives on the summary row anyway.
    """
    from coaxial import thermal as _thermal

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
    # THE THROTTLE AND THE BUDGET. `derate` is what the envelope is doing
    # to the current clamp right now - under one and the stage is still
    # driving, which is the whole difference between this and a trip.
    # `soak_j` is what the worst node can still absorb: divide by a power
    # and the answer is seconds at THAT power, not only at this one.
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
    rows.append(('winding', '%9.1f C est, upper foot bar' % winding(view)))
    rows.append(('power', '%9.1f W of %.0f log, lower' % (watts(view),
                                                          WATTS_SCALE)))
    if view['load']:
        rows.append(('load loop', '%9.1f A of %.0f, %s'
                     % (view['load_amps'], LOAD_PEAK_A,
                        'rising' if view['load_rising'] else 'falling')))
    rows.append(('NTC', '%7.1f C' % th['ntc'] if th.get('ntc') is not None
                 else '%7s' % 'unread'))
    if budget:
        left = budget.get('seconds_to_limit')
        rows.append(('worst', '%-11s %3.0f%%%s'
                     % (budget.get('worst_node', '?'),
                        100.0 * budget['worst'],
                        '  %.0f s' % left if left is not None else '')))
    return rows


#: The instrument column's width, `stage.frame_of`'s own. A click is in
#: that column when it lands within this many cells of the right edge -
#: which is how a view with no layout of its own knows where its boxes
#: went.
HUD_WIDTH = 40
#: The rows a box costs beyond its content: its two borders.
BOX_BORDER = 2


def screen_size(view):
    """The terminal's (width, height), or zeros when there is no terminal.

    Off `view['screen']`, the rich Console - NOT off the `console` every
    function here is handed, which is `is_terminal` and has no size.
    """
    screen = view.get('screen')
    try:
        return (screen.size.width, screen.size.height) if screen else (0, 0)
    except (AttributeError, OSError):
        return (0, 0)


def paged(view, panels, hud):
    """The instrument column, windowed, with an arrow where it continues.

    SEVEN BOXES DO NOT FIT. The column is the page's right-hand forty
    cells and the boxes fill it from the top; past the bottom of the
    terminal they are simply not drawn, and a reader has no way to know
    a THERMAL box exists at all. This shows as many as the terminal has
    room for and says which way the rest are, on a row that can be
    clicked to get there.

    Piped, nothing is windowed: a captured page is read in order and has
    no bottom to fall off.
    """
    from rich.text import Text as _Text

    # ONLY ON A TERMINAL. A pipe has a nominal size too - 79 by 25 - so
    # sizing alone would page a captured page, and a capture is read in
    # order and has no bottom to fall off.
    room = screen_size(view)[1] - 2 if view.get('terminal') else 0
    if room <= 0:
        view['pages'] = 1
        return [hud(title, rows) for title, rows in panels]

    heights = [len(rows) + BOX_BORDER for _, rows in panels]
    # The last page is packed from the END, so scrolling to the bottom
    # shows a full column rather than one box and a lot of air.
    last, used = len(panels), 0
    while last > 0 and used + heights[last - 1] + 1 <= room:
        used += heights[last - 1]
        last -= 1
    view['scroll'] = max(0, min(view['scroll'], last))

    at = view['scroll']
    out, taken = [], 1 if at else 0              # a row for the up arrow
    while at < len(panels) and taken + heights[at] <= room - 1:
        out.append(hud(*panels[at]))
        taken += heights[at]
        at += 1
    view['pages'] = (view['scroll'], at, len(panels))
    if view['scroll']:
        out.insert(0, _Text(' %s  %s above' % (UP, view['scroll']),
                            style='keys'))
    if at < len(panels):
        out.append(_Text(' %s  %d more' % (DOWN, len(panels) - at),
                         style='keys'))
    return out


def hauled(view, dx, dy):
    """A left-drag on the instrument column, dragged like a page.

    ONLY A DRAG THAT STARTED THERE. The press sets `grip`, so a drag
    beginning over the machine does not scroll the boxes beside it -
    the same rule a scrollbar has, and the reason the press is recorded
    at all.

    A whole box per `DRAG_ROWS` of travel, and the remainder is kept:
    rounding each frame's few cells to zero made a slow drag do nothing
    at all. Dragging DOWN brings the boxes above into view, which is
    which way paper moves under a hand.
    """
    del dx
    if not view.get('grip'):
        return
    view['haul'] += dy
    while abs(view['haul']) >= DRAG_ROWS:
        step = 1 if view['haul'] < 0 else -1
        view['haul'] -= step * -DRAG_ROWS
        at, seen, total = view.get('pages') or (0, 0, 0)
        if step > 0 and seen < total:
            view['scroll'] = at + 1
        elif step < 0 and at:
            view['scroll'] = at - 1
        else:
            view['haul'] = 0.0
            break


def scrolled(view, column, row):
    """One click: the arrows at the top and bottom of the box column.

    The hit test is the page template's own geometry rather than
    anything measured off the frame - `frame_of` puts the header on row
    one, the key bar on the last row, and the boxes in the right-hand
    `HUD_WIDTH` cells of everything between. The arrows are the first
    and last rows of that, which is where they are drawn.
    """
    width, height = screen_size(view)
    if not view.get('terminal') or not width or not height:
        return
    if column <= width - HUD_WIDTH:
        view['grip'] = False
        return
    # The press is also where a drag begins: remembered so `hauled` can
    # tell a page-drag from a drag across the drawing.
    view['grip'] = column > width - HUD_WIDTH
    at, seen, total = view.get('pages') or (0, 0, 0)
    if row == 2 and at:
        view['scroll'] = at - 1
    elif row == height - 1 and seen < total:
        view['scroll'] = at + 1


def compose(rig, origin, console, view):
    from screen import frame_of, hud

    s = view['state']
    # THE MACHINE, NOT A PROTRACTOR. A dial with a needle on it answers
    # 'what is the angle', which is the number in OBSERVER already. What
    # it cannot answer is which tooth is under which magnet, and that is
    # the question an observer gets wrong: a slipped pole is a perfectly
    # respectable angle. Here the can carries its magnets and the teeth
    # carry their phase letters, so a slip steps the band round and leaves
    # the letters where they were.
    pole_pairs = max(1, int(view['params'].get('motor_pole_pairs') or 1))
    # The true rotor is a notch on the can: the gap between it and the
    # magnet band under it IS the observer's error, in the units a magnet
    # works in rather than in electrical degrees.
    truth = view.get('model')
    amps, full = phase_amps(view)
    # THE THERMOMETERS ARE NAMED, on a row of their own above them. A
    # column of braille cannot carry a letter, and a stack of unlabelled
    # tubes beside a motor is a reader guessing which is which. The row
    # costs the machine one of its own, which is cheaper than the guess.
    heads = gutter_caption(view)
    # The names in ash, the readings in their own inks already.
    # ONLY THE FIRST ROW IS ONE COLOUR. The other two carry figures in
    # their own inks - the NTC as the measurement it is, the readings as
    # their nodes' margins - so they arrive already tinted in pieces.
    # EVERY CAPTION ROW ARRIVES INKED, in pieces: a name in its group's
    # colour, a leader in the track's grey, the one measurement in
    # TRUTH's. Nothing here is one colour any more.
    caption = list(heads[:CAPTION_ROWS])
    foot = heads[CAPTION_ROWS]
    turned = math.degrees(s['theta_hat']) / pole_pairs
    # THE CAN AND THE POINTER ARE DIFFERENT QUANTITIES. The can is drawn
    # from the electrical angle over the pole pairs, which is right
    # because the magnet band repeats every pole pair - whichever one it
    # lands in looks the same. The POINTER does not repeat: it is a mark
    # on a real rotor, and drawn the same way it swept 51 degrees of a
    # 14-pole machine and jumped back, over and over. So it rides the
    # travel this view has accumulated instead - the observed speed
    # integrated, which is mechanical revolutions and what a tare is for.
    art = machine.render(turned, view['slots'], 2 * pole_pairs,
                         ART_WIDTH, ART_ROWS,
                         truth_deg=(math.degrees(truth['theta']) / pole_pairs
                                    if truth else None),
                         amps=amps, full=full, aspect=view['aspect'],
                         pointer_deg=view['travel'] - view['tare'],
                         left=(soa_bars(view, SOA_NODES)
                               + [None] * NTC_GAP + ntc_bar(view)),
                         right=(soa_bars(view, BOARD_NODES)
                                + [None] * HEADROOM_GAP + headrooms(view)),
                         top=None,
                         bottom=[(min(1.0, (winding(view) - 20.0)
                                      / (WINDING_SCALE_C - 20.0)),
                                  machine.SOA_WARN),
                                 watts_bar(view)],
                         colour=True)
    art = '\n'.join(caption + [art, foot])
    panels = [('STATUS', status_rows(view)),
              ('DRIVE', drive_rows(view)),
              ('PHASES', phase_rows(view)),
              ('OBSERVER', observer_rows(view)),
              ('CHAIN', chain_rows(view)),
              ('LOOP', loop_rows(view)),
              ('THERMAL', thermal_rows(view))]
    boxes = paged(view, panels, hud)
    # FIXED-WIDTH LABELS. The bar wraps to whatever fits, so a label
    # that changed length reflowed the whole of it and the bottom of the
    # page jumped a line every time the mode changed - SENSORLESS is ten
    # characters and HOLD is four. Padded, the bar is the same length
    # whatever the drive is doing and only the words inside it change.
    keys = [('S', '%-5s' % ('STOP' if s['mode'] != 'off' else 'START')),
            ('M', '%-10s' % (s['mode'].upper() if s['mode'] != 'off'
                             else view['mode'].upper())),
            ('V', view['source'].upper()), ('I', 'INJ'),
            ('+ -', 'IQ'), ('[ ]', 'STEP'), ('O L', 'I/F'), ('R', 'RESET'),
            ('T', 'TARE'),
            # The word stays and the colour changes: a chip that
            # appeared and vanished moved every key after it.
            ('B', Text('START', style='alarm')
             if time.time() < view['burst_until'] else 'START'),
            ('E', Text('SPEED', style='chip.live') if view['spin']
             else 'SPEED'),
            ('W', Text('LOAD', style='chip.live') if view['load']
             else 'LOAD'), (UP + ' ' + DOWN, 'CLICK')]
    if view['switch']:
        keys.append(('A', Text('ARMED', style='chip.live')
                     if s['stage_enabled'] else 'ARM  '))
    keys += [('Q', 'EXIT'), ('ESC', 'MENU')]
    if view.get('said'):
        keys.append(('', view['said']))
    return frame_of(console, origin, 'ROTOR OBSERVER', art, boxes, keys)


def start(rig, view):
    """Enter the chosen mode with the setpoints the view holds. Sensorless
    on the model starts with the estimate 0.3 rad off the rotor, so a lock
    is something to watch rather than assume."""
    d = rig.board.drive
    if view['mode'] == 'sensorless' and view['source'] == 'model':
        d.set_theta(d.model()['theta'] + 0.3)
    d.setpoint(iq_ref=view['iq'], id_ref=view['id'], theta=0.0,
               omega_target=view['omega'] if view['mode'] == 'hold' else 0.0,
               accel=view['accel'], vd=view['vd'], vq=0.0)
    d.mode(view['mode'])
    return 'running %s' % view['mode']


def act(rig, key, view):
    """One keystroke against the board; returns what to say."""
    d = rig.board.drive
    try:
        if key == 's':
            if view['state']['mode'] != 'off':
                d.off()
                return 'stopped'
            return start(rig, view)
        if key == 'm':
            view['mode'] = MODES[(MODES.index(view['mode']) + 1) % len(MODES)]
            if view['state']['mode'] != 'off':
                return start(rig, view)
            return 'mode %s' % view['mode']
        if key == 'v':
            view['source'] = 'adc' if view['source'] == 'model' else 'model'
            d.source(view['source'])
            return 'source %s' % view['source']
        if key == 'i':
            view['inject'] = not view['inject']
            d.set_params(drv_inj_mv=view['v_inj'] if view['inject'] else 0.0)
            return 'injection %s' % ('on' if view['inject'] else 'off')
        if key in '+=-_':
            view['iq'] += view['step'] if key in '+=' else -view['step']
            view['iq'] = max(-view['i_max'], min(view['i_max'], view['iq']))
            d.setpoint(iq_ref=view['iq'])
            return 'iq_ref %+.2f A' % view['iq']
        if key in '[]':
            i = STEPS.index(view['step']) + (1 if key == ']' else -1)
            view['step'] = STEPS[max(0, min(len(STEPS) - 1, i))]
            return 'step %.2f A' % view['step']
        if key in 'ol':
            view['omega'] = max(0.0, min(LIMITS['omega'][1],
                                         view['omega'] + (50.0 if key == 'o' else -50.0)))
            d.setpoint(omega_target=view['omega'] if view['mode'] == 'hold' else 0.0)
            return 'I/f target %.0f rad/s' % view['omega']
        if key == 'b':
            view['burst_at'] = time.time()
            view['burst_until'] = view['burst_at'] + BURST_S + BURST_HOLD_S
            return ('heavy start - %.0f A for %.1f s, then %.0f s at half '
                    'speed' % (BURST_A, BURST_S, BURST_HOLD_S))
        if key == 'e':
            view['spin'] = not view['spin']
            view['spin_at'] = time.time()
            if not view['spin']:
                d.setpoint(omega_target=0.0)
            return ('speed loop running - down through the floor and back'
                    if view['spin'] else 'speed loop off')
        if key == 'w':
            view['load'] = not view['load']
            view['load_at'] = time.time()
            view['load_amps'] = view['load_written'] = 0.0
            if not view['load']:
                d.setpoint(id_ref=0.0)
            return ('load loop running - d current in steps'
                    if view['load'] else 'load loop off')
        if key == 't':
            # TARE: the pointer's zero, not the board's. Nothing is
            # written to the machine and no estimate moves - this is a
            # mark on the can, and where a mark on a can goes is a
            # bench's decision. Pressed again anywhere else it moves
            # there, which is what makes it useful for reading travel:
            # zero it at a stop and the pointer counts from the stop.
            view['tare'] = view['travel']
            return 'tared - the pointer reads travel from here'
        if key == 'r':
            d.model_reset()
            d.set_theta(view['theta0'] + 0.3)
            return 'model rotor reset, theta_hat 0.3 rad off it'
        if key == 'a' and view['switch']:
            if view['state']['stage_enabled']:
                rig.gates.disarm()
                return 'stage disarmed'
            rig.gates.arm(bypass_sto=True, ignore_interlock=not view['interlock'])
            return 'STAGE ARMED - the gates switch'
    except RigError as exc:
        return str(exc)
    return ''


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--port', default='COM4')
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--frames', type=int, default=0)
    p.add_argument('--hz', type=float, default=8.0)
    p.add_argument('--source', choices=('model', 'adc'), default='model')
    p.add_argument('--motor', help='a profile under motors/, written first')
    p.add_argument('--cell-aspect', type=float, default=machine.CELL_ASPECT,
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
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
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
    # Injection ON from the start: sensorless at standstill has no other
    # innovation, and a page that started without it drew the estimate
    # free-running 71 degrees from the model's rotor. I turns it off.
    d.set_params(drv_inj_mv=args.v_inj,
                 drv_eps_gain_ua_per_rad=eps_gain(params, args.v_inj, ts))
    d.source(args.source)
    say('ok', 'source', '%s%s' % (args.source, ' - the board integrates its own '
                                  'rotor' if args.source == 'model' else ''))
    say('ok', 'trip', '%.1f A clamp, %.1f A trip, rating %.0f'
        % (params['drv_i_max_ma'], params['drv_i_trip_ma'], RATING_A))
    return params


def demo_stage(rig, origin):
    """Give the stand-in a bridge to switch. Simulated only.

    Everything on this page worth watching is downstream of current in
    the legs, and there is no current without MOE: the thermal observer
    saw a stage that never switched, so nothing warmed, SOA HEADROOM sat
    at its ceiling and the switch thermometers at ambient.

    On a board this is `--switch` and a key press, because arming one is
    arming a power stage. A stand-in has no stage, and a page that cannot
    show the envelope working is not worth opening.
    """
    if origin.real:
        return
    rig.board.gate_drivers.bypass_break(True)
    rig.board.gate_drivers.enable()


def demo_defaults(args, origin):
    """What the stand-in comes up doing, and the iq step to walk it.

    Only the stand-in: on a board the view opens onto whatever the
    drive is already doing, and starting one is the operator's call.
    """
    # A STAND-IN THAT SITS STILL SHOWS NOTHING. On a real board the view
    # opens onto whatever the drive is doing and starting it is the
    # operator's call - it is a power stage. The stand-in has no stage and
    # no rotor until something asks for torque, so every panel reads zero
    # and the dial does not move: the observers have no back-EMF to work
    # with, the chain is `no back-EMF`, and the page looks broken rather
    # than idle. Simulated, it therefore comes up turning, on the model,
    # with a torque current the caller can still override.
    if not origin.real:
        args.start = True
        # A rotor moving at all wants more frames than a page of numbers
        # does. Only when the caller did not ask for a rate themselves.
        if args.hz == 8.0:
            args.hz = 12.0
        if args.source == 'model':
            # THE SPEED WORTH WATCHING IS THE SLOW ONE. The stand-in's own
            # damping is 1e-5, which puts 0.08 A at 3900 rpm - past the
            # hand-over, past the machine's envelope, and past anything a
            # first turn of a real rotor will do. The profile's 5e-4 puts
            # 0.1 A at 100 rpm instead, so `+` and `-` walk the range that
            # decides whether this drive works at all: 27 rpm, where 20
            # rad/s electrical is the leak's corner and the back-EMF
            # observers stop, up to a hundred.
            if args.b is None:
                args.b = 5e-4
            # AND SOMETHING TO TURN. The stand-in's placeholder inertia
            # is 2e-5 kg m^2, which is not even the bare rotor: a 63100
            # can is a steel shell 63 mm across with magnets in it, about
            # 0.64 kg at an effective 29 mm, so 5.4e-4 on its own. At the
            # placeholder the send reached three thousand rpm inside one
            # frame - the page redraws every seventy milliseconds and the
            # spin-up took less than one, so there was nothing to watch.
            #
            # This is the rotor AND a load on the shaft, which is what a
            # drive on a bench is turning. It is a stand-in's number and
            # says so; a bench with a real machine writes its own through
            # `--j` or a motor profile.
            if args.j is None:
                args.j = 8e-3
            if not args.iq:
                args.iq = 0.06
            # A CLAMP THE LOAD CAN REACH. The record's placeholder is 5 A
            # and this machine turns on a tenth of one, so a load step
            # hit the clamp before it made heat worth watching: three
            # phases at 5 A across 5.3 milliohms is 0.4 W against 1.8 W
            # of housekeeping. Forty amps is a fifth of the stage's
            # rating and puts 25 W in the legs, which the thermal
            # observer answers in seconds rather than in an afternoon.
            if args.i_max is None:
                args.i_max = 50.0
            if args.i_trip is None:
                args.i_trip = 70.0
            view_step = 0.01
    return 0.1


def main(argv=None):
    args = parse_args(argv)
    sane(args)

    from screen import open_rig, run_view, stage
    rig = open_rig('LINKING ROTOR OBSERVER', port=args.port,
                   power_afe=False,
                   simulated_device=bool(args.simulated))
    if rig is None:
        return 1
    origin, board = rig.origin, rig.board
    was_on = board.afe.is_on()
    want_afe = args.afe or args.source == 'adc'
    if want_afe != was_on:
        board.afe.enable() if want_afe else board.afe.disable()
        time.sleep(0.3)
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))
    view_step = demo_defaults(args, origin)
    demo_stage(rig, origin)

    try:
        params = preflight(rig, args)
    except RigError as exc:
        say('fail', 'drive', str(exc))
        rig.close()
        return 1

    view = {'scroll': 0, 'pages': None, 'haul': 0.0, 'grip': False,
            'screen': None, 'terminal': False,
            'source': args.source, 'mode': args.mode, 'iq': args.iq,
            'id': args.id, 'omega': args.omega, 'accel': args.accel,
            'vd': args.vd, 'v_inj': args.v_inj, 'inject': True,
            'inj_periods': int(params.get('drv_inj_periods') or 1),
            'step': view_step, 'slots': args.slots, 'switch': args.switch,
            'aspect': args.cell_aspect,
            'spin': not origin.real, 'spin_at': time.time(),
            'simulated': not origin.real,
            'tare': 0.0, 'sweep_at': time.time(),
            'travel': 0.0, 'travel_at': None, 'leaning': False,
            'winding': 20.0, 'winding_at': None,
            'burst_until': 0.0, 'bursting': False, 'stage': None,
            'burst_at': time.time(),
            'load': False, 'load_at': 0.0, 'load_rising': True,
            'load_amps': 0.0, 'load_written': 0.0,
            'interlock': args.interlock,
            'i_max': params['drv_i_max_ma'], 'theta0': args.theta0 or 0.0,
            'params': params, 'said': '', 'state': board.drive.state(),
            'chain': board.drive.observers(),
            'gate': board.gate_drivers.state(), 'model': None,
            'thermal': None, 'budget': None}
    if args.start:
        view['said'] = act(rig, 's', view)

    board_view = stage()
    console = board_view.is_terminal
    # THE CONSOLE ITSELF, not the boolean. `console` here is
    # `is_terminal` - every view in this tree passes that around under
    # that name and `frame_of` only wants its truth - but the paging has
    # to ASK THE TERMINAL HOW BIG IT IS. Reading `.size` off the boolean
    # raised, the guard swallowed it, `rows_of` answered zero, and the
    # column silently never paged: no arrows, and nothing to drag.
    view['screen'] = board_view
    view['terminal'] = console
    leaving = None
    thermal_at = [0.0]
    # HOW OFTEN THE THERMAL OBSERVER IS READ. Two seconds against a board
    # because it is two round trips on a link that is also carrying the
    # drive; there is no link to a stand-in, and at two seconds its
    # temperatures arrived in visible stairs however smoothly they were
    # integrated - a reading is only as continuous as its refresh.
    thermal_every = 2.0 if origin.real else 0.25

    def draw():
        try:
            view['state'] = board.drive.state()
            view['gate'] = board.gate_drivers.state()
            view['model'] = (board.drive.model()
                             if view['source'] == 'model' else None)
            # ONE REPLY FOR THE DIAL AND THE MARK. `model()` carries the
            # rotor's true angle AND the estimate; `state()` carries the
            # estimate a round trip later. Drawing one from each put 40
            # degrees between the needle and the mark at 2867 rad/s and
            # none of it was the observer's - 15 ms is 43 radians there.
            if view['model']:
                view['state']['theta_hat'] = view['model']['theta_hat']
                view['state']['omega_hat'] = view['model']['omega_hat']
            # The chain is one more round trip, and it is the point of
            # the view: a second answer to the angle, on the same
            # samples, with no shaft sensor behind it.
            view['chain'] = board.drive.observers()
            travel(view)
            turn_the_handle(rig, view)
            if time.time() - thermal_at[0] > thermal_every:
                view['thermal'] = board.thermal.state()
                view['budget'] = board.thermal.budget()
                thermal_at[0] = time.time()
        except RigError:
            pass                    # a missed reply is a missed frame
        return compose(rig, origin, console, view)

    def on_input(typed, _moved):
        for key in typed:
            view['said'] = act(rig, key, view) or view['said']

    try:
        leaving = run_view(board_view, console, 1.0 / max(args.hz, 0.5),
                           args.frames, draw, on_input, mouse=True,
                           on_click=lambda c, r: scrolled(view, c, r),
                           on_drag=lambda dx, dy: hauled(view, dx, dy))
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
                board.afe.enable() if was_on else board.afe.disable()
            done.append(('AFE_ON', 'back the way it was found'))
        except RigError as exc:
            done.append(('putting it back', 'FAILED: %s' % exc))
        rig.close()
        sys.stdout.write('\n')
        closing(done, console, 0)
    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
