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
    A       arm / disarm the stage - only with --switch
    Q / ESC close / menu

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
from coaxial.errors import RigError                         # noqa: E402
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
ART_WIDTH, ART_HEIGHT = 40, 22

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
SWEEP_LO_RPM, SWEEP_HI_RPM, SWEEP_S = 8.0, 90.0, 20.0
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
#: How wide the legend's bar is, in cells, and what it is drawn
#: with - the full braille cell, so the legend is made of the same
#: ink as the picture it is a key to.
BAR_CELLS = 12
BAR_GLYPH = chr(0x28FF)
#: The scroll affordances. Triangles rather than dots: they are
#: not part of the picture, they are something to click.
UP, DOWN = chr(0x25B2), chr(0x25BC)


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
        Text.from_ansi(tint('RUNNING %s' % s['mode'].upper(), SODIUM)
                       if running else tint('stopped', ASH)),
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
    return [('back-EMF', regime(view)),
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


def sweep(rig, view):
    """Walk the stand-in's speed across the sensorless floor and back.

    A stand-in sitting at one speed shows one regime, and the regime is
    the interesting part: this drives the rotor down until the back-EMF
    observers have nothing left, hands commutation to the microstepper
    there, and takes it back on the way up. HOLD is the trivial
    controller - it commutates on the COMMANDED angle and asks the rotor
    nothing - which is what is left when nothing can see the rotor, and
    it costs the current that an open loop always costs.

    Simulated only. Switching a commutation strategy under a rotor is a
    decision with a power stage behind it, and this is a demonstration.
    """
    now = time.time()
    passes = int((now - view['sweep']) / SWEEP_S)
    phase = ((now - view['sweep']) % SWEEP_S) / SWEEP_S
    ramp = 2.0 * phase if phase < 0.5 else 2.0 * (1.0 - phase)
    # BOTH WAYS ROUND, a pass each. A drive that only ever ran one way is
    # a drive whose sign errors are invisible: the observers carry a
    # signed speed, the leak correction leads one way and lags the other,
    # and the blend is on a magnitude. Reversing every pass is what makes
    # a wrong sign show up as a rotor that will not hold rather than as
    # nothing at all.
    way = 1.0 if passes % 2 == 0 else -1.0
    target = way * (SWEEP_HI_RPM - ramp * (SWEEP_HI_RPM - SWEEP_LO_RPM))
    o = view.get('chain') or {}
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    rpm = abs(o.get('omega') or 0.0) / pairs * 60.0 / math.tau
    drive = rig.board.drive
    if not o.get('valid'):
        # The stepper has no rotor to follow, so the sweep tells it which
        # way to walk rather than reading it back off an observer that
        # has nothing.
        rpm = math.copysign(rpm, target)
    want = 'hold' if not o.get('valid') else 'sensorless'
    if view['state']['mode'] != want:
        # The hand-over both ways: the stepper is given the speed the
        # chain last saw, so it picks the rotor up where it was left
        # rather than from a standstill it is not at.
        drive.setpoint(omega_target=target / 60.0 * math.tau * pairs)
        drive.mode(want)
    turning = math.copysign(rpm, o.get('omega') or target)
    dt = min(0.5, max(0.0, now - view['sweep_at']))
    view['sweep_at'] = now
    view['iq'] = max(-0.6, min(0.6, view['iq']
                               + SWEEP_GAIN * (target - turning) * dt))
    drive.setpoint(iq_ref=view['iq'],
                   omega_target=target / 60.0 * math.tau * pairs)


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


def thermal_rows(view):
    th, budget = view.get('thermal'), view.get('budget')
    if not th:
        return ['  (not read yet)']
    rows = [('NTC', '%7.1f C' % th['ntc'] if th.get('ntc') is not None
             else '%7s' % 'unread')]
    if budget:
        rows.append(('budget', '%7d %%   worst %s'
                     % (round(100.0 * budget['worst'] / 255.0),
                        budget.get('worst_node', '?'))))
    return rows


#: The instrument column's width, `stage.frame_of`'s own. A click is in
#: that column when it lands within this many cells of the right edge -
#: which is how a view with no layout of its own knows where its boxes
#: went.
HUD_WIDTH = 40
#: The rows a box costs beyond its content: its two borders.
BOX_BORDER = 2


def rows_of(console):
    """The terminal's height, or 0 where there is no terminal."""
    try:
        return console.size.height if console else 0
    except (AttributeError, OSError):
        return 0


def paged(view, panels, console, hud):
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

    room = rows_of(console) - 2                  # the header and the key bar
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


def scrolled(view, console, column, row):
    """One click: the arrows at the top and bottom of the box column.

    The hit test is the page template's own geometry rather than
    anything measured off the frame - `frame_of` puts the header on row
    one, the key bar on the last row, and the boxes in the right-hand
    `HUD_WIDTH` cells of everything between. The arrows are the first
    and last rows of that, which is where they are drawn.
    """
    width = 0
    try:
        width = console.size.width if console else 0
    except (AttributeError, OSError):
        return
    height = rows_of(console)
    if not width or not height or column <= width - HUD_WIDTH:
        return
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
                         ART_WIDTH, ART_HEIGHT,
                         truth_deg=(math.degrees(truth['theta']) / pole_pairs
                                    if truth else None),
                         amps=amps, full=full,
                         pointer_deg=view['travel'] - view['tare'],
                         colour=True)
    panels = [('STATUS', status_rows(view)),
              ('DRIVE', drive_rows(view)),
              ('PHASES', phase_rows(view)),
              ('OBSERVER', observer_rows(view)),
              ('CHAIN', chain_rows(view)),
              ('LOOP', loop_rows(view)),
              ('THERMAL', thermal_rows(view))]
    boxes = paged(view, panels, console, hud)
    keys = [('S', 'STOP' if s['mode'] != 'off' else 'START'),
            ('M', s['mode'].upper() if s['mode'] != 'off' else view['mode'].upper()),
            ('V', view['source'].upper()), ('I', 'INJ'),
            ('+ -', 'IQ'), ('[ ]', 'STEP'), ('O L', 'I/F'), ('R', 'RESET'),
            ('T', 'TARE'), (UP + ' ' + DOWN, 'CLICK')]
    if view['switch']:
        keys.append(('A', Text('ARMED', style='chip.live')
                     if s['stage_enabled'] else 'ARM'))
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
            if not args.iq:
                args.iq = 0.06
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
    try:
        params = preflight(rig, args)
    except RigError as exc:
        say('fail', 'drive', str(exc))
        rig.close()
        return 1

    view = {'scroll': 0, 'pages': None,
            'source': args.source, 'mode': args.mode, 'iq': args.iq,
            'id': args.id, 'omega': args.omega, 'accel': args.accel,
            'vd': args.vd, 'v_inj': args.v_inj, 'inject': True,
            'inj_periods': int(params.get('drv_inj_periods') or 1),
            'step': view_step, 'slots': args.slots, 'switch': args.switch,
            'sweep': time.time() if not origin.real else None,
            'tare': 0.0, 'sweep_at': time.time(),
            'travel': 0.0, 'travel_at': None, 'interlock': args.interlock,
            'i_max': params['drv_i_max_ma'], 'theta0': args.theta0 or 0.0,
            'params': params, 'said': '', 'state': board.drive.state(),
            'chain': board.drive.observers(),
            'gate': board.gate_drivers.state(), 'model': None,
            'thermal': None, 'budget': None}
    if args.start:
        view['said'] = act(rig, 's', view)

    board_view = stage()
    console = board_view.is_terminal
    leaving = None
    thermal_at = [0.0]

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
            if view['sweep'] and view['state']['mode'] != 'off':
                sweep(rig, view)
            if time.time() - thermal_at[0] > 2.0:   # the thermal observer, every 2 s
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
                           on_click=lambda c, r: scrolled(view, console, c, r))
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
