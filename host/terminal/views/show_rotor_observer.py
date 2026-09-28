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

The speed loop (iq, the shaft) and the load loop (id) are independent. A demo
board starts the speed loop on its own - the stand-in on its model, the emulated
MCU on its plant through the converters, on commissioning's gains; a bench board
starts neither unasked.

Every drive parameter is a switch, checked before it is written (`sane`):
trip <= the FETs' rating, clamp <= trip, injection <= a fifth of the link's
linear range, the stage armed only with --switch.
"""
import argparse
import math
import sys
import time
import types
from contextlib import suppress

from rich.text import Text

from coaxial.comm.hostclock import clock_of
from coaxial.comm.session import standing
from coaxial.control.commission import Commissioning
from coaxial.draw import cross_section
from coaxial.draw.gauges import TEMP_FLOOR_C, TEMP_SCALE_C, temp_share
from coaxial.errors import RigError
from coaxial.model import thermal as _thermal
from terminal.loader import TO_MENU
from terminal.ui import aspect as _aspect, console as _console, screen as _screen
from terminal.ui.screen import PORT, FPS_CAP, Feed, closing, mode_of, open_rig, run_view, say
from terminal.ui.stage import frame_of, hud, stage
from terminal.views.rotor.keys import LIMITS, MODES, RATING_A, act
from terminal.views.rotor.layout import (BOARD_NODES, BOX, CAPTION_ROWS,
                                         HEADROOM_GAP, LEFT_COLUMNS, NTC_GAP, RIGHT_COLUMNS,
                                         SOA_NODES, fit)
from terminal.views.rotor.legend import foot_furniture, gutter_caption, legend_drops
from terminal.views.rotor.motions import TOP_SHOWN_RPM, turn_the_handle
from terminal.views.rotor.rows import (chain_rows, drive_rows, loop_rows,
                                       observer_rows, phase_amps, phase_rows, pointer_rate,
                                       status_rows, travel)
from terminal.views.rotor.thermal import (headrooms, ntc_bar, policy_margin,
                                          soa_bars, thermal_rows, watts_bar, winding)

_screen.CHATTER = False     # the boot bar replaced the scroll


#: The thermistor's colour ramp, C: the page's scale (`coaxial.draw.gauges`, one
#: for every thermometer and the winding), not a limit (invariant 10).
NTC_COLD_C, NTC_HOT_C = TEMP_FLOOR_C, TEMP_SCALE_C

#: The bead's longest drawn step a frame, degrees: 2.5 turns a second at FPS_CAP, eight frames
#: a turn - a direction the eye follows where the true step is a turn and more.
BEAD_STEP_DEG = 45.0

#: Under this a frame the bead is the rotor, phase and speed (30 rpm at FPS_CAP); above it,
#: in proportion to the rotor's speed, BEAD_STEP_DEG a frame at the demo's top - so spinning
#: up, coasting and braking read as the bead speeding and slowing. Pinned at BEAD_STEP_DEG,
#: 589 and 2 965 rpm drew the same; on a log of the speed, a coast from 1 500 to 1 100 rpm
#: slowed the bead 5 % (2026-09-28).
SHOW_DEG = 9.0

#: How soon the bead closes on the true phase where it can show it, s.
LOCK_S = 0.2


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
    for name, key in (('kp', 'drv_kp'), ('ki', 'drv_ki'),
                      ('l1', 'drv_l1'), ('l2', 'drv_l2'),
                      ('i_max', 'drv_i_max'), ('i_trip', 'drv_i_trip'),
                      ('w_lo', 'drv_w_lo'), ('w_hi', 'drv_w_hi'),
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
    ld, lq = params['motor_ld'], params['motor_lq']
    return v_inj * ts * (lq - ld) / (ld * lq) if ld > 0.0 and lq > 0.0 else 0.0


def rearm_after_trip(rig, origin, view):
    """The stand-in's operator: re-arm the stage after a thermal trip."""
    if origin.real or not view.get('spin'):
        return
    budget = view.get('budget') or {}
    if not budget.get('trips') or budget.get('tripped'):
        return
    if view['state'].get('stage_enabled') or rig.gates.is_on():
        return
    rig.gates.on(bypass_sto=True, ignore_interlock=True)
    view['rearms'] = view.get('rearms', 0) + 1
    view['said'] = ('re-armed after thermal trip %d - the stand-in\'s '
                    'operator; the envelope is %d %% of the span'
                    % (budget['trips'],
                       int(round(100.0 * policy_margin(view)))))


def travelled(view, rate, now):
    """`travel` carried from its last sample to `now` at `rate`: the feed samples it, and a
    draw between two samples read the last one again."""
    was, at = view.get('travel_mark', (view['travel'], now))
    return was + rate * min(0.5, max(0.0, now - at))


def shown_rate(rate, top):
    """The bead's rate for the rotor's `rate`, deg/s: the rotor's under SHOW_DEG a frame, then in
    proportion to it up to BEAD_STEP_DEG a frame at `top`, deg/s, and on its log past that - so
    a rotor past the demo's top still turns the bead faster, never pinned."""
    show, most = SHOW_DEG * FPS_CAP, BEAD_STEP_DEG * FPS_CAP
    a = abs(rate)
    if a <= show:
        return rate
    top = max(top, 2.0 * show)
    if a <= top:
        return math.copysign(show + (most - show) * (a - show) / (top - show), rate)
    return math.copysign(most * (1.0 + math.log(a / top)), rate)


def bead_at(bead, true, rate, top, dt):
    """The bead's next angle: on at `shown_rate`, and where the rotor is slow enough for a frame
    to show its phase, drawn onto `true` within LOCK_S. A regime judged off the step `travel`
    made since the last draw took a fast can's phase the short way whenever a draw fell between
    two feed samples: backwards,
    47 times in 420 frames before the emulator and 16 in 200 after (2026-09-28)."""
    if bead is None:
        return true
    if abs(rate) <= SHOW_DEG * FPS_CAP:
        # Slow: onto `true`, the travel off the angle estimate, within LOCK_S - not the speed
        # estimate's step on top, whose sign near standstill is the observer's noise.
        return bead + ((true - bead + 180.0) % 360.0 - 180.0) * min(1.0, dt / LOCK_S)
    return bead + shown_rate(rate, top) * dt


def compose(rig, origin, console, view):

    s = view['state']
    # The dial is the motor (poles from the record), not a protractor.
    pole_pairs = max(1, int(view['params'].get('motor_pole_pairs') or 1))
    # The true rotor is a notch on the can: its gap to the magnet band is the
    # observer's error, in mechanical units.
    amps, full = phase_amps(view)
    # The thermometers are named on a row of their own above them.
    heads = gutter_caption(view)
    # The names in ash, the readings in their own inks already.
    caption = list(heads[:CAPTION_ROWS])
    foot = list(heads[CAPTION_ROWS:])          # FOOT_ROWS of them
    turned = math.degrees(s['theta_hat']) / pole_pairs
    # A can turning past half a magnet pitch a frame strobes at FPS_CAP - stands, or runs
    # backwards, as a wheel on film (43 rpm at 14 poles) - so the magnets blur into a band; the
    # bead keeps the true direction at BEAD_STEP_DEG a frame and takes the true phase, the short
    # way, once the can is slow enough to show it. `travel` itself stays exact.
    a_frame = math.degrees(abs(s['omega_hat'])) / pole_pairs / FPS_CAP
    # With hysteresis: the rotor crosses the threshold once, a low-speed estimate's noise many
    # times - the can flipped between band and magnets 7 times in one ramp (2026-09-28).
    smear = a_frame > (0.7 if view.get('smeared') else 1.0) * 90.0 / pole_pairs
    view['smeared'] = smear
    rate = pointer_rate(view)
    now = time.monotonic()
    dt = min(0.25, max(0.0, now - view.get('bead_at', now)))
    view['bead_at'] = now
    top = 6.0 * TOP_SHOWN_RPM
    view['bead'] = bead_at(view.get('bead'), travelled(view, rate, now), rate, top, dt)
    bead = view['bead']
    # The can and the pointer are different quantities.
    art = cross_section.render(turned, view['slots'], 2 * pole_pairs,
                         BOX.width, BOX.rows,
                         # The sensor's own stroke is not drawn.
                         truth_deg=None,
                         amps=amps, full=full, aspect=view['aspect'],
                         pointer_deg=bead - view['tare'],
                         pointer_rate=shown_rate(rate, top), smear=smear,
                         left=(soa_bars(view, SOA_NODES)
                               + [None] * NTC_GAP + ntc_bar(view)),
                         right=(soa_bars(view, BOARD_NODES)
                                + [None] * HEADROOM_GAP + headrooms(view)),
                         leaders=legend_drops(view, *cross_section.gutters(
                             BOX.width, BOX.rows,
                             LEFT_COLUMNS, RIGHT_COLUMNS))
                         + foot_furniture()[0],
                         rules=foot_furniture()[1],
                         top=None,
                         bottom=[(temp_share(winding(view)),
                                  cross_section.SOA_WARN),
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
    # Fixed-width labels.
    keys = [('S', '%-5s' % ('STOP' if s['mode'] != 'off' else 'START')),
            ('M', '%-10s' % (s['mode'].upper() if s['mode'] != 'off'
                             else view['mode'].upper())),
            ('V', view['source'].upper()), ('I', 'INJ'),
            ('+ -', 'IQ'), ('[ ]', 'STEP'), ('O L', 'I/F'), ('R', 'RESET'),
            ('T', 'TARE'),
            # The word stays and the colour changes: a chip that appeared and
            # vanished moved every key after it.
            ('B', Text('START', style='alarm')
             if view['clock'].now() < view['burst_until'] else 'START'),
            ('E', Text('SPEED', style='chip.live') if view['spin']
             else 'SPEED'),
            ('W', Text('LOAD', style='chip.live') if view['load']
             else 'LOAD'),
            # Who has the mouse.
            ('F', Text('MOUSE', style='chip.live') if _console.holding()
             else 'MOUSE')]
    if view['switch']:
        keys.append(('A', Text('ARMED', style='chip.live')
                     if s['stage_enabled'] else 'ARM  '))
    keys += [('Q', 'EXIT'), ('ESC', 'MENU')]
    if view.get('said'):
        keys.append(('', view['said']))
    return frame_of(console, origin, 'ROTOR OBSERVER', art, boxes, keys)


def aspect_of(args):
    """What makes the can round on this terminal."""
    return _aspect.aspect_of(args.cell_aspect)


def parse_args(argv):
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('--port', default=PORT)
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--frames', type=int, default=0)
    p.add_argument('--hz', type=float, default=FPS_CAP)
    # A terminal size to fit to: a piped run draws as that terminal would
    # (`tools/render/ansi2png.py` rasters it).
    p.add_argument('--width', type=int, default=None)
    p.add_argument('--height', type=int, default=None)
    p.add_argument('--source', choices=('model', 'adc'), default=None,
                   help="the drive's samples: the model, or the converters. Default: the "
                        'converters on an emulated board - its plant is its motor - the '
                        'model elsewhere')
    p.add_argument('--motor', help='a profile, a file or a name in coaxial/profiles/, written first')
    p.add_argument('--cell-aspect', type=float, default=None,
                   help='what makes the can round on this terminal. The '
                        'geometry is exactly round at 2.0 - measured, 25.16 '
                        'cell-widths each way - so an ellipse is the font '
                        'being taller than one by two. The steps are coarse: '
                        'a row is four dots, about 8 %% of the diameter, so '
                        'the value that rounds it is not always the font\'s '
                        'true ratio. Try 2.4 if it looks stretched in Y.')
    p.add_argument('--slots', type=int, default=24,
                   help='stator teeth to draw. Not a measurement: the slot '
                        'count is not in the calibration record and cannot '
                        'be inferred from the pole count, which is. The '
                        'poles are drawn from the record either way.')
    p.add_argument('--mode', choices=MODES, default='sensorless')
    p.add_argument('--switch', action='store_true',
                   help='let A arm the stage: gates.on(bypass_sto=True)')
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


def preflight(rig, args, demo=False):
    """Profile, parameters, source - every one checked, then written. A demo board's gains are
    commissioning's arithmetic on its record's motor: the record's placeholders (l1 0.1, l2 100)
    lost the rotor from standstill in 27 ms on the drive core."""
    d = rig.board.drive
    d.off()
    if args.motor:
        got = d.configure(profile=args.motor)['profile']
        say('ok', 'motor', got['name'])
    drive_params, model_params = parameters(args)
    if drive_params:
        d.configure(**drive_params)
    if model_params:
        d.model.configure(**model_params)
    if demo:
        written = Commissioning(rig).gains()['written']
        args.v_inj = written.get('drv_inj_volts', args.v_inj)
        if drive_params:
            d.configure(**drive_params)
    else:
        # Injection on from the start: at standstill it is the only innovation
        # (without it the estimate ran 71 degrees from the model's rotor).
        d.configure(drv_inj_volts=args.v_inj,
                    drv_eps_gain=eps_gain(d.params(), args.v_inj, d.state()['ts'] or 20e-6))
    params = d.params()
    d.configure(source=args.source)
    say('ok', 'source', '%s%s' % (args.source, ' - the board integrates its own '
                                  'rotor' if args.source == 'model' else ''))
    say('ok', 'trip', '%.1f A clamp, %.1f A trip, rating %.0f'
        % (params['drv_i_max'], params['drv_i_trip'], RATING_A))
    return params


def demo_stage(rig, origin):
    """Give a demo board a bridge to switch: the stand-in's, the emulated MCU's."""
    if not _screen.demo(origin):
        return
    rig.board.gate_drivers.configure(bypass_break=True)
    rig.board.gate_drivers.on()


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
    """The demo's motor on the model: the damping, the inertia and the torque current
    above, where the caller left them."""
    if args.b is None:
        args.b = DEMO_B
    if args.j is None:
        args.j = DEMO_J
    if not args.iq:
        args.iq = DEMO_IQ


def demo_defaults(args, origin):
    """What a demo board comes up doing, and the iq step to walk it: the stand-in's and the
    emulated MCU's model carry the demo's rotor, not the record's 2e-5 placeholder."""
    if not _screen.demo(origin):
        return BOARD_STEP
    args.start = True
    if args.i_max is None:
        args.i_max = DEMO_I_MAX
    if args.i_trip is None:
        args.i_trip = DEMO_I_TRIP
    if args.source != 'model':
        return BOARD_STEP
    _model_defaults(args)
    return DEMO_STEP


def _link(args):
    """(rig, params, was_on, step) with the front end where the source needs it;
    None when the board would not open."""
    rig = open_rig('LINKING ROTOR OBSERVER', port=args.port,
                   power_afe=False,
                   execution_mode=mode_of(args))
    if rig is None:
        return None
    origin, board = rig.origin, rig.board
    if not origin.real:
        # The stand-in tours its rooms as the identification earns them: TH OBS
        # walks UNCR, CONV, STABLE on the foot (bench 2026-09-06).
        rig.thermal.situation('tour')
    if args.source is None:
        # An emulated board's plant is its motor (board/emu/worlds/bench.json, the demo's
        # flywheel): driven, its currents heat the board and its NTC reads them.
        args.source = 'adc' if origin.real and _screen.demo(origin) else 'model'
    was_on = board.afe.is_on()
    # An emulated or simulated board follows the schematic: its stage's supply is the STO
    # chain's, whose pilot detector runs off AFE_ON.
    want_afe = args.afe or args.source == 'adc' or _screen.demo(origin)
    if want_afe != was_on:
        board.afe.write(want_afe)
        time.sleep(0.3)
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, standing(origin)))
    # The demo's defaults first: `preflight` hands `args` to the model.
    view_step = demo_defaults(args, origin)
    demo_stage(rig, origin)
    try:
        return rig, preflight(rig, args, _screen.demo(origin)), was_on, view_step
    except RigError as exc:
        say('fail', 'drive', str(exc))
        rig.close()
        return None


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

    linked = _link(args)
    if linked is None:
        return 1
    rig, params, was_on, view_step = linked
    origin, board = rig.origin, rig.board

    # Measured once, at start-up: the cell's shape is the terminal's and cannot
    # change under a running view.
    aspect, aspect_how = aspect_of(args)
    fit(aspect)
    clock = clock_of(rig)
    view = {'source': args.source, 'mode': args.mode, 'iq': args.iq,
            'id': args.id, 'omega': args.omega, 'accel': args.accel,
            'vd': args.vd, 'v_inj': args.v_inj, 'inject': True,
            'inj_periods': int(params.get('drv_inj_periods') or 1),
            'step': view_step, 'slots': args.slots, 'switch': args.switch,
            'aspect': aspect, 'aspect_how': aspect_how,
            'spin': _screen.demo(origin), 'spin_at': clock.now(),
            'simulated': not origin.real, 'demo': _screen.demo(origin), 'clock': clock,
            'tare': 0.0, 'sweep_at': clock.now(),
            # The rotor the demo's speed loop is designed on: the model's where the page
            # set one, else the demo's flywheel, which an emulated board's world carries.
            'j': args.j or DEMO_J, 'b': args.b or DEMO_B,
            'travel': 0.0, 'travel_at': None, 'leaning': False,
            'winding': _thermal.AMBIENT, 'winding_at': None,
            'burst_until': 0.0, 'bursting': False, 'stage': None,
            'burst_at': clock.now(),
            'load': False, 'load_at': 0.0, 'load_rising': True,
            'load_amps': 0.0, 'load_written': 0.0,
            'interlock': args.interlock,
            'i_max': params['drv_i_max'], 'theta0': args.theta0 or 0.0,
            'params': params, 'said': '', 'state': board.drive.state(),
            'chain': board.drive.observers.read(),
            'gate': board.gate_drivers.state(), 'model': None,
            'thermal': None, 'budget': None, 'ident': None}
    # The emulated MCU's drive starts off, the stand-in's running: a demo starts it.
    if args.start or (view['demo'] and origin.real):
        view['said'] = act(rig, 's', view)

    board_view = _console_for(args)
    console = board_view.is_terminal
    # `console` is the flag the keys and the closing want; `compose` gets the
    # console itself.
    leaving = None
    thermal_at = [0.0]
    # Thermal observer read period, s: the stand-in answers out of memory, a link
    # does not. The three reads cost 18 ms of a 50 ms tick on the emulator, where
    # the four every tick already cost 23 ms - asked four times a second they blew
    # the frame and the page froze and raced (2026-09-28).
    thermal_every = 0.25 if standing(origin) == 'simulated' else 2.0

    def sample():
        """The board's side of a frame, on the feed's thread: an emulated board's link is
        several times slower than a real one's, and the frame need not wait for it. The replies
        land in the view together: a frame drawn between them paired the new estimate with the
        last model's error."""
        with suppress(RigError):
            state = board.drive.state()
            gate = board.gate_drivers.state()
            model = board.drive.model.read() if view['source'] == 'model' else None
            # One reply for the dial and the mark.
            if model:
                state['theta_hat'] = model['theta_hat']
                state['omega_hat'] = model['omega_hat']
            # The chain: a second answer to the angle, no shaft sensor behind it.
            chain = board.drive.observers.read()
            view.update(state=state, gate=gate, model=model, chain=chain)
            travel(view)
            turn_the_handle(rig, view)
            if time.time() - thermal_at[0] > thermal_every:
                view['thermal'] = board.thermal.state()
                view['budget'] = board.thermal.budget()
                view['ident'] = board.thermal.identification()
                thermal_at[0] = time.time()
                rearm_after_trip(rig, origin, view)

    sample()
    feed = Feed(sample, period=1.0 / FPS_CAP).start()

    def draw():
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
        feed.stop()
        done = []
        try:
            board.drive.off()
            done.append(('drive', 'off, the compares released'))
            board.drive.configure(source='adc')
            rig.gates.off()
            done.append(('gate stage', 'disarmed, MOE clear, the break back'))
            board.gate_drivers.configure(sync=False)
            if board.afe.is_on() != was_on:
                board.afe.write(was_on)
            done.append(('AFE_ON', 'back the way it was found'))
        except RigError as exc:
            done.append(('putting it back', 'FAILED: %s' % exc))
        rig.close()
        sys.stdout.write('\n')
        closing(done, console, 0)
    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
