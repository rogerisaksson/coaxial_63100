#!/usr/bin/env python3
"""The rotor observer: the drive watched live, on the model or the converters.

    python tools/show_observer.py --simulated
    python tools/show_observer.py --port COM4 --source model --iq 0.5
    python tools/show_observer.py --port COM4 --source model --switch

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

from coaxial import Coaxial63100, dial                      # noqa: E402
from coaxial.errors import RigError                         # noqa: E402
from coaxial.raster import cell                             # noqa: E402
from screen import (ASH, SODIUM, TO_MENU,  # noqa: E402
                    Keys, closing, say, tint)

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
GLYPH_TRUE = '@'

#: The dial, drawn smaller than the shaft view's: four instrument boxes
#: sit beside it and the face is a pointer, not a protractor to read.
DIAL_RADIUS = 9.0
DIAL_WIDTH, DIAL_HEIGHT = 40, 14


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
    for name in ('vdc', 'load', 'noise', 'theta0', 'sat'):
        if getattr(args, name) is not None:
            model[name] = getattr(args, name)
    return drive, model


def eps_gain(params, v_inj, ts):
    """Demodulated amps per radian, from the record's Ld and Lq."""
    ld, lq = params['motor_ld_nh'], params['motor_lq_nh']
    return v_inj * ts * (lq - ld) / (ld * lq) if ld > 0.0 and lq > 0.0 else 0.0


def mark(art, degrees, glyph, radius=DIAL_RADIUS, width=DIAL_WIDTH):
    """A glyph on the dial's rim at `degrees`: the model's own rotor."""
    lines = art.split('\n')
    cx = (width - 1) / 2.0
    cy = (radius + 2.0) * dial.ROW_ASPECT + 1.5
    phi = math.radians(degrees)
    col = cell(cx + (radius + 1.2) * math.cos(phi))
    row = cell(cy - (radius + 1.2) * math.sin(phi) * dial.ROW_ASPECT)
    if 0 <= row < len(lines):
        line = lines[row].ljust(width)
        if 0 <= col < width:
            lines[row] = line[:col] + glyph + line[col + 1:]
    return '\n'.join(lines)


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


def compose(rig, origin, console, view):
    from screen import frame_of, hud

    s = view['state']
    art = dial.render(math.degrees(s['theta_hat']), DIAL_WIDTH, DIAL_HEIGHT,
                      radius=DIAL_RADIUS)
    if view.get('model'):
        art = mark(art, math.degrees(view['model']['theta']), GLYPH_TRUE)
    art = dial.colourise(art)
    boxes = [hud('DRIVE', drive_rows(view)),
             hud('OBSERVER', observer_rows(view)),
             hud('LOOP', loop_rows(view)),
             hud('THERMAL', thermal_rows(view))]
    keys = [('S', 'STOP' if s['mode'] != 'off' else 'START'),
            ('M', s['mode'].upper() if s['mode'] != 'off' else view['mode'].upper()),
            ('V', view['source'].upper()), ('I', 'INJ'),
            ('+ -', 'IQ'), ('[ ]', 'STEP'), ('O L', 'I/F'), ('R', 'RESET')]
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
                 'load', 'noise', 'theta0', 'sat'):
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


def main(argv=None):
    args = parse_args(argv)
    sane(args)

    from screen import boot, curtain, stage
    with boot('LINKING ROTOR OBSERVER'):
        rig = Coaxial63100(port=args.port, power_afe=False,
                           simulated_device=bool(args.simulated)).open()
    origin, board = rig.origin, rig.board
    was_on = board.afe.is_on()
    want_afe = args.afe or args.source == 'adc'
    if want_afe != was_on:
        board.afe.enable() if want_afe else board.afe.disable()
        time.sleep(0.3)
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))
    try:
        params = preflight(rig, args)
    except RigError as exc:
        say('fail', 'drive', str(exc))
        rig.close()
        return 1

    view = {'source': args.source, 'mode': args.mode, 'iq': args.iq,
            'id': args.id, 'omega': args.omega, 'accel': args.accel,
            'vd': args.vd, 'v_inj': args.v_inj, 'inject': True,
            'inj_periods': int(params.get('drv_inj_periods') or 1),
            'step': 0.1, 'switch': args.switch, 'interlock': args.interlock,
            'i_max': params['drv_i_max_ma'], 'theta0': args.theta0 or 0.0,
            'params': params, 'said': '', 'state': board.drive.state(),
            'gate': board.gate_drivers.state(), 'model': None,
            'thermal': None, 'budget': None}
    if args.start:
        view['said'] = act(rig, 's', view)

    board_view = stage()
    console = board_view.is_terminal
    leaving, frame, last_thermal = None, 0, 0.0
    try:
        with curtain(board_view) as show, Keys(console) as keys:
            while True:
                try:
                    view['state'] = board.drive.state()
                    view['gate'] = board.gate_drivers.state()
                    view['model'] = (board.drive.model()
                                     if view['source'] == 'model' else None)
                    if time.time() - last_thermal > 2.0:
                        view['thermal'] = board.thermal.state()
                        view['budget'] = board.thermal.budget()
                        last_thermal = time.time()
                except RigError:
                    pass                    # a missed reply is a missed frame
                show.update(compose(rig, origin, console, view), refresh=True)
                frame += 1
                if args.frames and frame >= args.frames:
                    break
                leaving, _ = keys.poll()
                if leaving:
                    break
                for key in keys.taken():
                    said = act(rig, key, view)
                    if said:
                        view['said'] = said
                time.sleep(1.0 / max(args.hz, 0.5))
    except KeyboardInterrupt:
        pass
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
