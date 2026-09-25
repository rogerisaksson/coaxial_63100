"""The rotor observer's text rows: drive, observer, phases, status, chain, loop."""
import math
import time

from rich.text import Text

from coaxial.draw import cross_section
from motor import pmsm
from terminal.ui.screen import ASH, SODIUM, tint
from terminal.views.rotor.layout import BAR_CELLS, BAR_GLYPH
from terminal.views.rotor.thermal import envelope_acting


#: Amber within this many of the chain's corner `wc`; below one corner the
#: chain calls itself invalid (red): `v - R i` has no back-EMF there.
FLOOR_MARGIN = 3.0


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


def phase_amps(view):
    """The three phase currents, and what to call full scale."""
    s = view['state']
    theta = s['theta_hat']
    cos, sin = math.cos(theta), math.sin(theta)
    alpha = s['id'] * cos - s['iq'] * sin
    beta = s['id'] * sin + s['iq'] * cos
    root3 = math.sqrt(3.0) / 2.0
    amps = (alpha, -0.5 * alpha + root3 * beta, -0.5 * alpha - root3 * beta)
    # Full scale is the vector, not the trip.
    return amps, math.hypot(s['id'], s['iq'])


def phase_rows(view):
    """The legend: which colour is which phase, and how hard it is driven."""

    amps, full = phase_amps(view)
    scale = full or max((abs(a) for a in amps), default=0.0) or 1.0
    rows = []
    for name, cls, value in zip(cross_section.PHASE_NAMES, cross_section.PHASE_CLASS, amps):
        ramp = cross_section.PHASE_RAMP[cls]
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
    """What motor the record says is on the shaft, or that it cannot say."""
    params = view['params']
    pairs = int(params.get('motor_pole_pairs') or 0)
    lam = params.get('motor_lambda') or 0.0
    if not pairs or not lam:
        return Text(' IDENTIFYING ', style='chip.sim')
    kv = pmsm.Parameters('', params.get('motor_r') or 0.0,
                          params.get('motor_ld') or 0.0,
                          params.get('motor_lq') or 0.0,
                          lam, pairs).kv
    return '%dN%dP %2d pp  KV %4.0f' % (view['slots'], 2 * pairs, pairs, kv)


def torque(view):
    """Shaft torque from the loop's own dq, newton-metres."""
    s, params = view['state'], view['params']
    pairs = params.get('motor_pole_pairs') or 0.0
    lam = params.get('motor_lambda') or 0.0
    ld = params.get('motor_ld') or 0.0
    lq = params.get('motor_lq') or 0.0
    return 1.5 * pairs * (lam * s['iq'] + (ld - lq) * s['id'] * s['iq'])


def status_rows(view):
    """Two rows, and neither of them is anywhere else on the page."""
    o = view.get('chain') or {}
    gone = view['travel'] - view['tare']
    loops = ' + '.join([n for n, on in (('speed', view['spin']),
                                        ('load', view['load'])) if on])
    if view['spin'] and view['stage']:
        loops = '%s: %s' % (view['stage'].upper(), loops)
    if view['clock'].now() < view['burst_until']:
        loops = 'BURST' + (' + ' + loops if loops else '')
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    speed = (view.get('chain') or {}).get('omega') or 0.0
    return [('motor', identity(view)),
            ('shaft', '%8.0f rpm %8.3f N.m'
             % (speed / pairs * 60.0 / math.tau, torque(view))),
            ('back-EMF', regime(view)),
            ('loops', loops or 'none - the drive is on its own'),
            ('travel', '%9.1f deg %7.2f turns %s'
             % (gone, gone / 360.0,
                'cw' if (o.get('omega') or 0.0) >= 0.0 else 'ccw')),
            # The cell's shape, and where the number came from.
            ('cell', '%.2f tall %s' % (view.get('aspect', cross_section.CELL_ASPECT),
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


def chain_rows(view):
    """The back-EMF chain that runs beside the loop, and what it costs."""
    o = view.get('chain')
    if not o:
        return [('chain', '%7s' % '--')] * 7
    pp = max(1.0, view['params'].get('motor_pole_pairs', 1.0))
    lam = view['params'].get('motor_lambda') or 0.0
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


#: The envelope's word beside the mode, in a red darker than the trip's 196
#: and the pulse's 210: held, not hurt.
THROTTLE_RED = 124


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
