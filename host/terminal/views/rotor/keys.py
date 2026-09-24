"""The rotor observer's keys: one handler per key, the table, the dispatch."""
import math
import time

from coaxial.errors import RigError
from terminal.views.rotor.motions import BURST_A, BURST_HOLD_S, BURST_S


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
        rig.gates.off()
        return 'stage disarmed'
    rig.gates.on(bypass_sto=True, ignore_interlock=not view['interlock'])
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
