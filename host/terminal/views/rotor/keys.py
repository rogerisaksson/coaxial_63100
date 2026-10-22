"""The rotor observer's keys: one handler per key, its rows of the terminal's one table (PAD)."""
import math

from rich.text import Text

from coaxial.errors import RigError
from terminal.ui import bindings, console as _console
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
        d.set_theta(d.model.read()['theta'] + 0.3)
    d.write(iq_ref=view['iq'], id_ref=view['id'], theta=0.0,
            omega_target=view['omega'] if view['mode'] == 'hold' else 0.0,
            accel=view['accel'], vd=view['vd'], vq=0.0)
    d.on(view['mode'])
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
    d.configure(source=view['source'])
    return 'source %s' % view['source']


def _key_inject(rig, d, key, view):
    view['inject'] = not view['inject']
    d.configure(drv_inj_volts=view['v_inj'] if view['inject'] else 0.0)
    return 'injection %s' % ('on' if view['inject'] else 'off')


def _key_iq(rig, d, key, view):
    view['iq'] += view['step'] if key in '+=' else -view['step']
    view['iq'] = max(-view['i_max'], min(view['i_max'], view['iq']))
    d.write(iq_ref=view['iq'])
    return 'iq_ref %+.2f A' % view['iq']


def _key_step(rig, d, key, view):
    i = STEPS.index(view['step']) + (1 if key == ']' else -1)
    view['step'] = STEPS[max(0, min(len(STEPS) - 1, i))]
    return 'step %.2f A' % view['step']


def _key_omega(rig, d, key, view):
    view['omega'] = max(0.0, min(LIMITS['omega'][1],
                                 view['omega'] + (50.0 if key == 'o' else -50.0)))
    d.write(omega_target=view['omega'] if view['mode'] == 'hold' else 0.0)
    return 'I/f target %.0f rad/s' % view['omega']


def _key_burst(rig, d, key, view):
    view['burst_at'] = view['clock'].now()
    view['burst_until'] = view['burst_at'] + BURST_S + BURST_HOLD_S
    return ('heavy start - %.0f A for %.1f s, then %.0f s at half '
            'speed' % (BURST_A, BURST_S, BURST_HOLD_S))


def _key_spin(rig, d, key, view):
    view['spin'] = not view['spin']
    view['spin_at'] = view['clock'].now()
    view['stage_index'] = None
    if not view['spin']:
        d.write(omega_target=0.0, iq_ref=0.0)
    return ('the demo: up, coast and brake each way, then a load'
            if view['spin'] else 'speed loop off')


def _key_load(rig, d, key, view):
    view['load'] = not view['load']
    view['load_at'] = view['clock'].now()
    view['load_amps'] = view['load_written'] = 0.0
    if not view['load']:
        d.write(id_ref=0.0)
    return ('load loop running - d current in steps'
            if view['load'] else 'load loop off')


def _key_tare(rig, d, key, view):
    """TARE: the pointer's zero, not the board's - the mark where the rotor stands now."""
    view['tare'] = view['state'].get('travel', view['travel'])
    return 'tared - the pointer reads travel from here'


def _key_reset(rig, d, key, view):
    d.model.reset()
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


def _told(handler):
    """The act of `handler` (rig, drive, key, view) on the board (`view['rig']`), a letter either
    case: what it says on the page, the board's refusal where it refuses."""
    def told(view, key, _now):
        try:
            view['said'] = handler(view['rig'], view['rig'].board.drive, key.lower(), view) or view['said']
        except RigError as exc:
            view['said'] = str(exc)
    return told


def _lit(word, on):
    """A bar word that stays put, its colour lit while `on`: a chip that came and went moved every
    key after it."""
    return Text(word, style='chip.live') if on else word


#: The page's keys, rows of the terminal's one table (`terminal.ui.bindings`), each act a handler
#: `(rig, drive, key, view) -> what to say`; F lends the mouse to the view (`console.SELECT_KEYS`).
PAD = bindings.page((
    ('DRIVE', 'S', 'the drive in its mode, or stopped',
     (('S', lambda v: '%-5s' % ('STOP' if v['state']['mode'] != 'off' else 'START')),),
     bindings.cased('s', _told(_key_start_stop))),
    ('DRIVE', 'M', 'the next mode: %s' % ', '.join(MODES),
     (('M', lambda v: '%-10s' % (v['state']['mode'].upper() if v['state']['mode'] != 'off' else v['mode'].upper())),),
     bindings.cased('m', _told(_key_mode))),
    ('DRIVE', 'V', "the observer's source: the model, the ADC", (('V', lambda v: v['source'].upper()),),
     bindings.cased('v', _told(_key_source))),
    ('DRIVE', 'I', 'the injection on, or off', (('I', 'INJ'),), bindings.cased('i', _told(_key_inject))),
    ('DRIVE', '+ -', 'iq_ref up, down a step', (('+ -', 'IQ'),), dict.fromkeys('+=-_', _told(_key_iq))),
    ('DRIVE', '[ ]', "the step down, up: %s A" % ', '.join('%g' % s for s in STEPS), (('[ ]', 'STEP'),),
     dict.fromkeys('[]', _told(_key_step))),
    ('DRIVE', 'O L', 'the I/f target up, down 50 rad/s', (('O L', 'I/F'),),
     {**bindings.cased('o', _told(_key_omega)), **bindings.cased('l', _told(_key_omega))}),
    ('DRIVE', 'A', 'the stage armed, or not - its switch given', (('A', lambda v: (_lit('ARMED', True) if v['state']['stage_enabled']
                                                                                   else 'ARM  ') if v['switch'] else None),),
     bindings.cased('a', _told(_key_arm))),
    ('DEMO', 'B', 'a heavy start: %.0f A for %.1f s, then %.0f s at half speed' % (BURST_A, BURST_S, BURST_HOLD_S),
     (('B', lambda v: Text('START', style='alarm') if v['clock'].now() < v['burst_until'] else 'START'),),
     bindings.cased('b', _told(_key_burst))),
    ('DEMO', 'E', 'the speed demo: up, coast and brake each way, a load', (('E', lambda v: _lit('SPEED', v['spin'])),),
     bindings.cased('e', _told(_key_spin))),
    ('DEMO', 'W', 'the load loop: d current in steps', (('W', lambda v: _lit('LOAD', v['load'])),),
     bindings.cased('w', _told(_key_load))),
    ('VIEW', 'T', "the pointer's zero where the rotor stands", (('T', 'TARE'),), bindings.cased('t', _told(_key_tare))),
    ('VIEW', 'R', "the model's rotor reset, theta_hat 0.3 rad off it", (('R', 'RESET'),), bindings.cased('r', _told(_key_reset))),
    ('VIEW', 'F', "the mouse the view's - lit -, or the terminal's", (('F', lambda _v: _lit('MOUSE', _console.holding())),), {}),
))
