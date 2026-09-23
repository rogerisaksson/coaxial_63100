"""The control law behind `0x6E` device 10: modes, setpoints, and what the
board measured while it ran.
"""
import json
import math
import time

from . import protocol
from .errors import RigError
from .protocol import DriveOp
from .subsystem import Device
from .wire import Reader, micro, pack

MODES = {'off': 0, 'volt': 1, 'hold': 2, 'sensorless': 3, 'polarity': 4}
MODE_NAMES = {v: k for k, v in MODES.items()}
FAULTS = {0: None, 1: 'overcurrent', 2: 'stage', 3: 'supply'}
SOURCES = {'adc': 0, 'model': 1}
SOURCE_NAMES = {v: k for k, v in SOURCES.items()}

#: The state reply's flag byte, bit by bit.
FLAGS = ('stage_enabled', 'afe_on', 'injecting', 'owns_compares',
         'sync_armed')
#: The state's currents and volts, in wire order, milli-units each.
MILLI_FIELDS = ('id', 'iq', 'vd', 'vq', 'vdc')
#: The state's observer terms, in wire order, micro-units each.
MICRO_FIELDS = ('eps', 'eps_amps', 'ih', 'e_bemf')

#: The on-board motor model's parameters by id, and the factor an SI value
#: is multiplied by on the wire. The model is the second sample source: the
#: rotor observer watched against a rotor whose angle is known, with the AFE off.
MODEL_PARAMS = (('r', 1e6), ('ld', 1e9), ('lq', 1e9), ('lambda', 1e6),
                ('pole_pairs', 1.0), ('sat', 1e6), ('i_sat', 1e3),
                ('j', 1e9), ('b', 1e9), ('load', 1e6), ('v_dt', 1e3),
                ('i_knee', 1e3), ('vdc', 1e3), ('noise', 1e6),
                ('theta0', 1e6), ('sub', 1.0))
MODEL_IDS = {name: i for i, (name, _) in enumerate(MODEL_PARAMS)}

#: Setpoints by id, and the factor an SI value is multiplied by on the wire.
SETPOINTS = (('id_ref', 1e3), ('iq_ref', 1e3), ('theta', 1e3),
             ('omega_target', 1e3), ('accel', 1e3), ('vd', 1e3), ('vq', 1e3),
             ('pol_volts', 1e3), ('pol_periods', 1.0), ('pol_gap', 1.0))
SETPOINT_IDS = {name: i for i, (name, _) in enumerate(SETPOINTS)}

#: The window's fields in wire order, and the micro-unit each comes in.
WINDOW_FIELDS = (('id', 1e6), ('iq', 1e6), ('vd', 1e6), ('vq', 1e6),
                 ('eps', 1e6), ('ih', 1e6), ('vdc', 1e3))

#: The moments' channels in wire order - the injected sequence's own.
MOMENT_CHANNELS = ('Phase U', 'Phase V', 'Phase W', 'DC bus')

#: Drive parameters in the calibration record, and their wire units, so a
#: commissioning writes SI and reads SI. Signed ones say so.
PARAMS = {
    'motor_r_uohm': 1e6, 'motor_ld_nh': 1e9, 'motor_lq_nh': 1e9,
    'motor_lambda_uvs': 1e6, 'motor_pole_pairs': 1.0,
    'drv_kp_mv_per_a': 1e3, 'drv_ki_v_per_as': 1.0,
    'drv_l1_milli': 1e3, 'drv_l2_milli': 1e3,
    'drv_inj_mv': 1e3, 'drv_inj_periods': 1.0, 'drv_inj_phase_mrad': 1e3,
    'drv_eps_gain_ua_per_rad': 1e6, 'drv_i_max_ma': 1e3, 'drv_i_trip_ma': 1e3,
    'drv_v_frac_ppm': 1e6, 'drv_sign': 1.0,
    'drv_w_lo_mrad_s': 1e3, 'drv_w_hi_mrad_s': 1e3, 'drv_dt_step_ma': 1e3,
    'drv_sigma_i_ua': 1e6, 'drv_trigger_ticks': 1.0,
    # The winding's envelope, CAL_VERSION 12: K/W and J/K in milli, the ceiling
    # in centi-degrees.
    'winding_k_per_w_milli': 1e3, 'winding_j_per_k_milli': 1e3,
    'winding_limit_centi': 1e2,
}
SIGNED = ('drv_inj_phase_mrad', 'drv_eps_gain_ua_per_rad', 'drv_sign',
          'winding_limit_centi')


def to_wire(name, value):
    """An SI value as the u32 the record holds for `name`."""
    raw = int(round(value * PARAMS[name]))
    return raw & 0xFFFFFFFF if name in SIGNED else raw


def from_wire(name, raw):
    """The record's u32 for `name`, in SI."""
    if name in SIGNED and raw & 0x80000000:
        raw -= 1 << 32
    return raw / PARAMS[name]


def _known(table, name, what):
    """`table[name]`, or a ValueError naming every `what` there is."""
    if name not in table:
        raise ValueError('%r is not a %s; they are %s'
                         % (name, what, ', '.join(table)))
    return table[name]


def _wrapped(radians):
    """An angle error folded into (-pi, pi]."""
    return (radians + math.pi) % math.tau - math.pi


def load_profile(drive, path):
    """A motor profile - a JSON file of `drive` parameters (the record's
    names, SI) and `model` parameters - written through `drive`, the
    board's or the stand-in's."""
    with open(path, encoding='utf-8') as handle:
        data = json.load(handle)
    done = {'name': data.get('name', path)}
    if data.get('drive'):
        done['drive'] = drive.set_params(**data['drive'])
    if data.get('model'):
        done['model'] = drive.model_param(**data['model'])
    return done


class Drive(Device, device=protocol.DEVICE_DRIVE):

    """Device 10 behind 0x6E: the current loop, injection and rotor observer."""

    def state(self):
        """What the drive is doing now, in SI."""
        r = Reader(self._op(DriveOp.STATE))
        out = {'mode': MODE_NAMES.get(r.u8(), 'unknown'),
               'fault': FAULTS.get(r.u8(), 'unknown')}
        out.update(r.flags(FLAGS))
        out['theta_hat'] = r.micro()
        out['omega_hat'] = r.milli()
        out['theta_cmd'] = r.micro()
        out['omega_cmd'] = r.milli()
        for name in MILLI_FIELDS:
            out[name] = r.milli()
        for name in MICRO_FIELDS:
            out[name] = r.micro()
        out['periods'] = r.u32()
        out['isr_cycles_last'] = r.u32()
        out['isr_cycles_max'] = r.u32()
        out['pol_pos'] = r.milli()
        out['pol_neg'] = r.milli()
        out['trigger'] = r.u16()
        out['ts'] = r.nano()
        # The worst end of a step in TIM1 ticks past the trigger: the whole
        # interrupt, against the period's 2 x ARR ticks.
        out['exit_ticks_max'] = r.maybe('u16')
        # The virtual step block by block, raw cycles; zero on the ADC.
        if r.remaining >= 12:
            out['cycles'] = {'sample': r.u32(), 'step': r.u32(),
                             'advance': r.u32()}
        return out

    def mode(self, name):
        """Enter a mode by name."""
        return self._ack(DriveOp.MODE, pack(('u8', _known(MODES, name, 'mode'))))

    def off(self):
        """Mode off. Never refused."""
        return self.mode('off')

    def _by_name(self, op, table, ids, what, values):
        """Named SI values, one `u8 id, i32` op each; what landed, in SI."""
        scales = dict(table)
        done = {}
        for name, value in values.items():
            ident = _known(ids, name, what)
            raw = int(round(value * scales[name]))
            self._ack(op, pack(('u8', ident), ('i32', raw)))
            done[name] = raw / scales[name]
        return done

    def setpoint(self, **values):
        """Set setpoints by name, SI: id_ref/iq_ref A, theta rad,
        omega_target rad/s, accel rad/s^2, vd/vq V, pol_volts V,
        pol_periods/pol_gap PWM periods.
        """
        return self._by_name(DriveOp.SETPOINT, SETPOINTS, SETPOINT_IDS,
                             'setpoint', values)

    def setpoints(self):
        """Every setpoint as the board holds it, SI."""
        r = Reader(self._op(DriveOp.SETPOINTS))
        raws = [r.i32() for _ in range(r.u8())]
        return {name: raw / scale
                for (name, scale), raw in zip(SETPOINTS, raws)}

    def set_theta(self, radians):
        """Put both frames at an angle: the polarity flip, or a known start."""
        return self._ack(DriveOp.THETA, pack(('i32', micro(radians))))

    def window(self):
        """The window since the last take, then a new one starts."""
        r = Reader(self._op(DriveOp.WINDOW))
        out = {'n': r.u32(), 'fields': {}}
        for name, scale in WINDOW_FIELDS:
            n = r.u32()
            mean = r.i32() / scale
            sd = r.u32() / scale
            out['fields'][name] = {'n': n, 'mean': mean if n else None,
                                   'sd': sd if n else None}
        out['rho'] = [r.micro() for _ in range(r.u8())]
        out['i_peak'] = r.milli()
        return out

    def moments_arm(self, periods):
        """Count raw codes at the sample point for this many periods."""
        return self._ack(DriveOp.MOMENTS_ARM, pack(('u32', int(periods))))

    def moments(self):
        """The moments so far: per channel mean, sd (codes), lowest, highest;
        `done` once `n` reached `want`; `trigger` is CCR5.
        """
        r = Reader(self._op(DriveOp.MOMENTS))
        out = {'done': bool(r.u8()), 'n': r.u32(), 'want': r.u32(),
               'trigger': r.u16(), 'channels': {}}
        for name in MOMENT_CHANNELS:
            out['channels'][name] = {'mean': r.milli(),
                                     'sd': r.milli('u32'),
                                     'lo': r.i32(), 'hi': r.i32()}
        return out

    def moments_run(self, periods, timeout=5.0, poll=0.02):
        """Arm, wait for the count, return the moments."""
        self.moments_arm(periods)
        deadline = time.time() + timeout
        while True:
            got = self.moments()
            if got['done']:
                return got
            if time.time() > deadline:
                raise RigError('%d of %d periods counted in %.1f s - is the '
                               'sync armed and the timer running? %s'
                               % (got['n'], periods, timeout, self.state()))
            time.sleep(poll)

    def reload(self):
        """Take the parameters out of the calibration record again."""
        return self._ack(DriveOp.RELOAD)

    def reset_cycles(self):
        """Forget the worst step cost, so a run is measured on its own."""
        return bool(Reader(self._op(DriveOp.CYCLES_RESET)).u8())

    # -- the model as the source -----------------------------------------

    def source(self, name):
        """Where the samples come from: 'adc' or 'model'."""
        return self._ack(DriveOp.SOURCE,
                         pack(('u8', _known(SOURCES, name, 'source'))))

    def model_param(self, **values):
        """Set model parameters by name, SI: r ohm, ld/lq H, lambda V.s,
        pole_pairs, sat (fraction Ld bends by at i_sat), i_sat A, j
        kg.m2, b N.m.s, load N.m, v_dt V, i_knee A, vdc V, noise A rms,
        theta0 rad, sub steps.
        """
        return self._by_name(DriveOp.MODEL_PARAM, MODEL_PARAMS, MODEL_IDS,
                             'model parameter', values)

    def model(self):
        """The model's truth: source, the rotor's angle and speed (electrical),
        its dq currents, the link it runs from.
        """
        r = Reader(self._op(DriveOp.MODEL))
        out = {'source': SOURCE_NAMES.get(r.u8(), 'unknown'),
               'theta': r.micro(), 'omega': r.milli(),
               'id': r.milli(), 'iq': r.milli(),
               'vdc': r.milli()}
        # The estimate in the same reply, so the error means something at
        # speed: two requests are 15 ms apart, six radians at 440 rad/s.
        if r.remaining >= 8:
            out['theta_hat'] = r.micro()
            out['omega_hat'] = r.milli()
            out['error'] = _wrapped(out['theta_hat'] - out['theta'])
        return out

    def observers(self):
        """The back-EMF observer chain that runs beside the loop, in SI."""
        r = Reader(self._op(DriveOp.OBSERVERS))
        out = {'valid': bool(r.u8()),
               'theta': r.micro(), 'omega': r.milli(),
               'blend': r.micro(),
               'dual_theta': r.micro(), 'dual_omega': r.milli(),
               'flux_theta': r.micro(), 'flux_omega': r.milli(),
               'lambda_hat': r.micro(),
               'theta_hat': r.micro(), 'omega_hat': r.milli(),
               'blend_lo': r.milli(), 'blend_hi': r.milli(),
               'wc': r.milli()}
        out['error'] = _wrapped(out['theta'] - out['theta_hat'])
        return out

    def model_reset(self):
        """The rotor back to theta0, at rest."""
        return self._ack(DriveOp.MODEL_RESET)

    def profile(self, path):
        """A motor profile written to the board (`load_profile`)."""
        return load_profile(self, path)

    # -- the record ------------------------------------------------------

    def params(self):
        """The drive's parameters out of the calibration record, in SI."""
        record = self.board.calibration.read()['params']
        return {name: from_wire(name, record[name])
                for name in PARAMS if name in record}

    def set_params(self, **values):
        """Write drive parameters into the record (RAM) in SI, and reload."""
        for name, value in values.items():
            _known(PARAMS, name, 'drive parameter')
            self.board.calibration.set_param(name, to_wire(name, value))
        self.reload()
        return {name: from_wire(name, to_wire(name, v))
                for name, v in values.items()}
