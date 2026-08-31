"""The control law behind `0x6E` device 10: modes, setpoints, and what the
board measured while it ran.

The board runs a dq current loop, HF injection, a rotor observer and an I/f ramp
at the PWM rate; nothing here closes a loop. This sets what it is asked to
do and reads back, in SI, what it did - the window's means and deviations,
the innovation's autocorrelation, the raw-code moments at the sample point.
The gains it runs on live in the calibration record (device 3), so the
board runs the same drive after a reset; `board.calibration.set_param` is
how they get there and `reload()` is how the drive picks them up.

Every refusal is the board's own sentence.
"""
import math
import time

from . import protocol
from .errors import RigError
from .subsystem import Subsystem
from .wire import Reader

DRIVE_OP_STATE = 0
DRIVE_OP_MODE = 1
DRIVE_OP_SETPOINT = 2
DRIVE_OP_SETPOINTS = 3
DRIVE_OP_THETA = 4
DRIVE_OP_WINDOW = 5
DRIVE_OP_MOMENTS_ARM = 6
DRIVE_OP_MOMENTS = 7
DRIVE_OP_RELOAD = 8
DRIVE_OP_CYCLES_RESET = 9
DRIVE_OP_SOURCE = 10
DRIVE_OP_MODEL_PARAM = 11
DRIVE_OP_MODEL = 12
DRIVE_OP_MODEL_RESET = 13

MODES = {'off': 0, 'volt': 1, 'hold': 2, 'sensorless': 3, 'polarity': 4}
MODE_NAMES = {v: k for k, v in MODES.items()}
FAULTS = {0: None, 1: 'overcurrent', 2: 'stage', 3: 'supply'}
SOURCES = {'adc': 0, 'model': 1}
SOURCE_NAMES = {v: k for k, v in SOURCES.items()}

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
}
SIGNED = ('drv_inj_phase_mrad', 'drv_eps_gain_ua_per_rad', 'drv_sign')


def to_wire(name, value):
    """An SI value as the u32 the record holds for `name`."""
    raw = int(round(value * PARAMS[name]))
    return raw & 0xFFFFFFFF if name in SIGNED else raw


def from_wire(name, raw):
    """The record's u32 for `name`, in SI."""
    if name in SIGNED and raw & 0x80000000:
        raw -= 1 << 32
    return raw / PARAMS[name]


class Drive(Subsystem):

    """Device 10 behind 0x6E: the current loop, injection and rotor observer."""

    def _op(self, op, payload=b''):
        return self.request(protocol.DEVICE,
                            bytes([protocol.DEVICE_DRIVE, op]) + bytes(payload))

    def state(self):
        """What the drive is doing now, in SI. Angles rad, speeds rad/s
        electrical, currents A, volts V; `isr_cycles_*` in raw CYCCNT."""
        r = Reader(self._op(DRIVE_OP_STATE))
        out = {'mode': MODE_NAMES.get(r.u8(), 'unknown'),
               'fault': FAULTS.get(r.u8(), 'unknown')}
        flags = r.u8()
        out['stage_enabled'] = bool(flags & 0x01)
        out['afe_on'] = bool(flags & 0x02)
        out['injecting'] = bool(flags & 0x04)
        out['owns_compares'] = bool(flags & 0x08)
        out['sync_armed'] = bool(flags & 0x10)
        out['theta_hat'] = r.i32() / 1e6
        out['omega_hat'] = r.i32() / 1e3
        out['theta_cmd'] = r.i32() / 1e6
        out['omega_cmd'] = r.i32() / 1e3
        for name in ('id', 'iq', 'vd', 'vq', 'vdc'):
            out[name] = r.i32() / 1e3
        for name in ('eps', 'eps_amps', 'ih', 'e_bemf'):
            out[name] = r.i32() / 1e6
        out['periods'] = r.u32()
        out['isr_cycles_last'] = r.u32()
        out['isr_cycles_max'] = r.u32()
        out['pol_pos'] = r.i32() / 1e3
        out['pol_neg'] = r.i32() / 1e3
        out['trigger'] = r.u16()
        out['ts'] = r.u32() / 1e9
        # The worst end of a step in TIM1 ticks past the trigger: the whole
        # interrupt, against the period's 2 x ARR ticks. Appended.
        out['exit_ticks_max'] = r.u16() if r.remaining >= 2 else None
        # The virtual step block by block, raw cycles; zero on the ADC.
        if r.remaining >= 12:
            out['cycles'] = {'sample': r.u32(), 'step': r.u32(),
                             'advance': r.u32()}
        return out

    def mode(self, name):
        """Enter a mode by name. The board refuses with the reason: a
        switching mode needs MOE set (gates.arm()) and the AFE on."""
        if name not in MODES:
            raise ValueError('%r is not a mode; they are %s'
                             % (name, ', '.join(MODES)))
        self.took(self._op(DRIVE_OP_MODE, bytes([MODES[name]])))
        return True

    def off(self):
        """Mode off. Never refused."""
        return self.mode('off')

    def setpoint(self, **values):
        """Set setpoints by name, SI: id_ref/iq_ref A, theta rad,
        omega_target rad/s, accel rad/s^2, vd/vq V, pol_volts V,
        pol_periods/pol_gap PWM periods. Returns what was set."""
        done = {}
        for name, value in values.items():
            if name not in SETPOINT_IDS:
                raise ValueError('%r is not a setpoint; they are %s'
                                 % (name, ', '.join(SETPOINT_IDS)))
            scale = dict(SETPOINTS)[name]
            raw = int(round(value * scale))
            self.took(self._op(DRIVE_OP_SETPOINT, bytes([SETPOINT_IDS[name]])
                               + raw.to_bytes(4, 'big', signed=True)))
            done[name] = raw / scale
        return done

    def setpoints(self):
        """Every setpoint as the board holds it, SI."""
        r = Reader(self._op(DRIVE_OP_SETPOINTS))
        count = r.u8()
        out = {}
        for i in range(count):
            raw = r.i32()
            if i < len(SETPOINTS):
                name, scale = SETPOINTS[i]
                out[name] = raw / scale
        return out

    def set_theta(self, radians):
        """Put both frames at an angle: the polarity flip, or a known start."""
        raw = int(round(radians * 1e6))
        self.took(self._op(DRIVE_OP_THETA, raw.to_bytes(4, 'big', signed=True)))
        return True

    def window(self):
        """The window since the last take, then a new one starts.

        `fields[name]` is `{'n', 'mean', 'sd'}` in SI; `rho[j]` is the
        innovation's autocorrelation at lag j+1, for the whiteness test;
        `i_peak` the largest |i_dq| seen. `n` counts periods.
        """
        r = Reader(self._op(DRIVE_OP_WINDOW))
        out = {'n': r.u32(), 'fields': {}}
        for name, scale in WINDOW_FIELDS:
            n = r.u32()
            mean = r.i32() / scale
            sd = r.u32() / scale
            out['fields'][name] = {'n': n, 'mean': mean if n else None,
                                   'sd': sd if n else None}
        lags = r.u8()
        out['rho'] = [r.i32() / 1e6 for _ in range(lags)]
        out['i_peak'] = r.i32() / 1e3
        return out

    def moments_arm(self, periods):
        """Count raw codes at the sample point for this many periods.
        Needs the sync armed; the board says so otherwise."""
        self.took(self._op(DRIVE_OP_MOMENTS_ARM, int(periods).to_bytes(4, 'big')))
        return True

    def moments(self):
        """The moments so far: per channel mean, sd (codes), lowest,
        highest; `done` once `n` reached `want`; `trigger` is CCR5."""
        r = Reader(self._op(DRIVE_OP_MOMENTS))
        out = {'done': bool(r.u8()), 'n': r.u32(), 'want': r.u32(),
               'trigger': r.u16(), 'channels': {}}
        for name in MOMENT_CHANNELS:
            out['channels'][name] = {'mean': r.i32() / 1e3,
                                     'sd': r.u32() / 1e3,
                                     'lo': r.i32(), 'hi': r.i32()}
        return out

    def moments_run(self, periods, timeout=5.0, poll=0.02):
        """Arm, wait for the count, return the moments. Blocking on this
        side only; the board never sits on a reply."""
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
        self.took(self._op(DRIVE_OP_RELOAD))
        return True

    def reset_cycles(self):
        """Forget the worst step cost, so a run is measured on its own."""
        return self._op(DRIVE_OP_CYCLES_RESET)[0] == 1

    # -- the model as the source -----------------------------------------

    def source(self, name):
        """Where the samples come from: 'adc' or 'model'. Refused while a
        mode runs. With the model the law needs no reference and no stage,
        and its duties reach the gates only if MOE happens to be set."""
        if name not in SOURCES:
            raise ValueError('%r is not a source; they are %s'
                             % (name, ', '.join(SOURCES)))
        self.took(self._op(DRIVE_OP_SOURCE, bytes([SOURCES[name]])))
        return True

    def model_param(self, **values):
        """Set model parameters by name, SI: r ohm, ld/lq H, lambda V.s,
        pole_pairs, sat (fraction Ld bends by at i_sat), i_sat A, j kg.m2,
        b N.m.s, load N.m, v_dt V, i_knee A, vdc V, noise A rms, theta0
        rad, sub steps. Returns what was set."""
        done = {}
        for name, value in values.items():
            if name not in MODEL_IDS:
                raise ValueError('%r is not a model parameter; they are %s'
                                 % (name, ', '.join(MODEL_IDS)))
            scale = dict(MODEL_PARAMS)[name]
            raw = int(round(value * scale))
            self.took(self._op(DRIVE_OP_MODEL_PARAM, bytes([MODEL_IDS[name]])
                               + raw.to_bytes(4, 'big', signed=True)))
            done[name] = raw / scale
        return done

    def model(self):
        """The model's truth: source, the rotor's angle and speed
        (electrical), its dq currents, the link it runs from."""
        r = Reader(self._op(DRIVE_OP_MODEL))
        out = {'source': SOURCE_NAMES.get(r.u8(), 'unknown'),
               'theta': r.i32() / 1e6, 'omega': r.i32() / 1e3,
               'id': r.i32() / 1e3, 'iq': r.i32() / 1e3,
               'vdc': r.i32() / 1e3}
        # The estimate in the same reply, so the error means something at
        # speed: two requests are 15 ms apart, six radians at 440 rad/s.
        if r.remaining >= 8:
            out['theta_hat'] = r.i32() / 1e6
            out['omega_hat'] = r.i32() / 1e3
            err = out['theta_hat'] - out['theta']
            out['error'] = (err + math.pi) % (2.0 * math.pi) - math.pi
        return out

    def model_reset(self):
        """The rotor back to theta0, at rest."""
        self.took(self._op(DRIVE_OP_MODEL_RESET))
        return True

    def profile(self, path):
        """A motor profile - a JSON file of `drive` parameters (the record's
        names, SI) and `model` parameters - written to the board. Returns
        what was written. The file says which motor; nothing here does."""
        import json

        with open(path, encoding='utf-8') as handle:
            data = json.load(handle)
        done = {'name': data.get('name', path)}
        if data.get('drive'):
            done['drive'] = self.set_params(**data['drive'])
        if data.get('model'):
            done['model'] = self.model_param(**data['model'])
        return done

    # -- the record ------------------------------------------------------

    def params(self):
        """The drive's parameters out of the calibration record, in SI."""
        record = self.board.calibration.read()['params']
        return {name: from_wire(name, record[name])
                for name in PARAMS if name in record}

    def set_params(self, **values):
        """Write drive parameters into the record (RAM) in SI, and reload.
        `board.calibration.save()` is what keeps them across a reset."""
        for name, value in values.items():
            if name not in PARAMS:
                raise ValueError('%r is not a drive parameter; they are %s'
                                 % (name, ', '.join(PARAMS)))
            self.board.calibration.set_param(name, to_wire(name, value))
        self.reload()
        return {name: from_wire(name, to_wire(name, v))
                for name, v in values.items()}
