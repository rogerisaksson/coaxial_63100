"""The stand-in drive: its state, modes, setpoints and parameters, on one lock."""
import math
import threading
import time
from typing import Callable, Optional, Any

from coaxial.devices.drive import MODES, PARAMS, SOURCES, DriveControl
from coaxial.errors import RigError
from coaxial.model.motor import BENCH_MOTOR
from coaxial.simulated.drive.capture import DriveCapture
from coaxial.simulated.drive.locked import _rotor_locked
from coaxial.simulated.drive.observers import DriveObservers
from coaxial.simulated.drive.plant import DrivePlant
from coaxial.simulated.values import DCBUS_V


class SimulatedDrive(DrivePlant, DriveObservers, DriveCapture, DriveControl):
    """The control law's device without a motor: a locked rotor at zero
    electrical angle, a phase resistance, two inductances that bend with
    the d current, a flux linkage and a dead-time voltage error - enough
    for every commissioning step to recover a number it can check against
    the constants below.
    """

    #: One definition, in `coaxial.model.motor`, so this machine cannot drift
    #: away from the ones the identification and the notebook use.
    R = BENCH_MOTOR.r
    LD = BENCH_MOTOR.ld
    LQ = BENCH_MOTOR.lq
    LAMBDA = BENCH_MOTOR.lam
    POLES = BENCH_MOTOR.poles
    SAT = 0.3           #: Ld bends by this much at I_SAT of d current
    I_SAT = 4.0
    V_DT = 0.5          #: the inverter's dead-time voltage error, V
    I_KNEE = 0.3
    #: TIM1's counter clock. Centre-aligned, so a period is 2 x ARR.
    CLK = 237.5e6
    #: WHERE THE STAGE SWITCHES, and the one place it is written. The
    #: `.ioc` sets ARR 2375 off 237.5 MHz, which is 50 kHz, and nothing on
    #: the wire moves it - ARR comes from CubeMX and the board only reads
    #: it back. The stand-in takes 10 kHz to 100 kHz instead, so the
    #: observers, the injection and the model can be tried across the
    #: range before a period other than this one exists to try them at:
    #:
    #:     rig.board.drive._sim.pwm_hz = 100e3
    #:
    #: `TS`, `FS` and `PERIOD` all follow it, and so does everything that
    #: reads them - the chain steps at whatever `TS` says, the way
    #: `drive_observer_step` takes its `ts` from the drive rather than a
    #: constant of its own.
    PWM_HZ = 50e3
    PWM_HZ_MIN = 10e3
    PWM_HZ_MAX = 100e3

    @property
    def pwm_hz(self):
        """Where the stage switches, Hz."""
        return self._pwm_hz or self.PWM_HZ

    @pwm_hz.setter
    def pwm_hz(self, hz):
        hz = float(hz)
        if not self.PWM_HZ_MIN <= hz <= self.PWM_HZ_MAX:
            raise RigError(
                'a switching frequency of %.0f Hz is outside %.0f to %.0f - '
                'below it the current ripple runs away and above it the '
                'dead time is a bigger share of the period than the duty'
                % (hz, self.PWM_HZ_MIN, self.PWM_HZ_MAX))
        self._pwm_hz = hz
        # The chain's constants are rates, not per-period gains, so they carry
        # across a change; its integrators do not - they hold flux accumulated
        # at the old period.
        self._obs = None

    @property
    def TS(self):
        """One control period, seconds."""
        return 1.0 / self.pwm_hz

    @property
    def FS(self):
        """Control periods a second - what a window or a moments run fills at."""
        return self.pwm_hz

    @property
    def PERIOD(self):
        """ARR + 1, which is what `Board_PwmPeriod` answers."""
        return int(round(self.CLK / (2.0 * self.pwm_hz))) + 1
    BEST_TRIGGER = 2300                          #: where the pickup is least

    def __init__(self):
        # ONE ROTOR, TWO THREADS.
        self._lock = threading.RLock()
        self._mode = 'off'
        #: The stage's PWM when set by hand, else PWM_HZ; and the shaft,
        #: accumulated - electrical theta wraps, the mechanical angle
        #: a shaft sensor reads does not.
        self._pwm_hz: Optional[float] = None
        self._mech = 0.0
        self._fault: Optional[str] = None
        self._sp = {'id_ref': 0.0, 'iq_ref': 0.0, 'theta': 0.0,
                    'omega_target': 0.0, 'accel': 0.0, 'vd': 0.0, 'vq': 0.0,
                    'pol_volts': 0.0, 'pol_periods': 0, 'pol_gap': 0}
        self._params = {}
        self._theta_hat = 0.7
        self._theta_hat_at = time.time()
        self._trigger = 2360
        self._mode_at = time.time()
        self._window_at = time.time()
        self._mom = None
        self._pol = (0.0, 0.0)
        self._cycles_max = 0
        self._source = 'adc'
        #: The virtual source's rotor, built on demand. The ADC source is
        #: what every existing caller uses and its rotor is deliberately
        #: still (see `model`), so a machine integrating in the background
        #: would be work nobody asked for.
        self._motor = None
        self._motor_at = 0.0
        self._motor_acc = 0.0
        #: The back-EMF chain, built on the first ask: a stand-in
        #: nobody asks for observers should not be integrating
        #: two of them in the background.
        self._obs = None
        self._obs_at = 0.0
        self._obs_frame = 0.0
        self._obs_flux_omega = 0.0
        self._obs_synced = None
        #: Whether the bridge is actually switching, asked of whatever
        #: owns the gates. None until the board wires it, and then the
        #: drive's own mode stands in - a drive with no stage behind it.
        self._switching: Optional[Callable[[], Any]] = None
        #: What the thermal envelope is scaling the clamp by, 1 to 0.
        self._derate = 1.0
        self._omega_hat = 0.0
        self._model = {'r': self.R, 'ld': self.LD, 'lq': self.LQ,
                       'lambda': self.LAMBDA, 'pole_pairs': float(self.POLES),
                       'sat': self.SAT, 'i_sat': self.I_SAT, 'j': 2e-5,
                       'b': 1e-5, 'load': 0.0, 'v_dt': self.V_DT,
                       'i_knee': self.I_KNEE, 'vdc': DCBUS_V, 'noise': 0.0,
                       'theta0': 0.0, 'sub': 4.0}

    @_rotor_locked
    def state(self):
        if self._source == 'model':
            self._read_model()                   # the rotor up to now, first
        self._converge()
        iid, iq, vd, vq = self._dq()
        ih, eps_amps = self._ih()
        periods = self._periods_since(self._mode_at)
        if self._mode == 'polarity':
            self._settle_polarity(periods)
        gain = self._p('drv_eps_gain_ua_per_rad', 0.0)
        return {
            'mode': self._mode, 'fault': self._fault,
            # THE BRIDGE, NOT THE MODE.
            'stage_enabled': bool(self._switching()) if self._switching
                             else self._mode != 'off',
            'afe_on': True,
            'injecting': bool(self._p('drv_inj_mv', 0.0)) and self._mode in ('hold', 'sensorless'),
            'owns_compares': self._mode != 'off', 'sync_armed': True,
            # On the model source the observer is the tracker that follows the
            # virtual rotor - a speed loop over omega_hat read 0.0 for ever
            # while the rotor did 8600 rad/s.
            'theta_hat': self._theta_hat,
            'omega_hat': (self._omega_hat if self._source == 'model'
                          else 0.0),
            'theta_cmd': (self._sp['theta']
                          + self._omega() * (time.time() - self._mode_at)) % (2 * math.pi),
            'omega_cmd': self._omega(),
            'id': iid, 'iq': iq, 'vd': vd, 'vq': vq, 'vdc': DCBUS_V,
            'eps': (eps_amps / gain) if gain else 0.0, 'eps_amps': eps_amps,
            'ih': ih, 'e_bemf': 0.0, 'periods': periods,
            'isr_cycles_last': 1450, 'isr_cycles_max': max(self._cycles_max, 1620),
            'pol_pos': self._pol[0], 'pol_neg': self._pol[1],
            'trigger': self._trigger, 'ts': self.TS,
            # The MINOR 2 appendix the board's op 0 carries - absent here,
            # rotor_observer_session read a KeyError off the stand-in.
            'exit_ticks_max': 2921,
            'cycles': {'sample': 610, 'step': 1690, 'advance': 620},
        }

    @_rotor_locked
    def _set_mode(self, name):
        if name not in MODES:
            raise ValueError('%r is not a mode; they are %s' % (name, ', '.join(MODES)))
        if name == 'polarity' and not self._sp['pol_periods']:
            raise RigError('polarity needs pol_periods above zero - one pulse '
                           'of no length measures nothing (simulated)')
        # INTEGRATE, THEN CHANGE.
        if self._source == 'model':
            self._read_model()
        self._mode = name
        self._fault = None
        self._mode_at = time.time()
        if name != 'off':
            self._pol = (0.0, 0.0)
        return True

    @_rotor_locked
    def _set_setpoints(self, **values):
        for name in values:
            if name not in self._sp:
                raise ValueError('%r is not a setpoint; they are %s' % (name, ', '.join(self._sp)))
        if self._source == 'model':
            self._read_model()                       # the old command's time, first
        self._sp.update({k: float(v) for k, v in values.items()})
        return dict(values)

    def setpoints(self):
        return dict(self._sp)

    @_rotor_locked
    def set_theta(self, radians):
        self._theta_hat = radians % (2 * math.pi)
        self._theta_hat_at = time.time()
        return True

    @_rotor_locked
    def _set_source(self, name):
        if name not in SOURCES:
            raise ValueError('%r is not a source; they are %s' % (name, ', '.join(SOURCES)))
        if self._mode != 'off':
            raise RigError('the drive is running - mode 0 first, then change '
                           'where its samples come from (simulated)')
        self._source = name
        return True

    #: What an uncommissioned board answers: the firmware's compiled-in
    #: placeholders (board_cal.c), in SI, the same as the real record reads.
    DEFAULTS = {
        'motor_r_uohm': 0.05, 'motor_ld_nh': 20e-6, 'motor_lq_nh': 25e-6,
        'motor_lambda_uvs': 0.005, 'motor_pole_pairs': 7.0,
        'drv_kp_mv_per_a': 0.1, 'drv_ki_v_per_as': 250.0,
        'drv_l1_milli': 0.1, 'drv_l2_milli': 100.0,
        'drv_inj_mv': 0.0, 'drv_inj_periods': 1.0, 'drv_inj_phase_mrad': 0.0,
        'drv_eps_gain_ua_per_rad': 0.0, 'drv_i_max_ma': 5.0,
        'drv_i_trip_ma': 100.0, 'drv_v_frac_ppm': 0.95, 'drv_sign': 1.0,
        'drv_w_lo_mrad_s': 60.0, 'drv_w_hi_mrad_s': 120.0,
        'drv_dt_step_ma': 1.0, 'drv_sigma_i_ua': 0.0, 'drv_trigger_ticks': 0.0,
    }

    def params(self):
        return {name: self._params.get(name, self.DEFAULTS.get(name, 0.0))
                for name in PARAMS}

    def _write_params(self, **values):
        for name in values:
            if name not in PARAMS:
                raise ValueError('%r is not a drive parameter; they are %s'
                                 % (name, ', '.join(PARAMS)))
        self._params.update({k: float(v) for k, v in values.items()})
        if 'drv_trigger_ticks' in values and values['drv_trigger_ticks']:
            self._trigger = int(values['drv_trigger_ticks'])
        return dict(values)
