"""The stand-in drive: the board's law on its motor in C (world_drive.c), this its edge."""
import math
import threading
import time
from typing import Callable, Optional, Any

from coaxial.devices.drive import (DT_ROWS, FAULTS, MODE_NAMES, MODES, PARAMS, SOURCES,
                                   DriveControl)
from coaxial.errors import RigError
from coaxial.simulated.drive.capture import DriveCapture
from coaxial.simulated.drive.locked import _rotor_locked
from coaxial.simulated.drive.plant import DrivePlant
from coaxial.simulated.slots import Slots
from coaxial.simulated.values import DCBUS_V
from motor.catalog import BENCH_MOTOR
from motor.pmsm import WINDING_J_PER_K, WINDING_K_PER_W
from tools.cores import stand

#: world_drive.c's slots.
_DRIVES = Slots(stand.SLOTS)

#: The most of the wall's time the law is run at one request, s: a board nobody asked for
#: longer stood still that long. Under any procedure's wait (commissioning's ramp, 1.05 s);
#: a second of it is 14 ms here.
CATCH_UP_S = 2.0

#: The record's drive parameters by the law's names (drive_params_t, harness.c's PARAM_ORDER).
LAW = {'motor_r': 'r', 'motor_ld': 'ld', 'motor_lq': 'lq', 'motor_lambda': 'lambda',
       'motor_pole_pairs': 'pole_pairs', 'drv_kp': 'kp', 'drv_ki': 'ki', 'drv_l1': 'l1',
       'drv_l2': 'l2', 'drv_inj_volts': 'inj_volts', 'drv_inj_periods': 'inj_periods',
       'drv_inj_phase': 'inj_phase', 'drv_eps_gain': 'eps_gain', 'drv_i_trip': 'i_trip',
       'drv_v_frac': 'v_frac', 'drv_w_lo': 'w_lo', 'drv_w_hi': 'w_hi',
       'drv_dt_step': 'dt_step'}


class SimulatedDrive(DrivePlant, DriveCapture, DriveControl):
    """The control law's device without a board: drive.c's law, a PWM period at a time, on
    drive_model.c's motor - the bench motor below, free on its bearings behind the stage -, so
    every commissioning step recovers a number it can check against these constants.
    """

    #: One definition, in `motor.catalog`, so this motor cannot drift
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
    #: Where the stage switches: the `.ioc`'s ARR 2375 off 237.5 MHz.
    PWM_HZ = 50e3
    #: One control period, s, and how many a second - what a window or a moments run fills at.
    TS = 1.0 / PWM_HZ
    FS = PWM_HZ
    #: ARR + 1, which is what `Board_PwmPeriod` answers.
    PERIOD = int(round(CLK / (2.0 * PWM_HZ))) + 1
    BEST_TRIGGER = 2300                          #: where the pickup is least

    #: Where the estimate starts, rad: off the rotor at theta0, as a board's is at power-up.
    THETA_HAT = 0.7

    def __init__(self):
        # One rotor, two threads.
        self._lock = threading.RLock()
        self._slot = _DRIVES.take(self)
        self._lib = stand.library()
        self._law = stand.law(self._slot, self.TS)
        self._sp = {'id_ref': 0.0, 'iq_ref': 0.0, 'theta': 0.0,
                    'omega_target': 0.0, 'accel': 0.0, 'vd': 0.0, 'vq': 0.0,
                    'pol_volts': 0.0, 'pol_periods': 0, 'pol_gap': 0}
        self._params = {}
        self._source = 'adc'
        #: The wall's clock the law stands at, the part of a period it is owed, and the
        #: seconds a world stepped on its own clock told it (`paced`), None on the wall's.
        self._at = time.time()
        self._owed = 0.0
        self._pace: Optional[float] = None
        self._trigger = 2360
        self._mom = None
        self._cycles_max = 0
        #: The board wires these: MOE and whether the bridge switches on it, asked of whatever
        #: owns the gates - none, a drive with no stage behind it, armed with its modes -, the
        #: triple armed at a mode as Board_DriveSetMode arms it, the AFE's rail, what a trip
        #: drops, the record's raw parameters.
        self._moe: Optional[Callable[[], Any]] = None
        self._switching: Optional[Callable[[], Any]] = None
        self._arm_sync: Optional[Callable[[], Any]] = None
        self._sync_armed: Callable[[], bool] = lambda: True
        self._afe_on: Callable[[], bool] = lambda: True
        self._drop: Optional[Callable[[], Any]] = None
        self._record: Callable[[], dict] = dict
        #: What the thermal envelope is scaling the clamp by, 1 to 0.
        self._derate = 1.0
        #: The sync's last window: the phases' mean squares, the link's mean amps.
        self._counted = ((0.0, 0.0, 0.0), 0.0)
        self._model = {'r': self.R, 'ld': self.LD, 'lq': self.LQ,
                       'lambda': self.LAMBDA, 'pole_pairs': float(self.POLES),
                       'sat': self.SAT, 'i_sat': self.I_SAT, 'j': 2e-5,
                       'b': 1e-5, 'load': 0.0, 'v_dt': self.V_DT,
                       'i_knee': self.I_KNEE, 'vdc': DCBUS_V, 'noise': 0.0,
                       'theta0': 0.0, 'sub': 4.0}
        self._law.model_params(**self._model)
        self._lib.world_drive_rest(self._slot)
        self._law.set_theta(self.THETA_HAT)
        self._load()

    # -- the clock --------------------------------------------------------

    def paced(self, dt):
        """The law run `dt` s more at its next request, and by nothing but what it is told
        from then on - a world stepped on its own clock tells each pass's; None, the wall's
        clock again. On the wall's a pass 0.25 s late turned the rotor those seconds under a
        setpoint meant for 50 ms: QUAD's frame, stepped its 50, passed a gate 2.7 m off under
        the gate's load."""
        with self._lock:
            self._pace = None if dt is None else (self._pace or 0.0) + float(dt)
            self._at = time.time()

    def _stage(self):
        """(MOE, the bridge switching on it), as whatever owns the gates has them."""
        supplied = bool(self._switching()) if self._switching else True
        return (bool(self._moe()) if self._moe else supplied), supplied

    def _advance(self):
        """The law brought to now, in whole periods: the wall's seconds since it last was,
        CATCH_UP_S at most, or what its world told; a trip drops the stage."""
        now = time.time()
        if self._pace is None:
            dt = max(0.0, min(now - self._at, CATCH_UP_S))
        else:
            dt, self._pace = self._pace, 0.0
        self._at = now
        whole = self._owed + dt * self.FS
        periods = int(whole)
        self._owed = whole - periods
        if periods and self._lib.world_drive_run(self._slot, periods, *self._stage()):
            if self._drop is not None:
                self._drop()

    def _now(self):
        """(the law's state, its slot's row) at this moment."""
        self._advance()
        return self._law.state(), stand.row(self._slot)

    # -- the record -------------------------------------------------------

    #: What an uncommissioned board answers: the firmware's compiled-in
    #: placeholders (board_cal.c), in SI, the same as the real record reads.
    DEFAULTS = {
        'motor_r': 0.05, 'motor_ld': 20e-6, 'motor_lq': 25e-6,
        'motor_lambda': 0.005, 'motor_pole_pairs': 7.0,
        'drv_kp': 0.1, 'drv_ki': 250.0,
        'drv_l1': 0.1, 'drv_l2': 100.0,
        'drv_inj_volts': 0.0, 'drv_inj_periods': 1.0, 'drv_inj_phase': 0.0,
        'drv_eps_gain': 0.0, 'drv_i_max': 5.0,
        'drv_i_trip': 100.0, 'drv_v_frac': 0.95, 'drv_sign': 1.0,
        'drv_w_lo': 60.0, 'drv_w_hi': 120.0,
        'drv_dt_step': 1.0, 'drv_sigma_i': 0.0, 'drv_trigger_ticks': 0.0,
        # The record's winding, CAL_VERSION 12's defaults (board_cal.c).
        'winding_k_per_w': WINDING_K_PER_W, 'winding_j_per_k': WINDING_J_PER_K,
        'winding_limit_c': 120.0,
    }

    def params(self):
        return {name: self._params.get(name, self.DEFAULTS.get(name, 0.0))
                for name in PARAMS}

    def _load(self):
        """Board_DriveParamsFromCal: the record's parameters into the law, the clamp under the
        envelope's derate, the dead-time table's rows."""
        record, raw = self.params(), self._record()
        law = {name: record[key] for key, name in LAW.items()}
        law['i_max'] = record['drv_i_max'] * self._derate
        law['sign'] = -1.0 if record['drv_sign'] < 0.0 else 1.0
        law.update(('dt%d' % k, raw.get(row, 0) / 1e3) for k, row in enumerate(DT_ROWS))
        self._law.params(**law)

    @_rotor_locked
    def _write_params(self, **values):
        for name in values:
            if name not in PARAMS:
                raise ValueError('%r is not a drive parameter; they are %s'
                                 % (name, ', '.join(PARAMS)))
        self._advance()
        self._params.update({k: float(v) for k, v in values.items()})
        if 'drv_trigger_ticks' in values and values['drv_trigger_ticks']:
            self._trigger = int(values['drv_trigger_ticks'])
        self._load()
        return dict(values)

    @_rotor_locked
    def reload(self):
        self._advance()
        self._load()
        return True

    def _clamp(self):
        """Board_DriveClamp: the clamp as the record has it, A."""
        return self._params.get('drv_i_max', self.DEFAULTS.get('drv_i_max', 0.0))

    @_rotor_locked
    def _derate_to(self, factor):
        """Board_DriveDerate: the clamp scaled, 0..1 - never raised."""
        self._advance()
        self._derate = max(0.0, min(1.0, factor))
        self._load()

    # -- modes and setpoints ----------------------------------------------

    @_rotor_locked
    def state(self):
        law, row = self._now()
        return {
            'mode': MODE_NAMES[int(law['mode'])], 'fault': FAULTS[int(law['fault'])],
            # The bridge's state, not the mode's.
            'stage_enabled': self._stage()[1] if self._switching else law['mode'] != 0.0,
            'afe_on': self._afe_on(),
            'injecting': bool(row['injecting']),
            'owns_compares': bool(row['driven']), 'sync_armed': self._sync_armed(),
            'theta_hat': law['theta_hat'], 'omega_hat': law['omega_hat'],
            'theta_cmd': law['theta_cmd'], 'omega_cmd': law['omega_cmd'],
            'id': law['id'], 'iq': law['iq'], 'vd': law['vd'], 'vq': law['vq'],
            'vdc': law['vdc'],
            'eps': law['eps'], 'eps_amps': law['eps_amps'],
            'ih': law['ih'], 'e_bemf': law['e_bemf'], 'periods': int(row['periods']),
            # isr_cycles_last, isr_cycles_max (1620 its floor), exit_ticks_max
            # (op 0's MINOR 2) and cycles are the stand-in's constants, not
            # measurements.
            'isr_cycles_last': 1450, 'isr_cycles_max': max(self._cycles_max, 1620),
            'pol_pos': law['pol_pos'], 'pol_neg': law['pol_neg'],
            'trigger': self._trigger if self._sync_armed() else 0, 'ts': self.TS,
            'exit_ticks_max': 2921,
            'cycles': {'sample': 610, 'step': 1690, 'advance': 620},
            'turns': int(law['turns']),
        }

    @_rotor_locked
    def _set_mode(self, name):
        if name not in MODES:
            raise ValueError('%r is not a mode; they are %s' % (name, ', '.join(MODES)))
        self._advance()
        # Board_DriveSetMode: the triple armed for any mode but off, the record taken again.
        if name != 'off' and self._arm_sync is not None:
            self._arm_sync()
        self._load()
        why = self._law.mode(MODES[name], self._stage()[0], self._afe_on())
        if why:
            raise RigError('%s (simulated)' % why)
        return True

    @_rotor_locked
    def _set_setpoints(self, **values):
        for name in values:
            if name not in self._sp:
                raise ValueError('%r is not a setpoint; they are %s' % (name, ', '.join(self._sp)))
        self._advance()                              # the old command's time, first
        self._sp.update({k: float(v) for k, v in values.items()})
        self._law.setpoints(**self._sp)
        return dict(values)

    def setpoints(self):
        return dict(self._sp)

    @_rotor_locked
    def set_theta(self, radians):
        self._advance()
        self._law.set_theta(radians)
        return True

    @_rotor_locked
    def _set_source(self, name):
        if name not in SOURCES:
            raise ValueError('%r is not a source; they are %s' % (name, ', '.join(SOURCES)))
        if self._now()[0]['mode'] != 0.0:
            raise RigError('the drive is running - mode 0 first, then change '
                           'where its samples come from (simulated)')
        self._source = name
        self._law.source(name == 'model')
        self._lib.world_drive_rest(self._slot)
        return True

    @_rotor_locked
    def _read_observers(self):
        """The chain the firmware runs (drive_observer.c), as the law stepped it."""
        law, _row = self._now()
        obs = self._law.obs()
        return {'valid': bool(obs['valid']), 'theta': obs['theta'], 'omega': obs['omega'],
                'blend': obs['blend'],
                'dual_theta': obs['dual_theta'], 'dual_omega': obs['dual_omega'],
                'flux_theta': obs['flux_theta'], 'flux_omega': obs['flux_omega'],
                'lambda_hat': obs['lambda_hat'],
                'theta_hat': law['theta_hat'], 'omega_hat': law['omega_hat'],
                'blend_lo': obs['blend_lo'], 'blend_hi': obs['blend_hi'], 'wc': obs['wc'],
                'error': (obs['theta'] - law['theta_hat'] + math.pi) % math.tau - math.pi}
