"""The control law's stand-in rotor: one rotor, one lock, and the
spring/inertia physics every motion verb is tested against."""
import functools
import math
import random
import threading
import time

from ..errors import RigError
from ..motor import BENCH_MOTOR, Motor
from .values import DCBUS_V, NOMINAL


def _rotor_locked(method):
    """Run a rotor-touching method under `self._lock`.

    The whole body, so 'advance to now, then apply the input' is atomic:
    no other thread's advance lands between the two. Re-entrant, so a
    guarded reader (`state`) calling a guarded advancer (`model`) is fine.
    """
    @functools.wraps(method)
    def wrapped(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return wrapped


class SimulatedDrive:
    """The control law's device without a motor: a locked rotor at zero
    electrical angle, a phase resistance, two inductances that bend with
    the d current, a flux linkage and a dead-time voltage error - enough
    for every commissioning step to recover a number it can check against
    the constants below. Time runs at the PWM rate off the wall clock, so
    a window or a moments run fills at 50 kHz.
    """

    #: One definition, in `coaxial.motor`, so this machine cannot drift
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
    SIGMA_I = 0.02      #: current noise on the shunts, A rms
    TS = 20e-6
    FS = 50000.0
    #: Raw codes at rest - THE SAME rest point every other path reads
    #: (`NOMINAL`), U, V, W and the DC link. It was a second table with
    #: its own numbers, and `offsets()` tared through this one while the
    #: DAQ records sat on the other: -500 codes of phantom current on
    #: every phase of a recorded frame.
    CENTRE = (NOMINAL[0], NOMINAL[1], NOMINAL[2], NOMINAL[5])
    NOISE = (24.0, 22.0, 25.0, 8.0)              #: codes rms at the quiet point
    BEST_TRIGGER = 2300                          #: where the pickup is least
    PERIOD = 2376
    #: Amps per code, the stand-in's own scaling (scaling.PHASE_ONBOARD):
    #: 3.3 V over 32768 codes, through 3.5 mohm x 4.5455.
    APC = 3.3 / 32768.0 / (0.0035 * 1500.0 / 330.0)
    #: The shunt chains' gain mismatch the commissioning has to find.
    GAIN = (1.0, 1.01, 0.995)

    def __init__(self):
        # ONE ROTOR, TWO THREADS. The DAQ reader thread reaches this rotor
        # through the shaft sensor (`SimulatedAngle._turn` -> `model()`)
        # while a motion loop on the main thread calls `state`/`setpoint`
        # here - both funnel into `_advance_model`, a read-modify-write of
        # `_motor_at` and the rotor. Two interleaved advances each compute
        # the same `dt` and the second clobbers: the glitch the position
        # notebook worked around by single-threading its trace. Re-entrant
        # because the guarded readers call the guarded advancers - the
        # same shape as `Transport.request`'s lock on the real wire.
        self._lock = threading.RLock()
        self._mode = 'off'
        self._fault = None
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
        self._omega_hat = 0.0
        self._model = {'r': self.R, 'ld': self.LD, 'lq': self.LQ,
                       'lambda': self.LAMBDA, 'pole_pairs': float(self.POLES),
                       'sat': self.SAT, 'i_sat': self.I_SAT, 'j': 2e-5,
                       'b': 1e-5, 'load': 0.0, 'v_dt': self.V_DT,
                       'i_knee': self.I_KNEE, 'vdc': DCBUS_V, 'noise': 0.0,
                       'theta0': 0.0, 'sub': 4.0}

    def _p(self, name, default):
        return self._params.get(name, default)

    def _ld(self, id_bias):
        return self.LD * (1.0 - self.SAT * math.tanh(id_bias / self.I_SAT))

    def _dt(self, amps):
        return self.V_DT * math.tanh(amps / self.I_KNEE)

    def _periods_since(self, at):
        return int((time.time() - at) * self.FS)

    def _omega(self):
        return self._sp['omega_target'] if self._mode == 'hold' else 0.0

    def _dq(self):
        """The loop's dq means for the mode it is in."""
        if self._mode == 'volt':
            iid = self._sp['vd'] / self.R * self._p('drv_sign', 1.0)
            return iid, self._sp['vq'] / self.R, self._sp['vd'], self._sp['vq']
        if self._mode not in ('hold', 'sensorless'):
            return 0.0, 0.0, 0.0, 0.0
        i_max = self._p('drv_i_max_ma', 5.0)
        iid = max(-i_max, min(i_max, self._sp['id_ref']))
        iq = max(-i_max, min(i_max, self._sp['iq_ref']))
        # The speed in the voltage solution: HOLD's is the command's,
        # SENSORLESS on the model is the tracker's - it was 0.0 there,
        # vq lost its back-EMF term, and a power measurement read
        # 0.34 W where the shaft alone carried 34.
        omega = self._omega()
        if self._mode == 'sensorless' and self._source == 'model':
            omega = self._omega_hat
        vd = self.R * iid + (2.0 / 3.0) * (self._dt(iid) + self._dt(iid / 2.0)) \
            - omega * self.LQ * iq
        vq = self.R * iq + omega * self._ld(iid) * iid + omega * self.LAMBDA
        return iid, iq, vd, vq

    def _ih(self):
        """The demodulated HF current step: V.T over the inductance along
        the injection axis, with the rotor at zero and the frame at
        `theta` (HOLD) or the rotor observer's estimate (SENSORLESS)."""
        v_inj = self._p('drv_inj_mv', 0.0)
        if not v_inj or self._mode not in ('hold', 'sensorless'):
            return 0.0, 0.0
        frame = self._sp['theta'] if self._mode == 'hold' else self._theta_hat
        phi = frame + self._p('drv_inj_phase_mrad', 0.0)
        iid = self._sp['id_ref'] if self._mode == 'hold' else 0.0
        ld = self._ld(iid)
        l_sum, l_del = (ld + self.LQ) / 2.0, (ld - self.LQ) / 2.0
        inv = (l_sum - l_del * math.cos(2.0 * phi)) / (ld * self.LQ)
        ih = v_inj * self.TS * inv
        eps = v_inj * self.TS * l_del * math.sin(2.0 * phi) / (ld * self.LQ)
        return ih, -eps

    def _converge(self):
        """SENSORLESS pulls theta_hat onto the rotor (0) or pi off it."""
        if self._mode != 'sensorless' or not self._p('drv_inj_mv', 0.0):
            return
        dt = time.time() - self._theta_hat_at
        self._theta_hat_at = time.time()
        target = 0.0 if math.cos(self._theta_hat) >= 0.0 else math.pi
        err = (self._theta_hat - target + math.pi) % (2 * math.pi) - math.pi
        self._theta_hat = (target + err * math.exp(-dt * 60.0)) % (2 * math.pi)

    @_rotor_locked
    def state(self):
        if self._source == 'model':
            self.model()                   # the rotor up to now, first
        self._converge()
        iid, iq, vd, vq = self._dq()
        ih, eps_amps = self._ih()
        periods = self._periods_since(self._mode_at)
        if self._mode == 'polarity':
            need = 2 * self._sp['pol_periods'] + 2 * self._sp['pol_gap'] + 4
            if periods >= need:
                aligned = math.cos(self._theta_hat) >= 0.0
                big, small = 34.0, 18.0
                self._pol = (big, small) if aligned else (small, big)
                self._mode = 'off'
        gain = self._p('drv_eps_gain_ua_per_rad', 0.0)
        return {
            'mode': self._mode, 'fault': self._fault,
            'stage_enabled': self._mode != 'off', 'afe_on': True,
            'injecting': bool(self._p('drv_inj_mv', 0.0)) and self._mode in ('hold', 'sensorless'),
            'owns_compares': self._mode != 'off', 'sync_armed': True,
            # On the model source the observer is the tracker that
            # follows the virtual rotor - a speed loop over omega_hat
            # read 0.0 for ever while the rotor did 8600 rad/s.
            'theta_hat': self._theta_hat,
            'omega_hat': (self._omega_hat if self._source == 'model'
                          else 0.0),
            'theta_cmd': (self._sp['theta'] + self._omega() * (time.time() - self._mode_at)) % (2 * math.pi),
            'omega_cmd': self._omega(),
            'id': iid, 'iq': iq, 'vd': vd, 'vq': vq, 'vdc': DCBUS_V,
            'eps': (eps_amps / gain) if gain else 0.0, 'eps_amps': eps_amps,
            'ih': ih, 'e_bemf': 0.0, 'periods': periods,
            'isr_cycles_last': 1450, 'isr_cycles_max': max(self._cycles_max, 1620),
            'pol_pos': self._pol[0], 'pol_neg': self._pol[1],
            'trigger': self._trigger, 'ts': self.TS,
            # The MINOR 2 appendix the board's op 0 carries - absent here,
            # rotor_observer_session read a KeyError off the stand-in.
            # The numbers are the bench's own measured shape (FINDINGS,
            # *The caches were off*): invented, like everything above.
            'exit_ticks_max': 2921,
            'cycles': {'sample': 610, 'step': 1690, 'advance': 620},
        }

    @_rotor_locked
    def mode(self, name):
        from ..drive import MODES
        if name not in MODES:
            raise ValueError('%r is not a mode; they are %s' % (name, ', '.join(MODES)))
        if name == 'polarity' and not self._sp['pol_periods']:
            raise RigError('polarity needs pol_periods above zero - one pulse '
                           'of no length measures nothing (simulated)')
        # INTEGRATE, THEN CHANGE. The rotor advances lazily, so the time
        # up to this input change belongs to the OLD mode and command - a
        # stepper that wrote 180 setpoints and read once handed the rotor
        # one 45-degree leap and a pole slip instead of a slew.
        if self._source == 'model':
            self.model()
        self._mode = name
        self._fault = None
        self._mode_at = time.time()
        if name != 'off':
            self._pol = (0.0, 0.0)
        return True

    def off(self):
        return self.mode('off')

    @_rotor_locked
    def setpoint(self, **values):
        for name in values:
            if name not in self._sp:
                raise ValueError('%r is not a setpoint; they are %s' % (name, ', '.join(self._sp)))
        if self._source == 'model':
            self.model()                       # the old command's time, first
        self._sp.update({k: float(v) for k, v in values.items()})
        return dict(values)

    def setpoints(self):
        return dict(self._sp)

    @_rotor_locked
    def set_theta(self, radians):
        self._theta_hat = radians % (2 * math.pi)
        self._theta_hat_at = time.time()
        return True

    def window(self):
        self._converge()
        n = max(1, self._periods_since(self._window_at))
        self._window_at = time.time()
        iid, iq, vd, vq = self._dq()
        ih, eps_amps = self._ih()
        gain = self._p('drv_eps_gain_ua_per_rad', 0.0)
        n_inj = self._p('drv_inj_periods', 1.0) or 1.0
        sd_eps = (self.SIGMA_I / n_inj / gain) if gain else 0.0
        fields = {
            'id': {'n': n, 'mean': iid, 'sd': self.SIGMA_I},
            'iq': {'n': n, 'mean': iq, 'sd': self.SIGMA_I},
            'vd': {'n': n, 'mean': vd, 'sd': 0.01},
            'vq': {'n': n, 'mean': vq, 'sd': 0.01},
            'eps': {'n': n // 2, 'mean': 0.0, 'sd': sd_eps},
            'ih': {'n': n // 2, 'mean': ih, 'sd': self.SIGMA_I / math.sqrt(2)},
            'vdc': {'n': n, 'mean': DCBUS_V, 'sd': 0.003},
        }
        rng = random.Random(n)
        return {'n': n, 'fields': fields,
                'rho': [rng.uniform(-0.008, 0.008) for _ in range(7)],
                'i_peak': math.hypot(iid, iq) + 3 * self.SIGMA_I}

    def _phase_amps(self):
        """The held current vector as the three phase currents, with the
        gain mismatch each shunt chain puts on its own."""
        iid, iq, _, _ = self._dq()
        if self._mode not in ('hold', 'volt'):
            return (0.0, 0.0, 0.0)
        th = self._sp['theta']
        ia = iid * math.cos(th) - iq * math.sin(th)
        ib = iid * math.sin(th) + iq * math.cos(th)
        return (ia, -0.5 * ia + 0.8660254 * ib, -0.5 * ia - 0.8660254 * ib)

    def _pickup(self):
        """Switching pickup at the sample point: a bump mid-period, where a
        50 % edge sits, and least near the top of the triangle."""
        x = (self._trigger - self.PERIOD / 2.0) / (self.PERIOD / 4.0)
        return 90.0 * math.exp(-x * x)

    def moments_arm(self, periods):
        self._mom = (time.time(), int(periods))
        return True

    def moments(self):
        if self._mom is None:
            n, want = 0, 0
        else:
            n = min(self._periods_since(self._mom[0]), self._mom[1])
            want = self._mom[1]
        rng = random.Random(self._trigger + n)
        channels = {}
        iabc = self._phase_amps()
        for k, name in enumerate(('Phase U', 'Phase V', 'Phase W', 'DC bus')):
            sd = self.NOISE[k] + (self._pickup() if k < 3 else 0.0)
            mean = self.CENTRE[k] + rng.gauss(0.0, sd / math.sqrt(max(n, 1)))
            if k < 3:
                mean += self.GAIN[k] * iabc[k] / self.APC
            channels[name] = {'mean': mean, 'sd': sd,
                              'lo': int(mean - 3.5 * sd), 'hi': int(mean + 3.5 * sd)}
        return {'done': bool(want) and n >= want, 'n': n, 'want': want,
                'trigger': self._trigger, 'channels': channels}

    def moments_run(self, periods, timeout=5.0, poll=0.02):
        self.moments_arm(periods)
        deadline = time.time() + timeout
        while True:
            got = self.moments()
            if got['done']:
                return got
            if time.time() > deadline:
                raise RigError('%d of %d periods counted (simulated)' % (got['n'], periods))
            time.sleep(poll)

    def reload(self):
        return True

    def reset_cycles(self):
        self._cycles_max = 0
        return True

    def trigger(self, ticks):
        """The stand-in's sample point, moved by its gate drivers' trigger()."""
        self._trigger = int(ticks)

    @_rotor_locked
    def source(self, name):
        from ..drive import SOURCES
        if name not in SOURCES:
            raise ValueError('%r is not a source; they are %s' % (name, ', '.join(SOURCES)))
        if self._mode != 'off':
            raise RigError('the drive is running - mode 0 first, then change '
                           'where its samples come from (simulated)')
        self._source = name
        return True

    @_rotor_locked
    def model_param(self, **values):
        from ..drive import MODEL_IDS
        for name in values:
            if name not in MODEL_IDS:
                raise ValueError('%r is not a model parameter; they are %s'
                                 % (name, ', '.join(MODEL_IDS)))
        if self._source == 'model':
            self.model()                   # the old parameters' time, first
        self._model.update({k: float(v) for k, v in values.items()})
        # The RUNNING rotor too, as the firmware's own model applies them:
        # writing `load` mid-hold reached only the dict, and the servo's
        # sag demo measured nothing because nothing sagged. The map is
        # explicit - `ld` is a METHOD on Motor (`ld0` holds the number)
        # and a guessed setattr would shadow it.
        if self._motor is not None:
            live = {'r': 'r', 'ld': 'ld0', 'lq': 'lq', 'lambda': 'lam',
                    'sat': 'sat', 'i_sat': 'i_sat', 'j': 'j', 'b': 'b',
                    'load': 'load', 'v_dt': 'v_dt', 'i_knee': 'i_knee'}
            for k, v in values.items():
                if k in live:
                    setattr(self._motor, live[k], float(v))
        return dict(values)

    def _pll_hz(self):
        """The natural frequency the loaded PLL gains imply.

        `pll_gains` puts l2 = wn^2 * t_update at t_update = 2 Ts, so wn
        comes back out of l2. A caller that tightens the observer sees a
        tighter estimate here, which is the only reason this reads the
        parameters at all rather than picking a number.
        """
        l2 = self._p('drv_l2_milli', 100.0)
        wn = math.sqrt(max(l2, 1e-9) / (2.0 * self.TS))
        return min(max(wn / (2.0 * math.pi), 1.0), 5000.0)

    def _advance_model(self):
        """Turn the virtual rotor by the torque the dq solution makes.

        MECHANICS ONLY, on purpose: `_dq()` is the electrical steady state
        the loop settled at, so integrating the electrical equations again
        would solve them twice. The rotor is what was missing - `model()`
        used to answer theta 0.0 and omega_hat 0.0, so nothing reading this
        stand-in could see an observer track anything.

        `theta_hat` follows the rotor through a one-pole lag at the PLL's
        own natural frequency. It is NOT the firmware's observer - that is
        C, and `tools/observer_run.py` is what runs the real one. It is a
        lag with the right bandwidth, so a chain built against this
        stand-in exercises its own arithmetic and not a fabricated error.
        """
        motor = self._machine()
        now = time.time()
        dt = min(now - self._motor_at, 0.25)     # bounded catch-up
        self._motor_at = now
        if dt <= 0.0:
            return motor
        if self._mode != 'off':
            iid, iq, _, _ = self._dq()
            ld = self._ld(iid)
            # TORQUE BY MODE. SENSORLESS commutates on the rotor, so iq is
            # torque current. HOLD commutates on the COMMANDED angle - a
            # stepper - and the rotor is dragged by the load-angle spring
            # `kt i sin(cmd - theta)`: it follows a slewed command, rings
            # after a step as a stepper does, and slips a pole if the
            # spring is overpowered, which is what a stepper is.
            hold = self._mode == 'hold'
            k_t = 1.5 * motor.p * motor.lam
            i_mag = math.hypot(iid, iq)
            if not hold:
                torque = 1.5 * motor.p * (motor.lam * iq
                                          + (ld - motor.lq) * iid * iq)
            cmd = (self._sp['theta']
                   + self._omega() * (now - dt - self._mode_at))
            w_cmd = self._omega()
            wm = motor.omega / motor.p
            # SUBSTEPPED, SYMPLECTIC. One Euler step over a poll gap
            # diverges: (1 - dt b/j) at the placeholder profile is -4 at a
            # 0.2 s poll and the rotor read +1896, -5770, +24964 rad/s on
            # three of them. Each slice stays a tenth of the mechanical
            # constant AND a twentieth of the spring's period; speed then
            # angle keeps the spring bounded rather than spiralling.
            step = min(0.002, 0.1 * motor.j / max(motor.b, 1e-12))
            if hold and i_mag > 0.0:
                spring = 1.5 * motor.p * motor.p * motor.lam * i_mag
                step = min(step, 0.3 * math.sqrt(motor.j / spring))
            n = max(1, int(math.ceil(dt / step)))
            h = dt / n
            theta = motor.theta
            for _ in range(n):
                if hold:
                    cmd += w_cmd * h
                    torque = k_t * i_mag * math.sin(cmd - theta)
                wm += (torque - motor.b * wm - motor.load) / motor.j * h
                theta += wm * motor.p * h
            # The SHAFT, accumulated: electrical theta wraps at 2 pi and a
            # shaft sensor reads the mechanical angle, which is 1/p of the
            # whole unwrapped travel - `SimulatedAngle` reads this.
            self._mech = (getattr(self, '_mech', 0.0)
                          + (theta - motor.theta) / motor.p)
            motor.omega = wm * motor.p
            motor.theta = theta % (2.0 * math.pi)
        # THE LAG IS CLOSED FORM, NOT INTEGRATED. A one-pole decay over dt
        # was tried first and read exactly 0.0000 rad: a caller polling
        # 50 ms apart is twelve PLL time constants apart, so the lag had
        # always fully settled and the field was useless. A type-2 PLL
        # tracking constant acceleration settles at alpha / wn^2 instead -
        # zero error at constant speed, growing with acceleration, and
        # independent of how often anyone asks.
        wn = 2.0 * math.pi * self._pll_hz()
        alpha = (motor.omega - self._omega_hat) / dt if dt > 0.0 else 0.0
        self._omega_hat = motor.omega
        self._theta_hat = (motor.theta + alpha / (wn * wn)) % (2.0 * math.pi)
        return motor

    def _machine(self):
        if self._motor is None:
            m = self._model
            self._motor = Motor(r=m['r'], ld=m['ld'], lq=m['lq'],
                                lam=m['lambda'], p=int(m['pole_pairs']),
                                j=m['j'], b=m['b'], load=m['load'],
                                sat=m['sat'], i_sat=m['i_sat'],
                                v_dt=m['v_dt'], i_knee=m['i_knee'],
                                theta=m['theta0'])
            self._motor_at = time.time()
        return self._motor

    @_rotor_locked
    def model(self):
        """The virtual source's rotor, or a still one on the ADC source.

        WHY THE ADC SOURCE ANSWERS ZERO. Its rotor is what the stand-in's
        dq means and its saliency are drawn around, and every commissioning
        step is checked against that - a machine that started turning under
        them would change what they recover. The rotor turns when the
        source is the model, which is what the model source is for.
        """
        iid, iq, _, _ = self._dq()
        if self._source != 'model':
            return {'source': self._source, 'theta': 0.0,
                    'omega': self._omega(), 'id': iid, 'iq': iq,
                    'vdc': self._model['vdc']}
        motor = self._advance_model()
        err = ((self._theta_hat - motor.theta + math.pi)
               % (2.0 * math.pi) - math.pi)
        return {'source': self._source, 'theta': motor.theta,
                'omega': motor.omega, 'id': iid, 'iq': iq,
                'vdc': self._model['vdc'],
                'theta_hat': self._theta_hat, 'omega_hat': self._omega_hat,
                'error': err}

    @_rotor_locked
    def model_reset(self):
        """The rotor back to theta0, at rest - the contract `drive.py` states."""
        self._motor = None
        self._omega_hat = 0.0
        self._theta_hat = self._model['theta0']
        return True

    def profile(self, path):
        import json
        with open(path, encoding='utf-8') as handle:
            data = json.load(handle)
        done = {'name': data.get('name', path)}
        if data.get('drive'):
            done['drive'] = self.set_params(**data['drive'])
        if data.get('model'):
            done['model'] = self.model_param(**data['model'])
        return done

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
        from ..drive import PARAMS
        return {name: self._params.get(name, self.DEFAULTS.get(name, 0.0))
                for name in PARAMS}

    def set_params(self, **values):
        from ..drive import PARAMS
        for name in values:
            if name not in PARAMS:
                raise ValueError('%r is not a drive parameter; they are %s' % (name, ', '.join(PARAMS)))
        self._params.update({k: float(v) for k, v in values.items()})
        if 'drv_trigger_ticks' in values and values['drv_trigger_ticks']:
            self._trigger = int(values['drv_trigger_ticks'])
        return dict(values)
