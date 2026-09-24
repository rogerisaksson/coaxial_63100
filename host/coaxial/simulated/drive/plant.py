"""The stand-in's machine: the dq currents it carries, the sample, the rotor it turns."""
import math
import time

from coaxial.devices.drive import MODEL_IDS
from coaxial.model.motor import Motor
from coaxial.model.sensorless import TORQUE_FACTOR
from coaxial.simulated.drive.locked import _rotor_locked


class DrivePlant:

    """The PMSM under the stand-in's drive: currents, the HF step, the rotor's motion."""

    #: What the polarity pulse reads for the aligned and the opposed half
    #: - invented, like everything here, and told apart by size.
    POL_READINGS = (34.0, 18.0)

    #: Periods past the two pulses and two gaps before the sign is read.
    POL_SETTLE = 4

    def _p(self, name, default):
        return self._params.get(name, default)

    # THE MACHINE THIS STAND-IN IS PRETENDING TO BE.
    @property
    def _r(self):
        return self._model['r']

    @property
    def _lq(self):
        return self._model['lq']

    @property
    def _lam(self):
        return self._model['lambda']

    @property
    def _poles(self):
        return int(self._model['pole_pairs'])

    def _ld(self, id_bias):
        m = self._model
        return m['ld'] * (1.0 - m['sat'] * math.tanh(id_bias / m['i_sat']))

    def _dt(self, amps):
        """The inverter's dead-time voltage error at this current."""
        m = self._model
        return m['v_dt'] * math.tanh(amps / m['i_knee'])

    def _periods_since(self, at):
        return int((time.time() - at) * self.FS)

    def _omega(self):
        return self._sp['omega_target'] if self._mode == 'hold' else 0.0

    def _carrying(self):
        """(amps, electrical angle) the stator carries right now: the dq
        solution's magnitude, at the command's angle in HOLD and the
        tracked rotor's otherwise.
        """
        iid, iq, _vd, _vq = self._dq()
        amps = math.hypot(iid, iq)
        if self._mode == 'hold':
            theta = (self._sp['theta']
                     + self._omega() * (time.time() - self._mode_at))
        else:
            theta = self._theta_hat
        return amps, theta % (2.0 * math.pi)

    def _dq(self):
        """The loop's dq means for the mode it is in."""
        if self._mode == 'volt':
            iid = self._sp['vd'] / self._r * self._p('drv_sign', 1.0)
            return iid, self._sp['vq'] / self._r, self._sp['vd'], self._sp['vq']
        if self._mode not in ('hold', 'sensorless'):
            return 0.0, 0.0, 0.0, 0.0
        # THE CLAMP, AS THE ENVELOPE LEFT IT.
        i_max = self._p('drv_i_max_ma', 5.0) * self._derate
        iid = max(-i_max, min(i_max, self._sp['id_ref']))
        iq = max(-i_max, min(i_max, self._sp['iq_ref']))
        # The speed in the voltage solution: HOLD's is the command's,
        # SENSORLESS on the model is the tracker's - it was 0.0 there, vq lost
        # its back-EMF term, and a power measurement read 0.34 W where the
        # shaft alone carried 34.
        omega = self._omega()
        if self._mode == 'sensorless' and self._source == 'model':
            omega = self._omega_hat
        vd = self._r * iid + (2.0 / 3.0) * (self._dt(iid) + self._dt(iid / 2.0)) \
            - omega * self._lq * iq
        vq = self._r * iq + omega * self._ld(iid) * iid + omega * self._lam
        return iid, iq, vd, vq

    def sample(self):
        """What a sampler in the control interrupt would see, this period."""
        iid, iq, _, _ = self._dq()
        theta = self._sp['theta'] if self._mode == 'hold' else self._theta_hat
        cos, sin = math.cos(theta), math.sin(theta)
        alpha = iid * cos - iq * sin
        beta = iid * sin + iq * cos
        root3 = math.sqrt(3.0) / 2.0
        # SWITCHING IS THE BRIDGE'S ANSWER, NOT THE LOOP'S.
        on = self._switching() if self._switching else self._mode != 'off'
        return {'amps': (alpha, -0.5 * alpha + root3 * beta,
                         -0.5 * alpha - root3 * beta) if on
                        else (0.0, 0.0, 0.0),
                'switching': bool(on)}

    def _ih(self):
        """The demodulated HF current step: V.T over the inductance along
        the injection axis, with the rotor at zero and the frame at
        `theta` (HOLD) or the rotor observer's estimate (SENSORLESS).
        """
        v_inj = self._p('drv_inj_mv', 0.0)
        if not v_inj or self._mode not in ('hold', 'sensorless'):
            return 0.0, 0.0
        frame = self._sp['theta'] if self._mode == 'hold' else self._theta_hat
        phi = frame + self._p('drv_inj_phase_mrad', 0.0)
        iid = self._sp['id_ref'] if self._mode == 'hold' else 0.0
        ld = self._ld(iid)
        l_sum, l_del = (ld + self._lq) / 2.0, (ld - self._lq) / 2.0
        inv = (l_sum - l_del * math.cos(2.0 * phi)) / (ld * self._lq)
        ih = v_inj * self.TS * inv
        eps = v_inj * self.TS * l_del * math.sin(2.0 * phi) / (ld * self._lq)
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

    def _settle_polarity(self, periods):
        """The polarity pulse ends itself once its two pulses and two gaps have
        run, leaving the two readings the sign is told from.
        """
        need = (2 * self._sp['pol_periods'] + 2 * self._sp['pol_gap']
                + self.POL_SETTLE)
        if periods < need:
            return
        big, small = self.POL_READINGS
        aligned = math.cos(self._theta_hat) >= 0.0
        self._pol = (big, small) if aligned else (small, big)
        self._mode = 'off'

    #: The model parameters a running Motor takes live, by the attribute
    #: each is on it - `ld` is a METHOD on Motor and `ld0` holds the number.
    LIVE = {'r': 'r', 'ld': 'ld0', 'lq': 'lq', 'lambda': 'lam',
            'sat': 'sat', 'i_sat': 'i_sat', 'j': 'j', 'b': 'b',
            'load': 'load', 'v_dt': 'v_dt', 'i_knee': 'i_knee'}

    @_rotor_locked
    def model_param(self, **values):
        for name in values:
            if name not in MODEL_IDS:
                raise ValueError('%r is not a model parameter; they are %s'
                                 % (name, ', '.join(MODEL_IDS)))
        if self._source == 'model':
            self.model()                   # the old parameters' time, first
        self._model.update({k: float(v) for k, v in values.items()})
        # The RUNNING rotor too, as the firmware's own model applies them:
        # writing `load` mid-hold reached only the dict, and the servo's sag
        # demo measured nothing because nothing sagged.
        motor = self._motor
        if motor is not None:
            for k in self.LIVE.keys() & values.keys():
                setattr(motor, self.LIVE[k], float(values[k]))
        # POLE PAIRS ARE NOT IN `live`: a Motor's `p` divides its own angle, so
        # changing it under a turning rotor is a different machine rather than
        # a different parameter.
        if 'pole_pairs' in values:
            self._motor = None
        # The chain took its R, L and lambda when it was built, so a machine
        # written after that would be observed as the old one - the observers
        # would still be reporting the stand-in's defaults while the model made
        # back-EMF for something else.
        self._obs = None
        return dict(values)

    def _pll_hz(self):
        """The natural frequency the loaded PLL gains imply."""
        l2 = self._p('drv_l2_milli', 100.0)
        wn = math.sqrt(max(l2, 1e-9) / (2.0 * self.TS))
        return min(max(wn / (2.0 * math.pi), 1.0), 5000.0)

    def _advance_model(self):
        """Turn the virtual rotor by the torque the dq solution makes."""
        motor = self._machine()
        now = time.time()
        dt = min(now - self._motor_at, 0.25)     # bounded catch-up
        self._motor_at = now
        if dt <= 0.0:
            return motor
        if self._mode != 'off':
            self._spin(motor, dt, now)
        # THE LAG IS CLOSED FORM, NOT INTEGRATED.
        wn = 2.0 * math.pi * self._pll_hz()
        alpha = (motor.omega - self._omega_hat) / dt if dt > 0.0 else 0.0
        self._omega_hat = motor.omega
        self._theta_hat = (motor.theta + alpha / (wn * wn)) % (2.0 * math.pi)
        return motor

    def _spin(self, motor, dt, now):
        """The rotor turned by the torque the dq solution makes, over `dt`,
        in fixed symplectic sub-steps.
        """
        iid, iq, _, _ = self._dq()
        ld = self._ld(iid)
        # TORQUE BY MODE.
        hold = self._mode == 'hold'
        k_t = TORQUE_FACTOR * motor.p * motor.lam
        i_mag = math.hypot(iid, iq)
        if not hold:
            torque = TORQUE_FACTOR * motor.p * (motor.lam * iq
                                      + (ld - motor.lq) * iid * iq)
        acc = self._motor_acc + dt
        cmd = (self._sp['theta']
               + self._omega() * (now - acc - self._mode_at))
        w_cmd = self._omega()
        wm = motor.omega / motor.p
        # SUBSTEPPED, SYMPLECTIC.
        step = min(0.002, 0.1 * motor.j / max(motor.b, 1e-12))
        if hold and i_mag > 0.0:
            spring = TORQUE_FACTOR * motor.p * motor.p * motor.lam * i_mag
            step = min(step, 0.05 * math.sqrt(motor.j / spring))
        # THE SUB-STEP IS FIXED and the remainder carried to the next call.
        h = step
        n = int(acc // h)
        self._motor_acc = acc - n * h
        theta = motor.theta
        # THE LINK RUNS OUT, and until now it never did.
        ceiling = (self._model['vdc'] / (math.sqrt(3.0) * motor.lam)
                   if motor.lam > 0.0 else float('inf'))
        for _ in range(n):
            if hold:
                cmd += w_cmd * h
                torque = k_t * i_mag * math.sin(cmd - theta)
            fade = max(0.0, 1.0 - abs(wm * motor.p) / ceiling)
            wm += (torque * fade - motor.b * wm - motor.load)                     / motor.j * h
            theta += wm * motor.p * h
        # The SHAFT, accumulated: electrical theta wraps at 2 pi and a shaft
        # sensor reads the mechanical angle, which is 1/p of the whole
        # unwrapped travel - `SimulatedAngle` reads this.
        self._mech += (theta - motor.theta) / motor.p
        motor.omega = wm * motor.p
        motor.theta = theta % (2.0 * math.pi)

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
            self._motor_acc = 0.0
        return self._motor

    @_rotor_locked
    def model(self):
        """The virtual source's rotor, or a still one on the ADC source."""
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
        self._motor_acc = 0.0
        self._omega_hat = 0.0
        self._obs = None
        self._theta_hat = self._model['theta0']
        return True
