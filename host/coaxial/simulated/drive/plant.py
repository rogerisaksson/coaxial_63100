"""The stand-in's motor: the dq currents it carries, the sample, the rotor it turns."""
import math
import time
from typing import Any

from coaxial.devices.drive import MODEL_IDS
from coaxial.model import inverter
from coaxial.simulated.drive.locked import _rotor_locked
from motor.pmsm import TORQUE_FACTOR, Motor

#: The most the rotor is turned on at one request, s: a stall past it is time it never turns.
CATCH_UP_S = 0.25


class DrivePlant:

    """The PMSM under the stand-in's drive: currents, the HF step, the rotor's motion."""

    # What the class this mixes into brings.
    FS: Any
    TS: Any
    _derate: Any
    _hat_path: Any
    _mode_at: Any
    _model: Any
    _params: Any
    _rng: Any
    _source: Any
    _sp: Any
    _switching: Any

    #: What the polarity pulse reads for the aligned and the opposed half
    #: - invented, like everything here, and told apart by size.
    POL_READINGS = (34.0, 18.0)

    #: Periods past the two pulses and two gaps before the sign is read.
    POL_SETTLE = 4

    #: What one shunt's noise leaves on alpha, beta, d or q: drive_clarke's
    #: amplitude-invariant form keeps sqrt(2/3) of it on each, uncorrelated.
    CLARKE_NOISE = math.sqrt(2.0 / 3.0)

    #: The noise's seed, taken again at a reset so a run repeats
    #: (drive_model_init).
    NOISE_SEED = 1

    def _p(self, name, default):
        return self._params.get(name, default)

    # The motor this stand-in models.
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

    def _noise(self, share=1.0):
        """The model's current noise, A rms, `share` of a shunt's: on the
        model source only, where drive_model_sample puts it on each sample.
        """
        return share * self._model['noise'] if self._source == 'model' else 0.0

    def _noisy(self, *amps, share=1.0):
        """These currents as the samples report them: the noise on each."""
        sd = self._noise(share)
        if sd <= 0.0:
            return amps
        return tuple(a + self._rng.gauss(0.0, sd) for a in amps)

    def _periods_since(self, at):
        return int((time.time() - at) * self.FS)

    def _omega(self):
        return self._cmd_at(time.time())[1] if self._mode == 'hold' else 0.0

    def _cmd_at(self, t):
        """(theta, omega) of HOLD's command at `t`, drive.c's command_frame in closed form: omega
        ramped at `accel` toward `omega_target`, standing still at none, and theta its integral.
        As theta_setpoint + omega_target x t the vector jumped when the target moved - 1 rad
        from 1 to 2 rad/s a second in (2026-09-28)."""
        theta, omega, at = self._cmd
        dt = max(0.0, t - at)
        target, accel = self._sp['omega_target'], abs(self._sp['accel'])
        gap = target - omega
        ramp = abs(gap) / accel if accel > 0.0 else (0.0 if gap == 0.0 else float('inf'))
        a = math.copysign(accel, gap)
        if dt <= ramp:
            return theta + omega * dt + 0.5 * a * dt * dt, omega + a * dt
        return theta + omega * ramp + 0.5 * a * ramp * ramp + target * (dt - ramp), target

    def _frame(self):
        """The electrical angle the loop's dq frame sits at: the command's, turning at its
        speed, in HOLD; the tracked rotor's otherwise."""
        if self._mode == 'hold':
            theta = self._cmd_at(time.time())[0]
        else:
            theta = self._theta_hat
        return theta % (2.0 * math.pi)

    def _carrying(self):
        """(amps, electrical angle) the stator carries right now: the dq
        solution's magnitude, at the loop's frame.
        """
        iid, iq, _vd, _vq = self._dq()
        return math.hypot(iid, iq), self._frame()

    def _duty(self):
        """What the modulator puts on the compares: drive_svm of the loop's volts at its
        frame; none while off."""
        if self._mode == 'off':
            return (0.0, 0.0, 0.0)
        _iid, _iq, vd, vq = self._dq()
        cos, sin = math.cos(self._frame()), math.sin(self._frame())
        return inverter.svm(vd * cos - vq * sin, vd * sin + vq * cos, self._model['vdc'])

    def _dq(self):
        """The loop's dq means for the mode it is in."""
        if self._mode == 'volt':
            iid = self._sp['vd'] / self._r * self._p('drv_sign', 1.0)
            return iid, self._sp['vq'] / self._r, self._sp['vd'], self._sp['vq']
        if self._mode not in ('hold', 'sensorless'):
            return 0.0, 0.0, 0.0, 0.0
        # The clamp, as the envelope left it.
        i_max = self._p('drv_i_max', 5.0) * self._derate
        iid = max(-i_max, min(i_max, self._sp['id_ref']))
        iq = max(-i_max, min(i_max, self._sp['iq_ref']))
        # The speed in the voltage solution: HOLD's is the command's,
        # SENSORLESS on the model is the tracker's - it was 0.0 there, vq lost
        # its back-EMF term, and a power measurement read 0.34 W where the
        # shaft alone carried 34.
        omega = self._omega()
        if self._mode == 'sensorless' and self._source == 'model':
            omega = self._omega_hat
        a = self._r * iid + (2.0 / 3.0) * (self._dt(iid) + self._dt(iid / 2.0))
        c = omega * self._ld(iid) * iid + omega * self._lam
        # The link holds the current only while its volts reach, |v| <= vdc / sqrt 3: past
        # that the loop saturates and the current it gets is the asked one scaled toward zero,
        # none once the back-EMF alone is the link's - the world core's drive_model is driven
        # by volts. A torque faded linearly with speed gave half of kt I at half the no-load
        # speed; iq moved to meet the limit either way ran a fixed wing to -29 000 rpm on
        # braking current (2026-09-28).
        vmax = self._model['vdc'] / math.sqrt(3.0)
        a0 = a * a + c * c - vmax * vmax
        a2 = iq * iq * ((omega * self._lq) ** 2 + self._r ** 2)
        a1 = 2.0 * iq * (self._r * c - a * omega * self._lq)
        if a2 + a1 + a0 > 0.0:                   # the asked current is past the link
            iq = 0.0 if a0 >= 0.0 else iq * (-a1 + math.sqrt(a1 * a1 - 4.0 * a2 * a0)) / (2.0 * a2)
        vd = a - omega * self._lq * iq
        vq = self._r * iq + c
        return iid, iq, vd, vq

    def sample(self):
        """What a sampler in the control interrupt would see, this period."""
        iid, iq, _, _ = self._dq()
        theta = self._frame()
        cos, sin = math.cos(theta), math.sin(theta)
        alpha = iid * cos - iq * sin
        beta = iid * sin + iq * cos
        root3 = math.sqrt(3.0) / 2.0
        # Switching is the bridge's answer, not the loop's.
        on = self._switching() if self._switching else self._mode != 'off'
        amps = ((alpha, -0.5 * alpha + root3 * beta,
                 -0.5 * alpha - root3 * beta) if on else (0.0, 0.0, 0.0))
        return {'amps': self._noisy(*amps), 'switching': bool(on), 'link': self._model['vdc']}

    def _ih(self):
        """The demodulated HF current step: V.T over the inductance along
        the injection axis, with the rotor at zero and the frame at
        `theta` (HOLD) or the rotor observer's estimate (SENSORLESS).
        """
        v_inj = self._p('drv_inj_volts', 0.0)
        if not v_inj or self._mode not in ('hold', 'sensorless'):
            return 0.0, 0.0
        phi = self._frame() + self._p('drv_inj_phase', 0.0)
        iid = self._sp['id_ref'] if self._mode == 'hold' else 0.0
        ld = self._ld(iid)
        l_sum, l_del = (ld + self._lq) / 2.0, (ld - self._lq) / 2.0
        inv = (l_sum - l_del * math.cos(2.0 * phi)) / (ld * self._lq)
        ih = v_inj * self.TS * inv
        eps = v_inj * self.TS * l_del * math.sin(2.0 * phi) / (ld * self._lq)
        return ih, -eps

    def _estimate(self, theta, ran=None):
        """theta_hat to `theta`, its whole turns counted as drive.c counts them: `ran`, rad, how
        far it went where the rotor says - a read turns it many times - else the nearer way."""
        gap = (theta - self._theta_hat + math.pi) % (2 * math.pi) - math.pi
        if ran is not None:
            gap += 2 * math.pi * round((ran - gap) / (2 * math.pi))
        self._hat_path += gap
        self._theta_hat = theta % (2 * math.pi)

    def _converge(self):
        """SENSORLESS pulls theta_hat onto the rotor or pi off it, the nearer - the injection's
        two answers - while the back-EMF has no weight: the model's rotor, the ADC source's
        standing at 0. Pulled to 0 or pi alone, a turning model's estimate flipped half a pitch
        at rest (2026-09-28)."""
        if self._mode != 'sensorless' or not self._p('drv_inj_volts', 0.0):
            return
        dt = time.time() - self._theta_hat_at
        self._theta_hat_at = time.time()
        motor = self._motor if self._source == 'model' else None
        if motor is not None and abs(self._omega_hat) > self._p('drv_w_lo', 0.0):
            return
        rotor = motor.theta if motor is not None else 0.0
        target = rotor if math.cos(self._theta_hat - rotor) >= 0.0 else rotor + math.pi
        err = (self._theta_hat - target + math.pi) % (2 * math.pi) - math.pi
        self._estimate(target + err * math.exp(-dt * 60.0))

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
    #: each is on it - `ld` is a method on Motor and `ld0` holds the number.
    LIVE = {'r': 'r', 'ld': 'ld0', 'lq': 'lq', 'lambda': 'lam',
            'sat': 'sat', 'i_sat': 'i_sat', 'j': 'j', 'b': 'b',
            'load': 'load', 'v_dt': 'v_dt', 'i_knee': 'i_knee'}

    @_rotor_locked
    def _set_model(self, **values):
        for name in values:
            if name not in MODEL_IDS:
                raise ValueError('%r is not a model parameter; they are %s'
                                 % (name, ', '.join(MODEL_IDS)))
        if self._source == 'model':
            self._read_model()                   # the old parameters' time, first
        self._model.update({k: float(v) for k, v in values.items()})
        # The running rotor too, as the firmware's own model applies them:
        # writing `load` mid-hold reached only the dict, and the servo's sag
        # demo measured nothing because nothing sagged.
        motor = self._motor
        if motor is not None:
            for k in self.LIVE.keys() & values.keys():
                setattr(motor, self.LIVE[k], float(values[k]))
        # Pole pairs are not in `LIVE`: a Motor's `p` divides its own angle, so
        # changing it under a turning rotor is a different motor rather than
        # a different parameter.
        if 'pole_pairs' in values:
            self._motor = None
        # The chain took its R, L and lambda when it was built, so a motor
        # written after that would be observed as the old one - the observers
        # would still be reporting the stand-in's defaults while the model made
        # back-EMF for something else.
        self._obs = None
        return dict(values)

    def _pll_hz(self):
        """The natural frequency the loaded PLL gains imply."""
        l2 = self._p('drv_l2', 100.0)
        wn = math.sqrt(max(l2, 1e-9) / (2.0 * self.TS))
        return min(max(wn / (2.0 * math.pi), 1.0), 5000.0)

    def paced(self, dt):
        """The rotor turned `dt` s more at its next request, and by nothing but what it is told
        from then on - a world stepped on its own clock tells each pass's; None, the wall's
        clock again. On the wall's a pass 0.25 s late turned the rotor those seconds under a
        setpoint meant for 50 ms: QUAD's frame, stepped its 50, passed a gate 2.7 m off under
        the gate's load (2026-10-06)."""
        self._pace = None if dt is None else (self._pace or 0.0) + float(dt)

    def _advance_model(self):
        """Turn the virtual rotor by the torque the dq solution makes."""
        motor = self._motor_model()
        now = time.time()
        if self._pace is None:
            dt = min(now - self._motor_at, CATCH_UP_S)
        else:
            dt, self._pace = self._pace, 0.0
        self._motor_at = now
        if dt <= 0.0:
            return motor
        # Off too: the bridge open, no current, the rotor on its drag and load - world.c's
        # coast(). Skipped, the shaft stood at 2 451 rpm for 18 s with the stage off (2026-09-28).
        mech = self._mech
        self._spin(motor, dt, now)
        # The tracker: its PLL's lag in closed form, not integrated.
        wn = 2.0 * math.pi * self._pll_hz()
        alpha = (motor.omega - self._omega_hat) / dt if dt > 0.0 else 0.0
        self._omega_hat = motor.omega
        self._estimate(motor.theta + alpha / (wn * wn), ran=(self._mech - mech) * motor.p)
        cmd, w_cmd = self._cmd_at(now)
        if self._mode == 'hold' and abs(w_cmd) <= self._p('drv_w_lo', 0.0):
            # drive.c's command frame below the back-EMF's speed: the frame the rotor is held in
            # is the estimate, not the rotor ringing in it - a stepper's mark shivered 11 degrees
            # a frame on the tracker (2026-09-28).
            self._estimate(cmd)
        return motor

    def _spin(self, motor, dt, now):
        """The rotor turned by the torque the dq solution makes, over `dt`,
        in fixed symplectic sub-steps.
        """
        iid, iq, _, _ = self._dq()
        ld = self._ld(iid)
        # Torque by mode.
        hold = self._mode == 'hold'
        k_t = TORQUE_FACTOR * motor.p * motor.lam
        i_mag = math.hypot(iid, iq)
        if not hold:
            torque = TORQUE_FACTOR * motor.p * (motor.lam * iq
                                      + (ld - motor.lq) * iid * iq)
        acc = self._motor_acc + dt
        cmd, w_cmd = self._cmd_at(now - acc) if hold else (0.0, 0.0)
        target, accel = self._sp['omega_target'], abs(self._sp['accel'])
        wm = motor.omega / motor.p
        # Sub-stepped, symplectic.
        step = min(0.002, 0.1 * motor.j / max(motor.b, 1e-12))
        if hold and i_mag > 0.0:
            spring = TORQUE_FACTOR * motor.p * motor.p * motor.lam * i_mag
            step = min(step, 0.05 * math.sqrt(motor.j / spring))
        # The sub-step is fixed and the remainder carried to the next call.
        h = step
        n = int(acc // h)
        self._motor_acc = acc - n * h
        theta = motor.theta
        for _ in range(n):
            if hold:
                # command_frame, a sub-step at a time.
                gap = target - w_cmd
                w_cmd = target if abs(gap) <= accel * h else w_cmd + math.copysign(accel * h, gap)
                cmd += w_cmd * h
                torque = k_t * i_mag * math.sin(cmd - theta)
            wm += (torque - motor.b * wm - motor.load)                     / motor.j * h
            theta += wm * motor.p * h
        # The shaft, accumulated: electrical theta wraps at 2 pi and a shaft
        # sensor reads the mechanical angle, which is 1/p of the whole
        # unwrapped travel - `SimulatedAngle` reads this.
        self._mech += (theta - motor.theta) / motor.p
        motor.omega = wm * motor.p
        motor.theta = theta % (2.0 * math.pi)
        if hold:
            self._cmd = [cmd, w_cmd, now - self._motor_acc]

    def _motor_model(self):
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
    def _read_model(self):
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
    def _reset_model(self):
        """The rotor back to theta0, at rest - the contract `drive.py` states."""
        self._motor = None
        self._motor_acc = 0.0
        self._omega_hat = 0.0
        self._obs = None
        self._rng.seed(self.NOISE_SEED)
        self._estimate(self._model['theta0'])
        return True
