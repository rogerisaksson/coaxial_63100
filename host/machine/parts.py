"""The parts a `Loop` is wired from: floats in, floats out, float parameters.

Each constructs with no arguments, so a panel can add one.
"""
import math

from machine.roles import Estimator, Filter, Regulator


# -- filters ---------------------------------------------------------------------------

class Gain(Filter):

    """y = k x: a unit, a pole count, a sign."""

    PARAMS = ('k',)

    def __init__(self, k=1.0):
        self.k = float(k)

    def step(self, dt, x=0.0):
        return {'y': self.k * x}


class Slew(Filter):

    """y follows x at most `rate` a second, from 0."""

    PARAMS = ('rate',)

    def __init__(self, rate=1.0):
        self.rate = float(rate)
        self.reset()

    def step(self, dt, x=0.0):
        self.y += max(-self.rate * dt, min(self.rate * dt, x - self.y))
        return {'y': self.y}

    def reset(self):
        self.y = 0.0


class Wrap(Filter):

    """Degrees from `zero`, folded into -180..180: an absolute sensor as a joint angle."""

    PARAMS = ('zero',)

    def __init__(self, zero=0.0):
        self.zero = float(zero)

    def step(self, dt, x=0.0):
        return {'y': (x - self.zero + 180.0) % 360.0 - 180.0}


class LowPass(Filter):

    """First order, time constant `tau` s."""

    PARAMS = ('tau',)

    def __init__(self, tau=0.1):
        self.tau = float(tau)
        self.reset()

    def step(self, dt, x=0.0):
        self.y += dt / (self.tau + dt) * (x - self.y)
        return {'y': self.y}

    def reset(self):
        self.y = 0.0


# -- estimators ------------------------------------------------------------------------

class SpeedKalman(Estimator):

    """A speed through the rotor's own law, w' = (kt iq - b w) / j: predicts on the
    command, corrects on the measurement; q process noise ((rad/s)^2/s), r its variance."""

    OUTPUTS = ('estimate', 'sigma')
    PARAMS = ('kt', 'j', 'b', 'q', 'r')

    def __init__(self, kt=0.05, j=2e-5, b=1e-5, q=1e4, r=100.0):
        self.kt, self.j, self.b, self.q, self.r = (float(v) for v in (kt, j, b, q, r))
        self.reset()

    def step(self, dt, measured=0.0, command=0.0):
        if self.w is None:
            self.w = measured
        else:
            self.w += dt * (self.kt * command - self.b * self.w) / self.j
            self.p += self.q * dt
            gain = self.p / (self.p + self.r)
            self.w += gain * (measured - self.w)
            self.p *= 1.0 - gain
        return {'estimate': self.w, 'sigma': math.sqrt(self.p)}

    def reset(self):
        self.w, self.p = None, self.r


# -- regulators ------------------------------------------------------------------------

class PI(Regulator):

    """command = kp e + ki integral(e), clamped to +/-limit, the integrator held while clamped."""

    PARAMS = ('kp', 'ki', 'limit')

    def __init__(self, kp=1e-3, ki=0.0, limit=1.0):
        self.kp, self.ki, self.limit = float(kp), float(ki), float(limit)
        self.reset()

    def step(self, dt, setpoint=0.0, measured=0.0):
        e = setpoint - measured
        raw = self.kp * e + self.x
        u = max(-self.limit, min(self.limit, raw))
        if u == raw:
            self.x += self.ki * e * dt
        return {'command': u}

    def reset(self):
        self.x = 0.0


class AngleHold(Regulator):

    """A joint angle (deg) as the drive's HOLD angle (rad electrical): the spring drags the
    rotor onto it; `ki` trims what a load sags, within +/-`trim` deg. The command moves at
    most `most` deg a step: a stalled pass costs time, never a slipped pole."""

    PARAMS = ('poles', 'theta0', 'ki', 'trim', 'most')

    def __init__(self, poles=7.0, theta0=0.0, ki=0.0, trim=5.0, most=6.0):
        self.poles, self.theta0, self.ki, self.trim, self.most = (
            float(v) for v in (poles, theta0, ki, trim, most))
        self.reset()

    def step(self, dt, setpoint=0.0, measured=0.0):
        self.x = max(-self.trim, min(self.trim, self.x + self.ki * (setpoint - measured) * dt))
        target = setpoint + self.x
        self.at += max(-self.most, min(self.most, target - self.at))
        return {'command': self.theta0 + math.radians(self.at) * self.poles}

    def reset(self):
        self.x = self.at = 0.0


class Direct(Regulator):

    """The setpoint as the command, within +/-`limit`: a current, a duty, a torque."""

    PARAMS = ('limit',)

    def __init__(self, limit=1.0):
        self.limit = float(limit)

    def step(self, dt, setpoint=0.0, measured=0.0):
        return {'command': max(-self.limit, min(self.limit, setpoint))}


class SpeedPI(Regulator):

    """iq from w: a PI whose zero cancels the mechanical pole, acceleration and drag
    (b w + load_k w|w|) fed forward, the integrator held while clamped or `held` (the
    inner loop saturated). `accel` unwired: the setpoint's own slope. `scale`: rad/s per
    unit of setpoint and measurement (RAD_S_PER_RPM for rpm)."""

    INPUTS = ('setpoint', 'measured', 'accel', 'held')
    PARAMS = ('hz', 'limit', 'kt', 'j', 'b', 'load_k', 'scale')

    def __init__(self, hz=3.0, limit=1.0, kt=0.05, j=2e-5, b=1e-5, load_k=0.0, scale=1.0):
        self.hz, self.limit, self.kt, self.j, self.b, self.load_k, self.scale = (
            float(v) for v in (hz, limit, kt, j, b, load_k, scale))
        self.reset()

    @classmethod
    def of(cls, hz, limit, motor, load=None):
        """From a motor (`poles lam j b`) and a load (`k`), as `motor` has them."""
        return cls(hz, limit, 1.5 * motor.poles * motor.lam, motor.j, motor.b,
                   load.k if load else 0.0)

    def step(self, dt, setpoint=0.0, measured=0.0, accel=None, held=0.0):
        setpoint, measured = setpoint * self.scale, measured * self.scale
        if accel is None:
            accel = (setpoint - self.was) / dt if dt else 0.0
        else:
            accel *= self.scale
        self.was = setpoint
        w0 = math.tau * self.hz
        err = setpoint - measured
        damp = self.b + 2.0 * self.load_k * abs(setpoint)
        ff = (self.j * accel + self.b * setpoint + self.load_k * setpoint * abs(setpoint)) / self.kt
        raw = w0 * self.j / self.kt * err + self.x + ff
        u = max(-self.limit, min(self.limit, raw))
        if u == raw and not held:
            self.x += w0 * damp / self.kt * err * dt
        return {'command': u}

    def reset(self):
        self.x = self.was = 0.0
