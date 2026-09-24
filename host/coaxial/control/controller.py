"""A controller put together from parts, and the parts.

    loop = Loop(source, sink, regulator, estimator=Direct(), prefilters=(), rate_hz=25)
    loop.write(w=100.0)              # the hook: setpoints from a script, a plan, a model
    loop.run(2.0)                    # serve it: prefilter, read, estimate, regulate, write
    loop.move(seconds=2.0, w=100.0)  # write, then run
    loop.follow(plan)                # a sequence of (seconds, setpoints), or a planner
    loop.read()                      # the last pass: setpoint, measured, estimate, command

source   an Input: read() -> dict (a board's device, a model)
sink     an Output: write(**command)
parts    Filter.step(x, dt), Estimator.step(measured, command, dt),
         Regulator.step(setpoint, estimate, dt) -> dict; reset()
Paced    any part on its own thread at its own rate
"""
import math
import threading
import time

from coaxial.control.loop import Signals, SpeedLoop
from coaxial.devices.roles import Controller, Estimator, Filter, Input, Regulator
from coaxial.errors import RigError


class Loop(Controller):

    """source -> estimator -> regulator -> sink, setpoints through the prefilters."""

    def __init__(self, source, sink, regulator, estimator=None, prefilters=(), rate_hz=25.0,
                 clock=time.monotonic, sleep=time.sleep):
        self.source, self.sink = source, sink
        self.regulator = regulator
        self.estimator = estimator or Direct()
        self.prefilters = tuple(prefilters)
        self.pause = 1.0 / float(rate_hz)
        self._clock, self._sleep = clock, sleep
        self._target, self._command, self._last = {}, {}, None
        self._t = 0.0

    def state(self):
        return {'target': dict(self._target), 'last': self._last, 't': self._t}

    def write(self, **setpoints):
        """The hook: new targets, reached through the prefilters on the next passes."""
        self._target.update(setpoints)
        return dict(self._target)

    def read(self, count=None, timeout=None):
        return self._last

    def reset(self):
        for part in (self.estimator, self.regulator) + self.prefilters:
            part.reset()
        self._command, self._last, self._t = {}, None, 0.0

    def step(self, dt):
        """One pass; the row it made."""
        setpoint = dict(self._target)
        for f in self.prefilters:
            setpoint = f.step(setpoint, dt)
        measured = self.source.read()
        if measured.get('fault'):
            raise RigError('%s faulted mid-loop - %s; the loop is over'
                           % (type(self.source).__name__, measured['fault']))
        estimate = self.estimator.step(measured, self._command, dt)
        self._command = self.regulator.step(setpoint, estimate, dt)
        self.sink.write(**self._command)
        self._t += dt
        self._last = {'t': self._t, 'dt': dt, 'setpoint': setpoint, 'measured': measured,
                      'estimate': estimate, 'command': dict(self._command)}
        return self._last

    def run(self, seconds, watch=None):
        """Serve the loop for `seconds` of the clock; `watch(loop)` after every pass."""
        rows = []
        last = start = self._clock()
        while self._clock() - start < seconds:
            now = self._clock()
            rows.append(self.step(max(now - last, 1e-6)))
            last = now
            if watch is not None:
                watch(self)
            self._sleep(self.pause)
        return rows

    def move(self, seconds, watch=None, **targets):
        self.write(**targets)
        return self.run(seconds, watch)

    def follow(self, plan, watch=None):
        """Setpoints in sequence: `plan` is (seconds, {setpoints}) pairs, or a planner
        `plan(loop) -> (seconds, {setpoints})`, None when done - a script, a table, a model."""
        steps = iter(plan) if not callable(plan) else iter(lambda: plan(self), None)
        rows = []
        for seconds, setpoints in steps:
            rows += self.move(seconds, watch, **setpoints)
        return rows

    def off(self):
        """The sink to its safe state, and every paced part stopped."""
        for part in (self.estimator, self.regulator) + self.prefilters:
            if isinstance(part, Paced):
                part.stop()
        return self.sink.off()


class Paced:

    """A part on its own thread at its own rate; the loop hands it its inputs and takes
    its latest answer. With `source`, it reads its own measurements (an estimator faster
    than the loop): `Paced(SpeedKalman(...), 500, source=Polled(drive.state))`."""

    def __init__(self, part, hz, source=None, clock=time.monotonic, sleep=time.sleep):
        self.part, self.pause, self.source = part, 1.0 / float(hz), source
        self._clock, self._sleep = clock, sleep
        self._args, self._out, self._thread = None, None, None
        self._lock, self._part_lock = threading.Lock(), threading.Lock()
        self._ready = threading.Event()
        self.steps = 0

    def step(self, *args):
        with self._lock:
            self._args = args[:-1]
        if self._thread is None:
            self._stopping = False
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        self._ready.wait()
        with self._lock:
            return self._out

    def _run(self):
        last = self._clock()
        while not self._stopping:
            now = self._clock()
            with self._lock:
                args = self._args
            if self.source is not None:
                args = (self.source.read(),) + args[1:]
            with self._part_lock:
                out = self.part.step(*args, max(now - last, 1e-6))
            last = now
            with self._lock:
                self._out = out
                self.steps += 1
            self._ready.set()
            self._sleep(self.pause)

    def reset(self):
        with self._part_lock:
            self.part.reset()

    def stop(self):
        if self._thread is not None:
            self._stopping = True
            self._thread.join()
            self._thread = None
            self._ready.clear()


class Polled(Input):

    """Any call that answers a dict, as a source: `Polled(drive.state)`."""

    def __init__(self, call):
        self.call = call

    def state(self):
        return self.call()

    def read(self, count=None, timeout=None):
        return self.call()


# -- estimators ------------------------------------------------------------------------

class Direct(Estimator):

    """The measurement is the estimate: `Direct(w=('omega_hat', 1 / poles))` picks and scales;
    no keys passes everything."""

    def __init__(self, **picked):
        self.picked = picked

    def step(self, measured, command, dt):
        if not self.picked:
            return dict(measured)
        return {name: measured[key] * scale for name, (key, scale) in self.picked.items()}


class SpeedKalman(Estimator):

    """Speed from a noisy speed, through the rotor's own law: w' = (kt iq - b w) / j.

    Predicts on `command`, corrects on `measured` = (key, scale) - ('omega_hat', 1 / poles)
    off a drive; q the process noise ((rad/s)^2 per s), r the measurement's variance."""

    def __init__(self, kt, j, b, q, r, key='w', command='iq_ref', measured=None):
        self.kt, self.j, self.b, self.q, self.r = kt, j, b, q, r
        self.key, self.command = key, command
        self.measured = measured or (key, 1.0)
        self.reset()

    def step(self, measured, command, dt):
        z = measured[self.measured[0]] * self.measured[1]
        if self.w is None:
            self.w = z
        else:
            iq = command.get(self.command, 0.0)
            self.w += dt * (self.kt * iq - self.b * self.w) / self.j
            self.p += self.q * dt
            gain = self.p / (self.p + self.r)
            self.w += gain * (z - self.w)
            self.p *= 1.0 - gain
        return {self.key: self.w, 'sigma': math.sqrt(self.p)}

    def reset(self):
        self.w, self.p = None, self.r


# -- regulators ------------------------------------------------------------------------

class PI(Regulator):

    """`out` = kp e + ki integral(e), clamped to +/-`limit`, the integrator held while clamped."""

    def __init__(self, kp, ki, limit, key='w', out='iq_ref'):
        self.kp, self.ki, self.limit, self.key, self.out = kp, ki, limit, key, out
        self.reset()

    def step(self, setpoint, estimate, dt):
        e = setpoint.get(self.key, 0.0) - estimate[self.key]
        raw = self.kp * e + self.x
        u = max(-self.limit, min(self.limit, raw))
        if u == raw:
            self.x += self.ki * e * dt
        return {self.out: u}

    def reset(self):
        self.x = 0.0


class SpeedPI(Regulator):

    """`coaxial.control.loop.SpeedLoop` as a part: the zero on the mechanical pole,
    acceleration and drag fed forward, `iq_ref` from `w`."""

    def __init__(self, hz, limit, motor, load=None):
        self.law = SpeedLoop(hz, limit, motor, load)
        self.reset()

    def step(self, setpoint, estimate, dt):
        s = self.bus
        w_ref = setpoint.get('w', 0.0)
        s.a_ref = (w_ref - s.w_ref) / dt if dt else 0.0
        s.w_ref, s.w = w_ref, estimate['w']
        self.law(s, dt)
        return {'iq_ref': s.iq_ref}

    def reset(self):
        self.bus = Signals()
        self.law.x = 0.0


# -- prefilters ------------------------------------------------------------------------

class Slew(Filter):

    """Each named setpoint moves at most `rate` a second, from 0: `Slew(w=2000.0)`."""

    def __init__(self, **rates):
        self.rates = rates
        self.reset()

    def step(self, x, dt):
        out = dict(x)
        for key, rate in self.rates.items():
            if key in x:
                was = self.at.get(key, 0.0)
                move = max(-rate * dt, min(rate * dt, x[key] - was))
                out[key] = self.at[key] = was + move
        return out

    def reset(self):
        self.at = {}


class LowPass(Filter):

    """First order, time constant `tau` s, on the named setpoints."""

    def __init__(self, tau, *keys):
        self.tau, self.keys = tau, keys
        self.reset()

    def step(self, x, dt):
        out = dict(x)
        k = dt / (self.tau + dt)
        for key in self.keys:
            if key in x:
                self.y[key] = self.y.get(key, 0.0) + k * (x[key] - self.y.get(key, 0.0))
                out[key] = self.y[key]
        return out

    def reset(self):
        self.y = {}
