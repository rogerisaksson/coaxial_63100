"""What the controller's suite shares: a rotor and a clock stood in, a speed read, a refusal heard."""
import random

from machine.errors import MachineError
from machine.rig.controller import Feedback, Loop
from machine.rig.parts import Slew, SpeedPI
from machine.rig.roles import Input, Output


KT, J, B = 0.07, 2e-5, 1e-5
DT = 0.002


class Rotor(Input, Output):

    """w' = (kt iq - b w) / j, read with `noise` rad/s rms on it."""

    WRITES = ('iq_ref',)

    def __init__(self, noise=5.0, fault=None):
        self.w, self.iq, self.noise, self._fault = 0.0, 0.0, noise, fault
        self.rng = random.Random(1)
        self.stopped = False

    def advance(self, dt):
        self.w += dt * (KT * self.iq - B * self.w) / J

    def state(self):
        return {'w': self.w}

    def read(self, count=None, timeout=None):
        return {'w': self.w + self.rng.gauss(0.0, self.noise), 'fault': self._fault,
                'spinning': abs(self.w) > 1.0, 'name': 'toy'}

    def write(self, **command):
        self.iq = command.get('iq_ref', self.iq)

    def off(self):
        self.iq, self.stopped = 0.0, True


class Clock:

    """Time that passes when the loop sleeps, and the rotor with it."""

    def __init__(self, rotor):
        self.t, self.rotor = 0.0, rotor

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        for _ in range(int(round(seconds / DT))):
            self.rotor.advance(DT)
            self.t += DT


def refused(report, name, call, word):
    """`call()` refused, `word` in its words."""
    try:
        call()
        report.check(name, False)
    except MachineError as exc:
        report.check(name, word in str(exc), exc)


def speed(rotor, regulator=None, estimator=None, **kw):
    """A loop of one feedback, 'speed', around the toy rotor."""
    loop = Loop({'rotor': rotor}, {'rotor': rotor}, **kw)
    loop.add('speed', Feedback(regulator or SpeedPI(3.0, 2.0, KT, J, B), setpoint='w_target',
                               measured='rotor.w', command='iq_ref', sink='rotor.iq_ref',
                               prefilter=Slew(400.0), estimator=estimator, ref='w_ref',
                               estimate='w_hat'))
    return loop


def served(rotor, loop, seconds):
    rows = []
    for _ in range(int(seconds / DT)):
        rotor.advance(DT)
        rows.append(loop.step(DT))
    return rows


def spread(values):
    m = sum(values) / len(values)
    return (sum((v - m) ** 2 for v in values) / len(values)) ** 0.5
