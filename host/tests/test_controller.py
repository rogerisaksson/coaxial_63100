#!/usr/bin/env python3
"""The composed controller: a loop put together from parts, against a toy rotor and the stand-in."""
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100                                            # noqa: E402
from coaxial.control.controller import (Direct, Loop, LowPass, Paced, PI,   # noqa: E402
                                        Slew, SpeedKalman, SpeedPI)
from coaxial.devices.roles import Estimator, Input, Output                  # noqa: E402
from coaxial.errors import RigError                                         # noqa: E402
from coaxial.model.motor import Parameters                                  # noqa: E402

KT, J, B = 0.07, 2e-5, 1e-5
MOTOR = Parameters(name='toy', r=0.05, ld=2e-5, lq=3e-5, lam=KT / 10.5, poles=7, j=J, b=B,
                   measured=False)
DT = 0.002


class Report:
    def __init__(self):
        self.passed = self.failed = 0

    def check(self, name, ok, detail=''):
        self.passed += bool(ok)
        self.failed += (not ok)
        print('  %s  %-58s %s' % ('PASS' if ok else 'FAIL', name, detail))


class Rotor(Input, Output):

    """w' = (kt iq - b w) / j; read with `noise` rad/s rms on it."""

    def __init__(self, noise=5.0, fault=None):
        self.w, self.iq, self.noise, self.fault = 0.0, 0.0, noise, fault
        self.rng = random.Random(1)
        self.stopped = False

    def advance(self, dt):
        self.w += dt * (KT * self.iq - B * self.w) / J

    def state(self):
        return {'w': self.w}

    def read(self, count=None, timeout=None):
        return {'w': self.w + self.rng.gauss(0.0, self.noise), 'fault': self.fault}

    def write(self, **command):
        self.iq = command['iq_ref']

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


def served(rotor, loop, seconds):
    rows = []
    for _ in range(int(seconds / DT)):
        rotor.advance(DT)
        rows.append(loop.step(DT))
    return rows


def mean_sd(values):
    m = sum(values) / len(values)
    return m, (sum((v - m) ** 2 for v in values) / len(values)) ** 0.5


def test_a_loop_holds_a_speed(report):
    for regulator, estimator in ((PI(kp=2e-3, ki=0.05, limit=2.0), Direct()),
                                 (SpeedPI(3.0, 2.0, MOTOR),
                                  SpeedKalman(KT, J, B, q=1e4, r=25.0))):
        rotor = Rotor()
        loop = Loop(rotor, rotor, regulator, estimator, prefilters=(Slew(w=400.0),))
        loop.write(w=200.0)
        served(rotor, loop, 3.0)
        name = '%s over %s' % (type(regulator).__name__, type(estimator).__name__)
        report.check('%s holds 200 rad/s within 2 %%' % name, abs(rotor.w - 200.0) < 4.0,
                     '%.1f' % rotor.w)


def test_a_part_swaps_in_one_line(report):
    rotor = Rotor()
    loop = Loop(rotor, rotor, PI(kp=2e-3, ki=0.05, limit=2.0))
    loop.write(w=100.0)
    served(rotor, loop, 2.0)
    loop.regulator = SpeedPI(3.0, 2.0, MOTOR)
    loop.write(w=150.0)
    served(rotor, loop, 3.0)
    report.check('the regulator swapped mid-run, and the new one holds the new target',
                 abs(rotor.w - 150.0) < 3.0, '%.1f' % rotor.w)
    loop.reset()
    report.check('reset clears every part', loop.regulator.law.x == 0.0 and loop.read() is None)


def test_the_estimator_is_quieter(report):
    rotor = Rotor(noise=20.0)
    loop = Loop(rotor, rotor, SpeedPI(3.0, 2.0, MOTOR), SpeedKalman(KT, J, B, q=1e4, r=400.0))
    loop.write(w=200.0)
    rows = served(rotor, loop, 3.0)[-500:]
    _, raw = mean_sd([r['measured']['w'] for r in rows])
    _, est = mean_sd([r['estimate']['w'] for r in rows])
    report.check('the Kalman estimate spreads less than half the raw read', est < raw / 2,
                 '%.2f against %.2f rad/s' % (est, raw))
    report.check('and predicts on the command it was handed',
                 'iq_ref' in rows[-1]['command'] and 'sigma' in rows[-1]['estimate'])


def test_prefilters_shape_the_setpoint(report):
    slew = Slew(w=400.0)
    got = [slew.step({'w': 200.0}, 0.01)['w'] for _ in range(10)]
    report.check('Slew moves 400 a second from 0', abs(got[-1] - 40.0) < 1e-9, got[-1])
    low = LowPass(0.1, 'w')
    for _ in range(100):
        y = low.step({'w': 1.0, 'x': 5.0}, 0.001)
    report.check('LowPass: 1 - 1/e after one tau, the others untouched',
                 abs(y['w'] - 0.632) < 0.01 and y['x'] == 5.0, y)


def test_setpoints_from_a_plan_or_a_planner(report):
    rotor = Rotor(noise=0.0)
    clock = Clock(rotor)
    loop = Loop(rotor, rotor, SpeedPI(3.0, 2.0, MOTOR), rate_hz=50, clock=clock,
                sleep=clock.sleep)
    loop.follow([(1.0, {'w': 100.0}), (1.5, {'w': 50.0})])
    report.check('a table: each row held for its seconds, the last one standing',
                 abs(rotor.w - 50.0) < 2.0 and loop.state()['target'] == {'w': 50.0},
                 '%.1f at %.2f s' % (rotor.w, clock.t))
    asked = []

    def planner(loop):
        """What a model would do: look at the last pass, name the next setpoint."""
        asked.append(loop.read()['estimate']['w'])
        return (0.5, {'w': asked[-1] + 20.0}) if len(asked) < 4 else None

    loop.follow(planner)
    report.check('a planner: asked after each block, until it answers None',
                 len(asked) == 4 and loop.state()['target']['w'] > 100.0, asked)


def test_a_fault_ends_the_loop(report):
    rotor = Rotor(fault='overcurrent')
    loop = Loop(rotor, rotor, PI(kp=1e-3, ki=0.0, limit=1.0))
    try:
        loop.step(DT)
        report.check('a source that reports a fault stops the loop', False)
    except RigError as exc:
        report.check('a source that reports a fault stops the loop', 'overcurrent' in str(exc),
                     exc)


class Counted(Estimator):
    def __init__(self):
        self.n = 0

    def step(self, measured, command, dt):
        self.n += 1
        return dict(measured)


def test_a_paced_part_keeps_its_own_rate(report):
    rotor = Rotor(noise=0.0)
    counted = Counted()
    paced = Paced(counted, hz=400, source=rotor)
    loop = Loop(rotor, rotor, PI(kp=1e-3, ki=0.0, limit=1.0), paced, rate_hz=20)
    loop.write(w=10.0)
    rows = loop.run(0.4)
    report.check('an estimator at 400 Hz steps many times a 20 Hz pass',
                 counted.n > 4 * len(rows), '%d steps, %d passes' % (counted.n, len(rows)))
    loop.off()
    stopped = counted.n
    time.sleep(0.05)
    report.check('off() stops its thread and the sink', counted.n == stopped and rotor.stopped,
                 '%d then %d' % (stopped, counted.n))


def test_velocity_is_a_loop(report):
    device = Coaxial63100(device=True).open()
    try:
        drive = device.drive
        drive.configure(source='model')
        drive.model.configure(j=J, b=B, load=0.0)
        device.gates.on(bypass_sto=True, ignore_interlock=True)
        lane = device.motion.velocity(2.0, estimator=SpeedKalman(
            1.5 * 7 * 0.005, J, B, q=1e5, r=100.0, measured=('omega_hat', 1.0 / 7)))
        with lane:
            rpm = lane.rpm(1500.0, seconds=2.0)
            last = lane.loop.read()
        report.check('motion.velocity runs a Loop, the estimator plugged in',
                     isinstance(lane.loop, Loop) and isinstance(lane.loop.estimator, SpeedKalman))
        report.check('and holds 1500 rpm on the stand-in within 5 %', abs(rpm - 1500.0) < 75.0,
                     '%.0f rpm, iq %.3f A' % (rpm, last['command']['iq_ref']))
    finally:
        device.gates.off()
        drive.configure(source='adc')
        device.close()


def main():
    report = Report()
    for test in (test_a_loop_holds_a_speed, test_a_part_swaps_in_one_line,
                 test_the_estimator_is_quieter, test_prefilters_shape_the_setpoint,
                 test_setpoints_from_a_plan_or_a_planner, test_a_fault_ends_the_loop,
                 test_a_paced_part_keeps_its_own_rate, test_velocity_is_a_loop):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
