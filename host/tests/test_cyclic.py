#!/usr/bin/env python3
"""The cyclic executive (machine.cyclic): its steps against machine.parts, its cycle on a toy rotor."""
import math
import os
import random
import sys
import types

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from machine import cyclic, parts  # noqa: E402
from machine.nodes import Module  # noqa: E402

from test_modbus_core import Report  # noqa: E402

#: Each kind: its part's inputs by name, as the step's u rows after dt.
PORTS = {'Gain': ('x',), 'Slew': ('x',), 'Wrap': ('x',), 'LowPass': ('x',),
         'SpeedKalman': ('measured', 'command'), 'PI': ('setpoint', 'measured'),
         'AngleHold': ('setpoint', 'measured'), 'Direct': ('setpoint',),
         'SpeedPI': ('setpoint', 'measured', 'accel', 'held')}

#: Parameters drawn a row, each kind's plausible span.
SPANS = {'k': (-3, 3), 'rate': (0.5, 50), 'zero': (-180, 180), 'tau': (0.01, 1), 'kt': (0.02, 0.2),
         'j': (1e-5, 1e-3), 'b': (1e-6, 1e-3), 'q': (1, 1e4), 'r': (1, 100), 'kp': (0, 1),
         'ki': (0, 20), 'limit': (0.5, 5), 'poles': (1, 14), 'theta0': (-3, 3), 'trim': (0, 10),
         'most': (0.5, 10), 'hz': (0.5, 10), 'load_k': (0, 1e-4), 'scale': (0.1, 1)}

ROWS, STEPS, DT = 4, 300, 0.004


def test_steps_are_the_parts(report):
    """Every kind over four rows of its own parameters, 300 steps of random inputs: the parts'
    outputs, row by row."""
    rng = random.Random(3)
    for kind, (step, _dtype) in cyclic.STEPS.items():
        cls = getattr(parts, kind)
        drawn = [{p: rng.uniform(*SPANS[p]) for p in cls.PARAMS} for _ in range(ROWS)]
        scalar = [cls(**d) for d in drawn]
        s = cyclic.state(kind, ROWS, **{p: [d[p] for d in drawn] for p in cls.PARAMS})
        ports = PORTS[kind]
        u = np.zeros((1 + len(ports), ROWS))
        worst = 0.0
        for _ in range(STEPS):
            u[0] = DT
            for i, port in enumerate(ports, 1):
                u[i] = [rng.choice((0.0, 1.0)) if port == 'held'
                        else math.nan if port == 'accel' and rng.random() < 0.5
                        else rng.uniform(-100, 100) for _ in range(ROWS)]
            y = step(s, u)
            for r, part in enumerate(scalar):
                kw = {p: (None if p == 'accel' and math.isnan(u[i][r]) else float(u[i][r]))
                      for i, p in enumerate(ports, 1)}
                want = next(iter(part.step(DT, **kw).values()))
                worst = max(worst, abs(y[r] - want) / max(1.0, abs(want)))
        report.check('%s steps as machine.parts does, row by row' % kind, worst < 1e-9,
                     '%.1e worst' % worst)


class Clock:
    """Time that moves only when slept."""

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def rotors(clock, js, every=0):
    """A node a rotor, w' = (kt iq - b w) / j stepped on the clock; `every` > 0 drops that read."""
    nodes, kt, b = [], 0.07, 1e-5
    for j in js:
        rotor = {'w': 0.0, 'iq': 0.0, 'at': 0.0, 'reads': 0}

        def read(rotor=rotor, j=j):
            dt = clock() - rotor['at']
            rotor['at'] = clock()
            rotor['w'] += dt * (kt * rotor['iq'] - b * rotor['w']) / j
            rotor['reads'] += 1
            if every and rotor['reads'] % every == 0:
                raise TimeoutError('no reply')
            return {'omega_hat': rotor['w']}

        writer = types.SimpleNamespace(write=lambda rotor=rotor, **kw: rotor.update(iq=kw['iq_ref']))
        nodes.append(types.SimpleNamespace(modules={'drive': Module(read=read, writer=writer)},
                                           rotor=rotor))
    return nodes


def speed_loop(nodes):
    f = cyclic.frame('w_target', 'omega_hat', 'w_ref', 'iq_ref', rows=len(nodes))
    e = cyclic.edge(nodes, 'drive', ins=('omega_hat',), outs=('iq_ref',))
    table = ((cyclic.slew, cyclic.state('Slew', len(nodes), rate=500.0), ('dt', 'w_target'),
              'w_ref'),
             (cyclic.pi, cyclic.state('PI', len(nodes), kp=0.02, ki=0.4, limit=2.0),
              ('dt', 'w_ref', 'omega_hat'), 'iq_ref'))
    return f, e, table


def test_a_cycle_holds_every_row(report):
    """Two rotors, one frame: the slew and the PI over both rows at once, each to its target."""
    clock = Clock()
    nodes = rotors(clock, (2e-5, 8e-5))
    f, e, table = speed_loop(nodes)
    f['w_target'] = (100.0, 200.0)
    log = cyclic.run((e,), table, f, 1500, 0.002, clock=clock, sleep=clock.sleep)
    last = log[-1]
    report.check('the log is the frame a pass, laid before the first',
                  log.shape == (1500, 2) and log.dtype == f.dtype, str(log.shape))
    report.check('each row settles on its own target',
                  all(abs(last['omega_hat'][r] / last['w_target'][r] - 1.0) < 0.02 for r in (0, 1)),
                  '%.1f of 100, %.1f of 200 rad/s' % tuple(last['omega_hat']))


def test_a_lost_frame_is_counted_not_raised(report):
    """Every seventh read times out: its row keeps the last value, the loss is counted, the cycle
    runs on."""
    clock = Clock()
    nodes = rotors(clock, (2e-5,), every=7)
    f, e, table = speed_loop(nodes)
    f['w_target'] = 100.0
    log = cyclic.run((e,), table, f, 1400, 0.002, clock=clock, sleep=clock.sleep)
    report.check('a seventh lost, counted on its row', e.lost[0] == 200, str(e.lost))
    report.check('and the loop still holds', abs(log[-1]['omega_hat'][0] / 100.0 - 1.0) < 0.02,
                 '%.1f rad/s' % log[-1]['omega_hat'][0])


def main():
    report = Report()
    for test in (test_steps_are_the_parts, test_a_cycle_holds_every_row,
                 test_a_lost_frame_is_counted_not_raised):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
