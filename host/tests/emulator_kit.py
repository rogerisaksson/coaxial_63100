"""What the emulator's suite runs its groups by: its tests in order and timed, a group of one."""
import time


def run(report, tests):
    """`tests` in order, their names first, each one's seconds after it."""
    for test, *given in tests:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        began = time.monotonic()
        test(report, *given)
        print('  %.0f s' % (time.monotonic() - began))


def one(test):
    """A group of `test` alone."""
    return lambda report, _names=(): run(report, [(test,)])


def stock_wbc():
    """The committed firmware model's host reference (`mechatronics.STOCK`): its loop's stand torques
    and its wrench step's torques and loads across the four swung cases."""
    import json
    import os
    import subprocess
    import sys
    import numpy as np
    from machine.body import mechatronics
    if mechatronics.ROBOT != mechatronics.STOCK:
        env = dict(os.environ, COAXIAL_ROBOT=mechatronics.STOCK)
        here = os.path.dirname(os.path.abspath(__file__))
        code = ('import json, sys; sys.path[:0] = [%r, %r]; '
                'from emulator_kit import stock_wbc; print(json.dumps(stock_wbc()))' % (os.path.dirname(here), here))
        return json.loads(subprocess.check_output([sys.executable, '-X', 'utf8', '-c', code], env=env, text=True))
    from machine.stack import wbc
    from test_wbc_core import _standing
    from tools.cores import wbc as core
    b = wbc.Body()
    s, ask = _standing(b)
    base, q, u, _t = core.state_of(b)
    c = core.shared()
    c.reset()
    loop_tau = [float(x) for x in c.stack(base, q, u, ask)['tau'][:3]]
    sole = s['feet'][1]
    swung = dict(ask, stance=(True, False), fold={1: (0.6, 0.0, 0.0)},
                 swing={1: (sole['sole'], np.zeros(3), np.zeros(3), sole['quat'])})
    steps = [([float(x) for x in c.wrench(base, q, u, asked)['tau'][:3]],
              [float(x) for x in c.wrench(base, q, u, asked)['bears']])
             for asked in (ask, swung, dict(swung, strides=1.0), dict(swung, strides=0.5))]
    return {'loop_tau': loop_tau, 'steps': steps}
