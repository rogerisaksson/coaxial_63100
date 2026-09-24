"""The controller: a loop put together from parts, and each part swapped."""
from .parts import code, md, section

TITLE = 'The controller'
SUMMARY = 'Source, estimator, regulator, sink and prefilters plugged into one `Loop`; setpoints from a table or a planner.'

SECTIONS = [
    section(
        'Source and sink',
        md('The drive on the model: `Polled(drive.state)` is the source, the drive itself the '
           'sink (`write(iq_ref=)`). The record gives the motor the regulator is tuned on.'),
        code('''from coaxial.control.controller import (Direct, LowPass, Loop, Paced, Polled, Slew,
                                        SpeedKalman, SpeedPI)
from coaxial.model.motor import Parameters
from coaxial.model.sensorless import RAD_S_PER_RPM

J, B = 2e-5, 1e-5
drive = device.drive
drive.configure(source='model')
drive.model.configure(j=J, b=B, load=0.0, noise=0.05)
device.gates.on(bypass_sto=True, ignore_interlock=True)
drive.write(id_ref=0.0, iq_ref=0.0)
drive.on('sensorless')

p = drive.params()
poles = int(p['motor_pole_pairs'])
motor = Parameters(name='the record', r=p['motor_r_uohm'], ld=p['motor_ld_nh'],
                   lq=p['motor_lq_nh'], lam=p['motor_lambda_uvs'], poles=poles, j=J, b=B,
                   measured=False)
kt = 1.5 * poles * motor.lam
source = Polled(drive.state)
print(sorted(source.read())[:8], '...')
print('poles %d, kt %.4f N.m/A' % (poles, kt))'''),
    ),
    section(
        'A loop from parts',
        code('''def rpm(rows, part):
    return [r[part]['w'] / RAD_S_PER_RPM for r in rows]

loop = Loop(source, drive,
            regulator=SpeedPI(3.0, 2.0, motor),
            estimator=Direct(w=('omega_hat', 1.0 / poles)),
            prefilters=(Slew(w=1500 * RAD_S_PER_RPM),),
            rate_hz=25)
rows = loop.move(2.0, w=1000 * RAD_S_PER_RPM)
for r in rows[::10]:
    print('%5.2f s  setpoint %6.0f  estimate %6.0f rpm  iq %6.3f A'
          % (r['t'], r['setpoint']['w'] / RAD_S_PER_RPM, r['estimate']['w'] / RAD_S_PER_RPM,
             r['command']['iq_ref']))'''),
    ),
    section(
        'Another estimator',
        md('`SpeedKalman` predicts on the command it was handed and corrects on the read.'),
        code('''def spread(values):
    m = sum(values) / len(values)
    return (sum((v - m) ** 2 for v in values) / len(values)) ** 0.5

held = loop.run(1.5)[-20:]
direct = spread(rpm(held, 'estimate'))
loop.estimator = SpeedKalman(kt, J, B, q=1e5, r=400.0, measured=('omega_hat', 1.0 / poles))
held = loop.run(1.5)[-20:]
kalman = spread(rpm(held, 'estimate'))
print('estimate spread at 1000 rpm: Direct %.1f rpm, SpeedKalman %.1f rpm' % (direct, kalman))'''),
    ),
    section(
        'A prefilter',
        md('0 to 2000 rpm with and without a `LowPass` behind the `Slew`: the peak current.'),
        code('''def peak(rows):
    return max(abs(r['command']['iq_ref']) for r in rows)

loop.move(2.0, w=0.0)
loop.prefilters = (Slew(w=3000 * RAD_S_PER_RPM),)
bare = peak(loop.move(1.5, w=2000 * RAD_S_PER_RPM))
loop.move(2.0, w=0.0)
loop.prefilters = (Slew(w=3000 * RAD_S_PER_RPM), LowPass(0.3, 'w'))
smooth = peak(loop.move(1.5, w=2000 * RAD_S_PER_RPM))
print('peak iq to 2000 rpm: Slew %.3f A, Slew + LowPass %.3f A' % (bare, smooth))'''),
    ),
    section(
        'A part at its own pace',
        md('`Paced` runs a part on its own thread; with a `source` it reads its own measurements. '
           'Here the estimator at 100 Hz under a 10 Hz loop.'),
        code('''fast = Paced(SpeedKalman(kt, J, B, q=1e5, r=400.0, measured=('omega_hat', 1.0 / poles)),
             hz=100, source=source)
loop.estimator, loop.pause = fast, 0.1
passes = len(loop.run(1.5))
print('%d estimator steps under %d loop passes' % (fast.steps, passes))
fast.stop()
loop.estimator, loop.pause = SpeedKalman(kt, J, B, q=1e5, r=400.0,
                                         measured=('omega_hat', 1.0 / poles)), 0.04'''),
    ),
    section(
        'Setpoints from a table, or a planner',
        md('`follow` takes (seconds, setpoints) rows, or a planner called after each block with '
           'the loop - the seam a model plugs into. None ends it.'),
        code('''table = [(1.0, {'w': 1500 * RAD_S_PER_RPM}), (1.0, {'w': 500 * RAD_S_PER_RPM})]
rows = loop.follow(table)
print('table: %d passes, last setpoint %.0f rpm' % (len(rows), rpm(rows, 'setpoint')[-1]))

asked = []

def planner(loop):
    """Up 25 % whenever the estimate is within 5 % of the setpoint; done past 1500 rpm."""
    last = loop.read()
    at, want = last['estimate']['w'], last['setpoint']['w']
    asked.append(round(at / RAD_S_PER_RPM))
    if want > 1500 * RAD_S_PER_RPM or len(asked) > 12:
        return None
    return (0.8, {'w': want * 1.25 if abs(at - want) < 0.05 * want else want})

rows = loop.follow(planner)
print('planner saw', asked)'''),
    ),
    section(
        'A regulator of your own',
        md('A `Regulator` is `step(setpoint, estimate, dt) -> command`. Proportional only: the '
           'drag it cannot hold without an integrator is its error.'),
        code('''from coaxial.devices.roles import Regulator

class Proportional(Regulator):
    def __init__(self, kp, limit):
        self.kp, self.limit = kp, limit

    def step(self, setpoint, estimate, dt):
        u = self.kp * (setpoint['w'] - estimate['w'])
        return {'iq_ref': max(-self.limit, min(self.limit, u))}

loop.regulator = Proportional(kp=2e-3, limit=2.0)
rows = loop.move(2.0, w=1000 * RAD_S_PER_RPM)
p_error = 1000 - sum(rpm(rows[-10:], 'estimate')) / 10
loop.regulator = SpeedPI(3.0, 2.0, motor)
rows = loop.move(2.0, w=1000 * RAD_S_PER_RPM)
pi_error = 1000 - sum(rpm(rows[-10:], 'estimate')) / 10
print('error at 1000 rpm: Proportional %.0f rpm, SpeedPI %.0f rpm' % (p_error, pi_error))
loop.move(1.0, w=0.0)
loop.off()
device.gates.off()
drive.configure(source='adc')'''),
    ),
]

RESULTS = [
    code('''print('1. estimate spread    Direct %.1f rpm, SpeedKalman %.1f rpm' % (direct, kalman))
print('2. peak current       Slew %.3f A, Slew + LowPass %.3f A' % (bare, smooth))
print('3. paced              %d estimator steps under %d loop passes' % (fast.steps, passes))
print('4. steady error       Proportional %.0f rpm, SpeedPI %.0f rpm' % (p_error, pi_error))'''),
    md('- One `Loop`; every part a line to swap. `motion.velocity` is this loop with '
       '`SpeedPI` and `Direct`.\n'
       '- A paced part with a `source` shares the link with the loop.'),
]

BENCH = ('The record commissioned first (`commissioning.ipynb`); no flags on `gates.on()`. '
         'At the bench `r` is the observer\'s measured variance.')

REFERENCES = [
    ('host/coaxial/control/controller.py', '`Loop`, `Paced`, `Polled`, the estimators, regulators, prefilters'),
    ('host/coaxial/devices/roles.py', 'the roles: Input, Output, Controller; Filter, Estimator, Regulator'),
    ('host/coaxial/control/motion.py', '`Velocity`: the same loop behind `device.motion.velocity`'),
    ('host/tests/test_controller.py', 'the loop against a toy rotor and the stand-in'),
]
