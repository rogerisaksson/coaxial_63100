"""The controller: feedback loops over float channels, each slot a part to swap."""
from .parts import code, md, section

TITLE = 'The controller'
SUMMARY = 'Sources in, sinks out, feedback loops of prefilter, measure, estimator and regulator between; a panel; saved.'

SECTIONS = [
    section(
        'Source and sink',
        md('The drive on the model: `Polled(drive.state)` is the source, the drive the sink. '
           'Every channel is a float; the record gives the motor.'),
        code('''from coaxial.control.controller import Feedback, Loop, Paced, Polled
from coaxial.control.parts import Gain, LowPass, Slew, SpeedKalman, SpeedPI
from coaxial.draw import ansi
from coaxial.draw.wiring import feedback
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
loop = Loop({'drive': Polled(drive.state)}, {'drive': drive}, rate_hz=25)
print('%d source channels, the sink writes %s' % (len(loop.channels()), ', '.join(drive.WRITES)))'''),
    ),
    section(
        'A feedback loop',
        code('''loop.add('speed', Feedback(
    SpeedPI.of(3.0, 2.0, motor), setpoint='w_target', measured='drive.omega_hat',
    command='iq_ref', sink='drive.iq_ref', prefilter=Slew(1500 * RAD_S_PER_RPM),
    measure=Gain(1.0 / poles), ref='w_ref', value='w_hat'))
ansi.image(feedback(loop, 'speed'))'''),
        code('''rows = loop.move(2.0, w_target=1000 * RAD_S_PER_RPM)
for r in rows[::10]:
    print('%5.2f s  w_ref %6.0f  w_hat %6.0f rpm  iq %6.3f A'
          % (r['t'], r['w_ref'] / RAD_S_PER_RPM, r['w_hat'] / RAD_S_PER_RPM, r['iq_ref']))'''),
    ),
    section(
        'An estimator in its slot',
        md('`SpeedKalman` predicts on the command and corrects on the measurement: the raw '
           'speed on `w`, the estimate on `w_hat`, one run.'),
        code('''def spread(values):
    m = sum(values) / len(values)
    return (sum((v - m) ** 2 for v in values) / len(values)) ** 0.5

f = loop.feedbacks['speed']
f.estimator, f.value, f.estimate = SpeedKalman(kt, J, B, 1e5, 400.0), 'w', 'w_hat'
loop.add('speed', f)
held = loop.run(1.5)[-20:]
raw, kalman = (spread([r[k] / RAD_S_PER_RPM for r in held]) for k in ('w', 'w_hat'))
print('spread at 1000 rpm: measured %.2f rpm, estimated %.2f rpm' % (raw, kalman))
ansi.image(feedback(loop, 'speed'))'''),
    ),
    section(
        'Another prefilter',
        md('0 to 2000 rpm through `Slew`, then through `LowPass`: the peak current each asks.'),
        code('''def peak(rows):
    return max(abs(r['iq_ref']) for r in rows)

loop.move(2.0, w_target=0.0)
bare = peak(loop.move(1.5, w_target=2000 * RAD_S_PER_RPM))
loop.move(2.0, w_target=0.0)
f.prefilter = LowPass(0.3)
loop.add('speed', f)
smooth = peak(loop.move(1.5, w_target=2000 * RAD_S_PER_RPM))
print('peak iq to 2000 rpm: Slew %.3f A, LowPass %.3f A' % (bare, smooth))'''),
    ),
    section(
        'A part at its own pace',
        md('`Paced` steps a part on its own thread; `feed` hands it fresh inputs between the '
           'loop\'s passes. The estimator at 100 Hz under a 10 Hz loop.'),
        code('''fast = Paced(SpeedKalman(kt, J, B, 1e5, 400.0), 100,
             feed=lambda: {'measured': drive.state()['omega_hat'] / poles})
f.estimator = fast
loop.add('speed', f)
loop.pause = 0.1
passes = len(loop.run(1.5))
print('%d estimator steps under %d loop passes' % (fast.steps, passes))
fast.stop()
f.estimator = SpeedKalman(kt, J, B, 1e5, 400.0)
loop.add('speed', f)
loop.pause = 0.04'''),
    ),
    section(
        'A regulator of your own',
        md('A `Regulator` names its PARAMS and steps: `step(dt, setpoint, measured)` -> '
           '`{\'command\': ...}`. Proportional alone leaves the drag as its error.'),
        code('''from coaxial.devices.roles import Regulator

class Proportional(Regulator):
    PARAMS = ('kp', 'limit')

    def __init__(self, kp=2e-3, limit=2.0):
        self.kp, self.limit = kp, limit

    def step(self, dt, setpoint=0.0, measured=0.0):
        u = self.kp * (setpoint - measured)
        return {'command': max(-self.limit, min(self.limit, u))}

def error(rows):
    return 1000 - sum(r['w_hat'] for r in rows[-10:]) / 10 / RAD_S_PER_RPM

f.regulator = Proportional()
loop.add('speed', f)
p_error = error(loop.move(2.0, w_target=1000 * RAD_S_PER_RPM))
f.regulator = SpeedPI.of(3.0, 2.0, motor)
loop.add('speed', f)
pi_error = error(loop.move(2.0, w_target=1000 * RAD_S_PER_RPM))
print('error at 1000 rpm: Proportional %.0f rpm, SpeedPI %.0f rpm' % (p_error, pi_error))'''),
    ),
    section(
        'The panel, and kept',
        md('The loops in a list with + and -; the selected one drawn, its channels, kinds and '
           'parameters in dropdowns and fields, live. `save` writes the loop; '
           '`Loop.load` rebuilds it on live sources and sinks.'),
        code('''import os
import tempfile
from coaxial.control.panel import panel

path = os.path.join(tempfile.gettempdir(), 'coaxial_controller.json')
loop.save_at_exit(path)
panel(loop, path)'''),
        code('''loop.save(path)
again = Loop.load(path, {'drive': Polled(drive.state)}, {'drive': drive})
kept = again.config() == loop.config()
print('saved %s: %d loop, %d parts, reloaded the same: %s'
      % (os.path.basename(path), len(again.feedbacks), len(again.parts), kept))
loop.move(1.0, w_target=0.0)
loop.off()
device.gates.off()
drive.configure(source='adc')'''),
    ),
]

RESULTS = [
    code('''print('1. estimate spread    measured %.2f rpm, SpeedKalman %.2f rpm' % (raw, kalman))
print('2. peak current       Slew %.3f A, LowPass %.3f A' % (bare, smooth))
print('3. paced              %d estimator steps under %d loop passes' % (fast.steps, passes))
print('4. steady error       Proportional %.0f rpm, SpeedPI %.0f rpm' % (p_error, pi_error))
print('5. kept               %s' % kept)'''),
    md('- `motion.velocity` is this loop: one feedback, `speed`.\n'
       '- A paced part with a `feed` reads the link between the loop\'s passes.'),
]

BENCH = ('The record commissioned first (`commissioning.ipynb`); no flags on `gates.on()`. '
         'At the bench `r` is the observer\'s measured variance.')

REFERENCES = [
    ('host/coaxial/control/controller.py', '`Loop`, `Feedback`, `Paced`, `Polled`; save and load'),
    ('host/coaxial/control/parts.py', 'the parts: Gain, Slew, LowPass, SpeedKalman, PI, SpeedPI'),
    ('host/coaxial/control/panel.py', 'the panel'),
    ('host/coaxial/draw/wiring.py', 'the pictures: a loop, a feedback'),
    ('host/coaxial/devices/roles.py', 'the roles, and Part: ports, PARAMS, step'),
    ('host/tests/test_controller.py', 'the loop against a toy rotor and the stand-in'),
]
