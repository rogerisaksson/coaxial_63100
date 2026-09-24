#!/usr/bin/env python3
"""The controller: feedback loops over float channels, its parts, its panel, its sequencer."""
import os
import random
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100                                            # noqa: E402
from coaxial.control.controller import Feedback, Loop, Paced, Polled        # noqa: E402
from coaxial.control.parts import Gain, LowPass, PI, Slew, SpeedKalman, SpeedPI  # noqa: E402
from coaxial.control.sequencer import Sequencer                             # noqa: E402
from coaxial.devices.roles import Estimator, Input, Output, Regulator       # noqa: E402
from coaxial.draw.wiring import diagram, feedback                           # noqa: E402
from coaxial.errors import RigError                                         # noqa: E402

KT, J, B = 0.07, 2e-5, 1e-5
DT = 0.002


class Report:
    def __init__(self):
        self.passed = self.failed = 0

    def check(self, name, ok, detail=''):
        self.passed += bool(ok)
        self.failed += (not ok)
        print('  %s  %-58s %s' % ('PASS' if ok else 'FAIL', name, detail))


class Rotor(Input, Output):

    """w' = (kt iq - b w) / j, read with `noise` rad/s rms on it."""

    WRITES = ('iq_ref',)

    def __init__(self, noise=5.0, fault=None):
        self.w, self.iq, self.noise, self.fault = 0.0, 0.0, noise, fault
        self.rng = random.Random(1)
        self.stopped = False

    def advance(self, dt):
        self.w += dt * (KT * self.iq - B * self.w) / J

    def state(self):
        return {'w': self.w}

    def read(self, count=None, timeout=None):
        return {'w': self.w + self.rng.gauss(0.0, self.noise), 'fault': self.fault,
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


def test_a_feedback_holds_a_speed(report):
    for regulator, estimator in ((PI(2e-3, 0.05, 2.0), None),
                                 (SpeedPI(3.0, 2.0, KT, J, B), SpeedKalman(KT, J, B, 1e4, 25.0))):
        rotor = Rotor()
        loop = speed(rotor, regulator, estimator)
        loop.write(w_target=200.0)
        served(rotor, loop, 3.0)
        report.check('%s%s holds 200 rad/s within 2 %%' % (
            type(regulator).__name__, ' over SpeedKalman' if estimator else ''),
            abs(rotor.w - 200.0) < 4.0, '%.1f' % rotor.w)


def test_every_channel_is_a_float(report):
    rotor = Rotor()
    loop = speed(rotor)
    loop.write(w_target=10, enable=True)
    row = loop.step(DT)
    report.check('ints and bools land as floats, a string never does',
                 all(isinstance(v, float) for v in row.values()) and row['enable'] == 1.0
                 and row['rotor.spinning'] == 0.0 and 'rotor.name' not in row, sorted(row))
    try:
        loop.write(mode='hold')
        report.check('a string setpoint is refused', False)
    except RigError as exc:
        report.check('a string setpoint is refused', 'float' in str(exc), exc)


def test_parts_swap_in_place(report):
    rotor = Rotor()
    loop = speed(rotor, PI(2e-3, 0.05, 2.0))
    loop.write(w_target=100.0)
    served(rotor, loop, 2.0)
    f = loop.feedbacks['speed']
    f.regulator = SpeedPI(3.0, 2.0, KT, J, B)
    loop.add('speed', f)
    loop.write(w_target=150.0)
    served(rotor, loop, 3.0)
    report.check('the regulator swapped mid-run holds the new target',
                 abs(rotor.w - 150.0) < 3.0, '%.1f' % rotor.w)
    report.check('its parts step in slot order',
                 list(loop.parts) == ['speed/prefilter', 'speed/regulator'], list(loop.parts))
    loop.add('outer', Feedback(PI(1.0, 0.0, 500.0), setpoint='x_target', measured='rotor.w',
                               command='w_target'))
    report.check('a second loop cascades into the first by channel name',
                 loop.channel_of('outer/regulator', 'command') == 'w_target'
                 and 'outer/regulator' in loop.parts)
    loop.remove('outer')
    report.check('and comes out again, parts and all', 'outer' not in loop.feedbacks
                 and not any(p.startswith('outer/') for p in loop.parts))
    try:
        loop.wire(**{'speed/regulator.nothing': 'x'})
        report.check('a port that is not there is refused', False)
    except RigError as exc:
        report.check('a port that is not there is refused', 'no port' in str(exc), exc)


def test_the_estimator_is_quieter(report):
    rotor = Rotor(noise=20.0)
    loop = speed(rotor, estimator=SpeedKalman(KT, J, B, 1e4, 400.0))
    loop.write(w_target=200.0)
    rows = served(rotor, loop, 3.0)[-500:]
    raw, est = spread([r['rotor.w'] for r in rows]), spread([r['w_hat'] for r in rows])
    report.check('the Kalman estimate spreads less than half the raw read', est < raw / 2,
                 '%.2f against %.2f rad/s' % (est, raw))


def test_filters(report):
    slew = Slew(400.0)
    y = [slew.step(0.01, x=200.0)['y'] for _ in range(10)][-1]
    report.check('Slew moves 400 a second from 0', abs(y - 40.0) < 1e-9, y)
    low = LowPass(0.1)
    for _ in range(100):
        y = low.step(0.001, x=1.0)['y']
    report.check('LowPass: 1 - 1/e after one tau', abs(y - 0.632) < 0.01, y)
    report.check('Gain scales', Gain(0.5).step(0.0, x=4.0) == {'y': 2.0})


def test_a_table_or_a_planner(report):
    rotor = Rotor(noise=0.0)
    clock = Clock(rotor)
    loop = speed(rotor, rate_hz=50, clock=clock, sleep=clock.sleep)
    loop.follow([(1.0, {'w_target': 100.0}), (1.5, {'w_target': 50.0})])
    report.check('a table: each row held for its seconds, the last one standing',
                 abs(rotor.w - 50.0) < 2.0, '%.1f at %.2f s' % (rotor.w, clock.t))
    asked = []

    def planner(loop):
        asked.append(loop.read()['w_ref'])
        return (0.5, {'w_target': asked[-1] + 20.0}) if len(asked) < 4 else None

    loop.follow(planner)
    report.check('a planner: asked after each block, until it answers None',
                 len(asked) == 4, asked)


def test_the_sequencer(report):
    folder = tempfile.mkdtemp()
    path = os.path.join(folder, 'steps.csv')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write('group,seconds,label,w_target,enable,w_ref.H,goto,times\n'
                     'init,0.2,,0,false,,,\n'
                     ',0.5,start,100,true,,,\n'
                     ',3,,150,,120,,\n'
                     ',0.5,,50,,,start,2\n'
                     ',0.5,end,0,,,,\n'
                     'cleanup,0.2,,0,false,,,\n')
    rotor = Rotor(noise=0.0)
    clock = Clock(rotor)
    loop = speed(rotor, rate_hz=50, clock=clock, sleep=clock.sleep)
    hooks = []
    seq = Sequencer.read(path, init=lambda _: hooks.append('init'),
                         cleanup=lambda _: hooks.append('cleanup'))
    out = seq.run(loop)
    rows = [s[0] for s in out.steps]
    report.check('init, the goto block three times, end, cleanup',
                 rows == [0] + [1, 2, 3] * 3 + [4, 5] and out.status == 'done', rows)
    report.check('H ends a step early, before its seconds',
                 all(s[2].startswith('w_ref') and s[3] < 3.0 for s in out.steps if s[0] == 2),
                 [round(s[3], 2) for s in out.steps if s[0] == 2])
    report.check('the hooks ran, init first, cleanup last', hooks == ['init', 'cleanup'])
    report.check('a blank cell holds its channel; true is 1',
                 all(r['enable'] == 1.0 for r in out.rows if int(r['step']) == 2))
    import pandas
    xlsx = os.path.join(folder, 'steps.xlsx')
    pandas.read_csv(path).to_excel(xlsx, index=False)
    again = Sequencer.read(xlsx)
    report.check('an xlsx reads as the csv does',
                 [repr(s) for g in again.groups.values() for s in g]
                 == [repr(s) for g in seq.groups.values() for s in g])
    hot = Sequencer([{'seconds': 3, 'w_target': 400.0},
                     {'group': 'cleanup', 'seconds': 0.5, 'w_target': 0.0}],
                    limits={'rotor.w': {'HH': 300.0}})
    out = hot.run(loop)
    report.check('HH trips the sequence, and cleanup still runs',
                 out.status == 'tripped' and 'rotor.w' in out.reason
                 and [s[0] for s in out.steps] == [0, 1], (out.status, out.reason))
    looped = Sequencer([{'seconds': 0.2, 'w_target': 10.0}], cycles=3)
    report.check('cycles repeat the main group', len(looped.run(loop).steps) == 3)
    forever = Sequencer([{'seconds': 0.2, 'label': 'a', 'goto': 'a', 'times': 'inf'}], limit=1.0)
    out = forever.run(loop)
    report.check('inf loops until the limit', out.status == 'limit' and 5 <= len(out.steps) <= 6,
                 len(out.steps))
    seq.jump('end')
    report.check('jump, back, forward move the next main row',
                 (seq.at, seq.back(), seq.forward()) == (3, 2, 3))
    try:
        Sequencer([{'seconds': 1, 'goto': 'nowhere'}])
        report.check('a goto to no label is refused', False)
    except RigError as exc:
        report.check('a goto to no label is refused', 'nowhere' in str(exc), exc)


def test_a_program_as_data(report):
    """3 x 4 by counters: set, add, and branch on a level - a counter machine."""
    table = ('seconds,label,a,a+,c,c+,t,t+,a.L,t.L,then,else\n'
             '0,,3,,0,,,,,,,\n'
             '0,outer,,,,,,,0,,stop,\n'
             '0,,,,,,4,,,,,\n'
             '0,inner,,,,,,,,0,dec,\n'
             '0,,,,,1,,-1,,,,inner\n'
             '0,dec,,-1,,,,,,,,outer\n')
    path = os.path.join(tempfile.mkdtemp(), 'multiply.csv')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(table)
    rotor = Rotor(noise=0.0)
    loop = speed(rotor)
    out = Sequencer.read(path).run(loop)
    report.check('the table multiplies: c = 12, and no time passed',
                 loop.bus['c'] == 12.0 and out.status == 'done' and loop.bus['t'] == 0.0,
                 (loop.bus['c'], out.status, len(out.steps)))
    endless = Sequencer([{'seconds': 0, 'label': 'spin', 'else': 'spin'}], max_steps=50)
    out = endless.run(loop)
    report.check('a program that never stops is stopped by max_steps',
                 out.status == 'limit' and len(out.steps) == 50, out.reason)
    try:
        Sequencer([{'seconds': 1, 'label': 'stop'}])
        report.check('stop is not a label', False)
    except RigError as exc:
        report.check('stop is not a label', 'stop' in str(exc), exc)


def test_save_and_load(report):
    rotor = Rotor()
    loop = speed(rotor, estimator=Paced(SpeedKalman(KT, J, B, 1e4, 25.0), 200))
    loop.write(w_target=120.0)
    loop.feedbacks['speed'].regulator.configure(hz=4.5)
    path = os.path.join(tempfile.mkdtemp(), 'loop.json')
    with loop.saving(path):
        pass
    again = Loop.load(path, {'rotor': rotor}, {'rotor': rotor})
    report.check('a saved loop loads with its loops, parameters, wires and setpoints',
                 again.config() == loop.config(), path)
    report.check('a paced part comes back paced',
                 isinstance(again.feedbacks['speed'].estimator, Paced))
    loop.off()
    again.off()


def test_a_fault_ends_the_loop(report):
    rotor = Rotor(fault='overcurrent')
    loop = speed(rotor)
    try:
        loop.step(DT)
        report.check('a source that reports a fault stops the loop', False)
    except RigError as exc:
        report.check('a source that reports a fault stops the loop', 'overcurrent' in str(exc),
                     exc)


class Counted(Estimator):
    def __init__(self):
        self.n = 0

    def step(self, dt, measured=0.0, command=0.0):
        self.n += 1
        return {'estimate': measured}


def test_a_paced_part_keeps_its_own_rate(report):
    rotor = Rotor(noise=0.0)
    counted = Counted()
    loop = speed(rotor, PI(1e-3, 0.0, 1.0),
                 Paced(counted, hz=400, feed=lambda: {'measured': rotor.state()['w']}),
                 rate_hz=20)
    loop.write(w_target=10.0)
    rows = loop.run(0.4)
    report.check('an estimator at 400 Hz steps many times a 20 Hz pass',
                 counted.n > 4 * len(rows), '%d steps, %d passes' % (counted.n, len(rows)))
    loop.off()
    stopped = counted.n
    time.sleep(0.05)
    report.check('off() stops its thread and the sink', counted.n == stopped and rotor.stopped)


class Proportional(Regulator):
    PARAMS = ('kp',)

    def __init__(self, kp=1e-3):
        self.kp = kp

    def step(self, dt, setpoint=0.0, measured=0.0):
        return {'command': self.kp * (setpoint - measured)}


def test_the_pictures_and_the_panel(report):
    rotor = Rotor()
    loop = speed(rotor, estimator=SpeedKalman())
    text = feedback(loop, 'speed', colour=False)
    report.check('the feedback picture names its slots and channels',
                 all(s in text for s in ('prefilter Slew', 'regulator SpeedPI', 'Σ', 'w_hat',
                                          'rotor.iq_ref', 'rotor.w')))
    report.check('the overview draws every part', all(p in diagram(loop, colour=False)
                                                     for p in loop.parts))
    try:
        import ipywidgets  # noqa: F401
    except ImportError:
        report.check('the panel (ipywidgets absent: skipped)', True)
        return
    from coaxial.control.panel import kinds, panel
    report.check('a part of your own is a kind the panel offers',
                 'Proportional' in kinds(Regulator) and 'Regulator' not in kinds(Regulator))
    box = panel(loop, path=os.path.join(tempfile.mkdtemp(), 'loop.json'))
    left = box.children[1].children[0]
    listing, named, (add, drop) = left.children[0], left.children[1].children[0], \
        left.children[2].children
    named.value = 'position'
    add.click()
    report.check('+ adds a loop and selects it', listing.value == 'position'
                 and 'position/regulator' in loop.parts)
    detail = box.children[2].children[0]
    slot = [c for c in detail.children if getattr(c, 'description', '') == 'kind']
    slot[0].value = 'LowPass'
    report.check('a slot\'s kind plugs that part in', 'position/prefilter' in loop.parts)
    drop.click()
    report.check('- takes it out', 'position' not in loop.feedbacks)


def test_velocity_is_a_feedback(report):
    device = Coaxial63100(device=True).open()
    try:
        drive = device.drive
        drive.configure(source='model')
        drive.model.configure(j=J, b=B, load=0.0)
        device.gates.on(bypass_sto=True, ignore_interlock=True)
        lane = device.motion.velocity(2.0, estimator=SpeedKalman(1.5 * 7 * 0.005, J, B, 1e5, 100.0))
        with lane:
            rpm = lane.rpm(1500.0, seconds=2.0)
        report.check('motion.velocity is a loop with one feedback, the estimator plugged in',
                     list(lane.loop.feedbacks) == ['speed']
                     and isinstance(lane.loop.feedbacks['speed'].estimator, SpeedKalman))
        report.check('and holds 1500 rpm on the stand-in within 5 %', abs(rpm - 1500.0) < 75.0,
                     '%.0f rpm' % rpm)
    finally:
        device.gates.off()
        drive.configure(source='adc')
        device.close()


def test_nodes_offer_then_configure(report):
    from coaxial.nodes import Nodes
    nodes = Nodes.discover(device=True)
    try:
        names = [n.name for n in nodes]
        report.check('every node on every bus, named where it sits',
                     len(nodes) == 20 and 'left_knee' in names and 'pelvis' in names, names[:4])
        caps = {c.name: c for c in nodes['left_knee'].capabilities('drive', 'angle')}
        report.check('a node says what it offers: ins with units, outs with ranges',
                     caps['left_knee.angle.degrees'].unit == 'deg'
                     and caps['left_knee.drive.iq_ref'].direction == 'out'
                     and caps['left_knee.drive.iq_ref'].high > 0,
                     caps['left_knee.drive.iq_ref'])
        card = nodes.card('angle', keys=('degrees',))
        report.check('the card is a line a node and direction',
                     card.count('\n') + 1 == 40 and 'left_knee.angle' not in card
                     and 'angle.degrees deg' in card, card.splitlines()[:2])
        loop = nodes.loop(inputs=['left_knee.angle'], outputs=['left_knee.drive'])
        loop.step(0.04)
        report.check('a loop over a node\'s modules reads its channels by dotted name',
                     'left_knee.angle.degrees' in loop.bus)
        try:
            nodes.loop(inputs=['left_knee.gears'])
            report.check('a module a node lacks is refused', False)
        except RigError as exc:
            report.check('a module a node lacks is refused', 'gears' in str(exc), exc)
    finally:
        nodes.close()


def test_the_body_runs_a_program(report):
    from coaxial.control.body import Body
    from coaxial.nodes import Nodes
    nodes = Nodes.discover(device=True)
    try:
        body = Body(nodes, joints=['left_hip', 'left_knee', 'right_hip', 'right_knee'])
        told = body.prompt()
        report.check('the prompt: the grammar, then each joint and what reads it back',
                     'One step a line' in told and 'set  left_hip, left_knee, right_hip, '
                     'right_knee deg -90..90  (read back as <name>.deg)' in told
                     and len(told) < 700, len(told))
        program = ('# a squat, twice\n'
                   '0.2 group=init left_hip=0 right_hip=0 left_knee=0 right_knee=0\n'
                   '2 label=down left_hip=-30 right_hip=-30 left_knee=60 right_knee=60 '
                   'left_knee.deg.H=58\n'
                   '1 left_hip=0 right_hip=0 left_knee=0 right_knee=0 goto=down times=1\n'
                   '1 group=cleanup left_hip=0 right_hip=0 left_knee=0 right_knee=0\n')
        out = Sequencer.parse(program, init=body.arm, cleanup=body.disarm).run(body.loop)
        downs = [s for s in out.steps if s[1] == 'down']
        report.check('the squat runs twice, each down ending on its level before its time',
                     out.status == 'done' and len(downs) == 2
                     and all(s[3] < 2.0 and 'past H' in s[2] for s in downs), out.summary())
        report.check('the summary is a few lines', len(out.summary('left_knee.deg')
                                                        .splitlines()) <= 9)
    finally:
        nodes.close()


def test_a_model_writes_lines(report):
    rotor = Rotor(noise=0.0)
    clock = Clock(rotor)
    loop = speed(rotor, rate_hz=50, clock=clock, sleep=clock.sleep)
    seq = Sequencer.parse('# lines, as a model answers\n'
                          '0 n=3 group=init\n'
                          '0.5 label=up w_target+=50 rotor.w.H=500\n'
                          '0 n+=-1 n.L=0 then=stop else=up\n')
    out = seq.run(loop)
    report.check('lines parse like the csv: counters, levels, jumps',
                 out.status == 'done' and loop.bus['w_target'] == 150.0, out.summary())
    for text, want in (('1 w_targt=100', 'did you mean w_target'),
                       ('1 w_target=1 rotor.ww.H=5', 'no channel rotor.ww to test'),
                       ('1 w_target 5', 'neither seconds nor name=value')):
        try:
            Sequencer.parse(text).run(loop)
            report.check('refused: %s' % want, False)
        except RigError as exc:
            report.check('refused: %s' % want, want in str(exc), exc)
    marker = Sequencer.parse('0.1 phase=2').run(loop)
    report.check('a name like no other is the program\'s own', marker.status == 'done'
                 and loop.bus['phase'] == 2.0)
    many = Sequencer.parse('0 label=a n+=1 n.H=40 then=stop else=a').run(loop)
    report.check('a long run summarises in a dozen lines',
                 len(many.summary().splitlines()) == 13, len(many.steps))


def main():
    report = Report()
    for test in (test_a_feedback_holds_a_speed, test_every_channel_is_a_float,
                 test_parts_swap_in_place, test_the_estimator_is_quieter, test_filters,
                 test_a_table_or_a_planner, test_the_sequencer, test_a_program_as_data,
                 test_save_and_load,
                 test_a_fault_ends_the_loop, test_a_paced_part_keeps_its_own_rate,
                 test_the_pictures_and_the_panel, test_velocity_is_a_feedback,
                 test_nodes_offer_then_configure, test_the_body_runs_a_program,
                 test_a_model_writes_lines):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
