#!/usr/bin/env python3
"""The controller: feedback loops over float channels, its parts, its panel, its sequencer."""
import importlib.util
import os
import random
import sys
import tempfile
import time

from coaxial import Coaxial63100
from machine.controller import Feedback, Loop, Paced
from machine.errors import MachineError
from machine.modes import SIMULATED
from machine.parts import Gain, LowPass, PI, Slew, SpeedKalman, SpeedPI
from machine.roles import Estimator, Input, Output, Regulator
from machine.sequencer import Sequencer
from machine.wiring import diagram, feedback

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
    except MachineError as exc:
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
    except MachineError as exc:
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

    def planner():
        while True:
            asked.append(loop.read()['w_ref'])
            if len(asked) == 4:
                return
            yield 0.5, {'w_target': asked[-1] + 20.0}

    loop.follow(planner())
    report.check('a generator plans: asked after each block, until it returns',
                 len(asked) == 4, asked)


def test_the_sequencer(report):
    folder = tempfile.mkdtemp()
    path = os.path.join(folder, 'steps.csv')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write('group,seconds,label,w_target,enable,w_ref.GE,goto,times\n'
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
    report.check('a test ends a step early, before its seconds',
                 all(s[2] == 'done' and s[3] < 3.0 for s in out.steps if s[0] == 2),
                 [(s[2], round(s[3], 2)) for s in out.steps if s[0] == 2])
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
    except MachineError as exc:
        report.check('a goto to no label is refused', 'nowhere' in str(exc), exc)


def test_a_program_as_data(report):
    """3 x 4 by counters: set, add, and branch on a level - a counter machine."""
    table = ('seconds,label,a,a+,c,c+,t,t+,a.LE,t.LE,then,else\n'
             '0,,3,,0,,,,,,,\n'
             '0,outer,,,,,,,0,,stop,\n'
             '0,,,,,,4,,,,,\n'
             '0,inner,,,,,,,,0,dec,\n'
             '0,,,,,1,,-1,,,inner,\n'
             '0,dec,,-1,,,,,,,outer,\n')
    path = os.path.join(tempfile.mkdtemp(), 'multiply.csv')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(table)
    rotor = Rotor(noise=0.0)
    loop = speed(rotor)
    out = Sequencer.read(path).run(loop)
    report.check('the table multiplies: c = 12, and no time passed',
                 loop.bus['c'] == 12.0 and out.status == 'done' and loop.bus['t'] == 0.0,
                 (loop.bus['c'], out.status, len(out.steps)))
    endless = Sequencer([{'seconds': 0, 'label': 'spin', 'then': 'spin'}], max_steps=50)
    out = endless.run(loop)
    report.check('a program that never stops is stopped by max_steps',
                 out.status == 'limit' and len(out.steps) == 50, out.reason)
    try:
        Sequencer([{'seconds': 1, 'label': 'stop'}])
        report.check('stop is not a label', False)
    except MachineError as exc:
        report.check('stop is not a label', 'stop' in str(exc), exc)


def test_the_table_refuses(report):
    """A row with no seconds, a group or a wait that is not one, a band not above 0,
    routines calling each other past 8 deep - each refused, saying which; the prompt is
    the grammar and the machine's card; a level on a channel the bus does not carry is
    left alone."""
    from machine import sequencer
    from machine.alarms import Alarms
    from machine.routines import Routine

    said = {}
    for text, routines in (('w_target=100', None), ('1 group=later', None),
                           ('1 wait=never', None), ('1 w_target=100 band=0', None),
                           ('0 run=deep', {'deep': Routine('0 run=deep', {})})):
        try:
            Sequencer.parse(text, routines=routines)
            said[text] = None
        except MachineError as exc:
            said[text] = str(exc)
    report.check('a table is refused, saying why, for a row it cannot run',
                 'needs seconds' in (said['w_target=100'] or '')
                 and 'group later' in (said['1 group=later'] or '')
                 and 'wait=never' in (said['1 wait=never'] or '')
                 and 'band=0' in (said['1 w_target=100 band=0'] or '')
                 and '8 deep' in (said['0 run=deep'] or ''), str(said))
    rotor = Rotor(noise=0.0)
    told = sequencer.prompt(speed(rotor, rate_hz=50, clock=Clock(rotor)))
    report.check('the prompt: the grammar, then this machine',
                 told.startswith(sequencer.GRAMMAR) and 'This machine:' in told)
    alarms = Alarms({'rotor.x': {'H': 1.0}})
    alarms.check({'rotor.w': 5.0})
    report.check('a level on a channel the bus does not carry is left alone',
                 not alarms.active)


def test_the_alarm_handler(report):
    """machine.alarms beside the sequencer: L and H logged once a step, LL and HH to cleanup,
    a row's level held from its row on, a timeout's alarm where the row does not branch, a
    stop from outside."""
    from machine.alarms import Alarms

    def ran(text, watch=None, **kw):
        """`text` on a rotor at rest."""
        rotor = Rotor(noise=0.0)
        clock = Clock(rotor)
        loop = speed(rotor, rate_hz=50, clock=clock, sleep=clock.sleep)
        return Sequencer.parse(text, **kw).run(loop, watch=watch)

    alarms = Alarms({'rotor.w': {'H': 80.0}})
    out = ran('1 w_target=100 wait=time\n1 w_target=100 wait=time\n1 w_target=0 wait=time',
              alarms=alarms)
    came = [a for a in out.alarms if ': H rotor.w' in a]
    went = [a for a in out.alarms if ': ok H rotor.w' in a]
    report.check('H is logged as it comes and as it goes, once each; the run goes on',
                 out.status == 'done' and len(came) == 1 and came[0].startswith('row 0')
                 and len(went) == 1 and went[0].startswith('row 2') and not alarms.active,
                 out.alarms)
    hover = Alarms({'rotor.w': {'H': 100.0}})
    ran('3 w_target=100 wait=time', alarms=hover)
    report.check('a value on its bound does not chatter: the deadband holds it',
                 len(hover.log) <= 2, hover.log)
    hot = ran('2 w_target=100\n0.5 group=cleanup w_target=0',
              alarms=Alarms({'rotor.w': {'HH': 50.0}}))
    report.check('HH trips to cleanup, the trip in the log',
                 hot.status == 'tripped' and 'HH rotor.w' in (hot.reason or '')
                 and any('HH rotor.w' in a for a in hot.alarms) and hot.steps[-1][0] == 1,
                 (hot.status, hot.reason))
    held = ran('1 w_target=30 rotor.w.HH=60\n2 w_target=100')
    report.check("a row's level holds from its row on",
                 held.status == 'tripped' and 'HH rotor.w' in (held.reason or ''), held.reason)
    cleared = ran('1 w_target=30 rotor.w.HH=60\n2 w_target=100 rotor.w.HH=inf')
    report.check('and inf clears it', cleared.status == 'done', cleared.reason)
    late = ran('0.3 w_target=500')
    report.check('a step out of time logs its timeout and goes on',
                 late.status == 'done' and any('timeout after 0.3 s' in a for a in late.alarms),
                 late.alarms)
    asked = ran('0.3 label=a w_target=500 else=b\n0 label=b')
    report.check('a row that branches takes its timeout as its answer: no alarm',
                 asked.status == 'done' and asked.steps[0][2] == 'timeout'
                 and not any('timeout' in a for a in asked.alarms), asked.alarms)
    alarms, passes = Alarms(), [0]

    def operator(_loop):
        passes[0] += 1
        if passes[0] == 20:
            alarms.stop('operator')
    stopped = ran('3 w_target=100 wait=time\n0.2 group=cleanup w_target=0', operator,
                  alarms=alarms)
    report.check('a stop from outside ends the run at the next pass, to cleanup',
                 stopped.status == 'tripped' and stopped.reason == 'stop: operator'
                 and stopped.steps[-1][0] == 1 and 'stop: operator' in stopped.alarms[0],
                 (stopped.status, stopped.reason, stopped.alarms[:1]))


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
    except MachineError as exc:
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
    if importlib.util.find_spec('ipywidgets') is None:
        report.check('the panel (ipywidgets absent: skipped)', True)
        return
    from machine.panel import kinds, panel
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
    device = Coaxial63100(execution_mode=SIMULATED).open()
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
    from machine.nodes import Nodes
    nodes = Nodes.discover(execution_mode=SIMULATED)
    try:
        names = [n.name for n in nodes]
        report.check('every node on every bus, named by bus and unit; the pack and camera too',
                     len(nodes.of_type('bldc_inverter')) == 20 and 'LL_2' in names
                     and [n.type for n in nodes][-2:] == ['bms', 'camera'], names[-3:])
        bare = Nodes.discover(execution_mode=SIMULATED, families=())
        report.check('no family, no inverters: the stand-in peripherals alone',
                     [n.type for n in bare] == ['bms', 'camera'], [n.name for n in bare])
        caps = {c.name: c for c in nodes['LL_2'].capabilities('drive', 'angle')}
        report.check('a node says what it offers: ins with units, outs with ranges',
                     caps['LL_2.angle.degrees'].unit == 'deg'
                     and caps['LL_2.drive.iq_ref'].direction == 'out'
                     and caps['LL_2.drive.iq_ref'].high > 0,
                     caps['LL_2.drive.iq_ref'])
        card = Nodes(nodes.of_type('bldc_inverter')).card('angle', keys=('degrees',))
        report.check('the card is a line a node and direction',
                     card.count('\n') + 1 == 40 and 'LL_2.angle' not in card
                     and 'angle.degrees deg' in card, card.splitlines()[:2])
        loop = nodes.loop(inputs=['LL_2.angle'], outputs=['LL_2.drive'])
        loop.step(0.04)
        report.check('a loop over a node\'s modules reads its channels by dotted name',
                     'LL_2.angle.degrees' in loop.bus)
        try:
            nodes.loop(inputs=['LL_2.gears'])
            report.check('a module a node lacks is refused', False)
        except MachineError as exc:
            report.check('a module a node lacks is refused', 'gears' in str(exc), exc)
    finally:
        nodes.close()


#: Four joints by the stand-in's wiring, named by hand.
LEGS = {'left_hip': 'LL_1', 'left_knee': 'LL_2', 'right_hip': 'RL_1', 'right_knee': 'RL_2'}


def test_fitment_by_measurement(report):
    from machine import Machine
    from machine.nodes import Nodes
    nodes = Nodes.discover(execution_mode=SIMULATED)
    try:
        humanoid = Machine(nodes, type='humanoid')
        where = {name: a.node.rig.board.system.version()['where'].replace(' ', '_')
                 for name, a in humanoid.actuators.items()}
        report.check('every joint lands where the stand-in put it, by a ring test alone',
                     len(where) == 20 and all(n == w for n, w in where.items()),
                     [(n, w, '%.1f Hz' % humanoid.actuators[n].node.identify()['hz'])
                      for n, w in where.items() if n != w])
        hz = [nodes['LL_%d' % u].identify()['hz'] for u in (1, 2, 3, 4)]
        report.check('down a leg each board rings higher: less carried outward',
                     hz == sorted(hz), ['%.1f' % h for h in hz])
        hip, knee = nodes['LL_1'], nodes['LL_2']
        hip.rig.drive.model.configure(j=2.6e-5)
        knee.rig.drive.model.configure(j=3.2e-5)
        hip.identify(again=True)
        knee.identify(again=True)
        swapped = Machine(nodes, type='humanoid')
        report.check('two loads swapped: the names follow the measurement',
                     swapped.actuators['left_hip'].node is knee
                     and swapped.actuators['left_knee'].node is hip,
                     (swapped.actuators['left_hip'].node.name,
                      swapped.actuators['left_knee'].node.name))
    finally:
        nodes.close()


def test_the_body_runs_a_program(report):
    from machine import Machine
    from machine.nodes import Nodes
    nodes = Nodes.discover(execution_mode=SIMULATED)
    try:
        body = Machine(nodes, {j: nodes[n].actuator('joint') for j, n in LEGS.items()})
        told = body.prompt()
        report.check('the prompt: the grammar, then each joint and what reads it back',
                     'One step a line' in told and 'set  left_hip, left_knee, right_hip, '
                     'right_knee deg -90..90  (read back as <name>.deg)' in told
                     and 'read battery.pack.amps' in told
                     and 'set  battery.contactor 0..1' in told and len(told) < 1100, len(told))
        program = ('# a squat, twice\n'
                   '0.2 group=init left_hip=0 right_hip=0 left_knee=0 right_knee=0\n'
                   '2 label=down left_hip=-30 right_hip=-30 left_knee=60 right_knee=60\n'
                   '1 left_hip=0 right_hip=0 left_knee=0 right_knee=0 goto=down times=1\n'
                   '1 group=cleanup left_hip=0 right_hip=0 left_knee=0 right_knee=0\n')
        out = Sequencer.parse(program, init=body.arm, cleanup=body.disarm).run(body.loop)
        downs = [s for s in out.steps if s[1] == 'down']
        report.check('the squat runs twice, each down done on arrival, before its time',
                     out.status == 'done' and len(downs) == 2
                     and all(s[3] < 2.0 and s[2] == 'done' for s in downs), out.summary())
        report.check('the summary is a few lines', len(out.summary('left_knee.deg')
                                                        .splitlines()) <= 9)
        gaps = []
        for _ in range(3):
            again = body.run(program)
            gaps.append(max(abs(r[j + '.ref'] - r[j + '.deg']) for r in again.rows
                            for j in body.actuators))
        report.check('armed again and again, no joint slips a pole (worst gap under 10 deg)',
                     max(gaps) < 10.0, ['%.1f' % g for g in gaps])
    finally:
        nodes.close()


def test_a_virtual_body_walks(report):
    """VIRTUAL: no board, the humanoid's twenty joints each where it is told, slewed; the walk
    (`machine.gait`) writes them all, each reads its command back, the type's routines run."""
    from machine import Machine, gait
    from machine.modes import VIRTUAL
    from machine.virtual import VirtualJoint
    body = Machine.discover('humanoid', execution_mode=VIRTUAL)
    report.check('twenty virtual joints, fitted bus by bus as the type names them',
                 len(body.actuators) == 20
                 and all(isinstance(a, VirtualJoint) for a in body.actuators.values())
                 and body.actuators['pelvis'].node.name == 'V1_1'
                 and body.actuators['right_foot'].node.name == 'V5_4',
                 {n: a.node.name for n, a in list(body.actuators.items())[:5]})
    body.arm()
    told = gait.walk(0.4)
    body.loop.write(**told)
    for _ in range(12):
        got = body.loop.step(0.04)
    worst = max(abs(got[j + '.deg'] - told[j]) for j in body.actuators)
    report.check('the walk written, every joint reads back where it was told', worst < 1e-9,
                 '%.3g deg' % worst)
    body.loop.write(left_knee=told['left_knee'] + 120.0)
    got = body.loop.step(0.05)
    got = body.loop.step(0.05)
    moved = got['left_knee.deg'] - told['left_knee']
    report.check('a joint slews at its rate: 120 deg asked, 75 there 0.05 s on (1500 deg/s)',
                 abs(moved - 75.0) < 1e-6, '%.3f deg' % moved)
    out = body.run('0 run=walk times=1')
    report.check('the type\'s walk routine runs on them', out.status == 'done', out.status)
    phases = [gait.walk(k / 50.0) for k in range(50)]
    report.check('the gait names every joint the body has, each within its span',
                 all(set(p) == set(body.actuators) for p in phases)
                 and all(abs(v) <= body.ranges[j][1] for p in phases for j, v in p.items()),
                 max(abs(v) for p in phases for v in p.values()))
    body.disarm()


def test_the_body_loops_on_its_boards(report):
    """node_hz: each joint's feedback runs on its board once armed; the host forwards
    setpoints and reads the joints back through the same measure."""
    from machine import Machine
    from machine.nodes import Nodes
    nodes = Nodes.discover(execution_mode=SIMULATED, peripherals=())
    try:
        body = Machine(nodes, {j: nodes[n].actuator('joint') for j, n in LEGS.items()},
                       node_hz=100)
        program = ('0.2 group=init left_hip=0 right_hip=0 left_knee=0 right_knee=0\n'
                   '2 label=down left_hip=-30 right_hip=-30 left_knee=60 right_knee=60\n'
                   '1 left_hip=0 right_hip=0 left_knee=0 right_knee=0\n'
                   '1 group=cleanup left_hip=0 right_hip=0 left_knee=0 right_knee=0\n')
        seen = {}

        def watch(loop):
            f = loop.feedbacks['left_knee']
            seen[type(f.regulator).__name__] = nodes['LL_2'].rig.ctrl.state()['running']
        out = Sequencer.parse(program, init=body.arm, cleanup=body.disarm).run(
            body.loop, watch=watch)
        down = [s for s in out.steps if s[1] == 'down']
        report.check('the squat runs on the boards, down done on arrival',
                     out.status == 'done' and down and down[0][2] == 'done', out.summary())
        report.check('armed, the host forwards: a pass-through, the board loop running',
                     seen.get('Direct') is True, seen)
        report.check('disarmed, the feedback is the host\'s again and the board loop stopped',
                     type(body.loop.feedbacks['left_knee'].regulator).__name__ == 'AngleHold'
                     and not nodes['LL_2'].rig.ctrl.state()['running'])
    finally:
        nodes.close()


def test_machine_types_and_routines(report):
    from machine import Machine
    from machine.routines import TYPES
    from machine.nodes import Nodes
    nodes = Nodes.discover(execution_mode=SIMULATED)
    try:
        for kind, program, back in (
                ('humanoid', '0 run=squat seconds=0.6\n0 run=look yaw=35', 'head.deg'),
                ('quad', '0 run=take_off seconds=0.6\n0 run=land seconds=0.6', 'rotor_fl.rpm'),
                ('fixed_wing', '0 run=take_off seconds=0.6\n0 run=bank deg=-15 seconds=0.6',
                 'aileron_l.deg'),
                ('ebike', '0 run=assist amps=3 seconds=0.6', 'assist.amps')):
            machine = Machine(nodes, type=kind)
            out = machine.run(program)
            report.check('%s: its routines run, its actuators read back' % kind,
                         out.status == 'done' and back in out.rows[-1], out.summary())
        humanoid = Machine(nodes, type='humanoid')
        report.check('the camera sees what the head turned to',
                     abs(humanoid.run('0 run=look yaw=35\n0.4').rows[-1]
                         ['head_camera.vision.target.x'] - 5.0 / 30.0) < 0.05)
        told = humanoid.prompt()
        report.check('the prompt names the type, its routines and what it reads',
                     'This machine (humanoid)' in told and 'walk(stride=20' in told
                     and 'head_camera.vision.target.x' in told, len(told))
        for text, want in (('0 run=wlak', 'did you mean walk'),
                           ('0 run=walk strides=3', 'walk takes stride'),
                           ('0 run=squat knee=120', 'outside -90..90')):
            try:
                humanoid.run(text)
                report.check('refused: %s' % want, False)
            except MachineError as exc:
                report.check('refused: %s' % want, want in str(exc), str(exc).splitlines()[-1])
        try:
            Machine(nodes, type='submarine')
            report.check('an unknown type is refused, the types named', False)
        except MachineError as exc:
            report.check('an unknown type is refused, the types named',
                         all(t in str(exc) for t in TYPES), exc)
        ebike = Machine.discover('ebike', execution_mode=SIMULATED)
        try:
            report.check('the factory: the family loaded, the type over what it found',
                         type(ebike.actuators['assist']).__module__ == 'coaxial.node'
                         and len(ebike.nodes) == len(nodes), list(ebike.actuators))
        finally:
            ebike.close()
    finally:
        nodes.close()


def test_live_from_a_stream(report):
    from machine.live import Live
    from machine import Machine
    from machine.nodes import Nodes
    nodes = Nodes.discover(execution_mode=SIMULATED)
    try:
        machine = Machine(nodes, {j: nodes[n].actuator('joint') for j, n in LEGS.items()})
        live = Live(machine, failsafe='0.6 left_knee=0 right_knee=0', timeout=0.8,
                    horizon=1.0).start()
        said = [live.send('0.4 left_knee=20 right_knee=20'),
                live.send('0.4 left_knee=4O'),
                live.send('0.4 left_knee=30 right_knee=30'),
                live.send('0.4 left_knee=40 right_knee=40')]
        report.check('chunks queue; a bad one is refused alone, in a line',
                     said[0] is None and 'left_knee=4O is not a number' in (said[1] or '')
                     and said[2] is None, said[1])
        report.check('past the horizon the writer is told to wait',
                     'send when it drains' in (said[3] or ''), said[3])
        time.sleep(2.5)
        state = live.state()
        report.check('silence past the timeout plays the failsafe',
                     state['status'] == 'failsafe' and state['played'] == 2
                     and abs(machine.pose()['left_knee']) < 5.0, state)
        report.check('each chunk\'s wait is kept: the latency', state['wait_max'] is not None)
        live.stop()
        report.check('stop: stopped, and nothing more is taken',
                     live.state()['status'] == 'stopped'
                     and 'stopped' in (live.send('0.4 left_knee=10') or ''))
        machine.limits['left_knee.deg'] = {'HH': 15.0}
        live = Live(machine, failsafe='0.4 left_knee=0', timeout=5.0).start()
        live.send('0.8 left_knee=30')
        time.sleep(1.6)
        report.check('a trip stops the machine, the failsafe played, the reason kept',
                     live.state()['status'] == 'tripped'
                     and 'left_knee.deg' in live.state()['reason'], live.state()['reason'])
        live.stop()
        machine.limits['left_knee.deg'] = {'H': 15.0}
        live = Live(machine, failsafe='0.4 left_knee=0', timeout=5.0).start()
        live.send('0.8 left_knee=30')
        woke = live.wait(low=-1.0, timeout=3.0)
        report.check('an H alarm wakes the writer, in a line, and the machine runs on',
                     woke.startswith('alarm ') and 'H left_knee.deg' in woke
                     and live.state()['status'] == 'running', woke)
        live.stop()
        del machine.limits['left_knee.deg']
    finally:
        nodes.close()


def test_a_model_streams_and_is_woken(report):
    from machine import Machine
    from machine.live import Live
    from machine.nodes import Nodes
    nodes = Nodes.discover(execution_mode=SIMULATED)
    try:
        machine = Machine(nodes, {j: nodes[n].actuator('joint') for j, n in LEGS.items()},
                          failsafe='0.6 left_knee=0 right_knee=0')
        now = machine.status()
        report.check('now is one line, as a program writes it',
                     now.startswith('now t=') and 'left_knee=' in now
                     and 'battery.pack.volts=' in now and '\n' not in now, now)
        report.check('asked again, only what changed', 'left_hip=' not in
                     machine.status(changed=True), machine.status(changed=True))
        live = Live(machine, timeout=2.0, horizon=2.0).start()
        answer = '0.6 left_knee=30 right_knee=30\n0.6 left_knee=0 right_knee=0\n'
        live.feed(answer[:20])
        unfinished = live.buffered()
        live.feed(answer[20:31])
        report.check('a line plays once finished, the rest of the answer still unwritten',
                     unfinished == 0.0 and live.buffered() > 0.3, live.buffered())
        report.check('the rest follows as it comes', live.feed(answer[31:]) == [])
        woke = live.wait(low=0.3, timeout=5.0)
        report.check('wait: one line when the buffer runs low, and what moved',
                     woke.startswith('low') and 'now t=' in woke and '\n' not in woke, woke)
        before = live.buffered()
        live.feed('0.3 label=bend left_knee=20\n0.3 left_knee=0 goto=bend times=1\n')
        tied = live.buffered()
        live.feed('\n')
        report.check('lines tied by a jump wait for their blank line, then go whole',
                     tied <= before and live.buffered() >= 0.5, (before, tied, live.buffered()))
        woke = live.wait(low=-1.0, timeout=8.0)
        report.check('silence past the timeout wakes the writer: the failsafe, in a line',
                     woke.startswith('failsafe: nothing sent'), woke)
        live.stop()
        report.check('an output no program set holds what it read: the pack stays closed',
                     'battery.pack.contactor=1' in machine.status(), machine.status()[-160:])
    finally:
        nodes.close()


def test_a_model_writes_lines(report):
    rotor = Rotor(noise=0.0)
    clock = Clock(rotor)
    loop = speed(rotor, rate_hz=50, clock=clock, sleep=clock.sleep)
    seq = Sequencer.parse('# lines, as a model answers\n'
                          '0 n=3 group=init\n'
                          '0.5 label=up w_target+=50 rotor.w.H=500\n'
                          '0 n+=-1 n.LE=0 then=stop else=up\n')
    out = seq.run(loop)
    report.check('lines parse like the csv: counters, levels, jumps',
                 out.status == 'done' and loop.bus['w_target'] == 150.0, out.summary())
    for text, want in (('1 w_targt=100', 'did you mean w_target'),
                       ('1 w_target=1 rotor.ww.H=5', 'no channel rotor.ww to test'),
                       ('1 w_target 5', 'neither seconds nor name=value')):
        try:
            Sequencer.parse(text).run(loop)
            report.check('refused: %s' % want, False)
        except MachineError as exc:
            report.check('refused: %s' % want, want in str(exc), exc)
    marker = Sequencer.parse('0.1 phase=2').run(loop)
    report.check('a name like no other is the program\'s own', marker.status == 'done'
                 and loop.bus['phase'] == 2.0)
    many = Sequencer.parse('0 label=a n+=1 n.GE=40 then=stop else=a').run(loop)
    report.check('a long run summarises in a dozen lines',
                 len(many.summary().splitlines()) == 13, len(many.steps))


def test_the_board_loops_a_joint(report):
    """A joint's feedback on device 12: rows streamed, played for their time, the last held."""
    from coaxial.devices.ctrl import dec
    from coaxial.errors import RigError
    from coaxial.node import Coaxial
    back = [int.from_bytes(b[:4], 'big', signed=True) * 10.0 ** int.from_bytes(b[4:], 'big',
                                                                               signed=True)
            for b in (dec(v) for v in (0.735, -2e-5, 90.0, 314.159265, 0.0))]
    report.check('a dec carries a parameter to float32 digits',
                 all(abs(b - v) <= 1e-7 * abs(v) for b, v in
                     zip(back, (0.735, -2e-5, 90.0, 314.159265, 0.0))), back)
    rig = Coaxial63100(execution_mode=SIMULATED).open()
    try:
        joint = Coaxial(rig, name='knee').actuator('joint')
        f = joint.feedback('knee')
        joint.arm(f)
        joint.align(f)
        time.sleep(0.3)
        f.measure.configure(zero=joint.zero())
        ctrl = rig.ctrl
        ctrl.load(f)
        ctrl.wire('angle', 'theta', 100)
        ctrl.write(0.0)
        ctrl.on()
        try:
            ctrl.slot('regulator', PI())
            report.check('a slot is refused while the loop runs', False)
        except RigError as exc:
            report.check('a slot is refused while the loop runs', 'off' in str(exc), exc)
        ctrl.rows([(0.8, 20.0), (0.8, -20.0)])
        time.sleep(0.7)
        up = ctrl.state()
        time.sleep(0.8)
        down = ctrl.state()
        time.sleep(0.5)
        held = ctrl.state()
        report.check('the shaft follows the rows, 20 deg then -20',
                     abs(up['value'] - 20.0) < 1.0 and abs(down['value'] + 20.0) < 1.0,
                     (up['value'], down['value']))
        report.check('and holds the last once they run out',
                     held['rows'] == 0 and not held['playing'] and held['played'] == 3
                     and abs(held['value'] + 20.0) < 1.0, held)
        try:
            ctrl.rows([(0.1, 0.0)] * 64)
            report.check('rows past the ring are refused, all of them', False)
        except RigError as exc:
            report.check('rows past the ring are refused, all of them',
                         'room' in str(exc) and ctrl.state()['rows'] == 0, exc)
        ctrl.off()
        report.check('off stops the ticks', not ctrl.is_on())
    finally:
        joint.disarm()
        rig.close()


def main():
    report = Report()
    for test in (test_a_feedback_holds_a_speed, test_every_channel_is_a_float,
                 test_parts_swap_in_place, test_the_estimator_is_quieter, test_filters,
                 test_a_table_or_a_planner, test_the_sequencer, test_a_program_as_data,
                 test_the_alarm_handler, test_the_table_refuses,
                 test_save_and_load,
                 test_a_fault_ends_the_loop, test_a_paced_part_keeps_its_own_rate,
                 test_the_pictures_and_the_panel, test_velocity_is_a_feedback,
                 test_nodes_offer_then_configure, test_fitment_by_measurement,
                 test_the_body_runs_a_program, test_a_virtual_body_walks,
                 test_a_model_writes_lines, test_machine_types_and_routines,
                 test_live_from_a_stream, test_a_model_streams_and_is_woken,
                 test_the_board_loops_a_joint, test_the_body_loops_on_its_boards):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
