"""A machine: actuators over nodes, the robot's other boards beside them, run by programs.

    machine = Machine.discover('humanoid', execution_mode=SIMULATED)  # a type over the nodes
    machine = Machine(nodes, type='quad')                   # TYPES: humanoid, quad, ...
    print(machine.prompt())        # the grammar; what a program sets and reads; its routines
    out = machine.run(program)     # checked, armed, run, disarmed however it ends
    machine.status(changed=True)   # 'now t=4.2 left_knee=30 ..': what moved, as a program says it

An actuator is a feedback on one node, of a kind the node offers (`Node.ACTUATORS`): its
setpoint is its name, it reads back as `<name>.<BACK>`. `node_hz`: an actuator whose node runs
loops (`Actuator.LOOPS`) runs its feedback there once armed, the host forwarding its setpoint. Nodes of a kind no actuator uses
are read each pass and their outputs set as `<node>.<key>`; their limits trip the run. A
type is a body and its routines - named programs a line calls: `0 run=walk times=4`. Which
board is which joint is measured (`fit`), never read from a name.
"""
import math
import time

from machine.alarms import Alarms
from machine.controller import Feedback
from machine.errors import MachineError
from machine.modes import HARDWARE
from machine.nodes import Nodes
from machine.parts import Direct
from machine.routines import TYPES
from machine.sequencer import GRAMMAR, Sequencer, card


def _mean_angle(degrees):
    """The mean of angles near each other, across the 0/360 seam."""
    s = sum(math.sin(math.radians(d)) for d in degrees)
    c = sum(math.cos(math.radians(d)) for d in degrees)
    return math.degrees(math.atan2(s, c)) % 360.0


class Actuator:

    """One kind of feedback on a node: the modules it READS and DRIVES, how it is built, armed
    and disarmed. ALIGNS: armed in steps (`ramp`, `align`), zeroed where it rests (`zero`).
    LOOPS: the node module that runs a feedback (`hand_over`), '' where the host runs it."""

    UNIT, READS, DRIVES, BACK, ALIGNS, LOOPS = '', '', '', '', False, ''

    def __init__(self, node):
        self.node = node

    def span(self):
        raise NotImplementedError

    def feedback(self, name):
        raise NotImplementedError

    def arm(self, f, arming=None):
        raise NotImplementedError

    def disarm(self):
        raise NotImplementedError

    def limits(self, name):
        low, high = self.span()
        margin = 0.1 * (high - low)
        return {'%s.%s' % (name, self.BACK): {'LL': low - margin, 'HH': high + margin}}

    def ramp(self, k, steps):
        pass

    def align(self, f):
        pass

    def zero(self):
        return None

    def hand_over(self, f, hz):
        """`f`, armed, run on the node at `hz`: its setpoint written to `<node>.<LOOPS>`."""
        raise MachineError('%s runs no loop on its node' % type(self).__name__)

    def take_back(self):
        """The node's loop stopped; the host runs the feedback again."""


def fit(nodes, body, arming=None):
    """[Subsystem] -> {name: Actuator}: subsystem i on bus i, the buses in the order found.
    On its bus a subsystem's actuators go outward in the order the boards `identify`: the
    lowest ring, the most carried, innermost. A subsystem of one takes the bus's first."""
    kinds = {s.kind for s in body}
    buses = []
    for node in nodes:
        if kinds & set(node.ACTUATORS) and node.identity.get('link') not in buses:
            buses.append(node.identity.get('link'))
    if len(buses) < len(body):
        raise MachineError('%d buses of actuators (%s), the body wants %d: %s' % (
            len(buses), ', '.join(map(str, buses)) or '-', len(body),
            ', '.join(s.name for s in body)))
    out = {}
    for subsystem, bus in zip(body, buses):
        boards = [n for n in nodes
                  if n.identity.get('link') == bus and subsystem.kind in n.ACTUATORS]
        if len(boards) < len(subsystem.actuators):
            raise MachineError('bus %s has %d boards that can be a %s; %s wants %d' % (
                bus, len(boards), subsystem.kind, subsystem.name, len(subsystem.actuators)))
        if len(subsystem.actuators) > 1:
            boards.sort(key=lambda n: n.identify(arming).get('hz', math.inf))
        out.update((name, board.actuator(subsystem.kind))
                   for name, board in zip(subsystem.actuators, boards))
    return out


class Machine:

    """Actuators by name over a Nodes, the other nodes read beside them, as one Loop: given
    as `actuators`, or by `type` - a name in TYPES. `arming` overrides each actuator's own."""

    def __init__(self, nodes, actuators=None, type=None, rate_hz=25.0, arming=None,
                 routines=None, failsafe=None, node_hz=None):
        if type is not None:
            if type not in TYPES:
                raise MachineError('no machine type %r - there are %s' % (type, ', '.join(TYPES)))
            actuators = fit(nodes, TYPES[type].body, arming)
            routines = dict(TYPES[type].routines, **(routines or {}))
            failsafe = failsafe or TYPES[type].failsafe
        self.nodes, self.actuators, self.type = nodes, dict(actuators or {}), type
        self.routines, self.arming, self.failsafe = dict(routines or {}), arming, failsafe
        self.node_hz, self._handed = node_hz, {}
        kinds = {a.node.type for a in self.actuators.values()}
        self.others = [n for n in nodes if n.type not in kinds]
        self.reads = sorted(c.name for n in self.others for c in n.capabilities()
                            if c.direction == 'in')
        self._said = {}
        inputs = sorted({'%s.%s' % (a.node.name, a.READS) for a in self.actuators.values()}
                        | {'%s.%s' % (n.name, m) for n in self.others for m in n.modules})
        outputs = sorted({'%s.%s' % (a.node.name, a.DRIVES) for a in self.actuators.values()}
                         | {'%s.%s' % (a.node.name, a.LOOPS) for a in self._on_nodes()}
                         | {'%s.%s' % (n.name, m) for n in self.others
                            for m, mod in n.modules.items() if mod.writer is not None})
        self.loop = nodes.loop(inputs=inputs, outputs=outputs, rate_hz=rate_hz)
        self.ranges, self.limits, self.units = {}, {}, {}
        for name, actuator in self.actuators.items():
            self.loop.add(name, actuator.feedback(name))
            self.ranges[name] = actuator.span()
            self.limits.update(actuator.limits(name))
            self.units[name] = actuator.UNIT
        for node in self.others:
            for m, module in node.modules.items():
                now = module.read() if module.writes and module.read else {}
                for key in module.writes:
                    channel = '%s.%s' % (node.name, key)
                    self.loop.route(**{'%s.%s.%s' % (node.name, m, key): channel})
                    self.ranges[channel] = module.ranges.get(key, (-math.inf, math.inf))
                    if key in now:              # unset, an output holds what it reads now
                        self.loop.write(**{channel: now[key]})
                for key, levels in module.limits.items():
                    self.limits['%s.%s.%s' % (node.name, m, key)] = dict(levels)
            node.couple(self)
        #: The levels, the alarm log, a stop: every program's handler (`machine.alarms`).
        self.alarms = Alarms(self.limits)

    @classmethod
    def discover(cls, type, port='COM4', execution_mode=HARDWARE, **kw):
        """Every board on every bus (`Nodes.discover`), the type over them."""
        return cls(Nodes.discover(port=port, execution_mode=execution_mode), type=type, **kw)

    # -- the run -----------------------------------------------------------------------

    def arm(self, loop=None, steps=6, settle=0.05, reads=8):
        """Every actuator on; those that align ramped over `steps`, aligned, their zeros the
        mean of `reads` readings where they rest."""
        aligns = {n: a for n, a in self.actuators.items() if a.ALIGNS}
        for name, actuator in self.actuators.items():
            actuator.arm(self.loop.feedbacks[name], self.arming)
        if aligns:
            for k in range(2, steps + 1):
                time.sleep(settle)
                for actuator in aligns.values():
                    actuator.ramp(k, steps)
            time.sleep(2.0 * settle)
            for name, actuator in aligns.items():
                actuator.align(self.loop.feedbacks[name])
            time.sleep(4.0 * settle)
            zeros = {name: [] for name in aligns}
            for _ in range(reads):
                for name, actuator in aligns.items():
                    zeros[name].append(actuator.zero())
                time.sleep(settle / 2.0)
            for name in aligns:
                self.loop.feedbacks[name].measure.configure(zero=_mean_angle(zeros[name]))
        for name in self.actuators:
            f = self.loop.feedbacks[name]
            f.prefilter.reset()
            f.regulator.reset()
        self.loop.write(**{name: 0.0 for name in self.actuators})
        for name, actuator in self.actuators.items():
            if actuator in self._on_nodes():
                self._hand_over(name, actuator)
        return list(self.actuators)

    def _on_nodes(self):
        """The actuators whose feedback their node runs."""
        return [a for a in self.actuators.values() if self.node_hz and a.LOOPS]

    def _hand_over(self, name, actuator):
        """The feedback to the node; on the host a pass-through to its setpoint, the same
        measure reading it back."""
        f = self.loop.feedbacks[name]
        actuator.hand_over(f, self.node_hz)
        self._handed[name] = f
        self.loop.add(name, Feedback(Direct(math.inf), setpoint=f.setpoint, measured=f.measured,
                                     command=name + '.set', measure=f.measure, value=f.value,
                                     sink='%s.%s.setpoint' % (actuator.node.name, actuator.LOOPS)))

    def disarm(self, loop=None):
        """Every actuator off; a feedback its node ran, the host's again."""
        for name, actuator in self.actuators.items():
            if name in self._handed:
                actuator.take_back()
                self.loop.add(name, self._handed.pop(name))
            actuator.disarm()
        return list(self.actuators)

    def prompt(self):
        """The grammar, what a program sets, what it reads: what a model is told."""
        sets = [ch for ch in self.ranges if ch not in self.actuators]
        reads = self.reads
        lines = [GRAMMAR, '', 'This machine%s:' % (' (%s)' % self.type if self.type else ''),
                 card(self.loop, self.units, self.ranges)]
        if sets:
            lines.append('set  %s' % ', '.join('%s %g..%g' % ((ch,) + self.ranges[ch])
                                               for ch in sets))
        if reads:
            lines.append('read %s' % ', '.join(reads))
        if self.routines:
            lines.append('run  ' + ', '.join('%s(%s)' % (name, ' '.join(
                '%s=%g' % kv for kv in r.defaults.items())) for name, r in self.routines.items()))
        lines.append('back now t=4.2 knee=30 ..: what changed')
        return '\n'.join(lines)

    def run(self, text, **kw):
        """A program as text: checked, armed, run, disarmed however it ends."""
        return Sequencer.parse(text, ranges=self.ranges, alarms=self.alarms, init=self.arm,
                               cleanup=self.disarm, routines=self.routines, **kw).run(self.loop)

    def status(self, changed=False):
        """Now, as a program writes it: 'now t=4.2 left_knee=30 battery.pack.volts=45.1' -
        every actuator's reading (deg and rpm whole, A to 0.1), every other read to 3 digits.
        `changed`: only what reads differently from the last status. Polls (`Loop.poll`)
        only while no pass has run: a running loop's bus is read, never its sources."""
        bus = self.loop.read()
        if any('%s.%s' % (n, a.BACK) not in bus for n, a in self.actuators.items()):
            bus = self.loop.poll()
        said = {}
        for name, a in self.actuators.items():
            v = bus.get('%s.%s' % (name, a.BACK))
            if v is not None:
                said[name] = '%.1f' % v if a.UNIT == 'A' else '%d' % round(v)
        said.update((c, '%.3g' % bus[c]) for c in self.reads if bus.get(c) is not None)
        shown = [(k, v) for k, v in said.items() if not changed or self._said.get(k) != v]
        self._said.update(said)
        return ' '.join(['now t=%.1f' % bus.get('t', 0.0)] + ['%s=%s' % kv for kv in shown])

    def pose(self):
        """Every actuator's reading now."""
        bus = self.loop.read()
        return {name: bus.get('%s.%s' % (name, a.BACK)) for name, a in self.actuators.items()}

    def close(self):
        """Every node closed."""
        self.nodes.close()
