"""A machine: actuators over nodes, the robot's other boards beside them, run by programs.

    machine = Machine.discover('humanoid', device=True)   # the nodes found, a type over them
    machine = Machine(nodes, type='quad')                 # TYPES: humanoid, quad, ...
    print(machine.prompt())        # the grammar; what a program sets and reads; its routines
    out = machine.run(program)     # checked, armed, run, disarmed however it ends

An actuator is a feedback on one node, of a kind the node offers (`Node.ACTUATORS`): its
setpoint is its name, it reads back as `<name>.<BACK>`. Nodes of a kind no actuator uses
are read each pass and their outputs set as `<node>.<key>`; their limits trip the run. A
type is its actuators and its routines - named programs a line calls: `0 run=walk times=4`.
"""
import math
import time

from machine.errors import MachineError
from machine.nodes import Nodes
from machine.routines import TYPES
from machine.sequencer import GRAMMAR, Sequencer, card


def _mean_angle(degrees):
    """The mean of angles near each other, across the 0/360 seam."""
    s = sum(math.sin(math.radians(d)) for d in degrees)
    c = sum(math.cos(math.radians(d)) for d in degrees)
    return math.degrees(math.atan2(s, c)) % 360.0


class Actuator:

    """One kind of feedback on a node: the modules it READS and DRIVES, how it is built, armed
    and disarmed. ALIGNS: armed in steps (`ramp`, `align`), zeroed where it rests (`zero`)."""

    UNIT, READS, DRIVES, BACK, ALIGNS = '', '', '', '', False

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


def assign(nodes, wanted):
    """[(name, kind)] -> {name: Actuator}, each on the next free node offering the kind;
    (None, kind): every node left that offers it, named where it sits."""
    free, out = list(nodes), {}
    for name, kind in wanted:
        offering = [n for n in free if kind in n.ACTUATORS]
        if name is None:
            out.update((n.name, n.actuator(kind)) for n in offering)
            free = [n for n in free if n not in offering]
            continue
        if not offering:
            raise MachineError('no node left offers a %s for %s - offered: %s' % (
                kind, name, ', '.join(sorted({k for n in nodes for k in n.ACTUATORS})) or '-'))
        free.remove(offering[0])
        out[name] = offering[0].actuator(kind)
    return out


class Machine:

    """Actuators by name over a Nodes, the other nodes read beside them, as one Loop: given
    as `actuators`, or by `type` - a name in TYPES. `arming` overrides each actuator's own."""

    def __init__(self, nodes, actuators=None, type=None, rate_hz=25.0, arming=None,
                 routines=None):
        if type is not None:
            if type not in TYPES:
                raise MachineError('no machine type %r - there are %s' % (type, ', '.join(TYPES)))
            actuators = assign(nodes, TYPES[type].actuators)
            routines = dict(TYPES[type].routines, **(routines or {}))
        self.nodes, self.actuators, self.type = nodes, dict(actuators), type
        self.routines, self.arming = dict(routines or {}), arming
        kinds = {a.node.type for a in self.actuators.values()}
        self.others = [n for n in nodes if n.type not in kinds]
        inputs = sorted({'%s.%s' % (a.node.name, a.READS) for a in self.actuators.values()}
                        | {'%s.%s' % (n.name, m) for n in self.others for m in n.modules})
        outputs = sorted({'%s.%s' % (a.node.name, a.DRIVES) for a in self.actuators.values()}
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
                for key in module.writes:
                    channel = '%s.%s' % (node.name, key)
                    self.loop.route(**{'%s.%s.%s' % (node.name, m, key): channel})
                    self.ranges[channel] = module.ranges.get(key, (-math.inf, math.inf))
                for key, levels in module.limits.items():
                    self.limits['%s.%s.%s' % (node.name, m, key)] = dict(levels)
            node.couple(self)

    @classmethod
    def discover(cls, type, port='COM4', device=False, **kw):
        """Every board on every bus (`Nodes.discover`), the type over them."""
        return cls(Nodes.discover(port=port, device=device), type=type, **kw)

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
        return list(self.actuators)

    def disarm(self, loop=None):
        """Every actuator off."""
        for actuator in self.actuators.values():
            actuator.disarm()
        return list(self.actuators)

    def prompt(self):
        """The grammar, what a program sets, what it reads: what a model is told."""
        sets = [ch for ch in self.ranges if ch not in self.actuators]
        reads = sorted(c.name for n in self.others for c in n.capabilities()
                       if c.direction == 'in')
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
        return '\n'.join(lines)

    def run(self, text, **kw):
        """A program as text: checked, armed, run, disarmed however it ends."""
        return Sequencer.parse(text, ranges=self.ranges, limits=self.limits, init=self.arm,
                               cleanup=self.disarm, routines=self.routines, **kw).run(self.loop)

    def pose(self):
        """Every actuator's reading now."""
        bus = self.loop.read()
        return {name: bus.get('%s.%s' % (name, a.BACK)) for name, a in self.actuators.items()}

    def close(self):
        """Every node closed."""
        self.nodes.close()
