"""The application's IO: boards as nodes, what each offers, loops over their channels.

    nodes = Nodes.discover(port='COM4')                       # every family's boards on every bus
    nodes = Nodes.discover(simulated=True)                       # the stand-in robot
    nodes = Nodes([board, bms, camera])                       # nodes in hand
    nodes.capabilities('drive', 'angle')                      # [Channel], '<node>.<module>.<key>'
    print(nodes.card('angle', 'drive'))                       # the same, one line a node
    loop = nodes.loop(inputs=['left_knee.angle'], outputs=['left_knee.drive'])

A node is one board: a type, where it sits, modules of float channels, the actuators it
offers (`ACTUATORS`, by kind). A family is the module that finds its boards: `FAMILIES`.
"""
import importlib
import importlib.util
from collections import namedtuple

from machine.controller import Loop, Polled, flat
from machine.errors import MachineError
from machine.roles import Endpoint

#: One channel a node offers: its name ('<node>.<module>.<key>'), 'in' or 'out', its unit,
#: its range (None where the node sets none).
Channel = namedtuple('Channel', 'name direction unit low high')

#: Board families by the module that finds them: `discover(port, device, units, **kw)` ->
#: [Node]. Imported only when discovering, skipped where its package is not installed.
FAMILIES = ('coaxial.node',)


class Module:

    """One of a node's IO modules: a read, and the writer and keys it takes, if any;
    `ranges` bound what it takes, `limits` ({key: {'LL': ..}}) trip a run on what it reads."""

    def __init__(self, read=None, writer=None, writes=(), ranges=None, limits=None):
        self.read, self.writer, self.writes = read, writer, tuple(writes)
        self.ranges, self.limits = dict(ranges or {}), dict(limits or {})


class Node(Endpoint):

    """One board: `capabilities()` first, then `source(module)` and `sink(module)` for a
    loop, `actuator(kind)` for a machine. `identity`: type, device, where, link, unit."""

    #: Units by key, where the key says it; a family adds its own.
    UNITS = {'volts': 'V', 'amps': 'A', 'degrees': 'deg', 'seconds': 's', 'temp': 'C',
             'soc': '1'}

    #: The actuators this node can be, by kind: {'joint': Joint, ..}.
    ACTUATORS = {}

    def __init__(self, name, identity, modules):
        self.name, self.identity, self.modules = name, dict(identity), dict(modules)

    @property
    def type(self):
        return self.identity.get('type')

    def state(self):
        return dict(self.identity, name=self.name, modules=list(self.modules))

    def capabilities(self, *modules):
        """Every channel of the named modules (all by default), each read once."""
        out = []
        for name in modules or self.modules:
            module = self.modules[name]
            prefix = '%s.%s.' % (self.name, name)
            if module.read is not None:
                out += [Channel(prefix + key, 'in', self.UNITS.get(key.rpartition('.')[2], ''),
                                None, None) for key in sorted(flat(module.read() or {}))]
            out += [Channel(prefix + key, 'out', self.UNITS.get(key, ''),
                            *module.ranges.get(key, (None, None))) for key in module.writes]
        return out

    def source(self, module):
        """The module as a loop's source."""
        return Polled(self.modules[module].read)

    def sink(self, module):
        """The module as a loop's sink."""
        return self.modules[module].writer

    def actuator(self, kind, **settings):
        """This node as one of its ACTUATORS."""
        if kind not in self.ACTUATORS:
            raise MachineError('%s (%s) is no %s - it offers %s' % (
                self.name, self.type, kind, ', '.join(self.ACTUATORS) or 'none'))
        return self.ACTUATORS[kind](self, **settings)

    def couple(self, machine):
        """A stand-in's physics: what it sees of the machine it serves. A board sees the world."""

    def close(self):
        pass


class Nodes:

    """Nodes by name, in the order given."""

    def __init__(self, nodes):
        self.nodes = {}
        for node in nodes:
            if node.name in self.nodes:
                raise MachineError('two nodes named %s - name one' % node.name)
            self.nodes[node.name] = node

    @classmethod
    def discover(cls, port='COM4', simulated=False, units=range(1, 17), peripherals=None,
                 families=FAMILIES, **kw):
        """Every board of every installed family on every bus this host reaches, and the
        robot's other boards: `peripherals`, or on the stand-in its pack and camera."""
        found = []
        for family in families:
            if importlib.util.find_spec(family.partition('.')[0]) is None:
                continue
            found += importlib.import_module(family).discover(port=port, simulated=simulated,
                                                              units=units, **kw)
        if peripherals is None and simulated:
            from machine.simulated import SimulatedBms, SimulatedCamera
            peripherals = [SimulatedBms(), SimulatedCamera()]
        return cls(found + list(peripherals or ()))

    def of_type(self, kind):
        """The nodes of one type: 'bldc_inverter', 'bms', 'camera'."""
        return [node for node in self if node.type == kind]

    def offering(self, kind):
        """The nodes that can be a `kind` of actuator."""
        return [node for node in self if kind in node.ACTUATORS]

    def __getitem__(self, name):
        return self.nodes[name]

    def __iter__(self):
        return iter(self.nodes.values())

    def __len__(self):
        return len(self.nodes)

    def capabilities(self, *modules):
        return [c for node in self for c in node.capabilities(*modules)]

    def card(self, *modules, keys=None):
        """One line a node and direction: 'in' keys with units, 'out' keys with ranges.
        `keys` keeps only those keys (e.g. ('degrees', 'theta'))."""
        lines = []
        for node in self:
            lines.append('%s: %s %s at %s, %s unit %s' % (
                node.name, node.type,
                node.identity['device'], node.identity['where'],
                node.identity['link'], node.identity['unit']))
            for direction in ('in', 'out'):
                shown = []
                for c in node.capabilities(*modules):
                    key = c.name[len(node.name) + 1:]
                    if c.direction != direction or (keys and key.rpartition('.')[2] not in keys):
                        continue
                    span = ' %g..%g' % (c.low, c.high) if c.low is not None else ''
                    shown.append('%s%s%s' % (key, ' ' + c.unit if c.unit else '', span))
                if shown:
                    lines.append('  %-3s %s' % (direction, ', '.join(shown)))
        return '\n'.join(lines)

    def loop(self, inputs=(), outputs=(), **kw):
        """A Loop whose sources are the '<node>.<module>' inputs and sinks the outputs."""
        def pick(spec):
            node, _, module = spec.partition('.')
            if node not in self.nodes or module not in self.nodes[node].modules:
                raise MachineError('no module %s - nodes %s; modules %s'
                                   % (spec, ', '.join(self.nodes),
                                      ', '.join(next(iter(self)).modules) if self.nodes else '-'))
            return self.nodes[node], module
        sources = {spec: node.source(module) for spec, (node, module)
                   in ((s, pick(s)) for s in inputs)}
        sinks = {spec: node.sink(module) for spec, (node, module)
                 in ((s, pick(s)) for s in outputs)}
        return Loop(sources, sinks, **kw)

    def close(self):
        for node in self:
            node.close()
