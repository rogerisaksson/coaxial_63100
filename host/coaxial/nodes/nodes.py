"""The application's IO: an array of nodes, what they offer, and loops over their channels.

    nodes = Nodes.of(Coaxial63100(port='COM4').open())       # one board today
    nodes = Nodes.discover(device=True)                       # every node on every bus
    nodes.capabilities('drive', 'angle')                      # [Channel], '<node>.<module>.<key>'
    print(nodes.card('angle', 'drive'))                       # the same, one line a node
    loop = nodes.loop(inputs=['left_knee.angle'], outputs=['left_knee.drive'])
"""
from coaxial.control.controller import Loop
from coaxial.errors import RigError
from coaxial.nodes.node import Node


class Nodes:

    """Nodes by name, in the order given."""

    def __init__(self, nodes):
        self.nodes = {}
        for node in nodes:
            if node.name in self.nodes:
                raise RigError('two nodes named %s - name one: Node(rig, name=...)' % node.name)
            self.nodes[node.name] = node

    @classmethod
    def of(cls, *rigs):
        """Opened rigs, each a node named where it sits."""
        return cls([Node(rig) for rig in rigs])

    @classmethod
    def discover(cls, port='COM4', device=False, units=range(1, 17), **kw):
        """Every node answering on every bus this host reaches, each opened on its own."""
        from coaxial import Coaxial63100
        first = Coaxial63100(port=port, device=device, **kw).open()
        found = [(bus, unit) for bus, _ in first.session.buses()
                 for unit, _ in first.session.scan(units, bus)]
        first.close()
        return cls.of(*(Coaxial63100(port=bus, unit=unit, device=device, **kw).open()
                        for bus, unit in found))

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
            lines.append('%s: %s at %s, %s unit %s' % (
                node.name, node.identity['device'], node.identity['where'],
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
                raise RigError('no module %s - nodes %s; modules %s'
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
