"""Virtual actuators: each joint where its command puts it, slewed - no board, no physics.

    machine = Machine.discover('humanoid', execution_mode=VIRTUAL)   # a joint an actuator
    machine.arm()
    machine.loop.write(left_knee=30.0)          # the API a program or a page drives
    machine.loop.step(0.04)                     # left_knee.deg follows at deg_s

A body's subsystem i is bus i and its actuators units 1.. in order, so `fit` names them as the
type does. Joints only: a rotor or a surface is refused.
"""
from machine.controller import Feedback
from machine.errors import MachineError
from machine.machine import Actuator
from machine.nodes import Module, Node
from machine.parts import Direct, Gain, Slew
from machine.routines import TYPES


class _Angle:

    """A virtual node's angle as a loop's sink: written, it is there."""

    def __init__(self, node):
        self.node = node

    def write(self, **values):
        if 'degrees' in values:
            self.node.degrees = float(values['degrees'])

    def off(self):
        return {'degrees': self.node.degrees}


class VirtualJoint(Actuator):

    """A joint at its command: the setpoint slewed at `deg_s`, within +/-`span` deg. 1500: a
    brisk walk's knee swings at 800; at 360 the read-back lagged 19 deg (2026-09-25)."""

    UNIT, READS, DRIVES, BACK = 'deg', 'angle', 'angle', 'deg'

    def __init__(self, node, span=150.0, deg_s=1500.0):
        super().__init__(node)
        self.half, self.deg_s = float(span), float(deg_s)

    def span(self):
        return (-self.half, self.half)

    def feedback(self, name):
        return Feedback(Direct(self.half), setpoint=name,
                        measured=self.node.name + '.angle.degrees', command=name + '.command',
                        sink='%s.angle.degrees' % self.node.name, prefilter=Slew(self.deg_s),
                        measure=Gain(1.0), ref=name + '.ref', value=name + '.deg')

    def arm(self, f, arming=None):
        pass

    def disarm(self):
        pass

    def zero(self):
        return self.node.degrees


class VirtualNode(Node):

    """One virtual joint on bus `link`, unit `unit`: an angle module, read and written."""

    ACTUATORS = {'joint': VirtualJoint}

    def __init__(self, link, unit):
        self.degrees = 0.0
        super().__init__('V%d_%d' % (link, unit),
                         {'type': 'virtual_joint', 'device': 'virtual', 'link': link,
                          'unit': unit},
                         {'angle': Module(read=lambda: {'degrees': self.degrees},
                                          writer=_Angle(self), writes=('degrees',))})

    def identify(self, arming=None, again=False):
        """Outward along its bus in unit order: {'hz': unit}."""
        return {'hz': float(self.identity['unit'])}


def body(type):
    """[VirtualNode] for machine type `type`'s body: a node an actuator, bus i its subsystem i."""
    if type not in TYPES:
        raise MachineError('no machine type %r - there are %s' % (type, ', '.join(TYPES)))
    nodes = []
    for link, subsystem in enumerate(TYPES[type].body, 1):
        if subsystem.kind != 'joint':
            raise MachineError('%s: virtual actuators are joints; %s wants %ss'
                               % (type, subsystem.name, subsystem.kind))
        nodes += [VirtualNode(link, unit) for unit in range(1, len(subsystem.actuators) + 1)]
    return nodes
