"""A body with mass as a Machine's nodes: a drive a joint, the pelvis's pose (`machine.physics`).

    nodes = body('gynoid')       # [DriveNode .., PoseNode] on one World, its buses wired
"""
import math

from machine.controller import Feedback
from machine.errors import MachineError
from machine.figure import JOINTS
from machine.machine import Actuator
from machine.nodes import Module, Node
from machine.parts import Direct, Gain
from machine.physics import READING, World
from machine.routines import TYPES


class _Drive:

    """A drive's setpoint as a loop's sink."""

    def __init__(self, world, index):
        self.world, self.index = world, index

    def write(self, **values):
        if 'degrees' in values:
            self.world.write(self.index, float(values['degrees']))

    def off(self):
        return {'degrees': math.degrees(self.world.target[self.index])}


class DriveJoint(Actuator):

    """A joint on its drive: the setpoint passed through, within +/-`span` deg; the drive's own
    PD holds it (`SERVO`)."""

    UNIT, READS, DRIVES, BACK = 'deg', 'angle', 'drive', 'deg'

    def __init__(self, node, span=170.0):
        super().__init__(node)
        self.half = float(span)

    def span(self):
        return (-self.half, self.half)

    def feedback(self, name):
        return Feedback(Direct(self.half), setpoint=name,
                        measured=self.node.name + '.angle.degrees', command=name + '.command',
                        sink='%s.drive.degrees' % self.node.name, measure=Gain(1.0),
                        value=name + '.deg')

    def arm(self, f, arming=None):
        pass

    def disarm(self):
        pass


class DriveNode(Node):

    """One joint's drive on bus `link`, unit `unit`: its angle read, its setpoint written."""

    ACTUATORS = {'joint': DriveJoint}

    def __init__(self, world, index, link, unit):
        self.world = world
        super().__init__('D%d_%d' % (link, unit),
                         {'type': 'joint_drive', 'device': 'mujoco', 'link': link, 'unit': unit},
                         {'angle': Module(read=self._read),
                          'drive': Module(writer=_Drive(world, index), writes=('degrees',))})
        self.index = index
        world.name(self.name, index)

    def _read(self):
        return dict(zip(READING, self.world.reading(self.index)))

    def source(self, module):
        """Its angle in its world's batch (`World.drives`), read once a pass for every drive."""
        return self.world.batch if module == 'angle' else super().source(module)

    def identify(self, arming=None, again=False):
        """Outward along its bus in unit order: {'hz': unit}."""
        return {'hz': float(self.identity['unit'])}


class PoseNode(Node):

    """Where the body is: the pelvis's place and turn and speeds, the centre of mass, the soles'
    loads - an IMU and its estimator, and the soles' load cells."""

    def __init__(self, world):
        self.world = world
        super().__init__('pelvis', {'type': 'imu', 'device': 'mujoco', 'link': 'dynamic',
                                    'unit': 0}, {'pose': Module(read=world.pose)})

    def couple(self, machine):
        """The world runs on the loop's clock."""
        self.world.clock = lambda: machine.loop.bus.get('t', 0.0)

    def close(self):
        self.world.close()


def body(type):
    """[DriveNode .., PoseNode] for machine type `type`: a drive a joint, bus i subsystem i, all
    on one World."""
    if type not in TYPES:
        raise MachineError('no machine type %r - there are %s' % (type, ', '.join(TYPES)))
    wanted = [j for s in TYPES[type].body for j in s.actuators]
    missing = [j for j in wanted if j not in JOINTS]
    if missing:
        raise MachineError('%s has no figure for %s: a body with mass is the gynoid\'s'
                           % (type, ', '.join(missing)))
    world = World()
    world.wire([[JOINTS.index(joint) for joint in subsystem.actuators]
                for subsystem in TYPES[type].body])
    nodes = [DriveNode(world, JOINTS.index(joint), link, unit)
             for link, subsystem in enumerate(TYPES[type].body, 1)
             for unit, joint in enumerate(subsystem.actuators, 1)]
    return nodes + [PoseNode(world)]
