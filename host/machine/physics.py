"""A body with mass under gravity: the figure in MuJoCo, each joint a drive with its own servo.

    machine = Machine.discover('gynoid', execution_mode=DYNAMIC)   # a drive a joint, `pelvis` reads
    machine.loop.write(left_knee=20.0); machine.loop.step(0.001)    # the world moves to the loop's time

A drive holds its setpoint by PD at the world's step, 1 ms, carrying it on at the rate it last
moved. The world advances when the loop reads it, to the loop's time, on the setpoints written the
pass before. The floor is y 0; only the soles, the toes, the knees and the knuckles touch it.
"""
import importlib
import math
import os

from machine.controller import Feedback
from machine.errors import MachineError
from machine.figure import CONTACTS, JOINTS, MASS_KG, SEGMENTS
from machine.machine import Actuator
from machine.nodes import Module, Node
from machine.parts import Direct, Gain
from machine.routines import TYPES

#: A drive by its joint's kind: (peak N m, kp N m/rad, kd N m s/rad, armature kg m^2). The ankles
#: stiffer than the body leaning on them (m g h, 490 N m/rad).
SERVO = {'spine': (150.0, 800.0, 30.0, 0.05), 'spine_roll': (150.0, 800.0, 30.0, 0.05),
         'waist': (80.0, 400.0, 20.0, 0.02),
         'neck': (15.0, 60.0, 2.0, 0.02), 'head': (8.0, 30.0, 1.0, 0.02),
         'shoulder': (40.0, 150.0, 6.0, 0.02), 'elbow': (25.0, 80.0, 3.0, 0.02),
         'wrist': (30.0, 150.0, 5.0, 0.02), 'gripper': (10.0, 40.0, 1.0, 0.02),
         'hip_yaw': (80.0, 300.0, 10.0, 0.05), 'hip_roll': (250.0, 900.0, 40.0, 0.05),
         'hip': (250.0, 900.0, 40.0, 0.05), 'knee': (250.0, 900.0, 35.0, 0.05),
         'ankle': (140.0, 1500.0, 40.0, 0.05), 'ankle_roll': (100.0, 1500.0, 40.0, 0.05),
         'foot': (25.0, 40.0, 1.0, 0.02)}

#: The world's step, s.
STEP_S = 0.001

#: A drive's copper loss per torque squared, W/(N m)^2: R/kt^2 of a geared joint motor. The work
#: it does is metered only where positive - a drive does not charge its battery braking.
LOSS_W = 0.01

#: The soles' friction: sliding, and turning in place (m) - a point of contact turns freely, and
#: on its ball's edge the stance foot spun under the swinging leg (2026-09-25).
FRICTION, TORSION_M = 1.0, 0.08


def kind(joint):
    """A joint's kind: its name after the side."""
    for k in ('hip_yaw', 'hip_roll', 'ankle_roll', 'spine_roll'):
        if joint.endswith(k):
            return k
    return joint.rsplit('_', 1)[-1]


def mjcf():
    """The figure as MuJoCo's XML: y up, a drive's motor on every joint, the floor's contacts."""
    kids = {}
    for seg in SEGMENTS:
        kids.setdefault(seg[1], []).append(seg)
    axes = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}

    def body(seg):
        name, _parent, joints, offset, rest, share, com, gyr = seg
        h = math.radians(rest) / 2.0
        out = ['<body name="%s" pos="%g %g %g" quat="%g 0 0 %g">' % (
            (name,) + tuple(offset) + (math.cos(h), math.sin(h)))]
        if seg[1] is None:
            out.append('<freejoint name="root"/>')
        for joint, axis, sign in joints:
            out.append('<joint name="%s" axis="%g %g %g" armature="%g"/>' % (
                (joint,) + tuple(sign * v for v in axes[axis]) + (SERVO[kind(joint)][3],)))
        mass = share * MASS_KG
        out.append('<inertial pos="%g %g %g" mass="%g" diaginertia="%g %g %g"/>' % (
            tuple(com) + (mass,) + tuple(mass * g * g for g in gyr)))
        for part, shape, size, at in CONTACTS:
            if name.endswith('_' + part):
                out.append('<geom type="%s" size="%s" pos="%g %g %g" contype="2" conaffinity="1" '
                           'condim="4" friction="%g %g 0.001"/>' % (
                               (shape, ' '.join('%g' % v for v in size)) + tuple(at)
                               + (FRICTION, TORSION_M)))
        for kid in kids.get(name, []):
            out += body(kid)
        return out + ['</body>']

    return '\n'.join(
        ['<mujoco model="gynoid">',
         '<option timestep="%g" gravity="0 -9.81 0" integrator="implicitfast"/>' % STEP_S,
         '<default><joint damping="0.3"/><geom contype="0" conaffinity="0"/></default>',
         '<worldbody>',
         '<geom name="floor" type="plane" size="100 100 0.1" quat="0.7071068 -0.7071068 0 0" '
         'contype="1" conaffinity="2"/>']
        + body(SEGMENTS[0])
        + ['</worldbody>', '<actuator>']
        + ['<motor joint="%s" ctrlrange="%g %g"/>' % (j, -SERVO[kind(j)][0], SERVO[kind(j)][0])
           for j in JOINTS]
        + ['</actuator>', '</mujoco>'])


class World:

    """The body in MuJoCo, advanced to `clock()` on the drives' setpoints (radians)."""

    def __init__(self):
        os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
        mujoco = importlib.import_module('mujoco')
        np = importlib.import_module('numpy')
        self._mj, self._np = mujoco, np
        self.model = mujoco.MjModel.from_xml_string(mjcf())
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in JOINTS])
        self.vadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in JOINTS])
        self.gains = np.array([SERVO[kind(j)][1:3] for j in JOINTS])
        self.peak = np.array([SERVO[kind(j)][0] for j in JOINTS])
        self.target = np.zeros(len(JOINTS))
        self.rate = np.zeros(len(JOINTS))
        self.was, self.stamp = self.target.copy(), 0.0
        self.pelvis = m.body('pelvis').id
        self.torso = m.body('torso').id
        self.soles = {side: {m.body(side + part).id for part in ('_foot', '_toes')}
                      for side in ('left', 'right')}
        self.push_n, self.push_until = np.zeros(3), -1.0
        #: The drives' energy since the reset: work done and work braked (J), heat (J), and
        #: torque held (N m s) - what a muscle would pay for.
        self.work = self.brake = self.heat = self.effort = 0.0
        self.clock = lambda: self.data.time

    def reset(self, degrees, where=(0.0, 1.0, 0.0), turn=(1.0, 0.0, 0.0, 0.0), rates=None,
              speed=(0.0, 0.0, 0.0)):
        """Every joint at {joint: degrees}, moving at `rates` (deg/s), its drive holding it
        there; the pelvis at `where` turned by the quaternion `turn`, moving at `speed` (m/s)."""
        d = self.data
        d.qpos[:], d.qvel[:] = 0.0, 0.0
        for i, joint in enumerate(JOINTS):
            d.qpos[self.qadr[i]] = math.radians(degrees.get(joint, 0.0))
            d.qvel[self.vadr[i]] = math.radians((rates or {}).get(joint, 0.0))
        d.qpos[0:3], d.qpos[3:7], d.qvel[0:3] = where, turn, speed
        self._mj.mj_forward(self.model, d)
        self.target[:] = d.qpos[self.qadr]
        self.was[:], self.rate[:] = self.target, d.qvel[self.vadr]
        self.work = self.brake = self.heat = self.effort = 0.0
        self.stamp = d.time

    def write(self, index, degrees):
        self.target[index] = math.radians(degrees)

    def advance(self):
        """On to the clock: every drive's PD each step, its setpoint carried on at its rate."""
        now, d = self.clock(), self.data
        if now <= d.time + 1e-9:
            return
        np, span = self._np, now - self.stamp
        if span > 1e-9:
            self.rate = (self.target - self.was) / span
        self.was, self.stamp = self.target.copy(), now
        start = d.time
        while d.time < now - 1e-9:
            ref = self.target + self.rate * (d.time - start)
            tau = (self.gains[:, 0] * (ref - d.qpos[self.qadr])
                   + self.gains[:, 1] * (self.rate - d.qvel[self.vadr]))
            d.ctrl[:] = np.clip(tau, -self.peak, self.peak)
            power = d.ctrl * d.qvel[self.vadr]
            self.work += float(power[power > 0.0].sum()) * STEP_S
            self.brake -= float(power[power < 0.0].sum()) * STEP_S
            self.heat += LOSS_W * float(d.ctrl @ d.ctrl) * STEP_S
            self.effort += float(np.abs(d.ctrl).sum()) * STEP_S
            d.xfrc_applied[self.torso, 0:3] = self.push_n if d.time < self.push_until else 0.0
            self._mj.mj_step(self.model, d)

    def push(self, force, seconds):
        """A shove on the torso, world newtons, for `seconds`."""
        self.push_n = self._np.array(force, float)
        self.push_until = self.data.time + seconds

    def angle(self, index):
        self.advance()
        d = self.data
        return math.degrees(d.qpos[self.qadr[index]]), math.degrees(d.qvel[self.vadr[index]])

    def pose(self):
        """The pelvis and the body: place, turn (quaternion), speeds (world), the centre of mass,
        each sole's load (N), and since the reset the drives' work done and braked and their
        heat (J), and the torque they held (N m s)."""
        self.advance()
        d, m = self.data, self.model
        omega = d.xmat[self.pelvis].reshape(3, 3) @ d.qvel[3:6]
        loads = {'left': 0.0, 'right': 0.0}
        force = self._np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            body = m.geom_bodyid[c.geom2] or m.geom_bodyid[c.geom1]
            for side, soles in self.soles.items():
                if body in soles:
                    self._mj.mj_contactForce(m, d, i, force)
                    loads[side] += force[0]
        com = d.subtree_com[self.pelvis]
        return dict(zip(('x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'), d.qpos[0:7].tolist()),
                    vx=d.qvel[0], vy=d.qvel[1], vz=d.qvel[2], wx=omega[0], wy=omega[1],
                    wz=omega[2], com_x=com[0], com_y=com[1], com_z=com[2],
                    left_load=loads['left'], right_load=loads['right'], t=d.time,
                    work=self.work, brake=self.brake, heat=self.heat, effort=self.effort)


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

    def _read(self):
        degrees, rate = self.world.angle(self.index)
        return {'degrees': degrees, 'rate': rate}

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
    nodes = [DriveNode(world, JOINTS.index(joint), link, unit)
             for link, subsystem in enumerate(TYPES[type].body, 1)
             for unit, joint in enumerate(subsystem.actuators, 1)]
    return nodes + [PoseNode(world)]
