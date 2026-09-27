"""A body with mass under gravity: the figure in MuJoCo, each joint a drive with its own servo.

    machine = Machine.discover('gynoid', execution_mode=DYNAMIC)   # a drive a joint, `pelvis` reads
    machine.loop.write(left_knee=20.0); machine.loop.step(0.001)    # the world moves to the loop's time

A drive holds its setpoint by PD at the world's step, 1 ms, carrying it on at the rate it last
moved. The world advances when the loop reads it, to the loop's time, on the setpoints written the
pass before. The floor is y 0, a slab over a plane a hole deep, its events parked out of the
way until placed (`World.terrain`); only the soles, the toes, the knees and the knuckles touch it.
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

#: The soles' give, MuJoCo's solref and solimp: a contact settles over SOLE_S s at SOLE_DAMP of
#: critical, its impedance SOLE_SOFT at a touch rising to 0.95 over SOLE_WIDTH_M of give - light
#: sneakers on her 1.60 m, 27 cm soles. Rigid (0.02, 1, 0.9, 0.001) a touchdown peaked at 1.9
#: kN, 3.5 times her weight; the give over 5 mm and the damping 1.5: 1.5 kN and the pendulum's
#: stir 1.7 -> 1.4 mm. Softer felled her first stride from standing every way: settling over
#: 0.035 s the body pitched on twice as fast (the sole a lag in the ankle's hold), damped 1.75
#: the stance foot's load flickered to 70 N as the other swung (2026-09-27).
SOLE_S, SOLE_DAMP, SOLE_SOFT, SOLE_WIDTH_M = 0.02, 1.5, 0.9, 0.005

#: The floor's events, placed on the walk's line by `World.terrain`: a hole HOLE_M deep and
#: HOLE_LONG_M long - the slab in two, the plane below showing through the gap; a sill SILL_M
#: high and SILL_LONG_M long; a patch SLIP_LONG_M long at SLIP_FRICTION; a loose rug RUG_LONG_M
#: long, RUG_M thick and RUG_KG, gripping the sole as the floor does and sliding on the floor at
#: RUG_FRICTION. Whole, the slab's halves meet at SEAM_M, past any walk. The halves are
#: compiled over the whole span and cut to size, the sill and the patch are mocap bodies: a
#: geom moved or grown past its compiled bounds is missed by the broadphase (the rug fell
#: through a slab grown 27 m, a box through a sill moved 1 m). Her toes skim at 3 cm through
#: the first 0.16 s of a swing, 0.5 m: a 2 cm sill they shoved at with 200 N and went over, 4
#: catches them; at 0.15 the patch let the stance foot creep 3 mm (the walk asks 0.17), at 0.06
#: it slid 10 cm back under the push-off; the rug at 0.3 lay still under a landing and a
#: push-off (the sole's shear 100 N, the rug's hold 165) (2026-09-27).
HOLE_M, HOLE_LONG_M, SILL_M, SILL_LONG_M = 0.03, 0.40, 0.04, 0.04
SLIP_LONG_M, SLIP_FRICTION = 0.5, 0.06
RUG_LONG_M, RUG_M, RUG_KG, RUG_FRICTION = 0.9, 0.01, 1.5, 0.1
SLAB_FROM_M, SEAM_M, SLAB_TO_M, PARKED_M = -20.0, 30.0, 80.0, -50.0


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
    give = ' solref="%g %g" solimp="%g 0.95 %g"' % (SOLE_S, SOLE_DAMP, SOLE_SOFT, SOLE_WIDTH_M)
    floor = ' contype="1" conaffinity="2"'

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
            if name == part or name.endswith('_' + part):
                out.append('<geom type="%s" size="%s" pos="%g %g %g" contype="2" conaffinity="1" '
                           'condim="4" friction="%g %g 0.001"%s/>' % (
                               (shape, ' '.join('%g' % v for v in size)) + tuple(at)
                               + (FRICTION, TORSION_M, give if part in ('foot', 'toes') else '')))
        for kid in kids.get(name, []):
            out += body(kid)
        return out + ['</body>']

    return '\n'.join(
        ['<mujoco model="gynoid">',
         '<option timestep="%g" gravity="0 -9.81 0" integrator="implicitfast"/>' % STEP_S,
         '<default><joint damping="0.3"/><geom contype="0" conaffinity="0"/></default>',
         '<worldbody>',
         '<geom name="floor" type="plane" size="100 100 0.1" pos="0 %g 0" '
         'quat="0.7071068 -0.7071068 0 0"%s/>' % (-HOLE_M, floor),
         '<geom name="slab_a" type="box" size="50 %g %g" pos="0 %g %g"%s/>' % (
             HOLE_M / 2.0, (SLAB_TO_M - SLAB_FROM_M) / 2.0, -HOLE_M / 2.0,
             (SLAB_FROM_M + SLAB_TO_M) / 2.0, floor),
         '<geom name="slab_b" type="box" size="50 %g %g" pos="0 %g %g"%s/>' % (
             HOLE_M / 2.0, (SLAB_TO_M - SLAB_FROM_M) / 2.0, -HOLE_M / 2.0,
             (SLAB_FROM_M + SLAB_TO_M) / 2.0, floor),
         '<body name="sill" mocap="true" pos="0 -1 0"><geom type="box" size="0.5 %g %g"%s/>'
         '</body>' % (SILL_M / 2.0, SILL_LONG_M / 2.0, floor),
         '<body name="slip" mocap="true" pos="0 -1 0"><geom type="box" size="0.5 0.0005 %g" '
         'priority="1" condim="4" friction="%g %g 0.001"%s%s/></body>' % (
             SLIP_LONG_M / 2.0, SLIP_FRICTION, TORSION_M, give, floor)]
        + body(SEGMENTS[0])
        + ['<body name="rug" pos="0 0.1 %g"><freejoint name="rug"/>' % PARKED_M,
           '<geom name="rug_under" type="box" size="0.3 %g %g" pos="0 %g 0" mass="%g" '
           'priority="1" friction="%g 0.005 0.0001" contype="3" conaffinity="3"/>' % (
               RUG_M / 4.0, RUG_LONG_M / 2.0, -RUG_M / 4.0, RUG_KG / 2.0, RUG_FRICTION),
           '<geom name="rug_top" type="box" size="0.3 %g %g" pos="0 %g 0" mass="%g" '
           'priority="2" condim="4" friction="%g %g 0.001" contype="3" conaffinity="3"%s/>' % (
               RUG_M / 4.0, RUG_LONG_M / 2.0, RUG_M / 4.0, RUG_KG / 2.0, FRICTION, TORSION_M,
               give),
           '</body>', '</worldbody>', '<actuator>']
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
        self._park()

    def _slab(self, a_to, b_from):
        """The slab's halves: from SLAB_FROM_M to `a_to` and from `b_from` to SLAB_TO_M, m."""
        m = self.model
        for name, z0, z1 in (('slab_a', SLAB_FROM_M, a_to), ('slab_b', b_from, SLAB_TO_M)):
            g = m.geom(name).id
            m.geom_size[g][2], m.geom_pos[g][2] = (z1 - z0) / 2.0, (z0 + z1) / 2.0

    def _park(self):
        """The floor whole, its events out of the way: the sill and the patch under the plane,
        the rug on the floor PARKED_M back."""
        self._slab(SEAM_M, SEAM_M)
        for name in ('sill', 'slip'):
            self._mocap(name, (0.0, -1.0, 0.0))
        self._rug(PARKED_M)

    def _mocap(self, name, at):
        m = self.model
        self.data.mocap_pos[m.body_mocapid[m.body(name).id]] = at

    def _rug(self, z):
        """The rug laid still on the floor, its front edge at `z`."""
        m, d = self.model, self.data
        joint = m.joint('rug').id
        adr, dof = m.jnt_qposadr[joint], m.jnt_dofadr[joint]
        d.qpos[adr:adr + 7] = (0.0, RUG_M / 2.0 + 0.001, z + RUG_LONG_M / 2.0, 1.0, 0.0, 0.0, 0.0)
        d.qvel[dof:dof + 6] = 0.0

    def terrain(self, kind, z):
        """The floor's event `kind` on the walk's line: a 'hole', a 'sill' or a 'slip' patch
        centred at world `z`, a 'rug' with its front edge there."""
        m, d = self.model, self.data
        if kind == 'hole':
            self._slab(z - HOLE_LONG_M / 2.0, z + HOLE_LONG_M / 2.0)
        elif kind == 'sill':
            self._mocap('sill', (0.0, SILL_M / 2.0, z))
        elif kind == 'slip':
            self._mocap('slip', (0.0, 0.0005, z))
        elif kind == 'rug':
            self._rug(z)
        else:
            raise MachineError('no floor event %r: hole, sill, slip or rug' % kind)
        self._mj.mj_forward(m, d)

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
        self._park()
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
            bodies = (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2])
            for side, soles in self.soles.items():
                if soles.intersection(bodies):
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
