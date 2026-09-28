"""A body with mass under gravity: the figure in MuJoCo, each joint a drive with its own servo.

    machine = Machine.discover('gynoid', execution_mode=DYNAMIC)   # a drive a joint, `pelvis` reads
    machine.loop.write(left_knee=20.0); machine.loop.step(0.001)    # the world moves to the loop's time

A drive holds its setpoint by PD at the world's step, 1 ms, carrying it on at the rate it last
moved - on its board, on its limb's bus (`machine.buses`), a process a limb in lockstep with the
world, the host's setpoints and its readings Modbus RTU frames on a socket a bus; a world with
no buses runs the drives itself. The world advances when the loop reads it, to the loop's time,
on the setpoints written the pass before. The floor is y 0, a slab over a plane a hole deep, its
events parked out of the way until placed (`World.terrain`); only the soles, the toes, the knees
and the knuckles touch it.
"""
import atexit
import importlib
import math
import os
from typing import Any

from machine import drives
from machine.drives import kind
from machine.buses import QUIET, Block, Buses
from machine.controller import Feedback
from machine import floor
from machine.errors import MachineError
from machine.figure import CONTACTS, HAIR_AT, HEM_AT, JOINTS, MASS_KG, SEGMENTS
from machine.machine import Actuator
from machine.nodes import Module, Node
from machine.parts import Direct, Gain
from machine.routines import TYPES

#: A drive by its joint's kind: (peak N m, kp N m/rad, kd N m s/rad, armature kg m^2). The ankles
#: stiffer than the body leaning on them (m g h, 490 N m/rad). The neck under a 3.3 kg head
#: 0.18 m up (0.13 kg m^2), damped at half critical: at 60 N m/rad it nodded 6.1 degrees a step
#: to her surge; 150, 300, 600: 3.0, 2.2, 1.7 (2026-09-28).
SERVO = {'spine': (150.0, 800.0, 30.0, 0.05), 'spine_roll': (150.0, 800.0, 30.0, 0.05),
         'waist': (80.0, 400.0, 20.0, 0.02),
         'neck': (15.0, 300.0, 6.0, 0.02), 'head': (8.0, 30.0, 1.0, 0.02),
         'shoulder': (40.0, 150.0, 6.0, 0.02), 'elbow': (25.0, 80.0, 3.0, 0.02),
         'wrist': (30.0, 150.0, 5.0, 0.02), 'gripper': (10.0, 40.0, 1.0, 0.02),
         'hip_yaw': (80.0, 300.0, 10.0, 0.05), 'hip_roll': (250.0, 900.0, 40.0, 0.05),
         'hip': (250.0, 900.0, 40.0, 0.05), 'knee': (250.0, 900.0, 35.0, 0.05),
         'ankle': (140.0, 1500.0, 40.0, 0.05), 'ankle_roll': (100.0, 1500.0, 40.0, 0.05),
         'foot': (25.0, 40.0, 1.0, 0.02)}

#: The world's step, s.
STEP_S = 0.001

#: How much of the drives' rotors seen through their cycloids (`drives.armature`) a joint carries
#: in place of SERVO's armature, 0 to 1; whether a joint's clamp is its drive's peak where that is
#: less (`drives.peak`), 0 or 1.
REFLECTED, CLAMPED = 0.0, 0.0

#: The boards' envelopes: 1 as built, derating and tripping them; 0 fantasy boards whose SOA
#: never binds, the heat counted - the walk, the clothes and the look are tuned on those
#: (`tests/test_gynoid.py`), the faults run on the built ones (`tests/test_gynoid_faults.py`).
ENVELOPE = 1.0

#: A board glitched (`World.glitch`): its switches' on-resistance SOA_RDS times - a gate drive
#: sagging, the FETs half on, in their SOA; or its nodes at WARM_C - run hard, hot.
SOA_RDS, WARM_C = 50.0, 100.0

#: What her clothes cover and what the cloth grips with, sliding: jeans over the pelvis, the
#: thighs and the knees, denim; a tee over the torso and the upper arms, cotton - the rest bare,
#: the soles the sneakers'. The cloth gives CLOTH_GIVE_M over its padding before it bears.
CLOTH = {'pelvis': 0.55, 'thigh': 0.55, 'shank': 0.55, 'torso': 0.45, 'upper_arm': 0.45}
CLOTH_GIVE_M = 0.004

#: A jeans' leg hangs `figure.HEM_AT` under the knee, HEM_KG HEM_M further down, on two hinges -
#: fore and aft, and aside - held to the shin by HEM_K N m/rad, damped by HEM_D N m s/rad and
#: stopped within HEM_STOP_S where its cloth meets the leg or the sneaker: its end HEM_FORE_DEG
#: forward (the heel), HEM_BACK_DEG back (the instep), HEM_SIDE_DEG aside (the ankle). It touches
#: nothing else. Free to 46 degrees it swung the leg's end through the cloth; stopped at 14,
#: softly, it swung to 19 and the shin stood 35 mm out of it aside, the sneaker 64 (2026-09-28).
HEM_M, HEM_KG, HEM_K, HEM_D = 0.2, 0.12, 0.6, 0.045
HEM_FORE_DEG, HEM_BACK_DEG, HEM_SIDE_DEG, HEM_STOP_S = 5.0, 8.0, 3.0, 0.005
HEMS = tuple('%s_hem_%s' % (side, axis) for side in ('left', 'right') for axis in 'xz')

#: Her hair's fall hangs from `figure.HAIR_AT`, HAIR_KG HAIR_M under it, on two hinges - fore and
#: aft, and aside - held by HAIR_K N m/rad and damped by HAIR_D N m s/rad: with gravity's 0.041 it
#: swings at 1.8 Hz, a third of critical. Stopped HAIR_DEG back, HAIR_IN_DEG forward and
#: HAIR_SIDE_DEG aside, where it meets her nape and her throat. It touches nothing.
HAIR_M, HAIR_KG, HAIR_K, HAIR_D = 0.07, 0.06, 0.016, 0.0035
HAIR_DEG, HAIR_IN_DEG, HAIR_SIDE_DEG = 20.0, 4.0, 8.0
HAIRS = ('hair_x', 'hair_z')

#: What hangs loose on her: the hinges `World.loose` reads.
LOOSE = HEMS + HAIRS

#: Her geoms' contact bits: 2 meets the floor's 1, 4 her own - a limb on a limb, MuJoCo leaving
#: out a segment and its parent. Floor alone, her feet passed 22 mm into each other as she
#: walked (2026-09-28).
ME, MEETS = 2 | 4, 1 | 4

#: Her segments that pass through each other: the thighs brush in her catwalk, and their spheres,
#: cruder than her, pressed up to 2 kN apart at every passing (2026-09-28).
APART = (('left_thigh', 'right_thigh'),)

#: The soles' friction: sliding, and turning in place (m) - a point of contact turns freely, and
#: on its ball's edge the stance foot spun under the swinging leg (2026-09-25).
FRICTION, TORSION_M = 1.0, 0.08

#: The soles' give, MuJoCo's solref and solimp: a contact settles over SOLE_S s at SOLE_DAMP of
#: critical, its impedance SOLE_SOFT at a touch rising to 0.95 over SOLE_WIDTH_M of give - light
#: sneakers. On 27 cm soles, rigid (0.02, 1, 0.9, 0.001) a touchdown peaked at 1.9
#: kN, 3.5 times her weight; the give over 5 mm and the damping 1.5: 1.5 kN and the pendulum's
#: stir 1.7 -> 1.4 mm. Softer felled her first stride from standing every way: settling over
#: 0.035 s the body pitched on twice as fast (the sole a lag in the ankle's hold), damped 1.75
#: the stance foot's load flickered to 70 N as the other swung (2026-09-27).
SOLE_S, SOLE_DAMP, SOLE_SOFT, SOLE_WIDTH_M = 0.02, 1.5, 0.9, 0.005




def mjcf():
    """The figure as MuJoCo's XML: y up, a drive's motor on every joint, the floor's contacts."""
    kids = {}
    for seg in SEGMENTS:
        kids.setdefault(seg[1], []).append(seg)
    axes = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}
    give = ' solref="%g %g" solimp="%g 0.95 %g"' % (SOLE_S, SOLE_DAMP, SOLE_SOFT, SOLE_WIDTH_M)
    cloth = ' solref="%g %g" solimp="%g 0.95 %g"' % (SOLE_S, SOLE_DAMP, SOLE_SOFT, CLOTH_GIVE_M)
    contacts = ' contype="1" conaffinity="2"'

    def body(seg):
        name, _parent, joints, offset, rest, share, com, gyr = seg
        h = math.radians(rest) / 2.0
        out = ['<body name="%s" pos="%g %g %g" quat="%g 0 0 %g">' % (
            (name,) + tuple(offset) + (math.cos(h), math.sin(h)))]
        if seg[1] is None:
            out.append('<freejoint name="root"/>')
        for joint, axis, sign in joints:
            out.append('<joint name="%s" axis="%g %g %g" armature="%g"/>' % (
                (joint,) + tuple(sign * v for v in axes[axis]) + (
                    SERVO[kind(joint)][3] + REFLECTED * (drives.armature(joint)
                                                         - SERVO[kind(joint)][3]),)))
        mass = share * MASS_KG
        out.append('<inertial pos="%g %g %g" mass="%g" diaginertia="%g %g %g"/>' % (
            tuple(com) + (mass,) + tuple(mass * g * g for g in gyr)))
        for part, shape, size, at in CONTACTS:
            if name == part or name.endswith('_' + part):
                felt = (give if part in ('foot', 'toes') else cloth if part in CLOTH else '')
                out.append('<geom type="%s" size="%s" pos="%g %g %g" contype="%d" '
                           'conaffinity="%d" condim="4" friction="%g %g 0.001"%s/>' % (
                               (shape, ' '.join('%g' % v for v in size)) + tuple(at)
                               + (ME, MEETS, CLOTH.get(part, FRICTION), TORSION_M, felt)))
        if name.endswith('_shank'):
            side = name[:-len('_shank')]
            out += ['<body name="%s_hem" pos="0 %g 0">' % (side, -HEM_AT)]
            out += ['<joint name="%s_hem_%s" axis="%s" stiffness="%g" damping="%g" '
                    'armature="0" limited="true" range="%g %g" solreflimit="%g 1"/>' % (
                        side, axis, direction, HEM_K, HEM_D, low, high, HEM_STOP_S)
                    for axis, direction, low, high in (
                        ('x', '1 0 0', -HEM_FORE_DEG, HEM_BACK_DEG),
                        ('z', '0 0 1', -HEM_SIDE_DEG, HEM_SIDE_DEG))]
            out += ['<inertial pos="0 %g 0" mass="%g" diaginertia="%g %g %g"/>' % (
                -HEM_M, HEM_KG, 0.02 * HEM_KG, 0.02 * HEM_KG, 0.02 * HEM_KG), '</body>']
        if name == 'head':
            out += ['<body name="hair" pos="%g %g %g">' % HAIR_AT]
            out += ['<joint name="hair_%s" axis="%s" stiffness="%g" damping="%g" armature="0" '
                    'limited="true" range="%g %g"/>' % (axis, direction, HAIR_K, HAIR_D, low, high)
                    for axis, direction, low, high in (
                        ('x', '1 0 0', -HAIR_IN_DEG, HAIR_DEG),
                        ('z', '0 0 1', -HAIR_SIDE_DEG, HAIR_SIDE_DEG))]
            out += ['<inertial pos="0 %g 0" mass="%g" diaginertia="%g %g %g"/>' % (
                -HAIR_M, HAIR_KG, 0.0025 * HAIR_KG, 0.0025 * HAIR_KG, 0.0025 * HAIR_KG),
                '</body>']
        for kid in kids.get(name, []):
            out += body(kid)
        return out + ['</body>']

    return '\n'.join(
        ['<mujoco model="gynoid">',
         '<option timestep="%g" gravity="0 -9.81 0" integrator="implicitfast"/>' % STEP_S,
         '<default><joint damping="0.3"/><geom contype="0" conaffinity="0"/></default>',
         '<worldbody>',
         ] + floor.ground(contacts, give, TORSION_M)
        + body(SEGMENTS[0])
        + floor.rug(give, FRICTION, TORSION_M)
        + ['</worldbody>', '<contact>']
        + ['<exclude body1="%s" body2="%s"/>' % pair for pair in APART]
        + ['</contact>', '<actuator>']
        + ['<motor joint="%s" ctrlrange="%g %g"/>' % (j, -max(SERVO[kind(j)][0], drives.peak(j)),
                                                      max(SERVO[kind(j)][0], drives.peak(j)))
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
        self.loose_at = np.array([m.jnt_qposadr[m.joint(j).id] for j in LOOSE])
        self.vadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in JOINTS])
        self.gains = np.array([SERVO[kind(j)][1:3] for j in JOINTS])
        self.peak = np.array([min(SERVO[kind(j)][0], drives.peak(j)) if CLAMPED
                              else SERVO[kind(j)][0] for j in JOINTS])
        #: Each drive's copper loss a torque squared, W/(N m)^2: R/kt^2 of its motor through its
        #: cycloid (`machine.drives`). The work it does is metered only where positive - a drive
        #: does not charge its battery braking.
        self.loss = np.array([drives.r_ohm(j) / drives.kt(j) ** 2 for j in JOINTS])
        self.target = np.zeros(len(JOINTS))
        self.rate = np.zeros(len(JOINTS))
        self.was, self.stamp = self.target.copy(), 0.0
        #: The buses and their block (`machine.buses`), the bus of each joint; the torque limit
        #: a step; the host's setpoints pending a broadcast and when they were written.
        self.buses: Any = None
        self.block: Any = None
        self.bus_of = {}
        self.limit = self.peak.copy()
        self.pending, self.pending_at = {}, 0.0
        self.pelvis = m.body('pelvis').id
        self.torso = m.body('torso').id
        self.soles = {side: {m.body(side + part).id for part in ('_foot', '_toes')}
                      for side in ('left', 'right')}
        self.push_n, self.push_until = np.zeros(3), -1.0
        #: Tugs on segments (`tug`): (body id, world newtons, until when).
        self.tugs = []
        #: A board glitched in its SOA (`glitch`): its index and until when.
        self.glitch_at, self.glitch_until = None, -1.0
        #: The drives' energy since the reset: work done and work braked (J), heat (J), and
        #: torque held (N m s) - what a muscle would pay for.
        self.work = self.brake = self.heat = self.effort = 0.0
        #: Where the world has stepped to, s: `advance` returns on it without touching MjData.
        self.at = 0.0
        self.clock = lambda: self.data.time
        self._park()

    def wire(self, limbs):
        """The boards on their buses: a bus a limb, `limbs` [[joint index, ..], ..], their
        processes in lockstep with the world over the block."""
        self.block = Block(len(JOINTS), len(limbs))
        self.block.gains[:] = self.gains.ravel()
        self.block.limit[:] = self.limit
        self.block.air[:] = self.block.rds[:] = self._np.ones(len(JOINTS))
        self.block.envelope[0] = ENVELOPE
        self.block.drive[:] = self._np.array([drives.heat(j) for j in JOINTS]).ravel()
        self.buses = Buses(self.block, limbs)
        self.bus_of = self.buses.of
        atexit.register(self.close)

    def close(self):
        """The buses' processes over, the block released."""
        if self.buses is not None:
            self.buses.close()
            self.block.close()
            self.buses, self.block, self.bus_of = None, None, {}

    def _park(self):
        floor.park(self)

    def terrain(self, kind, z):
        """The floor's event `kind` on the walk's line (`floor.place`)."""
        floor.place(self, kind, z)
        self._mj.mj_forward(self.model, self.data)

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
        self.stamp, self.at, self.glitch_at, self.pending = d.time, d.time, None, {}
        self.push_until, self.tugs = -1.0, []
        if self.buses is not None:
            self.buses.drain()
            self.block.hold[:] = self.target
            self.block.air[:] = self.block.rds[:] = self._np.ones(len(JOINTS))
            self.block.warm[:] = self._np.zeros(len(JOINTS))
            self.block.epoch[0] += 1
            for bus in self.buses.each:
                for i in bus.indices:
                    bus.hold(i, math.degrees(self.target[i]))

    def write(self, index, degrees):
        """A setpoint: to its board over the bus, with the pass's others (`advance`)."""
        self.target[index] = math.radians(degrees)
        self.pending[index], self.pending_at = degrees, self.clock()

    def advance(self):
        """On to the clock, a step at a time: the pass's setpoints broadcast on each bus and
        every board polled, the boards' loops a lockstep, the world stepped on their torques."""
        now = self.clock()
        if now <= self.at + 1e-9:
            return
        d = self.data
        np = self._np
        if self.buses is not None:
            for bus in self.buses.each:
                mine = ({i: self.pending.get(i, math.degrees(self.target[i]))
                         for i in bus.indices}
                        if any(i in self.pending for i in bus.indices) else {})
                bus.send(self.pending_at, now, mine)
            self.pending = {}
        else:
            span = now - self.stamp
            if span > 1e-9:
                self.rate = (self.target - self.was) / span
            self.was, self.stamp = self.target.copy(), now
        start = d.time
        while d.time < now - 1e-9:
            if self.buses is not None:
                b = self.block
                if self.glitch_at is not None and d.time >= self.glitch_until:
                    b.rds[self.glitch_at], self.glitch_at = 1.0, None
                b.time[0] = d.time
                b.q[:], b.qd[:] = d.qpos[self.qadr], d.qvel[self.vadr]
                self.buses.step()
                d.ctrl[:] = b.ctrl
            else:
                ref = self.target + self.rate * (d.time - start)
                tau = (self.gains[:, 0] * (ref - d.qpos[self.qadr])
                       + self.gains[:, 1] * (self.rate - d.qvel[self.vadr]))
                d.ctrl[:] = np.clip(tau, -self.limit, self.limit)
            power = d.ctrl * d.qvel[self.vadr]
            self.work += float(power[power > 0.0].sum()) * STEP_S
            self.brake -= float(power[power < 0.0].sum()) * STEP_S
            self.heat += float(self.loss @ (d.ctrl * d.ctrl)) * STEP_S
            self.effort += float(np.abs(d.ctrl).sum()) * STEP_S
            d.xfrc_applied[:, 0:3] = 0.0
            d.xfrc_applied[self.torso, 0:3] = self.push_n if d.time < self.push_until else 0.0
            for body, force, until in self.tugs:
                if d.time < until:
                    d.xfrc_applied[body, 0:3] += force
            self._mj.mj_step(self.model, d)
        self.at = d.time
        if self.buses is not None:
            self.buses.drain()

    def props(self):
        """What lies on the floor (`floor.props`), and a lace caught: ('lace', from, to) from the
        pulled foot to the other while the tug holds it."""
        m, d = self.model, self.data
        out = floor.props(self)
        if any(until > d.time for _b, _f, until in self.tugs):
            out.append(('lace', tuple(d.xpos[m.body('left_toes').id]),
                        tuple(d.xpos[m.body('right_foot').id])))
        return out

    def push(self, force, seconds):
        """A shove on the torso, world newtons, for `seconds`."""
        self.push_n = self._np.array(force, float)
        self.push_until = self.data.time + seconds

    def tug(self, segment, force, seconds):
        """A pull on `segment` at its origin, world newtons, for `seconds`: a lace caught."""
        self.tugs = [t for t in self.tugs if t[2] > self.data.time] + [
            (self.model.body(segment).id, self._np.array(force, float),
             self.data.time + seconds)]

    def glitch(self, joint, kind, seconds=0.0):
        """Drive `joint`'s board glitched: 'soa' its switches SOA_RDS times their on-resistance
        for `seconds`, 'hot' its nodes at WARM_C at once. What it does of it, and says of it,
        is its own (`machine.heat`)."""
        if self.buses is None:
            raise MachineError('a glitch is a board\'s: this world has no buses')
        i = JOINTS.index(joint)
        if kind == 'soa':
            if self.glitch_at is not None:
                self.block.rds[self.glitch_at] = 1.0
            self.block.rds[i], self.glitch_at = SOA_RDS, i
            self.glitch_until = self.data.time + seconds
        elif kind == 'hot':
            self.block.warm[i] = WARM_C
        else:
            raise MachineError('no glitch %r: soa or hot' % kind)

    def arm(self, index):
        """A joint's board's gates on again: the host's gate write, with the next pass."""
        if index in self.bus_of:
            self.bus_of[index].arm(index)

    def reading(self, index):
        """(degrees, deg/s, C, spent, derate, status) of a joint's drive as its board last
        answered the host (`machine.buses`) - with no bus the world's own angle, cool."""
        self.advance()
        if index in self.bus_of:
            return self.bus_of[index].reading(index)
        d = self.data
        return ((math.degrees(d.qpos[self.qadr[index]]), math.degrees(d.qvel[self.vadr[index]]))
                + QUIET[2:])

    def angle(self, index):
        """(degrees, deg/s) of a joint as its board last answered the host."""
        return self.reading(index)[:2]

    def gap(self, one, other, reach=0.3):
        """The least distance, m, between the geoms of segments `one` and those of `other`
        (negative: into each other), `reach` at most."""
        m, mj = self.model, self._mj
        of = self._geoms = getattr(self, '_geoms', {})
        for names in (one, other):
            if names not in of:
                ids = {m.body(n).id for n in names}
                of[names] = [g for g in range(m.ngeom) if m.geom_bodyid[g] in ids]
        return min(mj.mj_geomDistance(m, self.data, a, b, reach, None)
                   for a in of[one] for b in of[other])

    def lifted(self, names):
        """The least distance, m, from segments `names`' geoms to the floor's slab."""
        m, mj = self.model, self._mj
        of = self._geoms = getattr(self, '_geoms', {})
        if names not in of:
            ids = {m.body(n).id for n in names}
            of[names] = [g for g in range(m.ngeom) if m.geom_bodyid[g] in ids]
        slab = [m.geom(n).id for n in ('slab_a', 'slab_b')]
        return min(mj.mj_geomDistance(m, self.data, a, b, 0.5, None)
                   for a in of[names] for b in slab)

    def loose(self):
        """{hinge: degrees} of what hangs loose on her (`LOOSE`): the jeans' legs from her
        shins, her hair from her head."""
        return dict(zip(LOOSE, map(math.degrees, self.data.qpos[self.loose_at])))

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
        degrees, rate, celsius, spent, derate, status = self.world.reading(self.index)
        return {'degrees': degrees, 'rate': rate, 'celsius': celsius, 'spent': spent,
                'derate': derate, 'status': status}

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
