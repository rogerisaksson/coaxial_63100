"""A body with mass under gravity: the figure in MuJoCo, each joint a drive with its own servo.

    machine = Machine.discover('gynoid', execution_mode=DYNAMIC)   # its nodes: `machine.dynamic`
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
import ctypes
import importlib
import math
import os
from typing import Any

from machine import drives, gearbox, heat, linkage, room
from machine.drives import kind
from machine.buses import QUIET, Block, Buses
from machine.controller import Batch
from machine.errors import MachineError
from machine import floor
from machine.mjcf import LACE_HOLD_N, LACE_M, LOOSE, mjcf
from machine.servos import CLAMPED, SERVO
from machine.figure import JOINTS

#: The world's step, s.
STEP_S = 0.001

#: Each sole's load as its sensor has it: the solver's through a first-order band of LOAD_S s.
#: As the solver had it a pass, standing still it flickered 0-950 N, a foot at 0 N 104 passes
#: a second. At 20 ms the feet stand on 159 and 160 N, sd 0.02; the walk's touchdown 769 -> 253 N,
#: its power 683 -> 560 W (docs/findings/stacks.md).
LOAD_S = 0.02

#: A drive's reading, in `World.reading`'s order.
READING = ('degrees', 'rate', 'celsius', 'spent', 'derate', 'status')

#: The drives as built, each 0 or 1 (`machine.mjcf`): how much of their rotors and gearboxes
#: seen through them (`drives.armature`) a joint carries in place of SERVO's armature, how much
#: of that its board feeds forward on its setpoint's acceleration (`buses.ACCEL_S`); whether a
#: joint's clamp is its drive's peak where that is less (`drives.peak`); whether the assemblies
#: sit where they are, bodies of their own, their segments the lighter; how much of their
#: gearboxes' drag the joints carry, Coulomb (`drives.backdrive`); whether her skeleton collides,
#: its drums, boards, bones, rods and belts (`machine.skeleton`) - off: the ankle's one crank
#: 50-60 mm inside her shins met the other's walking and she fell in 1-3 s; on the parallel pair
#: the scoreboard 542 against 495 without, chance's 150, but shoved past saving into the crouch
#: her head met the floor at 1.03-1.66 m/s in 3 of 64 falls, off at 0.66-0.81 in 2.
#: ROTOR_FF: fed forward whole a standing start asked the knees and hips their clamps and she
#: fell back in 3 starts of 12; at 0.5 and up off her form.
REFLECTED, ROTOR_FF, PLACED, BACKDRIVE, SKELETON = 1.0, 0.25, 1.0, 1.0, 0.0
#: Where a drive's loop reads its angle and speed: 1 the motor's encoder, the rotor's side of
#: the gearbox; 0 the joint's own sensor, its link's.
WOUND = 1.0

#: The gearboxes between the rotors and the links (`machine.gearbox`): 1, each rotor runs free
#: across its play, winds the box and the structure up past it against the mesh's friction, and
#: the link is turned by that wind-up alone; 0, rigid, the rotor's inertia on the link's joint.
#: On the boxes the walk on setpoints drags its toes 7-19 mm at lift where 1 rigid and its form's
#: 2, the loop on its torques walks them at 265-326 J/m (docs/findings/drives.md).
#: ROOM: the living room round her (`machine.room`); SIGHT her eyes on it (`machine.sight`).
BOXED, ROOM, SIGHT = 0.0, 0.0, 0.0

#: Whether a drive on a gimbal's stage (`drives.mount`: the hip's roll M, its pitch L) rides it,
#: turning with the leg's yaw and roll, or the segment that stage hangs from - on them, 0.7 kg of
#: yaw's 8 % more inertia.
STAGED = 1.0

#: The boards' envelopes: 1 as built, derating and tripping them; 0 fantasy boards whose SOA
#: never binds, the heat counted (`tests/test_gynoid.py`).
ENVELOPE = 1.0

#: A board glitched (`World.glitch`): its switches' on-resistance SOA_RDS times - a gate drive
#: sagging, the FETs half on -, or its nodes at WARM_C.
SOA_RDS, WARM_C = 50.0, 100.0

#: Her harness (`World.hang`): N/m and N s/m on her pelvis's place, N m/rad and N m s/rad on its
#: turn - 0.7 of critical on her 31 kg and 3 kg m^2.
HANG = (4000.0, 500.0, 400.0, 50.0)


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
        self.peak = np.array([0.0 if drives.passive(j) else min(SERVO[kind(j)][0], drives.peak(j))
                              if CLAMPED else SERVO[kind(j)][0] for j in JOINTS])
        #: Each drive's copper loss a torque squared, W/(N m)^2: R/kt^2 of its motor through its
        #: cycloid (`machine.drives`), a pair's two. The work it does is metered only where
        #: positive - a drive does not charge its battery braking.
        self.loss = np.array([drives.motors(j) * drives.r_ohm(j) / drives.kt(j) ** 2
                              for j in JOINTS])
        #: Each joint's N m an amp (`drives.kt`); each parallel pair, (joint, its roll): each's
        #: current the first's share plus or less the second's, within its own board's (wbc_pass.c).
        self.kt = np.array([drives.kt(j) for j in JOINTS])
        self.pairs = np.array([(i, JOINTS.index(j[:-len(kind(j))] + linkage.PAIRS[kind(j)]))
                               for i, j in enumerate(JOINTS) if kind(j) in linkage.PAIRS],
                              int).reshape(-1, 2)
        #: The drives whose phases are shorted (`short`), and kt^2/R each gives a rad/s of its
        #: joint's speed against it, N m s/rad: the joint's own damping while shorted, MuJoCo's,
        #: integrated implicitly - as a torque each step against the speed, 140 N m s/rad on an
        #: ankle's 0.05 kg m^2 flipped its speed each step, +-140 N m at 500 Hz, drawing 794 W
        #: as she lay still.
        self.shorted = np.zeros(len(JOINTS), bool)
        #: The drives cut (`off`): gates off, no torque, nothing switched; a cut board's check
        #: current (`test`), A, and its winding's copper an amp squared (1.5 r), ohm.
        self.cut, self.testing = np.zeros(len(JOINTS), bool), np.zeros(len(JOINTS))
        self.ohm = np.array([drives.r_ohm(j) for j in JOINTS])
        #: The joints on a stroke's curve: (index, joint, SERVO's armature, its own damping).
        self.strokes = [(i, j, SERVO[kind(j)][3], float(m.dof_damping[self.vadr[i]]))
                        for i, j in enumerate(JOINTS)
                        if kind(j) in linkage.RODS or kind(j) in linkage.PLANAR]
        #: The joints a drive turns, not held or sprung (`drives.passive`).
        self.driven = np.array([not drives.passive(j) for j in JOINTS])
        self.damping = 1.0 / self.loss * self.driven
        self.free = m.dof_damping[self.vadr].copy()
        self.target = np.zeros(len(JOINTS))
        self.rate = np.zeros(len(JOINTS))
        #: A torque a joint fed forward onto its PD, N m - the whole-body law's (`machine.wbc`):
        #: a world without buses alone.
        self.feed = np.zeros(len(JOINTS))
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
        self.head = m.body('head').id
        #: Each body's sole, a column a side (left, right): its contacts bear on that side.
        self.soles = np.zeros((m.nbody, 2), bool)
        for k, side in enumerate(('left', 'right')):
            self.soles[[m.body(side + part).id for part in ('_foot', '_toes')], k] = True
        #: The named drives' channels, '<node>.angle.<key>' (READING), and their joints: one
        #: batch a pass (`drives`).
        self.keys, self.named = [], []
        self.batch = Batch(self.drives)
        self.push_n, self.push_until, self.pushed = np.zeros(3), -1.0, False
        #: Her pelvis's place and turn hung in a harness (`hang`), or None.
        self.hung: Any = None
        #: The lace (`lace`), and whether it is snagged.
        self.lace_at, self.laced = m.tendon('lace').id, False
        #: A board glitched in its SOA (`glitch`): its index and until when.
        self.glitch_at, self.glitch_until = None, -1.0
        #: The drives' energy since the reset: work done and work braked (J), heat (J), and
        #: torque held (N m s) - what a muscle would pay for.
        self.work = self.brake = self.heat = self.effort = 0.0
        #: Each gearbox's peak torque since the reset, N m: its drive's less its rotor's
        #: (`drives.shock` what it takes).
        self.armature = m.dof_armature[self.vadr].copy()
        self.geared = np.zeros(len(JOINTS))
        #: The gearboxes (BOXED, `machine.gearbox`), their rotors' inertia off the joints'
        #: armature; the motors' torques this step.
        self.boxes = gearbox.Boxes(np)
        self.motor = np.zeros(len(JOINTS))
        if BOXED:
            m.dof_armature[self.vadr[self.driven]] = [SERVO[kind(j)][3] for j in JOINTS
                                                      if not drives.passive(j)]
            self.armature = m.dof_armature[self.vadr].copy()
        #: Each sole's load as last read (`pose`, LOAD_S), and when; None since the reset.
        self.borne, self.borne_at = [0.0, 0.0], None
        #: Where the world has stepped to, s: `advance` returns on it without touching MjData.
        self.at = 0.0
        self.clock = lambda: self.data.time
        #: The pass in C (wbc_pass.c, `tools.cores.wbc.World`): its struct pointing at the
        #: arrays above and MuJoCo's once, its calls bound.
        self.passing = self._laid()
        self._park()

    def _laid(self):
        from tools.cores import wbc as cores
        np, d = self._np, self.data
        self._kept = tuple(np.ascontiguousarray(a, dtype=np.int32) for a in (self.qadr, self.vadr, self.pairs))
        p = cores.World(n=len(JOINTS), npairs=len(self.pairs), pelvis=self.pelvis, wound=WOUND)
        for name, a in zip(('qadr', 'vadr', 'pairs'), self._kept):
            setattr(p, name, cores.pointer(a, np.int32))
        for name, a in (('qpos', d.qpos), ('qvel', d.qvel), ('qacc', d.qacc), ('xmat', d.xmat), ('ctrl', d.ctrl),
                        ('target', self.target), ('rate', self.rate), ('feed', self.feed), ('gains', self.gains),
                        ('limit', self.limit), ('kt', self.kt), ('delta', self.boxes.delta), ('zero', self.boxes.zero),
                        ('armature', self.armature), ('motor', self.motor), ('geared', self.geared)):
            setattr(p, name, cores.pointer(a))
        for name, a in (('shorted', self.shorted), ('cut', self.cut)):
            setattr(p, name, cores.pointer(a, np.bool_))
        lib = cores.library()
        self._ctrl, self._motor, self._metered = lib.wbh_pass_ctrl, lib.wbh_pass_motor, lib.wbh_pass_metered
        self._drawn = lib.wbh_pass_drawn
        self._drawn_of = (cores.pointer(self.loss), cores.pointer(self.driven, np.bool_), cores.pointer(self.testing),
                          cores.pointer(self.ohm))
        return p

    def wire(self, limbs):
        """The boards on their buses: a bus a limb, `limbs` [[joint index, ..], ..], their
        processes in lockstep with the world over the block."""
        self.block = Block(len(JOINTS), len(limbs))
        self.block.gains[:] = self.gains.ravel()
        self.block.limit[:] = self.limit
        self.block.air[:] = self.block.rds[:] = self._np.ones(len(JOINTS))
        self.block.envelope[0] = ENVELOPE
        self.block.drive[:] = self._np.array([drives.heat(j) for j in JOINTS]).ravel()
        self.block.rotor[:] = self._np.array([REFLECTED * ROTOR_FF * drives.armature(j)
                                              for j in JOINTS])
        self.block.scale[:] = self._np.ones(len(JOINTS))
        self.block.emf[:] = self._np.array([drives.emf(j) for j in JOINTS])
        self.block.ohm[:] = self._np.array([drives.of(j)[1].r for j in JOINTS])
        self.block.volts[0] = drives.PACK_V / math.sqrt(3.0)
        for i, r in self.pairs:
            self.block.pair[i], self.block.pair[r] = r + 1, -(i + 1)
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

    def terrain(self, kind, z, x=0.0, heading=0.0):
        floor.place(self, kind, z, x, heading)
        self._mj.mj_forward(self.model, self.data)

    def scenario(self, name):
        floor.scenario(self, name)
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
        self.shorted[:] = self.cut[:] = False
        self.testing[:] = 0.0
        self.model.dof_damping[self.vadr] = self.free
        self.target[:] = d.qpos[self.qadr]
        self.was[:], self.rate[:] = self.target, d.qvel[self.vadr]
        self.work = self.brake = self.heat = self.effort = 0.0
        self.passing.effort = self.passing.work = self.passing.brake = 0.0
        self.geared[:] = 0.0
        self.boxes.reset(d.qpos[self.qadr])
        self.motor[:] = 0.0
        self.borne_at = None
        self.stamp, self.at, self.glitch_at, self.pending = d.time, d.time, None, {}
        self.push_until, self.pushed, self.laced = -1.0, False, False
        self.hung, d.xfrc_applied[:] = None, 0.0
        self.model.tendon_range[self.lace_at] = (0.0, 10.0)
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
                np.divide(self.target - self.was, span, out=self.rate)
            self.was[:], self.stamp = self.target, now
        start, p = d.time, self.passing
        while d.time < now - 1e-9:
            if self.buses is not None:
                b = self.block
                if self.glitch_at is not None and d.time >= self.glitch_until:
                    b.rds[self.glitch_at], self.glitch_at = 1.0, None
                b.time[0] = d.time
                b.q[:], b.qd[:] = d.qpos[self.qadr], d.qvel[self.vadr]
                b.delta[:] = (WOUND * self.boxes.delta if BOXED else self.boxes.rigid(np, b.q)) + self.boxes.zero
                b.delta_d[:] = WOUND * (self.boxes.w - b.qd) if BOXED else np.zeros(len(JOINTS))
                b.ahead[:] = WOUND * self.boxes.ahead(np, self.motor) if BOXED else np.zeros(len(JOINTS))
                self._stroked()
                self.buses.step()
                d.ctrl[:] = b.ctrl
            elif BOXED:
                ahead = WOUND * self.boxes.ahead(np, self.motor)
                wrate = WOUND * (self.boxes.w - d.qvel[self.vadr])
                p.delta = self.boxes.delta.ctypes.data
                self._ctrl(ctypes.byref(p), d.time - start, ahead.ctypes.data, wrate.ctypes.data)
            else:
                self._ctrl(ctypes.byref(p), d.time - start, None, None)
            self._motor(ctypes.byref(p), STEP_S)
            if BOXED:
                d.ctrl[:] = self.boxes.step(np, d.ctrl, d.qpos[self.qadr],
                                            d.qvel[self.vadr] + 0.5 * STEP_S * d.qacc[self.vadr],
                                            np.where(self.shorted, self.damping, 0.0), STEP_S)
                p.delta = self.boxes.delta.ctypes.data
            self.heat += float(self.loss @ (self.motor * self.motor)) * STEP_S
            pushing = d.time < self.push_until
            if pushing or self.pushed:
                d.xfrc_applied[:, 0:3] = 0.0
                d.xfrc_applied[self.torso, 0:3] = self.push_n if pushing else 0.0
            self.pushed = pushing
            if self.hung is not None:
                d.xfrc_applied[self.pelvis] = self._harness()
            self._mj.mj_step(self.model, d)
            if BOXED:
                self.boxes.moved(d.qpos[self.qadr])
            self._metered(ctypes.byref(p), self.boxes.w.ctypes.data if BOXED else None, STEP_S)
            if self.laced:
                self._unlace()
            if ROOM:
                room.touched(self, STEP_S)
        self.at = d.time
        self.effort, self.work, self.brake = p.effort, p.work, p.brake
        if self.buses is not None:
            self.buses.drain()

    def _stroked(self):
        """The joints a rod drives (`linkage.RODS`) as their angles have them: the ratio
        over their size's to their boards, the rotor seen to MuJoCo and their boards' feed, the
        drag and a short's damping to MuJoCo."""
        m, b = self.model, self.block
        for i, joint, base, free in self.strokes:
            s = drives.ratio(joint, math.degrees(self.data.qpos[self.qadr[i]])) / drives.ratio(joint)
            b.scale[i] = s
            seen = drives.armature(joint) * s * s
            m.dof_armature[self.vadr[i]] = base if BOXED else base + REFLECTED * (seen - base)
            self.boxes.j[i] = seen
            b.rotor[i] = REFLECTED * ROTOR_FF * seen
            m.dof_frictionloss[self.vadr[i]] = BACKDRIVE * drives.backdrive(joint) * s
            m.dof_damping[self.vadr[i]] = free + (self.damping[i] * s * s if self.shorted[i]
                                                  and not BOXED else 0.0)

    def dents(self):
        """Where her mattress gives under her (`floor.dents`)."""
        return floor.dents(self, self._np)

    def props(self):
        """What lies on the floor (`floor.props`), and a lace snagged: ('lace', from, to), shoe
        to shoe."""
        m, d = self.model, self.data
        out = floor.props(self) + (room.props(self) if ROOM else [])
        if self.laced:
            out.append(('lace', tuple(d.site_xpos[m.site('lace_left').id]),
                        tuple(d.site_xpos[m.site('lace_right').id])))
        return out

    def lace(self):
        """Her left shoe's lace snagged on her right shoe: LACE_M of it between them, or as far
        apart as they are now."""
        self.model.tendon_range[self.lace_at] = (
            0.0, max(LACE_M, float(self.data.ten_length[self.lace_at])))
        self.laced = True

    def _unlace(self):
        """The lace pulled off its snag: its tension LACE_HOLD_N."""
        d = self.data
        pull = sum(abs(d.efc_force[i]) for i in range(d.nefc)
                   if d.efc_type[i] == self._mj.mjtConstraint.mjCNSTR_LIMIT_TENDON
                   and d.efc_id[i] == self.lace_at)
        if pull >= LACE_HOLD_N:
            self.model.tendon_range[self.lace_at] = (0.0, 10.0)
            self.laced = False

    def push(self, force, seconds):
        """A shove on the torso, world newtons, for `seconds`."""
        self.push_n = self._np.array(force, float)
        self.push_until = self.data.time + seconds

    def hang(self):
        """Her pelvis hung where it is in a harness (HANG): springs to its place and its turn."""
        self.hung = (self.data.qpos[0:3].copy(), self.data.qpos[3:7].copy())

    def _harness(self):
        """The harness's wrench on her pelvis, world: twice the vector part of turn * conj(q)
        is the small turn from q to turn."""
        d, np = self.data, self._np
        at, turn = self.hung
        q, w = d.qpos[3:7], d.xmat[self.pelvis].reshape(3, 3) @ d.qvel[3:6]
        off = 2.0 * (q[0] * turn[1:] - turn[0] * q[1:] - np.cross(turn[1:], q[1:]))
        return np.r_[HANG[0] * (at - d.qpos[0:3]) - HANG[1] * d.qvel[0:3],
                     HANG[2] * off - HANG[3] * w]

    def pull(self):
        """What a load cell in the harness reads: its force on her pelvis, world, N, and its
        moment about her pelvis's origin, world, N m."""
        d, np = self.data, self._np
        w = self._harness()
        return np.r_[w[:3], w[3:] + np.cross(d.xipos[self.pelvis] - d.xpos[self.pelvis], w[:3])]

    def glitch(self, joint, kind, seconds=0.0, celsius=WARM_C):
        """Drive `joint`'s board glitched: 'soa' its switches SOA_RDS times their on-resistance
        for `seconds`, 'hot' its nodes at `celsius` at once. What it does of it, and says of it,
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
            self.block.warm[i] = celsius
        else:
            raise MachineError('no glitch %r: soa or hot' % kind)

    def arm(self, index):
        """A joint's board's gates on again: the host's gate write, with the next pass."""
        self.shorted[index], self.cut[index], self.testing[index] = False, False, 0.0
        self.model.dof_damping[self.vadr[index]] = self.free[index]
        if index in self.bus_of:
            self.bus_of[index].arm(index)

    def short(self, index):
        """A joint's board's phases shorted through the low sides: the host's gate write, with
        the next pass."""
        self.shorted[index], self.cut[index], self.testing[index] = True, False, 0.0
        self.model.dof_damping[self.vadr[index]] = self.free[index] + (
            0.0 if BOXED else self.damping[index])
        if index in self.bus_of:
            self.bus_of[index].short(index)

    def off(self, index):
        """A joint's board's gates off - no torque, nothing switched: the host's gate write, with
        the next pass."""
        self.shorted[index], self.cut[index], self.testing[index] = False, True, 0.0
        self.model.dof_damping[self.vadr[index]] = self.free[index]
        if index in self.bus_of:
            self.bus_of[index].off(index)

    def test(self, index, amps):
        """A cut board checking itself on a d-axis current, `amps`: no torque, drawn."""
        self.testing[index] = amps

    def drawn(self):
        """What her drives draw now, W: the work they do - braking gives nothing back -, their
        copper's heat and their boards' own, housekeeping each, switching neither shorted nor
        cut but checking (`test`), and that check's copper."""
        p = self.passing
        return self._drawn(ctypes.byref(p), self.motor.ctypes.data if BOXED else p.ctrl,
                           self.boxes.w.ctypes.data if BOXED else None, *self._drawn_of,
                           heat.HOUSEKEEPING_W, heat.SWITCHING_W)

    def reading(self, index):
        """(degrees, deg/s, C, spent, derate, status) of a joint's drive as its board last
        answered the host (`machine.buses`) - with no bus the world's own angle, cool."""
        self.advance()
        return self._reading(index)

    def name(self, node, index):
        """Drive `index`'s channels in the batch as node `node`'s angle."""
        self.keys += ['%s.angle.%s' % (node, key) for key in READING]
        self.named.append(index)

    def drives(self):
        """{'<node>.angle.<key>': float} of every named drive (`reading`), advanced once."""
        self.advance()
        return dict(zip(self.keys, [float(v) for i in self.named for v in self._reading(i)]))

    def _reading(self, index):
        if index in self.bus_of:
            return self.bus_of[index].reading(index)
        d = self.data
        return ((math.degrees(d.qpos[self.qadr[index]]), math.degrees(d.qvel[self.vadr[index]]))
                + QUIET[2:])

    def angle(self, index):
        """(degrees, deg/s) of a joint as its board last answered the host."""
        return self.reading(index)[:2]

    def _of(self, names):
        """The geoms of the segments whose names end in one of `names`."""
        m = self.model
        of = self._geoms = getattr(self, '_geoms', {})
        if names not in of:
            of[names] = [g for g in range(m.ngeom)
                         if m.body(m.geom_bodyid[g]).name.endswith(tuple(names))]
        return of[names]

    def gap(self, one, other, reach=0.3):
        """The least distance, m, between the geoms of segments `one` and those of `other`
        (`_of`; negative: into each other), `reach` at most."""
        return min(self._mj.mj_geomDistance(self.model, self.data, a, b, reach, None)
                   for a in self._of(one) for b in self._of(other))

    def lifted(self, names):
        """The least distance, m, from segments `names`' geoms (`_of`) to the floor's slab."""
        slab = [self.model.geom(n).id for n in ('slab_a', 'slab_b')]
        return min(self._mj.mj_geomDistance(self.model, self.data, a, b, 0.5, None)
                   for a in self._of(names) for b in slab)

    def loose(self):
        """{hinge: degrees} of what hangs loose on her (`LOOSE`): the jeans' legs from her
        shins, her hair from her head."""
        return dict(zip(LOOSE, map(math.degrees, self.data.qpos[self.loose_at])))

    def pose(self):
        """The pelvis and the body: place, turn (quaternion), speeds (world), the centre of mass,
        the head's turn (its IMU, head_q*), each sole's load (N), and since the reset the drives'
        work done and braked and their heat (J), and the torque they held (N m s)."""
        self.advance()
        d, m, np = self.data, self.model, self._np
        omega = (d.xmat[self.pelvis].reshape(3, 3) @ d.qvel[3:6]).tolist()
        loads, force = [0.0, 0.0], np.zeros(6)
        if d.ncon:
            on = self.soles[m.geom_bodyid[d.contact.geom]].any(axis=1)
            for side in (0, 1):
                for i in np.flatnonzero(on[:, side]).tolist():
                    self._mj.mj_contactForce(m, d, i, force)
                    loads[side] += float(force[0])
        if LOAD_S and self.borne_at is not None:
            k = 1.0 - math.exp(-(d.time - self.borne_at) / LOAD_S)
            loads = [w + k * (v - w) for w, v in zip(self.borne, loads)]
        self.borne, self.borne_at = loads, d.time
        com, v = d.subtree_com[self.pelvis].tolist(), d.qvel[0:3].tolist()
        return dict(zip(('x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'), d.qpos[0:7].tolist()),
                    **dict(zip(('head_qw', 'head_qx', 'head_qy', 'head_qz'),
                               d.xquat[self.head].tolist())),
                    vx=v[0], vy=v[1], vz=v[2], wx=omega[0], wy=omega[1], wz=omega[2],
                    com_x=com[0], com_y=com[1], com_z=com[2], left_load=loads[0],
                    right_load=loads[1], t=d.time, work=self.work, brake=self.brake,
                    heat=self.heat, effort=self.effort)
