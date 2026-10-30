"""Her body's one edge: what a robot of her build measures of the world, what her drives are told.

    y = edge.laid(world)                 # (SENSED,), its layout once
    edge.felt(world, y)                  # her pelvis's IMU and her joints' encoders, every pass
    edge.read(world, y)                  # and her soles' loads and her segments' contacts
    edge.drive(world, joints, target=, feed=, kp=, limit=)    # her boards told, the next pass
    edge.noise(world, LINE_BER); edge.burst(world, 'left_leg', 8)   # her buses' line, a test's burst
    edge.held(world, passes)             # BusFault past `passes` setpoints refused in a row
    edge.off(world)                      # her drives off

Her law reads this array alone - an IMU on her pelvis (its turn as a rotation vector, its spin and
its specific force, its own frame), each joint's angle and rate and each motor's angle through its
gearbox, each drive's torque, each sole's load cell, each segment's skin: the force on it of whatever it touches that
is not her, in its own frame -, each insole's centre of pressure, never the world's state; and writes her drives' setpoints, feed,
stiffness and clamps alone (test_structure).
"""
import ctypes
import functools
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body.figure import BALL, HEEL, JOINTS, SEGMENTS

G = np.array([0.0, -9.81, 0.0])
J = len(JOINTS)
#: Her segments with skin, `SEGMENTS`' order.
SKIN = tuple(seg[0] for seg in SEGMENTS)
#: Her soles' segments, each with an insole: (left foot, left toes, right foot, right toes).
INSOLES = ('left_foot', 'left_toes', 'right_foot', 'right_toes')
#: A foot's insole's cells: its heel's half and its ball's, split at the sole's middle.
FEET, MIDDLE_M = ('left_foot', 'right_foot'), 0.5 * (BALL - HEEL)
#: The array: the IMU's turn (w, x, y, z, world from pelvis), spin (rad/s) and specific force
#: (m/s^2), its frame; each joint's angle (rad) and rate (rad/s), each motor's angle seen through
#: its gearbox (rad); each sole's load (N); each segment's touch (N, its frame); each insole's
#: centre of pressure (m, its segment's frame) and its load (N); each foot's heel and ball cell's
#: load (N); where on each segment its touch is, its centre (m, its frame); each joint's drive's
#: torque as its current says it (N m, kt i through its box); her boards' DC link (V).
QUAT, GYRO, ACCEL = slice(0, 4), slice(4, 7), slice(7, 10)
Q, QD, QM = (slice(10 + k * J, 10 + (k + 1) * J) for k in range(3))
LOAD = slice(10 + 3 * J, 12 + 3 * J)
TOUCH = slice(12 + 3 * J, 12 + 3 * J + 3 * len(SKIN))
COP = slice(TOUCH.stop, TOUCH.stop + 3 * len(INSOLES))
PRESS = slice(COP.stop, COP.stop + len(INSOLES))
CELLS = slice(PRESS.stop, PRESS.stop + 2 * len(FEET))
AT = slice(CELLS.stop, CELLS.stop + 3 * len(SKIN))
TAU = slice(AT.stop, AT.stop + J)
LINK = slice(TAU.stop, TAU.stop + 1)
SENSED = LINK.stop
#: Her touch past this, N, a segment's: MuJoCo's soft contact holds a resting hand at a few.
TOUCHED_N = 5.0


def laid(world):
    """Her array for `world`, read: and the world's ids it reads by, kept on the world."""
    m = world.model
    hers = np.array([_rooted(m, b, world.pelvis) for b in range(m.nbody)])
    world.skin = np.full(m.nbody, -1, int)
    for k, name in enumerate(SKIN):
        world.skin[m.body(name).id] = k
    world.skin_bodies = np.array([m.body(name).id for name in SKIN])
    world.hers = hers
    world.insole = np.full(m.nbody, -1, int)
    for k, name in enumerate(INSOLES):
        world.insole[m.body(name).id] = k
    world.foot = np.full(m.nbody, -1, int)
    for k, name in enumerate(FEET):
        world.foot[m.body(name).id] = k
    from tools.cores import wbc as cores
    kept = {'gb': np.ascontiguousarray(m.geom_bodyid, dtype=np.int32),
            'hers': np.ascontiguousarray(hers, dtype=np.bool_),
            'sole_side': np.array([int(s[1]) if s.any() else -1 for s in world.soles], dtype=np.int32),
            'insole': np.ascontiguousarray(world.insole, dtype=np.int32),
            'foot': np.ascontiguousarray(world.foot, dtype=np.int32),
            'skin': np.ascontiguousarray(world.skin, dtype=np.int32),
            'loads': np.zeros(2), 'touch': np.zeros((len(SKIN), 3)), 'pressed': np.zeros(len(INSOLES)),
            'at': np.zeros((len(INSOLES), 3)), 'cells': np.zeros(2 * len(FEET)), 'felt_n': np.zeros(len(SKIN)),
            'felt_at': np.zeros((len(SKIN), 3))}
    c = cores.Contacts(nbody=m.nbody, nskin=len(SKIN), ninsoles=len(INSOLES), nfeet=len(FEET), middle_m=MIDDLE_M)
    for name in ('gb', 'sole_side', 'insole', 'foot', 'skin'):
        setattr(c, name, cores.pointer(kept[name], np.int32))
    c.hers = cores.pointer(kept['hers'], np.bool_)
    for name, a in (('xmat', world.data.xmat), ('xpos', world.data.xpos)):
        setattr(c, name, cores.pointer(a))
    for name in ('loads', 'touch', 'pressed', 'at', 'cells', 'felt_n', 'felt_at'):
        setattr(c, name, cores.pointer(kept[name]))
    #: The contacts' bins and the C that fills them (wbc_pass.c), the contacts' layout laid at
    #: the first contact.
    lib = cores.library()
    world.contacts = (c, kept, lib.wbh_pass_contacts, lib.wbh_pass_sensed)
    return read(world, np.zeros(SENSED))


def _rooted(m, body, pelvis):
    while body > 0:
        if body == pelvis:
            return True
        body = m.body_parentid[body]
    return False


def felt(world, y):
    """`y`'s IMU, encoders and drives' torques as the world stands now (wbc_pass.c, on the world's laid pass)."""
    _read()(ctypes.byref(world.passing), _cores().address(y), QUAT.start, GYRO.start, ACCEL.start, Q.start, QD.start,
            QM.start)
    y[TAU], y[LINK] = world.motor, world.link['v']
    return y


@functools.cache
def _read():
    """The C read, bound once."""
    return _cores().library().wbh_pass_felt


def layout(c, con):
    """The contacts' layout on `c` (wbc_contacts_t), laid at their first: a contact's bytes, its
    fields' offsets from its place."""
    if not c.stride:
        base = con.pos.ctypes.data
        c.stride, c.off_geom, c.off_frame = con.pos.strides[0], con.geom.ctypes.data - base, con.frame.ctypes.data - base
        c.off_dim, c.off_efc = con.dim.ctypes.data - base, con.efc_address.ctypes.data - base
        c.off_friction = con.friction.ctypes.data - base


def read(world, y):
    """`y` whole: the IMU and encoders (`felt`), her soles' loads, her segments' touch and
    where on each it is."""
    felt(world, y)
    d = world.data
    c, kept, contacts, sensed = world.contacts
    ncon = d.ncon
    # the contacts between a body of hers and one not, in C: each's force its rows of the
    # solver's (the cone elliptic, `mjcf.CONE`), a bin a segment in the contacts' order
    if ncon:
        con = d.contact
        layout(c, con)
        contacts(ctypes.byref(c), ncon, con.pos.ctypes.data, d.efc_force.ctypes.data)
    else:
        contacts(ctypes.byref(c), 0, None, None)
    # a shove is a hand on her: what the world applies to her bodies, her skin feels - at
    # their centres of mass, where the world applies it
    if d.xfrc_applied.any():
        touch, felt_n, felt_at = kept['touch'], kept['felt_n'], kept['felt_at']
        sb = world.skin_bodies
        hands = d.xfrc_applied[sb, :3]
        for k, body in enumerate(sb):
            hand = hands[k]
            if hand.any():
                hn = float(np.linalg.norm(hand))
                rm_t = d.xmat[body].reshape(3, 3).T
                touch[k] += rm_t @ hand
                felt_n[k] += hn
                felt_at[k] += hn * rm_t @ (d.xipos[body] - d.xpos[body])
    sensed(ctypes.byref(c), y.ctypes.data, LOAD.start, TOUCH.start, PRESS.start, COP.start, CELLS.start, AT.start)
    return y


def touched(y):
    """(SKIN,) each segment's touch, N."""
    return np.linalg.norm(y[TOUCH].reshape(-1, 3), axis=1)


def tilt(y):
    """Her pelvis's tilt from upright as her IMU has it, deg."""
    q = y[QUAT]
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * float(q[1] ** 2 + q[3] ** 2)))))


def yaw(y):
    """Her pelvis's turn about up as her IMU has it, rad."""
    q = y[QUAT]
    return 2.0 * math.atan2(float(q[2]), float(q[0]))


#: 10 Mbit/s RS485 Modbus RTU bus per extremity (`axis`, `left_arm`, `left_leg`, `right_arm`, `right_leg`).
BAUD = 10_000_000
#: Her buses' bit error rate on a run: a noisy industrial RS485 run's, 3 of her 10 000 frames a
#: second refused in 10 s; 0 laid, a test's faults its bursts (`burst`).
LINE_BER = 1e-7
LIMBS = (
    ('axis', ('spine', 'spine_roll', 'waist', 'neck', 'head')),
    ('left_arm', ('left_shoulder', 'left_elbow', 'left_wrist', 'left_gripper')),
    ('left_leg', ('left_hip_yaw', 'left_hip_roll', 'left_hip', 'left_knee', 'left_ankle', 'left_ankle_roll', 'left_foot')),
    ('right_arm', ('right_shoulder', 'right_elbow', 'right_wrist', 'right_gripper')),
    ('right_leg', ('right_hip_yaw', 'right_hip_roll', 'right_hip', 'right_knee', 'right_ankle', 'right_ankle_roll', 'right_foot')),
)
_SCALE = None


class LimbBus:

    """One extremity's 10 Mbit/s RS485 Modbus RTU SIL bus: commands cross as CRC-16/MODBUS frames
    into its SIL coaxial_63100 boards (`modbus_rtu.c`, `modbus_slave.c`) and can only address the
    boards wired to this extremity."""

    def __init__(self, link, name, joints, lib, baud=BAUD):
        self.link, self.name, self.joints, self.baud = int(link), name, tuple(joints), int(baud)
        self.indices = np.array([JOINTS.index(j) for j in self.joints], dtype=np.int32)
        self._lib = lib
        self._counts = np.zeros(5, dtype=np.uint64)
        self._wire = ctypes.c_double(0.0)
        self._buf = (ctypes.c_uint8 * 64)()
        self._len = ctypes.c_int(0)

    def _pull(self):
        self._lib.wbh_sil_stats(self.link, self._counts.ctypes.data, ctypes.byref(self._wire),
                                ctypes.byref(self._buf), ctypes.byref(self._len))

    @property
    def tx_bytes(self):
        self._pull()
        return int(self._counts[0])

    @property
    def frames(self):
        self._pull()
        return int(self._counts[1])

    @property
    def stale(self):
        """Setpoint frames refused in a row, now."""
        self._pull()
        return int(self._counts[4])

    @property
    def comm_errors(self):
        """Frames its boards refused, each once."""
        self._pull()
        return int(self._counts[2])

    @property
    def server_messages(self):
        self._pull()
        return int(self._counts[3])

    @property
    def wire_s(self):
        self._pull()
        return float(self._wire.value)

    @property
    def last_frame(self):
        self._pull()
        return bytes(self._buf[:self._len.value])

    def transceive(self, frame, world):
        """Verify CRC-16/MODBUS on `frame` via the SIL boards' `mb_rtu_t` + `mb_slave_t`, account
        10 Mbit/s 8N1 wire time, and apply the register write solely to this extremity's boards."""
        raw = bytes(frame)
        rc = int(self._lib.wbh_sil_transceive(ctypes.byref(world.passing), self.link, raw, len(raw)))
        if rc == -1:
            raise ValueError('invalid Modbus RTU broadcast on %s' % self.name)
        if rc == -2:
            raise ValueError('Modbus RTU unit span mismatch on %s' % self.name)
        if rc < 0:
            raise ValueError('Modbus RTU CRC-16 fault on %s' % self.name)


@functools.cache
def _cores():
    """`tools.cores.wbc`, imported on first use."""
    from tools.cores import wbc
    return wbc


def _buses_of(world):
    buses = getattr(world, 'limb_buses', None)
    if buses is None:
        lib = _cores().library()
        counts = np.array([len(jnts) for _, jnts in LIMBS], dtype=np.int32)
        flat = np.array([JOINTS.index(j) for _, jnts in LIMBS for j in jnts], dtype=np.int32)
        lib.wbh_sil_init(ctypes.byref(world.passing), len(LIMBS), counts.ctypes.data, flat.ctypes.data)
        buses = world.limb_buses = tuple(LimbBus(k, name, jnts, lib) for k, (name, jnts) in enumerate(LIMBS))
        world._sil_drive = lib.wbh_sil_drive
        world._idx_cache = {}
    return buses


def _c_arr(arr):
    """(`arr` laid contiguous in doubles, its address), (None, None) for none."""
    if arr is None:
        return None, None
    a = arr if (isinstance(arr, np.ndarray) and arr.dtype == np.float64 and arr.flags.c_contiguous) else np.ascontiguousarray(arr, dtype=np.float64)
    return a, _cores().address(a)


def drive(world, joints, target=None, feed=None, kp=None, limit=None):
    """Her SIL boards told for the next pass over each extremity's 10 Mbit/s RS485 Modbus RTU bus
    (`LIMBS`), each at `joints` (indices of JOINTS): their setpoints (rad), torques fed forward
    (N m), stiffness (N m/rad) and clamps (N m)."""
    _buses_of(world)
    cache = world._idx_cache
    key = id(joints) if isinstance(joints, np.ndarray) else tuple(joints)
    idx = cache.get(key)
    if idx is None:
        idx = cache[key] = np.ascontiguousarray(joints, dtype=np.int32)
    t_a, t_p = _c_arr(target)
    f_a, f_p = _c_arr(feed)
    k_a, k_p = _c_arr(kp)
    l_a, l_p = _c_arr(limit)
    world._sil_drive(ctypes.byref(world.passing), len(idx), _cores().address(idx), t_p, f_p, k_p, l_p)


def noise(world, ber, seed=1):
    """Her buses' line flipping a bit at `ber` (LINE_BER on a run), drawn from `seed`."""
    _buses_of(world)
    _cores().library().wbh_sil_noise(float(ber), int(seed))


def burst(world, bus, frames):
    """The next `frames` frames on `bus` (a LIMBS name) corrupted: a test's fault."""
    _buses_of(world)
    _cores().library().wbh_sil_burst([name for name, _j in LIMBS].index(bus), int(frames))


def held(world, passes):
    """Her buses still steering her: BusFault where one's setpoints were refused more than
    `passes` in a row - a fault no longer sporadic."""
    from machine.errors import BusFault
    at = ctypes.c_int(-1)
    run = int(_cores().library().wbh_sil_stalest(ctypes.byref(at)))
    if run > passes:
        raise BusFault('%s: %d setpoints refused in a row, past her horizon: drives off' % (LIMBS[at.value][0], run))


def off(world):
    """Her drives off: every board's clamp at zero."""
    _buses_of(world)
    _cores().library().wbh_sil_off(ctypes.byref(world.passing))
