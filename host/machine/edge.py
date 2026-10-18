"""Her body's one edge: what a robot of her build measures of the world, what her drives are told.

    y = edge.laid(world)                 # (SENSED,), its layout once
    edge.felt(world, y)                  # her pelvis's IMU and her joints' encoders, every pass
    edge.read(world, y)                  # and her soles' loads and her segments' contacts
    edge.drive(world, joints, target=, feed=, kp=, limit=)    # her boards told, the next pass

Her law reads this array alone - an IMU on her pelvis (its turn as a rotation vector, its spin and
its specific force, its own frame), each joint's angle and rate and each motor's angle through its
gearbox, each sole's load cell, each segment's skin: the force on it of whatever it touches that
is not her, in its own frame -, each insole's centre of pressure, never the world's state; and writes her drives' setpoints, feed,
stiffness and clamps alone (test_structure).
"""
import ctypes
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.figure import BALL, HEEL, JOINTS, SEGMENTS

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
#: load (N); where on each segment its touch is, its centre (m, its frame).
QUAT, GYRO, ACCEL = slice(0, 4), slice(4, 7), slice(7, 10)
Q, QD, QM = (slice(10 + k * J, 10 + (k + 1) * J) for k in range(3))
LOAD = slice(10 + 3 * J, 12 + 3 * J)
TOUCH = slice(12 + 3 * J, 12 + 3 * J + 3 * len(SKIN))
COP = slice(TOUCH.stop, TOUCH.stop + 3 * len(INSOLES))
PRESS = slice(COP.stop, COP.stop + len(INSOLES))
CELLS = slice(PRESS.stop, PRESS.stop + 2 * len(FEET))
AT = slice(CELLS.stop, CELLS.stop + 3 * len(SKIN))
SENSED = AT.stop
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
    """`y`'s IMU and encoders as the world stands now (wbc_pass.c, on the world's laid pass)."""
    _read()(ctypes.byref(world.passing), y.ctypes.data, QUAT.start, GYRO.start, ACCEL.start, Q.start, QD.start,
            QM.start)
    return y


def _read():
    """The C read, bound once."""
    global _FELT
    if _FELT is None:
        from tools.cores import wbc as cores
        _FELT = cores.library().wbh_pass_felt
    return _FELT


_FELT = None


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
        if not c.stride:
            base = con.pos.ctypes.data
            c.stride, c.off_geom, c.off_frame = con.pos.strides[0], con.geom.ctypes.data - base, con.frame.ctypes.data - base
            c.off_dim, c.off_efc = con.dim.ctypes.data - base, con.efc_address.ctypes.data - base
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


def drive(world, joints, target=None, feed=None, kp=None, limit=None):
    """Her boards told for the next pass, each at `joints` (indices of JOINTS): their setpoints
    (rad), torques fed forward (N m), stiffness (N m/rad) and clamps (N m)."""
    for have, into in ((target, world.target), (feed, world.feed), (limit, world.limit)):
        if have is not None:
            into[joints] = have
    if kp is not None:
        world.gains[joints, 0] = kp
