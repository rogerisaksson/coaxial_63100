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
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import physics
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
#: A contact's force rows read: its normal and its two tangents.
ROWS = np.arange(3)


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
    world._gb = np.asarray(m.geom_bodyid)
    world._sole_side = np.array([int(s[1]) if s.any() else -1 for s in world.soles])
    return read(world, np.zeros(SENSED))


def _rooted(m, body, pelvis):
    while body > 0:
        if body == pelvis:
            return True
        body = m.body_parentid[body]
    return False


def felt(world, y):
    """`y`'s IMU and encoders as the world stands now."""
    d = world.data
    y[QUAT] = d.qpos[3:7]
    y[GYRO] = d.qvel[3:6]
    rot = d.xmat[world.pelvis].reshape(3, 3)
    y[ACCEL] = rot.T @ (d.qacc[0:3] - G)
    y[Q] = d.qpos[world.qadr]
    y[QD] = d.qvel[world.vadr]
    y[QM] = y[Q] + physics.WOUND * world.boxes.delta + world.boxes.zero
    return y


def read(world, y):
    """`y` whole: the IMU and encoders (`felt`), her soles' loads, her segments' touch and
    where on each it is."""
    felt(world, y)
    d = world.data
    loads, touch = np.zeros(2), np.zeros((len(SKIN), 3))
    pressed, at, cells = np.zeros(len(INSOLES)), np.zeros((len(INSOLES), 3)), np.zeros(2 * len(FEET))
    felt_n, felt_at = np.zeros(len(SKIN)), np.zeros((len(SKIN), 3))
    ncon = d.ncon
    if ncon:
        # the contacts between a body of hers and one not, as arrays: each's force is its
        # rows of the solver's (the cone elliptic, `mjcf.CONE`), the frame's and the body's
        # products summed as the loop did, a bin a segment in the contacts' order
        geom = d.contact.geom[:ncon]
        b0, b1 = world._gb[geom[:, 0]], world._gb[geom[:, 1]]
        h0 = world.hers[b0]
        sel = np.flatnonzero(h0 != world.hers[b1])
    if ncon and len(sel):
        mine = np.where(h0[sel], b0[sel], b1[sel])
        adr, dim = d.contact.efc_address[:ncon][sel], d.contact.dim[:ncon][sel]
        have = (adr >= 0)[:, None] & (dim[:, None] > ROWS)
        force = np.where(have, d.efc_force[np.where(have, adr[:, None] + ROWS, 0)], 0.0)
        f0, af0 = force[:, 0], np.abs(force[:, 0])
        rm = d.xmat.reshape(-1, 3, 3)[mine]
        rel = d.contact.pos[:ncon][sel] - d.xpos[mine]
        frame = d.contact.frame[:ncon][sel].reshape(-1, 3, 3)
        push = (frame[:, 0] * force[:, 0:1] + frame[:, 1] * force[:, 1:2] + frame[:, 2] * force[:, 2:3]
                ) * np.where(h0[sel], -1.0, 1.0)[:, None]
        rf, ra = f0[:, None, None] * rm, af0[:, None, None] * rm
        where = rf[:, 0] * rel[:, 0:1] + rf[:, 1] * rel[:, 1:2] + rf[:, 2] * rel[:, 2:3]
        felt_where = ra[:, 0] * rel[:, 0:1] + ra[:, 1] * rel[:, 1:2] + ra[:, 2] * rel[:, 2:3]
        pushed = rm[:, 0] * push[:, 0:1] + rm[:, 1] * push[:, 1:2] + rm[:, 2] * push[:, 2:3]
        along = rm[:, 0, 2] * rel[:, 0] + rm[:, 1, 2] * rel[:, 1] + rm[:, 2, 2] * rel[:, 2]
        ss = world._sole_side[mine]
        on = ss >= 0
        loads = np.bincount(ss[on], f0[on], minlength=2)
        k = world.insole[mine]
        ins = k >= 0
        pressed = np.bincount(k[ins], f0[ins], minlength=len(INSOLES))
        rows = (3 * k[ins])[:, None] + ROWS
        at = np.bincount(rows.ravel(), where[ins].ravel(), minlength=3 * len(INSOLES)).reshape(-1, 3)
        f_id = world.foot[mine]
        ft = ins & (f_id >= 0)
        cells = np.bincount(2 * f_id[ft] + (along[ft] >= MIDDLE_M), f0[ft], minlength=2 * len(FEET))
        k = world.skin[mine]
        sk = k >= 0
        rows = (3 * k[sk])[:, None] + ROWS
        touch = np.bincount(rows.ravel(), pushed[sk].ravel(), minlength=3 * len(SKIN)).reshape(-1, 3)
        felt_n = np.bincount(k[sk], af0[sk], minlength=len(SKIN))
        felt_at = np.bincount(rows.ravel(), felt_where[sk].ravel(), minlength=3 * len(SKIN)).reshape(-1, 3)
    # a shove is a hand on her: what the world applies to her bodies, her skin feels - at
    # their centres of mass, where the world applies it
    if d.xfrc_applied.any():
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
    y[LOAD] = loads
    y[TOUCH] = touch.ravel()
    y[PRESS] = pressed
    y[COP] = (at / np.maximum(pressed, 1e-9)[:, None]).ravel()
    y[CELLS] = cells
    y[AT] = (felt_at / np.maximum(felt_n, 1e-9)[:, None]).ravel()
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
