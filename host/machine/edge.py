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
    world._gb = tuple(int(x) for x in m.geom_bodyid)
    world._hers = tuple(bool(x) for x in hers)
    world._insole = tuple(int(x) for x in world.insole)
    world._foot = tuple(int(x) for x in world.foot)
    world._skin = tuple(int(x) for x in world.skin)
    world._sole_side = tuple(int(s[1]) if s.any() else -1 for s in world.soles)
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
    m, d = world.model, world.data
    loads, touch, force = [0.0, 0.0], np.zeros((len(SKIN), 3)), np.zeros(6)
    pressed, at, cells = np.zeros(len(INSOLES)), np.zeros((len(INSOLES), 3)), np.zeros(2 * len(FEET))
    felt_n, felt_at = np.zeros(len(SKIN)), np.zeros((len(SKIN), 3))
    ncon = d.ncon
    if ncon:
        geoms = d.contact.geom[:ncon].tolist()
        c_pos = d.contact.pos[:ncon]
        c_frame = d.contact.frame[:ncon].reshape(ncon, 3, 3)
        gb, hers = world._gb, world._hers
        insole, foot, skin, sole_side = world._insole, world._foot, world._skin, world._sole_side
        xmat, xpos = d.xmat.reshape(-1, 3, 3), d.xpos
        mj_cf = world._mj.mj_contactForce
        for i, (g0, g1) in enumerate(geoms):
            b0, b1 = gb[g0], gb[g1]
            mine, theirs = (b0, b1) if hers[b0] else (b1, b0)
            if not hers[mine] or hers[theirs]:
                continue
            mj_cf(m, d, i, force)
            f0 = float(force[0])
            ss = sole_side[mine]
            if ss >= 0:
                loads[ss] += f0
            rm_t = xmat[mine].T
            rel = c_pos[i] - xpos[mine]
            k = insole[mine]
            if k >= 0:
                pressed[k] += f0
                at[k] += f0 * rm_t @ rel
                f_id = foot[mine]
                if f_id >= 0:
                    cells[2 * f_id + int(float(rm_t[2] @ rel) >= MIDDLE_M)] += f0
            k = skin[mine]
            if k >= 0:
                push = c_frame[i].T @ force[:3] * (1.0 if mine == b1 else -1.0)
                touch[k] += rm_t @ push
                af0 = abs(f0)
                felt_n[k] += af0
                felt_at[k] += af0 * rm_t @ rel
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
