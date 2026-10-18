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
    for i in range(d.ncon):
        c = d.contact[i]
        b0, b1 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        mine, theirs = (b0, b1) if world.hers[b0] else (b1, b0)
        if not world.hers[mine] or world.hers[theirs]:
            continue
        world._mj.mj_contactForce(m, d, i, force)
        on = world.soles[mine]
        if on.any():
            loads[int(on[1])] += float(force[0])
        k = world.insole[mine]
        if k >= 0:
            pressed[k] += float(force[0])
            at[k] += float(force[0]) * d.xmat[mine].reshape(3, 3).T @ (c.pos - d.xpos[mine])
            if world.foot[mine] >= 0:
                along = float(d.xmat[mine].reshape(3, 3)[:, 2] @ (c.pos - d.xpos[mine]))
                cells[2 * world.foot[mine] + int(along >= MIDDLE_M)] += float(force[0])
        k = world.skin[mine]
        if k >= 0:
            push = c.frame.reshape(3, 3).T @ force[:3] * (1.0 if mine == b1 else -1.0)
            touch[k] += d.xmat[mine].reshape(3, 3).T @ push
            felt_n[k] += abs(float(force[0]))
            felt_at[k] += abs(float(force[0])) * d.xmat[mine].reshape(3, 3).T @ (c.pos - d.xpos[mine])
    # a shove is a hand on her: what the world applies to her bodies, her skin feels - at
    # their centres of mass, where the world applies it
    for k, body in enumerate(world.skin_bodies):
        hand = d.xfrc_applied[body, :3]
        touch[k] += d.xmat[body].reshape(3, 3).T @ hand
        felt_n[k] += float(np.linalg.norm(hand))
        felt_at[k] += float(np.linalg.norm(hand)) * d.xmat[body].reshape(3, 3).T @ (d.xipos[body] - d.xpos[body])
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
