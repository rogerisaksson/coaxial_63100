"""Where her pelvis is and how fast, of what she measures alone (`machine.edge`).

A bearing insole's centre of pressure is a point of the floor that does not move.

    est = estimate.laid(body, where)          # where she was laid: the one thing she is told
    qpos, qvel = estimate.step(est, body, y, dt)   # her law's state of her array, a pass on

Her turn, her spin and her joints are her IMU's and her encoders'. Her pelvis's speed is carried
on by her IMU's specific force and gravity, and drawn to what each bearing insole's centre of
pressure says of it - the insoles weighed by their loads. From the soles' lowest corners alone it
erred 0.165 m/s at the 99th percentile of a walk, 0.41 at worst, where a sole rolled. Her height and place are where she was laid,
carried on: drift is her own, as on a robot.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import edge
from machine.figure import JOINTS
from machine.flight import TOUCH_N

#: Her speed as her IMU carries it on, drawn to what her bearing soles say over HOLD_S.
HOLD_S = 0.03


def laid(body, where):
    """Her estimate laid at `where`, her pelvis (m, world), still."""
    m = body.m
    qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in JOINTS])
    vadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in JOINTS])
    est = {'qpos': np.zeros(m.nq), 'qvel': np.zeros(m.nv), 'qadr': qadr, 'vadr': vadr,
           'pos': np.array(where, float), 'vel': np.zeros(3), 'jp': np.zeros((3, m.nv)),
           'speeds': np.zeros((len(edge.INSOLES), 3)), 'insoles': [m.body(n).id for n in edge.INSOLES]}
    est['qpos'][3] = 1.0
    return est


def _rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def step(est, body, y, dt):
    """(qpos, qvel) of her model as `y` has her, `est` carried on `dt` s."""
    m, d, mj = body.m, body.d, body.mj
    qpos, qvel, jp = est['qpos'], est['qvel'], est['jp']
    qpos[0:3], qpos[3:7], qpos[est['qadr']] = 0.0, y[edge.QUAT], y[edge.Q]
    qvel[0:3], qvel[3:6], qvel[est['vadr']] = 0.0, y[edge.GYRO], y[edge.QD]
    d.qpos[:], d.qvel[:] = qpos, qvel
    mj.mj_kinematics(m, d)
    mj.mj_comPos(m, d)
    for k, seg in enumerate(est['insoles']):
        at = d.xpos[seg] + d.xmat[seg].reshape(3, 3) @ y[edge.COP][3 * k:3 * k + 3]
        mj.mj_jac(m, d, jp, None, at, seg)
        est['speeds'][k] = jp @ qvel
    loads = y[edge.PRESS] * (y[edge.PRESS] >= TOUCH_N / 2.0)
    bears = loads.sum()
    falls = est['vel'] + (_rot(y[edge.QUAT]) @ y[edge.ACCEL] + edge.G) * dt
    went = -(loads @ est['speeds']) / max(bears, 1e-9)
    vel = falls + float(bears > 0.0) * min(1.0, dt / HOLD_S) * (went - falls)
    est['pos'] += 0.5 * (est['vel'] + vel) * dt
    est['vel'] = vel
    qpos[0:3], qvel[0:3] = est['pos'], vel
    return qpos, qvel
