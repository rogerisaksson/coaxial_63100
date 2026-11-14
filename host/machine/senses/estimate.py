"""Where its pelvis is and how fast, of what it measures alone (`machine.senses.edge`).

A bearing insole's centre of pressure is a point of the floor that does not move.

    est = estimate.laid(body, where)          # where it was laid: the one thing it is told
    qpos, qvel = estimate.step(est, body, y, dt)   # its law's state of its array, a pass on

Its turn, its spin and its joints are its IMU's and its encoders'. Its pelvis's speed is carried
on by its IMU's specific force and gravity, and drawn to what each bearing insole's centre of
pressure says of it - the insoles weighed by their loads. From the soles' lowest corners alone it
erred 0.165 m/s at the 99th percentile of a walk, 0.41 at worst, where a sole rolled. Its height and place are where it was laid,
carried on: drift is its own, as on a robot. What that draw has added up to, forgotten over
SLIP_S, is how fast they slide (`est['slip']`, world): walking a level floor 0.071 m/s at most,
turning 0.127; on ice 1.36 at the 90th percentile. Dropped at a pass no sole bore, a swing's
toes' scuff drew it and its return read a slide of the sole that held.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.senses import edge
from machine.body.figure import JOINTS
from machine.moves.leap import TOUCH_N

#: Its speed as its IMU carries it on, drawn to what its bearing soles say over HOLD_S; its soles'
#: slide forgotten over SLIP_S.
HOLD_S, SLIP_S = 0.03, 0.3


def laid(body, where):
    """The robot's estimate laid at `where`, its pelvis (m, world), still."""
    m = body.m
    qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in JOINTS])
    vadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in JOINTS])
    est = {'qpos': np.zeros(m.nq), 'qvel': np.zeros(m.nv), 'qadr': qadr, 'vadr': vadr,
           'pos': np.array(where, float), 'vel': np.zeros(3), 'jp': np.zeros((3, m.nv)),
           'speeds': np.zeros((len(edge.INSOLES), 3)), 'insoles': [m.body(n).id for n in edge.INSOLES],
           'slip': np.zeros(3)}
    est['qpos'][3] = 1.0
    return est


def _rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def step(est, body, y, dt):
    """(qpos, qvel) of its model as `y` has it, `est` carried on `dt` s."""
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
    rot = _rot(y[edge.QUAT])
    up_f = (rot @ y[edge.TOUCH][0:3])[1]
    loads = y[edge.PRESS] * (y[edge.PRESS] >= TOUCH_N / 2.0)
    bears = loads.sum() + up_f * (up_f >= TOUCH_N / 2.0)
    falls = est['vel'] + (rot @ y[edge.ACCEL] + edge.G) * dt
    went = -(loads @ est['speeds']) / np.maximum(bears, 1e-9)
    vel = falls + (bears > 0.0) * min(1.0, dt / HOLD_S) * (went - falls)
    est['slip'] = (1.0 - dt / SLIP_S) * (est['slip'] + falls - vel)
    est['pos'] += 0.5 * (est['vel'] + vel) * dt
    est['vel'] = vel
    qpos[0:3], qvel[0:3] = est['pos'], vel
    return qpos, qvel
