"""Her pelvis by a right-invariant EKF on SE_2+3(3): her IMU carries it, what bears her corrects it.

    f = inekf.laid(where, turn)                 # still at `where` (m, world), turned `turn`
    inekf.felt(f, y, dt)                        # each drive pass: her IMU into the pass's means
    inekf.stepped(f, body, y, shares, dt, inekf.points, inekf.step)   # a law pass: her bearing points off her posed body
    inekf.step(f, y, soles, anchors, shares, dt)   # the filter on given points
    f['R'], f['v'], f['p']                      # her pelvis's turn, velocity and place, world

The kernel's `propagate-invariant-ekf-se3` (Hartley, Ghaffari, Eustice, Grizzle 2020): the state
her pelvis's turn R, velocity v and place p and her soles' and core's contact points d, X in SE_2+3(3); her
gyro and accelerometer over the pass carry it on, their noise the text's (gyro, accel per root
hertz). A contact's point is the one under its centre of pressure: that point last pass, in her pelvis
now by her encoders' kinematics, is R'(d - p) - a rolling sole's or core's point that bears does not move -,
its innovation R h - (d - p), its noise her encoders' grain through her leg over the contact's share
of her weight (the text's `contact-weight-source`: what bears her); then d is laid at the point
under its centre of pressure now. A contact bearing under the comparator's release (`observer.RELEASE`)
or its innovation past the text's Mahalanobis gate (a slide) is let go and laid anew; her IMU the
pass's mean (`felt`). The update X+ = exp(dx) X, the
covariance in Joseph's form.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body import drives, figure, mechatronics
from machine.law import se2
from machine.senses import edge
from machine.texts import kernel
from machine.senses import observer

_EKF = kernel.node('propagate-invariant-ekf-se3', 'invariant-ekf')
_INV = mechatronics.library()['inverters'][mechatronics.robot()['stacks']['knee'][2]]
#: The IMU's noise densities (rad/s, m/s^2 a root hertz).
GYRO, ACCEL = _EKF['inertial-noise']['gyro-radps-sqrt-hz'], _EKF['inertial-noise']['accel-mps2-sqrt-hz']
ENCODER_COUNTS = int(_INV['encoder_counts'])
GRAIN, REACH = math.tau / (ENCODER_COUNTS * drives.RATIO), round(figure.standing(), 2)
CONTACTS, N = 3, 18
CORE = np.array([edge.SKIN.index('pelvis'), edge.SKIN.index('torso')])
#: Her world's up, against gravity (`edge.G`): what a look laid onto her floor turns her about (`seen`).
UP = -edge.G / np.linalg.norm(edge.G)
#: How fast a bearing sole's point rolls along it at most, m/s: its length (`figure.SOLE_HALF` twice) once a stride of
#: her walk (`se2.WALK_HZ`) - her drift's walk a pass (`stepped`).
ROLL_M_S = 2.0 * figure.SOLE_HALF * se2.WALK_HZ


def _skew(w):
    return np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])


def _exp(phi):
    """(Exp(phi), its left Jacobian) on SO(3)."""
    a = np.linalg.norm(phi)
    k = _skew(phi)
    small, b = a < 1e-9, np.maximum(a, 1e-9)
    s = np.where(small, 1.0, np.sin(b) / b)
    c = np.where(small, 0.5, (1.0 - np.cos(b)) / (b * b))
    j = np.where(small, 1.0 / 6.0, (b - np.sin(b)) / b ** 3)
    return np.eye(3) + s * k + c * k @ k, np.eye(3) + c * k + j * k @ k


#: A contact's innovation past this Mahalanobis square is a slide, let go (the text's gate).
GATE = _EKF['visual-innovation-law']['mahalanobis-gate-chi2']
#: Her IMU's spin and specific force in her edge's array, side by side (`edge.GYRO`, `edge.ACCEL`): summed
#: over a law pass as one array laid once, each at its own place in it.
IMU = slice(edge.GYRO.start, edge.ACCEL.stop)
IMU_GYRO, IMU_ACCEL = (slice(sl.start - IMU.start, sl.stop - IMU.start) for sl in (edge.GYRO, edge.ACCEL))


def laid(where, turn=(1.0, 0.0, 0.0, 0.0)):
    """Her pelvis still at `where`, turned `turn` (w, x, y, z); her soles' points unknown."""
    w, x, y, z = turn
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    imu = np.zeros(IMU.stop - IMU.start)
    return {'R': R, 'v': np.zeros(3), 'p': np.array(where, float), 'd': np.zeros((CONTACTS, 3)),
            'P': np.eye(N) * 1e-6, 'on': np.zeros(CONTACTS), 'imu': imu, 'gyro': imu[IMU_GYRO], 'accel': imu[IMU_ACCEL],
            'drift': np.zeros((4, 4)),
            'span': np.zeros(())}


def head(f, body):
    """Her head's pose {pos, quat} in the filter's frame: the filter's pelvis (R, p) and her head on her law's
    `body` as her estimate poses it - her eyes' pose (`sight.Keep.look`)."""
    d = body.d
    R, p = np.asarray(f['R']), np.asarray(f['p'])
    Rp, pp = d.xmat[body.pelvis].reshape(3, 3), d.xpos[body.pelvis]
    Rh, ph = d.xmat[body.head].reshape(3, 3), d.xpos[body.head]
    M = R @ Rp.T @ Rh
    w = 0.5 * math.sqrt(max(0.0, 1.0 + M[0, 0] + M[1, 1] + M[2, 2]))
    q = np.array([w, (M[2, 1] - M[1, 2]), (M[0, 2] - M[2, 0]), (M[1, 0] - M[0, 1])])
    q[1:] /= max(4.0 * w, 1e-12)
    return {'pos': p + R @ Rp.T @ (ph - pp), 'quat': q / np.linalg.norm(q)}


def felt(f, y, dt):
    """Her IMU over a drive pass `dt` into the law pass's means: her spin and specific force at once (IMU)."""
    f['imu'] += dt * y[IMU]
    f['span'] += dt


def points(body, y, local, weights):
    """(3, 3) each contact's point in her pelvis on her law's posed `body`: each sole's foot and toes
    insoles and her pelvis and torso skin (`local`, their frames) by their `weights` - a sole rolling
    onto its toes bears there (the foot's alone, her pelvis sank 0.39 m in 13.5 of walking)."""
    d, ids = body.d, list(body.insoles) + [body.pelvis, body.torso]
    Rp, pp = d.xmat[body.pelvis].reshape(3, 3), d.xpos[body.pelvis]
    out = np.zeros((CONTACTS, 3))
    for k in range(CONTACTS):
        pts = [d.xpos[ids[2 * k + j]] + d.xmat[ids[2 * k + j]].reshape(3, 3) @ local[2 * k + j] for j in range(2)]
        w = weights[2 * k:2 * k + 2]
        out[k] = Rp.T @ ((w[0] * pts[0] + w[1] * pts[1]) / np.maximum(w.sum(), 1e-9) - pp)
    return out


def stepped(f, body, y, shares, dt, points, step):
    """A law pass of her edge's `y` on her law's `body` posed at her estimate: last pass's bearing
    points now, this pass's laid; the filter `dt` on, each sole and her core bearing `shares` of her
    weight - by `points` and `step`, this module's or their C (`motion.laid`)."""
    core = np.linalg.norm(y[edge.TOUCH].reshape(-1, 3)[CORE], axis=1)
    local = np.vstack([y[edge.COP].reshape(-1, 3), y[edge.AT].reshape(-1, 3)[CORE]])
    weights = np.concatenate([np.maximum(y[edge.PRESS], 0.0), core]) + 1e-9
    sh = np.asarray(shares, float)
    sh = np.pad(sh, (0, CONTACTS - len(sh))) if len(sh) < CONTACTS else sh.copy()
    sh[2] *= float(core.sum() >= edge.TOUCHED_N)
    anchors = points(body, y, local, weights)
    soles = points(body, y, *f['last']) if 'last' in f else anchors
    f['last'] = (local, weights)
    # her drift since her floor last held her (`seen`): her gyro's on her turn, a sole's point's travel along it as it
    # rolls, bearing this pass and the last, on her place - what her contacts, telling her how she moved, cannot tell her
    # of where she is -, no further a pass than a sole rolls its length a stride (ROLL_M_S): a sole come down or let go,
    # its weight shifted heel to toe in a turn on the spot, jumps its point, no roll (her drift 0.2-1.2 m a look so);
    # summed in floats over her contacts - as numpy arrays it was 14 us of her pass, 4.7 so
    cap = (ROLL_M_S * dt) ** 2
    rolled = sum(min(x * x + y * y + z * z, cap) * (s >= observer.RELEASE) * (o > 0.5) for (x, y, z), s, o in
                 zip((soles - anchors).tolist(), sh.tolist(), f['on'].tolist()))
    drift = f['drift']
    drift[0, 0] += GYRO ** 2 * dt
    drift[1, 1] += rolled
    drift[2, 2] += rolled
    drift[3, 3] += rolled
    return step(f, y, soles, anchors, sh, dt)


def step(f, y, soles, anchors, shares, dt):
    """`f` a law pass `dt` on: her IMU's mean over it in `y` (`edge.GYRO`, `edge.ACCEL`); each contact's
    point that bore her last pass, in her pelvis now (`soles` (3, 3)), the point under its centre of
    pressure now (`anchors`), its share of her weight (`shares` (3,))."""
    R, v, p, d, P = f['R'], f['v'], f['p'], f['d'], f['P']
    span = np.maximum(f['span'], dt)
    w, a, g = f['gyro'] / span, f['accel'] / span, edge.G
    f['gyro'][:], f['accel'][:], f['span'][...] = 0.0, 0.0, 0.0
    turn, _j = _exp(w * dt)
    acc = R @ a + g
    f['R'], f['v'], f['p'] = R @ turn, v + acc * dt, p + v * dt + 0.5 * acc * dt * dt
    phi = np.eye(N)
    phi[3:6, 0:3], phi[6:9, 0:3], phi[6:9, 3:6] = _skew(g) * dt, 0.5 * _skew(g) * dt * dt, np.eye(3) * dt
    ad = np.zeros((N, N))
    for k, block in enumerate((None, f['v'], f['p'], d[0], d[1], d[2])):
        ad[3 * k:3 * k + 3, 0:3] = 0.0 if block is None else _skew(block) @ f['R']
        ad[3 * k:3 * k + 3, 3 * k:3 * k + 3] = f['R']
    q = np.diag([GYRO ** 2] * 3 + [ACCEL ** 2] * 3 + [0.0] * 3 + [(GRAIN * REACH) ** 2 / dt] * (3 * CONTACTS))
    P = phi @ P @ phi.T + ad @ q @ ad.T * dt
    r = np.concatenate([f['R'] @ soles[k] - (d[k] - f['p']) for k in range(CONTACTS)])
    H = np.zeros((3 * CONTACTS, N))
    for k in range(CONTACTS):
        H[3 * k:3 * k + 3, 6:9], H[3 * k:3 * k + 3, 9 + 3 * k:12 + 3 * k] = -np.eye(3), np.eye(3)
    fk = (GRAIN * REACH) ** 2 / np.maximum(np.asarray(shares, float), 1e-12)
    S0 = H @ P @ H.T + np.diag(np.repeat(fk, 3))
    slid = np.array([r[3 * k:3 * k + 3] @ np.linalg.solve(S0[3 * k:3 * k + 3, 3 * k:3 * k + 3], r[3 * k:3 * k + 3]) > GATE
                     for k in range(CONTACTS)])
    bear = (np.asarray(shares) >= observer.RELEASE) & (f['on'] > 0.5) & ~slid
    noise = np.repeat(fk / np.maximum(bear, 1e-12), 3)
    S = H @ P @ H.T + np.diag(noise)
    K = P @ H.T @ np.linalg.inv(S)
    dx = K @ r
    E, J = _exp(dx[0:3])
    f['R'], f['v'], f['p'] = E @ f['R'], E @ f['v'] + J @ dx[3:6], E @ f['p'] + J @ dx[6:9]
    IKH = np.eye(N) - K @ H
    P = IKH @ P @ IKH.T + K @ np.diag(noise) @ K.T
    for k in range(CONTACTS):
        idx = slice(9 + 3 * k, 12 + 3 * k)
        anew = 1.0 - bear[k]
        d[k] = f['p'] + f['R'] @ anchors[k]
        P[idx, :] *= 1.0 - anew
        P[:, idx] *= 1.0 - anew
        P[idx, idx] += anew * (P[6:9, 6:9] + np.eye(3) * (GRAIN * REACH) ** 2)
    f['P'], f['on'] = P, (np.asarray(shares) >= observer.RELEASE).astype(float)
    return f


def seen(f, fix, info, eye):
    """Whether `f` took a look laid onto her floor (`machine.eyes.ground.Mosaic.fixed`), the text's visual innovation:
    `fix` its turn about UP through `eye`, rad, and its shift along x, y, z, m, `info` its information (4, 4) - each
    direction her floor resolves -, weighed against her drift since her floor last held her (`f['drift']`, `stepped`),
    gated as her contacts are (GATE); taken, her whole frame turned about `eye` and shifted, her drift what is left of
    it. Her soles carry her place between looks, her floor holds it: her contacts tell her how she moved, not where she
    is, and her filter's own spread said so to 0.3-2 mm while her head read 0.142 m high in 21 s of a bed errand."""
    drift = f['drift']
    told = np.linalg.inv(np.linalg.inv(drift + np.eye(4) * (GRAIN * REACH) ** 2) + info)
    held = float(fix @ (info - info @ told @ info) @ fix <= GATE)
    step = held * (told @ info @ fix)
    c, s = math.cos(step[0]), math.sin(step[0])
    turn = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    e = np.asarray(eye, float)
    f['R'][...], f['v'][...] = turn @ f['R'], turn @ f['v']
    f['p'][...] = e + turn @ (f['p'] - e) + step[1:]
    f['d'][...] = e + (f['d'] - e) @ turn.T + step[1:]
    f['drift'][...] = held * 0.5 * (told + told.T) + (1.0 - held) * drift
    return held
