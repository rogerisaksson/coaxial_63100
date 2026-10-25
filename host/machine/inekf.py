"""Her pelvis by a right-invariant EKF on SE_2+2(3): her IMU carries it, her bearing soles correct it.

    f = inekf.laid(where, turn)                 # still at `where` (m, world), turned `turn`
    inekf.felt(f, y, dt)                        # each drive pass: her IMU into the pass's means
    inekf.stepped(f, body, y, shares, dt, inekf.points, inekf.step)   # a law pass: her soles' points off her posed body
    inekf.step(f, y, soles, anchors, shares, dt)   # the filter on given points
    f['R'], f['v'], f['p']                      # her pelvis's turn, velocity and place, world

The kernel's `propagate-invariant-ekf-se3` (Hartley, Ghaffari, Eustice, Grizzle 2020): the state
her pelvis's turn R, velocity v and place p and her soles' contact points d, X in SE_2+2(3); her
gyro and accelerometer over the pass carry it on, their noise the text's (gyro, accel per root
hertz). A sole's point is the one under its centre of pressure: that point last pass, in her pelvis
now by her encoders' kinematics, is R'(d - p) - a rolling sole's point that bears does not move -,
its innovation R h - (d - p), its noise her encoders' grain through her leg over the sole's share
of her weight (the text's `contact-weight-source`: what bears her); then d is laid at the point
under its centre of pressure now. A sole bearing under the comparator's release (`observer.RELEASE`)
or its innovation past the text's Mahalanobis gate (a slide) is let go and laid anew; her IMU the
pass's mean (`felt`). The update X+ = exp(dx) X, the
covariance in Joseph's form.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import edge, kernel, observer

_EKF = kernel.node('propagate-invariant-ekf-se3', 'invariant-ekf')
#: The IMU's noise densities (rad/s, m/s^2 a root hertz).
GYRO, ACCEL = _EKF['inertial-noise']['gyro-radps-sqrt-hz'], _EKF['inertial-noise']['accel-mps2-sqrt-hz']
#: Her encoders' grain (an A1335's 12 bits through her drives' 30:1, rad) through a leg's reach (m):
#: a bearing point's own walk a pass - walking it as the slip gate's 0.08 m/s, the filter took her
#: IMU over her soles and drifted 1.02 m in 13.5.
GRAIN, REACH = 2.0 * math.pi / 4096.0 / 30.0, 0.9
N = 15


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


def laid(where, turn=(1.0, 0.0, 0.0, 0.0)):
    """Her pelvis still at `where`, turned `turn` (w, x, y, z); her soles' points unknown."""
    w, x, y, z = turn
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return {'R': R, 'v': np.zeros(3), 'p': np.array(where, float), 'd': np.zeros((2, 3)),
            'P': np.eye(N) * 1e-6, 'on': np.zeros(2), 'gyro': np.zeros(3), 'accel': np.zeros(3), 'span': np.zeros(())}


def felt(f, y, dt):
    """Her IMU over a drive pass `dt` into the law pass's means (`edge.GYRO`, `edge.ACCEL`)."""
    f['gyro'] += dt * y[edge.GYRO]
    f['accel'] += dt * y[edge.ACCEL]
    f['span'] += dt


def points(body, y, local, weights):
    """(2, 3) each sole's point in her pelvis on her law's posed `body`: her foot's and toes' insoles'
    points `local` (`edge.COP`, their frames) by their `weights` - a sole rolling onto its toes bears
    there (the foot's alone, her pelvis sank 0.39 m in 13.5 of walking)."""
    d, ids = body.d, body.insoles
    Rp, pp = d.xmat[body.pelvis].reshape(3, 3), d.xpos[body.pelvis]
    out = np.zeros((2, 3))
    for k in range(2):
        pts = [d.xpos[ids[2 * k + j]] + d.xmat[ids[2 * k + j]].reshape(3, 3) @ local[2 * k + j] for j in range(2)]
        w = weights[2 * k:2 * k + 2]
        out[k] = Rp.T @ ((w[0] * pts[0] + w[1] * pts[1]) / np.maximum(w.sum(), 1e-9) - pp)
    return out


def stepped(f, body, y, shares, dt, points, step):
    """A law pass of her edge's `y` on her law's `body` posed at her estimate: last pass's soles'
    points now, this pass's laid; the filter `dt` on, each sole bearing `shares` of her weight -
    by `points` and `step`, this module's or their C (`motion.laid`)."""
    local, weights = y[edge.COP].reshape(-1, 3).copy(), np.maximum(y[edge.PRESS], 0.0) + 1e-9
    anchors = points(body, y, local, weights)
    soles = points(body, y, *f['last']) if 'last' in f else anchors
    f['last'] = (local, weights)
    return step(f, y, soles, anchors, shares, dt)


def step(f, y, soles, anchors, shares, dt):
    """`f` a law pass `dt` on: her IMU's mean over it in `y` (`edge.GYRO`, `edge.ACCEL`); each sole's
    point that bore her last pass, in her pelvis now (`soles` (2, 3)), the point under its centre of
    pressure now (`anchors`), its share of her weight (`shares` (2,))."""
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
    for k, block in enumerate((None, f['v'], f['p'], d[0], d[1])):
        ad[3 * k:3 * k + 3, 3 * k:3 * k + 3] = f['R']
        ad[3 * k:3 * k + 3, 0:3] = 0.0 if block is None else _skew(block) @ f['R']
    q = np.diag([GYRO ** 2] * 3 + [ACCEL ** 2] * 3 + [0.0] * 3 + [(GRAIN * REACH) ** 2 / dt] * 6)
    P = phi @ P @ phi.T + ad @ q @ ad.T * dt
    r = np.concatenate([f['R'] @ soles[k] - (d[k] - f['p']) for k in range(2)])
    H = np.zeros((6, N))
    H[0:3, 6:9] = H[3:6, 6:9] = -np.eye(3)
    H[0:3, 9:12], H[3:6, 12:15] = np.eye(3), np.eye(3)
    fk = (GRAIN * REACH) ** 2 / np.maximum(np.asarray(shares, float), 1e-12)
    S0 = H @ P @ H.T + np.diag(np.repeat(fk, 3))
    slid = np.array([r[3 * k:3 * k + 3] @ np.linalg.solve(S0[3 * k:3 * k + 3, 3 * k:3 * k + 3], r[3 * k:3 * k + 3]) > GATE
                     for k in range(2)])
    bear = (np.asarray(shares) >= observer.RELEASE) & (f['on'] > 0.5) & ~slid
    noise = np.repeat(fk / np.maximum(bear, 1e-12), 3)
    S = H @ P @ H.T + np.diag(noise)
    K = P @ H.T @ np.linalg.inv(S)
    dx = K @ r
    E, J = _exp(dx[0:3])
    f['R'], f['v'], f['p'] = E @ f['R'], E @ f['v'] + J @ dx[3:6], E @ f['p'] + J @ dx[6:9]
    IKH = np.eye(N) - K @ H
    P = IKH @ P @ IKH.T + K @ np.diag(noise) @ K.T
    for k in range(2):
        idx = slice(9 + 3 * k, 12 + 3 * k)
        anew = 1.0 - bear[k]
        d[k] = f['p'] + f['R'] @ anchors[k]
        P[idx, :] *= 1.0 - anew
        P[:, idx] *= 1.0 - anew
        P[idx, idx] += anew * (P[6:9, 6:9] + np.eye(3) * (GRAIN * REACH) ** 2)
    f['P'], f['on'] = P, (np.asarray(shares) >= observer.RELEASE).astype(float)
    return f
