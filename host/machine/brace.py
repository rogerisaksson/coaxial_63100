"""Her brace on the solve: the tuck she contracts into on a topple, her cervical joints held.

    Ms = brace.scaled(M, acv, w_up)                      # the solve's M, four dofs' rows scaled off
    jeff, leg_kp = brace.braced(M, Ms, q, v, h, ...)     # the solve's answer blended with the tuck's

The python of `wbc_brace.c` (`tools.cores.brace`), held bit for bit (test_ported); her loop runs
the C (`machine.motion.laid`).
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import se2, wbc

#: The tuck's posture, rad, and its lean along (AX) and across (LAT) her surety's direction.
TUCK = {'spine': 0.45, 'neck': 0.35, 'left_hip_roll': 0.12, 'right_hip_roll': -0.12,
        'left_hip': -1.05, 'right_hip': -1.05, 'left_knee': 1.45, 'right_knee': 1.45,
        'left_ankle': -0.45, 'right_ankle': -0.45, 'left_shoulder': 0.80, 'right_shoulder': 0.80,
        'left_elbow': 0.65, 'right_elbow': 0.65}
AX = {'spine': -0.25, 'neck': -0.20, 'left_shoulder': 0.65, 'right_shoulder': 0.65,
      'left_elbow': -0.40, 'right_elbow': -0.40}
LAT = {'left_shoulder_roll': 0.45, 'right_shoulder_roll': 0.45, 'left_hip_roll': 0.15, 'right_hip_roll': 0.15}


def scaled(M, acv, w_up):
    """The solve's M: `M` copied, the four dofs' (`acv`) rows and columns against the base scaled by
    1 - `w_up`."""
    Ms = M.copy()
    Ms[:6, acv] *= 1.0 - w_up[None, :]
    Ms[acv, :6] *= 1.0 - w_up[:, None]
    return Ms


def braced(M, Ms, q, v, h, posture, qacc, tau, over, sw, bw, d_ax, d_lat, w_hop, w_up, stance, brace, act, acv, cv,
           clamp, free_kp, legs):
    """(jeff, leg_kp): the solve's `qacc`, `tau` and `over` blended in place with the tuck's torques
    by her surety's `sw` and `bw` - the tuck's posture `brace` (3, driven) by its direction `d_ax`,
    `d_lat`, through her inertia `M` (`act` the driven dofs) -, the cervical and shoulder joints
    `cv` (dofs `acv`) held to their `posture` as far as `w_up` through the solve's `Ms`; her
    drives' inertia and her `legs`' stiffness off `free_kp` by their `stance`."""
    q_brace = np.array([1.0, d_ax, d_lat]) @ brace
    qdd_br = np.clip(3.5 * wbc.POSTURE_KD[0] * (q_brace - q) - 1.8 * wbc.POSTURE_KD[1] * v[act], -180.0, 180.0)
    tau_br = np.clip(M[act, :][:, act] @ qdd_br + M[act, :6] @ qacc[:6] + h[act], -clamp, clamp)
    qacc[act] = sw * qacc[act] + bw * qdd_br
    tau[:] = sw * tau + bw * tau_br
    kp_cv = np.array([3.2, 3.2, 1.0 + 2.2 * w_hop, 1.0 + 2.2 * w_hop]) * wbc.POSTURE_KD[0]
    kd_cv = np.array([1.8, 1.8, 1.0 + 0.8 * w_hop, 1.0 + 0.8 * w_hop]) * wbc.POSTURE_KD[1]
    qdd_cv = np.clip(kp_cv * (posture[cv] - q[cv]) - kd_cv * v[acv], -130.0, 130.0)
    tau_cv = Ms[acv, :][:, acv] @ qdd_cv + Ms[acv, :6] @ qacc[:6] + h[acv]
    qacc[acv] += w_up * (qdd_cv - qacc[acv])
    tau[cv] += w_up * (tau_cv - tau[cv])
    over[cv] = over[cv] & (w_up < 0.5)
    return np.diag(M)[act], np.concatenate([free_kp[leg] * (1.0 + (se2.STANDING_KP - 1.0) * float(stance[k]))
                                            for k, leg in enumerate(legs)])
