"""Her surety field, how sure she is of her stance: wbc_surety.c's python on her law's arrays.

    cur, vec = surety.field(y_in, y, fr, a, st, out, prev, dt, ct, t)   # her law's is its C (`motion.surety`)
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.law import latent, modal, poise
from machine.senses import edge


def field(y_in, y, fr, a, st, out, prev, dt, ct, t):
    """(her surety `dt` s on from `prev`, in [0, 1]; (surety along, across, its direction's two,
    its probe)) of her edge's array `y_in`, modal's `y`, `fr` and `a`, her latent `st` and its
    `out` alone at her time `t` - `ct` her tilt, deg, where given (nan: her IMU's): highest in a wide
    double stance, lower on a single walking sole, held in quasi-static equilibrium on a seat or
    bed, contracting into her 2D tuck-and-brace regime on an unrecoverable topple and absorbing at
    0. In C bit for bit (`g['surety_field']`, `motion.laid`)."""
    rot = fr[modal.ROT].reshape(3, 3)
    c, v = rot @ y[modal.COM], rot @ y[modal.VCOM]
    om = math.sqrt(modal.G / max(float(c[2] - fr[modal.FLOOR]), 0.3))
    xi, flat = c[:2] + v[:2] / om, (y[modal.SOLE].reshape(2, 3) @ rot.T)[:, :2]
    lo, hi = poise.boxes(flat, a[modal.EDGE], y[modal.TILT])
    qw, qx, qj, qk = y_in[edge.QUAT]
    acc, gyro = y_in[edge.ACCEL], y_in[edge.GYRO]
    up_w = np.array([2.0 * (qx * qj - qw * qk), 1.0 - 2.0 * (qx * qx + qk * qk), 2.0 * (qj * qk + qw * qx)])
    r_y = np.array([2.0 * (qx * qj + qw * qk), 1.0 - 2.0 * (qx * qx + qk * qk), 2.0 * (qj * qk - qw * qx)])
    ap_y, w_gyro = float(r_y @ acc), float(np.linalg.norm(gyro))
    wt = max(float(fr[modal.WEIGHT]), 1.0)
    loads = np.maximum(y[modal.LOAD], y_in[edge.LOAD])
    k_ax = max(float(st[latent.TO][latent.K][modal.AXIAL]), float(st[latent.FROM][latent.K][modal.AXIAL]))
    w_jump = min(1.0, max(0.0, (k_ax - 15000.0) / 5000.0))
    w, f_soles = np.maximum(np.clip(loads / (0.25 * wt), 0.0, 1.0), w_jump), float(loads.sum()) / wt
    theta = math.radians(float(np.nan_to_num(ct, nan=edge.tilt(y_in))))
    # what bears her bears her up to lying flat on it: counted from upright, lain on the bed her surety
    # fell to 0.03 and its brace stood in for her law through her rise; past flat, rolled back over it
    theta_e = max(0.0, theta - max(abs(float(a[modal.WANT][0])), float(a[modal.BORNE]) * (0.5 * math.pi)))
    v_norm = float(np.linalg.norm(v))
    w_ext = min(1.0, max(0.0, (ap_y / modal.G - f_soles) / 0.45)) * math.exp(-(w_gyro / 1.2) ** 2 - (v_norm / 0.18) ** 2 - (theta_e / 0.65) ** 4)
    probe = 0.008 * max(0.0, 4.0 * f_soles * (1.0 - f_soles)) * math.exp(-(w_gyro / 0.6) ** 2) * math.sin(math.tau * 6.0 * t)
    w_sum, w_both = float(w[0] + w[1]) + 1e-9, float(min(w[0], w[1]))
    lo_1, hi_1 = (w @ lo) / w_sum, (w @ hi) / w_sum
    b_lo = (1.0 - w_both) * lo_1 + w_both * lo.min(axis=0) - w_ext * 0.16
    b_hi = (1.0 - w_both) * hi_1 + w_both * hi.max(axis=0) + w_ext * 0.16
    mid, span = 0.5 * (b_lo + b_hi), np.maximum(0.5 * (b_hi - b_lo), 0.02)
    # sure of the air where the rows let the polygon go (out[HELD]: a throw, a landing's
    # absorption) - and, kept, the read at the ZMP's slot that is a mode's phase (out[57], PSI):
    # the seats lie on it (the bed bore 0.00 of her without it; docs/TODO.md)
    w_air = max(0.0, 1.0 - float(out[latent.ZMP]), 1.0 - float(out[latent.HELD]), w_jump)
    w_prop = max(w_ext, float(a[modal.BORNE]), w_air)
    w_sup = min(1.0, 0.42 + 0.30 * float(w[0] + w[1]) + 0.22 * float(max(w[0], w[1])) + 0.55 * w_prop)
    cap = w_sup * (0.85 + 0.15 * (span / (span + np.array([0.04, 0.05]))))
    r_gait = 0.28 * min(1.0, float(st[latent.NOW][latent.GAIN]))
    r_lo, r_hi = np.minimum(b_lo, lo.min(axis=0) - r_gait), np.maximum(b_hi, hi.max(axis=0) + r_gait)
    out_xi = (xi - np.clip(xi, r_lo, r_hi)) * (1.0 - w_prop)
    w_tilt = float(np.hypot(gyro[0], gyro[2]))
    s_dir = cap * (0.45 + 0.55 * np.exp(-((out_xi / 0.24) ** 2) - 0.04 * ((v[:2] * (1.0 - w_prop)) ** 2) - (0.22 * w_tilt) ** 2))
    u_xy = np.array([float(up_w @ rot[0]), float(up_w @ rot[1])])
    e_vec = (xi - mid) * (1.0 - w_ext) + 0.35 * u_xy + 0.15 * v[:2]
    d_xy = e_vec / math.hypot(float(e_vec[0]), float(e_vec[1]), 0.08)
    f_att = max(0.0, 1.0 - (theta_e / 1.20) ** 4) ** 2
    s_vec = np.clip(s_dir * f_att, 0.0, 1.0)
    raw = float(np.mean(s_vec))
    diff = raw - prev
    step = min(1.0, dt / 0.012) * min(diff, 0.0) + min(1.0, dt / 0.12) * max(prev ** 3, 0.08 * float(a[modal.BORNE])) * max(diff, 0.0)
    cur = min(1.0, max(0.0, prev + step))
    return cur, np.array([s_vec[0], s_vec[1], d_xy[0], d_xy[1], probe])
