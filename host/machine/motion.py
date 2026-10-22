"""Her motion pass: what her sensors read, through her latent loop, to what her drives are told.

The world is never seen: her array is `machine.edge`'s.

    motion.control(g, dt)    # a pass of the loop, every `g['every']`: her ask solved, her legs' stiffness
    motion.drives(g, dt)     # every pass: each drive's setpoint, feed and clamp, `g['cmd']`

`g` is `sprung.laid`'s; its `sense` the edge's array, its `est` her estimate (`machine.estimate`).
"""
import ctypes
import functools
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import borne, edge, estimate, grip, latent, modal, poise, se2, situation, stride, touch, waits, wbc


#: Her axial stiffness in her latent state, where it goes and where it comes from; past HOP_K
#: (N/m) a hop's.
_AXIAL_K = tuple(block.start + latent.K.start + modal.AXIAL for block in (latent.TO, latent.FROM))
HOP_K = 15000.0


def axial_k(st):
    """Her axial stiffness asked in her latent `st`, the stiffer of where it goes and where it comes
    from, N/m."""
    return max(float(st[_AXIAL_K[0]]), float(st[_AXIAL_K[1]]))


def seen(g, dt):
    """`wbc.sense`'s view of her as her sensors and her estimate have her, `dt` s on: both in C bit
    for bit once built (`g['estimated']`, `laid`; `g['sensed']`, `sprung.laid`)."""
    body, limbs = g['body'], g['limbs']
    qpos, qvel = g['estimated'](g['est'], body, g['sense'], dt)
    s = g['sensed'](body, qpos, qvel)
    s['hips'], s['arms'] = body.d.xpos[limbs['hips']].copy(), s['q'][limbs['shoulders']]
    s['prop'], s['contacts'] = borne.read(body, s, g['sense'], float(g['fr'][modal.WEIGHT]))
    return s


def gripped(out, held, stance, caps, into):
    """`out` in the situations she is in (`into` (S,), `situation.read`): each mode's sway and its
    sine scaled down to each's caps (S, N; `situation.text`) as far as she is in it; her drive's
    further to her going no faster than sqrt(2 STRIDE_M grip g) on her standing soles' grip
    (`held`) - a stride she can stop within: 0.77 m/s on ice (0.1), 1.9 at her law's own 0.6.
    Asked 0.69 on ice at 0.02 she ran on and fell at 3.0-3.4 s."""
    grip_now = ((stance + 1e-9) / (stance.sum() + 2e-9)) @ held
    sway = np.abs(out[latent.SWAY])
    under = into[:, None]
    capped = np.where(np.isnan(caps), 1.0, np.minimum(1.0, np.nan_to_num(caps) / np.maximum(sway, 1e-9)))
    k = np.min(1.0 - under + under * capped, axis=0, initial=1.0)
    turns = out[latent.TURNS]
    cruise = 2.0 / math.pi * stride.STRIDE_M * (sway[modal.DRIVE] * k[modal.DRIVE]) * math.sin(stride.PHI_W) * turns
    k[modal.DRIVE] *= np.minimum(1.0, np.sqrt(2.0 * stride.STRIDE_M * grip_now * modal.G) / np.maximum(cruise, 1e-9))
    sine = out[latent.SWAY] * np.sin(out[latent.PSI])
    out[latent.REF] += (k - 1.0) * sine
    out[latent.RATE] += (k - 1.0) * out[latent.SWAY] * turns * np.cos(out[latent.PSI])
    out[latent.ACC] -= (k - 1.0) * sine * turns * turns
    out[latent.SWAY] *= k


def _step_latent(g, dt):
    """Her latent state stepped: `waits.woken_rows` and `latent.step` in C (`g['waits']`, `g['latent']`)."""
    g['waits'].woken(g['tab'], g['t'], dt)
    return g['latent'](g['st'], g['tab'], g['t'], dt, g['out'])


def _step_modal(g, s, y, y_in, out):
    """Her upper body's yield and the loop's ask: `support.couple` and `impedance.ask` in C bit
    for bit (`g['couple']`, `g['impedance']`, `laid`)."""
    qd = np.ascontiguousarray(y_in[edge.QD][g['joints']], dtype=np.float64)
    dq = g['couple'](np.ascontiguousarray(s['q'], dtype=np.float64), qd, g['ref_q'], g['ref_v'], g['eso'][2], out)
    return dq, g['impedance'](out, y, g['fr'], g['a'])


_BRACE_TUCK = {'spine': 0.45, 'neck': 0.35, 'left_hip_roll': 0.12, 'right_hip_roll': -0.12,
               'left_hip': -1.05, 'right_hip': -1.05, 'left_knee': 1.45, 'right_knee': 1.45,
               'left_ankle': -0.45, 'right_ankle': -0.45, 'left_shoulder': 0.80, 'right_shoulder': 0.80,
               'left_elbow': 0.65, 'right_elbow': 0.65}
_BRACE_AX = {'spine': -0.25, 'neck': -0.20, 'left_shoulder': 0.65, 'right_shoulder': 0.65,
             'left_elbow': -0.40, 'right_elbow': -0.40}
_BRACE_LAT = {'left_shoulder_roll': 0.45, 'right_shoulder_roll': 0.45, 'left_hip_roll': 0.15, 'right_hip_roll': 0.15}


def surety_field(y_in, y, fr, a, st, out, prev, dt, ct, t):
    """(her surety `dt` s on from `prev`, in [0, 1]; (surety along, across, its direction's two,
    its probe)) of her edge's array `y_in`, modal's `y`, `fr` and `a`, her latent `st` and its
    `out` alone at her time `t` - `ct` her tilt, deg, where given (nan: her IMU's): highest in a wide
    double stance, lower on a single walking sole, held in quasi-static equilibrium on a seat or
    bed, contracting into her 2D tuck-and-brace regime on an unrecoverable topple and absorbing at
    0. In C bit for bit (`g['surety_field']`, `laid`)."""
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
    theta_e = max(0.0, theta - abs(float(a[modal.WANT][0])))
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


def surety(g, dt, ct=math.nan):
    """Her surety field stepped `dt` s on her arrays (`surety_field`, in C): `g['surety']` in
    [0, 1], `g['surety_vec']` (s_ax, s_lat, d_ax, d_lat), `g['surety_probe']`."""
    cur, vec = g['surety_field'](float(g['surety']), dt, ct, float(g['t']))
    s_ax, s_lat, d_ax, d_lat, g['surety_probe'] = vec.tolist()
    g['surety'], g['surety_vec'] = cur, (s_ax, s_lat, d_ax, d_lat)
    return cur


def control(g, dt):
    """A pass of her latent loop on what she senses: `g['got']` solved, `g['leg_kp']` her legs'
    stiffness by their stance."""
    body, ev, y_in = g['body'], g['every'], g['sense']
    g['t_law'] = g['t']
    s = g['s'] = seen(g, dt * ev)
    g['grip'] = g['felt'](g['grip'], g['grip_seen'], body.d.xmat[g['grip_at']].reshape(2, 2, 3, 3),
                          y_in[edge.TOUCH].reshape(-1, 3),
                          g['est']['slip'], float(g['fr'][modal.WEIGHT]), dt * ev, g['grip_prior'])
    g['loads'] = [float(v) for v in y_in[edge.LOAD]]
    y = g['modal_sensed'](s, g['loads'], g['y'])
    y[modal.CELLS] = y_in[edge.CELLS]
    y[modal.GRIP] = g['held'] = g['outside'](g['grip'], g['situations']['grips'])
    g['into'] = g['read'](g['situations'], g['readings'](
        y, g['fr'], g['a'], g['grip'], y_in[edge.TOUCH].reshape(-1, 3), float(g['fr'][modal.WEIGHT])))
    waits.held_back(g, dt * ev)
    waits.risen_back(g, dt * ev)
    waits.landed_back(g, dt * ev)
    out = _step_latent(g, dt * ev)
    se2.step_physical_twist(g, dt * ev, out)
    g['gripped'](out, g['held'], g['a'][modal.STANCE], g['situations']['caps'], g['into'])
    touch.step(g, out, dt * ev)
    dq, imp = _step_modal(g, s, y, y_in, out)
    w_back = float(g.get('_w_back', 0.0))
    imp[modal.HEADED][1] += w_back * (float(g.get('_backing_yaw', 0.0)) - imp[modal.HEADED][1])
    sr = surety(g, dt * ev)
    bw = max(0.0, 1.0 - (sr / 0.58) ** 4) ** 2
    sw = 1.0 - bw
    _, _, d_ax, d_lat = g['surety_vec']
    w_hop = float((axial_k(g['st']) > HOP_K) & (g['st'][latent.TO][latent.AMP][modal.DRIVE] < 0.1))
    imp[modal.BEND] *= sw
    imp[modal.SWUNG] *= sw
    imp[modal.PREFOLD] *= sw
    imp[modal.SOLE_ACC] *= sw
    imp[modal.FOLD] *= sw
    imp[modal.FOLD_RATE] *= sw
    imp[modal.STRIDES] *= sw
    imp[modal.COM_ACC] = np.array([sw * imp[modal.COM_ACC][0], sw * imp[modal.COM_ACC][1] - bw * 2.5, sw * imp[modal.COM_ACC][2]])
    q_post = sw * (g['posture'] + dq) + bw * (np.array([1.0, d_ax, d_lat]) @ g['brace'])
    posed = g['poses'](imp, y, g['fr'], q_post, g['bend'], g['arms'], body.knees, g.get('_twist_clear', 0.0) * sw)
    ask = g['ask'] = modal.given(imp, y, posed, body.knees, g['bend'])
    ask['mu'] = g['held']
    cv, acv = g['_cv_sh'], g['_acv_sh']
    w_gait = min(1.0, max(0.0, (0.05 - float(g['st'][latent.NOW][latent.GAIN])) / 0.05))
    w_drive = min(1.0, abs(float(g['st'][latent.TO][latent.AMP][modal.DRIVE])) / stride.STIRS)
    foot = min(1.0, max(0.0, float(g['out'][latent.HELD]) - 1.0))
    z_to = g['st'][latent.TO][latent.Z]
    ax = float(z_to[modal.AXIAL])
    rec = min(float(z_to[modal.PITCH]), float(np.min(g['tab'][:, latent.VALS][:, latent.Z][:, modal.PITCH], initial=0.0)))
    w_sit = min(1.0, max(0.0, (ax - 1.5) / 0.01)) * min(1.0, max(0.0, (rec + 0.2) / 0.01))
    rest = (1.0 - float(g['a'][modal.STRIDES])) * w_gait
    w_base = sw * max(w_hop, rest * (1.0 - w_sit))
    # her neck holds her head's pitch itself also while her rhythm is told to run, held to her
    # landings' footprint, but told no drive - a brake: coupled, her step's rounds threw her head
    # 101-105 deg as her last landing let her strides go; her head's yaw turns with her
    brake = foot * (1.0 - w_drive) * float(g['st'][latent.TO][latent.GAIN])
    w_neck = sw * max(w_hop, max(rest, brake) * (1.0 - w_sit))
    w_up = np.array([w_neck, w_base, w_hop * w_base, w_hop * w_base])
    M = g['braced'].scaled(s['M'], w_up)
    g['got'] = g['solve'](body, dict(s, M=M), dict(ask, wep=bool((g['left'] > 0.0) & (sr > 0.25)), derate=g['_derate']))
    got = g['got']
    g['jeff'], g['leg_kp'] = g['braced'].blend(s['M'], M, s['q'], s['v'], s['h'], ask['posture'], got['qacc'], got['tau'],
                                               got['over'], sw, bw, d_ax, d_lat, w_hop, w_up, np.array(ask['stance'], float))
    _told(g)


def _told(g):
    """The solve's answer and her drives' inertia told her drives' step (`g['drives']`'s arrays,
    `laid`): what it reads every pass till her law's next."""
    got, kept = g['got'], g['drives'][1]
    kept['qacc'][:] = got['qacc']
    kept['tau'][:] = got['tau']
    kept['jeff'][:] = g['jeff']
    kept['over'][:] = got['over']


def scaled(M, acv, w_up):
    """The solve's M: `M` copied, the four dofs' (`acv`) rows and columns against the base scaled by
    1 - `w_up`. In C bit for bit (`g['braced']`, `laid`)."""
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
    drives' inertia and her `legs`' stiffness off `free_kp` by their `stance`. In C bit for bit
    (`g['braced']`, `laid`)."""
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


def laid(g):
    """Her drive-side step laid in C (`wbc_pass.c`, `tools.cores.wbc.Drives`): the struct
    pointing at `g`'s arrays once - the observers, the setpoints and their rates, the holds, the
    knees' spring, the feed and the clamps her drives are told (`g['cmd']`) - and what it reads
    by, kept beside it. Her latent step, her upper body's coupling, her ask, her surety field, her
    touch's filter, her twist's step, her table's waits and her surety's brace on the solve in C, her
    python's bit for bit (`tools.cores.motion`, `surety`, `touch`, `twist`, `wait`, `brace`); her estimate, her grip's feel and floor, her
    situation, her drive's cap and her ask's poses compiled from their numpy (`tools.cores.bodied`)."""
    from machine.figure import JOINTS
    from tools.cores import bodied, motion as c_motion, surety as c_surety, touch as c_touch, twist as c_twist
    from tools.cores import brace as c_brace, wait as c_wait, wbc as cores
    g['estimated'] = bodied.on_body(estimate.step, 'estimate', (0, 2), body=1)
    g['felt'] = bodied.on_args(grip.felt, 'felt', (0, 1, 2, 3, 4, 5, 7))
    g['outside'] = bodied.on_args(grip.outside, 'outside', (0, 1))
    g['readings'] = bodied.on_args(situation.readings, 'readings', (0, 1, 2, 3, 4, 5))
    g['read'] = bodied.on_args(situation.read, 'read', (0, 1))
    g['gripped'] = bodied.on_args(gripped, 'gripped', (0, 1, 2, 3, 4))
    g['poses'] = bodied.on_args(modal.poses, 'poses', (0, 1, 2, 3, 4, 5, 7))
    g['modal_sensed'] = bodied.on_args(modal.sensed, 'modal_sensed', (0, 1, 2), sources={0: g['sensed']})
    k, ask = c_motion.laid()
    g['impedance'] = functools.partial(c_motion.asked, k, ask)
    g['surety_field'] = c_surety.laid(k, g['sense'], g['y'], g['fr'], g['a'], g['st'], g['out'])
    laid_ = c_motion.latent_laid()
    g['latent'], g['couple'] = functools.partial(c_motion.stepped, laid_), functools.partial(c_motion.coupled, laid_)
    body, joints = g['body'], g['joints']
    g['_skin_ids'] = np.array([body.m.body(n).id for n in edge.SKIN])
    g['_tips'] = np.array(touch.TIPS)
    g['touched_step'] = c_touch.laid(k, g['sense'], body.d.xmat, body.d.xpos, g['_skin_ids'], g['_tips'], g['y'],
                                     g['fr'], g['a'], g['st'], g['out'], g['rest'], g['touched'])
    g['twist_step'] = c_twist.laid(k, g['sense'], g['y'], g['fr'], g['a'], g['st'], g['out'], g['posture'],
                                   g['twisted'], g['words'], g['waist_idx'], g['head_idx'])
    g['waits'] = c_wait.laid(k, g['a'], g['out'], g['y'], g['fr'], g['st'])
    g['_cv_sh'] = list(g['cervical']) + list(g['limbs']['shoulders'])
    g['_acv_sh'] = body.act[g['_cv_sh']]
    g['_derate'] = g['wep_lim'] / np.maximum(1.0, body.wep)
    names = [JOINTS[j] for j in joints]
    g['brace'] = np.array([[d.get(n, 0.0) for n in names] for d in (_BRACE_TUCK, _BRACE_AX, _BRACE_LAT)])
    g['braced'] = c_brace.laid(body, g['brace'], g['_cv_sh'], g['_acv_sh'], g['legs'], g['free_kp'])
    n = len(joints)
    kept = {'joints': np.ascontiguousarray(joints, dtype=np.int32),
            'act': np.ascontiguousarray(body.act, dtype=np.int32), 'sb': np.zeros(n), 'feed': np.zeros(n),
            'limit': np.zeros(n), 'clamp_j': np.ascontiguousarray(g['clamp'][joints], dtype=np.float64),
            'qacc': np.zeros(body.n, dtype=np.float64), 'tau': np.zeros(n, dtype=np.float64),
            'jeff': np.ascontiguousarray(g['jeff'], dtype=np.float64), 'over': np.zeros(n, dtype=np.bool_)}
    g['got'] = {'qacc': kept['qacc'].copy(), 'tau': kept['tau'].copy(), 'over': kept['over'].copy()}
    p = cores.Drives(n=n, q_at=edge.Q.start, qd_at=edge.QD.start, qm_at=edge.QM.start, knee_nm=se2.KNEE_NM,
                     knee_soft=se2.KNEE_SOFT, leak_s=se2.LEAK_S, wep_hold_s=se2.WEP_HOLD_S)
    p.knee[0], p.knee[1] = body.knees
    p.cervical[0], p.cervical[1] = g['cervical']
    for name, arr in (('joints', kept['joints']), ('act', kept['act'])):
        setattr(p, name, cores.pointer(arr, np.int32))
    for name, arr in (('y', g['sense']), ('clamp', body.clamp), ('wep', body.wep), ('wep_lim', g['wep_lim']),
                      ('clamp_j', kept['clamp_j']), ('q_lo', g['q_lo']), ('q_hi', g['q_hi']), ('eso', g['eso']),
                      ('ref_q', g['ref_q']), ('ref_v', g['ref_v']), ('hold', g['hold']), ('sb', kept['sb']),
                      ('feed', kept['feed']), ('limit', kept['limit']), ('qacc', kept['qacc']),
                      ('tau', kept['tau']), ('jeff', kept['jeff'])):
        setattr(p, name, cores.pointer(arr))
    p.over = cores.address(kept['over'])
    g['cmd'] = {'target': g['ref_q'], 'feed': kept['feed'], 'limit': kept['limit']}
    g['drives'] = (p, kept, cores.library().wbh_pass_drives)


def drives(g, dt):
    """Each driven joint's setpoint (`g['ref_q']`), feed and clamp (`g['cmd']`) this pass, its
    observer stepped on its motor's encoder: the step in C (`laid`), told here what of her it
    reads whole - her surety as she tilts or slips, a hop, which WEP limit is hers."""
    p, _kept, step = g['drives']
    held, wo = float(g['hold'].any()), g['wo']
    ys = surety(g, dt)
    drv = 1.0 - (1.0 - ys) ** 3
    wo_s = wo * drv
    in_hop = int(axial_k(g['st']) > HOP_K)
    step(ctypes.byref(p), dt, wo_s, wo_s ** 3, drv, in_hop, 1)
    g['left'] = min(se2.WEP_S, max(0.0, g['left'] + dt * (se2.WEP_FILL * (1.0 - held) - held)))
