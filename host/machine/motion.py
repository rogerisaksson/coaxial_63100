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

from machine import borne, edge, estimate, grip, latent, modal, poise, se2, situation, stride, touch, wbc


def seen(g, dt):
    """`wbc.sense`'s view of her as her sensors and her estimate have her, `dt` s on: both in C bit
    for bit once built (`g['estimated']`, `laid`; `g['sensed']`, `sprung.laid`)."""
    body, limbs = g['body'], g['limbs']
    qpos, qvel = g['estimated'](g['est'], body, g['sense'], dt)
    s = g['sensed'](body, qpos, qvel)
    s['hips'], s['arms'] = body.d.xpos[limbs['hips']].copy(), s['q'][limbs['shoulders']]
    s['prop'], s['contacts'] = borne.read(body, s, g['sense'], float(g['fr'][modal.WEIGHT]))
    return s


def held_back(g, dt):
    """Rows that halt her rhythm - a stand, a hop's load - wait a pass while her capture point
    steps her or a swing is under way (`modal.NEED`, `modal.SWINGS`), every row after them, her
    labels and a hop's end with them: stepped through a hop's load, she left the floor rolling
    and came down at 60 deg. Held longer than two of her rhythm's swings (`latent.TURNS`) they
    come: the swing waited for is not coming - the brake's sway kept one commanded on two
    standing soles for ever, the stand that would end it never came, a sit asked held with it,
    and told to stop on ice she shuffled on 38 cm at 0.045 m/s. Each hold counts its own: a
    hop said 2.1 s after a stop, still stepping, loaded on one sole at once on the stand's. Her
    rhythm at rest, no swing comes: sitting, her centre of mass behind her heels, a seat asked
    was held for ever."""
    tab, t, a = g['tab'], g['t'], g['a']
    stepping = float((a[modal.NEED] > 0.0) | (a[modal.SWINGS].max() > 0.0))
    at = tab[:, latent.AT]
    halts = (tab[:, latent.SAID][:, latent.GAIN] > 0.5) & (tab[:, latent.VALS][:, latent.GAIN] < 0.05)
    due = halts & (at <= t + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)
    active = stepping * float(np.any(due))
    held_s = active * g.get('_held_s', 0.0)
    g['_held_s'] = held_s + active * dt
    turns = float(g['out'][latent.TURNS])
    within = float((turns > stride.TINY) & (held_s <= 4.0 * stride.PHI_W / max(turns, stride.TINY)))
    shift = dt * active * within
    first = float(np.min(at, where=due, initial=math.inf)) - latent.EDGE_S
    at += shift * (at >= first)
    g['labels'] = [(w + shift * (w >= first), n) for w, n in g['labels']]
    g['_jump_until'] = g.get('_jump_until', 0.0) + shift * (g.get('_jump_until', 0.0) > t)


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


def risen_back(g, dt):
    """Rows that raise her - her height asked up - while something bears her off her soles wait a
    pass, every row after them with them, while her capture point is off her soles: thrust up off
    a bed on the clock, her centre of mass 14 cm behind her soles' middle, she rose, her trunk
    came up and she stepped back as it let her go."""
    tab, t, y, fr = g['tab'], g['t'], g['y'], g['fr']
    has_prop = float(y[modal.PROP] > 0.0)
    g['_prop_until'] = has_prop * (t + 0.06) + (1.0 - has_prop) * g.get('_prop_until', -1.0)
    propped = float((y[modal.PROP] > 0.0) | (t <= g['_prop_until']))
    rot = fr[modal.ROT].reshape(3, 3)
    c, v = rot @ y[modal.COM], rot @ y[modal.VCOM]
    xi = c[:2] + v[:2] / math.sqrt(modal.G / max(float(c[2] - fr[modal.FLOOR]), 0.3))
    lo, hi = poise.boxes((y[modal.SOLE].reshape(2, 3) @ rot.T)[:, :2], g['a'][modal.EDGE], y[modal.TILT])
    outside = float(np.any((xi < lo.min(axis=0)) | (xi > hi.max(axis=0))))
    at, axial = tab[:, latent.AT], modal.AXIAL
    rises = ((tab[:, latent.SAID][:, latent.Z][:, axial] > 0.5)
             & (tab[:, latent.VALS][:, latent.Z][:, axial] < min(float(g['st'][latent.TO][latent.Z][axial]), 2.0) - 1e-6))
    due = rises & (at <= t + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)
    shift = dt * propped * outside * float(np.any(due))
    first = float(np.min(at, where=due, initial=math.inf)) - latent.EDGE_S
    at += shift * (at >= first)
    g['labels'] = [(w + shift * (w >= first), n) for w, n in g['labels']]


def _step_latent(g, dt):
    """Her latent state stepped, `latent.step` in C bit for bit (`g['latent']`, `laid`)."""
    st, tab, t = g['st'], g['tab'], g['t']
    hit = ((tab[:, latent.AT] <= t + latent.EDGE_S) & (tab[:, latent.AT] > t - dt + latent.EDGE_S)).astype(float)
    w_row = ((tab[:, latent.SAID].min(axis=1) > 0.5) & (tab[:, latent.VALS][:, latent.AMP].max(axis=1) > 0.05)).astype(float)
    wake = min(1.0, max(0.0, (0.04 - float(st[latent.NOW][latent.GAIN])) / 0.02)) * min(1.0, float(hit @ w_row))
    st[latent.THETA] *= 1.0 - wake
    w_ph = wake * np.minimum(hit @ tab[:, latent.SAID][:, latent.PHASE], 1.0)
    st[latent.NOW][latent.PHASE] += w_ph * (hit @ tab[:, latent.VALS][:, latent.PHASE] - st[latent.NOW][latent.PHASE])
    return g['latent'](st, tab, t, dt, g['out'])


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


def surety(g, dt, ct=math.nan):
    """Her directional surety field (`g['surety']` in [0, 1], `g['surety_vec']` = (s_ax, s_lat,
    d_ax, d_lat)) from her IMU, her soles' loads and her kinematics: highest in a wide double
    stance, lower on a single walking sole, held in quasi-static equilibrium on a seat or bed,
    contracting into her 2D tuck-and-brace regime on an unrecoverable topple and absorbing at 0."""
    y_in, y, fr, a = g['sense'], g['y'], g['fr'], g['a']
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
    k_ax = max(float(g['st'][latent.TO][latent.K][modal.AXIAL]), float(g['st'][latent.FROM][latent.K][modal.AXIAL]))
    w_jump = min(1.0, max(0.0, (k_ax - 15000.0) / 5000.0))
    w, f_soles = np.maximum(np.clip(loads / (0.25 * wt), 0.0, 1.0), w_jump), float(loads.sum()) / wt
    theta = math.radians(float(np.nan_to_num(ct, nan=edge.tilt(y_in))))
    theta_e = max(0.0, theta - abs(float(a[modal.WANT][0])))
    v_norm = float(np.linalg.norm(v))
    w_ext = min(1.0, max(0.0, (ap_y / modal.G - f_soles) / 0.45)) * math.exp(-(w_gyro / 1.2) ** 2 - (v_norm / 0.18) ** 2 - (theta_e / 0.65) ** 4)
    g['surety_probe'] = 0.008 * max(0.0, 4.0 * f_soles * (1.0 - f_soles)) * math.exp(-(w_gyro / 0.6) ** 2) * math.sin(math.tau * 6.0 * float(g.get('t', 0.0)))
    w_sum, w_both = float(w[0] + w[1]) + 1e-9, float(min(w[0], w[1]))
    lo_1, hi_1 = (w @ lo) / w_sum, (w @ hi) / w_sum
    b_lo = (1.0 - w_both) * lo_1 + w_both * lo.min(axis=0) - w_ext * 0.16
    b_hi = (1.0 - w_both) * hi_1 + w_both * hi.max(axis=0) + w_ext * 0.16
    mid, span = 0.5 * (b_lo + b_hi), np.maximum(0.5 * (b_hi - b_lo), 0.02)
    w_air = max(0.0, 1.0 - float(g['out'][latent.ZMP]), w_jump)
    w_prop = max(w_ext, float(a[modal.BORNE]), w_air)
    w_sup = min(1.0, 0.42 + 0.30 * float(w[0] + w[1]) + 0.22 * float(max(w[0], w[1])) + 0.55 * w_prop)
    cap = w_sup * (0.85 + 0.15 * (span / (span + np.array([0.04, 0.05]))))
    r_gait = 0.28 * min(1.0, float(g['st'][latent.NOW][latent.GAIN]))
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
    prev = float(g.get('surety', 1.0))
    diff = raw - prev
    step = min(1.0, dt / 0.012) * min(diff, 0.0) + min(1.0, dt / 0.12) * max(prev ** 3, 0.08 * float(a[modal.BORNE])) * max(diff, 0.0)
    cur = min(1.0, max(0.0, prev + step))
    g['surety'] = cur
    g['surety_vec'] = (float(s_vec[0]), float(s_vec[1]), float(d_xy[0]), float(d_xy[1]))
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
    held_back(g, dt * ev)
    risen_back(g, dt * ev)
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
    k_ax = max(float(g['st'][latent.TO][latent.K][modal.AXIAL]), float(g['st'][latent.FROM][latent.K][modal.AXIAL]))
    w_hop = float((k_ax > 15000.0) & (g['st'][latent.TO][latent.AMP][modal.DRIVE] < 0.1))
    imp[modal.BEND] *= sw
    imp[modal.SWUNG] *= sw
    imp[modal.PREFOLD] *= sw
    imp[modal.SOLE_ACC] *= sw
    imp[modal.FOLD] *= sw
    imp[modal.FOLD_RATE] *= sw
    imp[modal.STRIDES] *= sw
    imp[modal.COM_ACC] = np.array([sw * imp[modal.COM_ACC][0], sw * imp[modal.COM_ACC][1] - bw * 2.5, sw * imp[modal.COM_ACC][2]])
    q_brace = np.array([1.0, d_ax, d_lat]) @ g['brace']
    q_post = sw * (g['posture'] + dq) + bw * q_brace
    posed = g['poses'](imp, y, g['fr'], q_post, g['bend'], g['arms'], body.knees, g.get('_twist_clear', 0.0) * sw)
    ask = g['ask'] = modal.given(imp, y, posed, body.knees)
    ask['mu'] = g['held']
    cv, acv = g['_cv_sh'], g['_acv_sh']
    w_gait = min(1.0, max(0.0, (0.05 - float(g['st'][latent.NOW][latent.GAIN])) / 0.05))
    z_to = g['st'][latent.TO][latent.Z]
    ax = float(z_to[modal.AXIAL])
    rec = min(float(z_to[modal.PITCH]), float(np.min(g['tab'][:, latent.VALS][:, latent.Z][:, modal.PITCH], initial=0.0)))
    w_sit = min(1.0, max(0.0, (ax - 1.5) / 0.01)) * min(1.0, max(0.0, (rec + 0.2) / 0.01))
    w_base = sw * max(w_hop, (1.0 - float(g['a'][modal.STRIDES])) * w_gait * (1.0 - w_sit))
    w_up = w_base * np.array([1.0, 1.0, w_hop, w_hop])
    M = s['M'].copy()
    M[:6, acv] *= 1.0 - w_up[None, :]
    M[acv, :6] *= 1.0 - w_up[:, None]
    g['got'] = g['solve'](body, dict(s, M=M), dict(ask, wep=bool((g['left'] > 0.0) & (sr > 0.25)), derate=g['_derate']))
    act = body.act
    qdd_br = np.clip(3.5 * wbc.POSTURE_KD[0] * (q_brace - s['q']) - 1.8 * wbc.POSTURE_KD[1] * s['v'][act], -180.0, 180.0)
    tau_br = np.clip(s['M'][act, :][:, act] @ qdd_br + s['M'][act, :6] @ g['got']['qacc'][:6] + s['h'][act], -body.clamp, body.clamp)
    g['got']['qacc'][act] = sw * g['got']['qacc'][act] + bw * qdd_br
    g['got']['tau'][:] = sw * g['got']['tau'] + bw * tau_br
    kp_cv = np.array([3.2, 3.2, 1.0 + 2.2 * w_hop, 1.0 + 2.2 * w_hop]) * wbc.POSTURE_KD[0]
    kd_cv = np.array([1.8, 1.8, 1.0 + 0.8 * w_hop, 1.0 + 0.8 * w_hop]) * wbc.POSTURE_KD[1]
    qdd_cv = np.clip(kp_cv * (ask['posture'][cv] - s['q'][cv]) - kd_cv * s['v'][acv], -130.0, 130.0)
    tau_cv = M[acv, :][:, acv] @ qdd_cv + M[acv, :6] @ g['got']['qacc'][:6] + s['h'][acv]
    g['got']['qacc'][acv] += w_up * (qdd_cv - g['got']['qacc'][acv])
    g['got']['tau'][cv] += w_up * (tau_cv - g['got']['tau'][cv])
    g['got']['over'][cv] = g['got']['over'][cv] & (w_up < 0.5)
    g['jeff'] = np.diag(s['M'])[body.act]
    g['leg_kp'] = np.concatenate([g['free_kp'][leg] * (1.0 + (se2.STANDING_KP - 1.0) * float(ask['stance'][k]))
                                  for k, leg in enumerate(g['legs'])])


def laid(g):
    """Her drive-side step laid in C (`wbc_pass.c`, `tools.cores.wbc.Drives`): the struct
    pointing at `g`'s arrays once - the observers, the setpoints and their rates, the holds, the
    knees' spring, the feed and the clamps her drives are told (`g['cmd']`) - and what it reads
    by, kept beside it. Her latent step, her upper body's coupling and her ask in C, her python's
    bit for bit (`tools.cores.motion`); her estimate, her grip's feel and floor, her situation, her
    drive's cap and her ask's poses compiled from their numpy (`tools.cores.bodied`)."""
    from machine.figure import JOINTS
    from tools.cores import bodied, motion as c_motion, wbc as cores
    g['estimated'] = bodied.on_body(estimate.step, 'estimate', (0, 2), body=1)
    g['felt'] = bodied.on_args(grip.felt, 'felt', (0, 1, 2, 3, 4, 5, 7))
    g['outside'] = bodied.on_args(grip.outside, 'outside', (0, 1))
    g['readings'] = bodied.on_args(situation.readings, 'readings', (0, 1, 2, 3, 4, 5))
    g['read'] = bodied.on_args(situation.read, 'read', (0, 1))
    g['gripped'] = bodied.on_args(gripped, 'gripped', (0, 1, 2, 3, 4))
    g['poses'] = bodied.on_args(modal.poses, 'poses', (0, 1, 2, 3, 4, 5, 7))
    g['modal_sensed'] = bodied.on_args(modal.sensed, 'modal_sensed', (0, 1, 2), sources={0: g['sensed']})
    g['soles'] = bodied.on_body(touch.soles, 'soles', (1, 3, 4, 5, 6), body=0)
    g['impedance'] = functools.partial(c_motion.asked, *c_motion.laid())
    laid_ = c_motion.latent_laid()
    g['latent'], g['couple'] = functools.partial(c_motion.stepped, laid_), functools.partial(c_motion.coupled, laid_)
    body, joints = g['body'], g['joints']
    g['_skin_ids'] = np.array([body.m.body(n).id for n in edge.SKIN])
    g['_tips'] = np.array(touch.TIPS)
    g['_cv_sh'] = list(g['cervical']) + list(g['limbs']['shoulders'])
    g['_acv_sh'] = body.act[g['_cv_sh']]
    g['_derate'] = g['wep_lim'] / np.maximum(1.0, body.wep)
    names = [JOINTS[j] for j in joints]
    g['brace'] = np.array([[d.get(n, 0.0) for n in names] for d in (_BRACE_TUCK, _BRACE_AX, _BRACE_LAT)])
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
    got, p, kept, step = g['got'], *g['drives']
    held, wo = float(g['hold'].any()), g['wo']
    ys = surety(g, dt)
    drv = 1.0 - (1.0 - ys) ** 3
    wo_s = wo * drv
    k_ax = max(float(g['st'][latent.TO][latent.K][modal.AXIAL]), float(g['st'][latent.FROM][latent.K][modal.AXIAL]))
    in_hop = int(k_ax > 15000.0)
    kept['qacc'][:] = got['qacc']
    kept['tau'][:] = got['tau']
    kept['jeff'][:] = g['jeff']
    kept['over'][:] = got['over']
    step(ctypes.byref(p), dt, wo_s, wo_s ** 3, drv, in_hop, 1)
    g['left'] = min(se2.WEP_S, max(0.0, g['left'] + dt * (se2.WEP_FILL * (1.0 - held) - held)))
