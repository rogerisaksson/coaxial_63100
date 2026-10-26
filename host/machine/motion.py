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

from machine import barrier, bemf, borne, brace, capsules, edge, estimate, field, grip, inekf, latent, leap, modal, observer, ortho, poise, rounds, schema, se2, situation, somatic, stride, supervisor, touch, waits, wayfinder


#: A let-go leg's hip, rad (yaw, roll, pitch): its thigh level, a quarter turn down from an upright pelvis.
HUNG = np.array([0.0, 0.0, -0.5 * np.pi])
#: Her axial stiffness in her latent state, where it goes and where it comes from; past HOP_K
#: (N/m) a hop's.
_AXIAL_K = tuple(block.start + latent.K.start + modal.AXIAL for block in (latent.TO, latent.FROM))
HOP_K = 15000.0
#: A joint none of what bears her needs, of its clamp: the suite's passive cone (-32 dB). Her ankles
#: and knees are her soles' (`ortho.MANIFOLDS`), the rest of her let go as she lies (dorsal) or on
#: her side (trochanter): lying back on the bed, her law held to her soles, her legs swung 483 deg/s.
RELEASED = 10.0 ** (-32.0 / 20.0)
DISTAL = ('knee', 'ankle', 'ankle_roll')


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
    observer.step(g['observer'], s, qpos, dt)
    inekf.stepped(g['inekf'], body, g['sense'], g['observer']['soles'] / float(g['fr'][modal.WEIGHT]), dt,
                  g['inekf_points'], g['inekf_step'])
    s['hips'], s['arms'], s['elbows'], s['qpos'] = (body.d.xpos[limbs['hips']].copy(), s['q'][limbs['shoulders']],
                                                    s['q'][limbs['elbows']], qpos.copy())
    s['prop'], s['contacts'], s['palms'] = borne.read(body, s, g['sense'], float(g['fr'][modal.WEIGHT]), float(g['told'][ortho.PALMAR]))
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
    waits.borne_back(g, dt * ev)
    waits.latched_back(g, dt * ev)
    out = _step_latent(g, dt * ev)
    told = g['told'] = out[latent.MANIFOLDED].copy()
    out[latent.MANIFOLDED] = g['weighed'](g['observer'], told, float(g['fr'][modal.WEIGHT]), dt * ev)
    out[latent.HELD] *= out[latent.MANIFOLDED][0]
    unfelt, lean = float(out[latent.MANIFOLDED][0] - observer.said(told)[0]), float(told[ortho.FURNITURE].sum())
    g['seek'](g['field'], field.due(g['tab'], g['t'], dt * ev), y, lean, float(g['st'][latent.TO][latent.Z][modal.AXIAL]), dt * ev, out)
    se2.step_physical_twist(g, dt * ev, out)
    g['gripped'](out, g['held'], g['a'][modal.STANCE], g['situations']['caps'], g['into'])
    touch.step(g, out, dt * ev)
    dq, imp = _step_modal(g, s, y, y_in, out)
    w_back = float(g.get('_w_back', 0.0))
    imp[modal.HEADED][1] += w_back * (float(g.get('_backing_yaw', 0.0)) - imp[modal.HEADED][1])
    gauge = wayfinder.gauged(g, s)
    imp[modal.HEADED][1] += gauge
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
    posed = g['poses'](imp, y, g['fr'], q_post, g['bend'], g['arms'], g['reaches'], g['folds'], body.knees, g.get('_twist_clear', 0.0) * sw)
    ask = g['ask'] = modal.given(imp, y, posed, body.knees, g['bend'])
    # a leg let go (`impedance.ask`) hangs from a hip sprung to its thigh along what bears her - level in the world,
    # a quarter turn down from her pelvis less its pitch (HUNG, `modal._turns`) -, its knee and ankle hanging: sprung
    # where it was let go, or a quarter turn from a pelvis tilted 11-18 deg forward, the thigh pointed 15 deg down
    # off the seat's edge and the hanging foot stood on the floor bearing 0.3 of her through a hitch; sprung to its
    # posture's hip, a crouch's 45 deg, it floated and her trunk drifted 3 -> 26 deg forward in 4 s
    hips = np.array([leg[:3] for leg in rounds.LEGS])
    let_go = np.array(ask['let_go'], float)[:, None]
    hung = HUNG + np.array([0.0, 0.0, float(modal._turns(y, g['fr'][modal.ROT].reshape(3, 3))[0][0, 0])])
    ask['posture'][hips] = let_go * (hung - ask['posture'][hips]) + ask['posture'][hips]
    somatic.asked(g, s, out, told, ask)
    bears = np.array(g['loads']) >= leap.TOUCH_N
    held = np.concatenate([s['feet'][k]['pts'] for k in np.flatnonzero(bears | ~bears.any())] + [observer.bearing(g['observer'])])
    ask['com_acc'] = barrier.barred(ask['com_acc'], s['com'], s['vcom'], float(g['fr'][modal.OMEGA]), held,
                                    (1.0 - float(g['a'][modal.STRIDES])) * float(bears.any()) * unfelt, g['ringed'])
    ask['posture'][g['head_idx']] -= gauge
    ask['posture'], g['schema'] = schema.shielded(body, g['clear'](g['schema_body'], s, g['schema_at']), s, ask,
                                                  float(y_in[edge.LINK][0]))
    ask['mu'] = g['held']
    arms = [modal.ARMS, modal.REACH, modal.ELBOWS]
    ask['hold'] = max(1.0, float((g['out'][latent.KS][arms] / g['rest'][latent.K][arms]).max()))
    cv, acv = g['_cv_sh'], g['_acv_sh']
    w_gait = min(1.0, max(0.0, (0.05 - float(g['st'][latent.NOW][latent.GAIN])) / 0.05))
    w_drive = min(1.0, abs(float(g['st'][latent.TO][latent.AMP][modal.DRIVE])) / stride.STIRS)
    foot = min(1.0, max(0.0, float(g['out'][latent.HELD]) - 1.0))
    # a sole up while what bears her bears her is no stride: counted one, lying back her neck let
    # go 0.36 s and flexed 64 deg at 1034 deg/s
    rest = (1.0 - float(g['a'][modal.STRIDES]) * (1.0 - float(g['a'][modal.BORNE]))) * w_gait
    w_base = sw * max(w_hop, rest)
    # her neck holds her head's pitch itself also while her rhythm is told to run, held to her
    # landings' footprint, but told no drive - a brake: coupled, her step's rounds threw her head
    # 101-105 deg as her last landing let her strides go; her head's yaw turns with her. Seated
    # too: let go there, a rise's lean whipped her head 71-98 deg back and asked her neck 10 times
    # its clamp, the whole ask shrunk to 0.04 and she collapsed onto the seat
    brake = foot * (1.0 - w_drive) * float(g['st'][latent.TO][latent.GAIN])
    w_neck = sw * max(w_hop, max(rest, brake))
    w_up = np.array([w_neck, w_base, w_hop * w_base, w_hop * w_base])
    M = g['braced'].scaled(s['M'], w_up)
    lo, hi = bemf.bounds(s['v'][body.act], float(y_in[edge.LINK][0]))
    g['got'] = g['solve'](body, dict(s, M=M), dict(ask, wep=bool((g['left'] > 0.0) & (sr > 0.25)), derate=g['_derate'],
                                                    lo=lo, hi=hi))
    got = g['got']
    g['jeff'], g['leg_kp'] = g['braced'].blend(s['M'], M, s['q'], s['v'], s['h'], ask['posture'], got['qacc'], got['tau'],
                                               got['over'], sw, bw, d_ax, d_lat, w_hop, w_up, np.array(ask['stance'], float))
    _told(g)
    w = g['out'][latent.MANIFOLDED]
    g['release'][:] = 1.0 - (1.0 - RELEASED) * (1.0 - (g['distal'] * w[0] + (1.0 - g['distal']) * (1.0 - w[2] - w[3])))


def _told(g):
    """The solve's answer and her drives' inertia told her drives' step (`g['drives']`'s arrays,
    `laid`): what it reads every pass till her law's next."""
    got, kept = g['got'], g['drives'][1]
    kept['qacc'][:] = got['qacc']
    kept['tau'][:] = got['tau']
    kept['jeff'][:] = g['jeff']
    kept['over'][:] = got['over']


def laid(g):
    """Her drive-side step laid in C (`wbc_pass.c`, `tools.cores.wbc.Drives`): the struct
    pointing at `g`'s arrays once - the observers, the setpoints and their rates, the holds, the
    knees' spring, the feed and the clamps her drives are told (`g['cmd']`) - and what it reads
    by, kept beside it. Her latent step, her upper body's coupling, her ask, her surety field, her
    touch's filter, her twist's step, her table's waits and her surety's brace on the solve in C, her
    python's bit for bit (`tools.cores.motion`, `step`, `surety`, `touch`, `twist`, `wait`, `brace`); her estimate, her grip's feel and floor, her
    situation, her drive's cap, her ask's poses, her invariant EKF, her barrier (a ring's length a part), her
    field's seek, what bears her weighed, her shield's clearances and her wayfinder's gap compiled from
    their numpy (`tools.cores.bodied`)."""
    from machine.figure import JOINTS
    from tools.cores import bodied, motion as c_motion, surety as c_surety, touch as c_touch, twist as c_twist
    from tools.cores import brace as c_brace, wait as c_wait, wbc as cores
    g['estimated'] = bodied.on_body(estimate.step, 'estimate', (0, 2), body=1)
    g['felt'] = bodied.on_args(grip.felt, 'felt', (0, 1, 2, 3, 4, 5, 7))
    g['weighed'] = bodied.on_args(observer.weighed, 'weighed', (0, 1, 2, 3))
    g['clear'] = bodied.on_body(capsules.felt, 'clear', (1,))
    g['gap'] = bodied.on_shapes(wayfinder.gap, 'gap', (0, 1, 2, 3, 4, 5))
    g['outside'] = bodied.on_args(grip.outside, 'outside', (0, 1))
    g['readings'] = bodied.on_args(situation.readings, 'readings', (0, 1, 2, 3, 4, 5))
    g['read'] = bodied.on_args(situation.read, 'read', (0, 1))
    g['gripped'] = bodied.on_args(gripped, 'gripped', (0, 1, 2, 3, 4))
    g['poses'] = bodied.on_args(modal.poses, 'poses', (0, 1, 2, 3, 4, 5, 6, 7, 9))
    g['ringed'] = bodied.on_shapes(barrier.ringed, 'ringed', (0, 1, 2, 3, 4, 5))
    g['modal_sensed'] = bodied.on_args(modal.sensed, 'modal_sensed', (0, 1, 2), sources={0: g['sensed']})
    g['inekf_points'] = bodied.on_body(inekf.points, 'inekf_points', (2, 3))
    g['inekf_step'] = bodied.on_args(inekf.step, 'inekf', (0, 2, 3, 4))
    g['seek'] = bodied.on_args(field.seek, 'seek', (0, 1, 2, 3, 4, 6))
    k, ask = c_motion.laid()
    g['impedance'] = functools.partial(c_motion.asked, k, ask)
    g['surety_field'] = c_surety.laid(k, g['sense'], g['y'], g['fr'], g['a'], g['st'], g['out'])
    laid_ = c_motion.latent_laid()
    g['latent'], g['couple'] = functools.partial(c_motion.stepped, laid_), functools.partial(c_motion.coupled, laid_)
    body, joints = g['body'], g['joints']
    g['_skin_ids'] = np.array([body.m.body(n).id for n in edge.SKIN])
    g['schema_at'], g['schema_body'] = capsules.ids(body.m), capsules.Own(body)
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
    g['brace'] = np.array([[d.get(n, 0.0) for n in names] for d in (brace.TUCK, brace.AX, brace.LAT)])
    g['braced'] = c_brace.laid(body, g['brace'], g['_cv_sh'], g['_acv_sh'], g['legs'], g['free_kp'])
    n = len(joints)
    kept = {'joints': np.ascontiguousarray(joints, dtype=np.int32),
            'act': np.ascontiguousarray(body.act, dtype=np.int32), 'sb': np.zeros(n), 'feed': np.zeros(n),
            'limit': np.zeros(n), 'clamp_j': np.ascontiguousarray(g['clamp'][joints], dtype=np.float64),
            'qacc': np.zeros(body.n, dtype=np.float64), 'tau': np.zeros(n, dtype=np.float64),
            'jeff': np.ascontiguousarray(g['jeff'], dtype=np.float64), 'over': np.zeros(n, dtype=np.bool_),
            'release': np.ones(n)}
    g['release'], g['distal'] = kept['release'], np.array([float(nm.endswith(DISTAL)) for nm in names])
    g['got'] = {'qacc': kept['qacc'].copy(), 'tau': kept['tau'].copy(), 'over': kept['over'].copy()}
    k, bus, _arrays = g['bus'] = supervisor.bus(names, g['rate_hz'])
    p = cores.Drives(n=n, q_at=edge.Q.start, qd_at=edge.QD.start, qm_at=edge.QM.start, knee_nm=se2.KNEE_NM,
                     knee_soft=se2.KNEE_SOFT, leak_s=se2.LEAK_S, wep_hold_s=se2.WEP_HOLD_S,
                     bus_k=ctypes.addressof(k), bus=ctypes.addressof(bus), posture_kd=supervisor.POSTURE_KD)
    p.knee[0], p.knee[1] = body.knees
    p.cervical[0], p.cervical[1] = g['cervical']
    for name, arr in (('joints', kept['joints']), ('act', kept['act'])):
        setattr(p, name, cores.pointer(arr, np.int32))
    for name, arr in (('y', g['sense']), ('clamp', body.clamp), ('wep', body.wep), ('wep_lim', g['wep_lim']),
                      ('clamp_j', kept['clamp_j']), ('q_lo', g['q_lo']), ('q_hi', g['q_hi']), ('eso', g['eso']),
                      ('ref_q', g['ref_q']), ('ref_v', g['ref_v']), ('hold', g['hold']), ('sb', kept['sb']),
                      ('feed', kept['feed']), ('limit', kept['limit']), ('qacc', kept['qacc']),
                      ('tau', kept['tau']), ('jeff', kept['jeff']), ('release', kept['release'])):
        setattr(p, name, cores.pointer(arr))
    p.over = cores.address(kept['over'])
    g['cmd'] = {'target': g['ref_q'], 'feed': kept['feed'], 'limit': kept['limit'].copy()}
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
    # a setpoint within its drive's range (the manifest's): asked compression 4.0 the pattern's
    # ankle ask of -77 deg wound the joint to 215.8 of its -45..42 and felled her at 3.0 s.
    np.clip(g['ref_q'], g['rng'][:, 0], g['rng'][:, 1], out=g['ref_q'])
    # her clamps told as the step left them, zero where she rests limp (`se2.LIMP`), into a buffer laid
    # once: rebound to its own product a pass, the clamps froze at the first pass's and a rise from
    # the low chair fell at 31.9 s that stood
    from machine import se2, sprung
    np.multiply(_kept['limit'], 1.0 - float(sprung.doing(g) in se2.LIMP), out=g['cmd']['limit'])
    g['left'] = min(se2.WEP_S, max(0.0, g['left'] + dt * (se2.WEP_FILL * (1.0 - held) - held)))
