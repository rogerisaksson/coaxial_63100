"""Static physical, kinematic and cybernetic meta-compiler for her body, servos and gait.

    st = compile.subtrees()          # composite rigid-body masses, levers and inertias standing
    sv = compile.servos(overrides)   # {kind: (peak N m, kp N m/rad, kd N m s/rad, armature kg m^2)}
    gt = compile.gait(overrides)     # {leg_m, mass_kg, modes, rhythm, steer, poise, ...}
"""
from math import acos, asin, atan2, cos, degrees, dist, hypot, isnan, pi, radians, sin, sqrt, tau
import os
from typing import Any

from machine.body import build, drives, figure, mechatronics

_ENV, _BUILD, _LIB = mechatronics.environment(), mechatronics.robot(), mechatronics.library()
_BUS, _INV = _BUILD['bus'], _LIB['inverters'][_BUILD['stacks']['knee'][2]]
G_M_S2, _BUS_HZ, _LAW_DIV = float(_ENV['g_m_s2']), float(_BUS['bus_hz']), float(_BUS['law_div'])
_LAW_HZ, ROTOR_FF, _ESO_WO = _BUS_HZ / _LAW_DIV, round(1.0 / (1.0 + _LAW_DIV), 2), 0.10 * _BUS_HZ
ENC_RAD = tau / float(_INV['encoder_counts'])
_MU_DRY, _KIN_SHARE = float(_ENV['mu_dry']), float(_ENV['kinetic_share'])
_TAKES_NPZ = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'texts', 'takes.npz')
_AX, _AY, _AZ = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)


def _joint_of(kind):
    return kind if kind in figure.JOINTS else 'left_' + kind


def crb(segs, placed, names, pivot, axis):
    present = [n for n in names if n in segs]
    m = sum(segs[n][0] for n in present)
    com = [sum(segs[n][0] * figure.add(placed[n][0], figure.apply(placed[n][1], segs[n][1]))[i]
               for n in present) / m for i in range(3)]
    j_ax = 0.0
    for n in present:
        kg, c, (ixx, iyy, izz, ixy, ixz, iyz) = segs[n]
        at, rot = placed[n]
        pw = figure.add(at, figure.apply(rot, c))
        i_w = figure.mul(figure.mul(rot, ((ixx, ixy, ixz), (ixy, iyy, iyz), (ixz, iyz, izz))), figure.t(rot))
        d = [pw[i] - pivot[i] for i in range(3)]
        du = sum(d[i] * axis[i] for i in range(3))
        u_iu = sum(axis[i] * sum(i_w[i][k] * axis[k] for k in range(3)) for i in range(3))
        j_ax += u_iu + kg * (sum(v * v for v in d) - du * du)
    dc = [com[i] - pivot[i] for i in range(3)]
    du_c = sum(dc[i] * axis[i] for i in range(3))
    return m, sqrt(max(0.0, sum(v * v for v in dc) - du_c * du_c)), j_ax


def _dressed_statics(segs, pose, y_pelvis):
    placed = figure.frames(pose, (0.0, y_pelvis, 0.0), figure.IDENT)
    mt = sum(s[0] for s in segs.values())
    my = sum(segs[n][0] * figure.add(placed[n][0], figure.apply(placed[n][1], segs[n][1]))[1] for n in segs)
    garm = _BUILD.get('garments', {})
    hangs = [('hem', s + '_shank', (0.0, -figure.HEM_AT, 0.0)) for s in ('left', 'right')]
    for key, host, off in hangs + [('hair', 'head', figure.HAIR_AT)]:
        if key in garm:
            mh, lh = float(garm[key]['kg']), float(garm[key]['length_m'])
            mt += mh
            my += mh * (figure.add(placed[host][0], figure.apply(placed[host][1], off))[1] - lh)
    sole_y = figure.add(placed['left_foot'][0], figure.apply(
        placed['left_foot'][1], (0.0, -figure.ANKLE_H, 0.5 * (figure.BALL - figure.HEEL))))[1]
    return mt, my / mt - sole_y


def _subtrees():
    pose, origin = figure.stand(), (0.0, figure.standing(), 0.0)
    placed, segs = figure.frames(pose, origin, figure.IDENT), build.segments()
    kids: dict[Any, list[str]] = {}
    for seg in figure.SEGMENTS:
        kids.setdefault(seg[1], []).append(seg[0])
    stages = {seg[0]: [j for j, *_ in seg[2][:-1]] for seg in figure.SEGMENTS}

    def tree(root):
        out = [root] + [j for j in stages.get(root, ()) if j in segs]
        for child in kids.get(root, ()):
            out += tree(child)
        return out

    def sub(root, axis=_AX, pl=placed):
        return crb(segs, pl, tree(root), pl[root][0], axis)

    mass = figure.mass()
    m_dressed, h0 = _dressed_statics(segs, pose, origin[1])
    whole = list(segs)
    _, rx_ank, jx_ank = crb(segs, placed, whole, placed['left_foot'][0], _AX)
    _, rz_sole, jz_sole = crb(segs, placed, whole, (figure.HIP_HALF, 0.0, 0.0), _AZ)
    m_tr, r_tr, j_tr_x = sub('torso')
    m_lg, r_lg, j_lg_x = sub('left_thigh')
    qa, ql = degrees(0.25 * pi), degrees(0.5 * pi)
    flex = figure.frames({**pose, 'left_shoulder': qa, 'right_shoulder': qa, 'left_elbow': qa,
                          'right_elbow': qa, 'left_hip': -ql, 'right_hip': -ql, 'left_knee': ql,
                          'right_knee': ql}, origin, figure.IDENT)
    fold_deg = degrees(atan2(figure.SHANK, figure.THIGH + figure.SHANK))
    sw_pose = figure.frames({**pose, 'left_hip': -0.5 * fold_deg, 'left_knee': fold_deg}, origin, figure.IDENT)
    sq_pose = figure.frames(figure.SQUAT, origin, figure.IDENT)
    j_wa_y = 0.5 * (sub('torso', _AY)[2] + sub('torso', _AY, flex)[2])
    j_hy_y = 0.5 * (sub('left_thigh', _AY)[2] + sub('left_thigh', _AY, sq_pose)[2])
    return {
        'mass': mass, 'weight': m_dressed * G_M_S2, 'h0': h0, 'omega': sqrt(G_M_S2 / h0),
        'leg_m': figure.THIGH + figure.SHANK,
        'pend_x': jx_ank, 'pend_ank': (rx_ank, jx_ank), 'pend_sole': (rz_sole, jz_sole),
        'trunk': (m_tr, r_tr, j_tr_x, j_wa_y), 'leg': (m_lg, r_lg, j_lg_x, j_hy_y),
        'swing_leg': sub('left_thigh', _AX, sw_pose), 'neck': sub('neck'), 'head': sub('head', _AY),
        'arm': sub('left_upper_arm'), 'fore': sub('left_forearm'),
        'elbow_x': crb(segs, placed, ['left_upper_arm'], placed['left_forearm'][0], _AX)[2],
        'hand': sub('left_hand'), 'foot': sub('left_foot'),
        'neck_span': dist(placed['torso'][0], placed['neck'][0]),
        'unsprung_kg': 0.5 * segs['left_shank'][0] + segs['left_foot'][0] + segs['left_toes'][0],
    }


def _servos(st):
    m_tot, w_tot, h0 = st['mass'], st['weight'], st['h0']
    m_tr, r_tr, j_tr_x, j_tr_y = st['trunk']
    _, _, _, j_lg_y = st['leg']
    m_nk, r_nk, j_nk_x = st['neck']
    m_hd, _, j_hd_y = st['head']
    w_task = 0.1 * _ESO_WO / sqrt(figure.SCALE)
    w_inner, zeta_opt = 3.0 * w_task, sqrt(0.5)

    def j_drv(k):
        j = drives.armature(_joint_of(k))
        return j if j > 0.0 else (1.0 + drives.GEAR_J) * drives.size(k).rotor * drives.RATIO ** 2

    def sv(k, pk, kp, kd):
        a = (0.5 * ROTOR_FF * drives.armature('waist') if k == 'waist'
             else (ROTOR_FF + drives.GEAR_J) * drives.size(k).rotor * drives.RATIO ** 2)
        return (round(pk, 1), float(kp), float(kd), round(a, 2))

    def pd_joint(k, j_eff, zeta, bw=w_inner):
        k_res = 0.5 * drives.BOX_ZETA * drives.wind(_joint_of(k))
        kp = float(round(min(k_res, bw * bw * j_eff) if k_res > 0.0 else bw * bw * j_eff))
        return kp, float(round(2.0 * zeta * sqrt(kp * j_eff)))

    kp_ank = float(round(w_tot * h0 * h0 / (figure.BALL + figure.HEEL)))
    kd_ank = float(round(2.0 * sqrt(kp_ank * (st['foot'][2] + j_drv('ankle')))))
    j_kn = 0.5 * m_tot * (h0 - figure.ANKLE_H - figure.SHANK) ** 2 + j_drv('knee')
    kp_st, kd_hip = pd_joint('hip', 0.5 * m_tot * (0.5 * h0) ** 2 + j_drv('hip'), 0.5 * zeta_opt)
    kd_knee = float(round(zeta_opt * sqrt(kp_st * j_kn)))
    kp_sp, kd_sp = pd_joint('spine', j_tr_x + j_drv('spine'), 0.5)
    kp_wa, kd_wa = pd_joint('waist', j_tr_y + 1.5 * j_drv('waist'), 0.67)
    kp_hy, kd_hy = pd_joint('hip_yaw', j_lg_y + j_drv('hip_yaw'), 0.5)
    kp_nk, kd_nk = pd_joint('neck', j_nk_x + j_drv('neck'), zeta_opt, 2.0 * w_inner)
    kp_hd, kd_hd = pd_joint('head', j_hd_y + j_drv('head'), 0.5, 2.0 * w_task)
    kp_sh, kd_sh = pd_joint('shoulder', st['arm'][2] + j_drv('shoulder'), zeta_opt)
    kp_el, kd_el = pd_joint('elbow', st['elbow_x'] + st['fore'][2] + j_drv('elbow'), 0.5)
    kp_wr, kd_wr = pd_joint('wrist', st['hand'][2] + j_drv('wrist'), zeta_opt)
    kp_gr, kd_gr, _ = drives.PASSIVE.get('gripper', (40.0, 1.0, 0.0))

    def wep_sat(joint):
        s = drives.of(joint)[1]
        return drives.kt(joint) * s.amps * (s.rated / s.amps) ** (2.0 / 3.0)

    leg_wep = wep_sat('left_knee')
    sp_wep = max(drives.peak('spine'), wep_sat('spine') - m_tr * G_M_S2 * r_tr)
    nk_whip = m_nk * G_M_S2 * r_nk + (j_nk_x + m_nk * r_nk * st['neck_span']) * (sp_wep / j_tr_x)
    hd_fall = m_hd * (sqrt(kp_nk / (j_nk_x + j_drv('neck'))) * sqrt(2.0 * G_M_S2 * h0)) * sqrt(j_hd_y / m_hd)
    palm = w_tot / 3.0
    mu_sole = float(_BUILD['elastomers']['sole']['friction'])
    return {
        'spine': sv('spine', sp_wep, kp_sp, kd_sp),
        'spine_roll': sv('spine_roll', min(drives.peak('spine_roll'), drives.peak('waist') * zeta_opt), kp_sp, kd_sp),
        'waist': sv('waist', drives.peak('waist') * zeta_opt, kp_wa, kd_wa),
        'neck': sv('neck', min(drives.peak('neck'), nk_whip), kp_nk, kd_nk),
        'head': sv('head', min(drives.peak('head'), hd_fall), kp_hd, kd_hd),
        'shoulder': sv('shoulder', min(drives.peak('left_shoulder'), palm * hypot(figure.UPPER_ARM, figure.FOREARM)),
                       kp_sh, kd_sh),
        'elbow': sv('elbow', min(drives.peak('left_elbow'), palm * figure.FOREARM), kp_el, kd_el),
        'wrist': sv('wrist', palm * abs(figure.FIST_AT[1]), kp_wr, kd_wr),
        'gripper': sv('gripper', kp_gr * radians(15.0), kp_gr, kd_gr),
        'hip_yaw': sv('hip_yaw', min(drives.peak('left_hip_yaw'), mu_sole * w_tot * (figure.BALL + figure.HEEL)),
                      kp_hy, kd_hy),
        'hip_roll': sv('hip_roll', drives.peak('left_hip_roll') * zeta_opt, kp_st, kd_hip),
        'hip': sv('hip', leg_wep, kp_st, kd_hip),
        'knee': sv('knee', leg_wep, kp_st, kd_knee),
        'ankle': sv('ankle', drives.peak('left_ankle') * zeta_opt, kp_ank, kd_ank),
        'ankle_roll': sv('ankle_roll', 0.5 * drives.peak('left_ankle'), kp_ank, kd_ank),
        'foot': sv('foot', drives.TOE_K * atan2(figure.BALL, figure.ANKLE_H), 4.0 * drives.TOE_K, 0.1 * drives.TOE_K),
    }


def _standing_jump():
    import numpy as np
    ax_m, pt_m = mechatronics.MODES.index('axial_compression'), mechatronics.MODES.index('pitch_attitude')
    s_take = sqrt(figure.SCALE)
    with np.load(_TAKES_NPZ) as kept:
        t, z = kept['standing_jump_t'], kept['standing_jump_z'].astype(float)
    crouch = int(np.argmax(z[:, ax_m]))
    off = crouch + int(np.argmax(np.diff(z[crouch:, ax_m]) > 0.0))
    leaves = float((z[off - 1, ax_m] - z[off, ax_m]) / (t[off] - t[off - 1]))
    return {
        'crouch_s': round(float(t[crouch]), 3) * s_take, 'takeoff_s': round(float(t[off]), 3) * s_take,
        'crouch': round(float(z[crouch, ax_m]), 2), 'lean': round(float(z[crouch, pt_m]), 2),
        'straight': round(float(z[off, ax_m]), 2), 'leaves_1_s': round(leaves, 1) / s_take,
    }


def _knee_chord(deg):
    t, s = figure.THIGH, figure.SHANK
    return sqrt(t * t + s * s + 2.0 * t * s * cos(radians(deg)))


def _modes_gait(st, k_task, limb_k, poise_margin, mu):
    leg_m, h0, w0 = st['leg_m'], st['h0'], st['omega']
    ank = abs(drives.TRAVEL['ankle'][0]) - drives.TRAVEL['ankle_roll'][1]
    crouch = {**figure.stand(), 'left_hip': -ank, 'right_hip': -ank, 'left_knee': 2.0 * ank,
              'right_knee': 2.0 * ank, 'left_ankle': -ank, 'right_ankle': -ank}
    _, h_crouch = _dressed_statics(build.segments(), crouch, figure.pelvis_at(radians(2.0 * ank)))
    w_sole = figure.SOLE_HALF + figure.PAD_M
    k_ax_spec, k_g_spec = 0.5 * (k_task + limb_k), G_M_S2 / h0
    k_sag_share = (leg_m / (figure.BALL + figure.HEEL)) * (k_g_spec / k_ax_spec)
    return {
        'axial_m': round((h0 - h_crouch) / figure.SCALE, 3) * figure.SCALE,
        'pitch_rad': round(asin(min(1.0, (figure.BALL - poise_margin) / st['trunk'][1])), 2),
        'clear_m': figure.CONTACTS[1][2][1] + figure.PAD_M,
        'list_rad': round(atan2(figure.HIP_HALF + figure.SOLE_HALF - poise_margin, h0 - figure.ANKLE_H), 2),
        'turn_rad': round(atan2(figure.HIP_HALF, 0.5 * leg_m + figure.ANKLE_H), 2),
        'arm_rad': round(atan2(figure.UPPER_ARM, leg_m), 2),
        'rest_1_s': round(mu * w0, 2), 'limb_1_s2': limb_k, 'limb_1_s': 2.0 * sqrt(limb_k),
        'rest_axial_w': sqrt(k_ax_spec / k_g_spec),
        'rest_lateral_share': round(((figure.HIP_HALF + w_sole) / w_sole) * k_sag_share, 4),
        'rest_sagittal_share': round(k_sag_share, 4),
        'rest_axial_zeta': 1.0, 'rest_lateral_d_share': 0.5, 'rest_sagittal_zeta': sqrt(2.0 / 3.0),
    }


def _rhythm_gait(st, v0, w_l, s_t):
    m, w0 = st['mass'], st['omega']
    rx_a, jx_a = st['pend_ank']
    rz_s, jz_s = st['pend_sole']
    f_brisk = sqrt(m * G_M_S2 * rx_a / (jx_a + 2.0 * drives.armature('left_ankle'))) / pi
    f_quick = sqrt(m * G_M_S2 * rz_s / (
        jz_s + drives.armature('left_ankle_roll') + drives.armature('left_hip_roll'))) / pi
    return {
        'moving': 0.02, 'moving_edge': 0.02, 'gain_on': 0.02, 'gain_edge': 0.03,
        'rest_lo': 0.05, 'rest_hi': 0.30, 'rest_drive': 0.60,
        'edge_m_s': 0.004 * v0, 'edge_rad_s': 0.003 * w_l,
        'still_rad_s': 0.014 * w_l, 'side_m_s': 0.015 * v0,
        'turn_rad_s': 0.007 * w_l, 'turn_edge_rad_s': 0.0014 * w_l,
        'spin_rad_s': 0.011 * w_l, 'spin_still_m_s': 0.018 * v0, 'backing_m_s': 0.018 * v0,
        'turn_dc': 0.40, 'turn_dc_side': 0.12, 'turn_blend': 0.18,
        'turning': 0.008, 'turning_edge': 0.004, 'turn_sway': 0.14,
        'spin_fast_m_s': sqrt(0.075) * v0, 'spin_fast_edge_m_s': 0.018 * v0,
        'spin_drive': 0.035, 'spin_drive_s_m': 1.0 / v0, 'spin_drive_from_m_s': 0.055 * v0,
        'spin_lift': 0.95, 'slip_grip': 0.25, 'slip_grip_edge': 0.02, 'slip_m_s': 0.015 * v0,
        'slip_hz': round(w0 * s_t / pi, 2) / s_t,
        'brisk_hz': round(f_brisk * s_t, 2) / s_t, 'quick_hz': round(f_quick * s_t, 2) / s_t,
        'run_drive': 0.85, 'run_boost': 1.08, 'run_arms': 1.00,
        'back_drive': 0.28, 'back_boost': 1.25, 'side_shift': 0.105,
    }


def _steer_gait(leg_m, v0, w_l, w0, w_task, w_law):
    lag = 0.10
    tr_w, tr_h = drives.TRAVEL['waist'][1], drives.TRAVEL['head'][1]
    w_sh = round(tr_w / (tr_w + tr_h), 2)
    return {
        'slew_rad_s': 0.45 * w_l, 'side_told_m_s': 0.002 * v0, 'turn_told_rad_s': 0.0014 * w_l,
        'along_s': round(2.0 / w_law, 4), 'across_s': 1.0 / w_task, 'told_s': round(1.0 / w_law, 4),
        'stance_turn': 0.75, 'stance_turn_load': 0.30, 'stance_turn_k': 3.0,
        'lag_rad': lag, 'lead_1_s': w_task,
        'align_m_s': 0.018 * v0, 'align_side_m_s': 0.018 * v0,
        'align_speed_m_s': 0.12 * v0, 'align_own_m_s': 0.055 * v0,
        'align_rad': round(atan2(2.0 * figure.HIP_HALF, 0.5 * leg_m + figure.ANKLE_H + figure.BALL), 2),
        'align_1_s': 3.6 * w0, 'slack_rad': lag, 'flip_gain': 0.05, 'flip_drive': 0.02,
        'look_drive': 0.04, 'look_on': 0.1, 'look_back_rad_s': 0.014 * w_l,
        'look_turn_rad_s': 0.008 * w_l, 'look_rad': round(atan2(figure.UPPER_ARM, leg_m), 2),
        'look_1_s': 0.5 * w_task, 'look_back_1_s': 0.05 * w_task,
        'waist_share': w_sh, 'head_share': round(1.0 + lag - w_sh, 2),
        'clear_m': round(2.0 * (figure.SOLE_HALF + figure.PAD_M) + figure.SOLE_HALF / 3.0, 3),
    }


def _loop_gait(v0, w_task, w_eso):
    k_task = round(w_task * w_task, 1)
    k_swing, d_task = 16.0 * k_task, 2.0 * sqrt(k_task)
    mu = round(_KIN_SHARE * _MU_DRY, 2)
    return {
        'mu': mu, 'f_min_n': 0.0, 'slide_m_s': 0.055 * v0,
        'margin_m': figure.SOLE_HALF / 3.0, 'slack': 1e3,
        'height_band_m_s2': float(round(0.5 * mu * G_M_S2)), 'limit_s': 0.5 / w_task,
        'stop_rad': round(radians(drives.BACKLASH_DEG) + ENC_RAD, 2),
        'contact_1_s': d_task, 'held_1_s': 0.5 * w_eso, 'clamp_share': 0.95, 'near': 0.6,
        'turn_1_s2': k_task, 'turn_1_s': d_task, 'posture_1_s2': k_task, 'posture_1_s': d_task,
        'swing_1_s2': k_swing, 'swing_1_s': 2.0 * sqrt(k_swing), 'momentum_1_s': 0.3 * w_task,
        'height_1_s2': k_task, 'height_1_s': d_task, 'turn_w': 3.0, 'momentum_w': 0.3,
        'posture_w': 1.0, 'torque_w': 3.0, 'force_w': 1.0, 'height_w': 1.0, 'fold_w': 0.3,
    }


def _stride_gait(leg_m, v0, w_l, limb_k, run_v, walk_hz, f_sw, fold_rad, mu, w_eso, modes_g, rhythm_g, s_t):
    s_l, t_sw = s_t * s_t, 0.5 / f_sw
    swf = round(t_sw * 0.5 * (walk_hz + f_sw), 2)
    stride_m = round((run_v / (4.0 * walk_hz * sin(pi * swf))) / s_l, 2) * s_l
    pace_walk = 4.0 * stride_m * rhythm_g['run_drive'] * walk_hz
    reach_s = round(0.5 / ((rhythm_g['brisk_hz'] + rhythm_g['slip_hz']) * s_t), 2) * s_t
    return {
        'stride_m': stride_m, 'rock_rad': round(2.0 * radians(drives.TRAVEL['ankle'][1]) / pi, 2),
        'stirs': 0.02, 'swing': round(t_sw * rhythm_g['brisk_hz'], 1), 'swing_fast': swf,
        'pace_m_s': round((pace_walk + rhythm_g['edge_m_s']) / s_t, 2) * s_t,
        'pace_fast_m_s': round((rhythm_g['run_boost'] * pace_walk) / s_t, 2) * s_t,
        'back': 0.5, 'ahead_m_s': round(0.5 * mu * v0 / s_t, 1) * s_t,
        'reach_m': round((0.45 * leg_m) / s_l, 2) * s_l,
        'apart_m': round(2.0 * (figure.SOLE_HALF + figure.PAD_M), 3),
        'soon_s': 2.0 / w_eso, 'rise_u': 0.2, 'fall_u': 0.25, 'arrive_u': 0.9,
        'leads': 0.5, 'land_share': 1.0,
        'reach_m_s': round(((modes_g['axial_m'] - figure.PAD_M) / reach_s) / s_t, 2) * s_t,
        'reach_s': reach_s,
        'flat_1_s2': limb_k,
        'heel_deg': round(degrees(atan2(figure.ANKLE_H, figure.BALL - figure.TOE_RY)), 1),
        'heel_rad_s': round(sqrt(3.0) * w_l * s_t, 0) / s_t,
        'dip_m': figure.PAD_M + figure.CONTACTS[1][3][1], 'edge_rad': 0.03,
        'fold_rad': fold_rad, 'heel_up': 0.5,
    }


def _twist_gait(v0, w_l, w0, walk_v, run_v, walk_hz, s_t):
    s_l = figure.SCALE
    return {
        'walk_m_s': walk_v, 'run_m_s': run_v, 'back_m_s': round(0.125 * v0, 2),
        'strafe_m_s': round(0.18 * v0, 2),
        'yaw_rad_s': round(sqrt(2.0) * radians(drives.TRAVEL['hip_yaw'][1]) * walk_hz, 2),
        'stairs_m_s': round(0.18 * v0, 2), 'ice_m_s': round(0.125 * v0, 2),
        'slope_m_s': round(0.20 * v0, 2), 'ballistic_m_s': round(sqrt(0.075) * v0, 2),
        'circle_m_s': round(0.20 * v0, 2), 'circle_rad_s': round(0.125 * w_l, 2),
        'stop_m': round((walk_v / walk_hz) / s_l, 2) * s_l,
        'back_stop_m': round((figure.HEEL - figure.PAD_M) / s_l, 2) * s_l,
        'side_stop_m': round((figure.HIP_HALF + figure.SOLE_HALF) / s_l, 2) * s_l,
        'stop_spread_m': round(walk_v / (8.0 * walk_hz * s_l), 2) * s_l,
        'walk_hz': walk_hz, 'stand_s': round(round(0.5 * pi / (w0 * s_t), 1) * s_t, 3),
    }


def _gait(st):
    leg_m, mass_kg, w0 = st['leg_m'], st['mass'], st['omega']
    s_l, s_t = figure.SCALE, sqrt(figure.SCALE)
    t0, v0 = sqrt(leg_m / G_M_S2), sqrt(G_M_S2 * leg_m)
    w_l, w_eso = 1.0 / t0, _ESO_WO / s_t
    w_task, w_law = 0.1 * w_eso, 0.1 * _LAW_HZ / s_t
    m_sw, r_sw, j_sw_x = st['swing_leg']
    f_sw = sqrt(m_sw * G_M_S2 * r_sw / (j_sw_x + drives.armature('left_hip'))) / pi
    walk_hz = round(0.5 * (w0 / pi + f_sw) * s_t, 2) / s_t
    walk_v = round(0.40 * sqrt(G_M_S2 * (st['h0'] - figure.HIP_DROP) / s_l), 2) * s_t
    run_v = round(0.50 * v0 / s_t, 2) * s_t
    loop_g = _loop_gait(v0, w_task, w_eso)
    k_task = loop_g['turn_1_s2']
    limb_k, poise_margin = 4.0 * k_task, 0.40 * figure.SOLE_HALF
    modes_g = _modes_gait(st, k_task, limb_k, poise_margin, loop_g['mu'])
    bent = radians(figure.STAND_KNEE)
    fold_rad = round(2.0 * acos(cos(0.5 * bent) - 2.0 * figure.TOE_RY / leg_m) - bent, 2)
    r90 = figure.THIGH * figure.SHANK / hypot(figure.THIGH, figure.SHANK)
    m_eff = mass_kg + 2.0 * drives.armature('left_knee') / (r90 * r90)
    throw_acc = round((2.0 * drives.peak('left_knee') / r90 - st['weight']) / m_eff, 1)
    rhythm_g = _rhythm_gait(st, v0, w_l, s_t)
    leap = {
        'touch_n': round(st['unsprung_kg'] * G_M_S2, 1), 'touch_m': 0.4 * figure.PAD_M,
        'rest_m_s': round(sqrt(0.2 * G_M_S2 * figure.PAD_M), 2),
        'land_drop_m': round(leg_m - _knee_chord(degrees(fold_rad)), 3),
        'land_short_m': round(leg_m - _knee_chord(2.0 * figure.STAND_KNEE), 3),
        **_standing_jump(),
    }
    return {
        'leg_m': leg_m, 'mass_kg': mass_kg, 'leg_kg': st['leg'][0],
        'modes': modes_g, 'rhythm': rhythm_g,
        'steer': _steer_gait(leg_m, v0, w_l, w0, w_task, w_law),
        'poise': {'margin_m': poise_margin, 'capture_s': round(2.0 / (3.0 * w0 * s_t), 2) * s_t,
                  'lift_eps': 0.02, 'gate_m': figure.PAD_M, 'early': 0.3},
        'support': {'tau_s': round(pi / (w_eso * s_t), 2) * s_t, 'eso_rad_s2': round(w_l * w_l, 1),
                    'eso_scale_rad_s2': round(2.0 * w_l * w_l, 1),
                    'sigma_rad': round(atan2(figure.UPPER_ARM + figure.FIST_R, leg_m), 2),
                    'bleed': round(10.0 * w_l, 1)},
        'impedance': {'on_shift': 0.1, 'on_shift_1_s': round((w0 / pi) * s_t, 1) / s_t,
                      'going_s': round(t0 / s_t, 2) * s_t,
                      'ahead': 0.15, 'yaw_1_s': 0.5 * w_eso, 'throw_m_s2': throw_acc,
                      'low_m': round((figure.SHANK - figure.ANKLE_H - figure.PAD_M) / s_l, 2) * s_l,
                      'low_leg': 0.5, 'prop_on': 0.15},
        'loop': loop_g, 'leap': leap,
        'stride': _stride_gait(leg_m, v0, w_l, limb_k, run_v, walk_hz, f_sw, fold_rad,
                               loop_g['mu'], w_eso, modes_g, rhythm_g, s_t),
        'twist': _twist_gait(v0, w_l, w0, walk_v, run_v, walk_hz, s_t),
    }


def _core() -> dict[str, Any]:
    srcs = (mechatronics.robot_sources()
            + (__file__, drives.__file__, figure.__file__, build.__file__, _TAKES_NPZ))

    def build_core():
        st = _subtrees()
        return {'subtrees': st, 'servos': _servos(st), 'gait': _gait(st)}

    return mechatronics.compiled('humanoid_' + mechatronics.ROBOT, srcs, build_core)


def subtrees() -> dict[str, Any]:
    """Composite rigid-body masses, gravity levers and inertias of her kinematic subtrees standing."""
    return dict(_core()['subtrees'])


def servos(overrides=None) -> dict[str, tuple[float, float, float, float]]:
    """{kind: (peak N m, kp N m/rad, kd N m s/rad, armature kg m^2)} from mechanics and stacks."""
    out = dict(_core()['servos'])
    for k, sv in (overrides or {}).items():
        pk = drives.peak(_joint_of(k)) if isnan(sv[0]) else float(sv[0])
        out[k] = (pk, float(sv[1]), float(sv[2]), float(sv[3]))
    return out


def gait(overrides=None) -> dict[str, Any]:
    """Whole-body gait and control tables derived at this robot's leg length and built mass."""
    out = {k: dict(v) if isinstance(v, dict) else v for k, v in _core()['gait'].items()}
    for k, v in (overrides or {}).items():
        out[k] = {**out[k], **v} if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out
