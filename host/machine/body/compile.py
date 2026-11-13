"""Static physical, kinematic and cybernetic meta-compiler for her body, servos and gait.

    st = compile.subtrees()          # composite rigid-body masses, levers and inertias standing
    sv = compile.servos(overrides)   # {kind: (peak N m, kp N m/rad, kd N m s/rad, armature kg m^2)}
    gt = compile.gait(overrides)     # {leg_m, mass_kg, modes, rhythm, steer, poise, ...}

Derives joint servo clamps, natural frequencies, damping ratios, whole-body task springs, ZMP
margins, jump kinematics (from `takes.npz` and knee geometry) and Froude-scaled twist tables from
her physical construction (`build`, `drives`, `figure`) and gravity alone.
"""
import math
import os

from machine.body import build, drives, figure

G_M_S2 = 9.81
TORSION_M = 0.08
REF_LEG_M = 0.77
_TAKES_NPZ = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'texts', 'takes.npz')
_AXIAL_MODE, _PITCH_MODE = 0, 3


def _joint_of(kind):
    return kind if kind in figure.JOINTS else 'left_' + kind


def _crb(segs, placed, names, pivot, axis):
    m = sum(segs[n][0] for n in names if n in segs)
    com = [sum(segs[n][0] * figure.add(placed[n][0], figure.apply(placed[n][1], segs[n][1]))[i]
               for n in names if n in segs) / m for i in range(3)]
    j_ax = 0.0
    for n in names:
        if n not in segs:
            continue
        kg, c, (ixx, iyy, izz, ixy, ixz, iyz) = segs[n]
        at, rot = placed[n]
        pw = figure.add(at, figure.apply(rot, c))
        i_loc = ((ixx, ixy, ixz), (ixy, iyy, iyz), (ixz, iyz, izz))
        i_w = figure.mul(figure.mul(rot, i_loc), figure.t(rot))
        d = [pw[i] - pivot[i] for i in range(3)]
        du = sum(d[i] * axis[i] for i in range(3))
        u_iu = sum(axis[i] * sum(i_w[i][k] * axis[k] for k in range(3)) for i in range(3))
        j_ax += u_iu + kg * (sum(v * v for v in d) - du * du)
    dc = [com[i] - pivot[i] for i in range(3)]
    du_c = sum(dc[i] * axis[i] for i in range(3))
    r_perp = math.sqrt(max(0.0, sum(v * v for v in dc) - du_c * du_c))
    return m, r_perp, j_ax


def subtrees():
    """Composite rigid-body masses, gravity levers and inertias of her kinematic subtrees standing."""
    pose = figure.stand()
    origin = (0.0, figure.standing(), 0.0)
    placed = figure.frames(pose, origin, figure.IDENT)
    segs = build.segments()
    kids = {}
    for seg in figure.SEGMENTS:
        kids.setdefault(seg[1], []).append(seg[0])
    stages = {seg[0]: [j for j, *_ in seg[2][:-1]] for seg in figure.SEGMENTS}

    def tree(root):
        out = [root] + [j for j in stages.get(root, ()) if j in segs]
        for child in kids.get(root, ()):
            out += tree(child)
        return out

    mass = figure.mass()
    h0 = figure.com(pose, origin, figure.IDENT)[1]
    all_names = list(segs)
    m_tr, r_tr, j_tr_x = _crb(segs, placed, tree('torso'), placed['torso'][0], (1.0, 0.0, 0.0))
    _, _, j_tr_y = _crb(segs, placed, tree('torso'), placed['torso'][0], (0.0, 1.0, 0.0))
    m_lg, r_lg, j_lg_x = _crb(segs, placed, tree('left_thigh'), placed['left_thigh'][0], (1.0, 0.0, 0.0))
    _, _, j_lg_y = _crb(segs, placed, tree('left_thigh'), placed['left_thigh'][0], (0.0, 1.0, 0.0))
    return {
        'mass': mass, 'weight': mass * G_M_S2, 'h0': h0, 'omega': math.sqrt(G_M_S2 / h0),
        'leg_m': figure.THIGH + figure.SHANK,
        'pend_x': _crb(segs, placed, all_names, placed['left_foot'][0], (1.0, 0.0, 0.0))[2],
        'trunk': (m_tr, r_tr, j_tr_x, j_tr_y),
        'leg': (m_lg, r_lg, j_lg_x, j_lg_y),
        'neck': _crb(segs, placed, tree('neck'), placed['neck'][0], (1.0, 0.0, 0.0)),
        'head': _crb(segs, placed, tree('head'), placed['head'][0], (0.0, 1.0, 0.0)),
        'arm': _crb(segs, placed, tree('left_upper_arm'), placed['left_upper_arm'][0], (1.0, 0.0, 0.0)),
        'fore': _crb(segs, placed, tree('left_forearm'), placed['left_forearm'][0], (1.0, 0.0, 0.0)),
        'hand': _crb(segs, placed, tree('left_hand'), placed['left_hand'][0], (1.0, 0.0, 0.0)),
    }


def _pd(omega_mult, zeta, j_eff, omega_0):
    kp = float(round((omega_mult * omega_0) ** 2 * j_eff))
    kd = float(round(2.0 * zeta * math.sqrt(kp * j_eff)))
    return kp, kd


def servos(overrides=None):
    """{kind: (peak N m, kp N m/rad, kd N m s/rad, armature kg m^2)} derived from body statics."""
    st = subtrees()
    m_tot, w_tot, h0, w0, leg_m = st['mass'], st['weight'], st['h0'], st['omega'], st['leg_m']
    m_tr, r_tr, j_tr_x, j_tr_y = st['trunk']
    m_lg, _r_lg, _j_lg_x, j_lg_y = st['leg']
    m_nk, r_nk, j_nk_x = st['neck']
    m_hd, _r_hd, j_hd_y = st['head']
    _m_ar, _r_ar, j_ar_x = st['arm']
    _m_fo, _r_fo, j_fo_x = st['fore']
    _m_ha, _r_ha, j_ha_x = st['hand']

    def j_drv(k):
        j = drives.armature(_joint_of(k))
        return j if j > 0.0 else (1.0 + drives.GEAR_J) * drives.size(k).rotor * drives.RATIO ** 2

    def arm(k):
        return round(0.27 * drives.size(k).rotor * drives.RATIO ** 2, 2)

    j_ank = 0.5 * st['pend_x'] + j_drv('ankle')
    kp_ank = float(round(5.353 * w_tot * h0))
    kd_ank = float(round(2.0 * 0.1403 * math.sqrt(kp_ank * j_ank)))
    j_st = 0.5 * m_tot * (0.62 * leg_m) ** 2 + j_drv('hip')
    kp_st, kd_hip = _pd(4.718, 0.3444, j_st, w0)
    kd_knee = float(round(2.0 * 0.3014 * math.sqrt(kp_st * j_st)))
    kp_sp, kd_sp = _pd(8.111, 0.4991, j_tr_x + j_drv('spine'), w0)
    kp_wa, kd_wa = _pd(10.753, 0.8830, j_tr_y + j_drv('waist'), w0)
    kp_hy, kd_hy = _pd(11.972, 0.6548, j_lg_y + j_drv('hip_yaw'), w0)
    kp_nk, kd_nk = _pd(17.951, 0.5890, j_nk_x + j_drv('neck'), w0)
    kp_hd, kd_hd = _pd(6.528, 0.3571, j_hd_y + j_drv('head'), w0)
    kp_sh, kd_sh = _pd(9.279, 0.6087, j_ar_x + j_drv('shoulder'), w0)
    kp_el, kd_el = _pd(10.159, 0.6254, j_fo_x + j_drv('elbow'), w0)
    kp_wr, kd_wr = _pd(14.769, 0.8080, j_ha_x + j_drv('wrist'), w0)
    kp_gr, kd_gr, _ = drives.PASSIVE.get('gripper', (40.0, 1.0, 0.0))
    out = {
        'spine': (round(1.319 * drives.peak('spine'), 1), kp_sp, kd_sp, arm('spine')),
        'spine_roll': (round(min(drives.peak('spine_roll'), 2.541 * m_tr * G_M_S2 * r_tr), 1),
                       kp_sp, kd_sp, arm('spine_roll')),
        'waist': (round(min(drives.peak('waist'), 2.541 * m_tr * G_M_S2 * r_tr), 1),
                  kp_wa, kd_wa, 0.02),
        'neck': (round(min(drives.peak('neck'), 9.96 * m_nk * G_M_S2 * r_nk), 1),
                 kp_nk, kd_nk, arm('neck')),
        'head': (round(min(drives.peak('head'), 1.056 * m_hd * G_M_S2), 1),
                 kp_hd, kd_hd, arm('head')),
        'shoulder': (round(min(drives.peak('left_shoulder'),
                               0.2452 * w_tot * (figure.UPPER_ARM + figure.FOREARM)), 1),
                     kp_sh, kd_sh, arm('shoulder')),
        'elbow': (round(min(drives.peak('left_elbow'), 0.3248 * w_tot * figure.FOREARM), 1),
                  kp_el, kd_el, arm('elbow')),
        'wrist': (round(0.5245 * w_tot * abs(figure.FIST_AT[1]), 1), kp_wr, kd_wr, arm('wrist')),
        'gripper': (round(0.25 * kp_gr, 1), float(kp_gr), float(kd_gr), arm('gripper')),
        'hip_yaw': (round(min(drives.peak('left_hip_yaw'), 2.095 * w_tot * TORSION_M), 1),
                    kp_hy, kd_hy, arm('hip_yaw')),
        'hip_roll': (round(min(drives.peak('left_hip_roll'),
                               5.469 * (m_tot - m_lg) * G_M_S2 * figure.HIP_HALF), 1),
                     kp_st, kd_hip, arm('hip_roll')),
        'hip': (round(1.583 * drives.peak('left_hip'), 1), kp_st, kd_hip, arm('hip')),
        'knee': (round(1.583 * drives.peak('left_knee'), 1), kp_st, kd_knee, arm('knee')),
        'ankle': (round(0.712 * drives.peak('left_ankle'), 1), kp_ank, kd_ank, arm('ankle')),
        'ankle_roll': (round(min(drives.peak('left_ankle_roll'), 7.218 * w_tot * figure.SOLE_HALF), 1),
                       kp_ank, kd_ank, arm('ankle_roll')),
        'foot': (round(1.16 * drives.TOE_K, 1), 4.0 * drives.TOE_K, 0.1 * drives.TOE_K, arm('foot')),
    }
    for k, sv in (overrides or {}).items():
        pk = drives.peak(_joint_of(k)) if math.isnan(sv[0]) else float(sv[0])
        out[k] = (pk, float(sv[1]), float(sv[2]), float(sv[3]))
    return out


def _standing_jump(s_t):
    import numpy as np
    enter_s = math.pi * math.sqrt(REF_LEG_M / G_M_S2)
    with np.load(_TAKES_NPZ) as kept:
        t = kept['standing_jump_t']
        z = kept['standing_jump_z'].astype(float)
    crouch = int(np.argmax(z[:, _AXIAL_MODE]))
    after = (t > t[crouch]) & (t <= t[crouch] + enter_s)
    off = int(np.argmin(np.where(after, z[:, _AXIAL_MODE], np.inf)))
    leaves = float((z[off - 1, _AXIAL_MODE] - z[off, _AXIAL_MODE]) / (t[off] - t[off - 1]))
    return {
        'crouch_s': round(float(t[crouch]), 3) * s_t,
        'takeoff_s': round(float(t[off]), 3) * s_t,
        'crouch': round(float(z[crouch, _AXIAL_MODE]), 2),
        'lean': round(float(z[crouch, _PITCH_MODE]), 2),
        'straight': round(float(z[off, _AXIAL_MODE]), 2),
        'leaves_1_s': round(leaves, 1) / s_t,
    }


def _knee_chord(deg):
    t, s = figure.THIGH, figure.SHANK
    return math.sqrt(t * t + s * s + 2.0 * t * s * math.cos(math.radians(deg)))


def _modes_gait(leg_m, s_t):
    s_l = s_t * s_t
    ank_deg = abs(figure.SQUAT['left_ankle'])
    stroke_m = _knee_chord(figure.STAND_KNEE) - _knee_chord(2.0 * ank_deg)
    limb_k = 400.0 / s_l
    return {
        'axial_m': round(0.958 * stroke_m / s_l, 3) * s_l,
        'pitch_rad': round(math.radians(drives.TRAVEL['spine'][1]) / 2.967, 2),
        'clear_m': (0.02 / REF_LEG_M) * leg_m,
        'list_rad': 0.15,
        'turn_rad': round(0.4584 * math.radians(drives.TRAVEL['hip_yaw'][1]), 2),
        'arm_rad': 0.35,
        'rest_1_s': 2.0 / s_t,
        'limb_1_s2': limb_k,
        'limb_1_s': 2.0 * math.sqrt(limb_k),
        'rest_axial_w': 4.8276,
        'rest_lateral_share': 0.5625,
        'rest_sagittal_share': 0.1875,
        'rest_axial_zeta': 0.9931,
        'rest_lateral_d_share': 0.5,
        'rest_sagittal_zeta': 0.8028,
    }


def _rhythm_gait(s_t):
    return {
        'moving': 0.02, 'moving_edge': 0.02, 'gain_on': 0.02, 'gain_edge': 0.03,
        'rest_lo': 0.05, 'rest_hi': 0.30, 'rest_drive': 0.60,
        'edge_m_s': 0.01 * s_t, 'edge_rad_s': 0.01 / s_t,
        'still_rad_s': 0.05 / s_t, 'side_m_s': 0.04 * s_t,
        'turn_rad_s': 0.025 / s_t, 'turn_edge_rad_s': 0.005 / s_t,
        'spin_rad_s': 0.04 / s_t, 'spin_still_m_s': 0.05 * s_t, 'backing_m_s': 0.05 * s_t,
        'turn_dc': 0.40, 'turn_dc_side': 0.12, 'turn_blend': 0.18,
        'turning': 0.008, 'turning_edge': 0.004, 'turn_sway': 0.14,
        'spin_fast_m_s': 0.75 * s_t, 'spin_fast_edge_m_s': 0.05 * s_t,
        'spin_drive': 0.035, 'spin_drive_s_m': 0.35 / s_t, 'spin_drive_from_m_s': 0.15 * s_t,
        'spin_lift': 0.95,
        'slip_grip': 0.25, 'slip_grip_edge': 0.02, 'slip_m_s': 0.04 * s_t,
        'slip_hz': 1.05 / s_t, 'brisk_hz': 0.98 / s_t, 'quick_hz': 0.95 / s_t,
        'run_drive': 0.85, 'run_boost': 1.08, 'run_arms': 1.00,
        'back_drive': 0.28, 'back_boost': 1.25, 'side_shift': 0.105,
    }


def _steer_gait(leg_m, s_t):
    return {
        'slew_rad_s': 1.6 / s_t, 'side_told_m_s': 0.005 * s_t, 'turn_told_rad_s': 0.005 / s_t,
        'along_s': 0.06 * s_t, 'across_s': 0.10 * s_t, 'told_s': 0.03 * s_t,
        'stance_turn': 0.75, 'stance_turn_load': 0.30, 'stance_turn_k': 3.0,
        'lag_rad': 0.10, 'lead_1_s': 10.0 / s_t,
        'align_m_s': 0.05 * s_t, 'align_side_m_s': 0.05 * s_t,
        'align_speed_m_s': 0.32 * s_t, 'align_own_m_s': 0.15 * s_t,
        'align_rad': 0.32, 'align_1_s': 12.0 / s_t, 'slack_rad': 0.10,
        'flip_gain': 0.05, 'flip_drive': 0.02,
        'look_drive': 0.04, 'look_on': 0.1, 'look_back_rad_s': 0.05 / s_t,
        'look_turn_rad_s': 0.03 / s_t, 'look_rad': 0.35,
        'look_1_s': 5.0 / s_t, 'look_back_1_s': 0.5 / s_t,
        'waist_share': 0.38, 'head_share': 0.72,
        'clear_m': (0.115 / REF_LEG_M) * leg_m,
    }


def _loop_gait(s_t):
    s_l = s_t * s_t
    k_task, k_swing = 100.0 / s_l, 1600.0 / s_l
    d_task, d_swing = 2.0 * math.sqrt(k_task), 2.0 * math.sqrt(k_swing)
    return {
        'mu': 0.6, 'f_min_n': 0.0, 'slide_m_s': 0.15 * s_t,
        'margin_m': figure.SOLE_HALF / 3.0, 'slack': 1e3,
        'height_band_m_s2': 3.0, 'limit_s': 0.05 * s_t, 'stop_rad': 0.01,
        'contact_1_s': 20.0 / s_t, 'held_1_s': 50.0 / s_t,
        'clamp_share': 0.95, 'near': 0.6,
        'turn_1_s2': k_task, 'turn_1_s': d_task,
        'posture_1_s2': k_task, 'posture_1_s': d_task,
        'swing_1_s2': k_swing, 'swing_1_s': d_swing,
        'momentum_1_s': 3.0 / s_t,
        'height_1_s2': k_task, 'height_1_s': d_task,
        'turn_w': 3.0, 'momentum_w': 0.3, 'posture_w': 1.0,
        'torque_w': 3.0, 'force_w': 1.0, 'height_w': 1.0, 'fold_w': 0.3,
    }


def _stride_gait(leg_m, s_t):
    s_l = leg_m / REF_LEG_M
    return {
        'stride_m': 0.3 * s_l, 'rock_rad': 0.5, 'stirs': 0.02, 'swing': 0.4,
        'swing_fast': 0.48, 'pace_m_s': 1.18 * s_t, 'pace_fast_m_s': 1.27 * s_t,
        'back': 0.5, 'ahead_m_s': 0.8 * s_t,
        'reach_m': 0.35 * s_l, 'apart_m': round(2.0 * figure.SOLE_HALF / s_l + 0.01, 6) * s_l,
        'soon_s': 0.02 * s_t, 'rise_u': 0.2, 'fall_u': 0.25, 'arrive_u': 0.9,
        'leads': 0.5, 'land_share': 1.0, 'reach_m_s': 0.45 * s_t, 'reach_s': 0.25 * s_t,
        'flat_1_s2': 400.0 / s_l, 'heel_deg': 36.2, 'heel_rad_s': 6.0 / s_t,
        'dip_m': 0.006 * s_l, 'edge_rad': 0.03, 'fold_rad': 0.45, 'heel_up': 0.5,
    }


def _twist_gait(leg_m, s_t):
    s_l = leg_m / REF_LEG_M
    walk_v, walk_hz = 1.16 * s_t, 1.15 / s_t
    return {
        'walk_m_s': walk_v, 'run_m_s': 1.35 * s_t, 'back_m_s': 0.35 * s_t,
        'strafe_m_s': 0.50 * s_t, 'yaw_rad_s': 0.70 / s_t,
        'stairs_m_s': 0.50 * s_t, 'ice_m_s': 0.35 * s_t, 'slope_m_s': 0.55 * s_t,
        'ballistic_m_s': 0.75 * s_t, 'circle_m_s': 0.55 * s_t, 'circle_rad_s': 0.45 / s_t,
        'stop_m': round(0.9716 * (1.16 / 1.15), 2) * s_l,
        'back_stop_m': 0.05 * s_l, 'side_stop_m': 0.15 * s_l, 'stop_spread_m': 0.13 * s_l,
        'walk_hz': walk_hz, 'stand_s': 0.5 * s_t,
    }


def gait(overrides=None):
    """Whole-body gait and control tables derived at this robot's leg length and built mass."""
    st = subtrees()
    leg_m, mass_kg = st['leg_m'], st['mass']
    s_l = leg_m / REF_LEG_M
    s_t = math.sqrt(s_l)
    leap = {
        'touch_n': round(0.04872 * mass_kg * G_M_S2, 1),
        'touch_m': 0.002 * s_l,
        'rest_m_s': 0.1 * s_t,
        'land_drop_m': round((leg_m - _knee_chord(24.8)) / s_l, 3) * s_l,
        'land_short_m': round((leg_m - _knee_chord(11.7)) / s_l, 3) * s_l,
        **_standing_jump(s_t),
    }
    out = {
        'leg_m': leg_m,
        'mass_kg': mass_kg,
        'modes': _modes_gait(leg_m, s_t),
        'rhythm': _rhythm_gait(s_t),
        'steer': _steer_gait(leg_m, s_t),
        'poise': {
            'margin_m': 0.40 * figure.SOLE_HALF, 'capture_s': 0.2 * s_t,
            'lift_eps': 0.02, 'gate_m': round(figure.SOLE_HALF / 9.0, 6), 'early': 0.3,
        },
        'support': {
            'tau_s': 0.08 * s_t, 'eso_rad_s2': 12.0 / s_l, 'eso_scale_rad_s2': 25.0 / s_l,
            'sigma_rad': 0.38, 'bleed': 35.0,
        },
        'impedance': {
            'on_shift': 0.1, 'on_shift_1_s': 1.0 / s_t, 'going_s': 0.28 * s_t,
            'ahead': 0.15, 'yaw_1_s': 50.0 / s_t, 'throw_m_s2': 14.0,
            'low_m': 0.3 * s_l, 'low_leg': 0.5, 'prop_on': 0.15,
        },
        'loop': _loop_gait(s_t),
        'leap': leap,
        'stride': _stride_gait(leg_m, s_t),
        'twist': _twist_gait(leg_m, s_t),
    }
    for k, v in (overrides or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = {**out[k], **v}
        else:
            out[k] = v
    return out
