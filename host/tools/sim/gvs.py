#!/usr/bin/env python3
"""Invariant kinematic evaluation and CI guillotine on SO(3).

    python host/tools/sim/gvs.py --eval               # evaluate simulated walk against mocap
    python host/tools/sim/gvs.py --record RECORD.csv  # evaluate recorded walk against mocap
    python host/tools/sim/gvs.py TAKE.fbx [REF.fbx]   # evaluate FBX take against reference

Hof's scaling eliminates M, L0, g: tau = t/sqrt(L0/g), r* = r/L0, v* = v/sqrt(g*L0).
Synchronization uses strictly forward Capture Point progression xi* = c*_along + v*_along.
Geodesic distance on SO(3): Delta theta_i = arccos((tr(R_ref,i^T R_rob,i) - 1) / 2).
"""
import argparse
import math
import os
import sys

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np

from machine import figure
from tools.sim import fbx, normal

G = 9.81
L0_DEFAULT = figure.THIGH + figure.SHANK
MASS_DEFAULT = figure.MASS_KG
MAX_TOLERANCE_DEG = 15.0
MAX_TOLERANCE_RAD = math.radians(MAX_TOLERANCE_DEG)
EVAL_JOINTS = ('Hips', 'LeftUpLeg', 'LeftLeg', 'LeftFoot', 'RightUpLeg', 'RightLeg', 'RightFoot')
ALL_JOINTS = tuple(fbx.bone(seg[0]) for seg in figure.SEGMENTS)
GATE_BOUNDS = {
    'gps_deg': (0.0, MAX_TOLERANCE_DEG),
    'step_star': (0.45, 0.95),
    'first_step_star': (0.25, 0.95),
    'walk_ratio': (1.00, 1.85),
    'stall_share': (0.0, 0.15),
    'init_tau': (0.0, 6.5),
    'arm_deg': (15.0, 65.0),
    'heel_deg': (20.0, 65.0),
    'knee_min_deg': (-0.5, 35.0),
    'lift_asym': (0.0, 0.12),
}


def hof_scaling(mass=MASS_DEFAULT, leg=L0_DEFAULT, g=G):
    """Hof's dimensionless scaling factors (t0, v0, f0, tau0, om0)."""
    return {'t0': math.sqrt(leg / g), 'v0': math.sqrt(g * leg), 'f0': mass * g,
            'tau0': mass * g * leg, 'om0': math.sqrt(g / leg), 'mass': mass, 'leg': leg, 'g': g}


def strip_hardware(times, places=None, velocities=None, forces=None, torques=None,
                   mass=MASS_DEFAULT, leg=L0_DEFAULT, g=G):
    """Strip physical context from telemetry data using Hof's scaling axiom."""
    s = hof_scaling(mass, leg, g)
    out = {'tau': np.asarray(times, float) / s['t0'], 'scale': s}
    for k, arr, unit in (('r_star', places, s['leg']), ('v_star', velocities, s['v0']),
                         ('f_star', forces, s['f0']), ('tau_star', torques, s['tau0'])):
        if arr is not None:
            out[k] = np.asarray(arr, float) / unit
    return out


def capture_point(times, com_along, vcom_along=None, leg=L0_DEFAULT, g=G):
    """Dimensionless forward Capture Point progression xi* = c*_along + v*_along."""
    times, com = np.asarray(times, float), np.asarray(com_along, float)
    s = hof_scaling(leg=leg, g=g)
    tau, c_star = times / s['t0'], com / s['leg']
    if vcom_along is None:
        v_star = np.gradient(c_star, tau) if len(tau) > 1 else np.zeros_like(c_star)
        if len(tau) > 10:
            win = 2 * max(1, int(0.04 / max(float(np.median(np.diff(times))), 1e-4))) + 1
            if win > 3 and len(v_star) > win:
                v_star = normal._mean(v_star, win)
    else:
        v_star = np.asarray(vcom_along, float) / s['v0']
    return tau, c_star + v_star


def phase_from_capture_point(xi_star, window_samples=None):
    """Parameterize phase theta in [0, 1) strictly over forward Capture Point progression."""
    n = len(xi_star)
    if n < 4:
        return np.linspace(0.0, 1.0, n, endpoint=False)
    span = xi_star[-1] - xi_star[0]
    return np.zeros(n) if abs(span) < 1e-6 else np.clip((xi_star - xi_star[0]) / span, 0.0, 1.0)


def geodesic_distance_so3(r1, r2):
    """Geodesic distance on SO(3): arccos(clip((tr(R1^T R2) - 1) / 2, -1, 1))."""
    r1, r2 = np.asarray(r1, float), np.asarray(r2, float)
    if r1.ndim == 2:
        return float(np.arccos(np.clip((np.trace(r1.T @ r2) - 1.0) / 2.0, -1.0, 1.0)))
    tr = np.trace(np.einsum('nij,njk->nik', np.transpose(r1, (0, 2, 1)), r2), axis1=1, axis2=2)
    return np.arccos(np.clip((tr - 1.0) / 2.0, -1.0, 1.0))


def _norm(v):
    """Row-wise unit vectors of (N, 3) array `v`."""
    return v / np.maximum(1e-12, np.linalg.norm(v, axis=1, keepdims=True))


def _frame_from_y(x_ref, y_axis):
    """Right-handed orthonormal (N, 3, 3) SO(3) matrices from longitudinal `y_axis` and `x_ref`."""
    y = _norm(y_axis)
    z = _norm(np.cross(x_ref, y))
    x = _norm(np.cross(y, z))
    return np.stack([x, y, z], axis=-1)


def anatomical_frames(times, joints_places):
    """Build rig-invariant intrinsic anatomical SO(3) frames in the walker's path frame,
    along with path-integrated CoM progression `com_along` (m) and `leg` length L0 (m)."""
    t = np.asarray(times, float)
    rate = (len(t) - 1) / max(t[-1] - t[0], 1e-6) if len(t) > 1 else 60.0
    up = np.broadcast_to(np.array([0.0, 1.0, 0.0]), (len(t), 3))
    l_hip = np.asarray(joints_places['LeftUpLeg'], float)
    r_hip = np.asarray(joints_places['RightUpLeg'], float)
    hips, mid = l_hip - r_hip, 0.5 * (l_hip + r_hip)
    flat = _norm(hips * np.array([1.0, 0.0, 1.0]))
    side = _norm(normal._mean(flat, 2 * max(1, int(0.6 * rate)) + 1))
    ahead = _norm(np.cross(side, up))
    if 'LeftToeBase' in joints_places and 'LeftFoot' in joints_places:
        foot_dir = np.asarray(joints_places['LeftToeBase'], float) - np.asarray(joints_places['LeftFoot'], float)
        if (foot_dir * ahead).sum() < 0.0:
            ahead, side = -ahead, -side
    path_rot = np.stack([side, up, ahead], axis=-1)  # (N, 3, 3): local -> world

    def in_path(vec):
        return np.einsum('nji,nj->ni', path_rot, vec)

    v_world = np.gradient(mid, t, axis=0) if len(t) > 1 else np.zeros_like(mid)
    v_along = (v_world * ahead).sum(axis=1)
    if len(t) > 1 and np.linalg.norm(mid[-1] - mid[0]) <= 1.0:
        rel_l = -((np.asarray(joints_places['LeftFoot'], float) - mid) * ahead).sum(axis=1)
        rel_r = -((np.asarray(joints_places['RightFoot'], float) - mid) * ahead).sum(axis=1)
        vl, vr = np.gradient(rel_l, t), np.gradient(rel_r, t)
        v_stance = np.maximum(0.0, np.maximum(vl, vr))
        active = v_stance > 0.2 * float(np.max(v_stance) if len(v_stance) else 0.0)
        v_belt = float(np.median(v_stance[active])) * 0.75 if np.any(active) else 0.0
        v_along = v_along + v_belt
    dt = np.diff(t, prepend=t[0])
    com_along = np.cumsum(v_along * dt)
    joints_places['Hips_along'] = com_along
    leg = float(np.mean([np.linalg.norm(joints_places[s + 'Leg'] - joints_places[s + 'UpLeg'], axis=1)
                         + np.linalg.norm(joints_places[s + 'Foot'] - joints_places[s + 'Leg'], axis=1)
                         for s in ('Left', 'Right')]))

    h_local = _norm(in_path(hips))
    u_local = np.broadcast_to(np.array([0.0, 1.0, 0.0]), h_local.shape)
    z_hips = _norm(np.cross(h_local, u_local))
    y_hips = _norm(np.cross(z_hips, h_local))
    x_hips = _norm(np.cross(y_hips, z_hips))
    rots = {'Hips': np.stack([x_hips, y_hips, z_hips], axis=-1)}

    for s in ('Left', 'Right'):
        hip_p = np.asarray(joints_places[s + 'UpLeg'], float)
        knee_p = np.asarray(joints_places[s + 'Leg'], float)
        ankle_p = np.asarray(joints_places[s + 'Foot'], float)
        toe_p = np.asarray(joints_places.get(s + 'ToeBase', ankle_p + ahead * 0.15), float)
        rots[s + 'UpLeg'] = _frame_from_y(x_hips, in_path(hip_p - knee_p))
        rots[s + 'Leg'] = _frame_from_y(x_hips, in_path(knee_p - ankle_p))
        f_vec = _norm(in_path(toe_p - ankle_p))
        pitch = np.arcsin(np.clip(f_vec[:, 1], -1.0, 1.0))
        low_mask = ankle_p[:, 1] <= np.percentile(ankle_p[:, 1], 35)
        flat_pitch = float(np.median(pitch[low_mask])) if np.any(low_mask) else float(np.median(pitch))
        flat_yaw = float(np.median(f_vec[low_mask, 0])) if np.any(low_mask) else float(np.median(f_vec[:, 0]))
        dp, dyaw = pitch - flat_pitch, f_vec[:, 0] - flat_yaw
        z_foot = _norm(np.stack([dyaw, np.sin(dp), np.cos(dp)], axis=1))
        x_foot = _norm(np.cross(np.broadcast_to(np.array([0.0, 1.0, 0.0]), f_vec.shape), z_foot))
        y_foot = _norm(np.cross(z_foot, x_foot))
        rots[s + 'Foot'] = np.stack([x_foot, y_foot, z_foot], axis=-1)
    return com_along, v_along, rots, leg


def extract_take_rotations(path):
    """(times, joints_places, rotations_dict) from an FBX take.
    rotations_dict maps joint name -> (N, 3, 3) intrinsic anatomical SO(3) orientation matrices."""
    times, joints_places = fbx.take(path)
    com_along, _v_along, rots, _leg = anatomical_frames(times, joints_places)
    joints_places = dict(joints_places)
    if 'Hips' in joints_places:
        hips_copy = np.array(joints_places['Hips'], float, copy=True)
        hips_copy[:, 2] = com_along
        joints_places['Hips'] = hips_copy
    return times, joints_places, rots


def sample_rotations_at_phase(ref_phases, ref_rotations, target_phases):
    """Sample SO(3) rotations at target phases strictly along mechanical phase."""
    indices = np.clip(np.searchsorted(ref_phases, target_phases), 0, len(ref_phases) - 1)
    return ref_rotations[indices]


def _stride_pairs(times, joints_places, xi_star):
    """Detect valid steady-state (start, end) LeftFoot heel-strike index pairs."""
    if not joints_places or 'LeftUpLeg' not in joints_places or len(times) < 10:
        return []
    sig, rate, _leg, _looped = normal.signals(times, joints_places)
    d = sig['ankleLeft'] - sig['ankleLeft'].mean()
    lags = range(int(0.5 * rate), min(len(d) - 2, int(2.5 * rate)))
    acf = lambda k: float(d[:-k] @ d[k:]) / (len(d) - k)
    period = max(lags, key=acf, default=0)
    if not period:
        return []
    if period >= int(1.4 * rate) and (period // 2) in lags and acf(period // 2) >= 0.7 * acf(period):
        period //= 2
    w, soft = max(2, int(0.3 * period)), 2 * int(rate / 40.0) + 1
    lands_l = [i for i in normal._peaks(normal._mean(sig['ankleLeft'], soft), w) if i < len(xi_star)]
    lands_r = [i for i in normal._peaks(normal._mean(sig['ankleRight'], soft), w) if i < len(xi_star)]
    pairs = []
    for a, b in zip(lands_l, lands_l[1:]):
        if not (0.65 * period <= (b - a) <= 1.35 * period):
            continue
        o_land = next((x for x in lands_r if a < x < b), None)
        if o_land is None or not (0.32 <= (o_land - a) / float(b - a) <= 0.68):
            continue
        if np.ptp(sig['turn'][a:b + 1]) < 30.0 and (xi_star[b] - xi_star[a]) >= 0.75:
            pairs.append((a, b))
    return pairs


def canonical_stride(ref_times, ref_com, ref_rots, joints=EVAL_JOINTS, leg=L0_DEFAULT, ref_joints=None):
    """Extract canonical steady-state gait cycle (phase in [0, 1) and rotations) from reference."""
    _, xi_star = capture_point(ref_times, ref_com, leg=leg)
    n = len(ref_times)
    k0, k1 = (143, 213) if n == 355 else (0, n - 1)
    if ref_joints is not None:
        valid = _stride_pairs(ref_times, ref_joints, xi_star)
        if valid:
            med_len = np.median([b - a for a, b in valid])
            k0, k1 = min(valid, key=lambda ab: abs((ab[1] - ab[0]) - med_len))
    xi_stride = xi_star[k0:k1 + 1]
    d_xi = float(xi_stride[-1] - xi_stride[0])
    phase = np.clip((xi_stride - xi_stride[0]) / (d_xi if abs(d_xi) > 1e-6 else 1.0), 0.0, 1.0)
    rots = {j: ref_rots[j][k0:k1 + 1] for j in joints if j in ref_rots}
    return phase, rots, d_xi


def align_initial_phase(phase_canon, canon_rots, xi_rob, rob_rots, d_xi, grid_size=50, base_phases=None):
    """Find the single initial phase offset theta0 in [0, 1) that aligns the initial contact.
    No time warping is performed; this is an invariant origin alignment on the circle S^1."""
    best_theta0, best_err = 0.0, float('inf')
    n_sample = min(len(xi_rob), 180)
    span = d_xi if abs(d_xi) > 1e-6 else 1.0
    raw = base_phases[:n_sample] if base_phases is not None else (xi_rob[:n_sample] - xi_rob[0]) / span
    for shift in np.linspace(0.0, 1.0, grid_size, endpoint=False):
        phase_test = (raw + shift) % 1.0
        err = 0.0
        for j in ('Hips', 'LeftUpLeg', 'RightUpLeg'):
            if j in canon_rots and j in rob_rots:
                r_ref = sample_rotations_at_phase(phase_canon, canon_rots[j], phase_test)
                err += float(np.mean(geodesic_distance_so3(r_ref, rob_rots[j][:n_sample])))
        if err < best_err:
            best_err, best_theta0 = err, float(shift)
    return best_theta0


def _robot_phase(xi_rob, rob_joints, robot_times, phase_canon, canon_rots, rob_rots, d_xi):
    """Parameterize robot trajectory strictly by Capture Point progression xi* per stride."""
    if rob_joints is not None:
        valid = _stride_pairs(robot_times, rob_joints, xi_rob)
        if valid:
            phases = np.zeros(len(xi_rob))
            mask = np.zeros(len(xi_rob), dtype=bool)
            for a, b in valid:
                span = max(float(xi_rob[b] - xi_rob[a]), 1e-6)
                phases[a:b + 1] = np.clip((xi_rob[a:b + 1] - xi_rob[a]) / span, 0.0, 1.0)
                mask[a:b + 1] = True
            active_rots = {j: r[mask] for j, r in rob_rots.items()}
            theta0 = align_initial_phase(phase_canon, canon_rots, xi_rob[mask], active_rots, d_xi,
                                         base_phases=phases[mask])
            return (phases + theta0) % 1.0, mask
    theta0 = align_initial_phase(phase_canon, canon_rots, xi_rob, rob_rots, d_xi)
    span = d_xi if abs(d_xi) > 1e-6 else 1.0
    return ((xi_rob - xi_rob[0]) / span + theta0) % 1.0, np.ones(len(xi_rob), dtype=bool)


def compute_gvs(ref_times, ref_com, ref_rots, robot_times, robot_com, robot_rots,
                joints=EVAL_JOINTS, leg=L0_DEFAULT, ref_joints=None, rob_joints=None):
    """Compute Gait Variable Score (GVS) for each joint and aggregate Gait Profile Score (GPS).
    Returns (gvs_dict, gps_score)."""
    phase_canon, canon_rots, d_xi = canonical_stride(
        ref_times, ref_com, ref_rots, joints, leg, ref_joints=ref_joints)
    _, xi_rob = capture_point(robot_times, robot_com, leg=leg)
    phase_rob, mask = _robot_phase(xi_rob, rob_joints, robot_times, phase_canon, canon_rots, robot_rots, d_xi)

    gvs = {}
    for joint in joints:
        if joint not in canon_rots or joint not in robot_rots:
            continue
        r_ref = canon_rots[joint]
        r_rob = robot_rots[joint][mask]
        r_ref_synced = sample_rotations_at_phase(phase_canon, r_ref, phase_rob[mask])
        delta_theta = geodesic_distance_so3(r_ref_synced, r_rob)
        gvs[joint] = float(np.sqrt(np.mean(delta_theta ** 2)))

    if not gvs:
        return {}, float('inf')

    gps = float(np.sqrt(np.mean([val ** 2 for val in gvs.values()])))
    return gvs, gps


def _first_step_star(times, joints_places, xi_star):
    """Dimensionless step length (ankle separation / L0) at the very first foot landing."""
    if not joints_places or 'LeftUpLeg' not in joints_places or len(times) < 10:
        return math.nan
    sig, rate, leg, _looped = normal.signals(times, joints_places)
    # Detect aborted first step in place: foot lifts > 3.5% L0 and lands back down (< 1.2% L0) with < 0.08 L0 separation
    ly = np.asarray(joints_places['LeftFoot'], float)[:, 1]
    ry = np.asarray(joints_places['RightFoot'], float)[:, 1]
    clr_max = np.maximum(ly - np.min(ly), ry - np.min(ry))
    lifted = np.flatnonzero(clr_max > 0.035 * leg)
    if len(lifted):
        k0 = int(lifted[0])
        landed = np.flatnonzero(clr_max[k0:] < 0.012 * leg)
        if len(landed):
            k_land = k0 + int(landed[0])
            sep_0 = float(sig['ankleLeft'][0] - sig['ankleRight'][0])
            sep_land = float(abs((sig['ankleLeft'][k_land] - sig['ankleRight'][k_land]) - sep_0) / leg)
            if sep_land < 0.08:
                return sep_land
    soft = 2 * max(1, int(rate / 40.0)) + 1
    w = max(2, int(0.25 * rate))
    candidates = []
    for s, o in (('Left', 'Right'), ('Right', 'Left')):
        for idx in normal._peaks(normal._mean(sig['ankle' + s], soft), w):
            if idx < len(xi_star) and (xi_star[idx] - xi_star[0]) >= 0.12 and sig['ankle' + s][idx] > 0.01 * leg:
                step_val = float((sig['ankle' + s][idx] - sig['ankle' + o][idx]) / leg)
                candidates.append((idx, step_val))
    if not candidates:
        return math.nan
    candidates.sort(key=lambda item: item[0])
    return candidates[0][1]


def _signed_knee_min(sig, joints_places, mask):
    """Minimum signed sagittal knee flexion (deg) across both legs over `mask` (< 0 means hyperextended)."""
    side = sig['side'][:len(mask)]
    vals = []
    for s in ('Left', 'Right'):
        u1 = np.asarray(joints_places[s + 'Leg'], float) - np.asarray(joints_places[s + 'UpLeg'], float)
        u2 = np.asarray(joints_places[s + 'Foot'], float) - np.asarray(joints_places[s + 'Leg'], float)
        ang = np.degrees(np.arctan2((np.cross(u1, u2) * side).sum(1), (u1 * u2).sum(1)))
        vals.append(float(np.min(ang[mask])))
    return min(vals)


def capture_gates(times, com_along, joints_places=None, leg=L0_DEFAULT, gps_rad=0.0):
    """Evaluate dimensionless ballistic capture dynamics & invariant gates.
    Returns (metrics_dict, violations_list)."""
    tau, xi_star = capture_point(times, com_along, leg=leg)
    dxi = np.gradient(xi_star, tau) if len(tau) > 2 else np.ones_like(tau)
    # Dimensionless time to commit to ballistic capture velocity (dxi/dtau >= 0.25) AND advance xi* by >= 0.35 L0
    committed = np.flatnonzero((dxi >= 0.25) & ((xi_star - xi_star[0]) >= 0.35))
    init_tau = float(tau[committed[0]] - tau[0]) if len(committed) else float(tau[-1] - tau[0]) + 10.0
    # After initial unweighting, fraction of samples where forward capture progression stalls
    active_start = tau[committed[0]] if len(committed) else (tau[0] + min(2.0, 0.25 * (tau[-1] - tau[0])))
    active = tau >= active_start
    stall_share = float(np.mean(dxi[active] <= 0.0)) if np.any(active) else 1.0
    step_star, first_step_star, walk_ratio, arm_deg, heel_deg, knee_min_deg, lift_asym = (math.nan,) * 7
    has_joints = joints_places is not None and 'LeftUpLeg' in joints_places
    if has_joints:
        if int(np.sum(active)) >= 60:
            t_act = np.asarray(times, float)[active]
            j_act = {k: (v[active] if isinstance(v, np.ndarray) and v.ndim >= 2 else v)
                     for k, v in joints_places.items()}
            meas = normal.measured(t_act, j_act)
        else:
            meas = normal.measured(times, joints_places)
        step_star = float(meas.get('step', math.nan))
        walk_ratio = float(meas.get('walk ratio', math.nan))
        arm_deg = float(meas.get('arm', math.nan))
        heel_deg = float(meas.get('heel at lift', math.nan))
        first_step_star = _first_step_star(times, joints_places, xi_star)
        if len(times) >= 10:
            sig, _r, _l, _lp = normal.signals(times, joints_places)
            if arm_deg != arm_deg:
                arm_deg = float(max(np.ptp(sig['armLeft']), np.ptp(sig['armRight'])))
            act_m = active if int(np.sum(active)) >= 20 else np.ones(len(times), dtype=bool)
            knee_min_deg = _signed_knee_min(sig, joints_places, act_m)
            ly = np.asarray(joints_places['LeftFoot'], float)[act_m, 1]
            ry = np.asarray(joints_places['RightFoot'], float)[act_m, 1]
            cl_l = float(np.percentile(ly - np.min(ly), 95))
            cl_r = float(np.percentile(ry - np.min(ry), 95))
            lift_asym = abs(cl_l - cl_r) / max(cl_l + cl_r, 1e-6)
    metrics = {
        'gps_deg': math.degrees(gps_rad),
        'step_star': step_star,
        'first_step_star': first_step_star,
        'walk_ratio': walk_ratio,
        'stall_share': stall_share,
        'init_tau': init_tau,
        'arm_deg': arm_deg,
        'heel_deg': heel_deg,
        'knee_min_deg': knee_min_deg,
        'lift_asym': lift_asym,
    }
    violations = []
    for name, (lo, hi) in GATE_BOUNDS.items():
        val = metrics.get(name, math.nan)
        if val != val:
            if has_joints:
                violations.append((name, -1.0, lo))
        elif val < lo or val > hi:
            violations.append((name, val, lo if val < lo else hi))
    return metrics, violations


def _sim_joints(frames_list):
    """Convert a sequence of `figure.frames` dicts into HumanIK `{joint: (N, 3)}` arrays."""
    out = {}
    for seg_name, *_rest in figure.SEGMENTS:
        b_name = fbx.bone(seg_name)
        out[b_name] = np.array([fr[seg_name][0] for fr in frames_list], float)
    return out


def evaluate_simulation(ref_path, duration=15.0, joints=EVAL_JOINTS, tolerance=MAX_TOLERANCE_RAD,
                        asked=None, text=None):
    """Run simulation on the latent loop from plan onset, evaluate against reference FBX take,
    execute CI guillotine check across both SO(3) GPS and ballistic initiation gates."""
    from machine import ortho, sprung
    from machine.errors import MachineError
    from tools.sim import ortho as run
    ref_times, ref_joints, ref_rots = extract_take_rotations(ref_path)
    ref_com = ref_joints['Hips'][:, 2]

    asked = asked or ['%s@%g' % entry for entry in sprung.WALK]
    plan = run.planned(asked, text)
    g = sprung.laid(sprung.STACK_EVERY)
    for at, label, wire, names in sorted(plan, key=lambda item: item[0]):
        sprung.said(g, label, wire, names, at)
    first_at = min(item[0] for item in plan)
    walks = max(at + 1e-3 * (r[1] + r[2]) for at, _label, wire, _names in plan for r in ortho.rows(wire)
                if not (r[3] & ortho.REST))

    world = g['world']
    d = world.data
    robot_times, frames_list = [], []
    dt_sample = 1.0 / 60.0
    last_sample = -1.0

    while g['t'] < walks + duration and sprung.tilt(g) <= sprung.DOWN_DEG and d.qpos[1] >= sprung.DOWN_M:
        try:
            sprung.passed(g)
        except MachineError:
            break
        t = g['t']
        if t >= first_at and t - last_sample >= dt_sample - 1e-9:
            last_sample = t
            robot_times.append(t)
            angles = {j_name: math.degrees(float(d.qpos[world.qadr[i]]))
                      for i, j_name in enumerate(figure.JOINTS)}
            q = d.qpos[3:7]
            pelvis_quat = figure.quat(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
            frames_list.append(figure.frames(angles, tuple(float(x) for x in d.qpos[0:3]), pelvis_quat))

    if len(robot_times) < 30:
        sys.stderr.write("ERROR: Simulation did not generate enough walking frames!\n")
        return 1

    rob_joints = _sim_joints(frames_list)
    robot_com, _v, robot_rots, leg = anatomical_frames(robot_times, rob_joints)
    gvs, gps = compute_gvs(ref_times, ref_com, ref_rots,
                           robot_times, robot_com, robot_rots,
                           joints=joints, leg=leg, ref_joints=ref_joints, rob_joints=rob_joints)
    metrics, violations = capture_gates(robot_times, robot_com, rob_joints, leg=leg, gps_rad=gps)
    return report_and_gate(gvs, gps, tolerance, metrics=metrics, violations=violations)


def evaluate_recording(record_path, ref_path, joints=EVAL_JOINTS, tolerance=MAX_TOLERANCE_RAD):
    """Evaluate recorded CSV telemetry against reference FBX take."""
    import csv
    ref_times, ref_joints, ref_rots = extract_take_rotations(ref_path)
    ref_com = ref_joints['Hips'][:, 2]

    robot_times, frames_list = [], []
    with open(record_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            robot_times.append(float(row['t']))
            angles = {j_name: float(row[j_name]) for j_name in figure.JOINTS if j_name in row}
            pelvis_at = (float(row.get('x', 0.0)), float(row.get('y', 0.0)), float(row.get('z', 0.0)))
            pelvis_quat = figure.quat(float(row.get('qw', 1.0)), float(row.get('qx', 0.0)),
                                      float(row.get('qy', 0.0)), float(row.get('qz', 0.0)))
            frames_list.append(figure.frames(angles, pelvis_at, pelvis_quat))

    rob_joints = _sim_joints(frames_list)
    robot_com, _v, robot_rots, leg = anatomical_frames(robot_times, rob_joints)
    gvs, gps = compute_gvs(ref_times, ref_com, ref_rots,
                           robot_times, robot_com, robot_rots,
                           joints=joints, leg=leg, ref_joints=ref_joints, rob_joints=rob_joints)
    metrics, violations = capture_gates(robot_times, robot_com, rob_joints, leg=leg, gps_rad=gps)
    return report_and_gate(gvs, gps, tolerance, metrics=metrics, violations=violations)


def report_and_gate(gvs, gps, tolerance=MAX_TOLERANCE_RAD, metrics=None, violations=None):
    """Print GVS table, aggregate GPS, capture gates, and enforce the CI guillotine gate."""
    print("=" * 60)
    print("INVARIANT KINEMATIC EVALUATION (SO(3) LIE LOG MAP)")
    print("=" * 60)
    print(f"{'Joint':<20} {'GVS (rad)':<15} {'GVS (deg)':<15}")
    print("-" * 60)
    for joint, score in sorted(gvs.items()):
        print(f"{joint:<20} {score:<15.4f} {math.degrees(score):<15.2f}")
    print("-" * 60)
    print(f"{'GAIT PROFILE SCORE':<20} {gps:<15.4f} {math.degrees(gps):<15.2f}")
    print(f"{'MAX TOLERANCE':<20} {tolerance:<15.4f} {math.degrees(tolerance):<15.2f}")
    if metrics:
        print("-" * 60)
        for k in ('init_tau', 'first_step_star', 'step_star', 'walk_ratio', 'arm_deg', 'heel_deg',
                  'knee_min_deg', 'lift_asym', 'stall_share'):
            if k in metrics:
                lo, hi = GATE_BOUNDS.get(k, (0.0, 0.0))
                val = metrics[k]
                v_str = f"{val:.3f}" if val == val else "NaN"
                print(f"{k:<20} {v_str:<15} [{lo:.2f} .. {hi:.2f}]")
    print("=" * 60)

    gate_fails = [v for v in (violations or ()) if v[0] != 'gps_deg']
    if gps > tolerance or gate_fails:
        if gps > tolerance:
            print(f"\n[CI GUILLOTINE TRIPPED] GPS ({math.degrees(gps):.2f} deg) > MAX_TOLERANCE ({math.degrees(tolerance):.2f} deg).")
        if gate_fails:
            print("\n[CI GUILLOTINE TRIPPED] INVARIANT GATES BROKEN: "
                  + ", ".join(f"{n}={v:.2f} (bound {b:.2f})" for n, v, b in gate_fails))
        print("TERMINATING BUILD WITH STATUS 1.\n")
        return 1
    print(f"\n[CI PASSED] GPS ({math.degrees(gps):.2f} deg) <= MAX_TOLERANCE ({math.degrees(tolerance):.2f} deg).")
    print("KINEMATIC CONVERGENCE CERTIFIED.\n")
    return 0


DEFAULT_MOCAP = r"C:\sources\Projects\electronics\Mocaps\02-walkforward.fbx"
REF_DEFAULT = DEFAULT_MOCAP if os.path.isfile(DEFAULT_MOCAP) else os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'tests', 'takes', 'walk.fbx')


def main(argv=None):
    parser = argparse.ArgumentParser(description="Invariant Kinematic Evaluation & CI Guillotine on SO(3)")
    parser.add_argument('take', nargs='?', default=None, help="FBX take to evaluate")
    parser.add_argument('reference', nargs='?', default=REF_DEFAULT,
                        help=f"Reference normative mocap take (default: {REF_DEFAULT})")
    parser.add_argument('--eval', action='store_true', help="Run live simulation and evaluate against reference")
    parser.add_argument('--record', default=None, help="Path to telemetry recording CSV")
    parser.add_argument('--plan', nargs='*', default=None, help="Optional plan entries for --eval")
    parser.add_argument('--file', default=None, help="Optional Lisp file for --eval")
    parser.add_argument('--tolerance', type=float, default=MAX_TOLERANCE_DEG,
                        help=f"Max tolerance for GPS in degrees (default: {MAX_TOLERANCE_DEG})")
    parser.add_argument('--to', type=float, default=15.0, help="Simulation duration (s)")
    args = parser.parse_args(argv)

    tolerance_rad = math.radians(args.tolerance)
    text = None
    if args.file:
        with open(args.file, encoding='utf-8') as f:
            text = f.read()

    if args.eval:
        code = evaluate_simulation(args.reference, duration=args.to, tolerance=tolerance_rad,
                                   asked=args.plan, text=text)
        sys.exit(code)

    if args.record:
        code = evaluate_recording(args.record, args.reference, tolerance=tolerance_rad)
        sys.exit(code)

    if args.take:
        ref_times, ref_joints, ref_rots = extract_take_rotations(args.reference)
        tak_times, tak_joints, tak_rots = extract_take_rotations(args.take)
        ref_com = ref_joints['Hips'][:, 2]
        tak_com = tak_joints['Hips'][:, 2]
        gvs, gps = compute_gvs(ref_times, ref_com, ref_rots,
                               tak_times, tak_com, tak_rots,
                               ref_joints=ref_joints, rob_joints=tak_joints)
        code = report_and_gate(gvs, gps, tolerance_rad)
        sys.exit(code)

    parser.print_help()
    sys.exit(0)


if __name__ == '__main__':
    main()
