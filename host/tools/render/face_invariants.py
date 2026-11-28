"""Differential-geometry, Fibonacci-FFT, Darboux alignment, and gaze-kinematic head invariants."""
import math
import os
import sys
from typing import Any

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np  # noqa: E402
from scipy import ndimage  # noqa: E402

from coaxial.graphics.shapes import view  # noqa: E402
from tools.render import (  # noqa: E402
    face_built as fb, face_contours as fcon, face_cranium as fc,
    face_honeycomb as fh, face_hull as fhull, face_laid as fl, face_ray as fr,
)


def _cap_curvature(tris, cap_cols):
    raw = np.array([t for t, c in tris if c in cap_cols], float) * 1000.0
    pts_flat = raw.reshape(-1, 3)
    _, idx_u, inv = np.unique(np.round(pts_flat, 3), axis=0, return_index=True, return_inverse=True)
    v, f = pts_flat[idx_u], inv.reshape(-1, 3)
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 2] != f[:, 0])]
    nv, e_cnt = len(v), {}
    for tri in f:
        for a, b in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
            e = (a, b) if a < b else (b, a)
            e_cnt[e] = e_cnt.get(e, 0) + 1
    is_bnd = np.zeros(nv, dtype=bool)
    for (a, b), cnt in e_cnt.items():
        if cnt == 1:
            is_bnd[a] = is_bnd[b] = True
    v0, v1, v2 = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    fn = np.cross(v1 - v0, v2 - v0)
    a2 = np.linalg.norm(fn, axis=1)
    area = 0.5 * np.maximum(a2, 1e-12)
    lmax = np.maximum(np.linalg.norm(v1 - v0, axis=1), np.maximum(np.linalg.norm(v2 - v1, axis=1), np.linalg.norm(v0 - v2, axis=1)))
    max_asp = float((lmax / np.maximum(a2 / np.maximum(lmax, 1e-9), 1e-9)).max())
    va, vn, asum, hn = np.zeros(nv), np.zeros((nv, 3)), np.zeros(nv), np.zeros((nv, 3))
    for pa, pb, pc, vi, ia, ib in ((v0, v1, v2, f[:, 0], f[:, 1], f[:, 2]),
                                   (v1, v2, v0, f[:, 1], f[:, 2], f[:, 0]),
                                   (v2, v0, v1, f[:, 2], f[:, 0], f[:, 1])):
        u, w = pb - pa, pc - pa
        dot, cr = np.sum(u * w, axis=1), np.linalg.norm(np.cross(u, w), axis=1)
        np.add.at(va, vi, area / 3.0)
        np.add.at(vn, vi, fn)
        np.add.at(asum, vi, np.arctan2(cr, dot))
        we = 0.5 * np.clip(dot / np.maximum(cr, 1e-9), -50.0, 50.0)[:, None] * (v[ib] - v[ia])
        np.add.at(hn, ia, we)
        np.add.at(hn, ib, -we)
    vn /= np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-12)
    vs = np.maximum(va, 1e-6)
    h_curv, k_curv = -np.sum((hn / (2.0 * vs[:, None])) * vn, axis=1), (2.0 * np.pi - asum) / vs
    int2 = ~is_bnd
    for a, b in e_cnt:
        if is_bnd[a]:
            int2[b] = False
        if is_bnd[b]:
            int2[a] = False
    if int2.sum() < 5:
        int2 = ~is_bnd
    return round(float(np.sum((h_curv[int2] ** 2) * va[int2])), 3), round(100.0 * float((k_curv[int2] < -2e-4).mean()), 1), round(max_asp, 1)


def curvature_fairness_audit():
    """Measure discrete cotangent Willmore energy, aspect ratio, and Gaussian saddle percentage."""
    dp, fp, cp = fc.cranium_dome_panels(), fl.frontal_panels(), fl.cheek_and_nose_panels()
    cw = [_cap_curvature(dp[k], {fh.CRANIUM_DARK}) for k in ('dome_sagittal_front', 'dome_parietal_front_L', 'dome_parietal_mid_L', 'dome_parietal_rear_L')]
    _, fh_sad, _ = _cap_curvature(fp['forehead_porcelain_L'], {fh.SHELL_CF})
    _, ic_sad, _ = _cap_curvature(cp['cheek_inner_L'], {fh.SHELL_CF})
    return {'crown_willmore_max': max(x[0] for x in cw), 'crown_max_asp': max(x[2] for x in cw),
            'forehead_saddle_pct': fh_sad, 'cheek_saddle_pct': ic_sad}


def seam_continuity_audit():
    """Measure median 3D Z-step across all six coupled front facial panel seams."""
    po, p_cfg, fcam = fhull.panel_offset, fc.cfg(), fr.CANONICAL_CAMERAS['front']
    evs = {
        'forehead-diamond': lambda x, y: fl.eval_cranio_z(x, y) + 0.30 + po('forehead-diamond', x, y),
        'forehead-wings': lambda x, y: fl.eval_cranio_z(x, y) + po('forehead-wings', x, y),
        'face-brows': lambda x, y: fl._eval_brow_z(x, y) + po('face-brows', x, y),
        'face-inner-cheeks': lambda x, y: fl._eval_icheek_z(x, y) + po('face-inner-cheeks', x, y),
        'face-outer-cheeks': lambda x, y: fl._eval_ocheek_z(x, y) + po('face-outer-cheeks', x, y),
        'face-oled-mouth': fl._eval_mouth_z,
        'jaw-chin-cap': lambda x, y: fhull.eval_side_midline_z(y) + p_cfg['chin_z0'] - p_cfg['chin_sx'] * (math.hypot(abs(float(x)), 5.8) - 5.8) + po('jaw-chin-cap', x, y),
    }
    lmap = {'forehead-diamond': 'DIAMOND_LOOP', 'forehead-wings': 'TIARA_LOOP', 'face-brows': 'BROW_L_LOOP',
            'face-inner-cheeks': 'ICHEEK_L_LOOP', 'face-outer-cheeks': 'OCHEEK_L_LOOP', 'face-oled-mouth': 'MOUTH_LOOP', 'jaw-chin-cap': 'CHIN_LOOP'}
    meds = []
    for pa, pb in (('forehead-diamond', 'forehead-wings'), ('forehead-wings', 'face-brows'), ('face-brows', 'face-outer-cheeks'),
                   ('face-inner-cheeks', 'face-outer-cheeks'), ('face-inner-cheeks', 'face-oled-mouth'), ('face-oled-mouth', 'jaw-chin-cap')):
        la, lb = np.asarray(fcon.active_loop(lmap[pa]), float), np.asarray(fcon.active_loop(lmap[pb]), float)
        d = np.sqrt(((la[:, None, :] - lb[None, :, :]) ** 2).sum(2))
        diffs = [abs(fr.project_ray_z(la[i, 0], la[i, 1], fcam, evs[pa])[0][2] - fr.project_ray_z(lb[int(np.argmin(d[i])), 0], lb[int(np.argmin(d[i])), 1], fcam, evs[pb])[0][2])
                 for i in np.unique(np.where(d < 16.0)[0]) if d[i].min() < 16.0]
        if diffs:
            meds.append(float(np.median(diffs)))
    return {'seam_med_max_mm': round(max(meds or [0.0]), 3)}


def fibonacci_silhouette_audit(n_views=16):
    """Measure CPU-only Fibonacci-hemisphere silhouette Fourier decay slope and roughness."""
    posed_n, ht_a = fb.posed('neutral', include_neck=True), np.asarray(fl.anchors()[2], float)
    okeys = ('cranium_frontal', 'cranium_dome', 'cranium_occipital', 'left_temporal', 'right_temporal', 'left_mastoid',
             'right_mastoid', 'nasal_bridge', 'left_maxilla', 'right_maxilla', 'oled_bezel', 'oled_screen', 'left_ear_mic', 'right_ear_mic', 'neck_skin')
    pts_a = np.vstack([np.array([t for t, _ in posed_n[k]], np.float32).reshape(-1, 3) for k in okeys])
    p_rel = np.unique(np.rint(pts_a / 0.0022).astype(np.int32), axis=0).astype(np.float32) * 0.0022 - (ht_a + fr.canon_to_head((0.0, 121.0, -14.0))).astype(np.float32)
    ga, hfs, als = math.pi * (3.0 - math.sqrt(5.0)), [], []
    for k in range(int(n_views)):
        pt, yw = -18.0 + 56.0 * ((k + 0.5) / int(n_views)), ((math.degrees(k * ga) + 180.0) % 360.0) - 180.0
        pr = p_rel @ np.asarray(view(yw, pt), np.float32).reshape(3, 3).T
        zc = 1.50 - pr[:, 2]
        u, v_i = np.rint(79.5 + 680.0 * pr[:, 0] / zc).astype(np.int32), np.rint(99.5 - 680.0 * pr[:, 1] / zc).astype(np.int32)
        ok = (u >= 0) & (u < 160) & (v_i >= 0) & (v_i < 200)
        msk = np.zeros((200, 160), dtype=bool)
        msk[v_i[ok], u[ok]] = True
        msk = fr.fill_interior_holes(np.asarray(ndimage.binary_closing(msk, iterations=2), dtype=bool), 20000)
        res_l: Any = ndimage.label(msk)
        lbl = np.asarray(res_l[0], dtype=np.int64)
        cnt = np.bincount(lbl.ravel())
        cnt[0] = 0
        poly = np.asarray(fr.trace_exact_crack_polygon(lbl == int(np.argmax(cnt)), rdp_eps=0.0), float)
        s = np.r_[0.0, np.cumsum(np.hypot(np.diff(np.r_[poly[:, 0], poly[0, 0]]), np.diff(np.r_[poly[:, 1], poly[0, 1]])))]
        su = np.linspace(0.0, s[-1], 256, endpoint=False)
        xr, yr = np.interp(su, s, np.r_[poly[:, 0], poly[0, 0]]), np.interp(su, s, np.r_[poly[:, 1], poly[0, 1]])
        c = np.fft.fft((xr - xr.mean()) + 1j * (yr - yr.mean())) / 256.0
        mag = np.array([math.hypot(abs(c[mi]), abs(c[-mi])) for mi in range(1, 65)])
        mag /= max(mag[0], 1e-9)
        hfs.append(float(np.sum(mag[15:] ** 2) / max(np.sum(mag ** 2), 1e-12)))
        als.append(-float(np.polyfit(np.log(np.arange(2, 25)), np.log(np.maximum(mag[1:24], 1e-6)), 1)[0]))
    return {'fib_alpha_min': round(min(als), 2), 'fib_hf_max_1e4': round(1e4 * max(hfs), 2)}


def darboux_alignment_audit():
    """Measure Darboux-frame alignment penalty J_align for exposed structural cranial modules."""
    neu = fb.head_parts(**fb.PRESETS['neutral'], plates=True)
    s_pts, s_frm = [], []
    for phi in np.linspace(-math.pi, math.pi, 32, endpoint=False):
        for yl in np.linspace(-65.0, 80.0, 18):
            p0, nrm = fc.sample_head_surface_mm(float(phi), float(yl))
            ph = fr.canon_to_head(p0)
            t2 = fr.canon_to_head(fc.eval_head_surface_mm(float(phi) + 0.02, float(yl))) - ph
            t2 -= float(np.dot(t2, nrm)) * nrm
            l2 = float(np.linalg.norm(t2))
            if l2 > 1e-9:
                t2 /= l2
                s_pts.append(ph)
                s_frm.append((np.cross(nrm, t2), t2, nrm))
    s_arr = np.asarray(s_pts, float)
    out = {}
    for key, tag in (('cranium_occipital', 'darboux_occ_j'), ('left_mastoid', 'darboux_mastoid_j'), ('gaze_servos', 'darboux_gaze_j')):
        pts = np.array([t for t, _ in neu[key]], float).reshape(-1, 3)
        ctr = pts.mean(0)
        v_i = np.linalg.eigh(np.cov((pts - ctr).T))[1][:, -1]
        t1, t2, nrm = s_frm[int(np.argmin(((s_arr - ctr) ** 2).sum(1)))]
        out[tag] = round(1.0 - max(abs(float(np.dot(v_i, ax))) for ax in (t1, t2, nrm)) ** 2, 4)
    return out


def eyes_kinematics_audit():
    """Verify saccade Main Sequence saturation and zero-roll-axis Fick kinematics on the robot's eyes."""
    from machine.eyes import gaze, gimbal
    ts = [gaze.saccade(math.radians(a)) for a in (5.0, 15.0, 30.0, 50.0)]
    ws = [1.875 * a / t for a, t in zip((5.0, 15.0, 30.0, 50.0), ts)]
    r_yaw = gimbal.turned((math.radians(30.0), 0.0))
    r_pit = gimbal.turned((0.0, math.radians(20.0)))
    cardinal_torsion = max(abs(float(r_yaw[1, 0] - r_yaw[0, 1])), abs(float(r_pit[1, 0] - r_pit[0, 1])))
    return {'main_seq_ok': bool(all(a < b for a, b in zip(ts, ts[1:])) and ws[0] < ws[1] < ws[2] <= 605.0 and cardinal_torsion < 1e-9),
            'cardinal_torsion': round(cardinal_torsion, 6)}


def geometric_invariants_audit(n_views=16):
    """Run all four aesthetic and kinematic invariant audits in one headless call."""
    return {**curvature_fairness_audit(), **seam_continuity_audit(),
            **fibonacci_silhouette_audit(n_views), **darboux_alignment_audit(), **eyes_kinematics_audit()}
