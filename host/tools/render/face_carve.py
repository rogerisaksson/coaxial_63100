"""Full-resolution 360-deg visual-hull carver and decoupled sub-pixel silhouette sculpt."""
import json
import math
import os

import numpy as np
from scipy import ndimage

from tools.render import (
    face_built, face_contours as fcon, face_cranium as fc, face_delta,
    face_hull as fhull, face_laid as fl, face_ray as fr,
)


def ensure_lr():
    from coaxial.graphics import gpu
    if not face_delta._LR_CACHE:
        try:
            face_delta._LR_CACHE.append(gpu.LitRaster())
        except Exception:
            face_delta._LR_CACHE.append(None)
    return face_delta._LR_CACHE[0]


def stack_groups(grps):
    return np.vstack([v[0] for v in grps.values() if len(v[0])]), np.concatenate([v[1] for v in grps.values() if len(v[1])])


def _rebuild_hull_groups(grps, sheath, head=True, neck=True, fixed_liner=None):
    nt, _hr, ht, *_r = fl.anchors()
    ht_v, nt_v = np.asarray(ht, float), np.asarray(nt, float)
    if head:
        ll, lr_s = fl.cranial_liner_shells()
        ocp, mp, ears = fc.occipital_panels(), fc.mastoid_armor_plates(), fl.ear_microphones()
        if fixed_liner is None:
            cp, jp, ex = fl.cheek_and_nose_panels(), fl.lower_jaw_and_oled_panels(), face_built._expressions_posed(0.0, 0.0, 0.0, 0.0, 0.0)
            fixed_liner = cp['orbital_pillar_L'] + cp['orbital_pillar_R'] + ex['left_brow'] + ex['right_brow'] + jp['oled_carrier_bezel'] + fl.chassis_static()
        for k, ts in (('crown-top-plates', fc.cranium_dome_plates()), ('crown-nape-yoke', ocp['occipital_nape_yoke']),
                      ('crown-occipital-plates', ocp['occipital_upper_L'] + ocp['occipital_upper_R'] + ocp['occipital_lower_L'] + ocp['occipital_lower_R'] + mp + fl.mirror_x(mp)),
                      ('temple-ear-pods', ears['left_ear_mic'] + ears['right_ear_mic']), ('frame-liner-xtruss', ll + lr_s + ocp['occipital_xbrace_module'] + fixed_liner)):
            grps[k] = face_delta.pack_id_group(k, ts, ht_v)
    if neck:
        grps['neck-tubes-bust'] = face_delta.pack_id_group('neck-tubes-bust', sheath + [t for ts in fc.neck_hull_panels().values() for t in ts], nt_v)


def fast_panel_err(grps, refs, pid_focus=None, sil_only=False):
    lr, ht, pnames = ensure_lr(), fl.anchors()[2], list(face_delta.PART_IDS.keys())
    pos_id, mat_id = stack_groups(grps)
    tot_s, tot_p, pv = 0, 0, {}
    for vn, pid, _fn in face_delta.VIEWS:
        rf, seg = refs[vn]['fg'], refs[vn]['seg']
        _r, cf, pidx, _i = face_delta._raster_canonical(None, pos_id, mat_id, pid, ht, lr)
        se = int((rf[:int(0.77 * rf.shape[0])] ^ cf[:int(0.77 * rf.shape[0])]).sum())
        tot_s += se
        pv[vn] = {'_sil': se}
        if not sil_only:
            for k, mr in seg.items():
                if pid_focus is None or k == pid_focus:
                    ti = pnames.index(k) + 1
                    pe = int((mr ^ (fr.fill_interior_holes(pidx == ti, 38000) if k in face_delta.HONEYCOMB_PARTS else (pidx == ti))).sum())
                    tot_p += pe
                    pv[vn][k] = pe
    return tot_s, tot_p, pv


def _inside_4view_360(pts, fgs, cam_deltas):
    ok = np.ones(len(pts), dtype=bool)
    for vn, pid, _fn in face_delta.VIEWS:
        u, v, w, h = fr.project_uv_batch(pts, pid, cam_deltas[pid])
        ui, vi = np.rint(u).astype(int), np.rint(v).astype(int)
        inf = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
        vcl = np.clip(vi, 0, h - 1)
        hit = np.zeros(len(pts), dtype=bool)
        hit[inf] = fgs[vn][vi[inf], ui[inf]]
        hit[(vi >= h) & (ui >= -160) & (ui < w + 160)] = True
        if vn == 'BACK':
            hit[(vi >= int(0.84 * h)) & (vi < h) & (((ui < 0) & fgs[vn][vcl, 0]) | ((ui >= w) & fgs[vn][vcl, w - 1]))] = True
        ok &= hit
    return ok


def _carve_initial_360_hull(fgs, cam_deltas, front_outer_x, tuning):
    hny, hnp, nny, nnp = fhull.HEAD_NY, fhull.HEAD_NPHI, fhull.NECK_NY, fhull.NECK_NPHI
    ys_n, phis_n = np.linspace(fhull.NECK_Y_MIN, fhull.NECK_Y_MAX, nny), np.linspace(-math.pi, math.pi, nnp, endpoint=False)
    rhos = np.linspace(8.0, 152.0, 180)
    def_nxz = np.array([[[*fhull.default_neck_xz(float(yv), float(ph), tuning['bust_rx'])] for ph in phis_n] for yv in ys_n], float)
    zc_arr = np.zeros(nny, float)
    for i, yv in enumerate(ys_n):
        fy = float(yv)
        zp = np.c_[np.zeros(160), np.full(160, max(24.0, fy)), np.linspace(-116.0, 40.0, 160)]
        us, vs, ws, hs = fr.project_uv_batch(zp, 'profile90', cam_deltas['profile90'])
        z_in = zp[fgs['SIDE'][np.clip(np.rint(vs).astype(int), 0, hs - 1), np.clip(np.rint(us).astype(int), 0, ws - 1)], 2]
        zc = float(0.5 * (z_in.min() + z_in.max())) if len(z_in) >= 4 else (-42.0 - 0.22 * max(0.0, 24.0 - fy))
        zc_arr[i] = min(zc, -34.0) if fy > 58.0 else zc

    sp, cp, rr = np.sin(phis_n)[None, :, None], np.cos(phis_n)[None, :, None], rhos[None, None, :]
    z_n = zc_arr[:, None, None] + rr * cp
    pts_n = np.stack([np.broadcast_to(rr * sp, (nny, nnp, len(rhos))), np.broadcast_to(ys_n[:, None, None], z_n.shape), z_n], axis=-1).reshape(-1, 3)
    ok_n = _inside_4view_360(pts_n, fgs, cam_deltas).reshape(nny, nnp, len(rhos))
    vh_nxz = def_nxz.copy()
    for i, yv in enumerate(ys_n):
        fy, zc = float(yv), float(zc_arr[i])
        for j, ph in enumerate(phis_n):
            fph = float(ph)
            r_vh = float(rhos[ok_n[i, j]].max()) if ok_n[i, j].any() else math.hypot(def_nxz[i, j, 0], def_nxz[i, j, 1] - zc)
            xm_v, zm_v = r_vh * math.sin(fph), zc + r_vh * math.cos(fph)
            z_cap = (14.0 - 0.80 * max(0.0, fy - 62.0)) if abs(xm_v) <= 20.0 else (4.0 - 0.85 * max(0.0, fy - 60.0))
            if fy > 58.0 and zm_v > z_cap and math.cos(fph) > 0.08:
                r_vh = min(r_vh, max(12.0, (z_cap - zc) / math.cos(fph)))
            if fy > 72.0 and zm_v > -10.0 and abs(xm_v) > 27.5:
                r_vh = min(r_vh, 27.5 / max(0.2, abs(math.sin(fph))))
            ru = max(11.0, min(148.0, r_vh))
            zm_u = zc + ru * math.cos(fph)
            vh_nxz[i, j] = (ru * math.sin(fph), max(-58.0, zm_u) if 22.0 <= fy <= 78.0 else zm_u)
    for ax_k in (0, 1):
        vh_nxz[:, :, ax_k] = ndimage.gaussian_filter1d(ndimage.gaussian_filter1d(vh_nxz[:, :, ax_k], 0.9, axis=0, mode='nearest'), 0.9, axis=1, mode='wrap')

    ys_h, phis_h = np.linspace(-62.0, 90.5, hny), np.linspace(-math.pi, math.pi, hnp, endpoint=False)
    drs = np.linspace(-14.0, 22.0, 145)
    os.environ['FACE_HULL_JSON'] = fhull.dumps_hull({'cam_deltas': cam_deltas, 'front_outer_x': front_outer_x})
    sn_all = [fc.sample_head_surface_mm(float(ph), min(tuning['crown_top'] - 0.25, float(yl))) for yl in ys_h for ph in phis_h]
    p0_g = np.array([s[0] for s in sn_all], float).reshape(hny, hnp, 3)
    nrm_g = np.array([s[1] for s in sn_all], float).reshape(hny, hnp, 3)
    ok_h = _inside_4view_360((p0_g[:, :, None, :] + drs[None, None, :, None] * nrm_g[:, :, None, :]).reshape(-1, 3),
                             fgs, cam_deltas).reshape(hny, hnp, len(drs))
    vh_hdr = np.zeros((hny, hnp), float)
    for iv, yl in enumerate(ys_h):
        for iu, ph in enumerate(phis_h):
            if (abs(float(ph)) >= 0.40 or float(yl) >= 18.0) and ok_h[iv, iu].any():
                vh_hdr[iv, iu] = float(np.clip(drs[ok_h[iv, iu]].max(), -12.0, 20.5))
    sm_hdr = np.asarray(ndimage.gaussian_filter1d(ndimage.gaussian_filter1d(vh_hdr, 1.0, axis=0, mode='nearest'), 0.9, axis=1, mode='wrap'), float)
    sm_hdr[-4:] = np.maximum(sm_hdr[-4:], -0.9)
    return sm_hdr, vh_nxz, def_nxz, ys_h, phis_h, ys_n, phis_n


def _sm0(arr, sig=0.85):
    return ndimage.gaussian_filter1d(arr, sig, axis=0, mode='nearest')


def _scan_lr_edges(fg_r, fg_c, r0=36, r1_off=2, scale=0.22, lim=6.0):
    d_l, d_r = np.zeros(fg_r.shape[0], float), np.zeros(fg_r.shape[0], float)
    for py in range(r0, fg_r.shape[0] - r1_off):
        xr, xc = np.nonzero(fg_r[py])[0], np.nonzero(fg_c[py])[0]
        if len(xr) >= 4 and len(xc) >= 4:
            d_l[py] = np.clip(scale * float(xc[0] - xr[0]), -lim, lim)
            d_r[py] = np.clip(scale * float(xr[-1] - xc[-1]), -lim, lim)
    return ndimage.gaussian_filter1d(d_l, 0.45), ndimage.gaussian_filter1d(d_r, 0.45)


def _lock_rear_360(hdr, nxz, grps, sheath, refs, h_ctx, ys_h, phis_h, ys_n, phis_n, pv_b, best_s, best_p, fix_lin, guard_front):
    lr, ht, b_fg, cd_b = ensure_lr(), fl.anchors()[2], refs['BACK']['fg'], face_delta._cam_delta_for('rear')
    best_s, best_p, pv_b = fast_panel_err(grps, refs, pid_focus='temple-hex-bezels')
    pnames = list(face_delta.PART_IDS.keys())
    ti_t, ti_j = pnames.index('temple-hex-bezels') + 1, pnames.index('jaw-rear-hex') + 1
    b_cran = b_fg & (~refs['BACK']['seg'].get('temple-hex-bezels', False)) & (~refs['BACK']['seg'].get('jaw-rear-hex', False))
    by_ys = np.array([fr.project_ray_z(448.0, float(py) + 0.5, fr.CANONICAL_CAMERAS['rear'], lambda _x, _y: -65.0, 0.0, cd_b)[0][1] for py in range(b_fg.shape[0])], float)
    w_sh = np.clip((np.abs(phis_h) - 1.88) / 0.22, 0.0, 1.0) * (np.abs(np.sin(phis_h)) ** 1.3)
    wb_p, wb_n = (phis_h > 0.0).astype(float) * w_sh, (phis_h < 0.0).astype(float) * w_sh
    nb_p, nb_n = (phis_n > 2.02) & (phis_n < 3.06), (phis_n < -2.02) & (phis_n > -3.06)
    r_h = np.argmin(np.abs(by_ys[:, None] - (fr.HEAD_CENTER_Y + ys_h[None, :])), axis=0)
    r_n = np.argmin(np.abs(by_ys[:, None] - ys_n[None, :]), axis=0)
    for _ in range(6 if guard_front else 8):
        _r, cf_b, pidx_b, _i = face_delta._raster_canonical(None, *stack_groups(grps), 'rear', ht, lr)
        dx_bp, dx_bn = _scan_lr_edges(b_cran, cf_b & (pidx_b != ti_t) & (pidx_b != ti_j), 42, 1, lim=12.0)
        dx_np, dx_nn = _scan_lr_edges(b_fg, cf_b, 42, 1, lim=18.0)
        imp_b = False
        for st in (0.82, 0.36):
            c_hdr = np.clip(hdr + _sm0(st * (dx_bp[r_h, None] * wb_p[None, :] + dx_bn[r_h, None] * wb_n[None, :])), -18.0, 23.0)
            c_hdr[-4:] = np.maximum(c_hdr[-4:], -1.2)
            c_nxz, nm = nxz.copy(), ys_n <= 93.0
            c_nxz[nm, :, 0] = np.where(nb_p[None, :], np.maximum(8.0, c_nxz[nm, :, 0] + st * dx_np[r_n[nm], None]),
                                       np.where(nb_n[None, :], np.minimum(-8.0, c_nxz[nm, :, 0] - st * dx_nn[r_n[nm], None]), c_nxz[nm, :, 0]))
            os.environ['FACE_HULL_JSON'] = fhull.dumps_hull({**h_ctx, 'head_dr': c_hdr.tolist(), 'neck_xz': c_nxz.tolist()})
            g_try = dict(grps)
            _rebuild_hull_groups(g_try, sheath, fixed_liner=fix_lin)
            s_c, p_c, pv_c = fast_panel_err(g_try, refs, pid_focus='temple-hex-bezels')
            if (pv_c['BACK']['_sil'] + pv_c['BACK'].get('temple-hex-bezels', 0)
                    < pv_b['BACK']['_sil'] + pv_b['BACK'].get('temple-hex-bezels', 0) - 14
                    and (not guard_front or pv_c['FRONT']['_sil'] <= pv_b['FRONT']['_sil'] + 18)):
                hdr, nxz, grps, best_s, best_p, pv_b, imp_b = c_hdr, c_nxz, g_try, s_c, p_c, pv_c, True
                break
        if not imp_b:
            break
    return hdr, nxz, grps, best_s, best_p, pv_b


def _lock_side_and_persp(hdr, nxz, grps, sheath, refs, h_ctx, ys_h, phis_h, ys_n, phis_n, pv_b, best_s, best_p, fix_lin, guard_front):
    lr, ht, s_fg, p_fg = ensure_lr(), fl.anchors()[2], refs['SIDE']['fg'], refs['PERSP']['fg']
    cd_s, cd_p = face_delta._cam_delta_for('profile90'), face_delta._cam_delta_for('threeQuarter')
    w_back, w_front = (np.maximum(0.0, -np.cos(phis_h)) ** 6.5), (np.maximum(0.0, np.cos(phis_h)) ** 4.5)
    w_nb, w_nf, f_sec = (np.maximum(0.0, -np.cos(phis_n)) ** 3.6), (np.maximum(0.0, np.cos(phis_n)) ** 3.2), (np.cos(phis_n) > 0.08)
    u_hb, v_hb, _, _ = fr.project_uv_batch([fc.eval_head_surface_mm(math.pi, min(87.6, float(yl))) for yl in ys_h], 'profile90', cd_s)
    u_hf, v_hf, _, _ = fr.project_uv_batch([fc.eval_head_surface_mm(0.0, min(87.6, float(yl))) for yl in ys_h], 'profile90', cd_s)
    rb_i, rf_i = np.clip(np.rint(v_hb).astype(int), 0, s_fg.shape[0] - 2), np.clip(np.rint(v_hf).astype(int), 0, s_fg.shape[0] - 2)
    ub_i, uf_i = np.clip(np.rint(u_hb).astype(int), 0, s_fg.shape[1] - 2), np.clip(np.rint(u_hf).astype(int), 0, s_fg.shape[1] - 2)
    top_w, s_neck_fg = np.clip((ys_h - 24.0) / 48.0, 0.0, 1.0), s_fg.copy()
    s_neck_fg[:595, 560:] = s_neck_fg[:612, 705:] = False
    for _ in range(5 if guard_front else 7):
        _r, cf_s, pidx_s, _i = face_delta._raster_canonical(None, *stack_groups(grps), 'profile90', ht, lr)
        dz_b, dz_f = _scan_lr_edges(s_fg, cf_s, 36)
        dz_nb, dz_nf = _scan_lr_edges(s_neck_fg, cf_s & (pidx_s >= 18), 520, 1, lim=9.0)
        dy_col = np.zeros(s_fg.shape[1], float)
        for px in range(280, 845):
            yr, yc = np.nonzero(s_fg[:360, px])[0], np.nonzero(cf_s[:360, px])[0]
            if len(yr) >= 3 and len(yc) >= 3:
                dy_col[px] = np.clip(0.22 * float(int(yc[0]) - int(yr[0])), -5.5, 5.5)
        dy_col = ndimage.gaussian_filter1d(dy_col, 1.2)
        db = dz_b[rb_i] * (1.0 - 0.55 * top_w) + dy_col[ub_i] * top_w
        df = np.where(ys_h > 36.0, dz_f[rf_i] * (1.0 - 0.55 * top_w) + dy_col[uf_i] * top_w, 0.0)
        imp_s = False
        for st in (0.82, 0.36):
            c_hdr = np.clip(hdr + _sm0(st * (db[:, None] * w_back[None, :] + df[:, None] * w_front[None, :])), -15.0, 24.0)
            c_hdr[-5:] = np.maximum(c_hdr[-5:], -0.9)
            c_nxz = nxz.copy()
            for iv, yv in enumerate(ys_n):
                fy = float(yv)
                if fy <= 93.0:
                    vs_iv = np.clip(np.rint(fr.project_uv_batch(np.c_[c_nxz[iv, :, 0], np.full(len(phis_n), fy), c_nxz[iv, :, 1]], 'profile90', cd_s)[1]).astype(int), 0, s_fg.shape[0] - 2)
                    c_nxz[iv, :, 1] = c_nxz[iv, :, 1] - st * dz_nb[vs_iv] * w_nb + st * dz_nf[vs_iv] * w_nf
                    if 16.0 <= fy <= 79.0:
                        c_nxz[iv, :, 1] = np.maximum(-64.0, c_nxz[iv, :, 1])
                    elif fy > 79.0:
                        c_nxz[iv, :, 1] = np.maximum(-79.0, c_nxz[iv, :, 1])
                    if fy > 66.0:
                        c_nxz[iv, f_sec, 1] = np.minimum(12.0 - 0.85 * (fy - 66.0), c_nxz[iv, f_sec, 1])
            os.environ['FACE_HULL_JSON'] = fhull.dumps_hull({**h_ctx, 'head_dr': c_hdr.tolist(), 'neck_xz': c_nxz.tolist()})
            g_try = dict(grps)
            _rebuild_hull_groups(g_try, sheath, fixed_liner=fix_lin)
            s_c, p_c, pv_c = fast_panel_err(g_try, refs, sil_only=True)
            if pv_c['SIDE']['_sil'] < pv_b['SIDE']['_sil'] - 5 and (not guard_front or pv_c['FRONT']['_sil'] <= pv_b['FRONT']['_sil'] + 12):
                hdr, nxz, grps, best_s, best_p, pv_b, imp_s = c_hdr, c_nxz, g_try, s_c, p_c, pv_c, True
                break
        if not imp_s:
            break

    wp_r = np.exp(-((((phis_h + 2.57 + math.pi) % (2.0 * math.pi) - math.pi) / 0.30) ** 2))
    wn_r = (phis_n < -2.20) & (phis_n > -2.92)
    vp_i = np.clip(np.rint(fr.project_uv_batch([fc.eval_head_surface_mm(-2.57, min(87.6, float(yl))) for yl in ys_h], 'threeQuarter', cd_p)[1]).astype(int), 0, p_fg.shape[0] - 2)
    for _ in range(3):
        _r, cf_p, _p, _i = face_delta._raster_canonical(None, *stack_groups(grps), 'threeQuarter', ht, lr)
        dp_l, _dp_r = _scan_lr_edges(p_fg, cf_p, 40, 1, lim=5.0)
        imp_p = False
        for st in (0.60, 0.25):
            c_hdr = np.clip(hdr + st * dp_l[vp_i, None] * wp_r[None, :], -15.0, 24.0)
            c_nxz = nxz.copy()
            for iv, yv in enumerate(ys_n):
                if float(yv) <= 86.0:
                    vs_iv = np.clip(np.rint(fr.project_uv_batch(np.c_[c_nxz[iv, :, 0], np.full(len(phis_n), float(yv)), c_nxz[iv, :, 1]], 'threeQuarter', cd_p)[1]).astype(int), 0, p_fg.shape[0] - 2)
                    c_nxz[iv, wn_r, 0] -= 0.58 * st * dp_l[vs_iv[wn_r]]
                    c_nxz[iv, wn_r, 1] -= 0.80 * st * dp_l[vs_iv[wn_r]]
            os.environ['FACE_HULL_JSON'] = fhull.dumps_hull({**h_ctx, 'head_dr': c_hdr.tolist(), 'neck_xz': c_nxz.tolist()})
            g_try = dict(grps)
            _rebuild_hull_groups(g_try, sheath, fixed_liner=fix_lin)
            s_c, p_c, pv_c = fast_panel_err(g_try, refs, sil_only=True)
            if pv_c['PERSP']['_sil'] < pv_b['PERSP']['_sil'] - 15 and pv_c['SIDE']['_sil'] <= pv_b['SIDE']['_sil'] + 10 and pv_c['FRONT']['_sil'] <= pv_b['FRONT']['_sil'] + 6:
                hdr, nxz, grps, best_s, best_p, pv_b, imp_p = c_hdr, c_nxz, g_try, s_c, p_c, pv_c, True
                break
        if not imp_p:
            break
    return hdr, nxz, grps, best_s, best_p, pv_b


def _step_front_neck(c_nxz, ys_n, phis_n, f_fg, cd_f, step_f, dx_pos, dx_neg, wp_n, wn_n):
    h, w = f_fg.shape
    for iv, yv in enumerate(ys_n):
        us_iv, vs_iv, _, _ = fr.project_uv_batch(np.c_[c_nxz[iv, :, 0], np.full(len(phis_n), float(yv)), c_nxz[iv, :, 1]], 'front', cd_f)
        for j in range(len(phis_n)):
            if float(vs_iv[j]) >= h + 24.0:
                continue
            rj, uj = int(np.clip(round(float(vs_iv[j])), 0, h - 2)), int(round(float(us_iv[j])))
            xr = np.nonzero(f_fg[rj])[0]
            if len(xr) >= 4 and (uj < int(xr[0]) or uj > int(xr[-1])):
                c_nxz[iv, j, 0] += 0.20 * step_f * (float(xr[0] + 1 if uj < int(xr[0]) else xr[-1] - 1) - float(us_iv[j]))
            elif (rj < h - 8 or abs(float(phis_n[j])) < 1.95) and (uj < 0 or uj >= w or not f_fg[rj, uj]):
                c_nxz[iv, j, 0] *= (1.0 - 0.14 * step_f)
            elif float(yv) <= 80.0 and abs(float(phis_n[j])) < 1.95:
                c_nxz[iv, j, 0] += step_f * (float(dx_pos[rj]) * float(wp_n[j]) - float(dx_neg[rj]) * float(wn_n[j]))


def _sculpt_front_mode(mode, hdr, nxz, grps, sheath, refs, h_ctx, tuning, ys_h, phis_h, ys_n, phis_n, pv_b, best_s, best_p, fix_lin, rounds):
    lr, ht, f_fg, cd_f = ensure_lr(), fl.anchors()[2], refs['FRONT']['fg'], face_delta._cam_delta_for('front')
    pos_m, neg_m = phis_h >= 0.0, phis_h < 0.0
    wc = ((np.abs(np.sin(phis_h)) ** 2.0) * np.clip((1.90 - np.abs(phis_h)) / 0.24, 0.0, 1.0))[None, :]
    sec_f = np.clip((1.90 - np.abs(phis_n)) / 0.28, 0.0, 1.0) * np.minimum(1.0, np.abs(np.sin(phis_n)) * 1.45)
    wp_n, wn_n = (phis_n > 0.06).astype(float) * sec_f, (phis_n < -0.06).astype(float) * sec_f
    ri_h = np.clip(np.rint(fr.project_uv_batch([fc.eval_head_surface_mm(1.57, min(tuning['crown_top'] - 0.3, float(yl))) for yl in ys_h], 'front', cd_f)[1]).astype(int), 0, f_fg.shape[0] - 2)
    ui_top = np.clip(np.rint(fr.project_uv_batch([fc.eval_head_surface_mm(float(ph), tuning['crown_top'] - 4.0) for ph in phis_h], 'front', cd_f)[0]).astype(int), 0, f_fg.shape[1] - 2)
    w_top_y = np.clip((ys_h - 46.0) / 38.0, 0.0, 1.0)[:, None]
    for f_it in range(int(rounds)):
        _r, cf_f, _p, _i = face_delta._raster_canonical(None, *stack_groups(grps), 'front', ht, lr)
        dx_neg, dx_pos = _scan_lr_edges(f_fg, cf_f, 34, 1, lim=7.5)
        dy_top = np.zeros(f_fg.shape[1], float)
        for px in range(210, 686):
            yr, yc = np.nonzero(f_fg[:210, px])[0], np.nonzero(cf_f[:210, px])[0]
            if len(yr) >= 3 and len(yc) >= 3:
                dy_top[px] = np.clip(0.24 * float(int(yc[0]) - int(yr[0])), -4.0, 4.0)
        dy_top = ndimage.gaussian_filter1d(dy_top, 1.4)
        imp = False
        for step_f in (0.82, 0.36, 0.14):
            c_hdr, c_nxz = hdr.copy(), nxz.copy()
            if mode == 'head':
                c_hdr[:, pos_m] += step_f * dx_pos[ri_h, None] * wc[:, pos_m]
                c_hdr[:, neg_m] += step_f * dx_neg[ri_h, None] * wc[:, neg_m]
                c_hdr += step_f * w_top_y * dy_top[ui_top][None, :]
                c_hdr = np.clip(hdr + ndimage.gaussian_filter1d(c_hdr - hdr, 0.7, axis=0, mode='nearest'), -14.0, 24.0)
                c_hdr[-5:] = np.maximum(c_hdr[-5:], -1.4)
            else:
                _step_front_neck(c_nxz, ys_n, phis_n, f_fg, cd_f, step_f, dx_pos, dx_neg, wp_n, wn_n)
            os.environ['FACE_HULL_JSON'] = fhull.dumps_hull({**h_ctx, 'head_dr': c_hdr.tolist(), 'neck_xz': c_nxz.tolist()})
            g_try = dict(grps)
            _rebuild_hull_groups(g_try, sheath, head=(mode == 'head'), neck=(mode == 'neck'), fixed_liner=fix_lin)
            s_c, p_c, pv_c = fast_panel_err(g_try, refs, sil_only=True)
            if pv_c['FRONT']['_sil'] < pv_b['FRONT']['_sil']:
                hdr, nxz, grps, best_s, best_p, pv_b, imp = c_hdr, c_nxz, g_try, s_c, p_c, pv_c, True
                break
        if not imp:
            break
    return hdr, nxz, grps, best_s, best_p, pv_b


def carve_and_sculpt_hull(refs, cam_deltas, front_outer_x, tuning, panel_grids, loops, rounds=18, init_hull=None):
    """Full-res `360`-deg visual-hull carving + `BACK`/`SIDE`/`PERSP` sculpt + decoupled `FRONT` lock."""
    from coaxial.graphics import lit
    from coaxial.graphics.shapes import finer, loft
    from tools.render.robot_worn import NECK_RINGS
    os.environ['FACE_TUNE_JSON'] = json.dumps(tuning)
    if loops:
        os.environ['FACE_CONTOURS_JSON'] = fcon.dumps_contours(loops)
    c_sh, tr_sh, _u, _m = loft(finer(NECK_RINGS, -0.040, 0.048, 0.004), lit.paint((24, 26, 30)), poles=(-0.044, 0.050))
    c_sh[:, 0], c_sh[:, 2] = 0.38 * c_sh[:, 0], 0.38 * c_sh[:, 2] - 0.022
    sheath, fgs = [(c_sh[t], (24, 26, 30)) for t in tr_sh], {vn: refs[vn]['fg'] for vn, _, _ in face_delta.VIEWS}
    if init_hull is not None:
        hdr, nxz = np.asarray(init_hull[0], float).copy(), np.asarray(init_hull[1], float).copy()
        ys_h, phis_h = np.linspace(-62.0, 90.5, fhull.HEAD_NY), np.linspace(-math.pi, math.pi, fhull.HEAD_NPHI, endpoint=False)
        ys_n, phis_n = np.linspace(fhull.NECK_Y_MIN, fhull.NECK_Y_MAX, fhull.NECK_NY), np.linspace(-math.pi, math.pi, fhull.NECK_NPHI, endpoint=False)
        def_nxz = np.array([[[*fhull.default_neck_xz(float(yv), float(ph), tuning['bust_rx'])] for ph in phis_n] for yv in ys_n], float)
    else:
        hdr, nxz, def_nxz, ys_h, phis_h, ys_n, phis_n = _carve_initial_360_hull(fgs, cam_deltas, front_outer_x, tuning)
    h_ctx = {'cam_deltas': cam_deltas, 'front_outer_x': front_outer_x, 'panel_grids': panel_grids}
    os.environ['FACE_HULL_JSON'] = fhull.dumps_hull({**h_ctx, 'head_dr': hdr.tolist(), 'neck_xz': nxz.tolist()})
    grps = face_delta.build_id_groups_stl()
    best_s, best_p, pv_b = fast_panel_err(grps, refs)
    cp0, jp0, ex0 = fl.cheek_and_nose_panels(), fl.lower_jaw_and_oled_panels(), face_built._expressions_posed(0.0, 0.0, 0.0, 0.0, 0.0)
    fix_lin = cp0['orbital_pillar_L'] + cp0['orbital_pillar_R'] + ex0['left_brow'] + ex0['right_brow'] + jp0['oled_carrier_bezel'] + fl.chassis_static()
    args = (ys_h, phis_h, ys_n, phis_n)
    hdr, nxz, grps, best_s, best_p, pv_b = _lock_side_and_persp(hdr, nxz, grps, sheath, refs, h_ctx, *args, pv_b, best_s, best_p, fix_lin, False)
    hdr, nxz, grps, best_s, best_p, pv_b = _lock_rear_360(hdr, nxz, grps, sheath, refs, h_ctx, *args, pv_b, best_s, best_p, fix_lin, False)
    for mode in ('head', 'neck'):
        hdr, nxz, grps, best_s, best_p, pv_b = _sculpt_front_mode(mode, hdr, nxz, grps, sheath, refs, h_ctx, tuning, *args, pv_b, best_s, best_p, fix_lin, rounds)
    hdr, nxz, grps, best_s, best_p, pv_b = _lock_side_and_persp(hdr, nxz, grps, sheath, refs, h_ctx, *args, pv_b, best_s, best_p, fix_lin, True)
    hdr, nxz, grps, best_s, best_p, pv_b = _lock_rear_360(hdr, nxz, grps, sheath, refs, h_ctx, *args, pv_b, best_s, best_p, fix_lin, True)

    n_top = ys_n > 68.0
    for sc_ok in (1.0, 0.85, 0.65, 0.35, 0.0):
        c_nxz = nxz.copy()
        c_nxz[n_top] = (1.0 - sc_ok) * def_nxz[n_top] + sc_ok * nxz[n_top]
        os.environ['FACE_HULL_JSON'] = fhull.dumps_hull({**h_ctx, 'head_dr': hdr.tolist(), 'neck_xz': c_nxz.tolist()})
        if face_built.fast_hull_clear():
            nxz = c_nxz
            break
    return (tuple(tuple(round(float(hdr[r, c]), 3) for c in range(len(hdr[0]))) for r in range(len(hdr))),
            tuple(tuple((round(float(nxz[r, c, 0]), 2), round(float(nxz[r, c, 1]), 2)) for c in range(len(nxz[0]))) for r in range(len(nxz))))
