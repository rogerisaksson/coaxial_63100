#!/usr/bin/env python3
"""Full-resolution (896x1200 / 1200x896) unmasked raytracing + pixel-diff CAD optimizer."""
import concurrent.futures
import json
import os
import pathlib
from typing import Any

import numpy as np

from tools.render import (
    face_built, face_carve, face_contours as fcon, face_cranium as fc,
    face_delta, face_honeycomb as fh, face_hull as fhull, face_laid as fl, face_ray as fr,
)


def _fast_profile_cap(loop_name, eval_abs_x, bezel_mm=2.5, chamfer_mm=0.65):
    cam = fr.CANONICAL_CAMERAS['profile90']
    uv = [(-float(px), -float(py)) for px, py in fcon.active_loop(loop_name)]
    surf = lambda u, v, off: fr.canon_to_head(fr.project_profile_ray_x(-float(u), -float(v), 1, cam, eval_abs_x, float(off) / fr.SCALE_M)[0])
    cap = fh.curved_poly_plate(uv, surf, wall=bezel_mm * fr.SCALE_M, chamfer=chamfer_mm * fr.SCALE_M,
                               corner_r=0.0, col=fh.AMBER_GOLD, rim_col=fh.AMBER_DARK, step=16.0)
    return cap + fh.mirror_x(cap)


def rebuild_single_group(grps, pid, fast_proxy=False):
    """Rebuild ONLY `pid`'s `.stl`-quantized triangles."""
    ht_v, p = np.asarray(fl.anchors()[2], float), fc.cfg()
    if fast_proxy and pid in ('jaw-front-hex', 'jaw-rear-hex', 'temple-hex-bezels'):
        if pid == 'temple-hex-bezels':
            tris = _fast_profile_cap('PROF_TEMPLE_LOOP', fc.eval_temple_abs_x, bezel_mm=1.42, chamfer_mm=0.52)
        elif pid == 'jaw-front-hex':
            tris = _fast_profile_cap('PROF_JAW_FRONT_LOOP', fcon.eval_jaw_front_x, bezel_mm=1.95, chamfer_mm=0.52)
        else:
            tris = _fast_profile_cap('PROF_JAW_REAR_LOOP', lambda y, z: min(
                fhull.front_jaw_max_x(y, z) - 0.12,
                p['jaw_r_x0'] + p['jaw_r_sy'] * float(y) - p['jaw_r_sz'] * float(z) + fhull.panel_offset('jaw-rear-hex', y, z)), bezel_mm=2.10, chamfer_mm=0.52)
        grps[pid] = face_delta.pack_id_group(pid, tris, ht_v)
        return
    if pid == 'temple-hex-bezels':
        tl, tr = fc.temporal_amber_vents()
        grps[pid] = face_delta.pack_id_group(pid, tl + tr, ht_v)
    elif pid == 'forehead-wings':
        grps[pid] = face_delta.pack_id_group(pid, fl._front_plate('TIARA_LOOP', fl._eval_wing_z, fh.AMBER_GOLD, fh.AMBER_DARK, 0.68, 3.6), ht_v)
    elif pid == 'face-outer-cheeks':
        zy = fr.canon_tris_to_head(fl.csg.extrude(fl.csg.filleted2d(fl.csg.box2d((148.0, 153.0), (-26.0, -22.0)), r=0.6), a=55.8, b=57.8, axis=0, colour=fh.CHASSIS_CF, chamfer=0.3).tris())
        grps[pid] = face_delta.pack_id_group(pid, fl._front_plate('OCHEEK_L_LOOP', fl._eval_ocheek_z, bd=0.72, th=3.7) + zy + fl._front_plate('OCHEEK_R_LOOP', fl._eval_ocheek_z, bd=0.72, th=3.7) + fh.mirror_x(zy), ht_v)
    elif pid == 'face-oled-mouth':
        op = fr.camera_ray_planar_panel(fcon.active_loop('MOUTH_LOOP'), fr.CANONICAL_CAMERAS['front'], fl._eval_mouth_z, bevel_outset_px=2.8, bevel_depth_mm=0.66, thickness_mm=2.8, col=fh.MOUTH_SCREEN, rim_col=fh.CRANIUM_RIM, corner_r_px=6.6)
        grps[pid] = face_delta.pack_id_group(pid, op + fl.oled_mouth_pixels(0.0, 0.0), ht_v)
    elif pid in ('forehead-diamond', 'face-brows'):
        fp = fl.frontal_panels()
        grps[pid] = face_delta.pack_id_group(pid, {'forehead-diamond': fp['brow_amber_keystone'], 'face-brows': fp['forehead_porcelain_L'] + fp['forehead_porcelain_R']}[pid], ht_v)
    elif pid in ('face-inner-cheeks', 'face-nose'):
        cp = fl.cheek_and_nose_panels()
        grps[pid] = face_delta.pack_id_group(pid, {'face-inner-cheeks': cp['cheek_inner_L'] + cp['cheek_inner_R'], 'face-nose': cp['nose_porcelain']}[pid], ht_v)
    elif pid in ('jaw-chin-cap', 'jaw-front-hex', 'jaw-rear-hex'):
        jp = fl.lower_jaw_and_oled_panels()
        grps[pid] = face_delta.pack_id_group(pid, {'jaw-chin-cap': jp['chin_amber_bridge'], 'jaw-front-hex': jp['jaw_amber_front_L'] + jp['jaw_amber_front_R'],
                                                   'jaw-rear-hex': jp['jaw_amber_rear_L'] + jp['jaw_amber_rear_R']}[pid], ht_v)


def triangulate_panel_grids(refs, cam_deltas, front_outer_x, tuning, loops, head_dr, neck_xz, init_grids=None, only_pids=None):
    """Solve smooth `6x6` stereo panel depth grids across native full-res views."""
    gn = fhull.GRID_N
    os.environ['FACE_TUNE_JSON'] = json.dumps(tuning)
    if loops:
        os.environ['FACE_CONTOURS_JSON'] = fcon.dumps_contours(loops)
    grids: dict[str, list[list[float]]] = {
        k: ([[0.0] * gn for _ in range(gn)] if (not init_grids or (only_pids and k in only_pids))
            else [[float(x) for x in r] for r in init_grids[k]])
        for k in fhull.PANEL_ORDER
    }
    h_base = {'cam_deltas': cam_deltas, 'front_outer_x': front_outer_x, 'head_dr': head_dr, 'neck_xz': neck_xz}
    os.environ['FACE_HULL_JSON'] = fhull.dumps_hull({**h_base, 'panel_grids': grids})
    grps = face_delta.build_id_groups_stl()

    for pid in (only_pids or ('temple-hex-bezels', 'jaw-rear-hex', 'jaw-front-hex', 'face-outer-cheeks', 'forehead-wings', 'face-oled-mouth')):
        is_proxy = pid in ('temple-hex-bezels', 'jaw-rear-hex', 'jaw-front-hex')
        if is_proxy:
            rebuild_single_group(grps, pid, fast_proxy=True)
        s0, cur_pe, pv0 = face_carve.fast_panel_err(grps, refs, pid)
        f_p0, w_f = pv0['FRONT'].get(pid, 0), (26.0 if pid in ('face-outer-cheeks', 'temple-hex-bezels') else 8.0)
        w_fsil = 0.5 if pid in ('forehead-wings', 'face-oled-mouth') else 12.0
        cur_loss = cur_pe + w_fsil * pv0['FRONT']['_sil'] + 28.0 * pv0['SIDE'].get(pid, 0) + w_f * f_p0 + 0.08 * s0
        for step_mm in (1.20, 0.45):
            for br in (0, 3):
                for bc in (0, 3):
                    for sgn in (-1.0, 1.0):
                        pg_c = {k: [list(rw) for rw in v] for k, v in grids.items()}
                        for dr in (0, 1, 2):
                            for dc in (0, 1, 2):
                                pg_c[pid][br + dr][bc + dc] = max(-4.5, min(4.5, round(pg_c[pid][br + dr][bc + dc] + sgn * step_mm, 3)))
                        os.environ['FACE_HULL_JSON'] = fhull.dumps_hull({**h_base, 'panel_grids': pg_c})
                        g_try = dict(grps)
                        rebuild_single_group(g_try, pid, fast_proxy=is_proxy)
                        s_c, pe_c, pvc = face_carve.fast_panel_err(g_try, refs, pid)
                        cl = pe_c + w_fsil * pvc['FRONT']['_sil'] + 28.0 * pvc['SIDE'].get(pid, 0) + w_f * pvc['FRONT'].get(pid, 0) + 0.08 * s_c
                        if cl < cur_loss - 4.0 and pvc['FRONT'].get(pid, 0) <= f_p0 + 10:
                            grids, grps, cur_pe, cur_loss = pg_c, g_try, pe_c, cl
        if is_proxy:
            os.environ['FACE_HULL_JSON'] = fhull.dumps_hull({**h_base, 'panel_grids': grids})
            rebuild_single_group(grps, pid, fast_proxy=False)
            _, cur_pe, _ = face_carve.fast_panel_err(grps, refs, pid)
        print(f'  [grid_cd:{pid}] post-polish err_px={cur_pe}', flush=True)

    out_g = {k: tuple(tuple(round(float(x), 3) for x in r) for r in v) for k, v in grids.items()}
    os.environ['FACE_HULL_JSON'] = fhull.dumps_hull({**h_base, 'panel_grids': out_g})
    return out_g


def extract_front_outer_x(f_fg, cd_f, t_seg=None, cd_s=None):
    """Extract `108`-row `(py_front, half_w_sil_px, half_w_t_in_px, half_w_t_out_px, z_ant_mm)`."""
    fcam, scam = fr.CANONICAL_CAMERAS['front'], fr.CANONICAL_CAMERAS['profile90']
    mid_u = 0.5 * float(fcam['w']) - 0.5 + float(cd_f[4])
    prof_yz = [fr.project_profile_ray_x(px, py, 1, scam, lambda _y, _z: 62.0, 0.0, cd_s)[0][1:]
               for px, py in fcon.active_loop('PROF_TEMPLE_LOOP')]
    np_yz = len(prof_yz)
    pts = []
    for py_f in np.linspace(36.0, 935.0, 104):
        py = int(np.clip(round(float(py_f)), 0, f_fg.shape[0] - 1))
        fy_est = float(fr.project_ray_z(mid_u, float(py) + 0.5, fcam, lambda _x, _y: 0.0, 0.0, cd_f)[0][1])
        z_hits = [prof_yz[i][1] + (fy_est - prof_yz[i][0]) / (prof_yz[(i + 1) % np_yz][0] - prof_yz[i][0])
                  * (prof_yz[(i + 1) % np_yz][1] - prof_yz[i][1])
                  for i in range(np_yz) if (prof_yz[i][0] <= fy_est <= prof_yz[(i + 1) % np_yz][0]
                                            or prof_yz[(i + 1) % np_yz][0] <= fy_est <= prof_yz[i][0])
                  and abs(prof_yz[(i + 1) % np_yz][0] - prof_yz[i][0]) > 1e-6]
        za = float(max(z_hits)) if z_hits else -12.0
        xs = np.flatnonzero(f_fg[py])
        if len(xs) >= 4:
            hw_s = 0.5 * ((mid_u - float(xs[0])) + (float(xs[-1]) - mid_u))
            hw_i, hw_o = max(40.0, hw_s - 45.0), hw_s
            if t_seg is not None:
                xl, xr = np.flatnonzero(t_seg[py, :450]), np.flatnonzero(t_seg[py, 450:]) + 450
                if len(xl) >= 2 and len(xr) >= 2:
                    hw_o = 0.5 * ((mid_u - float(xl[0])) + (float(xr[-1]) - mid_u))
                    hw_i = max(180.0, min(hw_o - 2.0, 0.5 * ((mid_u - float(xl[-1])) + (float(xr[0]) - mid_u))))
            pts.append([round(float(py), 1), round(hw_s, 2), round(hw_i, 2), round(hw_o, 2), round(za, 1)])
    if len(pts) < 12:
        return fhull.DEFAULT_OUTER_X
    from scipy import ndimage
    arr = np.asarray(pts, float)
    for col_k in (1, 2, 3):
        arr[:, col_k] = np.round(ndimage.gaussian_filter1d(arr[:, col_k], 1.6, mode='nearest'), 2)
    return tuple(tuple(float(v) for v in r) for r in arr)


def polish_front_epipolar(refs, cam_deltas, front_outer_x, tuning, loops, hdr, nxz, grids):
    """Sub-pixel GPU polish of `front_outer_x` pixel half-widths against `FRONT` temple & mandible."""
    from scipy import ndimage
    lr, ht = face_carve.ensure_lr(), fl.anchors()[2]
    f_fg, t_mr = refs['FRONT']['fg'], refs['FRONT']['seg']['temple-hex-bezels']
    ti = list(face_delta.PART_IDS.keys()).index('temple-hex-bezels') + 1
    fo = np.asarray(front_outer_x, float).copy()
    os.environ['FACE_TUNE_JSON'] = json.dumps(tuning)
    if loops:
        os.environ['FACE_CONTOURS_JSON'] = fcon.dumps_contours(loops)
    os.environ['FACE_HULL_JSON'] = fhull.dumps_hull({
        'cam_deltas': cam_deltas, 'front_outer_x': fo.tolist(), 'head_dr': hdr, 'neck_xz': nxz, 'panel_grids': grids})
    grps = face_delta.build_id_groups_stl()
    st_cl = np.ones((13, 3), dtype=bool)
    for it in range(7):
        pos_id, mat_id = face_carve.stack_groups(grps)
        _r, cf, pidx, _i = face_delta._raster_canonical(None, pos_id, mat_id, 'front', ht, lr)
        t_mc = fr.fill_interior_holes(np.asarray(ndimage.binary_closing(pidx == ti, structure=st_cl), dtype=bool), 38000)
        for k in range(len(fo)):
            py = int(np.clip(round(float(fo[k, 0])), 0, f_fg.shape[0] - 1))
            xr_s, xc_s = np.flatnonzero(f_fg[py]), np.flatnonzero(cf[py])
            if len(xr_s) >= 4 and len(xc_s) >= 4 and 42 <= py <= 915:
                fo[k, 1] = max(60.0, fo[k, 1] + 0.55 * 0.5 * (float(xc_s[0] - xr_s[0]) + float(xr_s[-1] - xc_s[-1])))
            rl, cl = np.flatnonzero(t_mr[py, :450]), np.flatnonzero(t_mc[py, :450])
            rr, cr = np.flatnonzero(t_mr[py, 450:]), np.flatnonzero(t_mc[py, 450:])
            if len(rl) >= 2 and len(cl) >= 2 and len(rr) >= 2 and len(cr) >= 2:
                fo[k, 3] = max(190.0, fo[k, 3] + 0.55 * 0.5 * (float(cl[0] - rl[0]) + float(rr[-1] - cr[-1])))
                fo[k, 2] = max(160.0, min(fo[k, 3] - 3.0, fo[k, 2] + 0.55 * 0.5 * (float(cl[-1] - rl[-1]) + float(rr[0] - cr[0]))))
                fo[k, 1] = max(fo[k, 1], fo[k, 3])
        for col_k in (1, 2, 3):
            fo[:, col_k] = ndimage.gaussian_filter1d(fo[:, col_k], 1.4 if col_k == 1 else 0.72, mode='nearest')
        fo[:, 2] = np.minimum(fo[:, 2], fo[:, 3] - 2.5)
        fo[:, 1] = np.maximum(fo[:, 1], fo[:, 3])
        os.environ['FACE_HULL_JSON'] = fhull.dumps_hull({
            'cam_deltas': cam_deltas, 'front_outer_x': np.round(fo, 2).tolist(), 'head_dr': hdr, 'neck_xz': nxz, 'panel_grids': grids})
        for gp in ('temple-hex-bezels', 'jaw-front-hex', 'jaw-rear-hex'):
            rebuild_single_group(grps, gp, fast_proxy=(it < 4))
    return tuple(tuple(round(float(v), 2) for v in r) for r in fo)


SEARCH_SPACE = {
    'wing_zi': (-2.5, 2.5, 0.4), 'wing_zo': (0.08, 0.18, 0.008), 'wing_sx': (1.25, 1.65, 0.04),
    'ocheek_z0': (88.0, 99.0, 0.9), 'ocheek_sx': (1.35, 1.68, 0.03),
    'oled_z0': (-2.5, 2.5, 0.4), 'oled_sx': (0.80, 1.15, 0.04), 'oled_sy': (1.10, 1.60, 0.04),
    'jaw_f_x0': (10.5, 19.5, 0.6), 'jaw_f_sy': (0.32, 0.48, 0.02), 'jaw_f_sz': (0.44, 0.64, 0.03),
    'jaw_r_x0': (3.5, 18.5, 0.9), 'jaw_r_sy': (0.28, 0.56, 0.03), 'jaw_r_sz': (0.26, 0.46, 0.03),
    'temp_xa': (65.0, 68.70, 0.35), 'temp_xp': (65.0, 69.2, 0.35),
    'cam_s_dz': (-2.5, 2.5, 0.3), 'cam_p_dx': (-3.5, 3.5, 0.4), 'cam_b_dy': (-2.5, 2.5, 0.4),
}

STAGE_GROUPS = (
    ('temple-hex-bezels', ('temp_xa', 'temp_xp'), ('temple-hex-bezels',)),
    ('forehead-wings', ('wing_zi', 'wing_zo', 'wing_sx'), ('forehead-wings',)),
    ('jaw-rear-hex', ('jaw_r_x0', 'jaw_r_sy', 'jaw_r_sz'), ('jaw-rear-hex',)),
    ('jaw-front-hex', ('jaw_f_x0', 'jaw_f_sy', 'jaw_f_sz'), ('jaw-front-hex',)),
    ('face-outer-cheeks', ('ocheek_z0', 'ocheek_sx'), ('face-outer-cheeks',)),
    ('face-oled-mouth', ('oled_z0', 'oled_sx', 'oled_sy'), ('face-oled-mouth',)),
    ('assembly-camera-align', ('cam_s_dz', 'cam_p_dx', 'cam_b_dy'), tuple(face_delta.PART_IDS.keys())),
)


def _stage_score(rep, focus_panels):
    pp, s, vw = rep['per_panel'], rep['summary'], rep['views']
    foc = [pp[p]['iou'] + 18.0 * pp[p]['match'] - 2e-6 * pp[p]['occluded_px'] for p in focus_panels if p in pp]
    f_val = float(sum(foc) / len(foc)) if foc else float(s['mean_panel_iou'])
    f_vm, s_vm = float(vw['FRONT']['view_match']), float(vw['SIDE']['view_match'])
    pb_min = float(min(vw['PERSP']['view_match'], vw['BACK']['view_match']))
    pen = (-40.0 if pb_min < 0.995 else 0.0) + (-25.0 if s_vm < 0.999 else 0.0)
    return (0.30 * f_val + 0.70 * (float(s['objective']) + 42.0 * f_vm + 34.0 * s_vm
                                   + 14.0 * float(vw['FRONT']['sil_match']) + 12.0 * float(vw['SIDE']['sil_match'])
                                   + 8.0 * pb_min) + pen)


def _mesh_worker(pack):
    tag_val, cand, hull_json, loops_json = pack
    os.environ['COAXIAL_ROBOT'] = 'gynoid_mesomorph_medium_coax'
    os.environ['OPENBLAS_NUM_THREADS'] = '1'
    os.environ['FACE_TUNE_JSON'] = json.dumps(cand)
    os.environ['FACE_HULL_JSON'] = hull_json
    if loops_json:
        os.environ['FACE_CONTOURS_JSON'] = loops_json
    else:
        os.environ.pop('FACE_CONTOURS_JSON', None)
    ht_y = float(fl.anchors()[2][1])
    grps = face_delta.build_id_groups_stl()
    ear_x = 1e3 * float(grps['temple-ear-pods'][0][:, 0].max())
    cran_x = 1e3 * float(grps['temple-hex-bezels'][0][:, 0].max())
    face_x = 1e3 * float(grps['face-outer-cheeks'][0][:, 0].max())
    crown_mm = round(1e3 * float(grps['crown-top-plates'][0][:, 1].max() - ht_y), 2)
    if not (220.0 <= crown_mm <= 229.0 and 84.0 <= cran_x <= 89.5 and 74.0 <= face_x <= 82.0 and 86.0 <= ear_x <= 91.5):
        return tag_val, cand, hull_json, None
    return tag_val, cand, hull_json, (None, face_carve.stack_groups(grps), True,
                                      {'ok': True, 'crown_mm': crown_mm}, (cran_x, face_x, ear_x))


def _eval_cand_gpu(ref_dir, tag_val, cand, hull_json, pre, focus_panels):
    if pre is None:
        return tag_val, cand, hull_json, False, -99.0, {}, {}, {}
    os.environ['FACE_TUNE_JSON'] = json.dumps(cand)
    os.environ['FACE_HULL_JSON'] = hull_json
    rep = face_delta.measure(ref_dir, out_dir=None, tag='cand', use_gpu=True, fast_check=True, prebuilt=pre)
    return tag_val, cand, hull_json, bool(rep['checked_ok']), _stage_score(rep, focus_panels), rep['summary'], rep['per_panel'], rep['views']


def converge(ref_dir, out_dir, max_rounds=1, jobs=12, target_front=0.9999, target_other=0.995):
    """Run full-resolution (`896x1200` / `1200x896`) UNMASKED raytracing + pixel-diff convergence."""
    out_p, ref_p = pathlib.Path(out_dir), pathlib.Path(ref_dir)
    out_p.mkdir(parents=True, exist_ok=True)
    loops, holes_dict = fcon.extract_reference_contours_and_holes(ref_p)
    fcon.bake_contours_into_source(loops, holes_dict)
    loops_json = fcon.dumps_contours(loops)
    os.environ['FACE_CONTOURS_JSON'] = loops_json

    refs = face_delta._get_refs(ref_p)
    best = dict(fc.TUNING)
    os.environ['FACE_TUNE_JSON'] = json.dumps(best)
    rep0 = face_delta.measure(ref_dir, out_dir=None, tag='r0', use_gpu=True, fast_check=True)
    history: list[dict[str, Any]] = [{'stage': 'r0_init', 'summary': rep0['summary'], 'views': rep0['views']}]
    print(f'[phase0:init] ok={rep0["checked_ok"]} FRONT={rep0["views"]["FRONT"]["view_match"] * 100:.4f}% '
          f'min_other={min(rep0["views"][k]["view_match"] for k in ("SIDE", "PERSP", "BACK")) * 100:.4f}%', flush=True)

    hs0 = fhull.hull_state()
    if hs0['neck_xz']:
        cam_deltas = dict(hs0['cam_deltas'])
        fo0 = extract_front_outer_x(refs['FRONT']['fg'], cam_deltas['front'], refs['FRONT']['seg'].get('temple-hex-bezels'), cam_deltas['profile90'])
        init_pg = {k: (tuple((0.0,) * fhull.GRID_N for _ in range(fhull.GRID_N)) if k in ('face-oled-mouth', 'jaw-front-hex', 'jaw-rear-hex', 'temple-hex-bezels') else hs0['panel_grids'][k]) for k in fhull.PANEL_ORDER}
        hdr_t, nxz_t = face_carve.carve_and_sculpt_hull(refs, cam_deltas, fo0, best, init_pg, loops, rounds=8, init_hull=(hs0['head_dr'], hs0['neck_xz']))
        panel_grids = triangulate_panel_grids(refs, cam_deltas, fo0, best, loops, hdr_t, nxz_t,
                                              init_grids=init_pg, only_pids=('temple-hex-bezels', 'jaw-rear-hex', 'jaw-front-hex', 'forehead-wings', 'face-oled-mouth'))
        front_outer_x = polish_front_epipolar(refs, cam_deltas, fo0, best, loops, hdr_t, nxz_t, panel_grids)
    else:
        cam_deltas = dict(fhull.CAM_DELTAS)
        front_outer_x = extract_front_outer_x(refs['FRONT']['fg'], cam_deltas['front'], refs['FRONT']['seg'].get('temple-hex-bezels'), cam_deltas['profile90'])
        zero_pg = {k: tuple((0.0,) * fhull.GRID_N for _ in range(fhull.GRID_N)) for k in fhull.PANEL_ORDER}
        zhdr = tuple((0.0,) * fhull.HEAD_NPHI for _ in range(fhull.HEAD_NY))
        front_outer_x = polish_front_epipolar(refs, cam_deltas, front_outer_x, best, loops, zhdr, hs0['neck_xz'], zero_pg)
        hdr_t, nxz_t = face_carve.carve_and_sculpt_hull(refs, cam_deltas, front_outer_x, best, zero_pg, loops, rounds=7)
        panel_grids = triangulate_panel_grids(refs, cam_deltas, front_outer_x, best, loops, hdr_t, nxz_t)
        front_outer_x = polish_front_epipolar(refs, cam_deltas, front_outer_x, best, loops, hdr_t, nxz_t, panel_grids)

    hull_data: dict[str, Any] = {'cam_deltas': cam_deltas, 'front_outer_x': front_outer_x,
                                 'head_dr': hdr_t, 'neck_xz': nxz_t, 'panel_grids': panel_grids}
    hull_json = fhull.dumps_hull(hull_data)
    os.environ['FACE_HULL_JSON'] = hull_json
    rep_rt = face_delta.measure(ref_dir, out_dir=None, tag='raytraced', use_gpu=True, fast_check=True)
    history.append({'stage': 'raytraced_hull_and_stereo', 'summary': rep_rt['summary'], 'views': rep_rt['views']})
    print(f'[phase2:raytraced] ok={rep_rt["checked_ok"]} FRONT={rep_rt["views"]["FRONT"]["view_match"] * 100:.4f}% '
          f'SIDE={rep_rt["views"]["SIDE"]["view_match"] * 100:.4f}% '
          f'min_other={min(rep_rt["views"][k]["view_match"] for k in ("PERSP", "BACK")) * 100:.4f}%', flush=True)

    step_idx, scales = 0, (1.0, 0.40)
    if int(max_rounds) > 0:
        with concurrent.futures.ProcessPoolExecutor(max_workers=int(jobs)) as pool:
            for rnd in range(int(max_rounds)):
                scale, rnd_start = scales[min(rnd, len(scales) - 1)], dict(best)
                for st_name, keys, focus_panels in STAGE_GROUPS:
                    _, _, _, pre0 = _mesh_worker(('b', best, hull_json, loops_json))
                    _, _, _, ok0, sc0, _sm0, _, vw0 = _eval_cand_gpu(str(ref_dir), 'b', best, hull_json, pre0, focus_panels)
                    if (ok0 and vw0.get('FRONT', {}).get('view_match', 0.0) >= float(target_front)
                            and vw0.get('SIDE', {}).get('view_match', 0.0) >= 0.999
                            and min(vw0[k]['view_match'] for k in ('PERSP', 'BACK')) >= float(target_other)):
                        break
                    best_sc = sc0 if ok0 else -99.0
                    cands = [(k, {**best, k: nv}, hull_json, loops_json) for k in keys
                             for lo, hi, st in (SEARCH_SPACE[k],) for mul in (-1.0, -0.45, 0.45, 1.0)
                             for nv in (max(lo, min(hi, round(best[k] + mul * st * scale, 6))),) if abs(nv - best[k]) > 1e-7]
                    if not cands:
                        continue
                    meshes = list(pool.map(_mesh_worker, cands))
                    evals = [_eval_cand_gpu(str(ref_dir), tv, cd, hj, pr, focus_panels) for tv, cd, hj, pr in meshes]
                    imp: list[Any] = [(cd, sc, sm, vw) for _tv, cd, _hj, ok, sc, sm, _, vw in evals if ok and sc > best_sc + 1e-5]
                    if imp:
                        imp.sort(key=lambda x: x[1], reverse=True)
                        best, _sc_w, sm_win, vw_win = imp[0]
                        step_idx += 1
                        print(f'[r{rnd + 1}:{st_name}] FRONT={vw_win["FRONT"]["view_match"] * 100:.4f}% '
                              f'SIDE={vw_win["SIDE"]["view_match"] * 100:.4f}% '
                              f'min_other={min(vw_win[k]["view_match"] for k in ("PERSP", "BACK")) * 100:.4f}%', flush=True)
                        history.append({'stage': f'r{rnd + 1}_{st_name}', 'summary': sm_win})
                os.environ['FACE_TUNE_JSON'] = json.dumps(best)
                if not face_built.fast_hull_clear():
                    best = rnd_start
                    os.environ['FACE_TUNE_JSON'] = json.dumps(best)
    fo2 = polish_front_epipolar(refs, hull_data['cam_deltas'], hull_data['front_outer_x'], best, loops, hull_data['head_dr'], hull_data['neck_xz'], hull_data['panel_grids'])
    hdr2, nxz2 = face_carve.carve_and_sculpt_hull(
        refs, hull_data['cam_deltas'], fo2, best, hull_data['panel_grids'], loops, rounds=5, init_hull=(hull_data['head_dr'], hull_data['neck_xz']))
    fo2 = polish_front_epipolar(refs, hull_data['cam_deltas'], fo2, best, loops, hdr2, nxz2, hull_data['panel_grids'])
    cand_h = {**hull_data, 'front_outer_x': fo2, 'head_dr': hdr2, 'neck_xz': nxz2}
    os.environ['FACE_HULL_JSON'] = fhull.dumps_hull(cand_h)
    if face_built.fast_hull_clear():
        hull_data = cand_h

    os.environ['FACE_HULL_JSON'] = fhull.dumps_hull(hull_data)
    os.environ['FACE_TUNE_JSON'] = json.dumps(best)
    rep_fin = face_delta.measure(ref_dir, out_dir=out_dir, tag='final')
    (out_p / 'delta_final.json').write_text(json.dumps(rep_fin, indent=2), encoding='utf-8')
    history.append({'stage': 'final', 'summary': rep_fin['summary'], 'views': rep_fin['views']})
    for k_env in ('FACE_TUNE_JSON', 'FACE_HULL_JSON', 'FACE_CONTOURS_JSON'):
        os.environ.pop(k_env, None)
    (out_p / 'convergence_history.json').write_text(
        json.dumps({'best_tuning': best, 'steps_accepted': step_idx, 'history': history}, indent=2), encoding='utf-8')
    return {'tuning': best, 'hull': hull_data, 'loops': loops, 'holes': holes_dict}, history
