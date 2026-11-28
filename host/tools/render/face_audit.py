"""Multi-modal full-resolution CAD discrepancy auditor (gaps, intrusions, bezels, leaks and ROIs)."""
import os
import sys
from typing import Any

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np  # noqa: E402
from scipy import ndimage  # noqa: E402

from tools.render import face_contours as fcon, face_laid as fl, face_ray as fr  # noqa: E402


def mat_classes(rgb, fg, is_cad=False):
    """Classify pixels into `0=bg`, `1=dark_seam/hex`, `2=graphite/crown`, `3=amber`, `4=porcelain`."""
    r, g, b = rgb[..., 0].astype(float), rgb[..., 1].astype(float), rgb[..., 2].astype(float)
    lum = 0.299 * r + 0.587 * g + 0.114 * b
    am_lo, wh_lo, gr_lo = (52.0, 108.0, 22.0) if is_cad else (66.0, 128.0, 42.0)
    am = fg & (r > am_lo) & ((r - b) > (24.0 if is_cad else 28.0)) & (g > 24.0) & (r > g * 1.08)
    wh = fg & (~am) & (lum > wh_lo) & ((r - b) < 38.0)
    gr = fg & (~am) & (~wh) & (lum > gr_lo)
    dk = fg & (~am) & (~wh) & (~gr)
    return np.where(wh, 4, np.where(am, 3, np.where(gr, 2, np.where(dk, 1, 0)))).astype(np.uint8), lum


def _large_comps(mask, min_px=320):
    res_l: Any = ndimage.label(mask)
    lbl, num = np.asarray(res_l[0], dtype=np.int64), int(res_l[1])
    if num <= 0:
        return np.zeros_like(mask)
    cnt = np.bincount(lbl.ravel())
    keep = [i for i in range(1, num + 1) if cnt[i] >= int(min_px)]
    return np.isin(lbl, keep) if keep else np.zeros_like(mask)


def _crown_and_brows_aux(rgba, preset_id, base_seg):
    """Extract auxiliary ground-truth masks for crown tiles, side brows/cheeks and front mandible."""
    h, w = rgba.shape[:2]
    r, g, b, a = rgba[..., 0].astype(float), rgba[..., 1].astype(float), rgba[..., 2].astype(float), rgba[..., 3]
    fg = a > 127
    lum = 0.299 * r + 0.587 * g + 0.114 * b
    is_wh = fg & (lum > 98.0) & ((r - b) < 36.0)
    is_am = fg & (r > 58.0) & ((r - b) > 22.0) & (g > 22.0) & (r > g * 1.08)
    yy, xx = np.mgrid[0:h, 0:w]
    aux = {}
    if preset_id == 'front':
        excl = base_seg['forehead-wings'] | base_seg['forehead-diamond'] | base_seg['temple-hex-bezels']
        aux['crown'] = _large_comps(fg & (lum > 54.0) & ((r - b) < 22.0) & (yy <= 242) & (~excl), 400)
        jf_el = is_am & (yy >= 625) & (yy <= 922) & (~base_seg['jaw-chin-cap']) & (~base_seg['face-oled-mouth'])
        aux['mandible'] = fr.fill_interior_holes(_large_comps(jf_el & (xx < 415), 280), 18000) | fr.fill_interior_holes(_large_comps(jf_el & (xx > 481), 280), 18000)
    elif preset_id == 'profile90':
        aux['crown'] = _large_comps(fg & (lum > 54.0) & ((r - b) < 22.0) & (yy <= 202) & (xx >= 325) & (xx <= 805) & (~base_seg['forehead-wings']) & (~base_seg['temple-hex-bezels']), 400)
        wh_cut = is_wh & (~np.asarray(ndimage.grey_dilation(lum, size=(3, 3)) - lum > 34.0, dtype=bool)) & (lum > 114.0)
        aux['face-brows'] = fr.fill_interior_holes(np.asarray(ndimage.binary_dilation(fcon.seeded_comp(wh_cut & (yy <= 338), 835, 290, 22, 0)), dtype=bool) & is_wh & (yy <= 346), 1600)
    elif preset_id == 'threeQuarter':
        am_up = is_am & (xx >= 618) & (yy <= 295) & (~base_seg['temple-hex-bezels'])
        aux['forehead-wings'] = fcon.seeded_comp(am_up, 765, 185, 22, 1600)
        aux['crown'] = _large_comps(fg & (lum > 54.0) & ((r - b) < 22.0) & (yy <= 198) & (~aux['forehead-wings']) & (~base_seg['temple-hex-bezels']), 400)
        wh_cut = is_wh & (~np.asarray(ndimage.grey_dilation(lum, size=(3, 3)) - lum > 34.0, dtype=bool)) & (lum > 114.0)
        aux['face-brows'] = fr.fill_interior_holes(np.asarray(ndimage.binary_dilation(fcon.seeded_comp(wh_cut & (yy <= 338) & (xx <= 882), 775, 280, 24, 0)), dtype=bool) & is_wh & (yy <= 345), 1600)
    return aux


def mesh_and_seam_audit():
    """Audit 3D porcelain cap sliver/flipped triangles, tiara-crown gap, and occipital-crown step."""
    from tools.render import face_cranium as fc_mod, face_honeycomb as fh
    slivers = flipped = 0
    for ts in (*fl.frontal_panels().values(), *fl.cheek_and_nose_panels().values(), fl.lower_jaw_and_oled_panels()['flexible_oled_panel']):
        cap = [t for t, c in ts if c in (fh.SHELL_CF, fh.MOUTH_SCREEN)]
        if not cap:
            continue
        arr = np.asarray(cap, float)
        e0, e1, e2 = arr[:, 1] - arr[:, 0], arr[:, 2] - arr[:, 1], arr[:, 0] - arr[:, 2]
        lm = np.maximum(np.linalg.norm(e0, axis=1), np.maximum(np.linalg.norm(e1, axis=1), np.linalg.norm(e2, axis=1)))
        nrm = np.cross(e0, arr[:, 2] - arr[:, 0])
        nl = np.maximum(np.linalg.norm(nrm, axis=1), 1e-18)
        asp = lm / np.maximum(nl / np.maximum(lm, 1e-18), 1e-18)
        nz, ny = nrm[:, 2] / nl, nrm[:, 1] / nl
        cx_m, cy_m = np.abs(arr[:, :, 0].mean(axis=1)), arr[:, :, 1].mean(axis=1)
        slivers += int(((cy_m > 0.068) & (asp > 650.0)).sum())
        flipped += int(((cx_m < 0.044) & (cy_m > 0.068) & (ny > -0.65) & (nz < -0.50)).sum())
    fcam = fr.CANONICAL_CAMERAS['front']
    t_pts = np.array([fr.project_ray_z(px, py, fcam, fl._eval_wing_z)[0] for px, py in fcon.active_loop('TIARA_LOOP')])
    tc_gaps = []
    for c_rad in (0.0, 0.18, 0.36, 0.54, 0.72, 0.86):
        cp, cn = fc_mod.sample_crown_dome_mm(fc_mod._s_seam(0, c_rad), c_rad)
        cb = cp + (fc_mod.cfg()['crown_off'] + 2.05) * cn
        near = t_pts[np.abs(t_pts[:, 0] - cb[0]) <= 5.5]
        if len(near):
            top = near[int(np.argmax(near[:, 1]))]
            tc_gaps.append(float(np.hypot(cb[1] - top[1], cb[2] - top[2])))
    occ_u = fr.head_to_canon(np.vstack([t for t, _ in fc_mod.occipital_panels()['occipital_upper_L']]))
    dome_a = fr.head_to_canon(np.vstack([t for t, _ in fc_mod.cranium_dome_plates()]))
    oy_hi = float(occ_u[:, 1].max())
    occ_seam_z = float(occ_u[occ_u[:, 1] >= oy_hi - 5.0, 2].min())
    dome_r = dome_a[dome_a[:, 2] < -55.0]
    dy_lo = float(dome_r[:, 1].min())
    dome_seam_z = float(dome_r[dome_r[:, 1] <= dy_lo + 6.0, 2].min())
    occ_step = max(0.0, occ_seam_z - dome_seam_z, (dome_seam_z - occ_seam_z) - 3.6)
    cp_p = fl.cheek_and_nose_panels()
    vn = fr.head_to_canon(np.vstack([t for t, _ in cp_p['nose_porcelain'] if _ == fh.SHELL_CF]))
    vi = fr.head_to_canon(np.vstack([t for t, _ in cp_p['cheek_inner_L'] if _ == fh.SHELL_CF]))
    in_steps = []
    for fy in (116.0, 124.0, 132.0, 140.0, 148.0):
        sn = vn[(vn[:, 0] > 5.8) & (np.abs(vn[:, 1] - fy) < 2.5), 2]
        si_m = vi[(vi[:, 0] < 14.5) & (np.abs(vi[:, 1] - fy) < 2.5), 2]
        if len(sn) and len(si_m):
            in_steps.append(abs(float(sn.min()) - float(si_m.max())))
    in_step = max(in_steps or [0.0])
    return {'porcelain_sliver_tris': slivers, 'porcelain_flipped_nrm': flipped,
            'tiara_crown_gap_mm': round(max(tc_gaps or [0.0]), 2), 'occipital_crown_step_mm': round(occ_step, 2),
            'icheek_nose_step_mm': round(float(in_step), 2)}


def audit_discrepancies(vname, preset_id, rgba, ref_rgb, ref_fg, cad_rgb, cad_fg, seg, part_idx, part_names):
    """Quantify seam gaps ('glapp'), intrusions, bezel bloat/folding, Z-blotches, leaks & eye/hose ROIs."""
    rc, rlum = mat_classes(ref_rgb, ref_fg, is_cad=False)
    cc, clum = mat_classes(cad_rgb, cad_fg, is_cad=True)
    aux = _crown_and_brows_aux(rgba, preset_id, seg)
    b_any = (rc != ndimage.grey_erosion(rc, size=(3, 3))) | (cc != ndimage.grey_erosion(cc, size=(3, 3)))
    core = (ref_fg | cad_fg) & (~b_any)

    p_union, p_borders = np.zeros_like(ref_fg), np.zeros_like(ref_fg)
    for mr in (*seg.values(), *aux.values()):
        p_union |= mr
        p_borders |= mr ^ np.asarray(ndimage.binary_erosion(mr), dtype=bool)
    seam_zone = np.asarray(ndimage.binary_dilation(p_borders, iterations=5), dtype=bool)
    gap_m = core & seam_zone & (rc >= 3) & (cc <= 1)
    intr_m = core & (((cc == 4) & (rc == 3)) | ((cc == 3) & (rc == 4)))
    plate_in = np.asarray(ndimage.binary_erosion(p_union, iterations=4), dtype=bool)
    leak_m = core & (((rlum < 30.0) & (clum > 118.0)) | (plate_in & (rlum > 110.0) & (clum < 32.0)))

    bez_m, bez_stats = np.zeros_like(ref_fg), {}
    for hp in ('temple-hex-bezels', 'jaw-front-hex', 'jaw-rear-hex'):
        mr_h = aux.get('mandible') if (vname == 'FRONT' and hp == 'jaw-front-hex') else seg.get(hp)
        if mr_h is not None and vname != 'BACK':
            roi = np.asarray(ndimage.binary_dilation(mr_h, iterations=3), dtype=bool)
            ra_h, ca_h = roi & (rc == 3), roi & (cc == 3)
            bez_m |= (ra_h ^ ca_h) & (~b_any)
            bez_stats[hp] = {'bezel_iou': round(float((ra_h & ca_h).sum() / max(1, (ra_h | ca_h).sum())), 4),
                             'thickness_ratio': round(float(ca_h.sum()) / max(1.0, float(ra_h.sum())), 3)}

    fw_m = seg.get('forehead-wings') if 'forehead-wings' in seg else aux.get('forehead-wings')
    fb_m = seg.get('face-brows') if 'face-brows' in seg else aux.get('face-brows')
    cr_m = aux.get('crown')
    tiara_gap = tiara_crown_gap = white_in_temple = jaw_fold = crown_fused = 0
    if fw_m is not None and fb_m is not None:
        cor = np.asarray(ndimage.binary_dilation(fw_m, iterations=5), dtype=bool) & np.asarray(ndimage.binary_dilation(fb_m, iterations=5), dtype=bool)
        tg_m = cor & (rc >= 3) & (cc <= 1) & (~b_any)
        tiara_gap = int(tg_m.sum())
        gap_m |= tg_m
    if fw_m is not None and cr_m is not None:
        ti_cr = part_names.index('crown-top-plates') + 1
        tc_cor = cr_m & np.asarray(ndimage.binary_dilation(fw_m, iterations=22), dtype=bool)
        tc_m = tc_cor & (part_idx != ti_cr) & (cc <= 1) & (~b_any)
        tiara_crown_gap = int(tc_m.sum())
        gap_m |= tc_m
    if 'temple-hex-bezels' in seg:
        w_t = np.asarray(ndimage.binary_dilation(seg['temple-hex-bezels'], iterations=2), dtype=bool) & (cc == 4) & (~b_any)
        white_in_temple = int(w_t.sum())
        intr_m |= w_t
    jf_ref = aux.get('mandible') if vname == 'FRONT' else seg.get('jaw-front-hex')
    jaw_over_porc = 0
    if jf_ref is not None and 'face-oled-mouth' in seg:
        ti_m = part_names.index('face-oled-mouth') + 1
        jf_m = jf_ref & (rc == 3) & (part_idx == ti_m) & (~b_any)
        jaw_fold = int(jf_m.sum())
        intr_m |= jf_m
    jaw_ids = [part_names.index(k) + 1 for k in ('jaw-front-hex', 'jaw-rear-hex') if k in part_names]
    if jaw_ids and vname != 'BACK':
        jop_m = (rc == 4) & np.isin(part_idx, jaw_ids) & (~b_any)
        jaw_over_porc = int(jop_m.sum())
        intr_m |= jop_m
    if cr_m is not None:
        cr_hull = fr.fill_interior_holes(np.asarray(ndimage.binary_closing(cr_m, iterations=6), dtype=bool), 4000)
        ti_cr = part_names.index('crown-top-plates') + 1
        cf_m = cr_hull & (~cr_m) & (rlum < 44.0) & (part_idx == ti_cr) & (~b_any)
        crown_fused = int(cf_m.sum())
        intr_m |= cf_m

    seam_steps = ([abs(fl._eval_wing_z(ax, 176.0) - fl._eval_brow_z(ax, 174.0)) for ax in (18.0, 34.0, 48.0, 58.0)]
                  + [abs(fl._eval_brow_z(ax, 150.0) - fl._eval_ocheek_z(ax, 148.0)) for ax in (38.0, 48.0, 58.0)]
                  + [abs(fl._eval_icheek_z(34.0, y) - fl._eval_ocheek_z(34.5, y)) for y in (116.0, 130.0, 142.0)])
    from tools.render import face_cranium as fc_mod
    tx_mid = fc_mod.eval_temple_abs_x(166.5, -17.5)
    tx_conv = min(tx_mid - fc_mod.eval_temple_abs_x(191.5, -17.5), tx_mid - fc_mod.eval_temple_abs_x(166.5, -50.5))
    feat: dict[str, Any] = {'seam_step_max_mm': round(float(max(seam_steps)), 3),
                            'temple_convexity_mm': round(float(tx_conv), 2),
                            'jaw_over_porcelain_px': jaw_over_porc}
    if vname == 'FRONT':
        feat.update(mesh_and_seam_audit())
    if vname == 'FRONT':
        for tag, cx, cy in (('L', 316, 506), ('R', 580, 506)):
            p_c = float(clum[cy - 7:cy + 8, cx - 7:cx + 8].min())
            sc_band = clum[cy - 14:cy + 15, cx - 28:cx + 29]
            r_o, sc_hi = float(sc_band.max()), float(np.percentile(sc_band, 90))
            feat[f'eye_{tag}_oled_hole_ok'] = bool(p_c < 38.0 and r_o > 55.0 and sc_hi < 155.0)
        ul_band = clum[752:776, 425:475]
        feat['upper_lip_visible_ok'] = bool(float(ul_band.max() - ul_band.min()) > 18.0)
        feat['crown_panel_count'] = len(fc_mod.cranium_dome_panels())
        ti_nt = part_names.index('neck-tubes-bust') + 1
        v_hp = clum[1035:1085, 380:516] - ndimage.gaussian_filter(clum[1035:1085, 380:516], sigma=(3.5, 0.5))
        band = (np.std(v_hp, axis=0) > 1.8) & ((part_idx[1035:1085, 380:516] == ti_nt).any(axis=0))
        res_h: Any = ndimage.label(band)
        feat['neck_conduit_count'] = sum(1 for s in ndimage.find_objects(res_h[0]) if s is not None and (s[0].stop - s[0].start) >= 5)
    elif vname == 'SIDE':
        eye_see = (~cad_fg[350:435, 760:890]) & ref_fg[350:435, 760:890]
        feat['side_eye_nose_see_px'] = int(eye_see.sum())
        gap_m[350:435, 760:890] |= eye_see
    elif vname == 'BACK':
        feat['back_sil_asym_px'] = int((cad_fg[40:500, 4:450] ^ cad_fg[40:500, 450:896][:, ::-1]).sum())

    mat_err = int(((rc != cc) & core).sum())
    fg_u = max(1, int((ref_fg | cad_fg).sum()))
    rgb_mae = float(np.abs(ref_rgb[ref_fg | cad_fg].astype(float) - cad_rgb[ref_fg | cad_fg].astype(float)).mean())
    return {
        'fg_material_match': round(1.0 - mat_err / fg_u, 5),
        'fg_rgb_similarity': round(1.0 - rgb_mae / 255.0, 5),
        'seam_gap_excess_px': int(gap_m.sum()), 'tiara_forehead_gap_px': tiara_gap,
        'tiara_crown_gap_px': tiara_crown_gap, 'white_in_temple_px': white_in_temple,
        'jaw_folded_under_mouth_px': jaw_fold, 'crown_fused_seam_px': crown_fused,
        'panel_intrusion_px': int(intr_m.sum()), 'honeycomb_bezel_err_px': int(bez_m.sum()),
        'leak_or_blotch_px': int(leak_m.sum()), 'honeycomb_bezels': bez_stats, **feat,
    }, (gap_m, intr_m, bez_m, leak_m)


_TERM_CACHE: dict[Any, Any] = {}
_TERM_VER = 'v2'
_HIDDEN_HEAD = frozenset({
    'chassis', 'inner_liner', 'compute_heatsink', 'gaze_servos', 'gaze_linkages',
    'brow_servos', 'lid_servos', 'oled_driver_pcb', 'left_gimbal', 'right_gimbal', 'neck_armor',
})


def _term_cache_path(tag, extra):
    import hashlib
    from tools.render import face_cranium as fc_mod, face_laid as fl_mod
    sig = f'{_TERM_VER}|{tag}|{fc_mod.cache_key()}|{extra}|{os.path.getmtime(fl_mod.__file__):.3f}|{os.path.getmtime(fc_mod.__file__):.3f}'
    h = hashlib.sha256(sig.encode('utf-8')).hexdigest()[:16]
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    return os.path.join(root, '.cache', 'compiled', f'face_{tag}_{h}.npz')


def _weld_part_mesh(parts_dict, shift, voxel_m=0.0038):
    """Weld and optionally voxel-decimate `(pos, material)` per part for real-time `terminal-ux`."""
    from coaxial.graphics import lit
    from tools.render import face_honeycomb as fh
    fine_k = {'left_eye_oled', 'right_eye_oled', 'oled_mouth_graphic', 'left_camera', 'right_camera', 'nasal_bridge'}
    cs, ts, us, ms, base = [], [], [], [], 0
    sh = np.asarray(shift, np.float64)
    for key, tris in parts_dict.items():
        if not tris:
            continue
        raw = np.array([t for t, _c in tris], np.float64) + sh
        cols = np.array([c for _t, c in tris], np.uint32)
        gloss = ((cols == fh.OPTIC_GLASS).all(1) | (cols == fh.OPTIC_SCLERA).all(1) | (cols == fh.TITANIUM).all(1)
                 | ((key in ('left_eye_oled', 'right_eye_oled')) & (cols[:, 2] >= 36)) | (cols == (236, 244, 255)).all(1))
        flags = (np.uint32(lit.PAINTED) | np.where(gloss, np.uint32(lit.HARD), np.uint32(lit.WOVEN))
                 | (cols[:, 0] << np.uint32(16)) | (cols[:, 1] << np.uint32(8)) | cols[:, 2])
        v3, f3 = raw.reshape(-1, 3), np.repeat(flags, 3)
        step = (0.55 * voxel_m) if key in fine_k else voxel_m
        q = np.rint(v3 / step) if step > 0.0 else np.round(v3, 5)
        vk = np.c_[q, f3.astype(np.float64)]
        _, idx_u, inv = np.unique(vk, axis=0, return_index=True, return_inverse=True)
        tri = inv.reshape(-1, 3)
        keep = (tri[:, 0] != tri[:, 1]) & (tri[:, 1] != tri[:, 2]) & (tri[:, 2] != tri[:, 0])
        if not np.any(keep):
            continue
        cnt = np.bincount(inv, minlength=len(idx_u)).astype(np.float64)[:, None]
        cu = np.column_stack([np.bincount(inv, weights=v3[:, d], minlength=len(idx_u)) for d in range(3)]) / cnt
        tri_k = tri[keep]
        used, remap = np.unique(tri_k, return_inverse=True)
        cs.append(cu[used])
        ts.append(remap.reshape(-1, 3) + base)
        us.append(np.zeros((len(used), 2), np.float64))
        ms.append(f3[idx_u[used]])
        base += len(used)
    return np.vstack(cs), np.vstack(ts), np.vstack(us), np.concatenate(ms)


def terminal_parts(torso_at, neck_at, head_at):
    """Cached `[('mecha_head', 'head', ...), ('mecha_neck', 'neck', ...), ('mecha_collar', 'torso', ...)]` for `terminal-ux`."""
    from tools.render import face_built as fb, face_cranium as fc_mod
    sh_c = tuple(np.round(np.asarray(neck_at, float) - np.asarray(torso_at, float), 6))
    key = ('parts', fc_mod.cache_key(), sh_c)
    if key not in _TERM_CACHE:
        z0, path = (0.0, 0.0, 0.0), _term_cache_path('term', sh_c)
        names = (('mecha_head', 'head'), ('mecha_neck', 'neck'), ('mecha_collar', 'torso'))
        loaded = None
        if os.path.isfile(path):
            try:
                with np.load(path) as z:
                    loaded = [(n, p, (), z0, 0.0, (z[f'{n}_c'], z[f'{n}_t'], z[f'{n}_u'], z[f'{n}_m'])) for n, p in names]
            except Exception:
                loaded = None
        if loaded is None:
            hp = {k: v for k, v in fb.head_parts(**fb.PRESETS['neutral'], plates=True).items() if k not in _HIDDEN_HEAD}
            np_all = fc_mod.neck_armor_panels()
            collar_k = {'neck_clavicle_L', 'neck_clavicle_R', 'neck_trapezius_L', 'neck_trapezius_R'}
            nk_p = {k: v for k, v in np_all.items() if k not in collar_k}
            cl_p = {k: v for k, v in np_all.items() if k in collar_k}
            loaded = [
                ('mecha_head', 'head', (), z0, 0.0, _weld_part_mesh(hp, z0, voxel_m=0.0038)),
                ('mecha_neck', 'neck', (), z0, 0.0, _weld_part_mesh(nk_p, z0, voxel_m=0.0042)),
                ('mecha_collar', 'torso', (), z0, 0.0, _weld_part_mesh(cl_p, sh_c, voxel_m=0.0045)),
            ]
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                savez: Any = np.savez
                savez(path, **{f'{n}_{s}': arr for n, *_r, mesh in loaded for s, arr in zip('ctum', mesh)})
            except OSError:
                pass
        _TERM_CACHE[key] = loaded
    return _TERM_CACHE[key]


def render_terminal_head(pose, width, height, zoom=1.0, colour=True, lit_r=None):
    """Render the interactive 3D mechatronic head in truecolor Braille for `show_render.py --model head`."""
    from coaxial.draw.orientation import matrix
    from coaxial.graphics import engine, lit
    from coaxial.graphics.shapes import smooth
    from tools.render import face_built as fb, face_cranium as fc_mod
    ck = ('head_lit', fc_mod.cache_key())
    if ck not in _TERM_CACHE:
        path, got = _term_cache_path('head', 'v14'), None
        if os.path.isfile(path):
            try:
                with np.load(path) as z:
                    got = (z['pos'], z['nrm'], z['uv'], z['mat'], z['idx'], float(z['reach'][0]))
            except Exception:
                got = None
        if got is None:
            pts = {k: v for k, v in fb.posed('neutral', plates=True, include_neck=True).items() if k not in _HIDDEN_HEAD}
            pos, idx, uv, mat = _weld_part_mesh(pts, (0.0, 0.0, 0.0), voxel_m=0.0014)
            pos_c = (pos - np.array([0.0, 0.515, 0.005], dtype=np.float64)).astype(np.float32)
            nrm = smooth(pos_c, idx).astype(np.float32)
            reach = float(np.linalg.norm(pos_c, axis=1).max())
            got = (pos_c, nrm, uv.astype(np.float32), mat.astype(np.uint32), idx.astype(np.uint32), reach)
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                np.savez(path, pos=got[0], nrm=got[1], uv=got[2], mat=got[3], idx=got[4], reach=np.array([reach]))
            except OSError:
                pass
        _TERM_CACHE[ck] = got
    pos_c, nrm, uv, mat, idx, reach = _TERM_CACHE[ck]
    m = matrix(pose)
    fine = engine.fine(engine.camera(width, height, reach * 0.88, distance=2.6, zoom=zoom))
    c0 = np.zeros(3, dtype=np.float32)
    if lit_r is not None:
        depth, rgba = lit_r.raster(pos_c, nrm, uv, mat, idx, m, fine, c0, reach * 1.35)
    else:
        depth, rgba = lit.splat((pos_c, nrm, mat, uv), m, fine, c0)
    return '\n'.join(lit.braille(depth, rgba, np.zeros_like(depth), width, height, colour)), int(len(idx))

