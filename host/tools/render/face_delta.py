#!/usr/bin/env python3
"""Full-resolution (896x1200 / 1200x896) unmasked STL-roundtrip diff and occlusion audit."""
import argparse
import json
import os
import pathlib
import sys
from typing import Any

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
from scipy import ndimage  # noqa: E402

from tools.dev import token_budget  # noqa: E402
from tools.render import face_audit, face_built, face_contours as fcon, face_cranium as fc, face_honeycomb as fh, face_laid as fl, face_ray as fr  # noqa: E402

VIEWS = (
    ('FRONT', 'front', 'robot 1_front.png'),
    ('SIDE', 'profile90', 'robot 1_side.png'),
    ('PERSP', 'threeQuarter', 'robot 1_perspective.png'),
    ('BACK', 'rear', 'robot 1_back.png'),
)

CAD_FILES = tuple(f'host/tools/render/face_{k}.py' for k in (
    'honeycomb', 'contours', 'ray', 'hull', 'carve', 'cranium', 'laid', 'built', 'audit', 'converge', 'prints',
))

PART_IDS = {
    'forehead-diamond': (248, 52, 52), 'forehead-wings': (52, 248, 52), 'face-brows': (52, 88, 248),
    'face-inner-cheeks': (248, 232, 52), 'face-outer-cheeks': (248, 52, 232), 'face-nose': (52, 240, 240),
    'face-oled-mouth': (248, 144, 48), 'face-camera-eyes': (156, 248, 48), 'jaw-chin-cap': (48, 156, 248),
    'jaw-front-hex': (248, 48, 136), 'jaw-rear-hex': (144, 68, 248), 'crown-top-plates': (48, 224, 136),
    'crown-occipital-plates': (196, 184, 56), 'crown-nape-yoke': (64, 188, 188), 'temple-hex-bezels': (188, 72, 188),
    'temple-ear-pods': (236, 98, 196), 'frame-liner-xtruss': (96, 198, 96), 'neck-dorsal-spine': (96, 118, 212),
    'neck-side-blades': (218, 128, 96), 'neck-tubes-bust': (128, 212, 178),
}
HONEYCOMB_PARTS = frozenset(('jaw-front-hex', 'jaw-rear-hex', 'temple-hex-bezels', 'neck-side-blades'))
FRONT_FACE_PARTS = frozenset((
    'forehead-diamond', 'forehead-wings', 'face-brows', 'face-inner-cheeks',
    'face-outer-cheeks', 'face-nose', 'face-oled-mouth', 'jaw-chin-cap',
))
RIM_COLOURS = frozenset((
    fh.FRAME_VOID, fh.BOLT_SOCK, fh.BOLT_RIM, fh.BAFFLE_DARK,
    fh.CRANIUM_RIM, fh.PANEL_RIM, fh.AMBER_DARK, (152, 156, 166), (136, 86, 28), (128, 132, 142),
))

_LR_CACHE: list[Any] = []
_REF_CACHE: dict[str, dict[str, dict[str, Any]]] = {}


def segment_reference_panels(rgba, preset_id):
    """Segment full-resolution (`896x1200` / `1200x896`) ground-truth panel masks preserving holes & slots."""
    if preset_id in ('front', 'profile90'):
        out = fcon.segment_front_and_side(rgba, preset_id)
        return {k: v for k, v in out.items() if int(v.sum()) >= 80}
    h, w = rgba.shape[:2]
    r, g, b, a = rgba[..., 0].astype(float), rgba[..., 1].astype(float), rgba[..., 2].astype(float), rgba[..., 3]
    fg = a > 127
    lum = 0.299 * r + 0.587 * g + 0.114 * b
    is_wh = fg & (lum > 98.0) & ((r - b) < 36.0)
    is_am_hi = fg & (r > 82.0) & ((r - b) > 38.0) & (g > 36.0) & (r > g * 1.12)
    yy, xx = np.mgrid[0:h, 0:w]
    out = {}
    if preset_id == 'threeQuarter':
        out['temple-hex-bezels'] = fcon.seeded_comp(is_am_hi & (xx <= 616) & (yy <= 356), 445, 158, 20, 36000)
        am_cut = fg & (r > 58.0) & ((r - b) > 22.0) & (g > 22.0) & (lum > 78.0) & (~np.asarray(ndimage.grey_dilation(lum, size=(3, 3)) - lum > 38.0, dtype=bool))
        dm_p = fcon.seeded_comp(am_cut & (xx >= 792) & (yy >= 145) & (yy <= 286), 835, 215, 18, 400)
        out['forehead-wings'] = fr.fill_interior_holes(
            (fcon.seeded_comp(am_cut & (yy <= 250) & (~out['temple-hex-bezels']) & (~dm_p), 720, 175, 20, 400)
             | fcon.seeded_comp(am_cut & (yy <= 250) & (xx >= 820) & (~dm_p), 855, 158, 18, 400)), 1200)
        wh_cut = is_wh & (~np.asarray(ndimage.grey_dilation(lum, size=(3, 3)) - lum > 34.0, dtype=bool)) & (lum > 114.0)
        br_s = (fcon.seeded_comp(wh_cut & (yy <= 338) & (xx <= 862), 775, 280, 24, 0)
                | fcon.seeded_comp(wh_cut & (yy <= 345) & (xx >= 855), 878, 295, 18, 0))
        out['face-brows'] = fr.fill_interior_holes(np.asarray(ndimage.binary_dilation(br_s), dtype=bool) & is_wh & (yy <= 345), 1600)
        out['jaw-rear-hex'] = fcon.seeded_comp(is_am_hi & (xx <= 628) & (yy >= 430) & (yy <= 606), 530, 445, 20, 20000)
        out['jaw-front-hex'] = fcon.seeded_comp(is_am_hi & (xx >= 565) & (xx <= 765) & (yy >= 475), 632, 490, 20, 20000)
        c3 = (yy >= 615) & (yy <= 676) & (xx >= 762) & (xx <= 856) & (~out['jaw-front-hex'])
        out['jaw-chin-cap'] = fcon.seeded_comp(c3 & fg & (r > 60.0) & ((r - b) > 26.0) & (g > 26.0), 810, 640, 18, 1200)
        oc_b = ((yy >= 286.0 + 0.088 * (xx - 585.0)) & (yy <= 556)
                & np.where(yy < 395, xx <= 719.0 + 0.72 * (yy - 326.0), xx <= 723.0 - 0.15 * (yy - 422.0)))
        oc_er = fcon.seeded_comp(np.asarray(ndimage.binary_erosion(is_wh & oc_b), dtype=bool), 605, 455, 24, 0)
        out['face-outer-cheeks'] = fr.fill_interior_holes(np.asarray(ndimage.binary_dilation(oc_er), dtype=bool) & is_wh & oc_b, 1500)
        m3_box = (xx >= 682) & (xx <= 890) & (yy >= 505) & (yy <= 648) & (~out['jaw-front-hex']) & (~out['face-outer-cheeks']) & (~out['jaw-chin-cap'])
        m3_w = m3_box & fg & (lum > 52.0) & ((r - b) < 44.0)
        m3_er = fcon.seeded_comp(np.asarray(ndimage.binary_erosion(m3_w), dtype=bool), 820, 575, 18, 0)
        out['face-oled-mouth'] = fr.fill_interior_holes(np.asarray(ndimage.binary_dilation(m3_er), dtype=bool) & m3_w, 5200)
    elif preset_id == 'rear':
        is_am_b = fg & (r > 92.0) & ((r - b) > 42.0) & (g > 42.0) & (r > g * 1.14)
        out['temple-hex-bezels'] = is_am_b & (yy >= 200) & (yy <= 445) & ((xx <= 165) | (xx >= 730))
        out['jaw-rear-hex'] = is_am_b & (yy >= 590) & (yy <= 820) & ((xx <= 235) | (xx >= 660))
    return {k: v for k, v in out.items() if int(v.sum()) >= 80}


def pack_id_group(pid, tris_list, shift):
    """Quantize `tris_list` via `.stl` float32 round-trip; tag dark bores on face plates as frame."""
    from coaxial.graphics import lit
    if not tris_list:
        return np.zeros((0, 3), np.float32), np.zeros((0,), np.uint32)
    rgb_main, rgb_rim, split_rim = PART_IDS[pid], PART_IDS['frame-liner-xtruss'], (pid in FRONT_FACE_PARTS)
    q_mm = (np.array([t for t, _c in tris_list], np.float64) * 1e3).astype(np.float32)
    arr = (q_mm * np.float32(1e-3)).reshape(-1, 3) + np.asarray(shift, np.float32)
    cols = [rgb_rim if (split_rim and tuple(int(v) for v in c) in RIM_COLOURS) else rgb_main for _t, c in tris_list]
    ca = np.repeat(np.asarray(cols, np.uint32), 3, axis=0)
    mat = np.uint32(lit.PAINTED | lit.WOVEN) | (ca[:, 0] << np.uint32(16)) | (ca[:, 1] << np.uint32(8)) | ca[:, 2]
    return arr, mat.astype(np.uint32)


def build_id_groups_stl():
    """Return `{pid: (pos_f32, mat_u32)}` for all 20 `.stl`-quantized CAD part groups in torso frame."""
    from coaxial.graphics import lit
    from coaxial.graphics.shapes import finer, loft
    from tools.render.robot_worn import NECK_RINGS
    nt, _hr, ht, *_r = fl.anchors()
    ht_v, nt_v = np.asarray(ht, float), np.asarray(nt, float)
    fp, cp, jp = fl.frontal_panels(), fl.cheek_and_nose_panels(), fl.lower_jaw_and_oled_panels()
    ears, tv_l, tv_r = fl.ear_microphones(), *fc.temporal_amber_vents()
    mp, ocp = fc.mastoid_armor_plates(), fc.occipital_panels()
    eyes_d = face_built._eyes_posed((0.0, 0.0, 0.02))
    expr = face_built._expressions_posed(0.0, 0.0, 0.0, 0.0, 0.0)
    lash_l, lash_r = fl.eye_lashes()
    cup_l, cup_r = fl.eye_socket_cups()
    lin_l, lin_r = fl.cranial_liner_shells()
    nk = fc.neck_armor_panels()
    c_sh, tr_sh, _u, _m = loft(finer(NECK_RINGS, -0.040, 0.048, 0.004), lit.paint((24, 26, 30)), poles=(-0.044, 0.050))
    c_sh[:, 0] *= 0.38
    c_sh[:, 2] = c_sh[:, 2] * 0.38 - 0.022
    sheath = [(c_sh[t], (24, 26, 30)) for t in tr_sh]
    wings_t = fp.get('brow_amber_tiara') or (fp.get('brow_amber_arch_L', []) + fp.get('brow_amber_arch_R', []))

    head_g = {
        'forehead-diamond': fp['brow_amber_keystone'], 'forehead-wings': wings_t,
        'face-brows': fp['forehead_porcelain_L'] + fp['forehead_porcelain_R'],
        'face-inner-cheeks': cp['cheek_inner_L'] + cp['cheek_inner_R'],
        'face-outer-cheeks': cp['cheek_outer_L'] + cp['cheek_outer_R'], 'face-nose': cp['nose_porcelain'],
        'face-oled-mouth': jp['flexible_oled_panel'] + expr['oled_mouth_graphic'],
        'face-camera-eyes': (eyes_d['left_camera'] + eyes_d['right_camera'] + eyes_d['left_eye_oled'] + eyes_d['right_eye_oled']
                             + eyes_d['left_gimbal'] + eyes_d['right_gimbal']
                             + expr['left_upper_lid'] + expr['right_upper_lid'] + expr['left_lower_lid'] + expr['right_lower_lid']
                             + lash_l + lash_r + cup_l + cup_r),
        'jaw-chin-cap': jp['chin_amber_bridge'], 'jaw-front-hex': jp['jaw_amber_front_L'] + jp['jaw_amber_front_R'],
        'jaw-rear-hex': jp['jaw_amber_rear_L'] + jp['jaw_amber_rear_R'], 'crown-top-plates': fc.cranium_dome_plates(),
        'crown-occipital-plates': (ocp['occipital_upper_L'] + ocp['occipital_upper_R']
                                   + ocp['occipital_lower_L'] + ocp['occipital_lower_R'] + mp + fl.mirror_x(mp)),
        'crown-nape-yoke': ocp['occipital_nape_yoke'], 'temple-hex-bezels': tv_l + tv_r,
        'temple-ear-pods': ears['left_ear_mic'] + ears['right_ear_mic'],
        'frame-liner-xtruss': (lin_l + lin_r + cp['orbital_pillar_L'] + cp['orbital_pillar_R']
                               + expr['left_brow'] + expr['right_brow']
                               + jp['oled_carrier_bezel'] + ocp['occipital_xbrace_module'] + fl.chassis_static()),
    }
    neck_g = {
        'neck-dorsal-spine': [t for k, ts in nk.items() if k.startswith('neck_vertebra_') for t in ts],
        'neck-side-blades': [t for k, ts in nk.items() if k.startswith('neck_shroud_') for t in ts],
        'neck-tubes-bust': sheath + [t for k, ts in nk.items() if not k.startswith(('neck_vertebra_', 'neck_shroud_')) for t in ts],
    }
    return ({pid: pack_id_group(pid, ts, ht_v) for pid, ts in head_g.items()}
            | {pid: pack_id_group(pid, ts, nt_v) for pid, ts in neck_g.items()})


def _build_id_tris_stl():
    grps = build_id_groups_stl()
    return np.vstack([v[0] for v in grps.values() if len(v[0])]), np.concatenate([v[1] for v in grps.values() if len(v[1])])


_cam_delta_for = fc._active_cam_delta


def _raster_canonical(lit_pack, pos_id, mat_id, preset_id, ht, lr):
    from coaxial.graphics import lit
    cd = _cam_delta_for(preset_id)
    m1, cam1, centre = fr.gpu_camera_for(preset_id, ht, mult=1.0, cam_delta=cd)
    w, h = cam1['width'], cam1['height']
    idx_id = np.arange(len(pos_id), dtype=np.uint32).reshape(-1, 3)
    uv0 = np.zeros((len(pos_id), 2), np.float32)
    reach = float(np.linalg.norm(pos_id - np.asarray(centre, np.float32), axis=1).max()) + 0.05
    nrm_cam = np.broadcast_to(
        (np.asarray(m1, np.float32).reshape(3, 3).T @ np.array([0.0, 0.0, 1.0], np.float32))[None, :],
        pos_id.shape,
    ).copy()
    if lit_pack is not None:
        m2, cam2, _c = fr.gpu_camera_for(preset_id, ht, mult=2.0, cam_delta=cd)
        pos, nrm, uv, mat, idx = lit_pack
        d_lit, rgba_lit = lr.raster(pos, nrm, uv, mat, idx, m2, cam2, centre, reach) if lr else lit.splat(
            (pos, nrm, mat, uv), m2, cam2, centre)
        fg2 = d_lit > 0.0
        rgb = np.where(fg2[..., None], rgba_lit[..., :3].astype(np.float32),
                       np.array([18.0, 20.0, 24.0], np.float32)).reshape(h, 2, w, 2, 3).mean((1, 3))
        cad_rgb = np.clip(np.rint(rgb), 0, 255).astype(np.uint8)
    else:
        cad_rgb = np.zeros((h, w, 3), np.uint8)

    d_id, rgba_id = (lr.raster(pos_id, nrm_cam, uv0, mat_id, idx_id, m1, cam1, centre, reach)
                     if lr is not None else lit.splat((pos_id, nrm_cam, mat_id, uv0), m1, cam1, centre))
    rgb_i = rgba_id[..., :3].astype(np.float32)
    cad_m = (d_id > 0.0) & (rgb_i.max(axis=2) > 18.0)
    pal = np.array(list(PART_IDS.values()), np.float32)
    pal_u = pal / np.linalg.norm(pal, axis=1, keepdims=True)
    part_idx = np.zeros((h, w), np.int32)
    id_vis = np.zeros((h, w, 3), np.uint8)
    if cad_m.any():
        pix = rgb_i[cad_m]
        win = np.argmax((pix / np.maximum(np.linalg.norm(pix, axis=1, keepdims=True), 1e-6)) @ pal_u.T, axis=1)
        part_idx[cad_m] = win + 1
        id_vis[cad_m] = pal[win].astype(np.uint8)
    return cad_rgb, cad_m, part_idx, id_vis


def _chamfer_px(m1, m2):
    b1 = m1 ^ np.asarray(ndimage.binary_erosion(m1), dtype=bool)
    b2 = m2 ^ np.asarray(ndimage.binary_erosion(m2), dtype=bool)
    if not b1.any() or not b2.any():
        return 99.0
    d2 = np.asarray(ndimage.distance_transform_edt(~b2), dtype=float)
    d1 = np.asarray(ndimage.distance_transform_edt(~b1), dtype=float)
    return float(0.5 * (d2[b1].mean() + d1[b2].mean()))


def _get_refs(ref_p):
    key = str(ref_p.resolve())
    if key not in _REF_CACHE:
        d = {}
        for vname, preset_id, fn in VIEWS:
            raw = Image.open(ref_p / fn).convert('RGBA')
            rgba = np.asarray(raw, np.uint8)
            bg = Image.new('RGBA', raw.size, (18, 20, 24, 255))
            bg.alpha_composite(raw)
            rgb = np.asarray(bg.convert('RGB'), np.uint8)
            fg = rgba[..., 3] > 127
            d[vname] = {'rgb': rgb, 'rgba': rgba, 'fg': fg, 'seg': segment_reference_panels(rgba, preset_id)}
        _REF_CACHE[key] = d
    return _REF_CACHE[key]


def _save_png(arr, path):
    path.unlink(missing_ok=True)
    with open(path, 'wb') as f:
        Image.fromarray(arr).save(f, format='PNG')


def _save_full_res_diffs(out_p, tag, vname, ref_rgb, ref_fg, cad_rgb, cad_fg, part_idx, seg, part_names, masks=None):
    """Write standalone 1:1 full-res CAD and multi-modal discrepancy PNGs (ID + gap + bezel + intrusion + leak)."""
    _save_png(cad_rgb, out_p / f'cad_full_{tag}_{vname}.png')
    diff_vis = (0.44 * ref_rgb.astype(np.float32) + 0.44 * cad_rgb.astype(np.float32)).astype(np.uint8)
    diff_vis[~(ref_fg | cad_fg)] = (12, 14, 18)
    p_wrong, am_c = np.zeros_like(ref_fg), (cad_rgb[..., 0].astype(int) - cad_rgb[..., 2].astype(int)) > 24
    for pid, mr in seg.items():
        ti = part_names.index(pid) + 1
        mc = ((part_idx == ti) & am_c) if vname == 'BACK' else (fr.fill_interior_holes(part_idx == ti, 38000) if pid in HONEYCOMB_PARTS else (part_idx == ti))
        p_wrong |= (mr ^ mc)
    diff_vis[p_wrong] = (255, 56, 196)
    if masks is not None:
        gap_m, intr_m, bez_m, leak_m = masks
        diff_vis[bez_m] = (255, 226, 28)
        diff_vis[gap_m] = (255, 118, 20)
        diff_vis[intr_m | leak_m] = (255, 28, 44)
    diff_vis[ref_fg & (~cad_fg)] = (255, 132, 24)
    diff_vis[(~ref_fg) & cad_fg] = (24, 218, 255)
    _save_png(diff_vis, out_p / f'diff_full_{tag}_{vname}.png')


def measure(ref_dir, out_dir=None, tag='current', prev_json=None, use_gpu=True, fast_check=False, prebuilt=None):
    """Run full-resolution (`896x1200` / `1200x896`) UNMASKED (`0 px` edge forgiveness) `.stl` diff."""
    from coaxial.graphics import gpu
    ref_p, lr = pathlib.Path(ref_dir), None
    if use_gpu:
        if not _LR_CACHE:
            try:
                _LR_CACHE.append(gpu.LitRaster())
            except Exception:
                _LR_CACHE.append(None)
        lr = _LR_CACHE[0]
    ht = fl.anchors()[2]
    if prebuilt is not None:
        lit_pack, (pos_id, mat_id), bounds_ok, chk, (cran_x, face_x, ear_x) = prebuilt
    else:
        neu = face_built.head_parts(**face_built.PRESETS['neutral'], plates=True)
        ear_x, cran_x, face_x = (1e3 * float(np.vstack([t for t, _ in neu[k]])[:, 0].max())
                                 for k in ('left_ear_mic', 'left_temporal', 'left_maxilla'))
        crown_mm = round(1e3 * float(np.vstack([t for t, _ in neu['cranium_dome']])[:, 1].max()), 2)
        chk = {'ok': True, 'crown_mm': crown_mm} if fast_check else face_built.checked()
        bounds_ok = (bool(chk['ok']) and 220.0 <= chk['crown_mm'] <= 229.0
                     and 84.0 <= cran_x <= 89.5 and 74.0 <= face_x <= 82.0 and 86.0 <= ear_x <= 91.5)
        pos_id, mat_id = _build_id_tris_stl()
        lit_pack = None if (fast_check and out_dir is None) else face_built.lit_mesh(
            {k: v for k, v in face_built.posed('neutral').items() if not k.startswith('mount_')})

    refs, part_names, view_metrics, panel_acc = _get_refs(ref_p), list(PART_IDS.keys()), {}, {}
    for vname, preset_id, _fn in VIEWS:
        rd = refs[vname]
        ref_rgb, ref_fg, seg = rd['rgb'], rd['fg'], rd['seg']
        cad_rgb, cad_fg, part_idx, _id_vis = _raster_canonical(lit_pack, pos_id, mat_id, preset_id, ht, lr)
        h, w = ref_fg.shape
        canvas_n, h_cut = float(h * w), int(0.77 * h)
        sil_iou = float((ref_fg & cad_fg).sum() / max(1, (ref_fg | cad_fg).sum()))
        head_iou = float((ref_fg[:h_cut] & cad_fg[:h_cut]).sum() / max(1, (ref_fg[:h_cut] | cad_fg[:h_cut]).sum()))
        yy_s, xx_s = np.mgrid[0:h_cut, 0:w]
        f_roi = (yy_s < int(0.62 * h)) if preset_id == 'front' else ((xx_s >= int(0.36 * w)) if preset_id == 'profile90' else np.ones((h_cut, w), dtype=bool))
        b_sil = ref_fg[:h_cut] ^ np.asarray(ndimage.binary_erosion(ref_fg[:h_cut]), dtype=bool)
        aa_hd = ((rd['rgba'][:h_cut, :, 3] >= 12) & (rd['rgba'][:h_cut, :, 3] <= 244)) | np.asarray(ndimage.binary_dilation(b_sil, iterations=4), dtype=bool)
        sil_raw = int((ref_fg[:h_cut] ^ cad_fg[:h_cut]).sum())
        sil_err = int(((ref_fg[:h_cut] ^ cad_fg[:h_cut]) & f_roi & (~aa_hd)).sum())
        sil_match = 1.0 - float(sil_err) / canvas_n
        ch_px = _chamfer_px(ref_fg[:h_cut], cad_fg[:h_cut])

        v_panels, p_edges = {}, np.zeros_like(ref_fg)
        am_c = ((cad_rgb[..., 0].astype(int) - cad_rgb[..., 2].astype(int)) > 24) if lit_pack is not None else np.ones_like(ref_fg)
        seam_it = 1 if preset_id == 'front' else (2 if preset_id == 'profile90' else 3)
        for pid, mr in seg.items():
            t_idx = part_names.index(pid) + 1
            mc = ((part_idx == t_idx) & am_c) if preset_id == 'rear' else (fr.fill_interior_holes(part_idx == t_idx, 38000) if pid in HONEYCOMB_PARTS else (part_idx == t_idx))
            b_r = mr ^ np.asarray(ndimage.binary_erosion(mr), dtype=bool)
            p_edges |= b_r
            aa_seam = np.asarray(ndimage.binary_dilation(b_r, iterations=seam_it), dtype=bool)
            p_iou = float((mr & mc).sum() / max(1, (mr | mc).sum()))
            p_err = int(((mr ^ mc) & (~aa_seam)).sum())
            p_raw = int((mr ^ mc).sum())
            p_match = 1.0 - float(p_err) / canvas_n
            occluded = int((mr & (part_idx > 0) & (~mc) & (~aa_seam)).sum())
            p_ch = _chamfer_px(mr, mc) if mr.any() and mc.any() else 99.0
            v_panels[pid] = {'match': round(p_match, 6), 'raw_match': round(1.0 - float(p_raw) / canvas_n, 6),
                             'err_px': p_err, 'raw_err_px': p_raw, 'iou': round(p_iou, 4),
                             'occluded_px': occluded, 'chamfer_px': round(p_ch, 2)}
            panel_acc.setdefault(pid, []).append((p_match, 1.0 - float(p_raw) / canvas_n, p_iou, occluded, p_ch))

        seam_roi = np.asarray(ndimage.binary_dilation(p_edges[:h_cut], iterations=4), dtype=bool) & (ref_fg[:h_cut] | cad_fg[:h_cut])
        rr_h, cr_h = ref_rgb[:h_cut].astype(float), cad_rgb[:h_cut].astype(float)
        r_cls = np.where((rr_h.max(axis=2) > 110.0) & ((rr_h[..., 0] - rr_h[..., 2]) < 36.0), 2,
                         np.where((rr_h[..., 0] > 68.0) & ((rr_h[..., 0] - rr_h[..., 2]) > 26.0), 1, 0))
        c_cls = np.where((cr_h.max(axis=2) > 110.0) & ((cr_h[..., 0] - cr_h[..., 2]) < 36.0), 2,
                         np.where((cr_h[..., 0] > 68.0) & ((cr_h[..., 0] - cr_h[..., 2]) > 26.0), 1, 0))
        aa_all = np.asarray(ndimage.binary_dilation(p_edges[:h_cut], iterations=2), dtype=bool)
        seam_roi_m = (1.0 - float(((r_cls != c_cls) & seam_roi & (~aa_all)).sum()) / max(1, int(seam_roi.sum()))) if (lit_pack is not None and seam_roi.any()) else 1.0
        pm_m = float(np.mean([v['match'] for v in v_panels.values()])) if v_panels else sil_match
        pi_m = float(np.mean([v['iou'] for v in v_panels.values()])) if v_panels else sil_iou
        v_match = float(np.mean([sil_match] + [v['match'] for v in v_panels.values()]))
        disc, d_masks = (face_audit.audit_discrepancies(vname, preset_id, rd['rgba'], ref_rgb, ref_fg, cad_rgb, cad_fg, seg, part_idx, part_names)
                         if lit_pack is not None else ({}, None))
        view_metrics[vname] = {'native_size': [w, h], 'sil_iou': round(sil_iou, 4), 'head_sil_iou': round(head_iou, 4),
                               'sil_match': round(sil_match, 6), 'sil_raw_match': round(1.0 - float(sil_raw) / canvas_n, 6),
                               'sil_err_px': sil_err, 'chamfer_px': round(ch_px, 2), 'seam_roi_match': round(seam_roi_m, 5),
                               'panel_match_mean': round(pm_m, 6), 'panel_iou_mean': round(pi_m, 4),
                               'view_match': round(v_match, 6), 'discrepancies': disc, 'panels': v_panels}
        if out_dir:
            out_p = pathlib.Path(out_dir)
            out_p.mkdir(parents=True, exist_ok=True)
            _save_full_res_diffs(out_p, tag, vname, ref_rgb, ref_fg, cad_rgb, cad_fg, part_idx, seg, part_names, d_masks)

    per_panel = {k: {'match': round(float(np.mean([x[0] for x in vs])), 6), 'raw_match': round(float(np.mean([x[1] for x in vs])), 6),
                     'iou': round(float(np.mean([x[2] for x in vs])), 4), 'occluded_px': int(sum(x[3] for x in vs)),
                     'chamfer_px': round(float(np.mean([x[4] for x in vs])), 2)} for k, vs in panel_acc.items()}
    ave = lambda key: float(np.mean([v[key] for v in view_metrics.values()]))
    sil_m, head_m, ch_m, sil_mm, vw_m = ave('sil_iou'), ave('head_sil_iou'), ave('chamfer_px'), ave('sil_match'), ave('view_match')
    min_vw = float(min(v['view_match'] for v in view_metrics.values()))
    pm_m, pr_m, pi_m = (float(np.mean([v[k] for v in per_panel.values()])) for k in ('match', 'raw_match', 'iou'))
    occ_tot = int(sum(v['occluded_px'] for v in per_panel.values()))
    obj = 0.40 * pi_m + 0.35 * sil_m + 0.25 * head_m - 0.002 * ch_m - 1e-6 * occ_tot
    summary = {'mean_sil_iou': round(sil_m, 4), 'mean_head_sil_iou': round(head_m, 4),
               'mean_sil_match': round(sil_mm, 6), 'mean_view_match': round(vw_m, 6),
               'min_view_match': round(min_vw, 6), 'mean_chamfer_px': round(ch_m, 2),
               'mean_panel_match': round(pm_m, 6), 'mean_panel_raw_match': round(pr_m, 6),
               'mean_panel_iou': round(pi_m, 4), 'total_occluded_px': occ_tot, 'objective': round(obj, 5)}
    toks = {f: token_budget.tokens(f) for f in CAD_FILES if pathlib.Path(f).exists()}
    return {'tag': tag, 'checked_ok': bool(bounds_ok), 'summary': summary, 'views': view_metrics,
            'per_panel': per_panel, 'dims_mm': {'crown': chk['crown_mm'], 'cranium': round(cran_x, 2),
                                                'face': round(face_x, 2), 'ear': round(ear_x, 2)},
            'tokens_max': max(toks.values()) if toks else 0}


def main(argv=None):
    """CLI entry point for full-resolution unmasked 4-view `.stl` diff."""
    ap = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    ap.add_argument('--ref-dir', required=True)
    ap.add_argument('--out-dir', default=None)
    ap.add_argument('--tag', default='current')
    args = ap.parse_args(argv)
    os.environ.setdefault('COAXIAL_ROBOT', 'gynoid_mesomorph_medium_coax')
    rep = measure(args.ref_dir, args.out_dir, args.tag)
    if args.out_dir:
        (pathlib.Path(args.out_dir) / f'delta_{args.tag}.json').write_text(json.dumps(rep, indent=2), encoding='utf-8')
    print(json.dumps(rep['summary'], indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
