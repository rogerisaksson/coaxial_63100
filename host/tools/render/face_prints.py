#!/usr/bin/env python3
"""Converge full-res CAD, export screw-free panel STLs, M2/M3 screws, 3MF assembly and P1S plates."""
import argparse
import json
import math
import os
import pathlib
import re
import struct
import sys
import zipfile

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

from tools.render import csg, face_built, face_cranium as fc, face_honeycomb as fh, face_laid as fl  # noqa: E402
from tools.render import mounting as m_draw, prints as pr, solids  # noqa: E402

PLATE_M, TALL_M, CORNER_M, AIR_M = pr.PLATE_M, pr.TALL_M, pr.CORNER_M, pr.AIR_M


def _bake_tuning_into_source(best):
    """Update `TUNING` in `face_cranium.py` cleanly when `--converge` finds improved parameters."""
    p, ks = pathlib.Path(fc.__file__), list(fc.TUNING)
    src = p.read_text(encoding='utf-8')
    rows = ['    ' + ' '.join(f"'{k}': {best[k]:.4f}," for k in ks[i:i + 4]) for i in range(0, len(ks), 4)]
    new_src = re.sub(r'TUNING\s*=\s*\{[^}]+\}', '\n'.join(['TUNING = {', *rows, '}']), src, count=1)
    p.write_text(new_src, encoding='utf-8', newline='\n')
    fc.TUNING.update(best)


def assembled_panels(screw_free=True):
    """Return `(panels_dict, fasteners_torso)` with screw-free counterbored panels and M2/M3 instances."""
    nt, _hr, ht, *_r = fl.anchors()
    ht_v, nt_v = np.asarray(ht, float), np.asarray(nt, float)
    to_h = lambda tris: [(np.asarray(t, float) + ht_v, c) for t, c in tris]
    to_n = lambda tris: [(np.asarray(t, float) + nt_v, c) for t, c in tris]

    def _mirror_since(idx0, x_min=0.001):
        for pt, nrm, sp in list(fh.FASTENER_LOG[idx0:]):
            if pt[0] > x_min:
                fh.FASTENER_LOG.append(((-pt[0], pt[1], pt[2]), (-nrm[0], nrm[1], nrm[2]), sp))

    fh.FASTENER_LOG.clear()
    fh.SCREW_FREE_PANELS = bool(screw_free)
    out, head_spots, neck_spots = {}, [], []
    try:
        for k, ts in fl.frontal_panels().items():
            out[k] = (to_h(ts), 'amber_gold' if 'amber' in k else 'porcelain_white')
        for k, ts in fl.cheek_and_nose_panels().items():
            out[k] = (to_h(ts), 'graphite_dark' if 'pillar' in k else 'porcelain_white')
        for k, ts in fc.cranium_dome_panels().items():
            out[k] = (to_h(ts), 'graphite_dark')

        n0 = len(fh.FASTENER_LOG)
        tv_l, tv_r = fc.temporal_amber_vents()
        _mirror_since(n0, 0.001)
        out['temple_amber_vent_L'] = (to_h(tv_l), 'amber_gold')
        out['temple_amber_vent_R'] = (to_h(tv_r), 'amber_gold')

        n0 = len(fh.FASTENER_LOG)
        mp_l = fc.mastoid_panels_left()
        _mirror_since(n0, 0.001)
        for k, ts in mp_l.items():
            out[k + '_L'], out[k + '_R'] = (to_h(ts), 'graphite_dark'), (to_h(fh.mirror_x(ts)), 'graphite_dark')

        for k, ts in fc.occipital_panels().items():
            out[k] = (to_h(ts), 'graphite_dark')

        expr = face_built._expressions_posed(0.0, 0.0, 0.0, 0.0, 0.0)
        for k_o, k_e, grp in (('brow_hood_L', 'left_brow', 'graphite_dark'), ('brow_hood_R', 'right_brow', 'graphite_dark'),
                              ('upper_eyelid_L', 'left_upper_lid', 'graphite_dark'), ('upper_eyelid_R', 'right_upper_lid', 'graphite_dark'),
                              ('lower_eyelid_L', 'left_lower_lid', 'graphite_dark'), ('lower_eyelid_R', 'right_lower_lid', 'graphite_dark')):
            out[k_o] = (to_h(expr[k_e]), grp)

        n0 = len(fh.FASTENER_LOG)
        lj_p = fl.lower_jaw_and_oled_panels()
        _mirror_since(n0, 0.020)
        for k, ts in lj_p.items():
            grp = 'electronic_oled' if k == 'flexible_oled_panel' else ('amber_gold' if 'amber' in k else 'carbon_structural')
            out[k] = (to_h(ts), grp)

        n0 = len(fh.FASTENER_LOG)
        ears = fl.ear_microphones()
        _mirror_since(n0, 0.001)
        out['ear_housing_L'], out['ear_housing_R'] = (to_h(ears['left_ear_mic']), 'graphite_dark'), (to_h(ears['right_ear_mic']), 'graphite_dark')
        eyes = face_built._eyes_posed((0.0, 0.0, 0.02))
        for k_o, k_e, grp in (('camera_module_L', 'left_camera', 'carbon_structural'), ('camera_module_R', 'right_camera', 'carbon_structural'),
                              ('eye_oled_display_L', 'left_eye_oled', 'electronic_oled'), ('eye_oled_display_R', 'right_eye_oled', 'electronic_oled'),
                              ('gimbal_ring_L', 'left_gimbal', 'carbon_structural'), ('gimbal_ring_R', 'right_gimbal', 'carbon_structural')):
            out[k_o] = (to_h(eyes[k_e]), grp)
        out['cranial_chassis'] = (to_h(fl.chassis_static()), 'carbon_structural')
        for k, ts in fl.inner_liner_panels().items():
            out[k] = (to_h(ts), 'carbon_structural')
        out['compute_heatsink'] = (to_h(fl.compute_heatsink()), 'carbon_structural')
        out['oled_driver_pcb'] = (to_h(face_built.oled_driver_module()), 'electronic_oled')
        out['oled_mouth_pixels'] = (to_h(expr['oled_mouth_graphic']), 'electronic_oled')
        head_spots = [(tuple(np.asarray(pt) + ht_v), nrm, sp) for pt, nrm, sp in fh.FASTENER_LOG]

        fh.FASTENER_LOG.clear()
        for k, ts in fc.neck_armor_panels().items():
            out[k] = (to_n(ts), 'carbon_structural' if ('sleeve' in k or 'clevis' in k) else ('amber_gold' if 'amber' in k else 'graphite_dark'))
        for pt, nrm, sp in list(fh.FASTENER_LOG):
            neck_spots.append((tuple(np.asarray(pt) + nt_v), nrm, sp))
            if pt[0] > 0.015:
                neck_spots.append((tuple(np.array([-pt[0], pt[1], pt[2]]) + nt_v), (-nrm[0], nrm[1], nrm[2]), sp))
    finally:
        fh.SCREW_FREE_PANELS = False
        fh.FASTENER_LOG.clear()
    return out, head_spots + neck_spots


def _lay_on_bed(tris):
    """Orient `tris` (in m) flat onto bed `y = 0` minimizing height & footprint `(across_x, deep_z, tall_y)`."""
    pts = np.concatenate([t for t, _c in tris], axis=0)
    ctr = pts.mean(axis=0)
    _w, axes = np.linalg.eigh(np.cov((pts - ctr).T))
    up = axes[:, 0].copy()
    along = (pts - ctr) @ up
    span = max(1e-6, float(np.ptp(along)))
    if float(np.sum(along > along.max() - 0.22 * span)) > float(np.sum(along < along.min() + 0.22 * span)):
        up = -up
    ref = np.array([1.0, 0.0, 0.0]) if abs(float(up[0])) < 0.9 else np.array([0.0, 1.0, 0.0])
    side = np.cross(up, ref)
    side /= max(float(np.linalg.norm(side)), 1e-12)
    turn0 = np.array([side, up, np.cross(side, up)])
    flat = pts @ turn0.T
    best = (math.inf, turn0, np.zeros(3))
    for deg in range(0, 90, 3):
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        spin = np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])
        sz = np.ptp(flat @ spin.T, axis=0)
        if sz[0] * sz[2] < best[0]:
            best = (float(sz[0] * sz[2]), spin @ turn0, sz)
    _a, turn, sz = best
    low = (pts @ turn.T).min(axis=0)
    return [(t @ turn.T - low, col) for t, col in tris], (float(sz[0]), float(sz[2]), float(sz[1]))


def _pack_p1s_group(items):
    """Pack `[(name, tris_bed, (across, deep, tall))]` onto `256x256x250 mm` Bambu Lab P1S plates."""
    todo = sorted(items, key=lambda it: (-max(it[2][0], it[2][1]), -min(it[2][0], it[2][1])))
    plates, rows = [], []
    for name, tris, (ac, dp, tl) in todo:
        a, d, rot90 = (ac, dp, False) if ac >= dp else (dp, ac, True)
        for plate, lines in zip(plates, rows):
            for line in lines:
                x = max(line[2], CORNER_M[0] + AIR_M) if line[0] < CORNER_M[1] + AIR_M else line[2]
                if d <= line[1] + 1e-9 and x + a <= PLATE_M - AIR_M:
                    break
            else:
                z = lines[-1][0] + lines[-1][1] + AIR_M
                if z + d > PLATE_M - AIR_M:
                    continue
                lines.append([z, d, AIR_M])
                line, x = lines[-1], AIR_M
            plate.append((name, tris, x, line[0], a, d, tl, rot90, ac))
            line[2] = x + a + AIR_M
            break
        else:
            x, z = CORNER_M[0] + AIR_M, AIR_M
            if x + a > PLATE_M - AIR_M:
                x, z = AIR_M, CORNER_M[1] + AIR_M
            plates.append([(name, tris, x, z, a, d, tl, rot90, ac)])
            rows.append([[z, d, x + a + AIR_M]])
    return plates


def _to_zup_mm(tris_yup_m):
    return [(np.stack([1e3 * np.asarray(t, float)[:, 0], -1e3 * np.asarray(t, float)[:, 2],
                       1e3 * np.asarray(t, float)[:, 1]], axis=1), col) for t, col in tris_yup_m]


def _plate_item_tris_yup(tris, x, z, rot90, orig_ac):
    if rot90:
        tris = [(t[:, (2, 1, 0)] * (1.0, 1.0, -1.0) + (0.0, 0.0, orig_ac), c) for t, c in tris]
    return [(t + (x, 0.0, z), c) for t, c in tris]


def export_cad_glb(out_glb, preset='neutral'):
    """Export vertex-colored CAD head mesh with smooth crease-grouped `NORMAL` attributes to glTF 2.0 `.glb`."""
    pos, nrm, _uv, mat, idx = face_built.lit_mesh(face_built.posed(preset))
    verts = pos.astype('<f4')
    nrms = (nrm / np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-9)).astype('<f4')
    vcols = np.c_[(mat >> 16) & 255, (mat >> 8) & 255, mat & 255, np.full(len(mat), 255, np.uint32)].astype(np.uint8)
    faces = idx.astype('<u4')
    pathlib.Path(out_glb).parent.mkdir(parents=True, exist_ok=True)
    vb, nb, cb, ib = verts.tobytes(), nrms.tobytes(), vcols.tobytes(), faces.reshape(-1).tobytes()
    pad_b = lambda b: b + b'\x00' * ((4 - len(b) % 4) % 4)
    vb_p, nb_p, cb_p, ib_p = pad_b(vb), pad_b(nb), pad_b(cb), pad_b(ib)
    bin_blob = vb_p + nb_p + cb_p + ib_p
    o_n, o_c, o_i = len(vb_p), len(vb_p) + len(nb_p), len(vb_p) + len(nb_p) + len(cb_p)
    gltf = {
        'asset': {'version': '2.0'}, 'scene': 0, 'scenes': [{'nodes': [0]}], 'nodes': [{'mesh': 0}],
        'meshes': [{'primitives': [{'attributes': {'POSITION': 0, 'NORMAL': 1, 'COLOR_0': 2}, 'indices': 3, 'mode': 4}]}],
        'buffers': [{'byteLength': len(bin_blob)}],
        'bufferViews': [
            {'buffer': 0, 'byteOffset': 0, 'byteLength': len(vb), 'target': 34962},
            {'buffer': 0, 'byteOffset': o_n, 'byteLength': len(nb), 'target': 34962},
            {'buffer': 0, 'byteOffset': o_c, 'byteLength': len(cb), 'target': 34962},
            {'buffer': 0, 'byteOffset': o_i, 'byteLength': len(ib), 'target': 34963},
        ],
        'accessors': [
            {'bufferView': 0, 'componentType': 5126, 'count': len(verts), 'type': 'VEC3',
             'min': verts.min(0).astype(float).tolist(), 'max': verts.max(0).astype(float).tolist()},
            {'bufferView': 1, 'componentType': 5126, 'count': len(nrms), 'type': 'VEC3'},
            {'bufferView': 2, 'componentType': 5121, 'normalized': True, 'count': len(vcols), 'type': 'VEC4'},
            {'bufferView': 3, 'componentType': 5125, 'count': int(faces.size), 'type': 'SCALAR'},
        ],
    }
    jb = json.dumps(gltf, separators=(',', ':')).encode('utf-8')
    jb += b' ' * ((4 - len(jb) % 4) % 4)
    with open(out_glb, 'wb') as f:
        f.write(struct.pack('<III', 0x46546C67, 2, 28 + len(jb) + len(bin_blob)))
        f.write(struct.pack('<II', len(jb), 0x4E4F534A) + jb)
        f.write(struct.pack('<II', len(bin_blob), 0x004E4942) + bin_blob)
    return {'path': str(out_glb), 'verts': int(len(verts)), 'faces': int(len(faces))}


def write_3mf(objects, path):
    """Write multi-object colored millimeter `.3mf` package (`[(name, tris_zup_mm, rgb)]`)."""
    ct_xml = ('<?xml version="1.0" encoding="UTF-8"?>\n'
              '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
              '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
              '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
    rl_xml = ('<?xml version="1.0" encoding="UTF-8"?>\n'
              '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
              '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
              'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    palette, col_list = {}, []
    for _nm, _ts, rgb in objects:
        hx = '#%02X%02X%02XFF' % (int(rgb[0]), int(rgb[1]), int(rgb[2]))
        if hx not in palette:
            palette[hx] = len(col_list)
            col_list.append(hx)
    res_xml = ['  <basematerials id="1">'] + [f'   <base name="mat_{i}" displaycolor="{hx}"/>' for i, hx in enumerate(col_list)] + ['  </basematerials>']
    items_xml = []
    for oid, (name, tris_mm, rgb) in enumerate(objects, start=2):
        pidx = palette['#%02X%02X%02XFF' % (int(rgb[0]), int(rgb[1]), int(rgb[2]))]
        raw = np.array([t for t, _c in tris_mm], np.float64).reshape(-1, 3)
        verts, inv = np.unique(np.round(raw, 4), axis=0, return_inverse=True)
        faces = inv.reshape(-1, 3)
        v_s = '\n'.join(f'     <vertex x="{v[0]:.4f}" y="{v[1]:.4f}" z="{v[2]:.4f}"/>' for v in verts)
        t_s = '\n'.join(f'     <triangle v1="{f[0]}" v2="{f[1]}" v3="{f[2]}" pid="1" p1="{pidx}"/>' for f in faces)
        res_xml.append(f'  <object id="{oid}" name="{name}" type="model">\n   <mesh>\n    <vertices>\n{v_s}\n    </vertices>\n    <triangles>\n{t_s}\n    </triangles>\n   </mesh>\n  </object>')
        items_xml.append(f'  <item objectid="{oid}"/>')
    model_xml = ('<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" '
                 'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">\n <resources>\n'
                 + '\n'.join(res_xml) + '\n </resources>\n <build>\n' + '\n'.join(items_xml) + '\n </build>\n</model>')
    pathlib.Path(path).parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, 'w', compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr('[Content_Types].xml', ct_xml)
        zf.writestr('_rels/.rels', rl_xml)
        zf.writestr('3D/3dmodel.model', model_xml)
    return str(path)


def _export_p1s_plates(by_group, plt_dir):
    """Pack filament groups onto Bambu Lab P1S plates, write `.3mf`/`.stl`, and render `p1s_build_plates.png`."""
    from tools.render import elbow_mount as em
    all_plates = [(grp, p_items) for grp in ('porcelain_white', 'amber_gold', 'graphite_dark', 'carbon_structural')
                  if grp in by_group for p_items in _pack_p1s_group(by_group[grp])]
    plate_manifest = []
    px, cols_grid = 680, min(3, max(1, len(all_plates)))
    sheet = Image.new('RGB', (px * cols_grid, (px + 38) * (-(-len(all_plates) // cols_grid))), (16, 18, 22))
    pen = ImageDraw.Draw(sheet)
    bed_sq = np.array([(0.0, 0.0), (PLATE_M, 0.0), (PLATE_M, PLATE_M), (0.0, PLATE_M)])
    cutter_sq = np.array([(0.0, 0.0), (CORNER_M[0], 0.0), (CORNER_M[0], CORNER_M[1]), (0.0, CORNER_M[1])])

    for p_idx, (grp, p_items) in enumerate(all_plates, start=1):
        p_tag = f'p1s_plate_{p_idx:02d}_{grp}'
        plate_yup_draw = (em._rounded(bed_sq, -0.003, 0.0, pr.BED, r=0.001, steps=1)
                          + em._rounded(cutter_sq, 0.0, 0.0006, (120, 46, 46), r=0.0005, steps=1))
        plate_3mf_objs, plate_stl_mm, pcs_meta, over_pcs = [], [], [], []
        for name, bed_yup, x, z, a, d, tl, rot90, orig_ac in p_items:
            placed_yup = _plate_item_tris_yup(bed_yup, x, z, rot90, orig_ac)
            plate_yup_draw += placed_yup
            placed_zup_mm = [(1e3 * np.stack([t[:, 0], t[:, 2], t[:, 1]], axis=1), c) for t, c in placed_yup]
            plate_3mf_objs.append((name, placed_zup_mm, placed_yup[0][1] if placed_yup else (200, 200, 200)))
            plate_stl_mm += placed_zup_mm
            if x + a > PLATE_M - AIR_M + 1e-6 or z + d > PLATE_M - AIR_M + 1e-6 or tl > TALL_M:
                over_pcs.append(name)
            pcs_meta.append({'name': name, 'at_mm': [round(1e3 * x, 1), round(1e3 * z, 1)],
                             'footprint_mm': [round(1e3 * a, 1), round(1e3 * d, 1)], 'tall_mm': round(1e3 * tl, 1)})
        write_3mf(plate_3mf_objs, plt_dir / f'{p_tag}.3mf')
        csg.stl(plate_stl_mm, str(plt_dir / f'{p_tag}.stl'), scale=1.0)
        plate_manifest.append({'plate': p_idx, 'filament': grp, 'file_3mf': f'{p_tag}.3mf',
                               'file_stl': f'{p_tag}.stl', 'piece_count': len(p_items),
                               'over_bounds': over_pcs, 'pieces': pcs_meta})
        view = m_draw.draw(plate_yup_draw, yaw=28.0, pitch=48.0, px=px, half=0.62 * PLATE_M,
                           centre=(0.5 * PLATE_M, 0.02, 0.5 * PLATE_M))
        gx, gy = px * ((p_idx - 1) % cols_grid), (px + 38) * ((p_idx - 1) // cols_grid)
        sheet.paste(view, (gx, gy + 38))
        pen.text((gx + 14, gy + 10), f'P1S PLATE {p_idx}/{len(all_plates)} [{grp.upper()}] ({len(p_items)} pcs, 256x256mm)',
                 fill=(232, 236, 244))
    sheet_path = plt_dir / 'p1s_build_plates.png'
    sheet.save(sheet_path)
    return plate_manifest, sheet_path


def build_all(out_dir):
    """Export screw-free panel STLs, standalone M2/M3 Torx screw STLs, assembly and Bambu P1S plates."""
    root = pathlib.Path(out_dir)
    pan_dir, hw_dir, asm_dir, plt_dir = root / 'panels', root / 'hardware', root / 'assembly', root / 'p1s_plates'
    for d in (pan_dir, hw_dir, asm_dir, plt_dir):
        d.mkdir(parents=True, exist_ok=True)

    m2_local, m3_local = fc.standard_torx_screw('M2'), fc.standard_torx_screw('M3')
    m2_mm = [(1e3 * (t + (0.0, 0.0, 0.0060)), c) for t, c in m2_local]
    m3_mm = [(1e3 * (t + (0.0, 0.0, 0.0080)), c) for t, c in m3_local]
    csg.stl(m2_mm, str(hw_dir / 'M2x6_ISO14580_Torx_T6.stl'), scale=1.0)
    csg.stl(m3_mm, str(hw_dir / 'M3x8_ISO14580_Torx_T10.stl'), scale=1.0)
    m2_audit, m3_audit = solids.printable([t for t, _ in m2_mm]), solids.printable([t for t, _ in m3_mm])

    panels, fastener_spots = assembled_panels(screw_free=True)
    panel_manifest, by_group, asm_objs_3mf, asm_all_yup = [], {}, [], []
    for idx, (name, (tris_torso, grp)) in enumerate(panels.items(), start=1):
        asm_all_yup += tris_torso
        asm_objs_3mf.append((name, _to_zup_mm(tris_torso), tris_torso[0][1] if tris_torso else (200, 200, 200)))
        bed_yup, sz = _lay_on_bed(tris_torso)
        bed_zup_mm = [(1e3 * np.stack([t[:, 0], t[:, 2], t[:, 1]], axis=1), c) for t, c in bed_yup]
        stl_path = pan_dir / f'{idx:02d}_{name}.stl'
        csg.stl(bed_zup_mm, str(stl_path), scale=1.0)
        audit = solids.printable([1e3 * t for t, _c in bed_yup])
        panel_manifest.append({'index': idx, 'name': name, 'group': grp, 'stl': stl_path.name,
                               'size_mm': [round(1e3 * sz[0], 1), round(1e3 * sz[1], 1), round(1e3 * sz[2], 1)],
                               'cm3': audit['cm3'], 'open_edges': audit['open']})
        by_group.setdefault(grp, []).append((name, bed_yup, sz))

    m2_placed, m3_placed = [], []
    for pt, nrm, spec in fastener_spots:
        (m2_placed if spec == 'M2' else m3_placed).extend(fc.place_screw(m2_local if spec == 'M2' else m3_local, pt, nrm))
    if m2_placed:
        asm_all_yup += m2_placed
        asm_objs_3mf.append(('fasteners_M2x6_ISO14580_Torx_T6', _to_zup_mm(m2_placed), fh.BOLT_RIM))
    if m3_placed:
        asm_all_yup += m3_placed
        asm_objs_3mf.append(('fasteners_M3x8_ISO14580_Torx_T10', _to_zup_mm(m3_placed), fh.BOLT_RIM))

    csg.stl(_to_zup_mm(asm_all_yup), str(asm_dir / 'head_and_neck_assembled.stl'), scale=1.0)
    write_3mf(asm_objs_3mf, asm_dir / 'head_and_neck_assembled.3mf')
    glb_info = export_cad_glb(asm_dir / 'head_and_neck_assembled.glb')
    plate_manifest, sheet_path = _export_p1s_plates(by_group, plt_dir)
    face_built.render_sheets(str(asm_dir))

    summary = {
        'panel_count': len(panel_manifest),
        'open_edge_panels': [p['name'] for p in panel_manifest if p['open_edges'] > 0],
        'hardware_fasteners': {
            'M2x6_ISO14580_Torx_T6': {'stl': str(hw_dir / 'M2x6_ISO14580_Torx_T6.stl'),
                                      'qty': sum(1 for _p, _n, sp in fastener_spots if sp == 'M2'),
                                      'head_diam_mm': 3.8, 'thread_mm': 2.0, 'length_mm': 6.0, 'open_edges': m2_audit['open']},
            'M3x8_ISO14580_Torx_T10': {'stl': str(hw_dir / 'M3x8_ISO14580_Torx_T10.stl'),
                                       'qty': sum(1 for _p, _n, sp in fastener_spots if sp == 'M3'),
                                       'head_diam_mm': 5.5, 'thread_mm': 3.0, 'length_mm': 8.0, 'open_edges': m3_audit['open']},
        },
        'p1s_plate_count': len(plate_manifest),
        'p1s_over_bounds': [n for pm in plate_manifest for n in pm['over_bounds']],
        'assembly_stl': str(asm_dir / 'head_and_neck_assembled.stl'),
        'assembly_3mf': str(asm_dir / 'head_and_neck_assembled.3mf'),
        'assembly_glb': glb_info, 'plates_sheet': str(sheet_path), 'plates': plate_manifest, 'panels': panel_manifest,
    }
    (root / 'manifest.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    return summary


def main(argv=None):
    """Run automated full-res convergence loop then generate all panel STLs, assembly and P1S plates."""
    ap = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--ref-dir', default=None)
    ap.add_argument('--converge', action='store_true')
    ap.add_argument('--rounds', type=int, default=1)
    ap.add_argument('--jobs', type=int, default=12)
    args = ap.parse_args(argv)

    os.environ.setdefault('COAXIAL_ROBOT', 'gynoid_mesomorph_medium_coax')
    if args.converge and args.ref_dir:
        from tools.render import face_contours as fcon, face_converge, face_hull as fhull
        conv_res, hist = face_converge.converge(args.ref_dir, os.path.join(args.out_dir, 'deltas'), args.rounds, args.jobs)
        _bake_tuning_into_source(conv_res['tuning'])
        fhull.bake_hull_into_source(conv_res['hull'])
        if conv_res.get('loops'):
            fcon.bake_contours_into_source(conv_res['loops'], conv_res.get('holes'))
        fin_s = hist[-1]['summary']
        print(f'[converged] stages={len(hist) - 1} view_match={fin_s["mean_view_match"] * 100:.4f}% '
              f'panel_match={fin_s["mean_panel_match"] * 100:.4f}% obj={fin_s["objective"]:.5f}')

    chk = face_built.checked()
    if not chk['ok']:
        print('ERROR: face_built.checked() failed:', chk, file=sys.stderr)
        return 1
    summary = build_all(args.out_dir)
    print(json.dumps({k: summary[k] for k in ('panel_count', 'open_edge_panels', 'hardware_fasteners', 'p1s_plate_count',
                                              'p1s_over_bounds', 'assembly_3mf', 'plates_sheet')}, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
