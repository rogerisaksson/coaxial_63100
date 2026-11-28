"""Full-resolution studio reference contours, elliptical bores, Torx screws and OLED mouth graphic."""
import base64
import json
import math
import os
import pathlib
import re
import struct
import zlib
from typing import Any

LOOP_NAMES = (
    'TIARA_LOOP', 'DIAMOND_LOOP', 'BROW_L_LOOP', 'BROW_R_LOOP', 'NOSE_LOOP',
    'ICHEEK_L_LOOP', 'ICHEEK_R_LOOP', 'OCHEEK_L_LOOP', 'OCHEEK_R_LOOP',
    'MOUTH_LOOP', 'CHIN_LOOP', 'EYE_CUP_L_LOOP', 'EYE_CUP_R_LOOP',
    'PROF_TEMPLE_LOOP', 'PROF_JAW_FRONT_LOOP', 'PROF_JAW_REAR_LOOP',
)

CONTOURS_PACKED = (
    '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'
    'Zn120A=='
)

PANEL_HOLES_PX: dict[str, tuple[tuple[float, ...], ...]] = {
    'TIARA_LOOP': ((368.8, 186.2, 4.80, 4.30, -0.333, 1), (536.1, 185.4, 4.80, 4.30, 0.591, 1), (705.5, 277.8, 4.60, 2.90, 1.124, 1), (198.8, 277.7, 4.60, 2.90, -1.045, 1)),
    'DIAMOND_LOOP': ((451.8, 238.8, 4.90, 4.80, -0.487, 1), (451.9, 344.7, 4.90, 4.80, 0.207, 1)),
    'BROW_R_LOOP': ((189.1, 365.2, 4.80, 3.20, -1.405, 1), (384.1, 403.3, 4.90, 4.80, -0.764, 1)),
    'BROW_L_LOOP': ((714.2, 366.9, 4.80, 3.20, 1.400, 1), (519.4, 403.2, 4.90, 4.80, -1.324, 1)),
    'NOSE_LOOP': ((451.3, 426.3, 5.10, 5.00, 0.372, 1), (411.1, 634.5, 5.10, 4.10, -1.182, 1), (420.7, 675.5, 5.20, 4.20, 0.083, 1), (478.9, 674.9, 5.20, 4.20, -0.112, 1), (449.4, 681.1, 3.80, 3.40, -1.187, 0)),
    'ICHEEK_R_LOOP': ((354.8, 582.4, 4.80, 4.60, 0.983, 1), (311.9, 658.0, 5.10, 4.70, 1.224, 1)),
    'ICHEEK_L_LOOP': ((542.8, 582.1, 4.80, 4.60, 0.974, 1), (588.4, 657.7, 5.10, 4.70, -1.048, 1)),
    'OCHEEK_R_LOOP': ((178.1, 431.4, 4.70, 3.10, 1.446, 1), (176.2, 582.1, 4.70, 3.10, 1.436, 1)),
    'OCHEEK_L_LOOP': ((724.6, 433.1, 4.70, 3.10, -1.323, 1), (726.0, 582.5, 4.70, 3.10, -1.421, 1)),
    'CHIN_LOOP': ((408.2, 903.9, 4.80, 4.10, 0.665, 1), (489.0, 904.2, 4.80, 4.10, -0.664, 1)),
}

def eval_nose_z_mm(x_mm, y_mm, zr, dz=0.0, tip_dz=0.0):
    """Evaluate C2-smooth compact mecha nose `Z(X, Y)` locked to dorsal `zr`."""
    ax, fy, fzr = math.hypot(float(x_mm), 1.15) - 1.15, float(y_mm), float(zr)
    za = 62.5 + 0.08 * max(0.0, fy - 135.0) - 0.06 * max(0.0, 125.0 - fy)
    wt = math.exp(-(((fy - 114.0) / 8.5) ** 2))
    rw, aw = 2.35 + 0.35 * wt, 9.65 + 2.05 * wt
    u = max(0.0, (ax - 0.6 * rw) / max(1.0, aw - 0.6 * rw))
    s = (u * u / (1.0 + u * u)) if u < 1.0 else (0.5 + 0.5 * math.tanh(1.15 * (u - 1.0)))
    zb = (fzr - 0.12 * ax) * (1.0 - 1.32 * min(0.75, s)) + (za + 1.0 - 0.68 * max(0.0, ax - aw)) * (1.32 * min(0.75, s))
    return zb + float(dz) + float(tip_dz) * wt

_UNPACK_CACHE: list[Any] = [None, None]
_ENV_CACHE: list[Any] = [None, None]


def _default_loops() -> dict[str, tuple[tuple[float, float], ...]]:
    return {nm: ((430.0, 400.0), (470.0, 400.0), (450.0, 430.0)) for nm in LOOP_NAMES}


def pack_contours_hex(loops_dict):
    """Byte-delta-pack `0.25-px` crack `{name: [(x, y), ...]}` into compact zlib-9 base64."""
    buf = bytearray(struct.pack('<H', len(LOOP_NAMES)))
    for nm in LOOP_NAMES:
        pts = loops_dict[nm]
        buf += struct.pack('<H', len(pts))
        px0 = py0 = 0
        for x, y in pts:
            ix, iy = int(round(float(x) * 4.0)), int(round(float(y) * 4.0))
            dx, dy = ix - px0, iy - py0
            if -126 <= dx <= 126 and -126 <= dy <= 126:
                buf += struct.pack('<bb', dx, dy)
            else:
                buf += struct.pack('<bhh', -128, max(-32767, min(32767, dx)), max(-32767, min(32767, dy)))
            px0, py0 = ix, iy
    return base64.b64encode(zlib.compress(bytes(buf), level=9)).decode('ascii')


def unpack_contours_hex(hx: str) -> dict[str, tuple[tuple[float, float], ...]]:
    """Unpack base64 byte-delta `CONTOURS_PACKED` string into `{name: ((x, y), ...)}`."""
    if not hx:
        return _default_loops()
    raw = zlib.decompress(base64.b64decode(hx))
    cnt = struct.unpack_from('<H', raw, 0)[0]
    idx, out = 2, {}
    for k in range(min(int(cnt), len(LOOP_NAMES))):
        n = struct.unpack_from('<H', raw, idx)[0]
        idx += 2
        pts, ax, ay = [], 0, 0
        for _ in range(int(n)):
            b0 = struct.unpack_from('<b', raw, idx)[0]
            if b0 == -128:
                dx, dy = struct.unpack_from('<hh', raw, idx + 1)
                idx += 5
            else:
                dx, dy = b0, struct.unpack_from('<b', raw, idx + 1)[0]
                idx += 2
            ax += int(dx)
            ay += int(dy)
            pts.append((0.25 * ax, 0.25 * ay))
        out[LOOP_NAMES[k]] = tuple(pts)
    for nm, dv in _default_loops().items():
        out.setdefault(nm, dv)
    return out


def active_loop(name):
    """Return active full-resolution contour tuple `name` merged with `FACE_CONTOURS_JSON`."""
    if _UNPACK_CACHE[0] != CONTOURS_PACKED or _UNPACK_CACHE[1] is None:
        _UNPACK_CACHE[0], _UNPACK_CACHE[1] = CONTOURS_PACKED, unpack_contours_hex(CONTOURS_PACKED)
    raw = os.environ.get('FACE_CONTOURS_JSON')
    if not raw:
        return _UNPACK_CACHE[1][name]
    if _ENV_CACHE[0] != raw or _ENV_CACHE[1] is None:
        parsed = json.loads(raw)
        _ENV_CACHE[0] = raw
        _ENV_CACHE[1] = unpack_contours_hex(parsed['packed']) if 'packed' in parsed else parsed
    if name in _ENV_CACHE[1]:
        return tuple((float(p[0]), float(p[1])) for p in _ENV_CACHE[1][name])
    return _UNPACK_CACHE[1][name]


def active_holes(name):
    """Return native counterbore / recess specs for `name`."""
    return PANEL_HOLES_PX.get(name, ())


def dumps_contours(loops_dict):
    """Serialize `loops_dict` into a compact JSON string for `FACE_CONTOURS_JSON`."""
    cur = {nm: active_loop(nm) for nm in LOOP_NAMES}
    cur.update(loops_dict)
    return json.dumps({'packed': pack_contours_hex(cur)}, separators=(',', ':'))


def _fit_hole_ellipse(cm, edt):
    """Fit exact area-preserving `(cx, cy, rx, ry, theta, 1.0)` in crack coordinates to hole mask `cm`."""
    from tools.render import face_honeycomb as fh
    np = fh.np_mod()
    ys_h, xs_h = np.nonzero(cm)
    area = float(len(xs_h))
    px, py = xs_h.astype(float) + 0.5, ys_h.astype(float) + 0.5
    cx, cy = float(px.mean()), float(py.mean())
    dx, dy = px - cx, py - cy
    cxx = float(np.mean(dx * dx)) + 1.0 / 12.0
    cyy = float(np.mean(dy * dy)) + 1.0 / 12.0
    cxy = float(np.mean(dx * dy))
    tr, det = cxx + cyy, max(0.0, cxx * cyy - cxy * cxy)
    disc = math.sqrt(max(0.0, 0.25 * tr * tr - det))
    l1, l2 = max(0.25, 0.5 * tr + disc), max(0.25, 0.5 * tr - disc)
    th = 0.5 * math.atan2(2.0 * cxy, cxx - cyy)
    rx0, ry0 = 2.0 * math.sqrt(l1), 2.0 * math.sqrt(l2)
    sc = math.sqrt(area / max(1e-6, math.pi * rx0 * ry0))
    d_ed = max(2.0, float(edt[int(min(edt.shape[0] - 1, round(cy - 0.5))), int(min(edt.shape[1] - 1, round(cx - 0.5)))]) - 0.85)
    rx, ry = min(d_ed, rx0 * sc), min(d_ed, ry0 * sc)
    return (round(cx, 2), round(cy, 2), round(rx, 2), round(ry, 2), round(th, 3))


def seeded_comp(elig, sx, sy, radius=16, hole_area=0):
    """Extract connected component nearest `(sx, sy)` within `radius` (optionally filling interior holes)."""
    from scipy import ndimage
    from tools.render import face_honeycomb as fh, face_ray as fr
    np, (h, w) = fh.np_mod(), elig.shape
    rx, ry = int(round(sx)), int(round(sy))
    y0, y1, x0, x1 = max(0, ry - radius), min(h, ry + radius + 1), max(0, rx - radius), min(w, rx + radius + 1)
    ys, xs = np.nonzero(elig[y0:y1, x0:x1])
    if not len(xs):
        return np.zeros((h, w), dtype=bool)
    k = int(np.argmin(np.hypot(xs + x0 - rx, ys + y0 - ry)))
    res_lbl: Any = ndimage.label(elig)
    lbl = np.asarray(res_lbl[0], dtype=np.int64)
    cid = int(lbl[int(ys[k]) + y0, int(xs[k]) + x0])
    comp = (lbl == cid) if cid > 0 else np.zeros((h, w), dtype=bool)
    return fr.fill_interior_holes(comp, hole_area) if hole_area > 0 else comp


def segment_front_and_side(rgba, preset_id):
    """Segment ground-truth panel masks on `'front'` or `'profile90'` studio reference RGBA."""
    from tools.render import face_honeycomb as fh, face_ray as fr
    np = fh.np_mod()
    h, w = rgba.shape[:2]
    r, g, b, a = rgba[..., 0].astype(float), rgba[..., 1].astype(float), rgba[..., 2].astype(float), rgba[..., 3]
    fg = a > 127
    lum = 0.299 * r + 0.587 * g + 0.114 * b
    is_wh = fg & (lum > 98.0) & ((r - b) < 36.0)
    is_am = fg & (r > 80.0) & ((r - b) > 36.0) & (g > 34.0) & (r > g * 1.12)
    is_am_hi = fg & (r > 82.0) & ((r - b) > 38.0) & (g > 36.0) & (r > g * 1.12)
    yy, xx = np.mgrid[0:h, 0:w]
    from scipy import ndimage
    if preset_id == 'profile90':
        jf_s = seeded_comp(is_am_hi & (xx >= 638) & (yy >= 470), 720, 505, 20, 22000)
        ic_s = seeded_comp(is_wh & (yy <= 558), 760, 485, 24, 1600)
        m_box = (yy >= 534) & (yy <= 652) & (xx >= 774) & (~jf_s) & (~ic_s)
        m_w = fg & (lum > 54.0) & ((r - b) < 44.0) & m_box
        m_er = seeded_comp(np.asarray(ndimage.binary_erosion(m_w), dtype=bool), 830, 595, 18, 0)
        m_s = fr.fill_interior_holes(np.asarray(ndimage.binary_dilation(m_er), dtype=bool) & m_w, 4200)
        return {
            'temple-hex-bezels': seeded_comp(is_am_hi & (yy <= 368), 475, 170, 20, 38000),
            'jaw-rear-hex': seeded_comp(is_am_hi & (xx <= 718) & (yy >= 430), 625, 460, 20, 22000),
            'jaw-front-hex': jf_s,
            'forehead-wings': seeded_comp(is_am & (xx <= 875), 820, 145, 18, 1600),
            'face-oled-mouth': m_s,
        }
    diam = seeded_comp(is_am, 451, 290, 16, 0)
    wings = seeded_comp(is_am & (~diam) & (yy <= 330), 310, 200, 16, 0)
    brows = seeded_comp(is_wh, 295, 360, 16, 0) | seeded_comp(is_wh, 605, 360, 16, 0)
    nose = seeded_comp(is_wh, 450, 565, 16, 0)
    icheek = seeded_comp(is_wh, 345, 535, 18, 0) | seeded_comp(is_wh, 555, 535, 18, 0)
    ocheek = seeded_comp(is_wh, 225, 540, 16, 0) | seeded_comp(is_wh, 675, 540, 16, 0)
    am_t = fg & (r > 38.0) & ((r - b) > 12.0) & (g > 18.0) & (r > g * 1.06) & (~wings) & (~brows) & (~ocheek)
    m_in = (yy >= 725) & (yy <= 846) & (xx >= 342) & (xx <= 559) & fg & (lum > 64.0)
    m_cand = (yy >= 680) & (yy <= 872) & (xx >= 268) & (xx <= 630) & (is_wh | m_in) & (~icheek) & (~ocheek) & (~nose)
    m_er = seeded_comp(np.asarray(ndimage.binary_erosion(m_cand), dtype=bool), 450, 780, 18, 0)
    m_front = fr.fill_interior_holes(np.asarray(ndimage.binary_dilation(m_er), dtype=bool) & m_cand, 8000)
    c_box = (yy >= 862) & (yy <= 928) & (xx >= 380) & (xx <= 520)
    e_el = ((yy >= 452) & (yy <= 552) & (xx >= 210) & (xx <= 690) & fg
            & (~fr.fill_interior_holes(brows, 2200)) & (~fr.fill_interior_holes(nose, 2200))
            & (~fr.fill_interior_holes(icheek, 2200)) & (~fr.fill_interior_holes(ocheek, 2200)))
    return {
        'forehead-diamond': diam, 'forehead-wings': wings, 'face-brows': brows, 'face-nose': nose,
        'face-inner-cheeks': icheek, 'face-outer-cheeks': ocheek,
        'temple-hex-bezels': (seeded_comp((is_am_hi | am_t) & (xx <= 174) & (yy >= 228) & (yy <= 484), 135, 320, 18, 26000)
                              | seeded_comp((is_am_hi | am_t) & (xx >= 728) & (yy >= 228) & (yy <= 484), 765, 320, 18, 26000)),
        'face-oled-mouth': m_front,
        'jaw-chin-cap': seeded_comp(c_box & fg & (r > 60.0) & ((r - b) > 26.0) & (g > 26.0), 450, 895, 16, 0),
        'face-camera-eyes': seeded_comp(e_el, 345, 502, 20, 900) | seeded_comp(e_el, 555, 502, 20, 900),
    }


def extract_reference_contours_and_holes(ref_dir):
    """Extract sub-pixel full-resolution (`896x1200` / `1200x896`) crack contours & elliptical bores."""
    from PIL import Image
    from scipy import ndimage
    from tools.render import face_honeycomb as fh, face_ray as fr
    np, ref_p = fh.np_mod(), pathlib.Path(ref_dir)
    im_f = np.asarray(Image.open(ref_p / 'robot 1_front.png').convert('RGBA'), np.uint8)
    im_s = np.asarray(Image.open(ref_p / 'robot 1_side.png').convert('RGBA'), np.uint8)
    seg_f, seg_s = segment_front_and_side(im_f, 'front'), segment_front_and_side(im_s, 'profile90')
    yy, xx = np.mgrid[0:1200, 0:896]
    raw_front = {
        'TIARA_LOOP': seg_f['forehead-wings'], 'DIAMOND_LOOP': seg_f['forehead-diamond'],
        'BROW_R_LOOP': seg_f['face-brows'] & (xx < 450), 'BROW_L_LOOP': seg_f['face-brows'] & (xx >= 450),
        'NOSE_LOOP': seg_f['face-nose'],
        'ICHEEK_R_LOOP': seg_f['face-inner-cheeks'] & (xx < 450), 'ICHEEK_L_LOOP': seg_f['face-inner-cheeks'] & (xx >= 450),
        'OCHEEK_R_LOOP': seg_f['face-outer-cheeks'] & (xx < 450), 'OCHEEK_L_LOOP': seg_f['face-outer-cheeks'] & (xx >= 450),
        'MOUTH_LOOP': seg_f['face-oled-mouth'], 'CHIN_LOOP': seg_f['jaw-chin-cap'],
    }
    filled_map, holes_out = {}, {}
    for nm, m in raw_front.items():
        fm = fr.fill_interior_holes(m, 2200)
        filled_map[nm] = fm
        if nm != 'MOUTH_LOOP':
            edt = np.asarray(ndimage.distance_transform_edt(fm), dtype=float)
            res_h: Any = ndimage.label(fm & (~m))
            lh, nh = np.asarray(res_h[0], dtype=np.int64), int(res_h[1])
            hs = []
            for i in range(1, nh + 1):
                cm = lh == i
                if 18 <= int(cm.sum()) <= 1600:
                    cx_h, cy_h, rx_h, ry_h, th_h = _fit_hole_ellipse(cm, edt)
                    is_m2 = 0.0 if (nm == 'NOSE_LOOP' and cy_h > 678.0) else 1.0
                    if min(rx_h, ry_h) >= 2.1 and all(math.hypot(cx_h - o[0], cy_h - o[1]) > max(rx_h, ry_h) + max(o[2], o[3]) + 2.5 for o in hs):
                        hs.append((cx_h, cy_h, rx_h, ry_h, th_h, is_m2))
            if hs:
                holes_out[nm] = tuple(hs)

    ec = seg_f['face-camera-eyes']
    ec_r, ec_l = fr.fill_interior_holes(ec & (xx < 450), 900), fr.fill_interior_holes(ec & (xx >= 450), 900)
    loops = {nm: fr.trace_exact_crack_polygon(m, rdp_eps=0.42) for nm, m in filled_map.items()}
    loops['EYE_CUP_R_LOOP'] = fr.trace_exact_crack_polygon(ec_r, rdp_eps=0.55)
    loops['EYE_CUP_L_LOOP'] = fr.trace_exact_crack_polygon(ec_l, rdp_eps=0.55)
    for nm, pm in (('PROF_TEMPLE_LOOP', seg_s['temple-hex-bezels']),
                   ('PROF_JAW_FRONT_LOOP', seg_s['jaw-front-hex']),
                   ('PROF_JAW_REAR_LOOP', seg_s['jaw-rear-hex'])):
        raw_p = fr.trace_exact_crack_polygon(pm, rdp_eps=0.55)
        poly = [raw_p[0]] if raw_p else []
        for pt in raw_p[1:]:
            if math.hypot(pt[0] - poly[-1][0], pt[1] - poly[-1][1]) >= 5.2:
                poly.append(pt)
        while len(poly) > 5 and math.hypot(poly[0][0] - poly[-1][0], poly[0][1] - poly[-1][1]) < 5.2:
            poly.pop()
        loops[nm] = poly
    return loops, holes_out


def bake_contours_into_source(loops_dict, holes_dict=None):
    """Write `CONTOURS_PACKED` and `PANEL_HOLES_PX` into `face_contours.py`."""
    global CONTOURS_PACKED, PANEL_HOLES_PX
    cur = {nm: (loops_dict[nm] if nm in loops_dict else active_loop(nm)) for nm in LOOP_NAMES}
    packed = pack_contours_hex(cur)
    cp_txt = 'CONTOURS_PACKED = (\n' + '\n'.join(f"    '{packed[i:i + 92]}'" for i in range(0, len(packed), 92)) + '\n)'
    p = pathlib.Path(__file__)
    src = re.sub(r"CONTOURS_PACKED\s*=\s*(?:''|\(\n(?:\s+'[^']*'\n)+\))", cp_txt, p.read_text(encoding='utf-8'), count=1)
    if holes_dict:
        hl = ['PANEL_HOLES_PX: dict[str, tuple[tuple[float, ...], ...]] = {'] + [
            f"    '{k}': ({', '.join(f'({h[0]:.1f}, {h[1]:.1f}, {h[2]:.2f}, {h[3]:.2f}, {h[4]:.3f}, {h[5]:.0f})' for h in vs)},),"
            for k, vs in holes_dict.items()] + ['}']
        src = re.sub(r'PANEL_HOLES_PX:\s*dict\[[^\n]+\n(?:\s+[^\n]+\n)*\}', '\n'.join(hl), src, count=1)
        PANEL_HOLES_PX = dict(holes_dict)
    p.write_text(src, encoding='utf-8', newline='\n')
    CONTOURS_PACKED = packed
    _UNPACK_CACHE[:] = [None, None]
    _ENV_CACHE[:] = [None, None]


def build_lash_ribbon(eval_rim, outer_px, inner_px, front_off, back_off):
    """Build swept dark cat-eye lash ribbon along `eval_rim` from native `(px, py)` curves."""
    from tools.render import face_honeycomb as fh, face_ray as fr
    np, fcam = fh.np_mod(), fr.CANONICAL_CAMERAS['front']
    ts = np.linspace(0.0, 1.0, 19)
    ao, ai = np.asarray(outer_px, float), np.asarray(inner_px, float)
    so, si = np.linspace(0.0, 1.0, len(ao)), np.linspace(0.0, 1.0, len(ai))
    g = np.zeros((len(ts), 3, 3))
    for i, t in enumerate(ts):
        ft = float(t)
        ox, oy = float(np.interp(ft, so, ao[:, 0])), float(np.interp(ft, so, ao[:, 1]))
        ix, iy = float(np.interp(ft, si, ai[:, 0])), float(np.interp(ft, si, ai[:, 1]))
        g[i, 0] = fr.canon_to_head(fr.project_ray_z(ox, oy, fcam, eval_rim, front_off - 0.12)[0])
        g[i, 1] = fr.canon_to_head(fr.project_ray_z(0.5 * (ox + ix), 0.5 * (oy + iy), fcam, eval_rim, front_off)[0])
        g[i, 2] = fr.canon_to_head(fr.project_ray_z(ix, iy, fcam, eval_rim, front_off - 0.08)[0])
    sgn = 1.0 if float(ao[:, 0].mean()) < 450.0 else -1.0
    return fh.shell_patch(g, abs(float(back_off - front_off)) * fr.SCALE_M, 0.00014, fh.BAFFLE_DARK,
                          (sgn * 0.032, 0.098, 0.060), fh.BAFFLE_DARK)


def build_oled_mouth_pixels(eval_mouth_z, smile=0.0, jaw_open=0.0):
    """Render shaded upper lip, lower lip and oral cleft onto the sculpted OLED surface."""
    from tools.render import face_honeycomb as fh, face_ray as fr
    np = fh.np_mod()
    sm, jw = max(-1.0, min(1.0, float(smile))), max(0.0, min(1.0, float(jaw_open)))
    yc0, open_h, half_w = 84.8, 4.6 * jw, 14.4 + 2.0 * sm - 1.3 * jw * (1.0 - max(0.0, sm))
    us = np.linspace(-1.0, 1.0, 19)
    g_ul, g_ll, g_in = np.zeros((19, 4, 3)), np.zeros((19, 4, 3)), np.zeros((19, 5, 3))
    oled_pt = lambda xm, ym, dz_mm: fr.canon_to_head((xm, ym, eval_mouth_z(xm, ym) + dz_mm))
    for i, u in enumerate(us):
        fu, au = float(u), abs(float(u))
        xm, env = half_w * fu, max(0.0, 1.0 - au * au)
        yc = yc0 + (1.5 + 3.4 * sm) * (au ** 2.1)
        y_up, y_lo = yc + 0.38 * open_h * (env ** 0.75), yc - 0.62 * open_h * (env ** 0.78)
        cupid = (3.7 - 0.4 * jw) * (env ** 0.78) * (1.0 - 0.28 * math.exp(-((au / 0.15) ** 2)))
        low_h = (3.8 - 0.4 * jw) * (env ** 0.80)
        for j, fv in enumerate(np.linspace(0.0, 1.0, 4)):
            g_ul[i, j], g_ll[i, j] = oled_pt(xm, y_up + float(fv) * cupid, 0.46), oled_pt(xm, y_lo - float(fv) * low_h, 0.44)
        for j, fv in enumerate(np.linspace(0.0, 1.0, 5)):
            g_in[i, j] = oled_pt(xm, y_lo + float(fv) * max(0.65 * (0.35 + 0.65 * env), y_up - y_lo), 0.52)
    tris = (fh.shell_patch(g_ul, 0.00020, 0.00004, (158, 138, 148), (0.0, 0.030, 0.040), fh.MOUTH_SCREEN)
            + fh.shell_patch(g_ll, 0.00020, 0.00004, (194, 174, 184), (0.0, 0.030, 0.040), fh.MOUTH_SCREEN))
    cav = (52, 38, 46) if jw > 0.12 else (106, 90, 98)
    return tris + fh.shell_patch(g_in, 0.00018, 0.00003, cav, (0.0, 0.030, 0.040), (158, 138, 148))


def eval_jaw_front_x(y_mm, z_mm):
    """Concave mandible `|X|(Y, Z)` meeting `jaw-chin-cap` cleanly without overlapping porcelain."""
    from tools.render import face_hull as fhull
    fy, fz = float(y_mm), float(z_mm)
    m_ed = (27.2 + 0.35 * (fy - 80.0)) if fy >= 80.0 else (13.4 + 13.8 * (max(0.0, (fy - 68.5) / 11.5) ** 0.88))
    x_f = (m_ed + 3.45) if fy >= 70.5 else (13.4 + 3.4 * max(0.0, (fy - 61.0) / 9.5))
    z_r, z_f = 4.0 + 0.36 * max(0.0, fy - 78.0), 27.6 + 11.5 * (max(0.0, min(1.0, (94.0 - fy) / 26.0)) ** 1.20)
    x_r = (min(fhull.front_jaw_max_x(fy, fz) - 0.25, 40.6 + 0.32 * (fy - 79.0))
           if fy >= 79.0 else (14.4 + 26.2 * ((max(0.0, fy - 61.0) / 18.0) ** 1.28)))
    u_z = max(0.0, min(1.0, (fz - z_r) / max(4.5, z_f - z_r)))
    return max(13.2, x_f + (max(x_f + 1.2, x_r) - x_f) * ((1.0 - u_z) ** 1.32) + 0.55 * fhull.panel_offset('jaw-front-hex', fy, fz))
