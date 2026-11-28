"""Calibrated 360-deg 4-view cranial surface, crown tiles, temple wrap-around, X-truss and neck."""
import functools
import json
import math
import os
import sys
from typing import Any

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg, face_contours as fcon, face_hull as fhull, face_ray as fr  # noqa: E402
from tools.render.face_honeycomb import (
    AMBER_DARK, AMBER_GOLD, BAFFLE_DARK, BOLT_RIM, CRANIUM_DARK, CRANIUM_RIM, FRAME_VOID,
    HEX_DARK, TITANIUM, curved_honeycomb_bay, curved_poly_plate, mirror_x, np_mod, place_screw, socket_bolt,
    standard_torx_screw, throat_bellows_hoses, turn_tris, rot,
)

TUNING = {
    'crown_top': 87.8500, 'rx_scale': 1.0080, 'zf_scale': 1.0120, 'zb_scale': 1.0460,
    'diam_z0': 0.0000, 'diam_sx': 0.1400, 'diam_sy': 0.0400, 'wing_zi': 0.0000,
    'wing_zo': 0.1255, 'wing_sx': 1.4650, 'brow_zm': 0.0000, 'brow_zt': 0.2550,
    'brow_sx': 1.4500, 'icheek_z0': 78.4000, 'icheek_sx': 1.0250, 'ocheek_z0': 94.6000,
    'ocheek_sx': 1.5180, 'nose_dz': -5.2500, 'nose_wx': 1.0000, 'nose_tip': -3.4500,
    'oled_z0': 0.0000, 'oled_sx': 1.0000, 'oled_sy': 1.3800, 'chin_z0': 0.0000,
    'chin_sx': 0.7800, 'chin_sy': 0.0000, 'jaw_f_x0': 13.6000, 'jaw_f_sy': 0.4100,
    'jaw_f_sz': 0.5300, 'jaw_r_x0': 5.9500, 'jaw_r_sy': 0.4700, 'jaw_r_sz': 0.3600,
    'temp_xa': 68.6800, 'temp_xp': 67.8500, 'crown_off': -0.7800, 'occ_off': -0.8200,
    'ear_phi': 1.8320, 'ear_y': -13.0000, 'ear_inset': 0.0000, 'ear_r': 19.1000,
    'spine_dz': 0.6000, 'blade_r': 33.2000, 'bust_rx': 1.0400, 'cam_f_dy': 0.0000,
    'cam_s_dz': -0.1600, 'cam_p_dx': 0.0000, 'cam_b_dy': -1.2000,
}

_CFG_CACHE: list[Any] = ['', TUNING]
_NECK_CACHE: dict[Any, Any] = {}


def _active_cam_delta(pid):
    p, d = cfg(), list(fhull.cam_delta_for(pid))
    i, k = {'front': (2, 'cam_f_dy'), 'profile90': (3, 'cam_s_dz'), 'threeQuarter': (1, 'cam_p_dx')}.get(pid, (2, 'cam_b_dy'))
    d[i] += p[k]
    return tuple(d)


fr._CAM_DELTA_FN[0] = _active_cam_delta


def _xform_temple(pts, raw):
    cx, cy = sum(p[0] for p in raw) / len(raw), sum(p[1] for p in raw) / len(raw)
    ct, st = math.cos(math.radians(1.40)), math.sin(math.radians(1.40))
    return tuple((cx + 11.8 + (p[0] - cx) * 0.915 * ct - (p[1] - cy) * 0.985 * st,
                  cy - 0.3 + (p[0] - cx) * 0.915 * st + (p[1] - cy) * 0.985 * ct, *p[2:]) for p in pts)


def temple_prof_loop():
    """Return rotated and scaled `PROF_TEMPLE_LOOP` aligned across `FRONT` and `PERSP`."""
    raw = fcon.active_loop('PROF_TEMPLE_LOOP')
    return _xform_temple(raw, raw)


def inside_prof_temple(y_mm, z_mm):
    """True when `(y_mm, z_mm)` projects inside rotated `temple_prof_loop` (`+3%` margin)."""
    return fhull.inside_profile_poly(y_mm, z_mm, temple_prof_loop())


def cfg() -> dict[str, float]:
    """Return active `TUNING` merged with `FACE_TUNE_JSON` when set."""
    raw = os.environ.get('FACE_TUNE_JSON')
    if not raw:
        return TUNING
    if _CFG_CACHE[0] != raw:
        _CFG_CACHE[:] = [raw, {**TUNING, **json.loads(raw)}]
    return dict(_CFG_CACHE[1])


def cache_key():
    """Return hashable cache key `(tune_signature, hull_sig, loop_sig, screw_free)` for CAD caches."""
    from tools.render import face_honeycomb as fh_mod
    raw = os.environ.get('FACE_TUNE_JSON') or ''
    lraw = (os.environ.get('FACE_CONTOURS_JSON') or '') + fcon.CONTOURS_PACKED
    return (raw, tuple(TUNING.values()) if not raw else (), fhull.hull_cache_sig(), lraw, bool(fh_mod.SCREW_FREE_PANELS))


def _sstep(e0, e1, x):
    t = max(0.0, min(1.0, (x - e0) / (e1 - e0)))
    return t * t * (3.0 - 2.0 * t)


def _eval_head_mm(phi, y_loc, dr_sc=1.0):
    p, sin_p, cos_p = cfg(), math.sin(phi), math.cos(phi)
    ap, c_start, c_top = abs(math.atan2(sin_p, cos_p)), 8.0, p['crown_top']
    u_d = max(0.0, min(0.9994, (y_loc - c_start) / (c_top - c_start)))
    exp_d = 2.34 - 0.36 * _sstep(1.45, 2.25, ap)
    dome_s = (max(1e-4, 1.0 - (u_d ** exp_d)) ** (1.0 / exp_d)) if y_loc > c_start else 1.0
    rx = 67.4 * dome_s
    if y_loc < c_start:
        rx = 67.4 + (58.8 - 67.4) * _sstep(8.0, -28.0, y_loc)
        rx += (42.0 - rx) * _sstep(-26.0, -68.0, y_loc)
        rx += (22.0 - rx) * _sstep(-66.0, -86.0, y_loc)
        occ_rx = 66.4 - 9.5 * _sstep(-6.0, -52.0, y_loc) - 9.0 * _sstep(-46.0, -72.0, y_loc)
        rx += (occ_rx - rx) * _sstep(1.38, 2.05, ap)
    dr = (dr_sc * fhull.head_dr_mm(phi, y_loc) * _sstep(0.35, 0.90, ap)) if dr_sc else 0.0
    rx = rx * p['rx_scale'] + dr * abs(sin_p)
    x_out, fw = rx * sin_p, _sstep(math.pi * 0.56, math.pi * 0.22, ap)
    zf = ((69.2 - 3.2 * math.exp(-(((y_loc + 3.5) / 11.0) ** 2))
           - 6.5 * _sstep(-36.0, -72.0, y_loc) - 13.5 * _sstep(-68.0, -88.0, y_loc)) * dome_s * p['zf_scale'])
    zb = (((94.6 - 3.4 * _sstep(20.0, 66.0, y_loc)) * dome_s) if y_loc > c_start
          else (94.6 - 8.5 * _sstep(-6.0, -48.0, y_loc) - 10.5 * _sstep(-44.0, -72.0, y_loc))) * p['zb_scale']
    z = (zb + (zf - zb) * fw + dr) * cos_p - 5.2 * _sstep(-22.0, 82.0, y_loc)
    if ap < 1.35 and -90.0 < y_loc < 18.0:
        z -= 13.8 * _sstep(0.14, 0.92, ap) * _sstep(1.34, 0.96, ap) * _sstep(18.0, -14.0, y_loc)
        z -= 6.8 * math.exp(-(((ap - 0.42) / 0.22) ** 2 + ((y_loc + 5.5) / 8.2) ** 2))
    return (x_out, fr.HEAD_CENTER_Y + y_loc + 0.68 * dr * _sstep(48.0, 82.0, y_loc), z)


def eval_head_surface_mm(phi, y_loc):
    return _eval_head_mm(phi, y_loc, 1.0)


def sample_head_surface_mm(phi, y_loc):
    np, ct = np_mod(), cfg()['crown_top'] - 0.1
    p0 = np.asarray(_eval_head_mm(phi, y_loc, 1.0), float)
    dp = np.asarray(_eval_head_mm(phi + 0.008, y_loc, 0.15), float) - np.asarray(_eval_head_mm(phi - 0.008, y_loc, 0.15), float)
    dy = np.asarray(_eval_head_mm(phi, min(ct, y_loc + 0.4), 0.15), float) - np.asarray(_eval_head_mm(phi, max(-89.5, y_loc - 0.4), 0.15), float)
    n = np.cross(dp, dy)
    ln = float(np.linalg.norm(n))
    return p0, (n / ln if ln >= 1e-6 else np.array([math.sin(phi), 0.5, math.cos(phi)], float))


def _eval_crown_dome_mm(s_rad, c_rad, dr_sc=1.0):
    p = cfg()
    ux, cos_c = math.sin(c_rad), math.cos(c_rad)
    cs = cos_c * math.cos(s_rad)
    exp_y = 0.78 + 0.20 * _sstep(-0.38, 0.38, s_rad)
    uy = (cs ** exp_y) if cs >= 0.0 else -((-cs) ** 0.88)
    uz, tl, yl = cos_c * math.sin(s_rad), _sstep(-1.62, -2.02, s_rad), 8.0 + (p['crown_top'] - 9.2) * uy
    phi_c = math.atan2(ux, uz)
    w_dr = 1.0 - (1.0 - _sstep(0.15, -0.55, s_rad)) * (1.0 - _sstep(0.35, 0.78, abs(c_rad)) * _sstep(0.95, 0.35, s_rad))
    dr = (dr_sc * fhull.head_dr_mm(phi_c, yl) * w_dr) if dr_sc else 0.0
    sag_t = _sstep(-0.48, -1.03, s_rad) * max(0.0, 1.0 - (c_rad / 0.30) ** 2)
    occ_w = _sstep(-0.98, -1.36, s_rad) * (1.0 - 0.25 * _sstep(-1.72, -2.02, s_rad))
    ux_s = (math.hypot(ux, 0.035) ** 0.85) - (0.035 ** 0.85)
    af = 70.6 - 34.4 * ux_s
    ab = (95.8 - 3.6 * sag_t + 3.8 * occ_w - 4.2 * tl) * p['zb_scale'] + dr
    return ((68.2 - 8.5 * tl + 0.85 * dr) * p['rx_scale'] * ux, fr.HEAD_CENTER_Y + yl + 0.95 * dr * max(0.0, uy), -4.5 + (ab + (af - ab) * _sstep(-0.55, 0.45, uz)) * uz)


def sample_crown_dome_mm(s_rad, c_rad):
    np, eps = np_mod(), 0.008
    p0 = np.asarray(_eval_crown_dome_mm(s_rad, c_rad, 1.0), float)
    dc = np.asarray(_eval_crown_dome_mm(s_rad, c_rad + eps, 0.45), float) - np.asarray(_eval_crown_dome_mm(s_rad, c_rad - eps, 0.45), float)
    dns = np.asarray(_eval_crown_dome_mm(s_rad - eps, c_rad, 0.45), float) - np.asarray(_eval_crown_dome_mm(s_rad + eps, c_rad, 0.45), float)
    n = np.cross(dc, dns)
    ln = float(np.linalg.norm(n))
    return p0, (n / ln if ln >= 1e-6 else np.array([0.0, 1.0, 0.0], float))


_SEAM_AB = ((0.845, -0.035), (0.312, 0.022), (0.256, 0.020), (-0.365, 0.014),
            (-0.418, 0.014), (-1.080, 0.195),
            (-1.132, 0.195), (-1.510, 0.100), (-1.565, 0.100), (-1.925, -0.008))


def _s_seam(row, c):
    a, b = _SEAM_AB[row]
    return a + b * float(c) * float(c)


def _sc_band_plate(r_hi, r_lo, c0, c1, off_mm):
    np = np_mod()
    p00, p01 = sample_crown_dome_mm(_s_seam(r_hi, c0), c0)[0], sample_crown_dome_mm(_s_seam(r_hi, c1), c1)[0]
    p10, p11 = sample_crown_dome_mm(_s_seam(r_lo, c0), c0)[0], sample_crown_dome_mm(_s_seam(r_lo, c1), c1)[0]
    hs = 0.25 * float(np.linalg.norm(p10 - p00) + np.linalg.norm(p11 - p01))
    hw0, hw1 = 0.5 * float(np.linalg.norm(p01 - p00)), 0.5 * float(np.linalg.norm(p11 - p10))

    def eval_sc(u, v):
        tv = max(-0.04, min(1.04, 0.5 - float(v) / max(1e-3, 2.0 * hs)))
        tu = max(-0.04, min(1.04, 0.5 + float(u) / max(1e-3, 2.0 * (hw0 * (1.0 - tv) + hw1 * tv))))
        cr = c0 + (c1 - c0) * tu
        return sample_crown_dome_mm(_s_seam(r_hi, cr) * (1.0 - tv) + _s_seam(r_lo, cr) * tv, cr)

    tris = curved_poly_plate([(-hw0, hs), (hw0, hs), (hw1, -hs), (-hw1, -hs)],
                             lambda u, v, w: fr.canon_to_head(eval_sc(u, v)[0] + (off_mm + w / fr.SCALE_M) * eval_sc(u, v)[1]),
                             wall=4.65 * fr.SCALE_M, chamfer=0.88 * fr.SCALE_M, corner_r=1.65, col=CRANIUM_DARK, rim_col=CRANIUM_RIM, off=0.0, step=3.4)
    for bu, bv in ((-hw0 + 5.6, hs - 5.2), (hw0 - 5.6, hs - 5.2), (-hw1 + 5.6, -hs + 5.2), (hw1 - 5.6, -hs + 5.2)):
        pt, nrm = eval_sc(bu, bv)
        tris += socket_bolt(fr.canon_to_head(pt + (off_mm + 0.06) * nrm), nrm, r_out=0.0022 * 0.88, h_out=0.00048 * 0.88)
    return tris


def cranium_dome_panels():
    """Return `{panel_name: tris}` for all 9 (`3x3`) crown armor plates."""
    off, out = cfg()['crown_off'] + 2.05, {}
    for tag, rh, rl in (('front', 1, 0), ('mid', 3, 2), ('rear', 5, 4)):
        c1 = 0.955 if tag != 'front' else 0.892
        out[f'dome_sagittal_{tag}'] = _sc_band_plate(rh, rl, -0.228, 0.228, off)
        out[f'dome_parietal_{tag}_L'] = _sc_band_plate(rh, rl, 0.282, c1, off)
        out[f'dome_parietal_{tag}_R'] = _sc_band_plate(rh, rl, -c1, -0.282, off)
    return out


def cranium_dome_plates():
    """Return combined triangles of all 12 crown armor tiles."""
    return [t for ts in cranium_dome_panels().values() for t in ts]


def eval_temple_abs_x(y_mm, z_mm):
    """Evaluate convex temple honeycomb surface `|X|(Y, Z)`."""
    p, fy, fz = cfg(), float(y_mm), float(z_mm)
    x_sil, x_tin, x_tout, z_ant = fhull.ray_front_x_bounds(fy, fz)
    cy = 8.8 * (max(0.0, (fy - 166.0) / 26.0) ** 1.75) + 3.6 * (max(0.0, (166.0 - fy) / 22.0) ** 1.75)
    cz = 6.4 * (max(0.0, (-17.5 - fz) / 35.0) ** 1.75) + 8.8 * (max(0.0, (fz + 17.5) / 36.0) ** 1.65)
    dome = p['temp_xa'] + 1.15 + 0.35 * fhull.panel_offset('temple-hex-bezels', y_mm, z_mm) - cy - cz
    outer_t = min(68.00, x_sil + 0.40, x_tout + 0.40) - 0.85 * cy
    x_in = min(dome - 0.10, max(52.4, x_tin + 0.85 + (p['temp_xp'] - 67.85)))
    t_f = max(0.0, min(1.0, (fz - (z_ant - 8.5)) / 8.5))
    return max(52.4, min(outer_t, dome - (dome - x_in) * (t_f ** 1.25)))


def temporal_amber_vents():
    """Return `(left, right)` rotated and scaled temple honeycomb modules."""
    raw = fcon.active_loop('PROF_TEMPLE_LOOP')
    bolts = _xform_temple([(*b, 0.96) for b in ((422, 226), (562, 182), (692, 242), (478, 302), (616, 334))], raw)
    lp = fr.simplify_closed_rdp(_xform_temple(raw, raw), 2.4)
    tv = fr.profile_ray_honeycomb(lp, 1, eval_temple_abs_x, frame_w_px=10.6,
                                  pitch_px=14.0, fill=0.80, bezel_mm=1.48, chamfer_mm=0.50, bolts_px=bolts)
    return tv, mirror_x(tv)


def temporal_amber_vent():
    """Return `+X` temple vent."""
    return temporal_amber_vents()[0]


def _flat_plate(ctr_mm, nrm_v, pts_uv_mm, bolts_uv_mm, bsc=0.80):
    np = np_mod()
    c0, n = np.asarray(ctr_mm, float), np.asarray(nrm_v, float)
    n /= max(float(np.linalg.norm(n)), 1e-12)
    e1 = np.cross(np.array([0.0, 1.0, 0.0]), n)
    e1 /= max(float(np.linalg.norm(e1)), 1e-12)
    e2 = np.cross(n, e1)
    surf_m = lambda u, v, w: fr.canon_to_head(c0 + float(u) * e1 + float(v) * e2 + (float(w) / fr.SCALE_M) * n)
    tris = curved_poly_plate(pts_uv_mm, surf_m, wall=4.65 * fr.SCALE_M, chamfer=0.88 * fr.SCALE_M,
                             corner_r=1.65, col=CRANIUM_DARK, rim_col=CRANIUM_RIM, off=0.0, step=6.5)
    for bu, bv in bolts_uv_mm:
        tris += socket_bolt(surf_m(bu, bv, 0.06 * fr.SCALE_M), n, r_out=0.0022 * bsc, h_out=0.00044 * bsc)
    return tris


def mastoid_panels_left():
    """Return `{name: tris}` for conformal lower-occipital `+X` mastoid armor plate."""
    pt, nrm = sample_head_surface_mm(2.26, -26.0)
    return {'mastoid_plate': _flat_plate(tuple(pt - 2.6 * nrm), tuple(nrm),
                                         [(-9.4, 6.2), (9.2, 6.0), (8.6, -6.2), (-7.4, -6.2)], ((-0.6, 0.3),), 0.76)}


def mastoid_armor_plates():
    """Return `+X` mastoid triangles."""
    return mastoid_panels_left()['mastoid_plate']


def _box_mm(xr, yr, zr, col, ch=0.6):
    return fr.canon_tris_to_head(csg.extrude(csg.box2d(xr, yr), a=zr[0], b=zr[1], axis=2, colour=col, chamfer=ch).tris())


def occipital_panels():
    """Return `{panel_name: tris}` for occipital plates, nape yoke and X-truss."""
    p, off = cfg(), cfg()['crown_off'] + 2.05
    ru_l, ru_r = _sc_band_plate(7, 6, 0.296, 0.875, off), _sc_band_plate(7, 6, -0.875, -0.296, off)
    rl_l, rl_r = _sc_band_plate(9, 8, 0.296, 0.855, off), _sc_band_plate(9, 8, -0.855, -0.296, off)
    yoke = _flat_plate((0.0, fr.HEAD_CENTER_Y - 24.0, -74.5 * p['zb_scale']), (0.0, -0.36, -0.933),
                       [(-27.8, 8.0), (27.8, 8.0), (25.2, -8.2), (15.2, -8.2),
                        (12.4, -0.8), (-12.4, -0.8), (-15.2, -8.2), (-25.2, -8.2)], ((-21.0, 1.8), (21.0, 1.8)))
    yc, zc, np = fr.HEAD_CENTER_Y + 16.5, -94.0 * p['zb_scale'], np_mod()
    xb = _box_mm((-19.5, 19.5), (yc - 24.5, yc + 24.5), (zc + 17.5, zc + 22.5), FRAME_VOID, ch=0.4)
    for sgn in (1.0, -1.0):
        bar = csg.extrude(csg.filleted2d(csg.box2d((-23.0, 23.0), (-4.1, 4.1)), r=0.8), a=-2.7, b=2.7, axis=2, colour=CRANIUM_DARK, chamfer=0.5).tris()
        c45, s45 = math.cos(sgn * 0.25 * math.pi), math.sin(sgn * 0.25 * math.pi)
        xb += fr.canon_tris_to_head([(t @ np.array([[c45, s45, 0.0], [-s45, c45, 0.0], [0.0, 0.0, 1.0]]) + (0.0, yc, zc), c) for t, c in bar])
    xb += fr.canon_tris_to_head(csg.extrude(csg.circle2d(6.8, fn=20, at=(0.0, yc)), a=zc - 2.9, b=zc + 2.9, axis=2, colour=CRANIUM_DARK, chamfer=0.5).tris()
                                + csg.extrude(csg.circle2d(2.6, fn=16, at=(0.0, yc)), a=zc - 3.2, b=zc + 3.2, axis=2, colour=FRAME_VOID, chamfer=0.3).tris())
    for sgn in (1.0, -1.0):
        xb += (_box_mm((-19.8, 19.8), sorted((yc + sgn * 16.9, yc + sgn * 22.1)), (zc - 1.8, zc + 3.4), CRANIUM_DARK)
               + _box_mm(sorted((sgn * 16.2, sgn * 20.2)), (yc - 20.5, yc + 20.5), (zc - 1.2, zc + 4.0), FRAME_VOID)
               + _box_mm((-19.2, 19.2), sorted((yc + sgn * 9.3, yc + sgn * 13.1)), (zc + 1.2, zc + 5.2), FRAME_VOID)
               + socket_bolt(fr.canon_to_head((sgn * 16.8, yc + 17.6, zc - 3.2)), (0.0, 0.0, -1.0), r_out=0.0020, h_out=0.00042)
               + socket_bolt(fr.canon_to_head((sgn * 16.8, yc - 17.6, zc - 3.2)), (0.0, 0.0, -1.0), r_out=0.0020, h_out=0.00042)
               + socket_bolt(fr.canon_to_head((sgn * 14.0, yc + 25.0, zc + 3.0)), (0.0, 0.0, -1.0), r_out=0.0018, h_out=0.00037))
    xb += fr.canon_tris_to_head(csg.sweep(csg.circle2d(2.1, fn=12), [(15.5 * math.cos(a), yc + 8.8 + 15.5 * math.sin(a), zc + 9.2)
                                                                     for a in np.linspace(0.16 * math.pi, 0.84 * math.pi, 9)], chamfer_ends=0.5, colour=AMBER_DARK).tris())
    return {'occipital_upper_L': ru_l, 'occipital_upper_R': ru_r, 'occipital_lower_L': rl_l,
            'occipital_lower_R': rl_r, 'occipital_nape_yoke': yoke, 'occipital_xbrace_module': xb}


def occipital_and_xbrace():
    """Return combined triangles of rear occipital armor, nape yoke and X-truss."""
    return [t for ts in occipital_panels().values() for t in ts]


_NH = (0.0, 0.065, 0.012)


def _canon_to_neck(tris_mm):
    hr = np_mod().array(_NH, float)
    return [(t + hr, c) for t, c in fr.canon_tris_to_head(tris_mm)]


def _spine_segment_neck(idx):
    np, p, is_bot = np_mod(), cfg(), (idx == 0)
    hw_t, hw_b, tail_y, bh = (23.0, 24.0, -18.5, 8.0) if is_bot else (21.6, 22.4, -13.6, 6.2)
    top_y = 9.5 if idx == 3 else 11.8
    core_p = [(-hw_t, top_y), (-6.6, top_y), (-4.8, 3.6), (4.8, 3.6), (6.6, top_y), (hw_t, top_y), (hw_b, -9.8), (7.8, tail_y), (-7.8, tail_y), (-hw_b, -9.8)]
    core = csg.extrude(csg.filleted2d(core_p, r=1.1, n=3), a=-22.0, b=6.0, axis=2, colour=FRAME_VOID, chamfer=0.55).tris()
    wing_l = csg.extrude(csg.filleted2d([(6.2, top_y - 0.4), (hw_t + 0.6, top_y - 0.4), (hw_b + 0.6, -9.6), (8.0, -16.8 if is_bot else -13.0), (5.0, -2.8)], r=1.1, n=3),
                         a=2.4, b=7.8, axis=2, colour=CRANIUM_DARK, chamfer=0.55).tris()
    knuck = (csg.extrude(csg.box2d((-5.8, 5.8), (-2.0 * bh, 0.0)), a=1.0, b=7.2, axis=2, colour=CRANIUM_DARK, chamfer=0.45).tris()
             + csg.extrude(csg.circle2d(3.4, fn=14, at=(top_y - 0.8, 4.0)), a=-6.0, b=6.0, axis=0, colour=BOLT_RIM, chamfer=0.35).tris()
             + csg.extrude(csg.circle2d(4.5, fn=14, at=(top_y - 0.8, 4.0)), a=-3.0, b=3.0, axis=0, colour=CRANIUM_DARK, chamfer=0.40).tris())
    ct, st = math.cos((0.26, 0.18, 0.09, 0.03)[idx]), math.sin((0.26, 0.18, 0.09, 0.03)[idx])
    rm, pos = np.array([[-1.0, 0.0, 0.0], [0.0, ct, st], [0.0, st, -ct]], float), np.array([0.0, (-3.0, 24.5, 51.5, 77.5)[idx], (-89.2, -81.4, -76.5, -73.0)[idx] + p['spine_dz']], float)
    xform = lambda ts: [((np.asarray(t, float) @ rm.T) + pos, c) for t, c in ts]
    wl_m = _canon_to_neck(xform(wing_l)) + [t for bx, by in ((16.0, 5.8), (16.4, -6.4))
                                            for t in socket_bolt(fr.canon_to_head((np.array([-bx, by, 8.8], float) @ rm.T) + pos) + _NH, (0.0, st, -ct), r_out=0.0020, h_out=0.00045)]
    return wl_m, mirror_x(wl_m), _canon_to_neck(xform(core + knuck))


def _blade_module_left(row):
    np, cyl_r, ct, st, cz, sz = np_mod(), cfg()['blade_r'] + 7.2, math.cos(-0.56), math.sin(-0.56), math.cos(0.11), math.sin(0.11)
    rz_m, (piv_y, piv_z) = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]]), ((-9.5, -1.5) if row == 1 else (-35.5, 2.0))
    uv_poly = [(-(s * ct - y * st), s * st + y * ct) for s, y in ((-27.0, 9.8), (26.4, 9.2), (27.8, -9.2), (-25.4, -9.8))]
    surf_m = lambda u, v, w: fr.canon_to_head((np.array([(cyl_r + 1.2 + w / fr.SCALE_M) * math.cos(u / cyl_r) - 8.2, v + piv_y,
                                                         -(cyl_r + 1.2 + w / fr.SCALE_M) * math.sin(u / cyl_r) + piv_z]) @ rz_m.T) + (0.0, 84.5, -35.0)) + _NH
    bay = curved_honeycomb_bay(uv_poly, surf_m, rim_w=3.5, wall=2.0 * fr.SCALE_M, chamfer=0.45 * fr.SCALE_M, pitch_u=3.4, pitch_v=3.4, fill=0.78,
                               corner_r=1.65, frame_col=CRANIUM_DARK, rim_col=CRANIUM_RIM, hex_col=HEX_DARK, hole_col=BAFFLE_DARK, back_col=FRAME_VOID, off=0.0, step=4.8, add_back=True)
    for s_b, y_b in ((-22.8, 6.8), (0.0, 7.2), (22.8, -6.8)):
        u_b, v_b = -(s_b * ct - y_b * st), s_b * st + y_b * ct
        bay += socket_bolt(surf_m(u_b, v_b, 0.0001), np.array([math.cos(u_b / cyl_r), 0.0, -math.sin(u_b / cyl_r)]) @ rz_m.T, r_out=0.0018, h_out=0.00040)
    return bay


def neck_hull_panels():
    """Return `{panel_name: tris}` for neck sleeve, clavicles, trapezius and conduits."""
    slv_l, slv_r, clav_l, clav_r, trap_l, trap_r = fhull.build_neck_and_bust_shells(cfg()['bust_rx'])
    strut_l = (_canon_to_neck(csg.sweep(csg.circle2d(3.0, fn=12), [(12.5, 72.0, -40.5), (14.0, 84.5, -38.5)], chamfer_ends=0.4, colour=CRANIUM_DARK).tris()
                              + csg.extrude(csg.circle2d(4.4, fn=16, at=(84.5, -25.5)), a=19.5, b=24.2, axis=0, colour=CRANIUM_DARK, chamfer=0.45).tris())
               + socket_bolt(fr.canon_to_head((24.3, 84.5, -25.5)) + _NH, (1.0, 0.0, 0.0), r_out=0.0018, h_out=0.00038))
    out = {'neck_sleeve_L': slv_l + strut_l, 'neck_sleeve_R': slv_r + mirror_x(strut_l),
           'neck_clavicle_L': clav_l, 'neck_clavicle_R': clav_r, 'neck_trapezius_L': trap_l,
           'neck_trapezius_R': trap_r, **throat_bellows_hoses(_canon_to_neck)}
    for tag, yh in (('upper', 40.5), ('lower', 18.5)):
        zf, x1 = -0.6 + 2.9 * (((yh - 26.0) / 30.0) ** 2), 8.4 + 1.3 * ((yh + 8.0) / 64.0)
        yk = csg.oriented([(0.0, zf + 4.25), (4.1, zf + 3.95), (5.6, zf + 2.15), (x1, zf + 3.05),
                           (x1 + 3.2, zf + 1.65), (14.6, zf - 0.15), (14.6, zf - 4.40), (0.0, zf - 4.40)], ccw=True, clean=True)
        yk_l = (_canon_to_neck(csg.extrude(yk, a=yh - 2.5, b=yh + 2.5, axis=1, colour=CRANIUM_DARK, chamfer=0.50).tris())
                + socket_bolt(fr.canon_to_head((12.7, yh, zf + 0.35)) + _NH, (0.30, 0.0, 0.95), r_out=0.0019, h_out=0.00042))
        kc = csg.extrude(csg.filleted2d(csg.oriented([(-3.0, yh - 2.0), (3.0, yh - 2.0), (3.5, yh + 0.9), (0.0, yh + 2.4), (-3.5, yh + 0.9)], ccw=True, clean=True), r=0.45),
                         a=zf + 3.45, b=zf + 4.65, axis=2, colour=AMBER_GOLD, chamfer=0.32).tris()
        kt = _canon_to_neck(csg.extrude(csg.filleted2d(csg.box2d((x1 - 2.0, x1 + 2.0), (yh - 1.9, yh + 1.9)), r=0.45), a=zf + 2.35, b=zf + 3.55, axis=2, colour=AMBER_GOLD, chamfer=0.30).tris())
        out[f'neck_hose_holder_{tag}'], out[f'neck_hose_amber_clip_{tag}'] = yk_l + mirror_x(yk_l), _canon_to_neck(kc) + kt + mirror_x(kt)
    return out


def neck_armor_panels():
    """Return `{panel_name: tris}` in `neck` frame for spine, blades, throat hoses and bust."""
    out = neck_hull_panels()
    for idx in range(4):
        out[f'neck_vertebra_{idx + 1}_L'], out[f'neck_vertebra_{idx + 1}_R'], out[f'neck_vertebra_{idx + 1}_clevis'] = _spine_segment_neck(idx)
    for row in (0, 1):
        bl = _blade_module_left(row)
        out[f'neck_shroud_{row + 1}_L'], out[f'neck_shroud_{row + 1}_R'] = bl, mirror_x(bl)
    return out


def neck_armor_local(nod=0.0, head_yaw=0.0):
    """Build articulated cervical spine, side blades, throat hoses and bust."""
    np, ck = np_mod(), cache_key()
    if not ck[4] and ck not in _NECK_CACHE:
        _NECK_CACHE.clear()
        _NECK_CACHE[ck] = [t for ts in neck_armor_panels().values() for t in ts]
    tris = [t for ts in neck_armor_panels().values() for t in ts] if ck[4] else _NECK_CACHE[ck]
    if not nod and not head_yaw:
        return tris
    out = []
    for t, col in tris:
        c = np.asarray(t, float).copy()
        yn = c[:, 1].copy()
        if head_yaw:
            wy = np.clip((yn - 0.016) / 0.032, 0.0, 1.0)
            a, dx, dz = np.radians(head_yaw * wy * wy * (3.0 - 2.0 * wy)), c[:, 0].copy(), c[:, 2] - 0.012
            c[:, 0], c[:, 2] = np.cos(a) * dx + np.sin(a) * dz, 0.012 - np.sin(a) * dx + np.cos(a) * dz
        if nod:
            wn = np.clip((yn + 0.036) / 0.061, 0.0, 1.0)
            a, cy, cz = np.radians(nod * wn * wn * (3.0 - 2.0 * wn)), c[:, 1].copy(), c[:, 2].copy()
            c[:, 1], c[:, 2] = np.cos(a) * cy - np.sin(a) * cz, np.sin(a) * cy + np.cos(a) * cz
        out.append((c, col))
    return out


@functools.lru_cache(maxsize=None)
def compute_heatsink():
    """Build recessed cranial heatsink fins and heatpipes."""
    f0 = csg.extrude(csg.filleted2d(csg.box2d((-0.016, 0.016), (-0.003, 0.003)), r=0.0005), a=-0.0008, b=0.0008, axis=2, colour=BAFFLE_DARK, chamfer=0.0002).tris()
    ts = [t for d in range(24, 170, 24) for s, c in ((math.sin(math.radians(d)), math.cos(math.radians(d))),)
          for t in turn_tris(f0, rot('x', -math.atan2(0.036 * c, 0.034 * s)), (0.0, 0.0, 0.0), (0.0, 0.132 + 0.034 * s, 0.004 + 0.036 * c))]
    hp = csg.sweep(csg.circle2d(0.0017, fn=14), [(0.014, 0.124, 0.038), (0.024, 0.146, 0.026), (0.026, 0.154, -0.004), (0.022, 0.136, -0.028)], chamfer_ends=0.0004, colour=TITANIUM).tris()
    return ts + hp + mirror_x(hp)


_ = (standard_torx_screw, place_screw)
