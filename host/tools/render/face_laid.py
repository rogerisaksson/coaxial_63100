"""Full-resolution facial armor, continuous slotted tiara, sculpted nose, optics and smooth OLED."""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg, face_contours as fcon, face_cranium as fc, face_hull as fhull, face_ray as fr, knee_loads as kl, neck_laid as nl  # noqa: E402
from tools.render.face_honeycomb import (
    AMBER_DARK, AMBER_GOLD, BAFFLE_DARK, BOLT_RIM, CHASSIS_CF, CRANIUM_DARK, CRANIUM_RIM,
    FRAME_VOID, HEX_DARK, MOUTH_SCREEN, OPTIC_DARK,
    PANEL_RIM, SHELL_CF, TITANIUM, curved_honeycomb_bay, mirror_x, np_mod, rev, shell_patch, socket_bolt,
)

_LINER_CACHE = {}
_STATIC_CACHE = {}
_EAR_CACHE = {}


@functools.lru_cache(maxsize=None)
def anchors():
    from machine.body import figure
    seg, lay = {s[0]: s for s in figure.SEGMENTS}, nl.laid()
    nt, hr = tuple(float(v) for v in seg['neck'][3]), tuple(float(v) for v in seg['head'][3])
    ht = (nt[0] + hr[0], nt[1] + hr[1], nt[2] + hr[2])
    return nt, hr, ht, float(lay['head']['carrier'][1] - ht[1]), float(lay['head']['gear'])


def eval_cranio_z(ax, fy):
    ax, fy = abs(float(ax)), float(fy)
    w_m = math.exp(-((ax / 13.5) ** 2))
    u = (min(206.0, max(105.0, fy)) - 156.0) / 46.0
    z_lat = 71.0 - 1.6 * u - 15.2 * (u * u)
    z0 = fhull.eval_side_midline_z(min(206.0, max(105.0, fy))) * w_m + z_lat * (1.0 - w_m)
    t_lo = 1.0 / (1.0 + math.exp((fy - 138.0) / 10.5))
    return z0 - (0.56 + 0.21 * t_lo) * ax - (13.6 - 1.5 * t_lo) * ((ax / 50.0) ** 1.78)


_eval_wing_z = eval_cranio_z


def _lacrimal_recess_mm(ax, fy):
    fa = abs(float(ax))
    ux, um = fc._sstep(24.5, 10.5, fa), fc._sstep(9.5, 15.0, fa)
    return ux * (2.4 + 4.6 * math.exp(-(((float(fy) - 141.5) / 13.5) ** 2)) * (0.38 + 0.62 * um))


def _eval_brow_z(x, y):
    return eval_cranio_z(x, y) - 3.0 * fc._sstep(22.0, 10.0, abs(float(x))) * math.exp(-(((float(y) - 146.0) / 13.5) ** 2))


def _eval_icheek_z(x, y):
    ax, fy = abs(float(x)), float(y)
    zm = 82.6 - 1.295 * (ax - 2.52 * fc._sstep(28.2, 31.8, ax)) + 0.02 * (fy - 122.0) - 0.09 * max(0.0, fy - 138.0)
    return zm - _lacrimal_recess_mm(ax, fy)


def _eval_ocheek_z(x, y):
    ax, fy = abs(float(x)), float(y)
    zi = _eval_icheek_z(min(ax, 34.5), min(144.0, fy)) - 1.28 * max(0.0, ax - 34.5)
    wb = max(fc._sstep(32.2, 41.5, ax), fc._sstep(140.0, 148.0, fy))
    return zi * (1.0 - wb) + (eval_cranio_z(ax, fy) - 2.4 * fc._sstep(36.0, 52.0, ax) * fc._sstep(150.0, 134.0, fy)) * wb


def _eval_mouth_z(x, y):
    p, fx, fy = fc.cfg(), float(x), float(y)
    ax, dy = abs(fx), max(0.0, fy - 70.0)
    lip = (1.35 * math.exp(-(((fy - 86.2) / 3.2) ** 2)) + 0.85 * math.exp(-(((fy - 79.6) / 3.4) ** 2))
           - 0.45 * math.exp(-(((fy - 83.2) / 1.6) ** 2))) * math.exp(-(((ax / 13.8) ** 2)))
    z0 = 56.2 + 0.30 * dy + 5.0 * ((max(0.0, fy - 72.0) / 31.0) ** 2.15) + lip + p['oled_z0']
    ax_e = min(35.4, 14.5 + 0.62 * max(0.0, fy - 71.0))
    zs = min(z0 - 1.4, 34.6 - 0.23 * max(0.0, fy - 72.0))
    u = min(0.99, p['oled_sx'] * (math.hypot(fx, 4.5) - 4.5) / max(8.0, ax_e))
    return z0 - (z0 - zs) * (u ** p['oled_sy']) + 0.65 * fhull.panel_offset('face-oled-mouth', fx, fy)


@functools.lru_cache(maxsize=None)
def chassis_static():
    yc, rb = anchors()[3], 1e-3 * float(kl.HUB[1])
    yt = yc + 0.0052
    flng = csg.extrude(csg.circle2d(0.036, fn=40), holes=[csg.circle2d(rb + 0.0055, fn=36)], a=yc + 0.0006, b=yt, axis=1, colour=CHASSIS_CF)
    pks = [csg.filleted2d(csg.box2d(u, v), r=0.002) for u, v in (((-0.010, 0.010), (0.082, 0.090)), ((-0.034, -0.016), (0.086, 0.112)), ((0.016, 0.034), (0.086, 0.112)))]
    blk = csg.extrude(csg.filleted2d([(-0.035, yt), (0.035, yt), (0.036, 0.136), (-0.036, 0.136)], r=0.003), holes=pks, a=0.022, b=0.030, axis=2, colour=CHASSIS_CF)
    fr_ = csg.extrude(csg.filleted2d(csg.box2d((0.030, 0.048), (-0.034, 0.034)), r=0.0028), a=0.131, b=0.136, axis=1, colour=CHASSIS_CF, chamfer=0.0005)
    dck = csg.extrude(csg.filleted2d(csg.box2d((0.036, 0.048), (-0.032, 0.032)), r=0.0026), a=0.076, b=0.081, axis=1, colour=CHASSIS_CF, chamfer=0.0005)
    ear = csg.sweep(csg.circle2d(0.0032, fn=16), [(0.034, 0.088, 0.018), (0.052, 0.088, 0.002), (0.064, 0.088, 0.004)], chamfer_ends=0.0005, colour=CHASSIS_CF).tris()
    return csg.union(flng, blk, fr_, dck).tris() + ear + mirror_x(ear)


def eye_socket_cups():
    """Return `(cup_l, cup_r)` recessed almond eye cups and medial septum."""
    fc_ = fr.CANONICAL_CAMERAS['front']
    sp = fr.canon_tris_to_head(csg.extrude(csg.box2d((132.0, 152.0), (19.0, 44.0)), a=13.5, b=14.5, axis=0, colour=BAFFLE_DARK, chamfer=0.2).tris())
    mk = lambda lp: fr.camera_ray_planar_panel(fcon.active_loop(lp), fc_, lambda _x, _y: 26.5, bevel_outset_px=0.0, bevel_depth_mm=0.3, thickness_mm=1.4, col=BAFFLE_DARK, rim_col=FRAME_VOID, corner_r_px=0.0)
    return mk('EYE_CUP_L_LOOP') + sp, mk('EYE_CUP_R_LOOP') + mirror_x(sp)


def _clamp_canon_behind_face(p):
    cp, wy, fx, ax, pz = fc.cfg(), float(p[1]), float(p[0]), abs(float(p[0])), float(p[2])
    if wy < 218.0 and pz > -4.0:
        ym_seam = min(122.0, 100.0 + 0.80 * max(0.0, min(ax, 42.0) - 12.0))
        po_min = min(0.0, 0.65 * fhull.panel_offset('forehead-wings', fx, wy), 0.65 * fhull.panel_offset('face-brows', fx, wy),
                     0.35 * fhull.panel_offset('face-inner-cheeks', fx, wy), fhull.panel_offset('face-outer-cheeks', fx, wy),
                     0.65 * fhull.panel_offset('face-oled-mouth', fx, wy))
        if 164.0 <= wy <= 216.0:
            zl = min(_eval_wing_z(fx, wy), _eval_brow_z(ax, wy)) + po_min - 6.80
        else:
            zl = ((min(_eval_wing_z(fx, wy), _eval_brow_z(ax, wy)) - 4.2) if wy >= 164.0
                  else (min(_eval_brow_z(ax, wy), _eval_ocheek_z(ax, wy)) if wy >= 146.0
                        else ((_eval_icheek_z(ax, wy) if ax < 35.0 else _eval_ocheek_z(ax, wy))
                              if (wy >= ym_seam or ax > 35.5) else _eval_mouth_z(math.copysign(min(ax, 34.5), fx), wy) - 3.0))) + po_min - 6.8
        if 11.5 <= ax <= 45.0 and 127.5 <= wy <= 156.5:
            zl = min(zl, 9.8)
        elif 18.5 <= ax <= 31.5 and 156.5 < wy <= 161.0:
            zl = min(zl, 16.5)
        if ax <= 15.0 and 104.0 <= wy <= 170.0:
            zl = min(zl, fcon.eval_nose_z_mm(ax, wy, fhull.eval_side_midline_z(wy), cp['nose_dz'], cp.get('nose_tip', 0.0))
                     + min(0.0, 0.65 * fhull.panel_offset('face-nose', fx, wy)) - (15.5 if wy >= 122.0 else 8.5))
        p[2] = pz = min(pz, zl)
    elif ax <= 24.5 and 126.0 <= wy <= 184.0 and pz < -60.0:
        p[2] = pz = max(pz, -79.8)
    elif ax <= 24.0 and 114.0 <= wy <= 128.0 and pz < -60.0:
        p[2] = pz = max(pz, -75.5 * cp['zb_scale'])
    x_sil_lim, x_ti, _xo, _za = fhull.ray_front_x_bounds(wy, pz)
    if 112.0 <= wy <= 216.0 and -26.0 < pz < 68.0 and not fc.inside_prof_temple(wy, pz):
        x_sil_lim -= 5.8
        if ax > x_sil_lim:
            p[0] = math.copysign(max(8.0, x_sil_lim), float(p[0]))
            ax = abs(float(p[0]))
    if fc.inside_prof_temple(wy, pz):
        mx = min(x_sil_lim, x_ti - 2.2, fc.eval_temple_abs_x(wy, min(pz, 18.0)) - 5.2)
        if ax > mx:
            p[0] = math.copysign(max(8.0, mx), float(p[0]))
    elif wy <= 132.0 and -24.0 < pz < 54.0:
        jl = min(cp['jaw_r_x0'] - 2.35 + cp['jaw_r_sy'] * wy - cp['jaw_r_sz'] * pz + 0.6 * fhull.panel_offset('jaw-rear-hex', wy, pz),
                 fcon.eval_jaw_front_x(wy, pz),
                 fhull.front_jaw_max_x(wy, pz)) - 5.8
        if ax > jl:
            p[0] = math.copysign(max(8.0, jl), float(p[0]))
    hm = fr.canon_to_head(p)
    if hm[1] < 0.078:
        rm, req = math.hypot(float(hm[0]), float(hm[2])), (0.0472 if hm[1] < 0.055 else 0.0378)
        if 1e-6 < rm < req:
            hm[0], hm[2] = hm[0] * (req / rm), hm[2] * (req / rm)
    return hm


def _clamped_liner_pt(phi, yl, off_mm=0.0):
    p, (pt, nrm) = fc.cfg(), fc.sample_head_surface_mm(phi, yl)
    ep, ey = (abs(float(phi)) - p['ear_phi']) / 0.46, (float(yl) - p['ear_y']) / 25.5
    recess = 7.5 * math.exp(-(ep * ep + ey * ey))
    return _clamp_canon_behind_face(pt + (float(off_mm) - recess) * nrm)


def eye_lashes():
    """Return `(lash_l, lash_r)` swept cat-eye lash ribbons."""
    rim = lambda x, y: ((_eval_icheek_z(x, y) + 0.35 * fhull.panel_offset('face-inner-cheeks', float(x), float(y)))
                        if abs(float(x)) < 34.0 else (_eval_ocheek_z(x, y) + fhull.panel_offset('face-outer-cheeks', float(x), float(y)))) - 0.18
    lr = (fcon.build_lash_ribbon(rim, ((374.0, 529.0), (355.0, 496.0), (316.0, 477.5), (276.0, 483.0), (258.0, 502.5)),
                                 ((369.0, 531.0), (350.0, 503.0), (315.0, 484.5), (278.0, 489.0), (262.0, 504.0)), -0.10, -0.68)
          + fcon.build_lash_ribbon(rim, ((372.0, 535.0), (327.0, 546.0), (279.0, 532.0), (259.0, 505.5)),
                                   ((368.0, 533.0), (326.0, 542.0), (280.0, 528.0), (262.0, 504.5)), -0.10, -0.64))
    return mirror_x(lr), lr


def cranial_liner_shells():
    """Return `(left, right)` bilateral cranial liners."""
    np = np_mod()
    specs = ((0.0, 1.95, 24.0, 66.0, 12, 12), (0.68, 2.28, -55.5, -16.0, 14, 11), (2.28, math.pi, -38.5, -16.0, 10, 7),
             (0.68, 1.95, -16.0, 24.0, 10, 10), (0.0, 0.18, -50.0, 12.0, 9, 4),
             (0.18, 0.68, -54.0, -22.0, 9, 7), (0.40, 0.68, -22.0, 6.0, 7, 5), (0.43, 0.69, 6.0, 24.5, 6, 5))

    def _side(sgn):
        hl = []
        for p0, p1, y0, y1, ny, ns in specs:
            phs = np.linspace(p0, p1, ns) if sgn > 0 else np.linspace(-p1, -p0, ns)
            g = np.array([[_clamped_liner_pt(float(ph), float(yl), -4.75) for ph in phs] for yl in np.linspace(y0, y1, ny)])
            hl += shell_patch(g, 0.0017, 0.0003, FRAME_VOID, (0.0, 0.105, 0.015), CHASSIS_CF)
        crs = np.linspace(0.0, 0.95, 12) if sgn > 0 else np.linspace(-0.95, 0.0, 12)
        gc = np.array([[_clamp_canon_behind_face(fc.sample_crown_dome_mm(float(sr), float(cr))[0] - 3.85 * fc.sample_crown_dome_mm(float(sr), float(cr))[1])
                        for cr in crs] for sr in np.linspace(-1.92, 0.84, 22)])
        return hl + shell_patch(gc, 0.0016, 0.0003, FRAME_VOID, (0.0, 0.125, 0.005), CHASSIS_CF)

    rb = fr.canon_tris_to_head(csg.extrude(csg.box2d((168.0, 190.0), (-75.5, -70.5)), a=0.05, b=4.5, axis=0, colour=FRAME_VOID, chamfer=0.2).tris())
    ys_m = (64.0, 67.2, 70.0, 72.0, 99.5, 101.5, 108.2, 110.5, 120.0, 132.0, 144.0, 160.5, 163.5, 168.5)
    dz_m = (-3.6, -0.6, -0.6, -3.6, -5.5, -6.5, -9.8, -12.0, -17.5, -18.0, -18.0, -17.5, -16.5, -16.5)
    mp = [(y, 33.4) for y in ys_m] + [(y, fhull.eval_side_midline_z(y) + d) for y, d in reversed(list(zip(ys_m, dz_m)))]
    mb = fr.canon_tris_to_head(csg.extrude(csg.oriented(mp, ccw=True, clean=True), a=0.10, b=0.80, axis=0, colour=FRAME_VOID, chamfer=0.15).tris())
    return _side(1.0) + rb + mb, _side(-1.0) + mirror_x(rb + mb)


def inner_liner_panels():
    ll, lr = cranial_liner_shells()
    el, er = eye_lashes()
    return {'cranial_liner_L': ll + el, 'cranial_liner_R': lr + er}


def inner_liner():
    ck = fc.cache_key()
    all_t = lambda: [t for ts in inner_liner_panels().values() for t in ts]
    if ck[4]:
        return all_t()
    if ck not in _LINER_CACHE:
        _LINER_CACHE.clear()
        _LINER_CACHE[ck] = all_t()
    return _LINER_CACHE[ck]


compute_heatsink = fc.compute_heatsink


def _front_plate(ln, fn_z, col=SHELL_CF, rim=CRANIUM_RIM, bd=0.68, th=3.6, cr=6.6):
    return fr.camera_ray_planar_panel(fcon.active_loop(ln), fr.CANONICAL_CAMERAS['front'], fn_z, bevel_outset_px=2.8,
                                      bevel_depth_mm=bd, thickness_mm=th, col=col, rim_col=rim, holes_px=fcon.active_holes(ln), corner_r_px=cr)


def frontal_panels():
    eval_d = lambda x, y: eval_cranio_z(x, y) + 0.30 + 0.55 * fhull.panel_offset('forehead-diamond', float(x), float(y))
    eval_w = lambda x, y: eval_cranio_z(x, y) + 0.65 * fhull.panel_offset('forehead-wings', float(x), float(y))
    eval_b = lambda x, y: _eval_brow_z(x, y) + 0.65 * fhull.panel_offset('face-brows', float(x), float(y))
    return {'brow_amber_keystone': _front_plate('DIAMOND_LOOP', eval_d, AMBER_GOLD, AMBER_DARK, 0.72, 3.7),
            'brow_amber_tiara': _front_plate('TIARA_LOOP', eval_w, AMBER_GOLD, AMBER_DARK, 0.68, 3.6),
            'forehead_porcelain_L': _front_plate('BROW_L_LOOP', eval_b, SHELL_CF, CRANIUM_RIM, 0.70, 3.7),
            'forehead_porcelain_R': _front_plate('BROW_R_LOOP', eval_b, SHELL_CF, CRANIUM_RIM, 0.70, 3.7)}


def cheek_and_nose_panels():
    p, fcam = fc.cfg(), fr.CANONICAL_CAMERAS['front']
    zlug = fr.canon_tris_to_head(csg.extrude(csg.filleted2d(csg.box2d((148.0, 153.0), (-26.0, -22.0)), r=0.6), a=55.8, b=57.8, axis=0, colour=CHASSIS_CF, chamfer=0.3).tris())
    sx = p.get('nose_wx', 1.000)
    n_lp = [(450.0 + (px - 450.0) * sx, py) for px, py in fcon.active_loop('NOSE_LOOP')]
    n_hl = tuple((450.0 + (h[0] - 450.0) * sx, h[1], h[2] * sx, h[3], h[4], h[5]) for h in fcon.active_holes('NOSE_LOOP'))
    eval_ic = lambda x, y: _eval_icheek_z(x, y) + 0.35 * fhull.panel_offset('face-inner-cheeks', float(x), float(y))
    eval_oc = lambda x, y: _eval_ocheek_z(x, y) + fhull.panel_offset('face-outer-cheeks', float(x), float(y))

    def eval_nz(x, y):
        fx, ax, fy = float(x), abs(float(x)), float(y)
        xw = 7.6 + 1.3 * fc._sstep(123.0, 114.0, fy) - 1.5 * fc._sstep(140.0, 149.0, fy)
        wn = fc._sstep(xw - 3.8, xw - 0.15, ax)
        zn = fcon.eval_nose_z_mm(fx / sx, fy, fhull.eval_side_midline_z(min(153.5, fy)), p['nose_dz'], p.get('nose_tip', 0.0))
        po_n = 0.65 * fhull.panel_offset('face-nose', fx, fy)
        wt_lo = fc._sstep(115.5, 112.0, fy)
        return (1.0 - wn) * (zn + po_n) + wn * (eval_ic(math.copysign(max(ax, xw) + (2.1 if fx < 0.0 else 1.5), fx), fy) + 0.10 + wt_lo * po_n)

    nose = fr.camera_ray_planar_panel(n_lp, fcam, eval_nz, bevel_outset_px=2.6, bevel_depth_mm=0.66, thickness_mm=4.0, col=SHELL_CF, rim_col=CRANIUM_RIM, holes_px=n_hl, corner_r_px=7.2, inward_x=0.68)

    def gus_surf(u_mm, v_mm, w_m):
        pt, nrm = fc.sample_head_surface_mm(float(u_mm) / fr.SURFACE_ARC_R, float(v_mm))
        dw = float(w_m) / fr.SCALE_M
        pg = pt + (-2.4 + dw) * nrm
        ax_g, wy_g = abs(float(pg[0])), float(pg[1])
        pg[2] = min(float(pg[2]), min(_eval_ocheek_z(ax_g, wy_g), eval_oc(-ax_g, wy_g), eval_oc(ax_g, wy_g)) - 6.6 + dw, (15.5 + dw) if (ax_g <= 46.0 and 127.0 <= wy_g <= 164.0) else 99.0)
        xs_g, xt_g, *_ = fhull.ray_front_x_bounds(wy_g, float(pg[2]))
        mx_g = min(51.0, xs_g - 2.6, xt_g - 2.6, (fc.eval_temple_abs_x(wy_g, float(pg[2])) - 3.8) if fc.inside_prof_temple(wy_g, float(pg[2])) else 99.0) + dw
        if ax_g > mx_g:
            pg[0] = math.copysign(max(8.0, mx_g), float(pg[0]))
        return fr.canon_to_head(pg)

    msep = fr.canon_tris_to_head(csg.extrude(csg.oriented([(128.5, 43.0), (151.5, 43.0), (151.5, 49.2), (144.0, 49.6), (128.5, 49.6)], ccw=True, clean=True), a=10.9, b=13.4, axis=0, colour=BAFFLE_DARK, chamfer=0.2).tris())
    opil = curved_honeycomb_bay([(62.0, 9.5), (84.0, 5.5), (89.5, -5.5), (86.0, -19.0), (64.0, -17.5)], gus_surf, rim_w=1.7, wall=1.6 * fr.SCALE_M,
                                chamfer=0.35 * fr.SCALE_M, pitch_u=3.1, pitch_v=3.1, fill=0.78, corner_r=1.5, frame_col=CRANIUM_DARK,
                                rim_col=FRAME_VOID, hex_col=HEX_DARK, hole_col=BAFFLE_DARK, back_col=FRAME_VOID, off=0.0, step=4.8, add_back=True) + msep
    return {'cheek_inner_L': _front_plate('ICHEEK_L_LOOP', eval_ic, bd=0.72, th=3.5, cr=11.0),
            'cheek_inner_R': _front_plate('ICHEEK_R_LOOP', eval_ic, bd=0.72, th=3.5, cr=11.0),
            'cheek_outer_L': _front_plate('OCHEEK_L_LOOP', eval_oc, bd=0.72, th=3.6, cr=9.2) + zlug,
            'cheek_outer_R': _front_plate('OCHEEK_R_LOOP', eval_oc, bd=0.72, th=3.6, cr=9.2) + mirror_x(zlug),
            'orbital_pillar_L': opil, 'orbital_pillar_R': mirror_x(opil), 'nose_porcelain': nose}


def lower_jaw_and_oled_panels():
    """Build smooth OLED, bezel, chin and jaw honeycombs."""
    p, fcam = fc.cfg(), fr.CANONICAL_CAMERAS['front']
    oled = fr.camera_ray_planar_panel(fcon.active_loop('MOUTH_LOOP'), fcam, _eval_mouth_z, bevel_outset_px=2.8, bevel_depth_mm=0.66, thickness_mm=2.8, col=MOUTH_SCREEN, rim_col=CRANIUM_RIM, corner_r_px=6.6)
    bzh = ((18.0, 730.0), (58.0, 736.0), (112.0, 748.0), (104.0, 814.0), (52.0, 838.0))
    bzl = [(451.0 - dx, py) for dx, py in reversed(bzh)] + [(451.0 + dx, py) for dx, py in bzh]
    bezel = fr.camera_ray_planar_panel(bzl, fcam, lambda x, y: _eval_mouth_z(x, y) - 4.8, bevel_outset_px=0.0, bevel_depth_mm=0.30, thickness_mm=1.8, col=BAFFLE_DARK, rim_col=CHASSIS_CF, corner_r_px=0.0)
    subm = fr.canon_tris_to_head(csg.extrude(csg.filleted2d([(65.8, 19.5), (64.8, 26.5), (63.8, 30.2), (66.0, 30.5), (84.0, 26.5), (84.0, 19.5)], r=1.0, n=3), a=-5.2, b=5.2, axis=0, colour=FRAME_VOID, chamfer=0.35).tris())
    eval_c = lambda x, y: fhull.eval_side_midline_z(y) + p['chin_z0'] - p['chin_sx'] * max(0.0, abs(float(x)) - 5.5) + fhull.panel_offset('jaw-chin-cap', float(x), float(y))
    chin = fr.camera_ray_planar_panel(fcon.active_loop('CHIN_LOOP'), fcam, eval_c, bevel_outset_px=2.8, bevel_depth_mm=0.70, thickness_mm=3.5, col=AMBER_GOLD, rim_col=AMBER_DARK, holes_px=fcon.active_holes('CHIN_LOOP'), corner_r_px=6.4)
    jf_lp = fr.simplify_closed_rdp(fcon.active_loop('PROF_JAW_FRONT_LOOP'), 1.8)
    jr_lp = fr.simplify_closed_rdp(fcon.active_loop('PROF_JAW_REAR_LOOP'), 1.8)
    eval_jr = lambda y, z: min(fhull.front_jaw_max_x(y, z) - 0.85, p['jaw_r_x0'] - 2.35 + p['jaw_r_sy'] * float(y) - p['jaw_r_sz'] * float(z) + 0.6 * fhull.panel_offset('jaw-rear-hex', y, z))
    jf_l = fr.profile_ray_honeycomb(jf_lp, 1, fcon.eval_jaw_front_x, frame_w_px=10.8, pitch_px=10.6, fill=0.80, bezel_mm=1.80, chamfer_mm=0.50, bolts_px=((725.0, 544.0, 0.92), (784.0, 660.0, 0.92), (665.0, 616.0, 0.92)))
    jr_l = fr.profile_ray_honeycomb(jr_lp, 1, eval_jr, frame_w_px=11.4, pitch_px=11.0, fill=0.80, bezel_mm=1.90, chamfer_mm=0.50, bolts_px=((630.0, 478.0, 0.92), (680.0, 509.0, 0.92), (630.0, 592.0, 0.92)))
    return {'oled_carrier_bezel': bezel + subm, 'flexible_oled_panel': oled, 'chin_amber_bridge': chin,
            'jaw_amber_rear_L': jr_l, 'jaw_amber_rear_R': mirror_x(jr_l), 'jaw_amber_front_L': jf_l, 'jaw_amber_front_R': mirror_x(jf_l)}


def oled_mouth_pixels(smile=0.0, jaw_open=0.0):
    return fcon.build_oled_mouth_pixels(_eval_mouth_z, smile, jaw_open)


def static_plates():
    """Build static head plates."""
    ck = fc.cache_key()
    if not ck[4] and ck in _STATIC_CACHE:
        return _STATIC_CACHE[ck]
    fp, cp, jp = frontal_panels(), cheek_and_nose_panels(), lower_jaw_and_oled_panels()
    lt, rt = fc.temporal_amber_vents()
    lm = fc.mastoid_armor_plates()
    res = {'cranium_frontal': [t for ts in fp.values() for t in ts], 'cranium_dome': fc.cranium_dome_plates(),
           'cranium_occipital': fc.occipital_and_xbrace(), 'left_temporal': lt, 'right_temporal': rt,
           'left_mastoid': lm, 'right_mastoid': mirror_x(lm), 'left_orbital_pillar': cp['orbital_pillar_L'],
           'right_orbital_pillar': cp['orbital_pillar_R'], 'nasal_bridge': cp['nose_porcelain'],
           'left_maxilla': cp['cheek_outer_L'] + cp['cheek_inner_L'], 'right_maxilla': cp['cheek_outer_R'] + cp['cheek_inner_R'],
           'oled_bezel': jp['oled_carrier_bezel'] + jp['chin_amber_bridge'] + jp['jaw_amber_rear_L'] + jp['jaw_amber_rear_R'] + jp['jaw_amber_front_L'] + jp['jaw_amber_front_R'],
           'oled_screen': jp['flexible_oled_panel']}
    if not ck[4]:
        _STATIC_CACHE.clear()
        _STATIC_CACHE[ck] = res
    return res


def ear_microphones():
    """Build upright ear pods."""
    ck = fc.cache_key()
    if not ck[4] and ck in _EAR_CACHE:
        return _EAR_CACHE[ck]
    np, p = np_mod(), fc.cfg()
    re_m, pt_mm, nrm0 = p['ear_r'] * fr.SCALE_M, *fc.sample_head_surface_mm(p['ear_phi'], p['ear_y'])
    rc = re_m + 0.0012
    nrm = np.array([float(nrm0[0]), -0.045, float(nrm0[2])], float)
    nrm /= max(float(np.linalg.norm(nrm)), 1e-12)
    pos = fr.canon_to_head(pt_mm + (0.95 + p['ear_inset']) * nrm0)
    ht = min(0.0044, max(0.0030, (0.08795 - float(pos[0]) - 0.10 * re_m) / max(0.35, float(nrm[0]))))
    col_t = (rev([(0.0155, 0.0), (rc, 0.0), (rc, 0.42 * ht), (rc - 0.0016, 0.65 * ht), (0.0155, 0.65 * ht)], 2, (0.0, 0.0), 40, FRAME_VOID)
             + rev([(0.0042, 0.0002), (re_m, 0.0002), (re_m, 0.68 * ht), (re_m - 0.0020, ht), (0.0042, ht)], 2, (0.0, 0.0), 40, CRANIUM_DARK)
             + rev([(0.0, 0.0002), (0.0044, 0.0002), (0.0044, ht), (0.0032, ht - 0.0008), (0.0032, ht - 0.0022), (0.0, ht - 0.0022)], 2, (0.0, 0.0), 24, BOLT_RIM))
    e1 = np.cross((0.0, 1.0, 0.0), nrm)
    e1 /= max(float(np.linalg.norm(e1)), 1e-12)
    bas = np.stack([e1, np.cross(nrm, e1), nrm], axis=0)
    left = [(t @ bas + pos, c) for t, c in col_t] + socket_bolt(pos + (ht - 0.0003) * nrm, nrm, r_out=0.0032, h_out=0.00050, fn=18)
    mx = float(np.vstack([t for t, _ in left])[:, 0].max())
    if mx > 0.08805:
        left = [(t * (0.08800 / mx, 1.0, 1.0), c) for t, c in left]
    res = {'left_ear_mic': left, 'right_ear_mic': mirror_x(left)}
    if not ck[4]:
        _EAR_CACHE.clear()
        _EAR_CACHE[ck] = res
    return res


@functools.lru_cache(maxsize=None)
def camera_and_gimbal_local():
    """Build camera module and 2-axis gimbal ring."""
    cam = (rev([(0.0, 0.0024), (0.0050, 0.0024), (0.0058, 0.0010), (0.0058, -0.0020), (0.0046, -0.0052),
                (0.00255, -0.0058), (0.00255, -0.00735), (0.00215, -0.00745), (0.00215, -0.00685), (0.0, -0.00685)], fn=40, col=CHASSIS_CF)
           + rev([(0.0, -0.00680), (0.00214, -0.00680), (0.00214, -0.00738), (0.00135, -0.00762), (0.0, -0.00770)], fn=36, col=OPTIC_DARK)
           + rev([(0.0, -0.00725), (0.00062, -0.00725), (0.00062, -0.00768), (0.0, -0.00776)], at=(0.00095, 0.00075), fn=16, col=(236, 244, 255))
           + csg.extrude(csg.filleted2d(csg.box2d((-0.001, 0.001), (0.005, 0.0067)), r=0.00032), a=0.0006, b=0.0022, axis=2, colour=TITANIUM, chamfer=0.00025).tris())
    ring = rev(csg.filleted2d([(0.0064, -0.001), (0.0074, -0.001), (0.0074, 0.0011), (0.0064, 0.0011)], r=0.0003, n=3), fn=36, col=PANEL_RIM)
    pins = [t for a, b in ((-0.0086, -0.0073), (0.0073, 0.0086)) for t in csg.extrude(csg.circle2d(0.00075, fn=12), a=a, b=b, axis=1, colour=TITANIUM).tris()]
    return cam, ring, pins


def moving_rest():
    """Build retracted brow shutter and concentric eyelids."""
    from machine.eyes import gimbal, stereo
    np, ex = np_mod(), 0.5 * float(stereo.BASELINE_M)
    piv = np.array([ex, float(gimbal.EYES[1]), float(gimbal.EYES[2])]) - gimbal.mounted('left') @ (0.0, 0.0, -float(gimbal.LEVER['left']))
    xb, fb = np.linspace(0.0245, 0.0395, 11)[:, None], np.linspace(0.0, 1.0, 4)[None, :]
    gb = np.stack([np.broadcast_to(xb, (11, 4)), np.broadcast_to(0.1225 + 0.0020 * fb, (11, 4)), np.broadcast_to(0.0640 - 0.34 * (xb - 0.032), (11, 4))], axis=-1)
    lx = np.linspace(float(piv[0]) - 0.0055, float(piv[0]) + 0.0055, 13)[:, None]
    ryz = np.sqrt(np.maximum(30e-6, 0.0108 ** 2 - 0.50 * ((lx - float(piv[0])) ** 2)))
    lg = lambda a0, a1, dz=-0.00155: np.stack([np.broadcast_to(lx, (13, 5)), float(piv[1]) + ryz * np.sin(np.linspace(math.radians(a0), math.radians(a1), 5))[None, :],
                                               float(piv[2]) + dz + ryz * np.cos(np.linspace(math.radians(a0), math.radians(a1), 5))[None, :]], axis=-1)
    return (shell_patch(gb, 0.0012, 0.00022, CRANIUM_DARK, (0.032, 0.098, 0.050), BAFFLE_DARK),
            shell_patch(lg(44.0, 74.0), 0.0009, 0.00018, BAFFLE_DARK, tuple(piv), FRAME_VOID),
            shell_patch(lg(-78.0, -58.0, -0.00255), 0.0009, 0.00018, BAFFLE_DARK, tuple(piv), FRAME_VOID), piv)
