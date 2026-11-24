"""Multi-panel anime-mecha gynoid head from interlocking faceted armor tiles and fine honeycomb."""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg, face_honeycomb as fh, knee_loads as kl, neck_laid as nl  # noqa: E402
from tools.render.face_honeycomb import (
    ANODIZED, BAFFLE_DARK, CHASSIS_CF, HORN_BRASS, MIC_GOLD, PANEL_RIM,
    SERVO_CASE, SHELL_CF, TITANIUM, crown_seam_y as _crown_seam_y,
    ear_clear_y as _ear_clear_y, eye_bot as _eye_bot, eye_top as _eye_top,
    head_pt as _head_pt, mecha_tiles as _mecha_tiles, mirror_x, np_mod,
    planar_grid as _planar_grid, rev, rot, shell_patch, surf_grid as _surf_grid,
)


@functools.lru_cache(maxsize=None)
def anchors():
    """Return `(neck_torso, head_rel, head_torso, y_carrier, r_carrier)` in m."""
    from machine.body import figure
    seg = {s[0]: s for s in figure.SEGMENTS}
    nt, hr = tuple(float(v) for v in seg['neck'][3]), tuple(float(v) for v in seg['head'][3])
    ht, lay = (nt[0] + hr[0], nt[1] + hr[1], nt[2] + hr[2]), nl.laid()
    return nt, hr, ht, float(lay['head']['carrier'][1] - ht[1]), float(lay['head']['gear'])


def rod(p0, p1, r=0.00085, col=TITANIUM):
    """Build chamfered titanium pushrod with brass clevis collars between `p0` and `p1`."""
    np = np_mod()
    a, b = np.asarray(p0, float), np.asarray(p1, float)
    ln = float(np.linalg.norm(b - a))
    if ln < 0.003:
        return []
    u, c2, c3 = (b - a) / ln, csg.circle2d(r, fn=14), csg.circle2d(1.48 * r, fn=14)
    return (csg.sweep(c2, [a + u * s for s in (0.0, 0.0018, 0.5 * ln, ln - 0.0018, ln)],
                      chamfer_ends=0.35 * r, colour=col).tris()
            + csg.sweep(c3, [a, a + 0.0016 * u], chamfer_ends=0.3 * r, colour=HORN_BRASS).tris()
            + csg.sweep(c3, [b - 0.0016 * u, b], chamfer_ends=0.3 * r, colour=HORN_BRASS).tris())


def micro_servo(at, axis, ang, hlen=0.0058, hdir=(0.0, 0.0, 1.0), dual=False, sgn=1.0):
    """Build compact sub-micro model servo body at `at`, spline on `axis`, and brass horn at `ang` rad."""
    np = np_mod()
    cx, cy, cz = (float(v) for v in at)
    hx, hy, hz = (0.0050, 0.0044, 0.0048) if axis == 'x' else (
        (0.0044, 0.0050, 0.0048) if axis == 'y' else (0.0044, 0.0048, 0.0050))
    ai = {'x': 0, 'y': 1, 'z': 2}[axis]
    ur, vr = ((-hy, hy), (-hz, hz)) if ai == 0 else (((-hz, hz), (-hx, hx)) if ai == 1 else ((-hx, hx), (-hy, hy)))
    lo, hi = (cx - hx, cy - hy, cz - hz)[ai], (cx + hx, cy + hy, cz + hz)[ai]
    off = (0.0, cy, cz) if ai == 0 else ((cx, 0.0, cz) if ai == 1 else (cx, cy, 0.0))
    body = csg.extrude(csg.filleted2d(csg.box2d(ur, vr), r=0.0009, n=4), a=lo, b=hi, axis=ai,
                       colour=SERVO_CASE, chamfer=0.0005).translate(*off).tris()
    ears = csg.filleted2d(csg.box2d((ur[0] - 0.0016, ur[1] + 0.0016), (vr[0] * 0.45, vr[1] * 0.45)), r=0.0004)
    mid = 0.5 * (lo + hi)
    body += csg.extrude(ears, a=mid - 0.0008, b=mid + 0.0008, axis=ai, colour=ANODIZED, chamfer=0.00025).translate(*off).tris()
    sp = np.array([cx, cy, cz], float)
    sp[ai] = (hi + 0.0013) if sgn >= 0.0 else (lo - 0.0013)
    hd = np.asarray(hdir, float)
    hd[ai] = 0.0
    hd = rot(axis, ang) @ (hd / max(float(np.linalg.norm(hd)), 1e-12))
    t1, t0 = sp + float(hlen) * hd, sp - (float(hlen) if dual else 0.0014) * hd
    horn = csg.sweep(csg.filleted2d(csg.box2d((-0.0013, 0.0013), (-0.0007, 0.0007)), r=0.00032),
                     [t0, sp, t1], chamfer_ends=0.00025, colour=HORN_BRASS).tris()
    return body, horn, t1, t0


@functools.lru_cache(maxsize=None)
def chassis_static():
    """Build narrow internal carbon chassis, recessed dark orbital socket baffles, oral shadow wall and ear struts."""
    np = np_mod()
    y_car, (cnt, c_mm) = anchors()[3], kl.HUB
    rb, yt = 1e-3 * float(c_mm), y_car + 0.0068
    holes = [csg.circle2d(0.013, fn=32)] + [csg.circle2d(0.0053, fn=16, at=(rb * math.cos(a), rb * math.sin(a)))
                                            for k in range(cnt) for a in (2.0 * math.pi * (k + 0.5) / cnt,)]
    flange = csg.extrude(csg.circle2d(0.041, fn=48), holes=holes, a=y_car + 0.0005, b=y_car + 0.0066,
                         axis=1, colour=CHASSIS_CF, chamfer=0.0007)
    pockets = [csg.filleted2d(csg.box2d(u, v), r=0.002)
               for u, v in (((-0.011, 0.011), (0.082, 0.090)), ((-0.038, -0.018), (0.085, 0.113)),
                            ((0.018, 0.038), (0.085, 0.113)))]
    bulk = csg.extrude(csg.filleted2d([(-0.041, yt), (0.041, yt), (0.043, 0.138), (-0.043, 0.138)], r=0.004),
                       holes=pockets, a=0.024, b=0.033, axis=2, colour=CHASSIS_CF, chamfer=0.0008)
    fore = csg.extrude(csg.filleted2d(csg.box2d((0.032, 0.056), (-0.039, 0.039)), r=0.003),
                       a=0.133, b=0.138, axis=1, colour=CHASSIS_CF, chamfer=0.0006)
    deck = csg.extrude(csg.filleted2d(csg.box2d((0.038, 0.052), (-0.036, 0.036)), r=0.003),
                       a=0.057, b=0.062, axis=1, colour=CHASSIS_CF, chamfer=0.0006)
    wyz = csg.filleted2d([(0.057, 0.026), (0.057, 0.050), (yt + 0.006, 0.033), (yt + 0.006, 0.024)], r=0.002)
    wl, wr = (csg.extrude(wyz, a=a, b=b, axis=0, colour=CHASSIS_CF, chamfer=0.0006)
              for a, b in ((0.029, 0.035), (-0.035, -0.029)))
    ports = [csg.filleted2d(csg.box2d(u, v), r=0.0042)
             for u, v in (((-0.046, -0.015), (0.087, 0.114)), ((0.015, 0.046), (0.087, 0.114)))]
    eye_m = csg.extrude(csg.filleted2d(csg.box2d((-0.048, 0.048), (0.083, 0.118)), r=0.005),
                        holes=ports, a=0.058, b=0.062, axis=2, colour=BAFFLE_DARK, chamfer=0.0005)
    cowl_o = [((0.032 + 0.0168 * math.cos(a)), (0.100 + 0.0132 * math.sin(a)))
              for a in (2.0 * math.pi * k / 28 for k in range(28))]
    cowl_i = [((0.032 + 0.0136 * math.cos(a)), (0.100 + 0.0108 * math.sin(a)))
              for a in (2.0 * math.pi * k / 28 for k in range(28))]
    cowl_l = csg.extrude(cowl_o, holes=[cowl_i], a=0.068, b=0.084, axis=2, colour=BAFFLE_DARK, chamfer=0.0005).tris()
    oral_g = np.array([[[0.026 * float(u), 0.020 + 0.046 * float(v), 0.078 - 0.014 * float(u * u) - 0.004 * (1.0 - float(v))]
                        for v in np.linspace(0.0, 1.0, 5)] for u in np.linspace(-1.0, 1.0, 11)])
    oral = shell_patch(oral_g, 0.0022, 0.00036, BAFFLE_DARK, (0.0, 0.045, 0.010), BAFFLE_DARK)
    ear = csg.sweep(csg.circle2d(0.0042, fn=18), [(0.040, 0.093, 0.026), (0.052, 0.093, 0.012), (0.061, 0.093, 0.004)],
                    chamfer_ends=0.0006, colour=CHASSIS_CF).tris()
    return csg.union(flange, bulk, fore, deck, wl, wr, eye_m).tris() + cowl_l + mirror_x(cowl_l) + oral + ear + mirror_x(ear)


@functools.lru_cache(maxsize=None)
def compute_heatsink():
    """Return recessed cranial compute fins and heatpipes behind the fine-mesh honeycomb cowls."""
    return fh.compute_heatsink()


def _dome_grid(u0=0.0, u1=1.0, nu=13, nv=10):
    """Sample one segmented slice of the round upper cranial dome along the swept mecha V-chevron seam."""
    np = np_mod()
    us, vs = np.linspace(u0, u1, nu), np.linspace(0.0, 1.0, nv)
    g = np.zeros((nu, nv, 3))
    for i, u in enumerate(us):
        th = 0.048 + float(u) * (math.pi - 0.096)
        y0 = _crown_seam_y(th) + 0.0032
        phi0 = math.asin(max(0.05, min(0.95, (y0 - 0.156) / 0.0702)))
        phi1 = 0.5 * math.pi - 0.044
        for j, v in enumerate(vs):
            phi = phi0 + float(v) * (phi1 - phi0)
            yv = 0.156 + 0.0702 * math.sin(phi)
            p = _head_pt(th, yv, 0.0, False, False)
            g[i, j] = (max(0.0024, p[0]), p[1], p[2])
    return g


@functools.lru_cache(maxsize=None)
def static_plates():
    """Build the mecha-gynoid head out of small interlocking planar-bevelled armor tiles & dense fine honeycomb."""
    # 1. Forehead (`cranium_frontal`): 6 stepped interlocking small mecha tiles per side + 8x6 central honeycomb intake!
    def _fr_th(u, v):
        t_in = 0.022 + (0.122 * (((u - 0.42) / 0.58) ** 1.15) if u > 0.42 else 0.0)
        t_out = 0.72 + 0.12 * u
        return t_in + v * (t_out - t_in)

    def _fr_y(u, v):
        th0 = _fr_th(0.0, v)
        th1 = _fr_th(1.0, v)
        y_lo = (_eye_top(th0) + 0.0086) if th0 >= 0.205 else (0.1112 - 0.0028 * ((1.0 - th0 / 0.205) ** 1.4))
        y_hi = _crown_seam_y(th1) - 0.0012
        return y_lo + u * (y_hi - y_lo)

    f_tiles = _mecha_tiles(_fr_th, _fr_y, (
        (0.00, 0.30, 0.00, 0.39, 0.0010, 6, 5), (0.00, 0.30, 0.42, 1.00, 0.0005, 6, 5),
        (0.33, 0.67, 0.00, 0.61, 0.0004, 6, 5), (0.33, 0.67, 0.64, 1.00, 0.0010, 6, 5),
        (0.70, 1.00, 0.00, 0.43, 0.0008, 6, 5), (0.70, 1.00, 0.46, 1.00, 0.0003, 6, 5),
    ), wall=0.0023, ch=0.00056, flat=0.58)
    cr_g = _surf_grid(lambda u, v: (0.032 + 0.088 * (u ** 1.15)) * (2.0 * v - 1.0),
                      lambda u, v: 0.140 + 0.0265 * u, 9, 7, off=-0.0008, nose=False)
    front = f_tiles + mirror_x(f_tiles) + fh.honeycomb_patch(cr_g, nc=8, nr=6, wall=0.0020, margin=(0.14, 0.15), fill=0.72, col=SHELL_CF)

    # 2. Segmented 2-panel fine-mesh crown (`cranium_dome`, 14x9 total micro-hex!) & deep occipital cowl (`y = 26 mm`!).
    d_lobe = (fh.honeycomb_patch(_dome_grid(0.00, 0.475, nu=12, nv=10), nc=7, nr=9, wall=0.0022, margin=(0.13, 0.14), fill=0.72, col=SHELL_CF)
              + fh.honeycomb_patch(_dome_grid(0.515, 1.00, nu=12, nv=10), nc=7, nr=9, wall=0.0022, margin=(0.13, 0.14), fill=0.72, col=SHELL_CF))
    o_lobe = fh.honeycomb_patch(_surf_grid(lambda u, v: 2.34 + v * (math.pi - 0.045 - 2.34),
                                           lambda u, v: 0.026 + u * (_crown_seam_y(2.34 + v * (math.pi - 0.045 - 2.34)) - 0.0012 - 0.026),
                                           18, 7, off=-0.0004, nose=False),
                                nc=10, nr=8, wall=0.0022, margin=(0.14, 0.14), fill=0.72, col=SHELL_CF)

    # 3. Two-segment swept forward-leaning mecha temple (`left_temporal`, 14x6 micro-hex) & deep mastoid (`y = 26 mm`).
    def _tmp_th(u, v):
        t0 = 0.77 + 0.11 * v
        return t0 + u * (2.28 - t0)

    def _tmp_y(u, v):
        th = _tmp_th(u, v)
        y_lo = _ear_clear_y(th, 0.1195 - 0.012 * u)
        return y_lo + v * (_crown_seam_y(th) - 0.0012 - y_lo)

    l_temp = (fh.honeycomb_patch(_surf_grid(lambda u, v: _tmp_th(0.475 * u, v), lambda u, v: _tmp_y(0.475 * u, v), 11, 7, off=-0.0002, nose=False),
                                 nc=7, nr=6, wall=0.0022, margin=(0.14, 0.15), fill=0.72, col=SHELL_CF)
              + fh.honeycomb_patch(_surf_grid(lambda u, v: _tmp_th(0.515 + 0.485 * u, v), lambda u, v: _tmp_y(0.515 + 0.485 * u, v), 11, 7, off=-0.0005, nose=False),
                                   nc=7, nr=6, wall=0.0022, margin=(0.14, 0.15), fill=0.72, col=SHELL_CF))
    l_mast = fh.honeycomb_patch(_surf_grid(lambda u, v: 2.00 + u * (2.30 - 2.00),
                                           lambda u, v: 0.026 + v * (_ear_clear_y(2.00 + u * (2.30 - 2.00), 0.104) - 0.005 - 0.026),
                                           14, 6, off=-0.0004, nose=False),
                                nc=7, nr=6, wall=0.0022, margin=(0.15, 0.15), fill=0.72, col=SHELL_CF)

    # 4. Two-tile swept mecha temple-orbital wing (`left_orbital_pillar`).
    l_pil = _mecha_tiles(lambda u, v: 0.790 + v * (1.28 - 0.790), lambda u, v: 0.0988 + u * (0.1142 - 0.0988), (
        (0.0, 1.0, 0.00, 0.47, 0.0010, 5, 5), (0.0, 1.0, 0.53, 1.00, 0.0005, 5, 5),
    ), wall=0.0022, ch=0.00050, flat=0.55)

    # 5. Two-segment petite faceted anime-mecha nose (`nasal_bridge`: glabella diamond + slender faceted tip).
    l_nose = _mecha_tiles(
        lambda u, v: (0.094 + 0.016 * math.sin(math.pi * (u ** 0.85)) + 0.056 * (u ** 2.4)) * (2.0 * v - 1.0),
        lambda u, v: 0.0685 + u * (0.1080 - 0.0685),
        ((0.00, 0.50, 0.0, 1.0, 0.0009, 9, 5), (0.55, 1.00, 0.0, 1.0, 0.0005, 8, 5)),
        wall=0.0023, ch=0.00052, flat=0.40, nose=True)

    # 6. MULTI-TILE MECHA CHEEKBONE & CHEEK (`left_maxilla`): 3 stepped upper malar tiles + anterior cheek facet + 10x6 micro-honeycomb!
    def _zyg_th(u, v):
        return 0.155 + u * (1.33 - 0.155)

    def _zyg_y(u, v):
        th = _zyg_th(u, v)
        yh = _eye_bot(th) if th <= 0.775 else (0.0962 - 0.0018 * ((th - 0.775) / (1.33 - 0.775)))
        med = -0.0145 * ((1.0 - min(1.0, (th - 0.155) / 0.34)) ** 1.2)
        yl = 0.0710 + 0.0072 * ((th - 0.155) / (1.33 - 0.155)) + med
        return yl + v * (yh - yl)

    zyg_tiles = _mecha_tiles(_zyg_th, _zyg_y, (
        (0.00, 0.28, 0.0, 1.0, 0.0006, 6, 5),
        (0.32, 0.65, 0.0, 1.0, 0.0011, 7, 5),
        (0.69, 1.00, 0.0, 1.0, 0.0007, 7, 5),
    ), wall=0.0023, ch=0.00054, flat=0.58)

    def _sub_th(u, v):
        return 0.405 + u * (1.31 - 0.405)

    def _sub_y(u, v):
        th = _sub_th(u, v)
        t = (th - 0.405) / (1.31 - 0.405)
        yh = 0.0680 + 0.0082 * t
        yl = 0.0445 + 0.0145 * (t ** 1.15)
        return yl + v * (yh - yl)

    sub_front = _mecha_tiles(_sub_th, _sub_y, ((0.00, 0.26, 0.0, 1.0, 0.0004, 5, 5),), wall=0.0021, ch=0.00048, flat=0.60)
    sub_hc = fh.honeycomb_patch(_surf_grid(lambda u, v: _sub_th(0.30 + 0.70 * u, v),
                                           lambda u, v: _sub_y(0.30 + 0.70 * u, v), 16, 7, off=-0.0005, nose=False),
                                nc=10, nr=6, wall=0.0020, margin=(0.13, 0.14), fill=0.72, col=SHELL_CF)

    # 7. Petite serene anime-feminine sub-nasal & upper-mouth armor plates (`philtrum`, `upper_lip`).
    ph_g = _planar_grid(_surf_grid(lambda u, v: (0.136 - 0.016 * v) * (2.0 * u - 1.0),
                                   lambda u, v: 0.0550 + v * (0.0660 - 0.0550), 6, 4, off=0.0005, nose=False), 0.48)
    lip_g = _planar_grid(_surf_grid(lambda u, v: 0.222 * (2.0 * u - 1.0),
                                    lambda u, v: 0.0468 + v * (0.0532 - 0.0468) + 0.0014 * ((2.0 * u - 1.0) ** 2),
                                    8, 4, off=0.0007, nose=False), 0.40)

    l_max = zyg_tiles + sub_front + sub_hc
    return {'cranium_frontal': front, 'cranium_dome': d_lobe + mirror_x(d_lobe), 'cranium_occipital': o_lobe + mirror_x(o_lobe),
            'left_temporal': l_temp, 'right_temporal': mirror_x(l_temp), 'left_mastoid': l_mast, 'right_mastoid': mirror_x(l_mast),
            'left_orbital_pillar': l_pil, 'right_orbital_pillar': mirror_x(l_pil),
            'nasal_bridge': l_nose, 'left_maxilla': l_max, 'right_maxilla': mirror_x(l_max),
            'philtrum': shell_patch(ph_g, 0.0022, 0.00046, SHELL_CF), 'upper_lip': shell_patch(lip_g, 0.0022, 0.00046, SHELL_CF)}


@functools.lru_cache(maxsize=None)
def ear_microphones():
    """Build concentric circular binaural mecha ear-pods (`x_max ~ 70.2 mm`) flush on the temples."""
    ye, ze = 0.093, 0.004
    outer = rev([(0.0156, 0.0615), (0.0156, 0.0692), (0.0192, 0.0702), (0.0206, 0.0686),
                 (0.0206, 0.0610), (0.0188, 0.0598), (0.0156, 0.0615)], 0, (ye, ze), 48, SHELL_CF)
    inner_t = rev([(0.0102, 0.0640), (0.0102, 0.0688), (0.0128, 0.0694), (0.0138, 0.0682), (0.0138, 0.0640)], 0, (ye, ze), 40, TITANIUM)
    accent = rev([(0.0138, 0.0668), (0.0154, 0.0676), (0.0154, 0.0652), (0.0138, 0.0652)], 0, (ye, ze), 44, MIC_GOLD)
    holes = [csg.circle2d(0.0014, fn=12, at=(ye + 0.0068 * math.cos(math.radians(60.0 * k)), ze + 0.0068 * math.sin(math.radians(60.0 * k))))
             for k in range(6)] + [csg.circle2d(0.0032, fn=20, at=(ye, ze))]
    baffle = csg.extrude(csg.circle2d(0.0155, fn=40, at=(ye, ze)), holes=holes, a=0.0645, b=0.0662, axis=0, colour=BAFFLE_DARK, chamfer=0.0003).tris()
    core = rev([(0.0, 0.0645), (0.0030, 0.0645), (0.0030, 0.0678), (0.0018, 0.0684), (0.0, 0.0684)], 0, (ye, ze), 28, MIC_GOLD)
    left = outer + inner_t + accent + baffle + core
    return {'left_ear_mic': left, 'right_ear_mic': mirror_x(left)}


@functools.lru_cache(maxsize=None)
def camera_and_gimbal_local():
    """Return recessed expressive anime-mecha ocular camera module and 2-axis gimbal."""
    return fh.modern_optics_local()


@functools.lru_cache(maxsize=None)
def moving_rest():
    """Slice delicate anime brows, dark iris shutters, petite cheek-corner vane and multi-tile low feminine mandible."""
    from machine.eyes import gimbal, stereo
    np, ex = np_mod(), 0.5 * float(stereo.BASELINE_M)

    brow_g = _planar_grid(_surf_grid(lambda u, v: 0.238 + u * (0.735 - 0.238),
                                     lambda u, v: _eye_top(0.238 + u * (0.735 - 0.238)) + 0.0012 + v * 0.0040,
                                     9, 4, off=0.0028, nose=False), 0.42)

    piv = (np.array([ex, float(gimbal.EYES[1]), float(gimbal.EYES[2])])
           - gimbal.mounted('left') @ np.array([0.0, 0.0, -float(gimbal.LEVER['left'])]))
    lx_u = np.linspace(float(piv[0]) - 0.0100, float(piv[0]) + 0.0100, 9)

    def _lid_grid(a0, a1, tilt_deg=3.0):
        angs = np.linspace(math.radians(a0), math.radians(a1), 4)
        g = np.zeros((len(lx_u), len(angs), 3))
        for i, xv in enumerate(lx_u):
            dx = float(xv) - float(piv[0])
            u2 = (dx / 0.0100) ** 2
            rc = math.sqrt(max(36e-6, 0.0106 ** 2 - dx * dx))
            da = math.radians(tilt_deg) * (dx / 0.0100)
            z_tuck = -0.0026 * (u2 ** 1.15)
            for j, a in enumerate(angs):
                af = float(a) + da
                g[i, j] = (float(xv), float(piv[1]) + rc * math.sin(af), float(piv[2]) + rc * math.cos(af) + z_tuck)
        return g

    comm_g = _planar_grid(_surf_grid(lambda u, v: 0.250 + u * (0.370 - 0.250),
                                     lambda u, v: 0.0472 + 0.0014 * u + v * 0.0044, 5, 4, off=0.0005, nose=False), 0.50)

    # Multi-tile petite feminine mecha chin (`y = 5.0..44.2 mm`): lower-lip bezel + central chin shield + bilateral mental facets!
    def _ch_th(u, v):
        return (0.36 - 0.04 * (1.0 - v)) * (2.0 * u - 1.0)

    def _ch_y(u, v):
        return 0.0052 + v * (0.0442 - 0.0052) + 0.0014 * ((2.0 * u - 1.0) ** 2) * (1.0 - v)

    chin_tiles = _mecha_tiles(_ch_th, _ch_y, (
        (0.00, 1.00, 0.56, 1.00, 0.0007, 9, 4),
        (0.24, 0.76, 0.00, 0.51, 0.0009, 7, 5),
        (0.00, 0.20, 0.00, 0.51, 0.0003, 5, 5),
        (0.80, 1.00, 0.00, 0.51, 0.0003, 5, 5),
    ), wall=0.0022, ch=0.00050, flat=0.55)

    # Sculpted dark titanium sub-chin floor closing the underside of the jaw (`z = 60..92 mm`, clear of `mount_crown` in speaking!).
    sub_ch = np.array([[[(0.017 + 0.020 * float(v)) * float(u), 0.0054 + 0.011 * float(v),
                         0.092 - 0.032 * float(v) - 0.010 * (float(u) ** 2)]
                        for v in np.linspace(0.0, 1.0, 4)] for u in np.linspace(-1.0, 1.0, 7)])
    under_jaw = shell_patch(sub_ch, 0.0020, 0.00036, PANEL_RIM, (0.0, 0.060, 0.055), BAFFLE_DARK)

    # Segmented natural low jawline (`y = 7..34 mm` enclosing neck carrier!): anterior white mecha facet + dense 10x5 honeycomb jaw cowl!
    def _jaw_th(u, v):
        th_in = 0.35 + 0.04 * v
        return th_in + u * (1.38 - th_in)

    def _jaw_y(u, v):
        yl = 0.0072 + 0.0265 * (u ** 1.35)
        yh = 0.0420 + 0.0160 * (u ** 1.15)
        return yl + v * (yh - yl)

    lj_front = _mecha_tiles(_jaw_th, _jaw_y, ((0.00, 0.31, 0.0, 1.0, 0.0005, 6, 5),), wall=0.0021, ch=0.00048, flat=0.60)
    lj_hc = fh.honeycomb_patch(_surf_grid(lambda u, v: _jaw_th(0.35 + 0.65 * u, v),
                                          lambda u, v: _jaw_y(0.35 + 0.65 * u, v), 16, 7, off=-0.0004, nose=False),
                               nc=10, nr=5, wall=0.0020, margin=(0.13, 0.15), fill=0.72, col=SHELL_CF)
    lj_s = lj_front + lj_hc

    ryz = csg.filleted2d([(0.030, 0.060), (0.044, 0.064), (0.068, 0.028), (0.068, 0.020), (0.060, 0.020), (0.031, 0.052)], r=0.0020)
    ram = (csg.extrude(ryz, a=0.0398, b=0.0434, axis=0, colour=CHASSIS_CF, chamfer=0.0006).tris()
           + csg.extrude(csg.circle2d(0.0036, fn=20, at=(0.066, 0.024)), a=0.0434, b=0.0468, axis=0, colour=TITANIUM, chamfer=0.0004).tris())
    yoke = csg.extrude(csg.filleted2d(csg.box2d((-0.024, 0.024), (0.019, 0.025)), r=0.0016), a=0.074, b=0.079, axis=2, colour=CHASSIS_CF, chamfer=0.0004).tris()
    jaw_all = chin_tiles + under_jaw + lj_s + mirror_x(lj_s) + yoke + ram + mirror_x(ram)
    ulid = shell_patch(_lid_grid(24.0, 46.0, 3.0), 0.0011, 0.00024, PANEL_RIM, piv, BAFFLE_DARK)
    llid = shell_patch(_lid_grid(-40.0, -21.0, 2.0), 0.0011, 0.00024, PANEL_RIM, piv, BAFFLE_DARK)
    return shell_patch(brow_g, 0.0020, 0.00044, SHELL_CF), ulid, llid, piv, shell_patch(comm_g, 0.0019, 0.00038, SHELL_CF), jaw_all
