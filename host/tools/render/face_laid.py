"""Rounded-corner framed-honeycomb anime-mecha gynoid face panels without mouth."""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg, face_honeycomb as fh, knee_loads as kl, neck_laid as nl  # noqa: E402
from tools.render.face_honeycomb import (
    BAFFLE_DARK, BROW_DARK, CHASSIS_CF, MESH_PEARL, PANEL_RIM, SEAM_SILVER, SHELL_CF, TITANIUM,
    mirror_x, np_mod, shell_patch,
)

_PROF_Y = (-0.0088, 0.004, 0.022, 0.042, 0.060, 0.076, 0.090, 0.104, 0.118, 0.138, 0.156)
_PROF_X = (0.0265, 0.0412, 0.0522, 0.0605, 0.0672, 0.0704, 0.0712, 0.0708, 0.0696, 0.0718, 0.0724)
_PROF_ZF = (0.094, 0.1025, 0.1045, 0.1072, 0.1075, 0.1080, 0.1074, 0.1062, 0.1066, 0.1018, 0.0928)
_PROF_ZB = (-0.040, -0.048, -0.058, -0.066, -0.073, -0.078, -0.082, -0.085, -0.088, -0.092, -0.094)


@functools.lru_cache(maxsize=None)
def anchors():
    """Return `(neck_torso, head_rel, head_torso, y_carrier, r_carrier)` in m."""
    from machine.body import figure
    seg = {s[0]: s for s in figure.SEGMENTS}
    nt, hr = tuple(float(v) for v in seg['neck'][3]), tuple(float(v) for v in seg['head'][3])
    ht, lay = (nt[0] + hr[0], nt[1] + hr[1], nt[2] + hr[2]), nl.laid()
    return nt, hr, ht, float(lay['head']['carrier'][1] - ht[1]), float(lay['head']['gear'])


def _pchip(y, xs, ys):
    if y <= xs[0]:
        return ys[0]
    if y >= xs[-1]:
        return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= y:
            lo = mid
        else:
            hi = mid
    t = (y - xs[lo]) / (xs[hi] - xs[lo])
    s = t * t * (3.0 - 2.0 * t)
    return 0.55 * (ys[lo] + t * (ys[hi] - ys[lo])) + 0.45 * (ys[lo] + s * (ys[hi] - ys[lo]))


def _profile_at(y):
    if y >= 0.156:
        u = min(1.0, max(0.0, (y - 0.156) / 0.0704))
        c = math.sqrt(max(1e-6, 1.0 - u * u))
        return 0.0724 * c, -0.001 + 0.0938 * c, -0.001 - 0.0930 * c
    return _pchip(y, _PROF_Y, _PROF_X), _pchip(y, _PROF_Y, _PROF_ZF), _pchip(y, _PROF_Y, _PROF_ZB)


def _head_pt(th_rad, y, radial_off=0.0, nose=True, face_sculpt=True):
    """Evaluate smooth mouthless anime-feminine mecha head surface at azimuth `th_rad` and height `y`."""
    xw, zf, zb = _profile_at(y)
    zc, rz = 0.5 * (zf + zb), max(0.004, 0.5 * (zf - zb))
    st, ct, ath = math.sin(th_rad), math.cos(th_rad), abs(th_rad)
    front_w = max(0.0, ct)
    taper_y = max(0.0, min(1.0, (0.068 - y) / 0.072))
    p_exp = 2.00 - 0.06 * front_w * max(0.0, min(1.0, (0.130 - y) / 0.100))
    rx_eff = xw * (1.0 - 0.042 * (front_w ** 1.3) * (taper_y ** 1.15))
    rx_eff -= 0.0018 * math.exp(-((y - 0.134) / 0.025) ** 2) * math.exp(-((ath - 0.90) / 0.30) ** 2)
    x = rx_eff * math.copysign(abs(st) ** (2.0 / p_exp), st)
    z = zc + rz * math.copysign(abs(ct) ** (2.0 / p_exp), ct)

    if face_sculpt and ct > 0.05:
        y_orb = 0.1004 + 0.0012 * (ath - 0.48)
        z -= 0.0004 * math.exp(-1.45 * (((ath - 0.48) / 0.26) ** 2 + ((y - y_orb) / 0.0170) ** 2))
        y_chk = 0.0710 + 0.0038 * min(1.0, max(0.0, (ath - 0.22) / 0.75))
        ck = math.exp(-((ath - 0.54) / 0.48) ** 2) * math.exp(-((y - y_chk) / 0.024) ** 2)
        x += math.copysign(0.0024 * ck, st) if abs(st) > 1e-6 else 0.0
        z += 0.0030 * ck
        y_br = 0.1152 + 0.0018 * min(1.0, max(0.0, (ath - 0.20) / 0.56))
        z += 0.0014 * math.exp(-((y - y_br) / 0.0070) ** 2) * math.exp(-((ath - 0.48) / 0.40) ** 2)
        z += 0.0022 * math.exp(-(ath / 0.28) ** 2) * math.exp(-((y - 0.0050) / 0.0170) ** 2)

    if nose and ct > 0.48 and 0.060 <= y <= 0.107:
        tip = math.exp(-((y - 0.0730) / 0.0068) ** 2)
        br = 0.0017 * math.sin(math.pi * max(0.0, min(1.0, (y - 0.060) / 0.047)))
        z += (br + 0.0056 * tip) * max(0.0, 1.0 - (ath / (0.040 + 0.024 * tip)) ** 1.6)

    if radial_off != 0.0:
        r_xz = math.hypot(x, z - zc)
        if r_xz > 1e-6:
            x += radial_off * (x / r_xz)
            z += radial_off * ((z - zc) / r_xz)
    return (x, y, z)


def _surf_grid(th_fn, y_fn, nu, nv, off=0.0, nose=True, sculpt=True):
    np = np_mod()
    us, vs = np.linspace(0.0, 1.0, nu), np.linspace(0.0, 1.0, nv)
    g = np.zeros((nu, nv, 3))
    for i, u in enumerate(us):
        fu = float(u)
        for j, v in enumerate(vs):
            fv = float(v)
            g[i, j] = _head_pt(float(th_fn(fu, fv)), float(y_fn(fu, fv)), off, nose, sculpt)
    return g


def _framed_panel(th_fn, y_fn, nu=14, nv=10, off=0.0003, wall=0.0021, rim_m=0.0025, pitch=0.0044,
                  corner_r=0.0064, nose=False, fill=0.64, bar_col=MESH_PEARL):
    g = _surf_grid(th_fn, y_fn, nu, nv, off=off, nose=nose)
    return fh.honeycomb_patch(g, wall=wall, rim_m=rim_m, pitch=pitch, fill=fill, corner_r=corner_r,
                              col=SHELL_CF, bar_col=bar_col, hole_col=BAFFLE_DARK, rim_col=SEAM_SILVER)


def _eye_top(th):
    t = max(0.0, min(1.0, (th - 0.275) / 0.395))
    return 0.0998 + 0.0024 * t + 0.0094 * math.sin(math.pi * (t ** 0.86))


def _eye_bot(th):
    t = max(0.0, min(1.0, (th - 0.275) / 0.395))
    return 0.0980 + 0.0020 * t - 0.0072 * math.sin(math.pi * (t ** 0.90))


def _crown_seam_y(th):
    return 0.1585 + 0.0175 * (min(1.0, abs(th) / 0.92) ** 1.25)


def _ear_clear_y(th, y_base):
    p = _head_pt(th, y_base, 0.0, False, False)
    dz = p[2] - 0.004
    if abs(dz) < 0.0272:
        y_arc = 0.093 + math.sqrt(max(0.0, 0.0272 ** 2 - dz * dz))
        return 0.5 * (y_base + y_arc + math.hypot(y_base - y_arc, 0.0036))
    return y_base


def _dome_ab(alpha, beta, off=0.0):
    """Evaluate pole-free left dome ellipsoid point at sagittal angle `alpha` and lateral angle `beta`."""
    sx, sy, sz = math.sin(beta), math.cos(beta) * math.cos(alpha), math.cos(beta) * math.sin(alpha)
    x, y, z = 0.0724 * sx, 0.156 + 0.0704 * sy, -0.001 + (0.0938 if sz >= 0.0 else 0.0930) * sz
    if off != 0.0:
        n = math.sqrt((x / 0.0724) ** 2 + (sy ** 2) + ((z + 0.001) / 0.0934) ** 2)
        if n > 1e-6:
            x += off * sx / n
            y += off * sy / n
            z += off * sz / n
    return (max(0.0022, x), y, z)


def _dome_strip(u0, u1, v0, v1, nu=18, nv=10, off=0.0):
    """Pole-free sagittal-lateral grid over the left dome so hexagons stay equilateral across the apex."""
    np = np_mod()
    us, vs = np.linspace(u0, u1, nu), np.linspace(v0, v1, nv)
    g = np.zeros((nu, nv, 3))
    a_f, a_b, b_min = 1.44, -1.20, 0.036
    for i, u in enumerate(us):
        al = a_f + float(u) * (a_b - a_f)
        lo, hi = b_min + 0.06, 1.50
        for _ in range(10):
            mid = 0.5 * (lo + hi)
            px, py, pz = _dome_ab(al, mid, 0.0)
            th = math.atan2(px, pz + 0.001)
            if py > _crown_seam_y(th) + 0.0026:
                lo = mid
            else:
                hi = mid
        b_max = max(b_min + 0.12, lo)
        for j, v in enumerate(vs):
            g[i, j] = _dome_ab(al, b_min + float(v) * (b_max - b_min), off)
    return g


@functools.lru_cache(maxsize=None)
def chassis_static():
    """Build open structural carbon chassis, flange, eye bridge and ear struts (no outer liner)."""
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
    wl, wr = (csg.extrude(wyz, a=a, b=b, axis=0, colour=CHASSIS_CF, chamfer=0.0006) for a, b in ((0.029, 0.035), (-0.035, -0.029)))
    ports = [csg.filleted2d(csg.box2d(u, v), r=0.0042)
             for u, v in (((-0.045, -0.017), (0.087, 0.114)), ((0.017, 0.045), (0.087, 0.114)))]
    eye_m = csg.extrude(csg.filleted2d(csg.box2d((-0.048, 0.048), (0.083, 0.118)), r=0.005),
                        holes=ports, a=0.058, b=0.068, axis=2, colour=BAFFLE_DARK, chamfer=0.0005)
    ear = csg.sweep(csg.circle2d(0.0042, fn=18), [(0.040, 0.093, 0.026), (0.052, 0.093, 0.012), (0.061, 0.093, 0.004)],
                    chamfer_ends=0.0006, colour=CHASSIS_CF).tris()
    return csg.union(flange, bulk, fore, deck, wl, wr, eye_m).tris() + ear + mirror_x(ear)


@functools.lru_cache(maxsize=None)
def inner_liner():
    """Return uniform matte-dark inner shadow baffles mounted only when outer plates are present."""
    np = np_mod()
    glab_b = csg.extrude(csg.filleted2d(csg.box2d((-0.0135, 0.0135), (0.086, 0.108)), r=0.002),
                         a=0.072, b=0.086, axis=2, colour=BAFFLE_DARK, chamfer=0.0004).tris()
    cowl_o = [((0.032 + 0.0176 * math.cos(a)), (0.100 + 0.0168 * math.sin(a))) for a in (2.0 * math.pi * k / 28 for k in range(28))]
    cowl_i = [((0.032 + 0.0150 * math.cos(a)), (0.100 + 0.0148 * math.sin(a))) for a in (2.0 * math.pi * k / 28 for k in range(28))]
    cowl_l = csg.extrude(cowl_o, holes=[cowl_i], a=0.068, b=0.079, axis=2, colour=BAFFLE_DARK, chamfer=0.0004).tris()
    liner = []
    for t0, ts, y0, ys, nu, nv, of in (
        (0.000, 2.27, 0.114, 0.066, 8, 12, -0.0046),
        (0.72, 0.60, 0.088, 0.034, 6, 7, -0.0048),
        (0.00, 1.48, 0.052, 0.036, 6, 10, -0.0048),
        (1.98, math.pi - 2.00, 0.018, 0.160, 8, 8, -0.0048),
        (1.18, 0.82, 0.028, 0.090, 8, 7, -0.0050),
    ):
        s_p = shell_patch(_surf_grid(lambda u, v, a=t0, b=ts: a + v * b, lambda u, v, c=y0, d=ys: c + u * d,
                                     nu, nv, off=of, nose=False, sculpt=False), 0.0012, 0.00022, BAFFLE_DARK, rim_col=BAFFLE_DARK)
        liner += s_p + mirror_x(s_p)
    s_d = shell_patch(_dome_strip(0.00, 0.99, 0.00, 0.97, nu=12, nv=6, off=-0.0048), 0.0012, 0.00022, BAFFLE_DARK, rim_col=BAFFLE_DARK)
    # Recessed gunmetal-titanium mecha seam rails visible cleanly inside all wide 4.2-5.0 mm gaps.
    rail_m = shell_patch(_surf_grid(lambda u, v: 0.012 * (2.0 * v - 1.0), lambda u, v: 0.106 + u * 0.056,
                                    8, 4, off=-0.0036, nose=False, sculpt=False), 0.0010, 0.00020, PANEL_RIM, rim_col=TITANIUM)
    rail_fr = shell_patch(_surf_grid(lambda u, v: 0.03 + u * 0.76,
                                     lambda u, v: (0.1320 + 0.0175 * (min(1.0, (0.03 + u * 0.76) / 0.80) ** 1.15)) - 0.0010 + v * 0.0020,
                                     10, 3, off=-0.0036, nose=False, sculpt=False), 0.0009, 0.00018, PANEL_RIM, rim_col=TITANIUM)
    rail_c = shell_patch(_surf_grid(lambda u, v: 0.18 + u * 1.12, lambda u, v: (0.0558 + 0.0245 * (u ** 1.15)) + v * 0.0022,
                                    10, 3, off=-0.0038, nose=False, sculpt=False), 0.0009, 0.00018, PANEL_RIM, rim_col=TITANIUM)
    rail_chin = shell_patch(_surf_grid(lambda u, v: 0.011 * (2.0 * v - 1.0), lambda u, v: 0.002 + u * 0.050,
                                       8, 3, off=-0.0038, nose=False, sculpt=False), 0.0009, 0.00018, PANEL_RIM, rim_col=TITANIUM)
    oral_g = np.array([[[0.026 * float(u), 0.014 + 0.048 * float(v), 0.072 - 0.014 * float(u * u) - 0.003 * (1.0 - float(v))]
                        for v in np.linspace(0.0, 1.0, 5)] for u in np.linspace(-1.0, 1.0, 9)])
    oral = shell_patch(oral_g, 0.0018, 0.00030, BAFFLE_DARK, (0.0, 0.042, 0.010), BAFFLE_DARK)
    rails = rail_m + rail_chin + rail_fr + mirror_x(rail_fr) + rail_c + mirror_x(rail_c)
    return glab_b + cowl_l + mirror_x(cowl_l) + oral + liner + rails + s_d + mirror_x(s_d)


@functools.lru_cache(maxsize=None)
def compute_heatsink():
    """Return recessed cranial compute fins and heatpipes inside the cranial bay."""
    return fh.compute_heatsink()


@functools.lru_cache(maxsize=None)
def static_plates():
    """Build swept feminine rounded pearl-framed honeycomb panels with bold 4.2-5.0 mm mecha gaps."""
    def _fr_y_lo(th):
        ath = abs(th)
        return (_eye_top(ath) + 0.0026) if 0.275 <= ath <= 0.670 else (
            (0.1088 - 0.0016 * ((1.0 - ath / 0.275) ** 1.3)) if ath < 0.275 else (0.1074 + 0.0065 * ((ath - 0.670) / 0.12)))

    def _fr_mid_y(th):
        return 0.1320 + 0.0175 * (min(1.0, abs(th) / 0.80) ** 1.15)

    # Forehead: TWO swept feminine tiara-arched rounded framed-honeycomb panels per side with 4.6 mm gaps.
    f_lo = _framed_panel(
        lambda u, v: (0.032 + 0.004 * v) + u * ((0.695 + 0.075 * v) - (0.032 + 0.004 * v)),
        lambda u, v: _fr_y_lo(0.032 + u * 0.663) + v * (
            _fr_mid_y(0.036 + u * 0.734) - 0.0023 - _fr_y_lo(0.032 + u * 0.663)),
        nu=15, nv=9, off=0.0013, wall=0.0018, rim_m=0.0033, pitch=0.0050, corner_r=0.0065, fill=0.52)
    f_hi = _framed_panel(
        lambda u, v: 0.032 + u * (0.790 - 0.032),
        lambda u, v: (_fr_mid_y(0.032 + u * 0.758) + 0.0023) + v * (
            _crown_seam_y(0.032 + u * 0.758) - 0.0024 - (_fr_mid_y(0.032 + u * 0.758) + 0.0023)),
        nu=15, nv=9, off=0.0005, wall=0.0020, rim_m=0.0029, pitch=0.0052, corner_r=0.0072, fill=0.52)
    front = f_lo + mirror_x(f_lo) + f_hi + mirror_x(f_hi)

    d_lobe = fh.honeycomb_patch(_dome_strip(0.00, 1.00, 0.00, 1.00, nu=22, nv=11, off=0.0004),
                                wall=0.0021, rim_m=0.0029, pitch=0.0062, fill=0.54, corner_r=0.0082)
    o_lobe = _framed_panel(lambda u, v: 2.30 + u * (math.pi - 2.345),
                           lambda u, v: 0.016 + v * (_crown_seam_y(2.30 + u * 0.80) - 0.0185),
                           nu=14, nv=12, off=0.0004, wall=0.0021, rim_m=0.0029, pitch=0.0062, corner_r=0.0082, fill=0.54)

    def _tmp_th(u, v):
        t0 = 0.865 + 0.025 * v
        return t0 + u * (2.225 - t0)

    def _tmp_y(u, v):
        th = _tmp_th(u, v)
        y_lo = _ear_clear_y(th, 0.1195 - 0.004 * u)
        return y_lo + v * (_crown_seam_y(th) - 0.0024 - y_lo)

    l_temp = _framed_panel(_tmp_th, _tmp_y, nu=16, nv=10, off=0.0004, wall=0.0021, rim_m=0.0028, pitch=0.0060, corner_r=0.0080, fill=0.54)
    l_mast = _framed_panel(lambda u, v: 1.99 + u * 0.24,
                           lambda u, v: 0.016 + v * (_ear_clear_y(1.99 + u * 0.24, 0.100) - 0.0220),
                           nu=10, nv=9, off=0.0003, wall=0.0020, rim_m=0.0025, pitch=0.0056, corner_r=0.0066, fill=0.54)
    l_pil = _framed_panel(lambda u, v: (0.745 + 0.065 * v) + u * (1.29 - (0.745 + 0.065 * v)),
                          lambda u, v: (0.0980 + 0.0028 * u) + v * 0.0138, nu=11, nv=7, off=0.0003, wall=0.0020,
                          rim_m=0.0023, pitch=0.0046, corner_r=0.0048, fill=0.52)

    # Nose: ONE petite slender tapered feminine pearl-honeycomb shield with bold flanking gaps.
    l_nose = _framed_panel(
        lambda u, v: (0.068 + 0.020 * math.sin(math.pi * (max(0.0, 1.0 - v) ** 0.72)) - 0.014 * (v ** 1.4)) * (2.0 * u - 1.0),
        lambda u, v: 0.0620 + v * 0.0425,
        nu=12, nv=16, off=0.0005, wall=0.0019, rim_m=0.0019, pitch=0.0038, corner_r=0.0040, nose=True, fill=0.48, bar_col=(234, 238, 246))

    # Cheekbone: ONE graceful high-cheekbone feminine wing panel per side.
    def _chk_th(u, v):
        t_in = 0.154 + 0.026 * (v ** 1.40)
        return t_in + u * ((1.31 - 0.04 * v) - t_in)

    def _chk_y(u, v):
        th = _chk_th(u, v)
        yh = (_eye_bot(th) - 0.0016) if 0.275 <= th <= 0.670 else (
            (0.0968 - 0.0038 * ((0.275 - th) / 0.12)) if th < 0.275 else (0.0940 + 0.0032 * min(1.0, (th - 0.670) / 0.55)))
        yl = 0.0598 + 0.0245 * (u ** 1.15)
        return yl + v * (yh - yl)

    l_max = _framed_panel(_chk_th, _chk_y, nu=18, nv=10, off=0.0006, wall=0.0020, rim_m=0.0028, pitch=0.0050, corner_r=0.0078, fill=0.52)

    # Recessed dark inner baffle plates behind the nose-to-lower-cheek gap (no external mouth).
    ph_p = shell_patch(_surf_grid(lambda u, v: 0.085 * (2.0 * u - 1.0), lambda u, v: 0.0525 + v * 0.0085,
                                  8, 5, off=-0.0115, nose=False, sculpt=False), 0.0014, 0.00022, BAFFLE_DARK, rim_col=BAFFLE_DARK)
    lip_p = shell_patch(_surf_grid(lambda u, v: 0.095 * (2.0 * u - 1.0), lambda u, v: 0.0435 + v * 0.0065,
                                   8, 5, off=-0.0130, nose=False, sculpt=False), 0.0014, 0.00022, BAFFLE_DARK, rim_col=BAFFLE_DARK)
    return {'cranium_frontal': front, 'cranium_dome': d_lobe + mirror_x(d_lobe), 'cranium_occipital': o_lobe + mirror_x(o_lobe),
            'left_temporal': l_temp, 'right_temporal': mirror_x(l_temp), 'left_mastoid': l_mast, 'right_mastoid': mirror_x(l_mast),
            'left_orbital_pillar': l_pil, 'right_orbital_pillar': mirror_x(l_pil),
            'nasal_bridge': l_nose, 'left_maxilla': l_max, 'right_maxilla': mirror_x(l_max), 'philtrum': ph_p, 'upper_lip': lip_p}


@functools.lru_cache(maxsize=None)
def ear_microphones():
    """Return concentric circular binaural mecha ear-pods flush on the temples."""
    return fh.ear_microphones()


@functools.lru_cache(maxsize=None)
def camera_and_gimbal_local():
    """Return anime-mecha ocular camera module with porcelain sclera, sapphire iris and catchlights."""
    return fh.modern_optics_local()


@functools.lru_cache(maxsize=None)
def moving_rest():
    """Build arched anime brow, eyelids, internal smile lug and two meeting rounded lower-cheek panels."""
    from machine.eyes import gimbal, stereo
    np, ex = np_mod(), 0.5 * float(stereo.BASELINE_M)
    brow_g = _surf_grid(
        lambda u, v: 0.295 + u * 0.375,
        lambda u, v: (_eye_top(0.295 + u * 0.375) + 0.0033 + 0.0011 * math.sin(math.pi * (u ** 1.25)) + 0.0008 * u)
        + v * (0.0018 * (1.0 - 0.48 * (u ** 1.4))),
        13, 4, off=0.0038, nose=False)
    piv = (np.array([ex, float(gimbal.EYES[1]), float(gimbal.EYES[2])])
           - gimbal.mounted('left') @ np.array([0.0, 0.0, -float(gimbal.LEVER['left'])]))
    lx_u = np.linspace(float(piv[0]) - 0.0108, float(piv[0]) + 0.0108, 13)

    def _lid_grid(a0, a1, tilt_deg=2.2):
        angs = np.linspace(math.radians(a0), math.radians(a1), 5)
        g = np.zeros((len(lx_u), len(angs), 3))
        for i, xv in enumerate(lx_u):
            dx = float(xv) - float(piv[0])
            rc = math.sqrt(max(44e-6, 0.0119 ** 2 - dx * dx))
            da, z_tuck = math.radians(tilt_deg) * (dx / 0.0108), -0.0004 * (((dx / 0.0108) ** 2) ** 1.15)
            for j, a in enumerate(angs):
                af = float(a) + da
                g[i, j] = (float(xv), float(piv[1]) + rc * math.sin(af), float(piv[2]) + rc * math.cos(af) + z_tuck)
        return g

    # Recessed internal cheek-actuator lug driven by smile linkage (invisible externally, no mouth).
    comm_g = _surf_grid(lambda u, v: 0.270 + u * 0.055, lambda u, v: 0.0430 + 0.0006 * u + v * 0.0040,
                        6, 4, off=-0.0102, nose=False, sculpt=False)

    # Lower cheek: TWO soft feminine heart-oval rounded framed-honeycomb panels meeting across 4.2 mm midline gap, NO mouth.
    def _lchk_th(u, v):
        t_in = 0.032 + 0.007 * (1.0 - v)
        return t_in + u * ((1.35 - 0.03 * (1.0 - v)) - t_in)

    def _lchk_y(u, v):
        yl = -0.0062 + 0.0410 * (u ** 1.45)
        yh = 0.0544 + 0.0022 * math.exp(-((u / 0.14) ** 2)) + 0.0245 * (u ** 1.15)
        return yl + v * (yh - yl)

    lj_panel = _framed_panel(_lchk_th, _lchk_y, nu=18, nv=11, off=0.0005, wall=0.0021, rim_m=0.0028, pitch=0.0051, corner_r=0.0095, fill=0.52)
    # Matte-dark backing + rising inner vocalization baffle with titanium acoustic louver ribs.
    lj_back = shell_patch(_surf_grid(lambda u, v: _lchk_th(0.02 + 0.96 * u, 0.03 + 0.97 * v),
                                     lambda u, v: _lchk_y(0.02 + 0.96 * u, 0.03 + 0.97 * v) + 0.0085 * (v ** 3),
                                     10, 8, off=-0.0044, nose=False, sculpt=False),
                          0.0012, 0.00020, BAFFLE_DARK, rim_col=BAFFLE_DARK)
    lj_ribs = [shell_patch(_surf_grid(lambda u, v: _lchk_th(0.08 + 0.82 * u, 1.0),
                                      lambda u, v, dy=dy0: _lchk_y(0.08 + 0.82 * u, 1.0) + dy + 0.0013 * v,
                                      10, 3, off=-0.0031, nose=False, sculpt=False),
                           0.0009, 0.00018, TITANIUM, rim_col=PANEL_RIM) for dy0 in (0.0016, 0.0046)]
    sub_ch = np.array([[[(0.017 + 0.016 * float(v)) * float(u), -0.0082 + 0.013 * float(v),
                         0.091 - 0.025 * float(v) - 0.010 * (float(u) ** 2)]
                        for v in np.linspace(0.0, 1.0, 4)] for u in np.linspace(-1.0, 1.0, 7)])
    under_jaw = shell_patch(sub_ch, 0.0018, 0.00032, PANEL_RIM, (0.0, 0.055, 0.055), BAFFLE_DARK)
    ryz = csg.filleted2d([(0.030, 0.060), (0.044, 0.064), (0.068, 0.028), (0.068, 0.020), (0.060, 0.020), (0.031, 0.052)], r=0.0020)
    ram = (csg.extrude(ryz, a=0.0398, b=0.0434, axis=0, colour=CHASSIS_CF, chamfer=0.0006).tris()
           + csg.extrude(csg.circle2d(0.0036, fn=20, at=(0.066, 0.024)), a=0.0434, b=0.0468, axis=0, colour=TITANIUM, chamfer=0.0004).tris())
    yoke = csg.extrude(csg.filleted2d(csg.box2d((-0.024, 0.024), (0.014, 0.020)), r=0.0016), a=0.074, b=0.079, axis=2, colour=CHASSIS_CF, chamfer=0.0004).tris()
    lj_s = lj_panel + lj_back + lj_ribs[0] + lj_ribs[1]
    jaw_all = under_jaw + lj_s + mirror_x(lj_s) + yoke + ram + mirror_x(ram)
    ulid = shell_patch(_lid_grid(16.0, 39.5, 3.6), 0.0010, 0.00022, BAFFLE_DARK, piv, BAFFLE_DARK)
    llid = shell_patch(_lid_grid(-31.0, -18.5, 2.8), 0.0010, 0.00022, SHELL_CF, piv, SEAM_SILVER)
    return shell_patch(brow_g, 0.0010, 0.00024, BROW_DARK, rim_col=BAFFLE_DARK), ulid, llid, piv, shell_patch(comm_g, 0.0014, 0.00028, BAFFLE_DARK, rim_col=BAFFLE_DARK), jaw_all
