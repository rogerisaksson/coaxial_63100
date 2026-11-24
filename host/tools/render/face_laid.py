"""Rounded anime-mecha gynoid head panels, dark inner baffles, and micro-honeycomb vents."""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg, face_honeycomb as fh, knee_loads as kl, neck_laid as nl  # noqa: E402
from tools.render.face_honeycomb import (
    BAFFLE_DARK, BROW_DARK, CHASSIS_CF, PANEL_RIM, SEAM_SILVER, SHELL_CF, TITANIUM,
    mirror_x, np_mod, shell_patch,
)

_PROF_Y = (-0.0105, 0.004, 0.022, 0.042, 0.060, 0.076, 0.090, 0.104, 0.118, 0.138, 0.156)
_PROF_X = (0.0235, 0.0415, 0.0535, 0.0625, 0.0676, 0.0702, 0.0710, 0.0708, 0.0698, 0.0718, 0.0724)
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
    """Evaluate unified anime-feminine mecha head surface at azimuth `th_rad` and height `y`."""
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
        y_orb = 0.1002 + 0.0010 * (ath - 0.48)
        z -= 0.0004 * math.exp(-1.45 * (((ath - 0.48) / 0.26) ** 2 + ((y - y_orb) / 0.0165) ** 2))
        y_chk = 0.0630 + 0.0030 * min(1.0, max(0.0, (ath - 0.22) / 0.75))
        ck = math.exp(-((ath - 0.54) / 0.48) ** 2) * math.exp(-((y - y_chk) / 0.024) ** 2)
        x += math.copysign(0.0020 * ck, st) if abs(st) > 1e-6 else 0.0
        z += 0.0026 * ck
        y_br = 0.1145 + 0.0015 * min(1.0, max(0.0, (ath - 0.20) / 0.56))
        z += 0.0014 * math.exp(-((y - y_br) / 0.0070) ** 2) * math.exp(-((ath - 0.48) / 0.40) ** 2)
        mw = math.exp(-(ath / 0.22) ** 2)
        z += (0.0030 * mw * math.exp(-((y - 0.0448) / 0.0044) ** 2)
              + 0.0028 * mw * math.exp(-((y - 0.0352) / 0.0048) ** 2)
              - 0.0022 * math.exp(-(ath / 0.24) ** 2) * math.exp(-((y - 0.0225) / 0.0080) ** 2)
              + 0.0028 * math.exp(-(ath / 0.28) ** 2) * math.exp(-((y - 0.0020) / 0.0125) ** 2))

    if nose and ct > 0.48 and 0.066 <= y <= 0.107:
        tip = math.exp(-((y - 0.0725) / 0.0068) ** 2)
        br = 0.0016 * math.sin(math.pi * max(0.0, min(1.0, (y - 0.066) / 0.041)))
        z += (br + 0.0052 * tip) * max(0.0, 1.0 - (ath / (0.036 + 0.022 * tip)) ** 1.6)

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


def _planar_grid(grid, blend=0.28):
    np = np_mod()
    nu, nv = grid.shape[:2]
    us = np.linspace(0.0, 1.0, nu)[:, None, None]
    vs = np.linspace(0.0, 1.0, nv)[None, :, None]
    c00, c10, c11, c01 = grid[0, 0], grid[-1, 0], grid[-1, -1], grid[0, -1]
    bilin = (1.0 - us) * ((1.0 - vs) * c00 + vs * c01) + us * ((1.0 - vs) * c10 + vs * c11)
    return (1.0 - blend) * grid + blend * bilin


def _swept_panel(th_fn, y_fn, nu=12, nv=9, off=0.0, wall=0.0024, ch=0.00060, flat=0.06, nose=False, col=SHELL_CF, rim_col=SEAM_SILVER):
    g = _surf_grid(th_fn, y_fn, nu, nv, off=off, nose=nose)
    return shell_patch(_planar_grid(g, flat) if flat > 0.0 else g, wall, ch, col, rim_col=rim_col)


def _eye_top(th):
    t = max(0.0, min(1.0, (th - 0.275) / 0.395))
    return 0.0994 + 0.0018 * t + 0.0086 * math.sin(math.pi * (t ** 0.88))


def _eye_bot(th):
    t = max(0.0, min(1.0, (th - 0.275) / 0.395))
    return 0.0984 + 0.0016 * t - 0.0064 * math.sin(math.pi * (t ** 0.90))


def _crown_seam_y(th):
    return 0.1560 + 0.0200 * (min(1.0, abs(th) / 0.92) ** 1.15)


def _ear_clear_y(th, y_base):
    p = _head_pt(th, y_base, 0.0, False, False)
    dz = p[2] - 0.004
    if abs(dz) < 0.0268:
        y_arc = 0.093 + math.sqrt(max(0.0, 0.0268 ** 2 - dz * dz))
        return 0.5 * (y_base + y_arc + math.hypot(y_base - y_arc, 0.0036))
    return y_base


def _dome_pt(th, phi, off=0.0):
    p = _head_pt(th, 0.156 + 0.0704 * math.sin(phi), off, False, False)
    return (max(0.0008, p[0]), p[1], p[2])


def _dome_strip(u0, u1, v0, v1, nu=14, nv=7, off=0.0):
    np = np_mod()
    us, vs = np.linspace(u0, u1, nu), np.linspace(v0, v1, nv)
    g = np.zeros((nu, nv, 3))
    for i, u in enumerate(us):
        th = 0.011 + float(u) * (math.pi - 0.022)
        y0 = max(0.157, _crown_seam_y(th) + 0.0014)
        phi0 = math.asin(max(0.02, min(0.95, (y0 - 0.156) / 0.0704)))
        phi1 = 0.5 * math.pi - 0.022
        for j, v in enumerate(vs):
            g[i, j] = _dome_pt(th, phi0 + float(v) * (phi1 - phi0), off)
    return g


@functools.lru_cache(maxsize=None)
def chassis_static():
    """Build internal carbon chassis, dark inner baffle sub-shells, orbital collars and ear struts."""
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
    wl, wr = (csg.extrude(wyz, a=a, b=b, axis=0, colour=CHASSIS_CF, chamfer=0.0006) for a, b in ((0.029, 0.035), (-0.035, -0.029)))
    ports = [csg.filleted2d(csg.box2d(u, v), r=0.0042)
             for u, v in (((-0.045, -0.017), (0.087, 0.114)), ((0.017, 0.045), (0.087, 0.114)))]
    eye_m = csg.extrude(csg.filleted2d(csg.box2d((-0.048, 0.048), (0.083, 0.118)), r=0.005),
                        holes=ports, a=0.058, b=0.068, axis=2, colour=BAFFLE_DARK, chamfer=0.0005)
    glab_b = csg.extrude(csg.filleted2d(csg.box2d((-0.0130, 0.0130), (0.088, 0.108)), r=0.002),
                         a=0.072, b=0.086, axis=2, colour=BAFFLE_DARK, chamfer=0.0004).tris()
    cowl_o = [((0.032 + 0.0176 * math.cos(a)), (0.100 + 0.0168 * math.sin(a))) for a in (2.0 * math.pi * k / 28 for k in range(28))]
    cowl_i = [((0.032 + 0.0150 * math.cos(a)), (0.100 + 0.0148 * math.sin(a))) for a in (2.0 * math.pi * k / 28 for k in range(28))]
    cowl_l = csg.extrude(cowl_o, holes=[cowl_i], a=0.068, b=0.079, axis=2, colour=BAFFLE_DARK, chamfer=0.0004).tris()
    sub_cr = shell_patch(_surf_grid(lambda u, v: 0.155 * (2.0 * v - 1.0), lambda u, v: 0.130 + u * 0.032, 6, 7, off=-0.0032, nose=False, sculpt=False),
                         0.0012, 0.00022, BAFFLE_DARK, rim_col=BAFFLE_DARK)
    liner = list(sub_cr)
    for t0, ts, y0, ys, nu, nv, of in (
        (0.005, 2.27, 0.116, 0.064, 8, 12, -0.0036), (0.80, 0.54, 0.088, 0.034, 6, 7, -0.0042),
        (0.10, 1.44, 0.032, 0.051, 6, 10, -0.0046), (2.02, math.pi - 2.05, 0.018, 0.160, 8, 8, -0.0038),
        (1.18, 0.82, 0.028, 0.090, 8, 7, -0.0048),
    ):
        s_p = shell_patch(_surf_grid(lambda u, v, a=t0, b=ts: a + v * b, lambda u, v, c=y0, d=ys: c + u * d,
                                     nu, nv, off=of, nose=False, sculpt=False), 0.0014, 0.00025, BAFFLE_DARK, rim_col=BAFFLE_DARK)
        liner += s_p + mirror_x(s_p)
    s_d = shell_patch(_dome_strip(0.00, 0.99, 0.04, 0.94, nu=12, nv=6, off=-0.0040), 0.0014, 0.00025, BAFFLE_DARK, rim_col=BAFFLE_DARK)
    liner += s_d + mirror_x(s_d)
    oral_g = np.array([[[0.030 * float(u), 0.008 + 0.058 * float(v), 0.079 - 0.016 * float(u * u) - 0.004 * (1.0 - float(v))]
                        for v in np.linspace(0.0, 1.0, 5)] for u in np.linspace(-1.0, 1.0, 11)])
    oral = shell_patch(oral_g, 0.0020, 0.00032, BAFFLE_DARK, (0.0, 0.042, 0.010), BAFFLE_DARK)
    ear = csg.sweep(csg.circle2d(0.0042, fn=18), [(0.040, 0.093, 0.026), (0.052, 0.093, 0.012), (0.061, 0.093, 0.004)],
                    chamfer_ends=0.0006, colour=CHASSIS_CF).tris()
    return csg.union(flange, bulk, fore, deck, wl, wr, eye_m).tris() + glab_b + cowl_l + mirror_x(cowl_l) + oral + liner + ear + mirror_x(ear)


@functools.lru_cache(maxsize=None)
def compute_heatsink():
    """Return recessed cranial compute fins and heatpipes inside the cranial bay."""
    return fh.compute_heatsink()


@functools.lru_cache(maxsize=None)
def static_plates():
    """Build diagonal sculptural anime-mecha panels with uniform airflow gaps and fine honeycomb vents."""
    def _fr_y_lo(th):
        ath = abs(th)
        return (_eye_top(ath) + 0.0022) if 0.275 <= ath <= 0.670 else (
            (0.1060 - 0.0024 * ((1.0 - ath / 0.275) ** 1.3)) if ath < 0.275 else (0.1030 + 0.006 * ((ath - 0.670) / 0.10)))

    def _fr_mid_y(th):
        return 0.1325 + 0.0115 * (min(1.0, abs(th) / 0.80) ** 1.25)

    f_key = _swept_panel(
        lambda u, v: (0.335 + 0.065 * u) * (2.0 * v - 1.0),
        lambda u, v: _fr_y_lo((0.335 + 0.065 * u) * (2.0 * v - 1.0)) + u * (
            _fr_mid_y((0.335 + 0.065 * u) * (2.0 * v - 1.0)) - 0.0008 - _fr_y_lo((0.335 + 0.065 * u) * (2.0 * v - 1.0))),
        nu=9, nv=13, off=0.0012, wall=0.0018, flat=0.04)
    f_wing = _swept_panel(
        lambda u, v: (0.354 + 0.064 * u) + v * ((0.706 + 0.094 * u) - (0.354 + 0.064 * u)),
        lambda u, v: _fr_y_lo(0.354 + v * 0.352) + u * (_fr_mid_y(0.418 + v * 0.382) - 0.0008 - _fr_y_lo(0.354 + v * 0.352)),
        nu=9, nv=9, off=0.0012, wall=0.0018, flat=0.05)
    f_hi = _swept_panel(
        lambda u, v: (0.088 + 0.064 * u) + v * ((0.818 + 0.022 * u) - (0.088 + 0.064 * u)),
        lambda u, v: (_fr_mid_y(0.088 + v * 0.730) + 0.0010) + u * (
            _crown_seam_y(0.152 + v * 0.688) - 0.0012 - (_fr_mid_y(0.088 + v * 0.730) + 0.0010)),
        nu=10, nv=11, off=0.0002, flat=0.04)
    cr_g = _surf_grid(lambda u, v: (0.068 + 0.060 * u) * (2.0 * v - 1.0), lambda u, v: 0.1338 + u * 0.0206, 8, 8, off=-0.0006, nose=False)
    front = f_key + f_wing + mirror_x(f_wing) + f_hi + mirror_x(f_hi) + fh.honeycomb_patch(
        cr_g, pitch=0.0034, rim_m=0.0019, wall=0.0020, fill=0.72, col=PANEL_RIM, rim_col=SEAM_SILVER)

    d_low = shell_patch(_dome_strip(0.00, 1.00, 0.00, 0.45, nu=22, nv=8, off=0.0003), 0.0024, 0.00056, SHELL_CF)
    d_vent = fh.honeycomb_patch(_dome_strip(0.04, 0.94, 0.478, 0.684, nu=16, nv=6, off=-0.0006),
                                pitch=0.0044, rim_m=0.0024, wall=0.0021, fill=0.74)
    d_apex = shell_patch(_dome_strip(0.00, 1.00, 0.712, 1.00, nu=22, nv=7, off=0.0003), 0.0024, 0.00056, SHELL_CF)
    d_lobe = d_low + d_vent + d_apex

    o_top = _swept_panel(lambda u, v: 2.28 + v * (math.pi - 2.30),
                         lambda u, v: 0.106 + u * (_crown_seam_y(2.28 + v * 0.84) - 0.1072), nu=10, nv=9, off=0.0003, flat=0.04)
    o_vent = fh.honeycomb_patch(_surf_grid(lambda u, v: 2.31 + v * (math.pi - 2.346),
                                           lambda u, v: 0.015 + u * 0.087, 14, 7, off=-0.0006, nose=False),
                                pitch=0.0046, rim_m=0.0025, wall=0.0021, fill=0.74)
    o_lobe = o_top + o_vent

    def _tmp_th(u, v):
        t0 = 0.835 + 0.035 * v
        return t0 + u * (2.25 - t0)

    def _tmp_y(u, v):
        th = _tmp_th(u, v)
        y_lo = _ear_clear_y(th, 0.1188 - 0.005 * u)
        return y_lo + v * (_crown_seam_y(th) - 0.0012 - y_lo)

    t_front = _swept_panel(lambda u, v: _tmp_th(0.46 * u, v), lambda u, v: _tmp_y(0.46 * u, v), nu=11, nv=9, off=0.0003, flat=0.05)
    t_vent = fh.honeycomb_patch(_surf_grid(lambda u, v: _tmp_th(0.490 + 0.350 * u, v),
                                           lambda u, v: _tmp_y(0.490 + 0.350 * u, v), 11, 7, off=-0.0006, nose=False),
                                pitch=0.0044, rim_m=0.0024, wall=0.0021, fill=0.74)
    t_rear = _swept_panel(lambda u, v: _tmp_th(0.865 + 0.135 * u, v), lambda u, v: _tmp_y(0.865 + 0.135 * u, v), nu=7, nv=8, off=0.0003, flat=0.05)
    l_temp = t_front + t_vent + t_rear

    l_mast = fh.honeycomb_patch(_surf_grid(lambda u, v: 1.98 + u * 0.27,
                                           lambda u, v: 0.015 + v * (_ear_clear_y(1.98 + u * 0.27, 0.102) - 0.0198),
                                           11, 7, off=-0.0005, nose=False),
                                pitch=0.0044, rim_m=0.0024, wall=0.0021, fill=0.74)
    l_pil = _swept_panel(lambda u, v: (0.692 + 0.088 * u) + v * (1.27 - (0.692 + 0.088 * u)),
                         lambda u, v: 0.0982 + u * 0.0174, nu=9, nv=13, off=0.0003, flat=0.05)
    l_nose = _swept_panel(lambda u, v: (0.094 + 0.014 * math.sin(math.pi * (u ** 0.80)) + 0.066 * (u ** 1.65)) * (2.0 * v - 1.0),
                          lambda u, v: 0.0665 + u * 0.0353, nu=18, nv=11, off=0.0003, flat=0.04, nose=True)

    def _chk_split_y(u):
        return 0.0615 + 0.0245 * (u ** 1.08)

    def _cu_th(u, v):
        t_in = 0.116 + 0.068 * (v ** 1.50)
        return t_in + u * ((1.24 + 0.08 * (1.0 - v)) - t_in)

    def _cu_y(u, v):
        th = _cu_th(u, v)
        yh = (_eye_bot(th) - 0.0008) if 0.275 <= th <= 0.670 else (
            (0.0985 - 0.005 * ((0.275 - th) / 0.12)) if th < 0.275 else (0.0930 - 0.0022 * ((th - 0.670) / 0.60)))
        yl = _chk_split_y(u) + 0.0007
        return yl + v * (yh - yl)

    c_upper = _swept_panel(_cu_th, _cu_y, nu=18, nv=8, off=0.0007, wall=0.0020, flat=0.05)

    def _cl_th(u, v):
        p = min(1.0, max(0.0, (v - 0.24) / 0.48))
        t_in = 0.272 - 0.140 * (p * p * (3.0 - 2.0 * p))
        return t_in + u * ((1.14 + 0.12 * v) - t_in)

    c_lower = _swept_panel(_cl_th, lambda u, v: (0.0454 + 0.0220 * (u ** 1.15)) + v * (
        _chk_split_y(u) - 0.0007 - (0.0454 + 0.0220 * (u ** 1.15))), nu=16, nv=8, off=0.0002, flat=0.05)
    c_gill = fh.honeycomb_patch(_surf_grid(lambda u, v: (1.165 + 0.095 * v) + u * ((1.335 - 0.025 * v) - (1.165 + 0.095 * v)),
                                           lambda u, v: (0.0620 + 0.0045 * u) + v * ((0.0825 + 0.0010 * u) - (0.0620 + 0.0045 * u)),
                                           8, 5, off=-0.0008, nose=False),
                                pitch=0.0038, rim_m=0.0020, wall=0.0020, fill=0.73, col=PANEL_RIM, rim_col=BAFFLE_DARK)
    ph_p = _swept_panel(lambda u, v: (0.128 - 0.028 * v) * (2.0 * u - 1.0), lambda u, v: 0.0502 + v * 0.0142, nu=12, nv=7, off=0.0002, flat=0.03)
    lip_p = _swept_panel(lambda u, v: 0.208 * (2.0 * u - 1.0),
                         lambda u, v: 0.0414 + v * 0.0070 + 0.0006 * ((2.0 * u - 1.0) ** 2) - 0.0009 * v * math.exp(-((2.0 * u - 1.0) / 0.24) ** 2),
                         nu=17, nv=7, off=0.0003, flat=0.03)
    l_max = c_upper + c_lower + c_gill
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
    """Slice arched anime brow, eyelids, lip corner and diagonal V-tapered mandible."""
    from machine.eyes import gimbal, stereo
    np, ex = np_mod(), 0.5 * float(stereo.BASELINE_M)
    brow_g = _surf_grid(
        lambda u, v: 0.250 + u * 0.445,
        lambda u, v: (0.1134 + 0.0030 * math.sin(math.pi * (u ** 0.85)) + 0.0026 * (u ** 0.90)) + v * (0.0022 * (1.0 - 0.45 * (u ** 1.4))),
        13, 4, off=0.0026, nose=False)
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

    comm_g = _surf_grid(lambda u, v: 0.228 + u * 0.028, lambda u, v: 0.0416 + 0.0006 * u + v * 0.0040, 6, 4, off=0.0002, nose=False)
    chin_lip = _swept_panel(lambda u, v: (0.188 + 0.064 * v) * (2.0 * u - 1.0),
                            lambda u, v: 0.0175 + v * 0.0227 + 0.0006 * ((2.0 * u - 1.0) ** 2) * v, nu=17, nv=7, off=0.0003, flat=0.04)
    chin_cap = _swept_panel(lambda u, v: (0.142 + 0.046 * v) * (2.0 * u - 1.0),
                            lambda u, v: -0.0105 + v * 0.0267 + 0.0010 * ((2.0 * u - 1.0) ** 2) * (1.0 - v), nu=15, nv=8, off=0.0004, flat=0.04)
    sub_ch = np.array([[[(0.018 + 0.018 * float(v)) * float(u), -0.0102 + 0.013 * float(v),
                         0.091 - 0.025 * float(v) - 0.010 * (float(u) ** 2)]
                        for v in np.linspace(0.0, 1.0, 4)] for u in np.linspace(-1.0, 1.0, 7)])
    under_jaw = shell_patch(sub_ch, 0.0018, 0.00032, PANEL_RIM, (0.0, 0.055, 0.055), BAFFLE_DARK)

    def _jaw_th(u, v):
        th_in = 0.158 + 0.112 * v
        return th_in + u * (1.36 - th_in)

    def _jaw_y(u, v):
        yl = -0.0080 + 0.0365 * (u ** 1.35)
        return yl + v * ((0.0418 + 0.0220 * (u ** 1.15)) - yl)

    lj_front = _swept_panel(lambda u, v: _jaw_th(0.70 * u, v), lambda u, v: _jaw_y(0.70 * u, v), nu=15, nv=9, off=0.0003, flat=0.05)
    lj_gill = fh.honeycomb_patch(_surf_grid(lambda u, v: _jaw_th(0.730 + 0.270 * u, v),
                                            lambda u, v: _jaw_y(0.730 + 0.270 * u, v), 9, 6, off=-0.0008, nose=False),
                                 pitch=0.0038, rim_m=0.0020, wall=0.0020, fill=0.73, col=PANEL_RIM, rim_col=BAFFLE_DARK)
    lj_s = lj_front + lj_gill
    ryz = csg.filleted2d([(0.030, 0.060), (0.044, 0.064), (0.068, 0.028), (0.068, 0.020), (0.060, 0.020), (0.031, 0.052)], r=0.0020)
    ram = (csg.extrude(ryz, a=0.0398, b=0.0434, axis=0, colour=CHASSIS_CF, chamfer=0.0006).tris()
           + csg.extrude(csg.circle2d(0.0036, fn=20, at=(0.066, 0.024)), a=0.0434, b=0.0468, axis=0, colour=TITANIUM, chamfer=0.0004).tris())
    yoke = csg.extrude(csg.filleted2d(csg.box2d((-0.024, 0.024), (0.014, 0.020)), r=0.0016), a=0.074, b=0.079, axis=2, colour=CHASSIS_CF, chamfer=0.0004).tris()
    jaw_all = chin_lip + chin_cap + under_jaw + lj_s + mirror_x(lj_s) + yoke + ram + mirror_x(ram)
    ulid = shell_patch(_lid_grid(16.5, 31.5, 2.2), 0.0010, 0.00022, BAFFLE_DARK, piv, BAFFLE_DARK)
    llid = shell_patch(_lid_grid(-30.0, -17.5, 1.8), 0.0010, 0.00022, SHELL_CF, piv, SEAM_SILVER)
    return shell_patch(brow_g, 0.0010, 0.00024, BROW_DARK, rim_col=BAFFLE_DARK), ulid, llid, piv, shell_patch(comm_g, 0.0018, 0.00038, SHELL_CF), jaw_all
