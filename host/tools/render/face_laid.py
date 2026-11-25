"""Mechatronic anime-gynoid head CAD: unified valkyrie helmet, porcelain face, flat wings."""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg, face_honeycomb as fh, knee_loads as kl, neck_laid as nl  # noqa: E402
from tools.render.face_honeycomb import (
    BAFFLE_DARK, BROW_DARK, CHASSIS_CF, HEX_WALL, MESH_PEARL, MIC_GOLD, OPTIC_GLASS, PANEL_RIM,
    PORCELAIN, SEAM_SILVER, SHELL_CF, TITANIUM, mirror_x, np_mod, piston, rotary_hub, shell_patch,
)

_PROF_Y = (0.012, 0.022, 0.038, 0.056, 0.076, 0.096, 0.118, 0.140, 0.158)
_PROF_X = (0.0170, 0.0250, 0.0432, 0.0565, 0.0660, 0.0692, 0.0694, 0.0706, 0.0714)
_PROF_ZF = (0.0878, 0.0938, 0.1005, 0.1055, 0.1092, 0.1096, 0.1078, 0.1008, 0.0920)
_PROF_ZB = (-0.047, -0.053, -0.062, -0.070, -0.077, -0.082, -0.086, -0.090, -0.093)


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
        lo, hi = (mid, hi) if xs[mid] <= y else (lo, mid)
    t = (y - xs[lo]) / (xs[hi] - xs[lo])
    s = t * t * (3.0 - 2.0 * t)
    return 0.55 * (ys[lo] + t * (ys[hi] - ys[lo])) + 0.45 * (ys[lo] + s * (ys[hi] - ys[lo]))


def _profile_at(y):
    if y >= 0.158:
        u = min(1.0, max(0.0, (y - 0.158) / 0.0695))
        c = math.sqrt(max(1e-6, 1.0 - u * u))
        return 0.0714 * c, -0.0005 + 0.0925 * c, -0.0005 - 0.0925 * c
    return _pchip(y, _PROF_Y, _PROF_X), _pchip(y, _PROF_Y, _PROF_ZF), _pchip(y, _PROF_Y, _PROF_ZB)


def _head_pt(th_rad, y, radial_off=0.0):
    """Evaluate sleek anime-gynoid head anchor point at azimuth `th_rad` and height `y`."""
    xw, zf, zb = _profile_at(y)
    zc, rz = 0.5 * (zf + zb), max(0.004, 0.5 * (zf - zb))
    x, z = xw * math.sin(th_rad), zc + rz * math.cos(th_rad)
    if radial_off != 0.0:
        r_xz = math.hypot(x, z - zc)
        if r_xz > 1e-6:
            x, z = x + radial_off * (x / r_xz), z + radial_off * ((z - zc) / r_xz)
    return (x, y, z)


def _clear_mount(pt, r_min=0.0462):
    """Keep cervical and occipital shells smoothly outside `mount_crown` cylinder."""
    x, y, z = pt
    if y >= 0.064:
        return pt
    t = max(0.0, min(1.0, (0.064 - y) / 0.011))
    req = float(r_min) * (t * t * (3.0 - 2.0 * t))
    r = math.hypot(x, z)
    return (x * req / r, y, z * req / r) if (r < req and r > 1e-6) else pt


def _surf_grid(th_fn, y_fn, nu, nv, off=0.0, mount_r=0.0):
    np = np_mod()
    us, vs = np.linspace(0.0, 1.0, nu), np.linspace(0.0, 1.0, nv)
    g = np.zeros((nu, nv, 3))
    for i, u in enumerate(us):
        fu = float(u)
        for j, v in enumerate(vs):
            pt = _head_pt(float(th_fn(fu, float(v))), float(y_fn(fu, float(v))), off)
            g[i, j] = _clear_mount(pt, mount_r) if mount_r > 0.0 else pt
    return g


def _dome_ab(alpha, beta, off=0.0):
    sx, sy, sz = math.sin(beta), math.cos(beta) * math.cos(alpha), math.cos(beta) * math.sin(alpha)
    x, y, z = 0.0714 * sx, 0.158 + 0.0695 * sy, -0.0005 + 0.0925 * sz
    if off != 0.0:
        n = math.sqrt((x / 0.0714) ** 2 + (sy ** 2) + ((z + 0.0005) / 0.0925) ** 2)
        if n > 1e-6:
            x, y, z = x + off * sx / n, y + off * sy / n, z + off * sz / n
    return (max(0.0005, x), y, z)


def _crown_seam_y(th):
    """Dynamic swept aerodynamic helmet seam flowing from frontal temple down behind the ear-pod."""
    t = min(1.0, max(0.0, (abs(th) - 0.72) / 2.42))
    return 0.1840 - 0.0270 * (t * t * (3.0 - 2.0 * t))


def _dome_strip(u0, u1, v0, v1, nu=28, nv=10, off=0.0, a_f=1.00, a_b=-1.48, b_min=0.004, pad=0.0016):
    np = np_mod()
    us, vs = np.linspace(u0, u1, nu), np.linspace(v0, v1, nv)
    g = np.zeros((nu, nv, 3))
    for i, u in enumerate(us):
        al, lo, hi = a_f + float(u) * (a_b - a_f), b_min + 0.06, 1.54
        for _ in range(12):
            mid = 0.5 * (lo + hi)
            px, py, pz = _dome_ab(al, mid, 0.0)
            lo, hi = (mid, hi) if py > _crown_seam_y(math.atan2(px, pz + 0.0005)) + pad else (lo, mid)
        b_max = max(b_min + 0.12, lo)
        for j, v in enumerate(vs):
            g[i, j] = _dome_ab(al, b_min + float(v) * (b_max - b_min), off)
    return g


def _flat(pts, wall=0.0021, rim_m=0.0048, pitch=0.0078, fill=0.61, corner_c=0.0018, shift_n=0.0008, bar_col=MESH_PEARL):
    """Build a 100% flat unsculpted angular pearl-ceramic framed coarse-honeycomb armor wing."""
    return fh.planar_honeycomb_panel(pts, wall=wall, chamfer=0.00072, rim_m=rim_m, pitch=pitch, fill=fill,
                                     corner_c=corner_c, col=SHELL_CF, bar_col=bar_col, hole_col=HEX_WALL,
                                     rim_col=SEAM_SILVER, shift_n=shift_n)


def _porcelain_anime_face():
    """Build ONE continuous C1-smooth porcelain anime face with deep 3/4 lateral orbit and dark nasal channel."""
    np = np_mod()

    def _z_face(x, y, z_off=0.0):
        xw, zf, zb = _profile_at(y)
        zc, rz = 0.5 * (zf + zb), max(0.004, 0.5 * (zf - zb))
        ax = abs(x)
        st = max(-0.96, min(0.96, x / max(xw, 1e-4)))
        z0 = zc + rz * math.sqrt(max(0.04, 1.0 - st * st))
        eye_recess = -0.0030 * math.exp(-((ax - 0.032) / 0.011) ** 2) * math.exp(-((y - 0.100) / 0.011) ** 2)
        lat_rim = 0.0038 * math.exp(-((ax - 0.052) / 0.015) ** 2) * math.exp(-((y - 0.104) / 0.020) ** 2)
        w_lat = max(0.0, min(1.0, (ax - 0.011) / 0.015))
        t_y = max(0.0, min(1.0, (0.0945 - y) / 0.010))
        tuck = -(0.0032 + 0.0068 * w_lat) * (t_y ** 1.10) - 0.0036 * max(0.0, min(1.0, (y - 0.129) / 0.012))
        return z0 - 0.0014 + eye_recess + lat_rim + tuck + z_off

    def _eye_y(ax, top):
        if ax <= 0.0216:
            return 0.0982 + (0.00005 if top else 0.0)
        if ax >= 0.0410:
            t_out = min(1.0, (ax - 0.0410) / 0.018)
            return 0.1016 - 0.0030 * t_out + (0.00005 if top else 0.0)
        s = (ax - 0.0313) / 0.0097
        k = max(0.0, 1.0 - s * s)
        return 0.0998 + 0.0018 * s + (0.0072 * (k ** 0.68) if top else -0.0046 * (k ** 0.74))

    x_pos = np.concatenate([np.linspace(0.0, 0.0216, 7), np.linspace(0.0216, 0.0410, 15)[1:], np.linspace(0.0410, 0.0600, 7)[1:]])
    xs, vs = np.concatenate([-x_pos[::-1][:-1], x_pos]), np.linspace(0.0, 1.0, 11)
    g_up, g_lo = np.zeros((len(xs), len(vs), 3)), np.zeros((len(xs), len(vs), 3))
    for i, xv in enumerate(xs):
        x, ax = float(xv), abs(float(xv))
        u_w = min(1.0, ax / 0.0600)
        y_u0, y_l0 = _eye_y(ax, True), _eye_y(ax, False)
        y_u1 = 0.1305 + 0.0125 * (u_w ** 1.10)
        y_l1 = 0.0280 + 0.0590 * (u_w ** 1.28)
        for j, fv in enumerate(vs):
            v = float(fv)
            yu, yl = y_u0 + v * (y_u1 - y_u0), y_l0 + v * (y_l1 - y_l0)
            zt = -0.0010 * (u_w ** 2) * v
            g_up[i, j] = (x, yu, _z_face(x, yu, zt))
            g_lo[i, j] = (x, yl, _z_face(x, yl, -0.00006 + zt))
    ss_l = np.linspace(-0.96, 1.08, 19)
    g_lu = np.zeros((len(ss_l), 3, 3))
    for i, sv in enumerate(ss_l):
        s, sc = float(sv), max(-1.0, min(1.0, float(sv)))
        xi = 0.0313 + 0.0097 * sc
        yi = _eye_y(xi, True)
        if s > 1.0:
            t_w = (s - 1.0) / 0.08
            xi, yi, w_l = xi + 0.0020 * t_w, yi + 0.0016 * t_w, 0.0022 * (1.0 - 0.84 * t_w)
        else:
            w_l = 0.0010 + 0.0013 * max(0.0, 0.5 * (s + 1.0)) ** 1.4
        for j, fv in enumerate((0.0, 0.5, 1.0)):
            x, y = xi, yi + fv * w_l
            g_lu[i, j] = (x, y, _z_face(x, y, 0.0008))
    g_ll = np.zeros((13, 3, 3))
    for i, sv in enumerate(np.linspace(-0.88, 0.96, 13)):
        xi = 0.0313 + 0.0097 * float(sv)
        yi = _eye_y(xi, False)
        for j, fv in enumerate((0.0, 0.5, 1.0)):
            x, y = xi, yi - fv * (0.0007 + 0.0005 * max(0.0, float(sv)))
            g_ll[i, j] = (x, y, _z_face(x, y, 0.0007))
    g_nc = np.zeros((8, 5, 3))
    for i, fu in enumerate(np.linspace(0.0, 1.0, 8)):
        y = 0.0755 + 0.0305 * float(fu)
        hw = 0.0036 + 0.0022 * math.sin(math.pi * float(fu))
        for j, fv in enumerate(np.linspace(-1.0, 1.0, 5)):
            x = hw * float(fv)
            g_nc[i, j] = (x, y, _z_face(x, y, 0.0009))
    lashes = (shell_patch(g_lu, 0.0008, 0.00018, BAFFLE_DARK, rim_col=BAFFLE_DARK)
              + shell_patch(g_ll, 0.0007, 0.00016, PANEL_RIM, rim_col=BAFFLE_DARK))
    return (shell_patch(g_up, 0.0011, 0.00008, PORCELAIN, rim_col=PORCELAIN)
            + shell_patch(g_lo, 0.0011, 0.00008, PORCELAIN, rim_col=PORCELAIN)
            + shell_patch(g_nc, 0.0009, 0.00018, CHASSIS_CF, rim_col=TITANIUM)
            + lashes + mirror_x(lashes))


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
    ear = csg.sweep(csg.circle2d(0.0042, fn=18), [(0.040, 0.095, 0.026), (0.052, 0.095, 0.012), (0.061, 0.095, 0.002)],
                    chamfer_ends=0.0006, colour=CHASSIS_CF).tris()
    return csg.union(flange, bulk, fore, deck, wl, wr, eye_m).tris() + ear + mirror_x(ear)


@functools.lru_cache(maxsize=None)
def inner_liner():
    """Return seamless porcelain anime face, titanium brow sub-plate, cyan jewel, throat fairing and mechanics."""
    np, p = np_mod(), _head_pt
    face_core = _porcelain_anime_face()
    brow_sub = shell_patch(_surf_grid(lambda u, v: 0.02 + v * 0.82, lambda u, v: 0.128 + u * 0.062,
                                      9, 10, off=-0.0086), 0.0012, 0.00024, TITANIUM, rim_col=PANEL_RIM)
    spine_f = shell_patch(_surf_grid(lambda u, v: (0.034 + 0.030 * math.sin(math.pi * min(1.0, 0.12 + 0.78 * u))) * (2.0 * v - 1.0),
                                     lambda u, v: 0.130 + u * 0.056, 12, 5, off=-0.0016), 0.0013, 0.00024, CHASSIS_CF, rim_col=TITANIUM)
    spine_led = shell_patch(_surf_grid(lambda u, v: 0.012 * (2.0 * v - 1.0),
                                       lambda u, v: 0.140 + u * 0.034, 6, 3, off=-0.0006), 0.0008, 0.00018, OPTIC_GLASS, rim_col=TITANIUM)
    spine_d = shell_patch(_dome_strip(0.01, 0.98, 0.0, 1.0, nu=18, nv=3, off=-0.0012, b_min=0.0005, pad=0.0016),
                          0.0011, 0.00020, CHASSIS_CF, rim_col=OPTIC_GLASS)
    liner = []
    for t0, ts, y0, ys, nu, nv, of, mr in (
        (0.00, 2.28, 0.130, 0.056, 8, 12, -0.0102, 0.0),
        (0.88, 0.50, 0.086, 0.046, 6, 6, -0.0086, 0.0),
        (0.00, 1.24, 0.052, 0.032, 6, 10, -0.0094, 0.0462),
        (1.94, math.pi - 1.96, 0.034, 0.132, 9, 8, -0.0044, 0.0462),
        (1.26, 0.76, 0.042, 0.094, 8, 7, -0.0056, 0.0462),
    ):
        s_p = shell_patch(_surf_grid(lambda u, v, a=t0, b=ts: a + v * b, lambda u, v, c=y0, d=ys: c + u * d,
                                     nu, nv, off=of, mount_r=mr), 0.0012, 0.00022, CHASSIS_CF, rim_col=PANEL_RIM)
        liner += s_p + mirror_x(s_p)
    s_d = shell_patch(_dome_strip(0.00, 0.99, 0.00, 0.98, nu=12, nv=6, off=-0.0072, b_min=0.003, pad=0.0014),
                      0.0012, 0.00022, CHASSIS_CF, rim_col=PANEL_RIM)
    throat_l = shell_patch(_surf_grid(lambda u, v: 0.78 + v * (math.pi - 0.80),
                                      lambda u, v: (0.018 + 0.010 * v) + u * (0.030 - 0.004 * v),
                                      7, 12, off=-0.0016, mount_r=0.0464), 0.0013, 0.00024, CHASSIS_CF, rim_col=PANEL_RIM)
    hub = rotary_hub(0.0626, 0.088, 0.032)
    pistons = (
        piston(p(0.96, 0.126, -0.0038), (0.063, 0.096, 0.030), 0.0015, 0.0023)
        + piston(p(1.42, 0.122, -0.0038), (0.063, 0.094, 0.026), 0.0015, 0.0022)
        + piston(p(0.68, 0.064, -0.0048), (0.061, 0.082, 0.030), 0.0015, 0.0023)
        + piston(p(0.90, 0.056, -0.0046), (0.062, 0.080, 0.028), 0.0014, 0.0021)
    )
    chin_bar = shell_patch(_surf_grid(lambda u, v: 0.036 * (2.0 * v - 1.0), lambda u, v: 0.024 + u * 0.032,
                                      8, 4, off=-0.0056), 0.0012, 0.00024, TITANIUM, rim_col=PANEL_RIM)
    chin_accent = shell_patch(_surf_grid(lambda u, v: 0.014 * (2.0 * v - 1.0), lambda u, v: 0.029 + u * 0.020,
                                         6, 3, off=-0.0046), 0.0008, 0.00018, MIC_GOLD, rim_col=TITANIUM)
    oral_g = np.array([[[0.022 * float(u), 0.024 + 0.038 * float(v), 0.072 - 0.012 * float(u * u) - 0.003 * (1.0 - float(v))]
                        for v in np.linspace(0.0, 1.0, 5)] for u in np.linspace(-1.0, 1.0, 9)])
    oral = shell_patch(oral_g, 0.0015, 0.00026, CHASSIS_CF, (0.0, 0.044, 0.010), PANEL_RIM)
    return (face_core + brow_sub + mirror_x(brow_sub) + spine_f + spine_led + spine_d + mirror_x(spine_d) + oral
            + liner + s_d + mirror_x(s_d) + throat_l + mirror_x(throat_l) + chin_bar + chin_accent
            + hub + mirror_x(hub) + pistons + mirror_x(pistons))


@functools.lru_cache(maxsize=None)
def compute_heatsink():
    """Return recessed cranial compute fins and heatpipes inside the cranial bay."""
    return fh.compute_heatsink()


@functools.lru_cache(maxsize=None)
def static_plates():
    """Build streamlined valkyrie helmet cowls and deep sculpted 2-row flat ceramic-honeycomb face wings."""
    p, seam = _head_pt, _crown_seam_y
    f_lo = _flat([p(0.095, 0.1285), p(0.75, 0.1445), p(0.79, 0.1635), p(0.112, 0.1495)],
                 wall=0.0021, rim_m=0.0046, pitch=0.0076, fill=0.61, corner_c=0.0018, shift_n=0.0003)
    f_hi = _flat([p(0.112, 0.1605), p(0.79, 0.1730), p(0.81, 0.1855), p(0.095, 0.1795)],
                 wall=0.0021, rim_m=0.0048, pitch=0.0078, fill=0.61, corner_c=0.0018, shift_n=-0.0002)
    front = f_lo + mirror_x(f_lo) + f_hi + mirror_x(f_hi)
    d_lobe = shell_patch(_dome_strip(0.00, 1.00, 0.00, 1.00, nu=28, nv=10, off=0.0014, a_f=1.00, a_b=-1.48, b_min=0.004, pad=0.0016),
                         wall=0.0023, chamfer=0.00056, col=SHELL_CF, rim_col=SEAM_SILVER)
    o_lobe = shell_patch(_surf_grid(lambda u, v: 2.24 + v * (math.pi - 2.30),
                                    lambda u, v: 0.030 + u * (seam(2.24 + v * (math.pi - 2.30)) - 0.0322),
                                    13, 10, off=0.0014, mount_r=0.0474), wall=0.0023, chamfer=0.00054, col=SHELL_CF, rim_col=SEAM_SILVER)
    l_temp = shell_patch(_surf_grid(
        lambda u, v: 0.86 + v * 1.35,
        lambda u, v: (0.146 - 0.017 * math.sin(math.pi * min(1.0, v * 0.85)) - 0.008 * v) + u * (
            seam(0.86 + v * 1.35) - 0.0015 - (0.146 - 0.017 * math.sin(math.pi * min(1.0, v * 0.85)) - 0.008 * v)),
        10, 14, off=0.0012), wall=0.0022, chamfer=0.00052, col=SHELL_CF, rim_col=SEAM_SILVER)
    l_mast = shell_patch(_surf_grid(
        lambda u, v: (1.94 + 0.05 * u) + v * (0.24 - 0.05 * u),
        lambda u, v: (0.048 - 0.008 * v) + u * (0.068 + 0.012 * v),
        10, 7, off=0.0012, mount_r=0.0474), wall=0.0022, chamfer=0.00050, col=SHELL_CF, rim_col=SEAM_SILVER)
    l_pil = _flat([p(0.76, 0.1055), p(1.16, 0.1110), p(1.18, 0.1240), p(0.80, 0.1190)],
                  wall=0.0020, rim_m=0.0042, pitch=0.0076, fill=0.56, corner_c=0.0014, shift_n=0.0028)
    l_nose = _flat([(0.0038, 0.1050, 0.1080), (-0.0038, 0.1050, 0.1080), (-0.0054, 0.0830, 0.1132),
                    (-0.0031, 0.0772, 0.1108), (0.0031, 0.0772, 0.1108), (0.0054, 0.0830, 0.1132)],
                   wall=0.0020, rim_m=0.0028, pitch=0.0062, fill=0.54, corner_c=0.0008, shift_n=0.0006)
    l_max = _flat([p(0.24, 0.0625), p(1.08, 0.0765), p(1.25, 0.0975), p(0.28, 0.0885)],
                  wall=0.0021, rim_m=0.0054, pitch=0.0088, fill=0.61, corner_c=0.0018, shift_n=0.0052)
    ph_p = shell_patch(_surf_grid(lambda u, v: 0.085 * (2.0 * u - 1.0), lambda u, v: 0.056 + v * 0.008,
                                  8, 5, off=-0.0115), 0.0014, 0.00022, BAFFLE_DARK, rim_col=BAFFLE_DARK)
    lip_p = shell_patch(_surf_grid(lambda u, v: 0.095 * (2.0 * u - 1.0), lambda u, v: 0.047 + v * 0.006,
                                   8, 5, off=-0.0130), 0.0014, 0.00022, BAFFLE_DARK, rim_col=BAFFLE_DARK)
    return {'cranium_frontal': front, 'cranium_dome': d_lobe + mirror_x(d_lobe), 'cranium_occipital': o_lobe + mirror_x(o_lobe),
            'left_temporal': l_temp, 'right_temporal': mirror_x(l_temp), 'left_mastoid': l_mast, 'right_mastoid': mirror_x(l_mast),
            'left_orbital_pillar': l_pil, 'right_orbital_pillar': mirror_x(l_pil),
            'nasal_bridge': l_nose, 'left_maxilla': l_max, 'right_maxilla': mirror_x(l_max), 'philtrum': ph_p, 'upper_lip': lip_p}


@functools.lru_cache(maxsize=None)
def ear_microphones():
    """Return anime-mecha concentric binaural ear-pods with luminous cyan halo and gold ring."""
    return fh.ear_microphones()


@functools.lru_cache(maxsize=None)
def camera_and_gimbal_local():
    """Return flush expressive anime-gynoid eye module with cyan-sapphire iris and glossy catchlights."""
    return fh.modern_optics_local()


@functools.lru_cache(maxsize=None)
def moving_rest():
    """Build sleek arched anime brows, seamless tucked eyelids, smile lug and dainty flat V-chin wings."""
    from machine.eyes import gimbal, stereo
    np, ex, p = np_mod(), 0.5 * float(stereo.BASELINE_M), _head_pt
    brow_g = _surf_grid(
        lambda u, v: 0.17 + u * 0.47,
        lambda u, v: (0.1155 + 0.0046 * math.sin(math.pi * (0.14 + 0.80 * u))) + v * (0.0023 * (1.0 - 0.66 * u)),
        15, 3, off=0.0006)
    piv = (np.array([ex, float(gimbal.EYES[1]), float(gimbal.EYES[2])])
           - gimbal.mounted('left') @ np.array([0.0, 0.0, -float(gimbal.LEVER['left'])]))
    lx_u = np.linspace(float(piv[0]) - 0.0108, float(piv[0]) + 0.0108, 15)

    def _lid_grid(a0, a1, tilt_deg=7.8, wing=0.0):
        angs = np.linspace(math.radians(a0), math.radians(a1), 5)
        g = np.zeros((len(lx_u), len(angs), 3))
        for i, xv in enumerate(lx_u):
            dx = float(xv) - float(piv[0])
            u_x = dx / 0.0108
            ry = math.sqrt(max(36e-6, 0.0110 ** 2 - 0.82 * dx * dx))
            rz = math.sqrt(max(24e-6, 0.0082 ** 2 - 0.55 * dx * dx))
            da = math.radians(tilt_deg) * u_x + math.radians(wing) * max(0.0, u_x) ** 2
            for j, a in enumerate(angs):
                af = float(a) + da
                g[i, j] = (float(xv), float(piv[1]) + ry * math.sin(af), float(piv[2]) + rz * math.cos(af))
        return g

    comm_g = _surf_grid(lambda u, v: 0.270 + u * 0.055, lambda u, v: 0.045 + 0.0006 * u + v * 0.0040, 6, 4, off=-0.0165)
    lj_pts = [p(0.115, 0.0245), p(0.84, 0.0445), p(0.92, 0.0640), p(0.115, 0.0510)]
    lj_panel = _flat(lj_pts, wall=0.0021, rim_m=0.0056, pitch=0.0090, fill=0.60, corner_c=0.0018, shift_n=0.0020)
    lj_back = fh.planar_backing(lj_pts, wall=0.0011, inset_m=0.0026, corner_c=0.0018, col=CHASSIS_CF, shift_n=-0.0010)
    lj_ribs = [shell_patch(_surf_grid(lambda u, v: 0.14 + 0.70 * u,
                                      lambda u, v, dy=dy0: (0.0495 + 0.0125 * u) + dy + 0.0011 * v,
                                      10, 3, off=-0.0072), 0.0008, 0.00018, TITANIUM, rim_col=MIC_GOLD)
               for dy0 in (0.0000, 0.0026)]
    sub_ch = np.array([[[(0.014 + 0.024 * float(v)) * float(u), 0.020 + 0.008 * float(v),
                         0.083 - 0.030 * float(v) - 0.008 * (float(u) ** 2)]
                        for v in np.linspace(0.0, 1.0, 5)] for u in np.linspace(-1.0, 1.0, 9)])
    under_jaw = shell_patch(sub_ch, 0.0015, 0.00026, CHASSIS_CF, (0.0, 0.055, 0.055), PANEL_RIM)
    ryz = csg.filleted2d([(0.044, 0.056), (0.052, 0.060), (0.068, 0.028), (0.068, 0.020), (0.060, 0.020), (0.043, 0.048)], r=0.0020)
    ram = (csg.extrude(ryz, a=0.0398, b=0.0434, axis=0, colour=CHASSIS_CF, chamfer=0.0006).tris()
           + csg.extrude(csg.circle2d(0.0036, fn=20, at=(0.066, 0.024)), a=0.0434, b=0.0468, axis=0, colour=MIC_GOLD, chamfer=0.0004).tris())
    yoke = csg.extrude(csg.filleted2d(csg.box2d((-0.020, 0.020), (0.030, 0.036)), r=0.0016), a=0.062, b=0.067, axis=2, colour=CHASSIS_CF, chamfer=0.0004).tris()
    lj_s = lj_panel + lj_back + lj_ribs[0] + lj_ribs[1]
    jaw_all = under_jaw + lj_s + mirror_x(lj_s) + yoke + ram + mirror_x(ram)
    # Tucked seamless upper porcelain lid dome (`34.5..58.0 deg`) eliminates crease crack completely!
    ulid = (shell_patch(_lid_grid(24.0, 34.0, 7.8, wing=6.0), 0.0010, 0.00022, BAFFLE_DARK, piv, BAFFLE_DARK)
            + shell_patch(_lid_grid(34.5, 58.0, 6.0, wing=3.0), 0.0008, 0.00016, PORCELAIN, piv, PORCELAIN))
    llid = shell_patch(_lid_grid(-27.0, -17.5, 3.6, wing=2.0), 0.0008, 0.00018, PANEL_RIM, piv, TITANIUM)
    return shell_patch(brow_g, 0.0010, 0.00022, BROW_DARK, rim_col=BAFFLE_DARK), ulid, llid, piv, shell_patch(comm_g, 0.0014, 0.00028, BAFFLE_DARK, rim_col=BAFFLE_DARK), jaw_all
