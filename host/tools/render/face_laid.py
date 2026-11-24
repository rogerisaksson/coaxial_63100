"""Feminine anime/mecha cranial & facial armor panels with framed fine-mesh honeycomb windows."""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg, face_honeycomb as fh, knee_loads as kl, neck_laid as nl  # noqa: E402
from tools.render.face_honeycomb import (
    ANODIZED, BAFFLE_DARK, CHASSIS_CF, HORN_BRASS, MIC_GOLD, PANEL_RIM,
    SERVO_CASE, SHELL_CF, TITANIUM, mirror_x, np_mod, rev, rot, shell_patch,
)

SKULL_YC, SKULL_ZC, SKULL_RY = 0.1460, 0.0020, 0.0825


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
    """Build narrow internal carbon chassis (`+-45 mm`), contoured dark orbital socket shrouds and ear struts."""
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
    cowl_o = [((0.032 + 0.0205 * math.cos(a)), (0.100 + 0.0142 * math.sin(a))) for a in (2.0 * math.pi * k / 28 for k in range(28))]
    cowl_i = [((0.032 + 0.0138 * math.cos(a)), (0.100 + 0.0110 * math.sin(a))) for a in (2.0 * math.pi * k / 28 for k in range(28))]
    cowl_l = csg.extrude(cowl_o, holes=[cowl_i], a=0.069, b=0.080, axis=2, colour=BAFFLE_DARK, chamfer=0.0006).tris()
    oral_g = np.array([[[0.021 * float(u), 0.026 + 0.026 * float(v), 0.074 - 0.010 * float(u * u) - 0.003 * (1.0 - float(v))]
                        for v in np.linspace(0.0, 1.0, 5)] for u in np.linspace(-1.0, 1.0, 11)])
    oral = shell_patch(oral_g, 0.0022, 0.00036, BAFFLE_DARK, (0.0, 0.045, 0.010), BAFFLE_DARK)
    ear = csg.sweep(csg.circle2d(0.0042, fn=18), [(0.040, 0.093, 0.026), (0.052, 0.093, 0.012), (0.061, 0.093, 0.004)],
                    chamfer_ends=0.0006, colour=CHASSIS_CF).tris()
    return csg.union(flange, bulk, fore, deck, wl, wr, eye_m).tris() + cowl_l + mirror_x(cowl_l) + oral + ear + mirror_x(ear)


@functools.lru_cache(maxsize=None)
def compute_heatsink():
    """Return recessed cranial compute fins and heatpipes behind the fine-mesh honeycomb windows."""
    return fh.compute_heatsink()


def _skull_rx(z_norm, y_norm):
    """Smooth feminine ovoid half-width: ~70.6 mm at parietal boss near ear, ~50.5 mm at front temples."""
    zt = min(1.0, max(0.0, (z_norm - 0.08) / 0.88))
    return max(0.0465, 0.0708 - 0.0196 * (zt ** 0.82) - 0.0072 * (max(0.0, -z_norm) ** 1.8) - 0.0064 * (max(0.0, -y_norm) ** 1.55))


def _skull_x_from_yz(y, z):
    yn = (y - SKULL_YC) / (SKULL_RY if y >= SKULL_YC else 0.0760)
    zn = (z - SKULL_ZC) / (0.0965 if z >= SKULL_ZC else 0.0920)
    rx = _skull_rx(zn, yn)
    return min(0.0708, rx * math.sqrt(max(0.22, 1.0 - min(0.90, 0.86 * yn * yn + 0.18 * zn * zn))))


def _crown_lobe(th0, th1, x_in=0.0022, x_out_frac=0.92, n_u=20, brow_arch=False):
    """Build one side (`+x`) unbroken V-chevron anime/mecha cranial shield with 3.8 mm air gaps."""
    np = np_mod()
    us, vs = np.linspace(0.0, 1.0, n_u), np.linspace(0.0, 1.0, 9)
    grid = np.zeros((len(us), len(vs), 3))
    for i, u in enumerate(us):
        fu = float(u)
        for j, v in enumerate(vs):
            fv = float(v)
            if brow_arch:
                arch_up = 0.024 * math.sin(math.pi * min(1.0, (fv + 0.04) / 0.78))
                hood = -0.125 * (max(0.0, (fv - 0.72) / 0.28) ** 1.22)
                th_s = th0 + arch_up + hood
            else:
                th_s = th0
            th = th_s + fu * (th1 - th_s)
            sy, cz = math.sin(th), math.cos(th)
            rx = _skull_rx(cz, sy)
            ry = SKULL_RY if (sy >= 0.0 or cz >= 0.0) else 0.0760
            rz = (0.0976 + 0.0020 * math.exp(-((th + 0.08) / 0.30) ** 2)) if cz >= 0.0 else 0.0920
            x_in_u = x_in
            if brow_arch:
                if fu <= 0.16:
                    s_g = 1.0 - min(1.0, max(0.0, (fu - 0.02) / 0.13))
                    x_in_u = x_in + 0.0064 * s_g * s_g * (3.0 - 2.0 * s_g)
                elif fu >= 0.50:
                    s_v = (fu - 0.50) / 0.50
                    x_in_u = x_in + 0.0152 * (s_v ** 1.15)
            x = x_in_u + fv * (x_out_frac * rx - x_in_u)
            ux2 = min(0.88, (x / max(rx, 0.04)) ** 2)
            r = math.sqrt(max(0.14, 1.0 - 0.78 * ux2))
            ry_eff = ry * (r if sy >= 0.0 else (1.0 - 0.10 * ux2))
            mecha_v = (0.0012 * math.exp(-((fu - (0.20 + 0.44 * fv)) / 0.09) ** 2)) if brow_arch else 0.0
            z_wrap = (0.0054 * (max(0.0, (fv - 0.65) / 0.35) ** 1.15) * max(0.0, 1.0 - fu / 0.32)) if brow_arch else 0.0
            grid[i, j] = (x, SKULL_YC + ry_eff * sy, SKULL_ZC + rz * r * cz + mecha_v + z_wrap)
    return grid


def _temporal_grid(th0, th1, n_u=18):
    """Sleek swept-back mecha temporal/mastoid side shield framing fine-mesh honeycomb."""
    np = np_mod()
    ts_u, ts_v = np.linspace(0.0, 1.0, n_u), np.linspace(0.0, 1.0, 7)
    grid = np.zeros((len(ts_u), len(ts_v), 3))
    for i, u in enumerate(ts_u):
        th = th0 + float(u) * (th1 - th0)
        fa, sy, cz = (th + 0.24) / 4.12, math.sin(th), math.cos(th)
        rx = _skull_rx(cz, sy)
        r_top = math.sqrt(max(0.14, 1.0 - 0.78 * (0.94 ** 2)))
        ry0, rz0 = SKULL_RY if (sy >= 0.0 or cz >= 0.0) else 0.0760, 0.0970 if cz >= 0.0 else 0.0920
        yt, zt = SKULL_YC + ry0 * (r_top if sy >= 0.0 else 0.90) * sy - 0.0028, SKULL_ZC + rz0 * r_top * cz
        zb = 0.074 - 0.138 * fa - 0.010 * math.sin(math.pi * fa)
        yb_base, dz_e = 0.1145 - 0.032 * (fa ** 0.88), zb - 0.004
        ear_arc = (0.093 + math.sqrt(max(0.0, 0.0258 ** 2 - dz_e * dz_e))) if abs(dz_e) < 0.0258 else 0.074
        yb = 0.5 * (yb_base + ear_arc + math.hypot(yb_base - ear_arc, 0.0036))
        for j, v in enumerate(ts_v):
            wv = float(v)
            yv, zv = (1.0 - wv) * yt + wv * yb, (1.0 - wv) * zt + wv * zb
            grid[i, j] = (_skull_x_from_yz(yv, zv), yv, zv)
    return grid


@functools.lru_cache(maxsize=None)
def static_plates():
    """Build feminine anime/mecha cranial & facial armor plates with framed fine-mesh honeycomb windows."""
    np = np_mod()
    f_lobe = shell_patch(_crown_lobe(-0.370, 0.87, x_in=0.0019, x_out_frac=0.94, n_u=22, brow_arch=True),
                         0.0026, 0.00058, SHELL_CF)
    # Sleek V-chevron fine-mesh honeycomb coronal crest nestled in the upper forehead V-notch (`b2ee3d8d` / `06948046`).
    cr_u, cr_v = np.linspace(0.0, 1.0, 9), np.linspace(-1.0, 1.0, 7)
    cr_g = np.zeros((len(cr_u), len(cr_v), 3))
    for i, u in enumerate(cr_u):
        fu = float(u)
        th = 0.30 + 0.56 * fu
        hw = 0.0036 + 0.0104 * (fu ** 1.12)
        for j, v in enumerate(cr_v):
            xv = hw * float(v)
            r = math.sqrt(max(0.5, 1.0 - 0.6 * ((xv / 0.055) ** 2)))
            cr_g[i, j] = (xv, SKULL_YC + SKULL_RY * r * math.sin(th), SKULL_ZC + 0.0972 * r * math.cos(th))
    fg_c = np.array([[[(0.0064 - 0.0012 * float(u)) * float(v), 0.1180 + 0.0106 * float(u),
                       0.0936 + 0.0010 * float(u) - 0.0007 * abs(float(v))]
                      for v in np.linspace(-1.0, 1.0, 5)] for u in np.linspace(0.0, 1.0, 6)])
    front = (f_lobe + mirror_x(f_lobe)
             + fh.honeycomb_patch(cr_g, nc=4, nr=4, wall=0.0024, margin=(0.18, 0.18), fill=0.75, col=SHELL_CF)
             + fh.honeycomb_patch(fg_c, nc=3, nr=2, wall=0.0022, margin=(0.18, 0.20), fill=0.74, col=PANEL_RIM, hole_col=BAFFLE_DARK))

    d_lobe = fh.honeycomb_patch(_crown_lobe(0.93, 2.34, x_in=0.0026, x_out_frac=0.90, n_u=18),
                                nc=7, nr=7, wall=0.0026, margin=(0.15, 0.15), fill=0.76, col=SHELL_CF)
    o_lobe = fh.honeycomb_patch(_crown_lobe(2.42, 3.96, x_in=0.0026, x_out_frac=0.89, n_u=16),
                                nc=6, nr=5, wall=0.0026, margin=(0.16, 0.16), fill=0.76, col=SHELL_CF)
    l_temp = fh.honeycomb_patch(_temporal_grid(-0.14, 2.30, 18), nc=7, nr=4, wall=0.0026, margin=(0.15, 0.18), fill=0.75, col=SHELL_CF)
    l_mast = fh.honeycomb_patch(_temporal_grid(2.48, 3.86, 14), nc=4, nr=3, wall=0.0026, margin=(0.18, 0.20), fill=0.74, col=SHELL_CF)

    pil_g = np.array([[[0.0452 + 0.0056 * float(v) + 0.0005 * float(u),
                        0.0958 + 0.0124 * float(u) + 0.0014 * float(v),
                        0.0924 - 0.0168 * float(v) + 0.0012 * math.sin(math.pi * float(u))]
                       for v in np.linspace(0.0, 1.0, 5)] for u in np.linspace(0.0, 1.0, 8)])

    ns_u, ns_v = np.linspace(0.0, 1.0, 21), np.linspace(-1.0, 1.0, 9)
    nose_g = np.zeros((len(ns_u), len(ns_v), 3))
    for i, u in enumerate(ns_u):
        fu = float(u)
        if fu < 0.22:
            ts = fu / 0.22
            yv, zc, hw, drop = 0.0658 + 0.0058 * ts, 0.1004 + 0.0094 * ts, 0.0102 - 0.0006 * ts, 0.0086 + 0.0014 * ts
        else:
            tb = (fu - 0.22) / 0.78
            yv = 0.0716 + 0.0434 * tb
            zc = 0.1098 - 0.0132 * (tb ** 0.78) - 0.0042 * math.exp(-((tb - 0.72) / 0.22) ** 2)
            hw = 0.0080 + 0.0016 * math.exp(-(tb / 0.18) ** 2) - 0.0020 * math.sin(0.80 * math.pi * tb) + 0.0006 * (tb ** 2)
            drop = 0.0100 - 0.0054 * (tb ** 0.78)
        for j, v in enumerate(ns_v):
            ridge = max(0.0, (abs(float(v)) - 0.22) / 0.78)
            nose_g[i, j] = (hw * float(v), yv - 0.0010 * (ridge ** 1.2) * (1.0 - fu), zc - drop * (ridge ** 1.25))

    def _max_pt(t, sv):
        if t <= 0.64:
            tf = t / 0.64
            yh = 0.0954 - 0.0054 * math.sin(math.pi * min(1.0, max(0.0, (tf - 0.02) / 0.94))) + 0.0024 * (tf ** 1.3)
            yl = 0.0528 + 0.0074 * (tf ** 1.40) + 0.0068 * (max(0.0, (tf - 0.52) / 0.48) ** 1.3)
            xm_low, xm_mid, xo = 0.0246 + 0.0238 * (tf ** 0.90), 0.0124 + 0.0354 * tf, 0.0110 + 0.0372 * tf
            zh, zl = 0.0968 - 0.0076 * (tf ** 1.35), 0.0938 - 0.0188 * (tf ** 1.25)
        else:
            ta = (t - 0.64) / 0.36
            yh, yl = 0.0978 - 0.0028 * ta, 0.0670 + 0.0158 * ta
            xm_low = xm_mid = 0.0484 + 0.0203 * (ta ** 0.80)
            xo, zh, zl = 0.0482 + 0.0205 * (ta ** 0.80), 0.0892 - 0.0632 * ta, 0.0750 - 0.0490 * ta
        yv = (1.0 - sv) * yl + sv * yh
        s_nl = min(1.0, max(0.0, (sv - 0.05) / 0.42))
        xv = (1.0 - sv) * ((1.0 - s_nl) * xm_low + s_nl * xm_mid) + sv * xo
        apple = 0.0036 * math.sin(math.pi * min(1.0, sv * 1.05)) * (1.0 - 0.65 * max(0.0, (t - 0.58) / 0.42))
        return xv, yv, (1.0 - sv) * zl + sv * zh + apple

    vs_m = np.linspace(0.0, 1.0, 7)
    mx_f = np.array([[_max_pt(float(u), float(v)) for v in vs_m] for u in np.linspace(0.0, 0.34, 10)])
    mx_s = np.array([[_max_pt(float(u), float(v)) for v in vs_m] for u in np.linspace(0.37, 1.0, 15)])

    ph_g = np.array([[[(0.0178 - 0.0054 * float(v)) * float(u), 0.0548 + 0.0080 * float(v),
                       0.1012 - 0.0076 * (float(u) ** 2) - 0.0007 * math.exp(-((float(u) / 0.22) ** 2))]
                      for v in np.linspace(0.0, 1.0, 4)] for u in np.linspace(-1.0, 1.0, 9)])
    lip_g = np.array([[[(0.0182 - 0.0004 * float(v)) * float(u),
                        0.0484 + 0.0056 * float(v) - 0.0016 * (float(u) ** 2) + 0.0007 * math.exp(-(((abs(float(u)) - 0.24) / 0.14) ** 2)) * float(v),
                        0.1026 - 0.0082 * (float(u) ** 2) + (0.0012 if abs(float(v) - 0.48) < 0.1 else -0.0004 * float(v))]
                       for v in (0.0, 0.48, 1.0)] for u in np.linspace(-1.0, 1.0, 11)])

    l_max = shell_patch(mx_f, 0.0024, 0.00052, SHELL_CF) + fh.honeycomb_patch(mx_s, nc=6, nr=3, wall=0.0024, margin=(0.16, 0.20), fill=0.75, col=SHELL_CF)
    return {'cranium_frontal': front, 'cranium_dome': d_lobe + mirror_x(d_lobe), 'cranium_occipital': o_lobe + mirror_x(o_lobe),
            'left_temporal': l_temp, 'right_temporal': mirror_x(l_temp), 'left_mastoid': l_mast, 'right_mastoid': mirror_x(l_mast),
            'left_orbital_pillar': shell_patch(pil_g, 0.0022, 0.00050, SHELL_CF),
            'right_orbital_pillar': mirror_x(shell_patch(pil_g, 0.0022, 0.00050, SHELL_CF)),
            'nasal_bridge': shell_patch(nose_g, 0.0024, 0.00054, SHELL_CF), 'left_maxilla': l_max, 'right_maxilla': mirror_x(l_max),
            'philtrum': shell_patch(ph_g, 0.0022, 0.00046, SHELL_CF), 'upper_lip': shell_patch(lip_g, 0.0022, 0.00046, SHELL_CF)}


@functools.lru_cache(maxsize=None)
def ear_microphones():
    """Build concentric circular binaural transducer pods (`x_max ~ 70.2 mm`) flush on the slimmed mecha temples."""
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
    """Return modern recessed orbital camera, luminous iris bezel and 2-axis titanium gimbal ring."""
    return fh.modern_optics_local()


@functools.lru_cache(maxsize=None)
def moving_rest():
    """Build arched feminine mecha brows, almond-tucked eyeliner lids, and heart/V-line honeycomb mandible."""
    from machine.eyes import gimbal, stereo
    np, ex = np_mod(), 0.5 * float(stereo.BASELINE_M)
    br_u, br_v = np.linspace(0.0, 1.0, 9), np.array([0.0, 0.55, 1.0])
    brow_g = np.zeros((len(br_u), len(br_v), 3))
    for i, u in enumerate(br_u):
        t = float(u)
        x = (ex - 0.0195) + 0.0295 * t
        arch = 0.0028 * math.sin(math.pi * min(1.0, (t + 0.06) * 0.92))
        yl, yh = 0.1100 + arch + 0.0006 * t, 0.1100 + arch + 0.0006 * t + (0.0044 - 0.0016 * (t ** 1.3))
        zf = 0.0986 - 0.0030 * t - 0.0048 * (max(0.0, (t - 0.38) / 0.62) ** 1.25)
        for j, v in enumerate(br_v):
            sv = float(v)
            brow_g[i, j] = (x, yl + sv * (yh - yl), zf + (0.0009 if abs(sv - 0.55) < 0.08 else (-0.0007 if sv < 0.1 else -0.0012)))

    piv = (np.array([ex, float(gimbal.EYES[1]), float(gimbal.EYES[2])])
           - gimbal.mounted('left') @ np.array([0.0, 0.0, -float(gimbal.LEVER['left'])]))
    lx_u = np.linspace(float(piv[0]) - 0.0106, float(piv[0]) + 0.0106, 11)

    def _lid_grid(a0, a1, tilt_deg=5.5):
        angs = np.linspace(math.radians(a0), math.radians(a1), 5)
        g = np.zeros((len(lx_u), len(angs), 3))
        for i, xv in enumerate(lx_u):
            dx = float(xv) - float(piv[0])
            u2 = (dx / 0.0106) ** 2
            rc = math.sqrt(max(36e-6, 0.0115 ** 2 - dx * dx))
            da = math.radians(tilt_deg) * (dx / 0.0106)
            z_tuck = -0.0026 * (u2 ** 1.2)
            for j, a in enumerate(angs):
                af = float(a) + da
                g[i, j] = (float(xv), float(piv[1]) + rc * math.sin(af), float(piv[2]) + rc * math.cos(af) + z_tuck)
        return g

    comm_g = np.array([[[0.0192 + 0.0066 * float(u), 0.0482 + 0.0012 * float(u) + (2.0 * float(v) - 1.0) * 0.0010,
                         0.0932 - 0.0082 * float(u)] for v in np.linspace(0.0, 1.0, 4)] for u in np.linspace(0.0, 1.0, 5)])

    ch_u, ch_v = np.linspace(-1.0, 1.0, 11), np.array([0.0, 0.22, 0.56, 0.84, 1.0])
    yz_p = ((0.0148, -0.0102), (0.0206, 0.0016), (0.0348, -0.0018), (0.0430, 0.0018), (0.0458, 0.0002))
    chin_g = np.zeros((len(ch_u), len(ch_v), 3))
    for i, u in enumerate(ch_u):
        fu, au = float(u), abs(float(u))
        for j, (sv, (yb, zp)) in enumerate(zip(ch_v, yz_p)):
            x = (0.0078 + 0.0078 * (float(sv) ** 0.85)) * fu
            yv = yb + (0.0032 * (au ** 1.6) if j <= 1 else -0.0012 * (au ** 2))
            chin_g[i, j] = (x, yv, 0.1002 - 0.0072 * ((x / 0.0160) ** 2) + zp)

    lj_u, lj_v = np.linspace(0.0, 1.0, 12), np.linspace(0.0, 1.0, 6)
    ljaw_g = np.zeros((len(lj_u), len(lj_v), 3))
    for i, u in enumerate(lj_u):
        t = float(u)
        for j, v in enumerate(lj_v):
            sv = float(v)
            x_in = 0.0114 + 0.0074 * (sv ** 0.85)
            x = x_in + (0.0436 - x_in) * (t ** 0.94)
            yl, yh = 0.0182 + 0.0246 * (t ** 1.16), 0.0392 + 0.0112 * (t ** 1.18)
            zl, zh = 0.0922 - 0.0342 * (t ** 1.20), 0.0940 - 0.0348 * (t ** 1.20)
            ljaw_g[i, j] = (x, (1.0 - sv) * yl + sv * yh, (1.0 - sv) * zl + sv * zh + 0.0014 * math.sin(math.pi * sv))

    ryz = csg.filleted2d([(0.030, 0.060), (0.044, 0.064), (0.068, 0.028), (0.068, 0.020), (0.060, 0.020), (0.031, 0.052)], r=0.0020)
    ram = (csg.extrude(ryz, a=0.0398, b=0.0434, axis=0, colour=CHASSIS_CF, chamfer=0.0006).tris()
           + csg.extrude(csg.circle2d(0.0036, fn=20, at=(0.066, 0.024)), a=0.0434, b=0.0468, axis=0, colour=TITANIUM, chamfer=0.0004).tris())
    yoke = csg.extrude(csg.filleted2d(csg.box2d((-0.024, 0.024), (0.019, 0.025)), r=0.0016), a=0.074, b=0.079, axis=2, colour=CHASSIS_CF, chamfer=0.0004).tris()
    lj_s = fh.honeycomb_patch(ljaw_g, nc=5, nr=2, wall=0.0024, margin=(0.18, 0.22), fill=0.74, col=SHELL_CF)
    jaw_all = shell_patch(chin_g, 0.0024, 0.00052, SHELL_CF) + lj_s + mirror_x(lj_s) + yoke + ram + mirror_x(ram)
    ulid = (shell_patch(_lid_grid(22.0, 48.0, 5.5), 0.0012, 0.00026, SHELL_CF, piv)
            + shell_patch(_lid_grid(18.5, 22.2, 5.5), 0.0013, 0.00022, BAFFLE_DARK, piv, BAFFLE_DARK))
    return shell_patch(brow_g, 0.0022, 0.00046, SHELL_CF), ulid, shell_patch(_lid_grid(-48.0, -24.0, 3.5), 0.0011, 0.00024, SHELL_CF, piv), piv, shell_patch(comm_g, 0.0019, 0.00036, SHELL_CF), jaw_all
