"""Slimmed temporal skull and faceted hard-surface facial armor panels with open cooling gaps."""
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


def rod(p0, p1, r=0.00090, col=TITANIUM):
    """Build chamfered titanium pushrod with brass clevis collars between `p0` and `p1`."""
    np = np_mod()
    a, b = np.asarray(p0, float), np.asarray(p1, float)
    ln = float(np.linalg.norm(b - a))
    if ln < 0.003:
        return []
    u, c2, c3 = (b - a) / ln, csg.circle2d(r, fn=14), csg.circle2d(1.50 * r, fn=14)
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
    """Build narrow internal carbon chassis (`+-45 mm`), recessed orbital cowling, baffle and ear struts."""
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
    cowl_o = [((0.032 + 0.0205 * math.cos(a)), (0.100 + 0.0142 * math.sin(a)))
              for a in (2.0 * math.pi * k / 28 for k in range(28))]
    cowl_i = [((0.032 + 0.0138 * math.cos(a)), (0.100 + 0.0110 * math.sin(a)))
              for a in (2.0 * math.pi * k / 28 for k in range(28))]
    cowl_l = csg.extrude(cowl_o, holes=[cowl_i], a=0.071, b=0.081, axis=2, colour=BAFFLE_DARK, chamfer=0.0006).tris()
    oral_g = np.array([[[0.023 * float(u), 0.025 + 0.027 * float(v), 0.072 - 0.011 * float(u * u) - 0.003 * (1.0 - float(v))]
                        for v in np.linspace(0.0, 1.0, 5)] for u in np.linspace(-1.0, 1.0, 11)])
    oral = shell_patch(oral_g, 0.0022, 0.00036, BAFFLE_DARK, (0.0, 0.045, 0.010), BAFFLE_DARK)
    ear = csg.sweep(csg.circle2d(0.0042, fn=18), [(0.040, 0.093, 0.026), (0.052, 0.093, 0.012), (0.061, 0.093, 0.004)],
                    chamfer_ends=0.0006, colour=CHASSIS_CF).tris()
    return csg.union(flange, bulk, fore, deck, wl, wr, eye_m).tris() + cowl_l + mirror_x(cowl_l) + oral + ear + mirror_x(ear)


@functools.lru_cache(maxsize=None)
def compute_heatsink():
    """Return recessed cranial compute fins and heatpipes behind the ventilated armor gaps."""
    return fh.compute_heatsink()


def _skull_rx(z_norm, y_norm):
    """Smooth ovoid half-width: ~70.6 mm at parietal boss near ear, ~51.5 mm at front temples."""
    zt = min(1.0, max(0.0, (z_norm - 0.08) / 0.88))
    front_taper = 0.0192 * (zt ** 0.82)
    rear_taper = 0.0072 * (max(0.0, -z_norm) ** 1.8)
    low_taper = 0.0062 * (max(0.0, -y_norm) ** 1.55)
    return max(0.047, 0.0708 - front_taper - rear_taper - low_taper)


def _skull_x_from_yz(y, z):
    yn = (y - SKULL_YC) / (SKULL_RY if y >= SKULL_YC else 0.0760)
    zn = (z - SKULL_ZC) / (0.0965 if z >= SKULL_ZC else 0.0920)
    rx = _skull_rx(zn, yn)
    r2 = min(0.90, 0.86 * yn * yn + 0.18 * zn * zn)
    return min(0.0708, rx * math.sqrt(max(0.22, 1.0 - r2)))


def _crown_lobe(th0, th1, x_in=0.0024, x_out_frac=0.90, n_u=20, brow_arch=False):
    """Build one side (`+x`) full-coverage faceted cranial plate with 4 mm sagittal & temporal air gaps."""
    np = np_mod()
    us, vs = np.linspace(0.0, 1.0, n_u), np.array([0.0, 0.24, 0.54, 0.76, 1.0])
    grid = np.zeros((len(us), len(vs), 3))
    for i, u in enumerate(us):
        fu = float(u)
        for j, v in enumerate(vs):
            fv = float(v)
            if brow_arch:
                hood = -0.105 * (max(0.0, (fv - 0.75) / 0.25) ** 1.25)
                th_s = th0 + 0.018 * math.sin(math.pi * min(1.0, fv / 0.78)) + hood
            else:
                th_s = th0
            th = th_s + fu * (th1 - th_s)
            sy, cz = math.sin(th), math.cos(th)
            rx = _skull_rx(cz, sy)
            ry = SKULL_RY if (sy >= 0.0 or cz >= 0.0) else 0.0760
            rz = (0.0972 + 0.0016 * math.exp(-((th + 0.05) / 0.32) ** 2)) if cz >= 0.0 else 0.0920
            x_in_u = x_in
            if brow_arch and fu <= 0.24:
                s_glab = 1.0 - min(1.0, max(0.0, (fu - 0.04) / 0.18))
                x_in_u = x_in + 0.0078 * s_glab * s_glab * (3.0 - 2.0 * s_glab)
            x = x_in_u + fv * (x_out_frac * rx - x_in_u)
            ux2 = min(0.88, (x / max(rx, 0.04)) ** 2)
            r = math.sqrt(max(0.14, 1.0 - 0.78 * ux2))
            ry_eff = ry * (r if sy >= 0.0 else (1.0 - 0.10 * ux2))
            facet = -0.0008 * abs(fv - 0.54)
            grid[i, j] = (x, SKULL_YC + ry_eff * sy + facet * max(0.0, sy), SKULL_ZC + rz * r * cz + facet * cz)
    return grid


def _temporal_grid(th0, th1, n_u=18):
    """Sleek temporal/mastoid side plate meeting the forehead/dome across a crisp 4 mm cooling gap."""
    np = np_mod()
    ts_u, ts_v = np.linspace(0.0, 1.0, n_u), np.array([0.0, 0.28, 0.58, 0.82, 1.0])
    grid = np.zeros((len(ts_u), len(ts_v), 3))
    for i, u in enumerate(ts_u):
        th = th0 + float(u) * (th1 - th0)
        fa = (th + 0.24) / 4.12
        sy, cz = math.sin(th), math.cos(th)
        rx = _skull_rx(cz, sy)
        x_top = 0.93 * rx
        r_top = math.sqrt(max(0.14, 1.0 - 0.78 * ((x_top / rx) ** 2)))
        ry0, rz0 = SKULL_RY if (sy >= 0.0 or cz >= 0.0) else 0.0760, 0.0970 if cz >= 0.0 else 0.0920
        yt, zt = SKULL_YC + ry0 * (r_top if sy >= 0.0 else 0.90) * sy - 0.0028, SKULL_ZC + rz0 * r_top * cz
        zb = 0.072 - 0.136 * fa - 0.010 * math.sin(math.pi * fa)
        yb_base = 0.1155 - 0.033 * (fa ** 0.88)
        dz_e = zb - 0.004
        ear_arc = (0.093 + math.sqrt(max(0.0, 0.0258 ** 2 - dz_e * dz_e))) if abs(dz_e) < 0.0258 else 0.074
        yb = 0.5 * (yb_base + ear_arc + math.hypot(yb_base - ear_arc, 0.0036))
        for j, v in enumerate(ts_v):
            wv = float(v)
            yv, zv = (1.0 - wv) * yt + wv * yb, (1.0 - wv) * zt + wv * zb
            crease = -0.0008 * abs(wv - 0.58)
            grid[i, j] = (_skull_x_from_yz(yv, zv) + crease, yv, zv)
    return grid


@functools.lru_cache(maxsize=None)
def static_plates():
    """Build cohesive hard-surface facial & cranial armor plates separated by 3.5-4.5 mm airflow gaps."""
    np = np_mod()
    f_lobe = shell_patch(_crown_lobe(-0.365, 0.86, x_in=0.0020, x_out_frac=0.94, n_u=22, brow_arch=True),
                         0.0028, 0.00065, SHELL_CF)
    fg_u, fg_v = np.linspace(0.0, 1.0, 6), np.linspace(-1.0, 1.0, 5)
    fg_c = np.array([[[(0.0076 - 0.0012 * float(u)) * float(v),
                       0.1188 + 0.0124 * float(u),
                       0.0924 + 0.0014 * float(u) - 0.0008 * abs(float(v))]
                      for v in fg_v] for u in fg_u])
    front = f_lobe + mirror_x(f_lobe) + fh.honeycomb_patch(fg_c, nc=2, nr=2, wall=0.0024, margin=(0.18, 0.20),
                                                           fill=0.68, col=PANEL_RIM, hole_col=BAFFLE_DARK)

    d_lobe = fh.honeycomb_patch(_crown_lobe(0.93, 2.34, x_in=0.0028, x_out_frac=0.89, n_u=18),
                                nc=4, nr=4, wall=0.0028, margin=(0.24, 0.20), fill=0.66, col=SHELL_CF, hole_col=CHASSIS_CF)
    dome = d_lobe + mirror_x(d_lobe)

    o_lobe = fh.honeycomb_patch(_crown_lobe(2.42, 3.96, x_in=0.0028, x_out_frac=0.88, n_u=16),
                                nc=4, nr=3, wall=0.0028, margin=(0.24, 0.22), fill=0.66, col=SHELL_CF, hole_col=CHASSIS_CF)
    occip = o_lobe + mirror_x(o_lobe)

    l_temp = fh.honeycomb_patch(_temporal_grid(-0.14, 2.30, 18), nc=4, nr=2, wall=0.0028,
                                margin=(0.24, 0.30), fill=0.66, col=SHELL_CF, hole_col=CHASSIS_CF)
    l_mast = shell_patch(_temporal_grid(2.48, 3.86, 14), 0.0028, 0.00065, SHELL_CF)

    # Contoured lateral orbital rim (`y = 95.0..108.6 mm`, `x = 44.2..50.8 mm`) framing the outer eye corner.
    pil_u, pil_v = np.linspace(0.0, 1.0, 7), np.linspace(0.0, 1.0, 5)
    pil_g = np.array([[[0.0442 + 0.0064 * float(v) + 0.0008 * float(u),
                        0.0950 + 0.0136 * float(u),
                        0.0856 - 0.0134 * float(v) + 0.0010 * math.sin(math.pi * float(u))]
                       for v in pil_v] for u in pil_u])

    # Architectural 5-facet nasal bridge with nasion saddle dip & flared alar shield (`y = 64.6..115.0 mm`).
    ns_u = np.linspace(0.0, 1.0, 19)
    ns_v = np.array([-1.0, -0.68, -0.28, 0.0, 0.28, 0.68, 1.0])
    nose_g = np.zeros((len(ns_u), len(ns_v), 3))
    for i, u in enumerate(ns_u):
        fu = float(u)
        if fu < 0.22:
            ts = fu / 0.22
            yv, zc = 0.0646 + 0.0064 * ts, 0.0998 + 0.0108 * ts
            hw, drop = 0.0136 - 0.0012 * ts, 0.0108 + 0.0018 * ts
        else:
            tb = (fu - 0.22) / 0.78
            yv = 0.0710 + 0.0440 * tb
            saddle = -0.0038 * math.exp(-((tb - 0.74) / 0.20) ** 2)
            zc = 0.1106 - 0.0138 * (tb ** 0.78) + saddle
            alar = 0.0026 * math.exp(-(tb / 0.20) ** 2)
            hw = 0.0098 + alar - 0.0028 * math.sin(0.76 * math.pi * tb) - 0.0006 * tb
            drop = 0.0126 - 0.0070 * (tb ** 0.78)
        for j, v in enumerate(ns_v):
            av = abs(float(v))
            ridge = 0.0 if av <= 0.28 else (av - 0.28) / 0.72
            nose_g[i, j] = (hw * float(v), yv - 0.0012 * ridge * (1.0 - fu), zc - drop * (ridge ** 1.08))

    # Full-cheek malar armor plate with diagonal nasolabial fold & almond lower-orbital rim.
    mx_u, mx_v = np.linspace(0.0, 1.0, 22), np.array([0.0, 0.28, 0.62, 0.86, 1.0])
    max_g = np.zeros((len(mx_u), len(mx_v), 3))
    for i, u in enumerate(mx_u):
        t = float(u)
        if t <= 0.64:
            tf = t / 0.64
            almond = math.sin(math.pi * min(1.0, max(0.0, (tf - 0.02) / 0.94)))
            yh = 0.0966 - 0.0068 * almond - 0.0026 * tf
            yl = 0.0496 + 0.0092 * (tf ** 1.50) + 0.0096 * max(0.0, (tf - 0.55) / 0.45) ** 1.4
            xm = 0.0252 + 0.0234 * (tf ** 0.88)
            xo = 0.0134 + 0.0362 * tf
            zh = 0.0962 - 0.0176 * (tf ** 1.32)
            zl = 0.0926 - 0.0210 * (tf ** 1.25)
        else:
            ta = (t - 0.64) / 0.36
            yh = 0.0940 + 0.0008 * ta
            yl = 0.0684 + 0.0148 * ta
            xm = 0.0486 + 0.0201 * (ta ** 0.80)
            xo = 0.0496 + 0.0191 * (ta ** 0.80)
            zh = 0.0786 - 0.0526 * ta
            zl = 0.0716 - 0.0456 * ta
        for j, v in enumerate(mx_v):
            sv = float(v)
            yv = (1.0 - sv) * yl + sv * yh
            # Medial edge follows diagonal nasolabial fold (x=25.2 mm at mouth -> 16.8 mm at nostril -> 13.4 mm at inner eye)
            s_nl = min(1.0, max(0.0, (sv - 0.06) / 0.48))
            x_med = (1.0 - s_nl) * xm + s_nl * (0.0168 + 0.0320 * min(1.0, t / 0.64) if t <= 0.64 else xm)
            x_val = (1.0 - sv) * x_med + sv * xo
            ridge_z = 0.0034 * (1.0 - abs(sv - 0.62) / 0.62) * (1.0 - 0.55 * max(0.0, (t - 0.64) / 0.36))
            max_g[i, j] = (x_val, yv, (1.0 - sv) * zl + sv * zh + ridge_z)

    # Cohesive faceted maxillary philtrum & Cupid's-bow upper-lip armor inside the nasolabial fold.
    ph_u, ph_v = np.array([-1.0, -0.62, -0.20, 0.20, 0.62, 1.0]), np.linspace(0.0, 1.0, 4)
    ph_g = np.array([[[(0.0218 - 0.0074 * float(v)) * float(u),
                       0.0540 + 0.0080 * float(v),
                       0.1008 - 0.0094 * (float(u) ** 2) - (0.0009 if abs(float(u)) < 0.22 else 0.0)]
                      for v in ph_v] for u in ph_u])

    lp_u = np.array([-1.0, -0.66, -0.26, 0.0, 0.26, 0.66, 1.0])
    lp_v = np.array([0.0, 0.46, 1.0])
    lip_g = np.array([[[(0.0222 - 0.0006 * float(v)) * float(u),
                        0.0476 + 0.0058 * float(v) - 0.0020 * (float(u) ** 2) + (0.0008 if abs(abs(float(u)) - 0.26) < 0.05 and float(v) > 0.8 else 0.0),
                        0.1022 - 0.0098 * (float(u) ** 2) + (0.0014 if abs(float(v) - 0.46) < 0.1 else -0.0004 * float(v))]
                       for v in lp_v] for u in lp_u])

    l_pil = shell_patch(pil_g, 0.0022, 0.00055, SHELL_CF)
    nasal = shell_patch(nose_g, 0.0026, 0.00060, SHELL_CF)
    l_max = fh.honeycomb_patch(max_g, nc=3, nr=1, wall=0.0026, margin=(0.26, 0.34), fill=0.64,
                               col=SHELL_CF, hole_col=CHASSIS_CF)
    phil, u_lip = shell_patch(ph_g, 0.0024, 0.00050, SHELL_CF), shell_patch(lip_g, 0.0024, 0.00050, SHELL_CF)
    return {'cranium_frontal': front, 'cranium_dome': dome, 'cranium_occipital': occip,
            'left_temporal': l_temp, 'right_temporal': mirror_x(l_temp),
            'left_mastoid': l_mast, 'right_mastoid': mirror_x(l_mast),
            'left_orbital_pillar': l_pil, 'right_orbital_pillar': mirror_x(l_pil), 'nasal_bridge': nasal,
            'left_maxilla': l_max, 'right_maxilla': mirror_x(l_max), 'philtrum': phil, 'upper_lip': u_lip}


@functools.lru_cache(maxsize=None)
def ear_microphones():
    """Build concentric circular binaural transducer rings (`x_max ~ 70.2 mm`) flush on the slimmed temples."""
    ye, ze = 0.093, 0.004
    outer = rev([(0.0156, 0.0615), (0.0156, 0.0692), (0.0192, 0.0702), (0.0206, 0.0686),
                 (0.0206, 0.0610), (0.0188, 0.0598), (0.0156, 0.0615)], 0, (ye, ze), 48, SHELL_CF)
    inner_t = rev([(0.0102, 0.0640), (0.0102, 0.0688), (0.0128, 0.0694), (0.0138, 0.0682),
                   (0.0138, 0.0640)], 0, (ye, ze), 40, TITANIUM)
    accent = rev([(0.0138, 0.0668), (0.0154, 0.0676), (0.0154, 0.0652), (0.0138, 0.0652)], 0, (ye, ze), 44, MIC_GOLD)
    ports = [(ye + 0.0068 * math.cos(math.radians(60.0 * k)), ze + 0.0068 * math.sin(math.radians(60.0 * k)))
             for k in range(6)]
    holes = [csg.circle2d(0.0014, fn=12, at=c) for c in ports] + [csg.circle2d(0.0032, fn=20, at=(ye, ze))]
    baffle = csg.extrude(csg.circle2d(0.0155, fn=40, at=(ye, ze)), holes=holes,
                         a=0.0645, b=0.0662, axis=0, colour=BAFFLE_DARK, chamfer=0.0003).tris()
    core = rev([(0.0, 0.0645), (0.0030, 0.0645), (0.0030, 0.0678), (0.0018, 0.0684), (0.0, 0.0684)], 0, (ye, ze), 28, MIC_GOLD)
    left = outer + inner_t + accent + baffle + core
    return {'left_ear_mic': left, 'right_ear_mic': mirror_x(left)}


@functools.lru_cache(maxsize=None)
def camera_and_gimbal_local():
    """Return modern recessed orbital camera, sapphire lens bezel and 2-axis titanium gimbal ring."""
    return fh.modern_optics_local()


@functools.lru_cache(maxsize=None)
def moving_rest():
    """Build faceted brow strips, shutter eyelids, lip-corner tabs, and V-tapered 3-panel mandible with air seams."""
    from machine.eyes import gimbal, stereo
    np, ex = np_mod(), 0.5 * float(stereo.BASELINE_M)
    br_u, br_v = np.array([0.0, 0.28, 0.65, 1.0]), np.array([0.0, 0.56, 1.0])
    brow_g = np.zeros((len(br_u), len(br_v), 3))
    for i, u in enumerate(br_u):
        t = float(u)
        x = (ex - 0.0185) + 0.0282 * t
        arch = 0.0022 * math.sin(math.pi * min(1.0, (t + 0.04) * 0.96))
        yl, yh = 0.1098 + arch - 0.0016 * t, 0.1146 + arch - 0.0016 * t
        zf = 0.0984 - 0.0044 * t - 0.0080 * (max(0.0, (t - 0.36) / 0.64) ** 1.25)
        for j, v in enumerate(br_v):
            sv = float(v)
            z_bev = 0.0010 if abs(sv - 0.56) < 0.08 else (-0.0008 if sv < 0.1 else -0.0014)
            brow_g[i, j] = (x, yl + sv * (yh - yl), zf + z_bev)

    piv = (np.array([ex, float(gimbal.EYES[1]), float(gimbal.EYES[2])])
           - gimbal.mounted('left') @ np.array([0.0, 0.0, -float(gimbal.LEVER['left'])]))
    lx_u = np.linspace(float(piv[0]) - 0.0112, float(piv[0]) + 0.0112, 11)

    def _lid_grid(a0, a1):
        angs = np.linspace(math.radians(a0), math.radians(a1), 5)
        g = np.zeros((len(lx_u), len(angs), 3))
        for i, xv in enumerate(lx_u):
            rc = math.sqrt(max(36e-6, 0.0116 ** 2 - (float(xv) - float(piv[0])) ** 2))
            for j, a in enumerate(angs):
                g[i, j] = (float(xv), float(piv[1]) + rc * math.sin(float(a)), float(piv[2]) + rc * math.cos(float(a)))
        return g

    cm_u, cm_v = np.linspace(0.0, 1.0, 5), np.linspace(0.0, 1.0, 4)
    comm_g = np.array([[[0.0230 + 0.0066 * float(u),
                         0.0478 + 0.0010 * float(u) + (2.0 * float(v) - 1.0) * 0.0011,
                         0.0910 - 0.0090 * float(u)]
                        for v in cm_v] for u in cm_u])

    # Sculpted-faceted lower-lip, mentolabial crease, mental pad & tucked submental return.
    ch_u = np.array([-1.0, -0.62, -0.24, 0.0, 0.24, 0.62, 1.0])
    ch_v = np.array([0.0, 0.20, 0.56, 0.84, 1.0])
    yz_p = ((0.0132, -0.0108), (0.0196, 0.0018), (0.0342, -0.0020), (0.0422, 0.0020), (0.0448, 0.0002))
    chin_g = np.zeros((len(ch_u), len(ch_v), 3))
    for i, u in enumerate(ch_u):
        fu, au = float(u), abs(float(u))
        for j, (sv, (yb, zp)) in enumerate(zip(ch_v, yz_p)):
            hw = 0.0114 + 0.0068 * float(sv)
            x = hw * fu
            yv = yb + (0.0026 * (au ** 1.4) if j <= 1 else -0.0014 * (au ** 2))
            z_arch = 0.0998 - 0.0082 * ((x / 0.0185) ** 2)
            chin_g[i, j] = (x, yv, z_arch + zp)

    # Angled lateral mandibular armor plates separated from V-chin by crisp 3.5 mm diagonal airflow seams.
    lj_u, lj_v = np.linspace(0.0, 1.0, 8), np.array([0.0, 0.46, 1.0])
    ljaw_g = np.zeros((len(lj_u), len(lj_v), 3))
    for i, u in enumerate(lj_u):
        t = float(u)
        for j, v in enumerate(lj_v):
            sv = float(v)
            x_in = 0.0154 + 0.0064 * sv
            x = x_in + (0.0422 - x_in) * t
            yl, yh = 0.0156 + 0.0198 * (t ** 1.18), 0.0392 + 0.0074 * (t ** 1.18)
            zl, zh = 0.0916 - 0.0316 * (t ** 1.22), 0.0932 - 0.0324 * (t ** 1.22)
            crease = 0.0016 if abs(sv - 0.46) < 0.05 else 0.0
            ljaw_g[i, j] = (x, (1.0 - sv) * yl + sv * yh, (1.0 - sv) * zl + sv * zh + crease)

    ryz = csg.filleted2d([(0.028, 0.062), (0.043, 0.065), (0.068, 0.028), (0.068, 0.020), (0.060, 0.020), (0.029, 0.053)], r=0.0020)
    ram = (csg.extrude(ryz, a=0.0398, b=0.0434, axis=0, colour=CHASSIS_CF, chamfer=0.0006).tris()
           + csg.extrude(csg.circle2d(0.0036, fn=20, at=(0.066, 0.024)), a=0.0434, b=0.0468, axis=0,
                         colour=TITANIUM, chamfer=0.0004).tris())
    yoke = csg.extrude(csg.filleted2d(csg.box2d((-0.026, 0.026), (0.018, 0.025)), r=0.0016),
                       a=0.074, b=0.079, axis=2, colour=CHASSIS_CF, chamfer=0.0004).tris()
    lj_s = shell_patch(ljaw_g, 0.0026, 0.00055, SHELL_CF)
    jaw_all = shell_patch(chin_g, 0.0028, 0.00060, SHELL_CF) + lj_s + mirror_x(lj_s) + yoke + ram + mirror_x(ram)
    return (shell_patch(brow_g, 0.0022, 0.00048, SHELL_CF),
            shell_patch(_lid_grid(20.0, 46.0), 0.0013, 0.00030, SHELL_CF, piv),
            shell_patch(_lid_grid(-48.0, -24.0), 0.0013, 0.00030, SHELL_CF, piv),
            piv, shell_patch(comm_g, 0.0020, 0.00036, SHELL_CF), jaw_all)
