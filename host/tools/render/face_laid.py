"""Parametric CSG solids for the coaxial humanoid head, gimbals, ear mics and facial plates."""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg, knee_loads as kl, neck_laid as nl  # noqa: E402

SHELL_CF, CHASSIS_CF, TITANIUM, ANODIZED = (218, 222, 230), (36, 40, 48), (172, 178, 188), (226, 48, 178)
LIP_ROSE, SERVO_CASE, HORN_BRASS, NECK_SKIN = (166, 116, 110), (30, 34, 42), (208, 170, 82), (176, 180, 188)
OPTIC_DARK, OPTIC_GLASS, BAFFLE_DARK, MIC_GOLD = (10, 14, 22), (26, 62, 108), (18, 21, 28), (212, 176, 84)

SKULL_YC, SKULL_ZC, SKULL_RY = 0.1465, 0.0020, 0.0825


def np_mod():
    """Return `numpy` with `OPENBLAS_NUM_THREADS` capped first."""
    os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
    import numpy as np
    return np


@functools.lru_cache(maxsize=None)
def anchors():
    """Return `(neck_torso, head_rel, head_torso, y_carrier, r_carrier)` in m."""
    from machine.body import figure
    seg = {s[0]: s for s in figure.SEGMENTS}
    nt, hr = tuple(float(v) for v in seg['neck'][3]), tuple(float(v) for v in seg['head'][3])
    ht, lay = (nt[0] + hr[0], nt[1] + hr[1], nt[2] + hr[2]), nl.laid()
    return nt, hr, ht, float(lay['head']['carrier'][1] - ht[1]), float(lay['head']['gear'])


def rot(axis, rad):
    """Return 3x3 rotation matrix for `rad` around `'x'`, `'y'` or `'z'`."""
    np, c, s = np_mod(), math.cos(rad), math.sin(rad)
    if axis == 'x':
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    if axis == 'y':
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def turn_tris(tris, rot_col, pivot=(0.0, 0.0, 0.0), shift=(0.0, 0.0, 0.0)):
    """Rotate `tris` around `pivot` by column-axis matrix `rot_col` and add `shift`."""
    np = np_mod()
    piv, off, mat = np.asarray(pivot, float), np.asarray(shift, float), np.asarray(rot_col, float).T
    return [((t - piv) @ mat + piv + off, c) for t, c in tris]


def mirror_x(tris):
    """Mirror `tris` across `x = 0` while preserving outward face winding."""
    return [((t * (-1.0, 1.0, 1.0))[::-1].copy(), c) for t, c in tris]


def _rev(rz, axis=2, at=(0.0, 0.0), fn=36, col=TITANIUM):
    return csg.revolve_profile(rz, axis=axis, at=at, fn=fn, colour=col).tris()


def shell_patch(grid, wall=0.0030, chamfer=0.00045, col=SHELL_CF, ref=(0.0, 0.115, 0.020)):
    """Build a watertight chamfered 3D doubly-curved shell solid from `(Nu, Nv, 3)` outer vertices."""
    np = np_mod()
    g = np.asarray(grid, float).copy()
    nu, nv = g.shape[:2]
    du, dv = np.zeros_like(g), np.zeros_like(g)
    du[0], du[-1], du[1:-1] = g[1] - g[0], g[-1] - g[-2], 0.5 * (g[2:] - g[:-2])
    dv[:, 0], dv[:, -1], dv[:, 1:-1] = g[:, 1] - g[:, 0], g[:, -1] - g[:, -2], 0.5 * (g[:, 2:] - g[:, :-2])
    nrm = np.cross(du, dv)
    nrm /= np.maximum(np.linalg.norm(nrm, axis=-1, keepdims=True), 1e-12)
    if float(np.sum(nrm * (g - np.asarray(ref, float)))) < 0.0:
        g, nrm = g[:, ::-1].copy(), -nrm[:, ::-1].copy()
        du[0], du[-1], du[1:-1] = g[1] - g[0], g[-1] - g[-2], 0.5 * (g[2:] - g[:-2])
        dv[:, 0], dv[:, -1], dv[:, 1:-1] = g[:, 1] - g[:, 0], g[:, -1] - g[:, -2], 0.5 * (g[:, 2:] - g[:, :-2])
    su, sv = np.maximum(np.linalg.norm(du, axis=-1, keepdims=True), 1e-12), np.maximum(np.linalg.norm(dv, axis=-1, keepdims=True), 1e-12)
    ch, ins = min(float(chamfer), 0.36 * float(wall)), np.zeros_like(g)
    ins[0] += np.minimum(ch, 0.30 * su[0]) * (du[0] / su[0])
    ins[-1] -= np.minimum(ch, 0.30 * su[-1]) * (du[-1] / su[-1])
    ins[:, 0] += np.minimum(ch, 0.30 * sv[:, 0]) * (dv[:, 0] / sv[:, 0])
    ins[:, -1] -= np.minimum(ch, 0.30 * sv[:, -1]) * (dv[:, -1] / sv[:, -1])
    outer, inner, co_g, ci_g = g + ins, g - float(wall) * nrm + ins, g - ch * nrm, g - (float(wall) - ch) * nrm
    tris = []
    for i in range(nu - 1):
        for j in range(nv - 1):
            a, b, c, d = outer[i, j], outer[i + 1, j], outer[i + 1, j + 1], outer[i, j + 1]
            qa, qb, qc, qd = inner[i, j], inner[i + 1, j], inner[i + 1, j + 1], inner[i, j + 1]
            tris += [(np.array([a, b, c]), col), (np.array([a, c, d]), col),
                     (np.array([qa, qc, qb]), col), (np.array([qa, qd, qc]), col)]
    bidx = ([(i, 0) for i in range(nu - 1)] + [(nu - 1, j) for j in range(nv - 1)]
            + [(i, nv - 1) for i in range(nu - 1, 0, -1)] + [(0, j) for j in range(nv - 1, 0, -1)])
    rings, nb = [np.array([m[ij] for ij in bidx]) for m in (outer, co_g, ci_g, inner)], len(bidx)
    for ra, rb in zip(rings[:-1], rings[1:]):
        for k in range(nb):
            kn = (k + 1) % nb
            tris += [(np.array([ra[k], rb[k], rb[kn]]), col), (np.array([ra[k], rb[kn], ra[kn]]), col)]
    return tris


def rod(p0, p1, r=0.00095, col=TITANIUM):
    """Build chamfered titanium pushrod with brass clevis collars between `p0` and `p1`."""
    np = np_mod()
    a, b = np.asarray(p0, float), np.asarray(p1, float)
    ln = float(np.linalg.norm(b - a))
    if ln < 0.003:
        return []
    u, c2, c3 = (b - a) / ln, csg.circle2d(r, fn=14), csg.circle2d(1.55 * r, fn=14)
    return (csg.sweep(c2, [a + u * s for s in (0.0, 0.0020, 0.5 * ln, ln - 0.0020, ln)], chamfer_ends=0.35 * r, colour=col).tris()
            + csg.sweep(c3, [a, a + 0.0018 * u], chamfer_ends=0.3 * r, colour=HORN_BRASS).tris()
            + csg.sweep(c3, [b - 0.0018 * u, b], chamfer_ends=0.3 * r, colour=HORN_BRASS).tris())


def micro_servo(at, axis, ang, hlen=0.0064, hdir=(0.0, 0.0, 1.0), dual=False, sgn=1.0):
    """Build chamfered miniature servo body at `at`, spline on `axis`, and brass horn at `ang` rad."""
    np = np_mod()
    cx, cy, cz = (float(v) for v in at)
    hx, hy, hz = (0.0060, 0.0052, 0.0056) if axis == 'x' else (
        (0.0052, 0.0060, 0.0056) if axis == 'y' else (0.0052, 0.0056, 0.0060))
    ai = {'x': 0, 'y': 1, 'z': 2}[axis]
    ur, vr = ((-hy, hy), (-hz, hz)) if ai == 0 else (((-hz, hz), (-hx, hx)) if ai == 1 else ((-hx, hx), (-hy, hy)))
    lo, hi = (cx - hx, cy - hy, cz - hz)[ai], (cx + hx, cy + hy, cz + hz)[ai]
    off = (0.0, cy, cz) if ai == 0 else ((cx, 0.0, cz) if ai == 1 else (cx, cy, 0.0))
    body = csg.extrude(csg.filleted2d(csg.box2d(ur, vr), r=0.0011, n=4), a=lo, b=hi, axis=ai,
                       colour=SERVO_CASE, chamfer=0.0006).translate(*off).tris()
    ears = csg.filleted2d(csg.box2d((ur[0] - 0.0020, ur[1] + 0.0020), (vr[0] * 0.45, vr[1] * 0.45)), r=0.0005)
    mid = 0.5 * (lo + hi)
    body += csg.extrude(ears, a=mid - 0.0010, b=mid + 0.0010, axis=ai, colour=ANODIZED, chamfer=0.0003).translate(*off).tris()
    sp = np.array([cx, cy, cz], float)
    sp[ai] = (hi + 0.0015) if sgn >= 0.0 else (lo - 0.0015)
    hd = np.asarray(hdir, float)
    hd[ai] = 0.0
    hd = rot(axis, ang) @ (hd / max(float(np.linalg.norm(hd)), 1e-12))
    t1, t0 = sp + float(hlen) * hd, sp - (float(hlen) if dual else 0.0016) * hd
    horn = csg.sweep(csg.filleted2d(csg.box2d((-0.0015, 0.0015), (-0.0008, 0.0008)), r=0.00038),
                     [t0, sp, t1], chamfer_ends=0.0003, colour=HORN_BRASS).tris()
    return body, horn, t1, t0


@functools.lru_cache(maxsize=None)
def chassis_static():
    """Build dark carbon chassis, forward orbital socket mask, oral cavity baffle and ear struts."""
    np = np_mod()
    y_car, (cnt, c_mm) = anchors()[3], kl.HUB
    rb, yt = 1e-3 * float(c_mm), y_car + 0.0068
    holes = [csg.circle2d(0.013, fn=32)] + [csg.circle2d(0.0053, fn=16, at=(rb * math.cos(a), rb * math.sin(a)))
                                            for k in range(cnt) for a in (2.0 * math.pi * (k + 0.5) / cnt,)]
    flange = csg.extrude(csg.circle2d(0.042, fn=48), holes=holes, a=y_car + 0.0005, b=y_car + 0.0066,
                         axis=1, colour=CHASSIS_CF, chamfer=0.0007)
    pockets = [csg.filleted2d(csg.box2d(u, v), r=0.002)
               for u, v in (((-0.014, 0.014), (0.082, 0.090)), ((-0.038, -0.020), (0.082, 0.092)),
                            ((0.020, 0.038), (0.082, 0.092)))]
    bulk = csg.extrude(csg.filleted2d([(-0.044, yt), (0.044, yt), (0.048, 0.142), (-0.048, 0.142)], r=0.004),
                       holes=pockets, a=0.024, b=0.033, axis=2, colour=CHASSIS_CF, chamfer=0.0008)
    fore = csg.extrude(csg.filleted2d(csg.box2d((0.032, 0.058), (-0.045, 0.045)), r=0.003),
                       a=0.135, b=0.140, axis=1, colour=CHASSIS_CF, chamfer=0.0006)
    deck = csg.extrude(csg.filleted2d(csg.box2d((0.040, 0.054), (-0.042, 0.042)), r=0.003),
                       a=0.057, b=0.062, axis=1, colour=CHASSIS_CF, chamfer=0.0006)
    wyz = csg.filleted2d([(0.057, 0.026), (0.057, 0.052), (yt + 0.006, 0.033), (yt + 0.006, 0.024)], r=0.002)
    wl, wr = (csg.extrude(wyz, a=a, b=b, axis=0, colour=CHASSIS_CF, chamfer=0.0006)
              for a, b in ((0.031, 0.037), (-0.037, -0.031)))
    back_s = csg.extrude(csg.filleted2d(csg.box2d((-0.058, 0.058), (0.084, 0.120)), r=0.005),
                         a=0.033, b=0.036, axis=2, colour=CHASSIS_CF, chamfer=0.0005)
    ports = [csg.filleted2d(csg.box2d(u, v), r=0.0014)
             for u, v in (((-0.044, -0.031), (0.098, 0.105)), ((0.031, 0.044), (0.098, 0.105)),
                          ((-0.063, -0.045), (0.105, 0.116)), ((0.045, 0.063), (0.105, 0.116)))]
    eye_m = csg.extrude(csg.filleted2d(csg.box2d((-0.067, 0.067), (0.087, 0.118)), r=0.004),
                        holes=ports, a=0.069, b=0.072, axis=2, colour=CHASSIS_CF, chamfer=0.0005)
    oral_g = np.array([[[0.0270 * float(u), 0.0260 + 0.0270 * float(v),
                         0.0745 - 0.0135 * float(u * u) - 0.0030 * (1.0 - float(v))]
                        for v in np.linspace(0.0, 1.0, 6)] for u in np.linspace(-1.0, 1.0, 13)])
    oral = shell_patch(oral_g, 0.0024, 0.00036, BAFFLE_DARK, (0.0, 0.045, 0.010))
    ear = csg.sweep(csg.circle2d(0.0044, fn=20), [(0.042, 0.093, 0.026), (0.053, 0.093, 0.012), (0.062, 0.093, 0.006)],
                    chamfer_ends=0.0006, colour=CHASSIS_CF).tris()
    return csg.union(flange, bulk, fore, deck, wl, wr, back_s, eye_m).tris() + oral + ear + mirror_x(ear)


def _skull_x_from_yz(y, z):
    yn = (y - SKULL_YC) / (SKULL_RY if y >= SKULL_YC else 0.0760)
    zn = (z - SKULL_ZC) / (0.0968 if z >= SKULL_ZC else 0.0925)
    rx = 0.0724 - 0.0018 * (max(0.0, zn) ** 1.6) - 0.0032 * max(0.0, -zn) ** 2 - 0.0034 * max(0.0, -yn) ** 2
    denom = max(0.16, 1.0 - 0.72 * (max(0.0, zn) ** 2))
    return min(0.0724, rx * math.sqrt(max(0.03, (1.0 - min(0.95, yn * yn + zn * zn)) / denom)))


@functools.lru_cache(maxsize=None)
def static_plates():
    """Build smooth ovoid skull dome, ear-cradling temporal shells, nose, cheeks, philtrum and upper lip."""
    np = np_mod()
    us, vs = np.linspace(0.0, 1.0, 34), np.linspace(-1.0, 1.0, 21)
    dome_g = np.zeros((len(us), len(vs), 3))
    for i, u in enumerate(us):
        fu = float(u)
        for j, v in enumerate(vs):
            x, av = 0.0376 * float(v), abs(float(v))
            th0 = -0.285 + 0.012 * av
            th = th0 + fu * (4.00 - th0)
            sy, cz = math.sin(th), math.cos(th)
            rx = 0.0724 - 0.0018 * (max(0.0, cz) ** 1.6) - 0.0032 * max(0.0, -cz) ** 2
            ux2 = min(0.88, (x / rx) ** 2)
            r = math.sqrt(max(0.05, (1.0 - ux2) / max(0.16, 1.0 - 0.72 * ux2 * (max(0.0, cz) ** 2))))
            ry = SKULL_RY if (sy >= 0.0 or cz >= 0.0) else 0.0760
            rz = (0.0968 + 0.0014 * math.exp(-((th - 0.28) / 0.38) ** 2)) if cz >= 0.0 else 0.0925
            dome_g[i, j] = (x, SKULL_YC + ry * r * sy, SKULL_ZC + rz * r * cz)

    ts_u, ts_v = np.linspace(0.0, 1.0, 34), np.linspace(0.0, 1.0, 16)
    temp_g = np.zeros((len(ts_u), len(ts_v), 3))
    for i, u in enumerate(ts_u):
        fu = float(u)
        th = -0.268 + fu * 4.148
        sy, cz = math.sin(th), math.cos(th)
        rx0 = 0.0724 - 0.0018 * (max(0.0, cz) ** 1.6) - 0.0032 * max(0.0, -cz) ** 2
        ux2 = (0.0412 / rx0) ** 2
        r_top = math.sqrt((1.0 - ux2) / max(0.16, 1.0 - 0.72 * ux2 * (max(0.0, cz) ** 2)))
        ry0 = SKULL_RY if (sy >= 0.0 or cz >= 0.0) else 0.0760
        rz0 = 0.0968 if cz >= 0.0 else 0.0925
        yt, zt = SKULL_YC + ry0 * r_top * sy, SKULL_ZC + rz0 * r_top * cz
        zb = 0.076 - 0.136 * fu - 0.016 * math.sin(math.pi * fu)
        yb_base = 0.1218 - 0.0410 * (fu ** 0.85)
        dz_e = zb - 0.006
        ear_arc = (0.093 + math.sqrt(max(0.0, 0.0226 ** 2 - dz_e * dz_e))) if abs(dz_e) < 0.0226 else 0.072
        yb = 0.5 * (yb_base + ear_arc + math.hypot(yb_base - ear_arc, 0.0036))
        for j, v in enumerate(ts_v):
            wv = float(v)
            yv, zv = (1.0 - wv) * yt + wv * yb, (1.0 - wv) * zt + wv * zb
            temp_g[i, j] = (_skull_x_from_yz(yv, zv), yv, zv)

    pil_u, pil_v = np.linspace(0.0, 1.0, 11), np.linspace(0.0, 1.0, 7)
    pil_g = np.array([[[0.0706 - 0.0004 * (2.0 * float(u) - 1.0) ** 2 - 0.0004 * float(v),
                        0.0916 + 0.0210 * float(u),
                        0.0815 - 0.0125 * float(v) + 0.0014 * math.sin(math.pi * float(u))]
                       for v in pil_v] for u in pil_u])

    ns_u, ns_v = np.linspace(0.0, 1.0, 24), np.linspace(-1.0, 1.0, 15)
    nose_g = np.zeros((len(ns_u), len(ns_v), 3))
    for i, u in enumerate(ns_u):
        fu = float(u)
        yv = 0.0638 + 0.0580 * fu - 0.0012 * (fu ** 4)
        tip, sub = math.exp(-((fu - 0.13) / 0.11) ** 2), math.exp(-((fu - 0.01) / 0.06) ** 2)
        bridge = math.sin(math.pi * min(1.0, max(0.0, (fu - 0.12) / 0.88)))
        zc = 0.1010 - 0.0052 * fu + 0.0108 * tip - 0.0032 * sub - 0.0016 * bridge
        ala = math.exp(-((fu - 0.09) / 0.10) ** 2)
        hw, drop = 0.0072 - 0.0010 * bridge + 0.0046 * ala + 0.0014 * (fu ** 4), 0.0080 + 0.0040 * ala - 0.0028 * fu
        for j, v in enumerate(ns_v):
            av = abs(float(v))
            nose_g[i, j] = (hw * float(v), yv - 0.0012 * tip * (av ** 2) + 0.0010 * (fu ** 3) * (1.0 - av * av), zc - drop * (av ** 1.75))

    mx_u, mx_v = np.linspace(0.0, 1.0, 24), np.linspace(0.0, 1.0, 14)
    max_g = np.zeros((len(mx_u), len(mx_v), 3))
    for i, u in enumerate(mx_u):
        t = float(u)
        socket, med_rise = math.exp(-((t - 0.76) / 0.28) ** 2), 0.0116 * math.exp(-(t / 0.36) ** 2)
        yh, yl = 0.0968 + med_rise - 0.0086 * socket - 0.0010 * t, 0.0496 + 0.0104 * (t ** 1.30)
        ua = max(0.0, (t - 0.50) / 0.50)
        zh = 0.0960 - 0.0050 * (t ** 1.2) - 0.0128 * ua * ua * (3.0 - 2.0 * ua)
        zl = 0.0954 - 0.0110 * (t ** 1.3) - 0.0215 * ua * ua * (3.0 - 2.0 * ua)
        malar = 0.0044 * math.sin(math.pi * min(1.0, t * 1.06))
        for j, v in enumerate(mx_v):
            sv = float(v)
            yv = (1.0 - sv) * yl + sv * yh
            xm = 0.0092 + 0.0052 * math.exp(-((yv - 0.0680) / 0.0080) ** 2) + 0.0122 * max(0.0, (0.0640 - yv) / 0.0144) ** 1.15
            xo = (1.0 - sv) * 0.0470 + sv * 0.0706
            max_g[i, j] = ((1.0 - t) * xm + t * xo, yv, (1.0 - sv) * zl + sv * zh + malar * math.sin(math.pi * sv))

    ph_u, ph_v = np.linspace(-1.0, 1.0, 17), np.linspace(0.0, 1.0, 7)
    ph_g = np.array([[[(0.0196 - 0.0068 * float(v)) * float(u),
                       0.0532 + 0.0096 * float(v) + 0.0005 * float(u * u),
                       0.1008 - (0.0052 - 0.0018 * float(v)) * float(u * u) - 0.0012 * math.exp(-((float(u)) / 0.24) ** 2) * (1.0 - 0.4 * float(v))]
                      for v in ph_v] for u in ph_u])

    lp_u, lp_v = np.linspace(-1.0, 1.0, 21), np.linspace(0.0, 1.0, 8)
    lip_g = np.zeros((len(lp_u), len(lp_v), 3))
    for i, u in enumerate(lp_u):
        au = abs(float(u))
        cupid = 0.0013 * math.exp(-((au - 0.26) / 0.18) ** 2) - 0.0007 * math.exp(-(au / 0.12) ** 2)
        yh, yl = 0.0526 + cupid - 0.0030 * (au ** 1.8), 0.0466 + 0.0007 * math.exp(-(au / 0.22) ** 2) + 0.0008 * (au ** 2)
        zb, full = 0.1022 - 0.0076 * (au ** 1.85), 0.0022 * (1.0 - 0.72 * (au ** 1.6))
        for j, v in enumerate(lp_v):
            sv = float(v)
            lip_g[i, j] = (0.0202 * float(u), yl + sv * (yh - yl), zb - 0.0012 * sv + full * math.sin(math.pi * sv))

    dome = shell_patch(dome_g, 0.0032, 0.00048, SHELL_CF)
    l_temp = shell_patch(temp_g, 0.0030, 0.00045, SHELL_CF)
    l_pil = shell_patch(pil_g, 0.0018, 0.00032, SHELL_CF)
    nasal = shell_patch(nose_g, 0.0028, 0.00042, SHELL_CF)
    l_max = shell_patch(max_g, 0.0030, 0.00045, SHELL_CF)
    phil = shell_patch(ph_g, 0.0026, 0.00038, SHELL_CF)
    u_lip = shell_patch(lip_g, 0.0028, 0.00040, LIP_ROSE)
    return {'cranium_dome': dome, 'left_temporal': l_temp, 'right_temporal': mirror_x(l_temp),
            'left_orbital_pillar': l_pil, 'right_orbital_pillar': mirror_x(l_pil), 'nasal_bridge': nasal,
            'left_maxilla': l_max, 'right_maxilla': mirror_x(l_max), 'philtrum': phil, 'upper_lip': u_lip}


@functools.lru_cache(maxsize=None)
def ear_microphones():
    """Build flush binaural temporal microphone housings (`x ~ +-70.8 mm`), baffles and MEMS arrays."""
    ye, ze = 0.093, 0.006
    cup = _rev([(0.0132, 0.0630), (0.0132, 0.0686), (0.0164, 0.0706), (0.0180, 0.0692),
                (0.0180, 0.0620), (0.0164, 0.0606), (0.0, 0.0606), (0.0, 0.0630)], 0, (ye, ze), 44, TITANIUM)
    cup += _rev([(0.0150, 0.0696), (0.0162, 0.0708), (0.0170, 0.0700), (0.0158, 0.0688)], 0, (ye, ze), 44, ANODIZED)
    caps = [(ye + 0.0062 * math.cos(a), ze + 0.0062 * math.sin(a)) for a in (0.523599, 2.617994, 4.712389)]
    ports = [(ye + 0.0104 * math.cos(math.radians(90.0 + 60.0 * k)),
              ze + 0.0104 * math.sin(math.radians(90.0 + 60.0 * k))) for k in range(6)] + [(ye, ze)]
    holes = [csg.circle2d(0.0034, fn=20, at=c) for c in caps] + [csg.circle2d(0.0010, fn=12, at=c) for c in ports]
    baffle = csg.extrude(csg.circle2d(0.0130, fn=40, at=(ye, ze)), holes=holes,
                         a=0.0668, b=0.0680, axis=0, colour=BAFFLE_DARK, chamfer=0.0003).tris()
    grz = [(0.0022, 0.0636), (0.0031, 0.0636), (0.0031, 0.0674), (0.0026, 0.0678), (0.0022, 0.0674)]
    drz = [(0.0, 0.0636), (0.0021, 0.0636), (0.0021, 0.0670), (0.0, 0.0672)]
    mems = [t for c in caps for t in (_rev(grz, 0, c, 24, TITANIUM) + _rev(drz, 0, c, 24, MIC_GOLD))]
    left = cup + baffle + mems
    return {'left_ear_mic': left, 'right_ear_mic': mirror_x(left)}


@functools.lru_cache(maxsize=None)
def camera_and_gimbal_local():
    """Build compact orbital camera barrel, stepped anodized bezel, sapphire/obsidian lens and gimbal ring."""
    cam = (_rev([(0.0, 0.0040), (0.0050, 0.0040), (0.0058, 0.0028), (0.0058, -0.0024),
                 (0.0044, -0.0024), (0.0044, -0.0010), (0.0, -0.0010)], col=CHASSIS_CF)
           + _rev([(0.0040, -0.0024), (0.0066, -0.0024), (0.0066, -0.0044), (0.0059, -0.0060),
                   (0.0049, -0.0066), (0.0038, -0.0054), (0.0040, -0.0038)], col=ANODIZED)
           + _rev([(0.0026, -0.0038), (0.0040, -0.0038), (0.0038, -0.0054), (0.0026, -0.0050)], fn=32, col=OPTIC_DARK)
           + _rev([(0.0013, -0.0030), (0.0029, -0.0030), (0.0029, -0.0046), (0.0019, -0.0053), (0.0013, -0.0055)], fn=32, col=OPTIC_GLASS)
           + _rev([(0.0, -0.0030), (0.0013, -0.0030), (0.0013, -0.0055), (0.0, -0.0057)], fn=24, col=OPTIC_DARK)
           + csg.extrude(csg.filleted2d(csg.box2d((-0.0011, 0.0011), (0.0052, 0.0074)), r=0.00038),
                         a=0.0008, b=0.0030, axis=2, colour=TITANIUM, chamfer=0.0003).tris())
    ring = _rev(csg.filleted2d([(0.0072, -0.0013), (0.0085, -0.0013), (0.0085, 0.0013), (0.0072, 0.0013)], r=0.00040, n=3), fn=40, col=TITANIUM)
    pins = (csg.extrude(csg.circle2d(0.0009, fn=14), a=-0.0098, b=-0.0083, axis=1, colour=TITANIUM).tris()
            + csg.extrude(csg.circle2d(0.0009, fn=14), a=0.0083, b=0.0098, axis=1, colour=TITANIUM).tris())
    return cam, ring, pins


@functools.lru_cache(maxsize=None)
def moving_rest():
    """Build rest-pose feminine brows, fuller curved eyelids, tucked commissure tabs, lower lip and mandible."""
    from machine.eyes import gimbal, stereo
    np = np_mod()
    br_u, br_v = np.linspace(0.0, 1.0, 20), np.linspace(0.0, 1.0, 7)
    brow_g = np.zeros((len(br_u), len(br_v), 3))
    for i, u in enumerate(br_u):
        t = float(u)
        x, arch = 0.0092 + 0.0612 * t, 0.0038 * math.sin(math.pi * (t ** 0.78))
        yl = 0.1135 + arch + 0.0004 * t
        yh, ua = yl + 0.0054 - 0.0012 * t, max(0.0, (t - 0.50) / 0.50)
        zf = 0.0978 - 0.0052 * (t ** 1.15) - 0.0132 * ua * ua * (3.0 - 2.0 * ua)
        for j, v in enumerate(br_v):
            sv = float(v)
            brow_g[i, j] = (x, yl + sv * (yh - yl), zf - 0.0014 * sv + 0.0011 * math.sin(math.pi * sv))

    piv = (np.array([0.5 * stereo.BASELINE_M, float(gimbal.EYES[1]), float(gimbal.EYES[2])])
           - gimbal.mounted('left') @ np.array([0.0, 0.0, -float(gimbal.LEVER['left'])]))
    lx_u = np.linspace(float(piv[0]) - 0.0108, float(piv[0]) + 0.0078, 16)

    def _lid_grid(a0, a1):
        angs = np.linspace(math.radians(a0), math.radians(a1), 8)
        g = np.zeros((len(lx_u), len(angs), 3))
        for i, xv in enumerate(lx_u):
            rc = math.sqrt(max(36e-6, 0.0114 ** 2 - (float(xv) - float(piv[0])) ** 2))
            for j, a in enumerate(angs):
                g[i, j] = (float(xv), float(piv[1]) + rc * math.sin(float(a)), float(piv[2]) + rc * math.cos(float(a)))
        return g

    cm_u, cm_v = np.linspace(0.0, 1.0, 9), np.linspace(0.0, 1.0, 5)
    comm_g = np.array([[[0.0206 + 0.0088 * float(u),
                         0.0458 + 0.0008 * float(u) + (2.0 * float(v) - 1.0) * (0.0015 - 0.0003 * float(u)),
                         0.0934 - 0.0108 * (float(u) ** 1.15) + 0.0006 * math.sin(math.pi * float(v))]
                        for v in cm_v] for u in cm_u])

    ll_u, ll_v = np.linspace(-1.0, 1.0, 19), np.linspace(0.0, 1.0, 8)
    llip_g = np.zeros((len(ll_u), len(ll_v), 3))
    for i, u in enumerate(ll_u):
        au = abs(float(u))
        yh, yl = 0.0454 + 0.0005 * (au ** 2), 0.0392 + 0.0044 * (au ** 1.65)
        zb, full = 0.1014 - 0.0076 * (au ** 1.80), 0.0026 * (1.0 - 0.75 * (au ** 1.5))
        for j, v in enumerate(ll_v):
            sv = float(v)
            llip_g[i, j] = (0.0196 * float(u), yl + sv * (yh - yl), zb - 0.0010 * (1.0 - sv) + full * math.sin(math.pi * sv))

    jw_u, jw_v = np.linspace(-1.0, 1.0, 31), np.linspace(0.0, 1.0, 10)
    jaw_g = np.zeros((len(jw_u), len(jw_v), 3))
    for i, u in enumerate(jw_u):
        au, x = abs(float(u)), 0.0402 * float(u)
        yl, yh = 0.0082 + 0.0240 * (au ** 1.55), 0.0382 + 0.0088 * (au ** 1.55)
        zl, zh = 0.0930 - 0.0345 * (au ** 1.65), 0.0985 - 0.0365 * (au ** 1.65)
        chin, sulcus = 0.0038 * max(0.0, 1.0 - 1.45 * (au ** 2)), -0.0014 * max(0.0, 1.0 - 2.2 * (au ** 2))
        for j, v in enumerate(jw_v):
            sv = float(v)
            jaw_g[i, j] = (x, yl + sv * (yh - yl), (1.0 - sv) * zl + sv * zh + chin * math.sin(math.pi * min(1.0, sv * 1.35)) + sulcus * (sv ** 3))

    ryz = csg.filleted2d([(0.032, 0.060), (0.046, 0.064), (0.068, 0.028), (0.068, 0.020), (0.061, 0.020), (0.033, 0.051)], r=0.0022)
    ram = (csg.extrude(ryz, a=0.0404, b=0.0442, axis=0, colour=CHASSIS_CF, chamfer=0.0006).tris()
           + csg.extrude(csg.circle2d(0.0036, fn=20, at=(0.066, 0.024)), a=0.0442, b=0.0476, axis=0, colour=TITANIUM, chamfer=0.0004).tris())
    jaw_all = shell_patch(jaw_g, 0.0032, 0.00045, SHELL_CF) + shell_patch(llip_g, 0.0028, 0.00040, LIP_ROSE) + ram + mirror_x(ram)
    return (shell_patch(brow_g, 0.0028, 0.00042, SHELL_CF),
            shell_patch(_lid_grid(23.0, 50.0), 0.0013, 0.00028, SHELL_CF, piv),
            shell_patch(_lid_grid(-49.0, -26.0), 0.0013, 0.00028, SHELL_CF, piv),
            piv, shell_patch(comm_g, 0.0020, 0.00030, TITANIUM), jaw_all)
