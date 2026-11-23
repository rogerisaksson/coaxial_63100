"""Parametric CSG solids for the coaxial humanoid head, gimbals, ear mics and facial plates."""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg, knee_loads as kl, neck_laid as nl  # noqa: E402

SHELL_CF, CHASSIS_CF, TITANIUM, ANODIZED = (84, 89, 99), (44, 47, 54), (172, 178, 188), (226, 48, 178)
SERVO_CASE, HORN_BRASS, NECK_SKIN = (34, 38, 46), (208, 170, 82), (154, 158, 166)
OPTIC_DARK, OPTIC_GLASS, BAFFLE_DARK, MIC_GOLD = (14, 18, 26), (34, 72, 114), (22, 25, 32), (212, 176, 84)


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


def _thick_curve(pts, wall):
    """Build fixed-count 2D C-strip `[p_0..p_{K-1}, cap_hi_0..2, q_{K-1}..q_0, cap_lo_0..2]`."""
    np = np_mod()
    p = np.asarray(pts, float)
    tg = np.zeros_like(p)
    tg[0], tg[-1], tg[1:-1] = p[1] - p[0], p[-1] - p[-2], p[2:] - p[:-2]
    tg /= np.maximum(np.linalg.norm(tg, axis=1, keepdims=True), 1e-12)
    inner = p - float(wall) * np.c_[tg[:, 1], -tg[:, 0]]
    rc = 0.5 * abs(float(wall))
    mh, vh = 0.5 * (p[-1] + inner[-1]), 0.5 * (p[-1] - inner[-1])
    ml, vl = 0.5 * (p[0] + inner[0]), 0.5 * (inner[0] - p[0])
    angs = (0.25 * math.pi, 0.50 * math.pi, 0.75 * math.pi)
    ch = [mh + math.cos(a) * vh + math.sin(a) * rc * tg[-1] for a in angs]
    cl = [ml + math.cos(a) * vl - math.sin(a) * rc * tg[0] for a in angs]
    return np.vstack([p, ch, inner[::-1], cl])


def plate_loft(z0, z1, n, fn, wall, axis=0, chamfer=0.00045, col=SHELL_CF):
    """Watertight smooth C-strip loft across `n` analytic stations over `[z0, z1]` along `axis`."""
    np = np_mod()
    zs = [float(v) for v in np.linspace(z0, z1, n)]
    raw = [_thick_curve(fn(z), wall) for z in zs]
    ccw = csg._signed_area(raw[0]) >= 0.0
    rings = [p if ccw else p[::-1].copy() for p in raw]
    if chamfer > 1e-6 and len(rings) >= 2:
        d0 = min(1.3 * chamfer, 0.4 * abs(zs[1] - zs[0]))
        d1 = min(1.3 * chamfer, 0.4 * abs(zs[-1] - zs[-2]))
        rings = [csg.inset2d(rings[0], d0 / 1.3, True)] + rings + [csg.inset2d(rings[-1], d1 / 1.3, True)]
        sd = math.copysign(1.0, zs[-1] - zs[0])
        zs = [zs[0], zs[0] + sd * d0] + zs[1:-1] + [zs[-1] - sd * d1, zs[-1]]
    r3d = [csg._lay2d(r, z, axis) for r, z in zip(rings, zs)]
    tris = [t for a, b in zip(r3d[:-1], r3d[1:]) for t in csg._quads(a, b, col)]
    nv = len(raw[0])
    k = (nv - 6) // 2
    q = lambda m: 2 * k + 2 - m
    ears = [(m, m + 1, q(m + 1)) for m in range(k - 1)] + [(m, q(m + 1), q(m)) for m in range(k - 1)]
    ears += [(k + 3, k - 1, k), (k + 3, k, k + 1), (k + 3, k + 1, k + 2)]
    ears += [(0, 2 * k + 2, 2 * k + 3), (0, 2 * k + 3, 2 * k + 4), (0, 2 * k + 4, 2 * k + 5)]
    if not ccw:
        ears = [(nv - 1 - i, nv - 1 - m, nv - 1 - j) for i, j, m in ears]
    sp = zs[-1] >= zs[0]
    for ei, flip in ((0, sp), (-1, not sp)):
        c3 = r3d[ei]
        tris += [(c3[[i, m, j] if flip else [i, j, m]], col) for i, j, m in ears]
    return tris


def rod(p0, p1, r=0.0010, col=TITANIUM):
    """Build chamfered titanium pushrod with brass clevis collars between `p0` and `p1`."""
    np = np_mod()
    a, b = np.asarray(p0, float), np.asarray(p1, float)
    ln = float(np.linalg.norm(b - a))
    if ln < 0.003:
        return []
    u = (b - a) / ln
    c2, c3 = csg.circle2d(r, fn=14), csg.circle2d(1.55 * r, fn=14)
    pts = [a + u * s for s in (0.0, 0.0022, 0.5 * ln, ln - 0.0022, ln)]
    return (csg.sweep(c2, pts, chamfer_ends=0.35 * r, colour=col).tris()
            + csg.sweep(c3, [a, a + 0.0020 * u], chamfer_ends=0.3 * r, colour=HORN_BRASS).tris()
            + csg.sweep(c3, [b - 0.0020 * u, b], chamfer_ends=0.3 * r, colour=HORN_BRASS).tris())


def micro_servo(at, axis, ang, hlen=0.0068, hdir=(0.0, 0.0, 1.0), dual=False, sgn=1.0):
    """Build chamfered miniature servo body at `at`, spline on `axis`, and brass horn at `ang` rad."""
    np = np_mod()
    cx, cy, cz = (float(v) for v in at)
    hx, hy, hz = (0.0065, 0.0055, 0.0060) if axis == 'x' else (
        (0.0055, 0.0065, 0.0060) if axis == 'y' else (0.0055, 0.0060, 0.0065))
    ai = {'x': 0, 'y': 1, 'z': 2}[axis]
    ur, vr = ((-hy, hy), (-hz, hz)) if ai == 0 else (((-hz, hz), (-hx, hx)) if ai == 1 else ((-hx, hx), (-hy, hy)))
    lo, hi = (cx - hx, cy - hy, cz - hz)[ai], (cx + hx, cy + hy, cz + hz)[ai]
    off = (0.0, cy, cz) if ai == 0 else ((cx, 0.0, cz) if ai == 1 else (cx, cy, 0.0))
    body = csg.extrude(csg.filleted2d(csg.box2d(ur, vr), r=0.0012, n=4), a=lo, b=hi, axis=ai,
                       colour=SERVO_CASE, chamfer=0.0007).translate(*off).tris()
    ears = csg.filleted2d(csg.box2d((ur[0] - 0.0022, ur[1] + 0.0022), (vr[0] * 0.45, vr[1] * 0.45)), r=0.0006)
    mid = 0.5 * (lo + hi)
    body += csg.extrude(ears, a=mid - 0.0011, b=mid + 0.0011, axis=ai, colour=ANODIZED, chamfer=0.0003).translate(*off).tris()
    sp = np.array([cx, cy, cz], float)
    sp[ai] = (hi + 0.0016) if sgn >= 0.0 else (lo - 0.0016)
    hd = np.asarray(hdir, float)
    hd[ai] = 0.0
    hd = rot(axis, ang) @ (hd / max(float(np.linalg.norm(hd)), 1e-12))
    t1, t0 = sp + float(hlen) * hd, sp - (float(hlen) if dual else 0.0018) * hd
    prof = csg.filleted2d(csg.box2d((-0.0016, 0.0016), (-0.0009, 0.0009)), r=0.0004)
    horn = csg.sweep(prof, [t0, sp, t1], chamfer_ends=0.0003, colour=HORN_BRASS).tris()
    return body, horn, t1, t0


@functools.lru_cache(maxsize=None)
def chassis_static():
    """Build cranial carbon chassis on `carrier` with 8 M5 hub counterbores and ear-mic struts."""
    y_car, (cnt, c_mm) = anchors()[3], kl.HUB
    rb, yt = 1e-3 * float(c_mm), y_car + 0.0070
    holes = [csg.circle2d(0.013, fn=32)] + [csg.circle2d(0.0053, fn=16, at=(rb * math.cos(a), rb * math.sin(a)))
                                            for k in range(cnt) for a in (2.0 * math.pi * (k + 0.5) / cnt,)]
    flange = csg.extrude(csg.circle2d(0.044, fn=48), holes=holes, a=y_car + 0.0005, b=y_car + 0.0068,
                         axis=1, colour=CHASSIS_CF, chamfer=0.0007)
    pockets = [csg.filleted2d(csg.box2d(u, v), r=0.002)
               for u, v in (((-0.016, 0.016), (0.082, 0.090)), ((-0.044, -0.024), (0.082, 0.092)),
                            ((0.024, 0.044), (0.082, 0.092)))]
    w_out = csg.filleted2d([(-0.052, yt), (0.052, yt), (0.060, 0.138), (-0.060, 0.138)], r=0.004)
    bulk = csg.extrude(w_out, holes=pockets, a=0.025, b=0.035, axis=2, colour=CHASSIS_CF, chamfer=0.0009)
    fore = csg.extrude(csg.filleted2d(csg.box2d((0.034, 0.064), (-0.058, 0.058)), r=0.003),
                       a=0.132, b=0.137, axis=1, colour=CHASSIS_CF, chamfer=0.0006)
    deck = csg.extrude(csg.filleted2d(csg.box2d((0.045, 0.058), (-0.048, 0.048)), r=0.003),
                       a=0.057, b=0.062, axis=1, colour=CHASSIS_CF, chamfer=0.0006)
    wyz = csg.filleted2d([(0.057, 0.028), (0.057, 0.054), (yt + 0.006, 0.035), (yt + 0.006, 0.025)], r=0.002)
    wl, wr = (csg.extrude(wyz, a=a, b=b, axis=0, colour=CHASSIS_CF, chamfer=0.0006)
              for a, b in ((0.044, 0.050), (-0.050, -0.044)))
    ear = csg.sweep(csg.circle2d(0.0048, fn=20), [(0.048, 0.090, 0.026), (0.060, 0.090, 0.012), (0.0695, 0.090, 0.006)],
                    chamfer_ends=0.0006, colour=CHASSIS_CF).tris()
    return csg.union(flange, bulk, fore, deck, wl, wr).tris() + ear + mirror_x(ear)


@functools.lru_cache(maxsize=None)
def static_plates():
    """Build smooth C1-lofted cranium dome, temporal & lateral orbital arches, nose, maxillae and upper lip."""
    np = np_mod()

    def _dome_fn(x):
        sc, u = math.sqrt(max(0.30, 1.0 - (x / 0.088) ** 2)), (abs(x) / 0.046) ** 2
        yf, zf, pts = 0.1380 - 0.0030 * u, 0.096 - 0.005 * u, []
        for s in np.linspace(0.0, 1.0, 22):
            a = 0.36 + float(s) * (math.pi - 0.44)
            y, z = 0.102 + 0.096 * sc * math.sin(a), 0.008 + 0.090 * sc * math.cos(a)
            if s < 0.16:
                w = (float(s) / 0.16) ** 2 * (3.0 - 2.0 * float(s) / 0.16)
                y, z = (1.0 - w) * yf + w * y, (1.0 - w) * zf + w * z
            pts.append((y, z))
        return pts + [(0.046 + 0.005 * u, -0.072 * sc)]

    def _temp_fn(x):
        t, sc = (x - 0.0485) / 0.0310, math.sqrt(max(0.28, 1.0 - (x / 0.088) ** 2))
        yf, zf, yb, pts = 0.1355 - 0.0095 * (t ** 1.4), 0.093 - 0.014 * (t ** 1.7), 0.056 + 0.058 * (t ** 1.25), []
        for s in np.linspace(0.0, 1.0, 18):
            a = 0.32 + float(s) * (math.pi - 0.74)
            y = max(0.102 + 0.095 * sc * math.sin(a), (1.0 - float(s)) * yf + float(s) * yb)
            z = 0.008 + 0.088 * sc * math.cos(a)
            if s < 0.18:
                w = (float(s) / 0.18) ** 2 * (3.0 - 2.0 * float(s) / 0.18)
                y, z = (1.0 - w) * yf + w * y, (1.0 - w) * zf + w * z
            pts.append((y, z))
        return pts

    def _nose_fn(yv):
        s = (yv - 0.066) / 0.0695
        tip = math.exp(-((s - 0.13) / 0.11) ** 2)
        zf = 0.102 - 0.005 * s + 0.010 * tip
        hw = 0.0135 - 0.0052 * math.sin(math.pi * min(1.0, s * 1.15)) + 0.0018 * tip
        return [(zf - 0.0060 * float(u * u), hw * float(u)) for u in np.linspace(-1.0, 1.0, 15)]

    def _max_fn(x):
        t = (x - 0.0165) / 0.0637
        yh = 0.0835 + 0.0095 * max(0.0, 1.0 - t / 0.42) ** 2 + 0.0020 * math.sin(math.pi * t)
        yl, ua = 0.0585 + 0.0160 * (t ** 1.55), max(0.0, (t - 0.65) / 0.35)
        zp = 0.096 - 0.010 * (t ** 1.3) - 0.042 * ua * ua * (3.0 - 2.0 * ua)
        bg = 0.0045 * math.sin(math.pi * min(1.0, t * 1.15))
        return [(yl + float(s) * (yh - yl), zp - 0.002 * (1.0 - float(s)) + bg * math.sin(math.pi * float(s)))
                for s in np.linspace(0.0, 1.0, 10)]

    pil_fn = lambda x: [(0.0860 + float(s) * 0.0255, 0.086 - 0.011 * (x - 0.0748) / 0.0067 - 0.0036 * (2.0 * float(s) - 1.0) ** 2)
                        for s in np.linspace(0.0, 1.0, 9)]
    lip_fn = lambda x: [(0.0460 + 0.0010 * (x / 0.0225) ** 2 + float(s) * 0.0108,
                         0.1025 - 0.0080 * (x / 0.0225) ** 2 - 0.0015 * float(s) + 0.0014 * math.sin(math.pi * float(s)))
                        for s in np.linspace(0.0, 1.0, 8)]

    dome = plate_loft(-0.046, 0.046, 29, _dome_fn, 0.0036, chamfer=0.0005)
    l_temp = plate_loft(0.0485, 0.0795, 19, _temp_fn, 0.0034)
    l_pil = plate_loft(0.0748, 0.0815, 9, pil_fn, -0.0034)
    nasal = plate_loft(0.066, 0.1355, 21, _nose_fn, -0.0034, axis=1)
    l_max = plate_loft(0.0165, 0.0802, 21, _max_fn, -0.0034)
    u_lip = plate_loft(-0.0225, 0.0225, 17, lip_fn, -0.0032)
    return {'cranium_dome': dome, 'left_temporal': l_temp, 'right_temporal': mirror_x(l_temp),
            'left_orbital_pillar': l_pil, 'right_orbital_pillar': mirror_x(l_pil), 'nasal_bridge': nasal,
            'left_maxilla': l_max, 'right_maxilla': mirror_x(l_max), 'upper_lip': u_lip}


@functools.lru_cache(maxsize=None)
def ear_microphones():
    """Build binaural temporal microphone housings, perforated baffles and three MEMS capsules per side."""
    ye, ze = 0.090, 0.006
    cup = _rev([(0.0142, 0.0715), (0.0142, 0.0772), (0.0176, 0.0792), (0.0195, 0.0776),
                (0.0195, 0.0702), (0.0178, 0.0688), (0.0, 0.0688), (0.0, 0.0715)], 0, (ye, ze), 44, TITANIUM)
    cup += _rev([(0.0160, 0.0782), (0.0174, 0.0794), (0.0182, 0.0786), (0.0168, 0.0774)], 0, (ye, ze), 44, ANODIZED)
    caps = [(ye + 0.0066 * math.cos(a), ze + 0.0066 * math.sin(a)) for a in (0.523599, 2.617994, 4.712389)]
    ports = [(ye + 0.0112 * math.cos(math.radians(90.0 + 60.0 * k)),
              ze + 0.0112 * math.sin(math.radians(90.0 + 60.0 * k))) for k in range(6)] + [(ye, ze)]
    holes = [csg.circle2d(0.0038, fn=20, at=c) for c in caps] + [csg.circle2d(0.0011, fn=12, at=c) for c in ports]
    baffle = csg.extrude(csg.circle2d(0.0140, fn=40, at=(ye, ze)), holes=holes,
                         a=0.0754, b=0.0766, axis=0, colour=BAFFLE_DARK, chamfer=0.0003).tris()
    grz = [(0.0024, 0.0720), (0.0034, 0.0720), (0.0034, 0.0760), (0.0028, 0.0764), (0.0024, 0.0760)]
    drz = [(0.0, 0.0720), (0.0023, 0.0720), (0.0023, 0.0756), (0.0, 0.0758)]
    mems = [t for c in caps for t in (_rev(grz, 0, c, 24, TITANIUM) + _rev(drz, 0, c, 24, MIC_GOLD))]
    left = cup + baffle + mems
    return {'left_ear_mic': left, 'right_ear_mic': mirror_x(left)}


@functools.lru_cache(maxsize=None)
def camera_and_gimbal_local():
    """Build local orbital camera barrel, stepped anodized bezel, multi-element lens and gimbal ring."""
    cam = (_rev([(0.0, 0.0048), (0.0072, 0.0048), (0.0084, 0.0036), (0.0084, -0.0028),
                 (0.0062, -0.0028), (0.0062, -0.0010), (0.0, -0.0010)], col=CHASSIS_CF)
           + _rev([(0.0056, -0.0028), (0.0094, -0.0028), (0.0094, -0.0054), (0.0084, -0.0074),
                   (0.0070, -0.0080), (0.0052, -0.0066), (0.0056, -0.0046)], col=ANODIZED)
           + _rev([(0.0038, -0.0044), (0.0056, -0.0044), (0.0052, -0.0066), (0.0038, -0.0060)], fn=32, col=OPTIC_DARK)
           + _rev([(0.0, -0.0034), (0.0042, -0.0034), (0.0042, -0.0054), (0.0026, -0.0062), (0.0, -0.0065)], fn=32, col=OPTIC_GLASS)
           + csg.extrude(csg.filleted2d(csg.box2d((-0.0014, 0.0014), (0.0074, 0.0102)), r=0.0005),
                         a=0.0010, b=0.0038, axis=2, colour=TITANIUM, chamfer=0.0004).tris())
    ring = _rev(csg.filleted2d([(0.0102, -0.0018), (0.0124, -0.0018), (0.0124, 0.0018), (0.0102, 0.0018)], r=0.0005, n=3), fn=40, col=TITANIUM)
    pins = (csg.extrude(csg.circle2d(0.0012, fn=14), a=-0.0140, b=-0.0118, axis=1, colour=TITANIUM).tris()
            + csg.extrude(csg.circle2d(0.0012, fn=14), a=0.0118, b=0.0140, axis=1, colour=TITANIUM).tris())
    return cam, ring, pins


@functools.lru_cache(maxsize=None)
def moving_rest():
    """Build rest-pose articulated brow, eyelids, lip-corner linkage and tapered V-oval mandible."""
    from machine.eyes import gimbal, stereo
    np = np_mod()

    def _brow_fn(x):
        t = (x - 0.0160) / 0.0642
        yl = 0.1190 + 0.0026 * math.sin(math.pi * t) - 0.0025 * t * t
        yh, ua = yl + 0.0086 - 0.0014 * t, max(0.0, (t - 0.65) / 0.35)
        zf = 0.0985 - 0.0055 * (t ** 1.2) - 0.0140 * ua * ua * (3.0 - 2.0 * ua)
        return [(yl + float(s) * (yh - yl), zf - 0.0018 * float(s) + 0.0014 * math.sin(math.pi * float(s)))
                for s in np.linspace(0.0, 1.0, 7)]

    piv = (np.array([0.5 * stereo.BASELINE_M, float(gimbal.EYES[1]), float(gimbal.EYES[2])])
           - gimbal.mounted('left') @ np.array([0.0, 0.0, -float(gimbal.LEVER['left'])]))
    lid_fn = lambda a0, a1: (lambda x: [(float(piv[1]) + math.sqrt(max(64e-6, 0.0148 ** 2 - (x - float(piv[0])) ** 2)) * math.sin(float(a)),
                                         float(piv[2]) + math.sqrt(max(64e-6, 0.0148 ** 2 - (x - float(piv[0])) ** 2)) * math.cos(float(a)))
                                        for a in np.linspace(math.radians(a0), math.radians(a1), 8)])
    comm_fn = lambda x: [(0.0478 + 0.0010 * (x - 0.025) / 0.018 + (2.0 * float(s) - 1.0) * (0.0030 - 0.0006 * (x - 0.025) / 0.018),
                          0.0932 - 0.0105 * ((x - 0.025) / 0.018) ** 1.3 + 0.0010 * math.sin(math.pi * float(s)))
                         for s in np.linspace(0.0, 1.0, 5)]

    def _jaw_fn(x):
        u2 = (x / 0.0455) ** 2
        yl, yh = 0.0045 + 0.0195 * (u2 ** 0.75), 0.0405 - 0.0025 * u2
        zl, zh, ch = 0.0890 - 0.0380 * (u2 ** 0.85), 0.0990 - 0.0390 * (u2 ** 0.85), 0.0040 * max(0.0, 1.0 - 1.3 * u2)
        return [(yl + float(s) * (yh - yl), (1.0 - float(s)) * zl + float(s) * zh + ch * math.sin(math.pi * float(s)))
                for s in np.linspace(0.0, 1.0, 9)]

    ryz = csg.filleted2d([(0.024, 0.050), (0.038, 0.060), (0.068, 0.028), (0.068, 0.020), (0.061, 0.020), (0.025, 0.041)], r=0.0025)
    ram = (csg.extrude(ryz, a=0.0458, b=0.0502, axis=0, colour=CHASSIS_CF, chamfer=0.0007).tris()
           + csg.extrude(csg.circle2d(0.0038, fn=20, at=(0.066, 0.024)), a=0.0502, b=0.0540, axis=0, colour=TITANIUM, chamfer=0.0005).tris())
    return (plate_loft(0.0160, 0.0802, 19, _brow_fn, -0.0034),
            plate_loft(float(piv[0]) - 0.0104, float(piv[0]) + 0.0108, 13, lid_fn(24.0, 44.0), 0.0016, chamfer=0.00035),
            plate_loft(float(piv[0]) - 0.0104, float(piv[0]) + 0.0108, 13, lid_fn(-48.0, -28.0), 0.0016, chamfer=0.00035),
            piv, plate_loft(0.0250, 0.0430, 11, comm_fn, -0.0028, chamfer=0.0004, col=TITANIUM),
            plate_loft(-0.0455, 0.0455, 31, _jaw_fn, -0.0038) + ram + mirror_x(ram))
