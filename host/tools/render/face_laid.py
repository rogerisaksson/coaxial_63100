"""Parametric ventilated honeycomb shells, compute core, optics, ear mics and facial plates."""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg, face_honeycomb as fh, knee_loads as kl, neck_laid as nl  # noqa: E402
from tools.render.face_honeycomb import (  # noqa: F401
    ANODIZED, BAFFLE_DARK, CHASSIS_CF, HORN_BRASS, LIP_ROSE, MIC_GOLD,
    SERVO_CASE, SHELL_CF, TITANIUM, mirror_x, np_mod, rev, rot, shell_patch,
)

SKULL_YC, SKULL_ZC, SKULL_RY = 0.1465, 0.0020, 0.0825


@functools.lru_cache(maxsize=None)
def anchors():
    """Return `(neck_torso, head_rel, head_torso, y_carrier, r_carrier)` in m."""
    from machine.body import figure
    seg = {s[0]: s for s in figure.SEGMENTS}
    nt, hr = tuple(float(v) for v in seg['neck'][3]), tuple(float(v) for v in seg['head'][3])
    ht, lay = (nt[0] + hr[0], nt[1] + hr[1], nt[2] + hr[2]), nl.laid()
    return nt, hr, ht, float(lay['head']['carrier'][1] - ht[1]), float(lay['head']['gear'])


def rod(p0, p1, r=0.00095, col=TITANIUM):
    """Build chamfered titanium pushrod with brass clevis collars between `p0` and `p1`."""
    np = np_mod()
    a, b = np.asarray(p0, float), np.asarray(p1, float)
    ln = float(np.linalg.norm(b - a))
    if ln < 0.003:
        return []
    u, c2, c3 = (b - a) / ln, csg.circle2d(r, fn=14), csg.circle2d(1.55 * r, fn=14)
    return (csg.sweep(c2, [a + u * s for s in (0.0, 0.0020, 0.5 * ln, ln - 0.0020, ln)],
                      chamfer_ends=0.35 * r, colour=col).tris()
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
    """Build dark carbon chassis, 64 mm IPD orbital bulkhead, oral cavity baffle and ear struts."""
    np = np_mod()
    y_car, (cnt, c_mm) = anchors()[3], kl.HUB
    rb, yt = 1e-3 * float(c_mm), y_car + 0.0068
    holes = [csg.circle2d(0.013, fn=32)] + [csg.circle2d(0.0053, fn=16, at=(rb * math.cos(a), rb * math.sin(a)))
                                            for k in range(cnt) for a in (2.0 * math.pi * (k + 0.5) / cnt,)]
    flange = csg.extrude(csg.circle2d(0.042, fn=48), holes=holes, a=y_car + 0.0005, b=y_car + 0.0066,
                         axis=1, colour=CHASSIS_CF, chamfer=0.0007)
    pockets = [csg.filleted2d(csg.box2d(u, v), r=0.002)
               for u, v in (((-0.014, 0.014), (0.082, 0.090)), ((-0.042, -0.020), (0.084, 0.114)),
                            ((0.020, 0.042), (0.084, 0.114)))]
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
    ports = [csg.filleted2d(csg.box2d(u, v), r=0.0040)
             for u, v in (((-0.050, -0.013), (0.087, 0.117)), ((0.013, 0.050), (0.087, 0.117)))]
    eye_m = csg.extrude(csg.filleted2d(csg.box2d((-0.058, 0.058), (0.084, 0.120)), r=0.005),
                        holes=ports, a=0.064, b=0.067, axis=2, colour=CHASSIS_CF, chamfer=0.0005)
    oral_g = np.array([[[0.0270 * float(u), 0.0260 + 0.0270 * float(v),
                         0.0745 - 0.0135 * float(u * u) - 0.0030 * (1.0 - float(v))]
                        for v in np.linspace(0.0, 1.0, 6)] for u in np.linspace(-1.0, 1.0, 13)])
    oral = shell_patch(oral_g, 0.0024, 0.00036, BAFFLE_DARK, (0.0, 0.045, 0.010))
    ear = csg.sweep(csg.circle2d(0.0044, fn=20), [(0.042, 0.093, 0.026), (0.053, 0.093, 0.012), (0.062, 0.093, 0.006)],
                    chamfer_ends=0.0006, colour=CHASSIS_CF).tris()
    return csg.union(flange, bulk, fore, deck, wl, wr, back_s, eye_m).tris() + oral + ear + mirror_x(ear)


@functools.lru_cache(maxsize=None)
def compute_heatsink():
    """Return recessed internal cranial and facial compute heatsink fins behind honeycomb vents."""
    return fh.compute_heatsink()


def _skull_x_from_yz(y, z):
    yn = (y - SKULL_YC) / (SKULL_RY if y >= SKULL_YC else 0.0760)
    zn = (z - SKULL_ZC) / (0.0968 if z >= SKULL_ZC else 0.0925)
    rx = 0.0724 - 0.0018 * (max(0.0, zn) ** 1.6) - 0.0032 * max(0.0, -zn) ** 2 - 0.0034 * max(0.0, -yn) ** 2
    denom = max(0.16, 1.0 - 0.72 * (max(0.0, zn) ** 2))
    return min(0.0724, rx * math.sqrt(max(0.03, (1.0 - min(0.95, yn * yn + zn * zn)) / denom)))


def _dome_grid(th0, th1, n_u=20, trim_brow=False):
    np = np_mod()
    us, vs = np.linspace(0.0, 1.0, n_u), np.linspace(-1.0, 1.0, 17)
    grid = np.zeros((len(us), len(vs), 3))
    for i, u in enumerate(us):
        fu = float(u)
        for j, v in enumerate(vs):
            x, av = 0.0376 * float(v), abs(float(v))
            start = (th0 + 0.014 * av) if trim_brow else th0
            th = start + fu * (th1 - start)
            sy, cz = math.sin(th), math.cos(th)
            rx = 0.0724 - 0.0018 * (max(0.0, cz) ** 1.6) - 0.0032 * max(0.0, -cz) ** 2
            ux2 = min(0.88, (x / rx) ** 2)
            r = math.sqrt(max(0.05, (1.0 - ux2) / max(0.16, 1.0 - 0.72 * ux2 * (max(0.0, cz) ** 2))))
            ry = SKULL_RY if (sy >= 0.0 or cz >= 0.0) else 0.0760
            rz = (0.0968 + 0.0014 * math.exp(-((th - 0.28) / 0.38) ** 2)) if cz >= 0.0 else 0.0925
            grid[i, j] = (x, SKULL_YC + ry * r * sy, SKULL_ZC + rz * r * cz)
    return grid


def _temporal_grid(th0, th1, n_u=20):
    np = np_mod()
    ts_u, ts_v = np.linspace(0.0, 1.0, n_u), np.linspace(0.0, 1.0, 14)
    grid = np.zeros((len(ts_u), len(ts_v), 3))
    for i, u in enumerate(ts_u):
        th = th0 + float(u) * (th1 - th0)
        fa = (th + 0.268) / 4.148
        sy, cz = math.sin(th), math.cos(th)
        rx0 = 0.0724 - 0.0018 * (max(0.0, cz) ** 1.6) - 0.0032 * max(0.0, -cz) ** 2
        ux2 = (0.0414 / rx0) ** 2
        r_top = math.sqrt((1.0 - ux2) / max(0.16, 1.0 - 0.72 * ux2 * (max(0.0, cz) ** 2)))
        ry0, rz0 = SKULL_RY if (sy >= 0.0 or cz >= 0.0) else 0.0760, 0.0968 if cz >= 0.0 else 0.0925
        yt, zt = SKULL_YC + ry0 * r_top * sy, SKULL_ZC + rz0 * r_top * cz
        zb = 0.076 - 0.136 * fa - 0.016 * math.sin(math.pi * fa)
        yb_base = 0.1218 - 0.0410 * (fa ** 0.85)
        dz_e = zb - 0.006
        ear_arc = (0.093 + math.sqrt(max(0.0, 0.0226 ** 2 - dz_e * dz_e))) if abs(dz_e) < 0.0226 else 0.072
        yb = 0.5 * (yb_base + ear_arc + math.hypot(yb_base - ear_arc, 0.0036))
        for j, v in enumerate(ts_v):
            wv = float(v)
            yv, zv = (1.0 - wv) * yt + wv * yb, (1.0 - wv) * zt + wv * zb
            grid[i, j] = (_skull_x_from_yz(yv, zv), yv, zv)
    return grid


@functools.lru_cache(maxsize=None)
def static_plates():
    """Build segmented dense honeycomb cranium/temporal/maxilla shells with cooling gaps, nose and lip."""
    np = np_mod()
    front = fh.honeycomb_patch(_dome_grid(-0.245, 0.88, 22, trim_brow=True), nc=12, nr=13, wall=0.0026,
                               margin=(0.10, 0.10), fill=0.62, col=SHELL_CF, hole_col=CHASSIS_CF)
    dome = fh.honeycomb_patch(_dome_grid(0.95, 2.38, 24), nc=12, nr=17, wall=0.0026,
                              margin=(0.10, 0.08), fill=0.62, col=SHELL_CF, hole_col=CHASSIS_CF)
    occip = fh.honeycomb_patch(_dome_grid(2.45, 4.00, 24), nc=12, nr=17, wall=0.0026,
                               margin=(0.10, 0.08), fill=0.62, col=SHELL_CF, hole_col=CHASSIS_CF)
    l_temp = fh.honeycomb_patch(_temporal_grid(-0.24, 2.36, 24), nc=9, nr=19, wall=0.0026,
                                margin=(0.11, 0.08), fill=0.62, col=SHELL_CF, hole_col=CHASSIS_CF)
    l_mast = fh.honeycomb_patch(_temporal_grid(2.44, 3.88, 20), nc=9, nr=13, wall=0.0026,
                                margin=(0.11, 0.09), fill=0.62, col=SHELL_CF, hole_col=CHASSIS_CF)

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
        hw, drop = 0.0064 - 0.0009 * bridge + 0.0040 * ala + 0.0012 * (fu ** 4), 0.0076 + 0.0036 * ala - 0.0024 * fu
        for j, v in enumerate(ns_v):
            av = abs(float(v))
            nose_g[i, j] = (hw * float(v), yv - 0.0012 * tip * (av ** 2) + 0.0010 * (fu ** 3) * (1.0 - av * av),
                            zc - drop * (av ** 1.75))

    mx_u, mx_v = np.linspace(0.0, 1.0, 22), np.linspace(0.0, 1.0, 14)
    max_g = np.zeros((len(mx_u), len(mx_v), 3))
    for i, u in enumerate(mx_u):
        t = float(u)
        # Sculpt lower orbital rim cleanly beneath the 64 mm IPD eye and blend laterally into pillar.
        socket = math.exp(-((t - 0.36) / 0.25) ** 2)
        med_rise = 0.0085 * math.exp(-(t / 0.26) ** 2)
        yh, yl = 0.0942 + med_rise - 0.0110 * socket - 0.0012 * t, 0.0496 + 0.0104 * (t ** 1.30)
        ua = max(0.0, (t - 0.50) / 0.50)
        zh = 0.0956 - 0.0052 * (t ** 1.2) - 0.0128 * ua * ua * (3.0 - 2.0 * ua)
        zl = 0.0952 - 0.0110 * (t ** 1.3) - 0.0215 * ua * ua * (3.0 - 2.0 * ua)
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

    l_pil = shell_patch(pil_g, 0.0018, 0.00032, SHELL_CF)
    nasal = shell_patch(nose_g, 0.0028, 0.00042, SHELL_CF)
    l_max = fh.honeycomb_patch(max_g, nc=10, nr=7, wall=0.0026, margin=(0.12, 0.13), fill=0.60,
                               col=SHELL_CF, hole_col=CHASSIS_CF)
    phil, u_lip = shell_patch(ph_g, 0.0026, 0.00038, SHELL_CF), shell_patch(lip_g, 0.0028, 0.00040, LIP_ROSE)
    return {'cranium_frontal': front, 'cranium_dome': dome, 'cranium_occipital': occip,
            'left_temporal': l_temp, 'right_temporal': mirror_x(l_temp),
            'left_mastoid': l_mast, 'right_mastoid': mirror_x(l_mast),
            'left_orbital_pillar': l_pil, 'right_orbital_pillar': mirror_x(l_pil), 'nasal_bridge': nasal,
            'left_maxilla': l_max, 'right_maxilla': mirror_x(l_max), 'philtrum': phil, 'upper_lip': u_lip}


@functools.lru_cache(maxsize=None)
def ear_microphones():
    """Build flush binaural temporal microphone housings (`x ~ +-70.8 mm`), baffles and MEMS arrays."""
    ye, ze = 0.093, 0.006
    cup = rev([(0.0132, 0.0630), (0.0132, 0.0686), (0.0164, 0.0706), (0.0180, 0.0692),
               (0.0180, 0.0620), (0.0164, 0.0606), (0.0, 0.0606), (0.0, 0.0630)], 0, (ye, ze), 44, TITANIUM)
    cup += rev([(0.0150, 0.0696), (0.0162, 0.0708), (0.0170, 0.0700), (0.0158, 0.0688)], 0, (ye, ze), 44, ANODIZED)
    caps = [(ye + 0.0062 * math.cos(a), ze + 0.0062 * math.sin(a)) for a in (0.523599, 2.617994, 4.712389)]
    ports = [(ye + 0.0104 * math.cos(math.radians(90.0 + 60.0 * k)),
              ze + 0.0104 * math.sin(math.radians(90.0 + 60.0 * k))) for k in range(6)] + [(ye, ze)]
    holes = [csg.circle2d(0.0034, fn=20, at=c) for c in caps] + [csg.circle2d(0.0010, fn=12, at=c) for c in ports]
    baffle = csg.extrude(csg.circle2d(0.0130, fn=40, at=(ye, ze)), holes=holes,
                         a=0.0668, b=0.0680, axis=0, colour=BAFFLE_DARK, chamfer=0.0003).tris()
    grz = [(0.0022, 0.0636), (0.0031, 0.0636), (0.0031, 0.0674), (0.0026, 0.0678), (0.0022, 0.0674)]
    drz = [(0.0, 0.0636), (0.0021, 0.0636), (0.0021, 0.0670), (0.0, 0.0672)]
    mems = [t for c in caps for t in (rev(grz, 0, c, 24, TITANIUM) + rev(drz, 0, c, 24, MIC_GOLD))]
    left = cup + baffle + mems
    return {'left_ear_mic': left, 'right_ear_mic': mirror_x(left)}


@functools.lru_cache(maxsize=None)
def camera_and_gimbal_local():
    """Return modern recessed orbital camera, sapphire lens bezel and 2-axis titanium gimbal ring."""
    return fh.modern_optics_local()


@functools.lru_cache(maxsize=None)
def moving_rest():
    """Build sleek arched brows, 64 mm IPD curved eyelids, commissure tabs, lower lip and mandible."""
    from machine.eyes import gimbal, stereo
    np, ex = np_mod(), 0.5 * float(stereo.BASELINE_M)
    br_u, br_v = np.linspace(0.0, 1.0, 20), np.linspace(0.0, 1.0, 7)
    brow_g = np.zeros((len(br_u), len(br_v), 3))
    for i, u in enumerate(br_u):
        t = float(u)
        x = (ex - 0.018) + 0.042 * t
        arch = 0.0032 * math.sin(math.pi * (t ** 0.78))
        yl, yh = 0.1145 + arch + 0.0004 * t, 0.1145 + arch + 0.0004 * t + 0.0038 - 0.0008 * t
        ua = max(0.0, (t - 0.55) / 0.45)
        zf = 0.0972 - 0.0044 * (t ** 1.15) - 0.0072 * ua * ua * (3.0 - 2.0 * ua)
        for j, v in enumerate(br_v):
            sv = float(v)
            brow_g[i, j] = (x, yl + sv * (yh - yl), zf - 0.0014 * sv + 0.0010 * math.sin(math.pi * sv))

    piv = (np.array([ex, float(gimbal.EYES[1]), float(gimbal.EYES[2])])
           - gimbal.mounted('left') @ np.array([0.0, 0.0, -float(gimbal.LEVER['left'])]))
    lx_u = np.linspace(float(piv[0]) - 0.0110, float(piv[0]) + 0.0110, 16)

    def _lid_grid(a0, a1):
        angs = np.linspace(math.radians(a0), math.radians(a1), 8)
        g = np.zeros((len(lx_u), len(angs), 3))
        for i, xv in enumerate(lx_u):
            rc = math.sqrt(max(36e-6, 0.0116 ** 2 - (float(xv) - float(piv[0])) ** 2))
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
            jaw_g[i, j] = (x, yl + sv * (yh - yl),
                           (1.0 - sv) * zl + sv * zh + chin * math.sin(math.pi * min(1.0, sv * 1.35)) + sulcus * (sv ** 3))

    ryz = csg.filleted2d([(0.032, 0.060), (0.046, 0.064), (0.068, 0.028), (0.068, 0.020), (0.061, 0.020), (0.033, 0.051)], r=0.0022)
    ram = (csg.extrude(ryz, a=0.0404, b=0.0442, axis=0, colour=CHASSIS_CF, chamfer=0.0006).tris()
           + csg.extrude(csg.circle2d(0.0036, fn=20, at=(0.066, 0.024)), a=0.0442, b=0.0476, axis=0,
                         colour=TITANIUM, chamfer=0.0004).tris())
    jaw_all = shell_patch(jaw_g, 0.0032, 0.00045, SHELL_CF) + shell_patch(llip_g, 0.0028, 0.00040, LIP_ROSE) + ram + mirror_x(ram)
    return (shell_patch(brow_g, 0.0024, 0.00038, SHELL_CF),
            shell_patch(_lid_grid(20.0, 46.0), 0.0013, 0.00028, SHELL_CF, piv),
            shell_patch(_lid_grid(-48.0, -24.0), 0.0013, 0.00028, SHELL_CF, piv),
            piv, shell_patch(comm_g, 0.0020, 0.00030, TITANIUM), jaw_all)
