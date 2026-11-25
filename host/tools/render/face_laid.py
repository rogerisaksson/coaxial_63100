"""Flat unsculpted angular faceted-honeycomb feminine mecha face panels without mouth."""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg, face_honeycomb as fh, knee_loads as kl, neck_laid as nl  # noqa: E402
from tools.render.face_honeycomb import (
    BAFFLE_DARK, CHASSIS_CF, MESH_PEARL, PANEL_RIM, SEAM_SILVER, SHELL_CF, TITANIUM,
    mirror_x, np_mod, shell_patch,
)

_PROF_Y = (-0.0088, 0.004, 0.022, 0.042, 0.060, 0.076, 0.090, 0.104, 0.118, 0.138, 0.156)
_PROF_X = (0.0230, 0.0382, 0.0502, 0.0595, 0.0666, 0.0702, 0.0712, 0.0708, 0.0698, 0.0716, 0.0722)
_PROF_ZF = (0.0830, 0.0942, 0.1012, 0.1058, 0.1070, 0.1078, 0.1074, 0.1062, 0.1066, 0.1018, 0.0928)
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
        u = min(1.0, max(0.0, (y - 0.156) / 0.0724))
        c = math.sqrt(max(1e-6, 1.0 - u * u))
        return 0.0722 * c, -0.001 + 0.0938 * c, -0.001 - 0.0930 * c
    return _pchip(y, _PROF_Y, _PROF_X), _pchip(y, _PROF_Y, _PROF_ZF), _pchip(y, _PROF_Y, _PROF_ZB)


def _head_pt(th_rad, y, radial_off=0.0):
    """Evaluate unsculpted geometric ovoid anchor point at azimuth `th_rad` and height `y`."""
    xw, zf, zb = _profile_at(y)
    zc, rz = 0.5 * (zf + zb), max(0.004, 0.5 * (zf - zb))
    x = xw * math.sin(th_rad)
    z = zc + rz * math.cos(th_rad)
    if radial_off != 0.0:
        r_xz = math.hypot(x, z - zc)
        if r_xz > 1e-6:
            x += radial_off * (x / r_xz)
            z += radial_off * ((z - zc) / r_xz)
    return (x, y, z)


def _surf_grid(th_fn, y_fn, nu, nv, off=0.0):
    np = np_mod()
    us, vs = np.linspace(0.0, 1.0, nu), np.linspace(0.0, 1.0, nv)
    g = np.zeros((nu, nv, 3))
    for i, u in enumerate(us):
        fu = float(u)
        for j, v in enumerate(vs):
            fv = float(v)
            g[i, j] = _head_pt(float(th_fn(fu, fv)), float(y_fn(fu, fv)), off)
    return g


def _dome_ab(alpha, beta, off=0.0):
    """Evaluate upper-cranium ellipsoid anchor point at sagittal angle `alpha` and lateral angle `beta`."""
    sx, sy, sz = math.sin(beta), math.cos(beta) * math.cos(alpha), math.cos(beta) * math.sin(alpha)
    x, y, z = 0.0722 * sx, 0.156 + 0.0724 * sy, -0.001 + (0.0938 if sz >= 0.0 else 0.0930) * sz
    if off != 0.0:
        n = math.sqrt((x / 0.0722) ** 2 + (sy ** 2) + ((z + 0.001) / 0.0934) ** 2)
        if n > 1e-6:
            x += off * sx / n
            y += off * sy / n
            z += off * sz / n
    return (max(0.0030, x), y, z)


def _dome_b_seam(al, pad=0.0036):
    """Solve lateral angle `beta` where the cranium dome meets `_crown_seam_y(th) + pad`."""
    lo, hi = 0.12, 1.50
    for _ in range(12):
        mid = 0.5 * (lo + hi)
        px, py, pz = _dome_ab(al, mid, 0.0)
        if py > _crown_seam_y(math.atan2(px, pz + 0.001)) + pad:
            lo = mid
        else:
            hi = mid
    return lo


def _dome_strip(u0, u1, v0, v1, nu=12, nv=6, off=0.0):
    np = np_mod()
    us, vs = np.linspace(u0, u1, nu), np.linspace(v0, v1, nv)
    g = np.zeros((nu, nv, 3))
    a_f, a_b, b_min = 1.44, -1.20, 0.048
    for i, u in enumerate(us):
        al = a_f + float(u) * (a_b - a_f)
        b_max = max(b_min + 0.12, _dome_b_seam(al, 0.0032))
        for j, v in enumerate(vs):
            g[i, j] = _dome_ab(al, b_min + float(v) * (b_max - b_min), off)
    return g


def _crown_seam_y(th):
    return 0.1635 + 0.0085 * (min(1.0, abs(th) / 0.92) ** 1.10)


def _flat(pts, wall=0.0024, rim_m=0.0048, pitch=0.0078, fill=0.66, corner_c=0.0014, shift_n=0.0026, bar_col=MESH_PEARL):
    """Build a 100% flat angular ceramic frame enclosing coarse 6-sided honeycomb shifted outward by `shift_n`."""
    return fh.planar_honeycomb_panel(pts, wall=wall, chamfer=0.00074, rim_m=rim_m, pitch=pitch, fill=fill,
                                     corner_c=corner_c, col=SHELL_CF, bar_col=bar_col, hole_col=BAFFLE_DARK,
                                     rim_col=SEAM_SILVER, shift_n=shift_n)


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
    """Return recessed matte-dark inner shadow skull, satin ocular housings and machined titanium seam rails."""
    np = np_mod()
    glab_b = csg.extrude(csg.filleted2d(csg.box2d((-0.014, 0.014), (0.086, 0.118)), r=0.002),
                         a=0.072, b=0.086, axis=2, colour=BAFFLE_DARK, chamfer=0.0004).tris()
    cowl_o = [((0.032 + 0.0176 * math.cos(a)), (0.100 + 0.0168 * math.sin(a))) for a in (2.0 * math.pi * k / 28 for k in range(28))]
    cowl_i = [((0.032 + 0.0154 * math.cos(a)), (0.100 + 0.0148 * math.sin(a))) for a in (2.0 * math.pi * k / 28 for k in range(28))]
    cowl_l = csg.extrude(cowl_o, holes=[cowl_i], a=0.066, b=0.075, axis=2, colour=BAFFLE_DARK, chamfer=0.0004).tris()
    liner = []
    for t0, ts, y0, ys, nu, nv, of in (
        (0.000, 2.27, 0.116, 0.066, 8, 12, -0.0080),
        (0.70, 0.62, 0.088, 0.034, 6, 7, -0.0082),
        (0.00, 1.48, 0.052, 0.036, 6, 10, -0.0102),
        (1.98, math.pi - 2.00, 0.034, 0.144, 8, 8, -0.0054),
        (1.18, 0.82, 0.035, 0.083, 8, 7, -0.0056),
    ):
        s_p = shell_patch(_surf_grid(lambda u, v, a=t0, b=ts: a + v * b, lambda u, v, c=y0, d=ys: c + u * d,
                                     nu, nv, off=of), 0.0012, 0.00022, BAFFLE_DARK, rim_col=BAFFLE_DARK)
        liner += s_p + mirror_x(s_p)
    s_d = shell_patch(_dome_strip(0.00, 0.99, 0.00, 0.97, nu=12, nv=6, off=-0.0082), 0.0012, 0.00022, BAFFLE_DARK, rim_col=BAFFLE_DARK)
    # Machined satin-titanium structural sub-frame rails visible inside the ~7 mm panel channels.
    rail_m = shell_patch(_surf_grid(lambda u, v: 0.014 * (2.0 * v - 1.0), lambda u, v: 0.117 + u * 0.050,
                                    8, 4, off=-0.0038), 0.0010, 0.00022, TITANIUM, rim_col=PANEL_RIM)
    rail_fr = shell_patch(_surf_grid(lambda u, v: 0.04 + u * 0.72, lambda u, v: (0.1332 + 0.0154 * u) + v * 0.0022,
                                     8, 3, off=-0.0044), 0.0009, 0.00020, TITANIUM, rim_col=PANEL_RIM)
    rail_c = shell_patch(_surf_grid(lambda u, v: 0.18 + u * 0.96, lambda u, v: (0.0605 + 0.0195 * u) + v * 0.0024,
                                    8, 3, off=-0.0058), 0.0009, 0.00020, TITANIUM, rim_col=PANEL_RIM)
    rail_chin = shell_patch(_surf_grid(lambda u, v: 0.012 * (2.0 * v - 1.0), lambda u, v: 0.011 + u * 0.045,
                                       8, 3, off=-0.0054), 0.0009, 0.00020, TITANIUM, rim_col=PANEL_RIM)
    oral_g = np.array([[[0.026 * float(u), 0.014 + 0.048 * float(v), 0.068 - 0.014 * float(u * u) - 0.003 * (1.0 - float(v))]
                        for v in np.linspace(0.0, 1.0, 5)] for u in np.linspace(-1.0, 1.0, 9)])
    oral = shell_patch(oral_g, 0.0018, 0.00030, BAFFLE_DARK, (0.0, 0.042, 0.010), BAFFLE_DARK)
    return (glab_b + cowl_l + mirror_x(cowl_l) + oral + liner + rail_m + rail_chin
            + rail_fr + mirror_x(rail_fr) + rail_c + mirror_x(rail_c) + s_d + mirror_x(s_d))


@functools.lru_cache(maxsize=None)
def compute_heatsink():
    """Return recessed cranial compute fins and heatpipes inside the cranial bay."""
    return fh.compute_heatsink()


@functools.lru_cache(maxsize=None)
def static_plates():
    """Build completely flat unsculpted feminine ceramic-faceted coarse-honeycomb plates with ~7 mm channels."""
    p = _head_pt
    seam = _crown_seam_y

    # Forehead: TWO steeply tapered winged V-tiara flat ceramic plates per side framing an open brow shelf.
    f_lo = _flat([p(0.026, 0.1178), p(0.73, 0.1220), p(0.76, 0.1460), p(0.026, 0.1306)],
                 wall=0.0023, rim_m=0.0040, pitch=0.0076, fill=0.65, corner_c=0.0014, shift_n=0.0030)
    f_hi = _flat([p(0.026, 0.1376), p(0.765, 0.1528), p(0.790, seam(0.790) - 0.0034), p(0.026, seam(0.026) - 0.0034)],
                 wall=0.0023, rim_m=0.0046, pitch=0.0078, fill=0.66, corner_c=0.0015, shift_n=0.0028)
    front = f_lo + mirror_x(f_lo) + f_hi + mirror_x(f_hi)

    # Cranium dome: TWO harmonized lateral tiers x THREE sagittal facets seated on `_crown_seam_y + 3.6 mm`.
    def _d_facet(a0, a1, b_i0, b_o0, b_i1, b_o1, sh):
        return _flat([_dome_ab(a0, b_i0), _dome_ab(a0, b_o0), _dome_ab(a1, b_o1), _dome_ab(a1, b_i1)],
                     wall=0.0023, rim_m=0.0048, pitch=0.0084, fill=0.66, corner_c=0.0015, shift_n=sh)

    b_a0, b_a1, b_a2, b_a3 = _dome_b_seam(1.30), _dome_b_seam(0.50), _dome_b_seam(-0.36), _dome_b_seam(-1.12)
    d_lobe = (
        _d_facet(1.32, 0.54, 0.040, 0.50, 0.040, 0.52, 0.0028)
        + _d_facet(0.45, -0.32, 0.040, 0.52, 0.040, 0.52, 0.0028)
        + _d_facet(-0.41, -1.12, 0.040, 0.52, 0.040, 0.48, 0.0026)
        + _d_facet(1.30, 0.54, 0.59, b_a0, 0.61, b_a1, 0.0024)
        + _d_facet(0.45, -0.32, 0.61, b_a1, 0.61, b_a2, 0.0024)
        + _d_facet(-0.41, -1.12, 0.61, b_a2, 0.59, b_a3, 0.0024)
    )

    # Occipital: FOUR harmonized flat facets per side aligned to `_crown_seam_y - 3.4 mm`.
    o_lobe = []
    for t0, t1 in ((2.30, 2.68), (2.77, math.pi - 0.038)):
        o_lobe += _flat([p(t0, 0.095), p(t1, 0.095), p(t1, seam(t1) - 0.0034), p(t0, seam(t0) - 0.0034)],
                        wall=0.0023, rim_m=0.0048, pitch=0.0084, fill=0.66, corner_c=0.0015, shift_n=0.0022)
        o_lobe += _flat([p(t0, 0.020), p(t1, 0.020), p(t1, 0.087), p(t0, 0.087)],
                        wall=0.0023, rim_m=0.0048, pitch=0.0084, fill=0.66, corner_c=0.0015, shift_n=0.0022)

    # Temporal: TWO harmonized angled side-temple facets seated cleanly along `_crown_seam_y - 3.4 mm`.
    l_temp = (
        _flat([p(0.86, 0.119), p(1.48, 0.121), p(1.48, seam(1.48) - 0.0034), p(0.86, seam(0.86) - 0.0034)],
              wall=0.0023, rim_m=0.0048, pitch=0.0084, fill=0.66, corner_c=0.0015, shift_n=0.0022)
        + _flat([p(1.57, 0.121), p(2.21, 0.119), p(2.21, seam(2.21) - 0.0034), p(1.57, seam(1.57) - 0.0034)],
                wall=0.0023, rim_m=0.0048, pitch=0.0084, fill=0.66, corner_c=0.0015, shift_n=0.0022)
    )
    l_mast = _flat([p(1.78, 0.022), p(2.21, 0.022), p(2.21, 0.110), p(1.88, 0.070)],
                   wall=0.0022, rim_m=0.0044, pitch=0.0080, fill=0.65, corner_c=0.0014, shift_n=0.0020)
    # Orbital pillar: frames lateral eye corner cleanly (`0.64`) and spans back toward ear-mic (`1.31`).
    l_pil = _flat([p(0.64, 0.0970), p(1.31, 0.100), p(1.31, 0.114), p(0.66, 0.114)],
                  wall=0.0021, rim_m=0.0030, pitch=0.0064, fill=0.63, corner_c=0.0012, shift_n=0.0030)

    # Nose: ONE slim delicate flat angular hexagonal ceramic shield (~16 mm max width, coarse 6-sided mesh).
    l_nose = _flat([(0.0064, 0.1115, 0.1072), (-0.0064, 0.1115, 0.1072), (-0.0082, 0.0765, 0.1140),
                    (-0.0048, 0.0655, 0.1105), (0.0048, 0.0655, 0.1105), (0.0082, 0.0765, 0.1140)],
                   wall=0.0022, rim_m=0.0022, pitch=0.0052, fill=0.63, corner_c=0.0010, shift_n=0.0008, bar_col=(234, 238, 246))

    # Cheekbone: ONE flat swept angular feminine cheek wing lifted cleanly under the lower eyelid.
    l_max = _flat([p(0.125, 0.0630), p(1.15, 0.0835), p(1.17, 0.0965), p(0.600, 0.0932), p(0.138, 0.0916)],
                  wall=0.0023, rim_m=0.0048, pitch=0.0078, fill=0.66, corner_c=0.0014, shift_n=0.0048)

    # Recessed dark inner baffle plates behind the nose-to-lower-cheek gap (no external mouth).
    ph_p = shell_patch(_surf_grid(lambda u, v: 0.085 * (2.0 * u - 1.0), lambda u, v: 0.0525 + v * 0.0085,
                                  8, 5, off=-0.0115), 0.0014, 0.00022, BAFFLE_DARK, rim_col=BAFFLE_DARK)
    lip_p = shell_patch(_surf_grid(lambda u, v: 0.095 * (2.0 * u - 1.0), lambda u, v: 0.0435 + v * 0.0065,
                                   8, 5, off=-0.0130), 0.0014, 0.00022, BAFFLE_DARK, rim_col=BAFFLE_DARK)
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
    """Build pearl-ceramic arched brows on an open shelf, eyelids, internal smile lug and two sleek lower-cheek plates."""
    from machine.eyes import gimbal, stereo
    np, ex, p = np_mod(), 0.5 * float(stereo.BASELINE_M), _head_pt
    brow_g = _surf_grid(
        lambda u, v: 0.240 + u * 0.375,
        lambda u, v: (0.1095 + 0.0030 * math.sin(math.pi * (0.20 + 0.72 * u))) + v * (0.0028 * (1.0 - 0.25 * u)),
        11, 3, off=0.0068)
    piv = (np.array([ex, float(gimbal.EYES[1]), float(gimbal.EYES[2])])
           - gimbal.mounted('left') @ np.array([0.0, 0.0, -float(gimbal.LEVER['left'])]))
    lx_u = np.linspace(float(piv[0]) - 0.0113, float(piv[0]) + 0.0113, 13)

    def _lid_grid(a0, a1, tilt_deg=5.0):
        angs = np.linspace(math.radians(a0), math.radians(a1), 5)
        g = np.zeros((len(lx_u), len(angs), 3))
        for i, xv in enumerate(lx_u):
            dx = float(xv) - float(piv[0])
            rc = math.sqrt(max(44e-6, 0.0122 ** 2 - dx * dx))
            da, z_tuck = math.radians(tilt_deg) * (dx / 0.0113), -0.0004 * (((dx / 0.0113) ** 2) ** 1.15)
            for j, a in enumerate(angs):
                af = float(a) + da
                g[i, j] = (float(xv), float(piv[1]) + rc * math.sin(af), float(piv[2]) + rc * math.cos(af) + z_tuck)
        return g

    # Recessed internal cheek-actuator lug driven by smile linkage (invisible externally, no mouth).
    comm_g = _surf_grid(lambda u, v: 0.270 + u * 0.055, lambda u, v: 0.0430 + 0.0006 * u + v * 0.0040, 6, 4, off=-0.0148)

    # Lower cheek: TWO compact tucked flat angular ceramic shields meeting across a ~7.5 mm midline gap, NO mouth.
    lj_pts = [p(0.020, 0.0072), p(1.06, 0.0345), p(1.16, 0.0765), p(0.020, 0.0595)]
    lj_panel = _flat(lj_pts, wall=0.0023, rim_m=0.0050, pitch=0.0080, fill=0.66, corner_c=0.0022, shift_n=0.0038)
    lj_back = fh.planar_backing(lj_pts, wall=0.0012, inset_m=0.0026, corner_c=0.0022, col=BAFFLE_DARK, shift_n=0.0004)
    lj_ribs = [shell_patch(_surf_grid(lambda u, v: 0.14 + 0.80 * u,
                                      lambda u, v, dy=dy0: (0.0510 + 0.0175 * u) + dy + 0.0012 * v,
                                      10, 3, off=-0.0068), 0.0009, 0.00018, TITANIUM, rim_col=PANEL_RIM)
               for dy0 in (0.0000, 0.0028)]
    sub_ch = np.array([[[(0.014 + 0.011 * float(v)) * float(u), 0.0195 + 0.014 * float(v),
                         0.067 - 0.015 * float(v) - 0.010 * (float(u) ** 2)]
                        for v in np.linspace(0.0, 1.0, 4)] for u in np.linspace(-1.0, 1.0, 7)])
    under_jaw = shell_patch(sub_ch, 0.0018, 0.00032, PANEL_RIM, (0.0, 0.055, 0.055), BAFFLE_DARK)
    ryz = csg.filleted2d([(0.041, 0.058), (0.050, 0.062), (0.068, 0.028), (0.068, 0.020), (0.060, 0.020), (0.040, 0.050)], r=0.0020)
    ram = (csg.extrude(ryz, a=0.0398, b=0.0434, axis=0, colour=CHASSIS_CF, chamfer=0.0006).tris()
           + csg.extrude(csg.circle2d(0.0036, fn=20, at=(0.066, 0.024)), a=0.0434, b=0.0468, axis=0, colour=TITANIUM, chamfer=0.0004).tris())
    yoke = csg.extrude(csg.filleted2d(csg.box2d((-0.024, 0.024), (0.026, 0.032)), r=0.0016), a=0.066, b=0.071, axis=2, colour=CHASSIS_CF, chamfer=0.0004).tris()
    lj_s = lj_panel + lj_back + lj_ribs[0] + lj_ribs[1]
    jaw_all = under_jaw + lj_s + mirror_x(lj_s) + yoke + ram + mirror_x(ram)
    ulid = shell_patch(_lid_grid(14.0, 38.0, 5.0), 0.0010, 0.00022, BAFFLE_DARK, piv, BAFFLE_DARK)
    llid = shell_patch(_lid_grid(-30.5, -18.8, 3.2), 0.0010, 0.00022, SEAM_SILVER, piv, PANEL_RIM)
    return shell_patch(brow_g, 0.0013, 0.00028, SHELL_CF, rim_col=PANEL_RIM), ulid, llid, piv, shell_patch(comm_g, 0.0014, 0.00028, BAFFLE_DARK, rim_col=BAFFLE_DARK), jaw_all
