"""Flat angular faceted honeycomb armor panels, bevel shells, heatsink fins and eye optics."""
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg  # noqa: E402

SHELL_CF, PANEL_RIM, SEAM_SILVER, CHASSIS_CF, TITANIUM = (242, 245, 250), (62, 68, 80), (196, 202, 212), (24, 28, 36), (176, 182, 194)
ANODIZED, SERVO_CASE, HORN_BRASS, MESH_PEARL = (226, 48, 178), (28, 32, 40), (206, 168, 84), (228, 232, 240)
NECK_SKIN, OPTIC_DARK, OPTIC_GLASS, OPTIC_SCLERA = (48, 52, 62), (10, 14, 22), (42, 168, 248), (234, 238, 246)
BAFFLE_DARK, MIC_GOLD, BROW_DARK = (16, 19, 26), (218, 182, 92), (44, 48, 58)


def np_mod():
    """Return `numpy` with `OPENBLAS_NUM_THREADS` capped first."""
    os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
    import numpy as np
    return np


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


def rev(rz, axis=2, at=(0.0, 0.0), fn=36, col=TITANIUM):
    """Build revolved solid triangles from `(r, z)` profile via `csg.revolve_profile`."""
    return csg.revolve_profile(rz, axis=axis, at=at, fn=fn, colour=col).tris()


def _surface_frame(grid, wall, chamfer, ref):
    """Orient `(Nu, Nv, 3)` shell grid outward from `ref` and return `(g, du, dv, nrm, ch)`."""
    np = np_mod()
    g = np.asarray(grid, float).copy()
    for _ in range(2):
        du, dv = np.zeros_like(g), np.zeros_like(g)
        du[0], du[-1], du[1:-1] = g[1] - g[0], g[-1] - g[-2], 0.5 * (g[2:] - g[:-2])
        dv[:, 0], dv[:, -1], dv[:, 1:-1] = g[:, 1] - g[:, 0], g[:, -1] - g[:, -2], 0.5 * (g[:, 2:] - g[:, :-2])
        nrm = np.cross(du, dv)
        nrm /= np.maximum(np.linalg.norm(nrm, axis=-1, keepdims=True), 1e-12)
        if float(np.sum(nrm * (g - np.asarray(ref, float)))) >= 0.0:
            break
        g = g[:, ::-1].copy()
    return g, du, dv, nrm, min(float(chamfer), 0.38 * float(wall))


def shell_patch(grid, wall=0.0024, chamfer=0.00048, col=SHELL_CF, ref=(0.0, 0.115, 0.020), rim_col=SEAM_SILVER):
    """Build a watertight contoured armor panel with a distinct bevel rim (`rim_col`)."""
    np = np_mod()
    g, du, dv, nrm, ch = _surface_frame(grid, wall, chamfer, ref)
    nu, nv = g.shape[:2]
    su = np.maximum(np.linalg.norm(du, axis=-1, keepdims=True), 1e-12)
    sv = np.maximum(np.linalg.norm(dv, axis=-1, keepdims=True), 1e-12)
    ins = np.zeros_like(g)
    ins[0] += np.minimum(ch, 0.28 * su[0]) * (du[0] / su[0])
    ins[-1] -= np.minimum(ch, 0.28 * su[-1]) * (du[-1] / su[-1])
    ins[:, 0] += np.minimum(ch, 0.28 * sv[:, 0]) * (dv[:, 0] / sv[:, 0])
    ins[:, -1] -= np.minimum(ch, 0.28 * sv[:, -1]) * (dv[:, -1] / sv[:, -1])
    outer, inner = g + ins, g - float(wall) * nrm + ins
    co_g, ci_g = g - ch * nrm, g - (float(wall) - ch) * nrm
    rcol = col if rim_col is None else rim_col
    tris = []
    for i in range(nu - 1):
        for j in range(nv - 1):
            a, b, c, d = outer[i, j], outer[i + 1, j], outer[i + 1, j + 1], outer[i, j + 1]
            qa, qb, qc, qd = inner[i, j], inner[i + 1, j], inner[i + 1, j + 1], inner[i, j + 1]
            tris += [(np.array([a, b, c]), col), (np.array([a, c, d]), col),
                     (np.array([qa, qc, qb]), rcol), (np.array([qa, qd, qc]), rcol)]
    bidx = ([(i, 0) for i in range(nu - 1)] + [(nu - 1, j) for j in range(nv - 1)]
            + [(i, nv - 1) for i in range(nu - 1, 0, -1)] + [(0, j) for j in range(nv - 1, 0, -1)])
    rings, nb = [np.array([m[ij] for ij in bidx]) for m in (outer, co_g, ci_g, inner)], len(bidx)
    for ra, rb in zip(rings[:-1], rings[1:]):
        for k in range(nb):
            kn = (k + 1) % nb
            tris += [(np.array([ra[k], rb[k], rb[kn]]), rcol), (np.array([ra[k], rb[kn], ra[kn]]), rcol)]
    return tris


def _clip_convex_poly(poly, a_e, n_e, min_area=1e-10):
    """Clip 2D polygon `poly` against every inward half-plane `(x - a_e[i]) . n_e[i] >= 0`."""
    np = np_mod()
    cur = poly
    for a, n in zip(a_e, n_e):
        f = (cur - a) @ n
        if (f >= 0.0).all():
            continue
        if (f <= 0.0).all():
            return None
        nxt, m = [], len(cur)
        for i in range(m):
            j = (i + 1) % m
            p, q, fp, fq = cur[i], cur[j], float(f[i]), float(f[j])
            if fp >= 0.0:
                nxt.append(p)
            if (fp >= 0.0) != (fq >= 0.0) and abs(fp - fq) > 1e-12:
                nxt.append(p + (fp / (fp - fq)) * (q - p))
        if len(nxt) < 3:
            return None
        cur = np.asarray(nxt, float)
    area = 0.5 * abs(float(np.sum(cur[:, 0] * np.roll(cur[:, 1], -1) - np.roll(cur[:, 0], -1) * cur[:, 1])))
    if area < float(min_area):
        return None
    if min(float(cur[:, 0].max() - cur[:, 0].min()), float(cur[:, 1].max() - cur[:, 1].min())) < 0.00045:
        return None
    return cur


def _prism_tris(top, bot, top_col, side_col):
    """Emit watertight closed 3D prism triangles for CCW `top` and `bot` `(m, 3)` loops."""
    np = np_mod()
    m, tris = len(top), []
    for k in range(1, m - 1):
        tris.append((np.stack((top[0], top[k], top[k + 1])), top_col))
        tris.append((np.stack((bot[0], bot[k + 1], bot[k])), top_col))
    for k in range(m):
        kn = (k + 1) % m
        tris.append((np.stack((top[k], bot[k], bot[kn])), side_col))
        tris.append((np.stack((top[k], bot[kn], top[kn])), side_col))
    return tris


def _plane_basis(poly_3d, ref):
    """Fit outward-oriented orthonormal plane basis `(ctr, e1, e2, nrm, p2)` for 3D polygon `poly_3d`."""
    np = np_mod()
    pts = np.asarray(poly_3d, float)
    ctr = pts.mean(0)
    _, _, vt = np.linalg.svd(pts - ctr, full_matrices=False)
    nrm = vt[2] / max(float(np.linalg.norm(vt[2])), 1e-12)
    if float(np.dot(nrm, ctr - np.asarray(ref, float))) < 0.0:
        nrm = -nrm
    e1 = pts[1] - pts[0]
    e1 = e1 - float(np.dot(e1, nrm)) * nrm
    e1 = e1 / max(float(np.linalg.norm(e1)), 1e-12)
    e2 = np.cross(nrm, e1)
    p2 = csg.oriented(np.c_[(pts - ctr) @ e1, (pts - ctr) @ e2], ccw=True, clean=True)
    return ctr, e1, e2, nrm, p2


def planar_backing(poly_3d, wall=0.0012, inset_m=0.0022, corner_c=0.0014, col=BAFFLE_DARK,
                   ref=(0.0, 0.115, 0.020), shift_n=-0.0036):
    """Build a watertight flat matte backing slab strictly parallel to `planar_honeycomb_panel(poly_3d)`."""
    ctr, e1, e2, nrm, p2 = _plane_basis(poly_3d, ref)
    p_out = csg.oriented(csg.filleted2d(p2, r=float(corner_c), n=1), ccw=True, clean=True) if corner_c > 1e-4 else p2
    p_in = csg.inset2d(p_out, float(inset_m), outer=True) if inset_m > 1e-4 else p_out
    orig = ctr + float(shift_n) * nrm
    top = orig[None, :] + p_in[:, 0:1] * e1[None, :] + p_in[:, 1:2] * e2[None, :]
    bot = top - float(wall) * nrm[None, :]
    return _prism_tris(top, bot, col, col)


def planar_honeycomb_panel(poly_3d, wall=0.0024, chamfer=0.00068, rim_m=0.0034, pitch=0.0074, fill=0.72,
                           corner_c=0.0014, col=SHELL_CF, bar_col=MESH_PEARL, hole_col=BAFFLE_DARK,
                           rim_col=SEAM_SILVER, ref=(0.0, 0.115, 0.020), shift_n=0.0):
    """Build a 100% flat unsculpted angular polygonal frame (`col`) enclosing flat coarse honeycomb (`bar_col`)."""
    np = np_mod()
    ctr, e1, e2, nrm, p2 = _plane_basis(poly_3d, ref)
    p_out = csg.oriented(csg.filleted2d(p2, r=float(corner_c), n=1), ccw=True, clean=True) if corner_c > 1e-4 else p2
    # Cap rim width cleanly so narrow facets (e.g. nose) always retain an open central honeycomb bay.
    span = min(float(p_out[:, 0].max() - p_out[:, 0].min()), float(p_out[:, 1].max() - p_out[:, 1].min()))
    rim = min(float(rim_m), 0.25 * span)
    ch = min(float(chamfer), 0.34 * float(wall), 0.35 * rim)
    p_in = csg.inset2d(p_out, rim, outer=True)
    p_rab = csg.inset2d(p_out, 0.52 * rim, outer=True)
    p_ch = csg.inset2d(p_out, ch, outer=True)
    orig = ctr + float(shift_n) * nrm

    def _lift(xy, h_off):
        return orig[None, :] + float(h_off) * nrm[None, :] + xy[:, 0:1] * e1[None, :] + xy[:, 1:2] * e2[None, :]

    # Crisp flat angular cross-section: flat front face (`col`), flat 45-deg chamfer (`rim_col`), flat walls (`PANEL_RIM`).
    frame_rings = (
        _lift(p_in, 0.0),
        _lift(p_ch, 0.0),
        _lift(p_out, -ch),
        _lift(p_out, -float(wall)),
        _lift(p_rab, -float(wall)),
        _lift(p_in, -float(wall) + 0.00042),
    )
    band_cols = (col, rim_col, PANEL_RIM, PANEL_RIM, PANEL_RIM, PANEL_RIM)
    tris, nb = [], len(p_out)
    for idx_b, bcol in enumerate(band_cols):
        ra, rb = frame_rings[idx_b], frame_rings[(idx_b + 1) % len(frame_rings)]
        for k in range(nb):
            kn = (k + 1) % nb
            tris.append((np.stack((ra[k], rb[k], rb[kn])), bcol))
            tris.append((np.stack((ra[k], rb[kn], ra[kn])), bcol))

    a_e = p_rab
    d_e = np.roll(p_rab, -1, axis=0) - p_rab
    n_e = np.c_[-d_e[:, 1], d_e[:, 0]] / np.maximum(np.linalg.norm(d_e, axis=1, keepdims=True), 1e-12)
    c_e = np.einsum('ei,ei->e', a_e, n_e)

    def _sdf(xy):
        return np.min(np.einsum('ki,ei->ke', xy, n_e) - c_e[None, :], axis=1)

    p_m = max(0.0030, float(pitch))
    rise = p_m * math.sqrt(3.0) * 0.5
    x0, x1 = float(p_rab[:, 0].min()), float(p_rab[:, 0].max())
    y0, y1 = float(p_rab[:, 1].min()), float(p_rab[:, 1].max())
    mx0, my0 = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    nx, ny = int((x1 - x0) / (2.0 * p_m)) + 2, int((y1 - y0) / (2.0 * rise)) + 2
    ang = np.arange(6) * (math.pi / 3.0) + math.pi / 6.0
    unit_o = np.stack([np.cos(ang), np.sin(ang)], 1) * (p_m / math.sqrt(3.0))
    unit_i = unit_o * float(fill)
    ht, hb = -0.00032, -float(wall) + 0.00038
    whole, partial = {}, []
    for jr in range(-ny, ny + 1):
        cy, xs = my0 + jr * rise, 0.5 * p_m if (jr & 1) else 0.0
        for ic in range(-nx, nx + 1):
            o6 = np.array([mx0 + ic * p_m + xs, cy]) + unit_o
            i6 = np.array([mx0 + ic * p_m + xs, cy]) + unit_i
            fo = _sdf(o6)
            if (fo <= -0.0004).all():
                continue
            if (fo >= 0.0).all():
                whole[(ic, jr)] = (o6, i6)
            else:
                partial.append((o6, i6))
    even_nbr = ((0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, 0))
    odd_nbr = ((1, 1), (0, 1), (-1, 0), (0, -1), (1, -1), (1, 0))
    for (ic, jr), (o6, i6) in whole.items():
        ot, ob, it, ib = _lift(o6, ht), _lift(o6, hb), _lift(i6, ht), _lift(i6, hb)
        nbrs = odd_nbr if (jr & 1) else even_nbr
        for k in range(6):
            kn = (k + 1) % 6
            tris += [
                (np.stack((ot[k], ot[kn], it[kn])), bar_col), (np.stack((ot[k], it[kn], it[k])), bar_col),
                (np.stack((ob[k], ib[kn], ob[kn])), bar_col), (np.stack((ob[k], ib[k], ib[kn])), bar_col),
                (np.stack((it[k], it[kn], ib[kn])), hole_col), (np.stack((it[k], ib[kn], ib[k])), hole_col),
            ]
            di, dj = nbrs[k]
            if (ic + di, jr + dj) not in whole:
                tris += [(np.stack((ot[k], ob[kn], ot[kn])), hole_col), (np.stack((ot[k], ob[k], ob[kn])), hole_col)]
    min_bar_a = 0.12 * (1.0 - float(fill) ** 2) * (math.sqrt(3.0) * 0.5 * p_m * p_m / 6.0)
    for o6, i6 in partial:
        for k in range(6):
            kn = (k + 1) % 6
            clip = _clip_convex_poly(np.stack((o6[k], o6[kn], i6[kn], i6[k])), a_e, n_e, min_bar_a)
            if clip is not None:
                tris += _prism_tris(_lift(clip, ht), _lift(clip, hb), bar_col, hole_col)
    return tris


def compute_heatsink():
    """Build recessed cranial compute fins and titanium heatpipes inside the cranial bay."""
    tris = []
    for th_deg in (14.0, 34.0, 54.0, 74.0, 94.0, 114.0, 134.0, 154.0, 174.0, 194.0):
        th = math.radians(th_deg)
        sy, cz = math.sin(th), math.cos(th)
        ys, zs = 0.144 + 0.056 * sy, -0.004 + (0.062 if cz >= 0.0 else 0.064) * cz
        ang_x = math.atan2(0.062 * cz, 0.056 * sy)
        fin = csg.extrude(csg.filleted2d(csg.box2d((-0.020, 0.020), (-0.0036, 0.0036)), r=0.0005),
                          a=-0.0009, b=0.0009, axis=2, colour=BAFFLE_DARK, chamfer=0.0002).tris()
        tris += turn_tris(fin, rot('x', -ang_x), (0.0, 0.0, 0.0), (0.0, ys, zs))
    for sgn in (1.0, -1.0):
        tris += csg.sweep(csg.circle2d(0.0019, fn=14), [
            (sgn * 0.015, 0.130, 0.058), (sgn * 0.032, 0.150, 0.042),
            (sgn * 0.038, 0.164, 0.002), (sgn * 0.034, 0.146, -0.042),
            (sgn * 0.018, 0.118, -0.060),
        ], chamfer_ends=0.0005, colour=TITANIUM).tris()
    return tris


def modern_optics_local():
    """Build expressive feminine anime-mecha ocular module with 10.1 mm sclera, 6.2 mm sapphire iris & catchlights."""
    cam = (rev([(0.0, 0.0034), (0.0058, 0.0034), (0.0068, 0.0022), (0.0068, -0.0014),
                (0.0058, -0.0020), (0.0050, -0.0016), (0.0, -0.0016)], col=CHASSIS_CF)
           + rev([(0.0062, -0.0008), (0.0099, -0.0008), (0.0101, -0.0015), (0.0087, -0.0031),
                  (0.0063, -0.0040), (0.0062, -0.0018)], fn=48, col=OPTIC_SCLERA)
           + rev([(0.0058, -0.0018), (0.0063, -0.0018), (0.0063, -0.0040), (0.0058, -0.0042)], fn=44, col=(26, 48, 86))
           + rev([(0.0025, -0.0016), (0.0058, -0.0016), (0.0058, -0.0042), (0.0041, -0.0046), (0.0025, -0.0047)],
                 fn=48, col=OPTIC_GLASS)
           + rev([(0.0018, -0.0016), (0.0025, -0.0016), (0.0025, -0.0047), (0.0018, -0.0048)], fn=36, col=(164, 234, 255))
           + rev([(0.0, -0.0016), (0.0018, -0.0016), (0.0018, -0.0048), (0.0, -0.0049)], fn=32, col=OPTIC_DARK)
           + rev([(0.0, -0.0044), (0.00105, -0.0044), (0.00105, -0.00496), (0.00048, -0.00514), (0.0, -0.00518)],
                 at=(0.0018, 0.0017), fn=18, col=(252, 254, 255))
           + rev([(0.0, -0.0044), (0.00052, -0.0044), (0.00052, -0.00493), (0.0, -0.00508)],
                 at=(-0.0016, -0.0017), fn=14, col=(244, 249, 255))
           + csg.extrude(csg.filleted2d(csg.box2d((-0.0011, 0.0011), (0.0054, 0.0074)), r=0.00038),
                         a=0.0006, b=0.0026, axis=2, colour=TITANIUM, chamfer=0.0003).tris())
    ring = rev(csg.filleted2d([(0.0073, -0.0012), (0.0084, -0.0012), (0.0084, 0.0013), (0.0073, 0.0013)],
                              r=0.00038, n=3), fn=40, col=PANEL_RIM)
    pins = (csg.extrude(csg.circle2d(0.0009, fn=14), a=-0.0098, b=-0.0083, axis=1, colour=TITANIUM).tris()
            + csg.extrude(csg.circle2d(0.0009, fn=14), a=0.0083, b=0.0098, axis=1, colour=TITANIUM).tris())
    return cam, ring, pins


def rod(p0, p1, r=0.00085, col=TITANIUM):
    """Build chamfered titanium pushrod with dark collars between `p0` and `p1`."""
    np = np_mod()
    a, b = np.asarray(p0, float), np.asarray(p1, float)
    ln = float(np.linalg.norm(b - a))
    if ln < 0.003:
        return []
    u, c2, c3 = (b - a) / ln, csg.circle2d(r, fn=14), csg.circle2d(1.48 * r, fn=14)
    return (csg.sweep(c2, [a + u * s for s in (0.0, 0.0018, 0.5 * ln, ln - 0.0018, ln)],
                      chamfer_ends=0.35 * r, colour=col).tris()
            + csg.sweep(c3, [a, a + 0.0016 * u], chamfer_ends=0.3 * r, colour=PANEL_RIM).tris()
            + csg.sweep(c3, [b - 0.0016 * u, b], chamfer_ends=0.3 * r, colour=PANEL_RIM).tris())


def micro_servo(at, axis, ang, hlen=0.0058, hdir=(0.0, 0.0, 1.0), dual=False, sgn=1.0):
    """Build compact sub-micro servo body at `at`, spline on `axis`, and titanium horn at `ang` rad."""
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
    body += csg.extrude(ears, a=mid - 0.0008, b=mid + 0.0008, axis=ai, colour=PANEL_RIM, chamfer=0.00025).translate(*off).tris()
    sp = np.array([cx, cy, cz], float)
    sp[ai] = (hi + 0.0013) if sgn >= 0.0 else (lo - 0.0013)
    hd = np.asarray(hdir, float)
    hd[ai] = 0.0
    hd = rot(axis, ang) @ (hd / max(float(np.linalg.norm(hd)), 1e-12))
    t1, t0 = sp + float(hlen) * hd, sp - (float(hlen) if dual else 0.0014) * hd
    horn = csg.sweep(csg.filleted2d(csg.box2d((-0.0013, 0.0013), (-0.0007, 0.0007)), r=0.00032),
                     [t0, sp, t1], chamfer_ends=0.00025, colour=TITANIUM).tris()
    return body, horn, t1, t0


def ear_microphones():
    """Build concentric circular binaural mecha ear-pods flush on the temples."""
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


def neck_band(yn, tri):
    """Return quad-midpoint cervical band RGB for loft triangle `tri`."""
    ym = 0.5 * (float(yn[tri].min()) + float(yn[tri].max()))
    if ym < -0.004:
        return SHELL_CF
    if ym < 0.004:
        return BAFFLE_DARK
    return PANEL_RIM if (0.018 <= ym < 0.024 or 0.034 <= ym < 0.040 or 0.050 <= ym < 0.056) else CHASSIS_CF
