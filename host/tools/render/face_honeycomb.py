"""Fine-mesh framed honeycomb armor panels, bevel shells, heatsink fins and eye optics."""
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg  # noqa: E402

SHELL_CF, PANEL_RIM, SEAM_SILVER, CHASSIS_CF, TITANIUM = (240, 243, 248), (62, 68, 80), (196, 202, 212), (24, 28, 36), (176, 182, 194)
ANODIZED, SERVO_CASE, HORN_BRASS = (226, 48, 178), (28, 32, 40), (206, 168, 84)
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
    """Build a watertight contoured armor panel with a distinct gunmetal bevel rim (`rim_col`)."""
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


def _eval_patch(g, nrm, ch, wall, u, v):
    """Evaluate bilinear outer, recessed lattice, chamfer and inner 3D coordinates for `(u, v)` in `[0, 1]`."""
    np = np_mod()
    nu, nv = g.shape[:2]
    fu, fv = np.clip(u, 0.0, 1.0) * (nu - 1), np.clip(v, 0.0, 1.0) * (nv - 1)
    i0, j0 = np.minimum(np.floor(fu).astype(int), nu - 2), np.minimum(np.floor(fv).astype(int), nv - 2)
    tu, tv = (fu - i0)[:, None], (fv - j0)[:, None]
    w00, w10, w11, w01 = (1.0 - tu) * (1.0 - tv), tu * (1.0 - tv), tu * tv, (1.0 - tu) * tv
    base = w00 * g[i0, j0] + w10 * g[i0 + 1, j0] + w11 * g[i0 + 1, j0 + 1] + w01 * g[i0, j0 + 1]
    nv_ = w00 * nrm[i0, j0] + w10 * nrm[i0 + 1, j0] + w11 * nrm[i0 + 1, j0 + 1] + w01 * nrm[i0, j0 + 1]
    nv_ /= np.maximum(np.linalg.norm(nv_, axis=1, keepdims=True), 1e-12)
    t_u = ((1.0 - tv) * (g[i0 + 1, j0] - g[i0, j0]) + tv * (g[i0 + 1, j0 + 1] - g[i0, j0 + 1])) * (nu - 1)
    t_v = ((1.0 - tu) * (g[i0, j0 + 1] - g[i0, j0]) + tu * (g[i0 + 1, j0 + 1] - g[i0 + 1, j0])) * (nv - 1)
    lu, lv = np.maximum(np.linalg.norm(t_u, axis=1, keepdims=True), 1e-12), np.maximum(np.linalg.norm(t_v, axis=1, keepdims=True), 1e-12)
    ins = np.zeros_like(base)
    edge_u, edge_v = np.minimum(ch, 0.018 * lu) * (t_u / lu), np.minimum(ch, 0.018 * lv) * (t_v / lv)
    ins[u <= 1e-9] += edge_u[u <= 1e-9]
    ins[u >= 1.0 - 1e-9] -= edge_u[u >= 1.0 - 1e-9]
    ins[v <= 1e-9] += edge_v[v <= 1e-9]
    ins[v >= 1.0 - 1e-9] -= edge_v[v >= 1.0 - 1e-9]
    return base, nv_, base + ins, base - ch * nv_, base - (float(wall) - ch) * nv_, base - float(wall) * nv_ + ins


def _hex_mesh_2d(nc, nr, mu, mv, fill):
    """Return `(pts_uv, border_tris, lattice_tris, holes, border_ring, win_ring)` for a fine-mesh framed hex window."""
    nh = 2 * nc + 1
    su, sv = (1.0 - 2.0 * mu) / nh, (1.0 - 2.0 * mv) / nr
    u_all = [0.0] + [mu + k * su for k in range(nh + 1)] + [1.0]
    v_all = [0.0] + [mv + r * sv for r in range(nr + 1)] + [1.0]
    pts, idx_map = [], {}

    def vid(u, v):
        key = (round(float(u), 9), round(float(v), 9))
        if key not in idx_map:
            idx_map[key] = len(pts)
            pts.append((float(u), float(v)))
        return idx_map[key]

    grid_id = [[vid(u, v) for v in v_all] for u in u_all]
    border, lattice, holes = [], [], []
    nu_a, nv_a = len(u_all) - 1, len(v_all) - 1
    for i in range(nu_a):
        border += [(grid_id[i][0], grid_id[i + 1][0], grid_id[i + 1][1]), (grid_id[i][0], grid_id[i + 1][1], grid_id[i][1]),
                   (grid_id[i][nv_a - 1], grid_id[i + 1][nv_a - 1], grid_id[i + 1][nv_a]), (grid_id[i][nv_a - 1], grid_id[i + 1][nv_a], grid_id[i][nv_a])]
    for j in range(1, nv_a - 1):
        border += [(grid_id[0][j], grid_id[1][j], grid_id[1][j + 1]), (grid_id[0][j], grid_id[1][j + 1], grid_id[0][j + 1]),
                   (grid_id[nu_a - 1][j], grid_id[nu_a][j], grid_id[nu_a][j + 1]), (grid_id[nu_a - 1][j], grid_id[nu_a][j + 1], grid_id[nu_a - 1][j + 1])]
    rx, ry = fill * su, fill * (0.5 * sv)
    for r in range(nr):
        j, odd = r + 1, r % 2
        i0 = 1 if odd else nu_a - 2
        lattice += [(grid_id[i0][j], grid_id[i0 + 1][j], grid_id[i0 + 1][j + 1]), (grid_id[i0][j], grid_id[i0 + 1][j + 1], grid_id[i0][j + 1])]
        for c in range(nc):
            k0 = 1 + odd + 2 * c
            cu, cv = u_all[k0 + 1], 0.5 * (v_all[j] + v_all[j + 1])
            outer = (grid_id[k0][j], grid_id[k0 + 1][j], grid_id[k0 + 2][j],
                     grid_id[k0 + 2][j + 1], grid_id[k0 + 1][j + 1], grid_id[k0][j + 1])
            q = [vid(cu + dx, cv + dy) for dx, dy in ((-rx, -0.55 * ry), (0.0, -ry), (rx, -0.55 * ry),
                                                      (rx, 0.55 * ry), (0.0, ry), (-rx, 0.55 * ry))]
            for m in range(6):
                mn = (m + 1) % 6
                lattice += [(outer[m], outer[mn], q[mn]), (outer[m], q[mn], q[m])]
            holes.append(q)
    ring = ([grid_id[i][0] for i in range(nu_a)] + [grid_id[nu_a][j] for j in range(nv_a)]
            + [grid_id[i][nv_a] for i in range(nu_a, 0, -1)] + [grid_id[0][j] for j in range(nv_a, 0, -1)])
    win = ([grid_id[i][1] for i in range(1, nu_a - 1)] + [grid_id[nu_a - 1][j] for j in range(1, nv_a - 1)]
           + [grid_id[i][nv_a - 1] for i in range(nu_a - 1, 1, -1)] + [grid_id[1][j] for j in range(nv_a - 1, 1, -1)])
    return pts, border, lattice, holes, ring, win


def honeycomb_patch(grid, nc=None, nr=None, wall=0.0023, chamfer=0.00046, margin=None, fill=0.74,
                    col=SHELL_CF, hole_col=BAFFLE_DARK, ref=(0.0, 0.115, 0.020), bar_col=BAFFLE_DARK,
                    pitch=0.0040, rim_m=0.0025, rim_col=SEAM_SILVER):
    """Build a slim contoured armor bezel (`col`) framing metric-isotropic equilateral micro-honeycomb (`bar_col`)."""
    np = np_mod()
    g, _du, _dv, nrm, ch = _surface_frame(grid, wall, chamfer, ref)
    # Compute physical 3D arc lengths (`w_m`, `h_m`) so every hex cell is equilateral and every bezel stays slim (`~2.5 mm`).
    w_m = float(np.linalg.norm(g[1:] - g[:-1], axis=-1).sum(axis=0).mean())
    h_m = float(np.linalg.norm(g[:, 1:] - g[:, :-1], axis=-1).sum(axis=1).mean())
    mu = float(margin[0]) if margin is not None else min(0.22, max(0.06, float(rim_m) / max(w_m, 1e-4)))
    mv = float(margin[1]) if margin is not None else min(0.24, max(0.07, float(rim_m) / max(h_m, 1e-4)))
    uw_m, vh_m = max(0.004, w_m * (1.0 - 2.0 * mu)), max(0.004, h_m * (1.0 - 2.0 * mv))
    nc_i = int(nc) if nc is not None else max(3, min(14, int(round(uw_m / max(float(pitch), 0.0022)))))
    nr_i = int(nr) if nr is not None else max(2, min(11, int(round(vh_m / max(0.866 * float(pitch), 0.0020)))))
    pts_uv, border, lattice, holes, ring, win = _hex_mesh_2d(nc_i, nr_i, mu, mv, float(fill))
    uv = np.asarray(pts_uv, float)
    base, nv_, out, co, ci, inn = _eval_patch(g, nrm, ch, wall, uv[:, 0], uv[:, 1])
    rec = min(0.00060, 0.26 * float(wall))
    lat_f, lat_b = base - rec * nv_, base - (float(wall) - rec) * nv_
    tris = []
    for a, b, c in border:
        tris.append((np.stack((out[a], out[b], out[c])), col))
        tris.append((np.stack((inn[a], inn[c], inn[b])), PANEL_RIM))
    for a, b, c in lattice:
        tris.append((np.stack((lat_f[a], lat_f[b], lat_f[c])), bar_col))
        tris.append((np.stack((lat_b[a], lat_b[c], lat_b[b])), bar_col))
    nw = len(win)
    for k in range(nw):
        a, b = win[k], win[(k + 1) % nw]
        tris.append((np.stack((out[a], lat_f[a], lat_f[b])), PANEL_RIM))
        tris.append((np.stack((out[a], lat_f[b], out[b])), PANEL_RIM))
        tris.append((np.stack((lat_b[a], inn[a], inn[b])), PANEL_RIM))
        tris.append((np.stack((lat_b[a], inn[b], lat_b[b])), PANEL_RIM))
    for q in holes:
        for k in range(6):
            a, b = q[k], q[(k + 1) % 6]
            tris.append((np.stack((lat_f[a], lat_f[b], lat_b[b])), hole_col))
            tris.append((np.stack((lat_f[a], lat_b[b], lat_b[a])), hole_col))
    rings, nb = (out, co, ci, inn), len(ring)
    for ra, rb in zip(rings[:-1], rings[1:]):
        for k in range(nb):
            a, b = ring[k], ring[(k + 1) % nb]
            tris.append((np.stack((ra[a], rb[a], rb[b])), rim_col))
            tris.append((np.stack((ra[a], rb[b], ra[b])), rim_col))
    return tris


def compute_heatsink():
    """Build recessed cranial compute fins and titanium heatpipes visible through the fine honeycomb & air gaps."""
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
    """Build expressive anime-mecha ocular module with corner-to-corner 10.4 mm sclera, 5.5 mm sapphire iris & gimbal."""
    cam = (rev([(0.0, 0.0034), (0.0058, 0.0034), (0.0068, 0.0022), (0.0068, -0.0014),
                (0.0058, -0.0020), (0.0050, -0.0016), (0.0, -0.0016)], col=CHASSIS_CF)
           + rev([(0.0056, -0.0008), (0.0099, -0.0008), (0.0101, -0.0015), (0.0087, -0.0031),
                  (0.0057, -0.0042), (0.0056, -0.0018)], fn=48, col=OPTIC_SCLERA)
           + rev([(0.0053, -0.0018), (0.0057, -0.0018), (0.0057, -0.0043), (0.0053, -0.0044)], fn=44, col=BROW_DARK)
           + rev([(0.0024, -0.0016), (0.0053, -0.0016), (0.0053, -0.0044), (0.0038, -0.0047), (0.0024, -0.0047)],
                 fn=48, col=OPTIC_GLASS)
           + rev([(0.0018, -0.0016), (0.0024, -0.0016), (0.0024, -0.0047), (0.0018, -0.0048)], fn=36, col=(172, 230, 255))
           + rev([(0.0, -0.0016), (0.0018, -0.0016), (0.0018, -0.0048), (0.0, -0.0049)], fn=32, col=OPTIC_DARK)
           + rev([(0.0, -0.0044), (0.00095, -0.0044), (0.00095, -0.00495), (0.00045, -0.00512), (0.0, -0.00516)],
                 at=(0.0017, 0.0016), fn=18, col=(252, 254, 255))
           + rev([(0.0, -0.0044), (0.00048, -0.0044), (0.00048, -0.00492), (0.0, -0.00506)],
                 at=(-0.0015, -0.0016), fn=14, col=(244, 249, 255))
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
