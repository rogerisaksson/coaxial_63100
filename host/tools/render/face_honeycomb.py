"""Faceted guard-style armor panels, hexagonal vent windows, heatsink fins and eye optics."""
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg  # noqa: E402

SHELL_CF, PANEL_RIM, CHASSIS_CF, TITANIUM = (226, 230, 237), (92, 98, 110), (32, 36, 44), (174, 180, 190)
ANODIZED, LIP_ROSE, SERVO_CASE, HORN_BRASS = (226, 48, 178), (226, 230, 237), (28, 32, 40), (206, 168, 84)
NECK_SKIN, OPTIC_DARK, OPTIC_GLASS, BAFFLE_DARK, MIC_GOLD = (182, 186, 194), (10, 14, 22), (28, 72, 118), (16, 19, 26), (212, 176, 84)


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


def shell_patch(grid, wall=0.0028, chamfer=0.00065, col=SHELL_CF, ref=(0.0, 0.115, 0.020), rim_col=PANEL_RIM):
    """Build a watertight faceted armor panel with a distinct gunmetal bevel rim (`rim_col`)."""
    np = np_mod()
    g, du, dv, nrm, ch = _surface_frame(grid, wall, chamfer, ref)
    nu, nv = g.shape[:2]
    su = np.maximum(np.linalg.norm(du, axis=-1, keepdims=True), 1e-12)
    sv = np.maximum(np.linalg.norm(dv, axis=-1, keepdims=True), 1e-12)
    ins = np.zeros_like(g)
    ins[0] += np.minimum(ch, 0.30 * su[0]) * (du[0] / su[0])
    ins[-1] -= np.minimum(ch, 0.30 * su[-1]) * (du[-1] / su[-1])
    ins[:, 0] += np.minimum(ch, 0.30 * sv[:, 0]) * (dv[:, 0] / sv[:, 0])
    ins[:, -1] -= np.minimum(ch, 0.30 * sv[:, -1]) * (dv[:, -1] / sv[:, -1])
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
    """Evaluate bilinear outer, chamfer and inner 3D coordinates for `(u, v)` in `[0, 1]`."""
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
    edge_u, edge_v = np.minimum(ch, 0.020 * lu) * (t_u / lu), np.minimum(ch, 0.020 * lv) * (t_v / lv)
    ins[u <= 1e-9] += edge_u[u <= 1e-9]
    ins[u >= 1.0 - 1e-9] -= edge_u[u >= 1.0 - 1e-9]
    ins[v <= 1e-9] += edge_v[v <= 1e-9]
    ins[v >= 1.0 - 1e-9] -= edge_v[v >= 1.0 - 1e-9]
    return base + ins, base - ch * nv_, base - (float(wall) - ch) * nv_, base - float(wall) * nv_ + ins


def _hex_mesh_2d(nc, nr, mu, mv, fill):
    """Return `(pts_uv, border_tris, lattice_tris, holes, border_ring)` for a framed guard-style hex window."""
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
    return pts, border, lattice, holes, ring


def honeycomb_patch(grid, nc=4, nr=3, wall=0.0028, chamfer=0.00065, margin=(0.24, 0.24), fill=0.68,
                    col=SHELL_CF, hole_col=CHASSIS_CF, ref=(0.0, 0.115, 0.020), bar_col=CHASSIS_CF):
    """Build a solid pearl-white armor panel framing a black open honeycomb guard window (`bar_col`)."""
    np = np_mod()
    g, _du, _dv, nrm, ch = _surface_frame(grid, wall, chamfer, ref)
    pts_uv, border, lattice, holes, ring = _hex_mesh_2d(int(nc), int(nr), float(margin[0]), float(margin[1]), float(fill))
    uv = np.asarray(pts_uv, float)
    out, co, ci, inn = _eval_patch(g, nrm, ch, wall, uv[:, 0], uv[:, 1])
    hcol = PANEL_RIM if hole_col is None else hole_col
    tris = []
    for a, b, c in border:
        tris.append((np.stack((out[a], out[b], out[c])), col))
        tris.append((np.stack((inn[a], inn[c], inn[b])), PANEL_RIM))
    for a, b, c in lattice:
        tris.append((np.stack((out[a], out[b], out[c])), bar_col))
        tris.append((np.stack((inn[a], inn[c], inn[b])), bar_col))
    for q in holes:
        for k in range(6):
            a, b = q[k], q[(k + 1) % 6]
            tris.append((np.stack((out[a], out[b], inn[b])), hcol))
            tris.append((np.stack((out[a], inn[b], inn[a])), hcol))
    rings, nb = (out, co, ci, inn), len(ring)
    for ra, rb in zip(rings[:-1], rings[1:]):
        for k in range(nb):
            a, b = ring[k], ring[(k + 1) % nb]
            tris.append((np.stack((ra[a], rb[a], rb[b])), PANEL_RIM))
            tris.append((np.stack((ra[a], rb[b], ra[b])), PANEL_RIM))
    return tris


def compute_heatsink():
    """Build recessed cranial compute fins and titanium heatpipes visible through the airflow panel gaps."""
    tris = []
    for th_deg in (8.0, 28.0, 48.0, 68.0, 88.0, 108.0, 128.0, 148.0, 168.0, 188.0):
        th = math.radians(th_deg)
        sy, cz = math.sin(th), math.cos(th)
        ys, zs = 0.146 + 0.062 * sy, 0.002 + (0.072 if cz >= 0.0 else 0.070) * cz
        ang_x = math.atan2(0.072 * cz, 0.062 * sy)
        fin = csg.extrude(csg.filleted2d(csg.box2d((-0.022, 0.022), (-0.0036, 0.0036)), r=0.0005),
                          a=-0.0009, b=0.0009, axis=2, colour=BAFFLE_DARK, chamfer=0.0002).tris()
        tris += turn_tris(fin, rot('x', -ang_x), (0.0, 0.0, 0.0), (0.0, ys, zs))
    for sgn in (1.0, -1.0):
        tris += csg.sweep(csg.circle2d(0.0020, fn=14), [
            (sgn * 0.016, 0.130, 0.070), (sgn * 0.036, 0.150, 0.048),
            (sgn * 0.042, 0.168, 0.006), (sgn * 0.036, 0.148, -0.046),
            (sgn * 0.020, 0.118, -0.066),
        ], chamfer_ends=0.0005, colour=TITANIUM).tris()
    return tris


def modern_optics_local():
    """Build recessed titanium-ringed ocular camera module, sapphire iris bezel and 2-axis gimbal ring."""
    cam = (rev([(0.0, 0.0034), (0.0058, 0.0034), (0.0068, 0.0022), (0.0068, -0.0016),
                (0.0056, -0.0025), (0.0044, -0.0016), (0.0, -0.0016)], col=CHASSIS_CF)
           + rev([(0.0043, -0.0014), (0.0065, -0.0014), (0.0067, -0.0029), (0.0058, -0.0037),
                  (0.0042, -0.0033), (0.0042, -0.0020)], col=TITANIUM)
           + rev([(0.0026, -0.0020), (0.0043, -0.0020), (0.0042, -0.0033), (0.0026, -0.0031)], fn=36, col=OPTIC_DARK)
           + rev([(0.0012, -0.0018), (0.0028, -0.0018), (0.0028, -0.0032), (0.0019, -0.0037), (0.0012, -0.0038)],
                 fn=36, col=OPTIC_GLASS)
           + rev([(0.0, -0.0018), (0.0012, -0.0018), (0.0012, -0.0038), (0.0, -0.0039)], fn=24, col=OPTIC_DARK)
           + csg.extrude(csg.filleted2d(csg.box2d((-0.0011, 0.0011), (0.0054, 0.0074)), r=0.00038),
                         a=0.0006, b=0.0026, axis=2, colour=TITANIUM, chamfer=0.0003).tris())
    ring = rev(csg.filleted2d([(0.0073, -0.0013), (0.0086, -0.0013), (0.0086, 0.0013), (0.0073, 0.0013)],
                              r=0.00040, n=3), fn=40, col=TITANIUM)
    pins = (csg.extrude(csg.circle2d(0.0009, fn=14), a=-0.0098, b=-0.0083, axis=1, colour=TITANIUM).tris()
            + csg.extrude(csg.circle2d(0.0009, fn=14), a=0.0083, b=0.0098, axis=1, colour=TITANIUM).tris())
    return cam, ring, pins
