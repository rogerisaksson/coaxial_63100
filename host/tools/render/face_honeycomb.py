"""Dense ventilated honeycomb shell panels, internal compute heatsink and modern eye optics."""
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg  # noqa: E402

SHELL_CF, CHASSIS_CF, TITANIUM, ANODIZED = (218, 222, 230), (36, 40, 48), (172, 178, 188), (226, 48, 178)
LIP_ROSE, SERVO_CASE, HORN_BRASS, NECK_SKIN = (166, 116, 110), (30, 34, 42), (208, 170, 82), (176, 180, 188)
OPTIC_DARK, OPTIC_GLASS, BAFFLE_DARK, MIC_GOLD = (10, 14, 22), (26, 62, 108), (18, 21, 28), (212, 176, 84)


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
    ch = min(float(chamfer), 0.36 * float(wall))
    return g, du, dv, nrm, ch


def shell_patch(grid, wall=0.0030, chamfer=0.00045, col=SHELL_CF, ref=(0.0, 0.115, 0.020)):
    """Build a watertight chamfered 3D doubly-curved shell solid from `(Nu, Nv, 3)` outer vertices."""
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


def _eval_patch(g, nrm, ch, wall, u, v):
    """Evaluate bilinear outer, chamfer and inner 3D coordinates for `(u, v)` in `[0, 1]`."""
    np = np_mod()
    nu, nv = g.shape[:2]
    fu, fv = np.clip(u, 0.0, 1.0) * (nu - 1), np.clip(v, 0.0, 1.0) * (nv - 1)
    i0 = np.minimum(np.floor(fu).astype(int), nu - 2)
    j0 = np.minimum(np.floor(fv).astype(int), nv - 2)
    tu, tv = (fu - i0)[:, None], (fv - j0)[:, None]
    w00, w10, w11, w01 = (1.0 - tu) * (1.0 - tv), tu * (1.0 - tv), tu * tv, (1.0 - tu) * tv
    base = w00 * g[i0, j0] + w10 * g[i0 + 1, j0] + w11 * g[i0 + 1, j0 + 1] + w01 * g[i0, j0 + 1]
    nv_ = w00 * nrm[i0, j0] + w10 * nrm[i0 + 1, j0] + w11 * nrm[i0 + 1, j0 + 1] + w01 * nrm[i0, j0 + 1]
    nv_ /= np.maximum(np.linalg.norm(nv_, axis=1, keepdims=True), 1e-12)
    t_u = ((1.0 - tv) * (g[i0 + 1, j0] - g[i0, j0]) + tv * (g[i0 + 1, j0 + 1] - g[i0, j0 + 1])) * (nu - 1)
    t_v = ((1.0 - tu) * (g[i0, j0 + 1] - g[i0, j0]) + tu * (g[i0 + 1, j0 + 1] - g[i0 + 1, j0])) * (nv - 1)
    lu = np.maximum(np.linalg.norm(t_u, axis=1, keepdims=True), 1e-12)
    lv = np.maximum(np.linalg.norm(t_v, axis=1, keepdims=True), 1e-12)
    ins = np.zeros_like(base)
    edge_u = np.minimum(ch, 0.015 * lu) * (t_u / lu)
    edge_v = np.minimum(ch, 0.015 * lv) * (t_v / lv)
    ins[u <= 1e-9] += edge_u[u <= 1e-9]
    ins[u >= 1.0 - 1e-9] -= edge_u[u >= 1.0 - 1e-9]
    ins[v <= 1e-9] += edge_v[v <= 1e-9]
    ins[v >= 1.0 - 1e-9] -= edge_v[v >= 1.0 - 1e-9]
    return base + ins, base - ch * nv_, base - (float(wall) - ch) * nv_, base - float(wall) * nv_ + ins


def _hex_mesh_2d(nc, nr, mu, mv, fill):
    """Return `(pts_uv, top_tris, holes, border_ring)` for a conformal staggered hex array in `[0, 1]^2`."""
    nh = 2 * nc + 1
    su, sv = (1.0 - 2.0 * mu) / nh, (1.0 - 2.0 * mv) / nr
    us = [mu + k * su for k in range(nh + 1)]
    vs = [mv + r * sv for r in range(nr + 1)]
    u_all, v_all = [0.0] + us + [1.0], [0.0] + vs + [1.0]
    pts, idx_map = [], {}

    def vid(u, v):
        key = (round(float(u), 9), round(float(v), 9))
        i = idx_map.get(key)
        if i is None:
            i = len(pts)
            idx_map[key] = i
            pts.append((float(u), float(v)))
        return i

    grid_id = [[vid(u, v) for v in v_all] for u in u_all]
    top, holes = [], []

    def add_quad(a, b, c, d):
        top.append((a, b, c))
        top.append((a, c, d))

    # Perimeter solid bezel quads around [mu, 1-mu] x [mv, 1-mv] matching interior seams 1-to-1.
    nu_a, nv_a = len(u_all) - 1, len(v_all) - 1
    for i in range(nu_a):
        add_quad(grid_id[i][0], grid_id[i + 1][0], grid_id[i + 1][1], grid_id[i][1])
        add_quad(grid_id[i][nv_a - 1], grid_id[i + 1][nv_a - 1], grid_id[i + 1][nv_a], grid_id[i][nv_a])
    for j in range(1, nv_a - 1):
        add_quad(grid_id[0][j], grid_id[1][j], grid_id[1][j + 1], grid_id[0][j + 1])
        add_quad(grid_id[nu_a - 1][j], grid_id[nu_a][j], grid_id[nu_a][j + 1], grid_id[nu_a - 1][j + 1])

    rx, ry = fill * su, fill * (0.5 * sv)
    # Interior staggered rows of regular honeycomb cells and half-column solid end spacers.
    for r in range(nr):
        j, odd = r + 1, r % 2
        if odd:
            add_quad(grid_id[1][j], grid_id[2][j], grid_id[2][j + 1], grid_id[1][j + 1])
        else:
            add_quad(grid_id[nu_a - 2][j], grid_id[nu_a - 1][j], grid_id[nu_a - 1][j + 1], grid_id[nu_a - 2][j + 1])
        for c in range(nc):
            k0 = 1 + odd + 2 * c
            cu, cv = u_all[k0 + 1], 0.5 * (v_all[j] + v_all[j + 1])
            outer = (grid_id[k0][j], grid_id[k0 + 1][j], grid_id[k0 + 2][j],
                     grid_id[k0 + 2][j + 1], grid_id[k0 + 1][j + 1], grid_id[k0][j + 1])
            q = [vid(cu + dx, cv + dy) for dx, dy in (
                (-rx, -0.55 * ry), (0.0, -ry), (rx, -0.55 * ry),
                (rx, 0.55 * ry), (0.0, ry), (-rx, 0.55 * ry))]
            for m in range(6):
                mn = (m + 1) % 6
                add_quad(outer[m], outer[mn], q[mn], q[m])
            holes.append(q)

    ring = ([grid_id[i][0] for i in range(nu_a)] + [grid_id[nu_a][j] for j in range(nv_a)]
            + [grid_id[i][nv_a] for i in range(nu_a, 0, -1)] + [grid_id[0][j] for j in range(nv_a, 0, -1)])
    return pts, top, holes, ring


def honeycomb_patch(grid, nc=10, nr=12, wall=0.0026, chamfer=0.00040, margin=(0.09, 0.09), fill=0.68,
                    col=SHELL_CF, hole_col=None, ref=(0.0, 0.115, 0.020)):
    """Build a watertight doubly-curved shell plate perforated by a dense hexagonal honeycomb array."""
    np = np_mod()
    g, _du, _dv, nrm, ch = _surface_frame(grid, wall, chamfer, ref)
    pts_uv, top, holes, ring = _hex_mesh_2d(int(nc), int(nr), float(margin[0]), float(margin[1]), float(fill))
    uv = np.asarray(pts_uv, float)
    out, co, ci, inn = _eval_patch(g, nrm, ch, wall, uv[:, 0], uv[:, 1])
    hcol = col if hole_col is None else hole_col
    tris = []
    for a, b, c in top:
        tris.append((np.stack((out[a], out[b], out[c])), col))
        tris.append((np.stack((inn[a], inn[c], inn[b])), col))
    for q in holes:
        for k in range(6):
            a, b = q[k], q[(k + 1) % 6]
            tris.append((np.stack((out[a], out[b], inn[b])), hcol))
            tris.append((np.stack((out[a], inn[b], inn[a])), hcol))
    rings, nb = (out, co, ci, inn), len(ring)
    for ra, rb in zip(rings[:-1], rings[1:]):
        for k in range(nb):
            a, b = ring[k], ring[(k + 1) % nb]
            tris.append((np.stack((ra[a], rb[a], rb[b])), col))
            tris.append((np.stack((ra[a], rb[b], ra[b])), col))
    return tris


def compute_heatsink():
    """Build recessed cranial compute and vision processor heatsink fins behind honeycomb cooling panels."""
    np = np_mod()
    tris = []
    # Crown and upper frontal radial cooling fins seated cleanly inside ventilated panels.
    for th_deg in (-6.0, 12.0, 30.0, 48.0, 66.0, 84.0, 102.0, 120.0, 138.0, 156.0, 174.0, 192.0, 210.0):
        th = math.radians(th_deg)
        sy, cz = math.sin(th), math.cos(th)
        if abs(cz) < 0.05 and sy < 0.0:
            continue
        ys = 0.146 + 0.064 * sy
        zs = 0.002 + (0.075 if cz >= 0.0 else 0.072) * cz
        if zs > 0.068 and ys < 0.120:
            continue
        ang_x = math.atan2(0.074 * cz, 0.064 * sy)
        fin = csg.extrude(csg.filleted2d(csg.box2d((-0.026, 0.026), (-0.0038, 0.0038)), r=0.0005),
                          a=-0.0009, b=0.0009, axis=2, colour=BAFFLE_DARK, chamfer=0.0002).tris()
        tris += turn_tris(fin, rot('x', -ang_x), (0.0, 0.0, 0.0), (0.0, ys, zs))
    # Brushed titanium twin heatpipes tying temporal and occipital compute radiators together.
    for sgn in (1.0, -1.0):
        pipe = csg.sweep(csg.circle2d(0.0021, fn=14), [
            (sgn * 0.022, 0.132, 0.074), (sgn * 0.048, 0.150, 0.052),
            (sgn * 0.054, 0.168, 0.008), (sgn * 0.044, 0.148, -0.048),
            (sgn * 0.024, 0.118, -0.068),
        ], chamfer_ends=0.0005, colour=TITANIUM).tris()
        tris += pipe
    # Zygomatic and temporal side louver fins below eye plane behind maxilla and temporal vents.
    for y_at, z_at, rx_lim in ((0.084, 0.070, 0.044), (0.074, 0.068, 0.042), (0.064, 0.066, 0.040)):
        bar = csg.extrude(csg.filleted2d(csg.box2d((0.022, rx_lim), (y_at - 0.0018, y_at + 0.0018)), r=0.0005),
                          a=z_at - 0.005, b=z_at + 0.004, axis=2, colour=CHASSIS_CF, chamfer=0.0003).tris()
        tris += bar + mirror_x(bar)
    return tris


def modern_optics_local():
    """Build sleek recessed titanium-ringed ocular module with dark bezel, sapphire lens and gimbal ring."""
    cam = (rev([(0.0, 0.0034), (0.0056, 0.0034), (0.0066, 0.0022), (0.0066, -0.0016),
                (0.0054, -0.0024), (0.0042, -0.0016), (0.0, -0.0016)], col=CHASSIS_CF)
           + rev([(0.0042, -0.0014), (0.0062, -0.0014), (0.0064, -0.0028), (0.0056, -0.0036),
                  (0.0041, -0.0032), (0.0041, -0.0020)], col=TITANIUM)
           + rev([(0.0025, -0.0020), (0.0042, -0.0020), (0.0041, -0.0032), (0.0025, -0.0030)], fn=36, col=OPTIC_DARK)
           + rev([(0.0011, -0.0018), (0.0027, -0.0018), (0.0027, -0.0031), (0.0018, -0.0036), (0.0011, -0.0037)],
                 fn=36, col=OPTIC_GLASS)
           + rev([(0.0, -0.0018), (0.0011, -0.0018), (0.0011, -0.0037), (0.0, -0.0038)], fn=24, col=OPTIC_DARK)
           + csg.extrude(csg.filleted2d(csg.box2d((-0.0011, 0.0011), (0.0054, 0.0074)), r=0.00038),
                         a=0.0006, b=0.0026, axis=2, colour=TITANIUM, chamfer=0.0003).tris())
    ring = rev(csg.filleted2d([(0.0072, -0.0013), (0.0085, -0.0013), (0.0085, 0.0013), (0.0072, 0.0013)],
                              r=0.00040, n=3), fn=40, col=TITANIUM)
    pins = (csg.extrude(csg.circle2d(0.0009, fn=14), a=-0.0098, b=-0.0083, axis=1, colour=TITANIUM).tris()
            + csg.extrude(csg.circle2d(0.0009, fn=14), a=0.0083, b=0.0098, axis=1, colour=TITANIUM).tris())
    return cam, ring, pins
