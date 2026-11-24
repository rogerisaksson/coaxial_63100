"""Fine-mesh framed honeycomb armor panels, bevel shells, heatsink fins and eye optics."""
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg  # noqa: E402

SHELL_CF, PANEL_RIM, CHASSIS_CF, TITANIUM = (234, 238, 244), (78, 84, 96), (28, 32, 40), (174, 180, 190)
ANODIZED, SERVO_CASE, HORN_BRASS = (226, 48, 178), (28, 32, 40), (206, 168, 84)
NECK_SKIN, OPTIC_DARK, OPTIC_GLASS, BAFFLE_DARK, MIC_GOLD = (58, 62, 72), (10, 14, 22), (48, 158, 236), (16, 19, 26), (216, 178, 82)

# Master soft feminine anime-mecha head silhouette (m): natural low jaw/occiput enclosing neck carrier.
_PROF_Y = (0.004, 0.015, 0.028, 0.044, 0.058, 0.072, 0.086, 0.100, 0.114, 0.134, 0.156)
_PROF_X = (0.021, 0.037, 0.050, 0.059, 0.0645, 0.0678, 0.0695, 0.0698, 0.0688, 0.0716, 0.0724)
_PROF_ZF = (0.095, 0.103, 0.104, 0.107, 0.107, 0.107, 0.1065, 0.1055, 0.1060, 0.1010, 0.0925)
_PROF_ZB = (-0.046, -0.053, -0.060, -0.066, -0.071, -0.076, -0.081, -0.085, -0.088, -0.092, -0.094)


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


def shell_patch(grid, wall=0.0026, chamfer=0.00058, col=SHELL_CF, ref=(0.0, 0.115, 0.020), rim_col=PANEL_RIM):
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


def honeycomb_patch(grid, nc=7, nr=5, wall=0.0026, chamfer=0.00058, margin=(0.16, 0.17), fill=0.75,
                    col=SHELL_CF, hole_col=BAFFLE_DARK, ref=(0.0, 0.115, 0.020), bar_col=BAFFLE_DARK):
    """Build a proud pearl-white armor bezel (`col`) framing a recessed fine-mesh open honeycomb (`bar_col`)."""
    np = np_mod()
    g, _du, _dv, nrm, ch = _surface_frame(grid, wall, chamfer, ref)
    pts_uv, border, lattice, holes, ring, win = _hex_mesh_2d(int(nc), int(nr), float(margin[0]), float(margin[1]), float(fill))
    uv = np.asarray(pts_uv, float)
    base, nv_, out, co, ci, inn = _eval_patch(g, nrm, ch, wall, uv[:, 0], uv[:, 1])
    # Recess the fine-mesh honeycomb lattice cleanly inside the proud pearl-white outer frame (`0` open edges).
    rec = min(0.00065, 0.26 * float(wall))
    lat_f, lat_b = base - rec * nv_, base - (float(wall) - rec) * nv_
    tris = []
    for a, b, c in border:
        tris.append((np.stack((out[a], out[b], out[c])), col))
        tris.append((np.stack((inn[a], inn[c], inn[b])), PANEL_RIM))
    for a, b, c in lattice:
        tris.append((np.stack((lat_f[a], lat_f[b], lat_f[c])), bar_col))
        tris.append((np.stack((lat_b[a], lat_b[c], lat_b[b])), bar_col))
    # Watertight rabbet walls stepping from proud white frame (`out`/`inn`) down to recessed fine honeycomb (`lat_f`/`lat_b`).
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
            tris.append((np.stack((ra[a], rb[a], rb[b])), PANEL_RIM))
            tris.append((np.stack((ra[a], rb[b], ra[b])), PANEL_RIM))
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
    """Build recessed expressive anime-mecha ocular camera module with deep sapphire iris and silver ring."""
    cam = (rev([(0.0, 0.0034), (0.0058, 0.0034), (0.0068, 0.0022), (0.0068, -0.0016),
                (0.0056, -0.0026), (0.0048, -0.0018), (0.0, -0.0018)], col=CHASSIS_CF)
           + rev([(0.0049, -0.0015), (0.0066, -0.0015), (0.0067, -0.0030), (0.0059, -0.0038),
                  (0.0049, -0.0034), (0.0049, -0.0020)], col=PANEL_RIM)
           + rev([(0.0044, -0.0022), (0.0049, -0.0022), (0.0049, -0.0037), (0.0044, -0.0038)], fn=44, col=TITANIUM)
           + rev([(0.0017, -0.0019), (0.0044, -0.0019), (0.0044, -0.0038), (0.0030, -0.0041), (0.0017, -0.0041)],
                 fn=44, col=OPTIC_GLASS)
           + rev([(0.0, -0.0019), (0.0017, -0.0019), (0.0017, -0.0041), (0.0, -0.0042)], fn=32, col=OPTIC_DARK)
           + csg.extrude(csg.filleted2d(csg.box2d((-0.0011, 0.0011), (0.0054, 0.0074)), r=0.00038),
                         a=0.0006, b=0.0026, axis=2, colour=TITANIUM, chamfer=0.0003).tris())
    ring = rev(csg.filleted2d([(0.0073, -0.0013), (0.0086, -0.0013), (0.0086, 0.0013), (0.0073, 0.0013)],
                              r=0.00040, n=3), fn=40, col=PANEL_RIM)
    pins = (csg.extrude(csg.circle2d(0.0009, fn=14), a=-0.0098, b=-0.0083, axis=1, colour=TITANIUM).tris()
            + csg.extrude(csg.circle2d(0.0009, fn=14), a=0.0083, b=0.0098, axis=1, colour=TITANIUM).tris())
    return cam, ring, pins


def _pchip(y, xs, ys):
    """Evaluate smooth monotone cubic-blended profile value at `y`."""
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
    """Return smooth `(xw, zf, zb)` at height `y`, curving into a round crown peaking at `226.2 mm`."""
    if y >= 0.156:
        u = min(1.0, max(0.0, (y - 0.156) / 0.0702))
        c = math.sqrt(max(1e-6, 1.0 - u * u))
        return 0.0724 * c, -0.001 + 0.0935 * c, -0.001 - 0.0930 * c
    return _pchip(y, _PROF_Y, _PROF_X), _pchip(y, _PROF_Y, _PROF_ZF), _pchip(y, _PROF_Y, _PROF_ZB)


def head_pt(th_rad, y, radial_off=0.0, nose=True, face_sculpt=True):
    """Evaluate ONE unified cute anime-feminine mecha head surface (youthful cheeks, zero facelift pull)."""
    xw, zf, zb = _profile_at(y)
    zc = 0.5 * (zf + zb)
    rz = max(0.004, 0.5 * (zf - zb))
    st, ct = math.sin(th_rad), math.cos(th_rad)
    ath = abs(th_rad)

    front_w = max(0.0, ct)
    taper_y = max(0.0, min(1.0, (0.076 - y) / 0.072))
    p_exp = 1.98 - 0.08 * front_w * max(0.0, min(1.0, (0.135 - y) / 0.100))
    rx_eff = xw * (1.0 - 0.072 * (front_w ** 1.3) * (taper_y ** 1.15))
    temple = math.exp(-((y - 0.132) / 0.025) ** 2) * math.exp(-((ath - 0.90) / 0.30) ** 2)
    rx_eff -= 0.0022 * temple

    sx = math.copysign(abs(st) ** (2.0 / p_exp), st)
    cz_e = math.copysign(abs(ct) ** (2.0 / p_exp), ct)
    x = rx_eff * sx
    z = zc + rz * cz_e

    if face_sculpt and ct > 0.05:
        y_orb = 0.1000 + 0.0010 * (ath - 0.48)
        d_orb = ((ath - 0.48) / 0.25) ** 2 + ((y - y_orb) / 0.0140) ** 2
        z -= 0.0016 * math.exp(-1.55 * d_orb)

        y_chk = 0.0745 + 0.0068 * min(1.0, max(0.0, (ath - 0.24) / 0.72))
        cheek_soft = math.exp(-((ath - 0.56) / 0.36) ** 2) * math.exp(-((y - y_chk) / 0.0185) ** 2)
        x += math.copysign(0.0020 * cheek_soft, st) if abs(st) > 1e-6 else 0.0
        z += 0.0026 * cheek_soft

        y_br = 0.1128 + 0.0018 * min(1.0, max(0.0, (ath - 0.20) / 0.56))
        z += 0.0020 * math.exp(-((y - y_br) / 0.0068) ** 2) * math.exp(-((ath - 0.48) / 0.38) ** 2)
        aero = math.exp(-((ath - 0.78) / 0.13) ** 2) * math.exp(-((y - 0.156) / 0.032) ** 2)
        x += math.copysign(0.0020 * aero, st) if abs(st) > 1e-6 else 0.0
        z += 0.0020 * aero

        mouth_w = math.exp(-(ath / 0.21) ** 2)
        lip_u = 0.0026 * mouth_w * math.exp(-((y - 0.0505) / 0.0044) ** 2)
        lip_l = 0.0022 * mouth_w * math.exp(-((y - 0.0410) / 0.0046) ** 2)
        mento = -0.0016 * math.exp(-(ath / 0.23) ** 2) * math.exp(-((y - 0.030) / 0.0072) ** 2)
        chin_p = 0.0024 * math.exp(-(ath / 0.28) ** 2) * math.exp(-((y - 0.016) / 0.0095) ** 2)
        z += lip_u + lip_l + mento + chin_p

    if nose and ct > 0.45 and 0.066 <= y <= 0.112:
        tip_h = math.exp(-((y - 0.0735) / 0.0074) ** 2)
        bridge_h = 0.0026 * math.sin(math.pi * max(0.0, min(1.0, (y - 0.067) / 0.044)))
        w_n = 0.060 + 0.028 * tip_h
        n_lat = max(0.0, 1.0 - (ath / w_n) ** 1.45)
        z += (bridge_h + 0.0060 * tip_h) * n_lat

    if radial_off != 0.0:
        r_xz = math.hypot(x, z - zc)
        if r_xz > 1e-6:
            x += radial_off * (x / r_xz)
            z += radial_off * ((z - zc) / r_xz)
    return (x, y, z)


def surf_grid(th_fn, y_fn, nu, nv, off=0.0, nose=True, sculpt=True):
    """Sample `(nu, nv, 3)` panel grid directly from the unified master surface `head_pt`."""
    np = np_mod()
    us, vs = np.linspace(0.0, 1.0, nu), np.linspace(0.0, 1.0, nv)
    g = np.zeros((nu, nv, 3))
    for i, u in enumerate(us):
        fu = float(u)
        for j, v in enumerate(vs):
            fv = float(v)
            g[i, j] = head_pt(float(th_fn(fu, fv)), float(y_fn(fu, fv)), off, nose, sculpt)
    return g


def planar_grid(grid, blend=0.58):
    """Flatten interior curvature of a small mecha sub-panel toward its bilinear corner facet."""
    np = np_mod()
    nu, nv = grid.shape[:2]
    us = np.linspace(0.0, 1.0, nu)[:, None, None]
    vs = np.linspace(0.0, 1.0, nv)[None, :, None]
    c00, c10, c11, c01 = grid[0, 0], grid[-1, 0], grid[-1, -1], grid[0, -1]
    bilin = (1.0 - us) * ((1.0 - vs) * c00 + vs * c01) + us * ((1.0 - vs) * c10 + vs * c11)
    return (1.0 - blend) * grid + blend * bilin


def mecha_tiles(th_fn, y_fn, tiles, wall=0.0023, ch=0.00055, flat=0.55, nose=False, col=SHELL_CF):
    """Build interlocking small planar-faceted mecha armor tiles `(u0, u1, v0, v1, off, nu, nv)` on `head_pt`."""
    out = []
    for u0, u1, v0, v1, off, nu, nv in tiles:
        g = surf_grid(lambda u, v, a=u0, b=u1, c=v0, d=v1: th_fn(a + (b - a) * u, c + (d - c) * v),
                      lambda u, v, a=u0, b=u1, c=v0, d=v1: y_fn(a + (b - a) * u, c + (d - c) * v),
                      nu, nv, off=off, nose=nose)
        out += shell_patch(planar_grid(g, flat) if flat > 0.0 else g, wall, ch, col)
    return out


def eye_top(th):
    """Open, softly arched cute anime-almond upper eye contour (`+1.5 deg` gentle poise, `16.0 mm` tall)."""
    t = max(0.0, min(1.0, (th - 0.205) / 0.575))
    return 0.0994 + 0.0012 * t + 0.0090 * math.sin(math.pi * (t ** 0.92))


def eye_bot(th):
    """Gentle, relaxed lower anime-almond eye contour (zero upward facelift pull)."""
    t = max(0.0, min(1.0, (th - 0.205) / 0.575))
    return 0.0982 + 0.0012 * t - 0.0070 * math.sin(math.pi * (t ** 0.90))


def crown_seam_y(th):
    """Swept V-chevron mecha crown seam dipping gracefully at the forehead midline and sweeping back."""
    t = min(1.0, abs(th) / 0.85)
    return 0.1670 + 0.0185 * (t ** 0.88)


def ear_clear_y(th, y_base):
    """Lift lower temporal border cleanly over the circular ear-mic pod at `(y = 93, z = 4 mm)`."""
    p = head_pt(th, y_base, 0.0, False, False)
    dz = p[2] - 0.004
    if abs(dz) < 0.0268:
        y_arc = 0.093 + math.sqrt(max(0.0, 0.0268 ** 2 - dz * dz))
        return 0.5 * (y_base + y_arc + math.hypot(y_base - y_arc, 0.0036))
    return y_base
