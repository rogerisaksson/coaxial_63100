"""Beveled armor plates, skew-free surface triangulation, curved honeycombs, Torx seats and optics."""
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg  # noqa: E402

SHELL_CF, PANEL_RIM, SEAM_SILVER, CHASSIS_CF, TITANIUM = (245, 244, 240), (50, 52, 58), (190, 192, 196), (32, 34, 40), (146, 150, 158)
CRANIUM_DARK, CRANIUM_RIM, AMBER_GOLD, AMBER_DARK = (108, 108, 110), (54, 56, 60), (232, 158, 76), (158, 96, 28)
HEX_DARK, FRAME_VOID, BOLT_RIM, BOLT_SOCK = (40, 42, 48), (14, 16, 20), (32, 34, 38), (10, 12, 14)
MOUTH_SCREEN, LIP_TINT, LIP_LINE = (254, 254, 255), (222, 196, 206), (184, 160, 170)
ANODIZED, SERVO_CASE, HORN_BRASS, MESH_PEARL = (226, 48, 178), (28, 32, 40), (206, 168, 84), (234, 236, 240)
NECK_SKIN, OPTIC_DARK, OPTIC_GLASS, OPTIC_SCLERA = (50, 53, 58), (8, 10, 14), (40, 46, 56), (154, 160, 168)
BAFFLE_DARK, MIC_GOLD, BROW_DARK = (12, 14, 18), (220, 138, 44), (14, 16, 20)
PORCELAIN, HEX_WALL = (234, 236, 240), (36, 38, 44)
SCREW_FREE_PANELS = False
FASTENER_LOG = []


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


def shell_patch(grid, wall=0.0026, chamfer=0.00055, col=CRANIUM_DARK, ref=(0.0, 0.115, 0.015), rim_col=CRANIUM_RIM):
    """Build a watertight contoured armor panel with a distinct bevel rim (`rim_col`)."""
    np = np_mod()
    g, du, dv, nrm, ch = _surface_frame(grid, wall, chamfer, ref)
    nu, nv = g.shape[:2]
    su, sv = np.maximum(np.linalg.norm(du, axis=-1, keepdims=True), 1e-12), np.maximum(np.linalg.norm(dv, axis=-1, keepdims=True), 1e-12)
    ins = np.zeros_like(g)
    ins[0] += np.minimum(ch, 0.28 * su[0]) * (du[0] / su[0])
    ins[-1] -= np.minimum(ch, 0.28 * su[-1]) * (du[-1] / su[-1])
    ins[:, 0] += np.minimum(ch, 0.28 * sv[:, 0]) * (dv[:, 0] / sv[:, 0])
    ins[:, -1] -= np.minimum(ch, 0.28 * sv[:, -1]) * (dv[:, -1] / sv[:, -1])
    outer, inner, co_g, ci_g = g + ins, g - float(wall) * nrm + ins, g - ch * nrm, g - (float(wall) - ch) * nrm
    rcol, tris = col if rim_col is None else rim_col, []
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


def socket_bolt(pt, nrm, r_out=0.0022, h_out=0.0005, col_rim=BOLT_RIM, col_sock=BOLT_SOCK, fn=24):
    """Build flush 6-lobe Torx bolt (or screw-free counterbore seat when `SCREW_FREE_PANELS` is active)."""
    np = np_mod()
    n = np.asarray(nrm, float)
    n /= max(float(np.linalg.norm(n)), 1e-12)
    up = np.array([0.0, 1.0, 0.0]) if abs(float(n[1])) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = np.cross(up, n)
    e1 /= max(float(np.linalg.norm(e1)), 1e-12)
    e2, p0, is_screw = np.cross(n, e1), np.asarray(pt, float), (col_rim == BOLT_RIM)
    if SCREW_FREE_PANELS and is_screw:
        spec = 'M3' if float(r_out) >= 0.0028 else 'M2'
        FASTENER_LOG.append((tuple(float(v) for v in p0), tuple(float(v) for v in n), spec))
        r_cb, r_cl = (0.00295, 0.00160) if spec == 'M3' else (0.00210, 0.00110)
        ang = np.linspace(0.0, 2.0 * math.pi, 18, endpoint=False)
        ca, sa = np.cos(ang), np.sin(ang)
        ring = lambda rad, h: p0[None, :] + float(h) * n[None, :] + rad * (ca[:, None] * e1[None, :] + sa[:, None] * e2[None, :])
        rc0, rc1, rc2, rc3 = ring(r_cb, 0.00005), ring(r_cb, -0.0009), ring(r_cl, -0.0009), ring(r_cl, -0.0018)
        cb_top, cb_bot, tris, m = p0 + 0.00005 * n, p0 - 0.0018 * n, [], len(ang)
        for ra, rb in ((rc0, rc1), (rc1, rc2), (rc2, rc3)):
            tris += _stitch_rings(rb, ra, col_sock)
        for k in range(m):
            kn = (k + 1) % m
            tris += [(np.stack((cb_top, rc0[kn], rc0[k])), col_sock), (np.stack((cb_bot, rc3[k], rc3[kn])), col_sock)]
        return tris
    m = max(18, int(fn)) if is_screw else int(fn)
    ang = np.linspace(0.0, 2.0 * math.pi, m, endpoint=False)
    ca, sa = np.cos(ang), np.sin(ang)
    star = (0.50 * float(r_out) * (0.72 + 0.28 * np.cos(6.0 * ang))) if is_screw else np.full(m, 0.56 * float(r_out))
    ring = lambda rad, h: p0[None, :] + float(h) * n[None, :] + (np.asarray(rad, float)[:, None] if np.ndim(rad) > 0 else float(rad)) * (ca[:, None] * e1[None, :] + sa[:, None] * e2[None, :])
    r0, r1, r2, r3, r4 = ring(r_out, -0.0008), ring(r_out, 0.35 * h_out), ring(0.86 * float(r_out), h_out), ring(star, h_out), ring(0.92 * star, 0.12 * h_out)
    cb, ct, tris = p0 - 0.0008 * n, p0 + 0.10 * h_out * n, []
    for ra, rb, col in ((r0, r1, col_rim), (r1, r2, col_rim), (r2, r3, col_rim), (r3, r4, col_sock)):
        tris += _stitch_rings(ra, rb, col)
    for k in range(m):
        kn = (k + 1) % m
        tris += [(np.stack((ct, r4[k], r4[kn])), col_sock), (np.stack((cb, r0[kn], r0[k])), col_rim)]
    return tris


def _orient_uv(surf_fn, loop_uv, off):
    return np_mod().asarray([surf_fn(float(u), float(v), float(off)) for u, v in loop_uv], float)


def _winding_flip(ring_3d, ctr_3d, nrm_3d):
    np = np_mod()
    acc = sum(np.cross(ring_3d[k] - ctr_3d, ring_3d[(k + 1) % len(ring_3d)] - ctr_3d) for k in range(len(ring_3d)))
    return float(np.dot(acc, nrm_3d)) < 0.0


def _stitch_rings(ra, rb, col):
    np, m, out = np_mod(), len(ra), []
    for k in range(m):
        kn = (k + 1) % m
        out += [(np.stack((ra[k], ra[kn], rb[kn])), col), (np.stack((ra[k], rb[kn], rb[k])), col)]
    return out


def _add_bolts(surf_fn, off, bolts_uv, bolt_r, nrm_fn=None):
    np, tris = np_mod(), []
    for bu, bv in bolts_uv:
        bp = np.asarray(surf_fn(float(bu), float(bv), float(off)), float)
        bn = (np.asarray(nrm_fn(float(bu), float(bv)), float) if nrm_fn is not None
              else bp - np.asarray(surf_fn(float(bu), float(bv), float(off - 0.002)), float))
        tris += socket_bolt(bp, bn, r_out=float(bolt_r))
    return tris


def _subdiv_cap_and_rim(p_out, p_ch, max_edge):
    """Ear-clip clean non-collinear `p_ch`, then split interior AND boundary edges > `max_edge` in lockstep."""
    from tools.render import foot
    np = np_mod()
    verts = [(float(p[0]), float(p[1])) for p in p_ch]
    out_v = [(float(p[0]), float(p[1])) for p in p_out]
    b_chain = list(range(len(verts)))
    tris = [(int(t[0]), int(t[1]), int(t[2])) for t in foot._ears(np.asarray(verts, float))]
    lim2 = float(max_edge) ** 2
    for _ in range(5):
        mids, new_chain, nb = {}, [], len(b_chain)
        for k in range(nb):
            u, v = b_chain[k], b_chain[(k + 1) % nb]
            new_chain.append(u)
            key = (u, v) if u < v else (v, u)
            if (verts[u][0] - verts[v][0]) ** 2 + (verts[u][1] - verts[v][1]) ** 2 > lim2:
                mi = len(verts)
                mids[key] = mi
                verts.append((0.5 * (verts[u][0] + verts[v][0]), 0.5 * (verts[u][1] + verts[v][1])))
                out_v.append((0.5 * (out_v[u][0] + out_v[v][0]), 0.5 * (out_v[1 * v][1] if False else out_v[v][1]) + 0.5 * out_v[u][1]))
                new_chain.append(mi)
        b_chain = new_chain
        for a, b, c in tris:
            for u, v in ((a, b), (b, c), (c, a)):
                key = (u, v) if u < v else (v, u)
                if key not in mids and (verts[u][0] - verts[v][0]) ** 2 + (verts[u][1] - verts[v][1]) ** 2 > lim2:
                    mids[key] = len(verts)
                    verts.append((0.5 * (verts[u][0] + verts[v][0]), 0.5 * (verts[u][1] + verts[v][1])))
                    out_v.append((0.0, 0.0))
        if not mids:
            break
        nxt = []
        for a, b, c in tris:
            m0, m1, m2 = mids.get((a, b) if a < b else (b, a)), mids.get((b, c) if b < c else (c, b)), mids.get((c, a) if c < a else (a, c))
            cnt = (m0 is not None) + (m1 is not None) + (m2 is not None)
            if cnt == 0:
                nxt.append((a, b, c))
            elif cnt == 3:
                assert m0 is not None and m1 is not None and m2 is not None
                nxt += [(a, m0, m2), (m0, b, m1), (m2, m1, c), (m0, m1, m2)]
            elif cnt == 1:
                x, y, z, m = (a, b, c, m0) if m0 is not None else ((b, c, a, m1) if m1 is not None else (c, a, b, m2))
                assert m is not None
                nxt += [(x, m, z), (m, y, z)]
            else:
                x, y, z, ma, mb = (a, b, c, m0, m1) if m2 is None else ((b, c, a, m1, m2) if m0 is None else (c, a, b, m2, m0))
                assert ma is not None and mb is not None
                d_xm = (verts[x][0] - verts[mb][0]) ** 2 + (verts[x][1] - verts[mb][1]) ** 2
                d_zm = (verts[z][0] - verts[ma][0]) ** 2 + (verts[z][1] - verts[ma][1]) ** 2
                nxt += [(ma, y, mb)] + ([(x, ma, mb), (x, mb, z)] if d_xm <= d_zm else [(x, ma, z), (ma, mb, z)])
        tris = nxt
    return np.asarray(verts, float), tris, b_chain, np.asarray([out_v[i] for i in b_chain], float)


def curved_poly_plate(poly_uv, surf_fn, wall=0.0030, chamfer=0.00085, corner_r=0.0034,
                      col=CRANIUM_DARK, rim_col=CRANIUM_RIM, off=0.0, n_rings=3, step=0.007,
                      bolts_uv=(), bolt_r=0.0028, nrm_fn=None):
    """Build a skew-free watertight curved beveled polygonal armor plate on `surf_fn(u, v, off)`."""
    np, _ = np_mod(), n_rings
    p0 = csg.oriented(np.asarray(poly_uv, float), ccw=True, clean=True)
    p1 = csg.oriented(csg.filleted2d(p0, r=float(corner_r), n=3), ccw=True, clean=True) if corner_r > 1e-4 else p0
    span = min(float(p1[:, 0].max() - p1[:, 0].min()), float(p1[:, 1].max() - p1[:, 1].min()))
    bev_w = min(1.02 if span > 1.0 else 0.00102, 0.11 * span)
    p_ch = csg.inset2d(p1, bev_w, outer=True)
    if len(p_ch) != len(p1):
        ctr0 = p1.mean(0)
        p_ch = ctr0 + 0.90 * (p1 - ctr0)
    v_uv, cap_t, b_chain, b_out = _subdiv_cap_and_rim(p1, p_ch, max(float(step), 0.0042))
    v_top, v_bot = _orient_uv(surf_fn, v_uv, off), _orient_uv(surf_fn, v_uv, off - wall)
    t_bev = _orient_uv(surf_fn, b_out, off - min(float(chamfer), 0.42 * float(wall)))
    b_bev = _orient_uv(surf_fn, b_out, off - wall)
    ctr = b_out.mean(0)
    c_top = np.asarray(surf_fn(float(ctr[0]), float(ctr[1]), float(off)), float)
    c_bot = np.asarray(surf_fn(float(ctr[0]), float(ctr[1]), float(off - wall)), float)
    r_top0, r_bot0 = v_top[b_chain], v_bot[b_chain]
    flip = _winding_flip(r_top0, c_top, c_top - c_bot)
    ch_col = (102, 106, 114) if col == CRANIUM_DARK else rim_col
    wl_col = (50, 53, 60) if col == CRANIUM_DARK else rim_col
    tris = []
    for a, b, c in cap_t:
        i0, i1, i2 = (a, c, b) if flip else (a, b, c)
        tris += [(np.stack((v_top[i0], v_top[i1], v_top[i2])), col),
                 (np.stack((v_bot[i0], v_bot[i2], v_bot[i1])), rim_col)]
    rt, rb, tb, bb = (r_top0[::-1], r_bot0[::-1], t_bev[::-1], b_bev[::-1]) if flip else (r_top0, r_bot0, t_bev, b_bev)
    tris += _stitch_rings(rt, tb, ch_col) + _stitch_rings(tb, bb, wl_col) + _stitch_rings(bb, rb, rim_col)
    return tris + _add_bolts(surf_fn, off, bolts_uv, bolt_r, nrm_fn)


def _point_in_poly(xy, poly):
    np = np_mod()
    x, y, px, py = xy[:, 0], xy[:, 1], poly[:, 0], poly[:, 1]
    qx, qy, inside = np.roll(px, -1), np.roll(py, -1), np.zeros(len(xy), dtype=bool)
    for i in range(len(poly)):
        inside ^= ((py[i] > y) != (qy[i] > y)) & (x < (qx[i] - px[i]) * (y - py[i]) / ((qy[i] - py[i]) + 1e-18) + px[i])
    return inside


def _smooth_inset_pair(p_out, rim_w, step):
    from tools.render import foot
    np, n, lim = np_mod(), len(p_out), 1.38 * float(rim_w)
    p_in = np.zeros_like(p_out)
    for i in range(n):
        e0, e1 = p_out[i] - p_out[(i - 1) % n], p_out[(i + 1) % n] - p_out[i]
        l0, l1 = max(float(np.linalg.norm(e0)), 1e-9), max(float(np.linalg.norm(e1)), 1e-9)
        n0, n1 = np.array([-e0[1] / l0, e0[0] / l0]), np.array([-e1[1] / l1, e1[0] / l1])
        mit = n0 + n1
        ml = float(np.linalg.norm(mit))
        p_in[i] = p_out[i] + (float(rim_w) * n0 if ml < 1e-6 else min(lim, float(rim_w) / max(0.52, float(np.dot(mit / ml, n0)))) * (mit / ml))
    p_in = 0.22 * np.roll(p_in, 1, axis=0) + 0.56 * p_in + 0.22 * np.roll(p_in, -1, axis=0)
    bo, bi = [], []
    for i in range(n):
        j = (i + 1) % n
        k = max(1, int(math.ceil(max(float(np.linalg.norm(p_out[j] - p_out[i])), float(np.linalg.norm(p_in[j] - p_in[i]))) / float(step))))
        for s in range(k):
            t = s / k
            bo.append(p_out[i] + t * (p_out[j] - p_out[i]))
            bi.append(p_in[i] + t * (p_in[j] - p_in[i]))
    ctr0 = p_out.mean(0)
    span = max(1e-4, min(float(p_out[:, 0].max() - p_out[:, 0].min()), float(p_out[:, 1].max() - p_out[:, 1].min())))
    p_back = ctr0 + max(0.38, 1.0 - 1.9 * float(rim_w) / span) * (p_out - ctr0)
    for cand in (csg.inset2d(p_out, float(rim_w), outer=True), p_in, 0.5 * p_in + 0.5 * p_back, p_back):
        cl = csg.oriented(csg.clean2d(cand, eps=max(1e-4, 0.05 * float(step))), ccw=True, clean=True)
        if len(cl) >= 3 and len(foot._ears(np.asarray(cl, float))) == len(cl) - 2:
            p_back = cl
            break
    return p_back, np.asarray(bo, float), np.asarray(bi, float)


def curved_honeycomb_bay(poly_uv, surf_fn, rim_w=0.0052, wall=0.0032, chamfer=0.0009,
                         pitch_u=0.0054, pitch_v=0.0054, fill=0.72, corner_r=0.0040,
                         frame_col=AMBER_GOLD, rim_col=AMBER_DARK, hex_col=HEX_DARK,
                         hole_col=BAFFLE_DARK, back_col=FRAME_VOID, off=0.0,
                         bolts_uv=(), bolt_r=0.0026, nrm_fn=None, step=0.0060, add_back=True):
    """Build watertight curved beveled frame enclosing 3D dark hexagonal honeycomb."""
    np, ml = np_mod(), max(0.45 * float(rim_w), 0.75 * float(corner_r))
    pts = [poly_uv[0]]
    for pt in poly_uv[1:]:
        if math.hypot(pt[0] - pts[-1][0], pt[1] - pts[-1][1]) >= ml:
            pts.append(pt)
    while len(pts) > 6 and math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < ml:
        pts.pop()
    p0 = csg.oriented(np.asarray(pts if len(pts) >= 6 else poly_uv, float), ccw=True, clean=True)
    p_out = csg.oriented(csg.filleted2d(p0, r=float(corner_r), n=3), ccw=True, clean=True) if corner_r > 1e-4 else p0
    p_in_clean, b_out, b_in = _smooth_inset_pair(p_out, float(rim_w), float(step))
    ctr = b_out.mean(0)
    c_top, c_bot = np.asarray(surf_fn(float(ctr[0]), float(ctr[1]), float(off)), float), np.asarray(surf_fn(float(ctr[0]), float(ctr[1]), float(off - wall)), float)
    if _winding_flip(_orient_uv(surf_fn, b_out, off), c_top, c_top - c_bot):
        b_out, b_in = b_out[::-1].copy(), b_in[::-1].copy()

    ch = min(float(chamfer), 0.38 * float(wall))
    ch_col = (206, 140, 50) if frame_col == AMBER_GOLD else ((104, 107, 114) if frame_col == CRANIUM_DARK else rim_col)
    r_ft_i, r_ft_o = _orient_uv(surf_fn, b_in + 0.14 * (b_out - b_in), off), _orient_uv(surf_fn, b_out + 0.11 * (b_in - b_out), off)
    r_ch_o, r_bt_o = _orient_uv(surf_fn, b_out, off - ch), _orient_uv(surf_fn, b_out, off - wall)
    r_bt_i, r_ch_i = _orient_uv(surf_fn, b_in, off - wall), _orient_uv(surf_fn, b_in, off - ch)
    tris = [t for ra, rb, c in ((r_ft_i, r_ft_o, frame_col), (r_ft_o, r_ch_o, ch_col), (r_ch_o, r_bt_o, rim_col),
                                (r_bt_o, r_bt_i, rim_col), (r_bt_i, r_ch_i, rim_col), (r_ch_i, r_ft_i, ch_col)) for t in _stitch_rings(ra, rb, c)]
    if add_back:
        tris += curved_poly_plate(p_in_clean, surf_fn, wall=0.0011 * (float(wall) / 0.0032), chamfer=0.00018, corner_r=0.0,
                                  col=back_col, rim_col=back_col, off=off - wall - 0.16 * float(wall), step=step * 1.4)
    p_in, p_hl = p_in_clean, b_in + 0.06 * (b_out - b_in)
    u0, u1, v0, v1 = float(p_in[:, 0].min()), float(p_in[:, 0].max()), float(p_in[:, 1].min()), float(p_in[:, 1].max())
    mu, mv, pu, pv = 0.5 * (u0 + u1), 0.5 * (v0 + v1), float(pitch_u), float(pitch_v)
    rise = pv * math.sqrt(3.0) * 0.5
    nu_c, nv_c = int((u1 - u0) / (2.0 * pu)) + 2, int((v1 - v0) / (2.0 * rise)) + 2
    ang = np.arange(6) * (math.pi / 3.0) + math.pi / 6.0
    uo = np.stack([np.cos(ang) * (pu / math.sqrt(3.0)), np.sin(ang) * (pv / math.sqrt(3.0))], 1)
    ui, ht, hb = uo * float(fill), off - max(0.00090, ch + 0.00032), off - wall + 0.00028
    flip_h = _winding_flip(_orient_uv(surf_fn, np.array([[mu, mv]]) + uo, ht),
                           np.asarray(surf_fn(mu, mv, ht), float), c_top - c_bot)
    for jr in range(-nv_c, nv_c + 1):
        cv, us_off = mv + jr * rise, (0.5 * pu if (jr & 1) else 0.0)
        for ic in range(-nu_c, nu_c + 1):
            cu = mu + ic * pu + us_off
            if not _point_in_poly(np.array([[cu, cv]]), p_in)[0]:
                continue
            o6, i6 = np.array([cu, cv]) + uo, np.array([cu, cv]) + ui
            if not _point_in_poly(o6, p_hl).all():
                continue
            ot, ob, it, ib = _orient_uv(surf_fn, o6, ht), _orient_uv(surf_fn, o6, hb), _orient_uv(surf_fn, i6, ht), _orient_uv(surf_fn, i6, hb)
            if flip_h:
                ot, ob, it, ib = ot[::-1], ob[::-1], it[::-1], ib[::-1]
            for k in range(6):
                kn = (k + 1) % 6
                tris += [
                    (np.stack((ot[k], ot[kn], it[kn])), hex_col), (np.stack((ot[k], it[kn], it[k])), hex_col),
                    (np.stack((ob[k], ib[kn], ob[kn])), hex_col), (np.stack((ob[k], ib[k], ib[kn])), hex_col),
                    (np.stack((it[k], ib[k], ib[kn])), hole_col), (np.stack((it[k], ib[kn], it[kn])), hole_col),
                    (np.stack((ot[k], ot[kn], ob[kn])), hole_col), (np.stack((ot[k], ob[kn], ob[k])), hole_col),
                ]
    return tris + _add_bolts(surf_fn, off, bolts_uv, bolt_r, nrm_fn)


def standard_torx_screw(spec='M2', col_head=None, col_sock=None):
    """Build watertight ISO 14580 Torx screw (`'M2'` or `'M3'`) along +Z (m)."""
    np = np_mod()
    ch, cs = BOLT_RIM if col_head is None else col_head, BOLT_SOCK if col_sock is None else col_sock
    r_sh, l_sh, r_hd, h_hd, r_tx = ((0.0010, 0.0060, 0.00190, 0.00165, 0.00088) if spec == 'M2'
                                    else (0.0015, 0.0080, 0.00275, 0.00240, 0.00135))
    ang = np.linspace(0.0, 2.0 * math.pi, 36, endpoint=False)
    ca, sa, star = np.cos(ang), np.sin(ang), r_tx * (0.72 + 0.28 * np.cos(6.0 * ang))
    cr = lambda r, z: np.c_[(np.full(36, float(r)) if np.ndim(r) == 0 else np.asarray(r, float)) * ca,
                            (np.full(36, float(r)) if np.ndim(r) == 0 else np.asarray(r, float)) * sa, np.full(36, float(z))]
    rings = (cr(0.72 * r_sh, -l_sh), cr(r_sh, -l_sh + 0.28 * r_sh), cr(r_sh, 0.0), cr(r_hd, 0.0),
             cr(r_hd, 0.72 * h_hd), cr(0.88 * r_hd, h_hd), cr(star, h_hd), cr(0.92 * star, 0.28 * h_hd))
    cb, ct_p, tris = np.array([0.0, 0.0, -l_sh]), np.array([0.0, 0.0, 0.22 * h_hd]), []
    for ra, rb, c in zip(rings[:-1], rings[1:], (ch, ch, ch, ch, ch, ch, cs)):
        tris += _stitch_rings(ra, rb, c)
    return tris + [t for k in range(36) for kn in ((k + 1) % 36,) for t in ((np.stack((cb, rings[0][kn], rings[0][k])), ch), (np.stack((ct_p, rings[7][k], rings[7][kn])), cs))]


def place_screw(screw_local, pt, nrm):
    """Orient `+Z` `screw_local` triangles along unit normal `nrm` seated at `pt`."""
    np = np_mod()
    n = np.asarray(nrm, float)
    n /= max(float(np.linalg.norm(n)), 1e-12)
    up = np.array([0.0, 1.0, 0.0]) if abs(float(n[1])) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = np.cross(up, n)
    e1 /= max(float(np.linalg.norm(e1)), 1e-12)
    basis, p0 = np.stack([e1, np.cross(n, e1), n], axis=0), np.asarray(pt, float)
    return [(t @ basis + p0, c) for t, c in screw_local]


def neck_band(yn, tri):
    """Return quad-midpoint cervical band RGB for loft triangle `tri`."""
    ym = 0.5 * (float(yn[tri].min()) + float(yn[tri].max()))
    if ym < -0.004:
        return SHELL_CF
    if ym < 0.004:
        return BAFFLE_DARK
    return PANEL_RIM if (0.018 <= ym < 0.024 or 0.034 <= ym < 0.040 or 0.050 <= ym < 0.056) else CHASSIS_CF


def throat_bellows_hoses(to_neck_fn):
    """Build 3 S-curved corrugated robotic throat hoses (1 central + 2 flanking)."""
    np = np_mod()
    zf = lambda y: -0.6 + 2.9 * (((float(y) - 26.0) / 30.0) ** 2)
    ys = np.linspace(-8.0, 56.0, 15)
    hc = csg.sweep(csg.circle2d(3.05, fn=16), [(0.0, float(y), zf(y)) for y in ys], chamfer_ends=0.45, colour=CHASSIS_CF).tris()
    hl = csg.sweep(csg.circle2d(2.45, fn=14), [(8.4 + 1.3 * (float(y) + 8.0) / 64.0, float(y), zf(y) - 1.15) for y in ys], chamfer_ends=0.38, colour=CHASSIS_CF).tris()
    for k, ry in enumerate(np.linspace(-3.0, 53.0, 19)):
        fy, rc, z0, u0 = float(ry), (CRANIUM_DARK if (k & 1) == 0 else FRAME_VOID), zf(ry), (float(ry) + 8.0) / 64.0
        hc += csg.extrude(csg.circle2d(4.05, fn=16, at=(z0, 0.0)), a=fy - 0.74, b=fy + 0.74, axis=1, colour=rc, chamfer=0.28).tris()
        hl += csg.extrude(csg.circle2d(3.22, fn=14, at=(z0 - 1.15, 8.4 + 1.3 * u0)), a=fy - 0.68, b=fy + 0.68, axis=1, colour=rc, chamfer=0.25).tris()
    hc += csg.extrude(csg.filleted2d(csg.box2d((-3.2, 5.2), (-13.5, 13.5)), r=1.2), a=54.2, b=57.2, axis=1, colour=CRANIUM_DARK, chamfer=0.45).tris()
    nl = to_neck_fn(hl)
    return {'neck_conduit_center': to_neck_fn(hc), 'neck_conduits_L': nl, 'neck_conduits_R': mirror_x(nl)}
