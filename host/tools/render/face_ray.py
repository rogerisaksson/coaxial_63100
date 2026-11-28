"""Full-resolution camera-ray 45-degree CAD plates, through-bores and native studio cameras."""
import functools
import math
import os
import sys
from typing import Any

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg, face_honeycomb as fh  # noqa: E402

SCALE_M = 0.001292
HEAD_CENTER_Y, SURFACE_ARC_R, FRONT_MID_PX = 145.0, 68.0, 448.0
ORIGIN_HEAD_M = (0.0, -0.0798, 0.0390)

CANONICAL_CAMERAS = {
    'front': {'fov': 11.4, 'pos': (0.0, 125.0, 1180.0), 'target': (0.0, 121.0, -14.0), 'w': 896, 'h': 1200},
    'profile90': {'fov': 12.1, 'pos': (-1125.0, 148.0, -12.0), 'target': (8.0, 121.0, -12.0), 'w': 1200, 'h': 896},
    'threeQuarter': {'fov': 11.8, 'pos': (-948.64, 108.05, 586.46), 'target': (-2.0, 119.5, -12.0), 'w': 1200, 'h': 896},
    'rear': {'fov': 12.2, 'pos': (0.0, 134.0, -1125.0), 'target': (0.0, 117.0, -38.0), 'w': 896, 'h': 1200},
}
_SPEC_TO_PRESET = {id(v): k for k, v in CANONICAL_CAMERAS.items()}


def canon_to_head(pt_mm):
    return fh.np_mod().asarray(pt_mm, float) * SCALE_M + ORIGIN_HEAD_M


def head_to_canon(pt_m):
    return (fh.np_mod().asarray(pt_m, float) - ORIGIN_HEAD_M) / SCALE_M


def canon_tris_to_head(tris_mm):
    np = fh.np_mod()
    return [(np.asarray(t, float) * SCALE_M + ORIGIN_HEAD_M, col) for t, col in tris_mm]


def stl_roundtrip_tris(tris_m):
    if not tris_m:
        return []
    q = (fh.np_mod().array([t for t, _c in tris_m], float) * 1e3).astype('f4').astype('f8') * 1e-3
    return [(q[i], tris_m[i][1]) for i in range(len(tris_m))]


_CAM_DELTA_FN: list[Any] = [None]


def _norm_delta(cam_delta, preset_id=None):
    cd = _CAM_DELTA_FN[0](preset_id) if (cam_delta is None and preset_id and _CAM_DELTA_FN[0]) else cam_delta
    t = tuple(float(x) for x in cd) if cd else ()
    return t + (0.0,) * max(0, 7 - len(t))


@functools.lru_cache(maxsize=64)
def _cam_basis_key(fov, pos, target, w, h, d7):
    dfov, dtx, dty, dtz, dcx, dcy, droll = d7
    ox, oy, oz = pos
    tx, ty, tz = target[0] + dtx, target[1] + dty, target[2] + dtz
    fx, fy, fz = tx - ox, ty - oy, tz - oz
    d = max(math.sqrt(fx * fx + fy * fy + fz * fz), 1e-12)
    fx, fy, fz = fx / d, fy / d, fz / d
    rl = max(math.hypot(fz, fx), 1e-12)
    rx0, rz0 = -fz / rl, fx / rl
    ux0, uy0, uz0 = -rz0 * fy, rz0 * fx - rx0 * fz, rx0 * fy
    cr, sr = math.cos(math.radians(droll)), math.sin(math.radians(droll))
    rx, ry, rz = cr * rx0 + sr * ux0, sr * uy0, cr * rz0 + sr * uz0
    ux, uy, uz = -sr * rx0 + cr * ux0, cr * uy0, -sr * rz0 + cr * uz0
    return ((ox, oy, oz), (tx, ty, tz), (fx, fy, fz), (rx, ry, rz), (ux, uy, uz),
            d, math.tan(math.radians(0.5 * (fov + dfov))), w / h, float(w), float(h), dcx, dcy)


def _cam_basis(spec, cam_delta=None, preset_id=None):
    return _cam_basis_key(float(spec['fov']), tuple(spec['pos']), tuple(spec['target']),
                          float(spec['w']), float(spec['h']), _norm_delta(cam_delta, preset_id or _SPEC_TO_PRESET.get(id(spec))))


def gpu_camera_for(preset_id, head_torso, mult=1.0, cam_delta=None):
    np, fm = fh.np_mod(), float(mult)
    _o, tc, fwd, right, up, dc, th, _a, w0, h0, dcx, dcy = _cam_basis(CANONICAL_CAMERAS[preset_id], cam_delta, preset_id)
    w, h, c = int(round(w0 * fm)), int(round(h0 * fm)), canon_to_head(tc) + np.asarray(head_torso, float)
    cam = {'width': w, 'height': h, 'cx': 0.5 * w - 0.5 + dcx * fm, 'cy': 0.5 * h - 0.5 + dcy * fm, 'scale': 0.5 * h / th, 'aspect': 1.0, 'distance': dc * SCALE_M}
    return (*right, *up, -fwd[0], -fwd[1], -fwd[2]), cam, (float(c[0]), float(c[1]), float(c[2]))


def project_uv_batch(pts_canon_mm, preset_id, cam_delta=None):
    np = fh.np_mod()
    (ox, oy, oz), _t, fwd, right, up, _d, th, _a, w, h, dcx, dcy = _cam_basis(CANONICAL_CAMERAS[preset_id], cam_delta, preset_id)
    pts = np.asarray(pts_canon_mm, float)
    vx, vy, vz = pts[:, 0] - ox, pts[:, 1] - oy, pts[:, 2] - oz
    sc_z = (0.5 * h / th) / np.maximum(1e-6, vx * fwd[0] + vy * fwd[1] + vz * fwd[2])
    return (0.5 * w - 0.5 + dcx) + sc_z * (vx * right[0] + vy * right[1] + vz * right[2]), (0.5 * h - 0.5 + dcy) - sc_z * (vx * up[0] + vy * up[1] + vz * up[2]), int(w), int(h)


def _ray_spec(px, py, cam_spec, cam_delta=None):
    (ox, oy, oz), _t, (fx, fy, fz), (rx, ry, rz), (ux, uy, uz), _d, th, asp, w, h, dcx, dcy = _cam_basis(cam_spec, cam_delta)
    sx, sy = (((float(px) - dcx) / w) * 2.0 - 1.0) * asp * th, (1.0 - ((float(py) - dcy) / h) * 2.0) * th
    dx, dy, dz_d = fx + rx * sx + ux * sy, fy + ry * sx + uy * sy, fz + rz * sx + uz * sy
    il = 1.0 / max(math.sqrt(dx * dx + dy * dy + dz_d * dz_d), 1e-12)
    return (ox, oy, oz), (dx * il, dy * il, dz_d * il)


def _secant_t(g_fn, lo, hi, lo2, hi2):
    glo, ghi = g_fn(lo), g_fn(hi)
    if glo * ghi > 0.0:
        lo, hi, glo, ghi = lo2, hi2, g_fn(lo2), g_fn(hi2)
    tm = 0.5 * (lo + hi)
    for _ in range(12):
        sec = (lo - glo * (hi - lo) / (ghi - glo)) if abs(ghi - glo) > 1e-9 else 0.5 * (lo + hi)
        tm = sec if (lo + 1e-4 < sec < hi - 1e-4) else 0.5 * (lo + hi)
        gm = g_fn(tm)
        if abs(gm) < 1e-6 or (hi - lo) < 1e-6:
            break
        lo, hi, glo, ghi = (lo, tm, glo, gm) if glo * gm <= 0.0 else (tm, hi, gm, ghi)
    return tm


def pixel_ray(px, py, preset_id, cam_delta=None):
    return _ray_spec(px, py, CANONICAL_CAMERAS[preset_id], _norm_delta(cam_delta, preset_id))


def project_ray_z(px, py, cam_spec, eval_z, dz=0.0, cam_delta=None):
    (ox, oy, oz), (dx, dy, dz_d) = _ray_spec(px, py, cam_spec, cam_delta)
    fdz = float(dz)
    tm = _secant_t(lambda t: (oz + t * dz_d) - (float(eval_z(ox + t * dx, oy + t * dy)) + fdz), 900.0, 1320.0, 200.0, 2200.0)
    x, y = ox + tm * dx, oy + tm * dy
    z = float(eval_z(x, y)) + fdz
    nx, ny = -(float(eval_z(x + 0.15, y)) - float(eval_z(x - 0.15, y))) / 0.30, -(float(eval_z(x, y + 0.15)) - float(eval_z(x, y - 0.15))) / 0.30
    nl = 1.0 / math.sqrt(nx * nx + ny * ny + 1.0)
    return (x, y, z), (nx * nl, ny * nl, nl)


def project_profile_ray_x(px, py, side, cam_spec, eval_abs_x, normal_off=0.0, cam_delta=None):
    (ox, oy, oz), (dx, dy, dz_d) = _ray_spec(px, py, cam_spec, cam_delta)
    tm = _secant_t(lambda t: (ox + t * dx) + float(eval_abs_x(oy + t * dy, oz + t * dz_d)), 940.0, 1220.0, 400.0, 1800.0)
    y, z = oy + tm * dy, oz + tm * dz_d
    ax, fs, fo = float(eval_abs_x(y, z)), float(side), float(normal_off)
    ny, nz = -(float(eval_abs_x(y + 0.15, z)) - float(eval_abs_x(y - 0.15, z))) / 0.30, -(float(eval_abs_x(y, z + 0.15)) - float(eval_abs_x(y, z - 0.15))) / 0.30
    nl = 1.0 / math.sqrt(fs * fs + ny * ny + nz * nz)
    return (fs * ax + fo * fs * nl, y + fo * ny * nl, z + fo * nz * nl), (fs * nl, ny * nl, nz * nl)


def fill_interior_holes(mask, max_area=900):
    from scipy import ndimage
    np = fh.np_mod()
    m = np.asarray(mask, dtype=bool)
    res_l: Any = ndimage.label(~m)
    lbl, num = np.asarray(res_l[0], dtype=np.int64), int(res_l[1])
    if num <= 0:
        return m.copy()
    brd = set(np.unique(np.r_[lbl[0, :], lbl[-1, :], lbl[:, 0], lbl[:, -1]]))
    cnt = np.bincount(lbl.ravel())
    return m | np.isin(lbl, [i for i in range(1, num + 1) if i not in brd and cnt[i] <= int(max_area)])


def _rdp_open(pts, eps):
    if len(pts) <= 2:
        return list(pts)
    (x1, y1), (x2, y2) = pts[0], pts[-1]
    dx, dy, max_d, split = x2 - x1, y2 - y1, 0.0, 0
    ln = math.hypot(dx, dy)
    for i in range(1, len(pts) - 1):
        px, py = pts[i]
        d = math.hypot(px - x1, py - y1) if ln < 1e-6 else abs(dy * px - dx * py + x2 * y1 - y2 * x1) / ln
        if d > max_d:
            max_d, split = d, i
    return (_rdp_open(pts[:split + 1], eps)[:-1] + _rdp_open(pts[split:], eps)) if max_d > eps else [pts[0], pts[-1]]


def simplify_closed_rdp(pts, eps):
    h = len(pts) // 2
    return list(pts) if (len(pts) < 6 or eps <= 0.0) else (_rdp_open(pts[:h + 1], eps)[:-1] + _rdp_open(list(pts[h:]) + [pts[0]], eps)[:-1])


def trace_exact_crack_polygon(mask, rdp_eps=0.52):
    np = fh.np_mod()
    m = np.asarray(mask, dtype=np.uint8)
    h, w, mp = m.shape[0], m.shape[1], np.pad(m, 1)
    ys_s, xs_s = np.nonzero(m & (~mp[:-2, 1:-1]))
    if not len(xs_s):
        return []
    sx, sy, dxy = int(xs_s[0]), int(ys_s[0]), ((1, 0), (0, 1), (-1, 0), (0, -1))
    io = ((1, 1, 1, 0), (0, 1, 1, 1), (0, 0, 0, 1), (1, 0, 0, 0))
    has_c = lambda vx, vy, d: mp[vy + io[d][1], vx + io[d][0]] == 1 and mp[vy + io[d][3], vx + io[d][2]] == 0
    raw, vx, vy, dr = [], sx, sy, 0
    for _ in range((w + h) * 24):
        raw.append((float(vx), float(vy)))
        nx, ny = vx + dxy[dr][0], vy + dxy[dr][1]
        nxt = -1 if (nx == sx and ny == sy) else next(((dr + t) % 4 for t in (3, 0, 1) if has_c(nx, ny, (dr + t) % 4)), -1)
        if nxt < 0:
            break
        vx, vy, dr = nx, ny, nxt
    nr = len(raw)
    if nr < 6:
        return raw
    mid = [(0.25 * raw[i - 1][0] + 0.5 * raw[i][0] + 0.25 * raw[(i + 1) % nr][0],
            0.25 * raw[i - 1][1] + 0.5 * raw[i][1] + 0.25 * raw[(i + 1) % nr][1])
           if (abs(raw[(i + 1) % nr][0] - raw[i - 1][0]) == 1.0 and abs(raw[(i + 1) % nr][1] - raw[i - 1][1]) == 1.0)
           else raw[i] for i in range(nr)]
    return simplify_closed_rdp(mid, rdp_eps)


def _miter_inset_same_len(poly_ccw, dist, h_loops=()):
    np = fh.np_mod()
    pts, fd = np.asarray(poly_ccw, float), float(dist)
    n, lim, h_all = len(pts), 1.9 * abs(fd), (np.vstack(h_loops) if (fd > 0.0 and h_loops) else None)
    out = np.zeros_like(pts)
    for i in range(n):
        e0, e1 = pts[i] - pts[(i - 1) % n], pts[(i + 1) % n] - pts[i]
        l0, l1 = max(float(np.linalg.norm(e0)), 1e-9), max(float(np.linalg.norm(e1)), 1e-9)
        n0, n1 = np.array([-e0[1] / l0, e0[0] / l0]), np.array([-e1[1] / l1, e1[0] / l1])
        mit = n0 + n1
        ml = float(np.linalg.norm(mit))
        uv = n0 if ml < 1e-6 else (mit / ml)
        st = fd if ml < 1e-6 else max(-lim, min(lim, fd / max(0.50, float(np.dot(uv, n0)))))
        if h_all is not None and st > 0.8:
            st = min(st, max(0.8, float(np.sqrt(((h_all - pts[i]) ** 2).sum(axis=1)).min()) - 2.4))
        out[i] = pts[i] + st * uv
    return out


def _q2d(v, a, b, c):
    ax, ay, bx, by, cx, cy = v[a][0], v[a][1], v[b][0], v[b][1], v[c][0], v[c][1]
    return ((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) / max((bx - ax) ** 2 + (by - ay) ** 2 + (cx - bx) ** 2 + (cy - by) ** 2 + (ax - cx) ** 2 + (ay - cy) ** 2, 1e-18)


def _refine_delaunay_cap(v2, tris, out_uv, b_chain, hole_ce, max_edge=13.5):
    ek = lambda u, v: (u, v) if u < v else (v, u)
    d2 = lambda a, b: (v2[a][0] - v2[b][0]) ** 2 + (v2[a][1] - v2[b][1]) ** 2
    lim2 = float(max_edge) ** 2

    def flip_all():
        lck = hole_ce | {ek(b_chain[k], b_chain[(k + 1) % len(b_chain)]) for k in range(len(b_chain))}
        for _ in range(5):
            he = {e: v for ti, (a, b, c) in enumerate(tris) for e, v in (((a, b), (ti, c)), ((b, c), (ti, a)), ((c, a), (ti, b)))}
            ue, used, fl = {ek(u, v) for u, v in he}, set(), 0
            for (a, b), (t1, c) in list(he.items()):
                if a < b and (a, b) not in lck and (b, a) in he:
                    t2, d = he[(b, a)]
                    cd = ek(c, d)
                    if t1 not in used and t2 not in used and c != d and cd not in ue and min(_q2d(v2, d, c, a), _q2d(v2, c, d, b)) > max(1e-5, min(_q2d(v2, a, b, c), _q2d(v2, b, a, d)) + 1e-5):
                        tris[t1], tris[t2] = (d, c, a), (c, d, b)
                        used.update((t1, t2))
                        ue.discard((a, b))
                        ue.add(cd)
                        fl += 1
            if not fl:
                break

    flip_all()
    for _ in range(3):
        mids, nb_c, no_v, nb = {}, [], [], len(b_chain)
        for k in range(nb):
            u, v = b_chain[k], b_chain[(k + 1) % nb]
            nb_c.append(u)
            no_v.append(out_uv[k])
            key = ek(u, v)
            if key not in hole_ce and d2(u, v) > lim2:
                if key not in mids:
                    mids[key] = len(v2)
                    v2.append((0.5 * (v2[u][0] + v2[v][0]), 0.5 * (v2[u][1] + v2[v][1])))
                nb_c.append(mids[key])
                no_v.append((0.5 * (out_uv[k][0] + out_uv[(k + 1) % nb][0]), 0.5 * (out_uv[k][1] + out_uv[(k + 1) % nb][1])))
        b_chain, out_uv = nb_c, no_v
        for a, b, c in tris:
            for u, v in ((a, b), (b, c), (c, a)):
                key = ek(u, v)
                if key not in hole_ce and key not in mids and d2(u, v) > lim2:
                    mids[key] = len(v2)
                    v2.append((0.5 * (v2[u][0] + v2[v][0]), 0.5 * (v2[u][1] + v2[v][1])))
        if not mids:
            break
        nxt = []
        for a, b, c in tris:
            m0, m1, m2 = mids.get(ek(a, b)), mids.get(ek(b, c)), mids.get(ek(c, a))
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
                nxt += [(ma, y, mb)] + ([(x, ma, mb), (x, mb, z)] if d2(x, mb) <= d2(z, ma) else [(x, ma, z), (ma, mb, z)])
        tris = nxt
        flip_all()
    return tris, b_chain, out_uv


def _hole_loops_uv(holes_px, fn=16):
    np = fh.np_mod()
    out, m2_f, ang = [], [], np.linspace(0.0, 2.0 * math.pi, fn, endpoint=False)
    ca, sa = np.cos(ang), np.sin(ang)
    for h in holes_px:
        if len(h) >= 3 and isinstance(h[0], (tuple, list)):
            out.append(csg.oriented([(float(px), -float(py)) for px, py in h], ccw=False, clean=True))
            m2_f.append(True)
        elif len(h) >= 6:
            cx, cy, rx, ry, th, is_m2 = float(h[0]), float(h[1]), float(h[2]), float(h[3]), float(h[4]), bool(h[5])
            ct, st = math.cos(th), math.sin(th)
            out.append(csg.oriented(np.c_[cx + rx * ca * ct - ry * sa * st, -(cy + rx * ca * st + ry * sa * ct)], ccw=False, clean=False))
            m2_f.append(is_m2)
        else:
            out.append(csg.oriented(csg.circle2d(float(h[2]), fn=fn, at=(float(h[0]), -float(h[1]))), ccw=False, clean=False))
            m2_f.append(True)
    return out, m2_f


def _fillet_sharp_corners(pts, rad=6.8):
    n, out = len(pts), []
    for i in range(n):
        p0, p1, p2 = pts[(i - 1) % n], pts[i], pts[(i + 1) % n]
        v0x, v0y, v1x, v1y = p0[0] - p1[0], p0[1] - p1[1], p2[0] - p1[0], p2[1] - p1[1]
        l0, l1 = math.hypot(v0x, v0y), math.hypot(v1x, v1y)
        if l0 > 3.0 and l1 > 3.0 and (v0x * v1x + v0y * v1y) / (l0 * l1) > -0.925:
            d = min(float(rad), 0.38 * min(l0, l1))
            a0, a1 = (p1[0] + v0x * d / l0, p1[1] + v0y * d / l0), (p1[0] + v1x * d / l1, p1[1] + v1y * d / l1)
            out += [a0, (0.38 * a0[0] + 0.24 * p1[0] + 0.38 * a1[0], 0.38 * a0[1] + 0.24 * p1[1] + 0.38 * a1[1]), a1]
        else:
            out.append((float(p1[0]), float(p1[1])))
    return out


def _simple2d(ps):
    n = len(ps)
    if n < 3:
        return False
    ccw = lambda a, b, c: (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    for i in range(n):
        a, b = ps[i], ps[(i + 1) % n]
        for j in range(i + 2, n - (1 if i == 0 else 0)):
            c, d = ps[j], ps[(j + 1) % n]
            if ccw(a, b, c) * ccw(a, b, d) <= 0.0 and ccw(c, d, a) * ccw(c, d, b) <= 0.0:
                return False
    return True


def _prepare_panel_poly(raw_uv, h_loops, m2_flags, corner_r_px, bevel_outset_px):
    np = fh.np_mod()
    pk = lambda pt: (round(float(pt[0]), 4), round(float(pt[1]), 4))
    uniq = lambda ps: len({pk(p) for p in ps}) == len(ps) and _simple2d(ps)
    area = lambda ps: 0.5 * sum(ps[i][0] * ps[(i + 1) % len(ps)][1] - ps[(i + 1) % len(ps)][0] * ps[i][1] for i in range(len(ps)))
    rdp_tol = 3.15 if float(corner_r_px) >= 9.0 else (1.75 if float(corner_r_px) > 1.5 else 0.0)
    ins_w = max(0.0, float(bevel_outset_px))
    for r_try in ((rdp_tol, 2.4, 3.8, 5.5) if rdp_tol > 0.0 else (0.0,)):
        uv_outer = csg.oriented(csg.clean2d(simplify_closed_rdp(raw_uv, r_try) if r_try > 0.0 else raw_uv, eps=0.35), ccw=True, clean=True)
        if not uniq(uv_outer):
            continue
        if float(corner_r_px) > 0.1 and len(uv_outer) >= 4:
            fil = csg.oriented(csg.clean2d(_fillet_sharp_corners(uv_outer, corner_r_px), eps=0.25), ccw=True, clean=True)
            if len(fil) >= 4 and uniq(fil):
                uv_outer = fil
        for ins in ((ins_w, 1.8, 0.9, 0.35, 0.0) if ins_w > 0.15 else (0.0,)):
            cand = csg.oriented(csg.clean2d(_miter_inset_same_len(uv_outer, ins, h_loops), eps=0.20), ccw=True, clean=True) if ins > 0.0 else uv_outer
            if len(cand) == len(uv_outer) and area(cand) > 12.0 and uniq(cand):
                ca_np, hl_safe = np.asarray(cand, float), []
                for hl in h_loops:
                    ha = np.asarray(hl, float)
                    hc, d_m = ha.mean(axis=0), float(np.sqrt(((ca_np[:, None, :] - ha[None, :, :]) ** 2).sum(axis=2)).min())
                    hl_safe.append((hc + 0.65 * (ha - hc)) if d_m < 1.8 else ha)
                tp, te = csg.bridge_holes(cand, holes=hl_safe, clean=False)
                if len(te) == len(tp) - 2:
                    return (uv_outer if ins > 0.0 else _miter_inset_same_len(cand, -0.30)), cand, tp, te, hl_safe, m2_flags
    uv_outer = csg.oriented(csg.clean2d(simplify_closed_rdp(raw_uv, 4.2), eps=0.45), ccw=True, clean=True)
    tp, te = csg.bridge_holes(uv_outer, holes=(), clean=False)
    return _miter_inset_same_len(uv_outer, -0.30), uv_outer, tp, te, [], []


def camera_ray_planar_panel(pixel_loop, cam_spec, eval_plane_z, crease_lines_px=(),
                            bevel_outset_px=2.8, bevel_depth_mm=0.68, thickness_mm=3.6,
                            col=fh.SHELL_CF, rim_col=fh.SEAM_SILVER, holes_px=(),
                            bolts_px=(), grid_subdiv=None, corner_r_px=6.6, inward_x=0.0):
    """Build watertight camera-ray panel with corner rounds and 45-deg bevel."""
    np, _, _ = fh.np_mod(), crease_lines_px, grid_subdiv
    raw_uv = [(float(px), -float(py)) for px, py in pixel_loop]
    h_loops, m2_flags = _hole_loops_uv(holes_px)
    uv_outer, uv_front, f_poly, f_ears, h_loops, m2_flags = _prepare_panel_poly(raw_uv, h_loops, m2_flags, corner_r_px, bevel_outset_px)
    pk_p = lambda pt: (round(float(pt[0]), 4), round(float(pt[1]), 4))
    front_v2, coord_to_idx = [], {}
    for p in f_poly:
        key = pk_p(p)
        if key not in coord_to_idx:
            coord_to_idx[key] = len(front_v2)
            front_v2.append((float(p[0]), float(p[1])))
    b_chain = [coord_to_idx[pk_p(p)] for p in uv_front]
    out_uv = [(float(p[0]), float(p[1])) for p in uv_outer]
    h_chains = [[coord_to_idx[pk_p(p)] for p in hl] for hl in h_loops]
    hole_ce = {(min(hc[i], hc[(i + 1) % len(hc)]), max(hc[i], hc[(i + 1) % len(hc)])) for hc in h_chains for i in range(len(hc))}
    f_tris = [(a, b, c) for i0, i1, i2 in f_ears
              for a, b, c in ((coord_to_idx[pk_p(f_poly[int(i0)])], coord_to_idx[pk_p(f_poly[int(i1)])], coord_to_idx[pk_p(f_poly[int(i2)])]),)
              if a != b and b != c and c != a]
    f_tris, b_chain, out_uv = _refine_delaunay_cap(front_v2, f_tris, out_uv, b_chain, hole_ce, max_edge=10.2)

    bd, th = float(bevel_depth_mm), float(thickness_mm)
    mid_d = max(bd + 0.05, th - 0.35)
    kx = lambda x, y: max(float(inward_x), min(0.68, max(0.0, (abs(x) - 12.5) / 22.0)) * (0.18 if (132.5 <= y <= 151.5 and abs(x) <= 36.5) else 1.0))
    off3 = lambda pt, n, d: canon_to_head((pt[0] - d * kx(pt[0], pt[1]) * math.copysign(abs(n[0]), pt[0]), pt[1] - 0.18 * d * n[1], pt[2] - d * max(0.66, n[2])))
    fpn = [project_ray_z(u, -v, cam_spec, eval_plane_z, 0.0) for u, v in front_v2]
    opn = [project_ray_z(u, -v, cam_spec, eval_plane_z, 0.0) for u, v in out_uv]
    f_all, b_all = np.array([canon_to_head(p) for p, _ in fpn]), np.array([off3(p, n, th) for p, n in fpn])
    o_per, m_per = np.array([off3(p, n, bd) for p, n in opn]), np.array([off3(p, n, mid_d) for p, n in opn])
    f_per, b_per = f_all[b_chain], b_all[b_chain]
    tris = ([(np.stack((f_all[a], f_all[b], f_all[c])), col) for a, b, c in f_tris]
            + [(np.stack((b_all[a], b_all[c], b_all[b])), rim_col) for a, b, c in f_tris])
    is_porc = col in (fh.SHELL_CF, fh.MOUTH_SCREEN)
    ch_col = (214, 217, 224) if is_porc else ((212, 144, 52) if col == fh.AMBER_GOLD else rim_col)
    wl_col = (204, 208, 216) if is_porc else ((198, 132, 46) if col == fh.AMBER_GOLD else rim_col)
    m_p = len(b_chain)
    for i in range(m_p):
        j = (i + 1) % m_p
        tris += [(np.stack((f_per[i], o_per[i], o_per[j])), ch_col), (np.stack((f_per[i], o_per[j], f_per[j])), ch_col),
                 (np.stack((o_per[i], m_per[i], m_per[j])), wl_col), (np.stack((o_per[i], m_per[j], o_per[j])), wl_col),
                 (np.stack((m_per[i], b_per[i], b_per[j])), rim_col), (np.stack((m_per[i], b_per[j], m_per[j])), rim_col)]
    for hl, h_idx, is_m2 in zip(h_loops, h_chains, m2_flags):
        mh = len(h_idx)
        for i in range(mh):
            hi, hj = h_idx[i], h_idx[(i + 1) % mh]
            tris += [(np.stack((f_all[hi], b_all[hi], b_all[hj])), fh.BOLT_SOCK),
                     (np.stack((f_all[hi], b_all[hj], f_all[hj])), fh.BOLT_SOCK)]
        if fh.SCREW_FREE_PANELS and is_m2 and mh >= 8:
            pt_mm, nrm = project_ray_z(float(hl[:, 0].mean()), -float(hl[:, 1].mean()), cam_spec, eval_plane_z, 0.0)
            fh.FASTENER_LOG.append((tuple(float(v) for v in canon_to_head(pt_mm)), tuple(float(v) for v in nrm), 'M2'))
    for bpx, bpy, bsc in bolts_px:
        pt_mm, nrm = project_ray_z(bpx, bpy, cam_spec, eval_plane_z, 0.0)
        tris += fh.socket_bolt(canon_to_head(tuple(pt_mm[k] + 0.05 * nrm[k] for k in range(3))), nrm, r_out=0.0024 * float(bsc), h_out=0.00055 * float(bsc))
    return tris


def profile_ray_honeycomb(prof_loop_px, side, eval_abs_x, frame_w_px=14.5, pitch_px=14.5, fill=0.70,
                          bezel_mm=2.35, chamfer_mm=0.45, frame_col=fh.AMBER_GOLD, rim_col=fh.AMBER_DARK,
                          bolts_px=()):
    """Build profile-ray honeycomb."""
    cam = CANONICAL_CAMERAS['profile90']
    uv_loop = [(float(px) if side < 0 else -float(px), -float(py)) for px, py in prof_loop_px]
    surf_m = lambda u, v, w: canon_to_head(project_profile_ray_x(float(u) if side < 0 else -float(u), -float(v), side, cam, lambda y, z: max(6.0, float(eval_abs_x(y, z)) + float(w) / SCALE_M), 0.0)[0])
    tris = fh.curved_honeycomb_bay(
        uv_loop, surf_m, rim_w=float(frame_w_px), wall=float(bezel_mm) * SCALE_M,
        chamfer=float(chamfer_mm) * SCALE_M, pitch_u=float(pitch_px), pitch_v=float(pitch_px),
        fill=float(fill), corner_r=6.4, frame_col=frame_col, rim_col=rim_col,
        hex_col=fh.HEX_DARK, hole_col=fh.BAFFLE_DARK, back_col=fh.FRAME_VOID,
        off=0.0, bolts_uv=(), step=6.5, add_back=True)
    for bpx, bpy, bsc in bolts_px:
        pt, nrm = project_profile_ray_x(bpx, bpy, side, cam, eval_abs_x, 0.12)
        s = float(bsc)
        tris += (fh.socket_bolt(canon_to_head(pt), nrm, 0.0034 * s, 0.00024 * s, frame_col, rim_col, 18)
                 + fh.socket_bolt(canon_to_head(tuple(pt[k] + 0.24 * nrm[k] for k in range(3))), nrm, 0.0023 * s, 0.00062 * s))
    return tris
