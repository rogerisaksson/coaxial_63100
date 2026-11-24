"""The robot's seat bare: sculpted honeycomb nets and pearl-white rims over the hip roll drives.

    drive = ([(name, corners, triangles)], joint)   # what a side's hip yaw turns, at rest in the pelvis's frame
    parts += [('left_seat', 'left_hip_yaw', (), -joint, 0.0, seat.plate(1.0, drive, pelvis)), ...]
    parts += [('rim_left_seat', 'left_hip_yaw', (), -joint, 0.0, seat.rim(1.0, drive, pelvis)), ...]

A sculpted anatomical gluteal cheek a side where the robot's buttock stands: an open black honeycomb
(`plate`, `livery.VOID`) clipped flush into a smooth C-infinity contoured 3D pearl-white outer bezel
(`rim`, `livery.PAD` with curved profile, gunmetal rabbet and rivets), fixed on the hip's roll drive
stack a guard's air (`AIR_M`) off the mechanism and clear of the pelvis through the hip yaw's travel.
"""
import math

from coaxial.graphics.livery import BOLT, FRAME, PAD, VOID
from machine.body import drives
from machine.body.guards import AIR_M

CELL, FILL, CLEFT = 0.0115, 0.66, 0.022
RIM_W, RIM_T = 0.0132, 0.0034
RIVET_R, RIVET_H = 0.0026, 0.0008
STACK = ('drive_', 'inv_', 'gear_', 'bolts_')
BEND: tuple = ()
AROUND, SWUNG, SPACING = 96, 21, AIR_M
_SCULPT_CACHE: dict = {}


def _dense(meshes, step=SPACING):
    """Points over the triangles of `meshes` [(corners, triangles)], none further than `step` apart."""
    from coaxial.model.blocks import numpy as np
    out = [np.zeros((0, 3))]
    for c, t in meshes:
        if not len(t):
            continue
        a, b, d = (c[t[:, k]] for k in range(3))
        longest = np.max([np.linalg.norm(b - a, axis=1), np.linalg.norm(d - b, axis=1), np.linalg.norm(a - d, axis=1)], axis=0)
        n = np.maximum(1, np.ceil(longest / step)).astype(int)
        for m in np.unique(n):
            i, j = np.meshgrid(np.arange(m + 1), np.arange(m + 1), indexing='ij')
            w = np.stack([i[i + j <= m], j[i + j <= m]], 1) / m
            k = n == m
            out.append((a[k, None] + w[None, :, :1] * (b - a)[k, None] + w[None, :, 1:] * (d - a)[k, None]).reshape(-1, 3))
    pts = np.vstack(out)
    if not len(pts):
        return pts
    _, idx = np.unique(np.rint(pts / step).astype(np.int64), axis=0, return_index=True)
    return pts[np.sort(idx)]


def _min_dist_yaw(pts, frame, joint, reach_yaw):
    """(dist_to_frame, min_x_over_yaw) for `pts` (n, 3) rotated about `joint` across `[-reach_yaw, reach_yaw]`."""
    from coaxial.model.blocks import numpy as np
    near2, x_min = np.full(len(pts), np.inf), pts[:, 0].copy()
    if not len(pts):
        return near2, x_min
    if len(frame):
        y_lo, y_hi = float(pts[:, 1].min()) - 0.025, float(pts[:, 1].max()) + 0.025
        rp = np.hypot(pts[:, 0] - joint[0], pts[:, 2] - joint[2])
        rf = np.hypot(frame[:, 0] - joint[0], frame[:, 2] - joint[2])
        frame = frame[(frame[:, 1] >= y_lo) & (frame[:, 1] <= y_hi)
                      & (rf >= float(rp.min()) - 0.025) & (rf <= float(rp.max()) + 0.025)]
    for turn in np.radians(np.linspace(-reach_yaw, reach_yaw, SWUNG)):
        c_t, s_t = math.cos(turn), math.sin(turn)
        rel = pts[:, [0, 2]] - joint[[0, 2]]
        p = np.stack([joint[0] + rel[:, 0] * c_t + rel[:, 1] * s_t, pts[:, 1],
                      joint[2] + rel[:, 1] * c_t - rel[:, 0] * s_t], 1)
        x_min = np.minimum(x_min, p[:, 0])
        if len(frame):
            box = frame[((frame > p.min(axis=0) - 0.02) & (frame < p.max(axis=0) + 0.02)).all(axis=1)]
            for chunk in (c for c in np.array_split(box, max(1, len(box) // 2000)) if len(c)):
                d = p[:, None, :] - chunk[None, :, :]
                near2 = np.minimum(near2, (d * d).sum(axis=2).min(axis=1))
    return np.sqrt(near2), x_min


def _analytic_dome(rear_core, drv_pts):
    """Build C-infinity quartic gluteal dome `(bounds, depth_z, normal_at)` clearing `drv_pts` without grid kinks."""
    from coaxial.model.blocks import numpy as np
    sx0, sx1 = float(rear_core[:, 0].min()), float(rear_core[:, 0].max())
    sy0, sy1 = float(rear_core[:, 1].min()), float(rear_core[:, 1].max())
    sz0_core = float(rear_core[:, 2].min())
    cx_s, cy_s, r_s = 0.5 * (sx0 + sx1), 0.5 * (sy0 + sy1), 0.5 * max(sx1 - sx0, sy1 - sy0)
    w_hc = min(1.0, max(0.0, (-0.122 - sz0_core) / 0.014))
    if w_hc > 0.5:
        y_bot, y_top = min(max(sy0 - 0.056, cy_s - 0.106), -0.080), min(sy1 + 0.006, 0.068)
        x_in, x_out = CLEFT + 0.002, min(max(sx1 + 0.014, cx_s + r_s + 0.010), 0.156)
        cy = y_apex = 0.46 * y_bot + 0.54 * y_top
        d2, d4, cleft_a = 0.0145, 0.0032, 0.0018
    else:
        y_bot, y_top = max(sy0 - 0.020, cy_s - 0.100), sy1 + 0.020
        x_in, x_out = CLEFT + 0.011, min(max(sx1 + 0.014, cx_s + r_s + 0.010), 0.155)
        cy, y_apex = 0.46 * y_bot + 0.54 * y_top, 0.54 * y_bot + 0.46 * y_top
        d2, d4, cleft_a = 0.0155, 0.0036, 0.0032
    cx = 0.45 * x_in + 0.55 * x_out
    x_apex = cx - 0.002
    hx_p, hx_n = max(x_out - x_apex + 0.008, 0.045), max(x_apex - x_in + 0.010, 0.045)
    hy_p, hy_n = max(y_top - y_apex + 0.010, 0.050), max(y_apex - y_bot + 0.010, 0.050)
    hx_m, hy_m = 0.5 * (hx_p + hx_n), 0.5 * (hy_p + hy_n)
    hx_d, hy_d = 0.5 * (hx_p - hx_n), 0.5 * (hy_p - hy_n)

    def _rise(x, y):
        dx, dy = np.asarray(x, float) - x_apex, np.asarray(y, float) - y_apex
        hx = hx_m + hx_d * np.tanh(dx / 0.022)
        hy = hy_m + hy_d * np.tanh(dy / 0.024)
        r2 = (dx / hx) ** 2 + (dy / hy) ** 2
        return d2 * r2 + d4 * (r2 ** 2) + cleft_a * np.exp(-((np.asarray(x, float) - x_in) / 0.026) ** 2)

    r_pad = AIR_M + RIM_T + 0.0024
    near = drv_pts[(drv_pts[:, 0] >= x_in - 0.018) & (drv_pts[:, 0] <= x_out + 0.018)
                   & (drv_pts[:, 1] >= y_bot - 0.018) & (drv_pts[:, 1] <= y_top + 0.018)
                   & (drv_pts[:, 2] <= sz0_core + 0.048)]
    if len(near):
        vx, vy = near[:, 0] - x_apex, near[:, 1] - y_apex
        vn = np.maximum(np.hypot(vx, vy), 1e-9)
        rho = AIR_M + RIM_T + 0.0014
        rise_worst = _rise(near[:, 0] + rho * vx / vn, near[:, 1] + rho * vy / vn)
        z_req_ap = float(np.min(near[:, 2] - 1.12 * r_pad - rise_worst))
    else:
        z_req_ap = sz0_core - 1.15 * r_pad - 0.016
    z_apex = (max(min(z_req_ap, sz0_core - r_pad - 0.020, -0.143), -0.1465) if w_hc <= 0.5
              else min(z_req_ap, sz0_core - r_pad - 0.014, -0.168))

    def depth_z(x, y):
        return z_apex + _rise(x, y)

    def normal_at(x, y):
        eps = 0.0016
        dz_dx = (depth_z(x + eps, y) - depth_z(x - eps, y)) / (2.0 * eps)
        dz_dy = (depth_z(x, y + eps) - depth_z(x, y - eps)) / (2.0 * eps)
        n = np.stack([dz_dx, dz_dy, -np.ones_like(dz_dx)], -1)
        return n / np.linalg.norm(n, axis=-1)[..., None]

    return (cx, cy, cx_s, cy_s, r_s, sx0, x_in, x_out, y_bot, y_top, w_hc), depth_z, normal_at


def _contour_xy(bounds):
    """Strictly smooth convex ovoid gluteal outline `(t, dx_nom, dy_nom)` free of figure-8 kinks."""
    from coaxial.model.blocks import numpy as np
    cx, cy, cx_s, _cy_s, r_s, _sx0, x_in, x_out, y_bot, y_top, w_hc = bounds
    t = np.arange(AROUND) * (2.0 * math.pi / AROUND)
    ct, st = np.cos(t), np.sin(t)
    r_drv = min(r_s + 0.004, 0.050) if r_s > 0.048 else r_s + 0.004
    if w_hc > 0.5:
        x_out = max(x_out, min(cx_s + r_drv + 0.010, 0.158))
        x_in = min(x_in, max(cx_s - r_drv - 0.006, CLEFT))
    rx_m, rx_d = 0.5 * ((x_out - cx) + (cx - x_in)), 0.5 * ((x_out - cx) - (cx - x_in))
    ry_m, ry_d = 0.5 * ((y_top - cy) + (cy - y_bot)), 0.5 * ((y_top - cy) - (cy - y_bot))
    rx = rx_m + rx_d * ct
    ry = ry_m + ry_d * st
    s_top, s_bot = 0.5 * (1.0 + st), 0.5 * (1.0 - st)
    taper = (1.0 - 0.07 * (s_top ** 2.0) + 0.03 * (s_bot ** 1.6)) if w_hc > 0.5 else (
        1.0 - 0.14 * (s_top ** 2.0) + 0.05 * (s_bot ** 1.6))
    r_sup = (np.abs(ct) ** 2.06 + np.abs(st) ** 2.06) ** (-1.0 / 2.06)
    return t, rx * taper * r_sup * ct, ry * r_sup * st


def _smooth_periodic(arr, passes=64):
    """Apply periodic C-infinity Gaussian fairing to 1D ring `arr` without concave `np.minimum` kinks."""
    from coaxial.model.blocks import numpy as np
    out = arr.copy()
    for _ in range(passes):
        out = 0.25 * np.roll(out, -1) + 0.50 * out + 0.25 * np.roll(out, 1)
    return out


def _sculpt(side, drive, pelvis):
    """Shared C-infinity anatomical gluteal surface and smooth convex perimeter in left (+x) pelvis rest frame."""
    from coaxial.model.blocks import numpy as np
    key = (float(side), tuple(round(float(v), 6) for v in drive[1]),
           tuple((n, len(c), len(t_), round(float(c[0, 2]), 6) if len(c) else 0.0) for n, c, t_ in drive[0]),
           tuple((len(c), len(t_), round(float(c[0, 2]), 6) if len(c) else 0.0) for c, t_ in pelvis))
    if key in _SCULPT_CACHE:
        return _SCULPT_CACHE[key]
    flip = np.array([side, 1.0, 1.0])
    named_drv = [(n, np.asarray(c, float) * flip, np.asarray(t_)) for n, c, t_ in drive[0] if n.startswith(STACK)]
    all_drv = [(c, t_) for _n, c, t_ in named_drv]
    roll_pts = [c for n, c, _t in named_drv if 'roll' in n]
    core_pts = np.vstack(roll_pts if roll_pts else [c for c, _t in all_drv])
    rear_core = core_pts[core_pts[:, 2] <= float(core_pts[:, 2].min()) + 0.032]
    drv_pts = _dense(all_drv, step=0.003)
    frame = _dense([(np.asarray(c, float) * flip, np.asarray(t_)) for c, t_ in pelvis], step=0.003)
    joint, reach_yaw = np.asarray(drive[1], float) * flip, max(abs(x) for x in drives.TRAVEL['hip_yaw'])
    bounds, depth_z, normal_at = _analytic_dome(rear_core, drv_pts)
    cx, cy, sx0 = bounds[0], bounds[1], bounds[5]
    t, dx_nom, dy_nom = _contour_xy(bounds)
    x_contra_lim = -(sx0 + 0.0048) + AIR_M + 0.0006
    scales = np.ones(AROUND)
    for it in range(18):
        ox, oy = cx + scales * dx_nom, cy + scales * dy_nom
        oz, on = depth_z(ox, oy), normal_at(ox, oy)
        p_f, p_b = np.stack([ox, oy, oz], 1), np.stack([ox, oy, oz], 1) - RIM_T * on
        d_f, x_f = _min_dist_yaw(p_f, frame, joint, reach_yaw)
        d_b, x_b = _min_dist_yaw(p_b, frame, joint, reach_yaw)
        bad = (p_f[:, 0] < CLEFT - 0.002) | (p_b[:, 0] < CLEFT - 0.002) | (x_f < x_contra_lim) | (x_b < x_contra_lim)
        if len(frame):
            bad |= (d_f < AIR_M + 0.0032) | (d_b < AIR_M + 0.0032)
        if not bad.any():
            break
        scales = _smooth_periodic(np.where(bad, scales * 0.94, scales), passes=40 if it < 14 else 16)
    scales = _smooth_periodic(scales, passes=32)
    ox, oy = cx + scales * dx_nom, cy + scales * dy_nom
    oz, on = depth_z(ox, oy), normal_at(ox, oy)
    p_out = np.stack([ox, oy, oz], 1)
    tang = np.roll(np.stack([ox, oy], 1), -1, axis=0) - np.roll(np.stack([ox, oy], 1), 1, axis=0)
    in_xy = np.stack([-tang[:, 1], tang[:, 0]], 1)
    in_xy /= np.maximum(np.linalg.norm(in_xy, axis=1, keepdims=True), 1e-9)
    in_sign = np.sign((in_xy * np.stack([cx - ox, cy - oy], 1)).sum(1, keepdims=True))
    in_xy *= np.where(in_sign == 0.0, 1.0, in_sign)
    eps = 0.0016
    dz_din = (depth_z(ox + eps * in_xy[:, 0], oy + eps * in_xy[:, 1]) - oz) / eps
    step_2d = RIM_W / np.sqrt(1.0 + dz_din * dz_din)
    ix = _smooth_periodic(ox + step_2d * in_xy[:, 0], passes=10)
    iy = _smooth_periodic(oy + step_2d * in_xy[:, 1], passes=10)
    p_in, i_n = np.stack([ix, iy, depth_z(ix, iy)], 1), normal_at(ix, iy)
    res = {'flip': flip, 'joint': joint, 'reach_yaw': reach_yaw, 'frame': frame, 'sx0': sx0,
           'cx': cx, 'cy': cy, 't': t, 'ox': ox, 'oy': oy, 'ix': ix, 'iy': iy,
           'p_out': p_out, 'n_out': on, 'p_in': p_in, 'n_in': i_n, 'depth_z': depth_z, 'normal_at': normal_at}
    if len(_SCULPT_CACHE) > 8:
        _SCULPT_CACHE.clear()
    _SCULPT_CACHE[key] = res
    return res


def _clip_tri_2d(tri_xy, eval_f):
    """Clip 2D triangle `(3, 2)` to `eval_f(xy) >= 0`, returning a list of `(3, 2)` triangles."""
    from coaxial.model.blocks import numpy as np
    f = eval_f(tri_xy)
    if (f >= 0.0).all():
        return [tri_xy]
    if (f <= 0.0).all():
        return []
    poly = []
    for i in range(3):
        a, b, fa, fb = tri_xy[i], tri_xy[(i + 1) % 3], float(f[i]), float(f[(i + 1) % 3])
        if fa >= 0.0:
            poly.append(a)
        if (fa >= 0.0) != (fb >= 0.0) and abs(fa - fb) > 1e-12:
            poly.append(a + (fa / (fa - fb)) * (b - a))
    if len(poly) < 3:
        return []
    if len(poly) == 3:
        return [np.array(poly)]
    return [np.array([poly[0], poly[1], poly[2]]), np.array([poly[0], poly[2], poly[3]])]


def plate(side, drive, pelvis=()):
    """A side's sculpted black honeycomb net clipped flush into `rim`, `side` 1 for left, clear of `drive` and `pelvis`."""
    from coaxial.model.blocks import numpy as np
    g = _sculpt(side, drive, pelvis)
    cx, cy, ix, iy, ox, oy = g['cx'], g['cy'], g['ix'], g['iy'], g['ox'], g['oy']
    r_in = np.hypot(ix - cx, iy - cy)
    r_out = np.hypot(ox - cx, oy - cy)
    r_seam = np.minimum(r_in + 0.0026, r_out - 0.0036)
    ang_in = np.arctan2(iy - cy, ix - cx) % (2.0 * math.pi)
    order = np.argsort(ang_in)
    ang_pad = np.r_[ang_in[order][-1] - 2.0 * math.pi, ang_in[order], ang_in[order][0] + 2.0 * math.pi]
    seam_pad = np.r_[r_seam[order][-1], r_seam[order], r_seam[order][0]]

    def eval_f(xy):
        dx, dy = xy[:, 0] - cx, xy[:, 1] - cy
        return np.interp(np.arctan2(dy, dx) % (2.0 * math.pi), ang_pad, seam_pad) - np.hypot(dx, dy)

    max_r, rise = float(r_seam.max()), CELL * math.sqrt(3.0) / 2.0
    span_x, span_y = int(max_r / CELL) + 2, int(max_r / rise) + 2
    mids = np.array([(i * CELL + (j % 2) * CELL / 2.0, j * rise)
                     for j in range(-span_y, span_y + 1) for i in range(-span_x, span_x + 1)])
    hexa = np.arange(6) * (math.pi / 3.0) + math.pi / 6.0
    unit = np.stack([np.cos(hexa), np.sin(hexa)], 1) * CELL / math.sqrt(3.0)
    ring_xy = np.concatenate([unit, FILL * unit])
    k = np.arange(6)
    quads = np.concatenate([np.stack([k, (k + 1) % 6, 6 + (k + 1) % 6], 1), np.stack([k, 6 + (k + 1) % 6, 6 + k], 1)])
    clipped = []
    for mx, my in mids:
        cell = np.array([cx + mx, cy + my]) + ring_xy
        fc = eval_f(cell)
        if (fc <= -0.002).all():
            continue
        if (fc >= 0.0).all():
            for q in quads:
                clipped.append(cell[q])
        else:
            for q in quads:
                clipped.extend(_clip_tri_2d(cell[q], eval_f))
    if not clipped:
        return np.zeros((0, 3)), np.zeros((0, 3), int), np.zeros((0, 2)), np.zeros(0, int)
    tri_2d = np.array(clipped)
    area2 = np.abs((tri_2d[:, 1, 0] - tri_2d[:, 0, 0]) * (tri_2d[:, 2, 1] - tri_2d[:, 0, 1])
                   - (tri_2d[:, 2, 0] - tri_2d[:, 0, 0]) * (tri_2d[:, 1, 1] - tri_2d[:, 0, 1]))
    tri_2d = tri_2d[area2 > 1e-10]
    x_contra_lim = -(g['sx0'] + 0.0055) + AIR_M + 0.0008
    flat_xy = tri_2d.reshape(-1, 2)
    z_v, n_v = g['depth_z'](flat_xy[:, 0], flat_xy[:, 1]), g['normal_at'](flat_xy[:, 0], flat_xy[:, 1])
    pts_3d = np.stack([flat_xy[:, 0], flat_xy[:, 1], z_v], 1) - 0.0006 * n_v
    if len(pts_3d):
        d_pel, x_pel = _min_dist_yaw(pts_3d, g['frame'], g['joint'], g['reach_yaw'])
        keep = ((d_pel >= AIR_M + 0.0006) & (x_pel >= x_contra_lim)).reshape(-1, 3).all(axis=1)
        pts_3d = pts_3d.reshape(-1, 3, 3)[keep].reshape(-1, 3)
        n_v = n_v.reshape(-1, 3, 3)[keep].reshape(-1, 3)
    pts_u, inv = np.unique(np.round(pts_3d, 7), axis=0, return_inverse=True)
    # Keep exact unrounded coordinates at first occurrence for smooth fidelity
    _, first = np.unique(inv, return_index=True)
    corners = pts_3d[first]
    n_c = n_v[first]
    tri = inv.reshape(-1, 3)
    non_deg = (tri[:, 0] != tri[:, 1]) & (tri[:, 1] != tri[:, 2]) & (tri[:, 2] != tri[:, 0])
    tri = tri[non_deg]
    facing = (np.cross(corners[tri[:, 1]] - corners[tri[:, 0]], corners[tri[:, 2]] - corners[tri[:, 0]]) * n_c[tri[:, 0]]).sum(axis=1)
    tri[facing < 0.0] = tri[facing < 0.0][:, ::-1]
    laid = corners * g['flip']
    return laid, tri[:, ::int(math.copysign(1.0, side))], np.zeros((len(laid), 2)), np.full(len(laid), VOID)


def _strip(ring_a, ring_b, mat):
    from coaxial.model.blocks import numpy as np
    m, corners = len(ring_a), np.vstack([ring_a, ring_b])
    k, nxt = np.arange(m), (np.arange(m) + 1) % m
    tri = np.vstack([np.stack([k, nxt, m + nxt], 1), np.stack([k, m + nxt, m + k], 1)])
    return corners, tri, np.zeros((len(corners), 2)), np.full(len(corners), mat)


def _prism_hex(centre, normal, radius, z0, z1, mat):
    from coaxial.model.blocks import numpy as np
    ref = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.85 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(normal, ref)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(normal, e1)
    ang = np.arange(6) * (math.pi / 3.0)
    ring = radius * (np.outer(np.cos(ang), e1) + np.outer(np.sin(ang), e2))
    corners = np.vstack([centre + z0 * normal + ring, centre + z1 * normal + 0.88 * ring,
                         (centre + z0 * normal)[None], (centre + z1 * normal)[None]])
    tris = [row for i in range(6) for j in ((i + 1) % 6,)
            for row in ((i, j, 6 + j), (i, 6 + j, 6 + i), (12, j, i), (13, 6 + i, 6 + j))]
    return corners, np.array(tris), np.zeros((len(corners), 2)), np.full(len(corners), mat)


def rim(side, drive, pelvis=()):
    """A side's smooth curved-section 3D pearl-white bezel (`livery.PAD` + gunmetal rabbet & rivets) enclosing `plate`."""
    from coaxial.model.blocks import numpy as np
    g = _sculpt(side, drive, pelvis)
    ox, oy, ix, iy = g['ox'], g['oy'], g['ix'], g['iy']
    depth_z, normal_at = g['depth_z'], g['normal_at']

    def _ring(s_in, h_n):
        rx, ry = (1.0 - s_in) * ox + s_in * ix, (1.0 - s_in) * oy + s_in * iy
        rz, rn = depth_z(rx, ry), normal_at(rx, ry)
        return np.stack([rx, ry, rz], 1) + h_n * rn, rn

    t_in = g['p_in'] - g['p_out']
    t_in /= np.maximum(np.linalg.norm(t_in, axis=1, keepdims=True), 1e-9)
    # Smooth convex cross-section across the pearl-white bezel + deep inner rabbet enclosing the clipped honeycomb seam!
    r0, n0 = _ring(1.00, 0.0009)     # inner front lip overlapping clipped honeycomb
    r1, n1 = _ring(0.76, 0.0018)     # inner convex crown
    r2, n2 = _ring(0.38, 0.0020)     # outer convex crown
    r3, n3 = _ring(0.10, 0.0011)     # outer soft shoulder
    r4, n4 = _ring(0.00, 0.0000)     # outer edge
    r5, n5 = _ring(0.00, -RIM_T)     # outer back
    r6, _n6 = _ring(1.24, -RIM_T)    # rabbet back behind clipped honeycomb seam
    r7, n7 = _ring(1.24, -0.0005)    # rabbet shelf under clipped honeycomb seam
    r8, _n8 = _ring(1.00, -0.0005)   # inner pocket wall
    bands = [(_strip(r0, r1, PAD), n0 + 0.25 * t_in), (_strip(r1, r2, PAD), n1),
             (_strip(r2, r3, PAD), n2 - 0.15 * t_in), (_strip(r3, r4, PAD), n3 - 0.45 * t_in),
             (_strip(r4, r5, FRAME), -t_in), (_strip(r5, r6, FRAME), -n5),
             (_strip(r6, r7, FRAME), t_in), (_strip(r7, r8, FRAME), n7),
             (_strip(r8, r0, FRAME), t_in)]
    fixed = []
    for (c, tri, uv, mat), ref_n in bands:
        n_avg = np.vstack([ref_n, ref_n])
        fn = np.cross(c[tri[:, 1]] - c[tri[:, 0]], c[tri[:, 2]] - c[tri[:, 0]])
        flp = (fn * n_avg[tri].mean(1)).sum(1) < 0.0
        tri = tri.copy()
        tri[flp] = tri[flp][:, ::-1]
        fixed.append((c, tri, uv, mat))
    for idx in (12, 36, 60, 84):
        mid, nrm = _ring(0.54, 0.00195)
        fixed.append(_prism_hex(mid[idx], nrm[idx], RIVET_R * 1.38, 0.00004, 0.00038, FRAME))
        fixed.append(_prism_hex(mid[idx], nrm[idx], RIVET_R, 0.00039, 0.00039 + RIVET_H, BOLT))
    base, cs, ts, ms = 0, [], [], []
    for c, t, _uv, m in fixed:
        cs.append(c), ts.append(t + base), ms.append(m)
        base += len(c)
    corners = np.vstack(cs) * g['flip']
    tri = np.vstack(ts)[:, ::int(math.copysign(1.0, side))]
    return corners, tri, np.zeros((len(corners), 2)), np.concatenate(ms)
