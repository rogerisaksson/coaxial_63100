"""The robot's seat bare: sculpted honeycomb nets and pearl-white rims over the hip roll drives.

    drive = ([(name, corners, triangles)], joint)   # what a side's hip yaw turns, at rest in the pelvis's frame
    parts += [('left_seat', 'left_hip_yaw', (), -joint, 0.0, seat.plate(1.0, drive, pelvis)), ...]
    parts += [('rim_left_seat', 'left_hip_yaw', (), -joint, 0.0, seat.rim(1.0, drive, pelvis)), ...]

A sculpted anatomical gluteal cheek a side where the robot's buttock stands: an open black honeycomb
(`plate`, `livery.VOID`) framed by a contoured 3D pearl-white outer bezel (`rim`, `livery.PAD` with
gunmetal bevel walls and rivets), fixed on the hip's roll drive stack a guard's air (`AIR_M`) off
the mechanism and clear of the pelvis through the hip yaw's travel.
"""
import math

from coaxial.graphics.livery import BOLT, FRAME, PAD, VOID
from machine.body import drives
from machine.body.guards import AIR_M

CELL, FILL, CLEFT = 0.0125, 0.64, 0.022
RIM_W, RIM_T, BEVEL_M, PROUD_M = 0.013, 0.0036, 0.0022, 0.0004
RIVET_R, RIVET_H = 0.0028, 0.0009
STACK = ('drive_', 'inv_', 'gear_', 'bolts_')
BEND: tuple = ()
AROUND, SWUNG, SPACING = 72, 21, AIR_M
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


def _drive_req(meshes, r_pad, x0=-0.01, y0=-0.16, step=0.003, nx=72, ny=96):
    """Smooth 2D clearance ceiling `eval_req(x, y)` spherically `r_pad` clear of `meshes`."""
    from coaxial.model.blocks import numpy as np
    gx, gy = x0 + np.arange(nx) * step, y0 + np.arange(ny) * step
    z_raw = np.full((nx, ny), np.inf)
    for c, t in meshes:
        if not len(t):
            continue
        vx = np.clip(np.rint((c[:, 0] - x0) / step).astype(int), 0, nx - 1)
        vy = np.clip(np.rint((c[:, 1] - y0) / step).astype(int), 0, ny - 1)
        np.minimum.at(z_raw, (vx, vy), c[:, 2])
        a, b, d = c[t[:, 0]], c[t[:, 1]], c[t[:, 2]]
        denom = (b[:, 0] - a[:, 0]) * (d[:, 1] - a[:, 1]) - (d[:, 0] - a[:, 0]) * (b[:, 1] - a[:, 1])
        keep = np.abs(denom) > 1e-11
        a, b, d, denom = a[keep], b[keep], d[keep], denom[keep]
        ix0 = np.maximum(0, np.floor((np.minimum(a[:, 0], np.minimum(b[:, 0], d[:, 0])) - x0) / step).astype(int))
        ix1 = np.minimum(nx - 1, np.ceil((np.maximum(a[:, 0], np.maximum(b[:, 0], d[:, 0])) - x0) / step).astype(int))
        iy0 = np.maximum(0, np.floor((np.minimum(a[:, 1], np.minimum(b[:, 1], d[:, 1])) - y0) / step).astype(int))
        iy1 = np.minimum(ny - 1, np.ceil((np.maximum(a[:, 1], np.maximum(b[:, 1], d[:, 1])) - y0) / step).astype(int))
        for k in np.flatnonzero((ix1 >= ix0) & (iy1 >= iy0)):
            px, py = gx[ix0[k]:ix1[k] + 1, None], gy[None, iy0[k]:iy1[k] + 1]
            v0, v1, v2, dn = a[k], b[k], d[k], denom[k]
            w1 = ((px - v0[0]) * (v2[1] - v0[1]) - (v2[0] - v0[0]) * (py - v0[1])) / dn
            w2 = ((v1[0] - v0[0]) * (py - v0[1]) - (px - v0[0]) * (v1[1] - v0[1])) / dn
            w0 = 1.0 - w1 - w2
            ins = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
            if ins.any():
                sub = z_raw[ix0[k]:ix1[k] + 1, iy0[k]:iy1[k] + 1]
                sub[ins] = np.minimum(sub[ins], (w0 * v0[2] + w1 * v1[2] + w2 * v2[2])[ins])
    z_req = np.full((nx, ny), 0.1)
    m_rad = int(math.ceil(r_pad / step))
    for dx_i in range(-m_rad, m_rad + 1):
        for dy_i in range(-m_rad, m_rad + 1):
            d2 = (dx_i * step) ** 2 + (dy_i * step) ** 2
            if d2 <= r_pad * r_pad:
                dz = math.sqrt(r_pad * r_pad - d2)
                sx0, sx1 = max(0, dx_i), min(nx, nx + dx_i)
                sy0, sy1 = max(0, dy_i), min(ny, ny + dy_i)
                z_req[sx0 - dx_i:sx1 - dx_i, sy0 - dy_i:sy1 - dy_i] = np.minimum(
                    z_req[sx0 - dx_i:sx1 - dx_i, sy0 - dy_i:sy1 - dy_i], z_raw[sx0:sx1, sy0:sy1] - dz)
    z_sm = np.round(z_req, 8)
    for _ in range(6):
        p = np.pad(z_sm, 1, mode='edge')
        z_sm = np.minimum(z_sm, 0.25 * p[1:-1, 1:-1] + 0.125 * (p[:-2, 1:-1] + p[2:, 1:-1] + p[1:-1, :-2] + p[1:-1, 2:])
                          + 0.0625 * (p[:-2, :-2] + p[:-2, 2:] + p[2:, :-2] + p[2:, 2:]))
    z_sm = np.round(z_sm, 8)

    def eval_req(x, y):
        fx, fy = np.clip((x - x0) / step, 0.0, nx - 1.0001), np.clip((y - y0) / step, 0.0, ny - 1.0001)
        ix, iy = fx.astype(int), fy.astype(int)
        tx, ty = fx - ix, fy - iy
        return ((1.0 - tx) * (1.0 - ty) * z_sm[ix, iy] + tx * (1.0 - ty) * z_sm[ix + 1, iy]
                + (1.0 - tx) * ty * z_sm[ix, iy + 1] + tx * ty * z_sm[ix + 1, iy + 1])

    return eval_req


def _depth_model(rear_core, eval_req):
    """Build anatomical bounds and smooth 3D dome evaluators `(bounds, depth_z, normal_at)`."""
    from coaxial.model.blocks import numpy as np
    sx0, sx1 = float(rear_core[:, 0].min()), float(rear_core[:, 0].max())
    sy0, sy1 = float(rear_core[:, 1].min()), float(rear_core[:, 1].max())
    sz0_core = float(rear_core[:, 2].min())
    cx_s, cy_s, r_s = 0.5 * (sx0 + sx1), 0.5 * (sy0 + sy1), 0.5 * max(sx1 - sx0, sy1 - sy0)
    w_hc = min(1.0, max(0.0, (-0.122 - sz0_core) / 0.014))
    if w_hc > 0.5:
        y_bot, y_top = min(max(sy0 - 0.058, cy_s - 0.110), -0.082), min(sy1 + 0.004, 0.068)
        cy = y_apex = 0.46 * y_bot + 0.54 * y_top
        x_in, x_out = CLEFT + 0.002, min(max(sx1 + 0.014, cx_s + r_s + 0.010), 0.156)
    else:
        y_bot, y_top = max(sy0 - 0.022, cy_s - 0.105), sy1 + 0.022
        cy, y_apex = 0.45 * y_bot + 0.55 * y_top, 0.58 * y_bot + 0.42 * y_top
        x_in, x_out = CLEFT + 0.005, min(max(sx1 + 0.016, cx_s + r_s + 0.012), 0.160)
    cx = 0.44 * x_in + 0.56 * x_out
    x_apex = cx - 0.003
    z_req_c = float(eval_req(np.array([x_apex]), np.array([y_apex]))[0])
    z_apex = (min(sz0_core - (AIR_M + RIM_T) - 0.024, -0.143) if w_hc <= 0.5
              else min(z_req_c - 0.015, sz0_core - (AIR_M + RIM_T) - 0.018, -0.168))

    def depth_z(x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        ux = (x - x_apex) / np.where(x >= x_apex, max(x_out - x_apex, 0.02), max(x_apex - x_in + 0.006, 0.02))
        vy = (y - y_apex) / np.where(y >= y_apex, max(y_top - y_apex + 0.006, 0.02), max(y_apex - y_bot + 0.006, 0.02))
        r2 = np.clip(ux * ux + vy * vy, 0.0, 1.8)
        if w_hc > 0.5:
            z_anat = z_apex + 0.0152 * (0.72 * r2 + 0.22 * (r2 ** 1.45)) + 0.0024 * np.exp(-((x - x_in) / 0.022) ** 2)
            k = 0.009
        else:
            dome = 0.020 * (0.48 * r2 + 0.24 * (r2 ** 1.7))
            z_anat = (z_apex + dome + 0.012 * np.exp(-((x - x_in) / 0.022) ** 2)
                      + 0.014 * np.maximum(0.0, vy) ** 1.85 + 0.011 * np.maximum(0.0, -vy) ** 2.0)
            k = 0.005
        return -k * np.log(np.exp(-z_anat / k) + np.exp(-eval_req(x, y) / k))

    def normal_at(x, y):
        eps = 0.0018
        dz_dx = (depth_z(x + eps, y) - depth_z(x - eps, y)) / (2.0 * eps)
        dz_dy = (depth_z(x, y + eps) - depth_z(x, y - eps)) / (2.0 * eps)
        n = np.stack([dz_dx, dz_dy, -np.ones_like(dz_dx)], -1)
        return n / np.linalg.norm(n, axis=-1)[..., None]

    return (cx, cy, cx_s, cy_s, r_s, sx0, x_in, x_out, y_bot, y_top, w_hc), depth_z, normal_at


def _contour_xy(bounds):
    """Smooth gluteal outline `(t, dx_nom, dy_nom)` with waist/sacral taper and fuller cheek."""
    from coaxial.model.blocks import numpy as np
    cx, cy, cx_s, cy_s, r_s, _sx0, x_in, x_out, y_bot, y_top, w_hc = bounds
    t = np.arange(AROUND) * (2.0 * math.pi / AROUND)
    ct, st = np.cos(t), np.sin(t)
    r_drv = min(r_s + 0.002, 0.048) if r_s > 0.048 else r_s + 0.004
    if w_hc > 0.5:
        x_out = max(x_out, min(cx_s + r_drv + 0.008, 0.158))
        x_in = min(x_in, max(cx_s - r_drv - 0.004, CLEFT))
    rx_g, ry_g = np.where(ct >= 0.0, x_out - cx, cx - x_in), np.where(st >= 0.0, y_top - cy, cy - y_bot)
    s_top, s_bot, s_out, s_in = 0.5 * (1.0 + st), 0.5 * (1.0 - st), 0.5 * (1.0 + ct), 0.5 * (1.0 - ct)
    if w_hc > 0.5:
        x_sc = 1.0 - 0.08 * (s_top ** 2.0) * (s_out ** 1.3) - 0.10 * (s_top ** 2.2) * (s_in ** 1.3) + 0.05 * (s_bot ** 1.5) * (s_out ** 1.4)
        ks = 0.012
    else:
        x_sc = 1.0 - 0.28 * (s_top ** 2.0) * (s_out ** 1.3) - 0.30 * (s_top ** 2.2) * (s_in ** 1.3) + 0.08 * (s_bot ** 1.5) * (s_out ** 1.4)
        ks = 0.004
    y_sc = 1.0 - 0.04 * (s_top ** 2.0) * (s_out ** 1.5)
    r_sup = (np.abs(ct) ** 2.08 + np.abs(st) ** 2.08) ** (-1.0 / 2.08)
    gx, gy = cx + rx_g * x_sc * r_sup * ct, cy + ry_g * y_sc * r_sup * st
    dx_d, dy_d = (cx_s + r_drv * ct) - cx, (cy_s + r_drv * st) - cy
    ang_d, rad_d = np.arctan2(dy_d, dx_d) % (2.0 * math.pi), np.hypot(dx_d, dy_d)
    ord_d = np.argsort(ang_d)
    ang_dp = np.r_[ang_d[ord_d][-1] - 2.0 * math.pi, ang_d[ord_d], ang_d[ord_d][0] + 2.0 * math.pi]
    rad_dp = np.r_[rad_d[ord_d][-1], rad_d[ord_d], rad_d[ord_d][0]]
    rad_nom = ks * np.log(np.exp(np.hypot(gx - cx, gy - cy) / ks) + np.exp(np.interp(t, ang_dp, rad_dp) / ks))
    return t, rad_nom * ct, rad_nom * st


def _sculpt(side, drive, pelvis):
    """Shared anatomical gluteal surface and C2-smoothed perimeter in canonical left (+x) pelvis rest frame."""
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
    eval_req = _drive_req(all_drv, AIR_M + RIM_T + 0.0020)
    frame = _dense([(np.asarray(c, float) * flip, np.asarray(t_)) for c, t_ in pelvis], step=0.003)
    joint, reach_yaw = np.asarray(drive[1], float) * flip, max(abs(x) for x in drives.TRAVEL['hip_yaw'])
    bounds, depth_z, normal_at = _depth_model(rear_core, eval_req)
    cx, cy, sx0 = bounds[0], bounds[1], bounds[5]
    t, dx_nom, dy_nom = _contour_xy(bounds)
    x_contra_lim = -(sx0 + 0.0045) + AIR_M + 0.0005
    scales = np.ones(AROUND)
    for _ in range(14):
        ox, oy = cx + scales * dx_nom, cy + scales * dy_nom
        oz, on = depth_z(ox, oy), normal_at(ox, oy)
        p_f, p_b = np.stack([ox, oy, oz], 1), np.stack([ox, oy, oz], 1) - RIM_T * on
        d_f, x_f = _min_dist_yaw(p_f, frame, joint, reach_yaw)
        d_b, x_b = _min_dist_yaw(p_b, frame, joint, reach_yaw)
        bad = (p_f[:, 0] < CLEFT - 0.002) | (p_b[:, 0] < CLEFT - 0.002) | (x_f < x_contra_lim) | (x_b < x_contra_lim)
        if len(frame):
            bad |= (d_f < AIR_M + 0.002) | (d_b < AIR_M + 0.002)
        if not bad.any():
            break
        scales = np.where(bad, scales * 0.96, scales)
    for _ in range(28):
        scales = np.minimum(scales, 0.25 * np.roll(scales, -1) + 0.5 * scales + 0.25 * np.roll(scales, 1))
    ox, oy = cx + scales * dx_nom, cy + scales * dy_nom
    oz, on = depth_z(ox, oy), normal_at(ox, oy)
    p_out = np.stack([ox, oy, oz], 1)
    in_xy = np.stack([-dx_nom, -dy_nom], 1)
    in_xy /= np.maximum(np.linalg.norm(in_xy, axis=1, keepdims=True), 1e-9)
    eps = 0.0018
    dz_din = (depth_z(ox + eps * in_xy[:, 0], oy + eps * in_xy[:, 1]) - oz) / eps
    step_2d = RIM_W / np.sqrt(1.0 + dz_din * dz_din)
    ix, iy = ox + step_2d * in_xy[:, 0], oy + step_2d * in_xy[:, 1]
    p_in, i_n = np.stack([ix, iy, depth_z(ix, iy)], 1), normal_at(ix, iy)
    res = {'flip': flip, 'joint': joint, 'reach_yaw': reach_yaw, 'frame': frame,
           'cx': cx, 'cy': cy, 't': t, 'ox': ox, 'oy': oy, 'ix': ix, 'iy': iy,
           'p_out': p_out, 'n_out': on, 'p_in': p_in, 'n_in': i_n, 'depth_z': depth_z, 'normal_at': normal_at}
    if len(_SCULPT_CACHE) > 8:
        _SCULPT_CACHE.clear()
    _SCULPT_CACHE[key] = res
    return res


def plate(side, drive, pelvis=()):
    """A side's sculpted black honeycomb net in pelvis rest frame, `side` 1 for left, clear of `drive` and `pelvis`."""
    from coaxial.model.blocks import numpy as np
    g = _sculpt(side, drive, pelvis)
    cx, cy, ix, iy = g['cx'], g['cy'], g['ix'], g['iy']
    r_in, ang_in = np.hypot(ix - cx, iy - cy), np.arctan2(iy - cy, ix - cx) % (2.0 * math.pi)
    order = np.argsort(ang_in)
    ang_s, r_s = ang_in[order], r_in[order]
    ang_pad = np.r_[ang_s[-1] - 2.0 * math.pi, ang_s, ang_s[0] + 2.0 * math.pi]
    r_pad = np.r_[r_s[-1], r_s, r_s[0]]
    max_r, rise = float(r_in.max()), CELL * math.sqrt(3.0) / 2.0
    span_x, span_y = int(max_r / CELL) + 2, int(max_r / rise) + 2
    mids = np.array([(i * CELL + (j % 2) * CELL / 2.0, j * rise)
                     for j in range(-span_y, span_y + 1) for i in range(-span_x, span_x + 1)])
    hexa = np.arange(6) * (math.pi / 3.0) + math.pi / 6.0
    unit = np.stack([np.cos(hexa), np.sin(hexa)], 1) * CELL / math.sqrt(3.0)
    flat = mids[:, None, :] + np.concatenate([unit, FILL * unit])[None]
    px, py = cx + flat[..., 0], cy + flat[..., 1]
    reach = np.interp(np.arctan2(flat[..., 1], flat[..., 0]) % (2.0 * math.pi), ang_pad, r_pad)
    inside = (np.hypot(flat[..., 0], flat[..., 1]) < reach - 0.0005)[:, :6].all(axis=1)
    pts_3d = np.stack([px[inside].ravel(), py[inside].ravel(), g['depth_z'](px[inside], py[inside]).ravel()], 1)
    if len(g['frame']) and len(pts_3d):
        d_pel, _ = _min_dist_yaw(pts_3d, g['frame'], g['joint'], g['reach_yaw'])
        inside[np.flatnonzero(inside)[~(d_pel.reshape(-1, 12) >= AIR_M).all(axis=1)]] = False
    fx, fy = px[inside].ravel(), py[inside].ravel()
    corners, n = np.stack([fx, fy, g['depth_z'](fx, fy)], 1), g['normal_at'](fx, fy)
    k = np.arange(6)
    quads = np.concatenate([np.stack([k, (k + 1) % 6, 6 + (k + 1) % 6], 1), np.stack([k, 6 + (k + 1) % 6, 6 + k], 1)])
    tri = (12 * np.arange(int(inside.sum()))[:, None, None] + quads[None]).reshape(-1, 3)
    facing = (np.cross(corners[tri[:, 1]] - corners[tri[:, 0]], corners[tri[:, 2]] - corners[tri[:, 0]]) * n[tri[:, 0]]).sum(axis=1)
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
    """A side's contoured 3D pearl-white outer bezel (`livery.PAD` shell + gunmetal rabbet & rivets) enclosing `plate`."""
    from coaxial.model.blocks import numpy as np
    g = _sculpt(side, drive, pelvis)
    p_out, n_out, p_in, n_in = g['p_out'], g['n_out'], g['p_in'], g['n_in']
    t_in = p_in - p_out
    t_in /= np.maximum(np.linalg.norm(t_in, axis=1, keepdims=True), 1e-9)
    r_in_f = p_in + PROUD_M * n_in
    r_mid_f = p_out + BEVEL_M * t_in + PROUD_M * n_out
    r_lip_f = p_out - 0.0008 * n_out
    r_out_b = p_out - RIM_T * n_out
    r_rab_o = p_in - 0.0020 * n_in
    r_rab_i = p_in + 0.0022 * t_in - 0.0020 * n_in
    r_rab_b = p_in + 0.0022 * t_in - RIM_T * n_in
    bands = [(_strip(r_in_f, r_mid_f, PAD), n_out), (_strip(r_mid_f, r_lip_f, PAD), n_out - 0.5 * t_in),
             (_strip(r_lip_f, r_out_b, FRAME), -t_in), (_strip(r_out_b, r_rab_b, FRAME), -n_out),
             (_strip(r_rab_b, r_rab_i, FRAME), t_in), (_strip(r_rab_i, r_rab_o, FRAME), n_in),
             (_strip(r_rab_o, r_in_f, FRAME), t_in)]
    fixed = []
    for (c, tri, uv, mat), ref_n in bands:
        n_avg = np.vstack([ref_n, ref_n])
        fn = np.cross(c[tri[:, 1]] - c[tri[:, 0]], c[tri[:, 2]] - c[tri[:, 0]])
        flip = (fn * n_avg[tri].mean(1)).sum(1) < 0.0
        tri = tri.copy()
        tri[flip] = tri[flip][:, ::-1]
        fixed.append((c, tri, uv, mat))
    for idx in (9, 27, 45, 63):
        mid = 0.54 * r_in_f[idx] + 0.46 * r_mid_f[idx]
        nrm = 0.5 * (n_in[idx] + n_out[idx])
        nrm /= np.linalg.norm(nrm)
        fixed.append(_prism_hex(mid, nrm, RIVET_R * 1.42, 0.00008, 0.00048, FRAME))
        fixed.append(_prism_hex(mid, nrm, RIVET_R, 0.00049, 0.00049 + RIVET_H, BOLT))
    base, cs, ts, ms = 0, [], [], []
    for c, t, _uv, m in fixed:
        cs.append(c), ts.append(t + base), ms.append(m)
        base += len(c)
    corners = np.vstack(cs) * g['flip']
    tri = np.vstack(ts)[:, ::int(math.copysign(1.0, side))]
    return corners, tri, np.zeros((len(corners), 2)), np.concatenate(ms)
