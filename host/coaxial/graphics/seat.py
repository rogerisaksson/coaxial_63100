"""The robot's seat bare: sculpted honeycomb nets and pearl-white rims over the hip roll drives.

    drive = ([(name, corners, triangles)], joint)   # what a side's hip yaw turns, at rest in the pelvis's frame
    parts += [('left_seat', 'left_hip_yaw', (), -joint, 0.0, seat.plate(1.0, drive, pelvis)), ...]
    parts += [('rim_left_seat', 'left_hip_yaw', (), -joint, 0.0, seat.rim(1.0, drive, pelvis)), ...]

A sculpted anatomical gluteal cheek a side where the robot's buttock stands: an open black honeycomb
(`plate`, `livery.VOID`) framed by a contoured 3D pearl-white outer bezel (`rim`, `livery.PAD` with
gunmetal bevel walls and rivets), fixed on the hip's roll drive stack a guard's air (`AIR_M`) off
the mechanism and clear of the pelvis through the hip yaw's travel. The silhouette forms a defined
intergluteal cleft with an upper sacral V-taper, a smooth inward iliac/waist taper above, rounded
fuller lower-outer gluteal volume, and a natural infragluteal crescent below.
"""
import math

from coaxial.graphics.livery import BOLT, FRAME, PAD, VOID
from machine.body import drives
from machine.body.guards import AIR_M

#: Honeycomb pitch across flats, opening share, cleft offset, and rim bezel width/wall/bevel/rivet sizes, m.
CELL, FILL, CLEFT = 0.0125, 0.64, 0.022
RIM_W, RIM_T, BEVEL_M, PROUD_M = 0.013, 0.0036, 0.0022, 0.0004
RIVET_R, RIVET_H = 0.0024, 0.0006
STACK = ('drive_', 'inv_', 'gear_', 'bolts_')
BEND: tuple = ()
AROUND, SWUNG = 72, 21
SPACING = AIR_M


def _dense(meshes, step=SPACING):
    """Points over the triangles of `meshes` [(corners, triangles)], none further than `step` from the next."""
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
    return np.vstack(out)


def _min_dist_yaw(pts, frame, joint, reach_yaw):
    """(dist_to_frame, min_x_over_yaw) for `pts` (n, 3) rotated about `joint` across `[-reach_yaw, reach_yaw]`."""
    from coaxial.model.blocks import numpy as np
    near = np.full(len(pts), np.inf)
    x_min = pts[:, 0].copy()
    if not len(pts):
        return near, x_min
    for turn in np.radians(np.linspace(-reach_yaw, reach_yaw, SWUNG)):
        c_t, s_t = math.cos(turn), math.sin(turn)
        rel = pts[:, [0, 2]] - joint[[0, 2]]
        p = np.stack([joint[0] + rel[:, 0] * c_t + rel[:, 1] * s_t, pts[:, 1],
                      joint[2] + rel[:, 1] * c_t - rel[:, 0] * s_t], 1)
        x_min = np.minimum(x_min, p[:, 0])
        if len(frame):
            box = frame[((frame > p.min(axis=0) - 0.02) & (frame < p.max(axis=0) + 0.02)).all(axis=1)]
            for chunk in (c for c in np.array_split(box, max(1, len(box) // 2000)) if len(c)):
                near = np.minimum(near, np.linalg.norm(p[:, None, :] - chunk[None, :, :], axis=2).min(axis=1))
    return near, x_min


def _depth_model(sx0, sx1, sy0, sy1, sz0, sz1):
    """Build anatomical bounds and smooth depth/normal evaluators `(bounds, depth_z, normal_at)`."""
    from coaxial.model.blocks import numpy as np
    cx_s, cy_s = 0.5 * (sx0 + sx1), 0.5 * (sy0 + sy1)
    r_s = 0.5 * max(sx1 - sx0, sy1 - sy0)
    w_hc = min(1.0, max(0.0, (-0.120 - sz0) / 0.015))
    y_bot, y_top = min(sy0 - 0.020, -0.088 - 0.002 * w_hc), max(sy1 + 0.010, 0.054)
    cy, y_apex = 0.45 * y_bot + 0.55 * y_top, 0.55 * y_bot + 0.45 * y_top
    x_in, x_out = CLEFT, min(max(sx1 + 0.021, 0.145), 0.163 + 0.005 * w_hc)
    cx = 0.46 * x_in + 0.54 * x_out
    x_apex = cx - 0.004
    req_rim = AIR_M + RIM_T
    z_apex = (1.0 - w_hc) * min(sz0 - req_rim - 0.012, -0.136) + w_hc * (sz0 - req_rim - 0.005)
    z_front, band = 0.5 * (sz0 + sz1), req_rim + 0.003

    def depth_z(x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        ux = (x - x_apex) / np.where(x >= x_apex, max(x_out - x_apex, 0.02), max(x_apex - x_in + 0.006, 0.02))
        vy = (y - y_apex) / np.where(y >= y_apex, max(y_top - y_apex + 0.01, 0.02), max(y_apex - y_bot + 0.006, 0.02))
        r2 = np.clip(ux * ux + vy * vy, 0.0, 1.8)
        cleft_tuck = (0.011 - 0.005 * w_hc) * np.exp(-((x - x_in) / 0.020) ** 2)
        lumbar = (0.014 - 0.004 * w_hc) * np.maximum(0.0, vy) ** 2.0
        infra = (0.010 - 0.004 * w_hc) * np.maximum(0.0, -vy) ** 2.2
        if w_hc > 0.0:
            z_hc = z_apex + 0.011 * (r2 ** 2.2) + cleft_tuck + lumbar + infra
            z_pl = z_front + (z_apex - z_front) * (1.0 - 0.30 * r2) + cleft_tuck + lumbar + infra
            z_nom = (1.0 - w_hc) * z_pl + w_hc * z_hc
        else:
            z_nom = z_front + (z_apex - z_front) * (1.0 - 0.30 * r2) + cleft_tuck + lumbar + infra
        d_rad = np.maximum(0.0, np.hypot(x - cx_s, y - cy_s) - r_s)
        z_cap = sz0 - np.sqrt(np.maximum(0.0, band * band - d_rad * d_rad) + 1e-8)
        k = 0.005
        return -k * np.log(np.exp(-z_nom / k) + np.exp(-z_cap / k))

    def normal_at(x, y):
        eps = 0.0015
        dz_dx = (depth_z(x + eps, y) - depth_z(x - eps, y)) / (2.0 * eps)
        dz_dy = (depth_z(x, y + eps) - depth_z(x, y - eps)) / (2.0 * eps)
        n = np.stack([dz_dx, dz_dy, -np.ones_like(dz_dx)], -1)
        return n / np.linalg.norm(n, axis=-1)[..., None]

    return (cx, cy, cx_s, cy_s, r_s, x_in, x_out, y_bot, y_top, w_hc), depth_z, normal_at


def _contour_xy(bounds):
    """Smooth C-infinity 2D gluteal outline `(t, dx_nom, dy_nom)` enclosing the drive circle and gluteal cheek."""
    from coaxial.model.blocks import numpy as np
    cx, cy, cx_s, cy_s, r_s, x_in, x_out, y_bot, y_top, w_hc = bounds
    t = np.arange(AROUND) * (2.0 * math.pi / AROUND)
    ct, st = np.cos(t), np.sin(t)
    rx_g, ry_g = np.where(ct >= 0.0, x_out - cx, cx - x_in), np.where(st >= 0.0, y_top - cy, cy - y_bot)
    s_top, s_bot, s_out, s_in = 0.5 * (1.0 + st), 0.5 * (1.0 - st), 0.5 * (1.0 + ct), 0.5 * (1.0 - ct)
    x_sc = 1.0 - 0.22 * (s_top ** 2.0) * (s_out ** 1.3) - 0.20 * (s_top ** 2.0) * (s_in ** 1.3) + 0.06 * (s_bot ** 1.6) * (s_out ** 1.5)
    y_sc = 1.0 - 0.08 * (ct ** 2) * s_bot
    p_shape = 2.15
    r_sup = (np.abs(ct) ** p_shape + np.abs(st) ** p_shape) ** (-1.0 / p_shape)
    gx, gy = cx + rx_g * x_sc * r_sup * ct, cy + ry_g * y_sc * r_sup * st
    r_drv = r_s + 0.008 + 0.005 * w_hc
    dx_d, dy_d = (cx_s + r_drv * ct) - cx, (cy_s + r_drv * st) - cy
    ang_d, rad_d = np.arctan2(dy_d, dx_d) % (2.0 * math.pi), np.hypot(dx_d, dy_d)
    ord_d = np.argsort(ang_d)
    ang_dp = np.r_[ang_d[ord_d][-1] - 2.0 * math.pi, ang_d[ord_d], ang_d[ord_d][0] + 2.0 * math.pi]
    rad_dp = np.r_[rad_d[ord_d][-1], rad_d[ord_d], rad_d[ord_d][0]]
    rad_drive_t, rad_glute_t = np.interp(t, ang_dp, rad_dp), np.hypot(gx - cx, gy - cy)
    ks = 0.006
    rad_nom = ks * np.log(np.exp(rad_glute_t / ks) + np.exp(rad_drive_t / ks))
    return t, rad_nom * ct, rad_nom * st


def _sculpt(side, drive, pelvis):
    """Shared anatomical gluteal surface and trimmed perimeter in canonical left (+x) pelvis rest frame."""
    from coaxial.model.blocks import numpy as np
    flip = np.array([side, 1.0, 1.0])
    stack = np.vstack([np.asarray(c, float) * flip for n, c, _t in drive[0] if n.startswith(STACK)])
    frame = _dense([(np.asarray(c, float) * flip, np.asarray(t_)) for c, t_ in pelvis], step=0.003)
    joint, reach_yaw = np.asarray(drive[1], float) * flip, max(abs(x) for x in drives.TRAVEL['hip_yaw'])
    sx0, sx1 = float(stack[:, 0].min()), float(stack[:, 0].max())
    sy0, sy1 = float(stack[:, 1].min()), float(stack[:, 1].max())
    sz0, sz1 = float(stack[:, 2].min()), float(stack[:, 2].max())

    bounds, depth_z, normal_at = _depth_model(sx0, sx1, sy0, sy1, sz0, sz1)
    cx, cy = bounds[0], bounds[1]
    t, dx_nom, dy_nom = _contour_xy(bounds)
    x_contra_lim = -(sx0 + 0.0045) + AIR_M + 0.0005

    def eval_rim_pts(scale):
        ox, oy = cx + scale * dx_nom, cy + scale * dy_nom
        oz = depth_z(ox, oy)
        on = normal_at(ox, oy)
        return np.stack([ox, oy, oz], 1), np.stack([ox, oy, oz], 1) - RIM_T * on, on

    scales = np.ones(AROUND)
    for _ in range(12):
        p_f, p_b, _n = eval_rim_pts(scales)
        d_f, x_f = _min_dist_yaw(p_f, frame, joint, reach_yaw)
        d_b, x_b = _min_dist_yaw(p_b, frame, joint, reach_yaw)
        bad = (p_f[:, 0] < CLEFT - 0.003) | (p_b[:, 0] < CLEFT - 0.003) | (x_f < x_contra_lim) | (x_b < x_contra_lim)
        if len(frame):
            bad |= (d_f < AIR_M + 0.002) | (d_b < AIR_M + 0.002)
        if not bad.any():
            break
        scales = np.where(bad, scales * 0.96, scales)

    for _ in range(4):
        scales = np.minimum(scales, 0.25 * np.roll(scales, -1) + 0.5 * scales + 0.25 * np.roll(scales, 1))

    ox, oy = cx + scales * dx_nom, cy + scales * dy_nom
    oz, on = depth_z(ox, oy), normal_at(ox, oy)
    p_out = np.stack([ox, oy, oz], 1)
    in_xy = np.stack([-dx_nom, -dy_nom], 1)
    in_xy /= np.maximum(np.linalg.norm(in_xy, axis=1, keepdims=True), 1e-9)
    eps = 0.0015
    dz_din = (depth_z(ox + eps * in_xy[:, 0], oy + eps * in_xy[:, 1]) - oz) / eps
    step_2d = RIM_W / np.sqrt(1.0 + dz_din * dz_din)
    ix, iy = ox + step_2d * in_xy[:, 0], oy + step_2d * in_xy[:, 1]
    p_in, i_n = np.stack([ix, iy, depth_z(ix, iy)], 1), normal_at(ix, iy)

    return {
        'flip': flip, 'joint': joint, 'reach_yaw': reach_yaw, 'frame': frame, 'stack': stack,
        'cx': cx, 'cy': cy, 't': t, 'ox': ox, 'oy': oy, 'ix': ix, 'iy': iy,
        'p_out': p_out, 'n_out': on, 'p_in': p_in, 'n_in': i_n,
        'depth_z': depth_z, 'normal_at': normal_at,
    }


def plate(side, drive, pelvis=()):
    """A side's sculpted black honeycomb net in pelvis rest frame, `side` 1 for left, clear of `drive` and `pelvis`."""
    from coaxial.model.blocks import numpy as np
    g = _sculpt(side, drive, pelvis)
    cx, cy, ix, iy = g['cx'], g['cy'], g['ix'], g['iy']
    r_in = np.hypot(ix - cx, iy - cy)
    ang_in = np.arctan2(iy - cy, ix - cx) % (2.0 * math.pi)
    order = np.argsort(ang_in)
    ang_s, r_s = ang_in[order], r_in[order]
    ang_pad = np.r_[ang_s[-1] - 2.0 * math.pi, ang_s, ang_s[0] + 2.0 * math.pi]
    r_pad = np.r_[r_s[-1], r_s, r_s[0]]

    max_r = float(r_in.max())
    span_x = int(max_r / CELL) + 2
    rise = CELL * math.sqrt(3.0) / 2.0
    span_y = int(max_r / rise) + 2
    mids = np.array([(i * CELL + (j % 2) * CELL / 2.0, j * rise)
                     for j in range(-span_y, span_y + 1) for i in range(-span_x, span_x + 1)])
    hexa = np.arange(6) * (math.pi / 3.0) + math.pi / 6.0
    unit = np.stack([np.cos(hexa), np.sin(hexa)], 1) * CELL / math.sqrt(3.0)
    flat = mids[:, None, :] + np.concatenate([unit, FILL * unit])[None]

    px, py = cx + flat[..., 0], cy + flat[..., 1]
    way = np.arctan2(flat[..., 1], flat[..., 0]) % (2.0 * math.pi)
    reach = np.interp(way, ang_pad, r_pad)
    inside = (np.hypot(flat[..., 0], flat[..., 1]) < reach - 0.0005)[:, :6].all(axis=1)

    pts_3d = np.stack([px[inside].ravel(), py[inside].ravel(), g['depth_z'](px[inside], py[inside]).ravel()], 1)
    if len(g['frame']) and len(pts_3d):
        d_pel, _ = _min_dist_yaw(pts_3d, g['frame'], g['joint'], g['reach_yaw'])
        keep_pel = (d_pel.reshape(-1, 12) >= AIR_M).all(axis=1)
        inside[np.flatnonzero(inside)[~keep_pel]] = False

    fx, fy = px[inside].ravel(), py[inside].ravel()
    corners = np.stack([fx, fy, g['depth_z'](fx, fy)], 1)
    n = g['normal_at'](fx, fy)
    k = np.arange(6)
    quads = np.concatenate([np.stack([k, (k + 1) % 6, 6 + (k + 1) % 6], 1), np.stack([k, 6 + (k + 1) % 6, 6 + k], 1)])
    tri = (12 * np.arange(int(inside.sum()))[:, None, None] + quads[None]).reshape(-1, 3)
    facing = (np.cross(corners[tri[:, 1]] - corners[tri[:, 0]], corners[tri[:, 2]] - corners[tri[:, 0]]) * n[tri[:, 0]]).sum(axis=1)
    tri[facing < 0.0] = tri[facing < 0.0][:, ::-1]
    laid = corners * g['flip']
    return laid, tri[:, ::int(math.copysign(1.0, side))], np.zeros((len(laid), 2)), np.full(len(laid), VOID)


def _strip(ring_a, ring_b, mat):
    """Closed quad-band mesh between two aligned perimeter rings (AROUND, 3)."""
    from coaxial.model.blocks import numpy as np
    m = len(ring_a)
    corners = np.vstack([ring_a, ring_b])
    k = np.arange(m)
    nxt = (k + 1) % m
    tri = np.vstack([np.stack([k, nxt, m + nxt], 1), np.stack([k, m + nxt, m + k], 1)])
    return corners, tri, np.zeros((len(corners), 2)), np.full(len(corners), mat)


def _prism_hex(centre, normal, radius, z0, z1, mat):
    """Watertight hexagonal prism along `normal` from `z0` to `z1`, tapered `0.88` on top."""
    from coaxial.model.blocks import numpy as np
    ref = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.85 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(normal, ref)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(normal, e1)
    ang = np.arange(6) * (math.pi / 3.0)
    ring = radius * (np.outer(np.cos(ang), e1) + np.outer(np.sin(ang), e2))
    base = centre + z0 * normal + ring
    top = centre + z1 * normal + 0.88 * ring
    bot_c = centre + z0 * normal
    top_c = centre + z1 * normal
    corners = np.vstack([base, top, bot_c[None], top_c[None]])
    tris = []
    for i in range(6):
        j = (i + 1) % 6
        tris += [(i, j, 6 + j), (i, 6 + j, 6 + i), (12, j, i), (13, 6 + i, 6 + j)]
    return corners, np.array(tris), np.zeros((len(corners), 2)), np.full(len(corners), mat)


def _rivet(centre, normal):
    """Watertight gunmetal-collared `BOLT` rivet cap seated cleanly on the bezel face."""
    collar = _prism_hex(centre, normal, RIVET_R * 1.38, 0.00008, 0.00045, FRAME)
    head = _prism_hex(centre, normal, RIVET_R, 0.00046, 0.00046 + RIVET_H, BOLT)
    return collar, head


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
    r_in_b = p_in - RIM_T * n_in
    # Recessed gunmetal rabbet ledge behind the honeycomb perimeter (`-2.0 mm` behind `plate` -> zero crossing).
    r_rab_o = p_in - 0.0020 * n_in
    r_rab_i = p_in + 0.0022 * t_in - 0.0020 * n_in
    r_rab_b = p_in + 0.0022 * t_in - RIM_T * n_in

    bands = [
        _strip(r_in_f, r_mid_f, PAD),
        _strip(r_mid_f, r_lip_f, PAD),
        _strip(r_lip_f, r_out_b, FRAME),
        _strip(r_out_b, r_rab_b, FRAME),
        _strip(r_rab_b, r_rab_i, FRAME),
        _strip(r_rab_i, r_rab_o, FRAME),
        _strip(r_rab_o, r_in_f, FRAME),
    ]
    fixed = []
    band_norms = [n_out, n_out + 0.5 * (-t_in), -t_in, -n_out, t_in, n_in, t_in]
    for (c, tri, uv, mat), ref_n in zip(bands, band_norms):
        n_avg = np.vstack([ref_n, ref_n])
        fn = np.cross(c[tri[:, 1]] - c[tri[:, 0]], c[tri[:, 2]] - c[tri[:, 0]])
        if (fn * n_avg[tri[:, 0]]).sum() < 0.0:
            tri = tri[:, ::-1]
        fixed.append((c, tri, uv, mat))

    for idx in (9, 27, 45, 63):
        mid = 0.52 * r_in_f[idx] + 0.48 * r_mid_f[idx]
        nrm = 0.5 * (n_in[idx] + n_out[idx])
        nrm /= np.linalg.norm(nrm)
        fixed.extend(_rivet(mid, nrm))

    base, cs, ts, ms = 0, [], [], []
    for c, t, _uv, m in fixed:
        cs.append(c)
        ts.append(t + base)
        ms.append(m)
        base += len(c)
    corners = np.vstack(cs) * g['flip']
    tri = np.vstack(ts)[:, ::int(math.copysign(1.0, side))]
    return corners, tri, np.zeros((len(corners), 2)), np.concatenate(ms)
