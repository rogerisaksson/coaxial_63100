"""Symbolic Robot 1.0 parts, clothes, livery, seat nets and skin neck for the built figure.

    parts = robot_worn.neck_skin(nod=0.0, head_yaw=0.0, like=likeness.FOUND)
    mesh = robot_worn.lit_mesh(roll=0.0, dressed=True, like=likeness.FOUND)

Provides closed C1 neck skin, Robot 1.0 shoes, hip seat nets (`seat.plate`), smooth polar-draped
clothes with continuous sleeves, magenta drive livery and vertex meshes for `gpu.LitRaster`.
"""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: Closed C1 neck skin loft stations in neck local frame: (y, rx, rz, z_centre), m.
NECK_RINGS = ((-0.040, 0.056, 0.056, -0.005), (-0.020, 0.050, 0.054, -0.004), (0.000, 0.046, 0.052, -0.003),
              (0.025, 0.043, 0.048, -0.002), (0.042, 0.040, 0.036, 0.008), (0.058, 0.038, 0.034, 0.011),
              (0.072, 0.034, 0.030, 0.012))
#: Height band in torso frame covered cleanly by `neck_skin`, head/cloth groups, hip seat stack and skin tint.
NECK_Y = (0.339, 0.462)
HEAD_LIKED = ('like_head', 'like_hair', 'like_fall')
HEAD_SYM = ('head', 'jaw', 'nose', 'lips', 'left_ear', 'right_ear', 'left_eye', 'right_eye', 'hair', 'hair_fall')
CLOTH_LIKE = ('like_torso', 'like_left_upper_arm', 'like_right_upper_arm', 'like_pelvis', 'like_left_thigh',
              'like_right_thigh', 'like_left_shank', 'like_right_shank')
CLOTH_SYM = ('cloth_tee', 'cloth_seat', 'cloth_left_bust', 'cloth_right_bust', 'cloth_left_inner',
             'cloth_right_inner', 'cloth_left_sleeve', 'cloth_right_sleeve', 'cloth_left_thigh',
             'cloth_right_thigh', 'cloth_left_shin', 'cloth_right_shin', 'cloth_left_leg', 'cloth_right_leg')
SEAT_STACK = ('roll still', 'roll guard', 'roll lid', 'roll leads', 'roll ring')
SKIN_TONED = (172, 138, 121)


@functools.lru_cache(maxsize=None)
def _body_cached(dressed=False, like=None):
    """Cached `(body, names, frames, pos)` for `gynoid.body` in root pelvis frame."""
    import numpy as np
    from coaxial.graphics import gynoid
    plates = bool(like) or (not dressed)
    body = gynoid.body(dressed, None, like if like else None, plates)
    names = [p[0] for p in body.parts]
    root = ((0.0, 0.0, 0.0), np.eye(3))
    frames = body._frames({}, root)
    pos, _n = body.pose({}, root=root, _frames=frames)
    return body, names, frames, pos


def _skin_colour(like):
    from coaxial.graphics import lit
    if like:
        return SKIN_TONED
    return tuple(int(round(255.0 * c)) for c in lit.PALETTE[lit.SKIN])


def neck_skin(nod=0.0, head_yaw=0.0, like=None):
    """Smooth closed neck and nape skin cover over the neck drive in root (`pelvis`) frame."""
    import numpy as np
    from coaxial.graphics import lit
    from coaxial.graphics.shapes import finer, loft
    _b, names, frames, _p = _body_cached(False, None)
    neck_at = frames[names.index('neck')][1]
    head_rel = frames[names.index('head')][1] - neck_at
    rgb = _skin_colour(like)
    corners, tris, _uv, _m = loft(finer(NECK_RINGS, -0.040, 0.072, 0.004), lit.paint(rgb), poles=(-0.044, 0.076))
    y_n = corners[:, 1].copy()
    if head_yaw:
        wy = np.clip((y_n - 0.040) / 0.032, 0.0, 1.0)
        wy = wy * wy * (3.0 - 2.0 * wy)
        ang = np.radians(head_yaw * wy)
        c, s = np.cos(ang), np.sin(ang)
        dx, dz = corners[:, 0] - head_rel[0], corners[:, 2] - head_rel[2]
        corners[:, 0] = head_rel[0] + c * dx + s * dz
        corners[:, 2] = head_rel[2] - s * dx + c * dz
    if nod:
        wn = np.clip((y_n + 0.036) / 0.061, 0.0, 1.0)
        wn = wn * wn * (3.0 - 2.0 * wn)
        ang = np.radians(nod * wn)
        c, s = np.cos(ang), np.sin(ang)
        cy, cz = corners[:, 1].copy(), corners[:, 2].copy()
        corners[:, 1] = c * cy - s * cz
        corners[:, 2] = s * cy + c * cz
    pts = corners + neck_at
    return [(pts[t], rgb) for t in tris]


def trim_neck(nm_dict, nod=0.0):
    """Keep torso hardware below `NECK_Y[0]`, dropping the internal neck drive enclosed by `neck_skin`."""
    import numpy as np
    from tools.render import neck_mount as nm
    drop = {'neck drive', 'neck ring', 'neck plate', 'cheek', 'shoe', 'neck hose', 'floor pipe',
            'neck leads', 'crown', 'head drive', 'head ring', 'carrier'}
    keep_below = {'block', 'pipes', 'head hose'}
    kept = []
    _b, names, frames, _p = _body_cached(False, None)
    ny, nz = float(frames[names.index('neck')][1][1] - frames[names.index('torso')][1][1]), float(frames[names.index('neck')][1][2])
    rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, math.cos(math.radians(nod)), math.sin(math.radians(nod))],
                   [0.0, -math.sin(math.radians(nod)), math.cos(math.radians(nod))]]) if nod else None
    for name, tris in nm_dict.items():
        if name.endswith(' shoulder') or name in drop:
            continue
        if name in keep_below:
            for tri, col in tris:
                p = tri
                if rx is not None and name in nm.NODS:
                    p = (tri - np.array([0.0, ny, nz])) @ rx.T + np.array([0.0, ny, nz])
                if float(p[:, 1].max()) < NECK_Y[0]:
                    kept.append((tri, col))
        else:
            kept.extend(tris)
    return kept


def _extract_root(body, names, pos, wanted, norm=None):
    """Extract `[(tri, rgb)]` and `[(c, t, u, m, n)]` for parts `wanted` in root frame."""
    import numpy as np
    from tools.render import leg_worn as lw
    out_tris, meshes = [], []
    owner = np.repeat(np.arange(len(names)), [len(part[5][1]) for part in body.parts])
    index = body.index.astype(int)
    for n in wanted:
        if n not in names:
            continue
        k = names.index(n)
        lo, hi = body.spans[k]
        t_local = index[owner == k] - lo
        c = pos[lo:hi].copy()
        m = body.materials[lo:hi].copy()
        uv = body.uv[lo:hi].copy()
        out_tris += lw._tris(c, t_local, m)
        vn = None if norm is None else norm[lo:hi] / np.maximum(np.linalg.norm(norm[lo:hi], axis=1, keepdims=True), 1e-12)
        meshes.append((c, t_local, uv, m, vn))
    return out_tris, meshes


def symbolic_head(nod=0.0, head_yaw=0.0, like=None, raw=False):
    """Head, face, hair and ponytail in root frame using likeness when given or Robot 1.0 symbolic head."""
    import numpy as np
    body, names, _f, _p = _body_cached(not bool(like), like)
    ang = {'neck': nod, 'head': head_yaw} if (nod or head_yaw) else {}
    pos, norm = body.pose(ang, root=((0.0, 0.0, 0.0), np.eye(3)))
    tris, meshes = _extract_root(body, names, pos, HEAD_LIKED if like else HEAD_SYM, norm if raw else None)
    return meshes if raw else tris


def symbolic_hands(roll=0.0, turn=0.0, flex=0.0, like=None):
    """Left and right coaxial 5-fingered carbon hands in root frame (`_put`-aligned with `arm_worn`)."""
    import numpy as np
    from coaxial.graphics import livery
    from tools.render import arm_worn as aw, knee_mount as km, leg_worn as lw
    body, names, frames, pos = _body_cached(False, like)
    turn_fa, spot_fa = frames[names.index('left_forearm')]
    turn_p, shift_p, bend_p = aw.placing()
    off = np.array([float(shift_p[0] * turn_p[0, 0]), 0.010, 0.0])
    turn_ua, _s = frames[names.index('left_upper_arm')]
    _b0, n0, f0, _p0 = _body_cached(False, None)
    spot_put = f0[n0.index('left_forearm')][1]
    _p_aw, shoulder, _w_aw = aw.laid(False, ())
    part_names = ('form_left_hand',) if (like and 'form_left_hand' in names) else ('left_hand', 'left_fingers')
    _tris, raw_meshes = _extract_root(body, names, pos, part_names)

    def _move(pts):
        p = (pts - spot_fa) @ turn_fa + off
        if flex:
            p = p @ km._rx(bend_p * flex).T
        if turn:
            p = (p - shoulder) @ km._rx(-turn).T + shoulder
        if roll:
            from tools.render import shoulder_rolled as sr
            piv_a = shoulder + np.array([-sr.X_ABD, 0.0, 0.0])
            c_r, s_r = math.cos(math.radians(roll)), math.sin(math.radians(roll))
            p = (p - piv_a) @ np.array([[c_r, -s_r, 0.0], [s_r, c_r, 0.0], [0.0, 0.0, 1.0]]).T + piv_a
        return p @ turn_ua.T + spot_put

    col = lw._rgb(livery.TUBE)
    left = [(tri, col) for c, idx_tris, *_ in raw_meshes for tri in _move(c)[idx_tris]]
    return left + [((tri * (-1.0, 1.0, 1.0))[::-1], col) for tri, _c in left]


def symbolic_shoes(dressed=False):
    """Robot 1.0 symbolic shoes (`sneaker.feet` and `sneaker.soles`) in root frame."""
    import numpy as np
    from coaxial.graphics import livery, sneaker
    from tools.render import leg_worn as lw
    _b, names, frames, _p = _body_cached(False, None)
    tris_out, upper = [], lw._rgb(livery.TUBE)
    trim, lace, cnt = ((sneaker.GUM, sneaker.LACE, sneaker.COUNTER) if dressed
                       else ((44, 46, 52), (54, 56, 62), (50, 52, 58)))
    for side in ('left', 'right'):
        for seg, mesh in sneaker.feet(side).items():
            rot, at = frames[names.index(seg)]
            c = mesh[0] @ rot.T + at
            tris_out += [(c[t], upper) for t in mesh[1]]
        for s_name, parent, off, mesh in sneaker.soles(side):
            rot, at = frames[names.index(parent)]
            c = (mesh[0] + np.asarray(off, float)) @ rot.T + at
            tris_out += [(c[t], lace if 'lace' in s_name else (cnt if 'counter' in s_name else trim))
                         for t in mesh[1]]
    return tris_out


def trim_leg(leg_tris, foot_y, dressed=False):
    """Trim foot keel and hose triangles enclosed by `symbolic_shoes`."""
    if not dressed:
        return [(t, col) for t, col in leg_tris if float(t[:, 1].max()) >= foot_y + 0.002]
    return [(t, col) for t, col in leg_tris
            if foot_y + 0.006 <= float(t[:, 1].min()) < foot_y + 0.028 and float(t[:, 2].max()) < 0.015]


@functools.lru_cache(maxsize=None)
def seat_nets():
    """Bilateral buttock black honeycomb nets (`seat.plate`) over `hip_clamped`'s rear hip roll drive."""
    import numpy as np
    from coaxial.graphics import livery, seat
    from machine.body.figure import HIP_DROP, HIP_HALF
    from tools.render import hip_clamped as hc, leg_worn as lw, trunk_clamped as tc, trunk_laid as tl
    hip_spot = np.array([HIP_HALF, -HIP_DROP, 0.0])
    yaw_spot = np.array([HIP_HALF, -HIP_DROP + 0.103, 0.0])
    links = hc.links()
    drive_meshes = []
    for lk, keys in (('yaw', SEAT_STACK + ('block', 'trunk', 'trunk clips', 'riser')), ('roll', ('kingpin',))):
        for k in keys:
            tris = [t for t, _c in (links[lk].get(k) or ()) if float(t[:, 2].min()) < 0.005]
            if tris:
                pts = np.array(tris).reshape(-1, 3) + hip_spot
                drive_meshes.append(('drive_' + k.replace(' ', '_'), pts, np.arange(len(pts)).reshape(-1, 3)))
    at_tr = np.asarray(tl.joint()[0], float)
    pelvis_pts = [np.array([t for ts in links['pelvis'].values() for t, _c in ts]).reshape(-1, 3) + hip_spot]
    tr_pel = [t + at_tr for (link, _n), ts in tc.posed().items() if link == 'pelvis' for t, _c in ts]
    if tr_pel:
        pelvis_pts.append(np.array(tr_pel).reshape(-1, 3))
    pelvis = [(pts, np.arange(len(pts)).reshape(-1, 3)) for pts in pelvis_pts if len(pts)]
    c_l, t_l, _uv, _m = seat.plate(1.0, (drive_meshes, yaw_spot), pelvis)
    c_r, t_r, ink = c_l * (-1.0, 1.0, 1.0), t_l[:, ::-1], lw._rgb(livery.VOID)
    return [(c_l[tri], ink) for tri in t_l] + [(c_r[tri], ink) for tri in t_r]


def arm_parts_dressed(flex=0.0, turn=0.0, roll=0.0):
    """Left arm CAD in `arm_worn` elbow frame extended continuously into the sleeve without gaps."""
    import numpy as np
    from tools.render import arm_worn as aw, knee_mount as km, shoulder_rolled as sr
    _p, shoulder, _w = aw.laid(False, ())
    arm_d = aw.arm(flex, roll, turn)
    low = [t for k in ('elbow', 'hose', 'forearm tube') for t in arm_d[k]]
    low += [(t, c) for t, c in arm_d['upper arm tube'] if float(t[:, 1].min()) < 0.245]
    gu_low = [(t, c) for t, c in aw._guarded()['left_upper_arm'] if float(t[:, 1].max()) < 0.175]
    bend_p = aw.placing()[2]
    gf = km._turn(aw._guarded()['left_forearm'], bend_p * flex) if flex else aw._guarded()['left_forearm']

    def _pose(tris):
        out = tris
        if turn:
            rx = km._rx(-turn)
            out = [((t - shoulder) @ rx.T + shoulder, col) for t, col in out]
        if roll:
            piv_a = shoulder + np.array([-sr.X_ABD, 0.0, 0.0])
            c_r, s_r = math.cos(math.radians(roll)), math.sin(math.radians(roll))
            rz = np.array([[c_r, -s_r, 0.0], [s_r, c_r, 0.0], [0.0, 0.0, 1.0]])
            out = [((t - piv_a) @ rz.T + piv_a, col) for t, col in out]
        return out

    return low + _pose(gu_low + gf), [t for part in arm_d.values() for t in part] + _pose(
        aw._guarded()['left_upper_arm'] + gf)


def _drape_polar(corners, obs_pts, axis_a, axis_b, gap=0.006, fall_at=None, max_push=0.028):
    """Smooth radial outward envelope drape along segment `axis_a..axis_b` preserving cloth folds."""
    import numpy as np
    from scipy.ndimage import maximum_filter1d, uniform_filter1d
    v = corners.copy()
    span = axis_b - axis_a
    L = max(float(np.linalg.norm(span)), 1e-6)
    u = span / L
    ref = np.array([0.0, 0.0, 1.0]) if abs(u[2]) < 0.85 else np.array([1.0, 0.0, 0.0])
    e1 = np.cross(u, ref)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(u, e1)

    def _cyl(q):
        rel = q - axis_a
        s = rel @ u
        x, z = rel @ e1, rel @ e2
        b = (np.arctan2(z, x) / (2.0 * math.pi) * 64.0).astype(int) % 64
        return s, np.hypot(x, z), x, z, b

    s_c, r_c, xc, zc, b_c = _cyl(v)
    s0, s1 = float(s_c.min()) - 0.01, float(s_c.max()) + 0.01
    row_m = 0.006
    n_r = max(4, int((s1 - s0) / row_m) + 1)
    row_c = np.clip(((s_c - s0) / row_m).astype(int), 0, n_r - 1)
    near_obs = obs_pts[((obs_pts >= v.min(0) - 0.04) & (obs_pts <= v.max(0) + 0.04)).all(axis=1)]
    if len(near_obs):
        so, ro, _xo, _zo, bo = _cyl(near_obs)
        keep = (so >= s0) & (so <= s1) & (ro < 0.24)
        need = np.zeros((n_r, 64))
        if keep.any():
            np.maximum.at(need, (np.clip(((so[keep] - s0) / row_m).astype(int), 0, n_r - 1), bo[keep]),
                          ro[keep] + gap)
        want = np.clip(need[row_c, b_c] - r_c, 0.0, max_push)
        push = np.zeros((n_r, 64))
        np.maximum.at(push, (row_c, b_c), want)
        eased = uniform_filter1d(uniform_filter1d(
            maximum_filter1d(maximum_filter1d(push, 5, axis=0, mode='nearest'), 5, axis=1, mode='wrap'),
            7, axis=0, mode='nearest'), 7, axis=1, mode='wrap')
        move = np.maximum(want, eased[row_c, b_c])
        if fall_at is not None:
            hang = max(0, int((fall_at - s0) / row_m))
            wide = np.zeros((n_r, 64))
            np.maximum.at(wide, (row_c, b_c), r_c + move)
            wide[:hang] = np.maximum.accumulate(wide[:hang][::-1], axis=0)[::-1]
            move = np.where(row_c < hang, np.maximum(move, wide[row_c, b_c] - r_c), move)
        sc = (r_c + move) / np.maximum(r_c, 1e-6)
        v += (sc - 1.0)[:, None] * (xc[:, None] * e1 + zc[:, None] * e2)
    return v


def _garment_bone(name, frames_d, names_d):
    """`(axis_a, axis_b, fall_s)` in root frame for cloth part `name`."""
    import numpy as np
    if 'torso' in name or 'tee' in name or 'bust' in name or 'pelvis' in name or 'seat' in name:
        p0 = frames_d[names_d.index('pelvis')][1] - (0.0, 0.14, 0.0)
        p1 = frames_d[names_d.index('neck')][1]
        return p0, p1, None
    side = 'left_' if 'left' in name else 'right_'
    if 'upper_arm' in name or 'sleeve' in name:
        return frames_d[names_d.index(side + 'upper_arm')][1], frames_d[names_d.index(side + 'forearm')][1], None
    if 'thigh' in name or 'inner' in name:
        return frames_d[names_d.index(side + 'thigh')][1], frames_d[names_d.index(side + 'shank')][1], None
    p_a, p_b = frames_d[names_d.index(side + 'shank')][1], frames_d[names_d.index(side + 'foot')][1]
    return p_b, p_a, float(np.linalg.norm(p_a - p_b)) - 0.04


def draped_clothes(roll=0.0, turn=0.0, flex=0.0, obs_tris=(), like=None, raw=False):
    """Smoothly draped clothes in root frame using Robot 1.0 likeness or symbolic `_wear()` meshes."""
    import numpy as np
    from tools.render import leg_worn as lw, robot_built as rb
    body_d, names_d, _fd, _pd = _body_cached(True, like)
    root = ((0.0, 0.0, 0.0), np.eye(3))
    angles = ({'t_pose': 1.0} if abs(roll - rb.T_POSE_ROLL) < 1e-6 and not turn and not flex else {
        'arms_out': roll, 'left_shoulder': turn, 'right_shoulder': turn, 'left_elbow': flex, 'right_elbow': flex})
    frames_d = body_d._frames(angles, root)
    pos_d, norm_d = body_d.pose(angles, root=root, _frames=frames_d)
    pts = np.vstack([t for t, _c in obs_tris]) if obs_tris else np.zeros((0, 3))
    _, idx = np.unique(np.floor(pts / 0.005).astype(np.int64), axis=0, return_index=True) if len(pts) else (None, [])
    obs_pts = pts[idx] if len(pts) else pts
    tris_out, meshes_out = [], []
    for name in (CLOTH_LIKE if like else CLOTH_SYM):
        if name not in names_d:
            continue
        k = names_d.index(name)
        lo, hi = body_d.spans[k]
        tris = body_d.parts[k][5][1]
        if tris is None or not len(tris):
            continue
        a, b, fall_s = _garment_bone(name, frames_d, names_d)
        v = _drape_polar(pos_d[lo:hi], obs_pts, a, b, fall_at=fall_s)
        mats = body_d.materials[lo:hi]
        if raw:
            vn = norm_d[lo:hi] / np.maximum(np.linalg.norm(norm_d[lo:hi], axis=1, keepdims=True), 1e-12)
            meshes_out.append((v, tris, body_d.uv[lo:hi], mats, vn))
        else:
            tris_out += [(v[t], lw._rgb(mats[t[0]])) for t in tris]
    return meshes_out if raw else tris_out


def apply_livery(tris):
    """Recolour anodized drive cans and gearboxes into Robot 1.0 magenta livery."""
    from tools.render import elbow_mount as em
    swap = {em.SERVO: (255, 48, 200), em.GEAR: (196, 36, 152)}
    return [(t, swap.get(c, c)) for t, c in tris]


def _smooth_normals(corners, tris, crease_deg=35.0):
    """Per-corner vertex normals smoothing curved faces within `crease_deg` while keeping sharp chamfers."""
    import numpy as np
    v = corners.reshape(-1, 3)
    fn = np.cross(v[tris[:, 1]] - v[tris[:, 0]], v[tris[:, 2]] - v[tris[:, 0]])
    fa = np.linalg.norm(fn, axis=1, keepdims=True)
    fu = fn / np.maximum(fa, 1e-12)
    flat_c = v[tris].reshape(-1, 3)
    flat_t = np.arange(len(flat_c)).reshape(-1, 3)
    if crease_deg >= 179.0:
        vn = np.zeros_like(v)
        for j in range(3):
            np.add.at(vn, tris[:, j], fn)
        vn /= np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-12)
        return flat_c, flat_t, vn[tris].reshape(-1, 3)
    cos_t = math.cos(math.radians(crease_deg))
    v_n = np.repeat(fu, 3, axis=0).copy()
    tri_map = {}
    for ti, tri in enumerate(tris):
        for vi in tri:
            tri_map.setdefault(int(vi), []).append(ti)
    for ti, tri in enumerate(tris):
        u0 = fu[ti]
        for j, vi in enumerate(tri):
            acc = np.zeros(3)
            for nb in tri_map[int(vi)]:
                if float(u0 @ fu[nb]) >= cos_t:
                    acc += fn[nb]
            nlen = float(np.linalg.norm(acc))
            if nlen > 1e-12:
                v_n[3 * ti + j] = acc / nlen
    return flat_c, flat_t, v_n


def lit_mesh(roll=0.0, dressed=False, turn=0.0, flex=0.0, nod=0.0, head_yaw=0.0, like=None):
    """Indexed `(positions, normals, uv, materials, index)` for `gpu.LitRaster` of Robot 2.0."""
    import numpy as np
    from coaxial.graphics import lit
    from tools.render import robot_built as rb
    whole = rb.whole(roll=roll, dressed=dressed, turn=turn, flex=flex, nod=nod, head_yaw=head_yaw, like=like)
    pos_l, nrm_l, uv_l, mat_l, idx_l, base = [], [], [], [], [], [0]

    def _push(c, t, u, m, n):
        pos_l.append(c)
        nrm_l.append(n)
        uv_l.append(u)
        mat_l.append(m)
        idx_l.append(t + base[0])
        base[0] += len(c)

    for mesh in symbolic_head(nod=nod, head_yaw=head_yaw, like=like, raw=True):
        _push(*mesh)
    if dressed:
        wb = rb.whole(roll=roll, dressed=False, turn=turn, flex=flex, nod=nod, head_yaw=head_yaw, like=like)
        obs = [t for k in ('pectoral', 'left leg', 'right leg', 'left arm', 'right arm', 'trunk', 'trunk guards')
               for t in wb.get(k, ())]
        for mesh in draped_clothes(roll=roll, turn=turn, flex=flex, obs_tris=obs, like=like, raw=True):
            _push(*mesh)
    for key, part_tris in whole.items():
        if not part_tris or key in ('head', 'garments'):
            continue
        raw = np.array([t for t, _c in part_tris], dtype=np.float64)
        idx = np.arange(3 * len(raw)).reshape(-1, 3)
        c_u, idx_u = np.unique(np.round(raw.reshape(-1, 3), 6), axis=0, return_inverse=True)
        _fc, _ft, vn = _smooth_normals(c_u, idx_u.reshape(-1, 3), 180.0 if key in ('hands', 'shoes', 'neck skin') else 38.0)
        c_arr = np.asarray([c for _t, c in part_tris], dtype=np.uint32)
        hard = key not in ('neck skin', 'seat')
        tri_mat = (np.uint32(lit.PAINTED) | (np.uint32(lit.HARD) if hard else np.uint32(0))
                   | np.where((c_arr == (64, 66, 68)).all(axis=1), np.uint32(lit.TWILL), np.uint32(0))
                   | (c_arr[:, 0] << np.uint32(16)) | (c_arr[:, 1] << np.uint32(8)) | c_arr[:, 2])
        _push(raw.reshape(-1, 3), idx, np.zeros((3 * len(raw), 2), dtype=np.float32), np.repeat(tri_mat, 3), vn)
    return (np.vstack(pos_l).astype(np.float32), np.vstack(nrm_l).astype(np.float32),
            np.vstack(uv_l).astype(np.float32), np.concatenate(mat_l).astype(np.uint32),
            np.vstack(idx_l).astype(np.uint32))


def checked():
    """Verify neck manifold, neck clearance, seat nets, sleeve continuity, and likeness/symbolic builds."""
    from coaxial.graphics import gynoid, likeness
    from tools.render import clips, robot_built as rb, solids
    nk = neck_skin(0.0, 0.0, likeness.FOUND)
    pr_n = solids.printable([1e3 * t for t, _c in nk])
    w_bare = rb.whole(0.0, False, 0.0, 0.0, nod=0.0, head_yaw=0.0, like=likeness.FOUND)
    n_cross = sum(clips.crossed(rb.whole(0.0, False, 0.0, 0.0, nod=nd, like=likeness.FOUND)['neck skin'], w_bare['neck'])
                  for nd in (-45.0, 0.0, 60.0))
    s_cross = clips.crossed(w_bare['seat'], w_bare['left leg'] + w_bare['right leg'] + w_bare['trunk'])
    w_sym_d = rb.whole(0.0, True, 0.0, 0.0, like=None)
    ns = {p[0]: len(p[5][1]) for p in gynoid.body(False, None, likeness.FOUND, True).parts}
    gap_mm = 0.0
    if likeness.FOUND:
        w_lk_d = rb.whole(0.0, True, 0.0, 0.0, like=likeness.FOUND)
        arm_y = max(float(t[:, 1].max()) for t, _c in w_lk_d['left arm'])
        slv_y = min(float(t[:, 1].min()) for t, _c in w_lk_d['garments']
                    if float(t[:, 0].mean()) > 0.08 and float(t[:, 1].mean()) > 0.30)
        gap_mm = round(1e3 * max(0.0, slv_y - arm_y), 2)
    ok = (int(pr_n['open']) == 0 and int(n_cross) == 0 and int(s_cross) == 0 and len(w_bare['seat']) >= 1000
          and gap_mm <= 0.0 and ns.get('skin_neck', 0) > 0 and ns.get('drive_neck', -1) == 0
          and 'shoes' in w_bare and {'garments', 'neck skin', 'shoes'} <= set(w_sym_d))
    return {'ok': ok, 'neck_open': int(pr_n['open']), 'neck_cross': int(n_cross), 'seat_cross': int(s_cross),
            'seat_tris': len(w_bare['seat']), 'skin': ns.get('skin_neck', 0), 'drive': ns.get('drive_neck', -1),
            'bare': sorted(w_bare.keys()), 'sym': sorted(w_sym_d.keys()), 'gap_mm': gap_mm}
