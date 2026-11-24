"""Symbolic parts, clothes, livery, seat nets and skin neck for Robot 2.0.

    parts = robot_worn.neck_skin(nod=0.0, head_yaw=0.0, like=likeness.FOUND)
    mesh = robot_worn.lit_mesh(roll=0.0, dressed=True, like=likeness.FOUND)
"""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render.face_laid import np_mod as _np  # noqa: E402

NECK_RINGS = ((-0.040, 0.056, 0.056, -0.005), (-0.020, 0.050, 0.054, -0.004), (0.000, 0.046, 0.052, -0.003),
              (0.025, 0.043, 0.048, -0.002), (0.042, 0.040, 0.036, 0.008), (0.058, 0.036, 0.032, 0.011),
              (0.074, 0.030, 0.026, 0.012))
NECK_Y, SKIN_TONED = (0.339, 0.462), (172, 138, 121)
HEAD_LIKED = ('like_head', 'like_hair', 'like_fall')
HEAD_SYM = ('head', 'jaw', 'nose', 'lips', 'left_ear', 'right_ear', 'left_eye', 'right_eye', 'hair', 'hair_fall')
CLOTH_LIKE = tuple('like_' + s for s in ('torso', 'left_upper_arm', 'right_upper_arm', 'pelvis',
                                         'left_thigh', 'right_thigh', 'left_shank', 'right_shank'))
CLOTH_SYM = tuple('cloth_' + s for s in ('tee', 'seat') + tuple(
    a + b for a in ('left_', 'right_') for b in ('bust', 'inner', 'sleeve', 'thigh', 'shin', 'leg')))
SEAT_STACK = tuple('roll ' + s for s in ('still', 'guard', 'lid', 'leads', 'ring'))


def _step(x):
    u = _np().clip(x, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def _unit(x):
    np = _np()
    return x / np.maximum(np.linalg.norm(x, axis=-1, keepdims=True), 1e-12)


def _accum(n_v, idx, fn):
    np = _np()
    out = np.zeros((n_v, 3))
    for j in range(3):
        np.add.at(out, idx[:, j], fn)
    return out


@functools.lru_cache(maxsize=None)
def _body_cached(dressed=False, like=None):
    from coaxial.graphics import gynoid
    body = gynoid.body(dressed, None, like or None, bool(like) or (not dressed))
    names, root = [p[0] for p in body.parts], ((0.0, 0.0, 0.0), _np().eye(3))
    frames = body._frames({}, root)
    return body, names, frames, body.pose({}, root=root, _frames=frames)[0]


def _skin_colour(like):
    from coaxial.graphics import lit
    return SKIN_TONED if like else tuple(int(round(255.0 * c)) for c in lit.PALETTE[lit.SKIN])


def neck_skin(nod=0.0, head_yaw=0.0, like=None):
    """Smooth closed neck and nape skin cover over the neck drive in root frame."""
    from coaxial.graphics import lit
    from coaxial.graphics.shapes import finer, loft
    np, (_b, nm, fr, _p), rgb = _np(), _body_cached(False, None), _skin_colour(like)
    nat = fr[nm.index('neck')][1]
    hrel = fr[nm.index('head')][1] - nat
    c, tris, _u, _m = loft(finer(NECK_RINGS, -0.040, 0.074, 0.004), lit.paint(rgb), poles=(-0.044, 0.076))
    yn = c[:, 1].copy()
    if head_yaw:
        a = np.radians(head_yaw * _step((yn - 0.040) / 0.034))
        ca, sa, dx, dz = np.cos(a), np.sin(a), c[:, 0] - hrel[0], c[:, 2] - hrel[2]
        c[:, 0], c[:, 2] = hrel[0] + ca * dx + sa * dz, hrel[2] - sa * dx + ca * dz
    if nod:
        a = np.radians(nod * _step((yn + 0.036) / 0.061))
        ca, sa, cy, cz = np.cos(a), np.sin(a), c[:, 1].copy(), c[:, 2].copy()
        c[:, 1], c[:, 2] = ca * cy - sa * cz, sa * cy + ca * cz
    return [((c + nat)[t], rgb) for t in tris]


def trim_neck(nm_dict, nod=0.0):
    """Keep torso hardware below `NECK_Y[0]`, dropping the internal neck drive."""
    from tools.render import knee_mount as km, neck_mount as nm
    drop = {'neck drive', 'neck ring', 'neck plate', 'cheek', 'shoe', 'neck hose', 'floor pipe',
            'neck leads', 'crown', 'head drive', 'head ring', 'carrier'}
    (_b, ns, fr, _p), rx = _body_cached(False, None), km._rx(-nod) if nod else None
    piv = _np().array([0.0, float(fr[ns.index('neck')][1][1] - fr[ns.index('torso')][1][1]), float(fr[ns.index('neck')][1][2])])
    lim = {'block', 'pipes', 'head hose'}
    return [(t, c) for n, ts in nm_dict.items() if not n.endswith(' shoulder') and n not in drop for t, c in ts
            if n not in lim or float(((t - piv) @ rx.T + piv if (rx is not None and n in nm.NODS) else t)[:, 1].max()) < NECK_Y[0]]


def _weld_rgb(n_u, inv, tu, m, steps=4):
    np = _np()
    rgb, cnt = np.zeros((n_u, 3)), np.zeros((n_u, 1))
    np.add.at(rgb, inv, np.stack([(m >> 16) & 255, (m >> 8) & 255, m & 255], 1))
    np.add.at(cnt, inv, 1.0)
    rgb /= np.maximum(cnt, 1.0)
    src, dst = tu[:, [0, 1, 2, 1, 2, 0]].ravel(), tu[:, [1, 2, 0, 0, 1, 2]].ravel()
    deg = np.maximum(np.bincount(src, minlength=n_u), 1)[:, None]
    for _ in range(steps):
        acc = np.zeros_like(rgb)
        np.add.at(acc, src, rgb[dst])
        rgb = 0.4 * rgb + 0.6 * (acc / deg)
    return rgb


def _smooth_hair(c, t, uv, m):
    from coaxial.graphics import lit
    np = _np()
    c_u, inv = np.unique(np.round(c, 5), axis=0, return_inverse=True)
    tu = inv[t]
    _fc, _ft, vn = _smooth_normals(c_u, tu, 180.0)
    trgb = np.clip(np.rint(_weld_rgb(len(c_u), inv, tu, m, 5)[tu].mean(1)), 0, 255).astype(np.uint32)
    tmat = np.uint32(lit.PAINTED) | (trgb[:, 0] << 16) | (trgb[:, 1] << 8) | trgb[:, 2]
    return c[t].reshape(-1, 3), np.arange(3 * len(t)).reshape(-1, 3), uv[t].reshape(-1, 2), np.repeat(tmat, 3), vn


def _extract_root(body, names, pos, wanted, norm=None):
    """Extract parts in root frame."""
    import numpy as np
    from coaxial.graphics import panels
    from tools.render import leg_worn as lw
    out_tris, meshes, idx = [], [], body.index.astype(int)
    owner = np.repeat(np.arange(len(names)), [len(p[5][1]) for p in body.parts])
    for n in wanted:
        if n not in names:
            continue
        k = names.index(n)
        lo, hi = body.spans[k]
        tl, c, m, uv = idx[owner == k] - lo, pos[lo:hi].copy(), body.materials[lo:hi].copy(), body.uv[lo:hi].copy()
        vn = None if norm is None else _unit(norm[lo:hi])
        if n == 'like_head':
            tl = tl[~(m[tl] == panels.OPEN).any(1)]
        elif n in ('like_hair', 'like_fall'):
            c, tl, uv, m, vns = _smooth_hair(c, tl, uv, m)
            vn = vns if vn is not None else None
        out_tris += lw._tris(c, tl, m)
        meshes.append((c, tl, uv, m, vn))
    return out_tris, meshes


def symbolic_head(nod=0.0, head_yaw=0.0, like=None, raw=False):
    """Head, face, and hair in root frame."""
    import numpy as np
    body, names, _f, _p = _body_cached(not bool(like), like)
    ang = {'neck': nod, 'head': head_yaw} if (nod or head_yaw) else {}
    pos, norm = body.pose(ang, root=((0.0, 0.0, 0.0), np.eye(3)))
    tris, meshes = _extract_root(body, names, pos, HEAD_LIKED if like else HEAD_SYM, norm if raw else None)
    return meshes if raw else tris


def _shoulder_xform(pts, shoulder, turn, roll):
    import numpy as np
    from tools.render import knee_mount as km
    if turn:
        pts = (pts - shoulder) @ km._rx(-turn).T + shoulder
    if roll:
        from tools.render import shoulder_rolled as sr
        pa = shoulder + np.array([-sr.X_ABD, 0.0, 0.0])
        c, s = math.cos(math.radians(roll)), math.sin(math.radians(roll))
        pts = (pts - pa) @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]).T + pa
    return pts


def symbolic_hands(roll=0.0, turn=0.0, flex=0.0, like=None):
    """Left and right coaxial 5-fingered carbon hands in root frame (`_put`-aligned with `arm_worn`)."""
    import numpy as np
    from coaxial.graphics import livery
    from tools.render import arm_worn as aw, knee_mount as km, leg_worn as lw
    bd, nm, fr, pos = _body_cached(False, like)
    (tfa, sfa), (tp, sp, bp), tua = fr[nm.index('left_forearm')], aw.placing(), fr[nm.index('left_upper_arm')][0]
    off = np.array([float(sp[0] * tp[0, 0]), 0.010, 0.0])
    (_b0, n0, f0, _p0), sh = _body_cached(False, None), aw.laid(False, ())[1]
    sput = f0[n0.index('left_forearm')][1]
    pns = ('form_left_hand',) if (like and 'form_left_hand' in nm) else ('left_hand', 'left_fingers')
    mv = lambda q: _shoulder_xform(((q - sfa) @ tfa + off) @ (km._rx(bp * flex).T if flex else np.eye(3)), sh, turn, roll) @ tua.T + sput
    col = lw._rgb(livery.TUBE)
    left = [(tri, col) for c, it, *_ in _extract_root(bd, nm, pos, pns)[1] for tri in mv(c)[it]]
    return left + [((t * (-1.0, 1.0, 1.0))[::-1], col) for t, _c in left]


def symbolic_shoes(dressed=False):
    """Robot 1.0 symbolic shoes in root frame."""
    import numpy as np
    from coaxial.graphics import livery, sneaker
    from tools.render import leg_worn as lw
    _b, nm, fr, _p = _body_cached(False, None)
    out, up = [], lw._rgb(livery.TUBE)
    trim, lace, cnt = (sneaker.GUM, sneaker.LACE, sneaker.COUNTER) if dressed else ((44, 46, 52), (54, 56, 62), (50, 52, 58))
    for s in ('left', 'right'):
        for seg, ms in sneaker.feet(s).items():
            rot, at = fr[nm.index(seg)]
            out += [((ms[0] @ rot.T + at)[t], up) for t in ms[1]]
        for sn, par, off, ms in sneaker.soles(s):
            rot, at = fr[nm.index(par)]
            c, col = (ms[0] + np.asarray(off, float)) @ rot.T + at, lace if 'lace' in sn else (cnt if 'counter' in sn else trim)
            out += [(c[t], col) for t in ms[1]]
    return out


def trim_leg(leg_tris, foot_y, dressed=False):
    """Trim foot keel and hose triangles enclosed by `symbolic_shoes`."""
    return [(t, col) for t, col in leg_tris if (
        foot_y + 0.006 <= float(t[:, 1].min()) < foot_y + 0.028 and float(t[:, 2].max()) < 0.015 if dressed
        else float(t[:, 1].max()) >= foot_y + 0.002)]


@functools.lru_cache(maxsize=None)
def seat_nets():
    """Bilateral buttock honeycomb nets (`seat.plate`) and white rims (`seat.rim`) over `hip_clamped`."""
    from coaxial.graphics import livery, seat
    from coaxial.graphics.lit import PAINTED
    from machine.body.figure import HIP_DROP, HIP_HALF
    from tools.render import hip_clamped as hc, leg_worn as lw, trunk_clamped as tc, trunk_laid as tl
    np = _np()
    hip_spot, yaw_spot = np.array([HIP_HALF, -HIP_DROP, 0.0]), np.array([HIP_HALF, -HIP_DROP + 0.103, 0.0])
    links, p3, t3 = hc.links(), lambda ts: np.array(ts).reshape(-1, 3), lambda p: np.arange(len(p)).reshape(-1, 3)
    drives = [('drive_' + k.replace(' ', '_'), p := p3(ts) + hip_spot, t3(p))
              for k in SEAT_STACK if (ts := [t for t, _c in (links['yaw'].get(k) or ()) if float(t[:, 2].min()) < 0.005])]
    at_tr = np.asarray(tl.joint()[0], float)
    raw_p = [
        ([t for ts in links['pelvis'].values() for t, _c in ts], hip_spot),
        ([t + at_tr for (lk, _n), ts in tc.posed().items() if lk == 'pelvis' for t, _c in ts], 0.0),
    ]
    pelvis = [(p := p3(ts) + off, t3(p)) for ts, off in raw_p if ts]
    c_l, t_l, _u, _m = seat.plate(1.0, (drives, yaw_spot), pelvis)
    rc_l, rt_l, _ru, rm = seat.rim(1.0, (drives, yaw_spot), pelvis)
    c_r, t_r, rc_r, rt_r = c_l * (-1.0, 1.0, 1.0), t_l[:, ::-1], rc_l * (-1.0, 1.0, 1.0), rt_l[:, ::-1]
    ink, pad = lw._rgb(livery.VOID), (234, 238, 244)
    cols = [lw._rgb(rm[t[0]]) if int(rm[t[0]]) >= PAINTED else pad for t in rt_l]
    return ([(c_l[t], ink) for t in t_l] + [(c_r[t], ink) for t in t_r]
            + [(rc_l[t], col) for t, col in zip(rt_l, cols)] + [(rc_r[t], col) for t, col in zip(rt_r, cols)])


def arm_parts_dressed(flex=0.0, turn=0.0, roll=0.0):
    """Left arm CAD in `arm_worn` elbow frame extended into the sleeve without gaps."""
    from tools.render import arm_worn as aw, knee_mount as km
    sh, ad, gd = aw.laid(False, ())[1], aw.arm(flex, roll, turn), aw._guarded()
    low = [t for k in ('elbow', 'hose', 'forearm tube') for t in ad[k]] + [(t, c) for t, c in ad['upper arm tube'] if float(t[:, 1].min()) < 0.245]
    gul = [(t, c) for t, c in gd['left_upper_arm'] if float(t[:, 1].max()) < 0.105]
    gf = km._turn(gd['left_forearm'], aw.placing()[2] * flex) if flex else gd['left_forearm']
    pose = lambda ts: [(_shoulder_xform(t, sh, turn, roll), col) for t, col in ts] if (turn or roll) else ts
    return low + pose(gul + gf), [t for p in ad.values() for t in p] + pose(gd['left_upper_arm'] + gf)


def _drape_polar(corners, obs_pts, axis_a, axis_b, gap=0.0036, fall_at=None, max_push=0.016):
    """Smooth radial outward envelope drape along segment `axis_a..axis_b` preserving cloth folds."""
    import numpy as np
    from scipy.ndimage import maximum_filter1d, uniform_filter1d
    v, span = corners.copy(), axis_b - axis_a
    u = span / max(float(np.linalg.norm(span)), 1e-6)
    e1 = _unit(np.cross(u, np.array([0.0, 0.0, 1.0]) if abs(u[2]) < 0.85 else np.array([1.0, 0.0, 0.0])))
    e2 = np.cross(u, e1)
    cyl = lambda q: ((r := q - axis_a) @ u, np.hypot(x := r @ e1, z := r @ e2), x, z, (np.arctan2(z, x) / (2 * math.pi) * 64).astype(int) % 64)
    sc, rc, xc, zc, bc = cyl(v)
    s0, row_m = float(sc.min()) - 0.01, 0.006
    nr = max(4, int((float(sc.max()) + 0.01 - s0) / row_m) + 1)
    rf, bf = np.clip((sc - s0) / row_m, 0.0, nr - 1.0), np.mod(np.arctan2(zc, xc) / (2.0 * math.pi) * 64.0, 64.0)
    r0, b0 = np.clip(np.floor(rf).astype(int), 0, nr - 1), np.floor(bf).astype(int) % 64
    r1, b1, wr, wb = np.minimum(r0 + 1, nr - 1), (b0 + 1) % 64, rf - r0, bf - np.floor(bf)
    bil = lambda g: (1.0 - wr) * ((1.0 - wb) * g[r0, b0] + wb * g[r0, b1]) + wr * ((1.0 - wb) * g[r1, b0] + wb * g[r1, b1])
    f, rowc = lambda g, fn, n: fn(fn(g, n, 0, mode='nearest'), n, 1, mode='wrap'), np.clip(np.round(rf).astype(int), 0, nr - 1)
    near = obs_pts[((obs_pts >= v.min(0) - 0.04) & (obs_pts <= v.max(0) + 0.04)).all(1)]
    if len(near):
        so, ro, _x, _z, bo = cyl(near)
        keep, need, push = (so >= s0) & (so <= float(sc.max()) + 0.01) & (ro < 0.24), np.zeros((nr, 64)), np.zeros((nr, 64))
        if keep.any():
            np.maximum.at(need, (np.clip(((so[keep] - s0) / row_m).astype(int), 0, nr - 1), bo[keep]), ro[keep] + gap)
        want = np.clip(bil(f(need, maximum_filter1d, 3)) - rc, 0.0, max_push)
        np.maximum.at(push, (rowc, bc), want)
        move = np.maximum(want, bil(f(f(push, maximum_filter1d, 5), uniform_filter1d, 7)))
        if fall_at is not None:
            hg, wide = max(0, int((fall_at - s0) / row_m)), np.zeros((nr, 64))
            np.maximum.at(wide, (rowc, bc), rc + move)
            wide[:hg] = np.maximum.accumulate(wide[:hg][::-1], 0)[::-1]
            move = np.where(rowc < hg, np.maximum(move, 0.35 * (bil(wide) - rc) + 0.65 * move), move)
        v += ((rc + move) / np.maximum(rc, 1e-6) - 1.0)[:, None] * (xc[:, None] * e1 + zc[:, None] * e2)
    return v


def _garment_bone(name, fd, nd):
    """`(axis_a, axis_b, fall_s)` in root frame for cloth part `name`."""
    import numpy as np
    if any(k in name for k in ('torso', 'tee', 'bust', 'pelvis', 'seat')):
        return fd[nd.index('pelvis')][1] - (0.0, 0.14, 0.0), fd[nd.index('neck')][1], None
    side = 'left_' if 'left' in name else 'right_'
    if 'upper_arm' in name or 'sleeve' in name:
        return fd[nd.index(side + 'upper_arm')][1], fd[nd.index(side + 'forearm')][1], None
    if 'thigh' in name or 'inner' in name:
        return fd[nd.index(side + 'thigh')][1], fd[nd.index(side + 'shank')][1], None
    pa, pb = fd[nd.index(side + 'shank')][1], fd[nd.index(side + 'foot')][1]
    return pb, pa, float(np.linalg.norm(pa - pb)) - 0.04


def _slim_bust(corners, torso_y):
    """Subtly slim anterior bust protrusion on `like_torso` before polar draping."""
    import numpy as np
    v, dy = corners.copy(), corners[:, 1] - torso_y
    w = _step(1.0 - np.abs(dy - 0.205) / 0.095) * _step(1.0 - np.abs(v[:, 0]) / 0.135)
    band = (dy > 0.11) & (dy < 0.30) & (v[:, 2] > 0.01)
    cz = float(np.percentile(v[band, 2], 40)) if band.any() else 0.070
    v[:, 2] -= 0.24 * w * np.maximum(v[:, 2] - cz, 0.0)
    return v


def _cloth_normals(v, tris, vn0, mats):
    from coaxial.graphics import lit
    np = _np()
    fn = np.cross(v[tris[:, 1]] - v[tris[:, 0]], v[tris[:, 2]] - v[tris[:, 0]])
    _, inv = np.unique(np.round(v, 5), axis=0, return_inverse=True)
    n_u, ti, vn_u = int(inv.max()) + 1, inv[tris], np.zeros((1, 3))
    for use_vn in (False, True):
        flp = (fn * (vn_u[ti] if use_vn else vn0[tris]).mean(1)).sum(1) < 0.0
        if flp.any():
            tris[flp, 1], tris[flp, 2] = tris[flp, 2].copy(), tris[flp, 1].copy()
            fn[flp] = -fn[flp]
        if not use_vn or flp.any():
            vn_u = _accum(n_u, ti, fn)
    norms = np.linalg.norm(vn_u, axis=1, keepdims=True)
    vr = np.clip(np.rint(_weld_rgb(n_u, inv, ti, mats, 4)[inv]), 0, 255).astype(np.uint32)
    vm = (mats.astype(np.uint32) & ~np.uint32(0xFFFFFF)) | np.uint32(lit.PAINTED | lit.WOVEN) | (vr[:, 0] << 16) | (vr[:, 1] << 8) | vr[:, 2]
    return tris, np.where(norms[inv] > 1e-12, vn_u[inv] / np.maximum(norms[inv], 1e-12), _unit(vn0)), vm


def draped_clothes(roll=0.0, turn=0.0, flex=0.0, obs_tris=(), like=None, raw=False):
    from tools.render import leg_worn as lw, robot_built as rb
    np, (bd, nd, _fd, _pd) = _np(), _body_cached(True, like)
    root = ((0.0, 0.0, 0.0), np.eye(3))
    ang = {'t_pose': 1.0} if abs(roll - rb.T_POSE_ROLL) < 1e-6 and not turn and not flex else {
        'arms_out': roll, 'left_shoulder': turn, 'right_shoulder': turn, 'left_elbow': flex, 'right_elbow': flex}
    fd = bd._frames(ang, root)
    (pd_, norm_d), ty = bd.pose(ang, root=root, _frames=fd), float(fd[nd.index('torso')][1][1])
    pts = np.vstack([t for t, _c in obs_tris]) if obs_tris else np.zeros((0, 3))
    obs = pts[np.unique(np.floor(pts / 0.005).astype(np.int64), axis=0, return_index=True)[1]] if len(pts) else pts
    tout, mout = [], []
    for name in (CLOTH_LIKE if like else CLOTH_SYM):
        if name not in nd:
            continue
        k = nd.index(name)
        (lo, hi), tris = bd.spans[k], bd.parts[k][5][1].copy()
        if tris is None or not len(tris):
            continue
        a, b, fs = _garment_bone(name, fd, nd)
        v = _drape_polar(_slim_bust(pd_[lo:hi], ty) if (like and name == 'like_torso') else pd_[lo:hi], obs, a, b, fall_at=fs)
        mats = bd.materials[lo:hi]
        if raw:
            tris, vn, vm = _cloth_normals(v, tris, norm_d[lo:hi], mats)
            mout.append((v, tris, bd.uv[lo:hi], vm, vn))
        else:
            tout += [(v[t], lw._rgb(mats[t[0]])) for t in tris]
    return mout if raw else tout


def apply_livery(tris):
    from tools.render import elbow_mount as em
    s, b = (72, 76, 86), (44, 46, 54)
    m = {em.SERVO: (255, 48, 200), em.GEAR: (196, 36, 152), (154, 154, 154): s,
         (150, 154, 164): s, (226, 222, 206): s, (206, 208, 212): (118, 124, 134), (255, 217, 102): b}
    return [(t, m.get(c, b if c[0] + 40 < c[1] > c[2] + 40 else s if c[0] > 220 and c[1] > 190 and c[2] < 130 else c))
            for t, c in tris]


def _smooth_normals(corners, tris, crease_deg=35.0):
    np = _np()
    v = corners.reshape(-1, 3)
    fn = np.cross(v[tris[:, 1]] - v[tris[:, 0]], v[tris[:, 2]] - v[tris[:, 0]])
    fu, fc, ft = _unit(fn), v[tris].reshape(-1, 3), np.arange(3 * len(tris)).reshape(-1, 3)
    if crease_deg >= 179.0:
        return fc, ft, _unit(_accum(len(v), tris, fn))[tris].reshape(-1, 3)
    cv = tris.ravel()
    order = np.argsort(cv, kind='mergesort')
    counts = np.bincount(cv[order], minlength=len(v))
    starts, st = np.r_[0, np.cumsum(counts[:-1])], order // 3
    cos_t, v_n = math.cos(math.radians(crease_deg)), np.repeat(fu, 3, 0).copy()
    for d in np.unique(counts[counts > 1]):
        idx = starts[counts == d, None] + np.arange(int(d))
        ug, fg = fu[st[idx]], fn[st[idx]]
        acc = np.einsum('mij,mjk->mik', (np.einsum('mik,mjk->mij', ug, ug) >= cos_t).astype(float), fg)
        v_n[order[idx]] = np.where((nl := np.linalg.norm(acc, axis=-1, keepdims=True)) > 1e-12, acc / np.maximum(nl, 1e-12), ug)
    return fc, ft, v_n


def lit_mesh(roll=0.0, dressed=False, turn=0.0, flex=0.0, nod=0.0, head_yaw=0.0, like=None):
    from coaxial.graphics import lit
    from tools.render import robot_built as rb
    np = _np()
    u32, whole = np.uint32, rb.whole(roll=roll, dressed=dressed, turn=turn, flex=flex, nod=nod, head_yaw=head_yaw, like=like)
    pl, nl, ul, ml, il, base = [], [], [], [], [], [0]

    def _push(c, t, u, m, n):
        pl.append(c), nl.append(n), ul.append(u), ml.append(m), il.append(t + base[0])
        base[0] += len(c)

    for mesh in symbolic_head(nod=nod, head_yaw=head_yaw, like=like, raw=True):
        _push(*mesh)
    if dressed:
        wb = rb.whole(roll=roll, dressed=False, turn=turn, flex=flex, nod=nod, head_yaw=head_yaw, like=like)
        obs = [t for k in ('pectoral', 'left leg', 'right leg', 'left arm', 'right arm', 'trunk', 'trunk guards') for t in wb.get(k, ())]
        for mesh in draped_clothes(roll=roll, turn=turn, flex=flex, obs_tris=obs, like=like, raw=True):
            _push(*mesh)
    for key, part_tris in whole.items():
        if not part_tris or key in ('head', 'garments'):
            continue
        raw = np.array([t for t, _c in part_tris], float)
        idx = np.arange(3 * len(raw)).reshape(-1, 3)
        cu, iu = np.unique(np.round(raw.reshape(-1, 3), 6), axis=0, return_inverse=True)
        _fc, _ft, vn = _smooth_normals(cu, iu.reshape(-1, 3), 180.0 if key in ('hands', 'shoes', 'neck skin') else 38.0)
        ca = np.asarray([c for _t, c in part_tris], u32)
        tm = (u32(lit.PAINTED) | (u32(lit.HARD) if key not in ('neck skin', 'seat') else u32(0))
              | np.where((ca == (64, 66, 68)).all(1), u32(lit.TWILL), u32(0)) | (ca[:, 0] << 16) | (ca[:, 1] << 8) | ca[:, 2])
        _push(raw.reshape(-1, 3), idx, np.zeros((3 * len(raw), 2), 'f4'), np.repeat(tm, 3), vn)
    return (np.vstack(pl).astype('f4'), np.vstack(nl).astype('f4'), np.vstack(ul).astype('f4'),
            np.concatenate(ml).astype(u32), np.vstack(il).astype(u32))


def checked():
    """Verify neck manifold, neck clearance, seat nets, sleeve continuity, and builds."""
    from coaxial.graphics import gynoid, likeness
    from tools.render import clips, robot_built as rb, solids
    skin0 = neck_skin(0.0, 0.0, likeness.FOUND)
    pr_n = solids.printable([1e3 * t for t, _c in skin0])
    wb = rb.whole(0.0, False, 0.0, 0.0, nod=0.0, head_yaw=0.0, like=likeness.FOUND)
    nc = sum(clips.crossed(skin0 if nd == 0.0 else neck_skin(nd, 0.0, likeness.FOUND), wb['neck']) for nd in (-45.0, 0.0, 60.0))
    sc = clips.crossed(wb['seat'], wb['left leg'] + wb['right leg'] + wb['trunk'])
    wsd = rb.whole(0.0, True, 0.0, 0.0, like=None)
    ns = {p[0]: len(p[5][1]) for p in gynoid.body(False, None, likeness.FOUND, True).parts}
    gap_mm = 0.0
    if likeness.FOUND:
        wld = rb.whole(0.0, True, 0.0, 0.0, like=likeness.FOUND)
        ay = max(float(t[:, 1].max()) for t, _c in wld['left arm'])
        sy = min(float(t[:, 1].min()) for t, _c in wld['garments'] if float(t[:, 0].mean()) > 0.08 and float(t[:, 1].mean()) > 0.30)
        gap_mm = round(1e3 * max(0.0, sy - ay), 2)
    ok = (not int(pr_n['open']) and not int(nc) and not int(sc) and len(wb['seat']) >= 1000
          and gap_mm <= 0.0 and ns.get('skin_neck', 0) > 0 and ns.get('drive_neck', -1) == 0
          and 'shoes' in wb and {'garments', 'neck skin', 'shoes'} <= set(wsd))
    return {'ok': ok, 'neck_open': int(pr_n['open']), 'neck_cross': int(nc), 'seat_cross': int(sc),
            'seat_tris': len(wb['seat']), 'skin': ns.get('skin_neck', 0), 'drive': ns.get('drive_neck', -1),
            'bare': sorted(wb.keys()), 'sym': sorted(wsd.keys()), 'gap_mm': gap_mm}
