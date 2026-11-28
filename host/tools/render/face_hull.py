"""Full-360 multi-view raytraced hull, byte-delta Lorenzo tables, 6x6 stereo grids and cameras."""
import base64
import json
import math
import os
import pathlib
import re
import struct
import sys
import zlib
from typing import Any

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import face_honeycomb as fh, face_ray as fr  # noqa: E402

#: Per-camera native-resolution 6-DoF calibration `(dfov, dtx, dty, dtz, dcx_px, dcy_px)`.
CAM_DELTAS: dict[str, tuple[float, ...]] = {
    'front': (0.0000, 0.0000, 0.0000, 0.0000, 3.5000, 1.5000),
    'profile90': (-0.1010, 4.0880, 0.6600, 0.0000, -10.5450, -0.5950),
    'threeQuarter': (0.5171, -3.4215, -0.6350, -0.4725, -1.8850, 2.1550),
    'rear': (0.8040, 0.7720, 1.3050, 0.0000, 1.8050, -6.4050),
}

#: Byte-delta 2D-Lorenzo compressed base64 tables (`head_dr`, `neck_xz`, `panel_grids`, `front_outer_x`).
HULL_PACKED = (
    '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'
)

HEAD_NY, HEAD_NPHI = 36, 32
NECK_NY, NECK_NPHI = 40, 32
GRID_N = 14
NECK_Y_MIN, NECK_Y_MAX = -30.0, 91.5

DEFAULT_OUTER_X: tuple[tuple[float, ...], ...] = (
    (160.0, 290.0, 282.0, 290.0, -8.0), (340.0, 348.0, 280.0, 348.0, 18.0),
    (560.0, 336.0, 285.0, 336.0, 2.0), (740.0, 254.0, 250.0, 254.0, 20.0), (900.0, 126.0, 126.0, 126.0, 24.0),
)

PANEL_ORDER = (
    'forehead-diamond', 'forehead-wings', 'face-brows', 'face-inner-cheeks', 'face-outer-cheeks',
    'face-nose', 'face-oled-mouth', 'jaw-chin-cap', 'temple-hex-bezels', 'jaw-front-hex', 'jaw-rear-hex',
)
SIDE_PANELS = frozenset(('temple-hex-bezels', 'jaw-front-hex', 'jaw-rear-hex'))
PANEL_BOUNDS: dict[str, tuple[float, float, float, float]] = {
    'forehead-diamond': (0.0, 22.0, 160.0, 208.0),
    'forehead-wings': (0.0, 78.0, 170.0, 222.0),
    'face-brows': (0.0, 78.0, 142.0, 196.0),
    'face-inner-cheeks': (0.0, 46.0, 96.0, 164.0),
    'face-outer-cheeks': (24.0, 72.0, 98.0, 172.0),
    'face-nose': (0.0, 18.0, 96.0, 168.0),
    'face-oled-mouth': (0.0, 44.0, 64.0, 108.0),
    'jaw-chin-cap': (0.0, 18.0, 50.0, 72.0),
    'temple-hex-bezels': (138.0, 212.0, -82.0, 36.0),
    'jaw-front-hex': (54.0, 112.0, 2.0, 50.0),
    'jaw-rear-hex': (70.0, 122.0, -12.0, 32.0),
}

SIDE_MIDLINE_YZ = (
    (60.3, 50.8), (65.6, 54.5), (70.9, 56.3), (76.2, 57.4), (81.5, 58.4), (86.8, 60.8),
    (92.1, 63.4), (97.4, 65.2), (102.6, 69.2), (107.9, 76.1), (113.2, 81.1), (118.5, 77.6),
    (123.8, 74.4), (129.0, 71.5), (134.3, 70.2), (139.6, 70.2), (144.9, 70.2), (150.2, 72.0),
    (155.4, 72.8), (160.7, 72.8), (166.0, 72.8), (171.2, 71.5), (176.5, 69.9), (181.8, 68.5),
    (187.0, 65.6), (192.3, 63.0), (197.6, 59.3), (202.8, 54.3), (208.1, 49.5),
)


def eval_side_midline_z(y_mm):
    """Interpolate C1 Catmull-Rom sagittal midline `Z(0, y_mm)` from `robot 1_side.png`."""
    pts, fy = SIDE_MIDLINE_YZ, float(y_mm)
    n = len(pts)
    if fy <= pts[0][0]:
        return pts[0][1] + 0.62 * (fy - pts[0][0])
    if fy >= pts[-1][0]:
        return pts[-1][1] - 0.90 * (fy - pts[-1][0])
    lo, hi = 0, n - 1
    while hi - lo > 1:
        mid = (lo + hi) >> 1
        lo, hi = (mid, hi) if pts[mid][0] <= fy else (lo, mid)
    t = (fy - pts[lo][0]) / max(1e-6, pts[hi][0] - pts[lo][0])
    return _cr4(pts[max(0, lo - 1)][1], pts[lo][1], pts[hi][1], pts[min(n - 1, hi + 1)][1], t)


_BAKED_CACHE: list[Any] = [None, None]
_HULL_CACHE: list[Any] = [None, None]


def _lorenzo_encode_2d(m):
    nr, nc = len(m), len(m[0]) if m else 0
    return [max(-32767, min(32767, m[r][c] - ((m[r][c - 1] if c else 0) + (m[r - 1][c] if r else 0) - (m[r - 1][c - 1] if r and c else 0))))
            for r in range(nr) for c in range(nc)]


def _lorenzo_decode_2d(res, nr, nc):
    mat, idx = [[0] * nc for _ in range(nr)], 0
    for r in range(nr):
        for c in range(nc):
            mat[r][c] = int(res[idx]) + (mat[r][c - 1] if c else 0) + (mat[r - 1][c] if r else 0) - (mat[r - 1][c - 1] if r and c else 0)
            idx += 1
    return mat


def _norm_fo_row(p):
    return (round(float(p[0]), 2), round(float(p[1]), 2), round(float(p[2]), 2),
            round(float(p[3]), 2), round(float(p[4] if len(p) > 4 else 14.0), 2))


def pack_tables_hex(hdr, nxz, pg, fo=DEFAULT_OUTER_X):
    """Byte-delta-pack `360`-deg `(head_dr, neck_xz, panel_grids, front_outer_x)` compactly."""
    hny, hnp = len(hdr), len(hdr[0]) if hdr else 0
    nny, nnp = len(nxz), len(nxz[0]) if nxz else 0
    gn, nfo = len(pg[PANEL_ORDER[0]]), len(fo)
    q5 = lambda x: max(-30000, min(30000, int(round(float(x) * 4.0))))
    q10 = lambda x: max(-30000, min(30000, int(round(float(x) * 10.0))))
    ints = (_lorenzo_encode_2d([[q5(x) for x in r] for r in hdr])
            + _lorenzo_encode_2d([[q5(p[0]) for p in r] for r in nxz])
            + _lorenzo_encode_2d([[q5(p[1]) for p in r] for r in nxz])
            + _lorenzo_encode_2d([[q10(v) for v in _norm_fo_row(r)] for r in fo]))
    for k in PANEL_ORDER:
        ints += _lorenzo_encode_2d([[q10(x) for x in r] for r in pg[k]])
    buf = bytearray(struct.pack('<6H', hny, hnp, nny, nnp, gn, nfo))
    for v in ints:
        if -126 <= v <= 126:
            buf.append(v & 0xFF)
        else:
            buf += struct.pack('<bh', -128, v)
    return base64.b64encode(zlib.compress(bytes(buf), level=9)).decode('ascii')


def _unpack_hex(hx: str) -> dict[str, Any]:
    zero_hdr = tuple((0.0,) * HEAD_NPHI for _ in range(HEAD_NY))
    zero_pg = {k: tuple((0.0,) * GRID_N for _ in range(GRID_N)) for k in PANEL_ORDER}
    if not hx:
        return {'head_dr': zero_hdr, 'neck_xz': (), 'panel_grids': zero_pg, 'front_outer_x': DEFAULT_OUTER_X}
    raw = zlib.decompress(base64.b64decode(hx))
    hny, hnp, nny, nnp, gn, nfo = struct.unpack_from('<6H', raw, 0)
    pos, ints, tot = 12, [], hny * hnp + 2 * nny * nnp + nfo * 5 + len(PANEL_ORDER) * gn * gn
    while len(ints) < tot and pos < len(raw):
        b0 = struct.unpack_from('<b', raw, pos)[0]
        ints.append(int(struct.unpack_from('<h', raw, pos + 1)[0]) if b0 == -128 else int(b0))
        pos += 3 if b0 == -128 else 1
    idx = hny * hnp
    hdr = tuple(tuple(v * 0.25 for v in r) for r in _lorenzo_decode_2d(ints[:idx], hny, hnp))
    nxz: tuple[tuple[tuple[float, float], ...], ...] = ()
    if nny > 0 and nnp > 0:
        nxm, nzm = _lorenzo_decode_2d(ints[idx:idx + nny * nnp], nny, nnp), _lorenzo_decode_2d(ints[idx + nny * nnp:idx + 2 * nny * nnp], nny, nnp)
        idx += 2 * nny * nnp
        nxz = tuple(tuple((nxm[r][c] * 0.25, nzm[r][c] * 0.25) for c in range(nnp)) for r in range(nny))
    fo = tuple(tuple(v * 0.10 for v in r) for r in _lorenzo_decode_2d(ints[idx:idx + nfo * 5], nfo, 5)) if nfo > 0 else DEFAULT_OUTER_X
    idx += nfo * 5
    pg = {k: (tuple(tuple(v * 0.10 for v in r) for r in _lorenzo_decode_2d(ints[idx + i * gn * gn:idx + (i + 1) * gn * gn], gn, gn))
              if idx + (i + 1) * gn * gn <= len(ints) else tuple((0.0,) * gn for _ in range(gn)))
          for i, k in enumerate(PANEL_ORDER)}
    return {'head_dr': hdr, 'neck_xz': nxz, 'panel_grids': pg, 'front_outer_x': fo}


def dumps_hull(data):
    """Serialize `data` dict into a compact JSON string safe for Win32 `os.environ['FACE_HULL_JSON']`."""
    cur = hull_state()
    cd = {k: [round(float(x), 4) for x in data.get('cam_deltas', cur['cam_deltas'])[k]] for k in CAM_DELTAS}
    if any(k in data for k in ('head_dr', 'neck_xz', 'panel_grids', 'front_outer_x')):
        hx = pack_tables_hex(data.get('head_dr', cur['head_dr']), data.get('neck_xz', cur['neck_xz']),
                             data.get('panel_grids', cur['panel_grids']), data.get('front_outer_x', cur['front_outer_x']))
        return json.dumps({'cam_deltas': cd, 'packed': hx}, separators=(',', ':'))
    return json.dumps({'cam_deltas': cd}, separators=(',', ':'))


def hull_state() -> dict[str, Any]:
    """Return active hull state dict merged with `FACE_HULL_JSON` when set."""
    if _BAKED_CACHE[0] != HULL_PACKED or _BAKED_CACHE[1] is None:
        _BAKED_CACHE[0], _BAKED_CACHE[1] = HULL_PACKED, _unpack_hex(HULL_PACKED)
    baked = _BAKED_CACHE[1]
    raw = os.environ.get('FACE_HULL_JSON')
    if not raw:
        return {'cam_deltas': CAM_DELTAS, **baked}
    if _HULL_CACHE[0] == raw and _HULL_CACHE[1] is not None:
        return _HULL_CACHE[1]
    parsed: dict[str, Any] = json.loads(raw)
    tbl = _unpack_hex(parsed['packed']) if 'packed' in parsed else baked
    fo = tuple(_norm_fo_row(p) for p in parsed['front_outer_x']) if 'front_outer_x' in parsed else tbl['front_outer_x']
    merged: dict[str, Any] = {
        'cam_deltas': {k: tuple(float(x) for x in parsed.get('cam_deltas', {}).get(k, v)) for k, v in CAM_DELTAS.items()},
        'front_outer_x': fo,
        'head_dr': tuple(tuple(float(x) for x in r) for r in parsed['head_dr']) if 'head_dr' in parsed else tbl['head_dr'],
        'neck_xz': (tuple(tuple((float(p[0]), float(p[1])) for p in r) for r in parsed['neck_xz'])
                    if 'neck_xz' in parsed else tbl['neck_xz']),
        'panel_grids': {k: (tuple(tuple(float(x) for x in r) for r in parsed['panel_grids'][k])
                            if k in parsed.get('panel_grids', {}) else v) for k, v in tbl['panel_grids'].items()},
    }
    _HULL_CACHE[0], _HULL_CACHE[1] = raw, merged
    return merged


def hull_cache_sig():
    return (os.environ.get('FACE_HULL_JSON') or '', HULL_PACKED, tuple(CAM_DELTAS.values()))


def cam_delta_for(preset_id):
    return hull_state()['cam_deltas'].get(preset_id, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))


def _cr4(p0, p1, p2, p3, t):
    return p1 + 0.5 * t * (p2 - p0 + t * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3 + t * (3.0 * (p1 - p2) + p3 - p0)))


def eval_grid_2d(grid, u_norm, v_norm, periodic_u=False):
    nr = len(grid)
    if not nr:
        return 0.0
    nc, fv = len(grid[0]), max(0.0, min(1.0, float(v_norm))) * (nr - 1)
    iv = min(nr - 2, int(fv))
    tv = fv - iv
    if periodic_u:
        fu = (float(u_norm) % 1.0) * nc
        iu, tu = int(fu) % nc, fu - int(fu)
        cu = ((iu - 1) % nc, iu, (iu + 1) % nc, (iu + 2) % nc)
    else:
        fu = max(0.0, min(1.0, float(u_norm))) * (nc - 1)
        iu = min(nc - 2, int(fu))
        tu = fu - iu
        cu = (max(0, iu - 1), iu, iu + 1, min(nc - 1, iu + 2))
    rv = [_cr4(grid[r][cu[0]], grid[r][cu[1]], grid[r][cu[2]], grid[r][cu[3]], tu)
          for r in (max(0, iv - 1), iv, iv + 1, min(nr - 1, iv + 2))]
    return _cr4(rv[0], rv[1], rv[2], rv[3], tv)


def panel_offset(pid, u_val, v_val):
    grid = hull_state()['panel_grids'].get(pid)
    if not grid:
        return 0.0
    u0, u1, v0, v1 = PANEL_BOUNDS[pid]
    uq = float(u_val) if pid in SIDE_PANELS else abs(float(u_val))
    return eval_grid_2d(grid, (uq - u0) / max(1e-6, u1 - u0), (float(v_val) - v0) / max(1e-6, v1 - v0))


_RAY_F_CACHE: list[Any] = [None, None]
_PROF_P_CACHE: list[Any] = [None, None]


def inside_profile_poly(y_mm, z_mm, loop_pts):
    cd = cam_delta_for('profile90')
    if _PROF_P_CACHE[0] != (loop_pts, cd) or _PROF_P_CACHE[1] is None:
        arr = fh.np_mod().asarray(loop_pts, float)
        poly = (arr.mean(0) + 1.03 * (arr - arr.mean(0))).tolist()
        (ox, oy, oz), _t, fwd, right, up, _d, tan_h, _a, w, h, dcx, dcy = fr._cam_basis(fr.CANONICAL_CAMERAS['profile90'], cd, 'profile90')
        _PROF_P_CACHE[0], _PROF_P_CACHE[1] = (loop_pts, cd), (poly, ox, oy, oz, fwd, right, up, 0.5 * w - 0.5 + dcx, 0.5 * h - 0.5 + dcy, 0.5 * h / tan_h)
    poly, ox, oy, oz, fwd, right, up, cx_p, cy_p, sc = _PROF_P_CACHE[1]
    vx, vy, vz = 62.0 - ox, float(y_mm) - oy, float(z_mm) - oz
    zc = max(1e-6, vx * fwd[0] + vy * fwd[1] + vz * fwd[2])
    x, y = cx_p + sc * (vx * right[0] + vy * right[1] + vz * right[2]) / zc, cy_p - sc * (vx * up[0] + vy * up[1] + vz * up[2]) / zc
    inside, n = False, len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        if ((y0 > y) != (y1 > y)) and (x < (x1 - x0) * (y - y0) / ((y1 - y0) + 1e-18) + x0):
            inside = not inside
    return inside


def ray_front_x_bounds(y_mm, z_mm):
    """Fast scalar perspective-exact `(x_sil_mm, x_t_in_mm, x_t_out_mm, z_ant_mm)` at `(y_mm, z_mm)`."""
    hs = hull_state()
    cd, pts = hs['cam_deltas']['front'], hs['front_outer_x']
    if _RAY_F_CACHE[0] != cd or _RAY_F_CACHE[1] is None:
        (ox, oy, oz), _t, fwd, _r, up, _d, tan_h, _a, _w, h, _dcx, dcy = fr._cam_basis(fr.CANONICAL_CAMERAS['front'], cd, 'front')
        _RAY_F_CACHE[0], _RAY_F_CACHE[1] = cd, (ox, oy, oz, fwd[1], fwd[2], up[1], up[2], 0.5 * h - 0.5 + dcy, 0.5 * h / tan_h, tan_h / (0.5 * h))
    _ox, oy, oz, fy, fz, uy, uz, cy_p, sc, k_mm = _RAY_F_CACHE[1]
    vy, vz = float(y_mm) - oy, float(z_mm) - oz
    py = cy_p - sc * (vy * uy + vz * uz) / max(1e-6, vy * fy + vz * fz)
    if py <= pts[0][0]:
        hw_s, hw_i, hw_o, za = pts[0][1], pts[0][2], pts[0][3], pts[0][4]
    elif py >= pts[-1][0]:
        hw_s, hw_i, hw_o, za = pts[-1][1], pts[-1][2], pts[-1][3], pts[-1][4]
    else:
        lo, hi = 0, len(pts) - 1
        while hi - lo > 1:
            mid = (lo + hi) >> 1
            lo, hi = (mid, hi) if pts[mid][0] <= py else (lo, mid)
        p0, p1 = pts[lo], pts[hi]
        t = (py - p0[0]) / max(1e-6, p1[0] - p0[0])
        hw_s, hw_i, hw_o, za = (p0[k] + t * (p1[k] - p0[k]) for k in (1, 2, 3, 4))
    mm_px = max(100.0, oz - float(z_mm)) * k_mm
    return hw_s * mm_px, hw_i * mm_px, hw_o * mm_px, za


def front_jaw_max_x(y_mm, z_mm=22.0):
    return ray_front_x_bounds(y_mm, z_mm)[0]


def head_dr_mm(phi, y_loc):
    u = (math.atan2(math.sin(float(phi)), math.cos(float(phi))) + math.pi) / (2.0 * math.pi)
    return eval_grid_2d(hull_state()['head_dr'], u, (float(y_loc) + 62.0) / 152.5, periodic_u=True)


def default_neck_xz(yv, phi, bust_rx=1.020):
    sa, ca = math.sin(float(phi)), math.cos(float(phi))
    sgn, asa = (1.0 if sa >= 0.0 else -1.0), abs(sa)
    if yv >= 51.0:
        t = min(1.0, (yv - 51.0) / 45.0)
        return ((32.5 + 11.0 * t * asa) * sa, -40.5 - 2.0 * t + ((25.0 - 5.0 * t) * ca if ca >= 0.0 else (33.0 + 17.0 * t) * ca))
    t = max(0.0, (51.0 - yv) / 85.0)
    sd = 3.2 + 10.2 * math.sin(min(1.0, t * 1.25) * 0.5 * math.pi)
    z = ((ca ** 0.80) * (1.6 + 11.5 * (t ** 1.15))) if ca >= 0.0 else (-((-ca) ** 0.78) * (73.0 + 28.0 * (t ** 0.95)))
    if ca > 0.12:
        z += -math.exp(-((asa / 0.28) ** 2)) * sd + math.exp(-(((asa - 0.46) / 0.20) ** 2)) * (3.6 if 2.0 < yv < 28.0 else 1.0)
    if ca < -0.15:
        z += math.exp(-((asa / 0.26) ** 2)) * sd - math.exp(-(((asa - 0.44) / 0.20) ** 2)) * (3.2 if 3.0 < yv < 34.0 else 0.8)
    return (sgn * (asa ** 0.80) * (33.5 + 83.5 * (t ** 1.15)) * bust_rx, z)


def build_neck_and_bust_shells(bust_rx=1.020):
    np = fh.np_mod()
    nxz = hull_state()['neck_xz']
    hr = np.array([0.0, 0.065, 0.012], float)
    if nxz and len(nxz) >= 12:
        ny, nphi = len(nxz), len(nxz[0])
        ys_all = np.linspace(NECK_Y_MIN, NECK_Y_MAX, ny)
        def _npt(x0, y0, z0):
            yc = min(82.5 if z0 > -14.0 else (92.8 if z0 < -22.0 else 82.5 + 10.3 * (-14.0 - z0) / 8.0), y0)
            tk = 1.0 - 0.25 * max(0.0, min(1.0, (yc - 83.5) / 9.3)) if z0 < -14.0 else 1.0
            if abs(tk * x0) >= 9.5 and z0 > -4.5:
                yc = min(yc, 58.6)
            zc = min(z0, 16.5 - 0.60 * max(0.0, yc - 52.0)) if (yc > 52.0 and abs(x0) < 35.0) else (-41.0 + tk * (z0 + 41.0))
            if abs(tk * x0) < 14.5 and -8.0 < yc < 45.0:
                zc = min(zc, 0.2)
            return fr.canon_to_head((tk * x0, yc, zc)) + hr

        ring_all = np.array([[_npt(float(nxz[i][j % nphi][0]), float(ys_all[i]), float(nxz[i][j % nphi][1]))
                              for j in range(nphi + 1)] for i in range(ny)])
    else:
        nphi, ny = 48, 36
        ys_all = np.linspace(NECK_Y_MIN, NECK_Y_MAX, ny)
        phis = np.linspace(-math.pi, math.pi, nphi + 1)
        ring_all = np.array([[fr.canon_to_head((default_neck_xz(float(yv), float(a), bust_rx)[0],
                                                float(yv), default_neck_xz(float(yv), float(a), bust_rx)[1])) + hr
                              for a in phis] for yv in ys_all])
    split_y = max(4, min(len(ys_all) - 4, int(round((46.0 - NECK_Y_MIN) / (NECK_Y_MAX - NECK_Y_MIN) * (len(ys_all) - 1)))))
    q0, q1, q2, q3, q4 = 0, nphi // 4, nphi // 2, (3 * nphi) // 4, nphi
    gb, gs = ring_all[:split_y + 1], ring_all[split_y:]
    sp_fn = fh.shell_patch
    trap_r, clav_r = sp_fn(gb[:, q0:q1 + 1], 0.0022, 0.0004, fh.CRANIUM_RIM, (0.0, -0.035, -0.010), fh.FRAME_VOID), sp_fn(gb[:, q1:q2 + 1], 0.0022, 0.0004, fh.CRANIUM_RIM, (0.0, -0.035, -0.025), fh.FRAME_VOID)
    clav_l, trap_l = sp_fn(gb[:, q2:q3 + 1], 0.0022, 0.0004, fh.CRANIUM_RIM, (0.0, -0.035, -0.025), fh.FRAME_VOID), sp_fn(gb[:, q3:q4 + 1], 0.0022, 0.0004, fh.CRANIUM_RIM, (0.0, -0.035, -0.010), fh.FRAME_VOID)
    slv_r, slv_l = sp_fn(gs[:, q0:q2 + 1], 0.0018, 0.0003, fh.BAFFLE_DARK, (0.0, 0.048, -0.006), fh.FRAME_VOID), sp_fn(gs[:, q2:q4 + 1], 0.0018, 0.0003, fh.BAFFLE_DARK, (0.0, 0.048, -0.006), fh.FRAME_VOID)
    clav_l += fh.socket_bolt(fr.canon_to_head((26.5, -14.0, 12.2)) + hr, (0.18, 0.30, 0.94), r_out=0.0021, h_out=0.00045)
    clav_r += fh.socket_bolt(fr.canon_to_head((-26.5, -14.0, 12.2)) + hr, (-0.18, 0.30, 0.94), r_out=0.0021, h_out=0.00045)
    return slv_l, slv_r, clav_l, clav_r, trap_l, trap_r


def bake_hull_into_source(data):
    """Compress and write converged `cam_deltas` and `HULL_PACKED` into `face_hull.py`."""
    global CAM_DELTAS, HULL_PACKED
    cur = hull_state()
    cd = {k: tuple(round(float(x), 4) for x in data.get('cam_deltas', cur['cam_deltas'])[k]) for k in CAM_DELTAS}
    packed = pack_tables_hex(data.get('head_dr', cur['head_dr']), data.get('neck_xz', cur['neck_xz']),
                             data.get('panel_grids', cur['panel_grids']), data.get('front_outer_x', cur['front_outer_x']))
    chunks = [packed[i:i + 92] for i in range(0, len(packed), 92)]
    hp_txt = 'HULL_PACKED = (\n' + '\n'.join(f"    '{c}'" for c in chunks) + '\n)' if chunks else "HULL_PACKED = ''"
    cd_lines = ['CAM_DELTAS: dict[str, tuple[float, ...]] = {']
    for k, v in cd.items():
        cd_lines.append(f"    '{k}': ({', '.join(f'{x:.4f}' for x in v)}),")
    cd_lines.append('}')
    p = pathlib.Path(__file__)
    src = p.read_text(encoding='utf-8')
    src = re.sub(r'CAM_DELTAS:\s*dict\[[^\n]+\n(?:\s+[^\n]+\n)*\}', '\n'.join(cd_lines), src, count=1)
    src = re.sub(r"HULL_PACKED\s*=\s*(?:''|\(\n(?:\s+'[^']*'\n)+\))", hp_txt, src, count=1)
    p.write_text(src, encoding='utf-8', newline='\n')
    CAM_DELTAS, HULL_PACKED = cd, packed
    _BAKED_CACHE[0], _BAKED_CACHE[1] = None, None
    _HULL_CACHE[0], _HULL_CACHE[1] = None, None
