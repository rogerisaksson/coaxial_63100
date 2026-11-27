"""SIMD C11 board 3D shadow, shading, glow/halftone/rim, silhouette, and ground (`raster_board.c`)."""
import ctypes
import functools
import threading

import numpy as np

from tools.cores import raster

_BOARD_TLS = threading.local()
_ART_CACHE: dict = {}
_ART_CAP = 16
_PACKED_RUNS: dict = {}


@functools.lru_cache(maxsize=1)
def _lib():
    """Bind `raster_board.c` symbols on the shared `raster` library."""
    lib = raster.ensure()
    if lib is None:
        return None
    i32, f64, vp = ctypes.c_int, ctypes.c_double, ctypes.c_void_p
    lib.rs_shadow_cast.argtypes = [i32, vp, i32, vp, vp, vp, vp, vp, i32, f64, vp, vp, vp, vp]
    lib.rs_shadow_cast.restype = None
    lib.rs_raster_folded.argtypes = [
        i32, vp, i32, vp, vp, vp, f64, f64, f64, f64, f64, f64, f64, f64, f64,
        i32, i32, vp, vp, vp, vp, vp, vp, vp, vp, vp, vp, vp,
    ]
    lib.rs_raster_folded.restype = None
    lib.rs_shade.argtypes = [
        i32, i32, vp, vp, vp, vp, vp, vp, vp, i32, i32, vp, i32, f64, vp, vp, vp, vp, vp,
    ]
    lib.rs_shade.restype = None
    lib.rs_paint_glow.argtypes = [
        i32, i32, vp, vp, vp, vp, vp, vp, i32, i32, vp, vp, vp, vp, vp, i32, vp, vp, vp,
    ]
    lib.rs_paint_glow.restype = None
    lib.rs_paint_dots_rim.argtypes = [
        i32, i32, vp, vp, vp, i32, f64, f64, vp, vp, vp, i32, vp, vp,
    ]
    lib.rs_paint_dots_rim.restype = None
    lib.rs_edge.argtypes = [i32, i32, vp, vp, i32, vp, vp, i32, vp, vp, vp, vp]
    lib.rs_edge.restype = None
    lib.rs_ground_shaded.argtypes = [
        i32, vp, vp, vp, vp, vp, vp, i32, i32, i32, f64, f64,
        vp, i32, vp, vp, vp, vp, vp, vp, vp, vp, vp,
    ]
    lib.rs_ground_shaded.restype = i32
    lib.rs_ground_step.argtypes = [
        i32, f64, vp, i32, vp, vp, vp, f64, f64, f64,
        i32, vp, vp, vp, vp, vp, vp, vp, vp, vp,
    ]
    lib.rs_ground_step.restype = None
    return lib


def shadow_cast(solid, m, right, up, light, size: int, extent: float):
    """Orthographic `shadow.cast` depth buffer in C, or `None` without C."""
    lib = _lib()
    if lib is None:
        return None
    pos_c, idx_c, _nrm, la, lb, lc = raster._solid_arrays(solid)
    m9 = np.ascontiguousarray(m, dtype=np.float64).ravel()
    r3 = np.ascontiguousarray(right, dtype=np.float64).ravel()
    u3 = np.ascontiguousarray(up, dtype=np.float64).ravel()
    l3 = np.ascontiguousarray(light, dtype=np.float64).ravel()
    sbuf = np.empty(size * size, dtype=np.float64)
    lib.rs_shadow_cast(len(pos_c) // 3, pos_c.ctypes.data, len(idx_c) // 3, idx_c.ctypes.data,
                       m9.ctypes.data, r3.ctypes.data, u3.ctypes.data, l3.ctypes.data,
                       int(size), float(extent), la.ctypes.data, lb.ctypes.data, lc.ctypes.data,
                       sbuf.ctypes.data)
    return sbuf


def raster_folded(solid, m, fine_cam, width: int, height: int, beam, sun_min: float):
    """Fused fine triangle raster + 2x4 cell fold returning NumPy/bytes buffers, or `None`."""
    lib = _lib()
    if lib is None:
        return None
    pos_c, idx_c, nrm_c, sx, sy, ooz = raster._solid_arrays(solid)
    key = (width, height)
    fine_hit = getattr(_BOARD_TLS, 'fine', None)
    if fine_hit is None or fine_hit[0] != key:
        npix = (width * 2) * (height * 4)
        fine_hit = (
            key,
            np.empty(npix, dtype=np.float64),
            np.empty(npix, dtype=np.uint8),
            np.empty(npix, dtype=np.uint8),
        )
        _BOARD_TLS.fine = fine_hit
    fine_bufs = fine_hit[1:]
    ncells = width * height
    out_d = np.empty(ncells, dtype=np.float64)
    out_t = np.empty(ncells, dtype=np.uint8)
    out_s = np.empty(ncells, dtype=np.uint8)
    out_cov = np.empty(ncells, dtype=np.float64)
    out_r = np.empty(ncells, dtype=np.uint8)
    m9 = np.ascontiguousarray(m, dtype=np.float64).ravel()
    lx, ly, lz = beam if beam else (0.0, 0.0, 1.0)
    lib.rs_raster_folded(
        len(pos_c) // 3, pos_c.ctypes.data, len(idx_c) // 3, idx_c.ctypes.data,
        nrm_c.ctypes.data, m9.ctypes.data,
        float(fine_cam['distance']), float(fine_cam['scale']), float(fine_cam['cx']),
        float(fine_cam['cy']), float(fine_cam.get('aspect', 0.5)),
        float(lx), float(ly), float(lz), float(sun_min), width, height,
        sx.ctypes.data, sy.ctypes.data, ooz.ctypes.data,
        fine_bufs[0].ctypes.data, fine_bufs[1].ctypes.data, fine_bufs[2].ctypes.data,
        out_d.ctypes.data, out_t.ctypes.data, out_s.ctypes.data,
        out_cov.ctypes.data, out_r.ctypes.data,
    )
    return out_d, out_t, out_s, out_cov, out_r


def shade_arrays(depth_c, top_c, sun_c, cam, m, pivot: float, slope: float, floor_v: float,
                 art, shadow, shadow_step: float, bias: float, planes, tone_depth: float = 0.0):
    """Compute `(classes_bytes, levels_arr, bare_arr, seed_arr)` in C matching `engine.shade`."""
    lib = _lib()
    if lib is None:
        return None
    width, height = int(cam['width']), int(cam['height'])
    ncells = width * height
    m9 = np.ascontiguousarray(m, dtype=np.float64).ravel()
    cam5 = np.array([cam['distance'], cam['scale'], cam['cx'], cam['cy'], cam.get('reach', 1.0)],
                    dtype=np.float64)
    plane = float(planes[0]) if planes is not None else 0.0
    pars7 = np.array([pivot, slope, floor_v, shadow_step, bias, plane, tone_depth],
                     dtype=np.float64)
    art_ptr, art_w, art_h = 0, 0, 0
    if art and m9[8] >= 0.0:
        _rows, aw, ah, dense = art
        if aw > 0 and ah > 0:
            hit = _ART_CACHE.get(id(dense))
            if hit is None or hit[0] is not dense:
                while len(_ART_CACHE) >= _ART_CAP:
                    del _ART_CACHE[next(iter(_ART_CACHE))]
                hit = _ART_CACHE[id(dense)] = (dense, np.ascontiguousarray(dense, dtype=np.int32))
            art_ptr, art_w, art_h = hit[1].ctypes.data, int(aw), int(ah)
    s_ptr, s_n, s_ext, s_vecs_ptr = 0, 0, 1.0, 0
    s_vecs = None
    if shadow:
        sbuf, s_n, s_ext, s_right, s_up, s_beam = shadow
        sbuf_c = np.ascontiguousarray(sbuf, dtype=np.float64)
        s_vecs = np.array([*s_right, *s_up, *s_beam], dtype=np.float64)
        s_ptr, s_vecs_ptr = sbuf_c.ctypes.data, s_vecs.ctypes.data
    out_cls = np.empty(ncells, dtype=np.uint8)
    levels = np.zeros(ncells, dtype=np.float64)
    bare = np.zeros(ncells, dtype=np.float64)
    seed = np.zeros(ncells, dtype=np.float64)
    lib.rs_shade(
        width, height, depth_c.ctypes.data, top_c.ctypes.data, sun_c.ctypes.data,
        m9.ctypes.data, cam5.ctypes.data, pars7.ctypes.data,
        art_ptr, art_w, art_h, s_ptr, int(s_n), float(s_ext), s_vecs_ptr,
        out_cls.ctypes.data, levels.ctypes.data, bare.ctypes.data, seed.ctypes.data,
    )
    return bytearray(out_cls.tobytes()), levels, bare, seed


def paint_glow_dots_rim(width: int, height: int, cls_u8, reached_u8, buf_f, cov_f, levels_f,
                        bare_f, seed_f, colour: bool, rig, tables, expose_fn, face_dict, persist,
                        glyph, tone):
    """Run `_glow` + `_expose` + `_dots` + `_rim` in C, returning `heat` (or `None` in mono)."""
    lib = _lib()
    if lib is None:
        return False, None
    key, lamp, colf, rowf, distance, _scale = rig
    has_key = int(key is not None and lamp is not None and colf is not None
                  and rowf is not None and distance is not None)
    rig6 = np.array([*(key or (0.0, 0.0)), *(lamp or (0.0, 0.0, 0.0)), distance or 0.0],
                    dtype=np.float64)
    colf_c = np.ascontiguousarray(colf if has_key else [0.0], dtype=np.float64)
    rowf_c = np.ascontiguousarray(rowf if has_key else [0.0], dtype=np.float64)
    heat = np.zeros(width * height, dtype=np.float64) if colour else None
    rungs_c = np.ascontiguousarray(tables['rung'], dtype=np.int64)
    rgb_c = np.ascontiguousarray(tables['rgb'], dtype=np.int64)
    lib.rs_paint_glow(
        width, height, cls_u8.ctypes.data,
        0 if levels_f is None else levels_f.ctypes.data,
        0 if bare_f is None else bare_f.ctypes.data,
        0 if seed_f is None else seed_f.ctypes.data,
        cov_f.ctypes.data, buf_f.ctypes.data, int(bool(colour)), has_key,
        rig6.ctypes.data, colf_c.ctypes.data, rowf_c.ctypes.data,
        rungs_c.ctypes.data, rgb_c.ctypes.data, len(rgb_c),
        glyph.ctypes.data, tone.ctypes.data, 0 if heat is None else heat.ctypes.data,
    )
    lo, hi = expose_fn(heat, face_dict, persist) if colour else (0.0, 1.0)
    noise_c = np.ascontiguousarray(tables['noise'], dtype=np.int64)
    rim_c = np.ascontiguousarray(tables['rim'], dtype=np.int64)
    lib.rs_paint_dots_rim(
        width, height, cls_u8.ctypes.data, reached_u8.ctypes.data,
        0 if heat is None else heat.ctypes.data, int(bool(colour)), float(lo), float(hi),
        noise_c.ctypes.data, rim_c.ctypes.data, rgb_c.ctypes.data, len(rgb_c),
        glyph.ctypes.data, tone.ctypes.data,
    )
    return True, heat


def paint_edge(width: int, height: int, reached_u8, heat, colour: bool, edge_tbl, glow_rgb,
               glyph, tone) -> bool:
    """Run `strokes.edge` flood-fill and silhouette rim in C."""
    lib = _lib()
    if lib is None:
        return False
    ncells = width * height
    edge_hit = getattr(_BOARD_TLS, 'edge', None)
    if edge_hit is None or edge_hit[0] < ncells:
        edge_hit = (ncells, np.empty(ncells, dtype=np.int32), np.empty(ncells, dtype=np.int32))
        _BOARD_TLS.edge = edge_hit
    scratch = edge_hit[1:]
    et_c = np.ascontiguousarray(edge_tbl, dtype=np.int64)
    rgb_c = np.ascontiguousarray(glow_rgb, dtype=np.int64)
    heat_c = None if heat is None else np.ascontiguousarray(heat, dtype=np.float64)
    lib.rs_edge(
        width, height, reached_u8.ctypes.data, 0 if heat_c is None else heat_c.ctypes.data,
        int(bool(colour)), et_c.ctypes.data, rgb_c.ctypes.data, len(rgb_c),
        scratch[0].ctypes.data, scratch[1].ctypes.data, glyph.ctypes.data, tone.ctypes.data,
    )
    return True


def packed_runs(runs, key=None, cap: int = 192):
    """Pre-flatten `ground._runs` polylines for `ground_shaded`, cached by `key`."""
    if key is not None and key in _PACKED_RUNS:
        return _PACKED_RUNS[key]
    kept = [r for r in runs if len(r[1]) > 1]
    if not kept:
        return None
    labels: dict = {}
    ids = np.array([labels.setdefault(line, len(labels)) for line, _p, _f in kept], np.int32)
    fades = np.array([fade for _l, _p, fade in kept], np.float64)
    lengths = np.array([len(pts) for _l, pts, _f in kept], np.int32)
    p = np.array([pt for _l, pts, _f in kept for pt in pts], np.float64)
    start = np.ones(len(p), dtype=bool)
    start[np.cumsum(lengths) - 1] = False
    seg_s = np.flatnonzero(start).astype(np.int32)
    run_of = np.repeat(np.arange(len(kept), dtype=np.int32), lengths - 1)
    out = (p[:, 0], p[:, 1], p[:, 2], seg_s, ids[run_of], fades[run_of],
           len(labels), labels.get('horizon', -1))
    if key is not None:
        if len(_PACKED_RUNS) >= cap:
            _PACKED_RUNS.clear()
        _PACKED_RUNS[key] = out
    return out


def ground_runs_shaded(runs, key, cap: int, width: int, height: int, near_depth: float,
                       reach: float, roll=None):
    """Run `packed_runs` + optional `roll` + `ground_shaded` in C, or return `None`."""
    pk = packed_runs(runs, key, cap)
    if pk is None:
        return None
    fx, fy = roll(pk[0], pk[1]) if roll is not None else (pk[0], pk[1])
    return ground_shaded(fx, fy, pk[2], pk[3], pk[4], pk[5], pk[6], pk[7],
                         width, height, near_depth, reach)


def ground_step_shaded(static, r_list, width: int, height: int, step: int, cap: int, roll=None):
    """Build `ground._runs` + `packed_runs` in C (`rs_ground_step`) and shade, or return `None`."""
    lib = _lib()
    if lib is None:
        return None
    key = ('step', width, height, step)
    pk = _PACKED_RUNS.get(key)
    if pk is None:
        st_hit = _ART_CACHE.get(('fan', id(static)))
        if st_hit is None or st_hit[0] is not static:
            sky_y = np.array([pt[1] for pt in static['horizon']], dtype=np.float64)
            fan = static['fan']
            pts = np.array([pt for _k, run in fan for pt in run], dtype=np.float64).reshape(-1, 4)
            lens = [len(run) for _k, run in fan]
            starts = np.array([0] + list(np.cumsum(lens)), dtype=np.int32)
            lids_map: dict = {}
            lids = np.array([lids_map.setdefault(k, 1 + len(lids_map)) for k, _r in fan], np.int32)
            st_hit = _ART_CACHE[('fan', id(static))] = (static, sky_y, pts, starts, lids,
                                                        1 + len(lids_map))
        _, sky_y, fan_pts, fan_starts, fan_lids, base_lid = st_hit
        r6, r_lids = [], []
        hrow = static['hrow']
        for k, (yc, wc, x0, y0, _w0, x1, y1, _w1) in enumerate(r_list):
            if wc <= 0.0 or wc > 2.0 or yc < hrow:
                continue
            beyond = r_list[k + 1][0] if k + 1 < len(r_list) else hrow
            fade = min(1.0, max(0.0, (yc - beyond) / 1.5))
            if fade >= 0.1:
                r6.append((x0, y0, x1, y1, 1.0 / wc, fade))
                r_lids.append(base_lid + len(r_lids))
        n_labels = base_lid + len(r_lids)
        if n_labels > 63:
            return None
        rung6 = np.array(r6, dtype=np.float64) if r6 else np.zeros((0, 6), dtype=np.float64)
        rl_c = np.array(r_lids, dtype=np.int32)
        max_pts = (width + 1) + len(fan_pts) * 4 + len(r_lids) * (width + 2) + 32
        fx_b = np.empty(max_pts, dtype=np.float64)
        fy_b = np.empty(max_pts, dtype=np.float64)
        fd_b = np.empty(max_pts, dtype=np.float64)
        ss_b = np.empty(max_pts, dtype=np.int32)
        sl_b = np.empty(max_pts, dtype=np.int32)
        sf_b = np.empty(max_pts, dtype=np.float64)
        cnts = np.zeros(2, dtype=np.int32)
        south = float(static['south'])
        lib.rs_ground_step(
            int(width), float(static['far_depth']), sky_y.ctypes.data,
            len(fan_lids), fan_pts.ctypes.data, fan_starts.ctypes.data, fan_lids.ctypes.data,
            south, step / 96.0, south + 6.0 * 1.65,
            len(r_lids), rung6.ctypes.data, rl_c.ctypes.data,
            fx_b.ctypes.data, fy_b.ctypes.data, fd_b.ctypes.data,
            ss_b.ctypes.data, sl_b.ctypes.data, sf_b.ctypes.data, cnts.ctypes.data,
        )
        np_cnt, ns_cnt = int(cnts[0]), int(cnts[1])
        if ns_cnt == 0:
            return None
        pk = (fx_b[:np_cnt].copy(), fy_b[:np_cnt].copy(), fd_b[:np_cnt].copy(),
              ss_b[:ns_cnt].copy(), sl_b[:ns_cnt].copy(), sf_b[:ns_cnt].copy(), n_labels, 0)
        if len(_PACKED_RUNS) >= cap:
            _PACKED_RUNS.clear()
        _PACKED_RUNS[key] = pk
    near_d = float(static['near_depth'])
    fx, fy = roll(pk[0], pk[1]) if roll is not None else (pk[0], pk[1])
    return ground_shaded(fx, fy, pk[2], pk[3], pk[4], pk[5], pk[6], pk[7],
                         width, height, near_d, max(1e-9, float(static['far_depth']) - near_d))


def ground_shaded(fx, fy, fd, seg_s, seg_line, seg_fade, n_labels: int, horizon_id: int,
                  width: int, height: int, near_depth: float, reach: float):
    """Compute `(cells, masks, greys)` for `ground._laid` + `ground._shaded` in C, or `None`."""
    lib = _lib()
    if lib is None or n_labels > 64:
        return None
    ncells = width * height
    ts_cap = 2 * width + 4 * height + 16
    key = (ncells, ts_cap)
    gnd_hit = getattr(_BOARD_TLS, 'gnd', None)
    if gnd_hit is None or gnd_hit[0] != key:
        gnd_hit = (
            key,
            np.empty(ts_cap, dtype=np.float64),
            np.empty(ncells, dtype=np.int32),
            np.empty(ncells, dtype=np.uint64),
            np.empty(ncells, dtype=np.int64),
            np.empty(ncells, dtype=np.int64),
            np.empty(ncells, dtype=np.int64),
            np.empty(ncells, dtype=np.float64),
            np.empty(ncells, dtype=np.float64),
            np.empty(ncells, dtype=np.int32),
            np.empty(ncells, dtype=np.uint8),
        )
        _BOARD_TLS.gnd = gnd_hit
    sc = gnd_hit[1:]
    fx_c = np.ascontiguousarray(fx, dtype=np.float64)
    fy_c = np.ascontiguousarray(fy, dtype=np.float64)
    fd_c = np.ascontiguousarray(fd, dtype=np.float64)
    s_c = np.ascontiguousarray(seg_s, dtype=np.int32)
    l_c = np.ascontiguousarray(seg_line, dtype=np.int32)
    f_c = np.ascontiguousarray(seg_fade, dtype=np.float64)
    m = lib.rs_ground_shaded(
        len(s_c), fx_c.ctypes.data, fy_c.ctypes.data, fd_c.ctypes.data,
        s_c.ctypes.data, l_c.ctypes.data, f_c.ctypes.data,
        int(horizon_id), width, height, float(near_depth), float(reach),
        sc[0].ctypes.data, ts_cap, sc[1].ctypes.data, sc[2].ctypes.data,
        sc[3].ctypes.data, sc[4].ctypes.data, sc[5].ctypes.data,
        sc[6].ctypes.data, sc[7].ctypes.data, sc[8].ctypes.data, sc[9].ctypes.data,
    )
    return sc[3][:m].copy(), sc[4][:m].copy(), sc[5][:m].copy()
