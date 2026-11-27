"""SIMD C11 mesh transform, triangle/splat raster, drape, and Braille fold kernels (`raster.c`)."""
import ctypes
import functools
import os
import threading

import numpy as np

from tools.cores import build

SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'raster.c')
CELLS_SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'raster_cells.c')
STROKES_SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'raster_strokes.c')
BOARD_SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'raster_board.c')
DRAW_SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'raster_draw.c')
INST_SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'raster_inst.c')
FSAA_SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'raster_fsaa.c')
SOURCES = [SOURCE, CELLS_SOURCE, STROKES_SOURCE, BOARD_SOURCE, DRAW_SOURCE, INST_SOURCE, FSAA_SOURCE]
FLAGS = ('-O3', '-ffp-contract=off', '-fno-math-errno', '-fno-trapping-math',
         '-fvect-cost-model=dynamic') + build.native()
_SCRATCH: dict[int, tuple] = {}
_raster_tls = threading.local()
_SOLID_CACHE: dict[int, tuple] = {}
_SPANS: dict[int, tuple] = {}
_DRAPE_CACHE: dict[int, tuple] = {}
SCRATCH_SLOTS = 8


def current() -> bool:
    """Whether `raster.c` is already loaded or compiled on disk for this host."""
    return _lib.cache_info().currsize > 0 or build.is_built(SOURCES, [], 'raster', FLAGS)


def ensure():
    """Compile and load `raster.c` if a host C compiler is available."""
    return _lib()


@functools.lru_cache(maxsize=1)
def _lib():
    """Compile and load `raster.c`, or return `None` when no host C compiler is available."""
    if build.find_cc() is None:
        return None
    path, _warn = build.cached(SOURCES, [], 'raster', FLAGS)
    lib = ctypes.CDLL(path)
    i32, u32, f64, vp = ctypes.c_int, ctypes.c_uint32, ctypes.c_double, ctypes.c_void_p
    lib.rs_moved.argtypes = [i32, vp, vp, i32, vp, vp, vp, vp, vp]
    lib.rs_moved.restype = None
    lib.rs_project.argtypes = [i32, vp, vp, vp, f64, f64, f64, f64, f64, vp, vp, vp]
    lib.rs_project.restype = None
    lib.rs_splat.argtypes = [
        i32, vp, vp, vp, vp,
        f64, f64, f64, f64, f64,
        i32, i32, u32, f64,
        vp, vp, vp, vp, vp, vp, vp,
    ]
    lib.rs_splat.restype = None
    lib.rs_triangles.argtypes = [
        i32, vp, i32, vp, vp, vp,
        f64, f64, f64, f64, f64, f64, f64, f64, f64,
        i32, i32, i32,
        vp, vp, vp, vp, vp, vp,
    ]
    lib.rs_triangles.restype = None
    lib.rs_fold.argtypes = [vp, vp, vp, i32, i32, vp, vp, vp, vp, vp]
    lib.rs_fold.restype = None
    lib.rs_braille.argtypes = [vp, vp, i32, vp, vp, vp, vp, vp, vp, i32, i32, vp, vp, vp]
    lib.rs_braille.restype = None
    lib.rs_drape_row.argtypes = [
        i32, vp, vp, vp, vp, vp,
        i32, vp, vp, vp, vp, vp,
        i32, vp, vp, vp, vp,
        vp, vp, vp, vp, vp, vp, vp,
    ]
    lib.rs_drape_row.restype = None
    lib.rs_drape_finish.argtypes = [i32, vp, vp, vp, vp]
    lib.rs_drape_finish.restype = None
    i64 = ctypes.c_int64
    lib.rs_lines.argtypes = [vp, i32, i32, i32, vp, vp, vp, vp]
    lib.rs_lines.restype = None
    lib.rs_strokes_prep.argtypes = [
        i32, vp, vp, vp, f64, f64, f64, f64, f64, f64, i32, i32, vp, vp,
    ]
    lib.rs_strokes_prep.restype = None
    lib.rs_strokes_emit.argtypes = [i64, vp, i32, i32, vp, vp, vp, vp]
    lib.rs_strokes_emit.restype = None
    lib.rs_ground.argtypes = [
        i32, vp, vp, vp, f64, f64, f64, f64, f64, f64, i32, i32, vp,
    ]
    lib.rs_ground.restype = None
    lib.rs_pose_parts.argtypes = [i32, vp, vp, vp, vp, vp, vp, vp]
    lib.rs_pose_parts.restype = None
    return lib


def _scratch(ncells: int):
    hit = _SCRATCH.get(ncells)
    if hit is None:
        while len(_SCRATCH) >= SCRATCH_SLOTS:
            del _SCRATCH[next(iter(_SCRATCH))]
        hit = _SCRATCH[ncells] = (
            np.empty(ncells, dtype=np.float64),
            np.empty(ncells, dtype=np.float64),
            np.empty(ncells, dtype=np.int32),
            np.empty(ncells, dtype=np.int32),
            np.empty(ncells, dtype=np.int32),
        )
    return hit


def moved(points, normals, spans, frames):
    """Transform `points` and `normals` by `spans` and `frames` in C, or return `None` without C."""
    lib = _lib()
    if lib is None:
        return None
    pts = np.ascontiguousarray(points, dtype=np.float64)
    nrm = np.ascontiguousarray(normals, dtype=np.float64)
    out_p = np.empty_like(pts)
    out_n = np.empty_like(nrm)
    s_key = id(spans)
    s_hit = _SPANS.get(s_key)
    if s_hit is None or s_hit[0] is not spans:
        while len(_SPANS) >= SCRATCH_SLOTS:
            del _SPANS[next(iter(_SPANS))]
        s_arr = np.asarray(spans, dtype=np.int32).reshape(-1, 2)
        s_hit = _SPANS[s_key] = (spans, np.ascontiguousarray(s_arr, dtype=np.int32))
    sp_c = s_hit[1]
    nspans = min(len(sp_c), len(frames))
    if nspans <= 0 or len(pts) == 0:
        return out_p, out_n
    turns = np.zeros((nspans, 9), dtype=np.float64)
    spots = np.zeros((nspans, 3), dtype=np.float64)
    for k in range(nspans):
        f = frames[k]
        if f is not None and sp_c[k, 1] > sp_c[k, 0]:
            turns[k] = np.asarray(f[0], dtype=np.float64).ravel()
            spots[k] = f[1]
    lib.rs_moved(len(pts), pts.ctypes.data, nrm.ctypes.data, nspans, sp_c.ctypes.data,
                 turns.ctypes.data, spots.ctypes.data, out_p.ctypes.data, out_n.ctypes.data)
    return out_p, out_n


def project(points, m, cam, centre):
    """Project `points` in C matching `lit.project`, or return `None` when C is unavailable."""
    lib = _lib()
    if lib is None:
        return None
    pts = np.ascontiguousarray(points, dtype=np.float64)
    n = len(pts)
    sx = np.empty(n, dtype=np.float64)
    sy = np.empty(n, dtype=np.float64)
    w = np.empty(n, dtype=np.float64)
    if n > 0:
        m9 = np.ascontiguousarray(m, dtype=np.float64).ravel()
        ctr = np.ascontiguousarray(centre, dtype=np.float64).ravel()
        lib.rs_project(n, pts.ctypes.data, m9.ctypes.data, ctr.ctypes.data,
                       float(cam['distance']), float(cam['scale']), float(cam['cx']), float(cam['cy']),
                       float(cam.get('aspect', 0.5)), sx.ctypes.data, sy.ctypes.data, w.ctypes.data)
    return sx, sy, w


def splat_bins(positions, materials, m, cam, centre, under_bit: int, layer_m: float):
    """Compute `(depth, win)` 2x2 dot-splat buffers in C, or return `None` when C is unavailable."""
    lib = _lib()
    if lib is None:
        return None
    pos = np.ascontiguousarray(positions, dtype=np.float64)
    mats = np.ascontiguousarray(materials, dtype=np.uint32)
    m9 = np.ascontiguousarray(m, dtype=np.float64).ravel()
    ctr = np.ascontiguousarray(centre, dtype=np.float64).ravel()
    height, width = int(cam['height']), int(cam['width'])
    ncells = (height + 1) * (width + 1)
    best_sc, best_w, best_idx, best_ix, best_iy = _scratch(max(1, ncells))
    depth = np.empty((height, width), dtype=np.float64)
    win = np.empty((height, width), dtype=np.int32)
    lib.rs_splat(
        len(pos), pos.ctypes.data, mats.ctypes.data, m9.ctypes.data, ctr.ctypes.data,
        float(cam['distance']), float(cam['scale']), float(cam['cx']), float(cam['cy']),
        float(cam.get('aspect', 0.5)), width, height, int(under_bit), float(layer_m),
        best_sc.ctypes.data, best_w.ctypes.data, best_idx.ctypes.data,
        best_ix.ctypes.data, best_iy.ctypes.data, depth.ctypes.data, win.ctypes.data,
    )
    return depth, win


def _solid_arrays(solid):
    key = id(solid)
    hit = _SOLID_CACHE.get(key)
    if hit is None or hit[0] is not solid:
        while len(_SOLID_CACHE) >= SCRATCH_SLOTS * 2:
            del _SOLID_CACHE[next(iter(_SOLID_CACHE))]
        pos, idx, nrm = solid
        pos_c = np.ascontiguousarray(pos, dtype=np.float64).ravel()
        idx_c = np.ascontiguousarray(idx, dtype=np.int32).ravel()
        nrm_c = np.ascontiguousarray(nrm, dtype=np.float64).ravel()
        nverts = len(pos_c) // 3
        sx = np.empty(max(1, nverts), dtype=np.float64)
        sy = np.empty(max(1, nverts), dtype=np.float64)
        ooz = np.empty(max(1, nverts), dtype=np.float64)
        hit = _SOLID_CACHE[key] = (solid, pos_c, idx_c, nrm_c, sx, sy, ooz)
    return hit[1:]


def triangles(solid, m, cam, beam=None, sun_min=0.0, band=None):
    """Rasterize `solid` in C matching `engine.raster`, or return `None` when C is unavailable."""
    lib = _lib()
    if lib is None:
        return None
    pos_c, idx_c, nrm_c, sx, sy, ooz = _solid_arrays(solid)
    width, height = int(cam['width']), int(cam['height'])
    first, last = (int(band[0]), int(band[1])) if band else (0, height)
    rows = max(0, last - first)
    lx, ly, lz = beam if beam else (0.0, 0.0, 1.0)
    m9 = np.ascontiguousarray(m, dtype=np.float64).ravel()
    npix = width * rows
    depth = np.empty(npix, dtype=np.float64)
    top = np.empty(npix, dtype=np.uint8)
    sun = np.empty(npix, dtype=np.uint8)
    if npix > 0:
        lib.rs_triangles(
            len(pos_c) // 3, pos_c.ctypes.data, len(idx_c) // 3, idx_c.ctypes.data,
            nrm_c.ctypes.data, m9.ctypes.data,
            float(cam['distance']), float(cam['scale']), float(cam['cx']), float(cam['cy']),
            float(cam.get('aspect', 0.5)), float(lx), float(ly), float(lz), float(sun_min),
            width, first, last,
            sx.ctypes.data, sy.ctypes.data, ooz.ctypes.data,
            depth.ctypes.data, top.ctypes.data, sun.ctypes.data,
        )
    return depth.tolist(), bytearray(top.tobytes()), bytearray(sun.tobytes())


def fold(depth, top, sun, width: int, height: int):
    """Fold fine dot rasters down to cells in C matching `engine.fold`, or return `None`."""
    lib = _lib()
    if lib is None:
        return None
    d_c = np.ascontiguousarray(depth, dtype=np.float64)
    t_c = np.ascontiguousarray(top, dtype=np.uint8)
    s_c = np.ascontiguousarray(sun, dtype=np.uint8)
    ncells = max(0, int(width) * int(height))
    out_d = np.empty(ncells, dtype=np.float64)
    out_t = np.empty(ncells, dtype=np.uint8)
    out_s = np.empty(ncells, dtype=np.uint8)
    out_cov = np.empty(ncells, dtype=np.float64)
    out_r = np.empty(ncells, dtype=np.uint8)
    if ncells > 0:
        lib.rs_fold(d_c.ctypes.data, t_c.ctypes.data, s_c.ctypes.data, int(width), int(height),
                    out_d.ctypes.data, out_t.ctypes.data, out_s.ctypes.data,
                    out_cov.ctypes.data, out_r.ctypes.data)
    return (out_d.tolist(), bytearray(out_t.tobytes()), bytearray(out_s.tobytes()),
            out_cov.tolist(), bytearray(out_r.tobytes()))


def braille_cells(depth, rgb, floor_b, noise64, over, alone, leaders, floor_ink, leader_ink,
                  width: int, height: int):
    """Compute `(cells, hits, ink)` for `lit.braille` in C, or return `None` without C."""
    lib = _lib()
    if lib is None:
        return None
    d_c = np.ascontiguousarray(depth, dtype=np.float64)
    rgb_c = np.ascontiguousarray(rgb, dtype=np.uint8)
    fl_c = np.ascontiguousarray(floor_b, dtype=np.float64)
    n64_c = np.ascontiguousarray(noise64, dtype=np.float64)
    ov_c = None if over is None else np.ascontiguousarray(over, dtype=np.uint8)
    al_c = None if alone is None else np.ascontiguousarray(alone, dtype=np.uint8)
    ld_c = None if leaders is None else np.ascontiguousarray(leaders, dtype=np.uint8)
    inks6 = np.array([floor_ink[0], floor_ink[1], floor_ink[2],
                      leader_ink[0], leader_ink[1], leader_ink[2]], dtype=np.float64)
    cells = np.empty((height, width), dtype=np.uint8)
    hits = np.empty((height, width), dtype=np.uint8)
    ink = np.empty((height, width, 3), dtype=np.float64)
    lib.rs_braille(
        d_c.ctypes.data, rgb_c.ctypes.data, int(rgb_c.shape[-1]), fl_c.ctypes.data,
        n64_c.ctypes.data,
        0 if ov_c is None else ov_c.ctypes.data,
        0 if al_c is None else al_c.ctypes.data,
        0 if ld_c is None else ld_c.ctypes.data,
        inks6.ctypes.data, int(width), int(height),
        cells.ctypes.data, hits.ctypes.data, ink.ctypes.data,
    )
    return cells, hits, ink


def _drape_laid_arrays(laid):
    key = id(laid)
    hit = _DRAPE_CACHE.get(key)
    if hit is None or hit[0] is not laid:
        while len(_DRAPE_CACHE) >= 64:
            del _DRAPE_CACHE[next(iter(_DRAPE_CACHE))]
        full, there, facing, mid, share, at_pts, out_nrm, blend, about = laid
        f_idx = np.ascontiguousarray(full, dtype=np.int64)
        th_c = np.ascontiguousarray(there, dtype=np.float64)
        fa_c = np.ascontiguousarray(facing, dtype=np.float64)
        m_idx = np.ascontiguousarray(mid, dtype=np.int64)
        sh_c = np.ascontiguousarray(share, dtype=np.float64)
        at_c = np.ascontiguousarray(at_pts, dtype=np.float64)
        ou_c = np.ascontiguousarray(out_nrm, dtype=np.float64)
        ab_c = None if about is None else np.ascontiguousarray(about, dtype=np.float64)
        if blend is None:
            bl_c = None
        else:
            held = np.ascontiguousarray(blend[2], dtype=np.int64)
            sf = np.ascontiguousarray(np.searchsorted(held, f_idx), dtype=np.int64)
            sm = np.ascontiguousarray(np.searchsorted(held, m_idx), dtype=np.int64)
            wf = np.ascontiguousarray(blend[0], dtype=np.float64)
            wm = np.ascontiguousarray(blend[1], dtype=np.float64)
            bl_c = (held, sf, wf, sm, wm)
        hit = _DRAPE_CACHE[key] = (laid, f_idx, th_c, fa_c, m_idx, sh_c, at_c, ou_c, ab_c, bl_c)
    return hit[1:]


def drape_row(pos, norm, turn, spot, base, hinge, laid, rot_axes, rot_rads, sums=None):
    """Apply one `drape` garment row in C into `(pos, norm)` or `sums`, returning `True` if run."""
    lib = _lib()
    if lib is None:
        return False
    f_idx, th_c, fa_c, m_idx, sh_c, at_c, ou_c, ab_c, bl_c = _drape_laid_arrays(laid)
    t9 = np.ascontiguousarray(turn, dtype=np.float64).ravel()
    sp3 = np.ascontiguousarray(spot, dtype=np.float64).ravel()
    b9 = np.ascontiguousarray(base, dtype=np.float64).ravel()
    hi3 = np.ascontiguousarray(hinge, dtype=np.float64).ravel()
    ax_c = np.ascontiguousarray(rot_axes, dtype=np.int32)
    rd_c = np.ascontiguousarray(rot_rads, dtype=np.float64)
    lib.rs_drape_row(
        len(f_idx), f_idx.ctypes.data, th_c.ctypes.data, fa_c.ctypes.data, t9.ctypes.data,
        sp3.ctypes.data, len(m_idx), m_idx.ctypes.data, sh_c.ctypes.data, at_c.ctypes.data,
        ou_c.ctypes.data, 0 if ab_c is None else ab_c.ctypes.data,
        len(rd_c), ax_c.ctypes.data, rd_c.ctypes.data, b9.ctypes.data, hi3.ctypes.data,
        0 if bl_c is None else bl_c[1].ctypes.data,
        0 if bl_c is None else bl_c[2].ctypes.data,
        0 if bl_c is None else bl_c[3].ctypes.data,
        0 if bl_c is None else bl_c[4].ctypes.data,
        pos.ctypes.data, norm.ctypes.data, 0 if sums is None else sums.ctypes.data,
    )
    return True


def drape_finish(pos, norm, laid, sums):
    """Finish two-sided garment blend `sums` into `(pos, norm)` in C."""
    lib = _lib()
    if lib is None:
        return False
    bl_c = _drape_laid_arrays(laid)[8]
    if bl_c is None:
        return False
    held = bl_c[0]
    lib.rs_drape_finish(len(held), held.ctypes.data, sums.ctypes.data, pos.ctypes.data,
                        norm.ctypes.data)
    return True


def lines(dots, ax, ay, bx, by) -> bool:
    """Rasterize 2D segments into contiguous `bool` `dots` in C matching `callouts.lines`."""
    lib = _lib()
    if lib is None or not dots.flags.c_contiguous or dots.dtype != np.bool_:
        return False
    h, w = dots.shape
    ax_c = np.ascontiguousarray(ax, dtype=np.float64).ravel()
    ay_c = np.ascontiguousarray(ay, dtype=np.float64).ravel()
    bx_c = np.ascontiguousarray(bx, dtype=np.float64).ravel()
    by_c = np.ascontiguousarray(by, dtype=np.float64).ravel()
    n = min(len(ax_c), len(ay_c), len(bx_c), len(by_c))
    if n > 0 and h > 0 and w > 0:
        lib.rs_lines(dots.ctypes.data, int(h), int(w), int(n),
                     ax_c.ctypes.data, ay_c.ctypes.data, bx_c.ctypes.data, by_c.ctypes.data)
    return True


def strokes(ends, m, cam, centre, near_m: float):
    """Clip and step 3D segments `ends` `(2n, 3)` in C matching `scenery.strokes`."""
    lib = _lib()
    if lib is None:
        return None
    ends_c = np.ascontiguousarray(ends, dtype=np.float64).reshape(-1, 3)
    nseg = len(ends_c) // 2
    width, height = int(cam['width']), int(cam['height'])
    if nseg <= 0 or width <= 0 or height <= 0:
        return (np.empty(0, dtype=int), np.empty(0, dtype=int),
                np.empty(0, dtype=np.float64), np.empty(0, dtype=int))
    need = nseg * 10
    scratch = getattr(_raster_tls, 'strokes', None)
    if scratch is None or len(scratch) < need:
        scratch = np.empty(need, dtype=np.float64)
        _raster_tls.strokes = scratch
    m9 = np.ascontiguousarray(m, dtype=np.float64).ravel()
    c3 = np.ascontiguousarray(centre, dtype=np.float64).ravel()
    counts = np.zeros(2, dtype=np.int64)
    lib.rs_strokes_prep(
        nseg, ends_c.ctypes.data, m9.ctypes.data, c3.ctypes.data,
        float(cam['distance']), float(near_m), float(cam['scale']), float(cam['cx']),
        float(cam['cy']), float(cam.get('aspect', 0.5)), width, height,
        scratch.ctypes.data, counts.ctypes.data,
    )
    kept, total = int(counts[0]), int(counts[1])
    ys = np.empty(total, dtype=np.int64)
    xs = np.empty(total, dtype=np.int64)
    w_out = np.empty(total, dtype=np.float64)
    which = np.empty(total, dtype=np.int64)
    if kept > 0 and total > 0:
        lib.rs_strokes_emit(kept, scratch.ctypes.data, width, height,
                            ys.ctypes.data, xs.ctypes.data, w_out.ctypes.data, which.ctypes.data)
    return ys, xs, w_out, which


def ground(ends, m, cam, centre, near_m: float):
    """Fused `scenery.ground` `(height, width)` grid brightness in C."""
    lib = _lib()
    if lib is None:
        return None
    ends_c = np.ascontiguousarray(ends, dtype=np.float64).reshape(-1, 3)
    width, height = int(cam['width']), int(cam['height'])
    out = np.empty((height, width), dtype=np.float64)
    m9 = np.ascontiguousarray(m, dtype=np.float64).ravel()
    c3 = np.ascontiguousarray(centre, dtype=np.float64).ravel()
    lib.rs_ground(
        len(ends_c) // 2, ends_c.ctypes.data, m9.ctypes.data, c3.ctypes.data,
        float(cam['distance']), float(near_m), float(cam['scale']), float(cam['cx']),
        float(cam['cy']), float(cam.get('aspect', 0.5)), width, height, out.ctypes.data,
    )
    return out


def pose_parts(parts_p_n, turns9, spots3, total_n: int):
    """Transform `[(pts, nrm), ...]` in C into `(out_pos, out_nrm)`, or return `None`."""
    lib = _lib()
    if lib is None:
        return None
    nparts = len(parts_p_n)
    out_p = np.empty((total_n, 3), dtype=np.float64)
    out_n = np.empty((total_n, 3), dtype=np.float64)
    if nparts <= 0 or total_n <= 0:
        return out_p, out_n
    p_arrs = [np.ascontiguousarray(p, dtype=np.float64) for p, _n in parts_p_n]
    n_arrs = [np.ascontiguousarray(n, dtype=np.float64) for _p, n in parts_p_n]
    counts = np.array([len(a) for a in p_arrs], dtype=np.int32)
    if int(counts.sum()) != total_n or any(len(p) != len(n) for p, n in zip(p_arrs, n_arrs)):
        return None
    p_ptrs = np.array([a.ctypes.data for a in p_arrs], dtype=np.uintp)
    n_ptrs = np.array([a.ctypes.data for a in n_arrs], dtype=np.uintp)
    t_c = np.ascontiguousarray(turns9, dtype=np.float64)
    s_c = np.ascontiguousarray(spots3, dtype=np.float64)
    lib.rs_pose_parts(nparts, p_ptrs.ctypes.data, n_ptrs.ctypes.data, counts.ctypes.data,
                      t_c.ctypes.data, s_c.ctypes.data, out_p.ctypes.data, out_n.ctypes.data)
    return out_p, out_n
