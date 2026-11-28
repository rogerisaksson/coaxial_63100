"""SIMD C11 L2-wavefront stereo SGM disparity and 3D voxel raycast/scatter kernels (`vision.c`)."""
import ctypes
import functools
import os
import threading

import numpy as np

from tools.cores import build

_DIR = os.path.dirname(os.path.abspath(__file__))
VISION_C = os.path.join(_DIR, 'vision.c')
VISION_EPI_C = os.path.join(_DIR, 'vision_epi.c')
VISION_GRID_C = os.path.join(_DIR, 'vision_grid.c')
VISION_SOURCES = [VISION_C, VISION_EPI_C, VISION_GRID_C]
VISION_FLAGS = ('-O3', '-ffp-contract=off', '-fno-math-errno', '-fno-trapping-math',
                '-fvect-cost-model=dynamic') + build.native()
MAX_THREADS = 8
_vision_tls = threading.local()


def current() -> bool:
    """Whether `vision.c` is already loaded or compiled on disk for this host."""
    return _lib.cache_info().currsize > 0 or build.is_built(VISION_SOURCES, [], 'vision', VISION_FLAGS)


def ensure():
    """Compile and load `vision.c` if a host C compiler is available."""
    return _lib()


@functools.lru_cache(maxsize=1)
def _lib():
    """Compile and load `vision.c`, or return `None` when no host C compiler is available."""
    if build.find_cc() is None:
        return None
    path, _warn = build.cached(VISION_SOURCES, [], 'vision', VISION_FLAGS)
    lib = ctypes.CDLL(path)
    i32, i64, f32, f64, vp = (ctypes.c_int32, ctypes.c_int64, ctypes.c_float,
                              ctypes.c_double, ctypes.c_void_p)
    lib.vs_grey_u8.argtypes = [vp, vp, i32]
    lib.vs_grey_u8.restype = None
    lib.vs_box_i64.argtypes = [vp, vp, vp, vp, i32, i32, i32]
    lib.vs_box_i64.restype = None
    lib.vs_stereo_core.argtypes = [
        vp, vp, i32, i32, i32, i32, i32, i32, i64, i32, f32, f32,
        vp, vp, vp, vp, vp, vp, vp, vp, vp, vp, vp, vp, vp,
    ]
    lib.vs_stereo_core.restype = i32
    lib.vs_stereo_post.argtypes = [
        vp, vp, vp, vp, vp, vp, vp, vp,
        i32, i32, i32, i32, f64, f32,
        vp, vp, vp, vp, vp, vp,
    ]
    lib.vs_stereo_post.restype = None
    lib.vs_voxel_add.argtypes = [
        vp, vp, i32, vp, i32, i32, i32,
        f64, f64, f64, f64, f64, f64, f64, f64, i32, i32,
        vp, vp, vp, vp, vp, vp, vp, vp, vp, vp,
    ]
    lib.vs_voxel_add.restype = None
    lib.vs_warp_u8.argtypes = [vp, vp, vp, vp, i32, i32, i32, vp]
    lib.vs_warp_u8.restype = None
    lib.vs_epi_from_d.argtypes = [vp, i32, i32, f64, i32, i32, vp, vp, vp, vp, vp]
    lib.vs_epi_from_d.restype = None
    lib.vs_epi_kept.argtypes = [vp, vp, vp, i32, i32, vp]
    lib.vs_epi_kept.restype = None
    lib.vs_convex_f64.argtypes = [vp, i32, vp, vp]
    lib.vs_convex_f64.restype = i32
    lib.vs_grid_cc.argtypes = [vp, vp, i32, i32, i32, i32, vp, i32, vp, vp, vp, vp, vp, vp]
    lib.vs_grid_cc.restype = i32
    return lib


def _stereo_scratch(height: int, width: int, d_stride: int, rad: int):
    key = (height, width, d_stride, rad)
    got = getattr(_vision_tls, 'stereo', None)
    if got is None or got[0] != key:
        npix = height * width
        ring_rows = 2 * rad + 2
        got = (
            key,
            np.empty(4 * npix, dtype=np.uint64),
            np.empty(npix * d_stride, dtype=np.uint8),
            np.empty(npix * d_stride, dtype=np.float32),
            np.empty((height + width) * (d_stride + 2), dtype=np.float32),
            np.empty(MAX_THREADS * ring_rows * width, dtype=np.int64),
            np.empty(max(MAX_THREADS * width, width + 2 * npix), dtype=np.int64),
            np.empty(npix, dtype=np.int64),
            np.empty(npix, dtype=np.int64),
            np.empty(npix, dtype=np.int64),
            np.empty(npix, dtype=np.int64),
            np.empty(npix, dtype=np.int64),
            np.empty(npix, dtype=np.int64),
            np.empty(npix, dtype=np.float32),
            np.empty(npix, dtype=np.float32),
            np.empty(npix, dtype=np.int64),
            np.empty(npix, dtype=np.float32),
            np.empty(npix, dtype=np.float32),
            np.empty(npix, dtype=np.float32),
        )
        _vision_tls.stereo = got
    return got[1:]


def _voxel_scratch(nvox: int):
    got = getattr(_vision_tls, 'voxel', None)
    if got is None or got[0] != nvox:
        got = (
            nvox,
            np.empty(nvox, dtype=np.uint8),
            np.zeros(nvox, dtype=np.uint8),
            np.zeros(nvox, dtype=np.int32),
        )
        _vision_tls.voxel = got
    return got[1:]


def box_i64(arr, rad: int):
    """`(2 * rad + 1)^2` edge-padded box sum of 2D `int64` `arr` in C, or `None` without C."""
    lib = _lib()
    if lib is None or arr.ndim != 2:
        return None
    src = np.ascontiguousarray(arr, dtype=np.int64)
    height, width = src.shape
    dst = np.empty_like(src)
    hsum = np.empty_like(src)
    vsum = np.empty(max(1, width), dtype=np.int64)
    lib.vs_box_i64(src.ctypes.data, dst.ctypes.data, hsum.ctypes.data, vsum.ctypes.data,
                   height, width, int(rad))
    return dst


def disparity(left, right, d_count: int, census_xy: tuple[int, int], quantum: int,
              window: int, p1: float, p2: float, texture: float, unique: float, grey_fn):
    """Compute `(disp, kept)` matching `stereo.cpu_disparity` bit-for-bit, or `None` without C."""
    lib = _lib()
    if lib is None:
        return None
    d_stride = (int(d_count) + 15) & ~15
    cx, cy = int(census_xy[0]), int(census_xy[1])
    if int(d_count) <= 0 or d_stride > 128 or cx <= 0 or cy <= 0 or (cx * cy - 1) > 64:
        return None
    l_arr, r_arr = np.asarray(left), np.asarray(right)
    if l_arr.ndim < 2 or r_arr.ndim < 2:
        return None
    height, width = l_arr.shape[:2]
    rad = int(window) // 2
    if height <= 0 or width <= 0 or rad < 0 or r_arr.shape[:2] != (height, width):
        return None
    npix = height * width
    (census_buf, cost_u8, total_hwd, wave_buf, ring_buf, vsum_buf,
     box_a, box_b, box_c, gl_buf, gr_buf, best, tbest, second, seen, c0, lo, hi) = _stereo_scratch(
         height, width, d_stride, rad)
    if l_arr.dtype == np.uint8 and l_arr.shape == (height, width, 3):
        lc = np.ascontiguousarray(l_arr)
        lib.vs_grey_u8(lc.ctypes.data, gl_buf.ctypes.data, npix)
        gl = gl_buf.reshape(height, width)
    else:
        gl = np.ascontiguousarray(grey_fn(l_arr, np), dtype=np.int64)
    if r_arr.dtype == np.uint8 and r_arr.shape == (height, width, 3):
        rc = np.ascontiguousarray(r_arr)
        lib.vs_grey_u8(rc.ctypes.data, gr_buf.ctypes.data, npix)
        gr = gr_buf.reshape(height, width)
    else:
        gr = np.ascontiguousarray(grey_fn(r_arr, np), dtype=np.int64)

    status = lib.vs_stereo_core(
        gl.ctypes.data, gr.ctypes.data, height, width, int(d_count), d_stride,
        int(census_xy[0]) // 2, int(census_xy[1]) // 2, int(quantum), rad,
        float(p1), float(p2),
        census_buf.ctypes.data, cost_u8.ctypes.data, total_hwd.ctypes.data,
        wave_buf.ctypes.data, ring_buf.ctypes.data, vsum_buf.ctypes.data,
        best.ctypes.data, tbest.ctypes.data, second.ctypes.data, seen.ctypes.data,
        c0.ctypes.data, lo.ctypes.data, hi.ctypes.data,
    )
    if status != 0:
        return None
    disp = np.empty((height, width), dtype=np.float64)
    kept = np.empty((height, width), dtype=np.bool_)
    lib.vs_stereo_post(
        gl.ctypes.data, best.ctypes.data, tbest.ctypes.data, second.ctypes.data, seen.ctypes.data,
        c0.ctypes.data, lo.ctypes.data, hi.ctypes.data,
        height, width, int(d_count), rad, float(texture), float(np.float32(unique)),
        box_a.ctypes.data, box_b.ctypes.data, box_c.ctypes.data, vsum_buf.ctypes.data,
        disp.ctypes.data, kept.ctypes.data,
    )
    return disp, kept


def voxel_add(chart, xyz, rgb, eye, ox: float, floor_m: float, oz: float, ceil_m: float,
              near_m: float, far_m: float, hits_min: int, told_min: int, step_vox: float = 0.8):
    """Update `chart` (`voxels.Map`) in C matching `Map.add` bit-for-bit, or `None` without C."""
    lib = _lib()
    if lib is None:
        return None
    pts = np.ascontiguousarray(xyz, dtype=np.float64)
    cols = np.ascontiguousarray(rgb, dtype=np.float32)
    eye_c = np.ascontiguousarray(eye, dtype=np.float64).ravel()
    n_pts = len(pts)
    out_flat = np.empty(n_pts, dtype=np.int64)
    if n_pts > 0:
        occ_buf, crossed_buf, counts_buf = _voxel_scratch(chart.hits.size)
        lib.vs_voxel_add(
            pts.ctypes.data, cols.ctypes.data, n_pts, eye_c.ctypes.data,
            int(chart.nx), int(chart.ny), int(chart.nz),
            float(ox), float(floor_m), float(oz), float(chart.voxel),
            float(ceil_m), float(near_m), float(far_m), float(step_vox),
            int(hits_min), int(told_min),
            chart.hits.ctypes.data, chart.told.ctypes.data, chart.free.ctypes.data,
            chart.top.ctypes.data, chart.moments.ctypes.data, chart.colour.ctypes.data,
            occ_buf.ctypes.data, crossed_buf.ctypes.data, counts_buf.ctypes.data,
            out_flat.ctypes.data,
        )
    chart.looks, chart._made = chart.looks + 1, None
    return out_flat


def warp_u8(img, at4, wt4, on_mask, height: int, width: int, bits: int):
    """Bilinear 2x2 area epipolar warp of `(2 * height, 2 * width, 3)` `uint8` `img` in C."""
    lib = _lib()
    if lib is None:
        return None
    src = np.ascontiguousarray(img, dtype=np.uint8)
    at_c = np.ascontiguousarray(at4, dtype=np.int32)
    wt_c = np.ascontiguousarray(wt4, dtype=np.int32)
    on_c = np.ascontiguousarray(on_mask, dtype=np.uint8)
    dst = np.empty((height, width, 3), dtype=np.uint8)
    lib.vs_warp_u8(src.ctypes.data, at_c.ctypes.data, wt_c.ctypes.data, on_c.ctypes.data,
                   int(height), int(width), int(bits), dst.ctypes.data)
    return dst


def epi_from_d(d_xyz, height: int, width: int, f_px: float, bits: int, fine: int):
    """Compute `((V, U), on, (at4, wt4))` from `(height, width, 3)` `d_xyz` in C, or `None` without C."""
    lib = _lib()
    if lib is None or height <= 0 or width <= 0 or fine <= 0 or bits < 0 or bits > 30:
        return None
    dc = np.ascontiguousarray(d_xyz, dtype=np.float64)
    if dc.shape != (height, width, 3):
        return None
    npix = height * width
    v_out = np.empty((height, width), dtype=np.int64)
    u_out = np.empty((height, width), dtype=np.int64)
    on_out = np.empty((height, width), dtype=np.bool_)
    at4 = np.empty((4, npix), dtype=np.int32)
    wt4 = np.empty((4, npix), dtype=np.int32)
    lib.vs_epi_from_d(dc.ctypes.data, int(height), int(width), float(f_px), int(bits), int(fine),
                      v_out.ctypes.data, u_out.ctypes.data, on_out.ctypes.data,
                      at4.ctypes.data, wt4.ctypes.data)
    return (v_out, u_out), on_out, (at4, wt4)


def epi_kept(inner0, inner1, disp, height: int, width: int):
    """Compute `(height, width)` epipolar kept mask in C, or `None` without C."""
    lib = _lib()
    if lib is None or height <= 0 or width <= 0:
        return None
    i0 = np.ascontiguousarray(inner0, dtype=np.uint8)
    i1 = np.ascontiguousarray(inner1, dtype=np.uint8)
    dc = np.ascontiguousarray(disp, dtype=np.float64)
    if i0.shape != (height, width) or i1.shape != (height, width) or dc.shape != (height, width):
        return None
    out = np.empty((height, width), dtype=np.bool_)
    lib.vs_epi_kept(i0.ctypes.data, i1.ctypes.data, dc.ctypes.data, int(height), int(width), out.ctypes.data)
    return out


def _cc_scratch(n_total: int):
    got = getattr(_vision_tls, 'cc', None)
    if got is None or got[0] < n_total:
        cap = max(n_total, 4096)
        got = (
            cap,
            np.empty(cap, dtype=np.int32),
            np.empty(cap, dtype=np.int32),
            np.empty(cap, dtype=np.int32),
            np.empty(cap, dtype=np.int32),
            np.empty(cap + 1, dtype=np.int32),
        )
        _vision_tls.cc = got
    return got[1:]


def _hull_scratch(n_pts: int):
    got = getattr(_vision_tls, 'hull', None)
    if got is None or got[0] < n_pts:
        cap = max(n_pts, 512)
        got = (
            cap,
            np.empty((cap, 2), dtype=np.float64),
            np.empty((cap + 2, 2), dtype=np.float64),
        )
        _vision_tls.hull = got
    return got[1:]


def convex_f64(pts_xy):
    """`(m, 2)` float64 CCW convex hull of `(n, 2)` `pts_xy` in C, or `None` on non-finite/no C."""
    lib = _lib()
    if lib is None:
        return None
    pts = np.ascontiguousarray(pts_xy, dtype=np.float64).reshape(-1, 2)
    n_pts = len(pts)
    if n_pts == 0:
        return np.empty((0, 2), dtype=np.float64)
    work_xy, out_xy = _hull_scratch(n_pts)
    m = lib.vs_convex_f64(pts.ctypes.data, n_pts, work_xy.ctypes.data, out_xy.ctypes.data)
    if m < 0:
        return None
    return out_xy[:m].copy()


def _offs3(offsets, ndim: int):
    offs = np.asarray(offsets, dtype=np.int32).reshape(-1, ndim)
    if ndim < 3:
        offs = np.pad(offs, ((0, 0), (3 - ndim, 0)))
    return np.ascontiguousarray(offs, dtype=np.int32)


def grid_joined(on, offsets):
    """Connected components of boolean `on` (1D/2D/3D) in C matching `grids.joined`, or `None`."""
    lib = _lib()
    arr = np.asarray(on, dtype=np.bool_)
    ndim = arr.ndim
    if lib is None or ndim not in (1, 2, 3) or arr.size > (1 << 26):
        return None
    if arr.size == 0:
        return []
    src = np.ascontiguousarray(arr, dtype=np.uint8)
    d0, d1, d2 = ((1, 1) + src.shape)[-3:]
    offs3 = _offs3(offsets, ndim)
    pos_b, par_b, aux_b, lbl_b, st_b = _cc_scratch(src.size)
    coords = np.empty(ndim * src.size, dtype=np.int64)
    n_comp = lib.vs_grid_cc(
        src.ctypes.data, None, ndim, d0, d1, d2, offs3.ctypes.data, len(offs3),
        pos_b.ctypes.data, par_b.ctypes.data, aux_b.ctypes.data,
        coords.ctypes.data, lbl_b.ctypes.data, st_b.ctypes.data)
    if n_comp == 0:
        return []
    k_act = int(st_b[n_comp])
    c_tight = coords[:ndim * k_act].reshape(ndim, k_act)
    st = st_b[:n_comp + 1].tolist()
    return [tuple(c_tight[d, st[g]:st[g + 1]] for d in range(ndim)) for g in range(n_comp)]


def grid_cc_codes(codes, offsets):
    """`(idx, label, value)` for `codes >= 0` (1D/2D/3D) in C matching `grids.joined_by`, or `None`."""
    lib = _lib()
    arr = np.asarray(codes)
    ndim = arr.ndim
    if lib is None or ndim not in (1, 2, 3) or arr.size > (1 << 26):
        return None
    if arr.size == 0:
        return None
    src = np.ascontiguousarray(arr, dtype=np.int64)
    d0, d1, d2 = ((1, 1) + src.shape)[-3:]
    offs3 = _offs3(offsets, ndim)
    pos_b, par_b, aux_b, lbl_b, st_b = _cc_scratch(src.size)
    coords = np.empty(ndim * src.size, dtype=np.int64)
    k_act = lib.vs_grid_cc(
        None, src.ctypes.data, ndim, d0, d1, d2, offs3.ctypes.data, len(offs3),
        pos_b.ctypes.data, par_b.ctypes.data, aux_b.ctypes.data,
        coords.ctypes.data, lbl_b.ctypes.data, st_b.ctypes.data)
    if k_act == 0:
        return ()
    idx = coords[:ndim * k_act].reshape(ndim, k_act).copy()
    label = lbl_b[:k_act].astype(np.int64)
    value = src.reshape(-1)[aux_b[:k_act]]
    return idx, label, value
