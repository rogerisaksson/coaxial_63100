"""SIMD C11 mesh transform, projection, and O(N) dot-splat kernels (`raster.c`) for Braille 3D views."""
import ctypes
import functools
import os

import numpy as np

from tools.cores import build

SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'raster.c')
FLAGS = ('-O3', '-fno-math-errno', '-fno-trapping-math', '-fvect-cost-model=dynamic') + build.native()
_SCRATCH: dict[int, tuple] = {}
_SPANS: dict[int, tuple] = {}
SCRATCH_SLOTS = 8


@functools.lru_cache(maxsize=1)
def _lib():
    """Compile and load `raster.c`, or return `None` when no host C compiler is available."""
    if build.find_cc() is None:
        return None
    path, _warn = build.cached([SOURCE], [], 'raster', FLAGS)
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
