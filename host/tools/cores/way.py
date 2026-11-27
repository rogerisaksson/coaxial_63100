"""Bit-exact SIMD C11 wayfinding distance, separating-axis footprint gap, and A* kernels (`way.c`)."""
import ctypes
import functools
import math
import os
import threading

import numpy as np

from tools import REPO
from tools.cores import build

WAY_C = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'way.c')
WAY_SOURCES = [WAY_C, os.path.join(REPO, 'wbc', 'src', 'wbc_libm.c')]
WAY_INCLUDES = [os.path.join(REPO, 'wbc', 'inc')]
WAY_FLAGS = ('-O3', '-ffp-contract=off', '-fno-math-errno', '-fno-trapping-math',
             '-fvect-cost-model=dynamic') + build.native()
_way_tls = threading.local()


def current() -> bool:
    """Whether `way.c` is already loaded or compiled on disk for this host."""
    return _lib.cache_info().currsize > 0 or build.is_built(WAY_SOURCES, WAY_INCLUDES, 'way', WAY_FLAGS)


def ensure():
    """Compile and load `way.c` if a host C compiler is available."""
    return _lib()


@functools.lru_cache(maxsize=1)
def _lib():
    """Compile and load `way.c` with `wbc_libm.c`, or return `None` without a C compiler."""
    if build.find_cc() is None:
        return None
    path, _warn = build.cached(WAY_SOURCES, WAY_INCLUDES, 'way', WAY_FLAGS)
    lib = ctypes.CDLL(path)
    i32, f64, vp = ctypes.c_int32, ctypes.c_double, ctypes.c_void_p
    lib.wy_apart.argtypes = [i32, vp, i32, vp, vp, vp, vp]
    lib.wy_apart.restype = None
    lib.wy_gaps.argtypes = [
        i32, vp, vp, vp, vp, vp, i32, i32, vp, vp, vp, vp, vp,
        f64, f64, f64, vp,
    ]
    lib.wy_gaps.restype = i32
    lib.wy_astar.argtypes = [
        vp, i32, i32, i32, i32, i32, i32,
        vp, vp, vp, vp, i32, vp,
    ]
    lib.wy_astar.restype = i32
    lib.wy_nearest_free.argtypes = [vp, i32, i32, i32, i32, vp]
    lib.wy_nearest_free.restype = None
    lib.wy_walked.argtypes = [vp, i32, i32, i32, i32, f64, f64, f64, vp, vp, vp, i32]
    lib.wy_walked.restype = i32
    return lib


def apart(points, centres, axes, half):
    """`(N, M)` floor distances from `points` to footprints `(centres, axes, half)` in C."""
    lib = _lib()
    if lib is None:
        return None
    pts = np.ascontiguousarray(points, dtype=np.float64).reshape(-1, 2)
    c_arr = np.ascontiguousarray(centres, dtype=np.float64).reshape(-1, 2)
    ax_arr = np.ascontiguousarray(axes, dtype=np.float64).reshape(-1, 2, 2)
    hf_arr = np.ascontiguousarray(half, dtype=np.float64).reshape(-1, 2)
    n_pts, m_things = len(pts), len(c_arr)
    out = np.empty((n_pts, m_things), dtype=np.float64)
    if n_pts > 0 and m_things > 0:
        lib.wy_apart(n_pts, pts.ctypes.data, m_things, c_arr.ctypes.data,
                     ax_arr.ctypes.data, hf_arr.ctypes.data, out.ctypes.data)
    return out


def gaps(poses, phi, k_dirs: int, meets, toward, centres, axes, half):
    """`(N, M)` separating-axis pose-to-thing gaps in C, bit-for-bit equal to `footprint.gaps_twin`."""
    lib = _lib()
    if lib is None:
        return None
    p_arr = np.ascontiguousarray(poses, dtype=np.float64).reshape(-1, 3)
    n_poses, m_things = len(p_arr), len(meets)
    if m_things == 0:
        return np.full((n_poses, 0), np.inf)
    out = np.empty((n_poses, m_things), dtype=np.float64)
    if n_poses == 0:
        return out
    yaw = p_arr[:, 2]
    sy = np.ascontiguousarray(np.sin(yaw), dtype=np.float64)
    cy = np.ascontiguousarray(np.cos(yaw), dtype=np.float64)
    trig = getattr(_way_tls, 'phi_trig', None)
    if trig is None or trig[0] is not phi:
        trig = (
            phi,
            np.ascontiguousarray(np.cos(phi), dtype=np.float64),
            np.ascontiguousarray(np.sin(phi), dtype=np.float64),
        )
        _way_tls.phi_trig = trig
    m_u8 = meets if (type(meets) is np.ndarray and meets.dtype == np.uint8 and meets.flags.c_contiguous) \
        else np.ascontiguousarray(meets, dtype=np.uint8)
    tw_c = toward if (type(toward) is np.ndarray and toward.dtype == np.float64 and toward.flags.c_contiguous) \
        else np.ascontiguousarray(toward, dtype=np.float64)
    c_c = centres if (type(centres) is np.ndarray and centres.dtype == np.float64 and centres.flags.c_contiguous) \
        else np.ascontiguousarray(centres, dtype=np.float64)
    ax_c = axes if (type(axes) is np.ndarray and axes.dtype == np.float64 and axes.flags.c_contiguous) \
        else np.ascontiguousarray(axes, dtype=np.float64)
    hf_c = half if (type(half) is np.ndarray and half.dtype == np.float64 and half.flags.c_contiguous) \
        else np.ascontiguousarray(half, dtype=np.float64)
    two_pi = 2.0 * float(np.pi)
    rc = lib.wy_gaps(
        n_poses, p_arr.ctypes.data, sy.ctypes.data, cy.ctypes.data,
        trig[1].ctypes.data, trig[2].ctypes.data, int(k_dirs), m_things, m_u8.ctypes.data, tw_c.ctypes.data,
        c_c.ctypes.data, ax_c.ctypes.data, hf_c.ctypes.data,
        two_pi, two_pi / int(k_dirs), math.cos(math.pi / int(k_dirs)), out.ctypes.data,
    )
    return out if rc == 0 else None


def _astar_scratch(ncells: int):
    got = getattr(_way_tls, 'astar', None)
    if got is None or got[0] < ncells:
        cap = max(256, 8 * ncells)
        got = (
            ncells,
            cap,
            np.empty(ncells, dtype=np.float64),
            np.empty(ncells, dtype=np.int32),
            np.empty(cap, dtype=np.float64),
            np.empty(cap, dtype=np.int32),
            np.empty((ncells, 2), dtype=np.int32),
        )
        _way_tls.astar = got
    return got[1:]


def astar(cost, start: tuple[int, int], goal: tuple[int, int]):
    """8-way minimum-cost grid path from `start` to `goal` in C, or `None` without C."""
    lib = _lib()
    if lib is None:
        return None
    c_arr = np.ascontiguousarray(cost, dtype=np.float64)
    if c_arr.ndim != 2:
        return None
    nx, nz = c_arr.shape
    ncells = nx * nz
    if ncells == 0:
        return []
    cap, spent, came, heap_f, heap_n, out_path = _astar_scratch(ncells)
    length = lib.wy_astar(
        c_arr.ctypes.data, nx, nz, int(start[0]), int(start[1]), int(goal[0]), int(goal[1]),
        spent.ctypes.data, came.ctypes.data, heap_f.ctypes.data, heap_n.ctypes.data, cap,
        out_path.ctypes.data,
    )
    if length < 0:
        return None
    if length == 0:
        return []
    return list(map(tuple, out_path[:length].tolist()))


def nearest_free(blocked, cell: tuple[int, int]):
    """Nearest free `(i, j)` cell of 2D bool `blocked` in C, or `None` without C."""
    lib = _lib()
    if lib is None or getattr(blocked, 'ndim', 0) != 2:
        return None
    b_arr = np.ascontiguousarray(blocked, dtype=np.uint8)
    nx, nz = b_arr.shape
    if nx == 0 or nz == 0:
        return (int(cell[0]), int(cell[1]))
    out_ij = (ctypes.c_int32 * 2)()
    lib.wy_nearest_free(b_arr.ctypes.data, nx, nz, int(cell[0]), int(cell[1]), out_ij)
    return (int(out_ij[0]), int(out_ij[1]))


def walked(cells, start: tuple[int, int], d0: float, cell_m: float):
    """8-way Dijkstra distance field `(nx, nz)` from `start` over free `cells` in C."""
    lib = _lib()
    if lib is None or getattr(cells, 'ndim', 0) != 2:
        return None
    b_arr = np.ascontiguousarray(cells, dtype=np.uint8)
    nx, nz = b_arr.shape
    ncells = nx * nz
    if ncells == 0:
        return np.empty((nx, nz), dtype=np.float64)
    cap, _spent, _came, heap_f, heap_n, _out_path = _astar_scratch(ncells)
    out = np.empty((nx, nz), dtype=np.float64)
    w_card = float(cell_m) * math.hypot(1, 0)
    w_diag = float(cell_m) * math.hypot(1, 1)
    rc = lib.wy_walked(
        b_arr.ctypes.data, nx, nz, int(start[0]), int(start[1]),
        float(d0), w_card, w_diag, out.ctypes.data, heap_f.ctypes.data, heap_n.ctypes.data, cap,
    )
    return out if rc == 0 else None
