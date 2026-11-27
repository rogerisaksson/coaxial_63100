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
_phi_trig: list = [None, None, None]


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
        i32, vp, vp, vp, vp, i32, i32, vp, vp, vp, vp, vp,
        f64, f64, f64, vp,
    ]
    lib.wy_gaps.restype = i32
    lib.wy_astar.argtypes = [
        vp, i32, i32, i32, i32, i32, i32,
        vp, vp, vp, vp, i32, vp,
    ]
    lib.wy_astar.restype = i32
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
    yaw, at_pts = p_arr[:, 2], np.ascontiguousarray(p_arr[:, :2], dtype=np.float64)
    sy, cy = np.sin(yaw), np.cos(yaw)
    ahead = np.ascontiguousarray(np.stack([sy, cy], -1), dtype=np.float64)
    left = np.ascontiguousarray(np.stack([cy, -sy], -1), dtype=np.float64)
    if _phi_trig[0] is not phi:
        _phi_trig[:] = [phi, np.cos(phi)[None, :, None], np.sin(phi)[None, :, None]]
    dirs = np.ascontiguousarray(
        _phi_trig[1] * ahead[:, None, :] + _phi_trig[2] * left[:, None, :],
        dtype=np.float64,
    )
    m_u8 = np.ascontiguousarray(meets, dtype=np.uint8)
    tw_c = np.ascontiguousarray(toward, dtype=np.float64)
    c_c = np.ascontiguousarray(centres, dtype=np.float64)
    ax_c = np.ascontiguousarray(axes, dtype=np.float64)
    hf_c = np.ascontiguousarray(half, dtype=np.float64)
    two_pi = 2.0 * float(np.pi)
    rc = lib.wy_gaps(
        n_poses, at_pts.ctypes.data, ahead.ctypes.data, left.ctypes.data,
        dirs.ctypes.data, int(k_dirs), m_things, m_u8.ctypes.data, tw_c.ctypes.data,
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
