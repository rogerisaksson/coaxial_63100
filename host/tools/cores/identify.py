"""SIMD C11 kernels (`identify.c`) for campaign low-pass smoothing and capsule self-contact."""
import ctypes
import functools
import os

import numpy as np

from tools import REPO
from tools.cores import build

SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'identify.c')
SOURCES = [SOURCE, os.path.join(REPO, 'wbc', 'src', 'wbc_libm.c')]
INCLUDES = [os.path.join(REPO, 'wbc', 'inc')]
FLAGS = (('-O3', '-fno-math-errno', '-fno-trapping-math', '-fvect-cost-model=dynamic',
          '-fno-builtin-sin', '-fno-builtin-cos') + build.native())
KEPT = 8
_CAPS: dict[int, tuple] = {}


@functools.lru_cache(maxsize=1)
def _lib():
    """Compile and load `identify.c` with `wbc_libm.c`, or return `None` if no C compiler exists."""
    if build.find_cc() is None:
        return None
    path, _warn = build.cached(SOURCES, INCLUDES, 'identify', FLAGS)
    lib = ctypes.CDLL(path)
    i32, f64, vp = ctypes.c_int, ctypes.c_double, ctypes.c_void_p
    lib.id_lowpass.argtypes = [vp, i32, i32, f64]
    lib.id_lowpass.restype = None
    lib.id_self_hit.argtypes = [
        i32, i32, i32, i32, vp,
        i32, vp, vp, vp, vp,
        i32, vp, vp, vp, vp,
        i32, vp, vp, vp,
        i32, vp, vp, vp, vp,
        i32, vp, vp, vp,
    ]
    lib.id_self_hit.restype = i32
    return lib


def lowpass(x, a: float):
    """Filter `x` `(N, ...)` in C both ways with pole `a`, or return `None` when C is unavailable."""
    lib = _lib()
    if lib is None:
        return None
    arr = np.array(x, dtype=np.float64, copy=True, order='C')
    if len(arr) >= 2 and arr.size > 0:
        cols = arr.size // len(arr)
        lib.id_lowpass(arr.ctypes.data, len(arr), cols, float(a))
    return arr


def _packed_caps(lay):
    """Pack `lay['caps']` and `lay['parent']` into flat C arrays cached by `id(caps)` without mutating `lay`."""
    caps = lay.get('caps')
    if caps is None:
        return None
    key = id(caps)
    hit = _CAPS.get(key)
    if hit is not None and hit[0] is caps:
        return hit[1]
    b_pos, b_rot, jnts, ck, _cr, ca, cb, pr, pdrv, reach_sq, rad_sq = caps
    nb = len(lay['bodies'])
    parent = np.ascontiguousarray(lay['parent'], dtype=np.int32)
    b_pos_c = np.ascontiguousarray(b_pos, dtype=np.float64)
    b_rot_c = np.ascontiguousarray(b_rot.reshape(nb, 9), dtype=np.float64)
    j_span = np.zeros((nb, 2), dtype=np.int32)
    j_idx_l, j_ax_l, j_pos_l, j_has_l = [], [], [], []
    for k in range(nb):
        j_span[k, 0] = len(j_idx_l)
        for ji, ax, jp in jnts[k]:
            j_idx_l.append(int(ji))
            j_ax_l.append(ax)
            j_pos_l.append(jp)
            j_has_l.append(1 if any(jp) else 0)
        j_span[k, 1] = len(j_idx_l)
    nj_tot = len(j_idx_l)
    j_idx = np.array(j_idx_l or [0], dtype=np.int32)
    j_ax = np.array(j_ax_l or [[0.0, 0.0, 0.0]], dtype=np.float64)
    j_pos = np.array(j_pos_l or [[0.0, 0.0, 0.0]], dtype=np.float64)
    j_has = np.array(j_has_l or [0], dtype=np.uint8)
    ck_c = np.ascontiguousarray(ck, dtype=np.int32)
    ca_c = np.ascontiguousarray(ca, dtype=np.float64)
    cb_c = np.ascontiguousarray(cb, dtype=np.float64)
    pair_a = np.ascontiguousarray(pr[:, 0], dtype=np.int32)
    pair_b = np.ascontiguousarray(pr[:, 1], dtype=np.int32)
    rsq_c = np.ascontiguousarray(reach_sq, dtype=np.float64)
    csq_c = np.ascontiguousarray(rad_sq, dtype=np.float64)
    pdrv_u8 = np.ascontiguousarray(pdrv, dtype=np.uint8)
    ndrv = pdrv_u8.shape[1]
    pdrv_mask = np.zeros(max(1, len(pr)), dtype=np.uint64)
    if 0 < ndrv <= 64 and len(pr) > 0:
        shifts = np.arange(ndrv, dtype=np.uint64)
        pdrv_mask[:len(pr)] = np.bitwise_or.reduce(pdrv_u8.astype(np.uint64) << shifts, axis=1)
    packed = (nb, parent, b_pos_c, b_rot_c, j_span, nj_tot, j_idx, j_ax, j_pos, j_has,
              len(ck_c), ck_c, ca_c, cb_c, len(pr), pair_a, pair_b, rsq_c, csq_c, ndrv, pdrv_u8, pdrv_mask)
    while len(_CAPS) >= KEPT:
        del _CAPS[next(iter(_CAPS))]
    _CAPS[key] = (caps, packed)
    return packed


def self_hit(lay, ys_arr, quat_at: int, q_at: int):
    """Compute `(N, driven)` self-overlap mask in C, or return `None` when C is unavailable or exceeded."""
    lib = _lib()
    if lib is None:
        return None
    packed = _packed_caps(lay)
    if packed is None or not len(ys_arr):
        return np.zeros((len(ys_arr), len(lay['driven'])), dtype=bool)
    ys_c = np.ascontiguousarray(ys_arr, dtype=np.float64)
    (nb, parent, b_pos_c, b_rot_c, j_span, nj_tot, j_idx, j_ax, j_pos, j_has,
     ncaps, ck_c, ca_c, cb_c, npairs, pair_a, pair_b, rsq_c, csq_c, ndrv, pdrv_u8, pdrv_mask) = packed
    out = np.zeros((len(ys_c), ndrv), dtype=np.uint8)
    rc = lib.id_self_hit(
        len(ys_c), ys_c.shape[1], int(quat_at), int(q_at), ys_c.ctypes.data,
        nb, parent.ctypes.data, b_pos_c.ctypes.data, b_rot_c.ctypes.data, j_span.ctypes.data,
        nj_tot, j_idx.ctypes.data, j_ax.ctypes.data, j_pos.ctypes.data, j_has.ctypes.data,
        ncaps, ck_c.ctypes.data, ca_c.ctypes.data, cb_c.ctypes.data,
        npairs, pair_a.ctypes.data, pair_b.ctypes.data, rsq_c.ctypes.data, csq_c.ctypes.data,
        ndrv, pdrv_u8.ctypes.data, pdrv_mask.ctypes.data, out.ctypes.data,
    )
    if rc != 0:
        return None
    return out.view(bool)
