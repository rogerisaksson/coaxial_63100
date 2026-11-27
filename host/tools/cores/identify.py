"""SIMD C11 kernels (`identify.c`) for campaign low-pass, hem kinematics, and self-contact."""
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
_HEMS: dict[tuple[int, int], tuple] = {}


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
    lib.id_hem_anchor.argtypes = [
        i32, i32, i32, i32, i32, i32, i32, f64, vp,
        i32, vp, vp, vp,
        i32, vp, vp, vp,
        vp, vp, vp,
    ]
    lib.id_hem_anchor.restype = i32
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


def _quat_rot_rows(q_arr):
    """Convert `(K, 4)` `(w, x, y, z)` quaternions into contiguous `(K, 9)` rotation matrices."""
    q = np.asarray(q_arr, dtype=np.float64)
    if len(q) == 0:
        return np.zeros((1, 9), dtype=np.float64)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.column_stack([
        1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
        2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
        2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy),
    ]).astype(np.float64, copy=False)


def _hem_chains(m, lay):
    """Return cached `{k: (b, links, j_span, jnt_ids, j_idx)}` topology for `lay['hems']`."""
    key = (id(m), id(lay))
    hit = _HEMS.get(key)
    if hit is not None and hit[0] is m and hit[1] is lay:
        return hit[2]
    chains = {}
    for k in lay.get('hems', ()):
        k_i = int(k)
        b = int(lay['bodies'][k_i])
        rev, p = [], int(lay['parent'][k_i])
        while p > 0:
            rev.append(int(lay['bodies'][p]))
            p = int(lay['parent'][p])
        links = list(reversed(rev))
        j_span = np.zeros((max(1, len(links)), 2), dtype=np.int32)
        jnt_ids, j_idx_l = [], []
        for lk_i, link in enumerate(links):
            j_span[lk_i, 0] = len(j_idx_l)
            first = int(m.body_jntadr[link])
            for j in range(first, first + int(m.body_jntnum[link])):
                jnt_ids.append(j)
                j_idx_l.append(int(np.flatnonzero(lay['vadr'] == m.jnt_dofadr[j])[0]))
            j_span[lk_i, 1] = len(j_idx_l)
        chains[k_i] = (b, links, j_span, jnt_ids, np.array(j_idx_l or [0], dtype=np.int32), len(j_idx_l))
    while len(_HEMS) >= KEPT:
        del _HEMS[next(iter(_HEMS))]
    _HEMS[key] = (m, lay, chains)
    return chains


def hem_anchor(m, lay, ys_arr, dt: float, k: int, offsets: tuple[int, int, int, int, int]):
    """Compute `(ap, om, alb)` `(N - 1, 3)` in C for hem `k`, or return `None` when C is unavailable."""
    lib = _lib()
    if lib is None:
        return None
    chains = _hem_chains(m, lay)
    spec = chains.get(int(k))
    if spec is None:
        return None
    b, links, j_span, jnt_ids, j_idx, nj_tot = spec
    ys_c = np.ascontiguousarray(ys_arr, dtype=np.float64)
    n = len(ys_c) - 1
    if n <= 0:
        z = np.zeros((0, 3), dtype=np.float64)
        return z, z, z
    nlinks = len(links)
    link_pos = np.ascontiguousarray(m.body_pos[links], dtype=np.float64) if nlinks else np.zeros((1, 3))
    link_rot = _quat_rot_rows(m.body_quat[links]) if nlinks else np.zeros((1, 9))
    j_ax = np.ascontiguousarray(m.jnt_axis[jnt_ids], dtype=np.float64) if nj_tot else np.zeros((1, 3))
    end_pos = np.ascontiguousarray(m.body_pos[b], dtype=np.float64)
    ap = np.empty((n, 3), dtype=np.float64)
    om = np.empty((n, 3), dtype=np.float64)
    alb = np.empty((n, 3), dtype=np.float64)
    quat_at, gyro_at, accel_at, q_at, qd_at = (int(v) for v in offsets)
    rc = lib.id_hem_anchor(
        n, ys_c.shape[1], quat_at, gyro_at, accel_at, q_at, qd_at, float(dt), ys_c.ctypes.data,
        nlinks, link_pos.ctypes.data, link_rot.ctypes.data, j_span.ctypes.data,
        nj_tot, j_idx.ctypes.data, j_ax.ctypes.data, end_pos.ctypes.data,
        ap.ctypes.data, om.ctypes.data, alb.ctypes.data,
    )
    if rc != 0:
        return None
    return ap, om, alb
