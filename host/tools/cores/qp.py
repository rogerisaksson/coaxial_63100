"""machine.stack.qp.solve in C (wbc_qp.c), its QR and its sums: bit for bit on an exact host.

    x, held, u = qp.solve(H, g, A, b, warm=(), most=None)   # machine.stack.qp.solve's tuple; x None where none meets it
    Q, R = qp.qr(a)                                         # wbc_qr: machine.stack.qp.householder's bits, every machine's
    s = qp.npsum(a)                                         # wbc_np.h's npsum: numpy's pairwise sum of a run
"""
import ctypes
import functools

import numpy as np

from tools.cores import held, wbc

_d, _i, _vp = ctypes.c_double, ctypes.c_int, ctypes.c_void_p


@functools.lru_cache(maxsize=None)
def _typed():
    """The solve typed."""
    lib = wbc.library()
    lib.wbh_qp_solve.argtypes = [_vp] * 5 + [_i, _i, _vp, _i, _i, _vp, _vp, _vp, _vp]
    lib.wbh_qp_solve.restype = _i
    return lib.wbh_qp_solve


def solve(H, g, A, b, warm=(), most=None):
    """(x, the rows held, their multipliers) minimising 1/2 x'Hx + g'x with A x <= b, as `machine.stack.qp.solve`
    returns them (x None where none meets it); LinAlgError where numpy would raise."""
    g, b = np.ascontiguousarray(g, float).reshape(-1), np.ascontiguousarray(b, float).reshape(-1)
    n, m = len(g), len(b)
    H = np.ascontiguousarray(H, float).reshape(n, n)
    A = np.ascontiguousarray(A, float).reshape(m, n) if m else np.zeros((1, n))
    rows = np.ascontiguousarray(list(warm) or [0], np.int32)
    x, act, u, nact = np.zeros(n), np.zeros(n + m + 1, np.int32), np.zeros(n + m + 1), ctypes.c_int(0)
    got = _typed()(ctypes.addressof(held.blas()), wbc.address(H), wbc.address(g), wbc.address(A),
                   wbc.address(b if m else np.zeros(1)), n, m, wbc.address(rows), len(warm), int(most or 0),
                   wbc.address(x), wbc.address(act), ctypes.byref(nact), wbc.address(u))
    if got == 2:
        raise np.linalg.LinAlgError('wbc_qp_solve: numpy would raise')
    if got == 1:
        return None, (), np.zeros(0)
    k = int(nact.value)
    return x, tuple(int(i) for i in act[:k]), u[:k].copy()


@functools.lru_cache(maxsize=None)
def _harness():
    """The QR and the sum typed."""
    lib = wbc.library()
    lib.wbh_qr.argtypes = [_vp, _vp, _i, _i, _vp, _vp]
    lib.wbh_qr.restype = _i
    lib.wbh_npsum.argtypes = [_vp, _i]
    lib.wbh_npsum.restype = _d
    return lib


def qr(a):
    """(Q, R) of `a` (m, q), q <= m, by wbc_qr - the robot's own Householder in numpy's qr(a, 'complete')'s shapes -;
    None where it fails."""
    a = np.ascontiguousarray(a, float)
    m, q = a.shape
    Q, R = np.zeros((m, m)), np.zeros((m, q))
    got = _harness().wbh_qr(ctypes.addressof(held.blas()), wbc.address(a), m, q, wbc.address(Q), wbc.address(R))
    return None if got else (Q, R)


def npsum(a):
    """numpy's pairwise sum of the run `a`, in C (wbc_np.h's npsum)."""
    a = np.ascontiguousarray(a, float).reshape(-1)
    return float(_harness().wbh_npsum(wbc.address(a if a.size else np.zeros(1)), int(a.size)))
