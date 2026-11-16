"""machine.libm's functions in C through ctypes: wbc_libm.c's arrays and its scalar calls.

wbc/src/wbc_libm.c's arrays, and each scalar function as another core inlines it (wbc/test/harness_libm.c).

    from tools.cores import libm_c
    y = libm_c.call('sin', x)          # wbc_libm_sin over x, a new float64 array; 'arctan2' and 'hypot' take two
    s, c = libm_c.sincos(x)            # wbc_libm_sincos
    y = libm_c.scalar('sin', x)        # wbc_sin a call at a time ('sincos_sin', 'sincos_cos' its pair's)
"""
import ctypes
import functools

import numpy as np

from tools.cores import wbc

#: The functions of one argument and of two, numpy's names; wbh_libm_scalar's order after them.
ONE = ('sin', 'cos', 'tan', 'exp', 'expm1', 'log', 'arctan', 'arcsin', 'arccos')
TWO = ('arctan2', 'hypot')
SCALAR = ONE + TWO + ('sincos_sin', 'sincos_cos')
_dp, _i = ctypes.POINTER(ctypes.c_double), ctypes.c_int32


@functools.lru_cache(maxsize=None)
def lib():
    """The cores' library, wbh_libm_*'s types laid."""
    lib_ = wbc.library()
    for name in ONE:
        getattr(lib_, 'wbh_libm_' + name).argtypes = [_dp, _dp, _i]
    for name in TWO + ('sincos',):
        getattr(lib_, 'wbh_libm_' + name).argtypes = [_dp, _dp, _dp, _i]
    lib_.wbh_libm_scalar.argtypes = [_i, _dp, _dp, _dp, _i]
    return lib_


def _in(x):
    return np.ascontiguousarray(x, dtype=np.float64).ravel()


def _p(a):
    return a.ctypes.data_as(_dp)


def call(name, a, b=None):
    """wbc_libm_<name> over a (and b), a new array of a's shape."""
    shape = np.shape(a)
    a = _in(a)
    y = np.empty_like(a)
    if name in TWO:
        if b is None:
            raise ValueError('%s takes two arrays' % name)
        b = _in(np.broadcast_to(b, shape))
        getattr(lib(), 'wbh_libm_' + name)(_p(a), _p(b), _p(y), a.size)
    else:
        getattr(lib(), 'wbh_libm_' + name)(_p(a), _p(y), a.size)
    return y.reshape(shape)


def sincos(x):
    """wbc_libm_sincos: (sin x, cos x) from one reduction a lane."""
    shape = np.shape(x)
    x = _in(x)
    s, c = np.empty_like(x), np.empty_like(x)
    lib().wbh_libm_sincos(_p(x), _p(s), _p(c), x.size)
    return s.reshape(shape), c.reshape(shape)


def scalar(name, a, b=None):
    """The scalar function (wbc_sin, ...) of each element, a call at a time."""
    shape = np.shape(a)
    a = _in(a)
    b = a if b is None else _in(np.broadcast_to(b, shape))
    y = np.empty_like(a)
    lib().wbh_libm_scalar(SCALAR.index(name), _p(a), _p(b), _p(y), a.size)
    return y.reshape(shape)
