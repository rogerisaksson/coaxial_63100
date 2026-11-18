"""The robot's own elementary functions as its code calls them: every machine's bits, a call as math's.

    from machine import libm
    y = libm.sin(x)        # cos, tan, exp, expm1, log, arctan, arcsin, arccos, tanh: numpy's names, a float or an array
    s, c = libm.sincos(x)  # both of one reduction: a pair for less than the platform's two calls
    a = libm.arctan2(y, x), libm.hypot(x, y), libm.power(x, y)   # x^y: 10 ** (db / 20) is libm.power(10.0, db / 20)
    x2, x3, x4 = libm.sq(x), libm.cube(x), libm.quart(x)   # a whole power as its products, never `**`

The platform's libm rounds its last bit its own way (Windows' UCRT and Linux's glibc part in every function but
hypot: tests/test_platform.py), so what the robot computes with it parts between machines. These are wbc_libm.h's
bits, each call the cheapest way to them: a number inlined in C and an array of float64 its run - a few a call
each, more in vector lanes - through an extension of this interpreter (`tools.cores.libm_quick`: a float 0.05 us
where math's is 0.06, an array of three 0.2 where numpy's is 0.5); anything else - an array strided or of another
type, a number with an array, two arguments to broadcast - laid as C takes it first (`_other`). Where no compiler builds the
extension, the numpy twin (`machine.libm_twin`: the same bits, 81 us a call). A float in, a float out; an array in,
a new float64 array of its shape. A power is the platform's pow wherever a float or a numpy scalar takes `**` -
`x ** 2` there is not `x * x` once in 2000 -, so the code writes a whole power as its products (`sq`, `cube`,
`quart`: floats, arrays and traced arrays alike, wbc_sq's rounding).
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import libm_twin
from tools.cores import libm_quick

#: The functions, numpy's names.
NAMES = libm_quick.ONE + libm_quick.PAIR + libm_quick.TWO
_QUICK = libm_quick.module()


def _other(name, *args):
    """`name` of what C does not take as it is: numbers as floats, arrays as float64 in C's order - two broadcast to
    one shape -, then C's again; an array the robot's tracer follows (`tools.cores.trace`'s) its own statement."""
    for a in args:
        own = getattr(a, 'own', None)
        if own is not None:
            return own(name, *args)
    if any(np.iscomplexobj(a) for a in args):
        raise TypeError('%s of a complex number: the libm takes reals alone' % name)
    fn, laid = getattr(_QUICK, name), [np.asarray(a, dtype=np.float64) for a in args]
    shape = laid[0].shape if len(laid) == 1 else np.broadcast_shapes(*[a.shape for a in laid])
    if not shape:
        return fn(*[float(a) for a in laid])
    return fn(*[np.ascontiguousarray(np.broadcast_to(a, shape)) for a in laid])


def sq(x):
    """x x (wbc_sq)."""
    return x * x


def cube(x):
    """(x x) x (wbc_cube)."""
    return x * x * x


def quart(x):
    """(x x)(x x) (wbc_quart)."""
    s = x * x
    return s * s


if _QUICK is not None:
    _QUICK.laid(_other)
(sin, cos, tan, exp, expm1, log, arctan, arcsin, arccos, tanh, sincos, arctan2, hypot, power) = (
    getattr(_QUICK if _QUICK is not None else libm_twin, name) for name in NAMES)
