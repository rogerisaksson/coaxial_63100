"""The robot's own elementary functions as its code calls them: every machine's bits, a call as math's.

    from machine import libm
    y = libm.sin(x)        # cos, tan, exp, expm1, log, arctan, arcsin, arccos: numpy's names, a float or an array
    s, c = libm.sincos(x)  # both of one reduction: a pair for less than the platform's two calls
    a = libm.arctan2(y, x), libm.hypot(x, y)

The platform's libm rounds its last bit its own way (Windows' UCRT and Linux's glibc part in every function but
hypot: tests/test_platform.py), so what the robot computes with it parts between machines. These are wbc_libm.h's
bits, each call the cheapest way to them: a number inlined in C and an array of float64 its run - a few a call
each, more in vector lanes - through an extension of this interpreter (`tools.cores.libm_quick`: a float 0.05 us
where math's is 0.06, an array of three 0.2 where numpy's is 0.5); anything else - an array strided or of another
type, a number with an array, two arguments to broadcast - laid as C takes it first (`_other`). Where no compiler builds the
extension, the numpy twin (`machine.libm_twin`: the same bits, 81 us a call). A float in, a float out; an array in,
a new float64 array of its shape.
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
    one shape -, then C's again."""
    fn, laid = getattr(_QUICK, name), [np.asarray(a, dtype=np.float64) for a in args]
    shape = laid[0].shape if len(laid) == 1 else np.broadcast_shapes(*[a.shape for a in laid])
    if not shape:
        return fn(*[float(a) for a in laid])
    return fn(*[np.ascontiguousarray(np.broadcast_to(a, shape)) for a in laid])


if _QUICK is not None:
    _QUICK.laid(_other)
(sin, cos, tan, exp, expm1, log, arctan, arcsin, arccos, sincos, arctan2, hypot) = (
    getattr(_QUICK if _QUICK is not None else libm_twin, name) for name in NAMES)
