"""An ndarray's data pointer: where numpy keeps it in its object, found once a process; read off it."""
import ctypes
import numpy as np


def data_at():
    """Where in an ndarray's object its data pointer is, bytes: numpy's PyArrayObject, after the head."""
    probes = [np.empty(k + 1) for k in range(3)]
    for at in range(8, 80, 8):
        if all(ctypes.c_void_p.from_address(id(a) + at).value == a.ctypes.data for a in probes):
            return at
    raise RuntimeError("numpy's array head not found")


#: Where an ndarray's data pointer lies in its object (`address`).
_AT = data_at()


def address(a):
    """The address of an ndarray's first element, read off its object through ctypes - 1.2 us a call, a ctypes
    object made each -: `tools.cores.wbc.address` where the quick extension is not built (`quick.addressing`)."""
    return ctypes.c_void_p.from_address(id(a) + _AT).value or 0
