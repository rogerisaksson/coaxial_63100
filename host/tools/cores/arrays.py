"""Where numpy keeps an ndarray's data pointer in its object, found once a process."""
import ctypes
import numpy as np


def data_at():
    """Where in an ndarray's object its data pointer is, bytes: numpy's PyArrayObject, after the head."""
    probes = [np.empty(k + 1) for k in range(3)]
    for at in range(8, 80, 8):
        if all(ctypes.c_void_p.from_address(id(a) + at).value == a.ctypes.data for a in probes):
            return at
    raise RuntimeError("numpy's array head not found")
