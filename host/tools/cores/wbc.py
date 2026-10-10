"""wbc/ through ctypes: her poses, Jacobians, mass matrix and bias on the static model.

The chain the firmware runs, on the arrays `tools.cores.model` writes; one library a process.
"""
import ctypes
import functools
import os

import numpy as np

from tools import REPO
from tools.cores.build import cached

SOURCES = [os.path.join(REPO, 'wbc', 'test', 'harness.c'),
           os.path.join(REPO, 'wbc', 'src', 'wbc_body.c'),
           os.path.join(REPO, 'wbc', 'src', 'wbc_model.c')]
INCLUDES = [os.path.join(REPO, 'wbc', 'inc')]
#: The firmware compiles the chain at -O2 (CMakeLists.txt).
FLAGS = ('-O2',)


def typed(lib):
    """`lib`'s harness calls typed; `lib` back."""
    d, i = ctypes.c_double, ctypes.c_int
    dp = ctypes.POINTER(d)
    lib.wbh_links.restype = i
    lib.wbh_n.restype = i
    lib.wbh_name.restype = ctypes.c_char_p
    lib.wbh_name.argtypes = [i]
    lib.wbh_parent.restype = i
    lib.wbh_parent.argtypes = [i]
    lib.wbh_pose.argtypes = [dp, dp, dp]
    lib.wbh_mass.argtypes = [dp]
    lib.wbh_bias.argtypes = [dp, dp]
    lib.wbh_point.argtypes = [i, dp, dp]
    lib.wbh_jacobian.argtypes = [i, dp, dp]
    lib.wbh_seconds.restype = d
    lib.wbh_seconds.argtypes = [dp, dp, dp, i, i, i]
    return lib


@functools.lru_cache(maxsize=None)
def library():
    """The core built once a process for its sources' bytes."""
    return typed(ctypes.CDLL(cached(SOURCES, INCLUDES, 'wbc', FLAGS)[0]))


def _arr(a):
    return np.ascontiguousarray(a, dtype=np.float64)


def _ptr(a):
    return a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))


class Core:

    """The chain behind its harness: one body, posed, then asked."""

    def __init__(self, lib=None):
        self.lib = lib or library()
        self.links, self.n = int(self.lib.wbh_links()), int(self.lib.wbh_n())
        self.names = [self.lib.wbh_name(k).decode() for k in range(self.links)]
        self.parent = [int(self.lib.wbh_parent(k)) for k in range(self.links)]

    def pose(self, base, q):
        """Every link's frame in the world, (links, 12): R by rows, then p. `base` the pelvis's
        the same way, `q` the hinges' rad."""
        base, q, out = _arr(base), _arr(q), np.zeros((self.links, 12))
        self.lib.wbh_pose(_ptr(base), _ptr(q), _ptr(out))
        return out

    def mass(self):
        """The mass matrix at the pose, (n, n)."""
        out = np.zeros((self.n, self.n))
        self.lib.wbh_mass(_ptr(out))
        return out

    def bias(self, u):
        """C u + g at the pose, (n,); u the pelvis's twist (w, v) in its frame, then rad/s."""
        u, out = _arr(u), np.zeros(self.n)
        self.lib.wbh_bias(_ptr(u), _ptr(out))
        return out

    def point(self, link, r):
        """A point r of `link` in the world."""
        r, out = _arr(r), np.zeros(3)
        self.lib.wbh_point(link, _ptr(r), _ptr(out))
        return out

    def jacobian(self, link, r):
        """The point's Jacobian over u in the world, (6, n): rows w, then v."""
        r, out = _arr(r), np.zeros((6, self.n))
        self.lib.wbh_jacobian(link, _ptr(r), _ptr(out))
        return out

    def seconds(self, base, q, u, a, b, reps=1000):
        """Seconds a step on this host: the pose, M, the bias, links a's and b's Jacobians."""
        base, q, u = _arr(base), _arr(q), _arr(u)
        return float(self.lib.wbh_seconds(_ptr(base), _ptr(q), _ptr(u), a, b, reps))
