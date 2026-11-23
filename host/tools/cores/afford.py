"""What the robot's things afford it, told anew in C as a look moves them (wbc_afford.c), bit for bit.

    near = afford.near(now, was, told, cell, root2)         # furniture._near past its rows' identities: (n,) bool
    area = afford.clear(arrays, other, fb, lo, hi, reach, root2)    # affordance.clear on a look's arrays: m^2

A look's rows as `registry.Things` lays them (centres (n, 3), half sizes (n, 3), rotations (n, 3, 3)), numpy's own
BLAS for each product (`held.blas`), the robot's hypot; their numpy twins `furniture.near_twin` and
`affordance._covers`. A bed creeping under the robot re-told each look: its rows' nearness 110 us in numpy, 42 in C;
each top's clear 68 us, 25 in C; the bed errand's `furniture.lain` 1.6 % of its wall, 1.0 %.
"""
import ctypes
import functools

import numpy as np

from tools.cores import held, wbc

_i, _d, _vp = ctypes.c_int, ctypes.c_double, ctypes.c_void_p


@functools.lru_cache(maxsize=None)
def _typed():
    """(near, clear) typed, numpy's own BLAS beside them."""
    lib = wbc.library()
    lib.wbh_afford_near.argtypes = [_i] + [_vp] * 7 + [_d, _d, _vp]
    lib.wbh_afford_near.restype = _i
    lib.wbh_afford_clear.argtypes = [_vp, _i] + [_vp] * 5 + [_d] * 4
    lib.wbh_afford_clear.restype = _d
    return lib.wbh_afford_near, lib.wbh_afford_clear, ctypes.addressof(held.blas())


def _rows(arrays):
    """A look's (centres, half sizes, rotations) contiguous float64 - the very arrays where they are."""
    return [np.ascontiguousarray(a, dtype=np.float64) for a in arrays]


def near(now, was, told, cell, root2):
    """(n,) whether each row of `now` (centres, half sizes, rotations: (n, 3), (n, 3), (n, 9)) is told anew as it
    follows `was`: it moved - its numbers or, `told`, its word -, or a moved row's circle stands within `cell` m of
    its own either way, `root2` a rectangle's reach over its circle (`furniture.near_twin`)."""
    fn, _clear, _blas = _typed()
    (cn, hn, rn), (cw, hw, rw) = _rows(now), _rows(was)
    flags = np.ascontiguousarray(told, dtype=np.bool_)
    out = np.empty(len(flags), np.bool_)
    p = wbc.pointer
    fn(len(flags), p(cn), p(hn), p(rn), p(cw), p(hw), p(rw), p(flags, np.bool_), cell, root2, p(out, np.bool_))
    return out


def clear(arrays, other, fb, lo, hi, reach, root2):
    """The area of a box's top - its footprint `fb` (centre, axes, half sizes), its circle's radius `reach` - clear of
    the things `arrays` (centres, half sizes, rotations) each `other` marks whose bottoms lie in [lo, hi], m^2, `root2`
    a rectangle's reach over its circle (`affordance.clear` on a look's arrays)."""
    _near, fn, blas = _typed()
    c, h, r = _rows(arrays)
    marks = np.ascontiguousarray(other, dtype=np.bool_)
    box = np.ascontiguousarray(np.concatenate([np.ravel(fb[0]), np.ravel(fb[1]), np.ravel(fb[2])]), dtype=np.float64)
    p = wbc.pointer
    return float(fn(blas, len(marks), p(c), p(h), p(r), p(marks, np.bool_), p(box), float(lo), float(hi), float(reach),
                    root2))
