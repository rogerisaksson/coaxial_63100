"""machine.eyes.classes.of and wholes.grouped in C through ctypes, bit for bit on an exact host.

    told = classes.laid(lane=..., timber=..., glass=..., **heights)   # the text's heights as classes lays them
    cls, of, whole = told(centre, half, R, clear, code, flags)       # each thing's class, its told kind, its whole
    one, built = told.grouped(centre, half, R, code, flags, whole)   # the one thing each is of, the buildings

A thing's kind is passed as a code and its flags (BITS: what its look tells alone is a material, seen through,
furniture, a building's shell, an opening, a roof, a storey's, a door's or a bed's kind); its class comes back as
an index of `classes.CLASSES`, or -1 its whole's told name of the kind `of` names. The scratch the C lays its
pairs in is kept here and grown to the largest look.
"""
import ctypes
import functools

import numpy as np

from tools.cores import wbc

_d, _i, _vp = ctypes.c_double, ctypes.c_int32, ctypes.c_void_p
#: wbc_classes.h's boxes a look holds at most; a larger look stays numpy's.
MOST = 1024
#: A thing's flags (wbc_classes.h's WBC_CLASSES_*), and a told name's beside them (wbc_wholes.h's WBC_WHOLES_*).
BITS = {'raw': 1, 'see': 2, 'furn': 4, 'shell': 8, 'open': 16, 'roof': 32, 'storey': 64, 'door': 128, 'bed': 256,
        'wall': 512, 'through': 1024, 'stairs': 2048, 'notone': 4096}
#: wbc_classes_t's fields in its order: the text's heights (m), then numpy's sum of a box's extents and the kinds a
#: door's and a bed's look fall back on.
DOUBLES = ('flush', 'step', 'over', 'overhead', 'seat0', 'seat1', 'bed0', 'bed1', 'brace', 'lap', 'lumbar',
           'stance', 'thin_eps', 'cell', 'eps', 'brace_half', 'far_eps', 'half_thin_eps', 'cell_eps')
INTS = ('lane', 'timber', 'timber_flags', 'glass', 'glass_flags', 'spare')


class Classes(ctypes.Structure):

    """wbc_classes_t: the text's heights, numpy's sum, the fallback kinds."""

    _fields_ = [(name, _d) for name in DOUBLES] + [(name, _i) for name in INTS]


@functools.lru_cache(maxsize=None)
def _typed():
    """The library, its classes' entries typed, wbc_classes_t's size the C's."""
    lib = wbc.library()
    lib.wbh_classes_size.restype = ctypes.c_int
    if lib.wbh_classes_size() != ctypes.sizeof(Classes):
        raise ValueError('wbc_classes_t is %d bytes, its mirror %d' % (lib.wbh_classes_size(), ctypes.sizeof(Classes)))
    lib.wbh_classes_scratch.argtypes = [ctypes.c_int]
    lib.wbh_classes_scratch.restype = ctypes.c_longlong
    lib.wbh_classes_of.argtypes = [ctypes.POINTER(Classes), ctypes.c_int] + [_vp] * 10
    lib.wbh_classes_of.restype = ctypes.c_int
    lib.wbh_wholes_scratch.argtypes = [ctypes.c_int]
    lib.wbh_wholes_scratch.restype = ctypes.c_longlong
    lib.wbh_wholes_grouped.argtypes = [ctypes.POINTER(Classes), ctypes.c_int] + [_vp] * 12
    lib.wbh_wholes_grouped.restype = ctypes.c_int
    return lib


def laid(**fields):
    """`of` on the C laid with wbc_classes_t's `fields` (DOUBLES, INTS): told(centre, half, R, clear, code, flags) ->
    (cls, of, whole), int32 each, as `machine.eyes.classes.of` reads them; its scratch grown to the largest look."""
    lib, s = _typed(), Classes(**fields)
    fn, at, scratch = lib.wbh_classes_of, ctypes.byref(s), [np.zeros(0, np.uint8)]

    def told(centre, half, R, clear, code, flags):
        n = len(code)
        need = int(lib.wbh_classes_scratch(n))
        if len(scratch[0]) < need:
            scratch[0] = np.zeros(need, np.uint8)
        arrays = ([np.ascontiguousarray(a, np.float64) for a in (centre, half, R, clear)]
                  + [np.ascontiguousarray(a, np.int32) for a in (code, flags)])
        cls, of, whole = np.empty(n, np.int32), np.empty(n, np.int32), np.empty(n, np.int32)
        if fn(at, n, *[a.ctypes.data for a in arrays], cls.ctypes.data, of.ctypes.data, whole.ctypes.data,
              scratch[0].ctypes.data):
            raise ValueError('a look of %d things, past the %d the C holds' % (n, MOST))
        return cls, of, whole
    told.laid, told.grouped = s, _grouper(lib, s)
    return told


def _grouper(lib, s):
    """`wholes.grouped` on the C laid as `s`: grouped(centre, half, R, code, flags, whole) -> (each thing's one thing
    (n,) int32, [(whether a cabin, its low corner (3,), its high corner (3,), its members (k,))])."""
    fn, at, kept = lib.wbh_wholes_grouped, ctypes.byref(s), [np.zeros(0, np.uint8), np.zeros((0, 0), np.uint8)]

    def grouped(centre, half, R, code, flags, whole):
        n = len(code)
        need = int(lib.wbh_wholes_scratch(n))
        if len(kept[0]) < need:
            kept[0] = np.zeros(need, np.uint8)
        if kept[1].shape != (n, n):
            kept[1] = np.zeros((n, n), np.uint8)
        member = kept[1]
        arrays = ([np.ascontiguousarray(a, np.float64) for a in (centre, half, R)]
                  + [np.ascontiguousarray(a, np.int32) for a in (code, flags, whole)])
        one, cabin, blo, bhi = np.empty(n, np.int32), np.empty(n, np.int32), np.empty((n, 3)), np.empty((n, 3))
        nb = fn(at, n, *[a.ctypes.data for a in arrays], one.ctypes.data, cabin.ctypes.data, blo.ctypes.data,
                bhi.ctypes.data, member.ctypes.data, kept[0].ctypes.data)
        if nb < 0:
            raise ValueError('a look of %d things, past the %d the C holds' % (n, MOST))
        return one, [(bool(cabin[g]), blo[g].copy(), bhi[g].copy(), np.flatnonzero(member[g])) for g in range(nb)]
    return grouped
