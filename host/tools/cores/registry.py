"""Her map's look in C through ctypes (wbc_registry.c): `registry.passed` and `seen`, bit for bit.

    core = registry.laid(rows)          # her registry's slots' fields (`machine.eyes.registry.Rows`), laid once
    core.passed(live, movable, pos, quat, dt)
    n = core.seen(centre, half, R, ids, clear, codes, live, slot_codes, t, everywhere, pos, quat)
    core.looked, core.slot_of           # each thing looked at; the slot each of the n looked at took

Its slots' fields pointed at where the host lays them (an array a field on lines of its own), the registry's
constants, numpy's sum of each einsum of a look read off numpy's own results here (`lanes`) and numpy's OpenBLAS
for each product; a look's scratch laid in its struct once. The lodge's 64 things a look: the numpy's 760 us.
"""
import ctypes
import functools

import numpy as np

from tools.cores import orders, wbc

_i, _d, _q, _vp = ctypes.c_int, ctypes.c_double, ctypes.c_int64, ctypes.c_void_p
#: wbc_registry.h's boxes a look passes at most.
BOXES = 256
#: The slots' fields wbc_registry_t points at, in its order.
FIELDS = ('on', 'centre', 'half', 'R', 'var', 'salience', 'vel', 'last', 'seen_at', 'swell', 'drift', 'moving',
          'shown', 'clear', 'id', 'kind')
#: The draw numpy's sums are read off (`lanes`), its seed.
SEED = 5


class Registry(ctypes.Structure):

    """wbc_registry_t: the slots' fields, the constants, numpy's sums, a look's answer and scratch."""

    _fields_ = ([(name, _vp) for name in FIELDS] + [('blas', _vp), ('slots', _i), ('lanes', _i * 3)]
                + [(name, _d) for name in ('cell2', 'gate', 'hit', 'miss', 'prune', 'kept_s', 'swell_mps', 'swell_m',
                                           'diffuse', 'moving_chi2', 'near_m', 'far_m', 'tan_up', 'tan_across')]
                + [(name, _d * 3) for name in ('axis', 'up', 'eyes')] + [('corners', (_d * 3) * 8)]
                + [(name, _i * BOXES) for name in ('looked', 'slot_of')]
                + [(name, (_d * 3) * BOXES) for name in ('fc', 'fh')] + [('fR', (_d * 9) * BOXES), ('fclear', _d * BOXES)]
                + [(name, _q * BOXES) for name in ('fid', 'fcode', 'lid', 'rest')]
                + [('ahead', (_d * 3) * BOXES), ('scale', _d * BOXES)]
                + [(name, _i * BOXES) for name in ('matched', 'best', 'taken', 'inside', 'freed')]
                + [(name, (_d * 3) * BOXES) for name in ('gc', 'gh')] + [('gR', (_d * 9) * BOXES)]
                + [(name, (_d * 3) * (10 * BOXES)) for name in ('points', 'v')]
                + [(name, _d * (10 * BOXES)) for name in ('depth', 'upv')] + [('d2', _d * (BOXES * BOXES))])


@functools.lru_cache(maxsize=None)
def _typed():
    """(passed, seen) typed, wbc_registry_t's size the C's."""
    lib = wbc.library()
    lib.wbh_registry_size.restype = ctypes.c_int
    if lib.wbh_registry_size() != ctypes.sizeof(Registry):
        raise ValueError('wbc_registry_t is %d bytes, its mirror %d' % (lib.wbh_registry_size(), ctypes.sizeof(Registry)))
    at = ctypes.POINTER(Registry)
    lib.wbh_registry_passed.argtypes = [at, _vp, _i, _vp, _vp, _vp, _d]
    lib.wbh_registry_passed.restype = None
    lib.wbh_registry_seen.argtypes = [at, _i, _vp, _vp, _vp, _vp, _vp, _vp, _vp, _i, _vp, _d, _i, _vp, _vp]
    lib.wbh_registry_seen.restype = ctypes.c_int
    return lib.wbh_registry_passed, lib.wbh_registry_seen


@functools.lru_cache(maxsize=None)
def lanes():
    """numpy's sum of each einsum of a look here, as wbc_registry_t takes it (an index of `orders.ORDERS`): a box's
    corners ('nij,ncj->nci'), its nearest point's turn in ('nji,nj->ni') and out ('nij,nj->ni') - read off numpy's
    own results on a draw (`orders.summed`): AVX2's lanes or SSE's where it dispatches them, an Arm's fused."""
    rng = np.random.default_rng(SEED)
    R, X, e = rng.standard_normal((64, 3, 3)), rng.standard_normal((64, 8, 3)), rng.standard_normal((64, 3))
    forms = (('nij,ncj->nci', X, R[:, None, :, :], X[:, :, None, :]),
             ('nji,nj->ni', e, np.swapaxes(R, 1, 2), e[:, None, :]), ('nij,nj->ni', e, R, e[:, None, :]))
    got = []
    for subscripts, b, A, B in forms:
        terms, by = np.broadcast_arrays(A, B)
        got.append(orders.fitted(terms, by, np.einsum(subscripts, R, b)))
    return tuple(got)


def _flat(a, dtype: type = np.float64):
    return np.ascontiguousarray(a, dtype=dtype)


class Core:

    """A registry's look in C on its slots' fields `rows`, laid once (`laid`)."""

    def __init__(self, rows):
        from machine.eyes import registry as py
        from tools.cores import held
        t = self.t = Registry()
        for name in FIELDS:
            setattr(t, name, rows[name].ctypes.data)
        self.kept = (rows, held.blas())      # what its pointers point at, alive while it is
        t.blas, t.slots = ctypes.addressof(self.kept[1]), py.SLOTS
        t.lanes[:] = lanes()
        t.cell2, t.gate, t.hit, t.miss, t.prune = py.CELL2, py.GATE, py.HIT, py.MISS, py.PRUNE
        t.kept_s, t.swell_mps, t.swell_m, t.diffuse = py.KEPT_S, py.SWELL_MPS, py.SWELL_M, py.DIFFUSE
        t.moving_chi2, t.near_m, t.far_m, t.tan_up, t.tan_across = py.MOVING, py.NEAR_M, py.FAR_M, py.TAN_UP, py.TAN_ACROSS
        t.axis[:], t.up[:], t.eyes[:] = py.AXIS.tolist(), py.UP.tolist(), [float(e) for e in py.EYES]
        for k, corner in enumerate(py.CORNERS.tolist()):
            t.corners[k][:] = corner
        self.at, (self._passed, self._seen) = ctypes.byref(t), _typed()
        self.looked, self.slot_of = (np.ctypeslib.as_array(getattr(t, name)) for name in ('looked', 'slot_of'))

    def passed(self, live, movable, pos, quat, dt):
        """registry.passed past its slots' kinds: the live slots `live`, each `movable` or not, her head at `pos`
        turned by `quat`, `dt` s on."""
        live, movable, pos, quat = _flat(live, np.int64), _flat(movable), _flat(pos), _flat(quat)
        self._passed(self.at, live.ctypes.data, len(live), movable.ctypes.data, pos.ctypes.data, quat.ctypes.data, dt)

    def seen(self, centre, half, R, ids, clear, codes, live, slot_codes, t, everywhere, pos, quat):
        """registry.seen past its things' classes (wbc_registry_seen): those looked at, each looked at in `looked`, the
        slot each of them took in `slot_of`."""
        arrays = (_flat(centre), _flat(half), _flat(R), _flat(ids, np.int64), _flat(clear), _flat(codes, np.int64))
        live, slot_codes, pos, quat = _flat(live, np.int64), _flat(slot_codes, np.int64), _flat(pos), _flat(quat)
        n = self._seen(self.at, len(arrays[0]), *(a.ctypes.data for a in arrays), live.ctypes.data, len(live),
                       slot_codes.ctypes.data, t, int(bool(everywhere)), pos.ctypes.data, quat.ctypes.data)
        if n < 0:
            raise ValueError('a look of %d things, %d slots live: wbc_registry.h passes %d' % (len(arrays[0]), len(live),
                                                                                              BOXES))
        return n


def laid(rows):
    """`rows`' look in C (`machine.eyes.registry.Rows`): its fields pointed at, the registry's constants, numpy's
    sums and OpenBLAS laid once - a `Core`."""
    return Core(rows)
