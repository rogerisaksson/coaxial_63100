"""What bears her off her soles in C through ctypes (wbc_borne.c), `borne.read`'s bit for bit.

    read = borne.laid(b)                                     # her law's body, laid once
    prop, contacts, palms = read(b, s, y, weight, hands, let_go, dt)    # as borne.read gives them

Her skin's bearing points walked in C - each one's push turned to the world, its room along the floor from its
capsules, MuJoCo's own Jacobian and its drift at it (mj_jac, mj_jacDot by their pointers), numpy's OpenBLAS for
every product -, her contacts handed back as her python lays them: lying in bed the python's 192 us a law pass.
"""
import ctypes
import functools

import numpy as np

from tools.cores import wbc

_i, _d, _vp = ctypes.c_int, ctypes.c_double, ctypes.c_void_p
#: wbc_borne.h's most laid: skin segments, a segment's capsules, a read's contacts, degrees of freedom.
SKIN, CAPS, CONTACTS, DOF = 24, 8, 32, 64


class Borne(ctypes.Structure):

    """wbc_borne_t: the read laid on her law's model, a read's support and contacts beside it."""

    _fields_ = ([(name, _i) for name in ('skin', 'above', 'n', 'nv', 'touch_at', 'at_at', 'give_at', 'pelvis',
                                         'carpal', 'let')]
                + [('sole', _i * 2)] + [(name, _i * SKIN) for name in ('ids', 'order', 'hand', 'right', 'thigh', 'caps')]
                + [('cap_r', (_d * CAPS) * SKIN), ('cap_a', ((_d * 3) * CAPS) * SKIN), ('cap_b', ((_d * 3) * CAPS) * SKIN)]
                + [(name, _d) for name in ('bears_n', 'seat_x', 'g_down')]
                + [(name, _vp) for name in ('xmat', 'xpos', 'm', 'd', 'jac', 'jac_dot', 'blas')]
                + [('nc', _i), ('up', _d), ('low', _d * 3), ('high', _d * 3), ('palms', _d * 2)]
                + [(name, (_d * 3) * CONTACTS) for name in ('p', 'f', 'dJv')] + [('J', ((_d * DOF) * 3) * CONTACTS)]
                + [(name, _d * CONTACTS) for name in ('carpal_of', 'room', 'give')]
                + [(name, _i * CONTACTS) for name in ('side', 'patch', 'body')]
                + [(name, _d * (3 * DOF)) for name in ('jp', 'jr')])


@functools.lru_cache(maxsize=None)
def _typed():
    """The read typed, wbc_borne_t's size the C's."""
    lib = wbc.library()
    lib.wbh_borne_size.restype = ctypes.c_int
    if lib.wbh_borne_size() != ctypes.sizeof(Borne):
        raise ValueError('wbc_borne_t is %d bytes, its mirror %d' % (lib.wbh_borne_size(), ctypes.sizeof(Borne)))
    lib.wbh_borne_read.argtypes = [ctypes.POINTER(Borne), _vp, _vp, _d, _d, _vp, _d]
    lib.wbh_borne_read.restype = ctypes.c_int
    return lib.wbh_borne_read


def laid(b):
    """`borne.read` in C on her law's body `b` (`wbc.Body`), laid once - its skin, capsules and constants, her model's
    posed arrays, MuJoCo's Jacobians and numpy's OpenBLAS -: a read of borne.read's arguments, its result."""
    from machine.body import figure
    from machine.errors import MachineError
    from machine.senses import borne, edge, touch
    from tools.cores import held, trace_mj
    if len(edge.SKIN) > SKIN or max(len(c) for c in borne.CAPSULES) > CAPS or b.m.nv > DOF:
        raise ValueError('her skin, a capsule list or her model past wbc_borne.h')
    t = Borne(skin=len(edge.SKIN), above=len(borne.ABOVE), n=b.n, nv=b.m.nv, touch_at=edge.TOUCH.start,
              at_at=edge.AT.start, give_at=edge.GIVE.start, pelvis=borne.PELVIS, carpal=borne.CARPAL, let=borne.LET,
              bears_n=borne.BEARS_N, seat_x=figure.SEAT_X, g_down=float(-edge.G[1]))
    t.sole[:] = [touch.SOLES[k][0] for k in (0, 1)]
    ids, right = borne.laid(b).tolist(), set(borne.HANDS[len(touch.ARMS[0]):].tolist())
    for k, caps in enumerate(borne.CAPSULES):
        t.ids[k], t.hand[k], t.right[k], t.thigh[k], t.caps[k] = (ids[k], k in borne.HANDS, k in right,
                                                                  k in borne.THIGHS, len(caps))
        for c, (r, a, e) in enumerate(caps):
            t.cap_r[k][c], t.cap_a[k][c][:], t.cap_b[k][c][:] = r, a.tolist(), e.tolist()
    t.order[:t.above] = borne.ABOVE.tolist()
    lib, blas = trace_mj._library(), held.blas()
    t.xmat, t.xpos, t.m, t.d = wbc.address(b.d.xmat), wbc.address(b.d.xpos), b.m._address, b.d._address
    t.jac, t.jac_dot = (ctypes.cast(getattr(lib, name), _vp).value for name in ('mj_jac', 'mj_jacDot'))
    t.blas = ctypes.addressof(blas)
    step, at, n = _typed(), ctypes.byref(t), b.n
    views = {name: np.ctypeslib.as_array(getattr(t, name)) for name in
             ('p', 'f', 'dJv', 'J', 'carpal_of', 'room', 'give', 'side', 'patch', 'body', 'low', 'high', 'palms')}

    def read(_b, s, y, weight, hands=0.0, let_go=(0.0, 0.0), dt=0.0):
        go = np.ascontiguousarray(let_go, dtype=np.float64)
        v = np.ascontiguousarray(s['v'], dtype=np.float64)
        if step(at, wbc.address(y), wbc.address(v), weight, hands, wbc.address(go), dt) < 0:
            raise MachineError('her skin bears her at more than %d points (wbc_borne.h)' % CONTACTS)
        w = views
        contacts = [{'p': w['p'][c].copy(), 'f': w['f'][c].copy(), 'J': w['J'][c, :, :n].copy(),
                     'dJv': w['dJv'][c].copy(), 'carpal': float(w['carpal_of'][c]), 'side': int(w['side'][c]),
                     'patch': int(w['patch'][c]), 'room': float(w['room'][c]), 'body': int(w['body'][c]),
                     'give': float(w['give'][c])} for c in range(t.nc)]
        low, high = w['low'].copy(), w['high'].copy()
        held_ = np.isfinite(low[0])
        return (t.up / weight, np.where(held_, low, 0.0), np.where(held_, high, 0.0)), contacts, w['palms'] / weight
    read.kept = (t, b, blas)     # what its pointers point at, alive while it is
    return read
