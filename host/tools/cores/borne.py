"""What bears her off her soles in C through ctypes (wbc_borne.c), `borne.read`'s bit for bit.

    read = borne.laid(b)                                     # her law's body, laid once
    prop, contacts, palms = read(b, s, y, weight, hands, let_go)        # as borne.read gives them
    prop, contacts, palms = read(b, s, y, weight, hands, let_go, dt, g) # and her seat's (`emancipation.seated`)
    supports = borne.supported(read)                         # emancipation.supports on its contacts as last read

Her skin's bearing points walked in C - each one's push turned to the world, its room along the floor from its
capsules, MuJoCo's own Jacobian and its drift at it (mj_jac, mj_jacDot by their pointers), numpy's OpenBLAS for
every product -, her contacts handed back as her python lays them: lying in bed the python's 192 us a law pass. Given
her law's `g`, the seat her text tells bears her as `emancipation.seated` lays it, on the read's contacts before any
is a dict: its contacts out, its cone's four corners in (rising from the bed, the python's 173 us a law pass).
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

    _fields_ = ([(name, _i) for name in ('skin', 'above', 'n', 'nv', 'touch_at', 'at_at', 'give_at', 'nrm_at', 'pelvis',
                                         'carpal', 'let', 'role_spare')]
                + [('sole', _i * 2)] + [(name, _i * SKIN) for name in ('ids', 'order', 'hand', 'right', 'caps')]
                + [('cap_r', (_d * CAPS) * SKIN), ('cap_a', ((_d * 3) * CAPS) * SKIN), ('cap_b', ((_d * 3) * CAPS) * SKIN)]
                + [(name, _d) for name in ('bears_n', 'seat_x')]
                + [(name, _vp) for name in ('xmat', 'xpos', 'm', 'd', 'jac', 'jac_dot', 'blas')]
                + [('nc', _i), ('spare', _i), ('up', _d), ('low', _d * 3), ('high', _d * 3), ('palms', _d * 2)]
                + [(name, (_d * 3) * CONTACTS) for name in ('p', 'f', 'dJv')] + [('J', ((_d * DOF) * 3) * CONTACTS)]
                + [(name, _d * CONTACTS) for name in ('carpal_of', 'room', 'give')]
                + [(name, _i * CONTACTS) for name in ('side', 'patch', 'body')]
                + [(name, _d * (3 * DOF)) for name in ('jp', 'jr')]
                + [(name, _i) for name in ('seat_skin', 'seat_body', 'pelvis_body', 'seat_spare')]
                + [(name, _d * 2) for name in ('seat_dx', 'seat_dz')] + [(name, _d) for name in ('touch_m', 'reach_m', 'shed0')]
                + [('corner', _i * CONTACTS)] + [(name, _d) for name in ('seat_on', 'seat_near')]
                + [('nrm', (_d * 3) * CONTACTS)])


@functools.lru_cache(maxsize=None)
def _typed():
    """The read typed, wbc_borne_t's size the C's."""
    lib = wbc.library()
    lib.wbh_borne_size.restype = ctypes.c_int
    if lib.wbh_borne_size() != ctypes.sizeof(Borne):
        raise ValueError('wbc_borne_t is %d bytes, its mirror %d' % (lib.wbh_borne_size(), ctypes.sizeof(Borne)))
    lib.wbh_borne_read.argtypes = [ctypes.POINTER(Borne), _vp, _vp, _d, _vp]
    lib.wbh_borne_read.restype = ctypes.c_int
    lib.wbh_borne_seated.argtypes = [ctypes.POINTER(Borne), _vp, _vp, _i, _i, _d, _d, _d, _d, _vp, _vp]
    lib.wbh_borne_seated.restype = ctypes.c_int
    return lib.wbh_borne_read, lib.wbh_borne_seated


def laid(b):
    """`borne.read` in C on her law's body `b` (`wbc.Body`), laid once - its skin, capsules and constants, her model's
    posed arrays, MuJoCo's Jacobians and numpy's OpenBLAS -: a read of borne.read's arguments, its result."""
    from machine.body import figure
    from machine.errors import MachineError
    from machine.law import modal
    from machine.moves import cascade, emancipation, leap
    from machine.senses import borne, edge, observer, touch
    from tools.cores import held, trace_mj
    if len(edge.SKIN) > SKIN or max(len(c) for c in borne.CAPSULES) > CAPS or b.m.nv > DOF:
        raise ValueError('her skin, a capsule list or her model past wbc_borne.h')
    t = Borne(skin=len(edge.SKIN), above=len(borne.ABOVE), n=b.n, nv=b.m.nv, touch_at=edge.TOUCH.start,
              at_at=edge.AT.start, give_at=edge.GIVE.start, nrm_at=edge.NORMAL.start, pelvis=borne.PELVIS, carpal=borne.CARPAL, let=borne.LET,
              bears_n=borne.BEARS_N, seat_x=figure.SEAT_X)
    t.sole[:] = [touch.SOLES[k][0] for k in (0, 1)]
    ids, right = borne.laid(b).tolist(), set(borne.HANDS[len(touch.ARMS[0]):].tolist())
    for k, caps in enumerate(borne.CAPSULES):
        t.ids[k], t.hand[k], t.right[k], t.caps[k] = ids[k], k in borne.HANDS, k in right, len(caps)
        for c, (r, a, e) in enumerate(caps):
            t.cap_r[k][c], t.cap_a[k][c][:], t.cap_b[k][c][:] = r, a.tolist(), e.tolist()
    t.order[:t.above] = borne.ABOVE.tolist()
    lib, blas = trace_mj._library(), held.blas()
    t.xmat, t.xpos, t.m, t.d = wbc.address(b.d.xmat), wbc.address(b.d.xpos), b.m._address, b.d._address
    t.jac, t.jac_dot = (ctypes.cast(getattr(lib, name), _vp).value for name in ('mj_jac', 'mj_jacDot'))
    t.blas = ctypes.addressof(blas)
    (row,) = emancipation.SEAT_ROWS
    seg = emancipation.SEAT_SEG[row]
    t.seat_skin, t.seat_body, t.pelvis_body = edge.SKIN.index(seg), b.m.body(seg).id, b.pelvis
    t.seat_dx[:], t.seat_dz[:] = (emancipation.SEAT_BOX[row][k].tolist() for k in (0, 1))
    t.touch_m, t.reach_m, t.shed0 = leap.TOUCH_M, observer.REACH_M, cascade.SHED[0]
    (step, sat), at, n = _typed(), ctypes.byref(t), b.n
    views = {name: np.ctypeslib.as_array(getattr(t, name)) for name in
             ('p', 'f', 'nrm', 'dJv', 'J', 'carpal_of', 'room', 'give', 'side', 'patch', 'body', 'low', 'high', 'palms',
              'corner')}
    kept: dict = {}

    def read(_b, s, y, weight, hands=0.0, let_go=(0.0, 0.0), g=None):
        go = np.ascontiguousarray(let_go, dtype=np.float64)
        v = np.ascontiguousarray(s['v'], dtype=np.float64)
        if step(at, wbc.address(y), wbc.address(v), hands, wbc.address(go)) < 0:
            raise MachineError('her skin bears her at more than %d points (wbc_borne.h)' % CONTACTS)
        if g is not None:
            obs = g['observer']
            rows = kept.get(id(obs['row']))
            if rows is None:
                kept.clear()
                rows = kept[id(obs['row'])] = (obs['row'], np.ascontiguousarray(obs['row'], dtype=np.int64))
            if sat(at, wbc.address(obs['points']), wbc.address(rows[1]), len(rows[1]), row, float(g['told'][row]),
                   float(g['fr'][modal.FLOOR]), float(g.get('emancipated', np.zeros(2)).max()), float(g['sat']),
                   wbc.address(g['seat_f']), wbc.address(v)) < 0:
                raise MachineError('her skin and her seat bear her at more than %d points (wbc_borne.h)' % CONTACTS)
            g['seat_on'], g['seat_near'] = t.seat_on, t.seat_near
        w = views
        contacts = [{'p': w['p'][c].copy(), 'f': w['f'][c].copy(), 'n': w['nrm'][c].copy(), 'J': w['J'][c, :, :n].copy(),
                     'dJv': w['dJv'][c].copy(), 'carpal': float(w['carpal_of'][c]), 'side': int(w['side'][c]),
                     'patch': int(w['patch'][c]), 'room': float(w['room'][c]), 'body': int(w['body'][c]),
                     **({'corner': int(w['corner'][c])} if w['corner'][c] >= 0 else {}),
                     'give': float(w['give'][c])} for c in range(t.nc)]
        low, high = w['low'].copy(), w['high'].copy()
        held_ = np.isfinite(low[0])
        return (t.up / weight, np.where(held_, low, 0.0), np.where(held_, high, 0.0)), contacts, w['palms'] / weight
    read.kept = (t, b, blas)     # what its pointers point at, alive while it is
    return read


@functools.lru_cache(maxsize=None)
def _supporting():
    """Her supports' C typed (wbc_borne_supports)."""
    lib = wbc.library()
    lib.wbh_borne_supports.argtypes = [ctypes.POINTER(Borne), _i, _vp, _vp, _vp, _vp, _i, _vp, _i, _vp, _vp, _d, _i, _vp,
                                       _vp, _vp]
    lib.wbh_borne_supports.restype = _i
    return lib.wbh_borne_supports


def supported(read):
    """`emancipation.supports` in C (wbc_borne_supports) on the contacts `read` (`laid`'s) last read - her law reads
    them each pass before her legs' release, its seat's cone among them -: her observer's points told her python's
    (`emancipation.told_on`, its C where laid), the rest copied out in numpy's order into arrays of their own a call.
    Its arrays built in numpy, her legs' release took 47 us a law pass seated of it."""
    from machine.errors import MachineError
    from machine.moves import emancipation, leap
    t, fn, rooms = read.kept[0], _supporting(), np.ascontiguousarray(emancipation.ROOMS, dtype=np.float64)
    at, cap = ctypes.byref(t), CONTACTS + len(rooms) + 64

    def supports(s, obs, told, floor, legs, loads, on=None):
        top = max(floor, float(s.get('prop', (0.0, 0.0, (0.0, 0.0, 0.0)))[2][1]), float(s['com'][1]) - 0.35)
        said = np.ascontiguousarray(emancipation.told_on(obs, told, top, on), dtype=np.bool_)
        (c0, c1) = (np.ascontiguousarray(f['pts'], dtype=np.float64) for f in s['feet'])
        pts, room, soles = np.empty((cap, 2)), np.empty(cap), np.empty(2 * cap)
        loads_, legs_ = np.ascontiguousarray(loads, dtype=np.float64), np.ascontiguousarray(legs, dtype=np.float64)
        n = fn(at, len(said), wbc.pointer(said, np.bool_), wbc.pointer(obs['points']), wbc.pointer(rooms),
               wbc.pointer(c0), len(c0), wbc.pointer(c1), len(c1), wbc.pointer(loads_), wbc.pointer(legs_),
               leap.TOUCH_N, cap, wbc.pointer(pts), wbc.pointer(room), wbc.pointer(soles))
        if n < 0:
            raise MachineError('her supports past %d points (tools.cores.borne.supported)' % cap)
        return pts[:n], room[:n], soles[:2 * n].reshape(2, n)
    supports.kept = (read, rooms)
    return supports
