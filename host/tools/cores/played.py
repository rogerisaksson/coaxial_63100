"""A move of hers on her boards in C (wbc_sil_played): its step and its joints told in one call.

    p['pass'] = played.laid(g, world, p)    # as `joints.played` begins a move: its struct, the pass, its senses
    done = played.passed(p['pass'], t, dt, senses)

`machine.process.joints.played`'s move pass - the move's step on her angles in degrees (`tools.cores.bedexit`, `scoot`,
`bedentry`), then its joints' setpoints in radians and her clamps through her boards' drive (`edge.bound`'s) -
numpy's degrees, radians and takes operation for operation (test_tick): rising off the bed 47 of its 79 us a pass
were two Python-wrapped C calls and the arrays between them.
"""
import ctypes
import functools

import numpy as np

from tools.cores import wbc

_d, _i, _vp = ctypes.c_double, ctypes.c_int, ctypes.c_void_p
#: The struct's field each move's tables go in, by the move's module (`joints.MOVES`).
FIELDS = {'tools.cores.bedexit': 'exit', 'tools.cores.scoot': 'scoot', 'tools.cores.bedentry': 'entry'}


class Played(ctypes.Structure):

    """wbc_played_t: a move playing on her boards, pointing at her edge, her clamps and the move's own arrays."""

    _fields_ = [('exit', _vp), ('scoot', _vp), ('entry', _vp), ('state', _vp), ('cmd', _vp), ('y', _vp), ('q_at', _i),
                ('nq', _i), ('n', _i), ('idx', _vp), ('clamp', _vp), ('feed', _vp), ('kp', _vp), ('target', _vp),
                ('limit', _vp), ('world', _vp), ('q_deg', _d * 64)]


@functools.lru_cache(maxsize=None)
def typed():
    """The library's move passes typed (the told's and its step's alone), wbc_played_t's size the C's."""
    lib = wbc.library()
    lib.wbh_played_size.restype = _i
    if lib.wbh_played_size() != ctypes.sizeof(Played):
        raise ValueError('wbc_played_t is %d bytes, its mirror %d' % (lib.wbh_played_size(), ctypes.sizeof(Played)))
    for fn in (lib.wbh_sil_played, lib.wbh_played_step):
        fn.argtypes, fn.restype = [_vp, _d, _d, _vp], _d
    return lib


def laid(g, world, p):
    """(its address, the pass, the struct, its senses' buffer and address) for the move `p` plays (`joints.played`):
    her edge's angles, her clamps, the move's state, command, joints, feed, stiffness and buffers, her world's boards;
    None where its move is not one of her C moves or her boards are not laid."""
    from machine.senses import edge
    field = FIELDS.get(getattr(p['move'], '__name__', ''))
    if field is None or getattr(world, 'limb_buses', None) is None:
        return None
    lib, tables = typed(), p['move']._laid()[1]
    t = Played(state=wbc.pointer(p['state']), cmd=wbc.pointer(p['cmd']), y=wbc.pointer(g['sense']), q_at=edge.Q.start,
               nq=edge.Q.stop - edge.Q.start, n=len(p['joints']), idx=wbc.pointer(p['joints'], np.int32),
               clamp=wbc.pointer(g['clamp']), feed=wbc.pointer(p['feed']), kp=wbc.pointer(p['kp']),
               target=wbc.pointer(p['target']), limit=wbc.pointer(p['limit']), world=ctypes.addressof(world.passing))
    setattr(t, field, ctypes.addressof(tables))
    senses = np.zeros(8)
    return ctypes.addressof(t), lib.wbh_sil_played, t, senses, wbc.address(senses), tables


def passed(laid_, t, dt, senses):
    """The move `dt` s on at `t` on its `senses` (its SENSED's values, in order): its done, its joints told where not."""
    at, fn, _t, buf, buf_at, _tables = laid_
    buf[:len(senses)] = senses
    return fn(at, t, dt, buf_at)
