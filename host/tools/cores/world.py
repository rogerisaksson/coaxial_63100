"""Her world's steps in C (wbc_world.c): her drives, meters, DC link and switch about MuJoCo's step.

    at, rated, before, after, paired, s, kept = world.laid(w)   # on her world `w`, anew where `w.reset` lays her link

`machine.world.physics.World.advance`'s steps where nothing else is asked of them (`machine.world.steps`), the same
arithmetic in the same order (test_tick): between MuJoCo's steps her world's three C calls, its link's compiled part
and its room's contacts read in Python were 37 us a pass of the door errand's.
"""
import ctypes
import functools

import numpy as np

from tools.cores import motion, wbc

_d, _i, _vp = ctypes.c_double, ctypes.c_int, ctypes.c_void_p


class Steps(ctypes.Structure):

    """wbc_steps_t: her world's steps, pointing at her pass's struct, MuJoCo's data and her arrays."""

    _fields_ = ([(name, _vp) for name in ('pass_', 'd', 'blas', 'target', 'rate', 'was', 'link_v', 'link_peak',
                                           'link_bled', 'link_id', 'link_kt', 'link_ohm', 'link_amps', 'link_loss',
                                           'switch_geom', 'hands')]
                + [(name, _d) for name in ('house_w', 'onset_v', 'clamp_v', 'link_f', 'pack_v', 'pack_ohm', 'charge_a',
                                           'step_s', 'switch_s', 'switched')]
                + [('lamp', _i), ('spare', _i)])


@functools.lru_cache(maxsize=None)
def typed():
    """The library's steps typed, wbc_steps_t's size the C's."""
    lib = wbc.library()
    lib.wbh_steps_size.restype = _i
    if lib.wbh_steps_size() != ctypes.sizeof(Steps):
        raise ValueError('wbc_steps_t is %d bytes, its mirror %d' % (lib.wbh_steps_size(), ctypes.sizeof(Steps)))
    for name, args, back in (('rated', [_vp, _d], None), ('before', [_vp, _d], None), ('after', [_vp], None),
                             ('paired', [_vp, _vp, _vp, _d], _i)):
        fn = getattr(lib, 'wbh_world_' + name)
        fn.argtypes, fn.restype = args, back
    return lib


def laid(w):
    """(its address, rated, before, after, paired - each a call of its address -, the struct, what it points at) on her
    world `w`: its pass's struct (`w.passing`), MuJoCo's data, her setpoints, her DC link (`w.link`, laid anew by
    `w.reset`), her room's switch where her room is laid (`room.switch`)."""
    from machine.body import dclink, drives
    from machine.world import physics, room
    lib, link, point = typed(), w.link, wbc.pointer
    masks = room.switch(w) if physics.ROOM else (None, None)
    s = Steps(pass_=ctypes.addressof(w.passing), d=w.data._address, blas=ctypes.addressof(motion.numpy_blas()),
              target=point(w.target), rate=point(w.rate), was=point(w.was),
              **{'link_' + k: point(link[k]) for k in ('v', 'peak', 'bled', 'id', 'kt', 'ohm', 'amps', 'loss')},
              switch_geom=None if masks[0] is None else point(masks[0], np.bool_),
              hands=None if masks[1] is None else point(masks[1], np.bool_),
              house_w=link['house_w'], onset_v=dclink.ONSET_V, clamp_v=dclink.CLAMP_V, link_f=dclink.LINK_F,
              pack_v=drives.PACK_V, pack_ohm=dclink.PACK_OHM, charge_a=dclink.CHARGE_A, step_s=physics.STEP_S,
              switch_s=room.SWITCH_S, switched=getattr(w, 'switched', room.SWITCH_S), lamp=int(getattr(w, 'lamp', False)))
    at = ctypes.addressof(s)
    rated, before, after, paired = (functools.partial(getattr(lib, 'wbh_world_' + k), at)
                                    for k in ('rated', 'before', 'after', 'paired'))
    return at, rated, before, after, paired, s, (link, masks)
