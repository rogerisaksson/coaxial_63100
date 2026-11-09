"""Her world's steps in C (wbc_world.c): drives, gearboxes, meters, link, switch about MuJoCo's step.

    at, rated, before, after, paired, s, kept = world.laid(w)   # on her world `w`, anew where `w.reset` lays her link
    loop, data = world.numpy_tanh()                             # numpy's own float64 tanh loop, for C to call

`machine.world.physics.World.advance`'s steps where nothing else is asked of them (`machine.world.steps`), the same
arithmetic in the same order (test_tick): between MuJoCo's steps her world's three C calls, its link's compiled part
and its room's contacts read in Python were 37 us a pass of the door errand's; her ankles' leaves made her world's
steps python's whole, her gearboxes' numpy 279 us of a walk's pass, now theirs in C.
"""
import ctypes
import functools

import numpy as np

from tools.cores import motion, wbc

_d, _i, _vp = ctypes.c_double, ctypes.c_int, ctypes.c_void_p
#: A ufunc's inner loop (numpy's PyUFuncGenericFunction): its arguments, its length, their strides, its data.
_LOOP = ctypes.CFUNCTYPE(None, _vp, _vp, _vp, _vp)


class _Ufunc(ctypes.Structure):

    """The head of numpy's PyUFuncObject (numpy/ufuncobject.h) past the object's own: its loops, their data, their
    types (nargs a loop) - its public fields, in their order."""

    _fields_ = [('head', ctypes.c_byte * object.__basicsize__), ('nin', _i), ('nout', _i), ('nargs', _i),
                ('identity', _i), ('functions', ctypes.POINTER(_vp)), ('data', ctypes.POINTER(_vp)), ('ntypes', _i),
                ('reserved1', _i), ('name', ctypes.c_char_p), ('types', ctypes.POINTER(ctypes.c_char))]


class Boxes(ctypes.Structure):

    """wbc_boxes_t: her gearboxes, pointing at `gearbox.Boxes`'s arrays and her world's braking."""

    _fields_ = ([(name, _vp) for name in ('on', 'k', 'c', 'j', 'gap', 'shock', 'mesh', 'observe', 'track', 'peak',
                                           'damping', 'delta', 'w', 'hat', 'p_int', 'drive', 'asked', 'rate',
                                           'q_step', 'went', 'tanh', 'tanh_data')]
                + [(name, _d) for name in ('ahead_wind', 'ahead_play', 'play_nm', 'mesh_rad_s', 'rate_s')]
                + [('substeps', _i), ('spare', _i)])


class Steps(ctypes.Structure):

    """wbc_steps_t: her world's steps, pointing at her pass's struct, MuJoCo's data and her arrays."""

    _fields_ = ([(name, _vp) for name in ('pass_', 'd', 'blas', 'target', 'rate', 'was', 'link_v', 'link_peak',
                                           'link_bled', 'link_id', 'link_kt', 'link_ohm', 'link_amps', 'link_loss',
                                           'switch_geom', 'hands', 'boxes')]
                + [(name, _d) for name in ('house_w', 'onset_v', 'clamp_v', 'link_f', 'pack_v', 'pack_ohm', 'charge_a',
                                           'step_s', 'switch_s', 'switched')]
                + [('lamp', _i), ('spare', _i)])


@functools.lru_cache(maxsize=None)
def typed():
    """The library's steps typed, wbc_steps_t's and wbc_boxes_t's sizes the C's."""
    lib = wbc.library()
    for name, mirror in (('steps', Steps), ('boxes', Boxes)):
        size = getattr(lib, 'wbh_%s_size' % name)
        size.restype = _i
        if size() != ctypes.sizeof(mirror):
            raise ValueError('wbc_%s_t is %d bytes, its mirror %d' % (name, size(), ctypes.sizeof(mirror)))
    for name, args, back in (('rated', [_vp, _d], None), ('before', [_vp, _d], None), ('after', [_vp], None),
                             ('paired', [_vp, _vp, _vp, _d], _i)):
        fn = getattr(lib, 'wbh_world_' + name)
        fn.argtypes, fn.restype = args, back
    return lib


@functools.lru_cache(maxsize=None)
def numpy_tanh():
    """(numpy's own float64 tanh loop, its data): the loop `np.tanh` runs an array through - numpy's SIMD on this host,
    its bits not libm's in a fifth of its values -, found in its ufunc's loops by their types and checked on a probe
    to the bit: C calling it gets numpy's bits, as C calling numpy's own BLAS gets its (`motion.numpy_blas`)."""
    u, code = _Ufunc.from_address(id(np.tanh)), np.dtype(np.float64).num
    found = [k for k in range(u.ntypes) if u.nargs == 2 and u.types[2 * k] == u.types[2 * k + 1] == bytes([code])]
    if u.name != b'tanh' or not found:
        raise RuntimeError("numpy's tanh lays no float64 loop in its ufunc's head: her gearboxes in C need it")
    loop, data = u.functions[found[0]], u.data[found[0]]
    x = np.concatenate([np.linspace(-40.0, 40.0, 4097), np.geomspace(1e-300, 1e3, 4097)])
    y = np.empty_like(x)
    _LOOP(loop)(ctypes.cast((_vp * 2)(x.ctypes.data, y.ctypes.data), _vp),
                ctypes.cast((ctypes.c_ssize_t * 1)(len(x)), _vp), ctypes.cast((ctypes.c_ssize_t * 2)(8, 8), _vp), data)
    if y.tobytes() != np.tanh(x).tobytes():
        raise RuntimeError("numpy's float64 tanh loop is not np.tanh's to the bit")
    return loop, data


def boxed(w):
    """wbc_boxes_t on her world `w`'s gearboxes (`w.boxes`, their arrays stepped in place) and its braking, with what
    it points at; (None, ()) where none is stepped (`w.sprung`)."""
    from machine.body import drives, gearbox
    if not w.sprung:
        return None, ()
    bx, point = w.boxes, wbc.pointer
    on = np.ascontiguousarray(bx.sprung, np.bool_)
    loop, data = numpy_tanh()
    b = Boxes(on=point(on, np.bool_), damping=point(w.damping), tanh=loop, tanh_data=data,
              **{k: point(getattr(bx, k)) for k in ('k', 'c', 'j', 'gap', 'shock', 'mesh', 'observe', 'track', 'peak',
                                                     'delta', 'w', 'hat', 'p_int', 'drive', 'asked', 'rate', 'q_step',
                                                     'went')},
              ahead_wind=drives.AHEAD_WIND, ahead_play=drives.AHEAD_PLAY, play_nm=gearbox.PLAY_NM,
              mesh_rad_s=gearbox.MESH_RAD_S, rate_s=gearbox.RATE_S, substeps=gearbox.SUBSTEPS)
    return b, (on,)


def laid(w):
    """(its address, rated, before, after, paired - each a call of its address -, the struct, what it points at) on her
    world `w`: its pass's struct (`w.passing`), MuJoCo's data, her setpoints, her DC link (`w.link`, laid anew by
    `w.reset`), her room's switch where her room is laid (`room.switch`), her gearboxes where any is stepped
    (`boxed`)."""
    from machine.body import dclink, drives
    from machine.world import physics, room
    lib, link, point = typed(), w.link, wbc.pointer
    masks = room.switch(w) if physics.ROOM else (None, None)
    boxes, held = boxed(w)
    s = Steps(pass_=ctypes.addressof(w.passing), d=w.data._address, blas=ctypes.addressof(motion.numpy_blas()),
              target=point(w.target), rate=point(w.rate), was=point(w.was),
              **{'link_' + k: point(link[k]) for k in ('v', 'peak', 'bled', 'id', 'kt', 'ohm', 'amps', 'loss')},
              switch_geom=None if masks[0] is None else point(masks[0], np.bool_),
              hands=None if masks[1] is None else point(masks[1], np.bool_),
              boxes=None if boxes is None else ctypes.addressof(boxes),
              house_w=link['house_w'], onset_v=dclink.ONSET_V, clamp_v=dclink.CLAMP_V, link_f=dclink.LINK_F,
              pack_v=drives.PACK_V, pack_ohm=dclink.PACK_OHM, charge_a=dclink.CHARGE_A, step_s=physics.STEP_S,
              switch_s=room.SWITCH_S, switched=getattr(w, 'switched', room.SWITCH_S), lamp=int(getattr(w, 'lamp', False)))
    at = ctypes.addressof(s)
    rated, before, after, paired = (functools.partial(getattr(lib, 'wbh_world_' + k), at)
                                    for k in ('rated', 'before', 'after', 'paired'))
    return at, rated, before, after, paired, s, (link, masks, boxes, held)
