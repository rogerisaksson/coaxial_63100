"""The robot's world in C (wbc_world.c): drives, gearboxes, meters, link, switch between MuJoCo steps.

    at, rated, before, after, paired, s, kept = world.laid(w)   # on the robot's world `w`, anew where `w.reset` lays its link
    hold = world.held(vel, of, at, lie, still, by_geom, live, n, low) # its loose things asleep held a pass: woken
    s, stuck, slipped = world.stuck(pair, at, slip, row, held, borne, grip, share, step_s, fade)   # contacts' slips
    loop, data = world.numpy_tanh()                             # numpy's own float64 tanh loop, for C to call

`machine.world.physics.World.advance`'s steps where nothing else is asked of them (`machine.world.steps`), the same
arithmetic in the same order (test_tick): between MuJoCo's steps the robot's world's three C calls, its link's
compiled part and its room's contacts read in Python were 37 us a pass of the door errand's; the robot's ankles'
leaves made its world's steps python's whole, its gearboxes' numpy 279 us of a walk's pass, now
theirs in C.
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

    """wbc_boxes_t: the robot's gearboxes, pointing at `gearbox.Boxes`'s arrays and its world's braking."""

    _fields_ = ([(name, _vp) for name in ('on', 'k', 'c', 'j', 'gap', 'shock', 'mesh', 'v_mesh', 'inv_m',
                                           'observe', 'track', 'peak', 'damping', 'delta', 'w', 'hat', 'p_int',
                                           'drive', 'asked', 'rate', 'q_step', 'went', 'tanh', 'tanh_data')]
                + [(name, _d) for name in ('ahead_wind', 'ahead_play', 'play_nm', 'mesh_rad_s', 'rate_s')]
                + [('substeps', _i), ('spare', _i)])


class Steps(ctypes.Structure):

    """wbc_steps_t: the robot's world's steps, pointing at its pass's struct, MuJoCo's data and its arrays."""

    _fields_ = ([(name, _vp) for name in ('pass_', 'd', 'blas', 'target', 'rate', 'was', 'link_v', 'link_peak',
                                           'link_bled', 'link_id', 'link_kt', 'link_ohm', 'link_amps', 'link_loss',
                                           'switch_geom', 'hands', 'boxes', 'stuck')]
                + [(name, _d) for name in ('house_w', 'onset_v', 'clamp_v', 'link_f', 'pack_v', 'pack_ohm', 'charge_a',
                                           'step_s', 'switch_s', 'switched')]
                + [('lamp', _i), ('spare', _i)])


class Held(ctypes.Structure):

    """wbc_held_t: the robot's loose things asleep as `machine.world.loose.stick` holds them a pass, pointing at their
    arrays."""

    _fields_ = ([(name, _vp) for name in ('by_geom', 'live', 'vel', 'of', 'at', 'lie', 'still', 'woken')]
                + [('low', _d)] + [(name, _i) for name in ('nvel', 'nat', 'nstill', 'nthing')])


class Stuck(ctypes.Structure):

    """wbc_stuck_t: the world's contacts' slips as `machine.world.stiction` keeps them, pointing at their arrays."""

    _fields_ = ([(name, _vp) for name in ('pair', 'at', 'slip', 'row', 'held', 'borne', 'grip', 'share')]
                + [('step_s', _d), ('fade', _d), ('cap', _i), ('n', _i)])


@functools.lru_cache(maxsize=None)
def typed():
    """The library's steps typed, wbc_steps_t's and wbc_boxes_t's sizes the C's."""
    lib = wbc.library()
    for name, mirror in (('steps', Steps), ('boxes', Boxes), ('held', Held), ('stuck', Stuck)):
        size = getattr(lib, 'wbh_%s_size' % name)
        size.restype = _i
        if size() != ctypes.sizeof(mirror):
            raise ValueError('wbc_%s_t is %d bytes, its mirror %d' % (name, size(), ctypes.sizeof(mirror)))
    for name, args, back in (('rated', [_vp, _d], None), ('drives', [_vp, _d], None), ('stuck_step', [_vp], None),
                             ('before', [_vp, _d], None), ('contacts_after', [_vp], None), ('rest_after', [_vp], None),
                             ('after', [_vp], None), ('join_before', [_vp, _d], None),
                             ('split_after', [_vp, _vp, _vp, _vp], _i),
                             ('posed', [_vp, _vp, _vp, _i, _i, _vp, _i, _vp, _i, _vp, _vp, _vp, _vp, _vp], None),
                             ('paired', [_vp, _vp, _vp, _d], _i), ('ahead', [_vp, _vp, _vp], _i),
                             ('behind', [], _i), ('held', [_vp, _vp], _i), ('stuck', [_vp, _vp], None),
                             ('slipped', [_vp, _vp], None)):
        fn = getattr(lib, 'wbh_world_' + name)
        fn.argtypes, fn.restype = args, back
    return lib


@functools.lru_cache(maxsize=None)
def _mujoco_dll():
    """MuJoCo's native shared library loaded once beside its Python package, or None."""
    import glob
    import os
    import mujoco
    folder = os.path.dirname(mujoco.__file__)
    found = glob.glob(os.path.join(folder, 'mujoco.dll')) + glob.glob(os.path.join(folder, 'libmujoco*'))
    return ctypes.CDLL(found[0]) if found else None


@functools.lru_cache(maxsize=None)
def staged():
    """(a stage begun on the thread of the robot's C's beside its law, its join, MuJoCo's `mj_step1` where
    its library holds it): the robot's step's first half for `machine.world.steps.ahead`; None where MuJoCo's
    library is not found beside its package."""
    mj_dll = _mujoco_dll()
    if mj_dll is None:
        return None
    lib = typed()
    return lib.wbh_world_ahead, lib.wbh_world_behind, ctypes.cast(mj_dll.mj_step1, _vp).value


@functools.lru_cache(maxsize=None)
def _posed_fn():
    """`(wbh_world_posed, mj_kinematics address)` or `None` when MuJoCo's library is absent."""
    mj_dll = _mujoco_dll()
    if mj_dll is None:
        return None
    return typed().wbh_world_posed, ctypes.cast(mj_dll.mj_kinematics, _vp).value


def posed(m, d, qpos, bodies, jnt):
    """Batch `mj_kinematics` and frame gather in C (`wbc_world_posed`), or `None`."""
    pair = _posed_fn()
    if pair is None:
        return None
    fn, kin = pair
    q_c = np.ascontiguousarray(qpos, dtype=np.float64)
    b_c = np.ascontiguousarray(bodies, dtype=np.int32)
    j_c = np.ascontiguousarray(jnt, dtype=np.int32)
    if (q_c.ndim != 2 or q_c.shape[1] != m.nq or b_c.ndim != 1 or j_c.ndim != 1
            or (len(b_c) and (int(b_c.min()) < 0 or int(b_c.max()) >= m.nbody))
            or (len(j_c) and (int(j_c.min()) < 0 or int(j_c.max()) >= m.njnt))):
        return None
    n, nb, nj = len(q_c), len(b_c), len(j_c)
    xpos = np.empty((n, nb, 3), dtype=np.float64)
    xmat = np.empty((n, nb, 3, 3), dtype=np.float64)
    axis = np.empty((n, nj, 3), dtype=np.float64)
    anchor = np.empty((n, nj, 3), dtype=np.float64)
    if n > 0:
        fn(kin, m._address, d._address, n, m.nq, q_c.ctypes.data, nb, b_c.ctypes.data, nj, j_c.ctypes.data,
           xpos.ctypes.data, xmat.ctypes.data, axis.ctypes.data, anchor.ctypes.data)
    return xpos, xmat, axis, anchor


@functools.lru_cache(maxsize=None)
def numpy_tanh():
    """(numpy's own float64 tanh loop, its data): the loop `np.tanh` runs an array through - numpy's SIMD on this host,
    its bits not libm's in a fifth of its values -, found in its ufunc's loops by their types and checked on a probe
    to the bit: C calling it gets numpy's bits, as C calling numpy's own BLAS gets its (`motion.numpy_blas`)."""
    u, code = _Ufunc.from_address(id(np.tanh)), np.dtype(np.float64).num
    found = [k for k in range(u.ntypes) if u.nargs == 2 and u.types[2 * k] == u.types[2 * k + 1] == bytes([code])]
    if u.name != b'tanh' or not found:
        raise RuntimeError("numpy's tanh lays no float64 loop in its ufunc's head: the robot's gearboxes in C need it")
    loop, data = u.functions[found[0]], u.data[found[0]]
    x = np.concatenate([np.linspace(-40.0, 40.0, 4097), np.geomspace(1e-300, 1e3, 4097)])
    y = np.empty_like(x)
    _LOOP(loop)(ctypes.cast((_vp * 2)(x.ctypes.data, y.ctypes.data), _vp),
                ctypes.cast((ctypes.c_ssize_t * 1)(len(x)), _vp), ctypes.cast((ctypes.c_ssize_t * 2)(8, 8), _vp), data)
    if y.tobytes() != np.tanh(x).tobytes():
        raise RuntimeError("numpy's float64 tanh loop is not np.tanh's to the bit")
    return loop, data


def boxed(w):
    """wbc_boxes_t on the robot's world `w`'s gearboxes (`w.boxes`, their arrays stepped in place) and its braking,
    with what it points at; (None, ()) where none is stepped (`w.sprung`)."""
    from machine.body import drives, gearbox
    if not w.sprung:
        return None, ()
    bx, point = w.boxes, wbc.pointer
    on = np.ascontiguousarray(bx.sprung, np.bool_)
    loop, data = numpy_tanh()
    b = Boxes(on=point(on, np.bool_), damping=point(w.damping), tanh=loop, tanh_data=data,
              **{k: point(getattr(bx, k)) for k in ('k', 'c', 'j', 'gap', 'shock', 'mesh', 'v_mesh', 'inv_m',
                                                     'observe', 'track', 'peak', 'delta', 'w', 'hat', 'p_int',
                                                     'drive', 'asked', 'rate', 'q_step', 'went')},
              ahead_wind=drives.AHEAD_WIND, ahead_play=drives.AHEAD_PLAY, play_nm=gearbox.PLAY_NM,
              mesh_rad_s=gearbox.MESH_RAD_S, rate_s=gearbox.RATE_S, substeps=gearbox.SUBSTEPS)
    return b, (on,)


def laid(w):
    """(its address, rated, before, after, paired - each a call of its address -, the struct, what it points at) on the
    robot's world `w`: its pass's struct (`w.passing`), MuJoCo's data, the robot's setpoints, its DC link
    (`w.link`, laid anew by `w.reset`), its room's switch where its room is laid (`room.switch`), the
    robot's gearboxes where any is stepped (`boxed`), its contacts' slips (`w.stuck`, `machine.world.stiction`)."""
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
              boxes=None if boxes is None else ctypes.addressof(boxes), stuck=ctypes.addressof(w.stuck[0]),
              house_w=link['house_w'], onset_v=dclink.ONSET_V, clamp_v=dclink.CLAMP_V, link_f=dclink.LINK_F,
              pack_v=drives.PACK_V, pack_ohm=dclink.PACK_OHM, charge_a=dclink.CHARGE_A, step_s=physics.STEP_S,
              switch_s=room.SWITCH_S, switched=getattr(w, 'switched', room.SWITCH_S), lamp=int(getattr(w, 'lamp', False)))
    at = ctypes.addressof(s)
    rated, before, after, paired = (functools.partial(getattr(lib, 'wbh_world_' + k), at)
                                    for k in ('rated', 'before', 'after', 'paired'))
    stg = staged()
    s._split = (functools.partial(lib.wbh_world_drives, at), functools.partial(lib.wbh_world_stuck_step, at),
                functools.partial(lib.wbh_world_contacts_after, at), functools.partial(lib.wbh_world_rest_after, at),
                None if stg is None else functools.partial(stg[0], stg[2], w.model._address, w.data._address),
                None if stg is None else stg[1])
    s._fused = (functools.partial(lib.wbh_world_join_before, at),
                None if stg is None else functools.partial(lib.wbh_world_split_after, at, stg[2], w.model._address,
                                                           w.data._address))
    return at, rated, before, after, paired, s, (link, masks, boxes, held)


def held(vel, of, at, lie, still, by_geom, live, nthing, low):
    """The robot's loose things asleep held a pass in C (wbc_world_held) on `loose._held`'s arrays - their velocities
    three a thing, which they are, their coordinates, where each lies, their velocities held still, each geom's thing
    asleep, the geoms of what moves -: a call of MuJoCo's data, the things that woke (none: each held where it lies,
    still)."""
    lib, point = typed(), wbc.pointer
    ints = [np.ascontiguousarray(a, np.int32) for a in (by_geom, vel, of, at, still)]
    kept = ints + [np.ascontiguousarray(live, np.bool_), np.ascontiguousarray(lie, np.float64), np.zeros(nthing, np.uint8)]
    s = Held(**{k: point(a, np.int32) for k, a in zip(('by_geom', 'vel', 'of', 'at', 'still'), ints)},
             live=point(kept[5], np.bool_), lie=point(kept[6]), woken=point(kept[7], np.uint8), low=low,
             nvel=len(vel), nat=len(at), nstill=len(still), nthing=nthing)
    fn, address, woken = lib.wbh_world_held, ctypes.addressof(s), kept[7]

    def hold(d):
        return np.flatnonzero(woken).tolist() if fn(d._address, address) else []
    hold.kept = (s, kept)
    return hold


def stuck(pair, at, slip, row, held, borne, grip, share, step_s, fade):
    """(wbc_stuck_t on `machine.world.stiction.laid`'s arrays - each contact's geoms, where it bore, its slip, its
    friction rows' slips this step, what of its slip its cone held, its normal force; MuJoCo's geoms' friction and
    each geom's sliding share of it -, a step, s, and the friction's fade to its sliding share, 1 / v^2; its
    friction rows sprung between MuJoCo's step's halves (wbc_world_stuck), its slips yielded after it
    (wbc_world_slipped), each a call of MuJoCo's data)."""
    lib, point = typed(), wbc.pointer
    s = Stuck(pair=point(pair, np.int32), at=point(at), slip=point(slip), row=point(row), held=point(held),
              borne=point(borne), grip=point(grip), share=point(share), step_s=step_s, fade=fade, cap=len(at), n=0)
    address = ctypes.addressof(s)
    return (s, lambda d, fn=lib.wbh_world_stuck: fn(d._address, address),
            lambda d, fn=lib.wbh_world_slipped: fn(d._address, address))
