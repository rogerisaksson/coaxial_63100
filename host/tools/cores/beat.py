"""The robot's 1 kHz pass beside its law in C (wbc_beat.c): its felt and its drive, one call each.

    felt, drive, both, _b, _kept = g['beat'] = beat.laid(g)   # on the robot's laid session, anew where it is relaid
    felt(dt)                              # its IMU, encoders and torques into its edge; its observer's, filter's sums
    motion.drives(g, dt, drive)           # its drives' tick and its law's joints told its boards, one call
    motion.drives(g, dt, both)            # its felt and its drive in one call, between its law's passes

`machine.process.sprung.passed`'s edge.felt, observer.felt, inekf.felt, motion.drives and the robot's law's joints
told (its edge's bound drive) in that order, numpy's arithmetic operation for operation (test_tick): between its
law's passes its five calls and the arrays between them were 59 us a pass, 24 of them its boards' told; in one call,
35.
"""
import ctypes
import functools

import numpy as np

from tools.cores import wbc

_d, _i, _vp = ctypes.c_double, ctypes.c_int, ctypes.c_void_p


class Beat(ctypes.Structure):

    """wbc_beat_t: the robot's pass, pointing at its world's pass, its observer, its drives' tick and its
    arrays."""

    _fields_ = [('world', _vp), ('gmo', _vp), ('tick', _vp), ('y', _vp), ('motor', _vp), ('link', _vp), ('imu', _vp),
                ('span', _vp), ('idx', _vp), ('target', _vp), ('feed', _vp), ('kp', _vp), ('limit', _vp),
                ('ahead_fn', _vp), ('stage', _vp), ('model', _vp), ('data', _vp), ('ahead', _vp),
                ('quat_at', _i), ('gyro_at', _i), ('accel_at', _i), ('q_at', _i), ('qd_at', _i), ('qm_at', _i),
                ('tau_at', _i), ('link_at', _i), ('imu_at', _i), ('nimu', _i), ('nj', _i), ('n', _i)]


@functools.lru_cache(maxsize=None)
def typed():
    """The library's pass typed, wbc_beat_t's size the C's."""
    lib = wbc.library()
    lib.wbh_beat_size.restype = _i
    if lib.wbh_beat_size() != ctypes.sizeof(Beat):
        raise ValueError('wbc_beat_t is %d bytes, its mirror %d' % (lib.wbh_beat_size(), ctypes.sizeof(Beat)))
    lib.wbh_beat_felt.argtypes, lib.wbh_beat_felt.restype = [_vp, _d], None
    for fn in (lib.wbh_beat_drive, lib.wbh_beat_passed):
        fn.argtypes, fn.restype = [_vp, _d, _d, _d, _d], None
    return lib


def laid(g):
    """(the robot's felt - a call of `dt` -, its drive and both - each (its address, its call), as
    `motion.drives` takes a step -, the struct, the arrays it points at) on the robot's laid session `g`: the robot's
    world's pass and boards, its edge's array (`g['sense']`), its observer, its filter
    (`g['inekf']`), its drives' tick (`g['tick']`) and its law's joints told as its edge's
    bound drive tells them (`g['joints']`, `g['cmd']`)."""
    from machine.senses import edge, inekf
    from tools.cores import world as c_world
    lib, world, y, f, cmd = typed(), g['world'], g['sense'], g['inekf'], g['cmd']
    edge.buses(world)
    idx, link, nj = np.ascontiguousarray(g['joints'], dtype=np.int32), world.link['v'], len(world.motor)
    if nj != edge.TAU.stop - edge.TAU.start:
        raise ValueError('its world has %d motors, the robot\'s edge %d torques' % (nj, edge.TAU.stop - edge.TAU.start))
    stg, ahead = c_world.staged(), ctypes.c_int(0)
    b = Beat(world=ctypes.addressof(world.passing), gmo=ctypes.addressof(g['observer']['c']), tick=g['tick'][0],
             y=wbc.pointer(y), motor=wbc.pointer(world.motor), link=wbc.pointer(link), imu=wbc.pointer(f['imu']),
             span=wbc.pointer(f['span']), idx=wbc.pointer(idx, np.int32),
             **{name: wbc.pointer(a) for name, a in cmd.items()},
             ahead_fn=None if stg is None else ctypes.cast(stg[0], _vp).value,
             stage=None if stg is None else stg[2],
             model=None if stg is None else world.model._address,
             data=None if stg is None else world.data._address,
             ahead=ctypes.addressof(ahead),
             quat_at=edge.QUAT.start, gyro_at=edge.GYRO.start, accel_at=edge.ACCEL.start, q_at=edge.Q.start,
             qd_at=edge.QD.start, qm_at=edge.QM.start, tau_at=edge.TAU.start, link_at=edge.LINK.start,
             imu_at=inekf.IMU.start, nimu=inekf.IMU.stop - inekf.IMU.start, nj=nj, n=len(idx))
    at = ctypes.addressof(b)
    return (functools.partial(lib.wbh_beat_felt, at), (at, lib.wbh_beat_drive), (at, lib.wbh_beat_passed), b,
            (idx, y, world.motor, link, f['imu'], f['span'], world.passing, g['observer']['c'], g['tick'], *cmd.values(),
             ahead))
