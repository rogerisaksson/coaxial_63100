"""Her 1 kHz pass beside her law in C (wbc_beat.c): her felt and her drive, one call each.

    felt, drive, both, _b, _kept = g['beat'] = beat.laid(g)   # on her laid session, anew where she is relaid
    felt(dt)                              # her IMU, encoders and torques into her edge; her observer's, filter's sums
    motion.drives(g, dt, drive)           # her drives' tick and her law's joints told her boards, one call
    motion.drives(g, dt, both)            # her felt and her drive in one call, between her law's passes

`machine.process.sprung.passed`'s edge.felt, observer.felt, inekf.felt, motion.drives and her law's joints told (her
edge's bound drive) in that order, numpy's arithmetic operation for operation (test_tick): between her law's passes
her five calls and the arrays between them were 59 us a pass, 24 of them her boards' told; in one call, 35.
"""
import ctypes
import functools

import numpy as np

from tools.cores import wbc

_d, _i, _vp = ctypes.c_double, ctypes.c_int, ctypes.c_void_p


class Beat(ctypes.Structure):

    """wbc_beat_t: her pass, pointing at her world's pass, her observer, her drives' tick and her arrays."""

    _fields_ = [('world', _vp), ('gmo', _vp), ('tick', _vp), ('y', _vp), ('motor', _vp), ('link', _vp), ('imu', _vp),
                ('span', _vp), ('idx', _vp), ('target', _vp), ('feed', _vp), ('kp', _vp), ('limit', _vp),
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
    """(her felt - a call of `dt` -, her drive and both - each (its address, its call), as `motion.drives` takes a
    step -, the struct, the arrays it points at) on her laid session `g`: her world's pass and boards, her edge's array
    (`g['sense']`), her observer, her filter (`g['inekf']`), her drives' tick (`g['tick']`) and her law's joints told
    as her edge's bound drive tells them (`g['joints']`, `g['cmd']`)."""
    from machine.senses import edge, inekf
    lib, world, y, f, cmd = typed(), g['world'], g['sense'], g['inekf'], g['cmd']
    edge.buses(world)
    idx, link, nj = np.ascontiguousarray(g['joints'], dtype=np.int32), world.link['v'], len(world.motor)
    if nj != edge.TAU.stop - edge.TAU.start:
        raise ValueError('her world has %d motors, her edge %d torques' % (nj, edge.TAU.stop - edge.TAU.start))
    b = Beat(world=ctypes.addressof(world.passing), gmo=ctypes.addressof(g['observer']['c']), tick=g['tick'][0],
             y=wbc.pointer(y), motor=wbc.pointer(world.motor), link=wbc.pointer(link), imu=wbc.pointer(f['imu']),
             span=wbc.pointer(f['span']), idx=wbc.pointer(idx, np.int32),
             **{name: wbc.pointer(a) for name, a in cmd.items()},
             quat_at=edge.QUAT.start, gyro_at=edge.GYRO.start, accel_at=edge.ACCEL.start, q_at=edge.Q.start,
             qd_at=edge.QD.start, qm_at=edge.QM.start, tau_at=edge.TAU.start, link_at=edge.LINK.start,
             imu_at=inekf.IMU.start, nimu=inekf.IMU.stop - inekf.IMU.start, nj=nj, n=len(idx))
    at = ctypes.addressof(b)
    return (functools.partial(lib.wbh_beat_felt, at), (at, lib.wbh_beat_drive), (at, lib.wbh_beat_passed), b,
            (idx, y, world.motor, link, f['imu'], f['span'], world.passing, g['observer']['c'], g['tick'], *cmd.values()))
