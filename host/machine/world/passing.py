"""The world's pass in C laid once (wbc_pass.c): its struct pointed at her arrays and MuJoCo's."""
import ctypes

from machine.body import dclink
from machine.body.figure import JOINTS


def laid(world, wound):
    """The pass in C (`tools.cores.wbc.World`) on `world`'s arrays and MuJoCo's, its gearboxes wound by
    `wound`: its struct pointing at them once, its calls bound onto `world`."""
    from tools.cores import motion, wbc as cores
    np, d = world._np, world.data
    world._kept = tuple(np.ascontiguousarray(a, dtype=np.int32) for a in (world.qadr, world.vadr, world.pairs))
    p = cores.World(n=len(JOINTS), npairs=len(world.pairs), pelvis=world.pelvis, wound=wound)
    for name, a in zip(('qadr', 'vadr', 'pairs'), world._kept):
        setattr(p, name, cores.pointer(a, np.int32))
    for name, a in (('qpos', d.qpos), ('qvel', d.qvel), ('qacc', d.qacc), ('xmat', d.xmat), ('ctrl', d.ctrl),
                    ('target', world.target), ('rate', world.rate), ('feed', world.feed), ('gains', world.gains),
                    ('limit', world.limit), ('kt', world.kt), ('delta', world.boxes.delta), ('zero', world.boxes.zero),
                    ('armature', world.armature), ('motor', world.motor), ('geared', world.geared), ('loss', world.loss)):
        setattr(p, name, cores.pointer(a))
    for name, a in (('shorted', world.shorted), ('cut', world.cut)):
        setattr(p, name, cores.pointer(a, np.bool_))
    p.blas = ctypes.addressof(motion.numpy_blas())
    lib = cores.library()
    world._ctrl, world._motor, world._metered = lib.wbh_pass_ctrl, lib.wbh_pass_motor, lib.wbh_pass_metered
    world._drawn = lib.wbh_pass_drawn
    world._drawn_of = (p.loss, cores.pointer(world.driven, np.bool_), cores.pointer(world.testing),
                      cores.pointer(world.ohm))
    world.linked = dclink.compiled()
    return p
