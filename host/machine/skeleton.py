"""Her skeleton as MuJoCo collides it, a welded massless body a part riding its segment.

Each drive's drum, each board apart from its drive, the femurs and tibias, the pelvis's boom, the
ankles' bent rods and cranks, the hip rolls' spur pairs and the belts; her segments carry their
weight (`build.segments`).

    skeleton.bodies()           # [(body, segment it rides, [geom xml])], each geom in its frame
    skeleton.owners(model)      # {body id: segment}: her segments and what rides each
    skeleton.overlapping(model) # [(body, body)]: touching standing, so by design

A part a body of its own meets the segment its own hangs from - the folded femur the calf's rod -
which MuJoCo's parent and child never do; what touches standing is excluded (`machine.mjcf`).
"""
import math

from machine import drives, linkage
from machine.figure import SEGMENTS
from machine.gait import SHANK, THIGH

#: The femur and the tibia, her left's, through these points of their segments' frames, m (the
#: right's x mirrored), their tubes' radii: each from under its drive's gearbox to over the next's
#: motor's middle, the femur toward the thigh's front over its last 25 cm, clear of the calf's
#: drives and rod folded at 163 deg; from their drums' centres they ran 40 mm through the hip's
#: motor and gearbox (the user, 2026-10-02). Each drum they hang from or reach clamped by a collar
#: (`coaxial.graphics.mechanism`): (joint, its gearbox or motor, y on the segment).
HUNG = {'thigh': (((0.0285, -0.05, 0.0), (0.0, -0.14, 0.03), (0.0, 0.05 - THIGH, 0.0)), 0.016,
                  (('hip', 'gear', 0.0), ('knee', 'motor', -THIGH))),
        'shank': (((0.0285, -0.05, 0.0), (0.0, -SHANK, 0.0)), 0.014, (('knee', 'gear', 0.0),))}

#: A board's thickness with its parts, m; the pelvis's boom, its radius and length; a rod's tube's
#: radius and its crank's.
BOARD_T, BOOM, ROD_R, CRANK_R = 0.012, (0.015, 0.2), 0.009, 0.012

#: Her skeleton's contacts: with the floor and what lies on it, her skins and itself
#: (`mjcf.ME`, `MEETS`), its own friction.
CONTYPE, CONAFFINITY, FRICTION = 4, 5, 0.5

AXES = {'x': (1.0, 0.0, 0.0), 'y': (0.0, 1.0, 0.0), 'z': (0.0, 0.0, 1.0)}


def _geom(shape, radius, a, b):
    return ('<geom type="%s" size="%g" fromto="%g %g %g %g %g %g" contype="%d" conaffinity="%d" '
            'condim="3" friction="%g 0.005 0.0001"/>' % ((shape, radius) + tuple(a) + tuple(b)
                                                        + (CONTYPE, CONAFFINITY, FRICTION)))


def _along(at, axis, half):
    return (tuple(p - a * half for p, a in zip(at, axis)),
            tuple(p + a * half for p, a in zip(at, axis)))


def _drums():
    """[(joint, segment it rides, where, axis letter)]: each drive where `build.riders` puts it."""
    out = []
    for name, parent, joints, offset, *_ in SEGMENTS:
        for k, (joint, axis, _sign) in enumerate(joints):
            if drives.passive(joint):
                continue
            where = drives.mount(joint)
            if where is not None:
                rides, at = where[0], tuple(where[1])
                axis = (drives.JOINTS[drives.kind(joint)][1][2:3] or (axis,))[0]
            elif k == 0 and parent is not None:
                rides, at = parent, tuple(offset)
            else:
                rides, at = name, (0.0, 0.0, 0.0)
            out.append((joint, rides, at, axis))
    return out


def _rod(side, kind):
    """A rod's crank's hub, pin, bend and ball at rest in its drive's segment's frame."""
    s = 1.0 if side == 'left_' else -1.0
    seg = next(g for g in SEGMENTS if side + kind in [j for j, *_ in g[2]])
    up, ahead, r, t0, b, beta = linkage.RODS[kind][0]
    ox, oy, oz = seg[3]
    hub = (ox + s * linkage.ROD_OUT, oy + up, oz + ahead)
    pin = (hub[0], hub[1] - r * math.cos(t0), hub[2] + r * math.sin(t0))
    ball = (ox + s * linkage.BALL_OUT, oy - b * math.cos(beta), oz + b * math.sin(beta))
    return seg, hub, pin, linkage.bent(kind, pin, ball, (s, 0.0, 0.0), s), ball


def bodies():
    """[(body, segment it rides, [geom xml])]: her skeleton's parts, each its own welded body."""
    out = []
    for joint, rides, at, axis in _drums():
        half, r = drives.length(joint) / 2.0, drives.of(joint)[1].diameter / 2.0
        out.append((joint + '_drum', rides, [_geom('cylinder', r, *_along(at, AXES[axis], half))]))
    for joint, (seg, at, _kg, radius, faces) in drives.boards().items():
        out.append((joint + '_board', seg, [_geom('cylinder', radius,
                                                  *_along(at, AXES[faces], BOARD_T / 2.0))]))
    out.append(('pelvis_boom', 'pelvis', [_geom('cylinder', BOOM[0],
                                                *_along((0.0, 0.0, 0.0), AXES['x'], BOOM[1] / 2))]))
    for side in ('left_', 'right_'):
        s = 1.0 if side == 'left_' else -1.0
        for seg, (points, radius, _collars) in HUNG.items():
            p = [(s * x, y, z) for x, y, z in points]
            out.append((side + seg + '_bone', side + seg,
                        [_geom('capsule', radius, a, b) for a, b in zip(p, p[1:])]))
        for kind in linkage.RODS:
            seg, hub, pin, bend, ball = _rod(side, kind)
            out.append((side + kind + '_rod', seg[1], [
                _geom('capsule', CRANK_R, hub, pin), _geom('capsule', ROD_R, pin, bend),
                _geom('capsule', ROD_R, bend, ball)]))
        for kind, ratio in linkage.GEARS.items():
            joint = side + kind
            rides, (x, y, z) = drives.mount(joint) or ('', (0.0, 0.0, 0.0))
            pivot = next(g[3] for g in SEGMENTS if joint in [j for j, *_ in g[2]])
            face = z + drives.length(joint) / 2.0
            apart = math.hypot(pivot[0] - x, pivot[1] - y)
            r = apart / (1.0 + ratio)
            out.append((joint + '_spurs', rides, [
                _geom('cylinder', r, *_along((x, y, face), AXES['z'], 0.004)),
                _geom('cylinder', apart - r, *_along((pivot[0], pivot[1], face), AXES['z'],
                                                     0.004))]))
        for kind, (drive_r, joint_r, out_m) in linkage.BELTS.items():
            joint = side + kind
            drum = next((d for d in _drums() if d[0] == joint), None)
            seg = next(g for g in SEGMENTS if joint in [j for j, *_ in g[2]])
            if drum is None or drum[1] != seg[1]:
                continue
            o = s * out_m
            out.append((joint + '_belt', seg[1], [_geom(
                'capsule', max(drive_r, joint_r), (drum[2][0] + o,) + drum[2][1:],
                (seg[3][0] + o,) + tuple(seg[3][1:]))]))
    return out


def owners(model):
    """{body id: segment}: her segments' bodies and her skeleton's, each by the segment it rides."""
    out = {model.body(s[0]).id: s[0] for s in SEGMENTS}
    for body, rides, _geoms in bodies():
        try:
            out[model.body(body).id] = rides
        except KeyError:
            pass
    return out


def overlapping(model, angles):
    """[(body, body)]: the pairs touching with her joints at {joint: deg}, at least one her
    skeleton's - by design, a drum and what hangs on it, a rod's ball and its heel."""
    import importlib
    from machine.figure import JOINTS
    mujoco = importlib.import_module('mujoco')
    d = mujoco.MjData(model)
    mujoco.mj_resetData(model, d)
    for joint in JOINTS:
        j = model.joint(joint)
        d.qpos[j.qposadr[0]] = math.radians(angles.get(joint, 0.0))
    d.qpos[1] = 2.0                                    # clear of the floor
    mujoco.mj_forward(model, d)
    ours = {model.body(b).id for b, _r, _g in bodies()}
    pairs = set()
    for i in range(d.ncon):
        a, b = (int(model.geom_bodyid[g]) for g in (d.contact[i].geom1, d.contact[i].geom2))
        if a in ours or b in ours:
            pairs.add(tuple(sorted((model.body(a).name, model.body(b).name))))
    return sorted(pairs)
