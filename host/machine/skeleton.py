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
from typing import Any

from machine import drives, linkage
from machine.figure import SEGMENTS, SHANK, THIGH, UPPER_ARM

#: The femur and tibia, her left's, their tubes through these points of their segments' frames, m
#: (the right's x mirrored), their radii: femur from under the hip's gearbox to over the knee's
#: motor's middle, clear of the calf's drives and rod folded at 163 deg; tibia from the knee's
#: gearbox behind the ankle's drives to the ankle, between their rods. Each drum clamped by a
#: collar (`coaxial.graphics.mechanism`): (joint, its gearbox or motor, y on the segment).
#: Seven stock sizes in high-modulus carbon: 12x1, 20x1, 25x1, 30x2, 35x2, 40x2, 50x2 mm.
HUNG = {'thigh': ((((0.034, -0.05, 0.0), (0.0, -0.14, 0.03)),
                   ((0.0, -0.14, 0.03), (-0.019, 0.05 - THIGH, 0.012))), (0.020, 0.020),
                  (('hip', 'gear', 0.0), ('knee', 'motor', -THIGH))),
        'shank': ((((0.0285, -0.045, -0.032), (0.0, -0.17, -0.05)),
                   ((0.0, -0.17, -0.05), (0.0, 0.06 - SHANK, -0.008), (0.0, 0.035 - SHANK, 0.014)),
                   ((0.03, 0.02 - SHANK, 0.014), (0.0, 0.035 - SHANK, 0.014),
                    (-0.03, 0.02 - SHANK, 0.014)),
                   ((0.03, 0.02 - SHANK, 0.014), (0.03, -SHANK, 0.0)),
                   ((-0.03, 0.02 - SHANK, 0.014), (-0.03, -SHANK, 0.0))),
                  (0.020, 0.0175, 0.010, 0.010, 0.010),
                  (('knee', 'gear', 0.0),)),
        'upper_arm': ((((0.032, -0.004, 0.0), (0.032, -0.05, 0.0), (0.0, -0.077, 0.0)),
                       ((-0.014, -0.145, -0.016), (-0.012, -0.19, -0.012), (-0.012, -0.235, -0.01),
                        (-0.024, 0.042 - UPPER_ARM, 0.0))),
                      (0.0125, 0.0125), (('elbow', 'motor', -UPPER_ARM),)),
        'forearm': ((((0.034, -0.037, 0.0), (0.0, -0.055, 0.0)), ((0.0, -0.055, 0.0), (0.0, -0.089, 0.0)),
                     ((0.0, -0.142, -0.015), (0.0, -0.18, -0.012), (0.0, -0.215, -0.008),
                      (-0.016, -0.235, 0.0), (-0.016, -0.25, 0.0)),
                     ((0.0, -0.215, -0.008), (0.013, -0.235, 0.0), (0.013, -0.25, 0.0))),
                    (0.010, 0.0125, 0.010, 0.010), (('elbow', 'gear', 0.0),)),
        'foot': ((((0.0, 0.0, -0.026), (0.0, -0.035, -0.032), (0.0, -0.066, -0.045),
                   (0.0, -0.066, 0.105), (0.0, -0.061, 0.117)),
                  ((0.0, 0.0, 0.026), (0.0, -0.03, 0.018), (0.0, -0.066, 0.015)),
                  ((0.012, 0.0047, -0.0174), (0.0, 0.004, -0.024), (-0.016, 0.0057, -0.0213)),
                  ((-0.03, -0.061, 0.117), (0.03, -0.061, 0.117))), 0.006, ())}

#: A crank's radius (a rod's `linkage.ROD_R`).
CRANK_R = 0.012

#: The pelvis's boom, its radius and the points it runs through, m: 50x2 high-modulus (`HUNG`).
BOOM = (0.025, ((0.066, 0.062, 0.0), (0.006, -0.03, 0.0), (-0.006, -0.03, 0.0), (-0.066, 0.062, 0.0)))

#: Her trunk's frame: per frame, tubes through these points, TRUNK_R round, and rings (centre,
#: axis, radius, half), m in it; carbon tubes cut to length in printed PAHT-CF holders.
TRUNK_R = {'pelvis': (0.015, 0.015, 0.020), 'spine_roll': (0.007, 0.007), 'spine': (0.020,),
           'torso': (0.02, 0.015), 'neck': (0.010,)}
TRUNK = {'pelvis': ((((0.055, 0.045, 0.0), (0.055, 0.045, 0.066), (0.0, 0.099, 0.066)),
                     ((-0.055, 0.045, 0.0), (-0.055, 0.045, 0.066), (0.0, 0.099, 0.066)),
                     ((0.0, -0.03, 0.0), (0.0, -0.03, -0.06), (0.0, 0.099, -0.066))),
                    (((0.0, 0.12, 0.066), 'z', 0.009, 0.007),
                     ((0.0, 0.12, -0.066), 'z', 0.009, 0.007))),
         'spine_roll': ((((0.0, 0.0, 0.043), (0.0, 0.0, 0.052)),
                         ((0.0, 0.0, -0.043), (0.0, 0.0, -0.052))),
                        (((0.0, 0.0, 0.0), 'x', 0.043, 0.008),)),
         'spine': ((((0.049, 0.0, 0.0), (0.052, 0.048, 0.0), (0.0, 0.062, 0.0)),), ()),
         'torso': ((((0.0, 0.155, 0.0), (0.0, 0.352, 0.004)),
                    ((-0.105, 0.325, -0.005), (0.105, 0.325, -0.005))), ()),
         'neck': ((((0.047, 0.0, 0.0), (0.047, 0.042, 0.0), (0.0, 0.042, 0.012)),), ())}

#: The hip's gimbal (`gimbal`): tubes' radius, roll bearings (radius, half length, z), cradle band.
FORK_R, CROWN_R, BEARING, BAND = 0.015, 0.005, (0.009, 0.007, 0.060), (0.043, 0.008)
STEERER_R, ARM_R = 0.0125, 0.0125

#: Each drive held by what carries it - the hip's roll drum, the ankles', the toes' and the boards
#: held by nothing: its collars, COLLAR_M proud of its drum, each (m from
#: its middle toward its output, half width); its struts, STRUT_R round - 20 for the rods' 3-4 kN -
#: unless a third says (the ankle roll's 14: a 20 met the folded femur, 3 mm), (a, b) m in the
#: frame its drum rides, her left's (the right's x mirrored), from a collar into the fork's back
#: leg or the tibia. The boards on POSTS into the femur; the ankle's CROSS (pins' radius, half length)
#: on the stage between its pitch and its roll.
COLLAR_M, STRUT_R, POST_R = 0.004, 0.010, 0.004
HELD = {'hip_yaw': (((-0.014, 0.008),), ()),
        'hip_roll': (((0.018, 0.006), (-0.022, 0.006)),
                     (((0.02, 0.035, -0.055), (0.009, 0.035, -0.06)),
                      ((0.02, 0.058, -0.055), (-0.001, 0.059, -0.1)))),
        'ankle': (((0.0185, 0.006),), (((0.0205, -0.08, -0.019), (0.0205, -0.08, -0.037)),)),
        'ankle_roll': (((-0.0086, 0.006),),
                       (((0.0046, -0.15, -0.034), (0.0046, -0.15, -0.047), 0.007),)),
        'wrist': (((-0.022, 0.005), (0.022, 0.005)), ()),
        'foot': (((0.0, 0.006),), ()),
        'spine_roll': (((-0.025, 0.006),), ()),
        'waist': (((0.025, 0.006), (-0.025, 0.006)), ()), 'neck': (((0.0, 0.006),), ()),
        'head': (((0.0, 0.005),), ()), 'shoulder': (((-0.02, 0.006),), ())}
POSTS = {'thigh': (((-0.0019, -0.16, 0.0282), (0.057, -0.16, 0.0282)),
                   ((-0.0057, -0.20, 0.0246), (0.057, -0.20, 0.0246)))}
CROSS = ('ankle', 0.007, 0.026)

#: Her skeleton's contacts: her skins and itself (`mjcf.ME`, `MEETS`), its own friction - never
#: the floor: inside her shell, outside her skins' capsules, the toes' belt's pulley met it as
#: her sole sank walking and felled her at 6.05 s, a knee's drum landed past its gel, 935 N
#: against 102 bare.
CONTYPE, CONAFFINITY, FRICTION = 4, 4, 0.5

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
                axis = (drives.output(joint) or (axis,))[0]
            elif k == 0 and parent is not None:
                rides, at = parent, tuple(offset)
            else:
                rides, at = name, (0.0, 0.0, 0.0)
            out.append((joint, rides, at, axis))
    return out


def gimbal(stage):
    """([(a, b, radius)], [(centre, axis, radius, half)]): the hip's gimbal on its `stage`, her
    left's in its frame, the hip's centre the origin (the right's x mirrored) - 'hip_yaw' the
    fork, from its drive's output down a steerer to a crown over the pitch's L and its legs to the
    roll's bearings; 'hip_roll' the cradle, a band round the L, its trunnions in the bearings."""
    r, half, z = BEARING
    if stage == 'hip_roll':
        return ([((0.0, 0.0, s * BAND[0]), (0.0, 0.0, s * (z + half)), r - 0.002)
                 for s in (1.0, -1.0)], [((0.0, 0.0, 0.0), 'x', BAND[0], BAND[1])])
    thigh = next(s for s in SEGMENTS if s[0] == 'left_thigh')
    top = (drives.mount('left_hip_yaw') or ('', (0.0, 0.0, 0.0)))[1][1] - thigh[3][1] - (
        drives.length('left_hip_yaw') / 2.0)
    crown = top - 0.01
    legs = ((-0.02, z), (0.02, -z))
    tubes = [((0.0, top, 0.0), (0.0, crown, 0.0), STEERER_R)]
    for x, at in legs:
        tubes += [((0.0, crown, 0.0), (x, crown, at), CROWN_R), ((x, crown, at), (x, 0.0, at), FORK_R),
                  ((x, 0.0, at), (0.0, 0.0, at), ARM_R)]
    return tubes, [((0.0, 0.0, at), 'z', r, half) for _x, at in legs]


def held(side):
    """[(name, segment or stage it rides, shape)]: `side`'s collars, struts and posts (`HELD`,
    `POSTS`) and its ankle's cross, each ('ring', centre, axis, radius, half) or ('tube', a, b,
    radius) in the frame it rides."""
    s = 1.0 if side == 'left_' else -1.0
    placed = {j: (rides, at, letter) for j, rides, at, letter in _drums()}
    out = []
    for kind, (collars, struts) in HELD.items():
        joint = kind if kind in placed else 'left_' + kind
        if (joint == kind and side != 'left_') or joint not in placed:
            continue                                   # the right's of a trunk's; no drum held
        rides, at, letter = placed[joint]
        end = (drives.output(joint) or (letter, 1.0))[1]
        axis = tuple(end * v for v in AXES[letter])
        r = drives.of(joint)[1].diameter / 2.0 + COLLAR_M
        shapes = [('ring', tuple(p + a * along for p, a in zip(at, axis)), axis, r, half)
                  for along, half in collars] + [('tube', st[0], st[1], st[2] if len(st) > 2
                                                   else STRUT_R) for st in struts]
        name = kind if joint == kind else side + kind
        out += [(name, rides.replace('left_', side), shape) for shape in shapes]
    for seg, posts in POSTS.items():
        out += [(side + seg + '_posts', side + seg, ('tube', a, b, POST_R)) for a, b in posts]
    stage, r, half = CROSS
    out += [(side + stage + '_cross', side + stage, ('ring', (0.0, 0.0, 0.0), AXES[a], r, half))
            for a in 'xz']
    return [(name, rides, _mirror(shape, s)) for name, rides, shape in out]


def trunk():
    """[(frame, shape)]: her trunk's frame (`TRUNK`), each ('ring', centre, axis, radius, half)
    or ('tube', a, b, radius) in its frame."""
    out = []
    for frame, (tubes, rings) in TRUNK.items():
        out += [(frame, ('tube', a, b, r)) for points, r in zip(tubes, TRUNK_R[frame])
                for a, b in zip(points, points[1:])]
        out += [(frame, ('ring', c, AXES[axis], r, half)) for c, axis, r, half in rings]
    return out


def collars(side, seg):
    """[(centre, radius, half)]: the collars `seg`'s bones (`HUNG`) clamp round their drums' motors
    or gearboxes (`drives.along`), along x, its frame."""
    s, out = (1.0 if side == 'left_' else -1.0), []
    for joint, part, y in HUNG[seg][2]:
        _p, r, at, length = next(row for row in drives.along(side + joint) if row[0] == part)
        # A collar a millimetre short of the motor's and gearbox's seam: the knee's two turn;
        # a mounted drum where its mount puts it along the axis (the hip's 15 mm in).
        where = drives.mount(side + joint)
        at = s * (at + (0.0005 if part == 'gear' else -0.0005)) + (where[1][0] if where else 0.0)
        out.append(((at, y, 0.0), r + COLLAR_M, length / 2.0 - 0.0005))
    return out


def runs(seg, s=1.0):
    """[(a, b, radius)]: `seg`'s hung tubes (`HUNG`) segment by segment, a run's radius each (one
    for all, or one a run), its frame with x times `s`."""
    tubes, radius, _collars = HUNG[seg]
    radii = radius if isinstance(radius, tuple) else (radius,) * len(tubes)
    return [((s * a[0],) + tuple(a[1:]), (s * b[0],) + tuple(b[1:]), r)
            for points, r in zip(tubes, radii) for a, b in zip(points, points[1:])]


def _mirror(shape, s) -> Any:
    def m(p):
        return (s * p[0],) + tuple(p[1:])
    if shape[0] == 'ring':
        return ('ring', m(shape[1]), m(shape[2])) + shape[3:]
    return ('tube', m(shape[1]), m(shape[2]), shape[3])


def _rod(side, kind):
    """A rod's crank's hub, pin, bend and ball at rest in its drive's segment's frame."""
    s = 1.0 if side == 'left_' else -1.0
    seg = next(g for g in SEGMENTS if side + linkage.ROD_AT[kind][0] in [j for j, *_ in g[2]])
    up, ahead, r, t0 = linkage.RODS[kind][0][:4]
    (ox, oy, oz), (x, by, bz) = seg[3], linkage.ball(kind)
    hub = (ox, oy + up, oz + ahead)
    pin = (ox + s * linkage.ROD_AT[kind][1], hub[1] - r * math.cos(t0), hub[2] + r * math.sin(t0))
    ball = (ox + s * x, oy + by, oz + bz)
    return seg, hub, pin, linkage.bent(kind, pin, ball, (s, 0.0, 0.0), s), ball


def bodies():
    """[(body, segment it rides, [geom xml])]: her skeleton's parts, each its own welded body."""
    out = []
    for joint, rides, at, axis in _drums():
        end = drives.toward(joint, axis)
        out.append((joint + '_drum', rides, [
            _geom('cylinder', r, *_along(tuple(p + end * a * c for p, a in zip(at, AXES[axis])),
                                         AXES[axis], long / 2.0))
            for _part, r, c, long in drives.along(joint)]))
    for joint, (seg, at, _kg, radius, faces) in drives.boards().items():
        out.append((joint + '_board', seg, [_geom('cylinder', radius,
                                                  *_along(at, AXES[faces],
                                                          drives.tall(joint) / 2.0))]))
    out.append(('pelvis_boom', 'pelvis', [_geom('capsule', BOOM[0], a, b)
                                          for a, b in zip(BOOM[1], BOOM[1][1:])]))
    for joint in [j for kind in linkage.PLANAR for j in linkage.joints(kind)]:
        crank, horn, pin, ball = linkage.four_bar(joint, {})
        out.append((joint + '_rod', (drives.mount(joint) or ('pelvis',))[0], [
            _geom('capsule', linkage.PLATE_R, crank, pin),
            _geom('capsule', linkage.ROD_R, pin, ball)]))
        out.append((joint + '_horn', joint, [
            _geom('capsule', linkage.PLATE_R, (0.0, 0.0, horn[2]),
                  (ball[0] - horn[0], ball[1] - horn[1], ball[2]))]))
    for joint in [j for kind in linkage.GEARS for j in linkage.joints(kind)]:
        rides, (x, y, z) = drives.mount(joint) or ('', (0.0, 0.0, 0.0))
        pivot = drives.pivot(joint)
        face = z + drives.length(joint) / 2.0
        apart = math.hypot(pivot[0] - x, pivot[1] - y)
        r = apart / (1.0 + linkage.GEARS[drives.kind(joint)])
        out.append((joint + '_spurs', rides, [
            _geom('cylinder', r, *_along((x, y, face), AXES['z'], 0.004)),
            _geom('cylinder', apart - r, *_along((pivot[0], pivot[1], face), AXES['z'], 0.004))]))
    for side in ('left_', 'right_'):
        s = 1.0 if side == 'left_' else -1.0
        for stage in ('hip_yaw', 'hip_roll'):
            tubes, rings = gimbal(stage)
            out.append((side + stage + '_gimbal', side + stage, [
                _geom('capsule', r, (s * a[0],) + a[1:], (s * b[0],) + b[1:])
                for a, b, r in tubes] + [
                _geom('cylinder', r, *_along((s * c[0],) + c[1:], AXES[axis], half))
                for c, axis, r, half in rings]))
        for seg in HUNG:
            out.append((side + seg + '_bone', side + seg, [_geom('capsule', r, a, b)
                                                            for a, b, r in runs(seg, s)]))
        geoms = {}
        for name, rides, shape in held(side) + ([(f + '_frame', f, sh) for f, sh in trunk()]
                                                if side == 'left_' else []):
            g = (_geom('cylinder', shape[3], *_along(shape[1], shape[2], shape[4]))
                 if shape[0] == 'ring' else _geom('capsule', shape[3], shape[1], shape[2]))
            geoms.setdefault((name + '_held', rides), []).append(g)
        out += [(name, rides, g) for (name, rides), g in geoms.items()]
        for kind in linkage.RODS:
            seg, hub, pin, bend, ball = _rod(side, kind)
            out.append((side + kind + '_rod', seg[1], [
                _geom('capsule', CRANK_R, pin[:1] + hub[1:], pin),
                _geom('capsule', linkage.ROD_R, pin, bend),
                _geom('capsule', linkage.ROD_R, bend, ball)]))
        for kind, (drive_r, joint_r, out_m) in linkage.BELTS.items():
            joint = side + kind
            drum = next((d for d in _drums() if d[0] == joint), None)
            seg = next(g for g in SEGMENTS if joint in [j for j, *_ in g[2]])
            if drum is None or drum[1] != seg[1]:
                continue
            o, start = s * out_m, (drives.outlet(joint) or ('', drum[2]))[1]
            out.append((joint + '_belt', seg[1], [_geom(
                'capsule', max(drive_r, joint_r), (start[0] + o,) + tuple(start[1:]),
                (seg[3][0] + o,) + tuple(seg[3][1:]))]))
            if kind in linkage.BEVELS:
                r = linkage.BEVELS[kind]
                out.append((joint + '_bevel', seg[1], [
                    _geom('capsule', r, start, (start[0] + s * r,) + tuple(start[1:])),
                    _geom('capsule', r, start, (start[0],) + tuple(
                        v + (b - v) * r / max(1e-9, abs(b - v) + 1e-12) for v, b in
                        zip(start[1:], drum[2][1:])))]))
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


def printed_holders():
    """[(name, rides, print_axis, bore_r, wall_m, roof_deg)]: 2.5D PAHT-CF printed tube holders
    and split clamp rings - each extruded along `print_axis` with zero overhang (`roof_deg` 45 on
    transverse bolt holes)."""
    out = []
    for name, rides, shape in held('left_') + [(f + '_frame', f, sh) for f, sh in trunk()]:
        if shape[0] == 'ring':
            _t, _c, ax, r, _half = shape
            axis = 'xyz'[max(range(3), key=lambda i: abs(ax[i]))]
            out.append((name, rides, axis, round(r - COLLAR_M, 4), COLLAR_M, 45.0))
    return out


def flange_mounts():
    """[(joint, rides, bolt_circle_r, bolts, screw_m)]: direct axial bolt-flange mounts on motor
    cans and gearbox housings where a drum meets its bone or gimbal without an external collar."""
    out = []
    for joint, rides, _at, _axis in _drums():
        if not joint.startswith('right_'):
            r = drives.of(joint)[1].diameter / 2.0
            out.append((joint, rides, round(r - 0.004, 4), 8 if r >= 0.035 else 6, 0.004 if r >= 0.035 else 0.003))
    return out

