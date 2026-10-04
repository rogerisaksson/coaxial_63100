"""Her mechanism without her shell: carbon tubes, drums, rods on ball joints, quick-releases.

    parts = mechanism.parts()               # [(name, parent, joints, offset, rest, mesh)]
    parts = mechanism.parts(bare=True)      # motors, gearboxes, cranks, rods, balls: no body
    mechanism.posed(parts, placed, angles, frames)   # each crank's, pin's and rod's (turn, spot)
    layers = mechanism.wires(parts, frames)  # the kinematic stick figure: [(a, b, ink)], world

Her skeleton's parts (`machine.skeleton`), each drive's drum (`drums`), each belt over its two
pulleys; cranks, pins and rods posed each frame (`posed`), their parent '*'.
"""
import math

from coaxial.graphics import drums
from coaxial.graphics.lit import paint
from coaxial.graphics.shapes import drum, ellipsoid, limb, loft
from machine import build, drives, figure, linkage
from machine.skeleton import BOOM, HUNG, collars, gimbal, held, runs, trunk
from machine.figure import FOREARM, UPPER_ARM
from machine.gait import ANKLE_H, BALL, HEEL, SHANK, THIGH

#: The colours: the carbon's, the rods' and their ball joints' steel, the quick-releases' printed
#: polymer - violet, off the heat's ramp: orange read as a warm hip (the user, 2026-10-02) - and
#: each drive's drum by its frame (`drives.FRAMES`).
CARBON, ROD, STEEL, POLYMER = (92, 94, 106), (214, 214, 224), (246, 246, 246), (150, 100, 220)
#: A spur pair's face width, m (`linkage.GEARS`): its pinion on its drive's output face, its wheel
#: on the joint's axis, their pitch circles meeting.
SPUR_T = 0.008
#: A board apart from its drive: its laminate's green, BOARD_T thick with its parts, m.
PCB, BOARD_T = (40, 120, 70), 0.012
SIZED = {'A': (72, 140, 224), 'B': (60, 190, 170)}

#: Bare, a drive's stack drawn as its parts (`drives.along`): its inverter in the laminate's green,
#: its motor in its frame's colour, its gearbox in the gearbox's steel grey.
GEARBOX = (150, 152, 160)
PARTED = {'board': 'inv_', 'motor': 'drive_', 'gear': 'gear_'}

#: The stick figure's inks: the skeleton's, the cranks' and rods', the ball joints'; a wire
#: cylinder's rims RIM points round, a ball joint a cross BALL_R across; a bone broken RELEASE_GAP
#: clear either side of its quick-release, the collar in its polymer.
BONE_INK, ROD_INK, BALL_INK, RIM = (200, 200, 206), (250, 150, 60), (255, 255, 255), 10
RELEASE_GAP = 0.006
#: On her shell a quick-release's band stands RELEASE_PROUD_M proud of it (`bands`).
RELEASE_PROUD_M = 0.001

#: A ball joint BALL_R round (its rod `linkage.ROD_R`, its plane `linkage.ROD_AT`); a belt's
#: pulleys BELT_W wide.
BALL_R, BELT_W = 0.009, 0.012



def _tube(a, b, radius, material):
    """A tube from `a` to `b` in its segment's frame, its ends rounded."""
    from coaxial.model.blocks import numpy as np
    a, b = np.asarray(a, float), np.asarray(b, float)
    y = (a - b) / np.linalg.norm(a - b)
    x = np.cross(y, (0.0, 0.0, 1.0) if abs(y[2]) < 0.9 else (1.0, 0.0, 0.0))
    x = x / np.linalg.norm(x)
    c, t, u, m = limb(float(np.linalg.norm(b - a)), radius, radius, radius, material)
    return c @ np.stack([x, y, np.cross(x, y)], 1).T + a, t, u, m


def _join(meshes):
    """One mesh of `meshes`."""
    from coaxial.model.blocks import numpy as np
    at = np.cumsum([0] + [len(c) for c, *_ in meshes[:-1]])
    return (np.concatenate([c for c, *_ in meshes]),
            np.concatenate([t + k for (_c, t, _u, _m), k in zip(meshes, at)]),
            np.concatenate([u for _c, _t, u, _m in meshes]),
            np.concatenate([m for *_, m in meshes]))


def _gimbal(side, stage):
    """[(offset, mesh)]: the hip's gimbal on `stage` (`skeleton.gimbal`), in its frame."""
    s, steel = (1.0 if side == 'left_' else -1.0), paint(STEEL)
    tubes, rings = gimbal(stage)
    out = [((0.0, 0.0, 0.0), _tube((s * a[0],) + a[1:], (s * b[0],) + b[1:], r, steel))
           for a, b, r in tubes]
    return out + [((s * c[0],) + c[1:], drum(r, 2.0 * half, axis, steel))
                  for c, axis, r, half in rings]


def _hung(side, seg):
    """[mesh]: `seg`'s tubes through HUNG's points and its collars, its frame."""
    s, c = (1.0 if side == 'left_' else -1.0), paint(CARBON)
    out = [_tube(a, b, r, c) for a, b, r in runs(seg, s)]
    for at, r, half in collars(side, seg):
        corners, t, u, m = drum(r, 2.0 * half, 'x', c)
        out.append((corners + at, t, u, m))
    return out


def _held(side):
    """[(name, rides, offset, mesh)]: `side`'s collars in the printed polymer, its struts, posts
    and the ankle's cross in carbon and steel (`skeleton.held`), and with the left's the trunk's
    frame (`skeleton.trunk`), its bearings and band steel."""
    out = []
    framed = [(f + '_frame', f, sh) for f, sh in trunk()] if side == 'left_' else []
    for k, (name, rides, shape) in enumerate(held(side) + framed):
        if shape[0] == 'ring':
            _kind, centre, axis, r, half = shape
            letter = 'xyz'[[abs(a) for a in axis].index(1.0)]
            ink = paint(STEEL) if name.endswith(('_cross', '_frame')) else paint(POLYMER)
            out.append(('held%d_%s' % (k, name), rides, centre, drum(r, 2.0 * half, letter, ink)))
        else:
            _kind, a, b, r = shape
            out.append(('held%d_%s' % (k, name), rides, (0.0, 0.0, 0.0),
                        _tube(a, b, r, paint(CARBON))))
    return out


def _bones():
    """{segment: mesh}: each segment as its carbon tubes and plates, the limbs', the feet's and
    the trunk's as parts (`_hung`, `_held`)."""
    c = paint(CARBON)
    out = {'pelvis': _join([_tube(a, b, BOOM[0], c) for a, b in zip(BOOM[1], BOOM[1][1:])]),
           'torso': _nothing(loft([(0.0, 0.02, 0.02), (0.39, 0.02, 0.02)], c,
                                  poles=(-0.005, 0.395))),
           'neck': _nothing(loft([(0.0, 0.013, 0.013), (0.065, 0.013, 0.013)], c,
                                 poles=(-0.003, 0.068))),
           'head': ellipsoid((0.0, 0.095, 0.012), (0.05, 0.06, 0.06), c, rows=8)}
    for side in ('left_', 'right_'):
        out.update({side + 'upper_arm': _nothing(limb(UPPER_ARM, 0.012, 0.012, 0.012, c)),
                    side + 'forearm': _nothing(limb(FOREARM, 0.01, 0.01, 0.01, c)),
                    side + 'hand': ellipsoid((0.0, -0.035, 0.0), (0.012, 0.035, 0.02), c, rows=6),
                    side + 'fingers': ellipsoid((0.0, -0.03, 0.0), (0.009, 0.03, 0.015), c,
                                                rows=6),
                    side + 'thigh': _nothing(limb(THIGH, 0.016, 0.016, 0.016, c)),
                    side + 'shank': _nothing(limb(SHANK, 0.014, 0.014, 0.014, c)),
                    side + 'foot': _nothing(ellipsoid(
                        (0.0, 0.012 - ANKLE_H, (BALL - HEEL) / 2.0),
                        (0.034, 0.01, (BALL + HEEL) / 2.0), c, rows=6)),
                    side + 'toes': ellipsoid((0.0, 0.0, 0.03), (0.032, 0.008, 0.03), c, rows=6)})
    return out


def parts(bare=False):
    """[(name, parent, joints, offset, rest, mesh)]: the segments as their bones, the drums, the
    quick-releases, each rod's ball joint on the segment it turns, and each crank, pin and rod
    posed each frame (parent '*'); `bare` nothing of her body: each drive its motor and its
    gearbox, and the linkages."""
    bones = _bones()
    if bare:
        bones = {name: _nothing(mesh) for name, mesh in bones.items()}
    out = [(s[0], s[1], s[2], s[3], s[4], bones[s[0]]) for s in figure.SEGMENTS]
    out += [(name, parent, joints, at, 0.0, mesh)
            for name, parent, joints, at, mesh in drums.stages()]
    for name, parent, offset, (c, t, u, m) in drums.drums():
        joint = name[len('drive_'):]
        size = drives.of(joint)[1]
        sized = paint(SIZED[drives.of(joint)[0]])
        if not bare:
            out.append((name, parent, (), offset, 0.0, (c, t, u, m * 0 + sized)))
            continue
        axis, _half = drums.AXES[joint]
        letter = 'xyz'[[abs(a) for a in axis].index(1.0)]
        ink = {'board': paint(PCB), 'motor': sized, 'gear': paint(GEARBOX)}
        out += [(PARTED[part] + joint, parent, (), tuple(o + a * at for o, a in zip(offset, axis)),
                 0.0, drum(radius, long, letter, ink[part]))
                for part, radius, at, long in drives.along(joint)]
    poly, steel = paint(POLYMER), paint(STEEL)
    if not bare:
        out += [('hung%d_%s%s' % (k, side, seg), side + seg, (), (0.0, 0.0, 0.0), 0.0, mesh)
                for side in ('left_', 'right_') for seg in HUNG
                for k, mesh in enumerate(_hung(side, seg))]
        out += [(name, rides, (), at, 0.0, mesh) for side in ('left_', 'right_')
                for name, rides, at, mesh in _held(side)]
    for joint, (seg, at, _kg, radius, faces) in drives.boards().items():
        out.append(('board_' + joint, seg, (), at, 0.0, drum(radius, BOARD_T, faces, paint(PCB))))
    out += [('gimbal%d_%s' % (k, side + stage), side + stage, (), at, 0.0, mesh)
            for side in ('left_', 'right_') for stage in ('hip_yaw', 'hip_roll')
            for k, (at, mesh) in enumerate(_gimbal(side, stage))]
    # A held joint (`drives.WAYS`) has no drum: no pair on it - the wrists' bevels, held,
    # crashed the page (2026-10-04).
    for joint in [j for kind in linkage.GEARS for j in linkage.joints(kind)
                  if not drives.passive(j)]:
        seg, pinion, r, wheel, big = _spurs(joint)
        out += [('pinion_' + joint, seg, (), pinion, 0.0, drum(r, SPUR_T, 'z', steel)),
                ('spur_' + joint, seg, (), wheel, 0.0, drum(big, SPUR_T, 'z', steel))]
    for joint in [side + kind for kind in linkage.BEVELS for side in ('left_', 'right_')
                  if not drives.passive(side + kind)]:
        seg, drive, letter, driven, r = _bevel(joint)
        out += [('bevel_' + joint, seg, (), drive, 0.0, drum(r, linkage.BEVEL_T, letter, steel)),
                ('bevelo_' + joint, seg, (), driven, 0.0, drum(r, linkage.BEVEL_T, 'x', steel))]
    for side in ('left_', 'right_'):
        out += [('release_' + side + seg, side + seg, (), (0.0, y, 0.0), 0.0,
                 drum(radius, length, 'y', poly)) for seg, (y, radius, length, _kg)
                in build.RELEASES.items()]
        for kind in linkage.RODS:
            joint, rides = side + kind, side + linkage.ROD_AT[kind][0]
            turned = next(s[0] for s in figure.SEGMENTS if rides in [j for j, *_ in s[2]])
            ball = ellipsoid((0.0, 0.0, 0.0), (BALL_R,) * 3, steel, rows=6)
            out += [('end_' + joint, turned, (), _ball(kind, side), 0.0, ball),
                    ('crank_' + joint, '*', (), (0.0, 0.0, 0.0), 0.0,
                     limb(1.0, 0.012, 0.012, 0.012, steel)),
                    ('pin_' + joint, '*', (), (0.0, 0.0, 0.0), 0.0, ball),
                    ('rod_' + joint, '*', (), (0.0, 0.0, 0.0), 0.0,
                     limb(1.0, linkage.ROD_R, linkage.ROD_R, linkage.ROD_R, paint(ROD))),
                    ('rodb_' + joint, '*', (), (0.0, 0.0, 0.0), 0.0,
                     limb(1.0, linkage.ROD_R, linkage.ROD_R, linkage.ROD_R, paint(ROD)))]
    ball = ellipsoid((0.0, 0.0, 0.0), (BALL_R,) * 3, steel, rows=6)
    for joint in [j for kind in linkage.PLANAR for j in linkage.joints(kind)]:
        out += [('crank_' + joint, '*', (), (0.0, 0.0, 0.0), 0.0,
                 limb(1.0, linkage.PLATE_R, linkage.PLATE_R, linkage.PLATE_R, steel)),
                ('horn_' + joint, '*', (), (0.0, 0.0, 0.0), 0.0,
                 limb(1.0, linkage.PLATE_R, linkage.PLATE_R, linkage.PLATE_R, steel)),
                ('pin_' + joint, '*', (), (0.0, 0.0, 0.0), 0.0, ball),
                ('end_' + joint, '*', (), (0.0, 0.0, 0.0), 0.0, ball),
                ('rod_' + joint, '*', (), (0.0, 0.0, 0.0), 0.0,
                 limb(1.0, linkage.ROD_R, linkage.ROD_R, linkage.ROD_R, paint(ROD)))]
    return out


def bands(meshes):
    """[(name, parent, offset, mesh)]: on her shell, `meshes` {segment: mesh}, each limb's
    quick-release (`build.RELEASES`) a polymer band RELEASE_PROUD_M proud where the limb comes off."""
    from coaxial.model.blocks import numpy as np
    out = []
    for side in ('left_', 'right_'):
        for seg, (y, _r, length, _kg) in build.RELEASES.items():
            c = meshes[side + seg][0]
            rings = np.unique(np.round(c[:, 1], 5))
            reach = float(np.interp(y, rings, [np.hypot(*c[np.round(c[:, 1], 5) == r][:, [0, 2]].T)
                                               .max() for r in rings])) + RELEASE_PROUD_M
            out.append(('release_' + side + seg, side + seg, (0.0, y, 0.0),
                        drum(reach, length, 'y', paint(POLYMER))))
    return out


def _spurs(joint):
    """(segment, pinion's centre, radius, wheel's centre, radius): a pair on a drive along z."""
    seg, (x, y, z) = drives.mount(joint) or ('', (0.0, 0.0, 0.0))
    pivot = drives.pivot(joint)
    face = z + drums.AXES[joint][1] + SPUR_T / 2.0
    apart = math.hypot(pivot[0] - x, pivot[1] - y)
    r = apart / (1.0 + linkage.GEARS[drives.kind(joint)])
    return seg, (x, y, face), r, (pivot[0], pivot[1], face), apart - r


def _bevel(joint):
    """(segment, the drive's gear's centre, its axis letter, the driven's centre, radius): a
    bevel pair at its gearbox's end, the driven's axis x out to its side (`drives.outlet`)."""
    letter, end = drives.output(joint) or ('y', 1.0)
    seg, at = drives.mount(joint) or ('', (0.0, 0.0, 0.0))
    r, i = linkage.BEVELS[drives.kind(joint)], 'xyz'.index(letter)
    reach = drums.AXES[joint][1] + linkage.BEVEL_T / 2.0
    drive = tuple(v + end * reach if k == i else v for k, v in enumerate(at))
    o = (drives.outlet(joint) or (seg, at))[1]
    return seg, drive, letter, (o[0] + (r if joint.startswith('left_') else -r),) + o[1:], r


def _ball(kind, side):
    """A rod's ball joint on the segment its joint turns, its frame (`linkage.RODS`)."""
    x, y, z = linkage.ball(kind)
    return (x if side == 'left_' else -x, y, z)


def _hub(kind, side):
    """(segment, point): a rod's crank's hub on its drive's segment, about the joint's place."""
    up, ahead = linkage.RODS[kind][0][:2]
    seg = next(s for s in figure.SEGMENTS if side + linkage.ROD_AT[kind][0]
               in [j for j, *_ in s[2]])
    return seg[1], (seg[3][0], seg[3][1] + up, seg[3][2] + ahead)


def belts(np, placed):
    """[(a, b)]: each belt as drawn, world - its two pulleys' rims and its two runs, out to her
    side, the drive's pulley on its axis, the joint's on the joint's."""
    out = []
    for side in ('left_', 'right_'):
        for kind, (r1, r2, off) in linkage.BELTS.items():
            joint, where = side + kind, drives.mount(side + kind)
            if drives.passive(joint) or where is None:
                continue
            where = drives.outlet(joint) or where
            seat, (x, y, z) = where
            turn, spot = placed[seat]
            out_x = off if side == 'left_' else -off
            seg = next(s for s in figure.SEGMENTS if joint in [j for j, *_ in s[2]])
            c1, c2 = np.array([y, z]), np.array(seg[3][1:])
            d = c2 - c1
            u = d / np.linalg.norm(d)
            v = np.array([-u[1], u[0]])
            g = (r2 - r1) / np.linalg.norm(d)
            for s in (1.0, -1.0):
                n = -g * u + s * math.sqrt(1.0 - g * g) * v
                a, b = c1 + r1 * n, c2 + r2 * n
                out.append((spot + turn @ np.array([out_x, a[0], a[1]]),
                            spot + turn @ np.array([out_x, b[0], b[1]])))
            axis = turn @ np.array([1.0, 0.0, 0.0])
            for c, r in ((c1, r1), (c2, r2)):
                out += _rims(np, spot + turn @ np.array([out_x, c[0], c[1]]), axis, r,
                             BELT_W / 2.0)
    return out






def _nothing(mesh):
    """A mesh with no corners: a segment kept for its frame, not drawn."""
    c, t, u, m = mesh
    return c[:0], t[:0], u[:0], m[:0]




def _along(np, a, b):
    """(turn, spot) laying a part hung from its origin down -y a unit long from `a` to `b`."""
    d = b - a
    length = float(np.linalg.norm(d))
    u = d / length if length > 1e-9 else np.array([0.0, -1.0, 0.0])
    w = np.array([1.0, 0.0, 0.0]) if abs(u[0]) < 0.9 else np.array([0.0, 0.0, 1.0])
    x = np.cross(w, u)
    x /= np.linalg.norm(x)
    return np.column_stack([x, -u * length, np.cross(x, u)]), a


def posed(parts_, placed, angles, frames):
    """Each '*' part's (turn, spot) into `frames` (a part an entry, theirs None): a crank at its
    closure's angle (`linkage.crank`) in its plane (`linkage.ROD_AT`), its pin at its end, its
    rod to its ball joint - `placed` {segment: (turn, spot)}, world."""
    from coaxial.model.blocks import numpy as np
    at = {name: i for i, (name, *_rest) in enumerate(parts_)}
    for side in ('left_', 'right_'):
        for kind in linkage.RODS:
            joint = side + kind
            seat, centre = _hub(kind, side)
            turn, spot = placed[seat]
            r, t0 = linkage.RODS[kind][0][2:4]
            t = t0 + linkage.crank(joint, *linkage.turns(joint, angles))
            s = 1.0 if side == 'left_' else -1.0
            hub = spot + turn @ (np.asarray(centre, float)
                                 + np.array([s * linkage.ROD_AT[kind][1], 0.0, 0.0]))
            pin = hub + turn @ np.array([0.0, -r * math.cos(t), r * math.sin(t)])
            seg_turn, seg_spot = placed[parts_[at['end_' + joint]][1]]
            ball = seg_spot + seg_turn @ np.asarray(_ball(kind, side), float)
            bend = np.array(linkage.bent(kind, pin, ball, s * turn[:, 0], s))
            frames[at['crank_' + joint]] = _along(np, hub, pin)
            frames[at['pin_' + joint]] = (np.eye(3), pin)
            frames[at['rod_' + joint]] = _along(np, pin, bend)
            frames[at['rodb_' + joint]] = _along(np, bend, ball)
    for joint in [j for kind in linkage.PLANAR for j in linkage.joints(kind)]:
        crank, horn, pin, ball = (np.array(p) for p in linkage.four_bar(joint, angles))
        turn, spot = placed[(drives.mount(joint) or ('pelvis',))[0]]
        crank, horn, pin, ball = (spot + turn @ p for p in (crank, horn, pin, ball))
        frames[at['crank_' + joint]] = _along(np, crank, pin)
        frames[at['horn_' + joint]] = _along(np, horn, ball)
        frames[at['pin_' + joint]] = (np.eye(3), pin)
        frames[at['end_' + joint]] = (np.eye(3), ball)
        frames[at['rod_' + joint]] = _along(np, pin, ball)


def _rims(np, centre, axis, radius, half):
    """A wire cylinder about `axis` through `centre`: its two rims and four sides, [(a, b)]."""
    w = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(axis, w)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    round_ = [radius * (np.cos(t) * u + np.sin(t) * v)
              for t in np.linspace(0.0, 2.0 * np.pi, RIM, endpoint=False)]
    ends = [centre - axis * half, centre + axis * half]
    out = [(end + round_[k], end + round_[(k + 1) % RIM]) for end in ends for k in range(RIM)]
    return out + [(ends[0] + round_[k], ends[1] + round_[k]) for k in range(0, RIM, RIM // 4)]


def wires(parts_, frames):
    """[(a (n, 3), b (n, 3), ink)]: the stick figure's segments, world, in layers drawn in order -
    each bone from its parent's joint to its own, the shoulders off the torso's top, the head, the
    fingers, the sole; each motor and gearbox a wire cylinder on its axis; each crank and rod; each
    ball joint a cross."""
    from coaxial.model.blocks import numpy as np
    at = {p[0]: f for p, f in zip(parts_, frames)}
    bones, rods, balls, drives_, releases = [], [], [], {}, []
    turn, spot = at['pelvis']
    boom = [spot + turn @ np.array(p) for p in BOOM[1]]
    bones += list(zip(boom, boom[1:]))
    for side in ('left_', 'right_'):
        s = 1.0 if side == 'left_' else -1.0
        for stage in ('hip_yaw', 'hip_roll'):
            turn, spot = at[side + stage]
            tubes, rings = gimbal(stage)
            rods += [(spot + turn @ np.array((s * a[0],) + a[1:]),
                      spot + turn @ np.array((s * b[0],) + b[1:])) for a, b, _r in tubes]
            for c, axis, r, half in rings:
                rods += _rims(np, spot + turn @ np.array((s * c[0],) + c[1:]),
                              turn @ np.eye(3)['xyz'.index(axis)], r, half)
    for name, parent, *_rest in parts_[:len(figure.SEGMENTS)]:
        if parent == 'pelvis' and name.endswith('_thigh'):
            continue
        if parent:
            turn, spot = at[parent]
            root = spot + turn @ np.array([0.0, 0.325, 0.0]) if name.endswith('upper_arm') else spot
            held = build.RELEASES.get(parent.split('_', 1)[-1])
            if held is None:
                bones.append((root, at[name][1]))
                continue
            y, _r, length, _kg = held
            u = at[name][1] - root
            u = u / np.linalg.norm(u)
            bones += [(root, root + u * (-y - length / 2.0 - RELEASE_GAP)),
                      (root + u * (-y + length / 2.0 + RELEASE_GAP), at[name][1])]
    for name, tip in (('head', (0.0, 0.19, 0.0)), ('left_fingers', (0.0, -0.07, 0.0)),
                      ('right_fingers', (0.0, -0.07, 0.0)), ('left_toes', (0.0, 0.0, 0.06)),
                      ('right_toes', (0.0, 0.0, 0.06))):
        turn, spot = at[name]
        bones.append((spot, spot + turn @ np.array(tip)))
    for side in ('left_', 'right_'):
        turn, spot = at[side + 'foot']
        heel, ball = (spot + turn @ np.array(p) for p in ((0.0, -ANKLE_H, -HEEL),
                                                         (0.0, -ANKLE_H, BALL)))
        bones += [(spot, heel), (heel, ball), (ball, spot)]
    for (name, parent, *_rest), (turn, spot) in zip(parts_, frames):
        if name.startswith(('drive_', 'gear_', 'inv_')):
            prefix, joint = name.split('_', 1)
            axis, _half = drums.AXES[joint]
            part = {'drive': 'motor', 'gear': 'gear', 'inv': 'board'}[prefix]
            _p, radius, _at, long = next(r for r in drives.along(joint) if r[0] == part)
            ink = {'motor': SIZED[drives.of(joint)[0]], 'gear': GEARBOX, 'board': PCB}[part]
            drives_.setdefault(ink, []).extend(
                _rims(np, spot, turn @ np.asarray(axis, float), radius, long / 2.0))
        elif name.startswith(('crank_', 'rod_', 'rodb_')):
            rods.append((spot, spot + turn @ np.array([0.0, -1.0, 0.0])))
        elif name.startswith(('pin_', 'end_')):
            balls += [(spot - d, spot + d) for d in np.eye(3) * BALL_R]
        elif name.startswith(('bevel_', 'bevelo_')):
            _seg, _d, letter, _o, r = _bevel(name.split('_', 1)[1])
            axis = np.eye(3)['xyz'.index(letter if name.startswith('bevel_') else 'x')]
            rods += _rims(np, spot, turn @ axis, r, linkage.BEVEL_T / 2.0)
        elif name.startswith(('pinion_', 'spur_')):
            _seg, _p, r, _w, big = _spurs(name.split('_', 1)[1])
            rods += _rims(np, spot, turn @ np.array([0.0, 0.0, 1.0]),
                          r if name.startswith('pinion_') else big, SPUR_T / 2.0)
        elif name.startswith('release_'):
            _y, radius, length, _kg = build.RELEASES[name.split('_', 2)[2]]
            releases += _rims(np, spot, turn @ np.array([0.0, 1.0, 0.0]), radius, length / 2.0)
    placed = {p[0]: f for p, f in zip(parts_[:len(figure.SEGMENTS)], frames)}
    layers = [(bones, BONE_INK)] + [(lines, ink) for ink, lines in drives_.items()]
    layers += [(rods + belts(np, placed), ROD_INK), (balls, BALL_INK), (releases, POLYMER)]
    return [(np.array([a for a, _ in lines]), np.array([b for _, b in lines]), ink)
            for lines, ink in layers if lines]

