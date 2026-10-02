"""Her mechanism without her shell: carbon tubes, drums, rods on ball joints, quick-releases.

    parts = mechanism.parts()               # [(name, parent, joints, offset, rest, mesh)]
    parts = mechanism.parts(bare=True)      # motors, gearboxes, cranks, rods, balls: no body
    mechanism.posed(parts, placed, angles, frames)   # each crank's, pin's and rod's (turn, spot)
    layers = mechanism.wires(parts, frames)  # the kinematic stick figure: [(a, b, ink)], world

The segments are their carbon tubes and plates (`machine.build`); each drive's drum where it sits
(`drums`); a limb's quick-release a printed collar at its root; a rod from its drive's crank to a
ball joint on the segment it turns, the crank turned by its joint's angle times its lever ratio
(`drives.LINKS`). A crank, a rod and the crank's pin are posed each frame, `posed`, their parent
'*'.
"""
import math

from coaxial.graphics import drums
from coaxial.graphics.lit import paint
from coaxial.graphics.shapes import drum, ellipsoid, limb, loft
from machine import build, drives, figure
from machine.figure import FOREARM, UPPER_ARM
from machine.gait import ANKLE_H, BALL, HEEL, SHANK, THIGH

#: The colours: the carbon's, the rods' and their ball joints' steel, the quick-releases' printed
#: polymer, and each drive's drum by its size (`drives.SIZES`).
CARBON, ROD, STEEL, POLYMER = (92, 94, 106), (214, 214, 224), (246, 246, 246), (232, 122, 32)
SIZED = {'L': (72, 140, 224), 'M': (60, 190, 170), 'S': (230, 190, 70)}

#: Bare, a drive's drum drawn as its motor, MOTOR_SHARE of its length in its size's colour, and its
#: gearbox beside it on the axis, GEAR_RADIUS of its radius, in the gearbox's steel grey.
MOTOR_SHARE, GEAR_RADIUS, GEARBOX = 0.6, 0.8, (150, 152, 160)

#: The stick figure's inks: the skeleton's, the cranks' and rods', the ball joints'; a wire
#: cylinder's rims RIM points round, a ball joint a cross BALL_R across; a bone broken RELEASE_GAP
#: clear either side of its quick-release, the collar in its polymer.
BONE_INK, ROD_INK, BALL_INK, RIM = (200, 200, 206), (250, 150, 60), (255, 255, 255), 10
RELEASE_GAP = 0.006
#: On her shell a quick-release's band stands RELEASE_PROUD_M proud of it (`bands`).
RELEASE_PROUD_M = 0.001

#: Each rod by its joint's kind: its crank's radius, m, which way it points at rest (its drive's
#: segment's frame), and the ball joint it drives, on the next segment - the ankle's to the heel's
#: tuberosity behind and under the ankle, the knee's to the tibial tuberosity before and under the
#: knee. The tube ROD_R round, a ball joint BALL_R.
RODS = {'ankle': (0.030, (0.0, 0.0, -1.0), (0.0, -0.03, -0.05)),
        'knee': (0.028, (0.0, 0.0, 1.0), (0.0, -0.045, 0.04)),
        'hip': (0.030, (0.0, -1.0, 0.0), (0.0, -0.09, -0.035)),
        'hip_roll': (0.030, (0.0, -1.0, 0.0), (0.035, -0.06, 0.0))}
ROD_R, BALL_R = 0.009, 0.013


def _bones():
    """{segment: mesh}: each segment as its carbon tubes and plates."""
    c = paint(CARBON)
    out = {'pelvis': drum(0.02, 0.2, 'x', c),
           'torso': loft([(0.0, 0.02, 0.02), (0.39, 0.02, 0.02)], c, poles=(-0.005, 0.395)),
           'neck': loft([(0.0, 0.013, 0.013), (0.065, 0.013, 0.013)], c, poles=(-0.003, 0.068)),
           'head': ellipsoid((0.0, 0.095, 0.012), (0.05, 0.06, 0.06), c, rows=8)}
    for side in ('left_', 'right_'):
        out.update({side + 'upper_arm': limb(UPPER_ARM, 0.012, 0.012, 0.012, c),
                    side + 'forearm': limb(FOREARM, 0.01, 0.01, 0.01, c),
                    side + 'hand': ellipsoid((0.0, -0.035, 0.0), (0.012, 0.035, 0.02), c, rows=6),
                    side + 'fingers': ellipsoid((0.0, -0.03, 0.0), (0.009, 0.03, 0.015), c,
                                                rows=6),
                    side + 'thigh': limb(THIGH, 0.016, 0.016, 0.016, c),
                    side + 'shank': limb(SHANK, 0.014, 0.014, 0.014, c),
                    side + 'foot': ellipsoid((0.0, 0.012 - ANKLE_H, (BALL - HEEL) / 2.0),
                                             (0.034, 0.01, (BALL + HEEL) / 2.0), c, rows=6),
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
    for name, parent, offset, (c, t, u, m) in drums.drums():
        joint = name[len('drive_'):]
        size = drives.of(joint)[1]
        sized = paint(SIZED[drives.of(joint)[0]])
        if not bare:
            out.append((name, parent, (), offset, 0.0, (c, t, u, m * 0 + sized)))
            continue
        axis, half = drums.AXES[joint]
        motor, gear = 2.0 * half * MOTOR_SHARE, 2.0 * half * (1.0 - MOTOR_SHARE)
        letter = 'xyz'[axis.index(1.0)]
        if letter == 'x' and joint.startswith('right_'):
            axis = (-1.0, 0.0, 0.0)
        out += [(name, parent, (), tuple(o - a * gear / 2.0 for o, a in zip(offset, axis)), 0.0,
                 drum(size.diameter / 2.0, motor, letter, sized)),
                ('gear_' + joint, parent, (), tuple(o + a * motor / 2.0 for o, a in zip(offset, axis)),
                 0.0, drum(size.diameter / 2.0 * GEAR_RADIUS, gear, letter, paint(GEARBOX)))]
    poly, steel = paint(POLYMER), paint(STEEL)
    for side in ('left_', 'right_'):
        out += [('release_' + side + seg, side + seg, (), (0.0, y, 0.0), 0.0,
                 drum(radius, length, 'y', poly)) for seg, (y, radius, length, _kg)
                in build.RELEASES.items()]
        for kind, (_crank, _rest, end) in RODS.items():
            joint = side + kind
            turned = next(s[0] for s in figure.SEGMENTS if joint in [j for j, *_ in s[2]])
            ball = ellipsoid((0.0, 0.0, 0.0), (BALL_R,) * 3, steel, rows=6)
            out += [('end_' + joint, turned, (), _sided(end, side), 0.0, ball),
                    ('crank_' + joint, '*', (), (0.0, 0.0, 0.0), 0.0,
                     limb(1.0, 0.012, 0.012, 0.012, steel)),
                    ('pin_' + joint, '*', (), (0.0, 0.0, 0.0), 0.0, ball),
                    ('rod_' + joint, '*', (), (0.0, 0.0, 0.0), 0.0,
                     limb(1.0, ROD_R, ROD_R, ROD_R, paint(ROD)))]
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


def _sided(point, side):
    """`point` on `side`: the right's x mirrored."""
    return (-point[0] if side == 'right_' else point[0], point[1], point[2])


def _turned(v, axis, a):
    """`v` turned `a` rad about its frame's `axis` ('x', 'y' or 'z')."""
    c, s = math.cos(a), math.sin(a)
    i, j = {'x': (1, 2), 'y': (2, 0), 'z': (0, 1)}[axis]
    out = v.copy()
    out[i], out[j] = v[i] * c - v[j] * s, v[i] * s + v[j] * c
    return out


def _nothing(mesh):
    """A mesh with no corners: a segment kept for its frame, not drawn."""
    c, t, u, m = mesh
    return c[:0], t[:0], u[:0], m[:0]


def _mount(joint):
    """(segment, offset) its rod's drive sits at."""
    where = drives.mount(joint)
    if where is None:
        raise ValueError('%s drives no rod: its drive sits on its axis' % joint)
    return where


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
    """Each '*' part's (turn, spot) into `frames` (a part an entry, theirs None): a crank turned
    from its rest by its joint's angle times its lever, its pin at its end, its rod to its ball
    joint - `placed` {segment: (turn, spot)}, world."""
    from coaxial.model.blocks import numpy as np
    at = {name: i for i, (name, *_rest) in enumerate(parts_)}
    for side in ('left_', 'right_'):
        for kind, (crank, rest, end) in RODS.items():
            joint = side + kind
            seat, centre = _mount(joint)
            turn, spot = placed[seat]
            axis, sign = next((ax, s) for seg in figure.SEGMENTS for j, ax, s in seg[2]
                              if j == joint)
            a = math.radians(sign * float(angles.get(joint, 0.0)) * drives.LINKS[kind])
            pointing = _turned(np.asarray(_sided(rest, side), float), axis, a)
            hub = spot + turn @ np.asarray(centre, float)
            pin = hub + turn @ (crank * pointing)
            seg_turn, seg_spot = placed[parts_[at['end_' + joint]][1]]
            ball = seg_spot + seg_turn @ np.asarray(end, float)
            frames[at['crank_' + joint]] = _along(np, hub, pin)
            frames[at['pin_' + joint]] = (np.eye(3), pin)
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
    for name, parent, *_rest in parts_[:len(figure.SEGMENTS)]:
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
        if name.startswith(('drive_', 'gear_')):
            joint = name.split('_', 1)[1]
            size, (axis, half) = drives.of(joint), drums.AXES[joint]
            share = MOTOR_SHARE if name.startswith('drive_') else 1.0 - MOTOR_SHARE
            radius = size[1].diameter / 2.0 * (1.0 if name.startswith('drive_') else GEAR_RADIUS)
            ink = SIZED[size[0]] if name.startswith('drive_') else GEARBOX
            drives_.setdefault(ink, []).extend(
                _rims(np, spot, turn @ np.asarray(axis, float), radius, half * share))
        elif name.startswith(('crank_', 'rod_')):
            rods.append((spot, spot + turn @ np.array([0.0, -1.0, 0.0])))
        elif name.startswith(('pin_', 'end_')):
            balls += [(spot - d, spot + d) for d in np.eye(3) * BALL_R]
        elif name.startswith('release_'):
            _y, radius, length, _kg = build.RELEASES[name.split('_', 2)[2]]
            releases += _rims(np, spot, turn @ np.array([0.0, 1.0, 0.0]), radius, length / 2.0)
    layers = [(bones, BONE_INK)] + [(lines, ink) for ink, lines in drives_.items()]
    layers += [(rods, ROD_INK), (balls, BALL_INK), (releases, POLYMER)]
    return [(np.array([a for a, _ in lines]), np.array([b for _, b in lines]), ink)
            for lines, ink in layers if lines]

