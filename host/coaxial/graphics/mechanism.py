"""Her mechanism without her shell: carbon tubes, drums, rods on ball joints, quick-releases.

    parts = mechanism.parts()               # [(name, parent, joints, offset, rest, mesh)]
    parts = mechanism.parts(bare=True)      # the actuators alone: drums, cranks, rods, balls
    mechanism.posed(parts, placed, angles, frames)   # each crank's, pin's and rod's (turn, spot)

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
from machine import drives, figure
from machine.figure import FOREARM, UPPER_ARM
from machine.gait import ANKLE_H, BALL, HEEL, SHANK, THIGH

#: The colours: the carbon's, the rods' and their ball joints' steel, the quick-releases' printed
#: polymer, and each drive's drum by its size (`drives.SIZES`).
CARBON, ROD, STEEL, POLYMER = (92, 94, 106), (214, 214, 224), (246, 246, 246), (232, 122, 32)
SIZED = {'L': (72, 140, 224), 'M': (60, 190, 170), 'S': (230, 190, 70)}

#: Each rod by its joint's kind: its crank's radius, m, which way it points at rest (its drive's
#: segment's frame), and the ball joint it drives, on the next segment - the ankle's to the heel's
#: tuberosity behind and under the ankle, the knee's to the tibial tuberosity before and under the
#: knee. The tube ROD_R round, a ball joint BALL_R.
RODS = {'ankle': (0.030, (0.0, 0.0, -1.0), (0.0, -0.03, -0.05)),
        'knee': (0.028, (0.0, 0.0, 1.0), (0.0, -0.045, 0.04))}
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
    posed each frame (parent '*'); `bare` the drums and the linkages alone, the segments drawn
    as nothing but the soles that stand her on the floor."""
    bones = _bones()
    if bare:
        bones = {name: mesh if name.endswith(('_foot', '_toes')) else _nothing(mesh)
                 for name, mesh in bones.items()}
    out = [(s[0], s[1], s[2], s[3], s[4], bones[s[0]]) for s in figure.SEGMENTS]
    for name, parent, offset, (c, t, u, m) in drums.drums():
        sized = paint(SIZED[drives.of(name[len('drive_'):])[0]])
        out.append((name, parent, (), offset, 0.0, (c, t, u, m * 0 + sized)))
    poly, steel = paint(POLYMER), paint(STEEL)
    for side in ('left_', 'right_'):
        out += [] if bare else [
            ('release_' + side + 'arm', side + 'upper_arm', (), (0.0, -0.015, 0.0), 0.0,
             drum(0.022, 0.018, 'y', poly)),
            ('release_' + side + 'leg', side + 'thigh', (), (0.0, -0.02, 0.0), 0.0,
             drum(0.03, 0.022, 'y', poly))]
        for kind, (_crank, _rest, end) in RODS.items():
            joint = side + kind
            turned = next(s[0] for s in figure.SEGMENTS if joint in [j for j, *_ in s[2]])
            ball = ellipsoid((0.0, 0.0, 0.0), (BALL_R,) * 3, steel, rows=6)
            out += [('end_' + joint, turned, (), end, 0.0, ball),
                    ('crank_' + joint, '*', (), (0.0, 0.0, 0.0), 0.0,
                     limb(1.0, 0.012, 0.012, 0.012, steel)),
                    ('pin_' + joint, '*', (), (0.0, 0.0, 0.0), 0.0, ball),
                    ('rod_' + joint, '*', (), (0.0, 0.0, 0.0), 0.0,
                     limb(1.0, ROD_R, ROD_R, ROD_R, paint(ROD)))]
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
            sign = next(s for seg in figure.SEGMENTS for j, _a, s in seg[2] if j == joint)
            a = math.radians(sign * float(angles.get(joint, 0.0)) * drives.LINKS[kind])
            pointing = np.array([rest[0], rest[1] * math.cos(a) - rest[2] * math.sin(a),
                                 rest[1] * math.sin(a) + rest[2] * math.cos(a)])
            hub = spot + turn @ np.asarray(centre, float)
            pin = hub + turn @ (crank * pointing)
            seg_turn, seg_spot = placed[parts_[at['end_' + joint]][1]]
            ball = seg_spot + seg_turn @ np.asarray(end, float)
            frames[at['crank_' + joint]] = _along(np, hub, pin)
            frames[at['pin_' + joint]] = (np.eye(3), pin)
            frames[at['rod_' + joint]] = _along(np, pin, ball)
