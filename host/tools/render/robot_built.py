"""The build whole: every sketched mount where the figure carries it, standing; the rest the page's.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/robot_built.py OUT_DIR [--px N]

The sketches lie each in a frame of its own: the leg's and the hip's in the knee's (`leg_worn`), the arm's and the
shoulder's in the elbow's (`arm_worn`), the neck's, the head's and the bay's in the torso's (`neck_mount`,
`neck_hosed`), the trunk's in its joint's (`trunk_clamped`, a first draft: the pelvis's column, the drive pair, the
cardan joint at mid back, the chest's fork the spine's tube stands on - the prints `trunk_shaped` has shaped by
their loads drawn as their solids). The page's body ties those frames, the
robot at rest (`gynoid.body`): here each sketch is laid where its segment stands, the right side the left's
mirror. What no sketch draws yet - the hands - is the page's own, bare, in one faint ink (GHOST): seen for what
is missing. `sheets` draws the whole from ahead, from its side and from behind, and close on each of its regions,
a picture a file.
"""
import argparse
import functools
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import mounting as m  # noqa: E402

#: The ink of what no sketch draws yet, and the page's parts that are so: the hands.
GHOST = (62, 84, 116)
UNDRAWN = ('left_hand', 'left_fingers', 'right_hand', 'right_fingers')
#: The whole's views: (name, yaw deg, pitch deg); the regions drawn close: {name: (the page's segments whose
#: joints span it, half its picture's width m, its views)}.
VIEWS = (('ahead', 20.0, 6.0), ('side', 90.0, 4.0), ('behind', 200.0, 6.0))
CLOSE = {'hip': (('left_thigh', 'pelvis'), 0.20, (('ahead', 35.0, 10.0), ('behind', 215.0, 10.0))),
         'knee': (('left_shank', 'left_shank'), 0.20, (('ahead', 35.0, 10.0), ('behind', 215.0, 10.0))),
         'foot': (('left_foot', 'left_foot'), 0.18, (('side', 90.0, 8.0), ('behind', 215.0, 10.0))),
         'shoulder': (('left_upper_arm', 'left_upper_arm'), 0.18, (('ahead', 35.0, 10.0), ('behind', 215.0, 10.0))),
         'elbow': (('left_forearm', 'left_forearm'), 0.16, (('ahead', 35.0, 10.0), ('behind', 215.0, 10.0))),
         'neck': (('neck', 'neck'), 0.20, (('ahead', 25.0, 10.0), ('behind', 205.0, 10.0))),
         'trunk': (('torso', 'neck'), 0.26, (('ahead', 35.0, 10.0), ('side', 90.0, 4.0), ('behind', 215.0, 10.0)))}


@functools.lru_cache(maxsize=None)
def _figure():
    """(the page's bare body, its parts' names, their frames at rest, its corners there): the root's frame."""
    import numpy as np
    from coaxial.graphics import gynoid
    body = gynoid.body(False, None, None, True)
    names = [part[0] for part in body.parts]
    root = ((0.0, 0.0, 0.0), np.eye(3))
    frames = body._frames({}, root)
    pos, _n = body.pose({}, root=root, _frames=frames)
    return body, names, frames, pos


def _put(tris, segment, pivot):
    """`tris` of a sketch in `segment`'s frame at `pivot`'s origin, laid where the figure stands them."""
    _body, names, frames, _pos = _figure()
    turn, spot = frames[names.index(segment)][0], frames[names.index(pivot)][1]
    return [(tri @ turn.T + spot, colour) for tri, colour in tris]


def _mirrored(tris):
    """`tris` on the robot's other side: mirrored in its middle, their faces kept outward."""
    return [((tri * (-1.0, 1.0, 1.0))[::-1], colour) for tri, colour in tris]


def _undrawn():
    """What no sketch draws yet, the page's own in GHOST."""
    import numpy as np
    body, names, _frames, pos = _figure()
    owner = np.repeat(np.arange(len(names)), [len(part[5][1]) for part in body.parts])
    index = body.index.astype(int)
    return [(pos[tri], GHOST) for n in names if n in UNDRAWN for tri in index[owner == names.index(n)]]


@functools.lru_cache(maxsize=None)
def whole():
    """{name: tris} of the build, the root's frame (the pelvis's: x to the robot's left, y up, z ahead), standing:
    'left leg', 'right leg' - hip, knee, calf and foot with their guards and cables -, 'left arm', 'right arm' -
    shoulder, elbow, their tubes, hoses and guards -, 'neck' - the torso's block, the spine's tube, the bay, the neck's and
    the head's drives, their leads -, 'trunk' - the pelvis's column, the drive pair, the cardan joint, the chest's
    fork, upright, the hips' yaw drives the legs' own - and 'undrawn'."""
    import numpy as np
    from tools.render import arm_worn as aw, leg_clamped as lc, leg_worn as lw, neck_hosed as nh, neck_mount as nm
    from tools.render import trunk_clamped as tc, trunk_laid as tl, trunk_shaped as ts
    up, down = lw._bare(lc.stated(), 0.0)
    leg = _put([t for part in dict(up, **down).values() for t in part], 'left_thigh', 'left_shank')
    arm = _put([t for part in list(aw.arm(0.0).values()) + list(aw._guarded().values()) for t in part],
               'left_upper_arm', 'left_forearm')
    neck = [t for name, part in dict(nm.posed(low=tc.seat()), **nh.posed()).items() if not name.endswith(' shoulder')
            for t in part]
    at = np.asarray(tl.joint()[0], float)
    trunk = [(tri + at, colour) for (link, name), part in ts.posed().items() if link != 'hips' and name != 'tube'
             for tri, colour in part]
    return {'left leg': leg, 'right leg': _mirrored(leg), 'left arm': arm, 'right arm': _mirrored(arm),
            'neck': _put(neck, 'torso', 'torso'), 'trunk': _put(trunk, 'pelvis', 'pelvis'), 'undrawn': _undrawn()}


def sheets(out, px=2400):
    """[path]: the whole from each of VIEWS and each region of CLOSE from its own, a PNG each in the folder `out`,
    `px` a side."""
    import numpy as np
    _body, names, frames, _pos = _figure()
    tris = [t for part in whole().values() for t in part]
    corners = np.vstack([t for t, _c in tris])
    low, high = corners.min(axis=0), corners.max(axis=0)
    shots = [('whole from ' + name, 0.5 * (low + high), 0.52 * float((high - low).max()), yaw, pitch)
             for name, yaw, pitch in VIEWS]
    for region, ((a, b), half, views) in CLOSE.items():
        centre = 0.5 * (frames[names.index(a)][1] + frames[names.index(b)][1])
        shots += [('%s from %s' % (region, name), centre, half, yaw, pitch) for name, yaw, pitch in views]
    os.makedirs(out, exist_ok=True)
    paths = []
    for title, centre, half, yaw, pitch in shots:
        near = [(t, c) for t, c in tris if np.abs(t.mean(axis=0) - centre).max() < 1.6 * half]
        path = os.path.join(out, title.replace(' ', '_') + '.png')
        m.draw(near, yaw=yaw, pitch=pitch, px=px, half=half, centre=tuple(centre)).save(path)
        paths.append(path)
    return paths


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    parser.add_argument('--px', type=int, default=2400)
    args = parser.parse_args(argv)
    for path in sheets(args.out, args.px):
        print(path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
