"""The build whole: every sketched mount where the figure carries it, standing; the rest the page's.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/robot_built.py OUT_DIR [--px N]

The sketches lie each in a frame of its own: the leg's and the hip's in the knee's (`leg_worn`, `leg_shaped`), the arm's
and the shoulder's in the elbow's (`arm_worn`), the neck's, the head's and the bay's in the torso's (`neck_mount`,
`neck_hosed`), the trunk's in its joint's (`trunk_clamped`, a first draft: the pelvis's yoke, the cardan joint low
by the hips, the drive pair on the chest and the seat the spine's tube stands on - the prints `trunk_shaped` has
shaped by their loads drawn as their solids -, its guards as the page lays them clear of it, `trunk_worn`). The
page's body ties those frames, the
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


def _undrawn(roll=0.0):
    """What no sketch draws yet, the page's own in GHOST (hands rotated with `roll` when raised)."""
    import math
    import numpy as np
    body, names, frames, pos = _figure()
    owner = np.repeat(np.arange(len(names)), [len(part[5][1]) for part in body.parts])
    index = body.index.astype(int)
    out = []
    for n in names:
        if n not in UNDRAWN:
            continue
        tris = [(pos[tri], GHOST) for tri in index[owner == names.index(n)]]
        if roll:
            from tools.render import shoulder_rolled as sr
            side = 'left_' if n.startswith('left_') else 'right_'
            sgn = 1.0 if side == 'left_' else -1.0
            turn_s, spot_s = frames[names.index(side + 'upper_arm')]
            piv = spot_s + np.array([-sgn * sr.X_ABD, 0.0, 0.0]) @ turn_s.T
            c_r, s_r = math.cos(math.radians(sgn * roll)), math.sin(math.radians(sgn * roll))
            rz = np.array([[c_r, -s_r, 0.0], [s_r, c_r, 0.0], [0.0, 0.0, 1.0]])
            tris = [((t - piv) @ rz.T + piv, col) for t, col in tris]
        out += tris
    return out


@functools.lru_cache(maxsize=None)
def whole(roll=0.0):
    """{name: tris} of the build, the root's frame (the pelvis's: x to the robot's left, y up, z ahead), standing
    (with the arms raised `roll` deg when told, e.g. `roll=90.0` for T-pose): legs, arms, bilateral pectoral drives
    with SIMP chest mounts and carbon pushrods, neck/bay, trunk, trunk guards and undrawn hands."""
    import math
    import numpy as np
    from tools.render import arm_worn as aw, leg_clamped as lc, leg_shaped as ls, leg_worn as lw
    from tools.render import neck_hosed as nh, neck_mount as nm, shoulder_rolled as sr
    from tools.render import trunk_clamped as tc, trunk_laid as tl, trunk_shaped as ts, trunk_worn as tw
    leg = _put(ls.posed(lc.stated(), 0.0) + [t for part in lw.guards(lc.stated()).values() for t in part],
               'left_thigh', 'left_shank')
    guards_a = [t for part in aw._guarded().values() for t in part]
    if roll:
        piv_a = aw.laid(False, ())[1] + np.array([-sr.X_ABD, 0.0, 0.0])
        c_r, s_r = math.cos(math.radians(roll)), math.sin(math.radians(roll))
        rz = np.array([[c_r, -s_r, 0.0], [s_r, c_r, 0.0], [0.0, 0.0, 1.0]])
        guards_a = [((t - piv_a) @ rz.T + piv_a, col) for t, col in guards_a]
    arm = _put([t for part in aw.arm(0.0, roll).values() for t in part] + guards_a,
               'left_upper_arm', 'left_forearm')
    pec = sr.chest(roll, False)['pectoral'] + sr.chest(roll, True)['pectoral']
    neck = [t for name, part in dict(nm.posed(low=tc.seat()), **nh.posed()).items() if not name.endswith(' shoulder')
            for t in part]
    at = np.asarray(tl.joint()[0], float)
    trunk = [(tri + at, colour) for (link, name), part in ts.posed().items() if link != 'hips' and name != 'tube'
             for tri, colour in part]
    return {'left leg': leg, 'right leg': _mirrored(leg), 'left arm': arm, 'right arm': _mirrored(arm),
            'pectoral': _put(pec, 'torso', 'torso'), 'neck': _put(neck, 'torso', 'torso'),
            'trunk': _put(trunk, 'pelvis', 'pelvis'),
            'trunk guards': _put([(tri + at, colour) for tri, colour in tw.guards()], 'pelvis', 'pelvis'),
            'undrawn': _undrawn(roll)}


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
