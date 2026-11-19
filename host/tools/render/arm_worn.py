"""The robot's arm worn: guards on the elbow's mount, its tee's sleeve laid over, what cuts through.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/arm_worn.py OUT.png

The elbow's mount as elbow_mount draws it, the upper arm's tube on up under the shoulder's drives and the forearm's
down to the wrist, in the upper arm's frame at the elbow's pivot as the page lays the robot (`gynoid.body`): the
mount's x the page's, its stack's middle and way (`elbow_mount._stack`) undone, and its bend the page's (`placing`).
The robot's guards lie where its page lays them - a shoulder's cap and one down each arm (`leg_worn.worn`), each strut
stood on what it meets first; over them the likeness's tee's sleeve, laid as leg_worn lays the jeans
(`leg_worn.draped`). What still cuts through what is counted as leg_worn counts it.
"""
import argparse
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import elbow_mount as em  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import leg_worn as lw  # noqa: E402
from tools.render import mounting as m  # noqa: E402

#: The likeness's part the tee's sleeve is; the page's drums at the shoulder the upper arm's tube runs up under.
SLEEVE, SHOULDER = 'like_left_upper_arm', ('drive_left_shoulder', 'gear_left_shoulder', 'inv_left_shoulder')


def laid(dressed, names, angles=None):
    """({part: tris}, shoulder, wrist): the page's parts `names` of the robot posed `angles` (else at rest), each in
    the upper arm's frame at the elbow's pivot, and the shoulder's and the wrist's joints there."""
    import numpy as np
    from coaxial.graphics import gynoid, likeness
    body = gynoid.body(dressed=dressed, like=likeness.FOUND, plates=True)
    root, every = ((0.0, 0.0, 0.0), np.eye(3)), [p[0] for p in body.parts]
    frames = body._frames(angles or {}, root)
    pos, _n = body.pose(angles or {}, root=root, _frames=frames)
    turn, elbow = frames[every.index('left_upper_arm')][0], frames[every.index('left_forearm')][1]
    local = (pos - elbow) @ turn
    owner, index = np.repeat(np.arange(len(every)), [len(p[5][1]) for p in body.parts]), body.index.astype(int)
    parts = {n: lw._tris(local, index[owner == every.index(n)], body.materials) for n in names}
    shoulder, wrist = ((frames[every.index(n)][1] - elbow) @ turn for n in ('left_upper_arm', 'left_hand'))
    return parts, shoulder, wrist


def placing():
    """(turn, shift, bend): the sketch's frame to the page's - its x mirrored where its stack runs the other way and
    shifted by its middle, its z mirrored where the page's elbow bends the forearm the other way."""
    import numpy as np
    from machine.body import drives, skeleton
    names = [p for p, *_ in drives.along('left_elbow')]
    xs = [(float(c[2][0]), c[5]) for c in skeleton.cans() if c[0] == 'left_elbow']
    way = 1.0 if xs[names.index('gear')][0] > sum(x for x, _h in xs) / len(xs) else -1.0
    mid = 0.5 * (min(way * x - h for x, h in xs) + max(way * x + h for x, h in xs))
    _p, _s, hand = laid(False, (), {'left_elbow': 90.0})
    swept = km._turn([(np.array([[0.0, -0.2, 0.0]] * 3), (0, 0, 0))], 90.0)[0][0][0, 2]
    bend = 1.0 if (hand[2] > 0.0) == (swept > 0.0) else -1.0
    return np.diag([way, 1.0, bend]), np.array([mid, 0.0, 0.0]), bend


def arm(flex=0.0):
    """{name: tris} of the arm, the elbow's frame: the mount and its hose, the upper arm's tube up under the page's
    shoulder drums (they not drawn: the page's, under its tee's body, which no sketch here wears), the forearm's down
    to the wrist; the forearm bent `flex` deg the way the page bends it."""
    import numpy as np
    turn, shift, bend = placing()
    page, _shoulder, wrist = laid(False, SHOULDER)

    def placed(tris):                                    # into the page's frame, faces kept outward
        return [(((tri + shift) @ turn)[::int(np.linalg.det(turn))], c) for tri, c in tris]
    parts, cables, _still, _turned = em.bodies(flex=flex)
    still_parts, _gear, arm_in, arm_out = em._stack()
    over = max(r for *_x, r in still_parts) + 0.002
    face_t, face_s = over + em.SADDLE_M + em.SEAT_M, -(over + em.ROTOR_AIR + em.ROTOR_M + em.SEAT_M)
    drums = [t for v in page.values() for t in v]
    top, at = min(float(tri[:, 1].min()) for tri, _c in drums) - 0.005, (float(shift[0] * turn[0, 0]), 0.0)
    return {'elbow': placed(parts), 'hose': placed(cables),
            'upper arm tube': m._prism(m._circle(arm_in, 40, at), face_t + em.FLANGE_M, top, em.CARBON),
            'forearm tube': km._turn(m._prism(m._circle(arm_out, 40, at), float(wrist[1]) + 0.01,
                                              face_s - em.FLANGE_M, em.CARBON), bend * flex)}


def dressed(flex=0.0):
    """({name: tris} the arm and its guards ('guards ...'), the sleeve laid over them, the farthest it moved, m), the
    forearm bent `flex` deg the way the page bends it."""
    import numpy as np
    sleeve, shoulder, _wrist = laid(True, (SLEEVE,))
    put = lw.worn(arm(), {'left_upper_arm': shoulder, 'left_forearm': np.zeros(3)},
                  {'left_upper_arm': 'upper arm tube', 'left_forearm': 'forearm tube'})
    bare = dict(arm(flex), **{'guards upper arm': put['left_upper_arm'],
                              'guards forearm': km._turn(put['left_forearm'], placing()[2] * flex)})
    laid_over, moved = lw.draped(sleeve[SLEEVE], *([t for k, v in bare.items() if k.startswith('guards ') == worn
                                                    for t in v] for worn in (False, True)))
    return bare, laid_over, moved


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    args = parser.parse_args(argv)
    bare, sleeve, push = dressed()
    bent, bent_sleeve, bent_push = dressed(90.0)
    told, bent_told = lw.counted(bare, sleeve), lw.counted(bent, bent_sleeve)
    every, bent_all = [t for v in bare.values() for t in v], [t for v in bent.values() for t in v]
    views = (('from ahead', 35.0, 12.0), ('from the side', 90.0, 6.0), ('from behind', -145.0, 12.0))
    rows = [[('guards on, ' + name, every, yaw, pitch) for name, yaw, pitch in views]
            + [('the elbow bent 90 deg', bent_all, 70.0, 12.0)],
            [('in its tee, ' + name, every + sleeve, yaw, pitch) for name, yaw, pitch in views]
            + [('the elbow bent 90 deg', bent_all + bent_sleeve, 70.0, 12.0)]]
    lw.sheet(args.out, "arm worn: the robot's guards on the elbow's mount and tubes, the likeness's tee laid over them -"
             " out %.0f mm at the most; cutting through: %s, bent %s" % (1e3 * push, told, bent_told), rows)
    print(args.out, 'sleeve out %.1f / %.1f mm; crossing %s, bent %s' % (1e3 * push, 1e3 * bent_push, told, bent_told))
    return 0


if __name__ == '__main__':
    sys.exit(main())
