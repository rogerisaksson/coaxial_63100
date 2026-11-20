"""The robot's arm worn: guards on the elbow's mount, its tee's sleeve laid over, what cuts through.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/arm_worn.py OUT.png

The elbow's mount as elbow_mount draws it, the upper arm's tube on up to its foot under the shoulder's mount
(`shoulder_mount`, the arm hanging) and the forearm's down to the wrist, in the upper arm's frame at the elbow's
pivot as the page lays the robot (`gynoid.body`): the elbow's mount's x the page's, its stack's middle and way
(`elbow_mount._stack`) undone, and its bend the page's (`placing`); the shoulder's sketch turned about the arm's
line onto the page's. The robot's guards lie where its page lays them - a shoulder's cap and one down each arm
(`leg_worn.worn`), each strut, or a hoop's leg where the build clips a guard on one, stood on what it meets first,
the upper arm's clear of what turns with the forearm through the elbow's travel (`_swept`): an elbow's cap over its
drive's gable stays off the forearm bent its most; over them the likeness's tee's sleeve, laid as leg_worn
lays the jeans (`leg_worn.draped`). What still cuts through what is counted as leg_worn counts it.
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

#: The likeness's part the tee's sleeve is.
SLEEVE = 'like_left_upper_arm'


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
    """{name: tris} of the arm, the elbow's frame: the elbow's mount and its hose, the upper arm's tube up to its
    foot under the shoulder's mount, that mount ('shoulder ...', the arm hanging), the forearm's tube down to the
    wrist; the forearm bent `flex` deg the way the page bends it."""
    import numpy as np
    from tools.render import shoulder_laid as sl, shoulder_mount as sm
    turn, shift, bend = placing()
    _page, shoulder, wrist = laid(False, ())

    def placed(tris):                                    # into the page's frame, faces kept outward
        return [(((tri + shift) @ turn)[::int(np.linalg.det(turn))], c) for tri, c in tris]
    parts, cables, _still, _turned = em.bodies(flex=flex)
    still_parts, _gear, arm_in, arm_out = em._stack()
    over = max(r for *_x, r in still_parts) + 0.002
    face_t, face_s = over + em.SADDLE_M + em.SEAT_M, -(over + em.ROTOR_AIR + em.ROTOR_M + em.SEAT_M)
    top, at = float(shoulder[1]) - sl.laid()['seat'][2] - em.FLANGE_M, (float(shift[0] * turn[0, 0]), 0.0)
    out = {'elbow': placed(parts), 'hose': placed(cables),
           'upper arm tube': m._prism(m._circle(arm_in, 40, at), face_t + em.FLANGE_M, top, em.CARBON),
           'forearm tube': km._turn(m._prism(m._circle(arm_out, 40, at), float(wrist[1]) + 0.01,
                                             face_s - em.FLANGE_M, em.CARBON), bend * flex)}
    out.update({'shoulder ' + name: [(tri * (-1.0, 1.0, -1.0) + shoulder, c) for tri, c in tris]
                for name, tris in sm.posed(0.0).items()})     # half a turn about the arm's line: its x toward the torso
    return out


def _swept(step=40.0):
    """[(tri, colour)]: what turns with the forearm - the elbow's output side and the forearm's tube - through the
    elbow's travel `step` deg at a time (`drives.TRAVEL`), the elbow's frame."""
    import numpy as np
    from machine.body import drives
    turn, shift, bend = placing()
    tube, (low, high) = arm()['forearm tube'], drives.TRAVEL['elbow']
    out = []
    for flex in np.append(np.arange(low, high, step), high):
        parts, _cables, still, _turned = em.bodies(flex=float(flex))
        out += [(((tri + shift) @ turn)[::int(np.linalg.det(turn))], c) for tri, c in parts[len(still):]]
        out += km._turn(tube, bend * float(flex))
    return out


def _guarded():
    """{segment: tris}: the arm's guards, laid clear of the whole arm - the shoulder's mount with it -, the upper
    arm's of what turns with the forearm through the elbow's travel too (`_swept`)."""
    import numpy as np
    return lw.worn(arm(), {'left_upper_arm': laid(False, ())[1], 'left_forearm': np.zeros(3)},
                   {'left_upper_arm': 'upper arm tube', 'left_forearm': 'forearm tube'}, {'left_upper_arm': _swept()})


def capped():
    """How many triangle pairs of the upper arm's guards and the shoulder's mount cross, the arm hanging."""
    from tools.render import clips
    return clips.crossed(_guarded()['left_upper_arm'], [t for k, v in arm().items() if k.startswith('shoulder ')
                                                        for t in v])


def dressed(flex=0.0):
    """({name: tris} the arm and its guards ('guards ...'), the sleeve laid over them, the farthest it moved, m), the
    forearm bent `flex` deg the way the page bends it. The shoulder's mount is left out: the guards stand clear of it
    (`capped`), and it lies under the tee's body, which no sketch here wears."""
    sleeve, _shoulder, _wrist = laid(True, (SLEEVE,))
    put = _guarded()
    bare = dict({k: v for k, v in arm(flex).items() if not k.startswith('shoulder ')},
                **{'guards upper arm': put['left_upper_arm'],
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
    mount = [t for k, v in arm().items() if k.startswith('shoulder ') for t in v]
    views = (('from ahead', 35.0, 12.0), ('from the side', 90.0, 6.0), ('from behind', -145.0, 12.0))
    rows = [[('guards on, ' + name, every + mount, yaw, pitch) for name, yaw, pitch in views]
            + [('the elbow bent 90 deg', bent_all + mount, 70.0, 12.0)],
            [('in its tee, ' + name, every + sleeve, yaw, pitch) for name, yaw, pitch in views]
            + [('the elbow bent 90 deg', bent_all + bent_sleeve, 70.0, 12.0)]]
    lw.sheet(args.out, "arm worn: the robot's guards on the elbow's mount and tubes, the likeness's tee laid over them -"
             " out %.0f mm at the most; cutting through: %s, bent %s" % (1e3 * push, told, bent_told), rows)
    print(args.out, 'sleeve out %.1f / %.1f mm; crossing %s, bent %s; guards x shoulder %d'
          % (1e3 * push, 1e3 * bent_push, told, bent_told, capped()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
