"""The robot's seat bare of its clothes: a plate a side over its buttocks, its lower part sliding with the thigh."""
import sys

from tools.dev.focus import chosen
from views_kit import Report

def _beyond(points, a, b, r):
    """How far each of `points` stands out of the capsule `r` round from `a` to `b`, m: under 0 within it."""
    import numpy as np
    a, b = np.asarray(a, float), np.asarray(b, float)
    w = np.clip((points - a) @ (b - a) / max(float((b - a) @ (b - a)), 1e-12), 0.0, 1.0)
    return np.linalg.norm(points - (a + w[:, None] * (b - a)), axis=1) - r


def test_the_seat_plates_slide_with_the_thighs(report):
    """The robot's seat a plate a side (`plated.seat`): within its buttocks (`figure.SEAT_*`, what the floor and a chair
    meet), the two apart at the middle and each the other's mirror, a pad's face in the guards' rim, its back foam, four
    rivets; its lower part slides on the buttock with the thigh - forward with the hip forward, its top still, each
    corner as far from the buttock's middle -; a hip's yaw to its stops, its roll 30 deg either way and its pitch from
    30 back to 120 forward keep the plate within the robot's capsules and the mechanism a guard's air off it."""
    import itertools
    import numpy as np
    from coaxial.graphics import gynoid, livery
    from machine.body import figure, guards
    body = gynoid.body(dressed=False, plates=True)
    names = [p[0] for p in body.parts]
    k, kr, kp = (names.index(n) for n in ('left_seat', 'right_seat', 'pelvis'))
    left, right = body.parts[k][5], body.parts[kr][5]
    middle = np.array([figure.SEAT_X, figure.SEAT_Y, figure.SEAT_Z[0]])
    end = (figure.SEAT_X, figure.SEAT_Y, figure.SEAT_Z[1])
    within = float(_beyond(left[0], middle, end, figure.SEAT_R).max())
    report.check("a plate a side within its buttock, apart at the middle, the right the left's mirror; a pad's face in "
                 "the guards' rim, its back foam, four rivets",
                 within < 0.0 and float(left[0][:, 0].min()) > 0.01 and bool(np.allclose(right[0], left[0] * (-1.0, 1.0, 1.0)))
                 and {livery.PAD, livery.FRAME, livery.FOAM, livery.BOLT} == set(left[3].tolist())
                 and int((left[3] == livery.BOLT).sum()) == 4 * 7,
                 '%.4f m within its buttock, %.3f m off the middle' % (-within, left[0][:, 0].min()))
    stand, others = figure.stand(), [i for i, n in enumerate(names) if not n.endswith('_seat') and body.spans[i][1] > body.spans[i][0]]

    def laid(angles):
        pos, _n = body.pose(angles)
        return pos - (pos[slice(*body.spans[kp])] - body.parts[kp][5][0]).mean(axis=0)   # the pelvis moved, not turned
    still, bent = (laid(a)[slice(*body.spans[k])] for a in (stand, dict(stand, left_hip=-90.0)))
    low, top = left[0][:, 1] < -0.11, left[0][:, 1] > -0.08
    reach = np.abs(np.linalg.norm(bent - middle, axis=1) - np.linalg.norm(still - middle, axis=1)).max()
    report.check('its lower part slides forward on the buttock with the hip forward 90 deg, its top still, each corner as '
                 'far from the buttock\'s middle',
                 float((bent - still)[low, 2].mean()) > 0.02 and float(np.abs(bent - still)[top].max()) < 1e-9 and reach < 1e-9,
                 'its lowest corners %.3f m ahead, its top %.1e m, its reach %.1e m' % (
                     (bent - still)[low, 2].mean(), np.abs(bent - still)[top].max(), reach))
    caps = [c for c in figure.BODY if c[0] in ('pelvis', 'thigh')]
    out, gap = -np.inf, np.inf
    for yaw, roll, pitch in itertools.product((-25.0, 25.0), (-30.0, 30.0), (30.0, -60.0, -120.0)):
        angles = dict(stand, left_hip_yaw=yaw, left_hip_roll=roll, left_hip=pitch)
        pos = laid(angles)
        plate = pos[slice(*body.spans[k])]
        at, turn = (np.asarray(v, float) for v in figure.frames(angles, (0.0, 0.0, 0.0), figure.IDENT)['left_thigh'])
        far = np.min([_beyond(plate, a, b, r) if seg == 'pelvis' else _beyond(plate, at + turn @ np.asarray(a), at + turn @ np.asarray(b), r)
                      for seg, r, a, b in caps], axis=0)
        rest = np.vstack([pos[slice(*body.spans[i])] for i in others])
        rest = rest[(rest[:, 0] > 0.0) & (rest[:, 1] < -0.04) & (rest[:, 2] < -0.03)]
        out = max(out, float(far.max()))
        gap = min(gap, float(np.min(np.linalg.norm(rest[:, None, :] - plate[None, ::2, :], axis=2))))
    report.check("the hip turned to its stops and its roll and pitch's ends, the plate within the robot's capsules and "
                 "the mechanism a guard's air off it",
                 out < 0.0 and gap > guards.AIR_M, '%.4f m within at the least, the mechanism %.4f m off' % (-out, gap))


ROSTER = (test_the_seat_plates_slide_with_the_thighs,)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())