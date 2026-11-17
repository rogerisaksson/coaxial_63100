"""The robot's seat bare of its clothes: a black honeycomb net a side over its hip's roll drive, turning with it."""
import sys

from tools.dev.focus import chosen
from views_kit import Report


def _caps(angles):
    """[(r, a, b)] the pelvis's capsules and the left thigh's at `angles`, the pelvis's frame."""
    import numpy as np
    from machine.body import figure
    at, turn = (np.asarray(v, float) for v in figure.frames(angles, (0.0, 0.0, 0.0), figure.IDENT)['left_thigh'])
    return [(r, np.asarray(a, float), np.asarray(b, float)) if seg == 'pelvis' else
            (r, at + turn @ np.asarray(a, float), at + turn @ np.asarray(b, float))
            for seg, r, a, b in figure.BODY if seg in ('pelvis', 'thigh')]


def _borne(points, caps):
    """How far `points` stand beyond the planes the capsules `caps` bear on, each way from straight down to straight
    back (sat on a floor, leant back, lying on the back), m: under 0 within them."""
    import numpy as np
    turns = np.radians(np.linspace(0.0, 90.0, 19))
    ways = np.stack([np.zeros_like(turns), -np.cos(turns), -np.sin(turns)], 1)
    held = np.max([np.maximum(ways @ a, ways @ b) + r for r, a, b in caps], axis=0)
    return float((points @ ways.T - held).max())


def _nearest(points, others):
    """The least distance from `points` to `others`, m (inf with none)."""
    import numpy as np
    return min([float(np.linalg.norm(c[:, None, :] - points[None, :, :], axis=2).min())
                for c in np.array_split(others, max(1, len(others) // 2000)) if len(c)] or [np.inf])


def test_the_seat_nets_turn_with_the_drives(report):
    """The robot's seat a net a side (`seat.plate`): the guards' honeycomb alone, its bars black, over the hip's roll
    drive's stack a guard's air off it, the two apart at the middle and each the other's mirror; fixed on the drive -
    the hip's yaw turns it with the drive, its roll and its pitch leave it still -; a hip's yaw to its stops, its roll
    30 deg either way and its pitch from 30 back to 120 forward keep it within what the robot bears on (`figure.BODY`'s
    pelvis and thigh, what the floor, a chair and a bed meet) and the mechanism by it a guard's air off it."""
    import itertools
    import numpy as np
    from coaxial.graphics import gynoid, livery, seat
    from machine.body import figure, guards
    body = gynoid.body(dressed=False, plates=True)
    names = [p[0] for p in body.parts]
    k, kr, kp = (names.index(n) for n in ('left_seat', 'right_seat', 'pelvis'))
    left, right = body.parts[k][5], body.parts[kr][5]
    stand = figure.stand()

    def laid(angles):
        pos, _n = body.pose(angles)
        return pos - (pos[slice(*body.spans[kp])] - body.parts[kp][5][0]).mean(axis=0)   # the pelvis moved, not turned
    own = [i for i, n in enumerate(names) if body.parts[i][1] == 'left_hip_yaw' and n.startswith(seat.STACK)]
    pos = laid(stand)
    net, stack = pos[slice(*body.spans[k])], np.vstack([pos[slice(*body.spans[i])] for i in own])
    off = _nearest(net, stack)
    report.check("a net a side over its hip's roll drive a guard's air off it, its bars black, apart at the middle, the "
                 "right the left's mirror",
                 off >= guards.AIR_M and set(left[3].tolist()) == {livery.VOID} and len(left[0]) > 0
                 and float(left[0][:, 0].min()) > 0.01 and bool(np.allclose(right[0], left[0] * (-1.0, 1.0, 1.0))),
                 '%d corners, the drive %.4f m off, %.3f m off the middle' % (len(left[0]), off, left[0][:, 0].min()))
    gap0 = np.linalg.norm(net[:, None, :] - stack[None, ::7, :], axis=2)
    turned, still = (laid(dict(stand, **a)) for a in ({'left_hip_yaw': 25.0}, {'left_hip_roll': 30.0, 'left_hip': -90.0}))
    rigid = float(np.abs(np.linalg.norm(turned[slice(*body.spans[k])][:, None, :] - np.vstack(
        [turned[slice(*body.spans[i])] for i in own])[None, ::7, :], axis=2) - gap0).max())
    moved = float(np.abs(turned[slice(*body.spans[k])] - net).max())
    held = float(np.abs(still[slice(*body.spans[k])] - net).max())
    report.check("the hip's yaw turns it with the drive, its roll and its pitch leave it still",
                 rigid < 1e-9 and moved > 0.01 and held < 1e-9,
                 'with the yaw %.3f m moved, %.1e m off the drive\'s own; with the roll and the pitch %.1e m' % (moved, rigid, held))
    others = [i for i, n in enumerate(names) if not n.endswith('_seat') and body.spans[i][1] > body.spans[i][0]]
    out, gap = -np.inf, np.inf
    for yaw, roll, pitch in itertools.product((-25.0, 25.0), (-30.0, 30.0), (30.0, -60.0, -120.0)):
        angles = dict(stand, left_hip_yaw=yaw, left_hip_roll=roll, left_hip=pitch)
        pos = laid(angles)
        plate = pos[slice(*body.spans[k])]
        rest = np.vstack([pos[slice(*body.spans[i])] for i in others])
        lo, hi = plate.min(axis=0) - guards.AIR_M, plate.max(axis=0) + guards.AIR_M
        out = max(out, _borne(plate, _caps(angles)))
        gap = min(gap, _nearest(plate, rest[((rest > lo) & (rest < hi)).all(axis=1)]))
    report.check("the hip turned to its stops and its roll and pitch's ends, the net within what the robot bears on "
                 "and the mechanism by it a guard's air off it",
                 out < 0.0 and gap >= guards.AIR_M, '%.4f m within at the least, the mechanism %.4f m off' % (-out, gap))


ROSTER = (test_the_seat_nets_turn_with_the_drives,)


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
