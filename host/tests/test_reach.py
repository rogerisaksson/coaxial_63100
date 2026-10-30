"""Her leg's reach (`machine.body.reach`): its rows her figure's laid anew; what they hold, her stops
allow; a place landed stands in MuJoCo, either limb the one code; two feet's gap."""
import importlib
import math
import random
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The model's names of the limbs' indices.
SIDES = ('left', 'right')


def test_the_rows_are_her_figure(report):
    """machine/body/reach_rows.py is what tools/cores/reach.py lays from her figure now."""
    from tools.cores import reach as laid
    with open(laid.PATH, encoding='utf-8') as f:
        on_disk = f.read()
    report.check('the rows on disk are her figure laid anew', laid.render(laid.rows()) == on_disk,
                 'python tools/cores/reach.py --write')


def test_what_the_rows_hold_her_stops_allow(report):
    """Hips at random in a foot's frame, a third of them at the leg's length, where `land` draws
    a place in to: none the rows hold is past a stop (`figure.leg` against the compiled figure's
    stops and the rods' strokes). Of what her stops allow the rows leave out a row's caution
    and, at the leg's length, the strip past the ankle's island."""
    from machine.body import reach
    from tools.cores import reach as laid
    limits, rng = laid.stops(), random.Random(7)
    held, lost, past = [0, 0], [0, 0], 0.0
    for n in range(60000):
        r = (rng.uniform(-0.3, 0.3), rng.uniform(0.0, 0.78), rng.uniform(-0.8, 0.8))
        straight = n % 3 == 0
        if straight:
            k = (reach.THIGH + reach.SHANK) * (1.0 - 1e-9) / math.hypot(*r)
            r = (r[0] * k, r[1] * k, r[2] * k)
        exact, inside = laid.held(r, limits), min(reach.margins(r))
        held[straight] += exact
        lost[straight] += exact and inside < 0.0
        if inside >= 0.0 and not exact:
            past = max([past] + [max(limits[k][0] - q, q - limits[k][1])
                                 for k, q in zip(('knee', 'ankle', 'ankle_roll'), laid.bend(r))])
    report.check('no hip the rows hold is past a stop', past < 1e-6, '%.2g rad past at most' % past)
    report.check('of what her stops allow the rows leave out under a twentieth',
                 held[0] > 4000 and lost[0] < 0.05 * held[0],
                 "%d of %d; at the leg's length %d of %d" % (lost[0], held[0], lost[1], held[1]))


def _figure():
    mujoco = importlib.import_module('mujoco')
    from machine.body import mjcf
    model = mujoco.MjModel.from_xml_string(mjcf.mjcf())
    return mujoco, model, mujoco.MjData(model)


def _foot(mujoco, model, data, side, pelvis, heading, angles):
    """(ankle, turn) of a foot in MuJoCo, the pelvis at `pelvis` headed `heading`, the leg's six
    joints at `angles`."""
    from machine.body import figure
    data.qpos[:] = model.qpos0
    data.qpos[0:3] = pelvis
    data.qpos[3:7] = (math.cos(0.5 * heading), 0.0, math.sin(0.5 * heading), 0.0)
    for name, q in zip(figure.LEG, angles):
        data.qpos[model.joint(side + name).qposadr[0]] = q
    mujoco.mj_kinematics(model, data)
    body = data.body(side + '_foot')
    return [float(v) for v in body.xpos], [[float(v) for v in row] for row in body.xmat.reshape(3, 3)]


def test_a_place_landed_stands_in_mujoco(report):
    """Hips and wants at random, either limb: the place `land` answers, its leg solved
    (`figure.leg`) and set in the compiled figure, has the foot there, flat and turned as told,
    every stopped joint within its stop; its margins not negative, its knee the leg's."""
    from machine.body import figure, reach
    from machine.body.figure import ANKLE_H
    from tools.cores import reach as laid
    mujoco, model, data = _figure()
    limits, rng = laid.stops(), random.Random(11)
    off = turned = past = shy = bent = 0.0
    landed = moved = high = 0
    for _ in range(3000):
        limb = rng.randrange(2)
        sign, heading = reach.LIMBS[limb], rng.uniform(-math.pi, math.pi)
        pelvis = (rng.uniform(-1.0, 1.0), rng.uniform(0.35, 0.95), rng.uniform(-1.0, 1.0))
        turn = figure.ry(heading)
        hip = figure.hip(sign, pelvis, turn)
        want = ((hip[0] + rng.uniform(-0.6, 0.6), ANKLE_H, hip[2] + rng.uniform(-0.6, 0.6)),
                heading + rng.uniform(-0.7, 0.7))
        short = rng.choice((0.0, 0.0011))
        (ankle, yaw), m = reach.land(limb, hip, heading, want, short)
        if hip[1] - ANKLE_H > reach.THIGH + reach.SHANK - short:
            high += 1
            shy = max(shy, m)
            continue
        landed, moved = landed + 1, moved + (m < 0.0)
        angles = figure.leg(sign, pelvis, turn, ankle, figure.ry(yaw))
        at, flat = _foot(mujoco, model, data, SIDES[limb], pelvis, heading, angles)
        off = max(off, math.dist(at, ankle))
        turned = max(turned, max(abs(a - b) for ra, rb in zip(flat, figure.ry(yaw)) for a, b in zip(ra, rb)))
        past = max([past] + [max(limits[k][0] - q, q - limits[k][1])
                             for k, q in zip((n[1:] for n in figure.LEG), angles) if k in limits])
        r = reach.over(limb, hip, ankle, figure.ry(yaw))
        shy = max(shy, -min(reach.margins(r, short)))
        bent = max(bent, abs(reach.knee(r) - angles[3]))
    report.check('a place landed has the foot there in MuJoCo, flat and turned as told',
                 landed > 2000 and off < 1e-6 and turned < 1e-6,
                 '%d landed, %d of them drawn in, %d too high: %.2g m off, %.2g turned'
                 % (landed, moved, high, off, turned))
    report.check('every stopped joint within its stop', past < 1e-6, '%.2g rad past at most' % past)
    report.check('its margins not negative, its knee the leg\'s', shy < 1e-6 and bent < 1e-6,
                 '%.2g m short, the knee %.2g rad off' % (shy, bent))


def test_her_right_is_her_left_mirrored(report):
    """The one code, a limb an index: her right's landing is her left's mirrored - the hip, the
    want and the heading across."""
    from machine.body import reach
    rng, apart = random.Random(13), 0.0
    for _ in range(2000):
        hip = (rng.uniform(-1.0, 1.0), rng.uniform(0.3, 0.9), rng.uniform(-1.0, 1.0))
        heading = rng.uniform(-math.pi, math.pi)
        want = ((hip[0] + rng.uniform(-0.6, 0.6), 0.075, hip[2] + rng.uniform(-0.6, 0.6)),
                heading + rng.uniform(-0.7, 0.7))
        ((x, y, z), yaw), m = reach.land(0, hip, heading, want)
        ((mx, my, mz), myaw), mm = reach.land(1, (-hip[0], hip[1], hip[2]), -heading,
                                              ((-want[0][0], want[0][1], want[0][2]), -want[1]))
        apart = max(apart, abs(x + mx), abs(y - my), abs(z - mz), abs(m - mm),
                    abs(math.remainder(yaw + myaw, math.tau)))
    report.check('her right lands as her left mirrored', apart < 1e-9, '%.2g apart at most' % apart)


def _edge(corners, step=0.004):
    out = []
    for (x0, z0), (x1, z1) in zip(corners, corners[1:] + corners[:1]):
        n = max(1, int(math.hypot(x1 - x0, z1 - z0) / step))
        out += [(x0 + (x1 - x0) * k / n, z0 + (z1 - z0) * k / n) for k in range(n)]
    return out


def test_two_feet_clear_of_each_other(report):
    """`clear`: two feet side by side, one before the other, one on the other; at random against
    their edges walked 4 mm apart."""
    from machine.body import reach
    half, heel, front = reach.FOOT[1][0], -reach.FOOT[0][1], reach.FOOT[2][1]
    beside = reach.clear(((0.095, 0.075, 0.0), 0.0), ((-0.095, 0.075, 0.0), 0.0))
    before = reach.clear(((0.0, 0.075, 0.3), 0.0), ((0.0, 0.075, 0.0), 0.0))
    on = reach.clear(((0.0, 0.075, 0.0), 0.0), ((0.0, 0.075, 0.0), 0.0))
    report.check('side by side, one before the other and one on the other',
                 abs(beside - (0.19 - 2.0 * half)) < 1e-12 and abs(before - (0.3 - front - heel)) < 1e-12
                 and abs(on + 2.0 * half) < 1e-12, '%.3f, %.3f and %.3f m' % (beside, before, on))
    rng, worst, crossed = random.Random(17), 0.0, 0
    for _ in range(150):
        one = ((rng.uniform(-0.3, 0.3), 0.075, rng.uniform(-0.3, 0.3)), rng.uniform(-1.0, 1.0))
        other = ((rng.uniform(-0.3, 0.3), 0.075, rng.uniform(-0.3, 0.3)), rng.uniform(-1.0, 1.0))
        gap = reach.clear(one, other)
        walked = min(math.dist(p, q) for p in _edge(reach._corners(one)) for q in _edge(reach._corners(other)))
        if gap > 0.0:
            worst = max(worst, abs(gap - walked))
        else:
            crossed += 1
            worst = max(worst, walked - 0.003)
    report.check('at random its gap is their edges\' nearest, none where they cross', worst < 0.003,
                 '%.4f m off at most, %d of 150 crossing' % (worst, crossed))


ROSTER = [test_the_rows_are_her_figure, test_what_the_rows_hold_her_stops_allow,
          test_a_place_landed_stands_in_mujoco, test_her_right_is_her_left_mirrored,
          test_two_feet_clear_of_each_other]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
