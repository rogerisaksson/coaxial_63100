#!/usr/bin/env python3
"""What the robot's things afford it, told anew in C as a look moves them (`tools.cores.afford`, wbc_afford.c),
beside its numpy twins (`furniture.near_twin`, `affordance._covers`): looks drawn with rows crept, turned, renamed or
left, each row's nearness bit for bit; boxes' tops drawn under things of every height and turn, each top's clear area
bit for bit, its list's too; the robot's lodge's own things, every box's top on either slab and each thing crept in
turn; a look's time beside numpy's."""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The draws and their seed; the kinds they are of.
DRAWS, SEED = 1500, 23
KINDS = ('chair', 'table', 'bed', 'box', 'wall', 'door')


def _turn(rng):
    """A turn about the floor's normal, or a whole rotation drawn for a fifth of the things."""
    if rng.random() < 0.2:
        q, _r = np.linalg.qr(rng.standard_normal((3, 3)))
        return q * np.sign(np.linalg.det(q))
    a = rng.uniform(-math.pi, math.pi)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def _boxes(rng, n):
    """`affordance.Boxes` of n things drawn about a lodge's floor, their arrays beside them."""
    from machine.way import affordance
    centre = rng.uniform(-3, 3, (n, 3)) * (1.0, 0.25, 1.0) + (0.0, 0.5, 0.0)
    half, R = rng.uniform(0.02, 1.2, (n, 3)), np.array([_turn(rng) for _ in range(n)])
    boxes = affordance.Boxes([(KINDS[k % len(KINDS)], centre[k], half[k], R[k], k, float(rng.integers(2)))
                              for k in range(n)])
    boxes.arrays = (centre, half, R)
    return boxes


def _rows(arrays):
    n = len(arrays[0])
    return [a.reshape(n, -1) for a in arrays]


def test_a_looks_rows_told_anew_in_c_as_in_numpy(report, _c):
    """Rows of a look drawn and the next look's crept by a hair, moved a cell, renamed or left: whether each is told
    anew, C's and the twin's alike."""
    from machine.eyes.voxels import CELL_M
    from machine.way import affordance, furniture
    from tools.cores import afford
    rng = np.random.default_rng(SEED)
    apart, rows, moved_draws = 0, 0, 0
    for draw in range(DRAWS):
        n = int(rng.integers(1, 90))
        was = _rows(_boxes(rng, n).arrays)
        now = [a.copy() for a in was]
        for k in rng.choice(n, int(rng.integers(0, min(n, 4) + 1)), replace=False):
            step = (1e-12, 1e-4, 0.3)[draw % 3]
            now[0][k] += rng.normal(0.0, step, 3)
            if draw % 7 == 0:
                now[2][k] = _turn(rng).ravel()
        told = rng.random(n) < (0.05 if draw % 5 == 0 else 0.0)
        got, twin = (afford.near(now, was, told, CELL_M, affordance.ROOT2),
                     furniture.near_twin(now, was, told, CELL_M, affordance.ROOT2))
        apart += got.shape != twin.shape or bool((got != twin).any())
        rows, moved_draws = rows + n, moved_draws + bool(twin.any())
    report.check('%d looks of %d rows, %d of them moving: each row told anew in C as in numpy' % (DRAWS, rows, moved_draws),
                 not apart and moved_draws > DRAWS // 2, '%d looks apart' % apart)


def _cleared(box, things, slab):
    """(C's, the twin's) clear of `box`'s top under `things` (`Boxes`) on `slab`, as `affordance.clear` lays them."""
    from machine.way import affordance
    from tools.cores import afford
    fb = affordance.footprint(box)
    h = fb[2]
    lo, hi, reach = affordance.top(box) + slab[0], affordance.top(box) + slab[1], math.hypot(h[0], h[1])
    other = np.array([t is not box for t in things], bool)
    return (afford.clear(things.arrays, other, fb, lo, hi, reach, affordance.ROOT2),
            float(4.0 * h[0] * h[1]) - sum(affordance._covers(box, things, fb, (lo, hi), reach).tolist()))


def test_a_tops_clear_area_in_c_as_in_numpy(report, _c):
    """Boxes' tops drawn under things of every height and turn, on the seat's and the bed's slabs and on slabs drawn:
    each top's clear area C's and numpy's to the bit, and the list's (`affordance.covered` a thing at a time)."""
    from machine.way import affordance
    rng = np.random.default_rng(SEED + 1)
    apart, listed, tops, covered = 0, 0, 0, 0
    for draw in range(DRAWS):
        things = _boxes(rng, int(rng.integers(1, 70)))
        box = things[int(rng.integers(len(things)))] if draw % 4 else _boxes(rng, 1)[0]
        lo = float(rng.uniform(-0.6, 0.2))
        for slab in (affordance.SEAT_CLEAR, affordance.BED_CLEAR, (lo, lo + float(rng.uniform(0.0, 1.5)))):
            got, twin = _cleared(box, things, slab)
            apart += np.float64(got).tobytes() != np.float64(twin).tobytes()
            listed += np.float64(got).tobytes() != np.float64(affordance.clear(box, list(things), slab)).tobytes()
            tops, covered = tops + 1, covered + (twin < 4.0 * float(box[2][0]) * float(box[2][2]))
    report.check('%d tops, %d of them covered: each clear area C\'s as numpy\'s, and as the list\'s' % (tops, covered),
                 not apart and not listed and covered > tops // 8, '%d apart, %d from the list' % (apart, listed))


def test_the_lodges_things_told_in_c_as_in_numpy(report, _c):
    """The robot's lodge's own things as its session lays them: every box's top on either slab clear in C as in
    numpy, each thing crept in turn told anew alike; a look's re-telling in C beside numpy's."""
    from machine.eyes.voxels import CELL_M
    from machine.process import worker
    from machine.world import physics
    from machine.way import affordance, furniture
    from tools.cores import afford
    was = physics.ROOM, physics.SIGHT
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    try:
        s = worker.session()
        g = s['g']
        while g['t'] < 0.1:
            worker.stepped(s)
        boxes = furniture._boxes(g['things'], range(len(g['things'])))
    finally:
        physics.ROOM, physics.SIGHT = was
    apart = sum(np.float64(a).tobytes() != np.float64(b).tobytes()
                for box in boxes for slab in (affordance.SEAT_CLEAR, affordance.BED_CLEAR)
                for a, b in [_cleared(box, boxes, slab)])
    rows = _rows(boxes.arrays)
    told, crept = np.zeros(len(boxes), bool), 0
    for k in range(len(boxes)):
        now = [a.copy() for a in rows]
        now[0][k] += (1e-4, 0.0, -1e-4)
        got, twin = (afford.near(now, rows, told, CELL_M, affordance.ROOT2),
                     furniture.near_twin(now, rows, told, CELL_M, affordance.ROOT2))
        crept += bool((got != twin).any())
    pace = []
    for fn in (afford.near, furniture.near_twin):
        t0 = time.perf_counter()
        for _ in range(200):
            fn(now, rows, told, CELL_M, affordance.ROOT2)
        pace.append(1e6 * (time.perf_counter() - t0) / 200)
    report.check("the lodge's %d things: every top's clear and each crept thing's telling in C as in numpy "
                 '(%.1f us a telling, numpy %.1f)' % (len(boxes), pace[0], pace[1]),
                 len(boxes) > 10 and not apart and not crept, '%d tops apart, %d tellings apart' % (apart, crept))


ROSTER = (test_a_looks_rows_told_anew_in_c_as_in_numpy, test_a_tops_clear_area_in_c_as_in_numpy,
          test_the_lodges_things_told_in_c_as_in_numpy)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    from tools.cores.build import find_cc
    if find_cc() is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, None)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
