#!/usr/bin/env python3
"""What the robot's eyes take a thing for, in C (`tools.cores.classes`, wbc_classes.c) beside its numpy twin
(`classes.of(in_c=False)`): looks drawn to take every path - a floor and its walls, windows, doors and posts making a
house with a veranda and a terrace outside it, stairs up a platform, seats with backs and legs under their tops,
tables, beds, shelves, materials and classes already, every box turned any way -, and the robot's lodge's own
boxes (`world.props()`), every name and every whole bit for bit; a look's time beside numpy's. A house of
boxes told as the robot's eyes should take it (HOUSE), each piece its whole's, what is of one thing one."""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The draws' looks and their seed.
LOOKS, SEED = 400, 29
#: What a drawn thing is taken for: materials told by their boxes, what a look tells alone, classes already.
KINDS = ('wood', 'timber', 'stone', 'glass', 'bed', 'door', 'lamp', 'lit', 'rail', 'roof', 'switch', 'chair', 'table',
         'sofa', 'stool', 'wall', 'window', 'box')


def _turn(rng, upright):
    """A box's turn: upright, or any way."""
    if upright:
        yaw = rng.uniform(-math.pi, math.pi)
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    q, _r = np.linalg.qr(rng.standard_normal((3, 3)))
    return q * np.sign(np.linalg.det(q))


def _box(kind, x, y, z, hx, hy, hz, R=None, clear=1.0):
    return (kind, np.array([x, y, z]), np.array([hx, hy, hz]), np.eye(3) if R is None else R, 0, clear)


def _house(rng, things, x0, z0):
    """A house at (x0, z0): its floor, four walls with a door and panes, a roof over them, a floor by it (a veranda)
    and a platform sunk by it, stairs up to a platform at its side."""
    from machine.eyes import classes
    from machine.eyes.voxels import CELL_M
    w, d, h = rng.uniform(2.0, 4.0), rng.uniform(2.0, 4.0), rng.uniform(1.0, 1.4)
    things.append(_box('stone', x0, 0.0, z0, 0.5 * w, 0.02, 0.5 * d))
    for sx, sz, hx, hz in ((1, 0, 0.04, 0.5 * d), (-1, 0, 0.04, 0.5 * d), (0, 1, 0.5 * w, 0.04), (0, -1, 0.5 * w, 0.04)):
        things.append(_box('timber', x0 + sx * 0.5 * w, h, z0 + sz * 0.5 * d, hx, h, hz))
    things.append(_box('door', x0 + 0.5 * w, 0.95, z0 + 0.3 * d, 0.05, 0.95, 0.4))
    things.append(_box('glass', x0 - 0.5 * w, 1.4, z0, 0.03, 0.4, 0.5, clear=0.0))
    if rng.random() < 0.5:
        things.append(_box('roof', x0, 2.0 * h + 0.1, z0, 0.5 * w + 0.2, 0.08, 0.5 * d + 0.2))
    things.append(_box('stone', x0 + 0.5 * w + 0.8, 0.0, z0 - 0.2 * d, 0.7, 0.02, 0.5))
    things.append(_box('stone', x0 - 0.5 * w - 0.6, -0.3, z0, 0.5, 0.05, 0.5))
    for k in range(3):
        things.append(_box('stone', x0, 0.5 * (k + 1) * classes.STEP, z0 + 0.5 * d + 0.3 + k * 0.3, 0.6,
                           0.25 * (k + 1) * classes.STEP, 0.15 - 0.5 * CELL_M))
    things.append(_box('stone', x0, 0.45 * classes.STEP, z0 + 0.5 * d + 1.8, 0.8, 0.45 * classes.STEP, 0.6))


def _seat(rng, things, x, z, kind):
    """A seat of `kind` at (x, z): its top, four legs, a back where it has one, as a look sees them in pieces."""
    from machine.eyes import classes
    top, hx, hz = rng.uniform(*classes.SEAT), rng.uniform(0.22, 0.5), rng.uniform(0.22, 0.5)
    if kind in ('sofa', 'bench'):
        hx = 2.2 * max(hz, 0.5 * classes.STANCE)
    things.append(_box(kind, x, top - 0.03, z, hx, 0.03, hz))
    for sx, sz in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        things.append(_box(kind, x + sx * (hx - 0.03), 0.5 * (top - 0.06), z + sz * (hz - 0.03), 0.03,
                           0.5 * (top - 0.06), 0.03, clear=0.0))
    if kind in ('chair', 'sofa'):
        things.append(_box(kind, x, top + 0.25, z - hz + 0.03, hx, 0.25, 0.03, clear=0.0))


def _drawn(rng):
    """A look drawn: a house or two, seats, a table, a bed, shelves, posts and a sprinkle of boxes of every kind,
    sized and turned any way."""
    from machine.eyes import classes
    things = []
    for k in range(int(rng.integers(1, 3))):
        _house(rng, things, rng.uniform(-6, 6), rng.uniform(-6, 6) + 8.0 * k)
    for kind in rng.choice(('wood', 'chair', 'sofa', 'bench', 'stool'), int(rng.integers(1, 4))):
        _seat(rng, things, rng.uniform(-8, 8), rng.uniform(-8, 8), str(kind))
    things.append(_box('wood', rng.uniform(-8, 8), rng.uniform(classes.LAP, classes.BRACE) - 0.02, rng.uniform(-8, 8),
                       rng.uniform(0.3, 0.9), 0.02, rng.uniform(0.3, 0.9)))
    things.append(_box('bed', rng.uniform(-8, 8), rng.uniform(*classes.BED) - 0.15, rng.uniform(-8, 8), 1.0, 0.15, 0.5))
    things.append(_box('wood', rng.uniform(-8, 8), classes.BRACE + 0.3, rng.uniform(-8, 8), 0.4, 0.02, 0.15))
    for _ in range(int(rng.integers(5, 25))):
        kind = KINDS[int(rng.integers(len(KINDS)))]
        half = rng.uniform(0.02, 1.2, 3) * (1.0, 0.6, 1.0)
        things.append(_box(kind, rng.uniform(-9, 9), rng.uniform(-0.2, 2.8), rng.uniform(-9, 9), *half,
                           R=_turn(rng, rng.random() < 0.7), clear=float(rng.integers(2))))
    for k, t in enumerate(things):
        if rng.random() < 0.3:
            things[k] = (t[0], t[1], t[2], _turn(rng, True) @ t[3]) + t[4:]
    rng.shuffle(things)
    return things


def _both(things):
    """((names, whole) in C, (names, whole) in numpy, the C's seconds, numpy's) of `things`."""
    from machine.eyes import classes
    kinds, centre, half, R, clear = classes.stacked(things)
    t0 = time.perf_counter()
    got = classes.of(kinds, centre, half, R, clear)
    t1 = time.perf_counter()
    twin = classes.of(kinds, centre, half, R, clear, in_c=False)
    return got, twin, t1 - t0, time.perf_counter() - t1


def _apart(got, twin):
    """The things the C and numpy name or group apart: [(index, C's, numpy's)]."""
    names, whole = got
    names_, whole_ = twin
    return [(k, (a, int(w)), (b, int(v))) for k, (a, w, b, v) in enumerate(zip(names, whole, names_, whole_))
            if a != b or int(w) != int(v)]


def test_drawn_looks_classed_in_c_as_in_numpy(report):
    """Drawn looks through both: every name and every whole bit for bit, every class of the C's enum told at least
    once across the draws."""
    from machine.eyes import classes
    rng = np.random.default_rng(SEED)
    worst, told, sizes = [], set(), []
    for k in range(LOOKS):
        things = _drawn(rng)
        got, twin, _c, _n = _both(things)
        told |= set(got[0])
        sizes.append(len(things))
        apart = _apart(got, twin)
        if apart:
            worst.append((k, len(things), apart[:3]))
    report.check('%d drawn looks of %d-%d things classed in C as in numpy, every name and whole bit for bit'
                 % (LOOKS, min(sizes), max(sizes)), not worst, '%d apart: %s' % (len(worst), worst[:2]))
    missing = sorted(set(classes.CLASSES) - told)
    report.check('every class of the enum told across the draws (%d)' % len(classes.CLASSES), not missing,
                 ', '.join(missing) or 'all told')


def test_the_lodges_boxes_classed_in_c_as_in_numpy(report):
    """The robot's lodge's own boxes (`world.props()`) through both, as they stand and each thing moved a little
    many times over - a bed creeping under the robot renews its look every 9 ms -, every name and whole bit for bit;
    a look of the lodge in C faster than numpy's."""
    from machine.eyes import classes
    from test_sight import _room
    body, world = _room()
    try:
        world._mj.mj_forward(world.model, world.data)
        props = world.props()
    finally:
        body.close()
    rng, worst, spent = np.random.default_rng(SEED), [], ([], [])
    for k in range(60):
        things = [(t[0], np.asarray(t[1], float) + (rng.normal(0.0, 1e-7, 3) if k else 0.0)) + tuple(t[2:]) for t in props]
        got, twin, c, n = _both(things)
        spent[0].append(c)
        spent[1].append(n)
        apart = _apart(got, twin)
        if apart:
            worst.append((k, apart[:3]))
    report.check("the lodge's %d boxes classed in C as in numpy, as they stand and jittered 59 times over, every name and "
                 "whole bit for bit" % len(props), not worst, '%d apart: %s' % (len(worst), worst[:2]))
    us = [1e6 * float(np.median(each)) for each in spent]
    report.check("a look of the lodge classed in C on this host", us[0] < us[1], '%.0f us a look, numpy %.0f' % tuple(us))


UP = ((1, 0, 0), (0, 1, 0), (0, 0, 1))


def _boxed(kind, centre, half, clear=1.0, slot=0):
    """A box upright: its kind, centre, half sizes, no turn, its slot, whether over it is clear."""
    return (kind, centre, half, UP, slot, clear)


#: A house of boxes, each as the robot's eyes should take it and the box it is a piece of (`classes.of`'s whole, None
#: its own): the faces of what has no top seen clear over it (clear 0), the tops, a chair's back and leg.
HOUSE = (('wall', _boxed('timber', (3.0, 1.2, 0.0), (0.05, 1.2, 2.0), 0.0), None),
         ('window', _boxed('glass', (0.0, 1.2, 3.0), (1.0, 1.2, 0.01), 0.0), None),
         ('window', _boxed('timber', (1.05, 1.2, 3.0), (0.05, 1.2, 0.05), 0.0), None),       # the post framing the pane
         ('door', _boxed('door', (-1.5, 1.0, 3.0), (0.44, 1.0, 0.02), 0.0), None),
         ('wall', _boxed('door', (2.95, 2.2, 1.0), (0.02, 0.15, 0.5), 0.0), None),          # a door's colour high on a wall
         ('ceiling', _boxed('timber', (0.0, 2.6, 0.0), (1.0, 0.05, 1.0), 0.0), None),
         ('railing', _boxed('glass', (0.0, 0.7, -4.0), (1.0, 0.2, 0.01), 0.0), None),
         ('railing', _boxed('rail', (0.0, 1.0, -4.1), (1.0, 0.03, 0.03)), None),
         ('post', _boxed('timber', (-3.0, 1.2, -3.0), (0.05, 1.2, 0.05), 0.0), None),
         ('roof', _boxed('roof', (0.0, 3.5, 0.0), (3.0, 0.08, 3.0)), None),
         ('stairs', _boxed('wood', (6.0, 0.075, 0.0), (0.5, 0.075, 0.15)), None),
         ('stairs', _boxed('wood', (6.0, 0.15, -0.3), (0.5, 0.15, 0.15)), None),
         ('stairs', _boxed('wood', (6.0, 0.225, -0.6), (0.5, 0.225, 0.15)), None),
         ('platform', _boxed('wood', (6.0, 0.3, -1.25), (0.5, 0.3, 0.5)), None),               # the landing they reach
         ('step', _boxed('stone', (9.0, 0.1, 0.0), (0.3, 0.1, 0.15)), None),
         ('chair', _boxed('wood', (12.0, 0.45, 0.0), (0.22, 0.02, 0.22)), None),
         ('chair', _boxed('wood', (12.0, 0.72, 0.22), (0.22, 0.25, 0.02)), 15),                  # its back
         ('chair', _boxed('wood', (12.18, 0.215, -0.18), (0.02, 0.215, 0.02)), 15),              # a leg
         ('sofa', _boxed('wood', (12.0, 0.45, 4.0), (0.9, 0.02, 0.4)), None),
         ('sofa', _boxed('wood', (12.0, 0.72, 4.4), (0.9, 0.25, 0.02)), 18),
         ('table', _boxed('wood', (16.0, 0.74, 0.0), (0.6, 0.015, 0.35)), None),
         ('table', _boxed('wood', (16.55, 0.3625, 0.3), (0.02, 0.3625, 0.02)), 20),
         ('table', _boxed('wood', (16.0, 0.42, 4.0), (0.6, 0.02, 0.3)), None),                   # a low one
         ('bench', _boxed('wood', (20.0, 0.45, 0.0), (0.8, 0.02, 0.2)), None),
         ('stool', _boxed('wood', (20.0, 0.45, 4.0), (0.18, 0.02, 0.18)), None),
         ('cabinet', _boxed('wood', (24.0, 1.0, 0.0), (0.5, 1.0, 0.3)), None),
         ('shelf', _boxed('wood', (24.0, 1.5, 4.0), (0.4, 0.02, 0.15)), None),
         ('thing', _boxed('wood', (28.0, 0.26, 0.0), (0.3, 0.01, 0.3)), None),
         ('bed', _boxed('bed', (28.0, 0.25, 4.0), (0.45, 0.2, 1.0)), None),
         ('lamp', _boxed('lamp', (32.0, 0.75, 0.0), (0.02, 0.75, 0.02)), None),
         ('lamp', _boxed('lit', (32.0, 1.6, 0.0), (0.15, 0.12, 0.15)), None),
         ('switch', _boxed('switch', (34.0, 1.05, 0.0), (0.02, 0.04, 0.03)), None),
         ('ground', _boxed('wood', (40.0, -0.03, 0.0), (1.0, 0.03, 1.0)), None),               # flush, far from the house
         ('rug', _boxed('rug', (44.0, 0.01, 0.0), (0.3, 0.01, 0.6)), None),
         ('wall', _boxed('wood', (2.95, 1.5, -1.0), (0.05, 0.05, 0.05), 0.0), None))           # a wood sliver on the wall


def test_a_house_told_by_its_boxes(report):
    """What the robot's eyes take a house's things for (`classes.of`), each from its box (HOUSE): walls, a pane past
    the robot's brace and the post framing it a window, a door standing on the floor and its colour high on a wall
    none, a ceiling, railings, a free post, a stair's treads and the landing they reach, a lone step, a chair, sofa,
    table high and low, bench, stool, cabinet, shelf, bed, lamp lit or not, switch, the ground far from the house its
    facade and wall make, a scene's rug, a sliver of another colour on a wall the wall's; a chair's back and leg, a
    sofa's back and a table's leg the top's and unnamed (`wholes.named`); the classes told again stay; what is of one
    thing one - a pane and its post, a wall and what lies on it, a stair's treads, a chair's pieces -, a door, a
    landing, a second table and two windows a wall apart apart (`wholes.read`)."""
    from machine.eyes import classes, wholes
    told, boxes = [t for t, _b, _w in HOUSE], [b for _t, b, _w in HOUSE]
    names, whole = classes.of(*classes.stacked(boxes))
    pieces = [i if w is None else w for i, (_t, _b, w) in enumerate(HOUSE)]
    wrong = ['%d %s: %s' % (i, t, n) for i, (t, n) in enumerate(zip(told, names)) if t != n]
    report.check('a house told by its boxes, %d things, %d classes, each piece its whole\'s' % (len(boxes), len(set(told))),
                 not wrong and whole.tolist() == pieces, '; '.join(wrong) or 'wholes %s' % whole.tolist())
    named = wholes.named(boxes)
    unnamed = [i for i, n in enumerate(named) if n is None]
    report.check('a piece of furniture other than its top and what the robot cannot tell unnamed, the rest named',
                 unnamed == [16, 17, 19, 21, 27] and all(n == t for n, t in zip(named, told) if n is not None), str(unnamed))
    again = classes.of(names, *classes.stacked(boxes)[1:])[0]
    report.check('the classes told again stay', again == names, str([(a, b) for a, b in zip(names, again) if a != b]))
    ones = wholes.read(boxes)[1]
    one = lambda *at: len({ones[i] for i in at}) == 1
    joined = ((1, 2), (0, 4, 34), (6, 7), (10, 11, 12), (15, 16, 17), (18, 19), (20, 21), (29, 30))
    apart = wholes.read([_boxed('timber', (0.0, 1.2, 9.0), (0.5, 1.2, 0.05), 0.0)]
                         + [_boxed('glass', (x, 1.2, 9.0), (0.5, 0.6, 0.01), 0.0) for x in (-1.0, 1.0)])[1]
    report.check('what is of one thing one; a door, a landing, a second table, two windows a wall apart apart',
                 all(one(*at) for at in joined) and not any(one(*at) for at in ((1, 3), (12, 13), (20, 22)))
                 and apart[1] != apart[2], '%s; %s' % (ones, apart))


ROSTER = (test_drawn_looks_classed_in_c_as_in_numpy, test_the_lodges_boxes_classed_in_c_as_in_numpy,
          test_a_house_told_by_its_boxes)


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
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
