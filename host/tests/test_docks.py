#!/usr/bin/env python3
"""The robot's docks foreseen on its page (`machine.way.peek`): the lodge bed's long side and its foot.

The robot's page draws each thing's faces its wayfinder reaches (`peek.told`) before F is said. Woken in the lodge
and standing as the robot wakes, the bed offers its long side to the room and its foot: on the world's boxes
(`physics.SIGHT`
0) and on the robot's own eyes as its page's worker runs them (`machine.process.worker.run`: its eyes, its vision and its
planners each a process apart), its gaze looking round as it wakes."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The seconds the robot stands as it wakes, its peeks asked anew each round.
WAKE_S = 30.0


def _faces():
    """{name: (a point on the face's line (x, z), its outward normal (x, z), its half length)} of the lodge's bed:
    its long side to the room (+x, the west wall behind its other) and its foot (-z, the gable behind its head)."""
    from machine.world import room
    (x, z), (hx, _hy, hz) = room.BED_AT, room.BED
    return {'long side': ((x + hx, z), (1.0, 0.0), hz), 'foot': ((x, z - hz), (0.0, -1.0), hx)}


def _woke(sight):
    """[(x, z)] the bed's docks the robot's page draws WAKE_S s after it woke in the lodge: on the world's
    boxes, or on its eyes (`sight`) with its vision apart as its worker runs it; the
    robot's planner and its peeks apart either way."""
    from machine.world import physics
    from machine.process import worker
    from machine.way import peek, planner
    from machine.eyes import vision
    physics.ROOM, physics.CAMERA, physics.SIGHT = 1.0, float(sight), float(sight)
    s = worker.session()
    g = s['g']
    g['planner'], s['peeker'] = planner.Planner(), planner.Planner()
    if sight:
        g['vision'] = vision.Apart()
    parts = [g['planner'], s['peeker']] + ([g['vision']] if sight else [])
    try:
        for part in parts:
            part.start()
        while g['t'] < WAKE_S and s['fallen_at'] is None:
            worker.stepped(s)
            peek.peeked(s)
        return [(float(p[0]), float(p[2])) for d in peek.told(s) if d['kind'] == 'lie_bed' for p in d['at']]
    finally:
        for part in parts + [g.get('keep')]:
            if part is not None and hasattr(part, 'close'):
                part.close()
        physics.ROOM, physics.CAMERA, physics.SIGHT = 0.0, 0.0, 0.0


def _on(docks):
    """{face: the docks lying on it}: within a cell (`voxels.CELL_M`) of its line, along it within its length."""
    from machine.eyes.voxels import CELL_M
    out = {}
    for name, ((px, pz), (nx, nz), half) in _faces().items():
        out[name] = [(x, z) for x, z in docks
                     if abs((x - px) * nx + (z - pz) * nz) <= CELL_M and abs((x - px) * nz - (z - pz) * nx) <= half]
    return out


def _check(report, sight, label):
    docks = _woke(sight)
    on = _on(docks)
    report.check("the bed offers its long side to the room and its foot %s, %.0f s awake" % (label, WAKE_S),
                 all(on.values()), 'long side %s, foot %s; all its docks %s' % (
                     on['long side'], on['foot'], [(round(x, 2), round(z, 2)) for x, z in docks]))


def test_the_bed_offers_its_long_side_and_its_foot_on_the_worlds_boxes(report, _c=None):
    """The robot's registry told the world's boxes (`physics.SIGHT` 0): the bed's long side and its foot both
    offered."""
    _check(report, False, "on the world's boxes")


def test_the_bed_offers_its_long_side_and_its_foot_on_the_robots_eyes(report, _c=None):
    """The robot's registry told by its own eyes as its page's worker runs them: the bed's
    long side and its foot both offered, as on the world's boxes."""
    _check(report, True, 'on the robot\'s eyes')


ROSTER = (test_the_bed_offers_its_long_side_and_its_foot_on_the_worlds_boxes,
          test_the_bed_offers_its_long_side_and_its_foot_on_the_robots_eyes)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
