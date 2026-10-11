"""Her eyes (`machine.sight`): the stereo's depth held to the renderer's own on one look into the
room; a look round standing into her map, its occupied cells against the room's walls and
things, the chair and the table told where they stand; a way to the chair planned round what
she knows, clear of it all; the page's picture of her view (`coaxial.graphics.inset`)."""
import math
import sys

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The law takes her at TAKES_AT s, her heading turns from TURNS_AT, round once at the
#: standing rate; a look every LOOK_S.
TAKES_AT, TURNS_AT = 1.5, 3.0


def _room():
    from machine import physics
    physics.ROOM = physics.SIGHT = 1.0
    from machine import Machine
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    world = next(n.world for n in body.others + list(a.node for a in body.actuators.values())
                 if hasattr(n, 'world'))
    return body, world


def _standing(body):
    from machine.director import Director
    director = Director(body, stand_s=1e9)
    director.pace = -1.0
    director.begin(drop=0.004, stage='stand')
    body.loop.step(0.0)
    bus = body.loop.bus
    out = {}
    while bus['t'] < TAKES_AT:
        out = director.step(0.001)
        body.loop.write(**out)
        body.loop.step(0.001)
    return director, out


def _looked_round(body, world, keep, director, out, rate):
    """Her heading turned once about at `rate` on the law, looking; whether she stands."""
    from machine import errands, pace
    bus = body.loop.bus
    pace.take(director, bus, out)
    turned = 0.0
    while turned < math.tau and director.stage != 'fallen':
        g = director.going
        if g is not None and bus['t'] >= TURNS_AT:
            g.heading += rate * 0.001
            turned += rate * 0.001
        out = director.step(0.001)
        body.loop.write(**out)
        body.loop.step(0.001)
        keep.look(world, bus['t'])
    return director.stage != 'fallen'


def _nearest(things, kinds, at):
    near = [(math.dist(c, at), k, c, h, hi) for k, c, h, hi, _l in things if k in kinds]
    return min(near, default=None)


def test_the_stereo_holds_to_the_depth(report):
    """One look from her stand: the depth from the two eyes' disparity against the renderer's
    own, on the pixels kept."""
    from machine import sight
    body, world = _room()
    try:
        _standing(body)
        eyes = sight.Eyes(world)
        seen = eyes.see(world)
        disp, kept = sight.disparity(seen.left, seen.right)
        truth = eyes.truth(world)
        ok = kept & (truth > 0.3) & (truth < sight.FAR_M)
        err = np.abs(sight.depth(disp)[ok] - truth[ok]) / truth[ok]
        report.check('a twentieth of the pixels or more are kept', ok.mean() >= 0.05,
                     '%.0f %%' % (100.0 * ok.mean()))
        report.check('their depth is within 3 %% of the renderer\'s at the median, 10 %% at the '
                     'ninth decile', np.median(err) < 0.03 and np.percentile(err, 90) < 0.10,
                     'median %.1f %%, 90th %.1f %%' % (100 * np.median(err), 100 * np.percentile(err, 90)))
    finally:
        body.close()


def test_her_map_is_the_room(report):
    """A look round standing: nine in ten occupied cells within 0.15 m of a wall or a thing
    of the room, the chair and the table told within 0.3 m of where they stand, a way to the
    chair found clear of what she knows; her view for the page hulls what she told."""
    from machine import errands, room, sight
    body, world = _room()
    try:
        director, out = _standing(body)
        keep = sight.Keep(world)
        up = _looked_round(body, world, keep, director, out, errands.TURN_RAD_S[1])
        occ = keep.map.occupied()
        boxes = [((-room.WIDE, (room.FROM + room.TO) / 2.0), (room.THICK, (room.TO - room.FROM) / 2.0)),
                 ((room.WIDE, (room.FROM + room.TO) / 2.0), (room.THICK, (room.TO - room.FROM) / 2.0)),
                 ((0.0, room.FROM), (room.WIDE, room.THICK)), ((0.0, room.TO), (room.WIDE, room.THICK)),
                 (room.CHAIR_AT, (0.25, 0.25)), (room.BED_AT, (room.BED[0], room.BED[2])),
                 (room.TABLE_AT, (0.55, 0.3)), (room.LAMP_AT, (0.15, 0.15))]

        def away(x, z):
            return min(math.hypot(max(abs(x - c[0]) - h[0], 0.0), max(abs(z - c[1]) - h[1], 0.0))
                       for c, h in boxes)

        far = [away(*keep.map.centre(i, j)) for i, j in zip(*np.nonzero(occ))]
        near = sum(d <= 0.15 for d in far)
        report.check('looked round standing she stands, with a map', up and len(far) >= 100,
                     '%s, %d cells from %d looks' % ('stands' if up else 'fell', len(far), keep.map.looks))
        report.check('nine in ten occupied cells lie within 0.15 m of a wall or a thing',
                     near >= 0.9 * len(far), '%d of %d' % (near, len(far)))
        things = keep.map.things()
        for name, kinds, at in (('chair', ('chair',), room.CHAIR_AT), ('table', ('table',), room.TABLE_AT)):
            found = _nearest(things, kinds, at)
            report.check('the %s is told within 0.3 m of where it stands' % name,
                         found is not None and found[0] < 0.3,
                         '%.2f m off, high %.2f' % (found[0], found[4]) if found else 'not told')
        bus = body.loop.bus
        at = (bus['pelvis.pose.x'], bus['pelvis.pose.z'])
        to = (room.CHAIR_AT[0], room.CHAIR_AT[1] - 0.6)
        way = keep.map.plan(at, to)
        clear = all(away(*p) >= sight.RADIUS_M - sight.CELL_M for p in way)
        report.check('a way to the chair\'s target is found, its corners clear of what she knows by her radius',
                     bool(way) and clear and math.dist(way[-1], to) < 0.5,
                     ' -> '.join('(%+.1f %+.1f)' % w for w in way) or 'none')
        seen = keep.view()
        report.check('her view for the page is a picture with what she told hulled',
                     seen is not None and seen[0].shape == (sight.VIEW[1], sight.VIEW[0], 3)
                     and all(len(h) >= 3 for _k, _i, h in seen[1]),
                     '%d hulls: %s' % (len(seen[1]), ', '.join(k for k, _i, _h in seen[1])) if seen else 'none')
        from coaxial.graphics import inset
        cells = inset.cells(seen, 120)
        report.check('the picture\'s cells fill its frame', len(cells) >= sight.VIEW[0] * sight.VIEW[1] // 2,
                     '%d cells' % len(cells))
    finally:
        body.close()


ROSTER = [test_her_map_is_the_room, test_the_stereo_holds_to_the_depth]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
