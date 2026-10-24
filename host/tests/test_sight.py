"""Her eyes (`machine.sight`): the stereo's depth held to the renderer's own on one look into the
room; a look round, posed standing, into her map, its occupied cells against the room's walls and
things, the chair and the table told where they stand; a way to the chair planned round what
she knows, clear of it all; the page's picture of her view (`coaxial.graphics.inset`); her way
round the lodge's furniture walked through its corners (`machine.route`)."""
import itertools
import math
import sys
import time
from types import SimpleNamespace

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: A look round is LOOKS looks, her heading posed a turn about; the stereo is held to the
#: renderer out to STEREO_M, the room's depth: her map takes less (`sight.FAR_M`).
LOOKS, STEREO_M = 25, 4.0


def _room():
    from machine import physics
    physics.ROOM = physics.SIGHT = 1.0
    from machine import Machine
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    world = next(n.world for n in body.others + list(a.node for a in body.actuators.values())
                 if hasattr(n, 'world'))
    return body, world


def _stood(world, heading=0.0):
    """Her posed standing in the room's middle, headed `heading` rad: nothing walks her."""
    from machine import figure
    world.reset(figure.stand(), where=(0.0, figure.standing(), 0.0),
                turn=(math.cos(heading / 2.0), 0.0, math.sin(heading / 2.0), 0.0))


def _looked_round(world, keep):
    """Her heading posed once about, a look each of LOOKS."""
    from machine import sight
    for k in range(LOOKS):
        _stood(world, math.tau * k / LOOKS)
        keep.look(world, k * sight.LOOK_S)


def _nearest(things, kinds, at):
    near = [(math.dist(c, at), k, c, h, hi) for k, c, h, hi, _l in things if k in kinds]
    return min(near, default=None)


def test_the_stereo_holds_to_the_depth(report):
    """One look from her stand: the depth from the two eyes' disparity against the renderer's
    own, on the pixels kept."""
    from machine import sight, stereo
    body, world = _room()
    try:
        _stood(world)
        eyes = sight.Eyes(world)
        seen = eyes.see(world)
        disp, kept = sight.disparity(seen.left, seen.right)
        c_disp, c_kept = stereo.cpu_disparity(seen.left, seen.right)
        both = kept & c_kept
        truth = eyes.truth(world)
        ok = kept & (truth > 0.3) & (truth < STEREO_M)
        err = np.abs(sight.depth(disp)[ok] - truth[ok]) / truth[ok]
        report.check('a twentieth of the pixels or more are kept', ok.mean() >= 0.05,
                     '%.0f %%' % (100.0 * ok.mean()))
        report.check('their depth is within 3 %% of the renderer\'s at the median, 10 %% at the '
                     'ninth decile', np.median(err) < 0.03 and np.percentile(err, 90) < 0.10,
                     'median %.1f %%, 90th %.1f %%' % (100 * np.median(err), 100 * np.percentile(err, 90)))
        report.check('the card\'s disparity agrees with the CPU\'s within 0.05 px on 99 %% of pixels',
                     (kept == c_kept).mean() > 0.99 and np.max(np.abs(disp[both] - c_disp[both])) < 0.05,
                     '%.2f %% agree, max %.4f px' % (100.0 * (kept == c_kept).mean(),
                                                     float(np.max(np.abs(disp[both] - c_disp[both])))))
    finally:
        body.close()


def test_her_map_is_the_room(report):
    """A look round, posed standing: nine in ten occupied cells within 0.15 m of a wall or a thing
    of the room, the chair and the table told within 0.3 m of where they stand, a way to the
    chair found clear of what she knows; her view for the page hulls what she told."""
    from machine import room, sight
    body, world = _room()
    try:
        keep = sight.Keep(world)
        _looked_round(world, keep)
        occ = keep.map.occupied()
        boxes = [((-room.WIDE, (room.FROM + room.TO) / 2.0), (room.THICK, (room.TO - room.FROM) / 2.0)),
                 ((room.WIDE, (room.FROM + room.TO) / 2.0), (room.THICK, (room.TO - room.FROM) / 2.0)),
                 ((0.0, room.FROM), (room.WIDE, room.THICK)), ((0.0, room.TO), (room.WIDE, room.THICK)),
                 (room.CHAIR_AT, (0.25, 0.25)), (room.BED_AT, (room.BED[0], room.BED[2])),
                 (room.TABLE_AT, (0.55, 0.3)), (room.LAMP_AT, (0.15, 0.15)),
                 ((room.LOFT_X, room.LOFT_FROM / 2.0), (room.LOFT[0], room.LOFT_FROM / 2.0))]

        def away(x, z):
            return min(math.hypot(max(abs(x - c[0]) - h[0], 0.0), max(abs(z - c[1]) - h[1], 0.0))
                       for c, h in boxes)

        far = [away(*keep.map.centre(i, j)) for i, j in zip(*np.nonzero(occ))]
        near = sum(d <= 0.15 for d in far)
        report.check('looked round she has a map', len(far) >= 100,
                     '%d cells from %d looks' % (len(far), keep.map.looks))
        report.check('nine in ten occupied cells lie within 0.15 m of a wall or a thing',
                     near >= 0.9 * len(far), '%d of %d' % (near, len(far)))
        things = keep.map.things()
        for name, kinds, at in (('chair', ('chair',), room.CHAIR_AT),
                                ('table', ('table',), room.TABLE_AT),
                                ('bed', ('bed',), room.BED_AT)):
            found = _nearest(things, kinds, at)
            report.check('the %s is told within 0.3 m of where it stands' % name,
                         found is not None and found[0] < 0.3,
                         '%.2f m off, high %.2f' % (found[0], found[4]) if found else 'not told')
        at = (0.0, 0.0)
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
        cells = inset.eyes(seen, 120, 40)
        report.check('the picture\'s cells fill its frame', len(cells) >= sight.VIEW[0] * sight.VIEW[1] // 2,
                     '%d cells' % len(cells))
        moved = (-0.9, 1.2)
        world.model.body('bed').pos[:] = (moved[0], room.BED[1], moved[1])
        keep.map, keep.looked_at = sight.Map(), None
        _looked_round(world, keep)
        found = _nearest(keep.map.things(), ('bed',), moved)
        report.check('the bed moved is told within 0.3 m of its new place and size',
                     found is not None and found[0] < 0.3
                     and math.hypot(found[3][0] - room.BED[0], found[3][1] - room.BED[2]) < 0.2,
                     '%.2f m off, half (%.2f %.2f)' % (found[0], found[3][0], found[3][1])
                     if found else 'not told')
    finally:
        body.close()


def _laid(x, z, yaw=0.0):
    """`g` as `route.planned` reads it: her place, her heading, her rhythm at rest."""
    from machine import edge, latent
    sense = np.zeros(edge.SENSED)
    sense[edge.QUAT] = (math.cos(yaw / 2.0), 0.0, math.sin(yaw / 2.0), 0.0)
    return {'world': SimpleNamespace(data=SimpleNamespace(qpos=np.array([x, 0.0, z]))), 'sense': sense,
            'st': np.zeros(latent.STATE)}


def _bent(curve):
    """Her curve's sharpest bend, 1/m: its chords' turn over their length."""
    d = np.diff(curve[:, :2], axis=0)
    turn = np.abs(np.diff(np.unwrap(np.arctan2(d[:, 0], d[:, 1]))))
    return float((turn / np.maximum(0.5 * (np.hypot(*d[1:].T) + np.hypot(*d[:-1].T)), 1e-9)).max(initial=0.0))


def test_her_way_walked_through(report):
    """Her way to the chair's and the bed's stages from house.py's spots on the lodge's cells: every
    corridor holds no blocked cell's centre and the next overlaps it, every curve's samples lie in
    free cells and bend within her arc, her last stop her stage; to a walled-off goal and from
    inside a grown prop her stops are `route.way`'s corners, no curve; a plan within a look of her
    eyes (`sight.LOOK_S`), timed."""
    from machine import furniture, physics, route, sight, worker
    from tools.sim import house
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    g = worker.session()['g']
    cells = route.blocked(g)
    at = route.centres(cells)
    held, inside, overlap, off, bent, last, walked, kept, took = [], 0, True, 0, 0.0, True, 0, 0, []
    for verb in ('sit_chair', 'lie_bed'):
        dx, dz, hdd, _lat, most, where = route.docks()[verb]
        _d, px, pz, _top = furniture.nearest(g, verb, None, where)
        out = (math.sin(hdd), math.cos(hdd))
        d = route.staged((px + dx, pz + dz), out, most, cells)
        to = (px + dx + d * out[0], pz + dz + d * out[1])
        for x, z in house.spots():
            g_at = _laid(x, z)
            corners = route.way(g_at, to, cells)
            walls = [route.corridor(at, a, b) for a, b in route.carried(cells, [(x, z)] + corners)]
            inside += sum(int(((at @ A.T) < b).all(axis=1).sum()) for A, b in walls)
            overlap = overlap and all(_meets(w0, w1) for w0, w1 in zip(walls, walls[1:]))
            t0 = time.perf_counter()
            stops, curve, held = route.planned(g_at, to, cells, held)
            took.append(time.perf_counter() - t0)
            off += sum(int(cells[route._cell(px_, pz_)]) for px_, pz_ in curve[:, :2])
            bent = max(bent, _bent(curve) if len(curve) > 2 else 0.0)
            last = last and stops[-1] == to and stops == corners[len(corners) - len(stops):]
            walked += len(corners) - len(stops)
            kept += len(stops) - 1
    report.check('no corridor holds a blocked cell\'s centre, each meets the next', inside == 0 and overlap,
                 '%d centres inside' % inside)
    report.check('the curves\' samples all in free cells, bent within her arc', off == 0 and bent <= 1.0 / route.ARC_M,
                 '%d samples blocked, %.3f/m of %.3f' % (off, bent, 1.0 / route.ARC_M))
    report.check('her stops `way`\'s corners she does not walk through, her stage last', last,
                 '%d corners walked through, %d kept' % (walked, kept))
    report.check('a plan within a look (%.1f s)' % sight.LOOK_S, max(took) < sight.LOOK_S,
                 'median %.1f ms, most %.1f ms over %d plans' % (1e3 * float(np.median(took)), 1e3 * max(took), len(took)))
    walled = cells.copy()
    goal = route._cell(-0.2, 1.4)
    walled[goal[0] - 3:goal[0] + 4, goal[1] - 3:goal[1] + 4] = True
    walled[goal[0] - 2:goal[0] + 3, goal[1] - 2:goal[1] + 3] = False
    edge_ = [(-1.9, -1.0), (1.5, 0.8)]
    same = [(route.planned(_laid(*p), to_, c)[:2], route.way(_laid(*p), to_, c)) for p, to_, c in (
        (edge_[0], route._centre(*goal), walled), (edge_[1], (-0.35, 1.8), cells))]
    report.check('to a walled-off goal and from inside the chair\'s grown cells, `way`\'s corners, no curve',
                 all(stops == way and not len(curve) for (stops, curve), way in same),
                 [(stops, len(curve), way) for (stops, curve), way in same])


def _meets(w0, w1):
    """Whether two corridors share a point: where two of their rows cross, inside all of them."""
    A, b = np.vstack([w0[0], w1[0]]), np.r_[w0[1], w1[1]]
    pairs = [list(m) for m in itertools.combinations(range(len(b)), 2) if abs(np.linalg.det(A[list(m)])) > 1e-9]
    return any((A @ np.linalg.solve(A[m], b[m]) <= b + 1e-9).all() for m in pairs)


def test_her_things_fill_her_registry(report):
    """Her eyes' things into her registry (`sight.told`, `machine.registry`): looked round, each look's
    things seen from the head that looked, the chair, the table and the bed hold slots within 0.3 m of
    where they stand; on her loop under `physics.SIGHT` her things are her map's (`sprung.LOOKS`)."""
    from machine import physics, registry, room, sight, sprung
    body, world = _room()
    try:
        keep, slots = sight.Keep(world), registry.laid()
        hb = world.model.body('head').id
        for k in range(LOOKS):
            _stood(world, math.tau * k / LOOKS)
            keep.look(world, k * sight.LOOK_S)
            head = {'pos': np.array(world.data.xpos[hb]), 'quat': np.array(world.data.xquat[hb])}
            registry.seen(slots, sight.told(keep), head, k * sight.LOOK_S)
        boxes = registry.boxes(slots, least=-math.inf)
        kinds = sorted({b[0] for b in boxes})
        for name, at in (('chair', room.CHAIR_AT), ('table', room.TABLE_AT), ('bed', room.BED_AT)):
            near = [math.hypot(b[1][0] - at[0], b[1][2] - at[1]) for b in boxes if b[0] == name]
            report.check('the %s holds a slot within 0.3 m of where it stands' % name, bool(near) and min(near) < 0.3,
                         '%s of %d slots, %s' % (', '.join('%.2f' % d for d in near) or 'none', len(boxes), kinds))
        g = sprung.laid(sprung.STACK_EVERY)
        while g['t'] < 2.0 * sight.LOOK_S + 0.1:
            sprung.passed(g)
        report.check('on her loop under SIGHT her things are her map\'s: a keep looked twice, things told',
                     isinstance(g['keep'], sight.Keep) and g['keep'].map.looks >= 2 and len(g['things']) >= 1,
                     '%s, %d looks, %d things: %s' % (type(g['keep']).__name__, getattr(g['keep'], 'map', keep.map).looks,
                                                    len(g['things']), sorted({b[0] for b in g['things']})))
    finally:
        physics.SIGHT = 0.0
        body.close()


ROSTER = [test_her_map_is_the_room, test_the_stereo_holds_to_the_depth, test_her_way_walked_through,
          test_her_things_fill_her_registry]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
