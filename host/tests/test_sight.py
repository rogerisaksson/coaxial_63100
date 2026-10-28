"""Her eyes (`machine.sight`): the stereo's depth held to the renderer's own on one look into the
room; a look round, posed standing, into her map (`machine.voxels`), its occupied cells against the
room's walls and things, the chair, the table and the bed told where they stand by what they afford
her (`machine.affordance`), the bed's turn with it; a way to the chair planned round what she knows,
clear of it all; the page's picture of her view (`coaxial.graphics.inset`); her way round the lodge's
furniture walked through its corners (`machine.route`)."""
import itertools
import math
import sys
import time
from types import SimpleNamespace

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report
from machine import voxels

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


def _head(world):
    """Her head's pose as the world has it: she is posed, her estimate not running."""
    hb = world.model.body('head').id
    return {'pos': np.array(world.data.xpos[hb]), 'quat': np.array(world.data.xquat[hb])}


def _looked_round(world, keep):
    """Her heading posed once about, a look each of LOOKS from her head's pose."""
    from machine import sight
    for k in range(LOOKS):
        _stood(world, math.tau * k / LOOKS)
        keep.look(world, k * sight.LOOK_S, _head(world))


def _nearest(things, affords, at):
    """(its distance from `at`, the thing) of the thing affording `affords` (seat, bed, chair) nearest `at`."""
    from machine import affordance
    by = {'seat': affordance.sits, 'bed': affordance.lies,
          'chair': lambda t, ts: affordance.sits(t, ts) and affordance.backed(t, ts)}[affords]
    near = [(math.hypot(t[1][0] - at[0], t[1][2] - at[1]), t) for t in things if by(t, things)]
    return min(near, default=None, key=lambda n: n[0])


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
    of the room, a way to the chair found clear of what she knows; her view for the page hulls what she
    told. What a thing is, the classifier's own (`test_her_kinds_from_their_boxes`): held to her
    eyes' fragments the chair came and went with 0.05 px of her card's disparity against her CPU's."""
    from machine import room, sight
    body, world = _room()
    try:
        keep = sight.Keep(world)
        _looked_round(world, keep)
        occ = keep.map.occupied().any(axis=1)
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
        at = (0.0, 0.0)
        to = (room.CHAIR_AT[0], room.CHAIR_AT[1] - 0.6)
        way = sight.plan(keep.map.blocked(), at, to)
        clear = all(away(*p) >= voxels.RADIUS_M - voxels.CELL_M for p in way)
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
        moved, yaw = (-0.9, 1.2), 0.5
        bed = world.model.body('bed')
        bed.pos[:], bed.quat[:] = (moved[0], room.BED[1], moved[1]), (math.cos(yaw / 2.0), 0.0, math.sin(yaw / 2.0), 0.0)
        keep.forget()
        _looked_round(world, keep)
        found = _nearest(sight.told(keep), 'bed', moved)
        turn = None if found is None else abs(math.remainder(axis(found[1][2], found[1][3]) - axis((room.BED[0], 0.0, room.BED[2]), _turned(yaw)), math.pi))
        report.check('the bed moved and turned 0.5 rad is told within 0.3 m of its new place, its size and its turn',
                     found is not None and found[0] < 0.3 and turn is not None and turn < 0.15
                     and abs(max(found[1][2][0], found[1][2][2]) - room.BED[2]) < 0.2,
                     '%.2f m off, half (%.2f %.2f), turn off %.2f rad' % (found[0], found[1][2][0], found[1][2][2], turn)
                     if found else 'not told')
    finally:
        body.close()


#: The lodge's furniture to lay anywhere: (name, what it affords, where it stands, its seat's facing, rad, as laid);
#: a box its group's within REACH_M of where it stands; DRAWS layings, each moved, turned and sized.
FURNITURE = (('chair', 'chair', 'CHAIR_AT', math.pi), ('low chair', 'chair', 'CHAIR_LOW_AT', math.pi / 2.0),
             ('table', 'seat', 'TABLE_AT', None), ('bed', 'bed', 'BED_AT', None))
REACH_M, DRAWS = 0.7, 8


def test_her_kinds_from_their_boxes(report):
    """The classifier (`machine.affordance`) on the room's own boxes (`world.props()`, the stand-in's look) laid
    anywhere: each piece of furniture moved up to 1 m, turned any way and sized 0.8-1.25 along the floor, told what
    it affords where it now stands, its size and its turn, a seat's facing turned with it."""
    import random
    from machine import affordance, room
    body, world = _room()
    try:
        props, rng, worst = world.props(), random.Random(63100), [0.0, 0.0, 0.0]
        told = {name: 0 for name, *_ in FURNITURE}
        for _ in range(DRAWS):
            laid, want = [], []
            for name, affords, at_name, faces in FURNITURE:
                at, yaw, size = getattr(room, at_name), rng.uniform(-math.pi, math.pi), rng.uniform(0.8, 1.25)
                to = (at[0] + rng.uniform(-1.0, 1.0), at[1] + rng.uniform(-1.0, 1.0))
                R = np.array(_turned(yaw))
                group = [b for b in props if math.dist((b[1][0], b[1][2]), at) <= REACH_M and math.dist(b[1], (0, 0, 0)) > 0]
                for kind, c, h, turn in group:
                    off = R @ (size * np.array([c[0] - at[0], 0.0, c[2] - at[1]]))
                    laid.append((kind, np.array([to[0] + off[0], c[1], to[1] + off[2]]),
                                 np.array([size * h[0], h[1], size * h[2]]), R @ np.asarray(turn)))
                seat = min(group, key=lambda b: math.dist((b[1][0], b[1][2]), at))
                want.append((name, affords, to, np.array([size * seat[2][0], seat[2][1], size * seat[2][2]]),
                             None if faces is None else faces + yaw))
            for name, affords, to, half, faces in want:
                found = _nearest(laid, affords, to)
                if found is None:
                    continue
                box = found[1]
                off = abs(math.remainder(affordance.faced(box, laid) - faces, math.tau)) if faces is not None else 0.0
                worst = [max(worst[0], found[0]), max(worst[1], float(np.abs(box[2] - half).max())), max(worst[2], off)]
                told[name] += int(found[0] < 1e-6 and np.allclose(box[2], half) and off < 1e-6)
        report.check('each piece of furniture laid %d ways told what it affords where it stands, its size, its facing'
                     % DRAWS, all(n == DRAWS for n in told.values()),
                     '%s; worst %.1e m, %.1e m, %.1e rad' % (', '.join('%s %d' % kv for kv in told.items()), *worst))
    finally:
        body.close()


def axis(half, R):
    """The turn of a box's longer floor axis, rad in [0, pi)."""
    k = 0 if half[0] >= half[2] else 2
    return math.atan2(R[0][k], R[2][k]) % math.pi


def _turned(yaw):
    """A box turned `yaw` about the vertical."""
    return ((math.cos(yaw), 0.0, math.sin(yaw)), (0.0, 1.0, 0.0), (-math.sin(yaw), 0.0, math.cos(yaw)))


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
    from machine import affordance, furniture, physics, route, sight, worker
    from tools.sim import house
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    s = worker.session()
    g = s['g']
    while g['t'] < 0.1:
        worker.stepped(s)
    cells = route.blocked(g)
    at = route.centres(cells)
    held, inside, overlap, off, bent, last, walked, kept, took = [], 0, True, 0, 0.0, True, 0, 0, []
    for verb in ('sit_chair', 'lie_bed'):
        hdd, _lat, most, where = route.docks()[verb]
        _d, px, pz, _top, _thing, faced, box = furniture.nearest(g, verb, None, where)
        hdd = hdd if faced is None else faced
        out = (math.sin(hdd), math.cos(hdd))
        edge = affordance.support(box, hdd)
        d = route.staged((px + edge * out[0], pz + edge * out[1]), out, most, cells)
        to = (px + (edge + d) * out[0], pz + (edge + d) * out[1])
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
    """Her things into her registry (`machine.registry`): looked round, each look the room's boxes (`world.props()`,
    the stand-in's) seen from the head that looked, the chair (a seat with a back), the table (a seat) and the bed (a
    bed) hold slots her buses take within 0.3 m of where they stand; on her loop under `physics.SIGHT` her things
    are her map's (`sprung.LOOKS`)."""
    from machine import physics, registry, room, sight, sprung
    body, world = _room()
    try:
        keep, slots = sight.Keep(world), registry.laid()
        for k in range(LOOKS):
            _stood(world, math.tau * k / LOOKS)
            head = _head(world)
            keep.look(world, k * sight.LOOK_S, head)
            registry.seen(slots, world.props(), head, k * sight.LOOK_S)
        boxes = registry.boxes(slots)
        kinds = sorted({b[0] for b in boxes})
        for name, affords, at in (('chair', 'chair', room.CHAIR_AT), ('table', 'seat', room.TABLE_AT), ('bed', 'bed', room.BED_AT)):
            found = _nearest(boxes, affords, at)
            report.check('the %s holds a slot her buses take within 0.3 m of where it stands, a %s' % (name, affords),
                         found is not None and found[0] < 0.3,
                         '%s of %d slots, %s' % ('%.2f' % found[0] if found else 'none', len(boxes), kinds))
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
          test_her_kinds_from_their_boxes, test_her_things_fill_her_registry]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
