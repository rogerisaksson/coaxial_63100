"""Her eyes against the room: her map's things and her slots against the props, her way to the chair.

A look round the lodge standing; each prop's nearest told thing, its centre, its sizes and its long
axis' turn off the prop's; her registry's slots; her way to the chair's stage planned on her things
against the props'.

    python -X utf8 tools/sim/eyes.py [--looks 25]       # a look round standing, her things against the props
    python -X utf8 tools/sim/eyes.py --errand           # her loop: a turn in place, then told to sit, then to lie
"""
import argparse
import math
import sys
import time

import numpy as np

#: A thing told within this of a prop is its.
NEAR_M = 0.5


def _room():
    from machine import physics
    physics.ROOM = physics.SIGHT = 1.0
    from machine import Machine
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    world = next(n.world for n in body.others + list(a.node for a in body.actuators.values()) if hasattr(n, 'world'))
    return body, world


def _stood(world, heading):
    from machine import figure
    world.reset(figure.stand(), where=(0.0, figure.standing(), 0.0),
                turn=(math.cos(heading / 2.0), 0.0, math.sin(heading / 2.0), 0.0))


def axis(half, R):
    """The turn of a box's longer floor axis, rad in [0, pi)."""
    k = 0 if half[0] >= half[2] else 2
    return math.atan2(R[0][k], R[2][k]) % math.pi


def visible(props, eye, floor_m, ceil_m, far_m):
    """The props her eyes could take in from `eye`: standing above `floor_m`, under `ceil_m`, their nearest
    point within `far_m`."""
    from machine import registry
    out = []
    for p in props:
        if len(p) < 4 or len(p[1]) != 3 or np.size(p[2]) != 3 or np.size(p[3]) != 9:
            continue
        kind, c, h, R = p[0], np.array(p[1], float), np.array(p[2], float), np.array(p[3], float).reshape(3, 3)
        near = registry.nearest(c[None], h[None], R[None], eye)[0]
        if c[1] + h[1] > floor_m and c[1] - h[1] < ceil_m and np.linalg.norm(near - eye) <= far_m:
            out.append((kind, c, h, R))
    return out


def matched(props, told):
    """[(prop, its nearest told thing or None, centre off (m), sizes off (m), its axis off (rad))]."""
    from machine import affordance, voxels
    out = []
    for kind, c, h, R in props:
        best = None
        for t in told:
            tc = np.array(t[1], float)
            d = math.hypot(tc[0] - c[0], tc[2] - c[2])
            # the prop's centre within the told box, a cell round, before the nearest centre
            reach = affordance._within(t, (kind, c, (0.0, 0.0, 0.0), R))
            inside = bool((reach[:, 0] >= -np.array([t[2][0], t[2][2]]) - voxels.CELL_M).all()
                          and (reach[:, 1] <= np.array([t[2][0], t[2][2]]) + voxels.CELL_M).all())
            if best is None or (inside, -d) > (best[2], -best[0]):
                best = (d, t, inside)
        if best is None or (best[0] > NEAR_M and not best[2]):
            out.append(((kind, c, h, R), None, math.inf, math.inf, math.inf))
            continue
        d, t, _inside = best
        th, tR = np.array(t[2], float), np.array(t[3], float).reshape(3, 3)
        sizes = float(np.abs(np.sort(th[[0, 2]]) - np.sort(h[[0, 2]])).max())
        turn = abs(math.remainder(axis(th, tR) - axis(h, R), math.pi)) if abs(h[0] - h[2]) > 0.1 else 0.0
        out.append(((kind, c, h, R), t, d, sizes, turn))
    return out


def errand(seconds=20.0):
    """Her loop under SIGHT (the page's worker, her lodge): a turn in place for a look round, then
    `sit_chair`, then `lie_bed`, each from the room's middle: what she finds, where she ends, whether she
    sits or lies."""
    from machine import affordance, modal, physics, se2, sprung, worker
    physics.ROOM, physics.SIGHT = 1.0, 1.0
    s = worker.session()
    g, world = s['g'], s['g']['world']
    worker.apply(s, {'twist': (0.0, 0.0, se2.YAW_RATE)})
    while g['t'] < math.tau / se2.YAW_RATE + 1.0:
        worker.stepped(s)
    worker.apply(s, {'twist': (0.0, 0.0, 0.0)})
    while g['t'] < math.tau / se2.YAW_RATE + 3.0:
        worker.stepped(s)
    things = g['things']
    print('looked round in %.1f s: %d looks, %d things; seats %s; beds %s' % (
        g['t'], g['keep'].map.looks, len(things),
        [(t[0], round(t[1][0], 1), round(t[1][2], 1), '+back' if affordance.backed(t, things) else '') for t in things if affordance.sits(t, things)],
        [(t[0], round(t[1][0], 1), round(t[1][2], 1)) for t in things if affordance.lies(t, things)]))
    for verb, done, at in (('sit_chair', 'seated', None), ('lie_bed', 'bed_rest', None)):
        t0 = g['t']
        worker.apply(s, {'say': verb})
        er = g.get('_errand')
        print('%s: errand %s' % (verb, None if er is None else {k: (round(v, 2) if isinstance(v, float) else v)
                                                              for k, v in er.items() if k in ('x_dock', 'z_dock', 'xd', 'zd', 'thing')}))
        reached = None
        while g['t'] < t0 + seconds and s['fallen_at'] is None:
            worker.stepped(s)
            if sprung.doing(g) == done and reached is None:
                reached = g['t']
            if reached is not None and g['t'] > reached + 2.0:
                break
        d = world.data
        print('   at %.1f s: doing %s%s, fallen %s, at (%.2f %.2f %.2f), prop bears %.2f, loads %s' % (
            g['t'], sprung.doing(g), '' if reached is None else ' since %.1f' % reached, s['fallen_at'], *d.qpos[:3],
            float(g['y'][modal.PROP]), [round(v) for v in g['loads']]))
        if verb == 'sit_chair':
            worker.apply(s, {'say': 'rise_chair'})
            t1 = g['t']
            while g['t'] < t1 + 8.0 and s['fallen_at'] is None:
                worker.stepped(s)
            print('   risen: doing %s at %.1f s, fallen %s' % (sprung.doing(g), g['t'], s['fallen_at']))
        if s['fallen_at'] is not None:
            break
    physics.SIGHT = 0.0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--looks', type=int, default=25)
    ap.add_argument('--errand', action='store_true')
    a = ap.parse_args(argv)
    if a.errand:
        errand()
        return 0
    from machine import affordance, registry, route, sight, voxels
    body, world = _room()
    try:
        keep, slots = sight.Keep(world), registry.laid()
        hb = world.model.body('head').id
        took, eyes = [], []
        for k in range(a.looks):
            _stood(world, math.tau * k / a.looks)
            head = {'pos': np.array(world.data.xpos[hb]), 'quat': np.array(world.data.xquat[hb])}
            t0 = time.perf_counter()
            keep.look(world, k * sight.LOOK_S, head)
            took.append(time.perf_counter() - t0)
            eyes.append(sight.eye_pose(head)[0])
            registry.seen(slots, sight.told(keep), head, k * sight.LOOK_S)
        props = visible(world.props(), np.mean(eyes, axis=0), voxels.FLOOR_M, voxels.CEIL_M, voxels.FAR_M)
        told = sight.told(keep)
        boxes = registry.boxes(slots)
        print('%d looks, %.1f ms each; %d props her eyes could take in, %d things told, %d slots taken by her buses (%d live)'
              % (a.looks, 1e3 * float(np.median(took)), len(props), len(told), len(boxes),
                 int((slots['rows']['on'] > 0.0).sum())))
        print('%-8s %-24s %-8s %8s %8s %8s' % ('prop', 'at', 'told', 'off m', 'sizes m', 'turn deg'))
        rows = matched(props, told)
        for (kind, c, h, _R), t, d, sizes, turn in sorted(rows, key=lambda r: (r[1] is not None, r[2])):
            print('%-8s (%5.2f %4.2f %5.2f) %-8s %8s %8s %8s' % (
                kind, c[0], c[1], c[2], t[0] if t else '-', '%.2f' % d if t else '-', '%.2f' % sizes if t else '-',
                '%.0f' % math.degrees(turn) if t else '-'))
        found = [r for r in rows if r[1] is not None]
        strays = [t for t in told if id(t) not in {id(r[1]) for r in found}]
        print('told: %d of %d props within %.1f m; centre off median %.2f m, sizes %.2f, turn %.0f deg; %d things no prop\'s'
              % (len(found), len(props), NEAR_M, np.median([r[2] for r in found]) if found else math.nan,
                 np.median([r[3] for r in found]) if found else math.nan,
                 math.degrees(np.median([r[4] for r in found])) if found else math.nan, len(strays)))
        for t in sorted(strays, key=lambda t: -float(np.prod(t[2]))):
            print('   stray %-7s at (%5.2f %4.2f %5.2f) half (%.2f %.2f %.2f)' % (t[0], *t[1], *t[2]))
        by = {}
        for (kind, *_r), t, *_m in found:
            by.setdefault(kind, []).append(t[0])
        print('kinds told by prop kind: %s' % {k: sorted(set(v)) for k, v in by.items()})
        for name, affords in (('seats', affordance.sits), ('beds', affordance.lies)):
            for label, things in (('her map', told), ('her buses', boxes), ('the props', world.props())):
                mine = [t for t in things if len(t) >= 4 and affords(t, things)]
                print("%s among %s: %d (%s)" % (name, label, len(mine), ', '.join(
                    '%s (%.1f %.1f)%s' % (t[0], t[1][0], t[1][2], '+back' if affordance.backed(t, things) else '') for t in mine)))
        # her way to the chair's stage on her things and on the props
        from types import SimpleNamespace
        at = (0.0, 0.0)
        specs = route.docks()['sit_chair']
        from machine import room
        dock = (room.CHAIR_AT[0] + specs[0], room.CHAIR_AT[1] + specs[1])
        out = (math.sin(specs[2]), math.cos(specs[2]))
        ways = {}
        for name, things in (('props', world.props()), ('her things', boxes)):
            g = {'things': list(things), 'world': SimpleNamespace(data=SimpleNamespace(qpos=np.array([at[0], 0.0, at[1]])))}
            cells = route.blocked(g)
            d = route.staged(dock, out, specs[4], cells)
            stage = (dock[0] + d * out[0], dock[1] + d * out[1])
            ways[name] = (route.way(g, stage, cells), cells)
        truth = ways['props'][1]
        hers = ways['her things'][0]
        crossed = sum(int(truth[route._cell(x, z)]) for x, z in hers if 0 <= route._cell(x, z)[0] < truth.shape[0])
        print('her way to the chair on the props: %s' % ' -> '.join('(%+.2f %+.2f)' % w for w in ways['props'][0]))
        print('her way on her things:             %s; %d of its corners in the props\' grown cells; blocked cells %d of %d'
              % (' -> '.join('(%+.2f %+.2f)' % w for w in hers), crossed, int(ways['her things'][1].sum()), int(truth.sum())))
    finally:
        body.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
