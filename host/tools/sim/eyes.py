"""Her eyes against the room: her map's things and her slots against the props, her way to the chair.

A look round the lodge standing; each prop's nearest told thing, its centre, its sizes and its long
axis' turn off the prop's; her registry's slots; her way to the chair's stage planned on her things
against the props'.

    python -X utf8 tools/sim/eyes.py [--looks 25]       # a look round standing, her things against the props
    python -X utf8 tools/sim/eyes.py --camera           # her fake camera's frames (`sight.Eyes.shot`), not her stereo's
    python -X utf8 tools/sim/eyes.py --at 0 0 --at 1.5 0.3 --column 0.55 -0.8   # round from where she walks
    python -X utf8 tools/sim/eyes.py --objects --near 1.5 0.8                    # her segmenter's objects in her things
    python -X utf8 tools/sim/eyes.py --errand           # her loop: a turn in place, then told to sit, then to lie
"""
import argparse
import itertools
import math
import sys
import time

import numpy as np

#: A thing told within this of a prop is its.
NEAR_M = 0.5


def _room():
    from machine.world import physics
    physics.ROOM = physics.SIGHT = 1.0
    from machine import Machine
    from machine.rig.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    world = next(n.world for n in body.others + list(a.node for a in body.actuators.values()) if hasattr(n, 'world'))
    return body, world


def _stood(world, heading, at=(0.0, 0.0)):
    from machine.body import figure
    world.reset(figure.stand(), where=(at[0], figure.standing(), at[1]),
                turn=(math.cos(heading / 2.0), 0.0, math.sin(heading / 2.0), 0.0))


def axis(half, R):
    """The turn of a box's longer floor axis, rad in [0, pi)."""
    k = 0 if half[0] >= half[2] else 2
    return math.atan2(R[0][k], R[2][k]) % math.pi


def visible(props, eye, floor_m, ceil_m, far_m):
    """The props her eyes could take in from `eye`: standing above `floor_m`, under `ceil_m`, their nearest
    point within `far_m`."""
    from machine.eyes import registry
    out = []
    for p in props:
        if len(p) < 4 or len(p[1]) != 3 or np.size(p[2]) != 3 or np.size(p[3]) != 9:
            continue
        kind, c, h, R = p[0], np.array(p[1], float), np.array(p[2], float), np.array(p[3], float).reshape(3, 3)
        near = registry.nearest(c[None], h[None], R[None], eye)[0]
        if c[1] + h[1] > floor_m and c[1] - h[1] < ceil_m and np.linalg.norm(near - eye) <= far_m:
            out.append((kind, c, h, R))
    return out


def column(chart, x, z, r=0.2):
    """Her map's evidence over the floor within `r` m of (x, z), a layer a row up to a metre: the points in its
    voxels, the looks that told them and that saw through them, how many are occupied - where a thing is lost."""
    from machine.eyes import voxels
    cx = voxels.X_M[0] + (np.arange(chart.nx) + 0.5) * chart.voxel
    cz = voxels.Z_M[0] + (np.arange(chart.nz) + 0.5) * chart.voxel
    disc, occ = np.hypot(cx[:, None] - x, cz[None, :] - z) <= r, chart.occupied()
    for j in range(chart.ny):
        lo = voxels.FLOOR_M + j * chart.voxel
        if lo < 1.0:
            print('  %.2f-%.2f m: points %5d, told %3d, free %3d, occupied %d of %d' % (
                lo, lo + chart.voxel, chart.hits[:, j][disc].sum(), chart.told[:, j][disc].sum(),
                chart.free[:, j][disc].sum(), occ[:, j][disc].sum(), disc.sum()))


def near(things, x, z, r=0.6):
    """Her things whose footprint lies within `r` m of (x, z): each one's kind, class (`classes.named`), box, top and
    bottom, whether it is seen clear over, seats her, has a back (`affordance`)."""
    from machine.eyes import classes
    from machine.way import affordance
    names = classes.named(things)
    for t, name in zip(things, names):
        if affordance.distance(t, (x, z)) <= r:
            print('  %-7s %-9s c(%5.2f %4.2f %5.2f) half(%.2f %.2f %.2f) top %.2f bottom %.2f clear %.0f sits %d backed %d' % (
                t[0], name, *t[1], *t[2], affordance.top(t), affordance.bottom(t), float(t[5]) if len(t) > 5 else 1.0,
                affordance.sits(t, things), affordance.backed(t, things)))


def matched(props, told):
    """[(prop, its nearest told thing or None, centre off (m), sizes off (m), its axis off (rad))]."""
    from machine.way import affordance
    from machine.eyes import voxels
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


def _dock_why(at, things, floor, sc, box, standoff):
    """Why her dock on `box` from `at` is laid or not, `dock.docked`'s steps in turn: its seat past her legs' reach,
    each face's middle - how many of its stages her walk back and her turns from each arrival leave clear, the first
    both do, its cell free, her walk to it, m -; 'no face' without one."""
    from machine.body import footprint
    from machine.law import poise, se2
    from machine.moves import seated
    from machine.way import affordance, dock, furniture, route, wayfinder
    from machine.eyes import voxels
    across = dock.docks()[sc][0]
    over = sc == 'sit_chair' and affordance.top(box) > seated.POPLITEAL
    if over and not seated.reaches(affordance.top(box)):
        return 'its seat %.2f m up, her legs reach no floor from it' % affordance.top(box)
    across = seated.placed(seated.ON, 0.0)[2] if over else across
    cells = route.blocked(things, floor)
    far, s_dock = dock.walked(at, cells), poise.HALF_M + standoff
    outs = s_dock + np.arange(0.0, max(footprint.carry(180.0, w)[0] for w in (1.0, -1.0)) + voxels.CELL_M + se2.STOP_SPREAD_M,
                              0.25 * voxels.CELL_M)

    def gaps(poses, h):
        return footprint.gaps(poses, h, things, floor).min(axis=1, initial=math.inf)
    out = []
    for hn in furniture.GOES[sc][2](box, things):
        n = np.array([math.sin(hn), math.cos(hn)])
        edge = np.array([float(box[1][0]), float(box[1][2])]) + affordance.support(box, hn) * n
        b = np.arange(s_dock, outs[-1] + 1e-9, 0.5 * voxels.CELL_M)
        ok = gaps(np.column_stack([edge + b[:, None] * n, np.full(len(b), hn)]), footprint.reach('back')) >= wayfinder.MARGIN
        backed = np.array([bool(ok[b <= s + 1e-9].all()) for s in outs])
        turned = np.all([np.any([gaps(np.column_stack([edge + outs[:, None] * n, np.full(len(outs), hn + math.pi + a)]),
                                      footprint.swept(d, w)) >= wayfinder.MARGIN + footprint.spread(d, w)
                                 + se2.STOP_SPREAD_M * max(0.0, math.cos(a)) for w, d in dock._ways(hn + math.pi + a, hn)], axis=0)
                         for a in dock.ARRIVALS], axis=0)
        fits = backed & turned
        s = float(outs[np.argmax(fits)]) if fits.any() else None
        cell = route._cell(*(edge + (s or 0.0) * n))
        free = route._free(cells, cell)
        far_pose = np.array([[*(edge + outs[-1] * n), 0.0]])
        worst = []
        for a in dock.ARRIVALS:
            far_pose[0, 2] = hn + math.pi + a
            worst.append(max((round(float(gaps(far_pose, footprint.swept(d, w))[0]), 2),
                              round(wayfinder.MARGIN + footprint.spread(d, w) + se2.STOP_SPREAD_M * max(0.0, math.cos(a)), 2))
                             for w, d in dock._ways(hn + math.pi + a, hn)))
        out.append('%+.0f deg: lat %.2f, backed %d of %d, turned %d, fits at %s, its cell %s, walked %s; at %.2f m out (gap, '
                   'asked) by arrival %s' % (
            math.degrees(hn), affordance.support(box, hn + 0.5 * math.pi) - across, backed.sum(), len(outs), turned.sum(),
            None if s is None else round(s, 2), 'free' if free else 'barred', round(float(far[cell]), 2) if free else None,
            outs[-1], worst))
    print_ = affordance.footprint(box)
    return out or 'no face', 'her floor %.3f, its bottom %.3f top %.3f, its footprint %s' % (
        floor, float(box[1][1]) - float(box[2][1]), float(box[1][1]) + float(box[2][1]),
        [np.round(np.asarray(p, float), 2).tolist() for p in print_])


def errand(seconds=20.0, still=False):
    """Her loop under SIGHT (the page's worker, her lodge): a turn in place for a look round - her gaze exploring as
    she wakes, or held at rest (`still`, `gaze.rest`) -, then `sit_chair`, then `lie_bed`, each from the room's middle:
    what she finds, where she ends, whether she sits or lies."""
    from machine.way import affordance, dock, furniture, route, words
    from machine.law import modal, se2
    from machine.world import physics
    from machine.process import sprung, worker
    from machine.eyes import gaze, registry
    from machine.texts import maneuver
    physics.ROOM, physics.SIGHT = 1.0, 1.0
    s = worker.session()
    g, world = s['g'], s['g']['world']
    if still:
        gaze.rest(g)
    worker.apply(s, {'twist': (0.0, 0.0, se2.YAW_RATE)})
    while g['t'] < math.tau / se2.YAW_RATE + 1.0:
        worker.stepped(s)
    worker.apply(s, {'twist': (0.0, 0.0, 0.0)})
    while g['t'] < math.tau / se2.YAW_RATE + 3.0:
        worker.stepped(s)
    things = g['things']
    print('looked round in %.1f s: %d looks, %d things; seats %s; beds %s' % (
        g['t'], g['keep'].looks, len(things),
        [(t[0], round(t[1][0], 1), round(t[1][2], 1), '+back' if affordance.backed(t, things) else '') for t in things if affordance.sits(t, things)],
        [(t[0], round(t[1][0], 1), round(t[1][2], 1)) for t in things if affordance.lies(t, things)]))
    for verb, done, at in (('sit_chair', 'seated', None), ('lie_bed', 'bed_rest', None)):
        t0 = g['t']
        hit = furniture.nearest(g, verb, at)
        here, floor = (float(world.data.qpos[0]), float(world.data.qpos[2])), min(float(f['sole'][1]) for f in g['s']['feet'])
        cells = route.blocked(g['things'], floor)
        far = dock.walked(here, cells)
        print('%s goes to %s %.2f m off (its edge), within her reach %s; her map %.0f %% blocked, %.0f %% walked to from her' % (
            verb, None if hit[6] is None else hit[6][0], hit[0], words._within(verb, hit), 100.0 * cells.mean(),
            100.0 * np.isfinite(far).mean()))
        widest = sorted(g['things'], key=lambda t: -float(t[2][0]) * float(t[2][2]))[:4]
        nearest = sorted(g['things'], key=lambda t: math.dist((float(t[1][0]), float(t[1][2])), here))[:4]
        kinds = sorted({t[0] for t in g['things']})
        for label, those in (('widest', widest), ('nearest her', nearest)):
            print('   her %s things (kind, x, z, half x, half z, bottom, top): %s' % (label, [
                (t[0], *(round(float(v), 2) for v in (t[1][0], t[1][2], t[2][0], t[2][2], t[1][1] - t[2][1], t[1][1] + t[2][1])))
                for t in those]))
        print('   her things by kind: %s' % {k: sum(1 for t in g['things'] if t[0] == k) for k in kinds})
        if hit[6] is not None:
            standoff = maneuver.standoff(verb, g['suite']) if verb in maneuver.PLAYED else 0.0
            alone = dock.docked(here, [hit[6]], floor, verb, hit[6], standoff)
            laid = dock.docked(here, g['things'], floor, verb, hit[6], standoff)
            bars = [t for t in g['things'] if t is not hit[6] and alone is not None
                    and dock.docked(here, [hit[6], t], floor, verb, hit[6], standoff) is None] if laid is None else []
            print('   its dock among her things %s, alone %s; barred alone by %s' % (
                None if laid is None else laid['stage'], None if alone is None else alone['stage'],
                [(t[0], *(round(float(v), 2) for v in (t[1][0], t[1][2], t[2][0], t[2][2], t[1][1] + t[2][1]))) for t in bars]))
            for label, those in (('alone', [hit[6]]), ('among her things', g['things'])):
                print('   why, %s: %s' % (label, _dock_why(here, those, floor, verb, hit[6], standoff)))
        swells = np.array([float(t[6]) for t in g['things'] if len(t) > 6])
        print('   their swells: median %.3f m, most %.3f, %d past a blind spot\'s most (%.2f m: moving)' % (
            np.median(swells), swells.max(), int((swells > registry.SWELL_M + 1e-9).sum()), registry.SWELL_M))
        s['refused'] = None
        worker.apply(s, {'say': verb})
        er = g.get('_errand')
        print('%s: errand %s%s' % (verb, None if er is None else {k: (round(v, 2) if isinstance(v, float) else v)
                                                                for k, v in er.items() if k in ('x_dock', 'z_dock', 'xd', 'zd', 'thing')},
                                   '' if s['refused'] is None else ', refused: %s' % s['refused']))
        reached, looks, trail, let_go, stood = None, g['keep'].looks, [], None, None
        while g['t'] < t0 + seconds and s['fallen_at'] is None:
            worker.stepped(s)
            if sprung.doing(g) == done and reached is None:
                reached = g['t']
            if reached is not None and g['t'] > reached + 2.0:
                break
            phase = (g.get('_errand') or {}).get('phase')
            if g['keep'].looks != looks:
                looks, st = g['keep'].looks, g['_gaze']
                kind = ('scan' if st['points'] else 'ground' if st['ground'] is not None else 'dock' if st['fix'] is not None
                        else 'explore' if st['explore'] else 'rest')
                trail.append('%s/%s' % (phase, kind))
            let_go = g['t'] if let_go is None and phase == 'settle' else let_go
            if let_go is not None and stood is None and math.hypot(g['est']['vel'][0], g['est']['vel'][2]) < 0.05:
                stood = g['t']
        print('   her looks (phase/gaze): %s' % ', '.join('%s x%d' % (k, sum(1 for _ in grp)) for k, grp in itertools.groupby(trail)))
        print('   her gaze let go at %s s, she stood at %s s' % (None if let_go is None else round(let_go - t0, 2),
                                                               None if stood is None else round(stood - t0, 2)))
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
    ap.add_argument('--still', action='store_true', help='errand: her gaze held at rest through her look round')
    ap.add_argument('--seconds', type=float, default=20.0, help='errand: each verb\'s seconds at most')
    ap.add_argument('--camera', action='store_true')
    ap.add_argument('--column', type=float, nargs=2, metavar=('X', 'Z'), help="her map's evidence over (x, z)")
    ap.add_argument('--objects', action='store_true', help="her looks' objects boxed by her segmenter (`instances`)")
    ap.add_argument('--near', type=float, nargs=2, action='append', metavar=('X', 'Z'),
                    help='her things round (x, z), her map\'s and her buses\'')
    ap.add_argument('--at', type=float, nargs=2, action='append', metavar=('X', 'Z'),
                    help='a place she looks round from, where she walks (the room\'s middle without)')
    ap.add_argument('--turn', type=float, default=0.0, metavar='DEG',
                    help='her headings looking round turned on by this: her looks laid another way, the same round')
    ap.add_argument('--gaze', type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=('ACROSS', 'DOWN', 'VERGE'),
                    help='deg: her eyes turned so in her head each look (`gaze.aimed`), her stereo laid by it')
    a = ap.parse_args(argv)
    if a.objects:
        import os
        from machine.eyes import instances
        os.environ[instances.SEGMENT] = '1'
    if a.errand:
        errand(seconds=a.seconds, still=a.still)
        return 0
    from machine.way import affordance, route
    from machine.eyes import gaze, registry, sight, voxels
    body, world = _room()
    try:
        sights, keep, slots = sight.Eyes(world), sight.Keep(), registry.laid()
        hb = world.model.body('head').id
        took, eyes = [], []
        for k in range(a.looks * len(a.at or [None])):
            _stood(world, math.tau * k / a.looks + math.radians(a.turn), (a.at or [(0.0, 0.0)])[k // a.looks])
            head = {'pos': np.array(world.data.xpos[hb]), 'quat': np.array(world.data.xquat[hb])}
            gz = tuple(math.radians(v) for v in a.gaze)
            gaze.aimed(world, gz)
            t0 = time.perf_counter()
            keep.took(*(sights.shot(world, head) if a.camera else sights.stereo(world, head, gz)))
            took.append(time.perf_counter() - t0)
            eyes.append(sight.eye_pose(head)[0])
            registry.seen(slots, sight.told(keep), head, k * sight.LOOK_S)
        props = visible(world.props(), np.mean(eyes, axis=0), voxels.FLOOR_M, voxels.CEIL_M, voxels.FAR_M)
        if a.column:
            print('her eyes %.2f m up; her map over (%.2f, %.2f):' % (float(np.mean(eyes, axis=0)[1]), *a.column))
            column(keep.map, *a.column)
        told = sight.told(keep)
        boxes = registry.boxes(slots)
        for x, z in a.near or ():
            for label, things in (('her map', told), ('her buses', boxes)):
                print('%s round (%.2f, %.2f):' % (label, x, z))
                near(things, x, z)
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
        at = (0.0, 0.0)
        from machine.world import room
        props = list(world.props())
        seat = min((t for t in props if affordance.sits(t, props) and affordance.backed(t, props)),
                   key=lambda t: math.dist((t[1][0], t[1][2]), room.CHAIR_AT))
        hn = affordance.fronts(seat, props)[0]
        out = (math.sin(hn), math.cos(hn))
        dock = (seat[1][0] + affordance.support(seat, hn) * out[0], seat[1][2] + affordance.support(seat, hn) * out[1])
        ways = {}
        for name, things in (('props', world.props()), ('her things', boxes)):
            cells = route.blocked(list(things), 0.0)
            d = route.staged(dock, out, route.STRAIGHT_M, cells)
            stage = (dock[0] + d * out[0], dock[1] + d * out[1])
            ways[name] = (route.way(at, stage, np.where(cells, np.inf, 1.0)), cells)
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
