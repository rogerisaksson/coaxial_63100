"""Her wayfinder and her map (docs/todo/protocol.md): a gap's width across her way, what her looks told her.

The gauge's width through a doorway and the trunk's twist that fits her through it; her registry's slots
by her looks, in and out of her eyes' frustum."""
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report


def test_the_wayfinder_gauges_her_way(report):
    """The wayfinder's gauge (`machine.way.wayfinder`): the free width across her way through a doorway and
    inside its post; the trunk's twist that fits her through a gap - none square on, past twice
    atan(depth / across) under her shoulders, the text's most past her contorted width."""
    import math
    import numpy as np
    from machine.way import wayfinder as w
    centre = np.array([[-0.6, 1.0, 0.0], [0.6, 1.0, 0.0], [0.0, 0.3, 0.0]])
    half = np.array([[0.1, 1.0, 0.1], [0.1, 1.0, 0.1], [0.5, 0.3, 0.5]])
    R = np.repeat(np.eye(3)[None], 3, 0)
    widths = (w.gap(centre, half, R, (0.0, 0.0), (0.0, 1.0), 0.0, 2.0), w.gap(centre, half, R, (0.0, 1.0), (0.0, 1.0), 0.0, 2.0),
              w.gap(centre, half, R, (-0.6, 0.0), (0.0, 1.0), 0.0, 2.0))
    across, depth = w.NOMINAL, w.DEPTH
    first = 2.0 * math.atan2(depth, across)
    twists = [w.sliced(f * across, across, depth) for f in (1.2, 0.99, 0.8, 0.3)]
    report.check('a doorway 1.0 m between its posts (a low box under the band not), 4.0 past them, 0 in a post; '
                 'her chest %.0f mm across, %.0f deep: twists %s deg at 1.2, 0.99, 0.8, 0.3 of it'
                 % (1e3 * across, 1e3 * depth, [round(math.degrees(t), 1) for t in twists]),
                 bool(np.allclose(widths, (1.0, 4.0, 0.0))) and twists[0] == 0.0 and twists[1] >= first
                 and twists[1] < twists[2] <= w.TWIST and twists[3] == w.TWIST
                 and math.isclose(across * math.cos(twists[2]) + depth * math.sin(twists[2]), 0.8 * across), '')
    # the room's boxes laid to a doubling's capacity (`schema.room`), the last told again: every gap the same bits
    from machine.law import schema
    rng, off, counts = np.random.default_rng(7), 0, {}
    for n in (0, 1, 3, 5, 12, 64, 65, 107):
        turn = np.linalg.qr(rng.normal(size=(n, 3, 3)))[0]
        props = [('wood', c, h, r) for c, h, r in zip(rng.uniform(-3.0, 3.0, (n, 3)), rng.uniform(0.05, 0.8, (n, 3)), turn)]
        laid, plain = schema.room(props), [np.array([p[k] for p in props], float).reshape(shape)
                                           for k, shape in ((1, (-1, 3)), (2, (-1, 3)), (3, (-1, 3, 3)))]
        counts[n] = len(laid['centre'])
        for at in rng.uniform(-3.0, 3.0, (24, 2)):
            way = rng.normal(size=2)
            way /= np.linalg.norm(way)
            a, b = (float(w.gap(c, h, r, at, way, 0.0, w.NOMINAL)) for c, h, r in ((laid['centre'], laid['half'], laid['R']), plain))
            off += a != b
    report.check('her room\'s boxes laid to a doubling\'s capacity gauge every gap the same to the bit: %s' % counts,
                 off == 0 and counts == {0: 0, 1: 1, 3: 4, 5: 8, 12: 16, 64: 64, 65: 128, 107: 128}, '%d gaps off' % off)


def test_her_map_keeps_what_her_looks_told(report):
    """Her registry (`machine.eyes.registry`): three things before her looked at thrice confirmed, none behind
    her; a chair shoved at 1 m/s under the stand-in's looks (8 ms apart) kept one slot, moving once its
    drift passes the text's chi2, its velocity within 5 %; a pane 1.1 m from its twin never takes its
    slot; a thing before her unseen pruned at the text's rate, her head aside; the lamp out of her view
    kept 45 s, swollen 1.5 mm/s to 120 mm, missed past them; 256 kinds in one look, a 257th takes the least
    salient's slot, not the thrice-seen one's; her buses' boxes the slots at 1.8."""
    import math
    import numpy as np
    from machine.eyes import registry
    from machine.law import schema
    head = {'pos': np.array([0.0, 1.6, 0.0]), 'quat': np.array([1.0, 0.0, 0.0, 0.0])}
    eye = np.eye(3)
    before = [('chair', (0.3, 0.4, 2.0), (0.2, 0.4, 0.2), eye), ('table', (-0.5, 0.7, 2.5), (0.6, 0.02, 0.4), eye),
              ('lamp', (0.8, 1.0, 2.8), (0.1, 0.5, 0.1), eye)]
    behind = [('bed', (0.0, 0.3, -2.0), (0.5, 0.2, 1.0), eye)]
    slots = registry.laid()
    for k in range(3):
        registry.seen(slots, before + behind, head, 0.1 * k)
    r = slots['rows']
    first = r['salience'][:3].tolist()
    dt, flagged = schema.REFRESH_S, None
    for n in range(1, 41):
        chair = [('chair', (0.3, 0.4, 2.0 + 1.0 * n * dt), (0.2, 0.4, 0.2), eye)]
        registry.seen(slots, chair + before[1:], head, 0.2 + n * dt)
        flagged = n if flagged is None and r['moving'][0] else flagged
    speed = float(np.linalg.norm(r['vel'][0]))
    panes = registry.laid()
    registry.seen(panes, [('glass', (0.0, 1.2, 3.0), (0.01, 1.0, 0.5), eye)], head, 0.0)
    registry.seen(panes, [('glass', (1.1, 1.2, 3.0), (0.01, 1.0, 0.5), eye)], head, 0.1)
    report.check("three things before her thrice looked at: salience %.2f each, confirmed, the bed behind her none; the "
                 "chair shoved at 1 m/s one slot, moving from its %s. look at %.2f m/s; a pane's twin 1.1 m off its own slot"
                 % (first[0], flagged, speed),
                 first == [3 * registry.HIT] * 3 and all(v >= registry.CONFIRM for v in first) and int(r['on'].sum()) == 3
                 and slots['kinds'].count('chair') == 1 and flagged is not None and r['moving'][0] == 1.0
                 and math.isclose(speed, 1.0, rel_tol=0.05) and float(r['drift'][0]) > registry.MOVING
                 and slots['kinds'][:3] == ['chair', 'table', 'lamp'] and panes['kinds'].count('window') == 2
                 and panes['rows']['moving'][:2].sum() == 0.0, '%s %s %s' % (first, flagged, speed))
    looks, had, lamp = 0, float(r['salience'][1]), float(r['salience'][2])
    aside = {'pos': head['pos'], 'quat': np.array([math.cos(-0.3), 0.0, math.sin(-0.3), 0.0])}
    turned = {'pos': head['pos'], 'quat': np.array([0.0, 0.0, 1.0, 0.0])}
    while r['on'][1] and looks < 1000:
        registry.seen(slots, [], aside, 1.5 + 0.1 * (looks + 1))
        looks += 1
    t = 1.5 + 0.1 * looks
    registry.passed(slots, turned, registry.KEPT_S)
    swell_kept, sal_kept = float(r['swell'][2]), float(r['salience'][2])
    registry.seen(slots, [], turned, t + registry.KEPT_S + 1.0)
    registry.passed(slots, turned, 100.0)
    report.check('the table unseen before her pruned after %d looks from %.2f, her head aside; the lamp out of view kept at '
                 '%.2f, swollen %.1f mm after 45 s, then missed once, %.0f mm at most'
                 % (looks, had, sal_kept, 1e3 * swell_kept, 1e3 * float(r['swell'][2])),
                 looks == math.ceil((had - registry.PRUNE) / -registry.MISS)
                 and sal_kept == lamp and math.isclose(swell_kept, registry.SWELL_MPS * registry.KEPT_S)
                 and math.isclose(float(r['salience'][2]), sal_kept + registry.MISS) and r['swell'][2] == registry.SWELL_M,
                 str(r['salience'][:3]))
    full, at = registry.laid(), (0.0, 1.0, 1.5)
    registry.seen(full, [('thing%d' % k, at, (0.1, 0.1, 0.1), eye) for k in range(registry.SLOTS)], head, 0.0)
    for n in range(2):
        registry.seen(full, [('thing0', at, (0.1, 0.1, 0.1), eye)], head, 0.1 * (n + 1))
    registry.seen(full, [('new', at, (0.1, 0.1, 0.1), eye)], head, 0.3)
    taken = registry.boxes(full)
    report.check("%d slots full: a new thing takes the least salient's, not the thrice-seen one's; the buses take %d at %.1f"
                 % (registry.SLOTS, len(taken), registry.TAKEN),
                 full['kinds'][:2] == ['thing0', 'new'] and full['kinds'].count('') == 0
                 and len(taken) == 1 and taken[0][0] == 'thing0' and tuple(taken[0][1]) == at, str(full['kinds'][:3]))


def test_her_wayfinder_apart(report):
    """Her wayfinder apart (`machine.way.planner`): the lodge's chair docked by her planner's process at once,
    as answered here, her way to it watched LATENCY_S on, none before; what affords a verb kept of hers and of things
    given her each (`furniture._afforded`); a table top she goes to opened to her reach ahead
    before its face, behind it her reach across (`:affordance-aperture`); the text's clothoid her
    arc's, its rate her arc over a step (smooth_interactive_wayfinder.lisp as fitted)."""
    import math
    import numpy as np
    from machine.way import furniture, obstacles, planner, route
    from machine.texts import maneuver
    from machine.world import physics
    from machine.law import se2
    from machine.process import worker
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    s = worker.session()
    g = s['g']
    while g['t'] < 0.1:
        worker.stepped(s)
    floor = min(float(f['sole'][1]) for f in g['s']['feet'])
    box = furniture.nearest(g, 'sit_chair')[6]
    # her errand's look asks of hers and of things given it each pass (`words._seen`, `words._glimpsed`): each kept
    given, anew = list(g['things']), furniture._afforded({'things': g['things']}, 'sit_chair')
    hers, theirs = furniture._afforded(g, 'sit_chair'), furniture._afforded(g, 'sit_chair', given)
    kept = furniture._afforded(g, 'sit_chair') + furniture._afforded(g, 'sit_chair', given)
    alike = lambda a, b: len(a) == len(b) and all(p is q and own == mine for (p, own), (q, mine) in zip(a, b))
    report.check('what affords her sitting told of hers and of things given her, each kept: neither anew for the other',
                 all(a is b for a, b in zip(kept, hers + theirs)) and bool(anew[1])
                 and alike(hers[1], anew[1]) and alike(theirs[1], anew[1]), '%d afford it' % len(anew[1]))
    ask = {'kind': 'dock', 'at': (0.0, 0.0), 'yaw': 0.0, 'still': True, 'things': list(g['things']), 'floor': floor,
           'seen': None, 'slot': int(box[4]) if box is not None else -1, 'face': None, 'to': None, 'warm': (),
           'dock': ('sit_chair', box, maneuver.standoff('sit_chair')), 'passed': 0.0, 'way': ()}
    here: dict = planner.answered(ask)
    laid = here.get('laid') or {}
    told = early = watched = None
    # no dock laid, no way to watch: a watch of none killed her planner's process and the suite with it
    if 'stage' in laid:
        watch = dict(ask, kind='watch', to=laid['stage'], face=laid['face'], way=[(0.0, 0.0), laid['stage']])
        p = planner.Planner()
        try:
            p.ask(1.0, ask)
            told = p.answer(1.0)
            p.ask(2.0, watch)
            early, watched = p.answer(2.0 + 0.5 * planner.LATENCY_S), p.answer(2.0 + planner.LATENCY_S)
        finally:
            p.close()
    report.check('a dock answered at once as here, stage %s; a watch %.2f s on, none before' % (laid.get('stage'), planner.LATENCY_S),
                 told is not None and told.get('laid') == laid and told.get('stops') == here.get('stops')
                 and early is None and watched is not None, '%s %s' % (told, watched) if laid else 'no dock laid: %s' % here)
    top = [('wood', (0.0, floor + 0.9, 0.0), (0.4, 0.05, 0.3), np.eye(3), 7)]
    at = [(0.0, 0.3 + d) for d in (0.3, 0.36)] + [(0.0, -0.3 - d) for d in (0.3, 0.36)]
    plain, opened = (np.isinf(obstacles.costed(at, top, floor, np.ones(4), 7, f)) for f in (None, (0.0, 1.0)))
    report.check('the top before its face blocked 0.30 m out, free 0.36 (0.38 plain); behind it blocked both',
                 list(plain) == [True, True, True, True] and list(opened) == [True, False, True, True], '%s %s' % (plain, opened))
    c = obstacles.laws()['clothoid']
    report.check('the clothoid within her arc (%.2f /m), its rate her arc over a step (%.2f /m s)' % (c['kappa-max'], c['dkappa-max']),
                 math.isclose(c['kappa-max'], 1.0 / route.ARC_M, abs_tol=0.01)
                 and math.isclose(c['dkappa-max'], 1.0 / route.ARC_M / (0.5 * se2.BRAKE_S), abs_tol=0.01), str(c))


def test_her_gaps_as_numpys_einsums(report):
    """Her gaps to her things (`footprint.gaps`, its sums over pairs written out, her things' arrays laid once a list)
    and her floor's distances to them (`obstacles.apart`, the same) as numpy's einsums give them (`footprint.gaps_twin`,
    `obstacles.apart_twin`), bit for bit at every read her planner makes docking at the lodge's chair, its bed and its
    door."""
    import numpy as np
    from machine.body import footprint
    from machine.way import furniture, obstacles, planner
    from machine.texts import maneuver
    from machine.world import physics
    from machine.process import worker
    was, tally = (physics.ROOM, physics.SIGHT), {'calls': 0, 'apart': 0, 'cells': 0, 'costs': 0}
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    real, real_apart = footprint.gaps, obstacles.apart

    def differ(got, twin):
        return got.shape != twin.shape or not np.array_equal(got.view(np.int64), twin.view(np.int64))

    def beside(poses, h, things, floor):
        got = real(poses, h, things, floor)
        tally['apart'] += differ(got, footprint.gaps_twin(poses, h, things, floor))
        tally['calls'] += 1
        tally['cells'] += got.size
        return got

    def apart(points, things):
        got = real_apart(points, things)
        tally['apart'] += differ(got, obstacles.apart_twin(points, things))
        tally['costs'] += 1
        return got
    footprint.gaps, obstacles.apart = beside, apart
    try:
        s = worker.session()
        g = s['g']
        while g['t'] < 0.1:
            worker.stepped(s)
        floor = min(float(f['sole'][1]) for f in g['s']['feet'])
        for verb in ('sit_chair', 'lie_bed', 'exit_door'):
            box = furniture.nearest(g, verb)[6]
            if box is None:
                tally['apart'] += 1
                continue
            standoff = maneuver.standoff(verb) if verb in maneuver.PLAYED else 0.0
            for at in ((0.0, 0.0), (1.0, 1.5), (-1.0, 2.5), (1.5, -0.5)):
                planner.answered({'kind': 'dock', 'at': at, 'yaw': 0.0, 'still': True, 'things': list(g['things']),
                                  'floor': floor, 'seen': None, 'slot': int(box[4]) if len(box) > 4 else -1,
                                  'face': None, 'to': None, 'warm': (), 'dock': (verb, box, standoff), 'passed': 0.0,
                                  'way': ()})
    finally:
        footprint.gaps, obstacles.apart = real, real_apart
        physics.ROOM, physics.SIGHT = was
    report.check('her gaps and her floor\'s distances as numpy\'s einsums docking at the chair, the bed and the door: '
                 '%(calls)d reads, %(cells)d gaps, %(costs)d floors' % tally,
                 tally['calls'] > 100 and tally['costs'] > 5 and not tally['apart'], '%(apart)d apart' % tally)


ROSTER = (test_the_wayfinder_gauges_her_way, test_her_map_keeps_what_her_looks_told, test_her_wayfinder_apart,
          test_her_gaps_as_numpys_einsums)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
