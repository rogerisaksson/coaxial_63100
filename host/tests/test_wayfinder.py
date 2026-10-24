"""Her shield on the room and her wayfinder (docs/todo/protocol.md): her limbs off its boxes, her way.

The room's boxes and the patches their affordances take, her limbs held off them by their own joints;
her route's flow bent by what her shield holds, a gap's width across her way and the twist through it;
her map of what her looks told her."""
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report


def test_the_room_holds_her_limbs(report):
    """Her capsules against the room (`machine.schema`): each box the patches the text's affordance
    kernels give it - a seat her ischia and thighs', a bed her back's and hips', a table's edge her
    forearms' and palms', a step her soles' -; a capsule's axis against a box as a fine walk finds it;
    her forearm swung at a table corner held by her elbow alone, its floor the text's skin, opened by
    the approach window where her forearm's shove is scheduled on it and where her skin feels her
    forearm."""
    import numpy as np
    from machine import bemf, capsules, edge, schema, worker
    from machine.wbc import POSTURE_KD
    R = np.repeat(np.eye(3)[None], 4, 0)
    centre = np.array([[0.0, 0.45, 0.0], [0.0, 0.5, 2.0], [0.0, 0.74, -2.0], [3.0, 0.075, 0.0]])
    half = np.array([[0.22, 0.02, 0.22], [0.5, 0.1, 1.0], [0.6, 0.015, 0.35], [0.5, 0.075, 0.14]])
    took = schema.affords(centre, half, R, 0.0)
    names = {m: k for k, m in enumerate(schema.MANIFOLDS)}
    said = [{m for m in schema.MANIFOLDS if took[b, names[m]]} for b in range(4)]
    rng = np.random.default_rng(5)
    caps = {'p0': rng.normal(size=(24, 3)), 'p1': rng.normal(size=(24, 3)), 'r0': np.zeros(24), 'r1': np.zeros(24)}
    boxes = {'centre': rng.normal(size=(24, 3)), 'half': 0.1 + rng.random((24, 3)),
             'R': np.array([np.linalg.qr(rng.normal(size=(3, 3)))[0] for _ in range(24)])}
    pairs = np.stack([np.arange(24), np.arange(24)], 1)
    got = schema.boxed(caps, boxes, pairs)['d']
    ts = np.linspace(0.0, 1.0, 20001)[:, None]
    walked = np.array([schema._to_box(caps['p0'][k] + ts * (caps['p1'][k] - caps['p0'][k]),
                                      np.repeat(boxes['centre'][k:k + 1], len(ts), 0), boxes['half'][k],
                                      np.repeat(boxes['R'][k:k + 1], len(ts), 0))[0].min() for k in range(24)])
    report.check('a seat, a bed, a table, a step take their patches; 24 axes against boxes within %.1e m of a fine walk, '
                 'the golden bracket %d steps' % (float(np.abs(got - walked).max()), schema.SECTIONS),
                 {'ischial-bilateral', 'posterior-femoral-bilateral'} <= said[0]
                 and {'dorsal-supine-continuum', 'trochanter-gluteal-lateral'} <= said[1]
                 and {'ulnar-carpal-shove-bilateral', 'palmar-carpal-bilateral'} <= said[2]
                 and said[3] == {'plantar-bilateral', 'metatarsal-equinus-bilateral'}
                 and float(np.abs(got - walked).max())
                 < schema._GOLD ** schema.SECTIONS * float(np.sqrt(((caps['p1'] - caps['p0']) ** 2).sum(1)).max()),
                 str(said))
    s = worker.session()
    while s['g']['t'] < 1.0:
        worker.stepped(s)
    g = s['g']
    body, st, ask, at, d = g['body'], g['s'], g['ask'], g['schema_at'], g['schema_d']
    volts = float(g['sense'][edge.LINK][0])
    caps = capsules.laid(capsules.posed(body, d, st), at)
    arm = capsules.NAMES.index('forearm-ulnar-l')
    tip = caps['p1'][arm]
    corner = (tip + np.array([0.08, 0.0, 0.0]), np.array([0.04, 0.04, 0.04]), np.eye(3))
    props = [('table', tuple(corner[0]), tuple(corner[1]), tuple(map(tuple, corner[2])))]
    weights, touched = np.zeros(len(schema.MANIFOLDS)), np.zeros(len(capsules.NAMES))
    near = schema.boxed(caps, schema.room(props, weights, 0.0, touched), np.array([[arm, 0]]))
    jp = np.zeros((3, body.m.nv))
    body.mj.mj_jac(body.m, d, jp, None, near['a'][0], int(at[arm]))
    J = near['n'][0] @ jp[:, :body.n]
    moved = dict(st, v=st['v'] - 2.0 * J / float(J @ J))

    def shielded(weights, touched):
        return schema.shielded(body, d, moved, ask, volts, at, schema.room(props, weights, 0.0, touched))
    posture, held = shielded(weights, touched)
    palm = weights.copy()
    palm[list(schema.MANIFOLDS).index('ulnar-carpal-shove-bilateral')] = 1.0
    felt = touched.copy()
    felt[arm] = 1.0
    opened, by_palm = shielded(palm, touched)
    _p, by_skin = shielded(weights, felt)
    change = POSTURE_KD[0] * (posture - np.asarray(ask['posture']))
    elbow = bemf.DRIVEN.index('left_elbow')
    row = [list(x['room']['pairs'][:, 0]).index(arm) for x in (held, by_palm, by_skin)]
    report.check("her forearm %.0f mm off a table's corner closing at 2 m/s: held, %.1f rad/s^2 on her left elbow "
                 "alone (%d rows); her forearm's shove scheduled on the table opens it by the window (h %.3f -> %.3f m), "
                 "her skin feeling her forearm too" % (1e3 * float(near['d'][0]), float(change[elbow]),
                                                      len(held['room']['lam']), float(held['room']['h'][row[0]]),
                                                      float(by_palm['room']['h'][row[1]])),
                 float(np.abs(change[elbow])) > 0.0 and np.count_nonzero(change) == 1
                 and held['room']['lam'][row[0]] > 0.0
                 and np.isclose(float(by_palm['room']['h'][row[1]] - held['room']['h'][row[0]]), schema.WINDOW)
                 and np.isclose(float(by_skin['room']['h'][row[2]] - held['room']['h'][row[0]]), schema.WINDOW), '')


def test_the_wayfinder_bends_her_flow(report):
    """The wayfinder's two ways (`machine.wayfinder`): a held row's Sherman-Morrison pullback takes
    eta lam / (1 + eta lam) of her flow into the box off it, none of her flow along it or away, a word's
    twist untouched; the free width across her way through a doorway and inside its post; the
    trunk's twist that fits her through a gap - none square on, past twice atan(depth / across) under
    her shoulders, the text's most past her contorted width."""
    import math
    import numpy as np
    from machine import wayfinder as w
    lam = 0.1
    into = w.deflected((0.0, 1.0), [(0.0, -1.0)], [lam])
    along = w.deflected((1.0, 0.0), [(0.0, -1.0)], [lam])
    away = w.deflected((0.0, -1.0), [(0.0, -1.0)], [lam])
    keep = w.ETA * lam / (1.0 + w.ETA * lam)
    room = {'room': {'n': [(0.0, 0.0, -1.0)], 'lam': [lam]}}
    told, flow = w.turned((0.7, 0.0, 0.1), 0.0, room, 0.0), w.turned((0.7, 0.0, 0.1), 0.0, room, 1.0)
    report.check("into a box %.3f of her flow taken, along and away none; a word's twist kept, a route's bent"
                 % keep, math.isclose(into[1], 1.0 - keep) and into[0] == 0.0 and along == (1.0, 0.0)
                 and away == (0.0, -1.0) and told == (0.7, 0.0, 0.1) and math.isclose(flow[0], 0.7 * (1.0 - keep)), '')
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


def test_her_map_keeps_what_her_looks_told(report):
    """Her registry (`machine.registry`): three things before her looked at thrice confirmed, none behind
    her; a chair shoved at 1 m/s under the stand-in's looks (8 ms apart) kept one slot, moving once its
    drift passes the text's chi2, its velocity within 5 %; a pane 1.1 m from its twin never takes its
    slot; a thing before her unseen pruned at the text's rate, her head aside; the lamp out of her view
    kept 45 s, swollen 1.5 mm/s to 120 mm, missed past them; 256 kinds in one look, a 257th takes the least
    salient's slot, not the thrice-seen one's; her buses' boxes the slots at 1.8."""
    import math
    import numpy as np
    from machine import registry, schema
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
                 and slots['kinds'][:3] == ['chair', 'table', 'lamp'] and panes['kinds'].count('glass') == 2
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


ROSTER = (test_the_room_holds_her_limbs, test_the_wayfinder_bends_her_flow, test_her_map_keeps_what_her_looks_told)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
