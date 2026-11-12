"""The atlas's viewport: the child nearest it whichever page of keys holds it; its mirror world, laid out."""
import sys

from tools.dev.focus import chosen
from views_kit import Report


def _crossed(flat, arcs, span):
    """How many pairs of a scene's flow curves (`flat`'s segments, `arcs` where each of `span` starts) cross
    one another, those meeting at a node's end left out: in the flow world's plane (x, y)."""
    from coaxial.model.blocks import numpy as np
    a, b = flat['from'][:, :2], flat['to'][:, :2]
    of = np.repeat(np.arange(len(arcs)), span)
    ends = np.concatenate((a[arcs], b[arcs + span - 1]))
    near = np.linalg.norm(ends[:, None] - ends[None], axis=2) < 1e-9
    k = len(arcs)
    met = near[:k, :k] | near[:k, k:] | near[k:, :k] | near[k:, k:]

    def side(p, q, r):
        return np.sign((q[..., 0] - p[..., 0]) * (r[..., 1] - p[..., 1]) - (q[..., 1] - p[..., 1]) * (r[..., 0] - p[..., 0]))
    i, j = np.triu_indices(len(a), 1)
    i, j = i[~met[of[i], of[j]]], j[~met[of[i], of[j]]]
    cut = (side(a[i], b[i], a[j]) * side(a[i], b[i], b[j]) < 0) & (side(a[j], b[j], a[i]) * side(a[j], b[j], b[i]) < 0)
    return len({(of[p], of[q]) for p, q in zip(i[cut].tolist(), j[cut].tolist())})


def test_the_nearest_child_past_the_keys(report):
    """The node chosen holding more children than a page of keys: turned round, wherever the child nearest the
    viewport is one past the page (no F key on it), it is the one the panel beside marks, the viewport's top
    names and the active leaf panel shows, its title no key (terminal/views/atlas/panel.py, leaves.py)."""
    from terminal.views import show_atlas
    from terminal.views.atlas import leaves, panel, scene
    from tools.dev import atlas_tree
    tree = atlas_tree.build()
    state = show_atlas.opened(tree)
    show_atlas.focused(state, next(n for n in range(len(atlas_tree.LAYERS)) if len(state['kids'][n]) > 12))
    state['cam'].update(target=state['goal']['target'], distance=state['goal']['distance'])
    placing, keys = show_atlas.placed(state), show_atlas.keyed(state)
    named = {n: keys.get(n, '') for n in show_atlas.context(state)}
    past, held = 0, 0
    for turn in range(0, 360, 15):
        state['cam']['yaw'] += 15
        cells = {}
        lines, fell, v = scene.picture(placing, state['cam'], 100, 36, keys, set(), lambda f, c, r, t: cells.update(
            leaves.panels(tree, placing['rings'], f, named, c, r, t)) or cells)
        inside = [n for n in named if n in fell and 0 <= fell[n][0] < v['w'] and 0 <= fell[n][1] < v['h']]
        nearest = min(inside, key=lambda n: fell[n][2])
        if nearest in keys:
            continue
        past += 1
        title = ''.join(ch for (_c, _r), (ch, ink) in sorted(cells.items(), key=lambda kv: kv[0][::-1])
                        if ink == scene.TITLE).strip()
        held += panel.middle(fell, v, named) == nearest and title == scene.short(tree, nearest) \
            and show_atlas.titled(state, nearest, 100).startswith('TREE  ' + scene.short(tree, nearest))
    report.check('turned round, the child nearest past the keys marked, named on top, its panel the active one',
                 past and held == past, '%d of %d turns past the keys' % (held, past))


def test_the_mirror_closes_her_loop(report):
    """The mirror's flows come back (tools/dev/atlas_flow.py): what her pass ends with, the world, to what her
    edge reads the next - the loop closed -, and the wire's readings from their handler to the host's end;
    in the mirror the loop marked (↺) on a key's panel; a dot runs each curve, on in half a second."""
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import mirror, scene
    from tools.dev import atlas_flow, atlas_tree
    tree = atlas_tree.build()
    back = [(tree.id[a], tree.id[b], label) for a, b, kind, label in atlas_flow.flows(tree)
            if atlas_flow.KINDS[kind] == 'feedback']
    loop = ('host/machine/world/physics.py', 'host/machine/senses/edge.py', 'world')
    report.check("her loop closed through the world to her edge, the wire's readings back to the host",
                 loop in back and any(b.startswith('host/') and a.startswith('comms/') for a, b, _l in back),
                 '%d flows back' % len(back))
    state = show_atlas.opened(tree)
    show_atlas.focused(state, next(i for i, row in enumerate(atlas_tree.LAYERS) if row[0] == 'executive'))
    bindings.act_on(show_atlas.PAD, ['m'], state)
    turned = show_atlas.placed(state)
    marked = [f for flows in turned['rings'].values() for f in flows if f[0] == '↺']
    now, then = (mirror.pulsed(turned, t)['flat']['segink'] == scene.TOLD for t in (10.0, 10.5))
    report.check('in the mirror the loop marked on a panel; a dot on every curve, on along it half a second later',
                 marked and now.sum() == then.sum() == len(turned['arcs']) and (now != then).any(),
                 '%d marked, %d curves' % (len(marked), len(turned['arcs'])))


def test_the_mirror_faces_what_flows(report):
    """The atlas's mirror (atlas/mirror.py, atlas_flow.py): her pass and the wire flow; M faces the world of
    the nodes a flow joins - none other drawn -, framed whole; M back: the tree as it was seen."""
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import mirror, scene
    from tools.dev import atlas_flow, atlas_tree
    tree = atlas_tree.build()
    flows = atlas_flow.flows(tree)
    said = {(kind, label.partition('_')[0]) for _a, _b, kind, label in flows}
    state = show_atlas.opened(tree)
    show_atlas.focused(state, next(i for i, row in enumerate(atlas_tree.LAYERS) if row[0] == 'executive'))
    up, seen = show_atlas.placed(state)['at'], (dict(state['goal']), state['cam']['yaw'], state['cam']['pitch'])
    ends = [(scene.shown_of(tree, up, a), scene.shown_of(tree, up, b)) for a, b, _k, _l in flows]
    joined = {n for a, b in ends if min(a, b) >= 0 and a != b for n in (a, b)}
    bindings.act_on(show_atlas.PAD, ['m'], state)
    over, goal = show_atlas.placed(state), state['goal']
    framed = mirror.framed(over, *show_atlas.sized(state), show_atlas.keyed(state))
    report.check('the flows; M: the nodes a flow joins drawn, none other, faced whole',
                 {(0, 'heard'), (0, 'read'), (1, 'CMD')} <= said and set(over['at']) == joined < set(up)
                 and set(over['flat']['nodes'].tolist()) == joined and state['cam']['yaw'] == state['cam']['pitch'] == 0
                 and (goal['target'], goal['distance']) == framed, '%d of %d nodes drawn' % (len(joined), len(up)))
    bindings.act_on(show_atlas.PAD, ['m'], state)
    report.check('M back: the tree as it was seen', show_atlas.placed(state)['at'] == up
                 and (dict(state['goal']), state['cam']['yaw'], state['cam']['pitch']) == seen, '')


def test_the_mirror_lays_the_flow_out(report):
    """The mirror world at the trunk and at each layer (atlas/mirror.py): her loop's stages round one circle
    in its order - the first at its top, on clockwise -, each joined to the next by what passes or comes back,
    the last to the first; the wire's asks each to a column on the right but one that closes a loop; every
    node named on a drawing framed whole; the curves crossing counted."""
    import math
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import mirror, scene
    from tools.dev import atlas_flow, atlas_tree
    tree = atlas_tree.build()
    flows, state = atlas_flow.flows(tree), show_atlas.opened(tree)
    bindings.act_on(show_atlas.PAD, ['m'], state)
    looped, onward, unnamed, crossed = [], [], [], {}
    for focus in [-1] + list(range(len(atlas_tree.LAYERS))):
        show_atlas.focused(state, focus)
        over, keys, up = show_atlas.placed(state), show_atlas.keyed(state), state['memo'][('placed', focus)]['at']
        at, loop, where = over['at'], over['loop'], scene.short(tree, focus) if focus >= 0 else 'trunk'
        ends = {(scene.shown_of(tree, up, a), scene.shown_of(tree, up, b), kind) for a, b, kind, _l in flows}
        if loop:
            cx, cy = (sum(at[s][k] for s in loop) / len(loop) for k in range(2))
            turns = [math.atan2(at[s][1] - cy, at[s][0] - cx) - math.pi / 2 + 2 * math.pi * k / len(loop)
                     for k, s in enumerate(loop)]
            radii = [math.hypot(at[s][0] - cx, at[s][1] - cy) for s in loop]
            looped.append(max(radii) - min(radii) < 1e-9 and all(abs(math.remainder(t, 2 * math.pi)) < 1e-9 for t in turns)
                          and all(any((s, loop[(k + 1) % len(loop)], kind) in ends for kind in (0, 2))
                                  for k, s in enumerate(loop)))
        staged = {n for a, b, kind in ends if kind == 0 for n in (a, b)}
        asks = {(a, b) for a, b, kind in ends if kind == 1 and a != b and a in at and b in at and not {a, b} & staged}
        right = {(a, b) for a, b in asks if at[b][0] > at[a][0]}
        for a, b in asks - right:   # back to the left: only where the right-running asks lead from b to a
            reached, todo = {b}, [b]
            for n in todo:
                todo += [q for p, q in right if p == n and q not in reached]
                reached |= {q for p, q in right if p == n}
            onward.append('%s: %s -> %s' % (where, scene.short(tree, a), scene.short(tree, b)) if a not in reached else '')
        target, distance = mirror.framed(over, 150, 40, keys)
        sight = scene.view(scene.camera(0.0, 0.0, distance, target), 150, 40)
        words = mirror.labels(over, sight, 150, 40, keys)
        drawn = {w for r in range(40) for w in ''.join(words.get((c, r), (' ', 0))[0] for c in range(150)).split()}
        unnamed += ['%s: %s' % (where, t[1]) for t in over['tags'] if t[3] is not None and t[1] not in drawn]
        crossed[where] = _crossed(over['flat'], over['arcs'], mirror.SPAN)
    report.check('her loop round one circle in its order, closed; the asks to the right but a loop\'s',
                 looped and all(looped) and not any(onward), '%d loops; %s' % (len(looped), [o for o in onward if o]))
    report.check('every node named; the curves crossing counted', not unnamed,
                 '%s unnamed; crossings %s' % (unnamed[:4], ', '.join('%s %d' % kv for kv in crossed.items())))


ROSTER = (test_the_nearest_child_past_the_keys, test_the_mirror_closes_her_loop, test_the_mirror_faces_what_flows,
          test_the_mirror_lays_the_flow_out)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
