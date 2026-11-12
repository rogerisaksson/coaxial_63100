"""The atlas's viewport: the child nearest it whichever page of keys holds it; its mirror world's schematic."""
import sys

from tools.dev.focus import chosen
from views_kit import Report


def _crossed(lanes):
    """How many cells of a schematic's `lanes` (atlas/mirror.py's, corners in its cells) one lane only passes
    across another in, joining none of it (atlas/schematic.py draws them broken)."""
    from terminal.views.atlas import schematic
    straight, per = {schematic.LEFT | schematic.RIGHT, schematic.UP | schematic.DOWN}, {}
    for k, (_a, _b, corners, *_inks) in enumerate(lanes):
        cells = schematic.walked(corners)
        for (pc, pr), (qc, qr) in zip(cells, cells[1:]):
            way = schematic.LEFT | schematic.RIGHT if qr == pr else schematic.UP | schematic.DOWN
            for cell in ((pc, pr), (qc, qr)):
                per.setdefault(cell, {}).setdefault(k, set()).add(way)
    return sum({way for ways in got.values() for way in ways} == straight and all(len(w) == 1 for w in got.values())
               for got in per.values())


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
    in the mirror the loop marked (↺) on a key's panel; a dot runs each flow on its lane, on in half a second."""
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
    target, distance = mirror.framed(turned, 150, 40)
    sight = scene.view(scene.camera(0.0, 0.0, distance, target), 150, 40)
    paths = mirror.laid(turned, sight, 150, 40)[1]
    now, then = (mirror.dots(turned, sight, 150, 40, t) for t in (10.0, 10.5))
    cells = mirror.drawn(turned, sight, 150, 40, 10.0)
    report.check('in the mirror the loop marked on a panel; a dot on every flow, on its lane, bright, on along '
                 'it half a second later', marked and len(now) == len(then) == len(turned['dots'])
                 and all(cell in paths[k] and cells[cell][1] == scene.TOLD for cell, (k, _f) in zip(now, turned['dots']))
                 and now != then, '%d marked, %d flows' % (len(marked), len(now)))


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
    framed = mirror.framed(over, *show_atlas.sized(state))
    report.check('the flows; M: the nodes a flow joins drawn, none other, faced whole',
                 {(0, 'heard'), (0, 'read'), (1, 'CMD')} <= said and set(over['at']) == joined < set(up)
                 and set(over['flat']['nodes'].tolist()) == joined and state['cam']['yaw'] == state['cam']['pitch'] == 0
                 and (goal['target'], goal['distance']) == framed, '%d of %d nodes drawn' % (len(joined), len(up)))
    bindings.act_on(show_atlas.PAD, ['m'], state)
    report.check('M back: the tree as it was seen', show_atlas.placed(state)['at'] == up
                 and (dict(state['goal']), state['cam']['yaw'], state['cam']['pitch']) == seen, '')


def test_the_mirror_lays_the_flow_out(report):
    """The mirror's schematic at the trunk and at each layer (atlas/mirror.py, schematic.py): her loop's
    stages round a rectangle in its order - its first half along the top left to right, the rest back along
    the bottom -, each joined to the next by what passes or comes back, the last to the first; the wire's
    asks each to a column on the right but one that closes a loop; on a drawing framed whole every lane's
    cell a line or a head or the lane's own word, no name on a lane, every node named; the lanes crossing
    counted."""
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import mirror, scene, schematic
    from tools.dev import atlas_flow, atlas_tree
    tree = atlas_tree.build()
    flows, state = atlas_flow.flows(tree), show_atlas.opened(tree)
    bindings.act_on(show_atlas.PAD, ['m'], state)
    sharp = {*schematic.GLYPHS.values(), *schematic.HEAVY.values(), *schematic.HEADS.values(), *schematic.FILLED.values()}
    looped, onward, smudged, unnamed, crossed = [], [], [], [], {}
    for focus in [-1] + list(range(len(atlas_tree.LAYERS))):
        show_atlas.focused(state, focus)
        over, up = show_atlas.placed(state), state['memo'][('placed', focus)]['at']
        at, loop, where = over['marks'], over['loop'], scene.short(tree, focus) if focus >= 0 else 'trunk'
        ends = {(scene.shown_of(tree, up, a), scene.shown_of(tree, up, b), kind) for a, b, kind, _l in flows}
        if loop:
            half = (len(loop) + 1) // 2
            top, bottom = [at[s] for s in loop[:half]], [at[s] for s in loop[half:]]
            looped.append(len({y for _x, y in top}) == len({y for _x, y in bottom}) == 1 and top[0][1] < bottom[0][1]
                          and [x for x, _y in top] == sorted({x for x, _y in top})
                          and [x for x, _y in bottom] == sorted({x for x, _y in bottom}, reverse=True)
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
        target, distance = mirror.framed(over, 150, 40)
        sight = scene.view(scene.camera(0.0, 0.0, distance, target), 150, 40)
        cells, paths, _spots = mirror.laid(over, sight, 150, 40)
        lanes = {cell for path in paths for cell in path}
        smudged += ['%s %s' % (where, cell) for cell in lanes if cell in cells
                    and cells[cell][0] not in sharp and cells[cell][1] != scene.BRIEF]
        rows = [''.join(cells.get((c, r), (' ', 0))[0] for c in range(150)) for r in range(40)]
        unnamed += ['%s: %s' % (where, name) for *_at, name, ink, node in over['texts']
                    if node is not None and ink in (scene.NAME, scene.TOLD) and not any(name in row for row in rows)]
        crossed[where] = _crossed(over['lanes'])
    report.check("her loop round its rectangle in its order, closed; the asks to the right but a loop's",
                 looped and all(looped) and not any(onward), '%d loops; %s' % (len(looped), [o for o in onward if o]))
    report.check('every lane a line, a head or its own word, no name on one; every node named; the crossings '
                 'counted', not smudged and not unnamed, '%s smudged, %s unnamed; crossings %s' % (
                     smudged[:3], unnamed[:3], ', '.join('%s %d' % kv for kv in crossed.items())))


def test_the_mirror_zooms_and_pans(report):
    """The mirror's camera (atlas/mirror.py, show_atlas.py): zoomed in, every lane still reaches its nodes -
    its ends where they stand from their signs, in the terminal's cells -, two line cells at least behind
    every head; zoomed out no farther than its framing; an arrow moves its flat world, never turns it; the
    node nearest the viewport one - lit in it, named on top and in the panel where room is kept for it -,
    the panel's room kept or not."""
    import io
    from rich.console import Console
    from terminal.ui import bindings, stage
    from terminal.views import show_atlas
    from terminal.views.atlas import leaves, mirror, scene, schematic
    from tools.dev import atlas_tree
    tree = atlas_tree.build()
    state = show_atlas.opened(tree)
    heads = {*schematic.HEADS.values(), *schematic.FILLED.values()}

    def drawn(cols, rows, keys=()):
        view = Console(file=io.StringIO(), force_terminal=True, width=cols, height=rows, color_system='truecolor',
                       theme=stage.THEME, highlight=False, legacy_windows=False)
        draw = show_atlas.draw_of(view, state)
        bindings.act_on(show_atlas.PAD, list(keys), state)
        for _ in range(3):   # the ease at its end
            state['cam'].update(target=state['goal']['target'], distance=state['goal']['distance'])
            view.print(draw())
        over, (w, h) = show_atlas.placed(state), state['art']
        return over, scene.view(state['cam'], w, h), w, h

    bindings.act_on(show_atlas.PAD, ['m', 'f6'], state)
    loose, behind, distances = [], [], []
    for zoom in range(4):
        over, sight, w, h = drawn(150, 44, ['+'] * (zoom > 0))
        cells, paths, spots = mirror.laid(over, sight, w, h)
        distances.append(state['cam']['distance'])
        for (a, b, corners, _ink, into, out, _heavy), path in zip(over['lanes'], paths):
            for end, node, cell in ((path[0], a, corners[0]), (path[-1], b, corners[-1])):
                mark, sign = over['marks'][node], spots[node][0]
                loose += [end] if end != (sign[0] + cell[0] - mark[0], sign[1] + cell[1] - mark[1]) else []
            for seq, head in ((path[::-1], into), (path, out)):
                behind += [next((k for k, c in enumerate(seq[1:]) if cells.get(c, ' ')[0] in heads), len(seq) - 1)] \
                    if head is not None and zoom == 0 else []
    drawn(150, 44, ['-'] * 8)
    report.check('zoomed in every lane reaching its nodes, two line cells behind a head; out no farther than framed',
                 not loose and min(behind) >= 2 and distances == sorted(distances, reverse=True)
                 and state['cam']['distance'] == distances[0], '%d loose ends, %d cells at least behind a head; '
                 'distances %s' % (len(loose), min(behind), ' '.join('%.2f' % d for d in distances)))
    target = state['cam']['target']
    drawn(150, 44, ['right', 'down'])
    moved, agreed = state['cam']['target'] != target and state['cam']['yaw'] == state['cam']['pitch'] == 0, []
    for cols, rows in ((150, 44), (100, 30)):
        over, sight, w, h = drawn(cols, rows)
        mid = mirror.middle(over, state['fell'], sight, w)
        cells = mirror.over(over, sight, show_atlas.keyed(state), state['fell'], w, h)
        sign, name = mirror.laid(over, sight, w, h)[2][mid]
        panel = [''.join(cells[(c, r)][0] for c in range(w - leaves.width(w), w) if cells.get((c, r), ' ')[1:] == (scene.TITLE,))
                 for r in range(h)]
        agreed.append(show_atlas.titled(state, mid, w).startswith('FLOW  ' + scene.short(tree, mid))
                      and (not over['beside'] or any(scene.short(tree, mid) in t for t in panel)) and cells[sign][1] == scene.OPEN
                      and all(cells[c][1] == scene.TOLD for c in name if c in cells))
    report.check('an arrow moves the flat world, never turns it; the node nearest the viewport one, lit, on top, '
                 'in the panel', moved and all(agreed), 'moved %s; agreed %s' % (moved, agreed))


ROSTER = (test_the_nearest_child_past_the_keys, test_the_mirror_closes_her_loop, test_the_mirror_faces_what_flows,
          test_the_mirror_lays_the_flow_out, test_the_mirror_zooms_and_pans)


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
