"""The atlas's viewport: the child nearest it whichever page of keys holds it, its mirror's loops and pulses."""
import sys

from tools.dev.focus import chosen
from views_kit import Report


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


ROSTER = (test_the_nearest_child_past_the_keys, test_the_mirror_closes_her_loop)


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
