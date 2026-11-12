"""The atlas by application: each page's system by what it calls, the view of them, the mirror kept to one."""
import sys

from tools.dev.focus import chosen
from views_kit import Report

ROBOT, FLYERS = 'host/terminal/pages/humanoid.py', ('quad', 'helicopter', 'ornithopter', 'aerobat')


def test_each_page_comes_to_its_own(report):
    """The atlas's ties from symbol to symbol (tools/dev/atlas_calls.py) take a page's `run` to what it calls,
    none to the menu that starts every page: her pass's makers all the robot's, her law, stack and senses
    hers alone; the crafts' own the flyers' and theirs alone, none of them coming to her law
    (terminal/views/atlas/apps.py)."""
    from terminal.views.atlas import apps, scene
    from tools.dev import atlas_calls, atlas_flow, atlas_tree
    tree = atlas_tree.build()
    systems, served = apps.systems(tree), apps.served(tree)
    robot, flyers = tree.at[ROBOT], {tree.at['host/terminal/pages/%s.py' % f] for f in FLYERS}
    menu = any('host/terminal/menu.py::main' in atlas_calls.reach(tree, [tree.id[p] + '::run']) for p in systems)
    makers = {tree.up[n] for n in atlas_flow.makers(tree).values() if n >= 0}
    own = {scene.short(tree, f): {scene.short(tree, p) for p in ps} for f, ps in served.items()}
    hers = {f: own[f] for f in ('law/', 'stack/', 'senses/')}
    crafts = own.get('aircraft/', set())
    report.check("a page's run to what it calls, none to the menu; her pass's makers the robot's, her law, "
                 "stack and senses hers alone", sum(map(len, tree.calls.values())) > 10000 and not menu
                 and makers <= systems[robot] and all(ps == {'humanoid.py'} for ps in hers.values()),
                 '%d ties; menu reached %s; hers %s' % (sum(map(len, tree.calls.values())), menu, hers))
    report.check("the crafts' own the flyers' alone, none of them coming to her law",
                 crafts == {scene.short(tree, p) for p in flyers}
                 and not any(f in systems[p] for p in flyers for f, ps in served.items() if robot in ps and len(ps) == 1),
                 'aircraft/ %s' % sorted(crafts))


def test_the_atlas_by_application(report):
    """A lays the atlas by application (show_atlas.py, atlas/apps.py): the terminal out to its pages, the
    flyers side by side; each subsystem up at its layer's height, the more pages it serves the nearer the
    trunk, one's own as far out as any; a page's key lights its branch - it and its subsystems, the rest
    dim -, frames it and lists its own first; A again, the tree."""
    import math
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import apps, scene
    from tools.dev import atlas_tree
    tree = atlas_tree.build()
    state = show_atlas.opened(tree)
    layers = show_atlas.placed(state)['at']   # the tree's, its layers' heights
    bindings.act_on(show_atlas.PAD, ['a'], state)
    order, users = show_atlas.context(state), apps.served(tree)
    flown = sorted(order.index(tree.at['host/terminal/pages/%s.py' % f]) for f in FLYERS)
    shown = show_atlas.placed(state)
    out = {f: math.hypot(shown['at'][f][0], shown['at'][f][2]) for f in users if apps.layer(tree, f) < 9}
    high = {f: shown['at'][f][1] == layers[apps.layer(tree, f)][1] for f in users}
    by = sorted((len(users[f]), r) for f, r in out.items())
    nearer = all(r1 >= r2 - 2 * apps.APART for (n1, r1), (n2, r2) in zip(by, by[1:]) if n1 < n2)
    report.check('the pages round the terminal, the flyers side by side; each subsystem at its height, the more '
                 'it serves the nearer the trunk', len(order) == len(apps.pages(tree)) and flown[-1] - flown[0] == 3
                 and all(high.values()) and nearer, 'flyers at %s; %d subsystems' % (flown, len(users)))
    robot = tree.at[ROBOT]
    framed = show_atlas.frame(state, state['focus'])
    bindings.act_on(show_atlas.PAD, [show_atlas.keyed(state)[robot].lower()], state)
    lit, rings = show_atlas.placed(state)['flat']['lit'], show_atlas.placed(state)['rings'][robot]
    ink = dict(zip(show_atlas.placed(state)['flat']['nodes'].tolist(), show_atlas.placed(state)['flat']['ink'].tolist()))
    near, app = state['goal']['distance'], state['app']
    bindings.act_on(show_atlas.PAD, ['a'], state)
    report.check("a page's key lights its branch and frames it, its own first in its list; A again, the tree",
                 app == robot and lit == {robot} | {f for f, ps in users.items() if robot in ps}
                 and ink[robot] == scene.OPEN and all(ink[p] == scene.DIM_NODE for p in order if p != robot)
                 and [len(users[f]) for f in rings] == sorted(len(users[f]) for f in rings)
                 and near < framed[1] and show_atlas.context(state) == list(range(12)), '%d lit, %d listed' % (len(lit), len(rings)))


def test_the_mirror_keeps_to_a_page(report):
    """With a page chosen, M's mirror keeps to its system (atlas/mirror.py, atlas/nested.py): every flow
    between its files, a node opening to its parts alone - the robot's executive to her folders, no craft's
    -; the flyers' leaves her pass out."""
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import apps, nested, scene
    from tools.dev import atlas_tree
    tree = atlas_tree.build()
    state = show_atlas.opened(tree)
    robot = tree.at[ROBOT]
    bindings.act_on(show_atlas.PAD, ['f4', 'a'], state)
    bindings.act_on(show_atlas.PAD, [show_atlas.keyed(state)[robot].lower(), 'm'], state)
    over, files = show_atlas.placed(state), apps.systems(tree)[robot]
    opened = nested.opened(over, 6.0)
    parts = {p for inner, _near in opened for p in inner['own']}
    strays = [scene.short(tree, p) for p in parts if not any(p in scene.chain(tree, f) for f in files)]
    report.check("the mirror kept to the robot: every flow between her files, a node opening to her parts alone",
                 over['reach'] == files and all(a in files and b in files for a, b, _k, _l in over['flow'])
                 and opened and not strays and 'aircraft/' not in {scene.short(tree, n) for n in over['marks']},
                 '%d flows, %d parts open, strays %s' % (len(over['flow']), len(parts), strays[:4]))
    bindings.act_on(show_atlas.PAD, ['m'], state)
    bindings.act_on(show_atlas.PAD, [show_atlas.keyed(state)[tree.at['host/terminal/pages/quad.py']].lower(), 'm'], state)
    flown = show_atlas.placed(state)
    report.check("a flyer's mirror leaves her pass out", not any(k == 0 for _a, _b, k, _l in flown['flow'])
                 and flown['reach'] == apps.systems(tree)[tree.at['host/terminal/pages/quad.py']],
                 '%d flows' % len(flown['flow']))


ROSTER = (test_each_page_comes_to_its_own, test_the_atlas_by_application, test_the_mirror_keeps_to_a_page)


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
