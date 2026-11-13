"""The terminal's applications: each page's system by what it calls, their branches, the mirror kept to one."""
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


def down_to(state, page):
    """The terminal open on its applications (A where it is not), BKSP up its branches, then the key of each
    branch holding `page` and its own: the keys pressed."""
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import apps
    held, pressed = apps.clades(state['tree'])['pages'], []
    if not show_atlas._apping(state):
        bindings.act_on(show_atlas.PAD, ['a'], state)
    for _ in range(len(held) + 1):   # up its branches to the terminal
        if state['app'] is not None:
            bindings.act_on(show_atlas.PAD, ['\x08'], state)
    for _ in range(len(held) + 1):
        keys = show_atlas.keyed(state)
        key = keys.get(page) or next((keys[b] for b in keys if b < -1 and page in held[b]), None)
        if key is None:
            break
        pressed.append(key.lower())
        bindings.act_on(show_atlas.PAD, [key.lower()], state)
        if state['app'] == page:
            break
    return pressed


def test_the_terminal_opens_on_its_applications(report):
    """F1 at the trunk opens the terminal on its applications (show_atlas.py, atlas/apps.py): the trunk drawn
    and named as the tree has it, the terminal its root, out from it the branches the pages part along, each
    named by what its pages share alone - the flyers on aircraft/, side by side -, each subsystem up at its
    layer's height, the more pages it serves the nearer the trunk; the keys go down the branches, a page's
    lights it and its subsystems, frames them and offers them, its own first; BKSP climbs back, past the
    terminal to the trunk; A at the terminal its folders and back."""
    import math
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import apps, scene
    from tools.dev import atlas_tree
    tree = atlas_tree.build()
    state = show_atlas.opened(tree)
    layers = show_atlas.placed(state)['at']   # the tree's, its layers' heights
    bindings.act_on(show_atlas.PAD, ['f1'], state)
    shown, c, users, order = show_atlas.placed(state), apps.clades(tree), apps.served(tree), apps.ordered(tree)

    def seen(cam):
        """The scene from `cam` at 150 x 44: what it names, and the nodes drawn within it."""
        lines, fell, v = scene.picture(show_atlas.placed(state), cam, 150, 44, show_atlas.keyed(state), set())
        return '\n'.join(row[0] for row in lines), {n for n, (x, y, _d) in fell.items() if 0 <= x < v['w'] and 0 <= y < v['h']}

    def goal():
        """The camera the page eases to."""
        return scene.camera(target=state['goal']['target'], distance=state['goal']['distance'])
    plain, framed = seen(scene.camera())[0], dict(state['goal'])   # the tree's own view of the trunk
    named, (_said, fell) = [row[0] for row in atlas_tree.LAYERS if row[0] in plain], seen(goal())
    report.check('F1: the terminal open on its applications, framed on its branches, the trunk drawn and named, '
                 'the terminal its root', state['focus'] == 0 and show_atlas.titled(state, None, 100) == 'APPS'
                 and shown['focus'] == 0 and len(named) == len(atlas_tree.LAYERS) and shown['flat']['lit'] <= fell
                 and show_atlas.context(state) == c['kids'][None], '%d named: %s' % (len(named), named))
    flyers = frozenset(tree.at['host/terminal/pages/%s.py' % f] for f in FLYERS)
    flown = next((b for b, ps in c['pages'].items() if ps == flyers), None)
    runs = all(max(map(order.index, ps)) - min(map(order.index, ps)) == len(ps) - 1 for ps in c['pages'].values())
    report.check("out from it the branches the pages part along, each named by what they share alone, the flyers "
                 "on aircraft/", flown is not None and shown['names'][flown] == 'aircraft/' and runs
                 and sorted(order) == sorted(p for p, *_r in apps.pages(tree))
                 and set(c['up']) == set(order) | set(c['pages']) and all(b in fell for b in c['pages']),
                 ', '.join('%s %d' % (shown['names'][b], len(ps)) for b, ps in c['pages'].items()))
    out = {f: math.hypot(shown['at'][f][0], shown['at'][f][2]) for f in users if apps.layer(tree, f) < 9}
    by = sorted((len(users[f]), r) for f, r in out.items())
    nearer = all(r1 >= r2 - 2 * apps.APART for (n1, r1), (n2, r2) in zip(by, by[1:]) if n1 < n2)
    report.check('each subsystem at its height, the more it serves the nearer the trunk',
                 all(shown['at'][f][1] == layers[apps.layer(tree, f)][1] for f in users) and nearer,
                 '%d subsystems' % len(users))
    robot = tree.at[ROBOT]
    pressed = down_to(state, robot)
    path, node = [], robot
    while node is not None:
        path, node = [node] + path, c['up'][node]
    laid = show_atlas.placed(state)['flat']
    lit, rings, ink = laid['lit'], show_atlas.context(state), dict(zip(laid['nodes'].tolist(), laid['ink'].tolist()))
    mine, drawn = {f for f, ps in users.items() if robot in ps}, seen(goal())[1]
    report.check("the keys down the branches to a page; its key lights it and its subsystems, frames them and "
                 "offers them, its own first", state['app'] == robot and len(pressed) == len(path)
                 and lit == {0, *path} | mine and ink[robot] == scene.OPEN
                 and all(ink[p] == scene.DIM_NODE for p in order if p != robot) and set(rings) == mine
                 and [len(users[f]) for f in rings] == sorted(len(users[f]) for f in rings)
                 and lit <= drawn and state['goal'] != framed,
                 '%s: %d lit, %d drawn, %d offered' % (pressed, len(lit), len(lit & drawn), len(rings)))
    climbed = []
    for _ in path:
        bindings.act_on(show_atlas.PAD, ['\x08'], state)
        climbed.append(state['app'])
    bindings.act_on(show_atlas.PAD, ['\x08'], state)
    trunk = state['focus']
    bindings.act_on(show_atlas.PAD, ['a'], state)
    there = state['focus'], show_atlas.context(state) == c['kids'][None]
    bindings.act_on(show_atlas.PAD, ['a'], state)
    folders = show_atlas.context(state) == scene.children(tree, state['kids'], state['below'], 0)
    bindings.act_on(show_atlas.PAD, ['a'], state)
    report.check('BKSP climbs the branches, past the terminal to the trunk; A the terminal on its applications, '
                 'again its folders, and back', climbed == path[-2::-1] + [None] and trunk == -1
                 and there == (0, True) and folders and 'names' in show_atlas.placed(state),
                 '%s, trunk %d' % (climbed, trunk))


def test_the_mirror_keeps_to_a_page(report):
    """With a page chosen, M's mirror keeps to its system (atlas/mirror.py, atlas/nested.py): every flow
    between its files, a node opening to its parts alone - the robot's executive to her folders, no craft's
    -; the flyers' leaves her pass out; a branch's keeps to its pages'."""
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import apps, nested, scene
    from tools.dev import atlas_tree
    tree = atlas_tree.build()
    state = show_atlas.opened(tree)
    robot, quad = tree.at[ROBOT], tree.at['host/terminal/pages/quad.py']
    down_to(state, robot)
    bindings.act_on(show_atlas.PAD, ['a', '\x08', 'f4', 'm'], state)   # its folders, the trunk, the executive
    over, files = show_atlas.placed(state), apps.systems(tree)[robot]
    opened = nested.opened(over, 6.0)
    parts = {p for inner, _near in opened for p in inner['own']}
    strays = [scene.short(tree, p) for p in parts if not any(p in scene.chain(tree, f) for f in files)]
    report.check("the mirror kept to the robot: every flow between her files, a node opening to her parts alone",
                 over['reach'] == files and all(a in files and b in files for a, b, _k, _l in over['flow'])
                 and opened and not strays and 'aircraft/' not in {scene.short(tree, n) for n in over['marks']},
                 '%d flows, %d parts open, strays %s' % (len(over['flow']), len(parts), strays[:4]))
    bindings.act_on(show_atlas.PAD, ['m'], state)
    down_to(state, quad)
    bindings.act_on(show_atlas.PAD, ['a', '\x08', 'f4', 'm'], state)
    flown, held = show_atlas.placed(state), apps.clades(tree)['pages']
    craft = next((b for b, ps in held.items() if quad in ps and len(ps) == len(FLYERS)), None)
    report.check("a flyer's mirror leaves her pass out; a branch's keeps to its pages'",
                 not any(k == 0 for _a, _b, k, _l in flown['flow']) and flown['reach'] == apps.systems(tree)[quad]
                 and craft is not None
                 and apps.reach(tree, craft) == frozenset().union(*(apps.systems(tree)[p] for p in held[craft])),
                 '%d flows' % len(flown['flow']))


ROSTER = (test_each_page_comes_to_its_own, test_the_terminal_opens_on_its_applications,
          test_the_mirror_keeps_to_a_page)


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
