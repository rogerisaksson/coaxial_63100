"""The atlas by application: pages' systems, their tree, a dataflow a system, frames centred."""
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



def down_to(state, node):
    """The applications' tree (A where it is not shown), BKSP up its branches, then the key of each branch
    holding `node` - a page, or a branch - and its own: the keys pressed."""
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import apps
    held, pressed = apps.clades(state['tree'])['pages'], []
    mine = held.get(node, {node})
    if not show_atlas._apping(state):
        bindings.act_on(show_atlas.PAD, ['a'], state)
    for _ in range(len(held) + 1):   # up its branches to the terminal
        if state['app'] is not None:
            bindings.act_on(show_atlas.PAD, ['\x08'], state)
    for _ in range(len(held) + 1):
        keys = show_atlas.keyed(state)
        key = keys.get(node) or next((keys[b] for b in keys if b < -1 and mine <= held[b]), None)
        if key is None:
            break
        pressed.append(key.lower())
        bindings.act_on(show_atlas.PAD, [key.lower()], state)
        if state['app'] == node:
            break
    return pressed


def test_the_trunk_is_the_applications_tree(report):
    """The atlas opens on the applications' tree (show_atlas.py, atlas/apps.py): the terminal at its foot, the
    trunk up from it carrying what every page shares, at its top the crown, each branch carrying what its pages
    share alone - the flyers' on aircraft/ -, a page's own on its twig, the page at its tip; no pillar of layers;
    all in the plane the camera sees square on, from every side, no branch crossing another; the folders grown
    in a zoom key's step nearer than the frame; the keys down the branches, a page's lighting its way and its
    folders; BKSP back to the terminal; A the layers and back."""
    import itertools
    import math
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import apps, scene
    from tools.dev import atlas_tree
    tree = atlas_tree.build()
    state = show_atlas.opened(tree)
    c, users, order = apps.clades(tree), apps.served(tree), apps.ordered(tree)
    own, pol, laid = apps.owners(tree), apps.crown(tree), show_atlas.placed(state)
    nodes = set(laid['flat']['nodes'].tolist())
    flyers = frozenset(tree.at['host/terminal/pages/%s.py' % f] for f in FLYERS)
    flown = next((b for b, ps in c['pages'].items() if ps == flyers), None)
    theirs = {f for f, ps in users.items() if ps == flyers}
    report.check("the atlas opens on the applications' tree: the terminal its root, no pillar of layers, its keys "
                 "the branches out of the trunk, the flyers on aircraft/ with what they share alone",
                 show_atlas.titled(state, None, 100) == 'APPS' and laid['focus'] == 0
                 and nodes & set(range(len(atlas_tree.LAYERS))) == {0} and show_atlas.context(state) == c['kids'][None]
                 and flown is not None and laid['names'][flown] == 'aircraft/' and theirs
                 and all(own[f] == flown for f in theirs), '%d nodes, aircraft/ holds %d' % (len(nodes), len(theirs)))

    def side(a, b, p):
        return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])

    def seen(turn, pitch):
        """The tree turned `turn` degrees from the opening view and tipped to `pitch`: (how far any node stands off
        where the tree lays it - the trunk's on the line from the terminal to its top, a branch's own along its
        curve -, how far off the plane the camera sees square on, branch pairs crossing on screen)."""
        state['cam'].update(yaw=scene.camera()['yaw'] + turn, pitch=pitch)
        laid = show_atlas.placed(state)
        sight, hub, base = scene.view(dict(state['cam']), 150, 44), laid['hub'], laid['at'][0]

        def on(origin, r, a):
            return tuple(origin[k] + r * (math.cos(a) * sight['r'][k] + math.sin(a) * sight['u'][k]) for k in range(3))

        def hung(n):
            return sorted((f for f in users if own[f] == n), key=lambda f: (apps.layer(tree, f), f))
        trunk, off = hung(0), max(math.dist(laid['at'][n], on(hub, *ra)) for n, ra in pol.items())
        for j, f in enumerate(trunk):
            off = max(off, math.dist(laid['at'][f], on(base, math.dist(base, hub) * (j + 1) / (len(trunk) + 1), math.pi / 2)))
        for n, ra in pol.items():
            start, mine = (0.0, ra[1]) if c['up'][n] is None else pol[c['up'][n]], hung(n)
            off = max([off] + [math.dist(laid['at'][f], on(hub, *apps.along(start, ra, (j + 1) / (len(mine) + 1))))
                               for j, f in enumerate(mine)])
        square = max(abs(sum((p[k] - base[k]) * sight['f'][k] for k in range(3))) for p in laid['at'].values())

        def dots(p):
            x, y, z = (sum((p[k] - sight['eye'][k]) * sight[a][k] for k in range(3)) for a in ('r', 'u', 'f'))
            return x / z, -y / z
        ways = {n: [dots(on(hub, *q)) for q in apps.curve((0.0, ra[1]) if c['up'][n] is None else pol[c['up'][n]], ra)]
                for n, ra in pol.items()}
        crossed = sum(1 for a, b in itertools.combinations(ways, 2) if len({a, b, c['up'][a], c['up'][b]}) == 4
                      and any(side(p, q, r) * side(p, q, s) < 0 and side(r, s, p) * side(r, s, q) < 0
                              for p, q in zip(ways[a], ways[a][1:]) for r, s in zip(ways[b], ways[b][1:])))
        return off, square, crossed
    views = {(t, p): seen(t, p) for t, p in ((0, 18.0), (110, 18.0), (180, 18.0), (60, -30.0), (250, 60.0))}
    state['cam'].update(yaw=scene.camera()['yaw'], pitch=scene.camera()['pitch'])
    report.check('the trunk carries what every page shares, each branch what its pages share alone along it, a '
                 "page's own on its twig - all in the plane the camera sees square on from every side, no branch "
                 'crossing another', all(off < 1e-9 and square < 1e-9 and not crossed for off, square, crossed in views.values()),
                 '; '.join('%+d/%+d: %.0e off, %.0e out of the plane, %d crossed' % ((t, p) + v) for (t, p), v in views.items()))

    def drawn():
        """The folders drawn now."""
        return {n for n in show_atlas.placed(state)['flat']['nodes'].tolist() if n in users}
    state['cam'].update(target=state['goal']['target'], distance=state['goal']['distance'])   # eased to its frame
    far = drawn()
    bindings.act_on(show_atlas.PAD, ['+'], state)
    near = drawn()
    robot = tree.at[ROBOT]
    pressed = down_to(state, robot)
    state['cam'].update(target=state['goal']['target'], distance=state['goal']['distance'])
    path, node = [], robot
    while node is not None:
        path, node = [node] + path, c['up'][node]
    laid, mine = show_atlas.placed(state), {f for f, ps in users.items() if robot in ps}
    ink = dict(zip(laid['flat']['nodes'].tolist(), laid['flat']['ink'].tolist()))
    report.check("the folders grown in a zoom key's step nearer than the frame; the keys down the branches to a "
                 "page, its key lighting its way from the terminal and its folders, drawn and named, offering them, "
                 "its own first", not far and near == set(users) and state['app'] == robot and len(pressed) == len(path)
                 and laid['flat']['lit'] == {0, *path} | mine and drawn() == mine and ink[robot] == scene.OPEN
                 and all(ink[p] == scene.DIM_NODE for p in order if p != robot) and mine <= set(laid['groups'][1])
                 and show_atlas.context(state) == apps.offered(tree, robot),
                 '%d folders far, %d a step in; %s: %d lit' % (len(far), len(near), pressed, len(laid['flat']['lit'])))
    climbed = []
    for _ in path:
        bindings.act_on(show_atlas.PAD, ['\x08'], state)
        climbed.append(state['app'])
    bindings.act_on(show_atlas.PAD, ['\x08'], state)
    top = state['focus'], state['app']
    bindings.act_on(show_atlas.PAD, ['a'], state)
    layers = show_atlas.context(state) == list(range(len(atlas_tree.LAYERS))) and 'names' not in show_atlas.placed(state)
    bindings.act_on(show_atlas.PAD, ['a'], state)
    report.check('BKSP climbs the branches back to the terminal and stays there; A the layers and their folders, '
                 "and back to the applications' tree", climbed == path[-2::-1] + [None] and top == (-1, None)
                 and layers and show_atlas.context(state) == c['kids'][None], '%s, then %s' % (climbed, top))


def test_the_dataflow_draws_one_system(report):
    """D turns the page to one system's dataflow (show_atlas.py, atlas/apps.py, atlas/mirror.py), titled DATAFLOW
    and the system: with none chosen the robot's - her pass and what comes back of it, the wire -, every flow
    between its files, a node opening to its parts alone, the robot's executive to her folders, no craft's; D
    again the next page's, the quad's - no pass of hers, what its functions call of each other's -; a branch
    chosen, its pages' together; M back to the tree."""
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import apps, nested, scene
    from tools.dev import atlas_flow, atlas_tree
    tree = atlas_tree.build()
    state = show_atlas.opened(tree)
    robot, quad = tree.at[ROBOT], tree.at['host/terminal/pages/quad.py']
    passed, call = atlas_flow.KINDS.index('pass'), atlas_flow.KINDS.index('call')
    bindings.act_on(show_atlas.PAD, ['a', 'f4', 'd'], state)   # the layers, the executive, its dataflow
    over, files = show_atlas.placed(state), apps.systems(tree)[robot]
    opened = nested.opened(over, 6.0)
    parts = {p for inner, _near in opened for p in inner['own']}
    strays = [scene.short(tree, p) for p in parts if not any(p in scene.chain(tree, f) for f in files)]
    report.check("D the robot's dataflow where none is chosen: her pass, every flow between her files, a node "
                 'opening to her parts alone', state['mirror'] and show_atlas.titled(state, None, 100) == 'DATAFLOW HUMANOID'
                 and over['reach'] == files and all(a in files and b in files for a, b, _k, _l in over['flow'])
                 and any(k == passed for _a, _b, k, _l in over['flow']) and opened and not strays
                 and 'aircraft/' not in {scene.short(tree, n) for n in over['marks']},
                 '%d flows, %d parts open, strays %s' % (len(over['flow']), len(parts), strays[:4]))
    bindings.act_on(show_atlas.PAD, ['d'], state)
    flown, theirs = show_atlas.placed(state), apps.systems(tree)[quad]
    report.check("D again the next page's: the quad's, what its functions call of each other's, none of her pass",
                 show_atlas.titled(state, None, 100) == 'DATAFLOW QUAD ACRO FLYING' and flown['reach'] == theirs
                 and all(a in theirs and b in theirs for a, b, _k, _l in flown['flow'])
                 and not any(k == passed for _a, _b, k, _l in flown['flow']) and any(k == call for _a, _b, k, _l in flown['flow']),
                 '%d flows' % len(flown['flow']))
    bindings.act_on(show_atlas.PAD, ['m'], state)
    back = not state['mirror']
    held = apps.clades(tree)['pages']
    craft = next((b for b, ps in held.items() if quad in ps and len(ps) == len(FLYERS)), None)
    down_to(state, craft)
    bindings.act_on(show_atlas.PAD, ['d'], state)
    together = frozenset().union(*(apps.systems(tree)[p] for p in held.get(craft, ())))
    report.check("M back to the tree; a branch chosen, its pages' dataflow together", back and state['app'] == craft
                 and show_atlas.placed(state)['reach'] == together
                 and show_atlas.titled(state, None, 100) == 'DATAFLOW aircraft/', show_atlas.titled(state, None, 100))


def test_each_frame_stands_in_the_middle(report):
    """A node opened stands in the middle of the view (atlas/aim.py): the camera's target worked out so the
    projection of what its frame frames - in the layers the node and all laid out under it, in the applications'
    tree what stands drawn and lit at the frame - has its middle on the drawing's middle, as the view is turned,
    not guessed in the world."""
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import scene
    from tools.dev import atlas_tree
    tree, off = atlas_tree.build(), {}
    for label, keys, turn in (('a layer', ['a', 'f4'], 0), ('a layer turned', ['a', 'f4'], 60),
                              ('a folder', ['a', 'f4', 'f1'], 0), ('a file', ['a', 'f4', 'f1', 'f1'], 0),
                              ("the applications' tree", [], 0), ('the robot', ['f5', 'f3', 'f2', 'f2', 'f1'], 0),
                              ('the flyers', ['f5', 'f3', 'f2', 'f2', 'f2'], 0)):
        state = show_atlas.opened(tree)
        state['size'], state['cam']['yaw'] = (150, 44), state['cam']['yaw'] + turn
        bindings.act_on(show_atlas.PAD, keys or ['home'], state)
        state['cam'].update(target=state['goal']['target'], distance=state['goal']['distance'])   # eased to it
        laid, focus = show_atlas.placed(state), state['focus']
        flat, sight = scene.flat(laid), scene.view(state['cam'], *show_atlas.sized(state))
        framed = [p for n, p in zip(flat['nodes'].tolist(), flat['pos'].tolist()) if n in flat['lit']] \
            if show_atlas._apping(state) else [p for n, p in laid['at'].items() if focus in scene.chain(tree, n)]
        seen = [tuple(sum((p[k] - sight['eye'][k]) * sight[a][k] for k in range(3)) for a in ('r', 'u', 'f'))
                for p in framed]
        xs, ys = [x / z for x, _y, z in seen], [y / z for _x, y, z in seen]
        off[label] = sight['focal'] * max(abs(max(xs) + min(xs)), abs(max(ys) + min(ys))) / 2   # dots
    report.check("each node opened framed with its projection's middle on the drawing's middle - a layer, turned "
                 "too, a folder, a file, the applications' tree, a page, a branch", all(d < 1e-9 for d in off.values()),
                 ', '.join('%s %.1e dots off' % kv for kv in off.items()))


ROSTER = (test_each_page_comes_to_its_own, test_the_trunk_is_the_applications_tree, test_the_dataflow_draws_one_system,
          test_each_frame_stands_in_the_middle)


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
