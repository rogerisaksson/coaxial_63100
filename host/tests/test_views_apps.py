"""The atlas by application: pages' systems, their tree grown, keys in view, a dataflow a system, frames."""
import sys

from tools.dev.focus import chosen
from views_kit import Report

ROBOT, FLYERS = 'host/terminal/pages/humanoid.py', ('quad', 'helicopter', 'ornithopter', 'aerobat')


def page_of(tree, name):
    """A page's node by its module's name."""
    return tree.at['host/terminal/pages/%s.py' % name]


def test_each_page_comes_to_its_own(report):
    """The atlas's ties from symbol to symbol (tools/dev/atlas_calls.py) take a page's `run` to what it calls,
    none to the menu that starts every page nor to what a module imported runs as a script: her pass's makers all
    the robot's, her law, stack and senses hers alone; the crafts' own the flyers' and theirs alone, none of them
    coming to her law; each craft's loop - the tick its view gives its runner, the sky and law it steps -
    in its system (terminal/views/atlas/apps.py, tools/dev/atlas_flow.py)."""
    import ast
    from terminal.views.atlas import apps, scene
    from tools.dev import atlas_calls, atlas_flow, atlas_tree
    tree = atlas_tree.build()
    systems, served, reached = apps.systems(tree), apps.served(tree), apps.reached(tree)
    robot, flyers = tree.at[ROBOT], {page_of(tree, f) for f in FLYERS}
    menu = any('host/terminal/menu.py::main' in keys for keys in reached.values())
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
    rows = dict(atlas_calls.named(ast.parse("import x\nx.y = 1\ndef main():\n    x.go()\nif __name__ == '__main__':\n"
                                            "    main()\nelse:\n    x.seen()\n")))
    scripts = [k for keys in reached.values() for k in keys if k.endswith('::' + atlas_calls.SCRIPT)]
    quad_main = sorted(scene.short(tree, p) for p in flyers if 'host/terminal/views/show_quad.py::main' in reached[p])
    report.check("a module imported runs its own code, a script's guard not - its else does -, the guard a row "
                 "of its own: no page comes to what another page's view runs as a script",
                 rows[''] == ['x', 'x.seen', 'x.y'] and rows[atlas_calls.SCRIPT] == ['main'] and not scripts
                 and quad_main == ['quad.py'], 'rows %s; the quad view\'s main reached by %s' % (rows, quad_main))
    ticks, flies = atlas_flow.ticks(tree), atlas_flow.flying(tree)
    loops = {}
    for f in FLYERS:
        page = page_of(tree, f)
        flown = {t for k in reached[page] for t in flies.get(k, ())}
        loops[f] = (sorted(t.split('/')[-2] for t in flown),
                    {n for t in flown for a, b, _k, _l in ticks[t] for n in (a, b)} <= systems[page])
    sky = tree.at['host/machine/aircraft/helicopter/sky.py']
    report.check("each craft flies the tick its own view gives its runner, its loop's files - the sky and law it "
                 "steps - in its system", all(names == [f] and held for f, (names, held) in loops.items())
                 and sky in systems[page_of(tree, 'helicopter')], str(loops))


def eased(state):
    """The camera at its goal, as the page eases it there."""
    state['cam'].update(target=state['goal']['target'], distance=state['goal']['distance'])


def down_to(state, node):
    """The applications' tree (A where it is not shown), BKSP up to the terminal; the key of `node` - a page - or
    of a page under it - a branch -, a page of keys on (PGDN) where none holds one, then BKSP up to it; the
    camera eased to each frame: the keys pressed."""
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import apps
    held, pressed = apps.clades(state['tree'])['pages'], []
    mine = held.get(node, {node})
    if not show_atlas._apping(state):
        bindings.act_on(show_atlas.PAD, ['a'], state)
    while state['app'] is not None:
        bindings.act_on(show_atlas.PAD, ['\x08'], state)
    show_atlas.focused(state, -1)
    for _ in range(len(held) + 2):
        eased(state)
        keys = show_atlas.keyed(state)
        page = next((p for p in sorted(keys) if p in mine), None)
        if page is not None:
            pressed.append(keys[page].lower())
            bindings.act_on(show_atlas.PAD, [keys[page].lower()], state)
            break
        was = state['page']
        bindings.act_on(show_atlas.PAD, ['pagedown'], state)
        show_atlas.keyed(state)
        if state['page'] == was:
            break
    while state['app'] not in (node, None):
        pressed.append('bksp')
        bindings.act_on(show_atlas.PAD, ['\x08'], state)
    eased(state)
    return pressed


def test_the_tree_grows_as_a_tree(report):
    """The atlas opens on the applications' tree (show_atlas.py, atlas/apps.py) framed whole: the terminal at its
    foot, the trunk straight up from it carrying what every page shares, each branch what its pages share alone -
    the flyers' on aircraft/ -, a page's own on its twig, the page at its tip; no pillar of layers; grown as a tree
    - each bough out of its stem at its step, along the stem's way there, the leader straight on, each other with
    its chord at Murray's angle, round the stem by the golden angle from the last -, in the world: three-
    dimensional, the same from every side. Its keys the nodes named in view, the nearest the middle first, in
    reading order, PGDN the next, a turn of the view taking them anew; the folders grown in a zoom key's step
    nearer than the frame; a page's key lighting its way, its folders and the forks on its way named; BKSP back to
    the terminal; A the layers and back."""
    import math
    from coaxial.model.blocks import numpy as np
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import apps, scene
    from tools.dev import atlas_tree
    tree = atlas_tree.build()
    state = show_atlas.opened(tree)
    state['size'] = (150, 44)
    show_atlas.focused(state, -1)
    eased(state)
    c, users, order, own = apps.clades(tree), apps.served(tree), apps.ordered(tree), apps.owners(tree)
    laid = show_atlas.placed(state)
    nodes = set(laid['flat']['nodes'].tolist())
    flyers = frozenset(page_of(tree, f) for f in FLYERS)
    flown = next((b for b, ps in c['pages'].items() if ps == flyers), None)
    theirs = {f for f, ps in users.items() if ps == flyers}

    def projected():
        """{node: (across, down, its distance from the middle)} in dots, of those drawn within the drawing."""
        sight, flat, out = scene.view(state['cam'], *show_atlas.sized(state)), show_atlas.placed(state)['flat'], {}
        for n, p in zip(flat['nodes'].tolist(), flat['pos'].tolist()):
            x, y, z = (sum((p[k] - sight['eye'][k]) * sight[a][k] for k in range(3)) for a in ('r', 'u', 'f'))
            a, d = sight['w'] / 2 + x * sight['focal'] / z, sight['h'] / 2 - y * sight['focal'] / z
            if z > scene.NEAR and 0 <= a < sight['w'] and 0 <= d < sight['h']:
                out[n] = (a, d, math.hypot(a - sight['w'] / 2, d - sight['h'] / 2))
        return out
    inside = projected()
    report.check("the atlas opens on the applications' tree framed whole: the terminal its root, no pillar of "
                 "layers, the flyers on aircraft/ with what they share alone",
                 show_atlas.titled(state, None, 100) == 'APPS' and laid['focus'] == 0
                 and nodes & set(range(len(atlas_tree.LAYERS))) == {0} and laid['flat']['lit'] <= set(inside)
                 and flown is not None and laid['names'][flown] == 'aircraft/' and theirs
                 and all(own[f] == flown for f in theirs), '%d nodes, aircraft/ holds %d' % (len(nodes), len(theirs)))

    grown, base, off = apps.boughs(tree), laid['at'][0], {'step': 0.0, 'murray': 0.0, 'golden': 0.0, 'laid': 0.0}

    def held(n):
        return len(c['pages'].get(n, (n,)))
    for fork in [None, *sorted(c['pages'])]:
        kids, stem, steps = c['kids'][fork], grown[0 if fork is None else fork], apps.slots(tree, 0 if fork is None else fork)
        branches = [k for k in kids if k < -1]
        leader = max(branches, key=lambda k: (held(k), -k)) if branches else None
        order_ = sorted((k for k in kids if k != leader), key=lambda k: (-held(k), k)) + [leader] * bool(branches)
        above, last = held(fork) if fork else len(order), None
        for j, k in enumerate(order_, steps - len(kids) + 1):
            at, way = apps.along(stem, j / steps)
            off['step'] = max(off['step'], math.dist(at, grown[k][0]), math.dist(way, grown[k][1]))
            chord = np.subtract(apps.along(grown[k], 1.0)[0], grown[k][0])
            chord /= np.linalg.norm(chord)
            angle = math.acos(min(1.0, float(chord @ np.array(way))))
            off['murray'] = max(off['murray'], abs(angle - apps.murray(above, held(k))), angle * (k == leader))
            across = chord - (chord @ np.array(way)) * np.array(way)
            if np.linalg.norm(across) > 1e-6:   # round the stem from the last by the golden angle
                across /= np.linalg.norm(across)
                if last is not None:
                    off['golden'] = max(off['golden'], abs(float(last @ across) - math.cos(scene.GOLDEN)))
                last = across
            above -= held(k)
    for n, b in grown.items():   # a page or a branch at its bough's end, each folder at its step along it
        mine = sorted((f for f in users if own[f] == n), key=lambda f: (apps.layer(tree, f), f))
        spots = [(f, (j + 1) / apps.slots(tree, n)) for j, f in enumerate(mine)] + ([(n, 1.0)] if n else [])
        for m, t in spots:
            off['laid'] = max(off['laid'], math.dist(laid['at'][m], np.add(base, apps.along(b, t)[0])))
    upright = grown[0][1] == (0.0, 1.0, 0.0) and grown[0][3] == 0.0
    report.check("grown as a tree: the trunk upright; each bough out of its stem at its step along its way, the "
                 "leader straight on, each other's chord at Murray's angle, round the stem by the golden angle "
                 "from the last; a page at its twig's tip, each folder at its step on its bough",
                 upright and all(v < 1e-9 for v in off.values()), ', '.join('%s %.1e' % kv for kv in off.items()))

    def turned(yaw, pitch):
        state['cam'].update(yaw=yaw, pitch=pitch)
        return show_atlas.placed(state)['at']
    home = dict(laid['at'])
    views = [turned(scene.camera()['yaw'] + t, p) for t, p in ((110, 18.0), (180, 18.0), (60, -30.0), (250, 60.0))]
    state['cam'].update(yaw=scene.camera()['yaw'], pitch=scene.camera()['pitch'])
    moved = max(math.dist(v[n], home[n]) for v in views for n in home)
    spread = np.linalg.svd(np.array(list(home.values())) - np.mean(list(home.values()), 0), compute_uv=False)
    report.check('in the world, the same from every side, three-dimensional: its thinnest spread a tenth of its '
                 'widest at least', moved < 1e-12 and spread[-1] >= spread[0] / 10,
                 '%.0e moved; spread %s' % (moved, np.round(spread, 2).tolist()))

    def keys_check():
        """(keyed in view and named, the nearest the middle, in reading order): of the keys now."""
        keyed, at = show_atlas.keyed(state), projected()
        named = set().union(*show_atlas.placed(state)['groups'])
        ranked = sorted(keyed, key=lambda n: int(keyed[n][1:]))
        rest = [at[n][2] for n in at if n in named and n not in keyed]
        return (bool(keyed) and all(n in at and n in named for n in keyed),
                not rest or max(at[n][2] for n in keyed) <= min(rest) + 1e-9,
                [(int(at[n][1] // 4), at[n][0]) for n in ranked] == sorted((int(at[n][1] // 4), at[n][0]) for n in ranked),
                keyed)
    shown, nearest, reading, first = keys_check()
    bindings.act_on(show_atlas.PAD, ['pagedown'], state)
    later = show_atlas.keyed(state)
    at = projected()
    beyond = bool(later) and not set(later) & set(first) and min(at[n][2] for n in later) >= max(at[n][2] for n in first) - 1e-9
    bindings.act_on(show_atlas.PAD, ['pageup'], state)
    state['cam']['yaw'] += 120.0
    turned_shown, turned_nearest, turned_reading, again = keys_check()
    state['cam']['yaw'] -= 120.0
    report.check('the keys the nodes named in view, the nearest the middle first, in reading order; PGDN the next '
                 'ones; a turn of the view takes them anew',
                 shown and nearest and reading and beyond and turned_shown and turned_nearest and turned_reading
                 and again != first, '%d keyed, %d more on PGDN, turned %d' % (len(first), len(later), len(again)))

    def drawn():
        """The folders drawn now."""
        return {n for n in show_atlas.placed(state)['flat']['nodes'].tolist() if n in users}
    far = drawn()
    bindings.act_on(show_atlas.PAD, ['+'], state)
    near = drawn()
    robot = tree.at[ROBOT]
    pressed = down_to(state, robot)
    path, node = [], robot
    while node is not None:
        path, node = [node] + path, c['up'][node]
    laid, mine = show_atlas.placed(state), {f for f, ps in users.items() if robot in ps}
    ink = dict(zip(laid['flat']['nodes'].tolist(), laid['flat']['ink'].tolist()))
    forks = {n for g in laid['groups'] for n in g if n in c['pages']}
    report.check("the folders grown in a zoom key's step nearer than the frame; a page's key lighting its way from "
                 "the terminal and its folders, drawn and named, the forks on its way named, no other",
                 not far and near == set(users) and state['app'] == robot and pressed
                 and laid['flat']['lit'] == {0, *path} | mine and drawn() == mine and ink[robot] == scene.OPEN
                 and all(ink[p] == scene.DIM_NODE for p in order if p != robot) and mine <= set(laid['groups'][0])
                 and forks == set(path) - {robot}, '%d folders far, %d a step in; %s: %d lit, forks named %s'
                 % (len(far), len(near), pressed, len(laid['flat']['lit']), sorted(forks)))
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
                 "and back to the applications' tree, its keys in view", climbed == path[-2::-1] + [None]
                 and top == (-1, None) and layers and show_atlas._apping(state) and keys_check()[0],
                 '%s, then %s' % (climbed, top))


def test_the_dataflow_draws_one_system(report):
    """D turns the page to one system's dataflow (show_atlas.py, atlas/apps.py, atlas/mirror.py), titled DATAFLOW
    and the system, each its own loop the same way: with none chosen the robot's - her pass and what comes back of
    it, the wire -, every flow between its files, a node opening to its parts alone, the robot's executive to her
    folders, no craft's; D again the next page's, each craft's the tick its view flies - the aerobat's on its own
    sky and law, handed on to the helicopter's tick -, none of her pass; a branch chosen, its pages' together;
    M back to the tree."""
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import apps, nested, scene
    from tools.dev import atlas_flow, atlas_tree
    tree = atlas_tree.build()
    state = show_atlas.opened(tree)
    robot = tree.at[ROBOT]
    passed = atlas_flow.KINDS.index('pass')
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
    streams = {f for f in atlas_flow.flows(tree) if f[2] == passed}
    ticks, seen = atlas_flow.ticks(tree), {}
    for _ in FLYERS:
        bindings.act_on(show_atlas.PAD, ['d'], state)
        title, flow = show_atlas.titled(state, None, 100), show_atlas.placed(state)['flow']
        page = next(page_of(tree, f) for f in FLYERS if apps.named(tree, page_of(tree, f)) in title)
        name = tree.id[page].split('/')[-1][:-3]
        loop = ticks['host/terminal/views/%s/flight.py::flown' % name]
        theirs = apps.systems(tree)[page]
        seen[name] = (set(loop) <= set(flow) and not streams & set(flow) and show_atlas.placed(state)['reach'] == theirs
                      and all(a in theirs and b in theirs for a, b, _k, _l in flow), len(flow))
    sky = {tree.id[n] for a, b, _k, _l in ticks['host/terminal/views/aerobat/flight.py::flown'] for n in (a, b)}
    report.check("D again each craft's: the loop of the tick its view flies, none of her pass, every flow between "
                 "its files - the aerobat's on its own sky", sorted(seen) == sorted(FLYERS) and all(ok for ok, _n in seen.values())
                 and 'host/machine/aircraft/aerobat/sky.py' in sky and 'host/machine/aircraft/helicopter/sky.py' not in sky,
                 str(seen))
    bindings.act_on(show_atlas.PAD, ['m'], state)
    back = not state['mirror']
    held = apps.clades(tree)['pages']
    quad = page_of(tree, 'quad')
    craft = next((b for b, ps in held.items() if quad in ps and len(ps) == len(FLYERS)), None)
    down_to(state, craft)
    bindings.act_on(show_atlas.PAD, ['d'], state)
    together = frozenset().union(*(apps.systems(tree)[p] for p in held.get(craft, ())))
    flow = set(show_atlas.placed(state)['flow'])
    loops = all(set(ticks['host/terminal/views/%s/flight.py::flown' % f]) <= flow for f in FLYERS)
    report.check("M back to the tree; a branch chosen, its pages' dataflow together, each one's loop",
                 back and state['app'] == craft and show_atlas.placed(state)['reach'] == together and loops
                 and show_atlas.titled(state, None, 100) == 'DATAFLOW aircraft/', show_atlas.titled(state, None, 100))


def test_each_frame_stands_in_the_middle(report):
    """A node opened stands in the middle of the view (atlas/aim.py): the camera's target worked out so the
    projection of what its frame frames - in the layers the node and all laid out under it, in the applications'
    tree what stands drawn and lit at the frame - has its middle on the drawing's middle, as the view is turned,
    not guessed in the world."""
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import apps, scene
    from tools.dev import atlas_tree
    tree, off = atlas_tree.build(), {}
    held = apps.clades(tree)['pages']
    flyers = next(b for b, ps in held.items() if ps == frozenset(page_of(tree, f) for f in FLYERS))
    for label, keys, turn in (('a layer', ['a', 'f4'], 0), ('a layer turned', ['a', 'f4'], 60),
                              ('a folder', ['a', 'f4', 'f1'], 0), ('a file', ['a', 'f4', 'f1', 'f1'], 0),
                              ("the applications' tree", [], 0), ('the robot', tree.at[ROBOT], 0),
                              ('the flyers', flyers, 0)):
        state = show_atlas.opened(tree)
        state['size'], state['cam']['yaw'] = (150, 44), state['cam']['yaw'] + turn
        if isinstance(keys, int):
            down_to(state, keys)
        else:
            bindings.act_on(show_atlas.PAD, keys or ['home'], state)
        eased(state)
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


ROSTER = (test_each_page_comes_to_its_own, test_the_tree_grows_as_a_tree, test_the_dataflow_draws_one_system,
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
