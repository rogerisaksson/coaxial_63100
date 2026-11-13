"""The applications' tree grown down: a node opened in it grows its subtree as the tree grows, framed."""
import functools
import sys

from test_views_apps import FLYERS, ROBOT, down_to, drawn_lit, eased
from tools.dev.focus import chosen
from views_kit import Report


@functools.lru_cache(maxsize=1)
def built():
    """The atlas's tree, built once for every test: what it lays kept with it (atlas/apps.py's caches)."""
    from tools.dev import atlas_tree
    return atlas_tree.build()


def opened(depth):
    """(the atlas's tree, the page's state, [the nodes opened]): the robot chosen (`down_to`), the heaviest of her
    folders keyed opened in the tree, then for each depth more its first child keyed that holds any."""
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import subtree
    from tools.dev.atlas_tree import FILE, FOLDER
    tree = built()
    state = show_atlas.opened(tree)
    state['size'] = (150, 44)
    kids, below = subtree.holding(tree)
    down_to(state, tree.at[ROBOT])
    keys, way = show_atlas.keyed(state), []
    node = max((n for n in keys if n >= 0 and tree.kind[n] == FOLDER), key=lambda n: below[n][FILE])
    while node is not None and len(way) < depth:
        bindings.act_on(show_atlas.PAD, [keys[node].lower()], state)
        eased(state)
        way.append(node)
        keys = show_atlas.keyed(state)
        node = next((n for n in keys if n in kids[node] and kids[n]), None)
    return tree, state, way


def grown(tree, root):
    """What the subtree `root` grows (subtree.grown) where the tree holds it."""
    from terminal.views.atlas import apps, subtree
    return subtree.grown(tree, root, apps.grove(tree)[0][root], -2 - len(apps.clades(tree)['pages']))


def misses(tree, root):
    """The largest misses of the subtree `root` grows from the tree's own rule (atlas/growth.py), at every level of
    it: {a bough off its step on its stem, a leader off straight, a lateral's chord off Murray's angle of what it
    holds of what the stem carries there, a lateral's way round the stem off the golden angle from the last's}."""
    import math
    from coaxial.model.blocks import numpy as np
    from terminal.views.atlas import growth, scene, subtree
    _at, _rows, nested, _up, _names, boughs = grown(tree, root)
    off = dict.fromkeys(('step', 'leader', 'murray', 'golden'), 0.0)

    @functools.lru_cache(maxsize=None)
    def hold(n):
        return subtree.held(tree, n) if n >= 0 else sum(hold(k) for k in nested[n])
    for node, kids in nested.items():
        if not kids:
            continue
        ahead = [k for k in kids if nested.get(k)]
        leader = max(ahead, key=lambda k: (hold(k), -k)) if ahead else None
        order = sorted((k for k in kids if k != leader), key=lambda k: (-hold(k), k)) + [leader] * bool(ahead)
        above, last, steps = sum(map(hold, kids)), None, max(1, len(kids))
        for j, k in enumerate(order, steps - len(kids) + 1):
            at, way = growth.along(boughs[node], j / steps)
            off['step'] = max(off['step'], math.dist(at, boughs[k][0]), math.dist(way, boughs[k][1]))
            chord, ahead = np.subtract(growth.along(boughs[k], 1.0)[0], boughs[k][0]), np.array(way)
            angle = math.atan2(float(np.linalg.norm(np.cross(chord, ahead))), float(chord @ ahead))   # exact at 0 too
            off['leader' if k == leader else 'murray'] = max(off['leader' if k == leader else 'murray'],
                                                             abs(angle - growth.murray(above, hold(k))))
            if angle > 1e-6:   # its way round the stem in the stem's own frame there (growth.lean)
                side, other = np.array(growth.lean(way, 0.0)), np.array(growth.lean(way, math.pi / 2))
                turn = math.atan2(float(chord @ other), float(chord @ side))
                if last is not None:
                    miss = (turn - last - scene.GOLDEN) % (2 * math.pi)
                    off['golden'] = max(off['golden'], min(miss, 2 * math.pi - miss))
                last = turn
            above -= hold(k)
    return off


def test_a_folder_opened_grows_its_subtree(report):
    """A folder opened in the applications' tree grows its whole subtree out of it by the tree's own rule
    (atlas/subtree.py, atlas/growth.py): its children part along branches by the files their code calls, a
    file's symbols by the symbols theirs calls, every node a bough out of its stem at its step, the one holding
    most leading straight on, the rest at Murray's angles round the stem by the golden angle - forks at every
    level, no pole -; the tree itself unmoved; the subtree lit, two depths of it drawn, its children keyed where
    named in view and listed in its panel - still the applications' tree."""
    import math
    from terminal.views import show_atlas
    from terminal.views.atlas import scene, subtree
    tree, state, _way = opened(0)
    home = dict(show_atlas.placed(state)['at'])
    tree, state, (folder,) = opened(1)
    laid, (kids, below) = show_atlas.placed(state), subtree.holding(tree)
    at, _rows, nested, _up, _names, _boughs = grown(tree, folder)
    real, forks = {n for n in at if n >= 0}, {n for n in at if n < 0}
    depth = {n: len(scene.chain(tree, n)) - len(scene.chain(tree, folder)) for n in real}
    off, moved = misses(tree, folder), max(math.dist(laid['at'][n], p) for n, p in home.items() if n not in at)
    report.check("a folder opened grows its whole subtree out of it by the tree's rule: forks at every level, each "
                 "bough at its step, the leader straight on, Murray's angles, the golden turns; the tree unmoved",
                 show_atlas._apping(state) and show_atlas.titled(state, None, 100) == 'APPS' and state['focus'] == folder
                 and real == set(subtree.under(tree, folder)) - {folder} and forks and max(depth.values()) >= 2
                 and len(nested[folder]) < len(kids[folder]) and all(v < 1e-9 for v in off.values()) and moved < 1e-12,
                 '%s: %d nodes, %d forks, %d deep, %d kids on its shoot of %d children; %s; the tree moved %.0e'
                 % (scene.short(tree, folder), len(real), len(forks), max(depth.values()), len(nested[folder]),
                    len(kids[folder]), ', '.join('%s %.1e' % kv for kv in off.items()), moved))
    drawn, keyed = set(laid['flat']['nodes'].tolist()), show_atlas.keyed(state)
    named = set().union(*laid['groups'])
    report.check('the subtree lit, two depths of it drawn, its children keyed where named in view, listed in its panel',
                 laid['flat']['lit'] == {folder, *at} and {n for n in real if depth[n] <= 2} == drawn & real
                 and keyed and all(n in named for n in keyed) and any(n in kids[folder] for n in keyed)
                 and laid['rings'] == {folder: scene.children(tree, kids, below, folder)},
                 '%d lit, %d drawn, %d keyed' % (len(laid['flat']['lit']), len(drawn & real), len(keyed)))


def test_a_node_opened_in_it_lights_its_way(report):
    """A file opened in a subtree lights its way down - the forks between too - and all it holds, its siblings
    dim, its children in its panel; BKSP down the way back to the folder, out of the subtree to the page, then
    the branch above (show_atlas.py, atlas/apps.py)."""
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import apps, scene, subtree
    tree, state, (folder, file) = opened(2)
    laid, (kids, below) = show_atlas.placed(state), subtree.holding(tree)
    _at, _rows, nested, up, _names, _boughs = grown(tree, folder)
    way, mine = [file], [file]
    while way[-1] != folder:
        way.append(up[way[-1]])
    for n in mine:
        mine += nested.get(n, ())
    ink = dict(zip(laid['flat']['nodes'].tolist(), laid['flat']['ink'].tolist()))
    report.check('a file opened lights its way down, the forks between, and all it holds; its siblings dim',
                 state['focus'] == file and laid['flat']['lit'] == {*way, *mine} and ink[file] == scene.OPEN
                 and all(ink[c] == scene.DIM_NODE for c in kids[folder] if c != file and c in ink)
                 and laid['rings'] == {file: scene.children(tree, kids, below, file)},
                 '%s: %d on its way, %d it holds' % (scene.short(tree, file), len(way), len(mine)))
    robot, climbed = tree.at[ROBOT], []
    for _ in range(3):
        bindings.act_on(show_atlas.PAD, ['\x08'], state)
        climbed.append((state['focus'], state['app']))
    report.check('BKSP down the way back to the folder, out of the subtree to the page, then the branch above',
                 climbed == [(folder, robot), (-1, robot), (-1, apps.clades(tree)['up'][robot])]
                 and show_atlas._apping(state), str(climbed))


def test_a_subtree_stands_in_the_room_the_panel_leaves(report):
    """A subtree opened stands in the middle of the room its panel leaves (atlas/aim.py, atlas/leaves.py): the
    projection of what is lit at its frame - a folder's subtree, a file's way and all it holds - has its middle
    half a panel off the drawing's middle, away from the panel, and fits the room beside it - as it is drawn."""
    from terminal.views import show_atlas
    from terminal.views.atlas import leaves, scene
    off = {}
    for depth, label in ((1, 'a folder'), (2, 'a file')):
        _tree, state, _way = opened(depth)
        sight = scene.view(state['cam'], *show_atlas.sized(state))
        seen = [tuple(sum((p[k] - sight['eye'][k]) * sight[a][k] for k in range(3)) for a in ('r', 'u', 'f'))
                for p in drawn_lit(show_atlas.placed(state))]
        xs, ys = [x / z for x, _y, z in seen], [y / z for _x, y, z in seen]
        aside = leaves.width(show_atlas.sized(state)[0])   # half the panel, dots
        wide = sight['focal'] * (max(xs) - min(xs)) - (sight['w'] - 2 * aside)
        off[label] = max(abs(abs(sight['focal'] * (max(xs) + min(xs)) / 2) - aside),
                         sight['focal'] * abs(max(ys) + min(ys)) / 2, wide)
    report.check("a subtree opened framed in the middle of the room its panel leaves, fitting it - a folder's, a "
                 "file's", all(d < 1e-6 for d in off.values()), ', '.join('%s %.1e dots off' % kv for kv in off.items()))


def test_every_level_of_every_subtree(report):
    """Every node the tree holds - each folder, each page - grows its subtree by the tree's rule, and at every
    depth of it the heaviest node opened (a page chosen first, opened again) shows it so: still the applications'
    tree, lit its way down and all it holds, its children keyed where named in view and listed in its panel, in
    the room its panel leaves, the tree unmoved; BKSP down its way back to the node the tree holds, then out."""
    import math
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import apps, leaves, scene, subtree
    from tools.dev.atlas_tree import FILE, FOLDER, SYMBOL
    tree = built()
    kids, below = subtree.holding(tree)
    roots = sorted(n for n in apps.grove(tree)[0] if n >= 0 and tree.kind[n] in (FOLDER, FILE))
    home = show_atlas.opened(tree)
    home['size'] = (150, 44)
    show_atlas.focused(home, -1)
    home, worst, faults, opened_at = dict(show_atlas.placed(home)['at']), {}, [], {}
    for root in roots:
        for k, v in misses(tree, root).items():
            worst[k] = max(worst.get(k, 0.0), v)
        node, depth = root, 0
        while node is not None:
            state, own = show_atlas.opened(tree), apps.anchor(tree, node)   # its own: a page in a folder its own root
            state['size'] = (150, 44)
            at, _rows, nested, up, _names, _boughs = grown(tree, own)
            for _ in range(2):   # a page chosen first, opened again
                if state['focus'] != node:
                    show_atlas.opening(state, node)
            eased(state)
            laid, way, mine = show_atlas.placed(state), [node], [node]
            while way[-1] != own:
                way.append(up[way[-1]])
            for n in mine:
                mine += nested.get(n, ())
            ring, keyed = scene.children(tree, kids, below, node), show_atlas.keyed(state)
            sight, cols = scene.view(state['cam'], *show_atlas.sized(state)), show_atlas.sized(state)[0]

            def seen(p):
                x, y, z = (sum((p[k] - sight['eye'][k]) * sight[a][k] for k in range(3)) for a in ('r', 'u', 'f'))
                return sight['w'] / 2 + sight['focal'] * x / z, sight['h'] / 2 - sight['focal'] * y / z
            xs, ys = zip(*(seen(p) for p in drawn_lit(laid)))
            room = max(abs(abs((max(xs) + min(xs)) / 2 - sight['w'] / 2) - leaves.width(cols)),
                       abs((max(ys) + min(ys)) / 2 - sight['h'] / 2), max(xs) - min(xs) - (sight['w'] - 2 * leaves.width(cols)))
            inside = all(0 <= seen(laid['at'][n])[0] < sight['w'] and 0 <= seen(laid['at'][n])[1] < sight['h'] for n in keyed)
            moved = max([math.dist(laid['at'][n], p) for n, p in home.items() if n not in at] + [0.0])
            climbed = []
            while state['focus'] >= 0:
                bindings.act_on(show_atlas.PAD, ['\x08'], state)
                climbed.append(state['focus'])
            ok = {'apps': show_atlas._apping(state) and state['focus'] == -1, 'focus': laid['focus'] == node,
                  'lit': laid['flat']['lit'] == {*way, *mine}, 'rings': laid['rings'] == {node: ring},
                  'keys': inside and all(n in set().union(*laid['groups']) for n in keyed)
                  and (not ring or any(n in ring for n in keyed)),
                  'room': room < 1e-6, 'moved': moved < 1e-12, 'back': climbed == [n for n in way if n >= 0][1:] + [-1]}
            opened_at[depth] = opened_at.get(depth, 0) + 1
            faults += [(scene.short(tree, node), depth, [k for k, v in ok.items() if not v])] * (not all(ok.values()))
            node, depth = max(kids[node], key=lambda n: (below[n][SYMBOL], -n), default=None), depth + 1
    report.check("every node the tree holds grows its subtree by the tree's rule, and the heaviest at every depth of "
                 "it opened shows it so: lit, keyed, listed, in the room its panel leaves, the tree unmoved, BKSP out",
                 roots and all(v < 1e-9 for v in worst.values()) and not faults and len(opened_at) >= 3,
                 '%d roots, opened %s by depth; %s; faults %s' % (len(roots), opened_at,
                                                                ', '.join('%s %.1e' % kv for kv in worst.items()), faults[:3]))


def test_children_part_by_all_they_share(report):
    """Every stem of every subtree parts its children by all they share (atlas/subtree.py): the C cores' functions
    by the calls their bodies make, read from every C file the tree holds; a stem forks wherever two or more of its
    children, not all, share anything - what their code reaches, the words of their names and parameters -; none
    of eight children or more stands a pole."""
    from terminal.views.atlas import apps, subtree
    from tools.dev.atlas_tree import FILE, FOLDER
    tree = built()
    kids, calls = subtree.holding(tree)[0], subtree.called(tree)
    roots = sorted(n for n in apps.grove(tree)[0] if n >= 0 and tree.kind[n] in (FOLDER, FILE))
    cores = {}
    for ident, node in tree.at.items():
        path, _, name = ident.partition('::')
        if name and path.endswith('.c'):
            cores[path.split('/')[0]] = cores.get(path.split('/')[0], 0) + bool(calls.get(node))
    stems, unforked, poles, loose, every = 0, [], [], 0, 0
    for root in roots:
        _at, _rows, nested, _up, _names, _boughs = grown(tree, root)
        for node, ks in nested.items():
            if node < 0 or not kids[node]:
                continue
            ring, forked = set(kids[node]), any(k < 0 for k in ks)
            stems, loose, every = stems + 1, loose + sum(1 for k in ks if k >= 0), every + len(ring)
            partial = any(1 < len(set(s) & ring) < len(ring) for s in subtree.shared(tree, node).values())
            unforked += [tree.id[node]] * (partial and not forked)
            poles += [tree.id[node]] * (len(ring) >= 8 and not forked)
    report.check("every stem parts its children by all they share - the C cores' calls read -, forked wherever any "
                 "two share anything, no pole", cores.get('wbc', 0) > 0 and cores.get('world', 0) > 0 and not unforked
                 and not poles, '%d stems, %.0f%% of children straight on their stem; C functions calling: %s; unforked '
                 '%s, poles %s' % (stems, 100.0 * loose / every, cores, unforked[:3], poles[:3]))


def test_the_panel_never_covers_what_is_lit(report):
    """Turned and tipped as a drag does, the view keeps its frame (atlas/aim.eased): what is lit stays in the room
    its panel leaves, the panel on the side the frame left it (atlas/leaves.py) - over none of what is lit, in the
    opening view, the flyers' branch, every page chosen, every node the tree holds opened."""
    from terminal.views import show_atlas
    from terminal.views.atlas import aim, apps, leaves, scene
    from tools.dev.atlas_tree import FILE, FOLDER
    tree = built()
    held = apps.clades(tree)['pages']
    flyers = max(held, key=lambda b: (len(held[b]) == len(FLYERS), -len(held[b])))
    roots = sorted(n for n in apps.grove(tree)[0] if n >= 0 and tree.kind[n] in (FOLDER, FILE))
    views = [('tree', -1), ('chosen', flyers), *(('chosen', p) for p, *_said in apps.pages(tree)),
             *(('opened', r) for r in roots)]
    dim, covered, frames = {scene.FAINT, scene.DIM_EDGE, scene.DIM_NODE}, [], 0
    for kind, node in views:
        state = show_atlas.opened(tree)
        state['size'] = (150, 44)
        if kind == 'tree':
            show_atlas.focused(state, -1)
        elif kind == 'chosen':
            show_atlas.opening(state, node)
        while kind == 'opened' and state['focus'] != node:   # a page chosen first, opened again
            show_atlas.opening(state, node)

        def keep():
            return show_atlas._apping(state) and ((state['app'], state['focus']), *show_atlas.frame(state, state['focus']))
        aim.eased(state, 0.0, keep)   # the page's first frame: its frame taken as the view faces
        yaw = state['cam']['yaw']
        for pitch in (-30.0, 18.0, 60.0):
            for turn in range(0, 360, 45):
                state['cam'].update(yaw=yaw + turn, pitch=pitch)
                aim.eased(state, 0.0, keep)
                eased(state)
                cols, rows = show_atlas.sized(state)
                laid, marks = show_atlas.placed(state), show_atlas.keyed(state)
                lines, fell, _v = scene.picture(laid, state['cam'], cols, rows, marks, set(), None)
                lit = {(c, r) for r, (line, go, inks) in enumerate(lines) for k, (a, ink) in enumerate(zip(go, inks))
                       for c in range(a, go[k + 1] if k + 1 < len(go) else len(line))
                       if '⠁' <= line[c] <= '⣿' and ink // scene.SHADES not in dim}
                boxed = leaves.panels(tree, laid['rings'], fell, {n: marks.get(n, '') for n in show_atlas.context(state)},
                                      cols, rows, frozenset(), most=laid.get('panels', leaves.MOST), names=laid.get('names'),
                                      side=laid.get('side'))
                frames += 1
                covered += [(scene.short(tree, node) if node >= 0 else node, turn, pitch)] * bool(set(boxed) & lit)
    report.check('turned and tipped, the view keeps its frame and the panel covers none of what is lit - the opening '
                 'view, a branch, every page, every node the tree holds', not covered and frames == len(views) * 24,
                 '%d views, %d frames; covering %s' % (len(views), frames, covered[:3]))


ROSTER = (test_a_folder_opened_grows_its_subtree, test_a_node_opened_in_it_lights_its_way,
          test_a_subtree_stands_in_the_room_the_panel_leaves, test_every_level_of_every_subtree,
          test_children_part_by_all_they_share, test_the_panel_never_covers_what_is_lit)


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
