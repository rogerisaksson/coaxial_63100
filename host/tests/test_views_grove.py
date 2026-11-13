"""The applications' tree grown down: a node opened in it grows its subtree as the tree grows, framed."""
import functools
import sys

from test_views_apps import ROBOT, down_to, eased
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
    from terminal.views.atlas import apps
    from tools.dev.atlas_tree import FILE, FOLDER
    tree = built()
    state = show_atlas.opened(tree)
    state['size'] = (150, 44)
    kids, below = apps.holding(tree)
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


def chords(at, stem, children, weight):
    """The largest miss of `children` (in their order, the heaviest lowest) out of the upright stem from `stem`
    a step each: (each one's chord off Murray's angle of its share, each one's way round the stem off the golden
    angle from the last's) - from where they stand alone."""
    import math
    from coaxial.model.blocks import numpy as np
    from terminal.views.atlas import growth, scene
    murray, golden, above, last = 0.0, 0.0, sum(weight.values()), None
    for j, c in enumerate(children, 1):
        chord = np.subtract(at[c], (stem[0], stem[1] + growth.APART * j, stem[2]))
        chord /= np.linalg.norm(chord)
        murray = max(murray, abs(math.acos(min(1.0, float(chord[1]))) - growth.murray(above, weight[c])))
        across = np.array([chord[0], 0.0, chord[2]])
        if np.linalg.norm(across) > 1e-6:   # the last carries all left, the shoot going on as it
            across /= np.linalg.norm(across)
            golden = max(golden, abs(float(last @ across) - math.cos(scene.GOLDEN))) if last is not None else golden
            last = across
        above -= weight[c]
    return murray, golden


def test_a_folder_opened_grows_its_subtree(report):
    """A folder opened in the applications' tree grows its subtree out of it (atlas/apps.py, atlas/growth.py), the
    same tree inherited down: a shoot straight up from where the folder stands, its children out of it a step
    each, the heaviest lowest - each holding itself and the files under it -, round the shoot by the golden
    angle from the last, its chord at Murray's angle; the tree itself unmoved; the folder and its children lit,
    keyed where named in view, framed, listed in its panel - still the applications' tree."""
    import math
    from terminal.views import show_atlas
    from terminal.views.atlas import apps, scene
    from tools.dev.atlas_tree import FILE
    tree, state, _way = opened(0)
    home = dict(show_atlas.placed(state)['at'])
    tree, state, (folder,) = opened(1)
    laid, (kids, below) = show_atlas.placed(state), apps.holding(tree)
    ring = scene.children(tree, kids, below, folder)
    weight = {c: 1 + below[c][FILE] for c in ring}
    murray, golden = chords(laid['at'], laid['at'][folder], sorted(ring, key=lambda c: (-weight[c], c)), weight)
    moved = max(math.dist(laid['at'][n], p) for n, p in home.items() if n not in ring)
    report.check("a folder opened grows its subtree out of it, the tree unmoved: a shoot straight up, its children "
                 "a step each, the heaviest lowest, round it by the golden angle, at Murray's angles",
                 show_atlas._apping(state) and show_atlas.titled(state, None, 100) == 'APPS' and state['focus'] == folder
                 and set(ring) <= set(laid['at']) and moved < 1e-12 and murray < 1e-9 and golden < 1e-9,
                 '%s: %d children; Murray %.1e, golden %.1e, the tree moved %.0e' % (scene.short(tree, folder), len(ring),
                                                                                    murray, golden, moved))
    sight, keyed = scene.view(state['cam'], *show_atlas.sized(state)), show_atlas.keyed(state)

    def shown(n):
        p = laid['at'][n]
        x, y, z = (sum((p[k] - sight['eye'][k]) * sight[a][k] for k in range(3)) for a in ('r', 'u', 'f'))
        a, d = sight['w'] / 2 + x * sight['focal'] / z, sight['h'] / 2 - y * sight['focal'] / z
        return z > scene.NEAR and 0 <= a < sight['w'] and 0 <= d < sight['h']
    named = set().union(*laid['groups'])
    report.check('the folder and its children lit, framed, keyed where named in view, listed in its panel',
                 laid['flat']['lit'] == {folder, *ring} and all(shown(n) for n in laid['flat']['lit'])
                 and keyed and all(n in named and shown(n) for n in keyed) and any(n in ring for n in keyed)
                 and laid['rings'] == {folder: ring}, '%d lit, %d keyed' % (len(laid['flat']['lit']), len(keyed)))


def test_a_file_opened_leads_on_up(report):
    """A file opened in a subtree leads its folder's shoot straight on up, its symbols out of it as the folder's
    children grew, it and the way down lit, its siblings dim; BKSP down the way back to the folder, out of the
    subtree to the page, then the branch above (show_atlas.py, atlas/apps.py)."""
    import math
    from terminal.ui import bindings
    from terminal.views import show_atlas
    from terminal.views.atlas import apps, growth, scene
    from tools.dev.atlas_tree import SYMBOL
    tree, state, (folder, file) = opened(2)
    laid, (kids, below) = show_atlas.placed(state), apps.holding(tree)
    ring, inner = scene.children(tree, kids, below, folder), scene.children(tree, kids, below, file)
    root = laid['at'][folder]
    straight = math.dist(laid['at'][file], (root[0], root[1] + growth.APART * (len(ring) + len(inner)), root[2]))
    weight = {s: 1 + below[s][SYMBOL] for s in inner}
    murray, golden = chords(laid['at'], (root[0], root[1] + growth.APART * len(ring), root[2]),
                            sorted(inner, key=lambda s: (-weight[s], s)), weight)
    ink = dict(zip(laid['flat']['nodes'].tolist(), laid['flat']['ink'].tolist()))
    report.check("a file opened leads its folder's shoot straight on up, its symbols out of it at Murray's angles "
                 "and the golden turns; it and the way down lit, its siblings dim", state['focus'] == file
                 and straight < 1e-9 and murray < 1e-9 and golden < 1e-9 and laid['flat']['lit'] == {folder, file, *inner}
                 and ink[file] == scene.OPEN and all(ink[c] == scene.DIM_NODE for c in ring if c != file)
                 and laid['rings'] == {folder: ring, file: inner},
                 '%s, %d symbols; %.0e off straight, Murray %.1e, golden %.1e' % (scene.short(tree, file), len(inner),
                                                                                 straight, murray, golden))
    robot, climbed = tree.at[ROBOT], []
    for _ in range(3):
        bindings.act_on(show_atlas.PAD, ['\x08'], state)
        climbed.append((state['focus'], state['app']))
    report.check('BKSP down the way back to the folder, out of the subtree to the page, then the branch above',
                 climbed == [(folder, robot), (-1, robot), (-1, apps.clades(tree)['up'][robot])]
                 and show_atlas._apping(state), str(climbed))


def test_a_subtree_stands_in_the_middle(report):
    """A subtree opened stands in the middle of the view (atlas/aim.py): the projection of what is lit at its
    frame - the folder and its children, the file, the way down and its symbols - has its middle on the
    drawing's middle."""
    from terminal.views import show_atlas
    from terminal.views.atlas import scene
    off = {}
    for depth, label in ((1, 'a folder'), (2, 'a file')):
        _tree, state, _way = opened(depth)
        flat, sight = scene.flat(show_atlas.placed(state)), scene.view(state['cam'], *show_atlas.sized(state))
        seen = [tuple(sum((p[k] - sight['eye'][k]) * sight[a][k] for k in range(3)) for a in ('r', 'u', 'f'))
                for n, p in zip(flat['nodes'].tolist(), flat['pos'].tolist()) if n in flat['lit']]
        xs, ys = [x / z for x, _y, z in seen], [y / z for _x, y, z in seen]
        off[label] = sight['focal'] * max(abs(max(xs) + min(xs)), abs(max(ys) + min(ys))) / 2   # dots
    report.check("a subtree opened framed with its projection's middle on the drawing's middle - a folder's, a "
                 "file's", all(d < 1e-9 for d in off.values()), ', '.join('%s %.1e dots off' % kv for kv in off.items()))


ROSTER = (test_a_folder_opened_grows_its_subtree, test_a_file_opened_leads_on_up, test_a_subtree_stands_in_the_middle)


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
