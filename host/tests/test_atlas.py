#!/usr/bin/env python3
"""The atlas holds the tree on one trunk, the terminal up to the shunts, its ties read off the code.

Every tracked path in a layer of the table and every prefix of it holding one; the trunk from the terminal
to the analog front end, the word 'shunt' found up it; ties between files of every kind, the wire's two
ends among them; a chain of ties across the wire; the trunk, a layer, a folder and a file opened as lines;
the page with its script and the atlas inlined; the terminal's tree in 3D walked by its keys and its mouse
(tools/dev/atlas.py, tools/dev/atlas_tree.py, terminal/views/show_atlas.py, terminal/views/atlas/)."""
import functools
import json
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev import atlas, atlas_tree
from tools.dev.focus import chosen
from structure_kit import Report


@functools.lru_cache(maxsize=1)
def built():
    """The tree's atlas, read once for every test - afresh, no cache: what a cache must give back."""
    return atlas_tree.build(None)


def test_every_path_lies_in_one_layer(report):
    """Every path git tracks - a checked-out submodule's files with it - lies in a layer of the table and is
    a file of the atlas, and every prefix of the table holds a path: a new top-level folder is placed."""
    paths = atlas_tree.tracked()
    lost = [p for p in paths if atlas_tree.layer_of(p) is None]
    report.check('every tracked path lies in a layer', not lost, ', '.join(lost[:5]))
    dead = [prefix for row in atlas_tree.LAYERS for prefix in row[3] if not any(p.startswith(prefix) for p in paths)]
    report.check('and every prefix of the table holds one', not dead, ', '.join(dead))
    files = built().kind.count(atlas_tree.FILE)
    report.check('the atlas holds each path as a file', files == len(paths), '%d of %d' % (files, len(paths)))


def test_the_trunk_climbs_from_the_terminal_to_the_shunts(report):
    """The trunk runs from the terminal at its foot to its sensing at its crown, the analog front end, and the
    word 'shunt' is found up it: the terminal's drawings, the rig's scaling, the wire, the cores, the board's
    reads of the converters."""
    trunk = [row[0] for row in atlas_tree.LAYERS if row[1] == 'trunk']
    report.check('the trunk runs from the terminal to its sensing',
                 trunk[0] == 'terminal' and trunk[-1] == 'sensing', ' '.join(trunk))
    reached = {atlas_tree.LAYERS[layer][0] for layer in atlas.found(built(), 'shunt')}
    report.check("'shunt' is found on the terminal, the rig, the wire, the cores and the board",
                 {'terminal', 'rig', 'wire', 'cores', 'board'} <= reached, ', '.join(sorted(reached)))


def test_ties_join_files(report):
    """Every tie joins two files, none to itself; each kind is read; the host's end of the wire reaches the
    board's commands by the names and numbers both ends give them."""
    tree = built()
    wrong = [(s, d) for s, d in zip(tree.src, tree.dst)
             if s == d or tree.kind[s] != atlas_tree.FILE or tree.kind[d] != atlas_tree.FILE]
    report.check('every tie joins two files', not wrong, str(wrong[:3]))
    kinds = sorted({atlas_tree.TIES[t] for t in tree.tie})
    report.check('ties of every kind are read', kinds == sorted(atlas_tree.TIES), ', '.join(kinds))
    wire = {(tree.id[s], tree.id[d]) for s, d, t in zip(tree.src, tree.dst, tree.tie) if atlas_tree.TIES[t] == 'wire'}
    report.check("the wire's host end reaches the board's commands",
                 ('host/coaxial/comm/protocol.py', 'comms/src/cmd_board.c') in wire, str(sorted(wire)[:3]))
    imports = {(tree.id[s], tree.id[d]) for s, d, t in zip(tree.src, tree.dst, tree.tie) if atlas_tree.TIES[t] == 'imports'}
    report.check("a script's bare import of a module beside it, in a folder that is no package, is a tie",
                 ('host/tests/test_ci.py', 'host/tests/gynoid_kit.py') in imports, '')


def test_a_chain_of_ties_crosses_the_wire(report):
    """--path: a device's module to its handler, its import of the protocol module, then the wire."""
    chain = atlas.show_path(built(), 'host/coaxial/devices/analog.py', 'comms/src/cmd_board.c')
    report.check('a device reaches its handler in two ties, the last the wire',
                 len(chain) == 3 and chain[-1].lstrip().startswith('-wire-> comms/src/cmd_board.c'), ' | '.join(chain))


def test_each_node_opens_as_lines(report):
    """The trunk lists its layers, the crown first and the terminal last of them; a layer, a folder and a
    file open on their children and their ties; a name the atlas has not is none."""
    tree = built()
    trunk = atlas.show_trunk(tree)
    names = [line.split()[1] for line in trunk[2:] if line[:2].strip().isdigit()]
    report.check('the trunk lists its layers from the crown to the terminal', names[0] == 'sensing'
                 and names[-1] == 'terminal' and len(names) == 9, ' '.join(names))
    for key in ('executive', 'host/machine/law', 'host/coaxial/comm/protocol.py'):
        lines = [line for i in atlas.find_node(tree, key) for line in atlas.show_node(tree, i, 1)]
        report.check('%s opens on its children and ties' % key,
                     len(lines) > 3 and lines[-2].startswith('  -> ') and lines[-1].startswith('  <- '),
                     lines[0][:70] if lines else 'nothing')
    report.check('a name the atlas has not is none', atlas.find_node(tree, 'no/such/node') == [], '')
    parts = [tree.id[i] for i in atlas.find_node(tree, 'host/')]
    report.check("a folder on two layers is a node on each, its name finding both: host/'s agents and workshop",
                 sorted(parts) == ['host/@agents', 'host/@workshop'], str(parts))
    report.check('--depth 2 opens each layer of the trunk too', len(atlas.show_trunk(tree, 2)) > 2 * len(trunk),
                 '%d lines to %d' % (len(atlas.show_trunk(tree, 2)), len(trunk)))
    told = [atlas.told(('7ae76b6d', n)) for n in (0, 1, 3)]
    report.check('a commit said with the files the working tree has changed since',
                 told == ['7ae76b6d', '7ae76b6d + 1 file changed', '7ae76b6d + 3 files changed'], str(told))


def test_a_build_keeps_what_it_read(report):
    """atlas_tree.build with a cache: laying it and building from it both give the atlas read afresh, list
    for list - its ties in one order whatever the hash seed -; a file whose stamp moved is read again and
    kept again; a cache another code laid is left; a file's text is read when a word is looked for in it."""
    import marshal
    fresh, path = built(), 'host/tools/dev/atlas_tree.py'
    columns = ('id', 'up', 'kind', 'label', 'brief', 'line', 'src', 'dst', 'tie')

    def differs(atlas):
        return [c for c in columns if getattr(atlas, c) != getattr(fresh, c)]

    with tempfile.TemporaryDirectory() as folder:
        cache = os.path.join(folder, 'atlas.cache')
        laid, kept = atlas_tree.build(cache), atlas_tree.build(cache)
        report.check('laying the cache and building from it give the atlas read afresh',
                     not differs(laid) and not differs(kept), str(differs(laid) + differs(kept)))
        with open(cache, 'rb') as f:
            code, files = marshal.loads(f.read())
        stale = ((-1, -1), 'stale') + tuple(files[path][2:])
        with open(cache, 'wb') as f:
            f.write(marshal.dumps((code, {**files, path: stale})))
        again = atlas_tree.build(cache)
        with open(cache, 'rb') as f:
            told = marshal.loads(f.read())[1][path]
        report.check('a file whose stamp moved is read again, and kept again',
                     not differs(again) and told[atlas_tree.STAMP] != (-1, -1) and told[atlas_tree.BRIEF] != 'stale',
                     str(told[:2]))
        with open(cache, 'wb') as f:
            f.write(marshal.dumps(('another code', {**files, path: stale})))
        report.check("a cache another code laid is left", atlas_tree._kept(cache) == {}, '')
    report.check("a file's text read when a word is looked for in it", 'def build(' in fresh.lowered(path), '')


def test_the_page_carries_the_atlas(report):
    """--html: the template beside atlas.py with its script (atlas.js) and the atlas inlined - every node and
    tie -, ASCII, nothing left to fill: one file a browser opens from anywhere."""
    tree = built()
    with tempfile.TemporaryDirectory() as folder:
        with open(atlas.page(tree, os.path.join(folder, 'atlas.html')), encoding='utf-8') as f:
            text = f.read()
    data = json.loads(text.split('type="application/json">', 1)[1].split('</script>', 1)[0])
    report.check('the page inlines every node and tie', len(data['id']) == len(tree.id)
                 and len(data['src']) == len(tree.src), '%d nodes, %d ties' % (len(data['id']), len(data['src'])))
    with open(atlas.SCRIPT, encoding='utf-8') as f:
        script = f.read()
    report.check('its script inlined whole, ASCII, nothing left to fill',
                 script.replace('</', '<\\/') in text and text.isascii() and '__ATLAS__' not in text
                 and '__SCRIPT__' not in text, '%d characters of script' % len(script))


def test_the_terminal_page_walks_the_tree(report):
    """terminal/views/show_atlas.py on its state: the trunk in 3D, its twelve layers on F1 to F12 - the
    terminal F1 - drawn in braille and named, each open already (its folders laid out ahead), the trunk lit
    and the rest greyed out, the key nearest the viewport named with its brief along the top; F4
    chooses the executive - it and all laid out below it lit, the rest dim, its folders level round it with
    their files laid out ahead and the camera eased to frame them -, F1 its heaviest folder, its files
    hanging under it lit with it, the executive lit above it, BKSP back on the
    layer; the panel lists the layers far, the chosen layer's folders at its frame; a left drag turns the
    tree within its tip, the wheel brings it near; / and a word put its hits on the keys, ESC lets the word
    go and ESC again leaves for the front page."""
    from terminal.ui import bindings, screen
    from terminal.views import show_atlas
    from terminal.views.atlas import panel, scene
    tree = built()
    state = show_atlas.opened(tree)

    def typed(*keys):
        return bindings.act_on(show_atlas.PAD, list(keys), state)[0]

    def drawn():
        """The scene at the ease's end: its lines, the nodes drawn, the key nearest the viewport, the node the
        panel lists."""
        state['cam'].update(target=state['goal']['target'], distance=state['goal']['distance'])
        lines, fell, v = scene.picture(show_atlas.placed(state), state['cam'], 100, 36, show_atlas.keyed(state), set())
        mid = panel.middle(fell, v, show_atlas.keyed(state))
        return lines, fell, mid, show_atlas.listing(state, mid)

    def inks():
        """{node: its ink} of the scene as laid at its focus, a found word aside."""
        laid = scene.flat(show_atlas.placed(state))
        return dict(zip(laid['nodes'].tolist(), laid['ink'].tolist()))

    keys = show_atlas.keyed(state)
    report.check('at the trunk F1 to F12 open its twelve layers, the terminal F1',
                 list(keys) == list(range(len(atlas_tree.LAYERS))) and keys[0] == 'F1', str(keys))
    lines, fell, mid, listed = drawn()
    plain = screen.plain('\n'.join(lines))
    cells, named = sum(0x2800 <= ord(ch) <= 0x28FF for ch in plain), [row[0] for row in atlas_tree.LAYERS if row[0] in plain]
    report.check('the trunk drawn in braille, every layer before the camera and named; far, the panel lists the layers',
                 len(lines) == 36 and cells > 100 and all(layer in fell for layer in range(len(atlas_tree.LAYERS)))
                 and len(named) == len(atlas_tree.LAYERS) and listed == -1,
                 '%d rows, %d braille cells, %d drawn, %d named, listing %d' % (len(lines), cells, len(fell), len(named), listed))
    at = show_atlas.placed(state)['at']
    trunk, told = [i for i, row in enumerate(atlas_tree.LAYERS) if row[1] == 'trunk'], inks()
    report.check('every layer open already, its folders laid out ahead; nothing chosen, the trunk lit and the '
                 'rest greyed out',
                 all(f in at for layer in range(len(atlas_tree.LAYERS)) for f in state['kids'][layer])
                 and {told[t] for t in trunk} == {scene.NODE}
                 and {ink for node, ink in told.items() if node not in trunk} == {scene.DIM_NODE}, '%d nodes laid' % len(at))
    hidden, state['find'] = state['hidden'], '__init__'
    plumbing = [n for n, h in enumerate(hidden) if h and tree.kind[n] == atlas_tree.FILE]
    report.check('the tree keeps its __init__.py and __main__.py; the page has none as a child, finds none',
                 len(plumbing) > 10 and all(tree.label[n][:2] == '__' for n in plumbing)
                 and not any(hidden[c] for row in state['kids'] for c in row)
                 and not any(hidden[n] for rows in show_atlas.hits(state).values() for n, _ in rows), str(len(plumbing)))
    state['find'] = None
    title = show_atlas.titled(state, mid, 100)
    report.check('along the top: the key nearest the viewport, its name and its brief',
                 mid is not None and title.startswith('TREE  %s  %s' % (scene.short(tree, mid), tree.brief[mid][:20])),
                 title)
    executive = next(i for i, row in enumerate(atlas_tree.LAYERS) if row[0] == 'executive')
    typed('f%d' % (executive + 1))
    at, folders, told = show_atlas.placed(state)['at'], state['kids'][executive], inks()

    def under(node):
        """The nodes laid out below `node`."""
        return [n for n in at if n != node and node in scene.chain(tree, n)]

    def rest(node):
        """The inks of the nodes neither above `node`, nor it, nor laid out below it."""
        return {told[n] for n in at if n not in scene.chain(tree, node) and n not in under(node)}
    report.check('F4: the executive chosen and lit, its folders level round it and their files laid out ahead '
                 'under them all lit, the rest dim or a tie of its, the camera eased to frame them',
                 state['focus'] == executive and folders and all(at[f][1] == at[executive][1] for f in folders)
                 and all(at[c][1] < at[f][1] for f in folders for c in state['kids'][f])
                 and told[executive] == scene.OPEN and {told[n] for n in under(executive)} == {scene.NODE}
                 and rest(executive) <= {scene.DIM_NODE, scene.OUT, scene.IN} and scene.DIM_NODE in rest(executive)
                 and state['goal']['distance'] < scene.camera()['distance'],
                 '%d folders, %d laid out below' % (len(folders), len(under(executive))))
    lines, fell, mid, listed = drawn()
    report.check("at its frame the panel lists its folders", listed == executive, tree.id[listed] if listed >= 0 else '-1')
    heaviest = scene.children(tree, state['kids'], state['below'], executive)[0]
    typed('f1')
    at, files, told = show_atlas.placed(state)['at'], state['kids'][heaviest], inks()
    report.check('F1: its heaviest folder chosen, its files hanging under it lit with all laid out below it, '
                 'the executive lit above it',
                 state['focus'] == heaviest and files and all(at[f][1] < at[heaviest][1] for f in files)
                 and told[heaviest] == scene.OPEN and told[executive] == scene.NODE
                 and {told[n] for n in under(heaviest)} == {scene.NODE}, tree.id[heaviest])
    typed('\x08')
    report.check('BKSP: back on the layer', state['focus'] == executive, str(state['focus']))
    yaw, near = state['cam']['yaw'], state['cam']['distance']
    show_atlas.turned(state, 10, 100)
    show_atlas.wheeled(state, [], 0.12)
    report.check('a left drag turns the tree and tips it within its bounds; the wheel brings it near',
                 state['cam']['yaw'] == yaw - 10 * show_atlas.DRAG_DEG[0] and state['cam']['pitch'] == show_atlas.TIP_DEG[1]
                 and abs(state['cam']['distance'] * math.exp(0.12) - near) < 1e-9, str(state['cam']))
    typed('/', *'shunt')
    hits = show_atlas.context(state)
    report.check('/ and a word: its hits on the keys while it is typed', state['find'] == 'shunt' and state['typing']
                 and len(hits) > 5 and list(show_atlas.keyed(state)) == hits[:show_atlas.PAGE], '%d hits' % len(hits))
    typed('\r')
    first, then = typed('\x1b'), typed('\x1b')
    report.check('ESC lets the word go, ESC again the front page', state['find'] is None and first is None
                 and then == 'menu', '%r then %r' % (first, then))


def test_the_scene_is_rastered_as_data(report):
    """terminal/views/atlas/scene.py rasters every segment and disc of a frame in one pass: each segment's
    dots those its own line drew - cut to the drawing, then its longer extent's dots and two more spaced by
    numpy.linspace, each where it falls -, each disc's the dots whose middles lie within its radius; 300
    segments and 200 discs at random, ends far off the drawing among them."""
    import numpy as np
    from terminal.views.atlas import scene
    w, h, rng = 200, 120, np.random.default_rng(3)

    def line(x0, y0, x1, y1):
        """One segment's dots by the rule, a segment at a time."""
        t0, t1, dx, dy = 0.0, 1.0, x1 - x0, y1 - y0
        for p, q in ((-dx, x0), (dx, w - 1e-3 - x0), (-dy, y0), (dy, h - 1e-3 - y0)):
            if p == 0.0 and q < 0.0:
                return set()
            t0, t1 = (max(t0, q / p), t1) if p < 0.0 else (t0, min(t1, q / p)) if p > 0.0 else (t0, t1)
        if t0 > t1:
            return set()
        ax, ay, bx, by = x0 + dx * t0, y0 + dy * t0, x0 + dx * t1, y0 + dy * t1
        n = int(max(abs(bx - ax), abs(by - ay))) + 2
        xs = np.clip(np.linspace(ax, bx, n).astype(np.intp), 0, w - 1)
        ys = np.clip(np.linspace(ay, by, n).astype(np.intp), 0, h - 1)
        return set((ys * w + xs).tolist())

    x0, y0, x1, y1 = (rng.uniform(-300, 500, 300) for _ in range(4))
    flat, of, along = scene.segments(x0, y0, x1, y1, w, h)
    batched = [set(flat[of == k].tolist()) for k in range(300)]
    wrong = [k for k in range(300) if batched[k] != line(x0[k], y0[k], x1[k], y1[k])
             or not np.array_equal(along[of == k], np.arange(np.count_nonzero(of == k)))]
    report.check("every segment's dots, rastered with all the others, are those its own line drew, counted "
                 "along it from its start", not wrong and sum(map(len, batched)) > 1000, str(wrong[:5]))
    x, y, r = rng.uniform(-8, w + 8, 200), rng.uniform(-8, h + 8, 200), rng.uniform(0.8, 5.0, 200)
    flat, of = scene.discs(x, y, r, w, h)
    wrong = [k for k in range(200) if set(flat[of == k].tolist()) != {
        py * w + px for py in range(h) for px in range(w)
        if abs(px - x[k]) < 7 and abs(py - y[k]) < 7 and (px + 0.5 - x[k]) ** 2 + (py + 0.5 - y[k]) ** 2 <= r[k] * r[k]}]
    report.check("every disc's dots, rastered with all the others, are those whose middles lie within its radius",
                 not wrong, str(wrong[:5]))
    cam = scene.camera()
    near, v, far = cam['distance'], scene.view(cam, 100, 36), 1.8 * cam['distance']

    def at(depth, up, side):
        return [cam['target'][k] + v['f'][k] * (depth - near) + v['u'][k] * up + v['r'][k] * side for k in range(3)]
    lines = scene.picture({'tree': built(), 'focus': -1, 'out': {}, 'into': {}, 'flat': {
        'nodes': np.zeros(0, np.intp), 'pos': np.zeros((0, 3)), 'disc': np.zeros(0), 'ink': np.zeros(0, np.uint8),
        'lit': set(), 'from': np.array([at(near, 0.5, -1), at(far, -0.5 * far / near, -far / near)]),
        'to': np.array([at(near, 0.5, 1), at(far, -0.5 * far / near, far / near)]),
        'segink': np.array([scene.EDGE] * 2, np.uint8), 'thick': np.zeros(2, bool)}}, cam, 100, 36, {}, set())[0]
    dots = [sum(bin(ord(c) - 0x2800).count('1') for c in line if 0x2800 < ord(c) <= 0x28FF) for line in lines]
    sharp, sparse, edge = sum(dots[:18]), sum(dots[18:]), scene.EDGE * scene.SHADES
    report.check('a line at the point the view turns about drawn whole, one as long on the screen far off it a fifth '
                 'of its dots, darkest', scene.blur(0.0) == 0 and abs(scene.blur(scene.FOCUS) - 1 + math.exp(-1)) < 1e-9
                 and sharp > 24 and sparse < 0.35 * sharp and scene.shaded(edge) in ''.join(lines[:18])
                 and scene.shaded(edge + scene.SHADES - 1) in ''.join(lines[18:]), '%d and %d dots' % (sharp, sparse))


def test_the_page_holds_the_mouse(report):
    """show_atlas.mouse: the page holds the mouse from its start - no F waited for: a waiting page's clicks
    went to the terminal's own selection and paste, and a chord of both buttons froze the tree -; a chord of
    both buttons through the console's decoder (terminal.ui.console.Keys) turns the tree, moves it and draws
    on; a notch of the wheel is the same step in and out, ten notches down never leap in."""
    from terminal.ui.console import Keys
    from terminal.views import show_atlas
    from terminal.views.atlas import scene
    state = show_atlas.opened(built())
    hands = show_atlas.mouse(state)
    report.check('held from the start: no key to wait for, a left drag, a right drag, a click, the wheel',
                 hands['mouse'] and hands['select'] == frozenset()
                 and all(hands.get(k) for k in ('on_drag', 'on_pan', 'on_click', 'on_input')), str(sorted(hands)))
    state['size'], state['art'] = (140, 44), (100, 36)
    state['fell'], state['view'] = scene.picture(show_atlas.placed(state), state['cam'], 100, 36, {}, set())[1:]
    keys, before = Keys(False, mouse=True, pan=True), dict(state['cam'])
    for button, col, row, kind in ((0, 60, 20, 'M'), (2, 60, 20, 'M'), (32, 64, 21, 'M'), (34, 69, 23, 'M'),
                                   (0, 69, 23, 'm'), (34, 73, 24, 'M'), (2, 73, 24, 'm')):
        keys._mouse(button, col, row, kind)
    hands['on_drag'](*keys.dragged())
    hands['on_pan'](*keys.panned())
    for col, row in keys.clicked():
        hands['on_click'](col, row)
    lines = scene.picture(show_atlas.placed(state), state['cam'], 100, 36, show_atlas.keyed(state), set())[0]
    report.check('a chord of both buttons turns the tree and moves it, and the next frame draws',
                 state['cam']['yaw'] != before['yaw'] and state['cam']['target'] != before['target']
                 and all(map(math.isfinite, state['cam']['target'])) and len(lines) == 36, str(state['cam']))
    near = state['cam']['distance']
    show_atlas.wheeled(state, [], 0.12)
    show_atlas.wheeled(state, [], -0.12)
    back = state['cam']['distance']
    show_atlas.wheeled(state, [], -1.2)
    report.check('a notch in and one out come back; ten notches down take it e^1.2 farther, never nearer',
                 abs(back - near) < 1e-9 and abs(state['cam']['distance'] - min(show_atlas.DISTANCE[1], near * math.exp(1.2))) < 1e-9,
                 '%.4f -> %.4f -> %.4f' % (near, back, state['cam']['distance']))


ROSTER = (test_every_path_lies_in_one_layer, test_the_trunk_climbs_from_the_terminal_to_the_shunts,
          test_ties_join_files, test_a_chain_of_ties_crosses_the_wire, test_each_node_opens_as_lines,
          test_a_build_keeps_what_it_read, test_the_page_carries_the_atlas, test_the_terminal_page_walks_the_tree,
          test_the_scene_is_rastered_as_data, test_the_page_holds_the_mouse)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
