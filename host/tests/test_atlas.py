#!/usr/bin/env python3
"""The atlas holds the tree on one trunk, the terminal up to the shunts, its ties read off the code.

Every tracked path in a layer of the table and every prefix of it holding one; the trunk from the terminal
to the analog front end, the word 'shunt' found up it; ties between files of every kind, the wire's two
ends among them; a chain of ties across the wire; the trunk, a layer, a folder and a file opened as lines;
the page with its script and the atlas inlined; the terminal's tree in 3D walked by its keys and its mouse
(tools/dev/atlas.py, tools/dev/atlas_tree.py, terminal/views/show_atlas.py, terminal/views/atlas/)."""
import functools
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev import atlas, atlas_tree
from tools.dev.focus import chosen
from structure_kit import Report


@functools.lru_cache(maxsize=1)
def built():
    """The tree's atlas, read once for every test."""
    return atlas_tree.build()


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
    """The trunk runs from the terminal at its foot to the analog front end at its crown, and the word
    'shunt' is found up it: the terminal's drawings, the rig's scaling, the wire, the cores, the board's
    reads of the converters."""
    trunk = [row[0] for row in atlas_tree.LAYERS if row[1] == 'trunk']
    report.check('the trunk runs from the terminal to the analog front end',
                 trunk[0] == 'terminal' and trunk[-1] == 'analog', ' '.join(trunk))
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
    report.check('the trunk lists its layers from the crown to the terminal', names[0] == 'analog'
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
    terminal F1 - drawn in braille and named; F4 opens the executive, its folders level round it and the
    camera eased to frame them, F1 its heaviest folder, its files hanging under it, BKSP back on the layer;
    the panel lists the layers far, the open layer's folders at its frame; a left drag turns the tree within
    its tip, the wheel brings it near; / and a word put its hits on the keys, ESC lets the word go and ESC
    again leaves for the front page."""
    from terminal.ui import bindings, screen
    from terminal.views import show_atlas
    from terminal.views.atlas import panel, scene
    tree = built()
    state = show_atlas.opened(tree)

    def typed(*keys):
        return bindings.act_on(show_atlas.PAD, list(keys), state)[0]

    def drawn():
        """The scene at the ease's end: its lines, the nodes drawn, the view, the node the panel lists."""
        state['cam'].update(target=state['goal']['target'], distance=state['goal']['distance'])
        lines, fell, v = scene.picture(show_atlas.placed(state), state['cam'], 100, 36, show_atlas.keyed(state), set())
        return lines, fell, show_atlas.listing(state, panel.middle(fell, v))

    keys = show_atlas.keyed(state)
    report.check('at the trunk F1 to F12 open its twelve layers, the terminal F1',
                 list(keys) == list(range(len(atlas_tree.LAYERS))) and keys[0] == 'F1', str(keys))
    lines, fell, listed = drawn()
    plain = screen.plain('\n'.join(lines))
    cells, named = sum(0x2800 <= ord(ch) <= 0x28FF for ch in plain), [row[0] for row in atlas_tree.LAYERS if row[0] in plain]
    report.check('the trunk drawn in braille, every layer before the camera and named; far, the panel lists the layers',
                 len(lines) == 36 and cells > 100 and len(fell) == len(named) == len(atlas_tree.LAYERS) and listed == -1,
                 '%d rows, %d braille cells, %d drawn, %d named, listing %d' % (len(lines), cells, len(fell), len(named), listed))
    executive = next(i for i, row in enumerate(atlas_tree.LAYERS) if row[0] == 'executive')
    typed('f%d' % (executive + 1))
    at, folders = show_atlas.placed(state)['at'], state['kids'][executive]
    report.check('F4: the executive open, its folders level round it, the camera eased to frame them',
                 state['focus'] == executive and folders and all(at[f][1] == at[executive][1] for f in folders)
                 and state['goal']['distance'] < scene.camera()['distance'], '%d folders' % len(folders))
    lines, fell, listed = drawn()
    report.check("at its frame the panel lists its folders", listed == executive, tree.id[listed] if listed >= 0 else '-1')
    heaviest = scene.children(tree, state['kids'], state['below'], executive)[0]
    typed('f1')
    at, files = show_atlas.placed(state)['at'], state['kids'][heaviest]
    report.check('F1: its heaviest folder open, its files hanging under it',
                 state['focus'] == heaviest and files and all(at[f][1] < at[heaviest][1] for f in files), tree.id[heaviest])
    typed('\x08')
    report.check('BKSP: back on the layer', state['focus'] == executive, str(state['focus']))
    yaw, near = state['cam']['yaw'], state['cam']['distance']
    show_atlas.turned(state, 10, 100)
    show_atlas.wheeled(state, [], 0.12)
    report.check('a left drag turns the tree and tips it within its bounds; the wheel brings it near',
                 state['cam']['yaw'] == yaw - 10 * show_atlas.DRAG_DEG[0] and state['cam']['pitch'] == show_atlas.TIP_DEG[1]
                 and abs(state['cam']['distance'] * 1.12 - near) < 1e-9, str(state['cam']))
    typed('/', *'shunt')
    hits = show_atlas.context(state)
    report.check('/ and a word: its hits on the keys while it is typed', state['find'] == 'shunt' and state['typing']
                 and len(hits) > 5 and list(show_atlas.keyed(state)) == hits[:show_atlas.PAGE], '%d hits' % len(hits))
    typed('\r')
    first, then = typed('\x1b'), typed('\x1b')
    report.check('ESC lets the word go, ESC again the front page', state['find'] is None and first is None
                 and then == 'menu', '%r then %r' % (first, then))


ROSTER = (test_every_path_lies_in_one_layer, test_the_trunk_climbs_from_the_terminal_to_the_shunts,
          test_ties_join_files, test_a_chain_of_ties_crosses_the_wire, test_each_node_opens_as_lines,
          test_the_page_carries_the_atlas, test_the_terminal_page_walks_the_tree)


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
