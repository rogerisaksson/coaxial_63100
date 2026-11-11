#!/usr/bin/env python3
"""The atlas holds the tree on one trunk, the terminal up to the shunts, its ties read off the code.

Every tracked path in a layer of the table and every prefix of it holding one; the trunk from the terminal
to the analog front end, the word 'shunt' found up it; ties between files of every kind, the wire's two
ends among them; a chain of ties across the wire; the trunk, a layer, a folder and a file opened as lines;
the page with the atlas inlined; the terminal's page walked by its keys (tools/dev/atlas.py,
tools/dev/atlas_tree.py, terminal/views/show_atlas.py)."""
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


def test_the_page_carries_the_atlas(report):
    """--html: the template beside atlas.py with the atlas inlined - every node and tie -, ASCII, nothing
    left to fill."""
    tree = built()
    with tempfile.TemporaryDirectory() as folder:
        with open(atlas.page(tree, os.path.join(folder, 'atlas.html')), encoding='utf-8') as f:
            text = f.read()
    data = json.loads(text.split('type="application/json">', 1)[1].split('</script>', 1)[0])
    report.check('the page inlines every node and tie', len(data['id']) == len(tree.id)
                 and len(data['src']) == len(tree.src), '%d nodes, %d ties' % (len(data['id']), len(data['src'])))
    report.check('and it is ASCII with nothing left to fill', text.isascii() and '__ATLAS__' not in text, '')


def test_the_terminal_page_walks_the_tree(report):
    """terminal/views/show_atlas.py on its state: the trunk in braille, its strip for a small window; DOWN
    and ENTER open the second layer from the crown, ENTER its first folder, BKSP back with the cursor on it;
    / and a word list its hits, ESC lets the word go and ESC again leaves for the front page."""
    from terminal.ui import bindings, screen
    from terminal.views import show_atlas
    state = show_atlas.opened(built())

    def typed(*keys):
        return bindings.act_on(show_atlas.PAD, list(keys), state)[0]

    def braille(line):
        return sum(0x2800 <= ord(ch) <= 0x28FF for ch in screen.plain(line))

    slabs = [line for line in show_atlas.trunk_lines(state)[:-1] if braille(line)]
    report.check('the trunk is drawn in braille, two rows a layer, the strip two cells a layer',
                 len(slabs) == 2 * len(atlas_tree.LAYERS) and braille(show_atlas.strip(state)) == 2 * len(atlas_tree.LAYERS),
                 '%d rows, %d cells' % (len(slabs), braille(show_atlas.strip(state))))
    second = [layer for _n, layer in atlas.order()][1]
    typed('down', '\r')
    report.check('DOWN, ENTER: the second layer from the crown open', state['cur'] == second, str(state['cur']))
    typed('\r')
    folder = state['cur']
    typed('\x08')
    report.check('ENTER a folder, BKSP: back on its layer, the cursor on it', state['cur'] == second
                 and show_atlas.items(state)[show_atlas.cursor(state)] == folder, str(state['cur']))
    typed('/', *'shunt')
    report.check('/ and a word: its hits listed while it is typed', state['find'] == 'shunt' and state['typing']
                 and len(show_atlas.items(state)) > 5, '%d hits' % len(show_atlas.items(state)))
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
