#!/usr/bin/env python3
"""The system as one atlas: the trunk from the terminal up to the shunts, opened a layer at a time.

    python -X utf8 host/tools/dev/atlas.py                          # the trunk, the terminal at its foot
    python -X utf8 host/tools/dev/atlas.py executive                # a layer: its folders, its ties
    python -X utf8 host/tools/dev/atlas.py host/machine/law         # a folder: its files
    python -X utf8 host/tools/dev/atlas.py host/coaxial/comm/protocol.py   # a file: its symbols, its ties
    python -X utf8 host/tools/dev/atlas.py executive --depth 2      # two levels open at once
    python -X utf8 host/tools/dev/atlas.py --find shunt             # a word's column up the trunk
    python -X utf8 host/tools/dev/atlas.py --path terminal sensing  # the shortest chain of ties
    python -X utf8 host/tools/dev/atlas.py --html                   # the page, build/atlas.html
    python -X utf8 host/tools/dev/atlas.py --json -                 # the atlas as columns

The tree - the layers, every file on them, their symbols and the ties between files - is
tools/dev/atlas_tree.py's, read from the files as they are; here it is shown: as lines, a node at a time
(+ one to open, . a leaf), or as the page beside this file (atlas.html, its script atlas.js), the trunk
drawn as a stack. The terminal shows it as a tree in 3D on its front page's SYSTEM ATLAS
(terminal/views/show_atlas.py), turned by the mouse and opened by F1 to F12; W there lays this page.
"""
import argparse
import collections
import io
import json
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO
from tools.dev import host_map
from tools.dev.atlas_tree import FILE, KINDS, LAYER, LAYERS, SYMBOL, TIES, build

#: The page's template beside this file; where the page is laid unless told; how much a view names.
PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'atlas.html')
SCRIPT = os.path.join(os.path.dirname(PAGE), 'atlas.js')
OUT = os.path.join(REPO, 'build', 'atlas.html')
SHOWN, PER_LAYER = 12, 20


def tree_of(atlas, kept=None):
    """Each node's children, and how many of each kind lie beneath it, as columns - of the nodes `kept`
    (a truth a node; all where None): one not kept is no one's child, nor counted, nor what lies in it."""
    kids, below = [[] for _ in atlas.id], [[0] * 4 for _ in atlas.id]
    for i in range(len(atlas.id) - 1, -1, -1):
        up = atlas.up[i]
        if up >= 0 and (kept is None or kept[i]):
            kids[up].append(i)
            below[up][atlas.kind[i]] += 1
            below[up] = [a + b for a, b in zip(below[up], below[i])]
    for row in kids:
        row.reverse()
    return kids, below


def owners(atlas):
    """Each node's layer, folder and file (-1 above it), as rows."""
    out = []
    for i, (up, kind) in enumerate(zip(atlas.up, atlas.kind)):
        row = list(out[up]) if up >= 0 else [-1, -1, -1]
        if kind < SYMBOL:
            row[kind] = i
        out.append(row)
    return out


def ties_of(atlas, i, own):
    """({(other end, tie): count} leaving node i, the same reaching it), the other end taken at i's level;
    a symbol's are its file's."""
    i = atlas.up[i] if atlas.kind[i] == SYMBOL else i
    level, out, into = atlas.kind[i], collections.Counter(), collections.Counter()
    for s, d, t in zip(atlas.src, atlas.dst, atlas.tie):
        a, b = own[s][level], own[d][level]
        if a == i and b != i:
            out[(b, t)] += 1
        elif b == i and a != i:
            into[(a, t)] += 1
    return out, into


def _said(atlas, ties):
    total, kinds = collections.Counter(), collections.defaultdict(set)
    for (node, tie), n in ties.items():
        total[node] += n
        kinds[node].add(TIES[tie])
    said = ['%s %d %s' % (atlas.id[n], c, '/'.join(sorted(kinds[n]))) for n, c in total.most_common(SHOWN)]
    more = len(total) - SHOWN
    return ', '.join(said) + (', %d more' % more if more > 0 else '') or 'none'


def order():
    """(number, layer) from the trunk's crown down to its foot, then those beside it, unnumbered."""
    trunk = [i for i, row in enumerate(LAYERS) if row[1] == 'trunk']
    return [(str(n), i) for n, i in reversed(list(enumerate(trunk, 1)))] + \
        [('', i) for i in range(len(LAYERS)) if i not in trunk]


def commit():
    """The tree's commit, short, and how many tracked files the working tree has changed since: the atlas
    reads the files as they stand, not as the commit holds them."""
    def git(*args):
        return subprocess.run(('git',) + args, cwd=REPO, capture_output=True, text=True).stdout
    return git('rev-parse', '--short', 'HEAD').strip(), len(git('status', '--porcelain', '-uno').splitlines())


def told(at):
    """A commit as said: '7ae76b6d', or '7ae76b6d + 10 files changed' over a working tree that differs."""
    head, changed = at
    return head + (' + %d file%s changed' % (changed, '' if changed == 1 else 's') if changed else '')


def show_trunk(atlas, depth=1):
    """The trunk's lines: each layer from the crown down to the foot, then beside it, with what it holds,
    the layers it reaches (->) and those reaching it (<-); `depth` past 1 opens each layer that far - at
    2 the whole map's folders, each with its purpose."""
    kids, below = tree_of(atlas)
    own, stack = owners(atlas), order()
    trunk = sum(1 for n, _ in stack if n)
    lines = ['%s at %s: %d layers on the trunk, the terminal at its foot (1), the shunts at its crown (%d).'
             % (os.path.basename(REPO), told(commit()), trunk, trunk),
             '%d files, %d symbols, %d ties. Open one: atlas.py NODE [--depth N]; --find WORD; --path A B; '
             '--html; --json PATH' % (atlas.kind.count(FILE), atlas.kind.count(SYMBOL), len(atlas.src))]
    for n, i in stack:
        if not n and i == stack[trunk][1]:
            lines.append('beside the trunk:')
        out, into = ties_of(atlas, i, own)
        lines.append('%2s %-9s %3d folders %5d files  %s' % (n, LAYERS[i][0], len(kids[i]), below[i][FILE],
                                                              LAYERS[i][2]))
        lines.append('%14s-> %s' % ('', _said(atlas, out)))
        lines.append('%14s<- %s' % ('', _said(atlas, into)))
        _open(atlas, i, depth - 1, kids, below, lines, ' ' * 14)
    return lines


def _head(atlas, i, below, own):
    kind, brief = atlas.kind[i], atlas.brief[i]
    where = '' if kind == LAYER else ' in %s' % atlas.id[own[i][LAYER]]
    if kind == SYMBOL:
        return '%s (symbol, line %d%s): %s%s' % (atlas.id[i], atlas.line[i], where, atlas.label[i],
                                                 ' - ' + brief if brief else '')
    held = ', '.join(counted(below[i][k], KINDS[k]) for k in range(kind + 1, 4) if below[i][k])
    return '%s (%s%s; %s): %s' % (atlas.id[i], KINDS[kind], where, held or 'nothing beneath', brief or '-')


def counted(n, kind):
    """'1 file', '2 files': `n` of a `kind`."""
    return '%d %s%s' % (n, kind, '' if n == 1 else 's')


def _open(atlas, i, depth, kids, below, lines, indent):
    """Node i's children, `depth` levels of them: + one to open, . a leaf."""
    for k in kids[i] if depth > 0 else ():
        brief = ' - ' + atlas.brief[k] if atlas.brief[k] else ''
        if atlas.kind[k] == SYMBOL:
            lines.append('%s. %s :%d%s' % (indent, atlas.label[k], atlas.line[k], brief))
        else:
            count = below[k][FILE] if atlas.kind[k] < FILE else below[k][SYMBOL]
            lines.append('%s%s %s (%d)%s' % (indent, '+' if kids[k] else '.', atlas.id[k], count, brief))
        _open(atlas, k, depth - 1, kids, below, lines, indent + '  ')


def show_node(atlas, i, depth=1):
    """A node's lines: what it is, its children `depth` levels down, the ties leaving and reaching it."""
    kids, below = tree_of(atlas)
    own = owners(atlas)
    lines = [_head(atlas, i, below, own)]
    _open(atlas, i, depth, kids, below, lines, '  ')
    out, into = ties_of(atlas, i, own)
    return lines + ['  -> ' + _said(atlas, out), '  <- ' + _said(atlas, into)]


def find_node(atlas, key):
    """The nodes a command line names - a layer's key, a folder's path with or without its slash, a file's
    path, path::symbol - a folder split between layers in each part; [] where none."""
    key = key.replace('\\', '/').strip()
    for want in (key, key.rstrip('/') + '/'):
        hits = [i for i, name in enumerate(atlas.id) if re.sub(r'(@[\w-]+|#\d+)$', '', name) == want]
        if hits:
            return hits
    return []


def found(atlas, word, own=None):
    """{layer: [(node, named)]}: the nodes whose name, signature or brief holds `word` (named), and the files
    whose text does, in table order; `own` the atlas's owners where already laid."""
    low, own, hits = word.lower(), own or owners(atlas), collections.defaultdict(list)
    for i, (key, label, brief) in enumerate(zip(atlas.id, atlas.label, atlas.brief)):
        named = low in key.lower() or low in label.lower() or low in brief.lower()
        if named or (atlas.kind[i] == FILE and low in atlas.lowered(key)):
            hits[own[i][LAYER]].append((i, named))
    return hits


def show_find(atlas, word):
    """A word's column: its hits a layer at a time, from the trunk's crown down to its foot, then beside."""
    hits, stack = found(atlas, word), order()
    on = [i for n, i in stack if n and i in hits]
    lines = ["'%s' on %d of the trunk's %d layers and %d beside it, %d nodes"
             % (word, len(on), sum(1 for n, _ in stack if n), len(hits) - len(on), sum(map(len, hits.values())))]
    for n, i in stack:
        lines += ['%2s %s (%d)' % (n, LAYERS[i][0], len(hits[i]))] if i in hits else []
        for node, named in hits.get(i, [])[:PER_LAYER]:
            lines.append('      %s%s' % (atlas.id[node], '' if named else '  (in its text)'))
        if len(hits.get(i, [])) > PER_LAYER:
            lines.append('      ... %d more' % (len(hits[i]) - PER_LAYER))
    return lines


def _files(atlas, key, own):
    """The files a command line's node holds: a layer's, a folder's, a file, a symbol's file."""
    out = set()
    for i in find_node(atlas, key):
        i = atlas.up[i] if atlas.kind[i] == SYMBOL else i
        out.update(f for f, row in enumerate(own) if atlas.kind[f] == FILE and row[atlas.kind[i]] == i)
    return out


def show_path(atlas, a, b):
    """The shortest chain of ties from a to b, along or against each, through the system's layers (the
    workshop and the writing only where a or b lies)."""
    own = owners(atlas)
    starts, ends = _files(atlas, a, own), _files(atlas, b, own)
    if not starts or not ends:
        return ['no node %s' % (b if starts else a)]
    near = collections.defaultdict(list)
    for s, d, t in zip(atlas.src, atlas.dst, atlas.tie):
        near[s].append((d, '-%s->' % TIES[t]))
        near[d].append((s, '<-%s-' % TIES[t]))
    came: dict[int, tuple[int, str] | None] = {i: None for i in starts}
    queue, hit = collections.deque(sorted(starts)), None
    while queue and hit is None:
        i = queue.popleft()
        hit = i if i in ends else None
        for j, way in near[i] if hit is None else ():
            if j not in came and (j in ends or LAYERS[own[j][LAYER]][1] != 'about'):
                came[j] = (i, way)
                queue.append(j)
    if hit is None:
        return ['no chain of ties from %s to %s' % (a, b)]
    chain, step = [], came[hit]
    while step is not None:
        chain.append('  %s %s (%s)' % (step[1], atlas.id[hit], LAYERS[own[hit][LAYER]][0]))
        hit, step = step[0], came[step[0]]
    return ['%s (%s)' % (atlas.id[hit], LAYERS[own[hit][LAYER]][0])] + chain[::-1]


def data(atlas):
    """The atlas as JSON's columns, with what the page names its source by."""
    return {'repo': os.path.basename(REPO), 'commit': told(commit()), 'root': REPO.replace(os.sep, '/'),
            'layers': [list(row[:3]) for row in LAYERS], 'kinds': KINDS, 'ties': TIES,
            'id': atlas.id, 'up': atlas.up, 'kind': atlas.kind, 'label': atlas.label, 'brief': atlas.brief,
            'line': atlas.line, 'src': atlas.src, 'dst': atlas.dst, 'tie': atlas.tie}


def page(atlas, out=OUT):
    """Lay the page - the template beside this file, its script (atlas.js) and the atlas inlined, one file
    a browser opens from anywhere - and say where."""
    with open(PAGE, encoding='utf-8') as f:
        template = f.read()
    with open(SCRIPT, encoding='utf-8') as f:
        script = f.read().replace('</', '<\\/')
    blob = json.dumps(data(atlas), separators=(',', ':')).replace('</', '<\\/')
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, 'w', encoding='utf-8', newline='') as f:
        f.write(template.replace('/*__SCRIPT__*/', script, 1).replace('"__ATLAS__"', blob, 1))
    return out


def main(argv=None):
    """The atlas as asked: the trunk, a node, a word's column, a chain of ties, the page, the columns."""
    parser = argparse.ArgumentParser(description=host_map.brief(__doc__))
    parser.add_argument('node', nargs='?', help='a layer, a folder or a file from the root, or path::symbol')
    parser.add_argument('--depth', type=int, default=1, help='how many levels open below the node')
    parser.add_argument('--find', metavar='WORD', help="a word's column: names, signatures, briefs, texts")
    parser.add_argument('--path', nargs=2, metavar=('A', 'B'), help='the shortest chain of ties from A to B')
    parser.add_argument('--html', nargs='?', const=OUT, metavar='PATH', help='lay the page (build/atlas.html)')
    parser.add_argument('--json', metavar='PATH', help="the atlas's columns ('-' prints them)")
    args = parser.parse_args(argv)
    if isinstance(sys.stdout, io.TextIOWrapper):   # a cp1252 pipe cannot carry every brief
        sys.stdout.reconfigure(encoding='utf-8')
    atlas = build()
    if args.html:
        print(page(atlas, args.html))
    if args.json:
        blob = json.dumps(data(atlas), separators=(',', ':'))
        if args.json == '-':
            print(blob)
        else:
            with open(args.json, 'w', encoding='utf-8', newline='') as f:
                f.write(blob)
    if args.html or args.json:
        return 0
    nodes = find_node(atlas, args.node) if args.node else []
    if args.node and not nodes:
        print('no node %s: a layer (%s), a folder, a file or path::symbol' % (
            args.node, ', '.join(row[0] for row in LAYERS)), file=sys.stderr)
        return 1
    lines = (show_find(atlas, args.find) if args.find else show_path(atlas, *args.path) if args.path
             else [line for i in nodes for line in show_node(atlas, i, args.depth)] if nodes
             else show_trunk(atlas, args.depth))
    print('\n'.join(lines))
    return 0


if __name__ == '__main__':
    sys.exit(main())
