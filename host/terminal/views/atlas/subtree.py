"""A node opened in the applications' tree grows its subtree where it stands: the same tree, inherited.

Each node's children part along branches as the pages do (growth.nest), by all they share: what their code
reaches - a Python symbol's calls, a C function's by its body, a declaration's by its definition's, a file's
ties to what it imports, includes, names or answers on the wire -, within the node all the way, beyond it the
files all the way; and the words their names begin with ('world_sto_init', 'world_sto_step'). Each child holds
itself and all under it, a branch its children; grown as the tree is (growth.grow): the child or the branch
holding most leading straight on, the rest round the stem by the golden angle at Murray's angles, down to the
leaves, a branch named by the heaviest thing its children share.
"""
import functools
import os
import re

from terminal.views.atlas import growth, scene
from tools.dev import atlas, target_map
from tools.dev.atlas_tree import FILE, FOLDER, REPO


@functools.lru_cache(maxsize=2)
def holding(tree):
    """(kids, below) of the atlas's tree, its plumbing out (atlas.tree_of): what a node opened holds."""
    return atlas.tree_of(tree, [not h for h in atlas.plumbing(tree)])


def held(tree, node):
    """What `node` holds: itself and all under it."""
    return 1 + sum(holding(tree)[1][node])


@functools.lru_cache(maxsize=2)
def called(tree):
    """{node: the nodes it reaches directly}: its code's calls - a Python symbol's, a file's its module's and its
    symbols' (tree.calls); a C function's by its body, a declaration's by its definition's, over every C file the
    tree holds (target_map.bodies, CALL), a src/ folder's definition first -, and a file's ties."""
    out = {}
    for key, targets in tree.calls.items():
        nodes = {tree.at[t] for t in targets if t in tree.at}
        for node in {tree.at.get(key, -1), tree.at.get(key.partition('::')[0], -1)} - {-1}:
            out.setdefault(node, set()).update(nodes)
    bodies = {}
    for path in sorted(p for p in tree.at if p.endswith('.c') and '::' not in p):
        with open(os.path.join(REPO, path), encoding='utf-8', errors='replace') as f:
            bodies[path] = target_map.bodies(f.read())
    defines = {}
    for path in sorted(bodies, key=lambda p: ('/src/' not in p, p)):
        defines.update((name, path) for name in bodies[path] if name not in defines)
    for ident, node in tree.at.items():
        path, _, name = ident.partition('::')
        if path.endswith(('.c', '.h')) and name in defines:
            reached = {tree.at.get(defines[c] + '::' + c, -1) for c in target_map.CALL.findall(bodies[defines[name]][name])
                       if c in defines and c != name} - {-1}
            for n in (node, tree.at[path]):
                out.setdefault(n, set()).update(reached)
    for s, d in zip(tree.src, tree.dst):
        out.setdefault(s, set()).add(d)
    return out


def _closure(start, edges):
    """All `edges` {node: nodes} lead to from `start`, on and on."""
    seen, todo = set(), list(start)
    while todo:
        for n in edges.get(todo.pop(), ()):
            if n not in seen:
                seen.add(n)
                todo.append(n)
    return seen


@functools.lru_cache(maxsize=2)
def reaching(tree):
    """{file: the files it reaches all the way}: the files each file's code reaches (`called`), and theirs, on."""
    direct = {}
    for node, targets in called(tree).items():
        f = file_of(tree, node)
        if f is not None:
            direct.setdefault(f, set()).update({file_of(tree, t) for t in targets} - {f, None})
    return {f: frozenset(_closure([f], direct)) for f in direct}


def words(tree, node):
    """What `node` declares in words - its name's (a symbol's, a file's or a folder's but its ending), a symbol's
    parameters' -: each word, and the runs of them its name begins with; what words shared tell of kin."""
    ident = tree.id[node]
    runs = [w for w in re.split(r'[^A-Za-z0-9]+', ident.partition('::')[2]
                                or ident.rstrip('/').rsplit('/', 1)[-1].rsplit('.', 1)[0]) if w]
    said = re.findall(r'[A-Za-z_]\w*', tree.label[node].partition('(')[2]) if '::' in ident else []
    return {('name', tuple(runs[:k])) for k in range(1, len(runs) + 1)} | {('word', w) for w in runs + said}


def under(tree, node):
    """`node` and all under it."""
    kids, out = holding(tree)[0], [node]
    for n in out:
        out += kids[n]
    return out


def file_of(tree, node):
    """The file `node` lies in - itself where it is one -, None where it lies in none."""
    while node >= 0 and tree.kind[node] > FILE:
        node = tree.up[node]
    return node if node >= 0 and tree.kind[node] == FILE else None


def shared(tree, node):
    """{thing: `node`'s children sharing it}: what each child's code reaches (`called`) - within `node` all the
    way, beyond it the files all the way (`reaching`); a folder's children by the files alone -, and the words
    their names begin with (`words`)."""
    kids, edges, far, uses = holding(tree)[0], called(tree), reaching(tree), {}
    home = set(under(tree, node))
    local = {n: edges.get(n, set()) & home for n in home}
    for child in kids[node]:
        mine = set(under(tree, child))
        direct = set().union(*(edges.get(n, ()) for n in mine))
        inner = _closure(direct & home, local) | (direct & home)
        files = {file_of(tree, x) for x in direct - home} - {None}
        beyond = set().union(files, *(far.get(f, ()) for f in files))
        reached = {file_of(tree, x) for x in inner} | beyond if tree.kind[node] <= FOLDER \
            else inner | (direct - home) | beyond
        for thing in {('node', x) for x in reached - mine - {None}} | words(tree, child):
            uses.setdefault(thing, set()).add(child)
    return uses


def nesting(tree, node, first):
    """growth.nest of `node`'s children by all they share (`shared`), a thing weighed by what it holds, a word as
    one; its branches from `first` down."""
    return growth.nest(holding(tree)[0][node], shared(tree, node),
                       lambda thing: held(tree, thing[1]) if thing[0] == 'node' else 1, first)


@functools.lru_cache(maxsize=8)
def grown(tree, root, origin, first):
    """({node: where it stands}, (from, to, node, whether it leads) - its boughs' pieces (growth.pieces), arrays
    a piece a row -, {node: its kids}, {node: what it hangs from}, {branch: its name}, {node: its bough - the
    root's its shoot}): the subtree `root` grows out of `origin` straight up, as the tree's trunk out of the
    terminal - each node's children nested (`nesting`), a branch an id from `first` down -, grown as the tree is
    (growth.grow)."""
    from coaxial.model.blocks import numpy as np
    kids = holding(tree)[0]
    nested, up, members, names, todo = {}, {}, {}, {}, [root]
    for node in todo:
        n = nesting(tree, node, first - len(members))
        nested.update({b if b is not None else node: ks for b, ks in n['kids'].items()})
        up.update((x, node if p is None else p) for x, p in n['up'].items())
        members.update(n['held'])
        names.update((b, scene.short(tree, x) if kind == 'node' else ' '.join(x) if kind == 'name' else x)
                     for b, ((kind, x), *_rest) in n['by'].items())
        todo += [c for c in kids[node] if kids[c]]

    def hold(x):
        return sum(held(tree, i) for i in members[x]) if x in members else held(tree, x)
    stem = (origin, (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), 0.0, growth.APART * max(1, len(nested[root])), True)
    out = {root: stem}
    growth.grow(stem, root, nested, hold, lambda n: max(1, len(nested.get(n, ()))), sum(map(hold, nested[root])),
                [0], out)
    parts = growth.pieces(out)
    rows = (np.array([p[0] for p in parts], float).reshape(-1, 3), np.array([p[1] for p in parts], float).reshape(-1, 3),
            np.array([p[2] for p in parts], np.intp), np.array([p[3] for p in parts], bool))
    return {n: growth.along(b, 1.0)[0] for n, b in out.items() if n != root}, rows, nested, up, names, out
