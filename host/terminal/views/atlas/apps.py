"""The terminal's applications on the branches they part along, each page's subsystems up the trunk.

Each page of the terminal (terminal/pages/: its HEADLINE, its KEY) is an application; what it comes to is
what its `run` names, followed through the atlas's ties from symbol to symbol (tools/dev/atlas_calls.py) -
not a module's every import: a page asking one function of a module takes in none of its other users -,
and the C a file it comes to names or answers on the wire (`systems`). A folder is a subsystem of the pages
that come to it, of one alone or shared (`served`). The pages part along branches (`clades`): each set of
pages a folder serves alone, those sharing the most code first, kept where it nests in those kept, named by
its heaviest folder - aircraft/ the flyers'. `placed` lays the scene the atlas draws at the terminal
(scene.picture): the trunk as the tree has it, the terminal at its foot the root, the branches out from it
and down, the more pages one holds the nearer the trunk, the pages round their rim; each subsystem up at its
layer's height, toward the pages it serves, the farther out the fewer - a page's own out on its branch,
those all share on the trunk -; each page's way up through its subsystems, each hung from the nearest of its
own below. The node open lit - a branch with its pages, a page with its subsystems -, its keys `offered`;
`reach` what the mirror (atlas/mirror.py) keeps to.
"""
import ast
import functools
import math
import os

from terminal.views.atlas import nested, scene
from tools.dev import atlas_calls
from tools.dev.atlas_tree import FILE, LAYERS, REPO, TIES

#: Where the terminal's pages are; how far out a subsystem of one page stands and a page itself, how far
#: under the terminal the pages stand, world units; a subsystem's disc and its growth a page it serves, a
#: page's, a branch's; the room between subsystems that would stand as one.
PAGES, OUT, RING, BELOW = 'host/terminal/pages/', 2.6, 3.0, 3.0
DISC, GROWTH, PAGE_DISC, FORK_DISC, APART = 0.04, 0.008, 0.08, 0.06, 0.22


@functools.lru_cache(maxsize=2)
def pages(tree):
    """[(node, headline, key, what)] of the terminal's pages, as their files bind HEADLINE, KEY and WHAT."""
    out = []
    for path in sorted(p for p in tree.at if p.startswith(PAGES) and p.endswith('.py') and '::' not in p):
        with open(os.path.join(REPO, path), encoding='utf-8') as f:
            body = ast.parse(f.read()).body
        said = {}
        for node in body:
            if isinstance(node, ast.Assign) and isinstance(node.value, (ast.Tuple, ast.Constant)):
                names = node.targets[0].elts if isinstance(node.targets[0], ast.Tuple) else node.targets
                values = node.value.elts if isinstance(node.value, ast.Tuple) else [node.value]
                said.update((n.id, getattr(v, 'value', None)) for n, v in zip(names, values) if isinstance(n, ast.Name))
        if 'HEADLINE' in said:
            out.append((tree.at[path], said['HEADLINE'], said.get('KEY', ''), said.get('WHAT', '')))
    return out


@functools.lru_cache(maxsize=2)
def systems(tree):
    """{page: the files it comes to}: what its `run` names, on (atlas_calls.reach), and the C a file of
    them names or answers on the wire."""
    into = {}
    for s, d, t in zip(tree.src, tree.dst, tree.tie):
        if TIES[t] in ('names', 'wire') and tree.id[s].endswith('.py') and tree.id[d].endswith(('.c', '.h')):
            into.setdefault(s, set()).add(d)
    out = {}
    for page, *_said in pages(tree):
        files = {tree.at[k.partition('::')[0]] for k in atlas_calls.reach(tree, [tree.id[page] + '::run'])
                 if k.partition('::')[0] in tree.at}
        out[page] = frozenset(files | {d for f in files for d in into.get(f, ())})
    return out


def folder(tree, node):
    """The folder `node` lies in (itself where it is one)."""
    while node >= 0 and tree.kind[node] >= FILE:
        node = tree.up[node]
    return node


def layer(tree, node):
    """The layer `node` stands on."""
    while tree.up[node] >= 0:
        node = tree.up[node]
    return node


@functools.lru_cache(maxsize=2)
def served(tree):
    """{folder: the pages it serves}: those that come to a file of it (`systems`)."""
    out = {}
    for page, files in systems(tree).items():
        for f in files:
            out.setdefault(folder(tree, f), set()).add(page)
    return {f: frozenset(p) for f, p in out.items() if layer(tree, f) != f}


@functools.lru_cache(maxsize=2)
def clades(tree):
    """The branches the pages part along, out from the terminal: each set of pages a folder serves alone
    (`served`) - two or more, not all -, the most code shared first (its folders' files), kept where it nests
    in those kept. {'up': {branch or page: the branch it hangs from, None the terminal}, 'kids': {None or a
    branch: its pages, then its branches, the most pages first}, 'pages': {branch: its pages}, 'by': {branch:
    its folders, the heaviest first}}; a branch an id from -2 down, the one holding most first."""
    users, every = served(tree), len(pages(tree))
    files, sets = {}, {}
    for node, kind in enumerate(tree.kind):
        if kind == FILE:
            files[tree.up[node]] = files.get(tree.up[node], 0) + 1
    for f, ps in users.items():
        if 1 < len(ps) < every:
            sets.setdefault(ps, []).append(f)
    kept = []
    for s in sorted(sets, key=lambda s: (-sum(files.get(f, 0) for f in sets[s]), -len(s), sorted(s))):
        if all(s <= k or k <= s or not s & k for k in kept):
            kept.append(s)
    kept.sort(key=lambda s: (-len(s), sorted(s)))
    ids = {s: -2 - k for k, s in enumerate(kept)}
    up = {ids[s]: next((ids[k] for k in reversed(kept) if s < k), None) for s in kept}
    up.update((page, next((ids[k] for k in reversed(kept) if page in k), None)) for page, *_said in pages(tree))
    kids = {None: [], **{b: [] for b in ids.values()}}
    for node in sorted(up, key=lambda n: (n < 0, -len(kept[-2 - n]) if n < 0 else n)):
        kids[up[node]].append(node)
    return {'up': up, 'kids': kids, 'pages': {ids[s]: s for s in kept},
            'by': {ids[s]: sorted(sets[s], key=lambda f: (-files.get(f, 0), f)) for s in kept}}


@functools.lru_cache(maxsize=2)
def ordered(tree):
    """The pages round the terminal: its branches' leaves in order, a branch's pages side by side."""
    kids = clades(tree)['kids']

    def leaves(node):
        return [p for k in kids[node] for p in (leaves(k) if k < -1 else [k])]
    return leaves(None)


def offered(tree, app):
    """The nodes the keys open at `app` - None the terminal: its branches' -, a branch's pages and branches,
    a page's subsystems, its own first."""
    if app is None or app < -1:
        return list(clades(tree)['kids'][app])
    users = served(tree)
    return sorted((f for f, ps in users.items() if app in ps), key=lambda f: (len(users[f]), f))


def reach(tree, app):
    """The files `app` comes to - a page's (`systems`), a branch's pages' -, None for none chosen: what the
    mirror keeps to."""
    if app is None:
        return None
    return frozenset().union(*(systems(tree)[p] for p in clades(tree)['pages'].get(app, (app,))))


def placed(tree, upright, chosen=None):
    """The scene the atlas draws at the terminal by application (scene.picture), over `upright` the tree's
    own (its layers where they stand, its stubs), `chosen` the branch or page open (None: the terminal):
    `at` {node: where it stands}, `flat` its nodes and its branches, `rings` {page: its subsystems, its own
    first}, `names` {node: its name} - a page its headline, a branch its heaviest folder's, a folder its own
    or its parent's with it where two share one -, `briefs` - a branch's folders, what a page is - and
    `groups` their order."""
    from coaxial.model.blocks import numpy as np
    c, users, order = clades(tree), served(tree), ordered(tree)
    lay, every, root = upright['at'], len(order), 0
    angle = {page: 2 * math.pi * k / every for k, page in enumerate(order)}
    far = {**{b: (every - len(ps)) / max(1, every - 1) for b, ps in c['pages'].items()}, **dict.fromkeys(order, 1.0)}
    place = {n: lay[n] for n in range(len(LAYERS))}
    for n, k in far.items():   # the more pages a branch holds, the nearer the trunk, round toward them
        ps = c['pages'].get(n, (n,))
        a = math.atan2(sum(math.sin(angle[p]) for p in ps), sum(math.cos(angle[p]) for p in ps))
        x, y, z = lay[root]
        place[n] = (x + RING * k * math.cos(a), y - BELOW * k, z + RING * k * math.sin(a))
    spots = {}
    for f, ps in sorted(users.items()):
        spots.setdefault((layer(tree, f), ps), []).append(f)
    for (lay_, ps), fs in spots.items():   # toward the pages it serves, the farther out the fewer
        x, z = sum(math.cos(angle[p]) for p in ps), sum(math.sin(angle[p]) for p in ps)
        r, a = OUT * (every - len(ps)) / max(1, every - 1), math.atan2(z, x)
        for j, f in enumerate(fs):   # those that would stand as one spread along the way round, APART
            off = (j - (len(fs) - 1) / 2) * APART
            aa, rr = (a + off / r, r) if r > APART else (2 * math.pi * j / len(fs), APART * (len(fs) > 1))
            place[f] = (lay[lay_][0] + rr * math.cos(aa), lay[lay_][1], lay[lay_][2] + rr * math.sin(aa))
    held = c['pages'].get(chosen, frozenset(order) if chosen is None else {chosen})   # the pages open
    path, node = [], chosen
    while node is not None:
        path, node = [node] + path, c['up'][node]
    lit = {root, *path, *held, *(b for b, ps in c['pages'].items() if ps <= held),
           *(f for f, ps in users.items() if ps & held and chosen is not None)}
    trunk = [i for i, row in enumerate(LAYERS) if row[1] == 'trunk']
    parts = [(place[a], place[b], scene.EDGE if a in lit and b in lit else scene.DIM_EDGE, True)
             for a, b in zip(trunk, trunk[1:])]
    parts += [(p, place[n], scene.EDGE if n in lit else scene.DIM_EDGE, False) for p, n in upright['stubs']]
    for n in range(len(LAYERS)):   # a layer's ring round it, as the tree draws it
        x, y, z = place[n]
        ring = [(x + scene.HALO * cs, y, z + scene.HALO * sn) for cs, sn in scene.TURNS]
        parts += [(p, q, scene.EDGE if n in lit else scene.DIM_EDGE, False) for p, q in zip(ring, ring[1:])]
    parts += [(place[root if c['up'][n] is None else c['up'][n]], place[n], scene.EDGE if n in lit else scene.DIM_EDGE,
               True) for n in far]
    for page in order:   # its way up: each subsystem from the nearest of its own below
        mine = sorted((f for f, ps in users.items() if page in ps), key=lambda f: place[f][1])
        for k, f in enumerate(mine):
            below = [page] + [g for g in mine[:k] if place[g][1] < place[f][1]]
            under = min(below, key=lambda g: sum((a - b) ** 2 for a, b in zip(place[g], place[f])))
            parts.append((place[under], place[f], scene.EDGE if page in held and chosen is not None
                          else scene.DIM_EDGE, False))
    focus, forks = root if chosen is None else chosen, sorted(c['pages'], reverse=True)
    nodes = list(range(len(LAYERS))) + forks + order + sorted(users)
    ink = [scene.OPEN if n == focus else scene.NODE if n in lit else scene.DIM_NODE for n in nodes]
    disc = [scene.DISC[0]] * len(LAYERS) + [FORK_DISC] * len(forks) + [PAGE_DISC] * every \
        + [DISC + GROWTH * len(users[f]) for f in sorted(users)]
    ends = np.array([s[0] for s in parts] + [s[1] for s in parts], float).reshape(2, -1, 3)
    names = {**nested.names(tree, sorted(users)), **{page: headline for page, headline, *_said in pages(tree)}}
    names.update((b, names[fs[0]]) for b, fs in c['by'].items())
    keys = offered(tree, chosen)
    return {'tree': tree, 'focus': focus, 'at': place, 'out': {}, 'into': {}, 'names': names,
            'rings': {page: offered(tree, page) for page in order},
            'briefs': {**{b: 'theirs alone: ' + ' '.join(names[f] for f in fs) for b, fs in c['by'].items()},
                       **{page: what for page, _headline, _key, what in pages(tree) if what}},
            'groups': ([focus] + keys + path + [root],   # the keys' panels between
                       forks + order + list(range(len(LAYERS))) + sorted(users, key=lambda f: (-len(users[f]), f))),
            'flat': {'nodes': np.array(nodes, np.intp), 'pos': np.array([place[n] for n in nodes], float).reshape(-1, 3),
                     'disc': np.array(disc, float), 'ink': np.array(ink, np.uint8), 'lit': lit, 'from': ends[0],
                     'to': ends[1], 'segink': np.array([s[2] for s in parts], np.uint8),
                     'thick': np.array([s[3] for s in parts], bool)}}


def framed(scene_):
    """(target, distance) that frames what the scene (`placed`) lights - the terminal its branches, a branch
    its own and their subsystems, a page its subsystems -: its middle, as far as its widest."""
    lit = scene_['flat']['lit']
    xs, ys, zs = zip(*(p for n, p in scene_['at'].items() if n in lit))
    middle = ((max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, (max(zs) + min(zs)) / 2)
    return middle, 1.2 * max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)) + 1.0
