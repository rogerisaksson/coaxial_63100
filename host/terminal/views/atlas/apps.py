"""The atlas by application: the terminal out to its pages, each one's subsystems, its own and shared.

Each page of the terminal (terminal/pages/: its HEADLINE, its KEY) is an application; what it comes to is
what its `run` names, followed through the atlas's ties from symbol to symbol (tools/dev/atlas_calls.py) -
not a module's every import: a page asking one function of a module takes in none of its other users -,
and the C a file it comes to names or answers on the wire (`systems`). A folder is a subsystem of the pages
that come to it, of one alone or shared (`served`). `placed` lays the scene the atlas draws (scene.picture):
the terminal at the foot, its pages round it, the pages that share most side by side; each subsystem up at
its layer's height, toward the pages it serves, the farther out the fewer - a page's own out on its branch,
those all share on the trunk -; each page's branch from the terminal out to it and up through its
subsystems, each hung from the nearest of its own below. The page chosen lit, its branch, its subsystems
listed (its `rings`), its own first; `reach` what the mirror (atlas/mirror.py) keeps to.
"""
import ast
import functools
import math
import os

from terminal.views.atlas import nested, scene
from tools.dev import atlas_calls
from tools.dev.atlas_tree import FILE, REPO, TIES

#: Where the terminal's pages are; how far out a subsystem of one page stands and a page itself, how far
#: under the terminal's layer the pages stand and the terminal at their foot, world units; a subsystem's
#: disc and its growth a page it serves, a page's; the room between subsystems that would stand as one.
PAGES, OUT, RING, BELOW, FOOT = 'host/terminal/pages/', 2.6, 3.0, 0.5, 1.0
DISC, GROWTH, PAGE_DISC, APART = 0.04, 0.008, 0.08, 0.22


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
def ordered(tree):
    """The pages round the terminal: from the one coming to the most, each next the one sharing most with
    the last (their subsystems in common over all theirs), so those that share most stand side by side."""
    own = {page: {f for f, ps in served(tree).items() if page in ps} for page, *_said in pages(tree)}
    left = sorted(own, key=lambda p: (-len(own[p]), p))
    chain = [left.pop(0)]
    while left:
        last = own[chain[-1]]
        left.sort(key=lambda p: (-len(own[p] & last) / max(1, len(own[p] | last)), p))
        chain.append(left.pop(0))
    return chain


def reach(tree, page):
    """The files the page `page` comes to (`systems`), None for none chosen: what the mirror keeps to."""
    return None if page is None else systems(tree).get(page)


def placed(tree, at, chosen=None):
    """The scene the atlas draws by application (scene.picture), `at` the tree's own {layer: where it stands}
    for its layers' heights, `chosen` the page lit (None: all): `at` {node: where it stands}, `flat` its nodes
    and its branches, `rings` {page: its subsystems, its own first}, `names` {node: its name} - a page its
    headline, a folder its own or its parent's with it where two share one - and `groups` their order."""
    from coaxial.model.blocks import numpy as np
    order, users = ordered(tree), served(tree)
    angle = {page: 2 * math.pi * k / len(order) for k, page in enumerate(order)}
    root, place = 0, {0: (0.0, at[0][1] - FOOT, 0.0)}
    place.update({page: (RING * math.cos(angle[page]), at[0][1] - BELOW, RING * math.sin(angle[page])) for page in order})
    spots = {}
    for f, ps in sorted(users.items()):
        spots.setdefault((layer(tree, f), ps), []).append(f)
    for (lay, ps), fs in spots.items():   # toward the pages it serves, the farther out the fewer
        x, z = sum(math.cos(angle[p]) for p in ps), sum(math.sin(angle[p]) for p in ps)
        r, a = OUT * (len(order) - len(ps)) / max(1, len(order) - 1), math.atan2(z, x)
        for j, f in enumerate(fs):   # those that would stand as one spread along the way round, APART
            off = (j - (len(fs) - 1) / 2) * APART
            aa, rr = (a + off / r, r) if r > APART else (2 * math.pi * j / len(fs), APART * (len(fs) > 1))
            place[f] = (at[lay][0] + rr * math.cos(aa), at[lay][1], at[lay][2] + rr * math.sin(aa))
    lit = set(place) if chosen is None else {chosen} | {f for f, ps in users.items() if chosen in ps}
    parts = [(place[root], place[p], scene.EDGE if p in lit else scene.DIM_EDGE, True) for p in order]
    for page in order:   # its branch: each subsystem from the nearest of its own below
        mine = sorted((f for f, ps in users.items() if page in ps), key=lambda f: place[f][1])
        for k, f in enumerate(mine):
            below = [page] + [g for g in mine[:k] if place[g][1] < place[f][1]]
            under = min(below, key=lambda g: sum((a - b) ** 2 for a, b in zip(place[g], place[f])))
            parts.append((place[under], place[f], scene.EDGE if page == chosen or chosen is None else scene.DIM_EDGE,
                          False))
    nodes = [root] + order + sorted(users)
    ink = [scene.OPEN if n == chosen else scene.NODE if n in lit else scene.DIM_NODE for n in nodes]
    disc = [scene.DISC[0]] + [PAGE_DISC] * len(order) + [DISC + GROWTH * len(users[f]) for f in sorted(users)]
    ends = np.array([s[0] for s in parts] + [s[1] for s in parts], float).reshape(2, -1, 3)
    rings = {page: sorted((f for f, ps in users.items() if page in ps), key=lambda f: (len(users[f]), place[f][1], f))
             for page in order}
    first = [chosen] + rings[chosen] if chosen is not None else []
    names = {**nested.names(tree, sorted(users)), **{page: headline for page, headline, *_said in pages(tree)}}
    return {'tree': tree, 'focus': -1 if chosen is None else chosen, 'at': place, 'rings': rings, 'out': {}, 'into': {},
            'names': names,
            'groups': (first + order, sorted(users, key=lambda f: (-len(users[f]), f))),   # the keys' panels between
            'flat': {'nodes': np.array(nodes, np.intp), 'pos': np.array([place[n] for n in nodes], float).reshape(-1, 3),
                     'disc': np.array(disc, float), 'ink': np.array(ink, np.uint8), 'lit': lit, 'from': ends[0],
                     'to': ends[1], 'segink': np.array([s[2] for s in parts], np.uint8),
                     'thick': np.array([s[3] for s in parts], bool)}}


def framed(scene_):
    """(target, distance) that frames the scene (`placed`): its page chosen and its subsystems where one is,
    else whole - its middle, as far as its widest."""
    lit = scene_['flat']['lit']
    xs, ys, zs = zip(*(p for n, p in scene_['at'].items() if n in lit or scene_['focus'] < 0))
    middle = ((max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, (max(zs) + min(zs)) / 2)
    return middle, 1.2 * max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)) + 1.0


def titled(tree, page):
    """A page's headline and what it is, as the terminal's menu offers it."""
    return next(('%s  %s' % (headline, what) for node, headline, _key, what in pages(tree) if node == page), '')

