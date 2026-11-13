"""The terminal's applications on the branches they part along, each page's subsystems up the trunk.

Each page of the terminal (terminal/pages/: its HEADLINE, its KEY) is an application; what it comes to is
what its `run` names, followed through the atlas's ties from symbol to symbol (tools/dev/atlas_calls.py) -
not a module's every import: a page asking one function of a module takes in none of its other users -,
and the C a file it comes to names or answers on the wire (`systems`). A folder is a subsystem of the pages
that come to it, of one alone or shared (`served`). The pages part along branches (`clades`): each set of
pages a folder serves alone, those sharing the most code first, kept where it nests in those kept, named by
its heaviest folder - aircraft/ the flyers'. `placed` lays the scene the atlas draws at the terminal
(scene.picture): the trunk as the tree has it, the terminal at its foot the root, and under it on a taproot
the flower of its pages (`flower`) - one flat disc, so no branch crosses another, turned to the camera
wherever it looks from (`facing`), so never edge on nor mirrored: the branches ring in by their depth, each at the middle of its own, the
pages round the rim in their order, the rim open where the taproot comes in, a branch drawn as a curve from
the one it hangs from. The node open lit - a branch with its pages, a page with its subsystems, each up at
its layer's height toward the pages it serves, the farther out the fewer, each page's way up through them,
each hung from the nearest of its own below -, none other drawn; its keys `offered`; `reach` what the
mirror (atlas/mirror.py) keeps to.
"""
import ast
import functools
import math
import os

from terminal.views.atlas import nested, scene
from tools.dev import atlas_calls
from tools.dev.atlas_tree import FILE, LAYERS, REPO, TIES

#: Where the terminal's pages are; how far out a subsystem of one page stands and the flower's rim, how far
#: under the terminal it hangs - its rim and a layer's step, its crown under the terminal from any side -,
#: world units; a subsystem's disc and its growth a page it serves, a page's, a branch's; the room between
#: subsystems that would stand as one.
PAGES, OUT, RING = 'host/terminal/pages/', 2.6, 3.0
ROOT = RING + scene.STEP
DISC, GROWTH, PAGE_DISC, FORK_DISC, APART = 0.04, 0.008, 0.08, 0.06, 0.22
#: The flower's depth of field, a share of the camera's distance (scene.blur: the rim sharp, the trunk's crown
#: out of focus); a branch's curve, its steps.
FIELD, STEPS = 2.0, 12


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


@functools.lru_cache(maxsize=2)
def flower(tree):
    """{branch or page: (radius, angle)} on the flower's disc, the angle from its right toward its top: a page
    on the rim, the pages in their order (`ordered`) clockwise from the top, a page's room left open there for
    the taproot; a branch a ring in by its depth, at the middle of its own's angles - the terminal the hub."""
    c, order, depth = clades(tree), ordered(tree), {}

    def deepen(node, d):
        for k in c['kids'][node]:
            depth[k] = d + 1
            if k < -1:
                deepen(k, d + 1)
    deepen(None, 0)
    rim, out = max(depth[p] for p in order), {}
    out.update((p, (RING, math.pi / 2 - 2 * math.pi * (k + 1) / (len(order) + 1))) for k, p in enumerate(order))

    def middle(node):
        for k in c['kids'][node]:
            if k < -1:
                middle(k)
        a = [out[k][1] for k in c['kids'][node]]
        out[node] = (RING * depth[node] / rim, math.atan2(sum(map(math.sin, a)), sum(map(math.cos, a))))
    for b in c['kids'][None]:
        if b < -1:
            middle(b)
    return out


def faced(at, cam=None):
    """(hub, right, up) of the flower's disc under the terminal standing at `at`: ROOT under it, in the plane
    `cam` sees square on (scene.view's right and up; the view the page opens with where None)."""
    sight = scene.view(cam or scene.camera(), 1, 1)
    return (at[0], at[1] - ROOT, at[2]), sight['r'], sight['u']


def curve(start, end):
    """The way from `start` to `end`, (radius, angle) on the disc, in STEPS: out at an even pace, round eased
    in and out, so it leaves the one it hangs from and reaches its own along their radii - from the hub
    straight out."""
    (r0, a0), (r1, a1) = start, end
    turn = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0)) if r0 else 0.0
    return [(r0 + (r1 - r0) * t, a1 - turn + turn * t * t * (3 - 2 * t)) for t in (k / STEPS for k in range(STEPS + 1))]


def placed(tree, upright, chosen=None, cam=None):
    """The scene the atlas draws at the terminal by application (scene.picture), over `upright` the tree's
    own (its layers where they stand, its stubs), `chosen` the branch or page open (None: the terminal), its
    flower facing `cam` (`faced`) - the rest where the view the page opens with lays it, its subsystems toward
    the pages as that view's flower holds them -: `at` {node: where it stands} - every subsystem's, drawn where lit -, `flat` its nodes and its branches,
    `rings` {page: its subsystems, its own first}, `names` {node: its name} - a page its headline, a branch
    its heaviest folder's, a folder its own or its parent's with it where two share one -, `briefs` - a
    branch's folders, what a page is -, `groups` their order, `lefts` those named outward on their left,
    the `field` its focus holds and the leaf `panels` it keeps (atlas/leaves.py): the nearest's alone."""
    from coaxial.model.blocks import numpy as np
    c, users, order, pol = clades(tree), served(tree), ordered(tree), flower(tree)
    lay, every, root = upright['at'], len(order), 0
    (hub, right, up), home = faced(lay[root], cam), faced(lay[root])

    def on(r, a, axes=(right, up)):
        return tuple(hub[k] + r * (math.cos(a) * axes[0][k] + math.sin(a) * axes[1][k]) for k in range(3))
    place = {**{n: lay[n] for n in range(len(LAYERS))}, **{n: on(*ra) for n, ra in pol.items()}}
    still = {p: on(pol[p][0], pol[p][1], home[1:]) for p in order}   # a page where the opening view's flower holds it
    toward = {p: math.atan2(still[p][2] - lay[root][2], still[p][0] - lay[root][0]) for p in order}
    spots = {}
    for f, ps in sorted(users.items()):
        spots.setdefault((layer(tree, f), ps), []).append(f)
    for (lay_, ps), fs in spots.items():   # toward the pages it serves, the farther out the fewer
        x, z = sum(math.cos(toward[p]) for p in ps), sum(math.sin(toward[p]) for p in ps)
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
    shown, trunk = [f for f in sorted(users) if f in lit], [i for i, row in enumerate(LAYERS) if row[1] == 'trunk']
    parts = [(place[a], place[b], scene.EDGE if a in lit and b in lit else scene.DIM_EDGE, True)
             for a, b in zip(trunk, trunk[1:])]
    parts += [(p, place[n], scene.EDGE if n in lit else scene.DIM_EDGE, False) for p, n in upright['stubs']]
    for n in range(len(LAYERS)):   # a layer's ring round it, as the tree draws it
        x, y, z = place[n]
        ring = [(x + scene.HALO * cs, y, z + scene.HALO * sn) for cs, sn in scene.TURNS]
        parts += [(p, q, scene.EDGE if n in lit else scene.DIM_EDGE, False) for p, q in zip(ring, ring[1:])]
    parts.append((place[root], hub, scene.EDGE, True))   # the taproot
    for n, ra in pol.items():   # each branch and page a curve from the one it hangs from
        way = [on(*q) for q in curve((0.0, ra[1]) if c['up'][n] is None else pol[c['up'][n]], ra)]
        parts += [(p, q, scene.EDGE if n in lit else scene.DIM_EDGE, False) for p, q in zip(way, way[1:])]
    for page in (p for p in order if p in held and chosen is not None):   # its way up, each from its nearest below
        mine = sorted((f for f in shown if page in users[f]), key=lambda f: place[f][1])
        for k, f in enumerate(mine):
            below = [page] + [g for g in mine[:k] if place[g][1] < place[f][1]]
            under = min(below, key=lambda g: sum((a - b) ** 2 for a, b in zip(still[g] if g in still else place[g], place[f])))
            parts.append((place[under], place[f], scene.EDGE, False))
    focus, forks = root if chosen is None else chosen, sorted(c['pages'], reverse=True)
    nodes = list(range(len(LAYERS))) + forks + order + shown
    ink = [scene.OPEN if n == focus else scene.NODE if n in lit else scene.DIM_NODE for n in nodes]
    discs = [scene.DISC[0]] * len(LAYERS) + [FORK_DISC] * len(forks) + [PAGE_DISC] * every \
        + [DISC + GROWTH * len(users[f]) for f in shown]
    ends = np.array([s[0] for s in parts] + [s[1] for s in parts], float).reshape(2, -1, 3)
    names = {**nested.names(tree, sorted(users)), **{page: headline for page, headline, *_said in pages(tree)}}
    names.update((b, names[fs[0]]) for b, fs in c['by'].items())
    keys = offered(tree, chosen)
    lefts = {n for n, (_r, a) in pol.items() if math.cos(a) < 0} \
        | {f for f in shown if sum((place[f][k] - lay[layer(tree, f)][k]) * right[k] for k in range(3)) < 0}
    sight = cam or scene.camera()
    return {'tree': tree, 'focus': focus, 'at': place, 'out': {}, 'into': {}, 'names': names, 'lefts': lefts,
            'field': FIELD, 'panels': 1, 'upright': upright, 'chosen': chosen, 'faced': (sight['yaw'], sight['pitch']), 'rings': {page: offered(tree, page) for page in order},
            'briefs': {**{b: 'theirs alone: ' + ' '.join(names[f] for f in fs) for b, fs in c['by'].items()},
                       **{page: what for page, _headline, _key, what in pages(tree) if what}},
            'groups': ([focus] + keys + path + [root],   # the keys' panels between
                       forks + order + list(range(len(LAYERS))) + sorted(shown, key=lambda f: (-len(users[f]), f))),
            'flat': {'nodes': np.array(nodes, np.intp), 'pos': np.array([place[n] for n in nodes], float).reshape(-1, 3),
                     'disc': np.array(discs, float), 'ink': np.array(ink, np.uint8), 'lit': lit, 'from': ends[0],
                     'to': ends[1], 'segink': np.array([s[2] for s in parts], np.uint8),
                     'thick': np.array([s[3] for s in parts], bool)}}


def facing(scene_, cam):
    """The scene (`placed`) with its flower facing `cam`: laid anew where the camera turned since (a lay is a
    millisecond or two, so it follows a drag or an orbit frame by frame)."""
    if scene_['faced'] != (cam['yaw'], cam['pitch']):
        scene_.update(placed(scene_['tree'], scene_['upright'], scene_['chosen'], cam))
    return scene_


def framed(scene_):
    """(target, distance) that frames what the scene (`placed`) lights - the terminal its branches, a branch
    its own and their subsystems, a page its subsystems -: its middle, as far as its widest."""
    lit = scene_['flat']['lit']
    xs, ys, zs = zip(*(p for n, p in scene_['at'].items() if n in lit))
    middle = ((max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, (max(zs) + min(zs)) / 2)
    return middle, 1.2 * max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)) + 1.0
