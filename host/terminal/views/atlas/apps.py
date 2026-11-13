"""The terminal's applications as a tree: a trunk of what all share, branches of what some share.

Each page of the terminal (terminal/pages/: its HEADLINE, its KEY) is an application; what it comes to is
what its `run` names, followed through the atlas's ties from symbol to symbol (tools/dev/atlas_calls.py) -
not a module's every import: a page asking one function of a module takes in none of its other users -,
and the C a file it comes to names or answers on the wire (`systems`). A folder is a subsystem of the pages
that come to it, of one alone or shared (`served`). The pages part along branches (`clades`): each set of
pages a folder serves alone, those sharing the most code first, kept where it nests in those kept, named by
its heaviest folder - aircraft/ the flyers'. `placed` lays the tree the atlas draws (scene.picture): the
terminal at its foot, the trunk up from it carrying what every page shares, at its top the crown (`crown`),
its branches carrying what their pages share alone (`owners`), a page's own on its twig, the page at its
tip, all facing the camera (`facing`); the node open lit, its keys `offered`. The dataflow (atlas/mirror.py)
draws one system a time (`dataflow`): a page's, a branch's pages' together, the robot's where none is chosen.
"""
import ast
import functools
import math
import os

from terminal.views.atlas import aim, nested, scene
from tools.dev import atlas_calls, atlas_flow
from tools.dev.atlas_tree import FILE, REPO, TIES

#: Where the terminal's pages are; the crown's radius, world units; a folder's disc and its growth a page it
#: serves, a page's, a branch's; the room a folder takes on the trunk.
PAGES, RING = 'host/terminal/pages/', 3.0
DISC, GROWTH, PAGE_DISC, FORK_DISC, APART = 0.04, 0.008, 0.08, 0.06, 0.22
#: The tree's depth of field, a share of the camera's distance (scene.blur); a branch's curve, its steps.
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
def owners(tree):
    """{folder: where on the tree it hangs}: a page's own on its page, what pages share on the smallest branch
    holding them all (`clades`), the rest - every page's, or pages no branch holds together - on the trunk (the
    terminal, 0)."""
    c, out = clades(tree), {}
    for f, ps in served(tree).items():
        holders = [b for b, held in c['pages'].items() if ps <= held]
        out[f] = next(iter(ps)) if len(ps) == 1 else min(holders, key=lambda b: len(c['pages'][b])) if holders else 0
    return out


@functools.lru_cache(maxsize=2)
def crown(tree):
    """{branch or page: (radius, angle)} in the crown round its hub, the angle from its right toward its top: a
    page on the rim, the pages in their order (`ordered`) clockwise from the trunk, a page's room left open below
    where the trunk comes in; a branch a ring in by its depth, at the middle of its own's angles."""
    c, order, depth = clades(tree), ordered(tree), {}

    def deepen(node, d):
        for k in c['kids'][node]:
            depth[k] = d + 1
            if k < -1:
                deepen(k, d + 1)
    deepen(None, 0)
    rim, out = max(depth[p] for p in order), {}
    out.update((p, (RING, -math.pi / 2 - 2 * math.pi * (k + 1) / (len(order) + 1))) for k, p in enumerate(order))

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


def along(start, end, t):
    """The point `t` of the way from `start` to `end`, (radius, angle) in the crown: out at an even pace, round
    eased in and out, so it leaves the one it hangs from and reaches its own along their radii - from the hub
    straight out."""
    (r0, a0), (r1, a1) = start, end
    turn = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0)) if r0 else 0.0
    return r0 + (r1 - r0) * t, a1 - turn + turn * t * t * (3 - 2 * t)


def curve(start, end):
    """The way from `start` to `end` in STEPS (`along`)."""
    return [along(start, end, k / STEPS) for k in range(STEPS + 1)]


def placed(tree, upright, chosen=None, cam=None):
    """The scene the atlas draws by application (scene.picture), the tree's terminal (`upright`'s) its foot,
    `chosen` the branch or page open (None: none): the trunk up from the terminal carrying what every page
    shares - each folder no branch holds (`owners`) - by its layer, the terminal's first, APART a folder; at its
    top the crown (`crown`), each branch a curve out of the one it hangs from carrying what its pages share alone,
    a page's own on its twig, the page at its tip - all in the plane `cam` sees square on (the view the page
    opens with where None), so no branch crosses another nor is seen edge on. Lit: a branch with its pages and
    their folders, a page with its folders, named; the folders of none lit grown in a zoom key's step nearer than
    the frame. `at` {node: where it stands}, `flat`, `rings` {page: its folders, its own first}, `names` - a
    page its headline, a branch its heaviest folder's -, `briefs`, `groups`, `lefts` named on their left, the
    trunk's top its `hub`."""
    from coaxial.model.blocks import numpy as np
    c, users, order, pol, own = clades(tree), served(tree), ordered(tree), crown(tree), owners(tree)
    root, sight = 0, cam or scene.camera()
    view = scene.view(sight, 1, 1)
    right, up, base = view['r'], view['u'], upright['at'][root]

    def on(origin, r, a):
        return tuple(origin[k] + r * (math.cos(a) * right[k] + math.sin(a) * up[k]) for k in range(3))

    def hung(n):   # what hangs on the trunk or a branch, the nearest the terminal's layer first
        return sorted((f for f in users if own[f] == n), key=lambda f: (layer(tree, f), f))
    trunk = hung(root)
    hub = on(base, APART * (len(trunk) + 1), math.pi / 2)
    place = {root: base, **{n: on(hub, *ra) for n, ra in pol.items()}}
    place.update((f, on(base, APART * (j + 1), math.pi / 2)) for j, f in enumerate(trunk))
    for n, ra in pol.items():   # each branch's own along it
        mine, start = hung(n), (0.0, ra[1]) if c['up'][n] is None else pol[c['up'][n]]
        place.update((f, on(hub, *along(start, ra, (j + 1) / (len(mine) + 1)))) for j, f in enumerate(mine))
    held = c['pages'].get(chosen, frozenset(order) if chosen is None else {chosen})   # the pages open
    path, node = [], chosen
    while node is not None:
        path, node = [node] + path, c['up'][node]
    lit = {root, *path, *held, *(b for b, ps in c['pages'].items() if ps <= held),
           *(f for f, ps in users.items() if ps & held and chosen is not None)}
    # Nearer than the frame by a zoom key's step the folders of none lit grow in; to a billionth of a step, as a
    # key's 1 / 0.8 is no binary fraction.
    steps = round(math.log(spread([place[n] for n in lit])[1] / sight['distance']) / math.log(scene.ZOOM), 9)
    drawn = [f for f in sorted(users) if f in lit or steps > 0]
    parts = [(base, hub, scene.EDGE, True)]   # the trunk
    for n, ra in pol.items():   # each branch a curve from the one it hangs from
        way = [on(hub, *q) for q in curve((0.0, ra[1]) if c['up'][n] is None else pol[c['up'][n]], ra)]
        parts += [(p, q, scene.EDGE if n in lit else scene.DIM_EDGE, False) for p, q in zip(way, way[1:])]
    focus, forks = root if chosen is None else chosen, sorted(c['pages'], reverse=True)
    nodes = [root] + forks + order + drawn
    ink = [scene.OPEN if n == focus else scene.NODE if n in lit else scene.DIM_NODE for n in nodes]
    discs = [scene.DISC[0]] + [FORK_DISC] * len(forks) + [PAGE_DISC] * len(order) \
        + [DISC + GROWTH * len(users[f]) for f in drawn]
    ends = np.array([s[0] for s in parts] + [s[1] for s in parts], float).reshape(2, -1, 3)
    names = {**nested.names(tree, sorted(users)), **{page: headline for page, headline, *_said in pages(tree)}}
    names.update((b, names[fs[0]]) for b, fs in c['by'].items())
    keys = offered(tree, chosen)
    lefts = {n for n, (_r, a) in pol.items() if math.cos(a) < 0} | set(trunk[1::2]) \
        | {f for f in users if own[f] in pol and sum((place[f][k] - hub[k]) * right[k] for k in range(3)) < 0}
    named = sorted((f for f in drawn if f in lit), key=lambda f: (len(users[f]), f))
    return {'tree': tree, 'focus': focus, 'at': place, 'out': {}, 'into': {}, 'names': names, 'lefts': lefts,
            'field': FIELD, 'panels': 1, 'upright': upright, 'chosen': chosen, 'hub': hub,
            'faced': (sight['yaw'], sight['pitch'], sight['distance']), 'rings': {page: offered(tree, page) for page in order},
            'briefs': {**{b: 'theirs alone: ' + ' '.join(names[f] for f in fs) for b, fs in c['by'].items()},
                       **{page: what for page, _headline, _key, what in pages(tree) if what}},
            'groups': ([focus] + [k for k in keys if k in nodes] + path + [root],   # the keys' panels between
                       [n for n in forks + order if n in lit] + named + [n for n in forks + order if n not in lit]),
            'flat': {'nodes': np.array(nodes, np.intp), 'pos': np.array([place[n] for n in nodes], float).reshape(-1, 3),
                     'disc': np.array(discs, float), 'ink': np.array(ink, np.uint8), 'lit': lit, 'from': ends[0],
                     'to': ends[1], 'segink': np.array([s[2] for s in parts], np.uint8),
                     'thick': np.array([s[3] for s in parts], bool)}}


def facing(scene_, cam):
    """The scene (`placed`) facing `cam`, its folders grown to its distance: laid anew where the camera turned or
    came nearer since (a lay a millisecond or two, so it follows a drag, an orbit or a zoom frame by frame)."""
    if scene_['faced'] != (cam['yaw'], cam['pitch'], cam['distance']):
        scene_.update(placed(scene_['tree'], scene_['upright'], scene_['chosen'], cam))
    return scene_


def spread(points):
    """(target, distance) that frames `points`: their middle, as far as their widest."""
    xs, ys, zs = zip(*points)
    middle = ((max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, (max(zs) + min(zs)) / 2)
    return middle, 1.2 * max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)) + 1.0


def framed(scene_, cam=None):
    """(target, distance) that frames what the scene (`placed`) lights - the tree whole, a branch with its pages and
    their folders, a page with its folders -: as far as their box (`spread`), the target where they stand in the
    middle of `cam`'s view (aim.centred; the opening view's where None)."""
    lit = scene_['flat']['lit']
    points = [p for n, p in scene_['at'].items() if n in lit]
    distance = spread(points)[1]
    return aim.centred(points, cam or scene.camera(), distance), distance


@functools.lru_cache(maxsize=2)
def robot(tree):
    """The page whose system runs her pass (atlas_flow.makers): the dataflow drawn where none is chosen."""
    hers = {tree.up[n] for n in atlas_flow.makers(tree).values() if n >= 0}
    return next(p for p, files in systems(tree).items() if hers <= files)


def system(tree, app):
    """The application whose dataflow is drawn: `app` - a page, a branch its pages together -, else the robot."""
    return robot(tree) if app is None else app


def dataflow(tree, app, flows):
    """The flows of `app`'s system (`system`) between its files (`reach`): her pass and what comes back of it where
    it runs hers, else what its functions call of each other's (atlas_flow.called) - no pass told there -, and the
    wire it speaks."""
    files = reach(tree, system(tree, app))
    own = [f for f in flows if f[0] in files and f[1] in files]
    return own if any(f[2] == atlas_flow.KINDS.index('pass') for f in own) else own + atlas_flow.called(tree, files)


def following(tree, app):
    """The page after `app`'s system in the pages' order (`ordered`), round: the next dataflow."""
    order, now = ordered(tree), system(tree, app)
    return order[(order.index(now) + 1) % len(order)] if now in order else order[0]


def named(tree, node):
    """A page's headline, a branch's heaviest folder's name (`placed`'s `names`)."""
    by = clades(tree)['by']
    if node in by:
        return nested.names(tree, sorted(served(tree)))[by[node][0]]
    return next((headline for page, headline, *_said in pages(tree) if page == node), '')
