"""The terminal's applications as a tree: a trunk of what all share, branches of what some share.

Each page of the terminal (terminal/pages/: its HEADLINE, its KEY) is an application; what it comes to is
what its `run` names, followed through the atlas's ties from symbol to symbol (tools/dev/atlas_calls.py) -
not a module's every import: a page asking one function of a module takes in none of its other users -, the
loop of each tick it flies, its runner's where no call names it (`reached`), and the C a file it comes to
names or answers on the wire (`systems`). A folder is a subsystem of the pages
that come to it, of one alone or shared (`served`). The pages part along branches (`clades`): each set of
pages a folder serves alone, those sharing the most code first, kept where it nests in those kept, named by
its heaviest folder - aircraft/ the flyers' (growth.nest, the rule every level of the tree parts by). `placed`
lays the tree the atlas draws (scene.picture) as a tree grows (`boughs`, growth.grow; laid once in the world by
`grove`): the terminal at its foot, the trunk up from it
carrying what every page shares, the branch holding most pages leading on up, the rest arcs out round it by
the golden angle at Murray's angles, each bough carrying what its pages share alone (`owners`), a page's own
on its twig, the page at its tip; the node open lit, a fork named on the way open alone, the keys on the
nodes named in view (`seen`); a node opened in it grows the subtree it lies in out of the node the tree
holds, the same tree inherited down (atlas/subtree.py); framed in the room its panel leaves (`framed`). The
dataflow (atlas/mirror.py) draws one system a time (`dataflow`) - a page's,
a branch's pages' together, the robot's where none is chosen -, each its own loop: the robot's streams, a
craft's tick (tools/dev/atlas_flow.py).
"""
import ast
import functools
import math
import os

from terminal.views.atlas import aim, growth, leaves, nested, scene, subtree
from tools.dev import atlas_calls, atlas_flow
from tools.dev.atlas_tree import FILE, REPO, TIES

#: Where the terminal's pages are; a folder's disc and its growth a page it serves, a page's, a branch's, world
#: units.
PAGES, DISC, GROWTH, PAGE_DISC, FORK_DISC = 'host/terminal/pages/', 0.04, 0.008, 0.08, 0.06
#: The tree's depth of field, a share of the camera's distance (scene.blur).
FIELD = 2.0


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
    for page, keys in reached(tree).items():
        files = {tree.at[k.partition('::')[0]] for k in keys if k.partition('::')[0] in tree.at}
        out[page] = frozenset(files | {d for f in files for d in into.get(f, ())})
    return out


@functools.lru_cache(maxsize=2)
def reached(tree):
    """{page: the symbols its `run` comes to} (atlas_calls.reach), and those the ticks it flies come to, run by
    its runner where no call names them (atlas_flow.flying, stepped): what it runs, a function at a time."""
    flies, steps, out = atlas_flow.flying(tree), atlas_flow.stepped(tree), {}
    for page, *_said in pages(tree):
        keys, flown = set(atlas_calls.reach(tree, [tree.id[page] + '::run'])), set()
        while {t for k in keys for t in flies.get(k, ())} - flown:
            new = {t for k in keys for t in flies.get(k, ())} - flown
            flown |= new
            keys |= atlas_calls.reach(tree, [s for t in sorted(new) for s in steps[t]])
        out[page] = frozenset(keys)
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
    """The branches the pages part along, out from the terminal (growth.nest): each set of pages a folder serves
    alone (`served`) - two or more, not all -, the most code shared first (its folders' files), kept where it
    nests in those kept. {'up': {branch or page: the branch it hangs from, None the terminal}, 'kids': {None or a
    branch: its pages, then its branches, the most pages first}, 'pages': {branch: its pages}, 'by': {branch:
    its folders, the heaviest first}}; a branch an id from -2 down, the one holding most first."""
    files = {}
    for node, kind in enumerate(tree.kind):
        if kind == FILE:
            files[tree.up[node]] = files.get(tree.up[node], 0) + 1
    n = growth.nest([page for page, *_said in pages(tree)], served(tree), lambda f: files.get(f, 0))
    return {'up': n['up'], 'kids': n['kids'], 'pages': n['held'], 'by': n['by']}


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


def slots(tree, n):
    """How many steps a bough `n` (0 the trunk) is laid in: one for each folder it carries (`owners`), then one
    for each kid out of it - the last at its end -, a page's tip one."""
    c = clades(tree)
    return sum(1 for o in owners(tree).values() if o == n) + max(1, len(c['kids'].get(None if n == 0 else n, ())))


@functools.lru_cache(maxsize=2)
def boughs(tree):
    """{0, a branch or a page: (start, way, across, angle, length, whether it leads)}: the tree's boughs as it grows
    (growth.grow), the terminal at the origin: the trunk (0) straight up, growth.APART a step (`slots`) - a
    bough's folders along it from its start, its kids out of it above them -, the branches the pages part along
    (`clades`) out of it, each holding its pages."""
    c = clades(tree)
    out = {0: ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), 0.0, growth.APART * slots(tree, 0), True)}
    growth.grow(out[0], None, c['kids'], lambda n: len(c['pages'].get(n, (n,))),
                lambda n: slots(tree, 0 if n is None else n), len(pages(tree)), [0], out)
    return out


@functools.lru_cache(maxsize=2)
def grove(tree):
    """({node: where it stands}, [(from, to, bough, whether it leads)]): the tree laid as it grows (`boughs`), the
    terminal at the origin - a branch at its bough's end, a page at its twig's tip, each folder along the bough
    that carries it (`owners`) by its layer, the terminal's first -, each bough in pieces (growth.pieces)."""
    grown, own, users = boughs(tree), owners(tree), served(tree)
    at = {n: growth.along(b, 1.0)[0] for n, b in grown.items() if n != 0}
    for n, b in grown.items():   # what each bough carries along it
        mine = sorted((f for f in users if own[f] == n), key=lambda f: (layer(tree, f), f))
        at.update((f, growth.along(b, (j + 1) / slots(tree, n))[0]) for j, f in enumerate(mine))
    return at, growth.pieces(grown)


def anchor(tree, node):
    """The nearest of `node` and those above it that stands in the tree (`grove`) - a subtree's root -, else -1."""
    laid = grove(tree)[0]
    while node >= 0 and node not in laid:
        node = tree.up[node]
    return node


def placed(tree, upright, chosen=None, cam=None, opened=-1):
    """The scene the atlas draws by application (scene.picture), the tree's terminal (`upright`'s) its foot,
    `chosen` the branch or page open (None: none), `opened` a node opened in it (-1: none): the tree as it grows
    (`grove`), in the world - turned, the view goes round it -, the subtree of the node it lies in that the tree
    holds grown out of it (`anchor`, subtree.grown). Lit: a branch with its pages and their folders, a page with
    its folders, named - in a subtree the way down to the node opened and all it holds, its children, its way and
    the page named -; the folders of none lit grown in a zoom key's step nearer than the frame, a subtree drawn
    two depths under the node opened and a depth more a step nearer. `at`
    {node: where it stands}, `flat`, `rings` {page: its folders, its own first}, `names` - a page its headline, a
    branch its heaviest folder's -, `briefs`, `groups`, `lefts` named on their left - left of the trunk as `cam`
    sees it -, and the trunk's top its `hub`."""
    from coaxial.model.blocks import numpy as np
    c, users, order, (laid, ways) = clades(tree), served(tree), ordered(tree), grove(tree)
    root, top, sight = anchor(tree, opened) if opened >= 0 else -1, 0, cam or scene.camera()
    empty = ({}, (np.zeros((0, 3)), np.zeros((0, 3)), np.zeros(0, np.intp), np.zeros(0, bool)), {}, {}, {}, {})
    grown, inner, kin, up, forked, _boughs = subtree.grown(tree, root, laid[root], -2 - len(c['pages'])) \
        if root >= 0 else empty
    base, right = upright['at'][top], scene.view(sight, 1, 1)['r']

    def at(p):
        return tuple(base[k] + p[k] for k in range(3))
    place = {top: base, **{n: at(p) for n, p in laid.items()}, **{n: at(p) for n, p in grown.items()}}
    held = c['pages'].get(chosen, frozenset(order) if chosen is None else {chosen})   # the pages open
    path, node = [], chosen
    while node is not None:
        path, node = [node] + path, c['up'][node]
    way, walk, mine, level = [opened] * (root >= 0), [root] * (root >= 0), [opened] * (root >= 0), {root: 0}
    while way and way[-1] != root:   # the subtree's root down to the node opened, the forks between
        way.append(up[way[-1]])
    for n in walk:   # each node's depth under the root, a fork its stem's
        level.update((k, level[n] + (k >= 0)) for k in kin.get(n, ()))
        walk += kin.get(n, ())
    for n in mine:   # all the node opened holds
        mine += kin.get(n, ())
    rings = {opened: scene.children(tree, *subtree.holding(tree), opened)} if root >= 0 else {}
    lit = {top, *path, *held, *(b for b, ps in c['pages'].items() if ps <= held),
           *(f for f, ps in users.items() if ps & held and chosen is not None)}
    lit = {*way, *mine} if root >= 0 else lit
    # Nearer than the frame by a zoom key's step the folders of none lit grow in; to a billionth of a step, as a
    # key's 1 / 0.8 is no binary fraction.
    steps = round(math.log(spread([place[n] for n in lit])[1] / sight['distance']) / math.log(scene.ZOOM), 9)
    drawn = [f for f in sorted(users) if f in lit or steps > 0]
    deep = level.get(opened, 0) + 2 + max(0, math.floor(steps))
    shown = {n for n, d in level.items() if root >= 0 and d + (n < 0) <= deep}
    parts = [(at(a), at(b), scene.EDGE if n in lit else scene.DIM_EDGE, leads) for a, b, n, leads in ways]
    keep = np.isin(inner[2], sorted(shown))   # a subtree's pieces drawn, as arrays: a piece a row
    lights = np.where(np.isin(inner[2][keep], sorted(lit)), scene.EDGE, scene.DIM_EDGE)
    focus, forks = opened if root >= 0 else top if chosen is None else chosen, sorted(c['pages'], reverse=True)
    disc = {top: scene.DISC[0], **dict.fromkeys(forks, FORK_DISC), **dict.fromkeys(order, PAGE_DISC),
            **{f: DISC + GROWTH * len(users[f]) for f in drawn},
            **{n: scene.DISC[tree.kind[n]] if n >= 0 else FORK_DISC for n in grown}}
    nodes = [top] + forks + [n for n in order + drawn if n not in grown] + sorted(n for n in grown if n in shown)
    ink = [scene.OPEN if n == focus else scene.NODE if n in lit else scene.DIM_NODE for n in nodes]
    discs = [disc[n] for n in nodes]
    ends = np.concatenate((np.array([s[0] for s in parts] + [s[1] for s in parts], float).reshape(2, -1, 3),
                           np.stack((inner[0][keep], inner[1][keep])) + np.array(base)), axis=1)
    names = {**nested.names(tree, sorted(users)), **{page: headline for page, headline, *_said in pages(tree)}}
    names.update((b, names[fs[0]]) for b, fs in c['by'].items())
    names.update(forked)
    keys = rings.get(opened, []) if root >= 0 else [n for n in offered(tree, chosen) if n in nodes and n not in c['pages']]
    down = [n for n in way if n >= 0] + [chosen] * (chosen is not None)   # a subtree's forks unnamed: no key
    axis = {n: place[root] if n in grown else base for n in place}   # a subtree's named round its own shoot
    lefts = {n for n, p in place.items() if (p[0] - axis[n][0]) * right[0] + (p[2] - axis[n][2]) * right[2] < 0}
    named = sorted((f for f in drawn if f in lit), key=lambda f: (len(users[f]), f))
    return {'tree': tree, 'focus': focus, 'at': place, 'out': {}, 'into': {}, 'names': names, 'lefts': lefts,
            'field': FIELD, 'panels': 1, 'upright': upright, 'chosen': chosen, 'opened': opened,
            'hub': at(growth.along(boughs(tree)[top], 1.0)[0]), 'faced': (sight['yaw'], sight['pitch'], sight['distance']),
            'rings': rings if root >= 0 else {page: offered(tree, page) for page in order},
            'briefs': {**{b: 'theirs alone: ' + ' '.join(names[f] for f in fs) for b, fs in c['by'].items()},
                       **{page: what for page, _headline, _key, what in pages(tree) if what}},
            'groups': ([focus] + keys + down + path + [top],   # the panels between; a fork named on the way open alone
                       [] if root >= 0 else [n for n in order if n in lit] + named + [n for n in order if n not in lit]),
            'flat': {'nodes': np.array(nodes, np.intp), 'pos': np.array([place[n] for n in nodes], float).reshape(-1, 3),
                     'disc': np.array(discs, float), 'ink': np.array(ink, np.uint8), 'lit': lit, 'from': ends[0],
                     'to': ends[1], 'segink': np.concatenate(([s[2] for s in parts], lights)).astype(np.uint8),
                     'thick': np.concatenate(([s[3] for s in parts], inner[3][keep])).astype(bool)}}


def seen(scene_, cam, cols, rows, keys):
    """The nodes the keys take as `cam` shows the scene on a drawing `cols` x `rows`: those named within it, the
    nearest its middle first, `keys` a page, each page in reading order - down its rows, along each."""
    from coaxial.model.blocks import numpy as np
    sight, flat = scene.view(cam, cols, rows), scene_['flat']
    rel = flat['pos'] - np.array(sight['eye'])
    x, y, z = (rel @ np.array(sight[a]) for a in ('r', 'u', 'f'))
    with np.errstate(divide='ignore', invalid='ignore'):
        across, down = sight['w'] / 2 + x * sight['focal'] / z, sight['h'] / 2 - y * sight['focal'] / z
    inside = (z > scene.NEAR) & (across >= 0) & (across < sight['w']) & (down >= 0) & (down < sight['h']) \
        & np.isin(flat['nodes'], sorted(set().union(*scene_['groups'])))   # named: a key never without its name
    near = sorted(np.flatnonzero(inside).tolist(),
                  key=lambda k: (across[k] - sight['w'] / 2) ** 2 + (down[k] - sight['h'] / 2) ** 2)
    nodes = flat['nodes'].tolist()
    return [nodes[k] for start in range(0, len(near), keys)
            for k in sorted(near[start:start + keys], key=lambda k: (int(down[k] // 4), across[k]))]


def facing(scene_, cam):
    """The scene (`placed`) facing `cam`, its folders grown to its distance: laid anew where the camera turned or
    came nearer since (a lay a millisecond or two, so it follows a drag, an orbit or a zoom frame by frame)."""
    if scene_['faced'] != (cam['yaw'], cam['pitch'], cam['distance']):
        scene_.update(placed(scene_['tree'], scene_['upright'], scene_['chosen'], cam, scene_['opened']))
    return scene_


def spread(points):
    """(target, distance) that frames `points`: their middle, as far as their widest."""
    from coaxial.model.blocks import numpy as np
    at = np.asarray(points, float).reshape(-1, 3)
    low, high = at.min(0), at.max(0)
    return tuple(((low + high) / 2).tolist()), 1.2 * float((high - low).max()) + 1.0


def framed(scene_, cam=None, room=None):
    """(target, distance) that frames what the scene (`placed`) lights - the tree whole, a branch with its pages and
    their folders, a page with its folders, a subtree -: as far as their box (`spread`), the target where they
    stand in the middle of `cam`'s view (aim.centred; the opening view's where None) - on a drawing `room` (cols,
    rows) in the middle of the room the panel leaves (aim.beside, leaves.width), its `side` the panel's, kept for the
    view's turn."""
    cam = cam or scene.camera()
    key = (cam['yaw'], cam['pitch'], room)
    if scene_.get('framing', (None,))[0] != key:   # what it lights as it is drawn: its nodes, its boughs' pieces
        from coaxial.model.blocks import numpy as np
        flat = scene_['flat']
        bright = flat['segink'] == scene.EDGE
        points = np.concatenate((np.array([p for n, p in scene_['at'].items() if n in flat['lit']], float).reshape(-1, 3),
                                 flat['from'][bright], flat['to'][bright]))
        distance = spread(points)[1]
        sight = scene.view(dict(cam, distance=distance), *(room or (1, 1)))
        target, distance, side = (aim.centred(points, cam, distance), distance, None) if room is None else \
            aim.beside(points, cam, distance, 2 * leaves.width(room[0]) / sight['focal'], sight['w'] / 2 / sight['focal'])
        scene_['framing'], scene_['side'] = (key, (target, distance)), side
    return scene_['framing'][1]


@functools.lru_cache(maxsize=2)
def robot(tree):
    """The page whose system runs the robot's streams (atlas_flow.makers): the dataflow drawn where none is chosen."""
    hers = {tree.up[n] for n in atlas_flow.makers(tree).values() if n >= 0}
    return next(p for p, files in systems(tree).items() if hers <= files)


def system(tree, app):
    """The application whose dataflow is drawn: `app` - a page, a branch its pages together -, else the robot."""
    return robot(tree) if app is None else app


def dataflow(tree, app, flows):
    """The flows of `app`'s system (`system`) between its files (`reach`): its loop and what comes back of it - the
    robot's streams where it runs them, a craft's tick where it flies one (atlas_flow.flying) or comes to it, not
    one handed on to where it comes to the one handing it (atlas_flow.ticks, handing) -, else what its functions
    call of each other's (atlas_flow.called), and the wire it speaks."""
    held, files = clades(tree)['pages'].get(system(tree, app), (system(tree, app),)), reach(tree, system(tree, app))
    keys = frozenset().union(*(reached(tree)[p] for p in held))
    ticks, hands, flies = atlas_flow.ticks(tree), atlas_flow.handing(tree), atlas_flow.flying(tree)
    flown = {t for k in keys for t in flies.get(k, ())}
    came = flown | {k for k in ticks if k in keys}
    run = [k for k in ticks if k in flown or (k in keys and not any(hands.get(h) == k for h in came))]
    passed, wire = (atlas_flow.KINDS.index(kind) for kind in ('pass', 'wire'))
    mine = [f for f in flows if f[0] in files and f[1] in files]
    streams = any(f[2] == passed for f in mine)   # what comes back of the robot's streams where it runs them
    own = [f for f in mine if streams or f[2] == wire] + [f for k in run for f in ticks[k]]
    return own if any(f[2] == passed for f in own) else own + atlas_flow.called(tree, files)


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
