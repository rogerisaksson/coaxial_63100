"""The terminal's applications as a tree: a trunk of what all share, branches of what some share.

Each page of the terminal (terminal/pages/: its HEADLINE, its KEY) is an application; what it comes to is
what its `run` names, followed through the atlas's ties from symbol to symbol (tools/dev/atlas_calls.py) -
not a module's every import: a page asking one function of a module takes in none of its other users -, the
loop of each tick it flies, its runner's where no call names it (`reached`), and the C a file it comes to
names or answers on the wire (`systems`). A folder is a subsystem of the pages
that come to it, of one alone or shared (`served`). The pages part along branches (`clades`): each set of
pages a folder serves alone, those sharing the most code first, kept where it nests in those kept, named by
its heaviest folder - aircraft/ the flyers'. `placed` lays the tree the atlas draws (scene.picture) as a
tree grows (`boughs`, laid once in the world by `grove`): the terminal at its foot, the trunk up from it
carrying what every page shares, the branch holding most pages leading on up, the rest arcs out round it by
the golden angle at Murray's angles, each bough carrying what its pages share alone (`owners`), a page's own
on its twig, the page at its tip; the node open lit, a fork named on the way open alone, the keys on the
nodes named in view (`seen`). The dataflow (atlas/mirror.py) draws one system a time (`dataflow`) - a page's,
a branch's pages' together, the robot's where none is chosen -, each its own loop: the robot's streams, a
craft's tick (tools/dev/atlas_flow.py).
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


def lean(way, turn):
    """The unit way across the unit `way`, turned `turn` round it from a side of its own."""
    side = (0.0, way[2], -way[1]) if abs(way[0]) < 0.9 else (-way[2], 0.0, way[0])
    norm = math.sqrt(sum(v * v for v in side))
    side = tuple(v / norm for v in side)
    other = (way[1] * side[2] - way[2] * side[1], way[2] * side[0] - way[0] * side[2], way[0] * side[1] - way[1] * side[0])
    return tuple(math.cos(turn) * side[k] + math.sin(turn) * other[k] for k in range(3))


def along(bough, t):
    """(where, which way) a bough (`boughs`) stands `t` of its length out: an arc turning back to its stem's way by
    its angle, its chord at its angle - out of the stem half as far again, at its tip half -, as a bough sags out
    of its stem and its tip rises to the light; straight at none. A circle's arc: its chord as long as the arc by
    sinc of half the turn, along the way it heads halfway."""
    from coaxial.model.blocks import numpy as np
    start, way, across, angle, length, _leads = bough
    halfway, heads = angle * (1.5 - t / 2), angle * (1.5 - t)
    chord = float(length * t * np.sinc(angle * t / 2 / math.pi))
    return (tuple(start[k] + chord * (math.cos(halfway) * way[k] + math.sin(halfway) * across[k]) for k in range(3)),
            tuple(math.cos(heads) * way[k] + math.sin(heads) * across[k] for k in range(3)))


def murray(above, held):
    """The angle off its stem of a bough holding `held` of the `above` pages the stem carries where it leaves, by
    Murray's law - each bough's radius cubed as what it carries, the stem going on with the rest -: cos =
    (r0^4 + r1^4 - r2^4) / (2 r0^2 r1^2); 0 where it carries all, the stem going on as it."""
    r0, r1, r2 = (math.pow(n, 2.0 / 3.0) for n in (above, held, above - held))   # radii squared
    return math.acos((r0 * r0 + r1 * r1 - r2 * r2) / (2 * r0 * r1))


def slots(tree, n):
    """How many steps a bough `n` (0 the trunk) is laid in: one for each folder it carries (`owners`), then one
    for each kid out of it - the last at its end -, a page's tip one."""
    c = clades(tree)
    return sum(1 for o in owners(tree).values() if o == n) + max(1, len(c['kids'].get(None if n == 0 else n, ())))


@functools.lru_cache(maxsize=2)
def boughs(tree):
    """{0, a branch or a page: (start, way, across, angle, length, whether it leads)}: the tree's boughs, each an arc
    (`along`), the terminal at the origin, as a tree grows: the trunk (0) straight up, APART a step (`slots`); a
    bough's folders along it from its start, its kids out of it above them, a step each - the biggest lowest, as a
    tree's oldest -, the branch holding the most pages last, at its end, leading; each round the stem by the golden
    angle from the last (scene.GOLDEN, as leaves stand round a stem, none over another), out of it along its way
    and turning till it stands at the angle Murray's law gives what it holds of the pages the stem carries there
    (`murray`) - the leader, the rest, straight on, as the leader of a tree goes on -; as long as its own steps
    need, each other as far as the crown is wide where it leaves if that is further - a paraboloid's, RING times
    the root of the share of the pages from there up."""
    c, every = clades(tree), len(pages(tree))
    turn, out = [0], {0: ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), 0.0, APART * slots(tree, 0), True)}

    def held(n):
        return len(c['pages'].get(n, (n,)))

    def grow(node, bough):
        branches = [k for k in c['kids'][node] if k < -1]
        leader = max(branches, key=lambda k: (held(k), -k)) if branches else None
        kids = sorted((k for k in c['kids'][node] if k != leader), key=lambda k: (-held(k), k)) + [leader] * bool(branches)
        steps, above = slots(tree, 0 if node is None else node), held(node) if node else every
        for j, k in enumerate(kids, steps - len(kids) + 1):
            at, way = along(bough, j / steps)
            turn[0] += k != leader   # the leader no organ round the stem, the stem going on
            out[k] = (at, way, lean(way, turn[0] * scene.GOLDEN), murray(above, held(k)),
                      max(RING * math.sqrt(above / every) * (k != leader), APART * slots(tree, k)), k == leader)
            above -= held(k)
            if k < -1:
                grow(k, out[k])
    grow(None, out[0])
    return out


@functools.lru_cache(maxsize=2)
def grove(tree):
    """({node: where it stands}, [(from, to, bough, whether it leads)]): the tree laid as it grows (`boughs`), the
    terminal at the origin - a branch at its bough's end, a page at its twig's tip, each folder along the bough
    that carries it (`owners`) by its layer, the terminal's first -, each bough in STEPS straight pieces."""
    grown, own, users = boughs(tree), owners(tree), served(tree)
    at = {n: along(b, 1.0)[0] for n, b in grown.items() if n != 0}
    for n, b in grown.items():   # what each bough carries along it
        mine = sorted((f for f in users if own[f] == n), key=lambda f: (layer(tree, f), f))
        at.update((f, along(b, (j + 1) / slots(tree, n))[0]) for j, f in enumerate(mine))
    return at, [(along(b, i / STEPS)[0], along(b, (i + 1) / STEPS)[0], n, b[5]) for n, b in grown.items()
                for i in range(STEPS)]


def placed(tree, upright, chosen=None, cam=None):
    """The scene the atlas draws by application (scene.picture), the tree's terminal (`upright`'s) its foot,
    `chosen` the branch or page open (None: none): the tree as it grows (`grove`), in the world - turned, the
    view goes round it. Lit: a branch with its pages and their folders, a page
    with its folders, named; the folders of none lit grown in a zoom key's step nearer than the frame. `at`
    {node: where it stands}, `flat`, `rings` {page: its folders, its own first}, `names` - a page its headline, a
    branch its heaviest folder's -, `briefs`, `groups`, `lefts` named on their left - left of the trunk as `cam`
    sees it -, and the trunk's top its `hub`."""
    from coaxial.model.blocks import numpy as np
    c, users, order, (laid, pieces) = clades(tree), served(tree), ordered(tree), grove(tree)
    root, sight = 0, cam or scene.camera()
    base, right = upright['at'][root], scene.view(sight, 1, 1)['r']

    def at(p):
        return tuple(base[k] + p[k] for k in range(3))
    place = {root: base, **{n: at(p) for n, p in laid.items()}}
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
    parts = [(at(a), at(b), scene.EDGE if n == root or n in lit else scene.DIM_EDGE, leads) for a, b, n, leads in pieces]
    focus, forks = root if chosen is None else chosen, sorted(c['pages'], reverse=True)
    nodes = [root] + forks + order + drawn
    ink = [scene.OPEN if n == focus else scene.NODE if n in lit else scene.DIM_NODE for n in nodes]
    discs = [scene.DISC[0]] + [FORK_DISC] * len(forks) + [PAGE_DISC] * len(order) \
        + [DISC + GROWTH * len(users[f]) for f in drawn]
    ends = np.array([s[0] for s in parts] + [s[1] for s in parts], float).reshape(2, -1, 3)
    names = {**nested.names(tree, sorted(users)), **{page: headline for page, headline, *_said in pages(tree)}}
    names.update((b, names[fs[0]]) for b, fs in c['by'].items())
    keys = [n for n in offered(tree, chosen) if n in nodes and n not in c['pages']]   # its own, no fork
    lefts = {n for n, p in place.items() if (p[0] - base[0]) * right[0] + (p[2] - base[2]) * right[2] < 0}
    named = sorted((f for f in drawn if f in lit), key=lambda f: (len(users[f]), f))
    return {'tree': tree, 'focus': focus, 'at': place, 'out': {}, 'into': {}, 'names': names, 'lefts': lefts,
            'field': FIELD, 'panels': 1, 'upright': upright, 'chosen': chosen, 'hub': at(along(boughs(tree)[root], 1.0)[0]),
            'faced': (sight['yaw'], sight['pitch'], sight['distance']), 'rings': {page: offered(tree, page) for page in order},
            'briefs': {**{b: 'theirs alone: ' + ' '.join(names[f] for f in fs) for b, fs in c['by'].items()},
                       **{page: what for page, _headline, _key, what in pages(tree) if what}},
            'groups': ([focus] + keys + path + [root],   # the panels between; a fork named on the way open alone
                       [n for n in order if n in lit] + named + [n for n in order if n not in lit]),
            'flat': {'nodes': np.array(nodes, np.intp), 'pos': np.array([place[n] for n in nodes], float).reshape(-1, 3),
                     'disc': np.array(discs, float), 'ink': np.array(ink, np.uint8), 'lit': lit, 'from': ends[0],
                     'to': ends[1], 'segink': np.array([s[2] for s in parts], np.uint8),
                     'thick': np.array([s[3] for s in parts], bool)}}


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
