"""The atlas's mirror world: what flows between its files, laid out the way the data goes.

M turns the page to it: of the nodes shown, those a flow joins (tools/dev/atlas_flow.py, a flow's ends taken
to the nearest node shown) and nothing else, laid out by the flow, not by what holds what. Her loop is a
ring - of her pass and what comes back through the world, the longest run back to where it began, its
stages in order, what passes written by each arc -, the rest of her pass hanging off it; the wire is columns
from left to right - the host's end, the command handlers, the board's functions -, each column in the
order that crosses least, the readings running back under their asks; where nothing comes back, her pass
is columns too. The two stand side by side in a plane under the trunk's foot, each flow in its kind's ink.
The camera faces it whole while M holds (`framed`), every node named (`labels`), a bright dot running every
flow from its maker to its taker (`pulsed`), a panel at the right the flows of the node nearest the
viewport (`over`).
"""
import math

from terminal.views.atlas import leaves, scene
from terminal.views.atlas.panel import fit

#: Each kind's ink (atlas_flow.KINDS: her pass, the wire, what comes back); a flow's segments; the seconds a
#: dot takes along one and how far apart the flows' dots start, a share of the way (the golden one).
INKS, SPAN, TRAVEL_S, PHASE = (scene.TITLE, scene.NOTE, scene.HIT), 12, 2.0, 0.618
#: The flow world's room, world units: how far under the trunk's foot its plane stands; a ring's stages apart
#: along it, and a stage hanging off it out from the one it hangs from; a column's rows apart; the columns
#: apart; the ring and the columns apart; a word off its curve; how far what comes back bows under what goes
#: the same way, a share of a row; how far a curve off the ring bows, a share of its length; the sweeps
#: that order the columns.
DEPTH, STAGE, ROW, COLUMN, APART, OFF, UNDER, BOW, SWEEPS = 6.0, 1.0, 0.6, 2.4, 1.8, 0.35, 0.3, 0.2, 4
#: A word turns beside its point facing more across than this, else above or below it (an eighth round);
#: the rows kept free above and below the world framed - a name over its top node, the clock under it all.
ASIDE, MARGIN = math.sin(math.pi / 8), 2
#: Where a word may stand, its point's (right, left, above, below), in the order tried by the way it faces.
TRIED = {0: (0, 2, 3, 1), 1: (1, 2, 3, 0), 2: (2, 3, 0, 1), 3: (3, 2, 0, 1)}


def _angle(k, n):
    """The angle of the `k`th of `n` stages round the ring: the first at its top, on clockwise."""
    return math.pi / 2 - 2 * math.pi * k / n


def _loop(stages, edges, back):
    """Her loop: of the runs of `edges` {(a, b)} among `stages` back to where they began, the longest - one
    through what comes back (`back`) first among equals -, its stages in order from its earliest in her
    pass; none where no run comes back."""
    after, rank, best = {n: sorted(b for a, b in edges if a == n) for n in stages}, {n: k for k, n in enumerate(stages)}, ()
    for s in stages:   # each run once, from its earliest stage
        todo: list[tuple[int, ...]] = [(s,)]
        while todo:
            path = todo.pop()
            for m in after[path[-1]]:
                if m == s and len(path) > 1:
                    best = max(best, (len(path), any((p, q) in back for p, q in zip(path, path[1:] + (s,))), path))
                elif m not in path and rank[m] > rank[s]:
                    todo.append(path + (m,))
    return list(best[2]) if best else []


def _hung(loop, stages, edges):
    """{stage off `loop`: (the stage it hangs from, its hops out)}, the nearest first: by `edges` {(a, b)}
    either way from the loop out, breadth first; one joined to none hung from the loop's first."""
    near = {n: sorted({b for a, b in edges if a == n} | {a for a, b in edges if b == n}) for n in stages}
    hops, hung, reached = dict.fromkeys(loop, 0), {}, list(loop)
    for n in reached:   # the list grows as it is read
        for m in near[n]:
            if m not in hops:
                hops[m], hung[m] = hops[n] + 1, (n, hops[n] + 1)
                reached.append(m)
    hung.update({m: (loop[0], 1) for m in stages if m not in hops})
    return hung


def _ring(loop, hung):
    """({node: (u, v)} about the ring's middle, {node: its name's facing}, the radius): `loop` round a circle
    in order - the first at its top, on clockwise, STAGE apart along it -, each of `hung` (`_hung`) STAGE a
    hop further out than the stage it hangs from, fanned beside its siblings; names outward, but a loop
    stage's that others hang from, inward."""
    n, kin = len(loop), {}
    radius, step = max(STAGE, n * STAGE / (2 * math.pi)), 2 * math.pi / n
    angle = {s: _angle(k, n) for k, s in enumerate(loop)}
    for m, (parent, _hops) in hung.items():
        kin.setdefault(parent, []).append(m)
    for m, (parent, hops) in hung.items():   # the nearest first, so the stage it hangs from has its angle
        angle[m] = angle[parent] + (kin[parent].index(m) - (len(kin[parent]) - 1) / 2) * step / (2 * hops)
    out = {m: radius + hops * STAGE for m, (_p, hops) in hung.items()}
    at = {s: (out.get(s, radius) * math.cos(a), out.get(s, radius) * math.sin(a)) for s, a in angle.items()}
    inward = {s: -1.0 if s in kin and s not in hung else 1.0 for s in angle}
    return at, {s: (inward[s] * math.cos(a), inward[s] * math.sin(a)) for s, a in angle.items()}, radius


def _onward(nodes, edges, starts):
    """`edges` (from, to) less those that close a loop: back to a node still being followed, depth first from
    `starts`, then from each node not yet reached, in order."""
    after, seen, back = {n: sorted({b for a, b in edges if a == n}) for n in nodes}, {}, set()
    for root in [*starts, *sorted(nodes)]:
        if root in seen:
            continue
        seen[root], todo = True, [(root, iter(after.get(root, ())))]
        while todo:   # seen: True while followed, False once done
            n, ahead = todo[-1]
            m = next(ahead, None)
            if m is None:
                seen[n] = False
                todo.pop()
            elif seen.get(m):
                back.add((n, m))
            elif m not in seen:
                seen[m] = True
                todo.append((m, iter(after.get(m, ()))))
    return [e for e in edges if e not in back]


def _columns(nodes, edges, fixed):
    """({node: (u, v)}, the last column's u): `nodes` in columns COLUMN apart from u 0, each the longest run
    along `edges` (from, to) - less those closing a loop (`_onward`) - from where they start: `fixed`
    ({node: (u, v)} laid elsewhere, a column before the first) or a node none reaches; a column's rows ROW
    apart about v 0 in the order of their neighbours' mean rows (SWEEPS sweeps)."""
    starts = sorted(fixed) + sorted(n for n in nodes if not any(b == n for _a, b in edges))
    onward = _onward([*fixed, *nodes], edges, starts)
    left, depth = {n: {a for a, b in onward if b == n and a not in fixed} for n in nodes}, dict.fromkeys(fixed, 0)
    while left:   # no loop is left: some node has every edge in laid
        n = min(m for m in left if not left[m])
        depth[n] = 1 + max((depth[a] for a, b in onward if b == n), default=-1)
        del left[n]
        for into in left.values():
            into.discard(n)
    low, columns = min((depth[n] for n in nodes), default=0), {}
    near = {n: [b for a, b in edges if a == n] + [a for a, b in edges if b == n] for n in nodes}
    for n in sorted(nodes):
        columns.setdefault(depth[n] - low, []).append(n)
    pinned = {n: -v / ROW for n, (_u, v) in fixed.items()}

    def rows():
        return {**pinned, **{n: i - (len(c) - 1) / 2 for c in columns.values() for i, n in enumerate(c)}}
    for _ in range(SWEEPS):
        row = rows()
        for column in columns.values():
            column.sort(key=lambda n: (sum(row[m] for m in near[n]) / len(near[n]) if near[n] else row[n], n))
    row = rows()
    return {n: ((depth[n] - low) * COLUMN, -row[n] * ROW) for n in nodes}, max(columns, default=0) * COLUMN


def _round(center, radius, a, b, dip=0.0):
    """SPAN + 1 points round the circle about `center` from angle `a` to `b`, `dip` inside it at its middle."""
    return [(center[0] + r * math.cos(t), center[1] + r * math.sin(t)) for t, r in
            ((a + (b - a) * k / SPAN, radius - 4 * dip * k * (SPAN - k) / SPAN ** 2) for k in range(SPAN + 1))]


def _lane(p, q, lift=0.0):
    """SPAN + 1 points of an S from `p` to `q`, level at both ends as a column's lane is, `lift` off it at its
    middle."""
    return [(p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * (3 * t * t - 2 * t ** 3) + 4 * lift * t * (1 - t))
            for t in (k / SPAN for k in range(SPAN + 1))]


def _bezier(p, c, q):
    """SPAN + 1 points of the curve from `p` to `q` drawn toward `c` (a quadratic Bezier's)."""
    return [tuple((1 - t) ** 2 * p[k] + 2 * (1 - t) * t * c[k] + t * t * q[k] for k in range(2))
            for t in (k / SPAN for k in range(SPAN + 1))]


def _world(p):
    """A point (u, v) of the flow world's plane in the world: under the trunk's foot, facing a camera turned
    to none."""
    return (p[0], p[1] - DEPTH, 0.0)


def _names(tree, nodes):
    """{node: its name}: its own, the folder it is in before it where two share one."""
    own = {n: scene.short(tree, n) for n in nodes}
    shared = {s for s in own.values() if list(own.values()).count(s) > 1}
    return {n: '/'.join(tree.id[n].rstrip('/').split('/')[-2:]) + s[len(s.rstrip('/')):] if s in shared else s
            for n, s in own.items()}


def mirrored(placed, flow):
    """The flow world of the scene `placed` (show_atlas.placed), the module's head: its flat arrays laid as
    `scene.picture` reads them, `arcs` where each flow's segments start, `groups` none for it to name and
    `field` all in focus; `tags` [(point, word, ink, node, facing, disc)] its words - every node's name, what
    passes by her pass's curves -; `flows_out` and `flows_in` {node: flows} the chosen node's flows reach and
    come from; `rings` {node: its flows [(arrow, what passes, the name at the other end)]}; `loop` the ring's
    stages in order; `center` and `extent` (width, height)."""
    from coaxial.model.blocks import numpy as np
    tree, focus, shown, back = placed['tree'], placed['focus'], placed['at'], len(INKS) - 1
    lit = set(scene.chain(tree, focus) + [n for n in shown if focus in scene.chain(tree, n)]) if focus >= 0 else set()
    passed, stages = {}, []
    for a, b, kind, label in flow:
        a, b = scene.shown_of(tree, shown, a), scene.shown_of(tree, shown, b)
        if a >= 0 and b >= 0 and a != b:
            passed.setdefault((a, b, kind), []).append(label)
            stages += [n for n in (a, b) if kind == 0 and n not in stages]
    ahead = {(a, b) for a, b, kind in passed if kind == 0}
    home = {(a, b) for a, b, kind in passed if kind == back and a in stages and b in stages}
    loop = _loop(stages, ahead | home, home)
    ring, facing, radius = _ring(loop, _hung(loop, stages, ahead | home)) if loop else ({}, {}, 0.0)
    among = sorted({n for a, b, _k in passed for n in (a, b)} - set(ring))
    lanes, span = _columns(among, [(a, b) for a, b, kind in passed
                                   if kind != back and b in among and (a in among or a in ring)], ring)
    u0, u1 = min((u for u, _v in ring.values()), default=0.0), max((u for u, _v in ring.values()), default=0.0)
    width = u1 - u0 + (APART + span if ring and among else span)
    middle, start = (-width / 2 - u0, 0.0), width / 2 - span
    at = {**{n: (middle[0] + u, v) for n, (u, v) in ring.items()}, **{n: (start + u, v) for n, (u, v) in lanes.items()}}
    facing.update({n: (1.0, 0.0) if u == span else (0.0, 1.0) for n, (u, _v) in lanes.items()})
    index, rings, names = {n: k for k, n in enumerate(loop)}, {n: [] for n in at}, _names(tree, at)
    parts, arcs, said_on, out, into = [], [], [], {}, {}
    for (a, b, kind), labels in sorted(passed.items(), key=lambda kv: -len(kv[1])):
        said = ', '.join(sorted(set(labels)))
        if b in lit and a not in lit:
            into[a] = into.get(a, 0) + len(labels)
        elif a in lit and b not in lit:
            out[b] = out.get(b, 0) + len(labels)
        if kind != 1 and a in index and b in index and index[b] == (index[a] + 1) % len(loop):
            turn, step = _angle(index[a], len(loop)), 2 * math.pi / len(loop)
            points = _round(middle, radius, turn, turn - step, UNDER * ROW if kind == back and (a, b, 0) in passed else 0.0)
        elif a in index and b in index:   # across her loop: bowed toward its middle
            points = _bezier(at[a], tuple((at[a][k] + at[b][k]) / 4 + middle[k] / 2 for k in range(2)), at[b])
        elif a in ring and b in ring:     # off it: bowed to its left, the way back to the other side
            points = _bezier(at[a], ((at[a][0] + at[b][0]) / 2 - BOW * (at[b][1] - at[a][1]),
                                     (at[a][1] + at[b][1]) / 2 + BOW * (at[b][0] - at[a][0])), at[b])
        else:                             # a lane, under its ask where it comes back
            points = _lane(at[a], at[b], -UNDER * ROW if kind == back or at[b][0] < at[a][0] else 0.0)
        if kind != 1 and a in ring and b in ring:   # what passes, beside its curve on the side it bows
            mid, base = points[SPAN // 2], ((at[a][0] + at[b][0]) / 2, (at[a][1] + at[b][1]) / 2)
            d = math.hypot(mid[0] - base[0], mid[1] - base[1])
            face = ((mid[0] - base[0]) / d, (mid[1] - base[1]) / d) if d > 1e-9 else (0.0, 1.0)
            said_on.append((_world((mid[0] + OFF * face[0], mid[1] + OFF * face[1])),
                            ('↺ ' if kind == back else '') + said, scene.BRIEF, None, face, 0.0))
        arcs.append(len(parts))
        laid = [_world(p) for p in points]
        parts += [(p, q, INKS[kind]) for p, q in zip(laid, laid[1:])]
        arrows = ('↺', '↺') if kind == back else ('→', '←')
        rings[a].append((arrows[0], said, names[b]))
        rings[b].append((arrows[1], said, names[a]))
    nodes = sorted(at, key=lambda n: (n != focus, n not in lit, n))
    # A name beside its node; one turned into the ring stands OFF inside it, clear of the arcs either side.
    inside = {n for n in ring if facing[n][0] * (at[n][0] - middle[0]) + facing[n][1] * (at[n][1] - middle[1]) < 0}
    named = [(_world((at[n][0] + OFF * facing[n][0], at[n][1] + OFF * facing[n][1]) if n in inside else at[n]),
              names[n], scene.TOLD if n == focus else scene.NAME, n, facing[n],
              0.0 if n in inside else scene.DISC[tree.kind[n]]) for n in nodes]
    ink = [scene.OPEN if n == focus else (scene.OUT if out.get(n, 0) >= into.get(n, 0) else scene.IN)
           if n in out or n in into else scene.NODE if n in lit else scene.EDGE for n in nodes]
    ends = np.array([s[0] for s in parts] + [s[1] for s in parts], float).reshape(2, -1, 3)
    flat = {'nodes': np.array(nodes, np.intp), 'pos': np.array([_world(at[n]) for n in nodes], float).reshape(-1, 3),
            'disc': np.array([scene.DISC[tree.kind[n]] for n in nodes], float), 'ink': np.array(ink, np.uint8),
            'lit': lit, 'from': ends[0], 'to': ends[1], 'segink': np.array([s[2] for s in parts], np.uint8),
            'thick': np.zeros(len(parts), bool)}
    low, high = min((v for _u, v in at.values()), default=0.0), max((v for _u, v in at.values()), default=0.0)
    return {**placed, 'at': {n: _world(p) for n, p in at.items()}, 'out': {}, 'into': {}, 'flows_out': out,
            'flows_in': into, 'rings': rings, 'faint': [], 'flat': flat, 'arcs': np.array(arcs, np.intp),
            'tags': named + said_on, 'loop': loop, 'groups': ((), ()), 'field': math.inf,
            'center': _world((0.0, (low + high) / 2)), 'extent': (width, high - low)}


def framed(mirror, cols, rows, keyed=None):
    """(target, distance) facing the flow world whole on a drawing `cols` by `rows` cells: the active panel's
    width kept free at the right where a node has flows (leaves.width), the longest word beside it either
    side, MARGIN rows above and below; at the view's focal length (scene.view)."""
    keyed, sight = keyed or {}, scene.view(scene.camera(), cols, rows)
    width, height = mirror['extent']
    panel = leaves.width(cols) if mirror['rings'] else 0
    beside = max((len(t[1]) + len(keyed.get(t[3], '')) + 2 for t in mirror['tags']), default=0)
    across, down = max(1, cols - panel - 2 * beside), max(1, rows - 2 * MARGIN)
    distance = max(width * sight['focal'] / (2 * across), height * sight['focal'] / (4 * down), STAGE)
    x, y, z = mirror['center']
    return (x + panel * distance / sight['focal'], y, z), distance


def _put(out, taken, col, row, runs, cols, rows):
    """`runs` [(text, ink)] from (col, row) on in `out`; refused off the drawing, or where a cell it needs or
    one beside it is `taken` or in `out` already."""
    wide = sum(len(t) for t, _ink in runs)
    if row < 0 or row >= rows or col < 0 or col + wide > cols \
            or any((c, row) in taken or (c, row) in out for c in range(col - 1, col + wide + 1)):
        return False
    for t, ink in runs:
        for ch in t:
            out[(col, row)] = (ch, ink)
            col += 1
    return True


def labels(mirror, sight, cols, rows, keyed, taken=frozenset()):
    """{(col, row): (char, ink)}: the flow world's words (`mirrored`'s tags) on a drawing `cols` by `rows`
    cells seen through `sight` (scene.view) - the nodes' names, the keys' with their key, then what passes -,
    each beside its point the way it faces (out of the ring, above a column's, after the last column's),
    else on another side (TRIED), over no cell `taken` nor another word."""
    from coaxial.model.blocks import numpy as np
    tags = sorted(mirror['tags'], key=lambda t: (t[3] is None, t[3] not in keyed))
    if not tags:
        return {}
    rel = np.array([t[0] for t in tags], float) - np.array(sight['eye'])
    across, up, ahead = (rel @ np.array(sight[axis]) for axis in ('r', 'u', 'f'))
    out, w, h, focal = {}, sight['w'], sight['h'], sight['focal']
    for x, y, z, (_p, word, ink, node, (dx, dy), disc) in zip(across.tolist(), up.tolist(), ahead.tolist(), tags):
        if z < scene.NEAR:
            continue
        runs = ([(keyed[node] + ' ', scene.KEY)] if keyed.get(node) else []) + [(word, ink)]
        wide, px, py = sum(len(t) for t, _ink in runs), w / 2 + x * focal / z, h / 2 - y * focal / z
        reach = min(scene.DOTS[1], max(scene.DOTS[0], disc * focal / z)) if disc else 0.0
        mid, row = int(px // 2) - wide // 2, int(py // 4)
        spots = ((int((px + reach) // 2) + 1, row), (int((px - reach) // 2) - wide, row), (mid, row - 1), (mid, row + 1))
        faces = 0 if dx >= ASIDE else 1 if dx <= -ASIDE else 2 if dy > 0 else 3
        any(_put(out, taken, *spots[k], runs, cols, rows) for k in TRIED[faces])
    return out


def over(mirror, sight, keyed, fell, cols, rows, taken=frozenset()):
    """{(col, row): (char, ink)}: the flow world's words (`labels`, `keyed` {node: its key}), then at the right
    a panel listing the flows of the node nearest the viewport (leaves.panels: `lines`, `noted`), over none of
    them."""
    words = labels(mirror, sight, cols, rows, keyed, taken)
    named = {n: keyed.get(n, '') for n in mirror['rings']}
    return {**words, **leaves.panels(mirror['tree'], mirror['rings'], fell, named, cols, rows, taken | set(words),
                                     lines, noted, most=1, docked=True)}


def room(mirror, sight, cols):
    """`sight` (scene.view) on a drawing `cols` cells across narrowed to the room left of the panel (`over`,
    `framed`), the viewport's middle that room's."""
    return {**sight, 'w': 2 * (cols - leaves.width(cols))} if mirror['rings'] else sight


def pulsed(mirror, now):
    """The `mirror` (`mirrored`) at `now`: a bright dot on each flow's curve, run from its maker to its taker
    every TRAVEL_S, the curves' dots PHASE of the way apart - the way what flows goes, a loop going round."""
    from coaxial.model.blocks import numpy as np
    arcs, flat = mirror['arcs'], mirror['flat']
    if not len(arcs):
        return mirror
    ink = flat['segink'].copy()
    ink[arcs + ((now / TRAVEL_S + np.arange(len(arcs)) * PHASE) % 1.0 * SPAN).astype(np.intp)] = scene.TOLD
    return {**mirror, 'flat': {**flat, 'segink': ink}}


def lines(_tree, flows, wide, tall, active):
    """A mirror panel's lines, `wide` cells and `tall` lines at most: a flow a line - its arrow and the name
    at its other end, what passes beside -, what does not fit counted on the last."""
    shown = flows if len(flows) <= tall else flows[:tall - 1]
    named = min(max((len(name) + 2 for _a, _s, name in shown), default=0), wide // 2)
    out = [[(fit('%s %s' % (arrow, name), named).ljust(named + 1), scene.NAME if active
             else scene.NAME_DIM), (fit(said, wide - named - 1), scene.BRIEF)] for arrow, said, name in shown]
    return out + ([[('+%d more' % (len(flows) - len(shown)), scene.BRIEF)]] if len(shown) < len(flows) else [])


def noted(flows):
    """A mirror panel's note: how many flows."""
    return ' %d flow%s ' % (len(flows), '' if len(flows) == 1 else 's')
