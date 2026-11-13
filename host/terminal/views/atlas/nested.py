"""A mirror node opened as one nears it: its parts in a frame, the data through them, what each uses.

The mirror (atlas/mirror.py) draws a node of the tree as its sign and its name. Nearer, one holding parts
opens where its frame fits clear of all else (`scale`): its children the data passes through
(tools/dev/atlas_flow.py) and those they use - the atlas's imports, includes and names, the used one's data
into its user -, each hung from the nearest already in it, laid in columns as the wire is (schematic.wire);
its ports on its frame where its lanes meet it - left and right on its row, top and bottom on its column -,
the data on through them to the part that takes it (`nest`). An open node's parts open in turn: a layer its
folders, a folder its files (`opened`). A node's sign says what it is (SIGNS): an estimator, the robot's eyes,
senses, law and drives, the world, the wire, the board, a process, what wears.
"""
import functools
import math

from terminal.views.atlas import detail, scene, schematic
from tools.dev import atlas_flow
from tools.dev.atlas_tree import FILE, TIES

#: Each kind's ink (atlas_flow.KINDS: the pass, the wire, a call, what comes back); a part's use of another's.
INKS, USES = (scene.TITLE, scene.NOTE, scene.TIE_OUT, scene.HIT), scene.EDGE
#: A node's sign by the first word of its path naming what it is, its own name first: an estimator or an
#: observer, the robot's eyes, senses, law and drives, the world, the wire, the board, a process, what
#: wears; a dot for the rest.
SIGNS = (('ekf', '◈'), ('estimat', '◈'), ('observ', '◈'), ('kalman', '◈'), ('filter', '◈'),
         ('eye', '◉'), ('sight', '◉'), ('registry', '◉'), ('vision', '◉'), ('camera', '◉'),
         ('sens', '◎'), ('edge', '◎'), ('grip', '◎'), ('felt', '◎'), ('imu', '◎'), ('angle', '◎'), ('adc', '◎'),
         ('thermal', '◎'), ('law', '⎔'), ('modal', '⎔'), ('se2', '⎔'), ('latent', '⎔'), ('impedance', '⎔'),
         ('stack', '⎔'), ('wrench', '⎔'), ('chain', '⎔'), ('ctrl', '⎔'), ('motion', '⊛'), ('drive', '⊛'),
         ('motor', '⊛'), ('gate', '⊛'), ('pwm', '⊛'), ('power', '⊛'), ('world', '⊕'), ('twin', '⊕'),
         ('physics', '⊕'), ('wire', '⌁'), ('comm', '⌁'), ('link', '⌁'), ('uart', '⌁'), ('protocol', '⌁'),
         ('shtp', '⌁'), ('cmd', '⌁'), ('board', '▣'), ('process', '⟳'), ('worker', '⟳'), ('wear', '⧖'),
         ('watch', '⧖'), ('identify', '⧖'))
#: The ports a lane meets an open node by, on its frame's left, right, top and bottom, and the way out of it
#: from each; the cells kept clear about a frame - a lane outside on its way to a port runs there.
PORTS, CLEAR = {'L': -1, 'R': -2, 'T': -3, 'B': -4}, 1
OUTWARD = {-1: (-1, 0), -2: (1, 0), -3: (0, -1), -4: (0, 1)}


def names(tree, nodes):
    """{node: its name}: its own, the folder it is in before it where two share one."""
    own = {n: scene.short(tree, n) for n in nodes}
    shared = {s for s in own.values() if list(own.values()).count(s) > 1}
    return {n: '/'.join(tree.id[n].rstrip('/').split('/')[-2:]) + s[len(s.rstrip('/')):] if s in shared else s
            for n, s in own.items()}


def sign(tree, node):
    """`node`'s sign (SIGNS): of its path's parts, its own first, the first holding a word that names one."""
    for part in reversed(tree.id[node].lower().rstrip('/').split('/')):
        for word, glyph in SIGNS:
            if word in part:
                return glyph
    return '●'


def made(tree, node):
    """Where a word is made, as the tree writes it: its file and its function's name."""
    return '%s › %s' % (scene.short(tree, tree.up[node]), tree.id[node].rpartition('::')[2]) if node >= 0 else ''


@functools.lru_cache(maxsize=2)
def _kids(tree):
    """{node: its children}."""
    out = {}
    for n, up in enumerate(tree.up):
        out.setdefault(up, set()).add(n)
    return out


@functools.lru_cache(maxsize=2)
def _uses(tree):
    """{node: [(user, used)]}: the atlas's ties of use - imports, includes, names - by their two ends' nearest
    common ancestor, the node whose parts they join."""
    out = {}
    for s, d, t in zip(tree.src, tree.dst, tree.tie):
        above, m = set(), s
        while m >= 0:
            above.add(m)
            m = tree.up[m]
        m = d
        while m >= 0 and m not in above:
            m = tree.up[m]
        if TIES[t] != 'wire' and m >= 0 and m not in (s, d):
            out.setdefault(m, []).append((s, d))
    return out


def side(mark, corner):
    """The port (PORTS) a lane meets a node by: from the node's sign (`mark`) to its corner beside it."""
    dx, dy = corner[0] - mark[0], corner[1] - mark[1]
    return PORTS['R' if dx > 0 else 'L'] if dy == 0 else PORTS['B' if dy > 0 else 'T']


def nest(level, node):
    """The node `node` of `level` (the mirror, or a nest) opened, or None where it holds fewer than two parts:
    a level of its own - `marks` {part or port: its cell}, `own` and `named` its parts' names and their
    offsets, `lanes` (as the mirror's) and `said` what each carries, `dots` [(lane, forward)] a flow, `rings`
    {part: its flows}, `owner` (a node of the tree, out of it or in -> the part holding it, or the port its
    flow passes), `frame` (x0, y0, x1, y1), `ports` {(from, to): the port a flow that way between it and a node
    beside it passes} and `port` {port: its cell on the frame}, `anchor` the cell laid where the node is and
    `box` its frame's reach from there - in cells of its own; as it opens (`opened`) its `scale`, `origin` and
    `unit`: the mirror's cell its anchor is at, its cells a cell of its own."""
    tree, kids, marks = level['tree'], _kids(level['tree']).get(node, set()), level['marks']
    ports = {}   # {(from, to): the port a flow that way between the node and one beside it passes}
    for a, b, corners, *_rest in level['lanes']:
        if node in (a, b):
            ports[(a, b)] = side(marks[node], corners[0] if a == node else corners[-1])
    for (a, b), g in list(ports.items()):   # a lane's way back where no lane goes so
        ports.setdefault((b, a), g)

    def owner(n, out=True):   # its part of `node`, else the port its flow - out of `node`, or in - passes
        m = n
        while m >= 0 and m not in kids:
            m = tree.up[m]
        far = level['owner'](n, out) if m < 0 else None
        return m if m >= 0 else ports.get((node, far) if out else (far, node))
    data, took, maker = {}, atlas_flow.takers(tree), {label: made(tree, n) for label, n in atlas_flow.makers(tree).items()}
    for fa, fb, kind, label in level['flow']:
        a, b = owner(fa, False), owner(fb)
        if a is not None and b is not None and a != b and max(a, b) >= 0:
            data.setdefault((a, b, kind), []).append(label)
    kept = level['reach']   # a page's files alone, where the mirror keeps to one (atlas/apps.py)
    ties = [(s, d) for s, d in _uses(tree).get(node, ()) if kept is None or s in kept and d in kept]
    used = {(owner(d), owner(s)) for s, d in ties}   # the used one's data into its user
    bound = {(a, b) for s, d in (_uses(tree).get(level['node'], ()) if 'up' in level else ())   # through a port
             if kept is None or s in kept and d in kept
             for a, b in ((owner(d, False), owner(s)),) if a is not None and b is not None and (a < 0) != (b < 0)}
    near = {}
    for d, s in used:
        near.setdefault(d, set()).add(s)
        near.setdefault(s, set()).add(d)
    stations = sorted({n for a, b, _k in data for n in (a, b) if n >= 0})
    queue = stations or sorted(near, key=lambda n: (-len(near[n]), n))[:1]
    hung, seen = [], set(queue)
    for p in queue:   # the list grows as it is read: each part from the nearest already in
        for r in sorted(near.get(p, set()) - seen):
            seen.add(r)
            queue.append(r)
            hung.append((p, r))
    parts = sorted(seen)
    bound = {(a, b) for a, b in bound if max(a, b) in seen}
    gates = sorted({n for a, b, _k in data for n in (a, b) if n < 0} | {min(a, b) for a, b in bound})
    if len(parts) < 2:
        return None
    own = names(tree, parts)
    L, R, T, B = (PORTS[k] for k in 'LRTB')
    rightward = sum(len(v) for (a, b, _k), v in data.items() if a == L or b == R) \
        >= sum(len(v) for (a, b, _k), v in data.items() if a == R or b == L)
    depth = schematic.depths(sorted({n for a, b, _k in data for n in (a, b)}), sorted({(a, b) for a, b, _k in data})) \
        if stations else {queue[0]: 0}
    top = max(depth.values())
    col = {n: d if rightward else top - d for n, d in depth.items()}
    for p, r in hung:   # a part beside the one it hangs from: before it where its data goes into it
        col[r] = col[p] - 1 if (r, p) in used else col[p] + 1
    for a, b in sorted(bound):   # a port used only beside the part it reaches
        g, c = (a, b) if a < 0 else (b, a)
        col.setdefault(g, col[c] - 1 if a < 0 else col[c] + 1)
    if T in col and B in col:
        col[B] = col[T]
    tb = col.get(T, col.get(B))
    if tb is not None and any(col[n] == tb for a, b in [(a, b) for a, b, _k in data] + sorted(bound) if {a, b} & {T, B}
                              for n in (a, b) if n >= 0):
        col = {n: c + (c >= tb and n not in (T, B)) for n, c in col.items()}   # a column of its own
    low = min(c for n, c in col.items() if n not in (L, R))
    col = {n: c - low + (L in col) for n, c in col.items()}
    col.update({L: 0} if L in col else {})
    col.update({R: max(c for n, c in col.items() if n != R) + 1} if R in col else {})
    pairs = {}
    for a, b, kind in data:
        u, v = (a, b) if col[a] < col[b] else (b, a)
        pairs.setdefault((u, v), ([], []))[(u, v) != (a, b)].append((kind, a, b))
    for p, r in hung + sorted(bound):
        pairs.setdefault((p, r) if col[p] < col[r] else (r, p), ([], []))
    used |= bound
    marks_, routes, (width, tall) = schematic.wire(parts + gates, sorted(pairs), (), {**own, **dict.fromkeys(gates, '')},
                                                   col, (T if T in col else None, B if B in col else None))
    x0, x1 = -1 - (L not in col), marks_[R][0] + 1 if R in col else width + 1
    y0, y1 = -1, tall
    cx, cy = (x0 + x1) // 2, marks_[L if L in col else R][1] if {L, R} & set(col) else (y0 + y1) // 2
    port = {g: (x0, cy) if g == L else (x1, cy) if g == R else (marks_[g][0] if g in marks_ else cx, y0 if g == T else y1)
            for g in set(ports.values())}   # every side a lane meets it by, a port
    # Each lane its flows each way (the pass's heavy) and what it carries; a part's use of another's in its ink.
    lanes, said, dots, rings, named = [], [], [], {n: [] for n in parts}, {n: (2, 0) for n in parts}
    shown = {**own, **{g: _beyond(level, node, ports, g) for g in gates}}

    def inked(way):
        kinds = [kind for kind, _a, _b in way]
        return (INKS[-1] if len(INKS) - 1 in kinds else INKS[kinds[0]]) if kinds else None
    for u, v, corners, _heavy in routes:
        fore, back = pairs[(u, v)]
        ways = []
        for way in (fore, back):
            told = []
            for kind, a, b in way:
                words = sorted(set(data[(a, b, kind)]))
                told.append((kind, [(w, maker[w], detail.took(tree, [t for t in took.get((kind, w), ())
                                                                       if t >= 0 and owner(t) == b]))
                                    if w in maker else (w, None, None) for w in words]))
                arrows = ('↺', '↺') if kind == len(INKS) - 1 else ('→', '←')
                text, by = ', '.join(words), ', '.join(sorted({maker[w] for w in words if maker.get(w)}))
                for end, other, arrow in ((a, b, arrows[0]), (b, a, arrows[1])):
                    if end >= 0:
                        rings[end].append((arrow, text, shown[other], by))
                dots.append((len(lanes), way is fore))
            ways.append(told)
        if fore or back:
            lanes.append((u, v, corners, inked(fore) or inked(back), inked(fore), inked(back),
                          any(kind == 0 for way in (fore, back) for kind, _a, _b in way)))
        else:   # one uses the other: the used one's data into its user
            for d, s in ((u, v), (v, u)):
                for end, other, arrow in ((s, d, '←'), (d, s, '→')):
                    if (d, s) in used and end >= 0:
                        rings[end].append((arrow, 'used', shown[other], ''))
            lanes.append((u, v, corners, USES, USES if (u, v) in used else None, USES if (v, u) in used else None,
                          False))
        said.append(tuple(ways))
    out = {'tree': tree, 'node': node, 'marks': {**{n: marks_[n] for n in parts}, **port}, 'own': own, 'named': named,
           'lanes': lanes, 'said': said, 'dots': dots, 'rings': rings, 'owner': owner, 'flow': level['flow'],
           'frame': (x0, y0, x1, y1), 'ports': ports, 'port': port, 'anchor': (cx, cy), 'nests': {},
           'inks': dict.fromkeys(parts, scene.EDGE), 'up': level, 'reach': kept}
    out['box'] = _box(out, cx)
    return out


def _beyond(level, node, ports, gate):
    """The name of what lies through `node`'s port `gate` (`ports` {(from, to): port}) on `level`: a node of
    it, or one of its own ports - what lies beyond that."""
    far = next((a if b == node else b for (a, b), g in ports.items() if g == gate), None)
    if far is None or far >= 0:
        return level['own'].get(far, '')
    return _beyond(level['up'], level['node'], level['ports'], far)


def _box(inner, cx):
    """(left, right, up, down): the cells of nest `inner`'s frame, clear about it, from its column `cx`."""
    (x0, y0, x1, y1), cy = inner['frame'], inner['anchor'][1]
    return cx - x0 + CLEAR, x1 - cx + CLEAR, cy - y0 + CLEAR, y1 - cy + CLEAR


def scale(level, node, boxes):
    """The zoom over `level`'s cells - terminal cells to one of them - from which `node`'s frame, its box
    (`boxes` {node: (left, right, up, down) terminal cells from where it is}) clear about it, keeps clear of
    every other node's sign and name, of every lane but where one meets it, of `level`'s own frame, and within
    half the way to a node beside it that opens too, across or down whichever lets the two open sooner, so no
    two frames meet; a zoom key's step at least, inf where it never does."""
    (nx, ny), (left, right, up, down), marks = level['marks'][node], boxes[node], level['marks']

    def clear(lo, hi, olo, ohi):   # the least zoom a thing - lo..hi zoomed, olo..ohi not - is clear from
        ways = ((lo[0] - nx, right - olo[0]), (nx - hi[0], left + ohi[0]), (lo[1] - ny, down - olo[1]),
                (ny - hi[1], up + ohi[1]))
        return min(max(h / g, 0.0) if g > 0 else 0.0 if g == 0 and h < 0 else math.inf for g, h in ways)
    need = [scene.ZOOM]
    for m, (mx, my) in marks.items():
        if m != node and m >= 0:
            dx, dy = level['named'].get(m, (2, 0))
            need += [clear((mx, my), (mx, my), (0, 0), (0, 0)),
                     clear((mx, my), (mx, my), (dx, dy), (dx + len(level['own'][m]) - 1, dy))]
            if m in boxes:   # half the way to it, across or down, as it takes the same way to this one
                ax, ay, (ml, mr, mu, md) = mx - nx, my - ny, boxes[m]
                ways = [max(right if ax > 0 else left, ml if ax > 0 else mr) / abs(ax)] if ax else []
                ways += [max(down if ay > 0 else up, mu if ay > 0 else md) / abs(ay)] if ay else []
                pick = min(range(len(ways)), key=lambda i: ways[i])
                need.append(2 * ((right if ax > 0 else left) / abs(ax) if ax and pick == 0
                                 else (down if ay > 0 else up) / abs(ay)))
    for a, b, corners, *_rest in level['lanes']:
        points = [(marks[a], (corners[0][0] - marks[a][0], corners[0][1] - marks[a][1]))] \
            + [(c, (0, 0)) for c in corners[1:-1]] + [(marks[b], (corners[-1][0] - marks[b][0], corners[-1][1] - marks[b][1]))]
        for i, ((p, op), (q, oq)) in enumerate(zip(points, points[1:])):
            if not (i == 0 and a == node or i == len(points) - 2 and b == node):
                need.append(clear((min(p[0], q[0]), min(p[1], q[1])), (max(p[0], q[0]), max(p[1], q[1])),
                                  (min(op[0], oq[0]), min(op[1], oq[1])), (max(op[0], oq[0]), max(op[1], oq[1]))))
    if level.get('frame'):
        x0, y0, x1, y1 = level['frame']
        need += [clear(lo, hi, (0, 0), (0, 0)) for lo, hi in (((x0, y0), (x1, y0)), ((x0, y1), (x1, y1)),
                                                              ((x0, y0), (x0, y1)), ((x1, y0), (x1, y1)))]
    return max(need)


def opened(mirror, z):
    """[(nest, terminal cells to one of its cells)] the nodes open at zoom `z` (terminal cells to a cell of
    the mirror), the outer before those in it: one holding parts opens from its `scale`, a level's all laid
    (`nest`) once the zoom is a key's step in, then their scales with each other's frames known."""
    out = []

    def walk(level, near):
        if near < scene.ZOOM:
            return
        if not level['nests']:
            for node in level['marks']:
                if node >= 0 and level['tree'].kind[node] < FILE:
                    level['nests'][node] = nest(level, node)
            boxes = {n: inner['box'] for n, inner in level['nests'].items() if inner}
            for n, inner in level['nests'].items():   # laid where the node is from the column it opens soonest by:
                if inner:   # each bound on it falls or rises as that column goes right, so coarse, then fine
                    (x0, _y0, x1, _y1), cy = inner['frame'], inner['anchor'][1]

                    def soon(c, n=n, inner=inner):
                        return scale(level, n, {**boxes, n: _box(inner, c)}), abs(2 * c - x0 - x1)
                    step = max(1, (x1 - x0) // 16)
                    cx = min(range(x0, x1 + 1, step), key=soon)
                    cx = min(range(max(x0, cx - step + 1), min(x1, cx + step - 1) + 1), key=soon)
                    inner['anchor'], inner['box'] = (cx, cy), _box(inner, cx)
            boxes = {n: inner['box'] for n, inner in level['nests'].items() if inner}
            (ox, oy), (ax, ay), unit = level['origin'], level['anchor'], level['unit']
            for n, inner in level['nests'].items():
                if inner:
                    inner['scale'] = scale(level, n, boxes)
                    inner['origin'] = (ox + (level['marks'][n][0] - ax) * unit, oy + (level['marks'][n][1] - ay) * unit)
                    inner['unit'] = unit / inner['scale']
        for node in sorted(level['nests']):
            inner = level['nests'][node]
            if inner and near >= inner['scale']:
                out.append((inner, near / inner['scale']))
                walk(inner, near / inner['scale'])
    walk(mirror, z)
    return out


def composed(mirror, levels, places, cols, rows):
    """mirror.laid's and its flows drawn (mirror._kept: `levels` [(level, its zoom)], `places` their cells
    where the camera puts them): every level drawn at once - the mirror's, then each open node's frame (its
    sign and name on top, its ports where its lanes meet it, the lanes outside ending on them through its
    margin) and its parts within -, each open node's out of the background over a zoom key's step."""
    at = {id(level): dict(zip(level['anchors'], zip(c.tolist(), r.tolist()))) for (level, _n), (c, r) in zip(levels, places)}
    shown, tree = {id(level) for level, _n in levels}, mirror['tree']
    lanes, keys, said, texts, slots, flows = [], [], [], [], [], []
    for level, _near in levels:
        pos, marks, nests = at[id(level)], level['marks'], level['nests']
        opened = {n: nests[n] for n in nests if nests[n] and id(nests[n]) in shown}

        def end(node, cell, pos=pos, marks=marks):   # a lane's end where it stands from its node's sign
            (sx, sy), (mx, my) = pos[marks[node]], marks[node]
            return (sx + cell[0] - mx, sy + cell[1] - my)

        def gate(node, lane, toward):   # an open node's port its lane meets it by, out, along its margin
            inner = opened[node]
            g = inner['ports'][lane]
            (px, py), (ox, oy) = at[id(inner)][inner['port'][g]], OUTWARD[g]
            return [(px, py), (px + ox, py + oy), (toward[0], py + oy) if oy else (px + ox, toward[1])], (px - ox, py - oy)
        first = len(keys)
        for k, (a, b, cs, ink, into, out, heavy) in enumerate(level['lanes']):
            route, ends = [end(a, cs[0])] + [pos[c] for c in cs[1:-1]] + [end(b, cs[-1])], [pos[marks[a]], pos[marks[b]]]
            if a in opened:
                (start, ends[0]) = gate(a, (a, b), route[1])
                route = start + route[1:]
            if b in opened:
                (stop, ends[1]) = gate(b, (a, b), route[-2])
                route = route[:-1] + stop[::-1]
            lanes.append((*ends, route, ink, into, out, heavy))
            keys.append(k if level is mirror else (id(level), k))
            said.append(level['said'][k])
        flows += [(keys[first + k], forward, level['lanes'][k][1 if forward else 0]) for k, forward in level['dots']]
        if level is mirror:
            hidden = {marks[n] for n in opened}
            texts += [(pos[(x, y)][0] + dx, pos[(x, y)][1] + dy, text, ink, node)
                      for x, y, dx, dy, text, ink, node in mirror['texts'] if (x, y) not in hidden]
            for x, y, dx, dy, k, d, way in mirror['slots']:   # an open node's at the port its lane meets it by
                a, b = mirror['lanes'][k][:2]
                inner = opened.get(b if d == 0 else a)
                sx, sy = at[id(inner)][inner['port'][inner['ports'][(a, b)]]] if inner else pos[(x, y)]
                slots.append((sx + dx, sy + dy, way, k, mirror['said'][k], d))
            continue
        (x0, y0), (x1, y1) = pos[level['frame'][:2]], pos[level['frame'][2:]]
        gates, up, node = {pos[c] for c in level['port'].values()}, level['up'], level['node']
        frame = [((x0, y0), '╭'), ((x1, y0), '╮'), ((x0, y1), '╰'), ((x1, y1), '╯')] \
            + [((x, y), '─') for x in range(max(x0 + 1, 0), min(x1, cols)) for y in (y0, y1)] \
            + [((x, y), '│') for y in range(max(y0 + 1, 0), min(y1, rows)) for x in (x0, x1)]
        texts += [(x, y, ch, scene.NAME_DIM, None) for (x, y), ch in frame if (x, y) not in gates]
        texts += [(x0 + 2, y0, ' ', scene.NAME_DIM, None), (x0 + 3, y0, sign(tree, node), up['inks'][node], node),
                  (x0 + 4, y0, ' ', scene.NAME_DIM, None), (x0 + 5, y0, up['own'][node] + ' ', scene.NAME, node)]
        for n in level['own']:
            if n not in opened:
                (sx, sy), (dx, dy) = pos[marks[n]], level['named'][n]
                texts += [(sx, sy, sign(tree, n), level['inks'][n], n), (sx + dx, sy + dy, level['own'][n], scene.NAME, n)]
    cells, paths, runs = schematic.laid(lanes, [t[:4] for t in texts], cols, rows)
    spots = {}
    for x, y, text, _ink, node in texts:
        if node is not None:
            got, name = spots.get(node, (None, []))
            spots[node] = ((x, y), name) if got is None else (got, [(x + k, y) for k in range(len(text.rstrip()))])
    framed_ = {level['node'] for level, _n in levels[1:]}   # an open node's brief none: it is its frame
    words = detail.detailed(mirror, cells, list(zip(keys, said)), runs, slots,
                            {n: s for n, s in spots.items() if n not in framed_}, cols - mirror['beside'], rows)
    for level, near in levels[1:]:   # out of the background
        if near < scene.ZOOM:
            (x0, y0), (x1, y1), ink = at[id(level)][level['frame'][:2]], at[id(level)][level['frame'][2:]], detail.faded(near)
            cells.update({c: (ch, ink) for c, (ch, _i) in cells.items() if x0 <= c[0] <= x1 and y0 <= c[1] <= y1})
    return cells, dict(zip(keys, paths)), spots, words, flows
