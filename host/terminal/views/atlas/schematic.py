"""The mirror's schematic in the terminal's cells: laid out, in box-drawing lines, its words over.

What the mirror shows (atlas/mirror.py), knife sharp: no line cut into dots, none between two cells. Her
loop is laid round a rectangle in heavy lines (`ring`), the wire in columns whose lanes run square along
tracks between them (`wire`); a lane runs through its corners cell by cell (`walked`); where lanes join one
glyph carries every way out of the cell (a comb's teeth), where one only passes across another the upright
one runs on and the other breaks under it, so a crossing never reads as a join; a head ends a lane where a
flow goes in, pointing at the node it goes into; the words lie over the lines, and each lane's run where
nothing else is is found for what it carries (`laid`), a hung stage's beside its heads (`ring`'s slots).
A word keeps its place beside what it names in the terminal's cells, whatever the zoom spreads between them.
"""
from collections import Counter

#: The ways a line leaves a cell, bits; the glyph each set of them draws, the corners rounded - her loop's
#: heavy -; a lane's head by the way it points, her loop's filled.
UP, DOWN, LEFT, RIGHT = 1, 2, 4, 8
GLYPHS = {UP | DOWN: '│', LEFT | RIGHT: '─', DOWN | RIGHT: '╭', DOWN | LEFT: '╮', UP | RIGHT: '╰',
          UP | LEFT: '╯', UP | DOWN | RIGHT: '├', UP | DOWN | LEFT: '┤', DOWN | LEFT | RIGHT: '┬',
          UP | LEFT | RIGHT: '┴', UP | DOWN | LEFT | RIGHT: '┼', UP: '│', DOWN: '│', LEFT: '─', RIGHT: '─'}
HEAVY = {UP | DOWN: '┃', LEFT | RIGHT: '━', DOWN | RIGHT: '┏', DOWN | LEFT: '┓', UP | RIGHT: '┗',
         UP | LEFT: '┛', UP | DOWN | RIGHT: '┣', UP | DOWN | LEFT: '┫', DOWN | LEFT | RIGHT: '┳',
         UP | LEFT | RIGHT: '┻', UP | DOWN | LEFT | RIGHT: '╋', UP: '┃', DOWN: '┃', LEFT: '━', RIGHT: '━'}
HEADS, FILLED = {RIGHT: '▸', LEFT: '◂', DOWN: '▾', UP: '▴'}, {RIGHT: '▶', LEFT: '◀', DOWN: '▼', UP: '▲'}
#: The seconds a flow's packet takes along its lane, how far apart the flows' packets start, a share of the
#: way (the golden one, so no two run together), its cells; a stage hanging off the loop from the one it
#: hangs from, rows - two line cells between the heads of a lane both ways -; the sweeps that order the
#: wire's columns; the line cells at least before a head on her loop.
TRAVEL_S, PHASE, TRAIL, SPUR, SWEEPS, LEAD = 2.0, 0.618, 3, 5, 4, 3


def looped(stages, edges, back):
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


def hanging(loop, stages, edges):
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


def depths(nodes, edges):
    """{node: its column}: the longest run along `edges` (from, to) - less those closing a loop (`_onward`) -
    from a node none reaches."""
    onward = _onward(nodes, edges, sorted(n for n in nodes if not any(b == n for _a, b in edges)))
    left, depth = {n: {a for a, b in onward if b == n} for n in nodes}, {}
    while left:   # no loop is left: some node has every edge in laid
        n = min(m for m in left if not left[m])
        depth[n] = 1 + max((depth[a] for a, b in onward if b == n), default=-1)
        del left[n]
        for into in left.values():
            into.discard(n)
    return depth


def _ordered(columns, near):
    """`columns` [[node]] each sorted in place by its nodes' neighbours' mean places (`near` {node: [node]}),
    SWEEPS sweeps, a column's places about its middle."""
    for _ in range(SWEEPS):
        place = {n: i - (len(c) - 1) / 2 for c in columns for i, n in enumerate(c)}
        for column in columns:
            column.sort(key=lambda n: (sum(place[m] for m in near[n]) / len(near[n]) if near[n] else place[n], n))


def _tracked(hops, key, y):
    """The tracks of a channel's `hops` (from, to), left to right - `key` {hop: its track}, `y` {node: its
    row} -: a lane comes into its track on the left at its maker's row and leaves on the right at its
    taker's, crossing the tracks it passes whose runs hold that row; neighbours swapped while that crosses
    less, from the order of the tracks' nodes' rows."""
    keys = sorted({key[h] for h in hops}, key=lambda k: (y[k[0]], k[1]))
    ins = {k: {y[p] for p, q in hops if key[(p, q)] == k} for k in keys}   # a row's line once, however many lanes
    outs = {k: {y[q] for p, q in hops if key[(p, q)] == k} for k in keys}
    run = {k: (min(ins[k] | outs[k]), max(ins[k] | outs[k])) for k in keys}

    def crossed(a, b):   # a left of b
        return sum(run[a][0] <= r <= run[a][1] for r in ins[b]) + sum(run[b][0] <= r <= run[b][1] for r in outs[a])
    for _ in keys:
        for i in range(len(keys) - 1):
            if crossed(keys[i + 1], keys[i]) < crossed(keys[i], keys[i + 1]):
                keys[i], keys[i + 1] = keys[i + 1], keys[i]
    return keys


def ring(loop, hung, chords, said, names):
    """Her loop laid in cells from (0, 0): (marks {node: its dot's cell}, named {node: its name's offset from
    its dot}, routes [(a, b, corners, heavy)] - from the cell beside a's dot to the cell beside b's -, slots
    [(x, y, across, down, (a, b), way, grows)] where a hung stage's lane is written each way (0 a to b) - by
    a cell of the layout and its offset from it in the terminal's cells, the side it grows to -, (width,
    height)): its first half along the top left to right in heavy lines, the rest back along the bottom, the
    last up the left side to the first, LEAD line cells at least before a head, a stage's name inside, room
    for what passes in the line on its way - a side's on its run to the corner -; `hung` (`hanging`)
    straight out SPUR rows, what passes beside the head it goes into; `chords` [(a, b)] across it on rows of
    their own inside, a row clear either side; `said` (a, b) -> (what goes a to b, what comes back)."""
    n, t = len(loop), (len(loop) + 1) // 2
    top, bottom, kids, root = loop[:t], loop[t:], {}, {}
    for m, (p, _hops) in hung.items():
        kids.setdefault(p, []).append(m)
        root[m] = root.get(p, p)
    hangs = {s: [m for m in hung if root[m] == s] for s in loop}
    left = {s: max([len(said(hung[m][0], m)[0]) + 2 for m in hangs[s]], default=0) for s in loop}
    right = {s: max([len(said(hung[m][0], m)[1]) + 2 for m in hangs[s]] + [len(names[m]) + 2 for m in hangs[s]],
                    default=0) for s in loop}
    side, lead = [' · '.join(w for w in said(loop[i], loop[(i + 1) % n]) if w) for i in range(n)], 2 * LEAD + 4
    # The dots apart along the top and back along the bottom: the word in the line, the name, the hung words.
    gaps = [max(len(side[i]) + lead, len(names[top[i]]) + 4, right[top[i]] + left[top[i + 1]] + 3) for i in range(t - 1)]
    backs = [max(len(side[t + j]) + lead, len(names[bottom[j + 1]]) + 4, right[bottom[j + 1]] + left[bottom[j]] + 3)
             for j in range(len(bottom) - 1)]
    tl = max(LEAD + 2, left[top[0]] + 2)
    tr = max(LEAD + 2, len(names[top[-1]]) + 4, right[top[-1]] + 2, len(side[t - 1]) + lead)
    bl = max(LEAD + 2, left[bottom[-1]] + 2, len(side[-1]) + lead)
    br = max(LEAD + 2, len(names[bottom[0]]) + 4, right[bottom[0]] + 2)
    inner = max(tl + sum(gaps) + tr, bl + sum(backs) + br)
    x0 = 0
    x1 = x0 + inner
    xt = [x0 + tl + (inner - tl - sum(gaps) - tr) // 2 + sum(gaps[:i]) for i in range(t)]
    xb = [x1 - br - (inner - bl - sum(backs) - br) // 2 - sum(backs[:j]) for j in range(len(bottom))]
    y0 = max([h * SPUR for m, (_p, h) in hung.items() if root[m] in top], default=0)
    y1 = y0 + (5 + len(chords) if chords else 3)
    marks = {**{s: (x, y0) for s, x in zip(top, xt)}, **{s: (x, y1) for s, x in zip(bottom, xb)}}
    for m, (p, _hops) in hung.items():   # the nearest first: the stage it hangs from laid before it
        marks[m] = (marks[p][0] + 4 * kids[p].index(m), marks[p][1] + (-SPUR if root[m] in top else SPUR))
    named = {**dict.fromkeys(top, (2, 1)), **dict.fromkeys(bottom, (2, -1)), **dict.fromkeys(hung, (2, 0))}
    routes = [(top[i], top[i + 1], [(xt[i] + 1, y0), (xt[i + 1] - 1, y0)], True) for i in range(t - 1)]
    routes.append((top[-1], bottom[0], [(xt[-1] + 1, y0), (x1, y0), (x1, y1), (xb[0] + 1, y1)], True))
    routes += [(bottom[j], bottom[j + 1], [(xb[j] - 1, y1), (xb[j + 1] + 1, y1)], True) for j in range(len(bottom) - 1)]
    routes.append((bottom[-1], top[0], [(xb[-1] - 1, y1), (x0, y1), (x0, y0), (xt[0] - 1, y0)], True))
    slots = []
    for m, (p, _hops) in hung.items():
        (px, py), (mx, my), o = marks[p], marks[m], -1 if root[m] in top else 1
        routes.append((p, m, [(px, py + o), (px, py + 2 * o), (mx, py + 2 * o), (mx, my - o)], False))
        out, back = said(p, m)
        slots += [(mx, my, -2, -o, (p, m), 0, -1)] if out else []
        slots += [(px, py, 2, o, (p, m), 1, 1)] if back else []
    for k, (a, b) in enumerate(chords):
        (ax, ay), (bx, by), row = marks[a], marks[b], y0 + 3 + k
        routes.append((a, b, [(ax, ay + (1 if ay == y0 else -1)), (ax, row), (bx, row), (bx, by + (1 if by == y0 else -1))],
                       False))
    width = max([x1 + 1] + [x + 1 for x, _y in marks.values()]
                + [x + dx + len(said(*lane)[1]) for x, _y, dx, _dy, lane, d, _w in slots if d]
                + [marks[nd][0] + dx + len(names[nd]) for nd, (dx, _dy) in named.items()])
    return marks, named, routes, slots, (width, y1 + 1 + max([h * SPUR for m, (_p, h) in hung.items()
                                                                if root[m] in bottom], default=0))


def wire(nodes, lanes, asks, names, depth=None, pinned=(None, None)):
    """The wire laid in cells from (0, 0): (marks {node: its dot's cell}, routes [(u, v, corners, heavy)] - from
    the cell past u's name to the cell before v's dot, light -, (width, height)); `lanes` [(u, v)] each from its left
    node to its right one, the columns `depth` {node: its column} or by `asks` (from, to) (`depths`), each node's
    dot then its name - one of no name (a port) its dot alone, its lanes from it or into it -, `pinned` (top,
    bottom) at the top and the bottom of their columns; a lane past a column straight through it at a place of
    its own, a fan's lanes one track and its teeth."""
    depth = depth or depths(nodes, asks)
    columns = [[n for n in nodes if depth[n] == c] for c in range(max(depth.values(), default=-1) + 1)]
    col, chains = dict(depth), []
    for u, v in lanes:
        chain = [u]
        for c in range(col[u] + 1, col[v]):
            via = -10 - len(col)
            col[via] = c
            columns[c].append(via)
            chain.append(via)
        chains.append(chain + [v])
    hops = [(p, q) for chain in chains for p, q in zip(chain, chain[1:])]
    _ordered(columns, {n: [q for p, q in hops if p == n] + [p for p, q in hops if q == n] for c in columns for n in c})
    top, bottom = pinned
    for column in columns:
        column.sort(key=lambda n: (n != top) + (n == bottom))
    tall = max((len(c) for c in columns), default=0)
    y = {n: (tall - len(c)) // 2 + i for c in columns for i, n in enumerate(c)}
    y.update({n: row for n, row in ((top, 0), (bottom, tall - 1)) if n in y})
    outs, ins = Counter(p for p, _q in hops), Counter(q for _p, q in hops)
    key = {(p, q): (p, 0) if outs[p] >= ins[q] else (q, 1) for p, q in hops}
    tracks = [_tracked([h for h in hops if col[h[0]] == c], key, y) for c in range(len(columns))]
    xs, ends, x = [], [], 0
    for c, column in enumerate(columns):   # a column its dots, its names, a blank, its tracks two apart, a head
        xs.append(x)
        ends.append(x + 3 + max((len(names[n]) for n in column if n in names), default=0))
        x = ends[-1] + 2 * len(tracks[c]) + 2
    at = {(c, k): ends[c] + 1 + 2 * j for c, ts in enumerate(tracks) for j, k in enumerate(ts)}
    routes = []
    for chain in chains:
        u, v = names[chain[0]], names[chain[-1]]
        corners = [(xs[col[chain[0]]] + (3 + len(u) if u else 0), y[chain[0]])]
        for p, q in zip(chain, chain[1:]):
            corners += [(at[(col[p], key[(p, q)])], y[p]), (at[(col[p], key[(p, q)])], y[q])]
        routes.append((chain[0], chain[-1], corners + [(xs[col[chain[-1]]] - bool(v), y[chain[-1]])], False))
    marks = {n: (xs[col[n]], y[n]) for c in columns for n in c if n in names}
    return marks, routes, (ends[-1] - 1 if columns else 0, tall)


def walked(corners):
    """The cells a lane passes through `corners` in order: straight along a row or a column, along the row
    first and then the column where a corner shares neither with the one before."""
    out = [corners[0]]
    for c, r in corners[1:]:
        for tc, tr in ((c, out[-1][1]), (c, r)):
            pc, pr = out[-1]
            dc, dr = (tc > pc) - (tc < pc), (tr > pr) - (tr < pr)
            out += [(pc + dc * k, pr + dr * k) for k in range(1, max(abs(tc - pc), abs(tr - pr)) + 1)]
    return out


def toward(cell, at):
    """The way from `cell` to the cell `at`: along its longer distance."""
    dc, dr = at[0] - cell[0], at[1] - cell[1]
    return (RIGHT if dc > 0 else LEFT) if abs(dc) >= abs(dr) else (DOWN if dr > 0 else UP)


def laid(lanes, texts, cols, rows):
    """({(col, row): (char, ink)}, [cells] a lane, [cells] a lane's run): `lanes` [(from, to, corners, ink,
    head in, head out, heavy)] - each from the cell beside its node `from` through `corners` to the cell
    beside `to`, a head (its ink, or None) where a flow goes into `to` and one where a flow comes back into
    `from` - in lines, heads and the ways they reach their nodes, heavy where every lane through a cell is;
    then `texts` [(col, row, text, ink)] over them; all on a drawing `cols` by `rows` cells, the lanes' cells
    in order besides and each lane's longest run along a row of cells it alone passes straight through, no
    head nor text on one: where what it carries may be written."""
    per, heads, paths = {}, {}, []
    for start, end, corners, ink, into, back, heavy in lanes:
        cells, ways, tips = walked(corners), {}, FILLED if heavy else HEADS
        for cell, at in ((cells[0], start), (cells[-1], end)):
            ways[cell] = ways.get(cell, 0) | toward(cell, at)
        for (pc, pr), (qc, qr) in zip(cells, cells[1:]):
            go, come = (RIGHT, LEFT) if qc > pc else (LEFT, RIGHT) if qc < pc else (DOWN, UP) if qr > pr else (UP, DOWN)
            ways[(pc, pr)], ways[(qc, qr)] = ways.get((pc, pr), 0) | go, ways.get((qc, qr), 0) | come
        for cell, bits in ways.items():
            per.setdefault(cell, []).append((bits, ink, heavy))
        heads.update({cells[-1]: (tips[toward(cells[-1], end)], into)} if into is not None else {})
        heads.update({cells[0]: (tips[toward(cells[0], start)], back)} if back is not None else {})
        paths.append(cells)
    out = {}
    for cell, got in per.items():
        ways, glyphs = {bits for bits, _i, _h in got}, HEAVY if all(h for _b, _i, h in got) else GLYPHS
        if ways == {LEFT | RIGHT, UP | DOWN}:   # one lane across another, joining none of it: the upright on
            out[cell] = (glyphs[UP | DOWN], next(ink for bits, ink, _h in got if bits == UP | DOWN))
        else:   # every way out of the cell any lane takes
            mask = 0
            for bits in ways:
                mask |= bits
            out[cell] = (glyphs[mask], got[-1][1])
    out.update(heads)
    for col, row, text, ink in texts:
        out.update({(col + k, row): (ch, ink) for k, ch in enumerate(text)})
    runs = []
    for cells in paths:   # a run ends where another lane, a head, a text, a turn or the drawing's edge is
        best, run = [], []
        for cell in cells + [None]:
            if cell in per and len(per[cell]) == 1 and per[cell][0][0] == LEFT | RIGHT and out[cell][0] in '─━' \
                    and 0 <= cell[0] < cols and 0 <= cell[1] < rows:
                run.append(cell)
            else:
                best, run = max(best, run, key=len), []
        runs.append(best)
    return {cell: v for cell, v in out.items() if 0 <= cell[0] < cols and 0 <= cell[1] < rows}, paths, runs


def trail(path, forward, now, k):
    """([cells] the `k`th flow's packet lies on at `now`, its head first, TRAIL long; whether it is home, its
    head on the cell before its taker): run along `path` from its first cell to its last - back where not
    `forward` - every TRAVEL_S, the flows' packets PHASE apart."""
    way = path if forward else path[::-1]
    at = int((now / TRAVEL_S + k * PHASE) % 1.0 * len(way))
    return way[max(0, at - TRAIL + 1):at + 1][::-1], at == len(way) - 1


def dot(path, forward, now, k):
    """The cell of `path` the head of the `k`th flow's packet is on at `now` (`trail`)."""
    return trail(path, forward, now, k)[0][0]
