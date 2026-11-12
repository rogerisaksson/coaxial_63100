"""The atlas's mirror world: what flows between its files, drawn as a schematic of the data's way.

M turns the page to it: of the nodes shown, those a flow joins (tools/dev/atlas_flow.py, a flow's ends taken
to the nearest node shown) and nothing else, laid out by the flow in the terminal's own cells and drawn in
box-drawing lines (atlas/schematic.py), knife sharp. Her loop is a rectangle in heavy lines - of her pass
and what comes back through the world, the longest run back to where it began: its first half along the top
left to right, the rest back along the bottom, the last up the left side to the first; each stage its sign
on the line, its name inside, what passes written in the line on its way -, the rest of her pass hanging
straight off it, a flow across it inside it. The wire is columns left to right - the host's end, the
command handlers, the board's functions -, each node its sign and its name; its lanes run square along
tracks between the columns, a fan one track and its teeth, a lane past a column straight through it; where
nothing comes back her pass is columns too. A node's sign says what it is (SIGNS): her eyes, her senses,
her law, her drives, the world, the wire, the board. A lane carries both ways: a head where each of its
flows goes in. The camera faces it a world cell a terminal cell (`framed`), nearer as far as one likes, the
words beside what they name whatever the zoom; packets run every flow from its maker to its taker, the
taker's sign lit as one comes home (`drawn`); the node nearest the viewport (`middle`) is lit, named on top
and its flows listed in the panel at the right (`over`).
"""
import math

from terminal.views.atlas import leaves, panel, scene, schematic
from terminal.views.atlas.panel import fit

#: Each kind's ink (atlas_flow.KINDS: her pass, the wire, what comes back).
INKS = (scene.TITLE, scene.NOTE, scene.HIT)
#: The flow world in the world: how far under the trunk's foot its plane stands; a terminal cell's width
#: there, world units - a row two of them, braille's dots square, a cell two across and four down; her loop
#: and the wire apart, cells.
DEPTH, CELL, APART = 6.0, 0.1, 6
#: A node's sign by the first word of its path naming what it is, its own name first: her eyes, her senses,
#: her law, her drives, the world, the wire, the board, a process, what wears; a dot for the rest.
SIGNS = (('eye', '◉'), ('sight', '◉'), ('registry', '◉'), ('vision', '◉'), ('camera', '◉'),
         ('sens', '◎'), ('edge', '◎'), ('grip', '◎'), ('felt', '◎'), ('imu', '◎'), ('angle', '◎'), ('adc', '◎'),
         ('thermal', '◎'), ('law', '⎔'), ('modal', '⎔'), ('se2', '⎔'), ('latent', '⎔'), ('impedance', '⎔'),
         ('stack', '⎔'), ('wrench', '⎔'), ('chain', '⎔'), ('ctrl', '⎔'), ('motion', '⊛'), ('drive', '⊛'),
         ('motor', '⊛'), ('gate', '⊛'), ('pwm', '⊛'), ('power', '⊛'), ('world', '⊕'), ('twin', '⊕'),
         ('physics', '⊕'), ('wire', '⌁'), ('comm', '⌁'), ('link', '⌁'), ('uart', '⌁'), ('protocol', '⌁'),
         ('shtp', '⌁'), ('cmd', '⌁'), ('board', '▣'), ('process', '⟳'), ('worker', '⟳'), ('wear', '⧖'),
         ('watch', '⧖'), ('identify', '⧖'))


def _names(tree, nodes):
    """{node: its name}: its own, the folder it is in before it where two share one."""
    own = {n: scene.short(tree, n) for n in nodes}
    shared = {s for s in own.values() if list(own.values()).count(s) > 1}
    return {n: '/'.join(tree.id[n].rstrip('/').split('/')[-2:]) + s[len(s.rstrip('/')):] if s in shared else s
            for n, s in own.items()}


def _sign(tree, node):
    """`node`'s sign (SIGNS): of its path's parts, its own first, the first holding a word that names one."""
    for part in reversed(tree.id[node].lower().rstrip('/').split('/')):
        for word, sign in SIGNS:
            if word in part:
                return sign
    return '●'


def mirrored(placed, flow, room):
    """The flow world of the scene `placed` (show_atlas.placed) for a drawing `room` (cols, rows, the panel's
    width) - side by side or her loop over the wire, beside the panel where either fits (`beside` the room
    kept for it), else on the whole width -, the module's head: `marks` {node: its sign's cell}, `lanes` [(a,
    b, corners, ink, head in, head out, heavy)], `texts` [(x, y, across, down, text, ink, node)] - each by a
    cell and its offset in the terminal's cells, `node` the one it names -, in its cells; `dots` [(lane,
    forward)] a flow; its nodes' dots as `scene.picture` reads them (`flat`, none of its own names, all in
    focus), `anchors` the cells the camera places and `points` their middles in the world; `flows_out` and
    `flows_in` {node: flows} the chosen node's flows reach and come from; `rings` {node: its flows [(arrow,
    what passes, the name at the other end)]}; `loop` her loop's stages in order; `center` and `extent`."""
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
    loop = schematic.looped(stages, ahead | home, home)
    hung = schematic.hanging(loop, stages, ahead | home) if loop else {}
    ringed = set(loop) | set(hung)
    among = sorted({n for a, b, _k in passed for n in (a, b)} - ringed)
    names, index, n = _names(tree, ringed | set(among)), {s: k for k, s in enumerate(loop)}, len(loop)
    asks = [(a, b) for a, b, kind in passed if kind != back and a in among and b in among]
    depth, links, flows = schematic.depths(among, asks), {}, []
    for (a, b, kind), labels in sorted(passed.items()):   # each flow on the lane it runs, laid a to b
        text = ', '.join(sorted(set(labels)))
        if a in index and b in index and (index[b] - index[a]) % n == 1:
            key = (a, b)
        elif a in index and b in index and n > 2 and (index[a] - index[b]) % n == 1:
            key = (b, a)
        elif a in hung and hung[a][0] == b:
            key = (b, a)
        elif a in ringed and b in ringed:
            key = (b, a) if (b, a) in links else (a, b)
        else:
            key = (a, b) if depth.get(a, -1) < depth.get(b, -1) else (b, a)
        links.setdefault(key, ([], []))[key != (a, b)].append((kind, text))
        flows.append((a, b, kind, text, key))

    def said(a, b):
        return tuple(', '.join(('↺ ' if kind == back else '') + s for kind, s in way) for way in links.get((a, b), ((), ())))
    chords = [k for k in links if k[0] in index and k[1] in index and (index[k[1]] - index[k[0]]) % n != 1]
    rmarks, rnamed, rroutes, rtexts, (rw, rh) = schematic.ring(loop, hung, chords, said, names) if loop \
        else ({}, {}, [], [], (0, 0))
    wmarks, wroutes, (ww, wh) = schematic.wire(among, [k for k in links if k[0] in depth and k[1] in depth], asks, names) \
        if among else ({}, [], (0, 0))
    # Side by side or her loop over the wire, beside the panel where it fits, else on the whole width.
    cols, rows, beside = room
    gap = APART if rw and ww else 0
    shapes = {True: (rw + gap + ww, max(rh, wh)), False: (max(rw, ww), rh + 2 + wh)}
    side, beside = next(((s, p) for p in (beside, 0) for s in (True, False)
                         if shapes[s][0] + p <= cols and shapes[s][1] <= rows), (True, 0))
    width, height = shapes[side]
    ro, wo = ((0, (height - rh) // 2), (rw + gap, (height - wh) // 2)) if side \
        else (((width - rw) // 2, 0), ((width - ww) // 2, rh + 2))
    marks = {**{k: (x + ro[0], y + ro[1]) for k, (x, y) in rmarks.items()},
             **{k: (x + wo[0], y + wo[1]) for k, (x, y) in wmarks.items()}}
    routes = [(a, b, [(x + ro[0], y + ro[1]) for x, y in cs], heavy) for a, b, cs, heavy in rroutes] \
        + [(a, b, [(x + wo[0], y + wo[1]) for x, y in cs], heavy) for a, b, cs, heavy in wroutes]
    laid_ = {(a, b) for a, b, _cs, _h in routes}   # any lane left: from beside its left sign to beside its right one
    routes += [(a, b, [(marks[a][0] + 1, marks[a][1]), (marks[b][0] - 1, marks[b][1])], False) for a, b in links
               if (a, b) not in laid_]

    def inked(way):
        kinds = [kind for kind, _s in way]
        return (INKS[back] if back in kinds else INKS[kinds[0]]) if kinds else None
    lanes = [(a, b, cs, inked(links[(a, b)][0]) or inked(links[(a, b)][1]), inked(links[(a, b)][0]),
              inked(links[(a, b)][1]), heavy) for a, b, cs, heavy in routes]
    which = {(a, b): k for k, (a, b, *_rest) in enumerate(lanes)}
    rings, out, into = {nd: [] for nd in marks}, {}, {}
    for a, b, kind, text, _key in flows:
        arrows = ('↺', '↺') if kind == back else ('→', '←')
        rings[a].append((arrows[0], text, names[b]))
        rings[b].append((arrows[1], text, names[a]))
        if b in lit and a not in lit:
            into[a] = into.get(a, 0) + 1
        elif a in lit and b not in lit:
            out[b] = out.get(b, 0) + 1
    nodes = sorted(marks, key=lambda nd: (nd != focus, nd not in lit, nd))
    inks = {nd: scene.OPEN if nd == focus else (scene.OUT if out.get(nd, 0) >= into.get(nd, 0) else scene.IN)
            if nd in out or nd in into else scene.NODE if nd in lit else scene.EDGE for nd in nodes}
    texts = [(x + ro[0], y + ro[1], dx, dy, s, ink, None) for x, y, dx, dy, s, ink in rtexts]
    texts += [(*marks[nd], 0, 0, _sign(tree, nd), inks[nd], nd) for nd in nodes]
    texts += [(*marks[nd], *rnamed.get(nd, (2, 0)), names[nd], scene.TOLD if nd == focus else scene.NAME, nd) for nd in nodes]

    def world(cell):
        return ((cell[0] + 0.5 - width / 2) * CELL, -DEPTH - (2 * cell[1] + 1 - height) * CELL, 0.0)
    flat = {'nodes': np.array(nodes, np.intp), 'pos': np.array([world(marks[nd]) for nd in nodes], float).reshape(-1, 3),
            'disc': np.zeros(len(nodes)), 'ink': np.array([inks[nd] for nd in nodes], np.uint8), 'lit': lit,
            'from': np.zeros((0, 3)), 'to': np.zeros((0, 3)), 'segink': np.zeros(0, np.uint8), 'thick': np.zeros(0, bool)}
    anchors = sorted({c for lane in lanes for c in lane[2]} | set(marks.values()) | {(x, y) for x, y, *_t in texts})
    return {**placed, 'at': {nd: world(c) for nd, c in marks.items()}, 'out': {}, 'into': {}, 'flows_out': out,
            'flows_in': into, 'rings': rings, 'faint': [], 'flat': flat, 'loop': loop, 'marks': marks, 'lanes': lanes,
            'texts': texts, 'dots': [(which[key], key == (a, b)) for a, b, _k, _s, key in flows], 'anchors': anchors,
            'points': np.array([world(c) for c in anchors], float).reshape(-1, 3), 'groups': ((), ()),
            'field': math.inf, 'center': (0.0, -DEPTH, 0.0), 'extent': (width, height), 'beside': beside, 'kept': {}}


def framed(mirror, cols, rows):
    """(target, distance) facing the flow world on a drawing `cols` by `rows` cells, a world cell a terminal
    cell (CELL): its middle in the middle of the room left of the panel at the right where room was kept for
    it (`mirrored`'s `beside`) - one wider than the drawing from its left edge, where her loop begins and
    closes -, every world cell's middle in the middle of a terminal cell, whatever the sizes; the zoom's
    farthest, its words the size they are."""
    (x, y, z), (width, height), panel_ = mirror['center'], mirror['extent'], mirror['beside']
    across = (cols - width) / 2 - 1 if width > cols else (panel_ + (cols - width - panel_) % 2) / 2
    return (x + across * CELL, y + (rows - height) % 2 * CELL, z), scene.view(scene.camera(), cols, rows)['focal'] * CELL / 2


def laid(mirror, sight, cols, rows):
    """(cells {(col, row): (char, ink)}, [cells] a lane in order, {node: (its sign's cell, its name's cells)}):
    the schematic where `sight` (scene.view) puts its cells on a drawing `cols` by `rows` (schematic.laid) -
    a lane's ends and every word where they stand from what they name in the terminal's cells, only the cells
    between them spread by the zoom; laid anew only where a cell of it moves: the camera's ease creeps on."""
    from coaxial.model.blocks import numpy as np
    rel = mirror['points'] - np.array(sight['eye'])
    across, up, ahead = (rel @ np.array(sight[axis]) for axis in ('r', 'u', 'f'))
    ahead = np.maximum(ahead, scene.NEAR)
    col = np.floor((sight['w'] / 2 + across * sight['focal'] / ahead) / 2).astype(np.intp)
    row = np.floor((sight['h'] / 2 - up * sight['focal'] / ahead) / 4).astype(np.intp)
    key = (col.tobytes(), row.tobytes(), cols, rows)
    if key not in mirror['kept']:
        marks, at = mirror['marks'], dict(zip(mirror['anchors'], zip(col.tolist(), row.tolist())))

        def beside(node, cell):   # a lane's end where it stands from its node's sign
            (sx, sy), (mx, my) = at[marks[node]], marks[node]
            return (sx + cell[0] - mx, sy + cell[1] - my)
        lanes = [(at[marks[a]], at[marks[b]], [beside(a, cs[0])] + [at[c] for c in cs[1:-1]] + [beside(b, cs[-1])],
                  ink, into, out, heavy) for a, b, cs, ink, into, out, heavy in mirror['lanes']]
        texts = [(at[(x, y)][0] + dx, at[(x, y)][1] + dy, text, ink, node) for x, y, dx, dy, text, ink, node in mirror['texts']]
        cells, paths = schematic.laid(lanes, [t[:4] for t in texts], cols, rows)
        spots = {}
        for x, y, text, _ink, node in texts:
            if node is not None:
                sign, name = spots.get(node, (None, []))
                spots[node] = ((x, y), name) if sign is None else (sign, [(x + k, y) for k in range(len(text))])
        mirror['kept'] = {key: (cells, paths, spots)}
    return mirror['kept'][key]


def dots(mirror, sight, cols, rows, now):
    """The cell the head of each flow's packet is on at `now` (schematic.dot), on its lane as `laid`."""
    paths = laid(mirror, sight, cols, rows)[1]
    return [schematic.dot(paths[k], forward, now, i) for i, (k, forward) in enumerate(mirror['dots'])]


def middle(mirror, fell, sight, cols):
    """The node drawn nearest the middle of the room the schematic keeps (`room`) - its title's, its panel's
    and the one lit in it (`over`, show_atlas)."""
    return panel.middle(fell, room(mirror, sight, cols), mirror['rings'])


def drawn(mirror, sight, cols, rows, now, mid=None):
    """{(col, row): (char, ink)}: the schematic as `laid`, each flow's packet at `now` bright (schematic.trail),
    a taker's sign lit as its packet comes home; the node `mid`'s sign in the chosen node's ink, its name bright."""
    cells, paths, spots = laid(mirror, sight, cols, rows)
    cells, lanes = dict(cells), mirror['lanes']
    for i, (k, forward) in enumerate(mirror['dots']):
        packet, home = schematic.trail(paths[k], forward, now, i)
        lit = packet + ([spots[lanes[k][1] if forward else lanes[k][0]][0]] if home else [])
        cells.update({cell: (cells[cell][0], scene.TOLD) for cell in lit if cell in cells})
    sign, name = spots.get(mid, (None, []))
    cells.update({cell: (cells[cell][0], scene.TOLD) for cell in name if cell in cells})
    cells.update({sign: (cells[sign][0], scene.OPEN)} if sign in cells else {})
    return cells


def over(mirror, sight, keyed, fell, cols, rows, taken=frozenset(), now=0.0):
    """{(col, row): (char, ink)}: the schematic at `now`, the node nearest the viewport lit (`drawn`,
    `middle`), then at the right a panel listing that node's flows (leaves.panels, `keyed` {node: its key}:
    `lines`, `noted`), over none of it."""
    mid = middle(mirror, fell, sight, cols)
    cells = drawn(mirror, sight, cols, rows, now, mid)
    rings = {mid: mirror['rings'][mid]} if mid in mirror['rings'] else {}
    return {**cells, **leaves.panels(mirror['tree'], rings, fell, {n: keyed.get(n, '') for n in rings}, cols, rows,
                                     taken | set(cells), lines, noted, most=1, docked=True)}


def room(mirror, sight, cols):
    """`sight` (scene.view) on a drawing `cols` cells across narrowed to the room left of the panel where room
    was kept for it (`framed`), the viewport's middle that room's."""
    return {**sight, 'w': 2 * (cols - mirror['beside'])}


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
