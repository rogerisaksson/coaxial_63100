"""The SYSTEM ATLAS page's scene: the system as a tree in 3D, seen by an orbiting camera in braille.

The trunk stands up the y axis, the terminal at its foot and its sensing at its crown, a layer a
STEP apart; a layer beside it stands SIDE out, at the height of the trunk's layers it ties to most. The
node chosen and those above it are open, and so are the nodes its keys offer - its children, at the trunk
the layers -, their own children laid out ahead: a layer's folders ring it level, a folder's files and a
file's symbols hang under it as a cap, the heaviest at its foot, the rest spiralling out by the golden
angle. The chosen node, those above it and all laid out below it are lit - before any is chosen, the
trunk -, the rest grey; the chosen node's ties are bowed out from the trunk, not lost along it. Named in the tree (by a scene's own `names` where it has them): the chosen node, its keys,
those above it, the layers, its ties' ends and a word's hits; its leaves in panels (`over`, atlas/leaves.py).
Out of focus the farther from the point the view turns about: its dots fewer, scattered and darker.

Drawn as data: a focus's nodes and segments are laid once as flat arrays (`flat`); a frame projects them
all at once, cuts them at the near plane and the drawing's edge, rasters every segment's and disc's dots
in one pass, thins, scatters those out of focus, keeps each dot's highest-ranked ink and shade and
encodes the cells as braille, each cell's ink darkened by its sharpest dot's shade (`picture`).
"""
import functools
import math

from coaxial.draw import braille
from coaxial.graphics.callouts import FRAME_INK
from tools.dev.atlas_tree import FILE, FOLDER, LAYER, LAYERS, SYMBOL

#: A layer's step up the trunk, how far out a layer beside it stands, the near plane, world units; a key's zoom.
STEP, SIDE, NEAR, ZOOM = 1.0, 2.6, 0.05, 1.25
#: A layer's ring of folders: its radius's base and its growth a folder. A cap's radius - its base and
#: its growth a square root of its children - by the kind it hangs from, and its half-angle.
RING, CAP, CAP_DEG = (0.6, 0.04), {FOLDER: (0.25, 0.07), FILE: (0.12, 0.025)}, 75.0
GOLDEN = math.pi * (3.0 - math.sqrt(5.0))
#: A node's disc by kind, world units, and its least and most dots; the most names drawn; the faint
#: ties between layers shown at the trunk, and the chosen node's ties shown each way.
DISC, DOTS, NAMES, FAINT_TIES, TIES_SHOWN = (0.11, 0.05, 0.022, 0.011), (0.8, 4.0), 48, 16, 12
#: A layer's ring round its node - the turn and tip of the view seen in it -, world units, its sides and
#: their ends' turns.
HALO, SIDES = 0.24, 24
TURNS = tuple((math.cos(2 * math.pi * k / SIDES), math.sin(2 * math.pi * k / SIDES)) for k in range(SIDES + 1))
#: Out of focus (`blur`) a thing's distance from the point the view turns about, a share of the camera's to
#: it, makes half out at FOCUS - a framed node's reach (show_atlas.frame stands 2.8 off). Out of
#: focus a dot keeps of its share down to SPARSE, darkens to DARK of its ink in SHADES steps and strays up to
#: STRAY dots; a screen door picks which keep - a dot's threshold of 16 along a line from its start (DOOR,
#: the bit-reversed count) or round a node's middle (BAYER, the ordered dither) - and STRAYS its way (/3).
FOCUS, SPARSE, DARK, SHADES, STRAY = 0.36, 0.2, 0.2, 6, 2.0
DOOR = (0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15)
BAYER = ((0, 8, 2, 10), (12, 4, 14, 6), (3, 11, 1, 9), (15, 7, 13, 5))
STRAYS = (0, 3, -2, 1, -3, 2, -1, 0, 2, -3, 1, -2, 3, -1, 0, 1)
#: The inks by rank - a dot keeps its highest-ranked: the faint ties between layers, dim edges and nodes -
#: all but the chosen node's, those above it and those laid out below it -, lit edges, the chosen node's
#: ties out and in, lit nodes, then the legend's (the dark terminal's, validated all-pairs): reaches,
#: reached by, found, chosen. Then text's: a dim name, a lit one, the chosen node's, a key that chooses; a
#: panel's frame - her eyes' (coaxial.graphics.inset: the instruments' frame, the theme's name and value) -,
#: an idle panel's frame, a brief.
FAINT, DIM_EDGE, DIM_NODE, EDGE, TIE_OUT, TIE_IN, NODE, OUT, IN, HIT, OPEN, NAME_DIM, NAME, TOLD, KEY, \
    FRAME, TITLE, NOTE, FRAME_DIM, BRIEF = range(1, 21)
INK = {FAINT: (23, 26, 31), DIM_EDGE: (28, 32, 38), DIM_NODE: (42, 48, 56), EDGE: (178, 190, 204),
       TIE_OUT: (25, 158, 112), TIE_IN: (217, 89, 38), NODE: (247, 249, 252), OUT: (25, 158, 112),
       IN: (217, 89, 38), HIT: (213, 81, 129), OPEN: (57, 135, 229), NAME_DIM: (62, 69, 79),
       NAME: (198, 206, 216), TOLD: (250, 252, 255), KEY: (57, 135, 229), FRAME: FRAME_INK,
       TITLE: (0, 215, 215), NOTE: (255, 175, 0), FRAME_DIM: (44, 62, 62), BRIEF: (112, 122, 134)}
SGR = {0: '\x1b[0m', **{ink: '\x1b[38;2;%d;%d;%dm' % rgb for ink, rgb in INK.items()}}


@functools.lru_cache(maxsize=None)
def shaded(key):
    """The SGR of `key` - an ink times SHADES and its shade -: the ink darkened shade / (SHADES - 1) of the way
    down to DARK of itself; the reset for none."""
    ink, shade = divmod(key, SHADES)
    k = 1.0 - (1.0 - DARK) * shade / (SHADES - 1)
    return '\x1b[38;2;%d;%d;%dm' % tuple(round(c * k) for c in INK[ink]) if ink else SGR[0]
CUT = chr(0x2026)


def weight(tree, below, node):
    """How much a node holds, the order its children take: files under a layer or a folder, symbols in a
    file."""
    return below[node][FILE] if tree.kind[node] <= FOLDER else below[node][SYMBOL]


def children(tree, kids, below, node):
    """A node's children as its ring or cap lays them: the heaviest first, by name between equals."""
    return sorted(kids[node], key=lambda c: (-weight(tree, below, c), tree.id[c]))


def short(tree, node):
    """A node's name as the scene and the panel give it: a folder its last part, a file its name, a
    symbol its signature."""
    label = tree.label[node]
    return label.rstrip('/').rpartition('/')[2] + '/' if tree.kind[node] == FOLDER else label


def chain(tree, node):
    """The node and those above it - chosen while it is -, the layer first; none at the trunk."""
    out = []
    while node >= 0:
        out.append(node)
        node = tree.up[node]
    return out[::-1]


def layout(tree, kids, below, between, focus):
    """{node: (x, y, z)} of every node shown - the layers always, the children of the node chosen and of
    those above it, and laid out ahead the children of the nodes its keys offer (its own, at the trunk the
    layers) -, the stubs [(trunk point, beside layer)] that tie a layer beside the trunk to it, each open
    node's {node: reach}: its children's ring or cap radius, and {node: its children as they are laid}."""
    trunk = [i for i, row in enumerate(LAYERS) if row[1] == 'trunk']
    beside = [i for i, row in enumerate(LAYERS) if row[1] != 'trunk']
    at = {layer: (0.0, k * STEP, 0.0) for k, layer in enumerate(trunk)}
    stubs, reach, rings = [], {}, {}
    for k, layer in enumerate(beside):
        pull = {t: between[layer][t] + between[t][layer] for t in trunk}
        y = sum(at[t][1] * w for t, w in pull.items()) / (sum(pull.values()) or 1)
        a = math.pi / 2 + 2 * math.pi * k / len(beside)
        at[layer] = (SIDE * math.cos(a), y, SIDE * math.sin(a))
        stubs.append(((0.0, y, 0.0), layer))
    for node in chain(tree, focus) + list(range(len(LAYERS)) if focus < 0 else kids[focus]):
        ring = rings[node] = children(tree, kids, below, node)
        if not ring:
            continue
        (x, y, z), turn, n = at[node], (node * GOLDEN) % (2 * math.pi), len(ring)
        if tree.kind[node] == LAYER:
            reach[node] = r = RING[0] + RING[1] * n
            for k, child in enumerate(ring):
                a = turn + 2 * math.pi * k / n
                at[child] = (x + r * math.cos(a), y, z + r * math.sin(a))
            continue
        base, per = CAP[tree.kind[node]]
        reach[node] = r = base + per * math.sqrt(n)
        for k, child in enumerate(ring):
            polar, a = math.radians(CAP_DEG) * math.sqrt((k + 0.5) / n), turn + k * GOLDEN
            at[child] = (x + r * math.sin(polar) * math.cos(a), y - r * math.cos(polar),
                         z + r * math.sin(polar) * math.sin(a))
    return at, stubs, reach, rings


def shown_of(tree, at, node):
    """The node itself where it is shown, else its nearest ancestor that is: where its ties land."""
    while node >= 0 and node not in at:
        node = tree.up[node]
    return node


def tied(tree, own, at, focus):
    """({shown node: ties} the focus reaches, the same reaching it): each tie with one end within the
    focus and the other without, that end taken to the nearest node shown; a symbol's are its file's."""
    focus = tree.up[focus] if tree.kind[focus] == SYMBOL else focus
    level, out, into = tree.kind[focus], {}, {}
    for s, d in zip(tree.src, tree.dst):
        inside, reached = own[s][level] == focus, own[d][level] == focus
        if inside != reached:
            side, other = (out, d) if inside else (into, s)
            other = shown_of(tree, at, other)
            side[other] = side.get(other, 0) + 1
    return out, into


def camera(yaw=35.0, pitch=18.0, distance=9.5, target=(0.0, 4.0, 0.0)):
    """A camera: the point it orbits (target), from where round it (yaw, pitch, degrees), how far."""
    return {'yaw': yaw, 'pitch': pitch, 'distance': distance, 'target': target}


def view(cam, width, height):
    """`cam` on a drawing `width` x `height` cells: its eye, its frame (right, up, ahead), its focal
    length and size in dots - braille's dots are square, two a cell across and four down."""
    yaw, pitch = math.radians(cam['yaw']), math.radians(cam['pitch'])
    tx, ty, tz = cam['target']
    d = cam['distance']
    eye = (tx + d * math.cos(pitch) * math.sin(yaw), ty + d * math.sin(pitch), tz + d * math.cos(pitch) * math.cos(yaw))
    f = _unit((tx - eye[0], ty - eye[1], tz - eye[2]))
    r = _unit((-f[2], 0.0, f[0]))
    u = (r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0])
    w, h = 2 * width, 4 * height
    return {'eye': eye, 'r': r, 'u': u, 'f': f, 'focal': 0.92 * min(w, h), 'w': w, 'h': h}


def _unit(v):
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) or 1.0
    return (v[0] / n, v[1] / n, v[2] / n)


def bow(a, b, lift=1.0):
    """A tie from `a` to `b` as a curve's points, bowed away from the trunk (along x where its middle is
    on it) by `lift` times a quarter of its length and a little more."""
    mid = tuple((a[k] + b[k]) / 2 for k in range(3))
    away = _unit((mid[0], 0.0, mid[2])) if abs(mid[0]) + abs(mid[2]) > 1e-6 else (1.0, 0.0, 0.0)
    c = tuple(mid[k] + away[k] * lift * (0.25 * math.dist(a, b) + 0.2) for k in range(3))
    return [tuple((1 - t) ** 2 * a[k] + 2 * (1 - t) * t * c[k] + t * t * b[k] for k in range(3))
            for t in (i / 12 for i in range(13))]


def flat(scene):
    """The scene at its focus as flat arrays, laid once and kept in it: its nodes - indices, positions,
    discs' radii (world), inks before a found word - and its segments, both ends, an ink and whether two
    dots wide (the trunk) each."""
    if 'flat' not in scene:
        scene['flat'] = _flat(scene)
    return scene['flat']


def _flat(scene):
    from coaxial.model.blocks import numpy as np
    tree, at, focus, out, into = scene['tree'], scene['at'], scene['focus'], scene['out'], scene['into']
    trunk = [i for i, row in enumerate(LAYERS) if row[1] == 'trunk']
    nodes, parts = list(at), []
    # The chosen node, those above it and all laid out below it; before any is chosen, the trunk.
    lit = set(chain(tree, focus) + [n for n in nodes if focus in chain(tree, n)]) if focus >= 0 else set(trunk)

    def curve(points, ink):
        parts.extend((p, q, ink, False) for p, q in zip(points, points[1:]))

    parts += [(at[a], at[b], EDGE if a in lit and b in lit else DIM_EDGE, True) for a, b in zip(trunk, trunk[1:])]
    parts += [(point, at[layer], EDGE if layer in lit else DIM_EDGE, False) for point, layer in scene['stubs']]
    parts += [(at[tree.up[n]], at[n], EDGE if n in lit else DIM_EDGE, False) for n in nodes if tree.kind[n] != LAYER]
    for layer in range(len(LAYERS)):
        x, y, z = at[layer]
        curve([(x + HALO * c, y, z + HALO * s) for c, s in TURNS], EDGE if layer in lit else DIM_EDGE)
    for a, b in scene['faint'] if focus < 0 else ():
        curve(bow(at[a], at[b], 0.6), FAINT)
    for ties, ink, lift in ((into, TIE_IN, 0.6), (out, TIE_OUT, 1.0)):
        for other, _n in sorted(ties.items(), key=lambda kv: -kv[1])[:TIES_SHOWN]:
            curve(bow(at[focus], at[other], lift), ink)
    ends = np.array([s[0] for s in parts] + [s[1] for s in parts], float).reshape(2, -1, 3)
    ink = [OPEN if n == focus else (OUT if out.get(n, 0) >= into.get(n, 0) else IN) if n in out or n in into
           else NODE if n in lit else DIM_NODE for n in nodes]
    return {'nodes': np.array(nodes, np.intp), 'pos': np.array([at[n] for n in nodes], float).reshape(-1, 3),
            'disc': np.array([DISC[tree.kind[n]] for n in nodes], float), 'ink': np.array(ink, np.uint8),
            'lit': lit, 'from': ends[0], 'to': ends[1], 'segink': np.array([s[2] for s in parts], np.uint8),
            'thick': np.array([s[3] for s in parts], bool)}


def blur(away):
    """How far out of focus, 0 to 1, things `away` from the point the view turns about are - that distance a
    share of the camera's to the point -: 1 - e^-(away / FOCUS)^2, sharp at it, half out at FOCUS."""
    from coaxial.model.blocks import numpy as np
    return 1.0 - np.exp(-(np.asarray(away, float) / FOCUS) ** 2)


def _local(v, p):
    """World points (n, 3) in the camera's frame: [right, up, ahead], each (n,)."""
    d0, d1, d2 = p[:, 0] - v['eye'][0], p[:, 1] - v['eye'][1], p[:, 2] - v['eye'][2]
    return [d0 * v[axis][0] + d1 * v[axis][1] + d2 * v[axis][2] for axis in ('r', 'u', 'f')]


def segments(x0, y0, x1, y1, w, h):
    """Every dot of every segment (x0, y0)-(x1, y1) - dots, arrays - within a `w` x `h` drawing: (flat index,
    segment, its count along the segment) a dot. A segment is cut to the drawing (Liang-Barsky) and drawn
    as its longer extent's dots and two more, evenly spaced from its start to its end (numpy.linspace's),
    each where it falls."""
    from coaxial.model.blocks import numpy as np
    dx, dy = x1 - x0, y1 - y0
    t0, t1, out = np.zeros_like(dx), np.ones_like(dx), np.zeros(dx.shape, bool)
    with np.errstate(divide='ignore', invalid='ignore'):
        for p, q in ((-dx, x0), (dx, w - 1e-3 - x0), (-dy, y0), (dy, h - 1e-3 - y0)):
            r = q / p
            out |= (p == 0.0) & (q < 0.0)
            t0, t1 = np.where(p < 0.0, np.maximum(t0, r), t0), np.where(p > 0.0, np.minimum(t1, r), t1)
    keep = np.flatnonzero(~out & (t0 <= t1))
    x0, y0, dx, dy, t0, t1 = x0[keep], y0[keep], dx[keep], dy[keep], t0[keep], t1[keep]
    ax, ay, bx, by = x0 + dx * t0, y0 + dy * t0, x0 + dx * t1, y0 + dy * t1
    n = np.maximum(np.abs(bx - ax), np.abs(by - ay)).astype(np.intp) + 2
    seg = np.repeat(np.arange(len(n)), n)
    k = np.arange(len(seg)) - np.repeat(np.cumsum(n) - n, n)
    last = k == (n - 1)[seg]
    xs = np.where(last, bx[seg], k * ((bx - ax) / (n - 1))[seg] + ax[seg]).astype(np.intp)
    ys = np.where(last, by[seg], k * ((by - ay) / (n - 1))[seg] + ay[seg]).astype(np.intp)
    return np.clip(ys, 0, h - 1) * w + np.clip(xs, 0, w - 1), keep[seg], k


def discs(x, y, r, w, h):
    """Every dot of every disc of radius `r` round (x, y) - dots, arrays - within the drawing: (flat index,
    disc) a dot whose middle lies within its disc's radius of its middle."""
    from coaxial.model.blocks import numpy as np
    flat_at, of = [np.zeros(0, np.intp)], [np.zeros(0, np.intp)]
    reach = np.ceil(r).astype(np.intp) + 1
    for k in np.unique(reach).tolist():
        sel = np.flatnonzero(reach == k)
        off = np.arange(-k, k + 1)
        fx, fy, cx, cy, rr = np.floor(x[sel]), np.floor(y[sel]), x[sel], y[sel], r[sel] * r[sel]
        px = fx[:, None, None] + off[None, None, :]
        py = fy[:, None, None] + off[None, :, None]
        inside = ((px + 0.5 - cx[:, None, None]) ** 2 + (py + 0.5 - cy[:, None, None]) ** 2 <= rr[:, None, None]) \
            & (px >= 0) & (px < w) & (py >= 0) & (py < h)
        disc, j, i = np.nonzero(inside)
        flat_at.append(((fy[disc] + off[j]) * w + fx[disc] + off[i]).astype(np.intp))
        of.append(sel[disc])
    return np.concatenate(flat_at), np.concatenate(of)


@functools.lru_cache(maxsize=1)
def _glyphs():
    from coaxial.model.blocks import numpy as np
    return np.array([' '] + [braille.ALL[bits] for bits in range(1, 256)], dtype='<U1')


def cells(dots, text, w, h, shade=None):
    """The drawing as rows (its line, its runs' starts, their keys - `shaded`'s): a cell its text where it
    has any (`text` {(col, row): (char, ink)}), else its dots (`dots`, w * h ranks) in braille in the ink
    of the highest rank among them, darkened by the least of their `shade`s (w * h steps)."""
    from coaxial.model.blocks import numpy as np
    rows, cols = h // 4, w // 2
    grid = dots.reshape(h, w)[:rows * 4, :cols * 2].reshape(rows, 4, cols, 2)
    bits = np.zeros((rows, cols), np.intp)
    for dy in range(4):
        for lane in range(2):
            bits |= np.where(grid[:, dy, :, lane] > 0, braille.BIT[lane][dy], 0)
    inks, chars = grid.max(axis=(1, 3)).astype(np.intp), _glyphs()[bits]
    dark = np.zeros_like(inks) if shade is None else np.where(inks > 0, np.where(
        grid > 0, shade.reshape(h, w)[:rows * 4, :cols * 2].reshape(rows, 4, cols, 2), SHADES).min(axis=(1, 3)), 0)
    for (col, row), (ch, ink) in text.items():
        chars[row, col], inks[row, col], dark[row, col] = ch, ink, 0
    keys = inks * SHADES + dark
    go = np.flatnonzero(np.diff(keys, axis=1, prepend=-1))   # runs' starts
    at, line = np.searchsorted(go, np.arange(rows + 1) * cols).tolist(), chars.tobytes().decode('utf-32-le')
    key, go = keys.ravel()[go].tolist(), (go % cols).tolist()
    return [(line[r * cols:(r + 1) * cols], go[at[r]:at[r + 1]], key[at[r]:at[r + 1]]) for r in range(rows)]


def _label(text, col, row, runs, cols, rows):
    """Text from cell (col, row): [(text, ink)] runs; refused where a cell it needs or one beside it is
    taken, or off the drawing, so no name covers another."""
    width = sum(len(t) for t, _ in runs)
    if row < 0 or row >= rows or col < 0 or col + width > cols \
            or any((c, row) in text for c in range(col - 1, col + width + 1)):
        return False
    for t, ink in runs:
        for ch in t:
            text[(col, row)] = (ch, ink)
            col += 1
    return True


def picture(scene, cam, width, height, keyed, found, over=None):
    """The scene from `cam` on `width` x `height` cells, as rows (`cells`), where each node fell ({node:
    (x, y, depth)}, dots) and the view: `keyed` {node: its key}, `found` the nodes a word's hits lie under;
    `over` (fell, cols, rows, the cells the keys' names took) -> {(col, row): (char, ink)} laid after the
    chosen node's, its keys' and its chain's names, before the rest's - a scene's `groups` where it has
    them, its `lefts` named on their left."""
    from coaxial.model.blocks import numpy as np
    tree, f, focus = scene['tree'], flat(scene), scene['focus']
    v = view(cam, width, height)
    w, h, focal = v['w'], v['h'], v['focal']
    nodes, ink = f['nodes'], f['ink'].copy()
    if found:
        ink[np.isin(nodes, list(found)) & (nodes != focus)] = HIT
    with np.errstate(divide='ignore', invalid='ignore'):
        cx, cy, cz = _local(v, f['pos'])
        x, y = w / 2 + cx * focal / cz, h / 2 - cy * focal / cz
        size = np.minimum(DOTS[1], np.maximum(DOTS[0], f['disc'] * focal / cz))
        a, b = _local(v, f['from']), _local(v, f['to'])
        behind_a, behind_b = a[2] < NEAR, b[2] < NEAR
        t = (NEAR - a[2]) / (b[2] - a[2])
        cut = [a[k] + (b[k] - a[k]) * t for k in range(3)]
        a = [np.where(behind_a, cut[k], a[k]) for k in range(3)]
        b = [np.where(behind_b, cut[k], b[k]) for k in range(3)]
        xa, ya, xb, yb = w / 2 + a[0] * focal / a[2], h / 2 - a[1] * focal / a[2], \
            w / 2 + b[0] * focal / b[2], h / 2 - b[1] * focal / b[2]
    seen = np.flatnonzero(cz >= NEAR)
    drawn = np.flatnonzero(~(behind_a & behind_b))
    twice = drawn[f['thick'][drawn]]
    on, by, along = segments(np.concatenate((xa[drawn], xa[twice] + 1.0)), np.concatenate((ya[drawn], ya[twice])),
                             np.concatenate((xb[drawn], xb[twice] + 1.0)), np.concatenate((yb[drawn], yb[twice])),
                             w, h)
    under, of = discs(x[seen], y[seen], size[seen] + (nodes[seen] == focus), w, h)
    # Out of focus (`blur`) a thing keeps a share of its dots - a screen door's: a line's from its start, a
    # disc's round its node's middle -, they stray and darken; a scene's `field` in focus.
    line_of, disc_of, target = np.concatenate((drawn, twice))[by], seen[of], cam['target']
    near = cam['distance'] * scene.get('field', 1.0)
    across = under % w - np.floor(x[disc_of]).astype(np.intp)
    down = under // w - np.floor(y[disc_of]).astype(np.intp)
    count = np.concatenate((along, across * 4 + down))
    door = np.concatenate((np.array(DOOR)[along & 15], np.array(BAYER)[down & 3, across & 3]))
    away = np.linalg.norm((f['from'] + f['to']) / 2 - target, axis=1), np.linalg.norm(f['pos'] - target, axis=1)
    out = np.concatenate((blur(away[0] / near)[line_of], blur(away[1] / near)[disc_of]))
    stay = (door + 0.5) / 16 < 1.0 - out * (1.0 - SPARSE)
    spot, strays = np.concatenate((on, under)), np.array(STRAYS) * STRAY / 3.0
    px = np.clip(spot % w + np.rint(out * strays[count & 15]).astype(np.intp), 0, w - 1)
    py = np.clip(spot // w + np.rint(out * strays[(count + 5) & 15]).astype(np.intp), 0, h - 1)
    flat_at, out = (py * w + px)[stay], out[stay]
    ranks = np.concatenate((f['segink'][line_of], ink[disc_of]))[stay]
    dots, shade = np.zeros(w * h, np.uint8), np.zeros(w * h, np.uint8)
    for rank in np.unique(ranks).tolist():
        sel = ranks == rank
        dots[flat_at[sel]], shade[flat_at[sel]] = rank, np.rint(out[sel] * (SHADES - 1))
    fell = dict(zip(nodes[seen].tolist(), zip(x[seen].tolist(), y[seen].tolist(), cz[seen].tolist())))
    radius = dict(zip(nodes[seen].tolist(), size[seen].tolist()))
    cols, rows, lit = w // 2, h // 4, f['lit']
    tied_most = sorted(list(scene['out'].items()) + list(scene['into'].items()), key=lambda kv: -kv[1])
    groups = scene.get('groups') or ([focus] + list(keyed) + chain(tree, focus),
                                     list(range(len(LAYERS))) + [n for n, _ in tied_most] + sorted(found))
    text, named, done = {}, 0, set()
    for k, group in enumerate(groups):
        if k and over:   # `over` after the keys' names, over none of them, before the rest's
            text.update(over(fell, cols, rows, frozenset(text)))
        for node in group:
            if node in done or node not in fell or named >= NAMES:
                continue
            done.add(node)
            (nx, ny, _), r, key = fell[node], radius[node], keyed.get(node)
            name = scene.get('names', {}).get(node) or short(tree, node)
            name = name if len(name) <= 28 else name[:27] + CUT
            runs = ([(key + ' ', KEY)] if key else []) + [(name, TOLD if node == focus else NAME if node in lit else NAME_DIM)]
            right, left, row = int((nx + r) // 2) + 1, int((nx - r) // 2) - sum(len(t) for t, _ in runs), int(ny // 4)
            sides = (left, right) if node in scene.get('lefts', ()) else (right, left)
            # Its side (right, a scene's `lefts` left), the other, then a row or two up or down.
            if any(_label(text, col, row + step, runs, cols, rows) for step in (0, -1, 1, -2, 2) for col in sides):
                named += 1
    return cells(dots, text, w, h, shade), fell, v
