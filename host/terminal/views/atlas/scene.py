"""The SYSTEM ATLAS page's scene: the system as a tree in 3D, seen by an orbiting camera in braille.

The trunk stands up the y axis, the terminal at its foot and the analog front end at its crown, a layer a
STEP apart; a layer beside it stands SIDE out, at the height of the trunk's layers it ties to most. The
node open and those above it are open: a layer's folders ring it level - branches round the trunk -, a
folder's files and a file's symbols hang under it as a cap - leaves and buds -, the heaviest at its
foot and the rest spiralling out by the golden angle. A camera orbits a target (yaw, pitch, distance)
and sees it in perspective on braille's square dots: the tree's edges, the open node's ties bowed out
from the trunk so those along it are not lost in it, the nodes as discs and their names beside them;
the highest-ranked ink among a cell's dots inks the cell.
"""
import math

from coaxial.draw import braille
from tools.dev.atlas_tree import FILE, FOLDER, LAYER, LAYERS, SYMBOL

#: A layer's step up the trunk, how far out a layer beside it stands, the near plane; world units.
STEP, SIDE, NEAR = 1.0, 2.6, 0.05
#: A layer's ring of folders: its radius's base and its growth a folder. A cap's radius - its base and
#: its growth a square root of its children - by the kind it hangs from, and its half-angle.
RING, CAP, CAP_DEG = (0.6, 0.04), {FOLDER: (0.25, 0.07), FILE: (0.12, 0.025)}, 75.0
GOLDEN = math.pi * (3.0 - math.sqrt(5.0))
#: A node's disc by kind, world units, and its least and most dots; the most names drawn; the faint
#: ties between layers shown at the trunk, and an open node's ties shown each way.
DISC, DOTS, NAMES, FAINT_TIES, TIES_SHOWN = (0.11, 0.05, 0.022, 0.011), (0.8, 4.0), 48, 16, 12
#: A layer's ring round its node - the turn and tip of the view seen in it -, world units, and its sides.
HALO, SIDES = 0.24, 24
#: The inks by rank - a cell takes its highest-ranked dot's: the faint ties between layers, the tree's
#: edges far and near, the open node's ties out and in, nodes far and near, then the legend's (the dark
#: terminal's, validated all-pairs): reaches, reached by, found, open. Then text's: a name, a name told,
#: a key that opens it.
FAINT, EDGE_FAR, EDGE_NEAR, TIE_OUT, TIE_IN, NODE_FAR, NODE_NEAR, OUT, IN, HIT, OPEN, NAME, TOLD, KEY = \
    range(1, 15)
INK = {FAINT: (46, 54, 64), EDGE_FAR: (66, 76, 88), EDGE_NEAR: (104, 116, 130), TIE_OUT: (25, 158, 112),
       TIE_IN: (217, 89, 38), NODE_FAR: (128, 138, 150), NODE_NEAR: (214, 220, 228), OUT: (25, 158, 112),
       IN: (217, 89, 38), HIT: (213, 81, 129), OPEN: (57, 135, 229), NAME: (150, 160, 172),
       TOLD: (226, 231, 238), KEY: (57, 135, 229)}


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
    """The node and those above it - what is open while it is -, the layer first; none at the trunk."""
    out = []
    while node >= 0:
        out.append(node)
        node = tree.up[node]
    return out[::-1]


def layout(tree, kids, below, between, focus):
    """{node: (x, y, z)} of every node shown - the layers always, each open node's children round or under
    it -, the stubs [(trunk point, beside layer)] that tie a layer beside the trunk to it, and each open
    node's {node: reach}: its children's ring or cap radius."""
    trunk = [i for i, row in enumerate(LAYERS) if row[1] == 'trunk']
    beside = [i for i, row in enumerate(LAYERS) if row[1] != 'trunk']
    at = {layer: (0.0, k * STEP, 0.0) for k, layer in enumerate(trunk)}
    stubs, reach = [], {}
    for k, layer in enumerate(beside):
        pull = {t: between[layer][t] + between[t][layer] for t in trunk}
        y = sum(at[t][1] * w for t, w in pull.items()) / (sum(pull.values()) or 1)
        a = math.pi / 2 + 2 * math.pi * k / len(beside)
        at[layer] = (SIDE * math.cos(a), y, SIDE * math.sin(a))
        stubs.append(((0.0, y, 0.0), layer))
    for node in chain(tree, focus):
        ring = children(tree, kids, below, node)
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
    return at, stubs, reach


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


def local(v, p):
    """The world point `p` in the camera's frame: (right, up, ahead)."""
    d = (p[0] - v['eye'][0], p[1] - v['eye'][1], p[2] - v['eye'][2])
    return tuple(d[0] * v[axis][0] + d[1] * v[axis][1] + d[2] * v[axis][2] for axis in ('r', 'u', 'f'))


def _onto(v, c):
    return (v['w'] / 2 + c[0] * v['focal'] / c[2], v['h'] / 2 - c[1] * v['focal'] / c[2])


def project(v, p):
    """(x, y, depth) in dots of the world point `p`; None behind the near plane."""
    c = local(v, p)
    return _onto(v, c) + (c[2],) if c[2] >= NEAR else None


def segment(v, p, q):
    """The world segment p-q in dots, cut at the near plane; None where all of it lies behind."""
    a, b = local(v, p), local(v, q)
    if a[2] < NEAR and b[2] < NEAR:
        return None
    if a[2] < NEAR or b[2] < NEAR:
        t = (NEAR - a[2]) / (b[2] - a[2])
        cut = tuple(a[k] + (b[k] - a[k]) * t for k in range(3))
        a, b = (cut, b) if a[2] < NEAR else (a, cut)
    return _onto(v, a), _onto(v, b)


def bow(a, b, lift=1.0):
    """A tie from `a` to `b` as a curve's points, bowed away from the trunk (along x where its middle is
    on it) by `lift` times a quarter of its length and a little more."""
    mid = tuple((a[k] + b[k]) / 2 for k in range(3))
    away = _unit((mid[0], 0.0, mid[2])) if abs(mid[0]) + abs(mid[2]) > 1e-6 else (1.0, 0.0, 0.0)
    c = tuple(mid[k] + away[k] * lift * (0.25 * math.dist(a, b) + 0.2) for k in range(3))
    return [tuple((1 - t) ** 2 * a[k] + 2 * (1 - t) * t * c[k] + t * t * b[k] for k in range(3))
            for t in (i / 12 for i in range(13))]


def _clipped(a, b, w, h):
    """The part (t0, t1) of the segment a-b inside a `w` x `h` drawing (Liang-Barsky); t0 > t1 none."""
    t0, t1, dx, dy = 0.0, 1.0, b[0] - a[0], b[1] - a[1]
    for p, q in ((-dx, a[0]), (dx, w - 1e-3 - a[0]), (-dy, a[1]), (dy, h - 1e-3 - a[1])):
        if p == 0.0:
            if q < 0.0:
                return 1.0, 0.0
        elif p < 0.0:
            t0 = max(t0, q / p)
        else:
            t1 = min(t1, q / p)
    return t0, t1


class Canvas:
    """A drawing's dots, each the rank of the highest ink drawn on it, and its cells' text."""

    def __init__(self, v):
        from coaxial.model.blocks import numpy as np
        self.w, self.h = v['w'], v['h']
        self.dots = np.zeros((self.h, self.w), np.uint8)
        self.text = {}

    def line(self, a, b, ink):
        """The dots from `a` to `b` - (x, y) in dots - within the drawing."""
        from coaxial.model.blocks import numpy as np
        t0, t1 = _clipped(a, b, self.w, self.h)
        if t0 > t1:
            return
        x0, y0 = a[0] + (b[0] - a[0]) * t0, a[1] + (b[1] - a[1]) * t0
        x1, y1 = a[0] + (b[0] - a[0]) * t1, a[1] + (b[1] - a[1]) * t1
        n = int(max(abs(x1 - x0), abs(y1 - y0))) + 2
        xs = np.clip(np.linspace(x0, x1, n).astype(np.intp), 0, self.w - 1)
        ys = np.clip(np.linspace(y0, y1, n).astype(np.intp), 0, self.h - 1)
        self.dots[ys, xs] = np.maximum(self.dots[ys, xs], ink)

    def disc(self, x, y, r, ink):
        """A filled disc of radius `r` dots round (x, y)."""
        from coaxial.model.blocks import numpy as np
        x0, x1 = max(0, int(x - r)), min(self.w, int(x + r) + 1)
        y0, y1 = max(0, int(y - r)), min(self.h, int(y + r) + 1)
        if x0 < x1 and y0 < y1:
            yy, xx = np.mgrid[y0:y1, x0:x1]
            inside = (xx + 0.5 - x) ** 2 + (yy + 0.5 - y) ** 2 <= r * r
            box = self.dots[y0:y1, x0:x1]
            box[inside] = np.maximum(box[inside], ink)

    def label(self, col, row, runs):
        """Text from cell (col, row): [(text, ink)] runs; refused where a cell it needs is taken or off
        the drawing, so no name covers another."""
        width = sum(len(text) for text, _ in runs)
        if row < 0 or row >= self.h // 4 or col < 0 or col + width > self.w // 2 \
                or any((c, row) in self.text for c in range(col - 1, col + width + 1)):
            return False
        for text, ink in runs:
            for ch in text:
                self.text[(col, row)] = (ch, ink)
                col += 1
        return True

    def lines(self):
        """The drawing as lines of SGR runs: a cell its text where it has any, else its dots in braille
        in the ink of the highest rank among them."""
        from coaxial.model.blocks import numpy as np
        rows, cols = self.h // 4, self.w // 2
        cells = self.dots[:rows * 4, :cols * 2].reshape(rows, 4, cols, 2)
        bits = np.zeros((rows, cols), np.intp)
        for dy in range(4):
            for lane in range(2):
                bits |= np.where(cells[:, dy, :, lane] > 0, braille.BIT[lane][dy], 0)
        inks = cells.max(axis=(1, 3))
        out = []
        for row in range(rows):
            parts, last = [], 0
            for col, (b, k) in enumerate(zip(bits[row].tolist(), inks[row].tolist())):
                ch, k = self.text.get((col, row)) or ((braille.ALL[b], k) if b else (' ', 0))
                if k != last:
                    parts.append('\x1b[38;2;%d;%d;%dm' % INK[k] if k else '\x1b[0m')
                    last = k
                parts.append(ch)
            out.append(''.join(parts) + '\x1b[0m')
        return out


def picture(scene, cam, width, height, keyed, found):
    """The scene from `cam` on `width` x `height` cells, as lines, and where each node shown fell
    ({node: (x, y, depth)}, dots): `scene` the page's placed one, `keyed` {node: the key opening it},
    `found` the shown nodes a word's hits lie under."""
    tree, at, focus, out, into = scene['tree'], scene['at'], scene['focus'], scene['out'], scene['into']
    v, here = view(cam, width, height), cam['distance']
    canvas = Canvas(v)

    def depth_ink(p):
        return EDGE_NEAR if local(v, p)[2] < here else EDGE_FAR

    def edge(p, q, thick=False):
        seg = segment(v, p, q)
        if seg is not None:
            ink = depth_ink(tuple((p[k] + q[k]) / 2 for k in range(3)))
            for dx in ((0.0, 1.0) if thick else (0.0,)):
                canvas.line((seg[0][0] + dx, seg[0][1]), (seg[1][0] + dx, seg[1][1]), ink)

    def curve(points, ink):
        for seg in (segment(v, p, q) for p, q in zip(points, points[1:])):
            if seg is not None:
                canvas.line(*seg, ink)

    for a, b in scene['faint'] if focus < 0 else ():
        curve(bow(at[a], at[b], 0.6), FAINT)
    trunk = [i for i, row in enumerate(LAYERS) if row[1] == 'trunk']
    for a, b in zip(trunk, trunk[1:]):
        edge(at[a], at[b], thick=True)
    for layer in range(len(LAYERS)):
        x, y, z = at[layer]
        curve([(x + HALO * math.cos(a), y, z + HALO * math.sin(a))
               for a in (2 * math.pi * k / SIDES for k in range(SIDES + 1))], depth_ink(at[layer]))
    for point, layer in scene['stubs']:
        edge(point, at[layer])
    for node in at:
        if tree.kind[node] != LAYER:
            edge(at[tree.up[node]], at[node])
    for ties, ink, lift in ((into, TIE_IN, 0.6), (out, TIE_OUT, 1.0)):
        for other, _n in sorted(ties.items(), key=lambda kv: -kv[1])[:TIES_SHOWN]:
            curve(bow(at[focus], at[other], lift), ink)

    def ink_of(node, depth):
        if node == focus:
            return OPEN
        if node in found:
            return HIT
        if node in out or node in into:
            return OUT if out.get(node, 0) >= into.get(node, 0) else IN
        return NODE_NEAR if depth < here else NODE_FAR

    fell, size = {}, {}
    for node, p in at.items():
        q = project(v, p)
        if q is not None:
            fell[node] = q
            size[node] = min(DOTS[1], max(DOTS[0], DISC[tree.kind[node]] * v['focal'] / q[2]))
            canvas.disc(q[0], q[1], size[node] + (node == focus), ink_of(node, q[2]))
    tied_most = sorted(list(out.items()) + list(into.items()), key=lambda kv: -kv[1])
    order = [focus] + list(keyed) + list(range(len(LAYERS))) + [n for n, _ in tied_most] + sorted(found) \
        + sorted(fell, key=lambda n: fell[n][2])
    named, seen = 0, set()
    for node in order:
        if node in seen or node not in fell or named >= NAMES:
            continue
        seen.add(node)
        (x, y, _), r, key = fell[node], size[node], keyed.get(node)
        name = short(tree, node)
        name = name if len(name) <= 28 else name[:27] + chr(0x2026)
        runs = ([(key + ' ', KEY)] if key else []) + [(name, TOLD if node == focus or key else NAME)]
        across = sum(len(text) for text, _ in runs)
        if canvas.label(int((x + r) // 2) + 1, int(y // 4), runs) \
                or canvas.label(int((x - r) // 2) - across, int(y // 4), runs):
            named += 1
    return canvas.lines(), fell, v
