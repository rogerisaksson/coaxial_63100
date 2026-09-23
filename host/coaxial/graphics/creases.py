"""The parts' crease loops off the exact mesh: an edge where two faces fold
past OUTLINE_DEG, above the slab's measured top or under its bottom,
joined into loops and simplified along their chains.
"""
import math

from .. import orientation
from .solids import _decimated, _parametric, _slab_bottom, _slab_top


#: Outline sources kept the same way.
OUTLINES_KEPT = 4


#: THE OUTLINE: the parts' edges as a wireframe overlay, from the mesh's
#: own geometry - not from the raster. Two raster passes came before it
#: and both flickered under motion, because a per-cell test on a depth
#: buffer re-decides every cell as the picture slides a fraction of a
#: cell: a crest test inked 6 % of the cells and blinked them at the
#: ladder's top. Edges projected from FIXED vertices slide with the
#: picture instead. What an edge is: a crease where two faces meet past
#: OUTLINE_DEG (a part's top against its wall - 60 rather than 45 drops
#: chamfers and the facets of a round can), taken from the EXACT mesh
#: whatever the frame draws: the export indexed at OUTLINE_EXACT, a
#: clustering step of a hundredth of a micron that merges duplicate
#: corners and moves nothing. A decimate cannot do this job - its cell
#: is a floor on the height a part needs to keep its walls: grid 64
#: (1.56 mm cells) found 9 wide loops, grid 128 twenty, and the 1.5 mm
#: micro none at any grid, because a part under a cell tall clusters
#: flat - and the height gate below cannot come down to meet it, since
#: the slab's own corners sit a cell up too. Exact, the same export gave
#: 28 loops wider than 0.12 units including the 1.5 mm class at z 0.031,
#: for 0.37 s of indexing and 0.21 s of edges once a process (116,880
#: faces). Gated by HEIGHT ABOVE THE SLAB: only edges with a vertex
#: more than OUTLINE_RISE above the slab's top, and the slab's top is
#: MEASURED from the mesh (`_slab_top`), not assumed - it was assumed
#: to be z 0 for an evening, and the export centres on its bounding
#: box so the slab sits at z -0.069, which put the gate 5 mm up and
#: hid every part under that height, the 1.5 mm micro first. With the
#: gate a millimetre over the measured slab the same export gives 44
#: loops wider than 0.12 units, the micro's among them. The pads,
#: holes and copper on the slab's face sit within 0.035 mm of it and
#: fall under the gate; the slab's own RIM and BORE are wanted - "the
#: board's edge, enhanced too" - and come back in as the slab-level
#: crease loops wider than OUTLINE_RIM, which on the exact mesh are
#: their own loops and no tangle. Then grouped into connected LOOPS and
#: drawn only when a loop spans OUTLINE_CELLS on screen: a part smaller
#: than that is a flickering fragment, not a drawing - filtered rather
#: than drawn, on the bench's word - and comes back as the zoom brings
#: it up.
OUTLINE_DEG = 60.0
OUTLINE_RISE = 0.02


#: A slab-level loop this wide is the board's own edge (the rim is 2.0,
#: the bore 0.2); the mounting holes at 0.14 and the copper are not.
OUTLINE_RIM = 0.19


#: And per LOOP, in model units: edges adding up to more than this many
#: widths is crease detail, not an outline. See _outline_loops.
OUTLINE_DENSITY = 6.0


#: How far a vertex may sit from the slab's measured top and still count
#: as on it: the copper and mask layers are 0.0007 units (35 um) proud.
OUTLINE_LEVEL = 0.003


OUTLINE_EXACT = 200000


#: Consecutive creases that bend by less than this chord deviation, in
#: model units (0.15 mm), are ONE segment for the drawing. The export
#: tessellates a fillet into edges of a third of a millimetre and a
#: can's rim into dozens: measured at the view's zoom, 914 of the 1,458
#: edges in the 95 loops drawn were under half a cell face-on - every
#: one on OUTLINE_MIN_EDGE's threshold, popping in and out as the view
#: turned ("Schroedinger's pixels", the bench). Merged once per solid,
#: Douglas-Peucker along each chain of a loop: 778 edges from 1,458,
#: 20 % under half a cell from 63 %; the rim's 136 facets of 2.3 mm
#: become 64 chords of 4.9 mm, 0.06 mm off the circle; a fillet is a
#: bevel or two. A pin field's zigzag turns too sharply to merge and
#: stays under the min-edge filter as before. Blink events in a
#: 0.6-degree-a-frame tumble at the view's size: 227 to 102 over 38
#: frames, with the fixed-phase dots below.
OUTLINE_CHORD = 0.003


def _features(solid, min_deg=OUTLINE_DEG, min_rise=None):
    """[(a, b)] vertex pairs: the creases of `solid`, above `min_rise` when
    one is given.
    """
    pos, idx, nrm = solid
    cos_lim = math.cos(math.radians(min_deg))
    shared = {}
    for t in range(len(idx) // 3):
        tri = (idx[3 * t], idx[3 * t + 1], idx[3 * t + 2])
        for i in range(3):
            a, b = tri[i], tri[(i + 1) % 3]
            shared.setdefault((a, b) if a < b else (b, a), []).append(t)
    out = []
    for (a, b), tris in shared.items():
        if (min_rise is not None
                and max(pos[3 * a + 2], pos[3 * b + 2]) <= min_rise):
            continue
        if len(tris) == 1:
            out.append((a, b))
            continue
        t0, t1 = tris[0], tris[1]
        dot = (nrm[3 * t0] * nrm[3 * t1] + nrm[3 * t0 + 1] * nrm[3 * t1 + 1]
               + nrm[3 * t0 + 2] * nrm[3 * t1 + 2])
        if abs(dot) < cos_lim:
            out.append((a, b))
    return out


def _loops(edges, pos):
    """[(extent, [(a, b), ...])]: the connected pieces of an edge set, each
    with its XY extent in model units - what the size filter judges.
    """
    parent = {}

    def find(v):
        while parent.setdefault(v, v) != v:
            parent[v] = parent[parent[v]]
            v = parent[v]
        return v

    for a, b in edges:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    groups = {}
    for a, b in edges:
        groups.setdefault(find(a), []).append((a, b))
    out = []
    for members in groups.values():
        verts = {v for e in members for v in e}
        xs = [pos[3 * v] for v in verts]
        ys = [pos[3 * v + 1] for v in verts]
        out.append((max(max(xs) - min(xs), max(ys) - min(ys)), members))
    return out


def _chains(members):
    """A loop's edges as vertex chains: walked from every vertex that is
    not a plain link - an end, or a corner where three creases meet -
    then whatever closed rings are left, each ending on its start."""
    adj = {}
    for a, b in members:
        adj.setdefault(a, []).append(b)
        adj.setdefault(b, []).append(a)
    used = set()

    def walk(start, nxt):
        chain = [start, nxt]
        used.add((start, nxt) if start < nxt else (nxt, start))
        prev, at = start, nxt
        while len(adj[at]) == 2 and at != start:
            nxt = adj[at][0] if adj[at][1] == prev else adj[at][1]
            key = (at, nxt) if at < nxt else (nxt, at)
            if key in used:
                break
            used.add(key)
            chain.append(nxt)
            prev, at = at, nxt
        return chain

    chains = []
    ends = [(v, nxt) for v, links in adj.items() if len(links) != 2
            for nxt in links]
    for v, nxt in ends:
        if ((v, nxt) if v < nxt else (nxt, v)) not in used:
            chains.append(walk(v, nxt))
    for a, b in members:
        if ((a, b) if a < b else (b, a)) not in used:
            chains.append(walk(a, b))
    return chains


def _simplify(chain, pos, tol):
    """The chain's vertices with every one within `tol` of the chord
    between its kept neighbours dropped: Douglas-Peucker, iterative,
    in the model's three dimensions."""
    keep = [False] * len(chain)
    keep[0] = keep[-1] = True
    stack = [(0, len(chain) - 1)]
    while stack:
        i, j = stack.pop()
        if j - i < 2:
            continue
        a, b = 3 * chain[i], 3 * chain[j]
        ax, ay, az = pos[a], pos[a + 1], pos[a + 2]
        dx, dy, dz = pos[b] - ax, pos[b + 1] - ay, pos[b + 2] - az
        dd = dx * dx + dy * dy + dz * dz
        worst, where = tol * tol, -1
        for k in range(i + 1, j):
            p = 3 * chain[k]
            px, py, pz = pos[p] - ax, pos[p + 1] - ay, pos[p + 2] - az
            if dd:
                t = (px * dx + py * dy + pz * dz) / dd
                t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
                px, py, pz = px - t * dx, py - t * dy, pz - t * dz
            d2 = px * px + py * py + pz * pz
            if d2 > worst:
                worst, where = d2, k
        if where >= 0:
            keep[where] = True
            stack.append((i, where))
            stack.append((where, j))
    return [v for v, k in zip(chain, keep) if k]


#: The outline's loops per source solid, built once per process.
_OUTLINES = {}


def _outline_source():
    """(solid, loops) the outline draws from: the export indexed exact (see
    OUTLINE_EXACT), or the parametric board where there is none.
    """
    try:
        solid = _decimated(orientation.MODEL, OUTLINE_EXACT)
    except (OSError, ValueError):
        solid = _parametric()
    got = _OUTLINES.get(id(solid))
    if got is not None:
        return got
    if len(_OUTLINES) > OUTLINES_KEPT:
        _OUTLINES.clear()
    got = _OUTLINES[id(solid)] = (solid, _outline_loops(solid))
    return got


def _outline_loops(solid):
    """The loops the outline draws: every part standing OUTLINE_RISE over
    the measured slab, and every part hanging OUTLINE_RISE under its
    bottom face.
    """
    pos = solid[0]
    top = _slab_top(pos)
    bottom = _slab_bottom(pos, top)
    gate = top + OUTLINE_RISE
    sink = None if bottom is None else bottom - OUTLINE_RISE

    # Split by height BEFORE grouping.
    parts, below = [], []
    for a, b in _features(solid):
        za, zb = pos[3 * a + 2], pos[3 * b + 2]
        if max(za, zb) > gate:
            parts.append((a, b))
        elif sink is not None and min(za, zb) < sink \
                and not (abs(za - bottom) <= OUTLINE_LEVEL
                         and abs(zb - bottom) <= OUTLINE_LEVEL):
            below.append((a, b))
    loops = _loops(parts, pos) + _loops(below, pos)

    # And a loop that is more crease than outline - its edges adding up to over
    # OUTLINE_DENSITY times its width - is a pin field or a fin stack, not a
    # part's shape.
    def sparse(extent, members):
        length = 0.0
        for a, b in members:
            length += math.sqrt((pos[3 * a] - pos[3 * b]) ** 2
                                + (pos[3 * a + 1] - pos[3 * b + 1]) ** 2
                                + (pos[3 * a + 2] - pos[3 * b + 2]) ** 2)
        return length <= OUTLINE_DENSITY * extent

    # What remains is merged along its chains (OUTLINE_CHORD) - after the
    # density gate, which judges the creases as the mesh has them.
    drawn = []
    for extent, members in loops:
        if not sparse(extent, members):
            continue
        merged = []
        for chain in _chains(members):
            kept = _simplify(chain, pos, OUTLINE_CHORD)
            merged.extend(zip(kept, kept[1:]))
        drawn.append((extent, merged))
    return drawn
