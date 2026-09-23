"""The pre-scan: each part's crease loops fitted once to a block, drum, arch, stroke or wall hole.

Fitted off the exact mesh, so the overlay draws fixed edges and corners
instead of the tessellation's own folds.
"""
import math

from coaxial.graphics.creases import OUTLINE_LEVEL, OUTLINE_RISE, OUTLINES_KEPT, _outline_source
from coaxial.graphics.solids import _slab_bottom, _slab_top


#: THE PARTS AS BLOCKS AND DRUMS - the pre-scan. A part's crease loops
#: are wherever its tessellation folds past OUTLINE_DEG: a box's lid and
#: corners, but on a rounded part - the CM choke, the fuse - the
#: rounding's own facets, which make an edge OF THE CORNER and come
#: and go as the view turns; and a rounded extrusion folds only at its
#: two end profiles, two arches and nothing along it. So each part is
#: fitted ONCE, off the exact mesh, to the simple geometry it is - a
#: BLOCK (its footprint's least oriented box, lid and four legs), a DRUM
#: (a lone lid on one radius: the circle, and per frame the two
#: silhouette lines), or the sharp ARCH of an unpaired profile - and the
#: overlay draws that, edges and corners fixed, over the render (the
#: bench, 2026-09-23: "simplify the object to simple geometries, like
#: a block, and enhance its edges and corners"). Loops of one side
#: whose footprints overlap are one part; two arches of one width and
#: height, facing across, are one block.
STEREO_CIRCLE = 0.03   # a lid's radii vary under this share: a drum
STEREO_EVEN = 2.5      # ...with its corners' angular gaps within this
# ratio: a chamfered square's corners share a radius too, but crowd in pairs at
# the chamfers
STEREO_PAIR = 0.05     # arches pair when width and height agree this close
STEREO_TURN = 0.02     # a block turns off the board's axes only to save this
STEREO_SEGMENTS = 24   # a drum's circle
#: Two loops are one part when their footprints' overlap is this share
#: of the smaller one: a base ring under a lid, a rounding's facets on
#: the lid's footprint. Any overlap at all chained neighbours - five
#: capacitors each swallowed 85 to 91 loops of the pin fields round
#: them and none came out a drum.
STEREO_NEST = 0.6


_STEREO = {}


def _stereotypes():
    """[(kind, extent, data)] for the outline source, built once per
    source: 'block' and 'arch' carry 3D segments, 'drum' (cx, cy, r,
    ztop, zbase)."""
    solid, loops = _outline_source()
    got = _STEREO.get(id(loops))
    if got is None:
        if len(_STEREO) > OUTLINES_KEPT:
            _STEREO.clear()
        pos = solid[0]
        top = _slab_top(pos)
        got = _STEREO[id(loops)] = _stereotype_loops(
            pos, loops, top, _slab_bottom(pos, top))
    return got


def _stereotype_loops(pos, loops, top, bottom):
    """The primitives for `loops` over `pos`: one per part
    (`_part_groups`), the arches paired or sharp, the holes in walls
    as ovals."""
    groups, arches, holes = _part_groups(pos, loops, top, bottom)
    prims = []
    for over, rings in groups:
        prims += _part_primitives(over, rings, top, bottom)
    prims += _pair_arches(arches, top, bottom)
    return _merged(prims, top) + _wall_holes(holes)


def _footprint(kind, data):
    """((x0, x1, y0, y1), (z0, z1)) of a primitive on the board."""
    if kind == 'drum':
        cx, cy, r, ztop, zbase = data
        return ((cx - r, cx + r, cy - r, cy + r),
                (min(ztop, zbase), max(ztop, zbase)))
    segs = data if kind in ('block', 'ring', 'stroke') else data[0]
    xs = [s[0] for s in segs] + [s[3] for s in segs]
    ys = [s[1] for s in segs] + [s[4] for s in segs]
    zs = [s[2] for s in segs] + [s[5] for s in segs]
    return (min(xs), max(xs), min(ys), max(ys)), (min(zs), max(zs))


def _merged(prims, top):
    """The primitives with every block, arch and stroke whose footprint
    nests in a block's of the same side (`_nested`, the rule the loops
    are grouped by) absorbed into that block: the box round both, to the
    taller's height.
    """
    order = sorted((i for i, p in enumerate(prims)
                    if p[0] in ('block', 'arch', 'stroke')),
                   key=lambda i: -prims[i][1])
    taken, grown = set(), {}
    for n, i in enumerate(order):
        kind, _extent, data = prims[i]
        if i in taken or kind != 'block':
            continue
        box, (z0, z1) = _footprint(kind, data)
        over = z1 > top
        ztop, zbase = (z1, z0) if over else (z0, z1)
        corners = [(s[0], s[1]) for s in data] + [(s[3], s[4]) for s in data]
        for j in order[n + 1:]:
            if j in taken:
                continue
            kj, _ej, dj = prims[j]
            fp, (w0, w1) = _footprint(kj, dj)
            if (w1 > top) != over or not _nested(fp, box):
                continue
            taken.add(j)
            segs = dj if kj in ('block', 'stroke') else dj[0]
            corners += [(s[0], s[1]) for s in segs] + [(s[3], s[4]) for s in segs]
            ztop = max(ztop, w1) if over else min(ztop, w0)
            box = (min(box[0], fp[0]), max(box[1], fp[1]),
                   min(box[2], fp[2]), max(box[3], fp[3]))
            base = _box_along(corners, _box_angle(corners[:4]))
            grown[i] = ('block', _box_extent(base),
                        _block(base, base, ztop, zbase))
    return [grown.get(i, p) for i, p in enumerate(prims) if i not in taken]


def _part_groups(pos, loops, top, bottom):
    """([(over, points, loops)], arches, holes): the loops of one side whose
    footprints nest (STEREO_NEST), gathered into parts - a part's
    rounding, base and lid fold into separate loops on one footprint -
    and, apart, the loops lying in one vertical plane: a rounded part's
    end profiles, which reach the base, and the holes in its walls, which
    float above it (the screw terminals' openings, 40 of them 0.016 over
    the slab - drawn as arches they were rectangles).
    """
    parts, arches, holes = [], [], []
    for _extent, members in loops:
        verts = sorted({v for a, b in members for v in (a, b)})
        p3 = [(pos[3 * v], pos[3 * v + 1], pos[3 * v + 2]) for v in verts]
        zs = [p[2] for p in p3]
        over = max(zs) > top + OUTLINE_RISE
        line = _collinear([(p[0], p[1]) for p in p3])
        # A loop in a vertical plane with no height is a ridge along a part's
        # top - the choke's - and belongs to the part's block.
        if (line is not None and len(p3) >= 3
                and max(zs) - min(zs) > 3 * OUTLINE_LEVEL):
            base = top if over else bottom
            clear = ((min(zs) - base) if over else (base - max(zs))
                     if base is not None else 0.0)
            # A profile's legs reach the base within a millimetre; the screw
            # terminals' openings float 0.016 over it.
            (holes if clear > 3 * OUTLINE_LEVEL else arches).append(
                (over, p3, line))
            continue
        xs = [p[0] for p in p3]
        ys = [p[1] for p in p3]
        parts.append((over, p3, (min(xs), max(xs), min(ys), max(ys))))
    parent = list(range(len(parts)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    # A sweep along x: sorted by their left edge, a footprint is only tried
    # against the ones starting before its right edge.
    order = sorted(range(len(parts)), key=lambda i: parts[i][2][0])
    for n, i in enumerate(order):
        oi, _pi, a = parts[i]
        for j in order[n + 1:]:
            oj, _pj, b = parts[j]
            if b[0] > a[1]:
                break
            if oi == oj and _nested(a, b):
                parent[find(i)] = find(j)
    groups = {}
    for i, (over, p3, _box) in enumerate(parts):
        groups.setdefault(find(i), [over, []])[1].append(p3)
    return [(over, rings) for over, rings in groups.values()], arches, holes


def _nested(a, b):
    """Whether footprints `a` and `b` (x0, x1, y0, y1) overlap by
    STEREO_NEST of the smaller; a footprint with no area - a ridge, a
    point - nests when it lies inside the other."""
    ax0, ax1, ay0, ay1 = a
    bx0, bx1, by0, by1 = b
    dx = min(ax1, bx1) - max(ax0, bx0)
    dy = min(ay1, by1) - max(ay0, by0)
    if dx < 0.0 or dy < 0.0:
        return False
    small = min((ax1 - ax0) * (ay1 - ay0), (bx1 - bx0) * (by1 - by0))
    if small <= 1e-12:
        return (dx >= 0.999 * min(ax1 - ax0, bx1 - bx0)
                and dy >= 0.999 * min(ay1 - ay0, by1 - by0))
    return dx * dy >= STEREO_NEST * small


def _wall_holes(holes):
    """Each hole in a wall (over, points, (direction, middle)) as the
    oval inscribed in its bounds within its plane, twelve segments."""
    prims = []
    for _over, p3, ((ux, uy), (mx, my)) in holes:
        us = [(p[0] - mx) * ux + (p[1] - my) * uy for p in p3]
        zs = [p[2] for p in p3]
        u0, u1, z0, z1 = min(us), max(us), min(zs), max(zs)
        cu, cz = 0.5 * (u0 + u1), 0.5 * (z0 + z1)
        ru, rz = 0.5 * (u1 - u0), 0.5 * (z1 - z0)
        ring = []
        for k in range(12):
            a = 2.0 * math.pi * k / 12
            u, z = cu + ru * math.cos(a), cz + rz * math.sin(a)
            ring.append((mx + u * ux, my + u * uy, z))
        segs = [ring[k] + ring[(k + 1) % 12] for k in range(12)]
        prims.append(('hole', u1 - u0, (segs, (-uy, ux))))
    return prims


def _part_primitives(over, rings, top, bottom):
    """A part's primitives off its loops' points."""
    pts = [p for ring in rings for p in ring]
    zs = [p[2] for p in pts]
    ztop = max(zs) if over else min(zs)
    zbase = top if over else bottom
    if zbase is None:
        zbase = ztop
    widest = max(rings, key=lambda ring: math.hypot(
        max(p[0] for p in ring) - min(p[0] for p in ring),
        max(p[1] for p in ring) - min(p[1] for p in ring)))
    prims = []
    for ring in rings:
        circle = _ring_circle(ring, over)
        if circle is None:
            continue
        (cx, cy, r), z = circle
        if ring is widest:
            prims.append(('drum', 2.0 * r, (cx, cy, r, ztop, zbase)))
        else:
            prims.append(('ring', 2.0 * r, _circle_segments(cx, cy, r, z)))
    if prims and prims[0][0] == 'drum':
        return prims
    if len(pts) < 3:
        # One edge of the mesh - a wire's silhouette, a pin - is its own
        # stroke, not a box round a diagonal.
        return [('stroke', math.dist(pts[0][:2], pts[-1][:2]),
                 [pts[0] + pts[-1]])]
    angle = _box_angle([(p[0], p[1]) for p in widest])
    base = _box_along([(p[0], p[1]) for p in pts], angle)
    # A BLOCK, lid over base.
    return [('block', _box_extent(base), _block(base, base, ztop, zbase))] + prims


def _ring_circle(ring, over):
    """((cx, cy, r), z) when a loop's top corners - six or more - sit on one
    radius at even spacing round it, else None.
    """
    crest = max(p[2] for p in ring) if over else min(p[2] for p in ring)
    lid = [(p[0], p[1]) for p in ring if abs(p[2] - crest) <= 3 * OUTLINE_LEVEL]
    if len(lid) < 6:
        return None
    fit, dev = _circle_fit(lid)
    if dev >= STEREO_CIRCLE:
        return None
    cx, cy, _r = fit
    angles = sorted(math.atan2(p[1] - cy, p[0] - cx) for p in lid)
    gaps = [b - a for a, b in zip(angles, angles[1:])]
    gaps.append(angles[0] + 2.0 * math.pi - angles[-1])
    if max(gaps) > STEREO_EVEN * min(gaps):
        return None
    return fit, crest


def _circle_segments(cx, cy, r, z):
    ring = [(cx + r * math.cos(2.0 * math.pi * k / STEREO_SEGMENTS),
             cy + r * math.sin(2.0 * math.pi * k / STEREO_SEGMENTS))
            for k in range(STEREO_SEGMENTS)]
    return [ring[k] + (z,) + ring[(k + 1) % STEREO_SEGMENTS] + (z,)
            for k in range(STEREO_SEGMENTS)]


def _pair_arches(arches, top, bottom):
    """Each arch (over, points, (direction, middle)) sharp - up, across,
    down - and two of one width and height that face each other across
    their planes joined into one block."""
    fitted = []
    for over, p3, ((ux, uy), (mx, my)) in arches:
        zs = [p[2] for p in p3]
        ztop = max(zs) if over else min(zs)
        zbase = top if over else bottom
        if zbase is None:
            zbase = ztop
        us = [(p[0] - mx) * ux + (p[1] - my) * uy for p in p3]
        # the top's own flat, narrower than the base on a rounded profile: the
        # lid runs between its ends, the legs lean in
        tops = [u for u, p in zip(us, p3)
                if abs(p[2] - ztop) <= 3 * OUTLINE_LEVEL] or us
        at = (lambda u: (mx + u * ux, my + u * uy))
        fitted.append(((ux, uy), (mx, my), at(min(us)), at(max(us)),
                       at(min(tops)), at(max(tops)), ztop, zbase, over))
    used, prims = set(), []
    for i, (u, mid, a0, a1, t0, t1, ztop, zbase, over) in enumerate(fitted):
        if i in used:
            continue
        used.add(i)
        width, height = math.dist(a0, a1), abs(ztop - zbase)
        best = None
        for j, (u2, mid2, b0, b1, s0, s1, zt2, zb2, over2) in enumerate(fitted):
            if j in used or over2 != over:
                continue
            if abs(abs(u[0] * u2[0] + u[1] * u2[1]) - 1.0) > 0.01:
                continue
            if (abs(math.dist(b0, b1) - width) > STEREO_PAIR * width
                    or abs(abs(zt2 - zb2) - height) > STEREO_PAIR * height):
                continue
            dx, dy = mid2[0] - mid[0], mid2[1] - mid[1]
            along = abs(dx * u[0] + dy * u[1])
            across = abs(dy * u[0] - dx * u[1])
            if along > 0.1 * width or across < 0.2 * height:
                continue
            if best is None or across < best[0]:
                best = (across, j, b0, b1, s0, s1)
        if best is None:
            prims.append(('arch', width, ([
                (a0[0], a0[1], zbase, t0[0], t0[1], ztop),
                (t0[0], t0[1], ztop, t1[0], t1[1], ztop),
                (t1[0], t1[1], ztop, a1[0], a1[1], zbase)], (-u[1], u[0]))))
            continue
        across, j, b0, b1, _s0, _s1 = best
        used.add(j)
        if math.dist(a0, b0) > math.dist(a0, b1):
            b0, b1 = b1, b0
        base = (a0, a1, b1, b0)
        prims.append(('block', max(width, across),
                      _block(base, base, ztop, zbase)))
    return prims


def _block(base, lid, ztop, zbase):
    """The twelve segments of a block: its lid's four edges at `lid`, its
    base's four at `base`, and a leg from each base corner up to the
    lid's - straight where lid and base agree, leaning in where the crest
    is narrower.
    """
    segs = []
    for i in range(4):
        j = (i + 1) % 4
        segs.append(lid[i] + (ztop,) + lid[j] + (ztop,))
        segs.append(base[i] + (zbase,) + base[j] + (zbase,))
        segs.append(lid[i] + (ztop,) + base[i] + (zbase,))
    return segs


def _box_extent(corners):
    return max(math.dist(corners[0], corners[1]),
               math.dist(corners[1], corners[2]))


def _box_angle(pts):
    """The angle, in 2-degree steps, of the least-area box round `pts` - the
    board's own axes unless turning saves more than STEREO_TURN of the
    area.
    """
    areas = []
    for deg in range(0, 90, 2):
        a = math.radians(deg)
        ca, sa = math.cos(a), math.sin(a)
        us = [p[0] * ca + p[1] * sa for p in pts]
        vs = [-p[0] * sa + p[1] * ca for p in pts]
        areas.append(((max(us) - min(us)) * (max(vs) - min(vs)), a))
    least = min(areas)[0]
    if areas[0][0] <= least * (1.0 + STEREO_TURN):
        return 0.0
    return min(areas)[1]


def _box_along(pts, a):
    """The corners of the box round `pts` in the frame turned by `a`,
    in one fixed order, so two boxes in one frame pair corner by
    corner."""
    ca, sa = math.cos(a), math.sin(a)
    us = [p[0] * ca + p[1] * sa for p in pts]
    vs = [-p[0] * sa + p[1] * ca for p in pts]
    u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
    return [(u * ca - v * sa, u * sa + v * ca)
            for u, v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1))]


def _drum_segments(data, camx, camy):
    """A drum's circle at its lid, and the two silhouette lines the
    camera at (camx, camy) sees: the tangents from it to the circle."""
    cx, cy, r, ztop, zbase = data
    ring = [(cx + r * math.cos(2.0 * math.pi * k / STEREO_SEGMENTS),
             cy + r * math.sin(2.0 * math.pi * k / STEREO_SEGMENTS))
            for k in range(STEREO_SEGMENTS)]
    segs = []
    for k in range(STEREO_SEGMENTS):
        (x0, y0), (x1, y1) = ring[k], ring[(k + 1) % STEREO_SEGMENTS]
        segs.append((x0, y0, ztop, x1, y1, ztop))
    dx, dy = cx - camx, cy - camy
    reach = math.hypot(dx, dy)
    if reach > r:
        spread = math.acos(r / reach)
        base = math.atan2(dy, dx)
        for t in (base + spread, base - spread):
            gx, gy = cx + r * math.cos(t), cy + r * math.sin(t)
            segs.append((gx, gy, ztop, gx, gy, zbase))
    return segs


def _box_fit(pts):
    """(corners, dev): the least-area box round `pts` over angles in
    2-degree steps, and the points' mean distance to its boundary over
    its shorter side - zero when every point sits on a side."""
    best = None
    for deg in range(0, 90, 2):
        a = math.radians(deg)
        ca, sa = math.cos(a), math.sin(a)
        us = [p[0] * ca + p[1] * sa for p in pts]
        vs = [-p[0] * sa + p[1] * ca for p in pts]
        u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
        area = (u1 - u0) * (v1 - v0)
        if best is None or area < best[0]:
            best = (area, a, u0, u1, v0, v1, us, vs)
    _area, a, u0, u1, v0, v1, us, vs = best
    short = max(1e-9, min(u1 - u0, v1 - v0))
    dev = sum(min(u - u0, u1 - u, v - v0, v1 - v)
              for u, v in zip(us, vs)) / len(us) / short
    ca, sa = math.cos(a), math.sin(a)
    corners = [(u * ca - v * sa, u * sa + v * ca)
               for u, v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1))]
    return corners, dev


def _circle_fit(pts):
    """((cx, cy, r), dev): the least-squares circle's centre, the mean
    radius about it, and the radii's spread over the mean. Not the
    centroid: unevenly spaced points pull it off-centre - a DC bus
    screw hole read dev 0.047 and was drawn as a box (2026-09-23)."""
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    us = [p[0] - mx for p in pts]
    vs = [p[1] - my for p in pts]
    suu = sum(u * u for u in us)
    svv = sum(v * v for v in vs)
    suv = sum(u * v for u, v in zip(us, vs))
    ru = 0.5 * sum(u * (u * u + v * v) for u, v in zip(us, vs))
    rv = 0.5 * sum(v * (u * u + v * v) for u, v in zip(us, vs))
    det = suu * svv - suv * suv
    cx, cy = mx, my
    if det > 1e-18:
        cx += (ru * svv - rv * suv) / det
        cy += (rv * suu - ru * suv) / det
    rs = [math.hypot(p[0] - cx, p[1] - cy) for p in pts]
    r = sum(rs) / n
    dev = math.sqrt(sum((x - r) ** 2 for x in rs) / n) / r if r else 1.0
    return (cx, cy, r), dev


def _collinear(pts):
    """((ux, uy), (mx, my)) for points on one line - the line's
    direction and their middle - or None."""
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    syy = sum((p[1] - my) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    trace, det = sxx + syy, sxx * syy - sxy * sxy
    disc = math.sqrt(max(0.0, trace * trace / 4.0 - det))
    big, small = trace / 2.0 + disc, trace / 2.0 - disc
    # The mesh's coordinates carry a tenth of a millimetre of noise: a
    # three-corner ridge 0.14 long spread 1e-4 across it, past 1e-6.
    if big <= 0.0 or small > 1e-4 * big:
        return None
    ang = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    return (math.cos(ang), math.sin(ang)), (mx, my)
