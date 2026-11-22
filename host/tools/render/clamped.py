"""The printed clamps' shapes: turned slit bands, cages, blocks cupped round a drum.

Each laid along x and printable from a face across it, nothing past 45 degrees: the knee's and the leg's sketches
(knee_clamped, leg_clamped) build their pieces of them.
"""
import math

from tools.render import elbow_mount as em
from tools.render import mounting as m

#: The 45-degree chamfer on each printed edge, and the round on a block's upright corners, m.
EDGE_M, FILLET_M = 0.0015, 0.005
#: A cage's honeycomb: a cell's width round and its upright sides' length, the struts' width, the solid rims at its
#: ends, m.
CELL_M, SIDE_M, STRUT_M, RIM_M = 0.008, 0.004, 0.0018, 0.003


def hull(stations, step=0.0005):
    """The drums [(x0, x1, r)] a band's profile [(x, bore, outside)] stays inside, its cones cut `step` long."""
    out = []
    for (xa, _ia, oa), (xb, _ib, ob) in zip(stations[:-1], stations[1:]):
        n = max(1, int(math.ceil((xb - xa) / step))) if xb > xa else 0
        out += [(xa + (xb - xa) * i / n, xa + (xb - xa) * (i + 1) / n,
                 max(oa + (ob - oa) * i / n, oa + (ob - oa) * (i + 1) / n)) for i in range(n)]
    return out


def turned(stations, slit, colour=em.FRAME, k=48, gap=None):
    """A slit clamp band turned to a profile [(x, bore, outside)] rising along x - a step where two stations share an
    x, a cone between two that don't - its straight ends chamfered EDGE_M at 45 degrees without steepening an
    adjacent cone, and only its slit at `slit` (y = r cos) showing; whole round, a ring, with `slit` None; open
    `gap` rad about `slit` where told - a window, no slit."""
    import numpy as np
    (xa, ia, oa), (x1, i1, o1) = stations[0], stations[1]
    (xn, in_, on_), (xb, ib, ob) = stations[-2], stations[-1]
    head = [(xa, ia, oa)]
    if x1 - xa > 1e-9:
        ea = min(EDGE_M, 0.45 * (x1 - xa), 0.4 * (oa - ia))
        ci, co = abs(i1 - ia) <= 1e-6, abs(o1 - oa) <= 1e-6
        head = ([(xa, ia + (ea if ci else 0.0), oa - (ea if co else 0.0)),
                 (xa + ea, ia if ci else ia + (i1 - ia) * ea / (x1 - xa),
                  oa if co else oa + (o1 - oa) * ea / (x1 - xa))] if ci or co else head)
    tail = [(xb, ib, ob)]
    if xb - xn > 1e-9:
        eb = min(EDGE_M, 0.45 * (xb - xn), 0.4 * (ob - ib))
        ci, co = abs(ib - in_) <= 1e-6, abs(ob - on_) <= 1e-6
        tail = ([(xb - eb, ib if ci else ib - (ib - in_) * eb / (xb - xn),
                  ob if co else ob - (ob - on_) * eb / (xb - xn)),
                 (xb, ib + (eb if ci else 0.0), ob - (eb if co else 0.0))] if ci or co else tail)
    laid_out = head + list(stations[1:-1]) + tail
    gap = (0.0 if slit is None else em.SLIT_M / min(r for _x, r, _o in stations)) if gap is None else gap
    turns = np.linspace((slit or 0.0) + 0.5 * gap, (slit or 0.0) + 2.0 * math.pi - 0.5 * gap, k)

    def outline(x, inner, outer):
        rim = [(x, outer * math.cos(a), outer * math.sin(a)) for a in turns]
        bore = [(x, inner * math.cos(a), inner * math.sin(a)) for a in turns[::-1]]
        return np.array(rim + bore)
    layers = [outline(*s) for s in laid_out]
    tris, n = [], 2 * k
    for lo, hi in zip(layers[:-1], layers[1:]):
        for i in range(n):
            j = (i + 1) % n
            tris += [(np.array((lo[i], lo[j], hi[j])), colour), (np.array((lo[i], hi[j], hi[i])), colour)]
    for cap in (layers[0], layers[-1]):
        for j in range(k - 1):
            q = (cap[j], cap[j + 1], cap[n - 2 - j], cap[n - 1 - j])
            tris += [(np.array((q[0], q[1], q[2])), colour), (np.array((q[0], q[2], q[3])), colour)]
    return tris


def honeycomb(inner, outer, x0, x1, colour=em.FRAME, gap=None, gaps=None):
    """A cage's wall round x between radii from x0 to x1: a gabled solid rim at each end, and between them whole rows
    of honeycomb cells with upright sides and 45-degree roofs at `outer` - printable along x either way with zero
    bridges; open over `gap` (rad, rad, from +y toward +z) or `gaps`, framed by upright border posts."""
    import numpy as np
    open_ = gaps or (gap, gap)
    mid = 0.5 * (inner + outer)
    n = max(6, int(round(2.0 * math.pi * mid / CELL_M)))
    a, circ, slope = 2.0 * math.pi * mid / n, 2.0 * math.pi * mid, outer / mid
    roof, edge = 0.5 * a * slope, 0.0015
    x_lo, x_hi = x0 + edge + roof, x1 - edge - roof
    if x_hi <= x_lo + 0.002:
        return m._band_x(inner, outer, x0, x1, colour, k=96, gap=open_[0])
    rows = max(1, int(round((x_hi - x_lo - SIDE_M) / (SIDE_M + roof))))
    side = (x_hi - x_lo - rows * roof) / (rows + 1)

    def in_gap(u, g):
        if not g:
            return False
        span = (g[1] - g[0]) % (2.0 * math.pi)
        return ((u / mid - g[0]) % (2.0 * math.pi)) < span

    def at(u, x, r):
        return np.array((x, r * math.cos(u / mid), r * math.sin(u / mid)))

    def quad(p0, p1, p2, p3):
        return [(np.array((p0, p1, p2)), colour), (np.array((p0, p2, p3)), colour)]

    tris = []

    def add_strut(p, q):
        d = q - p
        ln = float(np.linalg.norm(d))
        if ln < 1e-6:
            return
        d /= ln
        p, q = p - 0.35 * STRUT_M * d, q + 0.35 * STRUT_M * d
        w = np.array([-d[1], d[0]]) * 0.5 * STRUT_M
        corners = [p + w, q + w, q - w, p - w]
        ri = [at(u, x, inner) for u, x in corners]
        ro = [at(u, x, outer) for u, x in corners]
        for s in range(4):
            e = (s + 1) % 4
            tris.extend([(np.array((ri[s], ri[e], ro[e])), colour), (np.array((ri[s], ro[e], ro[s])), colour)])
        tris.extend(quad(ro[0], ro[1], ro[2], ro[3]) + quad(ri[0], ri[3], ri[2], ri[1]))

    posts, seen = [], set()
    for g in open_:
        if g is not None and g not in seen:
            seen.add(g)
            posts += [(g[0] * mid - 0.5 * STRUT_M) % circ, (g[1] * mid + 0.5 * STRUT_M) % circ]
    for up in posts:
        add_strut(np.array([up, x0 + 0.4 * STRUT_M]), np.array([up, x1 - 0.4 * STRUT_M]))

    def clip_seg(u, xb, du):
        p, q = np.array([u, xb]), np.array([u + du, xb + roof])
        if not any(in_gap(u + s * du, g) for s in np.linspace(0.0, 1.0, 9) for g in open_):
            return p, q
        for post in posts:
            dp = ((post - u + 0.5 * circ) % circ) - 0.5 * circ
            if du * dp > 0.0 and abs(dp) < abs(du) and not any(in_gap(u + 0.5 * dp, g) for g in open_):
                return p, np.array([u + dp, xb + roof * (dp / du)])
        return None

    bot_u, top_u = list(posts), list(posts)
    for row in range(rows + 1):
        xa = x_lo + row * (side + roof)
        xb = xa + side
        ya, yb = (x_lo - 0.0005 if row == 0 else xa), (x_hi + 0.0005 if row == rows else xb)
        for i in range(n):
            u = (i + 0.5 * (row % 2)) * a
            if not any(in_gap(u, g) for g in open_):
                add_strut(np.array([u, ya]), np.array([u, yb]))
                (bot_u if row == 0 else []).append(u % circ)
                (top_u if row == rows else []).append(u % circ)
            for du in ((-0.5 * a, 0.5 * a) if row < rows else ()):
                seg = clip_seg(u, xb, du)
                if seg is not None:
                    add_strut(*seg)

    def dist(u, anchors):
        if not anchors:
            return 0.0
        return min(abs(((u - s + 0.5 * circ) % circ) - 0.5 * circ) for s in anchors)

    def rim_strip(g, anchors, low_end):
        steps = 8 * n
        if g:
            u0 = (g[1] * mid) % circ
            span = circ - ((g[1] - g[0]) % (2.0 * math.pi)) * mid
            us = np.linspace(u0, u0 + span, max(8, int(round(steps * span / circ)) + 1))
        else:
            us = np.linspace(0.0, circ, steps + 1)
        xs = [(max(x0 + 0.001, x_lo - slope * max(0.0, dist(u, anchors) - 0.35 * STRUT_M)) if low_end
               else min(x1 - 0.001, x_hi + slope * max(0.0, dist(u, anchors) - 0.35 * STRUT_M))) for u in us]
        for (u0, xa), (u1, xb) in zip(zip(us[:-1], xs[:-1]), zip(us[1:], xs[1:])):
            a0, a1, b0, b1 = (x0, x0, xa, xb) if low_end else (xa, xb, x1, x1)
            tris.extend(quad(at(u0, a0, outer), at(u1, a1, outer), at(u1, b1, outer), at(u0, b0, outer)))
            tris.extend(quad(at(u0, a0, inner), at(u0, b0, inner), at(u1, b1, inner), at(u1, a1, inner)))
            tris.extend(quad(at(u0, a0, inner), at(u1, a1, inner), at(u1, a1, outer), at(u0, a0, outer)))
            tris.extend(quad(at(u0, b0, inner), at(u0, b0, outer), at(u1, b1, outer), at(u1, b1, inner)))
        if g:
            u_s, u_e = us[0], us[-1]
            a_s, b_s = (x0, xs[0]) if low_end else (xs[0], x1)
            a_e, b_e = (x0, xs[-1]) if low_end else (xs[-1], x1)
            tris.extend(quad(at(u_s, a_s, inner), at(u_s, a_s, outer), at(u_s, b_s, outer), at(u_s, b_s, inner)))
            tris.extend(quad(at(u_e, a_e, inner), at(u_e, b_e, inner), at(u_e, b_e, outer), at(u_e, a_e, outer)))

    rim_strip(open_[0], bot_u, True)
    rim_strip(open_[1], top_u, False)
    return tris


def slotted(inner, outer, x0, x1, slots=(), colour=em.FRAME, k=192):
    """A solid cage wall round x between `inner` and `outer` from `x0` to `x1`, cut by `slots` [(bearing rad, a, b,
    width m)]: each slot open where `a` reaches `x0` or `b` reaches `x1`, and closed with a 45-degree gable at its
    inner end so nothing bridges along x either way."""
    import numpy as np
    if not slots:
        return m._band_x(inner, outer, x0, x1, colour, k=k)
    parsed = sorted((b % (2.0 * math.pi), s0, s1, math.asin(min(1.0, 0.5 * w / inner))) for b, s0, s1, w in slots)
    mids = [b for b, *_r in parsed]
    gaps = [((mids[(i + 1) % len(mids)] - mids[i]) % (2.0 * math.pi), mids[i]) for i in range(len(mids))]
    span, after = max(gaps)
    origin = (after + 0.5 * span) % (2.0 * math.pi)
    cuts = sorted(((b - origin) % (2.0 * math.pi), s0, s1, h) for b, s0, s1, h in parsed)
    pts = set(np.linspace(0.0, 2.0 * math.pi, k + 1).tolist())
    for c, _s0, _s1, h in cuts:
        pts.update(np.linspace(c - h, c, 5).tolist() + np.linspace(c, c + h, 5).tolist())
    turns = sorted(pts)

    def pt(x, r, u):
        a = origin + u
        return np.array([x, r * math.cos(a), r * math.sin(a)])

    def quad(p0, p1, p2, p3):
        return [(np.array((p0, p1, p2)), colour), (np.array((p0, p2, p3)), colour)]
    tris = []
    for u0, u1 in zip(turns[:-1], turns[1:]):
        um = 0.5 * (u0 + u1)
        hit = next(((c, s0, s1, h) for c, s0, s1, h in cuts if c - h - 1e-12 <= um <= c + h + 1e-12), None)
        spans = [(x0, x0, x1, x1)]
        if hit is not None:
            c, s0, s1, h = hit
            rise = outer * h
            lo0, lo1 = (max(x0, s0 - rise * (1.0 - min(abs(u - c) / h, 1.0))) for u in (u0, u1))
            hi0, hi1 = (min(x1, s1 + rise * (1.0 - min(abs(u - c) / h, 1.0))) for u in (u0, u1))
            spans = ([(x0, x0, lo0, lo1)] if s0 > x0 + 1e-9 else []) + ([(hi0, hi1, x1, x1)] if s1 < x1 - 1e-9 else [])
        for a0, a1, b0, b1 in spans:
            tris += quad(pt(a0, outer, u0), pt(a1, outer, u1), pt(b1, outer, u1), pt(b0, outer, u0))
            tris += quad(pt(a0, inner, u0), pt(b0, inner, u0), pt(b1, inner, u1), pt(a1, inner, u1))
            tris += quad(pt(a0, inner, u0), pt(a1, inner, u1), pt(a1, outer, u1), pt(a0, outer, u0))
            tris += quad(pt(b0, inner, u0), pt(b0, outer, u0), pt(b1, outer, u1), pt(b1, inner, u1))
    for c, s0, s1, h in cuts:
        xa, xb = (s0 if s0 > x0 + 1e-9 else x0), (s1 if s1 < x1 - 1e-9 else x1)
        tris += quad(pt(xa, inner, c - h), pt(xb, inner, c - h), pt(xb, outer, c - h), pt(xa, outer, c - h))
        tris += quad(pt(xa, inner, c + h), pt(xa, outer, c + h), pt(xb, outer, c + h), pt(xb, inner, c + h))
    return tris


def box(xs, zs):
    """A rectangle's outline in (x, z)."""
    import numpy as np
    return np.array([(xs[0], zs[0]), (xs[1], zs[0]), (xs[1], zs[1]), (xs[0], zs[1])])


def cupped(centre, r, y, zs, below=True):
    """The outline in (y, z) of a block from the level `y` to a circle round `centre` (y, z) of radius `r`, hugging
    it from below (or above) over z in `zs`, straight up its side (or down) past it."""
    import numpy as np
    way = -1.0 if below else 1.0
    arc = []
    for z in np.linspace(zs[1], zs[0], 33):
        d = z - centre[1]
        arc.append((centre[0] + way * math.sqrt(r * r - d * d) if abs(d) < r else centre[0], z))
    return np.array([(y, zs[0]), (y, zs[1])] + arc)


def necked(centre, r, y, zs, depth):
    """The outline in (y, z) of a block from its seat's edge - the level `y` over z in `zs` - tapering to a neck
    `depth` across where it meets a circle round `centre` (y, z) of radius `r`, aimed at the circle's middle, and
    hugging the circle between: a saddle's waist on its drum, or a foot's under it."""
    import numpy as np
    way = math.atan2(0.5 * (zs[0] + zs[1]) - centre[1], y - centre[0])          # toward the seat's middle
    half = math.asin(min(0.5 * depth / r, 1.0))
    arc = [(centre[0] + r * math.cos(t), centre[1] + r * math.sin(t)) for t in np.linspace(way - half, way + half, 17)]
    arc = arc if arc[0][1] >= arc[-1][1] else arc[::-1]                        # from the seat's far side back
    return np.array([(y, zs[0]), (y, zs[1])] + arc)


def bridged(a, ra, b, rb, depth):
    """The outline in (y, z) of a web `depth` across between two circles - round `a` (y, z) of radius `ra` and round
    `b` of radius `rb` - hugging each where it meets it."""
    import numpy as np
    out = []
    for c, r, o in ((a, ra, b), (b, rb, a)):
        way, half = math.atan2(o[1] - c[1], o[0] - c[0]), math.asin(min(0.5 * depth / r, 1.0))
        out += [(c[0] + r * math.cos(t), c[1] + r * math.sin(t)) for t in np.linspace(way - half, way + half, 17)]
    return np.array(out)


def oval(a, b, centre=(0.0, 0.0), n=96):
    """A seat's or a yoke's outline: the flanges' rounded oval (`elbow_mount.POWER`), half-widths `a` and `b`."""
    import numpy as np
    t = np.arange(n) * 2.0 * math.pi / n
    c, s = np.cos(t), np.sin(t)
    p = 2.0 / em.POWER
    return np.c_[centre[0] + np.sign(c) * np.abs(c) ** p * a, centre[1] + np.sign(s) * np.abs(s) ** p * b]


def tilted(tris, angle, centre):
    """`tris` turned `angle` rad about x round `centre` (y, z), +y toward +z."""
    import numpy as np
    c, s = math.cos(angle), math.sin(angle)
    turn, pivot = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]), np.array([0.0, centre[0], centre[1]])
    return [((tri - pivot) @ turn.T + pivot, col) for tri, col in tris]


def filleted(outline, r=FILLET_M, n=6, straight=165.0):
    """`outline` (2-d, in order round) with each corner sharper than `straight` degrees rounded `r` round, tangent to
    both its edges, a round taking at most half of each edge: an upright edge in print, so as printable rounded."""
    import numpy as np
    pts = np.asarray(outline, float)
    out = []
    for i in range(len(pts)):
        p, a, b = pts[i], pts[i - 1], pts[(i + 1) % len(pts)]
        ua, ub = a - p, b - p
        la, lb = float(np.linalg.norm(ua)), float(np.linalg.norm(ub))
        if la < 1e-9 or lb < 1e-9:
            continue
        ua, ub = ua / la, ub / lb
        angle = math.acos(max(-1.0, min(1.0, float(ua @ ub))))
        if math.degrees(angle) > straight:
            out.append(p)
            continue
        half = 0.5 * angle
        d = min(r / math.tan(half), 0.5 * la, 0.5 * lb)
        rr = d * math.tan(half)
        t1, t2 = p + ua * d, p + ub * d
        c = p + (ua + ub) / float(np.linalg.norm(ua + ub)) * (rr / math.sin(half))
        a1 = math.atan2(t1[1] - c[1], t1[0] - c[0])
        sweep = (math.atan2(t2[1] - c[1], t2[0] - c[0]) - a1 + math.pi) % (2.0 * math.pi) - math.pi
        out += [c + rr * np.array([math.cos(a1 + s * sweep), math.sin(a1 + s * sweep)])
                for s in np.linspace(0.0, 1.0, n + 1)]
    return np.array([p for i, p in enumerate(out) if float(np.linalg.norm(p - out[i - 1])) > 1e-9])  # rounds met
