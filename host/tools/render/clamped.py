"""The printed clamps' shapes: turned slit bands, honeycomb cages, blocks cupped round a drum.

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
    x, a cone between two that don't - its ends chamfered EDGE_M at 45 degrees and only its slit at `slit` (y = r
    cos) showing, its pinch screws in its wall across the slit; whole round, a ring, with `slit` None; open
    `gap` rad about `slit` where told - a window, no slit."""
    import numpy as np
    (xa, ia, oa), (xb, ib, ob) = stations[0], stations[-1]
    laid_out = ([(xa, ia + EDGE_M, oa - EDGE_M), (xa + EDGE_M, ia, oa)] + list(stations[1:-1])
                + [(xb - EDGE_M, ib, ob), (xb, ib + EDGE_M, ob - EDGE_M)])
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
    """A cage's wall round x between radii from x0 to x1: a solid rim at each end, and between them a honeycomb of
    cells with upright sides and 45-degree roofs - printable along x - the air runs through to what it holds; open
    over `gap` (rad, rad, from +y toward +z) where a window is - or over `gaps`, a window an end (at x0's, at
    x1's; None for none): the cells open over both, each rim over its own end's."""
    import numpy as np
    open_ = gaps or (gap, gap)
    tris = (m._band_x(inner, outer, x0, x0 + RIM_M, colour, k=96, gap=open_[0])
            + m._band_x(inner, outer, x1 - RIM_M, x1, colour, k=96, gap=open_[1]))
    mid = 0.5 * (inner + outer)
    n = max(6, int(round(2.0 * math.pi * mid / CELL_M)))
    a, side = 2.0 * math.pi * mid / n, 0.5 * SIDE_M      # a cell's width round, the wall shared out in whole cells
    pitch = SIDE_M + 0.5 * a                             # a row's height: its sides and one roof, the rows interlocked
    lo, hi = x0 + RIM_M - 0.0005, x1 - RIM_M + 0.0005

    def at(u, x, r):
        return (x, r * math.cos(u / mid), r * math.sin(u / mid))
    for j in range(-1, int(math.ceil((x1 - x0) / pitch)) + 1):
        xc = x0 + RIM_M + j * pitch
        for i in range(n):
            uc = (i + 0.5 * (j % 2)) * a
            v = [(uc, xc + side + 0.5 * a), (uc + 0.5 * a, xc + side), (uc + 0.5 * a, xc - side),
                 (uc, xc - side - 0.5 * a), (uc - 0.5 * a, xc - side), (uc - 0.5 * a, xc + side)]
            for (ua, xa), (ub, xb) in zip(v, v[1:] + v[:1]):
                t0, t1 = 0.0, 1.0                        # the strut clipped to between the rims, into them
                if xb != xa:
                    ta, tb = sorted(((lo - xa) / (xb - xa), (hi - xa) / (xb - xa)))
                    t0, t1 = max(t0, ta), min(t1, tb)
                elif not lo <= xa <= hi:
                    continue
                if t1 <= t0:
                    continue
                if any(g and g[0] < (0.5 * (ua + ub) / mid) % (2.0 * math.pi) < g[1] for g in open_):
                    continue
                p = np.array([ua + t0 * (ub - ua), xa + t0 * (xb - xa)])
                q = np.array([ua + t1 * (ub - ua), xa + t1 * (xb - xa)])
                d = (q - p) / max(float(np.linalg.norm(q - p)), 1e-9)
                w = np.array([-d[1], d[0]]) * 0.5 * STRUT_M
                quad = [p + w, q + w, q - w, p - w]
                ring_i = [np.array(at(u, x, inner)) for u, x in quad]
                ring_o = [np.array(at(u, x, outer)) for u, x in quad]
                for s in range(4):
                    e = (s + 1) % 4
                    tris += [(np.array((ring_i[s], ring_i[e], ring_o[e])), colour),
                             (np.array((ring_i[s], ring_o[e], ring_o[s])), colour)]
                tris += [(np.array((ring_o[0], ring_o[1], ring_o[2])), colour),
                         (np.array((ring_o[0], ring_o[2], ring_o[3])), colour)]
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
