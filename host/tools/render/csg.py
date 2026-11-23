"""Parametric CSG solid frontend: compiles composable geometry trees to manifold meshes and OpenSCAD.

    from tools.render import csg
    part = csg.extrude(outer, holes=(bore,), a=0.0, b=0.012, axis=1)
    tris = part.tris()
    code = csg.scad(part)
    csg.stl(tris, 'part.stl')
"""
import math

DEFAULT_COLOUR = (112, 116, 126)


def _signed_area(pts):
    """Signed 2D area of closed polygon `pts` (CCW positive)."""
    import numpy as np
    p = np.asarray(pts, float)
    return 0.5 * float(np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1]))


def clean2d(pts, eps=1e-7):
    """Remove duplicate and nearly collinear vertices from 2D polygon `pts`."""
    import numpy as np
    p = np.asarray(pts, float)
    keep = [p[0]]
    for pt in p[1:]:
        if float(np.linalg.norm(pt - keep[-1])) > eps:
            keep.append(pt)
    if len(keep) > 2 and float(np.linalg.norm(keep[0] - keep[-1])) <= eps:
        keep.pop()
    p = np.asarray(keep, float)
    out = []
    n = len(p)
    for i in range(n):
        u, v = p[i] - p[i - 1], p[(i + 1) % n] - p[i]
        lu, lv = float(np.linalg.norm(u)), float(np.linalg.norm(v))
        if lu > eps and lv > eps and abs(float(u[0] * v[1] - u[1] * v[0])) > 1e-11 * lu * lv:
            out.append(p[i])
    return np.asarray(out, float) if len(out) >= 3 else p


def oriented(pts, ccw=True, clean=True):
    """Return 2D loop wound CCW (`ccw=True`) or CW (`ccw=False`), removing collinear vertices when `clean`."""
    import numpy as np
    p = clean2d(pts) if clean else np.asarray(pts, float)
    return p if (_signed_area(p) >= 0.0) == ccw else p[::-1].copy()


def box2d(u, v):
    """CCW 2D rectangle over `u = (u0, u1)` and `v = (v0, v1)`."""
    import numpy as np
    return np.array([(u[0], v[0]), (u[1], v[0]), (u[1], v[1]), (u[0], v[1])], float)


def circle2d(r, fn=48, at=(0.0, 0.0)):
    """CCW 2D circle of radius `r` centred at `at`."""
    import numpy as np
    t = np.arange(fn) * 2.0 * math.pi / fn
    return np.c_[at[0] + r * np.cos(t), at[1] + r * np.sin(t)]


def teardrop2d(r, up=1.0, fn=48, at=(0.0, 0.0)):
    """CCW 2D teardrop: circle `r` with a circumscribed 45-deg roof apex along `up` (+1 or -1 on coord 1)."""
    import numpy as np
    s = 1.0 if up >= 0.0 else -1.0
    k = max(12, (3 * fn) // 4)
    angs = np.linspace(0.75 * math.pi, 2.25 * math.pi, k) if s > 0.0 else np.linspace(-0.25 * math.pi, 1.25 * math.pi, k)
    pts = [(at[0] + r * math.cos(a), at[1] + r * math.sin(a)) for a in angs]
    pts.append((at[0], at[1] + s * r * math.sqrt(2.0)))
    return oriented(pts, ccw=True)


def filleted2d(outline, r=0.002, n=6, straight=168.0):
    """Round corners of 2D `outline` sharper than `straight` deg using radius `r`."""
    from tools.render import clamped as cl
    pts = clean2d(outline)
    return cl.filleted(pts, r=r, n=n, straight=straight) if len(pts) >= 3 else pts


def _segments_cross(a, b, c, d, eps=1e-9):
    """True when open segments (a, b) and (c, d) intersect properly or pass near an endpoint."""
    def cr(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
    d1, d2 = cr(c, d, a), cr(c, d, b)
    d3, d4 = cr(a, b, c), cr(a, b, d)
    if (d1 * d2 < -eps) and (d3 * d4 < -eps):
        return True
    for pt, s, e in ((a, c, d), (b, c, d), (c, a, b), (d, a, b)):
        seg = e - s
        l2 = float(seg @ seg)
        if l2 > eps and 0.0 <= float((pt - s) @ seg) <= l2 and abs(cr(s, e, pt)) / math.sqrt(l2) < 1e-8:
            return True
    return False


def _visible(m, poly, all_loops, forbid):
    """Find vertex index in `poly` visible from hole vertex `m` without crossing any edge of `all_loops`."""
    import numpy as np
    order = sorted(range(len(poly)), key=lambda i: float(np.linalg.norm(poly[i] - m)))
    for i in order:
        if i in forbid:
            continue
        v = poly[i]
        d = v - m
        if float(d @ d) < 1e-14:
            continue
        blocked = False
        for loop in all_loops:
            n = len(loop)
            for k in range(n):
                p, q = loop[k], loop[(k + 1) % n]
                if (float(np.linalg.norm(p - v)) < 1e-9 or float(np.linalg.norm(q - v)) < 1e-9
                        or float(np.linalg.norm(p - m)) < 1e-9 or float(np.linalg.norm(q - m)) < 1e-9):
                    continue
                if _segments_cross(m, v, p, q):
                    blocked = True
                    break
            if blocked:
                break
        if not blocked:
            return i
    return order[0]


def bridge_holes(outer, holes=(), clean=True):
    """Bridge CW `holes` into CCW `outer` via mutually visible cut edges, returning `(simple_2d, tri_idx)`."""
    import numpy as np
    from tools.render import foot
    poly = oriented(outer, ccw=True, clean=clean)
    if not holes:
        return poly, foot._ears(poly)
    cw_holes = sorted((oriented(h, ccw=False, clean=clean) for h in holes), key=lambda h: -float(h[:, 0].max()))
    forbid = set()
    for idx_h, hole in enumerate(cw_holes):
        m_i = int(np.argmax(hole[:, 0]))
        m = hole[m_i]
        all_loops = [poly] + cw_holes[idx_h:]
        v_i = _visible(m, poly, all_loops, forbid)
        rolled = np.roll(hole, -m_i, axis=0)
        v_pt = poly[v_i]
        poly = np.vstack([poly[:v_i + 1], rolled, [rolled[0], v_pt], poly[v_i + 1:]])
        forbid = set()
    return poly, foot._ears(poly)


def _lay2d(pts, h, axis):
    """Map 2D coordinates `pts` onto plane `axis = h` preserving cyclic right-handed orientation `(i0, i1, axis)`."""
    import numpy as np
    p = np.asarray(pts, float)
    hs = np.full(len(p), float(h))
    if axis == 0:
        return np.c_[hs, p[:, 0], p[:, 1]]
    if axis == 1:
        return np.c_[p[:, 1], hs, p[:, 0]]
    return np.c_[p[:, 0], p[:, 1], hs]


def _scad_num(x):
    """Format float in mm cleanly for OpenSCAD output."""
    return ('%.4f' % float(x)).rstrip('0').rstrip('.') or '0'


def _scad_pts(pts, scale=1e3):
    return '[' + ', '.join('[%s, %s]' % (_scad_num(scale * p[0]), _scad_num(scale * p[1])) for p in pts) + ']'


def _indent(txt, spaces=2):
    pad = ' ' * spaces
    return '\n'.join(pad + ln if ln else ln for ln in txt.splitlines())


class Node:
    """Composable CSG tree node holding watertight triangles (m) and compiling to OpenSCAD text."""

    def __init__(self, tris=(), scad_fn=None, colour=DEFAULT_COLOUR):
        self._tris = list(tris)
        self._scad_fn = scad_fn
        self.colour = colour

    def tris(self):
        """Return `[(np.ndarray (3, 3), colour)]` in meters."""
        return list(self._tris)

    def to_scad(self):
        """Return OpenSCAD code string in millimetres."""
        return self._scad_fn() if self._scad_fn else _polyhedron_scad(self._tris)

    def __add__(self, other):
        return union(self, other)

    def __sub__(self, other):
        return Node(self._tris, lambda: 'difference() {\n%s\n%s\n}' % (
            _indent(self.to_scad()), _indent(other.to_scad())), self.colour)

    def translate(self, dx=0.0, dy=0.0, dz=0.0):
        """Shift solid by `(dx, dy, dz)` m."""
        import numpy as np
        off = np.array((dx, dy, dz), float)
        moved = [(t + off, c) for t, c in self._tris]
        v = ', '.join(_scad_num(1e3 * k) for k in off)
        return Node(moved, lambda: 'translate([%s]) {\n%s\n}' % (v, _indent(self.to_scad())), self.colour)

    def transform(self, mat, off=(0.0, 0.0, 0.0), det_sign=1.0):
        """Apply 3x3 row-vector matrix `mat` plus `off` m (`det_sign < 0` reverses winding)."""
        import numpy as np
        m, s = np.asarray(mat, float), np.asarray(off, float)
        if det_sign >= 0.0:
            pts = [(t @ m + s, c) for t, c in self._tris]
        else:
            pts = [((t @ m + s)[::-1], c) for t, c in self._tris]
        return Node(pts, lambda: _polyhedron_scad(pts), self.colour)

    def painted(self, colour):
        """Return copy recoloured with RGB tuple `colour`."""
        return Node([(t, colour) for t, _c in self._tris], self._scad_fn, colour)


def union(*nodes):
    """Combine multiple `Node` solids into one CSG union node."""
    tris = [t for n in nodes for t in (n.tris() if isinstance(n, Node) else n)]

    def _scad():
        body = '\n'.join(_indent(n.to_scad()) for n in nodes if isinstance(n, Node))
        return 'union() {\n%s\n}' % body
    return Node(tris, _scad)


def _polyhedron_scad(tris, scale=1e3):
    """Compile arbitrary closed mesh `tris` into an OpenSCAD `polyhedron(points=..., faces=...)` in mm."""
    import numpy as np
    if not tris:
        return 'union() {}'
    arr = scale * np.array([t for t, _c in tris], float)
    pts, inv = np.unique(np.round(arr.reshape(-1, 3), 5), axis=0, return_inverse=True)
    faces = inv.reshape(-1, 3)
    p_str = ', '.join('[%s, %s, %s]' % (_scad_num(x), _scad_num(y), _scad_num(z)) for x, y, z in pts)
    f_str = ', '.join('[%d, %d, %d]' % (int(a), int(b), int(c)) for a, b, c in faces)
    return 'polyhedron(points=[%s], faces=[%s]);' % (p_str, f_str)


def _axis_scad(body, a, b, axis):
    """Orient an OpenSCAD z-extrusion onto coordinate `axis` from `a` to `b` m."""
    lo, hi = min(a, b), max(a, b)
    ext = 'linear_extrude(height=%s) {\n%s\n}' % (_scad_num(1e3 * (hi - lo)), _indent(body))
    if axis == 0:
        return 'translate([%s, 0, 0]) rotate([0, 90, 0]) rotate([0, 0, 90]) {\n%s\n}' % (
            _scad_num(1e3 * lo), _indent(ext))
    if axis == 1:
        return 'translate([0, %s, 0]) rotate([-90, 0, 0]) rotate([0, 0, -90]) {\n%s\n}' % (
            _scad_num(1e3 * lo), _indent(ext))
    return 'translate([0, 0, %s]) {\n%s\n}' % (_scad_num(1e3 * lo), _indent(ext))


def extrude(outer, holes=(), a=0.0, b=1.0, axis=1, colour=DEFAULT_COLOUR):
    """Watertight 2-manifold extrusion of 2D `outer` minus `holes` along `axis` from `a` to `b` m."""
    import numpy as np
    lo, hi = min(a, b), max(a, b)
    ccw_out = oriented(outer, ccw=True)
    cw_holes = [oriented(h, ccw=False) for h in holes]
    poly, ears = bridge_holes(ccw_out, cw_holes)
    cap_lo, cap_hi = _lay2d(poly, lo, axis), _lay2d(poly, hi, axis)
    tris = []
    for i, j, k in ears:
        tris.append((np.array((cap_lo[i], cap_lo[k], cap_lo[j])), colour))
        tris.append((np.array((cap_hi[i], cap_hi[j], cap_hi[k])), colour))
    for loop in [ccw_out] + cw_holes:
        p0, p1 = _lay2d(loop, lo, axis), _lay2d(loop, hi, axis)
        n = len(loop)
        for i in range(n):
            j = (i + 1) % n
            tris.append((np.array((p0[i], p0[j], p1[j])), colour))
            tris.append((np.array((p0[i], p1[j], p1[i])), colour))

    def _scad():
        out_p = 'polygon(%s);' % _scad_pts(ccw_out)
        if not cw_holes:
            return _axis_scad(out_p, lo, hi, axis)
        h_s = '\n'.join(_indent('polygon(%s);' % _scad_pts(h[::-1])) for h in cw_holes)
        body = 'difference() {\n%s\n%s\n}' % (_indent(out_p), h_s)
        return _axis_scad(body, lo, hi, axis)
    return Node(tris, _scad, colour)


def loft(rings, holes=(), colour=DEFAULT_COLOUR):
    """Watertight closed 2-manifold loft through 3D rings `(m, k, 3)` with matching-vertex `holes` list."""
    import numpy as np
    r_arr = [np.asarray(r, float) for r in rings]
    n = len(r_arr[0])
    tris = []
    for a, b in zip(r_arr[:-1], r_arr[1:]):
        for i in range(n):
            j = (i + 1) % n
            tris.append((np.array((a[i], a[j], b[j])), colour))
            tris.append((np.array((a[i], b[j], b[i])), colour))
    h_arr = [[np.asarray(r, float) for r in hr] for hr in holes]
    for hr in h_arr:
        nh = len(hr[0])
        for a, b in zip(hr[:-1], hr[1:]):
            for i in range(nh):
                j = (i + 1) % nh
                tris.append((np.array((a[i], b[j], a[j])), colour))
                tris.append((np.array((a[i], b[i], b[j])), colour))
    for end_idx, flip in ((0, True), (-1, False)):
        ring = r_arr[end_idx]
        u = ring[1] - ring[0]
        u = u / max(float(np.linalg.norm(u)), 1e-12)
        normals = [np.cross(u, pt - ring[0]) for pt in ring[2:]]
        normal = max(normals, key=lambda v: float(v @ v))
        nrm = normal / max(float(np.linalg.norm(normal)), 1e-12)
        v = np.cross(nrm, u)
        o2d = np.c_[(ring - ring[0]) @ u, (ring - ring[0]) @ v]
        h2d = [np.c_[(hr[end_idx] - ring[0]) @ u, (hr[end_idx] - ring[0]) @ v] for hr in h_arr]
        all3d = np.vstack([ring] + [hr[end_idx] for hr in h_arr])
        o_clean = oriented(o2d, ccw=True)
        h_clean = [oriented(h, ccw=False) for h in h2d]
        poly, ears = bridge_holes(o_clean, h_clean)
        all2d = np.vstack([o2d] + h2d)
        idx_map = [int(np.argmin(np.linalg.norm(all2d - pt, axis=1))) for pt in poly]
        for i, j, k in ears:
            pts = all3d[[idx_map[i], idx_map[k], idx_map[j]]] if flip else all3d[[idx_map[i], idx_map[j], idx_map[k]]]
            tris.append((pts, colour))
    return Node(tris, lambda: _polyhedron_scad(tris), colour)


def revolve(stations, slit=None, fn=64, gap=None, colour=DEFAULT_COLOUR):
    """Watertight revolved sleeve/collar `[(x, bore, out)]` along x with optional clamp slit `slit` rad."""
    from tools.render import clamped as cl
    tris = cl.turned(stations, slit, colour=colour, k=fn, gap=gap)
    return Node(tris, lambda: _polyhedron_scad(tris), colour)


def hull(points, colour=DEFAULT_COLOUR):
    """Convex hull solid with outward-wound triangles and native OpenSCAD `hull()` output."""
    import numpy as np
    from scipy.spatial import ConvexHull
    pts = np.asarray(points, float)
    mid = pts.mean(axis=0)
    tris = []
    for face in ConvexHull(pts).simplices:
        tri = pts[face]
        w = np.cross(tri[1] - tri[0], tri[2] - tri[0]) @ (tri[0] - mid)
        tris.append((tri if w >= 0.0 else tri[::-1].copy(), colour))
    return Node(tris, lambda: _polyhedron_scad(tris), colour)


def scad(node, fn=64):
    """Compile `Node` (or raw `tris`) into an OpenSCAD `.scad` program string in millimetres."""
    body = node.to_scad() if isinstance(node, Node) else _polyhedron_scad(node)
    return '$fn = %d;\n\n%s\n' % (fn, body)


def write_scad(node, path, fn=64):
    """Write compiled OpenSCAD `.scad` program for `node` to `path`."""
    text = scad(node, fn=fn)
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    return path


def stl(tris, path, scale=1e3):
    """Export closed triangle mesh `tris` (scaled to mm by `scale`) as binary STL."""
    import numpy as np
    arr = scale * np.array([t for t, _c in tris], float)
    nrm = np.cross(arr[:, 1] - arr[:, 0], arr[:, 2] - arr[:, 0])
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-30
    rec = np.zeros(len(arr), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['n'], rec['v'] = nrm, arr
    with open(path, 'wb') as f:
        f.write(b'coaxial_csg'.ljust(80, b' '))
        f.write(np.uint32(len(arr)).tobytes())
        f.write(rec.tobytes())
    return path
