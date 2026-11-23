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


def inset2d(pts, amount, outer=True):
    """Return closed 2D loop `pts` with equal vertex count offset into solid by `amount` m (`outer` vs hole)."""
    import numpy as np
    p = np.asarray(pts, float)
    if abs(float(amount)) <= 1e-9 or len(p) < 3:
        return p.copy()
    e_prev = p - np.roll(p, 1, axis=0)
    e_next = np.roll(p, -1, axis=0) - p
    l_prev = np.linalg.norm(e_prev, axis=1)
    l_next = np.linalg.norm(e_next, axis=1)
    t1 = e_prev / np.maximum(l_prev[:, None], 1e-12)
    t2 = e_next / np.maximum(l_next[:, None], 1e-12)
    ccw_poly = _signed_area(p) >= 0.0
    # Left normal (-ty, tx) points inward for a CCW loop, outward for a CW loop.
    sgn = (1.0 if ccw_poly else -1.0) * (1.0 if outer else -1.0)
    n1 = sgn * np.c_[-t1[:, 1], t1[:, 0]]
    n2 = sgn * np.c_[-t2[:, 1], t2[:, 0]]
    m = n1 + n2
    m_len = np.linalg.norm(m, axis=1)
    safe = np.where(m_len[:, None] > 1e-9, m / np.maximum(m_len[:, None], 1e-12), n1)
    cos_half = np.maximum(np.sum(safe * n1, axis=1), 0.40)
    disp = (float(amount) / cos_half)[:, None] * safe
    edge = np.roll(p, -1, axis=0) - p
    d_edge = np.roll(disp, -1, axis=0) - disp
    e2 = np.sum(edge * edge, axis=1)
    de = np.sum(edge * d_edge, axis=1)
    bad = de < -1e-12
    alpha = float(np.min(0.80 * e2[bad] / (-de[bad]))) if np.any(bad) else 1.0
    out = p + min(1.0, alpha) * disp
    while _signed_area(out) * _signed_area(p) <= 0.0 and alpha > 1e-3:
        alpha *= 0.5
        out = p + alpha * disp
    return out


def offset2d(pts, delta):
    """Return cleaned closed 2D polygon `pts` expanded outward by `delta` m (negative shrinks inward)."""
    return clean2d(inset2d(pts, -float(delta), outer=True))


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


def _quads(a, b, colour, flip=False):
    """Side quad strip connecting matching closed 3D loops `a` and `b`."""
    import numpy as np
    n = len(a)
    if flip:
        return [t for i in range(n) for j in ((i + 1) % n,)
                for t in ((np.array((a[i], b[j], a[j])), colour), (np.array((a[i], b[i], b[j])), colour))]
    return [t for i in range(n) for j in ((i + 1) % n,)
            for t in ((np.array((a[i], a[j], b[j])), colour), (np.array((a[i], b[j], b[i])), colour))]


def _axis_turn(body, axis):
    """Orient Z-aligned OpenSCAD block onto `axis`."""
    if axis == 0:
        return 'rotate([0, 90, 0]) rotate([0, 0, 90]) {\n%s\n}' % _indent(body)
    if axis == 1:
        return 'rotate([-90, 0, 0]) rotate([0, 0, -90]) {\n%s\n}' % _indent(body)
    return body


def _axis_scad(body, a, b, axis):
    """Orient Z-extruded OpenSCAD block onto `axis` over `[a, b]` m."""
    lo, hi = min(a, b), max(a, b)
    ext = 'linear_extrude(height=%s) {\n%s\n}' % (_scad_num(1e3 * (hi - lo)), _indent(body))
    if axis < 2:
        t = '[%s, 0, 0]' if axis == 0 else '[0, %s, 0]'
        return ('translate(' + t + ') %s') % (_scad_num(1e3 * lo), _axis_turn(ext, axis))
    return 'translate([0, 0, %s]) {\n%s\n}' % (_scad_num(1e3 * lo), _indent(ext))


def extrude(outer, holes=(), a=0.0, b=1.0, axis=1, colour=DEFAULT_COLOUR,
            chamfer=0.0, chamfer_lo=None, chamfer_hi=None, slope=1.30):
    """Watertight 2-manifold extrusion of `outer` minus `holes` along `axis` with optional end chamfers."""
    import numpy as np
    lo, hi = min(a, b), max(a, b)
    c_lo = float(chamfer if chamfer_lo is None else chamfer_lo)
    c_hi = float(chamfer if chamfer_hi is None else chamfer_hi)
    ccw_out, cw_holes = oriented(outer, ccw=True), [oriented(h, ccw=False) for h in holes]
    if c_lo > 1e-6 or c_hi > 1e-6:
        tot = slope * (c_lo + c_hi)
        s = min(1.0, 0.7 * max(hi - lo, 1e-6) / tot) if tot > 1e-9 else 1.0
        c_lo, c_hi = s * c_lo, s * c_hi
        stops = [(lo, c_lo)] + ([(lo + slope * c_lo, 0.0)] if c_lo > 1e-6 else [])
        if c_hi > 1e-6 and hi - slope * c_hi > stops[-1][0] + 1e-6:
            stops.append((hi - slope * c_hi, 0.0))
        stops.append((hi, c_hi))
        return loft([_lay2d(inset2d(ccw_out, c, True), z, axis) for z, c in stops],
                    holes=[[_lay2d(inset2d(h, c, False), z, axis) for z, c in stops] for h in cw_holes], colour=colour)
    poly, ears = bridge_holes(ccw_out, cw_holes)
    cap_lo, cap_hi = _lay2d(poly, lo, axis), _lay2d(poly, hi, axis)
    tris = [t for i, j, k in ears for t in (
        (np.array((cap_lo[i], cap_lo[k], cap_lo[j])), colour), (np.array((cap_hi[i], cap_hi[j], cap_hi[k])), colour))]
    for loop in [ccw_out] + cw_holes:
        tris += _quads(_lay2d(loop, lo, axis), _lay2d(loop, hi, axis), colour)

    def _scad():
        out_p = 'polygon(%s);' % _scad_pts(ccw_out)
        if not cw_holes:
            return _axis_scad(out_p, lo, hi, axis)
        h_s = '\n'.join(_indent('polygon(%s);' % _scad_pts(h[::-1])) for h in cw_holes)
        return _axis_scad('difference() {\n%s\n%s\n}' % (_indent(out_p), h_s), lo, hi, axis)
    return Node(tris, _scad, colour)


def loft(rings, holes=(), colour=DEFAULT_COLOUR):
    """Watertight closed 2-manifold loft through 3D rings `(m, k, 3)` with matching-vertex `holes` list."""
    import numpy as np
    r_arr = [np.asarray(r, float) for r in rings]
    h_arr = [[np.asarray(r, float) for r in hr] for hr in holes]
    tris = [t for a, b in zip(r_arr[:-1], r_arr[1:]) for t in _quads(a, b, colour)]
    tris += [t for hr in h_arr for a, b in zip(hr[:-1], hr[1:]) for t in _quads(a, b, colour, True)]
    for end_idx, flip in ((0, True), (-1, False)):
        ring = r_arr[end_idx]
        u = ring[1] - ring[0]
        u = u / max(float(np.linalg.norm(u)), 1e-12)
        nrm = max((np.cross(u, pt - ring[0]) for pt in ring[2:]), key=lambda w: float(w @ w))
        v = np.cross(nrm / max(float(np.linalg.norm(nrm)), 1e-12), u)
        o2d = np.c_[(ring - ring[0]) @ u, (ring - ring[0]) @ v]
        h2d = [np.c_[(hr[end_idx] - ring[0]) @ u, (hr[end_idx] - ring[0]) @ v] for hr in h_arr]
        all3d = np.vstack([ring] + [hr[end_idx] for hr in h_arr])
        poly, ears = bridge_holes(oriented(o2d, ccw=True), [oriented(h, ccw=False) for h in h2d])
        all2d = np.vstack([o2d] + h2d)
        idx = [int(np.argmin(np.linalg.norm(all2d - pt, axis=1))) for pt in poly]
        for i, j, k in ears:
            tris.append((all3d[[idx[i], idx[k], idx[j]] if flip else [idx[i], idx[j], idx[k]]], colour))
    return Node(tris, lambda: _polyhedron_scad(tris), colour)


def loft_profiles(profiles, zs, axis=2, samples=16, chamfer_lo=0.0, chamfer_hi=0.0, slope=1.30, colour=DEFAULT_COLOUR):
    """Smooth C1 cosine-interpolated loft between equal-count 2D `profiles` across ascending `zs`."""
    import numpy as np
    profs, z_vals = [np.asarray(p, float) for p in profiles], [float(z) for z in zs]
    rings, z_seq = [], []
    for k in range(len(profs) - 1):
        for t in np.linspace(0.0, 1.0, max(2, samples), endpoint=(k == len(profs) - 2)):
            e = 0.5 * (1.0 - math.cos(math.pi * float(t)))
            rings.append((1.0 - e) * profs[k] + e * profs[k + 1])
            z_seq.append((1.0 - float(t)) * z_vals[k] + float(t) * z_vals[k + 1])
    if chamfer_lo > 1e-6 and len(rings) >= 2:
        dz = min(slope * float(chamfer_lo), 0.45 * abs(z_seq[1] - z_seq[0]))
        rings.insert(0, inset2d(rings[0], dz / slope, True))
        z_seq = [z_seq[0], z_seq[0] + math.copysign(dz, z_seq[-1] - z_seq[0])] + z_seq[1:]
    if chamfer_hi > 1e-6 and len(rings) >= 2:
        dz = min(slope * float(chamfer_hi), 0.45 * abs(z_seq[-1] - z_seq[-2]))
        rings.append(inset2d(rings[-1], dz / slope, True))
        z_seq = z_seq[:-1] + [z_seq[-1] - math.copysign(dz, z_seq[-1] - z_seq[0]), z_seq[-1]]
    return loft([_lay2d(r, z, axis) for r, z in zip(rings, z_seq)], colour=colour)


def revolve_profile(profile_rz, angle=2.0 * math.pi, start=0.0, axis=2, at=(0.0, 0.0), fn=48,
                    end_chamfer=0.0, bed_z=None, slope=1.30, colour=DEFAULT_COLOUR):
    """Revolve closed `(r, z)` polygon round `axis` centred at `at`, full 360-deg or chamfered sector."""
    import numpy as np
    if angle < 0.0:
        start, angle = start + angle, -angle
    cw_rz, n_ang = oriented(profile_rz, ccw=False), max(6, int(fn))

    def ring_at(rz, ang):
        uv = np.c_[at[0] + rz[:, 0] * math.cos(ang), at[1] + rz[:, 0] * math.sin(ang)]
        return np.array([_lay2d(uv[k:k + 1], float(rz[k, 1]), axis)[0] for k in range(len(rz))])

    def _scad():
        re = 'rotate_extrude(angle=%s, $fn=%d) {\n  polygon(%s);\n}' % (
            _scad_num(math.degrees(float(angle))), n_ang, _scad_pts(cw_rz[::-1]))
        if abs(start) > 1e-6:
            re = 'rotate([0, 0, %s]) {\n%s\n}' % (_scad_num(math.degrees(start)), _indent(re))
        xy = _lay2d(np.array([at]), 0.0, axis)[0]
        return 'translate([%s, %s, %s]) {\n%s\n}' % (
            _scad_num(1e3 * xy[0]), _scad_num(1e3 * xy[1]), _scad_num(1e3 * xy[2]), _indent(_axis_turn(re, axis)))

    if angle >= 2.0 * math.pi - 1e-6:
        rings = [ring_at(cw_rz, float(a)) for a in np.linspace(start, start + 2.0 * math.pi, n_ang, endpoint=False)]
        return Node([t for k in range(n_ang) for t in _quads(rings[k], rings[(k + 1) % n_ang], colour)], _scad, colour)
    angs = list(np.linspace(start, start + angle, n_ang))
    r_pts = [cw_rz for _ in angs]
    if end_chamfer > 1e-6:
        da = min(0.25 * angle, slope * float(end_chamfer) / max(float(np.min(cw_rz[:, 0])), 0.012))
        cap_rz = inset2d(cw_rz, float(end_chamfer), True)
        if bed_z is not None:
            cap_rz[np.abs(cw_rz[:, 1] - float(bed_z)) <= 2e-5, 1] = float(bed_z)
        angs = [start, start + da] + [a for a in angs[1:-1] if start + da + 1e-5 < a < start + angle - da - 1e-5] + [
            start + angle - da, start + angle]
        r_pts = [cap_rz] + [cw_rz] * (len(angs) - 2) + [cap_rz]
    return Node(loft([ring_at(p, float(a)) for p, a in zip(r_pts, angs)], colour=colour).tris(), _scad, colour)


def sweep(profile2d, path3d, twist=0.0, scale=(1.0, 1.0), chamfer_ends=0.0, closed=False, colour=DEFAULT_COLOUR):
    """Sweep 2D `profile2d` along 3D `path3d` using parallel-transport Bishop frames with optional twist/taper."""
    import numpy as np
    p2, pts = oriented(profile2d, ccw=True), np.asarray(path3d, float)
    n = len(pts)
    tang = np.zeros_like(pts)
    tang[0] = pts[1] - pts[-1 if closed else 0]
    tang[-1] = pts[0 if closed else -1] - pts[-2]
    for k in range(1, n - 1):
        tang[k] = pts[k + 1] - pts[k - 1]
    tang /= np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-12)
    ref = np.array((0.0, 0.0, 1.0)) if abs(float(tang[0, 2])) < 0.9 else np.array((1.0, 0.0, 0.0))
    u = np.cross(ref, tang[0])
    us = [u / max(float(np.linalg.norm(u)), 1e-12)]
    for k in range(1, n):
        pr = us[-1] - float(us[-1] @ tang[k]) * tang[k]
        if float(pr @ pr) < 1e-10:
            pr = np.cross(np.array((1.0, 0.0, 0.0)) if abs(float(tang[k, 0])) < 0.9 else np.array((0.0, 1.0, 0.0)), tang[k])
        us.append(pr / max(float(np.linalg.norm(pr)), 1e-12))
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    tot = max(float(arc[-1]), 1e-9)

    def _ring(k, sp):
        t = float(arc[k] / tot)
        sc, ang = (1.0 - t) * float(scale[0]) + t * float(scale[1]), float(twist) * t
        uk, vk = us[k], np.cross(tang[k], us[k])
        ur, vr = math.cos(ang) * uk + math.sin(ang) * vk, -math.sin(ang) * uk + math.cos(ang) * vk
        return pts[k] + sc * (sp[:, 0:1] * ur + sp[:, 1:2] * vr)

    rings = [_ring(k, p2) for k in range(n)]
    if closed:
        tris = [t for k in range(n) for t in _quads(rings[k], rings[(k + 1) % n], colour)]
        return Node(tris, lambda: _polyhedron_scad(tris), colour)
    if chamfer_ends > 1e-6 and n >= 2:
        c = float(chamfer_ends)
        cap_p = inset2d(p2, c, True)
        for idx, nb, at_s in ((0, 1, True), (n - 1, n - 2, False)):
            d = pts[nb] - pts[idx]
            ln = float(np.linalg.norm(d))
            sh = rings[idx] + min(0.45 * ln, 1.30 * c) * (d / max(ln, 1e-12))
            rings = ([_ring(idx, cap_p), sh] + rings[1:]) if at_s else (rings[:-1] + [sh, _ring(idx, cap_p)])
    return loft(rings, colour=colour)


def helix(profile2d, radius, pitch, turns, r_end=None, start=0.0, axis=2, at=(0.0, 0.0),
          fn_turn=36, chamfer_ends=0.0, colour=DEFAULT_COLOUR):
    """Parametric helical or spiral sweep of 2D `profile2d` round `axis` at `at`."""
    import numpy as np
    r0, r1 = float(radius), float(radius if r_end is None else r_end)
    steps = max(12, int(round(abs(float(turns)) * int(fn_turn)))) + 1
    ts = np.linspace(0.0, 1.0, steps)
    angs, rs, zs = float(start) + 2.0 * math.pi * float(turns) * ts, (1.0 - ts) * r0 + ts * r1, float(pitch) * float(turns) * ts
    uv = np.c_[at[0] + rs * np.cos(angs), at[1] + rs * np.sin(angs)]
    return sweep(profile2d, np.array([_lay2d(uv[k:k + 1], float(zs[k]), axis)[0] for k in range(steps)]),
                 chamfer_ends=chamfer_ends, colour=colour)


def revolve(stations, slit=None, fn=64, gap=None, colour=DEFAULT_COLOUR):
    """Watertight revolved sleeve/collar `[(x, bore, out)]` along x with optional clamp slit `slit` rad."""
    from tools.render import clamped as cl
    tris = cl.turned(stations, slit, colour=colour, k=fn, gap=gap)
    return Node(tris, lambda: _polyhedron_scad(tris), colour)


def hull(points, colour=DEFAULT_COLOUR):
    """Convex hull solid with outward-wound triangles."""
    import numpy as np
    from scipy.spatial import ConvexHull
    pts = np.asarray(points, float)
    mid = pts.mean(axis=0)
    tris = [(t if np.cross(t[1] - t[0], t[2] - t[0]) @ (t[0] - mid) >= 0.0 else t[::-1].copy(), colour)
            for t in (pts[f] for f in ConvexHull(pts).simplices)]
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
