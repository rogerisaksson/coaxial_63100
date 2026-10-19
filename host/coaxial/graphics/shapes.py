"""Closed meshes for the lit raster (`lit`): lofts, ellipsoids, drums, posed and sampled.

A loft runs through elliptical rings; normals are smoothed within a part, parts moved by their
frames, surfaces sampled into dots.

    corners, triangles, uv, materials = loft(rings, material)    # in its part's frame, y up
    positions, normals = moved(corners, normals, spans, frames)  # each part by (turn, spot)
    m = view(yaw, pitch)                                          # the engine's 3x3
"""
import math

#: Corners round a ring.
AROUND = 20


def loft(rings, material, poles=None, along='y'):
    """A closed body through elliptical rings (at, rx, rz[, dz[, lean[, dx]]]) up its axis, capped
    at `poles` (default the first and last ring): (corners, triangles, uv, materials) in its part's
    frame. `along` 'z' lays it forward, a ring's rz then its height and dz its drop; `lean`
    raises a ring's front lean * rz and lowers its back as far; `dx` moves a ring across."""
    from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
    k = np.arange(AROUND) * (2.0 * math.pi / AROUND)
    rows = [np.stack([r[1] * np.cos(k) + (r[5] if len(r) > 5 else 0.0),
                      float(r[0]) + (r[4] if len(r) > 4 else 0.0) * r[2] * np.sin(k),
                      r[2] * np.sin(k) + (r[3] if len(r) > 3 else 0.0)], 1) for r in rings]
    low, high = poles or (rings[0][0], rings[-1][0])
    ends = [[0.0, low, rings[0][3] if len(rings[0]) > 3 else 0.0],
            [0.0, high, rings[-1][3] if len(rings[-1]) > 3 else 0.0]]
    corners = np.vstack(rows + [np.array(ends)])
    n, j = len(rows), np.arange(AROUND)
    a = (np.arange(n - 1)[:, None] * AROUND + j).ravel()
    b = (np.arange(n - 1)[:, None] * AROUND + (j + 1) % AROUND).ravel()
    c, d = a + AROUND, b + AROUND
    bottom, top = n * AROUND, n * AROUND + 1
    last = (n - 1) * AROUND
    triangles = np.vstack([np.stack([a, c, b], 1), np.stack([c, d, b], 1),
                           np.stack([np.full(AROUND, bottom), j, (j + 1) % AROUND], 1),
                           np.stack([np.full(AROUND, top), last + (j + 1) % AROUND, last + j], 1)])
    uv = np.stack([np.concatenate([np.tile(j / AROUND, n), [0.0, 0.0]]), corners[:, 1]], 1)
    if along == 'z':                       # the axis forward: (x, y, z) -> (x, -z, y)
        corners = np.stack([corners[:, 0], -corners[:, 2], corners[:, 1]], 1)
    if callable(material):
        materials = material(corners)
    else:
        materials = np.full(len(corners), material)
    return corners, triangles, uv, materials


def limb(length, top, middle, bottom, material, flat=1.0, bulge_at=0.3, out=0.0, ahead=0.0,
         behind=0.0):
    """A tapered limb hanging from its joint down -y: `top` at the joint, `middle` at `bulge_at` of
    the way, `bottom` at the end, rounded; `flat` its depth over its width; its middle rings `out`
    across (+x), its upper rings `ahead` forward of its bone and `behind` deeper at its back - half
    that half way down -, the ends where they are."""
    rings = [(-length - 0.6 * bottom, 0.5 * bottom), (-length - 0.3 * bottom, 0.9 * bottom),
             (-length, bottom), (-length * (0.5 + 0.5 * bulge_at), 0.5 * (middle + bottom)),
             (-length * bulge_at, middle), (0.0, top), (0.3 * top, 0.88 * top),
             (0.55 * top, 0.5 * top)]
    deep = {3: 0.25 * behind, 4: 0.5 * behind, 5: 0.5 * behind}
    return loft([(y, r, r * flat + deep.get(i, 0.0), (ahead if i in (3, 4, 5) else 0.0)
                  - deep.get(i, 0.0), 0.0, out if i in (3, 4) else 0.0)
                 for i, (y, r) in enumerate(rings)], material)


def ellipsoid(centre, radii, material, rows=10):
    """An ellipsoid about `centre`, its poles on its y axis."""
    from coaxial.model.blocks import numpy as np
    cx, cy, cz = centre
    rx, ry, rz = radii
    phi = np.pi * np.arange(1, rows) / rows
    body = loft([(cy - ry * math.cos(p), rx * math.sin(p), rz * math.sin(p), cz) for p in phi],
                material, poles=(cy - ry, cy + ry))
    corners = body[0]
    corners[:, 0] += cx
    return body


def box(centre, half, material):
    """A box about `centre`, `half` sizes each way, its faces flat: four corners a face."""
    from coaxial.model.blocks import numpy as np
    cx, cy, cz = centre
    hx, hy, hz = half
    faces = []
    for axis, sign in ((0, -1.0), (0, 1.0), (1, -1.0), (1, 1.0), (2, -1.0), (2, 1.0)):
        u, v = [k for k in range(3) if k != axis]
        for su, sv in ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)):
            p = [0.0, 0.0, 0.0]
            p[axis], p[u], p[v] = sign, su, sv
            faces.append(p)
    corners = np.array(faces) * np.array([hx, hy, hz]) + np.array([cx, cy, cz])
    quads = np.arange(6)[:, None, None] * 4 + np.array([[0, 1, 2], [0, 2, 3]])[None]
    triangles = quads.reshape(-1, 3)
    uv = np.zeros((len(corners), 2))
    return corners, triangles, uv, np.full(len(corners), material)


def drum(radius, length, axis, material):
    """A drum `radius` round and `length` long on its part's `axis` ('x', 'y' or 'z') through
    the origin, its rims chamfered."""
    from coaxial.model.blocks import numpy as np
    half, lip = length / 2.0, min(0.004, 0.1 * radius)
    rings = [(-half, radius - lip, radius - lip), (-half + lip, radius, radius),
             (half - lip, radius, radius), (half, radius - lip, radius - lip)]
    corners, triangles, uv, materials = loft(rings, material)
    turn = {'x': ((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            'y': ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            'z': ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0))}[axis]
    return corners @ np.asarray(turn), triangles, uv, materials


def strut(a, b, radius, material):
    """A drum `radius` round from `a` to `b`, never straight up."""
    from coaxial.model.blocks import numpy as np
    a, b = np.asarray(a, float), np.asarray(b, float)
    u = (b - a) / np.linalg.norm(b - a)
    v = np.cross((0.0, 0.0, 1.0), u)
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    turn = np.eye(3) + vx + vx @ vx / (1.0 + u[2])
    corners, triangles, uv, materials = drum(radius, float(np.linalg.norm(b - a)), 'z', material)
    return corners @ turn.T + 0.5 * (a + b), triangles, uv, materials


def naca(u):
    """A symmetric NACA section's half thickness at `u` of its chord, over its thickness."""
    u = min(1.0, max(0.0, u))
    return 5.0 * (0.2969 * math.sqrt(u) - 0.1260 * u - 0.3516 * u * u + 0.2843 * u ** 3 - 0.1036 * u ** 4)


def surface(stations, rows, edges, half, place, colour, ink=None):
    """A lifting surface's panel between `edges(s) -> (front, back)` over the span `stations`, as
    (corners, triangles, uv, materials): its chord's `rows` (y, falling) held between the edges,
    `half(s, y)` its half thickness, `place(s, y, t)` its point `t` through (-: its top),
    `ink(s, y, side)` a quad's lettering; its faces turned out, its edges and ends closed."""
    from coaxial.model.blocks import numpy as np
    grid = [[min(edges(s)[0], max(edges(s)[1], r)) for r in rows] for s in stations]
    quads = []

    def face(pts, inside, mat):
        p = [np.asarray(place(*q), float) for q in pts]
        if np.linalg.norm(np.cross(p[2] - p[0], p[3] - p[1])) < 1e-12:
            return
        out = 0.25 * sum(p) - np.asarray(place(*inside), float)
        quads.append((p if np.cross(p[2] - p[0], p[3] - p[1]) @ out > 0.0 else p[::-1], mat))
    for i in range(len(stations) - 1):
        s0, s1 = stations[i], stations[i + 1]
        for j in range(len(rows) - 1):
            ys = (grid[i][j], grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1])
            sm, ym = 0.5 * (s0 + s1), 0.25 * sum(ys)
            for side in (-1.0, 1.0):
                pts = [(s, y, side * half(s, y)) for s, y in zip((s0, s1, s1, s0), ys)]
                face(pts, (sm, ym, 0.0), ink[1] if ink and ink[0](sm, ym, side) else colour)
        for j, way in ((0, -1.0), (len(rows) - 1, 1.0)):
            y0, y1 = grid[i][j], grid[i + 1][j]
            face([(s0, y0, -half(s0, y0)), (s1, y1, -half(s1, y1)), (s1, y1, half(s1, y1)),
                  (s0, y0, half(s0, y0))], (0.5 * (s0 + s1), 0.5 * (y0 + y1) + 0.01 * way, 0.0), colour)
    for i, way in ((0, 1.0), (len(stations) - 1, -1.0)):
        s = stations[i]
        for j in range(len(rows) - 1):
            y0, y1 = grid[i][j], grid[i][j + 1]
            face([(s, y0, -half(s, y0)), (s, y1, -half(s, y1)), (s, y1, half(s, y1)), (s, y0, half(s, y0))],
                 (s + 0.01 * way, 0.5 * (y0 + y1), 0.0), colour)
    corners = np.array([q for pts, _m in quads for q in pts], float)
    index = np.arange(len(quads))[:, None, None] * 4 + np.array([[0, 1, 2], [0, 2, 3]])[None]
    return corners, index.reshape(-1, 3), np.zeros((len(corners), 2)), np.array([m for _p, m in quads for _ in range(4)])


def smooth(corners, index):
    """Smooth normals: each corner's faces' normals, area-weighted - within its own part, whose
    corners no other part shares."""
    from coaxial.model.blocks import numpy as np
    a, b, c = (corners[index[:, k]] for k in range(3))
    faces = np.cross(b - a, c - a)
    out = np.zeros_like(corners)
    for k in range(3):
        np.add.at(out, index[:, k], faces)
    length = np.linalg.norm(out, axis=1, keepdims=True)
    return out / np.where(length > 0.0, length, 1.0)


_SHAPES_NP = None
_EYE3 = None


def _np():
    global _SHAPES_NP, _EYE3
    if _SHAPES_NP is None:
        from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
        _SHAPES_NP, _EYE3 = np, np.eye(3)
        _EYE3.flags.writeable = False
    return _SHAPES_NP


def moved(points, normals, spans, frames):
    """Points and normals each moved into the body's frame by its part's (turn, spot)."""
    np = _SHAPES_NP or _np()
    out_p, out_n = np.empty_like(points), np.empty_like(normals)
    for (lo, hi), (turn, spot) in zip(spans, frames):
        if hi > lo:
            t = turn.T
            np.matmul(points[lo:hi], t, out=out_p[lo:hi])
            out_p[lo:hi] += spot
            np.matmul(normals[lo:hi], t, out=out_n[lo:hi])
    return out_p, out_n


#: How far apart the dots drawn without a card are sampled on the surface, metres: under two
#: fine dots at 110 x 50 cells, so a 2x2 splat closes it; 7 mm was 278 000 points, 290 ms.
DENSE_M = 0.012


def sampled(body, part_of):
    """Every triangle of `body` - its corners, normals, uv, materials, index and parts laid end to
    end - sampled on a barycentric grid DENSE_M apart, grouped by part, `part_of` each
    triangle's: ((points, normals, uv, materials), [(lo, hi)] a part)."""
    np = _SHAPES_NP or _np()
    index = body.index.astype(int)
    a, b, c = (body.corners[index[:, k]] for k in range(3))
    edge = np.max([np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1),
                   np.linalg.norm(a - c, axis=1)], axis=0)
    steps = np.maximum(1, np.ceil(edge / DENSE_M)).astype(int)
    fields = (body.corners, body.normals, body.uv)
    chunks = [[] for _ in range(len(fields) + 1)]
    parts = []
    for n in np.unique(steps):
        tris = np.flatnonzero(steps == n)
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
        keep = (i + j) <= n
        weights = np.stack([n - i[keep] - j[keep], i[keep], j[keep]], 1) / float(n)
        for out, field in zip(chunks, fields):
            corners = np.stack([field[index[tris, k]] for k in range(3)], 1)
            out.append(np.einsum('sk,tk...->ts...', weights, corners).reshape(
                -1, *field.shape[1:]))
        # A material is its nearest corner's: blended, a hood's edge came out skin.
        chunks[3].append(body.materials[index[tris][:, weights.argmax(axis=1)]].ravel())
        parts.append(np.repeat(part_of[tris], len(weights)))
    order = np.argsort(np.concatenate(parts), kind='stable')
    dense = [np.concatenate(out)[order] for out in chunks]
    counts = np.bincount(np.concatenate(parts), minlength=len(body.parts))
    ends = np.cumsum(counts)
    return tuple(dense), list(zip((ends - counts).tolist(), ends.tolist()))


def turn_about(axis, degrees):
    """The 3x3 that turns `degrees` about `axis`, 'x', 'y', 'z' or a unit vector."""
    np = _SHAPES_NP or _np()
    if not degrees:
        return _EYE3 if _EYE3 is not None else np.eye(3)
    rad = math.radians(degrees)
    c, s = math.cos(rad), math.sin(rad)
    if not isinstance(axis, str):
        ux = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
        return np.eye(3) + s * ux + (1.0 - c) * (ux @ ux)
    if axis == 'x':
        return np.array(((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c)))
    if axis == 'y':
        return np.array(((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c)))
    return np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))


def view(yaw, pitch):
    """The view as the engine's 3x3, row-major: turned `yaw` about the upright, tipped `pitch`
    down."""
    return tuple((turn_about('x', pitch) @ turn_about('y', -yaw)).ravel().tolist())
