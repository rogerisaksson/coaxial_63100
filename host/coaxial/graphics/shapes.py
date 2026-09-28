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
    """A closed body through elliptical rings (at, rx, rz[, dz[, lean]]) up its axis, capped at
    `poles` (default the first and last ring): (corners, triangles, uv, materials) in its part's
    frame. `along` 'z' lays it forward, a ring's rz then its height and dz its drop; `lean`
    raises a ring's front lean * rz and lowers its back as far."""
    from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
    k = np.arange(AROUND) * (2.0 * math.pi / AROUND)
    rows = [np.stack([r[1] * np.cos(k), float(r[0]) + (r[4] if len(r) > 4 else 0.0) * r[2] * np.sin(k),
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


def moved(points, normals, spans, frames):
    """Points and normals each moved into the body's frame by its part's (turn, spot)."""
    from coaxial.model.blocks import numpy as np
    out_p, out_n = np.empty_like(points), np.empty_like(normals)
    for (lo, hi), (turn, spot) in zip(spans, frames):
        out_p[lo:hi] = points[lo:hi] @ turn.T + spot
        out_n[lo:hi] = normals[lo:hi] @ turn.T
    return out_p, out_n


#: How far apart the dots drawn without a card are sampled on the surface, metres: under two
#: fine dots at 110 x 50 cells, so a 2x2 splat closes it; 7 mm was 278 000 points, 290 ms.
DENSE_M = 0.012


def sampled(body, part_of):
    """Every triangle of `body` - its corners, normals, uv, materials, index and parts laid end to
    end - sampled on a barycentric grid DENSE_M apart, grouped by part, `part_of` each
    triangle's: ((points, normals, uv, materials), [(lo, hi)] a part)."""
    from coaxial.model.blocks import numpy as np
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
    """The 3x3 that turns `degrees` about `axis`, 'x', 'y' or 'z'."""
    from coaxial.model.blocks import numpy as np
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    if axis == 'x':
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    if axis == 'y':
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def view(yaw, pitch):
    """The view as the engine's 3x3, row-major: turned `yaw` about the upright, tipped `pitch`
    down."""
    return tuple((turn_about('x', pitch) @ turn_about('y', -yaw)).ravel().tolist())
