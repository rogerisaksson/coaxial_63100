"""An STL read once and reduced to points on its surface.

The board's own geometry, instead of a parametric stand-in: `render/models`
holds the CAD export, and what a picture of the board should look like is a
question the model already answers.

Reduced, because the export is 419,338 triangles and the drawing is a few
thousand character cells. Every triangle is sub-pixel; carrying them to the
renderer would cost a hundredfold for a picture that cannot show it. What
comes out is an area-weighted sampling of the surface - a point and the
normal there - which is what the renderer consumes anyway.

No numpy. requirements.txt says why it is not here, and the cost it would pay
for is a one-off: the reduction runs once and is cached beside the model, and
what the render loop then walks is the same size as the parametric surface it
replaces.
"""
import math
import os
import struct

#: Points kept from the surface. The render is a z-buffer over these, so
#: what matters is points per framebuffer pixel, not points per triangle:
#: below about one the buffer is sparse, neighbouring cells average whatever
#: few samples happened to land in them, and the board draws as a disc of
#: noise. Measured at 100x40 with two-times supersampling - 32,000 pixels:
#:
#:      45,000 points   0.6 per pixel    a circle of speckle
#:     120,000 points   3.8 per pixel    connectors and planes read cleanly
#:     420,000 points  13.1 per pixel    no better, and 3 fps instead of 9
#:
#: 120,000 builds in 1.3 s once and renders in 107 ms.
SAMPLES = 120000

#: Bumped when the reduction changes, so a stale cache is re-made rather than
#: read as if it came from this code.
CACHE_MAGIC = b'CX63SAMP3'


def _faces(raw):
    """(vertices, normal) per triangle, from an STL that may be either form."""
    if raw[:5] == b'solid' and b'facet normal' in raw[:2048]:
        return _faces_ascii(raw.decode('ascii', 'replace'))
    return _faces_binary(raw)


def _faces_binary(raw):
    count = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) < 84 + count * 50:
        raise ValueError('binary STL claims %d triangles and is %d bytes short'
                         % (count, 84 + count * 50 - len(raw)))

    at = 84
    for _ in range(count):
        f = struct.unpack_from('<12f', raw, at)
        at += 50
        yield ((f[3:6], f[6:9], f[9:12]), f[0:3])


def _faces_ascii(text):
    normal = (0.0, 0.0, 1.0)
    vertices = []
    for line in text.splitlines():
        word = line.split()
        if not word:
            continue
        if word[0] == 'facet' and len(word) >= 5:
            normal = tuple(float(v) for v in word[2:5])
            vertices = []
        elif word[0] == 'vertex' and len(word) >= 4:
            vertices.append(tuple(float(v) for v in word[1:4]))
        elif word[0] == 'endfacet' and len(vertices) == 3:
            yield (tuple(vertices), normal)


def _geometry(vertices, stated):
    """The face's centroid, unit normal and area.

    The normal is recomputed from the winding rather than trusted: exports
    write zeros there, and a zero normal shades as unlit and drops the face
    out of the picture entirely.
    """
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = vertices

    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az

    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    mag = math.sqrt(nx * nx + ny * ny + nz * nz)

    if mag < 1e-20:
        n = stated
        mag2 = math.sqrt(sum(c * c for c in n)) or 1.0
        n = (n[0] / mag2, n[1] / mag2, n[2] / mag2)
        return ((ax + bx + cx) / 3.0, (ay + by + cy) / 3.0,
                (az + bz + cz) / 3.0), n, 0.0

    return (((ax + bx + cx) / 3.0, (ay + by + cy) / 3.0, (az + bz + cz) / 3.0),
            (nx / mag, ny / mag, nz / mag), mag * 0.5)


def bounds(path):
    """(min, max) per axis over every vertex, and the triangle count."""
    raw = open(path, 'rb').read()
    lo = [float('inf')] * 3
    hi = [float('-inf')] * 3
    count = 0

    for vertices, _ in _faces(raw):
        count += 1
        for v in vertices:
            for a in range(3):
                if v[a] < lo[a]:
                    lo[a] = v[a]
                if v[a] > hi[a]:
                    hi[a] = v[a]

    return tuple(lo), tuple(hi), count


def _reduce(raw, count):
    """`count` points on the surface, area-weighted, with their normals.

    Area-weighted rather than one per triangle: an export tessellates a flat
    face into a handful of huge triangles and a fillet into thousands of slivers,
    so per-triangle sampling draws the fillet and loses the face.
    """
    total = 0.0
    for vertices, stated in _faces(raw):
        total += _geometry(vertices, stated)[2]

    if total <= 0.0:
        raise ValueError('the model has no surface area')

    step = total / count
    credit = 0.0
    out = []

    # Deterministic: a picture whose surface is re-sampled per run is a
    # picture that changes when nothing about the board did.
    import random
    rng = random.Random(63100)

    for vertices, stated in _faces(raw):
        centre, normal, area = _geometry(vertices, stated)
        credit += area
        first = True
        while credit >= step:
            credit -= step
            if first:
                out.append((centre, normal))
                first = False
            else:
                # Barycentric, so a triangle big enough for several points
                # gets them spread over it rather than stacked on its middle.
                a, b = rng.random(), rng.random()
                if a + b > 1.0:
                    a, b = 1.0 - a, 1.0 - b
                p, q, r = vertices
                out.append((tuple(p[i] + a * (q[i] - p[i]) + b * (r[i] - p[i])
                                  for i in range(3)), normal))

    return out


def _normalise(points):
    """Centred on the model and scaled so its widest span is two units across.

    The renderer works in units of the board's outer radius, so a model in
    millimetres and one in inches draw the same size.
    """
    lo = [min(p[a] for p, _ in points) for a in range(3)]
    hi = [max(p[a] for p, _ in points) for a in range(3)]
    mid = [(lo[a] + hi[a]) / 2.0 for a in range(3)]
    span = max(hi[a] - lo[a] for a in range(2)) or 1.0
    k = 2.0 / span

    return [(tuple((p[a] - mid[a]) * k for a in range(3)), n)
            for p, n in points]


def _cache_path(path):
    return os.path.splitext(path)[0] + '.samples'


def _write_cache(path, points):
    with open(path, 'wb') as f:
        f.write(CACHE_MAGIC)
        f.write(struct.pack('<I', len(points)))
        for p, n in points:
            f.write(struct.pack('<6f', p[0], p[1], p[2], n[0], n[1], n[2]))


def _read_cache(path, newer_than):
    """The cached sampling, or None if there is not a usable one.

    None rather than an exception for every way it can be unusable - absent,
    stale, from an older reduction, truncated by a run that was interrupted -
    because every one of them means the same thing to the caller: read the STL.
    """
    try:
        if os.path.getmtime(path) < newer_than:
            return None
        with open(path, 'rb') as f:
            if f.read(len(CACHE_MAGIC)) != CACHE_MAGIC:
                return None
            count = struct.unpack('<I', f.read(4))[0]
            raw = f.read(count * 24)
            if len(raw) != count * 24:
                return None
    except OSError:
        return None

    out = []
    for f6 in struct.iter_unpack('<6f', raw):
        out.append((f6[0:3], f6[3:6]))
    return out


def load(path, count=SAMPLES):
    """The model at `path` as [(point, normal)], centred and unit-scaled.

    Cached beside the STL: reducing 419,338 triangles takes about a second and
    a half, which is a second and a half of a live view not being on screen.
    """
    cache = _cache_path(path)
    got = _read_cache(cache, os.path.getmtime(path))
    if got is not None and len(got) >= count // 2:
        return got

    points = _normalise(_reduce(open(path, 'rb').read(), count))
    try:
        _write_cache(cache, points)
    except OSError:
        pass                      # a read-only tree still draws, just slower
    return points
