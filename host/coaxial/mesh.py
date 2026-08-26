"""An STL read once and reduced to a triangle mesh the renderer can draw.

The board's own geometry, instead of a parametric stand-in: `render/models`
holds the CAD export, and what a picture of the board should look like is a
question the model already answers.

Reduced, not resampled. An earlier version turned the surface into points and
the renderer splatted those; the docstring here claimed carrying triangles
would "cost a hundredfold", which was never measured and is not true. It was
measured afterwards: the full 419,338 triangles rasterise in 1.6 s a frame at
100x30, and 74% of the ones that get drawn are sub-pixel. So the mesh is
decimated to 12% of its faces and stays a MESH - which is what lets the
renderer weigh a face by its projected area, the thing point splatting cannot
do and the reason the board kept drawing as a disc of noise.

No numpy. requirements.txt says why it is not here, and the cost it would pay
for is a one-off: the reduction runs once and is cached beside the model.
"""
import array
import math
import os
import struct

#: Cells across the model for vertex clustering. Measured at 200x56, where
#: the triangle loop is 55% of the frame: grid 200 keeps 48,899 triangles and
#: costs 254 ms, grid 120 keeps 27,628 and costs 164. The mesh is already
#: finer than the character grid either way - 74% of what it draws is
#: sub-pixel - so the coarser one is the one worth having.
GRID = 120

#: Bumped when the reduction changes, so a stale cache is re-made rather than
#: read as if it came from this code.
CACHE_MAGIC = b'CX63IDX1'


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


def _clustered(faces, divisions):
    """(positions, indices, normals) for `faces`, vertices snapped to a grid.

    Vertex clustering: every vertex in a grid cell becomes that cell, and a
    triangle whose corners land in fewer than three cells has collapsed and
    is dropped. Crude next to a proper edge-collapse decimator, and it needs
    no topology, no error quadrics and no half-edges - which is what makes it
    forty lines instead of four hundred.

    Indexed, because clustering is what makes sharing worth having: this
    board's 48,899 triangles have only 23,810 distinct corners between them,
    so a renderer that transforms vertices rather than triangle corners does
    a sixth of the work for exactly the same picture.

    Measured on this board, 419,338 triangles in: grid 200 keeps 12% of them,
    320 keeps 22%, 480 keeps 29%. 74% of what the full mesh draws is
    sub-pixel at any size a terminal can show, so 12% loses nothing a reader
    could see.
    """
    step = 2.0 / divisions
    cells = {}
    positions = []
    indices = []
    normals = []

    for corners, stated in faces:
        found = []
        for corner in corners:
            key = (int(math.floor(corner[0] / step)),
                   int(math.floor(corner[1] / step)),
                   int(math.floor(corner[2] / step)))
            got = cells.get(key)
            if got is None:
                got = len(positions) // 3
                positions.append((key[0] + 0.5) * step)
                positions.append((key[1] + 0.5) * step)
                positions.append((key[2] + 0.5) * step)
                cells[key] = got
            found.append(got)

        a, b, c = found
        if a == b or b == c or a == c:
            continue

        normal = face_normal(positions[a * 3:a * 3 + 3],
                             positions[b * 3:b * 3 + 3],
                             positions[c * 3:c * 3 + 3], stated)
        if normal is None:
            continue

        # Shade with the ORIGINAL face's normal, not the snapped triangle's.
        # Snapping moves each corner up to half a cell, which tilts every
        # triangle of a flat surface a different way - so a plane comes out
        # crumpled and draws as static. The geometry may be coarse; the
        # normal it is lit by should not be.
        first = face_normal(corners[0], corners[1], corners[2], stated)
        if first is not None:
            normal = first

        indices.append(a)
        indices.append(b)
        indices.append(c)
        normals.append(normal[0])
        normals.append(normal[1])
        normals.append(normal[2])

    return positions, indices, normals


def face_normal(a, b, c, stated):
    """The unit normal from the winding, or None if the face has no area.

    From the winding rather than the STL's own field: exports write zeros
    there, and a zero normal shades as unlit, which drops the face out of
    the picture entirely.
    """
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]

    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    size = math.sqrt(nx * nx + ny * ny + nz * nz)

    if size > 1e-20:
        return (nx / size, ny / size, nz / size)

    size = math.sqrt(sum(v * v for v in stated))
    if size > 1e-20:
        return tuple(v / size for v in stated)
    return None


def _centred(faces):
    """Every face centred on the model and scaled to two units across.

    The renderer works in units of the board's outer radius, so a model in
    millimetres and one in inches draw the same size.
    """
    low = [min(v[a] for corners, _n in faces for v in corners)
           for a in range(3)]
    high = [max(v[a] for corners, _n in faces for v in corners)
            for a in range(3)]
    mid = [(low[a] + high[a]) / 2.0 for a in range(3)]
    span = max(high[a] - low[a] for a in range(2)) or 1.0
    k = 2.0 / span

    return [(tuple(tuple((v[a] - mid[a]) * k for a in range(3))
                   for v in corners), normal) for corners, normal in faces]


def _cache_path(path):
    return os.path.splitext(path)[0] + '.facets'


def _write_cache(path, positions, indices, normals):
    with open(path, 'wb') as f:
        f.write(CACHE_MAGIC)
        f.write(struct.pack('<II', len(positions) // 3, len(normals) // 3))
        array.array('f', positions).tofile(f)
        array.array('i', indices).tofile(f)
        array.array('f', normals).tofile(f)


def _read_cache(path, newer_than):
    """The cached triangles, or None if there is not a usable one.

    None rather than an exception for every way it can be unusable - absent,
    stale, from an older reduction, truncated by a run that was interrupted -
    because every one of them means the same thing to the caller: read the
    STL.
    """
    try:
        if os.path.getmtime(path) < newer_than:
            return None
        with open(path, 'rb') as f:
            if f.read(len(CACHE_MAGIC)) != CACHE_MAGIC:
                return None
            points, faces = struct.unpack('<II', f.read(8))
            positions = array.array('f')
            indices = array.array('i')
            normals = array.array('f')
            positions.fromfile(f, points * 3)
            indices.fromfile(f, faces * 3)
            normals.fromfile(f, faces * 3)
    except (OSError, EOFError, struct.error):
        return None

    # Lists, not the arrays they were stored as: reading an element of an
    # array('f') builds a new Python float every time, and the render loop
    # reads a dozen of them per triangle.
    return list(positions), list(indices), list(normals)


def facets(path, divisions=GRID):
    """(positions, indices, normals) for the model at `path`.

    Three floats per distinct vertex, three indices per triangle and three
    floats of unit normal per triangle, centred and scaled so the widest of
    X and Y is two units. Cached beside the STL: parsing and
    decimating 419,338 triangles takes about two seconds, which is two
    seconds of a live view not being on screen.
    """
    cache = _cache_path(path)
    got = _read_cache(cache, os.path.getmtime(path))
    if got is not None:
        return got

    positions, indices, normals = _clustered(
        _centred(list(_faces(open(path, 'rb').read()))), divisions)
    try:
        _write_cache(cache, positions, indices, normals)
    except OSError:
        pass                      # a read-only tree still draws, just slower
    return positions, indices, normals
