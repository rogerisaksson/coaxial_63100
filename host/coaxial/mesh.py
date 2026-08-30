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
for is a one-off: the reduction runs once per process and stays in memory.
"""
import math
import os
import struct

#: Cells across the model for vertex clustering. Measured at 200x56, where
#: the triangle loop is 55% of the frame: grid 200 keeps 48,899 triangles and
#: costs 254 ms, grid 120 keeps 27,628 and costs 164. The mesh is already
#: finer than the character grid either way - 74% of what it draws is
#: sub-pixel - so the coarser one is the one worth having.
GRID = 120

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
    # One iter_unpack over the body, the attribute word skipped by the
    # format: 0.60 s -> 0.46 s with the centring, 116,880 faces, against
    # an unpack_from per face.
    body = memoryview(raw)[84:84 + count * 50]
    for f in struct.iter_unpack('<12f2x', body):
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
    xs = [v[0] for corners, _n in faces for v in corners]
    ys = [v[1] for corners, _n in faces for v in corners]
    zs = [v[2] for corners, _n in faces for v in corners]
    mx = (min(xs) + max(xs)) / 2.0
    my = (min(ys) + max(ys)) / 2.0
    mz = (min(zs) + max(zs)) / 2.0
    span = max(max(xs) - min(xs), max(ys) - min(ys)) or 1.0
    k = 2.0 / span

    return [(tuple(((x - mx) * k, (y - my) * k, (z - mz) * k)
                   for x, y, z in corners), normal)
            for corners, normal in faces]


#: The centred faces of the STL read by this process, by (path, mtime):
#: every LOD and the shadow casters cluster from the same parse. The menu
#: parsed twice, 0.6 s each, for its two solids.
_LOADED = {}


def loaded(path):
    """The model's faces, centred and scaled, parsed once per process."""
    stamp = (path, os.path.getmtime(path))
    got = _LOADED.get(stamp)
    if got is None:
        _LOADED.clear()
        with open(path, 'rb') as f:
            got = _centred(list(_faces(f.read())))
        _LOADED[stamp] = got
    return got


_FACETS = {}


def facets(path, divisions=GRID):
    """(positions, indices, normals) for the model at `path`.

    Three floats per distinct vertex, three indices per triangle and three
    floats of unit normal per triangle, centred and scaled so the widest of
    X and Y is two units. Held in memory for the process - a cache file
    beside the STL was tried and is not wanted in the tree.
    """
    stamp = (path, divisions, os.path.getmtime(path))
    got = _FACETS.get(stamp)
    if got is None:
        got = _FACETS[stamp] = _clustered(loaded(path), divisions)
    return got
