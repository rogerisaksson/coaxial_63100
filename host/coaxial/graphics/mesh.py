"""An STL read once and reduced to a triangle mesh the renderer can draw."""
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
    # One iter_unpack over the body, the attribute word skipped by the format:
    # 0.60 s -> 0.46 s with the centring, 116,880 faces, against an unpack_from
    # per face.
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


def cell_key(point, step):
    """The grid cell a point clusters into at `step` units a cell: the
    one rule for the decimate and for anything that must land where the
    decimate's vertices did."""
    return (int(math.floor(point[0] / step)), int(math.floor(point[1] / step)),
            int(math.floor(point[2] / step)))


def _clustered(faces, divisions, keep=None):
    """(positions, indices, normals) for `faces`, vertices snapped to a
    grid.
    """
    step = 2.0 / divisions
    cells = {}
    centres = []            # the cell's middle, for the collapse test
    sums = []               # the corners that landed in it, summed, and
    indices = []            # how many: the vertex is their MEAN
    normals = []

    for corners, stated in faces:
        found = []
        for corner in corners:
            if keep is not None and keep(corner):
                key = (round(corner[0], 6), round(corner[1], 6),
                       round(corner[2], 6), 'exact')
                centre = corner
            else:
                key = cell_key(corner, step)
                centre = ((key[0] + 0.5) * step, (key[1] + 0.5) * step,
                          (key[2] + 0.5) * step)
            got = cells.get(key)
            if got is None:
                got = len(sums)
                centres.append(centre)
                sums.append([0.0, 0.0, 0.0, 0])
                cells[key] = got
            acc = sums[got]
            acc[0] += corner[0]
            acc[1] += corner[1]
            acc[2] += corner[2]
            acc[3] += 1
            found.append(got)

        a, b, c = found
        if a == b or b == c or a == c:
            continue

        normal = face_normal(centres[a], centres[b], centres[c], stated)
        if normal is None:
            continue

        # Shade with the ORIGINAL face's normal, not the snapped triangle's.
        first = face_normal(corners[0], corners[1], corners[2], stated)
        if first is not None:
            normal = first

        indices.append(a)
        indices.append(b)
        indices.append(c)
        normals.append(normal[0])
        normals.append(normal[1])
        normals.append(normal[2])

    # THE VERTEX IS THE MEAN OF WHAT LANDED IN THE CELL, not the cell's middle.
    positions = []
    for sx, sy, sz, n in sums:
        positions.append(sx / n)
        positions.append(sy / n)
        positions.append(sz / n)
    return positions, indices, normals


def face_normal(a, b, c, stated):
    """The unit normal from the winding, or None if the face has no area."""
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
    """Every face centred on the model and scaled to two units across."""
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
    """(positions, indices, normals) for the model at `path`."""
    stamp = (path, divisions, os.path.getmtime(path))
    got = _FACETS.get(stamp)
    if got is None:
        got = _FACETS[stamp] = _clustered(loaded(path), divisions)
    return got
