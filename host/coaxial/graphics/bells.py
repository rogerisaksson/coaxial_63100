"""Each drive's motor as its maker's model has it: the model's turned outline laid over its can.

A bell over its stator's foot, a hub on its face, read off a model beside the tree:

    MODELS = bells.found()                                 # {name: path}: the motors' models found
    rings, tall = bells.outline(path)                      # [(z, r)] its outline (height 1, rim 1), m/m
    mesh = bells.turned(kind, radius, length, letter, end, ink)   # its can, None with no model found

A model is a motor's OBJ (`wavefront`): its axis z, its stator's foot at z = 0 and its face up, metres -
the outer bodies of its maker's STEP, tessellated - under motors/ beside the tree (BESIDE folders up, as
`likeness` looks), else in the folder COAXIAL_MOTORS names; never the tree's. A motor is drawn after
the model whose name holds its kind's every word (one named 'U8 Lite' a U8's and a U8_LITE's), else
after the one whose height over its width is nearest its can's, laid over its can either way - its
rim the can's, its height the can's, its face toward its gearbox; with none found, its drum. The
outline is the model's corners and its triangles told on grids in BINS rings out from its axis, the
lowest and the highest in each (a ring with none its neighbours'), a point dropped within FINE of the
line its neighbours keep.
"""
import functools
import glob
import os

from coaxial.graphics import wavefront
from coaxial.graphics.likeness import BESIDE
from coaxial.graphics.shapes import ALONG, lain, loft

#: A model's outline is told in BINS rings out from its axis and kept to within FINE of its height and
#: its rim: a U8's foot, its bell, its face's slope and its hub in a dozen points.
BINS, FINE = 40, 0.01


def found(root=None):
    """{name: path} of the motors' models: COAXIAL_MOTORS's OBJs, else those under motors/ beside the
    repo `root` (the tree's unless told) or the checkout a worktree of it stands in, BESIDE folders
    up at the most; {} with none."""
    said = os.environ.get('COAXIAL_MOTORS')
    folders = [said] if said is not None else [os.path.join(b, 'motors') for b in wavefront.roots(root, BESIDE)]
    for folder in folders:
        got = sorted(glob.glob(os.path.join(folder, '*.obj')))
        if got:
            return {os.path.splitext(os.path.basename(path))[0]: path for path in got}
    return {}


MODELS = found()


def _thinned(points):
    """`points` [(z, r)] less those within FINE of the line through the two kept either side of them
    (Douglas and Peucker's), its ends kept."""
    from coaxial.model.blocks import numpy as np
    p = np.asarray(points, float)
    keep = np.zeros(len(p), bool)
    keep[[0, -1]] = True
    spans = [(0, len(p) - 1)]
    while spans:
        a, b = spans.pop()
        if b - a < 2:
            continue
        way, off = p[b] - p[a], p[a + 1:b] - p[a]
        long = float(np.hypot(*way))
        far = (np.abs(way[0] * off[:, 1] - way[1] * off[:, 0]) / long if long > FINE * FINE
               else np.hypot(off[:, 0], off[:, 1]))
        k = int(np.argmax(far))
        if far[k] > FINE:
            keep[a + 1 + k] = True
            spans += [(a, a + 1 + k), (a + 1 + k, b)]
    return [(float(z), float(r)) for z, r in p[keep]]


@functools.lru_cache(maxsize=None)
def outline(path):
    """([(z, r)], tall): the turned outline of the motor's model at `path`, its height 1 and its rim
    1 - from by its axis under its foot out to its rim, up it and back in over its face (BINS,
    FINE) -, and its height over its width."""
    from coaxial.model.blocks import numpy as np
    corners, groups, _at = wavefront.read(path)
    at = corners[np.vstack([t for _names, t in groups])]
    # each triangle told on a grid of its own, its cells no wider than a ring: a broad face's middle
    # had no corner in it, and the holes through it stood for its face there
    step = float(np.hypot(corners[:, 0], corners[:, 1]).max()) / BINS
    cells = np.ceil(np.linalg.norm(at - np.roll(at, 1, axis=1), axis=2).max(axis=1) / step).astype(int).clip(1)
    dots = [corners]
    for k in np.unique(cells):
        i, j = np.mgrid[0:k + 1, 0:k + 1]
        grid = np.stack([i, j], -1)[i + j <= k].reshape(-1, 2) / k
        w = np.column_stack([grid, 1.0 - grid.sum(axis=1)])
        dots.append(np.einsum('pj,tjd->tpd', w, at[cells == k]).reshape(-1, 3))
    points = np.vstack(dots)
    r, z = np.hypot(points[:, 0], points[:, 1]), points[:, 2] - points[:, 2].min()
    tall = float(z.max() / (2.0 * r.max()))
    r, z = r / r.max(), z / z.max()
    ring = np.minimum((r * BINS).astype(int), BINS - 1)
    low, high = np.full(BINS, np.inf), np.full(BINS, -np.inf)
    np.minimum.at(low, ring, z)
    np.maximum.at(high, ring, z)
    told, mid = np.isfinite(low), (np.arange(BINS) + 0.5) / BINS
    low, high = (np.interp(mid, mid[told], each[told]) for each in (low, high))
    return (_thinned(list(zip(low, mid)) + [(low[-1], 1.0), (high[-1], 1.0)] + list(zip(high[::-1], mid[::-1]))),
            tall)


def model(kind, tall):
    """The path of the model a motor of `kind` is drawn after (MODELS): the one whose name holds its
    kind's every word, else the one whose height over its width stands nearest `tall`."""
    words = kind.lower().split('_')
    named = [path for name, path in MODELS.items() if all(word in name.lower().replace(' ', '') for word in words)]
    return named[0] if named else min(MODELS.values(), key=lambda path: abs(outline(path)[1] - tall))


def turned(kind, radius, length, letter, end, ink):
    """A motor of `kind`'s can `radius` round and `length` long about its part's origin on its axis
    `letter` (`shapes.lain`), its face toward `end` (1 or -1 along it): its model's outline turned
    (`outline`, `shapes.loft`); None where no model is found (MODELS)."""
    if not MODELS:
        return None
    from coaxial.model.blocks import numpy as np
    rings = [((z - 0.5) * length, r * radius, r * radius)
             for z, r in outline(model(kind, length / (2.0 * radius)))[0]]
    corners, triangles, uv, materials = loft(rings, ink, poles=(rings[0][0], rings[-1][0]))
    if end * float(np.asarray(ALONG[letter])[1] @ np.eye(3)['xyz'.index(letter)]) < 0.0:
        corners, triangles = corners * (1.0, -1.0, 1.0), triangles[:, ::-1]
    return lain((corners, triangles, uv, materials), letter)
