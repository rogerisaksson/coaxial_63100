"""The STL as solids. A solid is (positions, indices, normals) - the
export decimated on a grid (`mesh._clustered`), with the bore's corners
kept exact - held in memory for the process, keyed on the file's mtime;
the slab's top and bottom faces measured off a solid's own vertices;
the shadow casters' coarse solid; and the parametric board for a tree
with no STL. `wireframe._lods` builds the levels of detail from here
and takes the preload's pickle in."""
import functools
import os

from . import crew, mesh
from .. import orientation


#: In-memory decimates of the STL, keyed on (path, divisions, mtime):
#: a fresh export replaces them by itself, and NOTHING is written
#: beside the model - the mesh module's disk cache is deliberately
#: bypassed here.
_MESHES = {}


def _decimated(path, divisions):
    stamp = (path, divisions, os.path.getmtime(path))
    got = _MESHES.get(stamp)
    if got is not None:
        return got
    # Six LODs and the shadow casters of one file coexist; only a
    # runaway set - a re-exported STL changing every stamp - clears
    # the lot.
    if len(_MESHES) > MESHES_KEPT:
        _forget()
    got = _MESHES[stamp] = mesh._clustered(mesh.loaded(path), divisions,
                                           keep=_bore_keep)
    return got


#: THE BORE STAYS EXACT in every decimate: corners within BORE_KEEP of
#: the axis - the hole is 0.2 across at the origin, its wall 0.032
#: thick - are their own clusters. Clustered, the wall's rings merged
#: (0.032 against a 0.042 cell at grid 48) and the see-through came
#: out smaller and shifted from the mesh's circle the art is drawn
#: to, so the bench saw TWO holes at i -0.7063 j 0.2652 k -0.4117
#: real 0.5110 (2026-09-23): the art's blank, ringed, up and right of
#: the decimate's slit. 86 corners at radius 0.100, 257 triangles
#: touching them, against the 48 decimate's 5 645.
BORE_KEEP = 0.115


def _bore_keep(corner):
    return corner[0] * corner[0] + corner[1] * corner[1] < BORE_KEEP * BORE_KEEP


def _forget():
    """Every solid this process holds, and the slab planes measured off
    them. `wireframe._forget` clears the outline's and the pre-scan's
    caches with it: a solid's id can come round again."""
    _MESHES.clear()
    _PLANES.clear()


#: Decimates kept before the cache is emptied - six LODs and the shadow
#: casters of one file coexist, so only a runaway set clears it.
MESHES_KEPT = 8


#: The decimation each zoom band earns, (zoom below, grid divisions).
#: Finer decimates cost real raster time - measured at 94x36, single
#: process: 16 -> 7.8 ms, 24 -> 13, 32 -> 29, 48 -> 37, 64 -> 63 - so
#: each only draws past the zoom that can see it. The parse is 0.46 s
#: once a process, then 0.14-0.18 s a grid; all six 1.2 s in parallel,
#: each worker parsing its own - which is why a view loads
#: them up front, behind its boot strip, not on the first zoom that
#: wants one.
LODS = ((1.0, 12), (1.5, 16), (2.0, 24), (2.7, 32), (3.6, 48), (None, 64))


def _decimate_missing(path, stamp, missing, progress):
    """The absent decimates, built by the crew at once."""
    if len(_MESHES) + len(missing) > MESHES_KEPT:
        _forget()
    for divisions, solid in crew.decimate(path, missing, progress).items():
        _MESHES[(path, divisions, stamp)] = solid


@functools.cache
def _parametric():
    """The parametric board, only for a tree without the STL - once,
    since `facets` at 48 steps is not free."""
    return orientation.facets(steps=48, relief=1.5)


@functools.cache
def _parametric_casters():
    """The parametric board for the shadow pass, without an STL."""
    return orientation.facets(steps=20, relief=1.5)


def _casters() -> tuple:
    """The shadow pass's own solid: the same STL, coarser still. The
    mesh cache keys on the file's mtime, so a fresh export replaces
    both solids by itself."""
    try:
        return _decimated(orientation.MODEL, 10)
    except (OSError, ValueError):
        return _parametric_casters()


def _slab_top(pos):
    """The z of the slab's top face, from the mesh: the most populated
    z level - unless a level at least 60 % as populated lies a
    millimetre or more ABOVE it, which is the slab's other face when
    the bottom happened to win the count. A part's lid never comes
    near the slab's population. Measured, never assumed - see
    OUTLINE_RISE."""
    counts = {}
    for i in range(2, len(pos), 3):
        key = round(pos[i], 4)
        counts[key] = counts.get(key, 0) + 1
    if not counts:
        return 0.0
    mode = max(counts, key=lambda k: counts[k])
    floor = 0.6 * counts[mode]
    higher = [z for z, n in counts.items()
              if n >= floor and z - mode >= 0.02]
    return max(higher) if higher else mode


#: A z level this share of the slab's most populated one, at least
#: 0.02 units (a millimetre) under the top, is the slab's bottom face.
SLAB_BOTTOM_SHARE = 0.3


def _slab_bottom(pos, top):
    """The z of the slab's bottom face, from the mesh: the most populated
    level a millimetre or more under `top` that still carries
    SLAB_BOTTOM_SHARE of the top's population - or None for a slab with
    one face, like the suite's synthetic one."""
    counts = {}
    for i in range(2, len(pos), 3):
        key = round(pos[i], 4)
        counts[key] = counts.get(key, 0) + 1
    if not counts:
        return None
    floor = SLAB_BOTTOM_SHARE * max(counts.values())
    under = [z for z, n in counts.items() if n >= floor and top - z >= 0.02]
    return max(under, key=lambda z: counts[z]) if under else None


#: Per solid, by identity: the slab's (top, bottom) z, the faces the
#: art is read on - measured off the solid's own vertices once.
_PLANES = {}


def _slab_planes(solid):
    """(top, bottom) of `solid`'s slab, from its vertices (`_slab_top`,
    `_slab_bottom`), once per solid."""
    got = _PLANES.get(id(solid))
    if got is None:
        if len(_PLANES) > MESHES_KEPT:
            _PLANES.clear()
        top = _slab_top(solid[0])
        got = _PLANES[id(solid)] = (top, _slab_bottom(solid[0], top))
    return got
