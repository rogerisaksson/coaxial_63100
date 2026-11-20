"""A printed piece as a solid: its drawing filled to voxels - its volume, its mass, its overhang.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/solids.py [NAME ...]

A piece as `prints.laid` lays it on its bed is a drawing: shells that overlap, wound either way. `filled` makes a
solid of it: the shells' faces marked in a grid of VOXEL_M cubes, the room round them flooded from the grid's edge,
and all the flood never reached kept - a face, or what faces enclose. `told` reads the solid: its volume (a face's
cell counted half) and mass in the print's material, the share of its cells that are faces alone - near one where
the flood got in: a shell drawn open, its volume then the faces' only - and its overhang: the faces' cells with
nothing under them, straight down or one cell aside (45 degrees), off the bed. `hung` tells the pieces held to
stand on their beds with nothing hung over them: the hip's own (STANDING), the trunk's solid ones (TRUNK).
"""
import argparse
import functools
import os
import sys
from typing import Any

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import knee_mount as km  # noqa: E402
from tools.render import knee_topo  # noqa: E402
from tools.render import prints  # noqa: E402

#: A cell's side, m, and how many points a cell's face is marked with along it.
VOXEL_M, MARKS = 0.0006, 3.0
#: The pieces held to hang nothing over their beds - the hip's frame - and what of an overhang is the cells' own
#: noise on a face that slopes or steps by less than a cell, cm2.
STANDING, NOISE_CM2 = ('yaw band', 'yoke', 'block', 'cradle', 'thigh arm', 'thigh knuckle'), 0.5
TRUNK = ('trunk web', 'trunk column', 'trunk tower', 'trunk fork')


@functools.lru_cache(maxsize=None)
def filled(name):
    """(solid, faces): a piece's cells [across, up, deep], VOXEL_M a side, a cell of room round them - those the
    flood from the grid's edge never reached, and of them those a face of the drawing passes."""
    import numpy as np
    from scipy import ndimage
    points = km._points(prints.laid(name)[0], VOXEL_M / MARKS)
    cells = np.floor(points / VOXEL_M).astype(int) + 1
    faces = np.zeros(cells.max(axis=0) + 2, bool)
    faces[tuple(cells.T)] = True
    labelled: Any = ndimage.label(~faces)
    rooms = labelled[0]
    return rooms != rooms[0, 0, 0], faces


def told(name):
    """{'cm3', 'g': the piece's volume and mass, 'shell': the share of its cells that are faces alone, 'over': cm2 of
    its faces with nothing under them within 45 degrees, off the bed}."""
    import numpy as np
    from scipy import ndimage
    solid, faces = filled(name)
    inside = solid & ~faces
    volume = (inside.sum() + 0.5 * faces.sum()) * VOXEL_M ** 3
    under: Any = ndimage.binary_dilation(solid, np.ones((3, 1, 3), bool))[:, :-1, :]    # a cell below, or one aside
    hung = solid[:, 1:, :] & ~under
    hung[:, :2, :] = False                                 # the bed's own layers
    return {'cm3': round(1e6 * float(volume), 1), 'g': round(1e3 * float(volume) * knee_topo.DENSITY, 1),
            'shell': round(float(faces.sum()) / max(int(solid.sum()), 1), 2),
            'over': round(1e4 * float(hung.sum()) * VOXEL_M ** 2, 1)}


def hung(names=STANDING):
    """{piece: cm2 of its faces hung over its bed}: the pieces held to hang nothing - STANDING, or `names`."""
    return {name: told(name)['over'] for name in names}


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('names', nargs='*')
    args = parser.parse_args(argv)
    total = 0.0
    for name, _sketch, _parts, _way, count in prints.PIECES:
        if args.names and name not in args.names:
            continue
        got = told(name)
        total += count * got['g']
        print('%-16s x%d  %7.1f cm3  %7.1f g  faces %.2f  overhang %6.1f cm2' % (
            name, count, got['cm3'], got['g'], got['shell'], got['over']))
    print('a leg prints %.0f g' % total)
    return 0


if __name__ == '__main__':
    sys.exit(main())
