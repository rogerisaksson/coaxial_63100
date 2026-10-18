"""Her world's props drawn as lines over her.

A box each - a sill, a chair, a wall -, a lace from shoe to shoe, a bed's mattress a grid sagging
where she presses it.

    layers = props.lines(props, m, cam, centre, travel, dents)   # [(dots, ink)] for `lit.braille`
"""
from coaxial.graphics.callouts import line
from coaxial.graphics.lit import project

PROP_INK = {'hole': (255, 96, 128), 'sill': (255, 184, 80), 'slip': (96, 214, 255),
            'rug': (200, 160, 110), 'lace': (230, 90, 230), 'stairs': (220, 225, 235),
            'brick': (205, 92, 70), 'board': (214, 178, 120), 'chair': (220, 165, 80),
            'bed': (130, 160, 230), 'slope': (120, 210, 140), 'ice': (170, 235, 255),
            'snow': (240, 245, 255), 'wall': (180, 180, 195), 'tree': (90, 170, 90),
            'glass': (110, 160, 210), 'wood': (176, 120, 70),
            'door': (205, 150, 90), 'lamp': (110, 100, 80), 'lit': (255, 220, 120),
            'switch': (200, 200, 200)}

_BOX_SIGNS = tuple((float(i), float(j), float(k)) for i in (-1, 1) for j in (-1, 1) for k in (-1, 1))
_BOX_EDGES = tuple((a, b) for a in range(8) for b in range(a + 1, 8) if bin(a ^ b).count('1') == 1)
#: A mattress's top drawn GRID lines each way, its give spread SAG_M about each press: a foam's.
GRID, SAG_M = 9, 0.12


def _sagged(pos, half, turn, dents, shift):
    """(corners, edges) of a box whose top is a grid sagging over `dents` [(x, y, z, depth)], its
    sides and bottom as a box's."""
    from coaxial.model.blocks import numpy as np
    rot, centre = np.asarray(turn, float), np.asarray(pos, float)
    u = np.linspace(-1.0, 1.0, GRID)
    local = np.array([(a * half[0], half[1], b * half[2]) for a in u for b in u])
    top = local @ rot.T + centre
    if dents:
        press = np.asarray(dents, float)
        near = np.exp(-((top[:, None, 0] - press[None, :, 0]) ** 2 + (top[:, None, 2] - press[None, :, 2]) ** 2)
                      / (2.0 * SAG_M ** 2))
        top[:, 1] -= (near * press[None, :, 3]).max(axis=1)
    box = (np.asarray(_BOX_SIGNS) * np.asarray(half)) @ rot.T + centre
    corners = np.vstack([top, box]) - shift
    n = GRID * GRID
    edges = ([(i * GRID + j, i * GRID + j + 1) for i in range(GRID) for j in range(GRID - 1)]
             + [(i * GRID + j, (i + 1) * GRID + j) for i in range(GRID - 1) for j in range(GRID)]
             + [(n + a, n + b) for a, b in _BOX_EDGES if _BOX_SIGNS[a][1] < 0.0 or _BOX_SIGNS[b][1] < 0.0])
    return corners, edges


def lines(props, m, cam, centre, travel, dents=None):
    """[(dots, ink)] of `props` (`floor.props`) seen by `cam`, a bed's top sagging over `dents`."""
    if not props:
        return []
    from coaxial.model.blocks import numpy as np
    signs, by_ink, shift = np.asarray(_BOX_SIGNS), {}, (travel[0], 0.0, travel[1])
    for kind, *shape in props:
        if kind == 'lace':
            corners, edges = np.array(shape, float) - shift, ((0, 1),)
        elif kind == 'bed' and min(shape[1][0], shape[1][2]) > 0.1:
            corners, edges = _sagged(shape[0], shape[1], shape[2], dents or (), np.asarray(shift))
        else:
            pos, half, turn = shape
            corners = (signs * np.asarray(half)) @ np.asarray(turn, float).T + (
                pos[0] - travel[0], pos[1], pos[2] - travel[1])
            edges = _BOX_EDGES
        sx, sy, w = project(corners, m, cam, centre)
        if (w <= 0.0).all():
            continue
        ink = PROP_INK.get(kind, (200, 200, 200))
        dots = by_ink.get(ink)
        if dots is None:
            dots = by_ink[ink] = np.zeros((cam['height'], cam['width']), bool)
        for a, b in edges:
            if w[a] > 0.0 and w[b] > 0.0:
                line(dots, (sx[a], sy[a]), (sx[b], sy[b]))
    return [(dots, ink) for ink, dots in by_ink.items()]
