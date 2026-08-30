#!/usr/bin/env python3
"""Silhouette check: the vector renderer against the CAD exporter's own
ASCII renders at known rotations.

Every fixture in tests/renders/ is named ascii-x{X}y{Y}z{Z}.txt after
the rotation its exporter applied. The same rotation renders here with
tip 0 - the exporter looks straight down the camera axis - and the two
silhouettes compare scale- and position-free: each bounding box is
resampled onto a common grid, then intersection over union. 1.0 is a
perfect match; the flat-disc model cannot reach it edge-on, where the
exporter draws the real assembly's thickness and connectors.

    python tools/facecheck.py
"""
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import wireframe                              # noqa: E402
from coaxial.orientation import _qmul                      # noqa: E402

HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RENDERS = os.path.join(HOST, 'tests', 'renders')

_CUBE = None


def solid_for(name):
    """The mesh a fixture renders with: the exporter's cube for cube-*,
    the board (None, render's default) otherwise. The cube is the light
    model's proof - flat faces make a shading bug a wrong character."""
    global _CUBE
    if not name.startswith('cube-'):
        return None
    if _CUBE is None:
        # wireframe's in-memory decimate: nothing written beside the STL.
        _CUBE = wireframe._decimated(
            os.path.join(HOST, '..', 'render', 'models', 'cube.stl'), 400)
    return _CUBE


def euler(x, y, z, order):
    """The quaternion for body-frame turns applied in `order`."""
    def turn(axis, deg):
        h = math.radians(deg) / 2.0
        s = math.sin(h)
        return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(h))

    parts = {'x': turn((1, 0, 0), x), 'y': turn((0, 1, 0), y),
             'z': turn((0, 0, 1), z)}
    q = (0.0, 0.0, 0.0, 1.0)
    for name in order:
        q = _qmul(q, parts[name])
    return q


def occupancy(text):
    cells = set()
    for j, row in enumerate(text.splitlines()):
        for i, c in enumerate(row):
            if c != ' ':
                cells.add((i, j))
    return cells


def shades(text):
    """cell -> ink class, for the light-agreement measure."""
    field = {}
    for j, row in enumerate(text.splitlines()):
        for i, c in enumerate(row):
            if c != ' ':
                field[(i, j)] = 0 if c == '.' else 1
    return field


def agreement(mine, theirs, n=48):
    """How often the two renders agree about '.'-vs-':' where both draw
    ink - the LIGHT, measured, not the silhouette."""
    def gridded(field):
        if not field:
            return {}
        pts = [(x, 2 * y + k) for x, y in field for k in (0, 1)]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
        w, h = (x1 - x0 + 1), (y1 - y0 + 1)
        out = {}
        for (x, y), cls in field.items():
            for k in (0, 1):
                yy = 2 * y + k
                ax, bx = int((x - x0) * n / w), int((x - x0 + 1) * n / w)
                ay, by = int((yy - y0) * n / h), int((yy - y0 + 1) * n / h)
                for gx in range(ax, max(ax + 1, bx)):
                    for gy in range(ay, max(ay + 1, by)):
                        out[(gx, gy)] = cls
        return out

    a, b = gridded(mine), gridded(theirs)
    both = set(a) & set(b)
    if not both:
        return 0.0
    return sum(a[c] == b[c] for c in both) / float(len(both))


def resample(cells, n=48):
    """The silhouette on a common n*n grid, scale and position free.
    Art rows are half-height cells; doubling y first squares them.

    Each source cell fills its whole SPAN on the target grid, not just
    its origin point: a 9-column sliver stretched to 48 as points is a
    comb of every fifth column, and two combs of different pitch
    measured IoU 0.01 on identical shapes."""
    if not cells:
        return set()
    cells = set((x, 2 * y) for x, y in cells) | \
        set((x, 2 * y + 1) for x, y in cells)
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    w, h = (x1 - x0 + 1), (y1 - y0 + 1)
    out = set()
    for x, y in cells:
        ax = int((x - x0) * n / w)
        bx = int((x - x0 + 1) * n / w)
        ay = int((y - y0) * n / h)
        by = int((y - y0 + 1) * n / h)
        for gx in range(ax, max(ax + 1, bx)):
            for gy in range(ay, max(ay + 1, by)):
                out.add((gx, gy))
    return out


def iou(a, b):
    return len(a & b) / float(len(a | b) or 1)


def main():
    fixtures = sorted(f for f in os.listdir(RENDERS)
                      if re.match(r'(ascii|cube)-x\d+y\d+z\d+\.txt$', f))
    print('%-24s %7s %7s %7s' % ('rotation', 'xyz', 'zyx', 'light'))
    for name in fixtures:
        x, y, z = (int(v) for v in re.findall(r'\d+', name))
        with open(os.path.join(RENDERS, name), encoding='ascii') as f:
            text = f.read()
        theirs = resample(occupancy(text))
        scores, light = [], 0.0
        for order in ('xyz', 'zyx'):
            art = wireframe.render(euler(x, y, z, order), 100, 50,
                                   colour=False, 
                                   horizon=False, tip=0.0,
                                   solid=solid_for(name))
            scores.append(iou(resample(occupancy(art)), theirs))
            if order == 'xyz':
                light = agreement(shades(art), shades(text))
        print('%-24s %7.2f %7.2f %7.2f'
              % (name[:-4], scores[0], scores[1], light))
    return 0


if __name__ == '__main__':
    sys.exit(main())
