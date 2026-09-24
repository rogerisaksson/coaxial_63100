"""The analytic oracle: the exporter's cube as rays against its slabs, no raster."""
import functools
import os

from coaxial.graphics import solids
from tools import REPO

CUBE_STL = os.path.join(REPO, 'render', 'models', 'cube.stl')
DISTANCE = 3.2


def oracle(q_m, cam, half, pivot, slope, floor):
    """The expected picture: rays against the cube's slabs, no raster."""
    m = q_m
    rows = []
    for py in range(cam['height']):
        v = (cam['cy'] - (py + 0.5)) / (cam['scale'] * 0.5)
        line = []
        for px in range(cam['width']):
            u = (px + 0.5 - cam['cx']) / cam['scale']
            ex, ey, ez = (m[6] * cam['distance'], m[7] * cam['distance'],
                          m[8] * cam['distance'])
            dx = m[0] * u + m[3] * v - m[6]
            dy = m[1] * u + m[4] * v - m[7]
            dz = m[2] * u + m[5] * v - m[8]
            t0, t1, ok = 0.0, 1e9, True
            for e, d, h in ((ex, dx, half[0]), (ey, dy, half[1]),
                            (ez, dz, half[2])):
                if abs(d) < 1e-12:
                    if abs(e) > h:
                        ok = False
                        break
                    continue
                ta, tb = (-h - e) / d, (h - e) / d
                if ta > tb:
                    ta, tb = tb, ta
                t0, t1 = max(t0, ta), min(t1, tb)
                if t0 > t1:
                    ok = False
                    break
            if not ok:
                line.append(0)
                continue
            level = pivot + slope * (cam['distance'] - t0) / cam['reach']
            if level < floor:
                level = floor
            level = 0.0 if level < 0.0 else (2.0 if level > 2.0
                                             else level)
            line.append(int(level + 0.5))
        rows.append(line)
    return rows


@functools.cache
def cube():
    """(solid, half extents): the exporter's cube decimated at 400, once a
    process."""
    got = solids._decimated(CUBE_STL, 400)
    pos = got[0]
    half = tuple(max(abs(pos[3 * i + k]) for i in range(len(pos) // 3))
                 for k in range(3))
    return got, half
