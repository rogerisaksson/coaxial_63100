"""The cast shadow's depth map: the scene from the light, rastered at once on arrays.

Texel for texel what the triangle-by-triangle raster cast (test_paint holds it to that
reference): the same projection, bounding boxes and barycentric weights in the same order, each
texel the nearest depth toward the light. 4.0 -> 0.3 ms a map at 56 x 56, a new map every
9-degree bin a turning board enters.
"""
import math


def cast(solid, m, light, size, extent):
    """(depths, size, extent, right, up, light): `solid` at rotation `m` seen orthographically
    along `light`, `size` texels a side over +-`extent` model units, -9 where nothing is."""
    from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
    pos, idx, _nrm = solid
    m0, m1, m2, m3, m4, m5, m6, m7, m8 = m
    lx, ly, lz = light
    rn = math.sqrt(lz * lz + lx * lx)
    right = (lz / rn, 0.0, -lx / rn)
    up = (ly * right[2] - lz * right[1],
          lz * right[0] - lx * right[2],
          lx * right[1] - ly * right[0])
    x, y, z = np.asarray(pos, float).reshape(-1, 3).T
    tx = m0 * x + m1 * y + m2 * z
    ty = m3 * x + m4 * y + m5 * z
    tz = m6 * x + m7 * y + m8 * z
    half = (size - 1) / 2.0
    la = ((tx * right[0] + ty * right[1] + tz * right[2]) / extent + 1.0) * half
    lb = ((tx * up[0] + ty * up[1] + tz * up[2]) / extent + 1.0) * half
    lc = tx * lx + ty * ly + tz * lz
    a, b, c = np.asarray(idx, np.int64).reshape(-1, 3).T
    x0, y0, x1, y1, x2, y2 = la[a], lb[a], la[b], lb[b], la[c], lb[c]
    da, db, dc = lc[a], lc[b], lc[c]
    area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
    flip = area < 0.0
    x1, x2 = np.where(flip, x2, x1), np.where(flip, x1, x2)
    y1, y2 = np.where(flip, y2, y1), np.where(flip, y1, y2)
    db, dc = np.where(flip, dc, db), np.where(flip, db, dc)
    area = np.abs(area)
    lo_x = np.maximum(0, np.minimum(np.minimum(x0, x1), x2).astype(np.int64))
    hi_x = np.minimum(size - 1, np.maximum(np.maximum(x0, x1), x2).astype(np.int64) + 1)
    lo_y = np.maximum(0, np.minimum(np.minimum(y0, y1), y2).astype(np.int64))
    hi_y = np.minimum(size - 1, np.maximum(np.maximum(y0, y1), y2).astype(np.int64) + 1)
    across = np.maximum(hi_x - lo_x + 1, 0)
    count = np.where(area != 0.0, across * np.maximum(hi_y - lo_y + 1, 0), 0)
    t = np.repeat(np.arange(len(area)), count)
    k = np.arange(int(count.sum())) - np.repeat(np.cumsum(count) - count, count)
    py = lo_y[t] + k // across[t]
    px = lo_x[t] + k % across[t]
    inv = 1.0 / area[t]
    w0 = (((x2 - x1)[t] * (py - y1[t])) - (y2 - y1)[t] * (px - x1[t])) * inv
    w1 = (((x0 - x2)[t] * (py - y2[t])) - (y0 - y2)[t] * (px - x2[t])) * inv
    w2 = 1.0 - w0 - w1
    inside = (w0 >= 0.0) & (w1 >= 0.0) & (w2 >= 0.0)
    t, w0, w1, w2 = t[inside], w0[inside], w1[inside], w2[inside]
    depths = np.full(size * size, -9.0)
    np.maximum.at(depths, (py[inside] * size + px[inside]),
                  w0 * da[t] + w1 * db[t] + w2 * dc[t])
    return depths, size, extent, right, up, (lx, ly, lz)
