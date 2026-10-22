"""The outline and the silhouette edge laid at once on arrays.

Dot for dot what `lines._outline` and `lines._edge` lay one dot at a time (test_paint holds
them to it).
"""
import functools

from coaxial.graphics.creases import _outline_source
from coaxial.graphics.lines import (_EDGE, EDGE_HOLE_CELLS, OUTLINE_CELLS, OUTLINE_GRACE,
                                    OUTLINE_MIN_EDGE, OUTLINE_SLOPE, STEREO_FACING)
from coaxial.graphics.paint import edge_tone
from coaxial.graphics.raster import BRAILLE, BRAILLE_BITS
from coaxial.graphics.shading import OUTLINE_BASE
from coaxial.graphics.solids import _slab_planes
from coaxial.graphics.stereotype import _drum_segments, _stereotypes


@functools.cache
def _tables():
    """BRAILLE_BITS by (column, row) and the edge's eight direction tables, as arrays."""
    from coaxial.model.blocks import numpy as np
    return np.array(BRAILLE_BITS, np.int64), np.array(_EDGE, np.int64)


def _segments(m, camx, camy, min_extent):
    """[(x0, y0, z0, x1, y1, z1)] of every primitive `lines._outline` draws at `m`."""
    out = []
    for kind, extent, data in _stereotypes():
        if extent < min_extent:
            continue
        if kind == 'drum':
            out += _drum_segments(data, camx, camy)
        elif kind in ('block', 'ring', 'stroke'):
            out += data
        else:
            segs, (nx, ny) = data
            if not abs(nx * m[6] + ny * m[7]) < STEREO_FACING:
                out += segs
    return out


def _project(m, cam, x, y, z):
    """`lines._outline`'s project on arrays: screen x, y and the perspective weight."""
    m0, m1, m2, m3, m4, m5, m6, m7, m8 = m
    scale, cx, cy, distance = cam['scale'], cam['cx'], cam['cy'], cam['distance']
    w = 1.0 / (distance - (m6 * x + m7 * y + m8 * z))
    return (cx + scale * w * (m0 * x + m1 * y + m2 * z),
            cy - scale * 0.5 * w * (m3 * x + m4 * y + m5 * z), w)


def _traced(x0, y0, w0, x1, y1, w1, slope):
    """`lines._trace` over every segment at once: (fx, fy, w, slope) at each crossing of the
    braille matrix's sub-columns (flatter than its aspect) or sub-rows (steeper)."""
    from coaxial.model.blocks import numpy as np
    dx, dy, dw = x1 - x0, y1 - y0, w1 - w0
    flat = np.abs(dx) >= 2.0 * np.abs(dy)
    drawn = ~(flat & (dx == 0.0))
    pitch = np.where(flat, 2.0, 4.0)
    a, b = np.where(flat, x0, y0), np.where(flat, x1, y1)
    first = np.ceil(pitch * np.minimum(a, b) - 0.5)
    count = np.where(drawn, np.maximum(np.floor(pitch * np.maximum(a, b) - 0.5) - first + 1.0,
                                       0.0), 0.0).astype(np.int64)
    s = np.repeat(np.arange(len(dx)), count)
    k = (np.repeat(first, count) + np.arange(int(count.sum()))
         - np.repeat(np.cumsum(count) - count, count))
    along = (k + 0.5) / pitch[s]
    t = (along - a[s]) / np.where(flat, dx, dy)[s]
    fx = np.where(flat[s], along, x0[s] + dx[s] * t)
    fy = np.where(flat[s], y0[s] + dy[s] * t, along)
    return fx, fy, w0[s] + dw[s] * t, slope[s]


def _span(buf, width):
    """Each cell's depth span: the largest difference to its four flat neighbours' depths, the
    grace an outline's dot gets there."""
    from coaxial.model.blocks import numpy as np
    n = len(buf)
    inv = np.divide(1.0, buf, out=np.zeros(n), where=buf != 0.0)
    span = np.zeros(n)
    cell = np.arange(n)
    for off in (-1, 1, -width, width):
        j = cell + off
        ok = (j >= 0) & (j < n)
        j = np.clip(j, 0, n - 1)
        span = np.maximum(span, np.where(ok & (buf[j] != 0.0), np.abs(inv[j] - inv), 0.0))
    return span


def _ink(glyph, tone, at, mask, colour, heat):
    """`lines._ink`: `mask` over each cell's own dots, in the edge tone."""
    from coaxial.model.blocks import numpy as np
    was = glyph[at]
    mask = np.where((was >= BRAILLE) & (was < BRAILLE + 256), mask | (was - BRAILLE), mask)
    glyph[at] = BRAILLE + mask
    if colour:
        base = heat[at] if heat is not None else np.zeros(len(at))
        tone[at] = edge_tone(np.where(base != 0.0, base, OUTLINE_BASE))


def outline(glyph, tone, buf, cam, m, colour, heat):
    """`lines._outline`: every part wide enough to read in dotted lines, hidden behind the
    solid within the grace, a lone cell no line."""
    from coaxial.model.blocks import numpy as np
    width, height = cam['width'], cam['height']
    distance = cam['distance']
    top, bottom = _slab_planes(_outline_source()[0])
    camz = distance * m[8]
    under = bottom is not None and camz < bottom
    over = bottom is not None and camz > top
    mid = top if bottom is None else 0.5 * (top + bottom)
    segs = _segments(m, distance * m[6], distance * m[7],
                     OUTLINE_CELLS / (cam['scale'] / distance))
    if not segs:
        return
    x0, y0, z0, x1, y1, z1 = np.asarray(segs, float).T
    sx0, sy0, wa = _project(m, cam, x0, y0, z0)
    sx1, sy1, wb = _project(m, cam, x1, y1, z1)
    long = ~(np.maximum(np.abs(sx1 - sx0), 2.0 * np.abs(sy1 - sy0)) < OUTLINE_MIN_EDGE)
    slope = np.where(np.where(z0 + z1 > 2.0 * mid, under, over), 0.0, OUTLINE_SLOPE)
    fx, fy, we, slope = _traced(sx0[long], sy0[long], wa[long], sx1[long], sy1[long], wb[long],
                                slope[long])
    inside = ~((fx < 0.0) | (fy < 0.0))
    fx, fy, we, slope = fx[inside], fy[inside], we[inside], slope[inside]
    px, py = fx.astype(np.int64), fy.astype(np.int64)
    inside = (px < width) & (py < height)
    fx, fy, we, slope, px, py = (v[inside] for v in (fx, fy, we, slope, px, py))
    at = py * width + px
    near = buf[at]
    tested = (near != 0.0) & (we > 0.0)
    here = 1.0 / np.where(tested, near, 1.0)
    far = 1.0 / np.where(tested, we, 1.0) - here
    shown = ~(tested & (far > OUTLINE_GRACE + _span(buf, width)[at] * slope))
    bits, _edge_tables = _tables()
    col = (fx - px >= 0.5).astype(np.int64)
    row = np.minimum(3, ((fy - py) * 4.0).astype(np.int64))
    masks = np.zeros(width * height, np.int64)
    np.bitwise_or.at(masks, at[shown], bits[col[shown], row[shown]])
    marked = (masks != 0).reshape(height, width)
    ring = np.pad(marked, 1)
    neighbours = sum(ring[1 + dr:1 + dr + height, 1 + dc:1 + dc + width]
                     for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc)
    kept = np.flatnonzero((marked & (neighbours > 0)).ravel())
    _ink(glyph, tone, kept, masks[kept], colour, heat)


def _runs(lit, open_):
    """`lit` spread along each row's runs of `open_` cells: a run lit anywhere is lit whole."""
    from coaxial.model.blocks import numpy as np
    starts = open_.copy()
    starts[:, 1:] &= ~open_[:, :-1]
    run = np.cumsum(starts.ravel()) - 1
    flat_open, flat_lit = open_.ravel(), lit.ravel()
    within = run[flat_open]
    lit_runs = np.bincount(within, weights=flat_lit[flat_open], minlength=int(run[-1]) + 1) > 0
    out = np.zeros(open_.size, bool)
    out[flat_open] = lit_runs[within]
    return out.reshape(open_.shape)


def _regions(reached, width, height):
    """`lines._regions`: 0 covered, 2 empty and joined to the frame's edge or to a hole of
    EDGE_HOLE_CELLS or more, 3 a pinhole - the exterior spread along rows and columns till it
    holds, the few holes inside flooded one by one."""
    from coaxial.model.blocks import numpy as np
    empty = (reached == 0).reshape(height, width)
    out = np.zeros((height, width), bool)
    out[[0, -1], :] = empty[[0, -1], :]
    out[:, [0, -1]] |= empty[:, [0, -1]]
    while True:
        spread = _runs(_runs(out, empty).T.copy(), empty.T.copy()).T
        if (spread == out).all():
            break
        out = spread
    region = np.where(empty, 2, 0).ravel()
    inner = set(np.flatnonzero((empty & ~out).ravel()).tolist())
    for start in sorted(inner):
        if start not in inner:
            continue
        comp, k = [start], 0
        inner.discard(start)
        while k < len(comp):
            i = comp[k]
            k += 1
            r, c = divmod(i, width)
            for j, ok in ((i - 1, c > 0), (i + 1, c < width - 1), (i - width, r > 0),
                          (i + width, r < height - 1)):
                if ok and j in inner:
                    inner.discard(j)
                    comp.append(j)
        if len(comp) < EDGE_HOLE_CELLS:
            region[comp] = 3
    return region


def edge(glyph, tone, reached, cam, colour, heat):
    """`lines._edge`: the covered dots that border the exterior or a hole, off the fold's
    reached bits, in the edge tone."""
    from coaxial.model.blocks import numpy as np
    width, height = cam['width'], cam['height']
    if heat is not None:
        reached = np.where(heat != 0.0, reached, 0)
    region = _regions(reached, width, height)
    kept = np.pad((region == 2).reshape(height, width), 1)
    near = np.zeros((height, width), bool)
    for dr in (0, 1, 2):
        for dc in (0, 1, 2):
            near |= kept[dr:dr + height, dc:dc + width]
    at = np.flatnonzero((reached != 0) & near.ravel())
    r, c = np.divmod(at, width)
    bits = reached[at]
    n = width * height
    _bits, (left, right, up, down, from_left, from_right, from_up, from_down) = _tables()
    beside = []
    for ok, j in ((c > 0, at - 1), (c < width - 1, at + 1), (r > 0, at - width),
                  (r < height - 1, at + width)):
        j = np.clip(j, 0, n - 1)
        beside.append(np.where(ok & (region[j] != 3), reached[j], 0xFF))
    mask = bits & (left[bits] | right[bits] | up[bits] | down[bits] | from_left[beside[0]]
                   | from_right[beside[1]] | from_up[beside[2]] | from_down[beside[3]])
    drawn = mask != 0
    _ink(glyph, tone, at[drawn], mask[drawn], colour, heat)
