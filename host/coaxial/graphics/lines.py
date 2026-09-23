"""Lines on the braille grid: segments, the stereotype outline and the slab's rim.

`_trace` puts a segment's dots on the matrix's own sub-columns and sub-rows;
`_outline` draws the stereotype primitives with the hidden-line test against
the depth buffer; `_edge` draws the slab's rim and holes as the raster's own
silhouette, off the fold's reached bits.
"""
import math

from .raster import BRAILLE, BRAILLE_BITS
from .shading import OUTLINE_BASE, _edge_tone
from .creases import _outline_source
from .solids import _slab_planes
from .stereotype import _drum_segments, _stereotypes


#: An edge shorter than this on SCREEN, in cells, is not drawn: a QFP's
#: hundred pin creases and a can's facets are a third of a cell each at
#: the view's zoom, and stacking their dots in one cell drew a solid
#: braille blob where the part was. Measured over the 90 loops wider
#: than 0.06 units: the blobs had 24-190 edges of 0.3-0.6 mm, the
#: outlines edges of millimetres. Judged per EDGE and per FRAME rather
#: than per loop, because a loop's total length could not tell a box
#: with its footprint (twice the perimeter) from the pins - and because
#: it is the zoom that decides: bring a can up and its facets draw.
OUTLINE_MIN_EDGE = 0.5


#: How far behind the frame's surface an edge may sit and still draw,
#: in model units: the raster's own error across a cell is under half
#: a millimetre at any view size, and the slab is 1.6 mm - so 0.6 mm
#: keeps an edge on its face and hides a top-side part seen from below.
#: A relative 2 % grace, the wire mode's, let 1.0 % of slab through.
OUTLINE_GRACE = 0.012


#: The share of a cell's own depth span - the largest difference to
#: its four neighbours' nearest samples - added to the grace: one, the
#: span itself, since a lid's edge and the lid's near corner are that
#: far apart in one cell. Zero is the fixed grace alone.
OUTLINE_SLOPE = 1.0


#: Two, from five by way of three, on the bench's word - "more of the
#: edge enhancer, also on somewhat smaller objects", then "even smaller
#: components highlighted". Counted at the view's zoom (46.8 cells per
#: unit): five drew 43 loops, three 65, two 95 - a part 2.1 mm wide is
#: the smallest now, an 0805 on its edge; 1.5 would draw 112 and one
#: 167, into the 0603s and the chip-scale parts, which OUTLINE_MIN_EDGE
#: would strip to a dot or two each.
OUTLINE_CELLS = 2


#: A wall's arch or hole draws only where the wall faces the camera by
#: this much (the wall's normal against the view's z axis, in the
#: plane): under it the feature is seen edge-on and would be a dash.
STEREO_FACING = 0.25


def _trace(x0, y0, w0, x1, y1, w1, dot):
    """`dot(fx, fy, w)` wherever the segment crosses one of the braille
    matrix's own lines: the sub-columns at half-cell pitch when it runs
    flatter than the matrix's aspect, the sub-rows at quarter pitch when
    steeper.
    """
    dx, dy, dw = x1 - x0, y1 - y0, w1 - w0
    if abs(dx) >= 2.0 * abs(dy) and dx == 0.0:
        return                                # a point, not a line
    if abs(dx) >= 2.0 * abs(dy):
        lo, hi = (x0, x1) if x0 < x1 else (x1, x0)
        for k in range(int(math.ceil(2.0 * lo - 0.5)),
                       int(math.floor(2.0 * hi - 0.5)) + 1):
            fx = (k + 0.5) / 2.0
            t = (fx - x0) / dx
            dot(fx, y0 + dy * t, w0 + dw * t)
    else:
        lo, hi = (y0, y1) if y0 < y1 else (y1, y0)
        for j in range(int(math.ceil(4.0 * lo - 0.5)),
                       int(math.floor(4.0 * hi - 0.5)) + 1):
            fy = (j + 0.5) / 4.0
            t = (fy - y0) / dy
            dot(x0 + dx * t, fy, w0 + dw * t)


def _outline(grid, tone, buf, cam, m, colour, heat=None):
    """The wireframe overlay: every part wide enough to read, as the
    primitive its loops fit (`_stereotypes` - a block, a drum, an arch),
    in dotted lines on the cells' 2x4 braille matrix, hidden where the
    solid stands in front - the grace keeps an edge from losing to the
    face it borders.
    """
    prims = _stereotypes()
    width, height = cam['width'], cam['height']
    scale, cx, cy, distance = cam['scale'], cam['cx'], cam['cy'], cam['distance']
    min_extent = OUTLINE_CELLS / (scale / distance)
    m0, m1, m2, m3, m4, m5, m6, m7, m8 = m
    n_cells = width * height
    masks = {}
    # FAR SIDE: an edge on the slab's other face from the camera is behind
    # 1.6 mm of board, and a tilted face's own span (0.04-0.08 a cell, face
    # down) must not lend it grace - the top's parts printed through.
    top, bottom = _slab_planes(_outline_source()[0])
    camz = distance * m8
    under = bottom is not None and camz < bottom
    over = bottom is not None and camz > top
    mid = top if bottom is None else 0.5 * (top + bottom)
    slope = OUTLINE_SLOPE

    def project(x, y, z):
        tz = m6 * x + m7 * y + m8 * z
        w = 1.0 / (distance - tz)
        return (cx + scale * w * (m0 * x + m1 * y + m2 * z),
                cy - scale * 0.5 * w * (m3 * x + m4 * y + m5 * z), w)

    def dot(fx, fy, we):
        if fx < 0.0 or fy < 0.0:
            return
        px, py = int(fx), int(fy)
        if px >= width or py >= height:
            return
        at = py * width + px
        near = buf[at]
        if near and we > 0.0:
            # THE GRACE FOLLOWS THE CELL'S OWN DEPTH SPAN.
            here = 1.0 / near
            span = 0.0
            for j in (at - 1, at + 1, at - width, at + width):
                if 0 <= j < n_cells and buf[j]:
                    d = 1.0 / buf[j] - here
                    if d < 0.0:
                        d = -d
                    if d > span:
                        span = d
            if 1.0 / we - here > OUTLINE_GRACE + span * slope:
                return                                # behind the surface
        col = 1 if fx - px >= 0.5 else 0
        row = min(3, int((fy - py) * 4.0))
        masks[at] = masks.get(at, 0) | BRAILLE_BITS[col][row]

    def segment(x0, y0, z0, x1, y1, z1):
        nonlocal slope
        sx0, sy0, wa = project(x0, y0, z0)
        sx1, sy1, wb = project(x1, y1, z1)
        if max(abs(sx1 - sx0), 2.0 * abs(sy1 - sy0)) < OUTLINE_MIN_EDGE:
            return                          # sub-pixel detail, see above
        slope = 0.0 if (under if z0 + z1 > 2.0 * mid else over) else OUTLINE_SLOPE
        _trace(sx0, sy0, wa, sx1, sy1, wb, dot)

    # The camera in model space, for the drums' silhouettes: the view's z axis
    # is m's third row, and the camera sits `distance` along it.
    camx, camy = distance * m6, distance * m7
    for kind, extent, data in prims:
        if extent < min_extent:
            continue
        if kind == 'drum':
            segs = _drum_segments(data, camx, camy)
        elif kind in ('block', 'ring', 'stroke'):
            segs = data
        else:
            # A wall's feature - an arch, a hole - seen edge-on is a dash:
            # face-on, the screw terminals' openings lay as short bright
            # strokes along the rim, "junk" on the bench's screenshot.
            segs, (nx, ny) = data
            if abs(nx * m6 + ny * m7) < STEREO_FACING:
                continue
        for s in segs:
            segment(*s)
    # Strays: a line is a chain of neighbouring cells, so a cell with no drawn
    # neighbour in its eight is a sample that cleared the depth test alone - a
    # grazing edge, a corner half behind a wall - and not a line.
    lone = [at for at in masks
            if not any((at + dr * width + dc) in masks
                       for dr in (-1, 0, 1) for dc in (-1, 0, 1)
                       if (dr or dc)
                       and 0 <= at % width + dc < width
                       and 0 <= at // width + dr < height)]
    for at in lone:
        del masks[at]
    return _ink(grid, tone, masks, width, colour, heat)


def _ink(grid, tone, masks, width, colour, heat):
    """`masks` ONTO the face's dots - a cell keeps its own dither under the
    line's - in the edge tone, lit by `heat` where the glow left any. The
    count of cells inked."""
    for at, mask in masks.items():
        r, c = divmod(at, width)
        was = grid[r][c]
        if BRAILLE <= ord(was) < BRAILLE + 256:
            mask |= ord(was) - BRAILLE
        grid[r][c] = chr(BRAILLE + mask)
        if colour:
            tone[r][c] = _edge_tone(heat[at] if heat is not None and heat[at]
                                    else OUTLINE_BASE)
    return len(masks)


#: Fewer empty cells than this inside the face are a pinhole - the
#: decimate's, or a dot the fold missed - not a hole with an edge.
EDGE_HOLE_CELLS = 3


def _edge_tables():
    """Per braille mask, one table a direction: the set dots whose in-cell
    neighbour that way is unset.
    """
    bits = BRAILLE_BITS
    left, right, up, down = [0] * 256, [0] * 256, [0] * 256, [0] * 256
    from_left, from_right = [0] * 256, [0] * 256
    from_up, from_down = [0] * 256, [0] * 256
    for mask in range(256):
        for lane in range(2):
            for row in range(4):
                bit = bits[lane][row]
                if not mask & bit:
                    continue
                if lane == 1 and not mask & bits[0][row]:
                    left[mask] |= bit
                if lane == 0 and not mask & bits[1][row]:
                    right[mask] |= bit
                if row > 0 and not mask & bits[lane][row - 1]:
                    up[mask] |= bit
                if row < 3 and not mask & bits[lane][row + 1]:
                    down[mask] |= bit
        for row in range(4):
            if not mask & bits[1][row]:
                from_left[mask] |= bits[0][row]
            if not mask & bits[0][row]:
                from_right[mask] |= bits[1][row]
        for lane in range(2):
            if not mask & bits[lane][3]:
                from_up[mask] |= bits[lane][0]
            if not mask & bits[lane][0]:
                from_down[mask] |= bits[lane][3]
    return left, right, up, down, from_left, from_right, from_up, from_down


_EDGE = _edge_tables()


def _regions(reached, width, height):
    """Per cell: 0 covered, 2 empty and joined to the frame's edge or to
    a hole of EDGE_HOLE_CELLS or more, 3 a pinhole."""
    n = width * height
    region = [0 if reached[i] else 1 for i in range(n)]

    def flood(start):
        comp = [start]
        region[start] = 2
        k = 0
        while k < len(comp):
            i = comp[k]
            k += 1
            r, c = divmod(i, width)
            for j in ((i - 1) if c else -1, (i + 1) if c < width - 1 else -1,
                      (i - width) if r else -1, (i + width) if r < height - 1 else -1):
                if j >= 0 and region[j] == 1:
                    region[j] = 2
                    comp.append(j)
        return comp

    border = (list(range(width)) + list(range(n - width, n))
              + list(range(0, n, width)) + list(range(width - 1, n, width)))
    for i in border:
        if region[i] == 1:
            flood(i)
    for i in range(n):
        if region[i] == 1:
            comp = flood(i)
            if len(comp) < EDGE_HOLE_CELLS:
                for j in comp:
                    region[j] = 3
    return region


def _edge(grid, tone, cells, cam, colour, heat=None):
    """The slab's edge and its holes as the RASTER'S OWN SILHOUETTE: the
    covered dots that border what the fold left empty - the exterior, and
    any hole of EDGE_HOLE_CELLS empty cells or more - at braille dot
    resolution, off the fold's `reached` bits and the tables above.
    """
    width, height = cam['width'], cam['height']
    reached = cells[2]
    if heat is not None:
        reached = [bits if heat[at] else 0 for at, bits in enumerate(reached)]
    region = _regions(reached, width, height)
    kept = [x == 2 for x in region]
    left, right, up, down, from_left, from_right, from_up, from_down = _EDGE
    full = 0xFF
    n = width * height
    masks = {}
    for i in range(n):
        bits = reached[i]
        if not bits:
            continue
        r, c = divmod(i, width)
        near = False
        for rr in (r - 1, r, r + 1):
            if 0 <= rr < height:
                base = rr * width
                for cc in (c - 1, c, c + 1):
                    if 0 <= cc < width and kept[base + cc]:
                        near = True
                        break
            if near:
                break
        if not near:
            continue
        # A pinhole beside us is no edge: seen as full.
        beside = [full, full, full, full]
        for k, (ok, j) in enumerate(((c > 0, i - 1), (c < width - 1, i + 1),
                                     (r > 0, i - width), (r < height - 1, i + width))):
            if ok and region[j] != 3:
                beside[k] = reached[j]
        mask = bits & (left[bits] | right[bits] | up[bits] | down[bits]
                       | from_left[beside[0]] | from_right[beside[1]]
                       | from_up[beside[2]] | from_down[beside[3]])
        if mask:
            masks[i] = mask
    return _ink(grid, tone, masks, width, colour, heat)
