"""three.js AsciiEffect, ported to run without a browser."""
import math

from machine import ansi

#: AsciiEffect's default ramp, darkest first.
CHARACTERS = ' .:-+*=%@#'

#: The key light's direction, in the board's own frame.
#:
#: `updateLightPosition` puts it at `(cos45 * bbox.max.z * 2, (bbox.max.y -
#: bbox.min.y) * 2, sin45 * bbox.max.z * 2)`, which for this model - 100 x
#: 100 x 51 mm - is world (35.9, 200, 35.9). Undoing the reference's own
#: `rotation.x = -90`, which it applies to every STL, puts it here.
LIGHT_DIRECTION = (35.9, -35.9, 200.0)

#: How far along it the light sits, in board radii: 206 mm over a 50 mm
#: radius, from the same position.
LIGHT_DISTANCE = 4.12

#: How much taller a character cell is than it is wide, in the font the
#: drawing will be read in. The renderer works in square pixels and folds
#: this in at the end, so a round board comes out round.
#:
#: 2.0 is the usual terminal monospace - Consolas, Cascadia Mono, DejaVu Sans
#: Mono and Menlo are all within a few percent of it at any size. Getting it
#: wrong does not blur the picture, it stretches it: at 1.6 a disc is drawn
#: 25% wide of round, which reads as the board being turned when it is not.
CELL_ASPECT = 2.0

#: Framebuffer samples per character cell edge. The GPU hands AsciiEffect an
#: already-rasterised canvas and it reads one pixel per cell; this has to do
#: the rasterising too, so it can render finer and average down.
#:
#: 1, because spending the pixels on CHARACTERS beats spending them inside
#: one. Measured, same board and light: 140x40 at 2 costs 284 ms for 5,600
#: characters; 200x56 at 1 costs 243 ms for 11,200. Supersampling smooths a
#: cell that a smaller font would have drawn properly.
SUPERSAMPLE = 1

#: The reference camera's field of view, in degrees.
FOV_DEGREES = 45.0

#: Why back-face culling is off, though it is worth 40% of a frame: this
#: board's STL is a CAD assembly, not a solid. Measured, 49% of its faces
#: have their normal pointing back toward the middle, and culling changed 39%
#: of the characters - it leaves no holes, it just shows whatever is behind
#: an inside-out face instead of the face itself. DoubleSide cannot be wrong
#: that way, which is why the reference uses it. Pass cull=True for a mesh
#: you know is closed and consistently wound.
CULLING = False

#: Bisection steps for the camera fit. Twenty halvings settle it well inside
#: one character.
FIT_HALVINGS = 20

#: One vertex in this many is enough to fit the camera to the model.
FIT_STEP = 12

#: Why `rasterise` asks whether a triangle touches its band before it asks
#: anything else: every worker walks the whole model, so whatever is done
#: before that question is paid for by every worker for every triangle,
#: however far it is from that worker's rows. Measured at 150x44 - asking it
#: last, after the winding, the area and the x bounds, cost 23 ms of each
#: worker's 31 ms of fixed overhead and held the parallel ceiling at 3.5x.
#: Asked first, the fixed cost is 14.5 ms and the ceiling is 6x.
BAND_FIRST = True


def _unit(vector):
    x, y, z = vector
    length = math.sqrt(x * x + y * y + z * z) or 1.0
    return (x / length, y / length, z / length)


def light_position():
    """Where the key light sits, in the model's own units."""
    x, y, z = _unit(LIGHT_DIRECTION)
    return (x * LIGHT_DISTANCE, y * LIGHT_DISTANCE, z * LIGHT_DISTANCE)


def brightness_char(brightness, ramp=CHARACTERS, invert=True):
    """AsciiEffect's mapping, including its inversion for dark mode."""
    index = int((1.0 - max(0.0, min(1.0, brightness))) * (len(ramp) - 1))

    if invert:
        index = len(ramp) - 1 - index
    return ramp[index]


def grid(width, height, supersample=SUPERSAMPLE, aspect=CELL_ASPECT):
    """(framebuffer columns, rows, rows per character cell) for a character
    grid.
    """
    cell_rows = max(1, int(round(aspect * supersample)))
    return width * supersample, height * cell_rows, cell_rows


def shade_of(normal, point, light, eye):
    """Lambert from a point light with no falloff, clamped at zero."""
    nx, ny, nz = normal
    ex, ey, ez = _unit((eye[0] - point[0], eye[1] - point[1],
                        eye[2] - point[2]))
    if nx * ex + ny * ey + nz * ez < 0.0:
        nx, ny, nz = -nx, -ny, -nz

    lx, ly, lz = _unit((light[0] - point[0], light[1] - point[1],
                        light[2] - point[2]))
    lambert = nx * lx + ny * ly + nz * lz
    return lambert if lambert > 0.0 else 0.0


def fit(verts, matrix, cols, rows, step=FIT_STEP, zoom=1.0):
    """(distance, x offset, y offset) that put the model in frame, filling
    it.
    """
    m0, m1, m2, m3, m4, m5, m6, m7, m8 = matrix
    scale = (rows / 2.0) / math.tan(math.radians(FOV_DEGREES) / 2.0)

    turned = []
    for i in range(0, len(verts) - 2, 3 * step):
        px, py, pz = verts[i], verts[i + 1], verts[i + 2]
        turned.append((m0 * px + m1 * py + m2 * pz,
                       m3 * px + m4 * py + m5 * pz,
                       m6 * px + m7 * py + m8 * pz))

    if not turned:
        return 3.0, 0.0, 0.0

    near = max(z for _x, _y, z in turned)
    reach = max(abs(x) for x, _y, _z in turned) or 1.0
    wide, tall = cols * zoom, rows * zoom

    def frame(distance):
        low_x = low_y = 1e30
        high_x = high_y = -1e30
        for x, y, z in turned:
            ooz = 1.0 / (distance - z)
            sx, sy = scale * ooz * x, scale * ooz * y
            if sx < low_x:
                low_x = sx
            if sx > high_x:
                high_x = sx
            if sy < low_y:
                low_y = sy
            if sy > high_y:
                high_y = sy
        return (max((high_x - low_x) / wide, (high_y - low_y) / tall),
                (high_x + low_x) / 2.0, (high_y + low_y) / 2.0)

    low, high = near + 1e-3, near + scale * reach * 4.0
    for _ in range(FIT_HALVINGS):
        middle = (low + high) / 2.0
        if frame(middle)[0] > 1.0:
            low = middle                 # too big for the frame: stand back
        else:
            high = middle

    _over, mid_x, mid_y = frame(high)
    return high, -mid_x, -mid_y


def _setup(model, matrix, width, height, distance, zoom, supersample,
           aspect, centre):
    """Everything render() settles before it starts filling pixels."""
    cols, rows, cell_rows = grid(width, height, supersample, aspect)

    if distance is None:
        distance = fit(model[0], matrix, cols, rows, zoom=zoom)[0]

    scale = (rows / 2.0) / math.tan(math.radians(FOV_DEGREES) / 2.0)
    cx, cy = (cols - 1) / 2.0, (rows - 1) / 2.0
    if centre:
        cx, cy = cx + centre[0], cy - centre[1]

    return cols, rows, cell_rows, distance, scale, cx, cy


def _project(positions, matrix, distance, scale, cx, cy):
    """Every distinct vertex, turned and projected, once."""
    m0, m1, m2, m3, m4, m5, m6, m7, m8 = matrix
    count = len(positions) // 3
    sx = [0.0] * count
    sy = [0.0] * count
    ooz = [0.0] * count
    vx = [0.0] * count
    vy = [0.0] * count
    vz = [0.0] * count

    for v in range(count):
        i = v + v + v
        px, py, pz = positions[i], positions[i + 1], positions[i + 2]
        z = m6 * px + m7 * py + m8 * pz
        behind = distance - z
        if behind <= 1e-6:
            continue                    # through the camera, or past it

        x = m0 * px + m1 * py + m2 * pz
        y = m3 * px + m4 * py + m5 * pz
        near = 1.0 / behind
        ooz[v], vx[v], vy[v], vz[v] = near, x, y, z
        sx[v] = cx + scale * near * x
        sy[v] = cy - scale * near * y

    # The view-space positions come back too: the light is a point light, so
    # its direction changes across the model, and a face has to be shaded where
    # it actually is.
    return sx, sy, ooz, vx, vy, vz


def rasterise(model, matrix, distance, scale, cx, cy, cols, top, bottom,
              lamp, cull=CULLING, tints=None, who=None):
    """(depth, value) for framebuffer rows `top` up to `bottom`."""
    positions, indices, normals = model
    m0, m1, m2, m3, m4, m5, m6, m7, m8 = matrix
    eye = (0.0, 0.0, distance)

    sx, sy, ooz, vx, vy, vz = _project(positions, matrix, distance, scale,
                                       cx, cy)
    deep = (bottom - top) * cols
    depth = [0.0] * deep
    value = [0.0] * deep

    for tri in range(len(indices) // 3):
        t = tri + tri + tri
        a, b, c = indices[t], indices[t + 1], indices[t + 2]

        y0, y1, y2 = sy[a], sy[b], sy[c]         # band first - see below
        low = y0 if y0 < y1 else y1
        if y2 < low:
            low = y2
        first = int(low)
        if first >= bottom:
            continue
        high = y0 if y0 > y1 else y1
        if y2 > high:
            high = y2
        last = int(high) + 1
        if last <= top:
            continue

        oa, ob, og = ooz[a], ooz[b], ooz[c]
        if not (oa and ob and og):
            continue

        x0, x1, x2 = sx[a], sx[b], sx[c]
        area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)

        if area <= 0.0 and (cull or area == 0.0):
            continue
        if area <= 0.0:
            # DoubleSide: turn the winding rather than dropping the face, so
            # the barycentric signs below hold either way round.
            x1, y1, x2, y2 = x2, y2, x1, y1
            ob, og = og, ob
            area = -area

        left = int(x0 if x0 < x1 else x1)
        if x2 < left:
            left = int(x2)
        right = int(x0 if x0 > x1 else x1)
        if x2 > right:
            right = int(x2)
        right += 1

        # Whether the triangle is sub-pixel is a fact about the TRIANGLE, so it
        # is decided before the band and the frame clip it.
        tiny = right - left == 1 and last - first == 1

        if first < top:
            first = top
        if last > bottom:
            last = bottom
        if right <= 0 or left >= cols:
            continue
        if left < 0:
            left = 0
        if right > cols:
            right = cols

        near = oa if oa > ob else ob
        if og > near:
            near = og

        j = tri + tri + tri
        n0, n1, n2 = normals[j], normals[j + 1], normals[j + 2]
        lit = shade_of((m0 * n0 + m1 * n1 + m2 * n2,
                        m3 * n0 + m4 * n1 + m5 * n2,
                        m6 * n0 + m7 * n1 + m8 * n2),
                       ((vx[a] + vx[b] + vx[c]) / 3.0,
                        (vy[a] + vy[b] + vy[c]) / 3.0,
                        (vz[a] + vz[b] + vz[c]) / 3.0), lamp, eye)

        # A triangle inside one pixel needs no weights, and 74% of what this
        # mesh draws at a terminal's size is that small.
        wear = None if who is None or tints is None else tints[tri] + 1
        at = (first - top) * cols + left
        if tiny and who is not None and wear is not None and near > depth[at]:
            who[at] = wear
        if tiny and near > depth[at]:
            depth[at] = near
            value[at] = lit
        if tiny:
            continue
        _fill(depth, value, who, wear, lit,
              (x0, y0, x1, y1, x2, y2, oa, ob, og), area,
              first, last, left, right, top, cols)

    return depth, value


def _fill(depth, value, who, wear, lit, tri, area,
          first, last, left, right, top, cols):
    """One non-tiny triangle into the band's buffers."""
    x0, y0, x1, y1, x2, y2, oa, ob, og = tri
    inv = 1.0 / area
    e0x, e0y = x2 - x1, y2 - y1
    e1x, e1y = x0 - x2, y0 - y2
    e2x, e2y = x1 - x0, y1 - y0

    for py in range(first, last):
        row = (py - top) * cols
        dy1, dy2, dy0 = py - y1, py - y2, py - y0
        for px in range(left, right):
            w0 = (e0x * dy1 - e0y * (px - x1)) * inv
            if w0 < 0.0:
                continue
            w1 = (e1x * dy2 - e1y * (px - x2)) * inv
            if w1 < 0.0:
                continue
            w2 = (e2x * dy0 - e2y * (px - x0)) * inv
            if w2 < 0.0:
                continue
            # 1/z is linear in screen space, so this interpolation is exact
            # rather than the usual affine approximation.
            here = w0 * oa + w1 * ob + w2 * og
            if here <= depth[row + px]:
                continue
            depth[row + px] = here
            value[row + px] = lit
            if who is not None:
                who[row + px] = wear


def render(model, matrix, width, height, distance=None, ramp=CHARACTERS,
           invert=True, supersample=SUPERSAMPLE, zoom=1.0, centre=None,
           light=None, aspect=CELL_ASPECT, cull=CULLING, ink=None,
           tints=None, ink_colour=None, shades=None, wire=False):
    """`model` under `matrix`, as `height` lines of `width` characters."""
    cols, rows, cell_rows, distance, scale, cx, cy = _setup(
        model, matrix, width, height, distance, zoom, supersample, aspect,
        centre)

    who = [0] * (rows * cols) if tints is not None else None
    depth, value = rasterise(model, matrix, distance, scale, cx, cy, cols,
                             0, rows, light if light else light_position(),
                             cull, tints=tints, who=who)
    return resolve(depth, value, width, height, cols, cell_rows,
                    supersample, ramp, invert, ink=ink, who=who,
                    ink_colour=ink_colour, shades=shades, wire=wire)


#: Relative 1/z step between neighbouring CELLS that reads as an edge.
#: Tuned by eye on the 28-cell toon mesh: at 0.03 nearly every cell of the
#: snapped geometry bordered a step and the picture was all outline; 0.10
#: keeps the silhouette and the tall parts and lets the faces be faces.
INK_STEP = 0.10

#: The wire mode's own, far finer depth threshold: a part stands 2-8 mm
#: off a 100 mm board, a 1-4 %% step in 1/z - INK_STEP was tuned against
#: cluster noise in the FILLED look and swallowed every component outline
#: when the fill went away.
WIRE_STEP = 0.018


def resolve(depth, value, width, height, cols, cell_rows, supersample,
             ramp, invert, ink=None, who=None, ink_colour=None,
             shades=None, wire=False):
    """The framebuffer down to characters, averaging each cell."""
    background = ramp[0] if invert else ramp[-1]

    # Per cell first: mean shade and nearest depth, so the ink test is cell
    # against cell rather than pixel against pixel.
    cells = width * height
    shade = [0.0] * cells
    near = [0.0] * cells
    paintbox = [0] * cells if who is not None else None
    z_lo, z_hi = 1e30, 0.0
    for r in range(height):
        row = r * width
        for c in range(width):
            total = seen = 0
            deep = 0.0
            wore = 0
            for dr in range(cell_rows):
                base = (r * cell_rows + dr) * cols + c * supersample
                for dc in range(supersample):
                    d = depth[base + dc]
                    if d:
                        total += value[base + dc]
                        seen += 1
                    if d > deep:
                        deep = d
                        wore = who[base + dc] if who is not None else wore
            if seen:
                shade[row + c] = total / seen
                near[row + c] = deep
                z_lo = min(z_lo, deep)
                z_hi = max(z_hi, deep)
            if seen and paintbox is not None:
                paintbox[row + c] = wore

    # The posterised band per cell, for the crease test: a component's side is
    # 2+ bands from its lit top whatever the depth noise says, which is what
    # lets the ink find parts the z-buffer cannot separate.
    steps = len(ramp) - 1
    band = [0] * cells
    if ink is not None:
        lit = [(1.0 - shade[at]) if invert else shade[at] for at in range(cells)]
        band = [int((1.0 - lit[at]) * steps) if near[at] else 0
                for at in range(cells)]

    lines = []
    for r in range(height):
        cells_out = []
        row = r * width
        for c in range(width):
            at = row + c
            if not near[at]:
                cells_out.append((background, None))
            elif wire:
                cells_out.append(_wire_cell(near, paintbox, shades,
                                            ink_colour, at, c, r, width,
                                            height, z_lo, z_hi))
            elif ink is not None and _inked(near, band, at, c, r,
                                            width, height):
                cells_out.append((ink, ink_colour))
            else:
                wear = (paintbox[at] - 1
                        if paintbox is not None and paintbox[at] else None)
                cells_out.append((brightness_char(shade[at], ramp, invert),
                                  _deepened(wear, shades, band[at], steps)))
        while cells_out and cells_out[-1] == (background, None):
            cells_out.pop()
        if paintbox is None:
            lines.append(''.join(ch for ch, _ in cells_out))
        else:
            lines.append(ansi.run(cells_out))

    return '\n'.join(lines)


def _wire_cell(near, paintbox, shades, ink_colour, at, c, r, width, height,
               z_lo, z_hi):
    """THE VECTOR LOOK: interiors dark, edges only, each with the stroke of
    its own direction.
    """
    sides = _edge_sides(near, None, at, c, r, width, height, step=WIRE_STEP)
    if sides is None:
        return (' ', None)
    horiz, vert = sides
    glyph = '+' if horiz and vert else '|' if horiz else '-'
    wearing = (paintbox[at] - 1 if paintbox is not None and paintbox[at]
               else ink_colour)
    if wearing is not None and shades is not None:
        span = (z_hi - z_lo) or 1.0
        third = min(2, int(3.0 * (near[at] - z_lo) / span))
        wearing = shades.get(wearing, (wearing,) * 3)[third]
    return (glyph, wearing)


def _deepened(wear, shades, band_at, steps):
    """The zone's colour, DEEPENED where the light falls away: the same
    face reads as one part with a lit top and a dark flank instead of one
    flat sticker."""
    if wear is None or shades is None:
        return wear
    third = min(2, band_at * 3 // max(1, steps))
    return shades.get(wear, (wear,) * 3)[third]


def _edge_sides(near, band, at, c, r, width, height, step=INK_STEP):
    """(horizontal, vertical) edge triggers for the wire mode, or None."""
    here = near[at]
    mine = band[at] if band is not None else None
    horiz = vert = False
    for other, sideways in (((at - 1) if c > 0 else -1, True),
                            ((at + 1) if c + 1 < width else -1, True),
                            ((at - width) if r > 0 else -1, False),
                            ((at + width) if r + 1 < height else -1, False)):
        if other < 0:
            continue
        there = near[other]
        jump = ((mine - band[other])
                if mine is not None and band is not None else 0)
        hit = (not there or here - there > step * here
               or jump >= 2 or jump <= -2)
        if hit and sideways:
            horiz = True
        elif hit:
            vert = True
    if not (horiz or vert):
        return None
    return horiz, vert


def _inked(near, band, at, c, r, width, height):
    """Whether the covered cell at `at` sits on an edge worth inking."""
    here = near[at]
    mine = band[at]
    for other in ((at - 1) if c > 0 else -1,
                  (at + 1) if c + 1 < width else -1,
                  (at - width) if r > 0 else -1,
                  (at + width) if r + 1 < height else -1):
        if other < 0:
            continue                   # the frame clips; that is not an edge
        there = near[other]
        if not there:
            return True                # silhouette against background
        # One-sided on purpose: only the NEARER cell of a step carries the
        # line, so an outline is one cell wide instead of two.
        if here - there > INK_STEP * here:
            return True
        jump = mine - band[other]
        if jump >= 2 or jump <= -2:
            return True
    return False
