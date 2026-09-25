"""The staged 3D engine: pure functions, one per pipeline step."""
import math

from coaxial.graphics.raster import BRAILLE_BITS, DOTS_X, DOTS_Y

#: Classes an art face loses leaning 90 degrees from the viewer:
#: 3.4 * (1 - cos 45) is one full class, the exporter's own y45 step.
LEAN = 3.4

#: The share of the depth ramp above ':' a cell's colour tone keeps;
#: its glyph class keeps all of it.
TONE_DEPTH = 0.0


def pose(rx, ry, rz, order='xyz'):
    """Body-frame turns in degrees, applied in `order`, as a matrix."""
    def axis(name, deg):
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        if name == 'x':
            return (1, 0, 0, 0, c, -s, 0, s, c)
        if name == 'y':
            return (c, 0, s, 0, 1, 0, -s, 0, c)
        return (c, -s, 0, s, c, 0, 0, 0, 1)

    degrees = {'x': rx, 'y': ry, 'z': rz}
    m = (1, 0, 0, 0, 1, 0, 0, 0, 1)
    for name in order:
        m = multiply(m, axis(name, degrees[name]))
    return m


def multiply(a, b):
    """Row-major 3x3 product a*b."""
    return tuple(sum(a[3 * r + k] * b[3 * k + c] for k in range(3))
                 for r in range(3) for c in range(3))


def camera(width, height, reach, distance=3.2, zoom=1.0, tip=0.0,
           lift=0.5):
    """Projection constants for a frame: everything project() needs."""
    t = math.radians(tip)
    ct, st = math.cos(t), math.sin(t)
    bound = 2.0 * reach / math.sqrt(distance * distance - reach * reach)
    scale = zoom * min(0.94 * width / bound, 0.92 * height / (0.5 * bound))
    return {'view': (1.0, 0.0, 0.0, 0.0, ct, st, 0.0, -st, ct),
            'scale': scale, 'cx': width / 2.0, 'cy': height * lift,
            'distance': distance, 'width': width, 'height': height,
            'reach': reach}


def project(cam, m, point):
    """One body point through attitude and camera to (sx, sy, depth)."""
    x, y, z = point
    tx = m[0] * x + m[1] * y + m[2] * z
    ty = m[3] * x + m[4] * y + m[5] * z
    tz = m[6] * x + m[7] * y + m[8] * z
    w = 1.0 / (cam['distance'] - tz)
    # `aspect` is the row's share of the column's scale: 0.5 for a character
    # cell, about twice as tall as wide; 1.0 for the braille dot raster `fine`
    # makes, whose samples are square.
    return (cam['cx'] + cam['scale'] * w * tx,
            cam['cy'] - cam['scale'] * cam.get('aspect', 0.5) * w * ty, w)


def fine(cam):
    """The camera at braille dot resolution: DOTS_X columns and DOTS_Y rows
    of samples per cell, square on screen, for the raster `fold` brings
    back down to cells.
    """
    return dict(cam, width=DOTS_X * cam['width'],
                height=DOTS_Y * cam['height'],
                scale=DOTS_X * cam['scale'],
                aspect=0.5 * DOTS_Y / DOTS_X,
                cx=DOTS_X * cam['cx'], cy=DOTS_Y * cam['cy'])


def raster(solid, m, cam, beam=None, sun_min=0.0, band=None):
    """Depth, horizontal-face and shadow-eligibility flags per cell."""
    pos, idx, nrm = solid
    m0, m1, m2, m3, m4, m5, m6, m7, m8 = m
    width, height = cam['width'], cam['height']
    first, last = band if band else (0, height)
    lx, ly, lz = beam if beam else (0.0, 0.0, 1.0)
    n = len(pos) // 3
    sx, sy, ooz = [0.0] * n, [0.0] * n, [0.0] * n
    for i in range(n):
        px, py, w = project(cam, m, (pos[3 * i], pos[3 * i + 1],
                                     pos[3 * i + 2]))
        sx[i], sy[i], ooz[i] = px, py, w

    rows = last - first
    depth = [0.0] * (width * rows)
    top = bytearray(width * rows)
    sun = bytearray(width * rows)
    for t in range(len(idx) // 3):
        a, b, c = idx[3 * t], idx[3 * t + 1], idx[3 * t + 2]
        bx, by, bz = nrm[3 * t], nrm[3 * t + 1], nrm[3 * t + 2]
        nx = m0 * bx + m1 * by + m2 * bz
        ny = m3 * bx + m4 * by + m5 * bz
        nz = m6 * bx + m7 * by + m8 * bz
        if nz < 0.0:
            nx, ny, nz = -nx, -ny, -nz
        # Shadow-eligible: facing the beam, and on the component side of the
        # board (body +z).
        lit = bz > 0.0 and nx * lx + ny * ly + nz * lz > sun_min
        flat = bz > 0.9 or bz < -0.9

        x0, y0, x1, y1, x2, y2 = sx[a], sy[a], sx[b], sy[b], sx[c], sy[c]
        area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
        if area == 0.0:
            continue
        oa, ob, og = ooz[a], ooz[b], ooz[c]
        if area < 0.0:
            x1, y1, x2, y2 = x2, y2, x1, y1
            ob, og = og, ob
            area = -area
        lo_x = max(0, int(min(x0, x1, x2)))
        hi_x = min(width - 1, int(max(x0, x1, x2)) + 1)
        lo_y = max(first, int(min(y0, y1, y2)))
        hi_y = min(last - 1, int(max(y0, y1, y2)) + 1)
        if hi_x < lo_x or hi_y < lo_y:
            continue
        inv = 1.0 / area
        # The two edge functions, each a row term less a column term.
        e0x, e0y = x2 - x1, y2 - y1
        e1x, e1y = x0 - x2, y0 - y2
        for py in range(lo_y, hi_y + 1):
            row = (py - first) * width
            r0 = e0x * (py - y1)
            r1 = e1x * (py - y2)
            # A triangle's pixels on a row are one span, so the first miss
            # after a hit ends the row: the same three tests on the same
            # floats, and the half of the bounding box past the far edge is not
            # visited.
            inside = False
            for px in range(lo_x, hi_x + 1):
                w0 = (r0 - e0y * (px - x1)) * inv
                if w0 < 0.0:
                    if inside:
                        break
                    continue
                w1 = (r1 - e1y * (px - x2)) * inv
                if w1 < 0.0:
                    if inside:
                        break
                    continue
                w2 = 1.0 - w0 - w1
                if w2 < 0.0:
                    if inside:
                        break
                    continue
                inside = True
                here = w0 * oa + w1 * ob + w2 * og
                at = row + px
                if here > depth[at]:
                    depth[at] = here
                    top[at] = flat
                    sun[at] = lit
    return depth, top, sun


#: A cell's eight dot samples in the fine raster: (column, row) offset
#: within the cell and the braille bit that dot is, so the fold's mask
#: is the glyph's own bit order and a dot grid can be clipped by one
#: `&`.
DOT_SAMPLES = tuple((lane, y, BRAILLE_BITS[lane][y])
                    for y in range(DOTS_Y) for lane in range(DOTS_X))


def _cellwise(field, width, height, dtype):
    """A fine raster (lists, bytes or an array) as (height, width, 8): a cell's
    samples in DOT_SAMPLES order."""
    from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
    return (np.asarray(field, dtype).reshape(height, DOTS_Y, width, DOTS_X)
            .transpose(0, 2, 1, 3).reshape(height, width, DOTS_X * DOTS_Y))


def fold(depth, top, sun, width, height):
    """A dot-resolution raster (`fine`) down to cells: (depth, top, sun,
    coverage, reached). A cell's depth, top and sun are its nearest sample's,
    the first of equals in DOT_SAMPLES order.
    """
    from coaxial.model.blocks import numpy as np
    d = _cellwise(depth, width, height, float)
    hit = d != 0.0
    hits = hit.sum(axis=2)
    near = d.argmax(axis=2)[..., None]
    bits = np.array([bit for _lane, _y, bit in DOT_SAMPLES])
    covered = hits > 0

    def at_near(field):
        got = np.take_along_axis(_cellwise(field, width, height, np.uint8), near, 2)[..., 0]
        return bytearray(np.where(covered, got, 0).astype(np.uint8).tobytes())

    return (np.where(covered, np.take_along_axis(d, near, 2)[..., 0], 0.0).ravel().tolist(),
            at_near(top), at_near(sun),
            np.where(covered, hits / float(len(DOT_SAMPLES)), 0.0).ravel().tolist(),
            bytearray((hit * bits).sum(axis=2).astype(np.uint8).tobytes()))


#: Beyond this radius a surface point takes no ink from the art: the
#: art's own ink reaches 0.94 to 1.00 of the span by direction
#: (measured 2026-09-23, 24 directions), so a covered cell at the
#: mesh's rim can land on the art's blank outside and draw nothing,
#: pulling the lit edge a cell inward - a second staircase inside the
#: first, the band inside the edges torn at i -0.1350 j -0.7956
#: k -0.5388 real -0.2413. Past the disc the rim is bare geometry and
#: draws by depth, like a wall.
ART_DISC = 0.96


def _art_hit(m, u, v, distance, tz, back, art_w, art_h, plane=0.0):
    """The art cell under each cell's own surface point (numpy arrays), that
    point's rise above the art's plane, and whether it is inside the unit
    disc at all: (ix, iy, rise, inside).
    """
    from coaxial.model.blocks import numpy as np
    d = distance - tz
    tx, ty = u * d, v * d
    hx = m[0] * tx + m[3] * ty + m[6] * tz
    hy = m[1] * tx + m[4] * ty + m[7] * tz
    hz = m[2] * tx + m[5] * ty + m[8] * tz
    inside = np.logical_not(hx * hx + hy * hy > ART_DISC * ART_DISC)
    lean = m[8] if m[8] >= 0.05 else (m[8] if m[8] <= -0.05
                                      else (0.05 if not back else -0.05))
    rise = (hz - plane) / lean
    hx = -hx if back else hx
    # An art cell covers [i/w, (i+1)/w) of the span, so the origin is cell w/2,
    # not (w-1)/2: scaled by (w - 1) the lookup sat half a cell low and left -
    # 0.02 units in x and 0.04 in y, a braille row on screen - and the bore's
    # blank sat beside the see-through. Truncated toward zero, as int() does.
    iy = np.clip(np.asarray((1.0 - (hy + 1.0) * 0.5) * art_h).astype(int), 0, art_h - 1)
    ix = np.clip(np.asarray((hx + 1.0) * 0.5 * art_w).astype(int), 0, art_w - 1)
    return ix, iy, rise, inside


#: The art's ink as an array, by the list it came as: converted once.
_DENSE = {}


def shade(depth, top, sun, cam, m, pivot, slope, floor,
          art=None, shadow=None, shadow_step=0.0, bias=0.0, levels=None,
          bare=None, seed=None, planes=None):
    """Depth to classes 0..2: 0 blank, 1 the exporter's '.', 2 its ':'.
    Every covered cell at once; `levels`, `bare`, `seed` written at those.
    """
    from coaxial.model.blocks import numpy as np
    width, height = cam['width'], cam['height']
    distance, scale = cam['distance'], cam['scale']
    cx, cy = cam['cx'], cam['cy']
    reach = cam.get('reach', 1.0)
    back = m[8] < 0.0
    # The art is the top's layout: seen from behind the slab shows none, or
    # the top's parts print through onto the bottom. It is read on the top
    # plane (`planes` is (top, bottom); z = 0 where a model names none).
    _rows, art_w, art_h, dense = art if (art and not back) else ([], 0, 0, [])
    plane = planes[0] if planes is not None else 0.0
    w_all = np.asarray(depth, float)
    at = np.flatnonzero(w_all != 0.0)
    py, px = np.divmod(at, width)
    w = w_all[at]
    lid = np.asarray(top, np.uint8)[at] != 0
    v = (cy - (py + 0.5)) / (scale * 0.5)
    u = (px + 0.5 - cx) / scale
    # The cell's own view-space point, back out of the projection: one depth
    # value is a full position.
    tz = distance - 1.0 / w
    tx, ty = u / w, v / w
    shaded = np.zeros(len(at))
    if shadow:
        sbuf, s_n, s_ext, s_right, s_up, s_beam = shadow
        rx, ry, rz = s_right
        ux, uy, uz = s_up
        bx, by, bz = s_beam
        si = ((tx * rx + ty * ry + tz * rz) / s_ext * 0.5 + 0.5) * (s_n - 1)
        sj = ((tx * ux + ty * uy + tz * uz) / s_ext * 0.5 + 0.5) * (s_n - 1)
        si, sj = si.astype(int), sj.astype(int)
        cast = (lid & (np.asarray(sun, np.uint8)[at] != 0)
                & (0 <= si) & (si < s_n) & (0 <= sj) & (sj < s_n))
        occluder = np.asarray(sbuf, float)[np.where(cast, sj * s_n + si, 0)]
        shaded = np.where(cast & (occluder > tx * bx + ty * by + tz * bz + bias),
                          shadow_step, 0.0)
    ink = np.full(len(at), -1)
    ix, iy, rise = px, py, np.zeros(len(at))
    if art_w:
        ink_of = _DENSE.get(id(dense))
        if ink_of is None or ink_of[0] is not dense:
            ink_of = _DENSE[id(dense)] = (dense, np.asarray(dense))
        aix, aiy, arise, inside = _art_hit(m, u, v, distance, tz, back, art_w, art_h, plane)
        hit = lid & inside
        ink = np.where(hit, ink_of[1][aiy, aix], -1)
        ix, iy, rise = np.where(hit, aix, px), np.where(hit, aiy, py), np.where(hit, arise, 0.0)
    level = pivot + slope * tz / reach
    if bare is not None:
        bare[:] = _written(bare, at, level)
    # An art cell's class is its ink dimmed by how far the face leans from the
    # viewer - no depth term - and never leans below the floor.
    lean = m[8] if m[8] >= 0.0 else -m[8]
    inked = pivot + ink - 2 - LEAN * (1.0 - lean) + slope * rise / reach - shaded
    inked = np.where((ink > 0) & (inked < floor), floor, inked)
    level = np.where(ink >= 0, inked, np.maximum(level - shaded, floor))
    if seed is not None:
        # A fixed 0..1 per cell for the glow's surface texture: hashed on the
        # art cell an art pixel shows, so the grain turns with the board; on the
        # screen cell elsewhere.
        seed[:] = _written(seed, at, (((ix * 73856093) ^ (iy * 19349663)) & 255) / 255.0)
    if levels is not None:
        # The tone is the class the glyph shows plus TONE_DEPTH of the residual -
        # depth grades within a class, never across the picture.
        cls = np.clip((level + 0.5).astype(int), 0, 2)
        levels[:] = _written(levels, at, cls + TONE_DEPTH * (level - cls))
    out = np.zeros(width * height, np.uint8)
    out[at] = (np.clip(level, 0.0, 2.0) + 0.5).astype(int)
    return bytearray(out.tobytes())


def _written(into, at, values):
    """`into` as a list with `values` at `at`, everything else as it was."""
    from coaxial.model.blocks import numpy as np
    whole = np.asarray(into, float)
    whole[at] = values
    return whole.tolist()


def compose(classes, width, height, ramp=' .:'):
    """The class field as text rows, the exporter's own characters."""
    return [''.join(ramp[classes[py * width + px]]
                    for px in range(width))
            for py in range(height)]
