"""The staged 3D engine: pure functions, one per pipeline step.

    pose      degrees -> rotation matrix (body->view, row-major)
    camera    frame + model reach -> projection constants
    raster    triangles + matrix + camera -> depth field + face flags
    shade     depth field (+ art, + shadow map) -> class field 0..2
    compose   class field -> text rows

Every stage takes plain data and returns plain data, so
tests/test_render.py holds each stage against exact expectations and
the whole chain against an analytic ray-cast oracle - render/render_demo.ps1
is that bench. wireframe.py dresses the output (backdrop,
colour, the wire fallback) and owns the fitted shading constants.

Depth IS the shading. A surface-normal lambert was built, fitted and
removed: the CAD exporter's cube reference shows its '.'-to-':'
boundary cutting ACROSS a flat face, which no normal-based light can
produce and a depth ramp does exactly.
"""
import math

from .raster import BRAILLE_BITS, DOTS_X, DOTS_Y

#: Classes an art face loses leaning 90 degrees from the viewer:
#: 3.4 * (1 - cos 45) is one full class, the exporter's own y45 step.
LEAN = 3.4

#: The share of the depth ramp ABOVE ':' a cell's colour tone keeps;
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
    """Projection constants for a frame: everything project() needs.

    The scale is the tangent-ray bound of the model's bounding sphere -
    attitude-independent, so the picture never breathes with rotation -
    and `lift` places the vertical centre (0.5 dead centre; the live
    view sits at 0.44). `tip` pitches the camera itself, in degrees."""
    t = math.radians(tip)
    ct, st = math.cos(t), math.sin(t)
    bound = 2.0 * reach / math.sqrt(distance * distance - reach * reach)
    scale = zoom * min(0.94 * width / bound, 0.92 * height / (0.5 * bound))
    return {'view': (1.0, 0.0, 0.0, 0.0, ct, st, 0.0, -st, ct),
            'scale': scale, 'cx': width / 2.0, 'cy': height * lift,
            'distance': distance, 'width': width, 'height': height,
            'reach': reach}


def project(cam, m, point):
    """One body point through attitude and camera to (sx, sy, depth).

    Depth is 1/(distance - view_z): larger is nearer, the raster's own
    convention."""
    x, y, z = point
    tx = m[0] * x + m[1] * y + m[2] * z
    ty = m[3] * x + m[4] * y + m[5] * z
    tz = m[6] * x + m[7] * y + m[8] * z
    w = 1.0 / (cam['distance'] - tz)
    # `aspect` is the row's share of the column's scale: 0.5 for a
    # character cell, about twice as tall as wide; 1.0 for the braille
    # dot raster `fine` makes, whose samples are square.
    return (cam['cx'] + cam['scale'] * w * tx,
            cam['cy'] - cam['scale'] * cam.get('aspect', 0.5) * w * ty, w)


def fine(cam):
    """The camera at braille dot resolution: DOTS_X columns and DOTS_Y
    rows of samples per cell, square on screen, for the raster `fold`
    brings back down to cells. Doubling both axes was the first form -
    2x2, a quadrant one dot wide and two tall - and along a shallow
    silhouette the rim could only sit on two of a cell's four dot rows,
    so the edge stepped by whole cells; the bench saw it jagged. The
    fine y is DOTS_Y times the cell y, so the fine scale is DOTS_X
    times the cell scale with the aspect that makes the row DOTS_Y
    times taller."""
    return dict(cam, width=DOTS_X * cam['width'],
                height=DOTS_Y * cam['height'],
                scale=DOTS_X * cam['scale'],
                aspect=0.5 * DOTS_Y / DOTS_X,
                cx=DOTS_X * cam['cx'], cy=DOTS_Y * cam['cy'])


def raster(solid, m, cam, beam=None, sun_min=0.0, band=None):
    """Depth, horizontal-face and shadow-eligibility flags per cell.

    The z-test decides who owns a cell. Normals only classify: which
    faces are horizontal in the BODY frame (they wear the art), and
    which face the shadow `beam` enough to be worth testing. Rewinding
    a flipped triangle swaps two verts, so their depths swap WITH them
    - the old engine tested the area after negating it and the swap
    never ran.

    `band` = (first row, last row + 1) rasters that strip alone and
    returns arrays of the strip's own height: what one process of a
    crew draws. A band owns its rows of the z-buffer outright, so the
    strips concatenate with nothing to merge; only the vertex pass is
    repeated per band."""
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
        # Shadow-eligible: facing the beam, AND on the component side of
        # the board (body +z). The solder side has nothing standing on it
        # to cast a shadow, and testing it anyway against the coarse
        # caster map chattered at grazing angles - cells flipping in and
        # out of shade frame to frame, seen on the bench from below.
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
        # The row term and the edge's slopes are the same for every
        # pixel of a row, so they are worked out once per row: the same
        # products in the same order as the plain form, so every weight
        # is the same float, and a third of the work per pixel.
        e0x, e0y = x2 - x1, y2 - y1
        e1x, e1y = x0 - x2, y0 - y2
        for py in range(lo_y, hi_y + 1):
            row = (py - first) * width
            r0 = e0x * (py - y1)
            r1 = e1x * (py - y2)
            # A triangle's pixels on a row are one span, so the first
            # miss after a hit ends the row: the same three tests on
            # the same floats, and the half of the bounding box past
            # the far edge is not visited. Measured at 108x40 on the
            # grid-32 solid: the dot raster 79 ms to 69, the old 2x2
            # 49 to 45 - the setup per triangle is most of what is
            # left, since the triangles are small.
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
#: IS the glyph's own bit order and a dot grid can be clipped by one
#: `&`.
DOT_SAMPLES = tuple((lane, y, BRAILLE_BITS[lane][y])
                    for y in range(DOTS_Y) for lane in range(DOTS_X))


def fold(depth, top, sun, width, height):
    """A dot-resolution raster (`fine`) down to cells: (depth, top, sun,
    coverage, reached). A cell takes its NEAREST subsample's depth and
    flags, and the share of its eight subsamples that hit - the
    anti-aliasing a glyph grid can carry: a rim cell an eighth covered
    draws faint, one seven-eighths covered nearly full. Neighbour-
    counting stood in for this and could not tell a straight edge from
    a stair.

    `reached` is WHICH of the eight hit, as the braille bit of each dot
    (`DOT_SAMPLES`), so a dot grid can be clipped to the model's own
    silhouette by masking: a dot the model missed is a dot outside the
    board, and the bench saw those spill past the rim. It was four
    quadrants first, one dot wide and two tall, and a shallow rim then
    sat on two of a cell's four rows and stepped by whole cells."""
    wide = DOTS_X * width
    out_depth = [0.0] * (width * height)
    out_top = bytearray(width * height)
    out_sun = bytearray(width * height)
    coverage = [0.0] * (width * height)
    reached = bytearray(width * height)
    per = float(len(DOT_SAMPLES))
    for py in range(height):
        row = py * width
        base = DOTS_Y * py * wide
        for px in range(width):
            cell = base + DOTS_X * px
            best, hits, where, bits = 0.0, 0, cell, 0
            for lane, y, bit in DOT_SAMPLES:
                at = cell + y * wide + lane
                d = depth[at]
                if d:
                    hits += 1
                    bits |= bit
                    if d > best:
                        best, where = d, at
            if hits:
                out_depth[row + px] = best
                out_top[row + px] = top[where]
                out_sun[row + px] = sun[where]
                coverage[row + px] = hits / per
                reached[row + px] = bits
    return out_depth, out_top, out_sun, coverage, reached


def _art_hit(m, u, v, distance, tz, back, art_w, art_h):
    """Where a cell's view ray meets the art plane: the art cell it shows
    and the cell's rise above the plane, or None when it misses the
    plane's unit disc. The UNMIRRORED plane point is what the rise is
    measured from, before the back view flips hx for the lookup."""
    dz = m[2] * u + m[5] * v - m[8]
    if abs(dz) <= 1e-9:
        return None
    t = -(distance * m[8]) / dz
    hx = distance * m[6] + t * (m[0] * u + m[3] * v - m[6])
    hy = distance * m[7] + t * (m[1] * u + m[4] * v - m[7])
    if t <= 0.0 or hx * hx + hy * hy > 1.0:
        return None
    rise = tz - (m[6] * hx + m[7] * hy)
    hx = -hx if back else hx
    iy = int((1.0 - (hy + 1.0) * 0.5) * (art_h - 1))
    ix = int((hx + 1.0) * 0.5 * (art_w - 1))
    return ix, iy, rise


def shade(depth, top, sun, cam, m, pivot, slope, floor,
          art=None, shadow=None, shadow_step=0.0, bias=0.0, levels=None,
          bare=None, seed=None):
    """Depth to classes 0..2: 0 blank, 1 the exporter's '.', 2 its ':'.

    Pass a preallocated float list as `levels` to also receive the
    UNROUNDED level per cell - the continuous shading a colour ramp can
    carry where three glyphs cannot. `bare` receives the level with no
    art, no shadow and no floor: pure geometry, what a relief pass must
    difference - art ink is integer steps and saturates any edge gain.

    class = pivot + slope * view_z / reach: fixed per MODEL, never per
    frame - per-frame normalisation amplified a flat view's numeric
    noise across the whole ramp, twice, measured - and reach-scaled so
    one ramp serves the unit board and the sqrt(3) cube alike. `art` is (rows, w, h,
    dense) laid on horizontal faces as pure class STEPS, so a face-on
    board passes the art through verbatim by construction. `shadow` is
    a light-space depth map; a shadowed art cell steps down by
    `shadow_step`. Bare geometry never fades below `floor` - the
    exporter's deepest visible cube face still draws '.'."""
    width, height = cam['width'], cam['height']
    distance, scale = cam['distance'], cam['scale']
    cx, cy = cam['cx'], cam['cy']
    reach = cam.get('reach', 1.0)
    rows, art_w, art_h, dense = art if art else ([], 0, 0, [])
    back = m[8] < 0.0
    if shadow:
        sbuf, s_n, s_ext, s_right, s_up, s_beam = shadow
        rx, ry, rz = s_right
        ux, uy, uz = s_up
        bx, by, bz = s_beam
    out = bytearray(width * height)
    for py in range(height):
        row = py * width
        v = (cy - (py + 0.5)) / (scale * 0.5)
        for px in range(width):
            w = depth[row + px]
            if w == 0.0:
                continue
            u = (px + 0.5 - cx) / scale
            # The cell's own view-space point, back out of the
            # projection: one depth value is a full position.
            tz = distance - 1.0 / w
            tx, ty = u / w, v / w
            shaded = 0.0
            if shadow and top[row + px] and sun[row + px]:
                sa = tx * rx + ty * ry + tz * rz
                sb = tx * ux + ty * uy + tz * uz
                si = int((sa / s_ext * 0.5 + 0.5) * (s_n - 1))
                sj = int((sb / s_ext * 0.5 + 0.5) * (s_n - 1))
                behind = (0 <= si < s_n and 0 <= sj < s_n
                          and sbuf[sj * s_n + si] > (tx * bx + ty * by
                                                     + tz * bz + bias))
                shaded = shadow_step if behind else 0.0
            ink = -1
            ix, iy = px, py
            hit = (_art_hit(m, u, v, distance, tz, back, art_w, art_h)
                   if art_w and top[row + px] else None)
            if hit is not None:
                ix, iy, rise = hit
                ink = dense[iy][ix]
            level = pivot + slope * tz / reach
            if bare is not None:
                bare[row + px] = level
            if ink >= 0:
                # An art cell's class is its ink dimmed by how far the
                # face leans from the viewer - no depth term. Measured
                # against the exporter's y45 board: with the depth
                # wedge the render drew 98% ':' against his 26%, and
                # his face dims UNIFORMLY with tilt (face-on 73% ':',
                # y45 74% '.') while tall parts break through - a
                # facing ratio, which LEAN scales to one full class at
                # 45 degrees. Bare geometry keeps the depth ramp: the
                # cube reference's class boundary crosses a flat face,
                # which facing alone cannot draw.
                # Height ABOVE the art plane rides the depth ramp, so
                # a tall lid climbs back through the lean: the
                # exporter's connector stays ':' at y45 while the face
                # around it drops to '.'.
                lean = m[8] if m[8] >= 0.0 else -m[8]
                level = (pivot + ink - 2 - LEAN * (1.0 - lean)
                         + slope * rise / reach - shaded)
                # INK NEVER LEANS BELOW THE FLOOR. At 73 degrees the
                # lean took every ':' of the art to class 0 (599 of
                # 653 blank cells, measured 2026-09-23) and the face
                # vanished, leaving the parts' walls and lids - drawn
                # bare, at class 2 - as a thick block with no plate
                # under them, the edge line wandering inside the
                # blanked face. The art's blanks (its holes, its
                # outside) stay blank: only ink has a floor.
                if ink > 0 and level < floor:
                    level = floor
            else:
                level = max(level - shaded, floor)
            if seed is not None:
                # A fixed 0..1 per cell for the glow's surface texture:
                # hashed on the ART cell an art pixel shows, so the grain
                # turns with the board; on the screen cell elsewhere.
                seed[row + px] = (((ix * 73856093) ^ (iy * 19349663))
                                  & 255) / 255.0
            if levels is not None:
                # The tone is the CLASS the glyph shows plus TONE_DEPTH
                # of the residual - depth grades within a class, never
                # across the picture. Measured with the whole ramp in
                # the tone: the far connector's ':' at tone 1 against
                # the near side's at 6, depth fog the exporter's images
                # do not have, and a 0.6 share of it still left the top
                # connectors of a Y-turned board two tones under the
                # rest. Light comes from the lamp, in the glow pass.
                cls = int(level + 0.5)
                cls = 0 if cls < 0 else (2 if cls > 2 else cls)
                levels[row + px] = cls + TONE_DEPTH * (level - cls)
            level = 0.0 if level < 0.0 else (2.0 if level > 2.0
                                             else level)
            out[row + px] = int(level + 0.5)
    return out


def compose(classes, width, height, ramp=' .:'):
    """The class field as text rows, the exporter's own characters."""
    return [''.join(ramp[classes[py * width + px]]
                    for px in range(width))
            for py in range(height)]
