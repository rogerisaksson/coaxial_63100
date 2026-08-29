"""A vector model of the board, and the line renderer that draws it.

The clustered STL is honest geometry but a dishonest DRAWING: five thousand
triangles wireframed is a circle full of marks. A vector display never drew
triangles - it drew the twenty edges somebody chose. So this module chooses
them: the rim as two rings with struts, the bore, and every part from the
parametric board as an eight-cornered cage, each edge carrying its zone's
colour.

Three things make it read as 3D, none of them more characters:

  * HIDDEN LINES ARE REMOVED. The solid parametric board is rastered into
    a depth buffer first, and an edge sample that loses the depth test is
    simply not drawn - the far rim vanishes behind the board the way it
    should.
  * DEPTH CUES THE BRIGHTNESS. Near edges burn, far edges dim, three
    shades per zone.
  * STROKES FOLLOW THE LINE. A segment draws with the glyph of its own
    screen direction - `|`, `-`, `/`, `\\` - so an edge reads as a stroke,
    not a row of pound signs.
"""
import math

#: OUTER and BORE are the parametric board's own - one definition, and the
#: two drawings can never disagree about the hole. THICK is this module's
#: and deliberately twice the real slab: a vector drawing honest about a
#: 1.6 mm laminate reads as a single line, and the two rim rings are what
#: sells the turn.
from .orientation import BORE, OUTER                       # noqa: E402

THICK = 0.05

#: Degrees the camera tips from straight-down toward the horizon. Enough
#: for the ground plane to READ as a landscape; small enough that a tared
#: board is still a face-on circle to the eye.
CAMERA_TIP = 34.0

#: The ground plane's height under the board, model units.
GROUND = -1.3

#: Struts between the two rim rings, and segments per ring.
RIM_SEGMENTS = 36
RIM_STRUTS = 12

#: The same palette orientation's toon uses, so both drawings agree about
#: what a part is. (dark, mid, bright) per zone, picked by depth.
SHADES = {
    'board': (17, 23, 37),
    'connector': (94, 166, 208),
    'fet': (130, 208, 214),
    'micro': (24, 38, 51),
    'passive': (22, 65, 114),
}


def _ring(radius, z, segments=RIM_SEGMENTS):
    """Closed ring of edges at height `z`, as [(a, b), ...] vertex pairs."""
    ring = [(radius * math.cos(2 * math.pi * i / segments),
             radius * math.sin(2 * math.pi * i / segments), z)
            for i in range(segments)]
    return [(ring[i], ring[(i + 1) % segments]) for i in range(segments)]


def _cage(phi_deg, radius, half_r, half_phi_deg, height, relief=3.0):
    """A part as its polar box's edges: top rectangle plus four posts."""
    half_phi = math.radians(half_phi_deg)
    phi0 = math.radians(phi_deg)
    top, base = THICK + height * relief, THICK

    def at(ri, pi, z):
        r = radius + (half_r if ri else -half_r)
        phi = phi0 + (half_phi if pi else -half_phi)
        return (r * math.cos(phi), r * math.sin(phi), z)

    corners = [(0, 0), (1, 0), (1, 1), (0, 1)]
    edges = []
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        edges.append((at(*a, top), at(*b, top)))       # the lid
        edges.append((at(*a, base), at(*a, top)))      # the post
    return edges


def _build():
    """[(a, b, zone)] - the whole board as chosen edges, built once."""
    from . import orientation

    edges = []
    for z in (THICK, -THICK):
        edges += [(a, b, 'board') for a, b in _ring(OUTER, z)]
    for i in range(RIM_STRUTS):
        phi = 2 * math.pi * i / RIM_STRUTS
        a = (OUTER * math.cos(phi), OUTER * math.sin(phi), THICK)
        b = (OUTER * math.cos(phi), OUTER * math.sin(phi), -THICK)
        edges.append((a, b, 'board'))
    edges += [(a, b, 'board') for a, b in _ring(BORE, THICK, 18)]

    for part in orientation.COMPONENTS:
        zone = part[0]
        edges += [(a, b, zone) for a, b in _cage(*part[1:])]
    return edges


_EDGES = None
_SOLID = None


def _model():
    global _EDGES, _SOLID
    if _EDGES is None:
        from . import orientation
        _EDGES = _build()
        # The occluder: the same board as coarse solid triangles, only ever
        # rasterised into depth. Low resolution on purpose - it decides
        # visibility, not appearance.
        _SOLID = orientation.facets(steps=48, relief=3.0)
    return _EDGES, _SOLID


def _depth(solid, m, distance, scale, cx, cy, width, height):
    """A width x height cell depth buffer of the solid board."""
    pos, idx, _nrm = solid
    m0, m1, m2, m3, m4, m5, m6, m7, m8 = m
    n = len(pos) // 3
    sx, sy, ooz = [0.0] * n, [0.0] * n, [0.0] * n
    for i in range(n):
        x, y, z = pos[3 * i], pos[3 * i + 1], pos[3 * i + 2]
        tx = m0 * x + m1 * y + m2 * z
        ty = m3 * x + m4 * y + m5 * z
        tz = m6 * x + m7 * y + m8 * z
        w = 1.0 / (distance - tz)
        ooz[i] = w
        sx[i] = cx + scale * w * tx
        sy[i] = cy - scale * 0.5 * w * ty

    buf = [0.0] * (width * height)
    for t in range(len(idx) // 3):
        a, b, c = idx[3 * t], idx[3 * t + 1], idx[3 * t + 2]
        x0, y0, x1, y1, x2, y2 = sx[a], sy[a], sx[b], sy[b], sx[c], sy[c]
        area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
        if area == 0.0:
            continue
        if area < 0.0:
            x1, y1, x2, y2 = x2, y2, x1, y1
            area = -area
        oa, ob, og = ooz[a], ooz[b], ooz[c]
        if area < 0.0:
            ob, og = og, ob
        lo_x = max(0, int(min(x0, x1, x2)))
        hi_x = min(width - 1, int(max(x0, x1, x2)) + 1)
        lo_y = max(0, int(min(y0, y1, y2)))
        hi_y = min(height - 1, int(max(y0, y1, y2)) + 1)
        if hi_x < lo_x or hi_y < lo_y:
            continue
        inv = 1.0 / area
        for py in range(lo_y, hi_y + 1):
            row = py * width
            for px in range(lo_x, hi_x + 1):
                w0 = ((x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)) * inv
                if w0 < 0.0:
                    continue
                w1 = ((x0 - x2) * (py - y2) - (y0 - y2) * (px - x2)) * inv
                if w1 < 0.0:
                    continue
                w2 = 1.0 - w0 - w1
                if w2 < 0.0:
                    continue
                here = w0 * oa + w1 * ob + w2 * og
                if here > buf[row + px]:
                    buf[row + px] = here
    return buf


#: Backdrop samples per window size. The ground never moves - it is the
#: camera's world, not the board's - so the projection is computed once
#: per (width, height) and each frame only replays it against the
#: depth buffer.
_BACKDROP = {}


def _backdrop(width, height, distance, view):
    """[(px, py, stroke)] for the ground grid: own scale, own centre.

    NOT the board's projection. The fitted scale magnifies a dinner
    plate to fill the frame, and at that magnification the horizon
    projects some forty rows above the window - what stayed on screen
    was the strip of rungs around the board, which read as two horizons
    with the far half exiting through the top. The backdrop shares the
    camera's tip but picks its scale from two anchors: the horizon near
    the top of the frame, the ground under the camera just off the
    bottom, so lines always rise from the lower edge and terminate ON
    the horizon."""
    got = _BACKDROP.get((width, height))
    if got is not None:
        return got

    v0, v1, v2, v3, v4, v5, v6, v7, v8 = view
    south, far = -5.0, 30.0

    def ray(wx, wy):
        tx = v0 * wx + v1 * wy + v2 * GROUND
        ty = v3 * wx + v4 * wy + v5 * GROUND
        tz = v6 * wx + v7 * wy + v8 * GROUND
        w = 1.0 / (distance - tz)
        return w * tx, w * ty, w

    hrow = max(1.0, height * 0.15)
    _x, top, _w = ray(0.0, far)
    _x, toe, _w = ray(0.0, 0.0)
    scale = (height * 1.08 - hrow) / (0.5 * (top - toe))
    cy = hrow + scale * 0.5 * top
    cx = width / 2.0

    def cast(wx, wy):
        # ray() already carries the perspective weight in its x and y.
        sx, sy, w = ray(wx, wy)
        return int(cx + scale * sx), int(cy - scale * 0.5 * sy), w

    marks = []
    hy = int(round(hrow))
    for px in range(width):
        marks.append((px, hy, '-'))

    # Meridians to the vanishing point, stroked in the direction they
    # travel; t*t sampling is densest at the south end where one line
    # covers the most rows. Points at or behind the camera plane mirror
    # through the projection - the w guard drops them before they paint.
    # THIRTEEN lines: the stylised attitude-indicator fan, not a map.
    samples = height * 6
    for k in range(-6, 7):
        fixed = k * 1.6
        last = None
        for i in range(samples + 1):
            wy = south + (far - south) * (i / samples) ** 2
            px, py, w = cast(fixed, wy)
            if w <= 0.0 or w > 2.0:
                continue
            stroke = ('|' if last is None or (px, py) == last
                      else _glyph(px - last[0], py - last[1]))
            last = (px, py)
            # Converging lines pile a solid wall into the rows under the
            # horizon; thin them with distance instead - every fourth
            # survives to the top, every second to the mid band.
            if py <= hy + 2 and k % 4 != 0:
                continue
            if py <= hy + 5 and k % 2 != 0:
                continue
            if 0 <= px < width and hy < py < height:
                marks.append((px, py, stroke))
    # A few SHORT rungs, clipped to the middle of the fan - edge-to-edge
    # lines read as a map grid, and the wanted look is the stylised
    # horizon of an attitude indicator. A rung landing on a taken row or
    # its neighbour is dropped: the fade IS the distance cue.
    taken = [hy]
    rung = south
    while rung < far:
        at = rung
        rung += 5.0
        _sx, ry, rw = cast(0.0, at)
        if rw <= 0.0 or rw > 2.0:
            continue
        if any(abs(ry - r) <= 1 for r in taken):
            continue
        taken.append(ry)
        for i in range(width + 1):
            wx = -7.0 + 14.0 * i / width
            px, py, w = cast(wx, at)
            if 0.0 < w <= 2.0 and 0 <= px < width and hy < py < height:
                marks.append((px, py, '-'))

    _BACKDROP[(width, height)] = marks
    return marks


def _ground(grid, tone, buf, distance, width, height, colour, view):
    """The landscape behind the board: the cached backdrop, replayed
    against this frame's depth buffer so the board occludes it."""
    ash = 240 if colour else None
    for px, py, stroke in _backdrop(width, height, distance, view):
        if buf[py * width + px] == 0.0 and grid[py][px] == ' ':
            grid[py][px] = stroke
            tone[py][px] = ash



#: The board's face as a shipped ASCII raster - a dithered render of the
#: real layout, boardface.txt beside this module. Sampled as a texture
#: on the board plane, so the picture turns WITH the attitude instead of
#: a cage of edges suggesting it.
_FACE = None


def _face():
    global _FACE
    if _FACE is None:
        import os
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'boardface.txt')
        try:
            with open(path, encoding='ascii') as art:
                rows = art.read().splitlines()
        except OSError:
            rows = []
        wide = max((len(r) for r in rows), default=0)
        _FACE = ([r.ljust(wide) for r in rows], wide, len(rows))
    return _FACE


def _skin(grid, tone, buf, m, distance, scale, cx, cy,
          width, height, colour):
    """The face raster mapped onto the board plane.

    Every visible cell casts a ray from the eye, hits z=0 in BODY frame
    and samples the art at the hit - the dither turns with the board.
    With the solder side toward the camera the sample mirrors in x, the
    way a translucent card would read from behind."""
    rows, art_w, art_h = _face()
    if not art_w:
        return
    back = m[8] < 0.0
    ramp = SHADES['board']
    ink = {'.': ramp[0], ':': ramp[1], '*': ramp[2]}
    for py in range(height):
        row = py * width
        v = (cy - (py + 0.5)) / (scale * 0.5)
        for px in range(width):
            if buf[row + px] == 0.0:
                continue
            u = (px + 0.5 - cx) / scale
            dz = m[2] * u + m[5] * v - m[8]
            if abs(dz) < 1e-9:
                continue
            t = -(distance * m[8]) / dz
            if t <= 0.0:
                continue
            hx = distance * m[6] + t * (m[0] * u + m[3] * v - m[6])
            hy = distance * m[7] + t * (m[1] * u + m[4] * v - m[7])
            if hx * hx + hy * hy > OUTER * OUTER:
                continue
            if back:
                hx = -hx
            ch = rows[int((1.0 - (hy / OUTER + 1.0) * 0.5) * (art_h - 1))][
                int((hx / OUTER + 1.0) * 0.5 * (art_w - 1))]
            if ch == ' ':
                continue
            grid[py][px] = ch
            tone[py][px] = ink.get(ch, ramp[1]) if colour else None


def _glyph(dx, dy):
    """The stroke of a screen direction. dy is in cell rows (half-height)."""
    if dx == 0.0 and dy == 0.0:
        return '+'
    steep = abs(dy) * 2.0
    if steep > 2.0 * abs(dx):
        return '|'
    if abs(dx) > 2.0 * steep:
        return '-'
    return '/' if (dx > 0) != (dy > 0) else '\\'


#: The body axes: +X carries the DC link (right of the board), +Y the
#: three phase connectors (top), +Z out of the component face. One colour
#: each, the labels drawn at the tips.
AXES = (((1.45, 0.0, 0.0), 'X', 208),
        ((0.0, 1.45, 0.0), 'Y', 114),
        ((0.0, 0.0, 0.9), 'Z', 51))


def render(q, width, height, zoom=1.0, colour=True, axes=True,
           horizon=True, face=True):
    """The board under rotation `q`, as a vector drawing.

    Cell-resolution: the strokes ARE the picture, so there is no half-block
    supersampling to average them away. `axes` draws the BODY frame - the
    labels turn with the board, which is what makes a mirrored mount
    readable at a glance. `horizon` draws the WORLD's level line behind
    everything, the old flight-sim cue: the board tilts, the horizon does
    not."""
    from . import ansi, orientation

    edges, solid = _model()
    # A NEAR-TOP camera, tipped just enough that the world grid recedes
    # to a horizon the way the old flight sims drew it. The tip is the
    # CAMERA's, applied outside the attitude, so screen X and Y still map
    # one-to-one onto board X and Y - the 55-degree gallery view mixed
    # them and sent the mount hunting through mirrors.
    tip = math.radians(CAMERA_TIP)
    ct, st = math.cos(tip), math.sin(tip)
    view = (1.0, 0.0, 0.0,
            0.0, ct, st,
            0.0, -st, ct)
    m = orientation._multiply(view, orientation.matrix(q))

    distance = 3.2
    # The fit is MEASURED, not derived: every endpoint is projected once at
    # unit scale, and the scale and centre come from the spans. Exact for
    # any attitude, so zoom 1.0 always just fills the frame.
    xs, ys = [], []
    spanning = [v for a, b, _zone in edges for v in (a, b)]
    if axes:
        spanning += [tip for tip, _l, _i in AXES]
    for v in spanning:
        if True:
            tx = m[0] * v[0] + m[1] * v[1] + m[2] * v[2]
            ty = m[3] * v[0] + m[4] * v[1] + m[5] * v[2]
            tz = m[6] * v[0] + m[7] * v[1] + m[8] * v[2]
            w = 1.0 / (distance - tz)
            xs.append(w * tx)
            ys.append(0.5 * w * ty)
    x_span = (max(xs) - min(xs)) or 1.0
    y_span = (max(ys) - min(ys)) or 1.0
    scale = zoom * min(0.94 * width / x_span, 0.92 * height / y_span)
    cx = width / 2.0 - scale * (max(xs) + min(xs)) / 2.0
    cy = height / 2.0 + scale * (max(ys) + min(ys)) / 2.0

    buf = _depth(solid, m, distance, scale, cx, cy, width, height)
    z_lo = min((v for v in buf if v), default=0.0)
    z_hi = max(buf, default=1.0) or 1.0
    span = (z_hi - z_lo) or 1.0

    grid = [[' '] * width for _ in range(height)]
    tone = [[None] * width for _ in range(height)]

    if horizon:
        tipm = view
        _ground(grid, tone, buf, distance, width, height, colour, tipm)

    skinned = False
    if face:
        _skin(grid, tone, buf, m, distance, scale, cx, cy,
              width, height, colour)
        skinned = _face()[1] > 0

    m0, m1, m2, m3, m4, m5, m6, m7, m8 = m
    for a, b, zone in edges:
        # The skin already carries the components as dither - the cages
        # on top of it were double vision. The rim and bore stay: they
        # are the crisp silhouette the dither cannot draw.
        if skinned and zone != 'board':
            continue
        ax = m0 * a[0] + m1 * a[1] + m2 * a[2]
        ay = m3 * a[0] + m4 * a[1] + m5 * a[2]
        az = m6 * a[0] + m7 * a[1] + m8 * a[2]
        bx = m0 * b[0] + m1 * b[1] + m2 * b[2]
        by = m3 * b[0] + m4 * b[1] + m5 * b[2]
        bz = m6 * b[0] + m7 * b[1] + m8 * b[2]

        wa, wb = 1.0 / (distance - az), 1.0 / (distance - bz)
        x0, y0 = cx + scale * wa * ax, cy - scale * 0.5 * wa * ay
        x1, y1 = cx + scale * wb * bx, cy - scale * 0.5 * wb * by

        steps = max(1, int(max(abs(x1 - x0), 2.0 * abs(y1 - y0))))
        glyph = _glyph(x1 - x0, y1 - y0)
        shades = SHADES.get(zone, SHADES['board'])
        for i in range(steps + 1):
            t = i / steps
            px, py = int(x0 + (x1 - x0) * t), int(y0 + (y1 - y0) * t)
            if not (0 <= px < width and 0 <= py < height):
                continue
            here = wa + (wb - wa) * t
            # The solid decides visibility; the 2 % grace keeps an edge
            # from losing to the very face it borders.
            if here < buf[py * width + px] * 0.98:
                continue
            third = min(2, int(3.0 * (here - z_lo) / span))
            grid[py][px] = glyph
            tone[py][px] = shades[third] if colour else None

    if axes:
        # The body frame, drawn over everything: three strokes out of the
        # bore, each ending in its letter, turning with the board.
        for tip, label, ink in AXES:
            tx = m0 * tip[0] + m1 * tip[1] + m2 * tip[2]
            ty = m3 * tip[0] + m4 * tip[1] + m5 * tip[2]
            tz = m6 * tip[0] + m7 * tip[1] + m8 * tip[2]
            w = 1.0 / (distance - tz)
            x1, y1 = cx + scale * w * tx, cy - scale * 0.5 * w * ty
            steps = max(1, int(max(abs(x1 - cx), 2.0 * abs(y1 - cy))))
            glyph = _glyph(x1 - cx, y1 - cy)
            for i in range(steps + 1):
                t = i / steps
                px, py = int(cx + (x1 - cx) * t), int(cy + (y1 - cy) * t)
                if 0 <= px < width and 0 <= py < height and t > 0.12:
                    grid[py][px] = glyph
                    tone[py][px] = ink if colour else None
            lx, ly = int(x1), int(y1)
            if 0 <= lx < width and 0 <= ly < height:
                grid[ly][lx] = label
                tone[ly][lx] = ink if colour else None

    if not colour:
        return '\n'.join(''.join(row).rstrip() for row in grid)
    return '\n'.join(
        ansi.run([(grid[r][c], tone[r][c]) for c in range(width)]).rstrip()
        for r in range(height))
