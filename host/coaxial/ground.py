"""The landscape behind the board: a horizon, a fan of lines to it and
rungs sliding toward the camera - the camera's world, not the board's,
so it is cast once per window size and per scroll step and replayed
against each frame's depth buffer. `_ground` is the one entry."""
import math

from .raster import BRAILLE, BRAILLE_BITS


#: The ground plane's height under the board, model units.
GROUND = -1.3


#: Backdrop cells per (window size, scroll step). The ground is the
#: camera's world, not the board's: it does not turn with the
#: attitude, so a backdrop is cast once per size and step and each
#: frame only replays it against the depth buffer.
_BACKDROP = {}


#: The backdrop's greys, 24-bit so the haze is continuous: a fan line
#: at the camera's feet and the horizon itself. AERIAL PERSPECTIVE - a
#: line's contrast falls as exp(-depth / L), and FAN_HAZE is what is
#: left of it at the horizon. Flat-toned, the fan got BRIGHTER toward
#: the horizon ("more realistic", the bench asked): the lines converge,
#: several land in one cell, and a cell of six dots at one tone is
#: three times the ink of one with two. So a cell's grey is divided by
#: the square root of the LINES through it - lines, not dots: a steep
#: line puts four dots in a cell and a shallow one two, the same ink
#: per length of line, and counting dots would have thinned the steep
#: ones. Measured at 150x44, a row's ink per column (its cells' grey x
#: dots over the width): flat at grey 68, the row under the horizon
#: carried 62 against 7 at the frame's foot - nine to one, the
#: convergence itself; hazed and divided, 47 against 12. The far band
#: is still denser, as a real floor's is, but faint now, not bright.
FAN_GREY = 118
FAN_HAZE = 0.4
HORIZON_GREY = 128


#: THE GROUND MOVES. Rungs across the fan - the grid's cross lines,
#: one every RUNG_SPACING of world, the fan's own pitch so the floor
#: is square - slide toward the camera at GROUND_SPEED spacings a
#: second, and the scene drifts slowly forward (a bonus the bench
#: asked for). A rung's phase is quantised to RUNG_STEPS per spacing
#: so a backdrop is built once per step and replayed: the horizon and
#: the fan are cast once per window size (_BACKDROP_STATIC), the rungs
#: added per step (_BACKDROP). Near the horizon the rungs crowd - at
#: 150x44 the horizon sits at row 6.6 and the far rungs at 7.2, 7.9,
#: 8.7, 9.7 - so each fades by the rows to the next, RUNG_FADE rows
#: for its full grey, and the far ones dissolve into the haze instead
#: of stacking into a bar. A rung under half its grey stays out of a
#: cell another line already holds: one tone per cell would lend it
#: the fan's.
RUNG_SPACING = 2.2
GROUND_SPEED = 0.2
#: Ninety-six steps a spacing: at 24 a rung stood for 0.21 s and
#: jumped, the near one 1.7 dot rows at a time - "static", the bench
#: said of the floor (2026-09-23); at 96 it moves every 52 ms by under
#: half a dot row (0.105 rows measured at 108x44), on the sub-row the
#: geometry says. A step is built once and kept; two window sizes'
#: worth, then the cache starts over.
RUNG_STEPS = 96
RUNG_FADE = 1.5
BACKDROPS_KEPT = 2 * RUNG_STEPS
_BACKDROP_STATIC = {}


def _ground_dot(cells, width, height, fx, fy, depth, line, fade=1.0):
    """One braille dot into the backdrop's cells: the mask, whether the
    horizon runs through the cell, the sum of its dots' depths, the dot
    count, the lines through it and the brightest line's fade."""
    if fx < 0.0 or fy < 0.0:
        return
    px, py = int(fx), int(fy)
    if px >= width or py >= height:
        return
    at = py * width + px
    cell = cells.get(at)
    if cell is None:
        cell = cells[at] = [0, False, 0.0, 0, set(), fade]
    elif fade < 0.5 and cell[4]:
        return
    col = 1 if fx - px >= 0.5 else 0
    row = min(3, int((fy - py) * 4.0))
    cell[0] |= BRAILLE_BITS[col][row]
    cell[1] |= line == 'horizon'
    cell[2] += depth
    cell[3] += 1
    cell[4].add(line)
    cell[5] = max(cell[5], fade)


def _rungs(static, phase):
    """The rungs at `phase` (0..1 of a spacing, toward the camera):
    [(row at the frame's centre, w, x0, y0, w0, x1, y1, w1)] from the
    nearest out, each spanning the fan's width. Rungs beyond the
    frame's foot are listed too - the fade of the last visible one
    is measured against the next."""
    cast, south, far = static['cast'], static['south'], static['far']
    out = []
    for k in range(int((far - south) / RUNG_SPACING)):
        wy = south + (k + 1.0 - phase) * RUNG_SPACING
        _cx, yc, wc = cast(0.0, wy)
        x0, y0, w0 = cast(-8.0 * RUNG_SPACING, wy)
        x1, y1, w1 = cast(8.0 * RUNG_SPACING, wy)
        out.append((yc, wc, x0, y0, w0, x1, y1, w1))
    return out


def _backdrop(width, height, distance, view, phase=0.0):
    """{cell: (braille mask, grey (r, g, b))} for the ground grid at a
    phase of its scroll: the static horizon and fan, the rungs at the
    phase's step, greys settled per cell. Cached per size and step.

    A step is the static's settled cells with the rungs' cells laid
    over: only the cells a rung touches are copied and greyed again.
    Copying every static cell and greying the lot cost 6.7 ms a cold
    step at 108x44 (9.2 at 150x44), and at 96 steps a spacing a cold
    step falls every 52 ms for the first five seconds of a size."""
    step = int(phase * RUNG_STEPS + 0.5) % RUNG_STEPS
    key = (width, height, step)
    got = _BACKDROP.get(key)
    if got is not None:
        return got
    if len(_BACKDROP) >= BACKDROPS_KEPT:
        _BACKDROP.clear()
    static = _BACKDROP_STATIC.get((width, height))
    if static is None:
        static = _ground_static(width, height, distance, view)
        static['masks'] = _greys(static['cells'], static)
        _BACKDROP_STATIC[(width, height)] = static
    masks = dict(static['masks'])
    masks.update(_greys(_rung_cells(static, width, height,
                                    step / RUNG_STEPS), static))
    _BACKDROP[key] = masks
    return masks


def _rung_cells(static, width, height, phase):
    """The cells the rungs at `phase` touch, in _ground_dot's shape,
    each seeded from the static cell under it - a rung under half its
    grey stays out of a cell another line already holds, the static's
    included. The dots are placed here rather than through _ground_dot:
    3 240 calls a step were most of its cost."""
    cells = {}
    under = static['cells']
    hrow = static['hrow']
    rungs = _rungs(static, phase)
    for k, (yc, wc, x0, y0, w0, x1, y1, w1) in enumerate(rungs):
        if wc <= 0.0 or wc > 2.0 or yc < hrow:
            continue
        beyond = rungs[k + 1][0] if k + 1 < len(rungs) else hrow
        fade = min(1.0, max(0.0, (yc - beyond) / RUNG_FADE))
        if fade < 0.1:
            continue
        depth = 1.0 / wc
        faint = fade < 0.5
        line = ('rung', k)
        for half in range(2 * width):
            fx = half / 2.0 + 0.25
            t = (fx - x0) / (x1 - x0)
            if not 0.0 <= t <= 1.0:
                continue
            fy = y0 + (y1 - y0) * t
            if fy < hrow or fy >= height:
                continue
            py = int(fy)
            at = py * width + (half >> 1)
            cell = cells.get(at)
            if cell is None:
                base = under.get(at)
                if base is None:
                    cell = cells[at] = [0, False, 0.0, 0, set(), fade]
                elif faint:
                    continue
                else:
                    cell = cells[at] = [base[0], base[1], base[2], base[3],
                                        set(base[4]), base[5]]
            elif faint and cell[4]:
                continue
            cell[0] |= BRAILLE_BITS[half & 1][min(3, int((fy - py) * 4.0))]
            cell[2] += depth
            cell[3] += 1
            cell[4].add(line)
            cell[5] = max(cell[5], fade)
    return cells


def _greys(cells, static):
    """{cell: (mask, grey (r, g, b))}: the horizon's own grey, or the
    fan's hazed by the cell's mean depth; either divided by the lines
    through the cell, and dimmed by the brightest line's fade."""
    near_depth = static['near_depth']
    reach = max(1e-9, static['far_depth'] - near_depth)
    masks = {}
    for at, (mask, horizon, depth, dots, lines, fade) in cells.items():
        if horizon:
            grey = HORIZON_GREY
        else:
            grey = FAN_GREY * FAN_HAZE ** ((depth / dots - near_depth) / reach)
        grey = int(grey * fade / math.sqrt(len(lines)) + 0.5)
        masks[at] = (mask, (grey, grey, grey))
    return masks


def _ground_static(width, height, distance, view):
    """The horizon and the fan for a window size: own scale, own
    centre, cast once. {'cells', 'cast', 'hrow', 'south', 'far',
    'near_depth', 'far_depth'} - what _backdrop adds the rungs to.

    NOT the board's projection. The fitted scale magnifies a dinner
    plate to fill the frame, and at that magnification the horizon
    projects some forty rows above the window - what stayed on screen
    was the strip of rungs around the board, which read as two horizons
    with the far half exiting through the top. The backdrop shares the
    camera's tip but picks its scale from two anchors: the horizon near
    the top of the frame, the ground under the camera just off the
    bottom, so lines always rise from the lower edge and terminate ON
    the horizon."""
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
        # FLOAT cells: the dots below land on the sub-row and sub-column
        # the geometry says, not the cell it rounds to.
        sx, sy, w = ray(wx, wy)
        return cx + scale * sx, cy - scale * 0.5 * sy, w

    # The backdrop is BRAILLE, like the outline: a 2x4 dot matrix per
    # cell. The horizon is one row of dots at its exact sub-row across
    # the width; the fan's lines are sampled and JOINED at the matrix's
    # pitch, so a line is a fine continuous run of dots rather than the
    # one dot per row it was - which alternated `.` cells with blanks on
    # every shallow slope and read as a stair. Every line runs to the
    # horizon's own sub-row and meets it: a gap of three sub-rows was
    # left under the horizon first, and the bench saw the lines "stop
    # before they reach the horizon".
    cells = {}

    def dot(fx, fy, depth, line):
        _ground_dot(cells, width, height, fx, fy, depth, line)

    _x, _y, w_far = ray(0.0, far)
    far_depth = 1.0 / w_far
    for half in range(2 * width):
        dot(half / 2.0 + 0.25, hrow, far_depth, 'horizon')

    # Seventeen lines, wide enough that the outer ones meet the horizon
    # past the frame edges - the fan covers the WHOLE line, not a band
    # in the middle. Dotted rather than stroked: directional glyphs
    # alternated `/-` `|-` on every shallow slope and came out ragged,
    # and a dotted floor under a block-shaded subject is the separation
    # itself.
    samples = height * 6
    near_depth = far_depth
    for k in range(-8, 9):
        fixed = k * 2.2
        prev = None
        for i in range(samples + 1):
            wy = south + (far - south) * (i / samples) ** 2
            fx, fy, w = cast(fixed, wy)
            if w <= 0.0 or w > 2.0 or fy < hrow:
                prev = None
                continue
            depth = 1.0 / w
            if prev is not None:
                _segment(dot, prev, fx, fy, depth, k)
            if prev is not None and fy < height:
                near_depth = min(near_depth, depth)
            prev = (fx, fy, depth)

    return {'cells': cells, 'cast': cast, 'hrow': hrow, 'south': south,
            'far': far, 'near_depth': near_depth, 'far_depth': far_depth}


def _ground(grid, tone, buf, distance, width, height, colour, view,
            phase=0.0):
    """The landscape behind the board: the cached backdrop at this
    phase of its scroll, replayed against this frame's depth buffer so
    the board occludes it.

    Plain occlusion, no halo: the depth solid's footprint already
    reaches a cell or two past the visible dither, and the one-cell
    keepout tried on top of that clipped the backdrop visibly far from
    the subject. Solid blocks against dim dots need no gap to separate."""
    for at, (mask, grey) in _backdrop(width, height, distance, view,
                                      phase).items():
        r, c = divmod(at, width)
        if buf[at] == 0.0 and grid[r][c] == ' ':
            grid[r][c] = chr(BRAILLE + mask)
            tone[r][c] = grey if colour else None


def _segment(dot, prev, fx, fy, depth, k):
    """The ground line from `prev` to here: EVERY dot it passes through,
    its depth interpolated for the buffer.

    Sampled one a dot along its steeper axis it came out dashed: each
    piece's count was truncated, so a piece 1.9 rows tall lit two dots
    and skipped one, and a diagonal stepping a dot column left a gap at
    the step - the bench saw the fan as jagged (2026-09-23). The pieces
    between the dot-column and dot-row crossings each lie inside one
    dot, so their midpoints are the line's supercover: a chain of
    touching dots. Twelve percent more ink at 108x44 (4010 dots to
    4512), cast once per window size."""
    dx, dy, dd = fx - prev[0], fy - prev[1], depth - prev[2]
    ts = {0.0, 1.0}
    if dx:
        a, b = sorted((prev[0] * 2.0, fx * 2.0))
        for i in range(math.floor(a) + 1, math.ceil(b)):
            ts.add((i / 2.0 - prev[0]) / dx)
    if dy:
        a, b = sorted((prev[1] * 4.0, fy * 4.0))
        for i in range(math.floor(a) + 1, math.ceil(b)):
            ts.add((i / 4.0 - prev[1]) / dy)
    ts = sorted(ts)
    for t0, t1 in zip(ts, ts[1:]):
        t = 0.5 * (t0 + t1)
        dot(prev[0] + dx * t, prev[1] + dy * t, prev[2] + dd * t, k)
