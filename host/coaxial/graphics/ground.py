"""The landscape behind the board: a bowed horizon, a dashed fan, rungs and gates toward the camera.

The camera's world, not the board's: cast once per window size and scroll
step, replayed against each frame's depth buffer.
"""
import math

from coaxial.graphics import approach
from coaxial.graphics.raster import BRAILLE, BRAILLE_BITS


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
#: the horizon, as a real one does: the lines converge,
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
RUNG_SPACING = 1.65
GROUND_SPEED = 0.2
#: Ninety-six steps a spacing: at 24 a rung stood for 0.21 s and
#: jumped, the near one 1.7 dot rows at a time - "static", the bench
#: said of the floor; at 96 it moves every 52 ms by under
#: half a dot row (0.105 rows measured at 108x44), on the sub-row the
#: geometry says. A step is built once and kept; two window sizes'
#: worth, then the cache starts over.
RUNG_STEPS = 96
RUNG_FADE = 1.5
BACKDROPS_KEPT = 2 * RUNG_STEPS
_BACKDROP_STATIC = {}
#: The floor's polylines per (window size, scroll step), flat or to be rolled.
_RUNS = {}

#: THE FAN MOVES WITH THE RUNGS. Its lines are dashed, DASH of each spacing lit from a
#: rung outward, and the dashes scroll toward the camera with the rungs: the floor reads
#: as travelled, not drawn.
#: Past DASHED spacings the fan runs solid into the haze - dashes there are under a dot
#: row and shimmer. The fan's dots are cast once a window size; a step keeps its dashes.
DASH = 0.5
DASHED = 6

#: Fan lines each side of the centre: far enough out that the outer ones meet the
#: horizon past the frame's edges.
FAN_HALF = 11

#: THE HORIZON BOWS: a planet under the approach, BOW of the height lower at the frame's
#: edges than at its centre, a parabola; the floor stops at it.
BOW = 0.09


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
    """The rungs at `phase` (0..1 of a spacing, toward the camera): [(row at
    the frame's centre, w, x0, y0, w0, x1, y1, w1)] from the nearest out,
    each spanning the fan's width.
    """
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
    """{cell: (braille mask, grey (r, g, b))} for the ground grid at a phase
    of its scroll: the horizon, the fan's dashes and the rungs at the phase's
    step, greys settled per cell.
    """
    step = int(phase * RUNG_STEPS + 0.5) % RUNG_STEPS
    key = (width, height, step)
    got = _BACKDROP.get(key)
    if got is not None:
        return got
    if len(_BACKDROP) >= BACKDROPS_KEPT:
        _BACKDROP.clear()
    static = _static(width, height, distance, view)
    laid = _laid(_runs(static, width, height, step), width, height)
    masks = {} if laid is None else {at: (mk, (g, g, g)) for at, mk, g in zip(
        laid[0].tolist(), laid[1].tolist(), _shaded(laid, static).tolist())}
    _BACKDROP[key] = masks
    return masks


def _static(width, height, distance, view):
    """The window size's horizon, camera and fan (`_ground_static`), cast once."""
    static = _BACKDROP_STATIC.get((width, height))
    if static is None:
        static = _BACKDROP_STATIC[(width, height)] = _ground_static(width, height, distance,
                                                                    view)
    return static


def _runs(static, width, height, step):
    """The floor at a scroll step as polylines, (line, [(fx, fy, depth)], fade), in the
    order _ground_dot's faint rule wants: the horizon, the fan's dashes, the rungs. Kept a
    step (`_RUNS`)."""
    key = (width, height, step)
    got = _RUNS.get(key)
    if got is not None:
        return got
    if len(_RUNS) >= BACKDROPS_KEPT:
        _RUNS.clear()
    phase = step / RUNG_STEPS
    south, sky = static['south'], static['sky']
    solid = south + DASHED * RUNG_SPACING
    runs: list = [('horizon', static['horizon'], 1.0)]
    for line, points in static['fan']:
        run = []
        for p, q in zip(points, points[1:]):
            for t0, t1 in _lit(p[3], q[3], south, phase, solid):
                a = tuple(p[i] + (q[i] - p[i]) * t0 for i in range(3))
                if run and t0 > 0.0:
                    runs.append((line, run, 1.0))
                    run = []
                run = run or [a]
                run.append(tuple(p[i] + (q[i] - p[i]) * t1 for i in range(3)))
                if t1 < 1.0:
                    runs.append((line, run, 1.0))
                    run = []
        if len(run) > 1:
            runs.append((line, run, 1.0))
    rungs = _rungs(static, phase)
    for k, (yc, wc, x0, y0, _w0, x1, y1, _w1) in enumerate(rungs):
        if wc <= 0.0 or wc > 2.0 or yc < static['hrow']:
            continue
        beyond = rungs[k + 1][0] if k + 1 < len(rungs) else static['hrow']
        fade = min(1.0, max(0.0, (yc - beyond) / RUNG_FADE))
        if fade < 0.1:
            continue
        run = []
        for col in range(width + 1):
            t = (col - x0) / (x1 - x0)
            fy = y0 + (y1 - y0) * t
            if 0.0 <= t <= 1.0 and sky(col) <= fy:
                run.append((float(col), fy, 1.0 / wc))
            elif len(run) > 1:
                runs.append((('rung', k), run, fade))
                run = []
            else:
                run = []
        if len(run) > 1:
            runs.append((('rung', k), run, fade))
    _RUNS[key] = runs
    return runs


def _lit(wy0, wy1, south, phase, solid):
    """The pieces of a fan segment from world y `wy0` to `wy1` a dash lights, as fractions
    [(t0, t1)]: DASH of each spacing from its rung out, all of it past `solid`."""
    if wy1 == wy0:
        return [(0.0, 1.0)]
    u0, u1 = sorted(((wy - south) / RUNG_SPACING + phase) for wy in (wy0, wy1))
    edges = {u0, u1}
    for n in range(int(math.floor(u0)), int(math.ceil(u1)) + 1):
        edges.update(e for e in (n, n + DASH) if u0 < e < u1)
    us = ((solid - south) / RUNG_SPACING + phase)
    if u0 < us < u1:
        edges.add(us)
    edges = sorted(edges)
    out = []
    for e0, e1 in zip(edges, edges[1:]):
        mid = 0.5 * (e0 + e1)
        if mid >= us or mid % 1.0 < DASH:
            t0, t1 = ((e0 - u0) / (u1 - u0), (e1 - u0) / (u1 - u0))
            if wy1 < wy0:
                t0, t1 = 1.0 - t1, 1.0 - t0
            if out and abs(out[-1][1] - t0) < 1e-9:
                out[-1] = (out[-1][0], t1)
            else:
                out.append((t0, t1))
    return sorted(out)


def _crossings(c0, dc, c1, scale):
    """_segment's crossings of the dot grid along one axis, every segment at once: (segment,
    t) for each `i / scale` strictly between a segment's ends, `scale` dots a cell."""
    from coaxial.model.blocks import numpy as np
    a, b = np.minimum(c0 * scale, c1 * scale), np.maximum(c0 * scale, c1 * scale)
    first = np.floor(a) + 1.0
    count = np.where(dc != 0.0, np.maximum(np.ceil(b) - first, 0.0), 0.0).astype(np.int64)
    seg = np.repeat(np.arange(len(dc)), count)
    i = (np.repeat(first, count) + np.arange(int(count.sum()))
         - np.repeat(np.cumsum(count) - count, count))
    return seg, (i / scale - c0[seg]) / dc[seg]


def _laid(runs, width, height, roll=None):
    """Polylines into _ground_dot's cells, every dot they pass through - each point moved
    by `roll` first, when given, so a rolled line is drawn, not a line's dots moved. Every
    segment at once (numpy), each dot where _segment and _ground_dot put it and in their
    order, a faint dot landing only where none before it has. The cells as arrays in the
    order their first dot landed: cell, mask, horizon, depth sum, dots, lines through it, the
    brightest fade, and the (cell, line) pairs; None for no line."""
    from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
    runs = [run for run in runs if len(run[1]) > 1]
    if not runs:
        return None
    labels: dict = {}
    ids = np.array([labels.setdefault(line, len(labels)) for line, _p, _f in runs])
    fades = np.array([fade for _l, _p, fade in runs], dtype=np.float64)
    lengths = np.array([len(points) for _l, points, _f in runs])
    p = np.array([point for _l, points, _f in runs for point in points], dtype=np.float64)
    fx, fy, fd = p[:, 0], p[:, 1], p[:, 2]
    if roll is not None:
        fx, fy = roll(fx, fy)
    start = np.ones(len(p), dtype=bool)
    start[np.cumsum(lengths) - 1] = False
    s = np.flatnonzero(start)
    x0, y0, d0 = fx[s], fy[s], fd[s]
    dx, dy, dd = fx[s + 1] - x0, fy[s + 1] - y0, fd[s + 1] - d0
    run_of = np.repeat(np.arange(len(runs)), lengths - 1)
    n = len(s)
    sx, tx = _crossings(x0, dx, fx[s + 1], 2.0)
    sy, ty = _crossings(y0, dy, fy[s + 1], 4.0)
    seg = np.concatenate([np.arange(n), np.arange(n), sx, sy])
    t = np.concatenate([np.zeros(n), np.ones(n), tx, ty])
    order = np.lexsort((t, seg))
    seg, t = seg[order], t[order]
    fresh = np.ones(len(t), dtype=bool)
    fresh[1:] = (seg[1:] != seg[:-1]) | (t[1:] != t[:-1])
    seg, t = seg[fresh], t[fresh]
    pair = seg[1:] == seg[:-1]
    sg = seg[:-1][pair]
    t = 0.5 * (t[:-1][pair] + t[1:][pair])
    gx, gy, gd = x0[sg] + dx[sg] * t, y0[sg] + dy[sg] * t, d0[sg] + dd[sg] * t
    inside = (gx >= 0.0) & (gy >= 0.0)
    px, py = gx.astype(np.int64), gy.astype(np.int64)
    inside &= (px < width) & (py < height)
    gx, gy, gd, sg = gx[inside], gy[inside], gd[inside], sg[inside]
    px, py = px[inside], py[inside]
    at = py * width + px
    col = (gx - px >= 0.5).astype(np.int64)
    row = np.minimum(3, ((gy - py) * 4.0).astype(np.int64))
    bit = np.asarray(BRAILLE_BITS)[col, row]
    line, fade = ids[run_of[sg]], fades[run_of[sg]]
    landed = fade >= 0.5
    landed[np.unique(at, return_index=True)[1]] = True
    at, bit, line = at[landed], bit[landed], line[landed]
    gd, fade = gd[landed], fade[landed]
    cell, first, inv = np.unique(at, return_index=True, return_inverse=True)
    m = len(cell)
    mask, horizon, depth = np.zeros(m, dtype=np.int64), np.zeros(m, dtype=bool), np.zeros(m)
    np.bitwise_or.at(mask, inv, bit)
    np.logical_or.at(horizon, inv, line == labels.get('horizon', -1))
    np.add.at(depth, inv, gd)
    dots = np.bincount(inv, minlength=m)
    faded = np.full(m, -np.inf)
    np.maximum.at(faded, inv, fade)
    held = np.unique(inv.astype(np.int64) * len(labels) + line)
    order = np.argsort(first, kind='stable')
    rank = np.empty(m, np.int64)
    rank[order] = np.arange(m)
    lines = np.bincount(held // len(labels), minlength=m)
    return (cell[order], mask[order], horizon[order], depth[order], dots[order], lines[order],
            faded[order], (rank[held // len(labels)], held % len(labels)))


def _lay(runs, width, height, roll=None):
    """`_laid`'s cells as _ground_dot keeps them: {cell: [mask, horizon, depth, dots, lines,
    fade]} in landing order, the lines a set."""
    laid = _laid(runs, width, height, roll)
    if laid is None:
        return {}
    cell, mask, horizon, depth, dots, _count, fade, (owner, line) = laid
    lines: list = [set() for _ in range(len(cell))]
    for j, k in zip(owner.tolist(), line.tolist()):
        lines[j].add(k)
    return {at: [mk, hz, d, n, ls, fd] for at, mk, hz, d, n, ls, fd in zip(
        cell.tolist(), mask.tolist(), horizon.tolist(), depth.tolist(), dots.tolist(), lines,
        fade.tolist())}


def _shaded(laid, static):
    """`_greys` on `_laid`'s arrays: each cell's grey, the haze's power Python's own."""
    from coaxial.model.blocks import numpy as np
    _cell, _mask, horizon, depth, dots, lines, fade, _pairs = laid
    near_depth = static['near_depth']
    reach = max(1e-9, static['far_depth'] - near_depth)
    grey = np.full(len(depth), float(HORIZON_GREY))
    fan = np.flatnonzero(~horizon)
    hazed = ((depth[fan] / dots[fan] - near_depth) / reach).tolist()
    grey[fan] = FAN_GREY * np.fromiter(map(pow, [FAN_HAZE] * len(hazed), hazed), float,
                                       len(hazed))
    return (grey * fade / np.sqrt(lines) + 0.5).astype(np.int64)


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
    """The horizon, the camera and the fan's dots for a window size: own scale,
    own centre, cast once.
    """
    v0, v1, v2, v3, v4, v5, v6, v7, v8 = view
    south, far = -5.0, 30.0

    def ray(wx, wy, wz=GROUND):
        tx = v0 * wx + v1 * wy + v2 * wz
        ty = v3 * wx + v4 * wy + v5 * wz
        tz = v6 * wx + v7 * wy + v8 * wz
        w = 1.0 / (distance - tz)
        return w * tx, w * ty, w

    hrow = max(1.0, height * 0.15)
    _x, top, _w = ray(0.0, far)
    _x, toe, _w = ray(0.0, 0.0)
    scale = (height * 1.08 - hrow) / (0.5 * (top - toe))
    cy = hrow + scale * 0.5 * top
    cx = width / 2.0

    def cast(wx, wy, wz=GROUND):
        # ray() already carries the perspective weight in its x and y.
        sx, sy, w = ray(wx, wy, wz)
        return cx + scale * sx, cy - scale * 0.5 * sy, w

    # The backdrop is BRAILLE, like the outline: a 2x4 dot matrix per cell.
    horizon = []
    _x, _y, w_far = ray(0.0, far)
    far_depth = 1.0 / w_far
    bow = BOW * height

    def sky(fx):
        """The horizon's row at column `fx`: hrow at the centre, `bow` lower at the edges."""
        return hrow + bow * ((fx - cx) / cx) ** 2

    for col in range(width + 1):
        horizon.append((float(col), sky(col), far_depth))
    def along(wx, depth):
        """How far out a fan dot is, world y, from its depth on the line at `wx`."""
        return (distance - depth - v6 * wx - v8 * GROUND) / v7

    static = {'horizon': horizon, 'cast': cast, 'along': along, 'hrow': hrow, 'sky': sky,
              'ground': GROUND, 'scale': scale, 'rows': 0.5 * scale,
              'south': south, 'far': far, 'far_depth': far_depth}
    static['fan'], static['near_depth'] = _fan(static, width, height)
    return static


def _fan(static, width, height):
    """The fan's lines as polylines, [(line, [(fx, fy, depth, world y)])], and the nearest
    depth in the frame. Wide enough that the outer lines meet the horizon past the frame's
    edges - the fan covers the WHOLE line, not a band in the middle."""
    cast, sky = static['cast'], static['sky']
    south, far = static['south'], static['far']
    # A world line is a straight screen line: the samples only carry depth for the haze
    # and the dashes their world y.
    samples = height * 2
    near_depth = static['far_depth']
    lines = []
    for k in range(-FAN_HALF, FAN_HALF + 1):
        fixed = k * RUNG_SPACING
        run = []
        for i in range(samples + 1):
            wy = south + (far - south) * (i / samples) ** 2
            fx, fy, w = cast(fixed, wy)
            if w <= 0.0 or w > 2.0 or fy < sky(fx):
                if len(run) > 1:
                    lines.append((k, run))
                run = []
                continue
            depth = 1.0 / w
            if run and fy < height:
                near_depth = min(near_depth, depth)
            run.append((fx, fy, depth, wy))
        if len(run) > 1:
            lines.append((k, run))
    return lines, near_depth


def _fan_cells(static, width, height, phase):
    """The horizon and the fan's dashes at `phase`, in _ground_dot's shape."""
    step = int(phase * RUNG_STEPS + 0.5) % RUNG_STEPS
    return _lay([r for r in _runs(static, width, height, step)
                 if not (isinstance(r[0], tuple) and r[0][0] == 'rung')], width, height)


def _ground(codes, inks, buf, distance, width, height, colour, view, scroll=None,
            flown=None):
    """The landscape behind the board at `scroll` seconds (None: at rest) into the frame's
    codepoints and packed tones, where the board's depth buffer `buf` (an array) is empty: the
    cached backdrop - rungs and dashes GROUND_SPEED spacings a second toward the camera -, or
    with `flown` (approach.flight) the floor's own dots rolled by the bank, the stars and the
    corridor's gates over them."""
    from coaxial.model.blocks import numpy as np
    phase = (scroll * GROUND_SPEED) % 1.0 if scroll is not None else 0.0
    if flown is None:
        masks = _backdrop(width, height, distance, view, phase)
        parts = [masks]
    else:
        static = _static(width, height, distance, view)
        roll = approach.roller(flown, width, height, static)
        step = int(phase * RUNG_STEPS + 0.5) % RUNG_STEPS
        laid = _laid(_runs(static, width, height, step), width, height, roll)
        if laid is not None:
            codes[laid[0]] = BRAILLE + laid[1]
            inks[laid[0]] = _shaded(laid, static) * 0x010101
        parts = [approach.stars(static, width, height, scroll or 0.0, roll),
                 approach.corridor(static, width, height,
                                   (scroll or 0.0) * GROUND_SPEED * RUNG_SPACING,
                                   flown['curve'], roll, _segment,
                                   beacons=int((scroll or 0.0) * 2.0) % 2 == 0,
                                   search=flown['search'])]
    for got in parts:
        if got:
            at = np.fromiter(got, np.int64, len(got))
            codes[at] = BRAILLE + np.fromiter((v[0] for v in got.values()), np.int64, len(got))
            inks[at] = np.fromiter(((g[0] << 16) | (g[1] << 8) | g[2]
                                    for _m, g in got.values()), np.int64, len(got))
    board = buf != 0.0
    codes[board] = 0x20
    inks[board | (not colour)] = -1


def _segment(dot, prev, fx, fy, depth, k):
    """The ground line from `prev` to here: EVERY dot it passes through, its
    depth interpolated for the buffer.
    """
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
