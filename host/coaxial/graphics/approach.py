"""The approach from the pilot's seat: gates down to a pad, the scene banking, a HUD - eye candy.

The flight is a function of the view's clock (`flight`): the corridor's curvature swings
left and right, the craft banks into the bend and turns with it, the nose bobs. The
ground (`coaxial.graphics.ground`) is cast flat and cached; each frame the corridor is
laid on it and the whole is rolled by the bank and lifted by the bob (`scene`). The HUD
is conformal: its ladder is the horizon itself at the camera's own degrees a row, its arc
the bank, its tape the heading the bend turns (2026-09-24, "Elite, Blade Runner,
Nostromo: an orbital approach, from the pilot's eyes"). The board's attitude is the
panel's beside it, never these.
"""
import math

from coaxial.graphics.raster import BRAILLE_BITS

AMBER = (255, 176, 0)
RED = (255, 72, 48)
DIM = (150, 104, 0)

#: A cell's height in widths: rolling the scene is a rotation of the screen as seen.
ASPECT = 2.0

#: The flight: the corridor's curvature swings +-CURVE (1/model unit) over CURVE_S
#: seconds; the craft banks BANK degrees at full curvature, into the bend, and turns TURN
#: degrees a second there; the nose bobs BOB degrees over BOB_S.
CURVE = 0.045
CURVE_S = 22.0
BANK = 14.0
TURN = 6.0
BOB = 1.2
BOB_S = 7.0
HEADING = 274.0


def flight(t):
    """The craft at `t` seconds: curve (1/unit, + to the right), bank (deg, + right wing
    down), bob (deg, + nose up), heading (deg), all smooth in `t`."""
    w = 2.0 * math.pi / CURVE_S
    swing = math.sin(w * t)
    return {'curve': CURVE * swing, 'bank': BANK * swing,
            'bob': BOB * math.sin(2.0 * math.pi * t / BOB_S),
            'heading': (HEADING + TURN / w * (1.0 - math.cos(w * t))) % 360.0}


#: The corridor, in the pilot's eye: square gates each GATE_EVERY deep from GATE_FIRST to
#: GATE_LAST ahead, sized and placed by 1/depth - the nearest frames the board, NEAR of
#: the frame wide; the farthest a few columns. The path falls away from the line of
#: sight as depth squared, reaching DIVE of the frame's height below its centre at
#: GATE_LAST - a steep dive to a point on the surface, short of the horizon, the gates
#: narrowing to it as they near it (x (1 - depth / GATE_LAST)) - and bends as depth
#: to the 1.5 with the flight's curvature, SWING of the frame's width at full bend. They
#: close on the craft with the floor's speed. GATE_HAZE of the amber is left at the last.
NEAR = 0.8
GATE_EVERY = 3.0
GATE_FIRST = 1.2
GATE_LAST = 26.0
DIVE = 0.36
SWING = 0.32
GATE_HAZE = 0.35
#: A gate within GREEN of the craft is cleared: green, just before it is passed.
GREEN = 0.9
CLEARED = (90, 255, 150)

#: The landing pad where the corridor closes: PAD of the frame wide, seen from high above
#: (FLAT of a square's height), a cross on it, its corner beacons blinking red.
PAD = 0.035
FLAT = 0.4


def corridor(static, width, height, travel, curve=0.0, roll=None, segment=None,
             beacons=False):
    """{cell: (braille mask, (r, g, b))}: the gates `travel` model units closer, each a
    square at its depth on the path, hazed by it, its corners moved by `roll` and its
    edges drawn dot by dot by `segment` (ground._segment)."""
    cx, cy = width / 2.0, height / 2.0
    size = NEAR * width * GATE_FIRST                     # half-width x depth, columns
    fall = DIVE * height / GATE_LAST ** 2                # rows at depth z: fall * z^2
    bend = SWING * width / GATE_LAST ** 1.5 * (curve / CURVE if CURVE else 0.0)
    shown = []
    z = GATE_FIRST + (GATE_EVERY - travel % GATE_EVERY) % GATE_EVERY
    while z <= GATE_LAST:
        half = 0.5 * size / z * (1.0 - z / GATE_LAST)
        shown.append((z, cx + bend * z ** 1.5, cy + fall * z * z, half))
        z += GATE_EVERY
    if not shown:
        return {}
    out = {}
    land = (cx + bend * GATE_LAST ** 1.5, cy + fall * GATE_LAST ** 2)

    def put(x, y, rgb):
        if 0.0 <= x < width and 0.0 <= y < height:
            px, py = int(x), int(y)
            at = py * width + px
            bit = BRAILLE_BITS[1 if x - px >= 0.5 else 0][min(3, int((y - py) * 4.0))]
            out[at] = (out[at][0] | bit if at in out else bit, rgb)

    # The flight path through the gates' centres: faint, dotted, the curve they lie on.
    path = [(x, y) for _z, x, y, _h in shown] + [land]
    if roll is not None:
        path = [roll(x, y) for x, y in path]
    # The pad the corridor closes to: an outline, a cross, beacons at its corners.
    px, py = land
    half = PAD * width
    tall = half / ASPECT * FLAT
    pad = [(px - half, py - tall), (px + half, py - tall), (px + half, py + tall),
           (px - half, py + tall)]
    cross = [((px - 0.5 * half, py), (px + 0.5 * half, py)),
             ((px, py - 0.5 * tall), (px, py + 0.5 * tall))]
    if roll is not None:
        pad = [roll(x, y) for x, y in pad]
        cross = [(roll(*p), roll(*q)) for p, q in cross]
    for (x0, y0), (x1, y1) in list(zip(pad, pad[1:] + pad[:1])) + cross:
        for i in range(25):
            put(x0 + (x1 - x0) * i / 24.0, y0 + (y1 - y0) * i / 24.0, AMBER)
    if beacons:
        for x, y in pad:
            put(x, y, RED)
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        for i in range(0, 12, 3):
            put(x0 + (x1 - x0) * i / 12.0, y0 + (y1 - y0) * i / 12.0, DIM)
    for z, x, y, half in reversed(shown):               # the nearest drawn last, on top
        fade = GATE_HAZE ** ((z - GATE_FIRST) / (GATE_LAST - GATE_FIRST))
        rgb = tuple(int(c * fade + 0.5) for c in (CLEARED if z < GATE_FIRST + GREEN
                                                   else AMBER))
        tall = half / ASPECT
        corners = [(x - half, y - tall), (x + half, y - tall), (x + half, y + tall),
                   (x - half, y + tall)]
        if roll is not None:
            corners = [roll(a, b) for a, b in corners]
        for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1]):
            if segment is not None:
                segment(lambda a, b, _d, _k, rgb=rgb: put(a, b, rgb), (x0, y0, z), x1, y1,
                        z, 0)
            else:
                for t in range(17):
                    put(x0 + (x1 - x0) * t / 16.0, y0 + (y1 - y0) * t / 16.0, rgb)
    return out


#: Stars over the horizon: STARS a 150-column frame, each on a fixed spot of the sky and a
#: brightness of its own, SCINTILLATING - the air's refraction and damp: a fast shimmer
#: of SHIMMER of its light at two rates of its own, its hue drifting toward WARM and back.
STARS = 22
SHIMMER = 0.35
STAR = (170, 182, 214)
WARM = (214, 176, 160)

#: Decoration, meaning nothing: a kana line under the tag (2026-09-24, "Ghost in the
#: Shell, Nostromo").
DATA = (60, 190, 140)
TAG = '>> ﾁｬｸﾘｸﾁ ﾀﾝｻｸﾁｭｳ'
SUBTAG = 'ﾁﾊﾞ ｽﾌﾟﾗｳﾙ ﾅﾋﾞ 7G'

#: A craft far below, rarely: each SLOT seconds holds a pass with PASS_CHANCE, at a moment,
#: from a side and at a speed of its own (CROSS seconds, least and most); the first at
#: FIRST. It comes in from its side's edge and arcs down through the bottom corner, diving
#: out under the frame. The arc keeps CLEAR columns, the craft's REACH and CORNER off the
#: board's BOUND - the circle the board stays inside at every attitude (`bound`) - so it is
#: planned once and never meets the board; no pass where the corner has under LEAST lengths.
#: CRAFT columns nose to tail at a 150-column frame: a spinner seen from above in steel
#: alone - a wedge lit along its spine, swept fins lit by its bank, a glinting canopy - its
#: trail fading along its arc for TRAIL lengths. It rolls up to ROLL degrees into its turn
#: and yaws YAW each way through it. One colour a cell: lights of other colours would flip
#: whole cells as it moves.
#: A marker glides in from outside, away from the board, over GRIP of the pass and closes
#: on the craft, AMBER; gripped, it squeezes once and holds it, DATA, labelled MARKED.
SLOT = 45.0
FIRST = 3.0
PASS_CHANCE = 0.55
CROSS = (2.4, 4.2)
CLEAR = 2.0
CORNER = 1.2
LEAST = 1.5
CRAFT = 9.5
ROLL = 35.0
YAW = 22.0
TRAIL = 1.4
HULL = (175, 196, 214)
GLASS = (235, 245, 255)
GRIP = (0.08, 0.38)
MARKED = 'ｼｷﾍﾞﾂ'

#: The craft from above, nose along +u, in its lengths (v across): the hull a wedge, the fins
#: swept back from it, the canopy on the nose.
HULL_SHAPE = ((0.62, 0.0), (0.46, 0.09), (0.16, 0.16), (-0.26, 0.19), (-0.52, 0.15),
              (-0.58, 0.07), (-0.58, -0.07), (-0.52, -0.15), (-0.26, -0.19), (0.16, -0.16),
              (0.46, -0.09))
FINS = (((0.02, 0.16), (-0.38, 0.44), (-0.52, 0.44), (-0.36, 0.17)),
        ((0.02, -0.16), (-0.38, -0.44), (-0.52, -0.44), (-0.36, -0.17)))
CANOPY_AT, CANOPY_R = (0.24, 0.0), (0.15, 0.075)
TAIL = -0.6
#: The craft's farthest dot from its centre, in lengths.
REACH = max(math.hypot(u, v) for u, v in HULL_SHAPE + FINS[0] + FINS[1])


def _inside(u, v, polygon):
    """Whether (u, v) is inside `polygon`: a ray to +u crosses its edges an odd number."""
    inside = False
    for (u0, v0), (u1, v1) in zip(polygon, polygon[1:] + polygon[:1]):
        if (v0 > v) != (v1 > v) and u < u0 + (u1 - u0) * (v - v0) / (v1 - v0):
            inside = not inside
    return inside


def _hashed(*n):
    """A number from integers, the same every run: a star's place, a glyph's pick."""
    h = 2166136261
    for v in n:
        h = ((h ^ (v & 0xFFFFFFFF)) * 16777619) & 0xFFFFFFFF
    return h


def stars(static, width, height, t, roll=None):
    """{cell: (braille mask, (r, g, b))}: the stars at `t`, each a dot over the horizon."""
    sky = static['sky']
    out = {}
    for i in range(max(6, STARS * width // 150)):
        x = _hashed(i, 1) % (2 * width) / 2.0 + 0.25
        top = sky(x) - 1.0
        if top <= 0.5:
            continue
        y = _hashed(i, 2) % 1000 / 1000.0 * top
        base = 0.35 + 0.65 * (_hashed(i, 3) % 100) / 100.0
        fast, faster = 2.3 + (_hashed(i, 4) % 9) * 0.37, 5.1 + (_hashed(i, 5) % 11) * 0.41
        shimmer = 0.5 * (math.sin(t * fast + i) + math.sin(t * faster + 2.0 * i))
        light = base * (1.0 - SHIMMER * (0.5 + 0.5 * shimmer))
        warm = 0.5 + 0.5 * math.sin(t * 0.7 + 1.7 * i)
        rgb = tuple(int(light * (c + (w - c) * warm * 0.6)) for c, w in zip(STAR, WARM))
        if roll is not None:
            x, y = roll(x, y)
        if 0.0 <= x < width and 0.0 <= y < height:
            px, py = int(x), int(y)
            bit = BRAILLE_BITS[1 if x - px >= 0.5 else 0][min(3, int((y - py) * 4.0))]
            out[py * width + px] = (bit, rgb)
    return out


def _pass(t):
    """The pass under way at `t`: (its number, k along it, its side +-1), or None."""
    n = int(t // SLOT)
    if n and _hashed(n, 10) % 100 >= PASS_CHANCE * 100:
        return None
    length = CROSS[0] + (CROSS[1] - CROSS[0]) * (_hashed(n, 11) % 100) / 100.0
    start = (n * SLOT + (_hashed(n, 12) % 1000) / 1000.0 * (SLOT - length) if n
             else FIRST)
    k = (t - start) / length
    if not 0.0 <= k <= 1.0:
        return None
    return n, k, 1.0 if _hashed(n, 13) % 2 else -1.0


def bound(cam, reach):
    """(cx, cy, radius in columns): the circle the board, of bounding radius `reach`, stays
    inside at every attitude - the camera's own fit, rows at ASPECT to a column."""
    d = cam['distance']
    return cam['cx'], cam['cy'], cam['scale'] * reach / math.sqrt(d * d - reach * reach)


def _arc(radius, side, width, height, phi):
    """A point of the corner arc: `phi` 0 at the side's edge, pi/2 at the bottom edge."""
    corner = 0.0 if side > 0 else float(width)
    return corner + side * radius * math.sin(phi), height - radius * math.cos(phi) / ASPECT


def arc_radius(width, height, side, board=None):
    """The corner arc's radius: its nearest point to the board's bound `board` (cx, cy,
    radius) CLEAR columns, the craft's REACH and CORNER - its marker's corners, rounded to
    their cells - off it; 0 where under LEAST lengths."""
    size = CRAFT * width / 150.0
    most = 0.55 * min(width, height * ASPECT)
    if board is None:
        return most
    cx, cy, reach = board
    corner = 0.0 if side > 0 else float(width)
    room = math.hypot(corner - cx, (height - cy) * ASPECT) - reach - CLEAR - REACH * size - CORNER
    radius = min(most, room)
    return radius if radius >= LEAST * size else 0.0


def _pose(width, height, t, board):
    """The pass on at `t`: {k, side, x, y, heading, bank, roll, size, track}, or None."""
    now = _pass(t)
    if now is None:
        return None
    _n, k, side = now
    radius = arc_radius(width, height, side, board)
    if not radius:
        return None

    def track(k):
        """In from beyond its edge, round the corner, out below the frame."""
        return _arc(radius, side, width, height, -0.35 + (math.pi / 2.0 + 0.5) * k)

    x, y = track(k)
    x1, y1 = track(k + 0.01)
    # The manoeuvre: the arc's own 90 degrees, a yaw S through it, a roll into the turn.
    heading = math.atan2((y1 - y) * ASPECT, x1 - x) + math.radians(
        YAW * math.sin(2.0 * math.pi * k)) * side
    roll = math.radians(ROLL * math.sin(math.pi * k))
    return {'k': k, 'side': side, 'x': x, 'y': y, 'heading': heading, 'bank': math.cos(roll),
            'roll': roll, 'size': CRAFT * width / 150.0, 'length': radius * (math.pi / 2.0 + 0.5),
            'track': track}


def craft(width, height, t, board=None):
    """{cell: (braille mask, (r, g, b))}: the craft at `t` when a pass is on and its corner
    has room, else {}; `board` (cx, cy, radius) its bound, kept clear at every attitude."""
    pose = _pose(width, height, t, board)
    if pose is None:
        return {}
    x0, y0, size, bank = pose['x'], pose['y'], pose['size'], pose['bank']
    c, s = math.cos(pose['heading']), math.sin(pose['heading'])
    lean = math.sin(pose['roll']) * pose['side']
    reach = REACH * size
    cells, inks = {}, {}

    def mark(x, y, rgb, rank):
        """A dot at the screen point (x, y); a cell's ink the highest rank's."""
        if 0.0 <= x < width and 0.0 <= y < height:
            at = int(y) * width + int(x)
            cells[at] = cells.get(at, 0) | BRAILLE_BITS[1 if x - int(x) >= 0.5 else 0][
                min(3, int((y - int(y)) * 4.0))]
            if rank >= inks.get(at, (-9, None))[0]:
                inks[at] = (rank, rgb)

    def steel(f):
        return tuple(int(ch * f) for ch in HULL)

    # The trail: from the tail onto the arc behind, fading.
    tx = x0 + size * TAIL * c
    ty = y0 + size * TAIL * s / ASPECT
    ax, ay = pose['track'](pose['k'] + TAIL * size / pose['length'])
    for i in range(int(2.0 * (TRAIL - abs(TAIL)) * size) + 1):
        f = i / max(1.0, 2.0 * (TRAIL - abs(TAIL)) * size)
        bx, by = pose['track'](pose['k'] + (TAIL - f * (TRAIL - abs(TAIL))) * size
                               / pose['length'])
        mark(bx + (1.0 - f) * (tx - ax), by + (1.0 - f) * (ty - ay), steel(0.8 - 0.6 * f), -1)

    for py in range(int(y0 - reach / ASPECT) - 1, int(y0 + reach / ASPECT) + 2):
        for half in range(2 * int(x0 - reach) - 2, 2 * int(x0 + reach) + 4):
            for row in range(4):
                x, y = half / 2.0 + 0.25, py + 0.125 + 0.25 * row
                dx, dy = (x - x0) / size, (y - y0) * ASPECT / size
                u, v = dx * c + dy * s, (-dx * s + dy * c) / bank
                if math.hypot(u, v * bank) > REACH:
                    continue
                if ((u - CANOPY_AT[0]) / CANOPY_R[0]) ** 2 + (v / CANOPY_R[1]) ** 2 <= 1.0:
                    mark(x, y, GLASS, 2)
                elif _inside(u, v, HULL_SHAPE):
                    mark(x, y, steel(1.0 - 0.35 * min(1.0, abs(v) / 0.19)), 1)   # its spine lit
                elif any(_inside(u, v, fin) for fin in FINS):
                    mark(x, y, steel(0.7 + 0.25 * lean * (1.0 if v > 0.0 else -1.0)), 0)
    return {at: (mask, inks[at][1]) for at, mask in cells.items()}


def marker(width, height, t, board=None):
    """[(row, col, text)], gripped: the marker's corners round the craft at `t` - closing
    in from outside, away from the board, over GRIP; gripped, squeezing once and holding,
    labelled MARKED. Never nearer the board than its gripped self."""
    pose = _pose(width, height, t, board)
    if pose is None or pose['k'] < GRIP[0]:
        return [], False
    k, x, y = pose['k'], pose['x'], pose['y']
    g = min(1.0, (k - GRIP[0]) / (GRIP[1] - GRIP[0]))
    g = g * g * (3.0 - 2.0 * g)
    cx, cy = (board[0], board[1]) if board else (width / 2.0, height / 2.0)
    ox, oy = x - cx, (y - cy) * ASPECT
    norm = math.hypot(ox, oy) or 1.0
    grip = REACH * pose['size'] / math.sqrt(2.0)          # its corners on the craft's reach
    # Its offset outward outruns its corners' growth: no nearer the board than gripped.
    away = 2.5 * grip * (1.0 - g)
    half = grip * (1.0 + 1.2 * (1.0 - g)
                   - 0.25 * math.sin(math.pi * min(1.0, max(0.0, (k - GRIP[1]) / 0.06))))
    mx, my = x + away * ox / norm, y + away * oy / norm / ASPECT
    top, bottom = int(my - half / ASPECT), int(my + half / ASPECT)
    left, right = int(mx - half), int(mx + half)
    out = [(top, left, '┌'), (top, right, '┐'), (bottom, left, '└'), (bottom, right, '┘')]
    if g >= 1.0:
        out.append((bottom + 1, right + 1 - len(MARKED) if pose['side'] > 0 else left, MARKED))
    return out, g >= 1.0


def roller(fl, width, height, static):
    """(fx, fy) -> (fx, fy): the screen as the craft sees it - rolled against the bank
    about the view's centre, lifted by the bob at the camera's own rows a degree."""
    angle = math.radians(fl['bank'])
    c, s = math.cos(angle), math.sin(angle)
    cx, cy = width / 2.0, height / 2.0
    lift = static['rows'] * math.tan(math.radians(fl['bob']))

    def roll(fx, fy):
        x, y = fx - cx, (fy - cy) * ASPECT
        return cx + x * c + y * s, cy + (-x * s + y * c) / ASPECT + lift
    return roll


def _put(grid, tone, row, col, text, ink, clear=False, buf=None):
    """`text` into the grid at (row, col), clipped to it and, given `buf`, kept off the
    board's cells; `clear`: its blanks too."""
    if not 0 <= row < len(grid):
        return
    width = len(grid[row])
    for i, ch in enumerate(text):
        c = col + i
        if 0 <= c < width and (ch != ' ' or clear) and not (buf and buf[row * width + c]):
            grid[row][c], tone[row][c] = ch, ink


def _free(grid, buf, row, col, n):
    """Whether the `n` cells from (row, col) are in the grid, off the board and hold no
    text - blank or a braille dot."""
    if not 0 <= row < len(grid) or col < 0 or col + n > len(grid[row]):
        return False
    width = len(grid[row])
    return all(not buf[row * width + c] and (grid[row][c] == ' '
                                             or 0x2800 <= ord(grid[row][c]) <= 0x28FF)
               for c in range(col, col + n))


#: The ladder's band: under the tape and the roll arc, over the bottom lines.
LADDER_TOP, LADDER_FOOT = 5, 2


def _ladder(grid, tone, buf, width, height, static, roll, ink):
    """The pitch ladder, conformal: a rung each 10 degrees from the horizon, at the rows
    the camera puts that angle, rolled with the scene; drawn where the board is not."""
    rows, hrow, cx = static['rows'], static['hrow'], width / 2.0
    gap, reach = 0.20 * width, 0.27 * width
    top, foot = LADDER_TOP, height - LADDER_FOOT
    marks = {}
    for deg in range(-60, 31, 10):
        y = hrow - rows * math.tan(math.radians(deg))
        if not -2.0 <= y <= height + 2.0:
            continue
        for side in (-1.0, 1.0):
            for k in range(int(2 * (reach - gap)) + 1):
                if deg < 0 and k % 3 == 2:
                    continue                      # below the horizon, dashed
                fx, fy = roll(cx + side * (gap + 0.5 * k), y)
                px, py = int(fx), int(fy)
                if 0 <= px < width and top <= py < foot and buf[py * width + px] == 0.0:
                    bit = BRAILLE_BITS[1 if fx - px >= 0.5 else 0][min(3, int((fy - py) * 4.0))]
                    marks[(py, px)] = marks.get((py, px), 0) | bit
            fx, fy = roll(cx + side * (reach + 2.0), y)
            if deg and top <= int(fy) < foot:
                label = '%+d' % deg
                _put(grid, tone, int(fy), int(fx) - (len(label) if side < 0 else 0), label, ink,
                     buf=buf)
    for (py, px), mask in marks.items():
        if grid[py][px] == ' ' or 0x2800 <= ord(grid[py][px]) <= 0x28FF:
            grid[py][px], tone[py][px] = chr(0x2800 + mask), ink


def hud(grid, tone, buf, width, height, fl, static, scroll, gates, box, colour, board=None):
    """The overlay, the flight's: the heading tape, the roll arc, the conformal ladder,
    lock brackets round the board (`box`: first and last row and column, or None), the
    pitch and bank, the clock, the gates flown, a tag; the craft over all, its marker
    beside the text, both kept off `board`, the board's bound at every attitude (`bound`)."""
    if width < 40 or height < 14:
        return
    amber, dim, red, data = (AMBER, DIM, RED, DATA) if colour else (None,) * 4
    t = scroll or 0.0
    roll = roller(fl, width, height, static)

    # The ladder first: text over it.
    _ladder(grid, tone, buf, width, height, static, roll, dim)

    # The heading tape: a degree a column, a tick each 5, each 10 labelled.
    span = min(61, width - 24) | 1
    left = (width - span) // 2
    ticks, labels = [' '] * span, [' '] * span
    for c in range(span):
        deg = fl['heading'] + (c - span // 2)
        near5 = round(deg / 5.0) * 5.0
        if abs(deg - near5) < 0.5:
            whole = int(near5) % 360
            ticks[c] = '|' if whole % 10 == 0 else "'"
            if whole % 10 == 0 and 1 <= c < span - 2:
                labels[c - 1:c + 2] = list('%03d' % whole)
    _put(grid, tone, 0, left, ''.join(labels), amber, buf=buf)
    _put(grid, tone, 1, left, ''.join(ticks), dim, buf=buf)
    _put(grid, tone, 2, width // 2 - 3, '<%03d>' % (int(round(fl['heading'])) % 360), amber,
         buf=buf)

    # The roll arc: -30..30, a column each 2 degrees, the pointer on the bank.
    arc = [' '] * 31
    for deg in range(-30, 31, 10):
        arc[15 + deg // 2] = '|' if deg == 0 else "'"
    _put(grid, tone, 3, width // 2 - 15, ''.join(arc), dim, buf=buf)
    pointer = 15 + int(round(max(-30.0, min(30.0, fl['bank'])) / 2.0))
    _put(grid, tone, 4, width // 2 - 15 + pointer, '^', amber, buf=buf)

    # Lock brackets round the board, when it is in the frame.
    if box is not None:
        top, bottom, first, last = box
        top, bottom = max(5, top - 1), min(height - 3, bottom + 1)
        first, last = max(0, first - 2), min(width - 1, last + 2)
        for row, col, text in ((top, first, '┌─'), (top, last - 1, '─┐'),
                               (bottom, first, '└─'), (bottom, last - 1, '─┘')):
            _put(grid, tone, row, col, text, amber, buf=buf)
        if int(t * 2.0) % 2 == 0:
            _put(grid, tone, top, first + 3, 'ﾛｯｸ ｵﾝ  ', amber, clear=True, buf=buf)

    # The view's pitch (its centre against the horizon) and bank; the clock, the gates.
    pitch = -math.degrees(math.atan((height / 2.0 - static['hrow']) / static['rows'])) \
        + fl['bob']
    _put(grid, tone, height - 2, 1, 'ﾋﾟｯﾁ %+05.1f  ﾊﾞﾝｸ %s%04.1f' % (
        pitch, 'R' if fl['bank'] >= 0 else 'L', abs(fl['bank'])), dim, buf=buf)
    minutes, seconds = divmod(t, 60.0)
    _put(grid, tone, height - 1, 1, 'T+%02d:%04.1f  ｹﾞｰﾄ %02d' % (minutes, seconds,
                                                                gates % 100), amber,
         buf=buf)
    if int(t * 1.5) % 2 == 0:
        _put(grid, tone, height - 1, width - len(TAG) - 1, TAG, red, buf=buf)
    _put(grid, tone, height - 2, width - len(SUBTAG) - 1, SUBTAG, data, buf=buf)

    # The craft's marker where no text is, whole or not at all; the craft over everything.
    corners, gripped = marker(width, height, t, board)
    for row, col, text in corners:
        if _free(grid, buf, row, col, len(text)):
            _put(grid, tone, row, col, text, data if gripped else amber)
    for at, (mask, rgb) in craft(width, height, t, board).items():
        r, c = divmod(at, width)
        grid[r][c], tone[r][c] = chr(0x2800 + mask), rgb if colour else None
