"""The craft that passes now and then, over the approach.

A craft far below, rarely: each SLOT seconds holds a pass with PASS_CHANCE, at a moment,
of a kind, from a side and at a speed of its own (CROSS or BY seconds, least and most);
slot 0 opens with a corner pass at FIRST and a fly-by at FIRST_BY. Each is planned once
against the board's BOUND - the circle the board stays inside at every attitude
(`bound`) - CLEAR columns and its own reach off it, so neither ever meets the board.
A corner pass comes in from its side's edge and arcs down through the bottom corner,
diving out under the frame, rolling to ROLL into its turn and yawing YAW each way
through it; none where the corner has under LEAST lengths.
A fly-by climbs toward space ahead of us, vertical, on one column: BY_LANE in from its
side's edge, or as far out as the bound asks anywhere on the way - from BY_START of the
frame down. We close on it: it grows from BY_FAR of BY_SIZE columns to the whole as an
approach magnifies (1 / (1 - (1 - BY_FAR) k)), its haze lifting from BY_HAZE, and moves
with its growth - slow far off, a swish out over the top edge near. A roll over BY_ROLL
of the way. Its jet runs dead straight back along its axis, a ramjet burning a metal:
JET lengths at most, ending CLEAR off the bound, white-hot at the nozzle to violet at the
end (JET_INK), SPARKS points of it flaring along the line each frame.
CRAFT columns nose to tail at a 150-column frame: a spinner seen from above in steel
alone - a wedge lit along its spine, swept fins lit by its bank, a glinting canopy - its
trail fading along its path for TRAIL lengths. One colour a cell: lights of other
colours would flip whole cells as it moves.
On a corner pass a marker glides in from outside, away from the board, over GRIP and
closes on the craft, AMBER: a frame FRAME lengths each side of it, square on screen, its
corners in the arc's plan with CORNER for their cells. Gripped, it squeezes once and
holds, DATA, MARKED in its top edge.
"""
import math

from coaxial.graphics.approach import AMBER, ASPECT, DATA, ease, free, hashed, put
from coaxial.graphics.raster import BRAILLE_BITS

SLOT = 45.0
FIRST = 3.0
FIRST_BY = 14.0
PASS_CHANCE = 0.55
CROSS = (2.4, 4.2)
BY = (2.6, 3.6)
CLEAR = 2.0
LEAST = 1.2
CRAFT = 9.5
ROLL = 35.0
YAW = 22.0
BY_SIZE = 16.0
BY_FAR = 0.12
BY_HAZE = 0.6
BY_START = 0.3
BY_ROLL = (0.15, 0.55)
BY_LANE = 9.0
JET = 3.0
JET_INK = ((255, 252, 240), (190, 222, 255), (90, 110, 220))
SPARKS = 4
TRAIL = 1.4
HULL = (175, 196, 214)
GLASS = (235, 245, 255)
GRIP = (0.08, 0.38)
FRAME = 0.75
CORNER = 1.2
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


def under_way(t):
    """The pass under way at `t`: (its number, k along it, its side +-1, 'corner' or 'by'),
    or None."""
    n = int(t // SLOT)
    if n:
        if hashed(n, 10) % 100 >= PASS_CHANCE * 100:
            return None
        kinds = (('by' if hashed(n, 14) % 2 else 'corner', None),)
    else:
        kinds = (('corner', FIRST), ('by', FIRST_BY))
    for i, (kind, start) in enumerate(kinds):
        least, most = CROSS if kind == 'corner' else BY
        length = least + (most - least) * (hashed(n, 11, i) % 100) / 100.0
        if start is None:
            start = n * SLOT + (hashed(n, 12) % 1000) / 1000.0 * (SLOT - length)
        k = (t - start) / length
        if 0.0 <= k <= 1.0:
            return n, k, 1.0 if hashed(n, 13, i) % 2 else -1.0, kind
    return None


def bound(cam, reach):
    """(cx, cy, radius in columns): the circle the board, of bounding radius `reach`, stays
    inside at every attitude - the camera's own fit, rows at ASPECT to a column."""
    d = cam['distance']
    return cam['cx'], cam['cy'], cam['scale'] * reach / math.sqrt(d * d - reach * reach)


def _arc(radius, side, width, height, phi):
    """A point of the corner arc: `phi` 0 at the side's edge, pi/2 at the bottom edge."""
    corner = 0.0 if side > 0 else float(width)
    return corner + side * radius * math.sin(phi), height - radius * math.cos(phi) / ASPECT


def arc_radius(width, height, side, board):
    """The corner arc's radius: its nearest point to the board's bound `board` (cx, cy,
    radius) CLEAR columns, the reach of the craft or of its marker's frame and CORNER off
    it; 0 where under LEAST lengths."""
    size = CRAFT * width / 150.0
    cx, cy, reach = board
    corner = 0.0 if side > 0 else float(width)
    room = (math.hypot(corner - cx, (height - cy) * ASPECT) - reach - CLEAR
            - max(REACH, FRAME * math.sqrt(2.0)) * size - CORNER)
    radius = min(0.55 * min(width, height * ASPECT), room)
    return radius if radius >= LEAST * size else 0.0


def _corner(width, height, side, board):
    """(track, size) of a corner pass - k to the screen point, k to columns - or None."""
    radius = arc_radius(width, height, side, board)
    if not radius:
        return None
    size = CRAFT * width / 150.0
    return (lambda k: _arc(radius, side, width, height, -0.35 + (math.pi / 2.0 + 0.5) * k),
            lambda k: size)


def _by(width, height, side, board):
    """(track, size) of a fly-by: from BY_START of the frame down, straight up and out over
    the top edge as it grows, on the one column CLEAR and its reach off the bound all the
    way."""
    cx, cy, reach = board
    near = BY_SIZE * width / 150.0
    edge = 0.0 if side > 0 else float(width)
    low, high = BY_START * height, -1.0 - REACH * near / ASPECT

    def size(k):
        return near * BY_FAR / (1.0 - (1.0 - BY_FAR) * k)

    def y(k):
        return low + (high - low) * (size(k) - near * BY_FAR) / (near - near * BY_FAR)

    def room(k):
        """Columns in from the edge to the bound, CLEAR and the reach off it, at k."""
        rim = reach + CLEAR + REACH * size(k)
        dy = (y(k) - cy) * ASPECT
        if rim <= abs(dy):
            return float(width)
        return side * (cx - side * math.sqrt(rim * rim - dy * dy) - edge)

    column = edge + side * min([BY_LANE * width / 150.0]
                               + [room(i / 40.0) - 0.2 for i in range(41)])
    return (lambda k: (column, y(k))), size


def pose(width, height, t, board):
    """The pass on at `t`: {k, side, kind, x, y, heading, bank, roll, size, sized, fade,
    speed, track, board}, or None; `board` None a bound of the frame's own."""
    now = under_way(t)
    if now is None:
        return None
    _n, k, side, kind = now
    if board is None:
        board = (width / 2.0, height / 2.0, 0.35 * min(width, height * ASPECT))
    path = (_corner if kind == 'corner' else _by)(width, height, side, board)
    if path is None:
        return None
    track, size = path
    x, y = track(k)
    x1, y1 = track(k + 0.01)
    heading = math.atan2((y1 - y) * ASPECT, x1 - x)
    if kind == 'corner':
        # The arc's own 90 degrees, a yaw S through it, a roll into the turn.
        heading += math.radians(YAW * math.sin(2.0 * math.pi * k)) * side
        roll = math.radians(ROLL * math.sin(math.pi * k))
    else:
        roll = 2.0 * math.pi * ease((k - BY_ROLL[0]) / (BY_ROLL[1] - BY_ROLL[0]))   # the roll program
    bank = math.cos(roll)
    return {'k': k, 'side': side, 'kind': kind, 'x': x, 'y': y, 'heading': heading,
            'bank': math.copysign(max(0.15, abs(bank)), bank), 'roll': roll, 'size': size(k),
            'sized': size,
            'fade': 1.0 if kind == 'corner' else BY_HAZE + (1.0 - BY_HAZE) * k,
            'speed': math.hypot(x1 - x, (y1 - y) * ASPECT) / 0.01, 'track': track,
            'board': board}


def craft(width, height, t, board=None):
    """{cell: (braille mask, (r, g, b), its body's)}: the craft at `t` when a pass is on and
    has room, else {} - a cell of its trail alone not its body's; `board` (cx, cy, radius)
    its bound, kept clear at every attitude."""
    now = pose(width, height, t, board)
    if now is None:
        return {}
    x0, y0, size, bank = now['x'], now['y'], now['size'], now['bank']
    c, s = math.cos(now['heading']), math.sin(now['heading'])
    lean = math.sin(now['roll']) * now['side']
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
        return tuple(int(ch * f * now['fade']) for ch in HULL)

    if now['kind'] == 'corner':
        # The trail: from the tail onto the path behind, fading.
        tx = x0 + size * TAIL * c
        ty = y0 + size * TAIL * s / ASPECT
        ax, ay = now['track'](now['k'] + TAIL * size / now['speed'])
        steps = 2.0 * (TRAIL - abs(TAIL)) * size
        for i in range(int(steps) + 1):
            f = i / max(1.0, steps)
            bx, by = now['track'](now['k'] + (TAIL - f * (TRAIL - abs(TAIL))) * size
                                   / now['speed'])
            mark(bx + (1.0 - f) * (tx - ax), by + (1.0 - f) * (ty - ay), steel(0.8 - 0.6 * f),
                 -1)
    else:
        _jet(now, mark, t)

    for py in range(int(y0 - reach / ASPECT) - 1, int(y0 + reach / ASPECT) + 2):
        for half in range(2 * int(x0 - reach) - 2, 2 * int(x0 + reach) + 4):
            for row in range(4):
                x, y = half / 2.0 + 0.25, py + 0.125 + 0.25 * row
                dx, dy = (x - x0) / size, (y - y0) * ASPECT / size
                u, v = dx * c + dy * s, (-dx * s + dy * c) / bank
                if math.hypot(u, v * bank) > REACH:
                    continue
                if bank > 0.0 and ((u - CANOPY_AT[0]) / CANOPY_R[0]) ** 2 \
                        + (v / CANOPY_R[1]) ** 2 <= 1.0:
                    mark(x, y, steel(GLASS[2] / HULL[2]) if now['fade'] < 1.0 else GLASS, 2)
                elif _inside(u, v, HULL_SHAPE):
                    mark(x, y, steel(1.0 - 0.35 * min(1.0, abs(v) / 0.19)), 1)   # its spine lit
                elif any(_inside(u, v, fin) for fin in FINS):
                    mark(x, y, steel(0.7 + 0.25 * lean * (1.0 if v > 0.0 else -1.0)), 0)
    return {at: (mask, inks[at][1], inks[at][0] >= 0) for at, mask in cells.items()}


def _jet(now, mark, t):
    """The riser's jet: dead straight back from its tail along its axis, JET lengths at most
    and short of the bound by CLEAR, white-hot to violet, fading over its last third,
    SPARKS points flaring on its line."""
    size, (cx, cy, reach) = now['size'], now['board']
    c, s = math.cos(now['heading']), math.sin(now['heading'])
    nx = now['x'] + size * TAIL * c
    ny = now['y'] + size * TAIL * s / ASPECT
    step, most, length = 0.25, JET * size, 0.0

    def at(d):
        return nx - d * c, ny - d * s / ASPECT

    while length < most:
        x, y = at(length + step)
        if math.hypot(x - cx, (y - cy) * ASPECT) < reach + CLEAR:
            break
        length += step
    flicker = 0.9 + 0.1 * math.sin(t * 37.0)
    for i in range(int(length / step) + 1):
        d = i * step
        f = min(1.0, d / most)
        a, b = (JET_INK[0], JET_INK[1]) if f < 0.5 else (JET_INK[1], JET_INK[2])
        share = 2.0 * f if f < 0.5 else 2.0 * f - 1.0
        fade = flicker * (1.0 - f) ** 0.6 * min(1.0, 3.0 * (1.0 - d / max(length, step)))
        mark(*at(d), tuple(int((p + (q - p) * share) * fade) for p, q in zip(a, b)), -1)
    for j in range(SPARKS if length else 0):
        mark(*at(hashed(int(t * 24.0), j) % 1000 / 1000.0 * length), JET_INK[0], -1)


def marker(width, height, t, board=None):
    """[(row, col, text)], gripped: the marker's frame round a corner pass's craft at `t` -
    closing in from outside, away from the board, over GRIP; gripped, squeezing once and
    holding, MARKED in its top edge. Never nearer the board than its gripped self."""
    now = pose(width, height, t, board)
    if now is None or now['kind'] != 'corner' or now['k'] < GRIP[0]:
        return [], False
    k, x, y = now['k'], now['x'], now['y']
    g = ease((k - GRIP[0]) / (GRIP[1] - GRIP[0]))
    cx, cy = (board[0], board[1]) if board else (width / 2.0, height / 2.0)
    ox, oy = x - cx, (y - cy) * ASPECT
    norm = math.hypot(ox, oy) or 1.0
    # Its offset outward outruns its corners' growth: no nearer the board than gripped.
    away = 1.5 * FRAME * math.sqrt(2.0) * now['size'] * (1.0 - g)
    half = FRAME * now['size'] * (1.0 + 1.2 * (1.0 - g) - 0.25 * math.sin(
        math.pi * min(1.0, max(0.0, (k - GRIP[1]) / 0.06))))
    mx, my = x + away * ox / norm, y + away * oy / norm / ASPECT
    top, bottom = int(my - half / ASPECT), int(my + half / ASPECT)
    left, right = int(mx - half), int(mx + half)
    out = [(top, left, '┌─'), (top, right - 1, '─┐'), (bottom, left, '└─'),
           (bottom, right - 1, '─┘')]
    if bottom - top >= 4:
        out += [(top + 1, left, '│'), (top + 1, right, '│'), (bottom - 1, left, '│'),
                (bottom - 1, right, '│')]
    gap = right - left - 3                     # between the corners' arms
    if g >= 1.0 and gap >= len(MARKED) + 2:
        out.append((top, left + 2 + (gap - len(MARKED)) // 2, MARKED))
    return out, g >= 1.0


def draw(grid, tone, buf, width, height, t, colour, board=None):
    """The craft's marker where no text is, whole or not at all; the craft over everything,
    its trail where no text is; `board` the board's bound (`bound`)."""
    if width < 40 or height < 14:
        return
    corners, gripped = marker(width, height, t, board)
    ink = (DATA if gripped else AMBER) if colour else None
    for row, col, text in corners:
        if free(grid, buf, row, col, len(text)):
            put(grid, tone, row, col, text, ink)
    for at, (mask, rgb, body) in craft(width, height, t, board).items():
        r, c = divmod(at, width)
        if body or grid[r][c] == ' ' or 0x2800 <= ord(grid[r][c]) <= 0x28FF:
            grid[r][c], tone[r][c] = chr(0x2800 + mask), rgb if colour else None
