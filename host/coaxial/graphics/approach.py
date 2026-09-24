"""The approach from the pilot's seat: an orbital corridor bending away, the scene banking
into it, a HUD reading what the screen shows - eye candy, amber.

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
        rgb = tuple(int(c * fade + 0.5) for c in AMBER)
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


def _put(grid, tone, row, col, text, ink):
    """`text` into the grid at (row, col), clipped to it."""
    if not 0 <= row < len(grid):
        return
    for i, ch in enumerate(text):
        c = col + i
        if 0 <= c < len(grid[row]) and ch != ' ':
            grid[row][c], tone[row][c] = ch, ink


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
                _put(grid, tone, int(fy), int(fx) - (len(label) if side < 0 else 0), label, ink)
    for (py, px), mask in marks.items():
        if grid[py][px] == ' ' or 0x2800 <= ord(grid[py][px]) <= 0x28FF:
            grid[py][px], tone[py][px] = chr(0x2800 + mask), ink


def hud(grid, tone, buf, width, height, fl, static, scroll, gates, box, colour):
    """The overlay, the flight's: the heading tape, the roll arc, the conformal ladder,
    lock brackets round the board (`box`: first and last row and column, or None), the
    pitch and bank, the clock, the gates flown and a tag."""
    if width < 40 or height < 14:
        return
    amber, dim, red = (AMBER, DIM, RED) if colour else (None, None, None)
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
    _put(grid, tone, 0, left, ''.join(labels), amber)
    _put(grid, tone, 1, left, ''.join(ticks), dim)
    _put(grid, tone, 2, width // 2 - 3, '<%03d>' % (int(round(fl['heading'])) % 360), amber)

    # The roll arc: -30..30, a column each 2 degrees, the pointer on the bank.
    arc = [' '] * 31
    for deg in range(-30, 31, 10):
        arc[15 + deg // 2] = '|' if deg == 0 else "'"
    _put(grid, tone, 3, width // 2 - 15, ''.join(arc), dim)
    pointer = 15 + int(round(max(-30.0, min(30.0, fl['bank'])) / 2.0))
    _put(grid, tone, 4, width // 2 - 15 + pointer, '^', amber)

    # Lock brackets round the board, when it is in the frame.
    if box is not None:
        top, bottom, first, last = box
        top, bottom = max(5, top - 1), min(height - 3, bottom + 1)
        first, last = max(0, first - 2), min(width - 1, last + 2)
        for row, col, text in ((top, first, '┌─'), (top, last - 1, '─┐'),
                               (bottom, first, '└─'), (bottom, last - 1, '─┘')):
            _put(grid, tone, row, col, text, amber)
        if int(t * 2.0) % 2 == 0:
            _put(grid, tone, top, first + 3, 'LOCK', amber)

    # The view's pitch (its centre against the horizon) and bank; the clock, the gates.
    pitch = -math.degrees(math.atan((height / 2.0 - static['hrow']) / static['rows'])) \
        + fl['bob']
    _put(grid, tone, height - 2, 1, 'PITCH %+05.1f  BANK %s%04.1f' % (
        pitch, 'R' if fl['bank'] >= 0 else 'L', abs(fl['bank'])), dim)
    minutes, seconds = divmod(t, 60.0)
    _put(grid, tone, height - 1, 1, 'T+%02d:%04.1f  GATE %02d' % (minutes, seconds,
                                                                gates % 100), amber)
    if int(t * 1.5) % 2 == 0:
        tag = '>> ORBITAL APPROACH'
        _put(grid, tone, height - 1, width - len(tag) - 1, tag, red)
