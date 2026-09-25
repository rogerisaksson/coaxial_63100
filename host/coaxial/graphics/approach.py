"""The approach from the pilot's seat: gates down to a pad, the scene banking, a HUD - eye candy.

The flight is a function of the view's clock (`flight`): level a while, a turn left or
right bending the corridor, level again; the craft banks into the bend and turns with it,
the nose bobs. The ground (`coaxial.graphics.ground`) is cast flat and cached; each frame
the corridor is laid on it and the whole is rolled by the bank and lifted by the bob.
The craft that passes now and then is `coaxial.graphics.craft`. The HUD
is conformal: its ladder is the horizon itself at the camera's own degrees a row, its arc
the bank, its tape the heading the bend turns (2026-09-24, "Elite, Blade Runner,
Nostromo: an orbital approach, from the pilot's eyes"). The board's attitude is the
panel's beside it, never these.
"""
import bisect
import math

from coaxial.graphics.raster import BRAILLE_BITS

AMBER = (255, 176, 0)
RED = (255, 72, 48)
DIM = (150, 104, 0)

#: A cell's height in widths: rolling the scene is a rotation of the screen as seen.
ASPECT = 2.0

#: The flight in legs from t 0, each drawn from its own seed: level for HOLD seconds (least,
#: most), then a turn of TURN_S seconds bending the corridor to DEPTH of +-CURVE (1/model
#: unit), rolled into and out of over ROLL_IN seconds at most, and level again - each state
#: held, never dithered. A turn reverses the last with REVERSE, else goes on the same way.
#: The craft banks BANK degrees at full curvature, into the bend, and turns TURN degrees a
#: second there; the nose bobs BOB degrees over BOB_S. The corridor's far end searches
#: for a site the while: each LOOK seconds it slides to the next candidate, over SLIDE
#: seconds (least, most), and holds there.
CURVE = 0.045
BANK = 14.0
TURN = 6.0
HOLD = (3.0, 11.0)
TURN_S = (4.0, 13.0)
DEPTH = (0.35, 1.0)
ROLL_IN = 2.5
REVERSE = 0.65
BOB = 1.2
BOB_S = 7.0
HEADING = 274.0
LOOK = 3.0
SLIDE = (0.7, 1.6)

#: The legs laid so far: (start s, seconds, peak curvature signed, heading at start, the
#: last turn's way); _STARTS their starts.
_LEGS, _STARTS = [], []


def _share(i, salt, span):
    """A number in `span` (least, most), leg `i`'s own."""
    return span[0] + (span[1] - span[0]) * (hashed(i, salt) % 1000) / 1000.0


def _swept(x, d, r):
    """Seconds at full curvature, integrated to `x` into a turn of `d` rolled over `r`."""
    def lead(u):
        return u ** 3 - u ** 4 / 2.0                     # the smoothstep's integral
    if x < r:
        return r * lead(x / r)
    if x <= d - r:
        return r / 2.0 + x - r
    return r / 2.0 + d - 2.0 * r + r * (0.5 - lead((d - x) / r))


def _leg(t):
    """The leg under way at `t`: (start, seconds, peak, heading at start, way)."""
    while not _LEGS or _LEGS[-1][0] + _LEGS[-1][1] <= t:
        i = len(_LEGS)
        if _LEGS:
            start, d, peak, heading, way = _LEGS[-1]
            if peak:
                heading += TURN * peak / CURVE * _swept(d, d, min(ROLL_IN, d / 3.0))
            start += d
        else:
            start, heading, way = 0.0, HEADING, -1.0
        if i % 2:
            way = -way if hashed(i, 21) % 1000 < REVERSE * 1000 else way
            d, peak = _share(i, 22, TURN_S), way * CURVE * _share(i, 23, DEPTH)
        else:
            d, peak = _share(i, 24, HOLD), 0.0
        _LEGS.append((start, d, peak, heading % 360.0, way))
        _STARTS.append(start)
    return _LEGS[bisect.bisect_right(_STARTS, t) - 1]


def flight(t):
    """The craft at `t` seconds: curve (1/unit, + to the right), bank (deg, + right wing
    down), bob (deg, + nose up), heading (deg), all smooth in `t`."""
    t = max(0.0, t)
    start, d, peak, heading, _way = _leg(t)
    x, r = t - start, min(ROLL_IN, d / 3.0)
    if peak:
        share = 1.0 if r <= x <= d - r else ease(min(x, d - x) / r)
        heading += TURN * peak / CURVE * _swept(x, d, r)
    else:
        share = 0.0
    curve = peak * share
    return {'curve': curve, 'bank': BANK * curve / CURVE,
            'bob': BOB * math.sin(2.0 * math.pi * t / BOB_S), 'heading': heading % 360.0,
            'search': _search(t)}


def _site(j):
    """Candidate site `j`: (across, dip), each -1..1."""
    return (hashed(j, 31) % 2001 / 1000.0 - 1.0, hashed(j, 32) % 2001 / 1000.0 - 1.0)


def _search(t):
    """(across, dip) the corridor's far end looks at, `t` seconds in: sliding to the next
    candidate at each LOOK, holding it after."""
    j = int(t // LOOK)
    k = ease((t - j * LOOK) / _share(j, 33, SLIDE))
    (a0, d0), (a1, d1) = _site(j - 1), _site(j)
    return a0 + (a1 - a0) * k, d0 + (d1 - d0) * k


#: The corridor, in the pilot's eye: square gates each GATE_EVERY deep from GATE_FIRST to
#: GATE_LAST ahead, sized and placed by 1/depth - the nearest frames the board, NEAR of
#: the frame wide; the farthest a few columns. The path falls away from the line of
#: sight as depth squared, reaching DIVE of the frame's height below its centre at
#: GATE_LAST - a steep dive to a point on the surface, short of the horizon, the gates
#: narrowing to it as they near it (x (1 - depth / GATE_LAST)) - and bends as depth
#: to the 1.5 with the flight's curvature, SWING of the frame's width at full bend. They
#: close on the craft with the floor's speed. GATE_HAZE of the amber is left at the last.
#: The far end sweeps for a site (`flight`'s search, each -1..1): SEARCH of the frame's
#: width across at GATE_LAST, as depth to the 1.5, and SEARCH_DIP of its dive.
NEAR = 0.8
GATE_EVERY = 3.0
GATE_FIRST = 1.2
GATE_LAST = 26.0
DIVE = 0.36
SWING = 0.32
GATE_HAZE = 0.35
SEARCH = 0.3
SEARCH_DIP = 0.18
#: A gate within GREEN of the craft is cleared: green, just before it is passed.
GREEN = 0.9
CLEARED = (90, 255, 150)

#: The landing pad where the corridor closes: PAD of the frame wide, seen from high above
#: (FLAT of a square's height), a cross on it, its corner beacons blinking red.
PAD = 0.035
FLAT = 0.4


def corridor(static, width, height, travel, curve, roll, segment, beacons=False,
             search=(0.0, 0.0)):
    """{cell: (braille mask, (r, g, b))}: the gates `travel` model units closer, each a
    square at its depth on the path, hazed by it, its corners moved by `roll` and its
    edges drawn dot by dot by `segment` (ground._segment)."""
    cx, cy = width / 2.0, height / 2.0
    size = NEAR * width * GATE_FIRST                     # half-width x depth, columns
    fall = DIVE * height / GATE_LAST ** 2 * (1.0 + SEARCH_DIP * search[1])
    bend = SWING * width / GATE_LAST ** 1.5 * (curve / CURVE if CURVE else 0.0)
    seek = SEARCH * width * search[0] / GATE_LAST ** 1.5
    shown = []
    z = GATE_FIRST + (GATE_EVERY - travel % GATE_EVERY) % GATE_EVERY
    while z <= GATE_LAST:
        half = 0.5 * size / z * (1.0 - z / GATE_LAST)
        shown.append((z, cx + (bend + seek) * z ** 1.5, cy + fall * z * z, half))
        z += GATE_EVERY
    out = {}
    land = (cx + (bend + seek) * GATE_LAST ** 1.5, cy + fall * GATE_LAST ** 2)

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
            segment(lambda a, b, _d, _k, rgb=rgb: put(a, b, rgb), (x0, y0, z), x1, y1, z, 0)
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

def hashed(*n):
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
        x = hashed(i, 1) % (2 * width) / 2.0 + 0.25
        top = sky(x) - 1.0
        if top <= 0.5:
            continue
        y = hashed(i, 2) % 1000 / 1000.0 * top
        base = 0.35 + 0.65 * (hashed(i, 3) % 100) / 100.0
        fast, faster = 2.3 + (hashed(i, 4) % 9) * 0.37, 5.1 + (hashed(i, 5) % 11) * 0.41
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


def ease(k):
    k = min(1.0, max(0.0, k))
    return k * k * (3.0 - 2.0 * k)


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


def put(grid, tone, row, col, text, ink, clear=False, buf=None):
    """`text` into the grid at (row, col), clipped to it and, given `buf`, kept off the
    board's cells; `clear`: its blanks too."""
    if not 0 <= row < len(grid):
        return
    width = len(grid[row])
    for i, ch in enumerate(text):
        c = col + i
        if 0 <= c < width and (ch != ' ' or clear) and not (buf and buf[row * width + c]):
            grid[row][c], tone[row][c] = ch, ink


def free(grid, buf, row, col, n):
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
                put(grid, tone, int(fy), int(fx) - (len(label) if side < 0 else 0), label, ink,
                     buf=buf)
    for (py, px), mask in marks.items():
        if grid[py][px] == ' ' or 0x2800 <= ord(grid[py][px]) <= 0x28FF:
            grid[py][px], tone[py][px] = chr(0x2800 + mask), ink


def hud(grid, tone, buf, width, height, fl, static, scroll, gates, box, colour):
    """The overlay, the flight's: the heading tape, the roll arc, the conformal ladder,
    lock brackets round the board (`box`: first and last row and column, or None), the
    pitch and bank, the clock, the gates flown, a tag."""
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
    put(grid, tone, 0, left, ''.join(labels), amber, buf=buf)
    put(grid, tone, 1, left, ''.join(ticks), dim, buf=buf)
    put(grid, tone, 2, width // 2 - 3, '<%03d>' % (int(round(fl['heading'])) % 360), amber,
         buf=buf)

    # The roll arc: -30..30, a column each 2 degrees, the pointer on the bank.
    arc = [' '] * 31
    for deg in range(-30, 31, 10):
        arc[15 + deg // 2] = '|' if deg == 0 else "'"
    put(grid, tone, 3, width // 2 - 15, ''.join(arc), dim, buf=buf)
    pointer = 15 + int(round(max(-30.0, min(30.0, fl['bank'])) / 2.0))
    put(grid, tone, 4, width // 2 - 15 + pointer, '^', amber, buf=buf)

    # Lock brackets round the board, when it is in the frame.
    if box is not None:
        top, bottom, first, last = box
        top, bottom = max(5, top - 1), min(height - 3, bottom + 1)
        first, last = max(0, first - 2), min(width - 1, last + 2)
        for row, col, text in ((top, first, '┌─'), (top, last - 1, '─┐'),
                               (bottom, first, '└─'), (bottom, last - 1, '─┘')):
            put(grid, tone, row, col, text, amber, buf=buf)
        if int(t * 2.0) % 2 == 0:
            put(grid, tone, top, first + 3, 'ﾛｯｸ ｵﾝ  ', amber, clear=True, buf=buf)

    # The view's pitch (its centre against the horizon) and bank; the clock, the gates.
    pitch = -math.degrees(math.atan((height / 2.0 - static['hrow']) / static['rows'])) \
        + fl['bob']
    put(grid, tone, height - 2, 1, 'ﾋﾟｯﾁ %+05.1f  ﾊﾞﾝｸ %s%04.1f' % (
        pitch, 'R' if fl['bank'] >= 0 else 'L', abs(fl['bank'])), dim, buf=buf)
    minutes, seconds = divmod(t, 60.0)
    put(grid, tone, height - 1, 1, 'T+%02d:%04.1f  ｹﾞｰﾄ %02d' % (minutes, seconds,
                                                                gates % 100), amber,
         buf=buf)
    if int(t * 1.5) % 2 == 0:
        put(grid, tone, height - 1, width - len(TAG) - 1, TAG, red, buf=buf)
    put(grid, tone, height - 2, width - len(SUBTAG) - 1, SUBTAG, data, buf=buf)
