"""The face's light: depth ramp, tone ladder, lamps, halftone, rim glyph, face art and cast shadow.
"""
import functools
import math
import os

from coaxial.draw.ansi import rgb as _rgb
from coaxial.graphics.raster import BRAILLE, BRAILLE_BITS, NOISE, NOISE_N, RUNGS, SHADE
from coaxial.graphics.solids import _casters


#: The light in VIEW space, over the viewer's shoulder: the key light's and
#: the cast shadows'. The face's base shading is depth, not Lambert: the
#: exporter's cube cuts its '.'/':' boundary across flat faces.
LIGHT = (0.60, 0.20, 0.77)

#: The shading constants, fitted by tools/lightfit.py against the
#: exporter's renders and PROVEN against test_render.py's analytic
#: oracle - do not hand-tweak what the fitter measures. SUN_MIN is the
#: grazing cutoff for the shadow test, SHADOW_DIM the cast-shadow class
#: step, BIAS the acne guard, FLOOR the darkest a bare-geometry cell
#: draws.
SUN_MIN = 0.31
SHADOW_DIM = 0.91
#: The fitted 0.24 exceeded every measured occluder gap (max 0.235
#: across five poses) - cast shadows never fired anywhere. 0.06 is
#: ~2.6 texels of the 56-cell map, enough for acne and nothing else.
BIAS = 0.06
FLOOR = 0.55


#: The face art draws verbatim - the artist's glyphs; depth lights it
#: through tone alone.

#: The ramp in the exporter's own charset, darkest first: censused over every
#: render, ' ' 3751, '.' 2265, ':' 14183 and nothing else. The 3D lives in
#: the characters; colour alone on a constant ':' read as no 3D, twice.
LIT = SHADE


#: Rungs per class in a mono render, near the exporter's densities (' ',
#: '.', ':' at 0, 96, 129 luma): 2.8 dots a cell, measured. 6 against 2 read
#: as a slab (3.6 dots, "blocky").
CLASS_RUNG = (0, 2, 4)


#: The level band the ladder spans in mono; rung 0 is blank. With colour
#: the glyph comes off `heat` (`_glow`).
LEVEL_LO, LEVEL_HI = 0.0, 2.0


#: The heat band the ladder spans, as a share of DIMMEST to the glow's top:
#: fitted at zoom 1 - 368 lit cells, p5-p95 heat 1.87-3.83 - 0.22 to 0.52.
#: The full range put every interior cell on one rung.
HEAT_LO, HEAT_HI = 0.20, 0.55


#: THE FACE IS A HALFTONE at dot resolution: a dot lights where the light at
#: its position clears a blue-noise threshold (`raster.NOISE`, 64 x 64),
#: fixed in screen space so a turning board moves the density, not the dots.
#: Rejected in rasters: one rung a cell (a carpet), per-dot in a fixed order
#: (a lattice), 8 x 8 Bayer at 150 x 44 (2 x 2 blocks, "blocky as hell"),
#: interleaved gradient noise and R2 (regular screens).

#: Where each of a cell's eight dots sits, in cells from the cell's
#: centre - two lanes a quarter cell either side, four rows at eighths
#: down - and which bit it is.
DOT_AT = tuple((lane * 0.5 - 0.25, (y - 1.5) / 4.0, BRAILLE_BITS[lane][y])
               for lane in range(2) for y in range(4))


#: The darkest lit density, a share of the dots: the tone carries the light,
#: the dots the shape. Rastered: 0.30-0.85 a brick wall (the braille box is
#: narrower than the cell), 0.08-0.35 a dusting; 0.42 since the face became
#: scanlines - 84 % of a lit row, only the darkest keeps gaps (0.35 and 0.5
#: rastered beside it).
DENSITY_FLOOR = 0.42


#: And the brightest: above half the dots a lit raised part saturated into a
#: solid block.
DENSITY_CEIL = 0.5


#: The face is scanlines: only these dot rows light, the density doubled
#: along them. Scattered points read as grain ("a shade pixelly"); two rows
#: always dark keep bricks out. Rastered beside the stipple.
SCAN_ROWS = (0, 2)


#: The dots `_dots` samples, once: offset, bit, lane and row, SCAN_ROWS only.
#: Per-dot int() and membership on all eight were a third of the pass.
SCAN_DOTS = tuple(
    (ox, oy, bit, 0 if ox < 0.0 else 1, int((oy + 0.5) * 4.0))
    for ox, oy, bit in DOT_AT if int((oy + 0.5) * 4.0) in SCAN_ROWS)


#: The exposure: the lit heat's percentiles at the ladder's ends, per frame,
#: followed at 0.3 a frame. A fixed window left 316 of 453 cells on rung 1
#: at 30 degrees, measured; it is never narrower than HEAT_HI - HEAT_LO.
EXPOSE = (0.05, 0.95)
EXPOSE_FOLLOW = 0.3
EXPOSE_LEAST = 16


#: Under this heat over the floor the tone rolls off toward DIMMEST: a hard
#: floor pinned half a 45-degree board at one tone (p50 0.40, measured).
KNEE = 1.0


#: One hue, the console's cyan, as a luminance ladder; grey read as a dead
#: channel beside the phosphor.
GLOW = (16, 23, 30, 37, 44, 51, 87, 123, 195)


#: The ladder as RGB, so fractional heat blends between rungs (the palette
#: alone broke the lamp's falloff into iso-lines). Rungs under 4.5:1 on black
#: get rewritten by a minimum-contrast terminal: .vscode/settings.json turns
#: VS Code's off (it lifted 72 % of a y45 board to one brightness).
GLOW_RGB = tuple(_rgb(c) for c in GLOW)


def _blend(heat):
    """The ladder colour at a fractional heat, linearly between rungs;
    the top rung itself is the ladder's last colour."""
    lo = min(int(heat), len(GLOW_RGB) - 2)
    f = heat - lo
    a, b = GLOW_RGB[lo], GLOW_RGB[lo + 1]
    return (int(a[0] + (b[0] - a[0]) * f + 0.5),
            int(a[1] + (b[1] - a[1]) * f + 0.5),
            int(a[2] + (b[2] - a[2]) * f + 0.5))


#: The glow sigmoid's exponent, t^E / (t^E + (1-t)^E). A power curve went
#: dark (tones 16-30); 2.2 split a y45 board into two tones; near 1 is the
#: exporter's near-linear ramp.
EDGE = 0.87


#: The tone ramp on the class scale, fitted by tools/tonecheck.py to the
#: exporter's lit screenshots: '.' at luma 93-99, ':' at 128-130, tones 1.7
#: and 2.8. Peak-channel matching landed two tones bright (hue differs).
TONE_LO = -0.47
TONE_SPAN = 7.0


#: The lighting rig: a weak even backlight - DIMMEST is its floor, no
#: cell falls to black - and ONE spot. SPOT_AT places it in frame
#: fractions (upper right), SPOT_R is its radius, SPOT its strength in
#: t-units; the falloff is quadratic to the rim. Tone only - the glyph
#: classes, the oracle and the calibration never see it.
SPOT = 0.30

#: Global dusk: the whole scene sits this far down the ramp before the
#: spot lifts its pool - the weak backlight, a shade darker on request.
DUSK = 0.24
SPOT_AT = (1.05, -0.08)
SPOT_R = 3.45


#: THE KEY LIGHT on the tone: Lambert on the screen-space normal (the
#: gradient of `bare` between neighbours) against LIGHT, the shadows' beam.
#: KEY is rungs per unit n.L; KEY_REST is n.L at the frame's centre for a
#: face-on board, so the calibrated rest tone stands. A point lamp at
#: KEY_DISTANCE along LIGHT, not a direction: a parallel beam lit a flat
#: board one tone. Before it the colour was flat - luma 97-142 for 80 % of
#: the face, identical at rest, 25 and 45 degrees.
KEY = 3.5
KEY_REST = 0.77
KEY_DISTANCE = 2.2


#: Relief off the depth buffer: a cell against its neighbour toward the lamp,
#: so a part's lit edge ignites. Through tanh, capped at RELIEF_CAP of the
#: ramp (a cliff made "black holes").
RELIEF = 9.0
RELIEF_CAP = 0.11


#: Texture: tone steps a cell's glow varies by its seed, per class, fitted by
#: tools/tonecheck.py --fit: the exporter's '.' cells differ by 21 luma (sd
#: 24, correlation 0.22), his ':' by 8-10 (0.5-0.7); ours were 2.8.
GRAIN_DOT = 1.45
GRAIN_COLON = 0.0


#: The tone ramp's working band: the darkest drawn cell never falls to
#: pure black and the hottest never reaches full white - both ends are
#: reserved so the picture always keeps its ink readable.
DIMMEST = 0.4
HOTTEST = -1


#: Tone steps a rim cell loses when the model misses ALL of it; it
#: loses the share it misses. A quarter missed drops 0.38 - 12 luma,
#: inside the exporter's own cell-to-cell spread - half 0.75, three
#: quarters 1.1.
FEATHER = 1.5


#: The board's face art (boardface.txt), a texture on the board plane, so it
#: turns with the attitude.

#: Ink per art character, the emboss's height field.
_DENSE = {' ': 0, '.': 1, ':': 2, '*': 3}


@functools.cache
def _face():
    """(rows, width, height, ink density per cell) of the board face art,
    read once; an empty face without the file."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'boardface.txt')
    try:
        with open(path, encoding='ascii') as art:
            rows = art.read().splitlines()
    except OSError:
        rows = []
    wide = max((len(r) for r in rows), default=0)
    rows = [r.ljust(wide) for r in rows]
    dense = [[_DENSE.get(c, 2) for c in r] for r in rows]
    return rows, wide, len(rows), dense


#: Shadow maps per attitude bucket. Rebuilt only when the rotation
#: leaves its ~9 degree bin: built every frame the map cost 14 ms and
#: the frame rate fell from 160 to 49, measured. A stepping shadow is
#: invisible at three glyphs; a 49 fps view is not.
_SHADOWS = {}


def _shadowmap(m, size=56, extent=1.3):
    """The scene from the LIGHT: an orthographic depth raster along the
    beam, so any cell can ask whether something sits sunward of it - the
    cast shadow a component throws across the pcb.
    """
    key = tuple(int(v * 6.5) for v in m)
    got = _SHADOWS.get(key)
    if got is not None:
        return got
    if len(_SHADOWS) > 64:
        _SHADOWS.clear()
    solid = _casters()
    pos, idx, _nrm = solid
    m0, m1, m2, m3, m4, m5, m6, m7, m8 = m
    lx, ly, lz = LIGHT
    rn = math.sqrt(lz * lz + lx * lx)
    right = (lz / rn, 0.0, -lx / rn)
    up = (ly * right[2] - lz * right[1],
          lz * right[0] - lx * right[2],
          lx * right[1] - ly * right[0])
    n = len(pos) // 3
    la, lb, lc = [0.0] * n, [0.0] * n, [0.0] * n
    half = (size - 1) / 2.0
    for i in range(n):
        x, y, z = pos[3 * i], pos[3 * i + 1], pos[3 * i + 2]
        tx = m0 * x + m1 * y + m2 * z
        ty = m3 * x + m4 * y + m5 * z
        tz = m6 * x + m7 * y + m8 * z
        la[i] = ((tx * right[0] + ty * right[1] + tz * right[2])
                 / extent + 1.0) * half
        lb[i] = ((tx * up[0] + ty * up[1] + tz * up[2])
                 / extent + 1.0) * half
        lc[i] = tx * lx + ty * ly + tz * lz

    sbuf = [-9.0] * (size * size)
    for t in range(len(idx) // 3):
        a, b, c = idx[3 * t], idx[3 * t + 1], idx[3 * t + 2]
        x0, y0, x1, y1, x2, y2 = la[a], lb[a], la[b], lb[b], la[c], lb[c]
        area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
        if area == 0.0:
            continue
        da, db, dc = lc[a], lc[b], lc[c]
        if area < 0.0:
            x1, y1, x2, y2 = x2, y2, x1, y1
            db, dc = dc, db
            area = -area
        lo_x = max(0, int(min(x0, x1, x2)))
        hi_x = min(size - 1, int(max(x0, x1, x2)) + 1)
        lo_y = max(0, int(min(y0, y1, y2)))
        hi_y = min(size - 1, int(max(y0, y1, y2)) + 1)
        inv = 1.0 / area
        # The row terms hoisted as in engine.raster: the same floats, less work
        # per pixel.
        e0x, e0y = x2 - x1, y2 - y1
        e1x, e1y = x0 - x2, y0 - y2
        for py in range(lo_y, hi_y + 1):
            row = py * size
            r0 = e0x * (py - y1)
            r1 = e1x * (py - y2)
            for px in range(lo_x, hi_x + 1):
                w0 = (r0 - e0y * (px - x1)) * inv
                if w0 < 0.0:
                    continue
                w1 = (r1 - e1y * (px - x2)) * inv
                if w1 < 0.0:
                    continue
                w2 = 1.0 - w0 - w1
                if w2 < 0.0:
                    continue
                here = w0 * da + w1 * db + w2 * dc
                at = row + px
                if here > sbuf[at]:
                    sbuf[at] = here
    made = (sbuf, size, extent, right, up, (lx, ly, lz))
    _SHADOWS[key] = made
    return made


#: The depth ramp: class = PIVOT + SLOPE * view-z / reach. Anchored on
#: the exporter's cube - deepest visible face '.', near faces ':' -
#: and fitted from there by tools/lightfit.py.
PIVOT = 2.375
SLOPE = 1.30


#: An edge's tone: the cell's own heat lifted OUTLINE_LIFT rungs, the full
#: lift at the ladder's top, half at its floor - brighter than the face under
#: it, glinting in the lamp's pool (bench: "highlighted with the light, not
#: just thicker"). OUTLINE_BASE is the heat where the face gave none.
#: Measured in luma over four attitudes: a lift of 2.5 put the line +51 over
#: the face, "barely noticeable"; 4.5 put it +82. 3.0 since the line lies on
#: the face's dots (2026-09-23): 4.5 glowed as a halo, 1.5 vanished.
OUTLINE_LIFT = 3.0
OUTLINE_BASE = 3.0


def _mono(level, phase=0.0):
    """One cell from the CLASS scale alone, for a render with no colour."""
    low = 0 if level < 1.0 else 1
    step = level - low
    step = 0.0 if step < 0.0 else (1.0 if step > 1.0 else step)
    rung = CLASS_RUNG[low] + step * (CLASS_RUNG[low + 1] - CLASS_RUNG[low])
    return _pattern(int(rung + 0.5), phase)


def _pattern(rung, phase):
    """The glyph for `rung` dots, phased."""
    rung = 1 if rung < 1 else (RUNGS if rung > RUNGS else rung)
    # THE SMOOTHEST PATTERN OF THE RUNG, EVERY TIME.
    return LIT[rung][0]


def _floor(heat):
    """The tone's floor as a roll-off: at or above `DIMMEST + KNEE`
    the heat is its own, below it decays toward DIMMEST and never
    reaches it, continuous in value and slope at the knee."""
    if heat >= DIMMEST + KNEE:
        return heat
    return DIMMEST + KNEE * math.exp((heat - DIMMEST - KNEE) / KNEE)


def _expose(heat, classes, persist=None):
    """The heat window the ladder spans this frame: `(lo, hi)`, lo at rung 0
    and hi at the top rung.
    """
    lit = sorted(heat[i] for i in range(len(classes)) if classes[i])
    top = len(GLOW) - 1 + HOTTEST
    least = (HEAT_HI - HEAT_LO) * (top - DIMMEST)
    if len(lit) < EXPOSE_LEAST:
        lo = DIMMEST + HEAT_LO * (top - DIMMEST)
        hi = lo + least
    else:
        lo = lit[int(EXPOSE[0] * (len(lit) - 1))]
        hi = lit[int(EXPOSE[1] * (len(lit) - 1))]
    if len(lit) >= EXPOSE_LEAST and hi - lo < least:
        mid = 0.5 * (lo + hi)
        lo, hi = mid - 0.5 * least, mid + 0.5 * least
    was = persist.get('exposure') if persist is not None else None
    if was is not None:
        lo = was[0] + EXPOSE_FOLLOW * (lo - was[0])
        hi = was[1] + EXPOSE_FOLLOW * (hi - was[1])
    if persist is not None:
        persist['exposure'] = (lo, hi)
    return lo, hi


def _key_lit(px, py, at, width, height, classes, bare, key, lamp, colf,
             rowf, distance, buf):
    """The key light on one cell: central differences of bare geometry where
    both neighbours are covered, else the face-on rest, so a silhouette
    cell neither flares nor drops.
    """
    if not (0 < px < width - 1 and 0 < py < height - 1
            and classes[at - 1] and classes[at + 1]
            and classes[at - width] and classes[at + width]):
        return KEY_REST
    gx = (bare[at + 1] - bare[at - 1]) * key[0]
    gy = (bare[at - width] - bare[at + width]) * key[1]
    dx, dy, dz = LIGHT
    if buf is not None and buf[at]:
        inv = 1.0 / buf[at]
        dx = lamp[0] - colf[px] * inv
        dy = lamp[1] - rowf[py] * inv
        dz = lamp[2] - distance + inv
        dn = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
        dx, dy, dz = dx / dn, dy / dn, dz / dn
    lit = (dz - gx * dx - gy * dy) / math.sqrt(gx * gx + gy * gy + 1.0)
    return max(lit, 0.0)


def _slope(before, here, after):
    """The heat's slope across a cell from its two neighbours along one
    axis: central where both are lit, one-sided where one is, flat
    where neither - a silhouette cell takes no gradient from the air."""
    if before is not None and after is not None:
        return 0.5 * (after - before)
    if after is not None:
        return after - here
    if before is not None:
        return here - before
    return 0.0


def _dots(grid, heat, classes, coverage, width, height, window,
          reached=None):
    """The glyphs, off the heat field: each of a cell's eight dots is lit
    where the density at the dot clears its blue-noise threshold.
    """
    lo, hi = window
    gain = 1.0 / (hi - lo) if hi > lo else 0.0
    levels = float(NOISE_N * NOISE_N)
    span = DENSITY_CEIL - DENSITY_FLOOR
    along = 4.0 / len(SCAN_ROWS)
    floor = DENSITY_FLOOR
    n = NOISE_N
    lone = DOT_AT[0][2]
    for py in range(height):
        row = py * width
        # The mask's rows for this cell row, looked up once per row rather than
        # once per dot.
        nrow = [NOISE[(py * 4 + y) % n] for y in range(4)]
        for px in range(width):
            at = row + px
            if not classes[at]:
                continue
            reach = (reached[at] if reached is not None and reached[at]
                     else 0xFF)
            here = heat[at]
            gx = _slope(heat[at - 1] if px and classes[at - 1] else None,
                        here,
                        heat[at + 1] if px < width - 1 and classes[at + 1]
                        else None)
            gy = _slope(heat[at - width] if py and classes[at - width]
                        else None,
                        here,
                        heat[at + width] if py < height - 1
                        and classes[at + width] else None)
            base = (here - lo) * gain
            mask = 0
            for ox, oy, bit, lane, y in SCAN_DOTS:
                if not reach & bit:
                    continue
                share = base + (gx * ox + gy * oy) * gain
                share = 0.0 if share < 0.0 else (1.0 if share > 1.0
                                                 else share)
                share = (floor + span * share) * along
                if share > 1.0:
                    share = 1.0
                if share * levels > nrow[y][(px * 2 + lane) % n] + 0.5:
                    mask |= bit
            grid[py][px] = chr(BRAILLE + (mask or lone))


def _edge_tone(base):
    """The tone an edge takes - a part's outline, the rim - off the heat
    under it, lifted by OUTLINE_LIFT toward the ladder's top rung, which
    HOTTEST keeps from the face: readable ink is what that rung is
    reserved for.
    """
    top_heat = len(GLOW) - 1 + HOTTEST
    return _blend(min(len(GLOW) - 1,
                      base + OUTLINE_LIFT * (0.5 + 0.5 * base / top_heat)))


def _edge_glyphs():
    """The braille LINE for every way a cell can be part-covered: of the
    dots the model reaches, those beside a dot it misses - across the
    lane, or the row above or below, inside the cell.
    """
    table = [0] * 256
    for reach in range(1, 255):
        bits = 0
        for lane in range(2):
            for y in range(4):
                if not reach & BRAILLE_BITS[lane][y]:
                    continue
                beside = (BRAILLE_BITS[1 - lane][y],
                          BRAILLE_BITS[lane][y - 1] if y else 0,
                          BRAILLE_BITS[lane][y + 1] if y < 3 else 0)
                if any(b and not reach & b for b in beside):
                    bits |= BRAILLE_BITS[lane][y]
        table[reach] = bits or BRAILLE_BITS[0][0]
    return tuple(table)


EDGE_GLYPH = _edge_glyphs()


def _rim(grid, tone, classes, reached, heat, width, height, colour):
    """THE EDGE, ENHANCED."""
    for py in range(height):
        row = py * width
        for px in range(width):
            at = row + px
            if not classes[at]:
                continue
            reach = (reached[at] if reached is not None and reached[at]
                     else 0xFF)
            if reach == 0xFF:
                continue
            # The line OVER the face's dots, not instead of them: replaced,
            # the face stepped a cell short of the silhouette.
            face = ord(grid[py][px]) - BRAILLE
            if not 0 <= face <= 0xFF:
                face = 0
            grid[py][px] = chr(BRAILLE + ((face & reach) | EDGE_GLYPH[reach]))
            if colour:
                base = (heat[at] if heat is not None and heat[at]
                        else OUTLINE_BASE)
                tone[py][px] = _edge_tone(base)


def _keylight(cam, width, height):
    """What the key light needs, once per frame, or `key` None with it off."""
    if cam is None or not KEY:
        return None, None, None, None, None, None
    per_bare = cam.get('reach', 1.0) / SLOPE
    distance, scale = cam['distance'], cam['scale']
    cx, cy = cam['cx'], cam['cy']
    across = distance / scale
    lx, ly, lz = LIGHT
    # The projection's per-column and per-row factors, once: the inner loop
    # multiplies, it does not divide.
    return ((0.5 * per_bare / across, 0.5 * per_bare / (2.0 * across)),
            (lx * KEY_DISTANCE, ly * KEY_DISTANCE, lz * KEY_DISTANCE),
            [(c + 0.5 - cx) / scale for c in range(width)],
            [(cy - (r + 0.5)) / (scale * 0.5) for r in range(height)],
            distance, scale)


def _glow(grid, tone, classes, levels, bare, seed, coverage, width, height,
          colour, cam=None, buf=None, heat_out=None):
    """Classes to glyphs, unrounded levels to the colour ramp."""
    steps = len(GLOW) - 1
    lo, span = TONE_LO, TONE_SPAN
    spot_x, spot_y = SPOT_AT
    rr = SPOT_R * SPOT_R
    top_heat = steps + HOTTEST
    key, lamp, colf, rowf, distance, scale = _keylight(cam, width, height)
    for py in range(height):
        row = py * width
        ny = (py + 0.5) / height - spot_y
        for px in range(width):
            cls = classes[row + px]
            if not cls:
                continue
            at = row + px
            # THE GLYPH CARRIES THE LEVEL, not the rounded class.
            hashed = seed[at] if seed is not None else 0.0
            level = levels[at] if levels is not None else float(cls)
            grid[py][px] = _mono(level, hashed)
            if not colour:
                continue
            nx = (px + 0.5) / width - spot_x
            # The desk-lamp pool: squared falloff lands at ZERO slope on the
            # rim.
            pool = 1.0 - (nx * nx + ny * ny) / rr
            t = (level - lo) / span - DUSK
            if pool > 0.0:
                t += SPOT * pool * pool
            # Relief is the SECOND difference of bare geometry, joined after
            # the sigmoid in tone steps.
            rel = 0.0
            if py > 1 and px < width - 2 and classes[at - width + 1] \
                    and classes[at - 2 * width + 2]:
                rel = math.tanh(
                    RELIEF * (bare[at] + bare[at - 2 * width + 2]
                              - 2.0 * bare[at - width + 1]))
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            hot = t ** EDGE
            hot = hot / (hot + (1.0 - t) ** EDGE)
            grain = GRAIN_DOT if cls == 1 else GRAIN_COLON
            heat = (hot * steps + RELIEF_CAP * steps * rel
                    + grain * (hashed - 0.5))
            # The key light: central differences of bare geometry where both
            # neighbours are covered, else the face-on rest, so a silhouette
            # cell neither flares nor drops.
            if (key is not None and lamp is not None and colf is not None
                    and rowf is not None and distance is not None):
                heat += KEY * (_key_lit(px, py, at, width, height, classes,
                                        bare, key, lamp, colf, rowf,
                                        distance, buf) - KEY_REST)
            # Anti-aliasing is the GLYPH only: a staircase corner (two or more
            # empty neighbours) thins ':' to '.'.
            missed = 1.0 - coverage[at]
            if missed:
                heat -= FEATHER * missed
            heat = top_heat if heat > top_heat else _floor(heat)
            if heat_out is not None:
                heat_out[at] = heat
            tone[py][px] = _blend(heat)
            # The glyph comes off the same heat in `_dots`, once the whole
            # field is known: it samples the neighbours too.
