"""The face painted at once: glow, exposure, halftone and rim on arrays.

Cell for cell what shading's passes paint one cell at a time (`wireframe._paint`, the reference
test_paint holds this to). tanh, pow and exp go through Python's own, element by element:
numpy's SIMD builds of them (SVML on Linux) differ from libm by an ulp, and a rounded glyph or
tone with it.
"""
import functools
import math

from coaxial.graphics.raster import BRAILLE, NOISE, NOISE_N, RUNGS
from coaxial.graphics.shading import (CLASS_RUNG, DENSITY_CEIL, DENSITY_FLOOR, DIMMEST, DOT_AT,
                                      DUSK, EDGE, EDGE_GLYPH, EXPOSE, EXPOSE_FOLLOW,
                                      EXPOSE_LEAST, FEATHER, GLOW, GLOW_RGB, GRAIN_COLON,
                                      GRAIN_DOT, HEAT_HI, HEAT_LO, HOTTEST, KEY, KEY_REST, KNEE,
                                      LIGHT, LIT, OUTLINE_BASE, OUTLINE_LIFT, RELIEF, RELIEF_CAP,
                                      SCAN_DOTS, SCAN_ROWS, SPOT, SPOT_AT, SPOT_R, TONE_LO,
                                      TONE_SPAN, _keylight)

#: A glyph's codepoint where nothing is painted.
BLANK = 0x20


@functools.cache
def _tables():
    """The passes' lookups as arrays, built once: each rung's smoothest glyph, the glow
    ladder, the rim's lines and the blue-noise mask."""
    from coaxial.model.blocks import numpy as np
    return {'rung': np.array([ord(LIT[r][0]) for r in range(RUNGS + 1)]),
            'rgb': np.array(GLOW_RGB, np.int64), 'rim': np.array(EDGE_GLYPH, np.int64),
            'noise': np.array(NOISE, np.int64), 'class_rung': np.array(CLASS_RUNG)}


def _mapped(fn, values, *args):
    """`fn(v, *args)` for each of `values` by Python's own float arithmetic."""
    from coaxial.model.blocks import numpy as np
    if args:
        return np.fromiter(map(fn, values.tolist(), *([a] * len(values) for a in args)),
                           float, len(values))
    return np.fromiter(map(fn, values.tolist()), float, len(values))


def blend(heat):
    """`shading._blend` on an array: the ladder's colour, packed 0xRRGGBB."""
    from coaxial.model.blocks import numpy as np
    rgb = _tables()['rgb']
    lo = np.minimum(heat.astype(np.int64), len(GLOW_RGB) - 2)
    f = (heat - lo)[:, None]
    a, b = rgb[lo], rgb[lo + 1]
    v = (a + (b - a) * f + 0.5).astype(np.int64)
    return (v[:, 0] << 16) | (v[:, 1] << 8) | v[:, 2]


def edge_tone(base):
    """`shading._edge_tone` on an array of heats, packed."""
    from coaxial.model.blocks import numpy as np
    top_heat = len(GLOW) - 1 + HOTTEST
    return blend(np.minimum(len(GLOW) - 1, base + OUTLINE_LIFT * (0.5 + 0.5 * base / top_heat)))


def _key_lit(face, at, px, py, rig):
    """`shading._key_lit` for every cell `at` under `rig` (`_keylight`'s): its neighbours' bare
    geometry against a point lamp where all four are covered, else the face-on rest."""
    from coaxial.model.blocks import numpy as np
    key, lamp, colf, rowf, distance = rig
    width, height, cls, bare, buf = (face[k] for k in ('width', 'height', 'cls', 'bare', 'buf'))
    inner = (px > 0) & (px < width - 1) & (py > 0) & (py < height - 1)
    j = np.where(inner, at, width + 1)
    inner &= ((cls[j - 1] != 0) & (cls[j + 1] != 0) & (cls[j - width] != 0)
              & (cls[j + width] != 0))
    gx = (bare[j + 1] - bare[j - 1]) * key[0]
    gy = (bare[j - width] - bare[j + width]) * key[1]
    near = buf[j]
    has = near != 0.0
    inv = 1.0 / np.where(has, near, 1.0)
    dx = lamp[0] - np.asarray(colf)[px] * inv
    dy = lamp[1] - np.asarray(rowf)[py] * inv
    dz = lamp[2] - distance + inv
    dn = np.sqrt(dx * dx + dy * dy + dz * dz)
    dn = np.where(dn == 0.0, 1.0, dn)
    dx = np.where(has, dx / dn, LIGHT[0])
    dy = np.where(has, dy / dn, LIGHT[1])
    dz = np.where(has, dz / dn, LIGHT[2])
    lit = (dz - gx * dx - gy * dy) / np.sqrt(gx * gx + gy * gy + 1.0)
    return np.where(inner, np.maximum(lit, 0.0), KEY_REST)


def _glow(face, glyph, tone, colour, cam):
    """`shading._glow`: each lit cell's glyph off its level; with colour its heat - the
    desk lamp's pool, relief, grain, the key light, the feather and the floor - and its tone.
    The heat field, or None in mono."""
    from coaxial.model.blocks import numpy as np
    t_ = _tables()
    width, height, cls = face['width'], face['height'], face['cls']
    at = face['lit']
    py, px = np.divmod(at, width)
    level = face['levels'][at] if face['levels'] is not None else cls[at].astype(float)
    hashed = face['seed'][at] if face['seed'] is not None else np.zeros(len(at))
    low = np.where(level < 1.0, 0, 1)
    step = np.clip(level - low, 0.0, 1.0)
    rungs = t_['class_rung']
    rung = rungs[low] + step * (rungs[low + 1] - rungs[low])
    glyph[at] = t_['rung'][np.clip((rung + 0.5).astype(np.int64), 1, RUNGS)]
    if not colour:
        return None
    steps = len(GLOW) - 1
    top_heat = steps + HOTTEST
    nx = (px + 0.5) / width - SPOT_AT[0]
    ny = (py + 0.5) / height - SPOT_AT[1]
    pool = 1.0 - (nx * nx + ny * ny) / (SPOT_R * SPOT_R)
    t = (level - TONE_LO) / TONE_SPAN - DUSK
    t = np.where(pool > 0.0, t + SPOT * pool * pool, t)
    bare = face['bare']
    relief = (py > 1) & (px < width - 2)
    j1 = np.where(relief, at - width + 1, 0)
    j2 = np.where(relief, at - 2 * width + 2, 0)
    relief &= (cls[j1] != 0) & (cls[j2] != 0)
    rel = np.zeros(len(at))
    r = np.flatnonzero(relief)
    rel[r] = _mapped(math.tanh, RELIEF * (bare[at[r]] + bare[j2[r]] - 2.0 * bare[j1[r]]))
    t = np.clip(t, 0.0, 1.0)
    hot = _mapped(pow, t, EDGE)
    hot = hot / (hot + _mapped(pow, 1.0 - t, EDGE))
    grain = np.where(cls[at] == 1, GRAIN_DOT, GRAIN_COLON)
    heat = hot * steps + RELIEF_CAP * steps * rel + grain * (hashed - 0.5)
    key, lamp, colf, rowf, distance, _scale = _keylight(cam, width, height)
    if (key is not None and lamp is not None and colf is not None and rowf is not None
            and distance is not None):
        rig = (key, lamp, colf, rowf, distance)
        heat = heat + KEY * (_key_lit(face, at, px, py, rig) - KEY_REST)
    heat = heat - FEATHER * (1.0 - face['coverage'][at])
    soft = ~(heat > top_heat) & (heat < DIMMEST + KNEE)
    heat = np.where(heat > top_heat, float(top_heat), heat)
    heat[soft] = DIMMEST + KNEE * _mapped(math.exp, (heat[soft] - DIMMEST - KNEE) / KNEE)
    field = np.zeros(width * height)
    field[at] = heat
    tone[at] = blend(heat)
    return field


def _expose(heat, face, persist):
    """`shading._expose`: the heat window the ladder spans this frame, followed from the
    last."""
    from coaxial.model.blocks import numpy as np
    lit = np.sort(heat[face['lit']])
    top = len(GLOW) - 1 + HOTTEST
    least = (HEAT_HI - HEAT_LO) * (top - DIMMEST)
    if len(lit) < EXPOSE_LEAST:
        lo = DIMMEST + HEAT_LO * (top - DIMMEST)
        hi = lo + least
    else:
        lo = float(lit[int(EXPOSE[0] * (len(lit) - 1))])
        hi = float(lit[int(EXPOSE[1] * (len(lit) - 1))])
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


def _slope(heat, here, before, before_ok, after, after_ok):
    """`shading._slope` on arrays: central, one-sided or flat by which neighbours are lit."""
    from coaxial.model.blocks import numpy as np
    b, a = heat[before], heat[after]
    return np.where(before_ok & after_ok, 0.5 * (a - b),
                    np.where(after_ok, a - here, np.where(before_ok, here - b, 0.0)))


def _dots(face, glyph, heat, window):
    """`shading._dots`: every lit cell's glyph off the heat field, a scan dot lit where the
    density there clears the blue noise."""
    from coaxial.model.blocks import numpy as np
    width, height, cls = face['width'], face['height'], face['cls']
    n = width * height
    lo, hi = window
    gain = 1.0 / (hi - lo) if hi > lo else 0.0
    levels = float(NOISE_N * NOISE_N)
    span = DENSITY_CEIL - DENSITY_FLOOR
    along = 4.0 / len(SCAN_ROWS)
    at = face['lit']
    py, px = np.divmod(at, width)
    reach = face['reached'][at]
    reach = np.where(reach != 0, reach, 0xFF)
    here = heat[at]
    left, right = np.maximum(at - 1, 0), np.minimum(at + 1, n - 1)
    up, down = np.maximum(at - width, 0), np.minimum(at + width, n - 1)
    gx = _slope(heat, here, left, (px > 0) & (cls[left] != 0),
                right, (px < width - 1) & (cls[right] != 0))
    gy = _slope(heat, here, up, (py > 0) & (cls[up] != 0),
                down, (py < height - 1) & (cls[down] != 0))
    base = (here - lo) * gain
    noise = _tables()['noise']
    mask = np.zeros(len(at), np.int64)
    for ox, oy, bit, lane, y in SCAN_DOTS:
        share = np.clip(base + (gx * ox + gy * oy) * gain, 0.0, 1.0)
        share = np.minimum((DENSITY_FLOOR + span * share) * along, 1.0)
        over = share * levels > noise[(py * 4 + y) % NOISE_N, (px * 2 + lane) % NOISE_N] + 0.5
        mask |= np.where(((reach & bit) != 0) & over, bit, 0)
    glyph[at] = BRAILLE + np.where(mask != 0, mask, DOT_AT[0][2])


def _rim(face, glyph, tone, heat, colour):
    """`shading._rim`: each part-covered lit cell's line over its face dots, in the edge
    tone."""
    from coaxial.model.blocks import numpy as np
    at = face['lit']
    reach = face['reached'][at]
    reach = np.where(reach != 0, reach, 0xFF)
    rim = reach != 0xFF
    at, reach = at[rim], reach[rim]
    was = glyph[at] - BRAILLE
    was = np.where((was >= 0) & (was <= 0xFF), was, 0)
    glyph[at] = BRAILLE + ((was & reach) | _tables()['rim'][reach])
    if colour:
        base = heat[at] if heat is not None else np.zeros(len(at))
        tone[at] = edge_tone(np.where(base != 0.0, base, OUTLINE_BASE))


def painted(cells, cam, m, colour, persist, foreign):
    """(cells, codepoints, packed tones or -1) for every cell the face marks, row by row:
    `wireframe._paint`'s glow, halftone, rim, outline and edge, each over the last."""
    from coaxial.model.blocks import numpy as np
    from coaxial.graphics import strokes
    buf, coverage, reached, classes, levels, bare, seed = cells
    width, height = cam['width'], cam['height']
    cls = np.frombuffer(bytes(classes), np.uint8)
    face = {'width': width, 'height': height, 'cls': cls, 'lit': np.flatnonzero(cls),
            'buf': np.asarray(buf, float), 'coverage': np.asarray(coverage, float),
            'reached': np.frombuffer(bytes(reached), np.uint8).astype(np.int64),
            'levels': None if levels is None else np.asarray(levels, float),
            'bare': None if bare is None else np.asarray(bare, float),
            'seed': None if seed is None else np.asarray(seed, float)}
    glyph = np.full(width * height, BLANK, np.int64)
    tone = np.full(width * height, -1, np.int64)
    heat = _glow(face, glyph, tone, colour, cam)
    if colour:
        _dots(face, glyph, heat, _expose(heat, face, persist))
    _rim(face, glyph, tone, heat, colour)
    if not foreign:
        strokes.outline(glyph, tone, face['buf'], cam, m, colour, heat)
        strokes.edge(glyph, tone, face['reached'], cam, colour, heat)
    at = np.flatnonzero(glyph != BLANK)
    return at, glyph[at], tone[at]
