"""A lit mesh in braille: its materials and light, the dot splat, the floor's grid, the fold.

The materials are `gpu.LIT_WGSL`'s, lit here alike for the dots drawn without a card; the fold
takes the dots down to ANSI cells.

    depth, rgb = splat((positions, normals, materials, uv), m, fine, centre)   # or a LitRaster
    lines = braille(depth, rgb, grid(m, fine, centre), width, height)

Its meshes come from `shapes`.
"""
from coaxial.graphics.callouts import LEADER_INK
from coaxial.graphics.raster import BRAILLE, BRAILLE_BITS, DOTS_X, DOTS_Y, NOISE
from machine import ansi

#: A corner's material, as `gpu.LIT_WGSL` colours it; past PAINTED the colour it wears.
MESH, SKIN, PLATE, CORE = 0, 1, 2, 3
PAINTED = 1 << 24


def paint(rgb):
    """The material that wears `rgb`, 0..255 each."""
    r, g, b = (int(c) & 255 for c in rgb)
    return PAINTED | r << 16 | g << 8 | b


#: The palette `gpu.LIT_WGSL` lights, for the dots drawn without a card.
PALETTE = ((0.58, 0.64, 0.72), (0.96, 0.79, 0.69), (0.80, 0.83, 0.88), (0.35, 0.85, 1.0))
KEY, FILL = (-0.45, 0.62, 0.64), (0.7, 0.1, 0.7)


def _light(normals, materials, uv):
    """`gpu.LIT_WGSL`'s light on corners in view space: (n, 3) RGB 0..255."""
    from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
    key = np.asarray(KEY) / np.linalg.norm(KEY)
    fill = np.asarray(FILL) / np.linalg.norm(FILL)
    n = normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-9)
    n = np.where(n[:, 2:3] < 0.0, -n, n)
    lam = np.clip(n @ key, 0.0, None)
    fil = 0.35 * np.clip(n @ fill, 0.0, None)
    rim = (1.0 - np.clip(n[:, 2], 0.0, 1.0)) ** 3
    half = (key + (0.0, 0.0, 1.0)) / np.linalg.norm(key + (0.0, 0.0, 1.0))
    spec = np.clip(n @ half, 0.0, None) ** 40
    base = np.asarray(PALETTE)[np.minimum(materials, 3)]
    worn = np.stack([(materials >> 16) & 255, (materials >> 8) & 255, materials & 255], 1)
    base = np.where((materials >= PAINTED)[:, None], worn / 255.0, base)
    g = np.abs(np.modf(uv * (22.0, 30.0))[0] - 0.5).max(axis=1)
    base = np.where((materials == MESH)[:, None] & (g > 0.4)[:, None], base * 0.5, base)
    shine = np.where(materials == SKIN, 0.12, 0.55)
    c = (base * (0.10 + 0.85 * lam + fil)[:, None] + (spec * shine)[:, None]
         + np.outer(rim * 0.8, (0.55, 0.75, 1.0)))
    c = np.where((materials == CORE)[:, None], base * (0.75 + 0.25 * lam)[:, None], c)
    return (np.clip(c, 0.0, 1.0) * 255.0).astype(np.uint8)


def project(points, m, cam, centre):
    """(sx, sy, w) of world points: engine.project, every point at once."""
    from coaxial.model.blocks import numpy as np
    q = (np.asarray(points) - centre) @ np.asarray(m).reshape(3, 3).T
    w = 1.0 / (cam['distance'] - q[:, 2])
    return (cam['cx'] + cam['scale'] * w * q[:, 0],
            cam['cy'] - cam['scale'] * cam.get('aspect', 0.5) * w * q[:, 1], w)


def splat(arrays, m, cam, centre):
    """(depth, colour) without a card: every corner of (positions, normals, materials, uv) a
    2x2 dot splat, the nearest winning."""
    from coaxial.model.blocks import numpy as np
    positions, normals, materials, uv = arrays
    height, width = cam['height'], cam['width']
    sx, sy, w = project(positions, m, cam, centre)
    colour = _light(normals @ np.asarray(m).reshape(3, 3).T, materials, uv)
    depth = np.zeros((height, width), np.float32)
    rgb = np.zeros((height, width, 3), np.uint8)
    order = np.argsort(w)
    ix, iy = np.rint(sx[order]).astype(int), np.rint(sy[order]).astype(int)
    for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1)):
        x, y = ix + dx, iy + dy
        keep = (x >= 0) & (x < width) & (y >= 0) & (y < height)
        depth[y[keep], x[keep]] = w[order][keep]
        rgb[y[keep], x[keep]] = colour[order][keep]
    return depth, rgb


#: The floor's grid pitch, metres, and its half extent.
FLOOR_PITCH, FLOOR_HALF = 0.3, 2.4

#: The floor's ink at full brightness: the house's teal.
FLOOR_INK = (40, 130, 140)


def grid(m, cam, centre, travel=0.0, pitch=FLOOR_PITCH, half=FLOOR_HALF):
    """The floor's grid points, `pitch` apart to `half` out and scrolled `travel` m on, as a
    (height, width) brightness: 0 none, far dimmer."""
    from coaxial.model.blocks import numpy as np
    ticks = np.arange(-half, half + 1e-9, pitch)
    gx, gz = np.meshgrid(ticks, ticks - (travel % pitch))
    points = np.stack([gx.ravel(), np.zeros(gx.size), gz.ravel()], 1)
    sx, sy, w = project(points, m, cam, centre)
    x, y = np.rint(sx).astype(int), np.rint(sy).astype(int)
    keep = (w > 0.0) & (x >= 0) & (x < cam['width']) & (y >= 0) & (y < cam['height'])
    out = np.zeros((cam['height'], cam['width']))
    edge = np.clip(1.0 - np.hypot(gx.ravel(), gz.ravel()) / (half * 1.1), 0.0, 1.0)
    out[y[keep], x[keep]] = 0.25 + 0.75 * edge[keep]
    return out


def _mask(height, width):
    from coaxial.model.blocks import numpy as np
    noise = np.asarray(NOISE, float) / 4096.0
    reps = (height // len(noise) + 1, width // len(noise[0]) + 1)
    return np.tile(noise, reps)[:height, :width]


def braille(depth, rgb, floor, width, height, colour=True, overlay=None, leaders=None,
            props=()):
    """Dot rasters down to cells: a dot where the light clears the blue noise, the silhouette and
    every depth step always; the floor's dots where nothing covers it, the `leaders`' dots and
    the `props`' [(dots, ink)] in their inks; the `overlay`'s cells {(row, col): (codepoint,
    key)} over all (`callouts`). Lines, ANSI where `colour`."""
    from coaxial.model.blocks import numpy as np
    covered = depth > 0.0
    for dots, _ink in props:
        leaders = dots if leaders is None else (leaders | dots)
    lum = rgb.astype(float) @ (0.2126, 0.7152, 0.0722) / 255.0
    pad = np.pad(depth, 1)
    steps = [pad[1:-1, :-2], pad[1:-1, 2:], pad[:-2, 1:-1], pad[2:, 1:-1]]
    edge = covered & np.any([(s == 0.0) | (np.abs(s - depth) > 0.04 * depth) for s in steps], 0)
    lit = (covered & (0.04 + 0.96 * np.clip(lum, 0.0, 1.0) ** 1.5 > _mask(*depth.shape))) | edge
    ground = (floor > 0.0) & ~covered
    lead = leaders & ~covered if leaders is not None else np.zeros_like(covered)
    bits = np.array([[BRAILLE_BITS[lane][y] for lane in range(DOTS_X)] for y in range(DOTS_Y)])
    cells = ((lit | ground | lead).reshape(height, DOTS_Y, width, DOTS_X)
             * bits[None, :, None, :]).sum(axis=(1, 3))
    hits = covered.reshape(height, DOTS_Y, width, DOTS_X).sum(axis=(1, 3))
    body = ((rgb * covered[..., None]).reshape(height, DOTS_Y, width, DOTS_X, 3).sum(axis=(1, 3))
            / np.maximum(hits, 1)[..., None])
    body = np.clip(body * 1.2 + 18.0, 0.0, 255.0)
    shine = floor.reshape(height, DOTS_Y, width, DOTS_X).max(axis=(1, 3))
    text = np.where(cells > 0, BRAILLE + cells, ord(' ')).tolist()
    for (row, col), (char, _ink) in (overlay or {}).items():
        text[row][col] = char
    if not colour:
        return [''.join(map(chr, row)) for row in text]
    # A cell's ink in steps of INK_STEP a channel, so runs of it share one escape; a blank cell
    # takes its left neighbour's, so a gap does not break a run. Cell by cell: 11.7 ms a frame at
    # 180 x 56, and rich parsed an escape a cell after it.
    ink = np.where((hits > 0)[..., None], body, np.asarray(FLOOR_INK) * shine[..., None])
    led = lead.reshape(height, DOTS_Y, width, DOTS_X).any(axis=(1, 3)) & (hits == 0)
    ink = np.where(led[..., None], np.asarray(LEADER_INK, float), ink)
    for dots, prop_ink in props:
        on = (dots & ~covered).reshape(height, DOTS_Y, width, DOTS_X).any(axis=(1, 3))
        ink = np.where((on & (hits == 0))[..., None], np.asarray(prop_ink, float), ink)
    ink = (ink.astype(int) // INK_STEP) * INK_STEP
    key = np.where(cells > 0, (ink[..., 0] << 16) | (ink[..., 1] << 8) | ink[..., 2], -1)
    for (row, col), (_char, packed) in (overlay or {}).items():
        key[row, col] = packed
    # A blank cell takes its left neighbour's ink, not its ground.
    left = np.maximum.accumulate(np.where(key >= 0, np.arange(width), 0), axis=1)
    carried = np.take_along_axis(key, left, axis=1)
    key = np.where((key < 0) & (carried >= 0), carried & 0xFFFFFF, carried)
    lines = []
    for row, keys in zip(text, key.tolist()):
        out, at, ground = [], 0, False
        line = ''.join(map(chr, row))
        for end in [i for i in range(1, width) if keys[i] != keys[i - 1]] + [width]:
            if keys[at] >= 0:
                out.append(_escape(keys[at]))
                if ground and keys[at] < 1 << 24:
                    out.append(NO_GROUND)
                ground = keys[at] >= 1 << 24
            out.append(line[at:end])
            at = end
        lines.append(''.join(out) + (ansi.RESET if max(keys) >= 0 else ''))
    return lines


#: A channel's step in the drawn ink.
INK_STEP = 8

#: Escapes by packed ink.
_ESCAPES = {}


#: The terminal's own ground again.
NO_GROUND = '\033[49m'


def _escape(packed):
    got = _ESCAPES.get(packed)
    if got is None:
        fg = packed & 0xFFFFFF
        got = ansi.code((fg >> 16, (fg >> 8) & 255, fg & 255))
        if packed >= 1 << 24:
            bg = (packed >> 24) - 1
            got += ansi.back((bg >> 16, (bg >> 8) & 255, bg & 255))
        _ESCAPES[packed] = got
    return got
