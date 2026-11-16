"""A lit mesh in braille: its materials and light, the dot splat, the floor's grid, the fold.

The materials are `gpu.LIT_WGSL`'s, lit here alike for the dots drawn without a card; the fold
takes the dots down to ANSI cells.

    depth, rgb = splat((positions, normals, materials, uv), m, fine, centre)   # or a LitRaster
    lines = braille(depth, rgb, grid(m, fine, centre), width, height)

The meshes come from `shapes`.
"""
from coaxial.graphics.callouts import LEADER_INK
from coaxial.graphics.raster import BRAILLE, BRAILLE_BITS, DOTS_X, DOTS_Y, NOISE
from machine.rig import ansi

#: A corner's material, as `gpu.LIT_WGSL` colours it; past PAINTED the colour it wears, WOVEN with
#: it a cloth's: no highlight and a faint rim - cloth is seldom glossy. UNDER with it, the corner
#: lies under a garment: drawn where it stands, it is taken LAYER_M further from the eye where
#: the nearest is told - what a garment covers never shows through it, bent or pressed, while what
#: stands more than that before it still hides it. A card tells it a pixel, the dots a dot.
#: TWILL with it, a carbon fibre's under a clear coat: a highlight GLOSS strong, and its dots
#: drawn through a twill's screen in place of the blue noise (`braille`), the weave seen in them.
#: HARD with it, a hard shell's - a plate: every dot of it drawn, through no screen, its rim
#: HARD_RIM: under the whole rim a black frame's edges were blue, a hue no pictured robot
#: wears; with TWILL too, a carbon plate's: every dot but its weave's rib. A carbon's coat glints, GLINT sharp
#: where a paint's highlight is 40, and under the glint lies a sheen, SHEEN's first at the
#: most and its second sharp - a grey streak down a limb, never white: with a paint's
#: highlight what faced the light - a chest, a seat from behind - was a white card, a body
#: white plastic; with the glint alone it was a flat shadow.
MESH, SKIN, PLATE, CORE = 0, 1, 2, 3
PAINTED, WOVEN, UNDER, TWILL, HARD = 1 << 24, 1 << 25, 1 << 26, 1 << 27, 1 << 28
LAYER_M, GLOSS, GLINT, SHEEN, HARD_RIM = 0.03, 0.9, 160.0, (0.6, 28.0), 0.3
#: A prop drawn over all and alone in its cells (`braille`'s props): a mark on a shell.
ALONE = 2


def paint(rgb, woven=False, twill=False, hard=False):
    """The material that wears `rgb`, 0..255 each; `woven` a cloth's, `twill` a carbon fibre's,
    `hard` a hard shell's."""
    r, g, b = (int(c) & 255 for c in rgb)
    return PAINTED | WOVEN * bool(woven) | TWILL * bool(twill) | HARD * bool(hard) | r << 16 | g << 8 | b


#: The palette `gpu.LIT_WGSL` lights, for the dots drawn without a card.
PALETTE = ((0.58, 0.64, 0.72), (0.96, 0.79, 0.69), (0.80, 0.83, 0.88), (0.35, 0.85, 1.0))
KEY, FILL = (-0.45, 0.62, 0.64), (0.7, 0.1, 0.7)

_CACHED = {}


def _const():
    got = _CACHED.get('c')
    if got is None:
        from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
        key = np.asarray(KEY) / np.linalg.norm(KEY)
        fill = np.asarray(FILL) / np.linalg.norm(FILL)
        half = (key + (0.0, 0.0, 1.0)) / np.linalg.norm(key + (0.0, 0.0, 1.0))
        pal = np.asarray(PALETTE)
        bits = np.array([[BRAILLE_BITS[lane][y] for lane in range(DOTS_X)]
                         for y in range(DOTS_Y)])[None, :, None, :]
        got = _CACHED['c'] = (np, key, fill, half, pal, bits,
                              np.asarray(FLOOR_INK), np.asarray(LEADER_INK, float))
    return got


def _light(normals, materials, uv):
    """`gpu.LIT_WGSL`'s light on corners in view space: (n, 4), RGB 0..255 and the screen a dot
    of it is drawn through - 255 the blue noise, 224 none (a hard shell's), a twill's under 192,
    either hand by the side its surface faces (`braille`)."""
    np, key, fill, half, pal, *_ = _const()
    n = normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-9)
    n = np.where(n[:, 2:3] < 0.0, -n, n)
    lam = np.clip(n @ key, 0.0, None)
    fil = 0.35 * np.clip(n @ fill, 0.0, None)
    rim = (1.0 - np.clip(n[:, 2], 0.0, 1.0)) ** 3
    lobe = np.clip(n @ half, 0.0, None)
    woven, twill, hard = (materials & WOVEN) > 0, (materials & TWILL) > 0, (materials & HARD) > 0
    spec = lobe ** np.where(twill, GLINT, 40.0) + twill * (SHEEN[0] / GLOSS) * lobe ** SHEEN[1]
    base = pal[np.minimum(materials, 3)]
    worn = np.stack([(materials >> 16) & 255, (materials >> 8) & 255, materials & 255], 1)
    base = np.where((materials >= PAINTED)[:, None], worn / 255.0, base)
    g = np.abs(np.modf(uv * (22.0, 30.0))[0] - 0.5).max(axis=1)
    base = np.where((materials == MESH)[:, None] & (g > 0.4)[:, None], base * 0.5, base)
    shine = np.where(materials == SKIN, 0.12, np.where(woven, 0.0, np.where(twill, GLOSS, 0.55)))
    c = (base * (0.10 + 0.85 * lam + fil)[:, None] + (spec * shine)[:, None]
         + np.outer(rim * np.where(woven, 0.15, np.where(hard, HARD_RIM, 0.8)), (0.55, 0.75, 1.0)))
    c = np.where((materials == CORE)[:, None], base * (0.75 + 0.25 * lam)[:, None], c)
    out = np.empty((len(n), 4), np.uint8)
    out[:, :3] = np.clip(c, 0.0, 1.0) * 255.0
    out[:, 3] = np.where(twill, np.where(n[:, 0] > 0.0, 0, 128) + 64 * hard, np.where(hard, 224, 255))
    return out


def project(points, m, cam, centre):
    """(sx, sy, w) of world points: engine.project, every point at once."""
    np = _const()[0]
    rot = np.asarray(m, float).reshape(3, 3).T
    pts = np.asarray(points, float)
    if len(pts) > 64:
        q = pts @ rot
        q -= np.asarray(centre, float) @ rot
    else:
        q = (pts - centre) @ rot
    dz = cam['distance'] - q[:, 2]
    w = np.divide(1.0, dz, out=np.zeros_like(dz), where=dz != 0.0)
    sw = cam['scale'] * w
    return (cam['cx'] + sw * q[:, 0],
            cam['cy'] - sw * cam.get('aspect', 0.5) * q[:, 1], w)


def splat(arrays, m, cam, centre):
    """(depth, colour) without a card: every corner of (positions, normals, materials, uv) a
    2x2 dot splat, the nearest winning - one UNDER a garment as LAYER_M further; the colour
    `_light`'s four a dot."""
    np = _const()[0]
    positions, normals, materials, uv = arrays
    height, width = int(cam['height']), int(cam['width'])
    sx, sy, w = project(positions, m, cam, centre)
    ix = np.rint(np.clip(sx, -2.0, width)).astype(np.int32)
    iy = np.rint(np.clip(sy, -2.0, height)).astype(np.int32)
    stride = width + 1
    cell = (iy + 1) * stride + (ix + 1) + 1
    cell[(w <= 0.0) | (ix < -1) | (ix >= width) | (iy < -1) | (iy >= height)] = 0
    order = np.argsort(w / (1.0 + LAYER_M * w * ((materials & UNDER) > 0)))
    cells = np.full((height + 1) * stride + 1, -1, np.int32)
    cells[cell[order]] = order
    top = cells[1:][cells[1:] >= 0]
    ix_t, iy_t, w_t = ix[top], iy[top], w[top]
    depth = np.zeros((height, width))
    win = np.full((height, width), -1, np.int32)
    for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1)):
        x, y = ix_t + dx, iy_t + dy
        keep = (x >= 0) & (x < width) & (y >= 0) & (y < height)
        depth[y[keep], x[keep]] = w_t[keep]
        win[y[keep], x[keep]] = top[keep]
    rgb = np.zeros((height, width, 4), np.uint8)
    hit = win >= 0
    if hit.any():
        pts = win[hit]
        rgb[hit] = _light(normals[pts] @ np.asarray(m).reshape(3, 3).T, materials[pts], uv[pts])
    return depth, rgb


#: The floor's grid pitch, metres, and its half extent.
FLOOR_PITCH, FLOOR_HALF = 0.3, 2.4

#: The floor's ink at full brightness: the house's teal.
FLOOR_INK = (40, 130, 140)


def grid(m, cam, centre, travel=(0.0, 0.0), pitch=FLOOR_PITCH, half=FLOOR_HALF):
    """The floor's grid points, `pitch` apart to `half` out and scrolled `travel` (x, z) m on, as
    a (height, width) brightness: 0 none, far dimmer."""
    np = _const()[0]
    ticks = np.arange(-half, half + 1e-9, pitch)
    gx, gz = np.meshgrid(ticks - (travel[0] % pitch), ticks - (travel[1] % pitch))
    points = np.stack([gx.ravel(), np.zeros(gx.size), gz.ravel()], 1)
    sx, sy, w = project(points, m, cam, centre)
    x, y = np.rint(sx).astype(int), np.rint(sy).astype(int)
    keep = (w > 0.0) & (x >= 0) & (x < cam['width']) & (y >= 0) & (y < cam['height'])
    out = np.zeros((cam['height'], cam['width']))
    edge = np.clip(1.0 - np.hypot(gx.ravel(), gz.ravel()) / (half * 1.1), 0.0, 1.0)
    out[y[keep], x[keep]] = 0.25 + 0.75 * edge[keep]
    return out


def _mask(height, width):
    got = _CACHED.get((height, width))
    if got is None:
        np = _const()[0]
        noise = np.asarray(NOISE, float) / 4096.0
        reps = (height // len(noise) + 1, width // len(noise[0]) + 1)
        got = _CACHED[(height, width)] = np.tile(noise, reps)[:height, :width]
    return got


#: A twill's rib, dots: a 2/2 twill's diagram a dot a thread - of RIB dots along a row the
#: darkest light keeps two, a row down them a dot aside; a highlight fills the rib. A hard
#: shell's twill keeps every dot whose screen is under HARDS: three of four, the fourth its rib.
RIB, HARDS = 4, 0.75


def _screened(mask, hand):
    """`mask`, the blue noise a dot's light is held to, with a twill's screen where `hand` (a
    dot's each, `_light`'s fourth) is under 208: its ribs rising to the right under 96, to the
    left over that - the weave's two halves meet in a V where a surface faces the eye -, a
    hard shell's twill (HARDS, a quarter over its hand) its rib's one dot in RIB alone, the
    rest through: carbon as a plate, solid, its weave a hatch; and none, every dot through,
    from there under 255: a hard shell's. The twills are kept whole a raster's size and copied
    in where theirs: told a twill's dot at a time it took 2.8 ms at 300 x 300."""
    np = _const()[0]
    if not (hand < 255).any():
        return mask
    got = _CACHED.get(('twill',) + mask.shape)
    if got is None:
        y, x = np.mgrid[:mask.shape[0], :mask.shape[1]]
        ribs = [((x + way * y) % RIB + 0.5) / RIB for way in (1, -1)]
        got = _CACHED[('twill',) + mask.shape] = [ribs[0], np.where(ribs[0] > HARDS, ribs[0], 0.0),
                                                  ribs[1], np.where(ribs[1] > HARDS, ribs[1], 0.0), 0.0]
    kind, out = np.digitize(hand, (32, 96, 160, 208, 255)), mask.copy()
    for k, screen in enumerate(got):
        np.copyto(out, screen, where=kind == k)
    return out


#: Carbon's weave at a cell's scale: a tow TOW cells wide along a row, a row down it a cell
#: aside - a 2/2 twill's diagram a cell a thread -, the tows that lie under WEFT darker. A
#: dot's screen is too fine to tell from afar: a body 17 cells wide in it read as grey mail.
TOW, WEFT = 2, 0.6


def _towed(hand, covered, hits, height, width):
    """(height, width): what a cell's ink is toned by for carbon's weave - 1 where none of its
    dots is a twill's (`hand`, a dot's screen, under 208 where `covered`), 1 - WEFT where all
    are and its tow lies under: tows TOW cells wide, a row down them a cell aside, to the right
    where its dots' ribs rise right and to the left where left."""
    np = _const()[0]
    cells = np.where(covered, hand, 255).reshape(height, DOTS_Y, width, DOTS_X)
    right, left = (cells < 96).sum(axis=(1, 3)), ((cells >= 96) & (cells < 208)).sum(axis=(1, 3))
    got = _CACHED.get(('tow', height, width))
    if got is None:
        y, x = np.mgrid[:height, :width]
        got = _CACHED[('tow', height, width)] = [(x + way * y) % (2 * TOW) < TOW for way in (1, -1)]
    return 1.0 - WEFT * np.where(right >= left, got[0], got[1]) * (right + left) / np.maximum(hits, 1)


def braille(depth, rgb, floor, width, height, colour=True, overlay=None, leaders=None,
            props=()):
    """Dot rasters down to cells: a dot where the light clears its screen - the blue noise, a
    twill's where `rgb`'s fourth tells one (`_screened`), its cells toned as its tows lie
    (`_towed`) -, the silhouette and every depth step always; the floor's dots where nothing covers it, the `leaders`' dots and the `props`'
    [(dots, ink[, over])] in their inks - an ink, or one a cell, (height, width, 3) -, where
    nothing covers them or, `over`, whatever does (a force's arrow), ALONE in their cells where
    `over` is that: nothing else of what is lit drawn there (a shell's mark); the `overlay`'s
    cells {(row, col): (codepoint, key)} over all (`callouts`). Lines, ANSI where `colour`."""
    np, _k, _f, _h, _p, _bits, floor_ink, leader_ink = _const()
    covered = depth > 0.0
    over, alone = np.zeros_like(covered), None
    for dots, _ink, *on_top in props:
        if on_top and on_top[0]:
            over = over | dots
            if on_top[0] == ALONE:
                alone = dots if alone is None else (alone | dots)
        else:
            leaders = dots if leaders is None else (leaders | dots)
    # The colours laid end to end apart from a dot's screen: weighed four a dot the light took
    # 1.16 ms at 300 x 300, three 0.58 and the laying 0.39.
    tone = np.ascontiguousarray(rgb[..., :3])
    lum = tone @ (0.2126 / 255.0, 0.7152 / 255.0, 0.0722 / 255.0)
    pad = np.pad(depth, 1)
    steps = [pad[1:-1, :-2], pad[1:-1, 2:], pad[:-2, 1:-1], pad[2:, 1:-1]]
    edge = covered & np.any([(s == 0.0) | (np.abs(s - depth) > 0.04 * depth) for s in steps], 0)
    # A covered cell keeps 0.4 of its dots at the least: on 0.04 + 0.96 lum^1.5 a mid-grey shell
    # read as a cloud of points. Black paint keeps none - a wing's lettering, on the floor's 0.4 a
    # blur.
    keep = (0.40 + 0.60 * np.clip(lum, 0.0, 1.0)) * (lum > 0.004)
    mask = _mask(*depth.shape)
    lit = covered & (keep > (_screened(mask, rgb[..., 3]) if rgb.shape[-1] > 3 else mask))
    if alone is not None:                      # a mark's cells: nothing else of what is lit in them
        a4 = alone[0::4] | alone[1::4] | alone[2::4] | alone[3::4]
        lit.reshape(height, DOTS_Y, width, DOTS_X).transpose(0, 2, 1, 3)[a4[:, 0::2] | a4[:, 1::2]] = False
    lit = lit | edge | over
    ground = (floor > 0.0) & ~covered
    lead = leaders & ~covered if leaders is not None else np.zeros_like(covered)
    on_dots = lit | ground | lead
    cells = ((on_dots[0::4, 0::2] * BRAILLE_BITS[0][0])
             | (on_dots[1::4, 0::2] * BRAILLE_BITS[0][1])
             | (on_dots[2::4, 0::2] * BRAILLE_BITS[0][2])
             | (on_dots[3::4, 0::2] * BRAILLE_BITS[0][3])
             | (on_dots[0::4, 1::2] * BRAILLE_BITS[1][0])
             | (on_dots[1::4, 1::2] * BRAILLE_BITS[1][1])
             | (on_dots[2::4, 1::2] * BRAILLE_BITS[1][2])
             | (on_dots[3::4, 1::2] * BRAILLE_BITS[1][3]))
    c4 = covered[0::4].astype(int) + covered[1::4] + covered[2::4] + covered[3::4]
    hits = c4[:, 0::2] + c4[:, 1::2]
    # A dot nothing covers is black as a raster leaves it (`splat`, a card's): none is cut out.
    # A cell's ink is the mean of the dots it may draw: black paint's, never drawn, are no part
    # of it - a pad's cells over its black backing were half as bright as their foam.
    m4 = tone[0::4].astype(int) + tone[1::4] + tone[2::4] + tone[3::4]
    inked = covered & (lum > 0.004)
    i4 = inked[0::4].astype(int) + inked[1::4] + inked[2::4] + inked[3::4]
    body = (m4[:, 0::2] + m4[:, 1::2]) / np.maximum(i4[:, 0::2] + i4[:, 1::2], 1)[..., None]
    body = np.clip(body * 1.2 + 18.0, 0.0, 255.0)
    if rgb.shape[-1] > 3 and (rgb[..., 3] < 208).any():                 # carbon's tows, a cell a thread
        body = body * _towed(rgb[..., 3], covered, hits, height, width)[..., None]
    shine = floor.reshape(height, DOTS_Y, width, DOTS_X).max(axis=(1, 3))
    text = np.where(cells > 0, BRAILLE + cells, ord(' ')).tolist()
    for (row, col), (char, _ink) in (overlay or {}).items():
        text[row][col] = char
    if not colour:
        return [''.join(map(chr, row)) for row in text]
    # A cell's ink in steps of INK_STEP a channel, so runs of it share one escape; a blank cell
    # takes its left neighbour's, so a gap does not break a run. Cell by cell: 11.7 ms a frame at
    # 180 x 56, and rich parsed an escape a cell after it.
    no_hit = hits == 0
    ink = np.where((hits > 0)[..., None], body, floor_ink * shine[..., None])
    led = (lead[0::4] | lead[1::4] | lead[2::4] | lead[3::4])
    led = (led[:, 0::2] | led[:, 1::2]) & no_hit
    ink = np.where(led[..., None], leader_ink, ink)
    for dots, prop_ink, *on_top in props:
        d4 = dots[0::4] | dots[1::4] | dots[2::4] | dots[3::4]
        on = d4[:, 0::2] | d4[:, 1::2]
        on = on if on_top and on_top[0] else on & no_hit
        ink = np.where(on[..., None], np.asarray(prop_ink, float), ink)
    ink = (ink.astype(int) // INK_STEP) * INK_STEP
    key = np.where(cells > 0, (ink[..., 0] << 16) | (ink[..., 1] << 8) | ink[..., 2], -1)
    for (row, col), (_char, packed) in (overlay or {}).items():
        key[row, col] = packed
    # A blank cell takes its left neighbour's ink, not its ground.
    left = np.maximum.accumulate(np.where(key >= 0, np.arange(width), 0), axis=1)
    carried = np.take_along_axis(key, left, axis=1)
    key = np.where((key < 0) & (carried >= 0), carried & 0xFFFFFF, carried)
    # A row's runs where a cell's key differs from its left one's, found in numpy: the Python
    # only over the runs.
    lines, bounds, inked = [], key[:, 1:] != key[:, :-1], (key.max(axis=1) >= 0).tolist()
    for row, keys, steps, any_ink in zip(text, key.tolist(), bounds, inked):
        out, at, ground = [], 0, False
        line = ''.join(map(chr, row))
        for end in (np.flatnonzero(steps) + 1).tolist() + [width]:
            if keys[at] >= 0:
                out.append(_escape(keys[at]))
                if ground and keys[at] < 1 << 24:
                    out.append(NO_GROUND)
                ground = keys[at] >= 1 << 24
            out.append(line[at:end])
            at = end
        lines.append(''.join(out) + (ansi.RESET if any_ink else ''))
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
