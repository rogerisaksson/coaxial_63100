"""The board's ASCII renderer: a depth-shaded raster, wire fallback.

The parametric board's triangles raster through a z-buffer; DEPTH is
the shading - class = PIVOT + SLOPE * view-z on the exporter's own
three-glyph ramp ' .:' - with the shipped face art as pure class steps
on the horizontal surfaces and cast shadows from a light-space map.
Depth, not surface-normal light: the exporter's cube reference showed
its '.'-to-':' boundary cutting ACROSS a flat face, which no lambert
can produce, and the whole fitted lambert path was removed on that
evidence.

The engine itself lives in coaxial/engine.py as five pure stages. Two
gates hold it honest - tests/test_render.py proves every stage and the
whole chain against an analytic ray-cast oracle, tools/facecheck.py
calibrates against the CAD exporter's own renders - and render/render_demo.ps1
runs both.

`face=False` falls back to the chosen-edge wire drawing: rim rings,
struts, bore and component cages, hidden lines removed by the same
z-buffer, strokes following their screen direction.
"""
import math

#: OUTER and BORE are the parametric board's own - one definition, and the
#: two drawings can never disagree about the hole. THICK is this module's
#: and deliberately twice the real slab: a vector drawing honest about a
#: 1.6 mm laminate reads as a single line, and the two rim rings are what
#: sells the turn.
from .orientation import BORE, OUTER                       # noqa: E402

THICK = 0.05

#: Degrees the camera tips from straight-down toward the horizon. Enough
#: for the ground plane to READ as a landscape; small enough that a tared
#: board is still a face-on circle to the eye.
CAMERA_TIP = 34.0

#: The ground plane's height under the board, model units.
GROUND = -1.3

#: Struts between the two rim rings, and segments per ring.
RIM_SEGMENTS = 36
RIM_STRUTS = 12

#: The same palette orientation's toon uses, so both drawings agree about
#: what a part is. (dark, mid, bright) per zone, picked by depth.
SHADES = {
    'board': (17, 23, 37),
    'connector': (94, 166, 208),
    'fet': (130, 208, 214),
    'micro': (24, 38, 51),
    'passive': (22, 65, 114),
}


def _ring(radius, z, segments=RIM_SEGMENTS):
    """Closed ring of edges at height `z`, as [(a, b), ...] vertex pairs."""
    ring = [(radius * math.cos(2 * math.pi * i / segments),
             radius * math.sin(2 * math.pi * i / segments), z)
            for i in range(segments)]
    return [(ring[i], ring[(i + 1) % segments]) for i in range(segments)]


def _cage(phi_deg, radius, half_r, half_phi_deg, height, relief=1.5):
    """A part as its polar box's edges: top rectangle plus four posts."""
    half_phi = math.radians(half_phi_deg)
    phi0 = math.radians(phi_deg)
    top, base = THICK + height * relief, THICK

    def at(ri, pi, z):
        r = radius + (half_r if ri else -half_r)
        phi = phi0 + (half_phi if pi else -half_phi)
        return (r * math.cos(phi), r * math.sin(phi), z)

    corners = [(0, 0), (1, 0), (1, 1), (0, 1)]
    edges = []
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        edges.append((at(*a, top), at(*b, top)))       # the lid
        edges.append((at(*a, base), at(*a, top)))      # the post
    return edges


def _build():
    """[(a, b, zone)] - the whole board as chosen edges, built once."""
    from . import orientation

    edges = []
    for z in (THICK, -THICK):
        edges += [(a, b, 'board') for a, b in _ring(OUTER, z)]
    for i in range(RIM_STRUTS):
        phi = 2 * math.pi * i / RIM_STRUTS
        a = (OUTER * math.cos(phi), OUTER * math.sin(phi), THICK)
        b = (OUTER * math.cos(phi), OUTER * math.sin(phi), -THICK)
        edges.append((a, b, 'board'))
    edges += [(a, b, 'board') for a, b in _ring(BORE, THICK, 18)]

    for part in orientation.COMPONENTS:
        zone = part[0]
        edges += [(a, b, zone) for a, b in _cage(*part[1:])]
    return edges


_EDGES = None
_SOLID = None

#: In-memory decimates of the STL, keyed on (path, divisions, mtime):
#: a fresh export replaces them by itself, and NOTHING is written
#: beside the model - the mesh module's disk cache is deliberately
#: bypassed here.
_MESHES = {}


def _decimated(path, divisions):
    import os
    from . import mesh
    stamp = (path, divisions, os.path.getmtime(path))
    got = _MESHES.get(stamp)
    if got is None:
        # Six LODs and the shadow casters of one file coexist; only a
        # runaway set - a re-exported STL changing every stamp - clears
        # the lot.
        if len(_MESHES) > 8:
            _MESHES.clear()
        got = mesh._clustered(mesh.loaded(path), divisions)
        _MESHES[stamp] = got
    return got


#: The decimation each zoom band earns, (zoom below, grid divisions).
#: Finer decimates cost real raster time - measured at 94x36, single
#: process: 16 -> 7.8 ms, 24 -> 13, 32 -> 29, 48 -> 37, 64 -> 63 - so
#: each only draws past the zoom that can see it. The parse is 0.46 s
#: once a process, then 0.14-0.18 s a grid; all six 1.2 s in parallel,
#: each worker parsing its own - which is why a view loads
#: them up front, behind its boot strip, not on the first zoom that
#: wants one.
LODS = ((1.0, 12), (1.5, 16), (2.0, 24), (2.7, 32), (3.6, 48), (None, 64))


def _lods(progress=None):
    """Every level of detail's solid, coarse to fine - what a crew holds,
    since the zoom picks among them by identity. The ones not yet in
    memory decimate IN PARALLEL, one process each; `progress(done,
    total, divisions)` is called as each lands."""
    import os
    from . import orientation
    path = orientation.MODEL
    stamp = os.path.getmtime(path)
    missing = [d for _z, d in LODS if (path, d, stamp) not in _MESHES]
    if len(missing) > 1:
        from . import crew
        if len(_MESHES) + len(missing) > 8:
            _MESHES.clear()
        for divisions, solid in crew.decimate(path, missing, progress).items():
            _MESHES[(path, divisions, stamp)] = solid
    return [_decimated(path, divisions) for _z, divisions in LODS]


#: The coarsest grid a view with a crew draws. Grid 16 is a polygon with
#: a notch in its rim and a hole in its face at the attitude view's
#: size - decimation artefacts, seen; 32 is clean, and with eight
#: workers a 94x36 frame at 32 costs ~35 ms, inside a 20 Hz budget.
CREW_LEAST = 32


def _model(zoom=1.0, least=0):
    """(edges, solid) - the solid at the LEVEL OF DETAIL the zoom earns,
    and never coarser than `least` divisions.

    Decimation fixed at 16 divisions, a zoomed-in board showed the same
    coarse facets bigger; the finer decimates cost real raster time, so
    they only load past the zoom that can see them."""
    global _EDGES, _SOLID
    if _EDGES is None:
        _EDGES = _build()
    from . import orientation
    divisions = next(d for upto, d in LODS
                     if d >= least and (upto is None or zoom < upto))
    try:
        solid = _decimated(orientation.MODEL, divisions)
    except (OSError, ValueError):
        # The parametric board, only for a tree without the STL.
        if _SOLID is None:
            _SOLID = orientation.facets(steps=48, relief=1.5)
        solid = _SOLID
    return _EDGES, solid


#: The cast-shadow beam in VIEW space, over the viewer's shoulder. The
#: SHADING is depth, not light - a surface-normal lambert was built,
#: fitted and REMOVED: the exporter's cube proved its '.'/'∶' boundary
#: cuts across flat faces, which only a depth ramp does. This vector
#: only decides where component shadows fall.
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


#: Backdrop samples per window size. The ground never moves - it is the
#: camera's world, not the board's - so the projection is computed once
#: per (width, height) and each frame only replays it against the
#: depth buffer.
_BACKDROP = {}


def _backdrop(width, height, distance, view):
    """[(px, py, stroke)] for the ground grid: own scale, own centre.

    NOT the board's projection. The fitted scale magnifies a dinner
    plate to fill the frame, and at that magnification the horizon
    projects some forty rows above the window - what stayed on screen
    was the strip of rungs around the board, which read as two horizons
    with the far half exiting through the top. The backdrop shares the
    camera's tip but picks its scale from two anchors: the horizon near
    the top of the frame, the ground under the camera just off the
    bottom, so lines always rise from the lower edge and terminate ON
    the horizon."""
    got = _BACKDROP.get((width, height))
    if got is not None:
        return got

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
        sx, sy, w = ray(wx, wy)
        return int(cx + scale * sx), int(cy - scale * 0.5 * sy), w

    marks = []
    hy = int(round(hrow))
    for px in range(width):
        marks.append((px, hy, '-'))

    # The fan is DOTTED, one dot per row per line. Directional strokes
    # were tried first: near-horizontal segments alternated glyphs and
    # every line came out ragged - `/-` `|-` pairs all the way down.
    # Dots have no direction to disagree about, and a dotted floor under
    # a block-shaded subject is the separation itself.
    # SEVEN dotted lines, a dot per row, nothing else. Thirteen lines
    # needed a crowding veto near the convergence, and every veto tried
    # - modular, greedy row-claim, sign-alternating - broke the fan's
    # symmetry some way the eye caught. Few enough lines never crowd,
    # and the small cluster where they meet the horizon IS the
    # vanishing point.
    # Wide enough that the outer lines meet the horizon past the frame
    # edges - the fan covers the WHOLE line, not a band in the middle.
    samples = height * 6
    for k in range(-8, 9):
        fixed = k * 2.2
        drawn = height + 9
        for i in range(samples + 1):
            wy = south + (far - south) * (i / samples) ** 2
            px, py, w = cast(fixed, wy)
            if w <= 0.0 or w > 2.0 or py >= drawn:
                continue
            if 0 <= px < width and hy < py < height:
                marks.append((px, py, '.'))
                drawn = py

    _BACKDROP[(width, height)] = marks
    return marks


def _ground(grid, tone, buf, distance, width, height, colour, view):
    """The landscape behind the board: the cached backdrop, replayed
    against this frame's depth buffer so the board occludes it.

    Plain occlusion, no halo: the depth solid's footprint already
    reaches a cell or two past the visible dither, and the one-cell
    keepout tried on top of that clipped the backdrop visibly far from
    the subject. Solid blocks against dim dots need no gap to separate."""
    dim = 238 if colour else None
    ash = 244 if colour else None
    for px, py, stroke in _backdrop(width, height, distance, view):
        if buf[py * width + px] == 0.0 and grid[py][px] == ' ':
            grid[py][px] = stroke
            tone[py][px] = ash if stroke == '-' else dim



#: The art draws VERBATIM - its own characters on screen, its blanks
#: left blank. Block-shade and ramp remappings were both built and both
#: rejected: the artist picked those glyphs, and the picture is theirs.
#: Depth still lights the face, but through TONE alone.

#: The face's light ramp IN THE EXPORTER'S OWN CHARSET, darkest to
#: brightest, with the tone each glyph wears. Censused over every
#: shipped render: ' ' 3751, '.' 2265, ':' 14183 and NOTHING else, so
#: the ramp is exactly those three. In an ASCII render the 3D lives in
#: the characters - the exporter's tilted views brighten a near edge to
#: denser glyphs and starve a far edge to sparser ones - so lighting
#: that only changed the COLOUR of a constant ':' carpet read as no 3D
#: at all, twice, on two different attempts.
LIT = ' .:'
#: ONE hue, the console theme's cyan, as a pure luminance ladder:
#: black through the teals to white-cyan for the sharpest highlight.
#: The object never changes colour - only how much light its
#: characters carry. Plain grey was tried and read as a dead channel
#: next to the stage's phosphor.
GLOW = (16, 23, 30, 37, 44, 51, 87, 123, 195)


def _rgb(code):
    """An xterm-256 cube colour as (r, g, b)."""
    c = code - 16
    return tuple(0 if v == 0 else 55 + 40 * v
                 for v in (c // 36, (c // 6) % 6, c % 6))


#: The same ladder as RGB, so a fractional heat can sit BETWEEN two
#: rungs: sent as 24-bit colour, the ramp is continuous. On the palette
#: alone a lit board lived on three cyans and the lamp's falloff broke
#: into hard iso-lines wherever a rounding boundary crossed the face.
#: The lower rungs sit under 4.5:1 contrast on black (2.8 at rung 1)
#: and a terminal enforcing a minimum contrast rewrites them: VS Code's
#: default lifted 72% of a y45 board's cells to one brightness -
#: .vscode/settings.json turns that off for this workspace.
GLOW_RGB = tuple(_rgb(c) for c in GLOW)


def _blend(heat):
    """The ladder colour at a fractional heat, linearly between rungs."""
    lo = int(heat)
    f = heat - lo
    a, b = GLOW_RGB[lo], GLOW_RGB[lo + 1]
    return (int(a[0] + (b[0] - a[0]) * f + 0.5),
            int(a[1] + (b[1] - a[1]) * f + 0.5),
            int(a[2] + (b[2] - a[2]) * f + 0.5))

#: Contrast of the glow SIGMOID t^E / (t^E + (1-t)^E): the midtone
#: stays put while shadows deepen and highlights sharpen together. A
#: plain power curve was tried first and pulled the whole board into
#: the dark end - tones 16..30 and nothing lit, measured. At 2.2 the
#: curve split a y45 board into two tones (47% of cells in one bin,
#: ':' pinned at 7); near 1 it is the exporter's own near-linear ramp.
EDGE = 0.87

#: The tone ramp runs on the CLASS scale, fitted by tools/tonecheck.py
#: to the exporter's lit screenshots (tests/renders/*.png): measured
#: there in Rec.709 luma, '.' cells sit at 93-99 and ':' at 128-130,
#: flat across the picture - tone 1.7 and 2.8 of GLOW. Matched on the
#: peak channel instead they landed two tones bright: his cyan-blue
#: and the console's cyan differ in hue. Scaled to PIVOT +- SLOPE the
#: ramp spanned the unclipped depth instead and pinned ':' at 7.
TONE_LO = -0.47
TONE_SPAN = 7.0

#: The lighting rig: a weak even backlight - DIMMEST is its floor, no
#: cell falls to black - and ONE spot. SPOT_AT places it in frame
#: fractions (upper right), SPOT_R is its radius, SPOT its strength in
#: t-units; the falloff is quadratic to the rim. Tone only - the glyph
#: classes, the oracle and the calibration never see it.
SPOT = 0.20

#: Global dusk: the whole scene sits this far down the ramp before the
#: spot lifts its pool - the weak backlight, a shade darker on request.
DUSK = 0.24
SPOT_AT = (1.05, -0.08)
SPOT_R = 3.45

#: Screen-space relief off the real depth buffer: a cell is compared
#: with its neighbour TOWARD the lamp, so a component edge facing the
#: light ignites and its far side drops into shadow. The global ramp
#: cannot see parts a tenth of a unit proud of the slab - one step of
#: nine - and the board read as one flat sheet, benched. The term goes
#: through tanh, so a cliff in the buffer saturates at RELIEF_CAP of
#: the ramp instead of slamming a cell to the end - the "black holes".
RELIEF = 9.0
RELIEF_CAP = 0.11

#: Surface texture: tone steps, peak to peak, a cell's glow varies by
#: its fixed seed - per glyph class, sized by tools/tonecheck.py --fit
#: against the exporter's screenshots. Measured there, his '.' dots
#: differ from their neighbours by 21 luma on average (sd 24, neighbour
#: correlation 0.22 - texture, not gradient) while his ':' blocks are
#: smooth (8-10 luma, correlation 0.5-0.7). Ours without it: 2.8 luma
#: between neighbours, a flat wash.
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

#: The board's face as a shipped ASCII raster - a dithered render of the
#: real layout, boardface.txt beside this module. Sampled as a texture
#: on the board plane, so the picture turns WITH the attitude instead of
#: a cage of edges suggesting it.
_FACE = None


#: Ink per art character, the emboss's height field.
_DENSE = {' ': 0, '.': 1, ':': 2, '*': 3}


def _face():
    global _FACE
    if _FACE is None:
        import os
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
        _FACE = (rows, wide, len(rows), dense)
    return _FACE


#: Shadow maps per attitude bucket. Rebuilt only when the rotation
#: leaves its ~9 degree bin: built every frame the map cost 14 ms and
#: the frame rate fell from 160 to 49, measured. A stepping shadow is
#: invisible at three glyphs; a 49 fps view is not.
_SHADOWS = {}


def _shadowmap(m, size=56, extent=1.3):
    """The scene from the LIGHT: an orthographic depth raster along the
    beam, so any cell can ask whether something sits sunward of it -
    the cast shadow a component throws across the pcb. Coarse in every
    axis on purpose: a soft-edged ASCII shadow needs no more, and the
    caster is a lower-resolution solid than the one that draws."""
    key = tuple(int(v * 6.5) for v in m)
    got = _SHADOWS.get(key)
    if got is not None:
        return got
    if len(_SHADOWS) > 64:
        _SHADOWS.clear()
    from . import orientation
    solid = _casters(orientation)
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
        for py in range(lo_y, hi_y + 1):
            row = py * size
            for px in range(lo_x, hi_x + 1):
                w0 = ((x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)) * inv
                if w0 < 0.0:
                    continue
                w1 = ((x0 - x2) * (py - y2) - (y0 - y2) * (px - x2)) * inv
                if w1 < 0.0:
                    continue
                w2 = 1.0 - w0 - w1
                if w2 < 0.0:
                    continue
                here = w0 * da + w1 * db + w2 * dc
                if here > sbuf[row + px]:
                    sbuf[row + px] = here
    made = (sbuf, size, extent, right, up, (lx, ly, lz))
    _SHADOWS[key] = made
    return made


_CASTERS = None


def _casters(orientation):
    """The shadow pass's own solid: the same STL, coarser still. The
    mesh cache keys on the file's mtime, so a fresh export replaces
    both solids by itself."""
    global _CASTERS
    if _CASTERS is None:
        from . import mesh
        try:
            _CASTERS = _decimated(orientation.MODEL, 10)
        except (OSError, ValueError):
            _CASTERS = orientation.facets(steps=20, relief=1.5)
    return _CASTERS


#: The depth ramp: class = PIVOT + SLOPE * view-z / reach. Anchored on
#: the exporter's cube - deepest visible face '.', near faces ':' -
#: and fitted from there by tools/lightfit.py.
PIVOT = 2.375
SLOPE = 1.30


def _glow(grid, tone, classes, levels, bare, seed, coverage, width, height,
          colour):
    """Classes to glyphs, unrounded levels to the colour ramp.

    The level spans PIVOT +- SLOPE by construction (view-z over reach
    is +-1), normalised and bent through the EDGE sigmoid: shadows
    deepen and highlights sharpen while the midtone stands. The SPOT
    adds its radial pool of light; a staircase corner on the
    SILHOUETTE thins its glyph, and nothing dims - the exporter's rim
    is as bright as his interior."""
    steps = len(GLOW) - 1
    lo, span = TONE_LO, TONE_SPAN
    spot_x, spot_y = SPOT_AT
    rr = SPOT_R * SPOT_R
    top_heat = steps + HOTTEST
    for py in range(height):
        row = py * width
        ny = (py + 0.5) / height - spot_y
        for px in range(width):
            cls = classes[row + px]
            if not cls:
                continue
            grid[py][px] = LIT[cls]
            if not colour:
                continue
            at = row + px
            nx = (px + 0.5) / width - spot_x
            # The desk-lamp pool: squared falloff lands at ZERO slope
            # on the rim. Linear-in-d2 ended at its steepest - a
            # terminator ring drawn across the board.
            pool = 1.0 - (nx * nx + ny * ny) / rr
            t = (levels[at] - lo) / span - DUSK
            if pool > 0.0:
                t += SPOT * pool * pool
            # Relief is the SECOND difference of bare geometry, joined
            # after the sigmoid in tone steps. Each rejection measured:
            # in t it died with the exposure, on `levels` the art's
            # integer ink saturated the tanh, and the first difference
            # drowned at y45 - the tilted face's own gradient (0.05 per
            # cell) matched the cos-shrunk component step. The second
            # difference cancels any uniform slope: y45 median 0.002
            # against 0.18 at the edges, which land as a bright/dark
            # cell pair - a contour line, not an area fill.
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
                    + grain * (seed[at] - 0.5))
            # Anti-aliasing is the GLYPH only: a staircase corner (two
            # or more empty neighbours) thins ':' to '.'. The tone once
            # feathered too, and that drew the LOD line: every cell on
            # the silhouette and round every hole sat at 55-62 luma
            # against 101-103 inside, a dark contour the exporter's
            # screenshots do not have (his rim 99, his interior 101).
            # Anti-aliasing by COVERAGE, from the 2x2 fold: a rim cell
            # dims by the share of it the model misses and thins its
            # glyph at half or less. A flat feather on every rim cell
            # drew a dark contour round the board and every hole (55
            # luma against 101 inside); counting empty neighbours could
            # not tell a straight edge from a stair and left it raw.
            missed = 1.0 - coverage[at]
            if missed:
                heat -= FEATHER * missed
                if missed >= 0.5 and cls == 2:
                    grid[py][px] = LIT[1]
            tone[py][px] = _blend(DIMMEST if heat < DIMMEST else
                                  (top_heat if heat > top_heat else heat))


def _glyph(dx, dy):
    """The stroke of a screen direction. dy is in cell rows (half-height)."""
    if dx == 0.0 and dy == 0.0:
        return '+'
    steep = abs(dy) * 2.0
    if steep > 2.0 * abs(dx):
        return '|'
    if abs(dx) > 2.0 * steep:
        return '-'
    return '/' if (dx > 0) != (dy > 0) else '\\'



def _raster(solid, m, cam, crew):
    """(depth, top, sun, coverage) at cell resolution: rastered at 2x2
    subsamples per cell - the camera doubled - and FOLDED, so every
    cell knows its coverage, which the rim's anti-aliasing runs on.
    Four times the raster, which is what the crew is for."""
    from . import engine
    fine = dict(cam, width=2 * cam['width'], height=2 * cam['height'],
                scale=2.0 * cam['scale'], cx=2.0 * cam['cx'],
                cy=2.0 * cam['cy'])
    if crew is not None and crew.holds(solid):
        buf, topf, sun = crew.raster(solid, m, fine, beam=LIGHT,
                                     sun_min=SUN_MIN)
    else:
        buf, topf, sun = engine.raster(solid, m, fine, beam=LIGHT,
                                       sun_min=SUN_MIN)
    return engine.fold(buf, topf, sun, cam['width'], cam['height'])


def _cells(solid, m, cam, crew, face, foreign):
    """Every per-cell stage: (depth, coverage, classes, levels, bare,
    seed). With a crew holding the solid the workers raster, fold and
    shade their own bands and the parent keeps only the glow, which
    needs neighbours. A foreign solid gets no cast shadows and no art;
    `face=False` leaves the class fields None for the wire drawing."""
    from . import engine
    width, height = cam['width'], cam['height']
    if face and crew is not None and crew.holds(solid):
        shading = (PIVOT, SLOPE, FLOOR, None if foreign else _shadowmap(m),
                   SHADOW_DIM, BIAS, not foreign)
        return crew.frame(solid, m, cam, LIGHT, SUN_MIN, shading)
    buf, topf, sun, coverage = _raster(solid, m, cam, crew)
    if not face:
        return buf, coverage, None, None, None, None
    levels = [0.0] * (width * height)
    bare = [0.0] * (width * height)
    seed = [0.0] * (width * height)
    classes = engine.shade(
        buf, topf, sun, cam, m, PIVOT, SLOPE, FLOOR,
        art=None if foreign else _face(),
        shadow=None if foreign else _shadowmap(m),
        shadow_step=SHADOW_DIM, bias=BIAS, levels=levels, bare=bare,
        seed=seed)
    return buf, coverage, classes, levels, bare, seed


def render(q, width, height, zoom=1.0, colour=True,
           horizon=True, face=True, tip=None, solid=None,
           distance=None, lift=0.44, crew=None, least=0):
    """The board under rotation `q`, as a vector drawing.

    Cell-resolution: the strokes ARE the picture, so there is no half-block
    supersampling to average them away. `horizon` draws the WORLD's level line behind
    everything, the old flight-sim cue: the board tilts, the horizon does
    not. `lift` is the model's vertical centre as a share of the frame
    (0.5 dead centre, smaller is higher). `crew` is a coaxial.crew.Crew
    holding THIS solid: the raster runs as row bands in its processes."""
    from . import ansi, engine, orientation

    # `solid` overrides the board with another mesh - facecheck proves
    # the LIGHT MODEL on the exporter's cube, whose flat faces turn a
    # shading bug into a wrong character instead of a vibe. An override
    # never wears the board's art.
    edges, board_solid = _model(
        zoom, max(least, CREW_LEAST) if crew is not None else least)
    foreign = solid is not None
    if not foreign:
        solid = board_solid
    # A NEAR-TOP camera, tipped just enough that the world grid recedes
    # to a horizon the way the old flight sims drew it. The tip is the
    # CAMERA's, applied outside the attitude, so screen X and Y still map
    # one-to-one onto board X and Y - the 55-degree gallery view mixed
    # them and sent the mount hunting through mirrors.
    # `tip` overrides the camera pitch - 0.0 looks straight down the
    # axis, the way the CAD exporter does, which is what facecheck
    # compares against.
    distance = 3.2 if distance is None else distance
    # The fit is a BOUNDING SPHERE, deliberately NOT the frame at hand.
    # Fitted per frame, a pitch about X shrank the projected span, the
    # scale swelled to refill the window and the board read as
    # TRANSLATING closer - seen on the bench 2026-08-30 and blamed on
    # the origin, which was innocent.
    # Reach comes from the SOLID's own vertices - whatever mesh the STL
    # holds today.
    pts = solid[0]
    reach = max(math.sqrt(pts[3 * i] ** 2 + pts[3 * i + 1] ** 2
                          + pts[3 * i + 2] ** 2)
                for i in range(len(pts) // 3))
    # The camera is tipped OUTSIDE the attitude, so screen X and Y map
    # one-to-one onto board X and Y; cy sits a shade above centre so the
    # ground fills the lower field.
    cam = engine.camera(width, height, reach, distance=distance,
                        zoom=zoom, tip=CAMERA_TIP if tip is None else tip,
                        lift=lift)
    view = cam['view']
    m = engine.multiply(view, orientation.matrix(q))
    scale, cx, cy = cam['scale'], cam['cx'], cam['cy']

    buf, coverage, classes, levels, bare, seed = _cells(
        solid, m, cam, crew, face, foreign)
    z_lo = min((v for v in buf if v), default=0.0)
    z_hi = max(buf, default=1.0) or 1.0
    span = (z_hi - z_lo) or 1.0

    grid = [[' '] * width for _ in range(height)]
    tone = [[None] * width for _ in range(height)]

    if horizon:
        tipm = view
        _ground(grid, tone, buf, distance, width, height, colour, tipm)

    if face:
        _glow(grid, tone, classes, levels, bare, seed, coverage, width,
              height, colour)

    m0, m1, m2, m3, m4, m5, m6, m7, m8 = m
    # The lit raster IS the picture; the chosen edges only draw in wire
    # mode. The edge-on fallback the flat texture needed is gone too -
    # the raster draws the slab's own side walls from any angle.
    for a, b, zone in ([] if face else edges):
        ax = m0 * a[0] + m1 * a[1] + m2 * a[2]
        ay = m3 * a[0] + m4 * a[1] + m5 * a[2]
        az = m6 * a[0] + m7 * a[1] + m8 * a[2]
        bx = m0 * b[0] + m1 * b[1] + m2 * b[2]
        by = m3 * b[0] + m4 * b[1] + m5 * b[2]
        bz = m6 * b[0] + m7 * b[1] + m8 * b[2]

        wa, wb = 1.0 / (distance - az), 1.0 / (distance - bz)
        x0, y0 = cx + scale * wa * ax, cy - scale * 0.5 * wa * ay
        x1, y1 = cx + scale * wb * bx, cy - scale * 0.5 * wb * by

        steps = max(1, int(max(abs(x1 - x0), 2.0 * abs(y1 - y0))))
        glyph = _glyph(x1 - x0, y1 - y0)
        shades = SHADES.get(zone, SHADES['board'])
        for i in range(steps + 1):
            t = i / steps
            px, py = int(x0 + (x1 - x0) * t), int(y0 + (y1 - y0) * t)
            if not (0 <= px < width and 0 <= py < height):
                continue
            here = wa + (wb - wa) * t
            # The solid decides visibility; the 2 % grace keeps an edge
            # from losing to the very face it borders.
            if here < buf[py * width + px] * 0.98:
                continue
            third = min(2, int(3.0 * (here - z_lo) / span))
            grid[py][px] = glyph
            tone[py][px] = shades[third] if colour else None


    if not colour:
        return '\n'.join(''.join(row).rstrip() for row in grid)
    return '\n'.join(
        ansi.run([(grid[r][c], tone[r][c]) for c in range(width)]).rstrip()
        for r in range(height))
