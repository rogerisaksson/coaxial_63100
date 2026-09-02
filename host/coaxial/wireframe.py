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
SPOT = 0.30

#: Global dusk: the whole scene sits this far down the ramp before the
#: spot lifts its pool - the weak backlight, a shade darker on request.
DUSK = 0.24
SPOT_AT = (1.05, -0.08)
SPOT_R = 3.45

#: THE KEY LIGHT, on the tone. The glyph classes are depth and stay
#: depth (the exporter's evidence, above); the colour was flat with
#: them - measured, luma 97-142 for 80 % of the face and identical at
#: rest, 25 and 45 degrees of tilt, because the tone was the class plus
#: the lamp's pool and nothing that saw the surface. This is Lambert
#: on the SCREEN-SPACE normal: the gradient of `bare` (a linear
#: function of view z) between neighbouring cells, scaled by the cell's
#: size in view units, against LIGHT - the same beam the cast shadows
#: come from, one light for both. KEY is rungs per unit of n.L; KEY_REST
#: is n.L for a face-on board (LIGHT's z), subtracted so the calibrated
#: rest tone stands and a tilt toward the light brightens, away
#: darkens, and a part's wall turned from the beam falls into shade.
#:
#: A POINT, not a direction. Directional, a flat board at rest was one
#: tone - "all the pixels the same brightness" from the bench - because
#: a plane under a parallel beam IS uniform. The lamp sits KEY_DISTANCE
#: along LIGHT in view space, so the direction to it changes across the
#: face: the near side takes it squarely, the far side obliquely, and a
#: resting board carries a gradient the way one under a desk lamp does.
#: KEY_REST is n.L at the frame's centre for a face-on board, kept so
#: the calibrated middle stands.
KEY = 3.5
KEY_REST = 0.77
KEY_DISTANCE = 2.2

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

#: THE OUTLINE: the parts' edges as a wireframe overlay, from the mesh's
#: own geometry - not from the raster. Two raster passes came before it
#: and both flickered under motion, because a per-cell test on a depth
#: buffer re-decides every cell as the picture slides a fraction of a
#: cell: a crest test inked 6 % of the cells and blinked them at the
#: ladder's top. Edges projected from FIXED vertices slide with the
#: picture instead. What an edge is: a crease where two faces meet past
#: OUTLINE_DEG (a part's top against its wall - 60 rather than 45 drops
#: chamfers and the facets of a round can), taken from the EXACT mesh
#: whatever the frame draws: the export indexed at OUTLINE_EXACT, a
#: clustering step of a hundredth of a micron that merges duplicate
#: corners and moves nothing. A decimate cannot do this job - its cell
#: is a floor on the height a part needs to keep its walls: grid 64
#: (1.56 mm cells) found 9 wide loops, grid 128 twenty, and the 1.5 mm
#: micro none at any grid, because a part under a cell tall clusters
#: flat - and the height gate below cannot come down to meet it, since
#: the slab's own corners sit a cell up too. Exact, the same export gave
#: 28 loops wider than 0.12 units including the 1.5 mm class at z 0.031,
#: for 0.37 s of indexing and 0.21 s of edges once a process (116,880
#: faces). Gated by HEIGHT ABOVE THE SLAB: only edges with a vertex
#: more than OUTLINE_RISE above the slab's top, and the slab's top is
#: MEASURED from the mesh (`_slab_top`), not assumed - it was assumed
#: to be z 0 for an evening, and the export centres on its bounding
#: box so the slab sits at z -0.069, which put the gate 5 mm up and
#: hid every part under that height, the 1.5 mm micro first. With the
#: gate a millimetre over the measured slab the same export gives 44
#: loops wider than 0.12 units, the micro's among them. The pads,
#: holes and copper on the slab's face sit within 0.035 mm of it and
#: fall under the gate; the slab's own RIM and BORE are wanted - "the
#: board's edge, enhanced too" - and come back in as the slab-level
#: crease loops wider than OUTLINE_RIM, which on the exact mesh are
#: their own loops and no tangle. Then grouped into connected LOOPS and
#: drawn only when a loop spans OUTLINE_CELLS on screen: a part smaller
#: than that is a flickering fragment, not a drawing - filtered rather
#: than drawn, on the bench's word - and comes back as the zoom brings
#: it up.
OUTLINE_DEG = 60.0
OUTLINE_RISE = 0.02
#: A slab-level loop this wide is the board's own edge (the rim is 2.0,
#: the bore 0.2); the mounting holes at 0.14 and the copper are not.
OUTLINE_RIM = 0.19
#: An edge shorter than this on SCREEN, in cells, is not drawn: a QFP's
#: hundred pin creases and a can's facets are a third of a cell each at
#: the view's zoom, and stacking their dots in one cell drew a solid
#: braille blob where the part was. Measured over the 90 loops wider
#: than 0.06 units: the blobs had 24-190 edges of 0.3-0.6 mm, the
#: outlines edges of millimetres. Judged per EDGE and per FRAME rather
#: than per loop, because a loop's total length could not tell a box
#: with its footprint (twice the perimeter) from the pins - and because
#: it is the zoom that decides: bring a can up and its facets draw.
OUTLINE_MIN_EDGE = 0.5
#: And per LOOP, in model units: edges adding up to more than this many
#: widths is crease detail, not an outline. See _outline_loops.
OUTLINE_DENSITY = 6.0
#: How far behind the frame's surface an edge may sit and still draw,
#: in model units: the raster's own error across a cell is under half
#: a millimetre at any view size, and the slab is 1.6 mm - so 0.6 mm
#: keeps an edge on its face and hides a top-side part seen from below.
#: A relative 2 % grace, the wire mode's, let 1.0 % of slab through.
OUTLINE_GRACE = 0.012
#: How far a vertex may sit from the slab's measured top and still count
#: as on it: the copper and mask layers are 0.0007 units (35 um) proud.
OUTLINE_LEVEL = 0.003
#: Three, from five, on the bench's word - "more of the edge enhancer,
#: also on somewhat smaller objects": two more rings of parts come into
#: the drawing at the view's zoom.
OUTLINE_CELLS = 3
OUTLINE_EXACT = 200000

#: The line's tone: the cell's OWN heat lifted OUTLINE_LIFT rungs - "a
#: touch brighter than the rest of the object", the bench's words - so
#: an edge in the key light's shade is a touch brighter shade and one
#: in its pool a touch brighter pool, and the line never flattens the
#: lighting it sits on. A fixed rung came before it (four, then six on
#: "more of the edge enhancer") and read as one colour painted over a
#: lit surface. The lift itself follows the light: a cell at the top of
#: the ladder lifts the full OUTLINE_LIFT, one at the floor half of it,
#: so an edge in the lamp's pool glints and one in shade only shows -
#: "highlighted with the light, not just thicker", the bench's words.
#: OUTLINE_BASE is the heat a line cell takes where the face gave none
#: - the rim's dots half off the silhouette.
OUTLINE_LIFT = 2.5
OUTLINE_BASE = 3.0

#: The line's pixels: braille, a 2x4 dot matrix per cell, so an edge
#: rasters at twice the column and four times the row resolution and
#: reads as a fine dotted line - the same dots the chat page's spinner
#: spins, asked for by the bench as "pixels". Dots only: an ASCII
#: stroke fallback (`- | / \`) was built and taken out on the
#: bench's word - the slashes read as jank next to the dots, and the
#: console that runs this already shows the spinner.
BRAILLE = 0x2800
#: Bit for (column, row) of the 2x4 cell, the braille standard's order.
BRAILLE_BITS = ((0x01, 0x02, 0x04, 0x40), (0x08, 0x10, 0x20, 0x80))


def _glow(grid, tone, classes, levels, bare, seed, coverage, width, height,
          colour, cam=None, buf=None, heat_out=None):
    """Classes to glyphs, unrounded levels to the colour ramp.

    The level spans PIVOT +- SLOPE by construction (view-z over reach
    is +-1), normalised and bent through the EDGE sigmoid: shadows
    deepen and highlights sharpen while the midtone stands. The SPOT
    adds its radial pool of light and the KEY light shades by the
    surface's slope; a staircase corner on the SILHOUETTE thins its
    glyph, and nothing dims - the exporter's rim is as bright as his
    interior. `cam` sizes the cells for the key light; without it the
    key is off (the tests that hold the pool's arithmetic pass none)."""
    steps = len(GLOW) - 1
    lo, span = TONE_LO, TONE_SPAN
    spot_x, spot_y = SPOT_AT
    rr = SPOT_R * SPOT_R
    top_heat = steps + HOTTEST
    # View z per unit of `bare`, and the cell's size in view units: one
    # column is (distance - z)/scale across, one row twice that down.
    # Both at the board's own depth - the gradient is a slope, and a
    # per-cell depth in the divisor moved the answer by under 1 %.
    key = None
    if cam is not None and KEY:
        per_bare = cam.get('reach', 1.0) / SLOPE
        distance, scale = cam['distance'], cam['scale']
        cx, cy = cam['cx'], cam['cy']
        across = distance / scale
        key = (0.5 * per_bare / across, 0.5 * per_bare / (2.0 * across))
        lx, ly, lz = LIGHT
        lamp = (lx * KEY_DISTANCE, ly * KEY_DISTANCE, lz * KEY_DISTANCE)
        # The projection's per-column and per-row factors, once: the
        # inner loop multiplies, it does not divide.
        colf = [(c + 0.5 - cx) / scale for c in range(width)]
        rowf = [(cy - (r + 0.5)) / (scale * 0.5) for r in range(height)]
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
            # The key light: central differences of bare geometry where
            # both neighbours are covered, else the face-on rest, so a
            # silhouette cell neither flares nor drops.
            if key is not None:
                lit = KEY_REST
                if (0 < px < width - 1 and 0 < py < height - 1
                        and classes[at - 1] and classes[at + 1]
                        and classes[at - width] and classes[at + width]):
                    gx = (bare[at + 1] - bare[at - 1]) * key[0]
                    gy = (bare[at - width] - bare[at + width]) * key[1]
                    # The direction to the lamp from THIS cell's point,
                    # back out of the projection as engine.shade does;
                    # without a depth buffer, the beam's direction.
                    dx, dy, dz = lx, ly, lz
                    if buf is not None and buf[at]:
                        inv = 1.0 / buf[at]
                        dx = lamp[0] - colf[px] * inv
                        dy = lamp[1] - rowf[py] * inv
                        dz = lamp[2] - distance + inv
                        dn = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
                        dx, dy, dz = dx / dn, dy / dn, dz / dn
                    lit = ((dz - gx * dx - gy * dy)
                           / math.sqrt(gx * gx + gy * gy + 1.0))
                    if lit < 0.0:
                        lit = 0.0
                heat += KEY * (lit - KEY_REST)
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
            heat = (DIMMEST if heat < DIMMEST else
                    (top_heat if heat > top_heat else heat))
            if heat_out is not None:
                heat_out[at] = heat
            tone[py][px] = _blend(heat)


def _slab_top(pos):
    """The z of the slab's top face, from the mesh: the most populated
    z level - unless a level at least 60 % as populated lies a
    millimetre or more ABOVE it, which is the slab's other face when
    the bottom happened to win the count. A part's lid never comes
    near the slab's population. Measured, never assumed - see
    OUTLINE_RISE."""
    counts = {}
    for i in range(2, len(pos), 3):
        key = round(pos[i], 4)
        counts[key] = counts.get(key, 0) + 1
    if not counts:
        return 0.0
    mode = max(counts, key=counts.get)
    floor = 0.6 * counts[mode]
    higher = [z for z, n in counts.items()
              if n >= floor and z - mode >= 0.02]
    return max(higher) if higher else mode


def _features(solid, min_deg=OUTLINE_DEG, min_rise=None):
    """[(a, b)] vertex pairs: the creases of `solid`, above `min_rise`
    when one is given.

    An edge shared by two faces whose normals differ by more than
    `min_deg`, or belonging to one face only (a shell boundary), with at
    least one end above `min_rise` in z. Pure, and exact on a synthetic
    box - test_render holds it there. The normals are the mesh's own
    per-triangle ones, which for a decimate are the ORIGINAL faces' (so
    a flat top stays one plane and makes no creases of its own).
    """
    pos, idx, nrm = solid
    cos_lim = math.cos(math.radians(min_deg))
    shared = {}
    for t in range(len(idx) // 3):
        tri = (idx[3 * t], idx[3 * t + 1], idx[3 * t + 2])
        for i in range(3):
            a, b = tri[i], tri[(i + 1) % 3]
            shared.setdefault((a, b) if a < b else (b, a), []).append(t)
    out = []
    for (a, b), tris in shared.items():
        if (min_rise is not None
                and max(pos[3 * a + 2], pos[3 * b + 2]) <= min_rise):
            continue
        if len(tris) == 1:
            out.append((a, b))
            continue
        t0, t1 = tris[0], tris[1]
        dot = (nrm[3 * t0] * nrm[3 * t1] + nrm[3 * t0 + 1] * nrm[3 * t1 + 1]
               + nrm[3 * t0 + 2] * nrm[3 * t1 + 2])
        if abs(dot) < cos_lim:
            out.append((a, b))
    return out


def _loops(edges, pos):
    """[(extent, [(a, b), ...])]: the connected pieces of an edge set,
    each with its XY extent in model units - what the size filter
    judges. Union-find, no recursion."""
    parent = {}

    def find(v):
        while parent.setdefault(v, v) != v:
            parent[v] = parent[parent[v]]
            v = parent[v]
        return v

    for a, b in edges:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    groups = {}
    for a, b in edges:
        groups.setdefault(find(a), []).append((a, b))
    out = []
    for members in groups.values():
        verts = {v for e in members for v in e}
        xs = [pos[3 * v] for v in verts]
        ys = [pos[3 * v + 1] for v in verts]
        out.append((max(max(xs) - min(xs), max(ys) - min(ys)), members))
    return out


#: The outline's loops per source solid, built once per process.
_OUTLINES = {}


def _outline_source():
    """(solid, loops) the outline draws from: the export indexed exact
    (see OUTLINE_EXACT), or the parametric board where there is none.
    Built once a process; a view warms it behind its boot strip."""
    from . import orientation
    try:
        solid = _decimated(orientation.MODEL, OUTLINE_EXACT)
    except (OSError, ValueError):
        global _SOLID
        if _SOLID is None:
            _SOLID = orientation.facets(steps=48, relief=1.5)
        solid = _SOLID
    got = _OUTLINES.get(id(solid))
    if got is None:
        if len(_OUTLINES) > 4:
            _OUTLINES.clear()
        got = _OUTLINES[id(solid)] = (solid, _outline_loops(solid))
    return got


def _outline_loops(solid):
    """The loops the outline draws: every part standing OUTLINE_RISE
    over the measured slab, and the slab's own edge - the slab-level
    crease loops wider than OUTLINE_RIM."""
    pos = solid[0]
    top = _slab_top(pos)
    gate = top + OUTLINE_RISE

    # Split by height BEFORE grouping. Grouping first and judging the
    # loop whole was tried: a part's footprint shares corners with the
    # copper around it, so the pads joined the part and drew - 11,982
    # edges against 3,123, and the blobs back. Split, a part's lid and
    # corners are one loop and its footprint on the slab another, drawn
    # when it is OUTLINE_RIM wide like the rim and the bore; the copper,
    # pads and holes under that width are nothing to draw.
    parts, level = [], []
    for a, b in _features(solid):
        za, zb = pos[3 * a + 2], pos[3 * b + 2]
        if max(za, zb) > gate:
            parts.append((a, b))
        elif abs(za - top) <= OUTLINE_LEVEL and abs(zb - top) <= OUTLINE_LEVEL:
            level.append((a, b))
    loops = _loops(parts, pos)
    loops += [(extent, members) for extent, members in _loops(level, pos)
              if extent >= OUTLINE_RIM]

    # And a loop that is more crease than outline - its edges adding up
    # to over OUTLINE_DENSITY times its width - is a pin field or a
    # fin stack, not a part's shape. Measured over the 90 loops wider
    # than 0.06 units: outlines at 1.1-5 (the rim's circle exactly 3.1),
    # the blobs at 9-31.
    def sparse(extent, members):
        length = 0.0
        for a, b in members:
            length += math.sqrt((pos[3 * a] - pos[3 * b]) ** 2
                                + (pos[3 * a + 1] - pos[3 * b + 1]) ** 2
                                + (pos[3 * a + 2] - pos[3 * b + 2]) ** 2)
        return length <= OUTLINE_DENSITY * extent

    return [(e, m) for e, m in loops if sparse(e, m)]


def _outline(grid, tone, buf, cam, m, colour, heat=None):
    """The wireframe overlay: every loop wide enough to read, as dotted
    lines in the cells' 2x4 braille matrix, hidden where the solid
    stands in front - the 2 % grace keeps an edge from losing to the
    face it borders. `heat` is the glow pass's per-cell heat, which the
    line lifts by OUTLINE_LIFT. Cells drawn, for the caller that
    counts."""
    solid, loops = _outline_source()
    pts = solid[0]
    width, height = cam['width'], cam['height']
    scale, cx, cy, distance = cam['scale'], cam['cx'], cam['cy'], cam['distance']
    min_extent = OUTLINE_CELLS / (scale / distance)
    m0, m1, m2, m3, m4, m5, m6, m7, m8 = m
    top_heat = len(GLOW) - 1 + HOTTEST
    seen = {}
    masks = {}

    def project(v):
        got = seen.get(v)
        if got is None:
            x, y, z = pts[3 * v], pts[3 * v + 1], pts[3 * v + 2]
            tz = m6 * x + m7 * y + m8 * z
            w = 1.0 / (distance - tz)
            got = seen[v] = (cx + scale * w * (m0 * x + m1 * y + m2 * z),
                             cy - scale * 0.5 * w * (m3 * x + m4 * y
                                                     + m5 * z), w)
        return got

    # The edge is sampled at the matrix's own pitch - half a cell
    # across, a quarter down - and each sample sets one dot.
    for extent, members in loops:
        if extent < min_extent:
            continue
        for a, b in members:
            x0, y0, wa = project(a)
            x1, y1, wb = project(b)
            span = max(abs(x1 - x0), 2.0 * abs(y1 - y0))
            if span < OUTLINE_MIN_EDGE:
                continue                    # sub-pixel detail, see above
            steps = max(1, int(2.0 * span))
            for i in range(steps + 1):
                t = i / steps
                fx, fy = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
                px, py = int(fx), int(fy)
                if not (0 <= px < width and 0 <= py < height):
                    continue
                at = py * width + px
                near = buf[at]
                we = wa + (wb - wa) * t
                if near and we > 0.0 and 1.0 / we - 1.0 / near > OUTLINE_GRACE:
                    continue                          # behind the surface
                col = 1 if fx - px >= 0.5 else 0
                row = min(3, int((fy - py) * 4.0))
                masks[at] = masks.get(at, 0) | BRAILLE_BITS[col][row]
    # Strays: a line is a chain of neighbouring cells, so a cell with no
    # drawn neighbour in its eight is a sample that cleared the depth
    # test alone - a grazing edge, a corner half behind a wall - and
    # not a line. Dropped, on the bench's word ("stray pixels here and
    # there"); a median filter would have eaten the lines themselves,
    # which are one dot thick by design.
    lone = [at for at in masks
            if not any((at + dr * width + dc) in masks
                       for dr in (-1, 0, 1) for dc in (-1, 0, 1)
                       if (dr or dc)
                       and 0 <= at % width + dc < width
                       and 0 <= at // width + dr < height)]
    for at in lone:
        del masks[at]
    for at, mask in masks.items():
        r, c = divmod(at, width)
        grid[r][c] = chr(BRAILLE + mask)
        if colour:
            base = heat[at] if heat is not None and heat[at] else OUTLINE_BASE
            lift = OUTLINE_LIFT * (0.5 + 0.5 * base / top_heat)
            tone[r][c] = _blend(min(top_heat, base + lift))
    return len(masks)


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
        heat = [0.0] * (width * height) if colour else None
        _glow(grid, tone, classes, levels, bare, seed, coverage, width,
              height, colour, cam=cam, buf=buf, heat_out=heat)
        # The outline, last, over the shading: the parts' edges as a
        # wireframe overlay from the mesh's own creases - see
        # OUTLINE_DEG. Measured before any of this: the parts were tone
        # relief alone, a rung's worth, and the board read as one sheet.
        if not foreign:
            _outline(grid, tone, buf, cam, m, colour, heat=heat)

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
