"""The face's light: the depth ramp and its constants, the tone ladder,
the desk lamp, the key light, the halftone's dots, the rim's line glyph,
the board's face art, the cast-shadow map. `_glow`, `_dots`, `_expose`
and `_rim` are the passes `wireframe._paint` runs in order; `_edge_tone`
is the tone every drawn line takes. Every constant here was fitted or
measured, and says so."""
import functools
import math
import os

from ..ansi import rgb as _rgb
from .raster import BRAILLE, BRAILLE_BITS, NOISE, NOISE_N, RUNGS, SHADE
from .solids import _casters


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
LIT = SHADE


#: What each class is worth on the ladder in a MONO render, where there
#: is no light to ask and the class is the whole signal. Blank, sparse,
#: and most of the way up.
#:
#: NEAR THE EXPORTER'S OWN DENSITIES. His ' ', '.' and ':' measure 0, 96
#: and 129 of 255 in Rec.709 luma, which on nine rungs is nothing, three
#: dots and four. Six against two was tried to spend the ladder and it
#: read as a slab: the board's top at 3.6 dots a cell where the exporter
#: draws two, "blocky" on the bench. Four against two keeps his ordering
#: and his weight - 2.8 dots a cell, measured - and the unrounded level
#: grades between them; the rungs above are for the light, in the colour
#: pass, where a surface actually turns away.
CLASS_RUNG = (0, 2, 4)


#: The band of level the ladder spans WHERE THERE IS NO LIGHT TO ASK.
#: The class scale is 0 to 2 by construction, and rung 0 is blank, so a
#: drawn cell starts at rung 1 and the seven above it carry the depth
#: that used to fall through a single step. Only the mono path uses it:
#: with colour on, the glyph comes off `heat`, which is the same number
#: the tone does - see `_glow`.
LEVEL_LO, LEVEL_HI = 0.0, 2.0


#: The band of `heat` the ladder spans, as a share of DIMMEST to the top
#: of the glow. NOT THE WHOLE OF IT: the lamp puts most of a board inside
#: a narrow band and stretching the ladder over the full range left every
#: interior cell on the same rung - the picture went solid. This is the
#: window the shading actually moves in, and what falls outside pins.
#: FITTED on the shipped board at zoom 1: 368 lit cells span heat 0.92
#: to 4.70 with the fifth and ninety-fifth percentiles at 1.87 and
#: 3.83, which as a share of DIMMEST to the top of the glow is 0.22 to
#: 0.52. What falls outside pins, which is what a percentile band is
#: for.
HEAT_LO, HEAT_HI = 0.20, 0.55


#: THE FACE IS A HALFTONE, at dot resolution. Each of a cell's eight
#: dots is lit where the light at the dot's own position clears its
#: threshold in a BLUE-NOISE mask laid over the DOTS - `raster.NOISE`,
#: 64 by 64, thirty-two cells wide and sixteen tall, four thousand
#: densities - fixed in screen space, so a board turning under it moves
#: the density and not the dots. The thermal picture draws through the
#: same mask.
#:
#: Three things before it, each seen in a raster of the frame rather
#: than in glyph counts. One rung per cell rounded a desk-lamp gradient
#: of a fraction of a rung a cell to the same rung across the face - a
#: carpet of `⢕` with the parts drawn on it. Sampling the light per dot
#: on the ladder's own nesting order fixed the carpet and left a sparse
#: lattice - eight dot positions in one fixed order, rows of dots a cell
#: apart on the dark half. An 8 x 8 Bayer matrix fixed the lattice and
#: at a real window (150 x 44) showed its own hierarchy: two-by-two
#: clusters that read as small square blocks across the board, "blocky
#: as hell". Interleaved gradient noise and the R2 sequence were
#: rastered beside it - a regular diagonal screen, and a half-structured
#: one. Blue noise has no structure at any density; the generator has
#: the method and the file beside `raster` is its output.

#: Where each of a cell's eight dots sits, in cells from the cell's
#: centre - two lanes a quarter cell either side, four rows at eighths
#: down - and which bit it is.
DOT_AT = tuple((lane * 0.5 - 0.25, (y - 1.5) / 4.0, BRAILLE_BITS[lane][y])
               for lane in range(2) for y in range(4))


#: The darkest a lit cell's density goes, as a share of its dots. THE
#: BRIGHTNESS WAS ENCODED TWICE: a dark cell was few dots AND a dim
#: tone, contrast squared, and the half of the board the lamp reaches
#: least dissolved into speckle. With the tone carrying the light, the
#: dots need only carry the shape - a floor of three dots in ten keeps
#: the face a surface at its darkest. Measured in a raster: at 0 the
#: dark side was scattered dots, at 0.5 the shading flattened; 0.3
#: keeps the gradient and the surface both.
#:
#: AND THEN DOWN TO 0.12, ON THE BENCH'S SCREENSHOT. In the terminal the
#: braille glyph box is narrower than the character cell - the fallback
#: font's, not this drawing's - so at three dots in ten and up every
#: cell is a brick with dark mortar round it, and the cell grid itself
#: is the block: "extremely blocky", with the picture attached. The
#: pattern of the dots was beside the point; only the DENSITY decides
#: whether a cell reads as a few points on a coloured field or as a
#: brick. Rastered at 120x40, zoom 1.5: 0.30 to 0.85 the brick wall,
#: 0.15 to 0.50 a stippled surface, 0.08 to 0.35 a light dusting with
#: the shape in the rim and the outlines. The tone carries the light;
#: the dots carry the shape, and few of them do it best.
#:
#: AND UP AGAIN TO 0.42 ONCE THE FACE WAS SCANLINES: "still a bit
#: pixelly", and the pixels were the breaks in the lines. With the tone
#: carrying the light, the lines can run nearly whole - 0.42 is 84 % of
#: a lit row, so only the darkest of the board keeps a gap here and
#: there, and the ceiling's 0.5 is a whole row. Rastered at 0.35, 0.42
#: and 0.5 at the bench's framing: the whole lines were the smoothest,
#: and 0.42 keeps a trace of shading in the dots on top of the tone's.
DENSITY_FLOOR = 0.42


#: And the brightest, well short of every dot, for the same reason: a
#: raised part lit squarely and its relief ignited saturated into a
#: solid bright rectangle. Under half its dots the brightest face is a
#: denser stipple, not a block; the rim and the outline draw solid on
#: their own terms, so an edge is still a line.
DENSITY_CEIL = 0.5


#: THE FACE IS SCANLINES. Of a cell's four dot rows only these light,
#: and the density rides along them doubled, so a cell carries the same
#: ink in half its rows. "A shade pixelly" was the stipple's last word
#: on the bench: scattered single points on black, however evenly
#: spread, are grain. Confined to alternate rows the same dots join into
#: fine broken horizontal lines that close up in the highlights - the
#: coherent structure a stipple has none of, and the retro terminal's
#: own - and with two rows of four always dark no cell can fill, so the
#: bricks cannot come back. Rastered beside the stipple and a per-cell
#: cap at the bench's framing; the lines were the smooth one.
SCAN_ROWS = (0, 2)


#: The dots `_dots` samples, worked out once: offset, bit, the lane and
#: the row in the cell - only the SCAN_ROWS, so the loop over a cell is
#: four dots and no arithmetic about which four. The bit is also what
#: the clipping tests: the fold's `reached` mask is in the glyph's own
#: bit order. Measured: the per-dot `int()` and membership test on all
#: eight were a third of the pass.
SCAN_DOTS = tuple(
    (ox, oy, bit, 0 if ox < 0.0 else 1, int((oy + 0.5) * 4.0))
    for ox, oy, bit in DOT_AT if int((oy + 0.5) * 4.0) in SCAN_ROWS)


#: The exposure: which percentiles of the frame's lit heat land at the
#: ladder's ends, and how fast the window follows from frame to frame.
#: PER FRAME, because a fixed window was fitted on one frame - 368 cells
#: at zoom 1 in one pose - and at the page's own size and a tilt of 30
#: degrees 316 of 453 lit cells sat on rung 1 with rungs 6 to 8 empty,
#: measured. The window is never narrower than the fitted one's share
#: of the ladder (`HEAT_HI - HEAT_LO`): a frame with little range keeps
#: the calibrated spread rather than stretching its grain over the
#: ladder, and a frame with more spends all of it. Followed at a third
#: a frame so a turning board's exposure glides rather than snaps.
EXPOSE = (0.05, 0.95)
EXPOSE_FOLLOW = 0.3
EXPOSE_LEAST = 16


#: Below this much heat over the floor the tone rolls off toward
#: DIMMEST instead of stopping on it. The hard floor pinned half of a
#: board turned 45 degrees from the beam at exactly DIMMEST (p50 0.40,
#: measured): one tone and, exposed, one rung - the dark face was a
#: slab. Rolled off, every cell under the knee keeps its order, so the
#: dots can still grade a face the lamp barely reaches. Nothing at or
#: above the knee moves: the calibrated middle stands.
KNEE = 1.0


#: ONE hue, the console theme's cyan, as a pure luminance ladder:
#: black through the teals to white-cyan for the sharpest highlight.
#: The object never changes colour - only how much light its
#: characters carry. Plain grey was tried and read as a dead channel
#: next to the stage's phosphor.
GLOW = (16, 23, 30, 37, 44, 51, 87, 123, 195)


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
    """The ladder colour at a fractional heat, linearly between rungs;
    the top rung itself is the ladder's last colour."""
    lo = min(int(heat), len(GLOW_RGB) - 2)
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
        # The row terms hoisted as in engine.raster: the same floats,
        # less work per pixel.
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
#:
#: 4.5, from 2.5, MEASURED: in Rec.709 luma at the view's size over
#: four attitudes, a 2.5 lift put the line +51 over the face beside it
#: (174 against 122, a tenth of the cells within +5) and the bench read
#: it as "barely noticeable" - a braille dot carries less ink than the
#: face's `.` and `:`, so a line needs more tone than a face to look
#: brighter at all. At 4.5 the line sits +82 (205 against 124, the
#: tenth at +38), and it may reach the ladder's top rung, which HOTTEST
#: keeps from the face: in the lamp's pool an edge glints white-cyan.
#:
#: 3.0 SINCE THE LINE LIES ON THE FACE'S DOTS (2026-09-23): the 4.5
#: was set while the line's one or two dots stood alone in their cell
#: and needed tone to be seen at all; merged, the cell carries five or
#: six dots, and at 4.5 it read as a glowing band round every part -
#: "a lot of halo in the edges". Measured on the raster at 30 and 45
#: degrees over 1.5, 2.5, 3.5 and 4.5: 1.5 vanishes into the dither,
#: 4.5 glows; 3.0 is a denser, brighter run of the same dither.
OUTLINE_LIFT = 3.0
OUTLINE_BASE = 3.0


def _mono(level, phase=0.0):
    """One cell from the CLASS scale alone, for a render with no colour.

    `level` is the unrounded 0 to 2, so the rung is `CLASS_RUNG`
    interpolated: a cell three quarters of the way from one class to the
    next sits three quarters of the way up the rungs between them.
    """
    low = 0 if level < 1.0 else 1
    step = level - low
    step = 0.0 if step < 0.0 else (1.0 if step > 1.0 else step)
    rung = CLASS_RUNG[low] + step * (CLASS_RUNG[low + 1] - CLASS_RUNG[low])
    return _pattern(int(rung + 0.5), phase)


def _pattern(rung, phase):
    """The glyph for `rung` dots, phased. A DRAWN CELL IS NEVER BLANK:
    the class already said there is something here, so the darkest a lit
    cell goes is one dot - rung 0 would punch holes in a face the
    geometry calls solid."""
    rung = 1 if rung < 1 else (RUNGS if rung > RUNGS else rung)
    # THE SMOOTHEST PATTERN OF THE RUNG, EVERY TIME. A per-cell grain
    # picked among the 28 arrangements that carry six dots, first
    # uniformly and then cubed toward the even end, and either way a
    # flat face wore a different pattern in every cell: 107 distinct
    # glyphs on the board's top at a single pose, against 79 with the
    # grain off - and the 79 are real edges. The bench called it blocky,
    # which is what a texture with no structure looks like. A flat
    # surface is a flat pattern; the block is spent where the LEVEL
    # changes, which is what the nine rungs are for. `phase` is kept in
    # the signature so a caller that computed one need not stop.
    return LIT[rung][0]


def _floor(heat):
    """The tone's floor as a roll-off: at or above `DIMMEST + KNEE`
    the heat is its own, below it decays toward DIMMEST and never
    reaches it, continuous in value and slope at the knee."""
    if heat >= DIMMEST + KNEE:
        return heat
    return DIMMEST + KNEE * math.exp((heat - DIMMEST - KNEE) / KNEE)


def _expose(heat, classes, persist=None):
    """The heat window the ladder spans this frame: `(lo, hi)`, lo at
    rung 0 and hi at the top rung. The frame's own percentiles, widened
    to at least the fitted window's share of the ladder, and followed
    from the previous frame through `persist` when there is one."""
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
    """The key light on one cell: central differences of bare geometry
    where both neighbours are covered, else the face-on rest, so a
    silhouette cell neither flares nor drops. The direction to the lamp
    is from THIS cell's point, back out of the projection as engine.shade
    does; without a depth buffer, the beam's direction."""
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
    where the density at the dot clears its Bayer threshold.

    The heat at a dot is the cell's own plus its gradient - `_slope`
    from the lit neighbours - times the dot's offset, normalised by
    `window` to a share of the ladder and lifted onto DENSITY_FLOOR.
    The threshold is the dot's rank in NOISE, by the dot's position on
    the SCREEN, so the mask tiles across cells and a flat region is one
    even, structureless field at its density. A DRAWN CELL IS NEVER
    BLANK: the
    class already decided there is something here, and a cell whose
    density clears no dot keeps one.

    CLIPPED TO THE MODEL: `reached` says which of a cell's eight dots
    the fine raster reached, in the glyph's own bit order, and a dot it
    missed stays dark - that dot is outside the board. Before this a
    rim cell a quarter covered drew its whole cell's density thinned by
    two rungs, and the dots spilled past the rim; the bench asked for
    them cut. With the clipping the thinning has nothing left to do
    and is gone. It clipped by quadrant first - one dot wide, two
    tall - and along a shallow rim the face ended two rows short or
    two rows long of the silhouette, cell by cell: the stair the bench
    saw as jagged."""
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
        # The mask's rows for this cell row, looked up once per row
        # rather than once per dot.
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
    """The tone an edge takes - a part's outline, the rim - off the
    heat under it, lifted by OUTLINE_LIFT toward the ladder's top rung,
    which HOTTEST keeps from the face: readable ink is what that rung
    is reserved for."""
    top_heat = len(GLOW) - 1 + HOTTEST
    return _blend(min(len(GLOW) - 1,
                      base + OUTLINE_LIFT * (0.5 + 0.5 * base / top_heat)))


def _edge_glyphs():
    """The braille LINE for every way a cell can be part-covered: of the
    dots the model reaches, those beside a dot it misses - across the
    lane, or the row above or below, inside the cell. A cell reached on
    its right half draws `⢸`, on its bottom two rows `⠤`, in one corner
    a stub, on a slant a stair of single dots - so along a rim the
    cells join into a drawn line one dot wide that follows the
    silhouette, and the face inside it stays a stipple. Two vocabularies
    on one board, which is what the bench asked for: "the highlight and
    the edges in another braille character". The table was sixteen
    quadrant masks first, the line on two of a cell's four rows only;
    at dot resolution it is one of 256 and sits on the row the edge
    crosses."""
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
    """THE EDGE, ENHANCED. A cell the model covers only in part is on a
    silhouette - the board's rim, a hole's edge - and is drawn as the
    braille LINE along that edge (`EDGE_GLYPH`), in the outline's tone:
    a hard bright line round every shape, over the stipple inside it.
    It was solid in the reached quadrants first, and solid cells beside
    a stipple are the very blocks the bench objected to; a line glyph
    is a different character from a fill, and reads as one.

    The bench asked for the retro-futurist terminal - Nostromo, Blade
    Runner - and that look is edges: a face is a texture, an edge is a
    line of light. Before this the rim was the halftone thinned by
    coverage, a soft edge with dots past the board wherever a cell was
    a quarter covered. PART-COVERED CELLS ONLY: a whole cell beside an
    uncovered one counted as an edge too, and where the parts crowd
    the board's far edge every cell has such a neighbour - the region
    came out as solid bright blobs, blocks by another route, in the
    bench's screenshot. The part-covered cells are the silhouette at
    dot resolution, and that is thin enough to be a line. The mono
    render takes the solid glyph and no tone: there the edge is the
    dots alone."""
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
            # The line OVER the face's own dots in the cell, not instead
            # of them: replaced, the fill stopped a whole cell short of
            # the silhouette wherever the rim crossed one, and the
            # face's edge stepped by cells behind a line that did not -
            # the stair the bench saw. Masked to the reach, since
            # `_dots` keeps one dot in a cell it lit nothing in.
            face = ord(grid[py][px]) - BRAILLE
            if not 0 <= face <= 0xFF:
                face = 0
            grid[py][px] = chr(BRAILLE + ((face & reach) | EDGE_GLYPH[reach]))
            if colour:
                base = (heat[at] if heat is not None and heat[at]
                        else OUTLINE_BASE)
                tone[py][px] = _edge_tone(base)


def _keylight(cam, width, height):
    """What the key light needs, once per frame, or `key` None with it off.

    OUT OF `_glow` BECAUSE IT IS SET-UP, not shading: the loop under it
    is the picture and this is the six numbers it reads. Answers
    `(key, lamp, colf, rowf, distance, scale)`.

    View z per unit of `bare`, and the cell's size in view units: one
    column is (distance - z)/scale across, one row twice that down. Both
    at the board's own depth - the gradient is a slope, and a per-cell
    depth in the divisor moved the answer by under 1 %.
    """
    if cam is None or not KEY:
        return None, None, None, None, None, None
    per_bare = cam.get('reach', 1.0) / SLOPE
    distance, scale = cam['distance'], cam['scale']
    cx, cy = cam['cx'], cam['cy']
    across = distance / scale
    lx, ly, lz = LIGHT
    # The projection's per-column and per-row factors, once: the inner
    # loop multiplies, it does not divide.
    return ((0.5 * per_bare / across, 0.5 * per_bare / (2.0 * across)),
            (lx * KEY_DISTANCE, ly * KEY_DISTANCE, lz * KEY_DISTANCE),
            [(c + 0.5 - cx) / scale for c in range(width)],
            [(cy - (r + 0.5)) / (scale * 0.5) for r in range(height)],
            distance, scale)


def _glow(grid, tone, classes, levels, bare, seed, coverage, width, height,
          colour, cam=None, buf=None, heat_out=None):
    """Classes to glyphs, unrounded levels to the colour ramp.

    The level spans PIVOT +- SLOPE by construction (view-z over reach
    is +-1), normalised and bent through the EDGE sigmoid: shadows
    deepen and highlights sharpen while the midtone stands. The SPOT
    adds its radial pool of light and the KEY light shades by the
    surface's slope; the rim is `_rim`'s, a line of light clipped to
    the model over the halftone's edge, and nothing dims - the
    exporter's rim is as bright as his interior. `cam` sizes the cells
    for the key light; without it the
    key is off (the tests that hold the pool's arithmetic pass none)."""
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
            # THE GLYPH CARRIES THE LEVEL, not the rounded class. Three
            # characters gave a leaning face one step to fall through and
            # a board came out flat; nine rungs of dots grade it. The
            # PHASE is the cell's own grain hash where there is one, so
            # two cells at one level wear different dots and a shallow
            # gradient does not band.
            # THE GLYPH IS THE LIGHT WHERE THERE IS LIGHT. With colour
            # on it is overwritten below from `heat` - the same number
            # the tone is blended from - so the lamp, the key and the
            # feather show in the dots as well as the hue. Here it is the
            # depth alone, which is all a mono render has.
            hashed = seed[at] if seed is not None else 0.0
            level = levels[at] if levels is not None else float(cls)
            grid[py][px] = _mono(level, hashed)
            if not colour:
                continue
            nx = (px + 0.5) / width - spot_x
            # The desk-lamp pool: squared falloff lands at ZERO slope
            # on the rim. Linear-in-d2 ended at its steepest - a
            # terminator ring drawn across the board.
            pool = 1.0 - (nx * nx + ny * ny) / rr
            t = (level - lo) / span - DUSK
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
                    + grain * (hashed - 0.5))
            # The key light: central differences of bare geometry where
            # both neighbours are covered, else the face-on rest, so a
            # silhouette cell neither flares nor drops.
            if (key is not None and lamp is not None and colf is not None
                    and rowf is not None and distance is not None):
                heat += KEY * (_key_lit(px, py, at, width, height, classes,
                                        bare, key, lamp, colf, rowf,
                                        distance, buf) - KEY_REST)
            # Anti-aliasing is the GLYPH only: a staircase corner (two
            # or more empty neighbours) thins ':' to '.'. The tone once
            # feathered too, and that drew the LOD line: every cell on
            # the silhouette and round every hole sat at 55-62 luma
            # against 101-103 inside, a dark contour the exporter's
            # screenshots do not have (his rim 99, his interior 101).
            # Anti-aliasing by COVERAGE, from the dot fold: a rim cell
            # dims by the share of it the model misses and thins its
            # glyph at half or less. A flat feather on every rim cell
            # drew a dark contour round the board and every hole (55
            # luma against 101 inside); counting empty neighbours could
            # not tell a straight edge from a stair and left it raw.
            missed = 1.0 - coverage[at]
            if missed:
                heat -= FEATHER * missed
            heat = top_heat if heat > top_heat else _floor(heat)
            if heat_out is not None:
                heat_out[at] = heat
            tone[py][px] = _blend(heat)
            # AND THE GLYPH OFF THE SAME NUMBER, in `_dots` once every
            # cell's heat is known: an ASCII render carries its 3D in the
            # characters, and the dots are sampled from the heat FIELD -
            # the neighbours' heat as well as this cell's - which this
            # loop, one cell at a time, has not got yet.
