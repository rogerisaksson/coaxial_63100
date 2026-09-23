"""The board's ASCII renderer: a depth-shaded raster with the parts drawn as
lines over it, and a wire fallback.
"""
import functools
import math
import os

from . import creases, engine, preload, solids, stereotype
from .. import ansi, orientation
from .creases import OUTLINE_EXACT
from .ground import GROUND_SPEED, _ground
from .lines import _edge, _outline
from .shading import (BIAS, FLOOR, LIGHT, PIVOT, SHADOW_DIM, SLOPE, SUN_MIN,
                      _dots, _expose, _face, _glow, _rim, _shadowmap)
from .solids import (LODS, MESHES_KEPT, _decimate_missing, _decimated,
                     _parametric, _slab_planes)
from .steady import _steady
from .triad import _triad

#: OUTER and BORE are the parametric board's own - one definition, and the
#: two drawings can never disagree about the hole. THICK is this module's
#: and deliberately twice the real slab: a vector drawing honest about a
#: 1.6 mm laminate reads as a single line, and the two rim rings are what
#: sells the turn.


THICK = 0.05


#: Degrees the camera tips from straight-down toward the horizon. Enough
#: for the ground plane to READ as a landscape; small enough that a tared
#: board is still a face-on circle to the eye.
CAMERA_TIP = 34.0


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

    edges = []
    for z in (THICK, -THICK):
        edges += [(a, b, 'board') for a, b in _ring(orientation.OUTER, z)]
    for i in range(RIM_STRUTS):
        phi = 2 * math.pi * i / RIM_STRUTS
        outer = orientation.OUTER
        a = (outer * math.cos(phi), outer * math.sin(phi), THICK)
        b = (outer * math.cos(phi), outer * math.sin(phi), -THICK)
        edges.append((a, b, 'board'))
    edges += [(a, b, 'board') for a, b in _ring(orientation.BORE, THICK, 18)]

    for part in orientation.COMPONENTS:
        zone = part[0]
        edges += [(a, b, zone) for a, b in _cage(*part[1:])]
    return edges


@functools.cache
def _edges():
    """The wire model, built once a process."""
    return _build()


def _lods(progress=None):
    """Every level of detail's solid, coarse to fine - what a crew holds,
    since the zoom picks among them by identity.
    """
    path = orientation.MODEL
    stamp = os.path.getmtime(path)
    missing = [d for _z, d in LODS if (path, d, stamp) not in solids._MESHES]
    if missing:
        bundle = preload.load(path)
        if bundle is not None:
            _adopt(path, bundle)
            missing = [d for _z, d in LODS if (path, d, stamp) not in solids._MESHES]
    if len(missing) > 1:
        _decimate_missing(path, stamp, missing, progress)
    return [_decimated(path, divisions) for _z, divisions in LODS]


def _forget():
    """Every cache of the model at once - the solids, the outline's
    loops, the pre-scan's primitives: a re-exported STL changes every
    stamp, and a freed solid's id can come round again on a new one."""
    solids._forget()
    creases._OUTLINES.clear()
    stereotype._STEREO.clear()


def _adopt(path, bundle):
    """The preload's decimates, exact index, loops and stereotypes into this
    process's caches, under the keys the builders would have used, so
    every later ask finds them built.
    """
    stamp = os.path.getmtime(path)
    if len(solids._MESHES) + len(bundle['lods']) + 1 > MESHES_KEPT:
        _forget()
    for divisions, solid in bundle['lods'].items():
        solids._MESHES[(path, divisions, stamp)] = solid
    exact, loops = bundle['exact']
    solids._MESHES[(path, OUTLINE_EXACT, stamp)] = exact
    creases._OUTLINES[id(exact)] = (exact, loops)
    stereotype._STEREO[id(loops)] = bundle['prims']
    return len(bundle['lods']), len(loops), len(bundle['prims'])


#: The coarsest grid a view with a crew draws. Grid 16 is a polygon with
#: a notch in its rim and a hole in its face at the attitude view's
#: size - decimation artefacts, seen; 32 looked clean face-on, but seen
#: steeply its face around the bore was gaps - the bore wall's and the
#: collar's triangles collapse on 0.0625 cells against a bore 0.2
#: across, and the ground showed through beside the ring the outline
#: drew where the hole is (the bench's "ghost hole", 2026-09-23).
#: Grid 48 keeps 5 645 triangles against 32's 5 570 and closes them;
#: measured in the attitude page's own loop, 31.4 -> 32.3 ms a frame.
CREW_LEAST = 48


#: The mesh a board of so many CELLS across earns: (cells below, grid
#: divisions). BY SIZE ON SCREEN, not by zoom alone - at zoom 0.88 the
#: attitude page drew a 150-column window from the 12-division mesh,
#: the coarsest, and the parts came out as chunky boxes; the zoom bands
#: below still apply, so a zoomed board stays finer than its size alone
#: would earn. Measured single-process at 150x44: 12 divisions 35 ms a
#: frame, 32 69 ms, 64 98 ms.
SIZE_LODS = ((30, 12), (45, 16), (60, 24), (80, 32), (110, 48), (None, 64))


def _earned(width, height, zoom):
    """The least divisions a frame this size earns: the board spans
    about four fifths of the frame's shorter side, in cells, times the
    zoom - a row being two cells tall."""
    cells = 0.8 * min(width, 2 * height) * zoom
    return next(d for upto, d in SIZE_LODS if upto is None or cells < upto)


def _model(zoom=1.0, least=0):
    """(edges, solid) - the solid at the LEVEL OF DETAIL the zoom earns, and
    never coarser than `least` divisions.
    """
    divisions = next(d for upto, d in LODS
                     if d >= least and (upto is None or zoom < upto))
    try:
        solid = _decimated(orientation.MODEL, divisions)
    except (OSError, ValueError):
        solid = _parametric()
    return _edges(), solid


#: How many frames a NEW pose is drawn in full before the face is held
#: from `persist` - the exposure glides toward a frame's percentiles
#: at EXPOSE_FOLLOW a frame, and eight frames put it within six
#: percent of where it is going. After that, while the pose holds,
#: the face is replayed and only the ground is drawn: at rest under
#: the view's deadband the pose IS held, and that is most of the time
#: a board spends on a bench. Measured at 108x40: a frame goes from
#: 52 ms (eight workers rastering, the parent shading) to the ground
#: and the rows alone.
FACE_SETTLE = 8


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
    """(depth, top, sun, coverage, reached) at cell resolution: rastered at
    the braille dots - 2x4 subsamples per cell, `engine.fine` - and
    FOLDED, so every cell knows its coverage and which of its dots the
    model reaches, which the rim's clipping runs on.
    """
    fine = engine.fine(cam)
    if crew is not None and crew.holds(solid):
        buf, topf, sun = crew.raster(solid, m, fine, beam=LIGHT,
                                     sun_min=SUN_MIN)
    else:
        buf, topf, sun = engine.raster(solid, m, fine, beam=LIGHT,
                                       sun_min=SUN_MIN)
    return engine.fold(buf, topf, sun, cam['width'], cam['height'])


def _cells(solid, m, cam, crew, face, foreign):
    """Every per-cell stage: (depth, coverage, reached, classes, levels,
    bare, seed).
    """
    width, height = cam['width'], cam['height']
    planes = None if foreign else _slab_planes(solid)
    if face and crew is not None and crew.holds(solid):
        shading = (PIVOT, SLOPE, FLOOR, None if foreign else _shadowmap(m),
                   SHADOW_DIM, BIAS, not foreign, planes)
        return crew.frame(solid, m, cam, LIGHT, SUN_MIN, shading)
    buf, topf, sun, coverage, reached = _raster(solid, m, cam, crew)
    if not face:
        return buf, coverage, reached, None, None, None, None
    levels = [0.0] * (width * height)
    bare = [0.0] * (width * height)
    seed = [0.0] * (width * height)
    classes = engine.shade(
        buf, topf, sun, cam, m, PIVOT, SLOPE, FLOOR,
        art=None if foreign else _face(),
        shadow=None if foreign else _shadowmap(m),
        shadow_step=SHADOW_DIM, bias=BIAS, levels=levels, bare=bare,
        seed=seed, planes=planes)
    return buf, coverage, reached, classes, levels, bare, seed


def _paint(grid, tone, cells, cam, m, colour, persist, foreign):
    """The face: glow, halftone, rim, outline - in that order, each over the
    last.
    """
    buf, coverage, reached, classes, levels, bare, seed = cells
    width, height = cam['width'], cam['height']
    heat = [0.0] * (width * height) if colour else None
    _glow(grid, tone, classes, levels, bare, seed, coverage, width,
          height, colour, cam=cam, buf=buf, heat_out=heat)
    if colour:
        _dots(grid, heat, classes, coverage, width, height,
              _expose(heat, classes, persist), reached)
    _rim(grid, tone, classes, reached, heat, width, height, colour)
    # The outline, last of the ink, over the shading: the parts' edges as a
    # wireframe overlay from the mesh's own creases - see OUTLINE_DEG.
    if not foreign:
        _outline(grid, tone, buf, cam, m, colour, heat=heat)
        _edge(grid, tone, cells, cam, colour, heat=heat)


def _painted(solid, m, cam, cells, colour, persist, foreign):
    """`(buf, layer)`: the face painted from `cells` onto a blank grid -
    `(row, col, glyph, tone)` for every cell it marks."""
    width, height = cam['width'], cam['height']
    grid = [[' '] * width for _ in range(height)]
    tone = [[None] * width for _ in range(height)]
    _paint(grid, tone, cells, cam, m, colour, persist, foreign)
    layer = [(r, c, grid[r][c], tone[r][c])
             for r in range(height) for c in range(width)
             if grid[r][c] != ' ']
    return cells[0], layer


def _hold(persist, key, m, cells, buf, layer, settles):
    if persist is not None:
        persist['face'] = {'key': key, 'm': m, 'raw': cells, 'buf': buf,
                           'cells': layer, 'settles': settles}


def _face_layer(solid, m, cam, crew, colour, persist, foreign, key):
    """`(buf, cells)`: the depth buffer and the face - `(row, col, glyph,
    tone)` for every cell it paints - HELD in `persist` under `key` and
    replayed while the pose is the one held, else drawn and held.
    """
    held = persist.get('face') if persist is not None else None
    if held is not None and held['key'] != key:
        held = None                           # another pose: start over
    if held is not None and held['settles'] >= FACE_SETTLE:
        return held['buf'], held['cells']
    settles = held['settles'] + 1 if held is not None else 0
    cells = (held['raw'] if held is not None
             else _cells(solid, m, cam, crew, True, foreign))
    buf, layer = _painted(solid, m, cam, cells, colour, persist, foreign)
    _hold(persist, key, m, cells, buf, layer, settles)
    return buf, layer


#: The pose's place in a face key; everything else in it is the framing.
POSE = 10


def _face_ahead(solid, m, cam, crew, colour, persist, foreign, key):
    """`(key, m, buf, layer)` of the pose to PAINT - one pose behind the one
    asked for while the board moves.
    """
    held = persist.get('face')
    flight = persist.get('flight')
    framing = key[:POSE] + key[POSE + 1:]

    def same_framing(other):
        return other[:POSE] + other[POSE + 1:] == framing

    def send(pose_key, pose_m):
        if crew.pending:                     # a persist reset mid-flight
            crew.collect()
        crew.submit(solid, pose_m, cam, LIGHT, SUN_MIN,
                    (PIVOT, SLOPE, FLOOR,
                     None if foreign else _shadowmap(pose_m), SHADOW_DIM,
                     BIAS, not foreign,
                     None if foreign else _slab_planes(solid)))
        persist['flight'] = {'key': pose_key, 'm': pose_m}

    def paint(pose_key, pose_m, cells, settles):
        buf, layer = _painted(solid, pose_m, cam, cells, colour, persist,
                              foreign)
        _hold(persist, pose_key, pose_m, cells, buf, layer, settles)
        return pose_key, pose_m, buf, layer

    if held is not None and held['key'] == key:
        if flight is not None:                # came and went unpainted
            crew.collect()
            persist['flight'] = None
        if held['settles'] >= FACE_SETTLE:
            return key, held['m'], held['buf'], held['cells']
        return paint(key, held['m'], held['raw'], held['settles'] + 1)
    if flight is not None:
        cells = crew.collect()
        persist['flight'] = None
        if flight['key'] == key:              # rested on the pose in flight
            return paint(key, m, cells, 0)
        if same_framing(flight['key']):
            send(key, m)
            return paint(flight['key'], flight['m'], cells, 0)
    send(key, m)
    if held is not None and same_framing(held['key']):
        return held['key'], held['m'], held['buf'], held['cells']
    cells = crew.collect()
    persist['flight'] = None
    return paint(key, m, cells, 0)


def _face_of(solid, q, m, cam, crew, colour, persist, foreign, ahead,
             framing):
    """`(m, buf, layer)` for the pose `q` under `framing` - everything else
    the face depends on, so a change in any of it is a new drawing and a
    pose the deadband holds is not.
    """
    key = framing + (tuple(round(v, 6) for v in q),
                     id(solid) if foreign else None)
    if ahead and persist is not None and crew is not None \
            and crew.holds(solid):
        _key, m, buf, layer = _face_ahead(solid, m, cam, crew, colour,
                                          persist, foreign, key)
        return m, buf, layer
    buf, layer = _face_layer(solid, m, cam, crew, colour, persist,
                             foreign, key)
    return m, buf, layer


def render(q, width, height, zoom=1.0, colour=True,
           horizon=True, face=True, tip=None, solid=None,
           distance=None, lift=0.44, crew=None, least=0, triad=False,
           persist=None, scroll=None, ahead=False):
    """The board under rotation `q`, as a vector drawing."""

    # `solid` overrides the board with another mesh - facecheck proves the
    # LIGHT MODEL on the exporter's cube, whose flat faces turn a shading bug
    # into a wrong character instead of a vibe.
    edges, board_solid = _model(
        zoom, max(least, _earned(width, height, zoom),
                  CREW_LEAST if crew is not None else 0))
    foreign = solid is not None
    if not foreign:
        solid = board_solid
    # A NEAR-TOP camera, tipped just enough that the world grid recedes to a
    # horizon the way the old flight sims drew it.
    distance = 3.2 if distance is None else distance
    # The fit is a BOUNDING SPHERE, deliberately NOT the frame at hand.
    pts = solid[0]
    reach = max(math.sqrt(pts[3 * i] ** 2 + pts[3 * i + 1] ** 2
                          + pts[3 * i + 2] ** 2)
                for i in range(len(pts) // 3))
    # The camera is tipped OUTSIDE the attitude, so screen X and Y map
    # one-to-one onto board X and Y; cy sits a shade above centre so the ground
    # fills the lower field.
    cam = engine.camera(width, height, reach, distance=distance,
                        zoom=zoom, tip=CAMERA_TIP if tip is None else tip,
                        lift=lift)
    view = cam['view']
    m = engine.multiply(view, orientation.matrix(q))
    scale, cx, cy = cam['scale'], cam['cx'], cam['cy']

    if face:
        # `m` comes back as the PAINTED pose's - one behind, ahead - so the
        # triad tilts with what is drawn.
        m, buf, layer = _face_of(
            solid, q, m, cam, crew, colour, persist, foreign, ahead,
            (width, height, zoom, colour, foreign, lift, distance, tip,
             least, crew is not None))
    else:
        buf, layer = _cells(solid, m, cam, crew, False, foreign)[0], ()

    grid = [[' '] * width for _ in range(height)]
    tone = [[None] * width for _ in range(height)]

    if horizon:
        phase = (scroll * GROUND_SPEED) % 1.0 if scroll is not None else 0.0
        _ground(grid, tone, buf, distance, width, height, colour, view,
                phase)
    for r, c, glyph, ink in layer:
        grid[r][c] = glyph
        tone[r][c] = ink
    if triad:
        _triad(grid, tone, cam, m, colour)
    if persist is not None:
        _steady(grid, tone, width, height, persist)

    z_lo = min((v for v in buf if v), default=0.0)
    z_hi = max(buf, default=1.0) or 1.0
    span = (z_hi - z_lo) or 1.0
    m0, m1, m2, m3, m4, m5, m6, m7, m8 = m
    # The lit raster IS the picture; the chosen edges only draw in wire mode.
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
            # The solid decides visibility; the 2 % grace keeps an edge from
            # losing to the very face it borders.
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
