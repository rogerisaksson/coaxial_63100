"""The board's attitude, drawn in characters."""
import functools
import math
import os
import random

from coaxial.draw import ascii3d
from coaxial.graphics import mesh
from machine import ansi

#: The board, in units of its own outer radius: 100 mm across, a 10 mm bore
#: through the middle, 1.6 mm thick. Proportions, not a measurement - the
#: drawing is a shape, and invariant 10 applies to it too.
OUTER = 1.0
BORE = 0.10
HALF_THICKNESS = 0.016

#: Perspective: K2 pushes the torus away from the eye and K1 scales what is
#: left onto the canvas. K1 is derived from the width, so a wider terminal
#: draws a bigger donut rather than the same one with more blank around it.
K2 = 5.0

#: Samples around the board. Fine enough that the surface has no gaps at this
#: character resolution; every sample is rotated, projected and z-tested.
PHI_STEPS = 260

#: Dimmest to brightest - a1k0n's ramp, twelve levels of how squarely a
#: surface faces the light. Bare copper; the parts have their own, below.
SHADES = '.,-~:;=!*#$@'

#: Where the light is, in the viewer's frame: over the reader's left shoulder
#: and towards the reader, so the face turned this way is the lit one.
LIGHT = (0.0, 1.0, 1.0)

#: Frames of the liveness spinner. A picture of a still board and a frozen
#: one look identical without something that moves.
SPINNER = ('|', '/', '-', '\\')


def rotate(q, v):
    """`v` turned by the unit quaternion `q`, given as (i, j, k, real)."""
    i, j, k, w = q
    x, y, z = v

    tx = 2.0 * (j * z - k * y)
    ty = 2.0 * (k * x - i * z)
    tz = 2.0 * (i * y - j * x)

    return (x + w * tx + (j * tz - k * ty),
            y + w * ty + (k * tx - i * tz),
            z + w * tz + (i * ty - j * tx))


def relative(q, reference):
    """`q` with `reference` taken out: the attitude SINCE the tare."""
    ri, rj, rk, rw = reference
    return _qmul(q, (-ri, -rj, -rk, rw))


def _qmul(a, b):
    """Hamilton product, (i, j, k, w) convention."""
    ai, aj, ak, aw = a
    bi, bj, bk, bw = b
    return (aw * bi + ai * bw + aj * bk - ak * bj,
            aw * bj - ai * bk + aj * bw + ak * bi,
            aw * bk + ai * bj - aj * bi + ak * bw,
            aw * bw - ai * bi - aj * bj - ak * bk)


_H = math.sqrt(0.5)

#: How the BNO085 might sit on the board. Near the centre, under the
#: A1335, turned +90 CCW about Z - the derivation is on MIRROR below.
#: The table remains as the dial that found it. (name, q).
MOUNTS = (
    ('Rz90', (0.0, 0.0, _H, _H)),
    ('Rz180', (0.0, 0.0, 1.0, 0.0)),
    ('Rz0', (0.0, 0.0, 0.0, 1.0)),
    ('Rz270', (0.0, 0.0, -_H, _H)),
    ('Rx180', (1.0, 0.0, 0.0, 0.0)),
    ('Rx180+Rz90', _qmul((1.0, 0.0, 0.0, 0.0), (0.0, 0.0, _H, _H))),
    ('Rx180+Rz180', _qmul((1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0))),
    ('Rx180+Rz270', _qmul((1.0, 0.0, 0.0, 0.0), (0.0, 0.0, -_H, _H))),
)

#: The default, first in the table.
MOUNT = MOUNTS[0][1]


#: Which raw quaternion components MIRROR before the mount is applied.
#: NONE: the mount answers this without a dial. Bench court 2026-08-29,
#: three observations against the tipped camera: board-X rotation drew as
#: Y, board-Y drew as X, CCW yaw drew CCW - an axis SWAP with Z clean,
#: which no mirror produces and only a 90-degree mount does. Three
#: independent layout reads agree on +90 CCW: U13's pin-1 dot in the
#: board-frame lower-left corner (Rz180 would put it lower-RIGHT), the
#: 3.8x5.2 outline lying with its long side - chip Y - along board X, and
#: the datasheet fig 4-3 row X=North Y=West Z=Up -> (w,x,y,z)
#: (sqrt2/2,0,0,sqrt2/2). "Roterad 180" was the eyeball estimate the
#: corner disproved.
MIRROR = (False, False, False)


def mounted(q, mount=None):
    """The BOARD's attitude, from the sensor's own quaternion."""
    i, j, k, w = q
    i, j, k = (-i if MIRROR[0] else i,
               -j if MIRROR[1] else j,
               -k if MIRROR[2] else k)
    return _qmul((i, j, k, w), _conj(MOUNT if mount is None else mount))


def _conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def attitude(q, tare=None):
    """What the display draws: the board's rotation since the tare."""
    if tare is None:
        return mounted(q)
    body = _qmul(_conj(tare), q)
    return _qmul(_qmul(MOUNT, body), _conj(MOUNT))


def matrix(q):
    """`q` as a 3x3 rotation, row-major and flattened."""
    i, j, k, w = normalise(q)

    ii, jj, kk = i * i, j * j, k * k
    ij, ik, jk = i * j, i * k, j * k
    wi, wj, wk = w * i, w * j, w * k

    return (1.0 - 2.0 * (jj + kk), 2.0 * (ij - wk), 2.0 * (ik + wj),
            2.0 * (ij + wk), 1.0 - 2.0 * (ii + kk), 2.0 * (jk - wi),
            2.0 * (ik - wj), 2.0 * (jk + wi), 1.0 - 2.0 * (ii + jj))


def normalise(q):
    """`q` as a unit quaternion, or the identity if it has no length."""
    i, j, k, w = q
    n = math.sqrt(i * i + j * j + k * k + w * w)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (i / n, j / n, k / n, w / n)


def angle_between(a, b):
    """Degrees of rotation from attitude `a` to attitude `b`."""
    ai, aj, ak, aw = normalise(a)
    bi, bj, bk, bw = normalise(b)
    dot = abs(ai * bi + aj * bj + ak * bk + aw * bw)
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def euler_degrees(q):
    """(roll, pitch, yaw) in degrees, for the caption above the picture."""
    i, j, k, w = normalise(q)

    roll = math.atan2(2.0 * (w * i + j * k), 1.0 - 2.0 * (i * i + j * j))
    sin_pitch = 2.0 * (w * j - k * i)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * k + i * j), 1.0 - 2.0 * (j * j + k * k))

    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def facing(q):
    """How much of the component side is turned towards the reader, -1..1."""
    normal = rotate(normalise(q), (0.0, 0.0, 1.0))
    return sum(normal[a] * CAMERA[a] for a in range(3))


#: What is on the component side, as (class, phi degrees, radius, radial
#: half-width, angular half-width in degrees, height). Angles run counter
#: clockwise from +X, so the first quadrant is the top right and the second
#: the top left. Positions are the layout's, in board radii - a drawing, not
#: a fabrication output, and invariant 10 applies here too.
COMPONENTS = [
    # Three phase connectors at the top, straddling the first and second
    # quadrants, out at the rim where the motor's leads reach them.
    ('connector', 68.0, 0.82, 0.11, 5.5, 0.10),
    ('connector', 90.0, 0.82, 0.11, 5.5, 0.10),
    ('connector', 112.0, 0.82, 0.11, 5.5, 0.10),

    # The gate drivers below them: six FETs, one pair per phase, on the same
    # radius so the DC link loop to each is the same length.
    ('fet', 60.0, 0.58, 0.09, 4.5, 0.05),
    ('fet', 72.0, 0.58, 0.09, 4.5, 0.05),
    ('fet', 84.0, 0.58, 0.09, 4.5, 0.05),
    ('fet', 96.0, 0.58, 0.09, 4.5, 0.05),
    ('fet', 108.0, 0.58, 0.09, 4.5, 0.05),
    ('fet', 120.0, 0.58, 0.09, 4.5, 0.05),

    # The micro, in the third quadrant and in close to the bore - the quietest
    # place on a board whose outside is a switching gate_drivers.
    ('micro', 215.0, 0.30, 0.13, 22.0, 0.035),

    # DC link, in the fourth quadrant against the first: the supply comes in
    # opposite the phases so the two harnesses do not share a run.
    ('connector', 338.0, 0.82, 0.11, 5.0, 0.10),
    ('connector', 358.0, 0.82, 0.11, 5.0, 0.10),
]


def _passives(count=26, seed=63100):
    """Small parts, scattered clear of the placed ones."""

    rng = random.Random(seed)
    taken = [(phi, r, dr, dphi) for _, phi, r, dr, dphi, _ in COMPONENTS]
    out = []

    while len(out) < count:
        phi = rng.uniform(0.0, 360.0)
        r = rng.uniform(BORE + 0.10, OUTER - 0.06)
        clear = True
        for tphi, tr, tdr, tdphi in taken:
            delta = abs((phi - tphi + 180.0) % 360.0 - 180.0)
            if delta < tdphi + 6.0 and abs(r - tr) < tdr + 0.05:
                clear = False
                break
        if clear:
            out.append(('passive', phi, r, 0.035, 2.0, 0.018))

    return out


#: What each part of the parametric board wears, 256-colour. The street
#: palette: the laminate is deep teal, the phase hardware runs sodium and
#: amber, the micro is the one cyan thing, and the passives get the dusty
#: green of a Nostromo readout. INK outlines in pale cyan over all of it.
PALETTE = {
    'board': 23,
    'connector': 208,
    'fet': 214,
    'micro': 51,
    'passive': 65,
}
INK_COLOUR = 159

#: Each zone colour as (dark, mid, bright) - picked by how lit a cell is,
#: so a part's flank falls into shadow in its own hue. Chosen on the
#: 256-colour cube one or two steps down the same column. NOT `SHADES`:
#: that name is the photographic ramp's glyph set above, and the dict
#: shadowed it - caught by the suite, not by me.
ZONE_SHADES = {
    23:  (17, 23, 37),        # board laminate: abyss teal up to teal
    214: (130, 208, 214),     # fet: burnt amber up to sodium
    208: (94, 166, 208),      # connector
    51:  (24, 38, 51),        # micro: deep cyan up to bright
    65:  (22, 65, 114),       # passive: phosphor greens
}


def _quad(out, a, b, c, d):
    """One four-sided face, as the two triangles it is made of."""
    for corners in ((a, b, c), (a, c, d)):
        normal = mesh.face_normal(corners[0], corners[1], corners[2],
                                   (0.0, 0.0, 1.0))
        if normal is None:
            continue
        for corner in corners:
            key = (round(corner[0], 5), round(corner[1], 5),
                   round(corner[2], 5))
            at = out['seen'].get(key)
            if at is None:
                at = len(out['pos']) // 3
                out['pos'].extend(key)
                out['seen'][key] = at
            out['idx'].append(at)
        out['nrm'].extend(normal)
        out['tint'].append(PALETTE.get(out['zone'], PALETTE['board']))


def _box(out, phi_deg, radius, half_r, half_phi_deg, height):
    """One part, as a box in polar coordinates."""
    half_phi = math.radians(half_phi_deg)
    phi0 = math.radians(phi_deg)
    height = height * out.get('relief', 1.0)
    top, base = HALF_THICKNESS + height, HALF_THICKNESS

    def at(ri, pi, z):
        r = radius + (half_r if ri else -half_r)
        phi = phi0 + (half_phi if pi else -half_phi)
        return (r * math.cos(phi), r * math.sin(phi), z)

    _quad(out, at(0, 0, top), at(1, 0, top), at(1, 1, top), at(0, 1, top))
    for ri, pi, rj, pj in ((0, 0, 1, 0), (1, 0, 1, 1),
                           (1, 1, 0, 1), (0, 1, 0, 0)):
        _quad(out, at(ri, pi, base), at(rj, pj, base),
              at(rj, pj, top), at(ri, pi, top))


def facets(steps=PHI_STEPS, tinted=False, relief=1.0) -> tuple:
    """The parametric board, with no STL: the DRAWING of this hardware."""
    out = {'pos': [], 'idx': [], 'nrm': [], 'seen': {}, 'tint': [],
           'zone': 'board', 'relief': relief}

    for part in COMPONENTS + _passives():
        out['zone'] = part[0]
        _box(out, *part[1:])
    out['zone'] = 'board'

    for pj in range(steps):
        a = 2.0 * math.pi * pj / steps
        b = 2.0 * math.pi * (pj + 1) / steps
        ca, sa, cb, sb = (math.cos(a), math.sin(a), math.cos(b), math.sin(b))

        for z, flip in ((HALF_THICKNESS, False), (-HALF_THICKNESS, True)):
            inner_a = (BORE * ca, BORE * sa, z)
            inner_b = (BORE * cb, BORE * sb, z)
            outer_a = (OUTER * ca, OUTER * sa, z)
            outer_b = (OUTER * cb, OUTER * sb, z)
            if flip:
                _quad(out, inner_a, outer_a, outer_b, inner_b)
            else:
                _quad(out, inner_a, inner_b, outer_b, outer_a)

        for radius, outward in ((OUTER, True), (BORE, False)):
            low_a = (radius * ca, radius * sa, -HALF_THICKNESS)
            low_b = (radius * cb, radius * sb, -HALF_THICKNESS)
            high_b = (radius * cb, radius * sb, HALF_THICKNESS)
            high_a = (radius * ca, radius * sa, HALF_THICKNESS)
            if outward:
                _quad(out, low_a, low_b, high_b, high_a)
            else:
                _quad(out, low_b, low_a, high_a, high_b)

    got = (out['pos'], out['idx'], out['nrm'])
    return got + (out['tint'],) if tinted else got


#: The CAD export, if this tree has one. The parametric board above is what
#: draws when it does not - a host with the library and no model still
#: gets a picture, and every test runs without a 21 MB file.
MODEL = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))), 'render', 'models',
    'coaxial_63100.stl')


def _load_model():
    """The mesh to draw: the export where there is one, the board otherwise."""
    try:
        return mesh.facets(MODEL)
    except (OSError, ValueError):
        return facets()


#: The cartoon model comes from the CAD EXPORT when the tree carries one:
#: the 2026-08-29 STL clustered to 5,326 triangles draws in 21 ms and is
#: the real board. The STL has no zones, so the tints come from HEIGHT -
#: the slab is teal, low parts phosphor, mid parts amber, the connectors
#: sodium - thresholds read off the export's own z histogram. A tree
#: without the STL falls back to the parametric board below.
TOON_DIVISIONS = 40

#: (upper z bound, colour) per height band, model units - the board's
#: diameter is 2.0. Above the last bound is the tallest band.
TOON_BANDS = ((0.01, 23), (0.05, 65), (0.10, 208))
TOON_TALL = 214

TOON_STEPS = 72

#: Four steps of fill and one of ink. The short ramp posterises the shading
#: - what makes an edge read as an edge instead of one more shade - and the
#: ink character stays out of the fill ramp so an outline is never mistaken
#: for a bright face.
TOON_RAMP = ' .:=+'   # five bands: four read flat on a mostly-flat board
TOON_INK = '#'

@functools.cache
def toon_mesh():
    """((positions, indices, normals), tints), built once on first use."""
    try:
        got = mesh.facets(MODEL, divisions=TOON_DIVISIONS)
        return got, _height_tints(got)
    except (OSError, ValueError):
        pos, idx, nrm, tints = facets(steps=TOON_STEPS, tinted=True,
                                      relief=3.0)
        return (pos, idx, nrm), tints


def _height_tints(model):
    """A colour per triangle, by how far its centroid stands off the slab."""
    pos, idx, _nrm = model
    tints = []
    for tri in range(len(idx) // 3):
        a, b, c = idx[3 * tri], idx[3 * tri + 1], idx[3 * tri + 2]
        z = (pos[3 * a + 2] + pos[3 * b + 2] + pos[3 * c + 2]) / 3.0
        for bound, colour in TOON_BANDS:
            if z < bound:
                tints.append(colour)
                break
        else:
            tints.append(TOON_TALL)
    return tints


#: The photographic mesh is LAZY: the toon view - the default everywhere -
#: never touches the STL, and loading its cache at import cost every view
#: 53 ms it did not use (a stale cache costs a 2 s rebuild). External
#: readers keep saying `orientation.MODEL_MESH`; PEP 562 serves them.
@functools.cache
def _model():
    return _load_model()


def __getattr__(name):
    if name == 'MODEL_MESH':
        return _model()
    raise AttributeError(name)

#: Where the camera stands: degrees round the board from +X, and up from its
#: plane. The reference's own angles are 13.2 up and 20.9 round - a product
#: shot of something standing on a bench, and this board lies flat.
#:
#: Measured over the same mesh and light, by how much of the ramp each angle
#: reaches: at 90 the drawing is two characters wide and flat by geometry
#: (down its own normal a part has no sides); at 55 the whole ramp is in use
#: and the parts stand up.
VIEW_AZIMUTH = 0.0
VIEW_ELEVATION = 55.0


def _multiply(a, b):
    """Row-major 3x3 product."""
    return tuple(sum(a[r * 3 + k] * b[k * 3 + c] for k in range(3))
                 for r in range(3) for c in range(3))


def viewpoint(azimuth=VIEW_AZIMUTH, elevation=VIEW_ELEVATION):
    """The rotation that carries a camera at (azimuth, elevation) onto +Z."""
    a = math.radians(-azimuth)
    e = math.radians(elevation - 90.0)
    ca, sa = math.cos(a), math.sin(a)
    ce, se = math.cos(e), math.sin(e)
    return _multiply((ce, 0.0, se, 0.0, 1.0, 0.0, -se, 0.0, ce),
                     (ca, -sa, 0.0, sa, ca, 0.0, 0.0, 0.0, 1.0))


#: Where the camera stands, as a unit vector in the board's own frame. What
#: `facing` asks its question against.
CAMERA = (math.cos(math.radians(VIEW_ELEVATION))
          * math.cos(math.radians(VIEW_AZIMUTH)),
          math.cos(math.radians(VIEW_ELEVATION))
          * math.sin(math.radians(VIEW_AZIMUTH)),
          math.sin(math.radians(VIEW_ELEVATION)))

#: Built once - it never changes, and it is on the path of every frame.
VIEWPOINT = viewpoint()

#: The key light, turned into the frame the viewpoint puts the model in.
#:
#: The light belongs to the world the board sits in, not to the camera: it
#: has to stay put when the viewer moves and swing when the BOARD moves,
#: because a highlight that slides as the board tilts is a cue and one that
#: slides when the camera tilts is a bug. Measured before this: dropping the
#: camera from 90 degrees to 60 darkened the board by two whole ramp steps,
#: with nothing about the board or the light having changed.
LAMP = tuple(sum(VIEWPOINT[r * 3 + k] * ascii3d.light_position()[k]
                 for k in range(3)) for r in range(3))

#: The camera fit, per window size. See _fit().
_FITS = {}
#: Fits kept before the cache is emptied - a wheel spun for a while.
FITS_KEPT = 64


def _fit(cols, rows, zoom=1.0, model=None):
    """How far to stand back for a window this size, with the board at rest."""
    model = _model() if model is None else model
    key = (id(model[0]), cols, rows, round(zoom, 3))
    got = _FITS.get(key)
    if got is not None:
        return got
    if len(_FITS) >= FITS_KEPT:
        _FITS.clear()           # a wheel spun for a while, not a leak
    got = _FITS[key] = ascii3d.fit(model[0], VIEWPOINT, cols, rows, zoom=zoom)
    return got



#: Where the board's centre sits down the attitude page, as a share of
#: its height (cy = height * LIFT). It sat above the frame's middle so
#: the model cleared the horizon line instead of sitting on it, and
#: came down a row at a time on the bench's word: 0.36 was the fit,
#: 0.39 (2026-08-30), 0.42 (2026-09-07, "a bit high up"), and dead
#: centre on 2026-09-23 - "move it down so it sits centred in the box".
#: A constant, not a fit per frame: the sphere's centre stays put while
#: the board tilts, so it never reads as translating.
LIFT = 0.5


def render(q, width=44, height=19, zoom=1.0, shop=None,
           ramp=ascii3d.CHARACTERS, toon=False, colour=False, wire=False,
           frame_on=True, crew=None, persist=None, scroll=None,
           ahead=False):
    """The board under rotation `q`, as `height` lines of `width`
    characters.
    """
    if wire and shop is None:
        # The vector drawing: chosen edges, hidden lines removed, depth- cued
        # strokes - coaxial.wireframe, not a wireframed mesh.
        from coaxial.graphics import wireframe   # here, not at the top: it imports this
        return wireframe.render(q, width, height, zoom=zoom, colour=colour,
                                horizon=frame_on, triad=frame_on, lift=LIFT,
                                crew=crew, persist=persist, scroll=scroll,
                                ahead=ahead)

    cols, rows, _cell = ascii3d.grid(width, height)
    if (toon or wire) and shop is None:
        # The whole cartoon package: the parametric board, posterised ramp,
        # culled back faces, depth-edge ink - and each part in its zone's
        # colour when `colour` is on.
        chosen, tints = toon_mesh()
        distance, off_x, off_y = _fit(cols, rows, zoom, chosen)
        return ascii3d.render(chosen, _multiply(VIEWPOINT, matrix(q)),
                              width, height, distance=distance,
                              centre=(off_x, off_y), light=LAMP,
                              ramp=TOON_RAMP, cull=True, ink=TOON_INK,
                              tints=tints if colour else None,
                              ink_colour=INK_COLOUR if colour else None,
                              shades=ZONE_SHADES if colour else None,
                              wire=wire)
    distance, off_x, off_y = _fit(cols, rows, zoom)
    if shop:
        return shop.render(_multiply(VIEWPOINT, matrix(q)), width, height,
                           distance=distance, centre=(off_x, off_y),
                           light=LAMP, ramp=ramp)
    return ascii3d.render(_model(), _multiply(VIEWPOINT, matrix(q)), width,
                          height, distance=distance, centre=(off_x, off_y),
                          light=LAMP, ramp=ramp)
def picture(q, width=44, height=19, frame=None, age=None, zoom=1.0,
            shop=None, toon=False, colour=False):
    """The drawing with the numbers it is a reading of, above it."""
    roll, pitch, yaw = euler_degrees(q)
    i, j, k, w = normalise(q)
    side = 'component side' if facing(q) > 0.0 else 'solder side'

    def label(text):
        return ansi.paint(text, ansi.ASH) if colour else text

    lines = [
        '%s %+.4f   %s %+.4f   %s %+.4f   %s %+.4f'
        % (label('q   i'), i, label('j'), j, label('k'), k,
           label('real'), w),
        '%s %+7.1f  %+7.1f  %+7.1f  %s'
        % (label('rpy'), roll, pitch, yaw, label('degrees')),
    ]

    if frame is not None:
        note = 'frame %-6d %s' % (frame, SPINNER[frame % len(SPINNER)])
        note += ('   no new sample for %d frame%s'
                 % (age, '' if age == 1 else 's')) if age else '   live'
        lines.append(note)

    lines += [
        '',
        render(q, width, height, zoom, shop, toon=toon, colour=colour),
        '',
        label('coaxial_63100 - facing you: ') + side,
    ]

    return '\n'.join(lines)
