"""The board's attitude, drawn in characters.

The IMU reports a unit quaternion; read aloud it tells nobody which way up
anything is. This draws the PCB instead.

The board is a flat annulus, 100 mm across with a 10 mm bore, PCB-thin,
mounted coaxially behind an outrunner's stator with the shaft through it.

Drawn the way a1k0n draws a donut (a1k0n.net/2011/07/20/donut-math.html):
sample the surface, project with perspective, z-buffer the nearest, shade by
how the surface faces the light. Not a torus - one radius would set both the
bore and the thickness, so a small hole forces a fat ring; the two faces and
two rims are sampled separately. The rotation is the reported quaternion, not
an animation angle, so the picture turns because the board did.

Pure: a quaternion in, text out. No serial port, terminal or clock, so it is
testable without a board; `tools/show_orientation.py` is what needs one.
"""
import math
import os

from . import ascii3d, mesh

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

#: Samples across a face and around the board. Fine enough that the surface
#: has no gaps at this character resolution; every sample is rotated,
#: projected and z-tested, so this is what it costs.
RADIAL_STEPS = 40
PHI_STEPS = 260
RIM_STEPS = 4

#: Dimmest to brightest - a1k0n's ramp, twelve levels of how squarely a
#: surface faces the light. Bare copper; the parts have their own, below.
SHADES = '.,-~:;=!*#$@'

#: A part is not distinguishable from the board it sits on by shading alone:
#: its top is parallel to the face under it, so it takes the same luminance
#: and the same character. Each class gets its own ramp instead, and the
#: board reads as populated rather than as a lit disc.
PART_SHADES = {
    'connector': ':H#',
    'fet':       '.o0',
    'micro':     '-=%',
    'passive':   '.:o',
}

#: Where the light is, in the viewer's frame: over the reader's left shoulder
#: and towards the reader, so the face turned this way is the lit one.
LIGHT = (0.0, 1.0, 1.0)

#: Frames of the liveness spinner. A picture of a still board and a frozen
#: one look identical without something that moves.
SPINNER = ('|', '/', '-', '\\')


def rotate(q, v):
    """`v` turned by the unit quaternion `q`, given as (i, j, k, real).

    The sandwich product q*v*conj(q), written out rather than composed from a
    quaternion class: there is one rotation in this repository and a class for
    it would be an abstraction with one implementation.
    """
    i, j, k, w = q
    x, y, z = v

    tx = 2.0 * (j * z - k * y)
    ty = 2.0 * (k * x - i * z)
    tz = 2.0 * (i * y - j * x)

    return (x + w * tx + (j * tz - k * ty),
            y + w * ty + (k * tx - i * tz),
            z + w * tz + (i * ty - j * tx))


def matrix(q):
    """`q` as a 3x3 rotation, row-major and flattened.

    render() turns 45,000 points and 45,000 normals per frame. Through
    rotate() that is 90,000 sandwich products; as a matrix it is one
    construction and nine multiplies apiece. Measured: 54 ms a frame down to
    28, which is the difference between 18 frames a second and 20.
    """
    i, j, k, w = normalise(q)

    ii, jj, kk = i * i, j * j, k * k
    ij, ik, jk = i * j, i * k, j * k
    wi, wj, wk = w * i, w * j, w * k

    return (1.0 - 2.0 * (jj + kk), 2.0 * (ij - wk), 2.0 * (ik + wj),
            2.0 * (ij + wk), 1.0 - 2.0 * (ii + kk), 2.0 * (jk - wi),
            2.0 * (ik - wj), 2.0 * (jk + wi), 1.0 - 2.0 * (ii + jj))


def normalise(q):
    """`q` as a unit quaternion, or the identity if it has no length.

    A rotation vector from a part that is still settling can read all zeros,
    and dividing by that would put a NaN on the screen where an orientation
    belongs.
    """
    i, j, k, w = q
    n = math.sqrt(i * i + j * j + k * k + w * w)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (i / n, j / n, k / n, w / n)


def euler_degrees(q):
    """(roll, pitch, yaw) in degrees, for the caption above the picture.

    Aerospace order - yaw about Z, then pitch about Y, then roll about X -
    because that is what anybody reading "the board is tilted" expects.
    """
    i, j, k, w = normalise(q)

    roll = math.atan2(2.0 * (w * i + j * k), 1.0 - 2.0 * (i * i + j * j))
    sin_pitch = 2.0 * (w * j - k * i)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * k + i * j), 1.0 - 2.0 * (j * j + k * k))

    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def facing(q):
    """How much of the component side is turned towards the reader, -1..1.

    The board's +Z normal after rotation, against the direction the camera
    actually stands in - not against +Z. The camera has been oblique since
    the viewpoint was taken from the reference, and reading the normal's z
    alone would have gone on answering for a camera that is not there.
    """
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

    # The micro, in the third quadrant and in close to the bore - the
    # quietest place on a board whose outside is a switching gate_drivers.
    ('micro', 215.0, 0.30, 0.13, 22.0, 0.035),

    # DC link, in the fourth quadrant against the first: the supply comes in
    # opposite the phases so the two harnesses do not share a run.
    ('connector', 338.0, 0.82, 0.11, 5.0, 0.10),
    ('connector', 358.0, 0.82, 0.11, 5.0, 0.10),
]


def _passives(count=26, seed=63100):
    """Small parts, scattered clear of the placed ones.

    Seeded, not random per frame: a picture whose passives move is a picture
    of a board that is coming apart.
    """
    import random

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


def _quad(out, a, b, c, d):
    """One four-sided face, as the two triangles it is made of."""
    for corners in ((a, b, c), (a, c, d)):
        normal = mesh.face_normal(corners[0], corners[1], corners[2],
                                   (0.0, 0.0, 1.0))
        if normal is None:
            continue
        for corner in corners:
            out[0].append(corner[0])
            out[0].append(corner[1])
            out[0].append(corner[2])
        out[1].append(normal[0])
        out[1].append(normal[1])
        out[1].append(normal[2])


def _box(out, phi_deg, radius, half_r, half_phi_deg, height):
    """One part, as a box in polar coordinates.

    Polar because it follows the board's curvature, which is what a part on a
    round PCB does and what keeps the outer ones from hanging off the rim.
    """
    half_phi = math.radians(half_phi_deg)
    phi0 = math.radians(phi_deg)
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


def facets():
    """The parametric board as (positions, indices, normals), with no STL.

    Four surfaces, because a board has four - the component face, the solder
    face, the outer rim and the bore - and then what is mounted on the
    component side. Built once and reused: the geometry does not change, only
    the rotation does.
    """
    out = ([], [], [], {})

    for part in COMPONENTS + _passives():
        _box(out, *part[1:])

    for pj in range(PHI_STEPS):
        a = 2.0 * math.pi * pj / PHI_STEPS
        b = 2.0 * math.pi * (pj + 1) / PHI_STEPS
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

    return out[0], out[1], out[2]


#: The CAD export, if this tree has one. The parametric board above is what
#: draws when it does not - a machine with the library and no model still
#: gets a picture, and every test runs without a 21 MB file.
MODEL = os.path.join(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))), 'render', 'models',
    'Coaxial 63100.stl')


def _load_model():
    """The mesh to draw: the export where there is one, the board otherwise."""
    try:
        return mesh.facets(MODEL)
    except (OSError, ValueError):
        return facets()


#: The board's triangles, read once at import. See _load_model().
MODEL_MESH = _load_model()

#: Where the camera stands, in the board's own frame: degrees round the board
#: from the +X axis, and degrees up from the board's plane.
#:
#: The reference stands its camera at `(bbox.max.x * 4, bbox.max.y,
#: bbox.max.z * 3)` after turning every STL by `rotation.x = -90`, which for
#: this model - 100 x 100 x 51 mm - is 13.2 degrees above the board and 20.9
#: round it. That is a product shot of something standing on a bench. This
#: board is horizontal and what the view is FOR is showing how it lies, so
#: the camera looks down at it instead.
#:
#: Looking down at a board that lies flat, but not straight down. Measured
#: over the same mesh and light, counting how much of the ramp each angle
#: reaches: at 90 degrees the drawing is two characters wide - # and @ - and
#: is flat by geometry, because down its own normal a component has no sides
#: and the board's face and every part's top share one normal. At 55 the
#: whole ramp is in use and the parts stand up. 30 is the reference's kind of
#: angle and reads as a product shot rather than as how the board lies.
VIEW_AZIMUTH = 0.0
VIEW_ELEVATION = 55.0


def _multiply(a, b):
    """Row-major 3x3 product."""
    return tuple(sum(a[r * 3 + k] * b[k * 3 + c] for k in range(3))
                 for r in range(3) for c in range(3))


def viewpoint(azimuth=VIEW_AZIMUTH, elevation=VIEW_ELEVATION):
    """The rotation that carries a camera at (azimuth, elevation) onto +Z.

    Applied before the board's own rotation, so it is where the observer
    stands and not something the board is doing: the board still turns
    exactly as the IMU says, seen from a fixed corner of the room.
    """
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
#: has to stay put when the observer moves and swing when the BOARD moves,
#: because a highlight that slides as the board tilts is a cue and one that
#: slides when the camera tilts is a bug. Measured before this: dropping the
#: camera from 90 degrees to 60 darkened the board by two whole ramp steps,
#: with nothing about the board or the light having changed.
LAMP = tuple(sum(VIEWPOINT[r * 3 + k] * ascii3d.light_position()[k]
                 for k in range(3)) for r in range(3))

#: The camera fit, per window size. See _fit().
_FITS = {}


def _fit(cols, rows, zoom=1.0):
    """How far to stand back for a window this size, with the board at rest.

    Measured from the viewpoint alone and not per frame: a fit that tracked
    the board's own rotation would grow and shrink the drawing as it tilted,
    which reads as the board moving toward you rather than turning. Fixed, a
    violent tilt can push a corner past the edge - a fair trade for a picture
    that fills the window at rest, and the numbers above it are the reading.

    Cached because a window is resized far less often than it is redrawn.
    """
    key = (cols, rows, round(zoom, 3))
    got = _FITS.get(key)
    if got is None:
        got = ascii3d.fit(MODEL_MESH[0], VIEWPOINT, cols, rows, zoom=zoom)
        if len(_FITS) > 64:
            _FITS.clear()       # a wheel spun for a while, not a leak
        _FITS[key] = got
    return got



def render(q, width=44, height=19, zoom=1.0, shop=None):
    """The board under rotation `q`, as `height` lines of `width` characters.

    The drawing is `ascii3d`, which is three.js's AsciiEffect ported out of
    the browser - its ramp, its light, its brightness mapping and its two
    framebuffer rows per character row. What is this module's is the model,
    the rotation and the caption.
    """
    cols, rows, _cell = ascii3d.grid(width, height)
    distance, off_x, off_y = _fit(cols, rows, zoom)
    draw = shop.render if shop else ascii3d.render
    model = () if shop else (MODEL_MESH,)
    return draw(*model, _multiply(VIEWPOINT, matrix(q)), width, height,
                distance=distance, centre=(off_x, off_y), light=LAMP)


def picture(q, width=44, height=19, frame=None, age=None, zoom=1.0,
            shop=None):
    """The drawing with the numbers it is a reading of, above it.

    The quaternion leads: it is what the part reports and what moves when the
    board does. `frame` and `age` are for a live view - a counter that moves
    says the picture is being redrawn, and the age says whether the part is
    still sending.
    """
    roll, pitch, yaw = euler_degrees(q)
    i, j, k, w = normalise(q)
    side = 'component side' if facing(q) > 0.0 else 'solder side'

    lines = [
        'q   i %+.4f   j %+.4f   k %+.4f   real %+.4f' % (i, j, k, w),
        'rpy %+7.1f  %+7.1f  %+7.1f  degrees' % (roll, pitch, yaw),
    ]

    if frame is not None:
        note = 'frame %-6d %s' % (frame, SPINNER[frame % len(SPINNER)])
        note += ('   no new sample for %d frame%s'
                 % (age, '' if age == 1 else 's')) if age else '   live'
        lines.append(note)

    lines += [
        '',
        render(q, width, height, zoom, shop),
        '',
        'coaxial_63100 - facing you: %s' % side,
    ]

    return '\n'.join(lines)
