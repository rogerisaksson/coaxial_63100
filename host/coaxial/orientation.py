"""The board's attitude, drawn in characters.

The IMU reports a rotation vector - a unit quaternion saying how the part is
turned relative to its reference frame. This turns that into a picture of the
PCB, because a quaternion read aloud tells nobody which way up anything is.

The board is a flat annulus: 100 mm across with a 10 mm bore, and about as
thin as a PCB is. It mounts coaxially behind an outrunner's stator with the
shaft through the middle.

Drawn the way a1k0n draws a donut (a1k0n.net/2011/07/20/donut-math.html) -
sample the surface, project with perspective, keep the nearest sample in a
z-buffer and shade each cell by how the surface there faces the light. Not a
torus, though: a torus with a small hole is necessarily a fat one, because
one radius sets both the bore and the thickness. This samples the two faces
and the two rims separately, which is what a board is. The rotation is not an
animation angle either - it is the quaternion the part reported, so the
picture turns because the board did.

Pure: a quaternion in, a block of text out. No serial port, no terminal, no
clock - so it is testable without a board, and `tools/show_orientation.py` is
the only thing that needs one.
"""
import math

from .raster import cell
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

    The board's +Z normal after rotation. Positive means the component side
    is visible, and the caption says which the reader is looking at.
    """
    return rotate(normalise(q), (0.0, 0.0, 1.0))[2]


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

    # The bridge below them: six FETs, one pair per phase, on the same
    # radius so the DC link loop to each is the same length.
    ('fet', 60.0, 0.58, 0.09, 4.5, 0.05),
    ('fet', 72.0, 0.58, 0.09, 4.5, 0.05),
    ('fet', 84.0, 0.58, 0.09, 4.5, 0.05),
    ('fet', 96.0, 0.58, 0.09, 4.5, 0.05),
    ('fet', 108.0, 0.58, 0.09, 4.5, 0.05),
    ('fet', 120.0, 0.58, 0.09, 4.5, 0.05),

    # The micro, in the third quadrant and in close to the bore - the
    # quietest place on a board whose outside is a switching bridge.
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


def _part(kind, phi_deg, radius, half_r, half_phi_deg, height):
    """One part's top and walls, as (point, normal, ramp) in the board frame.

    A box in polar coordinates: it follows the board's curvature, which is
    what a part on a round PCB does and what keeps the outer ones from
    hanging off the rim.
    """
    ramp = PART_SHADES[kind]
    top = HALF_THICKNESS + height
    half_phi = math.radians(half_phi_deg)
    phi0 = math.radians(phi_deg)
    out = []

    steps_r, steps_p = 7, 9
    for ri in range(steps_r):
        r = radius - half_r + 2.0 * half_r * ri / (steps_r - 1)
        for pi in range(steps_p):
            phi = phi0 - half_phi + 2.0 * half_phi * pi / (steps_p - 1)
            cp, sp = math.cos(phi), math.sin(phi)
            out.append(((r * cp, r * sp, top), (0.0, 0.0, 1.0), ramp))

            # The walls, so a part turned away from the reader still has a
            # side to it rather than vanishing to its own outline.
            if ri in (0, steps_r - 1):
                sign = -1.0 if ri == 0 else 1.0
                for zi in range(3):
                    z = HALF_THICKNESS + height * zi / 2.0
                    out.append(((r * cp, r * sp, z),
                                (sign * cp, sign * sp, 0.0), ramp))
            if pi in (0, steps_p - 1):
                sign = -1.0 if pi == 0 else 1.0
                for zi in range(3):
                    z = HALF_THICKNESS + height * zi / 2.0
                    out.append(((r * cp, r * sp, z),
                                (-sign * sp, sign * cp, 0.0), ramp))

    return out


def samples():
    """Every (point, normal, ramp) on the board, in its own frame.

    Four surfaces, because a board has four - the component face, the solder
    face, the outer rim and the bore - and then what is mounted on the
    component side. Built once and reused: the geometry does not change, only
    the rotation does, and rebuilding it per frame was the whole cost of a
    20 Hz redraw.
    """
    out = []

    for part in COMPONENTS + _passives():
        out.extend(_part(*part))

    for pj in range(PHI_STEPS):
        phi = 2.0 * math.pi * pj / PHI_STEPS
        cp, sp = math.cos(phi), math.sin(phi)

        for ri in range(RADIAL_STEPS):
            r = BORE + (OUTER - BORE) * ri / (RADIAL_STEPS - 1)
            for z, nz in ((HALF_THICKNESS, 1.0), (-HALF_THICKNESS, -1.0)):
                out.append(((r * cp, r * sp, z), (0.0, 0.0, nz), SHADES))

        # The rims. Thin, so a few samples across is enough to keep the edge
        # from breaking up when the board is turned towards edge on - which
        # is exactly when the rim is all there is to see.
        for si in range(RIM_STEPS):
            z = -HALF_THICKNESS + 2.0 * HALF_THICKNESS * si / (RIM_STEPS - 1)
            out.append(((OUTER * cp, OUTER * sp, z), (cp, sp, 0.0), SHADES))
            out.append(((BORE * cp, BORE * sp, z), (-cp, -sp, 0.0), SHADES))

    return out


#: The CAD export, if this tree has one. The parametric board below is what
#: draws when it does not - a machine with the library and no model still
#: gets a picture, and every test runs without a 21 MB file.
MODEL = os.path.join(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))), 'render', 'models',
    'Coaxial 63100.stl')


def _load_surface():
    """What to draw: the model where there is one, the parametric board
    otherwise. Read once at import - see samples()."""
    try:
        return [(p, nrm, SHADES) for p, nrm in mesh.load(MODEL)]
    except (OSError, ValueError):
        return samples()


#: The board's surface, built once. See _load_surface().
SURFACE = _load_surface()

#: The same surface as (point, normal), which is what ascii3d consumes. Built
#: once beside it rather than per frame: it is 45,000 tuples.
_PAIRS = [(point, normal) for point, normal, _ramp in SURFACE]


def _light():
    """The light direction, as a unit vector."""
    x, y, z = LIGHT
    n = math.sqrt(x * x + y * y + z * z)
    return (x / n, y / n, z / n)


def render(q, width=44, height=19, zoom=1.0):
    """The board under rotation `q`, as `height` lines of `width` characters.

    The drawing is `ascii3d`, which is three.js's AsciiEffect ported out of
    the browser - its ramp, its light, its brightness mapping and its two
    framebuffer rows per character row. What is this module's is the model,
    the rotation and the caption.
    """
    return ascii3d.render(_PAIRS, matrix(q), width, height, zoom=zoom)


def picture(q, width=44, height=19, frame=None, age=None, zoom=1.0):
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
        render(q, width, height, zoom),
        '',
        'coaxial_63100 - facing you: %s' % side,
    ]

    return '\n'.join(lines)
