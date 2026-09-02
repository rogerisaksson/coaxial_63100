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

from . import ansi, ascii3d, mesh

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


def relative(q, reference):
    """`q` with `reference` taken out: the attitude SINCE the tare.

    q_rel = q * conj(reference): the rotation SINCE the tare, in WORLD
    axes - the frame the camera lives in. The body-side form
    (conj(ref) * q) was tried first and it conjugated the Rz180 mounting
    through everything, which mirrored X and Y exactly as measured on the
    bench; on the world side the mount cancels out of the relative
    attitude entirely. The zero button still kills the mounting offset
    and the arbitrary yaw in one press.
    """
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
    """The BOARD's attitude, from the sensor's own quaternion.

    The measured MIRROR first, then q_board = q_sensor * conj(mount):
    sensor axes are R(mount) times board axes, so board coordinates are
    R(mount) applied to sensor coordinates, which is the RIGHT-side
    conjugate. At the old Rz180 the two sides were the same rotation and
    the distinction was invisible; at 90 degrees it is the whole bug."""
    i, j, k, w = q
    i, j, k = (-i if MIRROR[0] else i,
               -j if MIRROR[1] else j,
               -k if MIRROR[2] else k)
    return _qmul((i, j, k, w), _conj(MOUNT if mount is None else mount))


def _conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def attitude(q, tare=None):
    """What the display draws: the board's rotation since the tare.

    q_disp = MOUNT * (conj(tare) * q) * conj(MOUNT) - the body-relative
    change, conjugated into board axes through the mounting. Derivation:
    q_board = q_sensor * conj(MOUNT), so conj(q_board0) * q_board wraps
    the sensor-frame change in MOUNT on the LEFT. Every simpler form was
    tried and measured wrong on the bench:

      * q * MOUNT alone carries the resting mount and the arbitrary yaw;
      * body-side tare conj(ref) * q conjugated the mount through the
        result and mirrored X and Y - the first dial finding;
      * world-side tare q * conj(ref) cancelled the mount but expressed
        the turn in world axes, mirroring X once the tare yaw was 180;
      * conj(MOUNT) * body * MOUNT - this sandwich REVERSED - passed
        every numeric check at the old Rz180 mount, because a 180 equals
        its own conjugate and the two orders coincide. The 90-degree
        mount is where they part, and the bench said so: X drew as Y and
        Y as X while Z stayed true (2026-08-29).

    With MOUNT = Rz90: +theta about board X draws +theta about screen X,
    board Y about screen Y, CCW yaw draws CCW, and rest is identity.
    """
    if tare is None:
        return mounted(q)
    body = _qmul(_conj(tare), q)
    return _qmul(_qmul(MOUNT, body), _conj(MOUNT))


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


def angle_between(a, b):
    """Degrees of rotation from attitude `a` to attitude `b`.

    The rotation taking one to the other has a real part equal to the
    cosine of half the angle - |<a, b>| for unit quaternions, the
    absolute value because q and -q are the same rotation. Pure, for a
    view to ask "did the board actually move" before it redraws.
    """
    ai, aj, ak, aw = normalise(a)
    bi, bj, bk, bw = normalise(b)
    dot = abs(ai * bi + aj * bj + ak * bk + aw * bw)
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


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
    """One four-sided face, as the two triangles it is made of.

    Indexed through `out['seen']`, which is what made the old form a latent
    fault: it appended corner positions and per-face normals into the first
    two slots of a tuple and returned an empty third, so on a machine
    without the STL the fallback board had no indices and drew NOTHING.
    Never seen here because this tree carries the export - the shape only
    had to be wrong somewhere the tests never run.
    """
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
    """One part, as a box in polar coordinates.

    Polar because it follows the board's curvature, which is what a part on a
    round PCB does and what keeps the outer ones from hanging off the rim.
    """
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


def facets(steps=PHI_STEPS, tinted=False, relief=1.0):
    """The parametric board, with no STL: the DRAWING of this hardware.

    Four surfaces, because a board has four - the component face, the solder
    face, the outer rim and the bore - and then what is mounted on the
    component side. Built once and reused: the geometry does not change,
    only the rotation does.

    `steps` is the annulus resolution; the toon view runs it coarse on
    purpose. `tinted` appends the per-triangle palette colour, for the
    renderer's zone tints. `relief` scales the parts' heights - a DRAWING
    exaggerates: at 1.0 a FET is 0.05 units proud of a board seen from 55
    degrees, under one cell at any terminal size, and the picture read as
    an empty disc.
    """
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
#: draws when it does not - a machine with the library and no model still
#: gets a picture, and every test runs without a 21 MB file.
MODEL = os.path.join(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))), 'render', 'models',
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

_TOON = None


def toon_mesh():
    """((positions, indices, normals), tints), built once on first use."""
    global _TOON
    if _TOON is None:
        try:
            got = mesh.facets(MODEL, divisions=TOON_DIVISIONS)
            _TOON = (got, _height_tints(got))
        except (OSError, ValueError):
            built = facets(steps=TOON_STEPS, tinted=True, relief=3.0)
            _TOON = (built[:3], built[3])
    return _TOON


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
_PHOTO_MESH = None


def _model():
    global _PHOTO_MESH
    if _PHOTO_MESH is None:
        _PHOTO_MESH = _load_model()
    return _PHOTO_MESH


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
    """The rotation that carries a camera at (azimuth, elevation) onto +Z.

    Applied before the board's own rotation, so it is where the viewer
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
#: has to stay put when the viewer moves and swing when the BOARD moves,
#: because a highlight that slides as the board tilts is a cue and one that
#: slides when the camera tilts is a bug. Measured before this: dropping the
#: camera from 90 degrees to 60 darkened the board by two whole ramp steps,
#: with nothing about the board or the light having changed.
LAMP = tuple(sum(VIEWPOINT[r * 3 + k] * ascii3d.light_position()[k]
                 for k in range(3)) for r in range(3))

#: The camera fit, per window size. See _fit().
_FITS = {}


def _fit(cols, rows, zoom=1.0, model=None):
    """How far to stand back for a window this size, with the board at rest.

    Measured from the viewpoint alone and not per frame: a fit that tracked
    the board's own rotation would grow and shrink the drawing as it tilted,
    which reads as the board moving toward you rather than turning. Fixed, a
    violent tilt can push a corner past the edge - a fair trade for a picture
    that fills the window at rest, and the numbers above it are the reading.

    Cached because a window is resized far less often than it is redrawn.
    """
    model = _model() if model is None else model
    key = (id(model[0]), cols, rows, round(zoom, 3))
    got = _FITS.get(key)
    if got is None:
        got = ascii3d.fit(model[0], VIEWPOINT, cols, rows, zoom=zoom)
        if len(_FITS) > 64:
            _FITS.clear()       # a wheel spun for a while, not a leak
        _FITS[key] = got
    return got



def render(q, width=44, height=19, zoom=1.0, shop=None,
           ramp=ascii3d.CHARACTERS, toon=False, colour=False, wire=False,
           frame_on=True, crew=None):
    """The board under rotation `q`, as `height` lines of `width` characters.

    The drawing is `ascii3d`, which is three.js's AsciiEffect ported out of
    the browser - its ramp, its light, its brightness mapping and its two
    framebuffer rows per character row. What is this module's is the model,
    the rotation and the caption.

    `ramp` is threaded rather than left to ascii3d's default because the
    camera distance and centring are worked out HERE, from the fit: a caller
    that wanted a shorter ramp had to re-derive both, and reaching past this
    function for one of them is how a drawing ends up off-centre.
    """
    if wire and shop is None:
        # The vector drawing: chosen edges, hidden lines removed, depth-
        # cued strokes - coaxial.wireframe, not a wireframed mesh.
        from . import wireframe
        # Above the frame's centre: the model clears the horizon line
        # instead of sitting on it. 0.39 is one row below 0.36 at the
        # view's 36 rows (cy = height * lift), asked 2026-08-30.
        return wireframe.render(q, width, height, zoom=zoom, colour=colour,
                                horizon=frame_on, triad=frame_on, lift=0.39,
                                crew=crew)

    cols, rows, _cell = ascii3d.grid(width, height)
    if (toon or wire) and shop is None:
        # The whole cartoon package: the parametric board, posterised ramp,
        # culled back faces, depth-edge ink - and each part in its zone's
        # colour when `colour` is on. 956 triangles, so no process pool.
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
    draw = shop.render if shop else ascii3d.render
    model = () if shop else (_model(),)
    return draw(*model, _multiply(VIEWPOINT, matrix(q)), width, height,
                distance=distance, centre=(off_x, off_y), light=LAMP,
                ramp=ramp)


def picture(q, width=44, height=19, frame=None, age=None, zoom=1.0,
            shop=None, toon=False, colour=False):
    """The drawing with the numbers it is a reading of, above it.

    The quaternion leads: it is what the part reports and what moves when the
    board does. `frame` and `age` are for a live view - a counter that moves
    says the picture is being redrawn, and the age says whether the part is
    still sending.
    """
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
