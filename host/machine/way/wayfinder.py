"""The wayfinder's gauge (foliated_riemannian_nav.lisp): the gap her way meets, her trunk twisted.

    width = wayfinder.gap(centre, half, R, at, along, floor, most)  # the free width across her way
    yaw = wayfinder.sliced(width, NOMINAL, DEPTH)  # her trunk's twist that fits her through it
    yaw = wayfinder.ahead(boxes, at, heading, speed, floor, margin)  # the twist for her way AHEAD_S on
    yaw = wayfinder.gauged(g, s)               # a pass of her session: the room's boxes, her route's gauge

The text's forward gauge (`minkowski-gauge-projection`, `slice-narrow-aperture-axial`): the free width
across her way the text's 0.6 s ahead among what her high band meets, and the trunk's twist that fits
her chest through it, on a route's twist (an errand's), not a word's. Its backward coupling - her
flow bent by the rows a shield holds against the room - waits on such rows (`machine.law.schema`).
"""
import math

from machine.texts import kernel
from machine.body.figure import BODY, SEGMENTS

_ARM = next(seg for seg in SEGMENTS if seg[0] == 'left_upper_arm')
_TORSO = [c for c in BODY if c[0] == 'torso']
#: Her chest across her shoulders' skin and front to back, m: her upper arms' and her torso's capsules.
NOMINAL = 2.0 * (abs(_ARM[3][0]) + next(c[1] for c in BODY if c[0] == 'upper_arm'))
DEPTH = (max(max(c[2][2], c[3][2]) + c[1] for c in _TORSO) - min(min(c[2][2], c[3][2]) - c[1] for c in _TORSO))
_GAUGE = kernel.node('synthesize-riemannian-metric', 'geodesic')['minkowski-gauge-projection']
_SLICER = kernel.node('slice-narrow-aperture-axial', 'aperture')['aperture-yaw-slicer']
#: How far ahead along her flow a gap is gauged, s; her trunk's most twist, rad.
AHEAD_S, TWIST = _GAUGE['lookahead-horizon-s'], _SLICER['max-axial-twist-rad']
_BAND = kernel.node('configure-foliated-sdf-bands', 'sdf-bands')['slabs'][2].args['z-window-mm']
#: The high band's heights over her soles, m: what a gap across her chest is of.
BAND = (1e-3 * _BAND[0], 1e-3 * _BAND[1])
#: The skin a gap is read less, each side, m (`extrude-velocity-braking-cocoon`).
MARGIN = kernel.node('xform-universal-spatial-awareness', 'braking-cocoon')['base-margin-m']


def gap(centre, half, R, at, along, floor, most):
    """The free width across her way at `at` (x, z) going `along` (x, z, a unit): the nearest box
    either side along the line across it, at a height of each box within the high band over `floor`,
    among the boxes (centre, half sizes, rotations) the band meets - the others masked, shapes fixed
    by the boxes' -; a side `most` m at the farthest, 0 where `at` is within a box (the slab method).
    Compiled on her loop a room's size a part (`machine.law.motion.laid`)."""
    import numpy as np
    up = np.abs(R[:, 1, :])
    reach = (up * half).sum(1)
    meets = (centre[:, 1] - reach - floor <= BAND[1]) & (centre[:, 1] + reach - floor >= BAND[0])
    y = np.clip(centre[:, 1], floor + BAND[0], floor + BAND[1])
    o = np.stack([np.full(len(centre), at[0]), y, np.full(len(centre), at[1])], 1)
    lo_ = np.einsum('kji,kj->ki', R, o - centre)
    d = np.einsum('kji,j->ki', R, np.array([along[1], 0.0, -along[0]]))
    flat = np.abs(d) < 1e-12
    with np.errstate(divide='ignore', invalid='ignore'):
        t1, t2 = (-half - lo_) / d, (half - lo_) / d
    inside = np.abs(lo_) <= half
    enter = np.where(flat, np.where(inside, -np.inf, np.inf), np.minimum(t1, t2)).max(1)
    leave = np.where(flat, np.where(inside, np.inf, -np.inf), np.maximum(t1, t2)).min(1)
    hit = meets & (enter <= leave)
    ahead_ = np.where(hit & (enter >= 0.0), enter, most).min(initial=most)
    behind = np.where(hit & (leave <= 0.0), -leave, most).min(initial=most)
    within = (hit & (enter < 0.0) & (leave > 0.0)).any()
    return np.where(within, 0.0, np.minimum(ahead_, most) + np.minimum(behind, most))


def sliced(width, nominal, depth):
    """Her trunk's twist that fits her chest - `nominal` across, `depth` front to back, m - through a
    gap `width` m wide: 0 where she fits square on; else past the twist her depth first widens her to,
    nominal cos t + depth sin t = width; TWIST at most - past it the gap is under her contorted width."""
    rho, phi = math.hypot(nominal, depth), math.atan2(depth, nominal)
    fit = phi + math.acos(max(-1.0, min(1.0, width / rho)))
    return min(TWIST, fit) * float(width < nominal)


def ahead(boxes, at, heading, speed, floor, margin, gap=gap):
    """Her trunk's twist for the gap her way meets AHEAD_S on: `boxes` the room's (`schema.room`), `at`
    her pelvis (x, z) going `speed` m/s along `heading`, her lower sole at `floor`, each side's
    `margin` her skin's - by `gap`, this module's or its C (`gauged`)."""
    import numpy as np
    sh, ch = math.sin(heading), math.cos(heading)
    reach = max(0.0, speed) * AHEAD_S
    width = gap(boxes['centre'], boxes['half'], boxes['R'], np.array([at[0] + reach * sh, at[1] + reach * ch]),
                np.array([sh, ch]), floor, NOMINAL)
    return sliced(width - 2.0 * margin, NOMINAL, DEPTH)


def gauged(g, s):
    """Her trunk's twist this pass of her session `g`: the gauge's for the gap her route meets among
    her things' boxes (`schema.room`) - none on a word's twist -, her gaze to be kept against it; the
    gap in C (`g['gap']`, `machine.law.motion.laid`)."""
    from machine.law import schema
    floor = min(float(f['sole'][1]) for f in s['feet'])
    at = (float(g['est']['pos'][0]), float(g['est']['pos'][2]))
    return float(g.get('_errand') is not None) * ahead(schema.room(g.get('things', ())), at, g['heading'],
                                                        float(g.get('twist', (0.0,))[0]), floor, MARGIN, g['gap'])
