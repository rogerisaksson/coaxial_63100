"""The kernel's wayfinder by her shield (foliated_riemannian_nav.lisp): her flow bent, a gap's twist.

    v = wayfinder.deflected(v, normals, lams)    # her flow (x, z), each held room row's rank-1 pullback
    twist = wayfinder.turned(twist, heading, shield, flow)   # her twist, its flow her route's
    width = wayfinder.gap(centre, half, R, at, along, floor, most)  # the free width across her way
    yaw = wayfinder.sliced(width, NOMINAL, DEPTH)  # her trunk's twist that fits her through it
    yaw = wayfinder.ahead(boxes, at, heading, speed, floor, margin)  # the twist for her way AHEAD_S on

The text's backward coupling (`synthesize-riemannian-metric`'s `ecbf-stress-pullback`): each room row her
shield holds (`machine.schema`) deflects her flow by Sherman-Morrison - v - eta lam / (1 + eta lam) min(0,
<v, n>) n, n its normal along the floor - so she slides along what she would walk into, never into it.
Its forward gauge (`minkowski-gauge-projection`, `slice-narrow-aperture-axial`): the free width
across her way the text's 0.6 s ahead, among what her high band meets, and the trunk's twist that
fits her chest through it.
"""
import math

from machine import kernel
from machine.figure import BODY, SEGMENTS

_PULLBACK = kernel.node('synthesize-riemannian-metric', 'geodesic')['ecbf-stress-pullback']
#: The pullback's stiffness, 1/N.
ETA = _PULLBACK['coupling-gain-eta']


def deflected(v, normals, lams):
    """Her flow `v` (x, z) deflected by each held row's normal (x, z of its 3D normal, from the box to
    her) and multiplier: v - eta lam / (1 + eta lam) min(0, <v, n>) n, one after another."""
    x, z = float(v[0]), float(v[1])
    for (nx, nz), lam in zip(normals, lams):
        size = math.hypot(nx, nz)
        ux, uz = nx / max(size, 1e-12), nz / max(size, 1e-12)
        share = ETA * lam / (1.0 + ETA * lam) * min(0.0, x * ux + z * uz) * (size > 1e-12)
        x, z = x - share * ux, z - share * uz
    return x, z



def turned(twist, heading, shield, flow):
    """Her twist (forward, left, turn; m/s, rad/s) with its flow deflected by the room rows her
    shield held last (`schema.shielded`'s `room`), along the floor at her `heading`; `flow` 1 where
    her twist is her route's (an errand's), 0 a word's own - a walk told into a wall meets it."""
    vx, vy, wz = (float(v) for v in twist)
    room = (shield or {}).get('room') or {'n': (), 'lam': ()}
    sh, ch = math.sin(heading), math.cos(heading)
    x, z = deflected((vx * sh + vy * ch, vx * ch - vy * sh), [(n[0], n[2]) for n in room['n']],
                     [flow * lam for lam in room['lam']])
    return (x * sh + z * ch, x * ch - z * sh, wz)


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


def gap(centre, half, R, at, along, floor, most):
    """The free width across her way at `at` (x, z) going `along` (x, z, a unit): the nearest box
    either side along the line across it, at a height of each box within the high band over `floor`,
    among the boxes (centre, half sizes, rotations) the band meets; a side `most` m at the farthest,
    0 where `at` is within a box (the slab method)."""
    import numpy as np
    up = np.abs(R[:, 1, :])
    reach = (up * half).sum(1)
    meets = (centre[:, 1] - reach - floor <= BAND[1]) & (centre[:, 1] + reach - floor >= BAND[0])
    c, h, r = centre[meets], half[meets], R[meets]
    y = np.clip(c[:, 1], floor + BAND[0], floor + BAND[1])
    o = np.stack([np.full(len(c), at[0]), y, np.full(len(c), at[1])], 1)
    lo_ = np.einsum('kji,kj->ki', r, o - c)
    d = np.einsum('kji,j->ki', r, np.array([along[1], 0.0, -along[0]]))
    flat = np.abs(d) < 1e-12
    with np.errstate(divide='ignore', invalid='ignore'):
        t1, t2 = (-h - lo_) / d, (h - lo_) / d
    inside = np.abs(lo_) <= h
    enter = np.where(flat, np.where(inside, -np.inf, np.inf), np.minimum(t1, t2)).max(1)
    leave = np.where(flat, np.where(inside, np.inf, -np.inf), np.maximum(t1, t2)).min(1)
    hit = enter <= leave
    ahead = np.where(hit & (enter >= 0.0), enter, most).min(initial=most)
    behind = np.where(hit & (leave <= 0.0), -leave, most).min(initial=most)
    within = bool((hit & (enter < 0.0) & (leave > 0.0)).any())
    return 0.0 if within else float(min(ahead, most) + min(behind, most))


def sliced(width, nominal, depth):
    """Her trunk's twist that fits her chest - `nominal` across, `depth` front to back, m - through a
    gap `width` m wide: 0 where she fits square on; else past the twist her depth first widens her to,
    nominal cos t + depth sin t = width; TWIST at most - past it the gap is under her contorted width."""
    import math
    rho, phi = math.hypot(nominal, depth), math.atan2(depth, nominal)
    fit = phi + math.acos(max(-1.0, min(1.0, width / rho)))
    return min(TWIST, fit) * float(width < nominal)


def ahead(boxes, at, heading, speed, floor, margin):
    """Her trunk's twist for the gap her way meets AHEAD_S on: `boxes` the room's (`schema.room`), `at`
    her pelvis (x, z) going `speed` m/s along `heading`, her lower sole at `floor`, each side's
    `margin` her skin's (`schema.MARGIN`)."""
    import math
    sh, ch = math.sin(heading), math.cos(heading)
    reach = max(0.0, speed) * AHEAD_S
    width = gap(boxes['centre'], boxes['half'], boxes['R'], (at[0] + reach * sh, at[1] + reach * ch), (sh, ch),
                floor, NOMINAL)
    return sliced(width - 2.0 * margin, NOMINAL, DEPTH)


def gauged(g, told, s, y_in):
    """(the room's boxes, her trunk's twist) this pass of her session `g`: the boxes her things are
    (`schema.room`), their patches by her rows' weights `told` and her observer; the gauge's twist for
    the gap her route meets - none on a word's twist -, her gaze to be kept against it."""
    from machine import edge, modal, schema
    floor = min(float(f['sole'][1]) for f in s['feet'])
    boxes = schema.room(g.get('things', ()), schema.weighed(told, g['observer']['normal'], float(g['fr'][modal.WEIGHT])),
                        floor, schema.felt(y_in[edge.TOUCH]))
    at = (float(g['est']['pos'][0]), float(g['est']['pos'][2]))
    return boxes, float(g.get('_errand') is not None) * ahead(boxes, at, g['heading'], float(g.get('twist', (0.0,))[0]),
                                                              floor, schema.MARGIN)
