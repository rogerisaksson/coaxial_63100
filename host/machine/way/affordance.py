"""What the room's things afford her (affordance_funnel_calculus.lisp): a seat, a bed, a chair's back.

    affordance.sits(box, things)     # a seat: its top in the text's window, SEAT_M2 of it clear above
    affordance.lies(box, things)     # a bed: its top under the text's, its long side BED_M or more, clear above
    affordance.backed(box, things)   # a chair: a back of its kind on its edge, no longer, reaching her lumbar

A thing is a box (kind, centre, half sizes, turn, its slot, clear) as her registry holds them
(`machine.eyes.registry`), her eyes' or the stand-in's alike - `clear` whether the space over its top is seen
clear: a seat or a bed is such a surface, a structure's top unknown is neither; the windows are the text's
`evaluate-situational-affordances` kernels on its SDF slabs (`configure-foliated-sdf-bands`), her
lumbar's height over a seat her figure's.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body import figure
from machine.texts import kernel

_KERNELS = {k['affordance']: k for k in kernel.node('evaluate-situational-affordances', 'affordances')['morphological-kernels']}
_SLABS = [s.args['z-window-mm'] for s in kernel.node('configure-foliated-sdf-bands', 'sdf-bands')['slabs']]
#: A seat's top over the floor, m, and the least of it clear above, m^2; a bed's top at most and its long
#: side at least, m; over a seat the mid-pelvic and high-thoracic slabs clear, over a bed the high one, m.
SEAT_M = tuple(1e-3 * z for z in _KERNELS[':sit-on-fixture']['elevation-window-z-mm'])
SEAT_M2 = 1e-6 * _KERNELS[':sit-on-fixture']['min-polygon-area-mm2']
BED_M_WIN = tuple(1e-3 * z for z in _KERNELS[':lie-supine-on-fixture']['elevation-window-z-mm'])
BED_TOP_M = BED_M_WIN[1]
BED_M = 1e-3 * _KERNELS[':lie-supine-on-fixture']['min-major-axis-length-mm']
BED_M2 = BED_M * 2.0 * figure.ASIDE
SEAT_CLEAR, BED_CLEAR = (1e-3 * _SLABS[1][0], 1e-3 * _SLABS[2][1]), (0.0, 1e-3 * _SLABS[2][1])
#: Her lumbar's height over her seat, sat upright, m: the torso's first dorsal point (`figure.BODY`, the
#: observer's `dorsal-supine-continuum`) over her ischial capsules' underside - a seat's back reaching it
#: is a backrest; a stair's riser under it is none.
LUMBAR_M = (next(seg for seg in figure.SEGMENTS if seg[0] == 'torso')[3][1]
            + next(c for c in figure.BODY if c[0] == 'torso')[2][1] - (figure.SEAT_Y - figure.SEAT_R))
#: A rectangle's half extents along any axes reach its circle's radius at most: the rectangle round it on them, so
#: much of that radius its own circle's.
ROOT2 = math.sqrt(2.0)


def top(box):
    """A box's top over the floor, m."""
    return float(box[1][1]) + float(box[2][1])


def bottom(box):
    return float(box[1][1]) - float(box[2][1])


def surface(box):
    """Whether the space over a box's top is seen clear: its word where it has one, a plain box's yes."""
    return len(box) < 6 or float(box[5]) > 0.0


def footprint(box):
    """(its centre (2,), its floor axes (2, 2) a row each, its half sizes along them (2,)) of a box."""
    c, h, R = box[1], box[2], np.asarray(box[3], float).reshape(3, 3)
    return np.array([c[0], c[2]], float), np.array([[R[0][0], R[2][0]], [R[0][2], R[2][2]]]), np.array([h[0], h[2]], float)


def distance(box, at):
    """m from `at` (x, z) to a box's footprint, 0 within it."""
    c, axes, h = footprint(box)
    return float(np.hypot(*np.maximum(np.abs(axes @ (np.asarray(at, float) - c)) - h, 0.0)))


def support(box, heading):
    """The half extent of a box's footprint along `heading` (rad, (sin, cos) along x, z): its edge's
    distance from its centre that way."""
    _c, axes, h = footprint(box)
    return float(np.abs(axes @ np.array([math.sin(heading), math.cos(heading)])) @ h)


def _within(box, other):
    """`other`'s footprint in `box`'s floor frame: (its reach either way along each of box's axes (2, 2))."""
    c, axes, _h = footprint(box)
    oc, oaxes, oh = footprint(other)
    corners = oc + np.array([[sx, sz] for sx in (-1, 1) for sz in (-1, 1)]) * oh @ oaxes
    local = (corners - c) @ axes.T
    return np.column_stack([local.min(axis=0), local.max(axis=0)])


def covered(box, other):
    """The area of `box`'s footprint under `other`'s, m^2 (`other`'s as the rectangle round it in `box`'s
    frame)."""
    _c, _axes, h = footprint(box)
    reach = _within(box, other)
    sides = np.minimum(reach[:, 1], h) - np.maximum(reach[:, 0], -h)
    return float(np.prod(np.maximum(sides, 0.0)))


def clear(box, things, slab):
    """The area of `box`'s top clear of `things` whose bottoms lie in `slab` (lo, hi) over it, m^2: one whose
    footprint's circle misses `box`'s covers none of it - `covered` reads the rectangle round it in `box`'s frame,
    within ROOT2 of its circle -, 0.0 its sum's same bits unread: 87 of 108 over her lodge's beds, their every
    thing's `covered` 5.1 ms a look as a thing moved renews them (`furniture._afforded`), her seats' 10.2 ms."""
    c, _axes, h = footprint(box)
    lo, hi, reach = top(box) + slab[0], top(box) + slab[1], math.hypot(h[0], h[1])
    over = [t for t in things if t is not box and lo <= bottom(t) <= hi and math.hypot(
        float(t[1][0]) - c[0], float(t[1][2]) - c[1]) <= reach + ROOT2 * math.hypot(t[2][0], t[2][2]) + 1e-9]
    return float(4.0 * h[0] * h[1]) - sum(covered(box, t) for t in over)


def sits(box, things):
    """Whether `box` seats her among `things`: its top in SEAT_M, SEAT_M2 of it clear above."""
    return surface(box) and SEAT_M[0] <= top(box) <= SEAT_M[1] and clear(box, things, SEAT_CLEAR) >= SEAT_M2


def lies(box, things):
    """Whether `box` beds her among `things`: its top in BED_M_WIN, its long side BED_M or more, its width
    her shoulders' (`figure.ASIDE`) or more, BED_M2 of it clear above."""
    _c, _axes, h = footprint(box)
    return (surface(box) and BED_M_WIN[0] <= top(box) <= BED_TOP_M and 2.0 * float(h.max()) >= BED_M
            and float(h.min()) >= figure.ASIDE and clear(box, things, BED_CLEAR) >= BED_M2)


def back_of(box, things):
    """The back of a seat `box` among `things`: one of its kind reaching LUMBAR_M over the seat's top, its
    bottom no more than a cell (`voxels.CELL_M`) over that top - held within a cell of it, a chair painted
    whole had none: its back and rear legs one structure off the floor -, at its edge within a cell of its
    footprint: the text's backrest, the slab over its edge occupied (`:verify-high-thoracic-slab-occupancy`), its
    length none of it - a chair's back is one structure with its rear legs, 0.80 m on a 0.50 m seat, a low chair's
    with the wall it stands against, 1.02 m -; of such the nearest its edge, what stands at it - the loft's block a
    cell off the chair's seat was one too; none: None."""
    from machine.eyes.voxels import CELL_M
    _c, _axes, h = footprint(box)
    lumbar = top(box) + LUMBAR_M
    best = None
    for t in things:
        if t is box or t[0] != box[0] or not (top(t) >= lumbar and bottom(t) <= top(box) + CELL_M + 1e-9):
            continue
        reach = _within(box, t)
        gap = math.hypot(*(np.maximum(reach[:, 0] - h, 0.0) + np.maximum(-h - reach[:, 1], 0.0)))
        if gap <= CELL_M + 1e-9 and (best is None or gap < best[0]):
            best = (gap, t)
    return None if best is None else best[1]


def backed(box, things):
    """Whether a seat `box` has a back among `things` (`back_of`)."""
    return back_of(box, things) is not None


def door(box, _things):
    """Whether `box` is a door: its kind's."""
    return box[0] == 'door'


def faced(box, things):
    """The heading a seat `box` faces among `things`, rad (`se2`'s, (sin, cos) along x, z): the outward normal of
    its side away from its back (`back_of`) - the side its back's footprint lies furthest out past, as `faces`
    tells each -, whatever the back's length: a back one with a wall puts its centre metres along it; None without
    a back."""
    back = back_of(box, things)
    if back is None:
        return None
    _c, axes, h = footprint(box)
    mid = _within(box, back).mean(axis=1)
    k = int(np.argmax(np.abs(mid) / np.maximum(h, 1e-9)))
    s = -math.copysign(1.0, float(mid[k]))
    return math.atan2(s * float(axes[k][0]), s * float(axes[k][1]))


def faces(box, _things):
    """The headings of a box's four sides' outward normals, rad: its footprint's axes either way."""
    _c, axes, _h = footprint(box)
    return tuple(math.atan2(s * float(a[0]), s * float(a[1])) for a in axes for s in (1.0, -1.0))


def fronts(box, things):
    """The headings a seat `box` is sat on from, rad: the one it faces (`faced`), any side without a back."""
    f = faced(box, things)
    return faces(box, things) if f is None else (f,)


def sides(box, _things):
    """The headings of a box's long sides' outward normals, rad: a bed's sat on, a door's walked through."""
    _c, axes, h = footprint(box)
    a = axes[int(np.argmin(h))]
    return (math.atan2(float(a[0]), float(a[1])), math.atan2(-float(a[0]), -float(a[1])))


def ends(box, _things):
    """The headings of a box's short sides' outward normals, rad: a bed's ends, sat on to lie along it."""
    _c, axes, h = footprint(box)
    a = axes[int(np.argmax(h))]
    return (math.atan2(float(a[0]), float(a[1])), math.atan2(-float(a[0]), -float(a[1])))