"""What the room's things afford her (affordance_funnel_calculus.lisp): a seat, a bed, a chair's back.

    affordance.sits(box, things)     # a seat: its top in the text's window, SEAT_M2 of it clear above
    affordance.lies(box, things)     # a bed: its top under the text's, its long side BED_M or more, clear above
    affordance.backed(box, things)   # a chair: a back of its kind on its edge, no longer, reaching her lumbar

A thing is a box (kind, centre, half sizes, turn, its slot, clear) as her registry holds them
(`machine.registry`), her eyes' or the stand-in's alike - `clear` whether the space over its top is seen
clear: a seat or a bed is such a surface, a structure's top unknown is neither; the windows are the text's
`evaluate-situational-affordances` kernels on its SDF slabs (`configure-foliated-sdf-bands`), her
lumbar's height over a seat her figure's.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import figure, kernel

_KERNELS = {k['affordance']: k for k in kernel.node('evaluate-situational-affordances', 'affordances')['morphological-kernels']}
_SLABS = [s.args['z-window-mm'] for s in kernel.node('configure-foliated-sdf-bands', 'sdf-bands')['slabs']]
#: A seat's top over the floor, m, and the least of it clear above, m^2; a bed's top at most and its long
#: side at least, m; over a seat the mid-pelvic and high-thoracic slabs clear, over a bed the high one, m.
SEAT_M = tuple(1e-3 * z for z in _KERNELS[':sit-on-fixture']['elevation-window-z-mm'])
SEAT_M2 = 1e-6 * _KERNELS[':sit-on-fixture']['min-polygon-area-mm2']
BED_TOP_M = 1e-3 * _KERNELS[':lie-supine-on-fixture']['elevation-window-z-mm'][1]
BED_M = 1e-3 * _KERNELS[':lie-supine-on-fixture']['min-major-axis-length-mm']
SEAT_CLEAR, BED_CLEAR = (1e-3 * _SLABS[1][0], 1e-3 * _SLABS[2][1]), (1e-3 * _SLABS[2][0], 1e-3 * _SLABS[2][1])
#: Her lumbar's height over her seat, sat upright, m: the torso's first dorsal point (`figure.BODY`, the
#: observer's `dorsal-supine-continuum`) over her ischial capsules' underside - a seat's back reaching it
#: is a backrest; a stair's riser under it is none.
LUMBAR_M = (next(seg for seg in figure.SEGMENTS if seg[0] == 'torso')[3][1]
            + next(c for c in figure.BODY if c[0] == 'torso')[2][1] - (figure.SEAT_Y - figure.SEAT_R))


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
    """The area of `box`'s top clear of `things` whose bottoms lie in `slab` (lo, hi) over it, m^2."""
    _c, _axes, h = footprint(box)
    over = [t for t in things if t is not box and top(box) + slab[0] <= bottom(t) <= top(box) + slab[1]]
    return float(4.0 * h[0] * h[1]) - sum(covered(box, t) for t in over)


def sits(box, things):
    """Whether `box` seats her among `things`: its top in SEAT_M, SEAT_M2 of it clear above."""
    return surface(box) and SEAT_M[0] <= top(box) <= SEAT_M[1] and clear(box, things, SEAT_CLEAR) >= SEAT_M2


def lies(box, things):
    """Whether `box` beds her among `things`: its top under BED_TOP_M, its long side BED_M or more, nothing
    over it in the high slab."""
    _c, _axes, h = footprint(box)
    return (surface(box) and top(box) <= BED_TOP_M and 2.0 * float(h.max()) >= BED_M
            and clear(box, things, BED_CLEAR) >= 4.0 * float(h[0] * h[1]) - 1e-9)


def back_of(box, things):
    """The back of a seat `box` among `things`: one of its kind standing on the seat - its bottom within
    a cell (`voxels.CELL_M`) of the seat's top, at its edge within a cell of its footprint, no longer
    than the seat's side it stands along, a cell over - reaching LUMBAR_M over the top; a wall behind
    a bench or a stair beside a landing stands on the floor, none: None."""
    from machine.voxels import CELL_M
    _c, _axes, h = footprint(box)
    lumbar = top(box) + LUMBAR_M
    for t in things:
        if t is box or t[0] != box[0] or not (top(t) >= lumbar and abs(bottom(t) - top(box)) <= CELL_M + 1e-9):
            continue
        reach = _within(box, t)
        gap = np.maximum(reach[:, 0] - h, 0.0) + np.maximum(-h - reach[:, 1], 0.0)
        along = int(np.argmax(reach[:, 1] - reach[:, 0]))
        if math.hypot(*gap) <= CELL_M + 1e-9 and 2.0 * max(float(t[2][0]), float(t[2][2])) <= 2.0 * h[along] + 2.0 * CELL_M + 1e-9:
            return t
    return None


def backed(box, things):
    """Whether a seat `box` has a back among `things` (`back_of`)."""
    return back_of(box, things) is not None


def faced(box, things):
    """The heading a seat `box` faces among `things`, rad (`se2`'s, (sin, cos) along x, z): from its
    back's centre through its own; None without a back."""
    back = back_of(box, things)
    if back is None:
        return None
    return math.atan2(float(box[1][0]) - float(back[1][0]), float(box[1][2]) - float(back[1][2]))
