"""Her braking cocoon on her keys: she stops short of a thing as a let-go key stops her.

    cocoon.blocked(g)          # a pass: whether her stop along her asked way no longer fits short of her things
    tw = cocoon.refused(g, tw)  # a key's twist, its way into a thing her stop cannot spare taken off

proprioceptive_body_schema.lisp's `extrude-velocity-braking-cocoon` on her walk: her things (her registry's boxes)
she may not stand in (`voxels.STAND_M`), a door's or a slope's not (she pushes a door aside and walks a ramp), each
grown by her half width (`figure.WIDE_HALF`: by her hips' alone her arm went 0.15 m into the bed) - a building's
walls and windows (`classes.SHELL`) where her way runs into their face, less her half width: one brushed past is her
touch's and her arms' yield's (grown, a doorway's post 0.15 m off closed it; left out, she walked into a pane at
1.1 m/s and fell); her stop v T + v^2 / (2 a) along her way - T the text's horizon (`law.schema.HORIZON`), a her
braking (`route.BRAKE`) - against the room to the nearest along it less the text's skin (`wayfinder.MARGIN`).
Blocked, her keys are let go (`machine.process.worker`) and she stands by her own stop; turning and backing off are
hers. Her twist held to it outran it - 1.2 m/s to a chair's skin - and a zero cap threw her. On an errand or blind
on an open floor (`not (physics.ROOM or g.get('eyes'))`) none: an errand's dock is hers to reach
(`gate-affordance-contact-permeability`), an open floor's unannounced props her touch's and her situation's.
"""
import math

from machine.body import figure
from machine.eyes import classes, voxels
from machine.law import schema
from machine.way import furniture, route, wayfinder
from machine.world import physics

#: The things last laid as boxes (`_boxes`): those things and their arrays.
_KEPT: list = [None, None]


def blocked(g):
    """Whether her stop along her asked way (`g['asked']`) from her speed along it no longer fits short of her things:
    never on an errand, standing or with none seen."""
    tw = g.get('asked', (0.0, 0.0, 0.0))
    way = _way(g, tw)
    if way is None:
        return False
    vel = g['est']['vel']
    return _stop(max(0.0, float(vel[0]) * way[0] + float(vel[2]) * way[1])) >= _room(g, way)


def refused(g, tw):
    """Twist `tw` (her body's), its way taken off where her stop at its own speed cannot fit short of her things."""
    way = _way(g, tw)
    if way is None or _stop(math.hypot(tw[0], tw[1])) < _room(g, way):
        return tw
    return (0.0, 0.0, tw[2])


def _way(g, tw):
    """The unit way (x, z) of twist `tw` along her heading; None outside a room or sight, on an errand, or still."""
    v = math.hypot(tw[0], tw[1])
    if not (physics.ROOM or g.get('eyes')) or g.get('_errand') or v < 1e-6 or not g.get('things'):
        return None
    h = float(g['heading'])
    return (tw[0] * math.sin(h) + tw[1] * math.cos(h)) / v, (tw[0] * math.cos(h) - tw[1] * math.sin(h)) / v


def _stop(v):
    """Her stop from `v` m/s, m: the text's horizon's run and her braking's."""
    return v * schema.HORIZON + v * v / (2.0 * route.BRAKE)


def _room(g, way):
    """The room along `way` from her place to her nearest thing, less the text's skin, m: to a thing grown by her
    half width, to a wall's or a window's face her way runs into less it."""
    q = furniture.where(g)
    return ahead(_boxes(g['things']), (float(q[0]), float(q[2])), way) - wayfinder.MARGIN


def _boxes(things):
    """(centres (n, 2), axes (n, 2, 2), halves (n, 2), each one's distance off (n,)) of `things` she may not stand
    in, doors and slopes out: a thing grown by her half width, a building's shell (`classes.SHELL`) as it is and her
    half width off its face's distance - both in one pass (`ahead`), as two 139 us a pass of her keys, as one 67 -:
    laid once a look."""
    import numpy as np
    if _KEPT[0] is not things:
        kept = [t for t in things if t[0] not in ('door', 'slope') and t[1][1] + t[2][1] > voxels.STAND_M[0]
                and t[1][1] - t[2][1] < voxels.STAND_M[1]]
        shell = np.array([t[0] in classes.SHELL for t in kept], bool)
        c = np.array([(t[1][0], t[1][2]) for t in kept], float).reshape(-1, 2)
        axes = np.array([((t[3][0][0], t[3][2][0]), (t[3][0][2], t[3][2][2])) for t in kept], float).reshape(-1, 2, 2)
        half = (np.array([(t[2][0], t[2][2]) for t in kept], float).reshape(-1, 2)
                + np.where(shell, 0.0, figure.WIDE_HALF)[:, None])
        _KEPT[:] = things, (c, axes, half, np.where(shell, figure.WIDE_HALF, 0.0))
    return _KEPT[1]


def ahead(boxes, at, way):
    """How far from `at` (x, z) along `way` (a unit (x, z)) the nearest of `boxes` (`_boxes`') begins less its
    distance off, m: 0 within one and going deeper, inf none ahead. Its division where its way crosses an axis alone
    (`where`): no warnings to hush, each a context of numpy's."""
    import numpy as np
    c, axes, half, off = boxes
    if not len(c):
        return math.inf
    o = np.einsum('nij,nj->ni', axes, np.asarray(at, float) - c)
    d = axes @ np.asarray(way, float)
    flat, within = np.abs(d) > 1e-12, np.abs(o) <= half
    t1 = np.divide(-half - o, d, out=np.zeros_like(o), where=flat)
    t2 = np.divide(half - o, d, out=np.zeros_like(o), where=flat)
    lo = np.where(flat, np.minimum(t1, t2), np.where(within, -np.inf, np.inf)).max(axis=1)
    hi = np.where(flat, np.maximum(t1, t2), np.where(within, np.inf, -np.inf)).min(axis=1)
    inside = within.all(axis=1)
    far, od = np.abs(o) / half, o * d
    deeper = np.where(far[:, 1] > far[:, 0], od[:, 1], od[:, 0]) < 0.0
    hit = ((lo <= hi) & (hi >= 0.0) & ~inside) | (inside & deeper)
    return float((np.where(inside, 0.0, np.maximum(lo, 0.0)) - off)[hit].min()) if hit.any() else math.inf
