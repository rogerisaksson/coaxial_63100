"""Her braking cocoon on her keys: she stops short of a thing as a let-go key stops her.

    cocoon.blocked(g)          # a pass: whether her stop along her asked way no longer fits short of her things
    tw = cocoon.refused(g, tw)  # a key's twist, its way into a thing her stop cannot spare taken off

proprioceptive_body_schema.lisp's `extrude-velocity-braking-cocoon` on her walk: her things (her registry's boxes)
she may not stand in (`voxels.STAND_M`) - a building's shell's not (`classes.SHELL`: her touch and her arms' yield
hold walls, windows and doors; held off a jamb she stood short of every doorway) -, each grown by her trunk's half width (`figure.HIP_HALF`: her
arms yield on a jamb, grown by them or her planner's radius a doorway closed before her); her stop
v T + v^2 / (2 a) along her way - T the text's horizon (`law.schema.HORIZON`), a her braking (`route.BRAKE`) - against
the room to the nearest along it less the text's skin (`wayfinder.MARGIN`). Blocked, her keys are let go
(`machine.process.worker`) and she stands by her own stop; turning and backing off are hers. Her twist held to it
outran it - 1.2 m/s to a chair's skin - and a zero cap threw her. On an errand none: its dock is hers to reach
(`gate-affordance-contact-permeability`), F's errands as they were.
"""
import math

from machine.body import figure
from machine.eyes import classes, voxels
from machine.law import schema
from machine.way import furniture, route, wayfinder

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
    """The unit way (x, z) of twist `tw` along her heading; None on an errand, for no way or no things."""
    v = math.hypot(tw[0], tw[1])
    if g.get('_errand') or v < 1e-6 or not g.get('things'):
        return None
    h = float(g['heading'])
    return (tw[0] * math.sin(h) + tw[1] * math.cos(h)) / v, (tw[0] * math.cos(h) - tw[1] * math.sin(h)) / v


def _stop(v):
    """Her stop from `v` m/s, m: the text's horizon's run and her braking's."""
    return v * schema.HORIZON + v * v / (2.0 * route.BRAKE)


def _room(g, way):
    """The room along `way` from her place to her nearest thing, less the text's skin, m."""
    q = furniture.where(g)
    return ahead(_boxes(g['things']), (float(q[0]), float(q[2])), way) - wayfinder.MARGIN


def _boxes(things):
    """(centres (n, 2), axes (n, 2, 2), halves (n, 2)) of `things` she may not stand in, grown by her radius: laid
    once a look."""
    import numpy as np
    if _KEPT[0] is not things:
        kept = [t for t in things if t[0] not in classes.SHELL and t[1][1] + t[2][1] > voxels.STAND_M[0]
                and t[1][1] - t[2][1] < voxels.STAND_M[1]]
        c = np.array([(t[1][0], t[1][2]) for t in kept], float).reshape(-1, 2)
        axes = np.array([((t[3][0][0], t[3][2][0]), (t[3][0][2], t[3][2][2])) for t in kept], float).reshape(-1, 2, 2)
        half = np.array([(t[2][0], t[2][2]) for t in kept], float).reshape(-1, 2) + figure.HIP_HALF
        _KEPT[:] = things, (c, axes, half)
    return _KEPT[1]


def ahead(boxes, at, way):
    """How far from `at` (x, z) along `way` (a unit (x, z)) the nearest of `boxes` begins, m: 0 within one and going
    deeper, inf none ahead."""
    import numpy as np
    c, axes, half = boxes
    if not len(c):
        return math.inf
    o = np.einsum('nij,nj->ni', axes, np.asarray(at, float) - c)
    d = axes @ np.asarray(way, float)
    flat, within = np.abs(d) > 1e-12, np.abs(o) <= half
    with np.errstate(divide='ignore', invalid='ignore'):
        t1, t2 = (-half - o) / d, (half - o) / d
    lo = np.where(flat, np.minimum(t1, t2), np.where(within, -np.inf, np.inf)).max(axis=1)
    hi = np.where(flat, np.maximum(t1, t2), np.where(within, np.inf, -np.inf)).min(axis=1)
    inside = within.all(axis=1)
    k = np.argmax(np.abs(o) / half, axis=1)
    deeper = np.take_along_axis(o * d, k[:, None], axis=1)[:, 0] < 0.0
    hit = ((lo <= hi) & (hi >= 0.0) & ~inside) | (inside & deeper)
    return float(np.where(inside, 0.0, np.maximum(lo, 0.0))[hit].min()) if hit.any() else math.inf
