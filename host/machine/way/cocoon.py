"""The robot's braking cocoon on its keys: it stops short of a thing as a let-go key stops it.

    cocoon.blocked(g)          # a pass: whether the robot's stop along its asked way no longer fits short of its things
    tw = cocoon.refused(g, tw)  # a key's twist, its way into a thing the robot's stop cannot spare taken off

proprioceptive_body_schema.lisp's `extrude-velocity-braking-cocoon` on the robot's walk: its things (the
robot's registry's boxes) it may not stand in (`voxels.STAND_M`), a door's or a slope's not (it pushes
a door aside and walks a ramp), each grown by its half width (`figure.WIDE_HALF`: by its hips' alone
the robot's arm went 0.15 m into the bed) - a building's walls and windows (`classes.SHELL`) where its way
runs into their face, less its half width: one brushed past is its touch's and its arms'
yield's (grown, a doorway's post 0.15 m off closed it; left out, the robot walked into a pane at 1.1 m/s and fell);
the robot's stop v T + v^2 / (2 a) along its way - T the text's horizon (`law.schema.HORIZON`), a the
robot's braking (`route.BRAKE`) - against the room to the nearest along it less the text's skin
(`wayfinder.MARGIN`). Blocked, the robot's keys are let go (`machine.process.worker`) and it stands by its own stop;
turning and backing off are its own. The robot's twist held to it outran it - 1.2 m/s to a chair's skin - and a zero
cap threw the robot. On an errand or blind on an open floor (`not (physics.ROOM or g.get('eyes'))`) none: an
errand's dock is the robot's own to reach (`gate-affordance-contact-permeability`), an open floor's unannounced
props its touch's and its situation's.
"""
import math

from machine.body import figure, footprint
from machine.eyes import classes, registry, voxels
from machine.law import schema, se2
from machine.way import furniture, route, wayfinder
from machine.world import physics

#: The things last laid as boxes (`_boxes`): those things and their arrays, without and with doors.
_KEPT: list = [None, None, None, None]
#: The things the robot's clearance was last measured against (`_clear`), the place it was measured at and the
#: clearance there, m.
_FREE: list = [None, (0.0, 0.0), -math.inf]
#: How far off that place the clearance is measured anew, m - a pace's matter alone, any length gives the same
#: answers - and what a clearance is trusted short of, m: rounding's.
_AGAIN, _SLACK = 0.05, 1e-9


def _seen(g):
    """The robot's things for braking: under its eyes, boxes confirmed by stereo (`0.5 * registry.HIT`) before
    routing."""
    if g.get('eyes') and g.get('vision') is not None:
        return g['vision'].boxes(g, 0.5 * registry.HIT)
    return g.get('things')


def _rising(g):
    """Whether the robot rests on a thing (`RISE_OF`) or is still in the rise stunt off it: it is the robot's own to
    stand from."""
    from machine.process import gaits
    cur = se2.doing(g)
    return cur in se2.RISE_OF or (
        g.get('active_entry') in se2.RISE_OF.values() and (cur == 'init_walk' or gaits.in_stunt_transient(g)))


def _deeper(o, d, half, off, pad=None):
    """Whether `d` at `o` inside `half` (or `half + pad`) steps closer to the unexpanded box or its nearer face."""
    import numpy as np
    h0 = np.maximum(half - (figure.WIDE_HALF - off)[:, None], 0.0)
    over, od = (half if pad is None else half + pad) - np.abs(o), o * d
    return (((o - np.clip(o, -h0, h0)) * d).sum(axis=1) < -1e-9) | (
        np.where(over[:, 0] < over[:, 1], od[:, 0], od[:, 1]) < -1e-9)


def clear_of_rise(g):
    """Whether a completed rise has carried the robot's pelvis outside its things' half-width halo (or it steps deeper
    in)."""
    from machine.process import gaits
    cur = se2.doing(g)
    if cur in se2.RISE_OF or cur == 'init_walk' or gaits.in_stunt_transient(g):
        return False
    things = _seen(g)
    if not (physics.ROOM or g.get('eyes')) or not things:
        return True
    import numpy as np
    c, axes, half, off = _boxes(things)
    if not len(c):
        return True
    q = furniture.where(g)
    o = np.einsum('nij,nj->ni', axes, np.array([float(q[0]), float(q[2])]) - c)
    pad = np.where(off > 0.0, 0.0, wayfinder.MARGIN)[:, None]
    inside = (np.abs(o) <= half + pad).all(axis=1)
    if not inside.any():
        return True
    tw = g.get('asked', (0.0, 0.0, 0.0))
    v = math.hypot(tw[0], tw[1])
    if v <= 1e-6:
        return False
    h = float(g['heading'])
    d = axes @ np.array([(tw[0] * math.sin(h) + tw[1] * math.cos(h)) / v,
                         (tw[0] * math.cos(h) - tw[1] * math.sin(h)) / v])
    return bool((inside & _deeper(o, d, half, off, pad)).any())


def _vel_blocked(g):
    """Whether the robot's current horizontal velocity carries it into a thing before it can stop."""
    things = _seen(g)
    if not (physics.ROOM or g.get('eyes')) or g.get('_errand') or not things or _rising(g):
        return False
    vel = g['est']['vel']
    vx, vz = float(vel[0]), float(vel[2])
    vh = math.hypot(vx, vz)
    if vh <= se2.HALT_M_S:
        return False
    need = _stop(vh) + (2.0 * se2.STOP_SPREAD_M if g.get('_cocoon_cut') else 0.0)
    return not _spared(g, need, things) and need >= _room(g, (vx / vh, vz / vh), things)


def _spin_blocked(g, wz, cut=False):
    """Whether spinning in place toward `wz` carries its pelvis into a thing (`footprint.carry`)."""
    things = _seen(g)
    if not (physics.ROOM or g.get('eyes')) or g.get('_errand') or not things or _rising(g):
        return False
    cf, cl = footprint.carry(45.0, math.copysign(1.0, wz))
    carry_m = math.hypot(cf, cl)
    if carry_m <= 1e-6:
        return False
    h = float(g['heading'])
    way = ((cf * math.sin(h) + cl * math.cos(h)) / carry_m, (cf * math.cos(h) - cl * math.sin(h)) / carry_m)
    need = carry_m + (2.0 * se2.STOP_SPREAD_M if cut else 0.0)
    return not _spared(g, need, things) and need >= _room(g, way, things)


def hop_blocked(g):
    """Whether an in-stride leap along the robot's current velocity cannot land and stop short of its things or a closed
    door."""
    things = _seen(g)
    if not (physics.ROOM or g.get('eyes')) or g.get('_errand') or not things or _rising(g):
        return False
    vel = g['est']['vel']
    vx, vz = float(vel[0]), float(vel[2])
    vh = math.hypot(vx, vz)
    q = furniture.where(g)
    room = ahead(_boxes(things, True), (float(q[0]), float(q[2])), (vx / max(vh, 1e-9), vz / max(vh, 1e-9))) - wayfinder.MARGIN
    return vh > se2.HALT_M_S and _stop(vh) + 0.35 * vh >= room


def blocked(g):
    """Whether the robot's stop along its asked way (`g['asked']`), turning arc, or spin carry/momentum no
    longer fits short of its things: never on an errand, already braking or standing, or with none seen."""
    tw = g.get('asked', (0.0, 0.0, 0.0))
    way = _way(g, tw)
    if way is None:
        return (se2.doing(g) not in ('brake', 'stand', 'rest') and abs(float(tw[2])) > 1e-4
                and (_vel_blocked(g) or _spin_blocked(g, float(tw[2]))))
    vel = g['est']['vel']
    v_along = max(0.0, float(vel[0]) * way[0] + float(vel[2]) * way[1])
    side = se2.SIDE_STOP_M if abs(float(tw[1])) > 1e-4 and se2.doing(g) not in ('brake', 'stand', 'rest') else 0.0
    need = max(_stop(v_along), side)
    return not _spared(g, need) and need >= _arc_room(g, tw, v_along)


def refused(g, tw):
    """Twist `tw` (the robot's body's), its way taken off where the robot's stop along it or its turning arc cannot
    fit short of the robot's things; its spin held off while its carry or unsteered momentum runs
    into a thing."""
    way = _way(g, tw)
    v = math.hypot(tw[0], tw[1])
    if way is None:
        return ((0.0, 0.0, 0.0) if abs(float(tw[2])) > 1e-4
                and (_vel_blocked(g) or _spin_blocked(g, float(tw[2]), bool(g.get('_cocoon_cut')))) else tw)
    vel = g['est']['vel']
    v_along = max(0.0, float(vel[0]) * way[0] + float(vel[2]) * way[1])
    v_chk = max(v, v_along)
    need = _stop(v_chk) + (2.0 * se2.STOP_SPREAD_M if g.get('_cocoon_cut') else 0.0)
    if _spared(g, need) or need < _arc_room(g, tw, v_chk):
        return tw
    return (0.0, 0.0, 0.0 if _vel_blocked(g) else tw[2])


def _way(g, tw, dh=0.0):
    """The unit way (x, z) of twist `tw` along the robot's heading turned by `dh`; None outside a room or sight, on an
    errand, rising off a thing, or still."""
    v = math.hypot(tw[0], tw[1])
    if not (physics.ROOM or g.get('eyes')) or g.get('_errand') or v < 1e-6 or not _seen(g) or _rising(g):
        return None
    h = float(g['heading']) + dh
    return (tw[0] * math.sin(h) + tw[1] * math.cos(h)) / v, (tw[0] * math.cos(h) - tw[1] * math.sin(h)) / v


def _arc_room(g, tw, v_ref):
    """Minimum room along the robot's straight or yaw-curved braking horizon."""
    way0 = _way(g, tw)
    if way0 is None:
        return math.inf
    room = _room(g, way0)
    wz = float(tw[2])
    if abs(wz) <= 1e-4:
        return room
    t_arc = schema.HORIZON + 0.5 * v_ref / route.BRAKE
    for frac in (0.5, 1.0):
        w_k = _way(g, tw, frac * wz * t_arc)
        if w_k is not None:
            room = min(room, _room(g, w_k))
    return room


def _stop(v):
    """The robot's stop from `v` m/s, m: the text's horizon's run and its braking's."""
    return v * schema.HORIZON + v * v / (2.0 * route.BRAKE)


def _room(g, way, things=None):
    """The room along `way` from the robot's place to its nearest thing, less the text's skin, m: to a
    thing grown by it half width, to a wall's or a window's face its way runs into less it."""
    q = furniture.where(g)
    boxes = _boxes(_seen(g) if things is None else things)
    return ahead(boxes, (float(q[0]), float(q[2])), way) - wayfinder.MARGIN


def _spared(g, need, things=None):
    """Whether `need` m fit short of the robot's things along every way by its clearance alone (`_clear`): the room
    along a way (`_room`) is then no less, and is not asked."""
    q = furniture.where(g)
    free = _clear(_seen(g) if things is None else things, (float(q[0]), float(q[2])))
    return need + wayfinder.MARGIN + _SLACK < free


def _clear(things, at):
    """A floor under `ahead` from `at` (x, z) along every way, m: the least over the robot's things of a box's gap on
    its own axes less its distance off, as measured at a place and less what the robot has moved off that place
    since. A way enters a box no sooner than the gap - an axis of a box is no longer than one on the floor - and
    the gap closes no faster than the robot moves; within a box the floor is under zero and the room is asked."""
    import numpy as np
    kept, was, free = _FREE
    moved = math.hypot(at[0] - was[0], at[1] - was[1])
    if kept is not things or moved > _AGAIN:
        c, axes, half, off = _boxes(things)
        gap = np.abs(np.einsum('nij,nj->ni', axes, np.asarray(at, float) - c)) - half
        free = float((gap.max(axis=1) - off).min()) if len(c) else math.inf
        _FREE[:] = things, at, free
        return free
    return free - moved


def _boxes(things, doors=False):
    """(centres (n, 2), axes (n, 2, 2), halves (n, 2), each one's distance off (n,)) of `things` the robot may not
    stand in, slopes (and doors unless `doors`) out: a thing grown by its half width, a building's shell
    (`classes.SHELL`) as it is and the robot's half width off its face's distance - laid once a look."""
    import numpy as np
    slot = 2 if doors else 0
    if _KEPT[slot] is not things:
        skip = ('slope',) if doors else ('door', 'slope')
        kept = [t for t in things if t[0] not in skip and t[1][1] + t[2][1] > voxels.STAND_M[0]
                and t[1][1] - t[2][1] < voxels.STAND_M[1]]
        shell = np.array([t[0] in classes.SHELL for t in kept], bool)
        c = np.array([(t[1][0], t[1][2]) for t in kept], float).reshape(-1, 2)
        axes = np.array([((t[3][0][0], t[3][2][0]), (t[3][0][2], t[3][2][2])) for t in kept], float).reshape(-1, 2, 2)
        half = (np.array([(t[2][0], t[2][2]) for t in kept], float).reshape(-1, 2)
                + np.where(shell, 0.0, figure.WIDE_HALF)[:, None])
        _KEPT[slot:slot + 2] = things, (c, axes, half, np.where(shell, figure.WIDE_HALF, 0.0))
    return _KEPT[slot + 1]


def ahead(boxes, at, way):
    """How far from `at` (x, z) along `way` (a unit (x, z)) the nearest of `boxes` (`_boxes`') begins less its
    distance off, m: 0 within one and going deeper, inf none ahead. The division where the way crosses an axis alone
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
    deeper = _deeper(o, d, half, off)
    hit = ((lo <= hi) & (hi >= 0.0) & ~inside) | (inside & deeper)
    return float((np.where(inside, 0.0, np.maximum(lo, 0.0)) - off)[hit].min()) if hit.any() else math.inf
