"""The furniture her eyes tell her of: the thing a verb goes to nearest a place, its top and middle.

Her things are her registry's boxes (`g['things']`, `machine.registry`: her eyes' under `physics.SIGHT`,
the world's props standing in for them without), what each affords her the text's (`machine.affordance`):
a chair a seat with a back, the nearest seat without one; a bed what she can lie on; the door its kind.
"""
import math

from machine import affordance

#: What `verb` goes to among her things: whether a box affords it, whether it is the thing itself, and
#: the heading it faces (None, as the room lays it).
GOES = {'sit_chair': (affordance.sits, affordance.backed, affordance.faced),
        'lie_bed': (affordance.lies, lambda _b, _t: True, lambda _b, _t: None),
        'exit_door': (lambda b, _t: b[0] == 'door', lambda _b, _t: True, lambda _b, _t: None)}
_NONE = (lambda _b, _t: False, lambda _b, _t: False, lambda _b, _t: None)


def _things(g, things):
    """`things` (hers, `g['things']`, without) as boxes: kind, centre, half sizes, turn at the least."""
    return [p for p in (g['things'] if things is None else things)
            if len(p) >= 4 and len(p[1]) >= 3 and hasattr(p[2], '__len__') and len(p[2]) >= 3]


def _found(g, verb, at=None, thing=None, things=None):
    """(its distance from `at`, its edge's from her, the thing - kind, centre, half sizes, turn, slot) of
    what `verb` goes to nearest `at` (x, z) - her without one - among `things` (hers, `g['things']`,
    without), the thing itself before what else affords it; the thing of slot `thing` where she holds
    one still (an errand's, chosen when it was seen whole); None with none seen."""
    affords, itself, _faces = GOES.get(verb, _NONE)
    gx, gz = float(g['world'].data.qpos[0]), float(g['world'].data.qpos[2])
    ax, az = at if at is not None else (gx, gz)
    things = _things(g, things)
    held = [p for p in things if thing is not None and len(p) > 4 and p[4] == thing]
    hits = [
        (math.hypot(float(p[1][0]) - ax, float(p[1][2]) - az),
         max(0.0, math.hypot(float(p[1][0]) - gx, float(p[1][2]) - gz) - min(float(p[2][0]), float(p[2][2]))), p,
         itself(p, things))
        for p in (held or [p for p in things if affords(p, things)])
    ]
    return min(hits, default=None, key=lambda h: (not h[3], h[0], h[1]))


def nearest(g, verb, at=None, where=(0.0, 0.0), things=None):
    """`(dist_edge, px, pz, top, thing, faced, box)` of the thing `verb` goes to nearest `at` (x, z) - her
    without one - among `things` (hers without): its edge's distance from her, its centre, its top's
    height, its slot, the heading it faces (None as the room lays it), its box; `where`, no top, no slot,
    no facing and no box with none seen."""
    hit = _found(g, verb, at, things=things)
    if hit is None:
        return math.inf, where[0], where[1], None, None, None, None
    edge, p = hit[1], hit[2]
    return (edge, float(p[1][0]), float(p[1][2]), float(p[1][1]) + float(p[2][1]), (p[4] if len(p) > 4 else None),
            GOES.get(verb, _NONE)[2](p, _things(g, things)), p)


def seen(g, stunt, at=None, thing=None):
    """(its top, the face she backs into behind her soles' middle along her way, its middle to her
    left of theirs, m; that face's turn off square to her way, rad; its depth across that face, m)
    of the furniture stunt `stunt` is on nearest `at` (her without one), the thing of slot `thing`
    where given - a rise's that of the stunt it rises from -; None where she sees none. The face is the side whose outward normal lies most
    along her way: read as the box's reach along her way, a bed turned 0.6 rad to her was its far
    corner, -0.45 m where its long side lay 0.15 m behind her. Her soles about her root as her
    estimate has them, her root where `_found` reads her: a look is taken from her, her odometry's
    drift none of it - an errand drifted her estimate 3 cm, the seat seen 2.2 cm deep of where it
    stood, a thigh met its edge and she fell."""
    from machine import edge, maneuver, modal
    hit = _found(g, maneuver.PLAYED.get(stunt, (None, None))[1] or stunt, at, thing)
    if hit is None:
        return None
    _kind, (px, py, pz), half, turn = hit[2][:4]
    root = g['world'].data.qpos[:3] - g['est']['pos']
    mid, yaw = g['y'][modal.SOLE].reshape(2, 3).mean(axis=0) + root, edge.yaw(g['sense'])
    way, off = (math.sin(yaw), math.cos(yaw)), (px - float(mid[0]), pz - float(mid[2]))
    dots = [way[0] * turn[0][j] + way[1] * turn[2][j] for j in (0, 2)]
    k = int(abs(dots[1]) > abs(dots[0]))
    j, cos = 2 * k, abs(dots[k])
    face = (-math.copysign(1.0, dots[k]) * (off[0] * turn[0][j] + off[1] * turn[2][j]) - half[j]) / cos
    return (py + half[1], face, off[0] * way[1] - off[1] * way[0],
            math.remainder(math.atan2(turn[0][2], turn[2][2]) - yaw, 0.5 * math.pi), 2.0 * half[j])
