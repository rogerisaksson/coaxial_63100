"""The furniture her eyes tell her of: the thing a verb goes to nearest a place, its top and middle.

Her stereo is taken as calibrated as made: a thing's class, centre and size as her looks tell them
(`things.told`); till her looks feed this, the world's props stand for that answer, read as a
look's would be (`_match`: a kind and its dimensions).
"""
import math


def _match():
    from machine import room
    return {
        'sit_chair': lambda k, py, sx, sz: (
            k in ('chair', 'armchair', 'wood')
            and abs(py - room.SEAT_H) < room.SEAT[0] and 0.5 * room.SEAT[0] <= sx <= sz <= 2.0 * room.SEAT[0]),
        'lie_bed': lambda k, py, sx, _sz: k == 'bed' and py >= 0.5 * room.BED[1] and sx >= 0.5 * room.BED[0],
        'exit_door': lambda k, _py, _sx, _sz: k == 'door',
    }


def _found(g, verb, at=None):
    """(its distance from `at`, its edge's from her, the thing - kind, centre, half sizes, turn) of
    what `verb` goes to nearest `at` (x, z) - her without one -; None with none seen."""
    match = _match().get(verb, lambda *_a: False)
    gx, gz = float(g['world'].data.qpos[0]), float(g['world'].data.qpos[2])
    ax, az = at if at is not None else (gx, gz)
    hits = [
        (math.hypot(float(p[1][0]) - ax, float(p[1][2]) - az),
         max(0.0, math.hypot(float(p[1][0]) - gx, float(p[1][2]) - gz) - min(float(p[2][0]), float(p[2][2]))), p)
        for p in g['world'].props()
        if len(p) >= 4 and len(p[1]) >= 3 and hasattr(p[2], '__len__') and len(p[2]) >= 3
        and match(p[0], float(p[1][1]), min(float(p[2][0]), float(p[2][2])), max(float(p[2][0]), float(p[2][2])))
    ]
    return min(hits, default=None, key=lambda h: h[:2])


def nearest(g, verb, at=None, where=(0.0, 0.0)):
    """`(dist_edge, px, pz, top)` of the thing `verb` goes to nearest `at` (x, z) - her without one -:
    its edge's distance from her, its centre, its top's height; `where` and no top with none seen."""
    hit = _found(g, verb, at)
    if hit is None:
        return math.inf, where[0], where[1], None
    edge, p = hit[1], hit[2]
    return edge, float(p[1][0]), float(p[1][2]), float(p[1][1]) + float(p[2][1])


def seen(g, stunt, at=None):
    """(its top, its near edge behind her soles' middle along her way, its middle to her left of
    theirs, m; its edge's turn off square to her way, rad) of the furniture stunt `stunt` is on
    nearest `at` (her without one) - a rise's that of the stunt it rises from -; None where she sees
    none. Her soles about her root as her estimate has them, her root where `_found` reads her: a
    look is taken from her, her odometry's drift none of it - an errand drifted her estimate 3 cm,
    the seat seen 2.2 cm deep of where it stood, a thigh met its edge and she fell."""
    from machine import edge, maneuver, modal
    hit = _found(g, maneuver.PLAYED.get(stunt, (None, None))[1] or stunt, at)
    if hit is None:
        return None
    _kind, (px, py, pz), half, turn = hit[2][:4]
    root = g['world'].data.qpos[:3] - g['est']['pos']
    mid, yaw = g['y'][modal.SOLE].reshape(2, 3).mean(axis=0) + root, edge.yaw(g['sense'])
    way, off = (math.sin(yaw), math.cos(yaw)), (px - float(mid[0]), pz - float(mid[2]))
    reach = sum(abs(way[0] * turn[0][j] + way[1] * turn[2][j]) * half[j] for j in range(3))
    return (py + half[1], -(off[0] * way[0] + off[1] * way[1]) - reach, off[0] * way[1] - off[1] * way[0],
            math.remainder(math.atan2(turn[0][2], turn[2][2]) - yaw, 0.5 * math.pi))
