"""The furniture her eyes tell her of: the thing a verb goes to nearest a place, its top and middle.

Her things are her registry's boxes (`g['things']`, `machine.eyes.registry`: her eyes' under `physics.SIGHT`,
the world's props standing in for them without), what each affords her the text's (`machine.way.affordance`):
a chair a seat with a back, the nearest seat without one; a bed what she can lie on; the door its kind.
"""
import math
import operator

from machine.way import affordance

#: What `verb` goes to among her things: whether a box affords it, whether it is the thing itself, and
#: the headings of the faces she docks at (`route.docked`): a bed's ends and sides, her lie the edge's (`first`).
GOES = {'sit_chair': (affordance.sits, affordance.backed, affordance.fronts),
        'lie_bed': (affordance.lies, lambda _b, _t: True, affordance.faces),
        'leg_bed': (affordance.lies, lambda _b, _t: True, affordance.faces),
        'lie_floor': (affordance.lies, lambda _b, _t: True, affordance.sides),
        'exit_door': (affordance.door, lambda _b, _t: True, affordance.sides)}
_NONE = (lambda _b, _t: False, lambda _b, _t: False, lambda _b, _t: ())
#: A face's kind as a patch names the one it docks her at first (`maneuver.face`): a bed's ends, its sides.
KINDS = {':short': affordance.ends, ':long': affordance.sides}


def first(verb, box, things, heading):
    """0 where `heading` (rad) is a face of `box` among `things` that `verb`'s patch docks her at first
    (`maneuver.face`, KINDS) - any face where it names none -, 1 one she docks at only where none of those
    is hers to reach."""
    from machine.texts import maneuver
    kind = KINDS.get(maneuver.face(verb) or '') if verb in maneuver.PLAYED else None
    return int(kind is not None and all(abs(math.remainder(heading - h, 2.0 * math.pi)) > 1e-9
                                        for h in kind(box, things)))


def _things(g, things):
    """`things` (hers, `g['things']`, without) as boxes: kind, centre, half sizes, turn at the least."""
    return [p for p in (g['things'] if things is None else things) if _boxlike(p)]


def _boxlike(p):
    return len(p) >= 4 and len(p[1]) >= 3 and hasattr(p[2], '__len__') and len(p[2]) >= 3


def _afforded(g, verb, things=None):
    """(her boxes of `things` - hers, `g['things']`, without -, [(box, whether it is the thing itself)] of those
    `verb` goes to, in their order): told anew only as her things change - the very ones last told, or a look's of the
    same bits (`_alike`): the last told in its places -, kept on `g` till then, hers and those given each their own.
    Each pass of a bed's move found the bed again among the lodge's 64 things (a box `affordance.lies` reads every other
    thing over it), 0.86 ms of her 1 kHz pass while it played: lying down on the bed and rising from it she ran at half
    her pace; told anew each look, a third of it. Kept one a verb, her errand's look for a thing - hers and her
    registry's every slot, both each pass (`words._seen`, `words._glimpsed`) - told it anew at every call: 346 us of
    19 558 in her bed errand, 15 % of its wall."""
    source, key = (g['things'] if things is None else things), (verb, things is None)
    kept = g.setdefault('_afforded', {}).get(key)
    if kept is None or not (kept[0] is source and len(kept[1]) == len(source) and all(map(operator.is_, kept[1], source))):
        if kept is not None and _alike(kept[0], source):
            index, chosen = kept[2], kept[3]
        else:
            affords, itself, _faces = GOES.get(verb, _NONE)
            index = [k for k, p in enumerate(source) if _boxlike(p)]
            boxes = [source[k] for k in index]
            chosen = [(j, itself(p, boxes)) for j, p in enumerate(boxes) if affords(p, boxes)]
        boxes = [source[k] for k in index]
        kept = g['_afforded'][key] = (source, tuple(source), index, chosen, boxes, [(boxes[j], own) for j, own in chosen], {})
    return kept[4], kept[5]


def _faced(g, verb, things, box):
    """The headings of the faces `verb` docks her at on `box` (GOES), one of the boxes last told of `things`
    (`_afforded`): told once a telling, with what affords it - a seat's back sought among all her things at every
    read of her errand's thing (`nearest`), 100 us a third of its passes."""
    _source, _was, _index, _chosen, boxes, _afford, told = g['_afforded'][(verb, things is None)]
    if id(box) not in told:
        told[id(box)] = GOES.get(verb, _NONE)[2](box, boxes)
    return told[id(box)]


def _alike(was, things):
    """Whether `things` - a look's (`registry.Things`) - are `was`'s of the same bits: their boxes' arrays byte for
    byte, each one's kind, slot, clear and swell equal; False for any other kind of things."""
    arrays, before = getattr(things, 'arrays', None), getattr(was, 'arrays', None)
    if arrays is None or before is None or len(was) != len(things):
        return False
    return (all(a.tobytes() == b.tobytes() for a, b in zip(arrays, before))
            and all(p[0] == q[0] and p[4:] == q[4:] for p, q in zip(things, was)))


def _found(g, verb, at=None, thing=None, things=None):
    """(its distance from `at`, its edge's from her, the thing - kind, centre, half sizes, turn, slot) of
    what `verb` goes to nearest `at` (x, z) - her without one - among `things` (hers, `g['things']`,
    without), the thing itself before what else affords it; the thing of slot `thing` where she holds
    one still (an errand's, chosen when it was seen whole); None with none seen. What affords it told as her things
    change (`_afforded`); its edge's distance (`affordance.distance`) only where it decides - the nearest's, and those
    as near to the hair -, the first least by (not itself, distance, edge's) as among all."""
    gx, gz = float(g['world'].data.qpos[0]), float(g['world'].data.qpos[2])
    ax, az = at if at is not None else (gx, gz)
    things, afforded = _afforded(g, verb, things)
    held = [p for p in things if len(p) > 4 and p[4] == thing] if thing is not None else []
    itself = GOES.get(verb, _NONE)[1]
    near = [(math.hypot(float(p[1][0]) - ax, float(p[1][2]) - az), p, own)
            for p, own in ([(p, itself(p, things)) for p in held] if held else afforded)]
    if not near:
        return None
    least = min((not own, d) for d, _p, own in near)
    hits = [(d, affordance.distance(p, (gx, gz)), p, own) for d, p, own in near if (not own, d) == least]
    return min(hits, key=lambda h: (not h[3], h[0], h[1]))


def nearest(g, verb, at=None, where=(0.0, 0.0), things=None):
    """`(dist_edge, px, pz, top, thing, faces, box)` of the thing `verb` goes to nearest `at` (x, z) - her
    without one - among `things` (hers without): its edge's distance from her, its centre, its top's
    height, its slot, the headings of the faces she docks at (GOES), its box; `where`, no top, no slot,
    no faces and no box with none seen."""
    hit = _found(g, verb, at, things=things)
    if hit is None:
        return math.inf, where[0], where[1], None, None, None, None
    edge, p = hit[1], hit[2]
    return (edge, float(p[1][0]), float(p[1][2]), float(p[1][1]) + float(p[2][1]), (p[4] if len(p) > 4 else None),
            _faced(g, verb, things, p), p)


def lain(g, verb='lie_bed'):
    """(her root from the middle of the thing `verb` goes to nearest her along its length, m - its sign the box's -,
    that half length, m; her root's speed, m/s) of her things, (0.0, 0.0, speed) with none seen: her root where a
    look takes it (`_found`), her place in the room as her eyes tell it - not her dead reckoning, whose estimate drifted
    0.28 m along a bed as she lay down on it and stroked up it."""
    data = g['world'].data
    speed = float(math.sqrt(sum(float(v) ** 2 for v in data.qvel[:3])))
    hit = _found(g, verb)
    if hit is None:
        return 0.0, 0.0, speed
    _kind, (px, _py, pz), half, turn = hit[2][:4]
    j = 0 if half[0] > half[2] else 2
    along = (float(data.qpos[0]) - px) * turn[0][j] + (float(data.qpos[2]) - pz) * turn[2][j]
    return float(along), float(half[j]), speed


def seen(g, stunt, at=None, thing=None):
    """(its top, the face she backs into behind her soles' middle along her way, its middle to her
    left of theirs, m; that face's turn off square to her way, rad; its depth across that face and its
    width along it, m)
    of the furniture stunt `stunt` is on nearest `at` (her without one), the thing of slot `thing`
    where given - a rise's that of the stunt it rises from -; None where she sees none. The face is the side whose outward normal lies most
    along her way: read as the box's reach along her way, a bed turned 0.6 rad to her was its far
    corner, -0.45 m where its long side lay 0.15 m behind her. Her soles about her root as her
    estimate has them, her root where `_found` reads her: a look is taken from her, her odometry's
    drift none of it - an errand drifted her estimate 3 cm, the seat seen 2.2 cm deep of where it
    stood, a thigh met its edge and she fell."""
    from machine.senses import edge
    from machine.texts import maneuver
    from machine.law import modal
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
            math.remainder(math.atan2(turn[0][2], turn[2][2]) - yaw, 0.5 * math.pi), 2.0 * half[j], 2.0 * half[2 - j])
