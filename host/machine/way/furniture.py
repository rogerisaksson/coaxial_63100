"""The furniture the robot sees: the thing a verb goes to nearest a place, its top and middle.

The robot's things are its registry's boxes (`g['things']`, `machine.eyes.registry`: its eyes'
under `physics.SIGHT`, the world's props standing in for them without), what each affords it the text's
(`machine.way.affordance`): a chair a seat with a back, the nearest seat without one; a bed what it can
lie on; the door its kind.
"""
import math
import operator

from machine import libm
from machine.way import affordance

#: What `verb` goes to among the robot's things: whether a box affords it, whether it is the thing itself, and
#: the headings of the faces the robot docks at (`route.docked`): a bed's ends and sides, its lie the
#: edge's (`first`).
GOES = {'sit_chair': (affordance.sits, affordance.backed, affordance.fronts),
        'lie_bed': (affordance.lies, lambda _b, _t: True, affordance.faces),
        'leg_bed': (affordance.lies, lambda _b, _t: True, affordance.faces),
        'lie_floor': (affordance.lies, lambda _b, _t: True, affordance.sides),
        'exit_door': (affordance.door, lambda _b, _t: True, affordance.sides)}
_NONE = (lambda _b, _t: False, lambda _b, _t: False, lambda _b, _t: ())
#: A face's kind as a patch names the one it docks the robot at first (`maneuver.face`): a bed's ends, its
#: sides.
KINDS = {':short': affordance.ends, ':long': affordance.sides}


def first(verb, box, things, heading):
    """0 where `heading` (rad) is a face of `box` among `things` that `verb`'s patch docks the robot at first
    (`maneuver.face`, KINDS) - any face where it names none -, 1 one the robot docks at only where none of
    those is its own to reach."""
    from machine.texts import maneuver
    kind = KINDS.get(maneuver.face(verb) or '') if verb in maneuver.PLAYED else None
    return int(kind is not None and all(abs(math.remainder(heading - h, 2.0 * math.pi)) > 1e-9
                                        for h in kind(box, things)))


def _boxlike(p):
    return len(p) >= 4 and len(p[1]) >= 3 and hasattr(p[2], '__len__') and len(p[2]) >= 3


def _afforded(g, verb, things=None):
    """(the robot's boxes of `things` - its own, `g['things']`, without -, [(box, whether it is the
    thing itself)] of those `verb` goes to, in their order): told anew only as the robot's things change - the
    very ones last told, or a look's of the same bits (`_alike`): the last told in its places -, kept on `g`
    till then, the robot's own and those given each their own. Each pass of a bed's move found the bed again
    among the lodge's 64 things (a box `affordance.lies` reads every other thing over it), 0.86 ms of the
    robot's 1 kHz pass while it played: lying down on the bed and rising from it the robot ran at half the
    robot's pace; told anew each look, a third of it. Kept one a verb, the robot's errand's look for a thing -
    the robot's own and its registry's every slot, both each pass (`words._seen`, `words._glimpsed`) -
    told it anew at every call: 346 us of 19 558 in the robot's bed errand, 15 % of its wall."""
    source, key = (g['things'] if things is None else things), (verb, things is None)
    kept = g.setdefault('_afforded', {}).get(key)
    if kept is None or not _same(kept, source):
        if kept is not None and _alike(kept[0], source):
            index, chosen = kept[2], kept[3]
            laid, boxes = _laid(g, source)
            boxes = boxes if laid == index else _boxes(source, index)
        else:
            affords, itself, _faces = GOES.get(verb, _NONE)
            index, boxes = _laid(g, source)
            near, was = _nearby(g, kept, source, index), dict(kept[3]) if kept is not None else {}
            chosen = [(j, itself(p, boxes) if near is None or near[j] else was[j]) for j, p in enumerate(boxes)
                      if (affords(p, boxes) if near is None or near[j] else j in was)]
        kept = g['_afforded'][key] = (source, tuple(source), index, chosen, boxes, [(boxes[j], own) for j, own in chosen], {})
    return kept[4], kept[5]


def _same(kept, things):
    """Whether `kept` (things, a tuple of them as they were) holds the very list `things`, each thing in it the same."""
    return kept[0] is things and len(kept[1]) == len(things) and all(map(operator.is_, kept[1], things))


def _laid(g, source):
    """(index, boxes) of the things `source` (`_boxlike`, `_boxes`), laid once a list of things: each verb's telling of
    one look takes them - the robot's errand's, its peeks' three, its move's bed (`lain`) laid them anew each. A look's
    things with their arrays beside them (`registry.Things`) are boxes each, laid so: sought one by one, a bed creeping
    under the robot had them 32 us of each look."""
    laid = g.get('_boxes_laid')
    if laid is None or not _same(laid, source):
        index = (list(range(len(source))) if getattr(source, 'arrays', None) is not None
                 else [k for k, p in enumerate(source) if _boxlike(p)])
        laid = g['_boxes_laid'] = (source, tuple(source), index, _boxes(source, index))
    return laid[2], laid[3]


def _nearby(g, kept, source, index):
    """`_near` of `kept` and `source` at `index`, told once a pair of looks: each verb told last of the same look
    takes it."""
    if kept is None:
        return None
    memo = g.get('_nearby')
    if memo is None or memo[2] is not kept[0] or memo[3] != kept[2] or memo[4] != index or not _same(memo, source):
        memo = g['_nearby'] = (source, tuple(source), kept[0], kept[2], index, _near(kept, source, index))
    return memo[5]


def _boxes(things, index):
    """`things`' boxes at `index`, their arrays beside them where `things` has them (`affordance.Boxes`, a look's
    `registry.Things`) - what affords a verb read of all of them over a box at once (`affordance.clear`); all of them,
    its own arrays, read alone."""
    whole = len(index) == len(things)
    boxes = affordance.Boxes(things if whole else (things[k] for k in index))
    arrays = getattr(things, 'arrays', None)
    if arrays is not None and len(arrays[0]) == len(things):
        boxes.arrays = arrays if whole else tuple(a[list(index)] for a in arrays)
    return boxes


def _near(kept, things, index):
    """(n,) whether each of `things`' boxes at `index` is to be told anew by a verb (`GOES`) as `things` - a look's
    (`registry.Things`) - follow those `kept` told: it moved, or one that moved stands near it, its circle and the
    other's within one cell (`voxels.CELL_M`) - all a box's seat, bed and back read of another (`affordance.clear`'s
    circles, `back_of`'s cell); the rest as told before. None where it cannot be told: another count, no arrays.
    Each look of a bed awake under the robot's renewed all, 70 boxes' affordances 23 % of its lying."""
    import numpy as np
    from machine.eyes.voxels import CELL_M
    if kept is None or kept[2] != index:
        return None
    arrays, before = getattr(things, 'arrays', None), getattr(kept[0], 'arrays', None)
    if arrays is None or before is None or len(arrays[0]) != len(before[0]):
        return None
    n = len(index)
    at = slice(None) if n == len(arrays[0]) else np.asarray(index, int)     # all of them read in place
    now, was = [a[at].reshape(n, -1) for a in arrays], [a[at].reshape(n, -1) for a in before]
    # a thing the very tuple it was (its look's rows that moved alone told anew, `vision.ROWS`) is as it was
    told = np.array([things[k] is not kept[0][k]
                     and (things[k][0] != kept[0][k][0] or things[k][5:6] != kept[0][k][5:6]) for k in index], bool)
    core = affordance.cored()
    return (near_twin if core is None else core.near)(now, was, told, CELL_M, affordance.ROOT2)


def near_twin(now, was, told, cell, root2):
    """`tools.cores.afford.near` in numpy: (n,) whether each row of `now` (centres, half sizes, rotations) is told
    anew as it follows `was` - it moved, its numbers or (`told`) its word, or a moved row's circle and its own within
    `cell` m, either look's, `root2` a rectangle's reach over its circle."""
    import numpy as np
    moved = np.any(np.hstack(now) != np.hstack(was), axis=1) | told
    if not moved.any():
        return moved
    near = moved.copy()
    for c, h in ((now[0], now[1]), (was[0], was[1])):
        plan, r = c[:, [0, 2]], libm.hypot(h[:, 0], h[:, 2])
        gap = libm.hypot(*(plan[:, None, :] - plan[None, moved, :]).transpose(2, 0, 1))
        near |= (gap <= r[:, None] + root2 * r[None, moved] + cell + 1e-9).any(axis=1)
    return near


def _faced(g, verb, things, box):
    """The headings of the faces `verb` docks the robot at on `box` (GOES), one of the boxes last told of
    `things` (`_afforded`): told once a telling, with what affords it - a seat's back sought among all the
    robot's things at every read of its errand's thing (`nearest`), 100 us a third of its passes."""
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


def where(g):
    """The robot's pelvis (3,) in its things' frame: its invariant EKF's under its eyes, the world's
    without."""
    return g['inekf']['p'] if g.get('eyes') else g['world'].data.qpos[:3]


def floor(g):
    """The robot's lower sole's height (m) in its things' frame (`where`)."""
    return min(float(f['sole'][1]) for f in g['s']['feet']) - float(g['est']['pos'][1]) + float(where(g)[1])


def _found(g, verb, at=None, thing=None, things=None, edge=True):
    """(its distance from `at`, its edge's from the robot, the thing - kind, centre, half sizes, turn, slot) of
    what `verb` goes to nearest `at` (x, z) - it without one - among `things` (its own, `g['things']`,
    without), the thing itself before what else affords it; the thing of slot `thing` where the robot holds
    one still (an errand's, chosen when it was seen whole); None with none seen. What affords it told as the robot's
    things change (`_afforded`); its edge's distance (`affordance.distance`) only where it decides - the nearest's,
    and those as near to the hair -, the first least by (not itself, distance, edge's) as among all; not `edge`, its
    edge's distance None where one alone is least (`lain`'s and `seen`'s, which read the thing alone: the edge's
    distance was 21 us of the robot's every pass sat on a chair or lain on a bed)."""
    from machine.law import stride
    q = where(g)
    gx, gz = float(q[0]), float(q[2])
    ax, az = at if at is not None else (gx, gz)
    things, afforded = _afforded(g, verb, things)
    held = [(p, own) for p, own in afforded if len(p) > 4 and p[4] == thing
            and (at is None or affordance.distance(p, at) <= stride.STRIDE_M)] if thing is not None else []
    near = [(math.hypot(float(p[1][0]) - ax, float(p[1][2]) - az),
             not held and at is not None and affordance.distance(p, at) <= stride.STRIDE_M, p, own)
            for p, own in (held or afforded)]
    if not near:
        return None
    least = min((not on_at, not own, d) for d, on_at, _p, own in near)
    tied = [(d, p, own) for d, on_at, p, own in near if (not on_at, not own, d) == least]
    if not edge and len(tied) == 1:
        return tied[0][0], None, tied[0][1], tied[0][2]
    hits = [(d, affordance.distance(p, (gx, gz)), p, own) for d, p, own in tied]
    return min(hits, key=lambda h: (not h[3], h[0], h[1]))


def nearest(g, verb, at=None, where=(0.0, 0.0), things=None, thing=None):
    """`(dist_edge, px, pz, top, thing, faces, box)` of the thing `verb` goes to nearest `at` (x, z) - the
    robot without one - among `things` (its own without), the thing of slot `thing` where given: its
    edge's distance from the robot, its centre, its top's height, its slot, the headings of the faces the
    robot docks at (GOES), its box; `where`, no top, no slot, no faces and no box with none seen."""
    hit = _found(g, verb, at, thing=thing, things=things)
    if hit is None:
        return math.inf, where[0], where[1], None, None, None, None
    edge, p = float(hit[1] or 0.0), hit[2]
    return (edge, float(p[1][0]), float(p[1][2]), float(p[1][1]) + float(p[2][1]), (p[4] if len(p) > 4 else None),
            _faced(g, verb, things, p), p)


def slotted(g, verb, at=None, things=None):
    """Whether the thing `verb` goes to nearest `at` among `things` (`nearest`) is a slot of the robot's registry's:
    told of what affords it alone where each of them is one, or none is - the nearest sought (`_found`) only between
    both. The robot's errand's glimpse (`words._glimpsed`) asked the nearest's place, top and faces each pass for
    this one word: 42 us of its door errand's pass, its look held."""
    of = {len(p) > 4 and p[4] is not None for p, _own in _afforded(g, verb, things)[1]}
    return of.pop() if len(of) == 1 else bool(of) and nearest(g, verb, at, things=things)[4] is not None


def lain(g, verb='lie_bed'):
    """(the robot's root from the middle of the thing `verb` goes to nearest the robot along its axis, m -
    its sign the box's -, that half length, m; the robot's root's speed, m/s) of its things, (0.0,
    0.0, speed) with none seen: for `sit_chair`, the box's axis lies along the robot's heading (`seen`), for
    `lie_bed`, along its long side. The root sits where a look takes it (`_found`), as the robot's eyes
    tell it, not the robot's dead reckoning, whose estimate drifted 0.28 m along a bed as it lay."""
    q, data = where(g), g['world'].data
    speed = float(math.sqrt(sum(libm.sq(float(v)) for v in data.qvel[:3])))
    hit = _found(g, verb, edge=False)
    if hit is None:
        return 0.0, 0.0, speed
    _kind, (px, _py, pz), half, turn = hit[2][:4]
    if verb == 'sit_chair':
        from machine.senses import edge
        yaw = edge.yaw(g['sense'])
        way = (libm.sin(yaw), libm.cos(yaw))
        dots = [way[0] * turn[0][k] + way[1] * turn[2][k] for k in (0, 2)]
        j = 2 * int(abs(dots[1]) > abs(dots[0]))
    else:
        j = 0 if half[0] > half[2] else 2
    along = (float(q[0]) - px) * turn[0][j] + (float(q[2]) - pz) * turn[2][j]
    return float(along), float(half[j]), speed


def seen(g, stunt, at=None, thing=None, things=None):
    """(its top, the face the robot backs into behind its soles' middle along its way, its
    middle to the robot's left of theirs, m; that face's turn off square to its way, rad; its depth
    across that face and its width along it, m) of the furniture stunt `stunt` is on nearest `at` (the robot
    without one) among `things` (its own without), the thing of slot `thing` where given - a rise's that of the
    stunt it rises from -; None where the robot sees none. The face is the side whose outward normal lies most
    along the robot's way: read as the box's reach along its way, a bed turned 0.6 rad to it was its far corner,
    -0.45 m where its long side lay 0.15 m behind the robot. The robot's soles about its root as its estimate has
    them, its root where `_found` reads it: a look is taken from it, the robot's odometry's drift none of it - an
    errand drifted the robot's estimate 3 cm, the seat seen 2.2 cm deep of where it stood, a thigh met its edge
    and the robot fell."""
    from machine.senses import edge
    from machine.texts import maneuver
    from machine.law import modal
    hit = _found(g, maneuver.PLAYED.get(stunt, (None, None))[1] or stunt, at, thing, things=things, edge=False)
    if hit is None:
        return None
    _kind, (px, py, pz), half, turn = hit[2][:4]
    root = where(g) - g['est']['pos']
    mid, yaw = g['y'][modal.SOLE].reshape(2, 3).mean(axis=0) + root, edge.yaw(g['sense'])
    way, off = (libm.sin(yaw), libm.cos(yaw)), (px - float(mid[0]), pz - float(mid[2]))
    dots = [way[0] * turn[0][j] + way[1] * turn[2][j] for j in (0, 2)]
    k = int(abs(dots[1]) > abs(dots[0]))
    j, cos = 2 * k, abs(dots[k])
    face = (-math.copysign(1.0, dots[k]) * (off[0] * turn[0][j] + off[1] * turn[2][j]) - half[j]) / cos
    return (py + half[1], face, off[0] * way[1] - off[1] * way[0],
            math.remainder(libm.arctan2(turn[0][2], turn[2][2]) - yaw, 0.5 * math.pi), 2.0 * half[j], 2.0 * half[2 - j])


def at_hand(g, stunt, verb, at=None, thing=None):
    """`(seen, edge, reach, slot)` of the furniture stunt `stunt` plays on nearest `at` (slot `thing` where
    given), the robot looking up to `registry.TAKEN` where unseen or past `reach`, else reading positive-salience
    live slots behind the robot under `g['eyes']`."""
    from machine.eyes import registry
    from machine.law import stride
    from machine.process import sprung
    from machine.texts import maneuver
    from machine.way import dock
    reach_of = lambda s: stride.STRIDE_M + dock.standoff(stunt, None if s is None else float(s[0]), g['suite'])
    if thing is None and maneuver.PLAYED[stunt][1] is not None:
        sat = g.get('sat_thing')
        if sat is not None:
            near_sat = nearest(g, verb, at, thing=sat)
            if float(near_sat[0] or 0.0) <= stride.STRIDE_M + dock.standoff(stunt, near_sat[3], g['suite']):
                thing = sat
    s, near = seen(g, stunt, at, thing), nearest(g, verb, at, thing=thing)
    for _ in range(int(math.ceil(registry.TAKEN / registry.HIT))):
        if s is not None and float(near[0] or 0.0) <= reach_of(s):
            break
        sprung.looked(g)
        s, near = seen(g, stunt, at, thing), nearest(g, verb, at, thing=thing)
    if g.get('eyes') and (s is None or float(near[0] or 0.0) > reach_of(s)):
        live = g['vision'].boxes(g, 0.0)
        s_l, near_l = seen(g, stunt, at, thing, things=live), nearest(g, verb, at, things=live, thing=thing)
        if s_l is not None and float(near_l[0] or 0.0) <= reach_of(s_l):
            s, near = s_l, near_l
    return s, float(near[0] or 0.0), reach_of(s), near[4]
