"""The robot's movements by their orthopaedic names, from static arrays or the build's figure.

    kind, sign, low, high = terms.find('knee', 'flexion')   # deg; None: the robot has it not
    name = terms.joint(limb, kind)                          # a limb's joint; the trunk's, its own
    terms.TOWARD, terms.ABSENT                              # told toward a side; named, no joint

A part and a movement name a joint kind and the sign its angle takes the movement's own way;
its amount's stops with it, None where nothing stops it. A movement toward a side is told for
limb 0's and takes a limb's mirror (`reach_rows.LIMBS`) for another's. A limb is an index:
which joint of the robot's own is its knee is data, read from `term_rows` on `mechatronics.STOCK`
and compiled from the robot's figure on another build (`tools/cores/terms.py`).
"""
import math
import os

from machine.body import drives, figure, mechatronics

#: A joint is turned TURN_DEG to see what it moves; it makes a movement where MAKES of the
#: motion goes the movement's way.
TURN_DEG, MAKES = 2.0, 0.9
AHEAD, BACK, UP, OUT = (0.0, 0.0, 1.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)
DOWN_IT, UP_IT, ALONG_IT = (0.0, -1.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)

#: (part, segment - a limb's by its name less the limb's -, the movement, its opposite - none,
#: it is told toward a side -, what of the segment: its point or its axis, in its frame, the
#: way that goes: ahead, back, up, or out - from the midline; toward limb 0's side where the
#: movement is told toward one). A segment None: the robot has no such part.
ANATOMY = (
    ('hip', 'thigh', 'flexion', 'extension', 'point', DOWN_IT, AHEAD),
    ('hip', 'thigh', 'abduction', 'adduction', 'point', DOWN_IT, OUT),
    ('hip', 'thigh', 'lateral-rotation', 'medial-rotation', 'axis', ALONG_IT, OUT),
    ('knee', 'shank', 'flexion', 'extension', 'point', DOWN_IT, BACK),
    ('ankle', 'foot', 'dorsiflexion', 'plantarflexion', 'point', ALONG_IT, UP),
    ('foot', 'foot', 'eversion', 'inversion', 'axis', DOWN_IT, OUT),
    ('toes', 'toes', 'extension', 'flexion', 'point', ALONG_IT, UP),
    ('trunk', 'torso', 'flexion', 'extension', 'point', UP_IT, AHEAD),
    ('trunk', 'torso', 'lateral-flexion', None, 'point', UP_IT, OUT),
    ('trunk', 'torso', 'rotation', None, 'axis', ALONG_IT, OUT),
    ('neck', 'neck', 'flexion', 'extension', 'point', UP_IT, AHEAD),
    ('neck', 'neck', 'lateral-flexion', None, 'point', UP_IT, OUT),
    ('head', 'head', 'rotation', None, 'axis', ALONG_IT, OUT),
    ('shoulder', 'upper_arm', 'flexion', 'extension', 'point', DOWN_IT, AHEAD),
    ('shoulder', 'upper_arm', 'abduction', 'adduction', 'point', DOWN_IT, OUT),
    ('shoulder', 'upper_arm', 'lateral-rotation', 'medial-rotation', 'axis', ALONG_IT, OUT),
    ('shoulder', None, 'elevation', 'depression', None, None, None),
    ('shoulder', None, 'protraction', 'retraction', None, None, None),
    ('elbow', 'forearm', 'flexion', 'extension', 'point', DOWN_IT, AHEAD),
    ('forearm', 'forearm', 'supination', 'pronation', 'axis', ALONG_IT, OUT),
    ('wrist', 'hand', 'flexion', 'extension', 'point', DOWN_IT, AHEAD),
    ('wrist', 'hand', 'radial-deviation', 'ulnar-deviation', 'point', DOWN_IT, OUT),
    ('fingers', 'fingers', 'flexion', 'extension', 'point', DOWN_IT, AHEAD),
    ('thumb', None, 'opposition', 'reposition', None, None, None),
)


def limbs():
    """({segment less its limb's name: segment}, ..) of the robot's limbs - the segments named alike but
    for what comes before the first '_' -, the limb on +x first: limb 0, as `reach.LIMBS`."""
    names = [s[0] for s in figure.SEGMENTS]
    stems = {}
    for name in names:
        head, _, stem = name.partition('_')
        if stem and sum(n.partition('_')[2] == stem for n in names) == 2:
            stems.setdefault(head, {})[stem] = name
    placed = figure.frames({}, (0.0, 0.0, 0.0), figure.IDENT)
    out = sorted(stems.values(), key=lambda limb: -placed[next(iter(limb.values()))][0][0])
    if len(out) != 2 or not placed[next(iter(out[0].values()))][0][0] > 0.0:
        raise ValueError('its limbs are not two, one either side: %r' % (sorted(stems),))
    return tuple(out)


def moved(segment, kind, vector, joint, sign_deg):
    """What a point or an axis of `segment` does, world, the `joint` turned `sign_deg`: its
    motion over the turn's angle, a unit where it swings square to its joint."""
    there = []
    for deg in (0.0, sign_deg):
        at, turn = figure.frames({joint: deg}, (0.0, 0.0, 0.0), figure.IDENT)[segment]
        tip = figure.apply(turn, vector)
        there.append(figure.add(at, tip) if kind == 'point' else tip)
    return tuple((b - a) / math.radians(abs(sign_deg)) for a, b in zip(*there))


def made(segment, kind, vector, way):
    """(joint, sign) of the joint of `segment` that takes its point or axis `way`, world: the
    one MAKES of whose motion goes that way, or None."""
    found = []
    for jnt, _axis, _sign in next(s for s in figure.SEGMENTS if s[0] == segment)[2]:
        along = sum(a * b for a, b in zip(moved(segment, kind, vector, jnt, TURN_DEG), way))
        if abs(along) >= MAKES:
            found.append((jnt, math.copysign(1.0, along)))
    if len(found) > 1:
        raise ValueError('%s: two joints make one movement, %r' % (segment, found))
    return found[0] if found else None


def makers(pair, stem, kind, vector, way):
    """[(joint, sign), ..] of the joints that take a segment's point or axis `way` - one a limb,
    out from the midline its own way; the trunk's one, out toward limb 0's side - or None: the robot
    has no such segment, or no joint of it goes that way."""
    if stem is None or kind is None or vector is None or way is None:
        return None
    if stem in pair[0]:
        found = [made(limb[stem], kind, vector, (way[0] * mirror, way[1], way[2]))
                 for limb, mirror in zip(pair, (1.0, -1.0))]
    else:
        found = [made(stem, kind, vector, way)]
    joints = [f for f in found if f is not None]
    return joints if len(joints) == len(found) else None


def _term(found, limits):
    signs, spans = {s for _j, s in found}, {limits.get(j) for j, _s in found}
    if len(signs) != 1 or len(spans) != 1:
        raise ValueError('a movement the robot\'s limbs make unlike: %r, %r' % (found, spans))
    if any(drives.passive(j) for j, _s in found):
        return None
    sign, span = signs.pop(), spans.pop()
    if span is None:
        return drives.kind(found[0][0]), sign, None, None
    low, high = sorted(round(sign * v, 6) for v in span)
    return drives.kind(found[0][0]), sign, low, high


def rows(limits=None):
    """{TERMS, TOWARD, ABSENT, LIMBS} laid from the robot's figure and `limits` (`drives.TRAVEL`)."""
    if limits is None:
        limits = {j: tuple(round(float(v), 6) for v in drives.TRAVEL[drives.kind(j)])
                  for j in figure.JOINTS if drives.kind(j) in drives.TRAVEL}
    pair = limbs()
    found_terms, toward, absent, joints = {}, [], [], ({}, {})
    for part, stem, name, opposite, kind, vector, way in ANATOMY:
        found = makers(pair, stem, kind, vector, way)
        term = _term(found, limits) if found else None
        if found is None or term is None:
            absent += [(part, n) for n in (name, opposite) if n]
            continue
        j_kind, sign, low, high = term
        if len(found) == 2:
            for limb, (its, _sign) in zip(joints, found):
                limb[j_kind] = its
        found_terms[(part, name)] = term
        if opposite:
            found_terms[(part, opposite)] = (j_kind, -sign, None if high is None else -high,
                                             None if low is None else -low)
        else:
            toward.append((part, name))
            if low is not None and high is not None and low != -high:
                raise ValueError('%s %s: a movement toward a side stopped unlike either way' % (part, name))
    return {'TERMS': found_terms, 'TOWARD': tuple(toward), 'ABSENT': tuple(absent), 'LIMBS': joints}


def _compiled():
    def build_it():
        laid = rows()
        return {**laid, 'TERMS': tuple(laid['TERMS'].items())}
    srcs = mechatronics.robot_sources() + (
        __file__, mechatronics.__file__, figure.__file__, drives.__file__,
        os.path.join(os.path.dirname(__file__), 'linkage.py'))
    got = mechatronics.compiled('terms_' + mechatronics.ROBOT, srcs, build_it)
    return dict(got['TERMS']), got['TOWARD'], got['ABSENT'], got['LIMBS']


if mechatronics.ROBOT == mechatronics.STOCK:
    from machine.body.term_rows import ABSENT, LIMBS, TERMS, TOWARD
else:
    TERMS, TOWARD, ABSENT, LIMBS = _compiled()


def find(part, term):
    """(joint kind, sign, low, high) of a movement of a part, deg - or None: no driven joint of
    the robot's own makes it."""
    return TERMS.get((part, term))


def joint(limb, kind):
    """The joint of that kind of a limb's index; a kind no limb has - the trunk's, the neck's -
    is its own joint."""
    return LIMBS[limb].get(kind, kind)
