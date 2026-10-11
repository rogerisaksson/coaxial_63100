"""Her movements' orthopaedic terms as static arrays: which joint makes each, and which way.

    python tools/cores/terms.py --write     # machine/term_rows.py
    laid = terms.rows()                     # what the file holds, laid anew from her figure

Anatomy names a movement by what it does: flexion of the hip brings the knee forward, abduction
takes it from the midline, dorsiflexion raises the toes toward the shin. ANATOMY says that for
each part - a point or an axis of its segment and the way it goes. Which of her joints makes it,
and with which sign, is found: each joint of the segment is turned in her figure
(`figure.frames`) and the one whose motion is the term's is kept. A movement no driven joint of
hers makes is absent. A limb's terms are found on both limbs and must agree - a limb is an
index, its joints' names data. Ranges, deg: the compiled figure's stops, the ankle's rods'
strokes (`linkage`), the whole-body stack's own for her neck and head (`wbc.RANGES`); nothing
stopping it, None. `machine.terms` reads them, tests/test_terms.py holds them to MuJoCo.
"""
import importlib
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from machine import drives, figure, linkage  # noqa: E402
from tools import REPO  # noqa: E402
from tools.cores import written  # noqa: E402

PATH = os.path.join(REPO, 'host', 'machine', 'term_rows.py')
NOTICE = "Her movements' terms as static arrays: written by host/tools/cores/terms.py, not by hand."

#: A joint is turned TURN_DEG to see what it moves; it makes a movement where MAKES of the
#: motion goes the movement's way.
TURN_DEG, MAKES = 2.0, 0.9

AHEAD, BACK, UP, OUT = (0.0, 0.0, 1.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)
DOWN_IT, UP_IT, ALONG_IT = (0.0, -1.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)

#: (part, segment - a limb's by its name less the limb's -, the movement, its opposite - none,
#: it is told toward a side -, what of the segment: its point or its axis, in its frame, the
#: way that goes: ahead, back, up, or out - from the midline; toward limb 0's side where the
#: movement is told toward one). A segment None: she has no such part.
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
    """({segment less its limb's name: segment}, ..) of her limbs - the segments named alike but
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
        raise ValueError('her limbs are not two, one either side: %r' % (sorted(stems),))
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
    for joint, _axis, _sign in next(s for s in figure.SEGMENTS if s[0] == segment)[2]:
        along = sum(a * b for a, b in zip(moved(segment, kind, vector, joint, TURN_DEG), way))
        if abs(along) >= MAKES:
            found.append((joint, math.copysign(1.0, along)))
    if len(found) > 1:
        raise ValueError('%s: two joints make one movement, %r' % (segment, found))
    return found[0] if found else None


def stops():
    """{joint: (low, high) deg} of every joint something stops: the compiled figure's stops, the
    ankle's rods' strokes, the whole-body stack's own ranges."""
    mujoco = importlib.import_module('mujoco')
    from machine import mjcf, wbc
    m = mujoco.MjModel.from_xml_string(mjcf.mjcf())
    out = {j: tuple(round(math.degrees(float(v)), 6) for v in m.joint(j).range)
           for j in figure.JOINTS if m.jnt_limited[m.joint(j).id]}
    roll = linkage.ROLLED['ankle']
    soft = dict(wbc.RANGES, ankle=linkage.RODS['ankle'][1], ankle_roll=(-roll, roll))
    for joint in figure.JOINTS:
        if joint not in out and drives.kind(joint) in soft:
            out[joint] = tuple(float(v) for v in soft[drives.kind(joint)])
    return out


def makers(pair, stem, kind, vector, way):
    """[(joint, sign), ..] of the joints that take a segment's point or axis `way` - one a limb,
    out from the midline its own way; the trunk's one, out toward limb 0's side - or None: she
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
    """(joint kind, sign, low, high) of a movement the joints `found` make alike, or None: they
    have no drive."""
    signs, spans = {s for _j, s in found}, {limits.get(j) for j, _s in found}
    if len(signs) != 1 or len(spans) != 1:
        raise ValueError('a movement her limbs make unlike: %r, %r' % (found, spans))
    if any(drives.passive(j) for j, _s in found):
        return None
    sign, span = signs.pop(), spans.pop()
    if span is None:
        return drives.kind(found[0][0]), sign, None, None
    low, high = sorted(sign * v for v in span)
    return drives.kind(found[0][0]), sign, low, high


def rows():
    """What machine/term_rows.py holds, {name: value}."""
    limits, pair = stops(), limbs()
    terms, toward, absent, joints = {}, [], [], ({}, {})
    for part, stem, name, opposite, kind, vector, way in ANATOMY:
        found = makers(pair, stem, kind, vector, way)
        term = _term(found, limits) if found else None
        if found is None or term is None:
            absent += [(part, n) for n in (name, opposite) if n]
            continue
        joint, sign, low, high = term
        if len(found) == 2:
            for limb, (its, _sign) in zip(joints, found):
                limb[joint] = its
        terms[(part, name)] = term
        if opposite:
            terms[(part, opposite)] = (joint, -sign, None if high is None else -high,
                                       None if low is None else -low)
        else:
            toward.append((part, name))
            if low is not None and high is not None and low != -high:
                raise ValueError('%s %s: a movement toward a side stopped unlike either way' % (part, name))
    return {'TERMS': terms, 'TOWARD': tuple(toward), 'ABSENT': tuple(absent), 'LIMBS': joints}


def render(laid):
    """The module's text."""
    def number(v):
        return 'None' if v is None else '%g' % v

    lines = ['"""%s"""' % NOTICE, '',
             "#: (part, movement): (joint kind, sign, low, high) - the movement its own way is the",
             "#: joint's angle times sign, deg; low and high its amount's stops, None with none. A",
             "#: movement toward a side (TOWARD) is told for limb 0's: times a limb's mirror.",
             'TERMS = {']
    lines += ['    (%r, %r): (%r, %s, %s, %s),' % (part, name, kind, number(sign), number(low), number(high))
              for (part, name), (kind, sign, low, high) in laid['TERMS'].items()]
    lines += ['}', 'TOWARD = (%s)' % ''.join('(%r, %r), ' % t for t in laid['TOWARD']).rstrip(' '),
              '#: Movements anatomy names that no driven joint of hers makes.',
              'ABSENT = (']
    lines += ['    ' + ' '.join('(%r, %r),' % t for t in laid['ABSENT'][k:k + 3])
              for k in range(0, len(laid['ABSENT']), 3)]
    lines += [')', "#: Each limb's joints by kind: limb 0 the one on +x, as `reach_rows.LIMBS`.",
              'LIMBS = (']
    for limb in laid['LIMBS']:
        items = ['%r: %r' % item for item in limb.items()]
        rows = [', '.join(items[k:k + 4]) for k in range(0, len(items), 4)]
        lines += ['    {' + (',\n     '.join(rows)) + '},']
    return '\n'.join(lines + [')']) + '\n'


if __name__ == '__main__':
    if '--write' not in sys.argv[1:]:
        sys.exit(__doc__)
    done = rows()
    written(PATH, render(done))
    print('%d movements written, %d toward a side, %d absent' % (
        len(done['TERMS']), len(done['TOWARD']), len(done['ABSENT'])))
