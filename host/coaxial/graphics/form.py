"""The robot's form: a mannequin's body from beside a figure, laid on its skeleton, cut by segment.

A whole body stood arms down (`tools/sim/sewn.py --body`: thinned, y up, +z ahead, its left
+x) is what the robot's shell is made of where one lies beside the figure it wears. The
body's joints are found on it (`joints`): where its legs meet and where each arm meets its
trunk - the heights its mesh, taken from the floor up, first holds them together at -, its
arms' line, its bust, its neck. The body is laid on the robot's (`fitted`): its heights joint
to joint, its trunk as wide as the robot's joints stand apart, its depth and its limbs' girth
its own, its arms along the robot's as they hang. The body's triangles go to the segments
they move with: a surface the robot's guards are laid on (`guards`), and its hands. The body's
head and feet are left out: the robot wears a figure's head and its own shoes. The mesh is
never the tree's.

    parts = form.worn(likeness.FOUND)      # [(name, parent, offset, mesh)]: 'form_' and a segment
"""
import glob
import os

from coaxial.graphics import wavefront
from coaxial.graphics.fall import BUST, extent, slices
from coaxial.graphics.likeness import ELBOW, HIP_UP, WRIST
from coaxial.graphics.panels import SHELL
from machine.body import figure

#: A piece of a body is one a SHARE of its corners or more: a leg, an arm - no finger. A neck's
#: rim NECK_M off its narrowest, its jaw JAW_M: where the shoulders end and the head begins.
#: An arm's share of a corner grows over BLEND_M under its armpit and across its own radius.
SHARE, NECK_M, JAW_M, BLEND_M = 0.02, 0.035, 0.012, 0.06


def beside(like):
    """Where the body lies that the figure at `like` is formed on: COAXIAL_FORM's file ('none'
    for none), else the first OBJ under a formed/ in a folder beside the figure's; None where
    there is none."""
    said = os.environ.get('COAXIAL_FORM')
    if said is not None:
        return said if os.path.isfile(said) else None
    found = sorted(glob.glob(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(like))), '*', 'formed', '*.obj')))
    return found[0] if found else None


def _met(v, t):
    """[(height, corners)] ascending: a body's mesh taken from the floor up, a triangle at its
    highest corner's height - each time two of its pieces, SHARE of its corners or more each,
    come to be one: the height they meet at and the corners the lesser held. A body's legs
    meet at its crotch, an arm and its trunk at an armpit."""
    from coaxial.model.blocks import numpy as np
    of, held, out = list(range(len(v))), {k: [k] for k in range(len(v))}, []

    def root(k):
        while of[k] != k:
            of[k] = of[of[k]]
            k = of[k]
        return k
    tops = v[t][:, :, 1].max(axis=1)
    order = np.argsort(tops)
    for (a, b, c), top in zip(t[order].tolist(), tops[order].tolist()):
        for p, q in ((a, b), (b, c)):
            p, q = root(p), root(q)
            if p != q:
                p, q = (q, p) if len(held[p]) < len(held[q]) else (p, q)
                if len(held[q]) >= SHARE * len(v):
                    out.append((top, np.array(held[q])))
                of[q] = p
                held[p] += held.pop(q)
    return out


def joints(v, t):
    """A body's joints found on its corners `v` and triangles `t`, stood arms down, m: its crown;
    its crotch, hip, knee and ankle (y), its leg's axis at the hip and at the ankle (x, z); its
    bust (y); its left arm's shoulder, elbow, wrist and fingertip (x, y, z each) and its radius,
    its armpit (y), its trunk's side under it (x) and each arm's corners; its neck - where it
    leaves the shoulders, its jaw (y), its middle (z)."""
    from coaxial.model.blocks import numpy as np
    crown, met = float(v[:, 1].max()), _met(v, t)
    crotch, arms = met[0][0], sorted(met[1:3], key=lambda piece: -float(v[piece[1], 0].mean()))
    armpit, own = float(min(y for y, _ids in arms)), v[arms[0][1]]         # the lower: under it both arms are their own
    in_arm = np.zeros(len(v), bool)
    in_arm[np.concatenate([ids for _y, ids in arms])] = True
    # The arm's line: the way its corners lie longest, down it; its middle a slice at a time.
    mid = own.mean(axis=0)
    way = np.linalg.svd(own - mid)[2][0]
    way = way * (-1.0 if way[1] > 0.0 else 1.0)
    along = (own - mid) @ way
    at = np.arange(along.min(), along.max(), 0.02)
    seen = [k for k, a in enumerate(at) if (np.abs(along - a) < 0.02).any()]
    line = np.array([own[np.abs(along - at[k]) < 0.02].mean(axis=0) for k in seen])

    def on(a):
        return np.array([np.interp(a, at[seen], line[:, k]) for k in range(3)])
    top, tip = (armpit - mid[1]) / way[1], float(along.max())
    girth = float(np.median(np.linalg.norm(np.cross(own[along < top + 0.08] - mid, way), axis=1)))
    # The shoulder's joint: up the upper arm's own line from its armpit - the whole arm's leans
    # with its forearm, and stood the joint 44 mm further back -, a radius under
    # the shoulder's top over it.
    head, bend = on(top), on(top + ELBOW * (tip - top))
    rise = (head - bend) / np.linalg.norm(head - bend)
    over = v[np.hypot(v[:, 0] - head[0], v[:, 2] - head[2]) < girth, 1].max()
    shoulder = head + rise * (over - girth - head[1]) / rise[1]
    leg = v[(v[:, 0] > 0.0) & (v[:, 1] < crotch) & ~in_arm]
    ankle = min(((np.ptp(s[:, 0]), y) for y, s in slices(leg, 1, 0.03 * crown, 0.2 * crown) if len(s)))[1]

    def axis(y):
        s = leg[np.abs(leg[:, 1] - y) < 0.01]
        return (float(s[:, 0].min() + s[:, 0].max()) / 2.0, float(s[:, 2].min() + s[:, 2].max()) / 2.0)
    hip, trunk = float(crotch + HIP_UP * crown), v[~in_arm]
    bust = max(np.arange(hip + 0.1, shoulder[1] - girth, 0.01), key=lambda y: (extent(trunk, y, 0.01) or (0.0, -1.0))[1])
    rows = [(y, *got) for y in np.arange(shoulder[1] + 0.02, crown - 0.1, 0.01) for got in [extent(trunk, y, 0.01)] if got]
    narrow = min(rows, key=lambda row: row[1])
    base = next(y for y, wide, _f, _b in rows if wide < narrow[1] + NECK_M)
    jaw = next((y for y, wide, _f, _b in rows if y > narrow[0] and wide > narrow[1] + JAW_M), narrow[0] + 0.03)
    return {'crown': crown, 'crotch': float(crotch), 'hip': hip, 'knee': (hip + float(ankle)) / 2.0, 'ankle': float(ankle),
            'leg': axis(crotch - 0.04), 'foot': axis(ankle), 'bust': float(bust), 'armpit': armpit, 'girth': girth,
            'side': (extent(trunk, armpit - 0.02, 0.015) or (shoulder[0] - girth,))[0],
            'arm': (shoulder, on(top + ELBOW * (tip - top)), on(top + WRIST * (tip - top)), on(tip)), 'arms': in_arm,
            'neck': (float(base), float(jaw), (narrow[2] + narrow[3]) / 2.0)}


def _turn(a, b):
    """The least turn that lays the unit way `a` along `b`."""
    from coaxial.model.blocks import numpy as np
    v, c = np.cross(a, b), float(a @ b)
    x = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + x + x @ x / (1.0 + c)


def fitted(v, j):
    """(corners in the robot's pelvis frame at rest, each corner's place along its arm past the
    shoulder - m of the robot's, -1 on the trunk -, the share of its arm's turns each takes):
    the body's `v` at its joints `j` laid on the robot's. The body's heights go joint to joint -
    ankle, knee, hip, bust, shoulder, neck, jaw -; its trunk is as wide as the robot's hips and
    shoulders stand apart, its depth its own; a leg stands about the robot's hip, no wider than
    the robot's; an arm goes along the robot's from shoulder to elbow to wrist, what stands off
    its line turned as its bone is. A mannequin 1.75 m tall has the robot's girths as it is: by
    the trunk's width's scale its bust stood 180 mm ahead."""
    from coaxial.model.blocks import numpy as np
    at = {seg: np.array(place) for seg, (place, _turn_) in figure.frames({}, (0.0, 0.0, 0.0), figure.IDENT).items()}
    hip, knee, ankle = (at['left_' + seg] for seg in ('thigh', 'shank', 'foot'))
    joint, elbow, wrist = (at['left_' + seg] for seg in ('upper_arm', 'forearm', 'hand'))
    (s, e, w, _tip), (base, jaw, spine) = j['arm'], j['neck']
    high = at['torso'][1] + next(f[1] for seg, _r, f, _to in figure.BODY if seg == 'torso' and f[2] > 0.03)
    x, y, z = v[:, 0], v[:, 1], v[:, 2]
    up = np.interp(y, [j['ankle'], j['knee'], j['hip'], j['bust'], s[1], base, jaw],
                   [ankle[1], knee[1], hip[1], high, joint[1], at['neck'][1], at['head'][1]])
    # The body's hips and legs no wider than the robot's own (`figure.BODY`: a thigh's reach
    # aside of its hip's joint); its trunk across by the robot's joints' places over its own.
    out = float(np.abs(x[(np.abs(y - j['crotch']) < 0.02) & ~j['arms']]).max()) - j['leg'][0]
    slim = min(1.0, max(abs(f[0]) + r for seg, r, f, _to in figure.BODY if seg == 'thigh') / out)
    across = ((hip[0] + slim * out) / (out + j['leg'][0]), joint[0] / s[0])
    lx, lz = (np.interp(y, [j['ankle'], j['hip']], [j['foot'][i], j['leg'][i]]) for i in (0, 1))
    lean = np.interp(y, [j['hip'], s[1], base], [0.0, joint[2] - s[2] + j['leg'][1], at['head'][2] - spine + j['leg'][1]])
    trunk = np.stack([np.interp(y, [j['hip'], s[1]], across) * x, up, z - j['leg'][1] + lean], 1)
    legs = np.clip((j['crotch'] + 0.03 - y) / 0.06, 0.0, 1.0)[:, None]
    body = legs * np.stack([np.sign(x) * (hip[0] + slim * (np.abs(x) - lx)), up, slim * (z - lz)], 1) + (1.0 - legs) * trunk
    # The arms, the left's and the right's its mirror: along the arm's line from its shoulder.
    p = v * (np.sign(x)[:, None] * (1.0, 0.0, 0.0) + (0.0, 1.0, 1.0))
    ways = [(b - a) / np.linalg.norm(b - a) for a, b in ((s, e), (e, w), (joint, elbow), (elbow, wrist))]
    upper, fore = float((e - s) @ ways[0]), float((w - e) @ ways[1])
    long_u, long_f = float(np.linalg.norm(elbow - joint)), float(np.linalg.norm(wrist - elbow))
    a = np.where((p - e) @ ways[1] > 0.0, upper + (p - e) @ ways[1], (p - s) @ ways[0])
    bent = np.clip((a - upper + 0.03) / 0.06, 0.0, 1.0)[:, None]
    off = (1.0 - bent) * (p - s - np.outer((p - s) @ ways[0], ways[0])) + bent * (p - e - np.outer((p - e) @ ways[1], ways[1]))
    off = (1.0 - bent) * off @ _turn(ways[0], ways[2]).T + bent * off @ _turn(ways[1], ways[3]).T
    along = np.where(a < upper, a * long_u / upper, np.where(
        a < upper + fore, long_u + (a - upper) * long_f / fore, long_u + long_f + a - upper - fore))
    hung = np.where((along < long_u)[:, None], joint + np.outer(along, ways[2]), elbow + np.outer(along - long_u, ways[3])) + off
    hung[:, 0] *= np.sign(x)
    # An arm's share of a corner: all of one BLEND_M under its armpit; over the armpit as far out
    # of the trunk's side there as it stands across the arm's radius; none of the trunk's under it.
    share = np.clip((np.abs(x) - j['side']) / j['girth'], 0.0, 1.0)
    share = np.where(j['arms'], np.maximum(share, np.clip((j['armpit'] - y) / BLEND_M, 0.0, 1.0)),
                     share * (y > j['armpit']))
    return body + share[:, None] * (hung - body), np.where(share > 0.0, along, -1.0), share


def worn(like):
    """[(name, parent, offset, mesh)]: the body beside the figure at `like` (`beside`) on the
    robot - a part ('form_' and its segment) each segment from its neck to its shanks and its
    hands, a shell's paint, its frame's -; none with no body there. A mesh's second column is
    each corner's share of its shoulder's turns (`fitted`)."""
    from coaxial.model.blocks import numpy as np
    path = like and beside(like)
    if not path:
        return []
    v, groups, _at = wavefront.read(path)
    t = np.vstack([triangles for _names, triangles in groups])
    j = joints(v, t)
    # The body's bust BUST of it (`fall`, the figure's alike): ahead of its chest at its shoulders' height.
    chest = (extent(v[~j['arms']], j['arm'][0][1], 0.01) or (0.0, 0.0))[1]
    full = ~j['arms'] & (v[:, 1] < j['arm'][0][1]) & (v[:, 1] > j['hip']) & (v[:, 2] > chest)
    v[full, 2] = chest + BUST * (v[full, 2] - chest)
    rest, along, share = fitted(v, j)
    frames = {seg: (np.array(place), np.array(turn)) for seg, (place, turn) in figure.frames(
        {}, (0.0, 0.0, 0.0), figure.IDENT).items()}
    upper, fore = (float(np.linalg.norm(frames['left_' + b][0] - frames['left_' + a][0]))
                   for a, b in (('upper_arm', 'forearm'), ('forearm', 'hand')))
    mid, reach = rest[t].mean(axis=1), along[t].mean(axis=1)
    side = np.where(mid[:, 0] > 0.0, 'left_', 'right_')
    limb = np.where(reach > upper + fore - 0.015, 'hand', np.where(reach > upper - 0.03, 'forearm', 'upper_arm'))
    high = np.where(mid[:, 1] > frames['head'][0][1] - 0.02, 'head', np.where(
        mid[:, 1] > frames['neck'][0][1] - 0.02, 'neck', np.where(
            mid[:, 1] > frames['torso'][0][1] - 0.035, 'torso', np.where(
                mid[:, 1] > -0.11, 'pelvis', np.where(
                    mid[:, 1] > frames['left_shank'][0][1] + 0.035, 'thigh', 'shank')))))
    seg = np.where(reach > -0.5, np.char.add(side, limb), np.where(
        np.isin(high, ('thigh', 'shank')), np.char.add(side, high), high))
    keep = (seg != 'head') & (v[t][:, :, 1].min(axis=1) >= j['ankle'])
    out = []
    for name in np.unique(seg[keep]):
        place, turn = frames[str(name)]
        ids, index = np.unique(t[keep & (seg == name)], return_inverse=True)
        out.append(('form_' + str(name), str(name), (0.0, 0.0, 0.0), (
            (rest[ids] - place) @ turn, index.reshape(-1, 3), np.stack([share[ids], np.zeros(len(ids))], 1),
            np.full(len(ids), SHELL))))
    return out
