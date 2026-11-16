"""The robot's likeness: a figure's mesh from beside the tree, fitted to its skeleton, cut by segment.

A T-posed figure (`wavefront`: an OBJ, y up, +z ahead, its left +x) is read, its joints found on
it, what it wears dressed anew - the garments sewn for it laid on it, else a straight tee
(`garment`) -, its arms hung as the robot's hang and its lengths fitted to the robot's own -
trunk and limbs joint to joint, its spine on the robot's, its girth by its trunk's scale - and
its triangles laid on the segments they move with, bent at the joints as cloth is (`drape`,
`plated.BLENDS`). A cyborg: its face alone wears its maps' skin, the rest of its body a shell's paint;
its hair is its head's, what hangs free of the skull hung as the robot's ponytail swings. The
mesh is never the tree's: FOUND is where one lies (COAXIAL_LIKENESS, else the first T-posed one
under stunts/3d_models beside the repo), None where there is none and the robot is drawn as built.

    parts = likeness.worn(likeness.FOUND, inks, hung)   # [(name, parent[, joints], offset, mesh)]
"""
import glob
import math
import os
import re

from coaxial.graphics import fall, garment, wavefront
from coaxial.graphics.lit import PAINTED, UNDER, WOVEN
from machine.body import figure

#: What of a figure is left out, by a word in its group's or its material's name: what its mouth
#: and lashes hide, and its shoes - the robot wears its own.
LEFT_OUT = ('tongue', 'teeth', 'eyelash', 'cornea', 'heel', 'shoe', 'boot', 'sandal')
#: What a group is by a word in its names: the figure's body, bare where no garment is over it.
BODY = ('skin', 'nail', 'eye', 'hair', 'scalp')
#: A figure's arm from its shoulder to its fingertips: the shares its elbow and its wrist stand at.
ELBOW, WRIST = 0.39, 0.76
#: A figure's hip joint over its crotch, a share of its crown's height; a seam's half width at a
#: wrist, m; a sector along a limb, m - a garment's corners stand 10-20 mm apart; how far ahead
#: of its neck's front a figure's chin stands at the least, m - its face is its skin over that.
HIP_UP, SEAM_M, SECTOR, JAW_M = 0.05, 0.012, 0.03, 0.02
#: A shoulder's fall from the arms out to the arms hung, m: a clavicle 150 mm long turns 10-15
#: deg, its end 26-39 mm; a figure posed arms out, hung without it, stands square-shouldered.
SHRUG_M = 0.03
#: Hair this far behind the skull's back, m, hangs free of it (a ponytail stood 140 mm off); the
#: rings of triangles a body is kept under a garment's opening - a collar looked into showed a
#: hole about the neck; the folders up from the repo a figure is looked for beside: a worktree
#: stands three folders under its checkout; a T-posed figure's reach across,
#: a share of its height - arms out 0.95 and more, hung 45 deg 0.75, down 0.3-0.4.
FREE_M, RINGS, BESIDE, SPAN = 0.03, 4, 4, 0.85
#: A face's map's tints are worn TONE of themselves: the light raises a lit paint 1.3 fold and
#: the fold to cells 1.2 more - a face in its map's own tints was white where it was lit, its
#: eyes and lips lost; at 0.64 it was dim.
TONE = 0.72


def _t_posed(path):
    """Whether the figure at `path` stands T-posed: its corners reach SPAN of its height across."""
    with open(path, 'rb') as file:
        at = re.findall(rb'^v\s+(\S+)\s+(\S+)', file.read(), re.M)
    x, y = ([float(p[k]) for p in at] for k in (0, 1))
    return bool(at) and max(x) - min(x) >= SPAN * (max(y) - min(y))


def found(root=None):
    """Where a figure's mesh lies: COAXIAL_LIKENESS's file ('none' for none), else the first OBJ
    that stands T-posed under stunts/3d_models beside the repo `root` (the tree's unless told) -
    or beside the checkout a worktree of it stands in, BESIDE folders up at the most -, its
    folders in their numbers' order; None where there is none. A posed figure there is a
    reference, none to wear: seven of them laid beside the first, 'female_10' read before
    'female_1' and the page fell on a figure with no arms out to find."""
    said = os.environ.get('COAXIAL_LIKENESS')
    if said is not None:
        return said if os.path.isfile(said) else None
    beside = root or os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    for _ in range(BESIDE):
        beside = os.path.dirname(beside)
        for got in sorted(glob.glob(os.path.join(beside, 'stunts', '3d_models', '*', '*.obj')), key=lambda path: [
                int(word) if word.isdigit() else word.lower() for word in re.split(r'(\d+)', path)]):
            if _t_posed(got):
                return got
    return None


FOUND = found()


def joints(v):
    """A T-posed figure's joints found on its corners `v` (no hair, none left out), m: its crown,
    its arms' axis (y, z), fingertip, armpit, elbow and wrist (x), its crotch, hip, knee and
    ankle (y), its leg's axis at the hip and at the ankle (x, z)."""
    from coaxial.model.blocks import numpy as np
    slices, left = fall.slices, v[v[:, 0] > 0.0]
    crown, tip = float(v[:, 1].max()), float(left[:, 0].max())
    arm = left[left[:, 0] > 0.6 * tip]
    arm_y, arm_z = float(np.median(arm[:, 1])), float(np.median(arm[:, 2]))
    thick = float(np.median([np.ptp(s[:, 1]) for _at, s in slices(left, 0, 0.55 * tip, 0.8 * tip, 0.02) if len(s)]))
    pit = float(max(at for at, s in slices(left[left[:, 1] > 0.6 * crown], 0, 0.1 * tip, 0.6 * tip)
                    if len(s) and s[:, 1].min() < arm_y - 1.5 * thick))
    low = v[(v[:, 1] < 0.65 * crown) & (np.abs(v[:, 0]) < pit)]
    # The legs stand apart from the floor up to the crotch: the first slice nothing parts.
    crotch = next(at for at, s in slices(low, 1, 0.3 * crown, 0.65 * crown, 0.01)
                  if len(s) and np.abs(s[:, 0]).min() < 0.006)
    leg = low[low[:, 0] > 0.0]
    ankle = min(((np.ptp(s[:, 0]), at) for at, s in slices(leg, 1, 0.03 * crown, 0.2 * crown) if len(s)))[1]

    def axis(at):
        s = leg[np.abs(leg[:, 1] - at) < 0.01]
        return (float(s[:, 0].min() + s[:, 0].max()) / 2.0, float(s[:, 2].min() + s[:, 2].max()) / 2.0)
    hip = float(crotch + HIP_UP * crown)
    return {'crown': crown, 'arm': (arm_y, arm_z), 'tip': tip, 'pit': pit, 'elbow': pit + ELBOW * (tip - pit),
            'wrist': pit + WRIST * (tip - pit), 'crotch': float(crotch), 'hip': hip,
            'knee': (hip + float(ankle)) / 2.0, 'ankle': float(ankle), 'leg': axis(crotch - 0.04), 'foot': axis(ankle)}


def _sewn(path):
    """[(corners, triangles, each triangle's piece)]: the garments sewn for the figure at `path`,
    those thinned under worn/ beside it (`tools/sim/sewn.py`): a group a piece, named its
    garment's and a dot before its number."""
    from coaxial.model.blocks import numpy as np
    out = []
    for file in sorted(glob.glob(os.path.join(os.path.dirname(path), 'worn', '*.obj'))):
        v, groups, _at = wavefront.read(file)
        for sewn in sorted({names.split('.')[0] for names, _t in groups}):
            cut = [t for names, t in groups if names.split('.')[0] == sewn]
            ids, index = np.unique(np.concatenate(cut), return_inverse=True)
            out.append((v[ids], index.reshape(-1, 3), np.repeat(np.arange(len(cut)), [len(t) for t in cut])))
    return out


def _under(v, j, clothed):
    """Whether each corner of `v` stands under a garment's (`clothed`, its corners among `v`): about
    its limb's axis - the trunk's, a leg's, an arm's - a garment's corner in its sector, SECTOR along
    and a twelfth round, or in the one beside it round - a thinned garment's corners stand 14 mm
    apart, and a sector with none left the skin under it bare -, further out; and the corner
    between that garment's ends along the limb there - an end within SECTOR of the limb's own, a
    leg's at the crotch, an arm's at the armpit, is none. Told by sectors alone a neck stood
    covered 17 mm over its collar."""
    from coaxial.model.blocks import numpy as np
    x, y, z = v[:, 0], v[:, 1], v[:, 2]
    arm, leg = np.abs(x) > j['pit'], (y < j['crotch']) & (np.abs(x) <= j['pit'])
    lx, lz = (np.interp(y, [j['ankle'], j['hip']], [j['foot'][i], j['leg'][i]]) for i in (0, 1))
    limb, along = (arm | leg) * (1 + arm * 2 + (x > 0.0)), np.where(arm, np.abs(x), y)
    across = np.where(arm, y - j['arm'][0], np.where(leg, np.abs(x) - lx, x))
    ahead = z - np.where(arm, j['arm'][1], np.where(leg, lz, j['leg'][1]))
    row, about = limb * 128 + np.clip(along / SECTOR, 0, 126).astype(int), (
        (np.arctan2(ahead, across) + math.pi) / (math.pi / 6.0)).astype(int) % 12
    reach, out, ends = np.hypot(across, ahead), np.zeros((5 * 128, 12)), np.full((2, 5, 12), np.inf)
    np.maximum.at(out, (row[clothed], about[clothed]), reach[clothed])
    for end, sign in ((0, 1.0), (1, -1.0)):
        np.minimum.at(ends[end], (limb[clothed], about[clothed]), sign * along[clothed])
    out, ends = (np.stack([np.roll(told, turn, -1) for turn in (-1, 0, 1)]) for told in (out, ends))
    bounds = np.array([(j['crotch'], j['crown']), (0.0, j['crotch']), (0.0, j['crotch']), (j['pit'], j['tip']),
                       (j['pit'], j['tip'])])
    first = np.where(ends.min(axis=0)[0] < bounds[:, :1] + SECTOR, -np.inf, ends.min(axis=0)[0])
    last = np.where(-ends.min(axis=0)[1] > bounds[:, 1:] - SECTOR, np.inf, -ends.min(axis=0)[1])
    return (out.max(axis=0)[row, about] > reach) & (along >= first[limb, about]) & (along <= last[limb, about])


def _turned(q, deg):
    """Rows `q` turned about z by `deg`, a row its own."""
    from coaxial.model.blocks import numpy as np
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.stack([c * q[:, 0] - s * q[:, 1], s * q[:, 0] + c * q[:, 1], q[:, 2]], 1)


def fitted(v, j):
    """(corners in the robot's pelvis frame at rest, each corner's place along its arm past the
    shoulder - m of the robot's, -1 on the trunk -, the share of its arm's turns each takes, the
    corners with its arms out): the figure's `v` at its joints `j` laid on the robot's, its arms
    hung."""
    from coaxial.model.blocks import numpy as np
    at = {seg: np.array(place) for seg, (place, _turn) in figure.frames(
        {}, (0.0, 0.0, 0.0), figure.IDENT).items()}
    hip, knee, ankle = (at['left_' + seg] for seg in ('thigh', 'shank', 'foot'))
    shoulder, elbow, wrist = (at['left_' + seg] for seg in ('upper_arm', 'forearm', 'hand'))
    k = float((shoulder[1] - hip[1]) / (j['arm'][0] - j['hip']))
    x, y, z = v[:, 0], v[:, 1], v[:, 2]
    up = np.where(y >= j['hip'], hip[1] + k * (y - j['hip']), np.where(
        y >= j['knee'], hip[1] + (y - j['hip']) * (hip[1] - knee[1]) / (j['hip'] - j['knee']),
        knee[1] + (y - j['knee']) * (knee[1] - ankle[1]) / (j['knee'] - j['ankle'])))
    # A shoulder's joint stands a sleeve's radius past the armpit, on the arm's axis: the trunk as
    # wide between its arms as the robot's is. With the armpit's line laid on the joint the arms
    # hung half within the trunk's sides, 45 mm, and stood through its top's.
    base, rim, spine = j['neck']
    sleeve = j['arm'][0] - j['under']
    joint = j['pit'] + sleeve
    # The figure's hips and legs no wider than the robot's own (`figure.BODY`: a thigh's reach
    # aside of its hip's joint): 38 mm wider a side, they stood the tee's hem into the forearms.
    out = float(np.abs(x[np.abs(y - j['crotch']) < 0.02]).max()) - j['leg'][0]
    slim = min(k, max(abs(f[0]) + r for seg, r, f, _to in figure.BODY if seg == 'thigh') / out)
    wide = np.interp(y, [j['hip'], j['arm'][0], base], [
        (hip[0] + slim * out) / (out + j['leg'][0]), shoulder[0] / joint, k])
    lx, lz = (np.interp(y, [j['ankle'], j['hip']], [j['foot'][i], j['leg'][i]]) for i in (0, 1))
    legs = np.clip((j['crotch'] + 0.03 - y) / 0.06, 0.0, 1.0)[:, None]
    # The figure's spine on the robot's: its arms' axis on the shoulders' joints, its neck's middle on the
    # axis the head turns about - 30 and 42 mm ahead of where its own stood over its hips: a head
    # turned about an axis before its neck slid off it.
    lean = np.interp(y, [j['hip'], j['arm'][0], base], [0.0, shoulder[2] - k * (j['arm'][1] - j['leg'][1]),
                                                       at['head'][2] - k * (spine - j['leg'][1])])
    trunk = np.stack([wide * x, up, k * (z - j['leg'][1]) + lean], 1)
    out = legs * np.stack([np.sign(x) * (hip[0] + slim * (np.abs(x) - lx)), up, slim * (z - lz)], 1) + (1.0 - legs) * trunk
    # The arms: along the arm to the robot's lengths, their girth by k; hung by the robot's rest.
    a, bent, hand = np.abs(x) - joint, j['elbow'] - joint, j['wrist'] - joint
    upper, fore = math.dist(shoulder, elbow), math.dist(elbow, wrist)
    along = np.where(a < bent, a * upper / bent, np.where(
        a < hand, upper + (a - bent) * fore / (hand - bent), upper + fore + k * (a - hand)))
    hang, bend = (math.degrees(math.atan2(d[1], d[0])) for d in (elbow - shoulder, wrist - elbow))
    # The shoulders fall as the arms are hung: what stands by the neck comes SHRUG_M higher over
    # the joints than with the arms out - none of it at the armpit's width, all within a
    # collar's rim about the neck (`j['neck']`: where the neck leaves the shoulders, that rim).
    span = base - j['arm'][0]
    out[:, 1] += k * SHRUG_M * max(-hang, 0.0) / 90.0 * np.clip((y - j['arm'][0] + span) / (2.0 * span), 0.0, 1.0) * (
        np.maximum(np.clip((j['pit'] - np.abs(x)) / (j['pit'] - rim), 0.0, 1.0), np.clip((y - base) / span, 0.0, 1.0)))
    q = np.stack([along, k * (y - j['arm'][0]), k * (z - j['arm'][1])], 1)
    q = (upper, 0.0, 0.0) + _turned(q - (upper, 0.0, 0.0), (bend - hang) * np.clip((along - upper + 0.03) / 0.06, 0.0, 1.0))
    # An arm's share of a corner: none at the armpit's line, all 30 mm past the joint, and none
    # under the armpit (`j['under']`, a sleeve's underside) - a top's side there is the trunk's:
    # hung with the arm, it stood out in spikes as the arm was raised. A corner takes that share
    # of the shoulder's turns too (`plated.BLENDS`, `drape`): told by its height there, the armpit tore.
    arm = np.clip((a + sleeve) / (sleeve + 0.03), 0.0, 1.0) * np.clip((y - j['under'] + 0.025) / 0.02, 0.0, 1.0)
    hung = shoulder + _turned(q, hang * arm)
    hung[:, 0] *= np.sign(x)
    return out + arm[:, None] * (hung - out), np.where(arm > 0.0, along, -1.0), arm, out


def _rings(flags, triangles, rounds):
    """`flags`, a corner's each, and those of every triangle with a flagged corner, `rounds`
    times over: a ring of triangles a round."""
    out = flags.copy()
    for _ in range(rounds):
        out[triangles[out[triangles].any(axis=1)]] = True
    return out


def worn(path, inks, hung):
    """[(name, parent[, joints], offset, mesh)]: the figure at `path` on the robot, a cyborg: a
    part ('like_' and its segment) each segment but its feet's, its hair one on its head and
    what of that hangs free of its skull one hung there by the joints `hung` (the robot's
    ponytail's). `inks` [(word, material)] a group's paint by a word in its names, the first in
    them: 'face' its face's skin with no map of its own, 'skin' its limbs' - a shell's, no
    skin's -, 'joint' the dark of its neck and at its wrists. What it wears over its trunk gives way to the
    garments sewn for it, else a tee (`garment.dressed`). Of its body under its clothes what
    stands by their openings alone is kept, of a garment under another what that leaves bare,
    and what is left under a garment is UNDER (`lit`): no cloth is stood through - a hip bent,
    jeans whole under a tee stood out of it. The feet are left out. A mesh's uv is each corner's
    share of its shoulder's turns (`fitted`) and how folded it lies, its arms hung."""
    from coaxial.model.blocks import numpy as np
    v, groups, at = wavefront.read(path)
    groups = [(names, t) for names, t in groups if not any(word in names for word in LEFT_OUT)]
    # Each group's corners in its map's paint, its triangles those its map shows.
    maps = {names: wavefront.painted(path, names, at, t) for names, t in groups}
    tints = {names: got[0] for names, got in maps.items() if got}
    groups = [(names, got[1] if got else t) for names, t in groups for got in [maps[names]]]

    def corners(pick):
        return v[np.unique(np.concatenate([t.ravel() for names, t in groups if pick(names)] or [np.zeros(0, int)]))]

    def dressed(names):
        return not any(word in names for word in BODY)
    j, bare = joints(corners(lambda names: 'hair' not in names)), corners(lambda names: 'skin' in names)
    base, wide, front, back = garment.neck(bare, j)
    j['neck'], j['under'] = (base, wide + garment.NECK_M, (front + back) / 2.0), 2.0 * j['arm'][0] - base
    # The figure's face: its skin over its chin - the lowest of it over its neck's root that stands JAW_M
    # ahead of its neck.
    chin = bare[(bare[:, 1] > base) & (bare[:, 2] > front + JAW_M), 1].min(initial=j['crown'])
    tops = [names for names, t in groups if dressed(names) and v[t][:, :, 1].mean() > j['hip']]
    sewn = garment.dressed(_sewn(path), corners(lambda names: names in tops), corners(
        lambda names: dressed(names) and names not in tops), bare, j) if tops else []
    if tops:
        # The bust under the tee as the tee's: BUST of it, ahead of the chest at the shoulders' height.
        top, skin = corners(lambda names: names in tops), np.zeros(len(v), bool)
        skin[np.concatenate([t.ravel() for names, t in groups if 'skin' in names])] = True
        chest = (fall.extent(top[np.abs(top[:, 0]) < j['pit']], j['arm'][0]) or (0.0, 0.0, 0.0))[1]
        bust = skin & (np.abs(v[:, 0]) < j['pit']) & (v[:, 1] < j['arm'][0]) & (v[:, 1] > j['hip']) & (v[:, 2] > chest)
        v[bust, 2] = chest + fall.BUST * (v[bust, 2] - chest)
    # What it wore in their place goes: over its trunk, and under that where jeans are laid.
    laid, own = any(names == 'jeans' for names, *_rest in sewn), len(v)
    groups = [(names, t) for names, t in groups if names not in tops and not (laid and dressed(names))]
    lit = [np.ones(len(v))]
    for names, more, triangles, shade in sewn:
        groups.append((names, triangles + len(v)))
        v = np.vstack([v, more])
        lit.append(np.ones(len(more)) if shade is None else shade)
    lit = np.concatenate(lit)
    rest, along, share, spread = fitted(v, j)
    # How folded a corner lies at rest: its edges arms out over hung, the most of them.
    edges = np.concatenate([t for _names, t in groups])
    a, b = edges.ravel(), np.roll(edges, 1, axis=1).ravel()
    slack = np.ones(len(v))
    np.maximum.at(slack, a, np.linalg.norm(spread[a] - spread[b], axis=1) / np.maximum(np.linalg.norm(rest[a] - rest[b], axis=1), 1e-6))
    frames = {seg: (np.array(place), np.array(turn)) for seg, (place, turn) in figure.frames(
        {}, (0.0, 0.0, 0.0), figure.IDENT).items()}
    upper, fore = (float(np.linalg.norm(frames['left_' + b][0] - frames['left_' + a][0]))
                   for a, b in (('upper_arm', 'forearm'), ('forearm', 'hand')))
    # What covers what: its body 0, what it wears 1, what it wears over that 2. A corner one of
    # a higher covers is covered; its skin is kept RINGS rings of triangles from what is not.
    rank, skin = np.zeros(len(v), int), np.zeros(len(v), bool)
    for names, t in groups:
        rank[t] = (1 + (names == 'tee')) * dressed(names)
        skin[t] |= 'skin' in names
    over = [_under(v, j, rank == r) for r in (1, 2)]
    covered = np.where(rank == 0, over[0] | over[1], (rank == 1) & over[1])
    shown = _rings(skin & ~covered, np.concatenate([t for names, t in groups if 'skin' in names]), RINGS)
    # The hair free of the skull: under the head's joint, or FREE_M behind the skull's back there.
    head_y = frames['head'][0][1]
    skull = rest[skin & (rest[:, 1] > head_y)]
    rows = np.arange(head_y, skull[:, 1].max(initial=head_y) + 0.01, 0.02)
    behind = np.array([skull[np.abs(skull[:, 1] - y) < 0.02, 2].min(initial=0.0) for y in rows]) - FREE_M
    shell, face, joint = (dict(inks)[word] for word in ('skin', 'face', 'joint'))
    limbs = (np.abs(v[:own, 0]) > j['pit']) | (v[:own, 1] < j['crotch'])
    cut = {}
    for names, t in groups:
        ink, bared = tints.get(names), 'skin' in names or 'nail' in names
        if bared:
            # No skin's but its face's: its limbs a shell's, and what of its trunk its clothes
            # leave bare - its neck, in its collar - a joint's dark, its drives seen in it.
            ink, told = np.full(len(v), shell, np.int64), face if ink is None else (ink & ~0xFFFFFF) | sum(
                (((ink >> s) & 255) * TONE).astype(np.int64) << s for s in (16, 8, 0))
            ink[:own] = np.where((v[:own, 1] > chin) & ('skin' in names), told, np.where(limbs, shell, joint))
        else:
            ink = ink | WOVEN * dressed(names) if ink is not None else next(m for word, m in inks if word in names)
        if isinstance(ink, int) and ink >= PAINTED:        # a paint, each corner lit as its folds shade it
            rgb = np.clip(np.array([(ink >> s) & 255 for s in (16, 8, 0)]) * lit[:, None], 0, 255).astype(np.int64)
            ink = (ink & ~0xFFFFFF) | rgb[:, 0] << 16 | rgb[:, 1] << 8 | rgb[:, 2]
        mid, reach = rest[t].mean(axis=1), along[t].mean(axis=1)
        side = np.where(mid[:, 0] > 0.0, 'left_', 'right_')
        limb = np.where(reach > upper + fore - 0.015, 'hand', np.where(reach > upper - 0.03, 'forearm', 'upper_arm'))
        high = np.where(mid[:, 1] > head_y - 0.02, 'head', np.where(
            mid[:, 1] > frames['neck'][0][1] - 0.02, 'neck', np.where(
                mid[:, 1] > frames['torso'][0][1] - 0.035, 'torso', np.where(
                    mid[:, 1] > -0.11, 'pelvis', np.where(
                        mid[:, 1] > frames['left_shank'][0][1] + 0.035, 'thigh', 'shank')))))
        seg = np.where(reach > -0.5, np.char.add(side, limb), np.where(
            np.isin(high, ('thigh', 'shank')), np.char.add(side, high), high))
        if dressed(names):                     # a collar lies on the shoulders: nodded with the neck, it stood up behind
            seg = np.where(np.isin(seg, ('neck', 'head')), 'torso', seg)
        if 'hair' in names:
            seg = np.where((mid[:, 1] < head_y) | (mid[:, 2] < np.interp(mid[:, 1], rows, behind)), 'fall', 'hair')
        keep = shown[t].any(axis=1) & (v[t][:, :, 1].min(axis=1) >= j['ankle']) if 'skin' in names else ~(
            covered[t].all(axis=1) & dressed(names))
        for name in np.unique(seg[keep]):
            cut.setdefault(str(name), []).append((t[keep & (seg == name)], ink, bared))
    out = []
    for seg, pieces in cut.items():
        place, turn = frames['head' if seg in ('hair', 'fall') else seg]
        ids, index = np.unique(np.concatenate([t.ravel() for t, _ink, _bared in pieces]), return_inverse=True)
        paint = np.zeros(len(v), np.int64)
        for t, ink, bared in pieces:
            paint[t] = ink if isinstance(ink, int) else ink[t]
            paint[t[(np.abs(along[t] - upper - fore) < SEAM_M) & bared]] = joint
        paint[ids] |= UNDER * (covered[ids] & (paint[ids] >= PAINTED))
        local = (rest[ids] - place) @ turn
        mesh = (index.reshape(-1, 3), np.stack([share[ids], slack[ids]], 1), paint[ids])
        if seg == 'fall':                      # hung where it leaves the head: the top of what hangs
            tie = local[local[:, 1] > local[:, 1].max() - 0.03].mean(axis=0)
            out.append(('like_fall', 'head', hung, tuple(tie), (local - tie, *mesh)))
        else:
            out.append(('like_' + seg, 'head' if seg == 'hair' else seg, (0.0, 0.0, 0.0), (local, *mesh)))
    return out
