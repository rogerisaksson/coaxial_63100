"""Cloth bent at the joint it spans: a corner turned about it by its height's share of the turn.

A garment's corners past its segment's joint follow the segment beyond as far as they are
past it: each of the joint's turns taken by that share, about the joint, so a corner keeps its
reach from it. Blended between the two frames' places it sank to the chord's, cos(turn / 2) of
its reach - 0.51 at a hip bent 120 deg - and the shell stood through the cloth. A garment on
one side bends whole; one across both - a seat between two hips - takes each side's bend at a
corner by how near that side's joint the corner stands: masked by its side, a shin's cloth bent
its outer half alone and tore 86 mm at a knee bent 90 deg, and a seat's corners on its middle
followed neither leg and tore 171 mm in a squat.

    rings = drape.fine('cloth_tee', rings)                    # a ring a dot apart across its bends
    table = drape.table(parts, (corners, normals, spans), (dense, dense_normals, dense_spans))
    drape.bent(pos, norm, frames, angles, table)              # a pose, in place
"""
import math

from coaxial.graphics.shapes import DENSE_M, finer, turn_about
from machine.body import figure

#: A garment, the segment its corners bend into, and the local y where the bend begins (its
#: own frame whole) and ends (the other's whole), m; `{side}` either side.
BLENDS = (('cloth_tee', 'pelvis', 0.03, -0.035),
          ('cloth_seat', '{side}_thigh', -0.02, -0.11),
          ('cloth_{side}_shin', '{side}_thigh', -0.01, 0.035),
          ('cloth_{side}_sleeve', 'torso', -0.03, 0.03))

#: The two axes a turn about x, y or z turns, in its hand's order.
TURNED = {'x': (1, 2), 'y': (2, 0), 'z': (0, 1)}


def _segments():
    return {name: (parent, offset, rest) for name, parent, _j, offset, rest, *_r in figure.SEGMENTS}


def _into(seg, target, np):
    """(A, b): a point of `seg`'s frame into `target`'s at rest, `target` its parent or its
    child."""
    segs = _segments()
    parent, offset, rest = segs[seg]
    if target == parent:
        return turn_about('z', rest), np.asarray(offset, float)
    t_parent, t_offset, t_rest = segs[target]
    assert t_parent == seg, (seg, target)
    a = turn_about('z', t_rest).T
    return a, -(a @ np.asarray(t_offset, float))


def fine(name, rings):
    """`rings` of the garment `name` with one no further than a dot (`shapes.DENSE_M`) from the
    next across each of its bends: two to four a bend stood as facets at a knee bent 150 deg."""
    for pattern, _target, y0, y1 in BLENDS:
        if name in (pattern.format(side='left'), pattern.format(side='right')):
            rings = finer(rings, min(y0, y1), max(y0, y1), DENSE_M)
    return rings


def _laid(points, normals, first, zone, x, into, down, np, given=0.0, joint=0.0, pivot=None):
    """(full, there, facing, mid, share, at, out, blend, about) of a garment's `points` and `normals`,
    `first` their index: `full` those wholly its target's, `there` and `facing` theirs in its
    frame; `mid` those between the two, `share` of the joint's turns each takes, `at` and `out`
    theirs in the frame of the part that hangs at the joint - its target's (`down`) or its own. A
    zone of None: the share each takes is `given`, a point's each - a shoulder's, where no height
    tells the arm's from the trunk's -, its side's alone (`x`, 1 the left's). With `joint`, the
    side's joint's place aside, the garment is across both sides: a point takes this side's bend
    by how near its joint it stands, all of it past the joint and half on the middle, `blend`
    (each of `full`'s, each of `mid`'s); else None. `about` the point those between turn about in
    that frame - `pivot`, the garment's own frame's, laid into it -, else None: the joint."""
    y0, y1 = zone
    a, b = into
    w = np.clip((points[:, 1] - y0) / (y1 - y0), 0.0, 1.0) if y0 is not None else np.broadcast_to(
        given if down else 1.0 - given, len(points))
    w = w * (x * points[:, 0] > 0.0 if x and not joint else 1.0)
    near = np.clip(0.5 + x * points[:, 0] / (2.0 * abs(joint)), 0.0, 1.0) if joint else np.ones(len(points))
    full, mid = np.flatnonzero((w >= 1.0) & (near > 0.0)), np.flatnonzero((w > 0.0) & (w < 1.0) & (near > 0.0))
    there, facing = points[full] @ a.T + b, normals[full] @ a.T
    blend = (near[full], near[mid]) if joint else None
    if down:
        about = None if pivot is None else np.asarray(pivot, float) @ a.T + b
        return first + full, there, facing, first + mid, w[mid], points[mid] @ a.T + b, normals[mid] @ a.T, blend, about
    about = None if pivot is None else np.asarray(pivot, float)
    return first + full, there, facing, first + mid, 1.0 - w[mid], points[mid].copy(), normals[mid].copy(), blend, about


def table(parts, corners, dense, blends=BLENDS, given=(None, None)):
    """[(target, hung, joints, laid, laid dense)] a garment in `blends` (BLENDS' columns, and a
    point of the left's own frame its corners between turn about, the right's mirrored, where a
    row has one) among `parts` [(name, parent, ...)] and a side: its target's part, the part of
    the two that hangs from the other at the joint between them and that one's joints; `corners`
    and `dense` each (points, normals, spans), `laid` what of them bends (`_laid`); `given` the
    share of a joint's turns each of the corners and each of the dense points takes where a
    row's zone is None. A garment across both sides (a sided target, an unsided garment) bent by
    a zone takes both sides' bends between their joints, its two rows' `blend` each (weights,
    weights, the corners either row bends)."""
    from coaxial.model.blocks import numpy as np
    names, segs = [p[0] for p in parts], _segments()
    out = []
    for pattern, target, y0, y1, *about in blends:
        across = '{side}' in target and '{side}' not in pattern
        rows = []
        for side, x in (('left', 1.0), ('right', -1.0)):
            name = pattern.format(side=side)
            if name not in names:
                continue
            i, goal = names.index(name), target.format(side=side)
            seg = parts[i][1]
            hung = names.index(goal if segs[goal][0] == seg else seg)
            joint = float(segs[goal][1][0]) if across and y0 is not None and segs[goal][0] == seg else 0.0
            pivot = (about[0][0] * x, about[0][1], about[0][2]) if about else None
            laid = [_laid(points[slice(*spans[i])], normals[slice(*spans[i])], spans[i][0], (y0, y1),
                          x if across else 0.0, _into(seg, goal, np), names[hung] == goal, np,
                          0.0 if share is None else share[slice(*spans[i])], joint, pivot)
                    for (points, normals, spans), share in zip((corners, dense), given)]
            if len(laid[0][0]) or len(laid[0][3]):
                rows.append((names.index(goal), hung, parts[hung][2], *laid))
            if '{side}' not in pattern + target:
                break
        if rows and rows[0][3][7] is not None:     # the corners either side bends, laid by both rows
            held = [np.unique(np.concatenate([np.concatenate([row[3 + k][0], row[3 + k][3]]) for row in rows]))
                    for k in (0, 1)]
            rows = [(*row[:3], *((*row[3 + k][:7], (*row[3 + k][7], held[k]), *row[3 + k][8:]) for k in (0, 1)))
                    for row in rows]
        out += rows
    return out


def _turned(frames, angles, goal, hung, joints, laid):
    """(places, facings) of a row's corners wholly its target's, laid by its frame, (places,
    facings) of those between, turned about the joint - or the row's own point - by their share
    of each of its turns, and the joint's place."""
    from coaxial.model.blocks import numpy as np
    _full, there, facing, _mid, share, at, out, _blend, about = laid
    turn, spot = frames[goal]
    base, hinge = frames[hung]
    p, n = (at.copy() if about is None else at - about), out.copy()
    for joint, axis, sign in reversed(joints):
        whole = sign * float(angles.get(joint, 0.0))
        base = base @ turn_about(axis, -whole)
        i, j = TURNED[axis]
        c, s = np.cos(math.radians(whole) * share), np.sin(math.radians(whole) * share)
        for v in (p, n):
            v[:, i], v[:, j] = c * v[:, i] - s * v[:, j], s * v[:, i] + c * v[:, j]
    if about is not None:
        p += about
    return (there @ turn.T + spot, facing @ turn.T), (p @ base.T + hinge, n @ base.T), hinge


def bent(pos, norm, frames, angles, table, dense=False):
    """`pos` and `norm` (`shapes.moved`'s) bent in place at `angles` {joint: deg}: each row's
    corners wholly its target's laid by its frame, those between turned about the joint by
    their share of each of its turns, their reach from it kept; a garment across both sides
    where each side's bend puts it, by its weight there, its reach kept from the joints'
    places so weighted - averaged, a corner between two turns sank to their chord."""
    from coaxial.model.blocks import numpy as np
    k = 0
    while k < len(table):
        goal, hung, joints, *laid = table[k]
        full, mid, blend = laid[dense][0], laid[dense][3], laid[dense][7]
        if blend is None:
            (pos[full], norm[full]), (pos[mid], norm[mid]), _hinge = _turned(frames, angles, goal, hung, joints, laid[dense])
            k += 1
            continue
        held = blend[2]
        rows = [row for row in table[k:k + 2] if row[3 + dense][7] is not None and row[3 + dense][7][2] is held]
        sums = np.zeros((len(held), 10))           # weighted places, facings, joints and the weights
        for row_goal, row_hung, row_joints, *row_laid in rows:
            got = row_laid[dense]
            whole, part, hinge = _turned(frames, angles, row_goal, row_hung, row_joints, got)
            for idx, weight, (places, facings) in ((got[0], got[7][0], whole), (got[3], got[7][1], part)):
                at = np.searchsorted(held, idx)
                sums[at] += np.hstack([places, facings, np.broadcast_to(hinge, places.shape), np.ones((len(idx), 1))]) * weight[:, None]
        places, facings, joint = (sums[:, i:i + 3] / sums[:, 9:] for i in (0, 3, 6))
        reach, away = np.linalg.norm(pos[held] - joint, axis=1), places - joint
        pos[held] = joint + away * (reach / np.maximum(np.linalg.norm(away, axis=1), 1e-12))[:, None]
        norm[held] = facings
        k += len(rows)
