"""Cloth bent at the joint it spans, two frames a corner blended by its height.

A garment's corners past its segment's joint follow the segment beyond as far as they are
past it.

    table = drape.table(parts, corners, spans, dense, dense_spans)   # once, with the Body
    drape.bent(pos, norm, local, spans, frames, table)              # a pose, in place
"""
from coaxial.graphics.shapes import turn_about
from machine import figure

#: A garment, the segment its corners bend into, and the local y where the bend begins (its
#: own frame whole) and ends (the other's whole), m; `{side}` either side.
BLENDS = (('cloth_tee', 'pelvis', 0.03, -0.035),
          ('cloth_seat', '{side}_thigh', -0.02, -0.11),
          ('cloth_{side}_shin', '{side}_thigh', -0.01, 0.035),
          ('cloth_{side}_sleeve', 'torso', -0.03, 0.03))


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


def _weights(points, y0, y1, np):
    return np.clip((points[:, 1] - y0) / (y1 - y0), 0.0, 1.0)


def table(parts, corners, spans, dense, dense_spans):
    """[(part, target, A, b, w_corners, w_dense)] for every garment in BLENDS among `parts`
    [(name, parent, ...)], the weights from its corners' and its dots' local y."""
    from coaxial.model.blocks import numpy as np
    names = [p[0] for p in parts]
    out = []
    for pattern, target, y0, y1 in BLENDS:
        for side, x in (('left', 1.0), ('right', -1.0)):
            name = pattern.format(side=side)
            if name not in names:
                continue
            i = names.index(name)
            seg = parts[i][1]
            goal = target.format(side=side)
            a, b = _into(seg, goal, np)
            w_c = _weights(corners[slice(*spans[i])], y0, y1, np)
            w_d = _weights(dense[slice(*dense_spans[i])], y0, y1, np)
            if '{side}' in target:
                w_c = w_c * (x * corners[slice(*spans[i]), 0] > 0.0)
                w_d = w_d * (x * dense[slice(*dense_spans[i]), 0] > 0.0)
            if w_c.any():
                out.append((i, names.index(goal), a, b, w_c[:, None], w_d[:, None]))
            if '{side}' not in pattern + target:
                break
    return out


def bent(pos, norm, local, spans, frames, table, dense=False):
    """`pos` and `norm` (`shapes.moved`'s) bent: each table row's corners drawn toward where
    the target's frame carries them, by their weights."""
    for i, j, a, b, w_c, w_d in table:
        lo, hi = spans[i]
        turn, spot = frames[j]
        w = w_d if dense else w_c
        there = (local[lo:hi] @ a.T + b) @ turn.T + spot
        pos[lo:hi] += w * (there - pos[lo:hi])
        norm[lo:hi] += w * ((norm[lo:hi] @ a.T) @ turn.T - norm[lo:hi])
