"""Marks on the robot: small lettering on a guard's header, read only when its drive fills the view.

A label is the runs of its glyphs' ink told by a triangle of its guard (`lettering.laid`):
posed with the body each frame and seen where its surface faces the eye and nothing stands
PROUD_M before it (the frame's own depth). Where its joint's drive spans the view's shorter
side (`read`) it is drawn in its ink, alone in its cells (`lit.ALONE`); short of that its box
is filtered to its tone (`lit.TINT`): its cells drawn toward its ink by the share of the box
its ink covers - a smudge where it stands, no glyph. Read from 7 dots a cap, braille text at
medium sizes was clutter over the guards; three sparse lines over its box were lost on the rim.

    props = marks.layers(body, (pos, norm), m, fine, centre, depth)    # a frame: `lit.braille`'s
    marks.read(size, fine)                                             # whether a drive that size reads
"""
from typing import Any

from coaxial.graphics import lettering
from coaxial.graphics.lit import ALONE, TINT, project

#: A label's ink.
LETTER = (232, 236, 244)
#: Seen: a mark's surface faces the eye by FACING at the least (a cosine), nothing PROUD_M
#: before it.
FACING, PROUD_M = 0.25, 0.015


def read(size, cam):
    """Whether a label whose drive is `size` m across is read in the view of the fine camera
    `cam`: the drive spans the view's shorter side - its can fills the view."""
    return cam['scale'] / cam['distance'] * size >= min(cam['width'], cam['height'])


def _dots(a, b, m, cam, centre, depth, value=None):
    """The dots (`depth`'s shape, bool) of the segments from `a` to `b` (n, 3 each, world) seen
    through the view `m` about `centre` by the fine camera `cam`: a dot a step of a segment's
    longer side, where no more than PROUD_M of what `depth` tells stands before it; with `value`
    (n,) each dot its segment's, the most of those on it (float); None with none."""
    from coaxial.model.blocks import numpy as np
    (ax, ay, aw), (bx, by, bw) = (project(ends, m, cam, centre) for ends in (a, b))
    high, wide = depth.shape
    seen = np.flatnonzero((aw > 0.0) & (bw > 0.0) & (np.maximum(ax, bx) >= 0.0) & (np.minimum(ax, bx) < wide)
                          & (np.maximum(ay, by) >= 0.0) & (np.minimum(ay, by) < high))
    if not len(seen):
        return None
    steps = np.maximum(np.abs(bx[seen] - ax[seen]), np.abs(by[seen] - ay[seen])).astype(int) + 1
    on = np.repeat(seen, steps)
    t = (np.arange(len(on)) - np.repeat(np.cumsum(steps) - steps, steps)) / np.repeat(np.maximum(steps - 1, 1), steps)
    x, y = (np.rint(p[on] + t * (q[on] - p[on])).astype(int) for p, q in ((ax, bx), (ay, by)))
    w = aw[on] + t * (bw[on] - aw[on])
    within = (x >= 0) & (x < wide) & (y >= 0) & (y < high)
    x, y, w, on = x[within], y[within], w[within], on[within]
    near = depth[y, x]
    shown = (near > 0.0) & (w * (1.0 + PROUD_M * near) >= near)
    if not shown.any():
        return None
    if value is None:
        dots = np.zeros(depth.shape, bool)
        dots[y[shown], x[shown]] = True
        return dots
    dots = np.zeros(depth.shape)
    np.maximum.at(dots, (y[shown], x[shown]), np.asarray(value, float)[on[shown]])
    return dots


def layers(body, laid, m, cam, centre, depth):
    """[(dots, ink, how)], `lit.braille`'s props: the lettering of `body` (`lettering.laid`'s,
    laid once a body, on its guards) as it is posed - `laid` its corners' places and normals
    (`gynoid.Body.pose`'s), or what tells them, called where there is a label to draw -, seen
    through the view `m` about `centre` by the fine camera `cam`, where its surface faces the
    eye and no more than PROUD_M of what `depth` (the frame's, 1 / z a dot) tells stands before
    it: a label read (`read`) in LETTER alone in its cells, one short of that its box's rows,
    each dot its label's tone, tinting its cells toward LETTER (`lit.TINT`)."""
    from coaxial.model.blocks import numpy as np
    if not hasattr(body, 'lettered'):
        body.lettered = lettering.laid(body.parts, body.spans) or ()
    labels: Any = body.lettered
    if not labels:
        return []
    shown = read(labels[4], cam)
    drawn = np.where(labels[5] == 0, shown, ~shown)
    posed: Any = laid() if callable(laid) else laid
    pos, _norm = posed
    out = []
    for glyphs, how in ((True, ALONE), (False, TINT)):
        pick = np.flatnonzero(drawn & ((labels[5] == 0) == glyphs))
        if not len(pick):
            continue
        tri, first, last, turn = (told[pick] for told in labels[:4])
        p, e1, e2 = pos[tri[:, 0]], pos[tri[:, 1]] - pos[tri[:, 0]], pos[tri[:, 2]] - pos[tri[:, 0]]
        off = np.cross(e1, e2)
        off = off / np.maximum(np.linalg.norm(off, axis=1, keepdims=True), 1e-12) * turn[:, None]
        faced = off @ np.asarray(m).reshape(3, 3)[2] > FACING
        a, b = ((p + c[:, :1] * e1 + c[:, 1:2] * e2 + c[:, 2:] * off)[faced] for c in (first, last))
        dots = _dots(a, b, m, cam, centre, depth, None if glyphs else labels[6][pick][faced])
        out += [(dots, LETTER, how)] if dots is not None else []
    return out
