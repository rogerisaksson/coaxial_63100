"""Marks on the robot: small lettering on a guard's header, drawn a dot wide over what is lit.

A label is the runs of its glyphs' ink told by a triangle of its guard (`lettering.laid`):
posed with the body each frame and seen where its surface faces the eye and nothing stands
PROUD_M before it (the frame's own depth): a dot wide in its ink, alone in its cells
(`lit.ALONE`). What is too fine for the frame's scale is left out: a label is read up close.

    props = marks.layers(body, (pos, norm), m, fine, centre, depth)    # a frame: `lit.braille`'s
"""
from typing import Any

from coaxial.graphics import lettering
from coaxial.graphics.lit import ALONE, project

#: A label's ink.
LETTER = (232, 236, 244)
#: Seen: a mark's surface faces the eye by FACING at the least (a cosine), nothing PROUD_M
#: before it.
FACING, PROUD_M = 0.25, 0.015


def _dots(a, b, m, cam, centre, depth):
    """The dots (`depth`'s shape, bool) of the segments from `a` to `b` (n, 3 each, world) seen
    through the view `m` about `centre` by the fine camera `cam`: a dot a step of a segment's
    longer side, where no more than PROUD_M of what `depth` tells stands before it; None with
    none."""
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
    x, y, w = x[within], y[within], w[within]
    near = depth[y, x]
    shown = (near > 0.0) & (w * (1.0 + PROUD_M * near) >= near)
    if not shown.any():
        return None
    dots = np.zeros(depth.shape, bool)
    dots[y[shown], x[shown]] = True
    return dots


def layers(body, laid, m, cam, centre, depth):
    """[(dots, ink, ALONE)], `lit.braille`'s props: the lettering of `body` (`lettering.laid`'s,
    laid once a body, on its guards) as it is posed - `laid` its corners' places and normals
    (`gynoid.Body.pose`'s), or what tells them, called where there is a label to draw -, seen
    through the view `m` about `centre` by the fine camera `cam`, where its surface faces the
    eye and no more than PROUD_M of what `depth` (the frame's, 1 / z a dot) tells stands before
    it; none where the frame's scale is too coarse to read one at."""
    from coaxial.model.blocks import numpy as np
    if not hasattr(body, 'lettered'):
        body.lettered = lettering.laid(body.parts, body.spans) or ()
    labels: Any = body.lettered
    read = np.flatnonzero(cam['scale'] / cam['distance'] >= labels[4]) if labels else ()
    if not len(read):
        return []
    posed: Any = laid() if callable(laid) else laid
    pos, _norm = posed
    tri, first, last, turn, _least = (told[read] for told in labels)
    p, e1, e2 = pos[tri[:, 0]], pos[tri[:, 1]] - pos[tri[:, 0]], pos[tri[:, 2]] - pos[tri[:, 0]]
    off = np.cross(e1, e2)
    off = off / np.maximum(np.linalg.norm(off, axis=1, keepdims=True), 1e-12) * turn[:, None]
    faced = off @ np.asarray(m).reshape(3, 3)[2] > FACING
    dots = _dots(*((p + c[:, :1] * e1 + c[:, 1:2] * e2 + c[:, 2:] * off)[faced] for c in (first, last)), m, cam, centre, depth)
    return [(dots, LETTER, ALONE)] if dots is not None else []
