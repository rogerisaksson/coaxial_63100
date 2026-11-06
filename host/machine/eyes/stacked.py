"""A look's things as arrays, restacked where they changed; their classes kept while their values hold.

    names, centre, half, R, ids, clear = stacked.stack(looks)            # a look's boxes, stacked whole
    boxes, dirty = stacked.restacked(kept, looks, changed)               # kept look to look, its changed rows alone
    names = stacked.classed(slots, kinds, centre, half, R, clear, dirty) # their classes, told anew as values change

What her registry takes of a look (`machine.eyes.registry.seen`): each thing a box - a kind, a centre, half sizes, a
rotation, her eyes' id and whether the space over it is seen clear -, stacked as arrays; in her vision apart's
process kept from look to look, only the rows not the same objects as the last look's restacked.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.eyes import classes

#: A box's centre, half sizes and rotation: their shapes.
BOX = ((3,), (3,), (3, 3))


def stack(looks):
    """(kinds, centres (n, 3), halves (n, 3), rotations (n, 3, 3), ids (n,) 0 none, clear (n,) 1.0 where the space
    over a thing is seen clear, a box's without a word) of a look's things, those that are boxes: its things
    of four fields stacked at once, sized one by one where they do not stack as boxes."""
    rows = [p for p in looks if len(p) >= 4]
    try:
        stacked = [np.array([p[j] for p in rows], float).reshape((len(rows),) + s) for j, s in enumerate(BOX, 1)]
    except (TypeError, ValueError):
        rows = [p for p in rows if all(np.size(p[j]) == math.prod(s) for j, s in enumerate(BOX, 1))]
        stacked = [np.array([p[j] for p in rows], float).reshape((-1,) + s) for j, s in enumerate(BOX, 1)]
    return ([str(p[0]) for p in rows], *stacked, np.array([int(p[4]) if len(p) > 4 else 0 for p in rows], np.int64),
            np.array([float(p[5]) if len(p) > 5 else 1.0 for p in rows]))


def _box(p):
    """Whether look `p` stacks as a box (`stack`): four fields at least, a centre, half sizes and a rotation."""
    return len(p) >= 4 and all(np.size(p[j]) == math.prod(s) for j, s in enumerate(BOX, 1))


def restacked(kept, looks, changed):
    """(`stack(looks)`, whether a value of it changed since the last look): kept on `kept` look to look - its rows
    `changed` (those not the same objects as the last look's) restacked alone, numpy's conversion row by row, handed on
    as copies -, `stack`'s own where a thing of the look or of the last is not a box. Restacked whole, her lodge's 64
    things took 146 us of a look in her vision's process, its door alone changing."""
    was = kept.get('stacked')
    if was is None or len(was[0]) != len(looks) or not all(_box(looks[k]) for k in changed):
        boxes = stack(looks)
        whole = len(boxes[0]) == len(looks)
        kept['stacked'] = tuple(list(b) if isinstance(b, list) else b.copy() for b in boxes) if whole else None
        return boxes, True
    names, centre, half, R, ids, clear = was
    dirty = False
    for k in changed:
        p = looks[k]
        row = (np.asarray(p[1], float).reshape(3), np.asarray(p[2], float).reshape(3),
               np.asarray(p[3], float).reshape(3, 3), float(p[5]) if len(p) > 5 else 1.0)
        dirty = dirty or str(p[0]) != names[k] or not all(np.array_equal(a, b) for a, b in
                                                           zip(row, (centre[k], half[k], R[k], clear[k])))
        names[k], (centre[k], half[k], R[k], clear[k]), ids[k] = str(p[0]), row, int(p[4]) if len(p) > 4 else 0
    return (list(names), centre.copy(), half.copy(), R.copy(), ids.copy(), clear.copy()), dirty


def classed(slots, kinds, centre, half, R, clear, dirty=True):
    """A look's things' classes (`classes.of`), kept on `slots` with the look they are of and told anew only as it
    changes value: `of` is a pure function of the look, compares its numbers and never their zeros' signs, so the
    same bits. Told each look, the lodge's 64 things took 1.1 ms of its 1.64 - every 9 ms, joined by her law -, its
    door at rest the same values look after look. Not `dirty` (`restacked`), no value changed since the last look:
    what it was told then kept without comparing - each look told anew where one changed, the last is what it holds."""
    was = slots.get('classed')
    if was is not None and not dirty:
        return was[5]
    if was is not None and was[0] == kinds and all(np.array_equal(a, b) for a, b in zip(was[1:5], (centre, half, R, clear))):
        return was[5]
    names = classes.of(kinds, centre, half, R, clear)[0]
    slots['classed'] = (kinds, centre, half, R, clear, names)
    return names
