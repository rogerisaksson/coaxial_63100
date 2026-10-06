"""The quad's world drawn: the ground's grid, its pole and what stands about the course, as lines.

    floor = scenery.ground(m, cam, centre)          # a brightness a dot, for `lit.braille`
    props = scenery.props(m, cam, centre, gate)     # [(dots, a cell's ink)], for its props

What `machine.course` stands about its line, each a few edges: a tree its trunk and a cone, a
house its walls and its ridge, a car its body and its cabin, a mast its legs and rings; a gate
its frame on two posts, stood where `gate` is given - the one of that number, flown to next, lit
and doubled. And the pole beside the spot, a mark each metre up. Every edge is cut NEAR_M before
the eye and at the view's edges, drawn dimmer the further off and not at all far off; a cell
takes its nearest edge's ink.
"""
import functools
import math

from coaxial.graphics.raster import DOTS_X, DOTS_Y
from machine import course

#: The ground's grids, (pitch, half extent) m: a metre's about the spot, ten metres' to the
#: horizon's edge; a line is drawn from NEAR_M before the eye.
GROUND, NEAR_M = ((1.0, 10.0), (10.0, 150.0)), 0.2

#: The pole: where it stands, m, how high it is marked, and a mark's half width, the tenth's.
POLE_AT, POLE_TOP_M, TICK_M, TENTH_M = (-2.5, -2.5), 60, 0.06, 0.16

#: The inks: the pole, the trees, the houses, the cars, the masts, a gate and the one flown to
#: next. An edge keeps FAR_INK of its ink where it is drawn FAR_SIZE of its size at the framed
#: point, and is not drawn smaller: a band of every far thing's dots along the horizon.
INKS = {'pole': (40, 130, 140), 'tree': (70, 170, 90), 'house': (150, 165, 200),
        'car': (200, 90, 110), 'mast': (150, 150, 150), 'gate': (235, 110, 40),
        'next': (255, 235, 90)}
FAR_INK, FAR_SIZE = 0.25, 0.25


def strokes(ends, m, cam, centre):
    """(ys, xs, w, which): every dot of the world segments `ends` - (2n, 3), a pair a segment -
    in the camera `cam` looking with `m` at `centre`, its depth's 1/z and its segment's number.
    A segment is cut NEAR_M before the eye and at the view's edges, its depth carried along it
    - linear on the screen, as 1/z is: one behind the eye projects through it, one beside it to
    a line of a million dots."""
    from coaxial.model.blocks import numpy as np
    q = (np.asarray(ends, float) - np.asarray(centre, float)) @ np.asarray(m, float).reshape(3, 3).T
    a, b = q[0::2].copy(), q[1::2].copy()
    limit = cam['distance'] - NEAR_M
    which = np.nonzero((a[:, 2] < limit) | (b[:, 2] < limit))[0]
    a, b = a[which], b[which]
    for p, o in ((a, b), (b, a)):
        cut = p[:, 2] > limit
        t = (limit - p[cut, 2]) / (o[cut, 2] - p[cut, 2])
        p[cut] += (o[cut] - p[cut]) * t[:, None]

    def screen(v):
        w = 1.0 / (cam['distance'] - v[:, 2])
        return (cam['cx'] + cam['scale'] * w * v[:, 0],
                cam['cy'] - cam['scale'] * cam.get('aspect', 0.5) * w * v[:, 1], w)
    (x0, y0, w0), (x1, y1, w1) = screen(a), screen(b)
    lo, hi = np.zeros(len(a)), np.ones(len(a))
    for p, d, top in ((x0, x1 - x0, cam['width'] - 1.0), (y0, y1 - y0, cam['height'] - 1.0)):
        flat = np.abs(d) < 1e-12
        safe = np.where(flat, 1.0, d)
        t0, t1 = (0.0 - p) / safe, (top - p) / safe
        inside = (p >= 0.0) & (p <= top)
        lo = np.maximum(lo, np.where(flat, np.where(inside, 0.0, 1.0), np.minimum(t0, t1)))
        hi = np.minimum(hi, np.where(flat, np.where(inside, 1.0, 0.0), np.maximum(t0, t1)))
    seen = lo < hi
    x0, y0, w0, x1, y1, w1, lo, hi, which = (v[seen] for v in (x0, y0, w0, x1, y1, w1, lo, hi,
                                                              which))
    count = (np.maximum(np.abs(x1 - x0), np.abs(y1 - y0)) * (hi - lo)).astype(int) + 2
    of = np.repeat(np.arange(len(count)), count)
    step = np.arange(int(count.sum())) - np.repeat(np.cumsum(count) - count, count)
    t = lo[of] + (hi - lo)[of] * step / (count[of] - 1)
    xs = np.clip(np.rint(x0[of] + (x1 - x0)[of] * t).astype(int), 0, cam['width'] - 1)
    ys = np.clip(np.rint(y0[of] + (y1 - y0)[of] * t).astype(int), 0, cam['height'] - 1)
    return ys, xs, w0[of] + (w1 - w0)[of] * t, which[of]


def ground(m, cam, centre):
    """The ground's grid lines as a (height, width) brightness, each dot by its depth: 0 none,
    far dimmer."""
    from coaxial.model.blocks import numpy as np
    ys, xs, w, _which = strokes(_grid(), m, cam, centre)
    out = np.zeros((cam['height'], cam['width']))
    np.maximum.at(out, (ys, xs), 0.2 + 0.8 * np.clip(w * cam['distance'], 0.0, 1.0))
    return out


@functools.lru_cache(maxsize=None)
def _grid():
    """The ground's grid lines' ends, (2n, 3)."""
    from coaxial.model.blocks import numpy as np
    ends = []
    for pitch, half in GROUND:
        for t in np.arange(-half, half + 1e-9, pitch):
            ends += [(-half, 0.0, t), (half, 0.0, t), (t, 0.0, -half), (t, 0.0, half)]
    return np.asarray(ends)


def _loop(points):
    """The edges round `points`, closed."""
    return list(zip(points, points[1:] + points[:1]))


def _box(x, z, wide, deep, low, high, heading=0.0):
    """A box's twelve edges: `wide` across and `deep` along its `heading`, degrees, from `low`
    to `high`."""
    c, s = math.cos(math.radians(heading)), math.sin(math.radians(heading))
    foot = [(x + a * c + b * s, z - a * s + b * c)
            for a, b in ((-wide / 2, -deep / 2), (wide / 2, -deep / 2), (wide / 2, deep / 2),
                         (-wide / 2, deep / 2))]
    down, up = [(px, low, pz) for px, pz in foot], [(px, high, pz) for px, pz in foot]
    return _loop(down) + _loop(up) + list(zip(down, up))


@functools.lru_cache(maxsize=None)
def standing():
    """(ends, inks): every edge of what stands whatever is flown - the pole, the trees, the
    houses, the cars, the masts - its ends (2n, 3) and its ink's name an edge."""
    edges = {name: [] for name in INKS}
    x, z = POLE_AT
    for h in range(POLE_TOP_M):
        half = TENTH_M if (h + 1) % 10 == 0 else TICK_M
        edges['pole'] += [((x, float(h), z), (x, h + 1.0, z)),
                          ((x - half, h + 1.0, z), (x + half, h + 1.0, z))]
    for x, z, high, crown in course.TREES:
        foot, top = course.CROWN * high, (x, high, z)
        ring = [(x + crown * math.cos(k * math.tau / 6), foot, z + crown * math.sin(k * math.tau / 6))
                for k in range(6)]
        edges['tree'] += [((x, 0.0, z), (x, foot, z))] + _loop(ring) + [(p, top) for p in ring]
    for x, z, wide, deep, wall, ridge in course.HOUSES:
        ends = [(x - wide / 2, ridge, z), (x + wide / 2, ridge, z)]
        edges['house'] += _box(x, z, wide, deep, 0.0, wall) + [tuple(ends)] + [
            (end, (end[0], wall, z + side * deep / 2)) for end in ends for side in (-1.0, 1.0)]
    wide, high, long_ = course.CAR_M
    for x, z, heading in course.CARS:
        edges['car'] += (_box(x, z, wide, long_, course.CAR_LOW, course.CAR_BODY * high, heading)
                         + _box(x, z, course.CAR_CABIN[0] * wide, course.CAR_CABIN[1] * long_,
                                course.CAR_BODY * high, high, heading))
    for x, z, side, high in course.MASTS:
        edges['mast'] += _box(x, z, side, side, 0.0, high)
        for k in range(1, int(high / 3.5) + 1):
            edges['mast'] += _box(x, z, side, side, 3.5 * k, 3.5 * k)[:4]
    return _laid(edges)


@functools.lru_cache(maxsize=None)
def gates(ahead):
    """(ends, inks) of the course's gates, the one `ahead` lit and doubled."""
    edges = {'gate': [], 'next': []}
    for k, (x, y, z, heading) in enumerate(course.GATES):
        edges['next' if k == ahead else 'gate'] += course.gate(x, y, z, heading)
        if k == ahead:
            edges['next'] += course.gate(x, y, z, heading, 0.94 * course.GATE_M)[:4]
    return _laid(edges)


def _laid(edges):
    """{ink: [(a, b)]} as (ends (2n, 3), an ink's number an edge)."""
    from coaxial.model.blocks import numpy as np
    names = list(INKS)
    ends = [end for pairs in edges.values() for pair in pairs for end in pair]
    inks = [names.index(name) for name, pairs in edges.items() for _pair in pairs]
    return np.asarray(ends, float).reshape(-1, 3), np.asarray(inks, int)


def props(m, cam, centre, gate=None):
    """[(dots, inks)] for `lit.braille`: what stands in the camera `cam` looking with `m` at
    `centre` - with the course's gates where `gate` is the one flown to next - its dots and a
    cell's ink, the nearest edge's, dimmer the further off."""
    from coaxial.model.blocks import numpy as np
    ends, inks = standing()
    if gate is not None:
        more, theirs = gates(gate)
        ends, inks = np.vstack([ends, more]), np.concatenate([inks, theirs])
    ys, xs, w, which = strokes(ends, m, cam, centre)
    size = w * cam['distance']
    near = np.argsort(size)
    near = near[size[near] >= FAR_SIZE]
    ys, xs, which, size = ys[near], xs[near], which[near], size[near]
    dots = np.zeros((cam['height'], cam['width']), bool)
    dots[ys, xs] = True
    fade = FAR_INK + (1.0 - FAR_INK) * np.clip((size - FAR_SIZE) / (1.0 - FAR_SIZE), 0.0, 1.0)
    ink = np.zeros((cam['height'] // DOTS_Y, cam['width'] // DOTS_X, 3))
    ink[ys // DOTS_Y, xs // DOTS_X] = (
        np.asarray(list(INKS.values()), float)[inks[which]] * fade[:, None])
    return [(dots, ink)]
