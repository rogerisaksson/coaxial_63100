"""The sketches' hoses hung by MuJoCo (`machine.world.hoses`): a span let fall as its joint turns.

    path = hung.path(span, flex)       # (k, 3): the hose at `flex` deg in the sketch's frame, mouth to mouth
    long = hung.long(span)             # m: as far as its fittings ever stand apart, and SPARE_M
    least = hung.least(span)           # m: the least radius it bends at through its joint's travel

A sketch's span (`knee_hose.span`, `elbow_mount.span`): 'from' a fitting on the still side, 'to' one on the side turned
`flex` deg about x (`knee_mount._rx`) - each (side, its mouth, the way the hose leaves it) -, 'solids' cylinders on
either and 'travel' (from, to) deg, each side's frame the joint's with it straight. The fittings are laid facing each
other across the joint's axis near the middle of its travel: there the hose is straight and as long as it ever needs to
be, either way from there it bows - a slack between fittings that face buckles it, so it is cut no longer than `long`.
The joint is moved from straight STEP_DEG at a time, MOVED_S a step, the hose let fall at each (`_Swept`, kept a span
and length): its lie at any turn the same whatever was asked before; its links LINKS to its least bending radius, its
centre line a curve through their ends (`hoses.spline`).
"""
import functools
import math

#: The joint's step between two lies of its hose, deg, the time each step is moved through, s, and its links to its
#: least bending radius; what a hose is cut longer than its fittings' farthest, m: its cut's tolerance and what it
#: shrinks cold.
STEP_DEG, MOVED_S, LINKS, SPARE_M = 15.0, 0.1, 3, 0.002


def _radius(points):
    """The least radius a chain of links through `points` (k, 3) bends at, m: a link's length over its turn."""
    import numpy as np
    seg = np.diff(np.asarray(points, float), axis=0)
    long_ = np.linalg.norm(seg, axis=1)
    u = seg / long_[:, None]
    turn = np.arccos(np.clip((u[:-1] * u[1:]).sum(axis=1), -1.0, 1.0))
    return float((0.5 * (long_[:-1] + long_[1:]) / np.maximum(turn, 1e-12)).min())


class _Swept:
    """A span's hose `length` long in its own world, its joint moved from straight; each STEP_DEG's state kept."""

    def __init__(self, span, length):
        import numpy as np
        from machine.body import mechatronics
        from machine.world import hoses
        self.rest = (np.eye(3), np.zeros(3))
        least_ = float(mechatronics.materials()['hoses'][span['kind']][4])
        self.world = hoses.World([dict(span, length=length)], {'still': self.rest, 'turning': self.rest}, least_ / LINKS)
        self.grid, self.lies = {0: self.world.kept()}, {}

    def _turned(self, flex):
        import numpy as np
        from tools.render import knee_mount as km
        self.world.follow({'still': self.rest, 'turning': (km._rx(flex), np.zeros(3))}, MOVED_S)
        self.world.settle()

    def lie(self, flex):
        """The links' ends (k, 3) at `flex` deg - a link in each fitting too - reached from straight a step at a
        time."""
        if flex not in self.lies:
            k = int(flex / STEP_DEG + math.copysign(1e-9, flex))     # the step before it, toward straight
            way = 1 if k > 0 else -1
            for j in range(way, k + way, way):
                if j not in self.grid:
                    self.world.restore(self.grid[j - way])
                    self._turned(j * STEP_DEG)
                    self.grid[j] = self.world.kept()
            self.world.restore(self.grid[k])
            if abs(flex - k * STEP_DEG) > 1e-9:
                self._turned(flex)
            self.lies[flex] = self.world.lines(inside=True)[0]
        return self.lies[flex]


@functools.lru_cache(maxsize=None)
def _swept(key, length):
    import ast
    return _Swept(ast.literal_eval(key), length)


def _key(span):
    """`span` as a key: its numbers as plain floats, in tuples."""
    def plain(q):
        if isinstance(q, str):
            return q
        return tuple(plain(v) for v in q) if isinstance(q, (tuple, list)) or getattr(q, 'ndim', 0) else float(q)
    return repr({k: plain(v) for k, v in sorted(span.items()) if k in ('kind', 'from', 'to', 'solids')})


def turns(span):
    """The joint's turns its hose is laid at, deg: its travel's ends and each STEP_DEG between them."""
    lo, hi = (float(q) for q in span['travel'])
    steps = [k * STEP_DEG for k in range(int(math.ceil(lo / STEP_DEG)), int(math.floor(hi / STEP_DEG)) + 1)]
    return sorted(set(steps) | {lo, hi})


def reach(span):
    """The farthest the span's fittings' mouths stand apart through its joint's travel, m: at each degree."""
    import numpy as np
    from tools.render import knee_mount as km
    lo, hi = (float(q) for q in span['travel'])
    a, b = np.asarray(span['from'][1], float), np.asarray(span['to'][1], float)
    return max(float(np.linalg.norm(b @ km._rx(f).T - a)) for f in np.r_[np.arange(lo, hi, 1.0), hi])


def long(span):
    """The span's hose's length, m: its fittings' farthest (`reach`) and SPARE_M."""
    return round(reach(span) + SPARE_M, 4)


def least(span):
    """The least radius the span's hose bends at through its joint's travel (`turns`), m."""
    swept = _swept(_key(span), long(span))
    return min(_radius(swept.lie(flex)) for flex in turns(span))


def path(span, flex):
    """(k, 3): the span's hose at `flex` deg, the sketch's frame, from one fitting's mouth to the other's."""
    from machine.world import hoses
    return hoses.spline(_swept(_key(span), long(span)).lie(float(flex))[1:-1], 6)
