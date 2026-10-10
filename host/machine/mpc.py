"""Her balance a second ahead, one convex QP a way of standing: her ZMP inside her soles, a step free.

    out = plan(xi, omega, feet)                  # standing: stand on, or the cheapest step
    out = plan(xi, omega, feet, ('left', 0.18))  # a step in flight: its landing re-aimed
    out['p'], out['step'], out['xi_end']         # the ZMP now (x, z); (side, s to its landing,
                                                 # where (x, z)) or None; the DCM at the end

Her capture point (the DCM) runs from her ZMP as xi' = omega (xi - p): over the intervals DTS,
the ZMP constant in each, it is linear in them. A hypothesis is a schedule of supports: standing,
the hull of both soles all along; stepping, the standing sole from the call to the landing at
r, then the landed sole round r - r free within the standing foot's reach. Each is a QP (`qp.solve`): the
ZMP in its support (hard), the DCM at the end inside the last support (soft, TERMINAL_W - a
second's capture), the ZMP near its support's middle, r near a stance beside the standing sole,
the ZMP's moves smooth. She steps where standing cannot capture her (its end's slack) and a step can: the
cheapest. World frame: x her left, z ahead.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np  # noqa: E402

from machine import qp  # noqa: E402
from machine.figure import SOLE_HALF  # noqa: E402
from machine.gait import BALL, HEEL  # noqa: E402

#: The horizon, the user's second: intervals of 0.05 s for half of it, then 0.1 - at 20 of 0.05 a
#: plan of five hypotheses took 13-25 ms, more than its 20 ms tick.
DTS = np.array([0.05] * 10 + [0.1] * 5)
N = len(DTS)
STARTS = np.concatenate([[0.0], np.cumsum(DTS)])

#: A sole's support about its middle: half its width and half its length less MARGIN_M - at 0.02
#: she stepped where the stack alone stood on its ankles and hips, 60 N from 8 ways 22 of 24
#: and 44 steps where 24 and none (2026-10-10).
MARGIN_M = 0.005
HALF = (SOLE_HALF - MARGIN_M, (BALL + HEEL) / 2.0 - MARGIN_M)

#: The weights: the ZMP off its support's middle (1/m^2), the landing off the foot's place,
#: the DCM at the end off its support's middle, a ZMP's move between intervals; the end's
#: slack past its support.
MIDDLE_W, REACH_W, END_W, MOVE_W, TERMINAL_W = 1.0, 0.5, 1.0, 0.1, 1e4

#: A walk's DCM at the end off its periodic place (`periodic`), and off where her pace takes it
#: along her way, their weights: planned to stop on its second step, her feet landed wider each
#: step - 0.26, -0.16, 0.32 m - and she fell aside; unpaced, asked 0.5 m/s she went 0.21.
GOING_W, PACE_W = 10.0, 10.0

#: A step's reach from the standing sole's middle, its frame: across (out from it) and along;
#: its landing priced off a stance WIDTH_M beside it - priced off where the foot was, after a
#: 46 cm lunge her rear foot stepped 8 cm and 8 steps more followed her down (120 N, 2026-10-10).
ACROSS_M, ALONG_M, WIDTH_M = (0.12, 0.45), (-0.40, 0.50), 0.19

#: A step's swing, s, from its call: its sole lifted at once, the ZMP on the standing one - held
#: on its sole while it unloaded, its soft contact kept it pressed and she leaned onto it (80 N
#: from behind, 2026-10-10).
SWINGS = (0.20, 0.30)

#: Standing is captured if its end's slack is under CAPTURED_M.
CAPTURED_M = 1e-3

SIDES = ('left', 'right')
OTHER = {'left': 'right', 'right': 'left'}


def _axes(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c, -s]), np.array([s, c])


def box(centre, yaw, half=HALF):
    """(normals, offsets, middle) of a sole's rectangle in the floor's plane: n . p <= d."""
    across, along = _axes(yaw)
    normals = np.array([across, -across, along, -along])
    centre = np.asarray(centre, float)
    return normals, normals @ centre + np.array([half[0], half[0], half[1], half[1]]), centre


def corners(centre, yaw, half=HALF):
    across, along = _axes(yaw)
    return [np.asarray(centre, float) + i * half[0] * across + j * half[1] * along
            for i in (-1, 1) for j in (-1, 1)]


def hull(points):
    """(normals, offsets, middle) of the points' convex hull (monotone chain)."""
    pts = sorted(map(tuple, points))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    ring = lower[:-1] + upper[:-1]
    normals, offsets = [], []
    for a, b in zip(ring, ring[1:] + ring[:1]):
        n = np.array([b[1] - a[1], a[0] - b[0]])
        n /= np.linalg.norm(n)
        normals.append(n)
        offsets.append(float(n @ np.array(a)))
    return np.array(normals), np.array(offsets), np.mean(np.array(ring), axis=0)


def reach(yaw, side):
    """(normals, offsets) of where the `side` foot may land from a standing sole's middle, about it."""
    across, along = _axes(yaw)
    across = across * (1.0 if side == 'left' else -1.0)
    return (np.array([across, -across, along, -along]),
            np.array([ACROSS_M[1], -ACROSS_M[0], ALONG_M[1], -ALONG_M[0]]))


def beside(yaw, side, ahead=0.0):
    """The `side` foot's stance from a standing sole's middle, about it: WIDTH_M across, `ahead`
    m along."""
    across, along = _axes(yaw)
    return (WIDTH_M if side == 'left' else -WIDTH_M) * across + ahead * along


def _solve(xi, omega, supports, steps=(), going=None):
    """(cost, ZMPs (N, 2), landings [(2,)], the DCM at the end, the end's slack m) of a schedule:
    `supports` an interval each - (normals, offsets, middle), or j, landing j's sole round it;
    `steps` a landing each, (its sole's yaw, its reach about its anchor (normals, offsets), its
    place about its anchor priced, the anchor: a fixed (x, z) or the landing before); `going`
    (j, offset, (along, ahead)) a walk's end, the DCM `offset` from landing j and `ahead` m
    along - else it ends captured, inside the last support."""
    a = np.exp(omega * DTS)
    after = np.concatenate([np.cumprod(a[::-1])[::-1][1:], [1.0]])
    gamma = after * (1.0 - a)
    R, S = 2 * N, 2 * N + 2 * len(steps)
    nv = S + 4
    H, g, rows, rhs = np.zeros((nv, nv)), np.zeros(nv), [], []
    boxes = [box((0.0, 0.0), st[0]) for st in steps]

    def at(n, k=0, land=None, end=False):
        """A row n . (the ZMP of interval k, or the DCM at the end) less n . r_land."""
        row = np.zeros(nv)
        if end:
            row[:N], row[N:2 * N] = n[0] * gamma, n[1] * gamma
        else:
            row[k], row[N + k] = n
        if land is not None:
            row[R + 2 * land:R + 2 * land + 2] -= n
        return row

    def near(row, want, w):
        """w |row . y - want|^2 into the cost."""
        H[:, :] += w * np.outer(row, row)
        g[:] -= w * want * row

    def support(sup):
        return (boxes[sup], sup) if isinstance(sup, int) else (sup, None)

    for k, sup in enumerate(supports):
        (normals, offs, mid), land = support(sup)
        for n, d in zip(normals, offs):
            rows.append(at(n, k, land))
            rhs.append(d)
        for axis in (0, 1):
            near(at(np.eye(2)[axis], k, land), 0.0 if land is not None else mid[axis], MIDDLE_W)
    for k in range(1, N):
        for axis in (0, 1):
            row = np.zeros(nv)
            row[axis * N + k], row[axis * N + k - 1] = 1.0, -1.0
            near(row, 0.0, MOVE_W)
    xi0 = float(np.prod(a)) * np.asarray(xi, float)
    (normals, offs, mid), land = support(supports[-1])
    if going is None:
        for i, (n, d) in enumerate(zip(normals, offs)):
            row = at(n, land=land, end=True)
            row[S + min(i, 3)] = -1.0
            rows.append(row)
            rhs.append(d - float(n @ xi0))
        for axis in (0, 1):
            near(at(np.eye(2)[axis], land=land, end=True),
                 (0.0 if land is not None else mid[axis]) - xi0[axis], END_W)
    else:
        j, offset, pace = going
        for axis in (0, 1):
            near(at(np.eye(2)[axis], land=j, end=True), offset[axis] - xi0[axis], GOING_W)
        # where her pace takes her capture point over the horizon, along her way
        along, ahead = pace
        near(at(along, end=True), ahead - float(along @ xi0), PACE_W)
    for i in range(4):
        H[S + i, S + i] += TERMINAL_W
        rows.append(-np.eye(nv)[S + i])
        rhs.append(0.0)
    for j, (_yaw, (rn, rd), place, anchor) in enumerate(steps):
        # r_j - anchor: about a fixed point, or about the landing before
        rel = np.zeros((2, nv))
        rel[:, R + 2 * j:R + 2 * j + 2] = np.eye(2)
        off = np.zeros(2)
        if isinstance(anchor, int):
            rel[:, R + 2 * anchor:R + 2 * anchor + 2] -= np.eye(2)
        else:
            off = np.asarray(anchor, float)
        for axis in (0, 1):
            near(rel[axis], place[axis] + off[axis], REACH_W)
        for n, d in zip(rn, rd):
            rows.append(n @ rel)
            rhs.append(d + float(n @ off))
    H += 1e-9 * np.eye(nv)
    y, _held, _u = qp.solve(H, g, np.array(rows), np.array(rhs))
    if y is None:
        return math.inf, None, [], None, math.inf
    p = np.stack([y[:N], y[N:2 * N]], axis=1)
    end = xi0 + np.array([gamma @ y[:N], gamma @ y[N:2 * N]])
    return (float(0.5 * y @ H @ y + g @ y), p,
            [y[R + 2 * j:R + 2 * j + 2].copy() for j in range(len(steps))], end,
            float(y[S:S + 4].max()))


def _at(seconds):
    """The first interval starting at or after `seconds`."""
    return int(min(N, np.searchsorted(STARTS[:-1], seconds - 1e-9)))


def _steps(feet, side, first, then=None, ahead=0.0, both_s=0.0):
    """(supports, steps) of `side` landing in `first` s - and the other `then` s after, if asked
    - each placed `ahead` m along its standing sole's heading; both soles bearing `both_s` s
    before it lifts."""
    centre, yaw = feet[OTHER[side]]
    k, kb = max(1, _at(first)), min(_at(both_s), max(0, _at(first) - 1))
    supports = ([hull(corners(*feet['left']) + corners(*feet['right']))] * kb
                + [box(centre, yaw)] * (k - kb))
    steps = [(feet[side][1], reach(yaw, side), beside(yaw, side, ahead), centre)]
    if then is None:
        return supports + [0] * (N - k), steps
    k2 = max(k + 1, _at(first + then))
    steps.append((yaw, reach(feet[side][1], OTHER[side]),
                  beside(feet[side][1], OTHER[side], ahead), 0))
    return supports + [0] * (k2 - k) + [1] * (N - k2), steps


def periodic(omega, step_s, length, feet, side, landed_s):
    """Where a walk's DCM is about the `side` sole landed `landed_s` s into the horizon at its
    end, the LIPM's periodic gait: (w/2) tanh(omega T/2) off her midline at a landing, inside the
    new sole, and l/(e^(omega T) - 1) ahead of it, run on e^(omega u) u past it."""
    e = math.exp(omega * step_s)
    grow = math.exp(omega * max(0.0, STARTS[-1] - landed_s))
    across, along = _axes(feet[side][1])
    inward = -1.0 if side == 'left' else 1.0
    return grow * (inward * WIDTH_M / 2.0 * (1.0 - math.tanh(omega * step_s / 2.0)) * across
                   + length / (e - 1.0) * along)


def plan(xi, omega, feet, flight=None, walk=None):
    """{p, step, xi_end, cost, captured} for the capture point `xi` (x, z) at `omega`: `feet`
    {side: (sole's middle (x, z), yaw)}; `flight` (side, s to its landing) a step under way;
    `walk` (m/s, s a step, the side to step next, s on both soles before it lifts): her walk's
    steps, two in the horizon."""
    if walk is not None:
        speed, step_s, nxt, both_s = walk
        side, first = flight if flight is not None else (nxt, step_s)
        supports, steps = _steps(feet, side, first, step_s, speed * step_s, both_s)
        along = _axes(feet[OTHER[side]][1])[1]
        cost, p, r, end, slack = _solve(xi, omega, supports, steps,
                                        (1, periodic(omega, step_s, speed * step_s, feet,
                                                     OTHER[side], first + step_s),
                                         (along, float(along @ np.asarray(xi, float))
                                          + speed * float(STARTS[-1]))))
        centre = feet[OTHER[side]][0]
        return {'p': p[0] if p is not None else np.asarray(centre, float),
                'step': (side, first, r[0] if r else np.asarray(feet[side][0])),
                'xi_end': end, 'cost': cost, 'captured': slack < CAPTURED_M}
    if flight is not None:
        side, left_s = flight
        supports, steps = _steps(feet, side, left_s)
        cost, p, r, end, slack = _solve(xi, omega, supports, steps)
        return {'p': p[0] if p is not None else np.asarray(feet[OTHER[side]][0], float),
                'step': (side, left_s, r[0] if r else np.asarray(feet[side][0])),
                'xi_end': end, 'cost': cost, 'captured': slack < CAPTURED_M}
    both = hull(corners(*feet['left']) + corners(*feet['right']))
    cost, p, _r, end, slack = _solve(xi, omega, [both] * N)
    best = {'p': p[0] if p is not None else both[2], 'step': None, 'xi_end': end, 'cost': cost,
            'captured': slack < CAPTURED_M}
    if slack < CAPTURED_M:
        return best
    worst = slack
    for side in SIDES:
        for swing in SWINGS:
            supports, steps = _steps(feet, side, swing)
            c, p, r, e, sl = _solve(xi, omega, supports, steps)
            if p is not None and r and (sl, c) < (worst, best['cost']):
                worst = sl
                best = {'p': p[0], 'step': (side, swing, r[0]), 'xi_end': e,
                        'cost': c, 'captured': sl < CAPTURED_M}
    return best
