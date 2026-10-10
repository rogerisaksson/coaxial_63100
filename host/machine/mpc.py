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
second's capture), the ZMP near its support's middle, r near where the foot is, the ZMP's moves
smooth. She steps where standing cannot capture her (its end's slack) and a step can: the
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

#: A sole's support about its middle: half its width and half its length less MARGIN_M.
MARGIN_M = 0.02
HALF = (SOLE_HALF - MARGIN_M, (BALL + HEEL) / 2.0 - MARGIN_M)

#: The weights: the ZMP off its support's middle (1/m^2), the landing off the foot's place,
#: the DCM at the end off its support's middle, a ZMP's move between intervals; the end's
#: slack past its support.
MIDDLE_W, REACH_W, END_W, MOVE_W, TERMINAL_W = 1.0, 0.5, 1.0, 0.1, 1e4

#: A step's reach from the standing sole's middle, its frame: across (out from it) and along.
ACROSS_M, ALONG_M = (0.12, 0.45), (-0.40, 0.50)

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


def reach(centre, yaw, side):
    """(normals, offsets) of where the `side` foot may land from the standing sole's middle."""
    across, along = _axes(yaw)
    across = across * (1.0 if side == 'left' else -1.0)
    o = np.asarray(centre, float)
    return (np.array([across, -across, along, -along]),
            np.array([across @ o + ACROSS_M[1], -(across @ o) - ACROSS_M[0],
                      along @ o + ALONG_M[1], -(along @ o) - ALONG_M[0]]))


def _solve(xi, omega, supports, step=None):
    """(cost, ZMPs (N, 2), landing (2,) or None, the DCM at the end, the end's slack m) of a
    schedule: `supports` an interval each, (normals, offsets, middle) or None - the landed sole
    round the landing; `step` (the landed sole's yaw, its reach (normals, offsets), where the
    foot is) where one lands."""
    a = np.exp(omega * DTS)
    after = np.concatenate([np.cumprod(a[::-1])[::-1][1:], [1.0]])
    gamma = after * (1.0 - a)
    nr = 2 if step else 0
    R, S = 2 * N, 2 * N + nr
    nv = S + 4
    H, g, rows, rhs = np.zeros((nv, nv)), np.zeros(nv), [], []
    landed = box((0.0, 0.0), step[0] if step else 0.0)

    def at(n, k=0, land=False, end=False):
        """A row n . (the ZMP of interval k, or the DCM at the end) less n . r if `land`."""
        row = np.zeros(nv)
        if end:
            row[:N], row[N:2 * N] = n[0] * gamma, n[1] * gamma
        else:
            row[k], row[N + k] = n
        if land:
            row[R:R + 2] = -n
        return row

    def near(row, want, w):
        """w |row . y - want|^2 into the cost."""
        H[:, :] += w * np.outer(row, row)
        g[:] -= w * want * row

    for k, sup in enumerate(supports):
        normals, offs, mid = sup if sup is not None else landed
        for n, d in zip(normals, offs):
            rows.append(at(n, k, land=sup is None))
            rhs.append(d)
        for axis in (0, 1):
            e = np.eye(2)[axis]
            near(at(e, k, land=sup is None), 0.0 if sup is None else mid[axis], MIDDLE_W)
    for k in range(1, N):
        for axis in (0, 1):
            row = np.zeros(nv)
            row[axis * N + k], row[axis * N + k - 1] = 1.0, -1.0
            near(row, 0.0, MOVE_W)
    xi0 = float(np.prod(a)) * np.asarray(xi, float)
    last = supports[-1]
    normals, offs, mid = last if last is not None else landed
    for i, (n, d) in enumerate(zip(normals, offs)):
        row = at(n, land=last is None, end=True)
        row[S + min(i, 3)] = -1.0
        rows.append(row)
        rhs.append(d - float(n @ xi0))
    for axis in (0, 1):
        e = np.eye(2)[axis]
        near(at(e, land=last is None, end=True),
             (0.0 if last is None else mid[axis]) - xi0[axis], END_W)
    for i in range(4):
        H[S + i, S + i] += TERMINAL_W
        rows.append(-np.eye(nv)[S + i])
        rhs.append(0.0)
    if step:
        _yaw, (rn, rd), foot = step
        near(np.eye(nv)[R], foot[0], REACH_W)
        near(np.eye(nv)[R + 1], foot[1], REACH_W)
        for n, d in zip(rn, rd):
            row = np.zeros(nv)
            row[R:R + 2] = n
            rows.append(row)
            rhs.append(d)
    H += 1e-9 * np.eye(nv)
    y, _held, _u = qp.solve(H, g, np.array(rows), np.array(rhs))
    if y is None:
        return math.inf, None, None, None, math.inf
    p = np.stack([y[:N], y[N:2 * N]], axis=1)
    end = xi0 + np.array([gamma @ y[:N], gamma @ y[N:2 * N]])
    return (float(0.5 * y @ H @ y + g @ y), p, y[R:R + 2].copy() if step else None, end,
            float(y[S:S + 4].max()))


def _at(seconds):
    """The first interval starting at or after `seconds`."""
    return int(min(N, np.searchsorted(STARTS[:-1], seconds - 1e-9)))


def plan(xi, omega, feet, flight=None):
    """{p, step, xi_end, cost, captured} for the capture point `xi` (x, z) at `omega`: `feet`
    {side: (sole's middle (x, z), yaw)}; `flight` (side, s to its landing) a step under way."""
    if flight is not None:
        side, left_s = flight
        centre, yaw = feet[OTHER[side]]
        k = max(1, _at(left_s))
        cost, p, r, end, slack = _solve(xi, omega, [box(centre, yaw)] * k + [None] * (N - k),
                                        (feet[side][1], reach(centre, yaw, side), feet[side][0]))
        return {'p': p[0] if p is not None else np.asarray(centre, float),
                'step': (side, left_s, r if r is not None else np.asarray(feet[side][0])),
                'xi_end': end, 'cost': cost, 'captured': slack < CAPTURED_M}
    both = hull(corners(*feet['left']) + corners(*feet['right']))
    cost, p, _r, end, slack = _solve(xi, omega, [both] * N)
    best = {'p': p[0] if p is not None else both[2], 'step': None, 'xi_end': end, 'cost': cost,
            'captured': slack < CAPTURED_M}
    if slack < CAPTURED_M:
        return best
    worst = slack
    for side in SIDES:
        centre, yaw = feet[OTHER[side]]
        for swing in SWINGS:
            k = max(1, _at(swing))
            c, p, r, e, sl = _solve(xi, omega, [box(centre, yaw)] * k + [None] * (N - k),
                                    (feet[side][1], reach(centre, yaw, side), feet[side][0]))
            if p is not None and r is not None and (sl, c) < (worst, best['cost']):
                worst = sl
                best = {'p': p[0], 'step': (side, swing, r), 'xi_end': e,
                        'cost': c, 'captured': sl < CAPTURED_M}
    return best
