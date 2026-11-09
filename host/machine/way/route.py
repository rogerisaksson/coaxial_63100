"""Her way round her things: A* over their cells' costs (`machine.way.obstacles`), arcs at corners.

    cost = route.costs(things, floor, seen, slot, face)   # (nx, nz) a metre's cost through each cell, inf blocked
    cells = route.blocked(things, floor)             # her map's cells she may not stand in
    corners = route.way((x, z), to, cost)            # [(x, z)] from her place round them, `to` last
    stops, curve, held = route.planned((x, z), yaw, still, to, cost, held)   # her stops, the curve to the first
    k, aim, along, there, pace, bend = route.ahead(curve, k, (x, z))   # her place, aim, headings, speed, bend
    barred = route.barred(cells, [(x, z), ..])       # whether her way left is blocked past its first free cell
    d = route.staged((x, z), out, most, cells)       # m out along a dock's normal she stands free at

Her planner runs these apart (`machine.way.planner`).
"""
import bisect
import math

from machine.way import obstacles
from machine.law import rhythm, se2
from machine.eyes import sight, voxels
#: Setting off she walks while her heading is CONE off her leg at most, her walk's arc (WALK_SPEED /
#: YAW_RATE round) a cell off it at most.
CONE = math.acos(1.0 - voxels.CELL_M * se2.YAW_RATE / se2.WALK_SPEED)

#: Her walk's arc told, ARC_M round; her aim AIM_M on along her way, where pure pursuit on that arc
#: runs a cell inside it (ARC_M - sqrt(ARC_M^2 - AIM_M^2) = CELL_M); a stage STRAIGHT_M out: her aim,
#: her run-out STOP_M and a cell - told to stand mid-turn on (0.42, 0.03) she went on 0.5 rad off
#: her heading, 0.82 m into the chair's margin.
ARC_M = se2.WALK_SPEED / se2.YAW_RATE
AIM_M = ARC_M * math.sin(CONE)
STRAIGHT_M = AIM_M + se2.STOP_M + voxels.CELL_M
#: Her walking turn as she makes it, rad/s: told YAW_RATE, her heading turns her stance's share of it
#: (`[gait.steer]` stance_turn) - told 0.7 rad/s she turned 0.46-0.50 at 0.25-0.68 m/s, her stride
#: her speed told (`rhythm.modulated`), so her arc is her speed over TURN: her walk's 1.41 m round,
#: wider than the lodge's ways - a B-spline at it fitted none of 56 plans and she stood at each corner.
TURN = se2.YAW_RATE * se2.STEER['stance_turn']
#: Her braking, m/s^2 - her walk's speed run out over STOP_M - and her run-out's time, s; her slowest
#: walk, m/s: told slower she turns on the spot (`[gait.rhythm]` spin_still_m_s).
BRAKE, RUN_OUT_S = se2.WALK_SPEED ** 2 / (2.0 * se2.STOP_M), se2.STOP_M / se2.WALK_SPEED
SLOWEST = rhythm.R['spin_still_m_s']


def _cell(x, z):
    return (int(math.floor((x - voxels.X_M[0]) / voxels.CELL_M)), int(math.floor((z - voxels.Z_M[0]) / voxels.CELL_M)))


def _centre(i, j):
    return (voxels.X_M[0] + (i + 0.5) * voxels.CELL_M, voxels.Z_M[0] + (j + 0.5) * voxels.CELL_M)


def _free(cells, cell):
    return 0 <= cell[0] < cells.shape[0] and 0 <= cell[1] < cells.shape[1] and not cells[cell]


def _grid():
    """(nx * nz, 2) her map's cells' centres (x, z), and its shape."""
    np = sight._np()
    nx = int(round((voxels.X_M[1] - voxels.X_M[0]) / voxels.CELL_M))
    nz = int(round((voxels.Z_M[1] - voxels.Z_M[0]) / voxels.CELL_M))
    x, z = np.meshgrid(voxels.X_M[0] + (np.arange(nx) + 0.5) * voxels.CELL_M,
                       voxels.Z_M[0] + (np.arange(nz) + 0.5) * voxels.CELL_M, indexing='ij')
    return np.column_stack([x.ravel(), z.ravel()]), (nx, nz)


def costs(things, floor, seen=None, slot=-1, face=None):
    """(nx, nz) a metre's cost walked through each of her map's cells among `things` (her registry's
    boxes) over her soles' `floor` (`obstacles.costed`): inf where she may not stand, `seen` (nx, nz) 1
    where her eyes have seen - the whole room without -, `slot` the thing she goes to, opened before
    the `face` she docks at, its outward normal (x, z), None where she docks at none. The door among
    them at its live pose: closed, no way out exists and `way`'s fallback straight line is the push;
    ajar, the look along her way rounds its panel - held out of her map she ground it onto its 1.92
    rad stop and its spring flung her at 1.66 m/s."""
    np = sight._np()
    at, shape = _grid()
    known = np.ones(len(at)) if seen is None else np.asarray(seen, float).reshape(-1)
    return obstacles.costed(at, things, floor, known, slot, face).reshape(shape)


def blocked(things, floor, slot=-1, face=None):
    """Her map's cells she may not stand in among `things` over `floor` (`costs`' inf)."""
    return sight._np().isinf(costs(things, floor, None, slot, face))


def way(at, to, cost):
    """[(x, z)] her corners from `at` (x, z) to `to` through `cost`'s free cells (`costs`), its least,
    `to` last; [`to`] where no way is found or either end is off her map."""
    np = sight._np()
    start, goal = _cell(*at), _cell(*to)
    if not all(0 <= c[0] < cost.shape[0] and 0 <= c[1] < cost.shape[1] for c in (start, goal)):
        return [tuple(to)]
    cells = np.isinf(cost)
    start = start if not cells[start] else sight._nearest_free(cells, start)
    goal = goal if not cells[goal] else sight._nearest_free(cells, goal)
    path = sight._astar(cost, start, goal)
    return [_centre(*c) for c in sight._pruned(cost, path)[1:-1]] + [tuple(to)] if path else [tuple(to)]


def barred(cells, pts):
    """Whether her way through `pts` [(x, z)], her place first, is blocked in `cells`: a blocked cell past
    a free one along its legs, a cell a sample. Its leading blocked run (her own cell inside a thing's
    grown disc as she docks) and its last point (a stage laid inside one, `staged`'s) are hers to stand
    in: checked every look they replanned her mid-stride at the chair and she perched unsure, and felled
    her at the door."""
    samples = [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
               for a, b in zip(pts, pts[1:])
               for n in (max(1, int(math.ceil(math.dist(a, b) / voxels.CELL_M))),)
               for k in range(n + 1)]
    free = [_free(cells, _cell(x, z)) for x, z in samples[:-1]]
    return any(not f and any(free[:k]) for k, f in enumerate(free))


def staged(dock, out, most, cells):
    """m along `out` (a dock's normal, unit (x, z)) from `dock` she stands free at: the point of the
    first run of free cells along it nearest `most` - in the open `most` itself; `most` her walk's
    stopping distance and the stride a turn in place drifts her. The run's middle staged the chair
    before the table 0.45 m out and the click's errand perched her on its corner (pelvis 0.742, sure
    0.48); at 0.70 she sits (0.586, 0.95), the lodge's start sitting either way (16.6, 17.6 s)."""
    steps = [k * sight.CELL_M for k in range(int(round(2.0 * most / sight.CELL_M)) + 1)]
    free = [_free(cells, _cell(dock[0] + s * out[0], dock[1] + s * out[1])) for s in steps]
    k0 = next((k for k, f in enumerate(free) if f), None)
    if k0 is None:
        return most
    k1 = next((k for k in range(k0, len(free)) if not free[k]), len(free)) - 1
    return min(most, max(steps[k0], min(steps[k1], most)))


def _aim(v):
    """m her aim runs on ahead of her at `v` m/s: where pure pursuit on her arc at that speed (v /
    YAW_RATE round) runs a cell inside it, a cell at least - AIM_M at her walk."""
    return math.sqrt(max(2.0 * voxels.CELL_M * v / se2.YAW_RATE - voxels.CELL_M ** 2, voxels.CELL_M ** 2))


def _arc(p, a, b, r):
    """[(x, z)] her arc of radius `r` round the corner `p` from its leg in along `a` to its leg out
    along `b` (units), a half cell apart, from its tangent on the one to its tangent on the other."""
    turn = math.atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1])
    tau, side = r * math.tan(0.5 * abs(turn)), math.copysign(1.0, turn)
    t0 = (p[0] - tau * a[0], p[1] - tau * a[1])
    c = (t0[0] - side * r * a[1], t0[1] + side * r * a[0])
    ox, oz = t0[0] - c[0], t0[1] - c[1]
    n = int(math.ceil(2.0 * r * abs(turn) / voxels.CELL_M)) + 1
    phis = [turn * k / max(n - 1, 1) for k in range(n)]
    return [(c[0] + ox * math.cos(f) - oz * math.sin(f), c[1] + ox * math.sin(f) + oz * math.cos(f)) for f in phis]


def _line(a, b):
    """[(x, z)] from `a` to `b` a half cell apart, both ends."""
    n = max(1, int(math.ceil(2.0 * math.dist(a, b) / voxels.CELL_M)))
    return [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n + 1)]


def _widest(cells, p, a, b, lo, hi):
    """The widest radius from `lo` to `hi` whose arc round `p` (`_arc`) lies in free `cells`, to half a
    cell; 0 where none does."""
    def free(r):
        return all(_free(cells, _cell(x, z)) for x, z in _arc(p, a, b, r))
    if hi < lo or free(hi):
        return hi
    if not free(lo):
        return 0.0
    while hi - lo > 0.5 * voxels.CELL_M:
        lo, hi = (0.5 * (lo + hi), hi) if free(0.5 * (lo + hi)) else (lo, 0.5 * (lo + hi))
    return lo


def _room(ell, tan, out, stop):
    """Her arc's widest radius its legs give a corner turning by 2 atan(`tan`): its leg in `ell`
    (`out` its share of it), its leg out `out` - to a `stop`, her run-out (RUN_OUT_S at TURN times the
    radius) kept before it."""
    return min(se2.WALK_SPEED / TURN, ell / tan, out / (tan + RUN_OUT_S * TURN * float(stop)))


def rounded(cells, pts):
    """(curve, last): her way through `pts` [(x, z)] - her place, `way`'s corners, its end - round each
    corner on the widest arc her map's free `cells` and its legs give (`_widest`, `_room`), her walk's
    (WALK_SPEED / TURN round) at most, walked at TURN times its radius. The curve (N, 5): x, z, its
    length, her speed and its bend (1/m) a half cell apart, braked into each arc and out of it at
    BRAKE, into its end - where she stands - to her SLOWEST walk; `last` the index in `pts` of the
    first corner no arc of her SLOWEST walk rounds - a stop,
    the curve's end, her run-out kept before it."""
    np = sight._np()
    p = np.asarray(pts, float)
    ell = np.hypot(*np.diff(p, axis=0).T)
    u = np.diff(p, axis=0) / np.maximum(ell, 1e-12)[:, None]
    tans = [max(math.tan(0.5 * abs(math.atan2(a[0] * b[1] - a[1] * b[0], a @ b))), 1e-12) for a, b in zip(u, u[1:])]
    tight, n, radii = SLOWEST / TURN, len(p) - 1, []
    for i in range(1, n):
        r = _room(ell[i - 1] * (0.5 + 0.5 * float(i == 1)), tans[i - 1], ell[i] * (0.5 + 0.5 * float(i == n - 1)), i == n - 1)
        r = _widest(cells, p[i], u[i - 1], u[i], tight, r)
        if r < tight:
            break
        radii.append(r)
    last = len(radii) + 1
    while radii and last < n:
        i = len(radii)
        r = _room(ell[i - 1] * (0.5 + 0.5 * float(i == 1)), tans[i - 1], ell[i], True)
        radii[-1] = _widest(cells, p[i], u[i - 1], u[i], tight, min(r, radii[-1]))
        if radii[-1] >= tight:
            break
        radii.pop()
        last -= 1
    pieces, at = [], tuple(p[0])
    for i, r in enumerate(radii, start=1):
        arc = _arc(p[i], u[i - 1], u[i], r)
        pieces += [(_line(at, arc[0]), se2.WALK_SPEED, 0.0), (arc, min(se2.WALK_SPEED, TURN * r), 1.0 / r)]
        at = arc[-1]
    pieces.append((_line(at, tuple(p[last])), se2.WALK_SPEED, 0.0))
    rows = []
    for samples, cap, bend in pieces:
        for x, z in samples:
            if rows and math.hypot(x - rows[-1][0], z - rows[-1][1]) < 1e-9:
                rows[-1][2], rows[-1][3] = min(rows[-1][2], cap), max(rows[-1][3], bend)
            else:
                rows.append([x, z, cap, bend])
    q = np.asarray(rows, float)
    ds = np.hypot(*np.diff(q[:, :2], axis=0).T)
    v = q[:, 2].copy()
    v[-1] = min(v[-1], SLOWEST)
    for k in range(1, len(v)):
        v[k] = min(v[k], math.sqrt(v[k - 1] ** 2 + 2.0 * BRAKE * ds[k - 1]))
    for k in range(len(v) - 2, -1, -1):
        v[k] = min(v[k], math.sqrt(v[k + 1] ** 2 + 2.0 * BRAKE * ds[k]))
    return np.column_stack([q[:, :2], np.r_[0.0, np.cumsum(ds)], v, q[:, 3]]), last


def planned(at, yaw, still, to, cost, warm=(), passed=0.0):
    """(stops, curve, held): where she stands still on her way from `at` (x, z) to `to` through `cost`
    (`costs`) - `way`'s corners from the first no arc rounds (`rounded`), `to` last, its leading corners
    within `passed` of her dropped -, the curve she walks to the first ((0, 5) where she walks straight
    at it: one leg, no corner rounded, or a blocked sample past a free one, `barred`'s - her own run
    inside a grown thing's hers, as she rises off a chair she stood a corner at a time); none held. `yaw`, `still` and `warm` her planner's ask's: she turns standing where she is off her
    way, as at a stop (`steer`)."""
    np = sight._np()
    cells = np.isinf(cost)
    here = tuple(at)
    corners = way(here, to, cost)
    corners = [p for k, p in enumerate(corners) if any(math.dist(here, q) > passed for q in corners[:k + 1])]
    pts = [here] + corners
    curve, last = rounded(cells, pts) if len(pts) > 2 else (np.zeros((0, 5)), len(pts) - 1)
    if last <= 1 or barred(cells, [tuple(q) for q in curve[:, :2]]):
        curve, last = np.zeros((0, 5)), min(last, 1)
    return pts[last:] if len(pts) > 1 else [], curve, ()


def ahead(curve, k, at):
    """(k, (x, z), along, there, pace, bend): her place on `curve` ((N, 5) `rounded`'s) - its sample
    nearest `at` from the k-th on, AIM_M along it at most -, her aim on from there as far as her speed
    there runs it (`_aim`), its end at most, the curve's heading at her place and at her aim, rad, her
    speed at her place, m/s, and its bend at her aim, 1/m. Its columns read as lists once a curve (`_columns`):
    bisect's insertions searchsorted's on its arclength, sorted; numpy's hypot and argmin for her place."""
    np = sight._np()
    x, z, s, pace, bend = _columns(curve)
    near = slice(k, bisect.bisect_right(s, s[k] + AIM_M))
    k = k + int(np.argmin(np.hypot(curve[near, 0] - at[0], curve[near, 1] - at[1])))
    n = len(s)
    j = min(bisect.bisect_left(s, s[k] + _aim(pace[k])), n - 1)
    (k1, k0), (j1, j0) = ((min(i + 1, n - 1), min(i, n - 2)) for i in (k, j))
    along, there = math.atan2(x[k1] - x[k0], z[k1] - z[k0]), math.atan2(x[j1] - x[j0], z[j1] - z[j0])
    return k, (x[j], z[j]), along, there, pace[k], max(bend[k], bend[j] * float(pace[k] < se2.WALK_SPEED))


def _columns(curve):
    """`curve`'s columns as lists of floats, kept while it is the same curve - each a planner's answer, never written
    in place: searched in numpy each pass, its column copied out of its rows for it, it took 29 us of her walk."""
    if _COLUMNS[0] is not curve:
        _COLUMNS[:] = curve, curve.T.tolist()
    return _COLUMNS[1]


#: The last curve and its columns (`_columns`).
_COLUMNS: list = [None, None]
