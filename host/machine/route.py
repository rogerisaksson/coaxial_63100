"""Her way round the furniture: A* round her floor's things grown by her radius, a curve along it.

    cells = route.blocked(g)                     # her map's cells she may not stand in
    corners = route.way(g, to, cells)            # [(x, z)] from where she stands round them, `to` last
    stops, curve, held = route.planned(g, to, cells, held)   # where she stands still, the curve to the first
    k, aim, along, there = route.ahead(curve, k, (x, z))     # her place, aim, its heading at each
    d = route.staged((x, z), out, most, cells)   # m out along a dock's normal she stands free at
    specs = route.docks()                        # how each errand docks
"""
import math

from machine import figure, room, se2, sight, stride, voxels

#: Her map's things grown by her radius (`voxels.RADIUS_M`, her figure's): setting off she walks while
#: her heading is CONE off her leg at most, her walk's arc (WALK_SPEED / YAW_RATE round) a cell off it
#: at most.
GROWN_M = voxels.RADIUS_M
CONE = math.acos(1.0 - voxels.CELL_M * se2.YAW_RATE / se2.WALK_SPEED)

#: Her way walked through its corners: a convex corridor round each segment, her walk's arc ARC_M
#: about it at most; a cubic B-spline held in them, a knot a stride at her walk; her aim AIM_M on
#: along it, where pure pursuit on her arc runs a cell inside it (ARC_M - sqrt(ARC_M^2 - AIM_M^2) =
#: CELL_M); its last STRAIGHT_M on the last segment's line - her aim on it, then told to stand a
#: cell before it she goes on STOP_M: told it mid-turn on (0.42, 0.03) she went on 0.5 rad off her
#: heading, 0.82 m into the chair's margin.
ARC_M = se2.WALK_SPEED / se2.YAW_RATE
AIM_M = ARC_M * math.sin(CONE)
STRAIGHT_M = AIM_M + se2.STOP_M + voxels.CELL_M
#: Where her heading is ARCED or more off her curve's at her aim she walks on her arc, its twist -
#: the nearer of a walk and her arc, half her arc's turn over her aim -, else by her heading: held
#: by her heading a 0.5 rad turn left her path on its line 2 s; arced on the curve's bend alone
#: her heading ran 0.3 rad past it; arced on her straight ways too, the chair held 25 of 58 where
#: 21 and the bed 16 of 52 where 17, 7 down where 4 (findings/errands.md).
ARCED = 0.5 * AIM_M / ARC_M
#: Her arc's acceleration, WALK_SPEED * YAW_RATE, as the polygon of SIDES inscribed in it: her arc
#: held a cell wider at most (cos(pi / SIDES) >= cos(CONE)).
SIDES = math.ceil(math.pi / CONE)


def docks():
    """How each errand docks, from her figure's, her stride's and the room's dimensions: kind ->
    (the dock's normal toward her stage, rad - the thing facing as the room lays it, a chair -z, a
    bed +x, the door's into the room; a chair's own where it has a back, `furniture.nearest` -; how
    far along the edge her hips may land off it, m; where she stages before it - her walk's stopping
    distance and the stride a turn in place drifts her; the door's run in -, m; the prop's place
    failing one found). The dock itself is the thing's edge along that normal (`errand._laid`)."""
    d_stage = se2.STOP_M + stride.STRIDE_M
    return {
        'sit_chair': (math.pi, room.SEAT[0] - figure.HIP_HALF, d_stage, room.CHAIR_AT),
        'lie_bed': (math.pi / 2.0, room.BED[2] - figure.HIP_HALF, d_stage, room.BED_AT),
        'exit_door': (math.pi, 0.5 * (room.DOOR_X[1] - room.DOOR_X[0]) - figure.HIP_HALF, 4.0 * stride.STRIDE_M,
                      (0.5 * (room.DOOR_X[0] + room.DOOR_X[1]), room.TO)),
    }


def _cell(x, z):
    return (int(math.floor((x - voxels.X_M[0]) / voxels.CELL_M)), int(math.floor((z - voxels.Z_M[0]) / voxels.CELL_M)))


def _centre(i, j):
    return (voxels.X_M[0] + (i + 0.5) * voxels.CELL_M, voxels.Z_M[0] + (j + 0.5) * voxels.CELL_M)


def _free(cells, cell):
    return 0 <= cell[0] < cells.shape[0] and 0 <= cell[1] < cells.shape[1] and not cells[cell]


def blocked(g):
    """Her map's cells she may not stand in: each of her things (`g['things']`, her registry's boxes)
    within `voxels.STAND_M` over the floor - she steps over lower, walks under higher -, GROWN_M and
    its blind spot's swell round. The door among them at its live pose: closed, no way out exists and
    `way`'s fallback straight line is the push; ajar, the look along her way rounds its panel - held
    out of her map she ground it onto its 1.92 rad stop and its spring flung her at 1.66 m/s."""
    np = sight._np()
    nx = int(round((voxels.X_M[1] - voxels.X_M[0]) / voxels.CELL_M))
    nz = int(round((voxels.Z_M[1] - voxels.Z_M[0]) / voxels.CELL_M))
    x, z = np.meshgrid(voxels.X_M[0] + (np.arange(nx) + 0.5) * voxels.CELL_M,
                       voxels.Z_M[0] + (np.arange(nz) + 0.5) * voxels.CELL_M, indexing='ij')
    out = np.zeros((nx, nz), dtype=bool)
    for p in g['things']:
        if len(p) < 4 or len(p[1]) < 3 or not hasattr(p[2], '__len__') or len(p[2]) < 3:
            continue
        _kind, c, half, turn = p[:4]
        if c[1] + half[1] < voxels.STAND_M[0] or c[1] - half[1] >= voxels.STAND_M[1]:
            continue
        dx, dz = x - c[0], z - c[2]
        along_x = np.abs(dx * turn[0][0] + dz * turn[2][0])
        along_z = np.abs(dx * turn[0][2] + dz * turn[2][2])
        grown = GROWN_M + (float(p[6]) if len(p) > 6 else 0.0)
        out |= (along_x <= half[0] + grown) & (along_z <= half[2] + grown)
    return out


def way(g, to, cells):
    """[(x, z)] her corners from where she stands to `to` through `cells`' free ones, `to` last; [`to`]
    where no way is found or either end is off her map."""
    d = g['world'].data
    start, goal = _cell(float(d.qpos[0]), float(d.qpos[2])), _cell(*to)
    if not all(0 <= c[0] < cells.shape[0] and 0 <= c[1] < cells.shape[1] for c in (start, goal)):
        return [tuple(to)]
    start = start if not cells[start] else sight._nearest_free(cells, start)
    goal = goal if not cells[goal] else sight._nearest_free(cells, goal)
    path = sight._astar(cells, start, goal)
    return [_centre(*c) for c in sight._pruned(cells, path)[1:-1]] + [tuple(to)] if path else [tuple(to)]


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


def centres(cells):
    """(N, 2) the centres (x, z) of `cells`' blocked ones."""
    np = sight._np()
    ij = np.argwhere(cells)
    return np.column_stack([sight.X_M[0] + (ij[:, 0] + 0.5) * sight.CELL_M, sight.Z_M[0] + (ij[:, 1] + 0.5) * sight.CELL_M])


def corridor(at, a, b):
    """(A, b): A p <= b the convex polygon round the segment `a` -> `b` holding no blocked cell, `at`
    their centres ((N, 2) x, z): the segment inflated till it meets the nearest centre, a half-plane
    there, again for the nearest centre left till none is left (DecompUtil's, its ellipse the
    inflated segment); the half-plane the cell's face toward the segment where the whole segment
    lies beyond it, else the inflated segment's tangent moved half a cell in - the cell's square
    out, not its centre - but never past the segment, which A* grazing a cell's corner may cross;
    within ARC_M of the segment, on her map. DecompUtil's ellipse, shrunk on a centre 5 cm off a
    way's middle, came out 20 times longer than wide, and its tangents at the cells behind her
    start held the corridor a cell wide all along (the bed from (-1.9, -1.0)); tangents alone, a
    segment 9 deg up the chair's slot cut the slot to a wedge (the chair from (-1.3, -1.0))."""
    np = sight._np()
    a, b = np.asarray(a, float), np.asarray(b, float)
    half = max(0.5 * math.dist(a, b), 1e-9)
    u = (b - a) / (2.0 * half)
    w, mid = np.array([-u[1], u[0]]), 0.5 * (a + b)
    x, y = (at - mid) @ u, (at - mid) @ w
    near = (np.abs(x) <= half + ARC_M + sight.CELL_M) & (np.abs(y) <= ARC_M + sight.CELL_M)
    pts, x = at[near], x[near]
    off = pts - (mid + np.clip(x, -half, half)[:, None] * u)
    gap = np.hypot(off[:, 0], off[:, 1])
    rows, offs, left = [], [], np.ones(len(pts), bool)
    while left.any():
        k = np.flatnonzero(left)[np.argmin(gap[left])]
        tangent = off[k] / max(float(gap[k]), 1e-12)
        face = np.where(np.arange(2) == np.argmax(np.abs(tangent)), np.sign(tangent), 0.0)
        fits = max(float(face @ a), float(face @ b)) <= float(face @ pts[k]) - 0.5 * sight.CELL_M
        n = (tangent, face)[int(fits)]
        side = pts @ n
        rows.append(n)
        offs.append(float(side[k]) - min(0.5 * sight.CELL_M * (abs(n[0]) + abs(n[1])),
                                         max(float(side[k]) - max(float(n @ a), float(n @ b)), 0.0)))
        # its own side, one product: n @ pts[k] rounded apart from (pts @ n)[k] and kept it for ever
        left &= side < side[k]
    box = np.array([u, -u, w, -w, (1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)])
    reach = [u @ mid + half + ARC_M, -(u @ mid) + half + ARC_M, w @ mid + ARC_M, -(w @ mid) + ARC_M,
             sight.X_M[1], -sight.X_M[0], sight.Z_M[1], -sight.Z_M[0]]
    return np.vstack([np.reshape(rows, (-1, 2)), box]), np.r_[offs, reach]


def _basis(knots, p, ts):
    """(len(ts), len(knots) - p - 1): the degree-`p` B-spline basis on `knots` at `ts` (Cox-de Boor),
    the last knot in the last span."""
    np = sight._np()
    t, ts = np.asarray(knots, float), np.asarray(ts, float)
    nb = len(t) - 1
    span = np.minimum(np.searchsorted(t, ts, side='right') - 1, int(np.flatnonzero(t[:-1] < t[1:])[-1]))
    out = (np.arange(nb)[None, :] == span[:, None]).astype(float)
    for q in range(1, p + 1):
        lo, hi, lo1, hi1 = t[:nb - q], t[q:nb], t[1:nb - q + 1], t[q + 1:]
        left = np.divide(ts[:, None] - lo, hi - lo, out=np.zeros((len(ts), nb - q)), where=hi > lo)
        right = np.divide(hi1 - ts[:, None], hi1 - lo1, out=np.zeros((len(ts), nb - q)), where=hi1 > lo1)
        out = left * out[:, :nb - q] + right * out[:, 1:]
    return out


def _derived(knots, p):
    """(N - 1, N): a degree-`p` B-spline's derivative's control points on `knots` from its N own."""
    np = sight._np()
    t = np.asarray(knots, float)
    n = len(t) - p - 1
    k = p / (t[p + 1:p + n] - t[1:n])
    out = np.zeros((n - 1, n))
    out[np.arange(n - 1), np.arange(n - 1)], out[np.arange(n - 1), np.arange(1, n)] = -k, k
    return out


def spline(points, polygons, heading=None, warm=()):
    """(curve, held): a clamped uniform cubic B-spline from points[0] to points[-1] in `polygons`,
    one round each segment between them - (N, 3) x, z and arc length a half cell apart, None where
    none fits or it bends past her arc (1 / ARC_M) -, and the rows its QP (`laid`, `qp.solve` from
    `warm`) held. She walks it by its shape, not its time: its speed held at WALK_SPEED cos(a
    stride of her arc) as well, each span along the way's or a first solve's own direction, none
    fitted beside the table's corner (the chair from (-1.3, -1.0)) where the first bent 0.89/m at
    most, its speed 0.52 to 1.18 m/s."""
    np = sight._np()
    from machine import qp
    p = laid(points, polygons, heading)
    y, held, _u = qp.solve(p['H'], p['g'], p['A'], p['b'], warm)
    if y is None:
        return None, held
    ctrl, knots = (p['M'] @ y + p['x0']).reshape(-1, 2), p['knots']
    ts = np.linspace(0.0, knots[-1], math.ceil(2.0 * p['way_m'] / sight.CELL_M) + 1)
    on = _basis(knots, 3, ts) @ ctrl
    d1 = _derived(knots, 3) @ ctrl
    v, a = _basis(knots[1:-1], 2, ts) @ d1, _basis(knots[2:-2], 1, ts) @ (_derived(knots[1:-1], 2) @ d1)
    bent = float(np.max(np.abs(v[:, 0] * a[:, 1] - v[:, 1] * a[:, 0]) / np.maximum(np.hypot(*v.T), 1e-12) ** 3))
    curve = np.column_stack([on, np.r_[0.0, np.cumsum(np.hypot(*np.diff(on, axis=0).T))]])
    return (curve, None)[int(bent > 1.0 / ARC_M)], held


def laid(points, polygons, heading=None):
    """`spline`'s QP, min 1/2 y'Hy + g'y with A y <= b, X = M y + x0 its control points: {'H', 'g', 'A',
    'b', 'M', 'x0', 'knots', 'way_m', 'groups' (its rows' counts: corridors, arc, straight)}. Timed
    at her walk a knot a stride, each span's four control points in one polygon - by the way's arc
    length, her arc swapped in at each corner - so by the convex hull it is in them; its
    acceleration within her arc's WALK_SPEED * YAW_RATE; it leaves along `heading` where one is
    given and ends STRAIGHT_M on the last segment's line. Least squared speed (its length) and
    jerk, each over her walk's (WALK_SPEED, WALK_SPEED * YAW_RATE^2)."""
    np = sight._np()
    pts = np.asarray(points, float)
    ell = np.hypot(*np.diff(pts, axis=0).T)
    u = np.diff(pts, axis=0) / ell[:, None]
    turn = np.arctan2(u[:-1, 0] * u[1:, 1] - u[:-1, 1] * u[1:, 0], (u[:-1] * u[1:]).sum(axis=1))
    c = np.arange(len(turn))
    room_in = np.where(c == 0, ell[:-1], 0.5 * ell[:-1])
    room_out = np.where(c == len(turn) - 1, np.maximum(ell[1:] - STRAIGHT_M, 0.0), 0.5 * ell[1:])
    half = 0.5 * np.abs(turn)
    tau = np.minimum(ARC_M * np.tan(half), np.minimum(room_in, room_out))
    straight = ell - np.r_[0.0, tau] - np.r_[tau, 0.0]
    bend = 2.0 * tau * half / np.maximum(np.tan(half), 1e-12)
    lengths = np.r_[np.column_stack([straight[:-1], bend]).ravel(), straight[-1]]
    way_m = float(lengths.sum())
    last = math.ceil(STRAIGHT_M / stride.STRIDE_M)
    m = max(math.ceil(way_m / stride.STRIDE_M), last + 3)
    dt = way_m / (m * se2.WALK_SPEED)
    knots = np.r_[np.zeros(3), np.arange(m + 1) * dt, np.full(3, m * dt)]
    n1 = m + 3
    # each span's polygon by the way's arc length at its middle, a bend's first half the segment's before it
    mids = (np.arange(m) + 0.5) * way_m / m
    piece = np.minimum(np.searchsorted(np.cumsum(lengths), mids, side='right'), len(lengths) - 1)
    f = (mids - (np.cumsum(lengths) - lengths)[piece]) / np.maximum(lengths[piece], 1e-12)
    poly = piece // 2 + piece % 2 * (f >= 0.5)
    lead = np.asarray(heading if heading is not None else u[0], float)
    # the control points X = M y + x0: hers and its end fixed, her heading's where given, the last
    # spans' on the last segment's line
    free, line = np.arange(1 + int(heading is not None), m - last), np.arange(m - last, m + 2)
    M = np.zeros((2 * n1, 2 * len(free) + len(line)))
    M[2 * free, 2 * np.arange(len(free))], M[2 * free + 1, 2 * np.arange(len(free)) + 1] = 1.0, 1.0
    M[2 * line, 2 * len(free) + np.arange(len(line))] = -u[-1, 0]
    M[2 * line + 1, 2 * len(free) + np.arange(len(line))] = -u[-1, 1]
    x0 = np.tile(pts[0], n1)
    x0[[2, 3]] = pts[0] + lead * se2.WALK_SPEED * dt / 3.0
    x0[2 * line], x0[2 * line + 1], x0[-2:] = pts[-1, 0], pts[-1, 1], pts[-1]
    # least squared speed and jerk, each over her walk's
    d1, d2 = _derived(knots, 3), _derived(knots[1:-1], 2)
    jerk = _derived(knots[2:-2], 1) @ d2 @ d1
    nodes, weights = np.polynomial.legendre.leggauss(3)
    b2 = _basis(knots[1:-1], 2, ((np.arange(m)[:, None] + 0.5 + 0.5 * nodes) * dt).ravel())
    gram = (b2.T * np.tile(0.5 * dt * weights, m)) @ b2
    cost = np.kron(d1.T @ gram @ d1 / se2.WALK_SPEED ** 2
                   + dt * jerk.T @ jerk / (se2.WALK_SPEED * se2.YAW_RATE ** 2) ** 2, np.eye(2))
    # rows on X: corridors, her arc, the straight's length
    pairs = sorted({(i, int(poly[s])) for i in range(n1) for s in range(max(0, i - 3), min(i, m - 1) + 1)})
    walls = [np.kron(np.eye(n1)[i], polygons[k][0]) for i, k in pairs]
    reach = [polygons[k][1] for _i, k in pairs]
    sides = 2.0 * np.pi * np.arange(SIDES) / SIDES
    arc = ((d2 @ d1)[:, None, :, None] * np.column_stack([np.cos(sides), np.sin(sides)])[None, :, None, :]).reshape(-1, 2 * n1)
    end = np.kron(_basis(knots, 3, [(m - last) * dt])[0], u[-1])
    A = np.vstack(walls + [arc, end[None, :]])
    b = np.r_[np.concatenate(reach), np.full(len(arc), se2.WALK_SPEED * se2.YAW_RATE * math.cos(math.pi / SIDES)),
              u[-1] @ pts[-1] - STRAIGHT_M]
    return {'H': M.T @ cost @ M, 'g': M.T @ cost @ x0, 'A': A @ M, 'b': b - A @ x0, 'M': M, 'x0': x0,
            'knots': knots, 'way_m': way_m, 'groups': (sum(len(r) for r in reach), len(arc), 1)}


def _sharpest(points):
    """The index in `points` of its sharpest inner corner."""
    np = sight._np()
    u = np.diff(np.asarray(points, float), axis=0)
    u = u / np.hypot(*u.T)[:, None]
    return 1 + int(np.argmin((u[:-1] * u[1:]).sum(axis=1)))


def _run(cells, p, d, most):
    """m from `p` along `d` its cells run free, `most` at most, a half cell a step."""
    steps = [min(k * 0.5 * sight.CELL_M, most) for k in range(1, math.ceil(2.0 * most / sight.CELL_M) + 1)]
    return next((s0 for s0, s in zip([0.0] + steps, steps) if not _free(cells, _cell(p[0] + s * d[0], p[1] + s * d[1]))),
                steps[-1] if steps else 0.0)


def carried(cells, points):
    """[(a, b)] each segment of `points` carried past its corners by her arc's tangent there, ARC_M
    tan(turn / 2) and ARC_M at most, and a stride - the three control points a change of corridor
    shares reach a stride either side -, the last back STRAIGHT_M from its end at least, as far as
    their cells run free: the corridors' segments. Grown round the segment alone, the corridors met
    at the corner and no more - the tangent there caps them -, and no three control points fitted
    between (the chair's way from (-0.7, -1.0))."""
    np = sight._np()
    pts = np.asarray(points, float)
    ell = np.hypot(*np.diff(pts, axis=0).T)
    u = np.diff(pts, axis=0) / ell[:, None]
    turn = np.arctan2(u[:-1, 0] * u[1:, 1] - u[:-1, 1] * u[1:, 0], (u[:-1] * u[1:]).sum(axis=1))
    ext = ARC_M * np.minimum(np.tan(0.5 * np.abs(turn)), 1.0) + stride.STRIDE_M
    back, ahead = np.r_[0.0, ext], np.r_[ext, 0.0]
    back[-1] = max(back[-1], (STRAIGHT_M - ell[-1]) * float(len(ell) > 1))
    return [(pts[q] - _run(cells, pts[q], -u[q], back[q]) * u[q], pts[q + 1] + _run(cells, pts[q + 1], u[q], ahead[q]) * u[q])
            for q in range(len(u))]


def planned(g, to, cells, warm=(), passed=0.0):
    """(stops, curve, held): where she stands still on her way to `to` - `way`'s corners no curve
    fits through, `to` last, its leading corners within `passed` of her dropped -, the curve she
    walks to the first (`spline`; (0, 3) where she walks straight at it), the rows its QP held. Where
    none fits her heading is let go - she turns standing first, as she does at a stop -, then the
    way is cut at its sharpest corner, a stop: with none anywhere, `way`'s corners."""
    np = sight._np()
    from machine import edge, latent
    d = g['world'].data
    here = (float(d.qpos[0]), float(d.qpos[2]))
    corners = way(g, to, cells)
    corners = [p for k, p in enumerate(corners) if any(math.dist(here, q) > passed for q in corners[:k + 1])]
    yaw = edge.yaw(g['sense'])
    tries = ((math.sin(yaw), math.cos(yaw)), None)[int(float(g['st'][latent.NOW][latent.GAIN]) < 1e-6):]
    pts = [here] + corners
    at = centres(cells)
    last, curve, held = len(pts) - 1, None, tuple(warm)
    while curve is None and last > 1:
        walls = [corridor(at, a, b) for a, b in carried(cells, pts[:last + 1])]
        for lead in tries:
            curve, held = (curve, held) if curve is not None else spline(pts[:last + 1], walls, lead, held)
        last = last if curve is not None else _sharpest(pts[:last + 1])
    return pts[last:] if len(pts) > 1 else [], np.zeros((0, 3)) if curve is None else curve, held


def ahead(curve, k, at):
    """(k, (x, z), along, there): her place on `curve` ((N, 3) x, z, arc length) - its sample
    nearest `at` from the k-th on, AIM_M along it at most -, her aim AIM_M on from there, its end at
    most, the curve's heading at her place and at her aim, rad."""
    np = sight._np()
    s = curve[:, 2]
    near = slice(k, int(np.searchsorted(s, s[k] + AIM_M, side='right')))
    k = k + int(np.argmin(np.hypot(curve[near, 0] - at[0], curve[near, 1] - at[1])))
    j = min(int(np.searchsorted(s, s[k] + AIM_M)), len(s) - 1)
    on = [curve[min(i + 1, len(s) - 1)] - curve[min(i, len(s) - 2)] for i in (k, j)]
    along, there = (math.atan2(float(d[0]), float(d[1])) for d in on)
    return k, (float(curve[j, 0]), float(curve[j, 1])), along, there
