"""Where the robot docks at a thing: the face, its place along it, where it stands to turn onto it.

    laid = dock.docked((x, z), things, floor, 'sit_chair', box, standoff)   # the robot's dock and stage on `box`; None where none fits
    way = dock.turns(g, (x, z), heading, to, first)     # the way its turn in place clears its things, 0 none
    verbs = dock.docks()                                # its errands' verbs and how each docks

A seat or a bed: the robot's heels its stunt's standoff off the face's edge, facing away, the robot's hips
within the face - on a seat over its shins its fists set beside them (`seated.placed`): the
robot's hips' 0.21 m let it land 0.151 m aside and a fist met no seat; one its legs reach no
floor from (`seated.reaches`) no dock. The robot stands on the face's normal the least out that the robot's
turn in place onto the dock - from each of its ARRIVALS, still, its soles bearing alike
(`even`), its sweep as measured so (`footprint.swept`) - leaves each thing the robot sees, by its own height
against the robot's body's bands, its skin, that turn's spread (`footprint.spread`) and the robot's
stop's spread along its arrival toward the face (`wayfinder.MARGIN`, `se2.STOP_SPREAD_M`): at the
robot's skin alone it turned 1.6-4.2 cm off and touched -, and its walk back from there onto
the dock, its arms' swing with it, leaves them the robot's skin: a hand met the loft beside the chair; no
distance of its own: staged a stride past the robot's back's end (0.52 m), it settled 0.3-1.2 m out and
backed 0.3-1.3 m. Turned, the robot backs the few cm left onto the dock. The robot's stage out to its turn's
carry, a cell and its stop's spread: where it stops nearest, its aim less that spread, its turn ends it a cell
off the dock - out to its carry and a cell alone, its walk's stop spread 0.06 -> 0.13 m (1.16 m/s) left the
chair and the bed no stage, 1.8-2.6 cm short 0.47 m out. The faces the robot's stunt's patch docks the robot
at first (`furniture.first`: a bed's ends, its sides only where no end is the robot's own to reach), the
face's middle first, then the face its walk reaches soonest. Staged the robot's stopping distance and
a stride out (1.17 m), it backed 0.37-1.03 m after its turn, in 3.9-8.3 s.
"""
import functools
import heapq
import itertools
import math

from machine import libm
from machine.body import figure, footprint
from machine.law import modal, poise, se2, stride
from machine.moves import seated
from machine.eyes import voxels
from machine.way import wayfinder

#: The robot's arrivals off facing the face, rad: head on, aslant either way, along it either way.
ARRIVALS = tuple(k * 0.25 * math.pi for k in (0, 1, -1, 2, -2))


def docks():
    """How each errand docks, kind -> (the robot's half across that lands within the face - its hips on a
    seat, its hanging hands on a bed it lies along and in the doorway -, m; its run in through the door, m,
    None where it turns onto a dock). The robot's hips within its end, the robot landed 0.25 m off its middle
    and lay over its corner, 16 deg off its axis."""
    return {'sit_chair': (figure.HIP_HALF, None), 'lie_bed': (figure.ASIDE, None), 'leg_bed': (figure.ASIDE, None),
            'exit_door': (figure.ASIDE, 4.0 * stride.STRIDE_M)}


def _standoff_of(sc, top, got):
    from machine.law import impedance
    from machine.moves import leap
    from machine.texts import maneuver
    base = maneuver.standoff(sc, got) if sc in maneuver.PLAYED else 0.0
    if top is None or sc != 'sit_chair' or top <= seated.POPLITEAL:
        return base
    suite = maneuver._suite() if got is None else got
    modes, body = maneuver._manifest()
    hips = seated.sat(top) - impedance.PROP_ON * body['weight'] / modes['mode_axial_compression'][1]
    trunk = next((p.get('axial-counterweight', {}).get('dc-offset-rad', 0.0)
                  for op, p, _then in maneuver._played(sc, suite, top)[0]['_events']
                  if op == 'trigger-one-shot-envelope'), 0.0)
    knee, ankle = seated.landed(hips, figure.THIGH, top, trunk)
    lo, hi = 0.0, figure.THIGH
    for _ in range(seated.HALVINGS):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if seated.clear(knee, ankle, mid, top) < 0.0 else (lo, mid)
    return max(base, hi + leap.MIDDLE - poise.HALF_M)


@functools.lru_cache(maxsize=64)
def _standoff_at(_robot, _path, sc, top):
    return _standoff_of(sc, top, None)


def standoff(sc, top=None, got=None):
    """m, the robot's heels off the edge of the furniture stunt `sc` docks it at (`maneuver.standoff`), widened
    where a rigid seat `top` m up stands over `seated.POPLITEAL` so the robot's thighs clear that edge
    (`seated.clear`) in its balanced descent (`seated.landed`)."""
    if got is None:
        from machine.body import mechatronics
        from machine.texts import ortho
        return _standoff_at(mechatronics.ROBOT, ortho._FILE, sc, top)
    return _standoff_of(sc, top, got)


def shown(er):
    """Errand `er`'s docks as the page draws them - {'at': [(x, y, z)] each face the robot's wayfinder reached
    (`docked`'s 'reach') on its top, 'chosen' the one the robot's planner laid} -, None before its answer."""
    if not er or not er.get('docks'):
        return None
    hn = libm.arctan2(er['nx'], er['nz'])
    off = [abs(math.remainder(h - hn, math.tau)) for h, _x, _z in er['docks']]
    return {'at': [(x, er['top'], z) for _h, x, z in er['docks']], 'chosen': min(range(len(off)), key=off.__getitem__),
            'kind': er['kind'], 'slot': er.get('slot')}


def _floor(g):
    from machine.way import furniture
    return furniture.floor(g)


def _ways(heading, to):
    """[(way, deg)]: the robot's turns in place from `heading` onto `to` a half turn at most, +1 left, -1
    right."""
    return [(w, d) for w in (1.0, -1.0) for d in (math.degrees((w * (to - heading)) % math.tau),) if d <= 180.0 + 1e-6]


def _gap(things, at, heading, way, deg, floor):
    return footprint.clear([(at[0], at[1], heading)], footprint.swept(deg, way), things, floor)


def turns(g, at, heading, to, first):
    """The way (+1 left, -1 right) the robot's turn in place from `at` (x, z) facing `heading` onto `to`
    leaves its things its skin and its own end's spread, its sweep as measured from a walk's
    stop, `first` tried first; 0 where neither does."""
    floor = _floor(g)
    fits = {w: _gap(g['things'], at, heading, w, d, floor) >= wayfinder.MARGIN + footprint.spread(d, w)
            for w, d in _ways(heading, to)}
    return next((w for w in (first, -first) if fits.get(w, False)), 0.0)


@functools.cache
def _leap():
    """machine.moves.leap for `even`, imported at its first call and not looked up again each pass."""
    from machine.moves import leap
    return leap


def even(g, e=None):
    """Whether the robot's soles bear its alike, within a sole's touch (`leap.TOUCH_N`) of each other, held
    half its stride's period (`se2.BRAKE_S`) where an errand `e` keeps the time: still after a walk's stop its
    weight swings sole to sole 1.0-1.5 s (349/4 N to 94/211 to 153/153), crossing alike on the way; turned
    mid-swing it pivoted anywhere from 0.44 m behind it to 0.28 ahead, alike on the balls of its soles turning
    left (0.10 m ahead) and on their middle turning right (within 0.01). Or the robot's weight's split held
    within a sole's touch as long - its swing over where it ended: stood still at 164/141 N, still the
    robot's soles never crossed alike and it waited at the door's stage 30 s and more."""
    leap = _leap()
    split = float(g['loads'][0]) - float(g['loads'][1])
    alike = abs(split) <= leap.TOUCH_N
    if e is None:
        return alike
    e['alike_at'] = min(e.get('alike_at', math.inf), g['t']) if alike else math.inf
    if abs(split - e.get('split_was', math.inf)) > leap.TOUCH_N:
        e['split_was'], e['split_at'] = split, g['t']
    return g['t'] - min(e['alike_at'], e['split_at']) >= 0.5 * se2.BRAKE_S


def walked(at, cells):
    """(nx, nz) m the robot's walk over `cells`' free ones from `at` (x, z), inf where none reaches."""
    import numpy as np
    from machine.way import route
    from machine.eyes import grids
    here = route._cell(*at)
    start = here if route._free(cells, here) else grids.nearest_free(cells, here)
    out = np.full(cells.shape, np.inf)
    todo = [(voxels.CELL_M * math.dist(here, start), start)]
    steps = [(i, j, voxels.CELL_M * math.hypot(i, j)) for i in (-1, 0, 1) for j in (-1, 0, 1) if (i, j) != (0, 0)]
    while todo:
        m, (i, j) = heapq.heappop(todo)
        fresh = bool(m < out[i, j])
        out[i, j] = min(out[i, j], m)
        for item in [(m + w, (i + a, j + b)) for a, b, w in steps * fresh
                     if route._free(cells, (i + a, j + b)) and m + w < out[i + a, j + b]]:
            heapq.heappush(todo, item)
    return out


def docked(at, things, floor, sc, box, standoff):
    """Errand `sc`'s dock on the robot's thing `box` among `things` over `floor`, from `at` (x, z) - {'hn' the
    face's outward heading, 'face' its normal (x, z), 'edge' its edge's point the robot docks at (x, z),
    'lat_tol' how far along the edge it may land off that point, 'd_stage' m out along the face's
    normal it stands to turn, 'stage' (x, z)} -, None where none fits: the face's middle first, then
    the face its walk reaches soonest. At the door its face the robot's walk reaches soonest, the
    robot's run in its stage."""
    import numpy as np
    from machine.way import affordance, furniture, route
    across, run_in = docks()[sc]
    over = sc == 'sit_chair' and affordance.top(box) > seated.POPLITEAL
    if over and not seated.reaches(affordance.top(box)):
        return None
    across = seated.across() if over else across
    things = [t for t in things if t is box or not (
        affordance.top(t) <= affordance.bottom(box) + voxels.CELL_M + 1e-9 and affordance.covered(box, t) > 0.0)]
    cells = route.blocked(things, floor)
    far = walked(at, cells)
    s_dock = poise.HALF_M + standoff
    carried = max(footprint.carry(180.0, w)[0] for w in (1.0, -1.0))
    outs = np.array([run_in] if run_in is not None
                    else s_dock + np.arange(0.0, carried + voxels.CELL_M + se2.STOP_SPREAD_M, 0.25 * voxels.CELL_M))
    rows = []
    for hn in furniture.GOES[sc][2](box, things):
        n, tan = np.array([libm.sin(hn), libm.cos(hn)]), np.array([libm.cos(hn), -libm.sin(hn)])
        span = affordance.support(box, hn + 0.5 * math.pi)
        edge, lat = affordance.support(box, hn), span - (min(across, span) if span >= figure.HIP_HALF else across)
        mid = np.array([float(box[1][0]), float(box[1][2])]) + edge * n
        steps = int(max(lat, 0.0) / voxels.CELL_M)
        offs = [0.0] * (lat >= 0.0) + [s * k * voxels.CELL_M for k in range(1, steps + 1) for s in (1.0, -1.0)]
        rank = furniture.first(sc, box, things, hn)
        rows += [(t, hn, mid + t * tan, n, lat - abs(t), rank) for t in offs]

    def gaps(poses, h):
        return footprint.gaps(poses, h, things, floor).min(axis=1, initial=math.inf)

    def turned(edge, hn, n, a):
        """(len(outs),) whether the robot's turn onto the dock from arrival `a` at each stage, and where the
        robot's stop's spread along `a` toward the face carries its nearest, leaves its things
        the robot's skin and its end's spread."""
        heading, stop = hn + math.pi + a, se2.STOP_SPREAD_M * max(0.0, libm.cos(a))
        poses = np.column_stack([edge + outs[:, None] * n, np.full(len(outs), heading)])
        near = np.column_stack([edge + (outs - stop)[:, None] * n, np.full(len(outs), heading)])
        return np.any([np.minimum(gaps(poses, footprint.swept(d, w)), gaps(near, footprint.swept(d, w)))
                       >= wayfinder.MARGIN + footprint.spread(d, w) for w, d in _ways(heading, hn)], axis=0)

    def backed(edge, hn, n):
        """(len(outs),) whether the robot's walk back from each stage onto the dock leaves its things its
        skin."""
        b = np.arange(s_dock, outs[-1] + 1e-9, 0.5 * voxels.CELL_M)
        ok = gaps(np.column_stack([edge + b[:, None] * n, np.full(len(b), hn)]), footprint.reach('back')) >= wayfinder.MARGIN
        return np.array([bool(ok[b <= s + 1e-9].all()) for s in outs])

    def laid(row):
        off, hn, edge, n, lat, rank = row
        if not any(route._free(cells, c) and far[c] < math.inf for c in (route._cell(*(edge + s * n)) for s in outs)):
            return {'hn': hn, 'face': tuple(n), 'edge': tuple(edge), 'off': off, 'lat_tol': lat, 'd_stage': None,
                    'stage': tuple(edge), 'key': (True, rank, abs(off), math.inf)}
        turns = [turned(edge, hn, n, a) for a in ARRIVALS] if run_in is None else []
        fits = np.ones(1, bool) if run_in is not None else (
            backed(edge, hn, n) & turns[0] & (turns[1] | turns[2]) & (turns[3] | turns[4]))
        s = float(outs[np.argmax(fits)]) if fits.any() else None
        stage = edge + (s or 0.0) * n
        cell = route._cell(*stage)
        walk = far[cell] if s is not None and route._free(cells, cell) else math.inf
        return {'hn': hn, 'face': tuple(n), 'edge': tuple(edge), 'off': off, 'lat_tol': lat, 'd_stage': s,
                'stage': tuple(stage), 'key': (walk == math.inf, rank, abs(off), walk)}

    # each face by its offsets from its middle out, the first offset the robot's walk reaches ending that face
    faces, best, reach = list(dict.fromkeys(r[1] for r in rows)), None, {}
    for _off, level in itertools.groupby(sorted(rows, key=lambda r: abs(r[0])), key=lambda r: abs(r[0])):
        found = [laid(r) for r in level if r[1] not in reach]
        for r in [r for r in found if not r['key'][0]]:
            reach[r['hn']] = min(reach.get(r['hn'], r), r, key=lambda q: q['key'])
        best = min(found + ([best] if best is not None else []), key=lambda r: r['key']) if found else best
        if len(reach) == len(faces):
            break
    reached = tuple((hn, r['edge']) for hn, r in reach.items())
    return None if best is None or best['key'][0] else dict(best, reach=reached)


def staged(laid, prev, yaw, things, floor, cells, sc, box, standoff):
    """The robot's dock `laid` with `d_stage` and `stage` moved in along its face's normal to the nearest
    free cell of `cells` where the robot's walk back and its turn onto the dock from its way's arrival
    from `prev` (within `modal.LIST_RAD` either side) clear its things."""
    import numpy as np
    from machine.way import affordance, route
    if not laid or laid.get('d_stage') is None or docks()[sc][1] is not None:
        return laid
    things = [t for t in things if t is box or not (
        affordance.top(t) <= affordance.bottom(box) + voxels.CELL_M + 1e-9 and affordance.covered(box, t) > 0.0)]
    edge, hn, n = np.asarray(laid['edge'], float), float(laid['hn']), np.asarray(laid['face'], float)
    s_dock = poise.HALF_M + standoff
    outs = s_dock + np.arange(0.0, float(laid['d_stage']) - s_dock + 1e-9, 0.25 * voxels.CELL_M)
    b = np.arange(s_dock, outs[-1] + 1e-9, 0.5 * voxels.CELL_M)
    ok_b = footprint.gaps(np.column_stack([edge + b[:, None] * n, np.full(len(b), hn)]),
                          footprint.reach('back'), things, floor).min(axis=1, initial=math.inf) >= wayfinder.MARGIN
    for s in outs:
        stage = edge + float(s) * n
        if not route._free(cells, route._cell(*stage)) or not bool(ok_b[b <= s + 1e-9].all()):
            continue
        dx, dz = float(stage[0]) - float(prev[0]), float(stage[1]) - float(prev[1])
        h_arr = libm.arctan2(dx, dz) if math.hypot(dx, dz) > voxels.CELL_M else float(yaw)
        a_arr = se2.wrap_angle(h_arr - (hn + math.pi))
        if all(any(footprint.gaps(np.array([[stage[0], stage[1], hn + math.pi + a],
                                            [edge[0] + (s - se2.STOP_SPREAD_M * max(0.0, libm.cos(a))) * n[0],
                                             edge[1] + (s - se2.STOP_SPREAD_M * max(0.0, libm.cos(a))) * n[1],
                                             hn + math.pi + a]]),
                                  footprint.swept(d, w), things, floor).min(initial=math.inf)
                   >= wayfinder.MARGIN + footprint.spread(d, w) for w, d in _ways(hn + math.pi + a, hn))
               for a in (a_arr - modal.LIST_RAD, a_arr, a_arr + modal.LIST_RAD)):
            return dict(laid, d_stage=float(s), stage=(float(stage[0]), float(stage[1])))
    return laid
