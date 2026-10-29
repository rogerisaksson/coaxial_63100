"""Where she docks at a thing: the face, her place along it, where she stands to turn onto it.

    laid = dock.docked((x, z), things, floor, 'sit_chair', box, standoff)   # her dock and stage on `box`; None where none fits
    way = dock.turns(g, (x, z), heading, to, first)     # the way her turn in place clears her things, 0 none
    verbs = dock.docks()                                # her errands' verbs and how each docks

A seat or a bed: her heels its stunt's standoff off the face's edge, facing away, her hips within the
face - on a seat over her shins her fists set beside them (`seated.placed`): her hips' 0.21 m let her land
0.151 m aside and a fist met no seat; one her legs reach no floor from (`seated.reaches`) no dock. She stands on the face's normal the least out that her turn in place onto the dock - from each of
her ARRIVALS, still, her soles bearing alike (`even`), its sweep as measured so
(`footprint.swept`) - leaves each thing she sees, by its own height against her body's bands, her skin,
that turn's spread (`footprint.spread`) and her stop's spread along her arrival toward the face
(`wayfinder.MARGIN`, `se2.STOP_SPREAD_M`): at her skin alone she turned 1.6-4.2 cm off and touched -,
and her walk back from there onto the dock, its arms' swing with it, leaves them her skin: a hand met
the loft beside the chair; no
distance of its own: staged a stride past her back's end (0.52 m), she settled 0.3-1.2 m out and backed
0.3-1.3 m. Turned, she backs the few cm left onto the dock. The face's
middle first, then the face her walk reaches soonest. Staged her stopping distance and a stride out
(1.17 m), she backed 0.37-1.03 m after her turn, in 3.9-8.3 s.
"""
import heapq
import itertools
import math

from machine import figure, footprint, poise, se2, seated, stride, voxels, wayfinder

#: Her arrivals off facing the face, rad: head on, aslant either way, along it either way.
ARRIVALS = tuple(k * 0.25 * math.pi for k in (0, 1, -1, 2, -2))


def docks():
    """How each errand docks, kind -> (her half across that lands within the face - her hips on a seat
    or a bed, her hanging hands in the doorway -, m; her run in through the door, m, None where she turns
    onto a dock)."""
    return {'sit_chair': (figure.HIP_HALF, None), 'lie_bed': (figure.HIP_HALF, None),
            'exit_door': (figure.ASIDE, 4.0 * stride.STRIDE_M)}


def _floor(g):
    return min(float(f['sole'][1]) for f in g['s']['feet'])


def _ways(heading, to):
    """[(way, deg)]: her turns in place from `heading` onto `to` a half turn at most, +1 left, -1 right."""
    return [(w, d) for w in (1.0, -1.0) for d in (math.degrees((w * (to - heading)) % math.tau),) if d <= 180.0 + 1e-6]


def _gap(things, at, heading, way, deg, floor):
    return footprint.clear([(at[0], at[1], heading)], footprint.swept(deg, way), things, floor)


def turns(g, at, heading, to, first):
    """The way (+1 left, -1 right) her turn in place from `at` (x, z) facing `heading` onto `to` leaves
    her things her skin and its own end's spread, its sweep as measured from a walk's stop, `first` tried
    first; 0 where neither does."""
    floor = _floor(g)
    fits = {w: _gap(g['things'], at, heading, w, d, floor) >= wayfinder.MARGIN + footprint.spread(d, w)
            for w, d in _ways(heading, to)}
    return next((w for w in (first, -first) if fits.get(w, False)), 0.0)


def even(g, e=None):
    """Whether her soles bear her alike, within a sole's touch (`leap.TOUCH_N`) of each other, held half
    her stride's period (`se2.BRAKE_S`) where an errand `e` keeps the time: still after a walk's stop her
    weight swings sole to sole 1.0-1.5 s (349/4 N to 94/211 to 153/153), crossing alike on the way;
    turned mid-swing she pivoted anywhere from 0.44 m behind her to 0.28 ahead, alike on the balls of her
    soles turning left (0.10 m ahead) and on their middle turning right (within 0.01)."""
    from machine import leap, se2
    alike = abs(float(g['loads'][0]) - float(g['loads'][1])) <= leap.TOUCH_N
    if e is None:
        return alike
    e['alike_at'] = min(e.get('alike_at', math.inf), g['t']) if alike else math.inf
    return g['t'] - e['alike_at'] >= 0.5 * se2.BRAKE_S


def walked(at, cells):
    """(nx, nz) m her walk over `cells`' free ones from `at` (x, z), inf where none reaches."""
    import numpy as np
    from machine import route, sight
    here = route._cell(*at)
    start = here if route._free(cells, here) else sight._nearest_free(cells, here)
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
    """Errand `sc`'s dock on her thing `box` among `things` over `floor`, from `at` (x, z) - {'hn' the
    face's outward heading, 'face' its normal (x, z), 'edge' its edge's point she docks at (x, z),
    'lat_tol' how far along the edge she may land off that point, 'd_stage' m out along the face's
    normal she stands to turn, 'stage' (x, z)} -, None where none fits: the face's middle first, then
    the face her walk reaches soonest. At the door its face her walk reaches soonest, her run in its
    stage."""
    import numpy as np
    from machine import affordance, furniture, route
    across, run_in = docks()[sc]
    over = sc == 'sit_chair' and affordance.top(box) > seated.POPLITEAL
    if over and not seated.reaches(affordance.top(box)):
        return None
    across = seated.placed(seated.ON, 0.0)[2] if over else across
    cells = route.blocked(things, floor)
    far = walked(at, cells)
    s_dock = poise.HALF_M + standoff
    carried = max(footprint.carry(180.0, w)[0] for w in (1.0, -1.0))
    outs = np.array([run_in] if run_in is not None else s_dock + np.arange(0.0, carried + voxels.CELL_M, 0.25 * voxels.CELL_M))
    rows = []
    for hn in furniture.GOES[sc][2](box, things):
        n, tan = np.array([math.sin(hn), math.cos(hn)]), np.array([math.cos(hn), -math.sin(hn)])
        edge, lat = affordance.support(box, hn), affordance.support(box, hn + 0.5 * math.pi) - across
        mid = np.array([float(box[1][0]), float(box[1][2])]) + edge * n
        steps = int(max(lat, 0.0) / voxels.CELL_M)
        offs = [0.0] * (lat >= 0.0) + [s * k * voxels.CELL_M for k in range(1, steps + 1) for s in (1.0, -1.0)]
        rows += [(abs(t), hn, mid + t * tan, n, lat - abs(t)) for t in offs]

    def gaps(poses, h):
        return footprint.gaps(poses, h, things, floor).min(axis=1, initial=math.inf)

    def turned(edge, hn, n, a):
        """(len(outs),) whether her turn onto the dock from arrival `a` at each stage leaves her things
        her skin, its end's spread and her stop's along her arrival."""
        heading = hn + math.pi + a
        poses = np.column_stack([edge + outs[:, None] * n, np.full(len(outs), heading)])
        return np.any([gaps(poses, footprint.swept(d, w)) >= wayfinder.MARGIN + footprint.spread(d, w)
                       + se2.STOP_SPREAD_M * max(0.0, math.cos(a)) for w, d in _ways(heading, hn)], axis=0)

    def backed(edge, hn, n):
        """(len(outs),) whether her walk back from each stage onto the dock leaves her things her skin."""
        b = np.arange(s_dock, outs[-1] + 1e-9, 0.5 * voxels.CELL_M)
        ok = gaps(np.column_stack([edge + b[:, None] * n, np.full(len(b), hn)]), footprint.reach('back')) >= wayfinder.MARGIN
        return np.array([bool(ok[b <= s + 1e-9].all()) for s in outs])

    def laid(row):
        off, hn, edge, n, lat = row
        fits = np.ones(1, bool) if run_in is not None else backed(edge, hn, n) & np.all(
            [turned(edge, hn, n, a) for a in ARRIVALS], axis=0)
        s = float(outs[np.argmax(fits)]) if fits.any() else None
        stage = edge + (s or 0.0) * n
        cell = route._cell(*stage)
        walk = far[cell] if s is not None and route._free(cells, cell) else math.inf
        return {'hn': hn, 'face': tuple(n), 'edge': tuple(edge), 'lat_tol': lat, 'd_stage': s, 'stage': tuple(stage),
                'key': (walk == math.inf, off, walk)}

    # by their offsets along the face, the first her walk reaches ending it: none further out beats it
    best = None
    for _off, level in itertools.groupby(sorted(rows, key=lambda r: r[0]), key=lambda r: r[0]):
        found = [laid(r) for r in level] + ([best] if best is not None else [])
        best = min(found, key=lambda r: r['key'])
        if not best['key'][0]:
            break
    return None if best is None or best['key'][0] else best
