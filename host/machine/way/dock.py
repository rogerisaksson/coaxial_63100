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
0.3-1.3 m. Turned, she backs the few cm left onto the dock. Her stage out to her turn's carry, a cell and
her stop's spread: where she stops nearest, her aim less that spread, her turn ends her a cell off the
dock - out to her carry and a cell alone, her walk's stop spread 0.06 -> 0.13 m (1.16 m/s) left the
chair and the bed no stage, 1.8-2.6 cm short 0.47 m out. The faces her stunt's patch docks her at first
(`furniture.first`: a bed's ends, its sides only where no end is hers to reach), the face's middle first,
then the face her walk reaches soonest. Staged her stopping distance and a stride out
(1.17 m), she backed 0.37-1.03 m after her turn, in 3.9-8.3 s.
"""
import functools
import heapq
import itertools
import math

from machine.body import figure, footprint
from machine.law import poise, se2, stride
from machine.moves import seated
from machine.eyes import voxels
from machine.way import wayfinder

#: Her arrivals off facing the face, rad: head on, aslant either way, along it either way.
ARRIVALS = tuple(k * 0.25 * math.pi for k in (0, 1, -1, 2, -2))


def docks():
    """How each errand docks, kind -> (her half across that lands within the face - her hips on a seat,
    her hanging hands on a bed she lies along and in the doorway -, m; her run in through the door, m, None
    where she turns onto a dock). Her hips within its end, she landed 0.25 m off its middle and lay over
    its corner, 16 deg off its axis."""
    return {'sit_chair': (figure.HIP_HALF, None), 'lie_bed': (figure.ASIDE, None), 'leg_bed': (figure.ASIDE, None),
            'exit_door': (figure.ASIDE, 4.0 * stride.STRIDE_M)}


def shown(er):
    """Errand `er`'s docks as the page draws them - {'at': [(x, y, z)] each face her wayfinder reached
    (`docked`'s 'reach') on its top, 'chosen' the one her planner laid} -, None before its answer."""
    if not er or not er.get('docks'):
        return None
    hn = math.atan2(er['nx'], er['nz'])
    off = [abs(math.remainder(h - hn, math.tau)) for h, _x, _z in er['docks']]
    return {'at': [(x, er['top'], z) for _h, x, z in er['docks']], 'chosen': min(range(len(off)), key=off.__getitem__),
            'kind': er['kind'], 'slot': er.get('slot')}


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


@functools.cache
def _leap():
    """machine.moves.leap for `even`, imported at its first call and not looked up again each pass."""
    from machine.moves import leap
    return leap


def even(g, e=None):
    """Whether her soles bear her alike, within a sole's touch (`leap.TOUCH_N`) of each other, held half
    her stride's period (`se2.BRAKE_S`) where an errand `e` keeps the time: still after a walk's stop her
    weight swings sole to sole 1.0-1.5 s (349/4 N to 94/211 to 153/153), crossing alike on the way;
    turned mid-swing she pivoted anywhere from 0.44 m behind her to 0.28 ahead, alike on the balls of her
    soles turning left (0.10 m ahead) and on their middle turning right (within 0.01). Or her weight's
    split held within a sole's touch as long - her swing over where it ended: stood still at 164/141 N,
    still her soles never crossed alike and she waited at the door's stage 30 s and more."""
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
    """(nx, nz) m her walk over `cells`' free ones from `at` (x, z), inf where none reaches."""
    import numpy as np
    from machine.way import route
    from machine.eyes import sight
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
    from machine.way import affordance, furniture, route
    across, run_in = docks()[sc]
    over = sc == 'sit_chair' and affordance.top(box) > seated.POPLITEAL
    if over and not seated.reaches(affordance.top(box)):
        return None
    across = seated.placed(seated.ON, 0.0)[2] if over else across
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
        n, tan = np.array([math.sin(hn), math.cos(hn)]), np.array([math.cos(hn), -math.sin(hn)])
        span = affordance.support(box, hn + 0.5 * math.pi)
        edge, lat = affordance.support(box, hn), span - (min(across, span) if span >= figure.HIP_HALF else across)
        mid = np.array([float(box[1][0]), float(box[1][2])]) + edge * n
        steps = int(max(lat, 0.0) / voxels.CELL_M)
        offs = [0.0] * (lat >= 0.0) + [s * k * voxels.CELL_M for k in range(1, steps + 1) for s in (1.0, -1.0)]
        rank = furniture.first(sc, box, things, hn)
        rows += [(abs(t), hn, mid + t * tan, n, lat - abs(t), rank) for t in offs]

    def gaps(poses, h):
        return footprint.gaps(poses, h, things, floor).min(axis=1, initial=math.inf)

    def turned(edge, hn, n, a):
        """(len(outs),) whether her turn onto the dock from arrival `a` at each stage, and where her stop's
        spread along `a` toward the face carries her nearest, leaves her things her skin and its end's spread."""
        heading, stop = hn + math.pi + a, se2.STOP_SPREAD_M * max(0.0, math.cos(a))
        poses = np.column_stack([edge + outs[:, None] * n, np.full(len(outs), heading)])
        near = np.column_stack([edge + (outs - stop)[:, None] * n, np.full(len(outs), heading)])
        return np.any([np.minimum(gaps(poses, footprint.swept(d, w)), gaps(near, footprint.swept(d, w)))
                       >= wayfinder.MARGIN + footprint.spread(d, w) for w, d in _ways(heading, hn)], axis=0)

    def backed(edge, hn, n):
        """(len(outs),) whether her walk back from each stage onto the dock leaves her things her skin."""
        b = np.arange(s_dock, outs[-1] + 1e-9, 0.5 * voxels.CELL_M)
        ok = gaps(np.column_stack([edge + b[:, None] * n, np.full(len(b), hn)]), footprint.reach('back')) >= wayfinder.MARGIN
        return np.array([bool(ok[b <= s + 1e-9].all()) for s in outs])

    def laid(row):
        off, hn, edge, n, lat, rank = row
        turns = [turned(edge, hn, n, a) for a in ARRIVALS] if run_in is None else []
        fits = np.ones(1, bool) if run_in is not None else (
            backed(edge, hn, n) & turns[0] & (turns[1] | turns[2]) & (turns[3] | turns[4]))
        s = float(outs[np.argmax(fits)]) if fits.any() else None
        stage = edge + (s or 0.0) * n
        cell = route._cell(*stage)
        walk = far[cell] if s is not None and route._free(cells, cell) else math.inf
        return {'hn': hn, 'face': tuple(n), 'edge': tuple(edge), 'lat_tol': lat, 'd_stage': s, 'stage': tuple(stage),
                'key': (walk == math.inf, rank, off, walk)}

    # by their offsets along the face, the first her walk reaches ending it: none further out beats it
    best, reach = None, {}
    for _off, level in itertools.groupby(sorted(rows, key=lambda r: r[0]), key=lambda r: r[0]):
        found = [laid(r) for r in level]
        for r in [r for r in found if not r['key'][0]]:
            reach[r['hn']] = min(reach.get(r['hn'], r), r, key=lambda q: q['key'])
        best = min(found + ([best] if best is not None else []), key=lambda r: r['key'])
        if not best['key'][0]:
            break
    reached = tuple((hn, r['edge']) for hn, r in reach.items())
    return None if best is None or best['key'][0] else dict(best, reach=reached)
