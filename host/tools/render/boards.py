"""The coaxial 63100 board as built: its model beside the tree, laid where a sketch draws an inverter.

    tris = boards.placed(x0, x1, (y, z), back, turn)   # between x0 and x1 about (y, z); None with no model
    bearing = boards.band(slot)                       # where its heatsink lies round its face, rad
    far, near = boards.bus()                          # the bearings its bus's terminals span
    tris = boards.leads(x0, x1, (y, z), back, turn, band_r)    # the motor's phase leads, a lug each
    tris = boards.wired(x0, x1, (y, z), back, turn, entry, outlet, lid)   # the bus's leads under the guard

The board's model is the electronics' own (`electronic_simulations/3d_models/Coaxial 63100.glb`, Draco-packed),
unpacked once to an OBJ under boards/ beside the tree (BESIDE folders up, as `bells` finds the motors'; else the folder
COAXIAL_BOARDS names) - each group its parts' colour (`rgb_r_g_b`), merged near a millimetre a cell -, in its finishes
as built (FINISHES). On its tall side its heatsink: a finned strip across its power stage, between its middle's hole
and its three phase terminals, trimmed inside the rim and round what stands taller than the parts it lies on
(`_sink`). The board is laid with its face across x, its middle on the drive's axis and its bare side - the side its
parts stand least tall on - toward `back`, the motor's back it stands on, turned about the axis as its sketch puts its
heatsink or its bus's terminals (`bus`: the DC bus's press-fit pair at its rim, its RS-485's connector beside
them); what holds it keeps LEAD_M round its rim for its leads.
"""
import functools
import glob
import math
import os

from coaxial.graphics.likeness import BESIDE
from machine.body import mechatronics
from tools.render import knee_mount as km
from tools.render import mounting as m

#: The board's finishes as built where the electronics' export colours them otherwise: its soldermask near black (the
#: export's green) and its press-fit screw terminals tinned (the export's grey).
FINISHES: dict[tuple[int, ...], tuple[int, ...]] = {(0, 128, 0): (34, 36, 40), (178, 178, 178): (206, 208, 212)}
#: The board's heatsink, as laid: a straight strip across it over its power stage, SINK_M (wide, long) - between its
#: middle's hole and its three phase terminals, CUT_M off each, no wider than laid -; its base BASE_M on a gap pad PAD_M
#: over the parts it lies on, its fins FIN_M thick FIN_PITCH apart up to the board's full height; trimmed RIM_M inside
#: the rim and CUT_M round what stands taller than its pad, over the parts that keep the most fin (`_sink`);
#: clear-anodised aluminium; m.
SINK_M = (0.030, 0.070)
BASE_M, PAD_M, FIN_M, FIN_PITCH, RIM_M, CUT_M = 0.0015, 0.0005, 0.001, 0.003, 0.002, 0.001
ALUMINIUM = (168, 172, 178)
#: The heatsink's cell, m: its fins' thickness and their pitch whole cells.
CELL_M = 0.001
#: The room round the board's rim its bus's leads take on their way to its terminals, m: what a cage or a cover
#: round it keeps.
LEAD_M = 0.003
#: The leads a board takes (`materials.toml` [leads]): a phase's, a bus's power lead's and a signal lead's outside, m.
PHASE_M, BUS_M, SIGNAL_M = (float(mechatronics.materials()['leads'][k][1]) for k in ('phase', 'bus', 'signal'))
#: A phase's lead's ink, dark; its lug - a tinned 90-degree copper tube lug for it on an M6 stud, a 4 mm2 one, the
#: kind bought by the bag (no sheet tells its sizes): drawn - its tube's outside and bore round the lead's strands,
#: its palm that tube flat, twice its wall; its stud's hole; its barrel's length; its palm's width; how far its
#: barrel's axis stands off its hole's; its bend's inner radius, m. The lug's screw's head (ISO 7380-1 M6: across,
#: tall), m, bright. Where a phase's lead leaves the motor: how far along the band round it, and how deep into that band it
#: is drawn, m.
LEAD_INK = (54, 56, 62)
LUG = (0.0048, 0.0030, 0.0064)
BARREL_M, PALM_M, REACH_M, BEND_M = 0.008, 0.011, 0.013, 0.002
HEAD_M, STEEL, RUN_M, ROOT_M = (0.0105, 0.0033), (226, 230, 236), 0.008, 0.002
#: The bus's leads under the guard, their inks - positive, negative, the signal pair, the shield's lead - and a ring
#: lug's radius and thickness on a bus terminal, m.
BUS_INKS, EYE_M = ((150, 44, 40), (38, 38, 42), (118, 120, 128), (64, 124, 76)), (0.005, 0.001)

def found(root=None):
    """The board's OBJ: COAXIAL_BOARDS's, else under boards/ beside the repo `root` (the tree's unless told) or the
    checkout a worktree of it stands in, BESIDE folders up at the most; None with none."""
    said = os.environ.get('COAXIAL_BOARDS')
    folders = [said] if said is not None else []
    if said is None:
        beside = root or os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        for _ in range(BESIDE):
            beside = os.path.dirname(beside)
            folders.append(os.path.join(beside, 'boards'))
    return next((got[0] for got in (sorted(glob.glob(os.path.join(f, '*63100*.obj'))) for f in folders) if got), None)


@functools.lru_cache(maxsize=None)
def _model():
    """(corners (n, 3), [(rgb, triangles)], its laminate's group, the laminate's top face): the board, its bare side at
    x 0, its thickness along x and its tall side toward +x, its middle on the x axis; None with no model."""
    import numpy as np
    from coaxial.graphics import wavefront
    path = found()
    if path is None:
        return None
    corners, groups, _at = wavefront.read(path)
    c = (corners - 0.5 * (corners.min(axis=0) + corners.max(axis=0)))[:, [2, 0, 1]]   # its face across x: a turn
    rgbs = [tuple(int(v) for v in next(n for n in names.split() + ['rgb_150_150_150'] if n.startswith('rgb_'))
                  .split('_')[1:4]) for names, _t in groups]
    area = [0.5 * np.linalg.norm(np.cross(c[t[:, 1]] - c[t[:, 0]], c[t[:, 2]] - c[t[:, 0]]), axis=1).sum()
            for _n, t in groups]
    lam = int(np.argmax(area))                           # the laminate: the most of the board's faces
    face = c[groups[lam][1].reshape(-1), 0]
    flip = c[:, 0].max() - face.max() < face.min() - c[:, 0].min()
    c[:, 0] = -c[:, 0] if flip else c[:, 0]
    c[:, 0] -= c[:, 0].min()
    shaded = [(FINISHES.get(rgb, rgb), t[:, ::-1] if flip else t) for rgb, (_n, t) in zip(rgbs, groups)]
    return c, shaded, lam, float(c[groups[lam][1].reshape(-1), 0].max())


def band(slot):
    """The bearing about the board's axis its heatsink's band lies at (`_sink`), rad from +y toward +z in its own
    frame; 0 with no model."""
    return _sink(slot)[1]


def _pieces(corners, tris):
    """[corners (k, 3)] of each connected piece of a group's triangles."""
    import numpy as np
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    used = np.unique(tris)
    at = np.searchsorted(used, tris)
    edges = np.r_[at[:, [0, 1]], at[:, [1, 2]], at[:, [2, 0]]]
    n, label = connected_components(coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
                                               shape=(len(used), len(used))), directed=False)
    return [corners[used[label == k]] for k in range(n)]


def _tinned(c, shaded):
    """(the board's three phase terminals' corners - the three alike of its tinned press-fit terminals -, the
    others', the widest first)."""
    import numpy as np
    pieces = [p for rgb, tris in shaded if rgb == FINISHES[(178, 178, 178)] for p in _pieces(c, tris)]
    area = np.array([np.ptp(p[:, 1]) * np.ptp(p[:, 2]) for p in pieces])
    alike = min(range(len(pieces)), key=lambda i: float(np.sort(np.abs(area - area[i]))[:3].sum()))
    order = np.argsort(np.abs(area - area[alike]))
    return [pieces[i] for i in order[:3]], sorted((pieces[i] for i in order[3:]), key=lambda q: -np.ptp(
        q[:, 1]) * np.ptp(q[:, 2]))


def _phases(c, shaded):
    """The board's three phase terminals' corners."""
    return _tinned(c, shaded)[0]


@functools.lru_cache(maxsize=None)
def bus():
    """(far, near): the bearings about the board's axis its bus's terminals span - the widest of its tinned
    press-fits that is no phase's, the DC bus's pair, its RS-485's connector beside them -, from their end away
    from its phases' terminals to the end toward them, where the board lies bare; rad from +y toward +z in its own
    frame; (0, 0) with no model."""
    import numpy as np
    model = _model()
    if model is None:
        return 0.0, 0.0
    phases, others = _tinned(model[0], model[1])
    pair = others[0]
    mid = math.atan2(float(pair[:, 2].mean()), float(pair[:, 1].mean()))
    off = (np.arctan2(pair[:, 2], pair[:, 1]) - mid + math.pi) % (2.0 * math.pi) - math.pi
    ends = (mid + float(off.min()), mid + float(off.max()))
    beside = np.vstack(phases).mean(axis=0)
    toward = (math.atan2(float(beside[2]), float(beside[1])) - mid + math.pi) % (2.0 * math.pi) - math.pi
    return ends if toward > 0.0 else ends[::-1]


@functools.lru_cache(maxsize=None)
def _sink(slot):
    """([(tri, colour)], its strip's bearing, rad): the heatsink in the board's own frame (`_model`), its fins up to
    `slot` m over the bare side; ([], 0) with no model."""
    import numpy as np
    model = _model()
    if model is None:
        return [], 0.0
    c, shaded, lam, top = model
    phases = _phases(c, shaded)
    mid = np.mean([0.5 * (p.min(axis=0) + p.max(axis=0)) for p in phases], axis=0)
    phi = math.atan2(mid[2], mid[1])                     # toward the phase terminals, over the power stage
    toward = np.array([math.cos(phi), math.sin(phi)])
    hole = float(np.hypot(*c[shaded[lam][1].reshape(-1)][:, 1:].T).min())
    near = min(float((p[:, 1:] @ toward).min()) for p in phases) - CUT_M
    width = min(SINK_M[0], near - hole - CUT_M)          # between the hole and the terminals, no wider than laid
    p = km._points([(c[t], 0) for i, (_rgb, tris) in enumerate(shaded) if i != lam for t in tris], 0.7 * CELL_M)
    p = p[p[:, 0] > top + 1e-4]
    n = int(round((c[:, 1].max() - c[:, 1].min()) / CELL_M)) + 3
    grid = np.zeros((n, n))
    at = np.clip(np.rint(p[:, 1:] / CELL_M).astype(int) + n // 2, 0, n - 1)
    np.maximum.at(grid, (at[:, 0], at[:, 1]), p[:, 0] - top)
    u, v = np.meshgrid(near - width + (np.arange(round(width / CELL_M)) + 0.5) * CELL_M,
                       (np.arange(round(SINK_M[1] / CELL_M)) + 0.5) * CELL_M - 0.5 * SINK_M[1], indexing='ij')
    y, z = u * math.cos(phi) - v * math.sin(phi), u * math.sin(phi) + v * math.cos(phi)
    on = np.hypot(y, z) <= 0.5 * (c[:, 1].max() - c[:, 1].min()) - RIM_M
    cell = (np.clip(np.rint(y / CELL_M).astype(int) + n // 2, 0, n - 1),
            np.clip(np.rint(z / CELL_M).astype(int) + n // 2, 0, n - 1))
    reach, best = int(math.ceil(CUT_M / CELL_M)) + 1, None   # the cut, a cell more: the strip lies across the map's
    for under in np.unique(np.round(grid[cell][on], 4)) + PAD_M:   # each height it may lie on, to 0.1 mm: most fin
        tall = np.pad(grid > under - PAD_M, reach)
        tall = np.any([tall[reach + a:tall.shape[0] - reach + a, reach + b:tall.shape[1] - reach + b]
                       for a in range(-reach, reach + 1) for b in range(-reach, reach + 1)], axis=0)
        kept = on & ~tall[cell]
        fins = kept.sum() * (slot - top - under - BASE_M)
        if best is None or fins > best[0]:
            best = (fins, kept, float(under))
    assert best is not None, 'a height tried'
    _fins, kept, under = best
    turn = np.array([[math.cos(phi), -math.sin(phi)], [math.sin(phi), math.cos(phi)]])
    out = []

    def blocks(mask, x0, x1):                            # each run of kept cells along the band, a block
        for i in np.flatnonzero(mask.any(axis=1)):
            row = np.diff(np.r_[0, mask[i].astype(int), 0])
            for j0, j1 in zip(np.flatnonzero(row == 1), np.flatnonzero(row == -1)):
                a, b = u[i, 0] - 0.5 * CELL_M, u[i, 0] + 0.5 * CELL_M
                s, e = v[i, j0] - 0.5 * CELL_M, v[i, j1 - 1] + 0.5 * CELL_M
                out.extend(m._prism(np.array([[a, s], [b, s], [b, e], [a, e]]) @ turn.T, x0, x1, ALUMINIUM, axis=0))
    base = top + under
    blocks(kept, base, base + BASE_M)
    fin = (np.arange(u.shape[0]) % round(FIN_PITCH / CELL_M)) < round(FIN_M / CELL_M)
    blocks(kept & fin[:, None], base + BASE_M, slot)
    return out, float(phi)


@functools.lru_cache(maxsize=None)
def screws():
    """(its three phase terminals' middles and tops, its DC bus's pair's two terminals', its shield's terminal's -
    the largest part a terminal's pitch past that pair's end -, its RS-485's two connectors' - the two alike
    standing nearest that terminal, the chain in at one and out at the other): [(y, z, x)] each, m in the
    board's own frame - what a lid over it covers whole, what its leads run onto; four empty with no model."""
    import itertools
    import numpy as np
    model = _model()
    if model is None:
        return [], [], [], []
    c, shaded, _lam, laminate = model
    phases, others = _tinned(c, shaded)
    pair = others[0]
    lo, hi, top = pair[:, 1:].min(axis=0), pair[:, 1:].max(axis=0), float(pair[:, 0].max())
    step = np.where(hi - lo == (hi - lo).max(), 0.25 * (hi - lo), 0.0)      # its two, side by side its long way
    mid = 0.5 * (lo + hi)
    def at(q):
        return tuple(float(v) for v in (*(0.5 * (q[:, 1:].min(axis=0) + q[:, 1:].max(axis=0))), q[:, 0].max()))
    tall = [q for rgb, tris in shaded if rgb != FINISHES[(178, 178, 178)] for q in _pieces(c, tris)
            if q[:, 0].max() - laminate > 0.5 * (top - laminate)]
    def side(q):
        return np.ptp(q[:, 1:], axis=0)
    past = [q for q in tall if min(float(np.hypot(*(np.array(at(q)[:2]) - mid - k * 3.0 * step)))
                                   for k in (-1.0, 1.0)) < 2.0 * float(np.abs(step).max())]
    shield = max(past, key=lambda q: float(side(q).prod()))                 # the screw terminal, not a connector by it
    here = np.array(at(shield)[:2])
    twins = [(float(np.hypot(*(0.5 * (np.array(at(a)[:2]) + at(b)[:2]) - here))), at(a), at(b))
             for a, b in itertools.combinations([q for q in tall if q is not shield], 2)
             if abs(at(a)[2] - at(b)[2]) < 1e-4 and float(np.abs(side(a) - side(b)).min()) < 1e-4]
    return ([at(q) for q in phases], [tuple(float(v) for v in (*(mid + k * step), top)) for k in (-1.0, 1.0)],
            [at(shield)], list(min(twins)[1:]) if twins else [])


def _laid(x0, x1, at, back, turn):
    """(the map from the board's own frame to where it lies - between x0 and x1 about `at`, its bare side at `back`,
    turned `turn` rad -, +1 where its tall side stands toward +x, else -1)."""
    import numpy as np
    way, cos, sin = (1.0 if back == x0 else -1.0), math.cos(turn), math.sin(turn)

    def laid(q):
        return np.stack([back + way * q[..., 0], cos * q[..., 1] - sin * q[..., 2] + at[0],
                         sin * q[..., 1] + cos * q[..., 2] + at[1]], -1)
    return laid, way


def _screw(x, at, r=0.5 * HEAD_M[0]):
    """A terminal's screw's head, `r` round, on what it holds at `x` about `at` (y, z): a Torx's, its socket dark."""
    import numpy as np
    low = np.array([x, at[0], at[1]])
    return (km._rod(low, low + (HEAD_M[1], 0.0, 0.0), r, STEEL) + m._prism(
        m._socket('torx', 0.5 * r) + np.asarray(at[:2], float), x + HEAD_M[1], x + HEAD_M[1] + 0.0002, LEAD_INK, axis=0))


def _lug(top, at, out, up=False):
    """([(tri, colour)], (x, off) where its lead goes into it): a 90-degree tube lug in the board's own frame, its
    palm flat on a terminal's top at `top` (x) about `at` (y, z) and out along `out` (y, z, a unit) - its L along
    that way -, its barrel turned down past the board's rim; turned over, `up`, where its lead leaves over the board.
    The mouth is `off` out from `at`."""
    import numpy as np
    outside, bore, _hole = LUG
    thick, r, tin = outside - bore, 0.5 * outside, FINISHES[(178, 178, 178)]
    at, out = np.asarray(at, float), np.asarray(out, float)
    across, way = np.array([-out[1], out[0]]), (1.0 if up else -1.0)
    bend = BEND_M + r                                    # the bend's radius to the tube's middle
    neck, mid = REACH_M - bend, top + 0.5 * thick        # where the palm ends in the bend; the palm's middle

    def put(x, off):
        return np.array([x, at[0] + off * out[0], at[1] + off * out[1]])
    nose = [(0.5 * PALM_M * math.cos(a), 0.5 * PALM_M * math.sin(a)) for a in np.linspace(0.5 * math.pi, 1.5 * math.pi, 9)]
    flat = np.array([at + u * out + v * across for u, v in nose + [(neck, -r), (neck, r)]])
    tris = m._prism(flat, top, top + thick, tin, axis=0)
    tris += _screw(top + thick, at)
    line = [put(mid + way * bend * (1.0 - math.cos(a)), neck + bend * math.sin(a))
            for a in np.linspace(0.0, 0.5 * math.pi, 7)]
    end = mid + way * (bend + BARREL_M)
    tris += km._wire(line + [put(end, REACH_M)], r, tin)
    return tris, (end, REACH_M)


def window(turn=0.0):
    """(mid, half), rad: the bearings about the board's axis its phases' lugs' barrels and leads take up past its
    rim, the board turned `turn` rad - what a guard's wall is left open over; None with no model."""
    phases = screws()[0]
    if not phases:
        return None
    turns = sorted(math.atan2(z, y) for y, z, _x in phases)
    wide = math.asin((0.5 * LUG[0] + LEAD_M) / (math.hypot(*phases[0][:2]) + REACH_M))
    return turn + 0.5 * (turns[0] + turns[-1]), 0.5 * (turns[-1] - turns[0]) + wide


def leads(x0, x1, at, back, turn=0.0, band_r=None):
    """[(tri, colour)]: the motor's three phase leads where the board lies as `placed` has it - each a lug on its
    phase's terminal, its L out along the board's radius there and its barrel down past the rim (`_lug`), its
    screw's head on it; its lead out of that barrel, over onto the band round the motor - `band_r` round outside,
    where told -, RUN_M along it and into the motor; [] with no model."""
    import numpy as np
    if _model() is None:
        return []
    laid, way = _laid(x0, x1, at, back, turn)
    tris = []
    for y, z, top in screws()[0]:
        off = math.hypot(y, z)
        out = np.array([y, z]) / off
        lug, (mouth, reach) = _lug(top, (y, z), out)
        on = off + reach if band_r is None else band_r + 0.5 * PHASE_M      # the lead's middle, lying on the band
        over = abs(on - off - reach)                                         # its way over to the band, at 45 degrees
        path = [(mouth + ROOT_M, off + reach), (mouth - ROOT_M, off + reach), (mouth - ROOT_M - over, on),
                (mouth - ROOT_M - over - RUN_M, on)]
        path += [(path[-1][0], band_r - ROOT_M)] if band_r is not None else []
        wire = km._wire([np.array([x, q * out[0], q * out[1]]) for x, q in path], 0.5 * PHASE_M, LEAD_INK)
        tris += [(laid(tri), colour) for tri, colour in lug + wire]
    return tris if way > 0 else [(tri[::-1], colour) for tri, colour in tris]


def wired(x0, x1, at, back, turn, entry, outlet, lid, tee=False):
    """[(tri, colour)]: the bus's leads under the guard, the board lying as `placed` has it: its power pair, its
    signal pair and its shield's lead in at the bearing `entry` (rad, about the axis) - from the saddle's channel -,
    round the board's rim in the room its leads take (LEAD_M) and in onto the DC bus's two terminals and the
    shield's, a ring lug each, and into the RS-485's one connector; and on - from the same screws, the chain's two
    lugs under one Torx screw, and from the RS-485's other connector, no stub - over to `outlet` (y, z) and up
    through the guard there, `lid` (x in the board's frame) its face, round the rim again, over no screw. A `tee`'s -
    the board a branch off a T-cross on the trunk - all through that one place, the outlet (the wall at `entry`
    with none): the power pair and the shield's lead once, a lug a screw, the signal pair in and out again; []
    with no model."""
    import numpy as np
    model = _model()
    if model is None:
        return []
    laid, way = _laid(x0, x1, at, back, turn)
    _phases, pair, shield, serial = screws()
    rim = float(np.hypot(model[0][:, 1], model[0][:, 2]).max()) + 0.5 * LEAD_M
    cos, sin = math.cos(turn), math.sin(turn)
    hole = None if outlet is None else np.array([cos * (outlet[0] - at[0]) + sin * (outlet[1] - at[1]),
                                                 -sin * (outlet[0] - at[0]) + cos * (outlet[1] - at[1])])   # its frame
    start = entry - turn
    door = start if hole is None else math.atan2(hole[1], hole[0])           # where the chain goes on

    def round_(a, b, x):                                 # along the rim from the bearing a to b, the shorter way
        sweep = (b - a + math.pi) % (2.0 * math.pi) - math.pi
        return [(x, rim * math.cos(a + f * sweep), rim * math.sin(a + f * sweep))
                for f in np.linspace(0.0, 1.0, max(2, int(abs(sweep) / 0.1) + 1))]
    tris = []
    runs = [(p, p, BUS_M) for p in pair] + [(serial[0], serial[-1], 2.0 * SIGNAL_M), (shield[0], shield[0], SIGNAL_M)]
    for k, (((y, z, top), (v, w, _top), width), ink) in enumerate(zip(runs, BUS_INKS)):
        on, level = top + EYE_M[1] + 0.5 * width, LEAD_M + k * 2.0 * SIGNAL_M   # each its own height round the rim
        drop = [] if hole is None else [(lid, hole[0], hole[1]), (on + width, hole[0], hole[1])]
        come = drop + round_(door, math.atan2(z, y), on + width) if tee else round_(start, math.atan2(z, y), level)
        come += [(on, come[-1][1], come[-1][2]), (on, y, z)]
        go = [(on + width, v, w)] + round_(math.atan2(w, v), door, on + width) + drop[::-1]
        for path in ((come,) if tee and (y, z) == (v, w) else (come, go)):
            tris += km._wire([np.array(q) for q in path], 0.5 * width, ink)
        if (y, z) == (v, w):                             # a screw terminal: its lugs - the chain's two - and screw
            thick = (1.0 if tee else 2.0) * EYE_M[1]
            tris += km._rod(np.array([top, y, z]), np.array([top + thick, y, z]), EYE_M[0] * width / BUS_M,
                            FINISHES[(178, 178, 178)])
            tris += _screw(top + thick, (y, z), 0.5 * HEAD_M[0] * width / BUS_M)
    tris = [(laid(tri), colour) for tri, colour in tris]
    return tris if way > 0 else [(tri[::-1], colour) for tri, colour in tris]


def placed(x0, x1, at, back, turn=0.0):
    """[(tri, colour)]: the board and its heatsink between x0 and x1 about `at` (y, z), its bare side at `back` (x0 or
    x1), turned `turn` rad about the axis (+y toward +z); None with no model, where the sketch draws its drum."""
    model = _model()
    if model is None:
        return None
    c, shaded, _lam, _top = model
    laid, way = _laid(x0, x1, at, back, turn)
    pts = laid(c)
    board = [(pts[t if way > 0 else t[::-1]], rgb) for rgb, triangles in shaded for t in triangles]
    return board + [(laid(tri if way > 0 else tri[::-1]), rgb) for tri, rgb in _sink(round(abs(x1 - x0), 6))[0]]
