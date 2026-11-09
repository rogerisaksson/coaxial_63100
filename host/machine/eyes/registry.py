"""Her map of the room's things (lie_group_slam_core.lisp): 256 slots of what her looks told her.

    slots = registry.laid()                     # the text's 256 slots, none taken
    registry.seen(slots, looks, head, t)        # a look's things matched or new, each slot's salience moved
    registry.passed(slots, head, dt)            # between looks: movables diffused, blind spots swollen
    props = registry.boxes(slots)               # the things her buses take, as boxes (kind, centre, half, R)

A look's things are her eyes' or their stand-in (`world.props()`), each a box: a kind - its class (`classes.of`, the
text's `:semantic-class`), a matched slot taking the look's -, a centre, half
sizes, a rotation, and her eyes' an id and whether the space over it is seen clear (`sight.told`: a thing's
id the look before's where they share voxels; a surface's top clear, a structure's unknown, a prop's
clear) - a look's thing with a live slot's id is its, the slot's box the look's; of them those her eyes'
frustum shows any of (`sight`'s: NEAR_M to FAR_M, FOVY_DEG high and W / H as wide, DOWN_DEG under her head's
z) at the box's point nearest her eye. A slot (SLOT): its
kind, centre, half sizes and rotation, its position's variance (a look's cell squared, `sight.CELL_M`),
its salience (log-odds), its velocity, when it was last seen and whether in her frustum, its blind
spot's swell, its drift. A thing matches the live slot of its kind nearest it in the Wasserstein
metric between Gaussians at their centres with their half sizes for widths - |dc|^2 + |dh|^2, the slot's
centre where its velocity carries it - over the slot's and the look's variances, within the text's gate
(`assimilate-semantic-quadrics`, chi2 5.99, GATE), and is filtered in (a Kalman gain of the variances);
a match +0.85, a slot in the frustum and not matched -0.22 a look, out of it 0 while its blind
spot is kept (45 s, `specialize-with-blindspot-memory-persistence`) and -0.22 past that; a thing with no
slot within the gate takes a free slot, else the least salient's; a slot under -3.5 is pruned. A
movable's variance (a class not STRUCTURAL) grows 0.45 mm^2/s between looks; a slot out of her frustum
swells 1.5 mm/s to 120 mm, a match resets it. A slot's drift is its looks' moves from its centre over
the variances, summed - the still model's evidence against it -; past the text's chi2 (16.27,
`specialize-with-dynamic-agent-bifurcation`) the thing is moving: its velocity its move from its last
look over the time, its slot at each look, its variance the look's. Her buses take the slots at 1.8 (`specialize-with-affordance-field-projection`); 2.2 confirms.
"""
import math
import os
from itertools import compress

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.texts import kernel
from machine.eyes import stacked
from machine.eyes.gimbal import DOWN_DEG, EYES
from machine.eyes.voxels import CELL_M, FAR_M, NEAR_M
from machine.eyes.stereo import FOVY_DEG, H, W

_LAW = kernel.node('assimilate-semantic-quadrics', 'quadric-registry')
_BLIND = kernel.node('specialize-with-blindspot-memory-persistence', 'blindspot-memory')
#: The slots and the bytes each field's lines align to, the text's; a look's cell squared, m^2, a measurement's variance.
_LAYOUT = kernel.read()['spatial-object-soa-registry']['memory-layout']
SLOTS, ALIGN = int(_LAYOUT['max-tracked-objects']), int(_LAYOUT['align-bytes'])
CELL2 = CELL_M * CELL_M
#: The association's gate, chi2; a match's, a miss's and a frustum-empty miss's log-odds; the salience
#: a thing is confirmed at, pruned under, taken by her buses from.
GATE = _LAW['gate-threshold']
HIT, MISS = _LAW['occupancy-bayes-law']['on-frustum-match'], _LAW['occupancy-bayes-law']['on-frustum-empty-miss']
CONFIRM, PRUNE = _LAW['occupancy-bayes-law']['confirm-salience-thresh'], _LAW['occupancy-bayes-law']['prune-salience-floor']
TAKEN = kernel.node('specialize-with-affordance-field-projection', 'affordance-routing')['min-salience-log-odds']
#: A blind spot's keeping, s; its swell, m/s, and its most, m.
KEPT_S, SWELL_MPS, SWELL_M = _BLIND['persistence-horizon-s'], _BLIND['blindspot-margin-inflation-rate-mps'], _BLIND['max-inflation-cap-m']
#: A movable's diffusion, m^2/s; the classes that stand (`classes`), the text's `structural-lock-classes`: what a
#: house is built of, and its beds.
_DECAY = kernel.node('maintain-spatial-memory-decay', 'memory-decay')
DIFFUSE = 1e-6 * _DECAY['semi-static-drift-rate']
STRUCTURAL = tuple(c.lstrip(':') for c in _DECAY['static-locked-classes'])
#: The move past which a thing is moving, chi2.
MOVING = kernel.node('specialize-with-dynamic-agent-bifurcation', 'moving-objects')['static-violation-chi2']
#: A slot's fields (name, type, shape a slot): each an array over the slots of its own (`Rows`).
SLOT = (('on', 'f8', ()), ('kind', 'i4', ()), ('centre', 'f8', (3,)), ('half', 'f8', (3,)), ('R', 'f8', (3, 3)),
        ('var', 'f8', ()), ('salience', 'f8', ()), ('vel', 'f8', (3,)), ('last', 'f8', (3,)), ('seen_at', 'f8', ()),
        ('swell', 'f8', ()), ('drift', 'f8', ()), ('moving', 'f8', ()), ('shown', 'f8', ()), ('id', 'i8', ()),
        ('clear', 'f8', ()))
#: Her eyes' half angles' tangents, up and across; their look down her head's z, rad; their axis and
#: their up in her head's frame.
TAN_UP = math.tan(math.radians(FOVY_DEG) / 2.0)
TAN_ACROSS = TAN_UP * W / H
DOWN = math.radians(DOWN_DEG)
AXIS, UP = np.array([0.0, -math.sin(DOWN), math.cos(DOWN)]), np.array([0.0, math.cos(DOWN), math.sin(DOWN)])


def laid(twin=False):
    """{rows (`Rows`), kinds [str], classed}: the text's slots, none on, no look classed (`stacked.classed`); its
    looks in C (`tools.cores.registry`), numpy's own where `twin`."""
    return {'rows': _rows({}, twin), 'kinds': [''] * SLOTS, 'classed': None}


class Rows(dict):

    """The slots' fields (SLOT), as the text lays them: each an array of its own in one block, on ALIGN-byte lines
    of its own - a field read over the live slots reads its own lines alone. A packed record a slot (252 bytes) left
    each double past its kind off its 8 bytes: a field a view 252 bytes a step, numpy's loops copying it through a
    buffer. Pickled as its fields, laid anew where it lands (`_rows`); its look in C (`core`) laid at its first."""

    twin, core = False, None

    def __reduce__(self):
        return _rows, (dict(self), self.twin)


def _rows(values, twin=False):
    """`Rows` laid in one block, each field `values`' where given, else zeros; a numpy twin's where `twin`."""
    sizes = [(name, dtype, shape, SLOTS * math.prod(shape) * np.dtype(dtype).itemsize) for name, dtype, shape in SLOT]
    raw = np.zeros(sum(-(-n // ALIGN) * ALIGN for *_f, n in sizes) + ALIGN, np.uint8)
    at, rows = -raw.ctypes.data % ALIGN, Rows()
    rows.twin = twin
    for name, dtype, shape, n in sizes:
        rows[name] = raw[at:at + n].view(dtype).reshape((SLOTS,) + shape)
        if name in values:
            rows[name][...] = values[name]
        at += -(-n // ALIGN) * ALIGN
    return rows


def _frame(q):
    """The rotation of quaternion `q` (w, x, y, z)."""
    w, x, y, z = q
    return np.array([[1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
                     [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
                     [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)]])


def _eye(head):
    """(her eye's place, her head's rotation) of `head` {pos, quat}."""
    R = _frame(head['quat'])
    return np.asarray(head['pos']) + R @ np.asarray(EYES), R


def nearest(centre, half, R, eye):
    """(n, 3) each box's point nearest `eye`."""
    local = np.clip(np.einsum('nji,nj->ni', R, eye - centre), -half, half)
    return centre + np.einsum('nij,nj->ni', R, local)


def shown(points, eye, R):
    """(n,) whether each of `points` lies in her eyes' frustum, her eye at `eye`, her head's rotation `R`."""
    v = (np.asarray(points).reshape(-1, 3) - eye) @ R
    depth, up = v @ AXIS, v @ UP
    return (depth >= NEAR_M) & (depth <= FAR_M) & (np.abs(up) <= depth * TAN_UP) & (np.abs(v[:, 0]) <= depth * TAN_ACROSS)


#: A box's corners, in its half sizes.
CORNERS = np.array([[sx, sy, sz] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)])


def seen_of(centre, half, R, eye, turn):
    """(n,) whether each box shows in her eyes' frustum: any of its corners, its centre or its point nearest
    her eye in it - a table top's near edge under her view, its far edge in it."""
    if not len(centre):
        return np.zeros(0, bool)
    corners = centre[:, None, :] + np.einsum('nij,ncj->nci', R, CORNERS * half[:, None, :])
    points = np.concatenate([corners, centre[:, None, :], nearest(centre, half, R, eye)[:, None, :]], axis=1)
    return shown(points.reshape(-1, 3), eye, turn).reshape(len(centre), -1).any(axis=1)


def _w2(centre, half, at, halves, scale):
    """(n, m) the Wasserstein metric squared of each thing (`centre`, `half` (n, 3)) to each slot (`at`, `halves`
    (m, 3)), over the variances `scale` (m,)."""
    d, h = (at - centre[:, None]) ** 2, (halves - half[:, None]) ** 2
    # a sum over 3 in order, as .sum(2) adds it, without its reduction's setup on a 3-long axis
    return ((d[..., 0] + d[..., 1] + d[..., 2]) + (h[..., 0] + h[..., 1] + h[..., 2])) / scale


def _core(r):
    """`r`'s look in C (`tools.cores.registry`), laid at its first; None for a numpy twin's."""
    if r.twin:
        return None
    if r.core is None:
        from tools.cores import registry as c_registry
        r.core = c_registry.laid(r)
    return r.core


def warm():
    """Its C built and loaded, numpy's sums read (`tools.cores.registry`): a process's first look not kept waiting."""
    from tools.cores import registry as c_registry
    c_registry.laid(_rows({}))


def seen(slots, looks, head, t, everywhere=False, kept=None, changed=None):
    """`slots` by a look's things `looks` at `t` s, her `head` {pos, quat}: each thing by its class (`classes.of`,
    `stacked.classed`) into the slot it matches or a new one, every live slot's salience moved, the pruned let go;
    `everywhere`, the look shows the whole room (the stand-in's, the world's own boxes), else what her eyes' frustum
    shows. In C past its classes (`_seen_c`), numpy's own below for a twin; its boxes kept on `kept` and restacked where
    `changed` (the rows not the same objects as the last look's, her vision apart's), else stacked whole."""
    r, kinds = slots['rows'], slots['kinds']
    boxes, dirty = (stacked.stack(looks), True) if changed is None else stacked.restacked(kept, looks, changed)
    names, centre, half, R, ids, clear = boxes
    names = stacked.classed(slots, names, centre, half, R, clear, dirty)
    on = r['on']
    live = np.flatnonzero(on)
    if not len(names) and not len(live):
        return slots
    core = _core(r)
    if core is not None:
        return _seen_c(core, slots, names, centre, half, R, ids, clear, live, head, t, everywhere)
    eye, turn = _eye(head)
    looked = np.ones(len(names), bool) if everywhere else seen_of(centre, half, R, eye, turn)
    names, centre, half, R, ids, clear = (list(compress(names, looked)), centre[looked], half[looked], R[looked], ids[looked],
                                         clear[looked])
    var, salience, vel, last = r['var'], r['salience'], r['vel'], r['last']
    seen_at, swell, drift = r['seen_at'], r['swell'], r['drift']
    matched = np.zeros(len(live), bool)
    ahead = r['centre'][live] + vel[live] * (t - seen_at[live])[:, None]
    scale = var[live] + CELL2
    code = {}
    alike = np.array([code.setdefault(n, len(code)) for n in names])[:, None] == [code.get(kinds[i], -1) for i in live]
    d2 = np.where(alike, _w2(centre, half, ahead, r['half'][live], scale), np.inf)
    known = (ids[:, None] > 0) & (ids[:, None] == r['id'][live])
    d2 = np.where(known.any(axis=1)[:, None], np.where(known, 0.0, np.inf), d2)
    if len(live) and len(names):
        best = d2.argmin(axis=1)
        if (d2[np.arange(len(names)), best] <= GATE).all() and len(np.unique(best)) == len(best):
            return _matched(slots, live, best, matched, known, names, centre, half, R, clear, t, eye, turn)
    free = iter(np.flatnonzero(on == 0.0))
    for k, name in enumerate(names):
        best = int(d2[k].argmin()) if len(live) else -1
        if best >= 0 and d2[k, best] <= GATE:
            i, matched[best], d2[:, best] = int(live[best]), True, np.inf
            kinds[i] = name
            dt, move = max(t - seen_at[i], 1e-9), centre[k] - r['centre'][i]
            drift[i] += float(move @ move) / (var[i] + CELL2)
            moving = float(drift[i] > MOVING)
            gain = max(moving + (1.0 - moving) * var[i] / (var[i] + CELL2), float(known[k, best]))
            r['moving'][i], vel[i] = moving, moving * (centre[k] - last[i]) / dt
            r['centre'][i] += gain * move
            r['half'][i] += gain * (half[k] - r['half'][i])
            r['R'][i], var[i], r['clear'][i] = R[k], (1.0 - gain) * var[i] + moving * CELL2, clear[k]
            salience[i] += HIT
            last[i], seen_at[i], swell[i], r['shown'][i] = centre[k], t, 0.0, 1.0
            continue
        i = next(free, None)
        if i is None:
            # none free: the least salient's; a live slot so taken meets the things after it by its new half sizes
            i = int(np.argmin(np.where(on > 0.0, salience, np.inf)))
            j = np.flatnonzero((live == i) & ~matched)
            d2[k + 1:, j] = np.where(alike[k + 1:, j], _w2(centre[k + 1:], half[k + 1:], ahead[j], half[k:k + 1], scale[j]),
                                     np.inf)
        for (field, _type, _shape), v in zip(SLOT, (1.0, 0, centre[k], half[k], R[k], CELL2, HIT, 0.0, centre[k], t, 0.0,
                                                    0.0, 0.0, 1.0, ids[k], clear[k])):
            r[field][i] = v
        kinds[i] = name
    return _missed(slots, live, matched, t, eye, turn)


def _seen_c(core, slots, names, centre, half, R, ids, clear, live, head, t, everywhere):
    """`seen` past its things' classes in C (`tools.cores.registry`): its things' classes and its live slots' coded
    alike - a slot of none of them -1 -, each slot taken its thing's class as numpy's loop names it, in turn."""
    kinds, code = slots['kinds'], {}
    codes = np.array([code.setdefault(n, len(code)) for n in names], np.int64)
    slot_codes = np.array([code.get(kinds[i], -1) for i in live], np.int64)
    n = core.seen(centre, half, R, ids, clear, codes, live, slot_codes, t, everywhere, head['pos'], head['quat'])
    looked = names if everywhere else list(compress(names, core.looked[:len(names)]))
    for i, name in zip(core.slot_of[:n].tolist(), looked):
        kinds[i] = name
    return slots


def _matched(slots, live, best, matched, known, names, centre, half, R, clear, t, eye, turn):
    """`seen`'s loop at once where each thing's nearest gated slot is no other's: the loop's greedy pick is
    then each row's own, and its arithmetic run element by element - the same bits; each slot its thing's class."""
    r = slots['rows']
    i = live[best]
    for k, name in zip(i.tolist(), names):
        slots['kinds'][k] = name
    gained = known[np.arange(len(best)), best].astype(float)
    matched[best] = True
    var, drift, last = r['var'], r['drift'], r['last']
    dt, move = np.maximum(t - r['seen_at'][i], 1e-9), centre - r['centre'][i]
    drift[i] += (move * move).sum(axis=1) / (var[i] + CELL2)   # move @ move a row: a 3-sum in order, its bits
    moving = (drift[i] > MOVING).astype(float)
    r['moving'][i], r['vel'][i] = moving, moving[:, None] * (centre - last[i]) / dt[:, None]
    gain = np.maximum(moving + (1.0 - moving) * var[i] / (var[i] + CELL2), gained)
    r['centre'][i] += gain[:, None] * move
    r['half'][i] += gain[:, None] * (half - r['half'][i])
    r['R'][i], var[i], r['clear'][i] = R, (1.0 - gain) * var[i] + moving * CELL2, clear
    r['salience'][i] += HIT
    last[i], r['seen_at'][i], r['swell'][i], r['shown'][i] = centre, t, 0.0, 1.0
    return _missed(slots, live, matched, t, eye, turn)


def _missed(slots, live, matched, t, eye, turn):
    """The live slots no thing matched: shown as her frustum holds them, their salience moved; the pruned let go."""
    r = slots['rows']
    on, salience, seen_at = r['on'], r['salience'], r['seen_at']
    rest = live[~matched]
    if len(rest):
        inside = seen_of(r['centre'][rest], r['half'][rest], r['R'][rest], eye, turn)
        kept = t - seen_at[rest] <= KEPT_S
        r['shown'][rest] = inside
        salience[rest] += MISS * (inside | ~kept)
    on[(on > 0.0) & (salience < PRUNE)] = 0.0
    return slots


def passed(slots, head, dt):
    """`slots` `dt` s on between looks, her `head` {pos, quat}: a movable's variance diffused, a slot
    out of her frustum swollen toward SWELL_M - in C (`tools.cores.registry`), numpy's own below for a twin."""
    r, kinds = slots['rows'], slots['kinds']
    live = np.flatnonzero(r['on'])
    if not len(live):
        return slots
    movable = np.array([kinds[i] not in STRUCTURAL for i in live], float)
    core = _core(r)
    if core is not None:
        core.passed(live, movable, head['pos'], head['quat'], dt)
        return slots
    r['var'][live] += DIFFUSE * dt * movable
    eye, turn = _eye(head)
    out = 1.0 - seen_of(r['centre'][live], r['half'][live], r['R'][live], eye, turn)
    r['swell'][live] = np.minimum(SWELL_M, r['swell'][live] + SWELL_MPS * dt * out)
    return slots


def boxes(slots, least=TAKEN):
    """[(kind, centre, half sizes, rotation, its slot, clear over it, its swell)] of the slots at `least`
    salience: her buses' things, as `world.props()` gives them, each named by its slot, her route grown
    by the swell (`route.blocked`): its blind spot's, and a mover's wake - its floor speed over a look
    (`sight.LOOK_S`), where it can be before she looks again. Beside the door's swinging panel a
    hairline corridor opened and closed with it: threading it she brushed the panel, stumbled out at
    1.12 m/s and fell at the deck's rail; waked, she pushes square till the panel stands still."""
    return things(*boxed(slots, least))


def boxed(slots, least=TAKEN):
    """`boxes`' parts as arrays and lists - kinds, centres, half sizes, rotations, slots, clears, swells -: what
    her vision's process sends hers (`machine.eyes.vision`), laid as things there (`things`)."""
    from machine.eyes import sight
    r, kinds = slots['rows'], slots['kinds']
    taken = np.flatnonzero((r['on'] > 0.0) & (r['salience'] >= least))
    wake = np.array([kinds[i] == 'door' for i in taken], float) * r['shown'][taken]
    swell = r['swell'][taken] + sight.LOOK_S * wake * np.hypot(r['vel'][taken, 0], r['vel'][taken, 2])
    return ([kinds[i] for i in taken], r['centre'][taken], r['half'][taken], r['R'][taken], taken.tolist(),
            r['clear'][taken].tolist(), swell.tolist())


def things(kinds, centre, half, R, taken, clear, swell):
    """`boxed`'s parts as her things (`Things`): a tuple each, its arrays kept beside them."""
    laid = Things(zip(kinds, _tuples(centre), _tuples(half), _tuples(R), taken, clear, swell))
    laid.arrays = (centre, half, R)
    return laid


def keyed(things):
    """`things`' key for what they afford (`sprung.named`): each one's kind, slot, centre and its half sizes across
    the floor to the centimetre - a registry's (`Things`) off its arrays at once, numpy's rounding either way."""
    if getattr(things, 'arrays', None) is None:
        return tuple((t[0], t[4], round(t[1][0], 2), round(t[1][1], 2), round(t[1][2], 2), round(t[2][0], 2),
                      round(t[2][2], 2)) for t in things)
    centre, half, _R = things.arrays
    rounded = np.round(np.concatenate([centre, half[:, [0, 2]]], axis=1), 2).tolist()
    return tuple((t[0], t[4], *r) for t, r in zip(things, rounded))


class Things(list):

    """`boxes`'s things, their centres, half sizes and rotations kept as arrays beside them (`schema.room`); a
    plain list pickled - her page is sent the things, not the arrays: their numbers Python's floats, the same values,
    her state's pickle 1.35 -> 0.21 ms."""

    #: (centres, half sizes, rotations) as `things` laid them.
    arrays: tuple

    def __reduce__(self):
        centre, half, R = (a.tolist() for a in self.arrays)
        return list, ([(t[0], tuple(c), tuple(h), tuple(map(tuple, r))) + tuple(t[4:])
                       for t, c, h, r in zip(self, centre, half, R)],)


def _tuples(a):
    """`a`'s rows as nested tuples of its numpy floats, as tuple() of a row gives them."""
    return zip(*a.T) if a.ndim == 2 else zip(*map(_tuples, a.swapaxes(0, 1)))
