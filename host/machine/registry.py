"""Her map of the room's things (lie_group_slam_core.lisp): 256 slots of what her looks told her.

    slots = registry.laid()                     # the text's 256 slots, none taken
    registry.seen(slots, looks, head, t)        # a look's things matched or new, each slot's salience moved
    registry.passed(slots, head, dt)            # between looks: movables diffused, blind spots swollen
    props = registry.boxes(slots)               # the things her buses take, as boxes (kind, centre, half, R)

A look's things are her eyes' or their stand-in (`world.props()`), each a box: a kind, a centre, half
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
movable's variance (a kind not STRUCTURAL) grows 0.45 mm^2/s between looks; a slot out of her frustum
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

from machine import kernel
from machine.sight import DOWN_DEG, EYES, FOVY_DEG
from machine.voxels import CELL_M, FAR_M, NEAR_M
from machine.stereo import H, W

_LAW = kernel.node('assimilate-semantic-quadrics', 'quadric-registry')
_BLIND = kernel.node('specialize-with-blindspot-memory-persistence', 'blindspot-memory')
#: The slots, the text's; a look's cell squared, m^2, a measurement's variance.
SLOTS = int(kernel.read()['spatial-object-soa-registry']['memory-layout']['max-tracked-objects'])
CELL2 = CELL_M * CELL_M
#: The association's gate, chi2; a match's, a miss's and a frustum-empty miss's log-odds; the salience
#: a thing is confirmed at, pruned under, taken by her buses from.
GATE = _LAW['gate-threshold']
HIT, MISS = _LAW['occupancy-bayes-law']['on-frustum-match'], _LAW['occupancy-bayes-law']['on-frustum-empty-miss']
CONFIRM, PRUNE = _LAW['occupancy-bayes-law']['confirm-salience-thresh'], _LAW['occupancy-bayes-law']['prune-salience-floor']
TAKEN = kernel.node('specialize-with-affordance-field-projection', 'affordance-routing')['min-salience-log-odds']
#: A blind spot's keeping, s; its swell, m/s, and its most, m.
KEPT_S, SWELL_MPS, SWELL_M = _BLIND['persistence-horizon-s'], _BLIND['blindspot-margin-inflation-rate-mps'], _BLIND['max-inflation-cap-m']
#: A movable's diffusion, m^2/s; the kinds that stand (the text's structural classes as her things' kinds).
DIFFUSE = 1e-6 * kernel.node('maintain-spatial-memory-decay', 'memory-decay')['semi-static-drift-rate']
STRUCTURAL = ('door', 'bed', 'wall', 'stair', 'timber', 'glass', 'roof', 'stone', 'rail')
#: The move past which a thing is moving, chi2.
MOVING = kernel.node('specialize-with-dynamic-agent-bifurcation', 'moving-objects')['static-violation-chi2']
#: A slot's fields.
SLOT = np.dtype([('on', 'f8'), ('kind', 'i4'), ('centre', 'f8', 3), ('half', 'f8', 3), ('R', 'f8', (3, 3)),
                 ('var', 'f8'), ('salience', 'f8'), ('vel', 'f8', 3), ('last', 'f8', 3), ('seen_at', 'f8'),
                 ('swell', 'f8'), ('drift', 'f8'), ('moving', 'f8'), ('shown', 'f8'), ('id', 'i8'), ('clear', 'f8')])
#: Her eyes' half angles' tangents, up and across; their look down her head's z, rad; their axis and
#: their up in her head's frame.
TAN_UP = math.tan(math.radians(FOVY_DEG) / 2.0)
TAN_ACROSS = TAN_UP * W / H
DOWN = math.radians(DOWN_DEG)
AXIS, UP = np.array([0.0, -math.sin(DOWN), math.cos(DOWN)]), np.array([0.0, math.cos(DOWN), math.sin(DOWN)])
#: A box's centre, half sizes and rotation: their shapes.
BOX = ((3,), (3,), (3, 3))


def laid():
    """{rows (SLOTS, SLOT), kinds [str]}: the text's slots, none on."""
    return {'rows': np.zeros(SLOTS, SLOT), 'kinds': [''] * SLOTS}


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
_CORNERS = np.array([[sx, sy, sz] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)])


def seen_of(centre, half, R, eye, turn):
    """(n,) whether each box shows in her eyes' frustum: any of its corners, its centre or its point nearest
    her eye in it - a table top's near edge under her view, its far edge in it."""
    if not len(centre):
        return np.zeros(0, bool)
    corners = centre[:, None, :] + np.einsum('nij,ncj->nci', R, _CORNERS * half[:, None, :])
    points = np.concatenate([corners, centre[:, None, :], nearest(centre, half, R, eye)[:, None, :]], axis=1)
    return shown(points.reshape(-1, 3), eye, turn).reshape(len(centre), -1).any(axis=1)


def _boxes(looks):
    """(kinds, centres (n, 3), halves (n, 3), rotations (n, 3, 3), ids (n,) 0 none, clear (n,) 1.0 where the space
    over a thing is seen clear, a box's without a word) of a look's things, those that are boxes: its things
    of four fields stacked at once, sized one by one where they do not stack as boxes."""
    rows = [p for p in looks if len(p) >= 4]
    try:
        stacked = [np.array([p[j] for p in rows], float).reshape((len(rows),) + s) for j, s in enumerate(BOX, 1)]
    except (TypeError, ValueError):
        rows = [p for p in rows if all(np.size(p[j]) == math.prod(s) for j, s in enumerate(BOX, 1))]
        stacked = [np.array([p[j] for p in rows], float).reshape((-1,) + s) for j, s in enumerate(BOX, 1)]
    return ([str(p[0]) for p in rows], *stacked, np.array([int(p[4]) if len(p) > 4 else 0 for p in rows], np.int64),
            np.array([float(p[5]) if len(p) > 5 else 1.0 for p in rows]))


def _w2(centre, half, at, halves, scale):
    """(n, m) the Wasserstein metric squared of each thing (`centre`, `half` (n, 3)) to each slot (`at`, `halves`
    (m, 3)), over the variances `scale` (m,)."""
    d, h = (at - centre[:, None]) ** 2, (halves - half[:, None]) ** 2
    # a sum over 3 in order, as .sum(2) adds it, without its reduction's setup on a 3-long axis
    return ((d[..., 0] + d[..., 1] + d[..., 2]) + (h[..., 0] + h[..., 1] + h[..., 2])) / scale


def seen(slots, looks, head, t, everywhere=False):
    """`slots` by a look's things `looks` at `t` s, her `head` {pos, quat}: each thing into the slot it
    matches or a new one, every live slot's salience moved, the pruned let go; `everywhere`, the look
    shows the whole room (the stand-in's, the world's own boxes), else what her eyes' frustum shows."""
    r, kinds = slots['rows'], slots['kinds']
    names, centre, half, R, ids, clear = _boxes(looks)
    on = r['on']
    live = np.flatnonzero(on)
    if not len(names) and not len(live):
        return slots
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
            return _matched(slots, live, best, matched, known, centre, half, R, clear, t, eye, turn)
    free = iter(np.flatnonzero(on == 0.0))
    for k, name in enumerate(names):
        best = int(d2[k].argmin()) if len(live) else -1
        if best >= 0 and d2[k, best] <= GATE:
            i, matched[best], d2[:, best] = int(live[best]), True, np.inf
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
        r[i], kinds[i] = (1.0, 0, centre[k], half[k], R[k], CELL2, HIT, np.zeros(3), centre[k], t, 0.0, 0.0, 0.0, 1.0, ids[k],
                          clear[k]), name
    return _missed(slots, live, matched, t, eye, turn)


def _matched(slots, live, best, matched, known, centre, half, R, clear, t, eye, turn):
    """`seen`'s loop at once where each thing's nearest gated slot is no other's: the loop's greedy pick is
    then each row's own, and its arithmetic run element by element - the same bits."""
    r = slots['rows']
    i = live[best]
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
    out of her frustum swollen toward SWELL_M."""
    r, kinds = slots['rows'], slots['kinds']
    live = np.flatnonzero(r['on'])
    if not len(live):
        return slots
    movable = np.array([kinds[i] not in STRUCTURAL for i in live], float)
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
    her vision's process sends hers (`machine.vision`), laid as things there (`things`)."""
    from machine import sight
    r, kinds = slots['rows'], slots['kinds']
    taken = np.flatnonzero((r['on'] > 0.0) & (r['salience'] >= least))
    swell = r['swell'][taken] + sight.LOOK_S * np.hypot(r['vel'][taken, 0], r['vel'][taken, 2])
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

    def __reduce__(self):
        centre, half, R = (a.tolist() for a in self.arrays)
        return list, ([(t[0], tuple(c), tuple(h), tuple(map(tuple, r))) + tuple(t[4:])
                       for t, c, h, r in zip(self, centre, half, R)],)


def _tuples(a):
    """`a`'s rows as nested tuples of its numpy floats, as tuple() of a row gives them."""
    return zip(*a.T) if a.ndim == 2 else zip(*map(_tuples, a.swapaxes(0, 1)))
