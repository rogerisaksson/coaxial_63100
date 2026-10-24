"""Her map of the room's things (lie_group_slam_core.lisp): 256 slots of what her looks told her.

    slots = registry.laid()                     # the text's 256 slots, none taken
    registry.seen(slots, looks, head, t)        # a look's things matched or new, each slot's salience moved
    registry.passed(slots, head, dt)            # between looks: movables diffused, blind spots swollen
    props = registry.boxes(slots)               # the things her buses take, as boxes (kind, centre, half, R)

A look's things are her eyes' or their stand-in (`world.props()`), each a box: a kind, a centre, half
sizes, a rotation (`sight.Keep.map.things` gives a floor box and a height); of them those her eyes'
frustum shows (`sight`'s: NEAR_M to FAR_M, FOVY_DEG high and W / H as wide, DOWN_DEG under her head's
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

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import kernel
from machine.sight import CELL_M, DOWN_DEG, EYES, FAR_M, FOVY_DEG, NEAR_M
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
                 ('swell', 'f8'), ('drift', 'f8'), ('moving', 'f8'), ('shown', 'f8')])
#: Her eyes' half angles' tangents, up and across; their look down her head's z, rad.
TAN_UP = math.tan(math.radians(FOVY_DEG) / 2.0)
TAN_ACROSS = TAN_UP * W / H
DOWN = math.radians(DOWN_DEG)


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


def shown(points, head):
    """(n,) whether each of `points` lies in her eyes' frustum: `head` {pos, quat} her head's."""
    eye, R = _eye(head)
    v = (np.asarray(points).reshape(-1, 3) - eye) @ R
    depth = v @ np.array([0.0, -math.sin(DOWN), math.cos(DOWN)])
    up = v @ np.array([0.0, math.cos(DOWN), math.sin(DOWN)])
    return (depth >= NEAR_M) & (depth <= FAR_M) & (np.abs(up) <= depth * TAN_UP) & (np.abs(v[:, 0]) <= depth * TAN_ACROSS)


def _boxes(looks):
    """(kinds, centres (n, 3), halves (n, 3), rotations (n, 3, 3)) of a look's things, those that are boxes."""
    rows = [p for p in looks if len(p) >= 4 and np.size(p[1]) == 3 and np.size(p[2]) == 3 and np.size(p[3]) == 9]
    return ([str(p[0]) for p in rows], np.array([p[1] for p in rows], float).reshape(-1, 3),
            np.array([p[2] for p in rows], float).reshape(-1, 3), np.array([p[3] for p in rows], float).reshape(-1, 3, 3))


def seen(slots, looks, head, t):
    """`slots` by a look's things `looks` at `t` s, her `head` {pos, quat}: each thing into the slot it
    matches or a new one, every live slot's salience moved, the pruned let go."""
    r, kinds = slots['rows'], slots['kinds']
    names, centre, half, R = _boxes(looks)
    eye = _eye(head)[0]
    looked = shown(nearest(centre, half, R, eye), head)
    names = [n for n, on in zip(names, looked) if on]
    centre, half, R = centre[looked], half[looked], R[looked]
    live = np.flatnonzero(r['on'])
    matched = np.zeros(len(live), bool)
    ahead = r['centre'][live] + r['vel'][live] * (t - r['seen_at'][live])[:, None]
    scale = r['var'][live] + CELL2
    alike = np.array([[kinds[i] == n for i in live] for n in names], bool).reshape(len(names), len(live))
    for k in range(len(names)):
        w2 = ((ahead - centre[k]) ** 2).sum(1) + ((r['half'][live] - half[k]) ** 2).sum(1)
        d2 = np.where(matched | ~alike[k], np.inf, w2 / scale)
        best = int(np.argmin(d2)) if len(d2) else -1
        if best >= 0 and d2[best] <= GATE:
            i, matched[best] = int(live[best]), True
            dt, move = max(t - r['seen_at'][i], 1e-9), centre[k] - r['centre'][i]
            r['drift'][i] += float(move @ move) / (r['var'][i] + CELL2)
            moving = float(r['drift'][i] > MOVING)
            gain = moving + (1.0 - moving) * r['var'][i] / (r['var'][i] + CELL2)
            r['moving'][i], r['vel'][i] = moving, moving * (centre[k] - r['last'][i]) / dt
            r['centre'][i] += gain * move
            r['half'][i] += gain * (half[k] - r['half'][i])
            r['R'][i], r['var'][i] = R[k], (1.0 - gain) * r['var'][i] + moving * CELL2
            r['salience'][i] += HIT
            r['last'][i], r['seen_at'][i], r['swell'][i], r['shown'][i] = centre[k], t, 0.0, 1.0
            continue
        free = np.flatnonzero(r['on'] == 0.0)
        i = int(free[0]) if len(free) else int(np.argmin(np.where(r['on'] > 0.0, r['salience'], np.inf)))
        r[i] = (1.0, 0, centre[k], half[k], R[k], CELL2, HIT, np.zeros(3), centre[k], t, 0.0, 0.0, 0.0, 1.0)
        kinds[i] = names[k]
    rest = live[~matched]
    if len(rest):
        inside = shown(nearest(r['centre'][rest], r['half'][rest], r['R'][rest], eye), head)
        kept = t - r['seen_at'][rest] <= KEPT_S
        r['shown'][rest] = inside
        r['salience'][rest] += MISS * (inside | ~kept)
    gone = np.flatnonzero((r['on'] > 0.0) & (r['salience'] < PRUNE))
    r['on'][gone] = 0.0
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
    out = 1.0 - shown(nearest(r['centre'][live], r['half'][live], r['R'][live], _eye(head)[0]), head)
    r['swell'][live] = np.minimum(SWELL_M, r['swell'][live] + SWELL_MPS * dt * out)
    return slots


def boxes(slots, least=TAKEN):
    """[(kind, centre, half sizes swollen by its blind spot, rotation)] of the slots at `least` salience:
    her buses' things, as `world.props()` gives them."""
    r, kinds = slots['rows'], slots['kinds']
    return [(kinds[i], tuple(r['centre'][i]), tuple(r['half'][i] + r['swell'][i]), tuple(map(tuple, r['R'][i])))
            for i in np.flatnonzero((r['on'] > 0.0) & (r['salience'] >= least))]
