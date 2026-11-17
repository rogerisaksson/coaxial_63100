"""The robot's seat bare: a black honeycomb net a side over its hip's roll drive, turning with it.

    drive = ([(name, corners, triangles)], joint)   # what a side's hip yaw turns, at rest in the pelvis's frame
    parts += [('left_seat', 'left_hip_yaw', (), -joint, 0.0, seat.plate(1.0, drive)), ...]

A net a side where the robot's buttock is: the guards' honeycomb alone, open, its bars black, over the hip's roll
drive's stack - its motor's can (an outrunner, turning), its board, its gearbox and its bolts - a guard's air
(`machine.body.guards.AIR_M`) off it all round, the motor seen through each cell and nothing worn let in to the
can. The net is fixed on the drive and turns with it on the hip's yaw, so the drive never swings through it. The
net's form is a buttock's: a box of the power POWER about a middle SIDE of the way out from the cleft (CLEFT) and
EGG of the way up from the stack's foot - fuller toward the hip, fullest low down, rounding in toward the waist and
the cleft -, scaled out until the whole stack lies within it; the net's cells are CELL across their flats, each's
opening FILL of a cell, laid on a plane over the net's middle and back onto its round, those wholly within its
outline.
"""
import math

from coaxial.graphics.livery import VOID
from machine.body import drives
from machine.body.guards import AIR_M

#: The user's pick among sixty looks (M5): a white shell over the drive read as a box, as eyes through a window or as a
#: cup when round and big; the honeycomb alone, black, smaller, its bars thick enough to be seen, a buttock's form.
POWER, EGG, SIDE, CLEFT, CELL, FILL = 2.75, 0.45, 0.42, 0.016, 0.018, 0.75
STACK = ('drive_', 'inv_', 'gear_', 'bolts_')
A, B = (-90.0, 90.0), (-90.0, 90.0)
BEND: tuple = ()
AROUND, SWUNG = 48, 21
#: The drive's triangles told as points no further apart than SPACING; the outline sought out from the net's middle
#: in steps of half that, then halved REFINE times.
SPACING, REFINE = AIR_M, 10


def _dense(meshes):
    """Points over the triangles of `meshes` [(corners, triangles)], none further than SPACING from the next."""
    from coaxial.model.blocks import numpy as np
    out = [np.zeros((0, 3))]
    for c, t in meshes:
        a, b, d = (c[t[:, k]] for k in range(3))
        longest = np.max([np.linalg.norm(b - a, axis=1), np.linalg.norm(d - b, axis=1), np.linalg.norm(a - d, axis=1)], axis=0)
        n = np.maximum(1, np.ceil(longest / SPACING)).astype(int)
        for m in np.unique(n):
            i, j = np.meshgrid(np.arange(m + 1), np.arange(m + 1), indexing='ij')
            w = np.stack([i[i + j <= m], j[i + j <= m]], 1) / m
            k = n == m
            out.append((a[k, None] + w[None, :, :1] * (b - a)[k, None] + w[None, :, 1:] * (d - a)[k, None]).reshape(-1, 3))
    return np.vstack(out)


def plate(side, drive, pelvis=()):
    """A side's net's mesh in the pelvis's frame at rest, `side` 1 the left's, over `drive` ([(name, corners,
    triangles)] at rest in the pelvis's frame of what the side's hip yaw turns, and that joint's place), clear of
    `pelvis` (the pelvis's own meshes [(corners, triangles)] at rest) through the yaw's travel: its bars, each
    triangle facing out."""
    from coaxial.model.blocks import numpy as np
    flip = np.array([side, 1.0, 1.0])
    stack = _dense([(np.asarray(c, float) * flip, np.asarray(t)) for n, c, t in drive[0] if n.startswith(STACK)])
    low, high = stack.min(axis=0) - AIR_M, stack.max(axis=0) + AIR_M
    front = float((stack[:, 2].min() + stack[:, 2].max()) / 2.0)          # the net behind the stack's middle
    centre = np.array([CLEFT + SIDE * (high[0] - CLEFT), low[1] + EGG * (high[1] - low[1]), front])
    half = np.array([high[0] - centre[0], high[1] - centre[1], centre[2] - low[2], centre[1] - low[1], centre[0] - CLEFT])

    def axes_of(p):
        """Each point's semi-axes: out or in, up or down, back."""
        return np.stack([np.where(p[..., 0] > 0.0, half[0], half[4]), np.where(p[..., 1] > 0.0, half[1], half[3]),
                         np.full(p.shape[:-1], half[2])], -1)
    behind = stack[stack[:, 2] <= centre[2]] - centre
    grown = float((np.abs(behind / (axes_of(behind) - AIR_M)) ** POWER).sum(axis=1).max()) ** (1.0 / POWER)
    half = (half - AIR_M) * max(1.0, grown) + AIR_M                     # the whole stack within, a guard's air off
    r = float(half[2])
    (am, ah), (bm, bh) = ((math.radians(lo + hi) / 2.0, math.radians(hi - lo) / 2.0) for lo, hi in (A, B))

    def at(a, b):
        """(the net's face `a` down from straight back and `b` out, its normal out there)."""
        d = np.stack([np.sin(b), -np.cos(b) * np.sin(a), -np.cos(b) * np.cos(a)], -1)
        axes = axes_of(d)
        p = d * ((np.abs(d / axes) ** POWER).sum(axis=-1) ** (-1.0 / POWER))[..., None]
        n = np.sign(p) * np.abs(p / axes) ** (POWER - 1.0) / axes
        return centre + p, n / np.linalg.norm(n, axis=-1)[..., None]
    t = np.arange(AROUND) * (2.0 * math.pi / AROUND)
    u, v = np.cos(t), np.sin(t)

    def clear(a, b):
        """Whether the face `a` down and `b` out lies off the cleft."""
        return at(a, b)[0][:, 0] >= CLEFT
    steps = np.linspace(0.0, 1.0, int(math.ceil(r * max(ah, bh) / (SPACING / 2.0))) + 1)
    ok = clear(am + ah * np.outer(v, steps).ravel(), bm + bh * np.outer(u, steps).ravel()).reshape(len(t), len(steps))
    first = np.where(ok.all(axis=1), len(steps), np.argmin(ok, axis=1))
    lo, hi = steps[np.maximum(first - 1, 0)], steps[np.minimum(first, len(steps) - 1)]
    lo, hi = np.where(first >= len(steps), 1.0, lo), np.where(first >= len(steps), 1.0, hi)
    for _ in range(REFINE):
        mid = (lo + hi) / 2.0
        good = clear(am + ah * mid * v, bm + bh * mid * u)
        lo, hi = np.where(good, mid, lo), np.where(good, hi, mid)
    edge = lo
    # The cells on a plane over the net's middle, a row of whole ones, the next half a cell aside, laid back onto its
    # round: a cell's bars between its hexagon and one FILL of it, the cells wholly within the outline.
    span = int(r * max(ah, bh) / CELL) + 2
    rise = CELL * math.sqrt(3.0) / 2.0
    mids = np.array([(i * CELL + (j % 2) * CELL / 2.0, j * rise) for j in range(-span, span + 1) for i in range(-span, span + 1)])
    hexa = np.arange(6) * (math.pi / 3.0) + math.pi / 6.0
    unit = np.stack([np.cos(hexa), np.sin(hexa)], 1) * CELL / math.sqrt(3.0)
    flat = mids[:, None, :] + np.concatenate([unit, FILL * unit])[None]
    a, b = am + flat[..., 1] / r, bm + flat[..., 0] / r
    way = np.arctan2((a - am) / ah, (b - bm) / bh) % (2.0 * math.pi)
    reach = np.interp(way, np.r_[t, 2.0 * math.pi], np.r_[edge, edge[0]])
    inside = (np.hypot((a - am) / ah, (b - bm) / bh) < reach)[:, :6].all(axis=1)
    # Turned with the drive through the yaw's travel, no cell comes nearer the pelvis's frame than a guard's air.
    hexes = at(a[inside].ravel(), b[inside].ravel())[0]
    frame = _dense([(np.asarray(c, float) * flip, np.asarray(t_)) for c, t_ in pelvis])
    joint = np.asarray(drive[1], float) * flip
    reach_yaw = max(abs(x) for x in drives.TRAVEL['hip_yaw'])
    near = np.full(len(hexes), np.inf)
    for turn in np.radians(np.linspace(-reach_yaw, reach_yaw, SWUNG)):
        rel = hexes[:, [0, 2]] - joint[[0, 2]]
        p = np.stack([joint[0] + rel[:, 0] * math.cos(turn) + rel[:, 1] * math.sin(turn), hexes[:, 1],
                      joint[2] + rel[:, 1] * math.cos(turn) - rel[:, 0] * math.sin(turn)], 1)
        box = frame[((frame > p.min(axis=0) - AIR_M) & (frame < p.max(axis=0) + AIR_M)).all(axis=1)]
        for chunk in (c for c in np.array_split(box, max(1, len(box) // 2000)) if len(c)):
            near = np.minimum(near, np.linalg.norm(p[:, None, :] - chunk[None, :, :], axis=2).min(axis=1))
    inside[np.flatnonzero(inside)[~(near.reshape(-1, 12) >= AIR_M).all(axis=1)]] = False
    k = np.arange(6)
    quads = np.concatenate([np.stack([k, (k + 1) % 6, 6 + (k + 1) % 6], 1), np.stack([k, 6 + (k + 1) % 6, 6 + k], 1)])
    corners, n = at(a[inside].ravel(), b[inside].ravel())
    tri = (12 * np.arange(int(inside.sum()))[:, None, None] + quads[None]).reshape(-1, 3)
    facing = (np.cross(corners[tri[:, 1]] - corners[tri[:, 0]], corners[tri[:, 2]] - corners[tri[:, 0]]) * n[tri[:, 0]]).sum(axis=1)
    tri[facing < 0.0] = tri[facing < 0.0][:, ::-1]
    laid = corners * flip
    return laid, tri[:, ::int(math.copysign(1.0, side))], np.zeros((len(laid), 2)), np.full(len(laid), VOID)
