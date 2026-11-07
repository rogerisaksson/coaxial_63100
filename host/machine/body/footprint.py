"""Her body's footprint by height, and the gap it leaves each of her things.

    gap = footprint.gaps(poses, footprint.reach('stand'), things, floor)   # (N, M) m, pose by thing
    h = footprint.swept(180.0, 1.0)        # (bands, K) m: a half turn to her left, about where it began
    fwd, left = footprint.carry(180.0, 1.0)   # m: where that turn ends her pelvis, forward and left
    e = footprint.spread(180.0, 1.0)       # m: the most an end lay off that
    s = footprint.took(180.0)              # s: the most that turn takes, word to still

Her reach is `footprint_rows`' (`tools/cores/footprint.py`, laid from her own motions on her loop): a
band each BAND_M over her soles' floor, a direction each 2 pi / K from her heading toward her left. A
thing meets the bands its box spans over the floor - its top within her soles' touch of the floor is
floor (`leap.TOUCH_M`), its bottom over her head nothing. The gap is the most, over separating axes, of
the thing's least extent along an axis less her most: her K directions, and the thing's two axes either
way, her reach along one the more of her two directions round it over cos(pi / K) - so never more than
the true gap: two shapes touching never read apart.
"""
import functools
import math

from machine.way import affordance
from machine.body import footprint_rows
from machine.moves import leap

K, BAND_M, STEP_M, TURN_DEG = footprint_rows.K, footprint_rows.BAND_M, footprint_rows.STEP_M, footprint_rows.TURN_DEG
CARRY = {1.0: footprint_rows.CARRY_LEFT, -1.0: footprint_rows.CARRY_RIGHT}
SPREAD = {1.0: footprint_rows.SPREAD_LEFT, -1.0: footprint_rows.SPREAD_RIGHT}
TOOK = tuple(max(a, b) for a, b in zip(footprint_rows.TOOK_LEFT, footprint_rows.TOOK_RIGHT))


@functools.lru_cache(maxsize=1)
def _laid():
    """Her rows as arrays: {mode: (bands, K)}, {way: [(bands, K) a turn]}, (K,) her directions, rad."""
    import numpy as np
    modes = {m: np.array(getattr(footprint_rows, m.upper())) for m in ('stand', 'walk', 'back', 'side')}
    swept = {1.0: [np.array(h) for h in footprint_rows.LEFT], -1.0: [np.array(h) for h in footprint_rows.RIGHT]}
    return modes, swept, 2.0 * np.pi * np.arange(K) / K


def reach(mode):
    """(bands, K) m: her reach about her pelvis in her own frame in `mode` (stand, walk, back, side)."""
    return _laid()[0][mode]


def _bin(deg):
    """The index of the least of TURN_DEG a turn of `deg` is within, the last past it."""
    return next((k for k, top in enumerate(TURN_DEG) if deg <= top + 1e-6), len(TURN_DEG) - 1)


def swept(deg, way):
    """(bands, K) m: her reach over a turn in place of `deg` toward `way` (+1 left) and its settling, about
    her pelvis's place and heading as it began - the least of TURN_DEG holding it -; None past a half turn."""
    return _laid()[1][way][_bin(deg)] if deg <= TURN_DEG[-1] + 1e-6 else None


def carry(deg, way):
    """(forward, left) m: her pelvis's end after a turn of `deg` toward `way`, in its start's frame."""
    return CARRY[way][_bin(deg)]


def spread(deg, way):
    """m: the most such a turn's end lay off its mean carry from still, her soles bearing alike."""
    return SPREAD[way][_bin(deg)]


def took(deg):
    """s: the most a turn in place of `deg` takes from its word to still."""
    return TOOK[_bin(deg)]


#: The things last read (`_laid_things`): the list, its boxes' and footprints' arrays, its meetings by floor and reach.
_THINGS: list = [None, None, {}]


def _laid_things(things):
    """((M,) bottoms, (M,) tops, (M, 2) footprints' centres, (M, 2, 2) axes, (M, 2) half sizes) of `things`, and their
    meetings (`met`), laid once a list - nothing writes into her things -: a dock's ask reads them some 220 times
    (`dock.docked`), laid each its footprints were 343 thousand an errand."""
    import numpy as np
    if _THINGS[0] is not things:
        prints = [affordance.footprint(t) for t in things]
        _THINGS[:] = [things, (np.array([affordance.bottom(t) for t in things], float).reshape(-1),
                               np.array([affordance.top(t) for t in things], float).reshape(-1),
                               *(np.array([p[i] for p in prints]) for i in range(3))), {}]
    return _THINGS[1], _THINGS[2]


def met(things, floor, h):
    """((M,) whether each thing meets her, (M, K) her reach `h` toward it along each of her directions -
    the most over the bands its box spans over `floor`), kept with her things (`_laid_things`) by `floor` and `h`."""
    import numpy as np
    (bottoms, tops, *_prints), meetings = _laid_things(things)
    got = meetings.get((floor, id(h)))
    if got is None or got[0] is not h:
        lo, hi = bottoms - floor, tops - floor
        first = np.clip(np.floor(lo / BAND_M), 0, len(h) - 1).astype(int)
        last = np.clip(np.floor(hi / BAND_M), 0, len(h) - 1).astype(int)
        spans = (np.arange(len(h))[None, :] >= first[:, None]) & (np.arange(len(h))[None, :] <= last[:, None])
        toward = np.where(spans[:, :, None], h[None, :, :], 0.0).max(axis=1)
        got = meetings[(floor, id(h))] = (h, (hi > leap.TOUCH_M) & (lo < len(h) * BAND_M), toward)
    return got[1], got[2]


def gaps(poses, h, things, floor):
    """(N, M) m: each pose's (x, z, heading) least gap from her reach `h` (bands, K) to each of `things`
    (boxes: kind, centre, half sizes, turn) over `floor`, inf where the thing meets none of her bands. Each sum
    over a pair numpy's einsum's written out - its two products added into a zero, the same bits (`gaps_twin` the
    einsums) -: a peek's dock was 735 ms in einsum's loops; her things' arrays laid once a list."""
    import numpy as np
    phi = _laid()[2]
    poses = np.asarray(poses, float).reshape(-1, 3)
    if not len(things):
        return np.full((len(poses), 0), np.inf)
    meets, toward = met(things, floor, h)
    (_lo, _hi, c, axes, half), _meetings = _laid_things(things)
    yaw, at = poses[:, 2], poses[:, :2]
    ahead, left = np.stack([np.sin(yaw), np.cos(yaw)], -1), np.stack([np.cos(yaw), -np.sin(yaw)], -1)
    dirs = np.cos(phi)[None, :, None] * ahead[:, None, :] + np.sin(phi)[None, :, None] * left[:, None, :]
    d0, d1 = dirs[:, None, :, 0], dirs[:, None, :, 1]
    spans = np.abs((0.0 + axes[None, :, None, :, 0] * d0[..., None]) + axes[None, :, None, :, 1] * d1[..., None])
    least = (((0.0 + c[None, :, None, 0] * d0) + c[None, :, None, 1] * d1)
             - ((0.0 + spans[..., 0] * half[None, :, None, 0]) + spans[..., 1] * half[None, :, None, 1]))
    mine = (least - ((0.0 + at[:, None, 0] * dirs[..., 0]) + at[:, None, 1] * dirs[..., 1])[:, None, :]
            - toward[None, :, :]).max(axis=2)
    ax, hb = np.concatenate([axes, -axes], axis=1), np.concatenate([half, half], axis=1)
    ang = np.mod(np.arctan2((0.0 + ax[None, :, :, 0] * left[:, None, None, 0]) + ax[None, :, :, 1] * left[:, None, None, 1],
                            (0.0 + ax[None, :, :, 0] * ahead[:, None, None, 0]) + ax[None, :, :, 1] * ahead[:, None, None, 1]),
                 2.0 * np.pi)
    k0 = np.floor(ang / (2.0 * np.pi / K)).astype(int) % K
    m = np.arange(len(things))[None, :, None]
    bound = np.maximum(toward[m, k0], toward[m, (k0 + 1) % K]) / math.cos(math.pi / K)
    theirs = (((0.0 + c[:, None, 0] * ax[:, :, 0]) + c[:, None, 1] * ax[:, :, 1])[None] - hb[None]
              - ((0.0 + at[:, None, None, 0] * ax[None, :, :, 0]) + at[:, None, None, 1] * ax[None, :, :, 1])
              - bound).max(axis=2)
    return np.where(meets[None, :], np.maximum(mine, theirs), np.inf)


def gaps_twin(poses, h, things, floor):
    """`gaps` in numpy's einsums, as it was (test_wayfinder holds the two to the bit)."""
    import numpy as np
    phi = _laid()[2]
    poses = np.asarray(poses, float).reshape(-1, 3)
    if not len(things):
        return np.full((len(poses), 0), np.inf)
    meets, toward = met(things, floor, h)
    prints = [affordance.footprint(t) for t in things]
    c, axes, half = (np.array([p[i] for p in prints]) for i in range(3))
    yaw, at = poses[:, 2], poses[:, :2]
    ahead, left = np.stack([np.sin(yaw), np.cos(yaw)], -1), np.stack([np.cos(yaw), -np.sin(yaw)], -1)
    dirs = np.cos(phi)[None, :, None] * ahead[:, None, :] + np.sin(phi)[None, :, None] * left[:, None, :]
    least = np.einsum('mi,nki->nmk', c, dirs) - np.einsum('nmkj,mj->nmk', np.abs(np.einsum('mji,nki->nmkj', axes, dirs)), half)
    mine = (least - np.einsum('ni,nki->nk', at, dirs)[:, None, :] - toward[None, :, :]).max(axis=2)
    ax, hb = np.concatenate([axes, -axes], axis=1), np.concatenate([half, half], axis=1)
    ang = np.mod(np.arctan2(np.einsum('mai,ni->nma', ax, left), np.einsum('mai,ni->nma', ax, ahead)), 2.0 * np.pi)
    k0 = np.floor(ang / (2.0 * np.pi / K)).astype(int) % K
    m = np.arange(len(things))[None, :, None]
    bound = np.maximum(toward[m, k0], toward[m, (k0 + 1) % K]) / math.cos(math.pi / K)
    theirs = (np.einsum('mi,mai->ma', c, ax)[None] - hb[None] - np.einsum('ni,mai->nma', at, ax) - bound).max(axis=2)
    return np.where(meets[None, :], np.maximum(mine, theirs), np.inf)


def clear(poses, h, things, floor):
    """m: the least gap over `poses` to any of `things` (`gaps`), inf with none met."""
    g = gaps(poses, h, things, floor)
    return float(g.min()) if g.size else math.inf
