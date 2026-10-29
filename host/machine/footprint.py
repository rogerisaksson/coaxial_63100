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

from machine import affordance, footprint_rows, leap

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


def met(things, floor, h):
    """((M,) whether each thing meets her, (M, K) her reach `h` toward it along each of her directions -
    the most over the bands its box spans over `floor`)."""
    import numpy as np
    lo = np.array([affordance.bottom(t) for t in things], float).reshape(-1) - floor
    hi = np.array([affordance.top(t) for t in things], float).reshape(-1) - floor
    first = np.clip(np.floor(lo / BAND_M), 0, len(h) - 1).astype(int)
    last = np.clip(np.floor(hi / BAND_M), 0, len(h) - 1).astype(int)
    spans = (np.arange(len(h))[None, :] >= first[:, None]) & (np.arange(len(h))[None, :] <= last[:, None])
    toward = np.where(spans[:, :, None], h[None, :, :], 0.0).max(axis=1)
    return (hi > leap.TOUCH_M) & (lo < len(h) * BAND_M), toward


def gaps(poses, h, things, floor):
    """(N, M) m: each pose's (x, z, heading) least gap from her reach `h` (bands, K) to each of `things`
    (boxes: kind, centre, half sizes, turn) over `floor`, inf where the thing meets none of her bands."""
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
