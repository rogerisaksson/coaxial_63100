"""What is one thing, many to one, and the buildings the robot's things make (each told by `classes`).

    ones, built = wholes.grouped(names, whole, centre, half, R)   # the one thing each is of, the buildings they make
    names, ones, built = wholes.read(things)                      # each box's class, the one thing it is of, buildings
    names = wholes.named(things)                                  # each box's class, None a piece or what it cannot tell
    built = wholes.buildings(things)                              # the buildings boxes make

What is one thing, many to one (`grouped`, `_instances`): what of one class touches, slabs across each other apart
- a window's panes and the posts between them, a stair's treads, not two walls at a corner -; a building's walls on
one side one wall; a piece of furniture its top's, two chairs side by side two. A building (`classes.houses`): its
shell - walls, windows and doors, joined across what the robot's eyes may not have seen of it - with an opening in
it, enclosing a floor, and what roofs it over; a cabin of one storey, else a house.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.eyes import classes
from machine.eyes.classes import FURNITURE, STEP, THIN, THROUGH
from machine.eyes.voxels import CELL_M, FAR_M


def _instances(names, whole, beside, lo, hi, built):
    """(n,) the one thing each of `n` things told `names` is of, many to one - their `whole`s, which meet
    (`beside`), their boxes' corners `lo`, `hi` (n, 3), the buildings `built` (`classes.houses`') -: its lowest
    index. A piece of furniture its top's, two chairs side by side two; a building's walls on one side, its
    openings between them, one wall; what the robot sees through (THROUGH) in one plane within the robot's
    map's reach, no wall wider than THIN standing from its floor between - a glass wall's panes, a
    door set in it, a rail either side of a stair -, one, two windows a wall apart two; the rest what of one
    class touches - a window's panes and the posts between them, a stair's treads a rise (STEP) apart, a
    lamp's post and shade, a rail along a side -, slabs across each other apart, as two walls at a corner."""
    n = len(names)
    names = np.asarray(names, object)
    middle, half, slack = 0.5 * (lo + hi), 0.5 * (hi - lo), np.where(names == 'stairs', STEP, CELL_M)
    level = (np.abs(middle[:, None, 1] - middle[None, :, 1]) - half[:, None, 1] - half[None, :, 1]
             <= np.maximum(slack[:, None], slack[None, :]) + 1e-9)
    flat = half[:, ::2] <= 0.5 * THIN + 1e-9
    slab = flat & ~flat[:, ::-1]
    across = (slab[:, None, 0] & slab[None, :, 1]) | (slab[:, None, 1] & slab[None, :, 0])
    one = np.array([k not in FURNITURE and k != 'thing' for k in names], bool)
    link = (names[:, None] == names[None, :]) & beside & level & ~across & one[:, None] & one[None, :]
    plane = classes.planar(lo, hi)
    # what the robot sees through in one plane within its map's reach, no wall wider than THIN standing from its floor
    # between
    u = np.where(flat[:, 1], 0, 2)
    a_lo, a_hi = lo[np.arange(n), u], hi[np.arange(n), u]
    start, end = np.minimum(a_hi[:, None], a_hi[None, :]), np.maximum(a_lo[:, None], a_lo[None, :])
    pier = np.flatnonzero((names == 'wall') & (lo[:, 1] <= STEP))
    stood = ((np.minimum(a_hi[pier], end[:, :, None]) - np.maximum(a_lo[pier], start[:, :, None]) > THIN + 1e-9)
             & plane[:, None, pier] & level[:, None, pier] & level[None, :, pier]).any(axis=2)
    through = np.isin(names, THROUGH)
    link |= (names[:, None] == names[None, :]) & through[:, None] & plane & level & ~stood & (end - start <= FAR_M + 1e-9)
    for _name, _lo, _hi, members in built:
        wall = np.zeros(n, bool)
        wall[members] = names[members] == 'wall'
        link |= wall[:, None] & wall[None, :] & plane
    label = np.asarray(whole, int).copy()
    while True:
        low = np.minimum(label, np.where(link, label[None, :], n).min(axis=1))
        low = low[low]
        if (low == label).all():
            break
        label = low
    return label


#: Each told name's code and flags for the C (`tools.cores.classes.BITS`), told at its first grouping.
_NAMED: dict = {}


def _named(name):
    """(code, flags) of a told name `name` for the C: a building's shell, opening, roof or storey, a wall, seen
    through, a stair, no thing of its own (a piece of furniture, what none tells)."""
    got = _NAMED.get(name)
    if got is None:
        from tools.cores.classes import BITS
        flags = sum(BITS[bit] for bit, held in (('shell', name in classes.SHELL), ('open', name in classes.OPENINGS),
                                                 ('roof', name in classes.ROOFS), ('storey', name in classes.STOREY),
                                                 ('wall', name == 'wall'), ('through', name in THROUGH),
                                                 ('stairs', name == 'stairs'),
                                                 ('notone', name in FURNITURE or name == 'thing')) if held)
        got = _NAMED[name] = (len(_NAMED), flags)
    return got


def grouped(names, whole, centre, half, R, in_c=True):
    """((n,) the one thing each of `n` things told `names` - `of`'s, with their `whole`s - is of (`_instances`), the
    buildings they make [(class, centre (3,), half sizes (3,), members (k,))] (`classes.houses`)), of their centres and
    half sizes (n, 3) and turns (n, 3, 3). In C where `in_c` (`tools.cores.classes`, wbc_wholes.c: the lodge's 93
    boxes 1.6 ms of numpy, a state told the robot's page each time its things move), numpy's own its twin
    (`_grouped`), bit for bit."""
    from tools.cores.classes import MOST
    told = classes.core() if in_c and 0 < len(names) <= MOST else None
    if told is None:
        return _grouped(names, whole, centre, half, R)
    coded = [_named(str(name)) for name in names]
    one, built = told.grouped(centre, half, R, [c for c, _f in coded], [f for _c, f in coded], whole)
    return one.astype(int), [('cabin' if cabin else 'house', 0.5 * (a + b), 0.5 * (b - a), members)
                             for cabin, a, b, members in built]


def _grouped(names, whole, centre, half, R):
    """`grouped` in numpy, its twin."""
    if not len(names):
        return np.zeros(0, int), []
    e = np.einsum('nij,nj->ni', np.abs(R), half)
    lo, hi, beside = centre - e, centre + e, classes.meet(centre, e)
    built = classes.houses(names, beside, lo, hi)
    return (_instances(names, whole, beside, lo, hi, built),
            [(name, 0.5 * (a + b), 0.5 * (b - a), members) for name, a, b, members in built])


def read(things):
    """What the robot's eyes read of boxes `things` - `registry.boxes`' or `world.props()` -, at once: (each one's
    class, None for what it cannot tell and for a piece of furniture other than its top - one of FURNITURE's
    not seen clear over it or THIN across, a chair's back or legs -; the one thing each is of, its lowest index
    (`grouped`); the buildings they make [(class, centre, half sizes)])."""
    kinds, centre, half, R, clear = classes.stacked(things)
    names, whole = classes.of(kinds, centre, half, R, clear)
    ones, built = grouped(names, whole, centre, half, R)
    top = (clear > 0.0) & (2.0 * half[:, ::2].min(axis=1) > THIN + 1e-9)
    return ([None if name == 'thing' or (name in FURNITURE + ('bed',) and not t) else name
             for name, t in zip(names, top)],
            ones.tolist(), [(name, tuple(c.tolist()), tuple(h.tolist())) for name, c, h, _m in built])


def named(things):
    """Each of boxes `things` by its class, None for what the robot cannot tell and a piece of furniture
    (`read`)."""
    return read(things)[0]


def buildings(things):
    """[(class, centre, half sizes)] of the buildings boxes `things` make (`read`)."""
    return read(things)[2]
