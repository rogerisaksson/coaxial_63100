"""Where cloth hangs on a figure: its reach a way out a row, taut over hollows; a yoke's lines.

A top's cloth is told about its figure's upright axis: a way out (a spoke) a column, a height
a row. `hung` is the reach it hangs at from its hem to its neck - clear of what it lies on,
taut around a row, falling in from what stands out above it, half taut down a hollow. Over
the armpits a row tells too little - a shoulder's top is one height and many reaches -: there
cloth lies along each spoke's line from the armpit's row up and in over the shoulder to the
collar's rim (`yoke`), a place on it a share of that line's length (`along`). Laid by rows
alone, a top's shoulders stood level at its collar's height: a plate about the neck.

    ys, z0, reach = fall.hung(trunk, low, bare, j, top)     # rows hem to `top`, (n, SPOKES)
    far = fall.at(ys, reach, heights, turns)                # between its four nearest
    lines = fall.yoke(ys, reach, under, rim)                # (reach, height), (SPOKES, SHARES) each
    far, up = fall.along(lines, turns, shares)
"""
import math

#: The share of a bust kept (the user's: a little smaller); a cloth's ease off what it lies on, m.
BUST, EASE_M = 0.85, 0.008
#: A figure's rows: their pitch and half height, m.
ROW_M, SLAB_M = 0.01, 0.015
#: A top's fall: the ways out about its trunk's axis it is told in; how far cloth hung free falls
#: in a metre down (a sewn tee's front under its bust: 34 mm in 320); how far it bridges a hollow,
#: of the way from lying in it to drawn taut over it (a sewn tee lay 2 mm into the hollow between
#: its bust and 54 mm into its waist's behind: half, between a tube and a wrap); the places told
#: along a yoke's line. Down a hollow a straight cut's seam is a line, SEAM of the way to taut:
#: half taut there, the tee stood a tunic - 132 mm aside at its chest and 231 at its hem, the
#: way out between them bent, and over the seat a bustle behind.
SPOKES, FALL, TAUT, SHARES, SEAM = 48, 0.1, 0.5, 33, 1.0


def slices(points, axis, low, high, step=0.005):
    """[(at, the points within a slice `step` thick)] along `axis` from `low` to `high`."""
    from coaxial.model.blocks import numpy as np
    return [(at, points[np.abs(points[:, axis] - at) < step / 2.0]) for at in np.arange(low, high, step)]


def extent(points, y, half=0.02):
    """(half width, front, back) of `points` within `half` of the height `y`; None where none."""
    from coaxial.model.blocks import numpy as np
    s = points[np.abs(points[:, 1] - y) < half]
    return (float(np.abs(s[:, 0]).max()), float(s[:, 2].max()), float(s[:, 2].min())) if len(s) else None


def spokes(points, z0):
    """(the way out, in SPOKES a turn from ahead toward the left, and the reach) of `points` from
    the upright axis x = 0, z = `z0`."""
    from coaxial.model.blocks import numpy as np
    return (np.arctan2(points[:, 0], points[:, 2] - z0) / (2.0 * math.pi) * SPOKES % SPOKES,
            np.hypot(points[:, 0], points[:, 2] - z0))


def around(row):
    """A row's reach (SPOKES) with each way out no corner stands in (0) on the chord between
    its neighbours': a figure's corners stand 30-40 mm apart, and told between two by their
    turn one row's reach behind stood 0.142 where the rows about it had 0.107 - hung from down
    the trunk, the back's cloth stood 39 mm off the back. A row of none is left none."""
    from coaxial.model.blocks import numpy as np
    seen, step = np.flatnonzero(row), 2.0 * math.pi / SPOKES
    if not len(seen):
        return row
    b = np.searchsorted(seen, np.arange(SPOKES)) % len(seen)
    a = (b - 1) % len(seen)
    whole, part = ((seen[b] - seen[a] - 1) % SPOKES + 1) * step, (np.arange(SPOKES) - seen[a]) % SPOKES * step
    ra, rb = row[seen[a]], row[seen[b]]
    return np.where(whole < 0.9 * math.pi, ra * rb * np.sin(whole) / np.maximum(
        ra * np.sin(part) + rb * np.sin(whole - part), 1e-9), ra + (rb - ra) * part / whole)


def hung(trunk, low, bare, j, top):
    """(heights ascending, the axis's z, reach (n, SPOKES)) a top hangs at on a figure from its
    hem to `top`, a way out each about its trunk's upright axis: EASE_M clear of what it wears
    under (`low`) and, over that, of its trunk - what it wore there (`trunk`) and its skin
    (`bare`), its bust BUST of it; over its arms' height of its skin alone where that is seen
    around: what it wore has a collar. Taut around a row - over its bust, never between -; to
    its arms' height hung from above where they stand back, falling in FALL, and half taut down
    a hollow, from its bust to its hips: a straight cut, no wrap."""
    from coaxial.model.blocks import numpy as np
    arm, (_wide, chest, back) = j['arm'][0], extent(trunk, j['arm'][0]) or (0.0, 0.0, 0.0)

    def kept(points):
        out = points * 1.0
        out[:, 2] = np.where(out[:, 2] > chest, chest + BUST * (out[:, 2] - chest), out[:, 2])
        return out
    worn, skin = kept(trunk), kept(bare)
    ys, z0 = np.arange(float(trunk[:, 1].min()), top, ROW_M), (chest + back) / 2.0
    reach = np.zeros((len(ys), SPOKES))
    for k, y in enumerate(ys):
        for points in (low, np.vstack([skin, worn])) if y < arm else (skin, worn):
            turn, far = spokes(points[np.abs(points[:, 1] - y) < SLAB_M], z0)
            np.maximum.at(reach[k], turn.astype(int), far)
            if (reach[k] > 0.0).mean() > 0.75:
                break
        reach[k] = around(reach[k])
    there = np.flatnonzero(reach.any(axis=1))           # a row no corner stands in: between its neighbours'
    reach = np.stack([np.interp(ys, ys[there], reach[there, s]) for s in range(SPOKES)], 1) + EASE_M
    # Taut around: a row's hull - its support the most any spoke reaches a way, its reach the
    # nearest of those supports.
    turn = (np.arange(SPOKES) + 0.5) * 2.0 * math.pi / SPOKES
    facing = np.cos(turn[:, None] - turn[None])
    held = (reach[:, :, None] * facing[None]).max(axis=1)
    reach += TAUT * (np.where(facing[None] > 0.1, held[:, None, :] / np.maximum(facing[None], 0.1),
                              np.inf).min(axis=2) - reach)
    rows = int(np.searchsorted(ys, arm))
    for s in range(SPOKES):                     # to its arms' height: hung from above, taut down
        f, hull = reach[:rows, s], []
        for k in range(rows - 2, -1, -1):
            f[k] = max(f[k], f[k + 1] - FALL * ROW_M)
        for p in zip(ys, f):
            while len(hull) > 1 and ((hull[-1][0] - hull[-2][0]) * (p[1] - hull[-2][1])
                                     >= (hull[-1][1] - hull[-2][1]) * (p[0] - hull[-2][0])):
                hull.pop()
            hull.append(p)
        reach[:rows, s] = f + SEAM * (np.interp(ys[:rows], [p[0] for p in hull], [p[1] for p in hull]) - f)
    return ys, z0, reach


def at(ys, reach, y, turn):
    """`reach` (rows at `ys`, a spoke a column) at the heights `y` and the ways out `turn`: between
    its four nearest."""
    from coaxial.model.blocks import numpy as np
    v, u = np.clip((y - ys[0]) / ROW_M, 0.0, len(ys) - 1.001), (turn - 0.5) % SPOKES
    i, k = v.astype(int), u.astype(int)
    low, high = (reach[row, k] + (u - k) * (reach[row, (k + 1) % SPOKES] - reach[row, k]) for row in (i, i + 1))
    return low + (v - i) * (high - low)


def box(row, z0):
    """(half width, back, front) of a row's reach about the axis at `z0`."""
    from coaxial.model.blocks import numpy as np
    turn = (np.arange(SPOKES) + 0.5) * 2.0 * math.pi / SPOKES
    ahead = z0 + row * np.cos(turn)
    return float((row * np.abs(np.sin(turn))).max()), float(ahead.min()), float(ahead.max())


def yoke(ys, reach, under, rim):
    """(reach, height), (SPOKES, SHARES) each: along each spoke's line over `reach` (`hung`'s)
    from the height `under` up and in to where its reach comes down to `rim` (a spoke's each) -
    a collar's -, SHARES places at equal shares of the line's length: the height gained and the
    reach lost or won, summed."""
    from coaxial.model.blocks import numpy as np
    first, share = int(np.searchsorted(ys, under)), np.linspace(0.0, 1.0, SHARES)
    start, out = at(ys, reach, np.full(SPOKES, under), np.arange(SPOKES) + 0.5), np.zeros((2, SPOKES, SHARES))
    for s in range(SPOKES):
        r, y = np.concatenate([[start[s]], reach[first:, s]]), np.concatenate([[under], ys[first:]])
        end = next((k for k in range(1, len(r)) if r[k] <= rim[s]), len(r) - 1)
        part = (r[end - 1] - rim[s]) / (r[end - 1] - r[end]) if r[end] < rim[s] < r[end - 1] else 1.0
        r, y = r[:end + 1], y[:end + 1]
        r[end], y[end] = r[end - 1] + part * (r[end] - r[end - 1]), y[end - 1] + part * (y[end] - y[end - 1])
        length = np.concatenate([[0.0], np.cumsum(np.abs(np.diff(y)) + np.abs(np.diff(r)))])
        out[:, s] = np.interp(share * length[-1], length, r), np.interp(share * length[-1], length, y)
    return out


def along(lines, turn, share):
    """(reach, height) at the shares `share` (0 at the armpit's row, 1 at the collar's rim) of the
    lines of `yoke` at the ways out `turn`: between the four nearest."""
    from coaxial.model.blocks import numpy as np
    u, v = (turn - 0.5) % SPOKES, np.clip(share, 0.0, 1.0) * (SHARES - 1.001)
    k, i = u.astype(int), v.astype(int)
    low, high = (lines[:, k, q] + (u - k) * (lines[:, (k + 1) % SPOKES, q] - lines[:, k, q]) for q in (i, i + 1))
    return low + (v - i) * (high - low)
