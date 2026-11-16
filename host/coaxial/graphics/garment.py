"""A figure's garments: sewn ones laid on it, their folds kept, their fall its own; else a tee lofted.

A garment sewn by a cloth simulator (`tools/sim/sewn.py`, thinned beside the figure) hung on
another body in another pose: its folds are kept, its measures taken from the T-posed figure
it is laid on. A top's trunk takes the reach cloth hangs at there (`fall.hung`) - over its
bust, never between or under - and wears on it its own folds; over its armpits it lies on the
figure's shoulders, a corner as far along the way from the armpit's row to the collar's rim as
it stood on its own (`fall.yoke`); a sleeve - a piece of it one side of its middle - goes from
where it hangs onto the figure's arm, round, a girth for a girth. Legs go row by row onto the
figure's: hem on hem, crotch on crotch, waist on waist. What a top then meets - the garment
under its hem, the arm in its sleeve - it stands clear of.

    pieces = garment.dressed(sewn, top, low, skin, j)    # [(names, corners, triangles, lit)], T-posed
"""
import math
from typing import Any

from coaxial.graphics import fall
from coaxial.graphics.fall import EASE_M, ROW_M, SLAB_M, SPOKES, extent, slices

#: A tee's sleeves: their share of the upper arms (the robot's own tee's), their girth and the
#: upper arm's over the forearm's.
SLEEVE, CUFF, UPPER = 0.46, (1.3, 1.2), 1.1
#: The tee's neck's rim off the neck, m: a crew neck's, no collar up the neck (a sewn tee
#: measured: its opening 89 mm a side); a lofted bust's mound's radii across, up and ahead over
#: how far aside of the middle it stands.
NECK_M, MOUND = 0.035, (0.85, 0.9, 0.6)
#: The rows a garment's measure is a mean over; legs stand apart where a row's least |x| is over
#: PART_M; the passes a garment's reach is smoothed in, a corner its neighbours' mean a pass - 12
#: of them some 45 mm at 14 mm a corner: what stands off that is a fold, the garment's own.
SMOOTH, PART_M, FOLD = 7, 0.02, 12
#: A fold's shade: the passes of a corner's neighbours' mean it is told by (4: some 25 mm), the
#: depth under that, m, a corner is lit none at - were it not held to SHADES, the least and the
#: most it is lit: a valley darker, a ridge lighter.
SHADED, SHADE_M, SHADES = 4, 0.015, (0.72, 1.12)
#: The cone a corner clears what it meets in, deg.
CONE = 8.0


def _rows(points, ys, smooth=True):
    """(len(ys), 4): at each height of `ys`, ascending, the least and the most |x| and z of the
    `points` within SLAB_M of it - a height with none its neighbours' -, a mean over SMOOTH rows
    where `smooth`."""
    from coaxial.model.blocks import numpy as np
    x, out = np.abs(points[:, 0]), np.full((len(ys), 4), np.nan)
    for k, y in enumerate(ys):
        s = np.abs(points[:, 1] - y) < SLAB_M
        if s.any():
            out[k] = x[s].min(), x[s].max(), points[s, 2].min(), points[s, 2].max()
    seen = np.isfinite(out[:, 0])
    for c in range(4):
        col = np.interp(ys, ys[seen], out[seen, c])
        out[:, c] = np.convolve(np.pad(col, SMOOTH // 2, mode='edge'), np.ones(SMOOTH) / SMOOTH,
                                'valid') if smooth else col
    return out


def neck(skin, j):
    """(where a figure's neck leaves its shoulders - a collar's rim's height -, its half width,
    front and back where it is narrowest): off its trunk's skin (`skin` its skin's corners) over
    its arms."""
    from coaxial.model.blocks import numpy as np
    bare = skin[np.abs(skin[:, 0]) < j['pit']]
    over = [(y, *got) for y in np.arange(j['arm'][0] + 0.02, j['crown'] - 0.1, 0.01)
            for got in [extent(bare, y, 0.01)] if got]
    _y, wide, front, back = min(over, key=lambda row: row[1])
    return next(y for y, w, _f, _b in over if w < wide + NECK_M), wide, front, back


def _arms(skin, j):
    """(a figure's forearm's girth - a sleeve's measure -, [(names, corners, triangles)] the skin
    of its upper arms where a sleeve it wore had it cut away: a tube an arm)."""
    from coaxial.model.blocks import numpy as np
    pit, (arm_y, arm_z), reach = j['pit'], j['arm'], j['tip'] - j['pit']
    limb = skin[skin[:, 0] > pit]
    girths = [(at - pit, len(s), float(np.hypot(s[:, 1] - arm_y, s[:, 2] - arm_z).max()) if len(s) else 0.0)
              for at, s in slices(limb, 0, pit, j['wrist'], 0.02)]
    girth = float(np.median([g for a, n, g in girths if n >= 12 and a > 0.45 * reach]))
    whole = next(((a, g) for a, n, g in girths if n >= 12 and a > 0.1 * reach), (0.0, girth))
    if not any(n < 12 for a, n, _g in girths if 0.1 * reach < a < whole[0]):
        return girth, []
    at = np.arange(0.0, whole[0] + 0.03, 0.02)
    return girth, _tubes('skin', at, np.linspace(UPPER * girth, whole[1], len(at)), j)


def _tubes(names, at, radii, j):
    """[(names, corners, triangles)]: a tube along each of a figure's arms, `radii` at `at` past
    its armpit."""
    from coaxial.graphics.shapes import lain, loft
    corners, triangles = lain(loft([(a, r, r) for a, r in zip(at, radii)], 0), 'x')[:2]
    return [(names, (corners * (-1.0, 1.0, 1.0) + (j['pit'], *j['arm'])) * (side, 1.0, 1.0), triangles)
            for side in (1.0, -1.0)]


def _hung(points, ys, raw):
    """A top's sleeve as it hangs, its two as one: (the joint it hangs from (x, y), the way out
    along it (x, y), its radius, its length, its middle z, its armpit's height). The cuff is the
    chord from where the rows' most |x| (`raw` at `ys`) leaves the trunk's side - the knee
    furthest off the line from its middle height to its peak - up to that peak; the arm square to
    the chord through its middle, the joint on it over the knee."""
    from coaxial.model.blocks import numpy as np
    half = len(ys) // 2
    peak = half + int(np.argmax(raw[half:, 1]))
    a, b = np.array([raw[half, 1], ys[half]]), np.array([raw[peak, 1], ys[peak]])
    way = (b - a) / np.linalg.norm(b - a)
    knee = half + int(np.argmax(way[0] * (ys[half:peak + 1] - a[1]) - way[1] * (raw[half:peak + 1, 1] - a[0])))
    c = np.array([raw[knee, 1], ys[knee]])
    chord = b - c
    out = np.array([chord[1], -chord[0]]) / np.linalg.norm(chord)
    length = ((b + c) / 2.0 - c)[0] / out[0]
    on = (np.abs(points[:, 0]) > c[0]) & (points[:, 1] > c[1]) & (points[:, 1] < b[1])
    return ((b + c) / 2.0 - out * length, out, float(np.linalg.norm(chord)) / 2.0, float(length),
            float(points[on, 2].min() + points[on, 2].max()) / 2.0, float(c[1]))


def _clear(points, body, axis):
    """`points` moved out to stand EASE_M clear of the `body` corners they lie over: out from
    the nearest of an upright `axis` (its foot's and its head's height, its z), past the body's
    furthest corner within CONE of the same way out, off the axis's same height within SLAB_M."""
    from coaxial.model.blocks import numpy as np

    def out(p):
        foot = np.stack([np.zeros(len(p)), np.clip(p[:, 1], axis[0], axis[1]), np.full(len(p), axis[2])], 1)
        far = np.linalg.norm(p - foot, axis=1)
        return foot, (p - foot) / np.maximum(far, 1e-9)[:, None], far
    foot, way, far = out(points)
    under, toward, reach = out(body)
    for k in range(0, len(points), 512):
        near = (way[k:k + 512] @ toward.T > math.cos(math.radians(CONE))) & (
            np.abs(foot[k:k + 512, 1, None] - under[None, :, 1]) < SLAB_M)
        far[k:k + 512] = np.maximum(far[k:k + 512], np.where(near, reach[None], 0.0).max(axis=1, initial=0.0)
                                    + EASE_M * near.any(axis=1))
    return foot + way * far[:, None]


def _mean(values, triangles, passes, weight):
    """`values`, a corner's each, each a mean of its neighbours' over the mesh, `passes` times;
    `weight` what a corner counts for in its neighbours' means."""
    from coaxial.model.blocks import numpy as np
    out = values.reshape(len(values), -1) * 1.0
    for _ in range(passes):
        total, share = np.zeros_like(out), np.zeros(len(out))
        for a, b in ((0, 1), (1, 2), (2, 0), (1, 0), (2, 1), (0, 2)):
            np.add.at(total, triangles[:, a], out[triangles[:, b]] * weight[triangles[:, b], None])
            np.add.at(share, triangles[:, a], weight[triangles[:, b]])
        out = np.where(share[:, None] > 0.0, total / np.maximum(share, 1e-9)[:, None], out)
    return out.reshape(values.shape)


def _shaded(corners, triangles):
    """How lit each corner of a laid garment is by its folds, 1 where it lies flat: one under its
    neighbours' mean (`_mean`, SHADED passes) along the cloth's way out - a fold's valley - darker,
    none less than SHADES' least at SHADE_M deep, a ridge lighter. A fold drawn in dots is its
    shade."""
    from coaxial.model.blocks import numpy as np
    a, b, c = (corners[triangles[:, k]] for k in range(3))
    out = np.zeros_like(corners)
    for k in range(3):
        np.add.at(out, triangles[:, k], np.cross(b - a, c - a))
    out /= np.maximum(np.linalg.norm(out, axis=1, keepdims=True), 1e-12)
    # A garment's way out is away from its upright axis: one sewn inside out is turned.
    off = corners[:, [0, 2]] - corners[:, [0, 2]].mean(axis=0)
    out *= 1.0 if (out[:, [0, 2]] * off).sum() > 0.0 else -1.0
    every = np.ones(len(corners))
    deep = ((_mean(corners, triangles, SHADED, every) - corners) * out).sum(axis=1)
    # A round limb's corners all stand over their neighbours' mean: a fold is what a corner
    # stands under or over that of the cloth about it.
    return np.clip(1.0 - (deep - _mean(deep, triangles, FOLD, every)) / SHADE_M, *SHADES)


def _legged(corners, body, j):
    """A sewn garment with a leg each laid on a figure's `body` there - what it wears there,
    else its skin -, EASE_M clear, row by row: hem on hem, crotch on crotch, waist on waist."""
    from coaxial.model.blocks import numpy as np
    ys = np.arange(corners[:, 1].min(), corners[:, 1].max() + ROW_M, ROW_M)
    into = np.arange(body[:, 1].min(), body[:, 1].max() + ROW_M, ROW_M)
    rows = _rows(body, into) + (-EASE_M, EASE_M, -EASE_M, EASE_M)
    crotch = ys[int(np.argmax(_rows(corners, ys, False)[:, 0] < SLAB_M / 2.0))]
    y = np.interp(corners[:, 1], [ys[0], crotch, ys[-1]], [into[0], j['crotch'], into[-1]])
    a = [np.interp(corners[:, 1], ys, col) for col in _rows(corners, ys).T]
    b = [np.interp(y, into, col) for col in rows.T]
    x = np.maximum(b[0] + (np.abs(corners[:, 0]) - a[0]) * (b[1] - b[0]) / (a[1] - a[0]), 0.0)
    return np.stack([np.sign(corners[:, 0]) * x, y, b[2] + (corners[:, 2] - a[2]) * (b[3] - b[2]) / (a[3] - a[2])], 1)


def _topped(corners, triangles, piece, trunk, low, skin, girth, j):
    """A sewn top laid on a figure: its trunk at the reach cloth hangs at there (`fall.hung`),
    its own folds on it; over its armpit on the figure's shoulders, a corner as far along its
    spoke's line from the armpit's row to the collar's rim as it stood on its own (`fall.yoke`),
    that rim NECK_M off the figure's neck; its sleeves - its pieces (`piece`, a triangle's each)
    one side of its middle out to its side - from where they hang (`_hung`) along the figure's
    arms; clear of what is worn under it."""
    from coaxial.model.blocks import numpy as np
    pit, (arm_y, arm_z), under = j['pit'], j['arm'], j['under']
    ys = np.arange(corners[:, 1].min(), corners[:, 1].max() + ROW_M, ROW_M)
    joint, out, radius, length, middle, armpit = _hung(corners, ys, _rows(corners, ys, False))
    off = np.stack([np.abs(corners[:, 0]) - joint[0], corners[:, 1] - joint[1]], 1)
    s, c1, c2 = off @ out, off @ (-out[1], out[0]), corners[:, 2] - middle
    aside, wide = np.zeros(len(corners), bool), np.abs(corners[:, 0])
    for ids in (np.unique(triangles[piece == p]) for p in np.unique(piece)):
        aside[ids] |= wide[ids].min() > PART_M and wide[ids].max() > 0.9 * wide.max()
    # A seam's corners between the two (told by how near the sleeve's axis a corner stood, the
    # trunk's side was laid on the arm). Round by its outer half: hung, it lay flat, 81 by 52 mm.
    sleeve, cuff = _mean(aside * 1.0, triangles, 2, np.ones(len(corners))), aside & (s > length / 2.0)
    c1, c2 = ((c - (c[cuff].min() + c[cuff].max()) / 2.0) * 2.0 * radius / np.ptp(c[cuff]) for c in (c1, c2))
    # The trunk: where cloth hangs on the figure, and on that its folds - what stands off its
    # own reach smoothed over its mesh, its sleeves' left out of the mean.
    o = _rows(corners[~aside], ys[ys <= armpit])[-1]
    base = neck(skin, j)[0]
    into, z0, reach = fall.hung(trunk, low, skin[np.abs(skin[:, 0]) < pit], j, base + 0.03)
    turn, far = fall.spokes(corners, (o[2] + o[3]) / 2.0)
    fold, up = far - _mean(far, triangles, FOLD, 1.0 - sleeve), np.interp(corners[:, 1], [ys[0], armpit], [into[0], under])
    # The yoke: on its own, a corner's share of its spoke's way from the armpit's row - the reach
    # there - up and in to its collar's rim, the least reach over the armpit and that's height.
    spoke, start, rim, top = turn.astype(int) % SPOKES, np.zeros(SPOKES), np.full(SPOKES, np.inf), np.zeros(SPOKES)
    row, high = ~aside & (np.abs(corners[:, 1] - armpit) < SLAB_M), ~aside & (corners[:, 1] > armpit)
    np.maximum.at(start, spoke[row], far[row])
    np.minimum.at(rim, spoke[high], far[high])
    edge = high & (far < rim[spoke] + ROW_M)
    np.maximum.at(top, spoke[edge], corners[edge, 1])
    start, rim, top = (np.interp(np.arange(SPOKES), seen, told[seen], period=SPOKES)
                       for told in (start, np.where(np.isfinite(rim), rim, 0.0), top) for seen in [np.flatnonzero(told)])
    gain = np.clip(corners[:, 1] - armpit, 0.0, None) + np.clip(start[spoke] - far, 0.0, None)
    left = np.clip(top[spoke] - corners[:, 1], 0.0, None) + np.clip(far - rim[spoke], 0.0, None)
    share = gain / np.maximum(gain + left, 1e-9)
    line = fall.along(fall.yoke(into, reach, under, reach[into >= base].min(axis=0) + NECK_M), turn, share)
    over = corners[:, 1] > armpit
    far = np.where(over, line[0] + fold * (1.0 - share), fall.at(into, reach, up, turn) + fold)
    at = np.stack([far * np.abs(np.sin(turn * 2.0 * math.pi / SPOKES)), np.where(over, line[1], up),
                   z0 + far * np.cos(turn * 2.0 * math.pi / SPOKES)], 1)
    # The sleeves: along the arm to SLEEVE of the upper arm, their girth the forearm's by CUFF.
    kr = girth * (CUFF[0] + (CUFF[1] - CUFF[0]) * np.clip(s / length, 0.0, 1.0)) / radius
    kr = np.maximum(kr, (UPPER * girth + EASE_M) / np.maximum(np.hypot(c1, c2), 1e-9))    # clear of the arm in it
    arm = np.stack([pit + s * SLEEVE * (j['elbow'] - pit) / length, arm_y + kr * c1, arm_z + kr * c2], 1)
    at += sleeve[:, None] * (arm - at)
    met = low[low[:, 1] > into[0] - 2.0 * SLAB_M]
    at = _clear(at, np.abs(met) * (1.0, 0.0, 0.0) + met * (0.0, 1.0, 1.0), (into[0], under, z0))
    at[:, 0] *= np.sign(corners[:, 0])
    return at


def _lofted(trunk, low, skin, girth, j):
    """[(names, corners, triangles)]: a straight tee lofted on a figure with none sewn - its
    rings about the reach cloth hangs at (`fall.hung`) up to where its shoulders come within
    NECK_M of its neck, open there; a mound a side at its bust, its sleeves SLEEVE of the upper
    arms."""
    from coaxial.graphics.shapes import ellipsoid, loft
    from coaxial.model.blocks import numpy as np
    base, wide, _front, _back = neck(skin, j)
    into, z0, reach = fall.hung(trunk, low, skin[np.abs(skin[:, 0]) < j['pit']], j, base + 0.03)
    boxes = [fall.box(row, z0) for row in reach]
    rings = [(y, w, (f - b) / 2.0, (f + b) / 2.0) for y, (w, b, f) in list(zip(into, boxes))[::2] if w > wide + NECK_M]
    out = [('tee', *loft(rings, 0, poles=(into[0] - 0.004, rings[-1][0] - 0.05))[:2])]
    apex = max(range(int(np.searchsorted(into, j['arm'][0]))), key=lambda k: boxes[k][2])
    ahead = trunk[(np.abs(trunk[:, 1] - into[apex]) < 0.03) & (trunk[:, 0] > 0.02)]
    aside = float(ahead[np.argmax(ahead[:, 2]), 0])
    for side in (1.0, -1.0):
        out.append(('tee', *ellipsoid((side * aside, into[apex], boxes[apex][2] + 0.004 - MOUND[2] * aside),
                                      tuple(r * aside for r in MOUND), 0)[:2]))
    at = np.arange(-0.03, SLEEVE * (j['elbow'] - j['pit']) + 0.01, 0.02)
    return out + _tubes('tee', at, np.linspace(CUFF[0] * girth, CUFF[1] * girth, len(at)), j)


def dressed(sewn, top, low, skin, j):
    """[(names, corners, triangles, lit)] a T-posed figure wears in the place of what it wore
    over its trunk (`top`, that's corners; `low` those of what it wears under it, `skin` its
    skin's, `j` its joints): the garments `sewn` [(corners, triangles, piece)] laid on it - one
    with a leg each its 'jeans', laid first, one without its 'tee' over them -, `lit` how its folds
    shade each corner (`_shaded`); a tee lofted with none sewn; and its upper arms' skin where a
    longer sleeve had it cut away - `lit` None, lit as all. `j` gains 'under', its armpit's
    height: its arms' axis less a sleeve's radius."""
    from coaxial.model.blocks import numpy as np
    trunk = top[np.abs(top[:, 0]) < j['pit']]
    girth, bare = _arms(skin, j)
    j['under'] = j['arm'][0] - CUFF[0] * girth
    out: list[tuple[str, Any, Any, Any]] = [(*laid, None) for laid in bare]
    for corners, triangles, piece in sorted(sewn, key=lambda g: float(g[0][:, 1].max())):
        ys = np.arange(corners[:, 1].min(), corners[:, 1].max(), ROW_M)
        if _rows(corners, ys, False)[len(ys) // 4, 0] > PART_M:
            low = _legged(corners, low if len(low) else skin[skin[:, 1] < j['hip']], j)
            out.append(('jeans', low, triangles, _shaded(low, triangles)))
        else:
            at = _topped(corners, triangles, piece, trunk, low, skin, girth, j)
            out.append(('tee', at, triangles, _shaded(at, triangles)))
    if not any(names == 'tee' for names, *_rest in out):
        out += [(*laid, None) for laid in _lofted(trunk, low, skin, girth, j)]
    return out
