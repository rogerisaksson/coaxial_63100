"""The robot's foot printed in profile: a keel sized on its carbon plate, round the roll pin, on soles.

    parts = foot.parts()          # {'foot', 'plate', 'soles': tris}, the knee's frame, the leg straight

Two prints, the foot split down its middle and each half laid on that face, its profile across the way it is
printed along (x), KEEL_X either side of the leg's middle: a keel on the plate from the heel to the toes - as deep
as its push-off asks on the plate (`depth`) -, under the ankle two pillars rising to a bridge under the cross -
clear of its pitch pin however far the foot rolls (`drives.TRAVEL`), a hole through the block between them - and two
cheeks up round the roll pin's ends (`skeleton.CROSS`), the pin's bore a half round in each half. Between the
halves, behind the ankle, the heel's steel horn out to the shaft the rods' eyes ride (`leg_clamped`). Under the keel
the arched carbon plate along the sole's tube (`plate`), bonded to it, under it the soft soles, the heel's and the
ball's.
"""
import functools
import math

from tools.render import elbow_mount as em

#: The keel's half width across and its least depth, the cheeks' thickness along the pin and their bosses' reach over
#: it, the air the bridge keeps off the cross, the pillars' thickness under the cheeks and the bridge's own, the hole
#: between them through the block, m.
KEEL_X, KEEL_M, CHEEK_M, BOSS_M, AIR_M = 0.012, 0.006, 0.008, 0.012, 0.002
PILLAR_M, BRIDGE_M = 0.010, 0.008
#: The plate's thickness, its toes' rise at their tip, how far each soft sole reaches past the sole's tube's corner
#: toward the arch, m; the soles' soft polymer (TPU).
PLATE_T, TOE_UP, ARCH_IN = 0.003, 0.004, 0.02
TPU = (70, 96, 108)


def sole_run():
    """[(z, y)]: the sole's tube's corners along the foot, heel first, m in the knee's frame (`skeleton.HUNG`)."""
    from machine.body import skeleton
    from machine.body.figure import ANKLE_H, SHANK
    return sorted((float(q[2]), float(q[1]) - SHANK) for q in skeleton.HUNG['foot'][0][0] if q[1] < -0.5 * ANKLE_H)


def plate(zs):
    """The carbon plate's top at `zs` along the foot, m up the knee's frame: under the sole's tube, its toes risen
    TOE_UP."""
    import numpy as np
    from machine.body import skeleton
    from machine.body.figure import BALL, TOE_M
    run, radii, tip = sole_run(), skeleton.HUNG['foot'][1], BALL + TOE_M
    zs = np.asarray(zs, float)
    y = np.interp(zs, [z for z, _y in run], [y for _z, y in run]) - float(radii[0] if isinstance(radii, tuple)
                                                                         else radii)
    return y + np.where(zs > run[-1][0], TOE_UP * ((zs - run[-1][0]) / (tip - run[-1][0])) ** 2, 0.0)


def _sole(zs, heel, tip, ball, half):
    """A sole's half widths along it, m: a foot's shape - narrow at the heel, widest at the ball, round at both ends."""
    import numpy as np
    base = np.interp(zs, [heel, 0.0, 0.5 * ball, ball, 0.5 * (ball + tip), tip],
                     [0.62, 0.66, 0.7, 1.0, 0.88, 0.5]) * half
    return base * np.sqrt(np.clip((zs - heel) / 0.014, 0.0, 1.0) * np.clip((tip - zs) / 0.024, 0.0, 1.0))


def _loft(zs, width, y_lo, y_hi, colour, round_m=0.0015, k=3):
    """A body lofted along z through rounded cross-sections, each `width` either side and from `y_lo` up to `y_hi`."""
    import numpy as np
    rings = []
    for z, w, lo, hi in zip(zs, width, y_lo, y_hi):
        r = max(0.0, min(round_m, 0.45 * (hi - lo), 0.95 * w))
        pts = []
        for cx, cy, a0 in ((w - r, hi - r, 0.0), (r - w, hi - r, 0.5 * math.pi), (r - w, lo + r, math.pi),
                           (w - r, lo + r, 1.5 * math.pi)):
            pts += [(cx + r * math.cos(a0 + 0.5 * math.pi * i / k), cy + r * math.sin(a0 + 0.5 * math.pi * i / k))
                    for i in range(k + 1)]
        rings.append(np.array([(x, y, z) for x, y in pts]))
    tris, n = [], len(rings[0])
    for a, b in zip(rings[:-1], rings[1:]):
        for i in range(n):
            j = (i + 1) % n
            tris += [(np.array((a[i], a[j], b[j])), colour), (np.array((a[i], b[j], b[i])), colour)]
    for cap in (rings[0], rings[-1]):
        c = cap.mean(axis=0)
        tris += [(np.array((c, cap[i], cap[(i + 1) % n])), colour) for i in range(n)]
    return tris


def _ears(poly):
    """[(i, j, k)]: triangles filling a simple polygon (n, 2), an ear at a time."""
    import numpy as np
    left, out = list(range(len(poly))), []
    turn = np.sign(np.sum(poly[:, 0] * np.roll(poly[:, 1], -1) - np.roll(poly[:, 0], -1) * poly[:, 1]))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    while len(left) > 3:
        for n in range(len(left)):
            i, j, k = left[n - 1], left[n], left[(n + 1) % len(left)]
            a, b, c = poly[i], poly[j], poly[k]
            if turn * cross(a, b, c) <= 1e-14 or any(
                    turn * cross(a, b, poly[q]) > 0 and turn * cross(b, c, poly[q]) > 0 and turn * cross(c, a, poly[q]) > 0
                    for q in left if q not in (i, j, k)):
                continue
            out.append((i, j, k))
            left.pop(n)
            break
        else:
            break
    return out + ([tuple(left)] if len(left) == 3 else [])


def _slab(outline, x0, x1, colour):
    """[(tri, colour)]: the profile `outline` [(y, z)] - simple, not convex - from x0 to x1 across."""
    import numpy as np
    p = np.asarray(outline, float)
    lo, hi = np.c_[np.full(len(p), x0), p], np.c_[np.full(len(p), x1), p]
    tris = []
    for i in range(len(p)):
        j = (i + 1) % len(p)
        tris += [(np.array((lo[i], lo[j], hi[j])), colour), (np.array((lo[i], hi[j], hi[i])), colour)]
    for i, j, k in _ears(p):
        tris += [(np.array((lo[i], lo[k], lo[j])), colour), (np.array((hi[i], hi[j], hi[k])), colour)]
    return tris


def depth(zs):
    """The keel's depth at `zs` along the foot, m: the least that holds on its carbon plate - the plate its tension
    flange, the keel its compression - the drive's push (`sized.torque`, at the margin) on the ball, then on the toes'
    tip ahead of the ankle and on the heel behind it, each material at its allowable (`structural`); KEEL_M at least."""
    import numpy as np
    from machine.body import structural
    from machine.body.figure import BALL, HEEL, SOLE_HALF, TOE_M
    from tools.render import sized
    tip, push, zs = BALL + TOE_M, 1e-3 * sized.torque('ankle'), np.asarray(zs, float)
    moment = np.maximum.reduce([push / BALL * np.clip(BALL - zs, 0.0, None) * (zs > 0.0),
                                push / tip * np.clip(tip - zs, 0.0, None) * (zs > 0.0),
                                push / HEEL * np.clip(zs + HEEL, 0.0, None) * (zs < 0.0)])
    b, t, n = 2.0 * KEEL_X, PLATE_T, structural.CARBON[0] / structural.PAHT_CF[0]
    w = 2.0 * _sole(zs, -HEEL, tip, BALL, SOLE_HALF)
    hs = np.arange(KEEL_M, 0.05, 0.0005)[:, None]       # depths tried, half a millimetre apart
    a1, y1, a2, y2 = b * hs, t + 0.5 * hs, n * w * t, 0.5 * t
    mid = (a1 * y1 + a2 * y2) / (a1 + a2)               # the section's neutral axis, the plate n times as wide
    inertia = b * hs ** 3 / 12.0 + a1 * (y1 - mid) ** 2 + n * w * t ** 3 / 12.0 + a2 * (y2 - mid) ** 2
    holds = ((moment * (t + hs - mid) / inertia <= 1e6 * structural.PAHT_CF[2])
             & (n * moment * mid / inertia <= 1e6 * structural.CARBON[2]))
    return np.where(holds.any(axis=0), hs[np.argmax(holds, axis=0), 0], hs[-1, 0])


def roof(zs):
    """The keel's top at `zs` along the foot, m up the knee's frame: its depth (`depth`) over the plate (`plate`)."""
    return plate(zs) + depth(zs)


@functools.lru_cache(maxsize=None)
def parts():
    """{'foot', 'plate', 'soles': tris}: the printed foot, the plate and the soles, the knee's frame with the leg
    straight, the ankle SHANK under the knee."""
    import numpy as np
    from machine.body import drives, skeleton
    from machine.body.figure import ANKLE_H, BALL, HEEL, SHANK, SOLE_HALF, TOE_M
    y_a, tip = -SHANK, BALL + TOE_M
    _stage, pin_r, half = skeleton.CROSS
    roll = math.radians(max(abs(t) for t in drives.TRAVEL['ankle_roll']))
    bridge = y_a - (KEEL_X * math.sin(roll) + pin_r * math.cos(roll) + AIR_M)   # under the cross however it rolls
    run = sole_run()
    z_keel = np.linspace(-HEEL, tip, 48)
    floor = plate(z_keel)
    ridge = floor + depth(z_keel)
    keel = [(floor[i], z) for i, z in enumerate(z_keel)] + [(ridge[i], z) for i, z in enumerate(z_keel)][::-1]
    sunk = float(np.interp(half, z_keel, ridge)) - 0.002, float(np.interp(-half, z_keel, ridge)) - 0.002
    inner = half - PILLAR_M                              # the pillars under the cheeks, the hole between them open
    block = [(sunk[0], half), (y_a + BOSS_M, half), (y_a + BOSS_M, half - CHEEK_M), (bridge, half - CHEEK_M),
             (bridge, CHEEK_M - half), (y_a + BOSS_M, CHEEK_M - half), (y_a + BOSS_M, -half),
             (sunk[1], -half), (sunk[1], -inner), (bridge - BRIDGE_M, -inner),
             (bridge - BRIDGE_M, inner), (sunk[0], inner)]
    out = {'foot': _slab(keel, -KEEL_X, KEEL_X, em.FRAME) + _slab(block, -KEEL_X, KEEL_X, em.FRAME)}
    width = _sole(z_keel, -HEEL, tip, BALL, SOLE_HALF)
    out['plate'] = _loft(z_keel, width, floor - PLATE_T, floor, em.CARBON)
    soles, ground = [], y_a - ANKLE_H
    for z0, z1 in ((-HEEL, run[0][0] + ARCH_IN), (run[-2][0] - ARCH_IN, tip - 0.004)):   # the heel's, the ball's
        z = np.linspace(z0, z1, 24)
        soles += _loft(z, np.maximum(np.interp(z, z_keel, width) - 0.0015, 0.0), np.full(len(z), ground),
                       np.interp(z, z_keel, floor - PLATE_T) + 0.0005, TPU)
    out['soles'] = soles
    return out
